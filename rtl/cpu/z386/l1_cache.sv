// Physically indexed, physically tagged L1 cache for z386 0.3.
//
// CPU-side contract:
//   * cpu_addr is a physical byte address.
//   * A cache-hit read accepted in cycle N returns cpu_resp_valid in N+1.
//   * A write accepted in cycle N is posted to the write-through store queue.
//   * Miss/refill and uncached accesses use the memory-side burst interface.
//
// This deliberately avoids the old VIPT preread/finalize split.  The paging
// unit owns translation and only sends physical requests to this module.
module l1_cache #(
    // Four ways, 16 bytes per line. SET_BITS=8 gives a 16KB data cache
    // (256 sets x 4 ways x 16 B); =7 was 8KB.
    parameter integer SET_BITS = 8,
    parameter PROTECT_UMA_ROM = 0
) (
    input         clk,
    input         clk_en,      // clock enable: the core advances only when high
    input         reset,

    // CPU side — physical address request/response.
    input  [31:0] cpu_addr,    // physical byte address; the cache indexes off the
                               // page-offset bits [11:2] (translation-invariant,
                               // so available without the TLB result) and tags off
                               // [31:12], exactly like l1_icache
    input  [31:0] cpu_din,
    output [31:0] cpu_dout,
    input   [3:0] cpu_be,
    input         cpu_valid,
    input         cpu_write,
    input         cpu_lock,    // locked cycle: reads bypass the cache, writes never coalesce
    input         cpu_more,    // another cycle of the same sequence follows
    output        cpu_ready,
    output        cpu_resp_valid,
    output        store_pending, // a queued or presented write has not left yet

    // Memory side.
    output [31:0] mem_addr,
    output [31:0] mem_din,
    input  [31:0] mem_dout,
    output  [3:0] mem_be,
    output  [7:0] mem_burstcount,
    input         mem_busy,
    output        mem_valid,
    output        mem_write,
    output        mem_lock,
    output        mem_more,    // passed on for writes and bypassed reads, the ones that reach the bus
    input         mem_ready,
    input         mem_resp_valid,

    // Physical-address snoop.  The first implementation invalidates a whole
    // set; this is conservative and keeps snoop matching off the read hit path.
    input  [31:0] snoop_addr,
    input         snoop_valid,

    input         cache_enable,
    input   [1:0] ram_size,     // fitted DRAM: 0 2 MB, 1 4 MB, 2 6 MB, 3 8 MB
    input         pg            // CR0.PG: with 8 MB, a read of 600000-7FFFFF with paging off is the ROM disk
);

localparam integer WORD_OFFSET_BITS = 2;
localparam integer BYTE_OFFSET_BITS = 2;
localparam integer LINE_OFFSET_BITS = WORD_OFFSET_BITS + BYTE_OFFSET_BITS;
localparam integer NUM_SETS = 1 << SET_BITS;
localparam integer BRAM_ADDR_BITS = SET_BITS + WORD_OFFSET_BITS;
localparam integer TAG_BITS = 25 - LINE_OFFSET_BITS - SET_BITS;
localparam integer SET_LSB = LINE_OFFSET_BITS;
localparam integer SET_MSB = SET_LSB + SET_BITS - 1;
localparam integer TAG_LSB = SET_MSB + 1;
localparam integer TAG_MSB = 24;
localparam integer TAG_RAM_BITS = (TAG_BITS < 16) ? 16 : TAG_BITS;
localparam integer STOREQ_DEPTH = 3;
localparam integer STOREQ_IDX_BITS = 2;
localparam integer STOREQ_CNT_BITS = 2;
localparam [STOREQ_CNT_BITS-1:0] STOREQ_DEPTH_VALUE = 2'd3;
localparam [STOREQ_IDX_BITS-1:0] STOREQ_LAST_IDX = 2'd2;
localparam [SET_BITS-1:0] LAST_SET = SET_BITS'(NUM_SETS - 1);

// Address decomposition.  The cache covers the low 32MB physical window.
wire [TAG_BITS-1:0] cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
// Set/word array index from the physical address page-offset bits
// (cpu_addr[11:2], translation-invariant -- available without the TLB result).
wire [SET_BITS-1:0] cpu_set = cpu_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] cpu_word = cpu_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [BRAM_ADDR_BITS-1:0] cpu_bram_addr = {cpu_set, cpu_word};
wire [SET_BITS-1:0] snoop_set = snoop_addr[SET_MSB:SET_LSB];
// A write to a ROM image is issued but never kept: memory drops it.
wire cpu_rom_write = cpu_write && z386_pkg::phys_rom(cpu_addr, ram_size);
// A bus cycle sent with `more` promised the bus another cycle of the same
// access, and the bus refuses HOLD until it comes. The read closing that
// access goes to the bus even when it would hit.
reg  more_open;
// The enable is taken a clock late so the walk below is running before a
// cacheable request can be accepted.
reg  cache_enable_q;
// With 8 MB, 600000-7FFFFF reads as ROM while paging is off and as RAM
// otherwise, so those reads never touch a line.
wire cpu_rom_view = ram_size == 2'd3 && !pg && !cpu_write && cpu_addr[23:21] == 3'b011;
wire cpu_uncacheable = !cache_enable_q || !z386_pkg::phys_cacheable(cpu_addr, ram_size) || cpu_rom_write ||
                       (more_open && !cpu_write) || cpu_rom_view;
wire cpu_protect_write = PROTECT_UMA_ROM && cpu_write && (cpu_addr[24:18] == 7'b000_0011);

`ifdef Z386_DISABLE_CACHE_RAM_HINTS
reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
reg valid_way0 [0:NUM_SETS-1];
reg valid_way1 [0:NUM_SETS-1];
reg valid_way2 [0:NUM_SETS-1];
reg valid_way3 [0:NUM_SETS-1];
reg [2:0] plru_set [0:NUM_SETS-1];
`else
// Tag/data storage.
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
// Valid bits and PLRU are block RAM as well: each array has one write
// statement and one read index, so it infers a simple dual-port M10K.
(* ramstyle = "M10K" *) reg valid_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way3 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [2:0] plru_set [0:NUM_SETS-1];
`endif

reg [31:0] data_way0 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way1 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way2 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way3 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];

// Synchronous cache read result for the request accepted in the previous cycle.
reg [TAG_BITS-1:0] rd_tag0_r, rd_tag1_r, rd_tag2_r, rd_tag3_r;
reg rd_valid0_r, rd_valid1_r, rd_valid2_r, rd_valid3_r;
reg [31:0] rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r;
reg [2:0] rd_plru_r;

// Accepted request register.
reg        req_valid_r;
reg [31:0] req_addr_r;
reg [31:0] req_din_r;
reg  [3:0] req_be_r;
reg        req_write_r;
reg        req_lock_r;
reg        req_more_r;
reg        req_uncacheable_r;
reg        req_protect_write_r;
reg [TAG_BITS-1:0] req_tag_r;
reg [SET_BITS-1:0] req_set_r;
reg                req_snooped_r; // a snoop hit this set on the accept edge
reg [WORD_OFFSET_BITS-1:0] req_word_r;

// Write-through store queue.
reg [29:0] storeq_addr [0:STOREQ_DEPTH-1];
reg [31:0] storeq_data [0:STOREQ_DEPTH-1];
reg  [3:0] storeq_be   [0:STOREQ_DEPTH-1];
reg        storeq_lock [0:STOREQ_DEPTH-1];
reg        storeq_more [0:STOREQ_DEPTH-1];
reg        storeq_drop [0:STOREQ_DEPTH-1];   // memory will drop it: never forward
reg        storeq_valid[0:STOREQ_DEPTH-1];
reg [STOREQ_IDX_BITS-1:0] storeq_head;
reg [STOREQ_IDX_BITS-1:0] storeq_tail;
reg [STOREQ_CNT_BITS-1:0] storeq_count;
reg        storeq_draining;

wire [STOREQ_DEPTH-1:0] storeq_live = {storeq_valid[2] && !storeq_drop[2],
                                       storeq_valid[1] && !storeq_drop[1],
                                       storeq_valid[0] && !storeq_drop[0]};
wire storeq_full = (storeq_count == STOREQ_DEPTH_VALUE);
wire storeq_empty = (storeq_count == {STOREQ_CNT_BITS{1'b0}});
wire storeq_can_accept = !storeq_full || (storeq_draining && mem_ready);

// Memory-side registers.
reg        mem_valid_r;
reg        mem_write_r;
reg        mem_lock_r;
reg        mem_more_r;
reg [31:0] mem_addr_r;
reg [31:0] mem_din_r;
reg  [3:0] mem_be_r;
reg  [7:0] mem_burstcount_r;

// An uncached access with nothing queued ahead of it goes to the bus in
// its accept cycle, skipping the lookup and the store queue.
wire direct_now;
assign mem_valid = mem_valid_r || direct_now;
assign mem_write = direct_now ? cpu_write : mem_write_r;
assign mem_lock = direct_now ? cpu_lock : mem_lock_r;
assign mem_more = direct_now ? cpu_more : mem_more_r;
// A write leaving on the direct path is already on the bus port this
// cycle; counting it here would close a loop through the arbiter.
assign store_pending = !storeq_empty || (mem_valid_r && mem_write_r);
assign mem_addr = direct_now ? cpu_addr : mem_addr_r;
assign mem_din = direct_now ? cpu_din : mem_din_r;
assign mem_be = direct_now ? cpu_be : mem_be_r;
assign mem_burstcount = direct_now ? 8'd1 : mem_burstcount_r;

// Cache FSM.
localparam [2:0] S_IDLE        = 3'd1;
localparam [2:0] S_LOOKUP      = 3'd2;
localparam [2:0] S_FILL        = 3'd3;
localparam [2:0] S_BYPASS_WAIT = 3'd4;

reg [2:0] state;
reg [SET_BITS-1:0] init_set;
reg init_busy;      // valid bits still being cleared after reset; cacheable requests wait, uncached ones do not
reg [SET_BITS-1:0] snoop_set_r;
reg snoop_valid_r;
reg [WORD_OFFSET_BITS-1:0] fill_count;
reg [WORD_OFFSET_BITS-1:0] fill_target_word;
reg [SET_BITS-1:0] fill_set;
reg [TAG_BITS-1:0] fill_tag;
reg [1:0] fill_way;
reg [2:0] fill_plru_r;
reg fill_requested;
reg fill_target_returned;
reg fill_stale;        // a snoop hit this set while the fill was in flight
reg fill_done;         // last word is in; the tag/valid write is waiting for the valid port

reg [31:0] dout_r;
reg resp_valid_r;
reg ready_r;

// During the reset walk only uncached requests are taken, so ready must say so.
assign cpu_ready = ready_r && (!init_busy || cpu_uncacheable || (cpu_lock && !cpu_write));

function automatic [31:0] be_mask(input [3:0] be);
begin
    be_mask = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
end
endfunction

function automatic [31:0] merge32(input [31:0] old_data, input [31:0] new_data, input [3:0] be);
    automatic reg [31:0] mask;
begin
    mask = be_mask(be);
    merge32 = (old_data & ~mask) | (new_data & mask);
end
endfunction

function automatic [31:0] forward_storeq_slot(
    input [31:0] value,
    input        slot_live,
    input [29:0] slot_addr,
    input [31:0] slot_data,
    input  [3:0] slot_be,
    input [29:0] addr_dw
);
begin
    forward_storeq_slot = (slot_live && slot_addr == addr_dw) ?
                          merge32(value, slot_data, slot_be) : value;
end
endfunction

function automatic [STOREQ_IDX_BITS-1:0] storeq_next_idx(input [STOREQ_IDX_BITS-1:0] idx);
begin
    storeq_next_idx = (idx == STOREQ_LAST_IDX) ? {STOREQ_IDX_BITS{1'b0}} : (idx + 1'b1);
end
endfunction

function automatic [STOREQ_IDX_BITS-1:0] storeq_prev_idx(input [STOREQ_IDX_BITS-1:0] idx);
begin
    storeq_prev_idx = (idx == {STOREQ_IDX_BITS{1'b0}}) ? STOREQ_LAST_IDX : (idx - 1'b1);
end
endfunction

function automatic [1:0] way_encode(input [3:0] hit_vec);
begin
    way_encode = hit_vec[0] ? 2'd0 :
                 hit_vec[1] ? 2'd1 :
                 hit_vec[2] ? 2'd2 : 2'd3;
end
endfunction

function automatic [3:0] way_onehot(input [1:0] way);
begin
    case (way)
        2'd0: way_onehot = 4'b0001;
        2'd1: way_onehot = 4'b0010;
        2'd2: way_onehot = 4'b0100;
        default: way_onehot = 4'b1000;
    endcase
end
endfunction

function automatic [31:0] way_data_mux(
    input [1:0] way,
    input [31:0] data0,
    input [31:0] data1,
    input [31:0] data2,
    input [31:0] data3
);
begin
    case (way)
        2'd0: way_data_mux = data0;
        2'd1: way_data_mux = data1;
        2'd2: way_data_mux = data2;
        default: way_data_mux = data3;
    endcase
end
endfunction

function automatic [2:0] plru_update(input [2:0] plru, input [1:0] way);
begin
    case (way)
        2'd0: plru_update = {plru[2], 1'b1, 1'b1};
        2'd1: plru_update = {plru[2], 1'b0, 1'b1};
        2'd2: plru_update = {1'b1, plru[1], 1'b0};
        default: plru_update = {1'b0, plru[1], 1'b0};
    endcase
end
endfunction

function automatic [1:0] plru_victim(input [2:0] plru);
begin
    if (!plru[0])
        plru_victim = plru[1] ? 2'd1 : 2'd0;
    else
        plru_victim = plru[2] ? 2'd3 : 2'd2;
end
endfunction

wire [3:0] lookup_hit_vec = {
    rd_valid3_r && (rd_tag3_r == req_tag_r),
    rd_valid2_r && (rd_tag2_r == req_tag_r),
    rd_valid1_r && (rd_tag1_r == req_tag_r),
    rd_valid0_r && (rd_tag0_r == req_tag_r)
};
// The valid bits were sampled on the accept edge; a snoop that lands on this
// set in the accept or lookup cycle is not in them.  Refuse the stale hit.
wire lookup_snoop_conflict = req_snooped_r ||
    (snoop_valid_r && (snoop_set_r == req_set_r)) ||
    (snoop_valid && (snoop_set == req_set_r));
wire lookup_hit = |lookup_hit_vec && !lookup_snoop_conflict;
wire fill_snoop_now = (snoop_valid_r && (snoop_set_r == fill_set)) || (snoop_valid && (snoop_set == fill_set));
wire [1:0] lookup_way = way_encode(lookup_hit_vec);
wire [31:0] lookup_way_data = way_data_mux(lookup_way, rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r);
wire [BRAM_ADDR_BITS-1:0] req_bram_addr = {req_set_r, req_word_r};
wire can_accept_cpu = (state == S_IDLE) && !reset && (!init_busy || cpu_uncacheable || (cpu_lock && !cpu_write)) &&
                      (!cpu_write || cpu_protect_write || storeq_can_accept);
wire ready_when_idle = !reset && storeq_can_accept;
wire accept_cpu = cpu_valid && ready_r && can_accept_cpu;
wire [29:0] req_addr_dw = req_addr_r[31:2];
wire req_bypass_r = req_uncacheable_r || req_lock_r;   // reads that must reach the bus
wire [29:0] fill_addr_dw = {req_addr_r[31:4], fill_count};
logic [31:0] lookup_forward_data;
logic [31:0] fill_word_data;
logic [31:0] bypass_forward_data;
wire lookup_read_hit_now = (state == S_LOOKUP) && req_valid_r &&
                           !req_write_r && !req_bypass_r && lookup_hit;
wire direct_issue = (cpu_uncacheable || (cpu_lock && !cpu_write)) && !cpu_protect_write &&
                    storeq_empty && !storeq_draining && !mem_valid_r && !mem_busy;
// Same as accept_cpu && direct_issue (an empty queue always accepts), spelt
// without storeq_can_accept so the bus-side ready cannot loop back here.
assign direct_now = cpu_valid && ready_r && (state == S_IDLE) && !reset &&
                    (!init_busy || cpu_uncacheable || (cpu_lock && !cpu_write)) && direct_issue;

// One write per clock on the valid-bit RAMs.  The reset walk runs alone
// (cacheable requests wait, a snoop during it is dropped: nothing is
// allocated yet).  A snoop clear always writes in its own clock; a fill
// whose validate would collide with it holds in S_FILL for one more clock.
wire snoop_write = snoop_valid_r && !init_busy;
wire fill_last = (state == S_FILL) && mem_resp_valid && (fill_count == {WORD_OFFSET_BITS{1'b1}});
wire fill_validate = !fill_stale && !fill_snoop_now;
wire fill_commit = (state == S_FILL) && (fill_last || fill_done) && !(fill_validate && snoop_write);
wire fill_validate_now = fill_commit && fill_validate;
wire [3:0] valid_we = init_busy ? 4'hF :
                      snoop_write ? 4'hF :
                      fill_validate_now ? way_onehot(fill_way) : 4'h0;
wire [SET_BITS-1:0] valid_waddr = init_busy ? init_set :
                                  snoop_write ? snoop_set_r : fill_set;
wire valid_wbit = !init_busy && !snoop_write;   // only the validate writes a 1

// PLRU: the reset walk, a lookup hit and a fill end never share a clock.
wire plru_hit_we = (state == S_LOOKUP) && !req_protect_write_r && lookup_hit &&
                   (req_write_r ? !req_uncacheable_r : !req_bypass_r);
wire plru_we = init_busy || plru_hit_we || fill_last;
wire [SET_BITS-1:0] plru_waddr = init_busy ? init_set :
                                 plru_hit_we ? req_set_r : fill_set;
wire [2:0] plru_wdata = init_busy ? 3'b000 :
                        plru_hit_we ? plru_update(rd_plru_r, lookup_way) :
                                      plru_update(fill_plru_r, fill_way);

// Bypassed read data is handed on the cycle it arrives.
wire bypass_resp_now = (state == S_BYPASS_WAIT) && mem_resp_valid;

assign cpu_dout = lookup_read_hit_now ? lookup_forward_data :
                  bypass_resp_now ? bypass_forward_data : dout_r;
assign cpu_resp_valid = lookup_read_hit_now || bypass_resp_now || resp_valid_r;

// Store-queue drain issue, decoupled from the FSM: drains may launch while the
// FSM is accepting or patching, so back-to-back writes are not serialized
// behind accept/S_LOOKUP cycles.  Blocked only while a fill or bypass read
// is out on the memory side.  Every read that reaches the bus waits for the
// queue to empty first, so no read overtakes an older posted store: a
// locked read-modify-write stays whole on the bus and a device register or
// VRAM window sees the store before the read.
wire drain_block_state = ((state == S_FILL) && fill_requested) ||
                         (state == S_BYPASS_WAIT);
wire drain_issue_now = !storeq_empty && !storeq_draining && !mem_valid_r &&
                       !mem_busy && !drain_block_state;

// Coalesce a store (enqueued during its S_LOOKUP cycle, from registered
// request state) into the most recent queue entry when it targets the same
// DWORD.  Excluded: uncacheable writes (MMIO transaction boundaries must be
// preserved) and entries that are draining (their data is already on the
// memory command registers this cycle).
wire [STOREQ_IDX_BITS-1:0] storeq_prev = storeq_prev_idx(storeq_head);
wire storeq_merge_lookup = !storeq_empty && storeq_valid[storeq_prev] &&
                           (storeq_addr[storeq_prev] == req_addr_r[31:2]) &&
                           !req_uncacheable_r && !req_lock_r &&
                           !(storeq_prev == storeq_tail && (storeq_draining || drain_issue_now));
// Store-queue count after this cycle's enqueue, including a simultaneously
// completing drain.  Drives the post-write ready_r so a full queue is seen
// immediately despite the one-cycle-late enqueue.
wire storeq_dequeuing = storeq_draining && mem_ready;
wire [STOREQ_CNT_BITS-1:0] storeq_count_wr_next =
     storeq_merge_lookup ? (storeq_dequeuing ? storeq_count - 1'b1 : storeq_count)
                         : (storeq_dequeuing ? storeq_count : storeq_count + 1'b1);

always_comb begin
    lookup_forward_data = lookup_way_data;
    fill_word_data = mem_dout;
    bypass_forward_data = mem_dout;

    unique case (storeq_tail)
        2'd0: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
        end
        2'd1: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
        end
        default: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_live[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
        end
    endcase
end

task automatic write_cache_word(input [1:0] way, input [BRAM_ADDR_BITS-1:0] addr, input [31:0] data);
begin
    case (way)
        2'd0: data_way0[addr] <= data;
        2'd1: data_way1[addr] <= data;
        2'd2: data_way2[addr] <= data;
        default: data_way3[addr] <= data;
    endcase
end
endtask

task automatic write_cache_tag(input [1:0] way, input [SET_BITS-1:0] set, input [TAG_BITS-1:0] tag);
begin
    case (way)
        2'd0: tag_way0[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag};
        2'd1: tag_way1[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag};
        2'd2: tag_way2[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag};
        default: tag_way3[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag};
    endcase
end
endtask

// The only write statement of each bookkeeping RAM.
always_ff @(posedge clk) begin
    if (clk_en) begin
        if (valid_we[0]) valid_way0[valid_waddr] <= valid_wbit;
        if (valid_we[1]) valid_way1[valid_waddr] <= valid_wbit;
        if (valid_we[2]) valid_way2[valid_waddr] <= valid_wbit;
        if (valid_we[3]) valid_way3[valid_waddr] <= valid_wbit;
        if (plru_we) plru_set[plru_waddr] <= plru_wdata;
    end
end

// Preread runs on every ready idle cycle, with no cpu_valid/TLB gating: when
// no request is accepted the preread results are garbage that S_LOOKUP never
// sees (it is only entered on accept_cpu).  This keeps the TLB-hit cone off
// the wide rd_*_r register enables.
wire idle_preread = (state == S_IDLE) && ready_r;

always_ff @(posedge clk) begin
    if (clk_en) begin
    if (idle_preread) begin
        rd_tag0_r <= tag_way0[cpu_set][TAG_BITS-1:0];
        rd_tag1_r <= tag_way1[cpu_set][TAG_BITS-1:0];
        rd_tag2_r <= tag_way2[cpu_set][TAG_BITS-1:0];
        rd_tag3_r <= tag_way3[cpu_set][TAG_BITS-1:0];
        rd_valid0_r <= valid_way0[cpu_set];
        rd_valid1_r <= valid_way1[cpu_set];
        rd_valid2_r <= valid_way2[cpu_set];
        rd_valid3_r <= valid_way3[cpu_set];
        rd_data0_r <= data_way0[cpu_bram_addr];
        rd_data1_r <= data_way1[cpu_bram_addr];
        rd_data2_r <= data_way2[cpu_bram_addr];
        rd_data3_r <= data_way3[cpu_bram_addr];
        rd_plru_r <= plru_set[cpu_set];
    end
    end
end

always_ff @(posedge clk) begin
    if (clk_en) begin
    automatic reg [31:0] patched;

    if (reset) begin
        state <= S_IDLE;
        init_set <= {SET_BITS{1'b0}};
        init_busy <= 1'b1;
        cache_enable_q <= cache_enable;
        req_valid_r <= 1'b0;
        ready_r <= 1'b0;
        resp_valid_r <= 1'b0;
        dout_r <= 32'h0;
        mem_valid_r <= 1'b0;
        mem_write_r <= 1'b0;
        mem_lock_r <= 1'b0;
        mem_more_r <= 1'b0;
        mem_addr_r <= 32'h0;
        mem_din_r <= 32'h0;
        mem_be_r <= 4'h0;
        mem_burstcount_r <= 8'h0;
        storeq_head <= {STOREQ_IDX_BITS{1'b0}};
        storeq_tail <= {STOREQ_IDX_BITS{1'b0}};
        storeq_count <= {STOREQ_CNT_BITS{1'b0}};
        storeq_draining <= 1'b0;
        fill_requested <= 1'b0;
        fill_target_returned <= 1'b0;
        fill_stale <= 1'b0;
        fill_done <= 1'b0;
        more_open <= 1'b0;
        snoop_set_r <= {SET_BITS{1'b0}};
        snoop_valid_r <= 1'b0;
        req_snooped_r <= 1'b0;
        for (integer i = 0; i < STOREQ_DEPTH; i = i + 1)
            storeq_valid[i] <= 1'b0;
    end else begin
        ready_r <= (state == S_IDLE) && ready_when_idle;
        resp_valid_r <= 1'b0;
        if (mem_valid && mem_ready)
            more_open <= mem_more;
        snoop_valid_r <= snoop_valid;
        if (snoop_valid)
            snoop_set_r <= snoop_set;

        if (mem_valid_r && mem_ready)
            mem_valid_r <= 1'b0;

        if (storeq_draining && mem_ready) begin
            storeq_valid[storeq_tail] <= 1'b0;
            storeq_tail <= storeq_next_idx(storeq_tail);
            storeq_count <= storeq_count - 1'b1;
            storeq_draining <= 1'b0;
        end

        // FSM-independent store-queue drain launch (see drain_issue_now).
        if (drain_issue_now) begin
            mem_valid_r <= 1'b1;
            mem_write_r <= 1'b1;
            mem_lock_r <= storeq_lock[storeq_tail];
            mem_more_r <= storeq_more[storeq_tail];
            mem_addr_r <= {storeq_addr[storeq_tail], 2'b00};
            mem_din_r <= storeq_data[storeq_tail];
            mem_be_r <= storeq_be[storeq_tail];
            mem_burstcount_r <= 8'd1;
            storeq_draining <= 1'b1;
        end

        // Reset walk: one set per clock (the RAM clears ride on valid_we and
        // plru_we), alongside whatever uncached traffic the FSM carries.
        // Stores made while the cache was off went straight to the bus, so
        // turning it back on runs the walk again.
        cache_enable_q <= cache_enable;
        if (init_busy) begin
            if (init_set == LAST_SET) init_busy <= 1'b0;
            else                      init_set  <= init_set + 1'b1;
        end else if (cache_enable && !cache_enable_q) begin
            init_set <= {SET_BITS{1'b0}};
            init_busy <= 1'b1;
        end

        case (state)
            S_IDLE: begin
                // Wide request captures run on every ready cycle, with no
                // cpu_valid/TLB gating: garbage is captured when nothing is
                // accepted, but S_LOOKUP (the only consumer) is entered on
                // accept_cpu alone.  Keeps the TLB cone off these enables.
                if (ready_r) begin
                    req_addr_r <= cpu_addr;
                    req_din_r <= cpu_din;
                    req_be_r <= cpu_be;
                    req_write_r <= cpu_write;
                    req_lock_r <= cpu_lock;
                    req_more_r <= cpu_more;
                    req_uncacheable_r <= cpu_uncacheable;
                    req_protect_write_r <= cpu_protect_write;
                    req_tag_r <= cpu_tag;
                    req_set_r <= cpu_set;
                    req_snooped_r <= snoop_valid_r && (snoop_set_r == cpu_set);
                    req_word_r <= cpu_word;
                end
                if (direct_now) begin
                    mem_valid_r <= !mem_ready;
                    mem_write_r <= cpu_write;
                    mem_lock_r <= cpu_lock;
                    mem_more_r <= cpu_more;
                    mem_addr_r <= cpu_addr;
                    mem_din_r <= cpu_din;
                    mem_be_r <= cpu_be;
                    mem_burstcount_r <= 8'd1;
                    // a write is posted; a read waits for its data
                    if (!cpu_write) begin
                        ready_r <= 1'b0;
                        state <= S_BYPASS_WAIT;
                    end
                end else if (accept_cpu) begin
                    ready_r <= 1'b0;
                    req_valid_r <= 1'b1;
                    state <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
                req_valid_r <= 1'b0;

                if (req_protect_write_r) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else if (req_write_r) begin
                    // Store-queue enqueue, moved here from the accept cycle so
                    // its enables come from registered request state instead of
                    // the TLB-gated accept.  All inputs are req_*_r registers.
                    if (storeq_merge_lookup) begin
                        // Same-DWORD coalescing: fold into the newest entry.
                        storeq_data[storeq_prev] <= merge32(storeq_data[storeq_prev], req_din_r, req_be_r);
                        storeq_be[storeq_prev] <= storeq_be[storeq_prev] | req_be_r;
                    end else begin
                        storeq_addr[storeq_head] <= req_addr_r[31:2];
                        storeq_data[storeq_head] <= req_din_r;
                        storeq_be[storeq_head] <= req_be_r;
                        storeq_lock[storeq_head] <= req_lock_r;
                        storeq_more[storeq_head] <= req_more_r;
                        storeq_drop[storeq_head] <= z386_pkg::phys_rom(req_addr_r, ram_size);
                        storeq_valid[storeq_head] <= 1'b1;
                        storeq_head <= storeq_next_idx(storeq_head);
                    end
                    storeq_count <= storeq_count_wr_next;
                    if (lookup_hit && !req_uncacheable_r) begin
                        patched = merge32(lookup_way_data, req_din_r, req_be_r);
                        write_cache_word(lookup_way, req_bram_addr, patched);
                    end
                    state <= S_IDLE;
                    ready_r <= (storeq_count_wr_next != STOREQ_DEPTH_VALUE);
                end else if (req_bypass_r) begin
                    if (storeq_empty && !storeq_draining && !mem_valid_r && !mem_busy) begin
                        mem_valid_r <= 1'b1;
                        mem_write_r <= 1'b0;
                        mem_lock_r <= req_lock_r;
                        mem_more_r <= req_more_r;
                        mem_addr_r <= req_addr_r;
                        mem_din_r <= 32'h0;
                        mem_be_r <= req_be_r;
                        mem_burstcount_r <= 8'd1;
                        state <= S_BYPASS_WAIT;
                    end
                end else if (lookup_hit) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else begin
                    fill_set <= req_set_r;
                    fill_tag <= req_tag_r;
                    fill_way <= plru_victim(rd_plru_r);
                    fill_plru_r <= rd_plru_r;
                    fill_count <= {WORD_OFFSET_BITS{1'b0}};
                    fill_target_word <= req_word_r;
                    fill_requested <= 1'b0;
                    fill_target_returned <= 1'b0;
                    fill_stale <= req_snooped_r || (snoop_valid_r && (snoop_set_r == req_set_r)) ||
                                  (snoop_valid && (snoop_set == req_set_r));
                    state <= S_FILL;
                end
            end

            S_FILL: begin
                if (fill_snoop_now)
                    fill_stale <= 1'b1;
                if (!fill_requested && !mem_valid_r && !mem_busy && storeq_empty && !storeq_draining) begin
                    mem_valid_r <= 1'b1;
                    mem_write_r <= 1'b0;
                    mem_lock_r <= 1'b0;
                    mem_more_r <= 1'b0;
                    mem_addr_r <= {req_addr_r[31:4], 4'b0000};
                    mem_din_r <= 32'h0;
                    mem_be_r <= 4'hF;
                    mem_burstcount_r <= 8'd4;
                    fill_requested <= 1'b1;
                end

                if (mem_resp_valid) begin
                    write_cache_word(fill_way, {fill_set, fill_count}, fill_word_data);

                    if (fill_count == fill_target_word && !fill_target_returned) begin
                        dout_r <= fill_word_data;
                        resp_valid_r <= 1'b1;
                        fill_target_returned <= 1'b1;
                    end

                    fill_count <= fill_count + 1'b1;
                end

                // Tag and valid go in together (valid through valid_we).  A
                // DMA write that reached this set during the fill may have
                // hit a word already fetched: keep the line invalid rather
                // than validate stale data.  The other ways' valid bits are
                // never restored from a fill-start snapshot: a snoop clear
                // landing during the fill must survive.
                if (fill_last && !fill_commit)
                    fill_done <= 1'b1;
                if (fill_commit) begin
                    if (fill_validate)
                        write_cache_tag(fill_way, fill_set, fill_tag);
                    fill_done <= 1'b0;
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end
            end

            S_BYPASS_WAIT: begin
                if (mem_resp_valid) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (clk_en) begin
    if (!reset && !init_busy && cpu_valid && !cpu_ready && !(state == S_IDLE))
        ;
    end
end
// synthesis translate_on

// synthesis translate_off
// The single write ports rely on these writers never sharing a clock, and
// the PLRU read on accept never landing on the edge of a PLRU write.
always_ff @(posedge clk) begin
    if (clk_en && !reset) begin
        if (init_busy && (fill_validate_now || plru_hit_we || fill_last))
            $error("l1_cache: cache write during the reset walk");
        if (plru_hit_we && fill_last)
            $error("l1_cache: PLRU hit and fill updates in one clock");
        if (accept_cpu && !init_busy && plru_we)
            $error("l1_cache: PLRU read and write on the same edge");
    end
end
// synthesis translate_on

endmodule

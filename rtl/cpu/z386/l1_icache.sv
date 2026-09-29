// Read-only physically indexed, physically tagged L1 instruction cache.
//
// CPU-side contract:
//   * cpu_addr is a physical byte address.
//   * A cache-hit read accepted in cycle N returns cpu_resp_valid in N+1.
//   * Miss/refill and uncached accesses use the memory-side burst interface.
//
// This is the instruction-cache half of l1_cache.sv with the store buffer and
// write datapath removed.
module l1_icache #(
    parameter integer SET_BITS = 8   // 16KB icache (256 sets x 4 ways x 16 B); =7 was 8KB
) (
    input         clk,
    input         clk_en,      // clock enable: the core advances only when high
    input         reset,

    // CPU side — physical read request/response.
    input  [31:0] cpu_addr,
    output [127:0] cpu_line,
    input         cpu_valid,
    output        cpu_ready,
    output        cpu_resp_valid,
    // Uncached fill words as they arrive, ahead of the whole line.
    output        cpu_word_valid,
    output  [1:0] cpu_word_idx,
    output [31:0] cpu_word_data,
    output        cpu_fill_uncached, // the fill in progress allocates nothing
    input         cpu_abort,         // discard an uncached fill: it completes at once

    // Memory side.
    output [31:0] mem_addr,
    input  [31:0] mem_dout,
    output  [3:0] mem_be,
    output  [7:0] mem_burstcount,
    input         mem_busy,
    output        mem_valid,
    input         mem_ready,
    input         mem_resp_valid,

    // Physical-address snoop.  Data-bearing CPU store snoops patch matching
    // cached words; address-only external snoops invalidate matching lines.
    input  [31:0] snoop_addr,
    input  [31:0] snoop_data,
    input   [3:0] snoop_be,
    input         snoop_patch,
    input         snoop_valid,

    input         cache_enable,
    input   [1:0] ram_size,     // fitted DRAM: 0 2 MB, 1 4 MB, 2 6 MB, 3 8 MB
    input         pg            // CR0.PG: with 8 MB, a read of 600000-7FFFFF with paging off is the ROM disk
);

localparam integer WORD_OFFSET_BITS = 2;
localparam integer BYTE_OFFSET_BITS = 2;
localparam integer LINE_OFFSET_BITS = WORD_OFFSET_BITS + BYTE_OFFSET_BITS;
localparam integer NUM_SETS = 1 << SET_BITS;
localparam integer TAG_BITS = 25 - LINE_OFFSET_BITS - SET_BITS;
localparam integer SET_LSB = LINE_OFFSET_BITS;
localparam integer SET_MSB = SET_LSB + SET_BITS - 1;
localparam integer TAG_LSB = SET_MSB + 1;
localparam integer TAG_MSB = 24;
localparam integer TAG_RAM_BITS = (TAG_BITS < 16) ? 16 : TAG_BITS;
localparam [SET_BITS-1:0] LAST_SET = SET_BITS'(NUM_SETS - 1);
localparam integer PATCHQ_DEPTH = 3;
localparam integer PATCHQ_IDX_BITS = 2;
localparam [PATCHQ_IDX_BITS-1:0] PATCHQ_LAST_IDX = 2'd2;
localparam integer SET_DW_LSB = SET_LSB - BYTE_OFFSET_BITS;
localparam integer SET_DW_MSB = SET_MSB - BYTE_OFFSET_BITS;
localparam integer TAG_DW_LSB = TAG_LSB - BYTE_OFFSET_BITS;
localparam integer TAG_DW_MSB = TAG_MSB - BYTE_OFFSET_BITS;

wire [TAG_BITS-1:0] cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
wire [SET_BITS-1:0] cpu_set = cpu_addr[SET_MSB:SET_LSB];
wire [TAG_BITS-1:0] snoop_tag = snoop_addr[TAG_MSB:TAG_LSB];
wire [SET_BITS-1:0] snoop_set = snoop_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] snoop_word = snoop_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
// An uncacheable fetch takes the fill path without allocating: it starts at
// the requested word, stops at the end of the line, and hands each DWORD to
// the prefetcher as it arrives.
wire cpu_uncacheable = !cache_enable || !z386_pkg::phys_cacheable(cpu_addr, ram_size) ||
                       (ram_size == 2'd3 && !pg && cpu_addr[23:21] == 3'b011);

`ifdef Z386_DISABLE_CACHE_RAM_HINTS
reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
reg valid_way0 [0:NUM_SETS-1];
reg valid_way1 [0:NUM_SETS-1];
reg valid_way2 [0:NUM_SETS-1];
reg valid_way3 [0:NUM_SETS-1];
reg valid_snoop_way0 [0:NUM_SETS-1];
reg valid_snoop_way1 [0:NUM_SETS-1];
reg valid_snoop_way2 [0:NUM_SETS-1];
reg valid_snoop_way3 [0:NUM_SETS-1];
reg [2:0] plru_set [0:NUM_SETS-1];
`else
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
// Valid bits and PLRU are block RAM as well: each array has one write
// statement and one read index, so it infers a simple dual-port M10K.
// The valid bits are kept twice, one copy read at the lookup set and one
// at the snoop set; both copies take the same write.
(* ramstyle = "M10K" *) reg valid_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_way3 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_snoop_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_snoop_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_snoop_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg valid_snoop_way3 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [2:0] plru_set [0:NUM_SETS-1];
`endif

`ifdef Z386_DISABLE_CACHE_RAM_HINTS
reg [127:0] data_way0 [0:NUM_SETS-1];
reg [127:0] data_way1 [0:NUM_SETS-1];
reg [127:0] data_way2 [0:NUM_SETS-1];
reg [127:0] data_way3 [0:NUM_SETS-1];
`else
(* ramstyle = "M10K" *) reg [127:0] data_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way3 [0:NUM_SETS-1];
`endif

reg [TAG_BITS-1:0] rd_tag0_r, rd_tag1_r, rd_tag2_r, rd_tag3_r;
reg rd_valid0_r, rd_valid1_r, rd_valid2_r, rd_valid3_r;
reg snoop_rd_valid0_r, snoop_rd_valid1_r, snoop_rd_valid2_r, snoop_rd_valid3_r;
reg [127:0] rd_line0_r, rd_line1_r, rd_line2_r, rd_line3_r;
reg [2:0] rd_plru_r;

reg        req_valid_r;
reg [31:0] req_addr_r;
reg        req_uncacheable_r;
reg [TAG_BITS-1:0] req_tag_r;
reg [SET_BITS-1:0] req_set_r;
reg                req_snooped_r; // a snoop hit this line on the accept edge

reg        mem_valid_r;
reg [31:0] mem_addr_r;
reg  [7:0] mem_burstcount_r;
reg  [3:0] mem_be_r;

// An uncached fetch goes to the bus in its accept cycle: from the
// requested word to the end of the line.
wire direct_now;
wire [7:0] direct_burst = 8'd4 - {6'd0, cpu_addr[3:2]};
assign mem_valid = mem_valid_r || direct_now;
assign mem_addr = direct_now ? {cpu_addr[31:2], 2'b00} : mem_addr_r;
assign mem_be = direct_now ? (cpu_addr[1] ? 4'b1100 : 4'b1111) : mem_be_r;
assign mem_burstcount = direct_now ? direct_burst : mem_burstcount_r;

localparam [2:0] S_IDLE        = 3'd1;
localparam [2:0] S_LOOKUP      = 3'd2;
localparam [2:0] S_FILL        = 3'd3;
localparam [2:0] S_BYPASS_WAIT = 3'd4;

reg [2:0] state;
reg [SET_BITS-1:0] init_set;
reg init_busy;      // valid bits still being cleared after reset; cacheable fetches wait, uncached ones do not
reg [TAG_BITS-1:0] snoop_tag_r;
reg [SET_BITS-1:0] snoop_set_r;
reg [WORD_OFFSET_BITS-1:0] snoop_word_r;
reg [29:0] snoop_addr_dw_r;
reg [31:0] snoop_data_r;
reg [3:0] snoop_be_r;
reg snoop_patch_r;
reg snoop_valid_r;
reg [29:0] patchq_addr [0:PATCHQ_DEPTH-1];
reg [31:0] patchq_data [0:PATCHQ_DEPTH-1];
reg  [3:0] patchq_be   [0:PATCHQ_DEPTH-1];
reg        patchq_valid[0:PATCHQ_DEPTH-1];
reg [PATCHQ_IDX_BITS-1:0] patchq_head;
reg [WORD_OFFSET_BITS-1:0] fill_count;
reg [SET_BITS-1:0] fill_set;
reg [TAG_BITS-1:0] fill_tag;
reg [1:0] fill_way;
reg [127:0] fill_line;
reg [2:0] fill_plru_r;
reg fill_requested;
reg fill_stale;        // an address-only snoop (DMA) hit this line while the fill was in flight

reg [127:0] line_r;
reg resp_valid_r;
reg ready_r;

// Uncached words go to the prefetcher the cycle they land.
assign cpu_fill_uncached = (state == S_FILL) && req_uncacheable_r;
assign cpu_word_valid = cpu_fill_uncached && mem_resp_valid;
assign cpu_word_idx   = fill_count;
assign cpu_word_data  = fill_word_next;

// During the reset walk only uncached requests are taken, so ready must say so.
assign cpu_ready = ready_r && (!init_busy || cpu_uncacheable);

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

function automatic [127:0] way_line_mux(
    input [1:0] way,
    input [127:0] data0,
    input [127:0] data1,
    input [127:0] data2,
    input [127:0] data3
);
begin
    case (way)
        2'd0: way_line_mux = data0;
        2'd1: way_line_mux = data1;
        2'd2: way_line_mux = data2;
        default: way_line_mux = data3;
    endcase
end
endfunction

// select_word removed: the single-word read path is dead (superseded by the
// 128-bit cpu_line output).

function automatic [127:0] patch_line_word(input [127:0] line, input [1:0] word, input [31:0] data);
begin
    patch_line_word = line;
    patch_line_word[{word, 5'b0} +: 32] = data;
end
endfunction

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

function automatic [127:0] patch_line_word_be(
    input [127:0] line,
    input [1:0] word,
    input [31:0] data,
    input [3:0] be
);
begin
    patch_line_word_be = line;
    patch_line_word_be[{word, 5'b0} +: 32] =
        merge32(line[{word, 5'b0} +: 32], data, be);
end
endfunction

function automatic [PATCHQ_IDX_BITS-1:0] patchq_next_idx(input [PATCHQ_IDX_BITS-1:0] idx);
begin
    patchq_next_idx = (idx == PATCHQ_LAST_IDX) ? {PATCHQ_IDX_BITS{1'b0}} : (idx + 1'b1);
end
endfunction

function automatic logic line_match_dw(
    input [29:0] addr_dw,
    input [TAG_BITS-1:0] tag,
    input [SET_BITS-1:0] set
);
begin
    line_match_dw = (addr_dw[TAG_DW_MSB:TAG_DW_LSB] == tag) &&
                    (addr_dw[SET_DW_MSB:SET_DW_LSB] == set);
end
endfunction

function automatic logic word_match_dw(
    input [29:0] addr_dw,
    input [TAG_BITS-1:0] tag,
    input [SET_BITS-1:0] set,
    input [WORD_OFFSET_BITS-1:0] word
);
begin
    word_match_dw = line_match_dw(addr_dw, tag, set) &&
                    (addr_dw[WORD_OFFSET_BITS-1:0] == word);
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
// The valid bits were sampled on the accept edge, so a snoop landing on this
// line in the accept or lookup cycle is not in them.  Refuse that stale hit
// and refill instead.
wire lookup_snoop_conflict = req_snooped_r ||
    (snoop_valid_r && (snoop_tag_r == req_tag_r) && (snoop_set_r == req_set_r)) ||
    (snoop_valid && (snoop_tag == req_tag_r) && (snoop_set == req_set_r));
wire lookup_hit = |lookup_hit_vec && !lookup_snoop_conflict;
wire [1:0] lookup_way = way_encode(lookup_hit_vec);
wire [127:0] lookup_way_line = way_line_mux(lookup_way, rd_line0_r, rd_line1_r, rd_line2_r, rd_line3_r);
wire can_accept_cpu = (state == S_IDLE) && !reset && (!init_busy || cpu_uncacheable);
wire accept_cpu = cpu_valid && ready_r && can_accept_cpu;
assign direct_now = accept_cpu && cpu_uncacheable && !mem_valid_r && !mem_busy;
wire lookup_read_hit_now = (state == S_LOOKUP) && req_valid_r &&
                           !req_uncacheable_r && lookup_hit;
logic [PATCHQ_DEPTH-1:0] patchq_snoop_match;
logic patchq_snoop_hit;
logic [31:0] fill_word_next;
logic [127:0] fill_line_base;
logic [127:0] fill_line_next;

always_comb begin
    patchq_snoop_match = {PATCHQ_DEPTH{1'b0}};
    for (int p = 0; p < PATCHQ_DEPTH; p++)
        patchq_snoop_match[p] = patchq_valid[p] && patchq_addr[p] == snoop_addr_dw_r;
    patchq_snoop_hit = |patchq_snoop_match;
end

always_comb begin
    fill_word_next = mem_dout;
    for (int p = 0; p < PATCHQ_DEPTH; p++) begin
        if (patchq_valid[p] && word_match_dw(patchq_addr[p], fill_tag, fill_set, fill_count))
            fill_word_next = merge32(fill_word_next, patchq_data[p], patchq_be[p]);
    end
    if (snoop_valid_r && snoop_patch_r && word_match_dw(snoop_addr_dw_r, fill_tag, fill_set, fill_count))
        fill_word_next = merge32(fill_word_next, snoop_data_r, snoop_be_r);
    if (snoop_valid && snoop_patch && word_match_dw(snoop_addr[31:2], fill_tag, fill_set, fill_count))
        fill_word_next = merge32(fill_word_next, snoop_data, snoop_be);

    fill_line_base = fill_line;
    if (snoop_valid_r && snoop_patch_r && line_match_dw(snoop_addr_dw_r, fill_tag, fill_set))
        fill_line_base = patch_line_word_be(fill_line_base, snoop_word_r, snoop_data_r, snoop_be_r);
    if (snoop_valid && snoop_patch && line_match_dw(snoop_addr[31:2], fill_tag, fill_set))
        fill_line_base = patch_line_word_be(fill_line_base, snoop_word, snoop_data, snoop_be);
    fill_line_next = patch_line_word(fill_line_base, fill_count, fill_word_next);
end

wire fill_snoop_now = (snoop_valid_r && !snoop_patch_r && line_match_dw(snoop_addr_dw_r, fill_tag, fill_set)) ||
                      (snoop_valid && !snoop_patch && line_match_dw(snoop_addr[31:2], fill_tag, fill_set));

// One write per clock on the valid-bit RAMs.  The reset walk runs alone
// (cacheable fetches wait, a snoop during it is dropped: nothing is
// allocated yet).  A snoop clear always writes in its own clock; a fill
// whose validate would collide with it is not allocated.  That is safe
// here because the victim line's data and tag are untouched until the
// validate, so the fetch is only delivered without being kept.
wire [3:0] snoop_inv_mask = {
    snoop_rd_valid3_r && (tag_way3[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r),
    snoop_rd_valid2_r && (tag_way2[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r),
    snoop_rd_valid1_r && (tag_way1[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r),
    snoop_rd_valid0_r && (tag_way0[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r)
};
wire snoop_write = snoop_valid_r && !init_busy && |snoop_inv_mask;
wire fill_last = (state == S_FILL) && mem_resp_valid && (fill_count == {WORD_OFFSET_BITS{1'b1}});
wire fill_validate = !req_uncacheable_r && !fill_stale && !fill_snoop_now && !snoop_write;
wire fill_validate_now = fill_last && fill_validate;
wire [3:0] valid_we = init_busy ? 4'hF :
                      snoop_write ? snoop_inv_mask :
                      fill_validate_now ? way_onehot(fill_way) : 4'h0;
wire [SET_BITS-1:0] valid_waddr = init_busy ? init_set :
                                  snoop_write ? snoop_set_r : fill_set;
wire valid_wbit = !init_busy && !snoop_write;   // only the validate writes a 1

// PLRU: the reset walk, a lookup hit and a fill end never share a clock.
wire plru_hit_we = (state == S_LOOKUP) && lookup_hit && !req_uncacheable_r;
wire plru_fill_we = fill_last && !req_uncacheable_r;
wire plru_we = init_busy || plru_hit_we || plru_fill_we;
wire [SET_BITS-1:0] plru_waddr = init_busy ? init_set :
                                 plru_hit_we ? req_set_r : fill_set;
wire [2:0] plru_wdata = init_busy ? 3'b000 :
                        plru_hit_we ? plru_update(rd_plru_r, lookup_way) :
                                      plru_update(fill_plru_r, fill_way);

assign cpu_line = lookup_read_hit_now ? lookup_way_line : line_r;
assign cpu_resp_valid = lookup_read_hit_now || resp_valid_r;

task automatic write_cache_line(input [1:0] way, input [SET_BITS-1:0] set, input [127:0] line);
begin
    case (way)
        2'd0: data_way0[set] <= line;
        2'd1: data_way1[set] <= line;
        2'd2: data_way2[set] <= line;
        default: data_way3[set] <= line;
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
        if (valid_we[0]) begin valid_way0[valid_waddr] <= valid_wbit; valid_snoop_way0[valid_waddr] <= valid_wbit; end
        if (valid_we[1]) begin valid_way1[valid_waddr] <= valid_wbit; valid_snoop_way1[valid_waddr] <= valid_wbit; end
        if (valid_we[2]) begin valid_way2[valid_waddr] <= valid_wbit; valid_snoop_way2[valid_waddr] <= valid_wbit; end
        if (valid_we[3]) begin valid_way3[valid_waddr] <= valid_wbit; valid_snoop_way3[valid_waddr] <= valid_wbit; end
        if (plru_we) plru_set[plru_waddr] <= plru_wdata;
    end
end

// Snoop copy of the valid bits, read on the edge that captures the snoop.
always_ff @(posedge clk) begin
    if (clk_en && snoop_valid) begin
        snoop_rd_valid0_r <= valid_snoop_way0[snoop_set];
        snoop_rd_valid1_r <= valid_snoop_way1[snoop_set];
        snoop_rd_valid2_r <= valid_snoop_way2[snoop_set];
        snoop_rd_valid3_r <= valid_snoop_way3[snoop_set];
    end
end

always_ff @(posedge clk) begin
    if (clk_en) begin
    if (accept_cpu) begin
        rd_tag0_r <= tag_way0[cpu_set][TAG_BITS-1:0];
        rd_tag1_r <= tag_way1[cpu_set][TAG_BITS-1:0];
        rd_tag2_r <= tag_way2[cpu_set][TAG_BITS-1:0];
        rd_tag3_r <= tag_way3[cpu_set][TAG_BITS-1:0];
        rd_valid0_r <= valid_way0[cpu_set];
        rd_valid1_r <= valid_way1[cpu_set];
        rd_valid2_r <= valid_way2[cpu_set];
        rd_valid3_r <= valid_way3[cpu_set];
        rd_line0_r <= data_way0[cpu_set];
        rd_line1_r <= data_way1[cpu_set];
        rd_line2_r <= data_way2[cpu_set];
        rd_line3_r <= data_way3[cpu_set];
        rd_plru_r <= plru_set[cpu_set];
    end
    end
end

always_ff @(posedge clk) begin
    if (clk_en) begin
    if (reset) begin
        state <= S_IDLE;
        init_set <= {SET_BITS{1'b0}};
        init_busy <= 1'b1;
        req_valid_r <= 1'b0;
        ready_r <= 1'b0;
        resp_valid_r <= 1'b0;
        line_r <= 128'h0;
        mem_valid_r <= 1'b0;
        mem_addr_r <= 32'h0;
        mem_burstcount_r <= 8'h0;
        mem_be_r <= 4'hF;
        fill_line <= 128'h0;
        fill_requested <= 1'b0;
        fill_stale <= 1'b0;
        snoop_tag_r <= {TAG_BITS{1'b0}};
        snoop_set_r <= {SET_BITS{1'b0}};
        req_snooped_r <= 1'b0;
        snoop_word_r <= {WORD_OFFSET_BITS{1'b0}};
        snoop_addr_dw_r <= 30'h0;
        snoop_data_r <= 32'h0;
        snoop_be_r <= 4'h0;
        snoop_patch_r <= 1'b0;
        snoop_valid_r <= 1'b0;
        patchq_head <= {PATCHQ_IDX_BITS{1'b0}};
        for (integer p = 0; p < PATCHQ_DEPTH; p = p + 1)
            patchq_valid[p] <= 1'b0;
    end else begin
        ready_r <= (state == S_IDLE);
        resp_valid_r <= 1'b0;
        snoop_valid_r <= snoop_valid;
        if (snoop_valid) begin
            snoop_tag_r <= snoop_tag;
            snoop_set_r <= snoop_set;
            snoop_word_r <= snoop_word;
            snoop_addr_dw_r <= snoop_addr[31:2];
            snoop_data_r <= snoop_data;
            snoop_be_r <= snoop_be;
            snoop_patch_r <= snoop_patch;
        end

        if (mem_valid_r && mem_ready)
            mem_valid_r <= 1'b0;

        if (snoop_valid_r) begin
            // CPU stores can race ahead of an instruction-cache line fill.
            // Keep the recent data-bearing snoops so a later fill of the same
            // physical line returns self-modified code after a branch flush.
            if (snoop_patch_r) begin
                for (int p = 0; p < PATCHQ_DEPTH; p++) begin
                    if (patchq_snoop_match[p]) begin
                        patchq_data[p] <= merge32(patchq_data[p], snoop_data_r, snoop_be_r);
                        patchq_be[p] <= patchq_be[p] | snoop_be_r;
                    end
                end
                if (!patchq_snoop_hit) begin
                    patchq_valid[patchq_head] <= 1'b1;
                    patchq_addr[patchq_head] <= snoop_addr_dw_r;
                    patchq_data[patchq_head] <= snoop_data_r;
                    patchq_be[patchq_head] <= snoop_be_r;
                    patchq_head <= patchq_next_idx(patchq_head);
                end
            end else begin
                for (int p = 0; p < PATCHQ_DEPTH; p++) begin
                    if (patchq_valid[p] && line_match_dw(patchq_addr[p], snoop_tag_r, snoop_set_r))
                        patchq_valid[p] <= 1'b0;
                end
            end
        end

        // Reset walk: one set per clock (the RAM clears ride on valid_we and
        // plru_we), alongside whatever uncached fetches the FSM carries.
        if (init_busy) begin
            if (init_set == LAST_SET) init_busy <= 1'b0;
            else                      init_set  <= init_set + 1'b1;
        end

        case (state)

            S_IDLE: begin
                if (accept_cpu) begin
                    ready_r <= 1'b0;
                    req_valid_r <= !cpu_uncacheable;
                    req_addr_r <= cpu_addr;
                    req_uncacheable_r <= cpu_uncacheable;
                    req_tag_r <= cpu_tag;
                    req_set_r <= cpu_set;
                    req_snooped_r <= snoop_valid_r && (snoop_tag_r == cpu_tag) &&
                                     (snoop_set_r == cpu_set);
                    if (cpu_uncacheable) begin
                        // Nothing to look up: straight to the bus.
                        fill_set <= cpu_set;
                        fill_tag <= cpu_tag;
                        fill_count <= cpu_addr[3:2];
                        fill_line <= 128'h0;
                        fill_requested <= direct_now;
                        if (direct_now) begin
                            mem_valid_r <= !mem_ready;
                            mem_addr_r <= mem_addr;
                            mem_burstcount_r <= direct_burst;
                            mem_be_r <= mem_be;
                        end
                        state <= S_FILL;
                    end else begin
                        state <= S_LOOKUP;
                    end
                end
            end

            S_LOOKUP: begin
                req_valid_r <= 1'b0;

                if (lookup_hit && !req_uncacheable_r) begin
                    state <= S_IDLE;
                    ready_r <= 1'b1;
                end else begin
                    fill_set <= req_set_r;
                    fill_tag <= req_tag_r;
                    fill_way <= plru_victim(rd_plru_r);
                    fill_plru_r <= rd_plru_r;
                    fill_count <= req_uncacheable_r ? req_addr_r[3:2] : {WORD_OFFSET_BITS{1'b0}};
                    fill_line <= 128'h0;
                    fill_requested <= 1'b0;
                    fill_stale <= 1'b0;
                    state <= S_FILL;
                end
            end

            S_FILL: if (cpu_abort && req_uncacheable_r) begin
                // The bus unit drops the fetch too, so nothing more arrives.
                mem_valid_r <= 1'b0;
                resp_valid_r <= 1'b1;
                state <= S_IDLE;
                ready_r <= 1'b1;
            end else begin
                if (!fill_requested && !mem_valid_r && !mem_busy) begin
                    mem_valid_r <= 1'b1;
                    fill_requested <= 1'b1;
                    if (req_uncacheable_r) begin
                        // From the target word to the end of the line only.
                        mem_addr_r <= {req_addr_r[31:2], 2'b00};
                        mem_burstcount_r <= 8'd4 - {6'd0, req_addr_r[3:2]};
                        mem_be_r <= req_addr_r[1] ? 4'b1100 : 4'b1111;
                    end else begin
                        mem_addr_r <= {req_addr_r[31:4], 4'b0000};
                        mem_burstcount_r <= 8'd4;
                        mem_be_r <= 4'b1111;
                    end
                end

                if (fill_snoop_now)
                    fill_stale <= 1'b1;

                // A store snoop between two words still lands in the line.
                fill_line <= fill_line_base;
                if (mem_resp_valid) begin
                    fill_line <= fill_line_next;

                    if (fill_count == {WORD_OFFSET_BITS{1'b1}}) begin
                        // A DMA write into this line during the fill leaves
                        // fetched words stale: hand the line on, keep it
                        // invalid.  The valid bit goes in through valid_we
                        // with the tag.  The other ways' valid bits are never
                        // restored from a fill-start snapshot: a snoop
                        // invalidation that landed during the fill
                        // (self-modifying code) must survive.
                        if (fill_validate) begin
                            write_cache_line(fill_way, fill_set, fill_line_next);
                            write_cache_tag(fill_way, fill_set, fill_tag);
                        end
                        line_r <= fill_line_next;
                        resp_valid_r <= 1'b1;
                        state <= S_IDLE;
                        ready_r <= 1'b1;
                    end
                    fill_count <= fill_count + 1'b1;
                end
            end

            S_BYPASS_WAIT: begin
                if (mem_resp_valid) begin
                    line_r <= {4{mem_dout}};
                    resp_valid_r <= 1'b1;
                    state <= S_IDLE;
                    ready_r <= 1'b1;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
    end
end

// synthesis translate_off
// The single write ports rely on these writers never sharing a clock, and
// the PLRU read on accept never landing on the edge of a PLRU write.
always_ff @(posedge clk) begin
    if (clk_en && !reset) begin
        if (init_busy && (fill_validate_now || plru_hit_we || plru_fill_we))
            $error("l1_icache: cache write during the reset walk");
        if (plru_hit_we && plru_fill_we)
            $error("l1_icache: PLRU hit and fill updates in one clock");
        if (accept_cpu && !init_busy && plru_we)
            $error("l1_icache: PLRU read and write on the same edge");
    end
end
// synthesis translate_on

endmodule

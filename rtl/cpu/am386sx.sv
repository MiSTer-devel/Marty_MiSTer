// Copyright (c) 2026 Jamie Blanks
//
// 386SX-pinned CPU: the z386 core behind a bus unit that turns its 32-bit
// requests into 16-bit SX bus cycles. One `ce` pulse is one CPU clock
// (16 MHz on the Marty), so every T-state below lasts one pulse.
//
//   z386 request ──> word cycles ──> T1 (ADS_n low)  ──> T2 ... T2 (READY_n high)
//   dword, be[3:0]    low word if be[1:0]                  ^ sampled at the end
//   burst of N        high word if be[3:2]                   of each T2
//
// The core's `ready` means accepted: it is pulsed when the request is
// captured, and the core posts its next request while this one runs.
// Reads are handed back one dword per resp_valid.
//
// Address pipelining follows the datasheet: NA_n sampled low in a T2 (or
// T1P) with another cycle pending puts that cycle's address, byte enables
// and definition on the pins one state early, with ADS_n low, in T2P; the
// cycle then opens with T1P once READY_n ends the current one. With
// nothing pending the cycle ends in T2I and the next one starts with a
// plain T1. The Marty ties NA_n high, so it only ever sees T1/T2.
//
// Code is fetched the way the SX prefetcher does it: one word per idle bus
// slot, sequentially, into an 8-word stream that stands in for its 16-byte
// queue. A code request from the core attaches to the stream at its first
// word (restarting it after a jump) and is answered one dword per clock as
// the words come in; a data cycle from the core waits only for the word
// cycle in progress. A fetched word is kept only while it is still the
// next one the stream wants, so a write or a DMA snoop that lands on
// fetched-ahead words, or the core discarding its fetch (which then owes
// no more responses), drops them and the fetch in flight. An I/O write
// keeps the stream, as on the chip: code that switches a ROM bank through
// I/O has to jump before it runs from the new bank. The code fetched ahead of the decoder -
// the core's queue, the words on their way to it and the stream - is held
// to the 16 bytes the SX queue holds, counting the word about to be
// fetched.
// The core asks for the two acknowledge cycles itself (address 4, then
// address 0 with the vector on D7-D0); each is one INTA cycle here and
// LOCK_n stays low from the first to the end of the second, and HOLD
// waits for it. The core also marks locked cycles (XCHG with memory, the
// LOCK prefix, descriptor and page-table bit updates) and flags a request
// that another cycle of the same transfer follows: the other part of an
// access split over two dwords, or the write of a locked read-modify-
// write. LOCK_n then stays low until that cycle is done and HOLD waits
// for it. Otherwise HOLD is granted between cycles and floats every
// output but HLDA; when HOLD drops, HLDA falls and the next T1 begins on
// the same clock.
// HLT runs one halt indication cycle (M/IO high, D/C low, W/R high,
// address 2, BLE_n only) once the core's queued writes are out, and a
// shutdown runs the same cycle at address 0. Neither fetches code until
// the core is woken. During RESET the pins sit at the datasheet's idle
// levels: address all ones, both byte enables low, W/R low, D/C high,
// M/IO low, LOCK_n and ADS_n high, data floating.
//
// Not modelled: the coprocessor pins.

module am386sx
#(
	parameter BOOT_UCODE = 1    // run the ROM's reset routine after RESET (0: a bench forces the start state)
)
(
	input             clk,
	input             ce,           // one CPU clock per pulse
	input             cache_enable, // z386 L1 caches (off on a real SX)
	input       [1:0] ram_size,     // fitted DRAM: 0 2 MB, 1 4 MB, 2 6 MB, 3 8 MB
	input             RESET,

	output     [23:1] A,
	output            BHE_n,
	output            BLE_n,
	output            ADS_n,
	output            W_R_n,
	output            D_C_n,
	output            M_IO_n,
	output            LOCK_n,
	output            bus_oe,       // address and control pins driven (low in hold acknowledge)

	input      [15:0] D_i,
	output     [15:0] D_o,
	output            D_oe,

	input             READY_n,
	input             NA_n,
	input             HOLD,
	output reg        HLDA,
	input             INTR,
	input             NMI,
	output            SHUTDOWN,     // a third fault while delivering #DF; the core stops until RESET
	output            PE,           // CR0.PE: protected mode (not a pin)
	output            PG,           // CR0.PG: paging (not a pin)

	input      [23:1] snoop_addr,   // a bus master wrote this word: drop it from the caches
	input             snoop_valid,

	output     [15:0] dbg_CS,
	output    [255:0] dbg_gpr,
	output     [95:0] dbg_seg,
	output     [31:0] dbg_EIP,      // of the next instruction, valid with dbg_retire
	output            dbg_retire,  // one pulse per completed instruction

	// savestate: park at the next instruction boundary, read the state
	input             ss_stop,
	output            ss_halted,     // parked
	output            ss_quiet,      // parked with no write or bus cycle left
	output            ss_in_hlt,     // the park ended a HLT
	input       [5:0] ss_state_sel,
	output     [31:0] ss_state
);

// ---- core ----
// RESET is taken on the T-state enable, so it reaches every register here
// with the same timing budget as any other CPU signal; recognition is one
// T-state late, well inside the 15 CLK2 the datasheet asks for.
reg rst_q, cache_en_q;
reg [1:0] ram_size_q;
always @(posedge clk) if (ce) begin
	rst_q      <= RESET;
	cache_en_q <= cache_enable;   // taken on the enable like RESET; may change at run time
	ram_size_q <= ram_size;
end

wire [31:2] c_addr;
wire  [3:0] c_be;
wire  [7:0] c_burst;
wire [31:0] c_dout;
wire        c_valid, c_write, c_io, c_code, c_code_abort, c_inta;
wire        c_lock, c_more, c_halt;
wire  [5:0] c_queued;
reg  [31:0] c_din;
reg         c_ready, c_resp;
wire [31:0] cur_eip, next_eip;
wire        c_store_pending;
wire [31:0] core_state;
// The park lets the next instruction pop before it stops, so the core's
// EIP already points past it; the boundary is where the last retire left it.
reg  [31:0] boundary_eip;
always @(posedge clk) if (ce && dbg_retire) boundary_eip <= next_eip;
assign ss_state = (ss_state_sel == 6'd2) ? boundary_eip : core_state;

z386 #(.BUS_DATA_FIRST(1), .PHYS_ADDR_BITS(24), .BOOT_UCODE(BOOT_UCODE)) core
(
	.clk(clk),
	.clk_en(ce),
	.cache_enable(cache_en_q),
	.ram_size(ram_size_q),
	.reset_n(~rst_q),
	.addr(c_addr),
	.be(c_be),
	.burstcount(c_burst),
	.din(c_din),
	.dout(c_dout),
	.valid(c_valid),
	.ready(c_ready),
	.write(c_write),
	.io(c_io),
	.code(c_code),
	.code_abort(c_code_abort),
	.lock(c_lock),
	.more(c_more),
	.resp_valid(c_resp),
	.intr(INTR),
	.nmi(NMI),
	.inta(c_inta),
	.shutdown(SHUTDOWN),
	.halt(c_halt),
	.code_queued(c_queued),
	.snoop_addr({8'd0, snoop_addr, 1'b0}),
	.snoop_valid(snoop_valid),
	.single_step(ss_stop),
	.dbg_CS(dbg_CS),
	.dbg_EIP(cur_eip),
	.dbg_CS_base(),
	.dbg_pe(PE),
	.dbg_pg(PG),
	.dbg_vm(),
	.dbg_retire(dbg_retire),
	.dbg_next_eip(next_eip),
	.dbg_gpr(dbg_gpr),
	.dbg_seg(dbg_seg),
	.dbg_halted(ss_halted),
	.dbg_stopped_in_hlt(ss_in_hlt),
	.dbg_state_sel(ss_state_sel),
	.dbg_state(core_state),
	.dbg_store_pending(c_store_pending)
);

// bench-only: the retire-time EIP costs a 32-bit mux the hardware never reads
`ifdef VERILATOR
assign dbg_EIP = dbg_retire ? next_eip : cur_eip;
`else
assign dbg_EIP = 32'd0;
`endif

// ---- bus unit ----
localparam [2:0] TI = 3'd0, T1 = 3'd1, T2 = 3'd2, TH = 3'd3, T2I = 3'd4, T2P = 3'd5, T1P = 3'd6;
// what the next cycle on the bus will be
localparam [2:0] NX_NONE = 3'd0, NX_WORD = 3'd1, NX_REQ = 3'd2, NX_CODE = 3'd3, NX_HALT = 3'd4;

reg  [2:0] state;
reg  [2:0] nx;             // pipelined next cycle, on the pins from T2P until it starts
reg        inta_lock;      // between the two acknowledge cycles
reg        seq_hold;       // the last data cycle said another follows: no HOLD
reg        lock_hold;      // ... and it was locked: keep LOCK_n low meanwhile
reg        halt_req;       // a halt or shutdown indication cycle is owed
reg        halt_d;
reg  [1:0] quiet;          // clocks without a data request, saturating

// Data, I/O or acknowledge request captured from the core.
reg [31:2] r_addr;
reg  [3:0] r_be;
reg  [7:0] r_left;         // dwords still to move
reg [31:0] r_data;         // write data of the current dword / read assembly
reg        r_write, r_io, r_inta, r_lock, r_more;
reg        busy;           // a request is captured and not yet finished
reg        hi_word;        // current word cycle is the upper half
reg        cyc_halt;       // the cycle on the bus is a halt or shutdown indication

// Code request from the core, answered from the stream.
reg [23:2] code_addr;
reg  [7:0] code_left;      // dwords still owed
reg        code_busy;
// a code stream left waiting is dropped by the reset that follows; data must be through
assign ss_quiet = ss_halted && !c_store_pending && !busy && !(c_valid && !c_code);
reg        code_hi_only;   // the first dword wants only its upper word

// Code stream: sb_head is the address of the oldest word, the next fetch
// goes to sb_head + sb_count. Words are delivered from the head.
(* ramstyle = "logic" *) reg [15:0] sb_data [0:7];   // a word may be pushed and another delivered in one clock; keep flop semantics
reg [23:1] sb_head;
reg  [2:0] sb_rd;
reg  [3:0] sb_count;
reg        sb_valid;       // fetch ahead from sb_head
reg        cyc_code;       // the cycle on the bus fetches a stream word
reg [23:1] f_addr;         // address of that word
reg [23:1] f_next;         // address of the pipelined fetch behind it
// The pipelined data request, captured when it goes on the pins so the
// pins never depend on the core's live request.
reg [31:2] n_addr;
reg  [3:0] n_be;
reg        n_write, n_io, n_inta, n_lock, n_hi;
reg  [1:0] tr1;            // words delivered one clock ago

wire hi_needed  = r_be[2] | r_be[3];
wire in_cycle   = (state != TI) && (state != TH);
wire t_last     = (state == T2) || (state == T2I) || (state == T2P);   // READY_n is sampled
wire cyc_done   = t_last && !READY_n;
wire word_last  = hi_word | ~hi_needed;                                // last word of this dword
wire more_words = busy && !r_inta && !cyc_code && !cyc_halt && (!word_last || (r_left != 8'd1));

// A code request whose first word is the stream head continues it.
wire        attach     = c_valid && c_code && !code_busy;
wire        c_hi_only  = ~(c_be[0] | c_be[1]);
wire [23:1] first_word = {c_addr[23:2], c_hi_only};
wire        restart    = attach && !c_code_abort && !(sb_valid && sb_head == first_word);
wire        drop       = c_code_abort && (code_busy || attach);
wire [1:0]  need       = code_hi_only ? 2'd1 : 2'd2;
// The core sorts responses by kind, data first: no code word goes back
// while a data read is posted or running.
wire        data_rd    = (c_valid && !c_code && !c_write) || (busy && !r_write);
wire        deliver    = code_busy && !c_code_abort && !data_rd && (sb_count >= {2'd0, need});

// Writes that land on fetched-ahead words make them stale. The stream is
// worked out as if the cycle on the bus completes now (a write hits, a
// fetched word is pushed); READY_n only decides whether that view is
// committed, so the fetch-ahead arithmetic never waits on the pin.
wire        wr_cyc  = in_cycle && !cyc_code && !cyc_halt && r_write && !r_inta && !r_io;
wire [23:1] wr_off  = {r_addr[23:2], hi_word} - sb_head;
wire [23:1] sn_off  = snoop_addr - sb_head;
wire        wr_hit_d = wr_cyc && (wr_off < {19'd0, sb_count});
wire        sn_hit   = snoop_valid && (sn_off < {19'd0, sb_count});

logic [23:1] sb_head_n;
logic  [3:0] sb_count_d;    // the count if the cycle completes now
logic  [3:0] sb_count_n;    // the count committed this clock
logic  [2:0] sb_rd_n;
logic  [2:0] sb_wr;
logic        sb_valid_n;
logic        push_d;        // the fetch on the bus is still wanted
logic        push;

always_comb begin
	sb_head_n  = sb_head;
	sb_count_d = sb_count;
	sb_rd_n    = sb_rd;
	sb_valid_n = sb_valid;
	if (deliver) begin
		sb_head_n  = sb_head_n + {20'd0, need};
		sb_count_d = sb_count_d - {2'd0, need};
		sb_rd_n    = sb_rd_n + {1'b0, need};
	end
	sb_count_n = sb_count_d;
	if (sn_hit)   begin sb_count_d = 4'd0; sb_count_n = 4'd0; end
	if (wr_hit_d) sb_count_d = 4'd0;
	if (drop) begin
		sb_valid_n = 0;
		sb_count_d = 4'd0;
		sb_count_n = 4'd0;
	end
	if (restart) begin
		sb_valid_n = 1;
		sb_head_n  = first_word;
		sb_count_d = 4'd0;
		sb_count_n = 4'd0;
		sb_rd_n    = 3'd0;
	end
	// A fetched word is kept only if it is still the one the stream wants next.
	sb_wr  = sb_rd_n + sb_count_d[2:0];
	push_d = in_cycle && cyc_code && sb_valid_n && (f_addr == sb_head_n + {19'd0, sb_count_d});
	if (push_d) sb_count_d = sb_count_d + 4'd1;
	push   = cyc_done && push_d;
	if (cyc_done) sb_count_n = sb_count_d;
end

wire [23:1] sb_tail_n = sb_head_n + {19'd0, sb_count_n};
wire [23:1] sb_tail_d = sb_head_n + {19'd0, sb_count_d};
// The fetch after the one on the bus (or the next one, when the bus is not fetching).
wire [23:1] f_cand    = sb_tail_d;
wire [3:0]  ahead     = sb_count_d;
// Code fetched but not yet decoded, as the SX's 16-byte queue would hold
// it: the core's queue, the dwords on their way there (a delivery shows
// up in the count two clocks later), and the stream. A fetch starts only
// when its word fits.
wire [1:0]  tr0        = deliver ? need : 2'd0;
wire [6:0]  look_ahead = {1'b0, c_queued} + {4'd0, tr0, 1'b0} + {4'd0, tr1, 1'b0} + {2'd0, ahead, 1'b0};
wire        halt_lvl  = c_halt | SHUTDOWN;
wire        can_fetch = sb_valid_n && (look_ahead <= 7'd14) && !inta_lock && !halt_req && !halt_lvl;

// A data request; the one just accepted stays on the port one more clock.
wire c_req = c_valid && !c_code && !c_ready;

// The halt indication goes out once the core's writes have all reached
// the bus: a few quiet clocks cover the gaps between queued stores.
wire data_active = c_req || busy;
wire halt_ok     = halt_req && !data_active && (quiet == 2'd3);
wire hold_wins   = HOLD && !inta_lock && !seq_hold && !(busy && r_more) && !halt_ok;

// Next cycle: the rest of the current request, then HOLD, then the halt
// indication, a data request, a stream word.
logic [2:0] cand;
always_comb begin
	if (busy && r_inta)          cand = NX_NONE;
	else if (more_words)         cand = NX_WORD;
	else if (hold_wins)          cand = NX_NONE;
	else if (halt_ok)            cand = NX_HALT;
	else if (c_req)              cand = NX_REQ;
	else if (can_fetch)          cand = NX_CODE;
	else                         cand = NX_NONE;
end

// Pins show the cycle being addressed: the pipelined next one when there
// is one, else the cycle in progress (or the last one, while idle).
logic [23:1] p_a;
logic        p_bhe, p_ble, p_wr, p_dc, p_mio, p_lock;
always_comb begin
	p_lock = 0;
	case (nx)
	NX_WORD: begin
		p_a   = word_last ? {r_addr[23:2] + 22'd1, 1'b0} : {r_addr[23:2], 1'b1};
		p_bhe = word_last ? ~r_be[1] : ~r_be[3];
		p_ble = word_last ? ~r_be[0] : ~r_be[2];
		p_wr  = r_write; p_dc = 1; p_mio = ~r_io; p_lock = r_lock;
	end
	NX_REQ: if (n_inta) begin
		p_a = {21'd0, n_addr[2], 1'b0}; p_bhe = 1; p_ble = 0; p_wr = 0; p_dc = 0; p_mio = 0; p_lock = 1;
	end else begin
		p_a   = {n_addr[23:2], n_hi};
		p_bhe = n_hi ? ~n_be[3] : ~n_be[1];
		p_ble = n_hi ? ~n_be[2] : ~n_be[0];
		p_wr  = n_write; p_dc = 1; p_mio = ~n_io; p_lock = n_lock;
	end
	NX_CODE: begin p_a = f_next; p_bhe = 0; p_ble = 0; p_wr = 0; p_dc = 0; p_mio = 1; end
	NX_HALT: begin p_a = {22'd0, ~SHUTDOWN}; p_bhe = 1; p_ble = 0; p_wr = 1; p_dc = 0; p_mio = 1; end
	default: if (cyc_code) begin
		p_a = f_addr; p_bhe = 0; p_ble = 0; p_wr = 0; p_dc = 0; p_mio = 1;
	end else if (cyc_halt) begin
		p_a = {22'd0, ~SHUTDOWN}; p_bhe = 1; p_ble = 0; p_wr = 1; p_dc = 0; p_mio = 1;
	end else if (r_inta) begin
		p_a = {21'd0, r_addr[2], 1'b0}; p_bhe = 1; p_ble = 0; p_wr = 0; p_dc = 0; p_mio = 0;
	end else begin
		p_a   = {r_addr[23:2], hi_word};
		p_bhe = hi_word ? ~r_be[3] : ~r_be[1];
		p_ble = hi_word ? ~r_be[2] : ~r_be[0];
		p_wr  = r_write; p_dc = 1; p_mio = ~r_io;
	end
	endcase
end

assign ADS_n  = ~((state == T1) || (state == T2P));
assign A      = p_a;
assign BHE_n  = p_bhe;
assign BLE_n  = p_ble;
assign W_R_n  = p_wr;
assign D_C_n  = p_dc;
assign M_IO_n = p_mio;
assign LOCK_n = ~(((r_inta | r_lock) & busy) | inta_lock | lock_hold | p_lock);
assign bus_oe = (state != TH);
assign D_o    = hi_word ? r_data[31:16] : r_data[15:0];
assign D_oe   = in_cycle & (cyc_halt | (~cyc_code & r_write & ~r_inta));

// Begin a cycle: in T1, or in T1P when its address already went out.
task automatic start(input [2:0] kind, input logic piped);
	case (kind)
	NX_WORD: begin
		if (word_last) begin
			r_left  <= r_left - 8'd1;
			r_addr  <= r_addr + 30'd1;
			hi_word <= 0;
		end
		else hi_word <= 1;
		state <= piped ? T1P : T1;
	end
	NX_REQ: begin
		c_ready   <= 1;
		busy      <= 1;
		r_addr    <= piped ? n_addr  : c_addr;
		r_be      <= piped ? n_be    : c_be;
		r_left    <= (c_burst == 0) ? 8'd1 : c_burst;
		r_data    <= c_dout;
		r_write   <= piped ? n_write : c_write;
		r_io      <= piped ? n_io    : c_io;
		r_inta    <= piped ? n_inta  : c_inta;
		r_lock    <= piped ? n_lock  : c_lock;
		r_more    <= c_more;
		hi_word   <= piped ? n_hi    : ~(c_be[0] | c_be[1]);
		cyc_code  <= 0;
		cyc_halt  <= 0;
		seq_hold  <= 0;
		lock_hold <= 0;
		state     <= piped ? T1P : T1;
	end
	NX_CODE: begin
		f_addr   <= piped ? f_next : sb_tail_d;
		cyc_code <= 1;
		cyc_halt <= 0;
		state    <= piped ? T1P : T1;
	end
	NX_HALT: begin
		cyc_code  <= 0;
		cyc_halt  <= 1;
		halt_req  <= 0;
		seq_hold  <= 0;   // nothing follows a parked core
		lock_hold <= 0;
		state    <= piped ? T1P : T1;
	end
	default: state <= TI;
	endcase
endtask

// Next bus slot with nothing pipelined: the candidate, HOLD, or idle.
task automatic launch();
	if (cand != NX_NONE) start(cand, 1'b0);
	else if (hold_wins) begin
		state <= TH;
		HLDA  <= 1;
	end
	else state <= TI;
endtask

// The cycle just acknowledged is over: the pipelined one starts, else launch.
task automatic next_cycle();
	if (nx != NX_NONE) begin
		start(nx, 1'b1);
		nx <= NX_NONE;
	end
	else launch();
endtask

// NA_n sampled low: put the next address out (T2P) or wait for one (T2I).
task automatic pipeline();
	if (cand != NX_NONE) begin
		nx     <= cand;
		f_next <= f_cand;
		if (cand == NX_REQ) begin
			n_addr  <= c_addr;
			n_be    <= c_be;
			n_write <= c_write;
			n_io    <= c_io;
			n_inta  <= c_inta;
			n_lock  <= c_lock;
			n_hi    <= c_hi_only;
		end
		state  <= T2P;
	end
	else state <= T2I;
endtask

// READY_n sampled low at the end of a T2 state.
task automatic finish();
	if (cyc_code) next_cycle();
	else if (cyc_halt) begin
		cyc_halt <= 0;
		next_cycle();
	end
	else if (r_inta) begin
		c_din     <= {24'd0, D_i[7:0]};
		c_resp    <= 1;
		inta_lock <= r_addr[2];   // held after the first cycle, released by the second
		busy      <= 0;
		state     <= TI;
	end
	else begin
		if (!r_write) begin
			if (!word_last) r_data[15:0] <= D_i;
			else begin
				c_din  <= hi_word ? {D_i, r_data[15:0]} : {r_data[31:16], D_i};
				c_resp <= 1;
			end
		end
		if (word_last && r_left == 8'd1) begin
			busy      <= 0;
			seq_hold  <= r_more;
			lock_hold <= r_more & r_lock;
		end
		next_cycle();
	end
endtask

always @(posedge clk) begin
	if (rst_q) begin
		// idle bus levels: address all ones, both byte enables on, read, control, I/O
		state <= TI;
		nx <= NX_NONE;
		busy <= 0;
		c_ready <= 0;
		c_resp <= 0;
		HLDA <= 0;
		r_left <= 0;
		inta_lock <= 0;
		seq_hold <= 0;
		lock_hold <= 0;
		halt_req <= 0;
		halt_d <= 0;
		quiet <= 0;
		r_inta <= 0;
		r_lock <= 0;
		r_more <= 0;
		r_write <= 0;
		r_io <= 1;
		r_be <= 4'hF;
		r_addr <= {30{1'b1}};
		hi_word <= 1;
		cyc_halt <= 0;
		code_busy <= 0;
		code_left <= 0;
		code_addr <= 22'd0;
		code_hi_only <= 0;
		sb_head <= 23'd0;
		sb_rd <= 0;
		sb_count <= 0;
		sb_valid <= 0;
		cyc_code <= 0;
		f_addr <= 23'd0;
		f_next <= 23'd0;
		n_addr <= 30'd0; n_be <= 4'd0; n_write <= 0; n_io <= 0; n_inta <= 0; n_lock <= 0; n_hi <= 0;
		tr1 <= 0;
	end
	else if (ce) begin
		c_ready <= 0;
		c_resp <= 0;

		// ---- halt ----
		halt_d <= halt_lvl;
		if (halt_lvl && !halt_d) halt_req <= 1;
		if (data_active) quiet <= 0;
		else if (quiet != 2'd3) quiet <= quiet + 2'd1;

		// ---- code stream ----
		if (push) sb_data[sb_wr] <= D_i;
		tr1 <= tr0;
		sb_head  <= sb_head_n;
		sb_count <= sb_count_n;
		sb_rd    <= sb_rd_n;
		sb_valid <= sb_valid_n;

		// Code responses never meet a data read: the core holds one kind of
		// read at a time.
		if (attach) begin
			c_ready      <= 1;
			code_busy    <= 1;
			code_addr    <= c_addr[23:2];
			code_left    <= (c_burst == 0) ? 8'd1 : c_burst;
			code_hi_only <= c_hi_only;
		end
		// A discarded fetch owes nothing more.
		if (drop) begin
			code_busy <= 0;
			code_left <= 0;
		end
		if (deliver) begin
			c_din <= code_hi_only ? {sb_data[sb_rd], sb_data[sb_rd]}
			                      : {sb_data[sb_rd + 3'd1], sb_data[sb_rd]};
			c_resp       <= 1;
			code_addr    <= code_addr + 22'd1;
			code_left    <= code_left - 8'd1;
			code_hi_only <= 0;
			if (code_left == 8'd1) code_busy <= 0;
		end

		// ---- bus cycles ----
		//        NA_n high            NA_n low, next pending      NA_n low, nothing
		//   T1 -> T2 -> T2 ...        T2 -> T2P (ADS_n low)       T2 -> T2I
		//         READY_n: T1/TI/TH   READY_n: T1P -> T2P/T2/T2I  READY_n: T1/TI/TH
		case (state)
		TI: launch();

		TH: if (!HOLD) begin
			HLDA <= 0;
			launch();
		end

		T1: state <= T2;

		T2: if (!READY_n) finish();
		    else if (!NA_n) pipeline();

		T2I: if (!READY_n) finish();
		     else pipeline();

		T2P: if (!READY_n) finish();

		T1P: if (!NA_n) pipeline();
		     else state <= T2;

		default: state <= TI;
		endcase
	end
end

endmodule

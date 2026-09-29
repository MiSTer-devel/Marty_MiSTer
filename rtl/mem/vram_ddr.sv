// Copyright (c) 2026 Jamie Blanks
//
// The CRTC's line fetch served from a DDR3 copy of VRAM, so the SDRAM
// only carries the CPU and the VRAM RAM port.
//
//   VRAM port write ──> FIFO ──> one 64-bit DDR3 beat with byte enables
//   CRTC vf request ──> slot lookup ──> hit: answer next clock
//                                   miss: 8-word DDR3 burst into a slot
//
// The mirror holds VRAM's 512 KB at BASE. Two 8-word slots serve the
// fetch: a request that misses both allocates the older one with a burst
// from its own address, and once the head request is past the middle of
// its slot the other slot prefetches the next 8 words, so a line streams
// with the next burst under way. Answers keep the fetch port's contract:
// a req pulse while accept is high, in-order ready pulses with dout. A
// mirrored write that lands in a slot's range drops the slot.
//
// The framework's DDR3 port is shared: the savestate engine goes first,
// then mirror writes, then fetch reads. A burst waits for the write FIFO
// to drain so a fetched line sees the writes before it, unless it has
// waited 512 clocks, when the writes keep coming faster than the beam.
// `wr_full` holds the VRAM port's ready low while the FIFO is nearly full.

module vram_ddr #(
	parameter [28:0] BASE = 29'h07E00000   // 64-bit word address of the mirror: byte 0x3F000000, 512 KB aligned
)
(
	input             clk,
	input             reset,

	// VRAM port writes to mirror, the clock the port takes them
	input             wr_req,
	input      [18:1] wr_a,
	input       [1:0] wr_be,
	input      [15:0] wr_din,
	output            wr_full,

	// CRTC line fetch
	input             vf_req,
	input      [18:3] vf_a,
	output            vf_accept,
	output reg [63:0] vf_dout,
	output reg        vf_ready,

	// savestate engine, ahead of everything: its pins pass through while
	// nothing of ours is on the bus
	input             ss_we,
	input             ss_rd,
	input      [28:0] ss_addr,
	input      [63:0] ss_din,
	input       [7:0] ss_be,
	output            ss_busy,        // the DDR3 port as the engine sees it
	output            ss_dout_ready,

	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output     [28:0] DDRAM_ADDR,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output            DDRAM_WE,
	output            DDRAM_RD,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY
);

// ---- write mirror FIFO: {a[18:1], be, din} ----
reg  [8:0] fw, fr;             // write and read pointers
reg  [9:0] fcount;
reg  [1:0] f_age;              // clocks the head entry has been readable
reg        f_pop;
wire       f_empty = (fcount == 10'd0);
assign     wr_full = (fcount > 10'd480);
wire [35:0] f_q;
cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(36)) fifo
(
	.clk_i(clk),
	.addr_a_i(fw), .wren_a_i(wr_req), .wdata_a_i({wr_a, wr_be, wr_din}), .q_a_o(),
	.addr_b_i(fr), .wren_b_i(1'b0),   .wdata_b_i(36'd0),                 .q_b_o(f_q)
);
wire [18:1] f_a   = f_q[35:18];
wire  [1:0] f_be  = f_q[17:16];
wire [15:0] f_din = f_q[15:0];
wire        f_ok  = !f_empty && (f_age == 2'd2) && !f_pop;   // block RAM output settled on the head, count updated

// ---- fetch request queue: two outstanding, answered in order ----
reg [18:3] q_addr [0:1];
reg  [1:0] q_valid;
reg        wp, rp;
assign vf_accept = !(q_valid[0] && q_valid[1]);
wire [18:3] head_a = q_addr[rp];
wire        head_v = q_valid[rp];

// ---- burst slots: 8 words each, in one 16-entry block RAM ----
reg [18:3] s_base [0:1];
reg  [1:0] s_valid;            // slot owns an address range
reg  [3:0] s_have [0:1];       // words landed, 0..8
reg        s_fill;             // a burst is landing
reg        s_dirty;            // a write hit the landing slot: drop it when the burst ends
reg        s_old;              // slot to replace next
reg  [3:0] rd_a;
wire [63:0] rd_q;
reg  [3:0] wr_slot_a;
reg        wr_slot_we;
reg [63:0] wr_slot_d;
cache_ram_dp #(.ADDR_WIDTH(4), .DATA_WIDTH(64)) slots
(
	.clk_i(clk),
	.addr_a_i(wr_slot_a), .wren_a_i(wr_slot_we), .wdata_a_i(wr_slot_d), .q_a_o(),
	.addr_b_i(rd_a),      .wren_b_i(1'b0),       .wdata_b_i(64'd0),     .q_b_o(rd_q)
);

// where the head request lives
wire [18:3] d0 = head_a - s_base[0];
wire [18:3] d1 = head_a - s_base[1];
wire        in0 = s_valid[0] && (d0[18:6] == 13'd0);   // within the slot's 8 words
wire        in1 = s_valid[1] && (d1[18:6] == 13'd0);
wire        hit0 = in0 && (s_have[0][3] || d0[5:3] < s_have[0][2:0]);
wire        hit1 = in1 && (s_have[1][3] || d1[5:3] < s_have[1][2:0]);
wire        hit  = head_v && (hit0 || hit1);
wire        miss = head_v && !in0 && !in1;            // needs a burst of its own
// prefetch: the head is in the second half of its slot and the next range is not held
wire [18:3] next0 = s_base[0] + 16'd8;
wire [18:3] next1 = s_base[1] + 16'd8;
wire        pf0 = in0 && d0[5] && !(s_valid[1] && s_base[1] == next0);
wire        pf1 = in1 && d1[5] && !(s_valid[0] && s_base[0] == next1);
// a mirrored write into a slot's range
wire [18:3] w0 = wr_a[18:3] - s_base[0];
wire [18:3] w1 = wr_a[18:3] - s_base[1];
wire        w_in0 = wr_req && s_valid[0] && (w0[18:6] == 13'd0);
wire        w_in1 = wr_req && s_valid[1] && (w1[18:6] == 13'd0);

// ---- DDR3 sequencer ----
localparam [2:0] D_IDLE = 3'd0, D_SS = 3'd1, D_WRITE = 3'd2, D_READ = 3'd3, D_BURST = 3'd4;
reg  [2:0] dstate;
reg        burst_slot;
reg  [2:0] burst_n;
reg  [8:0] rd_wait;            // clocks a fetch has waited behind writes
reg [28:0] my_addr;
reg [63:0] my_din;
reg  [7:0] my_be;
reg        my_we, my_rd;
wire       ss_want   = ss_we || ss_rd;
wire       want_read = (miss || pf0 || pf1) && !s_fill;
wire       read_ok   = f_empty || rd_wait[8];
wire       eng       = (dstate == D_IDLE) || (dstate == D_SS);   // the engine has the pins
assign ss_busy       = DDRAM_BUSY || !eng;
assign ss_dout_ready = DDRAM_DOUT_READY && (dstate == D_SS);
assign DDRAM_ADDR     = eng ? ss_addr : my_addr;
assign DDRAM_DIN      = eng ? ss_din  : my_din;
assign DDRAM_BE       = eng ? ss_be   : my_be;
assign DDRAM_WE       = eng ? ss_we   : my_we;
assign DDRAM_RD       = eng ? ss_rd   : my_rd;
assign DDRAM_BURSTCNT = my_rd ? 8'd8 : 8'd1;

reg [1:0] answer_q;            // block RAM address in, then its output a clock later

always @(posedge clk) begin
	vf_ready   <= 0;
	wr_slot_we <= 0;
	f_pop      <= 0;
	if (reset) begin
		fw <= 0; fr <= 0; fcount <= 0; f_age <= 0;
		q_valid <= 0; wp <= 0; rp <= 0; answer_q <= 0;
		s_valid <= 0; s_fill <= 0; s_old <= 0; s_dirty <= 0;
		s_have[0] <= 0; s_have[1] <= 0;
		dstate <= D_IDLE; my_we <= 0; my_rd <= 0;
		rd_wait <= 0;
	end
	else begin
		// FIFO in and out
		if (wr_req) fw <= fw + 1'd1;
		if (wr_req && !f_pop) fcount <= fcount + 1'd1;
		else if (!wr_req && f_pop) fcount <= fcount - 1'd1;
		if (f_pop || f_empty) f_age <= 0;
		else if (f_age != 2'd2) f_age <= f_age + 1'd1;

		// requests in
		if (vf_req && vf_accept) begin
			q_addr[wp]  <= vf_a;
			q_valid[wp] <= 1;
			wp <= ~wp;
		end

		// answer the head from a slot: address the RAM, wait for its output, hand over
		answer_q <= {answer_q[0], 1'b0};
		if (hit && answer_q == 2'd0) begin
			rd_a <= hit0 ? {1'b0, d0[5:3]} : {1'b1, d1[5:3]};
			answer_q[0] <= 1;
		end
		if (answer_q[1]) begin
			vf_dout <= rd_q;
			vf_ready <= 1;
			q_valid[rp] <= 0;
			rp <= ~rp;
		end

		// a write into a held range makes the slot stale
		if (w_in0 && !(s_fill && burst_slot == 1'b0)) s_valid[0] <= 0;
		if (w_in1 && !(s_fill && burst_slot == 1'b1)) s_valid[1] <= 0;
		if ((w_in0 && burst_slot == 1'b0) || (w_in1 && burst_slot == 1'b1)) s_dirty <= s_fill;

		if (want_read && !read_ok) rd_wait <= rd_wait + 1'd1;
		else if (!want_read) rd_wait <= 0;

		case (dstate)
		D_IDLE: begin
			if (ss_want) begin
				// the engine's pins are on the bus; follow its read
				if (ss_rd && !DDRAM_BUSY) dstate <= D_SS;
			end
			else if (f_ok) begin
				my_addr <= BASE + {13'd0, f_a[18:3]};
				my_din  <= {4{f_din}};
				my_be   <= {6'd0, f_be} << {f_a[2:1], 1'b0};
				my_we   <= 1;
				dstate  <= D_WRITE;
			end
			else if (want_read && read_ok) begin
				burst_slot <= miss ? s_old : pf0 ? 1'b1 : 1'b0;
				my_addr <= BASE + {13'd0, miss ? head_a : pf0 ? next0 : next1};
				my_rd   <= 1;
				dstate  <= D_READ;
				rd_wait <= 0;
			end
		end
		D_SS: if (DDRAM_DOUT_READY) dstate <= D_IDLE;   // the engine reads one word at a time
		D_WRITE: if (!DDRAM_BUSY) begin
			my_we  <= 0;
			fr     <= fr + 1'd1;
			f_pop  <= 1;
			dstate <= D_IDLE;
		end
		D_READ: if (!DDRAM_BUSY) begin
			my_rd <= 0;
			s_base[burst_slot]  <= my_addr[15:0];
			s_valid[burst_slot] <= 1;
			s_have[burst_slot]  <= 0;
			s_fill  <= 1;
			s_dirty <= 0;
			s_old   <= ~burst_slot;
			burst_n <= 0;
			dstate  <= D_BURST;
		end
		D_BURST: if (DDRAM_DOUT_READY) begin
			wr_slot_a  <= {burst_slot, burst_n};
			wr_slot_d  <= DDRAM_DOUT;
			wr_slot_we <= 1;
			burst_n <= burst_n + 1'd1;
			if (burst_n == 3'd7) begin
				s_fill <= 0;
				if (s_dirty) s_valid[burst_slot] <= 0;
				dstate <= D_IDLE;
			end
		end
		default: dstate <= D_IDLE;
		endcase
		// a landed word counts the clock after it is written
		if (wr_slot_we) s_have[wr_slot_a[3]] <= s_have[wr_slot_a[3]] + 1'd1;
	end
end

endmodule

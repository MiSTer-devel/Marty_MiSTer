// Copyright (c) 2026 Jamie Blanks
//
// Intel 8253 programmable interval timer: three independent 16-bit down
// counters behind a byte bus. Each counter clock arrives as a one-cycle
// enable marking the falling CLK edge; gate levels are sampled at that
// edge, gate rising edges are caught between edges.
//
//   CPU write ──> control word / count bytes ──> counter n ──> OUT n
//                                                 ^ CLK n enable, GATE n
//
// A write takes effect on the rising edge of WR_n, a read drives D while
// RD_n is low and advances its byte pointer when RD_n rises.

module i8253
(
	input             clk,
	input             reset,

	input       [1:0] a,
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [7:0] d_i,
	output      [7:0] d_o,
	output            d_oe,

	input       [2:0] clk_ce,       // falling CLK edge per counter
	input       [2:0] gate,
	output      [2:0] out,

	// savestate port: eight bytes a counter, no side effects
	input             ss_cs,
	input             ss_wr,
	input       [4:0] ss_a,
	input       [7:0] ss_din,
	output      [7:0] ss_dout
);

// Strobe edges. A cycle is one or more clocks with the strobe low.
reg wr_q, rd_q;
reg [1:0] a_q;
reg [7:0] d_q;
wire wr_active = ~cs_n & ~wr_n;
wire rd_active = ~cs_n & ~rd_n;
wire wr_edge = wr_q & ~wr_active;   // WR_n just rose
wire rd_edge = rd_q & ~rd_active;   // RD_n just rose

always @(posedge clk) begin
	wr_q <= wr_active;
	rd_q <= rd_active;
	if (wr_active || rd_active) begin
		a_q <= a;
		d_q <= d_i;
	end
end

wire [7:0] cnt_dout [0:2];
wire [7:0] ss_cnt_dout [0:2];
wire       ctrl_wr  = wr_edge && a_q == 2'd3;
assign ss_dout = ss_a[4:3] == 2'd3 ? 8'h00 : ss_cnt_dout[ss_a[4:3]];

// reading the control address is a no-op: the bus stays undriven
assign d_o  = (a == 2'd3) ? 8'hFF : cnt_dout[a];
assign d_oe = rd_active && a != 2'd3;

genvar i;
generate for (i = 0; i < 3; i = i + 1) begin : counter
	i8253_counter c
	(
		.clk(clk),
		.reset(reset),
		.ctrl_wr(ctrl_wr && d_q[7:6] == i),
		.count_wr(wr_edge && a_q == i),
		.count_rd(rd_edge && a_q == i),
		.wdata(d_q),
		.rdata(cnt_dout[i]),
		.clk_ce(clk_ce[i]),
		.gate(gate[i]),
		.out(out[i]),
		.ss_cs(ss_cs && ss_a[4:3] == i),
		.ss_wr(ss_wr),
		.ss_a(ss_a[2:0]),
		.ss_din(ss_din),
		.ss_dout(ss_cnt_dout[i])
	);
end
endgenerate

endmodule

// One 8253 counter: modes 0-5, binary or BCD, latch and byte sequencing.
module i8253_counter
(
	input             clk,
	input             reset,

	input             ctrl_wr,      // control word addressed to this counter
	input             count_wr,     // count byte written
	input             count_rd,     // count byte read (RD_n rose)
	input       [7:0] wdata,
	output      [7:0] rdata,

	input             clk_ce,
	input             gate,
	output reg        out,

	input             ss_cs,
	input             ss_wr,
	input       [2:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

reg  [2:0] mode;
reg        bcd;
reg  [1:0] rl;           // 01 low byte, 10 high byte, 11 low then high
reg [15:0] cr;           // count register as written
reg        wr_hi;        // next write byte is the high one (rl == 11)
reg        rd_hi;        // next read byte is the high one
reg [15:0] ol;           // output latch
reg        ol_full;
reg [15:0] ce_cnt;       // counting element
reg        load;         // full count written, load on next clock
reg        active;       // counting element running
reg        just_loaded;  // first clock after a load (a written 0 is 65536)
reg        gate_q;       // gate at the last clock edge
reg        trig;         // gate rising edge seen since the last clock edge

wire [15:0] rd_src = ol_full ? ol : ce_cnt;
assign rdata = rd_hi ? rd_src[15:8] : rd_src[7:0];

always @* begin
	case (ss_a)
	3'd0: ss_dout = {mode, bcd, rl, wr_hi, rd_hi};
	3'd1: ss_dout = cr[7:0];
	3'd2: ss_dout = cr[15:8];
	3'd3: ss_dout = ol[7:0];
	3'd4: ss_dout = ol[15:8];
	3'd5: ss_dout = ce_cnt[7:0];
	3'd6: ss_dout = ce_cnt[15:8];
	default: ss_dout = {1'b0, ol_full, load, active, just_loaded, gate_q, trig, out};
	endcase
end

// Rising gate edges are caught between counter clocks or on the clock itself.
wire gate_rise = gate & ~gate_q;
wire trig_now  = trig | gate_rise;

// Count down by n (1 to 3). In BCD the low digit takes the whole step and
// borrows ten; the digits above only ever borrow one.
function [15:0] decn(input [15:0] v, input [1:0] n, input is_bcd);
	reg [3:0] n0, n1, n2, n3, d0;
	reg       borrow;
	begin
		n0 = v[3:0]; n1 = v[7:4]; n2 = v[11:8]; n3 = v[15:12];
		if (!is_bcd) decn = v - {14'd0, n};
		else begin
			borrow = n0 < {2'd0, n};
			d0 = borrow ? n0 + 4'd10 - {2'd0, n} : n0 - {2'd0, n};
			if (!borrow) decn = {n3, n2, n1, d0};
			else if (n1 != 0) decn = {n3, n2, n1 - 4'd1, d0};
			else if (n2 != 0) decn = {n3, n2 - 4'd1, 4'd9, d0};
			else decn = {(n3 == 0) ? 4'd9 : n3 - 4'd1, 4'd9, 4'd9, d0};
		end
	end
endfunction

// Mode 3 steps by two, with the odd-count adjustment on the first clock
// after a load: one when OUT is high, three when it is low.
wire  [1:0] step = (mode[1:0] == 2'b11) ? (just_loaded && ce_cnt[0] ? (out ? 2'd1 : 2'd3) : 2'd2) : 2'd1;
wire [15:0] next = decn(ce_cnt, step, bcd);
// A loaded zero counts as the full range, so it cannot expire at once.
wire        big   = just_loaded && ce_cnt == 16'd0;
wire        hit0  = !big && ce_cnt != 16'd0 && ce_cnt <= {14'd0, step};   // reaches or passes 0 this clock
wire        m2_last = ce_cnt == 16'd1;                    // mode 2 reload point

always @(posedge clk) begin
	if (reset) begin
		mode <= 3'd0; bcd <= 0; rl <= 2'b11;
		wr_hi <= 0; rd_hi <= 0; ol_full <= 0;
		load <= 0; active <= 0; just_loaded <= 0;
		out <= 1; trig <= 0; gate_q <= gate;
		cr <= 16'd0; ce_cnt <= 16'd0; ol <= 16'd0;
	end
	else begin
		if (gate_rise) trig <= 1;
		gate_q <= gate;

		// ---- bus side ----
		if (ctrl_wr) begin
			if (wdata[5:4] == 2'b00) begin
				if (!ol_full) begin
					ol <= ce_cnt;
					ol_full <= 1;
				end
			end
			else begin
				rl    <= wdata[5:4];
				mode  <= wdata[3:1];
				bcd   <= wdata[0];
				wr_hi <= wdata[5:4] == 2'b10;
				rd_hi <= wdata[5:4] == 2'b10;
				ol_full <= 0;
				active <= 0;
				load   <= 0;
				trig   <= 0;
				out    <= wdata[3:1] != 3'd0;   // mode 0 starts low, the rest high
			end
		end

		if (count_wr) begin
			if (mode == 3'd0) out <= 0;   // a new count restarts the terminal-count wait
			case (rl)
			2'b01: begin cr <= {8'd0, wdata}; load <= 1; end
			2'b10: begin cr <= {wdata, 8'd0}; load <= 1; end
			default: begin
				if (wr_hi) begin
					cr[15:8] <= wdata;
					load <= 1;
				end
				else begin
					cr[7:0] <= wdata;
					// first byte of a pair stops mode 0 counting
					if (mode == 3'd0) active <= 0;
				end
				wr_hi <= ~wr_hi;
			end
			endcase
		end

		if (count_rd) begin
			case (rl)
			2'b11: begin
				if (rd_hi) ol_full <= 0;
				rd_hi <= ~rd_hi;
			end
			default: ol_full <= 0;
			endcase
		end

		// Modes 2 and 3: a low gate forces OUT high at once.
		if (!gate && (mode[1:0] == 2'b10 || mode[1:0] == 2'b11)) out <= 1;

		// ---- counter clock ----
		if (clk_ce) begin
			just_loaded <= 0;
			case (mode[1:0] == 2'b10 ? 3'd2 : mode[1:0] == 2'b11 ? 3'd3 : mode)
			3'd0: begin   // interrupt on terminal count
				if (load) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
				end
				else if (active && gate) begin
					ce_cnt <= next;
					if (hit0) out <= 1;
				end
			end
			3'd1: begin   // one-shot on the gate edge
				if (trig_now) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
					out <= 0; trig <= 0;
				end
				else if (active) begin
					ce_cnt <= next;
					if (hit0) out <= 1;
				end
				if (load && !trig_now) load <= 0;   // new count waits for the next trigger
			end
			3'd2: begin   // rate generator
				if ((load && !active) || trig_now) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
					out <= 1; trig <= 0;
				end
				else if (active && gate) begin
					if (m2_last && !big) begin
						ce_cnt <= cr; load <= 0; just_loaded <= 1;
						out <= 1;
					end
					else begin
						ce_cnt <= next;
						out <= !(next == 16'd1);
					end
				end
			end
			3'd3: begin   // square wave
				if ((load && !active) || trig_now) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
					out <= 1; trig <= 0;
				end
				else if (active && gate) begin
					if (hit0) begin
						ce_cnt <= cr; load <= 0; just_loaded <= 1;
						out <= ~out;
					end
					else ce_cnt <= next;
				end
			end
			3'd4: begin   // software triggered strobe
				if (load) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
				end
				else if (active && gate) begin
					ce_cnt <= next;
					out <= !hit0;
				end
				else out <= 1;
			end
			default: begin   // 5: hardware triggered strobe
				if (trig_now) begin
					ce_cnt <= cr; load <= 0; active <= 1; just_loaded <= 1;
					out <= 1; trig <= 0;
				end
				else if (active) begin
					ce_cnt <= next;
					out <= !hit0;
				end
				if (load && !trig_now) load <= 0;
			end
			endcase
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
			3'd0: {mode, bcd, rl, wr_hi, rd_hi} <= ss_din;
			3'd1: cr[7:0] <= ss_din;
			3'd2: cr[15:8] <= ss_din;
			3'd3: ol[7:0] <= ss_din;
			3'd4: ol[15:8] <= ss_din;
			3'd5: ce_cnt[7:0] <= ss_din;
			3'd6: ce_cnt[15:8] <= ss_din;
			default: {ol_full, load, active, just_loaded, gate_q, trig, out} <= ss_din[6:0];
			endcase
		end
	end
end

endmodule

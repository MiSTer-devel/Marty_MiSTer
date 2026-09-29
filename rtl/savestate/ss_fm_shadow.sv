// Copyright (c) 2026 Jamie Blanks
//
// Savestate shadow of the YM3438's registers. The chip keeps its state in
// gate-level cells, so a save takes the last value written to each of its
// 512 registers instead, plus the key-on state of every channel; a load
// writes the shadow back and then plays it into the chip through its own
// bus pins at the chip's pace, key-ons last.
//
//   guest write 04D8-04DE ─> address latch per bank ─> shadow[bank, reg]
//   state port 00: shadow stream (any access to 01 rewinds)
//              02: write starts the replay, reads 1 while it runs
//              03/04: address latches   10-15: key-on per channel
module ss_fm_shadow
(
	input             clk,
	input             reset,

	// guest writes to the chip as the mainboard strobes them
	input             wr,            // one clock per byte
	input       [1:0] a,             // {bank, data}
	input       [7:0] din,

	// state port
	input             ss_cs,
	input             ss_wr,
	input             ss_step,
	input       [7:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,

	// replay drive: the chip's pins while `replaying`
	output reg        replaying,
	output reg        rep_cs,        // active high
	output reg        rep_wr,
	output reg  [1:0] rep_a,
	output reg  [7:0] rep_din
);

reg  [7:0] addr_latch [0:1];
reg  [3:0] keyon [0:5];
reg  [8:0] stream;               // state port stream address
reg  [9:0] rep_idx;              // 0-511 the registers, 512-517 the key-ons
reg  [9:0] rep_wait;
reg  [1:0] rep_phase;            // 0 address strobe, 1 data strobe, 2 gap
wire       rep_key = rep_idx >= 10'd512;
wire [2:0] rep_ch  = rep_idx[2:0];

// one port serves the guest write, the stream and the replay read in turn
wire        guest_we = wr && a[0];
wire  [8:0] ram_addr = replaying ? rep_idx[8:0] : ss_cs ? stream : {a[1], addr_latch[a[1]]};
wire        ram_we   = replaying ? 1'b0 : ss_cs ? (ss_wr && ss_a == 8'h00) : guest_we;
wire  [7:0] ram_d    = ss_cs ? ss_din : din;
wire  [7:0] ram_q;
cache_ram #(.ADDR_WIDTH(9), .DATA_WIDTH(8)) shadow
(
	.clk_i(clk), .addr_i(ram_addr), .wren_i(ram_we), .wdata_i(ram_d), .q_o(ram_q)
);

integer i;
always @(posedge clk) begin
	if (reset) begin
		addr_latch[0] <= 8'd0; addr_latch[1] <= 8'd0;
		for (i = 0; i < 6; i = i + 1) keyon[i] <= 4'd0;
		stream <= 9'd0;
		replaying <= 0;
		rep_cs <= 0; rep_wr <= 0;
	end
	else begin
		if (wr && !a[0]) addr_latch[a[1]] <= din;
		if (guest_we && !a[1] && addr_latch[0] == 8'h28 && din[2:0] <= 3'd5) keyon[din[2:0]] <= din[7:4];

		if (ss_cs && ss_a == 8'h01) stream <= 9'd0;
		else if (ss_cs && ss_a == 8'h00 && ss_step) stream <= stream + 1'd1;
		if (ss_cs && ss_wr) begin
			if (ss_a == 8'h03) addr_latch[0] <= ss_din;
			if (ss_a == 8'h04) addr_latch[1] <= ss_din;
			if (ss_a[7:4] == 4'h1 && ss_a[3:0] <= 4'd5) keyon[ss_a[2:0]] <= ss_din[3:0];
			if (ss_a == 8'h02 && !replaying) begin
				replaying <= 1;
				rep_idx   <= 10'd0;
				rep_phase <= 2'd0;
				rep_wait  <= 10'd16;
				rep_cs <= 0; rep_wr <= 0;
			end
		end

		// each register: its address, then its data, then a pause the chip
		// needs before the next write; the strobes stay low for 16 clocks
		if (replaying) begin
			rep_wait <= rep_wait - 1'd1;
			case (rep_phase)
			2'd0: begin
				rep_a   <= {rep_key ? 1'b0 : rep_idx[8], 1'b0};
				rep_din <= rep_key ? 8'h28 : rep_idx[7:0];
				rep_cs  <= 1; rep_wr <= 1;
				if (rep_wait == 0) begin rep_cs <= 0; rep_wr <= 0; rep_phase <= 2'd1; rep_wait <= 10'd1023; end
			end
			2'd1: if (rep_wait == 0) begin
				rep_a   <= {rep_key ? 1'b0 : rep_idx[8], 1'b1};
				rep_din <= rep_key ? {keyon[rep_ch], 1'b0, rep_ch} : ram_q;
				rep_cs  <= 1; rep_wr <= 1;
				rep_phase <= 2'd2;
				rep_wait  <= 10'd1023;
			end
			default: begin
				if (rep_wait == 10'd1007) begin rep_cs <= 0; rep_wr <= 0; end
				if (rep_wait == 0) begin
					rep_phase <= 2'd0;
					rep_wait  <= 10'd16;
					if (rep_idx == 10'd517) replaying <= 0;
					else rep_idx <= rep_idx + 1'd1;
				end
			end
			endcase
		end
	end
end

always @* begin
	case (ss_a)
	8'h00: ss_dout = ram_q;
	8'h02: ss_dout = {7'd0, replaying};
	8'h03: ss_dout = addr_latch[0];
	8'h04: ss_dout = addr_latch[1];
	8'h10, 8'h11, 8'h12, 8'h13, 8'h14, 8'h15: ss_dout = {4'd0, keyon[ss_a[2:0]]};
	default: ss_dout = 8'h00;
	endcase
end

endmodule

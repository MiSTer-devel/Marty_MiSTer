// Copyright (c) 2026 Jamie Blanks
//
// Battery-backed CMOS RAM, 8 KB of bytes reachable two ways: I/O 3000-3FFF
// on even addresses (one byte per word address) and the memory window
// D8000-D9FFF. Both carry the byte on D7-D0. Reads take one clock; the
// mainboard waits a T-state before sampling. The second port belongs to
// the backup engine that loads and saves the image.
//
// With 8 MB fitted the SYSTEM ROM still records a 6 MB memory end (5F at
// 318A, the byte TownsOS sizes memory from) and clears only 6 MB, so the
// CPU reads that byte as 7F; the image keeps what the ROM wrote.

module towns_cmos #(parameter SIM_INIT_FILE = " ")
(
	input             clk,
	input             ce,

	input      [12:0] addr,          // byte index
	input             wr,
	input       [7:0] din,
	output      [7:0] dout,
	input             mem8,          // 8 MB fitted

	input      [12:0] bk_addr,
	input             bk_wr,
	input       [7:0] bk_din,
	output      [7:0] bk_dout
);

wire [7:0] q;
reg [12:0] addr_q;
always @(posedge clk) addr_q <= addr;
assign dout = (mem8 && addr_q == 13'h0C5 && q == 8'h5F) ? 8'h7F : q;

cache_ram_dp #(.ADDR_WIDTH(13), .DATA_WIDTH(8), .SIM_INIT_FILE(SIM_INIT_FILE)) ram
(
	.clk_i(clk),
	.addr_a_i(addr), .wren_a_i(wr & ce), .wdata_a_i(din), .q_a_o(q),
	.addr_b_i(bk_addr), .wren_b_i(bk_wr), .wdata_b_i(bk_din), .q_b_o(bk_dout)
);

endmodule

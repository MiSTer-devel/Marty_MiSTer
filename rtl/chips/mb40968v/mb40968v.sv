// Copyright (c) 2026 Jamie Blanks
//
// MB40968V two-channel 8-bit video DAC: two independent latches clocked
// by CLKA and CLKB, no blanking or sync pins. Here each channel latches
// on its clock enable and the analogue outputs are the latched codes.

module mb40968v
(
	input        clk,
	input        clk_a,    // CLKA as an enable
	input        clk_b,    // CLKB as an enable
	input  [7:0] a,        // A1 (MSB) .. A8
	input  [7:0] b,        // B1 (MSB) .. B8
	output reg [7:0] aout,
	output reg [7:0] bout
);

always @(posedge clk) begin
	if (clk_a) aout <= a;
	if (clk_b) bout <= b;
end

endmodule

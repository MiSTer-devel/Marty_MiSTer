// Copyright (c) 2026 Jamie Blanks
//
// Sanyo LC7881 16-bit stereo audio DAC as it sits behind the PCM chip:
// the chip hands over a sample pair per period and the converter holds it
// until the next one. The serial data link between the two is collapsed
// to a parallel handover; the value is the same word.

module lc7881
(
	input             clk,
	input             reset,
	input             strobe,       // sample pair valid
	input signed [15:0] d_l,
	input signed [15:0] d_r,
	output reg signed [15:0] out_l,
	output reg signed [15:0] out_r
);

always @(posedge clk) begin
	if (reset) begin
		out_l <= 0; out_r <= 0;
	end
	else if (strobe) begin
		out_l <= d_l; out_r <= d_r;
	end
end

endmodule

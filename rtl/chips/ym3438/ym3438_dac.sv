// Copyright (c) 2026 Jamie Blanks
//
// The chip's analogue output stage: the YM3438 drives its DAC with the six
// channels one after another, four internal cycles each, and the filter
// after it averages them. Here the six values of one sample period are
// summed into a signed 12-bit pair with a strobe when the sum is ready.
//
//   MOL/MOR are offset binary, 0x100 = silence. A channel whose pan bit
//   is off (or the load slot) reads 0x100 and adds nothing.

module ym3438_dac
(
	input             clk,
	input             reset,
	input       [8:0] mol,
	input       [8:0] mor,
	input       [2:0] ch_index,
	input             out_enable,   // low during the load slot of each channel

	output reg signed [11:0] out_l,
	output reg signed [11:0] out_r,
	output reg               sample    // one clk pulse per sample period
);

reg  [8:0] mol_q, mor_q;
reg  [2:0] idx_q;
reg        en_q;
reg signed [11:0] acc_l, acc_r;

// The value is settled by the end of its slot: take it as the enable drops.
// The chip walks the channels as 1 5 3 0 4 2 (index order), so a sample
// period runs from index 1 to index 2.
wire capture = en_q & ~out_enable;
wire signed [11:0] val_l = {{4{~mol_q[8]}}, mol_q[7:0]};   // mol - 0x100
wire signed [11:0] val_r = {{4{~mor_q[8]}}, mor_q[7:0]};

always @(posedge clk) begin
	mol_q  <= mol;
	mor_q  <= mor;
	idx_q  <= ch_index;
	en_q   <= out_enable;
	sample <= 0;
	if (reset) begin
		acc_l <= 0; acc_r <= 0;
		out_l <= 0; out_r <= 0;
	end
	else if (capture) begin
		if (idx_q == 3'd2) begin
			out_l  <= acc_l + val_l;
			out_r  <= acc_r + val_r;
			acc_l  <= 0;
			acc_r  <= 0;
			sample <= 1;
		end
		else if (idx_q == 3'd1) begin
			acc_l <= val_l;
			acc_r <= val_r;
		end
		else begin
			acc_l <= acc_l + val_l;
			acc_r <= acc_r + val_r;
		end
	end
end

endmodule

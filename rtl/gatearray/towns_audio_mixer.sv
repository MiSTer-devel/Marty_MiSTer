// Copyright (c) 2026 Jamie Blanks
//
// Audio glue of the Towns sound section (Technical Databook 3rd ed. §5):
// the FM/PCM mute and audio registers, the two MB87078 electronic volumes
// in front of the CD-DA input, the PCM bank interrupt logic and IRQ13, and
// a digital mix of what the analogue mixer sums.
//
//   04D5 R/W  bit1 FM mute, bit0 PCM mute (0 = muted)
//   04E0/04E2 R/W  volume 1/2 DATA, six bits for the selected channel
//   04E1/04E3 R/W  volume 1/2 COM: C32, C0, EN, CH1, CH0
//   04E9 R    bit3 PCM interrupt, bit0 FM interrupt
//   04EA R/W  PCM interrupt mask, one bit per 8 KB of wave RAM
//   04EB R    PCM interrupt flags, cleared by the read
//   04EC R/W  bit7 LOFF (level LEDs off), bit6 MUTE (0 = no output)
//
//   FM and PCM bypass the volumes; volume 2 channels 0/1 are CD left/right
//   (channel 2 mic and 3 modem, volume 1 line-in, all absent on a Marty).
//   MB87078: EN=0 -> silence, C32=1 -> -32 dB, else C0=1 -> 0 dB, else
//   -(63-D)/2 dB; reset state 0 dB with EN set.
//
//   Levels between the three sources are unmeasured: each is scaled to
//   about half of full scale before the sum.
//
//   The PCM DAC output passes a smoothing filter with a cutoff of about
//   4 kHz (Databook §5.3.2); modelled first order, stepped at 800 kHz
//   with a 1/32 coefficient (fc = 800k / (2 pi 32) = 3.98 kHz). `ce` is
//   the CPU enable for the I/O strobes; `ce_16m` is the fixed 16 MHz the
//   filter divides.

module towns_audio_mixer
(
	input             clk,
	input             ce,
	input             ce_16m,
	input             reset,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	// savestate port: the registers as bytes
	input             ss_cs,
	input             ss_wr,
	input       [4:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output reg        io_sel,

	input signed [11:0] fm_l,       // six-channel sum from the FM chip
	input signed [11:0] fm_r,
	input             fm_irq_n,     // the chip's /IRQ pin
	input signed [15:0] pcm_l,
	input signed [15:0] pcm_r,
	input             pcm_bank_ev,
	input       [3:0] pcm_bank,
	input signed [15:0] cd_l,
	input signed [15:0] cd_r,
	input             beep,         // PIT tone gated by the buzzer enable

	output            irq13,
	output reg        led_off,
	output reg signed [15:0] out_l,
	output reg signed [15:0] out_r
);

wire sel_mute = io_addr == 16'h04D5;
wire sel_vol  = io_addr[15:2] == 14'h0138;          // 04E0-04E3
wire sel_int  = io_addr[15:2] == 14'h013A && io_addr[1:0] != 2'd0;   // 04E9-04EB
wire sel_aud  = io_addr == 16'h04EC;
wire wr = io_wr & ce;
wire rd = io_rd & ce;

reg  [1:0] mute;          // {FM, PCM}
reg        out_en;
reg  [7:0] pcm_mask, pcm_flags;

// ---- electronic volumes: per chip the selected channel and its COM bits,
// per channel the six-bit level and EN/C0/C32
reg  [4:0] vol_com  [0:1];
reg  [5:0] vol_data [0:7];       // {chip, channel}
reg  [2:0] vol_ctl  [0:7];       // {C32, C0, EN}
wire [2:0] vol_idx  = {io_addr[1], vol_com[io_addr[1]][1:0]};

integer i;
always @(posedge clk) begin
	if (reset) begin
		mute <= 2'b00;
		out_en <= 0;
		led_off <= 0;
		pcm_mask <= 8'h00;
		pcm_flags <= 8'h00;
		vol_com[0] <= 5'b00100; vol_com[1] <= 5'b00100;
		for (i = 0; i < 8; i = i + 1) begin vol_data[i] <= 6'h3F; vol_ctl[i] <= 3'b001; end
	end
	else begin
		if (wr) begin
			if (sel_mute) mute <= io_din[1:0];
			if (sel_aud)  {led_off, out_en} <= io_din[7:6];
			if (sel_int && io_addr[1:0] == 2'd2) pcm_mask <= io_din;
			if (sel_vol) begin
				if (io_addr[0]) begin
					vol_com[io_addr[1]] <= io_din[4:0];
					vol_ctl[{io_addr[1], io_din[1:0]}] <= io_din[4:2];
				end
				else vol_data[vol_idx] <= io_din[5:0];
			end
		end
		// flags: set by a bank event the mask allows, cleared by reading 04EB
		if (rd && sel_int && io_addr[1:0] == 2'd3) pcm_flags <= 8'h00;
		if (pcm_bank_ev && pcm_mask[pcm_bank[3:1]]) pcm_flags[pcm_bank[3:1]] <= 1;
		if (ss_cs && ss_wr) begin
			if (ss_a[4]) begin
				if (ss_a[3]) vol_ctl[ss_a[2:0]] <= ss_din[2:0]; else vol_data[ss_a[2:0]] <= ss_din[5:0];
			end
			else case (ss_a[3:0])
			4'd0: {led_off, out_en, mute} <= ss_din[3:0];
			4'd1: pcm_mask <= ss_din;
			4'd2: pcm_flags <= ss_din;
			4'd3: vol_com[0] <= ss_din[4:0];
			4'd4: vol_com[1] <= ss_din[4:0];
			default: ;
			endcase
		end
	end
end

always @* begin
	if (ss_a[4]) ss_dout = ss_a[3] ? {5'd0, vol_ctl[ss_a[2:0]]} : {2'd0, vol_data[ss_a[2:0]]};
	else case (ss_a[3:0])
	4'd0: ss_dout = {4'd0, led_off, out_en, mute};
	4'd1: ss_dout = pcm_mask;
	4'd2: ss_dout = pcm_flags;
	4'd3: ss_dout = {3'd0, vol_com[0]};
	4'd4: ss_dout = {3'd0, vol_com[1]};
	default: ss_dout = 8'h00;
	endcase
end

wire pcm_irq = pcm_flags != 8'h00;
wire fm_irq  = ~fm_irq_n;
assign irq13 = pcm_irq | fm_irq;

always @* begin
	io_sel = sel_mute | sel_vol | sel_int | sel_aud;
	io_dout = 8'hFF;
	if (sel_mute) io_dout = {6'd0, mute};
	if (sel_vol)  io_dout = io_addr[0] ? {3'd0, vol_com[io_addr[1]]} : {2'd0, vol_data[vol_idx]};
	if (sel_int) case (io_addr[1:0])
		2'd1: io_dout = {4'd0, pcm_irq, 2'd0, fm_irq};
		2'd2: io_dout = pcm_mask;
		default: io_dout = pcm_flags;
	endcase
	if (sel_aud)  io_dout = {led_off, out_en, 6'd0};
end

// ---- CD-DA through volume 2 channels 0 (left) and 1 (right) ----
// gain in Q15 from the 0.5 dB table
function [15:0] gain_q15(input [5:0] d);
	case (d)
	6'd0: gain_q15 = 16'd872;   6'd1: gain_q15 = 16'd923;   6'd2: gain_q15 = 16'd978;   6'd3: gain_q15 = 16'd1036;
	6'd4: gain_q15 = 16'd1098;  6'd5: gain_q15 = 16'd1163;  6'd6: gain_q15 = 16'd1232;  6'd7: gain_q15 = 16'd1304;
	6'd8: gain_q15 = 16'd1382;  6'd9: gain_q15 = 16'd1464;  6'd10: gain_q15 = 16'd1550; 6'd11: gain_q15 = 16'd1642;
	6'd12: gain_q15 = 16'd1740; 6'd13: gain_q15 = 16'd1843; 6'd14: gain_q15 = 16'd1952; 6'd15: gain_q15 = 16'd2067;
	6'd16: gain_q15 = 16'd2190; 6'd17: gain_q15 = 16'd2320; 6'd18: gain_q15 = 16'd2457; 6'd19: gain_q15 = 16'd2603;
	6'd20: gain_q15 = 16'd2757; 6'd21: gain_q15 = 16'd2920; 6'd22: gain_q15 = 16'd3093; 6'd23: gain_q15 = 16'd3277;
	6'd24: gain_q15 = 16'd3471; 6'd25: gain_q15 = 16'd3677; 6'd26: gain_q15 = 16'd3894; 6'd27: gain_q15 = 16'd4125;
	6'd28: gain_q15 = 16'd4370; 6'd29: gain_q15 = 16'd4628; 6'd30: gain_q15 = 16'd4903; 6'd31: gain_q15 = 16'd5193;
	6'd32: gain_q15 = 16'd5501; 6'd33: gain_q15 = 16'd5827; 6'd34: gain_q15 = 16'd6172; 6'd35: gain_q15 = 16'd6538;
	6'd36: gain_q15 = 16'd6925; 6'd37: gain_q15 = 16'd7336; 6'd38: gain_q15 = 16'd7770; 6'd39: gain_q15 = 16'd8231;
	6'd40: gain_q15 = 16'd8718; 6'd41: gain_q15 = 16'd9235; 6'd42: gain_q15 = 16'd9782; 6'd43: gain_q15 = 16'd10362;
	6'd44: gain_q15 = 16'd10976; 6'd45: gain_q15 = 16'd11626; 6'd46: gain_q15 = 16'd12315; 6'd47: gain_q15 = 16'd13045;
	6'd48: gain_q15 = 16'd13818; 6'd49: gain_q15 = 16'd14636; 6'd50: gain_q15 = 16'd15504; 6'd51: gain_q15 = 16'd16422;
	6'd52: gain_q15 = 16'd17395; 6'd53: gain_q15 = 16'd18426; 6'd54: gain_q15 = 16'd19518; 6'd55: gain_q15 = 16'd20675;
	6'd56: gain_q15 = 16'd21900; 6'd57: gain_q15 = 16'd23197; 6'd58: gain_q15 = 16'd24572; 6'd59: gain_q15 = 16'd26028;
	6'd60: gain_q15 = 16'd27570; 6'd61: gain_q15 = 16'd29204; 6'd62: gain_q15 = 16'd30934; default: gain_q15 = 16'd32767;
	endcase
endfunction

function [15:0] chan_gain(input [2:0] ctl, input [5:0] d);
	begin
		if (!ctl[0])     chan_gain = 16'd0;       // EN
		else if (ctl[2]) chan_gain = 16'd823;     // C32 wins over C0
		else if (ctl[1]) chan_gain = 16'd32767;   // C0
		else             chan_gain = gain_q15(d);
	end
endfunction

reg  [15:0] gain_l, gain_r;
reg  signed [31:0] cd_gl, cd_gr;
always @(posedge clk) begin
	gain_l <= chan_gain(vol_ctl[4], vol_data[4]);
	gain_r <= chan_gain(vol_ctl[5], vol_data[5]);
	cd_gl  <= cd_l * $signed({1'b0, gain_l});
	cd_gr  <= cd_r * $signed({1'b0, gain_r});
end

// ---- PCM output filter ----
// accumulators hold the output in Q5: adding the whole difference steps
// the output by 1/32 of it
reg   [4:0] lpf_div;
wire        lpf_step = ce_16m && lpf_div == 5'd19;
always @(posedge clk) if (reset) lpf_div <= 0; else if (ce_16m) lpf_div <= lpf_step ? 5'd0 : lpf_div + 1'd1;

reg  signed [20:0] lpf_acc_l, lpf_acc_r;
wire signed [15:0] pcm_fl = lpf_acc_l[20:5];
wire signed [15:0] pcm_fr = lpf_acc_r[20:5];
wire signed [16:0] lpf_dl = {pcm_l[15], pcm_l} - {pcm_fl[15], pcm_fl};
wire signed [16:0] lpf_dr = {pcm_r[15], pcm_r} - {pcm_fr[15], pcm_fr};
always @(posedge clk) begin
	if (reset) begin
		lpf_acc_l <= 0;
		lpf_acc_r <= 0;
	end else if (lpf_step) begin
		lpf_acc_l <= lpf_acc_l + {{4{lpf_dl[16]}}, lpf_dl};
		lpf_acc_r <= lpf_acc_r + {{4{lpf_dr[16]}}, lpf_dr};
	end
end

// ---- mix ----
wire signed [17:0] fm_sl = mute[1] ? {{2{fm_l[11]}}, fm_l, 4'd0} : 18'sd0;
wire signed [17:0] fm_sr = mute[1] ? {{2{fm_r[11]}}, fm_r, 4'd0} : 18'sd0;
wire signed [17:0] pcm_sl = mute[0] ? {{3{pcm_fl[15]}}, pcm_fl[15:1]} : 18'sd0;
wire signed [17:0] pcm_sr = mute[0] ? {{3{pcm_fr[15]}}, pcm_fr[15:1]} : 18'sd0;
// Q15 gain, then 7/32 of full scale: recordings of a real machine put the PCM
// about 7 dB higher against CD-DA than the earlier half-scale CD term did
wire signed [17:0] cd_sl = {{4{cd_gl[31]}}, cd_gl[30:17]} - {{7{cd_gl[31]}}, cd_gl[30:20]};
wire signed [17:0] cd_sr = {{4{cd_gr[31]}}, cd_gr[30:17]} - {{7{cd_gr[31]}}, cd_gr[30:20]};
// the beep is a square wave at 1/8 of full scale; the real level is unmeasured
wire signed [17:0] beep_s = beep ? 18'sd4096 : 18'sd0;
wire signed [17:0] sum_l = fm_sl + pcm_sl + cd_sl + beep_s;
wire signed [17:0] sum_r = fm_sr + pcm_sr + cd_sr + beep_s;

function signed [15:0] sat(input signed [17:0] v);
	if (v > 18'sd32767) sat = 16'sd32767;
	else if (v < -18'sd32768) sat = 16'sh8000;
	else sat = v[15:0];
endfunction

always @(posedge clk) begin
	out_l <= out_en ? sat(sum_l) : 16'sd0;
	out_r <= out_en ? sat(sum_r) : 16'sd0;
end

endmodule

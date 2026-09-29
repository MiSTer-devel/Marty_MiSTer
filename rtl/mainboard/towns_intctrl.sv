// Copyright (c) 2026 Jamie Blanks
//
// Timer interrupt control and the microsecond timers next to the PITs
// (Technical Databook 3rd ed. §3.4 and appendix H).
//
//   PIT1 OUT0 ─rise─> TMOUT0 ─┬─ & TM0MSK ─┐
//   PIT1 OUT1 ─rise─> TMOUT1 ─┴─ & TM1MSK ─┴─ OR ──> IRQ0
//   PIT1 OUT2 ──────────── & (SOUND | BUZZER) ───> beep
//
//   0060 W {TM0CLR, 0,0,0,0, SOUND, TM1MSK, TM0MSK}   R {1,1,1, SOUND, TM1MSK, TM0MSK, TMOUT1, TMOUT0}
//   0068 R/W bit 7 INTV-EN (0 = run), R bit 6 INTV-I, bit 5 INTV-OV, both cleared by the read
//   006A/006B  interval timer II period in microseconds, 0 = 65536
//   006C write: 1 us wait (the mainboard extends the cycle)
//   0026/0027  free-running 1 us counter on a 16-bit port: a word read
//              takes both bytes at once, two byte reads can tear
//   CFF98 (FMR buzzer switch, presented as I/O FF98) read = on, write = off
//
// TMOUT1 clears when a new count is written to timer 1; the timeout it
// guards is reprogrammed per I/O operation.

module towns_intctrl
(
	input             clk,
	input             ce,             // CPU enable, for the I/O strobes
	input             ce_16m,         // fixed 16 MHz, for the 1 us tick
	input             reset,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output reg        io_sel,
	output            io_wide,        // 16-bit register: io_dout_hi is its high byte
	output      [7:0] io_dout_hi,
	input             tmr1_wr,        // count byte written to timer 1

	input       [2:0] pit_out,
	output            irq0,
	output            beep,
	output            intv_irq,

	// savestate port: the state as bytes, no side effects
	input             ss_cs,
	input             ss_wr,
	input       [3:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

reg tmout0, tmout1, tm0msk, tm1msk, sound, buzzer;
reg [1:0] pit_q;

assign irq0 = (tmout0 & tm0msk) | (tmout1 & tm1msk);
assign beep = pit_out[2] & (sound | buzzer);

// 1 MHz enable: every sixteenth tick of the fixed 16 MHz.
reg  [3:0] us_div;
wire       ce_us = ce_16m && us_div == 4'd15;
reg [15:0] free_run;
assign io_wide    = io_addr[15:1] == 15'h0013;   // 0026/0027
assign io_dout_hi = free_run[15:8];

// Interval timer II
reg        intv_dis;                 // bit 7 as written
reg        intv_i, intv_ov;
reg [15:0] intv_period, intv_cnt;

assign intv_irq = intv_i & ~intv_dis;

always @(posedge clk) begin
	if (reset) begin
		tmout0 <= 0; tmout1 <= 0; tm0msk <= 0; tm1msk <= 0; sound <= 0; buzzer <= 0;
		pit_q <= pit_out[1:0]; us_div <= 4'd0; free_run <= 16'd0;
		intv_dis <= 1; intv_i <= 0; intv_ov <= 0; intv_period <= 16'd0; intv_cnt <= 16'd0;
	end
	else begin
		pit_q <= pit_out[1:0];
		if (pit_out[0] & ~pit_q[0]) tmout0 <= 1;
		if (pit_out[1] & ~pit_q[1]) tmout1 <= 1;

		if (ce_us) begin
			free_run <= free_run + 1'd1;
			if (intv_dis) intv_cnt <= intv_period;
			else if (intv_cnt == 16'd1) begin
				intv_cnt <= intv_period;
				if (intv_i) intv_ov <= 1;
				intv_i <= 1;
			end
			else intv_cnt <= intv_cnt - 1'd1;
		end

		if (ce_16m) us_div <= us_div + 1'd1;
		if (ce) begin
			if (tmr1_wr) tmout1 <= 0;

			// a new period or an enable starts the count at once, whatever
			// the phase of the 1 us tick
			if (io_wr) begin
				case (io_addr)
				16'h0060: begin
					if (io_din[7]) tmout0 <= 0;
					sound  <= io_din[2];
					tm1msk <= io_din[1];
					tm0msk <= io_din[0];
				end
				16'h0068: begin
					intv_dis <= io_din[7];
					if (!io_din[7]) intv_cnt <= intv_period;
				end
				16'h006A: begin
					intv_period[7:0] <= io_din;
					intv_cnt <= {intv_period[15:8], io_din};
				end
				16'h006B: begin
					intv_period[15:8] <= io_din;
					intv_cnt <= {io_din, intv_period[7:0]};
				end
				16'hFF98: buzzer <= 0;
				default: ;
				endcase
			end
			if (io_rd && io_addr == 16'hFF98) buzzer <= 1;
			if (io_rd && io_addr == 16'h0068) begin
				intv_i <= 0;
				intv_ov <= 0;
			end
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
			4'd0: {tmout0, tmout1, tm0msk, tm1msk, sound, buzzer, pit_q} <= ss_din;
			4'd1: us_div <= ss_din[3:0];
			4'd2: free_run[7:0] <= ss_din;
			4'd3: free_run[15:8] <= ss_din;
			4'd5: {intv_dis, intv_i, intv_ov} <= ss_din[2:0];
			4'd6: intv_period[7:0] <= ss_din;
			4'd7: intv_period[15:8] <= ss_din;
			4'd8: intv_cnt[7:0] <= ss_din;
			4'd9: intv_cnt[15:8] <= ss_din;
			default: ;
			endcase
		end
	end
end

always @* begin
	case (ss_a)
	4'd0: ss_dout = {tmout0, tmout1, tm0msk, tm1msk, sound, buzzer, pit_q};
	4'd1: ss_dout = {4'd0, us_div};
	4'd2: ss_dout = free_run[7:0];
	4'd3: ss_dout = free_run[15:8];
	4'd5: ss_dout = {5'd0, intv_dis, intv_i, intv_ov};
	4'd6: ss_dout = intv_period[7:0];
	4'd7: ss_dout = intv_period[15:8];
	4'd8: ss_dout = intv_cnt[7:0];
	4'd9: ss_dout = intv_cnt[15:8];
	default: ss_dout = 8'h00;
	endcase
end

always @* begin
	io_sel = 1;
	case (io_addr)
	16'h0026: io_dout = free_run[7:0];
	16'h0027: io_dout = free_run[15:8];
	16'h0060: io_dout = {3'b111, sound, tm1msk, tm0msk, tmout1, tmout0};
	16'h0068: io_dout = {intv_dis, intv_i, intv_ov, 5'd0};
	16'h006A: io_dout = intv_period[7:0];
	16'h006B: io_dout = intv_period[15:8];
	16'h006C: io_dout = 8'h00;
	16'hFF98: io_dout = 8'hFF;
	default: begin
		io_dout = 8'hFF;
		io_sel = 0;
	end
	endcase
end

endmodule

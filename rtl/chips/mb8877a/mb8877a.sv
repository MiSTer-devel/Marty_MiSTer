// Copyright (c) 2026 Jamie Blanks
//
// MB8877A floppy disk controller (WD1793 command set). The MPU side is the
// datasheet's register file; the drive side carries the control lines by
// name and the read/write data as a byte stream from the drive model, one
// byte_ce per byte time with flags saying which part of the track it was.
//
//   A1 A0   read      write            command types
//   0  0    status    command          I   restore / seek / step (bit 7 = 0)
//   0  1    track     track            II  read / write sector (10x)
//   1  0    sector    sector           III read address / read track / write track (11x)
//   1  1    data      data             IV  force interrupt (1101)
//
// Every delay counts chip clocks (ce), so a 1 MHz clock doubles the step
// and settling times exactly as on the part. Master reset loads command
// 03 and runs a Restore.

module mb8877a
(
	input             clk,
	input             ce,             // chip clock, 2 MHz or 1 MHz
	input             reset,

	// MPU side
	input       [1:0] a,
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [7:0] d_i,
	output reg  [7:0] d_o,
	output            d_oe,
	output reg        intrq,
	output reg        drq,

	// drive side
	output reg        step,
	output reg        dirc,           // 1 = step in
	output reg        hld,
	output reg        wg,             // write gate
	output            fmt,            // write track in progress
	input             ready,
	input             ip,
	input             tr00,
	input             wprt,

	// byte stream under the head
	input             byte_ce,
	input       [7:0] rd_byte,
	input             f_index,
	input             f_am,           // an address mark byte (A1 with the missing clock)
	input             f_id,
	input             f_dam,
	input             f_data,
	input             f_data_last,
	output            field,          // inside a data field: the drive keeps streaming this one
	input       [7:0] id_c, id_h, id_r, id_n,
	output reg  [7:0] wr_byte,
	// savestate port: the registers of an idle controller as bytes
	input             ss_cs,
	input             ss_wr,
	input       [3:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output            ss_busy,        // a command is running
	output reg        wr_en
);

// ---- MPU strobes: a write takes effect on the trailing edge of WE ----
reg        wr_q, rd_q;
reg  [1:0] a_q;
reg  [7:0] d_q;
wire       wr_act = ~cs_n & ~wr_n;
wire       rd_act = ~cs_n & ~rd_n;
wire       wr_end = wr_q & ~wr_act;
wire       rd_end = rd_q & ~rd_act;
always @(posedge clk) begin
	wr_q <= wr_act;
	rd_q <= rd_act;
	if (wr_act || rd_act) begin a_q <= a; d_q <= d_i; end
end
assign d_oe = rd_act;

// ---- registers ----
reg  [7:0] cmd, track, sector, data;
reg        busy;
reg        s_wprt, s_seekerr_rnf, s_crc, s_lost, s_wf_or_dam;   // latched status bits 6, 4..2, and 5 for type II/III
reg        type1;                 // the last command was type I or IV: status shows TR00 / index

// head load timing: HLT follows HLD after the board's one-shot (3.5-inch
// drives keep the heads loaded, so the delay is short)
reg  [6:0] hlt_cnt;
wire       hlt = hlt_cnt == 7'd100;
always @(posedge clk) begin
	if (reset || !hld) hlt_cnt <= 0;
	else if (ss_cs && ss_wr && ss_a == 4'd7) hlt_cnt <= ss_din[6:0];
	else if (ce && !hlt) hlt_cnt <= hlt_cnt + 1'd1;
end

// status byte: bit 7 is the live READY input, type I bits 2:1 are live too.
// Bit 5 (head engaged) reads 0 in type I status, as on the Towns board.
always @* begin
	case (a)
	2'd0: d_o = {~ready, s_wprt, type1 ? 1'b0 : s_wf_or_dam, s_seekerr_rnf, s_crc,
	             type1 ? tr00 : s_lost, type1 ? ip : drq, busy};
	2'd1: d_o = track;
	2'd2: d_o = sector;
	default: d_o = data;
	endcase
end

// ---- command decode ----
// type I: 0 restore, 1 seek, 001u step, 010u step in, 011u step out; flags h V r1 r0
// type II: 100m read sector, 101m write sector; flags m S E C a0
// type III: C read address, E read track, F write track; flag E
wire [7:0] c   = d_q;
wire c_type1   = ~c[7];
wire c_force   = c[7:4] == 4'hD;
wire c_restore = c[7:4] == 4'h0;
wire c_seek    = c[7:4] == 4'h1;
wire c_stepin  = c[7:5] == 3'b010;
wire c_stepout = c[7:5] == 3'b011;
wire c_h       = c[3];

// step rate in chip clocks: 3 / 6 / 10 / 15 ms at 2 MHz
reg [15:0] rate;
always @* case (cmd[1:0])
	2'd0: rate = 16'd6000;
	2'd1: rate = 16'd12000;
	2'd2: rate = 16'd20000;
	default: rate = 16'd30000;
endcase
localparam [15:0] SETTLE = 16'd30000;   // 15 ms at 2 MHz

// ---- sequencer ----
localparam [4:0]
	S_IDLE     = 5'd0,
	S_T1_STEP  = 5'd1,    // decide the next step or finish
	S_T1_PULSE = 5'd2,    // step pulse high
	S_T1_RATE  = 5'd3,    // step rate delay
	S_T1_SETTLE= 5'd4,    // verify: head settle
	S_T1_VERIFY= 5'd5,    // verify: find an ID with the track number
	S_HEAD     = 5'd6,    // type II/III: E delay then wait for HLT
	S_SEARCH   = 5'd7,    // find the ID field
	S_RD_DATA  = 5'd8,    // read sector data bytes
	S_RD_END   = 5'd9,    // after the last byte: next sector or done
	S_WR_GAP   = 5'd10,   // write sector: first DRQ must be answered before the data field
	S_WR_DATA  = 5'd11,
	S_RA_OUT   = 5'd12,   // read address: six bytes out
	S_RT_INDEX = 5'd13,   // read track: wait for the index
	S_RT_DATA  = 5'd14,
	S_WT_DRQ   = 5'd15,   // write track: first DRQ
	S_WT_INDEX = 5'd16,
	S_WT_DATA  = 5'd17,
	S_WT_CRC   = 5'd18,   // second CRC byte
	S_TAIL     = 5'd19,   // the CRC bytes pass before the command moves on
	S_DONE     = 5'd20;
reg  [4:0] st;
reg [15:0] tmr;
reg  [2:0] ip_cnt;       // index pulses seen while searching
reg        ip_q;
wire       ip_rise = ip & ~ip_q;
reg  [7:0] step_left;    // restore: up to 255 steps
reg  [2:0] ra_idx;
reg  [1:0] tail_n;
reg        tail_end;     // S_TAIL continues to S_DONE, else to S_RD_END
reg  [1:0] sec_n;        // sector length code from the matched ID
reg [10:0] rd_cnt;       // data bytes taken by this read
reg        dam_ok;       // the data mark behind the matched ID has passed
wire [10:0] rd_size = 11'd128 << sec_n;
reg        i_index, i_ready0, i_ready1;   // force interrupt conditions armed
reg        i_imm;         // D8 seen: its interrupt holds until a D0 has been loaded
reg        ready_q;
reg  [3:0] ip_cnt_idle;

// CRC-16 (x^16 + x^12 + x^5 + 1), preset FFFF, fed one byte at a time.
// Used for the CRC bytes of Read Address, Read Track and Write Track.
function [15:0] crc_byte(input [15:0] crc, input [7:0] b);
	integer i;
	reg [15:0] r;
	begin
		r = crc ^ {b, 8'd0};
		for (i = 0; i < 8; i = i + 1) r = r[15] ? {r[14:0], 1'b0} ^ 16'h1021 : {r[14:0], 1'b0};
		crc_byte = r;
	end
endfunction
reg [15:0] crc;
reg  [7:0] ra_byte [0:3];

// CRC of the field under the head, restarted at each address mark; it
// reads zero on the last CRC byte of a good ID or data field. Once data
// bytes are being counted a mark pattern inside the field is data too.
reg [15:0] rd_crc;
reg        f_am_q;
wire [15:0] rd_crc_now = crc_byte(rd_crc, rd_byte);
wire        crc_good   = rd_crc_now == 16'h0000;
wire        in_field   = (st == S_RD_DATA && rd_cnt != 0) || st == S_TAIL;
assign      field      = in_field || st == S_WR_DATA;
always @(posedge clk) if (byte_ce) begin
	f_am_q <= f_am;
	rd_crc <= (f_am && !f_am_q && !in_field) ? crc_byte(16'hFFFF, rd_byte) : rd_crc_now;
end

wire id_match = f_id && id_c == track && id_r == sector && (!cmd[1] || id_h[0] == cmd[3]);

assign fmt = wg & (cmd[7:4] == 4'hF);

always @(posedge clk) begin
	ip_q    <= ip;
	ready_q <= ready;
	wr_en   <= 0;
	step    <= 0;
	if (reset) begin
		// master reset: registers clear and a Restore (03) starts
		cmd <= 8'h03; track <= 8'h00; sector <= 8'h01; data <= 8'h00;
		busy <= 1; intrq <= 0; drq <= 0; hld <= 0; wg <= 0; dirc <= 0;
		s_wprt <= 0; s_seekerr_rnf <= 0; s_crc <= 0; s_lost <= 0; s_wf_or_dam <= 0;
		type1 <= 1;
		i_index <= 0; i_ready0 <= 0; i_ready1 <= 0; i_imm <= 0;
		step_left <= 8'd255;
		st <= S_T1_STEP; tmr <= 0; ip_cnt <= 0; ip_cnt_idle <= 0;
	end
	else begin
		// ---- MPU register access ----
		if (rd_end) begin
			if (a_q == 2'd0 && !i_imm) intrq <= 0;
			if (a_q == 2'd3) drq <= 0;
		end
		if (wr_end) begin
			case (a_q)
			2'd1: track  <= d_q;
			2'd2: sector <= d_q;
			2'd3: begin data <= d_q; drq <= 0; end
			default: ;
			endcase
		end

		// force interrupt conditions armed by an earlier D0
		if (i_index && ip_rise) intrq <= 1;
		if (i_ready0 && ready && !ready_q) intrq <= 1;
		if (i_ready1 && !ready && ready_q) intrq <= 1;

		// ---- command register ----
		if (wr_end && a_q == 2'd0) begin
			if (!i_imm) intrq <= 0;
			if (c_force) begin
				// end whatever runs; the status keeps its bits, busy drops and
				// the interrupted command's data request is dropped. With
				// nothing running the status takes the type I layout.
				cmd <= c;
				busy <= 0;
				wg <= 0;
				st <= S_IDLE;
				if (busy) drq <= 0;
				else begin
					type1 <= 1;
					s_seekerr_rnf <= 0; s_crc <= 0; s_lost <= 0; s_wf_or_dam <= 0;
				end
				i_index  <= c[2];
				i_ready0 <= c[0];
				i_ready1 <= c[1];
				i_imm    <= c[3];
				if (c[3]) intrq <= 1;
			end
			else if (!busy) begin
				cmd <= c;
				busy <= 1;
				drq <= 0;
				s_seekerr_rnf <= 0; s_crc <= 0; s_lost <= 0; s_wf_or_dam <= 0; s_wprt <= 0;
				i_index <= 0; i_ready0 <= 0; i_ready1 <= 0;
				ip_cnt <= 0;
				tmr <= 0;
				if (c_type1) begin
					type1 <= 1;
					hld <= c_h;
					if (c_restore) begin step_left <= 8'd255; dirc <= 0; data <= 8'h00; end
					if (c_seek) dirc <= data > track;
					if (c_stepin) dirc <= 1;
					if (c_stepout) dirc <= 0;
					st <= S_T1_STEP;
				end
				else begin
					// types II and III need a ready drive
					type1 <= 0;
					if (!ready) st <= S_DONE;
					else begin
						hld <= 1;
						st  <= S_HEAD;
					end
				end
			end
		end

		// ---- command execution ----
		case (st)
		S_IDLE: ;

		// type I: one step per pass until the target is reached. Stepping out
		// with TR00 already active issues no pulse and loads track 0 instead.
		S_T1_STEP: begin
			if (cmd[7:4] == 4'h0) begin                 // restore
				if (tr00) begin track <= 8'h00; st <= cmd[2] ? S_T1_SETTLE : S_DONE; end
				else if (step_left == 0) begin s_seekerr_rnf <= 1; st <= S_DONE; end
				else begin step_left <= step_left - 1'd1; st <= S_T1_PULSE; end
			end
			else if (cmd[7:4] == 4'h1) begin            // seek
				if (track == data) st <= cmd[2] ? S_T1_SETTLE : S_DONE;
				else if (data < track && tr00) begin track <= 8'h00; st <= cmd[2] ? S_T1_SETTLE : S_DONE; end
				else begin
					dirc  <= data > track;
					track <= (data > track) ? track + 1'd1 : track - 1'd1;
					st <= S_T1_PULSE;
				end
			end
			else if (!dirc && tr00) begin               // step out at track 0
				track <= 8'h00;
				st <= cmd[2] ? S_T1_SETTLE : S_DONE;
			end
			else begin                                  // step, step in, step out: one pulse
				if (cmd[4]) track <= dirc ? track + 1'd1 : track - 1'd1;
				st <= S_T1_PULSE;
			end
		end
		S_T1_PULSE: begin
			step <= 1;
			tmr  <= 0;
			st   <= S_T1_RATE;
		end
		S_T1_RATE: if (ce) begin
			if (tmr == rate - 1'd1) begin
				tmr <= 0;
				// single steps end here; restore and seek go round again
				if (cmd[7:5] == 3'b000) st <= S_T1_STEP;
				else st <= cmd[2] ? S_T1_SETTLE : S_DONE;
			end
			else tmr <= tmr + 1'd1;
		end
		S_T1_SETTLE: if (ce) begin
			hld <= 1;   // verify needs the head down whatever h said
			if (tmr == SETTLE - 1'd1) begin tmr <= 0; ip_cnt <= 0; st <= S_T1_VERIFY; end
			else tmr <= tmr + 1'd1;
		end
		S_T1_VERIFY: begin
			if (!ready) st <= S_DONE;
			else if (byte_ce && f_id && id_c == track) st <= S_DONE;
			else if (ip_rise) begin
				if (ip_cnt == 3'd4) begin s_seekerr_rnf <= 1; st <= S_DONE; end
				else ip_cnt <= ip_cnt + 1'd1;
			end
		end

		// type II / III: optional 15 ms delay, then the head must be loaded
		S_HEAD: if (ce) begin
			if (cmd[2] && tmr != SETTLE - 1'd1) tmr <= tmr + 1'd1;
			else if (hlt) begin
				tmr <= 0;
				ip_cnt <= 0;
				if (cmd[7:4] == 4'hC) begin ra_idx <= 0; st <= S_SEARCH; end
				else if (cmd[7:4] == 4'hE) st <= S_RT_INDEX;
				else if (cmd[7:4] == 4'hF) begin
					if (wprt) begin s_wprt <= 1; st <= S_DONE; end
					else begin drq <= 1; tmr <= 0; st <= S_WT_DRQ; end
				end
				else if (cmd[5] && wprt) begin s_wprt <= 1; st <= S_DONE; end
				else st <= S_SEARCH;
			end
		end

		// find the ID field: any ID for Read Address, the matching one otherwise
		S_SEARCH: begin
			if (!ready) st <= S_DONE;
			else if (byte_ce && f_id && (cmd[7:4] == 4'hC || id_match)) begin
				// a bad ID CRC flags and, for a sector command, is not a match
				if (!crc_good) s_crc <= 1;
				if (cmd[7:4] == 4'hC) begin
					// Read Address hands out C H R N and the ID CRC
					ra_byte[0] <= id_c; ra_byte[1] <= id_h; ra_byte[2] <= id_r; ra_byte[3] <= id_n;
					crc <= crc_byte(crc_byte(crc_byte(crc_byte(crc_byte(16'hCDB4, 8'hFE), id_c), id_h), id_r), id_n);
					ra_idx <= 0;
					st <= S_RA_OUT;
				end
				else if (crc_good) begin
					sec_n  <= id_n[7:2] != 0 ? 2'd3 : id_n[1:0];
					rd_cnt <= 0;
					dam_ok <= 0;
					if (cmd[5]) begin
						// write: the first byte must arrive before the data field
						drq <= 1;
						st  <= S_WR_GAP;
					end
					else st <= S_RD_DATA;
				end
			end
			else if (ip_rise) begin
				if (ip_cnt == 3'd4) begin s_seekerr_rnf <= 1; st <= S_DONE; end
				else ip_cnt <= ip_cnt + 1'd1;
			end
		end

		// read sector: each data byte lands in DR, lost if the last was not
		// taken; the chip counts the field from the length code in the ID
		S_RD_DATA: if (byte_ce) begin
			if (f_dam && rd_cnt == 0) begin dam_ok <= 1; s_wf_or_dam <= rd_byte == 8'hF8; end
			if (f_data && dam_ok) begin
				if (drq) s_lost <= 1;
				else data <= rd_byte;
				drq    <= 1;
				rd_cnt <= rd_cnt + 1'd1;
				if (rd_cnt == rd_size - 1'd1) begin tail_n <= 2'd2; tail_end <= 0; st <= S_TAIL; end
			end
		end
		S_TAIL: if (byte_ce) begin
			if (tail_n == 2'd1) begin
				wg <= 0;
				// a read's data CRC ends the command when it fails
				if (!wg && !tail_end && !crc_good) begin s_crc <= 1; st <= S_DONE; end
				else st <= tail_end ? S_DONE : S_RD_END;
			end
			else tail_n <= tail_n - 1'd1;
		end
		S_RD_END: begin
			if (cmd[4]) begin sector <= sector + 1'd1; ip_cnt <= 0; st <= S_SEARCH; end
			else st <= S_DONE;
		end

		// write sector: WG opens at the data field; a byte not supplied writes
		// as 00. The last byte raises no request: the CRC follows it.
		S_WR_GAP: if (byte_ce) begin
			if (f_dam) begin dam_ok <= 1; wg <= 1; wr_byte <= cmd[0] ? 8'hF8 : 8'hFB; wr_en <= 1; end
			if (f_data && dam_ok) begin
				if (drq) begin s_lost <= 1; drq <= 0; st <= S_DONE; end   // unanswered request is dropped
				else begin
					wg <= 1;
					wr_byte <= data; wr_en <= 1;
					drq <= ~f_data_last;
					if (f_data_last) begin tail_n <= 2'd2; tail_end <= 0; st <= S_TAIL; end
					else st <= S_WR_DATA;
				end
			end
		end
		S_WR_DATA: if (byte_ce && f_data) begin
			if (drq) begin s_lost <= 1; wr_byte <= 8'h00; end
			else wr_byte <= data;
			wr_en <= 1;
			drq   <= ~f_data_last;
			if (f_data_last) begin tail_n <= 2'd2; tail_end <= 0; st <= S_TAIL; end
		end

		// read address: six bytes at the byte rate; the sector register takes C
		S_RA_OUT: if (byte_ce) begin
			if (drq) s_lost <= 1;
			case (ra_idx)
			3'd0, 3'd1, 3'd2, 3'd3: data <= ra_byte[ra_idx[1:0]];
			3'd4: data <= crc[15:8];
			default: data <= crc[7:0];
			endcase
			drq <= 1;
			if (ra_idx == 3'd5) begin sector <= ra_byte[0]; tail_n <= 2'd1; tail_end <= 1; st <= S_TAIL; end
			else ra_idx <= ra_idx + 1'd1;
		end

		// read track: everything from one index pulse to the next
		S_RT_INDEX: begin
			if (!ready) st <= S_DONE;
			else if (byte_ce && f_index) begin data <= rd_byte; drq <= 1; st <= S_RT_DATA; end
		end
		S_RT_DATA: if (byte_ce) begin
			if (f_index) begin tail_n <= 2'd1; tail_end <= 1; st <= S_TAIL; end
			else begin
				if (drq) s_lost <= 1;
				else data <= rd_byte;
				drq <= 1;
			end
		end

		// write track: the MPU supplies every byte; F5/F6 are the A1/C2 marks,
		// F7 writes the two CRC bytes
		// the three-byte-time window runs off the chip clock (32 clocks per
		// byte at either rate), so it still expires when the drive is
		// deselected under the command
		S_WT_DRQ: begin
			if (ce) begin
				if (tmr == 16'd96) begin s_lost <= 1; drq <= 0; st <= S_DONE; end
				else tmr <= tmr + 1'd1;
			end
			if (!drq) st <= S_WT_INDEX;
		end
		S_WT_INDEX: begin
			if (!ready) st <= S_DONE;
			else if (byte_ce && f_index) begin wg <= 1; crc <= 16'hFFFF; st <= S_WT_DATA; end
		end
		S_WT_DATA: if (byte_ce) begin
			if (f_index) begin wg <= 0; st <= S_DONE; end
			else begin
				wr_en <= 1;
				if (drq) begin s_lost <= 1; wr_byte <= 8'h00; crc <= crc_byte(crc, 8'h00); end
				else case (data)
				8'hF5: begin wr_byte <= 8'hA1; crc <= 16'hCDB4; end
				8'hF6: begin wr_byte <= 8'hC2; crc <= crc_byte(crc, 8'hC2); end
				8'hF7: begin wr_byte <= crc[15:8]; st <= S_WT_CRC; end
				default: begin wr_byte <= data; crc <= crc_byte(crc, data); end
				endcase
				if (data != 8'hF7 || drq) drq <= 1;
			end
		end
		S_WT_CRC: if (byte_ce) begin
			wr_en   <= 1;
			wr_byte <= crc[7:0];
			crc     <= 16'hFFFF;
			drq     <= 1;
			st <= f_index ? S_DONE : S_WT_DATA;
			if (f_index) wg <= 0;
		end

		// busy holds until the last data request has been taken
		S_DONE: if (!drq) begin
			busy  <= 0;
			intrq <= 1;
			wg    <= 0;
			st    <= S_IDLE;
		end
		default: st <= S_IDLE;
		endcase

		// type I and IV status shows the live write protect line
		if (type1) s_wprt <= wprt;

		// head unloads after fifteen index pulses with no command
		if (busy) ip_cnt_idle <= 0;
		else if (ip_rise) begin
			if (ip_cnt_idle == 4'd14) hld <= 0;
			else ip_cnt_idle <= ip_cnt_idle + 1'd1;
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
			4'd0: cmd <= ss_din;
			4'd1: track <= ss_din;
			4'd2: sector <= ss_din;
			4'd3: data <= ss_din;
			4'd4: {s_wprt, s_seekerr_rnf, s_crc, s_lost, s_wf_or_dam, type1, intrq, drq} <= ss_din;
			4'd5: {hld, dirc, i_index, i_ready0, i_ready1, i_imm} <= ss_din[5:0];
			4'd6: ip_cnt_idle <= ss_din[3:0];
			default: ;
			endcase
		end
	end
end

assign ss_busy = busy;
always @* begin
	case (ss_a)
	4'd0: ss_dout = cmd;
	4'd1: ss_dout = track;
	4'd2: ss_dout = sector;
	4'd3: ss_dout = data;
	4'd4: ss_dout = {s_wprt, s_seekerr_rnf, s_crc, s_lost, s_wf_or_dam, type1, intrq, drq};
	4'd5: ss_dout = {2'd0, hld, dirc, i_index, i_ready0, i_ready1, i_imm};
	4'd6: ss_dout = {4'd0, ip_cnt_idle};
	4'd7: ss_dout = {1'b0, hlt_cnt};
	4'd8: ss_dout = {7'd0, busy};
	default: ss_dout = 8'h00;
	endcase
end

endmodule

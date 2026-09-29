// Copyright (c) 2026 Jamie Blanks
//
// The CD-ROM behind the Towns drive model, served by Linux as raw
// 2352-byte sectors on one OSD slot. The table of contents arrives as a
// record on its own download index whenever a disc is mounted or ejected
// (track starts, then from byte 512 the pause starts); from then on every
// drive request is answered here:
//
//   TUR / TOC   answered from the record, err when no disc
//   READ        one sector into the slot buffer, bytes 12..2351 copied to
//               the CDC sector RAM as words (the sector without its sync)
//   PLAY        sectors streamed as 44.1 kHz stereo samples, the next
//               sector fetched into the other buffer half while one plays;
//               the 96 subcode bytes Linux appends to each sector go out
//               one per frame for the CDC's 04CD, the first replaced by
//               the 1F block marker the ROM's CD player frames on;
//               silence while the head is in a data track, as the drive
//               mutes over one
//               the status stays "playing" for END_HOLD after the range
//               ends, the head already at the end address: the CD BIOS
//               hands a game the last sub-Q it read while playing, so that
//               reading has to be the end
//   PAUSE / RESUME / STOP
//   SEEK        the head moves, audio ends
//   SUBQ        control, track, index and MSF worked out from the head
//               position and the TOC; in a track's pause (index 00) the
//               relative time counts down, in the lead-out the track is AA
//
// Sector numbers from the drive model count from the first data sector;
// Linux counts from 00:00:00, 150 sectors earlier.

module hps_cd_host #(parameter CLK_RATE = 57272727)
(
	input             clk,
	input             reset,

	// TOC record download
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input       [9:0] ioctl_addr,
	input       [7:0] ioctl_dout,

	// block slot, 2 x 2352 bytes
	input             img_present,
	output reg [23:0] req_lba,
	output reg        req_bank,
	output reg        req_rd,
	input             blk_done,
	input             blk_err,
	output reg [12:0] buf_addr,
	input       [7:0] buf_dout,

	// requests from the drive model
	input             req,
	input       [3:0] req_op,
	input      [23:0] h_lba,
	input      [23:0] h_lba_end,
	output reg        busy,
	output reg        done,
	output reg        err,

	// table of contents: {ctrl, M, S, F}, one clock after toc_addr
	input       [6:0] toc_addr,
	output     [31:0] toc_data,
	output reg  [7:0] toc_first,
	output reg  [7:0] toc_last,
	output reg [99:0] toc_mode2,

	// CDC sector RAM: 1170 words, the sector from its header on
	output reg        sec_we,
	output reg [10:0] sec_addr,
	output reg [15:0] sec_data,

	// sub-channel Q
	output reg  [7:0] sq_status,   // 0x11 playing, 0x12 paused, 0x13 stopped
	output reg  [7:0] sq_ctrl,     // control nibble and q-mode 1
	output reg  [7:0] sq_track,    // AA in the lead-out
	output reg  [7:0] sq_index,    // 0 in a pause, else 1
	output reg  [7:0] sq_abs_m, sq_abs_s, sq_abs_f,
	output reg  [7:0] sq_rel_m, sq_rel_s, sq_rel_f,

	// CD audio
	output reg        cdda_ce,
	output reg [15:0] cdda_l,
	output reg [15:0] cdda_r,

	// subcode byte stream while playing
	output reg        sub_we,
	output reg  [7:0] sub_data,
	input             hold,            // the machine stands still: the audio waits too

	// savestate port: the playback position; writing 06h with bit 7 set
	// fetches the sector at the head again and resumes the audio there
	input             ss_cs,
	input             ss_wr,
	input       [3:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output            ss_busy          // that resume is still on its way
);

localparam [23:0] END_HOLD = CLK_RATE / 25;   // 40 ms
localparam [3:0] OP_TUR = 4'd0, OP_TOC = 4'd1, OP_READ = 4'd2, OP_PLAY = 4'd3,
                 OP_PAUSE = 4'd4, OP_RESUME = 4'd5, OP_STOP = 4'd6, OP_SUBQ = 4'd7, OP_SEEK = 4'd8;

// ---- table of contents ----
// Linux sends the record while it still holds the machine in reset, so
// nothing here is cleared by that reset: an empty record ejects the disc
// and the registers start from the FPGA's power-up zero.
reg        disc;
reg [23:0] toc_bytes;      // F, S, M of the entry being received
reg        toc_we;
reg  [6:0] toc_waddr;
reg [31:0] toc_wdata;

wire toc_load = ioctl_download && ioctl_index == 16'd250;

// the host's own lookups share the download port; a scan pauses during a load
reg  [6:0] scan_addr;
wire [31:0] scan_q;
wire [23:0] scan0_q;
reg        toc0_we;
reg        toc0_valid;     // the record carried index 0 starts (bytes 512 on)

cache_ram_dp #(.ADDR_WIDTH(7), .DATA_WIDTH(32)) toc
(
	.clk_i(clk),
	.addr_a_i(toc_we ? toc_waddr : scan_addr), .wren_a_i(toc_we), .wdata_a_i(toc_wdata), .q_a_o(scan_q),
	.addr_b_i(toc_addr), .wren_b_i(1'b0), .wdata_b_i(32'd0), .q_b_o(toc_data)
);

// where each track's pause (index 00) starts, {M, S, F}
cache_ram_dp #(.ADDR_WIDTH(7), .DATA_WIDTH(24)) toc0
(
	.clk_i(clk),
	.addr_a_i(toc0_we ? toc_waddr : scan_addr), .wren_a_i(toc0_we), .wdata_a_i(toc_bytes), .q_a_o(scan0_q),
	.addr_b_i(7'd0), .wren_b_i(1'b0), .wdata_b_i(24'd0), .q_b_o()
);

integer b;
always @(posedge clk) begin
	toc_we  <= 0;
	toc0_we <= 0;
	if (toc_load && ioctl_wr) begin
		if (ioctl_addr == 10'd0) begin toc_first <= ioctl_dout; toc0_valid <= 0; end
		else if (ioctl_addr == 10'd1) toc_last <= ioctl_dout;
		else if (ioctl_addr == 10'd2) disc <= ioctl_dout[0];
		else if (ioctl_addr >= 10'd4 && ioctl_addr < 10'd408) begin
			case (ioctl_addr[1:0])
			2'd0: toc_bytes[7:0]   <= ioctl_dout;
			2'd1: toc_bytes[15:8]  <= ioctl_dout;
			2'd2: toc_bytes[23:16] <= ioctl_dout;
			default: begin
				toc_we    <= 1;
				toc_waddr <= ioctl_addr[8:2] - 7'd1;
				toc_wdata <= {ioctl_dout, toc_bytes};
			end
			endcase
		end
		else if (ioctl_addr >= 10'd408 && ioctl_addr < 10'd421) begin
			// bitmap byte k covers tracks 8k .. 8k+7
			for (b = 0; b < 100; b = b + 1)
				if (b[9:3] == ioctl_addr[6:0] - 7'd24) toc_mode2[b] <= ioctl_dout[b[2:0]];
		end
		else if (ioctl_addr >= 10'd516 && ioctl_addr < 10'd912) begin
			// index 0 of track t at 512 + 4t, the entry layout again
			case (ioctl_addr[1:0])
			2'd0: toc_bytes[7:0]   <= ioctl_dout;
			2'd1: toc_bytes[15:8]  <= ioctl_dout;
			2'd2: toc_bytes[23:16] <= ioctl_dout;
			default: begin
				toc0_we    <= 1;
				toc0_valid <= 1;
				toc_waddr  <= ioctl_addr[8:2];
			end
			endcase
		end
	end
end

// ---- 44.1 kHz sample tick ----
reg [31:0] acc;
always @(posedge clk) begin
	cdda_ce <= 0;
	if (reset) acc <= 0;
	else if (acc + 32'd44100 >= CLK_RATE) begin
		acc     <= acc + 32'd44100 - CLK_RATE;
		cdda_ce <= 1;
	end
	else acc <= acc + 32'd44100;
end

// ---- minutes, seconds, frames of a TOC entry as a sector number ----
function [23:0] msf_lba(input [31:0] e);
	reg [23:0] m, sec;
	begin
		m   = {16'd0, e[23:16]};
		sec = {16'd0, e[15:8]};
		msf_lba = (m << 12) + (m << 8) + (m << 7) + (m << 4) + (m << 2)   // m * 4500
		        + (sec << 6) + (sec << 3) + (sec << 1) + sec              // s * 75
		        + {16'd0, e[7:0]};
	end
endfunction

// ---- request engine ----
localparam [3:0] S_IDLE = 4'd0, S_FETCH = 4'd1, S_COPY = 4'd2, S_PLAY_FETCH = 4'd3,
                 S_SCAN = 4'd4, S_SCAN_W = 4'd5, S_SCAN_Q = 4'd6, S_ABS = 4'd7, S_REL = 4'd8, S_FINISH = 4'd9;

reg  [3:0] state;
reg        req_pend;       // a drive request waiting for the engine
reg  [3:0] op;
reg [23:0] msf_rem;        // serial sector number to M, S, F
reg  [7:0] msf_m, msf_s;
reg        msf_run;
reg [23:0] pos;            // head position, absolute
reg [23:0] play_end;       // absolute, exclusive
reg        playing;
reg [23:0] end_hold;       // clocks of "playing" status left after the range ended
reg        fetching;       // a play sector is on its way into ~play_bank
reg        sector_done;    // the playing sector ended before the next landed
reg        resume_pend, resuming;   // savestate: fetch the head sector and play on
reg        next_ready;
reg        play_bank;
reg  [9:0] sample;         // 0..587 inside the playing sector
reg  [3:0] sample_step;
reg  [6:0] sub_idx;        // subcode bytes of the sector already sent, 0..96
reg  [2:0] sub_cnt;        // samples since the last subcode frame slot
reg [11:0] copy_idx;       // byte inside the sector during the copy
reg  [1:0] copy_phase;
reg  [6:0] scan_trk;       // 1 .. last track, then 100 for the lead-out
reg  [6:0] found_trk;
reg [23:0] found_lba;
reg  [3:0] found_ctrl;
reg        found_data;     // the head's track is a data track
reg [23:0] next_lba;       // start of the track after the head's
reg [23:0] next_idx0;      // where its pause starts
reg  [6:0] next_trk;
reg  [3:0] next_ctrl;
reg        next_found;
wire       in_pause = next_found && pos >= next_idx0;
reg        scan_play;      // the scan precedes a play, not a sub-Q answer
reg [23:0] mute_end;       // absolute, exclusive: sectors below it play as silence
reg        muted;

wire [11:0] sample_byte = {sample, 2'b00};

always @(posedge clk) begin
	done   <= 0;
	sec_we <= 0;
	req_rd <= 0;
	if (reset) begin
		state     <= S_IDLE;
		busy      <= 0;
		err       <= 0;
		pos       <= 0;
		playing   <= 0;
		end_hold  <= 0;
		fetching  <= 0;
		sector_done <= 0;
		resume_pend <= 0;
		resuming  <= 0;
		next_ready <= 0;
		play_bank <= 0;
		sample    <= 0;
		sample_step <= 0;
		sub_idx   <= 0;
		sub_cnt   <= 0;
		sub_we    <= 0;
		sq_status <= 8'h13;
		sq_ctrl   <= 8'h01;
		sq_track  <= 8'd1;
		sq_index  <= 8'd1;
		{sq_abs_m, sq_abs_s, sq_abs_f} <= 24'd0;
		{sq_rel_m, sq_rel_s, sq_rel_f} <= 24'd0;
		cdda_l    <= 0;
		cdda_r    <= 0;
		msf_run   <= 0;
		req_bank  <= 0;
		buf_addr  <= 0;
		req_pend  <= 0;
	end
	else begin
		if (req) req_pend <= 1;
		if (end_hold != 0) begin
			end_hold <= end_hold - 1'd1;
			if (end_hold == 24'd1) sq_status <= 8'h13;
		end

		if (msf_run) begin
			if (msf_rem >= 24'd4500) begin msf_rem <= msf_rem - 24'd4500; msf_m <= msf_m + 1'd1; end
			else if (msf_rem >= 24'd75) begin msf_rem <= msf_rem - 24'd75; msf_s <= msf_s + 1'd1; end
			else msf_run <= 0;
		end

		// ---- audio: four buffer bytes per sample, two clocks behind the
		// address; every sixth sample a subcode byte follows the same way ----
		sub_we <= 0;
		if (sample_step != 0) begin
			sample_step <= sample_step + 1'd1;
			case (sample_step)
			4'd1: buf_addr <= {play_bank, sample_byte};
			4'd2: buf_addr <= {play_bank, sample_byte + 12'd1};
			4'd3: begin buf_addr <= {play_bank, sample_byte + 12'd2}; cdda_l[7:0]  <= muted ? 8'd0 : buf_dout; end
			4'd4: begin buf_addr <= {play_bank, sample_byte + 12'd3}; cdda_l[15:8] <= muted ? 8'd0 : buf_dout; end
			4'd5: cdda_r[7:0] <= muted ? 8'd0 : buf_dout;
			4'd6: begin
				cdda_r[15:8] <= muted ? 8'd0 : buf_dout;
				if (sample == 10'd587) begin
					// the pickup parks at the end address, so sub-Q reads there
					// (the next track's index 1); games wait for that position
					if (pos + 1'd1 >= play_end) begin playing <= 0; pos <= play_end; end_hold <= END_HOLD; end
					else sector_done <= 1;
				end
				else sample <= sample + 1'd1;
				// 588 samples make 98 frame slots; the 96 bytes fill the first
				sub_cnt <= sub_cnt == 3'd5 ? 3'd0 : sub_cnt + 1'd1;
				if (sub_cnt != 3'd5 || sub_idx >= 7'd96) sample_step <= 0;
			end
			4'd7: buf_addr <= {play_bank, 12'd2352 + {5'd0, sub_idx}};
			4'd8: ;
			default: begin
				sub_data <= sub_idx == 7'd0 ? 8'h1F : buf_dout;
				sub_we   <= 1;
				sub_idx  <= sub_idx + 1'd1;
				sample_step <= 0;
			end
			endcase
		end
		// the next sector normally landed long ago; when Linux is late the
		// audio waits in silence rather than the drive stopping
		// (after the last sample's subcode byte, which reads the old bank)
		if (sector_done && next_ready && sample_step == 0) begin
			sector_done <= 0;
			next_ready  <= 0;
			sample      <= 0;
			sub_idx     <= 0;
			sub_cnt     <= 0;
			pos         <= pos + 1'd1;
			muted       <= pos + 1'd1 < mute_end;
			play_bank   <= ~play_bank;
		end
		if (playing && cdda_ce && sample_step == 0 && !sector_done && !hold) sample_step <= 4'd1;
		if (!playing || (sector_done && !next_ready)) begin cdda_l <= 0; cdda_r <= 0; end
		if (!playing) sector_done <= 0;
		if (!disc) playing <= 0;

		// the sector after the one playing lands in the other half
		if (playing && !fetching && !next_ready && !req_pend && pos + 1'd1 < play_end && state == S_IDLE) begin
			req_lba  <= pos + 24'd1;
			req_bank <= ~play_bank;
			req_rd   <= 1;
			fetching <= 1;
		end
		if (fetching && blk_done) begin
			fetching   <= 0;
			next_ready <= !blk_err;
		end

		case (state)
		S_IDLE: if (resume_pend && !fetching && !req_pend) begin
			resume_pend <= 0;
			resuming    <= 1;
			next_ready  <= 0;
			scan_trk <= 7'd1; found_data <= 0; next_found <= 0; scan_play <= 1;
			state    <= disc ? S_SCAN : S_IDLE;
		end
		else if (req_pend && !fetching) begin
			req_pend <= 0;
			busy <= 1;
			err  <= 0;
			op   <= req_op;
			if (req_op != OP_SUBQ) end_hold <= 0;
			case (req_op)
			OP_TUR, OP_TOC: begin err <= !disc; state <= S_FINISH; end
			OP_READ: begin
				playing  <= 0;
				sq_status <= 8'h13;
				pos      <= h_lba + 24'd150;
				req_lba  <= h_lba + 24'd150;
				req_bank <= 0;
				req_rd   <= 1;
				err      <= !disc;
				state    <= disc ? S_FETCH : S_FINISH;
			end
			OP_PLAY: begin
				pos       <= h_lba + 24'd150;
				play_end  <= h_lba_end + 24'd150;
				sample    <= 0;
				sub_idx   <= 0;
				sub_cnt   <= 0;
				play_bank <= 0;
				next_ready <= 0;
				err      <= !disc;
				scan_trk <= 7'd1; found_data <= 0; next_found <= 0; scan_play <= 1;
				state    <= disc ? S_SCAN : S_FINISH;
			end
			OP_PAUSE:  begin playing <= 0; if (sq_status == 8'h11) sq_status <= 8'h12; state <= S_FINISH; end
			OP_RESUME: begin if (sq_status == 8'h12 && pos < play_end) begin playing <= 1; sq_status <= 8'h11; end state <= S_FINISH; end
			OP_STOP:   begin playing <= 0; sq_status <= 8'h13; state <= S_FINISH; end
			OP_SEEK:   begin playing <= 0; sq_status <= 8'h13; pos <= h_lba + 24'd150; state <= S_FINISH; end
			default: begin scan_trk <= 7'd1; found_trk <= 7'd1; found_lba <= 0; found_ctrl <= 0; next_found <= 0; scan_play <= 0; state <= S_SCAN; end
			endcase
		end

		S_FETCH: if (blk_done) begin
			if (blk_err) begin err <= 1; state <= S_FINISH; end
			else begin copy_idx <= 12'd12; copy_phase <= 0; buf_addr <= 13'd12; state <= S_COPY; end
		end

		// bytes 12.. of the buffer to CDC words; the RAM answers a clock late
		S_COPY: begin
			copy_phase <= copy_phase + 1'd1;
			case (copy_phase)
			2'd0: buf_addr <= {1'b0, copy_idx} + 13'd1;
			2'd1: sec_data[7:0] <= buf_dout;
			2'd2: begin
				sec_data[15:8] <= buf_dout;
				sec_we   <= 1;
				sec_addr <= copy_idx[11:1] - 11'd6;
				copy_idx <= copy_idx + 12'd2;
				buf_addr <= {1'b0, copy_idx} + 13'd2;
				copy_phase <= 0;
				if (copy_idx == 12'd2350) state <= S_FINISH;
			end
			default: ;
			endcase
		end

		S_PLAY_FETCH: if (blk_done) begin
			if (blk_err) begin err <= 1; sq_status <= 8'h13; end
			else begin playing <= 1; sq_status <= 8'h11; end
			state <= S_FINISH;
		end

		// the last track starting at or before the head: sub-Q answers
		// from it, a play starting in a data track stays silent until the
		// track after it
		S_SCAN: if (!toc_load) begin
			scan_addr <= scan_trk;
			state <= S_SCAN_W;
		end
		S_SCAN_W: state <= S_SCAN_Q;
		S_SCAN_Q: begin
			if (msf_lba(scan_q) <= pos) begin
				found_trk <= scan_trk; found_lba <= msf_lba(scan_q); found_ctrl <= scan_q[27:24]; found_data <= scan_q[26];
			end
			else if (!next_found) begin
				next_lba <= msf_lba(scan_q); next_trk <= scan_trk; next_ctrl <= scan_q[27:24]; next_found <= 1;
				next_idx0 <= (toc0_valid && scan_trk != 7'd100) ? msf_lba({8'd0, scan0_q}) : msf_lba(scan_q);
			end
			if (scan_trk == 7'd100) begin
				if (scan_play) begin
					mute_end <= !found_data ? 24'd0 : next_found ? next_lba : play_end;
					muted    <= found_data;
					req_lba  <= pos;
					req_bank <= play_bank;
					req_rd   <= 1;
					state    <= S_PLAY_FETCH;
				end
				else begin
					msf_rem <= pos; msf_m <= 0; msf_s <= 0; msf_run <= 1;
					state <= S_ABS;
				end
			end
			else begin scan_trk <= scan_trk >= toc_last[6:0] ? 7'd100 : scan_trk + 1'd1; state <= S_SCAN; end
		end
		// a pause belongs to the track after it: index 00, time counting
		// down to zero in its last sector
		S_ABS: if (!msf_run) begin
			sq_track <= in_pause ? {1'b0, next_trk} : found_trk == 7'd100 ? 8'hAA : {1'b0, found_trk};
			sq_index <= in_pause ? 8'd0 : 8'd1;
			sq_ctrl  <= {in_pause ? next_ctrl : found_ctrl, 4'h1};
			sq_abs_m <= msf_m; sq_abs_s <= msf_s; sq_abs_f <= msf_rem[7:0];
			msf_rem <= in_pause ? next_lba - pos - 24'd1 : pos - found_lba; msf_m <= 0; msf_s <= 0; msf_run <= 1;
			state <= S_REL;
		end
		S_REL: if (!msf_run) begin
			sq_rel_m <= msf_m; sq_rel_s <= msf_s; sq_rel_f <= msf_rem[7:0];
			state <= S_FINISH;
		end

		S_FINISH: begin
			busy     <= 0;
			done     <= !resuming;   // a savestate resume answers nobody
			resuming <= 0;
			state    <= S_IDLE;
		end

		default: state <= S_IDLE;
		endcase
		if (ss_cs && ss_wr) begin
			case (ss_a)
			4'h0: pos[7:0] <= ss_din;        4'h1: pos[15:8] <= ss_din;        4'h2: pos[23:16] <= ss_din;
			4'h3: play_end[7:0] <= ss_din;   4'h4: play_end[15:8] <= ss_din;   4'h5: play_end[23:16] <= ss_din;
			4'h6: begin play_bank <= ss_din[0]; playing <= 0; resume_pend <= ss_din[7]; end
			4'h7: begin sq_status <= ss_din; if (ss_din == 8'h11 && !resume_pend) end_hold <= END_HOLD; end
			4'h8: sample[7:0] <= ss_din;     4'h9: sample[9:8] <= ss_din[1:0];
			4'hA: sq_track <= ss_din;
			default: ;
			endcase
		end
	end
end

assign ss_busy = resume_pend || resuming;
always @* begin
	case (ss_a)
	4'h0: ss_dout = pos[7:0];        4'h1: ss_dout = pos[15:8];        4'h2: ss_dout = pos[23:16];
	4'h3: ss_dout = play_end[7:0];   4'h4: ss_dout = play_end[15:8];   4'h5: ss_dout = play_end[23:16];
	4'h6: ss_dout = {playing, 6'd0, play_bank};
	4'h7: ss_dout = sq_status;
	4'h8: ss_dout = sample[7:0];     4'h9: ss_dout = {6'd0, sample[9:8]};
	4'hA: ss_dout = sq_track;
	default: ss_dout = 8'h00;
	endcase
end

endmodule

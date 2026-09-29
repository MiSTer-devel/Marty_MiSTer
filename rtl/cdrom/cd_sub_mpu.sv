// Copyright (c) 2026 Jamie Blanks
//
// The CD sub-MPU (MB88505H) and the drive behind it, as one behavioural
// model. It takes a command byte and eight parameters from the CDC, runs
// the drive through the ATAPI host, and answers with four-byte status
// groups pushed into the CDC status FIFO. Timing follows the drive, not
// the framework: DRY drops for ACCEPT_US after a command, sectors arrive at
// the 1x rate, seeks take a distance-dependent time.
//
//   command ─> 1 ms ─> dispatch ──┬─ seek:  status 00, audio ends, wait, status 04
//                                 ├─ read:  track lookup, status 00 (21 05 on audio), per sector 22 / DTS / DEI, 06 at the end
//                                 ├─ toc:   00 then 16/17 pairs from the TOC RAM
//                                 ├─ subq:  00, 18, 19, 19, 20 from READ SUBCHANNEL
//                                 └─ play / pause / resume / stop / state
//
// Status second byte: 9 no disc, 1 paused, 3 playing, else 0.
//
// A mount or eject from the OSD is a tray event: the disc goes away, the
// door closes DOOR_US later and the drive probes for what is in it.

module cd_sub_mpu #(
	parameter ACCEPT_US    = 1000,     // command to DRY and its first status
	parameter SECTOR_US    = 13333,    // 1x: 75 sectors per second
	parameter SEEK_BASE_US = 20000,
	parameter LOSTDATA_US  = 100000,   // DTS/STS must follow data ready within this
	parameter NOTIFY_US    = 1000,
	parameter DOOR_US      = 500000,   // tray event to the drive looking for the disc
	parameter [1:0] READ_RETRIES = 2'd3
)
(
	input             clk,
	input             ce_us,           // 1 MHz tick
	input             reset,           // includes SRST

	// command from the CDC
	input             cmd_strobe,
	input       [7:0] cmd,
	input      [63:0] params,          // {p7 .. p0}

	// status FIFO in the CDC
	output reg        st_push,
	output reg [31:0] st_data,         // {b0, b1, b2, b3}, b0 read first
	output reg        st_clear,
	input             st_full,

	// flags in the CDC
	output reg        dry,
	output reg        sirq_set,        // set the SIRQ flag: the interrupt request
	output reg        sirq_clr,
	input             dei,             // DMA-end pending
	input             irq_line,        // interrupt line as the PIC sees it
	input             hold,            // savestate in progress

	// sector transfer handshake with the CDC
	output reg        data_ready,      // a sector waits for DTS / STS
	output reg  [1:0] sector_form,     // 0 2048, 1 2336 (mode 2), 2 2340 (raw)
	output reg        sector_mode2,    // the track being read is mode 2
	input             xfer_start,      // DTS or STS written
	input             xfer_done,
	output reg        xfer_abort,

	// ATAPI host
	output reg        h_req,
	output reg  [3:0] h_op,
	output reg [23:0] h_lba,
	output reg [23:0] h_lba_end,
	input             h_busy,
	input             h_done,
	input             h_err,
	input             tray,             // Linux mounted or ejected the image
	input             drive_en,

	// table of contents written by the host
	output reg  [6:0] toc_addr,
	input      [31:0] toc_data,        // {ctrl, M, S, F}, one clock after toc_addr
	input       [7:0] toc_last,
	input      [99:0] toc_mode2,       // data track n is mode 2
	input       [7:0] sq_status,
	input       [7:0] sq_ctrl,
	input       [7:0] sq_track,
	input       [7:0] sq_index,
	input       [7:0] sq_abs_m, sq_abs_s, sq_abs_f,
	input       [7:0] sq_rel_m, sq_rel_s, sq_rel_f,

	// savestate port: the drive model as bytes, no side effects
	input             ss_cs,
	input             ss_wr,
	input       [5:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output            ss_quiet         // no host request on its way
);

localparam [3:0] OP_TUR = 4'd0, OP_TOC = 4'd1, OP_READ = 4'd2, OP_PLAY = 4'd3,
                 OP_PAUSE = 4'd4, OP_RESUME = 4'd5, OP_STOP = 4'd6, OP_SUBQ = 4'd7, OP_SEEK = 4'd8;

localparam [5:0] C_SEEK = 6'h00, C_MODE2 = 6'h01, C_MODE1 = 6'h02, C_RAW = 6'h03, C_PLAY = 6'h04,
                 C_TOC = 6'h05, C_SUBQ = 6'h06, C_DISC = 6'h1F, C_STATE = 6'h20, C_SET = 6'h21,
                 C_STOP = 6'h24, C_PAUSE = 6'h25, C_RESUME = 6'h27, C_UNK = 6'h3F;

localparam [4:0] M_INIT = 5'd0, M_INIT_TOC = 5'd1, M_IDLE = 5'd2, M_DELAY = 5'd3, M_REFRESH = 5'd4,
                 M_DISPATCH = 5'd5, M_SEEK = 5'd6, M_READ_FETCH = 5'd7, M_READ_WAIT = 5'd8,
                 M_READY = 5'd9, M_WAIT_DTS = 5'd10, M_XFER = 5'd11, M_READ_DONE = 5'd12,
                 M_TOC = 5'd13, M_SUBQ = 5'd14, M_HOST = 5'd15, M_EMIT = 5'd16, M_STOP_WAIT = 5'd17,
                 M_NOTIFY = 5'd18, M_PROBE = 5'd19, M_TRACK = 5'd20;

reg  [4:0] state /*verilator public*/, after_host, after_emit, after_refresh;
reg  [7:0] c;                 // command byte
reg  [7:0] p [0:7];
wire [5:0] code = {c[7], c[4:0]};
wire       f_status = c[5];
wire       f_irq    = c[6];

reg        disc, disc_changed;
reg  [1:0] cdda;              // 0 idle, 1 playing, 2 paused, 3 ended
reg        cdda_repeat;
reg [23:0] play_start, play_end;
reg [23:0] cur, last, head;
reg [23:0] timer;             // microseconds
reg [23:0] period;            // time to the next sector
reg [23:0] probe_timer;
reg        lost;              // lost-data timeout armed
reg  [3:0] emit_idx /*verilator public*/;
reg  [6:0] emit_trk;
reg  [3:0] emit_kind;         // which reply list is being produced
reg  [7:0] disc_type;
reg        toc_scan, scan_wait;
reg        h_pending;         // host request issued, completion not yet seen
reg        h_want;            // request waiting for the host to be free
reg        disc_seen;         // a disc has been present since reset
reg        media_pending;     // open the tray when the drive is next idle
reg        tray_seen;         // the tray has opened since reset
reg  [6:0] trk;               // track lookup: candidate track
reg        trk_data;          // its control bits say data
reg        trk_wait;
reg  [1:0] retry;             // re-reads left for the sector in progress

wire [7:0] s2 = !disc ? 8'd9 : cdda == 2'd2 ? 8'd1 : cdda == 2'd1 ? 8'd3 : 8'd0;

// a read is in progress: a new command cancels it and the sector it left armed
wire reading = state == M_READ_FETCH || state == M_READ_WAIT || state == M_READY || state == M_NOTIFY ||
               state == M_WAIT_DTS || state == M_XFER || (state == M_HOST && after_host == M_READ_WAIT);
// the drive is finding its disc and TOC (DRY low): a command waits for it
wire initing = state == M_INIT || state == M_INIT_TOC ||
               (state == M_HOST && (after_host == M_INIT_TOC || (after_host == M_IDLE && h_op == OP_TOC)));
reg  cmd_wait;

function [7:0] bcd2bin(input [7:0] b);
	bcd2bin = {1'b0, b[7:4], 3'd0} + {2'd0, b[7:4], 1'b0} + {4'd0, b[3:0]};
endfunction

function [7:0] bin2bcd(input [7:0] v);
	reg [3:0] t;
	reg [7:0] r;
	begin
		t = v >= 8'd90 ? 4'd9 : v >= 8'd80 ? 4'd8 : v >= 8'd70 ? 4'd7 : v >= 8'd60 ? 4'd6 : v >= 8'd50 ? 4'd5 :
		    v >= 8'd40 ? 4'd4 : v >= 8'd30 ? 4'd3 : v >= 8'd20 ? 4'd2 : v >= 8'd10 ? 4'd1 : 4'd0;
		r = v - {t, 3'd0} - {2'd0, t, 1'b0};   // v - 10t
		bin2bcd = {t, r[3:0]};
	end
endfunction

// (m*60 + s)*75 + f with shifts and adds; the inputs are binary
function [23:0] msf2lba(input [7:0] m, input [7:0] s, input [7:0] f);
	reg [23:0] mm, ss, m24, s24;
	begin
		m24 = {16'd0, m};
		s24 = {16'd0, s};
		mm = (m24 << 12) + (m24 << 8) + (m24 << 7) + (m24 << 4) + (m24 << 2);   // 4500
		ss = (s24 << 6) + (s24 << 3) + (s24 << 1) + s24;                         // 75
		msf2lba = mm + ss + {16'd0, f};
	end
endfunction

// Command parameters as LBAs, a stage per clock; every consumer runs
// after the acceptance delay, long after the parameters land.
reg  [23:0] p_start, p_end;
reg  [23:0] p_lba;       // head position of the start
reg  [23:0] seek_dist;
always @(posedge clk) begin
	p_start   <= msf2lba(bcd2bin(p[0]), bcd2bin(p[1]), bcd2bin(p[2]));
	p_end     <= msf2lba(bcd2bin(p[3]), bcd2bin(p[4]), bcd2bin(p[5]));
	p_lba     <= p_start < 24'd150 ? 24'd0 : p_start - 24'd150;
	seek_dist <= (p_lba > head) ? p_lba - head : head - p_lba;
end
wire [23:0] toc_lba   = msf2lba(toc_data[23:16], toc_data[15:8], toc_data[7:0]);   // entry start, absolute
wire  [6:0] toc_next  = ({1'b0, toc_addr} >= toc_last) ? 7'd100 : toc_addr + 1'd1;  // lead-out after the last track

// Issue a host request and come back to `next` when it completes.
task host(input [3:0] op, input [23:0] lba, input [23:0] lba_end, input [4:0] next);
	begin
		h_want     <= 1;
		h_op       <= op;
		h_lba      <= lba;
		h_lba_end  <= lba_end;
		after_host <= next;
		state      <= M_HOST;
	end
endtask

task push(input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3);
	begin
		st_push <= 1;
		st_data <= {b0, b1, b2, b3};
	end
endtask

// Reply lists longer than the FIFO are produced one group per clock as
// space allows; emit_kind selects the list, emit_idx the group.
localparam [3:0] E_NONE = 4'd0, E_TOC = 4'd1, E_SUBQ = 4'd2, E_DISC = 4'd3, E_STOP = 4'd4,
                 E_PAUSE = 4'd5, E_RESUME = 4'd6, E_WIN95 = 4'd7;

always @(posedge clk) begin
	st_push    <= 0;
	st_clear   <= 0;
	sirq_set   <= 0;
	sirq_clr   <= 0;
	data_ready <= 0;
	xfer_abort <= 0;
	h_req      <= 0;

	if (reset) begin
		state <= M_INIT;
		dry   <= 0;
		disc  <= 0;
		disc_changed <= 0;
		cdda  <= 0;
		cdda_repeat <= 0;
		head  <= 0;
		timer <= 0;
		period <= 0;
		probe_timer <= 0;
		lost  <= 0;
		sector_form <= 0;
		sector_mode2 <= 0;
		play_start <= 0;
		play_end <= 0;
		toc_scan <= 0;
		h_want <= 0;
		disc_type <= 8'h21;
		emit_kind <= E_NONE;
		media_pending <= 0;
		tray_seen <= 0;
		cmd_wait <= 0;
		retry <= 0;
	end
	// a savestate writes the registers piecemeal: the machine stands still
	// meanwhile, and a host request that was in flight is asked for again
	else if (!hold) begin
		if (hold_q && state == M_HOST) h_want <= 1;
		// the service also raises one after a mount, which the probe absorbs
		if (tray) begin media_pending <= 1; tray_seen <= 1; end
		if (ce_us) begin
			if (timer != 0) timer <= timer - 1'd1;
			if (period != 0) period <= period - 1'd1;
			if (probe_timer != 0) probe_timer <= probe_timer - 1'd1;
		end

		// A new command interrupts whatever the drive was doing.
		if (cmd_strobe) begin
			c <= cmd;
			{p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]} <= params;
			if (initing) cmd_wait <= 1;
			else begin
				dry   <= 0;
				timer <= ACCEPT_US[23:0];
				lost  <= 0;
				xfer_abort <= reading;
				state <= M_DELAY;
			end
		end
		else case (state)

		// Power-on: find the disc and read its table of contents.
		M_INIT: begin
			dry <= 0;
			if (drive_en) host(OP_TUR, 24'd0, 24'd0, M_INIT_TOC);
			else begin probe_timer <= 24'd500000; state <= M_PROBE; dry <= 1; end   // no drive: ready, no disc
		end
		M_INIT_TOC: begin
			if (h_err) begin disc <= 0; probe_timer <= 24'd500000; state <= M_PROBE; dry <= 1; end
			else host(OP_TOC, 24'd0, 24'd0, M_IDLE);
		end
		M_PROBE: if (probe_timer == 0) state <= M_INIT;

		M_IDLE: begin
			dry <= 1;
			// a command that arrived during the init runs now
			if (cmd_wait) begin
				cmd_wait <= 0;
				dry   <= 0;
				timer <= ACCEPT_US[23:0];
				lost  <= 0;
				state <= M_DELAY;
			end
			// tray event: the disc is gone until the door closes, then the
			// probe finds the new one and reports the change once
			else if (media_pending) begin
				media_pending <= 0;
				disc  <= 0;
				cdda  <= 0;
				probe_timer <= DOOR_US[23:0];
				state <= M_PROBE;
			end
			// a disc arriving later shows up as "media changed"
			else if (!disc && probe_timer == 0) begin probe_timer <= 24'd500000; state <= M_INIT; end
			// repeat play restarts on its own: watch for the end of the range
			else if (cdda == 2'd1 && cdda_repeat && probe_timer == 0) begin
				probe_timer <= 24'd100000;
				after_refresh <= M_IDLE;
				host(OP_SUBQ, 24'd0, 24'd0, M_REFRESH);
			end
		end

		// Acceptance delay, then a sub-channel refresh when audio is playing.
		M_DELAY: if (timer == 0) begin
			dry <= 1;
			after_refresh <= M_DISPATCH;
			if (cdda == 2'd1) host(OP_SUBQ, 24'd0, 24'd0, M_REFRESH);
			else state <= M_DISPATCH;
		end
		M_REFRESH: begin
			if (sq_status == 8'h13) begin
				if (cdda_repeat) host(OP_PLAY, play_start, play_end, after_refresh);
				else begin cdda <= 2'd3; state <= after_refresh; end
			end
			else state <= after_refresh;
		end

		M_DISPATCH: begin
			state <= M_IDLE;
			case (code)
			C_SEEK: begin
				st_clear <= 1;
				if (f_status) begin
					if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
					else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
					else begin
						push(8'h00, s2, 8'h00, 8'h00);
						if (f_irq) sirq_set <= 1;
					end
				end
				timer <= SEEK_BASE_US[23:0] + seek_dist;
				head  <= p_lba;
				cdda  <= 0;
				host(OP_SEEK, p_lba, 24'd0, M_SEEK);
			end

			C_MODE1, C_MODE2, C_RAW: begin
				cdda <= 0;
				sector_form <= code == C_MODE1 ? 2'd0 : code == C_MODE2 ? 2'd1 : 2'd2;
				if (!disc) push(8'h21, 8'h09, 8'h00, 8'h00);
				else if (p_start < 24'd150 || p_end < p_start) push(8'h21, 8'h01, 8'h00, 8'h00);
				else begin
					// which track holds the start sector decides the answer
					toc_addr <= 7'd1;
					trk      <= 7'd1;
					trk_wait <= 1;
					state    <= M_TRACK;
				end
			end

			C_PLAY: begin
				st_clear   <= 1;
				play_start <= p_start < 24'd150 ? 24'd0 : p_start - 24'd150;
				play_end   <= p_end < 24'd150 ? 24'd0 : p_end - 24'd150;
				cdda_repeat <= (p[6] == 8'd1);
				if (disc) cdda <= 2'd1;
				if (f_status) begin
					if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
					else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
					else push(8'h00, 8'h03, 8'h00, 8'h00);
					if (f_irq) sirq_set <= 1;
				end
				if (disc) host(OP_PLAY, p_start < 24'd150 ? 24'd0 : p_start - 24'd150, p_end < 24'd150 ? 24'd0 : p_end - 24'd150, M_IDLE);
			end

			C_TOC: begin
				st_clear <= 1;
				if (f_status && !disc) push(8'h00, 8'h09, 8'h00, 8'h00);
				else if (f_status && disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
				else begin
					if (f_status) push(8'h00, s2, 8'h00, 8'h00);
					emit_kind <= E_TOC;
					emit_idx  <= 0;
					emit_trk  <= 7'd1;
					toc_addr  <= 7'd100;
					after_emit <= M_IDLE;
					state <= M_EMIT;
					if (f_irq) sirq_set <= 1;
				end
			end

			C_SUBQ: if (f_status) begin
				if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
				else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
				else host(OP_SUBQ, 24'd0, 24'd0, M_SUBQ);
			end

			C_DISC: if (f_status) begin
				if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
				else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
				else begin
					push(8'h00, s2, 8'h00, 8'h00);
					if (p[0] == 8'd3) begin
						emit_kind <= E_DISC; emit_idx <= 0; after_emit <= M_IDLE; state <= M_EMIT;
					end
				end
				if (f_irq) sirq_set <= 1;
			end

			C_STATE, C_SET: if (f_status) begin
				if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
				// one group only: the ROM's boot probe reads a single group per
				// status request and a second one left in the FIFO is taken
				// as the acknowledge of its TOC command
				else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
				else push(8'h00, s2, 8'h00, 8'h00);
				if (cdda == 2'd3) cdda <= 0;
				if (f_irq) sirq_set <= 1;
			end

			C_STOP: begin
				if (cdda == 2'd1) begin timer <= 24'd1000; state <= M_STOP_WAIT; end
				else begin emit_kind <= E_STOP; emit_idx <= 0; after_emit <= M_IDLE; state <= M_EMIT; st_clear <= 1; end
				if (disc && cdda != 0) host(OP_STOP, 24'd0, 24'd0, cdda == 2'd1 ? M_STOP_WAIT : M_EMIT);
			end

			C_PAUSE: begin
				if (cdda == 2'd1) begin cdda <= 2'd2; host(OP_PAUSE, 24'd0, 24'd0, M_IDLE); end
				if (f_status) begin
					if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
					else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
					else begin emit_kind <= E_PAUSE; emit_idx <= 0; after_emit <= (cdda == 2'd1) ? M_HOST : M_IDLE; state <= M_EMIT; end
				end
			end

			C_RESUME: begin
				if (cdda == 2'd2) begin cdda <= 2'd1; host(OP_RESUME, 24'd0, 24'd0, M_IDLE); end
				if (f_status) begin
					if (!disc) push(8'h00, 8'h09, 8'h00, 8'h00);
					else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; end
					else begin emit_kind <= E_RESUME; emit_idx <= 0; after_emit <= (cdda == 2'd2) ? M_HOST : M_IDLE; state <= M_EMIT; end
				end
			end

			C_UNK: begin
				if (p[1] == 8'h5F && p[2] == 8'hFC && p[3] == 8'h5F && p[4] == 8'hFC) begin
					emit_kind <= E_WIN95; emit_idx <= 0; after_emit <= M_IDLE; state <= M_EMIT;
				end
				else push(8'h21, 8'h00, 8'h00, 8'h00);
			end

			default: ;
			endcase
		end

		// Walk the TOC one entry at a time: track 1's own entry first for its
		// control bits, then each following start (lead-out after the last)
		// until one lies past the requested sector.
		M_TRACK: begin
			if (trk_wait) trk_wait <= 0;
			else if (toc_addr == 7'd1) begin
				trk_data <= toc_data[26];
				toc_addr <= toc_next;
				trk_wait <= 1;
			end
			else if (p_start < toc_lba || toc_addr == 7'd100) begin
				sector_mode2 <= toc_mode2[trk];
				if (!trk_data) begin push(8'h21, 8'h05, 8'h00, 8'h00); state <= M_IDLE; end
				else begin
					push(8'h00, s2, 8'h00, 8'h00);
					sirq_set <= 1;
					retry  <= READ_RETRIES;
					cur    <= p_start - 24'd150;
					last   <= p_end - 24'd150;
					dry    <= 0;
					period <= SECTOR_US[23:0] + SEEK_BASE_US[23:0] + seek_dist;
					state  <= M_READ_FETCH;
				end
			end
			else begin
				trk_data <= toc_data[26];
				trk      <= toc_addr;
				toc_addr <= toc_next;
				trk_wait <= 1;
			end
		end

		M_SEEK: if (timer == 0) begin
			if (f_status) begin
				push(8'h04, 8'h00, 8'h00, 8'h00);
				if (f_irq) sirq_set <= 1;
			end
			state <= M_IDLE;
		end

		// One sector: fetch it, wait out the sector period, announce it.
		M_READ_FETCH: begin
			host(OP_READ, cur, 24'd0, M_READ_WAIT);
		end
		M_READ_WAIT: begin
			// a sector the host could not deliver is read again, as the drive
			// would on a marginal one; only a host that keeps failing is an error
			if (h_err && retry != 0) begin
				retry <= retry - 1'd1;
				state <= M_READ_FETCH;
			end
			else if (h_err) begin
				st_clear <= 1;
				push(8'h21, 8'h04, 8'h00, 8'h00);
				dry <= 1;
				if (f_status && f_irq) sirq_set <= 1;
				state <= M_IDLE;
			end
			else if (period == 0) state <= M_READY;
		end
		M_READY: begin
			// hold data ready while the previous DMA-end interrupt is unserviced
			if (dei && irq_line) begin timer <= NOTIFY_US[23:0]; state <= M_NOTIFY; end
			else begin
				push(8'h22, 8'h00, 8'h00, 8'h00);
				if (f_status && f_irq) sirq_set <= 1;
				data_ready <= 1;
				period <= SECTOR_US[23:0];
				timer  <= LOSTDATA_US[23:0];
				lost   <= 1;
				state  <= M_WAIT_DTS;
			end
		end
		M_NOTIFY: if (timer == 0) state <= M_READY;
		M_WAIT_DTS: begin
			if (xfer_start) begin lost <= 0; state <= M_XFER; end
			else if (timer == 0 && lost) begin
				// DMA never armed: abnormal termination
				st_clear <= 1;
				push(8'h21, 8'h0F, 8'h00, 8'h00);
				// the abort reply replaces the unserviced sector-ready one
				if (f_status && f_irq) sirq_set <= 1;
				else sirq_clr <= 1;
				dry <= 1;
				xfer_abort <= 1;
				lost  <= 0;
				state <= M_IDLE;
			end
		end
		M_XFER: if (xfer_done) begin
			head  <= cur;
			cur   <= cur + 1'd1;
			retry <= READ_RETRIES;
			if (cur >= last) begin timer <= NOTIFY_US[23:0]; state <= M_READ_DONE; end
			else state <= M_READ_FETCH;
		end
		M_READ_DONE: if (timer == 0) begin
			dry <= 1;
			st_clear <= 1;
			push(8'h06, 8'h00, 8'h00, 8'h00);
			if (f_status && f_irq) sirq_set <= 1;
			else sirq_clr <= 1;
			state <= M_IDLE;
		end

		M_SUBQ: begin
			emit_kind <= E_SUBQ; emit_idx <= 0; after_emit <= M_IDLE; state <= M_EMIT;
		end

		M_STOP_WAIT: if (timer == 0 && !h_busy) begin
			st_clear <= 1;
			cdda <= 2'd3;
			emit_kind <= E_STOP; emit_idx <= 0; after_emit <= M_IDLE; state <= M_EMIT;
		end

		// Host request: issued once the host is free, then waited for.
		M_HOST: if (h_want) begin
			if (!h_busy && !h_pending && !h_req) begin h_req <= 1; h_want <= 0; end
		end
		else if (!h_pending && !h_req) begin
			if (after_host == M_INIT_TOC) begin
				disc <= !h_err;
				// a disc that arrives through the tray is a change even when it
				// is the first one, which is how the ROM notices a disc inserted
				// at its wait screen; only the disc present at power-on is not
				if (!h_err && !disc) disc_changed <= disc_seen | tray_seen;
				state <= M_INIT_TOC;
			end
			else if (after_host == M_IDLE && h_op == OP_TOC) begin
				// TOC loaded: scan for a data track to answer command 1F
				toc_scan <= 1; scan_wait <= 1; toc_addr <= 7'd1; emit_trk <= 7'd1; disc_type <= 8'h21; state <= M_IDLE;
			end
			else state <= after_host;
		end

		// Reply groups, one every other clock so the FIFO count is current.
		M_EMIT: if (!st_full && !st_push) begin
			emit_idx <= emit_idx + 1'd1;
			case (emit_kind)
			E_TOC: case (emit_idx)
				4'd0: push(8'h16, 8'h00, 8'hA0, 8'h00);
				4'd1: push(8'h17, 8'h01, 8'h00, 8'h00);
				4'd2: push(8'h16, 8'h00, 8'hA1, 8'h00);
				4'd3: push(8'h17, bin2bcd(toc_last), 8'h00, 8'h00);
				4'd4: push(8'h16, 8'h00, 8'hA2, 8'h00);
				4'd5: begin push(8'h17, bin2bcd(toc_data[23:16]), bin2bcd(toc_data[15:8]), bin2bcd(toc_data[7:0])); toc_addr <= 7'd1; end
				4'd6: begin
					push(8'h16, toc_data[26] ? 8'h40 : 8'h00, bin2bcd({1'b0, emit_trk}), 8'h00);
					emit_idx <= 4'd7;
				end
				default: begin
					push(8'h17, bin2bcd(toc_data[23:16]), bin2bcd(toc_data[15:8]), bin2bcd(toc_data[7:0]));
					if ({1'b0, emit_trk} >= toc_last) state <= after_emit;
					else begin emit_trk <= emit_trk + 1'd1; toc_addr <= emit_trk + 1'd1; emit_idx <= 4'd6; end
				end
				endcase
			E_SUBQ: case (emit_idx)
				4'd0: push(8'h00, 8'h00, 8'h00, 8'h00);
				4'd1: push(8'h18, sq_ctrl, sq_track == 8'hAA ? 8'hAA : bin2bcd(sq_track), bin2bcd(sq_index));
				4'd2: push(8'h19, bin2bcd(sq_rel_m), bin2bcd(sq_rel_s), bin2bcd(sq_rel_f));
				4'd3: push(8'h19, 8'h00, bin2bcd(sq_abs_m), bin2bcd(sq_abs_s));
				default: begin push(8'h20, bin2bcd(sq_abs_f), 8'h00, 8'h00); state <= after_emit; end
				endcase
			E_DISC: case (emit_idx)
				4'd0: push(8'h18, disc_type, 8'h00, 8'h00);
				4'd1: push(8'h19, 8'h00, 8'h00, 8'h00);
				4'd2: push(8'h19, 8'h00, 8'h00, 8'h00);
				default: begin push(8'h20, 8'h00, 8'h00, 8'h00); state <= after_emit; end
				endcase
			E_STOP: case (emit_idx)
				4'd0: if (!disc) begin push(8'h00, 8'h09, 8'h00, 8'h00); cdda <= 0; state <= after_emit; end
				      else if (disc_changed) begin push(8'h21, 8'h08, 8'h00, 8'h00); disc_changed <= 0; cdda <= 0; state <= after_emit; end
				      else begin push(8'h00, 8'h00, 8'h00, 8'h00); cdda <= 2'd3; end
				4'd1: push(8'h11, 8'h00, 8'h00, 8'h00);
				default: begin push(8'h00, 8'h0D, 8'h00, 8'h00); state <= after_emit; end
				endcase
			E_PAUSE: case (emit_idx)
				4'd0: push(8'h00, 8'h01, 8'h00, 8'h00);
				default: begin push(8'h12, 8'h00, 8'h00, 8'h00); state <= after_emit; end
				endcase
			E_RESUME: case (emit_idx)
				4'd0: push(8'h00, 8'h00, 8'h00, 8'h00);
				default: begin push(8'h13, 8'h00, 8'h00, 8'h00); state <= after_emit; end
				endcase
			E_WIN95: case (emit_idx)
				4'd0: push(8'h00, 8'h00, 8'h00, 8'h00);
				default: begin push(8'h1F, 8'h5F, 8'hFC, 8'h01); state <= after_emit; end
				endcase
			default: state <= after_emit;
			endcase
		end

		default: state <= M_IDLE;
		endcase

		// Background scan of the TOC for a data track (disc type for 1F).
		// The RAM answers one clock after the address, hence the wait.
		if (toc_scan && state == M_IDLE) begin
			if (scan_wait) scan_wait <= 0;
			else begin
				if (toc_data[26]) disc_type <= 8'h41;
				if ({1'b0, emit_trk} >= toc_last) toc_scan <= 0;
				else begin emit_trk <= emit_trk + 1'd1; toc_addr <= emit_trk + 1'd1; scan_wait <= 1; end
			end
		end
	end
	if (!reset && ss_cs && ss_wr) begin
		if (ss_a >= 6'h14 && ss_a < 6'h32) begin
			case (ss_f)
			4'd0: play_start[8*ss_fb +: 8] <= ss_din;
			4'd1: play_end[8*ss_fb +: 8] <= ss_din;
			4'd2: cur[8*ss_fb +: 8] <= ss_din;
			4'd3: last[8*ss_fb +: 8] <= ss_din;
			4'd4: head[8*ss_fb +: 8] <= ss_din;
			4'd5: timer[8*ss_fb +: 8] <= ss_din;
			4'd6: period[8*ss_fb +: 8] <= ss_din;
			4'd7: probe_timer[8*ss_fb +: 8] <= ss_din;
			4'd8: h_lba[8*ss_fb +: 8] <= ss_din;
			4'd9: h_lba_end[8*ss_fb +: 8] <= ss_din;
			default: ;
			endcase
		end
		else if (ss_a[5:3] == 3'b001) p[ss_a[2:0]] <= ss_din;
		else case (ss_a)
			6'h00: state <= ss_din[4:0];
			6'h01: after_host <= ss_din[4:0];
			6'h02: after_emit <= ss_din[4:0];
			6'h03: after_refresh <= ss_din[4:0];
			6'h04: c <= ss_din;
			6'h05: {dry, disc, disc_changed, cdda, cdda_repeat, lost, cmd_wait} <= ss_din;
			6'h06: {sector_form, sector_mode2, toc_scan, scan_wait, h_want, media_pending, tray_seen} <= ss_din;
			6'h07: {emit_idx, emit_kind} <= ss_din;
			6'h10: emit_trk <= ss_din[6:0];
			6'h11: disc_type <= ss_din;
			6'h12: begin {trk_data, trk_wait} <= ss_din[1:0]; retry <= ss_din[4:3]; end
			6'h13: trk <= ss_din[6:0];
			6'h32: h_op <= ss_din[3:0];
			6'h33: toc_addr <= ss_din[6:0];
			default: ;
		endcase
	end
end

reg hold_q;
always @(posedge clk) hold_q <= hold;
always @(posedge clk) if (reset) disc_seen <= 0; else if (ss_cs && ss_wr && ss_a == 6'h12) disc_seen <= ss_din[2]; else if (disc) disc_seen <= 1;
assign ss_quiet = !h_pending && !h_want;

// 24-bit fields at three bytes each from 14h on: field and byte by table
wire  [5:0] ss_o = ss_a - 6'h14;
wire  [3:0] ss_f  = ss_o >= 6'd27 ? 4'd9 : ss_o >= 6'd24 ? 4'd8 : ss_o >= 6'd21 ? 4'd7 : ss_o >= 6'd18 ? 4'd6 :
                    ss_o >= 6'd15 ? 4'd5 : ss_o >= 6'd12 ? 4'd4 : ss_o >= 6'd9 ? 4'd3 : ss_o >= 6'd6 ? 4'd2 :
                    ss_o >= 6'd3 ? 4'd1 : 4'd0;
wire  [5:0] ss_f3 = {1'b0, ss_f, 1'b0} + {2'd0, ss_f};
wire  [1:0] ss_fb = ss_o[1:0] - ss_f3[1:0];
reg  [23:0] ss_fv;
always @* begin
	case (ss_f)
	4'd0: ss_fv = play_start;  4'd1: ss_fv = play_end;   4'd2: ss_fv = cur;         4'd3: ss_fv = last;
	4'd4: ss_fv = head;        4'd5: ss_fv = timer;      4'd6: ss_fv = period;      4'd7: ss_fv = probe_timer;
	4'd8: ss_fv = h_lba;       4'd9: ss_fv = h_lba_end;  default: ss_fv = 24'd0;
	endcase
end
always @* begin
	if (ss_a >= 6'h14 && ss_a < 6'h32) ss_dout = ss_fb == 2'd0 ? ss_fv[7:0] : ss_fb == 2'd1 ? ss_fv[15:8] : ss_fv[23:16];
	else if (ss_a[5:3] == 3'b001) ss_dout = p[ss_a[2:0]];   // 08-0F
	else case (ss_a)
		6'h00: ss_dout = {3'd0, state};
		6'h01: ss_dout = {3'd0, after_host};
		6'h02: ss_dout = {3'd0, after_emit};
		6'h03: ss_dout = {3'd0, after_refresh};
		6'h04: ss_dout = c;
		6'h05: ss_dout = {dry, disc, disc_changed, cdda, cdda_repeat, lost, cmd_wait};
		6'h06: ss_dout = {sector_form, sector_mode2, toc_scan, scan_wait, h_want, media_pending, tray_seen};
		6'h07: ss_dout = {emit_idx, emit_kind};
		6'h10: ss_dout = {1'b0, emit_trk};
		6'h11: ss_dout = disc_type;
		6'h12: ss_dout = {3'd0, retry, disc_seen, trk_data, trk_wait};
		6'h13: ss_dout = {1'b0, trk};
		6'h32: ss_dout = {4'd0, h_op};
		6'h33: ss_dout = {1'b0, toc_addr};
		default: ss_dout = 8'h00;
	endcase
end

always @(posedge clk) begin
	if (reset) h_pending <= 0;
	else if (h_req) h_pending <= 1;
	else if (h_done) h_pending <= 0;
end

endmodule

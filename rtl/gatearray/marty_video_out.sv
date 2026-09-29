// Copyright (c) 2026 Jamie Blanks
//
// Marty scan converter. Like the real part it never converts a raster:
// it runs its own fixed output timing and pulls rendered lines from
// VRAM through the CRTC's line renderer, one line at a time, at its own
// pace. The CRTC's raster keeps the timing software sees.
//
//   ┌──────────┐ pull_line/skip ┌───────────┐ 8 line banks ┌──────────┐
//   │ CRTC     │<───────────────┤ puller    ├─────────────>│ reader   │
//   │ renderer ├───── pixels ──>│           │              │ 3 taps   ├─> raster
//   └──────────┘                └───────────┘              └──────────┘
//
// The picture class comes from the CRTC geometry, as the Marty's
// converter is told the mode by software:
//   200-class: a 15 kHz picture, or one the CRTC doubles (ZV = 1):
//              every line as it is, in both fields
//   400-class: each TV line the average of two adjacent lines, the pair
//              phase moving by one per field
//   480-class: 6:5 reduction, three taps, 200 lines a field
// An undoubled window on a 31 kHz 525-line raster is part of that
// raster's 480-line picture and on a 24 kHz raster part of its 400-line
// one, wherever it sits and however short: a 320x240 window in the
// 640x480 raster keeps the shape it has on the monitor. So is a doubled
// picture on a 525-line raster run from CLKSEL 3 with a short HST.
// Modes: 0 original (480i NTSC, the above), 1 240p (everything halved to
// one progressive field), 2 480p (everything drawn at 525 lines: 480 and
// 400 line pictures as they are, 200-line ones doubled), 3 native (the
// CRTC raster untouched).
//
// Rendered lines are addressed one entry per dot from the earliest
// display start for 31/24 kHz sources, and one entry per output sample
// for 15 kHz sources, which are TV timed already. Analog video needs every
// sample to last the same whole number of clocks: four on the TV rasters,
// two for a 15 kHz picture on 480p.
//
// The output timing never follows the source. When the source frame
// lasts one output frame, the converter instead pulls the CRTC's raster
// into phase with its own, so games that flip at VSYNC do not tear.
//
// The blanks cover everything but the drawn picture, as the scaler
// stretches only the unblanked box. The box moves only once the drawn
// span has held still for a while, so a game that opens its display
// window a few dots a frame plays inside the old box against black.

module marty_video_out
#(
	parameter CLK_HZ = 57272727       // clk; the device's is four 14.318 MHz periods
)
(
	input             clk,
	input             reset,
	input             ce_14m,         // NTSC sample enable
	input             ce_25m,         // VGA sample enable
	input             ce_28m,         // render enable and doubled NTSC
	input       [1:0] mode,           // 0 original, 1 240p, 2 480p, 3 native
	input             show_blank,     // blank only what a set would not show; the rest draws black

	// CRTC render side
	output            pull,
	output reg        pull_frame,
	output reg        pull_field,
	output reg        pull_line,
	output reg        pull_skip,
	output            sync_out,       // frame start for a source running at the output rate
	output            follow_fa,      // the source runs at its own rate: the render follows its page flips
	input             r_ready,
	input             r_ack,
	input             r_ce,
	input      [10:0] r_x,
	input       [7:0] r_i,
	input       [7:0] g_i,
	input       [7:0] b_i,
	input             de_i,
	// CRTC raster: the native stream and the timing for genlock
	input             dot_ce,
	input             hs_i,
	input             vs_i,
	input             field_i,
	input             src_vs,
	input      [10:0] t_hcnt,         // CRTC timing raster: dot
	input      [10:0] t_vcnt,         // and half line
	// CRTC geometry
	input      [10:0] geo_hst,
	input       [1:0] geo_clksel,
	input      [10:0] geo_vds,
	input      [10:0] geo_vde,
	input      [10:0] geo_hds,
	input      [10:0] geo_hde,
	input      [10:0] geo_vst,
	input             geo_zv1,

	output            ce_pix,
	output reg  [7:0] r_o,
	output reg  [7:0] g_o,
	output reg  [7:0] b_o,
	output reg        hs_o,
	output reg        vs_o,
	output reg        hb_o,
	output reg        vb_o,
	output reg        field_o
);

// NTSC line at 14.318 MHz (measured on the Marty) and VGA line at 25.175 MHz;
// 480p is always the VGA line, a 15 kHz picture on it in 28.636 MHz samples
// (W). A converted picture sits centred in the active window (754 samples
// NTSC, 640 VGA, 728 W).
localparam N_TOTAL = 910, N_FRONT = 21, N_SYNC = 67, N_BLANK = 156, N_ACTIVE = 754;
localparam V_TOTAL = 800, V_FRONT = 16, V_SYNC = 96, V_BLANK = 160, V_ACTIVE = 640;
localparam W_TOTAL = 910, W_FRONT = 18, W_SYNC = 109, W_BLANK = 182, W_ACTIVE = 728;
localparam VS_HALF = 6;       // vertical sync in half lines
localparam TOP_BLANK = 17;    // lines blanked after the sync start
localparam [9:0] PULL_LEAD = 10'd6;   // output lines the render side runs ahead of the picture

wire        native = (mode == 2'd3);
wire        p480   = (mode == 2'd2);
wire        p240   = (mode == 2'd1);
assign      pull   = !native;

// ---- source geometry, latched at every output frame start ----
// With every layer off the CRTC reports no window. The picture then
// keeps the box of the last frame that had one and shows black inside
// it: an output frame with no active area at all makes the scaler cycle
// its stale buffers, which flickers.
wire        src_off = !(geo_vde > geo_vds);
reg  [10:0] h_hst, h_vds, h_vde, h_hds, h_hde, h_vst;
reg   [1:0] h_clksel;
reg         h_zv1;
always @(posedge clk) begin
	if (reset) begin
		h_hst <= 0; h_vds <= 0; h_vde <= 0; h_hds <= 0; h_hde <= 0; h_vst <= 0; h_clksel <= 0; h_zv1 <= 0;
	end
	else if (!src_off) begin
		h_hst <= geo_hst; h_vds <= geo_vds; h_vde <= geo_vde; h_hds <= geo_hds; h_hde <= geo_hde; h_vst <= geo_vst;
		h_clksel <= geo_clksel; h_zv1 <= geo_zv1;
	end
end
wire [10:0] g_hst    = src_off ? h_hst : geo_hst;
wire [10:0] g_vds    = src_off ? h_vds : geo_vds;
wire [10:0] g_vde    = src_off ? h_vde : geo_vde;
wire [10:0] g_hds    = src_off ? h_hds : geo_hds;
wire [10:0] g_hde    = src_off ? h_hde : geo_hde;
wire [10:0] g_vst    = src_off ? h_vst : geo_vst;
wire  [1:0] g_clksel = src_off ? h_clksel : geo_clksel;
wire        g_zv1    = src_off ? h_zv1 : geo_zv1;
wire        fast_now  = (g_hst < 11'd1024);
wire        src_525   = (g_vst >= 11'd1000);   // a 31 kHz 525-line raster: vertical positions are already VGA lines
wire [10:0] lines_now = (g_vde > g_vds) ? ((g_vde - g_vds) >> 1) : 11'd0;
wire [10:0] vlines    = g_zv1 ? (lines_now >> 1) : lines_now;   // distinct picture lines
wire        c3_525    = fast_now && src_525 && (g_clksel == 2'd3);
// A doubled picture: 200 lines at 38, 240 from 20; a taller one starts
// at the top blank and loses its last lines.
wire  [9:0] p0_dbl    = (vlines <= 11'd222) ? 10'd38 : (vlines <= 11'd243) ? 10'd260 - vlines[9:0] : TOP_BLANK[9:0];
// An undoubled window on a 31/24 kHz raster is part of that raster's
// 480- or 400-line picture and takes its reduction: d_off is where it
// starts below the raster's standard first line (35 on the 525-line
// raster, 32 on the 400-line one). The 400 class pairs lines (r, r+1)
// for row r, so the window's first row is the first of its field at or
// past d_off; the 480 class walks its 6:5 taps up to d_off after the
// frame start (walk, a few hundred clocks in the top blank).
wire  [9:0] first_ln  = src_525 ? 10'd35 : 10'd32;
wire  [9:0] d_off     = (fast_now && g_vds[10:1] > first_ln) ? g_vds[10:1] - first_ln : 10'd0;
wire  [9:0] j_pair_f  = (d_off + 10'd1 - {9'd0, half_start}) >> 1;   // interlaced: rows r = 2j + field
wire  [9:0] j_pair    = (d_off + 10'd1) >> 1;                        // progressive: rows r = 2j
wire  [9:0] a_pair_f  = {j_pair_f[8:0], half_start} - d_off;         // 0 or 1: the pair's first line inside the window
wire  [9:0] a_pair    = {j_pair[8:0], 1'b0} - d_off;
// where a 31/24 kHz raster line lands on the 480p frame: a 525-line
// source as it is, a 24 kHz raster 43 lines down (its line 32 at 75)
wire  [9:0] p0_fast   = src_525 ? g_vds[10:1] : g_vds[10:1] + 10'd43;
// 240p and 480p show a 525-line raster at the pace it is scanned. Each
// line is rendered only once the timing raster reaches it, as a monitor
// sees it, so a palette or scroll write between lines lands on the same
// line; a line the next output line needs is rendered even if the raster
// is late. In 480p the raster is pulled into phase three lines ahead of
// the output to leave the render its time and drifts back about a line
// before it is pulled again; in 240p it already runs ahead.
wire        track_now = (p480 || p240) && fast_now && src_525;
reg         track;
// Original reads a 480-class picture on such a raster at the Marty's
// pace: each output line's taps are fetched during the line before, and
// the CRTC frame starts a line before the output frame. Palette and
// scroll writes then land where a Marty shows them.
wire        list_now  = (mode == 2'd0) && fast_now && src_525 && (!g_zv1 || vlines > 11'd280 || c3_525);
reg         list;
reg         fast;             // one entry per dot
reg   [9:0] pic_first;        // first raster line of the picture
reg   [9:0] pic_lines;
reg   [1:0] clksel;
reg         w28;              // 480p lines in 28.636 MHz samples
reg  [10:0] hds;
reg  [10:0] pic_x0;           // first sample of a converted picture

// ---- output raster ----
wire        vga      = p480;
// On the device clk is four 14.318 MHz periods: the TV rasters sample on
// every fourth clock and a 15 kHz picture on 480p on every second, so a
// sample lasts the same clocks on every line. A bench on another clock
// takes the fractional enables.
localparam EXACT = (CLK_HZ == 57272727);
reg   [1:0] div4;
wire        ce_14x   = EXACT ? (div4 == 2'd0) : ce_14m;
wire        ce_28x   = EXACT ? div4[0] : ce_28m;
wire        w28_now  = !fast_now;
wire        ce_out   = !vga ? ce_14x : w28 ? ce_28x : ce_25m;
wire [10:0] h_total  = !vga ? N_TOTAL[10:0] : w28 ? W_TOTAL[10:0] : V_TOTAL[10:0];
wire [10:0] h_front  = !vga ? N_FRONT[10:0] : w28 ? W_FRONT[10:0] : V_FRONT[10:0];
wire [10:0] h_sync   = !vga ? N_SYNC[10:0]  : w28 ? W_SYNC[10:0]  : V_SYNC[10:0];
wire [10:0] h_blank  = !vga ? N_BLANK[10:0] : w28 ? W_BLANK[10:0] : V_BLANK[10:0];
// centre the source window in the active area; a wider one starts at its edge.
// A 15 kHz window on 480p is half its dots wide, 7/12 of them from CLKSEL 1.
// Spread over three clocks: the geometry only moves on CRTC writes and
// pic_x0 takes the result once a frame.
reg  [10:0] win_w, win_s, vga_off, ntsc_off;
wire [10:0] v_blank  = w28_now ? W_BLANK[10:0]  : V_BLANK[10:0];
wire [10:0] v_active = w28_now ? W_ACTIVE[10:0] : V_ACTIVE[10:0];
always @(posedge clk) begin
	win_w    <= (g_hde > g_hds) ? g_hde - g_hds : 11'd0;
	win_s    <= fast_now ? win_w : (g_clksel == 2'd1) ? (win_w >> 1) + (win_w >> 4) + (win_w >> 6) : (win_w >> 1);
	vga_off  <= (win_s < v_active) ? ((v_active - win_s) >> 1) : 11'd0;
	ntsc_off <= (win_w < N_ACTIVE) ? ((N_ACTIVE[10:0] - win_w) >> 1) : 11'd0;
end
wire [10:0] h_half  = h_total >> 1;
wire [10:0] frame_hl = p480 ? 11'd1050 : p240 ? 11'd524 : 11'd525;   // half lines per frame

reg  [10:0] ocnt;
reg  [10:0] ohl;              // half lines since the frame start
reg   [2:0] vsh;              // half lines into the vertical sync
reg         field;
reg  [10:0] src_hl;           // half lines since the last source vertical sync
reg         locked;
reg         src_vs_q;

wire        line_start  = ce_out && (ocnt == 11'd0);
wire        half_start  = ce_out && (ocnt == h_half);
wire        boundary    = line_start || (half_start && !p480 && !p240);
wire        src_vs_rise = src_vs && !src_vs_q;
wire        v_restart   = boundary && (ohl >= frame_hl - 1'd1);
wire        v_lead      = boundary && (ohl == frame_hl - 11'd7);   // three lines before the restart
wire        v_lead1     = boundary && (ohl == frame_hl - 11'd3);   // one line before it
assign      sync_out    = ce_out && locked && ((track && vga) ? v_lead : list ? v_lead1 : v_restart);
assign      follow_fa   = pull && !locked;
wire [10:0] ocnt_n      = (ocnt == h_total - 1'd1) ? 11'd0 : ocnt + 1'd1;
wire [10:0] ohl_n       = v_restart ? 11'd0 : (ce_out && (ocnt == 11'd0 || ocnt == h_half)) ? ohl + 1'd1 : ohl;
wire        field_n     = v_restart ? half_start : field;
wire [10:0] ohl_fld     = ohl_n - {10'd0, field_n};   // an odd field opens with a half line
wire  [9:0] oline_n     = ohl_fld[10:1];
wire  [2:0] vsh_n       = v_restart ? 3'd0 : ((line_start || half_start) && vsh != 3'd7) ? vsh + 1'd1 : vsh;

always @(posedge clk) begin
	if (reset) begin
		ocnt <= 0; ohl <= 0; vsh <= 3'd7; field <= 0; src_hl <= 0; locked <= 0; src_vs_q <= 0;
		fast <= 0; pic_first <= 0; pic_lines <= 0; clksel <= 0; w28 <= 0; hds <= 0; pic_x0 <= 0; track <= 0; list <= 0;
		div4 <= 0;
	end
	else begin
		src_vs_q <= src_vs;
		div4 <= div4 + 1'd1;
		// a source whose frame lasts about one output frame gets pulled into phase
		if (src_vs_rise) begin
			locked  <= (src_hl + 11'd6 >= frame_hl) && (src_hl <= frame_hl + 11'd6);
			src_hl  <= 0;
		end
		else if (ce_out && (ocnt == 11'd0 || ocnt == h_half) && src_hl != 11'h7FF) src_hl <= src_hl + 1'd1;

		if (ce_out) begin
			ocnt <= ocnt_n;
			vsh <= vsh_n;
			ohl <= ohl_n;
			if (v_restart) begin
				field <= half_start;
				fast <= fast_now;
				track <= track_now;
				list <= list_now;
				pic_first <= g_vds[10:1];
				pic_lines <= lines_now[9:0];
				clksel <= g_clksel;
				w28 <= w28_now;
				hds <= g_hds;
				pic_x0 <= p480 ? v_blank + vga_off : N_BLANK[10:0] + ntsc_off;
			end
		end
	end
end

// ---- reader plan: which source lines make output line j of the picture ----
// a is the first tap, counted from the picture's first line. The
// 480-class walks a = r + r/5 for r = 2j + field, weights by r mod 5.
reg   [9:0] p0;               // output line of the first picture line
reg   [9:0] a;                // first tap of the current output line
reg   [2:0] rem;              // r mod 5
reg         dbl;              // 480p 200-class: every line twice
reg         skip2;            // doubled source: every other raster line, one tap
reg   [4:0] w0, w1, w2;
reg   [1:0] ntap;             // taps in use: 1, 2 or 3
reg         in_pic;           // the current output line is a picture line
reg         walk;             // stepping the 6:5 taps down to the window

wire        pic_row = (oline_n >= p0) && (pic_lines != 0);
wire  [9:0] a_step  = (ntap == 2'd2 || skip2) ? 10'd2 : 10'd1;
wire  [2:0] rem_n   = (rem >= 3'd3) ? rem - 3'd3 : rem + 3'd2;
wire  [9:0] a_480n  = (rem >= 3'd3) ? a + 10'd3 : a + 10'd2;

always @(posedge clk) begin
	if (reset) begin
		p0 <= 0; a <= 0; rem <= 0; dbl <= 0; skip2 <= 0; w0 <= 0; w1 <= 0; w2 <= 0; ntap <= 2'd1; in_pic <= 0; walk <= 0;
	end
	else if (ce_out && v_restart) begin
		dbl <= 0;
		skip2 <= 0;
		rem <= 0;
		in_pic <= 0;
		walk <= 0;
		case ({p480, p240})
			2'b00: begin   // original: NTSC interlace
				if (g_zv1 && vlines <= 11'd280 && !c3_525) begin p0 <= p0_dbl; a <= 0; ntap <= 2'd1; w0 <= 5'd16; w1 <= 0; w2 <= 0; skip2 <= 1; end
				else if (!fast_now && vlines <= 11'd280) begin p0 <= g_vds[10:1]; a <= 0; ntap <= 2'd1; w0 <= 5'd16; w1 <= 0; w2 <= 0; end   // 15 kHz: its own line
				else if (fast_now ? !src_525 : vlines <= 11'd440) begin p0 <= 10'd38 + j_pair_f; a <= a_pair_f; ntap <= 2'd2; w0 <= 5'd8; w1 <= 5'd8; w2 <= 0; end
				else begin p0 <= 10'd38; a <= {9'd0, half_start}; rem <= {2'd0, half_start}; ntap <= 2'd3; walk <= d_off != 10'd0; end
			end
			2'b01: begin   // 240p
				if (g_zv1 && vlines <= 11'd280) begin p0 <= p0_dbl; a <= 0; ntap <= 2'd1; w0 <= 5'd16; w1 <= 0; w2 <= 0; skip2 <= 1; end
				else if (!fast_now && vlines <= 11'd280) begin p0 <= g_vds[10:1]; a <= 0; ntap <= 2'd1; w0 <= 5'd16; w1 <= 0; w2 <= 0; end
				else if (fast_now ? !src_525 : vlines <= 11'd440) begin p0 <= 10'd38 + j_pair; a <= a_pair; ntap <= 2'd2; w0 <= 5'd8; w1 <= 5'd8; w2 <= 0; end
				else begin p0 <= 10'd20 + j_pair; a <= a_pair; ntap <= 2'd2; w0 <= 5'd8; w1 <= 5'd8; w2 <= 0; end
			end
			default: begin // 480p
				ntap <= 2'd1; w0 <= 5'd16; w1 <= 0; w2 <= 0; a <= 0;
				if (lines_now <= 11'd280) begin p0 <= fast_now ? p0_fast : {g_vds[9:1], 1'b0}; dbl <= !fast_now; end   // only a 15 kHz picture is doubled
				else if (lines_now <= 11'd440) p0 <= p0_fast;
				else p0 <= g_vds[10:1];
			end
		endcase
	end
	else if (walk) begin
		// one row a clock until the taps reach the window, then count from it
		if (a < d_off) begin a <= a_480n; rem <= rem_n; p0 <= p0 + 1'd1; end
		else begin a <= a - d_off; walk <= 0; end
	end
	else if (line_start) begin
		// step past the line just shown
		if (in_pic) begin
			if (dbl) begin
				if (!ohl_n[1]) a <= a + 1'd1;   // entering an even line: the pair before is done
			end
			else if (ntap == 2'd3) begin
				a <= a_480n;
				rem <= rem_n;
			end
			else a <= a + a_step;
		end
		in_pic <= pic_row;
	end
	if (!native && ntap == 2'd3) begin
		case (rem)
			3'd0: begin w0 <= 5'd8; w1 <= 5'd6; w2 <= 5'd2; end
			3'd1: begin w0 <= 5'd6; w1 <= 5'd7; w2 <= 5'd3; end
			3'd2: begin w0 <= 5'd5; w1 <= 5'd6; w2 <= 5'd5; end
			3'd3: begin w0 <= 5'd3; w1 <= 5'd7; w2 <= 5'd6; end
			default: begin w0 <= 5'd2; w1 <= 5'd6; w2 <= 5'd8; end
		endcase
	end
end

// ---- puller: skip to the picture, then render lines in order ----
// Bank s mod 8 holds source line s; a bank is rewritten only once the
// reader has moved past it.
localparam S_IDLE = 2'd0, S_SKIP = 2'd1, S_LINE = 2'd2, S_WAIT = 2'd3;
reg   [1:0] pstate;
reg   [9:0] skips, s_next;
reg   [9:0] s_rendered;       // last source line in the banks
reg         s_valid;
reg   [9:0] bx0 [0:7];        // first and last entry written in each bank
reg   [9:0] bx1 [0:7];
reg   [9:0] wx0, wx1;
reg         w_any;
reg  [12:0] lb_wa;
reg  [23:0] lb_wd;
reg         lb_we;
reg         frame_go;
reg   [2:0] tail;             // render clocks since a paced line ended
localparam [2:0] TAIL = 3'd5;   // the CRTC's pixel pipe, with a clock to spare
reg   [9:0] fx0, fx1;         // drawn span of the frame being rendered

wire  [9:0] a_last  = a + {8'd0, ntap - 1'd1};
wire  [9:0] last_line = pic_lines - 1'd1;
wire  [9:0] a_lastc = (a_last > last_line) ? last_line : a_last;   // taps past the picture reuse its last line
wire        drawn   = in_pic && (a < pic_lines);
wire        line_ok = drawn && s_valid && (a_lastc <= s_rendered);
// A tracked line waits until the timing raster reaches its display start;
// a raster already back above the picture has wrapped, so nothing waits.
wire  [9:0] t_line    = t_vcnt[10:1];
wire  [9:0] s_raster  = pic_first + s_next;
wire        raster_ok = !(track && locked) || (t_line > s_raster) || (t_line == s_raster && t_hcnt >= hds) ||
                        (t_line < pic_first && s_next != 10'd0);
wire  [9:0] need_last = (ntap == 2'd3) ? a_480n + 10'd2 : a + a_step + {8'd0, ntap} - 10'd1;   // last line the next output line reads
wire        needed    = (s_next <= need_last);
wire        pace_ok   = list ? needed : (raster_ok || needed);
wire        paced     = (track && locked) || list;   // lines wait on the raster or the reader

// Where a rendered dot goes in its bank. 31/24 kHz lines: one entry per
// dot from the earliest display start. 15 kHz lines: one entry per output
// sample, on the TV rasters from the line start so a TV-timed picture
// keeps its place, on 480p from the display start so the doubled line
// fills its half-time slot. A sample holds two 28.636 MHz dots or 12/7
// of 24.545 MHz ones (597/1024 of a sample a dot), so CLKSEL 1 pixels
// come out two or three samples wide.
wire [10:0] x_dot = r_x - hds;
wire        rel   = fast || vga;
wire [10:0] x_src = vga ? x_dot : r_x;
wire [20:0] x_mul = {10'd0, x_src} * 21'd597;
wire  [9:0] x_15k = (clksel == 2'd1) ? x_mul[19:10] : x_src[10:1];
wire  [9:0] wx    = fast ? x_dot[9:0] : x_15k;
wire        w_ok  = rel ? (de_i && r_x >= hds && (!fast || x_dot < 11'd1024)) : 1'b1;

integer k;
always @(posedge clk) begin
	if (reset) begin
		pstate <= S_IDLE; skips <= 0; s_next <= 0; s_rendered <= 0; s_valid <= 0;
		pull_frame <= 0; pull_field <= 0; pull_line <= 0; pull_skip <= 0;
		wx0 <= 0; wx1 <= 0; w_any <= 0; lb_we <= 0; frame_go <= 0; fx0 <= 10'h3FF; fx1 <= 0; tail <= 0;
		for (k = 0; k < 8; k = k + 1) begin bx0[k] <= 10'h3FF; bx1[k] <= 0; end
	end
	else begin
		pull_frame <= 0;
		lb_we <= 0;
		if (ce_out && v_restart) frame_go <= 1;

		// The render side starts a few lines before the picture, not at the
		// frame start: the CRTC loads its line address at the top of the
		// window, so a game that writes FA in the vertical blank has to be
		// rendered after those writes.
		if (frame_go && pull && (oline_n + PULL_LEAD >= p0)) begin
			// a new output frame: restart the render side at the top of the field
			frame_go <= 0;
			pull_frame <= 1;
			pull_field <= field_i;
			pull_line <= 0;
			pull_skip <= 0;
			skips <= pic_first;
			s_next <= 0;
			s_valid <= 0;
			fx0 <= 10'h3FF; fx1 <= 0;
			pstate <= S_SKIP;
		end
		else case (pstate)
			S_IDLE: ;
			S_SKIP: begin
				if (r_ack && pull_skip) begin pull_skip <= 0; skips <= skips - 1'd1; end
				else if (skips == 0) begin pull_skip <= 0; pstate <= S_LINE; end
				else if (r_ready && !pull_skip) pull_skip <= 1;
			end
			S_LINE: begin
`ifdef VOUT_DEBUG
				if (r_ready && !pull_line && !(s_next < a + 10'd8) && s_next < pic_lines) $display("%0t waiting on reader: s_next %0d a %0d", $time, s_next, a);
`endif
				if (s_next >= pic_lines) pstate <= S_IDLE;
				else if (r_ack && pull_line) begin
					pull_line <= 0;
					wx0 <= 10'h3FF; wx1 <= 0; w_any <= 0;
					tail <= 0;
					pstate <= S_WAIT;
				end
				else if (r_ready && !pull_line && (s_next < a + 10'd8) && pace_ok) pull_line <= 1;
			end
			S_WAIT: begin
				if (r_ce && w_ok) begin
					lb_wa <= {s_next[2:0], wx};
					lb_wd <= de_i ? {r_i, g_i, b_i} : 24'd0;
					lb_we <= 1;
					// the lit span is the display window; a slow line also writes black around it
					if (de_i) begin
						if (!w_any || wx < wx0) wx0 <= wx;
						if (!w_any || wx > wx1) wx1 <= wx;
						w_any <= 1;
					end
				end
				// a paced line waits out the CRTC's pixel pipe: its last pixels
				// would otherwise land in the next line's bank or, while the next
				// line waits, nowhere
				if (r_ready && paced && tail != TAIL) begin
					if (r_ce) tail <= tail + 1'd1;
				end
				else if (r_ready) begin
`ifdef VOUT_DEBUG
					$display("%0t line %0d done a %0d s_next<a+8 %0d", $time, s_next, a, s_next < a + 10'd8);
`endif
					bx0[s_next[2:0]] <= w_any ? wx0 : 10'h3FF;
					bx1[s_next[2:0]] <= w_any ? wx1 : 10'd0;
					if (w_any && wx0 < fx0) fx0 <= wx0;
					if (w_any && wx1 > fx1) fx1 <= wx1;
					s_rendered <= s_next;
					s_valid <= 1;
					s_next <= s_next + 1'd1;
					pstate <= S_LINE;
				end
			end
		endcase
		if (!pull) pstate <= S_IDLE;
	end
end

// The box the blanks follow: the drawn span (samples across, output
// lines down) once it has repeated for BOX_HOLD frames. A changing span
// keeps the old box, so a curtain wipe that widens the display window
// every frame is not passed to the scaler as a new size each time; a
// row count one line off still counts as the same span, as the two
// fields of an interlaced frame differ by one.
localparam [4:0] BOX_HOLD = 5'd8;
reg   [9:0] fy0, fy1;         // drawn rows of the frame being shown
reg   [9:0] px0, px1, py0, py1;
reg   [9:0] cx0, cx1, cy0, cy1;   // the span waiting to become the box
reg   [4:0] box_age;
wire        span_same = (fx0 == cx0) && (fx1 == cx1)
                     && (fy0 + 1'd1 >= cy0) && (cy0 + 1'd1 >= fy0)
                     && (fy1 + 1'd1 >= cy1) && (cy1 + 1'd1 >= fy1);
always @(posedge clk) begin
	if (reset) begin
		px0 <= 10'h3FF; px1 <= 0; py0 <= 10'h3FF; py1 <= 0;
		cx0 <= 10'h3FF; cx1 <= 0; cy0 <= 10'h3FF; cy1 <= 0;
		fy0 <= 10'h3FF; fy1 <= 0; box_age <= 0;
	end
	else if (ce_out) begin
		if (drawn && y_ok_n && !line_start) begin   // at the line boundary drawn is still the line just ended
			if (oline_n < fy0) fy0 <= oline_n;
			if (oline_n > fy1) fy1 <= oline_n;
		end
		if (v_restart) begin
			fy0 <= 10'h3FF; fy1 <= 0;
			if (fx0 <= fx1 && fy0 <= fy1) begin   // a frame that drew nothing keeps the box
				if (!span_same) begin
					cx0 <= fx0; cx1 <= fx1; cy0 <= fy0; cy1 <= fy1;
					box_age <= 0;
					if (px0 > px1) begin px0 <= fx0; px1 <= fx1; py0 <= fy0; py1 <= fy1; end   // no box yet: take this one
				end
				else if (box_age != BOX_HOLD) box_age <= box_age + 1'd1;
				else begin px0 <= cx0; px1 <= cx1; py0 <= cy0; py1 <= cy1; end
			end
		end
	end
end

// ---- line banks ----
reg  [12:0] lb_ra;
wire [23:0] lb_q;
cache_ram_dp #(.ADDR_WIDTH(13), .DATA_WIDTH(24)) lb
(
	.clk_i(clk),
	.addr_a_i(lb_wa), .wren_a_i(lb_we), .wdata_a_i(lb_wd), .q_a_o(),
	.addr_b_i(lb_ra), .wren_b_i(1'b0), .wdata_b_i(24'd0), .q_b_o(lb_q)
);

// ---- reader: up to three reads per output sample, weighted sum ----
// Reads go out last tap first, one per clock after the sample tick, and
// a value is on lb_q two clocks after its address. The weights ride a
// two-stage pipe beside the reads; the first values are summed into acc
// and the last one is added as the sample is registered, so two clocks
// between ticks serve one tap and four serve three.
wire [10:0] x_next = rel ? (ocnt_n - pic_x0) : ((ocnt_n >= h_front) ? ocnt_n - h_front : ocnt_n + (h_total - h_front));
wire        x_ok_n = rel ? ((ocnt_n >= pic_x0) && (x_next < 11'd1024)) : 1'b1;
wire        y_ok_n = (oline_n >= TOP_BLANK) && (oline_n < frame_hl[10:1]);   // the half line closing a field stays blank
// the lines a set shows: NTSC 17..259 of the field, two lines blanked
// ahead of the sync; 480p is that frame doubled, 34..517
wire  [9:0] vis_y0 = vga ? 10'd34  : TOP_BLANK[9:0];
wire  [9:0] vis_y1 = vga ? 10'd518 : frame_hl[10:1] - 10'd2;
wire        vis_row = (oline_n >= vis_y0) && (oline_n < vis_y1);
wire  [9:0] tap1_raw = a + 10'd1, tap2_raw = a + 10'd2;
wire  [9:0] tap1 = (tap1_raw > last_line) ? last_line : tap1_raw;
wire  [9:0] tap2 = (tap2_raw > last_line) ? last_line : tap2_raw;
reg   [1:0] rphase;
reg   [9:0] rx;
reg         rx_ok;
reg  [11:0] acc_r, acc_g, acc_b;
reg   [4:0] w_p1, w_p2;       // weight of the read issued one and two clocks ago
reg         ok_p1, ok_p2;     // that read lies inside its line
reg         show;             // this sample is inside the picture

wire  [9:0] first_line = (ntap == 2'd3) ? tap2 : (ntap == 2'd2) ? tap1 : a;
wire  [4:0] first_w    = (ntap == 2'd3) ? w2 : (ntap == 2'd2) ? w1 : w0;
wire  [9:0] r1_line    = (ntap == 2'd3) ? tap1 : a;
wire  [4:0] r1_w       = (ntap == 2'd3) ? w1 : w0;
wire  [9:0] r2_line    = a;
// one span check serves whichever read goes out this clock
wire  [9:0] chk_line   = ce_out ? first_line : (rphase == 2'd1) ? r1_line : r2_line;
wire  [9:0] chk_x      = ce_out ? x_next[9:0] : rx;
wire        chk_ok     = (ce_out ? x_ok_n : rx_ok) && (chk_x >= bx0[chk_line[2:0]]) && (chk_x <= bx1[chk_line[2:0]]);
wire [11:0] term_r = {4'd0, lb_q[23:16]} * {7'd0, w_p2};
wire [11:0] term_g = {4'd0, lb_q[15:8]}  * {7'd0, w_p2};
wire [11:0] term_b = {4'd0, lb_q[7:0]}   * {7'd0, w_p2};
wire [11:0] sum_r = acc_r + (ok_p2 ? term_r : 12'd0);
wire [11:0] sum_g = acc_g + (ok_p2 ? term_g : 12'd0);
wire [11:0] sum_b = acc_b + (ok_p2 ? term_b : 12'd0);

always @(posedge clk) begin
	if (reset) begin
		rphase <= 2'd0; rx <= 0; rx_ok <= 0; acc_r <= 0; acc_g <= 0; acc_b <= 0;
		w_p1 <= 0; w_p2 <= 0; ok_p1 <= 0; ok_p2 <= 0; show <= 0; lb_ra <= 0;
		hs_o <= 0; vs_o <= 0; hb_o <= 1; vb_o <= 1; field_o <= 0; r_o <= 0; g_o <= 0; b_o <= 0;
	end
	else if (native) begin
		if (dot_ce) begin
			r_o <= r_i; g_o <= g_i; b_o <= b_i;
			hs_o <= hs_i; vs_o <= vs_i; hb_o <= ~de_i; vb_o <= ~de_i; field_o <= 0;
		end
	end
	else begin
		w_p2 <= w_p1;
		ok_p2 <= ok_p1;
		if (ce_out) begin
			// the sample whose reads went out after the last tick
			{r_o, g_o, b_o} <= show ? {sum_r[11:4], sum_g[11:4], sum_b[11:4]} : 24'd0;
			hs_o <= (ocnt >= h_front) && (ocnt < h_front + h_sync);
			hb_o <= (ocnt < h_blank) || (!show_blank && (!rx_ok || (rx < px0) || (rx > px1)));
			vs_o <= (vsh_n < VS_HALF);
			vb_o <= show_blank ? !vis_row : (!y_ok_n || (oline_n < py0) || (oline_n > py1));
			field_o <= field_n;
			// first read for the next sample
			rx <= x_next[9:0];
			rx_ok <= x_ok_n;
			show <= line_ok && y_ok_n && (ocnt_n >= h_blank) && x_ok_n;
			acc_r <= 0; acc_g <= 0; acc_b <= 0;
			lb_ra <= {first_line[2:0], x_next[9:0]};
			w_p1 <= first_w;
			ok_p1 <= chk_ok;
			rphase <= 2'd1;
		end
		else if (rphase == 2'd1) begin
			if (ntap != 2'd1) begin
				lb_ra <= {r1_line[2:0], rx};
				w_p1 <= r1_w;
				ok_p1 <= chk_ok;
			end
			rphase <= 2'd2;
		end
		else if (rphase == 2'd2) begin
			// the first value is on lb_q
			if (ntap != 2'd1) begin acc_r <= sum_r; acc_g <= sum_g; acc_b <= sum_b; end
			if (ntap == 2'd3) begin
				lb_ra <= {r2_line[2:0], rx};
				w_p1 <= w0;
				ok_p1 <= chk_ok;
			end
			rphase <= 2'd3;
		end
		else if (rphase == 2'd3) begin
			// the second value is on lb_q; the third is added at the tick
			if (ntap == 2'd3) begin acc_r <= sum_r; acc_g <= sum_g; acc_b <= sum_b; end
			rphase <= 2'd0;
		end
	end
end

assign ce_pix = native ? dot_ce : ce_out;

endmodule

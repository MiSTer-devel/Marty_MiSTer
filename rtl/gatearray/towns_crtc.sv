// Copyright (c) 2026 Jamie Blanks
//
// Towns CRTC and video output control: the 32 internal registers behind
// 0440/0442, the raster in dot clocks and half-lines, two layers with
// their own display windows, zoom and VRAM address generation, the
// palettes at FD90-FD9F, the sifter registers at 0448/044A, status at
// 044C and FDA0, and the VSYNC interrupt.
//
//   hcnt 0..HST (dots)            vcnt 0..VST (half-lines, +1 at 0 and HST/2)
//   HSYNC = hcnt < HSW1           VSYNC = VST1 <= vcnt < VST2
//   layer window: HDS..HDE x VDS..VDE   shift register starts at HAJ
//
// Each layer's line is transferred into a two-bank line buffer (the VRAM
// serial register) one raster line ahead: while line n is displayed from
// one bank, line n+1 is fetched into the other. A bank holds 256 dwords,
// the 1024 bytes of the widest 32768-colour line (512 px). Line addresses are FA
// (+FO on the odd field) then +LO every ZV+1 raster lines, in units of 4
// bytes (two-page mode) or 8 bytes (one-page mode, halves interleaved).
// With CEN clear the low eight bits of the unit counter do not carry, so
// a line wraps on itself (cylindrical scroll).
//
// Pixel pipeline, one stage per dot: line buffer read, palette, mix.
//
// The render side (line addresses, fetch, line buffers, pipeline) runs on
// its own counters. Natively they follow the raster; in pull mode the
// scan converter advances them a line at a time, the way the Marty's
// converter pulls lines from VRAM, and the raster only keeps the timing
// that software sees.

module towns_crtc
(
	input             clk,
	input             reset,
	input       [3:0] ce_clk,         // dot enables for CLKSEL 0..3

	// I/O byte lane
	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output            io_sel,

	// savestate port: registers, palettes and the raster position as
	// bytes; 90h streams the 256-colour palette R, G, B per entry and
	// any access to 93h rewinds it
	input             ss_cs,
	input             ss_wr,
	input             ss_step,        // one clock at the end of each engine cycle
	input       [7:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output            ss_quiet,       // no line fetch on its way

	// sprite controller status for 044C and the layer 1 page flip
	input             sp_busy,
	input             sp_page,
	input             sp_disp_page,

	// FMR display register bits (CFF82)
	input       [3:0] fmr_plane_mask,
	input             fmr_ps2,

	// VRAM line fetch: one 64-bit physical word per request
	output reg        fetch_req,      // held with fetch_a until fetch_ack
	output reg [18:3] fetch_a,
	input             fetch_ack,
	input      [63:0] fetch_data,
	input             fetch_valid,
	// pull mode: the converter drives the render side
	input             pull,
	input             ce_pull,        // render dot enable in pull mode
	input             pull_frame,     // start a field
	input             pull_field,
	input             pull_line,      // render the next line
	input             pull_skip,      // pass the next line without rendering
	input             sync_in,        // converter frame start: pull the raster into phase
	input             follow_fa,      // the source frame runs free of the output: FA writes move the render walk
	output            r_ready,        // a request may be issued
	output            r_ack,          // the request was taken
	output            r_ce,           // enable the render outputs advance on
	output reg [10:0] r_x,            // dot position of the pixel on r, g, b
	// geometry for the converter: HST, CLKSEL, the picture in half-lines
	output     [10:0] geo_hst,
	output      [1:0] geo_clksel,
	output     [10:0] geo_vds,
	output     [10:0] geo_vde,
	output     [10:0] geo_hds,
	output     [10:0] geo_hde,
	output     [10:0] geo_vst,
	output            geo_zv1,        // every shown layer doubles its lines
	// video
	output            dot_ce,
	output reg  [7:0] r,
	output reg  [7:0] g,
	output reg  [7:0] b,
	output reg        hs,
	output reg        vs,
	output reg        de,             // some layer window is open
	output reg        field,
	output reg        line_start,     // one dot_ce at hcnt 0
	output reg        vsync_irq,
	output            in_hsync,       // status for FDA0 and CFF86
	output            in_vsync,
	output     [10:0] t_hcnt,         // timing raster position: dot
	output     [10:0] t_vcnt,         // and half line
	output    [127:0] dbg_regs        // timing and mode registers for the beacon
);

// ---- registers ----
reg [15:0] cr [0:31];
reg  [4:0] ra;
reg  [1:0] sifter_ra;
reg  [7:0] sifter0, sifter1;   // 0448 sub-registers: control, priority
reg  [3:0] fda0;
reg        dpmd;
reg  [3:0] dpal [0:7];
reg  [7:0] pal_code;

localparam R_HSW1 = 0, R_HSW2 = 1, R_HST = 4, R_VST1 = 5, R_VST2 = 6, R_VST = 8,
           R_HDS0 = 9, R_HDE0 = 10, R_HDS1 = 11, R_HDE1 = 12, R_VDS0 = 13, R_VDE0 = 14, R_VDS1 = 15, R_VDE1 = 16,
           R_FA0 = 17, R_HAJ0 = 18, R_FO0 = 19, R_LO0 = 20, R_FA1 = 21, R_HAJ1 = 22, R_FO1 = 23, R_LO1 = 24,
           R_ZOOM = 27, R_CR0 = 28, R_CR1 = 29, R_FR = 30;

wire [10:0] hsw1 = cr[R_HSW1][10:0];
wire [10:0] hst  = cr[R_HST][10:0];
wire [10:0] vst1 = cr[R_VST1][10:0];
wire [10:0] vst2 = cr[R_VST2][10:0];
wire [10:0] vst  = cr[R_VST][10:0];
wire [15:0] zoom = cr[R_ZOOM];
wire [15:0] cr0  = cr[R_CR0];
wire  [1:0] clksel = cr[R_CR1][1:0];
wire        start  = cr0[15];
wire        pmode  = sifter0[4];    // 1 = two layers
assign dbg_regs = {cr[R_HST], cr[R_VST], cr[R_CR0], cr[R_CR1],
                   cr[R_HDS0], cr[R_VDS0], cr[R_FA0], sifter0, sifter1};
wire        pri    = sifter1[0];    // 1 = layer 1 in front
wire  [1:0] plt    = sifter1[5:4];

// bits per pixel from CR0 CL: two-page 01 = 16 bit, 11 = 4 bit; one-page 10 = 16, 11 = 8
wire  [1:0] cl0 = cr0[1:0], cl1 = cr0[3:2];
wire        bpp16_0 = pmode ? (cl0 == 2'd1) : (cl0 == 2'd2);
wire        bpp8_0  = !pmode && (cl0 == 2'd3);
wire        bpp16_1 = pmode ? (cl1 == 2'd1) : (cl1 == 2'd2);
wire        bpp8_1  = !pmode && (cl1 == 2'd3);
// The video output controller has its own colour setting in the same
// register as PMODE: CL bits 11 = 32768 colours, 10 = 256 (one page only),
// 01 = 16. The CRTC's CL above sets how VRAM is fetched; this one sets how
// the fetched values become colours, so a program that reprograms the CRTC
// first still shows its pixels through the palette it is on.
wire        col16_0 = sifter0[1:0] == 2'b11;
wire        col8_0  = !pmode && (sifter0[1:0] == 2'b10);
wire        col16_1 = sifter0[3:2] == 2'b11;

// display enables: sifter bits and FDA0 both have to allow the layer
wire        en0 = start && (pmode ? sifter0[0] : sifter0[3]) && (fda0[3] | fda0[2]);
wire        en1 = start && pmode && sifter0[2] && (fda0[1] | fda0[0]);

// ---- raster counters ----
assign dot_ce = ce_clk[clksel];

reg [10:0] hcnt, vcnt;
assign t_hcnt = hcnt;
assign t_vcnt = vcnt;
wire [11:0] hst_p1 = {1'b0, hst} + 12'd1;
wire [10:0] half   = hst_p1[11:1];
wire        h_last = (hcnt == hst);
wire        h_half = (hcnt == half);
// The period is VST+1 half-lines. An odd period (VST even) is interlace:
// the reset alternates between the line end and the half-line point and
// FIELD says which. An even period only resets at a line end, so the
// vertical timing stays locked to the line after a register change.
wire        v_last  = (vcnt >= vst);
wire        v_reset = v_last && (h_last || (h_half && !vst[0]));
// The converter's frame start restarts a raster that is more than a line
// out of phase with it; one that is close keeps its own reset.
reg         sync_req;
wire        v_near  = (vcnt <= 11'd2) || (vcnt + 11'd2 >= vst);

always @(posedge clk) begin
	if (reset) begin
		hcnt <= 0;
		vcnt <= 0;
		field <= 0;
		line_start <= 0;
		sync_req <= 0;
	end
	else if (ss_cs && ss_wr && ss_a[7:2] == 6'b010010) begin   // 4A-4D: the raster position
		case (ss_a[1:0])
		2'd0: hcnt[7:0]  <= ss_din;
		2'd1: {field, hcnt[10:8]} <= {ss_din[7], ss_din[2:0]};
		2'd2: vcnt[7:0]  <= ss_din;
		default: vcnt[10:8] <= ss_din[2:0];
		endcase
	end
	else begin
		if (sync_in) sync_req <= !v_near;
		if (dot_ce) begin
			line_start <= 0;
			if (h_last) begin
				hcnt <= 0;
				line_start <= 1;
			end
			else hcnt <= hcnt + 1'd1;
			if (h_last || h_half) begin
				if (v_reset || (sync_req && h_last)) begin
					vcnt <= 0;
					field <= h_half;
					sync_req <= 0;
				end
				else vcnt <= vcnt + 1'd1;
			end
		end
	end
end

wire hsync_w = (hcnt < hsw1);
wire vsync_w = (vcnt >= vst1) && (vcnt < vst2);
assign in_hsync = hsync_w;
assign in_vsync = vsync_w;
assign geo_hst    = hst;
assign geo_vst    = vst;
assign geo_clksel = clksel;

// ---- render counters ----
// Natively a copy of the raster. In pull mode a line runs from just
// before the earliest window to just after the latest, then jumps to the
// line end; a skipped line only moves the vertical bookkeeping. Two
// skips prime the two-line fetch lookahead at a field start.
reg  [10:0] r_hcnt, r_vcnt;
reg         r_field, r_run, r_skip;
reg   [1:0] r_prime;
wire        p_ce    = pull ? ce_pull : dot_ce;
wire [10:0] p_hcnt  = pull ? r_hcnt : hcnt;
wire [10:0] p_vcnt  = pull ? (r_skip ? r_vcnt + 1'd1 : r_vcnt) : vcnt;   // a skip stands for a line end
wire        p_field = pull ? r_field : field;
wire        f_idle;
wire        r_last   = r_run && (r_hcnt == hst) && f_idle;
wire        p_h_last = pull ? (r_last || r_skip) : h_last;
assign r_ready = pull && !r_run && !r_skip && (r_prime == 0);
assign r_ack   = r_ready && (pull_line || (pull_skip && f_idle));
assign r_ce    = p_ce;

wire [10:0] r_hstart, r_hend;
always @(posedge clk) begin
	if (reset) begin
		r_hcnt <= 0; r_vcnt <= 0; r_field <= 0; r_run <= 0; r_skip <= 0; r_prime <= 0;
	end
	else begin
		if (pull_frame) begin
			// two lines before the top of the field, in the old field
			r_vcnt <= (vst >= 11'd3) ? vst - 11'd3 : 11'd0;
			r_field <= ~pull_field;
			r_prime <= 2'd2;
			r_run <= 0;
		end
		else if (r_ready && pull_line) begin
			r_run <= 1;
			r_hcnt <= r_hstart;
			if (r_hstart > half) r_vcnt <= r_vcnt + 1'd1;
		end
		else if ((r_ready && pull_skip && f_idle) || (r_prime != 0 && !r_run && !r_skip && f_idle)) begin
			r_skip <= 1;
			if (r_prime != 0) r_prime <= r_prime - 1'd1;
		end
		if (r_skip && p_ce) begin
			r_skip <= 0;
			if (r_vcnt + 1'd1 >= vst) begin r_vcnt <= 0; r_field <= pull_field; end
			else r_vcnt <= r_vcnt + 11'd2;
		end
		else if (r_run && p_ce) begin
			if (r_hcnt == hst) begin
				// the line ends once the next fetch is in: the flip would abort it
				if (f_idle) begin
					r_run <= 0;
					if (r_vcnt >= vst) begin r_vcnt <= 0; r_field <= pull_field; end
					else r_vcnt <= r_vcnt + 1'd1;
				end
			end
			else if (r_hcnt == r_hend && r_hend < hst) begin
				r_hcnt <= hst;
				if (r_hcnt < half) r_vcnt <= r_vcnt + 1'd1;
			end
			else begin
				r_hcnt <= r_hcnt + 1'd1;
				if (r_hcnt == half) r_vcnt <= r_vcnt + 1'd1;
			end
		end
	end
end

// ---- per-layer geometry ----
wire [10:0] hds [0:1];
wire [10:0] hde [0:1];
wire [10:0] vds [0:1];
wire [10:0] vde [0:1];
wire [15:0] fa  [0:1];
wire [10:0] haj [0:1];
wire [15:0] fo  [0:1];
wire [15:0] lo  [0:1];
wire  [3:0] zh  [0:1];
wire  [3:0] zv  [0:1];
wire        cen [0:1];
wire        bpp16 [0:1];
wire        bpp8  [0:1];
wire        en    [0:1];

assign hds[0] = cr[R_HDS0][10:0];  assign hds[1] = cr[R_HDS1][10:0];
assign hde[0] = cr[R_HDE0][10:0];  assign hde[1] = cr[R_HDE1][10:0];
assign vds[0] = cr[R_VDS0][10:0];  assign vds[1] = cr[R_VDS1][10:0];
assign vde[0] = cr[R_VDE0][10:0];  assign vde[1] = cr[R_VDE1][10:0];
assign fa[0]  = cr[R_FA0];         assign fa[1]  = cr[R_FA1];
assign haj[0] = cr[R_HAJ0][10:0];  assign haj[1] = cr[R_HAJ1][10:0];
assign fo[0]  = cr[R_FO0];         assign fo[1]  = cr[R_FO1];
assign lo[0]  = cr[R_LO0];         assign lo[1]  = cr[R_LO1];
assign zh[0]  = zoom[3:0];         assign zh[1]  = zoom[11:8];
assign zv[0]  = zoom[7:4];         assign zv[1]  = zoom[15:12];
assign cen[0] = cr0[4];            assign cen[1] = cr0[5];

// render window across both layers, with room for the pipeline
wire [10:0] h_lo0 = (haj[0] < hds[0]) ? haj[0] : hds[0];
wire [10:0] h_lo1 = (haj[1] < hds[1]) ? haj[1] : hds[1];
wire [10:0] h_lo  = (en[0] && en[1]) ? ((h_lo0 < h_lo1) ? h_lo0 : h_lo1) : en[1] ? h_lo1 : h_lo0;
wire [10:0] h_hi  = (en[0] && en[1]) ? ((hde[0] > hde[1]) ? hde[0] : hde[1]) : en[1] ? hde[1] : hde[0];
assign r_hstart = (h_lo > 11'd4) ? h_lo - 11'd4 : 11'd0;
assign r_hend   = h_hi + 11'd4;
// the span that actually carries pixels: the shift register starts at HAJ,
// so a layer whose HAJ sits inside its window begins there, and a window
// running past the end of the line stops with the line
wire [10:0] p_lo0 = (haj[0] > hds[0]) ? haj[0] : hds[0];
wire [10:0] p_lo1 = (haj[1] > hds[1]) ? haj[1] : hds[1];
wire [10:0] p_hi0 = (hde[0] > hst) ? hst : hde[0];
wire [10:0] p_hi1 = (hde[1] > hst) ? hst : hde[1];
assign geo_hds  = (en[0] && en[1]) ? ((p_lo0 < p_lo1) ? p_lo0 : p_lo1) : en[1] ? p_lo1 : p_lo0;
assign geo_hde  = (en[0] && en[1]) ? ((p_hi0 > p_hi1) ? p_hi0 : p_hi1) : en[1] ? p_hi1 : p_hi0;
assign geo_vds  = (en[0] && en[1]) ? ((vds[0] < vds[1]) ? vds[0] : vds[1]) : en[1] ? vds[1] : en[0] ? vds[0] : 11'd0;
assign geo_vde  = (en[0] && en[1]) ? ((vde[0] > vde[1]) ? vde[0] : vde[1]) : en[1] ? vde[1] : en[0] ? vde[0] : 11'd0;
assign geo_zv1  = (!en[0] || zv[0] == 4'd1) && (!en[1] || zv[1] == 4'd1) && (en[0] || en[1]);
assign bpp16[0] = bpp16_0;         assign bpp16[1] = bpp16_1;
assign bpp8[0]  = bpp8_0;          assign bpp8[1]  = bpp8_1;
wire        col16 [0:1];
wire        col8  [0:1];
assign col16[0] = col16_0;         assign col16[1] = col16_1;
assign col8[0]  = col8_0;          assign col8[1]  = 1'b0;
assign en[0]  = en0;               assign en[1]  = en1;

// second half-line of the line after the one about to start: that is the
// line the fetcher loads while the next one is displayed
wire [11:0] v_next_raw = {1'b0, p_vcnt} + 12'd4;
wire        v_wrap     = (v_next_raw > {1'b0, vst});   // approximate across the reset
wire [11:0] v_next_w   = v_next_raw - {1'b0, vst} - 12'd1;
wire [10:0] v_next     = v_wrap ? v_next_w[10:0] : v_next_raw[10:0];
wire        next_field = v_wrap ? ~p_field : p_field;
wire [10:0] v_next_m1  = v_next - 11'd1;
wire        v_next_m1_ok = (v_next != 0);

// ---- line address generation and fetch, per layer ----
// nl_start: first unit of the next raster line; fetched into bank ~cur_bank.
reg  [15:0] nl_start [0:1];
reg   [3:0] nl_zv    [0:1];
reg         nl_valid [0:1];      // the next line is inside the window
reg         cur_bank;
reg   [7:0] dw_need  [0:1];      // dwords the last displayed line read, plus slack
reg   [7:0] dw_max   [0:1];
reg         dw_seen  [0:1];      // that line was inside the window

// A source whose frame rate differs from the output's has its output
// frames rendered across the game's page flips. The walk then moves with
// an FA write at once, so the rest of the frame comes from the page just
// shown instead of the one the game starts redrawing.
reg  [15:0] fa_seen  [0:1];      // the FA the walk's address is based on
wire [15:0] fa_step  [0:1];
assign fa_step[0] = follow_fa ? fa[0] - fa_seen[0] : 16'd0;
assign fa_step[1] = follow_fa ? fa[1] - fa_seen[1] : 16'd0;

integer L;
always @(posedge clk) begin
	if (reset) begin
		for (L = 0; L < 2; L = L + 1) begin
			nl_start[L] <= 0;
			nl_zv[L] <= 0;
			nl_valid[L] <= 0;
			dw_need[L] <= 8'hFF;
			fa_seen[L] <= 0;
		end
		cur_bank <= 0;
	end
	else begin
		if (pull_frame) begin
			nl_valid[0] <= 0;
			nl_valid[1] <= 0;
		end
		else if (p_ce && p_h_last) begin
			cur_bank <= ~cur_bank;
			for (L = 0; L < 2; L = L + 1) begin
				// the fetched line is half-lines v_next-1 and v_next. It is the first
				// line when the window opens inside it; when the window opened in the
				// second half of the line before, it is the second line.
				// The walk follows the raster whether or not the sifter shows the
				// layer: a game that flips layers mid-frame gets the new layer from
				// the current line, not from FA again.
				if (((v_next >= vds[L] && v_next < vde[L]) || (v_next_m1_ok && v_next_m1 >= vds[L] && v_next_m1 < vde[L])) && start) begin
					if (!nl_valid[L] && v_next == vds[L] + 11'd2) begin
						nl_start[L] <= fa[L] + (next_field ? fo[L] : 16'd0) + ((zv[L] == 0) ? lo[L] : 16'd0);
						nl_zv[L] <= (zv[L] == 0) ? 4'd0 : 4'd1;
						nl_valid[L] <= 1;
					end
					else if (!nl_valid[L] || v_next == vds[L] || v_next == vds[L] + 1'd1) begin
						nl_start[L] <= fa[L] + (next_field ? fo[L] : 16'd0);
						nl_zv[L] <= 0;
						nl_valid[L] <= 1;
					end
					else begin
						if (nl_zv[L] == zv[L]) begin
							nl_start[L] <= nl_start[L] + lo[L] + fa_step[L];
							nl_zv[L] <= 0;
						end
						else begin
							nl_start[L] <= nl_start[L] + fa_step[L];
							nl_zv[L] <= nl_zv[L] + 1'd1;
						end
					end
					// a displayed line tells how much the following ones need; one
					// the layer was turned off in has been read only up to that point
					if (dw_seen[L] && en[L]) dw_need[L] <= (dw_max[L] > 8'd251) ? 8'hFF : dw_max[L] + 8'd4;
				end
				else nl_valid[L] <= 0;
				fa_seen[L] <= fa[L];
			end
		end
		// a register change: fetch whole lines until a displayed one says how much
		if (io_wr && (io_addr == 16'h0442 || io_addr == 16'h0443 || io_addr == 16'h044A)) begin
			dw_need[0] <= 8'hFF;
			dw_need[1] <= 8'hFF;
		end
	end
end

// The fetcher walks both layers' next lines after each line start. A read
// covers two consecutive units. In two-page mode it gives two line dwords;
// in one-page mode reads alternate between the halves and each gives two
// of the four linear dwords of the chunk. f_base is the line-buffer index
// of the first dword a read delivers, negative when the line starts inside
// the aligned pair.
//
// Up to two reads are outstanding. Each request carries its own layer,
// bank and buffer index in a small queue, so an answer is written where
// it belongs even after a line restart; the address counters step at
// issue. Answers park in the queue entry and the writer drains them in
// order, two dwords on consecutive clocks, so answers may arrive back
// to back.
reg         f_layer, f_run, f_half, f_second;
reg  [15:0] f_u;               // unit counter, even aligned
reg   [9:0] f_base;            // signed line-buffer index
reg         f_kick, f_pending; // line start seen: restart the walk
reg   [1:0] f_out;             // entries issued and not yet written
reg         fq_wp, fq_ap, fq_rp;   // issue, answer and write pointers
reg         fq_layer [0:1];
reg         fq_bank  [0:1];
reg   [9:0] fq_idx0  [0:1];
(* ramstyle = "logic" *) reg [63:0] fq_data [0:1];   // read the clock it is used: registers, not a RAM
reg   [1:0] fq_done;
reg         f_hold;            // a request is out, waiting for its ack

wire [15:0] f_uw = cen[f_layer] ? f_u : {nl_start[f_layer][15:8], f_u[7:0]};

// physical 64-bit word: two-page layer L at L*0x40000; one-page even
// dwords in the low half, odd in the high half
reg  [18:3] f_phys;
always @* begin
	f_phys = {pmode ? f_layer : f_half, f_uw[15:1]};
	// FMR page 1 shows in the first 128 KB of layer 0
	if (f_phys[18] == 0 && f_phys[17] == 0 && fmr_ps2) f_phys[17] = 1;
	// sprite display page selects the layer 1 half
	if (pmode && f_layer && sp_disp_page) f_phys[17] = ~f_phys[17];
end

// line buffer write side
reg         lb_we [0:1];
reg   [8:0] lb_wa [0:1];
reg  [31:0] lb_wd [0:1];
wire  [9:0] f_idx0 = f_base + {9'd0, !pmode && f_half};        // first dword of the read
wire  [9:0] f_step = pmode ? 10'd2 : (f_half ? 10'd4 : 10'd0);
// a hidden layer is fetched all the same: a page flip through FDA0 or the
// sifter between the fetch and the display would otherwise show the bank
// left over from the last time the layer was on
wire        f_done = !nl_valid[f_layer] || (f_layer && !pmode) || (!f_base[9] && f_base[8:0] >= {1'b0, dw_need[f_layer]});
wire        f_issue = f_run && !f_hold && (f_out != 2'd2) && !f_pending && !f_kick && !f_done;
assign ss_quiet = !fetch_req && !f_hold && f_out == 2'd0;
wire  [9:0] c_idx0 = fq_idx0[fq_rp];
wire  [9:0] c_idx1 = c_idx0 + (pmode ? 10'd1 : 10'd2);
wire        f_write = fq_done[fq_rp] && !f_second;   // an answer waits and the writer is free

always @(posedge clk) begin
	if (reset) begin
		f_run <= 0;
		fetch_req <= 0;
		f_kick <= 0;
		f_pending <= 0;
		f_out <= 0;
		fq_wp <= 0; fq_ap <= 0; fq_rp <= 0; fq_done <= 0;
		f_hold <= 0;
		lb_we[0] <= 0;
		lb_we[1] <= 0;
		f_second <= 0;
	end
	else begin
		lb_we[0] <= 0;
		lb_we[1] <= 0;
		f_kick <= p_ce && p_h_last;
		if (f_kick) f_pending <= 1;

		// a request stays up until the controller takes it
		if (fetch_ack) begin
			fetch_req <= 0;
			f_hold <= 0;
		end

		if (f_pending || f_kick) begin
			// a new line: restart the walk; reads still out land in their own bank
			f_pending <= 0;
			f_layer <= 0;
			f_run <= 1;
			f_u <= {nl_start[0][15:1], 1'b0};
			f_base <= pmode ? {10{nl_start[0][0]}} : {{9{nl_start[0][0]}}, 1'b0};
			f_half <= 0;
		end
		else if (f_run && !f_hold && f_done) begin
			if (f_layer) f_run <= 0;
			else begin
				f_layer <= 1;
				f_u <= {nl_start[1][15:1], 1'b0};
				f_base <= pmode ? {10{nl_start[1][0]}} : {{9{nl_start[1][0]}}, 1'b0};
				f_half <= 0;
			end
		end
		else if (f_issue) begin
			fetch_a <= f_phys;
			fetch_req <= 1;
			f_hold <= 1;
			fq_layer[fq_wp] <= f_layer;
			fq_bank[fq_wp]  <= ~cur_bank;
			fq_idx0[fq_wp]  <= f_idx0;
			fq_wp <= ~fq_wp;
			f_base <= f_base + f_step;
			if (pmode || f_half) f_u <= f_u + 16'd2;
			if (!pmode) f_half <= ~f_half;
		end
		f_out <= f_out + {1'b0, f_issue} - {1'b0, f_write};

		// answers park in their entry
		if (fetch_valid) begin
			fq_data[fq_ap] <= fetch_data;
			fq_done[fq_ap] <= 1;
			fq_ap <= ~fq_ap;
		end

		// the writer: first dword, then the second next clock
		if (f_write) begin
			if (!c_idx0[9] && !c_idx0[8]) begin
				lb_we[fq_layer[fq_rp]] <= 1;
				lb_wa[fq_layer[fq_rp]] <= {fq_bank[fq_rp], c_idx0[7:0]};
				lb_wd[fq_layer[fq_rp]] <= fq_data[fq_rp][31:0];
			end
			f_second <= 1;
		end
		else if (f_second) begin
			if (!c_idx1[9] && !c_idx1[8]) begin
				lb_we[fq_layer[fq_rp]] <= 1;
				lb_wa[fq_layer[fq_rp]] <= {fq_bank[fq_rp], c_idx1[7:0]};
				lb_wd[fq_layer[fq_rp]] <= fq_data[fq_rp][63:32];
			end
			f_second <= 0;
			fq_done[fq_rp] <= 0;
			fq_rp <= ~fq_rp;
		end
	end
end

assign f_idle = !f_run && (f_out == 2'd0) && !f_pending && !f_kick && !f_second && !f_hold;
// an entry is free again only once written, so the issue side waits on f_out

// line buffers: 512 dwords per layer (two banks)
wire [31:0] lb_q [0:1];
reg   [8:0] lb_ra [0:1];

cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(32)) lb0
(
	.clk_i(clk),
	.addr_a_i(lb_wa[0]), .wren_a_i(lb_we[0]), .wdata_a_i(lb_wd[0]), .q_a_o(),
	.addr_b_i(lb_ra[0]), .wren_b_i(1'b0), .wdata_b_i(32'd0), .q_b_o(lb_q[0])
);

cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(32)) lb1
(
	.clk_i(clk),
	.addr_a_i(lb_wa[1]), .wren_a_i(lb_we[1]), .wdata_a_i(lb_wd[1]), .q_a_o(),
	.addr_b_i(lb_ra[1]), .wren_b_i(1'b0), .wdata_b_i(32'd0), .q_b_o(lb_q[1])
);

// ---- readout: shift register position per layer ----
reg   [9:0] px_cnt [0:1];
reg   [3:0] zh_cnt [0:1];
reg         started [0:1];
wire        v_act [0:1];
assign v_act[0] = (p_vcnt >= vds[0]) && (p_vcnt < vde[0]) && en[0];
assign v_act[1] = (p_vcnt >= vds[1]) && (p_vcnt < vde[1]) && en[1];

// position of the pixel at this dot: the shift register restarts at HAJ
wire  [9:0] px_now [0:1];
wire        started_now [0:1];
wire        win_now [0:1];
wire  [9:0] dw_of_px [0:1];
wire  [2:0] sub_of_px [0:1];
generate
	genvar G;
	for (G = 0; G < 2; G = G + 1) begin : g_pos
		assign started_now[G] = started[G] || (p_hcnt == haj[G]);
		assign px_now[G]      = (p_hcnt == haj[G]) ? 10'd0 : px_cnt[G];
		assign win_now[G]     = started_now[G] && v_act[G] && (p_hcnt >= hds[G]) && (p_hcnt < hde[G]);
		assign dw_of_px[G]    = bpp16[G] ? {1'b0, px_now[G][9:1]} : bpp8[G] ? {2'd0, px_now[G][9:2]} : {3'd0, px_now[G][9:3]};
		assign sub_of_px[G]   = bpp16[G] ? {2'd0, px_now[G][0]}  : bpp8[G] ? {1'b0, px_now[G][1:0]} : px_now[G][2:0];
	end
endgenerate

// pipeline: s1 value select, s2 palette, s3 output
reg   [2:0] s1_sub [0:1];
reg         s1_win [0:1];
reg         s1_hs, s1_vs, s1_de;
reg  [10:0] s1_x, s2_x, s3_x;
reg  [15:0] s2_val [0:1];
reg         s2_win [0:1];
reg         s2_hs, s2_vs, s2_de;
reg         s2_tr  [0:1];
reg  [23:0] s3_rgb [0:1];
reg         s3_win [0:1];
reg         s3_tr  [0:1];
reg         s3_hs, s3_vs, s3_de;

// 16-colour palettes as registers, 4 bits per gun
reg  [11:0] pal16 [0:31];   // {r, g, b}, entries 0-15 layer 0, 16-31 layer 1
wire  [4:0] pal16_ix [0:1]; // each layer's entry for its current pixel
wire [11:0] pal16_px [0:1];
generate
	for (G = 0; G < 2; G = G + 1) begin : g_pal
		assign pal16_ix[G] = (G == 0) ? {1'b0, s2_val[G][3:0]} : {1'b1, s2_val[G][3:0]};
		assign pal16_px[G] = pal16[pal16_ix[G]];
	end
endgenerate
reg   [7:0] p256_ra;
wire  [7:0] p256_r, p256_g, p256_b;
wire  [7:0] p256_cr, p256_cg, p256_cb;   // CPU side read-back
reg   [7:0] ss_pidx;                     // palette stream entry
reg   [1:0] ss_pph;                      // and its colour: 0 R, 1 G, 2 B
wire        ss_pal = ss_cs && ss_a == 8'h90;
wire  [7:0] pa_addr = ss_cs ? ss_pidx : pal_code;

always @(posedge clk) begin
	if (reset) begin
		for (L = 0; L < 2; L = L + 1) begin
			px_cnt[L] <= 0;
			zh_cnt[L] <= 0;
			started[L] <= 0;
			dw_max[L] <= 0;
			dw_seen[L] <= 0;
		end
	end
	else if (p_ce) begin
		for (L = 0; L < 2; L = L + 1) begin
			if (p_h_last) begin
				started[L] <= 0;
				dw_max[L] <= 0;
				dw_seen[L] <= 0;
			end
			else if (p_hcnt == haj[L]) begin
				started[L] <= 1;
				px_cnt[L] <= (zh[L] == 0) ? 10'd1 : 10'd0;
				zh_cnt[L] <= (zh[L] == 0) ? 4'd0 : 4'd1;
			end
			else if (started[L]) begin
				if (zh_cnt[L] == zh[L]) begin
					zh_cnt[L] <= 0;
					px_cnt[L] <= px_cnt[L] + 1'd1;
				end
				else zh_cnt[L] <= zh_cnt[L] + 1'd1;
			end

			// stage 0: line buffer address
			lb_ra[L] <= {cur_bank, dw_of_px[L][7:0]};
			s1_sub[L] <= sub_of_px[L];
			s1_win[L] <= win_now[L];
			if (win_now[L]) dw_seen[L] <= 1;
			if (win_now[L] && dw_of_px[L][7:0] > dw_max[L]) dw_max[L] <= dw_of_px[L][7:0];

			// stage 1: value out of the dword
			if (bpp16[L]) begin
				s2_val[L] <= s1_sub[L][0] ? lb_q[L][31:16] : lb_q[L][15:0];
				s2_tr[L]  <= s1_sub[L][0] ? lb_q[L][31] : lb_q[L][15];
			end
			else if (bpp8[L]) begin
				s2_val[L] <= {8'd0, lb_q[L][{s1_sub[L][1:0], 3'd0} +: 8]};
				s2_tr[L]  <= (lb_q[L][{s1_sub[L][1:0], 3'd0} +: 8] == 8'd0);
			end
			else begin
				s2_val[L] <= {12'd0, lb_q[L][{s1_sub[L], 2'd0} +: 4] & (L == 0 ? fmr_plane_mask : 4'hF)};
				s2_tr[L]  <= ((lb_q[L][{s1_sub[L], 2'd0} +: 4] & (L == 0 ? fmr_plane_mask : 4'hF)) == 4'd0);
			end
			s2_win[L] <= s1_win[L];

			// stage 2: colour, by the output controller's setting
			if (col16[L]) s3_rgb[L] <= {s2_val[L][9:5], s2_val[L][9:7], s2_val[L][14:10], s2_val[L][14:12], s2_val[L][4:0], s2_val[L][4:2]};
			else if (col8[L]) s3_rgb[L] <= {p256_r, p256_g, p256_b};
			else s3_rgb[L] <= {pal16_px[L][11:8], pal16_px[L][11:8], pal16_px[L][7:4], pal16_px[L][7:4], pal16_px[L][3:0], pal16_px[L][3:0]};
			s3_win[L] <= s2_win[L];
			s3_tr[L]  <= s2_tr[L];
		end
		p256_ra <= p256_val;

		s1_hs <= hsync_w; s1_vs <= vsync_w; s1_de <= win_now[0] | win_now[1]; s1_x <= p_hcnt;
		s2_hs <= s1_hs;   s2_vs <= s1_vs;   s2_de <= s1_de;   s2_x <= s1_x;
		s3_hs <= s2_hs;   s3_vs <= s2_vs;   s3_de <= s2_de;   s3_x <= s2_x;

		// stage 3: mix. The front layer shows unless transparent; the back
		// layer shows its colour even for value 0; nothing gives black.
		// A single page is the back layer whatever PRI says.
		hs <= s3_hs; vs <= s3_vs; de <= s3_de; r_x <= s3_x;
		if (!pmode) begin
			if (s3_win[0]) {r, g, b} <= s3_rgb[0];
			else {r, g, b} <= 24'd0;
		end
		else if (pri) begin
			if (s3_win[1] && !s3_tr[1]) {r, g, b} <= s3_rgb[1];
			else if (s3_win[0]) {r, g, b} <= s3_rgb[0];
			else {r, g, b} <= 24'd0;
		end
		else begin
			if (s3_win[0] && !s3_tr[0]) {r, g, b} <= s3_rgb[0];
			else if (s3_win[1]) {r, g, b} <= s3_rgb[1];
			else {r, g, b} <= 24'd0;
		end
	end
end

// 256-colour palette: three guns, CPU port A, display port B
wire        p256_sel = plt[0];
wire        p256_we_b = (io_wr && (io_addr == 16'hFD92) && p256_sel) || (ss_pal && ss_wr && ss_pph == 2'd2);
wire        p256_we_r = (io_wr && (io_addr == 16'hFD94) && p256_sel) || (ss_pal && ss_wr && ss_pph == 2'd0);
wire        p256_we_g = (io_wr && (io_addr == 16'hFD96) && p256_sel) || (ss_pal && ss_wr && ss_pph == 2'd1);
wire  [7:0] p256_wd   = ss_cs ? ss_din : io_din;
wire  [7:0] p256_val  = s1_sub[0][1] ? (s1_sub[0][0] ? lb_q[0][31:24] : lb_q[0][23:16]) : (s1_sub[0][0] ? lb_q[0][15:8] : lb_q[0][7:0]);

cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(8)) pal_r
(
	.clk_i(clk),
	.addr_a_i(pa_addr), .wren_a_i(p256_we_r), .wdata_a_i(p256_wd), .q_a_o(p256_cr),
	.addr_b_i(p256_ra), .wren_b_i(1'b0), .wdata_b_i(8'd0), .q_b_o(p256_r)
);
cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(8)) pal_g
(
	.clk_i(clk),
	.addr_a_i(pa_addr), .wren_a_i(p256_we_g), .wdata_a_i(p256_wd), .q_a_o(p256_cg),
	.addr_b_i(p256_ra), .wren_b_i(1'b0), .wdata_b_i(8'd0), .q_b_o(p256_g)
);
cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(8)) pal_b
(
	.clk_i(clk),
	.addr_a_i(pa_addr), .wren_a_i(p256_we_b), .wdata_a_i(p256_wd), .q_a_o(p256_cb),
	.addr_b_i(p256_ra), .wren_b_i(1'b0), .wdata_b_i(8'd0), .q_b_o(p256_b)
);

// ---- I/O ----
wire sel_crtc = (io_addr[15:4] == 12'h044) && (io_addr[3:0] <= 4'hC);
wire sel_pal  = (io_addr[15:4] == 12'hFD9) || (io_addr == 16'hFDA0);
wire sel_5ca  = (io_addr == 16'h05CA);
assign io_sel = sel_crtc | sel_pal | sel_5ca;

wire  [4:0] pal16_idx = {plt[1], pal_code[3:0]};

// The state port reads and writes cr and pal16 through the guest's own
// ports, with its index in place of ra / pal_code, so the arrays keep
// one read mux and one write port each.
wire  [4:0] cr_idx    = ss_cs ? ss_a[5:1] : ra;
wire  [4:0] pal_idx   = ss_cs ? ss_a[5:1] : pal16_idx;
wire [15:0] cr_cur    = cr[cr_idx];
wire [11:0] pal16_cur = pal16[pal_idx];
wire  [7:0] wd        = ss_cs ? ss_din : io_din;
wire        ss_cr     = ss_cs && ss_wr && ss_a < 8'h40;
wire        ss_pal16  = ss_cs && ss_wr && ss_a >= 8'h50 && ss_a < 8'h90;
wire        cr_we_lo  = ss_cs ? ss_cr && !ss_a[0] : io_wr && io_addr == 16'h0442;
wire        cr_we_hi  = ss_cs ? ss_cr &&  ss_a[0] : io_wr && io_addr == 16'h0443;
wire        pal_wr    = io_wr && !plt[0];
wire        pal_we_b  = ss_cs ? ss_pal16 && !ss_a[0] : pal_wr && io_addr == 16'hFD92;
wire        pal_we_g  = ss_cs ? ss_pal16 && !ss_a[0] : pal_wr && io_addr == 16'hFD96;
wire        pal_we_r  = ss_cs ? ss_pal16 &&  ss_a[0] : pal_wr && io_addr == 16'hFD94;
wire  [3:0] pal_wd_lo = ss_cs ? wd[3:0] : wd[7:4];   // the state bytes are {G, B} and {0, R}

// status read as the FR register
wire [15:0] fr_read = {(vcnt >= vds[1]) && (vcnt < vde[1]), (vcnt >= vds[0]) && (vcnt < vde[0]),
                       (hcnt >= hds[1]) && (hcnt < hde[1]), (hcnt >= hds[0]) && (hcnt < hde[0]),
                       field, vsync_w, hsync_w, 1'b0, 8'd0};

always @* begin
	io_dout = 8'hFF;
	case (io_addr)
		16'h0440: io_dout = 8'hFF;
		16'h0442: io_dout = (ra == R_FR[4:0]) ? fr_read[7:0] : cr_cur[7:0];
		16'h0443: io_dout = (ra == R_FR[4:0]) ? fr_read[15:8] : cr_cur[15:8];
		16'h0448: io_dout = {6'd0, sifter_ra};
		16'h044A: io_dout = sifter_ra[0] ? sifter1 : sifter0;
		16'h044C: io_dout = {dpmd, 5'd0, sp_busy, sp_page};
		16'hFD90: io_dout = pal_code;
		// 16-colour guns read back with the low nibble clear
		16'hFD92: io_dout = plt[0] ? p256_cb : {pal16_cur[3:0], 4'd0};
		16'hFD94: io_dout = plt[0] ? p256_cr : {pal16_cur[11:8], 4'd0};
		16'hFD96: io_dout = plt[0] ? p256_cg : {pal16_cur[7:4], 4'd0};
		16'hFDA0: io_dout = {6'd0, hsync_w, vsync_w};
		default: if (io_addr[15:3] == 13'h1FB3) io_dout = {4'd0, dpal[io_addr[2:0]]};   // FD98-FD9F
	endcase
end

always @* begin
	if (ss_a < 8'h40) ss_dout = ss_a[0] ? cr_cur[15:8] : cr_cur[7:0];
	else if (ss_a >= 8'h50 && ss_a < 8'h90) ss_dout = ss_a[0] ? {4'd0, pal16_cur[11:8]} : pal16_cur[7:0];
	else case (ss_a)
		8'h40: ss_dout = {3'd0, ra};
		8'h41: ss_dout = {sifter_ra, dpmd, vsync_irq, vsync_q, 3'd0};
		8'h42: ss_dout = sifter0;
		8'h43: ss_dout = sifter1;
		8'h44: ss_dout = {dpal[1], dpal[0]};
		8'h45: ss_dout = {dpal[3], dpal[2]};
		8'h46: ss_dout = {dpal[5], dpal[4]};
		8'h47: ss_dout = {dpal[7], dpal[6]};
		8'h48: ss_dout = pal_code;
		8'h49: ss_dout = {4'd0, fda0};
		8'h4A: ss_dout = hcnt[7:0];
		8'h4B: ss_dout = {field, 4'd0, hcnt[10:8]};
		8'h4C: ss_dout = vcnt[7:0];
		8'h4D: ss_dout = {5'd0, vcnt[10:8]};
		8'h90: ss_dout = ss_pph == 2'd0 ? p256_cr : ss_pph == 2'd1 ? p256_cg : p256_cb;
		default: ss_dout = 8'h00;
	endcase
end

// the palette stream steps a colour per engine cycle, an entry every three
always @(posedge clk) begin
	if (reset || (ss_cs && ss_a == 8'h93)) begin
		ss_pidx <= 8'd0;
		ss_pph  <= 2'd0;
	end
	else if (ss_pal && ss_step) begin
		if (ss_pph == 2'd2) begin ss_pph <= 2'd0; ss_pidx <= ss_pidx + 1'd1; end
		else ss_pph <= ss_pph + 1'd1;
	end
end

// VSYNC interrupt: latched at VSYNC start, released only by a write to 05CA
reg vsync_q;
integer i;
always @(posedge clk) begin
	if (reset) begin
		ra <= 0;
		sifter_ra <= 0;
		sifter0 <= 8'h15;
		sifter1 <= 8'h08;
		fda0 <= 4'hF;
		dpmd <= 0;
		pal_code <= 0;
		vsync_irq <= 0;
		vsync_q <= 0;
		for (i = 0; i < 32; i = i + 1) cr[i] <= 16'd0;
		for (i = 0; i < 8; i = i + 1) dpal[i] <= i[3:0];
		for (i = 0; i < 32; i = i + 1) pal16[i] <= {i[1] ? (i[3] ? 4'hF : 4'h8) : 4'h0, i[2] ? (i[3] ? 4'hF : 4'h8) : 4'h0, i[0] ? (i[3] ? 4'hF : 4'h8) : 4'h0};
		// power-on set: Databook set 6, 640x400 two layers, 16 colours (Tsugaru's reset)
		cr[R_HSW1] <= 16'h0040; cr[R_HSW2] <= 16'h0320; cr[R_HST] <= 16'h035F;
		cr[R_VST1] <= 16'h0000; cr[R_VST2] <= 16'h0010; cr[R_VST] <= 16'h036F;
		cr[R_HDS0] <= 16'h009C; cr[R_HDE0] <= 16'h031C; cr[R_HDS1] <= 16'h009C; cr[R_HDE1] <= 16'h031C;
		cr[R_VDS0] <= 16'h0040; cr[R_VDE0] <= 16'h0360; cr[R_VDS1] <= 16'h0040; cr[R_VDE1] <= 16'h0360;
		cr[R_HAJ0] <= 16'h009C; cr[R_LO0] <= 16'h0050; cr[R_HAJ1] <= 16'h009C; cr[R_LO1] <= 16'h0050;
		cr[25] <= 16'h004A; cr[26] <= 16'h0001; cr[R_CR0] <= 16'h803F; cr[R_CR1] <= 16'h0003; cr[31] <= 16'h0150;
	end
	else begin
		vsync_q <= vsync_w;
		if (vsync_w && !vsync_q) vsync_irq <= 1;
		if (io_wr && sel_5ca) vsync_irq <= 0;
		if (io_rd && io_addr == 16'h044C) dpmd <= 0;

		if (cr_we_lo) cr[cr_idx][7:0] <= wd;
		if (cr_we_hi) cr[cr_idx][15:8] <= wd;
		if (pal_we_b) pal16[pal_idx][3:0] <= pal_wd_lo;
		if (pal_we_g) pal16[pal_idx][7:4] <= wd[7:4];
		if (pal_we_r) pal16[pal_idx][11:8] <= pal_wd_lo;
		if (io_wr) begin
			case (io_addr)
				16'h0440: ra <= io_din[4:0];
				16'h0448: sifter_ra <= io_din[1:0];
				16'h044A: if (sifter_ra[0]) sifter1 <= io_din; else sifter0 <= io_din;
				16'hFD90: pal_code <= io_din;
				16'hFDA0: fda0 <= io_din[3:0];
				default: if (io_addr[15:3] == 13'h1FB3) begin
					dpal[io_addr[2:0]] <= io_din[3:0];
					dpmd <= 1;
				end
			endcase
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
				8'h40: ra <= ss_din[4:0];
				8'h41: {sifter_ra, dpmd, vsync_irq, vsync_q} <= ss_din[7:3];
				8'h42: sifter0 <= ss_din;
				8'h43: sifter1 <= ss_din;
				8'h44: {dpal[1], dpal[0]} <= ss_din;
				8'h45: {dpal[3], dpal[2]} <= ss_din;
				8'h46: {dpal[5], dpal[4]} <= ss_din;
				8'h47: {dpal[7], dpal[6]} <= ss_din;
				8'h48: pal_code <= ss_din;
				8'h49: fda0 <= ss_din[3:0];
				default: ;
			endcase
		end
	end
end

endmodule

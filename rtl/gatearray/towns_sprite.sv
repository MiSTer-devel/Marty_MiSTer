// Copyright (c) 2026 Jamie Blanks
//
// Towns sprite engine with its 128 KB pattern RAM. Once per frame, when
// SPEN is set, the engine clears the working half of VRAM layer 1 and
// transfers the last N index entries into it, one 16x16 pattern each:
//
//   index entry (8 bytes)      pattern RAM            VRAM layer 1
//   X, Y, attribute, colour -> 16-colour nibbles  --> colour table --> 256x256 half
//                              32k-colour words   ------------------> (double buffered)
//
// Pattern pixels are visited in storage order; the rotation code maps
// each to a destination offset and the shrink bits halve it, so a shrunk
// sprite lands twice on the same pixel and the later one stays. Pixels
// outside the drawn 256x254 region and transparent ones are not written.
// Every index entry in the range takes the same time, drawn or hidden by
// DISP and whatever it writes, as a real Towns times them; the walk skips
// unwritten pixels quickly so any entry fits that time. Busy rises a few
// microseconds after VSYNC does, then the clear and the start take their
// own fixed time before the first entry. The Marty's figures are the
// default; `fast` switches to a Towns II MX's.
//
// The clear is done by the VRAM controller (it replaces the half with
// the two clear lines); the engine only waits for it.

module towns_sprite #(
	parameter SPRITE_CLKS = 1968,   // 28.6 MHz clocks per index entry (68.7 us on a Marty)
	parameter CLEAR_CLKS  = 1730,   // clear and start, before the first entry (60 us)
	parameter START_CLKS  = 229,    // from VSYNC rising to busy (8 us)
	parameter MX_SPRITE_CLKS = 1632,   // Towns II MX: 57 us an entry
	parameter MX_CLEAR_CLKS  = 916,    // its clear (32 us)
	parameter MX_START_CLKS  = 0       // busy with VSYNC
)
(
	input             clk,
	input             ce,            // 28.636 MHz engine clock
	input             reset,

	// I/O 0450 (register select) and 0452 (data), byte lane
	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output      [7:0] io_dout,
	output            io_sel,

	// pattern RAM, CPU side: one word per cycle, data the next cycle
	input      [16:1] ram_a,
	input       [1:0] ram_be,
	input             ram_we,
	input      [15:0] ram_din,
	output     [15:0] ram_dout,

	input             vsync,         // CRTC VSYNC level
	input             fast,          // Towns II MX timing

	// VRAM word writes into layer 1; vw_ack once posted, vw_idle once done
	output reg        vw_req,
	output reg [18:1] vw_a,
	output reg [15:0] vw_din,
	input             vw_ack,
	input             vw_idle,

	// screen clear of one 256x256 half of layer 1
	output reg        clr_req,
	output reg        clr_page,
	input             clr_done,

	output reg        busy,          // SPD0
	output            active,        // a frame is on its way, busy or about to be
	output reg        page,          // half being drawn while busy
	output            disp_page,     // half the CRTC shows

	// savestate port: the registers and the page as bytes
	input             ss_cs,
	input             ss_wr,
	input       [2:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

// ---- registers ----
reg  [2:0] ra;
reg  [7:0] ind_lo;
reg        spen;
reg  [1:0] ind_hi;
reg  [8:0] ox, oy;
reg        dp1;

assign io_sel = (io_addr == 16'h0450) || (io_addr == 16'h0452);

reg [7:0] rd_data;
always @* begin
	rd_data = 8'hFF;
	if (io_addr == 16'h0450) rd_data = {5'd0, ra};
	else case (ra)
		3'd0: rd_data = ind_lo;
		3'd1: rd_data = {spen, 5'd0, ind_hi};
		3'd2: rd_data = ox[7:0];
		3'd3: rd_data = {7'd0, ox[8]};
		3'd4: rd_data = oy[7:0];
		3'd5: rd_data = {7'd0, oy[8]};
		3'd6: rd_data = {3'd0, dp1, 4'd0};
		default: rd_data = 8'h00;
	endcase
end
assign io_dout = rd_data;

always @* begin
	case (ss_a)
	3'd0: ss_dout = {5'd0, ra};
	3'd1: ss_dout = ind_lo;
	3'd2: ss_dout = {spen, 5'd0, ind_hi};
	3'd3: ss_dout = ox[7:0];
	3'd4: ss_dout = {6'd0, oy[8], ox[8]};
	3'd5: ss_dout = oy[7:0];
	3'd6: ss_dout = {dp1, 6'd0, page};
	default: ss_dout = 8'h00;
	endcase
end

always @(posedge clk) begin
	if (reset) begin
		ra <= 0; ind_lo <= 0; spen <= 0; ind_hi <= 0; ox <= 0; oy <= 0; dp1 <= 0;
	end
	else if (ss_cs && ss_wr) begin
		case (ss_a)
		3'd0: ra <= ss_din[2:0];
		3'd1: ind_lo <= ss_din;
		3'd2: {spen, ind_hi} <= {ss_din[7], ss_din[1:0]};
		3'd3: ox[7:0] <= ss_din;
		3'd4: {oy[8], ox[8]} <= ss_din[1:0];
		3'd5: oy[7:0] <= ss_din;
		3'd6: dp1 <= ss_din[7];
		default: ;
		endcase
	end
	else if (io_wr && io_sel) begin
		if (io_addr == 16'h0450) ra <= io_din[2:0];
		else case (ra)
			3'd0: ind_lo <= io_din;
			3'd1: {spen, ind_hi} <= {io_din[7], io_din[1:0]};
			3'd2: ox[7:0] <= io_din;
			3'd3: ox[8] <= io_din[0];
			3'd4: oy[7:0] <= io_din;
			3'd5: oy[8] <= io_din[0];
			3'd6: dp1 <= io_din[7];
			default: ;
		endcase
	end
end

// With the engine stopped DP1 picks the half shown; while it runs the
// half not being drawn is shown.
assign disp_page = spen ? ~page : dp1;

// ---- pattern RAM: two byte lanes, port A for the CPU, port B for the engine ----
wire [15:0] eng_a;
wire [15:0] eng_q;

cache_ram_dp #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) ram_lo
(
	.clk_i(clk),
	.addr_a_i(ram_a), .wren_a_i(ram_we & ram_be[0]), .wdata_a_i(ram_din[7:0]), .q_a_o(ram_dout[7:0]),
	.addr_b_i(eng_a), .wren_b_i(1'b0), .wdata_b_i(8'd0), .q_b_o(eng_q[7:0])
);

cache_ram_dp #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) ram_hi
(
	.clk_i(clk),
	.addr_a_i(ram_a), .wren_a_i(ram_we & ram_be[1]), .wdata_a_i(ram_din[15:8]), .q_a_o(ram_dout[15:8]),
	.addr_b_i(eng_a), .wren_b_i(1'b0), .wdata_b_i(8'd0), .q_b_o(eng_q[15:8])
);

// ---- frame sequencer ----
localparam [3:0] S_IDLE = 4'd0, S_CLEAR = 4'd1, S_IDX0 = 4'd2, S_IDX1 = 4'd3, S_IDX2 = 4'd4, S_IDX3 = 4'd5,
                 S_PAT = 4'd6, S_PAT1 = 4'd7, S_CT = 4'd8, S_PIX = 4'd9, S_WRITE = 4'd10, S_NEXT = 4'd11,
                 S_START = 4'd12, S_PAT_W = 4'd13, S_CT_W = 4'd14;

reg  [3:0] state;
reg        vsync_q;
reg  [9:0] idx;           // current index entry
reg [11:0] timer;         // clear time, 28.6 MHz clocks
reg        timer_done;
reg [11:0] spent;         // 28.6 MHz clocks since the entry started

// entry fields
reg  [8:0] sx0, sy0;
reg        offs;
reg  [2:0] rot;
reg        suy, sux;
reg  [9:0] pat;
reg        cten, spys;
reg [11:0] col;

// pattern walk
reg  [3:0] px, py;
reg  [3:0] nib;           // 16-colour pixel value
reg [15:0] pix;           // 32k pixel or colour table entry
reg        px_last;

// destination of the pixel in hand
reg  [3:0] dx, dy;
always @* begin
	case (rot)
		3'd0: {dx, dy} = {px, py};
		3'd1: {dx, dy} = {px, ~py};
		3'd2: {dx, dy} = {~px, py};
		3'd3: {dx, dy} = {~px, ~py};
		3'd4: {dx, dy} = {py, px};
		3'd5: {dx, dy} = {py, ~px};
		3'd6: {dx, dy} = {~py, px};
		default: {dx, dy} = {~py, ~px};
	endcase
	if (sux) dx = {1'b0, dx[3:1]};
	if (suy) dy = {1'b0, dy[3:1]};
end
wire [8:0] sx = sx0 + {5'd0, dx};
wire [8:0] sy = sy0 + {5'd0, dy};
wire       on_screen = !sx[8] && !sy[8] && (sy[7:1] != 0);

assign active = state != S_IDLE;

wire [11:0] entry_clks = fast ? MX_SPRITE_CLKS[11:0] : SPRITE_CLKS[11:0];
wire [11:0] clear_clks = fast ? MX_CLEAR_CLKS[11:0]  : CLEAR_CLKS[11:0];
wire [11:0] start_clks = fast ? MX_START_CLKS[11:0]  : START_CLKS[11:0];

// engine address into the pattern RAM (word units); a 16-colour row is
// four words, a 32k one sixteen
reg [15:0] eng_a_r;
assign eng_a = eng_a_r;
wire  [7:0] pyx_n     = {py, px} + 8'd1;
wire [15:0] next_a    = {pat[9:0], 6'd0} + (cten ? {10'd0, pyx_n[7:4], pyx_n[3:2]} : {8'd0, pyx_n});
wire [15:0] ct_word_a = {col[11:0], 4'd0} + {12'd0, nib};
wire  [3:0] nib_q     = eng_q[{px[1:0], 2'd0} +: 4];

always @(posedge clk) begin
	if (reset) begin
		state <= S_IDLE;
		busy <= 0;
		page <= 0;
		vsync_q <= 0;
		vw_req <= 0;
		clr_req <= 0;
		timer <= 0;
		timer_done <= 1;
		spent <= 0;
		idx <= 0;
		eng_a_r <= 0;
	end
	else if (ss_cs && ss_wr && ss_a == 3'd6) page <= ss_din[0];   // the engine is idle at a save
	else begin
		vsync_q <= vsync;
		if (vw_ack) vw_req <= 0;
		if (clr_done) clr_req <= 0;

		if (ce) begin
			if (timer != 0) timer <= timer - 1'd1;
			else timer_done <= 1;
			if (spent != 12'hFFF) spent <= spent + 1'd1;
		end

		case (state)
		// the count is taken at the edge: a game may rewrite it before busy
		S_IDLE: if (spen && vsync && !vsync_q) begin
			idx      <= {ind_hi, ind_lo};
			timer    <= start_clks;
			timer_done <= 0;
			state    <= S_START;
		end

		// swap halves, clear
		S_START: if (timer_done) begin
			busy     <= 1;
			page     <= ~page;
			clr_page <= ~page;
			clr_req  <= 1;
			timer    <= clear_clks;
			timer_done <= 0;
			state    <= S_CLEAR;
		end

		S_CLEAR: if (!clr_req && timer_done) begin
			eng_a_r <= {4'd0, idx, 2'd0};
			spent   <= 0;
			state   <= S_IDX0;
		end

		// four index words, one per cycle
		S_IDX0: begin eng_a_r <= {4'd0, idx, 2'd1}; state <= S_IDX1; end
		S_IDX1: begin sx0 <= eng_q[8:0]; eng_a_r <= {4'd0, idx, 2'd2}; state <= S_IDX2; end
		S_IDX2: begin sy0 <= eng_q[8:0]; eng_a_r <= {4'd0, idx, 2'd3}; state <= S_IDX3; end
		S_IDX3: begin
			{offs, rot, suy, sux, pat} <= eng_q;
			px <= 0;
			py <= 0;
			state <= S_PAT;
		end

		S_PAT: begin
			{cten, spys} <= eng_q[15:14];
			col <= eng_q[11:0];
			if (offs) begin
				sx0 <= sx0 + ox;
				sy0 <= sy0 + oy;
			end
			if (eng_q[13]) state <= S_NEXT;
			else begin
				eng_a_r <= {pat[9:0], 6'd0};
				state <= S_PAT_W;
			end
		end

		// the RAM answers one cycle after the address
		S_PAT_W: state <= S_PAT1;
		S_CT_W:  state <= S_PIX;

		// transparent: 16-colour value 0, 32k pattern bit 15 set; such a
		// pixel and one off the drawn region go straight to the next
		S_PAT1: begin
			if (cten) begin
				nib <= nib_q;
				state <= (nib_q != 0 && on_screen) ? S_CT : S_WRITE;
			end
			else begin
				pix <= eng_q;
				state <= (!eng_q[15] && on_screen) ? S_PIX : S_WRITE;
			end
		end

		S_CT: begin
			eng_a_r <= ct_word_a;
			state <= S_CT_W;
		end

		// a write is posted and the next pixel fetched while it completes;
		// the next write waits for the previous one to be taken
		S_PIX: if (!vw_req || vw_ack) begin
			vw_a    <= {1'b1, page, sy[7:0], sx[7:0]};
			vw_din  <= {spys, cten ? eng_q[14:0] : pix[14:0]};
			vw_req  <= 1;
			state   <= S_WRITE;
		end

		S_WRITE: begin
			if (px == 4'd15 && py == 4'd15) state <= S_NEXT;
			else begin
				{py, px} <= pyx_n;
				eng_a_r  <= next_a;
				state    <= S_PAT_W;
			end
		end

		S_NEXT: if (spent >= entry_clks && !vw_req) begin
			if (idx == 10'd1023) begin
				// the last pixel has to be in VRAM before the page shows
				if (vw_idle) begin
					busy  <= 0;
					state <= S_IDLE;
				end
			end
			else begin
				idx     <= idx + 1'd1;
				eng_a_r <= {4'd0, idx + 1'd1, 2'd0};
				spent   <= 0;
				state   <= S_IDX0;
			end
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule

// Copyright (c) 2026 Jamie Blanks
//
// The two TOWNS pad connectors (Technical Databook 3rd ed. §7.2-7.3) and
// the devices that can hang on them: a Marty pad, a 6-button pad, a
// TOWNS mouse fed from the framework's PS/2 mouse packets, or a CyberStick
// analog stick fed from the framework's analog axes.
//
//   04D0 / 04D2  R  {0, COM, TRIG2, TRIG1, RIGHT, LEFT, BACK, FWD}, 0 = pressed
//   04D6         W  {0, 0, JOY2 COM, JOY1 COM, JOY2 TRIG2, JOY2 TRIG1, JOY1 TRIG2, JOY1 TRIG1}
//
// Read bits 5:4 are the connector's pins 6/7 gated by the TRIG outputs and
// bit 6 is pin 8 gated by the COM output, the way the board wires them.
//
//   Marty pad   pins 1-4 directions, RUN pulls LEFT+RIGHT and SELECT pulls
//               UP+DOWN; ZOOM is a normally-closed switch shorting pin 8 to
//               GND, so with COM=1 bit 6 reads 1 only while it is pressed
//   Towns pad   the desktop Towns pad: the Marty pad without ZOOM, pin 8 open
//   6-button   a 74HC157 on pins 1-4: COM=0 directions, COM=1 Z Y X C
//   Capcom      the CPSF fighting pad: COM=1 puts R Y X SELECT on pins 1-4
//               and L START on pins 6-7 (Super Street Fighter II's pad
//               modes 1 and 2); RUN and SELECT are their own switches
//   mouse       four nibbles on pins 1-4, one per COM edge: X high, X low,
//               Y high, Y low; the deltas are latched on the first nibble
//               and right and down count negative. The mouse's own CPU puts
//               a nibble out 20 us after the first edge and 10 us after the
//               others; 150 us without an edge times the cycle out and drops
//               the report. Buttons on pins 6/7.
//               One PS/2 mouse feeds both ports; use it on one at a time.
//   analog      the CyberStick / XE-1AJ and the XE-1AP pad, one protocol.
//               A COM 1->0 edge starts a frame of twelve nibbles on pins
//               1-4; pin 7 (ACK) falls while a nibble is valid and pin 6
//               (HI/LO) alternates 0/1 per nibble. Idle is ACK 1, HI/LO 0,
//               pins 1-4 all 1. The stick times the frame itself:
//
//               COM   ‾‾‾‾‾‾\_______________________________________/‾‾‾‾
//               ACK   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______/‾‾‾‾‾‾\______/‾‾‾‾‾‾‾‾‾\____
//               HI/LO ______________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____________
//               D3-0  1111 <----- A -----><B: n0 ><- C -><D: n1 ><-- E --><n2>
//
//               A nibble stays on the pins until ACK falls for the next one,
//               so a game may read it either while ACK is low or, as the CRI
//               titles do, right after the HI/LO toggle while ACK is high.
//               Twelve nibbles: {E1 E2 start select}, {A B C D}, X hi, Y hi,
//               0, throttle hi, X lo, Y lo, 0, throttle lo, 1111, {A B A' B'}.
//               Buttons are 0 when pressed. Axes are 0 at up/left, 255 at
//               down/right.
//               The stick powers up slow; if COM is back high when the first,
//               second or third HI/LO=1 nibble becomes valid it switches to
//               one of three faster timings and stays there until reset.
//               Stick: A B are the triggers, C and D the body and throttle
//               buttons, E1 E2 the throttle rocker on the d-pad, A' B' the
//               body buttons on Y Z. Pad: E1 E2 are face buttons on Y Z.

module towns_padport #(
	parameter CLK_HZ = 57272727
)
(
	input             clk,
	input             ce,
	input             reset,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output reg        io_sel,
	// savestate port: the control register and each port's nibble cycle,
	// including the idle timers and the motion not yet latched
	input             ss_cs,
	input             ss_wr,
	input       [4:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,

	// per port: {Z, Y, X, C, zoom, select, run, B, A, up, down, left, right}, 1 = pressed
	input      [12:0] pad1,
	input      [12:0] pad2,
	input       [2:0] pad1_type,    // 0 Marty pad, 1 6-button pad, 2 mouse, 3 analog stick, 4 analog pad, 5 none, 6 Capcom 6-button, 7 Towns pad
	input       [2:0] pad2_type,
	input      [24:0] ps2_mouse,    // hps_io: [24] toggles per packet, [7:0] flags, [15:8] X, [23:16] Y
	input      [15:0] pad1_stick,   // hps_io analog: [7:0] X, [15:8] Y, signed, up and left negative
	input      [15:0] pad2_stick,
	input       [7:0] pad1_throttle, // signed, forward negative
	input       [7:0] pad2_throttle
);

wire sel_in  = (io_addr == 16'h04D0) || (io_addr == 16'h04D2);
wire sel_out = io_addr == 16'h04D6;

reg  [5:0] ctl;   // 04D6 bits 5:0
always @(posedge clk) begin
	if (reset) ctl <= 6'h00;
	else if (ss_cs && ss_wr && ss_a == 5'd0) ctl <= ss_din[5:0];
	else if (io_wr && ce && sel_out) ctl <= io_din[5:0];
end

// ---- mouse motion, accumulated until a nibble cycle takes it ----
reg         mouse_tog;
reg  signed [8:0] acc_x, acc_y;
wire signed [8:0] pkt_x = {ps2_mouse[4], ps2_mouse[15:8]};
wire signed [8:0] pkt_y = {ps2_mouse[5], ps2_mouse[23:16]};
wire        m_left  = ps2_mouse[0];
wire        m_right = ps2_mouse[1];

function signed [8:0] clamp8(input signed [9:0] v);
	if (v > 10'sd127) clamp8 = 9'sd127;
	else if (v < -10'sd128) clamp8 = -9'sd128;
	else clamp8 = v[8:0];
endfunction

// COM edges step the nibble; 150 us without an edge restarts the cycle
localparam [31:0] TIMEOUT_CLKS = CLK_HZ / 6667;
localparam [13:0] TIMEOUT = TIMEOUT_CLKS[13:0];
// The nibble reaches the pins TD after the edge. The Databook only bounds
// TD at 80 us for the first nibble and 40 us for the rest; shipped software
// reads sooner (Operation Wolf 26 us after the later edges, the 1990 Mouse
// BIOS 30 us), so the mouse answers in a quarter of those limits.
localparam [31:0] TD1_CLKS = CLK_HZ / 50000, TD2_CLKS = CLK_HZ / 100000;
reg  [1:0] phase [0:1];
reg [13:0] idle  [0:1];
reg [12:0] td    [0:1];
reg  [3:0] out_nib [0:1];
reg  [7:0] rep_x [0:1], rep_y [0:1];
reg        com_q [0:1];
wire [1:0] com = ctl[5:4];
wire [2:0] kind [0:1];
assign kind[0] = pad1_type;
assign kind[1] = pad2_type;
wire [1:0] has_mouse = {kind[1] == 3'd2, kind[0] == 3'd2};
wire [1:0] take;   // a mouse port starts a nibble cycle this clock

integer p;
always @(posedge clk) begin
	mouse_tog <= ps2_mouse[24];
	if (reset) begin
		acc_x <= 0; acc_y <= 0;
	end
	else if (ss_cs && ss_wr && ss_a[4:1] == 4'b0110) begin   // 0C/0D: motion not yet latched
		if (ss_a[0]) acc_y <= {ss_din[7], ss_din};
		else acc_x <= {ss_din[7], ss_din};
	end
	else if (take != 2'b00) begin
		acc_x <= (mouse_tog != ps2_mouse[24]) ? clamp8(10'sd0 - pkt_x) : 9'sd0;
		acc_y <= (mouse_tog != ps2_mouse[24]) ? clamp8(10'sd0 + pkt_y) : 9'sd0;
	end
	else if (mouse_tog != ps2_mouse[24]) begin
		acc_x <= clamp8(acc_x - pkt_x);   // right is negative on the Towns
		acc_y <= clamp8(acc_y + pkt_y);
	end
	for (p = 0; p < 2; p = p + 1) begin
		if (reset) begin
			phase[p] <= 2'd3; idle[p] <= 0; rep_x[p] <= 0; rep_y[p] <= 0; com_q[p] <= 0;
			td[p] <= 0; out_nib[p] <= 0;
		end
		else if (ss_cs && ss_wr && ss_a[4:2] == 3'b001 && ss_a[1] == p[0]) begin   // 04-07
			if (ss_a[0]) rep_y[p] <= ss_din;
			else rep_x[p] <= ss_din;
		end
		else if (ss_cs && ss_wr && ss_a[4:1] == (p[0] ? 4'b0101 : 4'b0100)) begin   // 08-0B: idle timer
			if (ss_a[0]) idle[p][13:8] <= ss_din[5:0];
			else idle[p][7:0] <= ss_din;
		end
		else if (ss_cs && ss_wr && ss_a == (p[0] ? 5'd2 : 5'd1)) {phase[p], com_q[p]} <= ss_din[2:0];
		else if (ss_cs && ss_wr && ss_a == 5'd14) out_nib[p] <= p[0] ? ss_din[7:4] : ss_din[3:0];
		else if (ss_cs && ss_wr && ss_a == (p[0] ? 5'd17 : 5'd15)) td[p][7:0]  <= ss_din;   // 0F-12: TD timers
		else if (ss_cs && ss_wr && ss_a == (p[0] ? 5'd18 : 5'd16)) td[p][12:8] <= ss_din[4:0];
		// restoring the control register must not read as a COM edge
		else if (ss_cs && ss_wr && ss_a == 5'd0) com_q[p] <= ss_din[4 + p[0]];
		else begin
			com_q[p] <= com[p];
			if (com_q[p] != com[p]) begin
				idle[p]  <= 0;
				phase[p] <= (idle[p] == TIMEOUT) ? 2'd0 : phase[p] + 1'd1;
				td[p]    <= (idle[p] == TIMEOUT || phase[p] == 2'd3) ? TD1_CLKS[12:0] : TD2_CLKS[12:0];
				if (take[p]) begin
					rep_x[p] <= acc_x[7:0];
					rep_y[p] <= acc_y[7:0];
				end
			end
			else if (idle[p] != TIMEOUT) begin
				idle[p] <= idle[p] + 1'd1;
				// a timed-out cycle drops what was being reported
				if (idle[p] == TIMEOUT - 1'd1) begin rep_x[p] <= 0; rep_y[p] <= 0; end
			end
			if (td[p] != 0) begin
				td[p] <= td[p] - 1'd1;
				if (td[p] == 13'd1) out_nib[p] <= nib_of[p];
			end
		end
	end
end

always @* begin
	case (ss_a)
	5'd0:  ss_dout = {2'd0, ctl};
	5'd1:  ss_dout = {5'd0, phase[0], com_q[0]};
	5'd2:  ss_dout = {5'd0, phase[1], com_q[1]};
	5'd4:  ss_dout = rep_x[0];
	5'd5:  ss_dout = rep_y[0];
	5'd6:  ss_dout = rep_x[1];
	5'd7:  ss_dout = rep_y[1];
	5'd8:  ss_dout = idle[0][7:0];
	5'd9:  ss_dout = {2'd0, idle[0][13:8]};
	5'd10: ss_dout = idle[1][7:0];
	5'd11: ss_dout = {2'd0, idle[1][13:8]};
	5'd12: ss_dout = acc_x[7:0];
	5'd13: ss_dout = acc_y[7:0];
	5'd14: ss_dout = {out_nib[1], out_nib[0]};
	5'd15: ss_dout = td[0][7:0];
	5'd16: ss_dout = {3'd0, td[0][12:8]};
	5'd17: ss_dout = td[1][7:0];
	5'd18: ss_dout = {3'd0, td[1][12:8]};
	default: ss_dout = 8'h00;
	endcase
end

genvar g;
generate for (g = 0; g < 2; g = g + 1) begin : port
	// COM edges on a port without the mouse must not touch the motion: Towns OS
	// polls the pad on the other port between mouse reads
	assign take[g] = has_mouse[g] && (com_q[g] != com[g]) && (idle[g] == TIMEOUT || phase[g] == 2'd3);
end endgenerate

// ---- CyberStick frame timing ----
// Phase lengths in clocks from the measured protocol, in 0.01 us units, for
// the four speeds the stick can settle on (0 slowest, 3 fastest)
function [13:0] cyc(input integer hundredths);
	integer v;
	begin
		v = (CLK_HZ / 10000 * hundredths) / 10000;
		cyc = v[13:0];
	end
endfunction

localparam [3:0] CS_A = 4'd0,    // COM fell, first nibble being prepared
                 CS_B = 4'd1,    // even nibble valid, HI/LO 0
                 CS_C1 = 4'd2,   // ACK rises
                 CS_C2 = 4'd3,   // HI/LO rises, odd nibble on the pins
                 CS_D1 = 4'd4,   // odd nibble valid; COM is sampled at the end
                 CS_D2 = 4'd5,
                 CS_E1 = 4'd6,   // ACK rises
                 CS_E2 = 4'd7,   // HI/LO falls
                 CS_E3 = 4'd8;   // next even nibble on the pins

function [13:0] cs_len(input [3:0] sub, input [1:0] speed);
	case (sub)
	CS_A:  case (speed) 2'd0: cs_len = cyc(8600); 2'd1: cs_len = cyc(7800); 2'd2: cs_len = cyc(7700); default: cs_len = cyc(7100); endcase
	CS_B:  case (speed) 2'd0: cs_len = cyc(7412); 2'd1: cs_len = cyc(5000); 2'd2: cs_len = cyc(2610); default: cs_len = cyc(1210); endcase
	CS_C1: cs_len = (speed == 2'd3) ? 14'd0 : cyc(400);
	CS_C2: cs_len = cyc(800);    // measured 3.88 us; longer so a read after the HI/LO rise still sees the old nibble
	CS_D1: cs_len = cyc(400);
	CS_D2: case (speed) 2'd0: cs_len = cyc(8400); 2'd1: cs_len = cyc(6000); 2'd2: cs_len = cyc(3610); default: cs_len = cyc(812); endcase
	CS_E1: cs_len = cyc(387);
	CS_E2: cs_len = cyc(1200);
	default: cs_len = cyc(600);
	endcase
endfunction

// per port: the running frame and the values latched when it started
reg        cs_run  [0:1];
reg  [2:0] cs_pair [0:1];   // nibble pair 0-5
reg  [3:0] cs_sub  [0:1];
reg [13:0] cs_t    [0:1];
reg  [1:0] cs_speed [0:1];
reg  [7:0] cs_x [0:1], cs_y [0:1], cs_th [0:1];
reg  [3:0] cs_b0 [0:1], cs_b1 [0:1], cs_b10 [0:1];
reg [15:0] stick [0:1];
reg  [7:0] throttle [0:1];
reg [12:0] btn [0:1];
reg  [3:0] cs_k [0:1];      // nibble on the pins, 0-11
reg  [3:0] cs_nib [0:1];
reg        cs_ack [0:1], cs_hl [0:1];
wire [1:0] has_cs = {kind[1] == 3'd3 || kind[1] == 3'd4, kind[0] == 3'd3 || kind[0] == 3'd4};

always @* begin
	stick[0] = pad1_stick; stick[1] = pad2_stick;
	throttle[0] = pad1_throttle; throttle[1] = pad2_throttle;
	btn[0] = pad1; btn[1] = pad2;
end

integer q;
always @(posedge clk) begin
	for (q = 0; q < 2; q = q + 1) begin
		if (reset) begin
			cs_run[q] <= 0; cs_speed[q] <= 0; cs_pair[q] <= 0; cs_sub[q] <= 0; cs_t[q] <= 0;
		end
		// a restored state starts the stick idle; the game polls it again
		else if (ss_cs && ss_wr && ss_a == 5'd0) cs_run[q] <= 0;
		// every COM 1->0 edge starts a frame, cutting short one still running:
		// a digital read that drops COM would otherwise leave the next analog
		// read syncing into the middle of the old frame
		else if (has_cs[q] && com_q[q] && !com[q]) begin
			cs_run[q]  <= 1;
			cs_pair[q] <= 0;
			cs_sub[q]  <= CS_A;
			cs_t[q]    <= cs_len(CS_A, cs_speed[q]);
			cs_x[q]    <= {~stick[q][7], stick[q][6:0]};
			cs_y[q]    <= {~stick[q][15], stick[q][14:8]};
			cs_th[q]   <= {~throttle[q][7], throttle[q][6:0]};
			// nibble 0 {A|A', B|B', C, D}, nibble 1 {E1, E2, start, select}, nibble 10 {A, B, A', B'}
			if (kind[q] == 3'd4) begin   // pad: E1 E2 on Y Z
				cs_b0[q]  <= ~{btn[q][4], btn[q][5], btn[q][9], btn[q][10]};
				cs_b1[q]  <= ~{btn[q][11], btn[q][12], btn[q][6], btn[q][7]};
				cs_b10[q] <= ~{btn[q][4], btn[q][5], 1'b0, 1'b0};
			end
			else begin                   // stick: rocker on down/up, A' B' on Y Z
				cs_b0[q]  <= ~{btn[q][4] | btn[q][11], btn[q][5] | btn[q][12], btn[q][9], btn[q][10]};
				cs_b1[q]  <= ~{btn[q][2], btn[q][3], btn[q][6], btn[q][7]};
				cs_b10[q] <= ~{btn[q][4], btn[q][5], btn[q][11], btn[q][12]};
			end
		end
		else if (!cs_run[q]) ;
		else if (cs_t[q] != 0) cs_t[q] <= cs_t[q] - 1'd1;
		else begin
			// the stick speeds up when COM is already high at the first
			// HI/LO=1 samples of a frame, and never slows down again
			if (cs_sub[q] == CS_D1 && com[q] && cs_pair[q] < 3'd3 && cs_speed[q] < 2'd3 - cs_pair[q][1:0])
				cs_speed[q] <= 2'd3 - cs_pair[q][1:0];
			if (cs_sub[q] == CS_E2 && cs_pair[q] == 3'd5) cs_run[q] <= 0;
			else if (cs_sub[q] == CS_E3) begin
				cs_sub[q]  <= CS_B;
				cs_t[q]    <= cs_len(CS_B, cs_speed[q]);
				cs_pair[q] <= cs_pair[q] + 1'd1;
			end
			else begin
				cs_sub[q] <= cs_sub[q] + 1'd1;
				cs_t[q]   <= cs_len(cs_sub[q] + 1'd1, cs_speed[q]);
			end
		end
	end
end

// the pins during each phase; the odd nibble moves in at D1, the even at E3
always @* begin
	for (q = 0; q < 2; q = q + 1) begin
		cs_ack[q] = !(cs_run[q] && (cs_sub[q] == CS_B || cs_sub[q] == CS_D1 || cs_sub[q] == CS_D2));
		cs_hl[q]  = cs_run[q] && (cs_sub[q] == CS_C2 || cs_sub[q] == CS_D1 || cs_sub[q] == CS_D2 || cs_sub[q] == CS_E1);
		if (!cs_run[q] || cs_sub[q] == CS_A) cs_k[q] = 4'd10;   // idle reads 1111
		else if (cs_sub[q] == CS_B || cs_sub[q] == CS_C1 || cs_sub[q] == CS_C2) cs_k[q] = {cs_pair[q], 1'b0};
		else if (cs_sub[q] == CS_E3) cs_k[q] = {cs_pair[q], 1'b0} + 4'd2;
		else cs_k[q] = {cs_pair[q], 1'b1};
		// each pair goes out second nibble first, the order the CRI titles
		// decode: {E1 E2 start select}, {A B C D}, X hi, Y hi, 0, throttle hi,
		// X lo, Y lo, 0, throttle lo, 1111, {A B A' B'}
		case (cs_k[q])
		4'd0:  cs_nib[q] = cs_b1[q];
		4'd1:  cs_nib[q] = cs_b0[q];
		4'd2:  cs_nib[q] = cs_x[q][7:4];
		4'd3:  cs_nib[q] = cs_y[q][7:4];
		4'd5:  cs_nib[q] = cs_th[q][7:4];
		4'd6:  cs_nib[q] = cs_x[q][3:0];
		4'd7:  cs_nib[q] = cs_y[q][3:0];
		4'd9:  cs_nib[q] = cs_th[q][3:0];
		4'd10: cs_nib[q] = 4'hF;
		4'd11: cs_nib[q] = cs_b10[q];
		default: cs_nib[q] = 4'h0;
		endcase
	end
end

// ---- the pins of the device on each port, 1 = released ----
// pins[6:0] = {pin 8, pin 7, pin 6, pin 4, pin 3, pin 2, pin 1}
reg  [6:0] pins [0:1];
reg [12:0] pad  [0:1];
reg  [3:0] nib_of [0:1];   // the nibble the mouse is putting out for the current phase
reg        up, down, left, right;
always @* begin
	pad[0] = pad1; pad[1] = pad2;
	for (p = 0; p < 2; p = p + 1)
		case (phase[p])
		2'd0: nib_of[p] = rep_x[p][7:4];
		2'd1: nib_of[p] = rep_x[p][3:0];
		2'd2: nib_of[p] = rep_y[p][7:4];
		default: nib_of[p] = rep_y[p][3:0];
		endcase
	for (p = 0; p < 2; p = p + 1) begin
		up    = ~(pad[p][3] | pad[p][7]);
		down  = ~(pad[p][2] | pad[p][7]);
		left  = ~(pad[p][1] | pad[p][6]);
		right = ~(pad[p][0] | pad[p][6]);
		case (kind[p])
		3'd1: pins[p] = {1'b1, ~pad[p][5], ~pad[p][4], com[p] ? {~pad[p][9], ~pad[p][10], ~pad[p][11], ~pad[p][12]} : {right, left, down, up}};
		3'd2: pins[p] = {1'b1, ~m_right, ~m_left, out_nib[p]};
		3'd3, 3'd4: pins[p] = {1'b1, cs_ack[p], cs_hl[p], cs_nib[p]};
		3'd5: pins[p] = 7'h7F;   // nothing plugged in: every pin floats to its pull-up
		3'd6: pins[p] = com[p] ? {1'b1, ~pad[p][6], ~pad[p][12], ~pad[p][7], ~pad[p][10], ~pad[p][11], ~pad[p][9]}
		                       : {1'b1, ~pad[p][5], ~pad[p][4], ~pad[p][0], ~pad[p][1], ~pad[p][2], ~pad[p][3]};
		3'd7: pins[p] = {1'b1, ~pad[p][5], ~pad[p][4], right, left, down, up};
		default: pins[p] = {pad[p][8], ~pad[p][5], ~pad[p][4], right, left, down, up};
		endcase
	end
end

always @* begin
	io_sel  = sel_in | sel_out;
	io_dout = 8'hFF;
	if (io_addr == 16'h04D0) io_dout = {1'b0, pins[0][6] & ctl[4], pins[0][5] & ctl[1], pins[0][4] & ctl[0], pins[0][3:0]};
	if (io_addr == 16'h04D2) io_dout = {1'b0, pins[1][6] & ctl[5], pins[1][5] & ctl[3], pins[1][4] & ctl[2], pins[1][3:0]};
end

endmodule

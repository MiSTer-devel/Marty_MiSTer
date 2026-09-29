// Copyright (c) 2026 Jamie Blanks
//
// Keyboard controller interface (Technical Databook 3rd ed. §7.1, an 8042
// behind three registers) with a JIS keyboard behind it, fed from the
// framework's PS/2 key events.
//
//   0600 R keyboard data (pops the output buffer)   W 8042 data: A1/A2 act as reset
//   0602 R {ST7-4, F1, F0, IBF, OBF}                W command: A0 hard reset, A1 soft reset,
//                                                     A9-AB repeat delay, AC-AE repeat rate
//   0604 R {.., NMI, KBINT}                         W KBMSK (bit 0)
//
// Reset replies as observed on an MX with a JIS keyboard: A0 or A1 ->
// B0 7F E8 25; A1 after A1 -> B0 7F. After B0 7F E8 25 the keyboard sends
// A0 and the key code for each key held, lowest code first, the form the
// SYSTEM ROM's boot key scan reads (a real keyboard's reply with keys held
// is unmeasured). Held keys come from a map of the host keyboard that a core
// reset keeps.
// A key event is two bytes: 80 | 20 (JIS) | 10 on release | 08 CTRL held |
// 04 SHIFT held, then the JIS key code. A held key repeats after the
// typematic delay (400 ms) at the typematic rate (30 ms) as F0 | 08 CTRL |
// 04 SHIFT, then the key code. A reset order restores the defaults.
// The keyboard queues its bytes and sends them down a 9600 bps line; the
// 8042 holds one byte in its output buffer (OBF) until the CPU takes it,
// and a host write sits in its input buffer (IBF) until the firmware
// takes it; an order written while IBF is set is ignored.
// KBINT is the output buffer holding data while the mask allows it.

module towns_keyboard #(
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
	output            irq,
	// savestate port: 00-0F the buffer, then the registers
	input             ss_cs,
	input             ss_wr,
	input       [4:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,

	input      [10:0] ps2_key,       // hps_io: [10] toggles per event, [9] pressed, [8] extended, [7:0] code
	input      [10:0] ps2_key_raw    // the framework's stream before the on-screen keyboard joins it
);

wire sel = (io_addr[15:3] == 13'h00C0) && !io_addr[0];   // 0600-0606
wire rd  = io_rd & ce & sel;
wire wr  = io_wr & ce & sel;

// ---- keyboard-side queue, 16 bytes ----
reg  [7:0] fifo [0:15];
reg  [3:0] head, tail;
wire [3:0] cnt = tail - head;
// the serial line: one byte every ten bit times
localparam [31:0] BYTE_CLKS = CLK_HZ / 960;
reg [16:0] ser_cnt;
reg        ser_busy;
// the 8042's buffers; the firmware takes a host byte after its poll loop
localparam [31:0] TAKE_CLKS = CLK_HZ / 25000;   // 40 us, unmeasured
reg  [7:0] obuf, ibuf;
reg        obf, ibf, ibuf_cmd;
reg [11:0] ibf_cnt;
wire       take = ibf && ibf_cnt == 0;
reg        kbmsk;
reg  [7:0] last_cmd;
assign irq = obf & kbmsk;

// pushes requested this clock: reset replies (up to four bytes) or a key pair
reg  [2:0] push_n;
reg  [7:0] push [0:3];
reg        flush;

// The state port shares the FIFO's first push lane and its read mux.
wire [3:0] fifo_wa0 = ss_cs ? ss_a[3:0] : flush ? 4'd0 : tail;
wire [7:0] fifo_wd0 = ss_cs ? ss_din : push[0];
wire       fifo_we0 = ss_cs ? ss_wr && !ss_a[4] : push_n != 0;
wire [7:0] fifo_rd  = fifo[ss_cs ? ss_a[3:0] : head];

always @* begin
	io_sel  = sel;
	case (io_addr[2:1])
	2'd0: io_dout = obf ? obuf : 8'h00;
	2'd1: io_dout = {6'd0, ibf, obf};
	2'd2: io_dout = {7'd0, irq};
	default: io_dout = 8'hFF;
	endcase
end

// ---- PS/2 events to JIS codes ----
// a key event waits while a reset reply is being pushed this clock
reg        key_tog;
wire       cmd_push = take && (ibuf == 8'hA0 || ibuf == 8'hA1 || ibuf == 8'hA2);
wire       key_ev   = key_tog != ps2_key[10] && !cmd_push;
wire [7:0] jis;
towns_keymap keymap(.ext(ps2_key[8]), .code(ps2_key[7:0]), .jis(jis));

reg        shift, ctrl;
reg  [7:0] rep_code;             // key being held for typematic
reg  [1:0] rep_delay_sel, rep_rate_sel;
reg [24:0] rep_cnt;
localparam [31:0] MS_CLKS = CLK_HZ / 1000;
localparam [24:0] MS = MS_CLKS[24:0];
wire [24:0] rep_delay = rep_delay_sel == 2'd2 ? 25'd500 * MS : rep_delay_sel == 2'd3 ? 25'd300 * MS : 25'd400 * MS;
wire [24:0] rep_rate  = rep_rate_sel  == 2'd0 ? 25'd50 * MS  : rep_rate_sel  == 2'd2 ? 25'd20 * MS  : 25'd30 * MS;
// the flags show the modifier state after the event, so a shift release
// carries no SHIFT
wire       shift_now = (jis == 8'h53) ? ps2_key[9] : shift;
wire       ctrl_now  = (jis == 8'h52) ? ps2_key[9] : ctrl;
wire [7:0] flags     = {1'b1, 2'b01, 1'b0, ctrl_now, shift_now, 2'b00};   // JIS, make
wire [7:0] rep_flags = {4'b1111, ctrl, shift, 2'b00};

wire       room = cnt < 4'd14;   // space for a key pair

// ---- keys held on the host keyboard, kept through a core reset ----
// Taken from the framework's own stream: the on-screen keyboard lets go of
// its keys at a reset without sending the breaks.
reg  [127:0] held = 128'd0;
reg          held_tog = 1'b0;
wire   [7:0] held_jis;
towns_keymap held_keymap
(
	.ext  (ps2_key_raw[8]),
	.code (ps2_key_raw[7:0]),
	.jis  (held_jis)
);

always @(posedge clk) begin
	held_tog <= ps2_key_raw[10];
	if (held_tog != ps2_key_raw[10] && held_jis != 8'h00) held[held_jis[6:0]] <= ps2_key_raw[9];
end

// After a reset reply, the held keys go out one code per clock, in the clocks
// nothing else pushes and a clock after the last push, so room is up to date.
reg        rpt_on;
reg  [6:0] rpt_idx;
wire       key_push = key_ev && jis != 8'h00 && room;
wire       rep_push = rep_code != 8'h00 && !cmd_push && rep_cnt == 0 && room;
wire       rpt_step = rpt_on && !ss_cs && !cmd_push && !key_push && !rep_push && push_n == 3'd0 && (!held[rpt_idx] || room);

integer i;
always @(posedge clk) begin
	if (key_ev) key_tog <= ps2_key[10];
	push_n <= 0;
	flush <= 0;
	if (reset) begin
		head <= 0; tail <= 0;
		obf <= 0; obuf <= 8'h00; ibf <= 0; ibuf <= 8'h00; ibuf_cmd <= 0; ibf_cnt <= 0;
		ser_busy <= 0; ser_cnt <= 0;
		last_cmd <= 8'h00; kbmsk <= 0;
		shift <= 0; ctrl <= 0;
		rep_code <= 8'h00; rep_cnt <= 0;
		rep_delay_sel <= 2'd1; rep_rate_sel <= 2'd1;   // A9 400 ms, AD 30 ms
		rpt_on <= 0; rpt_idx <= 7'd0;
	end
	else begin
		// a host byte waits in the input buffer until the firmware takes it
		if (wr && io_addr[2:1] == 2'd2) kbmsk <= io_din[0];
		else if (wr && io_addr[2:1] != 2'd3 && !ibf) begin
			ibf <= 1; ibuf <= io_din; ibuf_cmd <= io_addr[1]; ibf_cnt <= TAKE_CLKS[11:0];
		end
		if (ibf && ibf_cnt != 0) ibf_cnt <= ibf_cnt - 1'd1;
		if (take) begin
			ibf <= 0;
			if (!ibuf_cmd) begin
				if (ibuf == 8'hA1 || ibuf == 8'hA2) begin
					flush <= 1; push_n <= 3'd4;
					push[0] <= 8'hB0; push[1] <= 8'h7F; push[2] <= 8'hE8; push[3] <= 8'h25;
					rpt_on <= 0;
				end
			end
			else begin
				if (ibuf == 8'hA1 && last_cmd == 8'hA1) begin
					flush <= 1; push_n <= 3'd2; push[0] <= 8'hB0; push[1] <= 8'h7F;
					rpt_on <= 0;
				end
				else if (ibuf == 8'hA0 || ibuf == 8'hA1) begin
					flush <= 1; push_n <= 3'd4;
					push[0] <= 8'hB0; push[1] <= 8'h7F; push[2] <= 8'hE8; push[3] <= 8'h25;
					rep_delay_sel <= 2'd1; rep_rate_sel <= 2'd1;
					rpt_on <= 1; rpt_idx <= 7'd0;
				end
				else if (ibuf[7:2] == 6'b101010 && ibuf[1:0] != 2'd0) rep_delay_sel <= ibuf[1:0];   // A9 400, AA 500, AB 300 ms
				else if (ibuf[7:2] == 6'b101011 && ibuf[1:0] != 2'd3) rep_rate_sel  <= ibuf[1:0];   // AC 50, AD 30, AE 20 ms
				last_cmd <= ibuf;
			end
		end

		// key events; a full buffer drops the event, the way a slow host loses keys
		if (key_ev && jis != 8'h00 && room) begin
			push_n  <= 3'd2;
			push[0] <= flags | (ps2_key[9] ? 8'h00 : 8'h10);
			push[1] <= jis;
			if (jis == 8'h53) shift <= ps2_key[9];
			if (jis == 8'h52) ctrl  <= ps2_key[9];
			if (ps2_key[9]) begin rep_code <= jis; rep_cnt <= rep_delay; end
			else if (jis == rep_code) rep_code <= 8'h00;
		end
		else if (rep_code != 8'h00 && !cmd_push) begin
			if (rep_cnt != 0) rep_cnt <= rep_cnt - 1'd1;
			else if (room) begin
				push_n <= 3'd2; push[0] <= rep_flags; push[1] <= rep_code;
				rep_cnt <= rep_rate;
			end
		end

		// a held key goes out as its make: A0, then the code
		if (rpt_step) begin
			if (held[rpt_idx]) begin
				push_n <= 3'd2; push[0] <= 8'hA0; push[1] <= {1'b0, rpt_idx};
			end
			rpt_idx <= rpt_idx + 1'd1;
			if (rpt_idx == 7'd127) rpt_on <= 0;
		end

		// the line carries the head of the queue to the output buffer, one
		// byte time each, and waits while the CPU has not taken the last
		if (!ser_busy && cnt != 0) begin ser_busy <= 1; ser_cnt <= BYTE_CLKS[16:0]; end
		else if (ser_busy && ser_cnt != 0) ser_cnt <= ser_cnt - 1'd1;
		else if (ser_busy && !obf) begin obuf <= fifo_rd; obf <= 1; head <= head + 1'd1; ser_busy <= 0; end
		if (rd && io_addr[2:1] == 2'd0 && obf) obf <= 0;
		if (flush) begin head <= 0; tail <= 0; ser_busy <= 0; obf <= 0; end
		// the state port loads the FIFO through the first push lane
		if (fifo_we0) fifo[fifo_wa0] <= fifo_wd0;
		if (push_n != 0) begin
			for (i = 1; i < 4; i = i + 1)
				if (i < push_n) fifo[(flush ? 4'd0 : tail) + i[3:0]] <= push[i];
			tail <= (flush ? 4'd0 : tail) + {1'b0, push_n};
		end
		if (ss_cs && ss_wr && ss_a[4]) begin
			case (ss_a[3:0])
			4'd0: {head, tail} <= ss_din;
			4'd1: {rpt_on, kbmsk, shift, ctrl, rep_delay_sel, rep_rate_sel} <= ss_din;
			4'd2: last_cmd <= ss_din;
			4'd3: rep_code <= ss_din;
			4'd4: rep_cnt[7:0] <= ss_din;
			4'd5: rep_cnt[15:8] <= ss_din;
			4'd6: rep_cnt[23:16] <= ss_din;
			4'd7: {rpt_idx, rep_cnt[24]} <= ss_din;
			4'd8: obuf <= ss_din;
			4'd9: ibuf <= ss_din;
			4'd10: {obf, ibf, ibuf_cmd, ser_busy} <= ss_din[3:0];
			4'd11: ser_cnt[7:0] <= ss_din;
			4'd12: ser_cnt[15:8] <= ss_din;
			4'd13: ser_cnt[16] <= ss_din[0];
			4'd14: ibf_cnt[7:0] <= ss_din;
			4'd15: ibf_cnt[11:8] <= ss_din[3:0];
			endcase
		end
	end
end

always @* begin
	if (!ss_a[4]) ss_dout = fifo_rd;
	else case (ss_a[3:0])
	4'd0: ss_dout = {head, tail};
	4'd1: ss_dout = {rpt_on, kbmsk, shift, ctrl, rep_delay_sel, rep_rate_sel};
	4'd2: ss_dout = last_cmd;
	4'd3: ss_dout = rep_code;
	4'd4: ss_dout = rep_cnt[7:0];
	4'd5: ss_dout = rep_cnt[15:8];
	4'd6: ss_dout = rep_cnt[23:16];
	4'd7: ss_dout = {rpt_idx, rep_cnt[24]};
	4'd8: ss_dout = obuf;
	4'd9: ss_dout = ibuf;
	4'd10: ss_dout = {4'd0, obf, ibf, ibuf_cmd, ser_busy};
	4'd11: ss_dout = ser_cnt[7:0];
	4'd12: ss_dout = ser_cnt[15:8];
	4'd13: ss_dout = {7'd0, ser_cnt[16]};
	4'd14: ss_dout = ibf_cnt[7:0];
	4'd15: ss_dout = {4'd0, ibf_cnt[11:8]};
	endcase
end

endmodule

// PS/2 set 2 scan code to JIS keyboard code (figure 1-7-6 numbering).
// Keys the US layout lacks sit on Right Alt (kana/kanji), Right Ctrl
// (convert), Menu (no convert), F11 (PF11) and F12 (PF12). 0 = no key.
module towns_keymap
(
	input             ext,
	input       [7:0] code,
	output reg  [7:0] jis
);

always @* begin
	jis = 8'h00;
	if (!ext) case (code)
	8'h76: jis = 8'h01;   // ESC
	8'h16: jis = 8'h02;   // 1
	8'h1E: jis = 8'h03;   // 2
	8'h26: jis = 8'h04;   // 3
	8'h25: jis = 8'h05;   // 4
	8'h2E: jis = 8'h06;   // 5
	8'h36: jis = 8'h07;   // 6
	8'h3D: jis = 8'h08;   // 7
	8'h3E: jis = 8'h09;   // 8
	8'h46: jis = 8'h0A;   // 9
	8'h45: jis = 8'h0B;   // 0
	8'h4E: jis = 8'h0C;   // -
	8'h55: jis = 8'h0D;   // = on US, ^ on JIS
	8'h6A: jis = 8'h0E;   // yen (JIS)
	8'h66: jis = 8'h0F;   // backspace
	8'h0D: jis = 8'h10;   // tab
	8'h15: jis = 8'h11;   // Q
	8'h1D: jis = 8'h12;   // W
	8'h24: jis = 8'h13;   // E
	8'h2D: jis = 8'h14;   // R
	8'h2C: jis = 8'h15;   // T
	8'h35: jis = 8'h16;   // Y
	8'h3C: jis = 8'h17;   // U
	8'h43: jis = 8'h18;   // I
	8'h44: jis = 8'h19;   // O
	8'h4D: jis = 8'h1A;   // P
	8'h54: jis = 8'h1B;   // [ on US, @ on JIS
	8'h5B: jis = 8'h1C;   // ] on US, [ on JIS
	8'h5A: jis = 8'h1D;   // return
	8'h14: jis = 8'h52;   // left ctrl
	8'h1C: jis = 8'h1E;   // A
	8'h1B: jis = 8'h1F;   // S
	8'h23: jis = 8'h20;   // D
	8'h2B: jis = 8'h21;   // F
	8'h34: jis = 8'h22;   // G
	8'h33: jis = 8'h23;   // H
	8'h3B: jis = 8'h24;   // J
	8'h42: jis = 8'h25;   // K
	8'h4B: jis = 8'h26;   // L
	8'h4C: jis = 8'h27;   // ;
	8'h52: jis = 8'h28;   // ' on US, : on JIS
	8'h5D: jis = 8'h29;   // \ on US, ] on JIS
	8'h12: jis = 8'h53;   // left shift
	8'h59: jis = 8'h53;   // right shift
	8'h1A: jis = 8'h2A;   // Z
	8'h22: jis = 8'h2B;   // X
	8'h21: jis = 8'h2C;   // C
	8'h2A: jis = 8'h2D;   // V
	8'h32: jis = 8'h2E;   // B
	8'h31: jis = 8'h2F;   // N
	8'h3A: jis = 8'h30;   // M
	8'h41: jis = 8'h31;   // ,
	8'h49: jis = 8'h32;   // .
	8'h4A: jis = 8'h33;   // /
	8'h51: jis = 8'h34;   // ro (JIS)
	8'h29: jis = 8'h35;   // space
	8'h7C: jis = 8'h36;   // keypad *
	8'h79: jis = 8'h38;   // keypad +
	8'h7B: jis = 8'h39;   // keypad -
	8'h6C: jis = 8'h3A;   // keypad 7
	8'h75: jis = 8'h3B;   // keypad 8
	8'h7D: jis = 8'h3C;   // keypad 9
	8'h6B: jis = 8'h3E;   // keypad 4
	8'h73: jis = 8'h3F;   // keypad 5
	8'h74: jis = 8'h40;   // keypad 6
	8'h69: jis = 8'h42;   // keypad 1
	8'h72: jis = 8'h43;   // keypad 2
	8'h7A: jis = 8'h44;   // keypad 3
	8'h70: jis = 8'h46;   // keypad 0
	8'h71: jis = 8'h47;   // keypad .
	8'h05: jis = 8'h5D;   // F1
	8'h06: jis = 8'h5E;   // F2
	8'h04: jis = 8'h5F;   // F3
	8'h0C: jis = 8'h60;   // F4
	8'h03: jis = 8'h61;   // F5
	8'h0B: jis = 8'h62;   // F6
	8'h83: jis = 8'h63;   // F7
	8'h0A: jis = 8'h64;   // F8
	8'h01: jis = 8'h65;   // F9
	8'h09: jis = 8'h66;   // F10
	8'h78: jis = 8'h69;   // F11 -> PF11
	8'h07: jis = 8'h5B;   // F12 -> PF12
	8'h58: jis = 8'h55;   // caps lock
	8'h11: jis = 8'h5C;   // left alt
	8'h7E: jis = 8'h7C;   // scroll lock -> BREAK
	8'h0E: jis = 8'h5A;   // ` -> katakana
	8'h13: jis = 8'h5A;   // katakana/hiragana (JIS)
	8'h64: jis = 8'h58;   // henkan (JIS)
	8'h67: jis = 8'h57;   // muhenkan (JIS)
	default: jis = 8'h00;
	endcase
	else case (code)
	8'h4A: jis = 8'h37;   // keypad /
	8'h5A: jis = 8'h45;   // keypad enter
	8'h70: jis = 8'h48;   // insert
	8'h71: jis = 8'h4B;   // delete
	8'h6C: jis = 8'h4E;   // home
	8'h69: jis = 8'h73;   // end -> EXECUTE
	8'h7D: jis = 8'h6E;   // page up -> PREV
	8'h7A: jis = 8'h70;   // page down -> NEXT
	8'h75: jis = 8'h4D;   // up
	8'h6B: jis = 8'h4F;   // left
	8'h74: jis = 8'h51;   // right
	8'h72: jis = 8'h50;   // down
	8'h11: jis = 8'h59;   // right alt -> kana/kanji
	8'h14: jis = 8'h58;   // right ctrl -> convert
	8'h2F: jis = 8'h57;   // menu -> no convert
	8'h7C: jis = 8'h7D;   // print screen -> COPY
	default: jis = 8'h00;
	endcase
end

endmodule

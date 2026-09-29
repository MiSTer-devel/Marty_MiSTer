// Copyright (c) 2026 Jamie Blanks
//
// Towns RS-232C port with its modem card fitted, as software sees it: an
// 8251 plus the line-status, interrupt and modem-select registers around
// it. The Marty had neither; `enable` makes the addresses answer or stay
// empty. The far end is the framework UART, so the bit clock follows the
// baud the user picked there instead of the board's timer channel.
//
//   0A00/0A02  8251 data / command-status ──> txd, <── rxd
//   0A04       R line status {DSR, CD, CS, CI}
//   0A06       R interrupt cause {CI, CS, RSINT}; a read clears CI and CS
//   0A08       W {TxC ext, RxC ext, EXT DTR, CI, CS, SYNDET, RxRDY, TxRDY}
//   0A0A       R {ENBL, MODEM, MODINS} W {ENBL, MODEM}; MODINS low = card
//
// RSINT is the level OR of the enabled 8251 ready lines. CI and CS latch
// on the line's rising edge while enabled and clear when 0A06 is read.
// Undefined bits read 1.

module towns_rs232 #(
	parameter CLK_HZ = 57272727
)
(
	input             clk,
	input             reset,
	input             enable,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output            io_sel,

	input      [31:0] baud,          // wire rate of the far end

	output            txd,
	input             rxd,
	output            rts,           // active high, framework polarity
	output            dtr,
	input             cts,
	input             dsr,
	input             cd,
	input             ci,

	output            irq            // IRQ2
);

wire sel_usart = enable && (io_addr[15:2] == 14'h0280);   // 0A00, 0A02
wire sel_glue  = enable && (io_addr[15:4] == 12'h00A0) && (io_addr[3] | io_addr[2]);   // 0A04-0A0E
assign io_sel = sel_usart | sel_glue;

wire port_reset = reset | ~enable;

// Strobe edges for the glue registers
reg        wr_q, rd_q;
reg [15:0] addr_q;
reg  [7:0] din_q;
wire wr_active = io_wr & sel_glue;
wire rd_active = io_rd & sel_glue;
wire wr_edge = wr_q & ~wr_active;
wire rd_edge = rd_q & ~rd_active;
always @(posedge clk) begin
	wr_q <= wr_active;
	rd_q <= rd_active;
	if (wr_active || rd_active) begin
		addr_q <= io_addr;
		din_q  <= io_din;
	end
end

// ---- bit clock ----
// One tick per baud interval times the 8251 divisor, as a fraction of clk:
// 9600 baud at x16 is 153600 ticks a second.
wire  [1:0] baud_sel;
wire [31:0] tick_hz = (baud_sel == 2'd1) ? baud : (baud_sel == 2'd2) ? {baud[27:0], 4'd0} : {baud[25:0], 6'd0};
reg  [31:0] tick_acc;
reg         tick;
always @(posedge clk) begin
	tick <= 0;
	if (port_reset) tick_acc <= 0;
	else if (tick_acc + tick_hz >= CLK_HZ) begin
		tick_acc <= tick_acc + tick_hz - CLK_HZ;
		tick <= 1;
	end
	else tick_acc <= tick_acc + tick_hz;
end

// ---- USART ----
wire [7:0] u_dout;
wire       u_txrdy, u_rxrdy, u_brkdet, u_rts_n, u_dtr_n;

i8251 usart
(
	.clk(clk), .reset(port_reset),
	.c_d(io_addr[1]), .cs_n(~sel_usart), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(io_din), .d_o(u_dout), .d_oe(),
	.txc_ce(tick), .rxc_ce(tick), .cts_n(~cts), .dsr_n(~dsr),
	.rts_n(u_rts_n), .dtr_n(u_dtr_n), .txd(txd), .rxd(rxd),
	.txrdy(u_txrdy), .txempty(), .rxrdy(u_rxrdy), .brkdet(u_brkdet),
	.tx_full(), .rx_err(), .baud_sel(baud_sel)
);

// ---- glue registers ----
reg  [7:0] int_en;
reg        enbl, modem;
reg        ci_flag, cs_flag;
reg        ci_q, cs_q;

wire rsint = (int_en[0] & u_txrdy) | (int_en[1] & u_rxrdy) | (int_en[2] & u_brkdet);
assign irq = enable & (rsint | ci_flag | cs_flag);
assign rts = ~u_rts_n;
assign dtr = int_en[5] | ~u_dtr_n;

always @(posedge clk) begin
	ci_q <= ci;
	cs_q <= cts;
	if (port_reset) begin
		int_en  <= 8'h00;
		enbl    <= 0;
		modem   <= 1;
		ci_flag <= 0;
		cs_flag <= 0;
	end
	else begin
		if (int_en[4] && ci && !ci_q) ci_flag <= 1;
		if (int_en[3] && cts && !cs_q) cs_flag <= 1;
		if (wr_edge) case (addr_q[3:1])
			3'd4: int_en <= din_q;
			3'd5: {enbl, modem} <= din_q[7:6];
			default: ;
		endcase
		if (rd_edge && addr_q[3:1] == 3'd3) begin
			ci_flag <= 0;
			cs_flag <= 0;
		end
	end
end

always @(*) begin
	io_dout = 8'hFF;
	if (sel_usart) io_dout = u_dout;
	else if (sel_glue) case (io_addr[3:1])
		3'd2: io_dout = {4'hF, dsr, cd, cts, ci};
		3'd3: io_dout = {5'h1F, ci_flag, cs_flag, rsint};
		3'd5: io_dout = {enbl, modem, 1'b0, 5'h1F};
		default: ;
	endcase
end

endmodule

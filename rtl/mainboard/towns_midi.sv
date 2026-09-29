// Copyright (c) 2026 Jamie Blanks
//
// Towns MIDI interface card (FMT-40x family) as software sees it: two
// 8251 USARTs, a receive FIFO, a shared interrupt block and an 8253 for
// the MIDI clock. The Marty had no slot for one; `enable` makes the
// addresses answer or stay empty.
//
//   0E50/0E51  port A data / command-status ──> midi_tx, <── midi_rx
//   0E54/0E55  port B data / command-status (no sink)
//   0E52       receive FIFO: bytes port A received, a read pops one
//   0E53       FIFO register: W bit0 transfer enable; R bit0 busy (a byte
//              still in a holding buffer), bit1 FIFO empty, bit2 USART error
//   0E70       send interrupt mask; reads the masked TxRDY lines, active low
//   0E71       receive interrupt mask; same for "FIFO holds data"
//   0E73       timer interrupt mask (bits 1:0); reads the flags, clears
//   0E74-0E77  8253 counters 0-2 and control, 500 kHz clock
//
// A card clock of 500 kHz gives 31250 baud at the x16 mode and 2 us
// timer counts. The serial interrupt is the OR of the enabled ready
// lines, like the mainboard's RS-232C glue: a driver masks a port once
// it has nothing more to send. Timer flags latch on the OUT edge and
// clear when 0E73 is read.

module towns_midi
(
	input             clk,
	input             ce,            // fixed 16 MHz, for the card clock
	input             reset,
	input             enable,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output            io_sel,

	output            midi_tx,
	input             midi_rx,

	output            irq_serial,    // IRQ4
	output            irq_timer      // IRQ5
);

wire sel_usart1 = enable && (io_addr[15:1] == 15'h0728);   // 0E50, 0E51
wire sel_fifo   = enable && (io_addr[15:1] == 15'h0729);   // 0E52, 0E53
wire sel_usart2 = enable && (io_addr[15:1] == 15'h072A);   // 0E54, 0E55
wire sel_int    = enable && (io_addr[15:2] == 14'h039C);   // 0E70-0E73
wire sel_pit    = enable && (io_addr[15:2] == 14'h039D);   // 0E74-0E77
assign io_sel = sel_usart1 | sel_fifo | sel_usart2 | sel_int | sel_pit;

wire card_reset = reset | ~enable;

// 500 kHz card clock from the fixed 16 MHz
reg  [4:0] div;
reg        ce_card;
always @(posedge clk) begin
	ce_card <= 0;
	if (card_reset) div <= 5'd0;
	else if (ce) begin
		div <= div + 5'd1;
		if (div == 5'd31) ce_card <= 1;
	end
end

// Strobe edges for the card's own registers
reg wr_q, rd_q;
reg [15:0] addr_q;
reg  [7:0] din_q;
wire wr_active = io_wr & (sel_fifo | sel_int);
wire rd_active = io_rd & (sel_fifo | sel_int);
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

// ---- USARTs ----
// The card drains port A's receiver into the FIFO itself: a one-clock
// read cycle on the chip whenever it holds a byte and the CPU is away.
wire [7:0] u1_dout, u2_dout;
wire       u1_txrdy, u2_txrdy, u1_rxrdy, u1_full, u2_full, u1_err;
wire       u2_txd;
reg        drain, drain_q;
always @(posedge clk) begin
	drain   <= u1_rxrdy && !sel_usart1 && !drain && !drain_q && !card_reset;
	drain_q <= drain;
end

i8251 usart1
(
	.clk(clk), .reset(card_reset),
	.c_d(drain ? 1'b0 : io_addr[0]), .cs_n(~(sel_usart1 | drain)),
	.rd_n(~(io_rd | drain)), .wr_n(~(io_wr & ~drain)),
	.d_i(io_din), .d_o(u1_dout), .d_oe(),
	.txc_ce(ce_card), .rxc_ce(ce_card), .cts_n(1'b0), .dsr_n(1'b0),
	.rts_n(), .dtr_n(), .txd(midi_tx), .rxd(midi_rx),
	.txrdy(u1_txrdy), .txempty(), .rxrdy(u1_rxrdy), .tx_full(u1_full), .rx_err(u1_err), .baud_sel()
);

i8251 usart2
(
	.clk(clk), .reset(card_reset),
	.c_d(io_addr[0]), .cs_n(~sel_usart2), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(io_din), .d_o(u2_dout), .d_oe(),
	.txc_ce(ce_card), .rxc_ce(ce_card), .cts_n(1'b0), .dsr_n(1'b0),
	.rts_n(), .dtr_n(), .txd(u2_txd), .rxd(1'b1),
	.txrdy(u2_txrdy), .txempty(), .rxrdy(), .tx_full(u2_full), .rx_err(), .baud_sel()
);

// ---- receive FIFO ----
// in logic: the CPU reads the head combinationally while a byte may land
(* ramstyle = "logic" *) reg [7:0] fifo [0:15];
reg  [3:0] fifo_wp, fifo_rp;
wire       fifo_empty = fifo_wp == fifo_rp;
wire       fifo_pop   = rd_edge && addr_q[3:0] == 4'h2 && !addr_q[5] && !fifo_empty;
always @(posedge clk) begin
	if (card_reset) begin
		fifo_wp <= 4'd0;
		fifo_rp <= 4'd0;
	end
	else begin
		// the chip presents its byte during the drain clock
		if (drain && fifo_wp + 4'd1 != fifo_rp) begin
			fifo[fifo_wp] <= u1_dout;
			fifo_wp <= fifo_wp + 4'd1;
		end
		if (fifo_pop) fifo_rp <= fifo_rp + 4'd1;
	end
end

// ---- interrupt block ----
reg  [1:0] mask_send, mask_recv, mask_tmr;
reg  [1:0] flag_tmr;
reg  [1:0] out_q;
wire [1:0] send_now = {u2_txrdy, u1_txrdy} & mask_send;
wire [1:0] recv_now = {1'b0, ~fifo_empty} & mask_recv;
wire [2:0] pit_out;

always @(posedge clk) begin
	if (card_reset) begin
		mask_send <= 2'd0; mask_recv <= 2'd0; mask_tmr <= 2'd0;
		flag_tmr <= 2'd0;
		out_q <= pit_out[1:0];
	end
	else begin
		out_q  <= pit_out[1:0];
		flag_tmr  <= flag_tmr  | (pit_out[1:0] & ~out_q);

		if (wr_edge) case (addr_q[3:0])
			4'h3: if (addr_q[5]) begin
				// 0E73; bit 7 also clears the flags
				mask_tmr <= din_q[1:0];
				if (din_q[7]) flag_tmr <= 2'd0;
			end
			4'h0: mask_send <= din_q[1:0];
			4'h1: mask_recv <= din_q[1:0];
			default: ;
		endcase

		if (rd_edge && addr_q[5] && addr_q[1:0] == 2'd3) flag_tmr <= 2'd0;
	end
end

assign irq_serial = |send_now | |recv_now;
assign irq_timer  = |(flag_tmr & mask_tmr);

// ---- MIDI clock ----
wire [7:0] pit_dout;
i8253 pit
(
	.clk(clk), .reset(card_reset),
	.a(io_addr[1:0]), .cs_n(~sel_pit), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(io_din), .d_o(pit_dout), .d_oe(),
	.clk_ce({3{ce_card}}), .gate(3'b111), .out(pit_out)
);

// ---- read side ----
always @* begin
	io_dout = 8'hFF;
	if (sel_usart1) io_dout = u1_dout;
	if (sel_usart2) io_dout = u2_dout;
	if (sel_fifo)   io_dout = io_addr[0] ? {5'd0, u1_err, fifo_empty, u1_full | u2_full} :
	                          fifo_empty ? 8'hFF : fifo[fifo_rp];
	if (sel_pit)    io_dout = pit_dout;
	if (sel_int) case (io_addr[1:0])
		2'd0: io_dout = {6'h3F, ~send_now};
		2'd1: io_dout = {6'h3F, ~recv_now};
		2'd3: io_dout = {6'h00, flag_tmr};
		default: ;
	endcase
end

endmodule

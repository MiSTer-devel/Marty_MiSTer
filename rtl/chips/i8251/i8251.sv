// Copyright (c) 2026 Jamie Blanks
//
// Intel 8251A USART, asynchronous mode. The first command-port write
// after a reset is the mode word; a synchronous mode word swallows its
// sync characters and the receiver then stays idle. Command words
// follow; IR in a command returns to the mode word.
//
//   CPU write ──> holding buffer ──> shift register ──> TxD
//   RxD ──> start-bit hunt ──> shift register ──> receive buffer ──> CPU read
//
// TxC and RxC arrive as one-cycle enables. A bit lasts 1, 16 or 64
// of them per the mode word. The TxRDY pin needs TxEN and CTS_n low;
// the TxRDY status bit only needs the buffer empty. A character written
// while the transmitter is enabled still goes out if TxEN or CTS drops
// before it is loaded; one written while disabled waits.

module i8251
(
	input             clk,
	input             reset,

	input             c_d,          // 1 command / status, 0 data
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [7:0] d_i,
	output      [7:0] d_o,
	output            d_oe,

	input             txc_ce,
	input             rxc_ce,
	input             cts_n,
	input             dsr_n,
	output            rts_n,
	output            dtr_n,
	output            txd,
	input             rxd,

	output            txrdy,        // pin
	output            txempty,
	output            rxrdy,
	output            brkdet,       // SYNDET/BD pin, async mode

	// status taps for logic that sits beside the chip
	output            tx_full,      // holding buffer occupied
	output            rx_err,       // PE, OE or FE set
	output      [1:0] baud_sel      // mode word divisor: 01 x1, 10 x16, 11 x64
);

// Strobe edges. A cycle is one or more clocks with the strobe low.
reg       wr_q, rd_q, cd_q;
reg [7:0] d_q;
wire wr_active = ~cs_n & ~wr_n;
wire rd_active = ~cs_n & ~rd_n;
wire wr_edge = wr_q & ~wr_active;
wire rd_edge = rd_q & ~rd_active;

always @(posedge clk) begin
	wr_q <= wr_active;
	rd_q <= rd_active;
	if (wr_active || rd_active) begin
		cd_q <= c_d;
		d_q  <= d_i;
	end
end

// ---- mode and command words ----
localparam [1:0] ST_MODE = 2'd0, ST_SYNC1 = 2'd1, ST_SYNC2 = 2'd2, ST_CMD = 2'd3;
reg  [1:0] init;
reg  [7:0] mode;
reg        tx_en, rx_en, dtr, rts, sbrk, clr_err, soft_rst;

assign     baud_sel = mode[1:0];       // 00 sync, 01 x1, 10 x16, 11 x64
wire [1:0] nbits    = mode[3:2];       // 5 + n data bits
wire       pen      = mode[4];
wire       ep       = mode[5];
wire [1:0] stops    = mode[7:6];       // 01 1, 10 1.5, 11 2
wire       async    = baud_sel != 2'd0;

always @(posedge clk) begin
	clr_err  <= 0;
	soft_rst <= 0;
	if (reset) begin
		init  <= ST_MODE;
		mode  <= 8'h00;
		tx_en <= 0; rx_en <= 0; dtr <= 0; rts <= 0; sbrk <= 0;
	end
	else if (wr_edge && cd_q) begin
		case (init)
		ST_MODE: begin
			mode <= d_q;
			init <= (d_q[1:0] != 2'd0) ? ST_CMD : d_q[7] ? ST_SYNC1 : ST_SYNC2;
			tx_en <= 0; rx_en <= 0; dtr <= 0; rts <= 0; sbrk <= 0;
		end
		ST_SYNC1: init <= ST_CMD;
		ST_SYNC2: init <= ST_SYNC1;
		ST_CMD: begin
			if (d_q[6]) begin
				init <= ST_MODE;
				soft_rst <= 1;
				tx_en <= 0; rx_en <= 0; sbrk <= 0;
			end
			else begin
				tx_en   <= d_q[0];
				dtr     <= d_q[1];
				rx_en   <= d_q[2];
				sbrk    <= d_q[3];
				clr_err <= d_q[4];
				rts     <= d_q[5];
			end
		end
		endcase
	end
end

assign dtr_n = ~dtr;
assign rts_n = ~rts;

// Ticks per bit from the prescale; 1.5 stop bits are 24 ticks at x16.
wire [7:0] bit_ticks  = (baud_sel == 2'd1) ? 8'd1 : (baud_sel == 2'd2) ? 8'd16 : 8'd64;
wire [7:0] stop_ticks = (stops == 2'd3) ? {bit_ticks[6:0], 1'b0} :
                        (stops == 2'd2 && baud_sel != 2'd1) ? {1'b0, bit_ticks[7:1]} + bit_ticks : bit_ticks;
wire [3:0] data_len   = 4'd5 + {2'd0, nbits};

// ---- transmitter ----
reg  [7:0] tx_hold;
reg        tx_full_q;
reg        tx_armed;     // holding byte was written while enabled
reg  [7:0] tx_shift;
reg  [3:0] tx_bit;       // bits left in the shift register
reg  [7:0] tx_tick;
reg        tx_parity;
reg        tx_out;
localparam [1:0] TX_IDLE = 2'd0, TX_START = 2'd1, TX_DATA = 2'd2, TX_STOP = 2'd3;
reg  [1:0] tx_st;
reg        tx_par_phase;

wire tx_data_wr = wr_edge && !cd_q;
wire tx_go      = tx_full_q && (tx_armed || (tx_en && !cts_n)) && async;
wire tx_last    = tx_st == TX_STOP && tx_tick == 8'd1;

always @(posedge clk) begin
	if (reset || soft_rst) begin
		tx_full_q <= 0;
		tx_armed <= 0;
		tx_st   <= TX_IDLE;
		tx_out  <= 1;
		tx_tick <= 8'd0;
		tx_par_phase <= 0;
	end
	else begin
		if (tx_data_wr) begin
			tx_hold <= d_q;
			tx_full_q <= 1;
			tx_armed  <= tx_en && !cts_n;
		end
		if (txc_ce) begin
			case (tx_st)
			TX_IDLE: tx_out <= 1;
			TX_START, TX_DATA: begin
				if (tx_tick != 8'd1) tx_tick <= tx_tick - 8'd1;
				else begin
					tx_tick <= bit_ticks;
					if (tx_bit != 4'd0) begin
						tx_out    <= tx_shift[0];
						tx_parity <= tx_parity ^ tx_shift[0];
						tx_shift  <= {1'b0, tx_shift[7:1]};
						tx_bit    <= tx_bit - 4'd1;
						tx_st     <= TX_DATA;
					end
					else if (pen && !tx_par_phase) begin
						tx_out       <= tx_parity;
						tx_par_phase <= 1;
					end
					else begin
						tx_out  <= 1;
						tx_tick <= stop_ticks;
						tx_st   <= TX_STOP;
					end
				end
			end
			TX_STOP: begin
				if (tx_tick != 8'd1) tx_tick <= tx_tick - 8'd1;
				else begin
					tx_st   <= TX_IDLE;
					tx_tick <= 8'd0;
				end
			end
			endcase
			// next character: from idle, or straight after the stop bit
			if ((tx_st == TX_IDLE || tx_last) && tx_go) begin
				tx_shift  <= tx_hold;
				tx_full_q <= 0;
				tx_parity <= ~ep;
				tx_bit    <= data_len;
				tx_tick   <= bit_ticks;
				tx_out    <= 0;
				tx_st     <= TX_START;
				tx_par_phase <= 0;
			end
		end
	end
end

assign txd     = tx_out & ~sbrk;
assign txempty = tx_st == TX_IDLE && !tx_go;
assign txrdy   = ~tx_full_q & tx_en & ~cts_n;

// ---- receiver ----
// A low stop bit is a framing error; the receiver then treats the next
// bit slot as a possible start bit, so a break delivers an all-zero
// character every frame time. Two of those set break detect, which
// clears when RxD is next seen high and drops any frame in progress.
// After a hardware reset the line must be seen high once before a low
// counts as a start bit.
reg  [7:0] rx_buf;
reg        rx_rdy;
reg  [7:0] rx_shift;
reg  [3:0] rx_bit;
reg  [7:0] rx_tick;
reg        rx_parity, rx_par_phase;
reg        rx_zero;      // every bit of the frame so far was 0
reg        fe, oe, pe;
reg        bd, brk_pend;
reg        rxd_q;
localparam [1:0] RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2;
reg  [1:0] rx_st;

wire rx_data_rd = rd_edge && !cd_q;
wire rx_sample  = rx_st == RX_DATA && rx_tick == 8'd1;
wire rx_stop    = rx_sample && rx_bit == 4'd0 && (!pen || rx_par_phase);

always @(posedge clk) begin
	if (reset) rxd_q <= 0;
	if (reset || soft_rst) begin
		rx_rdy <= 0;
		rx_st  <= RX_IDLE;
		fe <= 0; oe <= 0; pe <= 0;
		bd <= 0; brk_pend <= 0;
	end
	else begin
		if (rx_data_rd) rx_rdy <= 0;
		if (clr_err) begin fe <= 0; oe <= 0; pe <= 0; end
		if (rxc_ce) begin
			rxd_q <= rxd;
			case (rx_st)
			RX_IDLE: if (rx_en && async && rxd_q && !rxd) begin
				rx_bit    <= data_len;
				rx_shift  <= 8'h00;
				rx_parity <= ~ep;
				rx_par_phase <= 0;
				rx_zero   <= 1;
				// x16/x64 re-check the start bit at its middle; x1 has
				// no middle, the next clock is the first data bit
				rx_tick <= (baud_sel == 2'd1) ? 8'd1 : {1'b0, bit_ticks[7:1]};
				rx_st   <= (baud_sel == 2'd1) ? RX_DATA : RX_START;
			end
			RX_START: begin
				if (rx_tick != 8'd1) rx_tick <= rx_tick - 8'd1;
				else if (!rxd) begin
					rx_tick <= bit_ticks;
					rx_st   <= RX_DATA;
				end
				else rx_st <= RX_IDLE;
			end
			RX_DATA: begin
				if (rx_tick != 8'd1) rx_tick <= rx_tick - 8'd1;
				else begin
					rx_tick <= bit_ticks;
					if (rxd) rx_zero <= 0;
					if (rx_bit != 4'd0) begin
						rx_shift  <= {rxd, rx_shift[7:1]};
						rx_parity <= rx_parity ^ rxd;
						rx_bit    <= rx_bit - 4'd1;
					end
					else if (pen && !rx_par_phase) begin
						if (rxd != rx_parity) pe <= 1;
						rx_par_phase <= 1;
					end
					else begin
						// stop bit: right-align the character, flag a
						// missing stop or an unread previous byte
						if (rx_rdy) oe <= 1;
						rx_buf <= rx_shift >> (4'd8 - data_len);
						rx_rdy <= 1;
						rx_st  <= RX_IDLE;
						brk_pend <= 0;
						if (!rxd) begin
							fe <= 1;
							// the next bit slot may already be a start bit
							rx_bit    <= data_len;
							rx_shift  <= 8'h00;
							rx_parity <= ~ep;
							rx_par_phase <= 0;
							rx_zero   <= 1;
							rx_tick   <= bit_ticks;
							rx_st     <= RX_START;
							if (rx_zero) begin
								brk_pend <= 1;
								if (brk_pend) bd <= 1;
							end
						end
					end
				end
			end
			default: rx_st <= RX_IDLE;
			endcase
			// the line coming back up ends the break and the frame with it
			if (bd && rxd) begin
				bd <= 0;
				brk_pend <= 0;
				if (!rx_stop) rx_st <= RX_IDLE;
			end
		end
	end
end

assign rxrdy   = rx_rdy & rx_en;
assign brkdet  = bd;
assign tx_full = tx_full_q;
assign rx_err  = fe | oe | pe;

// ---- bus ----
wire [7:0] status = {~dsr_n, bd, fe, oe, pe, txempty, rxrdy, ~tx_full_q};
assign d_o  = c_d ? status : rx_buf;
assign d_oe = rd_active;

endmodule

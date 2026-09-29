// Copyright (c) 2026 Jamie Blanks
//
// CPU side of the CD-ROM controller (Technical Databook 3rd ed. §6.3):
// the registers at 04C0H-04C6H, the parameter and status FIFOs, the two
// interrupt flags and the sector transfer engine. The sub-MPU model
// behind it decides what the status bytes say; this block only moves them.
//
//   04C0 W {SMIC, DEIC, 0,0,0, SRST, SMIM, DEIM}   R {SIRQ, DEI, STSF, DTSF, 0,0, SRQ, DRY}
//   04C2 W command                                 R status FIFO, four bytes per reply
//   04C4 W parameter FIFO, eight bytes             R sector data during a CPU transfer
//   04C6 W {0,0,0, DTS, STS, 0,0,0}
//   04CC R {0,0,0,0,0,0, OVER-RUN, SUBC-DAT-R}   04CD R subcode byte, P in bit 7 down to W
//
// A sector is announced by the sub-MPU, then moved either by DMA channel 3
// (fly-by, one byte or word per strobe, ends at the sector or at TC) or by
// the CPU reading 04C4H. DTS and STS are mode bits: written before or
// after the sector arrives, cleared by hardware when the transfer ends.
//
// The buffer holds a sector without its sync, so the three read forms are
// windows on the same 2340 bytes (header, then 2336):
//
//   MODE1  2048 from +4     user data; from +12 on a mode-2 track (after the subheader)
//   MODE2  2336 from +4     mode 1: user + EDC/ECC; mode 2: subheader first
//   RAW    2340 from +0     header (BCD M S F, mode) first

module towns_cdc
(
	input             clk,
	input             ce,             // CPU enable, for the I/O strobes
	input             ce_16m,         // fixed 16 MHz, for the drive timers
	input             reset,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output reg        io_sel,

	output            irq,            // IRQ 9

	// DMA channel 3, device to memory
	output            dma_req,
	input             dma_ack,        // this channel acknowledged
	input             dma_iord,       // IORD strobe low
	input             dma_word,       // word transfer this strobe
	input             dma_tc,         // terminal count with this strobe
	output     [15:0] dma_dout,

	// sub-MPU
	output            mpu_reset,
	output            ce_us,
	output reg        cmd_strobe,
	output reg  [7:0] cmd_out,
	output reg [63:0] params,
	input             st_push,
	input      [31:0] st_data,
	input             st_clear,
	output            st_full,
	input             dry,
	input             sirq_set,
	input             sirq_clr,
	output            dei_out,
	input             data_ready,
	input       [1:0] sector_form,
	input             sector_mode2,   // track is mode 2: subheader before the user data
	output reg        xfer_start,
	output reg        xfer_done,
	input             xfer_abort,

	// sector buffer fill from the ATAPI host
	input             sec_we,
	input      [10:0] sec_addr,
	input      [15:0] sec_data,

	// subcode bytes from the host, one per frame while audio plays
	input             sub_we,
	input       [7:0] sub_data,

	// savestate port: registers and the status queue as bytes; 40h
	// streams the sector buffer, any access to 41h rewinds it
	input             ss_cs,
	input             ss_wr,
	input             ss_step,
	input       [6:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

wire sel   = (io_addr[15:4] == 12'h04C);
wire rd    = io_rd & ce & sel;
wire wr    = io_wr & ce & sel;
wire r_4c0 = io_addr[3:0] == 4'h0, r_4c2 = io_addr[3:0] == 4'h2, r_4c4 = io_addr[3:0] == 4'h4, r_4c6 = io_addr[3:0] == 4'h6;
wire r_4cd = io_addr[3:0] == 4'hD;

// subcode register: a byte waits until the CPU reads it; the next one
// arriving first is an overrun
reg  [7:0] sub_byte;
reg        sub_datr, sub_over;

reg  sirq, dei, stsf, dtsf, smim, deim, srst;
reg  dts, sts;             // transfer control bits, cleared at transfer end
// the line is the two requests through their masks; SMIC/DEIC drop the requests
assign irq      = (sirq & smim) | (dei & deim);
assign dei_out  = dei;
assign mpu_reset = reset | srst;

// 1 MHz tick for the drive timers
reg [3:0] us_div;
assign ce_us = ce_16m && us_div == 4'd15;
always @(posedge clk) if (reset) us_div <= 0; else if (ss_cs && ss_wr && ss_a == 7'h02) us_div <= ss_din[3:0]; else if (ce_16m) us_div <= us_div + 1'd1;

// ---- parameter FIFO and command ----
reg  [7:0] param [0:7];
reg  [3:0] nparam;
reg        cmd_received;
reg  [7:0] cmd_r;
wire       cmd_complete = cmd_received && (nparam >= 4'd8 || cmd_r[4:0] == 5'd5);

// ---- status FIFO: eight replies of four bytes ----
reg [31:0] st_q [0:7];
reg  [2:0] st_wr, st_rd;
reg  [3:0] st_cnt /*verilator public*/;
reg  [1:0] st_byte;
wire       st_empty = st_cnt == 0;
assign     st_full  = st_cnt[3];
// The state port shares the queue's read mux and write port: bytes 10-2F
// are the eight entries, high byte first, the same order the guest reads.
wire [31:0] st_cur   = st_q[ss_cs ? ss_a[4:2] : st_rd];
wire  [1:0] st_sel   = ss_cs ? ss_a[1:0] : st_byte;
wire  [7:0] st_head  = st_sel == 2'd0 ? st_cur[31:24] : st_sel == 2'd1 ? st_cur[23:16] :
                       st_sel == 2'd2 ? st_cur[15:8]  : st_cur[7:0];
wire        st_qwr   = ss_cs ? ss_wr && (ss_a[6:4] == 3'd1 || ss_a[6:4] == 3'd2) : st_clear || (st_push && !st_full);
wire  [2:0] st_qidx  = ss_cs ? ss_a[4:2] : st_clear ? 3'd0 : st_wr;
wire [31:0] st_qdata = ss_cs ? {4{ss_din}} : st_data;
wire  [3:0] st_qbe   = ss_cs ? 4'b1000 >> ss_a[1:0] : 4'b1111;   // [3] is the high byte
// the parameter FIFO's indexed write lane
wire  [2:0] par_idx  = ss_cs ? ss_a[2:0] : nparam[2:0];
wire  [7:0] par_data = ss_cs ? ss_din : io_din;
wire        par_wr   = ss_cs ? ss_wr && ss_a[6:3] == 4'b0001 : wr && r_4c4 && nparam < 4'd8;

// ---- sector transfer ----
reg        armed;          // a sector waits in the buffer
reg        dma_active;
reg  [1:0] form;
reg        mode2;
reg [11:0] len;            // bytes in this sector as delivered
reg [11:0] ptr;            // next byte to hand out
wire [15:0] buf_q;
// where the window starts in the buffered sector
wire  [3:0] off = form == 2'd2 ? 4'd0 : form == 2'd1 ? 4'd4 : mode2 ? 4'd12 : 4'd4;
wire [11:0] idx = ptr + {8'd0, off};

wire [7:0] byte_lo = idx[0] ? buf_q[15:8] : buf_q[7:0];
wire [7:0] byte_hi = buf_q[15:8];   // word transfers keep idx even
assign dma_dout = dma_word ? {byte_hi, byte_lo} : {byte_lo, byte_lo};
assign dma_req  = dma_active;

// the strobe ends when IORD rises; that is when the DMAC has taken the data
reg  strobe_q;
reg  data_ready_q;
wire strobe = dma_ack & dma_iord & dma_active;
wire strobe_end = strobe_q & ~strobe;
reg  tc_q;

// the state port streams the buffer a byte at a time through the host's port
reg  [11:0] ss_ba;
wire        ss_buf = ss_cs && ss_a == 7'h40;
wire [15:0] ss_bq;
reg   [7:0] ss_bhold;                 // low byte kept while the high one is written
always @(posedge clk) begin
	if (reset || (ss_cs && ss_a == 7'h41)) ss_ba <= 12'd0;
	else if (ss_buf && ss_step) ss_ba <= ss_ba + 1'd1;
	if (ss_buf && ss_wr && !ss_ba[0]) ss_bhold <= ss_din;
end

cache_ram_dp #(.ADDR_WIDTH(11), .DATA_WIDTH(16)) sector_buf
(
	.clk_i(clk),
	.addr_a_i(ss_cs ? ss_ba[11:1] : sec_addr), .wren_a_i(ss_cs ? (ss_buf && ss_wr && ss_ba[0]) : sec_we),
	.wdata_a_i(ss_cs ? {ss_din, ss_bhold} : sec_data), .q_a_o(ss_bq),
	.addr_b_i(idx[11:1]), .wren_b_i(1'b0), .wdata_b_i(16'd0), .q_b_o(buf_q)
);

always @* begin
	if (ss_a[6:4] == 3'd1 || ss_a[6:4] == 3'd2) ss_dout = st_head;   // 10-2F: the status queue
	else if (ss_a[6:3] == 4'b0001) ss_dout = param[ss_a[2:0]];   // 08-0F
	else case (ss_a)
		7'h00: ss_dout = {sirq, dei, stsf, dtsf, smim, deim, irq, 1'b0};
		7'h01: ss_dout = {dts, sts, cmd_received, armed, dma_active, form, mode2};
		7'h02: ss_dout = {4'd0, us_div};
		7'h03: ss_dout = {4'd0, nparam};
		7'h04: ss_dout = cmd_r;
		7'h05: ss_dout = {2'd0, st_wr, st_rd};
		7'h06: ss_dout = {2'd0, st_cnt, st_byte};
		7'h07: ss_dout = {5'd0, strobe_q, data_ready_q, tc_q};
		7'h30: ss_dout = len[7:0];
		7'h31: ss_dout = {4'd0, len[11:8]};
		7'h32: ss_dout = ptr[7:0];
		7'h33: ss_dout = {4'd0, ptr[11:8]};
		7'h40: ss_dout = ss_ba[0] ? ss_bq[15:8] : ss_bq[7:0];
		default: ss_dout = 8'h00;
	endcase
end

always @* begin
	io_sel  = sel;
	io_dout = 8'hFF;
	case (io_addr[3:0])
	4'h0: io_dout = {sirq, dei, stsf, dtsf, 2'b00, ~st_empty, dry};
	4'h2: io_dout = st_empty ? 8'hFF : st_head;   // empty FIFO reads as FF; software polls for a non-zero byte
	4'h4: io_dout = stsf ? byte_lo : 8'h00;
	4'hC: io_dout = {6'd0, sub_over, sub_datr};
	4'hD: io_dout = sub_byte;
	default: io_dout = 8'hFF;
	endcase
end

always @(posedge clk) begin
	cmd_strobe <= 0;
	xfer_start <= 0;
	xfer_done  <= 0;
	srst       <= 0;
	strobe_q   <= strobe;
	tc_q       <= dma_tc;
	data_ready_q <= data_ready;

	if (reset | srst) begin
		sirq <= 0; dei <= 0; stsf <= 0; dtsf <= 0;
		if (reset) begin smim <= 0; deim <= 0; end
		nparam <= 0; cmd_received <= 0; cmd_r <= 0;
		st_wr <= 0; st_rd <= 0; st_cnt <= 0; st_byte <= 0;
		armed <= 0; dma_active <= 0; ptr <= 0; len <= 0; form <= 0; mode2 <= 0;
		dts <= 0; sts <= 0;
		sub_byte <= 0; sub_datr <= 0; sub_over <= 0;
	end
	else begin
		// ---- register writes ----
		if (wr && r_4c0) begin
			if (io_din[7]) sirq <= 0;
			if (io_din[6]) dei  <= 0;
			srst <= io_din[2];
			smim <= io_din[1];
			deim <= io_din[0];
		end
		if (wr && r_4c2) begin
			// a command sent before the last reply was cleared inherits its flags
			cmd_r <= sirq ? (io_din | (cmd_r & 8'h60)) : io_din;
			cmd_received <= 1;
		end
		if (wr && r_4c4) begin
			if (nparam >= 4'd8) begin
				{param[6], param[5], param[4], param[3], param[2], param[1], param[0]} <=
					{param[7], param[6], param[5], param[4], param[3], param[2], param[1]};
				param[7] <= io_din;
			end
			else nparam <= nparam + 1'd1;
		end
		if (wr && r_4c6) begin
			dts <= io_din[4];
			sts <= io_din[3];
		end

		// ---- a sector waits and a transfer mode is set ----
		if (armed && !dtsf && !stsf) begin
			if (dts) begin
				dtsf <= 1;
				dma_active <= 1;
				xfer_start <= 1;
			end
			else if (sts) begin
				stsf <= 1;
				xfer_start <= 1;
			end
		end

		// ---- command hand-off, one clock after the last byte ----
		if (cmd_complete && !cmd_strobe) begin
			cmd_strobe <= 1;
			cmd_out    <= cmd_r;
			params     <= {param[7], param[6], param[5], param[4], param[3], param[2], param[1], param[0]};
			cmd_received <= 0;
			nparam <= 0;
		end

		// ---- status FIFO ----
		if (st_clear) begin
			// a reply pushed with the clear becomes the first entry
			st_rd <= 0; st_byte <= 0;
			st_wr  <= {2'd0, st_push};
			st_cnt <= {3'd0, st_push};
		end
		else begin
			if (st_push && !st_full) st_wr <= st_wr + 1'd1;
			if (rd && r_4c2 && !st_empty) begin
				st_byte <= st_byte + 1'd1;
				if (st_byte == 2'd3) begin
					st_rd <= st_rd + 1'd1;
					// another reply waits and the command asked for interrupts
					if (cmd_r[6] && st_cnt > 4'd1) sirq <= 1;
				end
			end
			st_cnt <= st_cnt + {3'd0, st_push && !st_full} - {3'd0, rd && r_4c2 && !st_empty && st_byte == 2'd3};
		end

		// ---- flags from the sub-MPU ----
		if (sirq_set) sirq <= 1;
		if (sirq_clr) sirq <= 0;

		// ---- sector announced: arm once on the rising edge, since the level
		// outlives the transfer start and would keep resetting the pointer ----
		if (data_ready && !data_ready_q) begin
			armed <= 1;
			form  <= sector_form;
			mode2 <= sector_mode2;
			len   <= sector_form == 2'd0 ? 12'd2048 : sector_form == 2'd1 ? 12'd2336 : 12'd2340;
			ptr   <= 0;
		end

		// ---- DMA: advance at the end of each strobe ----
		if (strobe_end) begin
			if (ptr + {11'd0, dma_word} + 12'd1 >= len || tc_q) begin
				dma_active <= 0;
				armed <= 0;
				dtsf  <= 0;
				dts   <= 0;
				dei   <= 1;
				xfer_done <= 1;
			end
			else ptr <= ptr + {11'd0, dma_word} + 12'd1;
		end

		// ---- CPU transfer: one byte per read; STSF dropping is its end ----
		if (rd && r_4c4 && stsf) begin
			if (ptr + 12'd1 >= len) begin
				stsf  <= 0;
				sts   <= 0;
				armed <= 0;
				xfer_done <= 1;
				ptr <= 0;
			end
			else ptr <= ptr + 1'd1;
		end

		if (sub_we) begin
			sub_byte <= sub_data;
			sub_datr <= 1;
			if (sub_datr) sub_over <= 1;
		end
		if (rd && r_4cd) begin sub_datr <= 0; sub_over <= 0; end

		if (xfer_abort) begin
			dtsf <= 0; stsf <= 0; dts <= 0; sts <= 0; dma_active <= 0; armed <= 0;
		end
		if (par_wr) param[par_idx] <= par_data;
		if (st_qwr) begin
			if (st_qbe[3]) st_q[st_qidx][31:24] <= st_qdata[31:24];
			if (st_qbe[2]) st_q[st_qidx][23:16] <= st_qdata[23:16];
			if (st_qbe[1]) st_q[st_qidx][15:8]  <= st_qdata[15:8];
			if (st_qbe[0]) st_q[st_qidx][7:0]   <= st_qdata[7:0];
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
				7'h00: {sirq, dei, stsf, dtsf, smim, deim} <= ss_din[7:2];
				7'h01: {dts, sts, cmd_received, armed, dma_active, form, mode2} <= ss_din;
				7'h03: nparam <= ss_din[3:0];
				7'h04: cmd_r <= ss_din;
				7'h05: {st_wr, st_rd} <= ss_din[5:0];
				7'h06: {st_cnt, st_byte} <= ss_din[5:0];
				7'h30: len[7:0] <= ss_din;
				7'h31: len[11:8] <= ss_din[3:0];
				7'h32: ptr[7:0] <= ss_din;
				7'h33: ptr[11:8] <= ss_din[3:0];
				default: ;
			endcase
		end
	end
end

endmodule

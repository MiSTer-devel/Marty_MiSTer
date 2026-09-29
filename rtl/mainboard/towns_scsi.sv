// Copyright (c) 2026 Jamie Blanks
//
// The SCSI host interface of the rear-port card, as the Databook and the
// ROM driver see it: three byte registers in front of the bus lines.
//
//   0C30  data       R/W  the bus data lines; every access handshakes ACK
//   0C32  status     R    {REQ, I/O, MSG, C/D, BUSY, 0, INT, PERR}
//   0C32  control    W    {WEN, IMSK, 0, ATN, 0, SEL, DMAE, RST}
//   0C34  word DMA   R    FF: this card generation has no word transfers
//
// WEN gates every line the card drives. INT rises with REQ in a command,
// status or message phase and clears on the next data access; IMSK set
// lets it through as IRQ8 (the ROM driver polls with IMSK clear). DMAE
// turns REQ in a data phase into a request on DMA channel 1; each strobe
// moves one byte through the data register with the same ACK handshake.
// A selection is the ID mask written to the data register, then SEL set
// and cleared around BUSY from the target.

module towns_scsi
(
	input             clk,
	input             ce,            // 16 MHz T-state enable
	input             reset,
	input             enable,        // the card is fitted

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output      [7:0] io_dout,
	output            io_sel,
	output            irq,
	// savestate port: the control latches as two bytes
	input             ss_cs,
	input             ss_wr,
	input             ss_a,
	input       [7:0] ss_din,
	output      [7:0] ss_dout,

	// DMA channel 1: a byte per strobe, taken when the strobe ends
	output            dma_req,
	input             dma_ack,
	input             dma_iord,
	input             dma_iowr,
	input       [7:0] dma_din,
	output      [7:0] dma_dout,

	// SCSI bus, initiator side
	output            bus_sel,
	output            bus_atn,
	output            bus_rst,
	output reg        bus_ack,
	output      [7:0] bus_dout,
	output            bus_doe,       // driving the data lines
	input             bus_bsy,
	input             bus_req,
	input             bus_msg,
	input             bus_cd,
	input             bus_io,
	input       [7:0] bus_din
);

wire sel_data = enable && (io_addr[15:1] == 15'h0618);   // 0C30
wire sel_ctrl = enable && (io_addr[15:1] == 15'h0619);   // 0C32
wire sel_word = enable && (io_addr[15:1] == 15'h061A);   // 0C34
assign io_sel = sel_data | sel_ctrl | sel_word;

reg        wen, imsk, atn, sel, dmae, rst;
reg  [7:0] data_out;
reg        int_r;
reg        req_q;
reg        strobe_q, iowr_q;

assign bus_sel  = wen & sel;
assign bus_atn  = wen & atn;
assign bus_rst  = wen & rst;
assign bus_dout = data_out;
assign bus_doe  = wen & ~bus_io;

wire   data_phase = bus_bsy & ~bus_cd & ~bus_msg;
assign dma_req  = enable & dmae & data_phase & bus_req & ~bus_ack;
assign dma_dout = bus_din;
assign irq      = enable & imsk & int_r;

// a byte under ACK is already handshaked: REQ reads clear until the next one
wire [7:0] status = {bus_req & ~bus_ack, bus_io, bus_msg, bus_cd, bus_bsy, 1'b0, int_r, 1'b0};
assign io_dout = sel_ctrl ? status : sel_data ? (bus_io ? bus_din : data_out) : 8'hFF;

// a DMA byte is taken when its strobe ends
wire strobe     = dma_ack & (dma_iord | dma_iowr);
wire strobe_end = strobe_q & ~strobe;

// CPU access to the data register, once per lane
wire cpu_data = ce & sel_data & (io_rd | io_wr);
wire access   = cpu_data | strobe_end;

always @(posedge clk) begin
	strobe_q <= strobe;
	iowr_q   <= dma_ack & dma_iowr;
	req_q    <= bus_req;
	if (reset || !enable) begin
		{wen, imsk, atn, sel, dmae, rst} <= 6'd0;
		data_out <= 8'd0;
		int_r    <= 0;
		bus_ack  <= 0;
	end
	else begin
		if (ce && sel_ctrl && io_wr) begin
			{wen, imsk} <= io_din[7:6];
			atn  <= io_din[4];
			sel  <= io_din[2];
			dmae <= io_din[1];
			rst  <= io_din[0];
		end

		// the byte on its way out: from the CPU or from memory by DMA
		if (ce && sel_data && io_wr) data_out <= io_din;
		if (strobe_end && iowr_q)    data_out <= dma_din;

		// ACK follows the access and holds until the target drops REQ
		if (access && bus_req && bus_bsy) bus_ack <= 1;
		else if (!bus_req) bus_ack <= 0;

		// INT: REQ rising in a command, status or message phase
		if (rst) int_r <= 0;
		else if (bus_req && !req_q && bus_bsy && (bus_cd || bus_msg)) int_r <= 1;
		else if (access) int_r <= 0;

		if (ss_cs && ss_wr && !ss_a) {wen, imsk, atn, sel, dmae, rst, int_r} <= ss_din[6:0];
		if (ss_cs && ss_wr &&  ss_a) data_out <= ss_din;
	end
end
assign ss_dout = ss_a ? data_out : {1'b0, wen, imsk, atn, sel, dmae, rst, int_r};

endmodule

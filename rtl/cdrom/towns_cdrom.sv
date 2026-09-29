// Copyright (c) 2026 Jamie Blanks
//
// The Marty's CD-ROM: controller registers, sub-MPU drive model and the
// host that reads raw sectors from the framework's image slot.
//
//   CPU bus ── towns_cdc ──┬── cd_sub_mpu ── hps_cd_host ── hps_blk_slot (Linux)
//                          │        │              │
//                          │     status FIFO    TOC RAM, sector buffer, CD audio
//                          └── DMA channel 3 / IRQ 9

module towns_cdrom #(
	parameter CLK_RATE     = 57272727,
	parameter SECTOR_US    = 13333,
	parameter SEEK_BASE_US = 20000,
	parameter LOSTDATA_US  = 100000,
	parameter NOTIFY_US    = 1000
)
(
	input             clk,
	input             ce,
	input             ce_16m,
	input             reset,

	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output      [7:0] io_dout,
	output            io_sel,
	output            irq,

	output            dma_req,
	input             dma_ack,
	input             dma_iord,
	input             dma_word,
	input             dma_tc,
	output     [15:0] dma_dout,

	// TOC record download
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input       [9:0] ioctl_addr,
	input       [7:0] ioctl_dout,

	// image slot
	input             img_present,
	input             img_mounted,    // one clock: Linux mounted or ejected the image
	output     [23:0] blk_lba,
	output            blk_bank,
	output            blk_rd,
	input             blk_done,
	input             blk_err,
	output     [12:0] buf_addr,
	input       [7:0] buf_dout,

	// CD audio
	output            cdda_ce,
	output     [15:0] cdda_l,
	output     [15:0] cdda_r,

	// savestate ports of the controller, the drive model and the host
	input             ss_hold,
	input             ss_cs_cdc,
	input             ss_cs_mpu,
	input             ss_cs_host,
	input             ss_wr,
	input             ss_step,
	input       [6:0] ss_a,
	input       [7:0] ss_din,
	output      [7:0] ss_dout_cdc,
	output      [7:0] ss_dout_mpu,
	output      [7:0] ss_dout_host,
	output            ss_quiet,       // no host request on its way
	output            ss_busy         // the host is still resuming the audio
);

wire        mpu_reset, ce_us, cmd_strobe, st_push, st_clear, st_full, dry, sirq_set, sirq_clr, dei;
wire  [7:0] cmd;
wire [63:0] params;
wire [31:0] st_data;
wire        data_ready, xfer_start, xfer_done, xfer_abort;
wire  [1:0] sector_form;
wire        h_req, h_busy, h_done, h_err, sector_mode2;
wire  [3:0] h_op;
wire [23:0] h_lba, h_lba_end;
wire        sec_we, sub_we;
wire  [7:0] sub_data;
wire  [6:0] toc_raddr;
wire [31:0] toc_rdata;
wire  [7:0] toc_first, toc_last;
wire [99:0] toc_mode2;
wire [10:0] sec_addr;
wire [15:0] sec_data;
wire  [7:0] sq_status, sq_ctrl, sq_track, sq_index, sq_abs_m, sq_abs_s, sq_abs_f, sq_rel_m, sq_rel_s, sq_rel_f;

towns_cdc cdc
(
	.clk(clk), .ce(ce), .ce_16m(ce_16m), .reset(reset),
	.io_addr(io_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(io_din), .io_dout(io_dout), .io_sel(io_sel),
	.irq(irq),
	.dma_req(dma_req), .dma_ack(dma_ack), .dma_iord(dma_iord), .dma_word(dma_word), .dma_tc(dma_tc), .dma_dout(dma_dout),
	.mpu_reset(mpu_reset), .ce_us(ce_us),
	.cmd_strobe(cmd_strobe), .cmd_out(cmd), .params(params),
	.st_push(st_push), .st_data(st_data), .st_clear(st_clear), .st_full(st_full),
	.dry(dry), .sirq_set(sirq_set), .sirq_clr(sirq_clr), .dei_out(dei),
	.data_ready(data_ready), .sector_form(sector_form), .sector_mode2(sector_mode2),
	.xfer_start(xfer_start), .xfer_done(xfer_done), .xfer_abort(xfer_abort),
	.sec_we(sec_we), .sec_addr(sec_addr), .sec_data(sec_data), .sub_we(sub_we), .sub_data(sub_data),
	.ss_cs(ss_cs_cdc), .ss_wr(ss_wr), .ss_step(ss_step), .ss_a(ss_a), .ss_din(ss_din), .ss_dout(ss_dout_cdc)
);

cd_sub_mpu #(.SECTOR_US(SECTOR_US), .SEEK_BASE_US(SEEK_BASE_US), .LOSTDATA_US(LOSTDATA_US), .NOTIFY_US(NOTIFY_US)) mpu
(
	.clk(clk), .ce_us(ce_us), .reset(mpu_reset),
	.cmd_strobe(cmd_strobe), .cmd(cmd), .params(params),
	.st_push(st_push), .st_data(st_data), .st_clear(st_clear), .st_full(st_full),
	.dry(dry), .sirq_set(sirq_set), .sirq_clr(sirq_clr), .dei(dei), .irq_line(irq), .hold(ss_hold),
	.data_ready(data_ready), .sector_form(sector_form), .sector_mode2(sector_mode2),
	.xfer_start(xfer_start), .xfer_done(xfer_done), .xfer_abort(xfer_abort),
	.h_req(h_req), .h_op(h_op), .h_lba(h_lba), .h_lba_end(h_lba_end), .h_busy(h_busy), .h_done(h_done), .h_err(h_err),
	.tray(img_mounted), .drive_en(1'b1),
	.toc_addr(toc_raddr), .toc_data(toc_rdata), .toc_last(toc_last), .toc_mode2(toc_mode2),
	.sq_status(sq_status), .sq_ctrl(sq_ctrl), .sq_track(sq_track), .sq_index(sq_index),
	.sq_abs_m(sq_abs_m), .sq_abs_s(sq_abs_s), .sq_abs_f(sq_abs_f),
	.sq_rel_m(sq_rel_m), .sq_rel_s(sq_rel_s), .sq_rel_f(sq_rel_f),
	.ss_cs(ss_cs_mpu), .ss_wr(ss_wr), .ss_a(ss_a[5:0]), .ss_din(ss_din), .ss_dout(ss_dout_mpu), .ss_quiet(ss_quiet)
);

hps_cd_host #(.CLK_RATE(CLK_RATE)) host
(
	.clk(clk), .reset(mpu_reset),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
	.img_present(img_present),
	.req_lba(blk_lba), .req_bank(blk_bank), .req_rd(blk_rd), .blk_done(blk_done), .blk_err(blk_err),
	.buf_addr(buf_addr), .buf_dout(buf_dout),
	.req(h_req), .req_op(h_op), .h_lba(h_lba), .h_lba_end(h_lba_end),
	.busy(h_busy), .done(h_done), .err(h_err),
	.toc_addr(toc_raddr), .toc_data(toc_rdata), .toc_first(toc_first), .toc_last(toc_last), .toc_mode2(toc_mode2),
	.sec_we(sec_we), .sec_addr(sec_addr), .sec_data(sec_data), .sub_we(sub_we), .sub_data(sub_data),
	.sq_status(sq_status), .sq_ctrl(sq_ctrl), .sq_track(sq_track), .sq_index(sq_index),
	.sq_abs_m(sq_abs_m), .sq_abs_s(sq_abs_s), .sq_abs_f(sq_abs_f),
	.sq_rel_m(sq_rel_m), .sq_rel_s(sq_rel_s), .sq_rel_f(sq_rel_f),
	.cdda_ce(cdda_ce), .cdda_l(cdda_l), .cdda_r(cdda_r), .hold(ss_hold),
	.ss_cs(ss_cs_host), .ss_wr(ss_wr), .ss_a(ss_a[3:0]), .ss_din(ss_din), .ss_dout(ss_dout_host), .ss_busy(ss_busy)
);

endmodule

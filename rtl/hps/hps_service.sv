// Copyright (c) 2026 Jamie Blanks
//
// The Linux-facing half of the storage path: seven framework image slots,
// each behind one hps_blk_slot, plus the EXT_BUS request word the Marty
// service in Main_MiSTer polls for image creation.
//
//   hps_io ─┬─ S0 hps_blk_slot 2 x 2352 ─ hps_cd_host (in towns_cdrom)
//           ├─ S1 hps_blk_slot 16 KB    ─ towns_fdd (track records)
//           ├─ S2 hps_blk_slot 512      ─ towns_iccard
//           ├─ S3 hps_blk_slot 512      ─ towns_cmos_backup
//           ├─ S4 hps_blk_slot 512      ─ scsi_hdd (ID 0)
//           ├─ S5 hps_blk_slot 16 KB    ─ towns_fdd (drive 1 track records)
//           └─ S6 hps_blk_slot 512      ─ scsi_hdd (ID 1)
//   EXT_BUS ─ hps_ext ─ create-image requests

module hps_service #(parameter CLK_RATE = 57272727, parameter TIMEOUT = CLK_RATE * 2)
(
	input             clk,
	input             reset,
	inout      [35:0] EXT_BUS,

	// hps_io block port
	output     [31:0] sd_lba[7],
	output      [6:0] sd_rd,
	output      [6:0] sd_wr,
	input       [6:0] sd_ack,
	input      [13:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din[7],
	input             sd_buff_wr,
	input       [6:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,

	// image creation requests
	input       [9:0] trigger,
	input             size_code,

	// S0 CD-ROM
	output            cd_present,
	output            cd_mounted,
	input      [23:0] cd_lba,
	input             cd_bank,
	input             cd_rd,
	output            cd_done,
	output            cd_err,
	input      [12:0] cd_buf_addr,
	output      [7:0] cd_buf_dout,

	// S1 floppy
	output            fdd_present,
	output            fdd_wp,
	output            fdd_mounted,
	input       [7:0] fdd_lba,
	input             fdd_rd,
	input             fdd_wr,
	output            fdd_done,
	output            fdd_err,
	input      [13:0] fdd_buf_addr,
	input             fdd_buf_we,
	input       [7:0] fdd_buf_din,
	output      [7:0] fdd_buf_dout,

	// S2 IC card
	output            card_present,
	output     [17:0] card_blocks,
	output            card_mounted,
	input      [17:0] card_lba,
	input             card_rd,
	input             card_wr,
	output            card_done,
	output            card_err,
	input       [8:0] card_buf_addr,
	input             card_buf_we,
	input       [7:0] card_buf_din,
	output      [7:0] card_buf_dout,

	// S3 CMOS
	output            cmos_present,
	output            cmos_mounted,
	input       [3:0] cmos_lba,
	input             cmos_rd,
	input             cmos_wr,
	output            cmos_done,
	output            cmos_err,
	input       [8:0] cmos_buf_addr,
	input             cmos_buf_we,
	input       [7:0] cmos_buf_din,
	output      [7:0] cmos_buf_dout,

	// S4 SCSI disk
	output            hdd_present,
	output     [21:0] hdd_blocks,
	input      [21:0] hdd_lba,
	input             hdd_rd,
	input             hdd_wr,
	output            hdd_done,
	output            hdd_err,
	input       [8:0] hdd_buf_addr,
	input             hdd_buf_we,
	input       [7:0] hdd_buf_din,
	output      [7:0] hdd_buf_dout,

	// S5 second floppy
	output            fdd2_present,
	output            fdd2_wp,
	output            fdd2_mounted,
	input       [7:0] fdd2_lba,
	input             fdd2_rd,
	input             fdd2_wr,
	output            fdd2_done,
	output            fdd2_err,
	input      [13:0] fdd2_buf_addr,
	input             fdd2_buf_we,
	input       [7:0] fdd2_buf_din,
	output      [7:0] fdd2_buf_dout,

	// S6 second SCSI disk
	output            hdd2_present,
	output     [21:0] hdd2_blocks,
	input      [21:0] hdd2_lba,
	input             hdd2_rd,
	input             hdd2_wr,
	output            hdd2_done,
	output            hdd2_err,
	input       [8:0] hdd2_buf_addr,
	input             hdd2_buf_we,
	input       [7:0] hdd2_buf_din,
	output      [7:0] hdd2_buf_dout,

	output      [2:0] led_activity   // {hdd, cd, fdd}
);

wire cd_busy, fdd_busy, fdd2_busy, hdd_busy, hdd2_busy;
assign led_activity = {hdd_busy | hdd2_busy, cd_busy, fdd_busy | fdd2_busy};

hps_ext ext
(
	.clk_sys(clk),
	.EXT_BUS(EXT_BUS),
	.trigger(trigger),
	.size_code(size_code),
	.pending()
);

hps_blk_slot #(.BUF_AW(13), .LBA_W(24), .TIMEOUT(TIMEOUT), .BANKED(1)) cd_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[0]), .sd_rd(sd_rd[0]), .sd_wr(sd_wr[0]), .sd_ack(sd_ack[0]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[0]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[0]), .img_readonly(img_readonly), .img_size(img_size),
	.present(cd_present), .wp(), .blocks(), .mounted(cd_mounted),
	.req_lba(cd_lba), .req_bank(cd_bank), .req_rd(cd_rd), .req_wr(1'b0),
	.busy(cd_busy), .done(cd_done), .err(cd_err),
	.buf_addr(cd_buf_addr), .buf_we(1'b0), .buf_din(8'd0), .buf_dout(cd_buf_dout)
);

hps_blk_slot #(.BUF_AW(14), .LBA_W(8), .TIMEOUT(TIMEOUT)) fdd_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[1]), .sd_rd(sd_rd[1]), .sd_wr(sd_wr[1]), .sd_ack(sd_ack[1]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[1]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[1]), .img_readonly(img_readonly), .img_size(img_size),
	.present(fdd_present), .wp(fdd_wp), .blocks(), .mounted(fdd_mounted),
	.req_lba(fdd_lba), .req_bank(1'b0), .req_rd(fdd_rd), .req_wr(fdd_wr),
	.busy(fdd_busy), .done(fdd_done), .err(fdd_err),
	.buf_addr(fdd_buf_addr), .buf_we(fdd_buf_we), .buf_din(fdd_buf_din), .buf_dout(fdd_buf_dout)
);

hps_blk_slot #(.BUF_AW(9), .LBA_W(18), .TIMEOUT(TIMEOUT)) card_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[2]), .sd_rd(sd_rd[2]), .sd_wr(sd_wr[2]), .sd_ack(sd_ack[2]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[2]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[2]), .img_readonly(img_readonly), .img_size(img_size),
	.present(card_present), .wp(), .blocks(card_blocks), .mounted(card_mounted),
	.req_lba(card_lba), .req_bank(1'b0), .req_rd(card_rd), .req_wr(card_wr),
	.busy(), .done(card_done), .err(card_err),
	.buf_addr(card_buf_addr), .buf_we(card_buf_we), .buf_din(card_buf_din), .buf_dout(card_buf_dout)
);

hps_blk_slot #(.BUF_AW(9), .LBA_W(4), .TIMEOUT(TIMEOUT)) cmos_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[3]), .sd_rd(sd_rd[3]), .sd_wr(sd_wr[3]), .sd_ack(sd_ack[3]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[3]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[3]), .img_readonly(img_readonly), .img_size(img_size),
	.present(cmos_present), .wp(), .blocks(), .mounted(cmos_mounted),
	.req_lba(cmos_lba), .req_bank(1'b0), .req_rd(cmos_rd), .req_wr(cmos_wr),
	.busy(), .done(cmos_done), .err(cmos_err),
	.buf_addr(cmos_buf_addr), .buf_we(cmos_buf_we), .buf_din(cmos_buf_din), .buf_dout(cmos_buf_dout)
);

hps_blk_slot #(.BUF_AW(9), .LBA_W(22), .TIMEOUT(TIMEOUT)) hdd_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[4]), .sd_rd(sd_rd[4]), .sd_wr(sd_wr[4]), .sd_ack(sd_ack[4]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[4]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[4]), .img_readonly(img_readonly), .img_size(img_size),
	.present(hdd_present), .wp(), .blocks(hdd_blocks), .mounted(),
	.req_lba(hdd_lba), .req_bank(1'b0), .req_rd(hdd_rd), .req_wr(hdd_wr),
	.busy(hdd_busy), .done(hdd_done), .err(hdd_err),
	.buf_addr(hdd_buf_addr), .buf_we(hdd_buf_we), .buf_din(hdd_buf_din), .buf_dout(hdd_buf_dout)
);

hps_blk_slot #(.BUF_AW(14), .LBA_W(8), .TIMEOUT(TIMEOUT)) fdd2_slot
(
	.clk(clk), .reset(reset),
	.sd_lba(sd_lba[5]), .sd_rd(sd_rd[5]), .sd_wr(sd_wr[5]), .sd_ack(sd_ack[5]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[5]), .sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted[5]), .img_readonly(img_readonly), .img_size(img_size),
	.present(fdd2_present), .wp(fdd2_wp), .blocks(), .mounted(fdd2_mounted),
	.req_lba(fdd2_lba), .req_bank(1'b0), .req_rd(fdd2_rd), .req_wr(fdd2_wr),
	.busy(fdd2_busy), .done(fdd2_done), .err(fdd2_err),
	.buf_addr(fdd2_buf_addr), .buf_we(fdd2_buf_we), .buf_din(fdd2_buf_din), .buf_dout(fdd2_buf_dout)
);

hps_blk_slot #(.BUF_AW(9), .LBA_W(22), .TIMEOUT(TIMEOUT)) hdd2_slot
(
	.clk         (clk),
	.reset       (reset),

	.sd_lba      (sd_lba[6]),
	.sd_rd       (sd_rd[6]),
	.sd_wr       (sd_wr[6]),
	.sd_ack      (sd_ack[6]),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din (sd_buff_din[6]),
	.sd_buff_wr  (sd_buff_wr),
	.img_mounted (img_mounted[6]),
	.img_readonly(img_readonly),
	.img_size    (img_size),

	.present     (hdd2_present),
	.wp          (),
	.blocks      (hdd2_blocks),
	.mounted     (),
	.req_lba     (hdd2_lba),
	.req_bank    (1'b0),
	.req_rd      (hdd2_rd),
	.req_wr      (hdd2_wr),
	.busy        (hdd2_busy),
	.done        (hdd2_done),
	.err         (hdd2_err),
	.buf_addr    (hdd2_buf_addr),
	.buf_we      (hdd2_buf_we),
	.buf_din     (hdd2_buf_din),
	.buf_dout    (hdd2_buf_dout)
);

endmodule

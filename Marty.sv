//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Ports this core does not use yet /////////

assign ADC_BUS  = 'Z;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_SL = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

assign LED_USER  = hps_led[2] | hps_led[0];
assign LED_DISK  = {1'b1, hps_led[1]};
assign LED_POWER = {1'b1, retire_cnt[20]};
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

wire  [1:0] ar = status[122:121];
wire  [1:0] sys_speed    = status[50:49];         // Original, Plus, Great Scott
wire        cache_forced = sys_speed != 2'd0;     // Plus and up: the cache and no SDRAM waits
wire [11:0] crt_arx, crt_ary;

assign VIDEO_ARX = (ar == 2'd0) ? {1'b0, crt_arx} : {11'd0, ar - 2'd1};
assign VIDEO_ARY = (ar == 2'd0) ? {1'b0, crt_ary} : 13'd0;

`include "build_id.v"
// The core name selects the Marty service in Main_MiSTer. S0 is the
// CD-ROM (raw sectors), S1 the floppy (track records from a D88 or a raw
// image), S2 the IC card, S3 the CMOS image Linux mounts by itself from
// saves/Marty, S4 the SCSI card's hard disk (the card is fitted while a
// disk is mounted), S5 the second floppy of the two-drive mod (shown
// only with FDD Count 2), S6 the card's second hard disk. The triggers ask Linux for a blank image or an
// eject.
// Linux sends games/Marty/mrom.m36 and mrom.m37 by itself at start; the
// ROM lines (H2) show only while one is missing. An MGL cannot target a
// hidden line, so MGLs carry no ROM entries.
localparam CONF_STR = {
	"Marty;UART115200,MIDI,SS38000000:1000000;",
	"H2-YOU MUST PUT THE MASK ROMS IN GAMES;",
	"H2F1,M36,Mask ROM mrom.m36;",
	"H2F2,M37,Mask ROM mrom.m37;",
	"-;",
	"S0,CUECHDISO,CD-ROM:;",
	"r[36],Eject CD;",
	"-;",
	"S1,D88D77BINHDM,Floppy 1:;",
	"r[37],Eject floppy 1;",
	"h3S5,D88D77BINHDM,Floppy 2:;",
	"h3r[39],Eject floppy 2;",
	"-;",
	"SC2,ICM,IC Card:;",
	"SC4,VHDHDDH0 ,Hard Disk 1:;",
	"r[38],Eject hard disk 1;",
	"SC6,VHDHDDH1 ,Hard Disk 2:;",
	"r[53],Eject hard disk 2;",
	"-;",
	"P2,New blank image;",
	"P2-;",
	"P2r[33],New floppy 1 (1232K);",
	"h3P2r[48],New floppy 2 (1232K);",
	"P2r[34],New IC card (4M);",
	"P2O[26],Automount IC with CD,Yes,No;",
	"P2O[52],Automount FD with CD,Yes,No;",
	"P2-;",
	"P2O[30],Hard disk size,40M,80M;",
	"P2r[35],New hard disk 1;",
	"P2r[54],New hard disk 2;",
	"P3,Video & Audio;",
	"P3-;",
	"P3O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P3O[2:1],Video output,Real Marty,240p,480p;",
	"P3O[24],Drive lights,On,Off;",
	"P3O[3],Blanking area,Draw black,Trim;",
	"P4,System Options;",
	"P4-;",
	"P4O[5],Use Settings Database,Yes,No;",
	"P4O[51],Reset on CD Change,Yes,No;",
	"P4-;",
	"P4O[12:10],Pad 1,Towns pad,6-button pad,Mouse,Analog stick,Analog pad,None,Capcom 6-button,Marty pad;",
	"P4O[46:44],Pad 2,Towns pad,6-button pad,Mouse,Analog stick,Analog pad,None,Capcom 6-button,Marty pad;",
	"P4O[47],Swap controllers,Off,On;",
	"P4-;",
	"P4O[27],Machine ID,FM Towns,Marty;",
	"P4-;",
	"P4O[50:49],System Speed,Original,Plus,Great Scott;",
	"P4O[9:8],Main RAM,2 MB,4 MB,6 MB,8 MB(Unstable!!);",
	"P4O[31],FDD Count,1 (stock),2 (mod);",
	"P4O[23],MIDI card (FMT-40x),Off,On;",
	"P4O[25],RS-232C modem,Off,On;",
	"h1P1,MT32-pi;",
	"h1P1-;",
	"h1P1O[13],Use MT32-pi,Yes,No;",
	"h1P1O[15:14],Show Info,No,Yes,LCD-On(non-FB),LCD-Auto(non-FB);",
	"h1P1-;",
	"h1P1-,Default Config:;",
	"h1P1O[16],Synth,Munt,FluidSynth;",
	"h1P1O[18:17],Munt ROM,MT-32 v1,MT-32 v2,CM-32L;",
	"h1P1O[21:19],SoundFont,0,1,2,3,4,5,6,7;",
	"h1P1-;",
	"h1P1r[22],Reset Hanging Notes;",
	"P5,Savestates;",
	"P5-;",
	"P5O[41:40],Slot,1,2,3,4;",
	"P5r[42],Save state;",
	"P5r[43],Load state;",
	"-;",
	"R[0],Reset;",
	"I,",
	"Power off,",
	"MT32-pi: SoundFont #0,",
	"MT32-pi: SoundFont #1,",
	"MT32-pi: SoundFont #2,",
	"MT32-pi: SoundFont #3,",
	"MT32-pi: SoundFont #4,",
	"MT32-pi: SoundFont #5,",
	"MT32-pi: SoundFont #6,",
	"MT32-pi: SoundFont #7,",
	"MT32-pi: MT-32 v1,",
	"MT32-pi: MT-32 v2,",
	"MT32-pi: CM-32L,",
	"MT32-pi: Unknown mode,",
	"No IC card: mount or create one,",
	"IC card not answering,",
	"CMOS image not answering,",
	"State saved,",
	"State loaded,",
	"No state to load;",
	"J1,A,B,C,X,Y,Z,Run,Select,Zoom,Keyboard;",
	"jn,A,B,R,X,Y,L,Start,Select,L2,R2;",
	"v,1;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [21:0] gamma_bus;
wire   [1:0] buttons;
wire [127:0] status;
wire  [10:0] ps2_key;
wire  [24:0] ps2_mouse;
wire  [31:0] joystick_0, joystick_1;
wire  [15:0] joystick_l_analog_0, joystick_l_analog_1, joystick_r_analog_0, joystick_r_analog_1;
wire  [35:0] EXT_BUS;
wire         ioctl_download, ioctl_wr, ioctl_wait;
wire  [15:0] ioctl_index;
wire  [26:0] ioctl_addr;
wire   [7:0] ioctl_dout;
wire   [7:0] uart_mode;
wire  [31:0] uart_speed;
wire  [64:0] rtc;
wire  [31:0] sd_lba[7];
wire   [6:0] sd_rd, sd_wr, sd_ack, img_mounted;
wire  [13:0] sd_buff_addr;
wire   [7:0] sd_buff_dout;
wire   [7:0] sd_buff_din[7];
wire         sd_buff_wr, img_readonly;
wire  [63:0] img_size;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(7)) hps_io
(
	.clk_sys             (clk_sys),
	.HPS_BUS             (HPS_BUS),
	.EXT_BUS             (EXT_BUS),
	.gamma_bus           (gamma_bus),

	.forced_scandoubler  (forced_scandoubler),

	.buttons             (buttons),
	.status              (status),
	.info_req            (info_req),
	.info                (info_n),
	.status_menumask     ({12'd0, status[31], roms_in, mt32_available, 1'b0}),
	.uart_mode           (uart_mode),
	.uart_speed          (uart_speed),

	.ioctl_download      (ioctl_download),
	.ioctl_index         (ioctl_index),
	.ioctl_wr            (ioctl_wr),
	.ioctl_addr          (ioctl_addr),
	.ioctl_dout          (ioctl_dout),
	.ioctl_wait          (ioctl_wait & rom_download),   // only a ROM goes through the memory port

	.sd_lba              (sd_lba),
	.sd_blk_cnt          ('{0, 0, 0, 0, 0, 0, 0}),
	.sd_rd               (sd_rd),
	.sd_wr               (sd_wr),
	.sd_ack              (sd_ack),
	.sd_buff_addr        (sd_buff_addr),
	.sd_buff_dout        (sd_buff_dout),
	.sd_buff_din         (sd_buff_din),
	.sd_buff_wr          (sd_buff_wr),
	.img_mounted         (img_mounted),
	.img_readonly        (img_readonly),
	.img_size            (img_size),
	.RTC                 (rtc),

	.joystick_0          (joystick_0),
	.joystick_1          (joystick_1),
	.joystick_l_analog_0 (joystick_l_analog_0),
	.joystick_l_analog_1 (joystick_l_analog_1),
	.joystick_r_analog_0 (joystick_r_analog_0),
	.joystick_r_analog_1 (joystick_r_analog_1),
	.ps2_key             (ps2_key),
	.ps2_mouse           (ps2_mouse)
);

///////////////////////   CLOCKS   ///////////////////////////////

localparam CLK_SYS_HZ   = 57272727;
localparam CLK_SDRAM_HZ = 114545455;

wire clk_sys, clk_sdram, pll_locked;
pll pll
(
	.refclk   (CLK_50M),
	.rst      (1'b0),
	.outclk_0 (clk_sys),
	.outclk_1 (clk_sdram),
	.locked   (pll_locked)
);

// Linux holds status[0] while it mounts images and sends the clock; the
// machine stays in reset until that first pulse has ended. A ROM download
// resets it too; the CD table of contents (index 250) does not, and it
// must not be held back by the memory port either: with the CPU running,
// a stalled fast download drops bytes.
// The machine also waits for the SDRAM to initialise and for both mask
// ROMs to have been loaded; a Marty never runs without its ROM.
// Moving into or out of 8 MB changes the memory map, so it resets.
reg reset, init_done, old_rst, roms_q;
reg ram8_q;
wire ram8 = status[9:8] == 2'd3;
wire rom_download = ioctl_download && ioctl_index != 16'd250;
wire sdram_ready, verify_busy, verify_done;
wire [29:0] verify_words;
wire [7:0] dbg_port0;
wire [31:0] verify_sum;
wire roms_in;                                  // both mask ROMs fully loaded
wire verify_start = roms_in & ~roms_q;         // read the ROMs back once, before the CPU runs
always @(posedge clk_sys) begin
	roms_q  <= roms_in;
	ram8_q  <= ram8;
	reset   <= RESET | status[0] | buttons[1] | ~init_done | ~pll_locked | ~sdram_ready | rom_download | ~roms_in | verify_start | verify_busy |
	           (ram8 ^ ram8_q);
	old_rst <= status[0];
	if (RESET) init_done <= 0;
	else if (old_rst & ~status[0]) init_done <= 1;
end

///////////////////////   HPS SERVICE   //////////////////////////
// Images come through the framework's block port, one slot each; the
// clock comes from the framework RTC word, replayed to the machine once
// its own reset ends.
wire  [2:0] hps_led;
wire        cd_present, cd_mounted, cd_bank, cd_rd, cd_done, cd_err;
wire [23:0] cd_lba;
wire [12:0] cd_buf_addr;
wire  [7:0] cd_buf_dout;
wire        fdd_present, fdd_wp, fdd_mounted, fdd_rd, fdd_wr, fdd_done, fdd_err, fdd_buf_we;
wire  [7:0] fdd_lba;
wire [13:0] fdd_buf_addr;
wire  [7:0] fdd_buf_din, fdd_buf_dout;
wire        fdd2_present, fdd2_wp, fdd2_mounted, fdd2_rd, fdd2_wr, fdd2_done, fdd2_err, fdd2_buf_we;
wire  [7:0] fdd2_lba;
wire [13:0] fdd2_buf_addr;
wire  [7:0] fdd2_buf_din, fdd2_buf_dout;
wire        card_present, card_mounted, card_rd, card_wr, card_done, card_err, card_buf_we, card_warn;
wire [17:0] card_blocks, card_lba;
wire  [8:0] card_buf_addr;
wire  [7:0] card_buf_din, card_buf_dout;
wire        cmos_present, cmos_mounted, cmos_rd, cmos_wr, cmos_done, cmos_err, cmos_buf_we;
wire  [3:0] cmos_lba;
wire  [8:0] cmos_buf_addr;
wire  [7:0] cmos_buf_din, cmos_buf_dout;
wire        hdd_present, hdd_rd, hdd_wr, hdd_done, hdd_err, hdd_buf_we;
wire [21:0] hdd_blocks, hdd_lba;
wire  [8:0] hdd_buf_addr;
wire  [7:0] hdd_buf_din, hdd_buf_dout;
wire        hdd2_present, hdd2_rd, hdd2_wr, hdd2_done, hdd2_err, hdd2_buf_we;
wire [21:0] hdd2_blocks, hdd2_lba;
wire  [8:0] hdd2_buf_addr;
wire  [7:0] hdd2_buf_din, hdd2_buf_dout;

// requests to Linux: new floppy, new IC card, new hard disk, eject CD,
// eject floppy, eject hard disk, eject floppy 2, new floppy 2, eject hard
// disk 2, new hard disk 2; the size code carries the disk size option
wire  [9:0] new_trigger = {status[54:53], status[48], status[39], status[38:33]};
wire        new_size    = status[30];

hps_service #(.CLK_RATE(CLK_SYS_HZ)) hps_service
(
	.clk           (clk_sys),
	.reset         (RESET | ~pll_locked),
	.EXT_BUS       (EXT_BUS),

	.sd_lba        (sd_lba),
	.sd_rd         (sd_rd),
	.sd_wr         (sd_wr),
	.sd_ack        (sd_ack),
	.sd_buff_addr  (sd_buff_addr),
	.sd_buff_dout  (sd_buff_dout),
	.sd_buff_din   (sd_buff_din),
	.sd_buff_wr    (sd_buff_wr),
	.img_mounted   (img_mounted),
	.img_readonly  (img_readonly),
	.img_size      (img_size),

	.trigger       (new_trigger),
	.size_code     (new_size),

	.cd_present    (cd_present),
	.cd_mounted    (cd_mounted),
	.cd_lba        (cd_lba),
	.cd_bank       (cd_bank),
	.cd_rd         (cd_rd),
	.cd_done       (cd_done),
	.cd_err        (cd_err),
	.cd_buf_addr   (cd_buf_addr),
	.cd_buf_dout   (cd_buf_dout),

	.fdd_present   (fdd_present),
	.fdd_wp        (fdd_wp),
	.fdd_mounted   (fdd_mounted),
	.fdd_lba       (fdd_lba),
	.fdd_rd        (fdd_rd),
	.fdd_wr        (fdd_wr),
	.fdd_done      (fdd_done),
	.fdd_err       (fdd_err),
	.fdd_buf_addr  (fdd_buf_addr),
	.fdd_buf_we    (fdd_buf_we),
	.fdd_buf_din   (fdd_buf_din),
	.fdd_buf_dout  (fdd_buf_dout),

	.fdd2_present  (fdd2_present),
	.fdd2_wp       (fdd2_wp),
	.fdd2_mounted  (fdd2_mounted),
	.fdd2_lba      (fdd2_lba),
	.fdd2_rd       (fdd2_rd),
	.fdd2_wr       (fdd2_wr),
	.fdd2_done     (fdd2_done),
	.fdd2_err      (fdd2_err),
	.fdd2_buf_addr (fdd2_buf_addr),
	.fdd2_buf_we   (fdd2_buf_we),
	.fdd2_buf_din  (fdd2_buf_din),
	.fdd2_buf_dout (fdd2_buf_dout),

	.card_present  (card_present),
	.card_blocks   (card_blocks),
	.card_mounted  (card_mounted),
	.card_lba      (card_lba),
	.card_rd       (card_rd),
	.card_wr       (card_wr),
	.card_done     (card_done),
	.card_err      (card_err),
	.card_buf_addr (card_buf_addr),
	.card_buf_we   (card_buf_we),
	.card_buf_din  (card_buf_din),
	.card_buf_dout (card_buf_dout),

	.cmos_present  (cmos_present),
	.cmos_mounted  (cmos_mounted),
	.cmos_lba      (cmos_lba),
	.cmos_rd       (cmos_rd),
	.cmos_wr       (cmos_wr),
	.cmos_done     (cmos_done),
	.cmos_err      (cmos_err),
	.cmos_buf_addr (cmos_buf_addr),
	.cmos_buf_we   (cmos_buf_we),
	.cmos_buf_din  (cmos_buf_din),
	.cmos_buf_dout (cmos_buf_dout),

	.hdd_present   (hdd_present),
	.hdd_blocks    (hdd_blocks),
	.hdd_lba       (hdd_lba),
	.hdd_rd        (hdd_rd),
	.hdd_wr        (hdd_wr),
	.hdd_done      (hdd_done),
	.hdd_err       (hdd_err),
	.hdd_buf_addr  (hdd_buf_addr),
	.hdd_buf_we    (hdd_buf_we),
	.hdd_buf_din   (hdd_buf_din),
	.hdd_buf_dout  (hdd_buf_dout),

	.hdd2_present  (hdd2_present),
	.hdd2_blocks   (hdd2_blocks),
	.hdd2_lba      (hdd2_lba),
	.hdd2_rd       (hdd2_rd),
	.hdd2_wr       (hdd2_wr),
	.hdd2_done     (hdd2_done),
	.hdd2_err      (hdd2_err),
	.hdd2_buf_addr (hdd2_buf_addr),
	.hdd2_buf_we   (hdd2_buf_we),
	.hdd2_buf_din  (hdd2_buf_din),
	.hdd2_buf_dout (hdd2_buf_dout),

	.led_activity  (hps_led)
);

// the RTC word: BCD sec, min, hour, day, month, year; weekday 0 = Sunday
reg  rtc_q;
wire rtc_seed_valid = rtc[64] ^ rtc_q;
always @(posedge clk_sys) rtc_q <= rtc[64];

reg seed_pending, reset_q, seed_go;
always @(posedge clk_sys) begin
	reset_q <= reset;
	seed_go <= rtc_seed_valid & ~reset;
	if (rtc_seed_valid & reset) seed_pending <= 1;
	if (reset_q && !reset && seed_pending) begin seed_go <= 1; seed_pending <= 0; end
end

///////////////////////  MAINBOARD  /////////////////////////////
// CPU clock as an enable: one pulse per T-state by a fractional divider,
// 16 MHz (3 or 4 clk_sys apart), or 25 MHz (2 or 3) at Great Scott.
// The peripherals keep their own 16 MHz, so only the CPU speeds up.
wire        cpu_fast     = sys_speed[1];
wire [31:0] cpu_hz       = cpu_fast ? 32'd25000000 : 32'd16000000;

reg        ce_tick;
reg [31:0] ce_cpu_acc;
always @(posedge clk_sys) begin
	ce_tick <= 0;
	if (reset) ce_cpu_acc <= 0;
	else if (ce_cpu_acc + cpu_hz >= CLK_SYS_HZ) begin
		ce_cpu_acc <= ce_cpu_acc + cpu_hz - CLK_SYS_HZ;
		ce_tick    <= 1;
	end
	else ce_cpu_acc <= ce_cpu_acc + cpu_hz;
end
// a savestate holds the enables outside the CPU's through the mainboard
wire ss_run;
wire ce_cpu = ce_tick;

wire [23:1] mem_a;
wire  [1:0] mem_be;
wire        mem_we, mem_req, mem_ready;
wire [15:0] mem_din, mem_dout;
wire [18:3] vf_a;
wire        vf_req, vf_accept, vf_ready;
wire [63:0] vf_dout;
wire [18:1] vr_a;
wire  [1:0] vr_be;
wire        vr_we, vr_req, vr_ready;
wire [15:0] vr_din, vr_dout;
wire        ce_pix, vid_hs, vid_vs, vid_hb, vid_vb, vid_field;
wire  [7:0] vid_r, vid_g, vid_b;
wire        cpu_retire;
wire        power_off;
wire [15:0] audio_l, audio_r;

// OSD pad choice: Towns pad, 6-button pad, mouse, analog stick, analog pad, none, Capcom 6-button,
// Marty pad. The port numbers the Towns pad 7 and the Marty pad 0, so those two swap.
wire  [2:0] pad1_type = status[12:10] == 3'd0 ? 3'd7 : status[12:10] == 3'd7 ? 3'd0 : status[12:10];
wire  [2:0] pad2_type = status[46:44] == 3'd0 ? 3'd7 : status[46:44] == 3'd7 ? 3'd0 : status[46:44];

// Swap sends controller 1 (with its analog axes) to port 2 and controller 2 to port 1
wire        swap_pads = status[47];
wire [31:0] joy_p1 = swap_pads ? joystick_1 : joystick_0;
wire [31:0] joy_p2 = swap_pads ? joystick_0 : joystick_1;
wire [15:0] stick_p1 = swap_pads ? joystick_l_analog_1 : joystick_l_analog_0;
wire [15:0] stick_p2 = swap_pads ? joystick_l_analog_0 : joystick_l_analog_1;
wire  [7:0] throttle_p1 = swap_pads ? joystick_r_analog_1[15:8] : joystick_r_analog_0[15:8];
wire  [7:0] throttle_p2 = swap_pads ? joystick_r_analog_0[15:8] : joystick_r_analog_1[15:8];

// framework pad bits to the connector's view: {Z, Y, X, C, zoom, select, run, B, A, up, down, left, right}
wire [12:0] pad1_raw = {joy_p1[9:6], joy_p1[12:10], joy_p1[5:0]};
wire [12:0] pad2_raw = {joy_p2[9:6], joy_p2[12:10], joy_p2[5:0]};
wire [12:0] pad1, pad2;
wire [10:0] osk_key;

// A changed CMOS goes back to the backup image when the menu opens, on a
// software reset and on power-off.
reg  osd_q, osd_opened;
wire soft_reset_req;
always @(posedge clk_sys) begin
	osd_q      <= OSD_STATUS;
	osd_opened <= OSD_STATUS & ~osd_q;
end

// OSD notices: 1 power off (the register has no switch to open here),
// 2-13 the MT32-pi mode names, 14 card used with no image, 15 the card
// service timed out (the SCSI disk reports its own errors to the guest
// as a sense key), 16 the CMOS image timed out.
reg         power_off_q, info_req;
reg   [7:0] info_n;
wire        bk_fail   = card_done & card_err;
wire        cmos_fail = cmos_done & cmos_err;
wire        ss_event;
wire  [1:0] ss_code;
always @(posedge clk_sys) begin
	power_off_q <= power_off;
	info_req    <= (power_off & ~power_off_q) | mt32_info_req | card_warn | bk_fail | cmos_fail | ss_event;
	info_n      <= (power_off & ~power_off_q) ? 8'd1 : card_warn ? 8'd14 : bk_fail ? 8'd15 : cmos_fail ? 8'd16 :
		ss_event ? 8'd16 + {6'd0, ss_code} : {4'd0, mt32_info_disp};
end

marty_mainboard #(.CLK_HZ(CLK_SYS_HZ)) mainboard
(
	.clk             (clk_sys),
	.ce_cpu          (ce_cpu),
	.run             (ss_run),
	.ss_stop         (ss_stop),
	.ss_quiet        (ss_quiet),
	.ss_in_hlt       (ss_in_hlt),
	.ss_state_sel    (ss_state_sel),
	.ss_state        (ss_state),
	.ss_mode         (ss_mode),
	.ss_cpu_reset    (ss_cpu_reset),
	.ss_win_we       (ss_win_we),
	.ss_win_addr     (ss_win_addr),
	.ss_win_data     (ss_win_data),
	.ss_retire       (ss_retire),
	.ss_dev_busy     (ss_dev_busy),
	.ss_bus_req      (ss_bus_req),
	.ss_bus_we       (ss_bus_we),
	.ss_bus_io       (ss_bus_io),
	.ss_bus_hidden   (ss_bus_hidden),
	.ss_bus_raw      (ss_bus_raw),
	.ss_bus_a        (ss_bus_a),
	.ss_bus_be       (ss_bus_be),
	.ss_bus_din      (ss_bus_din),
	.ss_bus_dout     (ss_bus_dout),
	.ss_bus_ack      (ss_bus_ack),
	.reset           (reset),
	.cache_auto      (1'b0),
	.cache_enable    (cache_forced),
	.mem_fast        (cache_forced),          // turbo: the cache and no SDRAM waits
	.sprite_fast     (cpu_fast),              // Great Scott: sprites at Towns II MX speed
	.ram_size        (status[9:8]),
	.towns_id        (~status[27]),   // FM Towns unless the OSD says Marty
	.video_mode      (status[2:1]),
	.show_blank      (~status[3]),

	.mem_a           (mem_a),
	.mem_be          (mem_be),
	.mem_we          (mem_we),
	.mem_din         (mem_din),
	.mem_dout        (mem_dout),
	.mem_req         (mem_req),
	.mem_ready       (mem_ready),

	.vf_a            (vf_a),
	.vf_req          (vf_req),
	.vf_accept       (vf_accept),
	.vf_dout         (vf_dout),
	.vf_ready        (vf_ready),
	.vr_a            (vr_a),
	.vr_be           (vr_be),
	.vr_we           (vr_we),
	.vr_din          (vr_din),
	.vr_dout         (vr_dout),
	.vr_req          (vr_req),
	.vr_ready        (vr_ready & ~vram_wr_full),

	.ce_pix          (ce_pix),
	.vid_r           (vid_r),
	.vid_g           (vid_g),
	.vid_b           (vid_b),
	.vid_hs          (vid_hs),
	.vid_vs          (vid_vs),
	.vid_hb          (vid_hb),
	.vid_vb          (vid_vb),
	.vid_field       (vid_field),

	.power_off       (power_off),
	.soft_reset_req  (soft_reset_req),
	.beep_out        (),
	.ps2_key         (osk_key),
	.ps2_key_raw     (ps2_key),
	.ps2_mouse       (ps2_mouse),
	.pad1            (pad1),
	.pad2            (pad2),
	.pad1_type       (pad1_type),
	.pad2_type       (pad2_type),
	.pad1_stick      (stick_p1),
	.pad2_stick      (stick_p2),
	.pad1_throttle   (throttle_p1),
	.pad2_throttle   (throttle_p2),
	.audio_l         (audio_l),
	.audio_r         (audio_r),
	.midi_en         (status[23]),
	.midi_rx         (midi_rx),
	.midi_tx         (midi_tx),
	.rs_en           (status[25]),
	.rs_baud         (uart_speed),
	.rs_txd          (rs_txd),
	.rs_rxd          (rs_rxd),
	.rs_rts          (rs_rts),
	.rs_dtr          (rs_dtr),
	.rs_cts          (hps_serial & UART_CTS),
	.rs_dsr          (hps_serial & UART_DSR),
	.rs_cd           (hps_serial & UART_DSR),
	.fdd_present     (fdd_present),
	.fdd_wp          (fdd_wp),
	.fdd_mounted     (fdd_mounted),
	.fdd_lba         (fdd_lba),
	.fdd_rd          (fdd_rd),
	.fdd_wr          (fdd_wr),
	.fdd_done        (fdd_done),
	.fdd_err         (fdd_err),
	.fdd_buf_addr    (fdd_buf_addr),
	.fdd_buf_we      (fdd_buf_we),
	.fdd_buf_din     (fdd_buf_din),
	.fdd_buf_dout    (fdd_buf_dout),
	.two_drives      (status[31]),
	.fdd2_present    (fdd2_present),
	.fdd2_wp         (fdd2_wp),
	.fdd2_mounted    (fdd2_mounted),
	.fdd2_lba        (fdd2_lba),
	.fdd2_rd         (fdd2_rd),
	.fdd2_wr         (fdd2_wr),
	.fdd2_done       (fdd2_done),
	.fdd2_err        (fdd2_err),
	.fdd2_buf_addr   (fdd2_buf_addr),
	.fdd2_buf_we     (fdd2_buf_we),
	.fdd2_buf_din    (fdd2_buf_din),
	.fdd2_buf_dout   (fdd2_buf_dout),

	.card_present    (card_present),
	.card_blocks     (card_blocks),
	.card_lba        (card_lba),
	.card_rd         (card_rd),
	.card_wr         (card_wr),
	.card_done       (card_done),
	.card_err        (card_err),
	.card_buf_addr   (card_buf_addr),
	.card_buf_we     (card_buf_we),
	.card_buf_din    (card_buf_din),
	.card_buf_dout   (card_buf_dout),
	.card_warn       (card_warn),

	.cmos_present    (cmos_present),
	.cmos_mounted    (cmos_mounted),
	.cmos_lba        (cmos_lba),
	.cmos_rd         (cmos_rd),
	.cmos_wr         (cmos_wr),
	.cmos_done       (cmos_done),
	.cmos_err        (cmos_err),
	.cmos_buf_addr   (cmos_buf_addr),
	.cmos_buf_we     (cmos_buf_we),
	.cmos_buf_din    (cmos_buf_din),
	.cmos_buf_dout   (cmos_buf_dout),
	.cmos_save       (osd_opened | soft_reset_req | (power_off & ~power_off_q)),

	.hdd_present     (hdd_present),
	.hdd_blocks      (hdd_blocks),
	.hdd_lba         (hdd_lba),
	.hdd_rd          (hdd_rd),
	.hdd_wr          (hdd_wr),
	.hdd_done        (hdd_done),
	.hdd_err         (hdd_err),
	.hdd_buf_addr    (hdd_buf_addr),
	.hdd_buf_we      (hdd_buf_we),
	.hdd_buf_din     (hdd_buf_din),
	.hdd_buf_dout    (hdd_buf_dout),

	.hdd2_present    (hdd2_present),
	.hdd2_blocks     (hdd2_blocks),
	.hdd2_lba        (hdd2_lba),
	.hdd2_rd         (hdd2_rd),
	.hdd2_wr         (hdd2_wr),
	.hdd2_done       (hdd2_done),
	.hdd2_err        (hdd2_err),
	.hdd2_buf_addr   (hdd2_buf_addr),
	.hdd2_buf_we     (hdd2_buf_we),
	.hdd2_buf_din    (hdd2_buf_din),
	.hdd2_buf_dout   (hdd2_buf_dout),

	.ioctl_download  (ioctl_download),
	.ioctl_index     (ioctl_index),
	.ioctl_wr        (ioctl_wr),
	.ioctl_addr      (ioctl_addr[9:0]),
	.ioctl_dout      (ioctl_dout),
	.cd_present      (cd_present),
	.cd_mounted      (cd_mounted),
	.cd_lba          (cd_lba),
	.cd_bank         (cd_bank),
	.cd_rd           (cd_rd),
	.cd_done         (cd_done),
	.cd_err          (cd_err),
	.cd_buf_addr     (cd_buf_addr),
	.cd_buf_dout     (cd_buf_dout),

	.rtc_seed_valid  (seed_go),
	.rtc_seed_sec    (rtc[6:0]),
	.rtc_seed_min    (rtc[14:8]),
	.rtc_seed_hour   (rtc[21:16]),
	.rtc_seed_wday   (rtc[50:48] + 3'd1),
	.rtc_seed_day    (rtc[29:24]),
	.rtc_seed_month  (rtc[36:32]),
	.rtc_seed_year   (rtc[47:40]),

	.dbg_io_unmapped (dbg_io_unmapped),
	.dbg_io_wr       (dbg_io_wr),
	.dbg_io_rd       (dbg_io_rd),
	.dbg_io_addr     (dbg_io_addr),
	.dbg_io_data     (dbg_io_data),
	.dbg_spr_we      (),
	.dbg_spr_a       (),
	.dbg_spr_be      (),
	.dbg_spr_din     (),
	.dbg_bm_en       (1'b0),
	.dbg_bm_addr     (11'd0),
	.dbg_bm_q        (),
	.dbg_CS          (dbg_CS),
	.dbg_EIP         (dbg_EIP),
	.dbg_retire      (cpu_retire),
	.dbg_dma_bus     (),
	.dbg_crtc_regs   (dbg_crtc_regs),
	.dbg_gpr         (),
	.dbg_seg         ()
);

towns_memory #(.CLK_SDRAM_HZ(CLK_SDRAM_HZ), .ROM_M36_INDEX(1), .ROM_M37_INDEX(2)) memory
(
	.clk_sys        (clk_sys),
	.clk_sdram      (clk_sdram),
	.reset          (RESET | ~pll_locked),

	.mem_a          (mem_a),
	.mem_be         (mem_be),
	.mem_we         (mem_we),
	.mem_din        (mem_din),
	.mem_dout       (mem_dout),
	.mem_req        (mem_req),
	.mem_ready      (mem_ready),

	.vr_a           (vr_a),
	.vr_be          (vr_be),
	.vr_we          (vr_we),
	.vr_din         (vr_din),
	.vr_dout        (vr_dout),
	.vr_req         (vr_req),
	.vr_ready       (vr_ready),

	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.dos_two_drives (status[31]),
	.ioctl_wait     (ioctl_wait),
	.initialised    (sdram_ready),
	.dbg_port0      (dbg_port0),
	.roms_loaded    (roms_in),
	.verify_start   (verify_start),
	.verify_busy    (verify_busy),
	.verify_sum     (verify_sum),
	.verify_done    (verify_done),
	.verify_words   (verify_words),

	.SDRAM_CLK      (SDRAM_CLK),
	.SDRAM_CKE      (SDRAM_CKE),
	.SDRAM_A        (SDRAM_A),
	.SDRAM_BA       (SDRAM_BA),
	.SDRAM_DQ       (SDRAM_DQ),
	.SDRAM_DQML     (SDRAM_DQML),
	.SDRAM_DQMH     (SDRAM_DQMH),
	.SDRAM_nCS      (SDRAM_nCS),
	.SDRAM_nCAS     (SDRAM_nCAS),
	.SDRAM_nRAS     (SDRAM_nRAS),
	.SDRAM_nWE      (SDRAM_nWE)
);

// Heartbeat: the power LED toggles every 2^20 completed instructions.
reg [20:0] retire_cnt;
always @(posedge clk_sys) if (cpu_retire && ce_cpu) retire_cnt <= retire_cnt + 1'd1;

wire        dbg_io_unmapped, dbg_io_wr, dbg_io_rd;
wire [15:0] dbg_io_addr, dbg_CS;
wire  [7:0] dbg_io_data;
wire [31:0] dbg_EIP;
wire [127:0] dbg_crtc_regs;

assign DDRAM_CLK = clk_sys;

///////////////////////   SAVESTATES   ///////////////////////////
// Four 16 MB slots at 0x38000000, the framework's SS window.
wire        ss_busy, ss_req, ss_we, ss_done;
wire        ss_stop, ss_quiet, ss_in_hlt, ss_mode, ss_cpu_reset, ss_win_we, ss_retire;
wire        ss_bus_req, ss_bus_we, ss_bus_io, ss_bus_hidden, ss_bus_raw, ss_bus_ack, ss_dev_busy;
wire [23:1] ss_bus_a;
wire  [1:0] ss_bus_be;
wire [15:0] ss_bus_din, ss_bus_dout;
wire  [5:0] ss_state_sel;
wire [31:0] ss_state, ss_win_data;
wire  [9:0] ss_win_addr;
wire [28:0] ss_addr;
wire [63:0] ss_wdata, ss_rdata;
wire        ss_ddr_we, ss_ddr_rd, ss_ddr_busy, ss_ddr_dout_ready;
wire [28:0] ss_ddr_addr;
wire [63:0] ss_ddr_din;
wire  [7:0] ss_ddr_be;
reg   [1:0] ss_key_q;
always @(posedge clk_sys) ss_key_q <= status[43:42];

ss_engine ss_engine
(
	.clk           (clk_sys),
	.reset         (reset),
	.save_req      (status[42] & ~ss_key_q[0]),
	.load_req      (status[43] & ~ss_key_q[1]),
	.slot          (status[41:40]),
	.ram_size      (status[9:8]),
	.rom_sum       (verify_sum),
	.run           (ss_run),
	.busy          (ss_busy),
	.event_code    (ss_code),
	.event_req     (ss_event),
	.ss_stop       (ss_stop),
	.ss_quiet      (ss_quiet),
	.ss_in_hlt     (ss_in_hlt),
	.ss_state_sel  (ss_state_sel),
	.ss_state      (ss_state),
	.ss_mode       (ss_mode),
	.ss_cpu_reset  (ss_cpu_reset),
	.ss_win_we     (ss_win_we),
	.ss_win_addr   (ss_win_addr),
	.ss_win_data   (ss_win_data),
	.ss_retire     (ss_retire),
	.ss_dev_busy   (ss_dev_busy),
	.ss_bus_req    (ss_bus_req),
	.ss_bus_we     (ss_bus_we),
	.ss_bus_io     (ss_bus_io),
	.ss_bus_hidden (ss_bus_hidden),
	.ss_bus_raw    (ss_bus_raw),
	.ss_bus_a      (ss_bus_a),
	.ss_bus_be     (ss_bus_be),
	.ss_bus_din    (ss_bus_din),
	.ss_bus_dout   (ss_bus_dout),
	.ss_bus_ack    (ss_bus_ack),
	.ddr_req       (ss_req),
	.ddr_we        (ss_we),
	.ddr_addr      (ss_addr),
	.ddr_wdata     (ss_wdata),
	.ddr_rdata     (ss_rdata),
	.ddr_done      (ss_done)
);

ss_ddr ss_ddr
(
	.clk              (clk_sys),
	.reset            (reset),
	.req              (ss_req),
	.we               (ss_we),
	.addr             (ss_addr),
	.wdata            (ss_wdata),
	.be               (8'hFF),
	.rdata            (ss_rdata),
	.done             (ss_done),
	.busy             (),
	.DDRAM_BUSY       (ss_ddr_busy),
	.DDRAM_BURSTCNT   (),
	.DDRAM_ADDR       (ss_ddr_addr),
	.DDRAM_DIN        (ss_ddr_din),
	.DDRAM_BE         (ss_ddr_be),
	.DDRAM_WE         (ss_ddr_we),
	.DDRAM_RD         (ss_ddr_rd),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (ss_ddr_dout_ready)
);

///////////////////////   VRAM MIRROR   //////////////////////////
// The CRTC's line fetch reads a DDR3 copy of VRAM kept by mirroring the
// VRAM port's writes, so the SDRAM carries only the CPU and the VRAM
// port. The mirror shares the DDR3 port with the savestate engine.
wire vram_wr_full;
vram_ddr vram_ddr
(
	.clk              (clk_sys),
	.reset            (reset),
	.wr_req           (vr_req & vr_we),
	.wr_a             (vr_a),
	.wr_be            (vr_be),
	.wr_din           (vr_din),
	.wr_full          (vram_wr_full),
	.vf_req           (vf_req),
	.vf_a             (vf_a),
	.vf_accept        (vf_accept),
	.vf_dout          (vf_dout),
	.vf_ready         (vf_ready),
	.ss_we            (ss_ddr_we),
	.ss_rd            (ss_ddr_rd),
	.ss_addr          (ss_ddr_addr),
	.ss_din           (ss_ddr_din),
	.ss_be            (ss_ddr_be),
	.ss_busy          (ss_ddr_busy),
	.ss_dout_ready    (ss_ddr_dout_ready),
	.DDRAM_BUSY       (DDRAM_BUSY),
	.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
	.DDRAM_ADDR       (DDRAM_ADDR),
	.DDRAM_DIN        (DDRAM_DIN),
	.DDRAM_BE         (DDRAM_BE),
	.DDRAM_WE         (DDRAM_WE),
	.DDRAM_RD         (DDRAM_RD),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (DDRAM_DOUT_READY)
);

///////////////////////   MIDI   /////////////////////////////////

// The HPS UART carries MIDI in its MIDI mode and the RS-232C modem's
// serial line in its Modem, PPP or Console modes. The MIDI line has two
// homes: the MT32-pi on the user port and that HPS UART. A present and
// enabled pi takes the stream and the HPS side stays quiet; a present but
// disabled pi is muted instead. Input follows the same choice.
wire        midi_tx, midi_rx, pi_rx;
wire        rs_txd, rs_rxd, rs_rts, rs_dtr;
wire        hps_midi   = (uart_mode >= 8'd3);
wire        hps_serial = (uart_mode != 8'd0) && !hps_midi;

wire        mt32_disable  = status[13];
wire  [1:0] mt32_info     = status[15:14];
wire        mt32_mode_req = status[16];
wire  [1:0] mt32_rom_req  = status[18:17];
wire  [7:0] mt32_sf_req   = {5'd0, status[21:19]};
wire        mt32_reset    = status[22] | reset;

wire [15:0] mt32_i2s_r, mt32_i2s_l;
wire  [7:0] mt32_mode, mt32_rom, mt32_sf;
wire        mt32_lcd_en, mt32_lcd_pix, mt32_lcd_update;
wire        mt32_newmode, mt32_available;
wire        mt32_use  = mt32_available & ~mt32_disable;
wire        mt32_mute = mt32_available &  mt32_disable;

assign UART_TXD = hps_midi ? (midi_tx & ~mt32_use) : hps_serial ? rs_txd : 1'b1;
assign UART_RTS = hps_serial & rs_rts;
assign UART_DTR = hps_serial & rs_dtr;
assign midi_rx  = hps_midi ? UART_RXD : pi_rx;
assign rs_rxd   = ~hps_serial | UART_RXD;

mt32pi mt32pi
(
	.*,
	.reset   (mt32_reset),
	.midi_tx (midi_tx | mt32_mute),
	.midi_rx (pi_rx)
);

// An OSD notice names the synth the pi switched to; the LCD overlay
// shows its display for a while after each update or stays on.
reg        mt32_info_req;
reg  [3:0] mt32_info_disp;
always @(posedge clk_sys) begin
	reg old_mode;

	old_mode      <= mt32_newmode;
	mt32_info_req <= (old_mode ^ mt32_newmode) && (mt32_info == 2'd1);

	mt32_info_disp <= (mt32_mode == 8'hA2) ? (4'd2 + {1'b0, mt32_sf[2:0]}) :
		(mt32_mode == 8'hA1 && mt32_rom == 8'd0) ? 4'd10 :
		(mt32_mode == 8'hA1 && mt32_rom == 8'd1) ? 4'd11 :
		(mt32_mode == 8'hA1 && mt32_rom == 8'd2) ? 4'd12 : 4'd13;
end

reg mt32_lcd_on;
always @(posedge clk_sys) begin
	reg [27:0] hold;
	reg        old_update;

	old_update <= mt32_lcd_update;
	if (hold != 0) hold <= hold - 1'd1;

	if (mt32_info == 2'd2) mt32_lcd_on <= 1;
	else if (mt32_info != 2'd3) mt32_lcd_on <= 0;
	else begin
		if (hold == 0) mt32_lcd_on <= 0;
		if (old_update ^ mt32_lcd_update) begin
			mt32_lcd_on <= 1;
			hold <= 28'(CLK_SYS_HZ * 2);
		end
	end
end

wire mt32_lcd = mt32_lcd_on & mt32_lcd_en;

///////////////////////   AUDIO   ////////////////////////////////

// The pi's I2S stream joins the board mix with saturation.
wire [16:0] sum_l = {audio_l[15], audio_l} + (mt32_use ? {mt32_i2s_l[15], mt32_i2s_l} : 17'd0);
wire [16:0] sum_r = {audio_r[15], audio_r} + (mt32_use ? {mt32_i2s_r[15], mt32_i2s_r} : 17'd0);
reg  [15:0] out_l, out_r;
always @(posedge CLK_AUDIO) begin
	out_l <= (sum_l[16] ^ sum_l[15]) ? {sum_l[16], {15{~sum_l[16]}}} : sum_l[15:0];
	out_r <= (sum_r[16] ^ sum_r[15]) ? {sum_r[16], {15{~sum_r[16]}}} : sum_r[15:0];
end

assign AUDIO_L = out_l;
assign AUDIO_R = out_r;

///////////////////////   VIDEO   ////////////////////////////////

// The scan converter delivers 480i NTSC, 240p or 480p from lines it pulls
// through the CRTC; F1 carries the field for the interlace.
// The on-screen keyboard takes the pads while open, types into the
// keyboard controller's PS/2 stream and draws over the picture.
wire  [7:0] osk_r, osk_g, osk_b;
wire        osk_hs, osk_vs, osk_hb, osk_vb, osk_field;

osd_keyboard #(.CLK_HZ(CLK_SYS_HZ)) osd_keyboard
(
	.clk       (clk_sys),
	.reset     (reset),
	.toggle    (joystick_0[13] | joystick_1[13]),
	.lights_en (~status[24]),
	.activity  (hps_led),
	.pad1_i    (pad1_raw),
	.pad2_i    (pad2_raw),
	.pad1_o    (pad1),
	.pad2_o    (pad2),
	.open      (),
	.ps2_key_i (ps2_key),
	.ps2_key_o (osk_key),
	.ce_pix    (ce_pix),
	.r_i       (vid_r),
	.g_i       (vid_g),
	.b_i       (vid_b),
	.hs_i      (vid_hs),
	.vs_i      (vid_vs),
	.hb_i      (vid_hb),
	.vb_i      (vid_vb),
	.field_i   (vid_field),
	.r_o       (osk_r),
	.g_o       (osk_g),
	.b_o       (osk_b),
	.hs_o      (osk_hs),
	.vs_o      (osk_vs),
	.hb_o      (osk_hb),
	.vb_o      (osk_vb),
	.field_o   (osk_field)
);

// "Original" is the unblanked picture's shape on a 4:3 set: the NTSC
// rasters show 52.66 us by 485 lines as 4:3, the 480p raster 640 by 480
auto_crt_ar #(.A_V_VIS_NUM(8'd97), .A_V_VIS_DEN(8'd105), .B_H_VIS_NUM(8'd4), .B_H_VIS_DEN(8'd5), .B_V_VIS_NUM(8'd32), .B_V_VIS_DEN(8'd35)) crt_ar
(
	.clk    (clk_sys),
	.reset  (reset),
	.ce_pix (ce_pix),
	.use_b  (status[2:1] == 2'd2),
	.hsync  (osk_hs),
	.vsync  (osk_vs),
	.hblank (osk_hb),
	.vblank (osk_vb),
	.arx    (crt_arx),
	.ary    (crt_ary)
);

assign CLK_VIDEO = clk_sys;
assign VGA_F1 = osk_field;

// Scandoubler and HQ2x stay off: the line buffers do not fit alongside the core
video_mixer #(.LINE_LENGTH(1024), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO   (clk_sys),
	.CE_PIXEL    (CE_PIXEL),
	.ce_pix      (ce_pix),

	.scandoubler (1'b0),
	.hq2x        (1'b0),
	.gamma_bus   (gamma_bus),

	.R           (mt32_lcd ? {{2{mt32_lcd_pix}}, osk_r[7:2]} : osk_r),
	.G           (mt32_lcd ? {{2{mt32_lcd_pix}}, osk_g[7:2]} : osk_g),
	.B           (mt32_lcd ? {{2{mt32_lcd_pix}}, osk_b[7:2]} : osk_b),

	.HSync       (osk_hs),
	.VSync       (osk_vs),
	.HBlank      (osk_hb),
	.VBlank      (osk_vb),

	.HDMI_FREEZE (HDMI_FREEZE),
	.freeze_sync (),

	.VGA_R       (VGA_R),
	.VGA_G       (VGA_G),
	.VGA_B       (VGA_B),
	.VGA_VS      (VGA_VS),
	.VGA_HS      (VGA_HS),
	.VGA_DE      (VGA_DE)
);

endmodule

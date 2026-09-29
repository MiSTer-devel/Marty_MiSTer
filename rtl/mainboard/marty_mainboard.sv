// Copyright (c) 2026 Jamie Blanks
//
// The Marty mainboard: CPU, the system-controller pieces built so far and
// the bus glue between them. Memory that lives in SDRAM leaves through the
// `mem_*` port; the top wires it to the SDRAM controller and the harness
// serves it from a C++ array.
//
//   am386sx ──A/D/control──┬─ towns_memmap ──> mem_* (SDRAM: DRAM + ROMs)
//                          ├─ towns_vram_ctrl (A00000/B00000 VRAM, C0000 planes, 0458) ──> vf_*/vr_* (SDRAM: VRAM)
//                          ├─ towns_crtc      (0440-044C, FD90-FDA0, 05CA) ──> marty_video_out ──> video
//                          ├─ towns_sprite    (0450/0452, C00000 pattern RAM)
//                          ├─ towns_cmos      (D8000 window, I/O 3000-3FFF)
//                          ├─ towns_sysregs   (I/O 0020-05EC)
//                          ├─ i8259a x2       (0000/0002 master, 0010/0012 slave)
//                          ├─ i8253 x2        (0040-0046, 0050-0056)
//                          ├─ towns_intctrl   (0026/0027, 0060, 0068-006C)
//                          ├─ rtc58323        (0070 data, 0080 control)
//                          ├─ upd71071        (00A0-00AF, HOLD/HLDA bus master)
//                          ├─ mb8877a         (0200-0206, drive glue 0208-020E, IRQ6, DMA channel 0) ──> towns_fdd x2 ──> fdd_*, fdd2_* (HPS track records)
//                          ├─ towns_cmos_backup (CMOS image, first mount) ──> cmos_* (HPS blocks)
//                          ├─ towns_iccard    (048A/0490/0491, D00000 window) ──> card_* (HPS blocks)
//                          ├─ towns_scsi      (0C30-0C34, IRQ8, DMA channel 1, optional card) ─ scsi_hdd ──> hdd_* (HPS blocks)
//                          ├─ towns_keyboard  (0600-0604, IRQ1) <── ps2_key
//                          ├─ towns_padport   (04D0-04D6) <── pads, ps2_mouse
//                          ├─ towns_cdrom     (04C0-04CD, IRQ9, DMA channel 3) ──> cd_* (HPS raw sectors), cdda_* ──┐
//                          ├─ ym3438          (04D8-04DE) ──> ym3438_dac ──┐
//                          ├─ rf5c68          (04F0-04F8, F80000 wave window) ──> lc7881 ──┤
//                          ├─ towns_audio_mixer (04D5, 04E0-04E3, 04E9-04EC, IRQ13) <──────┘ ──> audio_*
//                          ├─ towns_midi      (0E50-0E55, 0E70-0E77, IRQ4/IRQ5, optional card) ──> midi_tx
//                          └─ towns_rs232     (0A00-0A0A, IRQ2, optional modem card) ──> rs_* (HPS UART)
//
// Byte devices sit on D7-D0. A 16-bit access to one runs two lanes back
// to back, low byte first. Wait counts are the floor each Marty part allows
// on the 16 MHz SX bus: 80 ns DRAM and VRAM need one (tRC 150-160 ns spans
// 2 T-states), the 150 ns mask ROM needs two. I/O is unmeasured.
// An interrupt acknowledge is a byte cycle with INTA_n low: the PICs see
// its edges and the second one returns the vector on the low lane.
// While the CPU is in hold the DMAC's strobes run memory cycles through
// the same SDRAM port; device data moves fly-by on the low lane.

module marty_mainboard #(
	parameter CMOS_INIT  = " ",   // simulation fixture
	parameter SS_WINDOW_HEX = "rtl/savestate/ss_window.hex",   // restore window image for simulation
	parameter FAST_FDD   = 0,     // simulation only: quick floppy spin-up and rotation
	parameter BOOT_WAIT  = 114545454,   // clocks to wait for the CMOS image before the CPU starts
	parameter CLK_HZ     = 57272727,
	parameter WAIT_RAM   = 1,
	parameter WAIT_ROM   = 2,
	parameter WAIT_VRAM  = 1,
	parameter WAIT_IO    = 2
)
(
	input             clk,
	input             ce_cpu,        // 16 MHz T-state enable
	input             run,           // low while a savestate holds the machine
	// savestate: park the CPU, read its state, restart it into the restore window
	input             ss_stop,       // park at the next instruction boundary
	output            ss_quiet,      // parked, nothing left on the bus
	output            ss_in_hlt,     // the park ended a HLT
	input       [5:0] ss_state_sel,
	output     [31:0] ss_state,
	input             ss_mode,       // the window covers FFF000-FFFFFF and the devices hold still
	input             ss_cpu_reset,  // pulse: reset the CPU alone
	input             ss_win_we,     // engine writes into the window, 32-bit words
	input       [9:0] ss_win_addr,
	input      [31:0] ss_win_data,
	output            ss_retire,     // one pulse per instruction the CPU completes
	output            ss_dev_busy,   // a device is still taking its restored state (FM replay)
	// savestate bus master: one word cycle at a time while the CPU is parked
	input             ss_bus_req,    // held until ss_bus_ack
	input             ss_bus_we,
	input             ss_bus_io,     // I/O cycle: the device steps for it
	input             ss_bus_hidden, // I/O cycle in the state space: {chip id, register}
	input             ss_bus_raw,    // DRAM under the overlays
	input      [23:1] ss_bus_a,
	input       [1:0] ss_bus_be,
	input      [15:0] ss_bus_din,
	output     [15:0] ss_bus_dout,
	output            ss_bus_ack,
	input             reset,
	input             cache_auto,    // cache and waits follow port 05EC fast mode
	input             cache_enable,
	input             mem_fast,      // turbo: no SDRAM waits at all, no debt
	input             sprite_fast,   // sprite engine at Towns II MX speed
	input       [1:0] ram_size,      // 0 the Marty's 2 MB, 1 4 MB, 2 6 MB, 3 8 MB
	input             towns_id,      // serial ID ROM of a desktop Towns
	input       [1:0] video_mode,    // 0 Marty original, 1 240p, 2 480p, 3 Towns native
	input             show_blank,    // draw the blanking area black instead of trimming it

	// SDRAM side: a rise of req queues a request, two may be in flight;
	// ready is one clock per answer, in order, with the data (reads) or
	// once written
	output     [23:1] mem_a,
	output      [1:0] mem_be,
	output            mem_we,
	output     [15:0] mem_din,
	input      [15:0] mem_dout,
	output            mem_req,
	input             mem_ready,

	// VRAM side of the SDRAM: line fetch and RAM port
	output     [18:3] vf_a,
	output            vf_req,
	input             vf_accept,
	input      [63:0] vf_dout,
	input             vf_ready,
	output     [18:1] vr_a,
	output      [1:0] vr_be,
	output            vr_we,
	output     [15:0] vr_din,
	input      [15:0] vr_dout,
	output            vr_req,
	input             vr_ready,

	// video after the Marty output stage
	output            ce_pix,
	output      [7:0] vid_r,
	output      [7:0] vid_g,
	output      [7:0] vid_b,
	output            vid_hs,
	output            vid_vs,
	output            vid_hb,
	output            vid_vb,
	output            vid_field,

	output            power_off,
	output            soft_reset_req,   // one clock: software wrote RST to I/O 20h
	output            beep_out,

	// input devices from the framework
	input      [10:0] ps2_key,
	input      [10:0] ps2_key_raw,   // the framework's keys alone, for the keys held at a reset
	input      [24:0] ps2_mouse,
	input      [12:0] pad1,          // {Z, Y, X, C, zoom, select, run, B, A, up, down, left, right}
	input      [12:0] pad2,
	input       [2:0] pad1_type,     // 0 Marty pad, 1 6-button pad, 2 mouse, 3 analog stick, 4 analog pad, 5 none
	input       [2:0] pad2_type,
	input      [15:0] pad1_stick,    // analog stick, {Y, X} signed
	input      [15:0] pad2_stick,
	input       [7:0] pad1_throttle,
	input       [7:0] pad2_throttle,

	// audio: the mixer out
	output signed [15:0] audio_l,
	output signed [15:0] audio_r,

	// MIDI interface card (FMT-40x): serial lines at 31250 baud
	input             midi_en,
	input             midi_rx,
	output            midi_tx,

	// RS-232C with a modem card: serial lines at the framework's baud
	input             rs_en,
	input      [31:0] rs_baud,
	output            rs_txd,
	input             rs_rxd,
	output            rs_rts,
	output            rs_dtr,
	input             rs_cts,
	input             rs_dsr,
	input             rs_cd,

	// floppies: track records from the HPS slots; drive 1 (DSL1) exists
	// only with two_drives, the DS1 wiring mod
	input             two_drives,
	input             fdd_present,
	input             fdd_wp,
	input             fdd_mounted,
	output      [7:0] fdd_lba,
	output            fdd_rd,
	output            fdd_wr,
	input             fdd_done,
	input             fdd_err,
	output     [13:0] fdd_buf_addr,
	output            fdd_buf_we,
	output      [7:0] fdd_buf_din,
	input       [7:0] fdd_buf_dout,
	input             fdd2_present,
	input             fdd2_wp,
	input             fdd2_mounted,
	output      [7:0] fdd2_lba,
	output            fdd2_rd,
	output            fdd2_wr,
	input             fdd2_done,
	input             fdd2_err,
	output     [13:0] fdd2_buf_addr,
	output            fdd2_buf_we,
	output      [7:0] fdd2_buf_din,
	input       [7:0] fdd2_buf_dout,

	// IC card image
	input             card_present,
	input      [17:0] card_blocks,
	output     [17:0] card_lba,
	output            card_rd,
	output            card_wr,
	input             card_done,
	input             card_err,
	output      [8:0] card_buf_addr,
	output            card_buf_we,
	output      [7:0] card_buf_din,
	input       [7:0] card_buf_dout,
	output            card_warn,      // one clock: the card window was used with no image

	// CMOS image
	input             cmos_present,
	input             cmos_mounted,   // one clock: the slot reported an image (or none)
	output      [3:0] cmos_lba,
	output            cmos_rd,
	output            cmos_wr,
	input             cmos_done,
	input             cmos_err,
	output      [8:0] cmos_buf_addr,
	output            cmos_buf_we,
	output      [7:0] cmos_buf_din,
	input       [7:0] cmos_buf_dout,
	input             cmos_save,      // one clock: write a changed CMOS back to the image

	// SCSI disk image
	input             hdd_present,
	input      [21:0] hdd_blocks,
	output     [21:0] hdd_lba,
	output            hdd_rd,
	output            hdd_wr,
	input             hdd_done,
	input             hdd_err,
	output      [8:0] hdd_buf_addr,
	output            hdd_buf_we,
	output      [7:0] hdd_buf_din,
	input       [7:0] hdd_buf_dout,

	// second SCSI disk image, ID 1
	input             hdd2_present,
	input      [21:0] hdd2_blocks,
	output     [21:0] hdd2_lba,
	output            hdd2_rd,
	output            hdd2_wr,
	input             hdd2_done,
	input             hdd2_err,
	output      [8:0] hdd2_buf_addr,
	output            hdd2_buf_we,
	output      [7:0] hdd2_buf_din,
	input       [7:0] hdd2_buf_dout,

	// CD-ROM: raw sectors from the HPS slot, TOC by download, audio out
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input       [9:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	input             cd_present,
	input             cd_mounted,
	output     [23:0] cd_lba,
	output            cd_bank,
	output            cd_rd,
	input             cd_done,
	input             cd_err,
	output     [12:0] cd_buf_addr,
	input       [7:0] cd_buf_dout,

	// framework clock seed for the RTC (BCD bytes)
	input             rtc_seed_valid,
	input       [6:0] rtc_seed_sec,
	input       [6:0] rtc_seed_min,
	input       [5:0] rtc_seed_hour,
	input       [2:0] rtc_seed_wday,
	input       [5:0] rtc_seed_day,
	input       [4:0] rtc_seed_month,
	input       [7:0] rtc_seed_year,

	// debug taps for the harness
	output reg        dbg_io_unmapped,   // pulse per I/O read lane nothing answered
	output            dbg_io_wr,         // pulse per I/O write lane
	output            dbg_io_rd,         // pulse per I/O read lane, data as the CPU sees it
	output     [15:0] dbg_io_addr,
	output      [7:0] dbg_io_data,
	output            dbg_spr_we,        // CPU write into sprite RAM (word address, lanes, data)
	output     [16:1] dbg_spr_a,
	output      [1:0] dbg_spr_be,
	output     [15:0] dbg_spr_din,
	input             dbg_bm_en,         // bench peek at the layer 1 clear bitmap
	input      [10:0] dbg_bm_addr,
	output     [63:0] dbg_bm_q,
	output     [15:0] dbg_CS,
	output     [31:0] dbg_EIP,
	output            dbg_retire,
	output            dbg_dma_bus,      // the DMAC owns the bus
	output    [127:0] dbg_crtc_regs,
	output    [255:0] dbg_gpr,          // {EDI, ESI, EBP, ESP, EBX, EDX, ECX, EAX}
	output     [95:0] dbg_seg           // {GS, FS, SS, DS, ES, CS}
);

// ---- CPU pins ----
// The savestate engine drives them in place of the parked CPU: one data
// cycle per request, ADS_n low until the sequencer takes it.
wire [23:1] cpu_a;
wire [15:0] cpu_d_o_pin;
wire        cpu_bhe_n, cpu_ble_n, cpu_ads_n, cpu_w_r_n, cpu_d_c_n, cpu_m_io_n;
reg         ss_cyc_open;
wire        ss_act  = ss_mode & ss_bus_req;
wire [23:1] a       = ss_act ? ss_bus_a : cpu_a;
wire [15:0] cpu_d_o = ss_act ? ss_bus_din : cpu_d_o_pin;
wire        bhe_n   = ss_act ? ~ss_bus_be[1] : cpu_bhe_n;
wire        ble_n   = ss_act ? ~ss_bus_be[0] : cpu_ble_n;
wire        ads_n   = ss_act ? ss_cyc_open : cpu_ads_n;
wire        w_r_n   = ss_act ? ss_bus_we : cpu_w_r_n;
wire        d_c_n   = ss_act ? 1'b1 : cpu_d_c_n;
wire        m_io_n  = ss_act ? ~ss_bus_io : cpu_m_io_n;
wire        lock_n, d_oe;
reg  [15:0] cpu_d_i;
reg         ready_n;
wire        cpu_ready_n;
wire [15:0] cpu_d_in;
wire        hold, hlda, soft_reset, cpu_pe, cpu_pg;

// The RST bit resets the CPU alone; hold it long enough for the core.
// After a reset the CPU also waits for the CMOS image: until its slot
// holds one (or BOOT_WAIT runs out) and the image has been loaded.
reg  [4:0] rst_cnt;
reg        cpu_shutdown;   // shutdown indication cycle seen: reset the CPU, flag it in 0020
reg        boot_hold;
reg        cmos_known;
reg [31:0] boot_wait;
wire       cmos_loading;
wire       cpu_reset = reset | (rst_cnt != 0) | boot_hold;
always @(posedge clk) begin
	if (reset) begin rst_cnt <= 5'd0; boot_hold <= 1; cmos_known <= 0; boot_wait <= BOOT_WAIT; end
	else if (soft_reset || cpu_shutdown || ss_cpu_reset) rst_cnt <= 5'd16;
	else if (ce_cpu && rst_cnt != 0) rst_cnt <= rst_cnt - 1'd1;
	if (cmos_mounted || cmos_present || boot_wait == 0) cmos_known <= 1;
	else if (boot_wait != 0) boot_wait <= boot_wait - 1'd1;
	if (!reset && ce_cpu && cmos_known && !cmos_loading) boot_hold <= 0;
end

am386sx cpu
(
	.clk(clk),
	.ce(ce_cpu),
	.cache_enable(cache_on),
	.ram_size(ram_size),
	.RESET(cpu_reset),
	.A(cpu_a),
	.BHE_n(cpu_bhe_n),
	.BLE_n(cpu_ble_n),
	.ADS_n(cpu_ads_n),
	.W_R_n(cpu_w_r_n),
	.D_C_n(cpu_d_c_n),
	.M_IO_n(cpu_m_io_n),
	.LOCK_n(lock_n),
	.bus_oe(),
	.D_i(cpu_d_in),
	.D_o(cpu_d_o_pin),
	.D_oe(d_oe),
	.READY_n(cpu_ready_n),
	.NA_n(na_n),
	.HOLD(hold),
	.HLDA(hlda),
	.INTR(intr),
	// NMI has one source on this machine: the keyboard interface's RAS
	// (service dump) request, raised by the controller on an undocumented
	// key sequence, flagged at 0604 bit 1, cleared by order B2h, gated by
	// the 0028 mask. The ROM handler saves SS:SP at 0000:03EC and halts on
	// an error screen; no software depends on it. Tied off until the key
	// sequence is known. The expansion-bus BNMI (05C0/05C2) has no source
	// on a Marty.
	.NMI(1'b0),
	.SHUTDOWN(),
	.PE(cpu_pe),
	.PG(cpu_pg),
	.snoop_addr(dma_snoop_a),
	.snoop_valid(dma_snoop),
	.dbg_CS(dbg_CS),
	.dbg_EIP(dbg_EIP),
	.dbg_retire(dbg_retire),
	.dbg_gpr(dbg_gpr),
	.dbg_seg(dbg_seg),
	.ss_stop(ss_stop),
	.ss_halted(cpu_parked),
	.ss_quiet(cpu_quiet),
	.ss_in_hlt(ss_in_hlt),
	.ss_state_sel(ss_state_sel),
	.ss_state(ss_state)
);

// Devices hold still while the CPU runs the restore stub or the engine
// copies memory; an engine I/O cycle steps them so the write registers.
wire        ce_dev = ce_cpu & (~ss_mode | (ss_act & ss_bus_io & ~ss_bus_hidden));

// Fixed 16 MHz for the parts with their own crystal: timers, RTC, FDC,
// 1 us wait, mixer filter, MIDI card, CD drive timers. They keep their rate whatever the
// CPU runs at. Held the same way as ce_dev while a savestate engine works.
localparam [27:0] HZ_16M = 28'd16000000;
reg  [27:0] acc_16m;
reg         ce_16m_tick;
always @(posedge clk) begin
	ce_16m_tick <= 0;
	if (acc_16m + HZ_16M >= CLK_HZ[27:0]) begin
		acc_16m     <= acc_16m + HZ_16M - CLK_HZ[27:0];
		ce_16m_tick <= 1;
	end
	else acc_16m <= acc_16m + HZ_16M;
end
wire        ce_16m = ce_16m_tick & (~ss_mode | (ss_act & ss_bus_io & ~ss_bus_hidden));


// ---- savestate glue ----
// The restore window: the engine fills the LOADALL table, the CPU reads
// the stub and the table as 16-bit words.
wire        cpu_parked, cpu_quiet;
reg  [11:1] sswin_a;
wire [31:0] sswin_q;
cache_ram_dp #(.ADDR_WIDTH(10), .DATA_WIDTH(32), .MEM_INIT_FILE("ss_window.mif"), .SIM_INIT_FILE(SS_WINDOW_HEX)) ss_window
(
	.clk_i(clk),
	.addr_a_i(ss_win_addr), .wren_a_i(ss_win_we), .wdata_a_i(ss_win_data), .q_a_o(),
	.addr_b_i(sswin_a[11:2]), .wren_b_i(1'b0), .wdata_b_i(32'd0), .q_b_o(sswin_q)
);
wire [15:0] sswin_dout = sswin_a[1] ? sswin_q[31:16] : sswin_q[15:0];

// ---- decode ----
// The DMAC's address takes the decoder while it owns the bus.
wire [23:1] sdram_a;
wire        sel_sdram, sel_rom, sel_cmos, sel_none, sel_vram, sel_sprite, sel_fmr, sel_pcm, sel_card, sel_sswin;
wire        main_mem_c0, ram_at_f8, dict_ram;
wire  [3:0] dict_bank;
wire [23:0] dma_a;
wire        dma_bus;
reg   [7:0] dma_ext_a;               // 00A7: A31-A24 of the DMA address
assign dbg_dma_bus = dma_bus;
// The 386SX bus has 24 address lines. Software programs the DMAC with the
// 32-bit addresses of a 386DX Towns, and the gate array folds those onto
// this board's windows:
//   80000000 VRAM            -> A00000 / B00000
//   81000000 sprite RAM      -> C00000
//   40000000 I/O expansion   -> C80000
//   C0000000 memory card     -> D00000
//   C2000000 OS and dict ROM -> E00000
//   C2100000 font ROM, CMOS  -> F00000
//   C2200000 PCM wave RAM    -> F80000
//   FFFC0000 system ROM      -> FC0000
reg  [23:1] dma_a_sx;
always @* begin
	dma_a_sx = dma_a[23:1];
	case (dma_ext_a)
	8'h80: dma_a_sx[23:20] = {3'b101, dma_a[20]};
	8'h81: dma_a_sx[23:19] = 5'b11000;
	8'h40: dma_a_sx[23:19] = 5'b11001;
	8'hC0: dma_a_sx[23:20] = 4'hD;
	8'hC2: dma_a_sx[23:19] = dma_a[21] ? 5'b11111 : {3'b111, dma_a[20], dma_a[19]};
	default: ;
	endcase
end
wire [23:1] bus_a = dma_bus ? dma_a_sx : a;
// 8 MB: the ROM disk loaders read 600000-7FFFFF with paging off; the OS and
// its programs use that RAM with paging on, and anyone's writes land in RAM
wire        rom_view = !dma_bus && !ss_act && !cpu_pg && !w_r_n;

towns_memmap memmap
(
	.a(bus_a),
	.rom_view(rom_view),
	.main_mem_c0(main_mem_c0),
	.ram_at_f8(ram_at_f8),
	.dict_ram(dict_ram),
	.dict_bank(dict_bank),
	.ram_size(ram_size),
	.ss_window(ss_mode),
	.ss_raw(ss_act & ss_bus_raw),
	.sdram_a(sdram_a),
	.sel_sdram(sel_sdram),
	.sel_rom(sel_rom),
	.sel_cmos(sel_cmos),
	.sel_vram(sel_vram),
	.sel_sprite(sel_sprite),
	.sel_fmr(sel_fmr),
	.sel_pcm(sel_pcm),
	.sel_card(sel_card),
	.sel_none(sel_none),
	.sel_sswin(sel_sswin)
);

// C0000-CFFFF splits into the plane window, the text VRAM area that lives
// in the sprite RAM, and the CFF80 registers presented as I/O FF8x
wire        fmr_regs   = sel_fmr && (bus_a[15:7] == 9'h1FF);
wire        fmr_tvram  = sel_fmr && bus_a[15] && !fmr_regs;
wire        fmr_planes = sel_fmr && !bus_a[15];
wire        fmr_ank_font;
// CA000-CAFFF 8 x 8 and CB000-CBFFF 8 x 16 ANK glyphs from the font ROM
// while CFF99 bit 0 is set; writes still land in the text VRAM under them
wire        ank_rd     = sel_fmr && fmr_ank_font && (a[15:13] == 3'b101) && m_io_n && !w_r_n;
wire [23:1] ank_a      = 23'h2C0000 + (a[12] ? 23'h01EC00 : 23'h01E800) + {11'd0, a[11:1]};

// ---- kanji CG window ----
// CFF94/CFF95 latch a JIS code, CFF96/CFF97 read the left and right byte of
// one 16 x 16 glyph row from the font ROM and the row steps after the
// right byte. The glyph index follows the ROM layout: rows 21-2F in three
// 32 x 8 blocks (row 28 takes slot 0), rows 30-7F in 32 x 16 blocks.
// I/O ports FF94-FF97 are the same registers (real-mode games use them).
reg   [6:0] kanji_hi;
reg   [7:0] kanji_lo;
reg   [3:0] kanji_row;
wire  [1:0] kanji_bx    = kanji_lo[6:5] - 2'd1;                    // (lo - 20) >> 5
wire  [2:0] kanji_by    = kanji_hi[6:4] - 3'd3;                    // (hi - 30) >> 4
wire  [3:0] kanji_blk   = {kanji_by, 1'b0} + {1'b0, kanji_by} + {2'b00, kanji_bx};   // by * 3 + bx
wire [12:0] kanji_code  = (kanji_hi[6:4] < 3'd3) ?
                          {3'b000, kanji_bx[0], kanji_bx[1], kanji_hi[2:0], kanji_lo[4:0]} :   // blocks 1 and 2 swap
                          {kanji_blk, kanji_hi[3:0], kanji_lo[4:0]} + 13'h0400;
wire        kanji_io    = !m_io_n && (a[15:2] == 14'h3FE5);                    // I/O FF94-FF97
wire        kanji_rd    = ((fmr_regs && m_io_n) || kanji_io) && !w_r_n && (a[6:1] == 6'h0B);   // CFF96/97, FF96/97
wire [23:1] kanji_a     = 23'h2C0000 + {6'd0, kanji_code, kanji_row};           // 16 words per glyph
wire        font_rd     = (kanji_rd | ank_rd) & ~dma_bus;                        // one font ROM word, read like ROM
wire [23:1] font_a      = kanji_rd ? kanji_a : ank_a;

// ---- cycle sequencer ----
// Captured at T1 (ADS_n low); READY_n answers at the end of a later T2.
localparam [3:0] S_IDLE = 4'd0, S_MEM = 4'd1, S_BYTE0 = 4'd2, S_BYTE1 = 4'd3, S_DONE = 4'd4,
                 S_SPR = 4'd6, S_VRAM = 4'd7, S_FMR = 4'd8, S_FMR1 = 4'd9, S_CARD = 4'd10, S_SSWIN = 4'd11;

reg  [3:0] state;
reg  [4:0] waits;
// A write to the 1 us wait register (006C) holds READY for a further 1 us
// on top of its normal I/O cycle: 16 T-states at 16 MHz.
localparam [4:0] WAIT_1US = 5'd16;
// A byte-device cycle is T1, the strobe state and WAIT_IO - 1 more, with
// READY driven in the last one, so ADS to READY is 2 + WAIT_IO T-states.
localparam [3:0] WAIT_LANE = (WAIT_IO > 0) ? WAIT_IO[3:0] - 4'd1 : 4'd0;
reg        c_io, c_wr, c_inta, c_cmos, c_fmrreg, c_pcm, c_hidden;
reg  [1:0] c_be;
reg [15:0] c_addr;      // byte address of the low lane (I/O) or CMOS index
reg  [7:0] byte_lo;     // low-lane read data while the high lane runs

wire is_inta  = ~m_io_n & ~d_c_n & ~dma_bus;
wire is_halt  = m_io_n & ~d_c_n & w_r_n & ~dma_bus;   // halt or shutdown indication: acknowledge, touch nothing
// A1 low marks the shutdown cycle. One T-state pulse, kept out of the bus
// block so the reset it starts cannot clear it before 0020 has latched it.
always @(posedge clk) if (ce_cpu) cpu_shutdown <= !cpu_reset && !hlda && !ads_n && is_halt && !a[1];

// ---- wait states ----
//
// No Marty has been measured, so each region gets the fewest waits its
// actual part can meet on the Am386SX-16 bus. One T-state is 62.5 ns; a
// zero-wait cycle is T1+T2 = 125 ns; the SX drives the address up to 36 ns
// into T1 and wants read data 9 ns before the end of T2, so external
// memory has about 80 ns from address to data, 142.5 ns with one wait,
// 205 ns with two. Every wait adds 62.5 ns.
//
//   Main RAM   4x HM514400AZ8 (Hitachi 1M x 4 fast-page DRAM, 80 ns grade)
//              as one 16-bit bank of 2 MB. tRAC 80 ns, tCAC 20 ns, tAA 40,
//              tRP 60, tRAS 80, tRC 150, tPC 50. A random cycle needs tRC,
//              150 ns > 125 ns, so 0 waits is impossible; 3 T-states =
//              187.5 ns clears it with the data landing at about RAS + 80
//              + 9 ns setup. Floor: 1. Fast-page hits could run at 0 if the
//              gate array kept RAS low; nothing shows that it did.
//   ROM        2x HN624116FB (Hitachi 16 Mbit mask ROM, 1M x 16), 150 ns
//              grade assumed (the S20/S21 marks are mask codes, not speed).
//              tAA 150 + 36 address + 9 setup = 195 ns > 178.5 (one wait),
//              < 241.5 (two). Floor: 2; the 200 ns grade would need 3.
//   VRAM       4x TMS48C121DZ-80 (TI 128K x 8 dual-port video DRAM).
//              tRAC 80, tRC 160, tRP 70, tPC 50. tRC 160 > 125: floor 1,
//              plus whatever the CRTC's serial transfers steal.
//   Sprite RAM TC511664BJ-80 (Toshiba 64K x 16 DRAM). tRAC 80, tRC 135
//              > 125: floor 1.
//   I/O        2, unmeasured.
//
// The Databook's 3/6 (compatibility) and 0/3 (fast) figures belong to the
// 32-bit desktop models and are not used here. The refresh stalls of the
// real DRAM (CBR, 1024 rows per 16.4 ms, one tRC each) are matched in kind
// by the SDRAM's auto-refresh, which is what wait_debt below evens out.
//
// A CPU SDRAM cycle asks the port during T1, straight from the pins, and
// READY answers in the T-state the data lands, so a RAM cycle costs
// 2 + WAIT_RAM T-states. The DMA path keeps its registered request.
//
// The SDRAM can answer late (refresh, another port). Every T-state spent
// waiting past the modelled count goes into wait_debt, and later RAM or ROM
// cycles drop one wait each while the debt lasts, so timing stays even
// over a few microseconds. A shaved cycle whose data is still late simply
// waits, and the extra T-states go back into the debt.
//
// The address is pipelined (a tRC 150 ns DRAM only reaches a 125 ns cycle
// with the address a T-state early), so the RAM waits go and only the
// mask ROM keeps one; mem_fast (turbo) drops that too. NA_n is
// low through a memory cycle, so the CPU puts its next cycle on the pins
// with ADS_n in T2P and opens it with T1P once READY answers. The pins
// then hold that cycle until it ends, so the T1 decode below serves it
// from pipe_pend at T1P. A pipelined read goes to the port as soon as
// its address is on the pins in T2P, a T before its T1P, and waits there
// behind the access in flight; the port answers in order, one clock each,
// and answers not yet taken by a T-state queue in ans_d.
//
//   T-state    T1   T2   T2P  |T1P  T2P  |T1P ...
//   pins       N    N    N+1  |N+1  N+2  |N+2
//   ADS_n      0    1    0    |1    0    |1
//   port       req N     req N+1   req N+2
//   READY_n              0    |     0    |
reg  [23:1] mem_a_r;
reg   [1:0] mem_be_r;
reg         mem_we_r, mem_req_r;
reg  [15:0] mem_din_r;
reg         pipe_pend;     // the next cycle is on the pins, not yet opened
reg         pipe_issued;   // and its read already went to the port
reg   [1:0] req_open;      // requests at the port, answers still to come
reg   [1:0] ans_cnt;       // answers back, not yet taken by a T-state
reg  [15:0] ans_d [0:1];
wire pin_mem  = !hlda && !is_inta && !is_halt && ((m_io_n && sel_sdram) || font_rd);
wire t1_mem   = (state == S_IDLE) && (!ads_n || pipe_pend) && pin_mem;
wire mem_ans  = mem_ready && (req_open != 0);   // an answer lands this clock
wire ans_avail = mem_ans || (ans_cnt != 0);
wire [15:0] ans_data = (ans_cnt != 0) ? ans_d[0] : mem_dout;
wire mem_done = (state == S_MEM) && (waits == 0) && ans_avail;
wire byte_done = (state == S_DONE) && (waits == 0);
// (held off one clock after a T1P request so the port sees a second rise)
reg  t1_mem_d;
wire pipe_ask = (state == S_MEM) && !ads_n && !pipe_pend && !pipe_issued && pin_mem && !w_r_n && !t1_mem_d;
// NA_n low while a memory cycle runs, and in T1P while its pipelined
// successor (still on the pins) is a memory cycle, so the next one queues too
wire na_n     = ~((state == S_MEM && !pipe_pend) || (state == S_IDLE && pipe_pend && pin_mem));
reg   [2:0] wait_debt;
reg         wait_shaved;   // this cycle runs one wait short of its floor
// Pipelined, the address reaches the DRAM a T-state early, so its one wait
// hides and a hit ends at 2 T; the slower mask ROM still shows one wait.
// Auto: the caches and the waits follow the fast-mode bit software sets.
wire        fast_mode;
wire        cache_on = cache_auto ? fast_mode : cache_enable;
wire        no_waits = cache_auto ? fast_mode : mem_fast;
wire  [3:0] mem_waits = (ss_act || no_waits) ? 4'd0 : {3'd0, sel_rom};   // the engine's copies need none
wire mem_ask   = (t1_mem && !pipe_issued) | pipe_ask;
assign mem_req = mem_req_r | mem_ask;
// The port samples these on the rise of mem_req. A registered (DMA) request
// never shares a clock with a CPU ask, so the registered one selects: the
// CPU's address and data reach the port without waiting for the decode.
assign mem_a   = mem_req_r ? mem_a_r : (font_rd ? font_a : sdram_a);
assign mem_be  = mem_req_r ? mem_be_r : {~bhe_n, ~ble_n};
// Port 0020 WRPROT guards the INT 2 (NMI) vector, the four bytes at physical
// 8-B. Writes there are dropped for every master. The CPU cache patched
// its copy when it accepted the write, so a dropped CPU write is snooped
// to make the cache forget it.
wire        nmi_vector_wp;
wire        nmi_vec_hit = nmi_vector_wp && (bus_a[23:2] == 22'd2);
assign mem_we  = mem_req_r ? mem_we_r : (w_r_n & ~sel_rom & ~nmi_vec_hit);
assign mem_din = mem_req_r ? mem_din_r : cpu_d_o;

// The port takes a request on the clock mem_req rises. Answers arrive in
// order; one not taken at the T-state it lands in waits in ans_d.
reg  mem_req_d;
wire mem_take = mem_req && !mem_req_d;
wire ans_take = ce_cpu && mem_done;
wire dispatch = ce_cpu && (state == S_IDLE) && !hlda && (!ads_n || pipe_pend);
// a savestate may take the machine once the parked CPU has nothing left on the bus
// ... and the render side has no fetch out and no sprite transfer running
assign ss_quiet  = cpu_quiet && state == S_IDLE && req_open == 0 && ans_cnt == 0 && !hlda && !hold &&
                   crtc_quiet && !sp_active && cd_quiet && fdd_quiet && fdd2_quiet && !(fdc_busy && (fdd_present || fdd2_present) && fdc_motor) &&
                   hdd_quiet && hdd2_quiet && card_quiet;
assign ss_retire = dbg_retire & ce_cpu;
// the engine's cycle: opened at dispatch, done when the CPU would see READY
assign cpu_ready_n = ready_n & ~mem_done & ~vram_done & ~byte_done;
assign cpu_d_in    = mem_done ? ans_data : vram_done ? vram_cpu_dout :
                     (byte_done && sel_none && !c_io && !c_cmos) ? 16'hFFFF : cpu_d_i;
assign ss_bus_ack  = ss_act & ss_cyc_open & ce_cpu & ~cpu_ready_n;
assign ss_bus_dout = cpu_d_in;
always @(posedge clk) begin
	if (!ss_act) ss_cyc_open <= 0;
	else if (dispatch) ss_cyc_open <= 1;
	else if (ss_bus_ack) ss_cyc_open <= 0;
end
always @(posedge clk) begin
	mem_req_d <= mem_req;
	t1_mem_d  <= t1_mem;
	if (cpu_reset) begin
		req_open    <= 0;
		ans_cnt     <= 0;
		pipe_issued <= 0;
	end
	else begin
		req_open <= req_open + {1'b0, mem_take} - {1'b0, mem_ans};
		case ({mem_ans, ans_take})
		2'b10: begin ans_d[ans_cnt[0]] <= mem_dout; ans_cnt <= ans_cnt + 1'd1; end
		2'b01: begin ans_d[0] <= ans_d[1]; ans_cnt <= ans_cnt - 1'd1; end
		2'b11: if (ans_cnt != 0) begin ans_d[0] <= ans_cnt[1] ? ans_d[1] : mem_dout; ans_d[1] <= mem_dout; end
		default: ;
		endcase
		if (ce_cpu && pipe_ask) pipe_issued <= 1;
		else if (dispatch) pipe_issued <= 0;
	end
end

// Byte-device address for the lane in progress.
wire        lane      = (state == S_BYTE1);
wire [15:0] lane_addr = {c_addr[15:1], lane};
wire        lane_rd   = (state == S_BYTE0 || state == S_BYTE1) && !c_wr;
wire        lane_wr   = (state == S_BYTE0 || state == S_BYTE1) && c_wr;
wire  [7:0] lane_din  = lane ? bm_dout[15:8] : bm_dout[7:0];

// I/O byte bus: every device answers with data and a select; the first
// select wins. Chips with datasheet pins get CS_n/RD_n/WR_n levels that
// last the whole lane.
wire  [7:0] sysregs_dout, cmos_dout, pic_m_dout, pic_s_dout, pit1_dout, pit2_dout, intctrl_dout;
wire        sysregs_sel, pic_m_oe, pic_s_oe, intctrl_sel;
// a 16-bit register answers both lanes in one cycle; the high lane is kept
// for the second half of the split I/O cycle instead of reading it again
wire        intctrl_wide;
wire  [7:0] intctrl_dout_hi;
wire        io_wide    = intctrl_wide;
wire  [7:0] io_dout_hi = intctrl_dout_hi;
reg         wide_q;
reg   [7:0] byte_hi;
wire        io_rd = lane_rd & (c_io | c_fmrreg | c_pcm) & ~c_inta & ~c_hidden & ~(wide_q & (state == S_BYTE1));
wire        io_wr = lane_wr & (c_io | c_fmrreg | c_pcm) & ~c_inta & ~c_hidden;

// ---- savestate state space ----
// The engine's hidden I/O cycles reach each chip's state port instead of
// its guest registers: the high address byte names the chip, the low
// byte the state byte. Id 0 is nobody.
localparam [7:0] SS_PIC_M = 8'h01, SS_PIC_S = 8'h02, SS_PIT1 = 8'h03, SS_PIT2 = 8'h04, SS_INTCTRL = 8'h05,
                 SS_SYSREGS = 8'h06, SS_DMAC = 8'h07, SS_BOARD = 8'h08, SS_CRTC = 8'h09, SS_VRAM = 8'h0A,
                 SS_SPRITE = 8'h0B, SS_KBD = 8'h0C, SS_PAD = 8'h0D, SS_PCM = 8'h0E, SS_MIXER = 8'h0F,
                 SS_FM = 8'h10, SS_CDC = 8'h11, SS_CDMPU = 8'h12, SS_CDHOST = 8'h13, SS_FDC = 8'h14, SS_FDD = 8'h15,
                 SS_SCSI = 8'h16, SS_HDD = 8'h17, SS_CARD = 8'h18;
wire        ss_wr    = lane_wr & c_hidden;
wire        ss_step  = ss_bus_ack;   // one clock as each engine cycle ends
// only an engine cycle names a chip: a guest address with the same high
// byte must not steer the chips' state-port muxes (the CD sector RAM's
// write port among them). c_hidden keeps the last cycle's value until the
// next dispatch, so the savestate mode gates it too.
wire  [7:0] ss_id    = (ss_mode && c_hidden) ? lane_addr[15:8] : 8'd0;
wire  [7:0] ss_reg   = lane_addr[7:0];
wire  [7:0] ss_pic_m_dout, ss_pic_s_dout, ss_pit1_dout, ss_pit2_dout, ss_intctrl_dout, ss_sysregs_dout, ss_dmac_dout;
wire  [7:0] ss_dout  = ss_id == SS_PIC_M   ? ss_pic_m_dout :
                       ss_id == SS_PIC_S   ? ss_pic_s_dout :
                       ss_id == SS_PIT1    ? ss_pit1_dout :
                       ss_id == SS_PIT2    ? ss_pit2_dout :
                       ss_id == SS_INTCTRL ? ss_intctrl_dout :
                       ss_id == SS_SYSREGS ? ss_sysregs_dout :
                       ss_id == SS_DMAC    ? ss_dmac_dout :
                       ss_id == SS_BOARD   ? ss_board_dout :
                       ss_id == SS_CRTC    ? ss_crtc_dout :
                       ss_id == SS_VRAM    ? ss_vram_dout :
                       ss_id == SS_SPRITE  ? ss_spr_dout :
                       ss_id == SS_KBD     ? ss_kbd_dout :
                       ss_id == SS_PAD     ? ss_pad_dout :
                       ss_id == SS_PCM     ? ss_pcm_dout :
                       ss_id == SS_MIXER   ? ss_mix_dout :
                       ss_id == SS_FM      ? ss_fm_dout :
                       ss_id == SS_CDC     ? ss_cdc_dout :
                       ss_id == SS_CDMPU   ? ss_cdmpu_dout :
                       ss_id == SS_CDHOST  ? ss_cdhost_dout :
                       ss_id == SS_FDC     ? ss_fdc_dout :
                       ss_id == SS_FDD     ? (ss_reg[3] ? ss_fdd2_dout : ss_fdd_dout) :
                       ss_id == SS_SCSI    ? ss_scsi_dout :
                       ss_id == SS_HDD     ? (ss_reg[1] ? ss_hdd2_dout : ss_hdd_dout) :
                       ss_id == SS_CARD    ? ss_card_dout : 8'h00;
wire  [7:0] ss_card_dout;
wire        card_quiet;
wire  [7:0] ss_cdc_dout, ss_cdmpu_dout, ss_cdhost_dout, ss_fdc_dout, ss_fdd_dout, ss_fdd2_dout, ss_scsi_dout, ss_hdd_dout, ss_hdd2_dout;
wire        cd_quiet, cd_busy, fdd_quiet, fdd2_quiet, fdc_busy, hdd_quiet, hdd2_quiet;
wire  [7:0] ss_crtc_dout, ss_vram_dout, ss_spr_dout, ss_kbd_dout, ss_pad_dout, ss_pcm_dout, ss_mix_dout;
wire        crtc_quiet;
// the board's own dividers and latches, kept in their own blocks below
wire        ss_board_wr = ss_wr && ss_id == SS_BOARD;
wire  [7:0] ss_board_dout =
	ss_reg == 8'h00 ? tmr_acc[7:0]    : ss_reg == 8'h01 ? tmr_acc[15:8]    : ss_reg == 8'h02 ? tmr_acc[23:16]    : ss_reg == 8'h03 ? {7'd0, tmr_acc[24]} :
	ss_reg == 8'h04 ? tmr_hi_acc[7:0] : ss_reg == 8'h05 ? tmr_hi_acc[15:8] : ss_reg == 8'h06 ? tmr_hi_acc[23:16] : ss_reg == 8'h07 ? {7'd0, tmr_hi_acc[24]} :
	ss_reg == 8'h08 ? {3'd0, ce_dma_tog, fdc_div} : ss_reg == 8'h09 ? {1'b0, kanji_hi} : ss_reg == 8'h0A ? kanji_lo :
	ss_reg == 8'h0B ? {4'd0, kanji_row} : ss_reg == 8'h0C ? dma_ext_a : ss_reg == 8'h0D ? {5'd0, wait_debt} :
	ss_reg == 8'h10 ? {1'b0, fdc_clksel, fdc_motor, fdc_side, fdc_irqmsk, fdc_hispd, fdc_modeb, fdc_drvchg} :
	ss_reg == 8'h11 ? {4'd0, fdc_dsl} : ss_reg == 8'h12 ? {rtc_cs, rtc_rd, rtc_wr, rtc_adrs, rtc_din} : 8'h00;
wire        inta_n = ~(c_inta && (state == S_BYTE0 || state == S_DONE));
wire        cmos_io   = c_io && (lane_addr[15:12] == 4'h3);
wire        pic_m_cs  = c_io && (lane_addr[15:2] == 14'h0000) && !lane_addr[0];   // 0000, 0002
wire        pic_s_cs  = c_io && (lane_addr[15:2] == 14'h0004) && !lane_addr[0];   // 0010, 0012
wire        pit1_cs   = c_io && (lane_addr[15:3] == 13'h0008) && !lane_addr[0];   // 0040-0046
wire        pit2_cs   = c_io && (lane_addr[15:3] == 13'h000A) && !lane_addr[0];   // 0050-0056
wire        io_sel    = cmos_io | sysregs_sel | intctrl_sel | pic_m_cs | pic_s_cs | pit1_cs | pit2_cs |
                        rtc_cs_io | rtc_ct_io | dmac_cs | crtc_sel | spr_io_sel | vram_io_sel | cd_sel |
                        fdc_cs | fdc_glue | fdc_ext | kbd_sel | pad_sel | fm_cs | pcm_cs | mix_sel | card_sel | midi_sel | scsi_sel | rs_sel;
wire  [7:0] io_dout   = c_hidden ? ss_dout :
                        pic_m_oe ? pic_m_dout : pic_s_oe ? pic_s_dout :
                        cmos_io  ? cmos_dout  : sysregs_sel ? sysregs_dout : intctrl_sel ? intctrl_dout :
                        crtc_sel ? crtc_dout  : spr_io_sel ? spr_io_dout : vram_io_sel ? vram_io_dout :
                        pic_m_cs ? pic_m_dout : pic_s_cs ? pic_s_dout :
                        pit1_cs  ? pit1_dout  : pit2_cs ? pit2_dout :
                        rtc_cs_io ? rtc_io_dout :
                        cd_sel   ? cd_dout    : fdc_cs ? fdc_dout : fdc_glue ? fdc_glue_dout : fdc_ext ? 8'h7F : kbd_sel ? kbd_dout :
                        pad_sel  ? pad_dout   : fm_cs ? fm_dout : pcm_oe ? pcm_dout : mix_sel ? mix_dout :
                        card_sel ? card_dout  : midi_sel ? midi_dout : scsi_sel ? scsi_dout : rs_sel ? rs_dout :
                        dmac_cs  ? (lane_addr[3:0] == 4'h7 ? dma_ext_a : dmac_dout) : 8'hFF;
wire  [7:0] byte_dout = c_cmos ? cmos_dout : io_dout;

// kanji CG window latches; the row steps once the right byte has been read
always @(posedge clk) begin
	if (reset) begin
		kanji_hi  <= 7'd0;
		kanji_lo  <= 8'd0;
		kanji_row <= 4'd0;
	end
	else if (ss_board_wr) begin
		if (ss_reg == 8'h09) kanji_hi  <= lane_din[6:0];
		if (ss_reg == 8'h0A) kanji_lo  <= lane_din;
		if (ss_reg == 8'h0B) kanji_row <= lane_din[3:0];
	end
	else if (ce_cpu) begin
		if (io_wr && (c_fmrreg || c_io) && lane_addr == 16'hFF94) kanji_hi <= lane_din[6:0];
		if (io_wr && (c_fmrreg || c_io) && lane_addr == 16'hFF95) begin
			kanji_lo  <= lane_din;
			kanji_row <= 4'd0;
		end
		if (t1_mem && kanji_rd && !bhe_n) kanji_row <= kanji_row + 1'd1;   // the right byte steps the row
	end
end

// ---- interrupt controllers ----
// Slave INT feeds master IR7; CAS selects the slave during the acknowledge.
wire        intr, slave_int;
wire  [2:0] cas;
wire  [6:0] irq_master;   // IR0-6 of the master
wire  [7:0] irq_slave;

i8259a pic_master
(
	.clk(clk), .reset(reset),
	.a0(lane_addr[1]), .cs_n(~pic_m_cs), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(lane_din), .d_o(pic_m_dout), .d_oe(pic_m_oe),
	.ir({slave_int, irq_master}), .int_o(intr), .inta_n(inta_n),
	.cas_i(3'd0), .cas_o(cas), .cas_oe(), .sp_n(1'b1),
	.ss_cs(ss_id == SS_PIC_M), .ss_wr(ss_wr), .ss_a(ss_reg[2:0]), .ss_din(lane_din), .ss_dout(ss_pic_m_dout)
);

i8259a pic_slave
(
	.clk(clk), .reset(reset),
	.a0(lane_addr[1]), .cs_n(~pic_s_cs), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(lane_din), .d_o(pic_s_dout), .d_oe(pic_s_oe),
	.ir(irq_slave), .int_o(slave_int), .inta_n(inta_n),
	.cas_i(cas), .cas_o(), .cas_oe(), .sp_n(1'b0),
	.ss_cs(ss_id == SS_PIC_S), .ss_wr(ss_wr), .ss_a(ss_reg[2:0]), .ss_din(lane_din), .ss_dout(ss_pic_s_dout)
);

// ---- interval timers ----
// Timer clocks as enables derived from the fixed 16 MHz enable. The ratios
// are exact on average; a single pulse may land one 16 MHz tick late. The
// accumulators need a bit above BASE_HZ so the sum cannot wrap.
localparam [24:0] BASE_HZ   = 25'd16000000;
localparam [24:0] TMR_HZ    = 25'd307200;    // counters 0 to 2
localparam [24:0] TMR_HI_HZ = 25'd1228800;   // baud-rate counter
reg  [24:0] tmr_acc, tmr_hi_acc;
reg         ce_tmr, ce_tmr_hi;
always @(posedge clk) begin
	ce_tmr <= 0;
	ce_tmr_hi <= 0;
	if (reset) begin
		tmr_acc <= 0;
		tmr_hi_acc <= 0;
	end
	else if (ce_16m) begin
		if (tmr_acc + TMR_HZ >= BASE_HZ) begin
			tmr_acc <= tmr_acc + TMR_HZ - BASE_HZ;
			ce_tmr <= 1;
		end
		else tmr_acc <= tmr_acc + TMR_HZ;
		if (tmr_hi_acc + TMR_HI_HZ >= BASE_HZ) begin
			tmr_hi_acc <= tmr_hi_acc + TMR_HI_HZ - BASE_HZ;
			ce_tmr_hi <= 1;
		end
		else tmr_hi_acc <= tmr_hi_acc + TMR_HI_HZ;
	end
	if (ss_board_wr) begin
		case (ss_reg)
		8'h00: tmr_acc[7:0] <= lane_din;      8'h01: tmr_acc[15:8] <= lane_din;
		8'h02: tmr_acc[23:16] <= lane_din;    8'h03: tmr_acc[24] <= lane_din[0];
		8'h04: tmr_hi_acc[7:0] <= lane_din;   8'h05: tmr_hi_acc[15:8] <= lane_din;
		8'h06: tmr_hi_acc[23:16] <= lane_din; 8'h07: tmr_hi_acc[24] <= lane_din[0];
		default: ;
		endcase
	end
end

wire  [2:0] pit1_out, pit2_out;

i8253 pit1
(
	.clk(clk), .reset(reset),
	.a(lane_addr[2:1]), .cs_n(~pit1_cs), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(lane_din), .d_o(pit1_dout), .d_oe(),
	.clk_ce({ce_tmr, ce_tmr, ce_tmr}), .gate(3'b111), .out(pit1_out),
	.ss_cs(ss_id == SS_PIT1), .ss_wr(ss_wr), .ss_a(ss_reg[4:0]), .ss_din(lane_din), .ss_dout(ss_pit1_dout)
);

i8253 pit2
(
	.clk(clk), .reset(reset),
	.a(lane_addr[2:1]), .cs_n(~pit2_cs), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(lane_din), .d_o(pit2_dout), .d_oe(),
	.clk_ce({ce_tmr_hi, ce_tmr_hi, ce_tmr}), .gate(3'b111), .out(pit2_out),
	.ss_cs(ss_id == SS_PIT2), .ss_wr(ss_wr), .ss_a(ss_reg[4:0]), .ss_din(lane_din), .ss_dout(ss_pit2_dout)
);

// ---- timer interrupt control, interval timer II, 1 us wait ----
wire irq0, intv_irq, beep;

towns_intctrl intctrl
(
	.clk(clk), .ce(ce_dev), .ce_16m(ce_16m), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(intctrl_dout), .io_sel(intctrl_sel), .io_wide(intctrl_wide), .io_dout_hi(intctrl_dout_hi),
	.tmr1_wr(io_wr && pit1_cs && lane_addr[2:1] == 2'd1),
	.pit_out(pit1_out), .irq0(irq0), .beep(beep), .intv_irq(intv_irq),
	.ss_cs(ss_id == SS_INTCTRL), .ss_wr(ss_wr), .ss_a(ss_reg[3:0]), .ss_din(lane_din), .ss_dout(ss_intctrl_dout)
);

// Sources not built yet are quiet. Keyboard IRQ1, RS-232C IRQ2, MIDI
// IRQ4/IRQ5, FDC IRQ6, CD-ROM IRQ9 (slave IR1), VSYNC IRQ11 (slave IR3),
// sound IRQ13 (slave IR5).
// interval timer II shares IR0 with the PIT timer interrupt
assign irq_master = {fdc_irq, midi_tmr_irq, midi_ser_irq, 1'b0, rs_irq, kbd_irq, irq0 | intv_irq};
assign irq_slave  = {2'd0, snd_irq, 1'b0, vsync_irq, 1'b0, cd_irq, scsi_irq};
assign beep_out   = beep;
assign soft_reset_req = soft_reset;

assign dbg_io_wr   = io_wr & ce_cpu;
assign dbg_io_rd   = io_rd & ce_cpu;
assign dbg_io_addr = lane_addr;
assign dbg_spr_we  = spr_ram_we;
assign dbg_spr_a   = spr_ram_a;
assign dbg_spr_be  = spr_ram_be;
assign dbg_spr_din = spr_ram_din;
assign dbg_io_data = io_wr ? lane_din : byte_dout;

towns_sysregs sysregs
(
	.clk(clk),
	.ce(ce_dev),
	.reset(reset),
	.io_addr(lane_addr),
	.io_rd(io_rd),
	.io_wr(io_wr),
	.io_din(lane_din),
	.io_word(c_be == 2'b11),
	.cpu_pe(cpu_pe),
	.io_dout(sysregs_dout),
	.io_sel(sysregs_sel),
	.shutdown(cpu_shutdown),
	.ram_size(ram_size),
	.towns_id(towns_id),
	.soft_reset(soft_reset),
	.power_off(power_off),
	.nmi_mask(),
	.nmi_vector_wp(nmi_vector_wp),
	.main_mem_c0(main_mem_c0),
	.ram_at_f8(ram_at_f8),
	.dict_ram(dict_ram),
	.dict_bank(dict_bank),
	.fast_mode(fast_mode),
	.ss_cs(ss_id == SS_SYSREGS), .ss_wr(ss_wr), .ss_a(ss_reg[1:0]), .ss_din(lane_din), .ss_dout(ss_sysregs_dout)
);

// ---- real-time clock ----
// 0070 W nibble to the chip, R {READY, 000, nibble}; 0080 W {CS, 0000, RD, WR, ADRS}.
reg   [3:0] rtc_din;
reg         rtc_cs, rtc_rd, rtc_wr, rtc_adrs;
wire  [3:0] rtc_dout;
wire        rtc_doe, rtc_busy_n;
wire        rtc_cs_io = c_io && lane_addr == 16'h0070;
wire        rtc_ct_io = c_io && lane_addr == 16'h0080;
wire  [7:0] rtc_io_dout = rtc_cs_io ? {rtc_busy_n, 3'd0, rtc_doe ? rtc_dout : 4'hF} : 8'hFF;

// 32.768 kHz enable: 128 pulses per 62500 ticks of the fixed 16 MHz,
// never held for a savestate.
reg  [15:0] rtc_acc;
reg         ce_32k;
always @(posedge clk) begin
	ce_32k <= 0;
	if (reset) rtc_acc <= 0;
	else if (ce_16m_tick) begin
		if (rtc_acc + 16'd128 >= 16'd62500) begin
			rtc_acc <= rtc_acc + 16'd128 - 16'd62500;
			ce_32k <= 1;
		end
		else rtc_acc <= rtc_acc + 16'd128;
	end
end

always @(posedge clk) begin
	if (reset) begin
		rtc_din <= 4'd0;
		{rtc_cs, rtc_rd, rtc_wr, rtc_adrs} <= 4'd0;
	end
	else if (ss_board_wr && ss_reg == 8'h12) {rtc_cs, rtc_rd, rtc_wr, rtc_adrs, rtc_din} <= lane_din;
	else if (ce_cpu && io_wr) begin
		if (rtc_cs_io) rtc_din <= lane_din[3:0];
		if (rtc_ct_io) {rtc_cs, rtc_rd, rtc_wr, rtc_adrs} <= {lane_din[7], lane_din[2:0]};
	end
end

rtc58323 rtc
(
	.clk(clk), .reset(reset), .ce_32k(ce_32k),
	.cs(rtc_cs), .rd(rtc_rd), .wr(rtc_wr), .adrs(rtc_adrs),
	.d_i(rtc_din), .d_o(rtc_dout), .d_oe(rtc_doe), .busy_n(rtc_busy_n),
	.seed_valid(rtc_seed_valid),
	.seed_sec(rtc_seed_sec), .seed_min(rtc_seed_min), .seed_hour(rtc_seed_hour), .seed_wday(rtc_seed_wday),
	.seed_day(rtc_seed_day), .seed_month(rtc_seed_month), .seed_year(rtc_seed_year)
);

// ---- DMA controller ----
// Half the CPU clock; the board's DMAC clock is not documented. 00A7 is
// the board's own latch for A31-A24, which a 24-bit bus never uses.
reg         ce_dma_tog;
wire        ce_dma = ce_dev & ce_dma_tog;
always @(posedge clk) if (reset) ce_dma_tog <= 0; else if (ss_board_wr && ss_reg == 8'h08) ce_dma_tog <= lane_din[4]; else if (ce_dev) ce_dma_tog <= ~ce_dma_tog;

wire        dmac_cs = c_io && (lane_addr[15:4] == 12'h00A);
wire  [7:0] dmac_dout;
wire        dma_ube_n, dma_mrd_n, dma_mwr_n, dma_iord_n, dma_iowr_n, dma_tc_n, dma_tc_oe;
wire  [3:0] dma_dack_n;
wire  [3:0] dma_req = {cd_dma_req, 1'b0, scsi_dma_req, fdc_drq};   // channel 3 CD-ROM, 1 SCSI, 0 floppy
wire [15:0] dma_dev_din = ~dma_dack_n[0] ? {fdc_dout, fdc_dout} :          // acknowledged device, both lanes; open bus otherwise
                          ~dma_dack_n[1] ? {scsi_dma_dout, scsi_dma_dout} :
                          ~dma_dack_n[3] ? cd_dma_dout : 16'hFFFF;
reg         dma_ready;
reg   [7:0] dma_mem_byte;            // memory data on its way to the device
reg         dma_snoop;               // a DMA memory write for the CPU caches, held to the next ce_cpu
reg  [23:1] dma_snoop_a;

upd71071 dmac
(
	.clk(clk), .clk_ce(ce_dma), .reset(reset),
	.cs_n(~(dmac_cs && lane_addr[3:0] != 4'h7)), .rd_n(~io_rd), .wr_n(~io_wr),
	.a(lane_addr[3:0]), .d_i(lane_din), .d_o(dmac_dout), .d_oe(),
	.hldrq(hold), .hldak(hlda), .bus_oe(dma_bus), .addr_o(dma_a), .ube_n(dma_ube_n),
	.mrd_n(dma_mrd_n), .mwr_n(dma_mwr_n), .iord_n(dma_iord_n), .iowr_n(dma_iowr_n),
	.ready(dma_ready), .dmarq(dma_req), .dmaak_n(dma_dack_n),
	.end_n(1'b1), .tc_n(dma_tc_n), .tc_oe(dma_tc_oe),
	.ss_cs(ss_id == SS_DMAC), .ss_wr(ss_wr), .ss_a(ss_reg[6:0]), .ss_din(lane_din), .ss_dout(ss_dmac_dout)
);

always @(posedge clk) begin
	if (reset) dma_ext_a <= 8'd0;
	else if (ss_board_wr && ss_reg == 8'h0C) dma_ext_a <= lane_din;
	else if (ce_cpu && io_wr && dmac_cs && lane_addr[3:0] == 4'h7) dma_ext_a <= lane_din;
end

wire        dma_mem_strobe = dma_bus & (~dma_mrd_n | ~dma_mwr_n);
wire  [1:0] dma_be = dma_ube_n ? 2'b01 : dma_a[0] ? 2'b10 : 2'b11;
// Whoever holds the bus drives the cycle: the DMAC's address, lanes,
// MRD/MWR and device data while it has the bus, else the CPU's pins.
// The decode and the target states below see only these.
wire        bm_mem  = dma_bus | m_io_n;
wire        bm_we   = dma_bus ? ~dma_mwr_n : w_r_n;
wire  [1:0] bm_be   = dma_bus ? dma_be : {~bhe_n, ~ble_n};
wire [15:0] bm_dout = dma_bus ? dma_dev_din : cpu_d_o;
reg         dma_cyc;                 // the cycle in progress is the DMAC's
wire        dma_start = hlda & dma_mem_strobe & ~dma_ready & ~dma_cyc;

// ---- floppy controller and drive ----
// The chip sits at the even addresses 0200-0206. The drive glue is the
// Databook register set: 0208 status/control, 020C select, 020E switch.
// DMA channel 0 reaches the data register through DACK0 as a chip select.
// Chip clock 2 MHz or 1 MHz by CLKSEL, both cut from the fixed 16 MHz enable.
reg   [3:0] fdc_div;
wire        ce_1m  = ce_16m && fdc_div == 4'd15;
wire        ce_fdc = ce_16m && (fdc_clksel ? fdc_div == 4'd15 : fdc_div[2:0] == 3'd7);
always @(posedge clk) if (reset) fdc_div <= 0; else if (ss_board_wr && ss_reg == 8'h08) fdc_div <= lane_din[3:0]; else if (ce_16m) fdc_div <= fdc_div + 1'd1;

wire        fdc_cs   = c_io && (lane_addr[15:3] == 13'h0040) && !lane_addr[0];   // 0200-0206
wire        fdc_glue = c_io && (lane_addr[15:3] == 13'h0041) && !lane_addr[0];   // 0208-020E
wire        fdc_ext  = c_io && (lane_addr == 16'h020D);                          // FDDV extension, reads 7F
wire        fdc_dma  = ~dma_dack_n[0];
wire  [7:0] fdc_dout;
wire        fdc_intrq, fdc_drq;
reg         fdc_clksel, fdc_motor, fdc_side, fdc_irqmsk, fdc_hispd, fdc_modeb, fdc_drvchg;
reg   [3:0] fdc_dsl;
wire        fdc_irq  = fdc_intrq & fdc_irqmsk;
wire        fdd_ready, fdd_ip, fdd_tr00, fdd_wprt, fdd_dskchg;
wire        fdd_step, fdd_dirc, fdd_wg, fdd_fmt, fdd_byte_ce, fdd_f_index, fdd_f_am, fdd_f_id, fdd_f_dam, fdd_f_data, fdd_f_data_last, fdd_field, fdd_wr_en;
wire  [7:0] fdd_rd_byte, fdd_id_c, fdd_id_h, fdd_id_r, fdd_id_n, fdd_wr_byte;
wire  [7:0] fdc_glue_dout = lane_addr[2:1] == 2'd0 ? {two_drives, 2'b00, 3'b011, fdd_ready, fdd_dskchg} :   // FD2: internal drive count; FDDV 011
                            lane_addr[2:1] == 2'd3 ? {7'd0, fdc_drvchg} : 8'hFF;

mb8877a fdc
(
	.clk(clk), .ce(ce_fdc), .reset(reset),
	.a(fdc_dma ? 2'b11 : lane_addr[2:1]), .cs_n(~(fdc_cs | fdc_dma)),
	.rd_n(fdc_dma ? dma_iord_n : ~io_rd), .wr_n(fdc_dma ? dma_iowr_n : ~io_wr),
	.d_i(fdc_dma ? dma_mem_byte : lane_din), .d_o(fdc_dout), .d_oe(), .intrq(fdc_intrq), .drq(fdc_drq),
	.step(fdd_step), .dirc(fdd_dirc), .hld(), .wg(fdd_wg), .fmt(fdd_fmt),
	.ready(fdd_ready), .ip(fdd_ip), .tr00(fdd_tr00), .wprt(fdd_wprt),
	.byte_ce(fdd_byte_ce), .rd_byte(fdd_rd_byte), .f_index(fdd_f_index), .f_am(fdd_f_am), .f_id(fdd_f_id), .f_dam(fdd_f_dam),
	.f_data(fdd_f_data), .f_data_last(fdd_f_data_last), .field(fdd_field), .id_c(fdd_id_c), .id_h(fdd_id_h), .id_r(fdd_id_r), .id_n(fdd_id_n),
	.wr_byte(fdd_wr_byte), .wr_en(fdd_wr_en),
	.ss_cs(ss_id == SS_FDC), .ss_wr(ss_wr), .ss_a(ss_reg[3:0]), .ss_din(lane_din), .ss_dout(ss_fdc_dout), .ss_busy(fdc_busy)
);

always @(posedge clk) begin
	if (reset) begin
		fdc_clksel <= 0; fdc_motor <= 0; fdc_side <= 0; fdc_irqmsk <= 0;
		fdc_hispd <= 0; fdc_modeb <= 0; fdc_dsl <= 0; fdc_drvchg <= 0;
	end
	else if (ss_board_wr && ss_reg == 8'h10) {fdc_clksel, fdc_motor, fdc_side, fdc_irqmsk, fdc_hispd, fdc_modeb, fdc_drvchg} <= lane_din[6:0];
	else if (ss_board_wr && ss_reg == 8'h11) fdc_dsl <= lane_din[3:0];
	else if (ce_cpu && io_wr && fdc_glue) begin
		case (lane_addr[2:1])
		2'd0: {fdc_clksel, fdc_motor, fdc_side, fdc_irqmsk} <= {lane_din[5], lane_din[4], lane_din[2], lane_din[0]};
		2'd2: begin
			// MODE-B and HISPD latch with a drive selection
			fdc_dsl <= lane_din[3:0];
			if (|lane_din[3:0]) {fdc_modeb, fdc_hispd} <= {lane_din[7], lane_din[6]};
		end
		2'd3: fdc_drvchg <= lane_din[0];
		default: ;
		endcase
	end
end

// The internal drive is DSL0; the DS1 mod adds a second one on DSL1.
// DRVCHG swaps the internal and external numbers. The FDC sees the
// selected drive; with none selected it sees drive 0, which reports
// not ready. A 2HD image needs the 2HD rotation mode.
wire        sel0 = fdc_drvchg ? fdc_dsl[2] : fdc_dsl[0];
wire        sel1 = (fdc_drvchg ? fdc_dsl[3] : fdc_dsl[1]) & two_drives;
wire        d0_ready, d0_ip, d0_tr00, d0_wprt, d0_dskchg, d0_byte_ce, d0_f_index, d0_f_am, d0_f_id, d0_f_dam, d0_f_data, d0_f_data_last;
wire        d1_ready, d1_ip, d1_tr00, d1_wprt, d1_dskchg, d1_byte_ce, d1_f_index, d1_f_am, d1_f_id, d1_f_dam, d1_f_data, d1_f_data_last;
wire  [7:0] d0_rd_byte, d0_id_c, d0_id_h, d0_id_r, d0_id_n;
wire  [7:0] d1_rd_byte, d1_id_c, d1_id_h, d1_id_r, d1_id_n;
assign {fdd_ready, fdd_ip, fdd_tr00, fdd_wprt, fdd_dskchg} = sel1 ? {d1_ready, d1_ip, d1_tr00, d1_wprt, d1_dskchg}
                                                                : {d0_ready, d0_ip, d0_tr00, d0_wprt, d0_dskchg};
assign {fdd_byte_ce, fdd_f_index, fdd_f_am, fdd_f_id, fdd_f_dam, fdd_f_data, fdd_f_data_last} =
	sel1 ? {d1_byte_ce, d1_f_index, d1_f_am, d1_f_id, d1_f_dam, d1_f_data, d1_f_data_last}
	     : {d0_byte_ce, d0_f_index, d0_f_am, d0_f_id, d0_f_dam, d0_f_data, d0_f_data_last};
assign {fdd_rd_byte, fdd_id_c, fdd_id_h, fdd_id_r, fdd_id_n} = sel1 ? {d1_rd_byte, d1_id_c, d1_id_h, d1_id_r, d1_id_n}
                                                                    : {d0_rd_byte, d0_id_c, d0_id_h, d0_id_r, d0_id_n};

towns_fdd #(.FAST(FAST_FDD)) fdd
(
	.clk(clk), .ce_1m(ce_1m), .reset(reset),
	.select(sel0), .motor(fdc_motor), .side(fdc_side), .step(fdd_step), .dirc(fdd_dirc),
	.wg(fdd_wg), .fmt(fdd_fmt), .hispd(fdc_hispd), .modeb(fdc_modeb),
	.ready(d0_ready), .ip(d0_ip), .tr00(d0_tr00), .wprt(d0_wprt), .dskchg(d0_dskchg),
	.byte_ce(d0_byte_ce), .rd_byte(d0_rd_byte), .f_index(d0_f_index), .f_am(d0_f_am), .f_id(d0_f_id), .f_dam(d0_f_dam),
	.f_data(d0_f_data), .f_data_last(d0_f_data_last), .fdc_field(fdd_field), .id_c(d0_id_c), .id_h(d0_id_h), .id_r(d0_id_r), .id_n(d0_id_n),
	.wr_byte(fdd_wr_byte), .wr_en(fdd_wr_en),
	.img_present(fdd_present), .img_wp(fdd_wp), .img_mounted(fdd_mounted),
	.req_lba(fdd_lba), .req_rd(fdd_rd), .req_wr(fdd_wr), .blk_done(fdd_done), .blk_err(fdd_err),
	.buf_addr(fdd_buf_addr), .buf_we(fdd_buf_we), .buf_din(fdd_buf_din), .buf_dout(fdd_buf_dout),
	.ss_cs(ss_id == SS_FDD && !ss_reg[3]), .ss_wr(ss_wr), .ss_a(ss_reg[2:0]), .ss_din(lane_din), .ss_dout(ss_fdd_dout), .ss_quiet(fdd_quiet)
);

towns_fdd #(.FAST(FAST_FDD)) fdd2
(
	.clk(clk), .ce_1m(ce_1m), .reset(reset),
	.select(sel1), .motor(fdc_motor), .side(fdc_side), .step(fdd_step), .dirc(fdd_dirc),
	.wg(fdd_wg), .fmt(fdd_fmt), .hispd(fdc_hispd), .modeb(fdc_modeb),
	.ready(d1_ready), .ip(d1_ip), .tr00(d1_tr00), .wprt(d1_wprt), .dskchg(d1_dskchg),
	.byte_ce(d1_byte_ce), .rd_byte(d1_rd_byte), .f_index(d1_f_index), .f_am(d1_f_am), .f_id(d1_f_id), .f_dam(d1_f_dam),
	.f_data(d1_f_data), .f_data_last(d1_f_data_last), .fdc_field(fdd_field), .id_c(d1_id_c), .id_h(d1_id_h), .id_r(d1_id_r), .id_n(d1_id_n),
	.wr_byte(fdd_wr_byte), .wr_en(fdd_wr_en),
	.img_present(fdd2_present), .img_wp(fdd2_wp), .img_mounted(fdd2_mounted),
	.req_lba(fdd2_lba), .req_rd(fdd2_rd), .req_wr(fdd2_wr), .blk_done(fdd2_done), .blk_err(fdd2_err),
	.buf_addr(fdd2_buf_addr), .buf_we(fdd2_buf_we), .buf_din(fdd2_buf_din), .buf_dout(fdd2_buf_dout),
	.ss_cs(ss_id == SS_FDD && ss_reg[3]), .ss_wr(ss_wr), .ss_a(ss_reg[2:0]), .ss_din(lane_din), .ss_dout(ss_fdd2_dout), .ss_quiet(fdd2_quiet)
);

// ---- CMOS backup: the 8 KB image on its own slot ----
wire        cb_hold, bk_ram_we;
wire [17:0] cb_blk_lba;
wire  [7:0] bk_ram_din, bk_ram_dout;
wire [12:0] bk_ram_addr;
assign cmos_lba = cb_blk_lba[3:0];

towns_cmos_backup cmos_backup
(
	.clk(clk), .reset(reset),
	.cpu_wr(ce_cpu & ((c_cmos & lane_wr) | (cmos_io & io_wr))), .save(cmos_save), .hdd_present({hdd2_present, hdd_present}), .loading(cmos_loading),
	.ram_addr(bk_ram_addr), .ram_we(bk_ram_we), .ram_din(bk_ram_din), .ram_dout(bk_ram_dout),
	.img_present(cmos_present), .img_blocks(18'd16),
	.blk_hold(cb_hold), .blk_grant(cb_hold), .blk_lba(cb_blk_lba), .blk_rd(cmos_rd), .blk_wr(cmos_wr),
	.blk_done(cmos_done), .blk_err(cmos_err), .buf_addr(cmos_buf_addr), .buf_we(cmos_buf_we), .buf_din(cmos_buf_din), .buf_dout(cmos_buf_dout)
);


// ---- IC memory card: the card image on IDE0 drive 0 ----
wire        card_sel, card_hold;
wire  [7:0] card_dout;
reg         card_req;
wire        card_ack;
wire [15:0] card_mem_dout;

towns_iccard iccard
(
	.clk(clk), .ce(ce_16m), .reset(reset), .cpu_held(boot_hold),
	.io_addr(lane_addr), .io_rd(io_rd & ce_cpu), .io_wr(io_wr & ce_cpu), .io_din(lane_din), .io_dout(card_dout), .io_sel(card_sel),
	.mem_a(bus_a[19:1]), .mem_be(bm_be), .mem_we(bm_we), .mem_din(bm_dout), .mem_req(card_req),
	.mem_dout(card_mem_dout), .mem_ack(card_ack), .warn(card_warn),
	.img_present(card_present), .img_blocks(card_blocks),
	.blk_hold(card_hold), .blk_grant(card_hold), .blk_lba(card_lba), .blk_rd(card_rd), .blk_wr(card_wr),
	.blk_done(card_done), .blk_err(card_err), .buf_addr(card_buf_addr), .buf_we(card_buf_we), .buf_din(card_buf_din), .buf_dout(card_buf_dout),
	.ss_cs(ss_id == SS_CARD), .ss_wr(ss_wr), .ss_a(ss_reg[1:0]), .ss_din(lane_din), .ss_dout(ss_card_dout), .ss_quiet(card_quiet)
);

// ---- SCSI card: host interface on the rear port, two hard disks on its bus ----
// Fitted when a disk image is mounted.
wire        scsi_sel, scsi_irq, scsi_dma_req;
wire  [7:0] scsi_dout, scsi_dma_dout;
wire        sb_sel, sb_atn, sb_rst, sb_ack, sb_doe;
wire  [7:0] sb_d_init, sb_d_targ;   // data lines, initiator side and target side
wire        sd_hold, sd2_hold;

// a target raises its lines only while it holds BSY, so the bus is their OR
wire  [1:0] t_bsy, t_req, t_msg, t_cd, t_io;
wire  [7:0] t0_dout, t1_dout;
wire        sb_bsy = |t_bsy, sb_req = |t_req, sb_msg = |t_msg, sb_cd = |t_cd, sb_io = |t_io;
assign      sb_d_targ = t_bsy[1] ? t1_dout : t0_dout;

towns_scsi scsi
(
	.clk(clk), .ce(ce_dev), .reset(reset), .enable(hdd_present | hdd2_present),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(scsi_dout), .io_sel(scsi_sel), .irq(scsi_irq),
	.dma_req(scsi_dma_req), .dma_ack(~dma_dack_n[1]), .dma_iord(~dma_iord_n), .dma_iowr(~dma_iowr_n),
	.dma_din(dma_mem_byte), .dma_dout(scsi_dma_dout),
	.bus_sel(sb_sel), .bus_atn(sb_atn), .bus_rst(sb_rst), .bus_ack(sb_ack), .bus_dout(sb_d_init), .bus_doe(sb_doe),
	.bus_bsy(sb_bsy), .bus_req(sb_req), .bus_msg(sb_msg), .bus_cd(sb_cd), .bus_io(sb_io), .bus_din(sb_d_targ),
	.ss_cs(ss_id == SS_SCSI), .ss_wr(ss_wr), .ss_a(ss_reg[0]), .ss_din(lane_din), .ss_dout(ss_scsi_dout)
);

scsi_hdd #(.ID(0), .LBA_W(22)) scsi_disk
(
	.clk(clk), .ce(ce_dev), .reset(reset),
	.present(hdd_present), .blocks(hdd_blocks),
	.bus_sel(sb_sel), .bus_atn(sb_atn), .bus_rst(sb_rst), .bus_ack(sb_ack), .bus_din(sb_d_init), .bus_doe(sb_doe),
	.bus_bsy(t_bsy[0]), .bus_req(t_req[0]), .bus_msg(t_msg[0]), .bus_cd(t_cd[0]), .bus_io(t_io[0]), .bus_dout(t0_dout),
	.blk_hold(sd_hold), .blk_grant(sd_hold), .blk_lba(hdd_lba), .blk_rd(hdd_rd), .blk_wr(hdd_wr),
	.blk_done(hdd_done), .blk_err(hdd_err), .buf_addr(hdd_buf_addr), .buf_we(hdd_buf_we), .buf_din(hdd_buf_din), .buf_dout(hdd_buf_dout),
	.ss_cs(ss_id == SS_HDD && !ss_reg[1]), .ss_wr(ss_wr), .ss_a(ss_reg[0]), .ss_din(lane_din), .ss_dout(ss_hdd_dout), .ss_quiet(hdd_quiet)
);

scsi_hdd #(.ID(1), .LBA_W(22)) scsi_disk2
(
	.clk      (clk),
	.ce       (ce_dev),
	.reset    (reset),

	.present  (hdd2_present),
	.blocks   (hdd2_blocks),

	.bus_sel  (sb_sel),
	.bus_atn  (sb_atn),
	.bus_rst  (sb_rst),
	.bus_ack  (sb_ack),
	.bus_din  (sb_d_init),
	.bus_doe  (sb_doe),
	.bus_bsy  (t_bsy[1]),
	.bus_req  (t_req[1]),
	.bus_msg  (t_msg[1]),
	.bus_cd   (t_cd[1]),
	.bus_io   (t_io[1]),
	.bus_dout (t1_dout),

	.blk_hold (sd2_hold),
	.blk_grant(sd2_hold),
	.blk_lba  (hdd2_lba),
	.blk_rd   (hdd2_rd),
	.blk_wr   (hdd2_wr),
	.blk_done (hdd2_done),
	.blk_err  (hdd2_err),
	.buf_addr (hdd2_buf_addr),
	.buf_we   (hdd2_buf_we),
	.buf_din  (hdd2_buf_din),
	.buf_dout (hdd2_buf_dout),

	.ss_cs    (ss_id == SS_HDD && ss_reg[1]),
	.ss_wr    (ss_wr),
	.ss_a     (ss_reg[0]),
	.ss_din   (lane_din),
	.ss_dout  (ss_hdd2_dout),
	.ss_quiet (hdd2_quiet)
);


// ---- keyboard controller ----
wire        kbd_sel, kbd_irq;
wire  [7:0] kbd_dout;

towns_keyboard #(.CLK_HZ(CLK_HZ)) keyboard
(
	.clk(clk), .ce(ce_dev), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(kbd_dout), .io_sel(kbd_sel), .irq(kbd_irq),
	.ss_cs(ss_id == SS_KBD), .ss_wr(ss_wr), .ss_a(ss_reg[4:0]), .ss_din(lane_din), .ss_dout(ss_kbd_dout),
	.ps2_key(ps2_key),
	.ps2_key_raw(ps2_key_raw)
);

// ---- pad ports ----
wire        pad_sel;
wire  [7:0] pad_dout;

towns_padport #(.CLK_HZ(CLK_HZ)) padport
(
	.clk(clk), .ce(ce_dev), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(pad_dout), .io_sel(pad_sel),
	.ss_cs(ss_id == SS_PAD), .ss_wr(ss_wr), .ss_a(ss_reg[4:0]), .ss_din(lane_din), .ss_dout(ss_pad_dout),
	.pad1(pad1), .pad2(pad2), .pad1_type(pad1_type), .pad2_type(pad2_type),
	.ps2_mouse(ps2_mouse),
	.pad1_stick(pad1_stick), .pad2_stick(pad2_stick),
	.pad1_throttle(pad1_throttle), .pad2_throttle(pad2_throttle)
);

// ---- CD-ROM ----
// Device-to-memory on channel 3: the CDC hands out a byte or a word per
// IORD strobe and sees terminal count with it.
wire        cd_sel, cd_irq, cd_dma_req;
wire  [7:0] cd_dout;
wire [15:0] cd_dma_dout;
wire        cd_dma_ack  = ~dma_dack_n[3];
wire        cd_dma_word = ~dma_ube_n & ~dma_a[0];
wire        cd_dma_tc   = ~dma_tc_n & dma_tc_oe;

wire signed [15:0] cdda_l, cdda_r;

towns_cdrom #(.CLK_RATE(CLK_HZ)) cdrom
(
	.clk(clk), .ce(ce_dev), .ce_16m(ce_16m), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(cd_dout), .io_sel(cd_sel), .irq(cd_irq),
	.dma_req(cd_dma_req), .dma_ack(cd_dma_ack), .dma_iord(~dma_iord_n), .dma_word(cd_dma_word),
	.dma_tc(cd_dma_tc), .dma_dout(cd_dma_dout),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
	.img_present(cd_present), .img_mounted(cd_mounted),
	.blk_lba(cd_lba), .blk_bank(cd_bank), .blk_rd(cd_rd), .blk_done(cd_done), .blk_err(cd_err),
	.buf_addr(cd_buf_addr), .buf_dout(cd_buf_dout),
	.cdda_ce(), .cdda_l(cdda_l), .cdda_r(cdda_r),
	.ss_hold(ss_mode), .ss_cs_cdc(ss_id == SS_CDC), .ss_cs_mpu(ss_id == SS_CDMPU), .ss_cs_host(ss_id == SS_CDHOST),
	.ss_wr(ss_wr), .ss_step(ss_step), .ss_a(ss_reg[6:0]), .ss_din(lane_din),
	.ss_dout_cdc(ss_cdc_dout), .ss_dout_mpu(ss_cdmpu_dout), .ss_dout_host(ss_cdhost_dout),
	.ss_quiet(cd_quiet), .ss_busy(cd_busy)
);


// ---- sound: FM, PCM, mixer ----
// Both chips take 8 MHz (the board's 32 MHz oscillator divided by four,
// the only clean derivation; Databook figures differ, see the evidence).
// The FM chip wants its clock as a level: PHI toggles at 16 MHz. The
// clocks keep running through reset, as the FM chip's reset pin needs
// them; any starting state of the dividers is fine.
localparam [27:0] HZ_8M = 28'd8000000;
reg  [27:0] acc8, acc16;
reg         ce_8m, fm_phi;
always @(posedge clk) begin
	ce_8m <= 0;
	if (run || fm_replaying) begin
		if (acc8 + HZ_8M >= CLK_HZ[27:0]) begin acc8 <= acc8 + HZ_8M - CLK_HZ[27:0]; ce_8m <= 1; end else acc8 <= acc8 + HZ_8M;
		if (acc16 + HZ_16M >= CLK_HZ[27:0]) begin acc16 <= acc16 + HZ_16M - CLK_HZ[27:0]; fm_phi <= ~fm_phi; end else acc16 <= acc16 + HZ_16M;
	end
end

// 04D8/04DA/04DC/04DE: A1 = a2, A0 = a1 of the chip
wire        fm_cs = c_io && (lane_addr[15:3] == 13'h009B) && !lane_addr[0];   // 04D8-04DE
wire  [7:0] fm_dout;
wire  [8:0] fm_mol, fm_mor;
wire  [2:0] fm_dac_idx;
wire        fm_dac_en, fm_irq_n, fm_sample;
wire signed [11:0] fm_l, fm_r;

// The savestate shadow of the chip's registers drives its pins while it
// replays them; the guest's writes are recorded as they pass.
wire        fm_replaying, fm_rep_cs, fm_rep_wr;
wire  [1:0] fm_rep_a;
wire  [7:0] fm_rep_din, ss_fm_dout;
reg         fm_wr_q;
always @(posedge clk) fm_wr_q <= io_wr & fm_cs & ce_cpu;
assign ss_dev_busy = fm_replaying | cd_busy;

ss_fm_shadow fm_shadow
(
	.clk(clk), .reset(reset),
	.wr(io_wr & fm_cs & ce_cpu & ~fm_wr_q), .a({lane_addr[2], lane_addr[1]}), .din(lane_din),
	.ss_cs(ss_id == SS_FM), .ss_wr(ss_wr), .ss_step(ss_step), .ss_a(ss_reg), .ss_din(lane_din), .ss_dout(ss_fm_dout),
	.replaying(fm_replaying), .rep_cs(fm_rep_cs), .rep_wr(fm_rep_wr), .rep_a(fm_rep_a), .rep_din(fm_rep_din)
);

ym3438 fm
(
	.MCLK(clk), .PHI(fm_phi),
	.DATA_i(fm_replaying ? fm_rep_din : lane_din), .DATA_o(fm_dout), .DATA_o_z(),
	.TEST_i(1'b0), .TEST_o(), .TEST_o_z(),
	.IC(~reset), .CS(fm_replaying ? ~fm_rep_cs : ~fm_cs), .WR(fm_replaying ? ~fm_rep_wr : ~io_wr), .RD(~io_rd),
	.ADDRESS(fm_replaying ? fm_rep_a : {lane_addr[2], lane_addr[1]}),
	.IRQ(fm_irq_n), .MOL(fm_mol), .MOR(fm_mor), .MOL_2612(), .MOR_2612(),
	.fm_clk1(), .DAC_ch_index(fm_dac_idx), .DAC_out_enable(fm_dac_en),
	.ym2612_status_enable(1'b0)
);

ym3438_dac fm_dac
(
	.clk(clk), .reset(reset),
	.mol(fm_mol), .mor(fm_mor), .ch_index(fm_dac_idx), .out_enable(fm_dac_en),
	.out_l(fm_l), .out_r(fm_r), .sample(fm_sample)
);

// 04F0-04F8 are the chip registers; the F80000 window lands on its A12
wire        pcm_reg = c_io && (lane_addr[15:4] == 12'h04F) && (lane_addr[3:0] <= 4'h8);
wire        pcm_cs  = pcm_reg | c_pcm;
wire [12:0] pcm_a   = c_pcm ? lane_addr[12:0] : {9'd0, lane_addr[3:0]};
wire  [7:0] pcm_dout;
wire        pcm_oe, pcm_sample, pcm_bank_ev;
wire  [3:0] pcm_bank;
wire signed [15:0] pcm_raw_l, pcm_raw_r, pcm_l, pcm_r;

rf5c68 pcm
(
	.clk(clk), .ce(ce_8m), .reset_n(~reset),
	.a(pcm_a), .cs_n(~pcm_cs), .rd_n(~io_rd), .wr_n(~io_wr),
	.d_i(lane_din), .d_o(pcm_dout), .d_oe(pcm_oe),
	.dac_l(pcm_raw_l), .dac_r(pcm_raw_r), .sample(pcm_sample), .sounding(),
	.bank_ev(pcm_bank_ev), .bank_ev_bank(pcm_bank),
	.ss_cs(ss_id == SS_PCM), .ss_wr(ss_wr), .ss_step(ss_step), .ss_a(ss_reg), .ss_din(lane_din), .ss_dout(ss_pcm_dout)
);

lc7881 pcm_dac
(
	.clk(clk), .reset(reset), .strobe(pcm_sample),
	.d_l(pcm_raw_l), .d_r(pcm_raw_r), .out_l(pcm_l), .out_r(pcm_r)
);

wire        mix_sel, snd_irq;
wire  [7:0] mix_dout;

towns_audio_mixer mixer
(
	.clk(clk), .ce(ce_dev), .ce_16m(ce_16m), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(mix_dout), .io_sel(mix_sel),
	.ss_cs(ss_id == SS_MIXER), .ss_wr(ss_wr), .ss_a(ss_reg[4:0]), .ss_din(lane_din), .ss_dout(ss_mix_dout),
	.fm_l(fm_l), .fm_r(fm_r), .fm_irq_n(fm_irq_n),
	.pcm_l(pcm_l), .pcm_r(pcm_r), .pcm_bank_ev(pcm_bank_ev), .pcm_bank(pcm_bank),
	.cd_l(cdda_l), .cd_r(cdda_r), .beep(beep),
	.irq13(snd_irq), .led_off(),
	.out_l(audio_l), .out_r(audio_r)
);

// ---- MIDI card ----
wire        midi_sel, midi_ser_irq, midi_tmr_irq;
wire  [7:0] midi_dout;

towns_midi midi
(
	.clk(clk), .ce(ce_16m), .reset(reset), .enable(midi_en),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(midi_dout), .io_sel(midi_sel),
	.midi_tx(midi_tx), .midi_rx(midi_rx),
	.irq_serial(midi_ser_irq), .irq_timer(midi_tmr_irq)
);

// ---- RS-232C with the modem card ----
wire        rs_sel, rs_irq;
wire  [7:0] rs_dout;

towns_rs232 #(.CLK_HZ(CLK_HZ)) rs232
(
	.clk(clk), .reset(reset), .enable(rs_en),
	.io_addr(lane_addr), .io_rd(io_rd), .io_wr(io_wr), .io_din(lane_din),
	.io_dout(rs_dout), .io_sel(rs_sel),
	.baud(rs_baud),
	.txd(rs_txd), .rxd(rs_rxd), .rts(rs_rts), .dtr(rs_dtr),
	.cts(rs_cts), .dsr(rs_dsr), .cd(rs_cd), .ci(1'b0),
	.irq(rs_irq)
);

// ---- video: CRTC, VRAM controller, sprite engine, Marty output stage ----
// Dot clocks for the four CLKSEL values, the 28.6 MHz engine clock and
// the 14.3 MHz output sample rate, all as fractional enables of clk.
localparam [27:0] HZ_28M = 28'd28636300, HZ_24M = 28'd24545400, HZ_25M = 28'd25175000, HZ_21M = 28'd21052500,
                  HZ_14M = 28'd14318150;
reg  [27:0] acc28, acc24, acc25, acc21, acc14;
reg   [3:0] ce_clk;
reg         ce_28m, ce_14m;
always @(posedge clk) begin
	if (reset) begin
		{acc28, acc24, acc25, acc21, acc14} <= 0;
		ce_clk <= 0; ce_28m <= 0; ce_14m <= 0;
	end
	else begin
		ce_clk <= 0; ce_28m <= 0; ce_14m <= 0;
		if (acc28 + HZ_28M >= CLK_HZ[27:0]) begin acc28 <= acc28 + HZ_28M - CLK_HZ[27:0]; ce_clk[0] <= 1; ce_28m <= 1; end else acc28 <= acc28 + HZ_28M;
		if (acc24 + HZ_24M >= CLK_HZ[27:0]) begin acc24 <= acc24 + HZ_24M - CLK_HZ[27:0]; ce_clk[1] <= 1; end else acc24 <= acc24 + HZ_24M;
		if (acc25 + HZ_25M >= CLK_HZ[27:0]) begin acc25 <= acc25 + HZ_25M - CLK_HZ[27:0]; ce_clk[2] <= 1; end else acc25 <= acc25 + HZ_25M;
		if (acc21 + HZ_21M >= CLK_HZ[27:0]) begin acc21 <= acc21 + HZ_21M - CLK_HZ[27:0]; ce_clk[3] <= 1; end else acc21 <= acc21 + HZ_21M;
		if (acc14 + HZ_14M >= CLK_HZ[27:0]) begin acc14 <= acc14 + HZ_14M - CLK_HZ[27:0]; ce_14m <= 1; end else acc14 <= acc14 + HZ_14M;
	end
end

wire  [7:0] crtc_dout, spr_io_dout, vram_io_dout;
wire        crtc_sel, spr_io_sel, vram_io_sel;
wire        sp_busy, sp_active, sp_page, sp_disp_page, vsync_irq;
wire  [3:0] fmr_plane_mask;
wire        fmr_ps2;
wire        fetch_req, fetch_ack, fetch_valid;
wire [18:3] fetch_a;
wire [63:0] fetch_data;
wire        dot_ce, crtc_hs, crtc_vs, crtc_de, crtc_field, crtc_line_start, in_hsync, in_vsync;
wire  [7:0] crtc_r, crtc_g, crtc_b;
wire        sp_vw_req, sp_vw_ack, sp_vw_idle, sp_clr_req, sp_clr_page, sp_clr_done;
wire [18:1] sp_vw_a;
wire [15:0] sp_vw_din;

// CPU side of the VRAM controller and the sprite RAM
// VRAM cycle: requested from the ADS T-state like the SDRAM path, with the
// address and data straight off the CPU pins, which hold until READY. The
// controller's ack ends the cycle once the wait floor has run out.
reg         fmr_req;
wire [15:0] vram_cpu_dout;
wire        vram_cpu_ack, fmr_ack;
reg         vram_rdy;      // the ack has landed for the cycle in progress
wire        t1_vram  = (state == S_IDLE) && !hlda && (!ads_n || pipe_pend) && m_io_n && !is_inta && !is_halt && sel_vram;
wire        vram_cpu_req = (t1_vram || state == S_VRAM) && !vram_rdy;
wire        vram_done = (state == S_VRAM) && (waits == 0) && (vram_rdy || vram_cpu_ack);
reg  [14:0] fmr_a;
reg         fmr_we;
reg   [7:0] fmr_din;
wire  [7:0] fmr_dout;
reg  [16:1] spr_ram_a;
reg   [1:0] spr_ram_be;
reg         spr_ram_we;
reg  [15:0] spr_ram_din;
wire [15:0] spr_ram_dout;
reg         tvram_wr;
wire [10:0] crtc_t_hcnt, crtc_t_vcnt;

towns_crtc crtc
(
	.dbg_regs(dbg_crtc_regs),
	.clk(clk), .reset(reset), .ce_clk(ce_clk & {4{run}}),
	.ss_quiet(crtc_quiet),
	.io_addr(lane_addr), .io_rd(io_rd & ce_cpu), .io_wr(io_wr & ce_cpu), .io_din(lane_din),
	.io_dout(crtc_dout), .io_sel(crtc_sel),
	.ss_cs(ss_id == SS_CRTC), .ss_wr(ss_wr), .ss_step(ss_step), .ss_a(ss_reg), .ss_din(lane_din), .ss_dout(ss_crtc_dout),
	.sp_busy(sp_busy), .sp_page(sp_page), .sp_disp_page(sp_disp_page),
	.fmr_plane_mask(fmr_plane_mask), .fmr_ps2(fmr_ps2),
	.fetch_req(fetch_req), .fetch_a(fetch_a), .fetch_ack(fetch_ack), .fetch_data(fetch_data), .fetch_valid(fetch_valid),
	.pull(vo_pull), .ce_pull(ce_28m & run), .pull_frame(vo_frame), .pull_field(vo_field), .pull_line(vo_line), .pull_skip(vo_skip), .sync_in(vo_sync), .follow_fa(vo_follow),
	.r_ready(vo_ready), .r_ack(vo_ack), .r_ce(vo_ce), .r_x(vo_x),
	.geo_hst(geo_hst), .geo_clksel(geo_clksel), .geo_vds(geo_vds), .geo_vde(geo_vde), .geo_hds(geo_hds), .geo_hde(geo_hde), .geo_vst(geo_vst), .geo_zv1(geo_zv1),
	.dot_ce(dot_ce), .r(crtc_r), .g(crtc_g), .b(crtc_b), .hs(crtc_hs), .vs(crtc_vs), .de(crtc_de),
	.field(crtc_field), .line_start(crtc_line_start), .vsync_irq(vsync_irq),
	.in_hsync(in_hsync), .in_vsync(in_vsync),
	.t_hcnt(crtc_t_hcnt),
	.t_vcnt(crtc_t_vcnt)
);

towns_vram_ctrl vram_ctrl
(
	.clk(clk), .reset(reset), .raw(ss_act),
	.ss_cs(ss_id == SS_VRAM), .ss_wr(ss_wr), .ss_a(ss_reg[3:0]), .ss_din(lane_din), .ss_dout(ss_vram_dout),
	.cpu_req(vram_cpu_req), .cpu_a(bus_a[20:1]), .cpu_be(bm_be), .cpu_we(bm_we),
	.cpu_din(bm_dout), .cpu_dout(vram_cpu_dout), .cpu_ack(vram_cpu_ack),
	.fmr_req(fmr_req), .fmr_a(fmr_a), .fmr_we(fmr_we), .fmr_din(fmr_din), .fmr_dout(fmr_dout), .fmr_ack(fmr_ack),
	.io_addr(lane_addr), .io_rd(io_rd & ce_cpu), .io_wr(io_wr & ce_cpu), .io_din(lane_din),
	.io_dout(vram_io_dout), .io_sel(vram_io_sel), .tvram_wr(tvram_wr),
	.in_hsync(in_hsync), .in_vsync(in_vsync),
	.fmr_plane_mask(fmr_plane_mask), .fmr_ps2(fmr_ps2),
	.fmr_ank_font(fmr_ank_font),
	.sp_req(sp_vw_req), .sp_a(sp_vw_a), .sp_din(sp_vw_din), .sp_ack(sp_vw_ack), .sp_idle(sp_vw_idle),
	.clr_req(sp_clr_req), .clr_page(sp_clr_page), .clr_done(sp_clr_done),
	.fetch_req(fetch_req), .fetch_a(fetch_a), .fetch_ack(fetch_ack), .fetch_data(fetch_data), .fetch_valid(fetch_valid),
	.dbg_bm_en(dbg_bm_en), .dbg_bm_addr(dbg_bm_addr), .dbg_bm_q(dbg_bm_q),
	.vf_a(vf_a), .vf_req(vf_req), .vf_accept(vf_accept), .vf_dout(vf_dout), .vf_ready(vf_ready),
	.vr_a(vr_a), .vr_be(vr_be), .vr_we(vr_we), .vr_din(vr_din), .vr_dout(vr_dout), .vr_req(vr_req), .vr_ready(vr_ready)
);

towns_sprite sprite
(
	.clk(clk), .ce(ce_28m & run), .reset(reset),
	.io_addr(lane_addr), .io_rd(io_rd & ce_cpu), .io_wr(io_wr & ce_cpu), .io_din(lane_din),
	.io_dout(spr_io_dout), .io_sel(spr_io_sel),
	.ss_cs(ss_id == SS_SPRITE), .ss_wr(ss_wr), .ss_a(ss_reg[2:0]), .ss_din(lane_din), .ss_dout(ss_spr_dout),
	.ram_a(spr_ram_a), .ram_be(spr_ram_be), .ram_we(spr_ram_we), .ram_din(spr_ram_din), .ram_dout(spr_ram_dout),
	.vsync(in_vsync), .fast(sprite_fast),
	.vw_req(sp_vw_req), .vw_a(sp_vw_a), .vw_din(sp_vw_din), .vw_ack(sp_vw_ack), .vw_idle(sp_vw_idle),
	.clr_req(sp_clr_req), .clr_page(sp_clr_page), .clr_done(sp_clr_done),
	.busy(sp_busy), .active(sp_active), .page(sp_page), .disp_page(sp_disp_page)
);

// The scan converter pulls rendered lines through the CRTC at its own pace
wire        vo_pull, vo_frame, vo_field, vo_line, vo_skip, vo_sync, vo_follow, vo_ready, vo_ack, vo_ce;
wire [10:0] vo_x, geo_hst, geo_vds, geo_vde, geo_hds, geo_hde, geo_vst;
wire        geo_zv1;
wire  [1:0] geo_clksel;

marty_video_out #(.CLK_HZ(CLK_HZ)) video_out
(
	.clk(clk), .reset(reset), .ce_14m(ce_14m), .ce_25m(ce_clk[2]), .ce_28m(ce_28m), .mode(video_mode), .show_blank(show_blank),
	.pull(vo_pull), .pull_frame(vo_frame), .pull_field(vo_field), .pull_line(vo_line), .pull_skip(vo_skip), .sync_out(vo_sync), .follow_fa(vo_follow),
	.r_ready(vo_ready), .r_ack(vo_ack), .r_ce(vo_ce), .r_x(vo_x),
	.r_i(crtc_r), .g_i(crtc_g), .b_i(crtc_b), .de_i(crtc_de),
	.dot_ce(dot_ce), .hs_i(crtc_hs), .vs_i(crtc_vs), .field_i(crtc_field), .src_vs(in_vsync),
	.t_hcnt(crtc_t_hcnt),
	.t_vcnt(crtc_t_vcnt),
	.geo_hst(geo_hst), .geo_clksel(geo_clksel), .geo_vds(geo_vds), .geo_vde(geo_vde), .geo_hds(geo_hds), .geo_hde(geo_hde), .geo_vst(geo_vst), .geo_zv1(geo_zv1),
	.ce_pix(ce_pix), .r_o(vid_r), .g_o(vid_g), .b_o(vid_b),
	.hs_o(vid_hs), .vs_o(vid_vs), .hb_o(vid_hb), .vb_o(vid_vb), .field_o(vid_field)
);

// I/O 3000-3FFF reaches the first 2 KB, one byte per even address.
wire [12:0] cmos_addr = c_cmos ? {c_addr[12:1], lane} : {2'd0, lane_addr[11:1]};

towns_cmos #(.SIM_INIT_FILE(CMOS_INIT)) cmos_ram
(
	.clk(clk),
	.ce(ce_cpu),
	.addr(cmos_addr),
	.wr((c_cmos & lane_wr) | (cmos_io & io_wr)),
	.din(lane_din),
	.dout(cmos_dout),
	.mem8(ram_size == 2'd3),
	.bk_addr(bk_ram_addr), .bk_wr(bk_ram_we), .bk_din(bk_ram_din), .bk_dout(bk_ram_dout)
);

always @(posedge clk) begin
	if (cpu_reset) begin
		state <= S_IDLE;
		wait_debt <= 0;
		wait_shaved <= 0;
		pipe_pend <= 0;
		ready_n <= 1;
		mem_req_r <= 0;
		dbg_io_unmapped <= 0;
		dma_ready <= 0;
		dma_cyc <= 0;
		dma_mem_byte <= 8'hFF;
		dma_snoop <= 0;
		dma_snoop_a <= 23'd0;
		vram_rdy <= 0;
		wide_q <= 0;
		fmr_req <= 0;
		card_req <= 0;
		spr_ram_we <= 0;
		tvram_wr <= 0;
		c_fmrreg <= 0;
		c_pcm <= 0;
	end
	else begin
		// the VRAM controller answers in its own time; the CPU waits in S_VRAM/S_FMR
		if (vram_cpu_ack) vram_rdy <= 1;
		if (fmr_ack) fmr_req <= 0;
		if (card_ack) card_req <= 0;
		spr_ram_we <= 0;
		tvram_wr <= 0;
	if (ce_cpu) begin
		mem_req_r <= 0;
		dbg_io_unmapped <= 0;
		ready_n <= 1;
		dma_snoop <= 0;

		// the DMAC holds its strobe until READY; dropping it ends the transfer
		if (!dma_mem_strobe) dma_ready <= 0;
		// a DMAC cycle ends where the CPU would have sampled READY: hand the
		// device its byte and answer the controller
		if (dma_cyc && state == S_IDLE) begin
			dma_mem_byte <= dma_a[0] ? cpu_d_i[15:8] : cpu_d_i[7:0];
			dma_ready    <= 1;
			dma_cyc      <= 0;
		end

		case (state)
		// a cycle starts on the CPU's ADS (or its pipelined successor) or on
		// the DMAC's memory strobe; from here on both look the same
		S_IDLE: if (hlda ? dma_start : (!ads_n || pipe_pend)) begin
			pipe_pend <= 0;
			dma_cyc <= hlda;
			c_io   <= ~bm_mem;
			c_hidden <= ss_act & ss_bus_hidden;
			c_wr   <= bm_we;
			c_inta <= is_inta;
			c_cmos <= sel_cmos & bm_mem;
			c_fmrreg <= fmr_regs & bm_mem;
			c_pcm  <= sel_pcm & bm_mem;
			c_be   <= bm_be;
			c_addr <= (fmr_regs & bm_mem) ? {8'hFF, 1'b1, bus_a[6:1], 1'b0} :
			          (sel_pcm & bm_mem)  ? {4'h1, bus_a[11:1], 1'b0} : {bus_a[15:1], 1'b0};
			if (is_halt) begin
				waits <= {1'b0, WAIT_RAM[3:0]};
				state <= S_DONE;
			end
			else if (font_rd) begin
				waits <= {1'b0, WAIT_ROM[3:0]};   // one glyph row, read like ROM
				state <= S_MEM;
			end
			else if (!bm_mem || sel_cmos || fmr_regs || sel_pcm) begin
				// byte devices: low lane first unless only the high byte is wanted
				state <= bm_be[0] ? S_BYTE0 : S_BYTE1;
				waits <= (!bm_mem && bm_we && bus_a[15:1] == 15'h0036) ? WAIT_LANE + WAIT_1US : {1'b0, WAIT_LANE};
			end
			else if (sel_sprite || fmr_tvram) begin
				// pattern RAM: one word, data the next cycle
				spr_ram_a   <= sel_sprite ? bus_a[16:1] : {2'b00, bus_a[14:1]};
				spr_ram_be  <= bm_be;
				spr_ram_we  <= bm_we;
				spr_ram_din <= bm_dout;
				tvram_wr    <= fmr_tvram & bm_we;
				waits <= {1'b0, WAIT_RAM[3:0]};
				state <= S_SPR;
			end
			else if (sel_card) begin
				card_req <= 1;
				state <= S_CARD;
			end
			else if (sel_sswin) begin
				sswin_a <= bus_a[11:1];
				waits   <= {1'b0, WAIT_ROM[3:0]};
				state   <= S_SSWIN;
			end
			else if (sel_vram) begin
				waits <= {1'b0, WAIT_VRAM[3:0]};
				state <= S_VRAM;
			end
			else if (fmr_planes) begin
				// plane window: one byte per request, low lane first
				fmr_a   <= {bus_a[14:1], ~bm_be[0]};
				fmr_we  <= bm_we;
				fmr_din <= bm_be[0] ? bm_dout[7:0] : bm_dout[15:8];
				fmr_req <= 1;
				state <= bm_be[0] ? S_FMR : S_FMR1;
			end
			else if (hlda && sel_sdram) begin
				// the DMAC's cycle keeps a registered request; its waits are
				// its own, not the CPU's debt
				mem_a_r   <= sdram_a;
				mem_be_r  <= dma_be;
				mem_we_r  <= ~dma_mwr_n & ~sel_rom & ~nmi_vec_hit;
				mem_din_r <= dma_dev_din;
				mem_req_r <= 1;
				waits     <= 5'd2;
				wait_shaved <= 0;
				dma_snoop   <= ~dma_mwr_n & ~sel_rom;
				dma_snoop_a <= dma_a_sx;
				state <= S_MEM;
			end
			else if (sel_sdram) begin
				wait_shaved <= (wait_debt != 0 && mem_waits != 0);
				if (wait_debt != 0 && mem_waits != 0) begin
					waits     <= {1'b0, mem_waits - 1'd1};
					wait_debt <= wait_debt - 1'd1;
				end
				else waits <= {1'b0, mem_waits};
				if (w_r_n && nmi_vec_hit) begin
					dma_snoop   <= 1;
					dma_snoop_a <= a;
				end
				state   <= S_MEM;
			end
			else begin
				// nothing there: open bus
				waits <= {1'b0, WAIT_RAM[3:0]};
				state <= S_DONE;
			end
		end

		S_MEM: begin
			if (!ads_n) pipe_pend <= 1;              // the next cycle, in T2P
			if (waits != 0) waits <= waits - 1'd1;
			else if (ans_avail) begin
				cpu_d_i <= ans_data;
				state   <= S_IDLE;
			end
			else if (wait_shaved) wait_shaved <= 0;   // the shaved wait, not a late answer
			else if (!no_waits && !dma_cyc && wait_debt != 3'd7) wait_debt <= wait_debt + 1'd1;
		end

		S_SPR: begin
			if (waits != 0) waits <= waits - 1'd1;
			else begin
				cpu_d_i <= spr_ram_dout;
				ready_n <= 0;
				state   <= S_IDLE;
			end
		end

		S_SSWIN: begin
			if (waits != 0) waits <= waits - 1'd1;
			else begin
				cpu_d_i <= sswin_dout;
				ready_n <= 0;
				state   <= S_IDLE;
			end
		end

		S_VRAM: begin
			if (waits != 0) waits <= waits - 1'd1;
			else if (vram_rdy || vram_cpu_ack) begin
				cpu_d_i  <= vram_cpu_dout;
				vram_rdy <= 0;
				state    <= S_IDLE;
			end
		end

		// the card answers when its cache has the block
		S_CARD: if (!card_req) begin
			cpu_d_i <= card_mem_dout;
			ready_n <= 0;
			state   <= S_IDLE;
		end

		// plane window low byte, then the high byte when the cycle wants both
		S_FMR: if (!fmr_req) begin
			byte_lo <= fmr_dout;
			if (c_be[1]) begin
				fmr_a[0] <= 1;
				fmr_din  <= bm_dout[15:8];
				fmr_req  <= 1;
				state    <= S_FMR1;
			end
			else begin
				cpu_d_i <= {8'hFF, fmr_dout};
				ready_n <= 0;
				state   <= S_IDLE;
			end
		end

		S_FMR1: if (!fmr_req) begin
			cpu_d_i <= {fmr_dout, c_be[0] ? byte_lo : 8'hFF};
			ready_n <= 0;
			state   <= S_IDLE;
		end

		S_BYTE0: begin
			if (io_rd) dbg_io_unmapped <= ~io_sel;
			byte_lo <= byte_dout;
			byte_hi <= io_dout_hi;
			wide_q  <= io_wide && c_io && !c_wr;
			if (c_be[1]) state <= S_BYTE1;
			else begin
				cpu_d_i <= {8'hFF, byte_dout};
				state <= S_DONE;
			end
		end

		S_BYTE1: begin
			if (io_rd) dbg_io_unmapped <= ~io_sel;
			cpu_d_i <= {wide_q ? byte_hi : byte_dout, c_be[0] ? byte_lo : 8'hFF};
			state <= S_DONE;
		end

		S_DONE: begin
			if (waits != 0) waits <= waits - 1'd1;
			else state <= S_IDLE;   // READY goes out through byte_done in this state
		end

		default: state <= S_IDLE;
		endcase
	end
	end
end

endmodule

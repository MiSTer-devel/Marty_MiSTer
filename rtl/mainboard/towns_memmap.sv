// Copyright (c) 2026 Jamie Blanks
//
// Marty physical address decode (24-bit SX bus) to the SDRAM layout and
// the on-chip windows. ROM placement follows the board (MAME marty_mem,
// Tsugaru townsdef.h); the SDRAM offsets are this core's own layout.
//
//   000000-1FFFFF  main DRAM (2 MB)          -> SDRAM 000000
//   200000-3FFFFF  DRAM expansion (4 MB option)
//   400000-5FFFFF  DRAM expansion (6 MB option) -> SDRAM A00000, clear of the ROMs
//   600000-7FFFFF  DRAM expansion (8 MB option) -> SDRAM C00000, except CPU reads with
//                  paging off, which see the OS ROM and EX ROM 0-2 as before
//     0C0000-0C7FFF  FMR-compatible VRAM plane window unless 0x404 MAIN MEM
//     0C8000-0CFFFF  FMR text VRAM and CFF80 registers, same switch
//     0D0000-0D7FFF  dictionary ROM bank (0x484) when 0x480 bit 0, same switch
//     0D8000-0D9FFF  CMOS RAM, same switches
//     0DA000-0EFFFF  nothing while 0x404 MAIN MEM is 0
//     0F8000-0FFFFF  last 32 KB of SYSTEM ROM unless 0x480 RAM
//   600000-67FFFF  OS ROM        (FMT_DOS)  -> SDRAM 400000
//   680000-87FFFF  EX ROM 0-3    (MAR_EX*)  -> SDRAM 680000
//   A00000-A7FFFF  VRAM, repeated to BFFFFF  -> SDRAM 900000
//   C00000-C1FFFF  sprite RAM (block RAM)
//   D00000-DFFFFF  IC card window (towns_iccard)
//   F40000-F41FFF  CMOS RAM, 386SX native window
//   E80000-EFFFFF  dictionary ROM (FMT_DIC) -> SDRAM 500000
//   F00000-F3FFFF  font ROM      (FMT_FNT)  -> SDRAM 580000
//   F80000-F80FFF  PCM wave RAM window, 4 KB bank of the RF5C68
//   FC0000-FFFFFF  SYSTEM ROM    (FMT_SYS)  -> SDRAM 600000
//
// SDRAM ROM block is the MAME "user" region order: DOS, F20, DIC, FNT,
// SYS, EX0-3 from 0x400000.

module towns_memmap
(
	input      [23:1] a,
	input             rom_view,       // 8 MB: this is a CPU read with paging off
	input             main_mem_c0,    // 0x404 bit 7: C0000-DFFFF is main RAM
	input             ram_at_f8,
	input             dict_ram,       // 0x480 bit 0: dictionary ROM and CMOS at D0000-D9FFF
	input       [3:0] dict_bank,      // 0x484: 32 KB bank of the dictionary ROM at D0000
	input       [1:0] ram_size,       // main RAM: 0 2 MB, 1 4 MB, 2 6 MB, 3 8 MB (over the OS ROM and EX ROM 0-2)
	input             ss_window,      // savestate restore: FFF000-FFFFFF is the stub RAM, not the ROM
	input             ss_raw,         // savestate copy: the DRAM under every overlay, up to the fitted size

	output reg [23:1] sdram_a,       // word address inside SDRAM
	output reg        sel_sdram,     // RAM or ROM in SDRAM
	output reg        sel_rom,       // ROM: writes are dropped, ROM wait states
	output reg        sel_cmos,      // D8000-D9FFF
	output reg        sel_vram,      // A00000 window: a[18:1] is the VRAM word
	output reg        sel_sprite,    // C00000 window: a[16:1] is the sprite RAM word
	output reg        sel_fmr,       // C0000-CFFFF plane window and its registers
	output reg        sel_pcm,       // F80000-F80FFF: a[11:1] is the wave RAM word
	output reg        sel_card,      // D00000-DFFFFF: a[19:1] is the word in the card window
	output reg        sel_none,      // nothing answers: reads 0xFFFF
	output reg        sel_sswin      // the restore window: a[11:1] is the word
);

localparam [23:1] SD_DOS = 23'h200000;   // 0x400000 >> 1
localparam [23:1] SD_DIC = 23'h280000;   // 0x500000
localparam [23:1] SD_FNT = 23'h2C0000;   // 0x580000
localparam [23:1] SD_SYS = 23'h300000;   // 0x600000
localparam [23:1] SD_EX  = 23'h340000;   // 0x680000

always @* begin
	sdram_a   = a;
	sel_sdram = 0;
	sel_rom   = 0;
	sel_cmos  = 0;
	sel_vram  = 0;
	sel_sprite = 0;
	sel_fmr   = 0;
	sel_pcm   = 0;
	sel_card  = 0;
	sel_none  = 0;
	sel_sswin = 0;

	if (ss_raw) begin
		if (a < 23'h100000) sel_sdram = 1;
		else if (ram_size != 2'd0 && a < 23'h200000) sel_sdram = 1;
		else if (ram_size[1] && a < 23'h300000) begin sdram_a = a + 23'h300000; sel_sdram = 1; end
		else if (ram_size == 2'd3 && a < 23'h400000) begin sdram_a = a + 23'h300000; sel_sdram = 1; end
		else sel_none = 1;
	end
	else if (a < 23'h100000) begin                     // first 2 MB
		if (a[23:1] >= 23'h7C000 && a[23:1] < 23'h80000 && !ram_at_f8) begin  // F8000-FFFFF boot ROM window
			sdram_a   = SD_SYS + 23'h1C000 + (a - 23'h7C000);
			sel_sdram = 1;
			sel_rom   = 1;
		end
		else if (a[23:16] == 8'h0D && !main_mem_c0) begin
			// D0000-DFFFF: dictionary ROM and CMOS only when mapped, else open
			if (dict_ram && a[23:13] == 11'h06C) sel_cmos = 1;             // D8000-D9FFF
			else if (dict_ram && !a[15]) begin                             // D0000-D7FFF
				sdram_a = SD_DIC + {5'd0, dict_bank, a[14:1]};
				sel_sdram = 1; sel_rom = 1;
			end
			else sel_none = 1;
		end
		else if (a[23:16] == 8'h0E && !main_mem_c0) sel_none = 1;   // E0000-EFFFF sits on the VRAM side of the switch
		else if (a[23:16] == 8'h0C && !main_mem_c0) sel_fmr = 1;    // C0000-CFFFF
		else sel_sdram = 1;
	end
	else if (ram_size != 2'd0 && a < 23'h200000) sel_sdram = 1;   // expansion to 4 MB
	else if ((ram_size[1] && a < 23'h300000) ||                  // expansion to 6 MB
	         (ram_size == 2'd3 && a < 23'h400000 && !rom_view)) begin   // and to 8 MB
		sdram_a   = a + 23'h300000;
		sel_sdram = 1;
	end
	else if (a >= 23'h300000 && a < 23'h340000) begin  // OS ROM
		sdram_a = SD_DOS + (a - 23'h300000);
		sel_sdram = 1; sel_rom = 1;
	end
	else if (a >= 23'h340000 && a < 23'h440000) begin  // EX ROM
		sdram_a = SD_EX + (a - 23'h340000);
		sel_sdram = 1; sel_rom = 1;
	end
	else if (a >= 23'h740000 && a < 23'h780000) begin  // dictionary ROM
		sdram_a = SD_DIC + (a - 23'h740000);
		sel_sdram = 1; sel_rom = 1;
	end
	else if (a >= 23'h780000 && a < 23'h7A0000) begin  // font ROM
		sdram_a = SD_FNT + (a - 23'h780000);
		sel_sdram = 1; sel_rom = 1;
	end
	else if (a[23:21] == 3'b101) sel_vram = 1;         // A00000-BFFFFF
	else if (a[23:17] == 7'h60) sel_sprite = 1;        // C00000-C1FFFF
	else if (a[23:13] == 11'h7A0) sel_cmos = 1;        // F40000-F41FFF
	else if (a[23:12] == 12'hF80) sel_pcm = 1;         // F80000-F80FFF
	else if (a[23:20] == 4'hD) sel_card = 1;           // D00000-DFFFFF
	else if (ss_window && a[23:12] == 12'hFFF) sel_sswin = 1;
	else if (a >= 23'h7E0000) begin                    // SYSTEM ROM
		sdram_a = SD_SYS + (a - 23'h7E0000);
		sel_sdram = 1; sel_rom = 1;
	end
	else sel_none = 1;
end

endmodule

// Copyright (c) 2026 Jamie Blanks
//
// Registers next to the CPU (Technical Databook 3rd ed. §3.6 and the
// later-model additions): reset reason, soft reset and power control, NMI
// mask, machine and CPU identification, memory size, speed control, the
// two memory switches. Byte-wide on the low data lines.
//
//   0020 R reset reason {POFF, SHUTDOWN, SOFT}, the low two cleared by read; W RST/POWOFF/WRPROT
//   0022 W power control (POW OFF)
//   0028 W NMI mask (bit 0); a read gives FF, the bit does not come back
//   0030 R {machine id[4:0], cpu id[2:0]}   0031 R machine id[15:8]
//        8 MB option: a word read in protected mode gives cpu id 2 (486), any other the Marty's 3
//   0032 W {ID RESET, ID CLK, CS_n, 0}  R {last[7:6], 0, ID DATA}: 256-bit serial ID ROM
//        CS_n low: a RESET falling edge returns to bit 0, a CLK rising edge advances
//   0404 R/W bit 7 MAIN MEM: C0000-CFFFF is main RAM (1) or the VRAM window (0)
//   0480 R/W bit 1 RAM at F8000-FFFFF instead of the boot ROM, bit 0 dictionary RAM
//   0484 R/W bits 3:0 dictionary ROM bank shown at D0000-D7FFF (32 KB each)
//   05C0 R/W bit 3 expansion bus NMI enable   05C2 R bit 3 expansion NMI, none here
//   0400 R system status (bit 0 resolution = 0)
//   05E8 R memory size in MB           05EC R/W bit 0 fast mode, bit 7 low = present

module towns_sysregs #(
	parameter [15:0] MACHINE_ID = 16'h4A03,   // Marty: 0x4A, 386SX
	// Serial ID image, 32 bytes, bit address A at byte 31 - A/8 bit A%8:
	// "0", "FUJITSU", reserved, machine number 4A03, serial 000007344,
	// zeros. Dumped from a Marty; only the serial digits vary per unit.
	parameter [255:0] SERIAL_ID = 256'h0465_54A4_9545_355F_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FF00_004A_0300_0007_3440_0000,
	// Same layout from a desktop Towns: the reserved bytes stay FF where the
	// Marty has 00 00 before its machine number. Some late discs' IPL reads
	// those bits and refuses to boot a Marty.
	parameter [255:0] SERIAL_ID_TOWNS = 256'h0465_54A4_9545_355F_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FEFF_FF0C_0200_0000_15E0_0000
)
(
	input             clk,
	input             ce,
	input             reset,          // system reset: clears the reason flags too

	input      [15:0] io_addr,
	input             io_rd,          // one-pulse strobes at ce
	input             io_wr,
	input       [7:0] io_din,
	input             io_word,        // the current cycle is a 16-bit access
	input             cpu_pe,         // the CPU is in protected mode
	output reg  [7:0] io_dout,
	output reg        io_sel,         // this block answers the current address

	input             shutdown,       // CPU shutdown cycle seen
	input       [1:0] ram_size,       // 05E8 reports 2, 4, 6 or 8 MB
	input             towns_id,       // serial ID ROM reads as a desktop Towns
	output reg        soft_reset,     // pulse: RST written
	output reg        power_off,
	output reg        nmi_mask,
	output reg        nmi_vector_wp,
	output reg        main_mem_c0,    // 0x404 bit 7
	output reg        ram_at_f8,      // 0x480 bit 1
	output reg        dict_ram,       // 0x480 bit 0
	output reg  [3:0] dict_bank,      // 0x484
	output reg        fast_mode,      // 0x5EC bit 0

	// savestate port: the state as bytes, no side effects
	input             ss_cs,
	input             ss_wr,
	input       [1:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

reg reason_soft, reason_shutdown, reason_poff;
reg bnmi_en;
reg [7:0] id_addr;
reg       id_rst_q, id_clk_q;
wire      id_bit = towns_id ? SERIAL_ID_TOWNS[id_addr] : SERIAL_ID[id_addr];

always @(posedge clk) begin
	soft_reset <= 0;
	if (reset) begin
		reason_soft <= 0;
		reason_shutdown <= 0;
		reason_poff <= 0;
		power_off <= 0;
		nmi_mask <= 0;
		nmi_vector_wp <= 0;
		main_mem_c0 <= 0;
		ram_at_f8 <= 0;
		dict_ram <= 0;
		dict_bank <= 4'd0;
		bnmi_en <= 0;
		fast_mode <= 1;
		id_addr <= 8'd0; id_rst_q <= 0; id_clk_q <= 0;
	end
	else if (ce) begin
		if (shutdown) reason_shutdown <= 1;
		if (io_wr) begin
			case (io_addr)
			16'h0020: begin
				nmi_vector_wp <= io_din[7];
				if (io_din[6]) power_off <= 1;
				if (io_din[0]) begin
					soft_reset <= 1;
					reason_soft <= 1;
				end
			end
			16'h0022: if (io_din[6]) power_off <= 1;
			16'h0028: nmi_mask <= io_din[0];
			16'h0032: begin
				id_rst_q <= io_din[7];
				id_clk_q <= io_din[6];
				if (!io_din[5]) begin
					if (id_rst_q && !io_din[7]) id_addr <= 8'd0;
					else if (!io_din[7] && io_din[6] && !id_clk_q) id_addr <= id_addr + 1'd1;
				end
			end
			16'h0404: main_mem_c0 <= io_din[7];
			16'h0480: {ram_at_f8, dict_ram} <= io_din[1:0];
			16'h0484: dict_bank <= io_din[3:0];
			16'h05C0: bnmi_en <= io_din[3];
			16'h05EC: fast_mode <= io_din[0];
			default: ;
			endcase
		end
		if (io_rd && io_addr == 16'h0020) begin
			reason_soft <= 0;
			reason_shutdown <= 0;
		end
	end
	if (ss_cs && ss_wr) begin
		case (ss_a)
		2'd0: {reason_soft, reason_shutdown, reason_poff, nmi_mask, nmi_vector_wp, bnmi_en, fast_mode, main_mem_c0} <= ss_din;
		2'd1: {ram_at_f8, dict_ram, dict_bank, id_rst_q, id_clk_q} <= ss_din;
		2'd2: id_addr <= ss_din;
		default: ;
		endcase
	end
end

always @* begin
	case (ss_a)
	2'd0: ss_dout = {reason_soft, reason_shutdown, reason_poff, nmi_mask, nmi_vector_wp, bnmi_en, fast_mode, main_mem_c0};
	2'd1: ss_dout = {ram_at_f8, dict_ram, dict_bank, id_rst_q, id_clk_q};
	2'd2: ss_dout = id_addr;
	default: ss_dout = 8'h00;
	endcase
end

always @* begin
	io_sel = 1;
	case (io_addr)
	16'h0020: io_dout = {5'd0, reason_poff, reason_shutdown, reason_soft};
	16'h0028: io_dout = 8'hFF;              // the mask bit does not read back
	// IO.SYS takes the CPU type for its memory map from real-mode reads, byte
	// or word; Teo's 486 check is a word read from protected mode
	16'h0030: io_dout = (ram_size == 2'd3 && io_word && cpu_pe) ? {MACHINE_ID[7:3], 3'd2} : MACHINE_ID[7:0];
	16'h0031: io_dout = MACHINE_ID[15:8];
	16'h0032: io_dout = {id_rst_q, id_clk_q, 5'd0, id_bit};
	16'h0024: io_dout = 8'h07;
	16'h0400: io_dout = 8'hFE;   // system status: bit 0 resolution, fixed 0 (medium)
	16'h0404: io_dout = {main_mem_c0, 7'd0};
	16'h0480: io_dout = {6'd0, ram_at_f8, dict_ram};
	16'h0484: io_dout = {4'd0, dict_bank};
	16'h05C0: io_dout = {4'd0, bnmi_en, 3'd0};
	16'h05C2: io_dout = 8'h00;
	16'h05E8: io_dout = {5'd0, ram_size, 1'b0} + 8'd2;   // size in MB
	16'h05EC: io_dout = {7'd0, fast_mode};
	default: begin
		io_dout = 8'hFF;
		io_sel = 0;
	end
	endcase
end

endmodule

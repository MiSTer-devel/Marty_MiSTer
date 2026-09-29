// Copyright (c) 2026 Jamie Blanks
//
// SDRAM holding main DRAM, the mask ROMs and VRAM. Port 0 serves the
// mainboard's word requests and the ROM loader, port 2 the VRAM RAM port
// (CPU and sprite engine). The CRTC line fetch reads a DDR3 mirror of
// VRAM (vram_ddr) and leaves the controller's first port idle.
//
// SDRAM layout (byte addresses within the 16 MB of a bank):
//   000000 main DRAM (2 MB)
//   400000 OS ROM   480000 20-dot font   500000 dictionary   580000 font
//   600000 SYSTEM   680000 EX ROM 0-3 (2 MB)
//   900000 VRAM (512 KB)
//   A00000 DRAM for 400000-7FFFFF (6 and 8 MB options)
//
// The controller keeps one row open per bank (address bits 25:24), so
// each consumer gets its own bank and keeps its row across the others'
// traffic: DRAM in banks 0 and 3 (odd 2 KB rows in 3), VRAM in bank 1,
// the ROMs in bank 2.
//
// The ROM loader rearranges the two Marty mask ROM images into that order:
//   mrom.m36  000000-07FFFF -> 400000   080000-1FFFFF -> 680000
//   mrom.m37  000000-07FFFF -> 800000   080000-1BFFFF -> 480000   1C0000-1FFFFF -> 600000
//
// The mainboard runs on clk_sys, the controller on clk_sdram (2x, same PLL).
// A request starts on the rising edge of `req` (a pulse or a level held
// until `ready` returns); `ready` drops the next cycle and comes back high
// with the data. The controller forwards its own inverted SDRAM_CLK and
// samples DQ in the pin cell on the edge and pipeline measured on the board.

module towns_memory #(
	parameter CLK_SDRAM_HZ = 114545455,
	parameter DQ_CAPTURE_PIPELINE = 2,  // rising-edge capture at 114.5 MHz, measured on the board
	parameter DQ_CAPTURE_RISING = 1,
	parameter ROM_M36_INDEX = 1,     // ioctl_index of mrom.m36
	parameter ROM_M37_INDEX = 2      // ioctl_index of mrom.m37
)
(
	input             clk_sys,
	input             clk_sdram,
	input             reset,          // asynchronous to the controller

	// mainboard word port: a rise of mem_req queues a request (two may be
	// in flight); mem_ready is one clock per answer, in order, with mem_dout
	input      [23:1] mem_a,
	input       [1:0] mem_be,
	input             mem_we,
	input      [15:0] mem_din,
	output     [15:0] mem_dout,
	input             mem_req,
	output            mem_ready,

	// VRAM RAM port: word access with byte enables
	input      [18:1] vr_a,
	input       [1:0] vr_be,
	input             vr_we,
	input      [15:0] vr_din,
	output     [15:0] vr_dout,
	input             vr_req,
	output            vr_ready,

	// ROM loader
	input             ioctl_download,
	input      [15:0] ioctl_index,
	input             ioctl_wr,
	input      [26:0] ioctl_addr,
	input       [7:0] ioctl_dout,
	input             dos_two_drives, // patch the ROM DOS out of single-floppy mode

	output            ioctl_wait,     // hold the download while a byte is in flight
	output            initialised,    // controller finished its init sequence
	output      [7:0] dbg_port0,      // {st, ready, sd_req, busy, ready, init} for the beacon

	output            roms_loaded,    // both mask ROM images fully received

	// ROM read-back check: sums every ROM byte through port 0 the way the
	// loader summed them on the way in, so the two can be compared
	input             verify_start,
	output reg        verify_busy,
	output reg        verify_done,
	output reg [31:0] verify_sum,
	output reg [29:0] verify_words,

	output            SDRAM_CLK,
	output            SDRAM_CKE,
	output     [12:0] SDRAM_A,
	output      [1:0] SDRAM_BA,
	inout      [15:0] SDRAM_DQ,
	output            SDRAM_DQML,
	output            SDRAM_DQMH,
	output            SDRAM_nCS,
	output            SDRAM_nCAS,
	output            SDRAM_nRAS,
	output            SDRAM_nWE
);

localparam [25:0] SD_VRAM = 26'h1900000;   // bank 1
localparam [1:0]  BANK_ROM = 2'd2;

// bank of a mainboard byte address: the ROMs sit at 400000-8FFFFF; DRAM
// alternates between banks 0 and 3 every 2 KB row, so a program's code
// row and its data or stack row can both stay open
function automatic [1:0] sd_bank(input [23:1] a);
	sd_bank = (a[23:22] == 2'b01 || a[23:20] == 4'h8) ? BANK_ROM : {a[11], a[11]};
endfunction

// ---- ROM loader: byte writes placed by image and offset ----
// Images come from the OSD F slots (index 1, 2) or from boot0.rom and
// boot1.rom, which the HPS sends as index 0 and 1<<6.
wire        is_m36 = ioctl_download && (ioctl_index == {10'd0, ROM_M36_INDEX[5:0]} || ioctl_index == 16'h0000);
wire        is_m37 = ioctl_download && (ioctl_index == {10'd0, ROM_M37_INDEX[5:0]} || ioctl_index == 16'h0040);
wire [23:0] off    = ioctl_addr[23:0];
wire [23:0] rom_a  = is_m36 ? (off < 24'h080000 ? 24'h400000 + off : 24'h600000 + off)
                            : (off < 24'h080000 ? 24'h800000 + off :
                               off < 24'h1C0000 ? 24'h400000 + off : 24'h440000 + off);

wire        rom_wr = ioctl_wr && (is_m36 || is_m37) && !mem_req;

// The Marty ROM DOS hard-codes MS-DOS single-drive mode (mov byte [40:46],1
// at EX ROM 0 offset C817), so any A:/B: alternation blocks in an invisible
// "insert disk" prompt. Desktop IO.SYS reads that flag from CMOS instead.
// Storing 0 there gives the desktop behaviour: B: fails cleanly. Only that
// exact instruction (26 C6 06 46 00 01) is changed, so other ROMs are left alone.
reg  [39:0] m36_last5;                   // last five m36 bytes, newest in the low byte
always @(posedge clk_sys)
	if (ioctl_wr && is_m36) m36_last5 <= {m36_last5[31:0], ioctl_dout};

wire        rom_patch = dos_two_drives && is_m36 && (off == 24'h08C81C)
                        && m36_last5 == 40'h26C6064600 && ioctl_dout == 8'h01;
wire  [7:0] rom_byte  = rom_patch ? 8'h00 : ioctl_dout;

// A download that ends is not a load: Linux also opens and closes a
// transfer for boot ROM files it fails to find. Count the bytes instead.
reg [20:0] m36_bytes, m37_bytes;         // saturate one short of the 2 MB image
always @(posedge clk_sys) begin
	if (ioctl_wr && is_m36 && !(&m36_bytes)) m36_bytes <= m36_bytes + 1'd1;
	if (ioctl_wr && is_m37 && !(&m37_bytes)) m37_bytes <= m37_bytes + 1'd1;
end
assign roms_loaded = (&m36_bytes) & (&m37_bytes);   // 2 MB each

// Read-back walk over the three loaded ROM ranges; runs while the CPU is
// held in reset, so port 0 is free.
reg  [23:1] vfy_a;
reg         vfy_req, vfy_wait;
always @(posedge clk_sys) begin
	vfy_req <= 0;
	if (reset) begin
		verify_busy <= 0;
		verify_done <= 0;
		vfy_wait <= 0;
	end
	else if (verify_start && !verify_busy) begin
		verify_busy  <= 1;
		verify_done  <= 0;
		verify_sum   <= 0;
		verify_words <= 0;
		vfy_a       <= 23'h200000;                 // byte 400000
		vfy_req     <= 1;
		vfy_wait    <= 1;
	end
	else if (verify_busy) begin
		if (vfy_wait) begin
			if (mem_ready && !vfy_req) begin
				verify_sum   <= verify_sum + {24'd0, mem_dout[7:0]} + {24'd0, mem_dout[15:8]};
				verify_words <= verify_words + 1'd1;
				vfy_wait     <= 0;
			end
		end
		else begin
			case (vfy_a)
			23'h2DFFFF: vfy_a <= 23'h300000;       // 5BFFFE -> 600000
			23'h31FFFF: vfy_a <= 23'h340000;       // 63FFFE -> 680000
			23'h43FFFF: begin verify_busy <= 0; verify_done <= 1; end   // 87FFFE: done
			default:    vfy_a <= vfy_a + 1'd1;
			endcase
			if (vfy_a != 23'h43FFFF) begin
				vfy_req  <= 1;
				vfy_wait <= 1;
			end
		end
	end
end

// The loader and the verify walk run with the CPU in reset, so their
// registered flags pick the port's inputs and the CPU request is the
// default: nothing in the address path waits for the request decode.
wire        rom_ld     = ioctl_wr && (is_m36 || is_m37);
wire        p0_req_sys = mem_req | rom_wr | vfy_req;
wire [25:0] p0_addr    = vfy_req ? {BANK_ROM, vfy_a, 1'b0} : rom_ld ? {BANK_ROM, rom_a[23:1], 1'b0} : {sd_bank(mem_a), mem_a, 1'b0};
wire  [1:0] p0_be      = vfy_req ? 2'b11 : rom_ld ? (rom_a[0] ? 2'b10 : 2'b01) : mem_be;
wire        p0_we      = vfy_req ? 1'b0 : rom_ld ? 1'b1 : mem_we;
wire [15:0] p0_din     = rom_ld ? {rom_byte, rom_byte} : mem_din;

// Reset brought into the controller's clock: `reset_sd` is its asynchronous
// reset, `reset_port` the synchronous copy the ports and init flag use.
reg reset_sd, reset_sd1, reset_port;
always @(posedge clk_sdram) {reset_port, reset_sd, reset_sd1} <= {reset_sd1, reset_sd1, reset};

wire        p0_busy, p0_ready, p1_busy, p2_busy, p2_ready;
wire        p0_req, p2_req;
wire        p0_we_sd, p2_we_sd;
wire [25:0] p0_a_sd, p2_a_sd;
wire [15:0] p0_d_sd, p2_d_sd;
wire  [1:0] p0_be_sd, p2_be_sd;
wire [63:0] p0_dout, p2_dout;
wire        p0_idle;
wire  [1:0] p0_st;

towns_sdram_qport port0
(
	.clk_sys(clk_sys), .clk_sdram(clk_sdram), .reset(reset), .reset_sd(reset_port),
	.req(p0_req_sys), .addr(p0_addr), .be(p0_be), .we(p0_we), .din(p0_din),
	.dout(mem_dout), .ready(mem_ready), .idle(p0_idle), .dbg_st(p0_st),
	.sd_req(p0_req), .sd_addr(p0_a_sd), .sd_be(p0_be_sd), .sd_we(p0_we_sd), .sd_din(p0_d_sd),
	.sd_dout(p0_dout[15:0]), .sd_busy(p0_busy), .sd_ready(p0_ready)
);

towns_sdram_port #(.DW(16)) port2
(
	.clk_sys(clk_sys), .clk_sdram(clk_sdram), .reset(reset), .reset_sd(reset_port),
	.req(vr_req), .addr(SD_VRAM | {7'd0, vr_a, 1'b0}), .be(vr_be), .we(vr_we), .din(vr_din),
	.dout(vr_dout), .ready(vr_ready), .idle(), .dbg_st(),
	.sd_req(p2_req), .sd_addr(p2_a_sd), .sd_be(p2_be_sd), .sd_we(p2_we_sd), .sd_din(p2_d_sd),
	.sd_dout(p2_dout[15:0]), .sd_busy(p2_busy), .sd_ready(p2_ready)
);

// The controller's busy is high until its init sequence completes; its
// first port never waits on another.
reg init_done, init_done_sys;
always @(posedge clk_sdram) begin
	if (reset_port) init_done <= 0;
	else if (!p1_busy) init_done <= 1;
end
always @(posedge clk_sys) init_done_sys <= init_done;

// Only a download may hold the HPS: the framework stalls its whole SPI
// handshake while this is high, and the CPU shares port 0.
assign ioctl_wait  = ioctl_download & ~p0_idle;
assign initialised = init_done_sys;
assign dbg_port0   = {p0_st, p0_idle, p0_req, p0_busy, p0_ready, init_done, 1'b0};

sdram #(.CLK_FREQ_HZ(CLK_SDRAM_HZ), .DQ_CAPTURE_PIPELINE(DQ_CAPTURE_PIPELINE), .DQ_CAPTURE_RISING(DQ_CAPTURE_RISING[0]),
        .PORT0_SIZE(3), .PORT1_SIZE(1), .PORT2_SIZE(1)) ctrl
(
	.clk(clk_sdram),
	.reset(reset_sd),
	.refresh(1'b0),

	// controller port 0: unused, the line fetch reads its DDR3 mirror
	.p0_req(1'b0),
	.p0_we(1'b0),
	.p0_addr(26'd0),
	.p0_din(64'd0),
	.p0_byte_en(8'hFF),
	.p0_dout(),
	.p0_busy(p1_busy),   // init done when the controller's first port stops being busy
	.p0_ready(),

	// controller port 1: the mainboard word port
	.p1_req(p0_req),
	.p1_we(p0_we_sd),
	.p1_addr(p0_a_sd),
	.p1_din({48'd0, p0_d_sd}),
	.p1_byte_en({6'd0, p0_be_sd}),
	.p1_dout(p0_dout),
	.p1_busy(p0_busy),
	.p1_ready(p0_ready),

	.p2_req(p2_req),
	.p2_we(p2_we_sd),
	.p2_addr(p2_a_sd),
	.p2_din({48'd0, p2_d_sd}),
	.p2_byte_en({6'd0, p2_be_sd}),
	.p2_dout(p2_dout),
	.p2_busy(p2_busy),
	.p2_ready(p2_ready),

	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),
	.SDRAM_A(SDRAM_A),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nWE(SDRAM_nWE)
);

endmodule

// One client port: the mainboard's request, held for a T-state on clk_sys,
// becomes a level the controller sees on clk_sdram (2x, same PLL, so the
// paths are timed rather than cut) and the answer comes back the same way.
// The controller takes a request on the edge where req is high and busy is
// low; in_flight keeps it from being offered twice.
//
//   req      _/‾‾‾‾\______________________   mainboard, level for a T-state
//   pend     __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____   one request, cleared when acked
//   sd_req   ___/‾‾‾\____________________   until the controller takes it
//   acked    ___________________/‾‾‾‾\____   dout valid, cleared with pend
//   ready    ‾‾‾\_____________________/‾‾‾   low from the cycle after req
module towns_sdram_port #(parameter DW = 16)
(
	input             clk_sys,
	input             clk_sdram,
	input             reset,          // clk_sys domain
	input             reset_sd,       // the same reset in the controller's clock

	input             req,
	input      [25:0] addr,
	input       [1:0] be,
	input             we,
	input    [DW-1:0] din,
	output reg [DW-1:0] dout,
	output            ready,
	output            idle,
	output      [1:0] dbg_st,

	output reg        sd_req,
	output reg [25:0] sd_addr,
	output reg  [1:0] sd_be,
	output reg        sd_we,
	output reg [DW-1:0] sd_din,
	input    [DW-1:0] sd_dout,
	input             sd_busy,
	input             sd_ready
);

reg pend, req_d, acked, acked_s, in_flight;
always @(posedge clk_sys) begin
	req_d <= req;
	if (reset) pend <= 0;
	else if (!pend) begin
		if (req && !req_d) begin
			sd_addr <= addr;
			sd_be   <= be;
			sd_we   <= we;
			sd_din  <= din;
			pend    <= 1;
		end
	end
	else if (acked) pend <= 0;
end

always @(posedge clk_sdram) begin
	if (reset_sd) begin
		sd_req    <= 0;
		in_flight <= 0;
		acked     <= 0;
		acked_s   <= 0;
	end
	else begin
		acked_s <= acked;
		sd_req <= pend & ~acked & ~in_flight;
		if (sd_req && !sd_busy) in_flight <= 1;
		else if (in_flight && sd_ready) begin
			in_flight <= 0;
			dout      <= sd_dout;
			acked     <= 1;
		end
		if (!pend) acked <= 0;
	end
end

// the answer shows the clock its trailing flag is seen, a clk_sys edge
// before pend clears; the data has then been settled for a controller cycle
assign ready  = ~pend | acked_s;
assign idle   = ~pend;
assign dbg_st = {in_flight, acked};

endmodule


// The CPU port: two requests queued in order, reads and writes. The
// mainboard raises req for a T-state; the port takes it on the rise while
// a slot is free and offers the oldest to the controller as soon as the
// one before it is answered, so a pipelined request's handshake hides
// behind the access in flight. ready is one clock per answer, in order,
// with dout (a write's answer is the chip taking it); idle when nothing
// is queued or in flight.
//
//   req      _/‾\______/‾\_________________________   two rises
//   count    __1________2___________1_________0____
//   sd_req   __/‾‾‾\______/‾‾‾‾‾‾‾‾‾‾‾\_____________  head, until taken
//   ready    ________________/‾\_________/‾\_______  in order, dout valid
module towns_sdram_qport #(parameter DW = 16)
(
	input             clk_sys,
	input             clk_sdram,
	input             reset,
	input             reset_sd,

	input             req,
	input      [25:0] addr,
	input       [1:0] be,
	input             we,
	input    [DW-1:0] din,
	output   [DW-1:0] dout,
	output            ready,
	output            idle,
	output      [1:0] dbg_st,

	output            sd_req,
	output     [25:0] sd_addr,
	output      [1:0] sd_be,
	output            sd_we,
	output   [DW-1:0] sd_din,
	input    [DW-1:0] sd_dout,
	input             sd_busy,
	input             sd_ready
);

reg [25:0] q_addr [0:1];
reg  [1:0] q_be   [0:1];
reg        q_we   [0:1];
(* ramstyle = "logic" *) reg [DW-1:0] q_din  [0:1];
(* ramstyle = "logic" *) reg [DW-1:0] q_data [0:1];   // read across the clock crossing: registers, not a RAM
reg  [1:0] q_valid;           // slot holds a request
reg  [1:0] q_done;            // slot holds an answer
// Each side reads the other's slot on the strength of a flag that trails
// the data by one controller cycle: the controller acts on q_valid only
// after seeing it on two edges (q_valid_d), and clk_sys takes an answer on
// q_done_s, published a cycle behind q_done. A clk_sys edge coincident
// with a controller edge can take a flag early; the data it points at has
// then been settled for a full controller period, longer than the skew
// between the two clock networks.
reg  [1:0] q_valid_d, q_done_s;
reg        wp, rp;            // clk_sys: next slot to fill, next slot to hand back
reg        ip;                // clk_sdram: next slot to offer the controller
reg  [1:0] q_issued;
reg        in_flight, req_d;

wire accept = !(q_valid[0] && q_valid[1]);
assign idle   = !(q_valid[0] || q_valid[1]);
assign dbg_st = {in_flight, q_done[rp]};

// the answer is handed over the clock its flag is seen
assign ready = q_valid[rp] && q_done_s[rp];
assign dout  = q_data[rp];

// The slot about to be filled follows the inputs every clock and freezes
// when its flag is set, so the request decode only reaches the flag.
integer k;
always @(posedge clk_sys) begin
	req_d <= req;
	for (k = 0; k < 2; k = k + 1)
		if (!q_valid[k] && wp == k[0]) begin
			q_addr[k] <= addr;
			q_be[k]   <= be;
			q_we[k]   <= we;
			q_din[k]  <= din;
		end
	if (reset) begin
		q_valid <= 0; wp <= 0; rp <= 0;
	end
	else begin
		if (req && !req_d && accept) begin
			q_valid[wp] <= 1;
			wp <= ~wp;
		end
		// answers come back in order; the slot is freed as it is handed back
		if (ready) begin
			q_valid[rp] <= 0;
			rp <= ~rp;
		end
	end
end

// The oldest unissued slot is offered straight from the queue: the
// controller holds busy until it has taken the request, and the slot
// keeps its contents until the answer is handed back.
assign sd_req  = q_valid_d[ip] && !q_issued[ip] && !in_flight;
assign sd_addr = q_addr[ip];
assign sd_be   = q_be[ip];
assign sd_we   = q_we[ip];
assign sd_din  = q_din[ip];

always @(posedge clk_sdram) begin
	if (reset_sd) begin
		in_flight <= 0; ip <= 0; q_issued <= 0; q_done <= 0; q_done_s <= 0; q_valid_d <= 0;
	end
	else begin
		q_valid_d <= q_valid;
		q_done_s  <= q_done;
		if (!q_valid[0]) begin q_issued[0] <= 0; q_done[0] <= 0; end
		if (!q_valid[1]) begin q_issued[1] <= 0; q_done[1] <= 0; end
		if (sd_req && !sd_busy) begin
			in_flight <= 1;
			q_issued[ip] <= 1;
		end
		if (in_flight && sd_ready) begin
			in_flight <= 0;
			q_data[ip] <= sd_dout;
			q_done[ip] <= 1;
			ip <= ~ip;
		end
	end
end

endmodule

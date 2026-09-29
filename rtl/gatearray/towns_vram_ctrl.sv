// Copyright (c) 2026 Jamie Blanks
//
// VRAM controller: the RAM port of the 512 KB dual-port VRAM for the CPU
// and the sprite engine, the serial-port line fetch for the CRTC, the
// packed-pixel write mask at 0458, the FMR-50 plane window at C0000 with
// its CFF8x registers, and the sprite screen clear.
//
//   CPU A00000 window  --> physical word  --+
//   CPU B00000 window  --> interleave     --+-> RAM port (SDRAM p2)
//   FMR C0000 window   --> plane RMW      --+
//   sprite writes      --> layer 1        --+
//   CRTC line fetch    --> 64-bit reads   ----> serial port (SDRAM p1)
//
// Sprite clear: the real chip fills a 128 KB half of layer 1 with its
// first two lines in 32 us, far beyond what SDRAM word writes can do.
// The half is instead marked pixel by pixel in a valid bitmap: the clear
// snapshots the two lines into a stripe RAM and zeroes the bitmap, every
// write sets its pixel's bit, and reads of an unset pixel return the
// stripe. Lines 0 and 1 are never substituted. Bits only matter once a
// half has been cleared for the first time.

module towns_vram_ctrl
(
	input             clk,
	input             reset,

	// CPU packed windows: a[20] = B00000 linear window, a[18:1] the word
	input             raw,         // savestate copy: writes ignore the 0458 mask
	input             ss_cs,       // savestate port: the registers as bytes
	input             ss_wr,
	input       [3:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	input             cpu_req,
	input      [20:1] cpu_a,
	input       [1:0] cpu_be,
	input             cpu_we,
	input      [15:0] cpu_din,
	output reg [15:0] cpu_dout,
	output reg        cpu_ack,

	// FMR plane window C0000-C7FFF, one byte per request
	input             fmr_req,
	input      [14:0] fmr_a,
	input             fmr_we,
	input       [7:0] fmr_din,
	output reg  [7:0] fmr_dout,
	output reg        fmr_ack,

	// registers: I/O 0458/045A/045B, 05C8, FF81-FF86 and CFF80-CFF99 as I/O
	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output            io_sel,
	input             tvram_wr,       // write into the text VRAM area

	input             in_hsync,
	input             in_vsync,
	output      [3:0] fmr_plane_mask,
	output            fmr_ps2,
	output            fmr_ank_font,   // CFF99 bit 0: CA000-CBFFF shows the ANK font ROM

	// sprite engine word writes into layer 1: acknowledged once posted,
	// idle once the last one has reached the SDRAM
	input             sp_req,
	input      [18:1] sp_a,
	input      [15:0] sp_din,
	output reg        sp_ack,
	output            sp_idle,

	input             clr_req,
	input             clr_page,
	output reg        clr_done,

	// CRTC line fetch
	// bench peek at the clear bitmap: with dbg_bm_en high port B follows dbg_bm_addr
	input             dbg_bm_en,
	input      [10:0] dbg_bm_addr,
	output     [63:0] dbg_bm_q,

	// a request is held with its address until fetch_ack; two may be
	// outstanding and the answers come back in order
	input             fetch_req,
	input      [18:3] fetch_a,
	output reg        fetch_ack,
	output reg [63:0] fetch_data,
	output reg        fetch_valid,

	// SDRAM ports
	output reg [18:3] vf_a,
	output reg        vf_req,
	input             vf_accept,
	input      [63:0] vf_dout,
	input             vf_ready,
	output reg [18:1] vr_a,
	output reg  [1:0] vr_be,
	output reg        vr_we,
	output reg [15:0] vr_din,
	input      [15:0] vr_dout,
	output reg        vr_req,
	input             vr_ready
);

// ---- registers ----
reg  [1:0] mask_ra;
reg [15:0] mask [0:1];      // 0458 register 0: lanes 0,1; register 1: lanes 2,3
reg  [7:0] cff81, cff82, cff83, cff80, cff99;
reg        md_flag;

assign fmr_plane_mask = {cff82[5], cff82[2:0]};
assign fmr_ps2 = cff82[4];
assign fmr_ank_font = cff99[0];

wire sel_mask = (io_addr == 16'h0458) || (io_addr == 16'h045A) || (io_addr == 16'h045B);
wire sel_5c8  = (io_addr == 16'h05C8);
wire sel_ff8  = (io_addr[15:4] == 12'h0FF8);   // FF8x; the mainboard presents CFF8x here too
wire sel_ff99 = (io_addr == 16'h0FF99);
assign io_sel = sel_mask | sel_5c8 | sel_ff8 | sel_ff99;

always @* begin
	io_dout = 8'hFF;
	if (io_addr == 16'h0458) io_dout = {6'd0, mask_ra};
	else if (io_addr == 16'h045A) io_dout = mask[mask_ra[0]][7:0];
	else if (io_addr == 16'h045B) io_dout = mask[mask_ra[0]][15:8];
	else if (sel_5c8) io_dout = {md_flag, 7'd0};
	else if (sel_ff8) case (io_addr[3:0])
		4'h0: io_dout = cff80;
		4'h1: io_dout = cff81;
		4'h2: io_dout = cff82;
		4'h3: io_dout = cff83;
		4'h4: io_dout = 8'h00;
		4'h6: io_dout = {in_hsync, 2'd0, 1'b1, 1'b0, in_vsync, 2'd0};   // bit 4 always reads 1
		default: io_dout = 8'hFF;
	endcase
	else if (sel_ff99) io_dout = cff99;
end

always @* begin
	case (ss_a)
	4'd0: ss_dout = {5'd0, md_flag, mask_ra};
	4'd1: ss_dout = mask[0][7:0];
	4'd2: ss_dout = mask[0][15:8];
	4'd3: ss_dout = mask[1][7:0];
	4'd4: ss_dout = mask[1][15:8];
	4'd5: ss_dout = cff80;
	4'd6: ss_dout = cff81;
	4'd7: ss_dout = cff82;
	4'd8: ss_dout = cff83;
	4'd9: ss_dout = cff99;
	default: ss_dout = 8'h00;
	endcase
end

always @(posedge clk) begin
	if (reset) begin
		mask_ra <= 0;
		mask[0] <= 16'hFFFF;
		mask[1] <= 16'hFFFF;
		cff80 <= 8'h00;
		cff81 <= 8'h0F;
		cff82 <= 8'h27;
		cff83 <= 8'h00;
		cff99 <= 8'h00;
		md_flag <= 0;
	end
	else begin
		if (tvram_wr) md_flag <= 1;
		if (io_rd && sel_5c8) md_flag <= 0;
		if (io_wr) begin
			if (io_addr == 16'h0458) mask_ra <= io_din[1:0];
			if (io_addr == 16'h045A) mask[mask_ra[0]][7:0] <= io_din;
			if (io_addr == 16'h045B) mask[mask_ra[0]][15:8] <= io_din;
			if (sel_ff8) case (io_addr[3:0])
				4'h0: cff80 <= io_din;
				4'h1: cff81 <= io_din;
				4'h2: cff82 <= io_din;
				4'h3: cff83 <= io_din;
				default: ;
			endcase
			if (sel_ff99) cff99 <= io_din;
		end
		if (ss_cs && ss_wr) begin
			case (ss_a)
			4'd0: {md_flag, mask_ra} <= ss_din[2:0];
			4'd1: mask[0][7:0] <= ss_din;
			4'd2: mask[0][15:8] <= ss_din;
			4'd3: mask[1][7:0] <= ss_din;
			4'd4: mask[1][15:8] <= ss_din;
			4'd5: cff80 <= ss_din;
			4'd6: cff81 <= ss_din;
			4'd7: cff82 <= ss_din;
			4'd8: cff83 <= ss_din;
			4'd9: cff99 <= ss_din;
			default: ;
			endcase
		end
	end
end

// ---- valid bitmap and stripe RAMs ----
// bitmap: one bit per layer 1 pixel word, 64 per entry, {half, word[15:6]}
reg  [10:0] bm_a_addr, bm_b_addr;
reg         bm_a_we;
reg  [63:0] bm_a_wd;
wire [63:0] bm_a_q, bm_b_q;

cache_ram_dp #(.ADDR_WIDTH(11), .DATA_WIDTH(64)) bitmap
(
	.clk_i(clk),
	.addr_a_i(bm_a_addr), .wren_a_i(bm_a_we), .wdata_a_i(bm_a_wd), .q_a_o(bm_a_q),
	.addr_b_i(bm_b_addr), .wren_b_i(1'b0), .wdata_b_i(64'd0), .q_b_o(bm_b_q)
);
assign dbg_bm_q = bm_b_q;

// stripe: lines 0 and 1 of each half, {half, line, word[7:2]}; two copies
// so the RAM port and the line fetch can look up at the same time
reg   [7:0] st_wa, st_ra_p2, st_ra_f;
reg         st_we;
reg  [63:0] st_wd;
wire [63:0] st_q_p2, st_q_f;

cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(64)) stripe_p2
(
	.clk_i(clk),
	.addr_a_i(st_wa), .wren_a_i(st_we), .wdata_a_i(st_wd), .q_a_o(),
	.addr_b_i(st_ra_p2), .wren_b_i(1'b0), .wdata_b_i(64'd0), .q_b_o(st_q_p2)
);

cache_ram_dp #(.ADDR_WIDTH(8), .DATA_WIDTH(64)) stripe_f
(
	.clk_i(clk),
	.addr_a_i(st_wa), .wren_a_i(st_we), .wdata_a_i(st_wd), .q_a_o(),
	.addr_b_i(st_ra_f), .wren_b_i(1'b0), .wdata_b_i(64'd0), .q_b_o(st_q_f)
);

reg  [1:0] armed;

// ---- clear: snapshot two lines through the fetch port, zero the bitmap ----
localparam [1:0] C_IDLE = 2'd0, C_RUN = 2'd1, C_END = 2'd2;
reg  [1:0] c_state;
reg  [6:0] c_rd;          // 128 reads of 8 bytes
reg  [9:0] c_zero;        // 1024 bitmap entries
reg        c_rd_done, c_zero_done, c_rd_wait;
reg        c_page;
wire       clr_active = (c_state == C_RUN);

// ---- fetch port arbitration: CRTC first, then the snapshot ----
// Two reads may be outstanding, kept in order in a two-entry queue; a
// snapshot read only goes out on an empty queue and blocks fetches until
// it is back. The substitution lookups are armed for the queue head.
reg [18:3] fq_addr [0:1];
reg        fq_clr  [0:1];
reg  [1:0] fq_cnt;
reg        fq_wp, fq_rp, c_out;
wire       fq_pop  = vf_ready && (fq_cnt != 0);
wire       fq_take = fetch_req && !fetch_ack && (fq_cnt != 2'd2) && vf_accept && !c_out;   // the ack takes a clock to drop the request

// RAM port state, declared here because the clear waits on it
localparam [3:0] P_IDLE = 4'd0, P_LOOK = 4'd1, P_LOOK2 = 4'd2, P_READ = 4'd3, P_MERGE = 4'd4, P_WRITE = 4'd5,
                 P_BIT = 4'd6, P_FMR_R0 = 4'd8, P_FMR_R1 = 4'd9, P_FMR_W0 = 4'd10, P_FMR_W1 = 4'd11,
                 P_LOOK1 = 4'd13;
reg  [3:0] p_state;
reg [17:0] p_wa;          // physical word address
wire       p_layer1 = p_wa[17];
// a layer 1 access between its lookup and its bit write: a clear must not
// start inside it, and a new one must not start while a clear is pending
wire       p_l1_open = p_layer1 && (p_state == P_LOOK1 || p_state == P_LOOK2 || p_state == P_READ ||
                                    p_state == P_MERGE || p_state == P_WRITE || p_state == P_BIT);
wire       p_l1_hold = p_layer1 && (clr_req || clr_active);

always @(posedge clk) begin
	if (reset) begin
		c_state <= C_IDLE;
		clr_done <= 0;
		armed <= 2'b00;
		fq_cnt <= 0; fq_wp <= 0; fq_rp <= 0; c_out <= 0;
		vf_req <= 0;
		fetch_ack <= 0;
		fetch_valid <= 0;
		st_we <= 0;
		c_rd_wait <= 0;
	end
	else begin
		clr_done <= 0;
		fetch_valid <= 0;
		fetch_ack <= 0;
		st_we <= 0;
		vf_req <= 0;

		if (fq_take) begin
			vf_a <= fetch_a;
			vf_req <= 1;
			fetch_ack <= 1;
			fq_addr[fq_wp] <= fetch_a;
			fq_clr[fq_wp] <= 0;
			fq_wp <= ~fq_wp;
		end
		else if (clr_active && !c_rd_done && !c_rd_wait && fq_cnt == 2'd0 && vf_accept) begin
			vf_a <= {1'b1, c_page, 7'd0, c_rd};
			vf_req <= 1;
			fq_addr[fq_wp] <= {1'b1, c_page, 7'd0, c_rd};
			fq_clr[fq_wp] <= 1;
			fq_wp <= ~fq_wp;
			c_rd_wait <= 1;
			c_out <= 1;
		end
		fq_cnt <= fq_cnt + {1'b0, fq_take || (clr_active && !c_rd_done && !c_rd_wait && fq_cnt == 2'd0 && vf_accept)} - {1'b0, fq_pop};

		// lookups for the substitution follow the queue head: a request into
		// an empty queue, or the one left behind a pop
		if (fq_take && (fq_cnt == 2'd0 || (fq_cnt == 2'd1 && fq_pop))) begin
			bm_b_addr <= {fetch_a[17], fetch_a[16:7]};
			st_ra_f   <= {fetch_a[17], fetch_a[9], fetch_a[8:3]};
		end
		else if (fq_pop && fq_cnt == 2'd2) begin
			bm_b_addr <= {fq_addr[~fq_rp][17], fq_addr[~fq_rp][16:7]};
			st_ra_f   <= {fq_addr[~fq_rp][17], fq_addr[~fq_rp][9], fq_addr[~fq_rp][8:3]};
		end

		if (fq_pop) begin
			fq_rp <= ~fq_rp;
			if (fq_clr[fq_rp]) begin
				st_wa <= {c_page, c_rd};
				st_wd <= vf_dout;
				st_we <= 1;
				c_rd <= c_rd + 1'd1;
				c_rd_wait <= 0;
				c_out <= 0;
				if (c_rd == 7'd127) c_rd_done <= 1;
			end
			else begin
				fetch_valid <= 1;
				// layer 1: substitute unset pixels of a cleared half, lines 2 and up
				if (fq_addr[fq_rp][18] && armed[fq_addr[fq_rp][17]] && fq_addr[fq_rp][16:9] >= 8'd2) begin
					fetch_data[15:0]  <= bm_b_q[{fq_addr[fq_rp][6:3], 2'd0}] ? vf_dout[15:0]  : st_q_f[15:0];
					fetch_data[31:16] <= bm_b_q[{fq_addr[fq_rp][6:3], 2'd1}] ? vf_dout[31:16] : st_q_f[31:16];
					fetch_data[47:32] <= bm_b_q[{fq_addr[fq_rp][6:3], 2'd2}] ? vf_dout[47:32] : st_q_f[47:32];
					fetch_data[63:48] <= bm_b_q[{fq_addr[fq_rp][6:3], 2'd3}] ? vf_dout[63:48] : st_q_f[63:48];
				end
				else fetch_data <= vf_dout;
			end
		end

		case (c_state)
		C_IDLE: if (clr_req && !clr_done && !p_l1_open) begin   // clr_done: the engine drops its request a clock later
			c_page <= clr_page;
			c_rd <= 0;
			c_rd_done <= 0;
			c_state <= C_RUN;
		end
		C_RUN: begin
			if (c_rd_done && c_zero_done) c_state <= C_END;
		end
		C_END: begin
			armed[c_page] <= 1;
			clr_done <= 1;
			c_state <= C_IDLE;
		end
		default: c_state <= C_IDLE;
		endcase

		if (dbg_bm_en) bm_b_addr <= dbg_bm_addr;
	end
end

// ---- RAM port: CPU, FMR plane bytes, sprite writes ----
// Every operation is a word at a physical address; a write that must keep
// some bits (write mask, byte lane into a substituted pixel) reads first.
// A write is posted: the port moves on and the next SDRAM access waits
// for it. The last bitmap entry read or written stays in a register, so
// the sprite pixels along a row skip the lookup.
reg  [1:0] p_client;      // 0 cpu, 1 fmr, 2 sprite
reg [10:0] bmc_addr;
reg [63:0] bmc_val;
reg        bmc_valid;
wire       vr_free = vr_ready && !vr_req;
reg        p_we;
reg  [1:0] p_be;
reg [15:0] p_din, p_old;
reg [15:0] p_mask;        // bit mask of the data actually written
reg        p_subst;       // this word lives in a cleared half
reg        p_bit;         // its valid bit
reg [15:0] p_stripe;
reg        p_have_old;
reg        p_rd_issued;   // the SDRAM read went out at acceptance

// FMR plane byte: four VRAM bytes, two words, at (page offset + 4n)
reg  [1:0] fmr_step;
reg [15:0] fmr_w0, fmr_w1;
reg  [7:0] fmr_byte;
wire [17:0] fmr_wa0 = {1'b0, cff83[4], fmr_a[14:0], 1'b0};   // page 1 of layer 0 is 0x20000 bytes up
wire [17:0] fmr_wa1 = {1'b0, cff83[4], fmr_a[14:0], 1'b1};
wire [31:0] fmr_pw  = plane_write(fmr_w0, fmr_w1, p_din[7:0], cff81[3:0]);

// linear (B00000) window: dword n of the linear space is dword n/2 of half n&1
wire [17:0] cpu_wa = cpu_a[20] ? {cpu_a[2], cpu_a[18:3], cpu_a[1]} : cpu_a[18:1];

// pixels of the plane byte: pixel j is the nibble j of bytes 4n..4n+3
function [7:0] plane_read;
	input [15:0] w0, w1;
	input [1:0] rc;
	integer j;
	reg [31:0] q;
	reg  [3:0] nib;
	begin
		q = {w1, w0};
		for (j = 0; j < 8; j = j + 1) begin
			nib = q[j * 4 +: 4];
			plane_read[7 - j] = nib[rc];
		end
	end
endfunction

function [31:0] plane_write;
	input [15:0] w0, w1;
	input [7:0] d;
	input [3:0] en;
	integer j, k;
	reg [31:0] q;
	begin
		q = {w1, w0};
		for (j = 0; j < 8; j = j + 1)
			for (k = 0; k < 4; k = k + 1)
				if (en[k]) q[j * 4 + k] = d[7 - j];
		plane_write = q;
	end
endfunction

wire [10:0] p_bm_entry = {p_wa[16], p_wa[15:6]};
wire  [5:0] p_bm_bit   = p_wa[5:0];
wire  [7:0] p_st_entry = {p_wa[16], p_wa[8], p_wa[7:2]};
wire        p_in_clear = p_layer1 && armed[p_wa[16]] && (p_wa[15:8] >= 8'd2);
wire        bm_free    = !(clr_active && !c_zero_done);
// a sprite write whose bitmap entry is the one in hand needs no lookup
wire [17:0] sp_wa      = sp_a[18:1];
wire        sp_hit     = bmc_valid && (bmc_addr == {sp_wa[16], sp_wa[15:6]}) && bm_free && !(clr_req || clr_active);
assign      sp_idle    = (p_state == P_IDLE) && !sp_req && vr_free;

always @(posedge clk) begin
	if (reset) begin
		p_state <= P_IDLE;
		cpu_ack <= 0;
		fmr_ack <= 0;
		sp_ack <= 0;
		vr_req <= 0;
		bm_a_we <= 0;
		bm_a_addr <= 0;
		bmc_valid <= 0;
		bmc_addr <= 0;
		bmc_val <= 0;
	end
	else begin
		cpu_ack <= 0;
		fmr_ack <= 0;
		sp_ack <= 0;
		vr_req <= 0;
		bm_a_we <= 0;

		// bitmap zeroing owns port A while it runs; the counter rests at
		// zero between clears
		if (!clr_active) begin
			c_zero <= 0;
			c_zero_done <= 0;
		end
		else if (!c_zero_done) begin
			bm_a_addr <= {c_page, c_zero};
			bm_a_wd <= 64'd0;
			bm_a_we <= 1;
			bmc_valid <= 0;
			c_zero <= c_zero + 1'd1;
			if (c_zero == 10'd1023) c_zero_done <= 1;
		end

		case (p_state)
		P_IDLE: begin
			p_have_old <= 0;
			p_rd_issued <= 0;
			// a CPU write is posted: acknowledged as it is taken, finished
			// here while the CPU moves on; a later request queues behind it.
			// A CPU read goes to the SDRAM at once when the port is free;
			// the layer 1 lookup runs while the SDRAM answers
			if (cpu_req && !cpu_ack) begin
				cpu_ack <= cpu_we;
				p_client <= 2'd0;
				p_wa <= cpu_wa;
				p_we <= cpu_we;
				p_be <= cpu_be;
				p_din <= cpu_din;
				p_mask <= {cpu_be[1] ? (raw ? 8'hFF : mask[cpu_a[1]][15:8]) : 8'h00, cpu_be[0] ? (raw ? 8'hFF : mask[cpu_a[1]][7:0]) : 8'h00};
				// layer 0 has no clear bitmap: skip the lookup
				p_subst <= 0;
				if (!cpu_we && vr_free) begin
					vr_a <= cpu_wa;
					vr_we <= 0;
					vr_req <= 1;
					p_rd_issued <= 1;
					p_state <= cpu_wa[17] ? P_LOOK : P_MERGE;
				end
				else p_state <= cpu_wa[17] ? P_LOOK : P_READ;
			end
			else if (fmr_req && !fmr_ack) begin
				p_client <= 2'd1;
				p_wa <= fmr_wa0;
				p_we <= fmr_we;
				p_be <= 2'b11;
				p_din <= {8'd0, fmr_din};
				p_mask <= 16'hFFFF;
				p_state <= P_FMR_R0;
			end
			else if (sp_req && !sp_ack) begin
				p_client <= 2'd2;
				p_wa <= sp_a;
				p_we <= 1;
				p_be <= 2'b11;
				p_din <= sp_din;
				p_mask <= 16'hFFFF;
				if (sp_hit) begin
					p_subst <= sp_wa[17] && armed[sp_wa[16]] && (sp_wa[15:8] >= 8'd2);
					p_bit <= bmc_val[sp_wa[5:0]];
					bm_a_addr <= {sp_wa[16], sp_wa[15:6]};
					p_state <= P_WRITE;
				end
				else p_state <= P_LOOK;
			end
		end

		// valid bit and stripe of the pixel, when it matters; the RAMs
		// answer one edge after the address
		P_LOOK: if (bm_free && !p_l1_hold) begin
			bm_a_addr <= p_bm_entry;
			st_ra_p2 <= p_st_entry;
			p_state <= P_LOOK1;
		end
		P_LOOK1: p_state <= P_LOOK2;
		P_LOOK2: begin
			p_subst <= p_in_clear;
			p_bit <= bm_a_q[p_bm_bit];
			p_stripe <= st_q_p2[{p_wa[1:0], 4'd0} +: 16];
			bmc_addr <= p_bm_entry;
			bmc_val <= bm_a_q;
			bmc_valid <= 1;
			p_state <= P_READ;
		end

		P_READ: begin
			if (p_we && p_mask == 16'hFFFF) p_state <= P_WRITE;   // whole word, nothing to keep
			else if (p_subst && !p_bit) begin
				// the SDRAM word is stale: the pixel is the stripe
				p_old <= p_stripe;
				p_have_old <= 1;
				p_state <= P_MERGE;
			end
			else if (p_rd_issued) p_state <= P_MERGE;
			else if (vr_free) begin
				vr_a <= p_wa;
				vr_we <= 0;
				vr_req <= 1;
				p_state <= P_MERGE;
			end
		end

		// only the CPU reads through here; the sprite always writes and the
		// FMR window has its own states
		P_MERGE: if (p_have_old || (vr_ready && !vr_req)) begin
			if (!p_have_old && !vr_req) p_old <= vr_dout;
			if (p_we) p_state <= P_WRITE;
			else begin
				cpu_dout <= p_have_old ? p_old : vr_dout;
				cpu_ack <= 1;
				p_state <= P_IDLE;
			end
		end

		P_WRITE: if (vr_free) begin
			// keep unmasked bits; a substituted pixel is written whole
			vr_a <= p_wa;
			vr_we <= 1;
			vr_be <= (p_subst && !p_bit) ? 2'b11 : p_be;
			vr_din <= (p_old & ~p_mask) | (p_din & p_mask);
			vr_req <= 1;
			if (p_client == 2'd2) sp_ack <= 1;
			p_state <= (p_subst && !p_bit) ? P_BIT : P_IDLE;
		end

		// the valid bit is set while the SDRAM write runs; the entry is on
		// port A since the lookup or the hit
		P_BIT: if (bm_free) begin
			bm_a_wd <= bmc_val | (64'd1 << p_bm_bit);
			bmc_val <= bmc_val | (64'd1 << p_bm_bit);
			bm_a_we <= 1;
			p_state <= P_IDLE;
		end

		// FMR plane byte: read both words, then assemble or rewrite
		P_FMR_R0: if (vr_free) begin
			vr_a <= fmr_wa0;
			vr_we <= 0;
			vr_req <= 1;
			fmr_step <= 0;
			p_state <= P_FMR_R1;
		end
		P_FMR_R1: if (vr_ready && !vr_req) begin
			if (fmr_step == 0) begin
				fmr_w0 <= vr_dout;
				vr_a <= fmr_wa1;
				vr_req <= 1;
				fmr_step <= 1;
			end
			else begin
				fmr_w1 <= vr_dout;
				if (p_we) p_state <= P_FMR_W0;
				else begin
					fmr_dout <= plane_read(fmr_w0, vr_dout, cff81[7:6]);
					fmr_ack <= 1;
					p_state <= P_IDLE;
				end
			end
		end
		P_FMR_W0: if (vr_free) begin
			{fmr_w1, fmr_w0} <= fmr_pw;
			vr_a <= fmr_wa0;
			vr_we <= 1;
			vr_be <= 2'b11;
			vr_din <= fmr_pw[15:0];
			vr_req <= 1;
			fmr_step <= 0;
			p_state <= P_FMR_W1;
		end
		P_FMR_W1: if (vr_ready && !vr_req) begin
			if (fmr_step == 0) begin
				vr_a <= fmr_wa1;
				vr_din <= fmr_w1;
				vr_req <= 1;
				fmr_step <= 1;
			end
			else begin
				fmr_ack <= 1;
				p_state <= P_IDLE;
			end
		end

		default: p_state <= P_IDLE;
		endcase
	end
end

endmodule

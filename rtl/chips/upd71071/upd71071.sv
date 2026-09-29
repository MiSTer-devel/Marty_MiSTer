// Copyright (c) 2026 Jamie Blanks
//
// NEC uPD71071 DMA controller: four channels, single and demand modes,
// fly-by transfers between memory and an acknowledged device. The
// registers sit on a byte bus (each half of a 16-bit register has its own
// address, so a 16-bit access is two byte accesses here).
//
//   DMARQ ──> SI ──> HLDRQ ──> S0 (HLDAK) ──> S1 ──> S2 ──> S3 ──> S4 ──┐
//             ^   address, DMAAK    read strobe   write strobe, READY   │
//             └────────── bus released after the service ──────────────┘
//
// One clk_ce pulse is one DMAC clock; bus strobes are level outputs held
// for the states the datasheet gives them. Memory-to-memory, cascade and
// bus-hold mode are not modelled: the Towns forbids them.

module upd71071
(
	input             clk,
	input             clk_ce,
	input             reset,

	// register side
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [3:0] a,
	input       [7:0] d_i,
	output      [7:0] d_o,
	output            d_oe,

	// bus master side
	output reg        hldrq,
	input             hldak,
	output            bus_oe,        // address and strobes driven
	output     [23:0] addr_o,
	output            ube_n,         // upper byte lane used
	output            mrd_n,
	output            mwr_n,
	output            iord_n,
	output            iowr_n,
	input             ready,
	input       [3:0] dmarq,
	output      [3:0] dmaak_n,
	input             end_n,
	output            tc_n,          // low during S3 of the last transfer
	output            tc_oe,

	// savestate port: the state as bytes, channels at 10h/20h/30h/40h
	input             ss_cs,
	input             ss_wr,
	input       [6:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

// ---- strobes ----
// A 16-bit CPU write is two byte lanes back to back with the write strobe
// held across both. Apply a register write both when the strobe ends and
// when the address steps to the next lane mid-strobe, so the low lane is
// not lost. a_q/d_q hold the lane being completed in either case.
reg  wr_q, rd_q;
reg  [3:0] a_q;
reg  [7:0] d_q;
wire wr_active = ~cs_n & ~wr_n;
wire rd_active = ~cs_n & ~rd_n;
wire lane_step = wr_q & wr_active & (a != a_q);
wire wr_edge = (wr_q & ~wr_active) | lane_step;
wire rd_edge = rd_q & ~rd_active;

always @(posedge clk) begin
	wr_q <= wr_active;
	rd_q <= rd_active;
	if (wr_active || rd_active) begin
		a_q <= a;
		d_q <= d_i;
	end
end

// ---- registers ----
reg        bus16;
reg  [1:0] sel;
reg        base_sel;
reg [23:0] base_addr [0:3];
reg [23:0] cur_addr  [0:3];
reg [15:0] base_cnt  [0:3];
reg [15:0] cur_cnt   [0:3];
reg  [7:0] mode      [0:3];
reg  [7:0] devctl;
reg  [1:0] devctl_hi;
reg  [3:0] tc_flag, srq, mask;
reg  [3:0] rq_q;                   // DMARQ as last sampled
reg  [1:0] rot_next;               // highest priority channel when rotating

wire       ddma = devctl[2];
wire       rot  = devctl[4];
wire       exw  = devctl[5];
wire       wev  = devctl_hi[1];        // READY waits apply to verify too

// The state port uses the guest's register port: its channel and data
// replace sel and d_q, so the channel arrays keep one guest-side write
// port and one read mux each. Bytes 10-1A, 20-2A, 30-3A, 40-4A are the
// four channels.
wire  [1:0] ss_ch    = ss_a[5:4] - 2'd1;
wire        ss_regs  = ss_cs && ss_wr && ss_a[6:4] != 3'd0 && ss_a[6:4] <= 3'd4;
wire  [1:0] gsel     = ss_cs ? ss_ch : sel;
wire  [7:0] gd       = ss_cs ? ss_din : d_q;
wire [23:0] g_base_addr = base_addr[gsel];
wire [23:0] g_cur_addr  = cur_addr[gsel];
wire [15:0] g_base_cnt  = base_cnt[gsel];
wire [15:0] g_cur_cnt   = cur_cnt[gsel];
wire  [7:0] g_mode      = mode[gsel];
// a guest write of a base byte also lands in the current register unless base_sel
wire        g_wr     = wr_edge && !ss_cs;
wire        g_cur    = g_wr && !base_sel;
wire  [2:0] ba_we    = ss_cs ? {3{ss_regs}} & {ss_a[3:0] == 4'h2, ss_a[3:0] == 4'h1, ss_a[3:0] == 4'h0}
                             : {3{g_wr}}  & {a_q == 4'h6, a_q == 4'h5, a_q == 4'h4};
wire  [2:0] ca_we    = ss_cs ? {3{ss_regs}} & {ss_a[3:0] == 4'h5, ss_a[3:0] == 4'h4, ss_a[3:0] == 4'h3}
                             : {3{g_cur}} & {a_q == 4'h6, a_q == 4'h5, a_q == 4'h4};
wire  [1:0] bc_we    = ss_cs ? {2{ss_regs}} & {ss_a[3:0] == 4'h7, ss_a[3:0] == 4'h6}
                             : {2{g_wr}}  & {a_q == 4'h3, a_q == 4'h2};
wire  [1:0] cc_we    = ss_cs ? {2{ss_regs}} & {ss_a[3:0] == 4'h9, ss_a[3:0] == 4'h8}
                             : {2{g_cur}} & {a_q == 4'h3, a_q == 4'h2};
wire        mode_we  = ss_cs ? ss_regs && ss_a[3:0] == 4'hA : g_wr && a_q == 4'hA;

wire [23:0] rd_addr = base_sel ? g_base_addr : g_cur_addr;
wire [15:0] rd_cnt  = base_sel ? g_base_cnt  : g_cur_cnt;
reg   [7:0] reg_dout;
always @* begin
	case (a)
	4'h0: reg_dout = {6'd0, bus16, 1'b0};
	4'h1: reg_dout = {3'd0, base_sel, sel == 2'd3, sel == 2'd2, sel == 2'd1, sel == 2'd0};
	4'h2: reg_dout = rd_cnt[7:0];
	4'h3: reg_dout = rd_cnt[15:8];
	4'h4: reg_dout = rd_addr[7:0];
	4'h5: reg_dout = rd_addr[15:8];
	4'h6: reg_dout = rd_addr[23:16];
	4'h8: reg_dout = devctl;
	4'h9: reg_dout = {6'd0, devctl_hi};
	4'hA: reg_dout = g_mode;
	4'hB: reg_dout = {rq_q, tc_flag};
	4'hE: reg_dout = {4'd0, srq};
	4'hF: reg_dout = {4'd0, mask};
	default: reg_dout = 8'h00;
	endcase
end
assign d_o  = reg_dout;
assign d_oe = rd_active;

// ---- transfer engine ----
localparam [2:0] SI = 3'd0, S0 = 3'd1, S1 = 3'd2, S2 = 3'd3, S3 = 3'd4, SW = 3'd5, S4 = 3'd6, SR = 3'd7;
reg  [2:0] state;
reg  [1:0] ch;                     // channel in service
reg        end_flag;
reg        served;                 // a channel ran during this bus grant

wire [3:0] pending = (dmarq & ~mask) | srq;
wire [1:0] ch_tmode = mode[ch][7:6];
wire       ch_dec   = mode[ch][5];
wire       ch_auti  = mode[ch][4];
wire       word     = bus16 & mode[ch][0];
wire       to_mem   = mode[ch][3:2] == 2'b01;   // I/O -> memory
wire       to_io    = mode[ch][3:2] == 2'b10;   // memory -> I/O
wire       verify   = mode[ch][3:2] == 2'b00;   // addresses only, no strobes
wire       last    = cur_cnt[ch] == 16'd0;
wire       in_xfer = state == S1 || state == S2 || state == S3 || state == SW || state == S4;

assign bus_oe  = in_xfer;
assign addr_o  = {cur_addr[ch][23:1], cur_addr[ch][0] & ~word};   // a word always goes out on an even address
assign ube_n   = word ? 1'b0 : bus16 ? ~cur_addr[ch][0] : 1'b1;   // an 8-bit bus never uses the upper lane
assign mrd_n   = ~(to_io  && (state == S2 || state == S3 || state == SW));
assign iord_n  = ~(to_mem && (state == S2 || state == S3 || state == SW));
assign mwr_n   = ~(to_mem && (state == S3 || state == SW || (exw && state == S2)));
assign iowr_n  = ~(to_io  && (state == S3 || state == SW || (exw && state == S2)));
assign dmaak_n = in_xfer ? ~(4'b0001 << ch) : 4'b1111;
assign tc_n    = ~(last && (state == S3 || state == SW));
assign tc_oe   = in_xfer;

// Priority pick: first pending channel from the rotation point.
wire [1:0] first = rot ? rot_next : 2'd0;
reg  [1:0] pick;
reg        pick_any;
integer i;
always @* begin
	pick = 2'd0; pick_any = 0;
	for (i = 3; i >= 0; i = i - 1)
		if (pending[(first + i[1:0]) & 2'd3]) begin
			pick = (first + i[1:0]) & 2'd3;
			pick_any = 1;
		end
end

wire [23:0] addr_step = word ? 24'd2 : 24'd1;

always @(posedge clk) begin
	if (reset) begin
		bus16 <= 0; sel <= 2'd0; base_sel <= 0;
		devctl <= 8'd0; devctl_hi <= 2'd0;
		tc_flag <= 4'd0; srq <= 4'd0; mask <= 4'hF; rq_q <= 4'd0; rot_next <= 2'd0;
		state <= SI; ch <= 2'd0; end_flag <= 0; served <= 0; hldrq <= 0;
		for (i = 0; i < 4; i = i + 1) mode[i] <= 8'd0;
	end
	else begin
		rq_q <= dmarq;

		// ---- register writes ----
		if (wr_edge) begin
			case (a_q)
			4'h0: begin
				bus16 <= d_q[1];
				if (d_q[0]) begin
					sel <= 2'd0; base_sel <= 0;
					devctl <= 8'd0; devctl_hi <= 2'd0;
					tc_flag <= 4'd0; srq <= 4'd0; mask <= 4'hF; rot_next <= 2'd0;
					for (i = 0; i < 4; i = i + 1) mode[i] <= 8'd0;
					state <= SI; hldrq <= 0;
				end
			end
			4'h1: begin sel <= d_q[1:0]; base_sel <= d_q[2]; end
			4'h8: devctl <= d_q;
			4'h9: devctl_hi <= d_q[1:0];
			4'hE: srq <= d_q[3:0];
			4'hF: mask <= d_q[3:0];
			default: ;
			endcase
		end
		if (ba_we[0]) base_addr[gsel][7:0]   <= gd;
		if (ba_we[1]) base_addr[gsel][15:8]  <= gd;
		if (ba_we[2]) base_addr[gsel][23:16] <= gd;
		if (ca_we[0]) cur_addr[gsel][7:0]    <= gd;
		if (ca_we[1]) cur_addr[gsel][15:8]   <= gd;
		if (ca_we[2]) cur_addr[gsel][23:16]  <= gd;
		if (bc_we[0]) base_cnt[gsel][7:0]    <= gd;
		if (bc_we[1]) base_cnt[gsel][15:8]   <= gd;
		if (cc_we[0]) cur_cnt[gsel][7:0]     <= gd;
		if (cc_we[1]) cur_cnt[gsel][15:8]    <= gd;
		if (mode_we)  mode[gsel]             <= gd;
		if (rd_edge && a_q == 4'hB) tc_flag <= 4'd0;

		// ---- DMA cycles ----
		if (clk_ce) begin
			case (state)
			SI: if (pick_any && !ddma) begin
				hldrq <= 1;
				state <= S0;
			end

			// the request may have gone (masked, or dropped) before the grant
			S0: if (hldak) begin
				ch <= pick;
				end_flag <= 0;
				served <= pick_any;
				state <= pick_any ? S1 : SR;
			end

			S1: state <= S2;

			S2: begin
				if (!end_n) end_flag <= 1;
				state <= S3;
			end

			S3, SW: state <= (ready || (verify && !wev)) ? S4 : SW;

			S4: begin
				cur_addr[ch] <= ch_dec ? cur_addr[ch] - addr_step : cur_addr[ch] + addr_step;
				cur_cnt[ch]  <= cur_cnt[ch] - 16'd1;
				if (last || end_flag) begin
					// service over: terminal count or END from the device
					tc_flag[ch] <= 1;
					if (ch_auti) begin
						cur_addr[ch] <= base_addr[ch];
						cur_cnt[ch]  <= base_cnt[ch];
					end
					else mask[ch] <= 1;
					state <= SR;
				end
				else if (ch_tmode == 2'b00 && dmarq[ch] && hldak) state <= S1;   // demand: keep going
				else if (ch_tmode == 2'b10 && hldak) state <= S1;                // block
				else state <= SR;
			end

			// Bus release: drop HLDRQ, wait for the CPU to take the bus back.
			// The address is still driven through S4 and the CPU here retakes
			// the bus in the T-state HOLD falls, so the drop waits for SR.
			SR: begin
				hldrq <= 0;
				srq <= 4'd0;
				if (rot && served) rot_next <= ch + 2'd1;
				if (!hldak) state <= SI;
			end

			default: state <= SI;
			endcase
		end
		if (ss_cs && ss_wr) begin
			if (ss_a[6:4] == 3'd0) begin
				case (ss_a[3:0])
				4'h0: {bus16, base_sel, sel, rot_next, end_flag, served} <= ss_din;
				4'h1: devctl <= ss_din;
				4'h2: devctl_hi <= ss_din[1:0];
				4'h3: {rq_q, tc_flag} <= ss_din;
				4'h4: {srq, mask} <= ss_din;
				4'h5: {state, ch, hldrq} <= ss_din[5:0];
				default: ;
				endcase
			end
		end
	end
end

always @* begin
	if (ss_a[6:4] == 3'd0) begin
		case (ss_a[3:0])
		4'h0: ss_dout = {bus16, base_sel, sel, rot_next, end_flag, served};
		4'h1: ss_dout = devctl;
		4'h2: ss_dout = {6'd0, devctl_hi};
		4'h3: ss_dout = {rq_q, tc_flag};
		4'h4: ss_dout = {srq, mask};
		4'h5: ss_dout = {2'd0, state, ch, hldrq};
		default: ss_dout = 8'h00;
		endcase
	end
	else if (ss_a[6:4] <= 3'd4) begin
		case (ss_a[3:0])
		4'h0: ss_dout = g_base_addr[7:0];
		4'h1: ss_dout = g_base_addr[15:8];
		4'h2: ss_dout = g_base_addr[23:16];
		4'h3: ss_dout = g_cur_addr[7:0];
		4'h4: ss_dout = g_cur_addr[15:8];
		4'h5: ss_dout = g_cur_addr[23:16];
		4'h6: ss_dout = g_base_cnt[7:0];
		4'h7: ss_dout = g_base_cnt[15:8];
		4'h8: ss_dout = g_cur_cnt[7:0];
		4'h9: ss_dout = g_cur_cnt[15:8];
		4'hA: ss_dout = g_mode;
		default: ss_dout = 8'h00;
		endcase
	end
	else ss_dout = 8'h00;
end

endmodule

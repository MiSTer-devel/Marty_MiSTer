// Copyright (c) 2026 Jamie Blanks
//
// Ricoh RF5C68A PCM sound generator, from the datasheet. Eight channels
// read 8-bit sign-magnitude samples (bit 7 set = positive, 80 = zero,
// 7F = -127) from a 64 KB wave memory through
// 16.11 fixed-point address pointers, scale them by ENV and the PAN
// nibbles, and sum into a 16-bit limiter every 384 source clocks.
//
//   CPU side (A12-A0, active-low CSB/RDB/WRB):
//     0000-0008  ENV PAN FDL FDH LSL LSH ST for the channel picked by CB,
//                control (ON, MOD, WB/CB), channel on/off (0 = sounding)
//     1000-1FFF  wave memory, 4 KB bank WB
//   Wave byte FF is the loop stop: the pointer reloads LS and reads again.
//   A silent channel keeps its pointer at ST<<8, so it starts there; an ST
//   write moves it at once. A sounding channel only steps by FD.
//   The wave memory powers up as FF (block RAM filled once after power-up).
//
//   The Towns board raises an interrupt when a channel's pointer leaves a
//   4 KB bank: bank_ev pulses with the bank just left (also when the loop
//   stop sits at the last byte of a bank).

module rf5c68
(
	input             clk,
	input             ce,            // source clock, 8 MHz
	input             reset_n,

	input      [12:0] a,
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [7:0] d_i,
	output      [7:0] d_o,
	output            d_oe,

	output reg signed [15:0] dac_l,  // limiter output, low 6 bits clear (10-bit DAC)
	output reg signed [15:0] dac_r,
	output reg        sample,        // one clk pulse per sample period
	output reg        sounding,      // control ON bit

	output reg        bank_ev,
	output reg  [3:0] bank_ev_bank,

	// savestate port: registers and pointers as bytes; 62h streams the
	// wave memory a byte per engine cycle, any access to 63h rewinds it
	input             ss_cs,
	input             ss_wr,
	input             ss_step,
	input       [7:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

// ---- registers ----
// Kept in logic: the engine reads a channel's entry in the same clock the
// CPU may write it, and a block RAM's mixed-port read-during-write is
// undefined on the target, where flops hand back the old value.
(* ramstyle = "logic" *) reg  [7:0] env [0:7];
(* ramstyle = "logic" *) reg  [7:0] pan [0:7];
(* ramstyle = "logic" *) reg [15:0] fd  [0:7];
(* ramstyle = "logic" *) reg [15:0] ls  [0:7];
(* ramstyle = "logic" *) reg  [7:0] st  [0:7];
reg  [3:0] wb;
reg  [2:0] cb;
reg  [7:0] ch_off;        // 1 = silent
reg [26:0] ptr [0:7];     // 16.11 address pointer

wire wr = ~cs_n & ~wr_n;
wire rd = ~cs_n & ~rd_n;
wire wave_sel = a[12];

// one write per bus cycle: WRB is a level, act on its falling edge. A
// 16-bit bus write holds WRB low across the address change, so a new
// address under a held WRB counts as another write.
reg  wr_q;
reg  [12:0] a_q;
wire wr_edge = wr & (~wr_q | a != a_q);
reg  st_wr;                // an ST write also moves a silent channel's pointer
reg  [2:0] st_ch;

// The state port goes through the channel registers' own ports: its
// index and data replace the guest's, so each array keeps one write port
// and one read mux. Bytes 00-07 ENV, 08-0F PAN, 10-1F FD, 20-2F LS,
// 30-37 ST, 40-5F the pointers.
wire        reg_wr   = wr_edge && !wave_sel;
wire        ss_reg   = ss_cs && ss_wr;
wire  [7:0] wd       = ss_cs ? ss_din : d_i;
wire  [2:0] wch      = ss_cs ? ss_a[2:0] : cb;
wire  [2:0] wch16    = ss_cs ? ss_a[3:1] : cb;
wire        env_we   = ss_cs ? ss_reg && ss_a[7:3] == 5'h00 : reg_wr && a[3:0] == 4'h0;
wire        pan_we   = ss_cs ? ss_reg && ss_a[7:3] == 5'h01 : reg_wr && a[3:0] == 4'h1;
wire        fd_we_lo = ss_cs ? ss_reg && ss_a[7:4] == 4'h1 && !ss_a[0] : reg_wr && a[3:0] == 4'h2;
wire        fd_we_hi = ss_cs ? ss_reg && ss_a[7:4] == 4'h1 &&  ss_a[0] : reg_wr && a[3:0] == 4'h3;
wire        ls_we_lo = ss_cs ? ss_reg && ss_a[7:4] == 4'h2 && !ss_a[0] : reg_wr && a[3:0] == 4'h4;
wire        ls_we_hi = ss_cs ? ss_reg && ss_a[7:4] == 4'h2 &&  ss_a[0] : reg_wr && a[3:0] == 4'h5;
wire        st_we    = ss_cs ? ss_reg && ss_a[7:3] == 5'h06 : reg_wr && a[3:0] == 4'h6;

always @(posedge clk) begin
	wr_q  <= wr;
	a_q   <= a;
	st_wr <= 0;
	if (!reset_n) begin
		sounding <= 0; wb <= 0; cb <= 0; ch_off <= 8'hFF;
	end
	else begin
		if (env_we)   env[wch] <= wd;
		if (pan_we)   pan[wch] <= wd;
		if (fd_we_lo) fd[wch16][7:0]  <= wd;
		if (fd_we_hi) fd[wch16][15:8] <= wd;
		if (ls_we_lo) ls[wch16][7:0]  <= wd;
		if (ls_we_hi) ls[wch16][15:8] <= wd;
		if (st_we)    st[wch] <= wd;
		if (reg_wr) begin
			case (a[3:0])
			4'h6: begin st_wr <= 1; st_ch <= cb; end
			4'h7: begin
				sounding <= d_i[7];
				if (d_i[6]) cb <= d_i[2:0]; else wb <= d_i[3:0];
			end
			4'h8: ch_off <= d_i;
			default: ;
			endcase
		end
		else if (ss_reg && ss_a[7:3] == 5'h07) begin
			case (ss_a[2:0])
			3'd0: wb <= ss_din[3:0];
			3'd1: {sounding, cb} <= {ss_din[7], ss_din[2:0]};
			3'd2: ch_off <= ss_din;
			default: ;
			endcase
		end
	end
end

// ---- wave memory ----
// Port A is the CPU window, port B the sample engine. A fill pass after
// power-up leaves every byte FF like the real chip's RAM comes up on a Marty.
reg        init_done = 0;
reg [15:0] init_a = 0;
reg [15:0] ss_wa;                      // wave stream address
wire       ss_wave = ss_cs && ss_a == 8'h62;
wire [15:0] cpu_a  = ss_cs ? ss_wa : {wb, a[11:0]};
always @(posedge clk) begin
	if (!reset_n || (ss_cs && ss_a == 8'h63)) ss_wa <= 16'd0;
	else if (ss_wave && ss_step) ss_wa <= ss_wa + 1'd1;
end
wire  [7:0] cpu_q;
reg  [15:0] eng_a;
wire  [7:0] eng_q;

always @(posedge clk) if (!init_done) {init_done, init_a} <= {1'b0, init_a} + 17'd1;

cache_ram_dp #(.ADDR_WIDTH(16), .DATA_WIDTH(8)) wave
(
	.clk_i(clk),
	.addr_a_i(init_done ? cpu_a : init_a),
	.wren_a_i(init_done ? ((wr & wave_sel & ~ss_cs) | (ss_wave & ss_wr)) : 1'b1),
	.wdata_a_i(init_done ? (ss_cs ? ss_din : d_i) : 8'hFF),
	.q_a_o(cpu_q),
	.addr_b_i(eng_a),
	.wren_b_i(1'b0),
	.wdata_b_i(8'h00),
	.q_b_o(eng_q)
);

assign d_o  = cpu_q;
assign d_oe = rd & wave_sel;

// ---- sample engine ----
// 384 source clocks per sample; the eight channels are walked one after
// another, four clocks each, well inside the period.
reg  [8:0] div;
reg        tick;
always @(posedge clk) begin
	tick <= 0;
	if (!reset_n) div <= 0;
	else if (ss_cs && ss_wr && ss_a == 8'h60) div[7:0] <= ss_din;
	else if (ss_cs && ss_wr && ss_a == 8'h61) div[8] <= ss_din[0];
	else if (ce) begin
		if (div == 9'd383) begin div <= 0; tick <= 1; end
		else div <= div + 1'd1;
	end
end

localparam [2:0] E_IDLE = 3'd0, E_ADDR = 3'd1, E_READ = 3'd2, E_LOOP = 3'd3, E_MIX = 3'd4, E_DONE = 3'd5;
reg  [2:0] e_state;
reg  [2:0] ch;
reg        looped;
reg signed [17:0] acc_l, acc_r;
reg  [15:0] cur_int;
wire  [2:0] rch     = ss_cs ? ss_a[2:0] : ch;
wire  [2:0] rch16   = ss_cs ? ss_a[3:1] : ch;
wire  [2:0] rchp    = ss_cs ? ss_a[4:2] : ch;
wire  [7:0] env_rd  = env[rch];
wire  [7:0] pan_rd  = pan[rch];
wire [15:0] fd_rd   = fd[rch16];
wire [15:0] ls_rd   = ls[rch16];
wire  [7:0] st_rd   = st[rch];
wire [26:0] ptr_rd  = ptr[rchp];
wire [15:0] ptr_int = ptr_rd[26:11];
wire        active  = sounding & ~ch_off[ch];

// sample x ENV x PAN nibble, >> 5, sign applied after (bit 7 clear = negative)
// The RAM byte is on eng_q through the E_LOOP clock: sample x ENV lands in
// a register at its end, and E_MIX multiplies that by PAN, applies the
// sign and accumulates, so no path runs from the RAM to the accumulator.
wire  [6:0] mag   = eng_q[6:0];
reg  [14:0] m_env_q;
reg         sign_q;
always @(posedge clk) begin
	m_env_q <= mag * env_rd;
	sign_q  <= eng_q[7];
end
wire [18:0] m_l   = m_env_q * pan_rd[3:0];
wire [18:0] m_r   = m_env_q * pan_rd[7:4];
wire signed [17:0] v_l = sign_q ? $signed({4'd0, m_l[18:5]}) : -$signed({4'd0, m_l[18:5]});
wire signed [17:0] v_r = sign_q ? $signed({4'd0, m_r[18:5]}) : -$signed({4'd0, m_r[18:5]});

wire [26:0] ptr_next = ptr_rd + {11'd0, fd_rd};
wire        bank_cross    = ptr_next[26:23] != ptr_rd[26:23];   // 4 KB bank boundary
wire        last_byte = ptr_int[11:0] == 12'hFFF;

// 16-bit limiter, then the top ten bits go to the DAC
function signed [15:0] limit(input signed [17:0] v);
	begin
		if (v > 18'sd32767) limit = 16'sd32767;
		else if (v < -18'sd32768) limit = 16'sh8000;
		else limit = v[15:0];
		limit[5:0] = 6'd0;
	end
endfunction

// The pointers' second write port: an ST write parks its channel, and the
// state port loads a byte at a time.
wire  [2:0] ptr_aux_ch = ss_cs ? ss_a[4:2] : st_ch;
wire        ptr_aux    = ss_cs ? ss_reg && ss_a[7:5] == 3'b010 : reset_n && st_wr && !(sounding && !ch_off[st_ch]);
wire  [3:0] ptr_aux_be = ss_cs ? 4'b0001 << ss_a[1:0] : 4'b1111;
wire [26:0] ptr_aux_wd = ss_cs ? {ss_din[2:0], ss_din, ss_din, ss_din} : {st[st_ch], 8'd0, 11'd0};

integer i;
always @(posedge clk) begin
	sample  <= 0;
	bank_ev <= 0;
	if (!reset_n) begin
		e_state <= E_IDLE;
		ch <= 0;
		acc_l <= 0; acc_r <= 0;
		dac_l <= 0; dac_r <= 0;
		for (i = 0; i < 8; i = i + 1) ptr[i] <= 0;
	end
	else case (e_state)
	E_IDLE: if (tick) begin
		ch <= 0;
		acc_l <= 0; acc_r <= 0;
		e_state <= E_ADDR;
	end

	// a silent channel is parked on ST; an active one reads its pointer
	E_ADDR: begin
		looped <= 0;
		if (!active) begin
			ptr[ch] <= {st_rd, 8'd0, 11'd0};
			e_state <= E_DONE;
		end
		else begin
			eng_a   <= ptr_int;
			cur_int <= ptr_int;
			e_state <= E_READ;
		end
	end

	E_READ: e_state <= E_LOOP;   // block RAM data lands one clock later

	// FF: reload the loop address and read once more
	E_LOOP: begin
		if (eng_q == 8'hFF && !looped) begin
			if (last_byte) begin bank_ev <= 1; bank_ev_bank <= cur_int[15:12]; end
			ptr[ch]  <= {ls_rd, 11'd0};
			eng_a    <= ls_rd;
			cur_int  <= ls_rd;
			looped   <= 1;
			e_state  <= E_READ;
		end
		else if (eng_q == 8'hFF) e_state <= E_DONE;   // loop stop at LS too: silent
		else e_state <= E_MIX;
	end

	E_MIX: begin
		acc_l <= acc_l + v_l;
		acc_r <= acc_r + v_r;
		ptr[ch] <= ptr_next;
		if (bank_cross) begin bank_ev <= 1; bank_ev_bank <= ptr_rd[26:23]; end
		e_state <= E_DONE;
	end

	E_DONE: begin
		if (ch == 3'd7) begin
			dac_l  <= limit(acc_l);
			dac_r  <= limit(acc_r);
			sample <= 1;
			e_state <= E_IDLE;
		end
		else begin
			ch <= ch + 1'd1;
			e_state <= E_ADDR;
		end
	end

	default: e_state <= E_IDLE;
	endcase
	if (ptr_aux) begin
		if (ptr_aux_be[0]) ptr[ptr_aux_ch][7:0]   <= ptr_aux_wd[7:0];
		if (ptr_aux_be[1]) ptr[ptr_aux_ch][15:8]  <= ptr_aux_wd[15:8];
		if (ptr_aux_be[2]) ptr[ptr_aux_ch][23:16] <= ptr_aux_wd[23:16];
		if (ptr_aux_be[3]) ptr[ptr_aux_ch][26:24] <= ptr_aux_wd[26:24];
	end
end

always @* begin
	case (ss_a[7:3])
	5'h00: ss_dout = env_rd;
	5'h01: ss_dout = pan_rd;
	5'h02, 5'h03: ss_dout = ss_a[0] ? fd_rd[15:8] : fd_rd[7:0];
	5'h04, 5'h05: ss_dout = ss_a[0] ? ls_rd[15:8] : ls_rd[7:0];
	5'h06: ss_dout = st_rd;
	5'h07: case (ss_a[2:0])
		3'd0: ss_dout = {4'd0, wb};
		3'd1: ss_dout = {sounding, 4'd0, cb};
		3'd2: ss_dout = ch_off;
		default: ss_dout = 8'h00;
		endcase
	5'h08, 5'h09, 5'h0A, 5'h0B: case (ss_a[1:0])
		2'd0: ss_dout = ptr_rd[7:0];
		2'd1: ss_dout = ptr_rd[15:8];
		2'd2: ss_dout = ptr_rd[23:16];
		default: ss_dout = {5'd0, ptr_rd[26:24]};
		endcase
	5'h0C: case (ss_a[2:0])
		3'd0: ss_dout = div[7:0];
		3'd1: ss_dout = {7'd0, div[8]};
		3'd2: ss_dout = cpu_q;
		default: ss_dout = 8'h00;
		endcase
	default: ss_dout = 8'h00;
	endcase
end

endmodule

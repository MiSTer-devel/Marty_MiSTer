// Copyright (c) 2026 Jamie Blanks
//
// JEIDA memory card slot as the 386SX Towns sees it: a 1 MB window at
// D00000 placed anywhere in the card's 64 MB by the bank register, the
// attribute-memory switch and the change/battery/protect status.
//
//   048A r   CHANGE(7) RED(5) YELLOW(4) CD1(2) CD0(1) WP(0); CHANGE clears on read
//   0490 r/w JB5-0: window bank, 1 MB units
//   0491 r/w bit 7 = 1 no JEIDA-4 card, bit 0 REG: attribute memory (reads FF)
//
// The card image is the HPS card image, block 0 up; a four block cache
// serves the CPU, which waits on a miss while the block is fetched. Dirty blocks go back when they are replaced or after 100 ms
// without an access. With no image the window reads FF and the first
// access raises warn, again after ten quiet seconds.

module towns_iccard
(
	input             clk,
	input             ce,             // fixed 16 MHz, for the write-back and warning timers
	input             reset,
	input             cpu_held,       // an image that arrives while the CPU waits is not a change

	// I/O registers
	input      [15:0] io_addr,
	input             io_rd,
	input             io_wr,
	input       [7:0] io_din,
	output reg  [7:0] io_dout,
	output            io_sel,

	// D00000-DFFFFF window: req rises for one word, ack answers it
	input      [19:1] mem_a,
	input       [1:0] mem_be,
	input             mem_we,
	input      [15:0] mem_din,
	input             mem_req,
	output reg [15:0] mem_dout,
	output reg        mem_ack,
	output reg        warn,           // one clock: window access with no image
	// savestate port: the window registers; writing 02h drops the cached lines
	input             ss_cs,
	input             ss_wr,
	input       [1:0] ss_a,
	input       [7:0] ss_din,
	output      [7:0] ss_dout,
	output            ss_quiet,       // no transfer running, nothing to write back

	// card image of the HPS service
	input             img_present,
	input      [17:0] img_blocks,

	// block port through hps_blk_mux
	output reg        blk_hold,
	input             blk_grant,
	output reg [17:0] blk_lba,
	output reg        blk_rd,
	output reg        blk_wr,
	input             blk_done,
	input             blk_err,
	output reg  [8:0] buf_addr,
	output reg        buf_we,
	output reg  [7:0] buf_din,
	input       [7:0] buf_dout
);

localparam [17:0] CARD_BASE = 18'd0;

// ---- registers ----
reg  [5:0] bank;
reg        attr;            // REG: attribute memory selected
reg        change;
reg        present_q;
wire       present     = img_present && img_blocks > CARD_BASE;
wire [17:0] card_blocks = img_blocks - CARD_BASE;

assign io_sel = io_addr == 16'h048A || io_addr == 16'h0490 || io_addr == 16'h0491;

reg rd_q;
always @* begin
	case (io_addr)
	16'h048A: io_dout = {change, 2'b00, 2'b00, ~present, ~present, 1'b0};
	16'h0490: io_dout = {2'b00, bank};
	default:  io_dout = {~present, 6'd0, attr};
	endcase
end

always @(posedge clk) begin
	present_q <= present;
	rd_q <= io_rd && io_addr == 16'h048A;
	if (reset) begin
		bank <= 0; attr <= 0; change <= 0;
	end
	else begin
		if (present != present_q && !cpu_held) change <= 1;
		if (rd_q && !(io_rd && io_addr == 16'h048A)) change <= 0;
		if (io_wr && io_addr == 16'h0490) bank <= io_din[5:0];
		if (io_wr && io_addr == 16'h0491) attr <= io_din[0];
		if (ss_cs && ss_wr && ss_a == 2'd0) bank <= ss_din[5:0];
		if (ss_cs && ss_wr && ss_a == 2'd1) {attr, change} <= ss_din[1:0];
	end
end
assign ss_dout  = ss_a == 2'd0 ? {2'd0, bank} : ss_a == 2'd1 ? {6'd0, attr, change} : 8'h01;
assign ss_quiet = cs == C_IDLE && !pending && dirty == 4'd0;

// ---- block cache: four 512-byte lines ----
reg  [3:0] valid, dirty;
reg [16:0] tag [0:3];       // card block number
reg  [1:0] victim;
reg [20:0] idle;            // 16 MHz ticks since the last access

reg  [19:1] q_a;
reg   [1:0] q_be;
reg         q_we;
reg  [15:0] q_din;
wire [25:0] q_off   = {bank, q_a, 1'b0};
wire [16:0] q_blk   = q_off[25:9];
wire        q_valid = present && !attr && {1'b0, q_blk} < card_blocks;

reg  [1:0] hit_way;
reg        hit;
integer i;
always @* begin
	hit = 0; hit_way = 2'd0;
	for (i = 0; i < 4; i = i + 1)
		if (valid[i] && tag[i] == q_blk) begin hit = 1; hit_way = i[1:0]; end
end

reg  [10:0] ram_a_addr, ram_b_addr;
reg         ram_a_we, ram_b_we;
reg   [7:0] ram_a_din, ram_b_din;
wire  [7:0] ram_a_q, ram_b_q;
cache_ram_dp #(.ADDR_WIDTH(11), .DATA_WIDTH(8)) lines
(
	.clk_i(clk),
	.addr_a_i(ram_a_addr), .wren_a_i(ram_a_we), .wdata_a_i(ram_a_din), .q_a_o(ram_a_q),
	.addr_b_i(ram_b_addr), .wren_b_i(ram_b_we), .wdata_b_i(ram_b_din), .q_b_o(ram_b_q)
);

localparam [3:0] C_IDLE = 4'd0, C_LOOK = 4'd1, C_ACCESS = 4'd2, C_WAIT = 4'd3, C_ANSWER = 4'd4, C_GRANT = 4'd5,
                 C_FLUSH_COPY = 4'd6, C_FLUSH = 4'd7, C_FETCH = 4'd8, C_FILL = 4'd9, C_RELEASE = 4'd10;
reg  [3:0] cs;
reg        pending;         // a CPU access waits behind the transfer
reg        mem_req_q;
reg  [1:0] way;             // line being moved
reg  [9:0] copy_i;
reg        fetch_after;     // flush is a replacement: fetch next

wire [1:0] repl = valid == 4'hF ? victim : ~valid[0] ? 2'd0 : ~valid[1] ? 2'd1 : ~valid[2] ? 2'd2 : 2'd3;
reg [27:0] holdoff;         // T-states until the next no-image warning
reg  [1:0] idle_way;
reg        idle_any;
always @* begin
	idle_any = 0; idle_way = 2'd0;
	for (i = 3; i >= 0; i = i - 1) if (dirty[i]) begin idle_any = 1; idle_way = i[1:0]; end
end

always @(posedge clk) begin
	mem_ack  <= 0;
	ram_a_we <= 0;
	ram_b_we <= 0;
	buf_we   <= 0;
	warn     <= 0;
	if (reset) begin
		valid <= 0; dirty <= 0; victim <= 0; idle <= 0; holdoff <= 0;
		cs <= C_IDLE; pending <= 0; blk_hold <= 0; blk_rd <= 0; blk_wr <= 0;
	end
	else begin
		if (!present || (ss_cs && ss_wr && ss_a == 2'd2)) begin valid <= 0; dirty <= 0; end
		if (ce && !(&idle)) idle <= idle + 1'd1;
		if (ce && holdoff != 0) holdoff <= holdoff - 1'd1;
		mem_req_q <= mem_req;
		if (mem_req && !mem_req_q) begin
			q_a <= mem_a; q_be <= mem_be; q_we <= mem_we; q_din <= mem_din;
			pending <= 1;
			idle <= 0;
		end

		case (cs)
		C_IDLE: begin
			if (pending) cs <= C_LOOK;
			else if (idle_any && idle >= 21'd1600000) begin
				// 100 ms without an access: write a dirty line back
				way <= idle_way;
				fetch_after <= 0;
				blk_hold <= 1;
				cs <= C_GRANT;
			end
		end

		C_LOOK: begin
			if (!q_valid) begin
				mem_dout <= 16'hFFFF;
				pending  <= 0;
				mem_ack  <= 1;
				cs <= C_IDLE;
				if (!present && holdoff == 0) begin
					warn    <= 1;
					holdoff <= 28'd160000000;
				end
			end
			else if (hit) begin
				way <= hit_way;
				cs  <= C_ACCESS;
			end
			else begin
				way <= repl;
				fetch_after <= 1;
				blk_hold <= 1;
				cs <= C_GRANT;
			end
		end

		// the two bytes of the word sit on the two ports
		C_ACCESS: begin
			ram_a_addr <= {way, q_a[8:1], 1'b0};
			ram_b_addr <= {way, q_a[8:1], 1'b1};
			if (q_we) begin
				ram_a_we  <= q_be[0];
				ram_b_we  <= q_be[1];
				ram_a_din <= q_din[7:0];
				ram_b_din <= q_din[15:8];
				dirty[way] <= 1;
			end
			cs <= C_WAIT;
		end
		C_WAIT: cs <= C_ANSWER;   // block RAM read latency
		C_ANSWER: begin
			mem_dout <= {ram_b_q, ram_a_q};
			pending  <= 0;
			mem_ack  <= 1;
			cs <= C_IDLE;
		end

		C_GRANT: if (blk_grant) begin
			copy_i <= 0;
			if (dirty[way]) cs <= C_FLUSH_COPY;
			else cs <= C_FETCH;
		end

		// flush: copy the line to the block buffer, then write it
		C_FLUSH_COPY: begin
			ram_a_addr <= {way, copy_i[8:0]};
			copy_i     <= copy_i + 1'd1;
			if (copy_i >= 10'd2) begin
				buf_addr <= copy_i[8:0] - 9'd2;
				buf_din  <= ram_a_q;
				buf_we   <= 1;
			end
			if (copy_i == 10'd513) begin
				blk_lba <= CARD_BASE + {1'b0, tag[way]};
				blk_wr  <= 1;
				cs <= C_FLUSH;
			end
		end
		C_FLUSH: if (blk_done) begin
			blk_wr <= 0;
			dirty[way] <= 0;
			copy_i <= 0;
			cs <= fetch_after ? C_FETCH : C_RELEASE;
		end

		// fetch: read the block, then copy it into the line. A failed
		// read answers FF and leaves the line empty.
		C_FETCH: begin
			blk_lba <= CARD_BASE + {1'b0, q_blk};
			blk_rd  <= 1;
			copy_i  <= 0;
			if (blk_done) begin
				blk_rd <= 0;
				if (blk_err) begin
					mem_dout <= 16'hFFFF;
					pending  <= 0;
					mem_ack  <= 1;
					cs <= C_RELEASE;
				end
				else cs <= C_FILL;
			end
		end
		C_FILL: begin
			buf_addr <= copy_i[8:0];
			copy_i   <= copy_i + 1'd1;
			if (copy_i >= 10'd2) begin
				ram_a_addr <= {way, copy_i[8:0] - 9'd2};
				ram_a_din  <= buf_dout;
				ram_a_we   <= 1;
			end
			if (copy_i == 10'd513) begin
				valid[way] <= 1;
				tag[way]   <= q_blk;
				victim     <= victim + 1'd1;
				cs <= C_RELEASE;
			end
		end
		C_RELEASE: begin
			blk_hold <= 0;
			cs <= C_IDLE;
		end
		default: cs <= C_IDLE;
		endcase
	end
end

endmodule

// Copyright (c) 2026 Jamie Blanks
//
// One OSD image slot of the framework block port, behind the guest block
// contract: the guest sets a block number and raises rd or wr, the block
// moves through the buffer, done pulses once with err valid. Linux answers
// each request in its own time; a request it never finishes ends with err
// after TIMEOUT clocks so a silent service cannot hold a guest forever.
//
//   req_rd/req_wr ─> sd_rd/sd_wr ─> sd_ack ┬─ sd_buff_wr: buffer <= Linux
//                                          └─ Linux <= buffer
//                    sd_ack falls ─> done
//
// The buffer is 2^BUF_AW bytes; the framework moves whatever block size
// Linux serves for the slot, up to that. With BANKED set a guest that keeps
// two blocks in flight selects the half the next one lands in with req_bank.

module hps_blk_slot #(parameter BUF_AW = 9, parameter LBA_W = 22, parameter TIMEOUT = 40000000, parameter BANKED = 0)
(
	input                  clk,
	input                  reset,

	// framework side, one entry of the hps_io arrays
	output reg      [31:0] sd_lba,
	output reg             sd_rd,
	output reg             sd_wr,
	input                  sd_ack,
	input           [13:0] sd_buff_addr,
	input            [7:0] sd_buff_dout,
	output           [7:0] sd_buff_din,
	input                  sd_buff_wr,
	input                  img_mounted,
	input                  img_readonly,
	input           [63:0] img_size,

	// guest side
	output reg             present,
	output reg             wp,
	output reg [LBA_W-1:0] blocks,        // 512-byte blocks in the image
	output reg             mounted,       // one clock: the image changed
	input      [LBA_W-1:0] req_lba,
	input                  req_bank,
	input                  req_rd,
	input                  req_wr,
	output                 busy,
	output reg             done,
	output reg             err,
	input     [BUF_AW-1:0] buf_addr,
	input                  buf_we,
	input            [7:0] buf_din,
	output           [7:0] buf_dout
);

localparam [2:0] S_IDLE = 3'd0, S_WAIT = 3'd1, S_MOVE = 3'd2, S_END = 3'd3, S_DROP = 3'd4;

reg  [2:0] state;
reg [31:0] timer;
reg        bank;

assign busy = state != S_IDLE;

always @(posedge clk) begin
	done    <= 0;
	mounted <= 0;
	if (reset) begin
		state   <= S_IDLE;
		sd_rd   <= 0;
		sd_wr   <= 0;
		sd_lba  <= 0;
		err     <= 0;
		present <= 0;
		wp      <= 0;
		blocks  <= 0;
		bank    <= 0;
	end
	else begin
		if (img_mounted) begin
			present <= img_size != 0;
			wp      <= img_readonly;
			blocks  <= img_size[LBA_W+8:9];
			mounted <= 1;
		end

		case (state)
		S_IDLE: if (req_rd || req_wr) begin
			sd_lba <= {{(32-LBA_W){1'b0}}, req_lba};
			sd_rd  <= req_rd;
			sd_wr  <= req_wr & ~req_rd;
			bank   <= req_bank;
			err    <= 0;
			timer  <= TIMEOUT;
			state  <= S_WAIT;
		end

		// hold the request until Linux takes it
		S_WAIT: begin
			if (sd_ack) begin
				sd_rd <= 0;
				sd_wr <= 0;
				state <= S_MOVE;
			end
			else if (timer == 0) begin
				sd_rd <= 0;
				sd_wr <= 0;
				err   <= 1;
				state <= S_END;
			end
			else timer <= timer - 1'd1;
		end

		S_MOVE: if (!sd_ack) state <= S_END;

		// one done, then wait for the guest to drop its request
		S_END: begin
			done  <= 1;
			state <= S_DROP;
		end

		S_DROP: if (!req_rd && !req_wr) state <= S_IDLE;

		default: state <= S_IDLE;
		endcase
	end
end

// framework port A, guest port B; the bank bit steers the framework side
wire [BUF_AW-1:0] fw_addr = BANKED ? {bank, sd_buff_addr[BUF_AW-2:0]} : sd_buff_addr[BUF_AW-1:0];

cache_ram_dp #(.ADDR_WIDTH(BUF_AW), .DATA_WIDTH(8)) buffer
(
	.clk_i(clk),
	.addr_a_i(fw_addr),
	.wren_a_i(sd_buff_wr & sd_ack),
	.wdata_a_i(sd_buff_dout),
	.q_a_o(sd_buff_din),
	.addr_b_i(buf_addr),
	.wren_b_i(buf_we),
	.wdata_b_i(buf_din),
	.q_b_o(buf_dout)
);

endmodule

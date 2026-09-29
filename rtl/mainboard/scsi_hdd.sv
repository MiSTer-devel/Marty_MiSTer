// Copyright (c) 2026 Jamie Blanks
//
// A SCSI hard disk on the card's bus: one target that answers the direct
// access commands Towns software uses, with 512-byte blocks served from the
// HPS block port. Selection, the REQ/ACK byte handshake and the phase lines
// follow the SCSI-1 initiator-target contract; the drive never disconnects.
//
//   free ─sel─> command ─> data in / data out ─> status ─> message in ─> free
//
// Each block of a READ or WRITE holds the block port only while its bytes
// move, so the card cache and the CMOS engine can slip in between blocks.
// A block the port cannot serve ends the command with CHECK CONDITION and
// a MEDIUM ERROR sense.

module scsi_hdd #(parameter ID = 0, parameter LBA_W = 22, parameter REQ_DELAY = 8)
(
	input                  clk,
	input                  ce,            // 16 MHz enable: the drive's own pace
	input                  reset,

	input                  present,
	input      [LBA_W-1:0] blocks,

	// SCSI bus, target side
	input                  bus_sel,
	input                  bus_atn,
	input                  bus_rst,
	input                  bus_ack,
	input            [7:0] bus_din,
	input                  bus_doe,       // the initiator drives the data lines
	output reg             bus_bsy,
	output reg             bus_req,
	output reg             bus_msg,
	output reg             bus_cd,
	output reg             bus_io,
	output           [7:0] bus_dout,

	// block port
	output reg             blk_hold,
	input                  blk_grant,
	output reg [LBA_W-1:0] blk_lba,
	output reg             blk_rd,
	output reg             blk_wr,
	// savestate port: the sense the next REQUEST SENSE answers, and idle
	input                  ss_cs,
	input                  ss_wr,
	input                  ss_a,
	input            [7:0] ss_din,
	output           [7:0] ss_dout,
	output                 ss_quiet,
	input                  blk_done,
	input                  blk_err,
	output           [8:0] buf_addr,
	output reg             buf_we,
	output           [7:0] buf_din,
	input            [7:0] buf_dout
);

localparam [7:0] OP_TEST_READY = 8'h00, OP_REZERO = 8'h01, OP_REQ_SENSE = 8'h03, OP_FORMAT = 8'h04,
                 OP_READ6 = 8'h08, OP_WRITE6 = 8'h0A, OP_SEEK6 = 8'h0B, OP_INQUIRY = 8'h12,
                 OP_MODE_SELECT = 8'h15, OP_MODE_SENSE = 8'h1A, OP_START_STOP = 8'h1B, OP_PREVENT = 8'h1E,
                 OP_READ_CAP = 8'h25, OP_READ10 = 8'h28, OP_WRITE10 = 8'h2A, OP_SEEK10 = 8'h2B, OP_VERIFY10 = 8'h2F;

localparam [3:0] SK_NONE = 4'h0, SK_MEDIUM = 4'h3, SK_ILLEGAL = 4'h5;

localparam [3:0] T_FREE = 4'd0, T_SELECTED = 4'd1, T_MSG_OUT = 4'd2, T_CMD = 4'd3, T_EXEC = 4'd4,
                 T_BLK_HOLD = 4'd5, T_BLK_READ = 4'd6, T_DATA_IN = 4'd7, T_DATA_OUT = 4'd8, T_BLK_WRITE = 4'd9,
                 T_STATUS = 4'd10, T_MSG_IN = 4'd11;

reg  [3:0] state;
reg  [7:0] cdb [0:11];
reg  [3:0] cdb_n, cdb_len;
reg  [7:0] out_byte;      // the byte on the bus in an in phase
reg  [7:0] in_byte;       // the last byte taken in an out phase
reg  [3:0] sense_key;
reg  [7:0] asc;
reg        check;         // status will be CHECK CONDITION
reg  [8:0] idx;           // byte within the block or the response
reg  [8:0] resp_len;      // bytes of a generated response
reg [15:0] blocks_left;
reg [LBA_W-1:0] lba;
reg        from_blocks;   // data in comes from the block buffer
reg        done_seen, err_seen;
reg  [3:0] pace;
reg  [1:0] settle;        // clocks for the buffer to answer a new idx

// the REQ/ACK handshake, one byte per pass:
//   H_RAISE: put the byte out (in phases), raise REQ
//   H_WAIT_ACK: initiator took it or gave it
//   H_WAIT_REL: REQ dropped, wait for ACK to drop, then pace the next
localparam [1:0] H_RAISE = 2'd0, H_WAIT_ACK = 2'd1, H_WAIT_REL = 2'd2, H_PACE = 2'd3;
reg  [1:0] hs;
wire       byte_done = (hs == H_PACE) && (pace == 0);

assign bus_dout = out_byte;
assign buf_addr = idx;
assign buf_din  = in_byte;

wire [7:0] op = cdb[0];
wire       is_write = (op == OP_WRITE6) || (op == OP_WRITE10);

// LBA and count from a group 0 or group 1 CDB
wire [20:0]      lba6      = {cdb[1][4:0], cdb[2], cdb[3]};
wire [31:0]      cdb_lba10 = {cdb[2], cdb[3], cdb[4], cdb[5]};
wire [15:0]      cdb_len6  = (cdb[4] == 8'd0) ? 16'd256 : {8'd0, cdb[4]};
wire [15:0]      cdb_len10 = {cdb[7], cdb[8]};
wire             lba10_big = |cdb_lba10[31:LBA_W];

wire [31:0]      cap_lba   = {{(32-LBA_W){1'b0}}, blocks - 1'd1};
wire [23:0]      cylinders = {{(24-LBA_W){1'b0}}, blocks >> 8};   // 8 heads x 32 sectors per cylinder

wire       bad_lun = (cdb[1][7:5] != 3'd0);
wire [8:0] mode_len = (cdb[2][5:0] == 6'h03 || cdb[2][5:0] == 6'h04) ? 9'd36 : (cdb[2][5:0] == 6'h3F) ? 9'd60 : 9'd12;

// generated responses: INQUIRY, REQUEST SENSE, READ CAPACITY, MODE SENSE
function [7:0] resp_byte(input [7:0] o, input [8:0] i);
	begin
		resp_byte = 8'h00;
		case (o)
		OP_INQUIRY: case (i)
			9'd0:  resp_byte = bad_lun ? 8'h7F : 8'h00;   // no device at that LUN
			9'd1:  resp_byte = 8'h00;
			9'd2:  resp_byte = 8'h01;   // SCSI-1 CCS
			9'd3:  resp_byte = 8'h01;
			9'd4:  resp_byte = 8'd31;
			// "FUJITSU " "MARTY SCSI DISK " "1.00"
			9'd8:  resp_byte = "F"; 9'd9:  resp_byte = "U"; 9'd10: resp_byte = "J"; 9'd11: resp_byte = "I";
			9'd12: resp_byte = "T"; 9'd13: resp_byte = "S"; 9'd14: resp_byte = "U"; 9'd15: resp_byte = " ";
			9'd16: resp_byte = "M"; 9'd17: resp_byte = "A"; 9'd18: resp_byte = "R"; 9'd19: resp_byte = "T";
			9'd20: resp_byte = "Y"; 9'd21: resp_byte = " "; 9'd22: resp_byte = "S"; 9'd23: resp_byte = "C";
			9'd24: resp_byte = "S"; 9'd25: resp_byte = "I"; 9'd26: resp_byte = " "; 9'd27: resp_byte = "D";
			9'd28: resp_byte = "I"; 9'd29: resp_byte = "S"; 9'd30: resp_byte = "K"; 9'd31: resp_byte = " ";
			9'd32: resp_byte = "1"; 9'd33: resp_byte = "."; 9'd34: resp_byte = "0"; 9'd35: resp_byte = "0";
			default: ;
			endcase
		OP_REQ_SENSE: case (i)
			9'd0:  resp_byte = 8'h70;
			9'd2:  resp_byte = {4'd0, sense_key};
			9'd7:  resp_byte = 8'd10;
			9'd12: resp_byte = asc;
			default: ;
			endcase
		OP_READ_CAP: case (i)
			9'd0: resp_byte = cap_lba[31:24];
			9'd1: resp_byte = cap_lba[23:16];
			9'd2: resp_byte = cap_lba[15:8];
			9'd3: resp_byte = cap_lba[7:0];
			9'd6: resp_byte = 8'h02;    // 512-byte blocks
			default: ;
			endcase
		OP_MODE_SENSE: begin
			// header, block descriptor, then page 3 (format) and 4 (geometry)
			case (i)
			9'd0:  resp_byte = mode_len[7:0] - 1'd1;
			9'd3:  resp_byte = 8'h08;
			9'd10: resp_byte = 8'h02;                             // 512-byte blocks
			9'd12: resp_byte = 8'h03; 9'd13: resp_byte = 8'd22;
			9'd23: resp_byte = 8'd32;                             // sectors per track
			9'd24: resp_byte = 8'h02;                             // 512 bytes per sector
			9'd36: resp_byte = 8'h04; 9'd37: resp_byte = 8'd22;
			9'd38: resp_byte = cylinders[23:16];
			9'd39: resp_byte = cylinders[15:8];
			9'd40: resp_byte = cylinders[7:0];
			9'd41: resp_byte = 8'd8;                              // heads
			default: ;
			endcase
		end
		default: ;
		endcase
	end
endfunction

// response length before the allocation cap
wire [8:0] alloc      = {1'b0, cdb[4]};
wire [8:0] gen_len    = (op == OP_INQUIRY)   ? (alloc < 9'd36 ? alloc : 9'd36) :
                        (op == OP_REQ_SENSE) ? (alloc < 9'd18 ? alloc : 9'd18) :
                        (op == OP_READ_CAP)  ? 9'd8 :
                        (op == OP_MODE_SENSE) ? (alloc < mode_len ? alloc : mode_len) : 9'd0;

// response and page bytes: page 4 shifts up when page 3 is not asked for
wire [8:0] page_idx = (cdb[2][5:0] == 6'h04 && idx >= 9'd12) ? idx + 9'd24 : idx;

wire capture = (state == T_DATA_OUT) && from_blocks;

always @(posedge clk) begin
	if (blk_done) begin done_seen <= 1; err_seen <= blk_err; end
	buf_we <= 0;
	if (settle != 0) settle <= settle - 1'd1;
	if (reset || bus_rst || !present) begin
		state    <= T_FREE;
		hs       <= H_RAISE;
		bus_bsy  <= 0; bus_req <= 0; bus_msg <= 0; bus_cd <= 0; bus_io <= 0;
		blk_hold <= 0; blk_rd <= 0; blk_wr <= 0;
		done_seen <= 0;
		sense_key <= SK_NONE;
		asc      <= 8'h00;
		check    <= 0;
		pace     <= 0;
		settle   <= 0;
		cdb_n    <= 0; cdb_len <= 0;
		idx      <= 0;
	end
	else begin
		// ---- the byte handshake, run by the phase states below ----
		if (ce) case (hs)
		H_WAIT_ACK: if (bus_ack) begin
			bus_req <= 0;
			in_byte <= bus_din;
			buf_we  <= capture;
			hs      <= H_WAIT_REL;
		end
		H_WAIT_REL: if (!bus_ack) begin pace <= REQ_DELAY[3:0]; hs <= H_PACE; end
		H_PACE:     if (pace != 0) pace <= pace - 1'd1;
		default: ;
		endcase

		case (state)
		// ---- bus free: answer a selection carrying our ID ----
		T_FREE: begin
			bus_bsy <= 0; bus_msg <= 0; bus_cd <= 0; bus_io <= 0; bus_req <= 0;
			blk_hold <= 0;
			if (ce && present && bus_sel && bus_doe && bus_din[ID]) begin
				bus_bsy <= 1;
				state   <= T_SELECTED;
			end
		end
		T_SELECTED: if (ce && !bus_sel) begin
			cdb_n   <= 0;
			cdb_len <= 4'd6;
			check   <= 0;
			hs      <= H_RAISE;
			state   <= bus_atn ? T_MSG_OUT : T_CMD;
		end

		// message out: take the identify message, then the command
		T_MSG_OUT: begin
			bus_msg <= 1; bus_cd <= 1; bus_io <= 0;
			if (ce && hs == H_RAISE) begin bus_req <= 1; hs <= H_WAIT_ACK; end
			if (byte_done) begin
				hs <= H_RAISE;
				if (!bus_atn) begin bus_msg <= 0; state <= T_CMD; end
			end
		end

		// ---- command: 6, 10 or 12 bytes by group ----
		T_CMD: begin
			bus_msg <= 0; bus_cd <= 1; bus_io <= 0;
			if (ce && hs == H_RAISE) begin bus_req <= 1; hs <= H_WAIT_ACK; end
			if (byte_done) begin
				hs <= H_RAISE;
				cdb[cdb_n] <= in_byte;
				if (cdb_n == 0) cdb_len <= (in_byte[7:5] == 3'd0) ? 4'd6 : (in_byte[7:5] == 3'd5) ? 4'd12 : 4'd10;
				cdb_n <= cdb_n + 1'd1;
				if (cdb_n + 1'd1 == cdb_len) state <= T_EXEC;
			end
		end

		// ---- decode: set up the data phase or go straight to status ----
		T_EXEC: begin
			idx <= 0;
			from_blocks <= 0;
			state <= T_STATUS;
			if (bad_lun && op != OP_INQUIRY && op != OP_REQ_SENSE) begin
				check <= 1; sense_key <= SK_ILLEGAL; asc <= 8'h25;   // no such LUN
			end
			else case (op)
			OP_TEST_READY, OP_REZERO, OP_SEEK6, OP_SEEK10, OP_START_STOP, OP_PREVENT, OP_VERIFY10: begin
				sense_key <= SK_NONE; asc <= 8'h00;
			end
			OP_INQUIRY, OP_READ_CAP, OP_MODE_SENSE, OP_REQ_SENSE: begin
				resp_len <= gen_len;
				if (op != OP_REQ_SENSE) begin sense_key <= SK_NONE; asc <= 8'h00; end
				else if (bad_lun) begin sense_key <= SK_ILLEGAL; asc <= 8'h25; end
				if (gen_len != 0) state <= T_DATA_IN;
			end
			OP_FORMAT: begin
				sense_key <= SK_NONE; asc <= 8'h00;
				if (cdb[1][4]) state <= T_DATA_OUT;
			end
			OP_MODE_SELECT: begin
				// swallow the parameter list
				sense_key <= SK_NONE; asc <= 8'h00;
				resp_len  <= alloc;
				if (alloc != 0) state <= T_DATA_OUT;
			end
			OP_READ6, OP_WRITE6, OP_READ10, OP_WRITE10: begin
				sense_key <= SK_NONE; asc <= 8'h00;
				from_blocks <= 1;
				lba         <= op[5] ? cdb_lba10[LBA_W-1:0] : {{(LBA_W-21){1'b0}}, lba6};
				blocks_left <= op[5] ? cdb_len10 : cdb_len6;
				if (op[5] && lba10_big) begin
					check <= 1; sense_key <= SK_ILLEGAL; asc <= 8'h21;
				end
				else if (!op[5] || cdb_len10 != 0) state <= T_BLK_HOLD;
			end
			default: begin
				check <= 1; sense_key <= SK_ILLEGAL; asc <= 8'h20;   // invalid opcode
			end
			endcase
		end

		// ---- one block: claim the port, read it (reads) or take the bytes (writes) ----
		T_BLK_HOLD: begin
			if (lba >= blocks) begin
				check <= 1; sense_key <= SK_ILLEGAL; asc <= 8'h21;   // LBA out of range
				state <= T_STATUS;
			end
			else begin
				blk_hold <= 1;
				if (blk_grant) begin
					idx       <= 0;
					done_seen <= 0;
					blk_lba   <= lba;
					if (is_write) state <= T_DATA_OUT;
					else begin blk_rd <= 1; state <= T_BLK_READ; end
				end
			end
		end
		T_BLK_READ: if (done_seen) begin
			blk_rd    <= 0;
			done_seen <= 0;
			if (err_seen) begin
				blk_hold <= 0;
				check <= 1; sense_key <= SK_MEDIUM; asc <= 8'h11;
				state <= T_STATUS;
			end
			else state <= T_DATA_IN;
		end
		T_BLK_WRITE: if (done_seen) begin
			blk_wr    <= 0;
			done_seen <= 0;
			blk_hold  <= 0;
			if (err_seen) begin
				check <= 1; sense_key <= SK_MEDIUM; asc <= 8'h0C;
				state <= T_STATUS;
			end
			else if (blocks_left == 0) state <= T_STATUS;
			else state <= T_BLK_HOLD;
		end

		// ---- data in: a block from the buffer or a generated response ----
		T_DATA_IN: begin
			bus_msg <= 0; bus_cd <= 0; bus_io <= 1;
			if (ce && hs == H_RAISE && settle == 0) begin
				out_byte <= from_blocks ? buf_dout : resp_byte(op, page_idx);
				bus_req  <= 1;
				hs       <= H_WAIT_ACK;
			end
			if (byte_done) begin
				hs     <= H_RAISE;
				idx    <= idx + 1'd1;
				settle <= 2'd2;
				if (from_blocks) begin
					if (idx == 9'd511) begin
						blk_hold    <= 0;
						lba         <= lba + 1'd1;
						blocks_left <= blocks_left - 1'd1;
						state       <= (blocks_left == 16'd1) ? T_STATUS : T_BLK_HOLD;
					end
				end
				else if (idx + 1'd1 == resp_len) begin
					state <= T_STATUS;
					if (op == OP_REQ_SENSE) begin sense_key <= SK_NONE; asc <= 8'h00; end
				end
			end
		end

		// ---- data out: a block into the buffer, or bytes to discard ----
		T_DATA_OUT: begin
			bus_msg <= 0; bus_cd <= 0; bus_io <= 0;
			if (ce && hs == H_RAISE) begin bus_req <= 1; hs <= H_WAIT_ACK; end
			if (byte_done) begin
				hs  <= H_RAISE;
				idx <= idx + 1'd1;
				if (from_blocks) begin
					if (idx == 9'd511) begin
						blk_wr      <= 1;
						blocks_left <= blocks_left - 1'd1;
						lba         <= lba + 1'd1;
						state       <= T_BLK_WRITE;
					end
				end
				else if (op == OP_FORMAT) begin
					// a 4-byte header sizes the defect list behind it; idx parks at 4
					if (idx == 9'd2) blocks_left <= {in_byte, 8'd0};
					if (idx == 9'd3) begin
						blocks_left[7:0] <= in_byte;
						if (blocks_left[15:8] == 8'd0 && in_byte == 8'd0) state <= T_STATUS;
					end
					if (idx >= 9'd4) begin
						idx         <= 9'd4;
						blocks_left <= blocks_left - 1'd1;
						if (blocks_left == 16'd1) state <= T_STATUS;
					end
				end
				else if (idx + 1'd1 == resp_len) state <= T_STATUS;
			end
		end

		// ---- status, then COMMAND COMPLETE, then bus free ----
		T_STATUS: begin
			bus_msg <= 0; bus_cd <= 1; bus_io <= 1;
			blk_hold <= 0;
			if (ce && hs == H_RAISE) begin
				out_byte <= check ? 8'h02 : 8'h00;
				bus_req  <= 1;
				hs       <= H_WAIT_ACK;
			end
			if (byte_done) begin hs <= H_RAISE; state <= T_MSG_IN; end
		end
		T_MSG_IN: begin
			bus_msg <= 1; bus_cd <= 1; bus_io <= 1;
			if (ce && hs == H_RAISE) begin
				out_byte <= 8'h00;
				bus_req  <= 1;
				hs       <= H_WAIT_ACK;
			end
			if (byte_done) begin hs <= H_RAISE; state <= T_FREE; end
		end
		default: state <= T_FREE;
		endcase
		if (ss_cs && ss_wr && !ss_a) {check, sense_key} <= ss_din[4:0];
		if (ss_cs && ss_wr &&  ss_a) asc <= ss_din;
	end
end
assign ss_dout  = ss_a ? asc : {3'd0, check, sense_key};
assign ss_quiet = state == T_FREE && !blk_hold;

endmodule

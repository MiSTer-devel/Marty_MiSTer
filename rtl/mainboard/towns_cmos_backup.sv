// Copyright (c) 2026 Jamie Blanks
//
// Battery backup for the CMOS RAM: a 16-block image on the HPS floppy
// service's second slot. The image is read into the RAM once per
// power-up, at the first mount; from then on the RAM is the truth and
// the image is rewritten, when the CPU has changed it, each time the OSD
// menu opens. A later mount keeps the running contents and saves them
// into the new image.
// A failed block ends the pass: a load leaves the RAM as it is, a save
// waits for the next CPU write.
//
// A mounted SCSI disk also gets a drive letter: Towns OS only lists the
// letters in the CMOS table (I/O 31DC on, type and unit per letter, 02 =
// SCSI, unit = ID<<4 | partition) and a Marty has no setup tool to add
// one. After the load, and whenever a disk appears, the first free letter
// from D: on becomes that disk's ID, partition 0, unless some letter
// already names that ID; 33CE moves the other way so the OS's byte sum stays.

module towns_cmos_backup
(
	input             clk,
	input             reset,

	input             cpu_wr,         // a CPU write landed in the RAM
	input             save,           // one clock: write the RAM back if it changed
	input       [1:0] hdd_present,    // SCSI disk images mounted, one bit per ID
	output            loading,

	// second port of the CMOS RAM
	output reg [12:0] ram_addr,
	output reg        ram_we,
	output reg  [7:0] ram_din,
	input       [7:0] ram_dout,

	// CMOS image of the HPS service
	input             img_present,
	input      [17:0] img_blocks,

	// floppy block port through hps_blk_mux
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

wire present = img_present && img_blocks >= 18'd16;

reg        loaded, dirty;
reg  [3:0] blk;
reg  [9:0] copy_i;
reg        saving;
reg  [2:0] st;
localparam [2:0] B_IDLE = 3'd0, B_GRANT = 3'd1, B_READ = 3'd2, B_FILL = 3'd3, B_COPY = 3'd4, B_WRITE = 3'd5, B_NEXT = 3'd6;

// drive-letter pass: the table is 16 letters of {type, unit} from byte
// index 0EE (I/O 31DC), the sum byte is at 1E7 (I/O 33CE)
localparam [12:0] TABLE = 13'h0EE, SUM = 13'h1E7;
reg  [1:0] assign_req, hdd_q;
reg        tid, found_scsi;             // the ID being given a letter
reg  [3:0] letter;
reg  [4:0] cand;                    // first free letter from D:, 16 = none
reg  [7:0] old_unit;
reg  [3:0] as;
localparam [3:0] A_IDLE = 4'd0, A_WAIT = 4'd1, A_CHECK = 4'd2, A_DECIDE = 4'd3, A_UNIT_WAIT = 4'd4,
                 A_UNIT_GET = 4'd5, A_UNIT_SET = 4'd6, A_SUM_ADDR = 4'd7, A_SUM_WAIT = 4'd8, A_SUM_FIX = 4'd9,
                 A_ID_WAIT = 4'd10, A_ID_CHECK = 4'd11;

// High from the clock the image appears, so a boot hold released on the
// same edge the load starts cannot slip past it.
assign loading = (st != B_IDLE && !saving) || (present && !loaded) || (|assign_req) || (as != A_IDLE);

always @(posedge clk) begin
	ram_we <= 0;
	buf_we <= 0;
	if (reset) begin
		loaded <= 0; dirty <= 0; st <= B_IDLE; saving <= 0;
		blk_hold <= 0; blk_rd <= 0; blk_wr <= 0;
		assign_req <= 0; hdd_q <= 0; as <= A_IDLE;
	end
	else begin
		if (cpu_wr) dirty <= 1;

		// a disk present when the reset ends counts as appearing
		hdd_q <= hdd_present;
		assign_req <= assign_req | (hdd_present & ~hdd_q);

		case (st)
		B_IDLE: begin
			if (present && !loaded) begin
				saving <= 0; blk <= 0; blk_hold <= 1; st <= B_GRANT;
			end
			else if (present && loaded && dirty && save) begin
				saving <= 1; blk <= 0; blk_hold <= 1; st <= B_GRANT;
			end
		end
		B_GRANT: if (blk_grant) begin
			copy_i <= 0;
			st <= saving ? B_COPY : B_READ;
		end

		// load: block into the RAM
		B_READ: begin
			blk_lba <= {14'd0, blk};
			blk_rd  <= 1;
			copy_i  <= 0;
			if (blk_done) begin
				blk_rd <= 0;
				if (blk_err) begin loaded <= 1; blk_hold <= 0; st <= B_IDLE; end
				else st <= B_FILL;
			end
		end
		B_FILL: begin
			// buf_dout shows the byte two clocks after buf_addr is set
			buf_addr <= copy_i[8:0];
			copy_i   <= copy_i + 1'd1;
			if (copy_i >= 10'd2) begin
				ram_addr <= {blk, copy_i[8:0] - 9'd2};
				ram_din  <= buf_dout;
				ram_we   <= 1;
			end
			if (copy_i == 10'd513) st <= B_NEXT;
		end

		// save: RAM into the buffer, then the block write
		B_COPY: begin
			if (copy_i == 0) dirty <= 0;
			ram_addr <= {blk, copy_i[8:0]};
			copy_i   <= copy_i + 1'd1;
			if (copy_i >= 10'd2) begin
				buf_addr <= copy_i[8:0] - 9'd2;
				buf_din  <= ram_dout;
				buf_we   <= 1;
			end
			if (copy_i == 10'd513) begin
				blk_lba <= {14'd0, blk};
				blk_wr  <= 1;
				st <= B_WRITE;
			end
		end
		B_WRITE: if (blk_done) begin
			blk_wr <= 0;
			if (blk_err) begin blk_hold <= 0; st <= B_IDLE; end
			else st <= B_NEXT;
		end

		B_NEXT: begin
			copy_i <= 0;
			if (blk == 4'd15) begin
				if (!saving) begin loaded <= 1; assign_req <= 2'b11; end
				blk_hold <= 0;
				st <= B_IDLE;
			end
			else begin
				blk <= blk + 1'd1;
				st  <= saving ? B_COPY : B_READ;
			end
		end
		default: st <= B_IDLE;
		endcase

		// the letter pass runs while the block engine is idle; the RAM
		// answers a read one clock after the address
		case (as)
		A_IDLE: if ((|assign_req) && st == B_IDLE) begin
			tid <= !assign_req[0];
			if (!hdd_present[!assign_req[0]]) assign_req[!assign_req[0]] <= 0;
			else begin
				letter <= 0; cand <= 5'd16; found_scsi <= 0;
				ram_addr <= TABLE;
				as <= A_WAIT;
			end
		end
		A_WAIT: as <= A_CHECK;
		// a SCSI letter also has its unit byte read for the ID
		A_CHECK: begin
			if (ram_dout == 8'hFF && letter >= 4'd3 && cand == 5'd16) cand <= {1'b0, letter};
			if (ram_dout == 8'h02) begin ram_addr <= ram_addr + 1'd1; as <= A_ID_WAIT; end
			else if (letter == 4'd15) as <= A_DECIDE;
			else begin
				letter   <= letter + 1'd1;
				ram_addr <= TABLE + {8'd0, letter + 1'd1, 1'b0};
				as <= A_WAIT;
			end
		end
		A_ID_WAIT: as <= A_ID_CHECK;
		A_ID_CHECK: begin
			if (ram_dout[7:4] == {3'd0, tid}) found_scsi <= 1;
			if (letter == 4'd15) as <= A_DECIDE;
			else begin
				letter   <= letter + 1'd1;
				ram_addr <= TABLE + {8'd0, letter + 1'd1, 1'b0};
				as <= A_WAIT;
			end
		end
		A_DECIDE: begin
			if (found_scsi || cand == 5'd16) begin assign_req[tid] <= 0; as <= A_IDLE; end
			else begin ram_addr <= TABLE + {8'd0, cand[3:0], 1'b1}; as <= A_UNIT_WAIT; end
		end
		A_UNIT_WAIT: as <= A_UNIT_GET;
		A_UNIT_GET: begin
			old_unit <= ram_dout;
			ram_addr <= TABLE + {8'd0, cand[3:0], 1'b0};
			ram_din  <= 8'h02;
			ram_we   <= 1;
			as <= A_UNIT_SET;
		end
		A_UNIT_SET: begin
			ram_addr <= TABLE + {8'd0, cand[3:0], 1'b1};
			ram_din  <= {3'd0, tid, 4'h0};
			ram_we   <= 1;
			as <= A_SUM_ADDR;
		end
		A_SUM_ADDR: begin ram_addr <= SUM; as <= A_SUM_WAIT; end
		A_SUM_WAIT: as <= A_SUM_FIX;
		// the type went FF -> 02 and the unit old -> ID<<4: the sum byte
		// takes the opposite change, old_unit - 3 - ID<<4 modulo 256
		A_SUM_FIX: begin
			ram_addr <= SUM;
			ram_din  <= ram_dout + old_unit - 8'd3 - {3'd0, tid, 4'h0};
			ram_we   <= 1;
			dirty    <= 1;
			assign_req[tid] <= 0;
			as <= A_IDLE;
		end
		default: as <= A_IDLE;
		endcase
	end
end

endmodule

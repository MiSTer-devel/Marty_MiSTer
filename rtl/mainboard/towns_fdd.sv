// Copyright (c) 2026 Jamie Blanks
//
// The Marty's 3.5-inch 3-mode floppy drive with the disk in it, as the
// FDC sees it: motor, spindle, head position, index hole and the byte
// stream under the head. The track under the head is a 16 KB record the
// HPS serves per (cylinder * 2 + head): a sector table and the sector
// data. The byte stream is built from that table, and a record the FDC
// wrote to goes back whole.
//
// Record layout, byte offsets:
//
//   0    sector count (0..32)     1    flags: bit0 2HD rate, bit1 unformatted, bit2 write protect,
//                                          bit3 slot table present
//   4    32 entries x 8 bytes:    C  H  R  N  status  density  off_lo  off_hi
//   256  sector data at each entry's offset, 128 << N bytes (N clipped to 3)
//   3FC0 32 x 16-bit slot lengths (bit3 set): bytes from one sector's sync to the next
//
//   status 00 ok, 10 deleted data mark, A0 bad ID CRC, B0 bad data CRC
//
// Without the slot table each sector takes 62 + size + GAP3 bytes. The HPS
// sends the table for a track that would not fit at that pitch: a protected
// disk whose sectors overlap, one sector's data field holding the next
// sector's ID. A data field always streams its full 128 << N bytes from
// the record once its mark has passed, whatever slot the head is in, and
// the next data mark starts the next field, so the FDC reads the same
// bytes the physical track carried.
//
// Track layout (Databook p.250), positions in bytes from the index hole:
//
//   0    GAP4a 80x4E   80 SYNC 12x00   92 IAM C2 C2 C2 FC   96 GAP1 50x4E
//   146  sector 0 ─┬─ SYNC 12x00  IDAM A1 A1 A1 FE  C H R N  CRC CRC
//                  │  GAP2 22x4E  SYNC 12x00  DAM A1 A1 A1 FB  data  CRC CRC
//                  └─ GAP3
//   ...  sector k follows sector k-1 in table order, pitch = 62 + size + GAP3
//        then GAP4b to the end of the track
//
// A byte passes under the head every 16 us (2HD) or 32 us (2DD). The FDC
// gets one byte_ce per byte with flags that say what the byte was.

module towns_fdd #(
	parameter FAST = 0        // simulation only: 4x rotation, 30 ms spin-up
)
(
	input             clk,
	input             ce_1m,          // 1 us tick
	input             reset,

	// drive interface
	input             select,
	input             motor,
	input             side,
	input             step,           // rising edge steps the head
	input             dirc,           // 1 = inward (higher cylinder)
	input             wg,             // write gate: bytes on wr_byte replace the track
	input             fmt,            // the write is a Write Track: parse the stream
	input             hispd,          // drive mode: 1 = 2HD data rate
	input             modeb,          // drive mode: 1 = 300 rpm at the 2HD rate (1.44 MB)
	output            ready,
	output            ip,             // index pulse
	output            tr00,
	output            wprt,
	output            dskchg,

	// byte stream under the head
	output reg        byte_ce,
	output reg  [7:0] rd_byte,
	output reg        f_index,        // first byte of the track
	output reg        f_am,           // an A1 sync byte with the missing clock
	output reg        f_id,           // last ID CRC byte: id_* describe the sector just passed
	output reg        f_dam,          // the data address mark
	output reg        f_data,         // a data byte
	output reg        f_data_last,
	input             fdc_field,      // the FDC is inside a data field: a passing mark does not take over
	// savestate port: head, spindle and rotation; a write to 06h with bit 0
	// drops the record so it is fetched again
	input             ss_cs,
	input             ss_wr,
	input       [2:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout,
	output            ss_quiet,       // no record on its way and none to write back
	output reg  [7:0] id_c, id_h, id_r, id_n,
	input       [7:0] wr_byte,
	input             wr_en,          // one byte from the FDC, aligned to byte_ce

	// track record slot: one 16 KB record per (cylinder, head)
	input             img_present,
	input             img_wp,
	input             img_mounted,    // one clock: the image changed
	output reg  [7:0] req_lba,        // cylinder * 2 + head
	output reg        req_rd,
	output reg        req_wr,
	input             blk_done,
	input             blk_err,
	output reg [13:0] buf_addr,
	output reg        buf_we,
	output reg  [7:0] buf_din,
	input       [7:0] buf_dout
);

// ---- the record under the head ----
reg  [5:0] n_count;        // sectors in the table, 0 when unformatted
reg        rec_2hd, rec_wp;
reg  [7:0] cur_c, cur_h, cur_r, cur_n, cur_st;   // table entry of the sector under the head
reg [13:0] cur_off;
reg        ent_req;        // the entry for sec_idx is being loaded
reg  [4:0] ms;             // record port slot after each byte
wire  [1:0] n_clip    = cur_n[7:2] != 0 ? 2'd3 : cur_n[1:0];
wire [10:0] sec_size  = 11'd128 << n_clip;
wire  [7:0] gap3      = n_clip == 2'd3 ? 8'd116 : n_clip == 2'd2 ? (rec_2hd ? 8'd84 : 8'd54) : n_clip == 2'd1 ? 8'd34 : 8'd22;
reg [11:0] cur_pitch;      // the slot table's length for this sector
reg        rec_slots;      // the record carries a slot table
wire [11:0] pitch     = rec_slots ? cur_pitch : 12'd62 + {1'b0, sec_size} + {4'd0, gap3};
// the drive reads only at the data rate the mode bits select
wire        mode_ok   = rec_2hd == hispd;
// 360 rpm in the 2HD 1.2 MB mode, 300 rpm otherwise
wire [13:0] track_len = (hispd && !modeb) ? 14'd10416 : hispd ? 14'd12500 : 14'd6250;
// a new image over one already in the drive is a disk swap: the drive
// shows no disk for a while, then the new one spins up
reg  [18:0] swap_us;
wire        present   = img_present && swap_us == 0;

// ---- motor, spindle, head ----
reg [18:0] spin_cnt;       // us since motor on, saturates at 300 ms
reg  [4:0] byte_cnt;
reg  [6:0] cyl;
reg        step_q, present_q, dskchg_r;
wire       spun_up   = spin_cnt == (FAST ? 19'd30000 : 19'd300000);
wire       byte_time = motor && (hispd ? byte_cnt == (FAST ? 5'd3 : 5'd15) : byte_cnt == (FAST ? 5'd7 : 5'd31));

assign ready  = select & present & motor & spun_up;
assign tr00   = select && cyl == 7'd0;
assign wprt   = img_wp | rec_wp | ~present;
assign dskchg = dskchg_r;

always @(posedge clk) begin
	step_q    <= step;
	present_q <= present;
	if (reset) begin
		// a disk already in the drive at power-on shows no change (real MX
		// observation); the images mount while the machine is held in reset
		spin_cnt <= 0; byte_cnt <= 0; cyl <= 0; dskchg_r <= ~img_present; swap_us <= 0;
	end
	else if (ss_cs && ss_wr) begin
		case (ss_a)
		3'd0: cyl <= ss_din[6:0];
		3'd1: spin_cnt[7:0] <= ss_din;
		3'd2: spin_cnt[15:8] <= ss_din;
		3'd3: {dskchg_r, spin_cnt[18:16]} <= {ss_din[7], ss_din[2:0]};
		default: ;
		endcase
	end
	else begin
		if (img_mounted && img_present && (present_q || swap_us != 0)) swap_us <= FAST ? 19'd50000 : 19'd500000;
		else if (ce_1m && swap_us != 0) swap_us <= swap_us - 1'd1;
		if (!motor || (present && !present_q)) spin_cnt <= 0;
		else if (ce_1m && !spun_up) spin_cnt <= spin_cnt + 1'd1;
		if (ce_1m) byte_cnt <= byte_time ? 5'd0 : byte_cnt + 1'd1;
		// a disk inserted while running raises the change line, and it
		// stays up until a step pulse finds a disk
		if (!img_present || img_mounted) dskchg_r <= 1;
		if (step && !step_q && select) begin
			if (present) dskchg_r <= 0;
			if (dirc) begin if (cyl != 7'd82) cyl <= cyl + 1'd1; end
			else if (cyl != 0) cyl <= cyl - 1'd1;
		end
	end
end

assign ss_quiet = xs == X_IDLE && !dirty;
always @* begin
	case (ss_a)
	3'd0: ss_dout = {1'b0, cyl};
	3'd1: ss_dout = spin_cnt[7:0];
	3'd2: ss_dout = spin_cnt[15:8];
	3'd3: ss_dout = {dskchg_r, 4'd0, spin_cnt[18:16]};
	3'd4: ss_dout = pos[7:0];
	3'd5: ss_dout = {2'd0, pos[13:8]};
	3'd6: ss_dout = 8'h01;   // the record is always refetched after a load
	default: ss_dout = 8'h00;
	endcase
end

// ---- rotation: position in the track as sector/offset counters ----
reg [13:0] pos;
reg  [5:0] sec_idx;        // sector under the head, n_count once in GAP4b
reg [11:0] sec_off;        // byte within that sector's slot
reg        settled;        // the record is in and the index has passed since
wire       pre_area  = pos < 14'd146;
wire       in_sector = !pre_area && sec_idx < n_count;
wire       marks     = in_sector && settled && mode_ok;
// the data field being streamed: started by a data mark, it runs for its
// full size and two CRC bytes even when the next slot begins before then.
// A mark inside it starts a new field only while the FDC is not reading
// or writing the field: a chip in the middle of one stays in that
// field's bit phase and takes what follows as data
reg        fld_on;
reg [13:0] fld_off;        // its data in the record
reg [10:0] fld_size;
reg [11:0] fld_cnt;        // bytes presented so far
reg        fld_bad;        // status B0: the CRC comes out wrong
wire       fld_data  = fld_on && fld_cnt < {1'b0, fld_size};
wire       dam_slot  = in_sector && sec_off == 12'd59 && marks;
wire       fld_take  = !(fld_on && fdc_field);   // a passing mark may start a field
// the slot's own sync, ID and data-mark bytes are presented even inside a
// streaming field; elsewhere the field's bytes are what passes the head
wire       slot_byte = in_sector && (sec_off < 12'd22 || (sec_off >= 12'd44 && sec_off < 12'd60));
wire [13:0] head_a   = fld_off + {2'd0, fld_cnt};
assign ip = select && present && motor && pos < 14'd64;

// ---- record fetch and write-back ----
reg        cache_ok;
reg  [6:0] cache_cyl;
reg        cache_side;
reg  [6:0] fetch_cyl;
reg        fetch_side;
reg        dirty, inval;
reg [12:0] idle_us;        // us since the write gate closed
reg  [1:0] hc;
reg  [1:0] xs;
localparam [1:0] X_IDLE = 2'd0, X_FETCH = 2'd1, X_HDR = 2'd2, X_FLUSH = 2'd3;
wire       stale     = !cache_ok || cache_cyl != cyl || cache_side != side;
wire       live      = cache_ok && !stale;
wire       idle_done = idle_us == (FAST ? 13'd2000 : 13'd8000);

// CRC-CCITT of the byte stream, running over the A1 A1 A1 mark and the
// field behind it; folded in one byte behind the head. The data field
// runs in crc; the ID CRC is built in crc_id as the entry loads, from the
// CRC of A1 A1 A1 FE, through the same stepper.
reg  [15:0] crc, crc_id;
reg         crc_fold;      // the byte just presented belongs to the data field CRC
function [15:0] crc_step(input [15:0] c, input [7:0] b);
	integer i;
	begin
		crc_step = c ^ {b, 8'd0};
		for (i = 0; i < 8; i = i + 1)
			crc_step = crc_step[15] ? {crc_step[14:0], 1'b0} ^ 16'h1021 : {crc_step[14:0], 1'b0};
	end
endfunction
wire        id_phase = ent_req && ms >= 5'd8 && ms <= 5'd11;
wire [15:0] crc_next = crc_step(id_phase ? crc_id : crc, id_phase ? buf_dout : rd_byte);

// The record port serves one access per slot of a short sequence after
// each byte: the FDC's byte lands at slot 2, the next data byte is
// prefetched at 3..5, a new table entry loads at 6..15 and the format
// parser's table bookkeeping writes at 16..22.
reg  [7:0] next_data;
wire [7:0] fld_byte = fld_cnt < {1'b0, fld_size} ? next_data :
                      fld_cnt == {1'b0, fld_size} ? crc_next[15:8] ^ {8{fld_bad}} : crc[7:0];   // bad data CRC
reg  [5:0] last_idx;
reg [13:0] last_a;
reg        last_in_data;
reg        last_dam;       // the byte just presented was the data mark slot
reg        st_fix;         // a written data mark sets the sector's status byte

// Write Track: the format stream is parsed into a new table
//   A1 A1 A1 FE C H R N ... A1 A1 A1 FB data
reg  [1:0] a1_cnt;
reg  [2:0] fs;             // 0 idle, 1-4 C H R N, 5 data
reg  [5:0] f_idx;          // entries written
reg [14:0] f_off;          // next free data byte
reg [10:0] f_cnt;
reg  [1:0] f_n;
reg        f_have_id, f_pend, f_hdr, fmt_q;
wire        f_idam  = a1_cnt == 2'd3 && wr_byte == 8'hFE;
wire        f_mark  = a1_cnt == 2'd3 && (wr_byte == 8'hFB || wr_byte == 8'hF8);
wire [10:0] f_size  = 11'd128 << f_n;
wire        f_room  = ({1'b0, f_off} + {5'd0, f_size}) <= 16'd16384;
wire [13:0] f_ent   = 14'd4 + {5'd0, f_idx, 3'd0};
wire [13:0] f_prev  = 14'd4 + {5'd0, f_idx - 6'd1, 3'd0};
wire [13:0] f_addr  = fs == 3'd5 ? f_off[13:0] + {3'd0, f_cnt} :
                      f_mark     ? f_prev + 14'd4 :
                                   f_ent + {11'd0, fs - 3'd1};
wire        f_we    = fs == 3'd5 ? f_room :
                      f_mark     ? f_have_id :
                                   fs != 3'd0 && f_idx < 6'd32;
wire  [7:0] f_din   = f_mark && fs != 3'd5 ? (wr_byte == 8'hF8 ? 8'h10 : 8'h00) : wr_byte;   // a data mark stores its status code

always @(posedge clk) begin
	buf_we  <= 0;
	byte_ce <= 0;
	f_index <= 0; f_id <= 0; f_dam <= 0; f_data <= 0; f_data_last <= 0;
	if (reset) begin
		pos <= 0; sec_idx <= 0; sec_off <= 0; settled <= 0; ms <= 0; ent_req <= 0; fld_on <= 0; crc_fold <= 0;
		cache_ok <= 0; dirty <= 0; inval <= 0; xs <= X_IDLE; req_rd <= 0; req_wr <= 0; idle_us <= 0;
		fs <= 0; a1_cnt <= 0; f_idx <= 0; f_off <= 0; f_pend <= 0; f_hdr <= 0; f_have_id <= 0; fmt_q <= 0; st_fix <= 0;
		n_count <= 0; rec_2hd <= 0; rec_wp <= 0; rec_slots <= 0; crc <= 16'hFFFF;
	end
	else begin
		fmt_q <= fmt & wg;
		if (img_mounted || !img_present) begin inval <= 1; cache_ok <= 0; end
		if (ss_cs && ss_wr) begin
			case (ss_a)
			3'd4: pos[7:0] <= ss_din;
			3'd5: pos[13:8] <= ss_din[5:0];
			3'd6: if (ss_din[0]) inval <= 1;
			default: ;
			endcase
		end
		if (stale) begin settled <= 0; fld_on <= 0; end
		if (wg) idle_us <= 0; else if (ce_1m && !idle_done) idle_us <= idle_us + 1'd1;

		// ---- the byte under the head ----
		if (ce_1m && byte_time) begin
			byte_ce  <= 1;
			ms       <= 5'd1;
			last_a   <= head_a;
			last_idx <= sec_idx;
			last_in_data <= fld_data;
			last_dam <= dam_slot;
			f_index  <= pos == 0 && settled;   // no index until the record is in
			if (pre_area) rd_byte <= pos < 14'd80 ? 8'h4E : pos < 14'd92 ? 8'h00 : pos < 14'd95 ? 8'hC2 : pos == 14'd95 ? 8'hFC : 8'h4E;
			else if (fld_on && !slot_byte) rd_byte <= fld_byte;
			else if (!in_sector)          rd_byte <= 8'h4E;
			else if (sec_off < 12'd12)    rd_byte <= 8'h00;
			else if (sec_off < 12'd15)    rd_byte <= 8'hA1;
			else if (sec_off == 12'd15)   rd_byte <= 8'hFE;
			else if (sec_off == 12'd16)   rd_byte <= cur_c;
			else if (sec_off == 12'd17)   rd_byte <= cur_h;
			else if (sec_off == 12'd18)   rd_byte <= cur_r;
			else if (sec_off == 12'd19)   rd_byte <= cur_n;
			else if (sec_off == 12'd20)   rd_byte <= crc_id[15:8] ^ {8{cur_st == 8'hA0}};   // bad ID CRC
			else if (sec_off == 12'd21)   rd_byte <= crc_id[7:0];
			else if (sec_off < 12'd44)    rd_byte <= 8'h4E;
			else if (sec_off < 12'd56)    rd_byte <= 8'h00;
			else if (sec_off < 12'd59)    rd_byte <= 8'hA1;
			else if (sec_off == 12'd59)   rd_byte <= cur_st == 8'h10 ? 8'hF8 : 8'hFB;
			else                          rd_byte <= 8'h4E;
			// the data field CRC restarts at the mark's sync bytes and folds
			// the mark, the data mark and every field byte
			if (in_sector && sec_off == 12'd56 && marks && fld_take) crc <= 16'hFFFF;
			else if (crc_fold) crc <= crc_next;
			crc_fold <= (in_sector && sec_off >= 12'd56 && sec_off <= 12'd59 && marks) || fld_data;
			f_am <= marks && (sec_off == 12'd12 || sec_off == 12'd13 || sec_off == 12'd14 ||
			                  sec_off == 12'd56 || sec_off == 12'd57 || sec_off == 12'd58);
			if (marks && sec_off == 12'd21) begin
				f_id <= 1;
				id_c <= cur_c; id_h <= cur_h; id_r <= cur_r; id_n <= cur_n;
			end
			f_dam       <= dam_slot;
			f_data      <= fld_data;
			f_data_last <= fld_data && fld_cnt == {1'b0, fld_size} - 1'd1;
			// the field: a data mark starts it, the last CRC byte ends it
			if (dam_slot && fld_take) begin
				fld_on <= 1; fld_off <= cur_off; fld_size <= sec_size; fld_cnt <= 0; fld_bad <= cur_st == 8'hB0;
			end
			else if (fld_on) begin
				if (fld_cnt == {1'b0, fld_size} + 1'd1) fld_on <= 0;
				else fld_cnt <= fld_cnt + 1'd1;
			end
			// advance
			if (pos == track_len - 1'd1) begin
				pos <= 0; sec_idx <= 0; sec_off <= 0; ent_req <= 1; fld_on <= 0;
				if (!stale) settled <= 1;
			end
			else begin
				pos <= pos + 1'd1;
				if (in_sector) begin
					if (sec_off == pitch - 1'd1) begin sec_off <= 0; sec_idx <= sec_idx + 1'd1; ent_req <= 1; end
					else sec_off <= sec_off + 1'd1;
				end
			end
		end

		// ---- record port sequence ----
		if (ms != 0) ms <= ms == 5'd23 ? 5'd0 : ms + 1'd1;
		case (ms)
		5'd3:  buf_addr <= head_a;
		5'd5:  next_data <= buf_dout;
		5'd6, 5'd7, 5'd8, 5'd9, 5'd10, 5'd11, 5'd12, 5'd13:
		       if (ent_req) buf_addr <= 14'd4 + {5'd0, sec_idx, 3'd0} + {9'd0, ms - 5'd6};
		5'd14, 5'd15:
		       if (ent_req) buf_addr <= 14'h3FC0 + {7'd0, sec_idx, ms[0]};
		5'd16: if (f_pend) begin buf_addr <= f_prev + 14'd4; buf_din <= 8'h00; buf_we <= 1; end
		       else if (st_fix) begin buf_addr <= 14'd8 + {5'd0, last_idx, 3'd0}; buf_din <= cur_st; buf_we <= 1; st_fix <= 0; end
		5'd17: if (f_pend) begin buf_addr <= f_prev + 14'd5; buf_din <= 8'h00; buf_we <= 1; end
		5'd18: if (f_pend) begin buf_addr <= f_prev + 14'd6; buf_din <= f_off[7:0]; buf_we <= 1; end
		5'd19: if (f_pend) begin buf_addr <= f_prev + 14'd7; buf_din <= {1'b0, f_off[14:8]}; buf_we <= 1; end
		5'd20: if (f_pend) begin buf_addr <= 14'd0; buf_din <= {2'd0, f_idx}; buf_we <= 1; f_pend <= 0; end
		5'd21: if (f_hdr) begin buf_addr <= 14'd0; buf_din <= 8'h00; buf_we <= 1; end
		5'd22: if (f_hdr) begin buf_addr <= 14'd1; buf_din <= {5'd0, rec_wp, 1'b0, hispd}; buf_we <= 1; f_hdr <= 0; end
		default: ;
		endcase
		if (ent_req) case (ms)
		5'd7:  crc_id <= 16'hB230;   // CRC of A1 A1 A1 FE
		5'd8:  begin cur_c  <= buf_dout; crc_id <= crc_next; end
		5'd9:  begin cur_h  <= buf_dout; crc_id <= crc_next; end
		5'd10: begin cur_r  <= buf_dout; crc_id <= crc_next; end
		5'd11: begin cur_n  <= buf_dout; crc_id <= crc_next; end
		5'd12: cur_st <= buf_dout;
		5'd14: cur_off[7:0]  <= buf_dout;
		5'd15: cur_off[13:8] <= buf_dout[5:0];
		5'd16: cur_pitch[7:0]  <= buf_dout;
		5'd17: begin cur_pitch[11:8] <= buf_dout[3:0]; ent_req <= 0; end
		default: ;
		endcase

		// ---- record transfers ----
		// a dirty record goes back once the FDC has been quiet for a while
		// or before the head leaves the track; a fetch that fails reads as
		// an unformatted track
		case (xs)
		X_IDLE: begin
			if (inval) begin inval <= 0; cache_ok <= 0; dirty <= 0; end
			else if (present && dirty && !wg && (idle_done || stale)) begin
				req_lba <= {cache_cyl, cache_side};
				req_wr  <= 1;
				dirty   <= 0;
				xs <= X_FLUSH;
			end
			else if (present && stale && !dirty) begin
				cache_ok   <= 0;
				fetch_cyl  <= cyl;
				fetch_side <= side;
				req_lba    <= {cyl, side};
				req_rd     <= 1;
				xs <= X_FETCH;
			end
		end
		X_FETCH: if (blk_done) begin
			req_rd <= 0;
			hc     <= 0;
			if (blk_err) begin
				n_count <= 0; rec_2hd <= hispd; rec_wp <= 0; rec_slots <= 0;
				cache_ok <= 1; cache_cyl <= fetch_cyl; cache_side <= fetch_side;
				xs <= X_IDLE;
			end
			else xs <= X_HDR;
		end
		X_HDR: begin
			hc <= hc + 1'd1;
			case (hc)
			2'd0: buf_addr <= 14'd0;
			2'd1: buf_addr <= 14'd1;
			2'd2: n_count  <= buf_dout > 8'd32 ? 6'd32 : buf_dout[5:0];
			default: begin
				if (buf_dout[1]) n_count <= 0;
				rec_2hd <= buf_dout[0];
				rec_wp  <= buf_dout[2];
				rec_slots <= buf_dout[3];
				cache_ok <= 1; cache_cyl <= fetch_cyl; cache_side <= fetch_side;
				xs <= X_IDLE;
			end
			endcase
		end
		X_FLUSH: if (blk_done) begin
			req_wr <= 0;
			xs <= X_IDLE;
		end
		default: xs <= X_IDLE;
		endcase

		// ---- writes from the FDC (only the selected drive takes them) ----
		// a format starts a new table; the track takes the drive's rate
		if (fmt && wg && !fmt_q && live && select) begin
			f_idx <= 0; f_off <= 15'd256; f_hdr <= 1; f_have_id <= 0; fs <= 0; a1_cnt <= 0;
			n_count <= 0; rec_2hd <= hispd; rec_slots <= 0; dirty <= 1;
		end
		if (wr_en && wg && live && select) begin
			if (fmt) begin
				if (f_we) begin buf_addr <= f_addr; buf_din <= f_din; buf_we <= 1; dirty <= 1; end
				if (fs == 3'd5) begin
					if (f_cnt == f_size - 1'd1) begin fs <= 0; f_off <= f_off + {4'd0, f_size}; end
					else f_cnt <= f_cnt + 1'd1;
				end
				else if (wr_byte == 8'hA1) a1_cnt <= a1_cnt == 2'd3 ? 2'd3 : a1_cnt + 1'd1;
				else begin
					a1_cnt <= 0;
					if (f_idam) fs <= 3'd1;
					else if (f_mark) begin
						if (f_have_id) begin fs <= 3'd5; f_cnt <= 0; f_have_id <= 0; end
					end
					else case (fs)
					3'd1: fs <= 3'd2;
					3'd2: fs <= 3'd3;
					3'd3: fs <= 3'd4;
					3'd4: begin
						fs  <= 0;
						f_n <= wr_byte[7:2] != 0 ? 2'd3 : wr_byte[1:0];
						if (f_idx < 6'd32) begin
							f_idx <= f_idx + 1'd1; n_count <= f_idx + 1'd1;
							f_pend <= 1; f_have_id <= 1;
						end
					end
					default: ;
					endcase
				end
			end
			else if (last_dam) begin
				if (last_idx == sec_idx && cur_st != 8'hA0) begin
					cur_st <= wr_byte == 8'hF8 ? 8'h10 : 8'h00;
					st_fix <= 1;
				end
			end
			else if (last_in_data) begin
				buf_addr <= last_a; buf_din <= wr_byte; buf_we <= 1; dirty <= 1;
			end
		end
		if (!wg) begin fs <= 0; a1_cnt <= 0; end
	end
end

endmodule

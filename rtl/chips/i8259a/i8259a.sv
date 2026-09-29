// Copyright (c) 2026 Jamie Blanks
//
// Intel 8259A programmable interrupt controller, 8086 vector mode.
//
//   IR[7:0] ──> edge/level sense ──> IRR ──┬─ priority resolver ──> INT
//                                  IMR ──┘        │
//                                          ISR <──┘ set at the first INTA
//                                                   vector on D at the second
//
// Priority rotates around `bottom`, the lowest-priority level. A master
// puts the requesting slave's ID on CAS during the acknowledge; a slave
// answers only when CAS carries its own ID. All state moves on strobe
// edges: a write at the rising edge of WR_n, an acknowledge at the falling
// edge of INTA_n.

module i8259a
(
	input             clk,
	input             reset,

	input             a0,
	input             cs_n,
	input             rd_n,
	input             wr_n,
	input       [7:0] d_i,
	output      [7:0] d_o,
	output            d_oe,

	input       [7:0] ir,
	output reg        int_o,
	input             inta_n,

	input       [2:0] cas_i,
	output reg  [2:0] cas_o,
	output            cas_oe,       // master drives CAS
	input             sp_n,         // 1 master, 0 slave (unbuffered mode)

	// savestate port: the whole state as eight bytes, no side effects
	input             ss_cs,
	input             ss_wr,
	input       [2:0] ss_a,
	input       [7:0] ss_din,
	output reg  [7:0] ss_dout
);

// ---- strobes ----
reg  wr_q, rd_q, inta_q;
reg  a0_q;
reg  [7:0] d_q;
wire wr_active = ~cs_n & ~wr_n;
wire rd_active = ~cs_n & ~rd_n;
wire wr_edge   = wr_q & ~wr_active;
wire rd_edge   = rd_q & ~rd_active;
wire inta_fall = inta_q & ~inta_n;
wire inta_rise = ~inta_q & inta_n;

always @(posedge clk) begin
	wr_q <= wr_active;
	rd_q <= rd_active;
	inta_q <= inta_n;
	if (wr_active || rd_active) begin
		a0_q <= a0;
		d_q  <= d_i;
	end
end

// ---- registers ----
reg  [7:0] imr, isr, edge_latch;
reg  [7:3] icw2;
reg  [7:0] icw3;
reg        ltim, single, ic4, sfnm, buf_mode, buf_master, aeoi;
reg  [1:0] icw_step;      // 0 = ready, 2/3/4 = next ICW expected
reg  [2:0] bottom;        // lowest-priority level
reg        smm;           // special mask mode
reg        read_isr;      // OCW3 RIS
reg        poll;          // next read answers the poll
reg        rot_aeoi;
reg        inta_second;   // the next INTA edge is the second one
reg  [2:0] ack_level;
reg        ack_valid;     // a request was accepted at the first INTA
reg        drive_vec;     // this chip supplies the vector
reg        int_blank;     // INT held low for a clock after the acknowledge

wire master = buf_mode ? buf_master : sp_n;
assign cas_oe = master;

// ---- request sense ----
wire [7:0] ir_q_rise;
reg  [7:0] ir_q;
assign ir_q_rise = ir & ~ir_q;
wire [7:0] irr = ltim ? ir : (ir & edge_latch);

// Priority of a level: 0 is highest. Rotation offsets by `bottom`.
function [2:0] prio(input [2:0] level, input [2:0] bot);
	prio = level - bot - 3'd1;
endfunction

// Highest-priority pending request and the in-service level it must beat.
reg  [2:0] req_level, isr_level;
reg        req_any, isr_any;
reg  [2:0] req_prio, isr_prio;
wire [7:0] isr_eff = smm ? (isr & ~imr) : isr;
wire [7:0] pending = irr & ~imr;
integer k;
always @* begin
	req_any = 0; req_level = 3'd7; req_prio = 3'd7;
	isr_any = 0; isr_level = 3'd7; isr_prio = 3'd7;
	for (k = 7; k >= 0; k = k - 1) begin
		if (pending[k[2:0]] && (!req_any || prio(k[2:0], bottom) < req_prio)) begin
			req_any = 1; req_level = k[2:0]; req_prio = prio(k[2:0], bottom);
		end
		if (isr_eff[k[2:0]] && (!isr_any || prio(k[2:0], bottom) < isr_prio)) begin
			isr_any = 1; isr_level = k[2:0]; isr_prio = prio(k[2:0], bottom);
		end
	end
end

// A slave input already in service still lets a higher request through
// in special fully nested mode.
wire slave_line = !single && master && icw3[req_level];
wire accept = req_any && (!isr_any || req_prio < isr_prio ||
                          (sfnm && slave_line && req_prio == isr_prio));

// ---- data out ----
wire [7:0] poll_byte = {accept, 4'd0, req_level};
wire [7:0] status    = read_isr ? isr : irr;
wire [7:0] vector    = {icw2[7:3], ack_level};
wire       vec_phase = ~inta_n & inta_second & drive_vec;
assign d_o  = vec_phase ? vector : poll ? poll_byte : a0 ? imr : status;
assign d_oe = rd_active | vec_phase;

// Highest-priority in-service bit, for non-specific EOI.
wire [2:0] eoi_level = isr_level;

always @* begin
	case (ss_a)
	3'd0: ss_dout = imr;
	3'd1: ss_dout = isr;
	3'd2: ss_dout = edge_latch;
	3'd3: ss_dout = {icw2, 3'd0};
	3'd4: ss_dout = icw3;
	3'd5: ss_dout = {ltim, single, ic4, sfnm, buf_mode, buf_master, aeoi, rot_aeoi};
	3'd6: ss_dout = {icw_step, bottom, smm, read_isr, poll};
	default: ss_dout = {inta_second, ack_valid, drive_vec, int_blank, 1'b0, ack_level};
	endcase
end

always @(posedge clk) begin
	if (reset) begin
		imr <= 8'd0; isr <= 8'd0; edge_latch <= 8'd0; ir_q <= 8'd0;
		icw2 <= 5'd0; icw3 <= 8'd0;
		ltim <= 0; single <= 0; ic4 <= 0; sfnm <= 0; buf_mode <= 0; buf_master <= 0; aeoi <= 0;
		icw_step <= 2'd0; bottom <= 3'd7; smm <= 0; read_isr <= 0; poll <= 0; rot_aeoi <= 0;
		inta_second <= 0; ack_level <= 3'd0; ack_valid <= 0; drive_vec <= 0;
		int_blank <= 0; int_o <= 0; cas_o <= 3'd0;
	end
	else begin
		ir_q <= ir;
		edge_latch <= edge_latch | ir_q_rise;
		// INT always drops once an acknowledge completes, even with another
		// request already waiting, so an edge-sensing master sees a new edge
		int_blank <= 0;
		int_o <= accept & ~int_blank;
		// A master names the slave on CAS from the end of the first INTA
		// to the end of the second; the lines rest low otherwise.
		if (master && inta_rise) cas_o <= (!inta_second && ack_valid && !single && icw3[ack_level]) ? ack_level : 3'd0;

		// ---- writes ----
		if (wr_edge) begin
			if (!a0_q && d_q[4]) begin
				// ICW1
				ltim   <= d_q[3];
				single <= d_q[1];
				ic4    <= d_q[0];
				imr <= 8'd0; isr <= 8'd0; edge_latch <= 8'd0;
				bottom <= 3'd7; smm <= 0; read_isr <= 0; poll <= 0; rot_aeoi <= 0;
				inta_second <= 0; ack_valid <= 0; drive_vec <= 0;
				if (!d_q[0]) begin
					sfnm <= 0; buf_mode <= 0; aeoi <= 0;
				end
				icw_step <= 2'd2;
			end
			else if (a0_q && icw_step != 2'd0) begin
				case (icw_step)
				2'd2: begin
					icw2 <= d_q[7:3];
					icw_step <= !single ? 2'd3 : ic4 ? 2'd1 : 2'd0;
				end
				2'd3: begin
					icw3 <= d_q;
					icw_step <= ic4 ? 2'd1 : 2'd0;
				end
				default: begin   // ICW4
					sfnm       <= d_q[4];
					buf_mode   <= d_q[3];
					buf_master <= d_q[2];
					aeoi       <= d_q[1];
					icw_step   <= 2'd0;
				end
				endcase
			end
			else if (a0_q) imr <= d_q;   // OCW1
			else if (!d_q[3]) begin
				// OCW2
				case (d_q[7:5])
				3'b001: if (isr_any) isr[eoi_level] <= 0;
				3'b011: isr[d_q[2:0]] <= 0;
				3'b101: if (isr_any) begin isr[eoi_level] <= 0; bottom <= eoi_level; end
				3'b111: begin isr[d_q[2:0]] <= 0; bottom <= d_q[2:0]; end
				3'b110: bottom <= d_q[2:0];
				3'b100: rot_aeoi <= 1;
				3'b000: rot_aeoi <= 0;
				default: ;
				endcase
			end
			else begin
				// OCW3
				if (d_q[6]) smm <= d_q[5];
				if (d_q[1]) read_isr <= d_q[0];
				poll <= d_q[2];
			end
		end

		// ---- poll: the read acts as an acknowledge ----
		if (rd_edge && poll) begin
			poll <= 0;
			if (accept) begin
				isr[req_level] <= 1;
				edge_latch[req_level] <= 0;
			end
		end

		// ---- acknowledge ----
		if (inta_fall) begin
			if (!inta_second) begin
				// first INTA: freeze the winner; nothing pending means IR7
				ack_valid <= accept;
				ack_level <= accept ? req_level : 3'd7;
				if (master) begin
					drive_vec <= !(accept && !single && icw3[req_level]);
					if (accept) begin
						isr[req_level] <= 1;
						edge_latch[req_level] <= 0;
					end
				end
			end
			else if (!master) begin
				// slave: commit only when addressed
				drive_vec <= cas_i == icw3[2:0];
				if (cas_i == icw3[2:0] && ack_valid) begin
					isr[ack_level] <= 1;
					edge_latch[ack_level] <= 0;
				end
			end
		end
		if (inta_rise) inta_second <= ~inta_second;
		if (inta_rise && inta_second) begin
			drive_vec <= 0;
			if (ack_valid && (master || cas_i == icw3[2:0])) int_blank <= 1;
			if (aeoi && ack_valid && (master || cas_i == icw3[2:0])) begin
				isr[ack_level] <= 0;
				if (rot_aeoi) bottom <= ack_level;
			end
		end
		// a savestate write replaces the byte outright
		if (ss_cs && ss_wr) begin
			case (ss_a)
			3'd0: imr <= ss_din;
			3'd1: isr <= ss_din;
			3'd2: edge_latch <= ss_din;
			3'd3: icw2 <= ss_din[7:3];
			3'd4: icw3 <= ss_din;
			3'd5: {ltim, single, ic4, sfnm, buf_mode, buf_master, aeoi, rot_aeoi} <= ss_din;
			3'd6: {icw_step, bottom, smm, read_isr, poll} <= ss_din;
			default: {inta_second, ack_valid, drive_vec, int_blank, ack_level} <= {ss_din[7:4], ss_din[2:0]};
			endcase
		end
	end
end

endmodule

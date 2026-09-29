// Copyright (c) 2026 Jamie Blanks
//
// Epson RTC-58323 (MSM58321-compatible) real-time clock: thirteen BCD
// digit registers behind a 4-bit bus, a 32.768 kHz divider and a busy
// flag around each seconds carry (8 counts before, 6 after).
//
//   D3-D0 in ─┬─ while ADRS ──> address latch
//             └─ while WR ────> register[address]
//   register[address] ──> D3-D0 out while CS and RD
//
// The chip powers up undefined; the seed port is the framework's way of
// setting it to the host clock and has no pin on the real part.

module rtc58323
(
	input             clk,
	input             reset,
	input             ce_32k,        // 32768 Hz

	input             cs,
	input             rd,
	input             wr,
	input             adrs,
	input       [3:0] d_i,
	output      [3:0] d_o,
	output            d_oe,
	output            busy_n,

	input             seed_valid,    // load the BCD bytes below
	input       [6:0] seed_sec,
	input       [6:0] seed_min,
	input       [5:0] seed_hour,     // 24 h
	input       [2:0] seed_wday,     // 1 = Sunday
	input       [5:0] seed_day,
	input       [4:0] seed_month,
	input       [7:0] seed_year
);

reg  [3:0] s1, mi1, h1, d1, mo1, y1, y10;
reg  [2:0] s10, mi10, w;
reg  [3:0] h10;                      // {24h, PM, tens}
reg  [3:0] d10;                      // {leap select, tens}
reg        mo10;
reg  [3:0] addr;
reg [14:0] sub;
reg        busy;
reg        min_carry, hour_carry;    // the last seconds carry also rolled the minute / hour

wire mode24 = h10[3];
wire pm     = h10[2];

assign busy_n = ~busy;

// Internal 1 Hz pulse: four counts high from the carry.
wire tick1 = sub[14:2] == 13'd0;

// ---- register read ----
reg [3:0] sel;
always @* begin
	case (addr)
	4'h0: sel = s1;
	4'h1: sel = {1'b0, s10};
	4'h2: sel = mi1;
	4'h3: sel = {1'b0, mi10};
	4'h4: sel = h1;
	4'h5: sel = h10;
	4'h6: sel = {1'b0, w};
	4'h7: sel = d1;
	4'h8: sel = d10;
	4'h9: sel = mo1;
	4'hA: sel = {3'd0, mo10};
	4'hB: sel = y1;
	4'hC: sel = y10;
	4'hD: sel = 4'd0;
	default: sel = {~(tick1 & hour_carry), ~(tick1 & min_carry), ~tick1, sub[4]};   // E/F: reference signals
	endcase
end
assign d_o  = sel;
assign d_oe = cs & rd;

// ---- calendar ----
// Year modulo 4 from the two BCD digits, matched against the leap select.
wire [1:0] ymod4    = {y10[0], 1'b0} + y1[1:0];
wire       leap     = ymod4 == (2'd0 - d10[3:2]);
wire       feb      = mo10 == 1'b0 && mo1 == 4'd2;
wire       thirty   = (mo10 == 1'b0 && (mo1 == 4'd4 || mo1 == 4'd6 || mo1 == 4'd9)) || (mo10 == 1'b1 && mo1 == 4'd1);
wire [1:0] dt       = d10[1:0];
wire       last_day = feb    ? (dt == 2'd2 && d1 == (leap ? 4'd9 : 4'd8)) :
                      thirty ? (dt == 2'd3 && d1 == 4'd0) :
                               (dt == 2'd3 && d1 == 4'd1);
wire       hour_end = mode24 ? (h10[1:0] == 2'd2 && h1 == 4'd3)            // 23 -> 00
                             : (h10[0] && h1 == 4'd1);                     // 11 -> 12 (AM/PM flip)
wire       day_end  = mode24 ? hour_end : (hour_end && pm);

always @(posedge clk) begin
	if (reset) begin
		{s1, s10, mi1, mi10, h1, w, d1, mo1, mo10, y1, y10} <= 0;
		h10 <= 4'b1000; d10 <= 4'd1; d1 <= 4'd1; mo1 <= 4'd1;
		addr <= 4'd0; sub <= 15'd0; busy <= 0;
		min_carry <= 0; hour_carry <= 0;
	end
	else if (seed_valid) begin
		s1 <= seed_sec[3:0];    s10 <= seed_sec[6:4];
		mi1 <= seed_min[3:0];   mi10 <= seed_min[6:4];
		h1 <= seed_hour[3:0];   h10 <= {2'b10, seed_hour[5:4]};
		w <= seed_wday - 3'd1;
		d1 <= seed_day[3:0];    d10 <= {2'b00, seed_day[5:4]};
		mo1 <= seed_month[3:0]; mo10 <= seed_month[4];
		y1 <= seed_year[3:0];   y10 <= seed_year[7:4];
		sub <= 15'd0; busy <= 0;
	end
	else begin
		// ---- bus ----
		// Both strobes act on level: the latches follow D while high.
		// A write is ignored from the carry until busy ends, so a pulse
		// landing there is lost and a held write lands afterwards.
		if (cs && adrs) addr <= d_i;
		if (cs && wr && !(busy && sub < 15'd8)) begin
			case (addr)
			4'h0: s1 <= d_i;
			4'h1: s10 <= d_i[2:0];
			4'h2: mi1 <= d_i;
			4'h3: mi10 <= d_i[2:0];
			4'h4: h1 <= d_i;
			4'h5: h10 <= {d_i[3], d_i[2] & ~d_i[3], d_i[1:0]};
			4'h6: w <= d_i[2:0];
			4'h7: d1 <= d_i;
			4'h8: d10 <= d_i;
			4'h9: mo1 <= d_i;
			4'hA: mo10 <= d_i[0];
			4'hB: y1 <= d_i;
			4'hC: y10 <= d_i;
			default: ;
			endcase
		end

		// ---- time ----
		if (ce_32k) begin
			sub <= sub + 1'd1;
			if (sub == 15'd32759) busy <= 1;
			if (sub == 15'd5)     busy <= 0;
			if (sub == 15'h7FFF) begin
				min_carry  <= s1 == 4'd9 && s10 == 3'd5;
				hour_carry <= s1 == 4'd9 && s10 == 3'd5 && mi1 == 4'd9 && mi10 == 3'd5;
				if (s1 != 4'd9) s1 <= s1 + 1'd1;
				else begin
					s1 <= 0;
					if (s10 != 3'd5) s10 <= s10 + 1'd1;
					else begin
						s10 <= 0;
						if (mi1 != 4'd9) mi1 <= mi1 + 1'd1;
						else begin
							mi1 <= 0;
							if (mi10 != 3'd5) mi10 <= mi10 + 1'd1;
							else begin
								mi10 <= 0;
								if (hour_end) begin
									if (mode24) begin
										h1 <= 0; h10[1:0] <= 0;
									end
									else begin
										h1 <= 4'd2; h10[1:0] <= 2'd1;   // 12
										h10[2] <= ~pm;
									end
								end
								else if (!mode24 && h10[0] && h1 == 4'd2) begin
									h1 <= 4'd1; h10[1:0] <= 0;           // 12 -> 1
								end
								else if (h1 != 4'd9) h1 <= h1 + 1'd1;
								else begin
									h1 <= 0; h10[1:0] <= h10[1:0] + 1'd1;
								end
								if (day_end) begin
									w <= (w == 3'd6) ? 3'd0 : w + 1'd1;
									if (last_day) begin
										d1 <= 4'd1; d10[1:0] <= 0;
										if (mo10 && mo1 == 4'd2) begin
											mo1 <= 4'd1; mo10 <= 0;
											if (y1 != 4'd9) y1 <= y1 + 1'd1;
											else begin
												y1 <= 0;
												y10 <= (y10 == 4'd9) ? 4'd0 : y10 + 1'd1;
											end
										end
										else if (mo1 != 4'd9) mo1 <= mo1 + 1'd1;
										else begin
											mo1 <= 0; mo10 <= 1;
										end
									end
									else if (d1 != 4'd9) d1 <= d1 + 1'd1;
									else begin
										d1 <= 0; d10[1:0] <= d10[1:0] + 1'd1;
									end
								end
							end
						end
					end
				end
			end
		end

		// Register D with WRITE high holds the top five divider stages and
		// the busy circuit in reset; the low stages keep running.
		if (cs && wr && addr == 4'hD) begin
			sub[14:10] <= 5'd0;
			busy <= 0;
		end
	end
end

endmodule

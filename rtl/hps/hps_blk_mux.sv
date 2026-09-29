// Copyright (c) 2026 Jamie Blanks
//
// Several guests share one HPS block port: a guest raises hold, waits for
// its grant, then owns the block buffer and the request lines until it
// drops hold. Lower index wins when several ask at once.

module hps_blk_mux #(parameter N = 2, parameter LBA_W = 18)
(
	input                  clk,
	input                  reset,

	input          [N-1:0] g_hold,
	output         [N-1:0] g_grant,
	input          [N-1:0] g_drive,      // which drive of the port each guest lives on
	input    [N*LBA_W-1:0] g_lba,
	input          [N-1:0] g_rd,
	input          [N-1:0] g_wr,
	output         [N-1:0] g_done,
	input        [N*9-1:0] g_buf_addr,
	input          [N-1:0] g_buf_we,
	input        [N*8-1:0] g_buf_din,

	// block port
	output                 req_drive,
	output     [LBA_W-1:0] req_lba,
	output                 req_rd,
	output                 req_wr,
	input                  busy,
	input                  done,
	output           [8:0] buf_addr,
	output                 buf_we,
	output           [7:0] buf_din
);

reg [$clog2(N)-1:0] owner;
reg                 held;

integer i;
always @(posedge clk) begin
	if (reset) begin
		held  <= 0;
		owner <= 0;
	end
	else if (!held) begin
		if (!busy && |g_hold) begin
			held <= 1;
			for (i = N - 1; i >= 0; i = i - 1) if (g_hold[i]) owner <= i[$clog2(N)-1:0];
		end
	end
	else if (!g_hold[owner]) held <= 0;
end

wire [N-1:0] sel = {{(N-1){1'b0}}, 1'b1} << owner;

assign g_grant  = held ? sel : {N{1'b0}};
assign g_done   = held ? sel & {N{done}} : {N{1'b0}};
assign req_drive = g_drive[owner];
assign req_lba  = g_lba[owner * LBA_W +: LBA_W];
// masked on the done clock: the guest drops its request one clock later
assign req_rd   = held & g_rd[owner] & ~done;
assign req_wr   = held & g_wr[owner] & ~done;
assign buf_addr = g_buf_addr[owner * 9 +: 9];
assign buf_we   = held & g_buf_we[owner];
assign buf_din  = g_buf_din[owner * 8 +: 8];

endmodule

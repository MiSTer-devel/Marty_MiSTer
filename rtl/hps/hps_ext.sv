// Copyright (c) 2026 Jamie Blanks
//
// EXT_BUS side of the Marty service in Main_MiSTer. Linux polls one word
// and acknowledges it with another:
//
//   0x70 get   cmd -> req    request word: bit 0 new floppy, 1 new IC card,
//                            2 new hard disk, 3 eject CD, 4 eject floppy,
//                            5 eject hard disk, 6 eject floppy 2,
//                            7 new floppy 2, 8 eject hard disk 2,
//                            9 new hard disk 2, 15 the disk size code
//   0x71 ack   cmd, mask     clears the request bits Linux has served
//
// Each request bit is set on the rising edge of its OSD trigger and stays
// set until acknowledged, so a trigger pressed between two polls is not
// lost and a held trigger makes one image.

module hps_ext
(
	input             clk_sys,
	inout      [35:0] EXT_BUS,
	input       [9:0] trigger,      // OSD momentary options
	input             size_code,
	output reg  [9:0] pending
);

wire [15:0] io_din    = EXT_BUS[31:16];
wire        io_strobe = EXT_BUS[33];
wire        io_enable = |EXT_BUS[35:34];

reg  [15:0] io_dout;
reg         dout_en;
assign EXT_BUS[15:0] = io_dout;
assign EXT_BUS[32]   = dout_en;

reg  [2:0] word_cnt;
reg  [7:0] cmd;
reg  [9:0] trigger_q;
wire       ack = io_enable && io_strobe && word_cnt == 3'd1 && cmd == 8'h71;

always @(posedge clk_sys) begin
	trigger_q <= trigger;
	pending   <= (pending | (trigger & ~trigger_q)) & ~(ack ? io_din[9:0] : 10'd0);

	if (~io_enable) begin
		word_cnt <= 0;
		io_dout  <= 0;
		dout_en  <= 0;
	end
	else if (io_strobe) begin
		io_dout <= 0;
		if (~&word_cnt) word_cnt <= word_cnt + 1'd1;
		case (word_cnt)
		0: begin
			cmd     <= io_din[7:0];
			dout_en <= (io_din[15:8] == 8'h00) && (io_din[7:0] == 8'h70 || io_din[7:0] == 8'h71);
			io_dout <= {size_code, 5'd0, pending};
		end
		default: ;
		endcase
	end
end

endmodule

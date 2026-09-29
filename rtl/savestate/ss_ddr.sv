// Copyright (c) 2026 Jamie Blanks
//
// Savestate DDR3 port: one 64-bit word per request on the framework's
// Avalon-style DDRAM bus. A request is a pulse on `req` with `we`, the
// word address and, for a write, the data; `done` pulses once the slave
// has taken the write or once the read data is in `rdata`. One request
// at a time.
//
//   req ──> ADDR/DIN/WE or RD held until BUSY drops ──> (read: wait for
//   DOUT_READY) ──> done
module ss_ddr
(
	input             clk,
	input             reset,

	input             req,
	input             we,
	input      [28:0] addr,          // 64-bit word address
	input      [63:0] wdata,
	input       [7:0] be,
	output reg [63:0] rdata,
	output reg        done,
	output            busy,

	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output reg [28:0] DDRAM_ADDR,
	output reg [63:0] DDRAM_DIN,
	output reg  [7:0] DDRAM_BE,
	output reg        DDRAM_WE,
	output reg        DDRAM_RD,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY
);

localparam [1:0] S_IDLE = 2'd0, S_ISSUE = 2'd1, S_READ = 2'd2;
reg [1:0] state;

assign DDRAM_BURSTCNT = 8'd1;
assign busy = state != S_IDLE;

always @(posedge clk) begin
	done <= 0;
	if (reset) begin
		state <= S_IDLE;
		DDRAM_WE <= 0;
		DDRAM_RD <= 0;
	end
	else begin
		case (state)
		S_IDLE: if (req) begin
			DDRAM_ADDR <= addr;
			DDRAM_DIN  <= wdata;
			DDRAM_BE   <= be;
			DDRAM_WE   <= we;
			DDRAM_RD   <= ~we;
			state      <= S_ISSUE;
		end
		// the slave takes the command at an edge where BUSY is low
		S_ISSUE: if (!DDRAM_BUSY) begin
			DDRAM_WE <= 0;
			DDRAM_RD <= 0;
			if (DDRAM_WE) begin done <= 1; state <= S_IDLE; end
			else state <= S_READ;
		end
		S_READ: if (DDRAM_DOUT_READY) begin
			rdata <= DDRAM_DOUT;
			done  <= 1;
			state <= S_IDLE;
		end
		default: state <= S_IDLE;
		endcase
	end
end

endmodule

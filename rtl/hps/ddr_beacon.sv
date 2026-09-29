// Copyright (c) 2026 Jamie Blanks
//
// Debug beacon: publishes eight 64-bit words in DDR3 so Linux can see the
// machine run (`mister-ddr hex`). Word 0 is {MAGIC, event count}, word 1
// {status, heartbeat}, words 2 to 7 whatever the top hands in (CPU
// position, last I/O, loader checksum, ROM read-back, CRTC registers). All are rewritten every 2^16 clocks, so a machine
// whose CPU never starts still shows its reset chain. Framework glue only;
// nothing in the machine depends on it.
module ddr_beacon #(
	parameter [28:0] ADDR  = 29'h07E00000,   // physical 0x3F000000
	parameter [31:0] MAGIC = 32'h4D415254    // "MART"
)
(
	input             clk,
	input             reset,
	input             event_pulse,
	input      [31:0] status,
	input      [63:0] word2,
	input      [63:0] word3,
	input      [63:0] word4,
	input      [63:0] word5,
	input      [63:0] word6,
	input      [63:0] word7,
	input             hold,          // start no new word while high
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output reg [28:0] DDRAM_ADDR,
	output reg [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output reg        DDRAM_WE
);

reg [31:0] count, beat;
reg [15:0] timer;
reg  [7:0] pending;          // one bit per word still to write

assign DDRAM_BURSTCNT = 8'd1;
assign DDRAM_BE       = 8'hFF;

always @(posedge clk) begin
	if (reset) begin
		count <= 0;
		beat <= 0;
		timer <= 0;
		pending <= 8'hFF;
		DDRAM_WE <= 0;
	end
	else begin
		if (event_pulse) count <= count + 1'd1;
		timer <= timer + 1'd1;
		if (&timer) begin
			beat <= beat + 1'd1;
			pending <= 8'hFF;
		end
		// Avalon write: hold WE, address and data until the slave takes
		// them at an edge where BUSY is low, then move to the next word.
		if (!DDRAM_BUSY) DDRAM_WE <= 0;
		if (pending != 0 && !DDRAM_BUSY && !hold) begin
			DDRAM_WE <= 1;
			if (pending[0]) begin
				DDRAM_ADDR <= ADDR;
				DDRAM_DIN  <= {MAGIC, count};
				pending[0] <= 0;
			end
			else if (pending[1]) begin
				DDRAM_ADDR <= ADDR + 29'd1;
				DDRAM_DIN  <= {status, beat};
				pending[1] <= 0;
			end
			else if (pending[2]) begin
				DDRAM_ADDR <= ADDR + 29'd2;
				DDRAM_DIN  <= word2;
				pending[2] <= 0;
			end
			else if (pending[3]) begin
				DDRAM_ADDR <= ADDR + 29'd3;
				DDRAM_DIN  <= word3;
				pending[3] <= 0;
			end
			else if (pending[4]) begin
				DDRAM_ADDR <= ADDR + 29'd4;
				DDRAM_DIN  <= word4;
				pending[4] <= 0;
			end
			else if (pending[5]) begin
				DDRAM_ADDR <= ADDR + 29'd5;
				DDRAM_DIN  <= word5;
				pending[5] <= 0;
			end
			else if (pending[6]) begin
				DDRAM_ADDR <= ADDR + 29'd6;
				DDRAM_DIN  <= word6;
				pending[6] <= 0;
			end
			else begin
				DDRAM_ADDR <= ADDR + 29'd7;
				DDRAM_DIN  <= word7;
				pending[7] <= 0;
			end
		end
	end
end

endmodule

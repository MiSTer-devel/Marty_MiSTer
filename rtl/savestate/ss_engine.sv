// Copyright (c) 2026 Jamie Blanks
//
// Savestate engine. A save parks the CPU at an instruction boundary,
// holds every clock enable that carries machine state, writes the image
// into the framework's DDR3 slot and restarts the CPU through the restore
// window; Main notices the bumped counter in the slot's first word and
// writes the file. A load reads the image back and restarts the same way.
//
//   request ──> ss_stop: CPU parks, bus drains ──> run low, devices held
//   ──> window <- CPU block ──> DDR3 image (CPU block, then the memories
//   word by word through the guest windows) ──> CPU reset ──> stub
//   retires into LOADALL ──> window unmapped, devices released
//
// Slot layout, 64-bit DDR3 words (Main reads the first two dwords):
//   word 0  [31:0] save counter        [63:32] image size in dwords
//   word 1  [31:0] "MRTY"              [63:32] image version
//   word 2  [31:0] {options, ram_size} [63:32] mask ROM checksum
//   word 3+ CPU state, two dwords a word, in the CPU's state-word order
//   word 32+ main DRAM (the fitted size), VRAM, sprite RAM; four bus
//            words a DDR word, low word first
//   then     the register bytes the script names, eight a DDR word
module ss_engine #(
	parameter [28:0] BASE_WORD = 29'h07000000,   // physical 0x38000000 in 64-bit words
	parameter [28:0] SLOT_WORDS = 29'h00200000,  // 16 MB per slot: 8 MB of DRAM plus the rest
	parameter [5:0]  STUB_RETIRES = 6'd22,       // instructions from the reset vector to LOADALL
	parameter SCRIPT_HEX = "rtl/savestate/ss_script.hex"   // simulation copy of the register script
)
(
	input             clk,
	input             reset,

	input             save_req,      // pulses
	input             load_req,
	input       [1:0] slot,
	input       [1:0] ram_size,
	input      [31:0] rom_sum,

	output reg        run,           // the enables outside the CPU's may advance
	output            busy,
	output reg  [1:0] event_code,    // 1 saved, 2 loaded, 3 nothing to load; pulses with event_req
	output reg        event_req,

	// mainboard
	output reg        ss_stop,
	input             ss_quiet,
	input             ss_in_hlt,
	output reg  [5:0] ss_state_sel,
	input      [31:0] ss_state,
	output reg        ss_mode,
	output reg        ss_cpu_reset,
	output reg        ss_win_we,
	output reg  [9:0] ss_win_addr,
	output reg [31:0] ss_win_data,
	input             ss_retire,
	input             ss_dev_busy,   // a device is still taking its state: the restart waits
	output reg        ss_bus_req,
	output reg        ss_bus_we,
	output reg        ss_bus_io,
	output reg        ss_bus_hidden,
	output reg        ss_bus_raw,
	output reg [23:1] ss_bus_a,
	output      [1:0] ss_bus_be,
	output     [15:0] ss_bus_din,
	input      [15:0] ss_bus_dout,
	input             ss_bus_ack,

	// DDR3 port
	output reg        ddr_req,
	output reg        ddr_we,
	output reg [28:0] ddr_addr,
	output reg [63:0] ddr_wdata,
	input      [63:0] ddr_rdata,
	input             ddr_done
);

localparam [31:0] MAGIC   = 32'h5954524D;   // "MRTY" in memory order
localparam [31:0] VERSION = 32'd2;
localparam [5:0]  CPU_WORDS = 6'd29;         // 58 state dwords
localparam [28:0] BULK_WORD = 29'd32;        // first DDR word of the memories

// memory sections: bus word address, length in 16-bit words, raw DRAM flag
localparam [22:0] DRAM_WORDS_2M = 23'h100000;
localparam [22:0] VRAM_WORDS = 23'h040000, SPR_WORDS = 23'h010000;
wire [22:0] dram_words = ram_size == 2'd0 ? DRAM_WORDS_2M : ram_size == 2'd1 ? 2 * DRAM_WORDS_2M :
                        ram_size == 2'd2 ? 3 * DRAM_WORDS_2M : 4 * DRAM_WORDS_2M;
wire [31:0] IMAGE_DWORDS = 32'd4 + 32'd58 + {10'd0, dram_words[22:1]} + {10'd0, VRAM_WORDS[22:1]} + {10'd0, SPR_WORDS[22:1]} + {2'd0, REG_BYTES[31:2]};

`include "rtl/savestate/ss_script_size.svh"

localparam [4:0] S_IDLE = 5'd0, S_STOP = 5'd1, S_SETTLE = 5'd2, S_CPU_LO = 5'd3, S_CPU_HI = 5'd4,
                 S_CPU_WIN = 5'd5, S_CPU_WR = 5'd6, S_RD_CNT = 5'd7, S_WR_HDR = 5'd8, S_WR_CNT = 5'd9,
                 S_RD_MAGIC = 5'd10, S_RD_INFO = 5'd11, S_RESTORE = 5'd12, S_WIN_HI = 5'd13,
                 S_STUB = 5'd14, S_RESUME = 5'd15,
                 S_BULK_START = 5'd16, S_BULK_RD = 5'd17, S_BULK_DDR = 5'd18,
                 S_BULK_LD = 5'd19, S_BULK_WR = 5'd20,
                 S_REG_FETCH = 5'd21, S_REG_ENTRY = 5'd22, S_REG_CYC = 5'd23, S_REG_DDR = 5'd24,
                 S_REG_LD = 5'd25, S_REG_DONE = 5'd26;
reg  [4:0] state;
reg  [1:0] sec;           // memory section
reg [22:0] left;          // bus words left in the section
reg  [1:0] part;          // bus word inside the DDR word
reg [63:0] pack;
reg [28:0] bulk_addr;
reg  [2:0] nbyte;         // register bytes packed into the DDR word
reg  [7:0] script_idx;
reg [13:0] cyc_left;
reg [15:0] io_a;
reg        io_hidden, io_fixed;
reg        entry_last;    // the cycle just done was the entry's last

// the register script: one entry per run of I/O bytes, terminated by all ones
wire [31:0] entry;
wire        entry_end = &entry;
cache_ram #(.ADDR_WIDTH(8), .DATA_WIDTH(32), .MEM_INIT_FILE("ss_script.mif"), .SIM_INIT_FILE(SCRIPT_HEX)) script
(
	.clk_i(clk), .addr_i(script_idx), .wren_i(1'b0), .wdata_i(32'd0), .q_o(entry)
);
reg        loading;
reg [23:0] timer;
reg  [1:0] word;
reg  [5:0] k;             // CPU block word
reg  [2:0] sample;
reg [31:0] count, lo, hi;
reg  [5:0] retires;
wire [28:0] slot_base = BASE_WORD + ({27'd0, slot} * SLOT_WORDS);
wire [31:0] state_fixed = (ss_state_sel == 6'd2 && ss_in_hlt) ? ss_state - 1'd1 : ss_state;   // EIP back onto the HLT

assign busy = state != S_IDLE;
assign ss_bus_din = ss_bus_io ? {pack[7:0], pack[7:0]} : pack[15:0];
assign ss_bus_be  = ss_bus_io ? {io_a[0], ~io_a[0]} : 2'b11;

always @(posedge clk) begin
	ddr_req      <= 0;
	event_req    <= 0;
	ss_cpu_reset <= 0;
	ss_win_we    <= 0;
	if (reset) begin
		state      <= S_IDLE;
		run        <= 1;
		ss_stop    <= 0;
		ss_mode    <= 0;
		ss_bus_req <= 0;
		ss_bus_io  <= 0;
		ss_bus_hidden <= 0;
	end
	else begin
		case (state)
		S_IDLE: if (save_req || load_req) begin
			loading <= load_req;
			ss_stop <= 1;
			timer   <= 24'hFFFFFF;
			state   <= S_STOP;
		end
		// the CPU parks at its next boundary and the bus drains; a stuck
		// machine is taken as it is once the timer runs out
		S_STOP: begin
			timer <= timer - 1'd1;
			if (ss_quiet || timer == 0) begin
				run     <= 0;
				ss_mode <= 1;
				timer   <= 24'd255;
				state   <= S_SETTLE;
			end
		end
		S_SETTLE: begin
			timer <= timer - 1'd1;
			if (timer == 0) begin
				k            <= 0;
				ss_state_sel <= 0;
				sample       <= 3'd3;
				state        <= S_CPU_LO;
			end
		end
		// ---- the live CPU state goes into the window, and into the image for a save ----
		// two state words a DDR word; the mux settles for a few clocks
		S_CPU_LO: begin
			sample <= sample - 1'd1;
			if (sample == 0) begin
				lo           <= state_fixed;
				ss_state_sel <= ss_state_sel + 1'd1;
				sample       <= 3'd3;
				state        <= S_CPU_HI;
			end
		end
		S_CPU_HI: begin
			sample <= sample - 1'd1;
			if (sample == 0) begin
				hi           <= state_fixed;
				ss_state_sel <= ss_state_sel + 1'd1;
				ss_win_we    <= 1;
				ss_win_addr  <= {3'd0, k, 1'b0};
				ss_win_data  <= lo;
				state        <= S_CPU_WIN;
			end
		end
		S_CPU_WIN: begin
			ss_win_we   <= 1;
			ss_win_addr <= {3'd0, k, 1'b1};
			ss_win_data <= hi;
			if (!loading) begin
				ddr_req   <= 1;
				ddr_we    <= 1;
				ddr_addr  <= slot_base + 29'd3 + {23'd0, k};
				ddr_wdata <= {hi, lo};
			end
			state <= S_CPU_WR;
		end
		S_CPU_WR: if (loading || ddr_done) begin
			k      <= k + 1'd1;
			sample <= 3'd3;
			state  <= S_CPU_LO;
			if (k == CPU_WORDS - 1'd1) begin
				ddr_req  <= 1;
				ddr_we   <= 0;
				ddr_addr <= loading ? slot_base + 29'd1 : slot_base;
				state    <= loading ? S_RD_MAGIC : S_RD_CNT;
			end
		end
		// ---- the memories, one bus word at a time ----
		S_BULK_START: begin
			part       <= 0;
			bulk_addr  <= sec == 2'd0 ? slot_base + BULK_WORD : bulk_addr;
			ss_bus_raw <= sec == 2'd0;
			ss_bus_a   <= sec == 2'd0 ? 23'h000000 : sec == 2'd1 ? 23'h500000 : 23'h600000;
			left       <= sec == 2'd0 ? dram_words : sec == 2'd1 ? VRAM_WORDS : SPR_WORDS;
			if (sec == 2'd3) begin
				// every section done: the register script follows
				script_idx <= 0;
				nbyte      <= 0;
				ss_bus_raw <= 0;
				state      <= S_REG_FETCH;
			end
			else if (loading) begin
				ddr_req  <= 1;
				ddr_we   <= 0;
				// bulk_addr only takes the section's base next clock
				ddr_addr <= sec == 2'd0 ? slot_base + BULK_WORD : bulk_addr;
				state    <= S_BULK_LD;
			end
			else begin
				ss_bus_req <= 1;
				ss_bus_we  <= 0;
				state      <= S_BULK_RD;
			end
		end
		S_BULK_RD: if (ss_bus_ack) begin
			ss_bus_req <= 0;
			pack       <= {ss_bus_dout, pack[63:16]};
			ss_bus_a   <= ss_bus_a + 1'd1;
			left       <= left - 1'd1;
			part       <= part + 1'd1;
			if (part == 2'd3) begin
				ddr_req   <= 1;
				ddr_we    <= 1;
				ddr_addr  <= bulk_addr;
				ddr_wdata <= {ss_bus_dout, pack[63:16]};
				state     <= S_BULK_DDR;
			end
			else begin
				ss_bus_req <= 1;
			end
		end
		S_BULK_DDR: if (ddr_done) begin
			bulk_addr <= bulk_addr + 1'd1;
			if (left == 0) begin
				sec   <= sec + 1'd1;
				state <= S_BULK_START;
			end
			else begin
				ss_bus_req <= 1;
				state      <= S_BULK_RD;
			end
		end
		// ---- the register bytes, one I/O cycle each ----
		S_REG_FETCH: begin
			sample <= 3'd1;
			state  <= S_REG_ENTRY;
		end
		S_REG_ENTRY: if (sample != 0) sample <= sample - 1'd1;
		else begin
			script_idx <= script_idx + 1'd1;
			cyc_left   <= entry[29:16];
			io_a       <= entry[15:0];
			io_hidden  <= entry[31];
			io_fixed   <= entry[30];
			if (entry_end) state <= S_REG_DONE;
			else if (loading && nbyte == 0) begin
				ddr_req  <= 1;
				ddr_we   <= 0;
				ddr_addr <= bulk_addr;
				state    <= S_REG_LD;
			end
			else begin
				ss_bus_req    <= 1;
				ss_bus_we     <= loading;
				ss_bus_io     <= 1;
				ss_bus_hidden <= entry[31];
				ss_bus_a      <= {8'd0, entry[15:1]};
				state         <= S_REG_CYC;
			end
		end
		S_REG_LD: if (ddr_done) begin
			pack          <= ddr_rdata;
			ss_bus_req    <= 1;
			ss_bus_we     <= 1;
			ss_bus_io     <= 1;
			ss_bus_hidden <= io_hidden;
			ss_bus_a      <= {8'd0, io_a[15:1]};
			state         <= S_REG_CYC;
		end
		S_REG_CYC: if (ss_bus_ack) begin
			ss_bus_req <= 0;
			// low byte first into the word for a save; a load shifts the word down
			pack     <= loading ? {8'd0, pack[63:8]} : {(io_a[0] ? ss_bus_dout[15:8] : ss_bus_dout[7:0]), pack[63:8]};
			nbyte      <= nbyte + 1'd1;
			if (!io_fixed) io_a <= io_a + 1'd1;
			cyc_left   <= cyc_left - 1'd1;
			entry_last <= cyc_left == 0;
			if (!loading && nbyte == 3'd7) begin
				ddr_req   <= 1;
				ddr_we    <= 1;
				ddr_addr  <= bulk_addr;
				ddr_wdata <= {(io_a[0] ? ss_bus_dout[15:8] : ss_bus_dout[7:0]), pack[63:8]};
				state     <= S_REG_DDR;
			end
			else begin
				if (loading && nbyte == 3'd7) bulk_addr <= bulk_addr + 1'd1;
				if (cyc_left == 0) state <= S_REG_FETCH;
				else if (loading && nbyte == 3'd7) begin
					ddr_req  <= 1;
					ddr_we   <= 0;
					ddr_addr <= bulk_addr + 1'd1;
					state    <= S_REG_LD;
				end
				else begin
					ss_bus_req <= 1;
					ss_bus_a   <= {8'd0, io_a[15:1] + {14'd0, io_a[0] & ~io_fixed}};
					state      <= S_REG_CYC;
				end
			end
		end
		S_REG_DDR: if (ddr_done) begin
			bulk_addr <= bulk_addr + 1'd1;
			if (entry_last) state <= S_REG_FETCH;
			else begin
				ss_bus_req <= 1;
				ss_bus_a   <= {8'd0, io_a[15:1]};
				state      <= S_REG_CYC;
			end
		end
		S_REG_DONE: begin
			ss_bus_io     <= 0;
			ss_bus_hidden <= 0;
			// a save writes the counter, a load restarts the CPU once every device has its state
			if (loading && ss_dev_busy) state <= S_REG_DONE;
			else if (loading) begin
				k        <= 0;
				ddr_req  <= 1;
				ddr_we   <= 0;
				ddr_addr <= slot_base + 29'd3;
				state    <= S_RESTORE;
			end
			else begin
				ddr_req   <= 1;
				ddr_we    <= 1;
				ddr_addr  <= slot_base;
				ddr_wdata <= {IMAGE_DWORDS, count};
				state     <= S_WR_CNT;
			end
		end
		S_BULK_LD: if (ddr_done) begin
			pack       <= ddr_rdata;
			ss_bus_req <= 1;
			ss_bus_we  <= 1;
			state      <= S_BULK_WR;
		end
		S_BULK_WR: if (ss_bus_ack) begin
			pack     <= {16'd0, pack[63:16]};
			ss_bus_a <= ss_bus_a + 1'd1;
			left     <= left - 1'd1;
			part     <= part + 1'd1;
			if (part == 2'd3) begin
				ss_bus_req <= 0;
				bulk_addr  <= bulk_addr + 1'd1;
				if (left == 1) begin
					sec   <= sec + 1'd1;
					state <= S_BULK_START;
				end
				else begin
					ddr_req  <= 1;
					ddr_we   <= 0;
					ddr_addr <= bulk_addr + 1'd1;
					state    <= S_BULK_LD;
				end
			end
		end
		// ---- save: header, then the counter Main watches ----
		S_RD_CNT: if (ddr_done) begin
			count    <= ddr_rdata[31:0] + 1'd1;
			word     <= 2'd1;
			ddr_req  <= 1;
			ddr_we   <= 1;
			ddr_addr <= slot_base + 29'd1;
			ddr_wdata <= {VERSION, MAGIC};
			state    <= S_WR_HDR;
		end
		S_WR_HDR: if (ddr_done) begin
			ddr_req <= 1;
			ddr_we  <= 1;
			if (word == 2'd1) begin
				word      <= 2'd2;
				ddr_addr  <= slot_base + 29'd2;
				ddr_wdata <= {rom_sum, 30'd0, ram_size};
			end
			else begin
				ddr_req <= 0;
				sec     <= 0;
				state   <= S_BULK_START;
			end
		end
		S_WR_CNT: if (ddr_done) begin
			event_code <= 2'd1;
			state      <= S_STUB;
			ss_cpu_reset <= 1;
			retires    <= 0;
			timer      <= 24'hFFFFF;
		end
		// ---- load: check the header, then the CPU block replaces the window ----
		S_RD_MAGIC: if (ddr_done) begin
			if (ddr_rdata == {VERSION, MAGIC}) begin
				ddr_req  <= 1;
				ddr_we   <= 0;
				ddr_addr <= slot_base + 29'd2;
				state    <= S_RD_INFO;
			end
			else begin
				event_code   <= 2'd3;
				state        <= S_STUB;
				ss_cpu_reset <= 1;
				retires      <= 0;
				timer        <= 24'hFFFFF;
			end
		end
		S_RD_INFO: if (ddr_done) begin
			if (ddr_rdata[63:32] == rom_sum && ddr_rdata[1:0] == ram_size) begin
				event_code <= 2'd2;
				sec        <= 0;
				state      <= S_BULK_START;
			end
			else begin
				event_code   <= 2'd3;
				state        <= S_STUB;
				ss_cpu_reset <= 1;
				retires      <= 0;
				timer        <= 24'hFFFFF;
			end
		end
		S_RESTORE: if (ddr_done) begin
			ss_win_we   <= 1;
			ss_win_addr <= {3'd0, k, 1'b0};
			ss_win_data <= ddr_rdata[31:0];
			lo          <= ddr_rdata[63:32];
			state       <= S_WIN_HI;
		end
		S_WIN_HI: begin
			ss_win_we   <= 1;
			ss_win_addr <= {3'd0, k, 1'b1};
			ss_win_data <= lo;
			k           <= k + 1'd1;
			if (k == CPU_WORDS - 1'd1) begin
				ss_cpu_reset <= 1;
				retires      <= 0;
				timer        <= 24'hFFFFF;
				state        <= S_STUB;
			end
			else begin
				ddr_req  <= 1;
				ddr_we   <= 0;
				ddr_addr <= slot_base + 29'd4 + {23'd0, k};
				state    <= S_RESTORE;
			end
		end
		// ---- the CPU alone runs the stub into LOADALL, then the machine goes on ----
		S_STUB: begin
			ss_stop <= 0;
			run     <= 1;
			timer   <= timer - 1'd1;
			if (ss_retire) retires <= retires + 1'd1;
			if ((ss_retire && retires == STUB_RETIRES - 1'd1) || timer == 0) begin
				if (timer == 0) event_code <= 2'd3;
				state <= S_RESUME;
			end
		end
		S_RESUME: begin
			ss_mode   <= 0;
			event_req <= 1;
			state     <= S_IDLE;
		end
		default: state <= S_IDLE;
		endcase
	end
end

endmodule

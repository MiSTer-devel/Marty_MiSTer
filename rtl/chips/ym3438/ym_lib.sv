// ym3438, ym7101, fc1004 common cells

module ym_sr_bit #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);
	
	reg [SR_LENGTH-1:0] v1 = 0;
	reg [SR_LENGTH-1:0] v2 = 0;
	
	wire [SR_LENGTH-1:0] v2_assign = c2 ? v1 : v2;
	
	//assign sr_out = v2_assign[SR_LENGTH-1];
	assign sr_out = v2[SR_LENGTH-1];
	
	always @(posedge MCLK)
	begin
		if (c1)
		begin
			if (SR_LENGTH == 1)
				v1 <= bit_in;
			else
				v1 <= { v2[SR_LENGTH-2:0], bit_in };
		end
		v2 <= v2_assign;
	end
endmodule

/*module ym_sr_bit #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);
	
	reg [SR_LENGTH-1:0] v1 = 0;
	reg [SR_LENGTH-1:0] v2 = 0;
	
	assign sr_out = v2[SR_LENGTH-1];
	
	always @(*)
	begin
		if (c1)
		begin
			if (SR_LENGTH == 1)
				v1 <= bit_in;
			else
				v1 <= { v2[SR_LENGTH-2:0], bit_in };
		end
		if (c2)
		begin
			v2 <= v1;
		end
	end
endmodule*/

/*module ym_sr_bit2 #(parameter SR_LENGTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input bit_in,
	output sr_out
	);
	
	reg [SR_LENGTH-1:0] v2 = 0;
	
	assign sr_out = v2[SR_LENGTH-1];
	
	always @(posedge c2)
	begin
		if (SR_LENGTH == 1)
			v2 <= bit_in;
		else
			v2 <= { v2[SR_LENGTH-2:0], bit_in };
	end
endmodule*/

module ym_sr_bit_array #(parameter SR_LENGTH = 1, DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input [DATA_WIDTH-1:0] data_in,
	output [DATA_WIDTH-1:0] data_out
	);
	
	// Long loops keep their stages in a block RAM ring instead of two flops
	// a stage; short ones stay as flop chains.
	localparam RING = DATA_WIDTH * (SR_LENGTH - 1) >= 40;
	
	generate
		if (RING)
		begin : ring
			ym_sr_ring #(.SR_LENGTH(SR_LENGTH), .DATA_WIDTH(DATA_WIDTH)) mem (
			.MCLK(MCLK),
			.c1(c1),
			.c2(c2),
			.data_in(data_in),
			.data_out(data_out)
			);
		end
		else
		begin : chain
			wire out[0:DATA_WIDTH-1];
			
			genvar i;
			for (i = 0; i < DATA_WIDTH; i = i + 1)
			begin : l1
				ym_sr_bit #(.SR_LENGTH(SR_LENGTH)) sr (
				.MCLK(MCLK),
				.c1(c1),
				.c2(c2),
				.bit_in(data_in[i]),
				.sr_out(out[i])
				);
				
				assign data_out[i] = out[i];
			end
		end
	endgenerate

endmodule

// A shift loop of SR_LENGTH stages held as a ring of SR_LENGTH-1 words plus
// the output register. c1 and c2 are the chip's two clock phases, each high
// for several MCLK and never together. The input is tracked while c1 is high
// (the master latch); the first MCLK of the following c2 phase moves the ring
// one step (the slave latch). Repeated phases without the other in between
// change nothing, as with the flop chain.
//
//   c1: in_q <= data_in          c2 (first MCLK after a c1):
//                                    out <= q; mem[p] <= in_q; p++
//
// q is the word read at p on the previous MCLK, so the step takes the old
// word before it writes the slot; the RAM's own read-during-write result
// is never used. p settles at least three MCLK before the step.
module ym_sr_ring #(parameter SR_LENGTH = 2, DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input [DATA_WIDTH-1:0] data_in,
	output [DATA_WIDTH-1:0] data_out
	);
	
	localparam DEPTH = SR_LENGTH - 1;
	localparam AW = DEPTH <= 2 ? 1 : $clog2(DEPTH);
	
	(* ramstyle = "M10K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
	reg [DATA_WIDTH-1:0] in_q = 0;
	reg [DATA_WIDTH-1:0] out = 0;
	reg [DATA_WIDTH-1:0] q;
	reg [AW-1:0] p = 0;
	reg armed = 0;
	
	integer k;
	initial for (k = 0; k < DEPTH; k = k + 1) mem[k] = 0;
	
	wire step = c2 & armed;
	
	always @(posedge MCLK)
	begin
		if (c1)
		begin
			in_q <= data_in;
			armed <= 1'b1;
		end
		if (step)
		begin
			mem[p] <= in_q;
			out <= q;
			p <= (p == DEPTH - 1) ? {AW{1'b0}} : p + 1'b1;
			armed <= 1'b0;
		end
		q <= mem[p];
	end
	
	assign data_out = out;
	
endmodule

module ym_cnt_bit #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

/*module ym_cnt_bit2
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	output val,
	output c_out
	);
	
	wire data_in;
	wire data_out;
	wire [1:0] sum;
	
	ym_sr_bit2 mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.bit_in(data_in),
		.sr_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {1'h0, c_in};
	assign val = data_out;
	assign data_in = reset ? 1'h0 : sum[0];
	assign c_out = sum[1];
	
endmodule*/

module ym_dlatch_1 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = c1 ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule

/*module ym_dlatch_1 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (c1)
			mem <= inp;
	end

	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

module ym_dlatch_2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c2,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = c2 ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule

/*module ym_dlatch_2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c2,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (c2)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

module ym_edge_detect
	(
	input MCLK,
	input c1,
	input inp,
	output outp
	);
	
	wire prev_out;
	
	ym_dlatch_1 prev
		(
		.MCLK(MCLK),
		.c1(c1),
		.inp(inp),
		.val(prev_out),
		.nval()
		);
	assign outp = ~(prev_out | ~inp);
endmodule

/*module ym_edge_detect
	(
	input MCLK,
	input c1,
	input inp,
	output outp
	);
	
	reg prev_out;
	
	always @(posedge c1)
	begin
		prev_out <= inp;
	end
	
	assign outp = ~(prev_out | ~inp);
endmodule*/

module ym_slatch #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule

/*module ym_slatch #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (en)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

/*module ym_slatch2 #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

module ym_slatch_t #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = en ? inp : mem;
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	assign val = mem_assign;
	assign nval = ~mem_assign;
	
endmodule

/*module ym_slatch_t #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (en)
			mem <= inp;
	end
	
	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

module ym_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output reg q = 1'h0,
	output reg nq = 1'h1
	);
	
	always @(posedge MCLK)
	begin
		q <= rst ? 1'h0 : (set ? 1'h1 : q);
		nq <= set ? 1'h0 : (rst ? 1'h1 : ~q); 
	end
	
endmodule

/*module ym_rs_trig
	(
	input MCLK,
	input set,
	input rst,
	output q,
	output nq
	);
	
	assign q = ~(rst | nq);
	assign nq = ~(set | q);
	
endmodule*/

module ym_rs_trig_sync
	(
	input MCLK,
	input set,
	input rst,
	input c1,
	output reg q = 1'h0,
	output reg nq = 1'h1
	);
	
	always @(posedge MCLK)
	begin
		q <= (c1 & rst) ? 1'h0 : ((c1 & set) ? 1'h1 : q);
		nq <= (c1 & set) ? 1'h0 : ((c1 & rst) ? 1'h1 : ~q); 
	end
	
endmodule

/*module ym_rs_trig_sync
	(
	input MCLK,
	input set,
	input rst,
	input c1,
	output q,
	output nq
	);
	
	assign q = ~((c1 & rst) | nq);
	assign nq = ~((c1 & set) | q);
	
endmodule*/

module ym_cnt_bit_load #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	wire [DATA_WIDTH-1:0] base_val = load ? load_val : data_out;
	
	assign sum = {1'h0, base_val} + {{DATA_WIDTH{1'h0}},c_in};
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign val = data_out;
	assign c_out = sum[DATA_WIDTH];
	
endmodule

module ym_dbg_read #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input prev,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output next
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
		
	wire [DATA_WIDTH-1:0] chain;
	
	assign data_in = chain | (load ? load_val : {DATA_WIDTH{1'h0}});
	
	generate
		if (DATA_WIDTH == 1)
			assign chain = prev;
		else
			assign chain = { prev, data_out[DATA_WIDTH-1:1] };
	endgenerate
	
	assign next = data_out[0];
	
endmodule

module ym_dbg_read_eg #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input prev,
	input load,
	input [DATA_WIDTH-1:0] load_val,
	output next
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
		
	wire [DATA_WIDTH-1:0] chain;
	
	assign data_in = chain | (load ? load_val : {DATA_WIDTH{1'h0}});
	
	generate
		if (DATA_WIDTH == 1)
			assign chain = prev;
		else
			assign chain = { data_out[DATA_WIDTH-2:0], prev };
	endgenerate
	
	assign next = data_out[DATA_WIDTH-1];
	
endmodule

module ym_slatch_r #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input rst,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH-1:0] mem_assign = rst ? {DATA_WIDTH{1'h0}} : (en ? inp : mem);
	
	always @(posedge MCLK)
	begin
		mem <= mem_assign;
	end
	
	//assign val = mem_assign;
	//assign nval = ~mem_assign;
	assign val = mem;
	assign nval = ~mem;
	
endmodule

/*module ym_slatch_r #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input en,
	input rst,
	input [DATA_WIDTH-1:0] inp,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval
	);
	
	reg [DATA_WIDTH-1:0] mem = {DATA_WIDTH{1'h0}};
	
	always @(*)
	begin
		if (rst)
			mem <= {DATA_WIDTH{1'h0}};
		else if (en)
			mem <= inp;
	end

	assign val = mem;
	assign nval = ~mem;
	
endmodule*/

module ym_cnt_bit_rs #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input reset,
	input set,
	output [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] nval,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH-1:0] data_out_s = set ? {DATA_WIDTH{1'h1}} : data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = {1'h0,data_out_s} + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out_s;
	assign nval = ~data_out_s;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

module ym_cnt_bit_rev #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input c1,
	input c2,
	input c_in,
	input dec,
	input reset,
	output [DATA_WIDTH-1:0] val,
	output c_out
	);
	
	wire [DATA_WIDTH-1:0] data_in;
	wire [DATA_WIDTH-1:0] data_out;
	wire [DATA_WIDTH:0] sum;
	
	ym_sr_bit_array #(.DATA_WIDTH(DATA_WIDTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(data_in),
		.data_out(data_out)
		);
	
	assign sum = { 1'h0, data_out } + {1'h0, {DATA_WIDTH{dec}}} + {{DATA_WIDTH{1'h0}}, c_in};
	assign val = data_out;
	assign data_in = reset ? {DATA_WIDTH{1'h0}} : sum[DATA_WIDTH-1:0];
	assign c_out = sum[DATA_WIDTH];
	
endmodule

module ym_sr_bit_en #(parameter SR_LENGTH = 2)
	(
	input MCLK,
	input c1,
	input c2,
	input en1,
	input en2,
	input data_in,
	output [SR_LENGTH-1:0] data_out
	);
	
	wire [SR_LENGTH-1:0] sr_out;
	wire [SR_LENGTH-1:0] sr_in =
		(en1 ? { sr_out[SR_LENGTH-2:0], data_in } : {SR_LENGTH{1'h0}}) |
		(en2 ? sr_out : {SR_LENGTH{1'h0}});
	
	assign data_out = sr_out;
	
	ym_sr_bit_array #(.DATA_WIDTH(SR_LENGTH)) mem
		(
		.MCLK(MCLK),
		.c1(c1),
		.c2(c2),
		.data_in(sr_in),
		.data_out(sr_out)
		);

endmodule


module ym_scnt_bit #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input load,
	input [DATA_WIDTH-1:0] val,
	input cin,
	input rst,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq,
	output cout
	);
	
	reg [DATA_WIDTH-1:0] l1 = {DATA_WIDTH{1'h0}}, l2 = {DATA_WIDTH{1'h0}};
	
	wire [DATA_WIDTH:0] sum = { 1'h0, l2 } + {{DATA_WIDTH{1'h0}}, cin};
	
	assign cout = sum[DATA_WIDTH];
	
	assign q = l2;
	assign nq = ~l2;
	
	always @(posedge MCLK)
	begin
		if (~rst)
		begin
			l1 <= {DATA_WIDTH{1'h0}};
			l2 <= {DATA_WIDTH{1'h0}};
		end
		else
		begin
			if (~clk)
				l1 <= ~load ? val : sum[DATA_WIDTH-1:0];
			else
				l2 <= l1;
		end
	end
	
endmodule


module ym_sdff #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);
	
	reg [DATA_WIDTH-1:0] l1 = {DATA_WIDTH{1'h0}}, l2 = {DATA_WIDTH{1'h0}};
	
	assign q = l2;
	assign nq = ~l2;
	
	always @(posedge MCLK)
	begin
		if (~clk)
			l1 <= val;
		else
			l2 <= l1;
	end
	
endmodule


module ym_sdffs #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input set,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);
	
	reg [DATA_WIDTH-1:0] l1, l2;
	
	assign q = l2;
	assign nq = ~l2;
	
	always @(posedge MCLK)
	begin
		if (~clk)
			l1 <= val;
		else if (~set)
			l1 <= {DATA_WIDTH{1'h1}};
		if (~set)
			l2 <= {DATA_WIDTH{1'h1}};
		else if (clk)
			l2 <= l1;
	end
	
endmodule


module ym_sdffr #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input reset,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);
	
	reg [DATA_WIDTH-1:0] l1 = {DATA_WIDTH{1'h0}}, l2 = {DATA_WIDTH{1'h0}};
	
	assign q = l2;
	assign nq = ~l2;
	
	always @(posedge MCLK)
	begin
		if (~reset)
			l1 <= {DATA_WIDTH{1'h0}};
		else if (~clk)
			l1 <= val;
		if (~reset)
			l2 <= {DATA_WIDTH{1'h0}};
		else if (clk)
			l2 <= l1;
	end
	
endmodule


module ym_sdffsr #(parameter DATA_WIDTH = 1)
	(
	input MCLK,
	input clk,
	input [DATA_WIDTH-1:0] val,
	input set,
	input reset,
	output [DATA_WIDTH-1:0] q,
	output [DATA_WIDTH-1:0] nq
	);
	
	reg [DATA_WIDTH-1:0] l1 = {DATA_WIDTH{1'h0}}, l2 = {DATA_WIDTH{1'h0}};
	
	assign q = (~set & ~reset) ? {DATA_WIDTH{1'h0}} : l2;
	assign nq = (~set & ~reset) ? {DATA_WIDTH{1'h0}} : ~l2;
	
	always @(posedge MCLK)
	begin
		if (~reset)
			l1 <= {DATA_WIDTH{1'h0}};
		else if (~set)
			l1 <= {DATA_WIDTH{1'h1}};
		else if (~clk)
			l1 <= val;
		if (~set)
			l2 <= {DATA_WIDTH{1'h1}};
		else if (~reset)
			l2 <= {DATA_WIDTH{1'h0}};
		else if (clk)
			l2 <= l1;
	end
	
endmodule


module ym_delaychain #(parameter DELAY_CNT = 1)
	(
	input MCLK,
	input inp,
	output outp
	);
	
	reg [DELAY_CNT-1:0] dl = {DELAY_CNT{1'h0}};
	
	always @(posedge MCLK)
	begin
		if (DELAY_CNT == 1)
			dl <= inp;
		else
			dl <= { dl[DELAY_CNT-2:0], inp };
	end
	
	assign outp = dl[DELAY_CNT-1];
	
endmodule

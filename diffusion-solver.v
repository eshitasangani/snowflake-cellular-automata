// //in the top-level module ///////////////////
`timescale 1ns/1ns

module testbench();

reg clk, reset;


//Initialize clocks and index
initial begin
    clk = 1'b0;
end

//Toggle the clocks
always begin
    #10
    clk  = !clk;
end

//Intialize and drive signals
initial begin
    reset  = 1'b0;
    #10 
    reset  = 1'b1;
    #30
    reset  = 1'b0;
end

wire signed [17:0] u_neighbor_0;
wire signed [17:0] u_neighbor_1;
wire signed [17:0] u_neighbor_2;
wire signed [17:0] u_neighbor_3;
wire signed [17:0] u_neighbor_4;
wire signed [17:0] u_neighbor_5;

assign u_neighbor_0 = 18'b00_0000000000010000;
assign u_neighbor_1 = 18'b00_0001000000000000;
assign u_neighbor_2 = 18'b00_0000001000000000;
assign u_neighbor_3 = 18'b00_0000010000000000;
assign u_neighbor_4 = 18'b00_0000010000000000;
assign u_neighbor_5 = 18'b00_1000000000000000;

wire signed [17:0] alpha;
assign alpha = 18'b01_0000000000000000;

wire signed [17:0] beta;
assign beta = 18'b00_0100000000000000;

wire is_frozen;
wire signed [17:0] u_next;
reg  signed [17:0] u_curr;

always @(posedge clk) begin
    if (reset) begin
        u_curr <= beta;
    end
    else begin
        u_curr <= u_next;
    end
end



diffusion_solver solver_inst (
    .u_neighbor_0 (u_neighbor_0),
    .u_neighbor_1 (u_neighbor_1),
    .u_neighbor_2 (u_neighbor_2),
    .u_neighbor_3 (u_neighbor_3),
    .u_neighbor_4 (u_neighbor_4),
    .u_neighbor_5 (u_neighbor_5),
    .u_curr       (u_curr),
    .v_next       (18'd0),
    .alpha        (alpha),
    .beta         (beta),
      
    .u_next       (u_next),
    .is_frozen    (is_frozen)
);

endmodule

/*
neighbor indexing: 
        - 0 1 
        2 x 3
        4 5 -
*/

/*
    this module calculates the diffusion equation (u) of one cell 
    it also outputs whether this cell is frozen at the end of calculating the diffusion
    u_next = u_curr + alpha / 2 * (u_avg - u_curr)
    u_avg = avg u over all neighbors
*/
module diffusion_solver (
    input  wire signed [17:0] u_neighbor_0,
    input  wire signed [17:0] u_neighbor_1,
    input  wire signed [17:0] u_neighbor_2,
    input  wire signed [17:0] u_neighbor_3,
    input  wire signed [17:0] u_neighbor_4,
    input  wire signed [17:0] u_neighbor_5,
    input  wire signed [17:0] u_curr,
    input  wire signed [17:0] v_next, // this is calculated outside of this module, when we calculate our current u and v

    input  wire signed [17:0] alpha,
    input  wire signed [17:0] beta,

    output wire signed [17:0] u_next,

    output wire        is_frozen
);

    wire signed [17:0] u_avg;
    wire signed [17:0] laplace_out;

    // s = u + v
    // frozen if s >= 1
    assign is_frozen = ((u_next + v_next) >= 18'b01_0000000000000000);
    assign u_next = u_curr + laplace_out;

    signed_mult u_avg_calc ( // divide by 6 (num neighbors) -- mult by 1/6
        .out(u_avg),
        .a  (u_neighbor_0+u_neighbor_1+u_neighbor_2+u_neighbor_3+u_neighbor_4+u_neighbor_5),
        .b  (18'b00_0010101010101010)
    );

    signed_mult laplace_calc ( // alpha / 2 * (u_avg - cell.u)
        .out(laplace_out),
        .a  (alpha >> 1),
        .b  (u_avg - u_curr)
    );

endmodule


//////////////////////////////////////////////////
//// signed mult of 2.16 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b);
	output 	signed  [17:0]	out;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;
	// intermediate full bit length
	wire 	signed	[35:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[35], mult_out[34:16]};
endmodule
//////////////////////////////////////////////////
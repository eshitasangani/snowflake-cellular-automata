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


reg  [18:0] write_addr_u_curr;
reg  [18:0] write_addr_v_next;

reg  [18:0] read_addr_u_curr;
reg  [18:0] read_addr_v_next;

reg         write_en_u_curr;
reg         write_en_v_next;

reg  signed [17:0] write_data_u_curr;
reg  signed [17:0] write_data_v_next;

wire signed [17:0] read_data_u_curr;
wire signed [17:0] read_data_v_next;

// inputs to diffusion solver

reg signed [17:0] v_next;
reg signed [17:0] v_next_reg; // store curr node v_next so we can free solver to calc diffuion of cell above us
reg signed [17:0] u_curr;
reg signed [17:0] u_neighbor_t;
reg signed [17:0] u_neighbor_b;
reg signed [17:0] u_neighbor_l;
reg signed [17:0] u_neighbor_r;
reg signed [17:0] u_neighbor_lc;
reg signed [17:0] u_neighbor_rc;

wire signed [17:0] alpha;
assign alpha = 18'b01_0000000000000000;

wire signed [17:0] beta;
assign beta = 18'b00_0100000000000000;

wire signed [17:0] gamma;
assign gamma = 18'b00_0010000000000000;


// outputs from diffusion solver
wire signed [17:0] u_next_top;
reg  signed [17:0] u_next_reg; // this will store our current node's output so that we can free up the solver to calculate the diffusion of the cell above us. 

wire is_frozen;
reg  is_frozen_reg; // store frozen val for the current node so we can free up solver to calculate diffusion for the node above us.
reg  is_frozen_bottom; // store frozen val for the cell below us 


/*
neighbor indexing: 
    even:
        4 0 5 
        2 x 3
        - 1 -
    odd:
        - 0 - 
        2 x 3
        4 1 5
*/

diffusion_solver curr_cell (
    .u_neighbor_0 (u_neighbor_t), // top
    .u_neighbor_1 (u_neighbor_b), // bottom
    .u_neighbor_2 (beta),         // left
    .u_neighbor_3 (beta),         // right
    .u_neighbor_4 (beta),         // left corner
    .u_neighbor_5 (beta),         // right corner
    .u_curr       (u_curr),
    .v_next       (v_next),
    .alpha        (alpha),
    .beta         (beta),
      
    .u_next       (u_next_top),
    .is_frozen    (is_frozen)
);

M10K_1000_8 M10k_u_curr ( 
    .q             (read_data_u_curr), 
    .d             (write_data_u_curr), 
    .write_address (write_addr_u_curr),
    .read_address  (read_addr_u_curr),
    .we            (write_en_u_curr),
    .clk           (clk)
);

M10K_1000_8 M10k_v_next ( // we only store v_next bc v_next = v_curr + gamma, but we never need to store v_curr
    .q             (read_data_v_next), 
    .d             (write_data_v_next), 
    .write_address (write_addr_v_next),
    .read_address  (read_addr_v_next),
    .we            (write_en_v_next),
    .clk           (clk)
);

// ------------------------- //
//       STATE MACHINE       //
// ------------------------- //

// 11 nodes

// reg [15:0] idx;

reg [4:0] state;

always @(posedge clk) begin
    if (reset) begin
        write_data_u_curr <= beta;
        write_data_v_next <= 18'd0;
        write_addr_u_curr <= 19'd0;
        write_addr_v_next <= 19'd0;
        write_en_u_curr <= 1'd1;
        write_en_v_next <= 1'd1;

        read_addr_u_curr <= 19'd0;
        read_addr_v_next <= 19'd0;

        state  <= 5'd0;
    end
    else begin
        case (state)

            5'd0: begin // initialization
                // SETTING INITIAL U AND V VALUES //
                // only center node is frozen, set to 1
                if (write_addr_u_curr == 19'd3) begin 
                    // on next cycle, it'll be the center node
                    write_data_u_curr <= 18'd0;
                    write_data_v_next <= 18'b01_0000000000000000 + gamma;
                end
                else if (write_addr_u_curr == 19'd2 || write_addr_u_curr == 19'd4) begin
                    // the nodes around the center node are receptive, but not frozen 
                    write_data_u_curr <= 18'd0;
                    write_data_v_next <= beta + gamma;
                end
                else begin
                    // all other nodes are nonreceptive
                    write_data_u_curr <= beta;
                    write_data_v_next <= 18'd0;
                end

                // MOVE FORWARD OR STOP //
                if (write_addr_u_curr >= 19'd10) begin
                    state <= 5'd1;
                    write_addr_u_curr <= 19'd0;
                    write_addr_v_next <= 19'd0;
                    write_en_u_curr <= 1'd0;
                    write_en_v_next <= 1'd0;
                end
                else begin
                    state <= 5'd0;
                    write_addr_u_curr <= write_addr_u_curr + 19'd1;
                    write_addr_v_next <= write_addr_v_next + 19'd1;
                    write_en_u_curr <= 1'd1;
                    write_en_v_next <= 1'd1;
                end
            end

            5'd1: begin
                u_neighbor_b      <= beta; // bottom 

                // read from m10ks for u_curr and v_next for this cell
                u_curr            <= read_data_u_curr;
                v_next            <= ((read_data_v_next + gamma) >= 18'b01_0000000000000000)? 18'b01_0000000000000000 : (read_data_v_next + gamma);
                read_addr_u_curr  <= read_addr_u_curr + 19'd1;
                state             <= 5'd2;
            end

            5'd2: begin
                // wait for next read to come back
                state             <= 5'd3;
            end
            5'd3: begin
                // read from m10k for top neighbor
                u_neighbor_t <= read_data_u_curr; 

                // increment these now, so we can read them a cycle earlier
                read_addr_u_curr <= read_addr_u_curr + 19'd1;
                read_addr_v_next <= read_addr_v_next + 19'd1;

                state             <= 5'd4;
            end
            5'd4: begin
                // now we can use diffusion solver output
                // store these outputs and then move up to calculate diffusion for the cell above us. 
                is_frozen_reg <= is_frozen;
                u_next_reg <= u_next_top; // u_next_top is solver output
                v_next_reg <= v_next;                

                // take a step up
                u_neighbor_b <= u_curr;
                u_curr <= u_neighbor_t;
                state <= 5'd5;
                

            end
            5'd5: begin // read m10k for the cell above us so we can calc diffusion for it
                // read from m10k for top neighbor
                u_neighbor_t <= read_data_u_curr; 

                // read from m10k for v_next
                v_next            <= ((read_data_v_next + gamma) >= 18'b01_0000000000000000)? 18'b01_0000000000000000 : (read_data_v_next + gamma);

                // increment these now, so we can read them a cycle earlier
                read_addr_u_curr <= read_addr_u_curr + 19'd1;
                read_addr_v_next <= read_addr_v_next + 19'd1;

                state             <= 5'd6;

            end
            5'd6: begin
                // check if any neighbors are frozen
                // eventually will have to also check r,l,rc,rl neighbors too but 
                // right now r,l,rc,rl are edge cells so they are static
                if (is_frozen_reg || is_frozen_bottom || is_frozen) begin
                    // if we or any neighbors are frozen, we are a receptive cell.
                    // write the new s=u+v value to v, write 0 to u
                    write_data_v_next <= v_next_reg + u_next_reg;
                    write_data_u_curr <= 18'd0;
                end
                else begin
                    // if us and none of our neighbors are frozen, we are a non-receptive cell
                    // write 0 to v, write the new s=u+v value to u
                    write_data_v_next <= 18'd0;
                    write_data_u_curr <= v_next_reg + u_next_reg;
                end

                write_en_u_curr <= 1'b1;
                write_en_v_next <= 1'b1;

                state <= 5'd7;
            end
            5'd7: begin
                // move up one cell!
                u_neighbor_b <= u_curr;
                u_curr       <= u_neighbor_t;

                is_frozen_bottom <= is_frozen_reg;
                is_frozen_reg    <= is_frozen;

                u_next_reg <= u_next_top;
                v_next_reg <= v_next;

                // read from m10k for top neighbor
                u_neighbor_t <= (read_addr_u_curr >= 19'd10) ? beta : read_data_u_curr; // consider top edge boundary

                // read from m10k for v_next
                v_next <= (read_addr_v_next >= 19'd10) ? 18'd0 : (((read_data_v_next + gamma) >= 18'b01_0000000000000000)? 18'b01_0000000000000000 : (read_data_v_next + gamma)); // consider top edge boundary

                // incr write addresses
                if (write_addr_u_curr >= 19'd10) begin
                    write_addr_u_curr <= 19'd0;
                    write_addr_v_next <= 19'd0;
                    read_addr_u_curr  <= 19'd0;
                    read_addr_v_next  <= 19'd0;
                    state             <= 5'd8;
                end
                else begin
                    write_addr_u_curr <= write_addr_u_curr + 19'd1;
                    write_addr_v_next <= write_addr_v_next + 19'd1;
                    // increment these now, so we can read them a cycle earlier
                    // consider top edge boundary: make sure we're not trying to read nonexistent data from m10ks
                    read_addr_u_curr  <= (read_addr_u_curr >= 19'd10) ? read_addr_u_curr : (read_addr_u_curr + 19'd1);
                    read_addr_v_next  <= (read_addr_v_next >= 19'd10) ? read_addr_v_next : (read_addr_v_next + 19'd1);
                    state             <= 5'd6;
                end
                write_en_u_curr <= 1'b0;
                write_en_v_next <= 1'b0;
            end

            5'd8: begin // this is a wait state so we can get our read data at the bottom in state 1 
                state <= 5'd1;
            end

        endcase
    end
end
endmodule



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

    wire signed [17:0] u_avg_0;
    wire signed [17:0] u_avg_1;
    wire signed [17:0] u_avg_total;
    wire signed [17:0] laplace_out;
    wire signed [17:0] u_next_tmp;

    // s = u + v
    // frozen if s >= 1
    assign is_frozen = ((u_next + v_next) >= 18'b01_0000000000000000);
    assign u_next_tmp = (u_curr + laplace_out); 
    assign u_next = (u_next_tmp >= 18'b01_0000000000000000) ? 18'b01_0000000000000000 : u_next_tmp;

    signed_mult u_avg_calc0 ( // divide by 6 (num neighbors) -- mult by 1/6
        .out(u_avg_0),
        .a  (u_neighbor_0+u_neighbor_1+u_neighbor_2),
        .b  (18'b00_0010101010101010) // 1/6
    );

    signed_mult u_avg_calc1 ( // divide by 6 (num neighbors) -- mult by 1/6
        .out(u_avg_1),
        .a  (u_neighbor_3+u_neighbor_4+u_neighbor_5),
        .b  (18'b00_0010101010101010) // 1/6
    );

    assign u_avg_total = u_avg_0 + u_avg_1;

    signed_mult laplace_calc ( // alpha / 2 * (u_avg - cell.u)
        .out(laplace_out),
        .a  (alpha >>> 1),
        .b  (u_avg_total - u_curr)
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


//============================================================
// M10K module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================

module M10K_1000_8( 
    output reg [17:0] q,
    input [17:0] d,
    input [18:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    reg [17:0] mem [68266:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	// reg [7:0] mem [153600:0]; // 2 solvers
	// reg [7:0] mem [76800:0]; // 4 solvers
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
        q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule
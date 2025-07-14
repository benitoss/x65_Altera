//`default_nettype none

module mult_accum (
    input           clk,
    
    input   [15:0]  input_a_16,
    input   [15:0]  input_b_16,
    input           mult_enabled,
    input           reset_accum,
    input           accumulate,
    input           add_or_sub,

    output reg [31:0] output_32  // <-- Keep as reg
);

reg [31:0] product;
reg [31:0] accum_reg;

always @(posedge clk) begin
    if (!mult_enabled)
        product <= input_a_16 * input_b_16;
    else
        product <= input_a_16 * input_b_16;

    if (reset_accum)
        accum_reg <= 32'd0;
    else if (accumulate) 
        accum_reg <= add_or_sub ? accum_reg - product : accum_reg + product;

    output_32 <= accum_reg;  // Registered output
end

endmodule

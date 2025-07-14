// Version optimized for DSP inference

module video_modulator_mult_u8xu8_pair #(
    parameter DATA_WIDTH = 8,
    parameter OUTPUT_WIDTH = 16
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,  // Clock enable
    
    input  wire [DATA_WIDTH-1:0] input_1a_8,
    input  wire [DATA_WIDTH-1:0] input_1b_8,
    input  wire [DATA_WIDTH-1:0] input_2a_8,
    input  wire [DATA_WIDTH-1:0] input_2b_8,
    
    output reg [OUTPUT_WIDTH-1:0] output_1_16,
    output reg [OUTPUT_WIDTH-1:0] output_2_16
);

    // Intermediate multiplication results
    reg [OUTPUT_WIDTH-1:0] mult_1_stage1;
    reg [OUTPUT_WIDTH-1:0] mult_2_stage1;
    
    // Pipeline stage 1: Perform multiplication
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_1_stage1 <= {OUTPUT_WIDTH{1'b0}};
            mult_2_stage1 <= {OUTPUT_WIDTH{1'b0}};
        end else if (enable) begin
            mult_1_stage1 <= input_1a_8 * input_1b_8;
            mult_2_stage1 <= input_2a_8 * input_2b_8;
        end
    end
    
    // Pipeline stage 2: Output register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_1_16 <= {OUTPUT_WIDTH{1'b0}};
            output_2_16 <= {OUTPUT_WIDTH{1'b0}};
        end else if (enable) begin
            output_1_16 <= mult_1_stage1;
            output_2_16 <= mult_2_stage1;
        end
    end

endmodule
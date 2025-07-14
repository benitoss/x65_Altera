// Generic Dual-Port RAM replacement for PDP4K
module generic_pdp4k #(
    parameter ADDR_WIDTH_R = 12,        // Read address width
    parameter ADDR_WIDTH_W = 12,        // Write address width  
    parameter DATA_WIDTH_R = 16,        // Read data width
    parameter DATA_WIDTH_W = 16,        // Write data width
    parameter DEPTH = 4096,             // Memory depth
    parameter INIT_MODE = "all_zero",   // "all_zero", "all_one", "mem_file"
    parameter INIT_VALUE_00 = 256'h0,
    parameter INIT_VALUE_01 = 256'h0,
    parameter INIT_VALUE_02 = 256'h0,
    parameter INIT_VALUE_03 = 256'h0,
    parameter INIT_VALUE_04 = 256'h0,
    parameter INIT_VALUE_05 = 256'h0,
    parameter INIT_VALUE_06 = 256'h0,
    parameter INIT_VALUE_07 = 256'h0,
    parameter INIT_VALUE_08 = 256'h0,
    parameter INIT_VALUE_09 = 256'h0,
    parameter INIT_VALUE_0A = 256'h0,
    parameter INIT_VALUE_0B = 256'h0,
    parameter INIT_VALUE_0C = 256'h0,
    parameter INIT_VALUE_0D = 256'h0,
    parameter INIT_VALUE_0E = 256'h0,
    parameter INIT_VALUE_0F = 256'h0
) (
    // Read port
    input  [ADDR_WIDTH_R-1:0] ADR,      // Read address
    input                     CKR,      // Read clock
    input                     CER,      // Read clock enable
    input                     RE,       // Read enable
    output [DATA_WIDTH_R-1:0] DO,       // Read data out
    
    // Write port
    input  [ADDR_WIDTH_W-1:0] ADW,      // Write address
    input  [DATA_WIDTH_W-1:0] DI,       // Write data in
    input                     CKW,      // Write clock
    input                     CEW,      // Write clock enable
    input                     WE,       // Write enable
    input  [DATA_WIDTH_W-1:0] MASK_N    // Write mask (active low)
);

    // Memory array - using maximum width to accommodate different configurations
    reg [DATA_WIDTH_W-1:0] memory [0:DEPTH-1];
    reg [DATA_WIDTH_R-1:0] read_data_reg;
    
    // Initialize memory based on INIT_MODE
    initial begin
        integer i;
        case (INIT_MODE)
            "all_zero": begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    memory[i] = {DATA_WIDTH_W{1'b0}};
                end
            end
            "all_one": begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    memory[i] = {DATA_WIDTH_W{1'b1}};
                end
            end
            "mem_file": begin
                // Initialize with provided values
                // This is a simplified version - in practice you'd need to
                // parse the 256-bit initialization values properly
                for (i = 0; i < DEPTH; i = i + 1) begin
                    memory[i] = {DATA_WIDTH_W{1'b0}};
                end
                // You would add logic here to load from INIT_VALUE_xx parameters
            end
            default: begin
                for (i = 0; i < DEPTH; i = i + 1) begin
                    memory[i] = {DATA_WIDTH_W{1'b0}};
                end
            end
        endcase
    end
    
    // Write port logic
    always @(posedge CKW) begin
        if (CEW && WE) begin
            integer j;
            for (j = 0; j < DATA_WIDTH_W; j = j + 1) begin
                if (!MASK_N[j]) begin  // MASK_N is active low
                    memory[ADW][j] <= DI[j];
                end
            end
        end
    end
    
    // Read port logic
    always @(posedge CKR) begin
        if (CER && RE) begin
            if (DATA_WIDTH_R == DATA_WIDTH_W) begin
                read_data_reg <= memory[ADR];
            end else if (DATA_WIDTH_R < DATA_WIDTH_W) begin
                // Narrow read - extract appropriate bits
                read_data_reg <= memory[ADR][DATA_WIDTH_R-1:0];
            end else begin
                // Wide read - this would need more complex logic
                read_data_reg <= memory[ADR];
            end
        end
    end
    
    assign DO = read_data_reg;

endmodule
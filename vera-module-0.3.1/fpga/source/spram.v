//`default_nettype none

module spram (
    input              CK,
    input      [13:0]  AD,
    input      [15:0]  DI,
    output reg [15:0]  DO,
    input      [3:0]   MASKWE,
    input              WE,
    input              CS,
    input              STDBY,
    input              SLEEP,
    input              PWROFF_N
);

// Memory declaration: 16,384 x 16 bits = 262,144 bits = ~256 Kbits
reg [15:0] ram [0:16383];

// Masked write logic
always @(posedge CK) begin
    if (CS && !STDBY && !SLEEP && PWROFF_N) begin
        if (WE) begin
            // Byte-write mask based on MASKWE[3:0]
            if (MASKWE[0]) ram[AD][3:0]   <= DI[3:0];  // Lower byte
            if (MASKWE[1]) ram[AD][7:4]   <= DI[7:4]; // Upper byte
            if (MASKWE[2]) ram[AD][11:8]  <= DI[11:8];
            if (MASKWE[3]) ram[AD][15:12] <= DI[15:12];
        end
        DO <= ram[AD];
    end
end

endmodule
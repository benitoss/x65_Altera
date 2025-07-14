module X65_Altera (
// 48 MHz FPGA clock input
    input CLK6X,  // 48 Mhz
	 input CLK_25, // 25 Mhz
	 input CLK6X_LOCKED,

// SNES
    output NESLATCH,
    output NESCLOCK,
    input NESDATA1,
    input NESDATA0,

// LEDs
    inout CPULED0,
    inout CPULED1,
    inout DIPLED0,
    inout DIPLED1,

// CPUTYPE strap
    input CPUTYPE02,        // board assembly CPU type: 0 => 65C816 (16b), 1 => 65C02 (8b)

// PS2 ports
    inout PS2_CLK,
    inout PS2_DATA,

    inout PS2_MCLK,         // via bidi level-shifter
    inout PS2_MDATA,        // via bidi level-shifter

// SDRAM 
	 
	 inout  reg [15:0] SDRAM_DQ,   // 16 bit bidirectional data bus
	 output reg [12:0] SDRAM_A,    // 13 bit multiplexed address bus
	 output reg        SDRAM_DQML, // byte mask
	 output reg        SDRAM_DQMH, // byte mask
	 output reg  [1:0] SDRAM_BA,   // two banks
	 output            SDRAM_nCS,  // a single chip select
	 output reg        SDRAM_nWE,  // write enable
	 output reg        SDRAM_nRAS, // row address select
	 output reg        SDRAM_nCAS, // columns address select
	 output            SDRAM_CLK,
	 output            SDRAM_CKE,

// Video

    output [3:0]	VGA_R,
	 output [3:0]	VGA_G,
	 output [3:0]	VGA_B,
	 output 			VGA_VS,
	 output			VGA_HS,

// I2S_AUDIO
	output        I2S_BCK,
	output        I2S_LRCK,
	output        I2S_DATA,
	 
// UART port
    input UART_CTS,
    output UART_RTS,
    output UART_TX,
    input UART_RX,

// Button input
    input ATTBTN

);



// CPU interface signals
    wire [7:0] CD;
    reg [15:12] CA;
    wire CRESn;
    wire CIRQn;
    wire CNMIn;
    wire CABORTn;
    wire CPHI2;
    wire CBE;
    wire CRDY;
    reg CSOB_MX;
    reg CSYNC_VPA;
    reg CMLn;
    reg CVPn;
    reg CVDA;
    reg CEF;
    reg CRWn;
    
    // Memory bus signals
    wire [20:12] MAH;
    wire [11:0] MAL;
    wire [7:0] MD;
    wire M1CSn;
    wire MRDn;
    wire MWRn;
    
    // I2C bus signals
    wire I2C_SCL;
    wire I2C_SDA;
    
    // SNES signals
//    wire NESLATCH;
//    wire NESCLOCK;
//    reg NESDATA1;
//    reg NESDATA0;
    
    // LED signals
//    wire CPULED0;
//    wire CPULED1;
//    wire DIPLED0;
//    wire DIPLED1;
    
    // CPU type strap
//    reg CPUTYPE02;
    
    // PS2 port signals
//    wire PS2K_CLK;
//    wire PS2K_DATA;
  
//    wire PS2M_CLK;
//    wire PS2M_DATA;
    
    // UART port signals
//    reg UART_CTS;
//    wire UART_RTS;
//    wire UART_TX;
//    reg UART_RX;
    
    // Chip-select signals
    wire VCS0n;
    wire ACS1n;
    wire ECS2n;
    wire UE_CS3n;
    
    // IRQ signals
    reg VIRQn;
    reg AIRQn;
    reg EIRQn;
    
    // Reset signals
    wire ERSTn;
    
    // ICD SPI-slave interface signals
    reg ICD_CSn;
    reg ICD_MOSI;
    wire ICD_MISO;
    reg ICD_SCK;
    
    // Button input
 //   reg ATTBTN;
    
    // Master SPI interface signals
    wire FMOSI;
    reg FMISO;
    wire FSCK;
    wire FLASHCSn;
   
nora_top nora (
        // Clock
        .CLK6X      		(CLK6X),
        .CLK6X_LOCKED 	(CLK6X_LOCKED),
		  
        // CPU interface
        .CD				(CD),
        .CA				(CA),
        .CRESn			(CRESn),
        .CIRQn			(CIRQn),
        .CNMIn			(CNMIn),
        .CABORTn		(CABORTn),
        .CPHI2			(CPHI2),
        .CBE			(CBE),
        .CRDY			(CRDY),
        .CSOB_MX		(CSOB_MX),
        .CSYNC_VPA	(CSYNC_VPA),
        .CMLn			(CMLn),
        .CVPn			(CVPn),
        .CVDA			(CVDA),
        .CEF			(CEF),
        .CRWn			(CRWn),
        
        // Memory bus
        .MAH			(MAH),
        .MAL			(MAL),
        .MD				(MD),
        .M1CSn			(M1CSn),
        .MRDn			(MRDn),
        .MWRn			(MWRn),
        
        // I2C bus
        .I2C_SCL		(I2C_SCL),
        .I2C_SDA		(I2C_SDA),
        
        // SNES
        .NESLATCH		(NESLATCH),
        .NESCLOCK		(NESCLOCK),
        .NESDATA1		(NESDATA1),
        .NESDATA0		(NESDATA0),
        
        // LEDs
        .CPULED0		(CPULED0),
        .CPULED1		(CPULED1),
        .DIPLED0		(DIPLED0),
        .DIPLED1		(DIPLED1),
        
        // CPU type strap
        .CPUTYPE02	(CPUTYPE02),
        
        // PS2 ports
        .PS2K_CLK		(PS2_CLK),
        .PS2K_DATA	(PS2_DATA),
//        `ifdef MOBOV1
//        .PS2K_CLKDR	(PS2K_CLKDR),
//        .PS2K_DATADR	(PS2K_DATADR),
//        `endif
        .PS2M_CLK		(PS2_MCLK),
        .PS2M_DATA	(PS2_MDATA),
//        `ifdef MOBOV1
//        .PS2M_CLKDR(PS2M_CLKDR),
//        .PS2M_DATADR(PS2M_DATADR),
//        `endif
        
        // UART port
        .UART_CTS		(UART_CTS),
        .UART_RTS		(UART_RTS),
        .UART_TX		(UART_TX),
        .UART_RX		(UART_RX),
        
        // Chip-selects
        .VCS0n			(VCS0n),
        .ACS1n			(ACS1n),
        .ECS2n			(ECS2n),
        .UE_CS3n		(UE_CS3n),
        
        // IRQ
        .VIRQn			(VIRQn),
        .AIRQn			(AIRQn),
        .EIRQn			(EIRQn),
        
        // Resets
        .ERSTn			(ERSTn),
        
        // ICD SPI-slave interface
        .ICD_CSn		(ICD_CSn),
        .ICD_MOSI		(ICD_MOSI),
        .ICD_MISO		(ICD_MISO),
        .ICD_SCK		(ICD_SCK),
        
        // Button input
        .ATTBTN		(ATTBTN),
        
        // Master SPI interface
        .FMOSI			(FMOSI),
        .FMISO			(FMISO),
        .FSCK			(FSCK),
        .FLASHCSn		(FLASHCSn)
    );
	 

	 // CPU instantiation
//    P65C816 cpu (
//        .CLK(CPHI2),
//        .RST_N(CRESn),
//        .CE(1'b1),
//        .RDY_IN(CRDY),
//        .NMI_N(CNMIn),
//        .IRQ_N(CIRQn),
//        .ABORT_N(1'b1),
//        .D_IN(MD),
//        .D_OUT(MD),
//        .A_OUT(cpu_a_out),
//        .WE(cpu_we),
//        .RDY_OUT(cpu_rdy_out),
//        .VPA(CSYNC_VPA),
//        .VDA(CVDA),
//        .MLB(CMLn),
//        .VPB(CVPn)
//    );

T65 u_T65 (
  .Mode    (2'b01),     //65c02
  .BCD_en  (1'b1),      //Verify
  .Res_n   (CRESn),
  .Enable  (1'b1),
  .Clk     (CPHI2),
  .Rdy     (CRDY),
  .Abort_n (CABORTn),
  .IRQ_n   (CIRQn),
  .NMI_n   (CNMIn),
  .SO_n    (~CSOB_MX),
  .R_W_n   (CRWn),
  .Sync    (CSYNC_VPA),
  .EF      (CEF),
  .MF      (),
  .XF      (),
  .ML_n    (CMLn),
  .VP_n    (CVPn),
  .VDA     (CVDA),
  .VPA     (CSYNC_VPA),
  .A       (CA),
  .DI      (CD),
  .DO      (CD),
  .Regs    (Regs),
  .DEBUG   (),
  .NMI_ack ()
);

// Instantiation of the top module
vera_top vera_top (
    // Clock
    .clk25(CLK_25),
    
    // External bus interface
    .extbus_cs_n(),
    .extbus_rd_n(),
    .extbus_wr_n(),
    .extbus_a(),
    .extbus_d(),
    .extbus_irq_n(),
    
    // VGA interface
    .vga_r(VGA_R),
    .vga_g(VGA_G),
    .vga_b(VGA_B),
    .vga_hsync(VGA_HS),
    .vga_vsync(VGA_VS),
    
    // SPI interface
    .spi_sck(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_ssel_n_sd(spi_ssel_n_sd),
    
    // Audio output
    .audio_lrck(vera_audio_lrck),
    .audio_bck(vera_audio_bck),
    .audio_data(vera_audio_data)
);

	wire vera_audio_lrck;
	wire vera_audio_data;
	wire vera_audio_bck;


aura aura_inst (
    // System Clock input
    .ASYSCLK(CLK_25),
    
    // I/O Bus
    .AB(address_bus),           // 5-bit address bus
    .DB(data_bus),             // 8-bit bidirectional data bus
    .ACS1N(audio_chip_select_n),
    .VCS0N(vera_chip_select_n),
    .MRDN(memory_read_n),
    .MWRN(memory_write_n),
    .AIRQN(audio_irq_n),
    .IOCSN(io_chip_select_n),
    
    // Audio input from VERA (I2S)
    .VAUDIO_LRCK(vera_audio_lrck),
    .VAUDIO_DATA(vera_audio_data),
    .VAUDIO_BCK(vera_audio_bck),
    
    // Audio output I2S
    .AUDIO_BCK(I2S_LRCKk),
    .AUDIO_DATA(I2S_DATA),
    .AUDIO_LRCK(I2S_BCK),
    
    // SD-Card sense and LED output
    .SD_SSELN(sd_slave_select_n),
    .AURALED(aura_led_out),
    
    // SPI Flash
    .ASPI_MOSI(spi_master_out),
    .ASPI_MISO(spi_master_in),
    .ASPI_SCK(spi_clock_out),
    .AFLASH_SSELN(flash_slave_select_n)
);
	 
	 
	 
endmodule

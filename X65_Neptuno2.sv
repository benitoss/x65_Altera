// X65 Altera port by Benitoss

module X65_Neptuno2 
#(VGA_BITS = 8)
(
	input         CLOCK_50,

	input         KEY1,
	input         KEY2,
	
	output        LED1,
	output        LED2,
	
	input         PS2_CLK,
	input         PS2_DAT,

	input         PS2_MCLK,
	input         PS2_MDAT,
	
	output [VGA_BITS-1:0] VGA_R,
	output [VGA_BITS-1:0] VGA_G,
	output [VGA_BITS-1:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,

	// SD ad
	 output        SD_CS_n,
    output        SD_CLK,
    output        SD_MOSI,
    input         SD_MISO,

	// SDRAM
	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	
	
`ifdef ANALOG_AUDIO
	output        AUDIO_L,
	output        AUDIO_R,
`endif


   // NES Joysticks
	 output NESLATCH,
    output NESCLOCK,
    input NESDATA1,
    input NESDATA0,
	

   // Joysticks
  	output wire   JOY_CLK,
   output wire   JOY_LOAD,
   input  wire   JOY_DATA,
   output wire   JOY_SELECT,
 

	output        I2S_BCK,
	output        I2S_LRCK,
	output        I2S_DATA


//`ifdef USE_AUDIO_IN
//	input         AUDIO_IN,
//`endif
//	input         UART_RX,
//	output        UART_TX

);

`ifdef NO_DIRECT_UPLOAD
localparam bit DIRECT_UPLOAD = 0;
wire SPI_SS4 = 1;
`else
localparam bit DIRECT_UPLOAD = 1;
`endif

`ifdef USE_QSPI
localparam bit QSPI = 1;
assign QDAT = 4'hZ;
`else
localparam bit QSPI = 0;
`endif

//`ifdef VGA_8BIT
//localparam VGA_BITS = 8;
//`else
//localparam VGA_BITS = 6;
//`endif

`ifdef USE_HDMI
localparam bit HDMI = 1;
assign HDMI_RST = 1'b1;
`else
localparam bit HDMI = 0;
`endif

`ifdef BIG_OSD
localparam bit BIG_OSD = 1;
`define SEP "-;",
`else
localparam bit BIG_OSD = 0;
`define SEP
`endif


`include "build_id.v"

//localparam CONF_STR = {
//	"X65;;",
//	"S0U,VHDIMG,Mount SD;",
//	"O2,UartMon,Yes,No;",
//	"O34,Scanlines,Off,25%,50%,75%;",
//	"O5,Blend,Off,On;",
//	"O6,Joystick Swap,Off,On;",
//	`SEP
//	"T0,Reset;",
//	"V,v1.00.",`BUILD_DATE
//};
//
//wire [63:0] status;
//wire        ioctl_downl;
//
//wire  [1:0] scanlines = status[4:3];
//wire        blend = status[5];
//wire        joyswap = status[6];
//wire        userport = status[7];
//wire        pausetzx = status[1];
//wire        invtapein = status[8];
//
////assign 		LED = ~ioctl_downl & (tzxplayer_pause | tape_motor_led);
//assign 		LED = ~ioctl_downl;
//assign 		SDRAM_CKE = 1;
//
//wire uart_rx_mon = ~status[2] ? UART_RX:1'b1;
//wire uart_rx_tmp = ~status[2] ? 1'b1:UART_RX;
//wire uart_tx_mon, uart_tx_tmp;
//assign UART_TX = ~status[2] ? uart_tx_mon:uart_tx_tmp;

// Clock generation
//wire sdclk, clk_112, clk_28, clk_28n = ~clk_28, clk_14, clk_7, pll_locked;
wire clk6x, clk_25, clk6x_locked;

pll	pll_inst (
	.inclk0 ( CLOCK_50 ),  // 50.00 Mhz
	.c0     ( clk6x ),  // 48.00 Mhz
   .c1     ( clk_25 ),	// 25.00 Mhz
	.locked ( clk6x_locked )
	);

//pll pll(
//	.inclk0(CLOCK_50),  // In Poseidon is 50 Mhz
//	.c0(clock27),
//	.c1(clock40_5),
//	.c2(clock81),
//	.c3(clock162),
//	.c4(clock162m), // phase shifted by -207 degrees for SDRAM read timing
//	.locked(pll_locked1)
//	);
//
//wire clock200, clock100, clock50, pll_locked2, clocksd;
//
//pll2	pll2 (
//	.inclk0 (CLOCK_50),
//	.c0 (clock200),
//	.c1 (clock100),
//	.c2 (clock50),
//	.c3 (clocksd),
//	.locked (pll_locked2)
//	);

//assign pll_locked = pll_locked1 & pll_locked2;
	
//reg         clk_28_psg_en;
//reg   [3:0] clk_28_div;

//always @(posedge clk_28) begin
//	clk_28_div <= clk_28_div + 1'd1;
//	clk_28_psg_en <= clk_28_div == 0;
//end
//
//// CPU clock gen
//wire        zxn_clock_contend;
//wire        zxn_clock_lsb;
//wire  [1:0] zxn_cpu_speed;
//reg         clk_3m5_cont;
//wire        clk_cpu;
//
//always @(posedge clk_7, posedge reset) begin
//	if (reset)
//		clk_3m5_cont <= 0;
//	else if (zxn_clock_lsb & !zxn_clock_contend)
//		clk_3m5_cont <= 0;
//	else if (!zxn_clock_lsb)
//		clk_3m5_cont <= 1;
//end
//
//reg  [3:0] clk_select;
//always @(posedge clk_28, posedge reset) begin
//	if (reset) begin
//		clk_select <= 4'b0001;
//	end
//	else if (zxn_ram_a_rfsh) begin
//		case (zxn_cpu_speed)
//		2'b00: clk_select <= 4'b0001;
//		2'b01: clk_select <= 4'b0010;
//		2'b10: clk_select <= 4'b0100;
//		2'b11: clk_select <= 4'b1000;
//		endcase
//	end
//end
//
//clock_mux clocks({clk_28, clk_14, clk_7, clk_3m5_cont},clk_select,clk_cpu);
//

//wire  [1:0] buttons;

//// Reset
//reg        reset = 1;
//wire       zxn_reset_soft;
//wire       zxn_reset_hard;
//reg [27:0] reset_cnt;
//
//always @(posedge clock27, negedge pll_locked) begin
//	if (!pll_locked) begin
//		reset <= 1;
//		reset_cnt <= 28'hfffffff;
//	end else begin
//		if (status[0] | buttons[1] | zxn_reset_soft | zxn_reset_hard)
//			reset_cnt <= 28'h000ffff;
//		else if (reset_cnt != 28'b0)
//			reset_cnt <= reset_cnt - 28'b1;
//
//		reset <= (reset_cnt != 28'b0);
//	end
//end

// User IO
//wire [63:0] status;
//wire  [1:0] buttons;
//wire  [1:0] switches;
//wire [15:0] joy0;
//wire [15:0] joy1;
//wire        scandoublerD;
//wire        ypbpr;
//wire        no_csync;
//wire [63:0] rtc;
//wire [15:0] audio;
//wire        key_strobe;
//wire        key_pressed;
//wire        key_extended;
//wire  [7:0] key_code;
//wire signed [8:0] mouse_x;
//wire signed [8:0] mouse_y;
//wire signed [3:0] mouse_z;
//wire  [7:0] mouse_flags;
//wire        mouse_strobe;

// conections between user_io (implementing the SPI communication 
// to the io controller) and the legacy SD Card wrapper
//wire        sd_busy;
//wire [31:0] sd_lba;
//wire  [1:0] sd_rd;
//wire  [1:0] sd_wr;
//wire        sd_ack;
//wire        sd_conf;
//wire        sd_sdhc;
//wire  [7:0] sd_dout;
//wire        sd_dout_strobe;
//wire  [7:0] sd_din;
//wire  [8:0] sd_buff_addr;
//wire        sd_ack_conf;
//wire  [1:0] img_mounted;
//wire [63:0] img_size;
//
//`ifdef USE_HDMI
//wire        i2c_start;
//wire        i2c_read;
//wire  [6:0] i2c_addr;
//wire  [7:0] i2c_subaddr;
//wire  [7:0] i2c_dout;
//wire  [7:0] i2c_din;
//wire        i2c_ack;
//wire        i2c_end;
//`endif
//
//
//wire ps2clk;
//wire ps2data;


//user_io #(.STRLEN(($size(CONF_STR)>>3)), .ROM_DIRECT_UPLOAD(DIRECT_UPLOAD), .FEATURES(32'h0 | (BIG_OSD << 13) | (HDMI << 14)), .PS2DIV(1000)) user_io (
//	.clk_sys        ( clock27         ),
//	.clk_sd         ( clocksd         ),
//	.conf_str       ( CONF_STR       ),
//	.SPI_CLK        ( SPI_SCK        ),
//	.SPI_SS_IO      ( CONF_DATA0     ),
//	.SPI_MISO       ( SPI_DO         ),
//	.SPI_MOSI       ( SPI_DI         ),
//	.buttons        ( buttons        ),
//	.switches       ( switches       ),
//	.scandoubler_disable(scandoublerD),
//	.ypbpr          ( ypbpr          ),
//	.no_csync       ( no_csync       ),
//	.core_mod       (                ),
//	.rtc            ( rtc            ),
//	.key_strobe     ( key_strobe     ),
//	.key_pressed    ( key_pressed    ),
//	.key_extended   ( key_extended   ),
//	.key_code       ( key_code       ),
//	.ps2_kbd_clk    ( ps2clk         ),
//	.ps2_kbd_data   ( ps2data        ),
//	.mouse_x        ( mouse_x        ),
//	.mouse_y        ( mouse_y        ),
//	.mouse_z        ( mouse_z        ),
//	.mouse_flags    ( mouse_flags    ),
//	.mouse_strobe   ( mouse_strobe   ),
//	.joystick_0     ( joy0           ),
//	.joystick_1     ( joy1           ),
//	.status         ( status         ),
//`ifdef USE_HDMI
//	.i2c_start      ( i2c_start      ),
//	.i2c_read       ( i2c_read       ),
//	.i2c_addr       ( i2c_addr       ),
//	.i2c_subaddr    ( i2c_subaddr    ),
//	.i2c_dout       ( i2c_dout       ),
//	.i2c_din        ( i2c_din        ),
//	.i2c_ack        ( i2c_ack        ),
//	.i2c_end        ( i2c_end        ),
//`endif
//   // interface to embedded legacy sd card wrapper
//	.sd_lba     	  ( sd_lba         ),
//	.sd_rd      	  ( sd_rd          ),
//	.sd_wr      	  ( sd_wr          ),
//	.sd_ack     	  ( sd_ack         ),
//	.sd_conf    	  ( sd_conf        ),
//	.sd_sdhc    	  ( sd_sdhc        ),
//	.sd_dout    	  ( sd_dout        ),
//	.sd_dout_strobe ( sd_dout_strobe ),
//	.sd_din     	  ( sd_din         ),
//	.sd_buff_addr   ( sd_buff_addr   ),
//	.sd_ack_conf    ( sd_ack_conf    ),
//
//	.img_mounted    ( img_mounted    ),
//	.img_size       ( img_size       )
//	);

//reg signed  [3:0] zxn_mouse_wheel;
//reg signed  [7:0] zxn_mouse_x;
//reg signed  [7:0] zxn_mouse_y;
//reg   [2:0] zxn_mouse_button;
//
//always @(posedge clk_28) begin
//	if (mouse_strobe) begin
//		zxn_mouse_x <= zxn_mouse_x + mouse_x;
//		zxn_mouse_y <= zxn_mouse_y + mouse_y;
//		zxn_mouse_wheel <= zxn_mouse_wheel + mouse_z;
//		zxn_mouse_button <= mouse_flags[2:0];
//	end
//end

//wire [10:0] zxn_joy_left =  joyswap ? joy0[10:0] : joy1[10:0];
//wire [10:0] zxn_joy_right = joyswap ? joy1[10:0] : joy0[10:0];
//wire  [2:0] zxn_joy_left_type;
//wire  [2:0] zxn_joy_right_type;
//wire        zxn_joy_io_mode_en;

// SD Card
//wire zxn_spi_ss_sd0_n;
//wire zxn_spi_sck;
//wire zxn_spi_mosi;
//wire sd_miso_i;
//
//
//wire sd_cs_n_int;
//wire sd_sck_int;
//wire sd_mosi_int;
//wire sd_miso_int;
//
//wire sd_cs_n_int_2;
//wire sd_sck_int_2;
//wire sd_mosi_int_2;
//wire sd_miso_int_2;
//
//
//sd_card sd_card (
//	// connection to io controller
//	.clk_sys      ( clocksd         ),
//	.reset 		  ( reset			),
//	.sd_lba       ( sd_lba         ),
//	.sd_rd        ( sd_rd[0]       ),
//	.sd_wr        ( sd_wr[0]       ),
//	.sd_ack       ( sd_ack         ),
//	.sd_ack_conf  ( sd_ack_conf    ),
//	.sd_conf      ( sd_conf        ),
//	.sd_sdhc      ( sd_sdhc        ),
//	.sd_buff_dout ( sd_dout        ),
//	.sd_buff_wr   ( sd_dout_strobe ),
//	.sd_buff_din  ( sd_din         ),
//	.sd_buff_addr ( sd_buff_addr   ),
//	.img_mounted  ( img_mounted[0] ),
//	.img_size     ( img_size       ),
//	.allow_sdhc   ( 1'b1           ),
//	.sd_busy      ( sd_busy        ),
//
//	// connection to local CPU
//	.sd_cs        ( sd_cs_n_int ),
//	.sd_sck_a       ( sd_sck_int  ),
//	.sd_sdi       ( sd_mosi_int),
//	.sd_sdo       ( sd_miso_int )
//);

//sd_wrap sd_wrap (
//	.clk_sys      ( clock200         ),
//	.reset 		  ( reset			),
//
//	.sd_cs_i        ( sd_cs_n_int ),
//	.sd_sck_i       ( sd_sck_int  ),
//	.sd_sdi_i       ( sd_mosi_int ),
//	.sd_sdo_i       ( sd_miso_int ),
//	
//	.sd_cs_o        ( sd_cs_n_int_2 ),
//	.sd_sck_o       ( sd_sck_int_2  ),
//	.sd_sdi_o       ( sd_mosi_int_2 ),
//	.sd_sdo_o       ( sd_miso_int_2 )
//	
//);



// data io (TZX upload)
//wire        ioctl_downl;
//wire        ioctl_upl;
//wire  [7:0] ioctl_index;
//wire        ioctl_wr;
//wire [24:0] ioctl_addr;
//wire  [7:0] ioctl_dout;
//wire  [7:0] ioctl_din;
//
//data_io #(.ROM_DIRECT_UPLOAD(DIRECT_UPLOAD)) data_io(
//	.clk_sys       ( clock27       ),
//	.SPI_SCK       ( SPI_SCK      ),
//	.SPI_SS2       ( SPI_SS2      ),
//	.SPI_SS4       ( SPI_SS4      ),
//	.SPI_DI        ( SPI_DI       ),
//	.SPI_DO        ( SPI_DO       ),
//	.ioctl_download( ioctl_downl  ),
//	.ioctl_upload  ( ioctl_upl    ),
//	.ioctl_index   ( ioctl_index  ),
//	.ioctl_wr      ( ioctl_wr     ),
//	.ioctl_addr    ( ioctl_addr   ),
//	.ioctl_dout    ( ioctl_dout   ),
//	.ioctl_din     ( ioctl_din    )
//);
//
//
//// Wait generator
//reg         sdram_cpuwaitD;
//reg         cpu_mreqD;
//reg         wait_t1t2;
//reg         zxn_ram_a_req_reg;
//
//
//


	wire joy1up;
   wire joy1down;
   wire joy1left;
   wire joy1right;
   wire joy1fire1;
   wire joy1fire2;

   wire joy2up;
   wire joy2down;
   wire joy2left;
   wire joy2right;
   wire joy2fire1;
   wire joy2fire2;
	
	assign JOY_SELECT = 1'b1;

neptuno_joydecoder  neptuno_joydecoder
(
	.clk_i           ( clk6x ),
	.joy_data_i      ( JOY_DATA ),
	.joy_clk_o       ( JOY_CLK ),
	.joy_load_o      ( JOY_LOAD ),

	.joy1_up_o       ( joy1up ),
	.joy1_down_o     ( joy1down ),
	.joy1_left_o     ( joy1left ),
	.joy1_right_o    ( joy1right ),
	.joy1_fire1_o    ( joy1fire1 ),
	.joy1_fire2_o    ( joy1fire2 ),

	.joy2_up_o       ( joy2up ),
	.joy2_down_o     ( joy2down ),
	.joy2_left_o     ( joy2left ),
	.joy2_right_o    ( joy2right ),
	.joy2_fire1_o    ( joy2fire1 ),
	.joy2_fire2_o    ( joy2fire2 )
);


wire			vdac_clk;
wire			vdac_blank_n;
wire			vsync;
wire			hsync;
wire  [3:0]	vgared;
wire  [3:0]	vgagreen;
wire  [3:0]	vgablue;

//wire led;
//wire led2;
//wire ps2clk;
//wire ps2data;

	//Instantiate the top module	 
X65_Altera x65_core (
        .CLK6X(CLK6X),    // 48 Mhz Clock
		  .CLK_25(clk_25),  // 25 Mhz Clock
        .CLK6X_LOCKED(CLK6X_LOCKED),
        
		  .NESLATCH(NESLATCH),
        .NESCLOCK(NESCLOCK),
        .NESDATA1(NESDATA1),
        .NESDATA0(NESDATA0),
		  
        .CPULED0(LED1),
        .CPULED1(LED2),
        
		  .DIPLED0(DIPLED0),
        .DIPLED1(DIPLED1),
        
		  .CPUTYPE02(1'b1),  //CPU type: 0 => 65C816 (16b), 1 => 65C02 (8b)
        
		  .PS2_CLK(PS2K_CLK),
        .PS2_DATA(PS2K_DATA),
        .PS2_MCLK(PS2M_CLK),
        .PS2_MDATA(PS2M_DATA),
        
		  .VGA_R(vgared),
		  .VGA_G(vgagreen),
		  .VGA_B(vgablue),
	     .VGA_VS(vsync),
	     .VGA_HS(hsync),
		  
		  .UART_CTS(UART_CTS),
        .UART_RTS(UART_RTS),
        .UART_TX(UART_TX),
        .UART_RX(UART_RX),
        
		  .ATTBTN(ATTBTN)
    );
	 
	assign VGA_R = {vgared[3:0],vgared[3:0]};
	assign VGA_G = {vgagreen[3:0],vgagreen[3:0]};
	assign VGA_B = {vgablue[3:0],vgablue[3:0]};
	assign VGA_VS = vsync;
	assign VGA_HS = hsync;  
	 
	 
// Instantiate the top module
//nora_top nora (
//        // Clock
//        .CLK6X      		(clk6x),
//        .CLK6X_LOCKED 	(clk6x_locked),
//		  
//        // CPU interface
//        .CD				(CD),
//        .CA				(CA),
//        .CRESn			(CRESn),
//        .CIRQn			(CIRQn),
//        .CNMIn			(CNMIn),
//        .CABORTn		(CABORTn),
//        .CPHI2			(CPHI2),
//        .CBE			   (CBE),
//        .CRDY			(CRDY),
//        .CSOB_MX		(CSOB_MX),
//        .CSYNC_VPA	   (CSYNC_VPA),
//        .CMLn			(CMLn),
//        .CVPn			(CVPn),
//        .CVDA			(CVDA),
//        .CEF			   (CEF),
//        .CRWn			(CRWn),
//        
//        // Memory bus
//        .MAH			   (MAH),
//        .MAL			   (MAL),
//        .MD				(MD),
//        .M1CSn			(M1CSn),
//        .MRDn			(MRDn),
//        .MWRn			(MWRn),
//        
//        // I2C bus
//        .I2C_SCL		(I2C_SCL),
//        .I2C_SDA		(I2C_SDA),
//        
//        // SNES
//        .NESLATCH		(NESLATCH),
//        .NESCLOCK		(NESCLOCK),
//        .NESDATA1		(NESDATA1),
//        .NESDATA0		(NESDATA0),
//        
//        // LEDs
//        .CPULED0		(CPULED0),
//        .CPULED1		(CPULED1),
//        .DIPLED0		(DIPLED0),
//        .DIPLED1		(DIPLED1),
//        
//        // CPU type strap
//        .CPUTYPE02	(CPUTYPE02),
//        
//        // PS2 ports
//        .PS2K_CLK		(PS2_CLK),
//        .PS2K_DATA	(PS2_DAT),
////        `ifdef MOBOV1
////        .PS2K_CLKDR	(PS2K_CLKDR),
////        .PS2K_DATADR	(PS2K_DATADR),
////        `endif
//        .PS2M_CLK		(PS2_MCLK),
//        .PS2M_DATA	(PS2_MDATA),
////        `ifdef MOBOV1
////        .PS2M_CLKDR(PS2M_CLKDR),
////        .PS2M_DATADR(PS2M_DATADR),
////        `endif
//        .SDRAM_CLK(SDRAM_CLK),
//	     .SDRAM_A(SDRAM_A),
//	     .SDRAM_BA(SDRAM_BA),
//	     .SDRAM_DQ(SDRAM_DQ),
//	     .SDRAM_DQML(SDRAM_DQML),
//	     .SDRAM_DQMH(SDRAM_DQMH),
//	     .SDRAM_nCS(SDRAM_nCS),
//	     .SDRAM_nWE(SDRAM_nWE),
//	     .SDRAM_nRAS(SDRAM_nRAS),
//	     .SDRAM_nCAS(SDRAM_nCAS),
//	     .SDRAM_CKE(SDRAM_CKE),
//
//	 
////	.vdac_clk				(vdac_clk),
////	.vdac_blank_n			(vdac_blank_n),
////	.vsync					(vsync ),
////	.hsync					(hsync ),
////	.vgared					(vgared),
////	.vgagreen				(vgagreen),
////	.vgablue					(vgablue),
////	
////	
////	  .audio_blck			(I2S_BCK),
////   .audio_lrclk			(I2S_LRCK),
////   .audio_sdata			(I2S_DATA),	 
//  
//        // UART port
//        .UART_CTS		(UART_CTS),
//        .UART_RTS		(UART_RTS),
//        .UART_TX		(UART_TX),
//        .UART_RX		(UART_RX),
//        
//        // Chip-selects
//        .VCS0n			(VCS0n),
//        .ACS1n			(ACS1n),
//        .ECS2n			(ECS2n),
//        .UE_CS3n		(UE_CS3n),
//        
//        // IRQ
//        .VIRQn			(VIRQn),
//        .AIRQn			(AIRQn),
//        .EIRQn			(EIRQn),
//        
//        // Resets
//        .ERSTn			(ERSTn),
//        
//        // ICD SPI-slave interface
//        .ICD_CSn		(ICD_CSn),
//        .ICD_MOSI		(ICD_MOSI),
//        .ICD_MISO		(ICD_MISO),
//        .ICD_SCK		(ICD_SCK),
//        
//        // Button input
//        .ATTBTN		(KEY1),
//        
//        // Master SPI interface
//        .FMOSI			(FMOSI),
//        .FMISO			(FMISO),
//        .FSCK			(FSCK),
//        .FLASHCSn		(FLASHCSn)
//    );



//container Mega65_instance (
//	.clock27					(clock27),
//	.cpuclock				(clock40_5),
//	.pixelclock				(clock81),
//	.clock162				(clock162),
//	.clock162m           (clock162m),
//	.clock200				(clock200),
//	.clock100				(clock100),
//	.ethclock				(clock50),
//	
//	.btnCpuReset			(KEY1),
//	
//	.ps2clk 					(PS2_CLK),
//	.ps2data					(PS2_DAT),
//	
//	.fa_left					( joy1left ), //1'b1
//	.fa_right				( joy1right ),
//   .fa_up					( joy1up ),
//   .fa_down					( joy1down ),
//   .fa_fire					( joy1fire1 ),
//   .fb_left					( joy2left ),
//   .fb_right				( joy2right ),
//   .fb_up					( joy2up ),
//   .fb_down					( joy2down ),
//   .fb_fire					( joy2fire1 ),
//	
//	.sdram_clk 				(SDRAM_CLK),
//   .sdram_cke				(SDRAM_CKE),
//   .sdram_ras_n			(SDRAM_nRAS),
//   .sdram_cas_n			(SDRAM_nCAS),
//   .sdram_we_n				(SDRAM_nWE), 
//   .sdram_cs_n				(SDRAM_nCS),
//   .sdram_ba				(SDRAM_BA),
//   .sdram_a					(SDRAM_A),
//   .sdram_dqml				(SDRAM_DQML),
//   .sdram_dqmh				(SDRAM_DQMH),
//   .sdram_dq				(SDRAM_DQ),
//	
//	.QspiDB					(      ),
//	.QspiCSn					(		 ),
//	
//	.vdac_clk				(vdac_clk),
//	.vdac_blank_n			(vdac_blank_n),
//	.vsync					(vsync ),
//	.hsync					(hsync ),
//	.vgared					(vgared),
//	.vgagreen				(vgagreen),
//	.vgablue					(vgablue),
//	
//	
//	.audio_blck				(I2S_BCK),
//   .audio_lrclk			(I2S_LRCK),
//   .audio_sdata			(I2S_DATA),
//
//
////   .eth_mdio            (eth_mdio),
////   .eth_mdc             (eth_mdc),
////   .eth_reset           (       ),
////   .eth_rxd             (eth_rxd),
////   .eth_txd             (eth_txd),
////   .eth_txen            (eth_txen),
////   .eth_rxer            (0),
////   .eth_rxdv            (eth_rxdv),
////   .eth_interrupt       (0), 
////	  .eth_clock           (       ),
//
//	
////Con la sd Interna contra el Framework no la detecta
////Con la sd Externa contra el Framework la detecta, pero de 2TB
//
//	// internal SD Card from Framework Card0
////   .sdReset				   (sd_cs_n_int),
////   .sdClock				   (sd_sck_int ),
////   .sdMOSI					(sd_mosi_int),
////   .sdMISO					(sd_miso_int),
//   .sd2Reset				   (SD_CS_n),
//   .sd2Clock				   (SD_CLK ),
//   .sd2MOSI					(SD_MOSI),
//   .sd2MISO					(SD_MISO),
//
//	
//	// external SD card in expansion bus Card1
////   .sd2Reset				(SD_CS_n),
////   .sd2Clock				(SD_CLK ),
////   .sd2MOSI					(SD_MOSI),
////   .sd2MISO					(SD_MISO),
//
//
//// internal card in expansion bus Card0
////	.sdReset				(SD_CS_n),
////   .sdClock				(SD_CLK ),
////   .sdMOSI					(SD_MOSI),
////   .sdMISO					(SD_MISO),
//
//// external SD Card from Framework	Card1
////	.sd2Reset				(sd_cs_n_int),
////   .sd2Clock				(sd_sck_int ),
////   .sd2MOSI					(sd_mosi_int),
////   .sd2MISO					(sd_miso_int),
//
//   //.RsRx					(UART_RX),
//   //.UART_TXD				(UART_TX),
//
//   .uart_rx (uart_rx_tmp),
//   .uart_tx (uart_tx_tmp),
//   .uart_rxd_mon(uart_rx_mon),
//   .uart_txd_mon(uart_tx_mon),
//
//	.led						(LED1),
//   .led2						(LED2)
//
//);
//



//reg   [5:0] joy_kempston;
//reg   [4:0] joy_sinclair1;
//reg   [4:0] joy_sinclair2;
//reg   [4:0] joy_cursor;


// Video out
//mist_video #(.COLOR_DEPTH(8), .SD_HCNT_WIDTH(10), .OUT_COLOR_DEPTH(VGA_BITS), .BIG_OSD(BIG_OSD)) mist_video(
//	.clk_sys        ( clock27           ),
//	.SPI_SCK        ( SPI_SCK          ),
//	.SPI_SS3        ( SPI_SS3          ),
//	.SPI_DI         ( SPI_DI           ),
//	.R              ( vgared     ),
//	.G              ( vgagreen     ),
//	.B              ( vgablue     ),
//	.HSync          ( hsync 		     ),
//	.VSync          ( vsync     		  ),
//	.VGA_R          ( VGA_R            ),
//	.VGA_G          ( VGA_G            ),
//	.VGA_B          ( VGA_B            ),
//	.VGA_VS         ( VGA_VS           ),
//	.VGA_HS         ( VGA_HS           ),
//	.ce_divider     ( 3'b1             ),
//	.rotate         ( 2'b00            ),
//	.blend          ( blend            ),
//	.scandoubler_disable( scandoublerD ),
//	.scanlines      ( scanlines        ),
//	.ypbpr          ( ypbpr            ),
//	.no_csync       ( no_csync         )
//	);



endmodule

//
//
//
//`ifdef I2S_AUDIO
//module mist_i2s_master
//(
//	input        reset,
//	input        clk,
//	input [31:0] clk_rate,
//
//	output reg sclk,
//	output reg lrclk,
//	output reg sdata,
//
//	input [AUDIO_DW-1:0]	left_chan,
//	input [AUDIO_DW-1:0]	right_chan
//);
//
//// Clock Setting
//parameter I2S_Freq = 48_000;     // 48 KHz
//parameter AUDIO_DW = 16;
//
//localparam I2S_FreqX2 = I2S_Freq*2*AUDIO_DW*2;
//
//reg  [31:0] cnt;
//wire [31:0] cnt_next = cnt + I2S_FreqX2;
//
//reg         ce;
//
//always @(posedge clk) begin
//	ce <= 0;
//	cnt <= cnt_next;
//	if(cnt_next >= clk_rate) begin
//		cnt <= cnt_next - clk_rate;
//		ce <= 1;
//	end
//end
//
//
//always @(posedge clk) begin
//	reg  [4:0] bit_cnt = 1;
//
//	reg [AUDIO_DW-1:0] left;
//	reg [AUDIO_DW-1:0] right;
//
//	if (reset) begin
//		bit_cnt <= 1;
//		lrclk   <= 1;
//		sclk    <= 1;
//		sdata   <= 1;
//		sclk    <= 1;
//	end
//	else begin
//		if(ce) begin
//			sclk <= ~sclk;
//			if(sclk) begin
//				if(bit_cnt == AUDIO_DW) begin
//					bit_cnt <= 1;
//					lrclk <= ~lrclk;
//					if(lrclk) begin
//						left  <= left_chan;
//						right <= right_chan;
//					end
//				end
//				else begin
//					bit_cnt <= bit_cnt + 1'd1;
//				end
//				sdata <= lrclk ? right[AUDIO_DW - bit_cnt] : left[AUDIO_DW - bit_cnt];
//			end
//		end
//	end
//end
//
//endmodule
//`endif
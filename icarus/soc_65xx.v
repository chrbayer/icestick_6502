// tst_6502.v - test 6502 core
// 02-11-19 E. Brombaugh

module soc_65xx (
    input clk,              // SOC System clock
    input reset_n,          // Low-true reset

	input IRQ_n,
	input NMI_n,

	input RX,				// serial RX
	output TX,				// serial TX

	input [7:0] gpio_a_i,
	output wire [7:0] gpio_a_o,

	input [7:0] gpio_b_i,
	output wire [7:0] gpio_b_o,

	input flag_n,
	output wire pc_n,

	input sp_in,
	output wire sp_out,

	input cnt_in,
	output wire cnt_out
);

	// Memory configuration
	parameter
		ROMPAGE0	= 4'hc,
		ROMPAGE1 	= 4'he,
		ROMPAGE2 	= 4'hf,
		IOPAGE  	= 4'hd,
		CIASUBPAGE	= 6'h00,
		ACIASUBPAGE	= 6'h01;

	// Peripheral clock
	parameter clk_freq    	= 16666666;

`ifndef VERIFICATION
	parameter  periph_freq = 3333333;
	localparam pclk_cnt = (clk_freq / periph_freq);
	localparam PCW = $clog2(pclk_cnt);

	reg pclk;
	reg [PCW-1:0] pclk_counter;

	always @(posedge clk or negedge reset_n)
	begin
		if(~reset_n)
		begin
			pclk <= 0;
			pclk_counter <= 0;
		end
		else if(pclk_counter == pclk_cnt[PCW-1:0])
		begin
			pclk <= 1;
			pclk_counter <= 0;
		end
		else
		begin
			pclk <= 0;
			pclk_counter <= pclk_counter + 1;
		end
	end
`endif

    // The 65xx
    wire [15:0] CPU_AB;
    reg [7:0] CPU_DI;
    wire [7:0] CPU_DO;
    wire CPU_WE_n, CPU_IRQ_n;
`ifdef CPU_6502
    cpu_6502 ucpu (
`elsif CPU_65C02
    cpu_65c02 ucpu (
`elsif CPU_65CE02
    cpu_65ce02 ucpu (
`elsif CPU_45GS02
    cpu_45gs02 ucpu (
`else
    cpu_45gs02 ucpu (
`endif
        .clk(clk),
        .reset_n(reset_n),
        .AB(CPU_AB),
        .DI(CPU_DI),
        .DO(CPU_DO),
        .WE_n(CPU_WE_n),
        .IRQ_n(CPU_IRQ_n),
        .NMI_n(NMI_n),
        .RDY(1'b1)
    );

	// address decode
	wire pFlash = (CPU_AB[15:12] == ROMPAGE1 || CPU_AB[15:12] == ROMPAGE2) ? 1'b1 : 1'b0;
	wire pIo = (CPU_AB[15:12] == IOPAGE) ? 1'b1 : 1'b0;
	wire pRam = ~pIo & ~pFlash;

	wire [5:0] ios = CPU_AB[11:6];

	// RAM @ pages 00-cf
	reg [7:0] ram_mem [0:49151];
	reg [7:0] ram_do;
	always @(posedge clk)
		if((CPU_WE_n == 1'b0) && (pRam == 1'b1))
			ram_mem[CPU_AB[15:0]] <= CPU_DO;
	always @(posedge clk)
		ram_do <= ram_mem[CPU_AB[15:0]];

`ifndef VERIFICATION
	// CIA @ page d0-df
	wire [7:0] cia_do;
	wire cia_irq_n;
	mos6526 #(
		.clk_freq(periph_freq)
	)
	umos6526 (
		.clk(clk),
		.phi2(pclk), // peripheral clock
		.reset_n(reset_n),
		.cs_n(~pIo | (ios != CIASUBPAGE)),
		.rw(CPU_WE_n),
		.rs(CPU_AB[3:0]),
		.db_in(CPU_DO),
		.db_out(cia_do),
		.pa_in(gpio_a_i),
		.pa_out(gpio_a_o),
		.pb_in(gpio_b_i),
		.pb_out(gpio_b_o),
		.flag_n(flag_n),
		.pc_n(pc_n),
		.sp_in(sp_in),
		.sp_out(sp_out),
		.cnt_in(cnt_in),
		.cnt_out(cnt_out),
		.irq_n(cia_irq_n)
	);

	// ACIA at page d0-df
	wire [7:0] acia_do;
	wire acia_irq_n;
	acia #(
		.clk_freq(periph_freq)
	)
	uacia (
		.clk(clk),							// system clock
		.pclk(pclk),						// peripheral clock
		.reset_n(reset_n),					// system reset
		.cs_n(~pIo | (ios != ACIASUBPAGE)),	// chip select
		.we_n(CPU_WE_n),					// write enable
		.rs(CPU_AB[0]),						// register select
		.rx(RX),							// serial receive
		.din(CPU_DO),						// data bus input
		.dout(acia_do),						// data bus output
		.tx(TX),							// serial transmit
		.irq_n(acia_irq_n)					// interrupt request
	);

	assign CPU_IRQ_n = IRQ_n & cia_irq_n & acia_irq_n;
`else
	assign CPU_IRQ_n = IRQ_n;
`endif

	// ROM @ pages e0-ff
	reg [7:0] rom_do;
`ifdef VERIFICATION
    reg [7:0] rom_mem [0:16383];
	initial
`ifdef VERIFICATION_6502
        $readmemh("6502_verification.hex", rom_mem);
`endif
`ifdef VERIFICATION_65C02
        $readmemh("65C02_verification.hex", rom_mem);
`endif
`ifdef DECIMAL_TEST_65C02
        $readmemh("65C02_decimal_test.hex", rom_mem);
`endif
`ifdef VERIFICATION_65CE02
        $readmemh("65CE02_opcodes_test.hex", rom_mem);
`endif
	always @(posedge clk)
		rom_do <= rom_mem[CPU_AB[13:0]];
`ifdef SIM
	initial
`ifdef VERIFICATION_6502
		$monitor("#%0t test_case = %0h", $time, ram_mem[512]);
`endif
`ifdef VERIFICATION_65C02
		$monitor("#%0t test_case = %0h", $time, ram_mem[512]);
`endif
`ifdef DECIMAL_TEST_65C02
		$monitor("#%0t ERROR = %0h", $time, ram_mem[11]);
`endif
`ifdef VERIFICATION_65CE02
		$monitor("#%0t test_case = %0h", $time, ram_mem[514]);
`endif
`endif
`else
    reg [7:0] rom_mem [0:8191];
	initial
        $readmemh("rom.hex", rom_mem);
	always @(posedge clk)
		rom_do <= rom_mem[CPU_AB[12:0]];
`endif


	// data mux
	reg [3:0] mux_sel;
	reg [5:0] sec_sel;
	always @(posedge clk)
		begin
			mux_sel <= CPU_AB[15:12];
			sec_sel <= CPU_AB[11:6];
		end
	always @*
		casez(mux_sel)
			ROMPAGE1,
			ROMPAGE2:	CPU_DI = rom_do;
`ifndef VERIFICATION
			IOPAGE:  casez(sec_sel)
					     CIASUBPAGE:   	CPU_DI = cia_do;
						 ACIASUBPAGE:   CPU_DI = acia_do;
						 default: 		CPU_DI = ram_do;
					 endcase
`else
			ROMPAGE0,
			IOPAGE:  CPU_DI = rom_do;
`endif
			default: CPU_DI = ram_do;
		endcase
endmodule

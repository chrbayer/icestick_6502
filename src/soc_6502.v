// tst_6502.v - test 6502 core
// 02-11-19 E. Brombaugh

module soc_6502(
    input clk,              // SOC System clock
    input reset_n,          // Low-true reset

	output wire [7:0] gpio_o,
	input [7:0] gpio_i,

	input RX,				// serial RX
	output TX				// serial TX
);

	// Peripheral clock
    localparam clk_freq    = 40000000;
    localparam periph_freq = 4000000;
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


    // The 6502
    wire [15:0] CPU_AB;
    reg [7:0] CPU_DI;
    wire [7:0] CPU_DO;
    wire CPU_WE_n, CPU_IRQ_n;
    cpu_65ce02 ucpu(
        .clk(clk),
        .reset_n(reset_n),
        .AB(CPU_AB),
        .DI(CPU_DI),
        .DO(CPU_DO),
        .WE_n(CPU_WE_n),
        .IRQ_n(CPU_IRQ_n),
        .NMI_n(1'b1),
        .RDY(1'b1)
    );

	// address decode - not fully decoded for 512-byte memories
	wire p0 = (CPU_AB[15:12] == 4'h0) ? 0 : 1;
	wire p1 = (CPU_AB[15:12] == 4'h1) ? 0 : 1;
	wire p2 = (CPU_AB[15:12] == 4'h2) ? 0 : 1;
	wire pf = (CPU_AB[15:12] == 4'hf) ? 0 : 1;

	// RAM @ pages 00-0f
	reg [7:0] ram_mem [0:4095];
	initial
        $readmemh("null.hex", ram_mem);
	reg [7:0] ram_do;
	always @(posedge clk)
		if((CPU_WE_n == 1'b0) && (p0 == 1'b0))
			ram_mem[CPU_AB[11:0]] <= CPU_DO;
	always @(posedge clk)
		ram_do <= ram_mem[CPU_AB[11:0]];

	// CIA @ page 10-1f
	wire [7:0] cia_do;
	wire cia_irq_n;
	mos6526 umos6526(
		.clk(clk),
		.phi2(pclk), // peripheral clock
		.reset_n(reset_n),
		.cs_n(p1),
		.rw(CPU_WE_n),
		.rs(CPU_AB[3:0]),
		.db_in(CPU_DO),
		.db_out(cia_do),
		.pa_in(gpio_i),
		.pb_in(gpio_i),
		.pb_out(gpio_o),
		.flag_n(1'b1),
		.sp_in(1'b0),
		.cnt_in(1'b0),
		.irq_n(cia_irq_n)
	);

	// ACIA at page 20-2f
	wire [7:0] acia_do;
	wire acia_irq_n;
	acia uacia(
		.clk(clk),				// system clock
		.pclk(pclk),			// peripheral clock
		.reset_n(reset_n),		// system reset
		.cs_n(p2),				// chip select
		.we_n(CPU_WE_n),		// write enable
		.rs(CPU_AB[0]),			// register select
		.rx(RX),				// serial receive
		.din(CPU_DO),			// data bus input
		.dout(acia_do),			// data bus output
		.tx(TX),				// serial transmit
		.irq_n(acia_irq_n)		// interrupt request
	);

	assign CPU_IRQ_n = cia_irq_n & acia_irq_n;

	// ROM @ pages f0,f1...
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
    reg [7:0] rom_mem [0:4095];
	initial
        $readmemh("rom.hex", rom_mem);
	always @(posedge clk)
		rom_do <= rom_mem[CPU_AB[11:0]];
`endif


	// data mux
	reg [3:0] mux_sel;
	always @(posedge clk)
		mux_sel <= CPU_AB[15:12];
	always @(*)
		casez(mux_sel)
			4'h0: CPU_DI = ram_do;
			4'h1: CPU_DI = cia_do;
			4'h2: CPU_DI = acia_do;
			4'hf: CPU_DI = rom_do;
			default: CPU_DI = rom_do;
		endcase
endmodule

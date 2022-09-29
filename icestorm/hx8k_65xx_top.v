module hx8k_65xx_top(
	input  RX,
	output TX,
	input  clk,
	output LED1,
	output LED2,
	input  BUT1,
	input  BUT2
);

	wire CLK1;
	wire locked;
	//assign CLK1 = clk;
	pll upll (.clock_in(clk), .clock_out(CLK1), .locked(locked));

	// reset generator waits > 10us
	reg [7:0] reset_cnt;
	reg reset_n;
	initial
        reset_cnt <= 8'h00;

	always @(posedge CLK1)
	begin
		if(reset_cnt != 8'hff)
        begin
            reset_cnt <= reset_cnt + 8'h01;
            reset_n <= 1'b0;
        end
        else
            reset_n <= 1'b1;
	end

	// test unit
	wire [7:0] gpio_a_i, gpio_a_o;
	wire [7:0] gpio_b_i, gpio_b_o;
	wire pc_n, sp_out, cnt_out;
	assign gpio_b_i[7:2] = 6'h00;
	soc_65xx u6502 (
		.clk(CLK1),
		.reset_n(reset_n),

		.IRQ_n(1'b1),
		.NMI_n(1'b1),

		.RX(RX),
		.TX(TX),

		.gpio_a_i(gpio_a_i),
		.gpio_a_o(gpio_a_o),
		.gpio_b_i(gpio_b_i),
		.gpio_b_o(gpio_b_o),

		.flag_n(1'b1),
		.pc_n(pc_n),
		.sp_in(1'b0),
		.sp_out(sp_out),
		.cnt_in(1'b0),
		.cnt_out(cnt_out)
	);

	// drive LEDs from GPIO
	assign {LED1,LED2} = gpio_b_o[7:6];
	assign gpio_b_i[1:0] = {BUT1,BUT2};
endmodule

module soc_65xx(
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
		RAMPAGE 	= 4'h0,
		IOPAGE  	= 4'hd,
		CIASUBPAGE	= 6'h00,
		ACIASUBPAGE	= 6'h01;

	// Peripheral clock
    localparam clk_freq    = 35000000;
    localparam periph_freq = 3500000;
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


    // The 65xx
    wire [15:0] CPU_AB;
    reg [7:0] CPU_DI;
    wire [7:0] CPU_DO;
    wire CPU_WE_n, CPU_IRQ_n;
    cpu_45gs02 ucpu (
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

	// address decode - not fully decoded for 512-byte memories
	wire pRam = (CPU_AB[15:12] == RAMPAGE) ? 0 : 1;
	wire pIo = (CPU_AB[15:12] == IOPAGE) ? 0 : 1;

	wire [5:0] ios = CPU_AB[11:6];

	// RAM @ pages 00-0f
	reg [7:0] ram_mem [0:4095];
	reg [7:0] ram_do;
	always @(posedge clk)
		if((CPU_WE_n == 1'b0) && (pRam == 1'b0))
			ram_mem[CPU_AB[11:0]] <= CPU_DO;
	always @(posedge clk)
		ram_do <= ram_mem[CPU_AB[11:0]];

	// CIA @ page 10-1f
	wire [7:0] cia_do;
	wire cia_irq_n;
	mos6526 #(
		.clk_freq(periph_freq)
	)
	umos6526 (
		.clk(clk),
		.phi2(pclk), // peripheral clock
		.reset_n(reset_n),
		.cs_n(pIo | (ios != CIASUBPAGE)),
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

	// ACIA at page 20-2f
	wire [7:0] acia_do;
	wire acia_irq_n;
	acia #(
		.clk_freq(periph_freq)
	)
	uacia (
		.clk(clk),							// system clock
		.pclk(pclk),						// peripheral clock
		.reset_n(reset_n),					// system reset
		.cs_n(pIo | (ios != ACIASUBPAGE)),	// chip select
		.we_n(CPU_WE_n),					// write enable
		.rs(CPU_AB[0]),						// register select
		.rx(RX),							// serial receive
		.din(CPU_DO),						// data bus input
		.dout(acia_do),						// data bus output
		.tx(TX),							// serial transmit
		.irq_n(acia_irq_n)					// interrupt request
	);

	assign CPU_IRQ_n = IRQ_n & cia_irq_n & acia_irq_n;

	// ROM @ pages f0,f1...
	reg [7:0] rom_do;
    reg [7:0] rom_mem [0:8191];
	initial
        $readmemh("rom.hex", rom_mem);
	always @(posedge clk)
		rom_do <= rom_mem[CPU_AB[12:0]];


	// data mux
	reg [3:0] mux_sel;
	reg [5:0] sec_sel;
	always @(posedge clk)
		begin
			mux_sel <= CPU_AB[15:12];
			sec_sel <= CPU_AB[11:6];
		end
	always @(*)
		casez(mux_sel)
			RAMPAGE: CPU_DI = ram_do;
			IOPAGE:  casez(sec_sel)
					     CIASUBPAGE:	CPU_DI = cia_do;
						 ACIASUBPAGE:   CPU_DI = acia_do;
						 default: 		CPU_DI = rom_do;
					 endcase
			default: CPU_DI = rom_do;
		endcase
endmodule

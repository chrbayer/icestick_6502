module hx8k_65xx_top #(
	parameter clk_freq    	= 9200000,
	parameter periph_freq 	= 3333333,
	parameter baudrate		= 115200
) (
`ifndef SIM
	input  clk,
    output [17:0] ADR,
    inout [15:0] DAT,
    output RAMOE,
    output RAMWE,
    output RAMCS,
`endif
	input  RX,
	output TX,
	output LED1,
	output LED2,
	input  BUT1,
	input  BUT2
);

`ifdef SIM
    // clock source
	reg clk;
    wire [17:0] ADR;
    wire [15:0] DAT;
    wire RAMOE;
    wire RAMWE;
    wire RAMCS;

    always
        #2 clk = ~clk;

    // reset
    initial
    begin
  		$dumpfile("tb_hx8k_65xx_top.lxt");
		$dumpvars;

        // init regs
        clk = 1'b0;

		#600000 $finish;
	end
`endif

	wire CLK0;
	wire CLK1;
	wire locked;
	pll upll (.clock_in(clk), .clock_out(CLK0), .locked(locked));
	clk_div3 udiv3 (.clk(CLK0), .clk_out(CLK1));

	// reset generator waits > 10us
	reg [6:0] reset_cnt = 7'd0;
	reg reset_n;

    wire [15:0] data_pins_in;
    wire [15:0] data_pins_out;
    wire data_pins_out_en;
	wire sram_ready;
	wire sram_read_n;
	wire sram_write_n;
	wire [17:0] sram_address;
	wire [15:0] sram_data_read;
	wire [15:0] sram_data_write;

`ifndef SIM
    SB_IO #(
        .PIN_TYPE(6'b 1010_01),
    ) sram_data_pins [15:0] (
        .PACKAGE_PIN(DAT),
        .OUTPUT_ENABLE(data_pins_out_en),
        .D_OUT_0(data_pins_out),
        .D_IN_0(data_pins_in),
    );
`else
	assign DAT = data_pins_out;
`endif

    sram usram (
		.clk(CLK0),
		.address(sram_address),
		.data_read(sram_data_read),
		.data_write(sram_data_write),
		.write(sram_write_n),
		.read(sram_read_n),
		.reset_n(reset_n),
		.ready(sram_ready),
        .data_pins_in(data_pins_in),
        .data_pins_out(data_pins_out),
        .data_pins_out_en(data_pins_out_en),
        .address_pins(ADR),
        .OE(RAMOE),
		.WE(RAMWE),
		.CS(RAMCS)
	);

`ifdef SIM
	sram_chip usram_chip (
		.clk(CLK0),
		.addr(sram_address),
		.data_in(data_pins_out),
		.data_out(data_pins_in),
		.oe(RAMOE),
		.we(RAMWE),
		.cs(RAMCS)
	);
`endif

	always @(posedge CLK0)
	begin
`ifdef SIM
		if(reset_cnt != 7'd1)
`else
		if(reset_cnt != 7'd126)
`endif
        begin
            reset_cnt <= reset_cnt + 7'd1;
            reset_n <= 1'b0;
        end
        else
            reset_n <= 1'b1;
	end

	// soc
	wire [18:0] bus_addr;
	wire [7:0] bus_do;
	assign sram_address = bus_addr[17:0];
	assign sram_data_write = { 8'h00, bus_do };

	wire [7:0] gpio_a_i, gpio_a_o;
	wire [7:0] gpio_b_i, gpio_b_o;
	wire pc_n, sp_out, cnt_out;
	assign gpio_b_i[7:2] = 6'h00;

	soc_65xx #(
		.clk_freq(clk_freq),
		.periph_freq(periph_freq),
		.baudrate(baudrate)
	)
	u6502 (
		.clk(CLK1),
		.reset_n(reset_n),
		.ready(1'b1),

		.IRQ_n(1'b1),
		.NMI_n(1'b1),

		.bus_addr(bus_addr),
		.bus_do(bus_do),
		.bus_di(sram_data_read[7:0]),
		.bus_read(sram_read_n),
		.bus_write(sram_write_n),

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

module clk_div3 (clk, clk_out);
	input clk;
	output clk_out;

	reg [1:0] pos_count = 1, neg_count = 0;

	always @(posedge clk)
		if( pos_count == 2 ) pos_count <= 0;
		else pos_count <= pos_count + 1;

	always @(negedge clk)
		if( neg_count == 2 ) neg_count <= 0;
		else neg_count <= neg_count + 1;

	assign clk_out = ((pos_count == 2) | (neg_count == 2));
endmodule

module soc_65xx #(
	parameter clk_freq    	= 9200000,
	parameter periph_freq 	= 3333333,
	parameter baudrate		= 115200
) (
    input clk,              // SOC System clock
    input reset_n,          // Low-true reset
	input ready,

	input IRQ_n,
	input NMI_n,

	output wire [18:0] bus_addr,
	output wire [7:0] bus_do,
	input [7:0] bus_di,
	output wire bus_read,
	output wire bus_write,

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
		ROMPAGE0	= 8'h0c,
		ROMPAGE1 	= 8'h0e,
		ROMPAGE2 	= 8'h0f,
		IOPAGE  	= 8'h0d,
		CIASUBPAGE	= 6'h00,
		ACIASUBPAGE	= 6'h01;

	parameter
		LOWER_BANK_CFG = 16'hf800,
		UPPER_BANK_CFG = 16'h3800;

	// peripheral clock settings
	localparam pclk_cnt = (clk_freq / periph_freq);
	localparam PCW = $clog2(pclk_cnt);

	reg pclk;
	reg [PCW-1:0] pclk_counter;

	always @(posedge clk)
	begin
		if(~reset_n)
		begin
			pclk <= 0;
			pclk_counter <= 0;
		end
		else if(pclk_counter == (pclk_cnt[PCW-1:0]-1))
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
    wire [19:0] CPU_AB;
    reg [7:0] CPU_DI;
    wire [7:0] CPU_DO;
    wire CPU_WE_n, CPU_IRQ_n;
     cpu_45gs02 #(
		.LOWER_BANK_CFG(LOWER_BANK_CFG),
		.UPPER_BANK_CFG(UPPER_BANK_CFG)
	) ucpu (
        .clk(clk),
        .reset_n(reset_n),
        .eAB(CPU_AB),
        .DI(CPU_DI),
        .DO(CPU_DO),
        .WE_n(CPU_WE_n),
        .IRQ_n(CPU_IRQ_n),
        .NMI_n(NMI_n),
        .RDY(ready)
    );

	// address decode
	wire pFlash = (CPU_AB[19:12] == ROMPAGE1 || CPU_AB[19:12] == ROMPAGE2) ? 1'b1 : 1'b0;
	wire pIo = (CPU_AB[19:12] == IOPAGE) ? 1'b1 : 1'b0;
	wire pRam = CPU_AB[19];

	wire [5:0] ios = CPU_AB[11:6];

	assign bus_read = pRam & CPU_WE_n;
	assign bus_write = pRam & ~CPU_WE_n;
	assign bus_addr = CPU_AB[18:0];
	assign bus_do = CPU_DO;

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
		.clk_freq(periph_freq),
		.baudrate(baudrate)
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

	// ROM @ pages e0-ff
	reg [7:0] rom_do;
    reg [7:0] rom_mem [0:8191];
	initial
`ifdef SIM
        $readmemh("code.hex", rom_mem);
`else
        $readmemh("rom.hex", rom_mem);
`endif
	always @(posedge clk)
		rom_do <= rom_mem[CPU_AB[12:0]];

	// data mux
	reg [7:0] mux_sel;
	reg [5:0] sec_sel;
	always @(posedge clk)
		begin
			mux_sel <= CPU_AB[19:12];
			sec_sel <= CPU_AB[11:6];
		end
	always @*
		casez(mux_sel)
			ROMPAGE1,
			ROMPAGE2:	CPU_DI = rom_do;
			IOPAGE:  casez(sec_sel)
					     CIASUBPAGE:	CPU_DI = cia_do;
						 ACIASUBPAGE:   CPU_DI = acia_do;
						 default: 		CPU_DI = bus_di;
					 endcase
			default: CPU_DI = bus_di;
		endcase

`ifdef SIM
	reg uart_reset_n;

	initial
	begin
		#2 uart_reset_n = 1'b0;
		#552 uart_reset_n = 1'b1;
	end

	wire [7:0] rx_data;
	wire rx_ready;
	wire rx_error;

	always @(posedge rx_ready)
		$write("%c", rx_data);

	acia_rx #(
		.clk_freq(periph_freq),
		.sym_rate(baudrate)
	)
	uuart
	(
		.clk(clk),
		.pclk(pclk),
		.reset_n(uart_reset_n),
		.rx_serial(TX),
		.rx_dat(rx_data),
		.rx_stb(rx_ready),
		.rx_err(rx_error)
	);
`endif

endmodule

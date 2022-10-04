// acia.v - strippped-down version of MC6850 ACIA with home-made TX/RX
// 03-02-19 E. Brombaugh

module acia (
	input clk,				// system clock
	input pclk,				// peripheral clock
	input reset_n,			// system reset
	input cs_n,				// chip select
	input we_n,				// write enable
	input rs,				// register select
	input rx,				// serial receive
	input [7:0] din,		// data bus input
	output reg [7:0] dout,	// data bus output
	output tx,				// serial tx_start
	output irq_n			// low-true interrupt request
);

	// default system clock 4MHz
    parameter clk_freq = 4000000;

	// hard-coded bit-rate
	localparam sym_rate = 9600;
    localparam sym_cnt = clk_freq / sym_rate;
	localparam SCW = $clog2(sym_cnt);

	// generate tx_start signal on write to register 1
	wire tx_start = ~cs_n & rs & ~we_n;

	// load control register
	reg [1:0] counter_divide_select, tx_start_control;
	reg [2:0] word_select; // dummy
	reg receive_interrupt_enable;
	always @(posedge clk)
	begin
		if(~reset_n)
		begin
			counter_divide_select <= 2'b00;
			word_select <= 3'b000;
			tx_start_control <= 2'b00;
			receive_interrupt_enable <= 1'b0;
		end
		else if(~cs_n & ~rs & ~we_n)
			{
				receive_interrupt_enable,
				tx_start_control,
				word_select,
				counter_divide_select
			} <= din;
	end

	// acia reset generation
	wire acia_rst_n = reset_n & (counter_divide_select != 2'b11);

	// load dout with either status or rx data
	wire [7:0] rx_dat, status;
	always @(posedge clk)
	begin
		if(~reset_n)
		begin
			dout <= 8'h00;
		end
		else
		begin
			if(~cs_n & we_n)
			begin
				if(rs)
					dout <= rx_dat;
				else
					dout <= status;
			end
		end
	end

	// tx empty is cleared when tx_start starts, cleared when tx_busy deasserts
	reg txe;
	wire tx_busy;
	reg prev_tx_busy;
	always @(posedge clk)
	begin
		if(~reset_n)
		begin
			txe <= 1'b1;
			prev_tx_busy <= 1'b0;
		end
		else if(pclk)
		begin
			prev_tx_busy <= tx_busy;

			if(tx_start)
				txe <= 1'b0;
			else if(prev_tx_busy & ~tx_busy)
				txe <= 1'b1;
		end
	end

	// rx full is set when rx_stb pulses, cleared when data reg read
	wire rx_stb;
	reg rxf;
	always @(posedge clk)
	begin
		if(~reset_n)
			rxf <= 1'b0;
		else if(pclk)
		begin
			if(rx_stb)
				rxf <= 1'b1;
			else if(~cs_n & rs & we_n)
				rxf <= 1'b0;
		end
	end

	// assemble status byte
	wire rx_err;
	assign status =
	{
		~irq_n,				// bit 7 = irq - forced inactive
		1'b0,				// bit 6 = parity error - unused
		rx_err,			    // bit 5 = overrun error - same as all errors
		rx_err,			    // bit 4 = framing error - same as all errors
		1'b0,				// bit 3 = /CTS - forced active
		1'b0,				// bit 2 = /DCD - forced active
		txe,				// bit 1 = tx_start empty
		rxf					// bit 0 = receive full
	};

	// Async Receiver
	acia_rx #(
		.SCW(SCW),				// rate counter width
		.sym_cnt(sym_cnt)		// rate count value
	)
	my_rx (
		.clk(clk),				// system clock
		.pclk(pclk),			// peripheral clock
		.reset_n(acia_rst_n), 	// system reset
		.rx_serial(rx),		    // raw serial input
		.rx_dat(rx_dat),        // received byte
		.rx_stb(rx_stb),        // received data available
		.rx_err(rx_err)         // received data error
	);

	// Transmitter
	acia_tx #(
		.SCW(SCW),              // rate counter width
		.sym_cnt(sym_cnt)       // rate count value
	)
	my_tx (
		.clk(clk),				// system clock
		.pclk(pclk),			// peripheral clock
		.reset_n(acia_rst_n),	// system reset
		.tx_dat(din),           // transmit data byte
		.tx_start(tx_start),    // trigger transmission
		.tx_serial(tx),         // tx serial output
		.tx_busy(tx_busy)       // tx is active (not ready)
	);

	// generate IRQ
	assign irq_n = ~((rxf & receive_interrupt_enable) | ((tx_start_control==2'b01) & txe));

endmodule

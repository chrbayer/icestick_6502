module hx8k_6502_top(
	input  RX,
	output TX,
	input  clk,
	output LED1,
	output LED2,
	input  BUT1,
	input  BUT2
);

        wire       CLK1;
        wire       locked;
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
	soc_6502 u6502 (
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

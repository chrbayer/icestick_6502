module gw1n1_6502_top (
	input  RX,
	output TX,
	input  clk,
	output LED1,
	output LED2,
	output LED3
);

    reg [1:0] clk_div = 0;
    wire clk_12;

    always @(posedge clk) begin
            clk_div <= clk_div + 2'b1;
    end

    assign clk_12 = clk_div[1];

	// reset generator waits > 10us
	reg [7:0] reset_cnt;
	reg reset_n;
	initial
        reset_cnt <= 8'h00;

	always @(posedge clk_12)
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
	wire [7:0] gpio_o, gpio_i;
	assign gpio_i = 8'h00;
	tst_6502 u6502(
		.clk(clk_12),
		.reset_n(reset_n),

		.gpio_o(gpio_o),
		.gpio_i(gpio_i),

		.RX(RX),
		.TX(TX)
	);

	// drive LEDs from GPIO
	assign {LED1,LED2,LED3} = gpio_o[7:5];
endmodule

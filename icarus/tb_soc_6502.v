// tb_tst_6502.v - testbench for test 6502 core
// 02-11-19 E. Brombaugh

//`timescale 1ns/1ps

module tb_soc_6502;
    reg clk;
    reg reset_n;
    reg RX;
    wire TX;
	reg [7:0] gpio_a_i;
	wire [7:0] gpio_a_o;
	reg [7:0] gpio_b_i;
	wire [7:0] gpio_b_o;
	reg flag_n;
	wire pc_n;
	reg sp_in;
	wire sp_out;
	reg cnt_in;
	wire cnt_out;

    // clock source
    always
        #2 clk = ~clk;

    // reset
    initial
    begin
`ifndef VERIFICATION
  		$dumpfile("tb_soc_6502.lxt");
		$dumpvars/*(1, uut)*/;
`endif

        // init regs
        clk = 1'b0;
        reset_n = 1'b0;

        // release reset
        #10
        reset_n = 1'b1;

`ifdef VERIFICATION
`ifdef VERIFICATION_6502
		#338380000 $finish;
`endif
`ifdef VERIFICATION_65C02
		#239610000 $finish;
`endif
`ifdef DECIMAL_TEST_65C02
		#34350000 $finish;
`endif
`ifdef VERIFICATION_65CE02
		#12830000 $finish;
`endif
`else       // stop after 1 sec
		#100000 $finish;
`endif
    end

    // Unit under test
    soc_6502 uut(
        .clk(clk),              // clock
        .reset_n(reset_n),      // Low-true reset
        .RX(RX),                // UART RX
        .TX(TX),                // UART TX
        .gpio_a_i(gpio_a_i),    // gpio a input
        .gpio_a_o(gpio_a_o),    // gpio a output
        .gpio_b_i(gpio_b_i),    // gpio b input
        .gpio_b_o(gpio_b_o),    // gpio b output
        .flag_n(flag_n),        // low-true FLAG
	    .pc_n(pc_n),            // low-true PC
	    .sp_in(sp_in),          // Serial Port in
        .sp_out(sp_out),        // Serual Port out
        .cnt_in(cnt_in),        // CNT in
        .cnt_out(cnt_out)       // CNT out
    );
endmodule

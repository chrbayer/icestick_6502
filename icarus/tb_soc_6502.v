// tb_tst_6502.v - testbench for test 6502 core
// 02-11-19 E. Brombaugh

//`timescale 1ns/1ps

module tb_soc_6502;
    reg clk;
    reg reset_n;
	wire [7:0] gpio_o;
	reg [7:0] gpio_i;

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
        .gpio_o(gpio_o),        // gpio output
        .gpio_i(gpio_i)         // gpio input
    );
endmodule

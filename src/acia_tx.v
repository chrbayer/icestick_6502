// acia_tx.v - serial transmit submodule
// 06-02-19 E. Brombaugh

module acia_tx #(
	// default sym rate counter for 9600bps @ 4MHz clk
    parameter SCW = 9,
	parameter sym_cnt = 417
) (
	input clk,				// system clock
	input pclk,				// peripheral clock
	input reset_n,			// system reset
	input [7:0] tx_dat,		// transmit data byte
	input tx_start,			// trigger transmission
	output tx_serial,		// tx serial output
	output reg tx_busy		// tx is active (not ready)
);

	// transmit machine
	reg [8:0] tx_sr;
	reg [3:0] tx_bcnt;
	reg [SCW-1:0] tx_rcnt;
	always @(posedge clk)
	begin
		if(~reset_n)
		begin
			tx_sr <= 9'h1ff;
			tx_bcnt <= 4'h0;
			tx_rcnt <= {SCW{1'b0}};
			tx_busy <= 1'b0;
		end
		else if(pclk)
		begin
			if(~tx_busy)
			begin
				if(tx_start)
				begin
					// start transmission
					tx_busy <= 1'b1;
					tx_sr <= {tx_dat,1'b0};
					tx_bcnt <= 4'd9;
					tx_rcnt <= sym_cnt[SCW-1:0];
				end
			end
			else
			begin
				if(~|tx_rcnt)
				begin
					// shift out next bit and restart
					tx_sr <= {1'b1,tx_sr[8:1]};
					tx_bcnt <= tx_bcnt - 1;
					tx_rcnt <= sym_cnt[SCW-1:0];

					if(~|tx_bcnt)
					begin
						// done - return to inactive state
						tx_busy <= 1'b0;
					end
				end
				else
					tx_rcnt <= tx_rcnt - 1;
			end
		end
	end

	// hook up output
	assign tx_serial = tx_sr[0];
endmodule

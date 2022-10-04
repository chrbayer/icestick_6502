module sram (
    input wire reset_n,
	input wire clk,
    input wire write,
    input wire read,
    input wire [15:0] data_write,       // the data to write
    output wire [15:0] data_read,       // the data that's been read
    input wire [17:0] address,          // address to write to
    output wire ready,                  // high when ready for next operation
    output wire data_pins_out_en,       // when to switch between in and out on data pins

    // SRAM pins
    output wire [17:0] address_pins,    // address pins of the SRAM
    input  wire [15:0] data_pins_in,
    output wire [15:0] data_pins_out,
    output wire OE,                     // output enable - low to enable
    output wire WE,                     // write enable - low to enable
    output wire CS                      // chip select - low to enable
);

    localparam STATE_IDLE = 0;
    localparam STATE_WRITE = 1;
    localparam STATE_WRITE_SETUP = 4;
    localparam STATE_READ_SETUP = 2;
    localparam STATE_READ = 3;

    reg output_enable;
    reg write_enable;
    reg chip_select;

    reg [2:0] state;
    reg [15:0] data_read_reg;
    reg [15:0] data_write_reg;
    reg [17:0] address_reg;

    assign data_pins_out_en = (state == STATE_WRITE) ? 1 : 0; // turn on output pins when writing data
    assign address_pins = address_reg;
    assign data_pins_out = data_write_reg;
    assign data_read = data_read_reg;
    assign OE = output_enable;
    assign WE = write_enable;
    assign CS = chip_select;

    assign ready = (reset_n && state == STATE_IDLE) ? 1 : 0;

    initial begin
        state <= STATE_IDLE;
        output_enable <= 1;
        chip_select <= 1;
        write_enable <= 1;
    end

	always @(posedge clk) begin
        if( ~reset_n ) begin
            state <= STATE_IDLE;
            output_enable <= 1;
            chip_select <= 1;
            write_enable <= 1;
        end
        else begin
            casez(state)
                STATE_IDLE: begin
                    output_enable <= 1;
                    chip_select <= 1;
                    write_enable <= 1;
                    if(write) state <= STATE_WRITE_SETUP;
                    else if(read) state <= STATE_READ_SETUP;
                end
                STATE_WRITE_SETUP: begin
                    write_enable <= 0;
                    chip_select <= 0;
                    data_write_reg <= data_write;
                    address_reg <= address;
                    state <= STATE_WRITE;
                end
                STATE_WRITE: begin
                    state <= STATE_IDLE;
                end
                STATE_READ_SETUP: begin
                    output_enable <= 0;
                    chip_select <= 0;
                    address_reg <= address;
                    state <= STATE_READ;
                end
                STATE_READ: begin
                    data_read_reg <= data_pins_in;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule

`ifdef SIM
module sram_chip #(
    parameter ADDR_WIDTH = 18,
    parameter DATA_WIDTH = 16,
    parameter DEPTH      = 262144
) (
    input                   clk,
    input [ADDR_WIDTH-1:0]  addr,
    input [DATA_WIDTH-1:0]  data_in,
    output [DATA_WIDTH-1:0] data_out,
    input                   cs,
    input                   we,
    input                   oe
);

    reg [DATA_WIDTH-1:0] tmp_data;
    reg [DATA_WIDTH-1:0] mem[0:DEPTH-1];

    reg addr_written;

    always @(posedge clk)
        if (~cs & ~we & ~addr_written) begin
            mem[addr] <= data_in;
            addr_written <= 1;
        end

    always @(negedge we)
        addr_written = 0;

    always @(posedge clk)
        tmp_data <= mem[addr];

    assign data_out = ~cs & ~oe & we ? tmp_data : 'hz;
endmodule
`endif

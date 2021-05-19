/*
 * verilog model of 65C02 CPU.
 *
 * Based on original 6502 "Arlet 6502 Core" by Arlet Ottens
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 *
 * Feel free to use this code in any project (commercial or not), as long as you
 * keep this message, and the copyright notice. This code is provided "as is",
 * without any warranties of any kind.
 *
 * Support for 65C02 instructions and addressing modes by David Banks and Ed Spittles
 *
 * (C) 2016 David Banks and Ed Spittles
 *
 * Feel free to use this code in any project (commercial or not), as long as you
 * keep this message, and the copyright notice. This code is provided "as is",
 * without any warranties of any kind.
 *
 */

/*
 * Note that not all 6502 interface signals are supported (yet).  The goal
 * is to create an Acorn Atom model, and the Atom didn't use all signals on
 * the main board.
 *
 * The data bus is implemented as separate read/write buses. Combine them
 * on the output pads if external memory is required.
 */

/*
 * Two things were needed to correctly implement 65C02 NOPs
 * 1. Ensure the microcode state machine uses an appropriate addressing mode for the opcode length
 * 2. Ensure there are no side-effects (e.g. register updates, memory stores, etc)
 *
 * If IMPLEMENT_NOPS is defined, the state machine is modified accordingly.
 */

`define IMPLEMENT_NOPS

/*
 * Two things were needed to correctly implement 65C02 BCD arithmentic
 * 1. The Z flag needs calculating over the BCD adjusted ALU output
 * 2. The N flag needs calculating over the BCD adjusted ALU output
 *
 * If IMPLEMENT_CORRECT_BCD_FLAGS is defined, this additional logic is added
 */

`define IMPLEMENT_CORRECT_BCD_FLAGS

module cpu_65ce02( clk, reset, AB, DI, DO, WE, IRQ, NMI, RDY );

input clk;              // CPU clock
input reset;            // reset signal
output reg [15:0] AB;   // address bus
input [7:0] DI;         // data in, read bus
output [7:0] DO;        // data out, write bus
output WE;              // write enable
input IRQ;              // interrupt request
input NMI;              // non-maskable interrupt request
input RDY;              // Ready signal. Pauses CPU when RDY=0

/*
 * internal signals
 */

reg  [15:0] PC;         // Program Counter
reg  [7:0] ABL;         // Address Bus Register LSB
reg  [7:0] ABH;         // Address Bus Register MSB
reg  [7:0] ADL;         // Address Bus Data Register LSB
reg  [7:0] ADH;         // Address Bus Data Register MSB
wire [7:0] ADD;         // Adder Hold Register (registered in ALU)
reg  [7:0] SPL;         // Stack Register LSB
//reg  [7:0] SPH;         // Stack Register MSB

reg  [7:0] DIHOLD;      // Hold for Data In
reg  DIHOLD_valid;      //
wire [7:0] DIMUX;       //

reg  [7:0] IRHOLD;      // Hold for Instruction register
reg  IRHOLD_valid;      // Valid instruction in IRHOLD

reg  [7:0] AXYZB[5:0];  // A, X, Y, Z, B register file

reg  C = 0;             // carry flag (init at zero to avoid X's in ALU sim)
reg  Z = 0;             // zero flag
reg  I = 0;             // interrupt flag
reg  D = 0;             // decimal flag
reg  V = 0;             // overflow flag
reg  N = 0;             // negative flag
wire AZ;                // ALU Zero flag
wire AZ1;               // ALU Zero flag (BCD adjusted)
reg  AZ2;               // ALU Second Zero flag, set using TSB/TRB semantics
wire AV;                // ALU overflow flag
wire AN;                // ALU negative flag
wire AN1;               // ALU negative flag (BCD adjusted)
wire HC;                // ALU half carry

reg  [7:0] AI;          // ALU Input A
reg  [7:0] BI;          // ALU Input B
wire [7:0] DI;          // Data In
wire [7:0] IR;          // Instruction register
reg  [7:0] DO;          // Data Out
wire [7:0] AO;          // ALU output after BCD adjustment
reg  WE;                // Write Enable
reg  CI;                // Carry In
wire CO;                // Carry Out
wire [7:0] PCH = PC[15:8];
wire [7:0] PCL = PC[7:0];

reg NMI_edge = 0;       // captured NMI edge

reg [2:0] regsel;                       // Select A, X, Y or S register
wire [7:0] regfile = regsel == SEL_SPL ? SPL : AXYZB[regsel];    // Selected register output

parameter
        SEL_A    = 3'd0,
        SEL_X    = 3'd1,
        SEL_Y    = 3'd2,
        SEL_Z    = 3'd3,
        SEL_B    = 3'd4,
        SEL_SPH  = 3'd5,
        SEL_SPL  = 3'd6;

/*
 * define some signals for watching in simulator output
 */


`ifdef SIM
wire [7:0]   A  = AXYZB[SEL_A];           // Accumulator
wire [7:0]   X  = AXYZB[SEL_X];           // X register
wire [7:0]   Y  = AXYZB[SEL_Y];           // Y register
wire [7:0] ZREG = AXYZB[SEL_Z];           // Y register
wire [7:0]   B  = AXYZB[SEL_B];           // Base register
`endif

wire [7:0] SPH = AXYZB[SEL_SPH];

wire [7:0] P = { N, V, 2'b11, D, I, Z, C };

/*
 * instruction decoder/sequencer
 */

reg [5:0] state;

/*
 * control signals
 */

reg PC_inc;             // Increment PC
reg [15:0] PC_temp;     // intermediate value of PC
reg [7:0] SPL_temp;     // intermediate value of stack pointer
reg SP_inc;             // Increment stack pointer
reg SP_dec;             // Decrement stack pointer

reg [3:0] src_reg;      // source register index
reg [3:0] dst_reg;      // destination register index

reg [3:0] index_sel;    // index register X, Y or Z
reg load_reg;           // loading a register (A, X, Y, S) in this instruction
reg inc;                // increment
reg write_back;         // set if memory is read/modified/written
reg load_only;          // LDA/LDX/LDY/LDZ instruction
reg store;              // doing store (STA/STX/STY)
reg adc_sbc;            // doing ADC/SBC
reg compare;            // doing CMP/CPY/CPX
reg shift;              // doing shift/rotate instruction
reg rotate;             // doing rotate (no shift)
reg backwards;          // backwards branch
reg cond_true;          // branch condition is true
reg [3:0] cond_code;    // condition code bits from instruction
reg [3:0] bit_code;     // bit position and polarity for SMB/RMB/BBS/BBR
reg bit_cond_true;      // branch condition is true in case of BBS/BBR
reg shift_right;        // Instruction ALU shift/rotate right
reg arith_shift;        // Instruction ALU shift arithmetic right
reg alu_shift_right;    // Current cycle shift right enable
reg alu_arith_shift;    // Current cycle arithmetic shift right enable
reg [3:0] op;           // Main ALU operation for instruction
reg [3:0] alu_op;       // Current cycle ALU operation
reg adc_bcd;            // ALU should do BCD style carry
reg adj_bcd;            // results should be BCD adjusted

/*
 * some flip flops to remember we're doing special instructions. These
 * get loaded at the DECODE state, and used later
 */
reg long_branch;        // doing a 16 bit range branch
reg trb_ins;            // doing TRB instruction
reg txb_ins;            // doing TSB/TRB instruction
reg xmb_ins;            // doing SMB/RMB instruction
reg bbx_ins;            // doing BBS/BBR instruction
reg ind_jsr;            // doing an indirect jump to subroutine IND and IND, X
reg ind_x_jsr;          // doing an indirect jump with X register offset to subroutine IND, X
reg bit_ins;            // doing BIT instruction
reg bit_ins_nv;         // doing BIT instruction that will update the n and v flags (i.e. not BIT imm)
reg plp;                // doing PLP instruction
reg php;                // doing PHP instruction
reg clc;                // clear carry
reg sec;                // set carry
reg cld;                // clear decimal
reg sed;                // set decimal
reg cli;                // clear interrupt
reg sei;                // set interrupt
reg clv;                // clear overflow
reg brk;                // doing BRK
reg neg;                // doing NEG
reg bsr;                // doing BSR instruction
reg rti;                // doing RTI
reg rtn;                // doing RTN

reg res;                // in reset

/*
 * ALU operations
 */

parameter
        OP_OR  = 4'b1100,
        OP_AND = 4'b1101,
        OP_EOR = 4'b1110,
        OP_ADD = 4'b0011,
        OP_SUB = 4'b0111,
        OP_ROL = 4'b1011,
        OP_A   = 4'b1111;

/*
 * Microcode state machine. Basically, every addressing mode has its own
 * path through the state machine. Additional information, such as the
 * operation, source and destination registers are decoded in parallel, and
 * kept in separate flops.
 */

parameter
    ABS0   = 6'd0,  // ABS     - fetch LSB
    ABS1   = 6'd1,  // ABS     - fetch MSB
    ABSX0  = 6'd2,  // ABS, X  - fetch LSB and send to ALU (+X)
    ABSX1  = 6'd3,  // ABS, X  - fetch MSB and send to ALU (+Carry)
    BRA0   = 6'd4,  // Branch  - fetch offset and send to 16 bit ALU
    BRA0B  = 6'd5,  // Branch  - fetch offset and send to 16 bit ALU, high byte
    BRA1   = 6'd6,  // Branch  - fetch opcode
    BRK0   = 6'd7,  // BRK/IRQ - push PCH, send S to ALU (-1)
    BRK1   = 6'd8,  // BRK/IRQ - push PCL, send S to ALU (-1)
    BRK2   = 6'd9,  // BRK/IRQ - push P, send S to ALU (-1)
    BRK3   = 6'd10, // BRK/IRQ - write S, and fetch @ fffe
    DECODE = 6'd11, // IR is valid, decode instruction, and write prev reg
    FETCH  = 6'd12, // fetch next opcode, and perform prev ALU op
    INDX0  = 6'd13, // (BP,X)  - fetch BP address, and send to ALU (+X)
    INDX1  = 6'd14, // (BP,X)  - fetch LSB at BP+X, calculate BP+X+1
    INDX2  = 6'd15, // (BP,X)  - fetch MSB at BP+X+1
    INDX3  = 6'd16, // (BP,X)  - fetch data
    INDY0  = 6'd17, // (BP),Y/Z  - fetch BP address, and send BP to ALU (+1)
    INDY1  = 6'd18, // (BP),Y/Z  - fetch at BP+1, and send LSB to ALU (+Y)
    INDY2  = 6'd19, // (BP),Y/Z  - fetch data, and send MSB to ALU (+Carry)
    JMP0   = 6'd20, // JMP     - fetch PCL and hold
    JMP1   = 6'd21, // JMP     - fetch PCH
    JMPI0  = 6'd22, // JMP IND - fetch LSB and send to ALU for delay (+0)
    JMPI1  = 6'd23, // JMP IND - fetch MSB, proceed with JMP0 state
    JSR0   = 6'd24, // JSR     - push PCH, save LSB, send S to ALU (-1)
    JSR1   = 6'd25, // JSR     - push PCL, send S to ALU (-1)
    JSR2   = 6'd26, // JSR     - write S
    JSR3   = 6'd27, // JSR     - fetch MSB
    PULL0  = 6'd28, // PLP/PLA/PLX/PLY/PLZ - fetch data from stack, write S
    PULL1  = 6'd29, // PLP/PLA/PLX/PLY/PLZ - prefetch op, but don't increment PC
    PUSH0  = 6'd30, // PHP/PHA/PHX/PHY/PHZ - send A to ALU (+0)
    READ   = 6'd31, // Read memory for read/modify/write (INC, DEC, shift)
    RDONLY = 6'd32, // Read memory for BBS/BBR
    REG    = 6'd33, // Read register for reg-reg transfers
    RTI0   = 6'd34, // RTI     - read P from stack
    RTI1   = 6'd35, // RTI     - read PCL from stack
    RTS0   = 6'd36, // RTS/RTN - read PCL from stack
    RTS1   = 6'd37, // RTS/RTN - write PCL to ALU, read PCH
    RTS2   = 6'd38, // RTS/RTN - load PC and increment
    RTS3   = 6'd39, // RTN     - Add value stored in ADD16 to S with ALU
    RTS4   = 6'd40, // RTN     - write S
    WRITE  = 6'd41, // Write memory for read/modify/write
    BP0    = 6'd42, // Z-page  - fetch BP address
    BPX0   = 6'd43, // BP, X   - fetch BP, and send to ALU (+X)
    BPX1   = 6'd44, // BP, X   - load from memory
    JMPIX0 = 6'd45, // JMP (,X)- fetch LSB and send to ALU (+X)
    JMPIX1 = 6'd46; // JMP (,X)- fetch MSB and send to ALU (+Carry)

`ifdef SIM

/*
 * easy to read names in simulator output
 */
reg [8*6-1:0] statename;

always @*
    case( state )
            DECODE: statename = "DECODE";
            REG:    statename = "REG";
            BP0:    statename = "BP0";
            BPX0:   statename = "BPX0";
            BPX1:   statename = "BPX1";
            ABS0:   statename = "ABS0";
            ABS1:   statename = "ABS1";
            ABSX0:  statename = "ABSX0";
            ABSX1:  statename = "ABSX1";
            INDX0:  statename = "INDX0";
            INDX1:  statename = "INDX1";
            INDX2:  statename = "INDX2";
            INDX3:  statename = "INDX3";
            INDY0:  statename = "INDY0";
            INDY1:  statename = "INDY1";
            INDY2:  statename = "INDY2";
            READ:   statename = "READ";
            RDONLY: statename = "RDONLY";
            WRITE:  statename = "WRITE";
            FETCH:  statename = "FETCH";
            PUSH0:  statename = "PUSH0";
            PULL0:  statename = "PULL0";
            PULL1:  statename = "PULL1";
            JSR0:   statename = "JSR0";
            JSR1:   statename = "JSR1";
            JSR2:   statename = "JSR2";
            JSR3:   statename = "JSR3";
            RTI0:   statename = "RTI0";
            RTI1:   statename = "RTI1";
            RTS0:   statename = "RTS0";
            RTS1:   statename = "RTS1";
            RTS2:   statename = "RTS2";
            RTS3:   statename = "RTS3";
            RTS4:   statename = "RTS4";
            BRK0:   statename = "BRK0";
            BRK1:   statename = "BRK1";
            BRK2:   statename = "BRK2";
            BRK3:   statename = "BRK3";
            BRA0:   statename = "BRA0";
            BRA0B:  statename = "BRA0B";
            BRA1:   statename = "BRA1";
            JMP0:   statename = "JMP0";
            JMP1:   statename = "JMP1";
            JMPI0:  statename = "JMPI0";
            JMPI1:  statename = "JMPI1";
            JMPIX0: statename = "JMPIX0";
            JMPIX1: statename = "JMPIX1";

    endcase

//always @( PC )
//      $display( "%t, PC:%04x IR:%02x A:%02x X:%02x Y:%02x S:%02x C:%d Z:%d V:%d N:%d P:%02x", $time, PC, IR, A, X, Y, S, C, Z, V, N, P );

`endif



/*
 * Program Counter Increment/Load. First calculate the base value in
 * PC_temp.
 */
always @*
    case( state )
        DECODE:         if( (~I & IRQ) | NMI_edge )
                            PC_temp = { ABH, ABL };
                        else
                            PC_temp = PC;

        JMP1,
        JMPI1:          PC_temp = { DIMUX, ADD };

        JSR3,
        RTS2:           PC_temp = { DIMUX, ADL };

        BRA1:           PC_temp = long_branch ? { ADD, ADL } : { ADH + (CO & ~backwards) - (backwards & ~CO), ADD };

        JMPIX1:         PC_temp = { DIMUX + CO, ADD };

        BRK2:           PC_temp =      res ? 16'hfffc :
                                  NMI_edge ? 16'hfffa : 16'hfffe;

        default:        PC_temp = PC;
    endcase

/*
 * Determine wether we need PC_temp, or PC_temp + 1
 */
always @*
    case( state )
        DECODE:         if( (~I & IRQ) | NMI_edge )
                            PC_inc = 0;
                        else
                            PC_inc = 1;

        ABS0,
        JMPIX0,
        JMPIX1,
        ABSX0,
        FETCH,
        RDONLY,
        BRA0,
        BRA0B,
        BRA1,
        BRK3,
        JMPI1,
        JMP1,
        RTS4:           PC_inc = 1;

        RTS2:           PC_inc = rti ? 0 : 1;

        default:        PC_inc = 0;
    endcase

/*
 * Stack pointer Increment/Decrement/Load. First calculate the base value in
 * SPL_temp.
 */
always @*
    case ( state )
        DECODE:         SPL_temp = ( regsel == SEL_SPL && write_register ) ? AO : SPL;

        default:        SPL_temp = SPL;
    endcase

/*
 * Determine wether we need SP_inc to increment stack pointer
 */
always @*
    case( state )
        PULL0,
        RTS0,
        RTS1,
        RTI0,
        RTI1:           SP_inc = 1;

        default:        SP_inc = 0;
    endcase

/*
 * Determine wether we need SP_dec to decrement stack pointer
 */
always @*
    case( state )
        BRK0,
        BRK1,
        BRK2,
        PUSH0,
        JSR0,
        JSR1:           SP_dec = 1;

        default:        SP_dec = 0;
    endcase

/*
 * Set new PC
 */
always @(posedge clk)
    if( RDY && ~reset )
        PC <= PC_temp + PC_inc;

/*
 * Set new stack pointer
 */
always @(posedge clk)
    if( RDY )
        SPL <= SPL_temp + SP_inc - SP_dec;

/*
 * Address Generator
 */

always @*
    case( state )
        INDX3,
        JMP1,
        JMPI1,
        ABS1:           AB = { DIMUX, ADD };

        INDY2,
        JMPIX1,
        ABSX1:          AB = { DIMUX + CO, ADD };

        BRA1:           AB = long_branch ? { ADD, ADL } : { ADH + (CO & ~backwards) - (backwards & ~CO), ADD };

        JSR0,
        JSR1,
        BRK0,
        BRK1,
        BRK2,
        PUSH0:          AB = { SPH, SPL };

        RTI0,
        RTI1,
        RTS0,
        RTS1,
        PULL0:          AB = { SPH, SPL + 8'd1 };

        INDY1,
        INDX1,
        BPX1,
        INDX2:          AB = { B, ADD };

        BP0,
        INDY0:          AB = { B, DIMUX };

        REG,
        READ,
        WRITE:          AB = { ABH, ABL };

        default:        AB = PC;
    endcase

/*
 * ABH/ABL pair is used for registering previous address bus state.
 * This can be used to keep the current address, freeing up the original
 * source of the address, such as the ALU or DI.
 */
always @(posedge clk)
    if( state != PUSH0 && RDY &&
        state != PULL0 && state != PULL1 )
    begin
        ABL <= AB[7:0];
        ABH <= AB[15:8];
    end

/*
 * ADH/ADL is a helper address register pair.
 */

always @(posedge clk)
    case( state )
        BRA0:   ADH <= PCH;

        BRA0B:  ADL <= ADD;

        JSR0,
        RTS1:   ADL <= DIMUX;
    endcase

/*
 * Data Out MUX
 */
always @*
    case( state )
        WRITE:   DO = ADD;

        JSR0,
        BRK0:    DO = PCH;

        JSR1,
        BRK1:    DO = PCL;

        PUSH0:   DO = php ? P : regfile;

        BRK2:    DO = (IRQ | NMI_edge) ? (P & 8'b1110_1111) : P;

        default: DO = regfile;
    endcase

/*
 * Write Enable Generator
 */

always @*
    case( state )
        BRK0,
        BRK1,
        BRK2,
        JSR0,
        JSR1,
        PUSH0,
        WRITE:   WE = 1;

        INDX3,  // only if doing a STA, STX or STY
        INDY2,
        ABSX1,
        ABS1,
        BPX1,
        BP0:     WE = store;

        default: WE = 0;
    endcase

/*
 * register file, contains A, X, Y and S (stack pointer) registers. At each
 * cycle only 1 of those registers needs to be accessed, so they combined
 * in a small memory, saving resources.
 */

reg write_register;             // set when register file is written

always @*
    case( state )
        DECODE: write_register = load_reg & ~plp;

       default: write_register = 0;
    endcase

/*
 * BCD adjust logic
 */

always @(posedge clk)
    adj_bcd <= adc_sbc & D;     // '1' when doing a BCD instruction

reg [3:0] ADJL;
reg [3:0] ADJH;

// adjustment term to be added to ADD[3:0] based on the following
// adj_bcd: '1' if doing ADC/SBC with D=1
// adc_bcd: '1' if doing ADC with D=1
// HC     : half carry bit from ALU
always @* begin
    casex( {adj_bcd, adc_bcd, HC} )
         3'b0xx: ADJL = 4'd0;   // no BCD instruction
         3'b100: ADJL = 4'd10;  // SBC, and digital borrow
         3'b101: ADJL = 4'd0;   // SBC, but no borrow
         3'b110: ADJL = 4'd0;   // ADC, but no carry
         3'b111: ADJL = 4'd6;   // ADC, and decimal/digital carry
    endcase
end

// adjustment term to be added to ADD[7:4] based on the following
// adj_bcd: '1' if doing ADC/SBC with D=1
// adc_bcd: '1' if doing ADC with D=1
// CO     : carry out bit from ALU
always @* begin
    casex( {adj_bcd, adc_bcd, CO} )
         3'b0xx: ADJH = 4'd0;   // no BCD instruction
         3'b100: ADJH = 4'd10;  // SBC, and digital borrow
         3'b101: ADJH = 4'd0;   // SBC, but no borrow
         3'b110: ADJH = 4'd0;   // ADC, but no carry
         3'b111: ADJH = 4'd6;   // ADC, and decimal/digital carry
    endcase
end

assign AO = { ADD[7:4] + ADJH, ADD[3:0] + ADJL };

`ifdef IMPLEMENT_CORRECT_BCD_FLAGS

assign AN1 = AO[7];
assign AZ1 = ~|AO;

`else

assign AN1 = AN;
assign AZ1 = AZ;

`endif

/*
 * write to a register. Usually this is the (BCD corrected) output of the
 * ALU, but in case of the JSR0 we use the S register to temporarily store
 * the PCL. This is possible, because the S register itself is stored in
 * the ALU during those cycles.
 */
always @(posedge clk)
    if( write_register & RDY )
        if ( regsel != SEL_SPL )
            AXYZB[regsel] <= AO;

/*
 * register select logic. This determines which of the A, X, Y or
 * S registers will be accessed.
 */

always @*
    case( state )
        INDY1,
        INDX0,
        BPX0,
        JMPIX0,
        ABSX0  : regsel = index_sel;

        DECODE : regsel = dst_reg;

        default: regsel = src_reg;
    endcase

/*
 * ALUs
 */

alu_65ce02 ualu( .clk(clk),
         .op(alu_op),
         .right(alu_shift_right),
         .arith(alu_arith_shift),
         .AI(AI),
         .BI(BI),
         .CI(CI),
         .BCD(adc_bcd & (state == FETCH)),
         .CO(CO),
         .OUT(ADD),
         .V(AV),
         .Z(AZ),
         .N(AN),
         .HC(HC),
         .RDY(RDY) );

/*
 * Select current ALU operation
 */

always @*
    case( state )
        READ:   alu_op = op;

        FETCH,
        REG :   alu_op = op;

        DECODE,
        ABS1:   alu_op = 1'bx;

     default:   alu_op = OP_ADD;
    endcase

/*
 * Determine shift right signal to ALU
 */

always @*
    if( state == FETCH || state == REG || state == READ )
        { alu_shift_right, alu_arith_shift } = { shift_right, arith_shift };
    else
        { alu_shift_right, alu_arith_shift } = 2'b00;

/*
 * Sign extend branch offset.
 */

always @(posedge clk)
    if( RDY )
        backwards <= DIMUX[7];

/*
 * ALU A Input MUX
 */

always @*
    case( state )
        INDX1:  AI = ADD;

        REG:    AI = neg ? 8'h00 : regfile;

        BPX0,
        INDX0,
        JMPIX0,
        ABSX0,
        RTS3,
        INDY1:  AI = regfile;

        READ:   AI = DIMUX;

        BRA0:   AI = PCL;

        BRA0B:  AI = ABH;

        FETCH:  AI = load_only ? 0 : regfile;

        DECODE,
        ABS1:   AI = 8'hxx;     // don't care

        default:  AI = 0;
    endcase

/*
 * ALU B Input mux
 */

always @*
    case( state )
        INDX1:  BI = 8'h00;

        RTS3:   BI = 0;//ADD16[7:0];

        REG:    BI = neg ? regfile : 8'h00;

        READ:   BI = xmb_ins ? (bit_code[3] ? 8'h01 << bit_code[2:0] : ~(8'h01 << bit_code[2:0])) : (txb_ins ? (trb_ins ? ~regfile : regfile) : 8'h00);

        DECODE,
        ABS1:   BI = 8'hxx;

         default:       BI = DIMUX;
    endcase

/*
 * ALU CI (carry in) mux
 */

always @*
    case( state )
        BRA0B,
        INDY2,
        JMPIX1,
        ABSX1:  CI = CO;

        DECODE,
        ABS1:   CI = 1'bx;

        READ,
        REG:    CI = rotate ? C :
                     shift ? 0 : inc;

        FETCH:  CI = rotate  ? C :
                     compare ? 1 :
                     (shift | load_only) ? 0 : C;

        INDY0,
        INDX1:  CI = 1;

        default:        CI = 0;
    endcase

/*
 * Processor Status Register update
 *
 */

/*
 * Update C flag when doing ADC/SBC, shift/rotate, compare
 */
always @(posedge clk )
    if( shift && state == WRITE )
        C <= CO;
    else if( state == RTI1 )
        C <= DIMUX[0];
    else if( ~write_back && state == DECODE ) begin
        if( adc_sbc | shift | compare )
            C <= CO;
        else if( plp )
            C <= ADD[0];
        else begin
            if( sec ) C <= 1;
            if( clc ) C <= 0;
        end
    end

/*
 * Special Z flag got TRB/TSB
 */
always @(posedge clk)
    AZ2 <= ~|(AI & regfile);

/*
 * Update Z, N flags when writing A, X, Y, Memory, or when doing compare
 */

always @(posedge clk)
    if( state == WRITE && ~xmb_ins )
        Z <= txb_ins ? AZ2 : AZ1;
    else if( state == RTI1 )
        Z <= DIMUX[1];
    else if( state == DECODE ) begin
        if( plp )
            Z <= ADD[1];
        else if( (load_reg & ((regsel != SEL_SPL) & (regsel != SEL_SPH))) | compare | bit_ins )
            Z <= AZ1;
    end

always @(posedge clk)
    if( state == WRITE && ~txb_ins && ~xmb_ins )
        N <= AN1;
    else if( state == RTI1 )
        N <= DIMUX[7];
    else if( state == DECODE ) begin
        if( plp )
            N <= ADD[7];
        else if( (load_reg & ((regsel != SEL_SPL) & (regsel != SEL_SPH))) | compare )
            N <= AN1;
    end else if( state == FETCH && bit_ins_nv )
        N <= DIMUX[7];

/*
 * Update I flag
 */

always @(posedge clk)
    if( state == BRK2 )
        I <= 1;
    else if( state == RTI1 )
        I <= DIMUX[2];
    else if( state == REG ) begin
        if( sei ) I <= 1;
        if( cli ) I <= 0;
    end else if( state == DECODE )
        if( plp ) I <= ADD[2];

/*
 * Update D flag
 */
always @(posedge clk )
    if( state == BRK2 )
        D <= 0;
    else if( state == RTI1 )
        D <= DIMUX[3];
    else if( state == DECODE ) begin
        if( sed ) D <= 1;
        if( cld ) D <= 0;
        if( plp ) D <= ADD[3];
    end

/*
 * Update V flag
 */
always @(posedge clk )
    if( state == RTI1 )
        V <= DIMUX[6];
    else if( state == DECODE ) begin
        if( adc_sbc ) V <= AV;
        if( clv )     V <= 0;
        if( plp )     V <= ADD[6];
    end else if( state == FETCH && bit_ins_nv )
        V <= DIMUX[6];

/*
 * Instruction decoder
 */

/*
 * IR register/mux. Hold previous DI value in IRHOLD in PULL0 and PUSH0
 * states. In these states, the IR has been prefetched, and there is no
 * time to read the IR again before the next decode.
 */

always @(posedge clk )
    if( reset )
        IRHOLD_valid <= 0;
    else if( RDY ) begin
        if( state == PULL0 || state == PUSH0 ) begin
            IRHOLD <= DIMUX;
            IRHOLD_valid <= 1;
        end else if( state == DECODE )
            IRHOLD_valid <= 0;
    end

assign IR = (IRQ & ~I) | NMI_edge ? 8'h00 :
                     IRHOLD_valid ? IRHOLD : DIMUX;

always @(posedge clk )
    if( RDY )
        DIHOLD <= DI;

assign DIMUX = ~RDY ? DIHOLD : DI;

/* Default behavior is that of prior cpus */
parameter
        ZEROPAGE  = 8'h00,
        STACKPAGE = 8'h01,
        ZDEFAULT  = 8'h00;

/*
 * Microcode state machine
 */
always @(posedge clk or posedge reset)
    if( reset )
    begin
        state <= BRK0;
        AXYZB[SEL_B] <= ZEROPAGE;
        AXYZB[SEL_Z] <= ZDEFAULT;
        AXYZB[SEL_SPH] <= STACKPAGE;
    end
    else if( RDY ) case( state )
        DECODE  :
            casex ( IR )
                // TODO Review for simplifications as in verilog the first matching case has priority
                8'b0000_0000:   state <= BRK0;
                8'b0010_0000:   state <= JSR0;
                8'b0010_001x:   state <= JSR0;
                8'b0010_1100:   state <= ABS0;  // BIT abs
                8'b1x01_1100:   state <= ABS0;  // STZ abs, CPZ abs
                8'b000x_1100:   state <= ABS0;  // TSB/TRB
                8'b0100_0000:   state <= RTI0;  // RTI
                8'b0100_1100:   state <= JMP0;
                8'b0110_00x0:   state <= RTS0;  // RTS, RTN
                8'b0110_1100:   state <= JMPI0;
                8'b0111_1100:   state <= JMPIX0;
                8'b0x00_1000:   state <= PUSH0;
                8'b0x10_1000:   state <= PULL0;
                8'b0xx1_1000:   state <= REG;   // CLC, SEC, CLI, SEI
                8'b11x0_00x0:   state <= FETCH; // IMM
                8'b1x10_00x0:   state <= FETCH; // IMM
                8'b1010_0011:   state <= FETCH; // IMM
                8'b1xx0_1100:   state <= ABS0;  // X/Y abs
                8'b1010_1011:   state <= ABS0;  // Z abs
                8'b1xxx_1000:   state <= REG;   // DEY, TYA, ...
                8'bxxx0_0001:   state <= INDX0;
                8'b000x_0100:   state <= BP0;   // TSB/TRB
                8'bxxxx_x111:   state <= BP0;   // SMB/RMB/BBS/BBR
                8'bxxx0_01xx:   state <= BP0;
                8'b1101_0100:   state <= BP0;   // CPZ (D4)
                8'bxxx0_1001:   state <= FETCH; // IMM
                8'bxxx0_1101:   state <= ABS0;  // even D column
                8'bxxx0_1110:   state <= ABS0;  // even E column
                8'bxxx1_0000:   state <= BRA0;  // odd 0 column (Branches)
                8'b1000_0000:   state <= BRA0;  // BRA
                8'bxxx1_0011:   state <= BRA0;  // odd 0 column (Branches)
                8'b1000_0011:   state <= BRA0;  // BRA
                8'b0110_0011:   state <= JSR0;  // BRS
                8'bxxx1_0001:   state <= INDY0; // odd 1 column
                8'bxxx1_0010:   state <= INDY0; // (BP),Z odd 2 column
                8'bxxx1_01xx:   state <= BPX0;  // odd 4,5,6,7 columns
                8'bxxx1_1001:   state <= ABSX0; // odd 9 column
                8'bx011_1100:   state <= ABSX0; // C column BIT (3C), LDY (BC)
                8'bxxx1_1101:   state <= ABSX0; // odd D column
                8'bxxx1_1110:   state <= ABSX0; // odd E column
                8'b10x1_1011:   state <= ABSX0; // LDZ (BB), STX (9B)
                8'b1000_1011:   state <= ABSX0; // STY (8B)
                8'bx101_1010:   state <= PUSH0; // PHX/PHY
                8'b1101_1010:   state <= PUSH0; // PHZ
                8'bx111_1010:   state <= PULL0; // PLX/PLY
                8'b1111_1011:   state <= PULL0; // PLZ
                8'bx0xx_1010:   state <= REG;   // <shift> A, TXA, ...
                8'bxxx0_1010:   state <= REG;   // <shift> A, TXA, DEX, ...  NOP
                8'b0xxx_1011:   state <= REG;   // TSY, DEZ, ...
                8'b0100_001x:   state <= REG;   // NEG, ASR
`ifdef IMPLEMENT_NOPS
                8'bxxxx_x011:   state <= REG;   // (NOP1: 3/B column)
                8'bxxx0_0010:   state <= FETCH; // (NOP2: 2 column, 4 column handled correctly below)
                8'bx1x1_1100:   state <= ABS0;  // (NOP3: C column)
`endif
            endcase

        BP0     : state <= bbx_ins ? RDONLY : write_back ? READ : FETCH;

        BPX0    : state <= BPX1;
        BPX1    : state <= write_back ? READ : FETCH;

        ABS0    : state <= ABS1;
        ABS1    : state <= write_back ? READ : FETCH;

        ABSX0   : state <= ABSX1;
        ABSX1   : state <= write_back ? READ : FETCH;

        JMPIX0  : state <= JMPIX1;
        JMPIX1  : state <= JMP0;

        INDX0   : state <= INDX1;
        INDX1   : state <= INDX2;
        INDX2   : state <= INDX3;
        INDX3   : state <= FETCH;

        INDY0   : state <= INDY1;
        INDY1   : state <= INDY2;
        INDY2   : state <= FETCH;

        READ    : state <= WRITE;
        WRITE   : state <= FETCH;
        FETCH   : state <= DECODE;

        REG     : state <= DECODE;

        RDONLY  : state <= bbx_ins ? BRA0 : FETCH;

        PUSH0   : state <= DECODE;

        PULL0   : state <= PULL1;
        PULL1   : state <= DECODE;

        JSR0    : state <= JSR1;
        JSR1    : state <= JSR2;
        JSR2    : state <= bsr ? BRA0B : ind_jsr ? (ind_x_jsr ? JMPIX1 : JMPI1) : JSR3;
        JSR3    : state <= FETCH;

        RTI0    : state <= RTI1;
        RTI1    : state <= RTS1;

        RTS0    : state <= RTS1;
        RTS1    : state <= RTS2;
        RTS2    : state <= rtn ? RTS3 : FETCH;
        RTS3    : state <= RTS4;
        RTS4    : state <= DECODE;

        BRA0    : state <= long_branch ? BRA0B : ((bbx_ins & bit_cond_true) | (~bbx_ins & cond_true) ? BRA1 : DECODE);
        BRA0B   : state <= (bbx_ins & bit_cond_true) | (~bbx_ins & cond_true) ? BRA1 : DECODE;
        BRA1    : state <= DECODE;

        JMP0    : state <= JMP1;
        JMP1    : state <= DECODE;

        JMPI0   : state <= JMPI1;
        JMPI1   : state <= JMP0;

        BRK0    : state <= BRK1;
        BRK1    : state <= BRK2;
        BRK2    : state <= BRK3;
        BRK3    : state <= JMP0;

    endcase

/*
 * Additional control signals
 */

always @(posedge clk)
     if( reset )
         res <= 1;
     else if( state == DECODE )
         res <= 0;

always @(posedge clk)
     if( state == DECODE && RDY )
        casex( IR )
                8'b0xx1_0010,   // ORA, AND, EOR, ADC (zp)
                8'b1x11_0010,   // LDA, SBC (zp)
                8'b00xx_1010,   // ASLA, INCA, ROLA
                8'b01x0_1010,   // LSRA, RORA
                8'b0111_1010,   // PLY
                8'b0xxx_xx01,   // ORA, AND, EOR, ADC
                8'b100x_10x0,   // DEY, TYA, TXA, TXS
                8'b1010_xxx0,   // LDA/LDX/LDY
                8'b1010_0011,   // LDZ
                8'b1011_1010,   // TSX
                8'b1011_x1x0,   // LDX/LDY
                8'b1100_1010,   // DEX
                8'b1111_101x,   // PLX, PLZ
                8'b1x1x_xx01,   // LDA, SBC
                8'b1xx0_1000,   // INY, INX
                8'b0x10_1000,   // PLP, PLA
                8'b0xxx_1011,   // TSY, INZ, TYS, DEZ, TAZ, TAB, TZA, TBA
                8'b101x_1011,   // LDZ
                8'b0100_001x:   // NEG, ASR
                                load_reg <= 1;

                default:        load_reg <= 0;
        endcase

always @(posedge clk)
     if( state == DECODE && RDY )
        casex( IR )
                8'b1110_1000,   // INX
                8'b1100_1010,   // DEX
                8'b1111_1010,   // PLX
                8'b1010_0010,   // LDX imm
                8'b101x_x110,   // LDX
                8'b101x_1x10:   // LDX, TAX, TSX
                                dst_reg <= SEL_X;

                8'b0101_1011:   // TAB
                                dst_reg <= SEL_B;

                8'b1001_1010:   // TXS
                                dst_reg <= SEL_SPL;

                8'b0010_1011:   // TYS
                                dst_reg <= SEL_SPH;

                8'b1x00_1000,   // DEY, INY
                8'b0000_1011,   // TSY
                8'b0111_1010,   // PLY
                8'b101x_x100,   // LDY
                8'b1010_x000:   // LDY imm, TAY
                                dst_reg <= SEL_Y;

                8'b00x1_1011,   // DEZ, INZ
                8'b0100_1011,   // TAZ
                8'b1010_0011,   // LDZ imm
                8'b101x_1011,   // LDZ
                8'b1111_1011:   // PLZ
                                dst_reg <= SEL_Z;

                default:        dst_reg <= SEL_A;
        endcase

always @(posedge clk)
     if( state == DECODE && RDY )
        casex( IR )
                8'b1011_1010:   // TSX
                                src_reg <= SEL_SPL;

                8'b0000_1011:   // TSY
                                src_reg <= SEL_SPH;

                8'b100x_0110,   // STX
                8'b1001_1011,   // STX
                8'b1000_1110,   // STX
                8'b100x_1010,   // TXA, TXS
                8'b1110_xx00,   // INX, CPX
                8'b110x_1010:   // PHX, DEX
                                src_reg <= SEL_X;

                8'b100x_0100,   // STY
                8'b1000_1100,   // STY
                8'b1000_1011,   // STY
                8'b1001_1000,   // TYA
                8'b1100_xx00,   // CPY
                8'b0101_1010,   // PHY
                8'b0010_1011,   // TYS
                8'b1x00_1000:   // DEY, INY
                                src_reg <= SEL_Y;

                8'b011x_0100,   // STZ
                8'b1001_1110,   // STZ
                8'b00x1_1011,   // DEZ, INZ
                8'b0110_1011,   // TZA
                8'b1x01_1100,   // STZ, CPZ
                8'b1101_1011,   // PHZ
                8'b1100_0010,   // CPZ
                8'b1101_0100:   // CPZ
                                src_reg <= SEL_Z;

                8'b0111_1011:   // TBA
                                src_reg <= SEL_B;

                default:        src_reg <= SEL_A;
        endcase

always @(posedge clk)
     if( state == DECODE && RDY )
        casex( IR )
                8'bxxx1_0001,   // INDY
                8'b10x1_0110,   // LDX zp,Y / STX zp,Y
                8'b1011_1110,   // LDX abs,Y
                8'bxxx1_1001,   // abs, Y
                8'b1001_1011:   // STX abs,Y
                                index_sel <= SEL_Y;

                8'bxxx1_0010:   // INDZ
                                index_sel <= SEL_Z;

                default:        index_sel <= SEL_X;
        endcase


always @(posedge clk)
     if( state == DECODE && RDY )
        casex( IR )
                8'b1001_0010,   // STA (zp)
                8'b100x_x1x0,   // STX, STY, STZ abs, STZ abs,x
                8'b011x_0100,   // STZ zp STZ zp,x
                8'b100x_0x01,   // STA
                8'b100x_1101,   // STA
                8'b1001_1001,   // STA
                8'b100x_1011:   // STX, STY
                                store <= 1;

                default:        store <= 0;

        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )             // DMB: Checked for 65C02 NOP collisions
                8'b0xxx_x110,   // ASL, ROL, LSR, ROR
                8'b000x_x100,   // TSB/TRB
                8'bxxxx_0111,   // SMB/RMB
                8'b11xx_x110,   // DEC/INC
                8'b010x_0100:   // ASR
                                write_back <= 1;

                default:        write_back <= 0;
        endcase


always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b101x_xxx0,   // LDA, LDX, LDY
                8'b101x_xx01,   // LDA
                8'b101x_x0x1:   // LDA, LDZ
                                load_only <= 1;

                default:        load_only <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0001_1010,   // INCA
                8'b111x_x110,   // INC
                8'b11x0_1000,   // INX, INY
                8'b0001_1011,   // INZ
                8'b0100_0010:   // NEG
                                inc <= 1;

                default:        inc <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'bx111_0010,   // SBC (zp), ADC (zp)
                8'bx11x_xx01:   // SBC, ADC
                                adc_sbc <= 1;

                default:        adc_sbc <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0111_0010,   // ADC (zp)
                8'b011x_xx01:   // ADC
                                adc_bcd <= D;

                default:        adc_bcd <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0xxx_x110,   // ASL, ROL, LSR, ROR (abs, absx, zpg, zpgx)
                8'b0xx0_1010,   // ASL, ROL, LSR, ROR (acc)
                8'b010x_0100,   // ASR
                8'b0100_0011:   // ASR
                                shift <= 1;

                default:        shift <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b1101_0010,   // CMP (zp)
                8'b11x0_0x00,   // CPX, CPY (imm/zp)
                8'b11x0_1100,   // CPX, CPY (abs)
                8'b110x_xx01,   // CMP
                8'b1101_x100:   // CPZ
                                compare <= 1;

                default:        compare <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b01xx_x110,   // ROR, LSR
                8'b01x0_1x10,   // ROR, LSR
                8'b010x_0100,   // ASR
                8'b0100_0011:   // ASR
                                shift_right <= 1;

                default:        shift_right <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b010x_0100,   // ASR
                8'b0100_0011:   // ASR
                                arith_shift <= 1;

                default:        arith_shift <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0x10_1010,   // ROL A, ROR A
                8'b0x1x_x110:   // ROR, ROL
                                rotate <= 1;

                default:        rotate <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0000_x100,   // TSB
                8'b1xxx_0111:   // SMB
                                op <= OP_OR;

                8'b1000_1001,   // BIT imm
                8'b001x_x100,   // BIT zp/abs/zpx/absx
                8'b0001_x100,   // TRB
                8'b0xxx_0111:   // RMB
                                op <= OP_AND;

                8'b00xx_x110,   // ROL, ASL
                8'b00x0_1010:   // ROL, ASL
                                op <= OP_ROL;

                8'b01xx_x110,   // ROR, LSR
                8'b01xx_1x10,   // ROR, LSR
                8'b010x_0100,   // ASR
                8'b0100_0011:   // ASR
                                op <= OP_A;

                8'b11x1_0010,   // CMP, SBC (zp)
                8'b0011_101x,   // DEC A, DEZ
                8'b1000_1000,   // DEY
                8'b1100_1010,   // DEX
                8'b110x_x110,   // DEC
                8'b11xx_xx01,   // CMP, SBC
                8'b11x0_0x00,   // CPX, CPY (imm, zpg)
                8'b11x0_1100,   // CPY, CPY
                8'b1101_x100,   // CPZ
                8'b1100_0010,   // CPZ
                8'b0100_0010:   // NEG
                                op <= OP_SUB;

                8'b00x1_0010,   // ORA, AND (zp)
                8'b0x01_0010,   // ORA, EOR (zp)
                8'b010x_xx01,   // EOR
                8'b00xx_xx01:   // ORA, AND
                                op <= { 2'b11, IR[6:5] };

                default:        op <= OP_ADD;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b001x_x100:   // BIT zp/abs/zpx/absx (update N,V,Z)
                                {bit_ins, bit_ins_nv}  <= 2'b11;

                8'b1000_1001:   // BIT imm (update Z)
                                {bit_ins, bit_ins_nv}  <= 2'b10;

                default:        // not a BIT instruction
                                {bit_ins, bit_ins_nv}  <= 2'b00;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b000x_x100:   // TRB/TSB
                                txb_ins <= 1;

                default:        txb_ins <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0001_x100:   // TRB
                                trb_ins <= 1;

                default:        trb_ins <= 0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'bxxxx_0111:   // SMB/RMB
                                {xmb_ins, bbx_ins, bit_code} <= {1'b1, 1'b0, IR[7:4]};

                8'bxxxx_1111:   // BBS/BBR
                                {xmb_ins, bbx_ins, bit_code} <= {1'b0, 1'b1, IR[7:4]};

                default:        {xmb_ins, bbx_ins, bit_code} <= 6'd0;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'b0010_0010:   // JSR IND
                                { ind_jsr, ind_x_jsr } <= 2'b10;

                8'b0010_0011:   // JSR IND, X
                                { ind_jsr, ind_x_jsr } <= 2'b11;

                default:        { ind_jsr, ind_x_jsr } <= 2'b00;
        endcase

always @(posedge clk )
     if( state == DECODE && RDY )
        casex( IR )
                8'bxxx1_0011,   // conditional branges
                8'b1000_0011:   // BRA
                                long_branch <= 1;

                default:        long_branch <= 0;
        endcase

/*
 * special instructions
 */
always @(posedge clk )
     if( state == DECODE && RDY ) begin
        php <= (IR == 8'h08);
        clc <= (IR == 8'h18);
        plp <= (IR == 8'h28);
        sec <= (IR == 8'h38);
        cli <= (IR == 8'h58);
        sei <= (IR == 8'h78);
        clv <= (IR == 8'hb8);
        cld <= (IR == 8'hd8);
        sed <= (IR == 8'hf8);
        brk <= (IR == 8'h00);
        neg <= (IR == 8'h42);
        bsr <= (IR == 8'h63);
        rti <= (IR == 8'h40);
        rtn <= (IR == 8'h62);
     end

always @(posedge clk)
    if( state == DECODE && RDY )
        cond_code <= IR[7:4];

always @*
    case( cond_code )
            4'b0001: cond_true = ~N;
            4'b0011: cond_true = N;
            4'b0101: cond_true = ~V;
            4'b0111: cond_true = V;
            4'b1001: cond_true = ~C;
            4'b1011: cond_true = C;
            4'b1101: cond_true = ~Z;
            4'b1111: cond_true = Z;
            default: cond_true = 1; // BRA is 80
    endcase

always @(posedge clk)
        if ( state == RDONLY & RDY )
            case( bit_code[3:0] )
                    4'b0000: bit_cond_true <= ~DIMUX[0];
                    4'b0001: bit_cond_true <= ~DIMUX[1];
                    4'b0010: bit_cond_true <= ~DIMUX[2];
                    4'b0011: bit_cond_true <= ~DIMUX[3];
                    4'b0100: bit_cond_true <= ~DIMUX[4];
                    4'b0101: bit_cond_true <= ~DIMUX[5];
                    4'b0110: bit_cond_true <= ~DIMUX[6];
                    4'b0111: bit_cond_true <= ~DIMUX[7];
                    4'b1000: bit_cond_true <= DIMUX[0];
                    4'b1001: bit_cond_true <= DIMUX[1];
                    4'b1010: bit_cond_true <= DIMUX[2];
                    4'b1011: bit_cond_true <= DIMUX[3];
                    4'b1100: bit_cond_true <= DIMUX[4];
                    4'b1101: bit_cond_true <= DIMUX[5];
                    4'b1110: bit_cond_true <= DIMUX[6];
                    4'b1111: bit_cond_true <= DIMUX[7];
            endcase


reg NMI_1 = 0;          // delayed NMI signal

always @(posedge clk)
    NMI_1 <= NMI;

always @(posedge clk )
    if( NMI_edge && state == BRK2 )
        NMI_edge <= 0;
    else if( NMI & ~NMI_1 )
        NMI_edge <= 1;

endmodule

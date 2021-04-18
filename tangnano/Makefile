# Makefile for icestorm tools + yosys + nextpnr
# Modified from examples in icestorm distribution
# 03-03-19 E. Brombaugh
# this version builds the CPU code from C

# src directory
VPATH = ../src/ ../verilog-6502/ ../osvdu/

SRC =	gw1n1_6502_top.v ../src/tst_6502.v ../src/acia.v \
		../verilog-6502/cpu.v ../verilog-6502/ALU.v \
		../src/acia_rx.v ../src/acia_tx.v

# preparing the machine code
REAL_HEX = rom.hex

# project stuff
PROJ = gw1n1_6502_top
PIN_DEF = gw1n1_6502_top.cst
DEVICE = GW1N-1
DEVICE_PACKAGE = GW1N-LV1QN48C6/I5
BOARD = tangnano

YOSYS = yosys
NEXTPNR = nextpnr-gowin
RM = rm
GOWINPACK = gowin_pack
GOWINPROG = openFPGALoader
VERILATOR = verilator
TECH_LIB = /usr/local/share/yosys/gowin/cells_sim.v

all: $(PROJ).fs

%.json: $(SRC) $(PIN_DEF) $(REAL_HEX)
	echo $(YOSYS) -p 'synth_gowin -top $(PROJ) -json intermediate.json' $(SRC)
	$(NEXTPNR) --device $(DEVICE_PACKAGE) --json intermediate.json --cst $(PIN_DEF) --write $@
	#$(RM) intermediate.json

$(REAL_HEX):
	$(MAKE) -C ../cc65/
	cp ../cc65/$(REAL_HEX) .

assembly:
	$(MAKE) -C ../assembly/
	cp ../assembly/$(REAL_HEX) .

%.fs: %.json
	$(GOWINPACK) -d $DEVICE $< $@

prog: $(PROJ).fs
	$(GOWINPROG) -b $(BOARD) $<

lint: $(SRC)
	$(VERILATOR) --lint-only -Wall --top-module $(PROJ) $(TECH_LIB) $(SRC)

clean:
	rm -f *.json *.bin *.hex
	$(MAKE) clean -C ../cc65/

.SECONDARY:
.PHONY: all prog clean

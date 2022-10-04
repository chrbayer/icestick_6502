# Makefile for icestorm tools + yosys + nextpnr
# Modified from examples in icestorm distribution
# 03-03-19 E. Brombaugh
# this version builds the CPU code from C

SRC =	hx8k_65xx_top.v pll.v sram.v \
		../src/acia.v ../src/acia_rx.v ../src/acia_tx.v ../src/mos6526.v \
		../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_45gs02.v

# preparing the machine code
REAL_HEX = rom.hex

# project stuff
PROJ = hx8k_65xx_top
PIN_DEF = hx8k_65xx_top.pcf
SDC = hx8k_65xx_top.sdc
DEVICE = hx8k
PACKAGE = ct256

YOSYS = yosys
NEXTPNR = nextpnr-ice40
NEXTPNR_ARGS = --pre-pack $(SDC) --randomize-seed #--placer sa
ICEPACK = icepack
ICETIME = icetime
ICEPROG = iceprogduino
VERILATOR = verilator
VLOG = iverilog

all: $(PROJ).bin

%.json: $(SRC) $(REAL_HEX)
	$(YOSYS) -p 'synth_ice40 -top $(PROJ) -json $@' $(SRC)

%.asc: %.json $(PIN_DEF)
	$(NEXTPNR) $(NEXTPNR_ARGS) --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PIN_DEF) --asc $@

$(REAL_HEX):
	$(MAKE) -C ../kickc/
	cp ../kickc/$(REAL_HEX) ./$(REAL_HEX)

assembly:
	$(MAKE) -C ../assembly/
	cp ../assembly/$(REAL_HEX) .

%.bin: %.asc
	$(ICEPACK) $< $@

%.rpt: %.asc
	$(ICETIME) -d $(DEVICE) -mtr $@ $<

prog: $(PROJ).bin
	$(ICEPROG) $<

lint: $(SRC)
	$(VERILATOR) --lint-only -Wall --top-module $(PROJ) $(SRC)

tb_hx8k_65xx_top: $(SRC) $(REAL_HEX)
	$(VLOG) -D icarus -D SIM -o tb_hx8k_65xx_top $(SRC)
	./tb_hx8k_65xx_top -lxt2-space

clean:
	rm -f $(PROJ).json $(PROJ).asc $(PROJ).rpt $(PROJ).bin $(PROJ).hex rom.hex tb_hx8k_65xx_top *.vcd *.lxt
	$(MAKE) clean -C ../kickc/

.SECONDARY:
.PHONY: all prog clean

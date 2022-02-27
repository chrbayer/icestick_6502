# Makefile for icestorm tools + yosys + nextpnr
# Modified from examples in icestorm distribution
# 03-03-19 E. Brombaugh
# this version builds the CPU code from C

# src directory
#VPATH = ../src/ ../verilog-6502/ ../osvdu/

SRC =	hx8k_6502_top.v pll.v ../src/soc_6502.v ../src/acia.v \
		../verilog-6502/cpu_45gs02.v ../verilog-6502/alu_65ce02.v \
		../src/acia_rx.v ../src/acia_tx.v ../src/mos6526.v

# preparing the machine code
FAKE_HEX = rom.hex
REAL_HEX = code.hex

# project stuff
PROJ = hx8k_6502_top
PIN_DEF = hx8k_6502_top.pcf
SDC = hx8k_6502_top.sdc
DEVICE = hx8k

YOSYS = yosys
NEXTPNR = nextpnr-ice40
NEXTPNR_ARGS = --pre-pack $(SDC) --randomize-seed #--placer sa
ICEPACK = icepack
ICETIME = icetime
ICEPROG = iceprogduino
ICEBRAM = icebram
VERILATOR = verilator

all: $(PROJ).bin

$(FAKE_HEX):
	$(ICEBRAM) -g 8 4096 > $(FAKE_HEX)

%.json: $(SRC) $(FAKE_HEX)
	$(YOSYS) -p 'synth_ice40 -top $(PROJ) -json $@' $(SRC)

%.asc: %.json $(PIN_DEF)
	$(NEXTPNR) $(NEXTPNR_ARGS) --$(DEVICE) --json $< --pcf $(PIN_DEF) --asc $@

$(REAL_HEX):
	$(MAKE) -C ../kickc/
	cp ../kickc/$(FAKE_HEX) ./$(REAL_HEX)

assembly:
	$(MAKE) -C ../assembly/
	cp ../assembly/$(REAL_HEX) .

%.bin: %.asc $(REAL_HEX)
	$(ICEBRAM) $(FAKE_HEX) $(REAL_HEX) < $< > temp.asc
	$(ICEPACK) temp.asc $@

%.rpt: %.asc
	$(ICETIME) -d $(DEVICE) -mtr $@ $<

prog: $(PROJ).bin
	$(ICEPROG) $<

recode:
	rm -f $(REAL_HEX) $(PROJ).bin
	$(MAKE) prog

lint: $(SRC)
	$(VERILATOR) --lint-only -Wall --top-module $(PROJ) $(SRC)

clean:
	rm -f $(PROJ).json $(PROJ).asc temp.asc $(PROJ).rpt $(PROJ).bin $(PROJ).hex code.hex rom.hex
	$(MAKE) clean -C ../kickc/

.SECONDARY:
.PHONY: all prog recode clean

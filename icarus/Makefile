# Makefile for Icarus Verilog simulation
# 02-11-2019 E. Brombaugh

# sources
SOURCES = 	tb_soc_6502.v ../src/soc_6502.v ../src/acia.v ../src/acia_rx.v ../src/acia_tx.v \
			../verilog-6502/alu_6502.v ../verilog-6502/cpu_65c02.v

# top level
TOP = tb_soc_6502

# Executables
VLOG = iverilog
WAVE = gtkwave
TECH_LIB = /usr/local/share/yosys/ice40/cells_sim.v

# targets
all: $(TOP).vcd

$(TOP).vcd: $(TOP)
	cp ../cc65/rom.hex .
	./$(TOP)

$(TOP): $(SOURCES)
	$(VLOG) -D icarus -l $(TECH_LIB) -o $(TOP) $(SOURCES)

clean:
	rm -rf $(TOP) $(TOP).vcd

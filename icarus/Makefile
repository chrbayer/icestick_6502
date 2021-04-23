# Makefile for Icarus Verilog simulation
# 02-11-2019 E. Brombaugh

# sources
SOURCES = 	tb_soc_6502.v ../src/soc_6502.v ../src/acia.v ../src/acia_rx.v ../src/acia_tx.v \
			../verilog-6502/alu_6502.v ../verilog-6502/cpu_65c02.v

# top level
TOP = tb_soc_6502

# Executables
VLOG = iverilog
TECH_LIB = /usr/local/share/yosys/ice40/cells_sim.v

# targets
all: $(TOP).lxt

$(TOP).lxt: $(TOP)
	cp ../cc65/rom.hex .
	./$(TOP) -lxt2-space

$(TOP): $(SOURCES)
	$(VLOG) -D icarus -D SIM -l $(TECH_LIB) -o $(TOP) $(SOURCES)

tb_soc_6502_verification: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_6502 -l $(TECH_LIB) -o tb_soc_6502_verification $(SOURCES)
	./tb_soc_6502_verification -lxt2-space

tb_soc_65C02_verification: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65C02 -l $(TECH_LIB) -o tb_soc_65C02_verification $(SOURCES)
	./tb_soc_65C02_verification -lxt2-space

tb_soc_65C02_decimal_test: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D DECIMAL_TEST_65C02 -l $(TECH_LIB) -o tb_soc_65C02_decimal_test $(SOURCES)
	./tb_soc_65C02_decimal_test -lxt2-space

clean:
	rm -rf $(TOP) tb_soc_6502_verification tb_soc_65C02_verification tb_soc_65C02_decimal_test rom.hex *.vcd *.lxt

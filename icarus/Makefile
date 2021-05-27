# Makefile for Icarus Verilog simulation
# 02-11-2019 E. Brombaugh

# sources
SOURCES = 	tb_soc_6502.v ../src/soc_6502.v ../src/acia.v ../src/acia_rx.v ../src/acia_tx.v \
			../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_65ce02.v

# top level
TOP = tb_soc_6502

# Executables
VLOG = iverilog

# targets
all: $(TOP).lxt

$(TOP).lxt: $(TOP)
	cp ../cc65/rom.hex .
	./$(TOP) -lxt2-space

$(TOP): $(SOURCES)
	$(VLOG) -D icarus -D SIM -o $(TOP) $(SOURCES)

tb_soc_6502_verification: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_6502 -o tb_soc_6502_verification $(SOURCES)
	./tb_soc_6502_verification -lxt2-space

tb_soc_65C02_verification: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65C02 -o tb_soc_65C02_verification $(SOURCES)
	./tb_soc_65C02_verification -lxt2-space

tb_soc_65C02_decimal_test: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D DECIMAL_TEST_65C02 -o tb_soc_65C02_decimal_test $(SOURCES)
	./tb_soc_65C02_decimal_test -lxt2-space

tb_soc_65CE02_opcodes_test: $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65CE02 -o tb_soc_65CE02_opcodes_test $(SOURCES)
	./tb_soc_65CE02_opcodes_test -lxt2-space

clean:
	rm -rf $(TOP) tb_soc_6502_verification tb_soc_65C02_verification tb_soc_65C02_decimal_test tb_soc_65CE02_opcodes_test rom.hex *.vcd *.lxt

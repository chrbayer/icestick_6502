# Makefile for Icarus Verilog simulation
# 02-11-2019 E. Brombaugh

# sources
SOURCES = 	../src/soc_6502.v ../src/acia.v ../src/acia_rx.v ../src/acia_tx.v \
			../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_65ce02.v

# top level
TOP = tb_soc_6502

# Executables
VLOG = iverilog
VERILATOR = verilator

# targets
all: $(TOP).lxt

$(TOP).lxt: $(TOP)
	cp ../cc65/rom.hex .
	./$(TOP) -lxt2-space

$(TOP): tb_soc_6502.v $(SOURCES)
	$(VLOG) -D icarus -D SIM -o $(TOP) tb_soc_6502.v $(SOURCES)

tb_soc_6502_verification: tb_soc_6502.v $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_6502 -o tb_soc_6502_verification tb_soc_6502.v $(SOURCES)
	./tb_soc_6502_verification -lxt2-space

tb_soc_65C02_verification: tb_soc_6502.v $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65C02 -o tb_soc_65C02_verification tb_soc_6502.v $(SOURCES)
	./tb_soc_65C02_verification -lxt2-space

tb_soc_65C02_decimal_test: tb_soc_6502.v $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D DECIMAL_TEST_65C02 -o tb_soc_65C02_decimal_test tb_soc_6502.v $(SOURCES)
	./tb_soc_65C02_decimal_test -lxt2-space

tb_soc_65CE02_opcodes_test: tb_soc_6502.v $(SOURCES)
	$(VLOG) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65CE02 -o tb_soc_65CE02_opcodes_test tb_soc_6502.v $(SOURCES)
	./tb_soc_65CE02_opcodes_test -lxt2-space

lint:
	$(VERILATOR) --lint-only $(SOURCES)

verify:
	$(VERILATOR) -Wno-fatal -cc --exe -DVERIFICATION -DVERIFICATION_6502 verification_6502.cpp $(SOURCES)
	make -j4 -C obj_dir -f Vsoc_6502.mk
	./obj_dir/Vsoc_6502
	$(VERILATOR) -Wno-fatal -cc --exe -DVERIFICATION -DVERIFICATION_65C02 verification_65c02.cpp $(SOURCES)
	make -j4 -C obj_dir -f Vsoc_6502.mk
	./obj_dir/Vsoc_6502
	$(VERILATOR) -Wno-fatal -cc --exe -DVERIFICATION -DDECIMAL_TEST_65C02 decimal_test_65c02.cpp $(SOURCES)
	make -j4 -C obj_dir -f Vsoc_6502.mk
	./obj_dir/Vsoc_6502
	$(VERILATOR) -Wno-fatal -cc --exe -DVERIFICATION -DVERIFICATION_65CE02 verification_65ce02.cpp $(SOURCES)
	make -j4 -C obj_dir -f Vsoc_6502.mk
	./obj_dir/Vsoc_6502

clean:
	rm -rf $(TOP) tb_soc_6502_verification tb_soc_65C02_verification tb_soc_65C02_decimal_test tb_soc_65CE02_opcodes_test \
		   rom.hex *.vcd *.lxt obj_dir

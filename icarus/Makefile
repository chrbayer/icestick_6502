# Makefile for Icarus Verilog simulation
# 02-11-2019 E. Brombaugh

# cpu configuration
CPU ?= 45GS02

# program execution configuration
PROG ?= cc65

# sources
SOURCES = soc_65xx.v ../src/acia.v ../src/acia_rx.v ../src/acia_tx.v ../src/mos6526.v
SOURCES_VERIFY = soc_65xx.v

ifeq "$(CPU)" "6502"
	SOURCES += ../verilog-6502/alu_6502.v ../verilog-6502/cpu_6502.v
	SOURCES_VERIFY += ../verilog-6502/alu_6502.v ../verilog-6502/cpu_6502.v
endif

ifeq "$(CPU)" "65C02"
	SOURCES += ../verilog-6502/alu_6502.v ../verilog-6502/cpu_65c02.v
	SOURCES_VERIFY += ../verilog-6502/alu_6502.v ../verilog-6502/cpu_65c02.v
endif

ifeq "$(CPU)" "65CE02"
	SOURCES += ../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_65ce02.v
	SOURCES_VERIFY += ../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_65ce02.v
endif

ifeq "$(CPU)" "45GS02"
	SOURCES += ../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_45gs02.v
	SOURCES_VERIFY += ../verilog-6502/alu_65ce02.v ../verilog-6502/cpu_45gs02.v
endif

# top level
TOP = tb_soc_$(CPU)

# Executables
VLOG = iverilog
VERILATOR = verilator

# targets
all: $(TOP).fst

$(TOP).fst: $(TOP)
	make -C ../$(PROG)
	cp ../$(PROG)/rom.hex .
	./$(TOP) -fst

$(TOP): tb_soc_65xx.v $(SOURCES)
	$(VLOG) -D CPU_$(CPU) -D icarus -D SIM -o $(TOP) tb_soc_65xx.v $(SOURCES)

tb_soc_6502_verification: tb_soc_65xx.v $(SOURCES)
	make -j4 -C ../6502_65C02_functional_tests/tass64
	$(VLOG) -D CPU_$(CPU) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_6502 -o tb_soc_6502_verification tb_soc_65xx.v $(SOURCES)
	./tb_soc_6502_verification -fst

tb_soc_65C02_verification: tb_soc_65xx.v $(SOURCES)
	make -j4 -C ../6502_65C02_functional_tests/tass64
	$(VLOG) -D CPU_$(CPU) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65C02 -o tb_soc_65C02_verification tb_soc_65xx.v $(SOURCES)
	./tb_soc_65C02_verification -fst

tb_soc_65C02_decimal_test: tb_soc_65xx.v $(SOURCES)
	make -j4 -C ../6502_65C02_functional_tests/tass64
	$(VLOG) -D CPU_$(CPU) -D icarus -D SIM -D VERIFICATION -D DECIMAL_TEST_65C02 -o tb_soc_65C02_decimal_test tb_soc_65xx.v $(SOURCES)
	./tb_soc_65C02_decimal_test -fst

tb_soc_65CE02_opcodes_test: tb_soc_65xx.v $(SOURCES)
	make -j4 -C ../6502_65C02_functional_tests/tass64
	$(VLOG) -D CPU_$(CPU) -D icarus -D SIM -D VERIFICATION -D VERIFICATION_65CE02 -o tb_soc_65CE02_opcodes_test tb_soc_65xx.v $(SOURCES)
	./tb_soc_65CE02_opcodes_test -fst

lint:
	$(VERILATOR) --lint-only -DCPU_$(CPU) $(SOURCES)

verify:
	make -j4 -C ../6502_65C02_functional_tests/tass64
	rm -rf obj_dir
	$(VERILATOR) -Wno-fatal -cc --exe --public -DCPU_$(CPU) -DVERIFICATION -DVERIFICATION_6502 verification_6502.cpp $(SOURCES_VERIFY)
	make -j4 -C obj_dir -f Vsoc_65xx.mk
	./obj_dir/Vsoc_65xx
ifeq ($(CPU),$(filter $(CPU),65C02 65CE02 45GS02))
	$(VERILATOR) -Wno-fatal -cc --exe --public -DCPU_$(CPU) -DVERIFICATION -DVERIFICATION_65C02 verification_65c02.cpp $(SOURCES_VERIFY)
	make -j4 -C obj_dir -f Vsoc_65xx.mk
	./obj_dir/Vsoc_65xx
	$(VERILATOR) -Wno-fatal -cc --exe --public -DCPU_$(CPU) -DVERIFICATION -DDECIMAL_TEST_65C02 decimal_test_65c02.cpp $(SOURCES_VERIFY)
	make -j4 -C obj_dir -f Vsoc_65xx.mk
	./obj_dir/Vsoc_65xx
ifeq ($(CPU),$(filter $(CPU),65CE02 45GS02))
	$(VERILATOR) -Wno-fatal -cc --exe --public -DCPU_$(CPU) -DVERIFICATION -DVERIFICATION_65CE02 verification_65ce02.cpp $(SOURCES_VERIFY)
	make -j4 -C obj_dir -f Vsoc_65xx.mk
	./obj_dir/Vsoc_65xx
endif
else
	$(VERILATOR) -Wno-fatal -cc --exe --public -DCPU_$(CPU) -DVERIFICATION -DDECIMAL_TEST_6502 decimal_test_65c02.cpp $(SOURCES_VERIFY)
	make -j4 -C obj_dir -f Vsoc_65xx.mk
	./obj_dir/Vsoc_65xx
endif

clean:
	rm -rf tb_soc_6502 tb_soc_65C02 tb_soc_65CE02 tb_soc_45GS02 \
		   tb_soc_6502_verification tb_soc_65C02_verification tb_soc_65C02_decimal_test tb_soc_65CE02_opcodes_test \
		   rom.hex *.vcd *.fst obj_dir

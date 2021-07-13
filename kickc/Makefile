# Makefile for kickc to verilog hex

# C and assy source files
SRC =	soc-test.c

# compiler output
OBJ = rom

# build tools & options
KICKC = kickc
KICKC_ARGS = -S -Ocoalesce -Oliverangecallpath -Oloophead -Onocache -Ouplift=200
KICKASS = kickassembler
HEXDUMP = hexdump
HEXDUMP_ARGS = -v -e '1/1 "%02x " "\n"'

# Targets
all: $(OBJ).hex

$(OBJ).hex: soc-test.bin
	$(HEXDUMP) $(HEXDUMP_ARGS) $< > $@

soc-test.bin: soc-test.asm
	$(KICKASS) soc-test.asm

soc-test.asm: $(SRC)
	$(KICKC) $(KICKC_ARGS) soc-test.c

clean:
	-rm -f *.asm test-soc.bin $(OBJ).hex

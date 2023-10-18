# Makefile for kickc to verilog hex

# C and assy source files
SRC =	startup.s soc-test.c

# compiler output
OBJ = rom

# build tools & options
VC = vc
VC_ARGS = +ice6502 -O2 -size -o $(OBJ)
HEXDUMP = hexdump
HEXDUMP_ARGS = -v -e '1/1 "%02x " "\n"'

# Targets
all: $(OBJ).hex

$(OBJ).hex: $(OBJ)
	$(HEXDUMP) $(HEXDUMP_ARGS) $< > $@

$(OBJ): $(SRC) ice6502
	$(VC) $(VC_ARGS) $(SRC)

clean:
	-rm -f $(OBJ) $(OBJ).hex rom.map listfile

# Makefile for cc65 to verilog hex
# 03-04-19 E. Brombaugh

# C and assy source files
SRC =	main.c acia.s interrupt.s vectors.s

# compiler output
OBJ = rom

# build tools & options
CL65 = cl65
CLFLAGS  = -t none -O --cpu 65c02 -C sbc.cfg -m $(OBJ).map
LIB = sbc.lib
HEXDUMP = hexdump
HEXDUMP_ARGS = -v -e '1/1 "%02x " "\n"'

# Targets
all: $(OBJ).hex

$(OBJ).hex: $(OBJ)
	$(HEXDUMP) $(HEXDUMP_ARGS) $< > $@

test_asm.o: test_asm.s
	ca65 --cpu 4510 test_asm.s

$(OBJ): $(SRC) test_asm.o
	$(CL65) $(CLFLAGS) -o $(OBJ) $(SRC) $(LIB) test_asm.o

clean:
	-rm -f *.o $(OBJ) $(OBJ).hex $(OBJ).map

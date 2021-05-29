#include <stdio.h>
#include <stdlib.h>
#include "Vsoc_6502.h"
#include "verilated.h"

void tick(unsigned int tickcount, Vsoc_6502 *tb) {
    tb->eval();

    if (tickcount < 5)
       tb->reset = 1;
    else
       tb->reset = 0;

    tb->clk = 1;

    tb->eval();

    tb->clk = 0;
}

int main(int argc, char**argv) {
    unsigned int tickcount = 0;
    int ret = -1;

    // Call  commandArgs  first!
    Verilated::commandArgs(argc,argv);

    //  Instantiate  our  design
    Vsoc_6502 *tb = new Vsoc_6502;

    int test_case = -1;

    for(int k = 0; k < (1<<26); k++) {
          tick(++tickcount, tb);
          int current = tb->soc_6502__DOT__ram_mem[512];
          if (current != test_case) {
              test_case = current;
              printf("#%d test_case: %x\n", tickcount, test_case);
          }
          if (current == 0xf0) {
              ret = 0;
              break;
          }
    }

    return ret;
}

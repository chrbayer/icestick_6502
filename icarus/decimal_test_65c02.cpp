#include <stdio.h>
#include <stdlib.h>
#include "Vsoc_65xx.h"
#include "Vsoc_65xx___024root.h"
#include "verilated.h"

void tick(unsigned int tickcount, Vsoc_65xx *tb) {
    tb->eval();

    if (tickcount < 5)
       tb->reset_n = 0;
    else
       tb->reset_n = 1;

    tb->clk = 1;

    tb->eval();

    tb->clk = 0;
}

int main(int argc, char**argv) {
    unsigned int tickcount = 0;
    int det = 0;
    int ret = -1;

    // Call  commandArgs  first!
    Verilated::commandArgs(argc,argv);

    //  Instantiate  our  design
    Vsoc_65xx *tb = new Vsoc_65xx;

    // Make sure that no interrupts are triggered
    tb->IRQ_n = 1;
    tb->NMI_n = 1;

    int test_case = -1;

    for(int k = 0; k < (1<<24); k++) {
          tick(++tickcount, tb);
          int current = tb->rootp->soc_65xx__DOT__ram_mem[21];
          if (current != test_case) {
              test_case = current;
              printf("#%d test_case: %x\n", tickcount, test_case);
              if (current == 0) {
                  ++det;
              }
          }
          if (det == 2) {
              ret = 0;
              break;
          }
    }

    if (ret == 0) {
        printf("\nSucceeded after %d ticks\n", tickcount);
    }

    printf("\n");

    return ret;
}

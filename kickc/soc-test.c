#pragma target(soc)

#include "soc.h"


const unsigned int LOW = 0x1000;
const unsigned int HIGH = 0xF000;
const unsigned int INC = 0x800;

unsigned int t = 0x4000;
bool d = true;
unsigned char x = 0;
unsigned int a;

void acia_tx_chr(char character)
{
    while (!(ACIA->CTRL & 0x02)) ;
    ACIA->DATA = character;
}

void acia_tx_str(char *text)
{
    while( *text != 0)
    {
        acia_tx_chr(*text);
        ++text;
    }
}

void main() {
    unsigned char old = 0;

    // initialize stack
    asm
    {
        ldx #$ff
        txs
    }

    *(&a) = 0x1234;

    ++a;

    if (a == 0x1235) a = 0;

/*  acia_tx_chr('H');
    acia_tx_chr('E');
    acia_tx_chr('L');
    acia_tx_chr('L');
    acia_tx_chr('O');
    acia_tx_chr(',');
    acia_tx_chr(' ');
    acia_tx_chr('W');
    acia_tx_chr('O');
    acia_tx_chr('R');
    acia_tx_chr('L');
    acia_tx_chr('D');
    acia_tx_chr('!');
    acia_tx_chr(0x0a);*/

    // Configure CIA
    CIA->PORT_B_DDR = 0xC0;

    // Infinite loop
    while(1) {
        CIA->TIMER_B = t;
        CIA->TIMER_B_CONTROL = CIA_TIMER_CONTROL_ONESHOT | CIA_TIMER_CONTROL_START;

        while (CIA->TIMER_B_CONTROL & CIA_TIMER_CONTROL_START)
        {
        }

        // Write counter to port
        CIA->PORT_B = (x & 0xC0);
        x++;

        if ((x & 0xC0) != (old & 0xC0)) {
            if (d) {
                t += INC;
                if (t > HIGH) {
                    d = false;
                }
            }
            else {
                t -= INC;
                if (t < LOW) {
                    d = true;
                }
            }
        }

        old = x;
    }
}


// Interrupt Vectors
#pragma data_seg(Vectors)
__export void (*VECTORS[])() = {
    0,
    &main,
    0
};

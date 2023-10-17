#include "soc.h"


const unsigned int LOW = 0x1000;
const unsigned int HIGH = 0xF000;
const unsigned int INC = 0x800;

unsigned int t = 0x4000;
unsigned char d = 1;
unsigned char x = 0;

void main() {
    unsigned char old = 0;

    // Configure CIA
    CIA->PORT_B_DDR = 0xC0;
    CIA->PORT_B = 0x00;

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
                    d = 0;
                }
            }
            else {
                t -= INC;
                if (t < LOW) {
                    d = 1;
                }
            }
        }

        old = x;
    }
}

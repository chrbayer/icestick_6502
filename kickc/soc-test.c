#pragma target(soc)

#include "soc.h"


unsigned char x = 0;


void main() {

    // Configure CIA
    CIA->PORT_B_DDR = 0xC0;
    CIA->TIMER_B = 0x4000;

    // Infinite loop
    while(1) {
        CIA->TIMER_B_CONTROL = CIA_TIMER_CONTROL_ONESHOT | CIA_TIMER_CONTROL_START;

        while (CIA->TIMER_B_CONTROL & CIA_TIMER_CONTROL_START)
        {
        }

        // Write counter to port
        CIA->PORT_B = (x & 0xC0);
        x++;
    }
}


// Interrupt Vectors
#pragma data_seg(Vectors)
export void (*VECTORS[])() = {
    0,
    &main,
    0
};

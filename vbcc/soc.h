#include "mos6526.h"

/// CIA
#define CIA ((struct MOS6526_CIA *)0xD000)

/// ACIA
struct MOS6551_ACIA {
    /// CTRL port
    char CTRL;
    /// STATUS port
    char DATA;
};
#define ACIA ((struct MOS6551_ACIA *)0xD040)

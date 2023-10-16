#include "mos6526.h"

/// CIA
struct MOS6526_CIA * CIA = (struct MOS6526_CIA *)0xD000;

/// ACIA
struct MOS6551_ACIA {
    /// CTRL port
    char CTRL;
    /// STATUS port
    char DATA;
};
struct MOS6551_ACIA * ACIA = (struct MOS6551_ACIA *)0xD040;

/// @file
/// Custom SOC
#ifndef __SOC__
#error "Target platform must be SOC"
#endif
#include <mos6526.h>
#include <mos6551.h>

/// CIA
struct MOS6526_CIA * CIA = (struct MOS6526_CIA *)0xD000;

/// ACIA
struct MOS6551_ACIA * ACIA = (struct MOS6551_ACIA *)0xD040;

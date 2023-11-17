#ifndef __SOC_H__
#define __SOC_H__

/// @file
/// Custom SOC
#ifndef __SOC__
#error "Target platform must be SOC"
#endif
#include <mos6526.h>

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

/// Control - Master reset (0x03: reset active)
const char ACIA_CONTROL_RESET = 0b00000011;
/// Control - Transmit interrupt enable (1: enabled, 0: disabled)
const char ACIA_CONTROL_TRANSMIT_INT_ENABLED = 0b00100000;
/// Control - Receive interrupt enable (1: enabled, 0: disabled)
const char ACIA_CONTROL_RECEIVE_INT_ENABLED = 0b10000000;
/// Status - Receive data register full (1: full, 0: not full)
const char ACIA_STATUS_RECEIVE_DATA_REG_FULL = 0b00000001;
/// Status - Transmit data register empty (1: empty, 0: not empty)
const char ACIA_STATUS_TRANSMIT_DATA_REG_EMPTY = 0b00000010;
/// Status - Receive data error detected (1: error detected, 0: error not detected)
const char ACIA_STATUS_RECEIVE_ERROR_OCCURED = 0b00010000;
/// Status - Interupt request present (1: IRQ present, 0: IRQ not present)
const char ACIA_STATUS_IRQ_ACTIVE = 0b10000000;

#endif

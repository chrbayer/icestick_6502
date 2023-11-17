#ifndef __SOC_H__
#define __SOC_H__

/// The MOS 6526 Complex Interface Adapter (CIA)
/// http://archive.6502.org/datasheets/mos_6526_cia_recreated.pdf
struct MOS6526_CIA {
    /// Port A
    volatile char PORT_A;
    /// Port B
    volatile char PORT_B;
    /// Port A data direction register.
    char PORT_A_DDR;
    /// Port B data direction register.
    char PORT_B_DDR;
    /// Timer A Value
    volatile unsigned int TIMER_A;
    /// Timer B Value
    volatile unsigned int TIMER_B;
    /// Time-of-day real-time-clock tenth seconds (BCD)
    volatile char TOD_10THS;
    /// Time-of-day real-time-clock seconds (BCD)
    volatile char TOD_SEC;
    /// Time-of-day real-time-clock minutes (BCD)
    volatile char TOD_MIN;
    /// Time-of-day real-time-clock hours (BCD)
    volatile char TOD_HOURS;
    /// Serial Shift Register
    volatile char SERIAL_DATA;
    /// Interrupt Status & Control Register
    volatile char INTERRUPT;
    /// Timer A Control Register
    volatile char TIMER_A_CONTROL;
    /// Timer B Control Register
    volatile char TIMER_B_CONTROL;
};
#define CIA ((struct MOS6526_CIA *)0xD000)

/// Value that disables all CIA interrupts when stored to the CIA Interrupt registers
#define CIA_INTERRUPT_CLEAR 0x7f
/// Timer Control - Start/stop timer (0:stop, 1: start)
#define CIA_TIMER_CONTROL_STOP 0x00
/// Timer Control - Start/stop timer (0:stop, 1: start)
#define CIA_TIMER_CONTROL_START 0x01
/// Timer Control - Time CONTINUOUS/ONE-SHOT (0:CONTINUOUS, 1: ONE-SHOT)
#define CIA_TIMER_CONTROL_CONTINUOUS 0x00
/// Timer Control - Time CONTINUOUS/ONE-SHOT (0:CONTINUOUS, 1: ONE-SHOT)
#define CIA_TIMER_CONTROL_ONESHOT 0x08
/// Timer A Control - Timer counts (0:system cycles, 1: CNT pulses)
#define CIA_TIMER_CONTROL_A_COUNT_CYCLES 0x00
/// Timer A Control - Timer counts (0:system cycles, 1: CNT pulses)
#define CIA_TIMER_CONTROL_A_COUNT_CNT 0x20
/// Timer A Control - Serial Port Mode (0: Serial Port Input, 1: Serial Port Output)
#define CIA_TIMER_CONTROL_A_SERIAL_IN 0x00
/// Timer A Control - Serial Port Mode (0: Serial Port Input, 1: Serial Port Output)
#define CIA_TIMER_CONTROL_A_SERIAL_OUT 0x40
/// Timer A Control - time-of-day clock Mode (0: 60Hz, 1: 50Hz)
#define CIA_TIMER_CONTROL_A_TOD_60HZ 0x00
/// Timer A Control - time-of-day clock Mode (0: 60Hz, 1: 50Hz)
#define CIA_TIMER_CONTROL_A_TOD_50HZ 0x80
/// Timer B Control - Timer counts (00:system cycles, 01: CNT pulses, 10: timer A underflow, 11: time A underflow while CNT is high)
#define CIA_TIMER_CONTROL_B_COUNT_CYCLES 0x00
/// Timer B Control - Timer counts (00:system cycles, 01: CNT pulses, 10: timer A underflow, 11: time A underflow while CNT is high)
#define CIA_TIMER_CONTROL_B_COUNT_CNT 0x20
/// Timer B Control - Timer counts (00:system cycles, 01: CNT pulses, 10: timer A underflow, 11: time A underflow while CNT is high)
#define CIA_TIMER_CONTROL_B_COUNT_UNDERFLOW_A 0x40
/// Timer B Control - Timer counts (00:system cycles, 01: CNT pulses, 10: timer A underflow, 11: time A underflow while CNT is high)
#define CIA_TIMER_CONTROL_B_COUNT_UNDERFLOW_A_CNT 0x60
/// Timer B Control - time-of-day write mode (0: TOD clock, 1: TOD alarm)
#define CIA_TIMER_CONTROL_B_TOD_CLOCK_SET 0x00
/// Timer B Control - time-of-day write mode (0: TOD clock, 1: TOD alarm)
#define CIA_TIMER_CONTROL_B_TOD_ALARM_SET 0x80


/// ACIA
struct MOS6551_ACIA {
    /// CTRL port
    volatile char CTRL;
    /// STATUS port
    volatile char DATA;
};
#define ACIA ((struct MOS6551_ACIA *)0xD040)

/// Control - Master reset (0x03: reset active)
#define ACIA_CONTROL_RESET 0x03
/// Control - Transmit interrupt enable (1: enabled, 0: disabled)
#define ACIA_CONTROL_TRANSMIT_INT_ENABLED 0x20
/// Control - Receive interrupt enable (1: enabled, 0: disabled)
#define ACIA_CONTROL_RECEIVE_INT_ENABLED 0x80
/// Status - Receive data register full (1: full, 0: not full)
#define ACIA_STATUS_RECEIVE_DATA_REG_FULL 0x01
/// Status - Transmit data register empty (1: empty, 0: not empty)
#define ACIA_STATUS_TRANSMIT_DATA_REG_EMPTY 0x02
/// Status - Receive data error detected (1: error detected, 0: error not detected)
#define ACIA_STATUS_RECEIVE_ERROR_OCCURED 0x10
/// Status - Interupt request present (1: IRQ present, 0: IRQ not present)
#define ACIA_STATUS_IRQ_ACTIVE 0x80

#endif

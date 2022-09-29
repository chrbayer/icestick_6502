/*
 * fpga.h - handy info about the FPGA
 * 03-04-19 E. Brombaugh
 */

#ifndef __FPGA__
#define __FPGA__

#define CIA_PRA         (*(volatile unsigned char *) 0xD000)
#define CIA_PRB         (*(volatile unsigned char *) 0xD001)
#define CIA_DDRA        (*(volatile unsigned char *) 0xD002)
#define CIA_DDRB        (*(volatile unsigned char *) 0xD003)
#define CIA_TA_LO       (*(volatile unsigned char *) 0xD004)
#define CIA_TA_HI       (*(volatile unsigned char *) 0xD005)
#define CIA_TB_LO       (*(volatile unsigned char *) 0xD006)
#define CIA_TB_HI       (*(volatile unsigned char *) 0xD007)
#define CIA_TOD_10THS   (*(volatile unsigned char *) 0xD008)
#define CIA_TOD_SEC     (*(volatile unsigned char *) 0xD009)
#define CIA_TOD_MIN     (*(volatile unsigned char *) 0xD00A)
#define CIA_TOD_HR      (*(volatile unsigned char *) 0xD00B)
#define CIA_SDR         (*(volatile unsigned char *) 0xD00C)
#define CIA_ICR         (*(volatile unsigned char *) 0xD00D)
#define CIA_CRA         (*(volatile unsigned char *) 0xD00E)
#define CIA_CRB         (*(volatile unsigned char *) 0xD00F)

#define ACIA_CTRL       (*(volatile unsigned char *) 0xD040)
#define ACIA_DATA       (*(volatile unsigned char *) 0xD041)

#endif

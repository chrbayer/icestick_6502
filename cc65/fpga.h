/*
 * fpga.h - handy info about the FPGA
 * 03-04-19 E. Brombaugh
 */

#ifndef __FPGA__
#define __FPGA__

#define CIA_PRA         (*(volatile unsigned char *) 0x1000)
#define CIA_PRB         (*(volatile unsigned char *) 0x1001)
#define CIA_DDRA        (*(volatile unsigned char *) 0x1002)
#define CIA_DDRB        (*(volatile unsigned char *) 0x1003)
#define CIA_TA_LO       (*(volatile unsigned char *) 0x1004)
#define CIA_TA_HI       (*(volatile unsigned char *) 0x1005)
#define CIA_TB_LO       (*(volatile unsigned char *) 0x1006)
#define CIA_TB_HI       (*(volatile unsigned char *) 0x1007)
#define CIA_TOD_10THS   (*(volatile unsigned char *) 0x1008)
#define CIA_TOD_SEC     (*(volatile unsigned char *) 0x1009)
#define CIA_TOD_MIN     (*(volatile unsigned char *) 0x100A)
#define CIA_TOD_HR      (*(volatile unsigned char *) 0x100B)
#define CIA_SDR         (*(volatile unsigned char *) 0x100C)
#define CIA_ICR         (*(volatile unsigned char *) 0x100D)
#define CIA_CRA         (*(volatile unsigned char *) 0x100E)
#define CIA_CRB         (*(volatile unsigned char *) 0x100F)

#define ACIA_CTRL (*(unsigned char *) 0x2000)
#define ACIA_DATA (*(unsigned char *) 0x2001)

#endif

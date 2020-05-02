/*
   THIS STRUCTURE IS DEFINED TO THE SPECIAL FACULTY REGISTER OF V25/V35.
   DATE 08 JULY88

   *** updated for v25+ ***

   Copyright (C) NEC Corporation 1988
*/

#ifndef __V25_H__
#define __V25_H__

#include <dos.h>

/*
   SFR STRUCTURE
*/

typedef struct SFR
        {
         unsigned char port0;          /* port 0                                       */
         unsigned char portm0;         /* port 0 mode                                  */
         unsigned char portmc0;        /* port 0 control                               */
         unsigned char dummy1[5];
         unsigned char port1;          /* port 1                                       */
         unsigned char portm1;         /* port 1 mode                                  */
         unsigned char portmc1;        /* port 1 control                               */
         unsigned char dummy2[5];
         unsigned char port2;          /* port 2                                       */
         unsigned char portm2;         /* port 2 mode                                  */
         unsigned char portmc2;        /* port 2 control                               */
         unsigned char dummy3[37];
         unsigned char portT;          /* port T                                       */
         unsigned char dummy4[2];
         unsigned char portmT;         /* port T mode                                  */
         unsigned char dummy5[4];
         unsigned char intm;           /* external interrupt mode register 'IMR'       */
         unsigned char dummy6[3];
         unsigned char ems0;           /* int0 macro service control reg.              */
         unsigned char ems1;           /* int1 macro service control reg.              */
         unsigned char ems2;           /* int2 macro service control reg.              */
         unsigned char dummy7[5];
         unsigned char exic0;          /* int0 ext. interrupt control register 'IRC'   */
         unsigned char exic1;          /* int1 ext. interrupt control register 'IRC'   */
         unsigned char exic2;          /* int2 ext. interrupt control register 'IRC'   */
         unsigned char dummy8[17];
         unsigned char rxb0;           /* receive buffer 0                             */
         unsigned char dummy9;
         unsigned char txb0;           /* transmit buffer 0                            */
         unsigned char dummy10[2];
         unsigned char srms0;          /* receive macro service control reg.           */
         unsigned char stms0;          /* transmit macro service control reg.          */
         unsigned char dummy11;
         unsigned char scm0;           /* serial mode register                         */
         unsigned char scc0;           /* serial control register                      */
         unsigned char brg0;           /* baud rate register                           */
         unsigned char scs0;           /* serial status register                       */
         unsigned char seic0;          /* serial error interrupt control reg. 'IRC'    */
         unsigned char sric0;          /* serial receive interrupt control reg. 'IRC'  */
         unsigned char stic0;          /* serial transmit interrupt control reg. 'IRC' */
         unsigned char dummy12;
         unsigned char rxb1;           /* receive buffer 1                             */
         unsigned char dummy13;
         unsigned char txb1;           /* transmit buffer 1                            */
         unsigned char dummy14[2];
         unsigned char srms1;          /* serial receive macro service control reg.    */
         unsigned char stms1;          /* serial transmit macro service control reg.   */
         unsigned char dummy15;
         unsigned char scm1;           /* serial mode register                         */
         unsigned char scc1;           /* serial control register                      */
         unsigned char brg1;           /* baud rate register                           */
         unsigned char scs1;           /* serial status register                       */
         unsigned char seic1;          /* serial error interrupt control reg. 'IRC'    */
         unsigned char sric1;          /* serial receive interrupt control reg. 'IRC'  */
         unsigned char stic1;          /* serial transmit interrupt control reg. 'IRC' */
         unsigned char dummy16;
         unsigned int  tm0;            /* timer 0                                      */
         unsigned int  md0;            /* timer 0 modulus                              */
         unsigned int  dummy17[2];
         unsigned int  tm1;            /* timer 1                                      */
         unsigned int  md1;            /* timer 1 modulus                              */
         unsigned int  dummy18[2];
         unsigned char tmc0;           /* timer 0 control                              */
         unsigned char tmc1;           /* timer 1 control                              */
         unsigned char dummy19[2];
         unsigned char tmms0;          /* timer 0 macro service control reg.           */
         unsigned char tmms1;          /* timer 1 macro service control reg.           */
         unsigned char tmms2;          /* timer 2 macro service control reg.           */
         unsigned char dummy20[5];
         unsigned char tmic0;          /* timer 0 interrupt control reg. 'IRC'         */
         unsigned char tmic1;          /* timer 1 interrupt control reg. 'IRC'         */
         unsigned char tmic2;          /* timer 2 interrupt control reg. 'IRC'         */
         unsigned char dummy21;
         unsigned char dmac0;          /* DMA 0 control                                */
         unsigned char dmam0;          /* DMA 0 mode                                   */
         unsigned char dmac1;          /* DMA 1 control                                */
         unsigned char dmam1;          /* DMA 1 mode                                   */
         unsigned char dummy22[8];
         unsigned char dic0;           /* DMA channel 0 interrupt control reg. 'IRC'   */
         unsigned char dic1;           /* DMA channel 1 interrupt control reg. 'IRC'   */
         unsigned char dummy23[18];
         unsigned int  sar0;           /* DMA channel 0 source address low word        */
         unsigned char sar0h;          /* DMA channel 0 source address high byte       */
         unsigned char dummy24;
         unsigned int  dar0;           /* DMA channel 0 destination address low word   */
         unsigned char dar0h;          /* DMA channel 0 destination address high byte  */
         unsigned char dummy25;
         unsigned int  tc0;            /* DMA channel 0 terminal count                 */
         unsigned char dummy26[6];
         unsigned int  sar1;           /* DMA channel 1 source address low word        */
         unsigned char sar1h;          /* DMA channel 1 source address high byte       */
         unsigned char dummy27;
         unsigned int  dar1;           /* DMA channel 1 destination address low word   */
         unsigned char dar1h;          /* DMA channel 1 destination address high byte  */
         unsigned char dummy28;
         unsigned int  tc1;            /* DMA channel 1 terminal count                 */
         unsigned char dummy29[6];
         unsigned char stbc;           /* standby control reg.                         */
         unsigned char rfm;            /* refresh mode reg.                            */
         unsigned char dummy30[6];
         unsigned int  wtc;            /* wait control reg.                            */
         unsigned char flag;           /* user flag register                           */
         unsigned char prc;            /* processor control register                   */
         unsigned char tbic;           /* time base interrupt control reg.             */
         unsigned char dummy31[2];
         unsigned char irqs;           /* interrupt source register                    */
         unsigned char dummy32[12];
         unsigned char ispr;           /* interrupt priority register                  */
         unsigned char dummy33[2];
         unsigned char idb;            /* internal data base reg.                      */
        };

/*
   REGISTER BAMK STRUCTURE
*/

typedef struct REGBNK
        {
         int reserve;
         int vec_pc;
         int save_psw;
         int save_pc;
         int ds0;
         int ss;
         int ps;
         int ds1;
         int iy;
         int ix;
         int bp;
         int sp;
         int bw;
         int dw;
         int cw;
         int aw;
        };

/*
   MACRO CHANNEL STRUCTURE
*/

typedef struct macroChannel_tag
        {
         unsigned char bMSC;
         unsigned char bSFRP;
         unsigned char bSCHR;
         unsigned char bReserved;
         unsigned int  wMSP;
         unsigned int  wMSS;
        } macroChannel;

/*
   macro definitions for I/O port bit set/reset positions
*/

#define  BIT_0   0
#define  BIT_1   1
#define  BIT_2   2
#define  BIT_3   3
#define  BIT_4   4
#define  BIT_5   5
#define  BIT_6   6
#define  BIT_7   7

/* NEC V25:   CLR1  byte ptr [bx + _mem8_], _bitPos_
*/
#define  BIT_CLR(_bitPos_, _mem8_) asm {                            \
                                        push   es;                  \
                                        mov    bx, 0f000h;          \
                                        mov    es, bx;              \
                                        mov    bx, 0ff00h;          \
                                        db     26h;                 \
                                        db     00fh;                \
                                        db     01ah;                \
                                        db     087h;                \
                                        dw     ._mem8_;             \
                                        db     _bitPos_;            \
                                        pop    es;                  \
                                       }


/* NEC V25:   SET1  byte ptr [bx + _mem8_], _bitPos_
*/
#define  BIT_SET(_bitPos_, _mem8_) asm {                            \
                                        push   es;                  \
                                        mov    bx, 0f000h;          \
                                        mov    es, bx;              \
                                        mov    bx, 0ff00h;          \
                                        db     26h;                 \
                                        db     0fh;                 \
                                        db     1ch;                 \
                                        db     87h;                 \
                                        dw     ._mem8_;             \
                                        db     _bitPos_;            \
                                        pop    es;                  \
                                       }

/* NEC V25:   NOT1  byte ptr [bx + _mem8_], _bitPos_
*/
#define  BIT_NOT(_bitPos_, _mem8_) asm {                            \
                                        push   es;                  \
                                        mov    bx, 0f000h;          \
                                        mov    es, bx;              \
                                        mov    bx, 0ff00h;          \
                                        db     26h;                 \
                                        db     00fh;                \
                                        db     01eh;                \
                                        db     087h;                \
                                        dw     ._mem8_;             \
                                        db     _bitPos_;            \
                                        pop    es;                  \
                                       }

#endif /* __V25_H__ */

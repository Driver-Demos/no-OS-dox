/***************************************************************************//**
 *   @file   ade7880.h
 *   @brief  Header file of ADE7880 Driver.
 *   @author REtz (radu.etz@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __ADE7880_H__
#define __ADE7880_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_print_log.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* SPI commands */
#define ADE7880_SPI_READ_CMD        	0x01
#define ADE7880_SPI_WRITE_CMD       	0x00

#define ENABLE                  	0x0001
#define DISABLE                 	0x0000

/* ADE7880 Register Map */
#define ADE7880_REG_AIGAIN		0x4380
#define ADE7880_REG_AVGAIN		0x4381
#define ADE7880_REG_BIGAIN		0x4382
#define ADE7880_REG_BVGAIN		0x4383
#define ADE7880_REG_CIGAIN		0x4384
#define ADE7880_REG_CVGAIN		0x4385
#define ADE7880_REG_NIGAIN		0x4386
#define ADE7880_REG_DICOEFF		0x4388
#define ADE7880_REG_APGAIN		0x4389
#define ADE7880_REG_AWATTOS		0x438A
#define ADE7880_REG_BPGAIN		0x438B
#define ADE7880_REG_BWATTOS		0x438C
#define ADE7880_REG_CPGAIN		0x438D
#define ADE7880_REG_CWATTOS		0x438E
#define ADE7880_REG_AIRMSOS 		0x438F
#define ADE7880_REG_AVRMSOS 		0x4390
#define ADE7880_REG_BIRMSOS		0x4391
#define ADE7880_REG_BVRMSOS		0x4392
#define ADE7880_REG_CIRMSOS		0x4393
#define ADE7880_REG_CVRMSOS 		0x4394
#define ADE7880_REG_NIRMSOS 		0x4395
#define ADE7880_REG_HPGAIN		0x4398
#define ADE7880_REG_ISUMLVL		0x4399
#define ADE7880_REG_VLEVEL		0x439F
#define ADE7880_REG_AFWATTOS		0x43A2
#define ADE7880_REG_BFWATTOS		0x43A3
#define ADE7880_REG_CFWATTOS		0x43A4
#define ADE7880_REG_AFVAROS		0x43A5
#define ADE7880_REG_BFVAROS     	0x43A6
#define ADE7880_REG_CFVAROS		0x43A7
#define ADE7880_REG_AFIRMSOS		0x43A8
#define ADE7880_REG_BFIRMSOS		0x43A9
#define ADE7880_REG_CFIRMSOS		0x43AA
#define ADE7880_REG_AFVRMSOS		0x43AB
#define ADE7880_REG_BFVRMSOS		0x43AC
#define ADE7880_REG_CFVRMSOS		0x43AD
#define ADE7880_REG_HXWATTOS		0x43AE
#define ADE7880_REG_HYWATTOS		0x43AF
#define ADE7880_REG_HZWATTOS		0x43B0
#define ADE7880_REG_HXVAROS 		0x43B1
#define ADE7880_REG_HYVAROS		0x43B2
#define ADE7880_REG_HZVAROS		0x43B3
#define ADE7880_REG_HXIRMSOS		0x43B4
#define ADE7880_REG_HYIRMSOS		0x43B5
#define ADE7880_REG_HZIRMSOS		0x43B6
#define ADE7880_REG_HXVRMSOS		0x43B7
#define ADE7880_REG_HYVRMSOS		0x43B8
#define ADE7880_REG_HZVRMSOS		0x43B9
#define ADE7880_REG_AIRMS		0x43C0
#define ADE7880_REG_AVRMS		0x43C1
#define ADE7880_REG_BIRMS		0x43C2
#define ADE7880_REG_BVRMS		0x43C3
#define ADE7880_REG_CIRMS		0x43C4
#define ADE7880_REG_CVRMS		0x43C5
#define ADE7880_REG_NIRMS		0x43C6
#define ADE7880_REG_ISUM		0x43C7

/* Internal DSP memory RAM registers*/
#define ADE7880_REG_RUN			0xE228

/* Billable registers*/
#define ADE7880_REG_AWATTHR		0xE400
#define ADE7880_REG_BWATTHR		0xE401
#define ADE7880_REG_CWATTHR		0xE402
#define ADE7880_REG_AFWATTHR		0xE403
#define ADE7880_REG_BFWATTHR		0xE404
#define ADE7880_REG_CFWATTHR		0xE405
#define ADE7880_REG_AFVARHR		0xE409
#define ADE7880_REG_BFVARHR		0xE40A
#define ADE7880_REG_CFVARHR		0xE40B
#define ADE7880_REG_AVAHR		0xE40C
#define ADE7880_REG_BVAHR		0xE40D
#define ADE7880_REG_CVAHR		0xE40E

/* Configuration and PQ registers */
#define ADE7880_REG_IPEAK		0xE500
#define ADE7880_REG_VPEAK		0xE501
#define ADE7880_REG_STATUS0		0xE502
#define ADE7880_REG_STATUS1		0xE503
#define ADE7880_REG_AIMAV	 	0xE504
#define ADE7880_REG_BIMAV 		0xE505
#define ADE7880_REG_CIMAV	 	0xE506
#define ADE7880_REG_OILVL	 	0xE507
#define ADE7880_REG_OVLVL	 	0xE508
#define ADE7880_REG_SAGLVL	 	0xE509
#define ADE7880_REG_MASK0	 	0xE50A
#define ADE7880_REG_MASK1	 	0xE50B
#define ADE7880_REG_IAWV	 	0xE50C
#define ADE7880_REG_IBWV	 	0xE50D
#define ADE7880_REG_ICWV	 	0xE50E
#define ADE7880_REG_INWV	 	0xE50F
#define ADE7880_REG_VAWV		0xE510
#define ADE7880_REG_VBWV		0xE511
#define ADE7880_REG_VCWV		0xE512
#define ADE7880_REG_AWATT		0xE513
#define ADE7880_REG_BWATT		0xE514
#define ADE7880_REG_CWATT		0xE515
#define ADE7880_REG_AVA			0xE519
#define ADE7880_REG_BVA			0xE51A
#define ADE7880_REG_CVA			0xE51B
#define ADE7880_REG_CHECKSUM		0xE51F
#define ADE7880_REG_VNOM		0xE520
#define ADE7880_REG_LAST_RWDATA32	0xE5FF
#define ADE7880_REG_PHSTATUS		0xE600
#define ADE7880_REG_ANGLE0		0xE601
#define ADE7880_REG_ANGLE1		0xE602
#define ADE7880_REG_ANGLE2		0xE603
#define ADE7880_REG_PHNOLOAD		0xE608
#define ADE7880_REG_LINECYC		0xE60C
#define ADE7880_REG_ZXTOUT		0xE60D
#define ADE7880_REG_COMPMODE		0xE60E
#define ADE7880_REG_GAIN		0xE60F
#define ADE7880_REG_CFMODE		0xE610
#define ADE7880_REG_CF1DEN		0xE611
#define ADE7880_REG_CF2DEN		0xE612
#define ADE7880_REG_CF3DEN		0xE613
#define ADE7880_REG_APHCAL		0xE614
#define ADE7880_REG_BPHCAL		0xE615
#define ADE7880_REG_CPHCAL		0xE616
#define ADE7880_REG_PHSIGN 		0xE617
#define ADE7880_REG_CONFIG		0xE618
#define ADE7880_REG_MMODE		0xE700
#define ADE7880_REG_ACCMODE		0xE701
#define ADE7880_REG_LCYCMODE		0xE702
#define ADE7880_REG_PEAKCYC		0xE703
#define ADE7880_REG_SAGCYC		0xE704
#define ADE7880_REG_CFCYC		0xE705
#define ADE7880_REG_HSDC_CFG		0xE706
#define ADE7880_REG_VERSION		0xE707
#define ADE7880_REG_RESERVED		0xE7E4
#define ADE7880_REG_LAST_RWDATA8	0xE7FD
#define ADE7880_REG_FVRMS		0xE880
#define ADE7880_REG_FIRMS		0xE881
#define ADE7880_REG_FWATT		0xE882
#define ADE7880_REG_FVAR		0xE883
#define ADE7880_REG_FVA 		0xE884
#define ADE7880_REG_FPF			0xE885
#define ADE7880_REG_VTHD		0xE886
#define ADE7880_REG_ITHD		0xE887
#define ADE7880_REG_HXVRMS		0xE888
#define ADE7880_REG_HXIRMS		0xE889
#define ADE7880_REG_HXWATT		0xE88A
#define ADE7880_REG_HXVAR		0xE88B
#define ADE7880_REG_HXVA		0xE88C
#define ADE7880_REG_HXPF		0xE88D
#define ADE7880_REG_HXVHD		0xE88E
#define ADE7880_REG_HXIHD		0xE88F
#define ADE7880_REG_HYVRMS		0xE890
#define ADE7880_REG_HYIRMS		0xE891
#define ADE7880_REG_HYWATT		0xE892
#define ADE7880_REG_HFVAR		0xE893
#define ADE7880_REG_HYVA		0xE894
#define ADE7880_REG_HYPF		0xE895
#define ADE7880_REG_HYVHD		0xE896
#define ADE7880_REG_HYIHD		0xE897
#define ADE7880_REG_HZVRMS		0xE898
#define ADE7880_REG_HZIRMS		0xE899
#define ADE7880_REG_HZWATT		0xE89A
#define ADE7880_REG_HZVAR		0xE89B
#define ADE7880_REG_HZVA		0xE89C
#define ADE7880_REG_HZPF		0xE89D
#define ADE7880_REG_HZVHD		0xE89E
#define ADE7880_REG_HZIHD		0xE89F
#define ADE7880_REG_HCONFIG		0xE900
#define ADE7880_REG_APF			0xE902
#define ADE7880_REG_BPF			0xE903
#define ADE7880_REG_CPF			0xE904
#define ADE7880_REG_APERIOD		0xE905
#define ADE7880_REG_BPERIOD		0xE906
#define ADE7880_REG_CPERIOD		0xE907
#define ADE7880_REG_APNOLOAD		0xE908
#define ADE7880_REG_VARNOLOAD		0xE909
#define ADE7880_REG_VANOLOAD		0xE90A
#define ADE7880_REG_LAST_ADD		0xE9FE
#define ADE7880_REG_LAST_RWDATA16	0xE9FF
#define ADE7880_REG_CONFIG3		0xEA00
#define ADE7880_REG_LAST_OP		0xEA01
#define ADE7880_REG_WTHR		0xEA02
#define ADE7880_REG_VARTHR 		0xEA03
#define ADE7880_REG_VATHR		0xEA04
#define ADE7880_REG_HX			0xEA08
#define ADE7880_REG_HY 			0xEA09
#define ADE7880_REG_HZ			0xEA0A
#define ADE7880_REG_LPOILVL		0xEC00
#define ADE7880_REG_CONFIG2 		0xEC01

/* ADE7880_REG_IPEAK Bit Definition */
#define ADE7880_IPPHASE2		NO_OS_BIT(26)
#define ADE7880_IPPHASE1		NO_OS_BIT(25)
#define ADE7880_IPPHASE0		NO_OS_BIT(24)
#define ADE7880_IPEAKVAL		NO_OS_GENMASK(23, 0)

/* ADE7880_REG_VPEAK Bit Definition */
#define ADE7880_VPPHASE2		NO_OS_BIT(26)
#define ADE7880_VPPHASE1		NO_OS_BIT(25)
#define ADE7880_VPPHASE0		NO_OS_BIT(24)
#define ADE7880_VPEAKVAL		NO_OS_GENMASK(23, 0)

/* ADE7880_REG_STATUS0 Bit Definition */
#define ADE7880_STATUS0_HREADY		NO_OS_BIT(19)
#define ADE7880_STATUS0_REVPSUM3	NO_OS_BIT(18)
#define ADE7880_STATUS0_DREADY		NO_OS_BIT(17)
#define ADE7880_STATUS0_CF3		NO_OS_BIT(16)
#define ADE7880_STATUS0_CF2		NO_OS_BIT(15)
#define ADE7880_STATUS0_CF1		NO_OS_BIT(14)
#define ADE7880_STATUS0_REVPSUM2	NO_OS_BIT(13)
#define ADE7880_STATUS0_REVFRPC		NO_OS_BIT(12)
#define ADE7880_STATUS0_REVFRPB 	NO_OS_BIT(11)
#define ADE7880_STATUS0_REVFRPA 	NO_OS_BIT(10)
#define ADE7880_STATUS0_REVPSUM1 	NO_OS_BIT(9)
#define ADE7880_STATUS0_REVAPC 		NO_OS_BIT(8)
#define ADE7880_STATUS0_REVAPB 		NO_OS_BIT(7)
#define ADE7880_STATUS0_REVAPA 		NO_OS_BIT(6)
#define ADE7880_STATUS0_LENERGY 	NO_OS_BIT(5)
#define ADE7880_STATUS0_VAEHF 		NO_OS_BIT(4)
#define ADE7880_STATUS0_FREHF 		NO_OS_BIT(3)
#define ADE7880_STATUS0_FAEHF 		NO_OS_BIT(1)
#define ADE7880_STATUS0_AEHF 		NO_OS_BIT(0)

/* ADE7880_REG_STATUS1 Bit Definition */
#define ADE7880_STATUS1_CRC		NO_OS_BIT(25)
#define ADE7880_STATUS1_PKV		NO_OS_BIT(24)
#define ADE7880_STATUS1_PKI		NO_OS_BIT(23)
#define ADE7880_STATUS1_MISMTCH		NO_OS_BIT(20)
#define ADE7880_STATUS1_SEQERR		NO_OS_BIT(19)
#define ADE7880_STATUS1_OV		NO_OS_BIT(18)
#define ADE7880_STATUS1_OI		NO_OS_BIT(17)
#define ADE7880_STATUS1_SAG		NO_OS_BIT(16)
#define ADE7880_STATUS1_RSTDONE		NO_OS_BIT(15)
#define ADE7880_STATUS1_ZXIC		NO_OS_BIT(14)
#define ADE7880_STATUS1_ZXIB		NO_OS_BIT(13)
#define ADE7880_STATUS1_ZXIA		NO_OS_BIT(12)
#define ADE7880_STATUS1_ZXVC		NO_OS_BIT(11)
#define ADE7880_STATUS1_ZXVB		NO_OS_BIT(10)
#define ADE7880_STATUS1_ZXVA		NO_OS_BIT(9)
#define ADE7880_STATUS1_ZXTOIC		NO_OS_BIT(8)
#define ADE7880_STATUS1_ZXTOIB		NO_OS_BIT(7)
#define ADE7880_STATUS1_ZXTOIA		NO_OS_BIT(6)
#define ADE7880_STATUS1_ZXTOVC		NO_OS_BIT(5)
#define ADE7880_STATUS1_ZXTOVB		NO_OS_BIT(4)
#define ADE7880_STATUS1_ZXTOVA		NO_OS_BIT(3)
#define ADE7880_STATUS1_VANLOAD		NO_OS_BIT(2)
#define ADE7880_STATUS1_FNLOAD		NO_OS_BIT(1)
#define ADE7880_STATUS1_NLOAD		NO_OS_BIT(0)

/* ADE7880_REG_MASK0 Bit Definition */
#define ADE7880_MASK0_HREADY		NO_OS_BIT(19)
#define ADE7880_MASK0_REVPSUM3 		NO_OS_BIT(18)
#define ADE7880_MASK0_DREADY		NO_OS_BIT(17)
#define ADE7880_MASK0_CF3		NO_OS_BIT(16)
#define ADE7880_MASK0_CF2		NO_OS_BIT(15)
#define ADE7880_MASK0_CF1		NO_OS_BIT(14)
#define ADE7880_MASK0_REVPSUM2		NO_OS_BIT(13)
#define ADE7880_MASK0_REVFRPC		NO_OS_BIT(12)
#define ADE7880_MASK0_REVFRPB		NO_OS_BIT(11)
#define ADE7880_MASK0_REVFRPA		NO_OS_BIT(10)
#define ADE7880_MASK0_REVPSUM1	 	NO_OS_BIT(9)
#define ADE7880_MASK0_REVAPC		NO_OS_BIT(8)
#define ADE7880_MASK0_REVAPB		NO_OS_BIT(7)
#define ADE7880_MASK0_REVAPA		NO_OS_BIT(6)
#define ADE7880_MASK0_LENERGY		NO_OS_BIT(5)
#define ADE7880_MASK0_VAEHF		NO_OS_BIT(4)
#define ADE7880_MASK0_FREHF		NO_OS_BIT(3)
#define ADE7880_MASK0_FAEHF		NO_OS_BIT(1)
#define ADE7880_MASK0_AEHF		NO_OS_BIT(0)

/* ADE7880_REG_MASK1 Bit Definition */
#define ADE7880_MASK1_CRC		NO_OS_BIT(25)
#define ADE7880_MASK1_PKV		NO_OS_BIT(24)
#define ADE7880_MASK1_PKI		NO_OS_BIT(23)
#define ADE7880_MASK1_MISMTCH		NO_OS_BIT(20)
#define ADE7880_MASK1_SEQERR		NO_OS_BIT(19)
#define ADE7880_MASK1_OV		NO_OS_BIT(18)
#define ADE7880_MASK1_OI		NO_OS_BIT(17)
#define ADE7880_MASK1_SAG		NO_OS_BIT(16)
#define ADE7880_MASK1_RSTDONE		NO_OS_BIT(15)
#define ADE7880_MASK1_ZXIC		NO_OS_BIT(14)
#define ADE7880_MASK1_ZXIB		NO_OS_BIT(13)
#define ADE7880_MASK1_ZXIA		NO_OS_BIT(12)
#define ADE7880_MASK1_ZXVC 		NO_OS_BIT(11)
#define ADE7880_MASK1_ZXVB		NO_OS_BIT(10)
#define ADE7880_MASK1_ZXVA		NO_OS_BIT(9)
#define ADE7880_MASK1_ZXTOIC		NO_OS_BIT(8)
#define ADE7880_MASK1_ZXTOIB		NO_OS_BIT(7)
#define ADE7880_MASK1_ZXTOIA		NO_OS_BIT(6)
#define ADE7880_MASK1_ZXTOVC		NO_OS_BIT(5)
#define ADE7880_MASK1_ZXTOVB		NO_OS_BIT(4)
#define ADE7880_MASK1_ZXTOVA		NO_OS_BIT(3)
#define ADE7880_MASK1_VANLOAD		NO_OS_BIT(2)
#define ADE7880_MASK1_FNLOAD		NO_OS_BIT(1)
#define ADE7880_MASK1_NLOAD		NO_OS_BIT(0)

/* ADE7880_REG_PHSTATUS Bit Definition */
#define ADE7880_VSPHASE2		NO_OS_BIT(14)
#define ADE7880_VSPHASE1		NO_OS_BIT(13)
#define ADE7880_VSPHASE0		NO_OS_BIT(12)
#define ADE7880_OVPHASE2		NO_OS_BIT(11)
#define ADE7880_OVPHASE1		NO_OS_BIT(10)
#define ADE7880_OVPHASE0		NO_OS_BIT(9)
#define ADE7880_OIPHASE2		NO_OS_BIT(5)
#define ADE7880_OIPHASE1		NO_OS_BIT(4)
#define ADE7880_OIPHASE0		NO_OS_BIT(3)

/* ADE7880_REG_PHNOLOAD Bit Definition */
#define ADE7880_VANLPHASE2		NO_OS_BIT(8)
#define ADE7880_VANLPHASE1		NO_OS_BIT(7)
#define ADE7880_VANLPHASE0		NO_OS_BIT(6)
#define ADE7880_FNLPHASE2		NO_OS_BIT(5)
#define ADE7880_FNLPHASE1		NO_OS_BIT(4)
#define ADE7880_FNLPHASE0		NO_OS_BIT(3)
#define ADE7880_NLPHASE2		NO_OS_BIT(2)
#define ADE7880_NLPHASE1		NO_OS_BIT(1)
#define ADE7880_NLPHASE0		NO_OS_BIT(0)

/* ADE7880_REG_COMPMODE Bit Definition */
#define ADE7880_SELFREQ			NO_OS_BIT(14)
#define ADE7880_VNOMCEN			NO_OS_BIT(13)
#define ADE7880_VNOMBEN			NO_OS_BIT(12)
#define ADE7880_VNOMAEN			NO_OS_BIT(11)
#define ADE7880_ANGLESEL		NO_OS_GENMASK(10, 9)
#define ADE7880_TERMSEL3_2		NO_OS_BIT(8)
#define ADE7880_TERMSEL3_1		NO_OS_BIT(7)
#define ADE7880_TERMSEL3_0		NO_OS_BIT(6)
#define ADE7880_TERMSEL2_2		NO_OS_BIT(5)
#define ADE7880_TERMSEL2_1		NO_OS_BIT(4)
#define ADE7880_TERMSEL2_0		NO_OS_BIT(3)
#define ADE7880_TERMSEL1_2		NO_OS_BIT(2)
#define ADE7880_TERMSEL1_1		NO_OS_BIT(1)
#define ADE7880_TERMSEL1_0		NO_OS_BIT(0)

/* ADE7880_REG_GAIN Bit Definition */
#define ADE7880_PGA3			NO_OS_GENMASK(8, 6)
#define ADE7880_PGA2			NO_OS_GENMASK(5, 3)
#define ADE7880_PGA1			NO_OS_GENMASK(2, 0)

/* ADE7880_REG_CFMODE Bit Definition */
#define ADE7880_CF3LATCH		NO_OS_BIT(14)
#define ADE7880_CF2LATCH		NO_OS_BIT(13)
#define ADE7880_CF1LATCH		NO_OS_BIT(12)
#define ADE7880_CF3DIS			NO_OS_BIT(11)
#define ADE7880_CF2DIS			NO_OS_BIT(10)
#define ADE7880_CF1DIS			NO_OS_BIT(9)
#define ADE7880_CF3SEL			NO_OS_GENMASK(8, 6)
#define ADE7880_CF2SEL			NO_OS_GENMASK(5, 3)
#define ADE7880_CF1SEL			NO_OS_GENMASK(2, 0)

/* ADE7880_REG_APHCAL, ADE7880_REG_BPHCAL,
ADE7880_REG_CPHCAL, Bit Definition */
#define ADE7880_PHCALVAL		NO_OS_GENMASK(9, 0)

/* ADE7880_REG_PHSIGN Bit Definition */
#define ADE7880_SUM3SIGN		NO_OS_BIT(8)
#define ADE7880_SUM2SIGN		NO_OS_BIT(7)
#define ADE7880_CFVARSIGN		NO_OS_BIT(6)
#define ADE7880_BFVARSIGN		NO_OS_BIT(5)
#define ADE7880_AFVARSIGN		NO_OS_BIT(4)
#define ADE7880_SUM1SIGN		NO_OS_BIT(3)
#define ADE7880_CWSIGN			NO_OS_BIT(2)
#define ADE7880_BWSIGN			NO_OS_BIT(1)
#define ADE7880_AWSIGN			NO_OS_BIT(0)

/* ADE7880_REG_CONFIG Bit Definition */
#define ADE7880_VTOIC			NO_OS_GENMASK(13, 12)
#define ADE7880_VTOIB			NO_OS_GENMASK(11, 10)
#define ADE7880_VTOIA			NO_OS_GENMASK(9, 8)
#define ADE7880_SWRST 			NO_OS_BIT(7)
#define ADE7880_HSDCEN 			NO_OS_BIT(6)
#define ADE7880_MOD2SHORT 		NO_OS_BIT(5)
#define ADE7880_MOD1SHORT 		NO_OS_BIT(4)
#define ADE7880_SWAP 			NO_OS_BIT(3)
#define ADE7880_CF2DIS 			NO_OS_BIT(2)
#define ADE7880_INTEN 			NO_OS_BIT(0)

/* ADE7880_REG_MMODE Bit Definition */
#define ADE7880_PEAKSEL2		NO_OS_BIT(4)
#define ADE7880_PEAKSEL1		NO_OS_BIT(3)
#define ADE7880_PEAKSEL0		NO_OS_BIT(2)

/* ADE7880_REG_ACCMODE Bit Definition */
#define ADE7880_REVAPSEL	 	NO_OS_BIT(6)
#define ADE7880_CONSEL	 		NO_OS_GENMASK(5, 4)
#define ADE7880_VARACC	 		NO_OS_GENMASK(3, 2)
#define ADE7880_WATTACC	 		NO_OS_GENMASK(1, 0)

/* ADE7880_REG_LCYCMODE Bit Definition */
#define ADE7880_PFMODE	 		NO_OS_BIT(7)
#define ADE7880_RSTREAD	 		NO_OS_BIT(6)
#define ADE7880_ZXSEL2	 		NO_OS_BIT(5)
#define ADE7880_ZXSEL1	 		NO_OS_BIT(4)
#define ADE7880_ZXSEL0	 		NO_OS_BIT(3)
#define ADE7880_LVA	 		NO_OS_BIT(2)
#define ADE7880_LVAR	 		NO_OS_BIT(1)
#define ADE7880_LWATT	 		NO_OS_BIT(0)

/* ADE7880_REG_HSDC_CFG Bit Definition */
#define ADE7880_HSAPOL			NO_OS_BIT(5)
#define ADE7880_HXFER	 		NO_OS_GENMASK(4, 3)
#define ADE7880_HGAP			NO_OS_BIT(2)
#define ADE7880_HSIZE			NO_OS_BIT(1)
#define ADE7880_HCLK			NO_OS_BIT(0)

/* ADE7880_REG_CONFIG3 Bit Definition */
#define ADE7880_ININTEN			NO_OS_BIT(3)
#define ADE7880_INSEL			NO_OS_BIT(2)
#define ADE7880_LPFSEL			NO_OS_BIT(1)
#define ADE7880_HPFEN			NO_OS_BIT(0)

/* ADE7880_REG_HCONFIG Bit Definition */
#define ADE7880_ACTPHSEL		NO_OS_GENMASK(9, 8)
#define ADE7880_HRATE			NO_OS_GENMASK(7, 5)
#define ADE7880_HSTIME			NO_OS_GENMASK(4, 3)
#define ADE7880_HPHASE			NO_OS_GENMASK(2, 1)
#define ADE7880_HRCFG			NO_OS_BIT(0)

/* ADE7880_REG_LPOILVL Bit Definition */
#define ADE7880_LPLINE			NO_OS_GENMASK(7, 3)
#define ADE7880_LPOIL			NO_OS_GENMASK(2, 0)

/* ADE7880_REG_CONFIG2 Bit Definition */
#define ADE7880_I2C_LOCK		NO_OS_BIT(1)
#define ADE7880_EXTREFEN		NO_OS_BIT(0)

/* Miscellaneous Definitions */
#define ADE7880_CHIP_ID			0x0EA0
#define ADE7880_RESET_RECOVER   	100
#define ADE7880_RAM_PROTECTION1 	0xE7FE
#define ADE7880_RAM_PROTECTION2 	0xE7E3
#define ADE7880_RAM_PROT_VAL1   	0xAD
#define ADE7880_RAM_PROT_VAL2   	0x80
#define ADE7880_SET_SPI_ADDR        	0xEBFF
#define ADE7880_DUMB_VAL            	0x01


/*Configuration registers*/
/* Set DICOEFF= 0xFFFF8000 when integrator is enabled*/
#define ADE7880_DICOEFF 		0x00000000
#define ADE7880_VLEVEL_VAL      	0x007A1200
/*Constant Definitions***/
/*DSP ON*/
#define ADE7880_RUN_ON 			0x0001
// /*Full scale Codes (FS) referred from Datasheet.*/
// /*Respective digital codes are produced when ADC inputs*/
// /*are at full scale. Do not Change. */
#define ADE7880_RMS_FS_CODES  		5326737

/* Assuming a transformer ratio of 1000:1 and 10 ohms burden resistance value */
#define ADE7880_BURDEN_RES              10
#define ADE7880_CURRENT_TR_RATIO        1000
#define ADE7880_CURRENT_TR_FCN          (ADE7880_CURRENT_TR_RATIO / ADE7880_BURDEN_RES)
/* Assuming a voltage divider with Rlow 1k and Rup 1MEG */
#define ADE7880_UP_RES                	1000000
#define ADE7880_DOWN_RES		1000
#define ADE7880_VOLTAGE_TR_FCN		((ADE7880_DOWN_RES + ADE7880_UP_RES) / ADE7880_DOWN_RES)

// 0.5V full scale * 0.707 * 10000 for mili units
#define ADE7880_FS_VOLTAGE           	3535

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ade7880_anglesel_e` is an enumeration that defines the different
 * modes of angle measurement available in the ADE7880 device. It allows
 * the selection of whether to measure angles between phase voltages and
 * currents, only between phase voltages, only between phase currents, or
 * to disable angle measurement altogether. This enumeration is used to
 * configure the ADE7880 for specific angle measurement requirements in
 * power monitoring applications.
 *
 * @param ADE7880_ANGLESEL_V_I The angles between phase voltages and phase
 * currents are measured.
 * @param ADE7880_ANGLESEL_V The angles between phase voltages are measured.
 * @param ADE7880_ANGLESEL_I The angles between phase currents are measured.
 * @param ADE7880_NO_ANGLESEL No angles are measured.
 ******************************************************************************/
enum ade7880_anglesel_e {
	/* The angles between phase voltages and phase currents are measured. */
	ADE7880_ANGLESEL_V_I,
	/* The angles between phase voltages are measured */
	ADE7880_ANGLESEL_V,
	/* The angles between phase currents are measured. */
	ADE7880_ANGLESEL_I,
	/* no angles are measured. */
	ADE7880_NO_ANGLESEL
};

/***************************************************************************//**
 * @brief The `ade7880_pga_gain_e` is an enumeration that defines the possible
 * gain settings for the ADE7880 device, specifically for phase currents,
 * neutral current, and phase voltages. Each enumerator corresponds to a
 * specific gain value, allowing the user to select the appropriate gain
 * setting for their application. This gain selection is crucial for
 * adjusting the sensitivity of the device to different signal levels.
 *
 * @param ADE7880_PGAGAIN_1 Represents a gain of 1.
 * @param ADE7880_PGAGAIN_2 Represents a gain of 2.
 * @param ADE7880_PGAGAIN_4 Represents a gain of 4.
 * @param ADE7880_PGAGAIN_8 Represents a gain of 8.
 * @param ADE7880_PGAGAIN_16 Represents a gain of 16.
 ******************************************************************************/
enum ade7880_pga_gain_e {
	/* gain 1 */
	ADE7880_PGAGAIN_1,
	/* gain 2 */
	ADE7880_PGAGAIN_2,
	/* gain 4 */
	ADE7880_PGAGAIN_4,
	/* gain 8 */
	ADE7880_PGAGAIN_8,
	/* gain 16 */
	ADE7880_PGAGAIN_16,
};

/***************************************************************************//**
 * @brief The `ade7880_cfxsel_e` is an enumeration that defines the different
 * modes for selecting the proportionality of the CFx frequency in the
 * ADE7880 device. Each enumerator represents a specific mode where the
 * CFx frequency is proportional to different types of power measurements
 * across the phases, such as total active power, apparent power,
 * fundamental active power, or fundamental reactive power. This
 * selection is crucial for configuring the device to measure and output
 * the desired power metrics.
 *
 * @param ADE7880_CFXSEL_0 The CFx frequency is proportional to the sum of total
 * active powers on each phase.
 * @param ADE7880_CFXSEL_1 The CFx frequency is proportional to the sum of
 * apparent powers on each phase.
 * @param ADE7880_CFXSEL_2 The CFx frequency is proportional to the sum of
 * fundamental active powers on each phase.
 * @param ADE7880_CFXSEL_3 The CFx frequency is proportional to the sum of
 * fundamental reactive powers on each phase.
 ******************************************************************************/
enum ade7880_cfxsel_e {
	/* the CFx frequency is proportional to the sum of total active powers on
	each phase */
	ADE7880_CFXSEL_0,
	/* the CFx frequency is proportional to the sum of apparent powers on
	each phase */
	ADE7880_CFXSEL_1 = 2,
	/* the CFx frequency is proportional to the sum of fundamental active
	powers on each phase */
	ADE7880_CFXSEL_2,
	/* the CFx frequency is proportional to the sum of fundamental reactive
	powers on each phase */
	ADE7880_CFXSEL_3
};

/***************************************************************************//**
 * @brief The `ade7880_vtoia_e` enumeration defines constants that specify which
 * phase voltage (A, B, or C) is considered in conjunction with Phase A
 * current in the power path of the ADE7880 device. This enumeration is
 * used to configure the device to measure and analyze the power path for
 * a specific phase voltage, allowing for precise energy monitoring and
 * management in multi-phase systems.
 *
 * @param ADE7880_VTOIA_A Represents the phase A voltage.
 * @param ADE7880_VTOIA_B Represents the phase B voltage.
 * @param ADE7880_VTOIA_C Represents the phase C voltage.
 ******************************************************************************/
enum ade7880_vtoia_e {
	/* Phase A voltage */
	ADE7880_VTOIA_A,
	/* Phase B voltage */
	ADE7880_VTOIA_B,
	/* Phase C voltage */
	ADE7880_VTOIA_C
};

/***************************************************************************//**
 * @brief The `ade7880_vtoib_e` enumeration defines constants that specify which
 * phase voltage (A, B, or C) is considered in conjunction with Phase B
 * current in the power path of the ADE7880 device. This enumeration is
 * used to configure the device to measure and analyze the power
 * characteristics of different phases in a three-phase electrical
 * system.
 *
 * @param ADE7880_VTOIB_B Represents the phase B voltage.
 * @param ADE7880_VTOIB_C Represents the phase C voltage.
 * @param ADE7880_VTOIB_A Represents the phase A voltage.
 ******************************************************************************/
enum ade7880_vtoib_e {
	/* Phase B voltage */
	ADE7880_VTOIB_B,
	/* Phase C voltage */
	ADE7880_VTOIB_C,
	/* Phase A voltage */
	ADE7880_VTOIB_A
};

/***************************************************************************//**
 * @brief The `ade7880_vtoic_e` enumeration defines constants that represent the
 * phase voltages (A, B, and C) associated with phase C current in the
 * power path of the ADE7880 device. This enumeration is used to specify
 * which phase voltage is considered in conjunction with the phase C
 * current for power calculations.
 *
 * @param ADE7880_VTOIC_C Represents the phase C voltage.
 * @param ADE7880_VTOIC_A Represents the phase A voltage.
 * @param ADE7880_VTOIC_B Represents the phase B voltage.
 ******************************************************************************/
enum ade7880_vtoic_e {
	/* Phase C voltage */
	ADE7880_VTOIC_C,
	/* Phase A voltage */
	ADE7880_VTOIC_A,
	/* Phase B voltage */
	ADE7880_VTOIC_B
};

/***************************************************************************//**
 * @brief The `ade7880_wattacc_e` enumeration defines the different modes of
 * accumulation for the total and fundamental active powers in the
 * ADE7880 device. It includes options for signed, positive-only, and
 * absolute accumulation, as well as a reserved setting that defaults to
 * the signed mode. This enumeration is used to configure how the device
 * accumulates power measurements, which is crucial for accurate energy
 * monitoring and management.
 *
 * @param ADE7880_WATTACC_SIGNED_ACC Signed accumulation mode of the total and
 * fundamental active powers.
 * @param ADE7880_WATTACC_POSITIVE_ACC Positive only accumulation mode of the
 * total and fundamental active powers.
 * @param ADE7880_WATTACC_RESERVED Reserved; behaves like WATTACC[1:0] = 00 when
 * set.
 * @param ADE7880_WATTACC_ABSOLUTE_ACC Absolute accumulation mode of the total
 * and fundamental active powers.
 ******************************************************************************/
enum ade7880_wattacc_e {
	/* Signed accumulation mode of the total and fundamental active powers */
	ADE7880_WATTACC_SIGNED_ACC,
	/* Positive only accumulation mode of the total and fundamental active powers */
	ADE7880_WATTACC_POSITIVE_ACC,
	/* Reserved. When set, the device behaves like WATTACC[1:0] = 00. */
	ADE7880_WATTACC_RESERVED,
	/* Absolute accumulation mode of the total and fundamental active powers */
	ADE7880_WATTACC_ABSOLUTE_ACC
};

/***************************************************************************//**
 * @brief The `ade7880_varacc_e` enumeration defines the different modes of
 * accumulation for fundamental reactive powers in the ADE7880 device. It
 * includes options for signed accumulation, absolute accumulation, and a
 * mode that depends on the sign of the fundamental active power. There
 * is also a reserved value that defaults to a specific behavior. This
 * enumeration is used to configure how the device processes and
 * accumulates reactive power data.
 *
 * @param ADE7880_VARACC_SIGNED_ACC Represents the signed accumulation mode of
 * the fundamental reactive powers.
 * @param ADE7880_VARACC_RESERVED Reserved value that behaves like VARACC[1:0] =
 * 00 when set.
 * @param ADE7880_VARACC_SIGN_WATTACC Accumulates fundamental reactive power
 * based on the sign of the fundamental
 * active power.
 * @param ADE7880_VARACC_ABSOLUTE_ACC Represents the absolute accumulation mode
 * of the fundamental reactive powers.
 ******************************************************************************/
enum ade7880_varacc_e {
	/* Signed accumulation mode of the fundamental reactive powers */
	ADE7880_VARACC_SIGNED_ACC,
	/* reserved. When set, the device behaves like VARACC[1:0] = 00. */
	ADE7880_VARACC_RESERVED,
	/* The fundamental reactive power is accumulated, depending on the sign of the
	fundamental active power */
	ADE7880_VARACC_SIGN_WATTACC,
	/* Absolute accumulation mode of the fundamental reactive powers */
	ADE7880_VARACC_ABSOLUTE_ACC
};

/***************************************************************************//**
 * @brief The `ade7880_consel_e` enumeration defines various configurations for
 * 3-phase electrical systems in the ADE7880 energy metering IC. Each
 * enumerator specifies a different wiring and sensor setup, allowing the
 * device to adapt to different types of electrical installations. This
 * flexibility is crucial for accurate energy measurement and monitoring
 * in diverse electrical environments.
 *
 * @param ADE7880_CONSEL_3P_3W Represents a 3-phase four wires configuration
 * with three voltage sensors.
 * @param ADE7880_CONSEL_3P_3W_DELTA Represents a 3-phase three wires delta
 * connection where BVRMS register contains
 * the rms value of VA-VC.
 * @param ADE7880_CONSEL_3P_4W Represents a 3-phase four wires configuration
 * with two voltage sensors.
 * @param ADE7880_CONSEL_3P_4W_DELTA Represents a 3-phase four wires delta
 * connection.
 ******************************************************************************/
enum ade7880_consel_e {
	/* 3-phase four wires with three voltage sensors */
	ADE7880_CONSEL_3P_3W,
	/* 3-phase three wires delta connection. In this mode, BVRMS register contains the rms
	value of VA-VC */
	ADE7880_CONSEL_3P_3W_DELTA,
	/* 3-phase four wires with two voltage sensors. */
	ADE7880_CONSEL_3P_4W,
	/* 3-phase four wires delta connection. */
	ADE7880_CONSEL_3P_4W_DELTA
};

/***************************************************************************//**
 * @brief The `ade7880_hxfer_e` enumeration defines the different data
 * transmission modes for the High-Speed Data Capture (HSDC) interface of
 * the ADE7880 device. Each enumerator specifies a distinct set of data
 * that the HSDC can transmit, ranging from a comprehensive set of
 * waveforms and power values to specific instantaneous current and
 * voltage values, or phase power values. The reserved enumerator ensures
 * backward compatibility by defaulting to a predefined behavior.
 *
 * @param ADE7880_HXFER_16 HSDC transmits sixteen 32-bit words including various
 * waveforms and power values.
 * @param ADE7880_HXFER_7 HSDC transmits seven instantaneous current and voltage
 * values.
 * @param ADE7880_HXFER_9 HSDC transmits nine instantaneous phase power values.
 * @param ADE7880_HXFER_RESERVED Reserved setting that defaults to HXFER[1:0] =
 * 00 behavior.
 ******************************************************************************/
enum ade7880_hxfer_e {
	/* HSDC transmits sixteen 32-bit words in the following order: IAWV, VAWV, IBWV, VBWV, ICWV,
	VCWV, INWV, AVA, BVA, CVA, AWATT, BWATT, CWATT, AFVAR, BFVAR, and CFVAR. */
	ADE7880_HXFER_16,
	/* HSDC transmits seven instantaneous values of currents and voltages: IAWV, VAWV,
	IBWV, VBWV, ICWV, VCWV, and INWV. */
	ADE7880_HXFER_7,
	/* HSDC transmits nine instantaneous values of phase powers: AVA, BVA, CVA, AWATT,
	BWATT, CWATT, AFVAR, BFVAR, and CFVAR. */
	ADE7880_HXFER_9,
	/* 11 = reserved. If set, the ADE7880 behaves as if HXFER[1:0] = 00. */
	ADE7880_HXFER_RESERVED
};

/***************************************************************************//**
 * @brief The `ade7880_hphase_e` enumeration defines the different phases or
 * neutral current that can be analyzed by the harmonic calculations
 * block in the ADE7880 device. It includes options for each of the three
 * phases (A, B, and C) as well as the neutral current, allowing the
 * device to perform harmonic analysis on the selected phase or current.
 *
 * @param ADE7880_HPHASE_A Represents Phase A voltage and current.
 * @param ADE7880_HPHASE_B Represents Phase B voltage and current.
 * @param ADE7880_HPHASE_C Represents Phase C voltage and current.
 * @param ADE7880_HPHASE_N Represents Neutral current.
 ******************************************************************************/
enum ade7880_hphase_e {
	/* Phase A voltage and current */
	ADE7880_HPHASE_A,
	/* Phase B voltage and current */
	ADE7880_HPHASE_B,
	/* Phase C voltage and current */
	ADE7880_HPHASE_C,
	/* Neutral current */
	ADE7880_HPHASE_N
};

/***************************************************************************//**
 * @brief The `ade7880_hstime_e` enumeration defines various delay periods in
 * milliseconds that determine when the HREADY bit in the STATUS0
 * register is set to 1, provided the HRCFG bit is set to 0. This is used
 * in the context of the ADE7880 device to manage timing for certain
 * operations.
 *
 * @param ADE7880_HSTIM_500 Represents a delay period of 500 milliseconds.
 * @param ADE7880_HSTIM_700 Represents a delay period of 750 milliseconds.
 * @param ADE7880_HSTIM_1000 Represents a delay period of 1000 milliseconds.
 * @param ADE7880_HSTIM_1250 Represents a delay period of 1250 milliseconds.
 ******************************************************************************/
enum ade7880_hstime_e {
	/* 500 ms */
	ADE7880_HSTIM_500,
	/* 750 ms */
	ADE7880_HSTIM_700,
	/* 1000 ms */
	ADE7880_HSTIM_1000,
	/* 1250 ms */
	ADE7880_HSTIM_1250
};

/***************************************************************************//**
 * @brief The `ade7880_hrate_e` is an enumeration that defines various update
 * rates for harmonic registers in the ADE7880 device. Each enumerator
 * corresponds to a specific time interval, ranging from 125 microseconds
 * to 1.024 seconds, or the option to disable harmonic calculations
 * entirely. This allows for flexible configuration of the harmonic data
 * update frequency, which is crucial for applications requiring precise
 * power quality measurements.
 *
 * @param ADE7880_HRATE_8K Represents a harmonic update rate of 125 µs (8 kHz).
 * @param ADE7880_HRATE_4K Represents a harmonic update rate of 250 µs (4 kHz).
 * @param ADE7880_HRATE_1K Represents a harmonic update rate of 1 ms (1 kHz).
 * @param ADE7880_HRATE_62_5 Represents a harmonic update rate of 16 ms (62.5
 * Hz).
 * @param ADE7880_HRATE_7_8125 Represents a harmonic update rate of 128 ms
 * (7.8125 Hz).
 * @param ADE7880_HRATE_1_953125 Represents a harmonic update rate of 512 ms
 * (1.953125 Hz).
 * @param ADE7880_HRATE_0_9765625 Represents a harmonic update rate of 1.024 sec
 * (0.9765625 Hz).
 * @param ADE7880_HRATE_DISABLED Indicates that harmonic calculations are
 * disabled.
 ******************************************************************************/
enum ade7880_hrate_e {
	/* 125 µs (8 kHz rate) */
	ADE7880_HRATE_8K,
	/* 250 µs (4 kHz rate) */
	ADE7880_HRATE_4K,
	/* 1 ms (1 kHz rate) */
	ADE7880_HRATE_1K,
	/* 16 ms (62.5 Hz rate) */
	ADE7880_HRATE_62_5,
	/* 128 ms (7.8125 Hz rate) */
	ADE7880_HRATE_7_8125,
	/* 512 ms (1.953125 Hz rate) */
	ADE7880_HRATE_1_953125,
	/* 1.024 sec (0.9765625 Hz rate) */
	ADE7880_HRATE_0_9765625,
	/* harmonic calculations disabled */
	ADE7880_HRATE_DISABLED
};

/***************************************************************************//**
 * @brief The `ade7880_actphsel_e` enumeration defines the possible phase
 * voltage selections used as a time base for harmonic calculations in
 * the ADE7880 device. It allows the user to specify which phase voltage
 * (A, B, or C) should be used, with an additional reserved option that
 * defaults to Phase C if chosen. This selection is crucial for accurate
 * harmonic analysis and power quality measurements in multi-phase
 * electrical systems.
 *
 * @param ADE7880_ACTPHSEL_A Represents the selection of Phase A voltage for
 * harmonic calculations.
 * @param ADE7880_ACTPHSEL_B Represents the selection of Phase B voltage for
 * harmonic calculations.
 * @param ADE7880_ACTPHSEL_C Represents the selection of Phase C voltage for
 * harmonic calculations.
 * @param ADE7880_ACTPHSEL_RESERVED Reserved value, defaults to using Phase C
 * voltage if selected.
 ******************************************************************************/
enum ade7880_actphsel_e {
	/* Phase A voltage */
	ADE7880_ACTPHSEL_A,
	/* Phase B voltage */
	ADE7880_ACTPHSEL_B,
	/* Phase C voltage */
	ADE7880_ACTPHSEL_C,
	/* reserved. If selected, phase C voltage is used */
	ADE7880_ACTPHSEL_RESERVED
};

/***************************************************************************//**
 * @brief The `ade7880_freq_sel_e` is an enumeration that defines the frequency
 * selection options for the ADE7880 device, specifically allowing the
 * user to choose between 50 Hz and 60 Hz. This selection is crucial for
 * configuring the device to operate correctly in regions with different
 * power line frequencies.
 *
 * @param ADE7880_SELFREQ_50 Represents a frequency selection of 50 Hz.
 * @param ADE7880_SELFREQ_60 Represents a frequency selection of 60 Hz.
 ******************************************************************************/
enum ade7880_freq_sel_e {
	/* 50 Hz */
	ADE7880_SELFREQ_50,
	/* 60 Hz */
	ADE7880_SELFREQ_60,
};

/***************************************************************************//**
 * @brief The `ade7880_phase` enumeration defines the available phases (A, B,
 * and C) for the ADE7880 device, which is used in power measurement and
 * monitoring applications. This enumeration allows the software to
 * reference specific phases when interacting with the device's
 * functionality, such as reading energy or power data for a particular
 * phase.
 *
 * @param ADE7880_PHASE_A Represents phase A in the ADE7880 device.
 * @param ADE7880_PHASE_B Represents phase B in the ADE7880 device.
 * @param ADE7880_PHASE_C Represents phase C in the ADE7880 device.
 ******************************************************************************/
enum ade7880_phase {
	ADE7880_PHASE_A,
	ADE7880_PHASE_B,
	ADE7880_PHASE_C
};

/***************************************************************************//**
 * @brief The `ade7880_power_mode_e` is an enumeration that defines the
 * different power modes available for the ADE7880 device. These modes
 * control the power consumption and functionality of the device, ranging
 * from full operation in NORMAL_MODE to minimal operation in SLEEP_MODE.
 * Each mode is designed to optimize the device's performance and power
 * usage based on the application's requirements.
 *
 * @param NORMAL_MODE Represents the normal power mode (PSM0) where the device
 * operates fully.
 * @param REDUCED_POWER_MODE Represents a reduced power mode (PSM1) where the
 * device measures the mean absolute values of the
 * 3-phase currents.
 * @param LOW_POWER_MODE Represents a low power mode (PSM2) for current peak
 * detection.
 * @param SLEEP_MODE Represents a sleep mode (PSM3) where the device is in a low
 * power state.
 ******************************************************************************/
enum ade7880_power_mode_e {
	/* SPI is not available in PSM2 & PSM3*/
	/* PSM0 normal mode */
	NORMAL_MODE,
	/* PSM1 measure the mean absolute values (mav)
	of the 3-phase currents */
	REDUCED_POWER_MODE,
	/* PSM2 Current peak detect mode */
	LOW_POWER_MODE,
	/* PSM3 Sleep mode */
	SLEEP_MODE
};

/***************************************************************************//**
 * @brief The `ade7880_init_param` structure is used to define the
 * initialization parameters for the ADE7880 device. It includes pointers
 * to SPI and GPIO descriptors necessary for setting up communication and
 * control modes, as well as a variable to select the power mode of the
 * device. This structure is essential for configuring the device before
 * it is used in applications that require energy measurement and
 * management.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param psm0_desc Pointer to the GPIO descriptor for the PSM0 mode.
 * @param psm1_desc Pointer to the GPIO descriptor for the PSM1 mode.
 * @param reset_desc Pointer to the GPIO descriptor for the reset functionality.
 * @param power_mode 8-bit unsigned integer representing the power mode
 * selection.
 ******************************************************************************/
struct ade7880_init_param {
	/** Device communication descriptor */
	struct no_os_spi_init_param 	*spi_init;
	/* psm0 descriptor */
	struct no_os_gpio_desc      	*psm0_desc;
	/* psm1 descriptor */
	struct no_os_gpio_desc      	*psm1_desc;
	/* reset descriptor */
	struct no_os_gpio_desc      	*reset_desc;
	/* Variable for mode selection */
	uint8_t             		power_mode;
};

/***************************************************************************//**
 * @brief The `ade7880_dev` structure is a comprehensive representation of the
 * ADE7880 device, encapsulating all necessary descriptors and variables
 * for its operation. It includes pointers to SPI and GPIO descriptors
 * for communication and control, as well as variables to store real-time
 * RMS values for current and voltage. Additionally, it maintains a field
 * for selecting the device's power mode, facilitating efficient power
 * management and operational control.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param psm0_desc Pointer to the GPIO descriptor for the PSM0 mode.
 * @param psm1_desc Pointer to the GPIO descriptor for the PSM1 mode.
 * @param reset_desc Pointer to the GPIO descriptor for the reset functionality.
 * @param irms_val Stores the instantaneous RMS current value.
 * @param vrms_val Stores the instantaneous RMS voltage value.
 * @param power_mode Indicates the current power mode of the device.
 ******************************************************************************/
struct ade7880_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/* psm0 descriptor */
	struct no_os_gpio_desc      	*psm0_desc;
	/* psm1 descriptor */
	struct no_os_gpio_desc      	*psm1_desc;
	/* reset descriptor */
	struct no_os_gpio_desc      	*reset_desc;
	/* Variable storing the IRMS value */
	uint32_t			irms_val;
	/* Variable storing the VRMS value */
	uint32_t			vrms_val;
	/* Variable for mode selection */
	uint8_t             		power_mode;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from the
 * ADE7880 device using SPI communication. It should be called when you
 * need to read data from the device's registers. The function requires a
 * valid device structure and a register address to read from. It
 * supports reading from 8-bit, 16-bit, and 32-bit registers,
 * automatically determining the register size based on the address.
 * Ensure that the device is properly initialized before calling this
 * function. The function returns an error code if the device structure
 * or the output data pointer is null, or if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ade7880_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address within the ADE7880's address space.
 * @param reg_data A pointer to a 32-bit variable where the read register value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure (e.g.,
 * -ENODEV if the device is null, -EINVAL if reg_data is null, or an
 * error code from SPI communication).
 ******************************************************************************/
int ade7880_read(struct ade7880_dev *dev, uint16_t reg_addr,
		 uint32_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function is used to write a 32-bit data value to a specified
 * register address on the ADE7880 device. It must be called with a valid
 * device structure that has been properly initialized. The function
 * handles different register sizes (8, 16, or 32 bits) based on the
 * register address provided. It returns an error code if the device
 * structure is null, indicating that the device is not available. This
 * function is essential for configuring the device or updating its
 * settings.
 *
 * @param dev A pointer to an initialized ade7880_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param reg_addr A 16-bit unsigned integer specifying the register address to
 * write to. The address determines the size of the register (8,
 * 16, or 32 bits).
 * @param reg_data A 32-bit unsigned integer containing the data to be written
 * to the specified register. The function handles the data size
 * based on the register address.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -ENODEV if the device structure is null.
 ******************************************************************************/
int ade7880_write(struct ade7880_dev *dev, uint16_t reg_addr,
		  uint32_t reg_data);

/* Set power mode */
/***************************************************************************//**
 * @brief This function configures the power mode of the ADE7880 device based on
 * the current power mode setting in the device structure. It must be
 * called with a valid device structure that has been properly
 * initialized, including valid GPIO descriptors for power mode control.
 * The function handles different power modes such as sleep, low power,
 * reduced power, and normal mode by setting the appropriate GPIO values.
 * It returns an error code if the device structure is null, if the GPIO
 * descriptors are not set, or if an invalid power mode is specified.
 *
 * @param dev A pointer to an initialized `ade7880_dev` structure. This
 * structure must not be null and must contain valid `psm0_desc` and
 * `psm1_desc` GPIO descriptors. The function will return an error if
 * these conditions are not met.
 * @return Returns 0 on success, or a negative error code on failure. Possible
 * error codes include -ENODEV if the device structure is null, -EINVAL
 * if the GPIO descriptors are not set or if an invalid power mode is
 * specified.
 ******************************************************************************/
int ade7880_set_power_mode(struct ade7880_dev *dev);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * ADE7880 device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets the new bits as
 * specified by the reg_data parameter. This function should be called
 * when you need to change only certain bits of a register without
 * affecting the others. It requires a valid device structure and a valid
 * register address. The function returns an error code if the device is
 * not initialized or if the read/write operations fail.
 *
 * @param dev A pointer to an initialized ade7880_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADE7880 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, masked by the
 * mask parameter. Only the bits specified by the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -ENODEV if the device is not initialized.
 ******************************************************************************/
int ade7880_update_bits(struct ade7880_dev *dev, uint16_t reg_addr,
			uint32_t mask, uint32_t reg_data);

/* Read Energy/Power for specific phase */
/***************************************************************************//**
 * @brief This function retrieves the instantaneous root mean square (RMS)
 * current and voltage values for a specified phase of the ADE7880
 * device. It should be called when you need to obtain the current and
 * voltage measurements for a particular phase (A, B, or C). The function
 * requires a valid device structure and a specified phase. It updates
 * the device structure with the RMS values in milliamps and millivolts,
 * respectively. Ensure the device is properly initialized before calling
 * this function. The function returns an error code if the device is not
 * available or if an invalid phase is specified.
 *
 * @param dev A pointer to an ade7880_dev structure representing the device.
 * Must not be null. The function will return -ENODEV if this
 * parameter is null.
 * @param phase An enum ade7880_phase value specifying the phase
 * (ADE7880_PHASE_A, ADE7880_PHASE_B, or ADE7880_PHASE_C) for which
 * to read data. If an invalid phase is provided, the function
 * returns -EINVAL.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -ENODEV for a null device pointer
 * or -EINVAL for an invalid phase). The irms_val and vrms_val fields of
 * the ade7880_dev structure are updated with the RMS current and
 * voltage values, respectively.
 ******************************************************************************/
int ade7880_read_data_ph(struct ade7880_dev *dev, enum ade7880_phase phase);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes an ADE7880 device using the provided
 * initialization parameters, setting up the necessary communication
 * interfaces and power modes. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure, configures the power mode, performs a hardware reset, and
 * initializes the SPI interface. If any step fails, it cleans up and
 * returns an error code. Ensure that all required descriptors in the
 * initialization parameters are valid and non-null before calling this
 * function.
 *
 * @param device A pointer to a pointer of ade7880_dev structure where the
 * initialized device instance will be stored. Must not be null.
 * The caller takes ownership of the allocated memory upon
 * successful initialization.
 * @param init_param A structure containing initialization parameters such as
 * SPI initialization parameters and GPIO descriptors for
 * power modes and reset. All pointers within this structure
 * must be valid and non-null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no resources are leaked.
 ******************************************************************************/
int ade7880_init(struct ade7880_dev **device,
		 struct ade7880_init_param init_param);

/* Setup the device */
/***************************************************************************//**
 * @brief This function sets up the ADE7880 device by configuring its registers
 * with initial values necessary for operation. It must be called after
 * the device has been initialized and before any other operations are
 * performed. The function configures channel gains, sets the frequency
 * to 50 Hz, initializes RAM registers, and enables DSP RAM protection
 * and the DSP itself. It returns an error code if any step fails,
 * ensuring that the device is not partially configured.
 *
 * @param dev A pointer to an initialized `ade7880_dev` structure representing
 * the device. Must not be null. If null, the function returns
 * -ENOMEM.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * which step failed.
 ******************************************************************************/
int ade7880_setup(struct ade7880_dev *dev);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function is used to properly remove an ADE7880 device instance
 * and free associated resources. It should be called when the device is
 * no longer needed, ensuring that any allocated resources are released
 * and the device is properly shut down. This function must be called
 * after the device has been initialized and used, to prevent resource
 * leaks. It handles the removal of the SPI descriptor and frees the
 * memory allocated for the device structure. If the SPI removal fails,
 * the function returns an error code.
 *
 * @param dev A pointer to an initialized `ade7880_dev` structure representing
 * the device to be removed. Must not be null. The function will free
 * the memory associated with this structure.
 * @return Returns 0 on successful removal and resource release, or a negative
 * error code if the SPI removal fails.
 ******************************************************************************/
int ade7880_remove(struct ade7880_dev *dev);

/* Get interrupt indicator from STATUS0 register. */
/***************************************************************************//**
 * @brief This function is used to obtain the interrupt status from the STATUS0
 * register of the ADE7880 device. It should be called when you need to
 * check specific interrupt conditions indicated by the mask. The
 * function requires a valid device structure and a non-null pointer for
 * storing the status. It returns an error code if the device is not
 * initialized or if the status pointer is null.
 *
 * @param dev A pointer to an initialized ade7880_dev structure representing the
 * device. Must not be null.
 * @param msk A 32-bit mask indicating which bits in the STATUS0 register to
 * check. Valid values depend on the specific bits of interest.
 * @param status A pointer to a uint8_t where the resulting status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * initialized or if the status pointer is null.
 ******************************************************************************/
int ade7880_get_int_status0(struct ade7880_dev *dev, uint32_t msk,
			    uint8_t *status);

#endif // __ADE7880_H__

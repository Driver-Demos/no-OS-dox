/***************************************************************************//**
 *   @file   ade9078.h
 *   @brief  Header file of ADE9078 Driver.
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
#ifndef __ADE9078_H__
#define __ADE9078_H__

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
#define ADE9078_SPI_READ		NO_OS_BIT(3)

#define ENABLE                  	0x0001
#define DISABLE                 	0x0000

/* ADE9078 Register Map */
#define ADE9078_REG_AIGAIN		0x0000
#define ADE9078_REG_AIGAIN0		0x0001
#define ADE9078_REG_AIGAIN1		0x0002
#define ADE9078_REG_AIGAIN2		0x0003
#define ADE9078_REG_AIGAIN3		0x0004
#define ADE9078_REG_AIGAIN4		0x0005
#define ADE9078_REG_APHCAL0		0x0006
#define ADE9078_REG_APHCAL1		0x0007
#define ADE9078_REG_APHCAL2		0x0008
#define ADE9078_REG_APHCAL3		0x0009
#define ADE9078_REG_APHCAL4		0x000A
#define ADE9078_REG_AVGAIN		0x000B
#define ADE9078_REG_AIRMSOS		0x000C
#define ADE9078_REG_AVRMSOS		0x000D
#define ADE9078_REG_APGAIN		0x000E
#define ADE9078_REG_AWATTOS		0x000F
#define ADE9078_REG_AVAROS		0x0010
#define ADE9078_REG_AFVAROS		0x0012
#define ADE9078_REG_BIGAIN		0x0020
#define ADE9078_REG_BIGAIN0		0x0021
#define ADE9078_REG_BIGAIN1		0x0022
#define ADE9078_REG_BIGAIN2		0x0023
#define ADE9078_REG_BIGAIN3		0x0024
#define ADE9078_REG_BIGAIN4		0x0025
#define ADE9078_REG_BPHCAL0 		0x0026
#define ADE9078_REG_BPHCAL1		0x0027
#define ADE9078_REG_BPHCAL2 		0x0028
#define ADE9078_REG_BPHCAL3		0x0029
#define ADE9078_REG_BPHCAL4		0x002A
#define ADE9078_REG_BVGAIN		0x002B
#define ADE9078_REG_BIRMSOS		0x002C
#define ADE9078_REG_BVRMSOS		0x002D
#define ADE9078_REG_BPGAIN		0x002E
#define ADE9078_REG_BWATTOS		0x002F
#define ADE9078_REG_BVAROS		0x0030
#define ADE9078_REG_BFVAROS		0x0032
#define ADE9078_REG_CIGAIN		0x0040
#define ADE9078_REG_CIGAIN0		0x0041
#define ADE9078_REG_CIGAIN1		0x0042
#define ADE9078_REG_CIGAIN2		0x0043
#define ADE9078_REG_CIGAIN3		0x0044
#define ADE9078_REG_CIGAIN4		0x0045
#define ADE9078_REG_CPHCAL0		0x0046
#define ADE9078_REG_CPHCAL1		0x0047
#define ADE9078_REG_CPHCAL2		0x0048
#define ADE9078_REG_CPHCAL3		0x0049
#define ADE9078_REG_CPHCAL4		0x004A
#define ADE9078_REG_CVGAIN		0x004B
#define ADE9078_REG_CIRMSOS		0x004C
#define ADE9078_REG_CVRMSOS		0x004D
#define ADE9078_REG_CPGAIN		0x004E
#define ADE9078_REG_CWATTOS		0x004F
#define ADE9078_REG_CVAROS		0x0050
#define ADE9078_REG_CFVAROS		0x0052
#define ADE9078_REG_CONFIG0		0x0060
#define ADE9078_REG_MTTHR_L0		0x0061
#define ADE9078_REG_MTTHR_L1 		0x0062
#define ADE9078_REG_MTTHR_L2 		0x0063
#define ADE9078_REG_MTTHR_L3		0x0064
#define ADE9078_REG_MTTHR_L4		0x0065
#define ADE9078_REG_MTTHR_H0		0x0066
#define ADE9078_REG_MTTHR_H1 		0x0067
#define ADE9078_REG_MTTHR_H2 		0x0068
#define ADE9078_REG_MTTHR_H3		0x0069
#define ADE9078_REG_MTTHR_H4		0x006A
#define ADE9078_REG_NIRMSOS		0x006B
#define ADE9078_REG_ISUMRMSOS		0x006C
#define ADE9078_REG_NIGAIN		0x006D
#define ADE9078_REG_NPHCAL		0x006E
#define ADE9078_REG_VNOM		0x0071
#define ADE9078_REG_DICOEFF     	0x0072
#define ADE9078_REG_ISUMLVL		0x0073
#define ADE9078_REG_AI_PCF		0x020A
#define ADE9078_REG_AV_PCF		0x020B
#define ADE9078_REG_AIRMS		0x020C
#define ADE9078_REG_AVRMS		0x020D
#define ADE9078_REG_AWATT		0x0210
#define ADE9078_REG_AVAR		0x0211
#define ADE9078_REG_AVA			0x0212
#define ADE9078_REG_AFVAR		0x0214
#define ADE9078_REG_APF			0x0216
#define ADE9078_REG_AMTREGION 		0x021D
#define ADE9078_REG_BI_PCF		0x022A
#define ADE9078_REG_BV_PCF		0x022B
#define ADE9078_REG_BIRMS		0x022C
#define ADE9078_REG_BVRMS		0x022D
#define ADE9078_REG_BWATT		0x0230
#define ADE9078_REG_BVAR		0x0231
#define ADE9078_REG_BVA			0x0232
#define ADE9078_REG_BFVAR		0x0234
#define ADE9078_REG_BPF			0x0236
#define ADE9078_REG_BMTREGION		0x023D
#define ADE9078_REG_CI_PCF		0x024A
#define ADE9078_REG_CV_PCF		0x024B
#define ADE9078_REG_CIRMS		0x024C
#define ADE9078_REG_CVRMS		0x024D
#define ADE9078_REG_CWATT		0x0250
#define ADE9078_REG_CVAR		0x0251
#define ADE9078_REG_CVA			0x0252
#define ADE9078_REG_CFVAR		0x0254
#define ADE9078_REG_CPF			0x0256
#define ADE9078_REG_CMTREGION		0x025D
#define ADE9078_REG_NI_PCF		0x0265
#define ADE9078_REG_NIRMS		0x0266
#define ADE9078_REG_ISUMRMS		0x0269
#define ADE9078_REG_VERSION2		0x026A
#define ADE9078_REG_AWATT_ACC		0x02E5
#define ADE9078_REG_AWATTHR_LO		0x02E6
#define ADE9078_REG_AWATTHR_HI		0x02E7
#define ADE9078_REG_AVAR_ACC		0x02EF
#define ADE9078_REG_AVARHR_LO		0x02F0
#define ADE9078_REG_AVARHR_HI		0x02F1
#define ADE9078_REG_AVA_ACC		0x02F9
#define ADE9078_REG_AVAHR_LO		0x02FA
#define ADE9078_REG_AVAHR_HI		0x02FB
#define ADE9078_REG_AFVAR_ACC	 	0x030D
#define ADE9078_REG_AFVARHR_LO 		0x030E
#define ADE9078_REG_AFVARHR_HI	 	0x030F
#define ADE9078_REG_BWATT_ACC	 	0x0321
#define ADE9078_REG_BWATTHR_LO	 	0x0322
#define ADE9078_REG_BWATTHR_HI	 	0x0323
#define ADE9078_REG_BVAR_ACC	 	0x032B
#define ADE9078_REG_BVARHR_LO	 	0x032C
#define ADE9078_REG_BVARHR_HI	 	0x032D
#define ADE9078_REG_BVA_ACC	 	0x0335
#define ADE9078_REG_BVAHR_LO	 	0x0336
#define ADE9078_REG_BVAHR_HI	 	0x0337
#define ADE9078_REG_BFVAR_ACC		0x0349
#define ADE9078_REG_BFVARHR_LO		0x034A
#define ADE9078_REG_BFVARHR_HI		0x034B
#define ADE9078_REG_CWATT_ACC		0x035D
#define ADE9078_REG_CWATTHR_LO		0x035E
#define ADE9078_REG_CWATTHR_HI		0x035F
#define ADE9078_REG_CVAR_ACC		0x0367
#define ADE9078_REG_CVARHR_LO		0x0368
#define ADE9078_REG_CVARHR_HI		0x0369
#define ADE9078_REG_CVA_ACC		0x0371
#define ADE9078_REG_CVAHR_LO		0x0372
#define ADE9078_REG_CVAHR_HI		0x0373
#define ADE9078_REG_CFVAR_ACC		0x0385
#define ADE9078_REG_CFVARHR_LO		0x0386
#define ADE9078_REG_CFVARHR_HI		0x0387
#define ADE9078_REG_PWATT_ACC		0x0397
#define ADE9078_REG_NWATT_ACC		0x039B
#define ADE9078_REG_PVAR_ACC		0x039F
#define ADE9078_REG_NVAR_ACC		0x03A3
#define ADE9078_REG_IPEAK		0x0400
#define ADE9078_REG_VPEAK		0x0401
#define ADE9078_REG_STATUS0		0x0402
#define ADE9078_REG_STATUS1		0x0403
#define ADE9078_REG_EVENT_STATUS	0x0404
#define ADE9078_REG_MASK0		0x0405
#define ADE9078_REG_MASK1		0x0406
#define ADE9078_REG_EVENT_MASK		0x0407
#define ADE9078_REG_USER_PERIOD		0x040E
#define ADE9078_REG_VLEVEL 		0x040F
#define ADE9078_REG_APERIOD		0x0418
#define ADE9078_REG_BPERIOD		0x0419
#define ADE9078_REG_CPERIOD		0x041A
#define ADE9078_REG_COM_PERIOD		0x041B
#define ADE9078_REG_ACT_NL_LVL		0x041C
#define ADE9078_REG_REACT_NL_LVL	0x041D
#define ADE9078_REG_APP_NL_LVL		0x041E
#define ADE9078_REG_PHNOLOAD		0x041F
#define ADE9078_REG_WTHR		0x0420
#define ADE9078_REG_VARTHR		0x0421
#define ADE9078_REG_VATHR		0x0422
#define ADE9078_REG_LAST_DATA_32	0x0423
#define ADE9078_REG_ADC_REDIRECT	0x0424
#define ADE9078_REG_CF_LCFG		0x0425
#define ADE9078_REG_PART_ID		0x0472
#define ADE9078_REG_RUN 		0x0480
#define ADE9078_REG_CONFIG1		0x0481
#define ADE9078_REG_ANGL_VA_VB		0x0482
#define ADE9078_REG_ANGL_VB_VC		0x0483
#define ADE9078_REG_ANGL_VA_VC		0x0484
#define ADE9078_REG_ANGL_VA_IA		0x0485
#define ADE9078_REG_ANGL_VB_IB		0x0486
#define ADE9078_REG_ANGL_VC_IC		0x0487
#define ADE9078_REG_ANGL_IA_IB		0x0488
#define ADE9078_REG_ANGL_IB_IC		0x0489
#define ADE9078_REG_ANGL_IA_IC		0x048A
#define ADE9078_REG_CFMODE		0x0490
#define ADE9078_REG_COMPMODE		0x0491
#define ADE9078_REG_ACCMODE		0x0492
#define ADE9078_REG_CONFIG3		0x0493
#define ADE9078_REG_CF1DEN		0x0494
#define ADE9078_REG_CF2DEN		0x0495
#define ADE9078_REG_CF3DEN		0x0496
#define ADE9078_REG_CF4DEN		0x0497
#define ADE9078_REG_ZXTOUT		0x0498
#define ADE9078_REG_ZXTHRSH		0x0499
#define ADE9078_REG_ZX_LP_SEL		0x049A
#define ADE9078_REG_SEQ_CYC		0x049C
#define ADE9078_REG_PHSIGN		0x049D
#define ADE9078_REG_WFB_CFG		0x04A0
#define ADE9078_REG_WFB_PG_IRQEN	0x04A1
#define ADE9078_REG_WFB_TRG_CFG		0x04A2
#define ADE9078_REG_WFB_TRG_STAT	0x04A3
#define ADE9078_REG_CONFIG5		0x04A4
#define ADE9078_REG_CRC_RSLT		0x04A8
#define ADE9078_REG_CRC_SPI		0x04A9
#define ADE9078_REG_LAST_DATA_16	0x04AC
#define ADE9078_REG_LAST_CMD		0x04AE
#define ADE9078_REG_CONFIG2		0x04AF
#define ADE9078_REG_EP_CFG		0x04B0
#define ADE9078_REG_PWR_TIME		0x04B1
#define ADE9078_REG_EGY_TIME		0x04B2
#define ADE9078_REG_CRC_FORCE		0x04B4
#define ADE9078_REG_CRC_OPTEN		0x04B5
#define ADE9078_REG_TEMP_CFG		0x04B6
#define ADE9078_REG_PSM2_CFG		0x04B8
#define ADE9078_REG_PGA_GAIN		0x04B9
#define ADE9078_REG_CHNL_DIS		0x04BA
#define ADE9078_REG_WR_LOCK 		0x04BF
#define ADE9078_REG_VAR_DIS		0x04E0
#define ADE9078_REG_RESERVED1		0x04F0
#define ADE9078_REG_VERSION 		0x04FE
#define ADE9078_REG_AI_SINC_DAT		0x0500
#define ADE9078_REG_AV_SINC_DAT		0x0501
#define ADE9078_REG_BI_SINC_DAT 	0x0502
#define ADE9078_REG_BV_SINC_DAT 	0x0503
#define ADE9078_REG_CI_SINC_DAT 	0x0504
#define ADE9078_REG_CV_SINC_DAT 	0x0505
#define ADE9078_REG_NI_SINC_DAT 	0x0506
#define ADE9078_REG_AI_LPF_DAT	 	0x0510
#define ADE9078_REG_AV_LPF_DAT	 	0x0511
#define ADE9078_REG_BI_LPF_DAT	 	0x0512
#define ADE9078_REG_BV_LPF_DAT	 	0x0513
#define ADE9078_REG_CI_LPF_DAT	 	0x0514
#define ADE9078_REG_CV_LPF_DAT	 	0x0515
#define ADE9078_REG_NI_LPF_DAT	 	0x0516
#define ADE9078_REG_AV_PCF_1	 	0x0600
#define ADE9078_REG_BV_PCF_1	 	0x0601
#define ADE9078_REG_CV_PCF_1	 	0x0602
#define ADE9078_REG_NI_PCF_1	 	0x0603
#define ADE9078_REG_AI_PCF_1	 	0x0604
#define ADE9078_REG_BI_PCF_1	 	0x0605
#define ADE9078_REG_CI_PCF_1	 	0x0606
#define ADE9078_REG_AIRMS_1	 	0x0607
#define ADE9078_REG_BIRMS_1	 	0x0608
#define ADE9078_REG_CIRMS_1	 	0x0609
#define ADE9078_REG_AVRMS_1	 	0x060A
#define ADE9078_REG_BVRMS_1	 	0x060B
#define ADE9078_REG_CVRMS_1	 	0x060C
#define ADE9078_REG_NIRMS_1	 	0x060D
#define ADE9078_REG_AWATT_1	 	0x060E
#define ADE9078_REG_BWATT_1	 	0x060F
#define ADE9078_REG_CWATT_1	 	0x0610
#define ADE9078_REG_AVA_1	 	0x0611
#define ADE9078_REG_BVA_1	 	0x0612
#define ADE9078_REG_CVA_1	 	0x0613
#define ADE9078_REG_AVAR_1	 	0x0614
#define ADE9078_REG_BVAR_1	 	0x0615
#define ADE9078_REG_CVAR_1	 	0x0616
#define ADE9078_REG_AFVAR_1	 	0x0617
#define ADE9078_REG_BFVAR_1	 	0x0618
#define ADE9078_REG_CFVAR_1	 	0x0619
#define ADE9078_REG_APF_1	 	0x061A
#define ADE9078_REG_BPF_1	 	0x061B
#define ADE9078_REG_CPF_1	 	0x061C
#define ADE9078_REG_AV_PCF_2	 	0x0680
#define ADE9078_REG_AI_PCF_2	 	0x0681
#define ADE9078_REG_AIRMS_2	 	0x0682
#define ADE9078_REG_AVRMS_2	 	0x0683
#define ADE9078_REG_AWATT_2	 	0x0684
#define ADE9078_REG_AVA_2	 	0x0685
#define ADE9078_REG_AVAR_2 	 	0x0686
#define ADE9078_REG_AFVAR_2 	 	0x0687
#define ADE9078_REG_APF_2 	 	0x0688
#define ADE9078_REG_BV_PCF_2	 	0x0693
#define ADE9078_REG_BI_PCF_2	 	0x0694
#define ADE9078_REG_BIRMS_2	 	0x0695
#define ADE9078_REG_BVRMS_2	 	0x0696
#define ADE9078_REG_BWATT_2 	 	0x0697
#define ADE9078_REG_BVA_2 	 	0x0698
#define ADE9078_REG_BVAR_2 	 	0x0699
#define ADE9078_REG_BFVAR_2 	 	0x069A
#define ADE9078_REG_BPF_2 	 	0x069B
#define ADE9078_REG_CV_PCF_2  	 	0x06A6
#define ADE9078_REG_CI_PCF_2  	 	0x06A7
#define ADE9078_REG_CIRMS_2  	 	0x06A8
#define ADE9078_REG_CVRMS_2  	 	0x06A9
#define ADE9078_REG_CWATT_2  	 	0x06AA
#define ADE9078_REG_CVA_2  	 	0x06AB
#define ADE9078_REG_CVAR_2  	 	0x06AC
#define ADE9078_REG_CFVAR_2  	 	0x06AD
#define ADE9078_REG_CPF_2  	 	0x06AE
#define ADE9078_REG_NI_PCF_2	 	0x06B9
#define ADE9078_REG_NIRMS_2 	 	0x06BA

/* ADE9078_REG_CONFIG0 Bit Definition */
#define ADE9078_DISRPLPF		NO_OS_BIT(13)
#define ADE9078_DISAPLPF		NO_OS_BIT(12)
#define ADE9078_ININTEN			NO_OS_BIT(11)
#define ADE9078_VNOMC_EN		NO_OS_BIT(10)
#define ADE9078_VNOMB_EN		NO_OS_BIT(9)
#define ADE9078_VNOMA_EN 		NO_OS_BIT(8)
#define ADE9078_ZX_SRC_SEL 		NO_OS_BIT(6)
#define ADE9078_INTEN 			NO_OS_BIT(5)
#define ADE9078_MTEN 			NO_OS_BIT(4)
#define ADE9078_HPFDIS 			NO_OS_BIT(3)
#define ADE9078_ISUM_CFG		NO_OS_GENMASK(1, 0)

/* ADE9078_REG_AMTREGION Bit Definition */
#define ADE9078_AREGION			NO_OS_GENMASK(3, 0)

/* ADE9078_REG_BMTREGION Bit Definition */
#define ADE9078_BREGION			NO_OS_GENMASK(3, 0)

/* ADE9078_REG_CMTREGION Bit Definition */
#define ADE9078_CREGION			NO_OS_GENMASK(3, 0)

/* ADE9078_REG_IPEAK Bit Definition */
#define ADE9078_IPPHASE			NO_OS_GENMASK(26, 24)
#define ADE9078_IPEAKVAL		NO_OS_GENMASK(23, 0)

/* ADE9078_REG_VPEAK Bit Definition */
#define ADE9078_VPPHASE			NO_OS_GENMASK(26, 24)
#define ADE9078_VPEAKVAL		NO_OS_GENMASK(23, 0)

/* ADE9078_REG_STATUS0 Bit Definition */
#define ADE9078_STATUS0_MISMTCH		NO_OS_BIT(24)
#define ADE9078_STATUS0_COH_WFB_FULL	NO_OS_BIT(23)
#define ADE9078_STATUS0_WFB_TRIG	NO_OS_BIT(22)
#define ADE9078_STATUS0_PF_RDY		NO_OS_BIT(21)
#define ADE9078_STATUS0_PWRRDY		NO_OS_BIT(18)
#define ADE9078_STATUS0_PAGE_FULL	NO_OS_BIT(17)
#define ADE9078_STATUS0_WFB_TRIG_IRQ	NO_OS_BIT(16)
#define ADE9078_STATUS0_DREADY		NO_OS_BIT(15)
#define ADE9078_STATUS0_CF4 		NO_OS_BIT(14)
#define ADE9078_STATUS0_CF3 		NO_OS_BIT(13)
#define ADE9078_STATUS0_CF2 		NO_OS_BIT(12)
#define ADE9078_STATUS0_CF1 		NO_OS_BIT(11)
#define ADE9078_STATUS0_REVPSUM4 	NO_OS_BIT(10)
#define ADE9078_STATUS0_REVPSUM3 	NO_OS_BIT(9)
#define ADE9078_STATUS0_REVPSUM2 	NO_OS_BIT(8)
#define ADE9078_STATUS0_REVPSUM1 	NO_OS_BIT(7)
#define ADE9078_STATUS0_REVRPC 		NO_OS_BIT(6)
#define ADE9078_STATUS0_REVRPB 		NO_OS_BIT(5)
#define ADE9078_STATUS0_REVRPA 		NO_OS_BIT(4)
#define ADE9078_STATUS0_REVAPC 		NO_OS_BIT(3)
#define ADE9078_STATUS0_REVAPB 		NO_OS_BIT(2)
#define ADE9078_STATUS0_REVAPA 		NO_OS_BIT(1)
#define ADE9078_STATUS0_EGYRDY		NO_OS_BIT(0)

/* ADE9078_REG_STATUS1 Bit Definition */
#define ADE9078_STATUS1_ERROR3		NO_OS_BIT(31)
#define ADE9078_STATUS1_ERROR2		NO_OS_BIT(30)
#define ADE9078_STATUS1_ERROR1		NO_OS_BIT(29)
#define ADE9078_STATUS1_ERROR0		NO_OS_BIT(28)
#define ADE9078_STATUS1_CRC_DONE	NO_OS_BIT(27)
#define ADE9078_STATUS1_CRC_CHG		NO_OS_BIT(26)
#define ADE9078_STATUS1_SEQERR		NO_OS_BIT(18)
#define ADE9078_STATUS1_RSTDONE		NO_OS_BIT(16)
#define ADE9078_STATUS1_ZXIC		NO_OS_BIT(15)
#define ADE9078_STATUS1_ZXIB		NO_OS_BIT(14)
#define ADE9078_STATUS1_ZXIA		NO_OS_BIT(13)
#define ADE9078_STATUS1_ZXCOMB		NO_OS_BIT(12)
#define ADE9078_STATUS1_ZXVC		NO_OS_BIT(11)
#define ADE9078_STATUS1_ZXVB		NO_OS_BIT(10)
#define ADE9078_STATUS1_ZXVA		NO_OS_BIT(9)
#define ADE9078_STATUS1_ZXTOVC		NO_OS_BIT(8)
#define ADE9078_STATUS1_ZXTOVB		NO_OS_BIT(7)
#define ADE9078_STATUS1_ZXTOVA		NO_OS_BIT(6)
#define ADE9078_STATUS1_RFNOLOAD	NO_OS_BIT(4)
#define ADE9078_STATUS1_VANLOAD		NO_OS_BIT(2)
#define ADE9078_STATUS1_RNLOAD		NO_OS_BIT(1)
#define ADE9078_STATUS1_ANLOAD		NO_OS_BIT(0)

/* ADE9078_REG_EVENT_STATUS Bit Definition */
#define ADE9078_EVENT_DREADY		NO_OS_BIT(16)
#define ADE9078_EVENT_RFNOLOAD 		NO_OS_BIT(14)
#define ADE9078_EVENT_VANLOAD		NO_OS_BIT(12)
#define ADE9078_EVENT_RNLOAD		NO_OS_BIT(11)
#define ADE9078_EVENT_ANLOAD		NO_OS_BIT(10)
#define ADE9078_EVENT_REVPSUM4		NO_OS_BIT(9)
#define ADE9078_EVENT_REVPSUM3		NO_OS_BIT(8)
#define ADE9078_EVENT_REVPSUM2		NO_OS_BIT(7)
#define ADE9078_EVENT_REVPSUM1		NO_OS_BIT(6)

/* ADE9078_REG_MASK0 Bit Definition */
#define ADE9078_MASK0_MISMTCH		NO_OS_BIT(24)
#define ADE9078_MASK0_COH_WFB_FULL 	NO_OS_BIT(23)
#define ADE9078_MASK0_WFB_TRIG		NO_OS_BIT(22)
#define ADE9078_MASK0_THD_PF_RDY	NO_OS_BIT(21)
#define ADE9078_MASK0_PWRRDY		NO_OS_BIT(18)
#define ADE9078_MASK0_PAGE_FULL		NO_OS_BIT(17)
#define ADE9078_MASK0_WFB_TRIG_IRQ	NO_OS_BIT(16)
#define ADE9078_MASK0_DREADY		NO_OS_BIT(15)
#define ADE9078_MASK0_CF4		NO_OS_BIT(14)
#define ADE9078_MASK0_CF3		NO_OS_BIT(13)
#define ADE9078_MASK0_CF2	 	NO_OS_BIT(12)
#define ADE9078_MASK0_CF1		NO_OS_BIT(11)
#define ADE9078_MASK0_REVPSUM4		NO_OS_BIT(10)
#define ADE9078_MASK0_REVPSUM3		NO_OS_BIT(9)
#define ADE9078_MASK0_REVPSUM2		NO_OS_BIT(8)
#define ADE9078_MASK0_REVPSUM1		NO_OS_BIT(7)
#define ADE9078_MASK0_REVRPC		NO_OS_BIT(6)
#define ADE9078_MASK0_REVRPB		NO_OS_BIT(5)
#define ADE9078_MASK0_REVRPA		NO_OS_BIT(4)
#define ADE9078_MASK0_REVAPC		NO_OS_BIT(3)
#define ADE9078_MASK0_REVAPB		NO_OS_BIT(2)
#define ADE9078_MASK0_REVAPA		NO_OS_BIT(1)
#define ADE9078_MASK0_EGYRDY		NO_OS_BIT(0)

/* ADE9078_REG_MASK1 Bit Definition */
#define ADE9078_MASK1_ERROR3		NO_OS_BIT(31)
#define ADE9078_MASK1_ERROR2		NO_OS_BIT(30)
#define ADE9078_MASK1_ERROR1		NO_OS_BIT(29)
#define ADE9078_MASK1_ERROR0		NO_OS_BIT(28)
#define ADE9078_MASK1_CRC_DONE		NO_OS_BIT(27)
#define ADE9078_MASK1_CRC_CHG		NO_OS_BIT(26)
#define ADE9078_MASK1_SEQERR		NO_OS_BIT(18)
#define ADE9078_MASK1_ZXIC		NO_OS_BIT(15)
#define ADE9078_MASK1_ZXIB		NO_OS_BIT(14)
#define ADE9078_MASK1_ZXIA		NO_OS_BIT(13)
#define ADE9078_MASK1_ZXCOMB		NO_OS_BIT(12)
#define ADE9078_MASK1_ZXVC		NO_OS_BIT(11)
#define ADE9078_MASK1_ZXVB 		NO_OS_BIT(10)
#define ADE9078_MASK1_ZXVA		NO_OS_BIT(9)
#define ADE9078_MASK1_ZXTOVC		NO_OS_BIT(8)
#define ADE9078_MASK1_ZXTOVB		NO_OS_BIT(7)
#define ADE9078_MASK1_ZXTOVA		NO_OS_BIT(6)
#define ADE9078_MASK1_RFNOLOAD		NO_OS_BIT(4)
#define ADE9078_MASK1_VANLOAD		NO_OS_BIT(2)
#define ADE9078_MASK1_RNLOAD		NO_OS_BIT(1)
#define ADE9078_MASK1_ANLOAD		NO_OS_BIT(0)

/* ADE9078_REG_EVENT_MASK Bit Definition */
#define ADE9078_EVENT_DREADY_MSK	NO_OS_BIT(16)
#define ADE9078_EVENT_RFNOLOAD_MSK	NO_OS_BIT(14)
#define ADE9078_EVENT_VANLOAD_MSK	NO_OS_BIT(12)
#define ADE9078_EVENT_RNLOAD_MSK	NO_OS_BIT(11)
#define ADE9078_EVENT_ANLOAD_MSK	NO_OS_BIT(10)
#define ADE9078_EVENT_REVPSUM4_MSK	NO_OS_BIT(9)
#define ADE9078_EVENT_REVPSUM3_MSK	NO_OS_BIT(8)
#define ADE9078_EVENT_REVPSUM2_MSK	NO_OS_BIT(7)
#define ADE9078_EVENT_REVPSUM1_MSK	NO_OS_BIT(6)

/* ADE9078_REG_VLEVEL Bit Definition */
#define ADE9078_VLEVEL_VAL 		NO_OS_GENMASK(23, 0)

/* ADE9078_REG_PHNOLOAD Bit Definition */
#define ADE9078_CFVARNL			NO_OS_BIT(16)
#define ADE9078_CVANL			NO_OS_BIT(14)
#define ADE9078_CVARNL			NO_OS_BIT(13)
#define ADE9078_CWATTNL			NO_OS_BIT(12)
#define ADE9078_BFVARNL			NO_OS_BIT(10)
#define ADE9078_BVANL			NO_OS_BIT(8)
#define ADE9078_BVARNL			NO_OS_BIT(7)
#define ADE9078_BWATTNL			NO_OS_BIT(6)
#define ADE9078_AFVARNL			NO_OS_BIT(4)
#define ADE9078_AVANL			NO_OS_BIT(2)
#define ADE9078_AVARNL			NO_OS_BIT(1)
#define ADE9078_AWATTNL			NO_OS_BIT(0)

/* ADE9078_REG_ADC_REDIRECT Bit Definition */
#define ADE9078_VC_DIN	 		NO_OS_GENMASK(20, 18)
#define ADE9078_VB_DIN	 		NO_OS_GENMASK(17, 15)
#define ADE9078_VA_DIN	 		NO_OS_GENMASK(14, 12)
#define ADE9078_IN_DIN	 		NO_OS_GENMASK(11, 9)
#define ADE9078_IC_DIN	 		NO_OS_GENMASK(8, 6)
#define ADE9078_IB_DIN	 		NO_OS_GENMASK(5, 3)
#define ADE9078_IA_DIN	 		NO_OS_GENMASK(2, 0)

/* ADE9078_REG_CF_LCFG Bit Definition */
#define ADE9078_CF4_LT			NO_OS_BIT(22)
#define ADE9078_CF3_LT			NO_OS_BIT(21)
#define ADE9078_CF2_LT			NO_OS_BIT(20)
#define ADE9078_CF1_LT			NO_OS_BIT(19)
#define ADE9078_CF_LTMR			NO_OS_GENMASK(18, 0)

/* ADE9078_REG_PART_ID Bit Definition */
#define ADE9078_AD73370_ID		NO_OS_BIT(21)
#define ADE9078_ADE9000_ID		NO_OS_BIT(20)
#define ADE9078_ADE9004_ID		NO_OS_BIT(16)

/* ADE9078_REG_CONFIG1 Bit Definition */
#define ADE9078_EXT_REF			NO_OS_BIT(15)
#define ADE9078_IRQ0_ON_IRQ1		NO_OS_BIT(12)
#define ADE9078_BURST_EN		NO_OS_BIT(11)
#define ADE9078_PWR_SETTLE		NO_OS_GENMASK(9, 8)
#define ADE9078_CF_ACC_CLR		NO_OS_BIT(5)
#define ADE9078_CF4_CFG			NO_OS_GENMASK(3, 2)
#define ADE9078_CF3_CFG			NO_OS_BIT(1)
#define ADE9078_SWRST			NO_OS_BIT(0)

/* ADE9078_REG_CFMODE Bit Definition */
#define ADE9078_CF4DIS			NO_OS_BIT(15)
#define ADE9078_CF3DIS			NO_OS_BIT(14)
#define ADE9078_CF2DIS			NO_OS_BIT(13)
#define ADE9078_CF1DIS			NO_OS_BIT(12)
#define ADE9078_CF4SEL			NO_OS_GENMASK(11, 9)
#define ADE9078_CF3SEL			NO_OS_GENMASK(8, 6)
#define ADE9078_CF2SEL			NO_OS_GENMASK(5, 3)
#define ADE9078_CF1SEL 			NO_OS_GENMASK(2, 0)

/* ADE9078_REG_COMPMODE Bit Definition */
#define ADE9078_TERMSEL4		NO_OS_GENMASK(11, 9)
#define ADE9078_TERMSEL3		NO_OS_GENMASK(8, 6)
#define ADE9078_TERMSEL2		NO_OS_GENMASK(5, 3)
#define ADE9078_TERMSEL1 		NO_OS_GENMASK(2, 0)

/* ADE9078_REG_ACCMODE Bit Definition */
#define ADE9078_SELFREQ			NO_OS_BIT(8)
#define ADE9078_ICONSEL			NO_OS_BIT(7)
#define ADE9078_VCONSEL			NO_OS_GENMASK(6, 4)
#define ADE9078_VARACC	 		NO_OS_GENMASK(3, 2)
#define ADE9078_WATTACC			NO_OS_GENMASK(1, 0)

/* ADE9078_REG_CONFIG3 Bit Definition */
#define ADE9078_PEAKSEL	 		NO_OS_GENMASK(4, 2)

/* ADE9078_REG_ZX_LP_SEL Bit Definition */
#define ADE9078_LP_SEL			NO_OS_GENMASK(4, 3)
#define ADE9078_ZX_SEL	 		NO_OS_GENMASK(2, 1)

/* ADE9078_REG_PHSIGN Bit Definition */
#define ADE9078_SUM4SIGN		NO_OS_BIT(9)
#define ADE9078_SUM3SIGN		NO_OS_BIT(8)
#define ADE9078_SUM2SIGN		NO_OS_BIT(7)
#define ADE9078_SUM1SIGN		NO_OS_BIT(6)
#define ADE9078_CVARSIGN		NO_OS_BIT(5)
#define ADE9078_CWSIGN			NO_OS_BIT(4)
#define ADE9078_BVARSIGN 		NO_OS_BIT(3)
#define ADE9078_BWSIGN			NO_OS_BIT(2)
#define ADE9078_AVARSIGN		NO_OS_BIT(1)
#define ADE9078_AWSIGN			NO_OS_BIT(0)

/* ADE9078_REG_WFB_CFG Bit Definition */
#define ADE9078_WF_IN_EN		NO_OS_BIT(12)
#define ADE9078_WF_SRC			NO_OS_GENMASK(9, 8)
#define ADE9078_WF_MODE			NO_OS_BIT(7, 6)
#define ADE9078_WF_CAP_SEL		NO_OS_BIT(5)
#define ADE9078_WF_CAP_EN		NO_OS_BIT(4)
#define ADE9078_BURST_CHAN		NO_OS_GENMASK(3, 0)

/* ADE9078_WFB_TRG_CFG Bit Definition */
#define ADE9078_TRIG_FORCE		NO_OS_BIT(10)
#define ADE9078_ZXCOMB			NO_OS_BIT(9)
#define ADE9078_ZXVC			NO_OS_BIT(8)
#define ADE9078_ZXVB 			NO_OS_BIT(7)
#define ADE9078_ZXVA			NO_OS_BIT(6)
#define ADE9078_ZXIC			NO_OS_BIT(5)
#define ADE9078_ZXIB			NO_OS_BIT(4)
#define ADE9078_ZXIA			NO_OS_BIT(3)

/* ADE9078_WFB_TRG_STAT Bit Definition */
#define ADE9078_WFB_LAST_PAGE		NO_OS_GENMASK(15, 12)
#define ADE9078_WFB_TRIG_ADDR		NO_OS_GENMASK(10, 0)

/* ADE9078_CONFIG2 Bit Definition */
#define ADE9078_UPERIOD_SEL		NO_OS_BIT(12)
#define ADE9078_HPF_CRN			NO_OS_GENMASK(11, 9)

/* ADE9078_EP_CFG Bit Definition */
#define ADE9078_NOLOAD_TMR		NO_OS_GENMASK(15, 13)
#define ADE9078_PWR_SIGN_SEL_1		NO_OS_BIT(7)
#define ADE9078_RD_RST_EN		NO_OS_BIT(5)
#define ADE9078_EGY_LD_ACCUM		NO_OS_BIT(4)
#define ADE9078_EGY_TMR_MODE		NO_OS_BIT(1)
#define ADE9078_EGY_PWR_EN		NO_OS_BIT(0)

/* ADE9078_CRC_FORCE Bit Definition */
#define ADE9078_FORCE_CRC_UPDATE	NO_OS_BIT(0)

/* ADE9078_CRC_OPTEN Bit Definition */
#define ADE9078_CRC_WFB_TRG_CFG_EN	NO_OS_BIT(15)
#define ADE9078_CRC_WFB_PG_IRQEN	NO_OS_BIT(14)
#define ADE9078_CRC_WFB_CFG_EN		NO_OS_BIT(13)
#define ADE9078_CRC_SEQ_CYC_EN		NO_OS_BIT(12)
#define ADE9078_CRC_ZXLPSEL_EN		NO_OS_BIT(11)
#define ADE9078_CRC_ZXTOUT_EN		NO_OS_BIT(10)
#define ADE9078_CRC_APP_NL_LVL_EN	NO_OS_BIT(9)
#define ADE9078_CRC_REACT_NL_LVL_EN	NO_OS_BIT(8)
#define ADE9078_CRC_ACT_NL_LVL_EN	NO_OS_BIT(7)
#define ADE9078_CRC_EVENT_MASK_EN	NO_OS_BIT(2)
#define ADE9078_CRC_MASK1_EN		NO_OS_BIT(1)
#define ADE9078_CRC_MASK0_EN		NO_OS_BIT(0)

/* ADE9078_PSM2_CFG Bit Definition */
#define ADE9078_PKDET_LVL		NO_OS_BIT(8, 5)
#define ADE9078_LPLINE			NO_OS_BIT(4, 0)

/* ADE9078_PGA_GAIN Bit Definition */
#define ADE9078_VC_GAIN			NO_OS_GENMASK(13, 12)
#define ADE9078_VB_GAIN			NO_OS_GENMASK(11, 10)
#define ADE9078_VA_GAIN			NO_OS_GENMASK(9, 8)
#define ADE9078_IN_GAIN			NO_OS_GENMASK(7, 6)
#define ADE9078_IC_GAIN			NO_OS_GENMASK(5, 4)
#define ADE9078_IB_GAIN			NO_OS_GENMASK(3, 2)
#define ADE9078_IA_GAIN			NO_OS_GENMASK(1, 0)

/* ADE9078_CHNL_DIS Bit Definition */
#define ADE9078_VC_DISADC		NO_OS_BIT(6)
#define ADE9078_VB_DISADC		NO_OS_BIT(5)
#define ADE9078_VA_DISADC		NO_OS_BIT(4)
#define ADE9078_IN_DISADC		NO_OS_BIT(3)
#define ADE9078_IC_DISADC		NO_OS_BIT(2)
#define ADE9078_IB_DISADC		NO_OS_BIT(1)
#define ADE9078_IA_DISADC		NO_OS_BIT(0)

/* ADE9078_VAR_DIS Bit Definition */
#define ADE9078_VARDIS			NO_OS_BIT(0)

/* Miscellaneous Definitions */
#define ADE9078_CHIP_ID			0x63
#define ADE9078_PART_ID         	0
#define ADE9078_RESET_RECOVER   	100

/*Configuration registers*/
/*PGA@0x0000. Gain of all channels=1*/
#define ADE9078_PGA_GAIN 		0x0000
/*Integrator disabled*/
#define ADE9078_CONFIG0 		0x00000000
/*CF3/ZX pin outputs Zero crossing */
#define ADE9078_CONFIG1 		0x0002
/*Default High pass corner frequency of 1.2475Hz*/
#define ADE9078_CONFIG2 		0x0A00
/*Peak and overcurrent detection disabled*/
#define ADE9078_CONFIG3 		0x0000
/*50Hz operation, 3P4W Wye configuration, signed accumulation*/
#define ADE9078_ACCMODE 		0x0000
/*Line period and zero crossing obtained from combined signals VA,VB and VC*/
#define ADE9078_ZX_LP_SEL 		0x001E
/*Enable EGYRDY interrupt*/
#define ADE9078_MASK0 			0x00000001
/*MASK1 interrupts disabled*/
#define ADE9078_MASK1 			0x00000000
/*Events disabled */
#define ADE9078_EVENT_MASK 		0x00000000
/*Assuming Vnom=1/2 of full scale.*/
/*Refer Technical reference manual for detailed calculations.*/
#define ADE9078_VLEVEL  		0x00117514
/* Set DICOEFF= 0xFFFFE000 when integrator is enabled*/
#define ADE9078_DICOEFF 		0x00000000
/*Constant Definitions***/
/*DSP ON*/
#define ADE9078_RUN_ON 			0x0001
/*Energy Accumulation Settings*/
/*Enable energy accumulation, accumulate samples at 8ksps*/
/*latch energy accumulation after EGYRDY*/
/*If accumulation is changed to half line cycle mode, change EGY_TIME*/
#define ADE9078_EP_CFG 			0x0001
/*Accumulate 8000 samples*/
#define ADE9078_EGY_TIME 		0x1F3F
/*Waveform buffer Settings*/
/*Neutral current samples enabled, Resampled data enabled*/
/*Burst all channels*/
#define ADE9078_WFB_CFG 		0x1000
/*size of buffer to read. 512 Max.Each element IA,VA...IN has max 512 points*/
/*[Size of waveform buffer/number of sample sets = 2048/4 = 512]*/
/*(Refer ADE9078 technical reference manual for more details)*/
#define WFB_ELEMENT_ARRAY_SIZE 		512
/*Full scale Codes (FS) referred from Datasheet.*/
/*Respective digital codes are produced when ADC inputs*/
/*are at full scale. Do not Change. */
#define ADE9078_RMS_FS_CODES  		52866837
#define ADE9078_WATT_FS_CODES 		20823646

/* Assuming a transformer ratio of 1000:1 and 10 ohms burden resistance value */
#define ADE9078_BURDEN_RES              10
#define ADE9078_CURRENT_TR_RATIO        1000
#define ADE9078_CURRENT_TR_FCN          (ADE9078_CURRENT_TR_RATIO / ADE9078_BURDEN_RES)
/* Assuming a voltage divider with Rlow 1k and Rup 990k */
#define ADE9078_UP_RES                	990000
#define ADE9078_DOWN_RES		1000
#define ADE9078_VOLTAGE_TR_FCN		((ADE9078_DOWN_RES + ADE9078_UP_RES) / ADE9078_DOWN_RES)

// 0.707V rms full scale * 1000 for mili units
#define ADE9078_FS_VOLTAGE           	707

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ade9078_isum_cfg_e` is an enumeration that defines various
 * configuration options for calculating the neutral current RMS in the
 * ADE9078 energy metering IC. It includes options for approximating the
 * neutral current RMS, as well as detecting positive and negative
 * mismatches between neutral and phase currents. This enumeration is
 * used to configure the behavior of the ADE9078 in terms of how it
 * processes and calculates current mismatches and RMS values.
 *
 * @param ADE9078_ISUM_APROX_N Represents an approximated neutral current RMS
 * calculation.
 * @param ADE9078_ISUM_DET_MISM_POS Determines positive mismatch between neutral
 * and phase currents.
 * @param ADE9078_ISUM_DET_MISM_NEG Determines negative mismatch between neutral
 * and phase currents.
 * @param ADE9078_ISUM_APROX_N_RMS Represents an approximated neutral current
 * RMS calculation.
 ******************************************************************************/
enum ade9078_isum_cfg_e {
	/* Approximated neutral current rms calculation */
	ADE9078_ISUM_APROX_N,
	/* Determine positive mismatch between neutral and
	phase currents */
	ADE9078_ISUM_DET_MISM_POS,
	/* determine negative mismatch between neutral and
	phase currents */
	ADE9078_ISUM_DET_MISM_NEG,
	/* approximated neutral current rms calculation */
	ADE9078_ISUM_APROX_N_RMS
};

/***************************************************************************//**
 * @brief The `ade9078_aregion_sel_e` enumeration defines a set of constants
 * used to select which AIGAINx and APHCALx settings are currently being
 * used in the ADE9078 device. These settings are used for gain and phase
 * calibration of the device, with options ranging from 0 to 4 for
 * different calibration settings, and an option to disable the function
 * entirely. This enumeration is part of the configuration for the
 * ADE9078, a device used for energy measurement and monitoring.
 *
 * @param ADE9078_AIGAIN_APHCAL_0 Represents the first gain and phase
 * calibration setting.
 * @param ADE9078_AIGAIN_APHCAL_1 Represents the second gain and phase
 * calibration setting.
 * @param ADE9078_AIGAIN_APHCAL_2 Represents the third gain and phase
 * calibration setting.
 * @param ADE9078_AIGAIN_APHCAL_3 Represents the fourth gain and phase
 * calibration setting.
 * @param ADE9078_AIGAIN_APHCAL_4 Represents the fifth gain and phase
 * calibration setting.
 * @param ADE9078_AIGAIN_APHCAL_DISABLE Indicates that the function is disabled,
 * with a value of 15.
 ******************************************************************************/
enum ade9078_aregion_sel_e {
	/* 0 */
	ADE9078_AIGAIN_APHCAL_0,
	/* 1 */
	ADE9078_AIGAIN_APHCAL_1,
	/* 2 */
	ADE9078_AIGAIN_APHCAL_2,
	/* 3 */
	ADE9078_AIGAIN_APHCAL_3,
	/* 4 */
	ADE9078_AIGAIN_APHCAL_4,
	/* function disabled */
	ADE9078_AIGAIN_APHCAL_DISABLE = 15
};

/***************************************************************************//**
 * @brief The `ade9078_bregion_sel_e` enumeration defines the possible gain and
 * phase calibration settings for the B region in the ADE9078 device.
 * Each enumerator corresponds to a specific calibration setting, with
 * the option to disable the function entirely. This enumeration is used
 * to select which BIGAINx and BPHCALx settings are currently active,
 * allowing for precise control over the calibration of the B region.
 *
 * @param ADE9078_BIGAIN_BPHCAL_0 Represents the first gain and phase
 * calibration setting for B region.
 * @param ADE9078_BIGAIN_BPHCAL_1 Represents the second gain and phase
 * calibration setting for B region.
 * @param ADE9078_BIGAIN_BPHCAL_2 Represents the third gain and phase
 * calibration setting for B region.
 * @param ADE9078_BIGAIN_BPHCAL_3 Represents the fourth gain and phase
 * calibration setting for B region.
 * @param ADE9078_BIGAIN_BPHCAL_4 Represents the fifth gain and phase
 * calibration setting for B region.
 * @param ADE9078_BIGAIN_BPHCAL_DISABLE Indicates that the gain and phase
 * calibration function is disabled.
 ******************************************************************************/
enum ade9078_bregion_sel_e {
	/* 0 */
	ADE9078_BIGAIN_BPHCAL_0,
	/* 1 */
	ADE9078_BIGAIN_BPHCAL_1,
	/* 2 */
	ADE9078_BIGAIN_BPHCAL_2,
	/* 3 */
	ADE9078_BIGAIN_BPHCAL_3,
	/* 4 */
	ADE9078_BIGAIN_BPHCAL_4,
	/* function disabled */
	ADE9078_BIGAIN_BPHCAL_DISABLE = 15
};

/***************************************************************************//**
 * @brief The `ade9078_cregion_sel_e` enumeration defines the selection of
 * CIGAIN and CPHCAL configurations for the ADE9078 device. It provides
 * options to select from five different configurations, each
 * corresponding to a specific gain and phase calibration setting, or to
 * disable the function entirely. This enumeration is used to manage and
 * switch between different calibration settings for the C phase in the
 * ADE9078 energy metering IC.
 *
 * @param ADE9078_CIGAIN_CPHCAL_0 Represents the first configuration of CIGAIN
 * and CPHCAL.
 * @param ADE9078_CIGAIN_CPHCAL_1 Represents the second configuration of CIGAIN
 * and CPHCAL.
 * @param ADE9078_CIGAIN_CPHCAL_2 Represents the third configuration of CIGAIN
 * and CPHCAL.
 * @param ADE9078_CIGAIN_CPHCAL_3 Represents the fourth configuration of CIGAIN
 * and CPHCAL.
 * @param ADE9078_CIGAIN_CPHCAL_4 Represents the fifth configuration of CIGAIN
 * and CPHCAL.
 * @param ADE9078_CIGAIN_CPHCAL_DISABLE Indicates that the CIGAIN and CPHCAL
 * function is disabled.
 ******************************************************************************/
enum ade9078_cregion_sel_e {
	/* 0 */
	ADE9078_CIGAIN_CPHCAL_0,
	/* 1 */
	ADE9078_CIGAIN_CPHCAL_1,
	/* 2 */
	ADE9078_CIGAIN_CPHCAL_2,
	/* 3 */
	ADE9078_CIGAIN_CPHCAL_3,
	/* 4 */
	ADE9078_CIGAIN_CPHCAL_4,
	/* function disabled */
	ADE9078_CIGAIN_CPHCAL_DISABLE = 15
};

/***************************************************************************//**
 * @brief The `ade9078_cf4_pin_out_cfg_e` is an enumeration that defines the
 * possible configurations for the CF4 pin output on the ADE9078 device.
 * It allows the selection of different functionalities such as digital
 * to frequency conversion, event signaling, or data readiness
 * indication, which can be output through the CF4 pin. This
 * configuration is crucial for interfacing the ADE9078 with other
 * components in a system, enabling it to signal specific conditions or
 * data states.
 *
 * @param ADE9078_CF4_D_F_CONV Represents a digital to frequency converter
 * output on the CF4 pin.
 * @param ADE9078_CF4_D_F_CONV2 Represents a second digital to frequency
 * converter output on the CF4 pin.
 * @param ADE9078_CF4_EVENT Indicates an event output on the CF4 pin.
 * @param ADE9078_CF4_DREADY Indicates a data ready signal output on the CF4
 * pin.
 ******************************************************************************/
enum ade9078_cf4_pin_out_cfg_e {
	/* Digital to freq converter */
	ADE9078_CF4_D_F_CONV,
	/* Digital to freq converter */
	ADE9078_CF4_D_F_CONV2,
	/* Event */
	ADE9078_CF4_EVENT,
	/* Dready */
	ADE9078_CF4_DREADY,
};

/***************************************************************************//**
 * @brief The `ade9078_pwr_settle_e` is an enumeration that defines different
 * power settle times for the ADE9078 device. Each enumerator corresponds
 * to a specific time duration in milliseconds, which is used to
 * configure the power settling time in the device. This configuration is
 * crucial for ensuring that the device stabilizes its power measurements
 * over the specified time period, allowing for accurate readings.
 *
 * @param ADE9078_PWR_SETTLE_0 Represents a power settle time of 64 ms.
 * @param ADE9078_PWR_SETTLE_1 Represents a power settle time of 128 ms.
 * @param ADE9078_PWR_SETTLE_2 Represents a power settle time of 256 ms.
 * @param ADE9078_PWR_SETTLE_3 Represents a power settle time of 0 ms.
 ******************************************************************************/
enum ade9078_pwr_settle_e {
	/* 64 ms */
	ADE9078_PWR_SETTLE_0,
	/* 128 ms */
	ADE9078_PWR_SETTLE_1,
	/* 256 ms */
	ADE9078_PWR_SETTLE_2,
	/* 0 ms */
	ADE9078_PWR_SETTLE_3
};

/***************************************************************************//**
 * @brief The `ade9078_cf4_sel_e` enumeration defines various types of energy
 * outputs that can be configured on the CF4 pin of the ADE9078 device.
 * Each enumerator corresponds to a specific type of power measurement,
 * such as active, reactive, or apparent power, and is used to select the
 * desired energy output for monitoring and analysis purposes. The
 * enumeration values are used to configure the TERMSEL4 in the COMPMODE
 * register, determining which phases are included in the measurement.
 *
 * @param ADE9078_CF4_SEL_ACTIV_P Represents the total active power.
 * @param ADE9078_CF4_SEL_REACTIV_P Represents the total reactive power.
 * @param ADE9078_CF4_SEL_APPARENT_P Represents the total apparent power.
 * @param ADE9078_CF4_SEL_FUN_REACTIVE_P Represents the fundamental reactive
 * power, assigned a value of 4.
 * @param ADE9078_CF4_SEL_TOTAL_ACTIVE_P Represents the total active power,
 * assigned a value of 6.
 * @param ADE9078_CF4_SEL_TOTAL_ACTIVE_P_2 Represents an alternative total
 * active power.
 ******************************************************************************/
enum ade9078_cf4_sel_e {
	/* Total active power */
	ADE9078_CF4_SEL_ACTIV_P,
	/* Total reactive power */
	ADE9078_CF4_SEL_REACTIV_P,
	/* Total apparent power */
	ADE9078_CF4_SEL_APPARENT_P,
	/* Fundamental reactive power */
	ADE9078_CF4_SEL_FUN_REACTIVE_P = 4,
	/* Total active power */
	ADE9078_CF4_SEL_TOTAL_ACTIVE_P = 6,
	/* Total active power2 */
	ADE9078_CF4_SEL_TOTAL_ACTIVE_P_2,
};

/***************************************************************************//**
 * @brief The `ade9078_freq_sel_e` is an enumeration that defines the frequency
 * selection options for the ADE9078 device, specifically allowing the
 * user to choose between 50 Hz and 60 Hz. This selection is crucial for
 * configuring the device to operate correctly in regions with different
 * power line frequencies.
 *
 * @param ADE9078_SELFREQ_50 Represents a frequency selection of 50 Hz.
 * @param ADE9078_SELFREQ_60 Represents a frequency selection of 60 Hz.
 ******************************************************************************/
enum ade9078_freq_sel_e {
	/* 50 Hz */
	ADE9078_SELFREQ_50,
	/* 60 Hz */
	ADE9078_SELFREQ_60,
};

/***************************************************************************//**
 * @brief The `ade9078_vconsel_e` enumeration defines various hardware
 * configurations for the ADE9078 device, specifically focusing on
 * different wiring configurations for electrical systems. These
 * configurations include both 3-wire and 4-wire setups, with some being
 * non-Blondel compliant, which affects how voltage measurements are
 * calculated between phases. This enumeration is crucial for selecting
 * the appropriate configuration for accurate power and energy
 * measurements in different electrical systems.
 *
 * @param ADE9078_4WIRE_WYE Represents a 4-wire wye configuration.
 * @param ADE9078_3WIRE_DELTA Represents a 3-wire delta configuration where VB'
 * = VA − VC.
 * @param ADE9078_4WIRE_WYE_VA_VC Represents a 4-wire wye configuration, non-
 * Blondel compliant, where VB' = −VA − VC.
 * @param ADE9078_4WIRE_WYE_VA Represents a 4-wire delta configuration, non-
 * Blondel compliant, where VB' = −VA.
 * @param ADE9078_3WIRE_DELTA_2 Represents a 3-wire delta configuration where
 * VA' = VA − VB, VB' = VA − VC, and VC' = VC − VB.
 ******************************************************************************/
enum ade9078_vconsel_e {
	/* 4 wire wye */
	ADE9078_4WIRE_WYE,
	/* 3-wire delta. VB' = VA − VC */
	ADE9078_3WIRE_DELTA,
	/* 4-wire wye, nonBlondel compliant. VB' = −VA − VC */
	ADE9078_4WIRE_WYE_VA_VC,
	/* 4-wire delta, nonBlondel compliant. VB' = −VA */
	ADE9078_4WIRE_WYE_VA,
	/* 3-wire delta. VA' = VA − VB; VB' = VA − VC; VC' = VC − VB*/
	ADE9078_3WIRE_DELTA_2
};

/***************************************************************************//**
 * @brief The `ade9078_var_acc_mode_e` is an enumeration that defines the modes
 * of accumulation for total and fundamental reactive power in the
 * ADE9078 energy metering IC. It specifies how the reactive power is
 * accumulated in energy registers and CFx pulses, offering options for
 * signed, absolute, positive, and negative accumulation modes. This
 * allows for flexible configuration of the device to suit different
 * metering requirements.
 *
 * @param ADE9078_ACC_SIGNED Represents the signed accumulation mode for
 * reactive power.
 * @param ADE9078_ACC_ABSOLUTE Represents the absolute value accumulation mode
 * for reactive power.
 * @param ADE9078_ACC_POSITIVE Represents the positive accumulation mode for
 * reactive power.
 * @param ADE9078_ACC_NEGATIVE Represents the negative accumulation mode for
 * reactive power.
 ******************************************************************************/
enum ade9078_var_acc_mode_e {
	/* signed acc mode */
	ADE9078_ACC_SIGNED,
	/* absolute value acc mode */
	ADE9078_ACC_ABSOLUTE,
	/* positive acc mode */
	ADE9078_ACC_POSITIVE,
	/* negative acc mode */
	ADE9078_ACC_NEGATIVE
};

/***************************************************************************//**
 * @brief The `ade9078_line_period_sel_e` is an enumeration that defines the
 * different modes of line period measurement used in the ADE9078 device.
 * It specifies how the line period is measured for various calculations
 * such as VRMS half-cycle, 10-cycle RMS, 12-cycle RMS, and resampling.
 * Each enumerator corresponds to a specific accumulation mode, allowing
 * the device to measure line periods based on signed, absolute,
 * positive, or negative accumulation.
 *
 * @param ADE9078_APERIOD Represents the signed accumulation mode for line
 * period measurement.
 * @param ADE9078_BPERIOD Represents the absolute value accumulation mode for
 * line period measurement.
 * @param ADE9078_CPERIOD Represents the positive accumulation mode for line
 * period measurement.
 * @param ADE9078_COM_PERIOD Represents the negative accumulation mode for line
 * period measurement.
 ******************************************************************************/
enum ade9078_line_period_sel_e {
	/* signed acc mode */
	ADE9078_APERIOD,
	/* absolute value acc mode */
	ADE9078_BPERIOD,
	/* positive acc mode */
	ADE9078_CPERIOD,
	/* negative acc mode */
	ADE9078_COM_PERIOD
};

/***************************************************************************//**
 * @brief The `ade9078_zx_select_e` enumeration defines the possible zero-
 * crossing signal selections for the ADE9078 device. These selections
 * determine which phase's voltage zero-crossing signal is routed to the
 * CF3/ZX output pin, which can be used for line cycle energy
 * accumulation. The options include zero-crossing signals for individual
 * phases A, B, and C, as well as a combined signal from all three
 * phases.
 *
 * @param ADE9078_ZXVA_SEL Represents the zero-crossing signal for Phase A
 * voltage.
 * @param ADE9078_ZXVB_SEL Represents the zero-crossing signal for Phase B
 * voltage.
 * @param ADE9078_ZXVC_SEL Represents the zero-crossing signal for Phase C
 * voltage.
 * @param ADE9078_ZXCOMB_SEL Represents the zero-crossing signal for the
 * combined signal from VA, VB, and VC.
 ******************************************************************************/
enum ade9078_zx_select_e {
	/* Phase A voltage zero-crossing signal */
	ADE9078_ZXVA_SEL,
	/* Phase B voltage zero-crossing signal */
	ADE9078_ZXVB_SEL,
	/* Phase C voltage zero-crossing signal */
	ADE9078_ZXVC_SEL,
	/* Zero-crossing on combined signal from VA, VB, and VC */
	ADE9078_ZXCOMB_SEL
};

/***************************************************************************//**
 * @brief The `ade9078_wf_src_e` enumeration defines the different sources for
 * waveform data in the ADE9078 device, specifying the type of data
 * processing and sampling rate for waveform buffer sources. It includes
 * options for raw Sinc4 output, Sinc4 with IIR low-pass filtering, and
 * DSP-processed waveform samples, each with distinct sampling rates.
 *
 * @param ADE9078_SRC_SINC4 Represents the Sinc4 output at 32 kSPS.
 * @param ADE9078_SRC_SINC4_IIR Represents the Sinc4 + IIR LPF output at 8 kSPS.
 * @param ADE9078_SRC_DSP Represents the current and voltage channel waveform
 * samples processed by the DSP at 4 kSPS.
 ******************************************************************************/
enum ade9078_wf_src_e {
	/* Sinc4 output at 32 kSPS */
	ADE9078_SRC_SINC4,
	/* Sinc4 + IIR LPF output at 8 kSPS */
	ADE9078_SRC_SINC4_IIR = 2,
	/* Current and voltage channel waveform samples,
	processed by the DSP (xI_PCF, xV_PCF) at 4 kSPS */
	ADE9078_SRC_DSP
};

/***************************************************************************//**
 * @brief The `ade9078_wf_mode_e` is an enumeration that defines the different
 * modes for handling waveform data in the ADE9078 device. It specifies
 * how the waveform buffer should be filled and managed, including
 * stopping when full, filling continuously with specific trigger
 * conditions, centering captures around events, or saving event
 * addresses. This enumeration is crucial for configuring the waveform
 * data handling behavior in applications using the ADE9078 energy
 * metering IC.
 *
 * @param ADE9078_MODE_STOP_FULL Stops when the waveform buffer is full.
 * @param ADE9078_MODE_TRIG_EN_EVENTS Continuously fills and stops only on
 * enabled trigger events.
 * @param ADE9078_MODE_CENTER_CAPTURE Continuously fills and centers capture
 * around enabled trigger events.
 * @param ADE9078_MODE_SAVE_EVENT_ADDR Continuously fills and saves the event
 * address of enabled trigger events.
 ******************************************************************************/
enum ade9078_wf_mode_e {
	/* Stop when waveform buffer is full */
	ADE9078_MODE_STOP_FULL,
	/* Continuous fill—stop only on enabled trigger
	events */
	ADE9078_MODE_TRIG_EN_EVENTS,
	/* Continuous filling—center capture around
	enabled trigger events. */
	ADE9078_MODE_CENTER_CAPTURE,
	/* Continuous fill—save event address of enabled
	trigger events */
	ADE9078_MODE_SAVE_EVENT_ADDR
};

/***************************************************************************//**
 * @brief The `ade9078_burst_ch_e` enumeration defines the different channel
 * configurations available for burst reading from the ADE9078 waveform
 * buffer through SPI. It includes options for reading all channels,
 * specific pairs of current and voltage channels, individual channels,
 * and a disabled state for single address reading. This enumeration is
 * used to configure the data output from the waveform buffer,
 * facilitating efficient data acquisition in various application
 * scenarios.
 *
 * @param ADE9078_BURST_ALL_CH Represents all channels for burst reading.
 * @param ADE9078_BURST_IA_VA Represents IA and VA channels for burst reading.
 * @param ADE9078_BURST_IB_VB Represents IB and VB channels for burst reading.
 * @param ADE9078_BURST_IC_VC Represents IC and VC channels for burst reading.
 * @param ADE9078_BURST_IA Represents the IA channel for burst reading.
 * @param ADE9078_BURST_VA Represents the VA channel for burst reading.
 * @param ADE9078_BURST_IB Represents the IB channel for burst reading.
 * @param ADE9078_BURST_VB Represents the VB channel for burst reading.
 * @param ADE9078_BURST_IC Represents the IC channel for burst reading.
 * @param ADE9078_BURST_VC Represents the VC channel for burst reading.
 * @param ADE9078_BURST_IN Represents the IN channel for burst reading if
 * WF_IN_EN is enabled.
 * @param ADE9078_BURST_DISABLED Disables burst reading, allowing single address
 * read.
 ******************************************************************************/
enum ade9078_burst_ch_e {
	/* All channels */
	ADE9078_BURST_ALL_CH,
	/* IA and VA */
	ADE9078_BURST_IA_VA,
	/* IB and VB */
	ADE9078_BURST_IB_VB,
	/* IC and VC */
	ADE9078_BURST_IC_VC,
	/* IA */
	ADE9078_BURST_IA = 8,
	/* VA */
	ADE9078_BURST_VA,
	/* IB */
	ADE9078_BURST_IB,
	/* VB */
	ADE9078_BURST_VB,
	/* IC */
	ADE9078_BURST_IC,
	/* VC */
	ADE9078_BURST_VC,
	/* IN if WF_IN_EN = 1*/
	ADE9078_BURST_IN,
	/* Burst Disable read single addr */
	ADE9078_BURST_DISABLED
};

/***************************************************************************//**
 * @brief The `ade9078_hpf_freq_e` is an enumeration that defines various high-
 * pass filter corner frequencies (f3dB) for the ADE9078 device. These
 * frequencies are used to configure the high-pass filter when the HPFDIS
 * bit in the CONFIG0 register is set to zero, allowing the user to
 * select the desired cutoff frequency for filtering purposes. Each
 * enumerator corresponds to a specific frequency value, providing
 * flexibility in signal processing applications.
 *
 * @param ADE9078_HPF_36_695 Represents a high-pass filter corner frequency of
 * 39.695 Hz.
 * @param ADE9078_HPF_19_6375 Represents a high-pass filter corner frequency of
 * 19.6375 Hz.
 * @param ADE9078_HPF_9_895 Represents a high-pass filter corner frequency of
 * 9.895 Hz.
 * @param ADE9078_HPF_4_9675 Represents a high-pass filter corner frequency of
 * 4.9675 Hz.
 * @param ADE9078_HPF_2_49 Represents a high-pass filter corner frequency of
 * 2.49 Hz.
 * @param ADE9078_HPF_1_2475 Represents a high-pass filter corner frequency of
 * 1.2475 Hz.
 * @param ADE9078_HPF_0_625 Represents a high-pass filter corner frequency of
 * 0.625 Hz.
 * @param ADE9078_HPF_0_3125 Represents a high-pass filter corner frequency of
 * 0.3125 Hz.
 ******************************************************************************/
enum ade9078_hpf_freq_e {
	/* 39.695 Hz. */
	ADE9078_HPF_36_695,
	/* 19.6375 Hz. */
	ADE9078_HPF_19_6375,
	/* 9.895 Hz. */
	ADE9078_HPF_9_895,
	/* 4.9675 Hz. */
	ADE9078_HPF_4_9675,
	/* 2.49 Hz. */
	ADE9078_HPF_2_49,
	/* 1.2475 Hz. */
	ADE9078_HPF_1_2475,
	/* 0.625 Hz. */
	ADE9078_HPF_0_625,
	/* 0.3125 Hz. */
	ADE9078_HPF_0_3125
};

/***************************************************************************//**
 * @brief The `ade9078_no_load_tmr_e` is an enumeration that defines various
 * configurations for evaluating the no load condition in the ADE9078
 * device. Each enumerator specifies the number of 4 kSPS samples over
 * which the no load condition is assessed, ranging from 64 to 4096
 * samples, with an option to disable the no load threshold evaluation
 * entirely. This configuration is crucial for determining the presence
 * of a load based on the number of samples analyzed.
 *
 * @param ADE9078_NOLOAD_SAMPLES_64 Represents a configuration for evaluating no
 * load condition over 64 samples.
 * @param ADE9078_NOLOAD_SAMPLES_128 Represents a configuration for evaluating
 * no load condition over 128 samples.
 * @param ADE9078_NOLOAD_SAMPLES_256 Represents a configuration for evaluating
 * no load condition over 256 samples.
 * @param ADE9078_NOLOAD_SAMPLES_512 Represents a configuration for evaluating
 * no load condition over 512 samples.
 * @param ADE9078_NOLOAD_SAMPLES_1024 Represents a configuration for evaluating
 * no load condition over 1024 samples.
 * @param ADE9078_NOLOAD_SAMPLES_2048 Represents a configuration for evaluating
 * no load condition over 2048 samples.
 * @param ADE9078_NOLOAD_SAMPLES_4096 Represents a configuration for evaluating
 * no load condition over 4096 samples.
 * @param ADE9078_NOLOAD_SAMPLES_DISABLE Disables the no load threshold
 * evaluation.
 ******************************************************************************/
enum ade9078_no_load_tmr_e {
	/* 64 samples */
	ADE9078_NOLOAD_SAMPLES_64,
	/* 128 samples */
	ADE9078_NOLOAD_SAMPLES_128,
	/* 256 samples */
	ADE9078_NOLOAD_SAMPLES_256,
	/* 512 samples */
	ADE9078_NOLOAD_SAMPLES_512,
	/* 1024 samples */
	ADE9078_NOLOAD_SAMPLES_1024,
	/* 2048 samples */
	ADE9078_NOLOAD_SAMPLES_2048,
	/* 4096 samples */
	ADE9078_NOLOAD_SAMPLES_4096,
	/* disable no load threshold */
	ADE9078_NOLOAD_SAMPLES_DISABLE
};

/***************************************************************************//**
 * @brief The `ade9078_pkdet_lvl_e` is an enumeration that defines various peak
 * detection levels for the ADE9078 device, expressed as ratios of the
 * input signal level to the full scale. Each enumerator represents a
 * specific peak detection threshold, ranging from 100:1 to 1600:1, which
 * can be used to configure the PSM2 low power comparator for peak
 * current detection.
 *
 * @param ADE9078_PKDET_LVL_100 Represents a peak detection level of 100:1.
 * @param ADE9078_PKDET_LVL_200 Represents a peak detection level of 200:1.
 * @param ADE9078_PKDET_LVL_300 Represents a peak detection level of 300:1.
 * @param ADE9078_PKDET_LVL_400 Represents a peak detection level of 400:1.
 * @param ADE9078_PKDET_LVL_500 Represents a peak detection level of 500:1.
 * @param ADE9078_PKDET_LVL_600 Represents a peak detection level of 600:1.
 * @param ADE9078_PKDET_LVL_700 Represents a peak detection level of 700:1.
 * @param ADE9078_PKDET_LVL_800 Represents a peak detection level of 800:1.
 * @param ADE9078_PKDET_LVL_900 Represents a peak detection level of 900:1.
 * @param ADE9078_PKDET_LVL_1000 Represents a peak detection level of 1000:1.
 * @param ADE9078_PKDET_LVL_1100 Represents a peak detection level of 1100:1.
 * @param ADE9078_PKDET_LVL_1200 Represents a peak detection level of 1200:1.
 * @param ADE9078_PKDET_LVL_1300 Represents a peak detection level of 1300:1.
 * @param ADE9078_PKDET_LVL_1400 Represents a peak detection level of 1400:1.
 * @param ADE9078_PKDET_LVL_1500 Represents a peak detection level of 1500:1.
 * @param ADE9078_PKDET_LVL_1600 Represents a peak detection level of 1600:1.
 ******************************************************************************/
enum ade9078_pkdet_lvl_e {
	/* 100:1 */
	ADE9078_PKDET_LVL_100,
	/* 200:1 */
	ADE9078_PKDET_LVL_200,
	/* 300:1 */
	ADE9078_PKDET_LVL_300,
	/* 400:1 */
	ADE9078_PKDET_LVL_400,
	/* 500:1 */
	ADE9078_PKDET_LVL_500,
	/* 600:1 */
	ADE9078_PKDET_LVL_600,
	/* 700:1 */
	ADE9078_PKDET_LVL_700,
	/* 800:1 */
	ADE9078_PKDET_LVL_800,
	/* 900:1 */
	ADE9078_PKDET_LVL_900,
	/* 1000:1 */
	ADE9078_PKDET_LVL_1000,
	/* 1100:1 */
	ADE9078_PKDET_LVL_1100,
	/* 1200:1 */
	ADE9078_PKDET_LVL_1200,
	/* 1300:1 */
	ADE9078_PKDET_LVL_1300,
	/* 1400:1 */
	ADE9078_PKDET_LVL_1400,
	/* 1500:1 */
	ADE9078_PKDET_LVL_1500,
	/* 1600:1 */
	ADE9078_PKDET_LVL_1600
};

/***************************************************************************//**
 * @brief The `ade9078_vc_gain_e` enumeration defines the programmable gain
 * amplifier (PGA) gain settings for the Voltage Channel C ADC in the
 * ADE9078 device. It provides four discrete gain levels, allowing the
 * user to select the appropriate gain for their application, thereby
 * optimizing the signal-to-noise ratio and dynamic range of the voltage
 * measurements.
 *
 * @param ADE9078_VC_GAIN_1 Represents a gain setting of 1 for Voltage Channel
 * C.
 * @param ADE9078_VC_GAIN_2 Represents a gain setting of 2 for Voltage Channel
 * C.
 * @param ADE9078_VC_GAIN_3 Represents a gain setting of 3 for Voltage Channel
 * C.
 * @param ADE9078_VC_GAIN_4 Represents a gain setting of 4 for Voltage Channel
 * C.
 ******************************************************************************/
enum ade9078_vc_gain_e {
	/* Gain = 1 */
	ADE9078_VC_GAIN_1,
	/* Gain = 2 */
	ADE9078_VC_GAIN_2,
	/* Gain = 3 */
	ADE9078_VC_GAIN_3,
	/* Gain = 4 */
	ADE9078_VC_GAIN_4
};

/***************************************************************************//**
 * @brief The `ade9078_phase` enumeration defines the available phases (A, B,
 * and C) for the ADE9078 device, which is used in power measurement and
 * monitoring applications. Each enumerator corresponds to a specific
 * phase, allowing the software to reference and manipulate data related
 * to that phase in the device.
 *
 * @param ADE9078_PHASE_A Represents phase A in the ADE9078 device.
 * @param ADE9078_PHASE_B Represents phase B in the ADE9078 device.
 * @param ADE9078_PHASE_C Represents phase C in the ADE9078 device.
 ******************************************************************************/
enum ade9078_phase {
	ADE9078_PHASE_A,
	ADE9078_PHASE_B,
	ADE9078_PHASE_C
};

/***************************************************************************//**
 * @brief The `ade9078_egy_model` is an enumeration that defines the available
 * user energy use models for the ADE9078 device. It provides three
 * distinct models: one that includes a reset functionality, another that
 * calculates energy based on half line cycles, and a third that uses a
 * specified number of samples for energy calculation. This enumeration
 * is used to configure the energy measurement behavior of the ADE9078
 * device.
 *
 * @param ADE9078_EGY_WITH_RESET Represents an energy model with reset
 * functionality.
 * @param ADE9078_EGY_HALF_LINE_CYCLES Represents an energy model based on half
 * line cycles.
 * @param ADE9078_EGY_NR_SAMPLES Represents an energy model based on a number of
 * samples.
 ******************************************************************************/
enum ade9078_egy_model {
	ADE9078_EGY_WITH_RESET,
	ADE9078_EGY_HALF_LINE_CYCLES,
	ADE9078_EGY_NR_SAMPLES
};

/***************************************************************************//**
 * @brief The `ade9078_power_mode_e` is an enumeration that defines the
 * different power modes available for the ADE9078 device. These modes
 * include NORMAL_MODE for standard operation, TAMPER_MODE for tamper
 * detection, CURRENT_PEAK_DETECT_MODE for detecting current peaks, and
 * IDLE_MODE for low-power idle state. The enumeration is used to
 * configure the power mode of the ADE9078, which affects the device's
 * operation and power consumption characteristics.
 *
 * @param NORMAL_MODE Represents the normal power mode (PSM0) of the ADE9078.
 * @param TAMPER_MODE Represents the tamper measurement power mode (PSM1) of the
 * ADE9078.
 * @param CURRENT_PEAK_DETECT_MODE Represents the current peak detect power mode
 * (PSM2) of the ADE9078.
 * @param IDLE_MODE Represents the idle power mode (PSM3) of the ADE9078.
 ******************************************************************************/
enum ade9078_power_mode_e {
	/* SPI is not available in PSM2 & PSM3*/
	/* PSM0 normal mode */
	NORMAL_MODE,
	/* PSM1 Tamper measurement mode */
	TAMPER_MODE,
	/* PSM2 Current peak detect mode */
	CURRENT_PEAK_DETECT_MODE,
	/* PSM3 IDLE */
	IDLE_MODE
};

/***************************************************************************//**
 * @brief The `ade9078_init_param` structure is used to define the
 * initialization parameters for the ADE9078 device. It includes pointers
 * to SPI and GPIO descriptors necessary for device communication and
 * control, as well as a variable to select the power mode of the device.
 * This structure is essential for setting up the device with the correct
 * communication interfaces and operational mode.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param psm0_desc Pointer to the GPIO descriptor for PSM0.
 * @param psm1_desc Pointer to the GPIO descriptor for PSM1.
 * @param reset_desc Pointer to the GPIO descriptor for the reset functionality.
 * @param power_mode 8-bit unsigned integer for selecting the power mode.
 ******************************************************************************/
struct ade9078_init_param {
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
 * @brief The `ade9078_dev` structure is a comprehensive representation of the
 * ADE9078 device, encapsulating all necessary descriptors and variables
 * for its operation. It includes pointers to SPI and GPIO descriptors
 * for communication and control, as well as variables to store key
 * electrical measurements such as WATT, IRMS, and VRMS values.
 * Additionally, it holds a power mode variable to manage the device's
 * operational state, making it integral for interfacing with and
 * controlling the ADE9078 energy metering IC.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param psm0_desc Pointer to the GPIO descriptor for the PSM0 mode.
 * @param psm1_desc Pointer to the GPIO descriptor for the PSM1 mode.
 * @param reset_desc Pointer to the GPIO descriptor for the reset functionality.
 * @param watt_val Stores the WATT value measured by the device.
 * @param irms_val Stores the IRMS (current RMS) value measured by the device.
 * @param vrms_val Stores the VRMS (voltage RMS) value measured by the device.
 * @param power_mode Indicates the current power mode of the device.
 ******************************************************************************/
struct ade9078_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/* psm0 descriptor */
	struct no_os_gpio_desc      	*psm0_desc;
	/* psm1 descriptor */
	struct no_os_gpio_desc      	*psm1_desc;
	/* psm1 descriptor */
	struct no_os_gpio_desc      	*reset_desc;
	/* Variable storing the WATT value */
	uint32_t			watt_val;
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
 * @brief Use this function to read the value of a specified register from the
 * ADE9078 device. It requires a valid device structure and a register
 * address to read from. The function will store the read value in the
 * provided output parameter. Ensure that the device has been properly
 * initialized before calling this function. The function handles both
 * 16-bit and 32-bit registers, depending on the address range. It
 * returns an error code if the device structure or output parameter is
 * null, or if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ade9078_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer.
 * @param reg_data A pointer to a 32-bit unsigned integer where the read
 * register value will be stored. Must not be null.
 * @return Returns 0 on success, a negative error code on failure (e.g., -ENODEV
 * if the device is null, -EINVAL if reg_data is null, or an error code
 * from SPI communication).
 ******************************************************************************/
int ade9078_read(struct ade9078_dev *dev, uint16_t reg_addr,
		 uint32_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit or 32-bit value to a specified
 * register of the ADE9078 device. It is essential to ensure that the
 * device structure is properly initialized and that the device is ready
 * for communication before calling this function. The function handles
 * both 16-bit and 32-bit registers, automatically determining the
 * register size based on the address range. It returns an error code if
 * the device is not available or if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ade9078_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param reg_addr The address of the register to write to. It is a 16-bit
 * unsigned integer, and the function determines the register
 * size based on this address.
 * @param reg_data The data to write to the register. It is a 32-bit unsigned
 * integer, but only the relevant bits are used depending on the
 * register size.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -ENODEV if the device is not available.
 ******************************************************************************/
int ade9078_write(struct ade9078_dev *dev, uint16_t reg_addr,
		  uint32_t reg_data);

/* Set power mode */
/***************************************************************************//**
 * @brief This function configures the power mode of the ADE9078 device based on
 * the current power mode setting in the device structure. It must be
 * called with a valid device structure that has been properly
 * initialized. The function checks for null pointers and invalid power
 * mode settings, returning appropriate error codes if these conditions
 * are not met. It manipulates GPIO pins to set the device into the
 * desired power mode, which can be one of several predefined modes such
 * as IDLE_MODE, CURRENT_PEAK_DETECT_MODE, TAMPER_MODE, or NORMAL_MODE.
 *
 * @param dev A pointer to an ade9078_dev structure representing the device.
 * This must not be null and should be properly initialized with
 * valid GPIO descriptors for psm0_desc and psm1_desc. The power_mode
 * field must be set to a valid mode; otherwise, the function returns
 * an error.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -ENODEV if the device pointer is null, or -EINVAL if the power mode
 * is invalid or GPIO descriptors are missing.
 ******************************************************************************/
int ade9078_set_power_mode(struct ade9078_dev *dev);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief Use this function to modify specific bits in a register of the ADE9078
 * device. It is useful when only certain bits need to be changed without
 * affecting the rest of the register. The function reads the current
 * value of the register, applies the mask to clear the bits to be
 * updated, and then sets the new bits according to the provided data.
 * This function must be called with a valid device structure pointer,
 * and it will return an error if the device pointer is null or if the
 * read/write operations fail.
 *
 * @param dev A pointer to an ade9078_dev structure representing the device.
 * Must not be null. The function will return -ENODEV if this
 * parameter is null.
 * @param reg_addr The address of the register to be updated. It is a 16-bit
 * unsigned integer representing a valid register address in the
 * ADE9078 device.
 * @param mask A 32-bit unsigned integer where bits set to 1 indicate which bits
 * in the register should be updated. The mask is used to clear the
 * bits in the register before setting new values.
 * @param reg_data A 32-bit unsigned integer containing the new data to be
 * written to the register. Only the bits specified by the mask
 * will be updated with this data.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the read/write operations fail.
 ******************************************************************************/
int ade9078_update_bits(struct ade9078_dev *dev, uint16_t reg_addr,
			uint32_t mask, uint32_t reg_data);

/* Read Energy/Power for specific phase */
/***************************************************************************//**
 * @brief This function retrieves the instantaneous RMS current, RMS voltage,
 * and active power values for a specified phase of the ADE9078 device.
 * It should be called when accurate phase-specific energy measurements
 * are required. The function updates the device structure with the
 * measured values in milliamps, millivolts, and milliwatts,
 * respectively. It is essential to ensure that the device is properly
 * initialized before calling this function. The function handles invalid
 * phase inputs by returning an error code, and it also checks for a
 * valid device pointer, returning an error if the device is not
 * available.
 *
 * @param dev A pointer to an ade9078_dev structure representing the device.
 * Must not be null. The function returns -ENODEV if this parameter
 * is null.
 * @param phase An enum ade9078_phase value specifying the phase (A, B, or C)
 * for which data is to be read. If an invalid phase is provided,
 * the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. Updates
 * the irms_val, vrms_val, and watt_val fields of the ade9078_dev
 * structure with the measured values for the specified phase.
 ******************************************************************************/
int ade9078_read_data_ph(struct ade9078_dev *dev, enum ade9078_phase phase);

/* Set User Energy use model */
/***************************************************************************//**
 * @brief This function sets the energy accumulation model for the ADE9078
 * device, allowing the user to specify how energy data is accumulated
 * and reset. It must be called with a valid device structure and a
 * supported energy model. The function handles different energy models,
 * each requiring specific configurations, and writes these settings to
 * the device. It is essential to ensure the device is properly
 * initialized before calling this function. Invalid model or value
 * combinations will result in an error.
 *
 * @param dev A pointer to an initialized ade9078_dev structure representing the
 * device. Must not be null.
 * @param model An enum value of type ade9078_egy_model specifying the energy
 * model to set. Must be a valid model defined in the enum.
 * @param value A uint16_t value used for specific model configurations. For
 * ADE9078_EGY_WITH_RESET, this must be 1; other models may use
 * different values.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -ENODEV if the device is null or -EINVAL for invalid parameters.
 ******************************************************************************/
int ade9078_set_egy_model(struct ade9078_dev *dev, enum ade9078_egy_model model,
			  uint16_t value);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the ADE9078 device by setting up the
 * necessary SPI communication and configuring the power mode and reset
 * GPIOs. It must be called before any other operations on the device.
 * The function performs a hardware reset and verifies the device ID to
 * ensure proper initialization. If initialization fails at any step, it
 * cleans up allocated resources and returns an error code.
 *
 * @param device A pointer to a pointer of ade9078_dev structure where the
 * initialized device instance will be stored. Must not be null.
 * The caller takes ownership of the allocated device structure
 * upon successful initialization.
 * @param init_param A structure containing initialization parameters such as
 * SPI and GPIO descriptors and the desired power mode. All
 * fields must be properly initialized, and the GPIO
 * descriptors must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no resources are leaked.
 ******************************************************************************/
int ade9078_init(struct ade9078_dev **device,
		 struct ade9078_init_param init_param);

/* Setup the device */
/***************************************************************************//**
 * @brief This function is used to configure the ADE9078 device with a set of
 * predefined settings necessary for its operation. It must be called
 * after the device has been initialized and before any data operations
 * are performed. The function writes a series of configuration values to
 * the device's registers, enabling features such as channel gains, DSP
 * operation, and energy accumulation settings. If the device pointer is
 * null, the function returns an error. It also returns an error if any
 * of the register writes fail, ensuring that the device is only
 * considered set up if all configurations are successfully applied.
 *
 * @param dev A pointer to an initialized ade9078_dev structure. Must not be
 * null. The function returns an error if this parameter is null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if any register write operation fails.
 ******************************************************************************/
int ade9078_setup(struct ade9078_dev *dev);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief Use this function to properly remove an ADE9078 device instance and
 * free associated resources. It should be called when the device is no
 * longer needed, ensuring that any allocated resources are released.
 * This function must be called after the device has been initialized and
 * used, to prevent resource leaks. It handles the removal of the SPI
 * descriptor and frees the device structure. Ensure that the `dev`
 * parameter is valid and initialized before calling this function.
 *
 * @param dev A pointer to an `ade9078_dev` structure representing the device to
 * be removed. Must not be null and should be a valid, initialized
 * device instance. The function will handle invalid or null pointers
 * by not performing any operations.
 * @return Returns 0 on successful removal and resource release, or a negative
 * error code if the SPI removal fails.
 ******************************************************************************/
int ade9078_remove(struct ade9078_dev *dev);

/* Get interrupt indicator from STATUS0 register. */
/***************************************************************************//**
 * @brief Use this function to check the status of specific interrupts in the
 * ADE9078 device by reading the STATUS0 register. This function is
 * useful for determining if certain events have occurred, as indicated
 * by the interrupt mask provided. It must be called with a valid device
 * structure and a non-null status pointer. The function will return an
 * error if the device is not initialized or if the status pointer is
 * null.
 *
 * @param dev A pointer to an initialized ade9078_dev structure representing the
 * device. Must not be null.
 * @param msk A 32-bit mask indicating which interrupt status bits to check.
 * Each bit corresponds to a specific interrupt in the STATUS0
 * register.
 * @param status A pointer to a uint8_t where the function will store the result
 * of the interrupt status check. Must not be null.
 * @return Returns 0 on success, a negative error code if the device is not
 * initialized or if the status pointer is null, or if there is an error
 * reading the register.
 ******************************************************************************/
int ade9078_get_int_status0(struct ade9078_dev *dev, uint32_t msk,
			    uint8_t *status);

#endif // __ADE9078_H__

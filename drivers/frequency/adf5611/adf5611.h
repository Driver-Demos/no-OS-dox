/***************************************************************************//**
 *   @file   adf5611.h
 *   @brief  Implementation of adf5611 Driver.
 *   @author Jude Osemene (jude.osemene@analog.com)
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
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"

/* ADF5611 REG0000 Map */
#define ADF5611_SOFT_REST_R_MSK             NO_OS_BIT(7)
#define ADF5611_LSB_FIRST_R_MSK             NO_OS_BIT(6)
#define ADF5611_ADDRESS_ASC_R_MSK           NO_OS_BIT(5)
#define ADF5611_SDO_ACTIVE_R_MSK            NO_OS_BIT(4)
#define ADF5611_SDO_ACTIVE_MSK              NO_OS_BIT(3)
#define ADF5611_ADDRESS_ASC_MSK             NO_OS_BIT(2)
#define ADF5611_LSB_FIRST_MSK               NO_OS_BIT(1)
#define ADF5611_SOFT_RESET_MSK              NO_OS_BIT(0)
#define ADF5611_RESET_CMD		    0x81

/* ADF5611 REG0000 NO_OS_BIT DEFINITION */
#define ADF5611_SDO_ACTIVE_SPI_3WIRE        0x0
#define ADF5611_SDO_ACTIVE_SPI_4WIRE        0x1

#define ADF5611_ADDRESS_ASC_AUTO_DECR       0x0
#define ADF5611_ADDRESS_ASC_AUTO_INCR       0x1

#define ADF5611_LSB_FIRST_MSB		    0x0
#define ADF5611_LSB_FIRST_LSB		    0x1

#define ADF5611_SOFT_RESET_NORMAL_OP	    0x0
#define ADF5611_SOFT_RESET_EN		    0x1

/* ADF5611 REG0001 MAP */
#define ADF5611_SINGLE_INSTR_MSK            NO_OS_BIT(7)
#define ADF5611_MASTER_READBACK_CTRL_MSK    NO_OS_BIT(5)

/* ADF5611 REG0001 NO_OS_BIT DEFINITION */
#define ADF5611_SPI_STREAM_EN		    0x0
#define ADF5611_SPI_STREAM_DIS		    0x1

#define ADF5611_RB_SLAVE_REG		    0x0
#define ADF5611_RB_MASTER_REG		    0x1

/* ADF5611 REG0003 NO_OS_BIT DEFINITION */
#define ADF5611_CHIP_TYPE		    0x06

/* ADF5611 REG0004 NO_OS_BIT DEFINITION */
#define ADF5611_PRODUCT_ID_LSB		    0x0005

/* ADF5611 REG0005 NO_OS_BIT DEFINITION */
#define ADF5611_PRODUCT_ID_MSB		    0x0005

/* ADF5611 REG000A Map */
#define ADF5611_SCRATCHPAD_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG000C NO_OS_BIT DEFINITION */
#define ADF5611_VENDOR_ID_LSB		    0x56

/* ADF5611 REG000D NO_OS_BIT DEFINITION */
#define ADF5611_VENDOR_ID_MSB		    0x04

/* ADF5611 REG0010 MAP*/
#define ADF5611_N_INT_LSB_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0011 MAP*/
#define ADF5611_N_INT_MID_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0012 MAP */
#define ADF5611_FRAC1WORD_LSB_MSK	    NO_OS_GENMASK(7, 4)
#define ADF5611_N_INT_MSB_MSK		    NO_OS_GENMASK(3, 0)

/* ADF5611 REG0013 MAP */
#define ADF5611_FRAC1WORD_MID_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0014 MAP */
#define ADF5611_FRAC1WORD_MID_MSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0015 MAP */
#define ADF5611_M_VCO_BIAS_MSK              NO_OS_GENMASK(7, 5)
#define ADF5611_FRAC1WORD_MSB_MSK           NO_OS_GENMASK(4, 0)

/* ADF5611 REG0016 MAP */
#define ADF5611_M_VCO_BAND_MSK              NO_OS_GENMASK(7, 1)
#define ADF5611_M_VCO_CORE_MSK              NO_OS_BIT(0)

/* ADF5611 REG0016 NO_OS_BIT DEFINITION */
#define ADF5611_VCO_0_HIGHEST_FREQUENCY     0x0
#define ADF5611_VCO_1_LOWEST_FREQUENCY      0x1

/* ADF5611 REG0017 MAP */
#define ADF5611_FRAC2WORD_LSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0018 MAP */
#define ADF5611_FRAC2WORD_MID_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0019 MAP */
#define ADF5611_FRAC2WORD_MSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG001A MAP */
#define ADF5611_MOD2WORD_LSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG001B MAP */
#define ADF5611_MOD2WORD_MID_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG001C MAP */
#define ADF5611_MOD2WORD_MSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG001D MAP */
#define ADF5611_BLEED_I_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG001E MAP */
#define ADF5611_EN_AUTOCAL_MSK              NO_OS_BIT(7)
#define ADF5611_EN_BLEED_MSK                NO_OS_BIT(6)
#define ADF5611_EN_DCLK_MODE_MSK            NO_OS_BIT(5)
#define ADF5611_EN_DNCLK_MSK                NO_OS_BIT(4)
#define ADF5611_DNCLK_DIV1_MSK              NO_OS_GENMASK(3, 2)
#define ADF5611_PFD_POL_MSK                 NO_OS_BIT(1)
#define ADF5611_BLEED_POL_MSK               NO_OS_BIT(0)

/* ADF5611 REG001E NO_OS_BIT DEFINITION */
#define ADF5611_VCO_CALIBRATION_DIS         0x0
#define ADF5611_VCO_CALIBRATION_EN          0x1

#define ADF5611_BLEED_CURRENT_DIS           0x0
#define ADF5611_BLEED_CURRENT_EN            0x1

#define ADF5611_FREQUENCY_REDUCTION_DIS     0x0
#define ADF5611_FREQUENCY_REDUCTION_EN      0x1

#define ADF5611_DIV_NCLK_OFF                0x0
#define ADF5611_DIV_NCLK_ON                 0x1

#define ADF5611_CURRENT_SINK                0x0
#define ADF5611_CURRENT_SOURCE              0x1

/* ADF5611 REG001F MAP */
#define ADF5611_R_DIV_LSB_MSK               NO_OS_GENMASK(7, 0)

/* ADF5611 REG0020 MAP */
#define ADF5611_R_DIV_MSB_MSK          	    NO_OS_GENMASK(5, 0)


/* ADF5611 REG0021 MAP */
#define ADF5611_INTMODE_EN_MSK              NO_OS_BIT(6)
#define ADF5611_RST_RDIV_MSK                NO_OS_BIT(5)
#define ADF5611_EN_RCNTR_MSK                NO_OS_BIT(4)
#define ADF5611_CP_I_MSK                    NO_OS_GENMASK(3, 0)

/* ADF5611 REG0021 NO_OS_BIT DEFINITION */
#define ADF5611_FRAC_MODE                   0x0
#define ADF5611_INTEGER_MODE                0x1

/* ADF5611 REG0022 MAP */
#define ADF5611_RFOUT_DIV_MSK      	    NO_OS_GENMASK(7, 5)
#define ADF5611_RFOUT_PWR_MSK               NO_OS_GENMASK(4, 3)
#define ADF5611_DIV_PWR_MSK                 NO_OS_GENMASK(1, 0)

/* ADF5611 REG0023 MAP*/
#define ADF5611_PHASE_WORD_LSB_MSK          NO_OS_GENMASK(7, 0)

/* ADF5611 REG0024 MAP*/
#define ADF5611_PHASE_WORD_MID_MSK          NO_OS_GENMASK(7, 0)

/* ADF5611 REG0025 MAP*/
#define ADF5611_PHASE_WORD_MSB_MSK          NO_OS_GENMASK(7, 0)

/* ADF5611 REG0026 MAP */
#define ADF5611_LSB_P1_MSK		    NO_OS_BIT(6)
#define ADF5611_VAR_MOD_EN_MSK              NO_OS_BIT(5)
#define ADF5611_DITHER1_SCALE_MSK           NO_OS_GENMASK(4, 2)
#define ADF5611_EN_DITHER2_MSK              NO_OS_BIT(1)
#define ADF5611_EN_DITHER1_MSK              NO_OS_BIT(0)

/* ADF5611 REG0027 MAP */
#define ADF5611_PD_ALL_MSK		    NO_OS_BIT(7)
#define ADF5611_PD_RDIV_MSK		    NO_OS_BIT(6)
#define ADF5611_PD_NDIV_MSK		    NO_OS_BIT(5)
#define ADF5611_PD_VCO_MSK		    NO_OS_BIT(4)
#define ADF5611_PD_LD_MSK		    NO_OS_BIT(3)
#define ADF5611_PD_PFDCP_MSK		    NO_OS_BIT(2)
#define ADF5611_PD_ADC_MSK		    NO_OS_BIT(1)
#define ADF5611_PD_CALGEN_MSK		    NO_OS_BIT(0)

/* ADF5611 REG0028 MAP */
#define ADF5611_PD_PFDNCLK_MSK		    NO_OS_BIT(1)
#define ADF5611_PD_ODIV_MSK		    NO_OS_BIT(0)

/* ADF5611 REG0029 MAP */
#define ADF5611_LDWIN_PW_MSK		    NO_OS_GENMASK(7, 5)
#define ADF5611_LD_COUNT_MSK		    NO_OS_GENMASK(4, 0)

/* ADF5611 REG002A MAP*/
#define ADF5611_EN_CP_IBX_MSK		    NO_OS_GENMASK(7, 6)
#define ADF5611_EN_LOL_MSK		    NO_OS_BIT(5)
#define ADF5611_EN_LDWIN_MSK		    NO_OS_BIT(4)
#define ADF5611_SPARE_2A_MSK                NO_OS_BIT(3)
#define ADF5611_RST_LD_MSK		    NO_OS_BIT(2)
#define ADF5611_ABPW_WD_MSK		    NO_OS_BIT(1)
#define ADF5611_RST_CNTR_MSK                NO_OS_BIT(0)

/* ADF5611 REG002B MAP */
#define ADF5611_MUXOUT_MSK		    NO_OS_GENMASK(7, 4)
#define ADF5611_EN_MUXOUT_MSK               NO_OS_BIT(3)
#define ADF5611_EN_CPTEST_MSK		    NO_OS_BIT(2)
#define ADF5611_CP_DOWN_MSK		    NO_OS_BIT(1)
#define ADF5611_CP_UP_MSK		    NO_OS_BIT(0)

/* ADF5611 REG002C MAP */
#define ADF5611_CLKODIV_DB_MSK              NO_OS_BIT(7)
#define ADF5611_DCLK_DIV_DB_MSK             NO_OS_BIT(6)
#define ADF5611_SPARE_2C_MSK                NO_OS_BIT(5)
#define ADF5611_RST_SYS_MSK		    NO_OS_BIT(4)
#define ADF5611_EN_ADC_CLK_MSK		    NO_OS_BIT(3)
#define ADF5611_EN_VCAL_MSK		    NO_OS_BIT(2)
#define ADF5611_CAL_CT_SEL_MSK		    NO_OS_BIT(1)
#define ADF5611_EN_NOTCH_MSK		    NO_OS_BIT(0)

/* ADF5611 REG002D Map */
#define ADF5611_VCO_FSM_TEST_MUX_MSK	    NO_OS_GENMASK(7, 5)
#define ADF5611_SPARE_2D_MSK		    NO_OS_GENMASK(4, 3)
#define ADF5611_O_VCO_BIAS_MSK		    NO_OS_BIT(2)
#define ADF5611_O_VCO_BAND_MSK		    NO_OS_BIT(1)
#define ADF5611_O_VCO_CORE_MSK		    NO_OS_BIT(0)


/* ADF5611 REG002F MAP */
#define ADF5611_CAL_COUNT_TO_MSK 	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0030 MAP */
#define ADF5611_CAL_VTUNE_TO_LSB_MSK	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0031 MAP */
#define ADF5611_0_VCO_DB_MSK                NO_OS_BIT(7)
#define ADF5611_CAL_VTUNE_TO_MSB_MSK        NO_OS_GENMASK(6, 0)

/* ADF5611 REG0032 MAP */
#define ADF5611_CAL_VCO_TO_LSB_MSK          NO_OS_GENMASK(7, 0)

/* ADF5611 REG0033 MAP */
#define ADF5611_DEL_CTRL_DB_MSK             NO_OS_BIT(7)
#define ADF5611_CAL_VCO_TO_MSB_MSK          NO_OS_GENMASK(6, 0)

/* ADF5611 REG0034 MAP */
#define ADF5611_CNTR_DIV_WORD_LSB_MSK       NO_OS_GENMASK(7, 0)

/* ADF5611 REG0035 MAP */
#define ADF5611_SPARE_35_MSK                NO_OS_GENMASK(7, 6)
#define ADF5611_CMOS_OV_MSK                 NO_OS_BIT(5)
#define ADF5611_CMOS_OV(x)                  no_os_field_prep(ADF5611_CMOS_OV_MSK, x)
#define ADF5611_READ_MODE_MSK               NO_OS_BIT(4)
#define ADF5611_CNTR_DIV_WORD_MSB_MSK       NO_OS_GENMASK(3, 0)

/* ADF5611 REG0036 MAP */
#define ADF5611_ADC_CLK_DIV_MSK             NO_OS_GENMASK(7, 0)

/* ADF5611 REG0037 MAP */
#define ADF5611_EN_ADC_CNV_MSK		    NO_OS_BIT(7)
#define ADF5611_EN_ADC_VTEST_MSK	    NO_OS_BIT(6)
#define ADF5611_ADC_VTEST_SEL_MSK	    NO_OS_BIT(5)
#define ADF5611_ADC_MUX_SEL_MSK		    NO_OS_BIT(4)
#define ADF5611_ADC_F_CONV_MSK		    NO_OS_BIT(3)
#define ADF5611_ADC_C_CONV_MSK		    NO_OS_BIT(2)
#define ADF5611_EN_ADC_MSK		    NO_OS_BIT(1)
#define ADF5611_ADC_CLK_TEST_SEL_MSK        NO_OS_BIT(0)

/* ADF5611 REG0038 MAP */
#define ADF5611_SPARE_38_MSK                NO_OS_BIT(7)
#define ADF5611_VPTAT_CALGEN_MSK            NO_OS_GENMASK(6, 0)

/* ADF5611 REG0039 MAP */
#define ADF5611_SPARE_39_MSK                NO_OS_BIT(7)
#define ADF5611_VCTAT_CALGEN_MSK            NO_OS_GENMASK(6, 0)

/* ADF5611 REG003A MAP */
#define ADF5611_NVMDIN_MSK                  NO_OS_GENMASK(7, 0)

/* ADF5611 REG003B MAP */
#define ADF5611_SPARE_3B_MSK		    NO_OS_BIT(7)
#define ADF5611_NVMADDR_MSK		    NO_OS_GENMASK(6, 3)
#define ADF5611_NVMNO_OS_BIT_SEL_MSK	    NO_OS_GENMASK(2, 0)

/* ADF5611 REG003C MAP */
#define ADF5611_TRIM_LATCH_MSK		    NO_OS_BIT(7)
#define ADF5611_NVMTEST_MSK		    NO_OS_BIT(6)
#define ADF5611_NVMPROG_MSK		    NO_OS_BIT(5)
#define ADF5611_NVMRD_MSK		    NO_OS_BIT(4)
#define ADF5611_NVMSTART_MSK	            NO_OS_BIT(3)
#define ADF5611_NVMON_MSK	            NO_OS_BIT(2)
#define ADF5611_MARGIN_MSK	            NO_OS_GENMASK(1, 0)

/* ADF5611 REG003D MAP */
#define ADF5611_NVMDOUT_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG003E MAP */
#define ADF5611_SCAN_MODE_CODE_MSK 	    NO_OS_GENMASK(7, 0)

/* ADF5611 REG003F MAP */
#define ADF5611_TEMP_OFFSET_MSK		    NO_OS_GENMASK(7, 0)

/* ADF5611 REG0040 MAP */
#define ADF5611_SPARE_40_MSK		    NO_OS_GENMASK(7, 6)
#define ADF5611_TEMP_SLOPE_MSK		    NO_OS_GENMASK(5, 0)

/* ADF5611 REG0044 MAP */
#define ADF5611_ADC_ST_CNV_MSK		    NO_OS_BIT(0)

/* ADF5611 REG0048 MAP */
#define ADF5611_ADC_BUSY_MSK		    NO_OS_BIT(2)
#define ADF5611_FSM_BUSY_MSK		    NO_OS_BIT(1)
#define ADF5611_LOCKED_MSK		    NO_OS_BIT(0)

/* ADF5611 REG0100 MAP*/
#define ADF5611_CORE0_BIAS_TABLE_1_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE0_BIAS_TABLE_0_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0101 MAP*/
#define ADF5611_CORE0_BIAS_TABLE_3_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE0_BIAS_TABLE_2_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0102 MAP*/
#define ADF5611_CORE0_BIAS_TABLE_5_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE0_BIAS_TABLE_4_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0103 MAP*/
#define ADF5611_CORE0_BIAS_TABLE_7_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE0_BIAS_TABLE_6_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0104 MAP*/
#define ADF5611_CORE1_BIAS_TABLE_1_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE1_BIAS_TABLE_0_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0105 MAP*/
#define ADF5611_CORE1_BIAS_TABLE_3_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE1_BIAS_TABLE_2_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0106 MAP*/
#define ADF5611_CORE1_BIAS_TABLE_5_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE1_BIAS_TABLE_4_MSK      NO_OS_GENMASK(2, 0)

/* ADF5611 REG0107 MAP*/
#define ADF5611_CORE1_BIAS_TABLE_7_MSK      NO_OS_GENMASK(5, 3)
#define ADF5611_CORE1_BIAS_TABLE_6_MSK      NO_OS_GENMASK(2, 0)


#define ADF5611_SPI_4W_CFG(x)		    (no_os_field_prep(ADF5611_SDO_ACTIVE_MSK, x) | \
					     no_os_field_prep(ADF5611_SDO_ACTIVE_R_MSK, x))

#define ADF5611_SPI_SCRATCHPAD_TEST	0x2A

/* ADF5611 SPECIFICATIONS */
#define ADF5611_SPI_WRITE_CMD		0x0
#define ADF5611_SPI_READ_CMD		0x8000
#define ADF5611_SPI_DUMMY_DATA		0x00
#define ADF5611_BUFF_SIZE_BYTES		3
#define ADF5611_VCO_FREQ_MIN		3650000000U // 3.65MHz
#define ADF5611_VCO_FREQ_MAX		7300000000U // 7.3MHz
#define ADF5611_MOD1WORD		0x2000000U   // 2^25
#define ADF5611_MOD2WORD_MAX		0xFFFFFFU    // 2^24 - 1
#define ADF5611_CHANNEL_SPACING_MAX	78125U
#define ADF5611_CPI_VAL_MAX		15
#define ADF5611_REF_DIV_MAX             16383
#define ADF5611_BLEED_TIME_CONST        4
#define ADF5611_BLEED_CURRENT           3125
#define ADF5611_RFOUT_PWR_MAX		3
#define ADF5611_RFOUTDIV_PWR_MAX	ADF5611_RFOUT_PWR_MAX
#define ADF5611_RFOUTDIV_DIV_MAX	7U
#define ADF5611_POR_DELAY_US		200
#define ADF5611_LKD_DELAY_US		500
#define ADF5611_RFOUT_MAX               14600000000U 	//14.6GHz
#define ADF5611_RFOUT_MIN               7300000000U 	//7.3GHz
#define ADF5611_REF_CLK_MAX		300000000000U   //300MHz                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
#define ADF5611_REF_CLK_MIN		50000000U       //50MHz
#define ADF5611_OUTPUT_DOUBLER          0x2U
#define ADF5611_PFD_FREQ_MAX		100000000U	//100MHz
#define ADF5612_RFOUT_MAX               8500000000U 	//8.5GHz
#define ADF5612_RFOUT_MIN               7300000000U 	//7.3GHz
#define ADF5612_VCO_FREQ_MAX            7300000000U 	//7.3GHz
#define KHZ				1000
#define MHZ				KHZ * KHZ
#define GHZ                             KHZ * KHZ * KHZ
#define s_TO_ns				1000000000U
#define ns_TO_ps			1000
#define uA_TO_A				1000000

/***************************************************************************//**
 * @brief The `adf5611_device_id` is an enumeration that defines the supported
 * device IDs for the ADF5611 and ADF5612 devices. This enumeration is
 * used to identify and differentiate between the two types of devices
 * within the driver implementation, allowing for specific configurations
 * and operations to be applied based on the device type.
 *
 * @param ID_ADF5611 Represents the device ID for the ADF5611 device.
 * @param ID_ADF5612 Represents the device ID for the ADF5612 device.
 ******************************************************************************/
enum adf5611_device_id {
	ID_ADF5611,
	ID_ADF5612,
};

/***************************************************************************//**
 * @brief The `adf5611_init_param` structure is used to initialize the ADF5611
 * device, containing parameters for SPI communication, reference and
 * output frequencies, and various configuration settings such as charge
 * pump current and dividers. It is essential for setting up the device
 * with the correct operational parameters before use.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param spi4wire Boolean indicating if SPI 4-wire mode is used.
 * @param cmos_3v3 Boolean indicating if CMOS 3.3V is used.
 * @param ref_clk_freq Reference clock frequency in Hz.
 * @param rfout_freq RF output frequency in Hz.
 * @param ref_div Reference divider value.
 * @param cp_i Charge pump current setting.
 * @param bleed_word Bleed current setting.
 * @param ld_count Lock detect count setting.
 * @param rfoutdiv_div RF output divider value.
 * @param id Device ID from the adf5611_device_id enum.
 ******************************************************************************/
struct adf5611_init_param {
	/** SPI Initialization Parameters*/
	struct no_os_spi_init_param	*spi_init;
	/** SPI 4-Wire */
	bool				spi4wire;
	bool				cmos_3v3;
	uint64_t			ref_clk_freq;
	uint64_t			rfout_freq;
	uint8_t				ref_div;
	uint8_t				cp_i;
	uint16_t			bleed_word;
	uint8_t				ld_count;
	uint8_t				rfoutdiv_div;
	enum adf5611_device_id		id;
};

/***************************************************************************//**
 * @brief The `adf5611_dev` structure is a device descriptor for the ADF5611, a
 * frequency synthesizer. It contains configuration parameters and
 * settings necessary for controlling the device, such as SPI
 * communication details, reference and output frequencies, and various
 * operational settings like charge pump current and frequency limits.
 * This structure is essential for initializing and managing the ADF5611
 * device in a system.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param spi4wire Boolean indicating if SPI 4-wire mode is used.
 * @param cmos_3v3 Boolean indicating if CMOS 3.3V is used.
 * @param ref_clk_freq Frequency of the input reference clock in Hz.
 * @param rfout_freq Frequency of the RF output in Hz.
 * @param ref_div Reference divider value.
 * @param cp_i Charge pump current setting.
 * @param bleed_word Bleed current setting.
 * @param ld_count Lock detect count setting.
 * @param rfoutdiv_div RF output divider value.
 * @param freq_max Maximum frequency limit in Hz.
 * @param freq_min Minimum frequency limit in Hz.
 * @param vco_max Maximum VCO frequency in Hz.
 * @param vco_min Minimum VCO frequency in Hz.
 ******************************************************************************/
struct adf5611_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** SPI 3-Wire */
	bool				spi4wire;
	bool				cmos_3v3;
	/** Input Reference Clock */
	uint64_t			ref_clk_freq;
	/** Input Reference Clock */
	uint64_t			rfout_freq;
	uint32_t			ref_div;
	uint8_t				cp_i;
	uint16_t			bleed_word;
	uint8_t				ld_count;
	uint8_t				rfoutdiv_div;
	uint64_t			freq_max;
	uint64_t			freq_min;
	uint64_t			vco_max;
	uint64_t			vco_min;
};

/***************************************************************************//**
 * @brief The `reg_sequence` structure is used to define a register and its
 * corresponding value for the ADF5611 device. It is typically used to
 * store default register values or to facilitate register configuration
 * by specifying the register address and the value to be written. This
 * structure is essential for managing the configuration of the ADF5611
 * device through SPI communication.
 *
 * @param reg A 16-bit unsigned integer representing the register address.
 * @param val An 8-bit unsigned integer representing the value to be written to
 * the register.
 ******************************************************************************/
struct reg_sequence {
	uint16_t reg;
	uint8_t val;
};

/***************************************************************************//**
 * @brief The `adf5611_reg_defaults` is a static constant array of `struct
 * reg_sequence` that holds default register values for the ADF5611
 * device. Each element in the array consists of a register address and
 * its corresponding default value. This array is used to initialize the
 * ADF5611 registers to their default states during device setup.
 *
 * @details This variable is used to set the default register values for the
 * ADF5611 device during initialization.
 ******************************************************************************/
static const struct reg_sequence adf5611_reg_defaults[] = {
	{ 0x0000, 0x18 },
	{ 0x001F, 0x02 },
	{ 0x0020, 0x00 },
	{ 0x001E, 0xE2 },
	{ 0x002C, 0x08 },
	{ 0x001F, 0xF3 },
	{ 0x0104, 0x24 },
	{ 0x0105, 0x24 },
	{ 0x0106, 0x24 },
	{ 0x0107, 0x24 },
	{ 0x0040, 0x2B },
	{ 0x003F, 0x5D },
	{ 0x0039, 0x22 },
	{ 0x0038, 0x4E },
	{ 0x0037, 0x82 },
	{ 0x0036, 0x3E },
	{ 0x0035, 0x00 },
	{ 0x0033, 0x00 },
	{ 0x0032, 0x64 },
	{ 0x0031, 0x00 },
	{ 0x0030, 0x32 },
	{ 0x002F, 0x19 },
	{ 0x002D, 0x00 },
	{ 0x002C, 0x08 },
	{ 0x002B, 0x00 },
	{ 0x002A, 0x70 },
	{ 0x0029, 0x2C },
	{ 0x0028, 0x01 },
	{ 0x0027, 0x00 },
	{ 0x0026, 0x00 },
	{ 0x0025, 0x7f },
	{ 0x0024, 0xff },
	{ 0x0023, 0xff },
	{ 0x0022, 0x18 },
	{ 0x0021, 0x5f },
	{ 0x0020, 0x00 },
	{ 0x001F, 0x02 },
	{ 0x001E, 0x92 },
	{ 0x001D, 0x11 },
	{ 0x001C, 0x00 },
	{ 0x001B, 0x00 },
	{ 0x001A, 0x01 },
	{ 0x0019, 0x00 },
	{ 0x0018, 0x00 },
	{ 0x0017, 0x00 },
	{ 0x0016, 0x00 },
	{ 0x0015, 0x00 },
	{ 0x0014, 0x00 },
	{ 0x0013, 0x00 },
	{ 0x0012, 0x00 },
	{ 0x0010, 0x78 },
};

/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * of the ADF5611 device using the SPI interface. It is essential to
 * ensure that the device has been properly initialized and that the SPI
 * descriptor within the device structure is valid before calling this
 * function. The function constructs a command based on the register
 * address and data, and then sends it over SPI. It is important to
 * handle the return value to check for any communication errors.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null and should be properly initialized with a valid
 * SPI descriptor.
 * @param reg_addr A 16-bit unsigned integer representing the register address
 * to which data will be written. The address should be within
 * the valid range of the device's register map.
 * @param data An 8-bit unsigned integer representing the data to be written to
 * the specified register. The data should be formatted according to
 * the register's requirements.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error in the SPI communication.
 ******************************************************************************/
int adf5611_spi_write(struct adf5611_dev *dev, uint16_t reg_addr, uint8_t data);

/***************************************************************************//**
 * @brief Use this function to read a specific register from the ADF5611 device
 * using SPI communication. It is essential to ensure that the device has
 * been properly initialized and configured before calling this function.
 * The function sends a read command to the specified register address
 * and retrieves the data. It is important to handle the return value to
 * check for any communication errors.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Valid
 * register addresses depend on the device specification.
 * @param data A pointer to a uint8_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int adf5611_spi_read(struct adf5611_dev *dev, uint16_t reg_addr, uint8_t *data);

/***************************************************************************//**
 * @brief Use this function to modify specific bits in a register of the ADF5611
 * device over SPI. It reads the current value of the register, applies
 * the mask to clear the bits, and then sets the bits according to the
 * provided data. This function should be called when you need to change
 * specific settings in a register without affecting other bits. Ensure
 * that the device is properly initialized before calling this function.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev Pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADF5611 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Only the bits set in the mask will be affected.
 * @param data The new values for the bits specified by the mask. Bits not
 * covered by the mask are ignored.
 * @return Returns 0 on success, or a negative error code if the SPI read or
 * write operation fails.
 ******************************************************************************/
int adf5611_spi_update_bits(struct adf5611_dev *dev, uint16_t reg_addr,
			    uint8_t mask, uint8_t data);

/***************************************************************************//**
 * @brief This function configures the reference clock frequency for the
 * specified ADF5611 device. It should be called when you need to update
 * the reference clock frequency, ensuring it is within the valid range
 * defined by the device specifications. If the provided frequency
 * exceeds the maximum or is below the minimum allowed values, it will be
 * clamped to the nearest valid limit. The function must be called with a
 * valid device descriptor, and it will return an error if the descriptor
 * is null.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param val The desired reference clock frequency in Hz. Valid values are
 * between ADF5611_REF_CLK_MIN (50 MHz) and ADF5611_REF_CLK_MAX (300
 * MHz). Values outside this range will be clamped.
 * @return Returns 0 on success or a negative error code if the device
 * descriptor is null or if setting the frequency fails.
 ******************************************************************************/
int adf5611_set_ref_clk(struct adf5611_dev *dev, uint64_t val);

/***************************************************************************//**
 * @brief Use this function to obtain the current reference clock frequency of
 * an initialized ADF5611 device. It is essential to ensure that the
 * device pointer is valid before calling this function. The function
 * will store the reference clock frequency in the provided memory
 * location. If the device pointer is null, the function will return an
 * error code.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. If null, the function returns -EINVAL.
 * @param val A pointer to a uint64_t variable where the reference clock
 * frequency will be stored. Must not be null.
 * @return Returns 0 on success, or -EINVAL if the device pointer is null.
 ******************************************************************************/
int adf5611_get_ref_clk(struct adf5611_dev *dev, uint64_t *val);

/***************************************************************************//**
 * @brief This function sets the reference divider value for the ADF5611 device,
 * which is used to divide the input reference clock frequency. It must
 * be called with a valid device descriptor that has been initialized.
 * The function ensures that the divider value does not exceed the
 * maximum allowed value by clamping it if necessary. After setting the
 * divider, it updates the device's frequency settings. This function
 * should be used when the reference clock division needs to be adjusted,
 * and it returns an error code if the device descriptor is null.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null. The function returns an error if this
 * parameter is null.
 * @param div An integer representing the desired reference divider value. Valid
 * values are from 0 to ADF5611_REF_DIV_MAX. Values greater than
 * ADF5611_REF_DIV_MAX are clamped to ADF5611_REF_DIV_MAX.
 * @return Returns 0 on success or a negative error code if the device
 * descriptor is null or if there is an error updating the frequency
 * settings.
 ******************************************************************************/
int adf5611_set_ref_div(struct adf5611_dev *dev, int32_t div);

/***************************************************************************//**
 * @brief This function is used to obtain the current reference divider value
 * from an ADF5611 device. It reads the necessary registers over SPI to
 * determine the divider value and stores it in the provided output
 * parameter. This function should be called when the reference divider
 * value is needed for configuration or diagnostic purposes. It is
 * important to ensure that the device has been properly initialized
 * before calling this function to avoid communication errors.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param div A pointer to an int32_t where the reference divider value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adf5611_get_ref_div(struct adf5611_dev *dev, int32_t *div);

/***************************************************************************//**
 * @brief This function configures the charge pump current of the ADF5611 device
 * by setting the appropriate register value. It should be called when
 * the charge pump current needs to be adjusted as part of the device
 * configuration. The function ensures that the provided current value
 * does not exceed the maximum allowable value by clamping it if
 * necessary. After setting the charge pump current, the function updates
 * the device frequency settings. This function must be called with a
 * valid device descriptor that has been properly initialized.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null and should be initialized before calling this
 * function. The caller retains ownership.
 * @param reg_val An integer representing the desired charge pump current
 * setting. Valid values range from 0 to ADF5611_CPI_VAL_MAX
 * (15). Values greater than the maximum are clamped to
 * ADF5611_CPI_VAL_MAX.
 * @return Returns an integer status code from the adf5611_set_freq function,
 * indicating success or failure of the frequency update.
 ******************************************************************************/
int adf5611_set_cp_i(struct adf5611_dev *dev, int32_t reg_val);

/***************************************************************************//**
 * @brief Use this function to obtain the current charge pump setting from an
 * ADF5611 device. It reads the relevant register via SPI and updates the
 * provided variable with the charge pump current value. This function
 * should be called when you need to verify or log the current charge
 * pump setting. Ensure that the device has been properly initialized
 * before calling this function to avoid communication errors.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param reg_val A pointer to an int32_t where the charge pump current value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adf5611_get_cp_i(struct adf5611_dev *dev, int32_t *reg_val);

/***************************************************************************//**
 * @brief This function configures the output power level of the ADF5611 device.
 * It should be called when the user needs to adjust the power output of
 * the device. The function ensures that the power level does not exceed
 * the maximum allowed value by clamping it if necessary. It requires a
 * valid device descriptor and a power level within the acceptable range.
 * The function communicates with the device over SPI to update the power
 * setting.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param pwr An int8_t representing the desired output power level. Valid
 * values are from 0 to ADF5611_RFOUT_PWR_MAX. Values above the
 * maximum are clamped to ADF5611_RFOUT_PWR_MAX.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the SPI communication.
 ******************************************************************************/
int adf5611_set_output_power(struct adf5611_dev *dev, int8_t pwr);

/***************************************************************************//**
 * @brief Use this function to obtain the current output power setting of the
 * ADF5611 device. It is essential to call this function after the device
 * has been properly initialized and configured. The function reads the
 * relevant register from the device and extracts the output power value.
 * Ensure that the provided device structure is valid and that the
 * pointer for storing the power value is not null. The function returns
 * an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param pwr A pointer to an int8_t variable where the output power value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adf5611_get_output_power(struct adf5611_dev *dev, int8_t *pwr);

/***************************************************************************//**
 * @brief This function sets the power level for the RF output divider of the
 * ADF5611 device. It should be called when you need to adjust the power
 * level of the RF output divider to a specific value. The function
 * ensures that the power level does not exceed the maximum allowed value
 * by clamping it if necessary. It is important to ensure that the device
 * has been properly initialized before calling this function.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null, and the device must be initialized.
 * @param pwr An integer representing the desired power level for the RF output
 * divider. Valid values range from 0 to ADF5611_RFOUTDIV_PWR_MAX.
 * Values greater than the maximum are clamped to
 * ADF5611_RFOUTDIV_PWR_MAX.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adf5611_set_rfoutdiv_power(struct adf5611_dev *dev, int32_t pwr);

/***************************************************************************//**
 * @brief This function is used to obtain the current RF output divider power
 * setting from an ADF5611 device. It should be called when you need to
 * read the power setting for the RF output divider, typically for
 * monitoring or configuration purposes. The function requires a valid
 * device descriptor and a pointer to an integer where the power setting
 * will be stored. It is important to ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the read operation fails.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * This must be a valid, initialized device descriptor and must not
 * be null.
 * @param pwr A pointer to an int32_t where the RF output divider power setting
 * will be stored. This pointer must not be null, and the caller is
 * responsible for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adf5611_get_rfoutdiv_power(struct adf5611_dev *dev, int32_t *pwr);

/***************************************************************************//**
 * @brief This function configures the RF output divider of the ADF5611 device
 * to the specified value. It should be called when the RF output divider
 * needs to be set or adjusted. The function ensures that the divider
 * value does not exceed the maximum allowed value by clamping it if
 * necessary. It requires a valid device descriptor and updates the
 * device's configuration via SPI communication. This function must be
 * called after the device has been properly initialized.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param div_val The desired RF output divider value as an 8-bit unsigned
 * integer. Valid values range from 0 to
 * ADF5611_RFOUTDIV_DIV_MAX. Values exceeding the maximum are
 * clamped to ADF5611_RFOUTDIV_DIV_MAX.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device pointer is null.
 ******************************************************************************/
int adf5611_set_rfoutdiv_divider(struct adf5611_dev *dev, uint8_t div_val);

/***************************************************************************//**
 * @brief This function is used to obtain the current RF output divider value
 * from an ADF5611 device. It should be called when you need to know the
 * current divider setting for the RF output. The function requires a
 * valid device descriptor and a pointer to store the retrieved divider
 * value. It reads the necessary register via SPI and extracts the
 * divider value, storing it in the provided pointer. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The device should be initialized before calling
 * this function.
 * @param div A pointer to an int8_t where the RF output divider value will be
 * stored. Must not be null. The function writes the retrieved
 * divider value to this location.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adf5611_get_rfoutdiv_divider(struct adf5611_dev *dev, int8_t *div);

/***************************************************************************//**
 * @brief This function is used to enable or disable the RF output divider on
 * the ADF5611 device. It should be called when you need to control the
 * state of the RF output divider, typically as part of configuring the
 * device for a specific operation. Ensure that the device has been
 * properly initialized before calling this function. The function
 * updates the relevant register to reflect the desired state.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param en A boolean value indicating whether to enable (true) or disable
 * (false) the RF output divider.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adf5611_set_en_rfoutdiv(struct adf5611_dev *dev, bool en);

/***************************************************************************//**
 * @brief Use this function to check whether the RF output divider is enabled or
 * disabled on the ADF5611 device. It must be called with a valid device
 * descriptor and a pointer to a boolean variable where the result will
 * be stored. This function is typically used in scenarios where the
 * configuration of the RF output needs to be verified or adjusted.
 * Ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null. The function will return an error if the device
 * is not properly initialized or if communication fails.
 * @param en A pointer to a boolean variable where the enable status of the RF
 * output divider will be stored. Must not be null. The function will
 * write true if the divider is enabled, false otherwise.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adf5611_get_en_rfoutdiv(struct adf5611_dev *dev, bool *en);

/***************************************************************************//**
 * @brief This function sets the RF output frequency of the ADF5611 device to
 * the specified value. It ensures that the frequency is within the valid
 * range by clamping it to the maximum or minimum allowed values if
 * necessary. This function should be called when you need to update the
 * RF output frequency of the device, and it must be used with a properly
 * initialized device structure. The function returns an integer status
 * code indicating the success or failure of the operation.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param val The desired RF output frequency in Hz. The value will be clamped
 * to the range [ADF5611_RFOUT_MIN, ADF5611_RFOUT_MAX] if it falls
 * outside this range.
 * @return Returns an integer status code from the adf5611_set_freq function,
 * indicating success or failure of the frequency setting operation.
 ******************************************************************************/
int adf5611_set_rfout(struct adf5611_dev *dev, uint64_t val);

/***************************************************************************//**
 * @brief Use this function to obtain the current RF output frequency from an
 * ADF5611 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function reads
 * multiple registers from the device to compute the frequency and stores
 * the result in the provided memory location. This function returns an
 * error code if any of the SPI read operations fail, indicating that the
 * frequency could not be retrieved.
 *
 * @param dev A pointer to an initialized adf5611_dev structure representing the
 * device. Must not be null.
 * @param val A pointer to a uint64_t variable where the RF output frequency
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if a SPI read
 * operation fails.
 ******************************************************************************/
int adf5611_get_rfout(struct adf5611_dev *dev, uint64_t *val);

/***************************************************************************//**
 * @brief This function configures the frequency of the ADF5611 device based on
 * the parameters set in the device structure. It must be called after
 * the device has been properly initialized and configured with the
 * desired frequency settings. The function handles various internal
 * configurations and calibrations necessary to achieve the specified
 * frequency. It returns an error code if any step in the configuration
 * process fails, ensuring that the device is correctly set up before
 * returning control to the caller.
 *
 * @param dev A pointer to an initialized adf5611_dev structure. This structure
 * must be properly configured with the desired frequency and other
 * relevant parameters before calling this function. The pointer must
 * not be null, and the caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the frequency
 * setting process fails at any step.
 ******************************************************************************/
int adf5611_set_freq(struct adf5611_dev *dev);

/***************************************************************************//**
 * @brief This function sets up and initializes an ADF5611 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function configures the SPI interface,
 * sets device-specific parameters, and writes default register values.
 * It also performs a reset and a basic communication test to ensure the
 * device is operational. If initialization fails at any step, resources
 * are cleaned up and an error code is returned.
 *
 * @param dev A pointer to a pointer of an adf5611_dev structure. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to an adf5611_init_param structure containing the
 * initialization parameters. Must not be null and should be
 * properly populated with valid configuration data.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the dev pointer is updated to point to a newly allocated and
 * initialized adf5611_dev structure.
 ******************************************************************************/
int adf5611_init(struct adf5611_dev **device,
		 struct adf5611_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly remove an ADF5611 device instance and
 * release any associated resources. It should be called when the device
 * is no longer needed, typically during application shutdown or when
 * reinitializing the device. This function ensures that the SPI
 * descriptor associated with the device is removed and the device memory
 * is freed if the SPI removal fails. It is important to ensure that the
 * device pointer is valid and initialized before calling this function.
 *
 * @param dev A pointer to an adf5611_dev structure representing the device
 * instance to be removed. Must not be null and should point to a
 * valid, initialized device structure. The function does not handle
 * null pointers and expects the caller to ensure the validity of the
 * input.
 * @return Returns 0 on successful removal of the device instance. The function
 * does not return error codes for SPI removal failures, but it attempts
 * to free the device memory in such cases.
 ******************************************************************************/
int adf5611_remove(struct adf5611_dev *dev);

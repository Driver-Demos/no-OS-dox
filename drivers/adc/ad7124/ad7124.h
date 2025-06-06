/***************************************************************************//**
*   @file    ad7124.h
*   @brief   AD7124 header file.
*   	     Devices: AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015-2019, 2023(c) Analog Devices, Inc.
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.******************************************************************************/
#ifndef __AD7124_H__
#define __AD7124_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_delay.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define	AD7124_RW 1   /* Read and Write */
#define	AD7124_R  2   /* Read only */
#define AD7124_W  3   /* Write only */

/* Total Number of Setups */
#define AD7124_MAX_SETUPS	8
/* Maximum number of channels */
#define AD7124_MAX_CHANNELS	16

/* AD7124-4 Standard Device ID */
#define AD7124_4_STD_ID  0x04
/* AD7124-4 B Grade Device ID */
#define AD7124_4_B_GRADE_ID  0x06
/* Device ID for the re-designed die in the AD7124-4 standard part and B-grade */
#define AD7124_4_NEW_ID  0x07

/* AD7124-8 Standard Device ID */
#define AD7124_8_STD_ID  0x14
/* AD7124-8 B and W Grade Device ID */
#define AD7124_8_B_W_GRADE_ID  0x16
/* Device ID for the re-designed die in the AD7124-8 standard part, B-grade and W-grade */
#define AD7124_8_NEW_ID  0x17

/* AD7124 Register Map */
#define AD7124_COMM_REG      0x00
#define AD7124_STATUS_REG    0x00
#define AD7124_ADC_CTRL_REG  0x01
#define AD7124_DATA_REG      0x02
#define AD7124_IO_CTRL1_REG  0x03
#define AD7124_IO_CTRL2_REG  0x04
#define AD7124_ID_REG        0x05
#define AD7124_ERR_REG       0x06
#define AD7124_ERREN_REG     0x07
#define AD7124_CH0_MAP_REG   0x09
#define AD7124_CH1_MAP_REG   0x0A
#define AD7124_CH2_MAP_REG   0x0B
#define AD7124_CH3_MAP_REG   0x0C
#define AD7124_CH4_MAP_REG   0x0D
#define AD7124_CH5_MAP_REG   0x0E
#define AD7124_CH6_MAP_REG   0x0F
#define AD7124_CH7_MAP_REG   0x10
#define AD7124_CH8_MAP_REG   0x11
#define AD7124_CH9_MAP_REG   0x12
#define AD7124_CH10_MAP_REG  0x13
#define AD7124_CH11_MAP_REG  0x14
#define AD7124_CH12_MAP_REG  0x15
#define AD7124_CH13_MAP_REG  0x16
#define AD7124_CH14_MAP_REG  0x17
#define AD7124_CH15_MAP_REG  0x18
#define AD7124_CFG0_REG      0x19
#define AD7124_CFG1_REG      0x1A
#define AD7124_CFG2_REG      0x1B
#define AD7124_CFG3_REG      0x1C
#define AD7124_CFG4_REG      0x1D
#define AD7124_CFG5_REG      0x1E
#define AD7124_CFG6_REG      0x1F
#define AD7124_CFG7_REG      0x20
#define AD7124_FILT0_REG     0x21
#define AD7124_FILT1_REG     0x22
#define AD7124_FILT2_REG     0x23
#define AD7124_FILT3_REG     0x24
#define AD7124_FILT4_REG     0x25
#define AD7124_FILT5_REG     0x26
#define AD7124_FILT6_REG     0x27
#define AD7124_FILT7_REG     0x28
#define AD7124_OFFS0_REG     0x29
#define AD7124_OFFS1_REG     0x2A
#define AD7124_OFFS2_REG     0x2B
#define AD7124_OFFS3_REG     0x2C
#define AD7124_OFFS4_REG     0x2D
#define AD7124_OFFS5_REG     0x2E
#define AD7124_OFFS6_REG     0x2F
#define AD7124_OFFS7_REG     0x30
#define AD7124_GAIN0_REG     0x31
#define AD7124_GAIN1_REG     0x32
#define AD7124_GAIN2_REG     0x33
#define AD7124_GAIN3_REG     0x34
#define AD7124_GAIN4_REG     0x35
#define AD7124_GAIN5_REG     0x36
#define AD7124_GAIN6_REG     0x37
#define AD7124_GAIN7_REG     0x38

/* Communication Register bits */
#define AD7124_COMM_REG_WEN    (0 << 7)
#define AD7124_COMM_REG_WR     (0 << 6)
#define AD7124_COMM_REG_RD     (1 << 6)
#define AD7124_COMM_REG_RA(x)  ((x) & 0x3F)

/* Status Register bits */
#define AD7124_STATUS_REG_RDY          (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG   (1 << 6)
#define AD7124_STATUS_REG_POR_FLAG     (1 << 4)
#define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

/* ADC_Control Register bits */
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   (1 << 12)
#define AD7124_ADC_CTRL_REG_CONT_READ      (1 << 11)
#define AD7124_ADC_CTRL_REG_DATA_STATUS    (1 << 10)
#define AD7124_ADC_CTRL_REG_CS_EN          (1 << 9)
#define AD7124_ADC_CTRL_REG_REF_EN         (1 << 8)
#define AD7124_ADC_CTRL_REG_POWER_MODE(x)  (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_REG_MODE(x)        (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_REG_CLK_SEL(x)     (((x) & 0x3) << 0)

/* IO_Control_1 Register bits */
#define AD7124_IO_CTRL1_REG_GPIO_DAT2     (1 << 23)
#define AD7124_IO_CTRL1_REG_GPIO_DAT1     (1 << 22)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL2    (1 << 19)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL1    (1 << 18)
#define AD7124_IO_CTRL1_REG_PDSW          (1 << 15)
#define AD7124_IO_CTRL1_REG_IOUT1(x)      (((x) & 0x7) << 11)
#define AD7124_IO_CTRL1_REG_IOUT0(x)      (((x) & 0x7) << 8)
#define AD7124_IO_CTRL1_REG_IOUT_CH1(x)   (((x) & 0xF) << 4)
#define AD7124_IO_CTRL1_REG_IOUT_CH0(x)   (((x) & 0xF) << 0)

/* IO_Control_1 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT4     (1 << 23)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT3     (1 << 22)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT2     (1 << 21)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT1     (1 << 20)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL4    (1 << 19)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL3    (1 << 18)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL2    (1 << 17)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL1    (1 << 16)

/* IO_Control_2 Register bits */
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 15)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 14)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 11)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 10)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 5)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 4)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* IO_Control_2 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15  (1 << 15)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14  (1 << 14)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13  (1 << 13)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12  (1 << 12)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11  (1 << 11)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10  (1 << 10)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9   (1 << 9)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8   (1 << 8)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 7)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 6)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 5)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 4)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 3)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 2)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* ID Register bits */
#define AD7124_ID_REG_DEVICE_ID(x)   (((x) & 0xF) << 4)
#define AD7124_ID_REG_SILICON_REV(x) (((x) & 0xF) << 0)

/* Error Register bits */
#define AD7124_ERR_REG_LDO_CAP_ERR        (1 << 19)
#define AD7124_ERR_REG_ADC_CAL_ERR        (1 << 18)
#define AD7124_ERR_REG_ADC_CONV_ERR       (1 << 17)
#define AD7124_ERR_REG_ADC_SAT_ERR        (1 << 16)
#define AD7124_ERR_REG_AINP_OV_ERR        (1 << 15)
#define AD7124_ERR_REG_AINP_UV_ERR        (1 << 14)
#define AD7124_ERR_REG_AINM_OV_ERR        (1 << 13)
#define AD7124_ERR_REG_AINM_UV_ERR        (1 << 12)
#define AD7124_ERR_REG_REF_DET_ERR        (1 << 11)
#define AD7124_ERR_REG_DLDO_PSM_ERR       (1 << 9)
#define AD7124_ERR_REG_ALDO_PSM_ERR       (1 << 7)
#define AD7124_ERR_REG_SPI_IGNORE_ERR     (1 << 6)
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR   (1 << 5)
#define AD7124_ERR_REG_SPI_READ_ERR       (1 << 4)
#define AD7124_ERR_REG_SPI_WRITE_ERR      (1 << 3)
#define AD7124_ERR_REG_SPI_CRC_ERR        (1 << 2)
#define AD7124_ERR_REG_MM_CRC_ERR         (1 << 1)
#define AD7124_ERR_REG_ROM_CRC_ERR        (1 << 0)

/* Error_En Register bits */
#define AD7124_ERREN_REG_MCLK_CNT_EN           (1 << 22)
#define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   (1 << 21)
#define AD7124_ERREN_REG_LDO_CAP_CHK(x)        (((x) & 0x3) << 19)
#define AD7124_ERREN_REG_ADC_CAL_ERR_EN        (1 << 18)
#define AD7124_ERREN_REG_ADC_CONV_ERR_EN       (1 << 17)
#define AD7124_ERREN_REG_ADC_SAT_ERR_EN        (1 << 16)
#define AD7124_ERREN_REG_AINP_OV_ERR_EN        (1 << 15)
#define AD7124_ERREN_REG_AINP_UV_ERR_EN        (1 << 14)
#define AD7124_ERREN_REG_AINM_OV_ERR_EN        (1 << 13)
#define AD7124_ERREN_REG_AINM_UV_ERR_EN        (1 << 12)
#define AD7124_ERREN_REG_REF_DET_ERR_EN        (1 << 11)
#define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN (1 << 10)
#define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      (1 << 9)
#define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN (1 << 8)
#define AD7124_ERREN_REG_ALDO_PSM_ERR_EN       (1 << 7)
#define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     (1 << 6)
#define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   (1 << 5)
#define AD7124_ERREN_REG_SPI_READ_ERR_EN       (1 << 4)
#define AD7124_ERREN_REG_SPI_WRITE_ERR_EN      (1 << 3)
#define AD7124_ERREN_REG_SPI_CRC_ERR_EN        (1 << 2)
#define AD7124_ERREN_REG_MM_CRC_ERR_EN         (1 << 1)
#define AD7124_ERREN_REG_ROM_CRC_ERR_EN        (1 << 0)

/* Channel Registers 0-15 bits */
#define AD7124_CH_MAP_REG_CH_ENABLE    (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x)      (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x)      (((x) & 0x1F) << 0)

/* Configuration Registers 0-7 bits */
#define AD7124_CFG_REG_BIPOLAR     (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x)  (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP    (1 << 8)
#define AD7124_CFG_REG_REF_BUFM    (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP    (1 << 6)
#define AD7124_CFG_REG_AINN_BUFM   (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x)  ((x) & 0x3) << 3
#define AD7124_CFG_REG_PGA(x)      (((x) & 0x7) << 0)

/* Filter Register 0-7 bits */
#define AD7124_FILT_REG_FILTER(x)         (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60             (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x)    (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE      (1 << 16)
#define AD7124_FILT_REG_FS(x)             (((x) & 0x7FF) << 0)

#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
#define AD7124_DISABLE_CRC 0
#define AD7124_USE_CRC 1
#define AD7124_CHMAP_REG_SETUP_SEL_MSK  	NO_OS_GENMASK(14,12)
#define AD7124_CHMAP_REG_AINPOS_MSK    		NO_OS_GENMASK(9,5)
#define AD7124_CHMAP_REG_AINNEG_MSK    		NO_OS_GENMASK(4,0)
#define AD7124_ADC_CTRL_REG_MODE_MSK   		NO_OS_GENMASK(5,2)
#define AD7124_SETUP_CONF_REG_REF_SEL_MSK	NO_OS_GENMASK(4,3)
#define AD7124_REF_BUF_MSK                  NO_OS_GENMASK(8,7)
#define AD7124_AIN_BUF_MSK                  NO_OS_GENMASK(6,5)
#define AD7124_POWER_MODE_MSK			    NO_OS_GENMASK(7,6)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ad7124_device_type` enumeration defines the types of AD7124
 * devices, specifically distinguishing between the AD7124-4 and AD7124-8
 * models. This enumeration is used to identify and differentiate between
 * these two variants of the AD7124 analog-to-digital converter (ADC)
 * within the software, allowing for device-specific configurations and
 * operations.
 *
 * @param ID_AD7124_4 Represents the AD7124-4 device type.
 * @param ID_AD7124_8 Represents the AD7124-8 device type.
 ******************************************************************************/
enum ad7124_device_type {
	ID_AD7124_4,
	ID_AD7124_8
};

/***************************************************************************//**
 * @brief The `ad7124_mode` enumeration defines the various operational modes
 * for the AD7124 analog-to-digital converter (ADC). These modes include
 * continuous and single conversion modes, as well as various calibration
 * modes for both input and system levels. The enumeration provides a
 * structured way to manage the ADC's operational state, allowing for
 * efficient switching between different modes of operation depending on
 * the application requirements.
 *
 * @param AD7124_CONTINUOUS Represents the continuous conversion mode.
 * @param AD7124_SINGLE Represents the single conversion mode.
 * @param AD7124_STANDBY Represents the standby mode.
 * @param AD7124_POWER_DOWN Represents the power-down mode.
 * @param AD7124_IDLE Represents the idle mode.
 * @param AD7124_IN_ZERO_SCALE_OFF Represents the input zero-scale offset
 * calibration mode.
 * @param AD7124_IN_FULL_SCALE_GAIN Represents the input full-scale gain
 * calibration mode.
 * @param AD7124_SYS_ZERO_SCALE_OFF Represents the system zero-scale offset
 * calibration mode.
 * @param AD7124_SYS_ZERO_SCALE_GAIN Represents the system full-scale gain
 * calibration mode.
 * @param ADC_MAX_MODES Defines the maximum number of modes available.
 ******************************************************************************/
enum ad7124_mode {
	AD7124_CONTINUOUS,
	AD7124_SINGLE,
	AD7124_STANDBY,
	AD7124_POWER_DOWN,
	AD7124_IDLE,
	AD7124_IN_ZERO_SCALE_OFF,
	AD7124_IN_FULL_SCALE_GAIN,
	AD7124_SYS_ZERO_SCALE_OFF,
	AD7124_SYS_ZERO_SCALE_GAIN,
	ADC_MAX_MODES
};

/***************************************************************************//**
 * @brief The `ad7124_analog_input` enumeration defines various analog input
 * sources for the AD7124 ADC channels. It includes standard analog input
 * channels (AIN0 to AIN15), a temperature sensor input, and several
 * reference and ground inputs. This enumeration is used to specify the
 * input source for each channel in the AD7124 ADC configuration,
 * allowing for flexible mapping of inputs to channels based on the
 * application's requirements.
 *
 * @param AD7124_AIN0 Represents the first analog input channel.
 * @param AD7124_AIN1 Represents the second analog input channel.
 * @param AD7124_AIN2 Represents the third analog input channel.
 * @param AD7124_AIN3 Represents the fourth analog input channel.
 * @param AD7124_AIN4 Represents the fifth analog input channel.
 * @param AD7124_AIN5 Represents the sixth analog input channel.
 * @param AD7124_AIN6 Represents the seventh analog input channel.
 * @param AD7124_AIN7 Represents the eighth analog input channel.
 * @param AD7124_AIN8 Represents the ninth analog input channel.
 * @param AD7124_AIN9 Represents the tenth analog input channel.
 * @param AD7124_AIN10 Represents the eleventh analog input channel.
 * @param AD7124_AIN11 Represents the twelfth analog input channel.
 * @param AD7124_AIN12 Represents the thirteenth analog input channel.
 * @param AD7124_AIN13 Represents the fourteenth analog input channel.
 * @param AD7124_AIN14 Represents the fifteenth analog input channel.
 * @param AD7124_AIN15 Represents the sixteenth analog input channel.
 * @param AD7124_TEMP_SENSOR Represents the temperature sensor input.
 * @param AD7124_AVSS Represents the analog ground reference.
 * @param AD7124_IN_REF Represents the internal reference input.
 * @param AD7124_DGND Represents the digital ground reference.
 * @param AD7124_AVDD_AVSS_P Represents the positive analog supply reference.
 * @param AD7124_AVDD_AVSS_M Represents the negative analog supply reference.
 * @param AD7124_IOVDD_DGND_P Represents the positive digital supply reference.
 * @param AD7124_IOVDD_DGND_M Represents the negative digital supply reference.
 * @param AD7124_ALDO_AVSS_P Represents the positive analog LDO supply
 * reference.
 * @param AD7124_ALDO_AVSS_M Represents the negative analog LDO supply
 * reference.
 * @param AD7124_DLDO_DGND_P Represents the positive digital LDO supply
 * reference.
 * @param AD7124_DLDO_DGND_M Represents the negative digital LDO supply
 * reference.
 * @param AD7124_V_20MV_P Represents the positive 20mV reference.
 * @param AD7124_V_20MV_M Represents the negative 20mV reference.
 ******************************************************************************/
enum ad7124_analog_input {
	AD7124_AIN0,
	AD7124_AIN1,
	AD7124_AIN2,
	AD7124_AIN3,
	AD7124_AIN4,
	AD7124_AIN5,
	AD7124_AIN6,
	AD7124_AIN7,
	AD7124_AIN8,
	AD7124_AIN9,
	AD7124_AIN10,
	AD7124_AIN11,
	AD7124_AIN12,
	AD7124_AIN13,
	AD7124_AIN14,
	AD7124_AIN15,
	AD7124_TEMP_SENSOR,
	AD7124_AVSS,
	AD7124_IN_REF,
	AD7124_DGND,
	AD7124_AVDD_AVSS_P,
	AD7124_AVDD_AVSS_M,
	AD7124_IOVDD_DGND_P,
	AD7124_IOVDD_DGND_M,
	AD7124_ALDO_AVSS_P,
	AD7124_ALDO_AVSS_M,
	AD7124_DLDO_DGND_P,
	AD7124_DLDO_DGND_M,
	AD7124_V_20MV_P,
	AD7124_V_20MV_M
};

/***************************************************************************//**
 * @brief The `ad7124_analog_inputs` structure is used to define the positive
 * and negative analog input channels for the AD7124 device, which is a
 * precision analog-to-digital converter. This structure allows the
 * configuration of input channels by specifying which analog inputs are
 * used as the positive and negative terminals, facilitating differential
 * input measurements.
 *
 * @param ainp Represents the positive analog input channel for the AD7124
 * device.
 * @param ainm Represents the negative analog input channel for the AD7124
 * device.
 ******************************************************************************/
struct ad7124_analog_inputs {
	enum ad7124_analog_input ainp;
	enum ad7124_analog_input ainm;
};

/***************************************************************************//**
 * @brief The `ad7124_channel_map` structure is used to define the configuration
 * of a channel in the AD7124 ADC device. It includes a flag to enable or
 * disable the channel, a selection for the setup configuration, and a
 * nested structure for specifying the analog inputs. This structure is
 * crucial for mapping the ADC's channels to their respective
 * configurations and inputs, allowing for flexible and precise control
 * over the ADC's operation.
 *
 * @param channel_enable A boolean flag indicating if the channel is enabled.
 * @param setup_sel An 8-bit unsigned integer selecting the setup configuration
 * for the channel.
 * @param ain A structure representing the positive and negative analog inputs
 * for the channel.
 ******************************************************************************/
struct ad7124_channel_map {
	bool channel_enable;
	uint8_t setup_sel;
	struct ad7124_analog_inputs ain;
};

/***************************************************************************//**
 * @brief The `ad7124_reference_source` enumeration defines the possible
 * reference sources for the AD7124 ADC device. It includes options for
 * using external references (REFIN1 and REFIN2), an internal 2.5V
 * reference, and a reference derived from the AVDD and AVSS power supply
 * voltages. The enumeration also includes a value to represent the
 * maximum number of reference sources, which can be useful for
 * validation or iteration purposes.
 *
 * @param EXTERNAL_REFIN1 Represents an external reference source using
 * REFIN1+/-.
 * @param EXTERNAL_REFIN2 Represents an external reference source using
 * REFIN2+/-.
 * @param INTERNAL_REF Represents an internal 2.5V reference source.
 * @param AVDD_AVSS Represents a reference source using AVDD - AVSS.
 * @param MAX_REF_SOURCES Indicates the maximum number of reference sources
 * available.
 ******************************************************************************/
enum ad7124_reference_source {
	/* External Reference REFIN1+/-*/
	EXTERNAL_REFIN1,
	/* External Reference REFIN2+/-*/
	EXTERNAL_REFIN2,
	/* Internal 2.5V Reference */
	INTERNAL_REF,
	/* AVDD - AVSS */
	AVDD_AVSS,
	/* Maximum Reference Sources */
	MAX_REF_SOURCES
};

/***************************************************************************//**
 * @brief The `ad7124_channel_setup` structure is used to configure individual
 * channels of the AD7124 ADC. It allows the user to specify whether the
 * channel operates in bipolar or unipolar mode, and whether the
 * reference and analog input buffers are enabled. Additionally, it
 * defines the reference voltage source for the channel, which is crucial
 * for accurate ADC conversions. This setup is essential for tailoring
 * the ADC's behavior to specific application requirements, ensuring
 * optimal performance and accuracy.
 *
 * @param bi_unipolar Indicates if the channel is configured for bipolar or
 * unipolar operation.
 * @param ref_buff Specifies if the reference buffer is enabled for the channel.
 * @param ain_buff Specifies if the analog input buffer is enabled for the
 * channel.
 * @param ref_source Defines the source of the reference voltage for the
 * channel, using the ad7124_reference_source enum.
 ******************************************************************************/
struct ad7124_channel_setup {
	bool bi_unipolar;
	bool ref_buff;
	bool  ain_buff;
	enum ad7124_reference_source ref_source;
};

/***************************************************************************//**
 * @brief The `ad7124_power_mode` enumeration defines the different power modes
 * available for the AD7124 device, which include low, mid, and high
 * power settings. These modes allow the user to configure the device's
 * power consumption and performance characteristics according to the
 * application's requirements.
 *
 * @param AD7124_LOW_POWER Represents the low power mode for the AD7124 device.
 * @param AD7124_MID_POWER Represents the mid power mode for the AD7124 device.
 * @param AD7124_HIGH_POWER Represents the high power mode for the AD7124
 * device.
 ******************************************************************************/
enum ad7124_power_mode {
	AD7124_LOW_POWER,
	AD7124_MID_POWER,
	AD7124_HIGH_POWER
};

/* Device register info */
/***************************************************************************//**
 * @brief The `ad7124_st_reg` structure is used to represent a register in the
 * AD7124 device, encapsulating essential information such as the
 * register's address, its current value, the size of the register, and
 * its read/write permissions. This structure is crucial for managing the
 * configuration and operation of the AD7124 device by allowing the
 * software to interact with the device's registers efficiently.
 *
 * @param addr Stores the address of the register.
 * @param value Holds the value to be written to or read from the register.
 * @param size Indicates the size of the register in bytes.
 * @param rw Specifies the read/write permissions of the register.
 ******************************************************************************/
struct ad7124_st_reg {
	int32_t addr;
	int32_t value;
	int32_t size;
	int32_t rw;
};

/* AD7124 registers list */
/***************************************************************************//**
 * @brief The `ad7124_registers` enumeration defines a comprehensive list of
 * register identifiers for the AD7124 analog-to-digital converter (ADC)
 * device. Each enumerator corresponds to a specific register within the
 * AD7124, facilitating the management and configuration of the device's
 * various functionalities, such as status monitoring, control settings,
 * data handling, input/output configurations, error management, and
 * channel-specific settings. This enumeration is essential for
 * developers working with the AD7124, as it provides a structured way to
 * reference and manipulate the device's registers programmatically.
 *
 * @param AD7124_Status Represents the status register of the AD7124 device.
 * @param AD7124_ADC_Control Represents the ADC control register of the AD7124
 * device.
 * @param AD7124_Data Represents the data register of the AD7124 device.
 * @param AD7124_IOCon1 Represents the first IO control register of the AD7124
 * device.
 * @param AD7124_IOCon2 Represents the second IO control register of the AD7124
 * device.
 * @param AD7124_ID Represents the ID register of the AD7124 device.
 * @param AD7124_Error Represents the error register of the AD7124 device.
 * @param AD7124_Error_En Represents the error enable register of the AD7124
 * device.
 * @param AD7124_Mclk_Count Represents the master clock count register of the
 * AD7124 device.
 * @param AD7124_Channel_0 Represents the configuration for channel 0 of the
 * AD7124 device.
 * @param AD7124_Channel_1 Represents the configuration for channel 1 of the
 * AD7124 device.
 * @param AD7124_Channel_2 Represents the configuration for channel 2 of the
 * AD7124 device.
 * @param AD7124_Channel_3 Represents the configuration for channel 3 of the
 * AD7124 device.
 * @param AD7124_Channel_4 Represents the configuration for channel 4 of the
 * AD7124 device.
 * @param AD7124_Channel_5 Represents the configuration for channel 5 of the
 * AD7124 device.
 * @param AD7124_Channel_6 Represents the configuration for channel 6 of the
 * AD7124 device.
 * @param AD7124_Channel_7 Represents the configuration for channel 7 of the
 * AD7124 device.
 * @param AD7124_Channel_8 Represents the configuration for channel 8 of the
 * AD7124 device.
 * @param AD7124_Channel_9 Represents the configuration for channel 9 of the
 * AD7124 device.
 * @param AD7124_Channel_10 Represents the configuration for channel 10 of the
 * AD7124 device.
 * @param AD7124_Channel_11 Represents the configuration for channel 11 of the
 * AD7124 device.
 * @param AD7124_Channel_12 Represents the configuration for channel 12 of the
 * AD7124 device.
 * @param AD7124_Channel_13 Represents the configuration for channel 13 of the
 * AD7124 device.
 * @param AD7124_Channel_14 Represents the configuration for channel 14 of the
 * AD7124 device.
 * @param AD7124_Channel_15 Represents the configuration for channel 15 of the
 * AD7124 device.
 * @param AD7124_Config_0 Represents the configuration register 0 of the AD7124
 * device.
 * @param AD7124_Config_1 Represents the configuration register 1 of the AD7124
 * device.
 * @param AD7124_Config_2 Represents the configuration register 2 of the AD7124
 * device.
 * @param AD7124_Config_3 Represents the configuration register 3 of the AD7124
 * device.
 * @param AD7124_Config_4 Represents the configuration register 4 of the AD7124
 * device.
 * @param AD7124_Config_5 Represents the configuration register 5 of the AD7124
 * device.
 * @param AD7124_Config_6 Represents the configuration register 6 of the AD7124
 * device.
 * @param AD7124_Config_7 Represents the configuration register 7 of the AD7124
 * device.
 * @param AD7124_Filter_0 Represents the filter register 0 of the AD7124 device.
 * @param AD7124_Filter_1 Represents the filter register 1 of the AD7124 device.
 * @param AD7124_Filter_2 Represents the filter register 2 of the AD7124 device.
 * @param AD7124_Filter_3 Represents the filter register 3 of the AD7124 device.
 * @param AD7124_Filter_4 Represents the filter register 4 of the AD7124 device.
 * @param AD7124_Filter_5 Represents the filter register 5 of the AD7124 device.
 * @param AD7124_Filter_6 Represents the filter register 6 of the AD7124 device.
 * @param AD7124_Filter_7 Represents the filter register 7 of the AD7124 device.
 * @param AD7124_Offset_0 Represents the offset register 0 of the AD7124 device.
 * @param AD7124_Offset_1 Represents the offset register 1 of the AD7124 device.
 * @param AD7124_Offset_2 Represents the offset register 2 of the AD7124 device.
 * @param AD7124_Offset_3 Represents the offset register 3 of the AD7124 device.
 * @param AD7124_Offset_4 Represents the offset register 4 of the AD7124 device.
 * @param AD7124_Offset_5 Represents the offset register 5 of the AD7124 device.
 * @param AD7124_Offset_6 Represents the offset register 6 of the AD7124 device.
 * @param AD7124_Offset_7 Represents the offset register 7 of the AD7124 device.
 * @param AD7124_Gain_0 Represents the gain register 0 of the AD7124 device.
 * @param AD7124_Gain_1 Represents the gain register 1 of the AD7124 device.
 * @param AD7124_Gain_2 Represents the gain register 2 of the AD7124 device.
 * @param AD7124_Gain_3 Represents the gain register 3 of the AD7124 device.
 * @param AD7124_Gain_4 Represents the gain register 4 of the AD7124 device.
 * @param AD7124_Gain_5 Represents the gain register 5 of the AD7124 device.
 * @param AD7124_Gain_6 Represents the gain register 6 of the AD7124 device.
 * @param AD7124_Gain_7 Represents the gain register 7 of the AD7124 device.
 * @param AD7124_REG_NO Represents the total number of registers in the AD7124
 * device.
 ******************************************************************************/
enum ad7124_registers {
	AD7124_Status,
	AD7124_ADC_Control,
	AD7124_Data,
	AD7124_IOCon1,
	AD7124_IOCon2,
	AD7124_ID,
	AD7124_Error,
	AD7124_Error_En,
	AD7124_Mclk_Count,
	AD7124_Channel_0,
	AD7124_Channel_1,
	AD7124_Channel_2,
	AD7124_Channel_3,
	AD7124_Channel_4,
	AD7124_Channel_5,
	AD7124_Channel_6,
	AD7124_Channel_7,
	AD7124_Channel_8,
	AD7124_Channel_9,
	AD7124_Channel_10,
	AD7124_Channel_11,
	AD7124_Channel_12,
	AD7124_Channel_13,
	AD7124_Channel_14,
	AD7124_Channel_15,
	AD7124_Config_0,
	AD7124_Config_1,
	AD7124_Config_2,
	AD7124_Config_3,
	AD7124_Config_4,
	AD7124_Config_5,
	AD7124_Config_6,
	AD7124_Config_7,
	AD7124_Filter_0,
	AD7124_Filter_1,
	AD7124_Filter_2,
	AD7124_Filter_3,
	AD7124_Filter_4,
	AD7124_Filter_5,
	AD7124_Filter_6,
	AD7124_Filter_7,
	AD7124_Offset_0,
	AD7124_Offset_1,
	AD7124_Offset_2,
	AD7124_Offset_3,
	AD7124_Offset_4,
	AD7124_Offset_5,
	AD7124_Offset_6,
	AD7124_Offset_7,
	AD7124_Gain_0,
	AD7124_Gain_1,
	AD7124_Gain_2,
	AD7124_Gain_3,
	AD7124_Gain_4,
	AD7124_Gain_5,
	AD7124_Gain_6,
	AD7124_Gain_7,
	AD7124_REG_NO
};

/***************************************************************************//**
 * @brief The `ad7124_dev` structure is a comprehensive representation of an
 * AD7124 device, encapsulating all necessary configurations and states
 * required for its operation. It includes pointers to SPI communication
 * descriptors and device register settings, as well as flags and
 * counters for managing CRC usage and device readiness. The structure
 * also defines the operational mode, active device type, and power mode,
 * along with arrays for channel setups and mappings, allowing for
 * detailed configuration of the device's analog-to-digital conversion
 * capabilities.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param regs Pointer to the device's register settings.
 * @param use_crc Flag indicating whether CRC is used.
 * @param check_ready Flag indicating whether to check if the device is ready.
 * @param spi_rdy_poll_cnt Counter for SPI ready polling.
 * @param mode Current mode of the AD7124 device.
 * @param active_device Type of the active AD7124 device.
 * @param ref_en Flag indicating if the reference is enabled.
 * @param power_mode Current power mode of the device.
 * @param setups Array of channel setups for the device.
 * @param chan_map Array of channel mappings for the device.
 ******************************************************************************/
struct ad7124_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* Device Settings */
	struct ad7124_st_reg	*regs;
	int16_t use_crc;
	int16_t check_ready;
	int16_t spi_rdy_poll_cnt;
	enum ad7124_mode mode;
	/* Active Device */
	enum ad7124_device_type active_device;
	/* Reference enable */
	bool ref_en;
	/* Power modes */
	enum ad7124_power_mode power_mode;
	/* Setups */
	struct ad7124_channel_setup setups[AD7124_MAX_SETUPS];
	/* Channel Mapping*/
	struct ad7124_channel_map chan_map[AD7124_MAX_CHANNELS];
};

/***************************************************************************//**
 * @brief The `ad7124_init_param` structure is used to initialize and configure
 * the AD7124 device, which is a precision analog-to-digital converter.
 * It includes parameters for SPI communication, device settings such as
 * register configurations, CRC usage, and readiness checks. The
 * structure also specifies the operating mode, active device type,
 * reference enablement, power mode, and configurations for channel
 * setups and mappings, allowing for comprehensive customization of the
 * AD7124's operation.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param regs Pointer to an array of device register settings.
 * @param use_crc Flag to indicate if CRC is used.
 * @param check_ready Flag to indicate if the device readiness should be
 * checked.
 * @param spi_rdy_poll_cnt Counter for SPI ready polling.
 * @param mode Operating mode of the AD7124 device.
 * @param active_device Type of the active AD7124 device.
 * @param ref_en Flag to enable or disable the reference.
 * @param power_mode Power mode setting for the device.
 * @param setups Array of channel setup configurations.
 * @param chan_map Array of channel mappings.
 ******************************************************************************/
struct ad7124_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_init;
	/* Device Settings */
	struct ad7124_st_reg	*regs;
	int16_t use_crc;
	int16_t check_ready;
	int16_t spi_rdy_poll_cnt;
	enum ad7124_mode mode;
	/* Active Device */
	enum ad7124_device_type active_device;
	/* Reference enable */
	bool ref_en;
	/* Power modes */
	enum ad7124_power_mode power_mode;
	/* Setups */
	struct ad7124_channel_setup setups[AD7124_MAX_SETUPS];
	/* Channel Mapping*/
	struct ad7124_channel_map chan_map[AD7124_MAX_CHANNELS];
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Reads the value of the specified register without a device state check. */
/***************************************************************************//**
 * @brief This function is used to read the value of a specified register from
 * an AD7124 device without performing any checks on the device's state.
 * It is useful in scenarios where the device state is already known or
 * managed externally. The function requires a valid device structure and
 * a register structure to store the read value. It handles CRC checks if
 * enabled and manages special cases for reading data registers with
 * status information. The function should be called only when the device
 * is properly initialized and configured.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param p_reg A pointer to an ad7124_st_reg structure representing the
 * register to be read. Must not be null. The structure will be
 * populated with the read value.
 * @return Returns 0 on success, a negative error code on failure (e.g., -EINVAL
 * for invalid parameters, -EBADMSG for CRC errors).
 ******************************************************************************/
int32_t ad7124_no_check_read_register(struct ad7124_dev *dev,
				      struct ad7124_st_reg* p_reg);

/* Writes the value of the specified register without a device state check. */
/***************************************************************************//**
 * @brief This function is used to write a value to a specified register of the
 * AD7124 device without performing any checks on the device's current
 * state. It is useful in scenarios where the user is certain about the
 * device's readiness and wants to perform a write operation directly.
 * The function requires a valid device structure and a register
 * structure containing the address, value, and size of the register to
 * be written. It handles CRC computation if enabled in the device
 * settings. The function returns an error code if the device structure
 * is null.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg An ad7124_st_reg structure containing the register address, value,
 * and size. The address specifies which register to write to, the
 * value is the data to be written, and the size indicates the number
 * of bytes to write.
 * @return Returns an int32_t error code indicating the success or failure of
 * the write operation. A negative value indicates an error, such as
 * -EINVAL if the device structure is null.
 ******************************************************************************/
int32_t ad7124_no_check_write_register(struct ad7124_dev *dev,
				       struct ad7124_st_reg reg);

/* Reads the value of the specified register. */
/***************************************************************************//**
 * @brief This function reads the value of a specified register from an AD7124
 * device. It should be used when you need to retrieve the current value
 * of a register. Before calling this function, ensure that the device is
 * properly initialized and that the `ad7124_dev` structure is correctly
 * configured. The function checks if the device is ready for SPI
 * communication unless the register being accessed is the error
 * register. If the device is not ready, it waits until it becomes ready.
 * This function is useful for obtaining configuration or status
 * information from the device.
 *
 * @param dev A pointer to an `ad7124_dev` structure representing the device.
 * Must not be null. The structure should be properly initialized and
 * configured before use.
 * @param p_reg A pointer to an `ad7124_st_reg` structure that specifies the
 * register to be read. The `addr` field must be set to the address
 * of the register. The function will populate the `value` field
 * with the read data. Must not be null.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad7124_read_register(struct ad7124_dev *dev,
			     struct ad7124_st_reg* p_reg);

/* Wrap the read register function to give it a modern signature. */
/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from an
 * AD7124 device and stores it in the provided output parameter. It is
 * essential to ensure that the device has been properly initialized
 * before calling this function. The function requires a valid device
 * structure and a register index within the valid range. It is useful
 * for obtaining configuration or status information from the device. The
 * function will return an error code if the read operation fails, and
 * the output parameter will not be modified in such cases.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null.
 * @param reg The index of the register to read. Must be within the valid range
 * of register indices for the device.
 * @param readval A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t ad7124_read_register2(struct ad7124_dev *dev,
			      uint32_t reg,
			      uint32_t *readval);

/* Writes the value of the specified register. */
/***************************************************************************//**
 * @brief This function writes a value to a specified register of the AD7124
 * device. It should be used when you need to configure or update the
 * settings of the AD7124 by writing to its registers. Before calling
 * this function, ensure that the device is properly initialized and that
 * the `dev` structure is correctly set up. If the `check_ready` flag in
 * the `dev` structure is set, the function will first wait for the
 * device to be ready for SPI communication. This function is essential
 * for configuring the device's operational parameters and should be used
 * with care to avoid writing incorrect values to the registers.
 *
 * @param dev A pointer to an `ad7124_dev` structure representing the device.
 * Must not be null. The structure should be properly initialized and
 * configured before use.
 * @param p_reg An `ad7124_st_reg` structure containing the register address,
 * value to write, size, and read/write permissions. The address
 * must correspond to a valid register on the AD7124 device.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad7124_write_register(struct ad7124_dev *dev,
			      struct ad7124_st_reg reg);

/* Wrap the write register function to give it a modern signature. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to a register on the
 * AD7124 device. It is essential to ensure that the device has been
 * properly initialized and is ready to accept write operations before
 * calling this function. The function updates the internal register
 * value and then performs the write operation. It is typically used when
 * configuring the device or changing its settings. Proper error handling
 * should be implemented to handle any issues that may arise during the
 * write process.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null, and the device should be ready for
 * communication.
 * @param reg The register address to which the value will be written. Must be a
 * valid register address for the AD7124 device.
 * @param writeval The value to be written to the specified register. Should be
 * within the valid range for the register being accessed.
 * @return Returns an int32_t status code indicating success or failure of the
 * write operation.
 ******************************************************************************/
int32_t ad7124_write_register2(struct ad7124_dev *dev,
			       uint32_t reg,
			       uint32_t writeval);

/* Resets the device. */
/***************************************************************************//**
 * @brief This function is used to reset the AD7124 device, which is necessary
 * to ensure the device is in a known state before starting any
 * operations. It should be called after the device has been initialized
 * and before any configuration or data acquisition is performed. The
 * function disables CRC checking and waits for the device to complete
 * its power-on reset sequence. It is important to ensure that the `dev`
 * parameter is a valid, non-null pointer to an initialized `ad7124_dev`
 * structure. If the device is not ready or the reset fails, an error
 * code is returned.
 *
 * @param dev A pointer to an `ad7124_dev` structure representing the device to
 * be reset. Must not be null. If null, the function returns an error
 * code.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int32_t ad7124_reset(struct ad7124_dev *dev);

/* Waits until the device can accept read and write user actions. */
/***************************************************************************//**
 * @brief This function is used to ensure that the AD7124 device is ready to
 * accept SPI read and write operations. It should be called before
 * attempting any SPI communication with the device to avoid errors. The
 * function checks the device's error register to determine readiness and
 * will return an error if the device is not ready within the specified
 * timeout period. It is important to ensure that the device structure is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param timeout The maximum number of attempts to check the device's
 * readiness. Must be a positive integer. If the timeout is
 * reached without the device being ready, the function returns
 * -ETIMEDOUT.
 * @return Returns 0 if the device is ready for SPI communication. Returns
 * -EINVAL if the dev parameter is null, or -ETIMEDOUT if the timeout is
 * reached without the device being ready. Other negative values may be
 * returned to indicate different errors.
 ******************************************************************************/
int32_t ad7124_wait_for_spi_ready(struct ad7124_dev *dev,
				  uint32_t timeout);

/* Waits until the device finishes the power-on reset operation. */
/***************************************************************************//**
 * @brief This function is used to ensure that the AD7124 device has completed
 * its power-on reset sequence before any further operations are
 * attempted. It should be called after the device is powered on and
 * before any other interactions with the device. The function will
 * repeatedly check the power-on reset flag in the device's status
 * register until it is cleared or the specified timeout period elapses.
 * If the device does not power on within the timeout period, the
 * function will return a timeout error.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param timeout The maximum number of attempts to check the power-on status
 * before timing out. Must be a positive integer.
 * @return Returns 0 if the device powers on successfully within the timeout
 * period. Returns a negative error code if the device pointer is null,
 * if a read error occurs, or if the timeout period elapses without the
 * device powering on.
 ******************************************************************************/
int32_t ad7124_wait_to_power_on(struct ad7124_dev *dev,
				uint32_t timeout);

/* Waits until a new conversion result is available. */
/***************************************************************************//**
 * @brief This function is used to wait until a new conversion result is
 * available from the AD7124 device. It should be called when a
 * conversion is expected, and the caller needs to ensure that the device
 * is ready to provide the result. The function will poll the device's
 * status register until the conversion is ready or the specified timeout
 * is reached. It is important to ensure that the device is properly
 * initialized before calling this function. If the device is not ready
 * within the timeout period, the function will return a timeout error.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param timeout The maximum number of polling attempts to wait for the
 * conversion to be ready. Must be a positive integer. If the
 * timeout is reached without the conversion being ready, the
 * function returns a timeout error.
 * @return Returns 0 if the conversion is ready before the timeout. Returns a
 * negative error code if the device pointer is null, if a read error
 * occurs, or if the timeout is reached.
 ******************************************************************************/
int32_t ad7124_wait_for_conv_ready(struct ad7124_dev *dev,
				   uint32_t timeout);

/* Reads the conversion result from the device. */
/***************************************************************************//**
 * @brief This function retrieves the latest conversion result from the AD7124
 * device and stores it in the provided memory location. It should be
 * called after ensuring that a conversion is ready, typically by using a
 * function like `ad7124_wait_for_conv_ready`. The function requires a
 * valid device structure and a non-null pointer to store the conversion
 * data. If the device structure is null, the function returns an error
 * code.
 *
 * @param dev A pointer to an initialized `ad7124_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @param p_data A pointer to an `int32_t` where the conversion result will be
 * stored. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns 0 on success, or a negative error code if the device
 * structure is null.
 ******************************************************************************/
int32_t ad7124_read_data(struct ad7124_dev *dev,
			 int32_t* p_data);

/* Get the ID of the channel of the latest conversion. */
/***************************************************************************//**
 * @brief Use this function to obtain the ID of the channel that was active
 * during the most recent conversion on the AD7124 device. This function
 * should be called after a conversion has been completed to determine
 * which channel's data is currently available. It requires a valid
 * device structure and a pointer to store the status. The function reads
 * the status register and extracts the active channel ID. Ensure that
 * the device is properly initialized and configured before calling this
 * function.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param status A pointer to a uint32_t variable where the channel ID will be
 * stored. Must not be null. The function will write the active
 * channel ID to this location.
 * @return Returns 0 on success, or a negative error code if the register read
 * fails.
 ******************************************************************************/
int32_t ad7124_get_read_chan_id(struct ad7124_dev *dev, uint32_t *status);

/* Computes the CRC checksum for a data buffer. */
/***************************************************************************//**
 * @brief This function calculates the CRC-8 checksum for a given data buffer
 * using a predefined polynomial representation. It is typically used to
 * verify data integrity in communication protocols. The function
 * requires a pointer to the data buffer and the size of the buffer as
 * inputs. It is important to ensure that the buffer pointer is not null
 * and that the buffer size is correctly specified to avoid undefined
 * behavior.
 *
 * @param p_buf Pointer to the data buffer for which the CRC-8 checksum is to be
 * computed. Must not be null, and the caller retains ownership of
 * the buffer.
 * @param buf_size The size of the data buffer in bytes. Must be a non-zero
 * value to perform the computation.
 * @return Returns the computed CRC-8 checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t ad7124_compute_crc8(uint8_t* p_buf,
			    uint8_t buf_size);

/* Computes the XOR checksum for a data buffer. */
uint8_t AD7124_ComputeXOR8(uint8_t * p_buf,
			   uint8_t buf_size);

/* Updates the CRC settings. */
/***************************************************************************//**
 * @brief This function checks the current configuration of the AD7124 device to
 * determine whether CRC error checking is enabled. It should be called
 * whenever there is a need to update or verify the CRC setting of the
 * device, particularly after changes to the device's error enable
 * register. The function requires a valid device structure and will not
 * perform any action if the provided device pointer is null.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null. The function will return immediately if this
 * parameter is null.
 * @return None
 ******************************************************************************/
void ad7124_update_crcsetting(struct ad7124_dev *dev);

/* Updates the device SPI interface settings. */
/***************************************************************************//**
 * @brief This function adjusts the SPI settings of an AD7124 device by checking
 * the error enable register to determine if SPI ignore errors are
 * enabled. It should be called whenever there is a need to update the
 * device's SPI settings, particularly after modifying the error enable
 * register. The function requires a valid device structure and will not
 * perform any action if the provided device pointer is null.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null. If null, the function returns immediately without
 * making any changes.
 * @return None
 ******************************************************************************/
void ad7124_update_dev_spi_settings(struct ad7124_dev *dev);

/* Get the AD7124 reference clock. */
/***************************************************************************//**
 * @brief This function retrieves the reference clock frequency of the AD7124
 * device based on its current power mode. It should be called when the
 * user needs to know the clock frequency for further calculations or
 * configurations. The function requires a valid device structure and a
 * pointer to store the clock frequency. It returns an error code if the
 * device register cannot be read or if the power mode is invalid.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null.
 * @param f_clk A pointer to a float where the clock frequency will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the device register
 * cannot be read or if the power mode is invalid.
 ******************************************************************************/
int32_t ad7124_fclk_get(struct ad7124_dev *dev, float *f_clk);

/* Get the filter coefficient for the sample rate. */
/***************************************************************************//**
 * @brief This function retrieves the filter coefficient for a specified channel
 * on the AD7124 device. It should be called when you need to determine
 * the filter settings for a particular channel, which can affect the
 * sample rate and data processing. The function requires a valid device
 * structure and a channel number within the supported range. The filter
 * coefficient is returned through a pointer parameter. Ensure that the
 * device is properly initialized before calling this function, and
 * handle any error codes returned to manage potential read failures.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null.
 * @param chn_num The channel number for which the filter coefficient is to be
 * retrieved. Must be a valid channel number within the device's
 * supported range.
 * @param flt_coff A pointer to a uint16_t where the filter coefficient will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7124_fltcoff_get(struct ad7124_dev *dev,
			   int16_t chn_no,
			   uint16_t *flt_coff);

/* Calculate ODR of the device. */
/***************************************************************************//**
 * @brief Use this function to determine the output data rate (ODR) of a
 * specific channel on the AD7124 device. This function should be called
 * after the device has been properly initialized and configured. It
 * reads the necessary registers to compute the ODR based on the filter
 * settings and clock frequency. The function handles error conditions by
 * returning negative values, which should be checked by the caller to
 * ensure successful execution.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null.
 * @param chn_num The channel number for which to calculate the ODR. Valid range
 * is from 0 to AD7124_MAX_CHANNELS - 1. Invalid channel numbers
 * may result in undefined behavior.
 * @return Returns the calculated ODR as a float. If an error occurs, a negative
 * value is returned indicating the error.
 ******************************************************************************/
float ad7124_get_odr(struct ad7124_dev *dev, int16_t ch_no);

/* Set ODR of the device. */
/***************************************************************************//**
 * @brief This function configures the output data rate (ODR) for a specific
 * channel on the AD7124 device. It should be called when you need to
 * adjust the sampling rate of a channel to match your application's
 * requirements. The function requires a valid device structure and a
 * channel number within the supported range. It calculates and sets the
 * appropriate filter settings to achieve the desired ODR, clamping the
 * filter settings to a minimum of 1 and a maximum of 2047 to ensure
 * valid operation. The function must be called after the device has been
 * properly initialized.
 *
 * @param dev A pointer to an initialized ad7124_dev structure representing the
 * device. Must not be null.
 * @param odr The desired output data rate in Hz. Must be a positive floating-
 * point number.
 * @param chn_num The channel number for which to set the ODR. Must be a valid
 * channel number within the device's supported range.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad7124_set_odr(struct ad7124_dev *dev,
		       float odr,
		       int16_t chn_no);

/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function allows for writing specific bits to a register on an
 * AD7124 device by applying a mask. It first reads the current value of
 * the register, applies the mask to clear the bits to be modified, and
 * then writes the new data. This is useful for modifying only certain
 * bits of a register without affecting other bits. The function should
 * be called when the device is ready for communication, and the register
 * address must be valid for the AD7124 device. It returns an error code
 * if the read or write operation fails.
 *
 * @param dev A pointer to an ad7124_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function.
 * @param reg_addr The address of the register to be written to. Must be a valid
 * register address for the AD7124 device.
 * @param data The data to be written to the register. Only the bits specified
 * by the mask will be affected.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected by the data
 * parameter.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad7124_reg_write_msk(struct ad7124_dev *dev,
			 uint32_t reg_addr,
			 uint32_t data,
			 uint32_t mask);

/* Set ADC Mode */
/***************************************************************************//**
 * @brief Use this function to configure the AD7124 device to operate in a
 * specific ADC mode. This function should be called after the device has
 * been initialized and before starting any ADC operations that depend on
 * the mode setting. The function validates the input parameters and
 * returns an error if the device pointer is null or if the specified
 * mode is invalid. It updates the device's mode setting and writes the
 * new mode to the ADC control register.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null. The function will return an error
 * if this parameter is null.
 * @param adc_mode An enum value of type ad7124_mode representing the desired
 * ADC mode. Must be a valid mode less than ADC_MAX_MODES. If an
 * invalid mode is provided, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null, the mode is invalid, or if there is a failure in writing to
 * the device register.
 ******************************************************************************/
int ad7124_set_adc_mode(struct ad7124_dev *device, enum ad7124_mode mode);

/* Enable/Disable Channels */
/***************************************************************************//**
 * @brief Use this function to enable or disable a specific channel on the
 * AD7124 device. This function should be called when you need to change
 * the operational status of a channel, such as during initialization or
 * when reconfiguring the device. Ensure that the device structure is
 * properly initialized before calling this function. The function
 * updates both the device's register and its internal channel map to
 * reflect the new status. It returns an error code if the operation
 * fails, which should be checked to ensure the channel status was set
 * successfully.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null.
 * @param chn_num The channel number to be modified. Valid range is 0 to
 * AD7124_MAX_CHANNELS - 1.
 * @param channel_status A boolean value indicating the desired status of the
 * channel. 'true' to enable the channel, 'false' to
 * disable it.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad7124_set_channel_status(struct ad7124_dev *device,
			      uint8_t chn_no,
			      bool channel_status);

/* Configure Analog inputs to channel */
/***************************************************************************//**
 * @brief This function is used to configure a specific channel on the AD7124
 * device to use designated positive and negative analog inputs. It
 * should be called after the device has been initialized and before
 * starting any data acquisition. The function updates the channel
 * mapping with the specified inputs and writes the configuration to the
 * device registers. It is important to ensure that the channel number is
 * within the valid range and that the device pointer is not null. The
 * function returns an error code if the configuration fails.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null.
 * @param chn_num The channel number to configure, ranging from 0 to
 * AD7124_MAX_CHANNELS - 1. Invalid values may result in
 * undefined behavior.
 * @param analog_input A structure of type ad7124_analog_inputs specifying the
 * positive and negative analog inputs to connect. The
 * inputs must be valid enum values from
 * ad7124_analog_input.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad7124_connect_analog_input(struct ad7124_dev *device,
				uint8_t chn_no,
				struct ad7124_analog_inputs analog_input);

/* Assign setup to channel */
/***************************************************************************//**
 * @brief This function is used to assign a specific setup configuration to a
 * channel on the AD7124 device, which is necessary for configuring the
 * channel's operation. It should be called after the device has been
 * initialized and before starting any data acquisition. The function
 * modifies the channel's setup selection in the device's channel map. It
 * is important to ensure that the channel number and setup index are
 * within valid ranges to avoid errors.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null.
 * @param chn_num The channel number to which the setup is to be assigned. Valid
 * range is 0 to AD7124_MAX_CHANNELS - 1.
 * @param setup The setup index to assign to the channel. Valid range is 0 to
 * AD7124_MAX_SETUPS - 1.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad7124_assign_setup(struct ad7124_dev *device,
			uint8_t ch_no,
			uint8_t setup);

/* Assign polarity to setup */
/***************************************************************************//**
 * @brief Use this function to configure the polarity of a specific setup on the
 * AD7124 device, allowing for either bipolar or unipolar operation. This
 * function should be called when you need to change the input signal
 * range for a given setup. Ensure that the device is properly
 * initialized before calling this function. The function updates both
 * the device's register and its internal state to reflect the new
 * polarity setting.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null.
 * @param bipolar A boolean value indicating the desired polarity mode. Set to
 * true for bipolar mode, false for unipolar mode.
 * @param setup_id An unsigned 8-bit integer representing the setup ID to
 * configure. Must be within the range of available setups for
 * the device.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad7124_set_polarity(struct ad7124_dev* device,
			bool bipolar,
			uint8_t setup_id);

/* Assign reference source to setup */
/***************************************************************************//**
 * @brief This function configures the reference source for a specific setup in
 * the AD7124 device, allowing the user to select between different
 * reference sources such as internal or external references. It must be
 * called with a valid device pointer and a setup ID within the allowed
 * range. The function also enables or disables the reference based on
 * the provided flag. It returns an error code if the device pointer is
 * null or if the reference source is invalid.
 *
 * @param device A pointer to an ad7124_dev structure representing the device.
 * Must not be null.
 * @param ref_source An enum value of type ad7124_reference_source specifying
 * the reference source. Must be less than MAX_REF_SOURCES.
 * @param setup_id An unsigned 8-bit integer representing the setup ID. Must be
 * within the range of available setups.
 * @param ref_en A boolean flag indicating whether to enable (true) or disable
 * (false) the reference.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int ad7124_set_reference_source(struct ad7124_dev* device,
				enum ad7124_reference_source ref_source,
				uint8_t setup_id,
				bool ref_en);

/* Enable/Disable input and reference buffers to setup */
/***************************************************************************//**
 * @brief This function configures the input and reference buffers for a
 * specific setup on the AD7124 device. It should be called when you need
 * to enable or disable these buffers for a particular setup, identified
 * by the setup ID. The function requires a valid device structure and a
 * setup ID within the valid range. It returns an error code if the
 * operation fails, otherwise it updates the device's setup configuration
 * to reflect the new buffer settings.
 *
 * @param device A pointer to an ad7124_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param inbuf_en A boolean value indicating whether to enable (true) or
 * disable (false) the input buffer for the specified setup.
 * @param refbuf_en A boolean value indicating whether to enable (true) or
 * disable (false) the reference buffer for the specified
 * setup.
 * @param setup_id An unsigned 8-bit integer representing the setup ID. Must be
 * within the range of available setups for the device.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad7124_enable_buffers(struct ad7124_dev* device,
			  bool ain_buff,
			  bool ref_buff,
			  uint8_t setup_id);

/* Select the power mode */
/***************************************************************************//**
 * @brief This function configures the power mode of the specified AD7124
 * device, allowing the user to optimize power consumption based on
 * application needs. It should be called when a change in power mode is
 * required, such as switching between low power and high performance
 * modes. The function must be called with a valid device structure that
 * has been properly initialized. It updates the device's internal state
 * to reflect the new power mode. If the operation fails, an error code
 * is returned.
 *
 * @param device A pointer to an initialized ad7124_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type ad7124_power_mode indicating the desired
 * power mode. Valid values are AD7124_LOW_POWER, AD7124_MID_POWER,
 * and AD7124_HIGH_POWER.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad7124_set_power_mode(struct ad7124_dev *device,
			  enum ad7124_power_mode mode);

/* Initializes the AD7124 */
/***************************************************************************//**
 * @brief This function sets up the AD7124 device by initializing its SPI
 * communication, configuring its registers, and setting up channels and
 * modes as specified in the initialization parameters. It must be called
 * before any other operations on the AD7124 device to ensure proper
 * configuration. The function allocates memory for the device structure
 * and configures the device based on the provided initialization
 * parameters. If any step in the setup process fails, the function will
 * return an error code and ensure that any allocated resources are
 * properly freed.
 *
 * @param device A pointer to a pointer of type `struct ad7124_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct ad7124_init_param` containing the
 * initialization parameters for the device. This includes SPI
 * settings, device type, power modes, and channel
 * configurations. The pointer must not be null, and the
 * structure must be fully populated with valid data.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered during setup.
 ******************************************************************************/
int32_t ad7124_setup(struct ad7124_dev **device,
		     struct ad7124_init_param *init_param);

/* Free the resources allocated by ad7124_setup(). */
/***************************************************************************//**
 * @brief This function is used to clean up and release resources associated
 * with an AD7124 device instance. It should be called when the device is
 * no longer needed, typically at the end of its usage lifecycle. The
 * function ensures that any allocated memory and associated SPI
 * resources are properly freed. It is important to ensure that the
 * device pointer provided is valid and was previously initialized using
 * ad7124_setup().
 *
 * @param dev A pointer to an ad7124_dev structure representing the device
 * instance to be removed. Must not be null and should point to a
 * valid, initialized device structure. If the pointer is invalid,
 * the behavior is undefined.
 * @return Returns 0 on success, or a negative error code if the SPI removal
 * fails.
 ******************************************************************************/
int32_t ad7124_remove(struct ad7124_dev *dev);

#endif /* __AD7124_H__ */


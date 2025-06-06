/***************************************************************************//**
 *   @file   iio_adis_internals.h
 *   @brief  Internal include file used for specific chip IIO driver definitions
 *           which are not useful for the user.
 *   @author RBolboac (ramona.gradinariu@analog.com)
 *******************************************************************************
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
 ******************************************************************************/

#ifndef IIO_ADIS_INTERNALS_H
#define IIO_ADIS_INTERNALS_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio.h"
#include <errno.h>

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adis_iio_chan_type` enumeration defines various channel types for
 * an ADIS IIO (Industrial Input/Output) device, including gyroscope,
 * accelerometer, temperature, delta angle, and delta velocity channels
 * across different axes. This enumeration is used to identify and manage
 * the different types of data channels available in the ADIS device,
 * facilitating the configuration and data acquisition processes in
 * applications that utilize these sensors.
 *
 * @param ADIS_GYRO_X Represents the X-axis gyroscope channel.
 * @param ADIS_GYRO_Y Represents the Y-axis gyroscope channel.
 * @param ADIS_GYRO_Z Represents the Z-axis gyroscope channel.
 * @param ADIS_ACCEL_X Represents the X-axis accelerometer channel.
 * @param ADIS_ACCEL_Y Represents the Y-axis accelerometer channel.
 * @param ADIS_ACCEL_Z Represents the Z-axis accelerometer channel.
 * @param ADIS_TEMP Represents the temperature channel.
 * @param ADIS_DELTA_ANGL_X Represents the X-axis delta angle channel.
 * @param ADIS_DELTA_ANGL_Y Represents the Y-axis delta angle channel.
 * @param ADIS_DELTA_ANGL_Z Represents the Z-axis delta angle channel.
 * @param ADIS_DELTA_VEL_X Represents the X-axis delta velocity channel.
 * @param ADIS_DELTA_VEL_Y Represents the Y-axis delta velocity channel.
 * @param ADIS_DELTA_VEL_Z Represents the Z-axis delta velocity channel.
 * @param ADIS_NUM_CHAN Represents the total number of channels.
 ******************************************************************************/
enum adis_iio_chan_type {
	ADIS_GYRO_X,
	ADIS_GYRO_Y,
	ADIS_GYRO_Z,
	ADIS_ACCEL_X,
	ADIS_ACCEL_Y,
	ADIS_ACCEL_Z,
	ADIS_TEMP,
	ADIS_DELTA_ANGL_X,
	ADIS_DELTA_ANGL_Y,
	ADIS_DELTA_ANGL_Z,
	ADIS_DELTA_VEL_X,
	ADIS_DELTA_VEL_Y,
	ADIS_DELTA_VEL_Z,
	ADIS_NUM_CHAN,
};

/***************************************************************************//**
 * @brief The `adis_iio_debug_attrs` is an enumeration that defines a
 * comprehensive set of diagnostic and configuration attributes for ADIS
 * IIO devices. These attributes cover a wide range of diagnostics,
 * including sensor initialization failures, data path overruns, memory
 * update failures, and various sensor-specific errors. Additionally, it
 * includes attributes for configuration settings such as FIFO control,
 * filter settings, bias correction, and command operations like self-
 * tests and memory updates. This enumeration is crucial for monitoring
 * and configuring the operational status and performance of ADIS
 * devices, providing detailed insights into potential issues and
 * allowing for precise control over device settings.
 *
 * @param ADIS_DIAG_SNSR_INIT_FAILURE Indicates a sensor initialization failure
 * diagnostic.
 * @param ADIS_DIAG_DATA_PATH_OVERRUN Indicates a data path overrun diagnostic.
 * @param ADIS_DIAG_FLS_MEM_UPDATE_FAILURE Indicates a flash memory update
 * failure diagnostic.
 * @param ADIS_DIAG_SPI_COMM_ERR Indicates an SPI communication error
 * diagnostic.
 * @param ADIS_DIAG_STANDBY_MODE Indicates the device is in standby mode.
 * @param ADIS_DIAG_SNSR_FAILURE Indicates a general sensor failure diagnostic.
 * @param ADIS_DIAG_MEM_FAILURE Indicates a memory failure diagnostic.
 * @param ADIS_DIAG_CLK_ERR Indicates a clock error diagnostic.
 * @param ADIS_DIAG_GYRO1_FAILURE Indicates a failure in the first gyroscope.
 * @param ADIS_DIAG_GYRO2_FAILURE Indicates a failure in the second gyroscope.
 * @param ADIS_DIAG_ACCL_FAILURE Indicates an accelerometer failure diagnostic.
 * @param ADIS_DIAG_X_AXIS_GYRO_FAILURE Indicates a failure in the X-axis
 * gyroscope.
 * @param ADIS_DIAG_Y_AXIS_GYRO_FAILURE Indicates a failure in the Y-axis
 * gyroscope.
 * @param ADIS_DIAG_Z_AXIS_GYRO_FAILURE Indicates a failure in the Z-axis
 * gyroscope.
 * @param ADIS_DIAG_X_AXIS_ACCL_FAILURE Indicates a failure in the X-axis
 * accelerometer.
 * @param ADIS_DIAG_Y_AXIS_ACCL_FAILURE Indicates a failure in the Y-axis
 * accelerometer.
 * @param ADIS_DIAG_Z_AXIS_ACCL_FAILURE Indicates a failure in the Z-axis
 * accelerometer.
 * @param ADIS_DIAG_ADUC_MCU_FAULT Indicates a fault in the ADuC microcontroller
 * unit.
 * @param ADIS_DIAG_CONFIG_CALIB_CRC_ERR Indicates a configuration calibration
 * CRC error.
 * @param ADIS_DIAG_OVERRANGE Indicates an overrange condition diagnostic.
 * @param ADIS_DIAG_TEMP_ERR Indicates a temperature error diagnostic.
 * @param ADIS_DIAG_POWER_SUPPLY_FAILURE Indicates a power supply failure
 * diagnostic.
 * @param ADIS_DIAG_BOOT_MEMORY_FAILURE Indicates a boot memory failure
 * diagnostic.
 * @param ADIS_DIAG_REG_NVM_ERR Indicates a non-volatile memory register error.
 * @param ADIS_DIAG_WDG_TIMER_FLAG Indicates a watchdog timer flag diagnostic.
 * @param ADIS_DIAG_INT_PROC_SUPPLY_ERR Indicates an internal processor supply
 * error.
 * @param ADIS_DIAG_EXT_5V_SUPPLY_ERR Indicates an external 5V supply error.
 * @param ADIS_DIAG_INT_SNSR_SUPPLY_ERR Indicates an internal sensor supply
 * error.
 * @param ADIS_DIAG_INT_REG_ERR Indicates an internal register error.
 * @param ADIS_DIAG_CHECKSUM_ERR Indicates a checksum error diagnostic.
 * @param ADIS_DIAG_FLS_MEM_WR_CNT_EXCEED Indicates a flash memory write count
 * exceedance.
 * @param ADIS_DIAG_LOST_SAMPLES_COUNT Indicates a count of lost samples.
 * @param ADIS_TIME_STAMP Represents a timestamp attribute.
 * @param ADIS_DATA_CNTR Represents a data counter attribute.
 * @param ADIS_FIFO_CNT Represents a FIFO count attribute.
 * @param ADIS_SPI_CHKSUM Represents an SPI checksum attribute.
 * @param ADIS_FIFO_EN Indicates FIFO enable status.
 * @param ADIS_FIFO_OVERFLOW Indicates a FIFO overflow condition.
 * @param ADIS_FIFO_WM_INT_EN Indicates FIFO watermark interrupt enable status.
 * @param ADIS_FIFO_WM_INT_POL Indicates FIFO watermark interrupt polarity.
 * @param ADIS_FIFO_WM_LVL Indicates FIFO watermark level.
 * @param ADIS_FILT_SIZE_VAR_B Indicates filter size variable B.
 * @param ADIS_GYRO_MEAS_RANGE Indicates gyroscope measurement range.
 * @param ADIS_DR_ENABLE Indicates data ready enable status.
 * @param ADIS_DR_POLARITY Indicates data ready polarity.
 * @param ADIS_DR_LINE_SEL Indicates data ready line selection.
 * @param ADIS_SYNC_POLARITY Indicates sync polarity.
 * @param ADIS_SYNC_LINE_SEL Indicates sync line selection.
 * @param ADIS_SYNC_MODE Indicates sync mode.
 * @param ADIS_ALARM_IND_ENABLE Indicates alarm indicator enable status.
 * @param ADIS_ALARM_IND_POLARITY Indicates alarm indicator polarity.
 * @param ADIS_ALARM_IND_LINE_SEL Indicates alarm indicator line selection.
 * @param ADIS_DIO_1_DIR Indicates direction of DIO 1.
 * @param ADIS_DIO_2_DIR Indicates direction of DIO 2.
 * @param ADIS_DIO_3_DIR Indicates direction of DIO 3.
 * @param ADIS_DIO_4_DIR Indicates direction of DIO 4.
 * @param ADIS_DIO_1_LVL Indicates level of DIO 1.
 * @param ADIS_DIO_2_LVL Indicates level of DIO 2.
 * @param ADIS_DIO_3_LVL Indicates level of DIO 3.
 * @param ADIS_DIO_4_LVL Indicates level of DIO 4.
 * @param ADIS_SENS_BW Indicates sensor bandwidth.
 * @param ADIS_PT_OF_PERC_ALGNMT Indicates point of percentage alignment.
 * @param ADIS_LINEAR_ACCL_COMP Indicates linear acceleration compensation.
 * @param ADIS_BURST_SEL Indicates burst selection.
 * @param ADIS_BURST32 Indicates 32-bit burst mode.
 * @param ADIS_TIMESTAMP32 Indicates 32-bit timestamp mode.
 * @param ADIS_SYNC_4KHZ Indicates 4kHz sync mode.
 * @param ADIS_GYRO_FIR_ENABLE Indicates gyroscope FIR filter enable status.
 * @param ADIS_ACCL_FIR_ENABLE Indicates accelerometer FIR filter enable status.
 * @param ADIS_FIR_EN_XG Indicates FIR filter enable for X-axis gyroscope.
 * @param ADIS_FIR_EN_YG Indicates FIR filter enable for Y-axis gyroscope.
 * @param ADIS_FIR_EN_ZG Indicates FIR filter enable for Z-axis gyroscope.
 * @param ADIS_FIR_EN_XA Indicates FIR filter enable for X-axis accelerometer.
 * @param ADIS_FIR_EN_YA Indicates FIR filter enable for Y-axis accelerometer.
 * @param ADIS_FIR_EN_ZA Indicates FIR filter enable for Z-axis accelerometer.
 * @param ADIS_FIR_BANK_SEL_XG Indicates FIR bank selection for X-axis
 * gyroscope.
 * @param ADIS_FIR_BANK_SEL_YG Indicates FIR bank selection for Y-axis
 * gyroscope.
 * @param ADIS_FIR_BANK_SEL_ZG Indicates FIR bank selection for Z-axis
 * gyroscope.
 * @param ADIS_FIR_BANK_SEL_XA Indicates FIR bank selection for X-axis
 * accelerometer.
 * @param ADIS_FIR_BANK_SEL_YA Indicates FIR bank selection for Y-axis
 * accelerometer.
 * @param ADIS_FIR_BANK_SEL_ZA Indicates FIR bank selection for Z-axis
 * accelerometer.
 * @param ADIS_UP_SCALE Indicates up scale factor.
 * @param ADIS_DEC_RATE Indicates decimation rate.
 * @param ADIS_BIAS_CORR_TBC Indicates bias correction time base control.
 * @param ADIS_BIAS_CORR_EN_XG Indicates bias correction enable for X-axis
 * gyroscope.
 * @param ADIS_BIAS_CORR_EN_YG Indicates bias correction enable for Y-axis
 * gyroscope.
 * @param ADIS_BIAS_CORR_EN_ZG Indicates bias correction enable for Z-axis
 * gyroscope.
 * @param ADIS_BIAS_CORR_EN_XA Indicates bias correction enable for X-axis
 * accelerometer.
 * @param ADIS_BIAS_CORR_EN_YA Indicates bias correction enable for Y-axis
 * accelerometer.
 * @param ADIS_BIAS_CORR_EN_ZA Indicates bias correction enable for Z-axis
 * accelerometer.
 * @param ADIS_CMD_BIAS_CORR_UPDATE Indicates command for bias correction
 * update.
 * @param ADIS_CMD_FACT_CALIB_RESTORE Indicates command for factory calibration
 * restore.
 * @param ADIS_CMD_SNSR_SELF_TEST Indicates command for sensor self-test.
 * @param ADIS_CMD_FLS_MEM_UPDATE Indicates command for flash memory update.
 * @param ADIS_CMD_FLS_MEM_TEST Indicates command for flash memory test.
 * @param ADIS_CMD_FIFO_FLUSH Indicates command for FIFO flush.
 * @param ADIS_CMD_SW_RES Indicates command for software reset.
 * @param ADIS_CMD_WRITE_LOCK Indicates command for write lock.
 * @param ADIS_PROC_REV Indicates processor revision.
 * @param ADIS_BOOT_REV Indicates boot revision.
 * @param ADIS_FIRM_REV Indicates firmware revision.
 * @param ADIS_FIRM_DATE Indicates firmware date.
 * @param ADIS_PROD_ID Indicates product ID.
 * @param ADIS_SERIAL_NUM Indicates serial number.
 * @param ADIS_LOT_NUM Indicates lot number.
 * @param ADIS_USR_SCR_1 Indicates user scratchpad 1.
 * @param ADIS_USR_SCR_2 Indicates user scratchpad 2.
 * @param ADIS_USR_SCR_3 Indicates user scratchpad 3.
 * @param ADIS_USR_SCR_4 Indicates user scratchpad 4.
 * @param ADIS_FLS_MEM_WR_CNTR Indicates flash memory write counter.
 * @param ADIS_EXT_CLK_FREQ Indicates external clock frequency.
 ******************************************************************************/
enum adis_iio_debug_attrs {
	ADIS_DIAG_SNSR_INIT_FAILURE,
	ADIS_DIAG_DATA_PATH_OVERRUN,
	ADIS_DIAG_FLS_MEM_UPDATE_FAILURE,
	ADIS_DIAG_SPI_COMM_ERR,
	ADIS_DIAG_STANDBY_MODE,
	ADIS_DIAG_SNSR_FAILURE,
	ADIS_DIAG_MEM_FAILURE,
	ADIS_DIAG_CLK_ERR,
	ADIS_DIAG_GYRO1_FAILURE,
	ADIS_DIAG_GYRO2_FAILURE,
	ADIS_DIAG_ACCL_FAILURE,
	ADIS_DIAG_X_AXIS_GYRO_FAILURE,
	ADIS_DIAG_Y_AXIS_GYRO_FAILURE,
	ADIS_DIAG_Z_AXIS_GYRO_FAILURE,
	ADIS_DIAG_X_AXIS_ACCL_FAILURE,
	ADIS_DIAG_Y_AXIS_ACCL_FAILURE,
	ADIS_DIAG_Z_AXIS_ACCL_FAILURE,
	ADIS_DIAG_ADUC_MCU_FAULT,
	ADIS_DIAG_CONFIG_CALIB_CRC_ERR,
	ADIS_DIAG_OVERRANGE,
	ADIS_DIAG_TEMP_ERR,
	ADIS_DIAG_POWER_SUPPLY_FAILURE,
	ADIS_DIAG_BOOT_MEMORY_FAILURE,
	ADIS_DIAG_REG_NVM_ERR,
	ADIS_DIAG_WDG_TIMER_FLAG,
	ADIS_DIAG_INT_PROC_SUPPLY_ERR,
	ADIS_DIAG_EXT_5V_SUPPLY_ERR,
	ADIS_DIAG_INT_SNSR_SUPPLY_ERR,
	ADIS_DIAG_INT_REG_ERR,
	ADIS_DIAG_CHECKSUM_ERR,
	ADIS_DIAG_FLS_MEM_WR_CNT_EXCEED,
	ADIS_DIAG_LOST_SAMPLES_COUNT,

	ADIS_TIME_STAMP,
	ADIS_DATA_CNTR,

	ADIS_FIFO_CNT,
	ADIS_SPI_CHKSUM,

	ADIS_FIFO_EN,
	ADIS_FIFO_OVERFLOW,
	ADIS_FIFO_WM_INT_EN,
	ADIS_FIFO_WM_INT_POL,
	ADIS_FIFO_WM_LVL,

	ADIS_FILT_SIZE_VAR_B,
	ADIS_GYRO_MEAS_RANGE,

	ADIS_DR_ENABLE,
	ADIS_DR_POLARITY,
	ADIS_DR_LINE_SEL,
	ADIS_SYNC_POLARITY,
	ADIS_SYNC_LINE_SEL,
	ADIS_SYNC_MODE,
	ADIS_ALARM_IND_ENABLE,
	ADIS_ALARM_IND_POLARITY,
	ADIS_ALARM_IND_LINE_SEL,
	ADIS_DIO_1_DIR,
	ADIS_DIO_2_DIR,
	ADIS_DIO_3_DIR,
	ADIS_DIO_4_DIR,
	ADIS_DIO_1_LVL,
	ADIS_DIO_2_LVL,
	ADIS_DIO_3_LVL,
	ADIS_DIO_4_LVL,
	ADIS_SENS_BW,
	ADIS_PT_OF_PERC_ALGNMT,
	ADIS_LINEAR_ACCL_COMP,
	ADIS_BURST_SEL,
	ADIS_BURST32,
	ADIS_TIMESTAMP32,
	ADIS_SYNC_4KHZ,
	ADIS_GYRO_FIR_ENABLE,
	ADIS_ACCL_FIR_ENABLE,
	ADIS_FIR_EN_XG,
	ADIS_FIR_EN_YG,
	ADIS_FIR_EN_ZG,
	ADIS_FIR_EN_XA,
	ADIS_FIR_EN_YA,
	ADIS_FIR_EN_ZA,
	ADIS_FIR_BANK_SEL_XG,
	ADIS_FIR_BANK_SEL_YG,
	ADIS_FIR_BANK_SEL_ZG,
	ADIS_FIR_BANK_SEL_XA,
	ADIS_FIR_BANK_SEL_YA,
	ADIS_FIR_BANK_SEL_ZA,

	ADIS_UP_SCALE,
	ADIS_DEC_RATE,

	ADIS_BIAS_CORR_TBC,
	ADIS_BIAS_CORR_EN_XG,
	ADIS_BIAS_CORR_EN_YG,
	ADIS_BIAS_CORR_EN_ZG,
	ADIS_BIAS_CORR_EN_XA,
	ADIS_BIAS_CORR_EN_YA,
	ADIS_BIAS_CORR_EN_ZA,

	ADIS_CMD_BIAS_CORR_UPDATE,
	ADIS_CMD_FACT_CALIB_RESTORE,
	ADIS_CMD_SNSR_SELF_TEST,
	ADIS_CMD_FLS_MEM_UPDATE,
	ADIS_CMD_FLS_MEM_TEST,
	ADIS_CMD_FIFO_FLUSH,
	ADIS_CMD_SW_RES,
	ADIS_CMD_WRITE_LOCK,

	ADIS_PROC_REV,
	ADIS_BOOT_REV,
	ADIS_FIRM_REV,
	ADIS_FIRM_DATE,
	ADIS_PROD_ID,
	ADIS_SERIAL_NUM,
	ADIS_LOT_NUM,
	ADIS_USR_SCR_1,
	ADIS_USR_SCR_2,
	ADIS_USR_SCR_3,
	ADIS_USR_SCR_4,
	ADIS_FLS_MEM_WR_CNTR,
	ADIS_EXT_CLK_FREQ,
};

/***************************************************************************//**
 * @brief The `adis_iio_scale_fractional` structure is used to represent a
 * fractional scale format in the ADIS IIO driver, where the scale is
 * defined as the ratio of a dividend to a divisor. This structure is
 * useful for representing scaling factors in a fractional format, which
 * can be applied to sensor data to convert raw values into meaningful
 * units.
 *
 * @param dividend Scale dividend.
 * @param divisor Scale divisor.
 ******************************************************************************/
struct adis_iio_scale_fractional {
	/** Scale dividend. */
	uint32_t dividend;
	/** Scale divisor. */
	uint32_t divisor;
};

/***************************************************************************//**
 * @brief The `adis_iio_scale_fractional_log2` structure is used to represent a
 * scale in a fractional log2 format, where the scale is calculated as
 * the dividend divided by 2 raised to the power of the `power` member.
 * This structure is part of the ADIS IIO driver and is used to handle
 * scaling operations in a logarithmic base-2 format, which can be useful
 * for efficient computation in certain applications.
 *
 * @param dividend Scale dividend.
 * @param power Scale 2's power.
 ******************************************************************************/
struct adis_iio_scale_fractional_log2 {
	/** Scale dividend. */
	uint32_t dividend;
	/** Scale 2's power. */
	uint32_t power;
};

/***************************************************************************//**
 * @brief The `adis_iio_dev` structure is a descriptor for an ADIS IIO device,
 * encapsulating both the ADIS device and IIO device structures. It
 * manages various settings and states related to data acquisition, such
 * as sampling frequency, burst size, and sync mode. It also tracks the
 * number of lost samples and the current data counter for buffer
 * readings. Additionally, it includes a data buffer for storing sample
 * sets and a flag indicating FIFO support. The structure is essential
 * for interfacing with ADIS devices in an IIO context, providing
 * necessary configurations and state management.
 *
 * @param adis_dev Pointer to the ADIS device structure.
 * @param iio_dev Pointer to the IIO device structure.
 * @param samples_lost Number of lost samples for the current buffer reading.
 * @param data_cntr Current data counter for the current buffer reading.
 * @param sampling_frequency ADIS sampling frequency.
 * @param burst_size Current setting for ADIS burst size.
 * @param burst_sel Current setting for ADIS burst data selection.
 * @param sync_mode Current setting for ADIS sync mode.
 * @param data Data buffer to store one sample-set.
 * @param has_fifo Indicates if the IIO device offers FIFO support for buffer
 * reading.
 * @param rang_mdl_txt Gyroscope measurement range value in text.
 * @param hw_trig_desc Pointer to the hardware trigger descriptor.
 ******************************************************************************/
struct adis_iio_dev {
	/** ADIS device structure. */
	struct adis_dev *adis_dev;
	/** IIO device structure. */
	struct iio_device *iio_dev;
	/** Number of lost samples for the current buffer reading. */
	uint16_t samples_lost;
	/** Current data counter for the current buffer reading.*/
	uint32_t data_cntr;
	/** ADIS sampling frequency. */
	uint32_t sampling_frequency;
	/** Current setting for adis burst size. */
	uint32_t burst_size;
	/** Current setting for adis burst data selection. */
	uint32_t burst_sel;
	/** Current setting for adis sync mode. */
	uint32_t sync_mode;
	/** Data buffer to store one sample-set. */
	uint16_t data[26];
	/** True if iio device offers FIFO support for buffer reading. */
	bool has_fifo;
	/** Gyroscope measurement range value in text. */
	const char *rang_mdl_txt;
	struct iio_hw_trig *hw_trig_desc;
};

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ADIS_ACCEL_CHAN(mod, idx, adis_nb, attr) { \
	.ch_type = IIO_ACCEL, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
	.scan_index = idx, \
	.scan_type = &adis##adis_nb##_iio_accel_scan_type,  \
        .attributes = attr,  \
}

#define ADIS_GYRO_CHAN(mod, idx, adis_nb, attr) { \
	.ch_type = IIO_ANGL_VEL, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
	.scan_index = idx, \
	.scan_type = &adis##adis_nb##_iio_anglvel_scan_type,  \
        .attributes = attr,  \
}

#define ADIS_DELTA_ANGL_CHAN(mod, idx, adis_nb, attr) { \
	.ch_type = IIO_DELTA_ANGL, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
	.scan_index = idx, \
	.scan_type = &adis##adis_nb##_iio_delta_angl_scan_type,  \
        .attributes = attr,  \
}

#define ADIS_DELTA_ANGL_CHAN_NO_SCAN(mod, idx, attr) { \
	.ch_type = IIO_DELTA_ANGL, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
        .attributes = attr,  \
}

#define ADIS_DELTA_VEL_CHAN(mod, idx, adis_nb, attr) { \
	.ch_type = IIO_DELTA_VELOCITY, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
	.scan_index = idx, \
	.scan_type = &adis##adis_nb##_iio_delta_vel_scan_type,  \
        .attributes = attr,  \
}

#define ADIS_DELTA_VEL_CHAN_NO_SCAN(mod, idx, attr) { \
	.ch_type = IIO_DELTA_VELOCITY, \
	.modified = true, \
	.channel2 = IIO_MOD_##mod, \
	.address = idx, \
        .attributes = attr,  \
}

#define ADIS_TEMP_CHAN(idx, adis_nb, attr) { \
	.ch_type = IIO_TEMP, \
	.channel = 0, \
	.indexed = true, \
	.address = idx, \
	.scan_index = idx, \
	.scan_type = &adis##adis_nb##_iio_temp_scan_type,  \
        .attributes = attr,  \
}

/******************************************************************************/
/************************ Variables Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adis_dev_attrs` is an external array of `iio_attribute`
 * structures. These attributes are likely used to define various
 * properties or settings for an ADIS device within the IIO (Industrial
 * I/O) framework.
 *
 * @details This variable is used to store and manage device-specific attributes
 * for ADIS devices in the IIO subsystem.
 ******************************************************************************/
extern struct iio_attribute adis_dev_attrs[];
/***************************************************************************//**
 * @brief The `adis_iio_anglvel_attrs` is an external array of `iio_attribute`
 * structures. These attributes are likely related to angular velocity
 * measurements for the ADIS device, as suggested by the name 'anglvel'
 * which is a common abbreviation for angular velocity.
 *
 * @details This variable is used to define and manage the attributes associated
 * with angular velocity channels in the ADIS IIO driver.
 ******************************************************************************/
extern struct iio_attribute adis_iio_anglvel_attrs[];
/***************************************************************************//**
 * @brief `adis_iio_delta_angl_attrs` is an external array of `struct
 * iio_attribute` that is used to define attributes related to delta
 * angle measurements in the ADIS IIO driver. This array is part of the
 * internal driver definitions for specific chip implementations, which
 * are not intended for end-user interaction.
 *
 * @details This variable is used to store and manage the attributes associated
 * with delta angle channels in the ADIS IIO driver.
 ******************************************************************************/
extern struct iio_attribute adis_iio_delta_angl_attrs[];
/***************************************************************************//**
 * @brief The `adis_iio_delta_vel_attrs` is an external array of `iio_attribute`
 * structures. It is used to define attributes related to delta velocity
 * measurements in the ADIS IIO driver.
 *
 * @details This variable is used to store and manage the attributes associated
 * with delta velocity channels in the ADIS IIO device.
 ******************************************************************************/
extern struct iio_attribute adis_iio_delta_vel_attrs[];
/***************************************************************************//**
 * @brief The `adis_iio_accel_attrs` is an external array of `iio_attribute`
 * structures. It is used to define attributes specific to the
 * accelerometer channels in the ADIS IIO driver.
 *
 * @details This variable is used to store and manage the attributes related to
 * accelerometer data in the ADIS IIO device driver.
 ******************************************************************************/
extern struct iio_attribute adis_iio_accel_attrs[];
/***************************************************************************//**
 * @brief The `adis_iio_temp_attrs` is an external array of `iio_attribute`
 * structures. It is used to define attributes related to temperature
 * channels for ADIS devices in the IIO (Industrial I/O) subsystem.
 *
 * @details This variable is used to store and manage temperature-related
 * attributes for ADIS devices within the IIO framework.
 ******************************************************************************/
extern struct iio_attribute adis_iio_temp_attrs[];
/***************************************************************************//**
 * @brief The `adis_iio_trig_desc` is a global variable of type `struct
 * iio_trigger`, which is part of the IIO (Industrial I/O) subsystem in
 * the Linux kernel. This structure is used to describe a trigger
 * mechanism for IIO devices, which can be used to synchronize data
 * acquisition with external events or conditions.
 *
 * @details This variable is used to define and manage the trigger mechanism for
 * ADIS IIO devices, allowing them to respond to specific events or
 * conditions for data acquisition.
 ******************************************************************************/
extern struct iio_trigger adis_iio_trig_desc;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function should be called before enabling a trigger on an ADIS
 * IIO device. It configures the device based on the provided mask,
 * setting up burst data selection and burst size if applicable. It also
 * initializes internal counters and, if the device supports FIFO,
 * configures the FIFO to overwrite old data when full. The function
 * requires a valid device pointer and returns an error code if the
 * device is not properly initialized or if the mask is invalid.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null and
 * must point to a valid, initialized device.
 * @param mask A 32-bit unsigned integer representing the configuration mask. It
 * should not have both ADIS_BURST_DATA_SEL_0_CHN_MASK and
 * ADIS_BURST_DATA_SEL_1_CHN_MASK set simultaneously, as this will
 * result in an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other error codes for specific
 * failures during configuration.
 ******************************************************************************/
int adis_iio_pre_enable(void* dev, uint32_t mask);
/***************************************************************************//**
 * @brief This function should be called after a trigger has been disabled on an
 * ADIS IIO device. It ensures that any necessary post-disable operations
 * are performed, such as disabling the FIFO if it is supported by the
 * device. The function requires a valid device pointer and will return
 * an error if the device is not properly initialized or if the device
 * does not support the required operations.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null and
 * must point to a properly initialized device. If null or improperly
 * initialized, the function returns -EINVAL.
 * @param mask A 32-bit unsigned integer representing the mask for the
 * operation. The specific use of this parameter is not detailed in
 * the header file, but it is required for the function call.
 * @return Returns 0 on success, or a negative error code (e.g., -EINVAL) if the
 * device pointer is null or the device is not properly initialized.
 ******************************************************************************/
int adis_iio_post_disable(void* dev, uint32_t mask);
/*! Read adis iio samples for the active channels. */
int adis_iio_read_samples(void* dev, int* buff, uint32_t samples);
/***************************************************************************//**
 * @brief This function is used as a callback to handle data ready triggers for
 * an IIO device. It should be called when a data ready interrupt occurs,
 * and it processes the active channels of the device. The function
 * requires a valid `iio_device_data` structure, which must contain a
 * valid `adis_iio_dev` device descriptor. If the provided `dev_data` is
 * null or if the `adis_dev` within the `adis_iio_dev` is null, the
 * function returns an error code. This function is typically used in the
 * context of IIO device drivers to manage data acquisition.
 *
 * @param dev_data A pointer to an `iio_device_data` structure representing the
 * IIO device data. It must not be null, and it must contain a
 * valid `adis_iio_dev` device descriptor. If invalid, the
 * function returns -EINVAL.
 * @return Returns 0 on success, or -EINVAL if the input data is invalid.
 ******************************************************************************/
int adis_iio_trigger_handler(struct iio_device_data *dev_data);
/***************************************************************************//**
 * @brief This function processes IIO trigger events for ADIS devices that
 * support FIFO, ensuring data is read and handled correctly. It should
 * be called when a trigger event occurs, and it manages the reading of
 * FIFO data, handling burst requests if necessary. The function must be
 * called with a valid `iio_device_data` structure, and it temporarily
 * disables the hardware trigger during its operation. It returns an
 * error code if the input is invalid or if any operation fails during
 * execution.
 *
 * @param dev_data A pointer to an `iio_device_data` structure representing the
 * device data. Must not be null. The function returns -EINVAL
 * if this parameter is null or if the `adis_dev` field within
 * the associated `adis_iio_dev` structure is null.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input.
 ******************************************************************************/
int adis_iio_trigger_handler_with_fifo(struct iio_device_data *dev_data);

/***************************************************************************//**
 * @brief This function retrieves specific debug attributes from an ADIS IIO
 * device and writes the result into a provided buffer. It is used to
 * access diagnostic information or configuration settings identified by
 * the `priv` parameter. The function must be called with a valid device
 * pointer and a buffer large enough to hold the output. If the device
 * pointer is null or the specified attribute is invalid, the function
 * returns an error code. This function is typically used in debugging or
 * monitoring scenarios where detailed device status is required.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. The
 * function returns -EINVAL if this parameter is null.
 * @param buf A character buffer where the read attribute value will be stored.
 * The buffer must be large enough to hold the output data.
 * @param len The length of the buffer `buf`. It should be sufficient to store
 * the formatted output value.
 * @param channel A pointer to an `iio_ch_info` structure, which provides
 * channel-specific information. This parameter is not used in
 * the function and can be null.
 * @param priv An integer representing the specific debug attribute to read. It
 * must correspond to a valid attribute defined in the
 * `adis_iio_debug_attrs` enumeration. If the attribute is invalid,
 * the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The result
 * of the read operation is written to the `buf` parameter.
 ******************************************************************************/
int adis_iio_read_debug_attrs(void *dev, char *buf, uint32_t len,
			      const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function is used to write debug attributes to an ADIS IIO device,
 * allowing for configuration or control of various device parameters. It
 * should be called when there is a need to modify specific attributes of
 * the device, as indicated by the `priv` parameter. The function
 * requires a valid device pointer and a buffer containing the value to
 * be written. It returns an error code if the device is not properly
 * initialized or if the operation fails.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. The
 * function returns -EINVAL if this parameter is null.
 * @param buf A character buffer containing the value to be written. The buffer
 * should be properly formatted as expected by the device.
 * @param len The length of the buffer. It should be sufficient to contain the
 * value to be written.
 * @param channel A pointer to the IIO channel information structure. This
 * parameter is not used in the function and can be null.
 * @param priv An integer representing the specific debug attribute to be
 * written. It determines which device parameter is being modified.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input or other error codes for specific operation
 * failures.
 ******************************************************************************/
int adis_iio_write_debug_attrs(void *dev, char *buf, uint32_t len,
			       const struct iio_ch_info *channel, intptr_t priv);

/***************************************************************************//**
 * @brief This function is used to read a 16-bit register value from a specified
 * register of an ADIS IIO device. It is typically called when there is a
 * need to retrieve configuration or status information from the device.
 * The function requires a valid device descriptor and a register address
 * to read from. The result is stored in the provided output parameter.
 * It is important to ensure that the device is properly initialized
 * before calling this function to avoid undefined behavior.
 *
 * @param device A pointer to an adis_iio_dev structure representing the ADIS
 * IIO device. Must not be null, and the device should be properly
 * initialized before use.
 * @param reg A 32-bit unsigned integer specifying the register address to read
 * from. The valid range depends on the specific device's register
 * map.
 * @param readval A pointer to a 32-bit unsigned integer where the read register
 * value will be stored. Must not be null.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the read operation.
 ******************************************************************************/
int adis_iio_read_reg(struct adis_iio_dev *device, uint32_t reg,
		      uint32_t *readval);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to a specific register
 * on an ADIS IIO device. It is typically called when there is a need to
 * configure or modify the settings of the device by writing to its
 * registers. The function requires a valid device descriptor and
 * register address, and it returns an integer status code indicating the
 * success or failure of the operation. It is important to ensure that
 * the device is properly initialized before calling this function.
 *
 * @param device A pointer to an adis_iio_dev structure representing the ADIS
 * IIO device. Must not be null, and the device should be properly
 * initialized before use.
 * @param reg A 32-bit unsigned integer specifying the register address to which
 * the value will be written. The valid range depends on the specific
 * device's register map.
 * @param writeval A 32-bit unsigned integer representing the value to be
 * written to the specified register. The valid range and
 * meaning of the value depend on the specific register being
 * accessed.
 * @return Returns an integer status code, where 0 typically indicates success
 * and a negative value indicates an error.
 ******************************************************************************/
int adis_iio_write_reg(struct adis_iio_dev *device, uint32_t reg,
		       uint32_t writeval);

/***************************************************************************//**
 * @brief This function retrieves raw data from a specified channel of an ADIS
 * IIO device and formats it into a buffer. It should be called when raw
 * sensor data is needed from a specific channel, such as gyroscope or
 * accelerometer readings. The function requires a valid device pointer
 * and channel information. It returns an error if the device is not
 * properly initialized or if the channel address is invalid. The
 * function formats the retrieved data into the provided buffer, ensuring
 * the buffer is large enough to hold the data.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. The
 * function returns -EINVAL if this parameter is null.
 * @param buf A character buffer where the raw data will be formatted and
 * stored. The buffer must be large enough to hold the formatted
 * data.
 * @param len The length of the buffer in bytes. It should be sufficient to
 * store the formatted data.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel from which to read data. The channel's address must
 * correspond to a valid ADIS channel.
 * @param priv An integer value reserved for private use, typically for passing
 * additional information. It is not used in this function.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int adis_iio_read_raw(void *dev, char *buf, uint32_t len,
		      const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the scale attribute for a specified IIO
 * channel associated with an ADIS device and formats it into a buffer.
 * It should be called when the scale of a channel needs to be read, such
 * as angular velocity, acceleration, delta angle, delta velocity, or
 * temperature. The function requires a valid device pointer and channel
 * information. If the device or channel type is invalid, it returns an
 * error code. The function formats the scale as a fractional or
 * fractional log2 value, depending on the channel type, and writes it to
 * the provided buffer.
 *
 * @param dev A pointer to the device structure. Must not be null. The function
 * returns -EINVAL if this parameter is null.
 * @param buf A character buffer where the formatted scale value will be
 * written. The buffer must be large enough to hold the formatted
 * output.
 * @param len The length of the buffer. It should be sufficient to store the
 * formatted scale value.
 * @param channel A pointer to the IIO channel information structure. It
 * specifies the type of channel for which the scale is being
 * read. Must not be null.
 * @param priv An integer pointer used for private data. It is not used in this
 * function and can be ignored.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other error codes for specific
 * channel type failures.
 ******************************************************************************/
int adis_iio_read_scale(void *dev, char *buf, uint32_t len,
			const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the offset value for a specified IIO channel
 * associated with a given device. It is primarily used to obtain the
 * temperature offset when the channel type is IIO_TEMP. The function
 * must be called with a valid device pointer and a channel structure
 * that specifies the channel type. If the device or the associated ADIS
 * device is not properly initialized, the function will return an error.
 * The offset value is formatted and stored in the provided buffer.
 *
 * @param dev A pointer to the device structure. Must not be null. The device
 * should be properly initialized and associated with an ADIS device.
 * @param buf A character buffer where the formatted offset value will be
 * stored. The buffer must be large enough to hold the formatted
 * data.
 * @param len The length of the buffer. It should be sufficient to store the
 * formatted offset value.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel type. The function currently supports only the
 * IIO_TEMP channel type.
 * @param priv An integer pointer used for private data. It is not utilized in
 * this function.
 * @return Returns 0 on success, or a negative error code if the device is
 * invalid, the channel type is unsupported, or another error occurs.
 ******************************************************************************/
int adis_iio_read_offset(void *dev, char *buf, uint32_t len,
			 const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the calibration bias value for a specified
 * channel of an ADIS device and formats it into a buffer. It should be
 * called when the calibration bias of a specific sensor channel is
 * needed. The function requires a valid device pointer and channel
 * information. It handles errors by returning negative error codes if
 * the device or channel is invalid or if the read operation fails.
 *
 * @param dev A pointer to the device structure. Must not be null. The function
 * returns -EINVAL if this parameter is null or if the device's
 * internal ADIS device pointer is null.
 * @param buf A character buffer where the formatted calibration bias value will
 * be stored. The buffer must be large enough to hold the formatted
 * data.
 * @param len The length of the buffer. It should be sufficient to store the
 * formatted calibration bias value.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel for which the calibration bias is to be read. The
 * channel's address must correspond to a valid sensor channel
 * (e.g., ADIS_GYRO_X, ADIS_ACCEL_Y). If the address is invalid,
 * the function returns -EINVAL.
 * @param priv An integer value that is not used in this function but is part of
 * the function signature for compatibility with other similar
 * functions.
 * @return Returns 0 on success, or a negative error code on failure. The buffer
 * is populated with the formatted calibration bias value if successful.
 ******************************************************************************/
int adis_iio_read_calibbias(void *dev, char *buf, uint32_t len,
			    const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief Use this function to set a calibration bias for a specific channel of
 * an ADIS IIO device. It requires a valid device pointer and a buffer
 * containing the bias value in a specific format. The function supports
 * writing biases to gyroscope and accelerometer channels. Ensure the
 * device and its associated ADIS device are properly initialized before
 * calling this function. Invalid device pointers or unsupported channel
 * addresses will result in an error.
 *
 * @param dev Pointer to the ADIS IIO device structure. Must not be null. The
 * function returns an error if this is invalid.
 * @param buf Character buffer containing the calibration bias value to be
 * written. The value must be in a format that can be parsed as an
 * integer.
 * @param len Length of the buffer. It is used to determine the size of the data
 * in 'buf'.
 * @param channel Pointer to an IIO channel information structure specifying the
 * target channel. The channel's address must correspond to a
 * supported gyroscope or accelerometer channel.
 * @param priv Private data associated with the operation. This parameter is not
 * used in the function.
 * @return Returns 0 on success or a negative error code on failure, such as
 * invalid input or unsupported channel.
 ******************************************************************************/
int adis_iio_write_calibbias(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the calibration scale value for a specified
 * channel of an ADIS IIO device and formats it into a buffer. It should
 * be called when the calibration scale of a specific channel is needed,
 * such as for gyroscope or accelerometer axes. The function requires a
 * valid device pointer and channel information. It returns an error if
 * the device or channel is invalid or if the read operation fails. The
 * buffer is populated with the formatted scale value if the operation is
 * successful.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. The
 * function returns -EINVAL if this parameter is null.
 * @param buf A character buffer where the formatted scale value will be stored.
 * The buffer must be large enough to hold the formatted output.
 * @param len The length of the buffer. It should be sufficient to store the
 * formatted scale value.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel for which the scale is to be read. The channel's
 * address must correspond to a valid gyroscope or accelerometer
 * channel.
 * @param priv An integer pointer used for private data. This parameter is not
 * used in the function and can be set to 0.
 * @return Returns 0 on success, with the buffer containing the formatted scale
 * value. Returns a negative error code on failure, such as -EINVAL for
 * invalid parameters or if the channel is not supported.
 ******************************************************************************/
int adis_iio_read_calibscale(void *dev, char *buf, uint32_t len,
			     const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function is used to set a calibration scale for a specific
 * channel of an ADIS device. It should be called when a new calibration
 * scale needs to be applied to a gyroscope or accelerometer channel. The
 * function requires a valid device pointer and a buffer containing the
 * scale value in a specific format. It returns an error if the device
 * pointer is null, if the channel is not supported, or if the scale
 * value cannot be parsed. The function must be called with a valid
 * channel address corresponding to a gyroscope or accelerometer axis.
 *
 * @param dev A pointer to the device structure. Must not be null. The function
 * returns -EINVAL if this parameter is null.
 * @param buf A character buffer containing the calibration scale value to be
 * written. The value must be in a format that can be parsed by the
 * function.
 * @param len The length of the buffer. This parameter is used to determine the
 * size of the input data.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel to which the calibration scale should be applied. The
 * channel's address must correspond to a supported gyroscope or
 * accelerometer axis.
 * @param priv An integer value that is not used in this function but is part of
 * the function signature for compatibility with other similar
 * functions.
 * @return Returns 0 on success, or a negative error code on failure. Possible
 * errors include -EINVAL for invalid input or unsupported channel, and
 * other negative values for parsing errors.
 ******************************************************************************/
int adis_iio_write_calibscale(void *dev, char *buf, uint32_t len,
			      const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function is used to set the sampling frequency of an ADIS IIO
 * device. It should be called when there is a need to update the
 * device's sampling rate. The function requires a valid device pointer
 * and a buffer containing the new frequency value in a specific format.
 * It returns an error code if the device pointer is null or if the
 * frequency value cannot be parsed or set. The function updates the
 * device's internal sampling frequency state upon successful execution.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. The
 * function returns -EINVAL if this parameter is null.
 * @param buf A character buffer containing the new sampling frequency value.
 * The value should be in a format that can be parsed as an integer.
 * @param len The length of the buffer. It is used to determine the size of the
 * data in 'buf'.
 * @param channel A pointer to an IIO channel information structure. This
 * parameter is not used in the function and can be null.
 * @param priv An integer used for private data. This parameter is not used in
 * the function and can be any value.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid input or other error codes for parsing or setting
 * failures.
 ******************************************************************************/
int adis_iio_write_sampling_freq(void *dev, char *buf,
				 uint32_t len, const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the current sampling frequency of a specified
 * ADIS IIO device and formats it into a string. It should be called when
 * the user needs to obtain the sampling frequency for display or logging
 * purposes. The function requires a valid device pointer and a buffer to
 * store the formatted frequency. It handles invalid device pointers by
 * returning an error code.
 *
 * @param dev A pointer to the ADIS IIO device structure. Must not be null. If
 * null, the function returns -EINVAL.
 * @param buf A character buffer where the formatted sampling frequency will be
 * stored. The caller must ensure it is large enough to hold the
 * formatted output.
 * @param len The length of the buffer provided in 'buf'. It should be
 * sufficient to store the formatted frequency string.
 * @param channel A pointer to an IIO channel information structure. This
 * parameter is not used in the function and can be null.
 * @param priv An integer value representing private data. This parameter is not
 * used in the function.
 * @return Returns 0 on success, or a negative error code on failure (e.g.,
 * -EINVAL for invalid input). The formatted sampling frequency is
 * written to the provided buffer.
 ******************************************************************************/
int adis_iio_read_sampling_freq(void *dev, char *buf, uint32_t len,
				const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function retrieves the low-pass filter frequency for a specified
 * channel of an ADIS device and formats it into a buffer. It should be
 * called when the low-pass filter frequency needs to be read for a
 * specific channel, such as a gyroscope or accelerometer axis. The
 * function requires a valid device pointer and channel information. If
 * the device or channel is invalid, the function returns an error. The
 * frequency is formatted and stored in the provided buffer, which must
 * be large enough to hold the formatted data.
 *
 * @param dev A pointer to the device structure. Must not be null. If null, the
 * function returns -EINVAL.
 * @param buf A character buffer where the formatted frequency will be stored.
 * The buffer must be large enough to hold the formatted data.
 * @param len The length of the buffer. It should be sufficient to store the
 * formatted frequency value.
 * @param channel A pointer to the channel information structure, specifying
 * which channel's low-pass filter frequency to read. The channel
 * must be valid and correspond to a supported channel type.
 * @param priv An integer value that is not used in this function but is part of
 * the function signature for compatibility with other similar
 * functions.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int adis_iio_read_lpf(void *dev, char *buf, uint32_t len,
		      const struct iio_ch_info *channel, intptr_t priv);
/***************************************************************************//**
 * @brief This function configures the low-pass filter frequency for a specified
 * channel of an ADIS device. It should be called when you need to adjust
 * the filter settings for either gyroscope or accelerometer channels.
 * The function requires a valid device pointer and channel information.
 * It parses the frequency value from the provided buffer and applies it
 * to the specified channel. If the device or channel is invalid, or if
 * the frequency parsing fails, the function returns an error code.
 *
 * @param dev A pointer to the device structure. Must not be null. The function
 * returns -EINVAL if this parameter is null or if the device's ADIS
 * structure is not initialized.
 * @param buf A character buffer containing the frequency value to be set. The
 * buffer should be formatted correctly to allow parsing of an
 * integer value.
 * @param len The length of the buffer. This parameter is used to ensure the
 * buffer is read correctly.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel for which the low-pass filter frequency is to be set.
 * The channel's address determines which specific channel (e.g.,
 * gyroscope X, Y, Z, or accelerometer X, Y, Z) is targeted.
 * @param priv An integer value that is not used in this function but is
 * typically used for private data in similar API functions.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input or other error codes for parsing failures.
 ******************************************************************************/
int adis_iio_write_lpf(void *dev, char *buf, uint32_t len,
		       const struct iio_ch_info *channel, intptr_t priv);
#endif

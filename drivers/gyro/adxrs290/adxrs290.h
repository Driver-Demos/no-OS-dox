/***************************************************************************//**
 *   @file   adxrs290.h
 *   @brief  Implementation of ADXRS290 Driver.
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#ifndef ADXRS290_H_
#define ADXRS290_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* ADXRS290 ID */
#define ADXRS290_ADI_ID				0xAD
#define ADXRS290_MEMS_ID			0x1D
#define ADXRS290_DEV_ID				0x92

/* ADXRS290 Registers */
#define ADXRS290_REG_ADI_ID			0x00
#define ADXRS290_REG_MEMS_ID			0x01
#define ADXRS290_REG_DEV_ID			0x02
#define ADXRS290_REG_REV_ID			0x03
#define ADXRS290_REG_SN0			0x04
#define ADXRS290_REG_SN1			0x05
#define ADXRS290_REG_SN2			0x06
#define ADXRS290_REG_SN3			0x07
#define ADXRS290_REG_DATAX0			0x08
#define ADXRS290_REG_DATAX1			0x09
#define ADXRS290_REG_DATAY0			0x0A
#define ADXRS290_REG_DATAY1			0x0B
#define ADXRS290_REG_TEMP0			0x0C
#define ADXRS290_REG_TEMP1			0x0D

#define ADXRS290_REG_POWER_CTL			0x10
#define ADXRS290_REG_FILTER			0x11
#define ADXRS290_REG_DATA_READY			0x12

#define ADXRS290_READ				NO_OS_BIT(7)
#define ADXRS290_TSM				NO_OS_BIT(0)
#define ADXRS290_MEASUREMENT			NO_OS_BIT(1)
#define ADXRS290_DATA_RDY_OUT			NO_OS_BIT(0)
#define ADXRS290_SYNC_MASK			0x03
#define ADXRS290_SYNC(x)			(x) & ADXRS290_SYNC_MASK
#define ADXRS290_LPF_MASK			0x07
#define ADXRS290_LPF(x)				(x) & ADXRS290_LPF_MASK
#define ADXRS290_HPF_MASK			0xF0
#define ADXRS290_HPF(x)				((x) & ADXRS290_HPF_MASK ) >> 4

#define ADXRS290_READ_REG(reg)	(ADXRS290_READ | (reg))

#define ADXRS290_MAX_TRANSITION_TIME_MS 100
#define ADXRS290_CHANNEL_COUNT			3
#define ADXRS290_CHANNEL_MASK			0x07
/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxrs290_mode` enumeration defines the operational modes for the
 * ADXRS290 device, which is a gyroscope sensor. It includes two modes:
 * standby and measurement. The standby mode is used when the device is
 * not actively measuring, conserving power, while the measurement mode
 * is used when the device is actively capturing data. This enumeration
 * is crucial for controlling the power state and data acquisition
 * behavior of the ADXRS290 sensor.
 *
 * @param ADXRS290_MODE_STANDBY Represents the standby mode of the ADXRS290
 * device.
 * @param ADXRS290_MODE_MEASUREMENT Represents the measurement mode of the
 * ADXRS290 device.
 ******************************************************************************/
enum adxrs290_mode {
	/** Standby mode */
	ADXRS290_MODE_STANDBY,
	/** Measurement mode */
	ADXRS290_MODE_MEASUREMENT,
};

/***************************************************************************//**
 * @brief The `adxrs290_channel` enumeration defines the different data channels
 * available in the ADXRS290 sensor, which include the X-axis, Y-axis,
 * and temperature channels. This enumeration is used to specify which
 * channel's data is being accessed or manipulated in the sensor's
 * operations.
 *
 * @param ADXRS290_CHANNEL_X Represents the X-axis channel of the ADXRS290
 * sensor.
 * @param ADXRS290_CHANNEL_Y Represents the Y-axis channel of the ADXRS290
 * sensor.
 * @param ADXRS290_CHANNEL_TEMP Represents the temperature channel of the
 * ADXRS290 sensor.
 ******************************************************************************/
enum adxrs290_channel {
	/** X-Axis */
	ADXRS290_CHANNEL_X,
	/** Y-Axis */
	ADXRS290_CHANNEL_Y,
	/** Temp */
	ADXRS290_CHANNEL_TEMP,
};

/***************************************************************************//**
 * @brief The `adxrs290_lpf` enumeration defines various low-pass filter
 * settings for the ADXRS290 device, each corresponding to a specific
 * cutoff frequency. These settings allow the user to configure the
 * device's low-pass filter to suit different application requirements,
 * effectively controlling the bandwidth of the signal being processed.
 *
 * @param ADXRS290_LPF_480HZ Represents a low-pass filter setting with a cutoff
 * frequency of 480 Hz.
 * @param ADXRS290_LPF_320HZ Represents a low-pass filter setting with a cutoff
 * frequency of 320 Hz.
 * @param ADXRS290_LPF_160HZ Represents a low-pass filter setting with a cutoff
 * frequency of 160 Hz.
 * @param ADXRS290_LPF_80HZ Represents a low-pass filter setting with a cutoff
 * frequency of 80 Hz.
 * @param ADXRS290_LPF_56HZ6 Represents a low-pass filter setting with a cutoff
 * frequency of 56.6 Hz.
 * @param ADXRS290_LPF_40HZ Represents a low-pass filter setting with a cutoff
 * frequency of 40 Hz.
 * @param ADXRS290_LPF_28HZ3 Represents a low-pass filter setting with a cutoff
 * frequency of 28.3 Hz.
 * @param ADXRS290_LPF_20HZ Represents a low-pass filter setting with a cutoff
 * frequency of 20 Hz.
 ******************************************************************************/
enum adxrs290_lpf {
	ADXRS290_LPF_480HZ,
	ADXRS290_LPF_320HZ,
	ADXRS290_LPF_160HZ,
	ADXRS290_LPF_80HZ,
	ADXRS290_LPF_56HZ6,
	ADXRS290_LPF_40HZ,
	ADXRS290_LPF_28HZ3,
	ADXRS290_LPF_20HZ
};

/***************************************************************************//**
 * @brief The `adxrs290_hpf` enumeration defines various high-pass filter
 * settings for the ADXRS290 gyroscope sensor, each corresponding to a
 * specific cutoff frequency. These settings allow the user to configure
 * the sensor to filter out low-frequency noise below the specified
 * cutoff frequency, enhancing the accuracy of angular velocity
 * measurements by eliminating unwanted signal components.
 *
 * @param ADXRS290_HPF_ALL_PASS Represents a high-pass filter setting that
 * allows all frequencies to pass through.
 * @param ADXRS290_HPF_0HZ011 Represents a high-pass filter setting with a
 * cutoff frequency of 0.011 Hz.
 * @param ADXRS290_HPF_0HZ022 Represents a high-pass filter setting with a
 * cutoff frequency of 0.022 Hz.
 * @param ADXRS290_HPF_0HZ044 Represents a high-pass filter setting with a
 * cutoff frequency of 0.044 Hz.
 * @param ADXRS290_HPF_0HZ087 Represents a high-pass filter setting with a
 * cutoff frequency of 0.087 Hz.
 * @param ADXRS290_HPF_0HZ175 Represents a high-pass filter setting with a
 * cutoff frequency of 0.175 Hz.
 * @param ADXRS290_HPF_0HZ350 Represents a high-pass filter setting with a
 * cutoff frequency of 0.350 Hz.
 * @param ADXRS290_HPF_0HZ700 Represents a high-pass filter setting with a
 * cutoff frequency of 0.700 Hz.
 * @param ADXRS290_HPF_1HZ400 Represents a high-pass filter setting with a
 * cutoff frequency of 1.400 Hz.
 * @param ADXRS290_HPF_2HZ800 Represents a high-pass filter setting with a
 * cutoff frequency of 2.800 Hz.
 * @param ADXRS290_HPF_11HZ30 Represents a high-pass filter setting with a
 * cutoff frequency of 11.30 Hz.
 ******************************************************************************/
enum adxrs290_hpf {
	ADXRS290_HPF_ALL_PASS,
	ADXRS290_HPF_0HZ011,
	ADXRS290_HPF_0HZ022,
	ADXRS290_HPF_0HZ044,
	ADXRS290_HPF_0HZ087,
	ADXRS290_HPF_0HZ175,
	ADXRS290_HPF_0HZ350,
	ADXRS290_HPF_0HZ700,
	ADXRS290_HPF_1HZ400,
	ADXRS290_HPF_2HZ800,
	ADXRS290_HPF_11HZ30
};

/***************************************************************************//**
 * @brief The `adxrs290_init_param` structure is used to initialize the ADXRS290
 * device driver with specific settings for SPI communication, optional
 * GPIO synchronization, and initial filter and mode configurations. It
 * provides a comprehensive setup for the device, ensuring that all
 * necessary parameters are defined before the device is used, allowing
 * for flexible configuration of the device's operational parameters.
 *
 * @param spi_init This member holds the SPI initialization parameters for the
 * device.
 * @param gpio_sync This optional pointer to a GPIO initialization structure is
 * used for synchronization; if not set, data ready
 * functionality will fail.
 * @param mode This member specifies the initial operating mode of the device,
 * such as standby or measurement.
 * @param lpf This member sets the initial low-pass filter settings for the
 * device.
 * @param hpf This member sets the initial high-pass filter settings for the
 * device.
 ******************************************************************************/
struct adxrs290_init_param {
	/** SPI Initialization structure. */
	struct no_os_spi_init_param	spi_init;
	/** Optional. If not set adxrs290_get_data_ready will fail */
	struct no_os_gpio_init_param	*gpio_sync;
	/** Initial Mode */
	enum adxrs290_mode	mode;
	/** Initial lpf settings */
	enum adxrs290_lpf	lpf;
	/** Initial hpf settings */
	enum adxrs290_hpf	hpf;
};

/***************************************************************************//**
 * @brief The `adxrs290_dev` structure serves as a device driver handler for the
 * ADXRS290 gyroscope sensor. It encapsulates the necessary components
 * for interfacing with the sensor, including an SPI descriptor for
 * communication, a GPIO descriptor for synchronization, and a channel
 * mask to specify which channels are active. This structure is essential
 * for managing the state and configuration of the ADXRS290 device within
 * the driver.
 *
 * @param spi_desc A pointer to the SPI handler used for communication with the
 * device.
 * @param gpio_sync A pointer to the GPIO descriptor used for synchronization.
 * @param ch_mask A bitmask representing the active channels on the device.
 ******************************************************************************/
struct adxrs290_dev {
	/** SPI handler */
	struct no_os_spi_desc		*spi_desc;
	/** GPIO */
	struct no_os_gpio_desc	*gpio_sync;
	/** Active Channels */
	uint8_t			ch_mask;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief This function reads a single byte from a specified register of the
 * ADXRS290 device. It is typically used to retrieve configuration or
 * status information from the device. The function requires a valid
 * device structure and a register address to read from. The caller must
 * ensure that the device has been properly initialized before calling
 * this function. The function will return an error if the read operation
 * fails.
 *
 * @param dev A pointer to an initialized `adxrs290_dev` structure representing
 * the device. Must not be null.
 * @param address The register address to read from. Must be a valid register
 * address defined for the ADXRS290.
 * @param data A pointer to a uint8_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if the read operation fails.
 ******************************************************************************/
int32_t adxrs290_reg_read(struct adxrs290_dev *dev, uint8_t address,
			  uint8_t *data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function writes a byte of data to a specific register on the
 * ADXRS290 device using SPI communication. It is typically used to
 * configure the device by setting control registers. The function
 * requires a valid device structure that has been initialized and a
 * valid register address. The address is masked to ensure it is within
 * the writable range. This function should be called when the device is
 * in a state that allows register modification, such as during
 * initialization or configuration changes.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null, and the SPI descriptor within must
 * be properly configured.
 * @param address The register address to which the data will be written. The
 * address is masked to ensure it is within the valid range (0x00
 * to 0x7F).
 * @param data The byte of data to write to the specified register. There are no
 * restrictions on the value of this parameter.
 * @return Returns an int32_t status code from the SPI write operation, where 0
 * indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t adxrs290_reg_write(struct adxrs290_dev *dev, uint8_t address,
			   uint8_t data);

/* Set mode */
/***************************************************************************//**
 * @brief Use this function to change the operational mode of the ADXRS290
 * device between standby and measurement modes. This function should be
 * called when you need to switch the device's mode of operation, for
 * example, to conserve power by setting it to standby when measurements
 * are not needed. Ensure that the device has been properly initialized
 * before calling this function. The function will return an error if an
 * invalid mode is provided.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adxrs290_mode indicating the desired
 * operational mode. Valid values are ADXRS290_MODE_STANDBY and
 * ADXRS290_MODE_MEASUREMENT. Providing an invalid mode will result
 * in an error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or an invalid mode is specified.
 ******************************************************************************/
int32_t adxrs290_set_op_mode(struct adxrs290_dev *dev, enum adxrs290_mode mode);

/* Get the low-pass filter pole location. */
/***************************************************************************//**
 * @brief Use this function to obtain the current low-pass filter pole location
 * setting of the ADXRS290 device. This function is typically called
 * after the device has been initialized and is operational. It reads the
 * relevant register to determine the low-pass filter setting and returns
 * it through the provided pointer. Ensure that the device structure is
 * properly initialized before calling this function. The function will
 * return an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param lpf A pointer to an adxrs290_lpf enum where the current low-pass
 * filter setting will be stored. Must not be null.
 * @return Returns an int32_t error code: 0 for success, or a negative error
 * code if the read operation fails.
 ******************************************************************************/
int32_t adxrs290_get_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf *lpf);

/* Set the low-pass filter pole location. */
/***************************************************************************//**
 * @brief This function configures the low-pass filter (LPF) pole location for
 * the ADXRS290 device, which affects the frequency response of the
 * sensor's output. It should be called when you need to adjust the LPF
 * settings to match your application's requirements. Ensure that the
 * device has been properly initialized before calling this function. The
 * function reads the current filter settings, modifies the LPF bits, and
 * writes the new configuration back to the device. It returns an error
 * code if the operation fails, which can occur if the device is not
 * properly connected or if there is a communication error.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param lpf An enum value of type adxrs290_lpf representing the desired low-
 * pass filter setting. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t adxrs290_set_lpf(struct adxrs290_dev *dev, enum adxrs290_lpf lpf);

/* Get the high-pass filter pole location. */
/***************************************************************************//**
 * @brief Use this function to obtain the current high-pass filter configuration
 * of the ADXRS290 device. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * reads the relevant register and updates the provided high-pass filter
 * enumeration with the current setting. This function is useful for
 * applications that need to verify or log the current filter settings of
 * the device.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param hpf A pointer to an adxrs290_hpf enumeration where the current high-
 * pass filter setting will be stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails. The hpf parameter is updated with the current
 * high-pass filter setting on success.
 ******************************************************************************/
int32_t adxrs290_get_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf *hpf);

/* Set the high-pass filter pole location. */
/***************************************************************************//**
 * @brief This function configures the high-pass filter (HPF) setting of the
 * ADXRS290 device to the specified pole location. It should be called
 * when you need to adjust the HPF to filter out low-frequency noise or
 * drift in the sensor data. Ensure that the device is properly
 * initialized before calling this function. The function reads the
 * current filter settings, modifies the HPF bits, and writes the new
 * configuration back to the device. It returns an error code if the
 * operation fails, which can occur if there is a communication issue
 * with the device.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param hpf An enum value of type adxrs290_hpf representing the desired high-
 * pass filter pole location. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t adxrs290_set_hpf(struct adxrs290_dev *dev, enum adxrs290_hpf hpf);

/* Read X or Y angular velocity data */
/***************************************************************************//**
 * @brief Use this function to obtain the angular velocity data from a specified
 * channel of the ADXRS290 device. It is essential to ensure that the
 * device has been properly initialized and is in the correct operational
 * mode before calling this function. The function reads data from the
 * specified channel, which can be either the X-axis or Y-axis, and
 * stores the result in the provided rate pointer. This function is
 * useful for applications requiring real-time angular velocity
 * measurements. Handle the return value to check for any communication
 * errors.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param ch An enum value of type adxrs290_channel specifying the channel to
 * read from. Valid values are ADXRS290_CHANNEL_X or
 * ADXRS290_CHANNEL_Y.
 * @param rate A pointer to an int16_t where the read angular velocity data will
 * be stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * a communication error.
 ******************************************************************************/
int32_t adxrs290_get_rate_data(struct adxrs290_dev *dev,
			       enum adxrs290_channel ch, int16_t *rate);

/* Read Temperature data */
/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from the
 * ADXRS290 device. It must be called with a valid device structure that
 * has been properly initialized. The function communicates with the
 * device over SPI to read the temperature registers and stores the
 * result in the provided memory location. Ensure that the device is in a
 * state capable of providing temperature data before calling this
 * function. The function returns an error code if the SPI communication
 * fails.
 *
 * @param dev A pointer to an initialized 'adxrs290_dev' structure representing
 * the device. Must not be null.
 * @param temp A pointer to an int16_t variable where the temperature data will
 * be stored. Must not be null.
 * @return Returns an int32_t error code. A non-negative value indicates
 * success, while a negative value indicates an error during SPI
 * communication.
 ******************************************************************************/
int32_t adxrs290_get_temp_data(struct adxrs290_dev *dev, int16_t *temp);

/* Get the burst data */
/***************************************************************************//**
 * @brief This function retrieves burst data from the ADXRS290 device, which
 * includes data from active channels as specified by the device's
 * channel mask. It should be called when burst data is needed from the
 * device, typically after the device has been properly initialized and
 * configured. The function writes the retrieved data into the provided
 * buffer and updates the channel count to reflect the number of active
 * channels. It returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure representing
 * the device. Must not be null.
 * @param burst_data A pointer to an array of int16_t where the burst data will
 * be stored. The array must be large enough to hold data for
 * all active channels. Must not be null.
 * @param ch_cnt A pointer to a uint8_t where the number of active channels will
 * be stored. Must not be null.
 * @return Returns an int32_t error code. A non-negative value indicates
 * success, while a negative value indicates an error during SPI
 * communication.
 ******************************************************************************/
int32_t adxrs290_get_burst_data(struct adxrs290_dev *dev, int16_t *burst_data,
				uint8_t *ch_cnt);

/* Set the ADXRS290 active channels */
/***************************************************************************//**
 * @brief Use this function to specify which channels of the ADXRS290 device
 * should be active. This is typically done after initializing the device
 * and before starting data acquisition. The function updates the
 * device's channel mask to reflect the desired active channels. It is
 * important to ensure that the device structure is properly initialized
 * before calling this function. The function does not perform any error
 * checking on the input mask beyond applying a predefined channel mask.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure. This must not
 * be null, and the structure should be properly initialized before
 * calling this function.
 * @param mask A 32-bit unsigned integer representing the desired active
 * channels. The value is bitwise ANDed with ADXRS290_CHANNEL_MASK
 * to ensure only valid channels are set.
 * @return Returns 0 on success. The function does not perform any error
 * checking beyond masking the input.
 ******************************************************************************/
int32_t adxrs290_set_active_channels(struct adxrs290_dev *dev, uint32_t mask);

/* Get the data ready state */
/***************************************************************************//**
 * @brief This function checks the data ready status of the ADXRS290 device by
 * reading the state of a GPIO pin. It should be called when you need to
 * determine if new data is available from the device. The function
 * requires that the device has been properly initialized and that the
 * GPIO pin for data ready signaling is configured. If the GPIO pin is
 * not set up, the function will return an error. The data ready status
 * is returned through the provided boolean pointer.
 *
 * @param dev A pointer to an initialized adxrs290_dev structure. This must not
 * be null and should have a valid GPIO configuration for data ready
 * signaling.
 * @param rdy A pointer to a boolean variable where the data ready status will
 * be stored. Must not be null. The function sets this to true if
 * data is ready, false otherwise.
 * @return Returns 0 on success, or a negative error code if the GPIO pin is not
 * configured or if there is an error reading the GPIO value.
 ******************************************************************************/
int32_t adxrs290_get_data_ready(struct adxrs290_dev *dev, bool *rdy);

/* Init. the comm. peripheral and checks if the ADXRS290 part is present. */
/***************************************************************************//**
 * @brief This function initializes the ADXRS290 device by setting up the
 * necessary communication peripherals and configuring the device
 * according to the specified initialization parameters. It must be
 * called before any other operations on the device. The function checks
 * for the presence of the device and configures it to the desired mode,
 * low-pass filter, and high-pass filter settings. It also optionally
 * configures a GPIO pin for synchronization. If initialization fails at
 * any step, the function ensures that allocated resources are properly
 * freed before returning an error code.
 *
 * @param device A pointer to a pointer of type `struct adxrs290_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory upon successful initialization.
 * @param init_param A pointer to a constant `struct adxrs290_init_param`
 * containing the initialization parameters. This includes SPI
 * initialization parameters, optional GPIO synchronization
 * settings, and initial mode, low-pass filter, and high-pass
 * filter settings. The pointer must not be null, and the
 * structure must be properly populated before calling the
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or device not found. On success, the `device`
 * pointer is updated to point to the initialized device structure.
 ******************************************************************************/
int32_t adxrs290_init(struct adxrs290_dev **device,
		      const struct adxrs290_init_param *init_param);

/* Free the resources allocated by adxrs290_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an ADXRS290
 * device when it is no longer needed. This function should be called to
 * clean up after a device has been initialized and used, ensuring that
 * all allocated memory and hardware resources are properly freed. It is
 * important to call this function to prevent resource leaks in the
 * system.
 *
 * @param dev A pointer to an adxrs290_dev structure representing the device to
 * be removed. Must not be null. The function assumes ownership of
 * the resources and will free them, so the caller should not use the
 * pointer after calling this function.
 * @return Returns 0 to indicate successful removal of the device resources.
 ******************************************************************************/
int32_t adxrs290_remove(struct adxrs290_dev *dev);


#endif /* ADXRS290_H_ */

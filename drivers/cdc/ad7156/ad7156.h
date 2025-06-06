/***************************************************************************//**
 *   @file   AD7156.h
 *   @brief  Header file of AD7156 Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#ifndef __AD7156_H__
#define __AD7156_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/*************************** AD7156 Definitions *******************************/
/******************************************************************************/
/*!< AD7156 slave address */
#define AD7156_ADDRESS                  0x48

/*!< AD7156 registers */
#define AD7156_REG_STATUS               0x00
#define AD7156_REG_CH1_DATA_H           0x01
#define AD7156_REG_CH1_DATA_L           0x02
#define AD7156_REG_CH2_DATA_H           0x03
#define AD7156_REG_CH2_DATA_L           0x04
#define AD7156_REG_CH1_AVG_H            0x05
#define AD7156_REG_CH1_AVG_L            0x06
#define AD7156_REG_CH2_AVG_H            0x07
#define AD7156_REG_CH2_AVG_L            0x08
#define AD7156_REG_CH1_SENS_THRSH_H     0x09
#define AD7156_REG_CH1_TMO_THRSH_L      0x0A
#define AD7156_REG_CH1_SETUP            0x0B
#define AD7156_REG_CH2_SENS_THRSH_H     0x0C
#define AD7156_REG_CH2_TMO_THRSH_L      0x0D
#define AD7156_REG_CH2_SETUP            0x0E
#define AD7156_REG_CONFIG               0x0F
#define AD7156_REG_PWR_DWN_TMR          0x10
#define AD7156_REG_CH1_CAPDAC           0x11
#define AD7156_REG_CH2_CAPDAC           0x12
#define AD7156_REG_SERIAL_N3            0x13
#define AD7156_REG_SERIAL_N2            0x14
#define AD7156_REG_SERIAL_N1            0x15
#define AD7156_REG_SERIAL_N0            0x16
#define AD7156_REG_CHIP_ID              0x17

/*!< AD7156_REG_STATUS definition */
#define AD7156_STATUS_PWR_DWN           (1 << 7)
#define AD7156_STATUS_DAC_STEP2         (1 << 6)
#define AD7156_STATUS_OUT2              (1 << 5)
#define AD7156_STATUS_DAC_STEP1         (1 << 4)
#define AD7156_STATUS_OUT1              (1 << 3)
#define AD7156_STATUS_C1_C2             (1 << 2)
#define AD7156_STATUS_RDY2              (1 << 1)
#define AD7156_STATUS_RDY1              (1 << 0)

/*!< AD7156_REG_CH1_SETUP definition */
#define AD7156_CH1_SETUP_RANGE(x)       (((x) & 0x3) << 6)
#define AD7156_CH1_SETUP_HYST1          (1 << 4)
#define AD7156_CH1_SETUP_THR1(x)        ((x) & 0xF)

/*!< AD7156_REG_CH2_SETUP definition */
#define AD7156_CH2_SETUP_RANGE(x)       (((x) & 0x3) << 6)
#define AD7156_CH2_SETUP_HYST2          (1 << 4)
#define AD7156_CH2_SETUP_THR2(x)        ((x) & 0xF)

/*!< AD7156_CH1_SETUP_RANGE(x) and AD7156_CH2_SETUP_RANGE(x) options */
#define AD7156_CDC_RANGE_2_PF           0
#define AD7156_CDC_RANGE_0_5_PF         1
#define AD7156_CDC_RANGE_1_PF           2
#define AD7156_CDC_RANGE_4_PF           3

/*!< AD7156_REG_CONFIG definition */
#define AD7156_CONFIG_THR_FIXED         (1 << 7)
#define AD7156_CONFIG_THR_MD(x)         (((x) & 0x3) << 5)
#define AD7156_CONFIG_EN_CH1            (1 << 4)
#define AD7156_CONFIG_EN_CH2            (1 << 3)
#define AD7156_CONFIG_MD(x)             ((x) & 0x7)

/*!< AD7156_CONFIG_THR_FIXED options */
#define AD7156_ADAPTIVE_THRESHOLD       0
#define AD7156_FIXED_THRESHOLD          1

/*!< AD7156_CONFIG_THR_MD(x) options */
#define AD7156_THR_MODE_NEGATIVE        0
#define AD7156_THR_MODE_POSITIVE        1
#define AD7156_THR_MODE_IN_WINDOW       2
#define AD7156_THR_MODE_OU_WINDOW       3

/*!< AD7156_CONFIG_MD(x) options */
#define AD7156_CONV_MODE_IDLE            0
#define AD7156_CONV_MODE_CONT_CONV       1
#define AD7156_CONV_MODE_SINGLE_CONV     2
#define AD7156_CONV_MODE_PWR_DWN         3

/*!< AD7156_REG_PWR_DWN_TMR definition */
#define AD7156_PWR_DWN_TMR_TIMEOUT(x)   (((x) & 0x3F) | (1 << 6))

/*!< AD7156_REG_CH1_CAPDAC */
#define AD7156_CH1_CAPDAC_DAC_EN1       (1 << 7)
#define AD7156_CH1_CAPDAC_DAC_AUTO1     (1 << 6)
#define AD7156_CH1_CAPDAC_DAC_VAL1(x)   ((x) & 0x3F)

/*!< AD7156_REG_CH2_CAPDAC */
#define AD7156_CH2_CAPDAC_DAC_EN2       (1 << 7)
#define AD7156_CH2_CAPDAC_DAC_AUTO2     (1 << 6)
#define AD7156_CH2_CAPDAC_DAC_VAL2(x)   ((x) & 0x3F)

/*!< Chip identification code */
#define AD7156_DEFAULT_ID               0x88

/*!< AD7156 Reset Command */
#define AD7156_RESET_CMD                0xBF

/*!< AD7156 Channels */
#define AD7156_CHANNEL1                 1
#define AD7156_CHANNEL2                 2

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7156_dev` structure is used to represent a device instance of
 * the AD7156 capacitive sensor. It contains a pointer to an I2C
 * descriptor for handling communication with the device, as well as
 * floating-point values that specify the range settings for the two
 * channels of the AD7156. This structure is essential for configuring
 * and interacting with the AD7156 sensor in a software application.
 *
 * @param i2c_desc Pointer to an I2C descriptor for communication.
 * @param ad7156_channel1_range Floating-point value representing the range
 * setting for channel 1.
 * @param ad7156_channel2_range Floating-point value representing the range
 * setting for channel 2.
 ******************************************************************************/
struct ad7156_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* Device Settings */
	float ad7156_channel1_range;
	float ad7156_channel2_range;
};

/***************************************************************************//**
 * @brief The `ad7156_init_param` structure is used to initialize the AD7156
 * capacitive sensor device. It contains parameters for setting up the
 * I2C communication interface and configuring the range settings for the
 * two channels of the device. This structure is essential for
 * establishing the initial configuration of the AD7156, allowing it to
 * communicate with the host system and operate within the specified
 * capacitance range for each channel.
 *
 * @param i2c_init This member is a structure that holds the initialization
 * parameters for the I2C communication interface.
 * @param ad7156_channel1_range This member specifies the range setting for
 * channel 1 of the AD7156 device.
 * @param ad7156_channel2_range This member specifies the range setting for
 * channel 2 of the AD7156 device.
 ******************************************************************************/
struct ad7156_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* Device Settings */
	float ad7156_channel1_range;
	float ad7156_channel2_range;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to perform a burst read from the AD7156 device,
 * starting at a specified register address. It is essential to ensure
 * that the device has been properly initialized and is ready for
 * communication before calling this function. The function reads a
 * specified number of bytes from the device and stores them in the
 * provided buffer. This function is typically used when you need to
 * retrieve configuration or status information from the device. Ensure
 * that the buffer provided is large enough to hold the number of bytes
 * requested.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param p_read_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold
 * 'bytes_number' bytes.
 * @param register_address The starting register address from which to begin
 * reading. Must be a valid register address on the
 * AD7156.
 * @param bytes_number The number of bytes to read from the device. Must be a
 * positive integer and should not exceed the available data
 * size from the starting register.
 * @return None
 ******************************************************************************/
void ad7156_get_register_value(struct ad7156_dev *dev,
			       uint8_t* p_read_data,
			       uint8_t register_address,
			       uint8_t bytes_number);

/***************************************************************************//**
 * @brief Use this function to write a specified value into one or two registers
 * of the AD7156 device. It is essential to ensure that the device has
 * been properly initialized and is ready for communication before
 * calling this function. The function constructs a data buffer with the
 * register address and the value to be written, then sends it over I2C.
 * The number of bytes to write must be specified, and it should match
 * the size of the register(s) being targeted. This function does not
 * return a value, and any errors during the I2C communication are not
 * reported back to the caller.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param register_value The value to be written to the register(s). It is a
 * 16-bit value, and the relevant bytes are extracted
 * based on the bytes_number parameter.
 * @param register_address The address of the register where the data will be
 * written. It must be a valid register address for the
 * AD7156 device.
 * @param bytes_number The number of bytes to write, which can be 1 or 2. It
 * determines how many bytes of the register_value are
 * written to the device.
 * @return None
 ******************************************************************************/
void ad7156_set_register_value(struct ad7156_dev *dev,
			       uint16_t register_value,
			       uint8_t  register_address,
			       uint8_t  bytes_number);
/***************************************************************************//**
 * @brief This function initializes the AD7156 capacitive sensor device by
 * setting up the necessary I2C communication and configuring the device
 * with the specified initialization parameters. It must be called before
 * any other operations on the device to ensure proper setup. The
 * function checks if the device is present by verifying its chip ID. If
 * the initialization is successful, a device handle is returned for
 * further operations. Ensure that the provided initialization parameters
 * are correctly set to match the desired configuration.
 *
 * @param device A pointer to a pointer of type `struct ad7156_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad7156_init_param` containing
 * the initialization parameters for the device, including I2C
 * settings and channel range configurations. The values must
 * be valid and appropriate for the intended use of the
 * device.
 * @return Returns an `int8_t` status code: 0 for success, or -1 if
 * initialization fails (e.g., due to memory allocation failure or
 * incorrect device ID).
 ******************************************************************************/
int8_t ad7156_init(struct ad7156_dev **device,
		   struct ad7156_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release the resources associated with an
 * AD7156 device instance. It should be called when the device is no
 * longer needed, typically at the end of its usage lifecycle, to ensure
 * that all allocated resources are properly freed. This function must be
 * called after a successful call to ad7156_init(). Failure to call this
 * function may result in resource leaks.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device
 * instance to be removed. This pointer must not be null, and it must
 * point to a valid device instance that was previously initialized
 * using ad7156_init(). Passing an invalid or null pointer will
 * result in undefined behavior.
 * @return Returns an int32_t status code indicating the success or failure of
 * the operation. A return value of 0 typically indicates success, while
 * a non-zero value indicates an error occurred during the resource
 * release process.
 ******************************************************************************/
int32_t ad7156_remove(struct ad7156_dev *dev);

/***************************************************************************//**
 * @brief Use this function to reset the AD7156 device to its default state.
 * This is typically done to ensure the device is in a known state before
 * starting configuration or data acquisition. The function must be
 * called with a valid device structure that has been properly
 * initialized. It communicates with the device over I2C to issue the
 * reset command, so the I2C interface must be correctly set up and
 * operational.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. This
 * structure must be initialized and must not be null. The function
 * assumes the I2C descriptor within this structure is valid and
 * ready for communication.
 * @return None
 ******************************************************************************/
void ad7156_reset(struct ad7156_dev *dev);

/***************************************************************************//**
 * @brief This function configures the power mode of the AD7156 capacitive
 * sensor device. It should be used to change the device's operational
 * state, such as setting it to idle, continuous conversion, single
 * conversion, or power-down mode. This function must be called with a
 * valid device structure and a power mode value that corresponds to one
 * of the predefined modes. It is important to ensure that the device has
 * been properly initialized before calling this function to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. This
 * must not be null, and the device must be initialized prior to
 * calling this function. The caller retains ownership.
 * @param pwr_mode An 8-bit unsigned integer representing the desired power
 * mode. Valid values are defined by AD7156_CONFIG_MD(x)
 * options, such as AD7156_CONV_MODE_IDLE,
 * AD7156_CONV_MODE_CONT_CONV, AD7156_CONV_MODE_SINGLE_CONV, and
 * AD7156_CONV_MODE_PWR_DWN. Invalid values may result in
 * undefined behavior.
 * @return None
 ******************************************************************************/
void ad7156_set_power_mode(struct ad7156_dev *dev,
			   uint8_t pwr_mode);

/***************************************************************************//**
 * @brief This function is used to control the conversion state of a specific
 * channel on the AD7156 device. It allows enabling or disabling the
 * conversion process for either channel 1 or channel 2. This function
 * should be called when you need to start or stop data conversion on a
 * particular channel, typically after the device has been initialized.
 * The function modifies the device's configuration register to reflect
 * the desired state. Ensure that the device pointer is valid and
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. Must
 * not be null and should be initialized prior to calling this
 * function.
 * @param channel Specifies the channel to be configured. Valid values are 1 for
 * channel 1 and 2 for channel 2. If an invalid channel is
 * specified, the behavior is undefined.
 * @param enable_conv A flag indicating whether to enable (non-zero value) or
 * disable (zero value) conversion on the specified channel.
 * @return None
 ******************************************************************************/
void ad7156_channel_state(struct ad7156_dev *dev,
			  uint8_t channel,
			  uint8_t enable_conv);

/***************************************************************************//**
 * @brief This function configures the input range for a specified channel on
 * the AD7156 capacitive sensor device. It should be used when you need
 * to adjust the sensitivity of the channel to different capacitance
 * ranges. The function must be called with a valid device structure and
 * appropriate channel and range values. It updates the device's internal
 * settings to reflect the new range configuration. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param channel Specifies the channel to configure. Valid values are 1
 * (AD7156_CHANNEL1) or 2 (AD7156_CHANNEL2). If an invalid
 * channel is provided, behavior is undefined.
 * @param range Specifies the range setting for the channel. Valid values are 0
 * (2 pF), 1 (0.5 pF), 2 (1 pF), or 3 (4 pF). Values outside this
 * range may result in undefined behavior.
 * @return None
 ******************************************************************************/
void ad7156_set_range(struct ad7156_dev *dev,
		      uint32_t channel,
		      uint8_t range);

/***************************************************************************//**
 * @brief This function is used to obtain the current input range setting of a
 * specified channel on the AD7156 device, expressed in picofarads (pF).
 * It should be called when the user needs to know the configured range
 * for a channel, which can be useful for interpreting sensor data
 * correctly. The function updates the device structure with the range
 * information for the specified channel. It is important to ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param channel The channel number to query, either 1 or 2. If an invalid
 * channel is specified, behavior is undefined.
 * @return Returns the range of the specified channel in picofarads (pF).
 ******************************************************************************/
float ad7156_get_range(struct ad7156_dev *dev,
		       uint32_t channel);

/***************************************************************************//**
 * @brief Use this function to set the threshold mode of the AD7156 capacitive
 * sensor device. This function allows you to configure whether the
 * threshold is fixed or adaptive and to select the threshold mode, such
 * as negative, positive, in-window, or out-of-window. It must be called
 * with a valid device structure that has been initialized. This function
 * modifies the configuration register of the device to apply the
 * specified threshold settings.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param thr_mode A uint8_t value specifying the threshold mode. Valid values
 * are 0 (negative), 1 (positive), 2 (in-window), and 3 (out-of-
 * window). Values outside this range may result in undefined
 * behavior.
 * @param thr_fixed A uint8_t value indicating whether the threshold is fixed
 * (1) or adaptive (0). Any value other than 0 or 1 may result
 * in undefined behavior.
 * @return None
 ******************************************************************************/
void ad7156_set_threshold_mode(struct ad7156_dev *dev,
			       uint8_t thr_mode,
			       uint8_t thr_fixed);

/***************************************************************************//**
 * @brief Use this function to configure the threshold level for a specific
 * channel on the AD7156 capacitive sensor device. This function is
 * typically called when setting up the device for operation in fixed
 * threshold mode. The threshold value is specified in picofarads and is
 * automatically clamped to the valid range for the device. Ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number for which the threshold is being set. Valid
 * values are 1 or 2, corresponding to AD7156_CHANNEL1 and
 * AD7156_CHANNEL2.
 * @param p_fthr The desired threshold value in picofarads. The function will
 * clamp this value to the device's valid range if necessary.
 * @return None
 ******************************************************************************/
void ad7156_set_threshold(struct ad7156_dev *dev,
			  uint8_t channel,
			  float p_fthr);

/***************************************************************************//**
 * @brief Use this function to set the sensitivity for a specific channel on the
 * AD7156 device when operating in adaptive threshold mode. This function
 * calculates the raw sensitivity value based on the provided sensitivity
 * in picofarads and the channel's range, then writes it to the
 * appropriate sensitivity register. Ensure that the device is properly
 * initialized and configured before calling this function. The function
 * does not return a value, and it assumes that the channel parameter is
 * valid.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number for which to set the sensitivity. Valid
 * values are 1 or 2, corresponding to AD7156_CHANNEL1 and
 * AD7156_CHANNEL2.
 * @param p_fsensitivity The desired sensitivity in picofarads. This value is
 * used to compute the raw sensitivity value written to
 * the device.
 * @return None
 ******************************************************************************/
void ad7156_set_sensitivity(struct ad7156_dev *dev,
			    uint8_t channel,
			    float p_fsensitivity);

/***************************************************************************//**
 * @brief This function retrieves a 12-bit data sample from the specified
 * channel of an AD7156 capacitive sensor device. It is typically used to
 * obtain raw sensor data for further processing or analysis. The
 * function requires a valid device structure, which should be
 * initialized and configured prior to calling this function. The channel
 * parameter determines which channel's data is read, and it must be
 * either 1 or 2. The function returns the data as a 16-bit unsigned
 * integer, with the 12-bit sample stored in the lower bits. Ensure that
 * the device is properly powered and configured to avoid erroneous
 * readings.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function. The caller retains ownership.
 * @param channel An unsigned 8-bit integer specifying the channel to read from.
 * Valid values are 1 or 2. If an invalid channel is specified,
 * the function defaults to reading from channel 2.
 * @return Returns a 16-bit unsigned integer containing the 12-bit sample from
 * the specified channel.
 ******************************************************************************/
uint16_t ad7156_read_channel_data(struct ad7156_dev *dev,
				  uint8_t channel);

/***************************************************************************//**
 * @brief This function is used to read a 12-bit sample from a specified channel
 * of the AD7156 device after ensuring that the conversion is complete.
 * It should be called when a precise reading from a specific channel is
 * required, and the caller must wait for the conversion to finish. The
 * function is blocking and will not return until the conversion is
 * complete. It is important to ensure that the device is properly
 * initialized and configured before calling this function.
 *
 * @param dev A pointer to an ad7156_dev structure representing the device
 * instance. Must not be null, and the device must be initialized
 * before use.
 * @param channel The channel number to read from, which can be either 1 or 2.
 * If an invalid channel is specified, the function defaults to
 * channel 2.
 * @return Returns a 12-bit unsigned integer representing the sample data read
 * from the specified channel.
 ******************************************************************************/
uint16_t ad7156_wait_read_channel_data(struct ad7156_dev *dev,
				       uint8_t channel);

/***************************************************************************//**
 * @brief Use this function to obtain the capacitance measurement in picofarads
 * from a specified channel of the AD7156 device. It is essential to
 * ensure that the device is properly initialized and configured before
 * calling this function. The function reads raw data from the specified
 * channel, applies clamping to ensure the data is within a valid range,
 * and then converts it to a capacitance value in picofarads based on the
 * channel's configured range. This function is useful for applications
 * requiring precise capacitance measurements from the AD7156 sensor.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to read from, typically AD7156_CHANNEL1 or
 * AD7156_CHANNEL2. Invalid channel numbers may result in
 * undefined behavior.
 * @return Returns the capacitance value in picofarads as a float.
 ******************************************************************************/
float ad7156_read_channel_capacitance(struct ad7156_dev *dev,
				      uint8_t channel);

/***************************************************************************//**
 * @brief This function is used to obtain the capacitance value in picofarads
 * (pF) from a specified channel of the AD7156 device after ensuring that
 * the conversion process is complete. It is useful in applications where
 * precise capacitance measurements are required, and the function
 * handles the conversion of raw data to a meaningful capacitance value.
 * The function should be called when the device is properly initialized
 * and configured, and the channel parameter must be valid. The function
 * clamps the raw data to a specific range before conversion to ensure
 * accurate results.
 *
 * @param dev A pointer to an initialized ad7156_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to read from, typically 1 or 2. Invalid
 * channel numbers may result in undefined behavior.
 * @return Returns the capacitance value of the specified channel in picofarads
 * (pF) as a float.
 ******************************************************************************/
float ad7156_wait_read_channel_capacitance(struct ad7156_dev *dev,
		uint8_t channel);

#endif	/* __AD7156_H__ */

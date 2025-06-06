/***************************************************************************//**
 *   @file   ad7091r5.h
 *   @brief  Header file for ad7091r5 Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
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

#ifndef SRC_AD7091R5_H_
#define SRC_AD7091R5_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include "no_os_i2c.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7091R5_CHANNEL_NO		4
#define AD7091R5_BITS			12
#define AD7091R5_CONV_MASK		NO_OS_GENMASK(AD7091R5_BITS - 1, 0)

/* AD7091r5 registers */
#define AD7091R5_REG_RESULT		0
#define AD7091R5_REG_CHANNEL		1
#define AD7091R5_REG_CONF		2
#define AD7091R5_REG_ALERT		3
#define AD7091R5_REG_CH_LOW_LIMIT(ch)	((ch) * 3 + 4)
#define AD7091R5_REG_CH_HIGH_LIMIT(ch)	((ch) * 3 + 5)
#define AD7091R5_REG_CH_HYSTERESIS(ch)	((ch) * 3 + 6)

/* AD7091R5_REG_RESULT */
#define REG_RESULT_CH_ID(x)		(((x) >> 13) & 0x3)
#define REG_RESULT_CONV_DATA(x)		((x) & AD7091R5_CONV_MASK)

/* AD7091R5_REG_CONF */
#define REG_CONF_SLEEP_MODE_MASK	NO_OS_BIT(0)
#define REG_CONF_SLEEP_MODE(x)		((x & 0x03) << 0)

#define REG_CONF_GPO1_MASK		NO_OS_BIT(2)
#define REG_CONF_GPO1(x)		((x & 0x01) << 2)

#define REG_CONF_GPO0_MASK		NO_OS_BIT(3)
#define REG_CONF_GPO0(x)		((x & 0x01) << 3)

#define REG_CONF_GPO0_MODE_MASK		(NO_OS_BIT(15) | NO_OS_BIT(5) | NO_OS_BIT(4))
#define REG_CONF_GPO0_ALERT(x)		((x & 0x01) << 4)
#define REG_CONF_GPO0_BUSY(x)		((x & 0x01) << 5)
#define REG_CONF_GPO0_DRIVE_TYPE(x)	((x & 0x01) << 15)

#define REG_CONF_CYCLE_TIMER_MASK	(NO_OS_BIT(7) | NO_OS_BIT(6))
#define REG_CONF_CYCLE_TIMER(x)		((x & 0x03) << 6)

#define REG_CONF_GPO2_MASK		NO_OS_BIT(14)
#define REG_CONF_GPO2(x)		((x & 0x01) << 14)

#define REG_CONF_RESET_MASK		NO_OS_BIT(9)
#define REG_CONF_RESET(x)		((x & 0x01) << 9)

#define REG_CONF_MODE_MASK		(NO_OS_BIT(10) | NO_OS_BIT(8))
#define REG_CONF_AUTO(x)		((x & 0x01) << 8)
#define REG_CONF_CMD(x)			((x & 0x01) << 10)

/* AD7091R5_REG_ALERT */
#define REG_ALERT_MASK(x, ch)		(x >> (ch * 2))

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ad7091r5_mode` enumeration defines the different operational
 * modes for the AD7091R5 device, which is an analog-to-digital
 * converter. These modes determine how and when the device initiates
 * conversion processes, allowing for flexibility in how data is sampled
 * and processed. The modes include a sample mode triggered by an
 * external signal, a command mode triggered by a clock signal, and an
 * automatic cycling mode for continuous conversion.
 *
 * @param AD7091R5_MODE_SAMPLE Sample mode where conversion is initiated by the
 * CONVST signal.
 * @param AD7091R5_MODE_COMMAND Command mode where conversion starts on the
 * first positive edge of the SCL signal.
 * @param AD7091R5_MODE_AUTOCYCLE Sample mode where conversion occurs
 * continuously.
 ******************************************************************************/
enum ad7091r5_mode {
	/** Sample mode, conversion started by CONVST */
	AD7091R5_MODE_SAMPLE,
	/** Command mode, conversion starts on the first pos edge of SCL */
	AD7091R5_MODE_COMMAND,
	/** Sample mode, convert continuously */
	AD7091R5_MODE_AUTOCYCLE,
};

/***************************************************************************//**
 * @brief The `ad7091r5_sleep_mode` enumeration defines the various sleep modes
 * available for the AD7091R5 device, which is a converter. Each mode
 * specifies whether the sleep mode and internal reference are turned on
 * or off, allowing for different power-saving configurations and
 * operational states of the device.
 *
 * @param AD7091R5_SLEEP_MODE_0 Sleep mode Off, Internal reference Off.
 * @param AD7091R5_SLEEP_MODE_1 Sleep mode Off, Internal reference On.
 * @param AD7091R5_SLEEP_MODE_2 Sleep mode On, Internal reference Off.
 * @param AD7091R5_SLEEP_MODE_3 Sleep mode On, Internal reference On.
 ******************************************************************************/
enum ad7091r5_sleep_mode {
	/** Default operation:
	 * Sleep mode Off, Internal reference Off */
	AD7091R5_SLEEP_MODE_0,
	/** Sleep mode Off, Internal reference On */
	AD7091R5_SLEEP_MODE_1,
	/** Sleep mode On, Internal reference Off */
	AD7091R5_SLEEP_MODE_2,
	/** Sleep mode On, Internal reference On */
	AD7091R5_SLEEP_MODE_3,
};

/***************************************************************************//**
 * @brief The `ad7091r5_port` enumeration defines the general-purpose output
 * ports available on the AD7091R5 device. These ports can be used for
 * various output functions, such as signaling or controlling external
 * components, and are identified as GPO0, GPO1, and GPO2.
 *
 * @param AD7091R5_GPO0 Represents the general-purpose output port 0.
 * @param AD7091R5_GPO1 Represents the general-purpose output port 1.
 * @param AD7091R5_GPO2 Represents the general-purpose output port 2.
 ******************************************************************************/
enum ad7091r5_port {
	/** GPO0 */
	AD7091R5_GPO0,
	/** GPO1 */
	AD7091R5_GPO1,
	/** GPO2 */
	AD7091R5_GPO2,
};

/***************************************************************************//**
 * @brief The `ad7091r5_gpo0_mode` enumeration defines the possible
 * configurations for the GPO0 port of the AD7091R5 device. It allows the
 * port to be set as a general output, an alert indicator, or a busy
 * indicator, providing flexibility in how the port can be used to signal
 * different states or conditions of the device.
 *
 * @param AD7091R5_GPO0_ENABLED GPO0 is configured as an output port.
 * @param AD7091R5_GPO0_ALERT GPO0 is configured as an alert indicator.
 * @param AD7091R5_GPO0_BUSY GPO0 is configured as a busy indicator, indicating
 * the device is converting.
 ******************************************************************************/
enum ad7091r5_gpo0_mode {
	/** GPO0 is output port */
	AD7091R5_GPO0_ENABLED,
	/** GPO0 is Alert indicator */
	AD7091R5_GPO0_ALERT,
	/** GPO0 is busy indicator, device is converting */
	AD7091R5_GPO0_BUSY,
};

/***************************************************************************//**
 * @brief The `ad7091r5_cycle_timer` enumeration defines the possible cycle
 * timer intervals for the AD7091R5 device's autocycle mode, allowing the
 * user to select different time intervals (100, 200, 400, or 800
 * microseconds) for automatic cycling of the device's operations.
 *
 * @param AD7091R5_CYCLE_TIMER_100uS Represents a cycle timer interval of 100
 * microseconds.
 * @param AD7091R5_CYCLE_TIMER_200uS Represents a cycle timer interval of 200
 * microseconds.
 * @param AD7091R5_CYCLE_TIMER_400uS Represents a cycle timer interval of 400
 * microseconds.
 * @param AD7091R5_CYCLE_TIMER_800uS Represents a cycle timer interval of 800
 * microseconds.
 ******************************************************************************/
enum ad7091r5_cycle_timer {
	/** 100 uS */
	AD7091R5_CYCLE_TIMER_100uS,
	/** 200 uS */
	AD7091R5_CYCLE_TIMER_200uS,
	/** 400 uS */
	AD7091R5_CYCLE_TIMER_400uS,
	/** 800 uS */
	AD7091R5_CYCLE_TIMER_800uS,
};

/***************************************************************************//**
 * @brief The `ad7091r5_alert_type` enumeration defines the possible alert
 * states for the AD7091R5 device, which include no alert, high alert,
 * and low alert. This enumeration is used to represent the alert status
 * of the device, allowing the system to respond appropriately to
 * different alert conditions.
 *
 * @param AD7091R5_NO_ALERT Represents a state where there is no alert.
 * @param AD7091R5_HIGH_ALERT Indicates a high alert condition.
 * @param AD7091R5_LOW_ALERT Indicates a low alert condition.
 ******************************************************************************/
enum ad7091r5_alert_type {
	/** No alert */
	AD7091R5_NO_ALERT,
	/** High alert */
	AD7091R5_HIGH_ALERT,
	/** Low alert */
	AD7091R5_LOW_ALERT,
};

/***************************************************************************//**
 * @brief The `ad7091r5_limit` enumeration defines the types of limits that can
 * be set for the AD7091R5 device, including low limit, high limit, and
 * hysteresis. These limits are used to configure the device's behavior
 * in response to input signals, allowing for precise control over its
 * operation.
 *
 * @param AD7091R5_LOW_LIMIT Represents the low limit for the AD7091R5 device.
 * @param AD7091R5_HIGH_LIMIT Represents the high limit for the AD7091R5 device.
 * @param AD7091R5_HYSTERESIS Represents the hysteresis value for the AD7091R5
 * device.
 ******************************************************************************/
enum ad7091r5_limit {
	/** Low limit */
	AD7091R5_LOW_LIMIT,
	/** High limit */
	AD7091R5_HIGH_LIMIT,
	/** Hysteresis */
	AD7091R5_HYSTERESIS,
};

/***************************************************************************//**
 * @brief The `ad7091r5_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD7091R5 device.
 * It includes pointers to structures that define the initialization
 * parameters for the I2C communication interface and the GPIO used for
 * the device's RESET functionality. This structure is essential for
 * configuring the device before it can be used for data acquisition.
 *
 * @param i2c_init Pointer to an I2C initialization parameter structure.
 * @param gpio_resetn Pointer to a GPIO initialization parameter structure for
 * the RESET pin.
 ******************************************************************************/
struct ad7091r5_init_param {
	/* I2C */
	struct no_os_i2c_init_param		*i2c_init;
	/** RESET GPIO initialization structure. */
	struct no_os_gpio_init_param	*gpio_resetn;
};

/***************************************************************************//**
 * @brief The `ad7091r5_dev` structure represents a device instance for the
 * AD7091R5, an analog-to-digital converter. It contains pointers to
 * descriptors for I2C communication and GPIO handling, specifically for
 * the RESET functionality, enabling the device to interface with other
 * hardware components through these protocols.
 *
 * @param i2c_desc Pointer to an I2C descriptor for communication.
 * @param gpio_resetn Pointer to a GPIO descriptor for handling the RESET pin.
 ******************************************************************************/
struct ad7091r5_dev {
	/* I2C descriptor */
	struct no_os_i2c_desc 	*i2c_desc;
	/** RESET GPIO handler. */
	struct no_os_gpio_desc	*gpio_resetn;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the AD7091R5 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function configures the device to
 * operate in command mode by default. If the initialization fails at any
 * step, it cleans up any allocated resources and returns an error code.
 * Ensure that valid pointers are provided for both the device and
 * initialization parameters to avoid undefined behavior.
 *
 * @param device A pointer to a pointer of type struct ad7091r5_dev. This must
 * not be null, as it will be allocated and initialized by the
 * function. The caller will own the allocated memory upon
 * successful initialization.
 * @param init_param A pointer to a struct ad7091r5_init_param containing the
 * initialization parameters. This must not be null and should
 * be properly initialized with valid I2C and GPIO parameters
 * before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid arguments, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int32_t ad7091r5_init(struct ad7091r5_dev **device,
		      struct ad7091r5_init_param *init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * allocated for an AD7091R5 device when it is no longer needed. It
 * ensures that the I2C and GPIO resources are freed and the memory
 * allocated for the device structure is released. This function must be
 * called only after the device has been successfully initialized. If the
 * provided device pointer is null, the function will return an error
 * code indicating invalid input.
 *
 * @param dev A pointer to an ad7091r5_dev structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on successful removal of the device and its resources. If
 * an error occurs during the removal of I2C or GPIO resources, a
 * negative error code is returned.
 ******************************************************************************/
int32_t ad7091r5_remove(struct ad7091r5_dev *dev);

/* Set device mode */
/***************************************************************************//**
 * @brief This function configures the AD7091R5 device to operate in one of the
 * specified modes: sample, command, or autocycle. It must be called with
 * a valid device structure and a mode from the predefined enumeration.
 * The function is typically used to switch the device's operational mode
 * based on application requirements. It is important to ensure that the
 * device has been properly initialized before calling this function. If
 * an invalid mode is provided, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns
 * -EINVAL.
 * @param mode An enumeration value of type ad7091r5_mode indicating the desired
 * operating mode. Valid values are AD7091R5_MODE_SAMPLE,
 * AD7091R5_MODE_COMMAND, and AD7091R5_MODE_AUTOCYCLE. If an invalid
 * mode is provided, the function returns -EINVAL.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad7091r5_set_mode(struct ad7091r5_dev *dev,
			  enum ad7091r5_mode mode);

/* Set device sleep mode */
int32_t ad7091r5_sleep_mode(struct ad7091r5_dev *dev,
			    enum ad7091r5_sleep_mode mode);

/* Set device set port value */
/***************************************************************************//**
 * @brief This function is used to set the value of a specified general-purpose
 * output (GPO) port on the AD7091R5 device. It should be called when you
 * need to control the state of one of the GPO ports (GPO0, GPO1, or
 * GPO2). The function requires a valid device structure and a port
 * identifier. If the device structure is null or the port identifier is
 * invalid, the function returns an error. This function is typically
 * used in applications where the GPO ports are used for signaling or
 * control purposes.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param port An enum value of type ad7091r5_port specifying which GPO port to
 * set. Valid values are AD7091R5_GPO0, AD7091R5_GPO1, and
 * AD7091R5_GPO2. If an invalid port is specified, the function
 * returns an error.
 * @param value A boolean value indicating the desired state of the specified
 * GPO port. True sets the port high, and false sets it low.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad7091r5_set_port(struct ad7091r5_dev *dev,
			  enum ad7091r5_port port, bool value);

/* Set device set GPO0 mode */
/***************************************************************************//**
 * @brief This function sets the mode and drive type for the GPO0 pin of the
 * AD7091R5 device. It should be called when you need to configure the
 * GPO0 pin to function as an output port, an alert indicator, or a busy
 * indicator. The function requires a valid device structure and a mode
 * from the predefined GPO0 modes. It also allows specifying whether the
 * output should be CMOS or open-drain. The function must be called with
 * a properly initialized device structure, and it returns an error code
 * if the device is not valid or if an invalid mode is provided.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns
 * -EINVAL.
 * @param mode An enum value of type ad7091r5_gpo0_mode specifying the desired
 * mode for the GPO0 pin. Valid values are AD7091R5_GPO0_ENABLED,
 * AD7091R5_GPO0_ALERT, and AD7091R5_GPO0_BUSY. If an invalid mode
 * is provided, the function returns -EINVAL.
 * @param is_cmos A boolean indicating whether the GPO0 pin should be configured
 * as CMOS (true) or open-drain (false).
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int32_t ad7091r5_set_gpo0_mode(struct ad7091r5_dev *dev,
			       enum ad7091r5_gpo0_mode mode,
			       bool is_cmos);
/* Set cycle timer for autocycle mode*/
/***************************************************************************//**
 * @brief This function configures the cycle timer for the AD7091R5 device when
 * operating in autocycle mode. It should be called after the device has
 * been initialized and is particularly useful for setting the timing
 * interval between automatic conversions. The function requires a valid
 * device structure and a specified cycle timer value. If the device
 * structure is null, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns
 * -EINVAL.
 * @param timer An enum value of type ad7091r5_cycle_timer specifying the
 * desired cycle timer interval. Valid values are
 * AD7091R5_CYCLE_TIMER_100uS, AD7091R5_CYCLE_TIMER_200uS,
 * AD7091R5_CYCLE_TIMER_400uS, and AD7091R5_CYCLE_TIMER_800uS.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int32_t ad7091r5_set_cycle_timer(struct ad7091r5_dev *dev,
				 enum ad7091r5_cycle_timer timer);

/* Set high limit, low limit, hysteresis. */
/***************************************************************************//**
 * @brief This function is used to configure a specific limit (low, high, or
 * hysteresis) for a given channel on the AD7091R5 device. It should be
 * called when you need to set threshold values for monitoring or
 * controlling the behavior of the device. The function requires a valid
 * device structure and a channel number within the supported range. If
 * the device pointer is null or the limit type is invalid, the function
 * returns an error code. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure. Must not be
 * null. The caller retains ownership.
 * @param limit An enum value of type ad7091r5_limit indicating which limit to
 * set (low, high, or hysteresis). Must be a valid enum value.
 * @param channel A uint8_t representing the channel number. Must be within the
 * valid range of channels supported by the device.
 * @param value A uint16_t representing the limit value to set. The value is
 * masked to fit within the device's conversion data range.
 * @return Returns an int32_t status code. Returns 0 on success or a negative
 * error code on failure, such as -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad7091r5_set_limit(struct ad7091r5_dev *dev,
			   enum ad7091r5_limit limit,
			   uint8_t channel,
			   uint16_t value);

/* Get alert. */
/***************************************************************************//**
 * @brief Use this function to check the alert status of a specific channel on
 * the AD7091R5 device. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * requires a valid channel number within the range of available channels
 * and a non-null pointer to store the alert status. If the device
 * pointer or alert pointer is null, or if the channel number is out of
 * range, the function will return an error code. This function is useful
 * for monitoring the alert conditions of the device channels.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null.
 * @param channel The channel number for which the alert status is to be
 * retrieved. Must be less than AD7091R5_CHANNEL_NO.
 * @param alert A pointer to an ad7091r5_alert_type variable where the alert
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer,
 * alert pointer is null, or the channel number is invalid.
 ******************************************************************************/
int32_t ad7091r5_get_alert(struct ad7091r5_dev *dev,
			   uint8_t channel,
			   enum ad7091r5_alert_type *alert);

/* Get high limit, low limit, hysteresis. */
/***************************************************************************//**
 * @brief Use this function to obtain the current setting of a specified limit
 * (low, high, or hysteresis) for a particular channel on the AD7091R5
 * device. This function requires a valid device structure and a pointer
 * to store the retrieved limit value. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function will return an error if the device or value pointer is null,
 * or if an invalid limit type is specified.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null.
 * @param limit An enum ad7091r5_limit value indicating which limit to retrieve
 * (low, high, or hysteresis). Must be a valid limit type.
 * @param channel A uint8_t representing the channel number for which the limit
 * is to be retrieved. Valid channel numbers depend on the device
 * configuration.
 * @param value A pointer to a uint16_t where the retrieved limit value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * (e.g., invalid parameters or I2C communication failure).
 ******************************************************************************/
int32_t ad7091r5_get_limit(struct ad7091r5_dev *dev,
			   enum ad7091r5_limit limit,
			   uint8_t channel,
			   uint16_t *value);

/* Select device channel. */
/***************************************************************************//**
 * @brief Use this function to set the active channel on the AD7091R5 device,
 * which must be done before initiating a conversion on a specific
 * channel. This function should be called only after the device has been
 * successfully initialized. It ensures that the channel number is within
 * the valid range and returns an error if the device pointer is null or
 * the channel number is invalid. The function also accounts for a
 * latency of one conversion before the channel conversion sequence is
 * updated.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param channel The channel number to be selected, ranging from 0 to
 * AD7091R5_CHANNEL_NO - 1. If the channel number is out of this
 * range, the function returns an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad7091r5_set_channel(struct ad7091r5_dev *dev,
			     uint8_t channel);

/* Read one sample. */
/***************************************************************************//**
 * @brief Use this function to read a single conversion result from a specified
 * channel of the AD7091R5 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function will attempt to set the specified channel and then read the
 * conversion result. If the channel ID in the result does not match the
 * requested channel, an error is returned. This function is useful for
 * obtaining a single data point from the ADC for a specific channel.
 *
 * @param dev A pointer to an initialized ad7091r5_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param channel The channel number to read from. Valid values are from 0 to
 * AD7091R5_CHANNEL_NO - 1. If the channel is out of range, the
 * function returns an error.
 * @param read_val A pointer to a uint16_t where the read conversion value will
 * be stored. Must not be null. If null, the function returns an
 * error.
 * @return Returns 0 on success, a negative error code on failure, or -1 if the
 * channel ID in the result does not match the requested channel.
 ******************************************************************************/
int32_t ad7091r5_read_one(struct ad7091r5_dev *dev,
			  uint8_t channel,
			  uint16_t *read_val);

#endif /* SRC_AD7091R5_H_ */

/***************************************************************************//**
 *   @file   ad7091r8.h
 *   @brief  Implementation of AD7091R-8 driver header file.
 *   @author Marcelo Schmitt (marcelo.schmitt@analog.com)
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

#ifndef __AD7091R8_H__
#define __AD7091R8_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>

#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7091R_NUM_CHANNELS(id)	(1 << ((id) + 1))
#define AD7091R8_BITS			12

#define AD7091R8_CONV_MASK		NO_OS_GENMASK(AD7091R8_BITS - 1, 0)

/* AD7091r8 registers */
#define AD7091R8_REG_RESULT		0x00
#define AD7091R8_REG_CHANNEL		0x01
#define AD7091R8_REG_CONF		0x02
#define AD7091R8_REG_ALERT		0x03
#define AD7091R8_REG_CH_LOW_LIMIT(ch)	((ch) * 3 + 4)
#define AD7091R8_REG_CH_HIGH_LIMIT(ch)	((ch) * 3 + 5)
#define AD7091R8_REG_CH_HYSTERESIS(ch)	((ch) * 3 + 6)

/* AD7091R8_REG_RESULT */
#define AD7091R8_REG_RESULT_DATA_MASK	NO_OS_GENMASK(11, 0)
#define AD7091R8_REG_RESULT_ALT_MASK	NO_OS_BIT(12)
#define AD7091R8_REG_RESULT_CH_ID_MASK	NO_OS_GENMASK(15, 13)

/* AD7091R8_REG_CONF */
#define REG_CONF_SLEEP_MODE_MASK	NO_OS_GENMASK(1, 0)
#define REG_CONF_GPO1_MASK		NO_OS_BIT(2)
#define REG_CONF_GPO0_MASK		NO_OS_BIT(3)
#define REG_CONF_GPO0_MODE_MASK		NO_OS_GENMASK(6, 4)
#define REG_CONF_ALERT_STICKY_MASK	NO_OS_BIT(7)
#define REG_CONF_RESET_MASK		NO_OS_BIT(9)

/* AD7091R8_REG_ALERT */
#define REG_ALERT_MASK(x, ch)		(x >> (ch * 2))

/* AD7091R8 read/write protocol masks */
#define AD7091R8_REG_DATA_MSK		NO_OS_GENMASK(9, 0)
#define AD7091R8_RD_WR_FLAG_MSK		NO_OS_BIT(10)
#define AD7091R8_REG_ADDR_MSK		NO_OS_GENMASK(15, 11)

/*****************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7091r8_device_id` enumeration defines identifiers for different
 * variants of the AD7091R series of devices, specifically the AD7091R-2,
 * AD7091R-4, and AD7091R-8. These identifiers are used to distinguish
 * between the different models within the AD7091R series, which may have
 * varying features or capabilities. This enumeration is crucial for
 * configuring and managing device-specific operations in the AD7091R-8
 * driver.
 *
 * @param AD7091R2 Represents the AD7091R-2 device variant.
 * @param AD7091R4 Represents the AD7091R-4 device variant.
 * @param AD7091R8 Represents the AD7091R-8 device variant.
 ******************************************************************************/
enum ad7091r8_device_id {
	AD7091R2,
	AD7091R4,
	AD7091R8,
};

/***************************************************************************//**
 * @brief The `ad7091r8_names` variable is a static array of constant character
 * pointers, which stores the names of different AD7091R devices. Each
 * element in the array corresponds to a specific device ID defined in
 * the `ad7091r8_device_id` enumeration.
 *
 * @details This variable is used to map device IDs to their respective string
 * names for identification and display purposes.
 ******************************************************************************/
static char * const ad7091r8_names[] = {
	[AD7091R2] = "ad7091r-2",
	[AD7091R4] = "ad7091r-4",
	[AD7091R8] = "ad7091r-8",
};

/***************************************************************************//**
 * @brief The `ad7091r8_sleep_mode` enumeration defines the various sleep modes
 * supported by the AD7091R-8 device, which is a type of analog-to-
 * digital converter. Each enumerator specifies a combination of the
 * sleep mode and the state of the internal reference, allowing for
 * different power-saving configurations and operational states of the
 * device.
 *
 * @param AD7091R8_SLEEP_MODE_0 Represents the default operation with sleep mode
 * off and internal reference off.
 * @param AD7091R8_SLEEP_MODE_1 Represents sleep mode off with internal
 * reference on.
 * @param AD7091R8_SLEEP_MODE_2 Represents sleep mode on with internal reference
 * off.
 * @param AD7091R8_SLEEP_MODE_3 Represents sleep mode on with internal reference
 * on.
 ******************************************************************************/
enum ad7091r8_sleep_mode {
	/** Default operation:
	 * Sleep mode Off, Internal reference Off */
	AD7091R8_SLEEP_MODE_0,
	/** Sleep mode Off, Internal reference On */
	AD7091R8_SLEEP_MODE_1,
	/** Sleep mode On, Internal reference Off */
	AD7091R8_SLEEP_MODE_2,
	/** Sleep mode On, Internal reference On */
	AD7091R8_SLEEP_MODE_3,
};

/***************************************************************************//**
 * @brief The `ad7091r8_port` enumeration defines the general-purpose output
 * ports available in the AD7091R-8 device. These ports, GPO0 and GPO1,
 * are used for various output functionalities, such as signaling or
 * control, in the context of the AD7091R-8 analog-to-digital converter.
 * The enumeration provides a clear and concise way to reference these
 * ports within the software, facilitating easier configuration and
 * management of the device's output capabilities.
 *
 * @param AD7091R8_GPO0 Represents the general-purpose output 0.
 * @param AD7091R8_GPO1 Represents the general-purpose output 1.
 ******************************************************************************/
enum ad7091r8_port {
	/** GPO0 */
	AD7091R8_GPO0,
	/** GPO1 */
	AD7091R8_GPO1,
};

/***************************************************************************//**
 * @brief The `ad7091r8_gpo0_mode` enumeration defines the possible
 * configurations for the GPO0 port on the AD7091R-8 device. It allows
 * the port to be set as a general output, an alert indicator, or a busy
 * indicator, providing flexibility in how the port can be used to signal
 * different states or conditions of the device.
 *
 * @param AD7091R8_GPO0_ENABLED GPO0 is configured as an output port.
 * @param AD7091R8_GPO0_ALERT GPO0 is configured as an alert indicator.
 * @param AD7091R8_GPO0_BUSY GPO0 is configured as a busy indicator, indicating
 * the device is converting.
 ******************************************************************************/
enum ad7091r8_gpo0_mode {
	/** GPO0 is output port */
	AD7091R8_GPO0_ENABLED = 0,
	/** GPO0 is Alert indicator */
	AD7091R8_GPO0_ALERT = 16,
	/** GPO0 is busy indicator, device is converting */
	AD7091R8_GPO0_BUSY = 48,
};

/***************************************************************************//**
 * @brief The `ad7091r8_limit` enumeration defines the types of limits that can
 * be set for the AD7091R-8 device, which include low limit, high limit,
 * and hysteresis. These limits are used to configure the threshold
 * values for the device's operation, allowing it to trigger alerts or
 * adjust its behavior based on the input signal levels.
 *
 * @param AD7091R8_LOW_LIMIT Represents the low limit threshold for the
 * AD7091R-8 device.
 * @param AD7091R8_HIGH_LIMIT Represents the high limit threshold for the
 * AD7091R-8 device.
 * @param AD7091R8_HYSTERESIS Represents the hysteresis value for the AD7091R-8
 * device.
 ******************************************************************************/
enum ad7091r8_limit {
	/** Low limit */
	AD7091R8_LOW_LIMIT,
	/** High limit */
	AD7091R8_HIGH_LIMIT,
	/** Hysteresis */
	AD7091R8_HYSTERESIS,
};

/***************************************************************************//**
 * @brief The `ad7091r8_alert_type` enumeration defines the possible alert
 * states for the AD7091R-8 device, which include no alert, high alert,
 * and low alert. This enumeration is used to represent the alert status
 * of the device, allowing the system to respond appropriately to
 * different alert conditions.
 *
 * @param AD7091R8_NO_ALERT Represents a state where there is no alert.
 * @param AD7091R8_HIGH_ALERT Indicates a high alert condition.
 * @param AD7091R8_LOW_ALERT Indicates a low alert condition.
 ******************************************************************************/
enum ad7091r8_alert_type {
	/** No alert */
	AD7091R8_NO_ALERT,
	/** High alert */
	AD7091R8_HIGH_ALERT,
	/** Low alert */
	AD7091R8_LOW_ALERT,
};

/***************************************************************************//**
 * @brief The `ad7091r8_dev` structure is a data structure used to represent and
 * manage the state of an AD7091R-8 device, which is a type of analog-to-
 * digital converter. It includes members for handling SPI communication,
 * GPIO control for various pins (CONVST, RESET, ALERT), and
 * configuration settings such as the reference voltage, device
 * identifier, and sleep mode. This structure is essential for
 * interfacing with the AD7091R-8 device, allowing for configuration and
 * control of its operation through software.
 *
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 * @param vref_mv An integer representing the reference voltage in millivolts.
 * @param gpio_convst A pointer to a GPIO descriptor for the CONVST pin.
 * @param gpio_reset A pointer to a GPIO descriptor for the RESET pin.
 * @param gpio_alert A pointer to a GPIO descriptor for the ALERT pin.
 * @param device_id An enumeration value identifying the specific AD7091R device
 * variant.
 * @param sleep_mode An enumeration value representing the sleep mode of the
 * AD7091R device.
 ******************************************************************************/
struct ad7091r8_dev {
	/** SPI descriptor **/
	//spi_desc *spi_desc;
	struct no_os_spi_desc *spi_desc;
	/* Reference voltage */
	int vref_mv;
	/* CONVST GPIO handler */
	struct no_os_gpio_desc *gpio_convst;
	/** RESET GPIO handler. */
	struct no_os_gpio_desc	*gpio_reset;
	/** ALERT GPIO handler. */
	struct no_os_gpio_desc	*gpio_alert;
	/* AD7091R specific device identifier */
	enum ad7091r8_device_id device_id;
	/* AD7091R device sleep mode */
	enum ad7091r8_sleep_mode sleep_mode;
};

/***************************************************************************//**
 * @brief The `ad7091r8_init_param` structure is used to initialize the
 * AD7091R-8 device, encapsulating all necessary parameters for setting
 * up the device. It includes pointers to SPI and GPIO initialization
 * parameters, an external voltage reference, and enumerations for device
 * identification and sleep mode configuration. This structure is
 * essential for configuring the device's communication and operational
 * settings before use.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param vref_mv External voltage reference in millivolts.
 * @param gpio_convst Pointer to CONVST GPIO initialization parameters.
 * @param gpio_reset Pointer to Reset GPIO initialization parameters.
 * @param gpio_alert Pointer to Alert GPIO initialization parameters.
 * @param device_id AD7091R specific device identifier.
 * @param sleep_mode AD7091R device sleep mode.
 ******************************************************************************/
struct ad7091r8_init_param {
	/* SPI initialization parameters */
	struct no_os_spi_init_param *spi_init;
	/* External Voltage Reference */
	int vref_mv;
	/* CONVST GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_convst;
	/* Reset GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_reset;
	/* Alert GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_alert;
	/* AD7091R specific device identifier */
	enum ad7091r8_device_id device_id;
	/* AD7091R device sleep mode */
	enum ad7091r8_sleep_mode sleep_mode;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD7091R-8 device using the provided
 * initialization parameters, setting up the necessary SPI and GPIO
 * configurations. It must be called before any other operations on the
 * device. The function allocates memory for the device structure and
 * configures the device according to the specified parameters, including
 * setting the sleep mode and reference voltage. If initialization fails
 * at any step, it cleans up any allocated resources and returns an error
 * code.
 *
 * @param device A pointer to a pointer of type `struct ad7091r8_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad7091r8_init_param` containing
 * initialization parameters such as SPI and GPIO
 * configurations, device ID, sleep mode, and reference
 * voltage. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no resources are left allocated.
 ******************************************************************************/
int ad7091r8_init(struct ad7091r8_dev **device,
		  struct ad7091r8_init_param *init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD7091R-8 device instance when it is no longer
 * needed. It ensures that all GPIO and SPI resources are freed and the
 * device structure is deallocated. This function must be called after
 * the device has been initialized and used, to prevent resource leaks.
 * It is important to ensure that the `dev` parameter is not null before
 * calling this function, as passing a null pointer will result in
 * undefined behavior.
 *
 * @param dev A pointer to an `ad7091r8_dev` structure representing the device
 * to be removed. This pointer must not be null, and the caller
 * retains ownership of the pointer itself, but the resources it
 * points to will be freed.
 * @return Returns 0 on success, or a negative error code if any of the resource
 * removals fail.
 ******************************************************************************/
int ad7091r8_remove(struct ad7091r8_dev *dev);

/* Set device sleep mode */
/***************************************************************************//**
 * @brief This function configures the sleep mode of the AD7091R-8 device to one
 * of the predefined modes. It should be called when the device is
 * operational and properly initialized. The function updates the
 * device's configuration register to reflect the desired sleep mode. It
 * is important to ensure that the device pointer is valid before calling
 * this function, as passing a null pointer will result in an error. The
 * function returns an error code if the operation fails, allowing the
 * caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enumerated value of type ad7091r8_sleep_mode indicating the
 * desired sleep mode. Valid values are AD7091R8_SLEEP_MODE_0,
 * AD7091R8_SLEEP_MODE_1, AD7091R8_SLEEP_MODE_2, and
 * AD7091R8_SLEEP_MODE_3.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad7091r8_set_sleep_mode(struct ad7091r8_dev *dev,
			    enum ad7091r8_sleep_mode mode);

/* Set device set port value */
/***************************************************************************//**
 * @brief This function is used to set the value of a specified general-purpose
 * output (GPO) port on the AD7091R-8 device. It is typically called when
 * there is a need to control the state of the GPOs, which can be used
 * for various signaling purposes. The function requires a valid device
 * structure and a port identifier, which must be one of the defined GPO
 * ports. The value parameter determines the state to set the port to,
 * where true typically represents a high state and false a low state. If
 * an invalid port is specified, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param port An enum value of type ad7091r8_port specifying which GPO port to
 * set. Valid values are AD7091R8_GPO0 and AD7091R8_GPO1.
 * @param value A boolean value indicating the desired state of the specified
 * port. True sets the port high, and false sets it low.
 * @return Returns 0 on success or a negative error code if the port is invalid
 * or if there is a communication failure.
 ******************************************************************************/
int ad7091r8_set_port(struct ad7091r8_dev *dev,
		      enum ad7091r8_port port,
		      bool value);

/* Set device set GPO0 mode */
/***************************************************************************//**
 * @brief Use this function to set the mode of the GPO0 pin on the AD7091R-8
 * device, which can be configured as an output port, an alert indicator,
 * or a busy indicator. This function should be called after the device
 * has been initialized. The mode is specified using the
 * `ad7091r8_gpo0_mode` enumeration, and the `is_cmos` parameter
 * determines whether the output is in CMOS mode. Proper configuration of
 * the GPO0 pin is essential for the intended operation of the device's
 * general-purpose output functionality.
 *
 * @param dev A pointer to an initialized `ad7091r8_dev` structure representing
 * the device. Must not be null.
 * @param mode An `ad7091r8_gpo0_mode` enumeration value specifying the desired
 * mode for the GPO0 pin. Valid values are `AD7091R8_GPO0_ENABLED`,
 * `AD7091R8_GPO0_ALERT`, and `AD7091R8_GPO0_BUSY`.
 * @param is_cmos A boolean value indicating whether the GPO0 output should be
 * in CMOS mode. If true, CMOS mode is enabled; otherwise, it is
 * not.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad7091r8_set_gpo0_mode(struct ad7091r8_dev *dev,
			   enum ad7091r8_gpo0_mode mode,
			   bool is_cmos);

/* Set high limit, low limit, hysteresis. */
/***************************************************************************//**
 * @brief This function is used to configure a specific limit (low, high, or
 * hysteresis) for a given channel on the AD7091R-8 device. It is
 * essential to call this function after initializing the device and
 * before starting any operations that depend on these limits. The
 * function requires a valid device structure, a limit type, a channel
 * number, and a limit value. If an invalid limit type is provided, the
 * function will return an error. This function is useful for setting
 * thresholds that trigger alerts or control the behavior of the device.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param limit An enum value of type ad7091r8_limit indicating the type of
 * limit to set (low, high, or hysteresis). Invalid values will
 * result in an error.
 * @param channel A uint8_t representing the channel number for which the limit
 * is being set. Must be within the valid range of channels for
 * the device.
 * @param value A uint16_t representing the limit value to set. The value is
 * masked to fit within the device's conversion mask.
 * @return Returns 0 on success or a negative error code if an invalid limit
 * type is provided.
 ******************************************************************************/
int ad7091r8_set_limit(struct ad7091r8_dev *dev,
		       enum ad7091r8_limit limit,
		       uint8_t channel,
		       uint16_t value);

/* Get alert. */
/***************************************************************************//**
 * @brief This function is used to obtain the alert status of a specific channel
 * on the AD7091R-8 device. It should be called when you need to check if
 * a channel has triggered an alert condition. The function requires a
 * valid device structure and a pointer to store the alert type. It is
 * important to ensure that the channel number is within the valid range
 * for the device, as determined by the device's ID. If the device
 * structure or alert pointer is null, or if the channel number is
 * invalid, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param channel The channel number for which the alert status is requested.
 * Must be within the valid range for the device, as determined
 * by AD7091R_NUM_CHANNELS.
 * @param alert A pointer to an enum ad7091r8_alert_type where the alert status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if there is a communication error with the
 * device.
 ******************************************************************************/
int ad7091r8_get_alert(struct ad7091r8_dev *dev,
		       uint8_t channel,
		       enum ad7091r8_alert_type *alert);

/* Get high limit, low limit, hysteresis. */
/***************************************************************************//**
 * @brief This function is used to obtain the current setting of a specified
 * limit (low limit, high limit, or hysteresis) for a particular channel
 * on the AD7091R-8 device. It is essential to call this function after
 * the device has been properly initialized and configured. The function
 * requires a valid device structure, a limit type, a channel number, and
 * a pointer to store the retrieved limit value. If the limit type is
 * invalid, the function returns an error code. This function is useful
 * for monitoring and adjusting the device's operational parameters.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param limit An enum value of type ad7091r8_limit indicating which limit to
 * retrieve (low limit, high limit, or hysteresis). Must be a valid
 * enum value.
 * @param channel A uint8_t representing the channel number for which the limit
 * is to be retrieved. Must be within the valid range of channels
 * for the device.
 * @param value A pointer to a uint16_t where the retrieved limit value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the limit type is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int ad7091r8_get_limit(struct ad7091r8_dev *dev,
		       enum ad7091r8_limit limit,
		       uint8_t channel,
		       uint16_t *value);

/* Select device channel. */
/***************************************************************************//**
 * @brief Use this function to select a specific channel on the AD7091R-8 device
 * for subsequent operations. It is essential to ensure that the device
 * is properly initialized before calling this function. The function
 * checks if the provided channel number is within the valid range for
 * the device's configuration. If the device pointer is null or the
 * channel number is invalid, the function returns an error code. This
 * function also triggers a conversion start pulse and writes the channel
 * selection to the device register.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null. The function returns an error if
 * this parameter is null.
 * @param channel An unsigned 8-bit integer representing the channel to be
 * selected. The value must be less than the number of channels
 * supported by the device, as determined by the device's
 * configuration. If the channel is out of range, the function
 * returns an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null, the channel is invalid, or if there is an error during the
 * conversion start pulse or register write operation.
 ******************************************************************************/
int ad7091r8_set_channel(struct ad7091r8_dev *dev,
			 uint8_t channel);

/* Read one sample. */
/***************************************************************************//**
 * @brief This function reads a single sample from a specified channel of the
 * AD7091R-8 device and stores the result in the provided memory
 * location. It should be called when a single channel's data is needed
 * from the device. The function requires a valid device structure and a
 * non-null pointer for storing the read value. It also checks if the
 * specified channel is within the valid range for the device. If any of
 * these conditions are not met, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param channel The channel number to read from. Must be within the valid
 * range of channels for the device, determined by the device's
 * ID.
 * @param read_val A pointer to a uint16_t where the read sample value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer,
 * read_val pointer is null, or if the channel is out of range.
 ******************************************************************************/
int ad7091r8_read_one(struct ad7091r8_dev *dev,
		      uint8_t chan,
		      uint16_t *read_val);

/* Read next channel set in the channel sequencer. */
/***************************************************************************//**
 * @brief This function retrieves the next channel value from the AD7091R-8
 * device's channel sequencer. It should be called when a sequenced read
 * is required, typically after the device has been initialized and
 * configured. The function requires a valid device structure and a
 * pointer to store the read value. It handles any necessary
 * communication with the device and returns an error code if the
 * operation fails. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @param read_val A pointer to a uint16_t variable where the read value will be
 * stored. Must not be null. The function writes the result of
 * the read operation to this location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad7091r8_sequenced_read(struct ad7091r8_dev *dev,
			    uint16_t *read_val);

/* Read device register. */
/***************************************************************************//**
 * @brief This function reads a 16-bit register from the AD7091R-8 device using
 * SPI communication. It should be called when you need to retrieve the
 * current value of a specific register on the device. The function
 * requires a valid device structure and a pointer to store the read
 * data. It handles special cases for certain registers, such as
 * triggering a conversion for the result register. Ensure that the
 * device is properly initialized before calling this function. The
 * function returns an error code if the operation fails or if invalid
 * parameters are provided.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the AD7091R-8 device.
 * @param reg_data A pointer to a uint16_t variable where the read register data
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ad7091r8_spi_reg_read(struct ad7091r8_dev *dev,
			  uint8_t reg_addr,
			  uint16_t *reg_data);

/* Write to device register. */
/***************************************************************************//**
 * @brief This function writes a 16-bit data value to a specified register
 * address on the AD7091R-8 device using the SPI interface. It is
 * essential to ensure that the device structure is properly initialized
 * and not null before calling this function. The function prepares the
 * data according to the AD7091R-8 protocol and performs a single SPI
 * transfer. It is typically used when configuring the device or updating
 * its settings. The function returns an error code if the device pointer
 * is null or if the SPI transfer fails.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param reg_addr The address of the register to which the data will be
 * written. It is a 5-bit value, as the register address is set
 * in bits B15:B11 of the protocol.
 * @param reg_data The 16-bit data to be written to the specified register. The
 * data is set in bits B9:B0 of the protocol.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device pointer is null.
 ******************************************************************************/
int ad7091r8_spi_reg_write(struct ad7091r8_dev *dev,
			   uint8_t reg_addr,
			   uint16_t reg_data);

/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function allows for writing specific bits to a register of the
 * AD7091R-8 device by applying a mask. It first reads the current value
 * of the register, applies the mask to clear specific bits, and then
 * writes the new data to those bits. This is useful for modifying only
 * certain parts of a register without affecting other bits. The function
 * should be called when the device is properly initialized and the
 * register address is valid. It returns an error code if the read or
 * write operation fails.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD7091R-8 device.
 * @param mask A 16-bit mask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The 16-bit data to be written to the masked bits of the register.
 * Only bits corresponding to the mask will be written.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int ad7091r8_spi_write_mask(struct ad7091r8_dev* dev,
			    uint8_t reg_addr,
			    uint16_t mask,
			    uint16_t data);

/***************************************************************************//**
 * @brief This function is used to trigger a conversion on the AD7091R-8 device
 * by pulsing its CONVST pin. It should be called when a conversion is
 * needed, and the device must be properly initialized before calling
 * this function. The function handles the GPIO operations required to
 * pulse the pin low and then high. If the device structure is null, the
 * function returns an error. It is important to ensure that the device
 * is in a state ready for conversion before calling this function.
 *
 * @param dev A pointer to an initialized ad7091r8_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in setting the GPIO values.
 ******************************************************************************/
int ad7091r8_pulse_convst(struct ad7091r8_dev *dev);

#endif // __AD7091R8_H__

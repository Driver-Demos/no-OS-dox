/***************************************************************************//**
 *   @file   ad5761r.h
 *   @brief  Header file of AD5761R Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2015(c) Analog Devices, Inc.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#ifndef AD5761R_H_
#define AD5761R_H_

/* Input Shift Register Format */
#define AD5761R_INPUT_ZERO_BIT		(1 << 20)
#define AD5761R_INPUT_ADDR_CMD(x)	(((x) & 0xF) << 16)
#define AD5761R_INPUT_DATA(x)		(((x) & 0xFFFF) << 0)

#define AD5761R_DATA(x)			(((x) & 0xFFFF) << 0)
#define AD5721R_DATA(x)			(((x) & 0xFFF) << 4)

/* Input Shift Register Commands */
#define CMD_NOP				0x0
#define CMD_WR_TO_INPUT_REG		0x1
#define CMD_UPDATE_DAC_REG		0x2
#define CMD_WR_UPDATE_DAC_REG		0x3
#define CMD_WR_CTRL_REG			0x4
#define CMD_NOP_ALT_1			0x5
#define CMD_NOP_ALT_2			0x6
#define CMD_SW_DATA_RESET		0x7
#define CMD_RESERVED			0x8
#define CMD_DIS_DAISY_CHAIN		0x9
#define CMD_RD_INPUT_REG		0xA
#define CMD_RD_DAC_REG			0xB
#define CMD_RD_CTRL_REG			0xC
#define CMD_NOP_ALT_3			0xD
#define CMD_NOP_ALT_4			0xE
#define CMD_SW_FULL_RESET		0xF

/* Control Register Format */
#define AD5761R_CTRL_SC			(1 << 12)		// RO
#define AD5761R_CTRL_BO			(1 << 11)		// RO
#define AD5761R_CTRL_CV(x)		(((x) & 0x3) << 9)	// RW
#define AD5761R_CTRL_OVR		(1 << 8)		// RW
#define AD5761R_CTRL_B2C		(1 << 7)		// RW
#define AD5761R_CTRL_ETS		(1 << 6)		// RW
#define AD5761R_CTRL_IRO		(1 << 5)		// RW
#define AD5761R_CTRL_PV(x)		(((x) & 0x3) << 3)	// RW
#define AD5761R_CTRL_RA(x)		(((x) & 0x7) << 0)	// RW

/* Disable Daisy-Chain Register Format */
#define AD5761R_DIS_DAISY_CHAIN_DDC(x)	(((x) & 0x1) << 0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5761r_type` enumeration defines the types of devices supported
 * by the AD5761R driver, specifically the AD5761R and AD5721R models.
 * This enumeration is used to specify the device type when initializing
 * or configuring the driver, allowing the software to handle device-
 * specific operations and settings appropriately.
 *
 * @param AD5761R Represents the AD5761R device type.
 * @param AD5721R Represents the AD5721R device type.
 ******************************************************************************/
enum ad5761r_type {
	AD5761R,
	AD5721R,
};

/***************************************************************************//**
 * @brief The `ad5761r_reg` enumeration defines symbolic names for the registers
 * of the AD5761R device, which include the input, DAC, and control
 * registers. These symbolic names are used to facilitate register access
 * and manipulation within the driver code, providing a clear and concise
 * way to reference specific registers of the AD5761R device.
 *
 * @param AD5761R_REG_INPUT Represents the input register of the AD5761R device.
 * @param AD5761R_REG_DAC Represents the DAC register of the AD5761R device.
 * @param AD5761R_REG_CTRL Represents the control register of the AD5761R
 * device.
 ******************************************************************************/
enum ad5761r_reg {
	AD5761R_REG_INPUT,
	AD5761R_REG_DAC,
	AD5761R_REG_CTRL,
};

/***************************************************************************//**
 * @brief The `ad5761r_scale` enumeration defines the possible scale settings
 * for the AD5761R device, which are used to configure the output voltage
 * scaling. The three possible values are zero, half, and full, allowing
 * for different levels of output voltage scaling based on the
 * application requirements.
 *
 * @param AD5761R_SCALE_ZERO Represents a scale setting of zero.
 * @param AD5761R_SCALE_HALF Represents a scale setting of half.
 * @param AD5761R_SCALE_FULL Represents a scale setting of full.
 ******************************************************************************/
enum ad5761r_scale {
	AD5761R_SCALE_ZERO,
	AD5761R_SCALE_HALF,
	AD5761R_SCALE_FULL,
};

/***************************************************************************//**
 * @brief The `ad5761r_range` enumeration defines a set of possible voltage
 * output ranges for the AD5761R digital-to-analog converter (DAC). Each
 * enumerator represents a specific voltage range that the DAC can be
 * configured to output, allowing for flexibility in applications
 * requiring different voltage levels. This enumeration is used to set
 * the output range of the DAC, which is crucial for ensuring the device
 * operates within the desired voltage limits for a given application.
 *
 * @param AD5761R_RANGE_M_10V_TO_P_10V Represents a voltage range from -10V to
 * +10V.
 * @param AD5761R_RANGE_0_V_TO_P_10V Represents a voltage range from 0V to +10V.
 * @param AD5761R_RANGE_M_5V_TO_P_5V Represents a voltage range from -5V to +5V.
 * @param AD5761R_RANGE_0V_TO_P_5V Represents a voltage range from 0V to +5V.
 * @param AD5761R_RANGE_M_2V5_TO_P_7V5 Represents a voltage range from -2.5V to
 * +7.5V.
 * @param AD5761R_RANGE_M_3V_TO_P_3V Represents a voltage range from -3V to +3V.
 * @param AD5761R_RANGE_0V_TO_P_16V Represents a voltage range from 0V to +16V.
 * @param AD5761R_RANGE_0V_TO_P_20V Represents a voltage range from 0V to +20V.
 ******************************************************************************/
enum ad5761r_range {
	AD5761R_RANGE_M_10V_TO_P_10V,
	AD5761R_RANGE_0_V_TO_P_10V,
	AD5761R_RANGE_M_5V_TO_P_5V,
	AD5761R_RANGE_0V_TO_P_5V,
	AD5761R_RANGE_M_2V5_TO_P_7V5,
	AD5761R_RANGE_M_3V_TO_P_3V,
	AD5761R_RANGE_0V_TO_P_16V,
	AD5761R_RANGE_0V_TO_P_20V,
};

/***************************************************************************//**
 * @brief The `ad5761r_dev` structure is a comprehensive representation of the
 * AD5761R device, encapsulating both hardware interface descriptors and
 * device-specific settings. It includes SPI and GPIO descriptors for
 * communication and control, as well as configuration parameters such as
 * device type, output range, and voltage scales. Additionally, it
 * provides boolean flags to enable or disable various features like
 * internal reference, temperature shutdown, bipolar range, overrange,
 * and daisy-chain mode, allowing for flexible and precise control of the
 * AD5761R device's operation.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_reset_value Value for the reset GPIO pin.
 * @param gpio_clr Pointer to a GPIO descriptor for the clear pin.
 * @param gpio_clr_value Value for the clear GPIO pin.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param gpio_ldac_value Value for the LDAC GPIO pin.
 * @param type Specifies the type of AD5761R device.
 * @param ra Specifies the output range of the device.
 * @param pv Specifies the power-up voltage scale.
 * @param cv Specifies the clear voltage scale.
 * @param int_ref_en Boolean to enable or disable the internal reference.
 * @param exc_temp_sd_en Boolean to enable or disable exceed temperature
 * shutdown.
 * @param b2c_range_en Boolean to enable or disable the two's complement bipolar
 * range.
 * @param ovr_en Boolean to enable or disable the overrange feature.
 * @param daisy_chain_en Boolean to enable or disable daisy-chain mode.
 ******************************************************************************/
struct ad5761r_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	uint8_t		gpio_reset_value;
	struct no_os_gpio_desc	*gpio_clr;
	uint8_t		gpio_clr_value;
	struct no_os_gpio_desc	*gpio_ldac;
	uint8_t		gpio_ldac_value;
	/* Device Settings */
	enum ad5761r_type	type;
	enum ad5761r_range	ra;
	enum ad5761r_scale	pv;
	enum ad5761r_scale	cv;
	bool		int_ref_en;
	bool		exc_temp_sd_en;
	bool		b2c_range_en;
	bool		ovr_en;
	bool		daisy_chain_en;
};

/***************************************************************************//**
 * @brief The `ad5761r_init_param` structure is used to initialize and configure
 * the AD5761R device, a digital-to-analog converter (DAC). It includes
 * parameters for SPI communication, GPIO settings for control pins
 * (reset, clear, and LDAC), and various device settings such as type,
 * output range, power-up and clear voltage scales, and several boolean
 * flags to enable or disable features like internal reference, exceed
 * temperature shutdown, two's complement range, overrange, and daisy-
 * chain mode. This structure is essential for setting up the device
 * before operation.
 *
 * @param spi_init Initializes the SPI communication parameters.
 * @param gpio_reset Defines the GPIO parameters for the reset pin.
 * @param gpio_reset_value Specifies the value to set on the reset GPIO pin.
 * @param gpio_clr Defines the GPIO parameters for the clear pin.
 * @param gpio_clr_value Specifies the value to set on the clear GPIO pin.
 * @param gpio_ldac Defines the GPIO parameters for the LDAC pin.
 * @param gpio_ldac_value Specifies the value to set on the LDAC GPIO pin.
 * @param type Specifies the type of the AD5761R device.
 * @param out_range Defines the output voltage range of the device.
 * @param pwr_voltage Specifies the power-up voltage scale.
 * @param clr_voltage Specifies the clear voltage scale.
 * @param int_ref_en Enables or disables the internal reference.
 * @param exc_temp_sd_en Enables or disables the exceed temperature shutdown
 * feature.
 * @param b2c_range_en Enables or disables the two's complement bipolar output
 * range.
 * @param ovr_en Enables or disables the 5% overrange feature.
 * @param daisy_chain_en Enables or disables the daisy-chain mode.
 ******************************************************************************/
struct ad5761r_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_reset;
	uint8_t		gpio_reset_value;
	struct no_os_gpio_init_param	gpio_clr;
	uint8_t		gpio_clr_value;
	struct no_os_gpio_init_param	gpio_ldac;
	uint8_t		gpio_ldac_value;
	/* Device Settings */
	enum ad5761r_type	type;
	enum ad5761r_range	out_range;
	enum ad5761r_scale	pwr_voltage;
	enum ad5761r_scale	clr_voltage;
	bool		int_ref_en;
	bool		exc_temp_sd_en;
	bool		b2c_range_en;
	bool		ovr_en;
	bool		daisy_chain_en;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the AD5761R device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function initializes SPI and GPIO
 * interfaces and configures device settings such as output range, power-
 * up voltage, and daisy-chain mode. If initialization fails at any step,
 * the function returns an error code and no device is created. The
 * caller is responsible for managing the memory of the device structure.
 *
 * @param device A pointer to a pointer of type `struct ad5761r_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad5761r_init_param` containing
 * initialization parameters for the device, including SPI and
 * GPIO configurations, and device-specific settings. All
 * fields must be properly initialized before calling the
 * function.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to a newly allocated and
 * initialized device structure.
 ******************************************************************************/
int32_t ad5761r_init(struct ad5761r_dev **device,
		     struct ad5761r_init_param init_param);
/* Free the resources allocated by ad5761r_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an AD5761R
 * device instance, including SPI and GPIO descriptors. It should be
 * called when the device is no longer needed to ensure proper cleanup
 * and avoid resource leaks. This function must be called after the
 * device has been initialized with `ad5761r_init`. It handles the
 * removal of SPI and GPIO resources and frees the memory allocated for
 * the device structure. The function returns a status code indicating
 * the success or failure of the resource removal process.
 *
 * @param dev A pointer to an `ad5761r_dev` structure representing the device
 * instance to be removed. This pointer must not be null, and the
 * device must have been previously initialized. The function will
 * handle null GPIO descriptors gracefully.
 * @return Returns an `int32_t` status code. A value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t ad5761r_remove(struct ad5761r_dev *dev);
/* SPI write to device. */
/***************************************************************************//**
 * @brief This function sends a command and data to the AD5761R device over SPI.
 * It is used to write to the device's registers by specifying the
 * register address and the data to be written. The function must be
 * called with a valid device structure that has been initialized and
 * configured for SPI communication. It is important to ensure that the
 * device is properly initialized before calling this function to avoid
 * communication errors.
 *
 * @param dev A pointer to an initialized `ad5761r_dev` structure representing
 * the device. This must not be null, and the SPI descriptor within
 * must be properly configured.
 * @param reg_addr_cmd A 8-bit value representing the register address or
 * command to be sent to the device. This value should be
 * within the valid range of commands supported by the
 * device.
 * @param reg_data A 16-bit value representing the data to be written to the
 * specified register. The data should be formatted according to
 * the device's requirements for the specific register.
 * @return Returns an `int32_t` status code, where 0 indicates success and a
 * negative value indicates an error during the SPI write operation.
 ******************************************************************************/
int32_t ad5761r_write(struct ad5761r_dev *dev,
		      uint8_t reg_addr_cmd,
		      uint16_t reg_data);
/* SPI read from device. */
/***************************************************************************//**
 * @brief This function reads data from a specified register of the AD5761R
 * device using SPI communication. It should be called when you need to
 * retrieve the current value of a register from the device. Ensure that
 * the device has been properly initialized before calling this function.
 * The function writes the register address command to the device and
 * reads back the data into the provided buffer. It is important to
 * handle the return value to check for any communication errors.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param reg_addr_cmd A uint8_t value representing the register address command
 * to be read from. Must be a valid command as defined by
 * the device's protocol.
 * @param reg_data A pointer to a uint16_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5761r_read(struct ad5761r_dev *dev,
		     uint8_t reg_addr_cmd,
		     uint16_t *reg_data);
/* Readback the register data. */
/***************************************************************************//**
 * @brief Use this function to read data from a specific register of the AD5761R
 * device when daisy-chain mode is enabled. It is essential to ensure
 * that the device is configured with daisy-chain mode enabled before
 * calling this function, as the readback operation is only supported in
 * this mode. The function reads the specified register and returns the
 * data through the provided pointer. If the daisy-chain mode is not
 * enabled or an invalid register is specified, the function returns an
 * error code.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null and must have daisy-chain mode enabled.
 * @param reg An enum value of type ad5761r_reg specifying the register to read
 * from. Valid values are AD5761R_REG_INPUT, AD5761R_REG_DAC, and
 * AD5761R_REG_CTRL. Invalid values result in an error.
 * @param reg_data A pointer to a uint16_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if daisy-chain mode is disabled or an
 * invalid register is specified.
 ******************************************************************************/
int32_t ad5761r_register_readback(struct ad5761r_dev *dev,
				  enum ad5761r_reg reg,
				  uint16_t *reg_data);
/* Configure the part based on the settings stored in the device structure. */
/***************************************************************************//**
 * @brief This function configures the AD5761R device using the settings
 * specified in the provided device structure. It should be called after
 * the device has been initialized and any desired settings have been
 * configured in the `ad5761r_dev` structure. The function writes these
 * settings to the device's control register, ensuring that the device
 * operates according to the specified parameters. It is important to
 * ensure that the `dev` parameter is not null and points to a properly
 * initialized `ad5761r_dev` structure.
 *
 * @param dev A pointer to an `ad5761r_dev` structure containing the device
 * settings. Must not be null. The structure should be initialized
 * and configured with the desired settings before calling this
 * function.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad5761r_config(struct ad5761r_dev *dev);
/* Enable/disable daisy-chain mode. */
/***************************************************************************//**
 * @brief Use this function to control the daisy-chain mode of the AD5761R
 * device, which must be initialized before calling this function. This
 * mode allows multiple devices to be connected in series, sharing a
 * common data line. The function updates the device's internal state and
 * sends a command to the device to enable or disable the daisy-chain
 * mode based on the provided parameter. It is important to ensure that
 * the device is in a suitable state to accept this configuration change.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A boolean value where true enables the daisy-chain mode and
 * false disables it.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad5761r_set_daisy_chain_en_dis(struct ad5761r_dev *dev,
				       bool en_dis);
/* Get the status of the daisy-chain mode. */
/***************************************************************************//**
 * @brief This function is used to check whether the daisy-chain mode is
 * currently enabled or disabled on the AD5761R device. It should be
 * called when you need to verify the daisy-chain configuration of the
 * device. The function requires a valid device structure that has been
 * initialized and a pointer to a boolean variable where the status will
 * be stored. The function does not perform any error checking on the
 * input parameters, so it is the caller's responsibility to ensure that
 * the device is properly initialized and the pointer is valid.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a boolean variable where the daisy-chain status
 * will be stored. Must not be null.
 * @return Returns 0 on success. The boolean pointed to by en_dis is set to true
 * if daisy-chain mode is enabled, or false if it is disabled.
 ******************************************************************************/
int32_t ad5761r_get_daisy_chain_en_dis(struct ad5761r_dev *dev,
				       bool *en_dis);
/* Set the output_range. */
/***************************************************************************//**
 * @brief Use this function to configure the output voltage range of the AD5761R
 * device. This function should be called when you need to change the
 * voltage range settings of the device. It updates the device's
 * configuration to reflect the new range and applies the changes
 * immediately. Ensure that the device has been properly initialized
 * before calling this function. The function returns an error code if
 * the configuration update fails.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param out_range An enum value of type ad5761r_range specifying the desired
 * output voltage range. Must be a valid range defined in the
 * ad5761r_range enumeration.
 * @return Returns an int32_t error code. A non-zero value indicates a failure
 * in updating the device configuration.
 ******************************************************************************/
int32_t ad5761r_set_output_range(struct ad5761r_dev *dev,
				 enum ad5761r_range out_range);
/* Get the output_range. */
/***************************************************************************//**
 * @brief Use this function to obtain the current output range configuration of
 * an AD5761R device. This function is typically called after the device
 * has been initialized and configured, to verify or log the current
 * output range setting. It is important to ensure that the device
 * pointer is valid and that the output range pointer is not null before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param out_range A pointer to an ad5761r_range enum where the current output
 * range will be stored. Must not be null.
 * @return Returns 0 on success, and the current output range is stored in the
 * location pointed to by out_range.
 ******************************************************************************/
int32_t ad5761r_get_output_range(struct ad5761r_dev *dev,
				 enum ad5761r_range *out_range);
/* Set the power up voltage. */
/***************************************************************************//**
 * @brief This function configures the power-up voltage scale of the AD5761R
 * device by setting the specified scale value. It should be called when
 * you need to define the voltage level that the device will use upon
 * powering up. The function requires a valid device structure and a
 * scale value from the predefined enumeration. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. After setting the scale, the function applies the
 * configuration to the device.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param pv An enum ad5761r_scale value representing the desired power-up
 * voltage scale. Must be a valid value from the ad5761r_scale
 * enumeration.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad5761r_set_power_up_voltage(struct ad5761r_dev *dev,
				     enum ad5761r_scale pv);
/* Get the power up voltage. */
/***************************************************************************//**
 * @brief Use this function to obtain the current power-up voltage scale setting
 * of the AD5761R device. This function is typically called after the
 * device has been initialized to verify or log the power-up
 * configuration. It requires a valid device structure and a pointer to
 * an enum where the power-up voltage scale will be stored. Ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param pv A pointer to an enum ad5761r_scale variable where the power-up
 * voltage scale will be stored. Must not be null.
 * @return Returns 0 on success. The power-up voltage scale is stored in the
 * location pointed to by pv.
 ******************************************************************************/
int32_t ad5761r_get_power_up_voltage(struct ad5761r_dev *dev,
				     enum ad5761r_scale *pv);
/* Set the clear voltage. */
/***************************************************************************//**
 * @brief This function sets the clear voltage scale for the AD5761R device by
 * updating the device's configuration with the specified scale. It
 * should be called when you need to change the clear voltage setting of
 * the device. The function requires a valid device structure and a clear
 * voltage scale enumeration value. It returns an integer status code
 * indicating success or failure of the operation. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param cv An enumeration value of type ad5761r_scale representing the desired
 * clear voltage scale. Valid values are AD5761R_SCALE_ZERO,
 * AD5761R_SCALE_HALF, and AD5761R_SCALE_FULL.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad5761r_set_clear_voltage(struct ad5761r_dev *dev,
				  enum ad5761r_scale cv);
/* Get the clear voltage. */
/***************************************************************************//**
 * @brief Use this function to obtain the current clear voltage setting of the
 * AD5761R device. This function is typically called after the device has
 * been initialized and configured. It provides the clear voltage
 * setting, which is used to determine the output voltage level when the
 * clear pin is activated. Ensure that the device pointer is valid and
 * that the function is called in a context where the clear voltage
 * setting is relevant.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param cv A pointer to an ad5761r_scale variable where the current clear
 * voltage setting will be stored. Must not be null.
 * @return Returns 0 on success. The clear voltage setting is written to the
 * location pointed to by cv.
 ******************************************************************************/
int32_t ad5761r_get_clear_voltage(struct ad5761r_dev *dev,
				  enum ad5761r_scale *cv);
/* Enable/disable internal reference. */
/***************************************************************************//**
 * @brief Use this function to control the internal reference of the AD5761R
 * device, which can be enabled or disabled based on the application's
 * requirements. This function should be called after the device has been
 * initialized. Changing the internal reference state will reconfigure
 * the device settings accordingly. Ensure that the device pointer is
 * valid before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A boolean value where 'true' enables the internal reference and
 * 'false' disables it.
 * @return Returns an int32_t status code from the ad5761r_config function,
 * indicating success or an error code.
 ******************************************************************************/
int32_t ad5761r_set_internal_reference_en_dis(struct ad5761r_dev *dev,
		bool en_dis);
/* Get the status of the internal reference. */
/***************************************************************************//**
 * @brief This function is used to check whether the internal reference of the
 * AD5761R device is enabled or disabled. It should be called when you
 * need to verify the current state of the internal reference setting.
 * The function requires a valid device structure that has been
 * initialized and configured. It does not modify the device state but
 * provides the status through the output parameter.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a boolean variable where the function will store
 * the status of the internal reference. Must not be null.
 * @return Returns 0 on success, and the status of the internal reference is
 * stored in the boolean pointed to by en_dis.
 ******************************************************************************/
int32_t ad5761r_get_internal_reference_en_dis(struct ad5761r_dev *dev,
		bool *en_dis);
/* Enable/disable ETS (exceed temperature shutdown) function. */
/***************************************************************************//**
 * @brief This function is used to control the exceed temperature shutdown (ETS)
 * feature of the AD5761R device. It should be called when there is a
 * need to enable or disable the ETS functionality, which is used to
 * protect the device from overheating. The function must be called with
 * a valid device structure that has been properly initialized. After
 * setting the ETS feature, the function will configure the device with
 * the new setting.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A boolean value where 'true' enables the exceed temperature
 * shutdown feature and 'false' disables it.
 * @return Returns an int32_t value indicating the success or failure of the
 * configuration operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5761r_set_exceed_temp_shutdown_en_dis(struct ad5761r_dev *dev,
		bool en_dis);
/* Get the status of the ETS (exceed temperature shutdown) function. */
/***************************************************************************//**
 * @brief This function checks whether the exceed temperature shutdown (ETS)
 * feature is enabled or disabled on the AD5761R device. It is useful for
 * monitoring the device's thermal protection status. The function must
 * be called with a valid device structure that has been initialized. The
 * status is returned through a pointer, which must not be null. This
 * function does not modify the device state.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a boolean variable where the status of the ETS
 * feature will be stored. Must not be null.
 * @return Returns 0 on success, with the ETS status stored in the provided
 * boolean pointer.
 ******************************************************************************/
int32_t ad5761r_get_exceed_temp_shutdown_en_dis(struct ad5761r_dev *dev,
		bool *en_dis);
/* Enable/disable the twos complement bipolar output range. */
/***************************************************************************//**
 * @brief This function is used to configure the AD5761R device to either enable
 * or disable the two's complement bipolar output range. It should be
 * called when there is a need to change the output range configuration
 * of the device. The function must be called with a valid device
 * structure that has been initialized. After setting the configuration,
 * the device is reconfigured to apply the changes.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure. This parameter
 * must not be null, and the device must be properly initialized
 * before calling this function.
 * @param en_dis A boolean value where 'true' enables the two's complement
 * bipolar range and 'false' disables it.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad5761r_set_2c_bipolar_range_en_dis(struct ad5761r_dev *dev,
		bool en_dis);
/* Get the status of the twos complement bipolar output range. */
/***************************************************************************//**
 * @brief This function is used to check whether the two's complement bipolar
 * output range is enabled or disabled on the AD5761R device. It should
 * be called when you need to verify the current configuration of the
 * device regarding its output range settings. Ensure that the device has
 * been properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a boolean variable where the function will store
 * the status of the two's complement bipolar range. Must not be
 * null.
 * @return Returns 0 on success, and the status is written to the boolean
 * pointed to by en_dis.
 ******************************************************************************/
int32_t ad5761r_get_2c_bipolar_range_en_dis(struct ad5761r_dev *dev,
		bool *en_dis);
/* Enable/disable the 5% overrange. */
/***************************************************************************//**
 * @brief This function is used to control the 5% overrange feature of the
 * AD5761R device, which allows the output range to extend beyond its
 * nominal limits by 5%. It should be called when the device is already
 * initialized and configured. Enabling this feature can be useful in
 * applications requiring a slightly extended output range. After setting
 * the overrange feature, the function reconfigures the device to apply
 * the change. Ensure that the device is in a state where reconfiguration
 * is safe before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A boolean value where 'true' enables the 5% overrange feature
 * and 'false' disables it.
 * @return Returns an int32_t status code from the ad5761r_config function,
 * indicating success or failure of the reconfiguration process.
 ******************************************************************************/
int32_t ad5761r_set_overrange_en_dis(struct ad5761r_dev *dev,
				     bool en_dis);
/* Get the status of the 5% overrange. */
/***************************************************************************//**
 * @brief Use this function to check whether the 5% overrange feature is
 * currently enabled or disabled on the AD5761R device. This function is
 * useful for verifying the current configuration of the device,
 * particularly in applications where precise control over the output
 * range is required. It must be called with a valid device structure
 * that has been properly initialized. The function writes the status to
 * the provided boolean pointer, indicating whether the overrange feature
 * is enabled (true) or disabled (false).
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a boolean variable where the function will store
 * the overrange status. Must not be null.
 * @return Returns 0 on success. The boolean pointed to by en_dis is set to true
 * if overrange is enabled, false otherwise.
 ******************************************************************************/
int32_t ad5761r_get_overrange_en_dis(struct ad5761r_dev *dev,
				     bool *en_dis);
/* Get the short-circuit condition. */
/***************************************************************************//**
 * @brief Use this function to check if a short-circuit condition is present in
 * the AD5761R device. It reads the control register to determine the
 * status and updates the provided boolean pointer with the result. This
 * function should be called when you need to monitor the device for
 * short-circuit conditions, typically as part of a diagnostic or safety
 * check. Ensure that the device has been properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param sc A pointer to a boolean variable where the short-circuit condition
 * status will be stored. Must not be null.
 * @return Returns an int32_t status code indicating success or failure of the
 * read operation. The boolean pointed to by 'sc' is updated to reflect
 * the short-circuit condition status.
 ******************************************************************************/
int32_t ad5761r_get_short_circuit_condition(struct ad5761r_dev *dev,
		bool *sc);
/* Get the brownout condition. */
/***************************************************************************//**
 * @brief Use this function to check if the AD5761R device is currently
 * experiencing a brownout condition. This function reads the control
 * register of the device to determine the status. It should be called
 * when you need to monitor the power status of the device, especially in
 * environments where power stability is a concern. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param bo A pointer to a boolean variable where the brownout condition status
 * will be stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. The boolean pointed to by 'bo' is set to true if a
 * brownout condition is detected, otherwise false.
 ******************************************************************************/
int32_t ad5761r_get_brownout_condition(struct ad5761r_dev *dev,
				       bool *bo);
/* Set the reset pin value. */
/***************************************************************************//**
 * @brief This function sets the value of the reset pin for the AD5761R device,
 * which is used to control the reset state of the device. It should be
 * called when you need to change the reset state of the device,
 * typically during initialization or when resetting the device. The
 * function requires that the device structure has been properly
 * initialized and that the reset GPIO is configured. If the reset GPIO
 * is not available, the function will return an error.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure. This must not
 * be null and should have a valid gpio_reset configured.
 * @param value The value to set on the reset pin, typically 0 or 1. Values
 * outside this range may result in undefined behavior.
 * @return Returns 0 on success, or -1 if the reset GPIO is not configured.
 ******************************************************************************/
int32_t ad5761r_set_reset_pin(struct ad5761r_dev *dev,
			      uint8_t value);
/* Get the reset pin value. */
/***************************************************************************//**
 * @brief This function retrieves the current value of the reset pin associated
 * with the specified AD5761R device. It should be called when you need
 * to check the state of the reset pin. The function requires that the
 * device structure has been properly initialized and that the reset pin
 * is configured. If the reset pin is not available, the function will
 * return an error code.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null.
 * @param value A pointer to a uint8_t where the reset pin value will be stored.
 * Must not be null.
 * @return Returns 0 on success and stores the reset pin value in the provided
 * location. Returns -1 if the reset pin is not available.
 ******************************************************************************/
int32_t ad5761r_get_reset_pin(struct ad5761r_dev *dev,
			      uint8_t *value);
/* Set the clr pin value. */
/***************************************************************************//**
 * @brief This function sets the value of the CLR (clear) pin for the AD5761R
 * device, which is used to control the device's clear functionality. It
 * should be called when you need to change the state of the CLR pin. The
 * function requires that the device has been properly initialized and
 * that the CLR pin is configured. If the CLR pin is not configured, the
 * function will return an error. This function is typically used in
 * scenarios where the clear functionality of the device needs to be
 * controlled programmatically.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized with a configured CLR pin.
 * @param value The value to set on the CLR pin, typically 0 or 1. The function
 * will store this value and attempt to set the CLR pin
 * accordingly.
 * @return Returns 0 on success, or -1 if the CLR pin is not configured. If the
 * GPIO operation fails, a negative error code is returned.
 ******************************************************************************/
int32_t ad5761r_set_clr_pin(struct ad5761r_dev *dev,
			    uint8_t value);
/* Get the clr pin value. */
/***************************************************************************//**
 * @brief Use this function to obtain the current state of the CLR (clear) pin
 * associated with the AD5761R device. This function is useful for
 * checking the pin's status in applications where the CLR pin is used to
 * reset or clear certain conditions on the device. It must be called
 * with a valid device structure that has been properly initialized. If
 * the CLR pin is not configured, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure. This must not
 * be null and should represent a valid device instance.
 * @param value A pointer to a uint8_t where the CLR pin value will be stored.
 * This must not be null. If the CLR pin is configured, the current
 * value will be written here.
 * @return Returns 0 on success, with the CLR pin value stored in the provided
 * pointer. Returns -1 if the CLR pin is not configured.
 ******************************************************************************/
int32_t ad5761r_get_clr_pin(struct ad5761r_dev *dev,
			    uint8_t *value);
/* Set the ldac pin value. */
/***************************************************************************//**
 * @brief This function sets the LDAC (Load DAC) pin of the AD5761R device to
 * the specified value. It should be called when you need to control the
 * LDAC pin state, typically to latch data into the DAC register. The
 * function requires that the device has been properly initialized and
 * that the LDAC GPIO is configured. If the LDAC GPIO is not available,
 * the function will return an error code.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure. Must not be
 * null and should have a valid gpio_ldac configured.
 * @param value The value to set the LDAC pin to, typically 0 or 1. Values
 * outside this range may result in undefined behavior.
 * @return Returns 0 on success, or -1 if the LDAC GPIO is not configured.
 ******************************************************************************/
int32_t ad5761r_set_ldac_pin(struct ad5761r_dev *dev,
			     uint8_t value);
/* Get the ldac pin value. */
/***************************************************************************//**
 * @brief Use this function to obtain the current state of the LDAC pin
 * associated with the AD5761R device. This function should be called
 * when you need to check the LDAC pin status, which is typically used to
 * control the update of the DAC output. Ensure that the device structure
 * is properly initialized and that the LDAC pin is configured before
 * calling this function. If the LDAC pin is not available, the function
 * will return an error code.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure. This must not
 * be null and should represent a valid device context.
 * @param value A pointer to a uint8_t where the LDAC pin value will be stored.
 * This must not be null. The function writes the current LDAC pin
 * value to this location if successful.
 * @return Returns 0 on success, indicating that the LDAC pin value was
 * successfully retrieved and stored in the provided location. Returns
 * -1 if the LDAC pin is not available, indicating an error.
 ******************************************************************************/
int32_t ad5761r_get_ldac_pin(struct ad5761r_dev *dev,
			     uint8_t *value);
/* Write to input register. */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to the input
 * register of an AD5761R or AD5721R device. It should be called when you
 * need to update the input register with new data, which can later be
 * transferred to the DAC register for output. The function requires a
 * valid device structure that has been initialized and configured. It
 * handles different data formats based on the device type specified in
 * the device structure.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null. The device type within this structure
 * determines the data format used.
 * @param dac_data A 16-bit unsigned integer representing the data to be written
 * to the input register. The valid range is 0 to 65535, and the
 * function adapts the data format based on the device type.
 * @return Returns an int32_t status code indicating success or failure of the
 * write operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5761r_write_input_register(struct ad5761r_dev *dev,
				     uint16_t reg_data);
/* Update DAC register. */
/***************************************************************************//**
 * @brief This function is used to update the DAC register of the AD5761R device
 * with the value currently held in the input register. It should be
 * called when the input register has been set to the desired value and
 * you want to apply this value to the DAC output. The function requires
 * a valid device structure that has been initialized and configured. It
 * is important to ensure that the device is properly initialized before
 * calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. This parameter must not be null, and the device must be
 * properly initialized before calling this function. If the pointer
 * is invalid, the behavior is undefined.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad5761r_update_dac_register(struct ad5761r_dev *dev);
/* Write to input register and update DAC register. */
/***************************************************************************//**
 * @brief This function writes a given data value to the DAC register of the
 * AD5761R or AD5721R device and updates it immediately. It should be
 * used when you need to set a new output value for the DAC. The function
 * requires a valid device structure that has been initialized and
 * configured. The behavior of the data writing depends on the type of
 * device specified in the device structure. Ensure that the device is
 * properly initialized before calling this function to avoid undefined
 * behavior.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null. The device type within this structure
 * determines the data format used.
 * @param dac_data A 16-bit unsigned integer representing the data to be written
 * to the DAC register. The valid range depends on the device
 * type: for AD5761R, it uses the full 16 bits, while for
 * AD5721R, only the lower 12 bits are used.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5761r_write_update_dac_register(struct ad5761r_dev *dev,
		uint16_t reg_data);
/*Software data reset. */
/***************************************************************************//**
 * @brief Use this function to reset the data registers of the AD5761R device to
 * their default state via a software command. This function is typically
 * called when a reset of the device's data state is required without
 * affecting other configurations. It should be called when the device is
 * properly initialized and communication with the device is established.
 * The function returns an error code if the reset command fails to
 * execute.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered.
 ******************************************************************************/
int32_t ad5761r_software_data_reset(struct ad5761r_dev *dev);
/* Software full reset. */
/***************************************************************************//**
 * @brief Use this function to reset the AD5761R device to its default state via
 * software. This is typically done to ensure the device is in a known
 * state before configuration or after an error condition. The function
 * must be called with a valid device structure that has been properly
 * initialized. It is important to ensure that the device is not in use
 * by other operations when performing the reset to avoid undefined
 * behavior.
 *
 * @param dev A pointer to an initialized ad5761r_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * structure.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5761r_software_full_reset(struct ad5761r_dev *dev);

#endif // AD5761R_H_

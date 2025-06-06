/***************************************************************************//**
 *   @file   ad7746.h
 *   @brief  Header file of AD7746 Driver.
 *   @author Dragos Bogdan (dragos.bogdan@analog.com)
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#ifndef _AD7746_H_
#define _AD7746_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_util.h"
#include "no_os_i2c.h"

/* AD7746 Slave Address */
#define AD7746_ADDRESS			0x48

/* AD7746 Reset command */
#define AD7746_RESET_CMD		0xBF

/* AD7746 Register Definition */
#define AD7746_REG_STATUS		0u
#define AD7746_REG_CAP_DATA_HIGH	1u
#define AD7746_REG_CAP_DATA_MID		2u
#define AD7746_REG_CAP_DATA_LOW		3u
#define AD7746_REG_VT_DATA_HIGH		4u
#define AD7746_REG_VT_DATA_MID		5u
#define AD7746_REG_VT_DATA_LOW		6u
#define AD7746_REG_CAP_SETUP		7u
#define AD7746_REG_VT_SETUP		8u
#define AD7746_REG_EXC_SETUP		9u
#define AD7746_REG_CFG			10u
#define AD7746_REG_CAPDACA		11u
#define AD7746_REG_CAPDACB		12u
#define AD7746_REG_CAP_OFFH		13u
#define AD7746_REG_CAP_OFFL		14u
#define AD7746_REG_CAP_GAINH		15u
#define AD7746_REG_CAP_GAINL		16u
#define AD7746_REG_VOLT_GAINH		17u
#define AD7746_REG_VOLT_GAINL		18u

#define AD7746_NUM_REGISTERS		(AD7746_REG_VOLT_GAINL + 1u)

/* AD7746_REG_STATUS bits */
#define AD7746_STATUS_EXCERR_MSK	NO_OS_BIT(3)
#define AD7746_STATUS_RDY_MSK		NO_OS_BIT(2)
#define AD7746_STATUS_RDYVT_MSK		NO_OS_BIT(1)
#define AD7746_STATUS_RDYCAP_MSK	NO_OS_BIT(0)

/* AD7746_REG_CAP_SETUP bits */
#define AD7746_CAPSETUP_CAPEN_MSK	NO_OS_BIT(7)
#define AD7746_CAPSETUP_CIN2_MSK	NO_OS_BIT(6)
#define AD7746_CAPSETUP_CAPDIFF_MSK	NO_OS_BIT(5)
#define AD7746_CAPSETUP_CAPCHOP_MSK	NO_OS_BIT(0)

/* AD7746_REG_VT_SETUP bits */
#define AD7746_VTSETUP_VTEN_MSK		NO_OS_BIT(7)
#define AD7746_VTSETUP_VTMD_MSK		NO_OS_GENMASK(6,5)
#define AD7746_VTSETUP_EXTREF_MSK	NO_OS_BIT(4)
#define AD7746_VTSETUP_VTSHORT_MSK	NO_OS_BIT(1)
#define AD7746_VTSETUP_VTCHOP_MSK	NO_OS_BIT(0)

/* AD7746_REG_EXC_SETUP bits */
#define AD7746_EXCSETUP_CLKCTRL_MSK	NO_OS_BIT(7)
#define AD7746_EXCSETUP_EXCON_MSK	NO_OS_BIT(6)
#define AD7746_EXCSETUP_EXCB_MSK	NO_OS_GENMASK(5,4)
#define AD7746_EXCSETUP_EXCA_MSK	NO_OS_GENMASK(3,2)
#define AD7746_EXCSETUP_EXCLVL_MSK	NO_OS_GENMASK(1,0)

/* AD7746_REG_CFG bits */
#define AD7746_CONF_VTF_MSK		NO_OS_GENMASK(7,6)
#define AD7746_CONF_CAPF_MSK		NO_OS_GENMASK(5,3)
#define AD7746_CONF_MD_MSK		NO_OS_GENMASK(2,0)

/* AD7746_REG_CAPDACx bits */
#define AD7746_CAPDAC_DACEN_MSK		NO_OS_BIT(7)
#define AD7746_CAPDAC_DACP_MSK		NO_OS_GENMASK(6,0)

/***************************************************************************//**
 * @brief The `ad7746_id` enumeration defines identifiers for different models
 * of the AD774x series of capacitive sensor devices, specifically the
 * AD7745, AD7746, and AD7747. These identifiers are used to distinguish
 * between the different models within the driver code, allowing for
 * model-specific configurations and operations.
 *
 * @param ID_AD7745 Represents the identifier for the AD7745 device.
 * @param ID_AD7746 Represents the identifier for the AD7746 device.
 * @param ID_AD7747 Represents the identifier for the AD7747 device.
 ******************************************************************************/
enum ad7746_id {
	ID_AD7745,
	ID_AD7746,
	ID_AD7747,
};

/***************************************************************************//**
 * @brief The `ad7746_cap` structure is used to configure the capacitive
 * measurement settings of the AD7746 device. It includes flags to enable
 * capacitive measurement, select the input channel, choose between
 * single-ended or differential measurement, and enable chopping for
 * improved accuracy. This structure is part of the setup configuration
 * for the AD7746 capacitive sensor interface.
 *
 * @param capen Indicates if the capacitive measurement is enabled.
 * @param cin2 Specifies if the second capacitive input is used.
 * @param capdiff Determines if the capacitive measurement is differential.
 * @param capchop Indicates if chopping is enabled for capacitive measurement.
 ******************************************************************************/
struct ad7746_cap {
	bool capen;
	bool cin2;
	bool capdiff;
	bool capchop;
};

/***************************************************************************//**
 * @brief The `ad7746_vtmd` enumeration defines the various voltage and
 * temperature measurement modes available for the AD7746 device. Each
 * enumerator corresponds to a specific mode of operation, allowing the
 * device to measure internal temperature, external temperature, VDD
 * voltage, or an external voltage input. This enumeration is used to
 * configure the AD7746 for the desired measurement mode.
 *
 * @param AD7746_VTMD_INT_TEMP Represents the internal temperature measurement
 * mode.
 * @param AD7746_VTMD_EXT_TEMP Represents the external temperature measurement
 * mode.
 * @param AD7746_VTMD_VDD_MON Represents the VDD voltage monitoring mode.
 * @param AD7746_VIN_EXT_VIN Represents the external voltage input measurement
 * mode.
 ******************************************************************************/
enum ad7746_vtmd {
	AD7746_VTMD_INT_TEMP,
	AD7746_VTMD_EXT_TEMP,
	AD7746_VTMD_VDD_MON,
	AD7746_VIN_EXT_VIN
};

/***************************************************************************//**
 * @brief The `ad7746_vt` structure is used to configure the voltage and
 * temperature measurement settings for the AD7746 device. It includes
 * flags to enable the measurement, select the measurement mode, and
 * configure additional options such as using an external reference,
 * shorting the input, and enabling chopping for improved measurement
 * accuracy.
 *
 * @param vten A boolean indicating if the voltage/temperature measurement is
 * enabled.
 * @param vtmd An enumeration specifying the mode of voltage/temperature
 * measurement.
 * @param extref A boolean indicating if an external reference is used for
 * measurements.
 * @param vtshort A boolean indicating if the voltage/temperature input is
 * shorted.
 * @param vtchop A boolean indicating if chopping is enabled for
 * voltage/temperature measurements.
 ******************************************************************************/
struct ad7746_vt {
	bool vten;
	enum ad7746_vtmd vtmd;
	bool extref;
	bool vtshort;
	bool vtchop;
};

/***************************************************************************//**
 * @brief The `ad7746_exc_pin` enumeration defines the possible states of the
 * excitation pin for the AD7746 capacitive sensor. It includes three
 * states: disabled, inverted, and normal, which are used to configure
 * the behavior of the excitation pin in the sensor's setup.
 *
 * @param AD7746_EXC_PIN_DISABLED Represents the state where the excitation pin
 * is disabled.
 * @param AD7746_EXC_PIN_INVERTED Represents the state where the excitation pin
 * is inverted.
 * @param AD7746_EXC_PIN_NORMAL Represents the state where the excitation pin is
 * in its normal configuration.
 ******************************************************************************/
enum ad7746_exc_pin {
	AD7746_EXC_PIN_DISABLED,
	AD7746_EXC_PIN_INVERTED,
	AD7746_EXC_PIN_NORMAL
};

/***************************************************************************//**
 * @brief The `ad7746_exclvl` enumeration defines different excitation levels
 * for the AD7746 device, which are used to configure the excitation
 * current levels in capacitive sensing applications. Each enumerator
 * corresponds to a specific fraction of the maximum excitation level,
 * allowing for precise control over the excitation current used in the
 * sensor setup.
 *
 * @param AD7746_EXCLVL_1_DIV_8 Represents an excitation level of 1/8.
 * @param AD7746_EXCLVL_2_DIV_8 Represents an excitation level of 2/8.
 * @param AD7746_EXCLVL_3_DIV_8 Represents an excitation level of 3/8.
 * @param AD7746_EXCLVL_4_DIV_8 Represents an excitation level of 4/8.
 ******************************************************************************/
enum ad7746_exclvl {
	AD7746_EXCLVL_1_DIV_8,
	AD7746_EXCLVL_2_DIV_8,
	AD7746_EXCLVL_3_DIV_8,
	AD7746_EXCLVL_4_DIV_8
};

/***************************************************************************//**
 * @brief The `ad7746_exc` structure is used to configure the excitation
 * settings of the AD7746 capacitive sensor. It includes control flags
 * for enabling the excitation circuit and clock control, as well as
 * enumerations to define the configuration of excitation pins A and B,
 * and the excitation level. This structure is essential for setting up
 * the excitation parameters that influence the sensor's operation and
 * measurement accuracy.
 *
 * @param clkctrl A boolean indicating the clock control state for the
 * excitation circuit.
 * @param excon A boolean indicating whether the excitation circuit is enabled.
 * @param excb An enum specifying the configuration of the excitation pin B.
 * @param exca An enum specifying the configuration of the excitation pin A.
 * @param exclvl An enum specifying the excitation level.
 ******************************************************************************/
struct ad7746_exc {
	bool clkctrl;
	bool excon;
	enum ad7746_exc_pin excb;
	enum ad7746_exc_pin exca;
	enum ad7746_exclvl exclvl;
};

/***************************************************************************//**
 * @brief The `ad7746_md` enumeration defines the various operational modes for
 * the AD7746 capacitive sensor device. These modes include idle,
 * continuous conversion, single conversion, power-down, offset
 * calibration, and gain calibration. Each mode is represented by a
 * specific enumerator, allowing for easy configuration and control of
 * the device's operational state.
 *
 * @param AD7746_MODE_IDLE Represents the idle mode of the AD7746 device.
 * @param AD7746_MODE_CONT Represents the continuous conversion mode of the
 * AD7746 device.
 * @param AD7746_MODE_SINGLE Represents the single conversion mode of the AD7746
 * device.
 * @param AD7746_MODE_POWERDOWN Represents the power-down mode of the AD7746
 * device.
 * @param AD7746_MODE_OFFSET_CALIB Represents the offset calibration mode of the
 * AD7746 device, with a specific value of 5.
 * @param AD7746_MODE_GAIN_CALIB Represents the gain calibration mode of the
 * AD7746 device.
 ******************************************************************************/
enum ad7746_md {
	AD7746_MODE_IDLE,
	AD7746_MODE_CONT,
	AD7746_MODE_SINGLE,
	AD7746_MODE_POWERDOWN,
	AD7746_MODE_OFFSET_CALIB = 5,
	AD7746_MODE_GAIN_CALIB
};

/***************************************************************************//**
 * @brief The `ad7746_config` structure is used to configure the AD7746
 * capacitive sensor and temperature sensor device. It contains fields
 * for setting the voltage and temperature filter (`vtf`), the
 * capacitance filter (`capf`), and the mode of operation (`md`). This
 * structure is essential for defining the operational parameters of the
 * AD7746, allowing users to customize the device's behavior according to
 * their specific application needs.
 *
 * @param vtf A uint8_t field representing the voltage and temperature filter
 * settings.
 * @param capf A uint8_t field representing the capacitance filter settings.
 * @param md An enum ad7746_md field representing the mode of operation for the
 * AD7746 device.
 ******************************************************************************/
struct ad7746_config {
	uint8_t vtf;
	uint8_t capf;
	enum ad7746_md md;
};

/***************************************************************************//**
 * @brief The `ad7746_setup` structure is a compound data type used to
 * encapsulate the configuration settings for the AD7746 capacitive
 * sensor device. It includes sub-structures for configuring capacitive
 * measurements (`cap`), voltage and temperature measurements (`vt`),
 * excitation settings (`exc`), and general device configuration
 * (`config`). This structure is essential for initializing and managing
 * the AD7746 device's operational parameters.
 *
 * @param cap Represents the capacitive measurement setup configuration.
 * @param vt Represents the voltage and temperature measurement setup
 * configuration.
 * @param exc Represents the excitation setup configuration.
 * @param config Represents the general configuration settings for the AD7746
 * device.
 ******************************************************************************/
struct ad7746_setup {
	struct ad7746_cap cap;
	struct ad7746_vt vt;
	struct ad7746_exc exc;
	struct ad7746_config config;
};

/***************************************************************************//**
 * @brief The `ad7746_init_param` structure is used to encapsulate the
 * initialization parameters required to set up an AD7746 device. It
 * includes the I2C initialization parameters, the specific device ID,
 * and the setup configuration which includes various settings for
 * capacitance, voltage, excitation, and general configuration. This
 * structure is essential for initializing the AD7746 device with the
 * desired operational parameters.
 *
 * @param i2c_init This member is a structure that holds the initialization
 * parameters for the I2C interface.
 * @param id This member is an enumeration that specifies the ID of the AD7746
 * device variant.
 * @param setup This member is a structure that contains the configuration
 * settings for the AD7746 device.
 ******************************************************************************/
struct ad7746_init_param {
	struct no_os_i2c_init_param i2c_init;
	enum ad7746_id id;
	struct ad7746_setup setup;
};

/***************************************************************************//**
 * @brief The `ad7746_dev` structure is a compound data type used to encapsulate
 * all necessary information and configurations for interfacing with an
 * AD7746 capacitive sensor via I2C. It includes a pointer to an I2C
 * descriptor for communication, an identifier for the specific device
 * variant, a buffer for register data, and a setup structure that holds
 * various configuration parameters for the device's operation.
 *
 * @param i2c_dev Pointer to an I2C descriptor for communication with the AD7746
 * device.
 * @param id Enumeration value representing the specific AD7746 device variant.
 * @param buf Buffer array to hold data for all AD7746 registers plus one
 * additional byte.
 * @param setup Structure containing configuration settings for the AD7746
 * device.
 ******************************************************************************/
struct ad7746_dev {
	struct no_os_i2c_desc *i2c_dev;
	enum ad7746_id id;
	uint8_t buf[AD7746_NUM_REGISTERS + 1u];
	struct ad7746_setup setup;
};

/***************************************************************************//**
 * @brief This function initializes an AD7746 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure, sets up the I2C communication, and configures the device
 * according to the specified setup parameters. If any step fails, the
 * function ensures proper cleanup and returns an error code. The caller
 * is responsible for providing valid initialization parameters and
 * handling the device pointer upon successful initialization.
 *
 * @param device A pointer to a pointer of type `struct ad7746_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad7746_init_param` containing the
 * initialization parameters, including I2C settings and
 * device configuration. Must not be null and must be properly
 * initialized before calling this function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `device` pointer is set to point to the initialized
 * device structure.
 ******************************************************************************/
int32_t ad7746_init(struct ad7746_dev **device,
		    struct ad7746_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to write a specified number of bytes to a
 * register on the AD7746 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function requires a valid register address and a data buffer
 * containing the bytes to be written. It is important to note that the
 * number of bytes to be written should not exceed the total number of
 * registers available on the device. If the data pointer is null, the
 * register address is invalid, or the byte count exceeds the register
 * limit, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null.
 * @param reg The register address to which data will be written. Must be less
 * than AD7746_NUM_REGISTERS.
 * @param data A pointer to the data buffer containing the bytes to be written.
 * Must not be null.
 * @param bytes_number The number of bytes to write from the data buffer. Must
 * not exceed AD7746_NUM_REGISTERS.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int32_t ad7746_reg_write(struct ad7746_dev *dev, uint8_t reg,
			 uint8_t* data, uint16_t bytes_number);
/***************************************************************************//**
 * @brief This function reads a specified number of bytes from a given register
 * of the AD7746 device into a provided data buffer. It is typically used
 * to retrieve configuration or measurement data from the device. The
 * function must be called with a valid device structure that has been
 * properly initialized. The register address must be within the valid
 * range of AD7746 registers, and the number of bytes to read must not
 * exceed the total number of available registers. If the data buffer is
 * null or the parameters are out of range, the function returns an error
 * code.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null.
 * @param reg The starting register address from which to read. Must be less
 * than AD7746_NUM_REGISTERS.
 * @param data A pointer to a buffer where the read data will be stored. Must
 * not be null.
 * @param bytes_number The number of bytes to read from the device. Must be less
 * than or equal to AD7746_NUM_REGISTERS.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad7746_reg_read(struct ad7746_dev *dev, uint8_t reg,
			uint8_t* data, uint16_t bytes_number);
/***************************************************************************//**
 * @brief Use this function to reset the AD7746 device, which is typically
 * necessary when initializing the device or recovering from an error
 * state. This function sends a reset command to the device over I2C. It
 * must be called with a valid device structure that has been properly
 * initialized. If the device pointer is null, the function will return
 * an error code.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device to reset. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the I2C write operation fails.
 ******************************************************************************/
int32_t ad7746_reset(struct ad7746_dev *dev);
/***************************************************************************//**
 * @brief This function is used to clean up and release resources associated
 * with an AD7746 device instance. It should be called when the device is
 * no longer needed, typically at the end of its lifecycle. The function
 * ensures that any allocated resources, such as I2C descriptors, are
 * properly freed. It is safe to call this function with a null pointer,
 * in which case it will have no effect. After calling this function, the
 * device pointer should not be used unless it is reinitialized.
 *
 * @param dev A pointer to an ad7746_dev structure representing the device
 * instance to be removed. This pointer must be valid and non-null
 * unless the function is called with a null pointer, in which case
 * the function does nothing. The caller retains ownership of the
 * pointer.
 * @return Returns 0 in all cases, indicating successful removal or no operation
 * if the input was null.
 ******************************************************************************/
int32_t ad7746_remove(struct ad7746_dev *dev);
/***************************************************************************//**
 * @brief This function sets the capacitive measurement configuration for the
 * AD7746 device by writing the specified settings to the appropriate
 * register. It should be called when the user needs to configure the
 * capacitive measurement parameters such as enabling the capacitive
 * measurement, selecting the input channel, and setting differential and
 * chopping modes. The function must be called with a valid device
 * structure that has been initialized. If the device pointer is null,
 * the function returns an error. The function updates the device's
 * internal setup state with the new configuration.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param cap A structure of type ad7746_cap containing the desired capacitive
 * measurement settings. This includes enabling capacitive
 * measurement, selecting the input channel, and setting differential
 * and chopping modes.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if writing to the device register fails.
 ******************************************************************************/
int32_t ad7746_set_cap(struct ad7746_dev *dev, struct ad7746_cap cap);
/***************************************************************************//**
 * @brief Use this function to set up the voltage and temperature measurement
 * parameters for the AD7746 device. This function should be called after
 * the device has been initialized and before starting any measurement
 * operations that depend on these settings. It configures various
 * aspects of the voltage and temperature measurement, such as enabling
 * the measurement, selecting the measurement mode, and setting reference
 * and chopping options. The function returns an error code if the device
 * pointer is null or if the register write operation fails.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param vt A structure of type ad7746_vt containing the desired voltage and
 * temperature measurement settings. This includes enabling the
 * measurement, selecting the measurement mode, and configuring
 * reference and chopping options. The caller retains ownership of
 * this structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the register write operation fails.
 ******************************************************************************/
int32_t ad7746_set_vt(struct ad7746_dev *dev, struct ad7746_vt vt);
/***************************************************************************//**
 * @brief Use this function to set the excitation parameters of the AD7746
 * device, which include clock control, excitation connection, and
 * excitation levels. This function should be called when you need to
 * configure or update the excitation settings of the device. It is
 * important to ensure that the device pointer is valid before calling
 * this function, as passing a null pointer will result in an error. The
 * function updates the device's internal setup structure with the new
 * excitation settings upon successful execution.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param exc A structure of type ad7746_exc containing the desired excitation
 * settings, including clock control, excitation connection, and
 * excitation levels. The values should be within the valid range
 * defined by the ad7746_exc structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if writing to the device register fails.
 ******************************************************************************/
int32_t ad7746_set_exc(struct ad7746_dev *dev, struct ad7746_exc exc);
/***************************************************************************//**
 * @brief This function sets the configuration of the AD7746 device using the
 * provided configuration parameters. It should be called when the device
 * needs to be configured with new settings, such as after initialization
 * or when changing operational modes. The function requires a valid
 * device structure and a configuration structure containing the desired
 * settings. If the device pointer is null, the function returns an
 * error. The configuration is applied by writing to the device's
 * configuration register, and the device's internal setup state is
 * updated accordingly.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param config An ad7746_config structure containing the desired configuration
 * settings for the device. The structure includes fields for
 * voltage and temperature filter, capacitance filter, and mode
 * settings.
 * @return Returns 0 on success, or a negative error code if the configuration
 * could not be applied.
 ******************************************************************************/
int32_t ad7746_set_config(struct ad7746_dev *dev, struct ad7746_config config);
/***************************************************************************//**
 * @brief This function is used to enable or disable the capacitive DAC A on the
 * AD7746 device and set its code value. It should be called when you
 * need to adjust the capacitive offset for the sensor connected to the
 * AD7746. The function requires a valid device structure pointer and
 * will return an error if the device pointer is null. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the capacitive DAC A.
 * @param code An 8-bit unsigned integer representing the code to set for the
 * capacitive DAC A. The valid range is 0 to 127, as only the lower
 * 7 bits are used.
 * @return Returns an int32_t value. A non-negative value indicates success,
 * while a negative value indicates an error, such as an invalid device
 * pointer.
 ******************************************************************************/
int32_t ad7746_set_cap_dac_a(struct ad7746_dev *dev, bool enable, uint8_t code);
/***************************************************************************//**
 * @brief Use this function to enable or disable the capacitive digital-to-
 * analog converter (CAPDAC) B and set its code on an AD7746 device. This
 * function should be called after the device has been properly
 * initialized. It modifies the CAPDAC B register based on the provided
 * parameters. Ensure that the device pointer is valid before calling
 * this function, as passing a null pointer will result in an error.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the CAPDAC B.
 * @param code An 8-bit unsigned integer representing the code to set for CAPDAC
 * B. The valid range is 0 to 127, as only the lower 7 bits are
 * used.
 * @return Returns an int32_t value. A non-negative value indicates success,
 * while a negative value indicates an error, such as an invalid device
 * pointer.
 ******************************************************************************/
int32_t ad7746_set_cap_dac_b(struct ad7746_dev *dev, bool enable, uint8_t code);
/***************************************************************************//**
 * @brief Use this function to configure the capacitive offset of the AD7746
 * device, which is necessary for calibration and accurate capacitance
 * measurements. This function should be called after initializing the
 * device and before starting measurements. It writes the specified
 * offset value to the appropriate register in the device. Ensure that
 * the device pointer is valid and that the offset value is within the
 * acceptable range for the device.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null.
 * @param offset A 16-bit unsigned integer representing the capacitive offset to
 * be set. The valid range is device-specific, and values outside
 * this range may result in undefined behavior.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad7746_set_cap_offset(struct ad7746_dev *dev, uint16_t offset);
/***************************************************************************//**
 * @brief This function configures the capacitive gain setting of the AD7746
 * device. It should be called when the user needs to adjust the gain for
 * capacitive measurements. The function requires a valid device
 * structure and a gain value to be provided. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. The gain value is written to the appropriate register in the
 * device, and the function returns an error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param gain A 16-bit unsigned integer representing the gain value to be set.
 * The valid range is determined by the device's specifications.
 * Invalid values may result in an error code being returned.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating failure.
 ******************************************************************************/
int32_t ad7746_set_cap_gain(struct ad7746_dev *dev, uint16_t gain);
/***************************************************************************//**
 * @brief This function configures the voltage gain setting of the AD7746
 * device. It should be called when you need to adjust the voltage gain
 * for measurements. Ensure that the device has been properly initialized
 * before calling this function. The function writes the specified gain
 * value to the appropriate register in the device. It is important to
 * provide a valid gain value within the acceptable range for the device
 * to function correctly.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null.
 * @param gain A 16-bit unsigned integer representing the desired voltage gain.
 * The valid range depends on the device specifications.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad7746_set_volt_gain(struct ad7746_dev *dev, uint16_t gain);
/***************************************************************************//**
 * @brief Use this function to obtain the latest voltage and temperature data
 * from an AD7746 device. It must be called with a valid device structure
 * that has been properly initialized. The function will block until the
 * data is ready, and it will store the retrieved data in the provided
 * memory location. Ensure that the device is not in power-down mode
 * before calling this function. If the device is in single conversion
 * mode, it will be set to idle mode after the data is retrieved.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param vt_data A pointer to a uint32_t where the voltage and temperature data
 * will be stored. Must not be null. If null, the function
 * returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The
 * vt_data pointer is updated with the retrieved data on success.
 ******************************************************************************/
int32_t ad7746_get_vt_data(struct ad7746_dev *dev, uint32_t *vt_data);
/***************************************************************************//**
 * @brief Use this function to obtain the latest capacitive measurement data
 * from an AD7746 device. It must be called after the device has been
 * properly initialized and configured. The function waits for the
 * capacitive data to be ready before reading it. If the device is in
 * single conversion mode, it will be set to idle mode after the data is
 * retrieved. This function returns an error code if the device or the
 * output pointer is null, or if there is a communication error with the
 * device.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param cap_data A pointer to a uint32_t where the capacitive data will be
 * stored. Must not be null. The function will return an error
 * if this parameter is null.
 * @return Returns 0 on success, or a negative error code on failure. The
 * capacitive data is written to the location pointed to by cap_data on
 * success.
 ******************************************************************************/
int32_t ad7746_get_cap_data(struct ad7746_dev *dev, uint32_t *cap_data);
/***************************************************************************//**
 * @brief Use this function to initiate either an offset or gain calibration on
 * the AD7746 device. It must be called with a valid calibration mode
 * after the device has been properly initialized. The function will
 * block until the calibration process is complete or a timeout occurs.
 * It is important to ensure that the device is in a suitable state for
 * calibration before calling this function.
 *
 * @param dev A pointer to an initialized ad7746_dev structure representing the
 * device to be calibrated. Must not be null.
 * @param md An enum value of type ad7746_md specifying the calibration mode.
 * Must be either AD7746_MODE_OFFSET_CALIB or AD7746_MODE_GAIN_CALIB.
 * Invalid values will result in an error.
 * @return Returns 0 on success, a negative error code on failure, or -ETIMEDOUT
 * if the calibration process times out.
 ******************************************************************************/
int32_t ad7746_calibrate(struct ad7746_dev *dev, enum ad7746_md md);

#endif // _AD7746_H

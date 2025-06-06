/**************************************************************************//**
*   @file   ad5629r.h
*   @brief  Header file of AD5629R Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
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
******************************************************************************/

#ifndef _AD5629R_H_
#define _AD5629R_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_i2c.h"

/******************************************************************************/
/******************* Macros and Constants Definitions *************************/
/******************************************************************************/
/* AD5629R Device I2C Address */
#define AD5629R_I2C_ADDR_0          0x54 // A1=0 and A0=0 (A0_Pin=High)
#define AD5629R_I2C_ADDR_1          0x56 // A1=1 and A0=0 (A0_Pin=NC)
#define AD5629R_I2C_ADDR_2          0x57 // A1=1 and A0=1 (A0_Pin=Low)

#define MAX_RESOLUTION              16

/* Commands */
#define AD5629R_WRITE_N             0x0
#define AD5629R_UPDATE_N            0x1
#define AD5629R_WRITE_N_UPDATE_ALL  0x2
#define AD5629R_WRITE_N_UPDATE_N    0x3
#define AD5629R_POWER               0x4
#define AD5629R_LOAD_CLEAR_REG      0x5
#define AD5629R_LOAD_LDAC_REG       0x6
#define AD5629R_RESET               0x7
#define AD5629R_REFERENCE           0x8

/* AD5629R GPIO */
/* LDAC - GPIO0 */
#define AD5629R_LDAC_OUT            no_os_gpio_direction_output(dev->gpio_ldac, \
			            NO_OS_GPIO_HIGH);
#define AD5629R_LDAC_LOW            no_os_gpio_set_value(dev->gpio_ldac,        \
			            NO_OS_GPIO_LOW)
#define AD5629R_LDAC_HIGH           no_os_gpio_set_value(dev->gpio_ldac,        \
			            NO_OS_GPIO_HIGH)
/* CLR - GPIO1 */
#define AD5629R_CLR_OUT             no_os_gpio_direction_output(dev->gpio_clr,  \
			            NO_OS_GPIO_HIGH);
#define AD5629R_CLR_LOW             no_os_gpio_set_value(dev->gpio_clr,         \
			            NO_OS_GPIO_LOW)
#define AD5629R_CLR_HIGH            no_os_gpio_set_value(dev->gpio_clr,         \
			            NO_OS_GPIO_HIGH)

/* DAC Addresses */
#define AD5629R_DAC_A_ADDR          0x0
#define AD5629R_DAC_B_ADDR          0x1
#define AD5629R_DAC_C_ADDR          0x2
#define AD5629R_DAC_D_ADDR          0x3
#define AD5629R_DAC_E_ADDR          0x4
#define AD5629R_DAC_F_ADDR          0x5
#define AD5629R_DAC_G_ADDR          0x6
#define AD5629R_DAC_H_ADDR          0x7
#define AD5629R_DAC_ALL_ADDR        0xF

/* DAC Selection */
#define DAC_A_SEL           1
#define DAC_B_SEL           2
#define DAC_C_SEL           4
#define DAC_D_SEL           8
#define DAC_E_SEL           16
#define DAC_F_SEL           32
#define DAC_G_SEL           64
#define DAC_H_SEL           128

/* Power modes */
#define PWR_NORMAL              0
#define PWR_1K_TO_GND           1
#define PWR_100K_TO_GND         2
#define PWR_3_STATE             3

/* Clear code values */
#define CLR_TO_ZEROSCALE        0
#define CLR_TO_MIDSCALE         1
#define CLR_TO_FULLSCALE        2
#define CLR_NOOP                3

/* Internal reference status */
#define REF_ON          1
#define REF_OFF         0

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5629r_type` enumeration defines a set of constants representing
 * different types of AD56xx series digital-to-analog converters (DACs)
 * from Analog Devices. Each enumerator corresponds to a specific model
 * within the series, allowing the software to handle multiple device
 * types using a unified interface. This enumeration is used to specify
 * the active device type in the driver, facilitating device-specific
 * operations and configurations.
 *
 * @param ID_AD5629R Represents the AD5629R device type.
 * @param ID_AD5669R Represents the AD5669R device type.
 * @param ID_AD5668 Represents the AD5668 device type.
 * @param ID_AD5648 Represents the AD5648 device type.
 * @param ID_AD5628 Represents the AD5628 device type.
 ******************************************************************************/
enum ad5629r_type {
	ID_AD5629R,
	ID_AD5669R,
	ID_AD5668,
	ID_AD5648,
	ID_AD5628,
};

/***************************************************************************//**
 * @brief The `comm_type_t` is an enumeration that defines the types of
 * communication protocols supported by the AD5629R device, specifically
 * SPI and I2C. This enum is used to specify the communication method
 * when interfacing with the device, allowing the software to handle
 * different communication protocols appropriately.
 *
 * @param com_spi Represents the SPI communication type.
 * @param com_i2c Represents the I2C communication type.
 ******************************************************************************/
enum comm_type_t {
	com_spi,
	com_i2c
};

/***************************************************************************//**
 * @brief The `ad5629r_chip_info` structure is used to encapsulate information
 * about a specific AD5629R chip, including its resolution and the
 * communication protocol it uses. This structure is essential for
 * configuring and interfacing with the AD5629R DAC, as it provides the
 * necessary details to ensure proper communication and operation of the
 * device.
 *
 * @param resolution Specifies the resolution of the DAC in bits.
 * @param communication Defines the type of communication interface used, either
 * SPI or I2C.
 ******************************************************************************/
struct ad5629r_chip_info {
	uint32_t resolution;
	enum comm_type_t communication;
};

/***************************************************************************//**
 * @brief The `ad5629r_dev` structure is designed to encapsulate the necessary
 * descriptors and settings for interfacing with an AD5629R device. It
 * includes pointers to I2C and SPI descriptors for communication, GPIO
 * descriptors for control pins (LDAC and CLR), and an enumeration to
 * specify the active device type. This structure is essential for
 * managing the communication and control of the AD5629R digital-to-
 * analog converter (DAC) within a system.
 *
 * @param i2c_desc Pointer to an I2C descriptor for I2C communication.
 * @param spi_desc Pointer to an SPI descriptor for SPI communication.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param gpio_clr Pointer to a GPIO descriptor for the CLR pin.
 * @param act_device Enumeration indicating the active device type.
 ******************************************************************************/
struct ad5629r_dev {
	/* I2C */
	struct no_os_i2c_desc		*i2c_desc;
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_ldac;
	struct no_os_gpio_desc	*gpio_clr;
	/* Device Settings */
	enum ad5629r_type		act_device;
};

/***************************************************************************//**
 * @brief The `ad5629r_init_param` structure is used to initialize the AD5629R
 * device, encapsulating the necessary parameters for I2C and SPI
 * communication, as well as GPIO configurations for the LDAC and CLR
 * pins. It also includes a field to specify the active device type,
 * allowing for flexible initialization of different devices within the
 * AD5629R family.
 *
 * @param i2c_init Initializes the I2C communication parameters.
 * @param spi_init Initializes the SPI communication parameters.
 * @param gpio_ldac Configures the GPIO parameters for the LDAC pin.
 * @param gpio_clr Configures the GPIO parameters for the CLR pin.
 * @param act_device Specifies the active device type from the ad5629r_type
 * enumeration.
 ******************************************************************************/
struct ad5629r_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_ldac;
	struct no_os_gpio_init_param	gpio_clr;
	/* Device Settings */
	enum ad5629r_type	act_device;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initializes the communication with the device. */
/***************************************************************************//**
 * @brief This function sets up the AD5629R device for communication by
 * initializing the necessary communication interfaces and GPIOs based on
 * the provided initialization parameters. It must be called before any
 * other operations on the device to ensure proper setup. The function
 * allocates memory for the device structure and configures the
 * communication interface (SPI or I2C) as specified. It also initializes
 * the LDAC and CLR GPIOs to their default states. If memory allocation
 * fails or any initialization step encounters an error, the function
 * returns a negative status code.
 *
 * @param device A pointer to a pointer of type `struct ad5629r_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad5629r_init_param` containing
 * initialization parameters for the device, including
 * communication settings and GPIO configurations. The
 * structure must be properly populated before calling the
 * function.
 * @return Returns 0 on success or a negative error code if initialization
 * fails.
 ******************************************************************************/
int8_t ad5629r_init(struct ad5629r_dev **device,
		    struct ad5629r_init_param init_param);
/* Free the resources allocated by AD5629R_Init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD5629R device instance when it is no longer needed. This function
 * should be called to clean up after a successful initialization with
 * `ad5629r_init`. It handles the removal of communication interfaces
 * (SPI or I2C) and any associated GPIOs, ensuring that all allocated
 * resources are freed. Failure to call this function may result in
 * resource leaks.
 *
 * @param dev A pointer to an `ad5629r_dev` structure representing the device
 * instance to be removed. Must not be null. The function will handle
 * the deallocation of resources associated with this device.
 * @return Returns an `int32_t` indicating the success or failure of the
 * resource removal process. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t ad5629r_remove(struct ad5629r_dev *dev);
/* Writes a value to Input Register N of selected DAC channel. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to the input register
 * of a designated DAC channel on the AD5629R device. It is typically
 * called when you need to set or update the input register for a
 * specific DAC channel without immediately updating the output. The
 * function requires a valid device structure and assumes that the device
 * has been properly initialized. It does not perform any updates to the
 * DAC output itself, only the input register is affected.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device. Must not be null.
 * @param dac_n The DAC channel number to which the value will be written. Valid
 * values are typically from 0 to 7, corresponding to DAC_A_ADDR to
 * DAC_H_ADDR.
 * @param dac_value The 16-bit value to write to the input register of the
 * specified DAC channel. The value should be within the
 * resolution supported by the device.
 * @return None
 ******************************************************************************/
void ad5629r_write_reg_n(struct ad5629r_dev *dev,
			 uint8_t dac_n,
			 uint16_t dac_value);
/* Updates selected DAC register. */
/***************************************************************************//**
 * @brief This function is used to update the register of a specific DAC channel
 * on the AD5629R device. It should be called when the input register of
 * a DAC channel has been set and the update of the DAC output is
 * required. The function requires a valid device structure and a DAC
 * channel identifier. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5629r_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param dac_n An 8-bit unsigned integer representing the DAC channel to
 * update. Valid values are typically within the range of available
 * DAC channels for the device.
 * @return None
 ******************************************************************************/
void ad5629r_update_dac_n(struct ad5629r_dev *dev,
			  uint8_t dac_n);
/* Writes a value to Input Register N of selected DAC channel, then updates all. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to the input register
 * of a selected DAC channel and subsequently update all DAC outputs with
 * the new value. It is typically called when a synchronized update of
 * all DAC channels is required after setting a new value to one of them.
 * The function must be called with a valid device structure that has
 * been properly initialized. It is important to ensure that the `dac_n`
 * parameter corresponds to a valid DAC channel address, and the
 * `dac_value` is within the acceptable range for the device's
 * resolution.
 *
 * @param dev A pointer to an initialized `ad5629r_dev` structure representing
 * the device. Must not be null.
 * @param dac_n The address of the DAC channel to which the value is written.
 * Must be a valid DAC address as defined by the device.
 * @param dac_value The value to write to the DAC's input register. Must be
 * within the range supported by the device's resolution.
 * @return None
 ******************************************************************************/
void ad5629r_write_reg_nupdate_all(struct ad5629r_dev *dev,
				   uint8_t dac_n,
				   uint16_t dac_value);
/* Writes a value to Input Register N and updates the respective DAC channel. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to the input register
 * of a designated DAC channel and immediately update that channel with
 * the new value. It is typically called when precise control over a
 * single DAC channel is required, ensuring that the input register and
 * the DAC output are synchronized. The function should be used after the
 * device has been properly initialized. It does not handle invalid DAC
 * channel numbers or values beyond the DAC's resolution, so the caller
 * must ensure these parameters are valid.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * DAC device. Must not be null.
 * @param dac_n The DAC channel number to be written to and updated. Valid
 * values are typically within the range of available DAC channels,
 * such as 0 to 7 for individual channels or 15 for all channels.
 * @param dac_value The value to write to the DAC channel's input register. Must
 * be within the valid range for the DAC's resolution,
 * typically 0 to 65535 for a 16-bit DAC.
 * @return None
 ******************************************************************************/
void ad5629r_write_reg_nupdate_n(struct ad5629r_dev *dev,
				 uint8_t dac_n,
				 uint16_t dac_value);
/* Sets the power mode for one or more selected DAC channels. */
/***************************************************************************//**
 * @brief Use this function to configure the power mode for one or more DAC
 * channels on the AD5629R device. This function is typically called when
 * you need to change the power state of the DAC channels, such as
 * putting them into a low-power mode or returning them to normal
 * operation. Ensure that the device has been properly initialized before
 * calling this function. The function does not return a value, and it is
 * the caller's responsibility to ensure that the parameters are valid.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device. Must not be null.
 * @param dac_sel A bitmask representing the DAC channels to configure. Valid
 * values are combinations of DAC_A_SEL, DAC_B_SEL, DAC_C_SEL,
 * DAC_D_SEL, DAC_E_SEL, DAC_F_SEL, DAC_G_SEL, DAC_H_SEL, or
 * DAC_ALL_ADDR to select all channels.
 * @param mode The power mode to set for the selected DAC channels. Valid values
 * are PWR_NORMAL, PWR_1K_TO_GND, PWR_100K_TO_GND, and PWR_3_STATE.
 * @return None
 ******************************************************************************/
void ad5629r_set_power_mode(struct ad5629r_dev *dev,
			    uint8_t dac_sel,
			    uint8_t mode);
/* Loads the Clear Code Register with a certain value. */
/***************************************************************************//**
 * @brief This function is used to set the Clear Code Register of the AD5629R
 * device to a specified value, which determines the output level when a
 * clear operation is triggered. It should be called when you need to
 * configure the behavior of the DAC outputs during a clear event. Ensure
 * that the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device. Must not be null.
 * @param clear_value An 8-bit value representing the clear code to be loaded
 * into the register. Valid values are typically defined by
 * the device's specifications.
 * @return None
 ******************************************************************************/
void ad5629r_load_clear_code_reg(struct ad5629r_dev *dev,
				 uint8_t clear_value);
/* Loads the LDAC register with a certain value. */
/***************************************************************************//**
 * @brief This function is used to load the LDAC (Load DAC) register of the
 * AD5629R device with a specified value, which determines the DAC
 * channels affected by subsequent commands. It should be called when you
 * need to configure which DAC channels will be updated by the LDAC pin.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device. Must not be null.
 * @param dac_sel A uint8_t value representing the DAC channels to be selected.
 * Valid values are combinations of DAC_A_SEL, DAC_B_SEL,
 * DAC_C_SEL, DAC_D_SEL, DAC_E_SEL, DAC_F_SEL, DAC_G_SEL, and
 * DAC_H_SEL. Invalid values may result in undefined behavior.
 * @return None
 ******************************************************************************/
void ad5629r_load_ldac_reg(struct ad5629r_dev *dev,
			   uint8_t dac_sel);
/* Makes a power-on reset. */
/***************************************************************************//**
 * @brief This function is used to reset the AD5629R device to its default
 * power-on state. It should be called when a reset of the device is
 * required, such as during initialization or when recovering from an
 * error state. The function requires a valid device structure that has
 * been initialized. It does not return any value and does not modify the
 * input structure beyond the reset operation.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device to be reset. Must not be null. The caller retains ownership
 * of the structure.
 * @return None
 ******************************************************************************/
void ad5629r_reset(struct ad5629r_dev *dev);
/* Turns on/off the internal reference. */
/***************************************************************************//**
 * @brief This function is used to control the internal reference of the AD5629R
 * device, allowing the user to enable or disable it as needed. It should
 * be called when there is a need to change the reference status, such as
 * during initialization or when switching between internal and external
 * references. The function requires a valid device structure and a
 * status indicating whether to turn the reference on or off. It does not
 * return a value and assumes the device has been properly initialized
 * before calling.
 *
 * @param dev A pointer to an initialized ad5629r_dev structure representing the
 * device. Must not be null.
 * @param status A uint8_t value indicating the desired reference status. Use
 * REF_ON to enable the internal reference and REF_OFF to disable
 * it. Invalid values may lead to undefined behavior.
 * @return None
 ******************************************************************************/
void ad5629r_set_ref(struct ad5629r_dev *dev,
		     uint8_t status);


#endif /* AD5629_H_ */

/***************************************************************************//**
*   @file   ad9833.h
*   @brief  Header file of AD9833 Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.******************************************************************************/
#ifndef _AD9833_H_
#define _AD9833_H_

/******************************************************************************/
/******************************* Include Files ********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************* Macros and Constants Definitions ***********************/
/******************************************************************************/

#define AD9833_CTRLB28          (1 << 13)
#define AD9833_CTRLHLB          (1 << 12)
#define AD9833_CTRLFSEL         (1 << 11)
#define AD9833_CTRLPSEL         (1 << 10)
#define AD9834_CTRLPINSW        (1 << 9)
#define AD9833_CTRLRESET        (1 << 8)
#define AD9833_CTRLSLEEP1       (1 << 7)
#define AD9833_CTRLSLEEP12      (1 << 6)
#define AD9833_CTRLOPBITEN      (1 << 5)
#define AD9834_CTRLSIGNPIB      (1 << 4)
#define AD9833_CTRLDIV2         (1 << 3)
#define AD9833_CTRLMODE         (1 << 1)

/* GPIOs */
#define AD9834_PSEL_OUT         no_os_gpio_direction_output(dev->gpio_psel,  \
			        NO_OS_GPIO_HIGH)
#define AD9834_PSEL_LOW         no_os_gpio_set_value(dev->gpio_psel,         \
			        NO_OS_GPIO_LOW)
#define AD9834_PSEL_HIGH        no_os_gpio_set_value(dev->gpio_psel,         \
			        NO_OS_GPIO_HIGH)

#define AD9834_FSEL_OUT         no_os_gpio_direction_output(dev->gpio_fsel,  \
			        NO_OS_GPIO_HIGH)
#define AD9834_FSEL_LOW         no_os_gpio_set_value(dev->gpio_fsel,         \
			        NO_OS_GPIO_LOW)
#define AD9834_FSEL_HIGH        no_os_gpio_set_value(dev->gpio_fsel,         \
			        NO_OS_GPIO_HIGH)

#define AD9834_RESET_OUT        no_os_gpio_direction_output(dev->gpio_reset, \
			        NO_OS_GPIO_HIGH)
#define AD9834_RESET_LOW        no_os_gpio_set_value(dev->gpio_reset,        \
			        NO_OS_GPIO_LOW)
#define AD9834_RESET_HIGH       pio_set_value(dev->gpio_reset,         \
			        NO_OS_GPIO_HIGH)

#define AD9834_SLEEP_OUT        no_os_gpio_direction_output(dev->gpio_sleep, \
			        NO_OS_GPIO_HIGH)
#define AD9834_SLEEP_LOW        no_os_gpio_set_value(dev->gpio_sleep,        \
			        NO_OS_GPIO_LOW)
#define AD9834_SLEEP_HIGH       no_os_gpio_set_value(dev->gpio_sleep,         \
			        NO_OS_GPIO_HIGH)


#define BIT_F0ADDRESS           0x4000      // Frequency Register 0 address.
#define BIT_F1ADDRESS           0x8000      // Frequency Register 1 address.
#define BIT_P0ADDRESS           0xC000      // Phase Register 0 address.
#define BIT_P1ADDRESS           0xE000      // Phase Register 1 address.

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad9833_type` enumeration defines a set of constants representing
 * different types of AD983x devices, specifically the AD9833, AD9834,
 * AD9837, and AD9838. This enumeration is used to specify the active
 * device type in the driver, allowing the software to handle different
 * models of the AD983x series of waveform generator ICs from Analog
 * Devices.
 *
 * @param ID_AD9833 Represents the AD9833 device type.
 * @param ID_AD9834 Represents the AD9834 device type.
 * @param ID_AD9837 Represents the AD9837 device type.
 * @param ID_AD9838 Represents the AD9838 device type.
 ******************************************************************************/
enum ad9833_type {
	ID_AD9833,
	ID_AD9834,
	ID_AD9837,
	ID_AD9838,
};

/***************************************************************************//**
 * @brief The `ad9833_dev` structure is designed to encapsulate the
 * configuration and control parameters for an AD9833 waveform generator
 * device. It includes pointers to SPI and GPIO descriptors for managing
 * communication and control signals, as well as fields for device-
 * specific settings such as the active device type, programming method,
 * and control register values. This structure is essential for
 * initializing and operating the AD9833 device, allowing for flexible
 * configuration and control of waveform generation.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_psel Pointer to a GPIO descriptor for phase select control.
 * @param gpio_fsel Pointer to a GPIO descriptor for frequency select control.
 * @param gpio_reset Pointer to a GPIO descriptor for reset control.
 * @param gpio_sleep Pointer to a GPIO descriptor for sleep control.
 * @param act_device Enumeration indicating the active AD9833 device type.
 * @param prog_method 8-bit value indicating the programming method.
 * @param ctrl_reg_value 16-bit value representing the control register
 * settings.
 * @param test_opbiten 16-bit value for testing the output bit enable.
 ******************************************************************************/
struct ad9833_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc		*gpio_psel;
	struct no_os_gpio_desc		*gpio_fsel;
	struct no_os_gpio_desc		*gpio_reset;
	struct no_os_gpio_desc		*gpio_sleep;
	/* Device Settings */
	enum ad9833_type		act_device;
	uint8_t				prog_method;
	uint16_t			ctrl_reg_value;
	uint16_t			test_opbiten;
};

/***************************************************************************//**
 * @brief The `ad9833_init_param` structure is used to initialize and configure
 * the AD9833 waveform generator device. It includes parameters for
 * setting up SPI communication and configuring GPIO pins for various
 * control signals such as PSEL, FSEL, RESET, and SLEEP. Additionally, it
 * specifies the type of AD9833 device being used, allowing for flexible
 * initialization of different models within the series.
 *
 * @param spi_init Initializes the SPI communication parameters for the AD9833
 * device.
 * @param gpio_psel Configures the GPIO parameters for the PSEL pin.
 * @param gpio_fsel Configures the GPIO parameters for the FSEL pin.
 * @param gpio_reset Configures the GPIO parameters for the RESET pin.
 * @param gpio_sleep Configures the GPIO parameters for the SLEEP pin.
 * @param act_device Specifies the active device type from the supported AD9833
 * series.
 ******************************************************************************/
struct ad9833_init_param {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* GPIO */
	struct no_os_gpio_init_param		gpio_psel;
	struct no_os_gpio_init_param		gpio_fsel;
	struct no_os_gpio_init_param		gpio_reset;
	struct no_os_gpio_init_param		gpio_sleep;
	/* Device Settings */
	enum ad9833_type		act_device;
};

/***************************************************************************//**
 * @brief The `ad9833_chip_info` structure is used to store essential
 * configuration parameters for the AD9833 waveform generator device. It
 * includes the master clock frequency (`mclk`) and a frequency constant
 * (`freq_const`) that are crucial for calculating and setting the output
 * frequency of the device. This structure is part of the AD9833 driver,
 * which facilitates communication and control of the device through SPI
 * and GPIO interfaces.
 *
 * @param mclk Represents the master clock frequency for the AD9833 device.
 * @param freq_const A constant used for frequency calculations in the AD9833
 * device.
 ******************************************************************************/
struct ad9833_chip_info {
	uint32_t	mclk;
	float		freq_const;
};

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/
/* Initialize the SPI communication with the device. */
/***************************************************************************//**
 * @brief This function sets up the AD9833 device by initializing its SPI
 * communication and configuring the necessary GPIO pins. It must be
 * called before any other operations on the device to ensure proper
 * setup. The function allocates memory for the device structure and
 * initializes the SPI and GPIO interfaces based on the provided
 * initialization parameters. It also resets the device and sets initial
 * frequency and phase values. The caller is responsible for providing
 * valid initialization parameters and handling the allocated device
 * structure.
 *
 * @param device A pointer to a pointer of type `struct ad9833_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and handle the allocated memory.
 * @param init_param A structure of type `struct ad9833_init_param` containing
 * initialization parameters for the SPI and GPIO interfaces,
 * as well as the specific device type. All fields must be
 * properly initialized before calling the function.
 * @return Returns an int8_t status code. A return value of 0 indicates success,
 * while a negative value indicates an error occurred during
 * initialization.
 ******************************************************************************/
int8_t ad9833_init(struct ad9833_dev **device,
		   struct ad9833_init_param init_param);
/* Free the resources allocated by adf7023_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * AD9833 device when it is no longer needed. This includes deallocating
 * memory and removing any associated SPI and GPIO descriptors. It is
 * important to call this function to prevent resource leaks after you
 * are done using the device. Ensure that the device pointer is valid and
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad9833_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The function will handle invalid pointers by
 * returning an error code.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a non-zero
 * value indicates an error occurred during resource deallocation.
 ******************************************************************************/
int32_t ad9833_remove(struct ad9833_dev *dev);
/* Transmits 16 bits on SPI. */
/***************************************************************************//**
 * @brief This function is used to send a 16-bit data value to the AD9833 device
 * via SPI communication. It should be called whenever a new control or
 * data word needs to be transmitted to the device. The function requires
 * a valid device structure that has been initialized with SPI settings.
 * If the SPI transmission fails, the function attempts to reset the
 * device by sending a reset command and then reinitializing it. This
 * function is typically used in the context of configuring or
 * controlling the AD9833 waveform generator.
 *
 * @param dev A pointer to an ad9833_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership and must ensure it is not null.
 * @param value A 16-bit integer representing the data to be transmitted. The
 * value is split into two 8-bit packets for transmission. There
 * are no specific constraints on the value, but it should
 * represent a valid command or data for the AD9833 device.
 * @return None
 ******************************************************************************/
void ad9833_tx_spi(struct ad9833_dev *dev,
		   int16_t value);
/* Selects the type of output. */
/***************************************************************************//**
 * @brief This function configures the output waveform mode of an AD9833 series
 * device, such as AD9833, AD9834, AD9837, or AD9838. It should be called
 * after the device has been initialized and is used to switch between
 * different waveform outputs like sinusoid, triangle, and DAC data
 * modes. The function modifies the control register of the device based
 * on the specified mode. It returns a status indicating success or
 * failure, particularly when the requested mode is not supported by the
 * device type.
 *
 * @param dev A pointer to an initialized ad9833_dev structure representing the
 * device. Must not be null.
 * @param out_mode An unsigned 8-bit integer specifying the desired output mode.
 * Valid values are 0 for sinusoid, 1 for triangle, 2 for DAC
 * Data MSB/2, and 3 for DAC Data MSB. Values outside this range
 * default to sinusoid.
 * @return Returns 0 on success, or -1 if the requested mode is not supported by
 * the device type.
 ******************************************************************************/
int8_t ad9833_out_mode(struct ad9833_dev *dev,
		       uint8_t v_out_mode);
/* Loads a frequency value in a register. */
/***************************************************************************//**
 * @brief This function configures the frequency of the AD9833 waveform
 * generator by writing a frequency value to one of its frequency
 * registers. It should be called after the device has been initialized
 * and is ready for configuration. The function allows selecting between
 * two frequency registers, enabling the user to switch between different
 * frequency settings. Care should be taken to ensure that the frequency
 * value is within the valid range for the device, and the correct
 * register number is specified.
 *
 * @param dev A pointer to an initialized ad9833_dev structure representing the
 * device. Must not be null.
 * @param register_number Specifies which frequency register to use (0 or 1).
 * Values outside this range may result in undefined
 * behavior.
 * @param frequency_value The desired frequency value to set. Must be within the
 * valid range supported by the device.
 * @return None
 ******************************************************************************/
void ad9833_set_freq(struct ad9833_dev *dev,
		     uint8_t register_number,
		     uint32_t frequency_value);
/* Loads a phase value in a register. */
/***************************************************************************//**
 * @brief This function is used to set the phase value for a specific phase
 * register in the AD9833 device. It should be called when you need to
 * update the phase of the waveform generated by the device. The function
 * requires a valid device structure and a phase value to be provided.
 * The phase value is converted and transmitted to the device via SPI.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad9833_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param register_number An unsigned 8-bit integer indicating which phase
 * register to use (0 or 1). Values outside this range
 * may result in undefined behavior.
 * @param phase_value A floating-point value representing the phase to be set.
 * The value is expected to be within a valid range for phase
 * settings, typically between 0 and 2π radians.
 * @return None
 ******************************************************************************/
void ad9833_set_phase(struct ad9833_dev *dev,
		      uint8_t register_number,
		      float phase_value);
/* Select the frequency register to be used. */
/***************************************************************************//**
 * @brief This function is used to select which frequency register (either 0 or
 * 1) will be used by the AD9833 device for signal generation. It must be
 * called after the device has been initialized and configured. The
 * function handles both software and hardware programming methods,
 * depending on the device's configuration. It is important to ensure
 * that the `dev` parameter is a valid, initialized device structure
 * before calling this function. The function does not return a value,
 * and it modifies the device's control register or GPIO state based on
 * the selected programming method.
 *
 * @param dev A pointer to an initialized `ad9833_dev` structure representing
 * the device. Must not be null.
 * @param freq_reg An unsigned 8-bit integer indicating the frequency register
 * to select. Valid values are 0 or 1. If an invalid value is
 * provided, the behavior is undefined.
 * @return None
 ******************************************************************************/
void ad9833_select_freq_reg(struct ad9833_dev *dev,
			    uint8_t freq_reg);
/* Select the phase register to be used. */
/***************************************************************************//**
 * @brief This function is used to select which phase register (0 or 1) is
 * active for the AD9833 device. It should be called when you need to
 * switch between different phase settings. The function requires that
 * the device has been properly initialized and configured. The selection
 * method depends on the programming method set in the device structure,
 * either through SPI communication or GPIO manipulation. Ensure that the
 * `dev` parameter is a valid pointer to an initialized `ad9833_dev`
 * structure.
 *
 * @param dev A pointer to an `ad9833_dev` structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param phase_reg An unsigned 8-bit integer indicating the phase register to
 * select. Valid values are 0 or 1. If an invalid value is
 * provided, the behavior is undefined.
 * @return None
 ******************************************************************************/
void ad9833_select_phase_reg(struct ad9833_dev *dev,
			     uint8_t phase_reg);
/* Enable / Disable the sleep function. */
/***************************************************************************//**
 * @brief This function configures the sleep mode of an AD9833 device, allowing
 * the user to power down the DAC, disable the internal clock, or both,
 * depending on the specified mode. It should be called when the device
 * needs to enter a low-power state. The function requires a valid device
 * structure and a sleep mode value, and it updates the device's control
 * register accordingly. The function's behavior varies based on the
 * programming method set in the device structure.
 *
 * @param dev A pointer to an ad9833_dev structure representing the device. Must
 * not be null, and the structure should be properly initialized
 * before calling this function.
 * @param sleep_mode An 8-bit unsigned integer specifying the sleep mode. Valid
 * values are 0 (no power-down), 1 (DAC powered down), 2
 * (internal clock disabled), and 3 (both DAC powered down and
 * internal clock disabled). Invalid values result in no
 * action.
 * @return None
 ******************************************************************************/
void ad9833_sleep_mode(struct ad9833_dev *dev,
		       uint8_t sleep_mode);

/***************************************************************************//**
 * @brief This function configures the AD9834 device to use either a software or
 * hardware programming method based on the provided value. It should be
 * called when there is a need to switch between programming methods,
 * typically during device initialization or reconfiguration. The
 * function updates the device's control register and internal
 * programming method state. Ensure that the device structure is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad9833_dev structure representing the
 * device. Must not be null.
 * @param value A uint8_t value indicating the desired programming method. Use 0
 * for software method and 1 for hardware method. Other values are
 * treated as 0 (software method).
 * @return None
 ******************************************************************************/
void ad9834_select_prog_method(struct ad9833_dev *dev,
			       uint8_t value);

/***************************************************************************//**
 * @brief This function is used to configure the logic output settings of an
 * AD9834 device by setting the operation bit enable, sign bit, and
 * divide by 2 options. It should be called when you need to adjust these
 * specific output settings on the device. The function modifies the
 * control register of the device based on the provided parameters and
 * updates the device's state accordingly. It is important to ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad9833_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param opbiten A uint8_t value indicating whether the operation bit is
 * enabled (1) or disabled (0). Values other than 0 or 1 are
 * treated as 0.
 * @param signpib A uint8_t value indicating whether the sign bit is enabled (1)
 * or disabled (0). Values other than 0 or 1 are treated as 0.
 * @param div2 A uint8_t value indicating whether the divide by 2 function is
 * enabled (1) or disabled (0). Values other than 0 or 1 are treated
 * as 0.
 * @return None
 ******************************************************************************/
void ad9834_logic_output(struct ad9833_dev *dev,
			 uint8_t opbiten,
			 uint8_t signpib,
			 uint8_t div2);

#endif  /* _AD9833_H_ */

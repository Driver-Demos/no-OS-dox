/***************************************************************************//**
*   @file   adf4156.h
*   @brief  Header file of ADF4156 Driver for Microblaze processor.
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
#ifndef _ADF4156_H_
#define _ADF4156_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************* Macros and Constants Definitions ***********************/
/******************************************************************************/
/* FRAC/INT Register R0 */
/* MUXOUT Control */
#define ADF4156_MUXOUT_Z            (0x00 << 27)
#define ADF4156_MUXOUT_DVdd         (0x01 << 27)
#define ADF4156_MUXOUT_DGND         (0x02 << 27)
#define ADF4156_MUXOUT_RDIV         (0x03 << 27)
#define ADF4156_MUXOUT_NDIV         (0x04 << 27)
#define ADF4156_MUXOUT_ALOCK        (0x05 << 27)
#define ADF4156_MUXOUT_DLOCK        (0x06 << 27)
#define ADF4156_MUXOUT_SDO          (0x07 << 27)
#define ADF4156_MUXOUT_CLKDIV       (0x0A << 27)
#define ADF4156_MUXOUT_FLS          (0x0C << 27)
#define ADF4156_MUXOUT_RDIV2        (0x0D << 27)
#define ADF4156_MUXOUT_NDIV2        (0x0E << 27)
#define ADF4156_MUXOUT_RESERVED     (0x0F << 27)
/* 12-Bit Integer Value INT */
#define ADF4156_INT_VAL(x)          ((x & 0xFFF) << 15)
/* 12-Bit Fractional Value FRAC */
#define ADF4156_FRAC_VAL(x)         ((x & 0xFFF) << 3)
/* Control bits */
#define ADF4156_R0_CTRL             0x00

/* PHASE Register R1 */
#define ADF4156_PHASE_VAL(x)        ((x & 0xFFF) << 3)
#define ADF4156_R1_CTRL             0x01

/* MOD/R Register R2 */
/* Noise Mode */
#define ADF4156_LOW_NOISE           (0x00 << 29)
#define ADF4156_LOW_SPUR            (0x03 << 29)
/* CSR EN */
#define ADF4156_CSR_EN(x)           ((x & 0x1) << 28)
/* Current setting */
#define ADF4156_CURR_SET(x)         ((x & 0xF) << 24)
/* Prescaler */
#define ADF4156_PRESCALER(x)        ((x & 0x1) << 22)
/* RDIV2 */
#define ADF4156_RDIV2(x)            ((x & 0x1) << 21)
/* Reference Doubler */
#define ADF4156_REF_DBL(x)          ((x & 0x1) << 20)
/* 5-Bit R-Counter */
#define ADF4156_R_CNT(x)            ((x & 0x1F) << 15)
/* 12-Bit Modulus Word */
#define ADF4156_MOD_WORD(x)         ((x & 0xFFF) << 3)
/* Control bits */
#define ADF4156_R2_CTRL             0x02

/* Function Register R3 */
/* Sigma Delta Reset */
#define ADF4156_SIG_DEL_RST(x)      ((x & 0x1) << 14)
/* LDP */
#define ADF4156_LDP(x)              ((x & 0x1) << 7)
/* PD Polarity */
#define ADF4156_PD_POL(x)           ((x & 0x1)  << 6)
/* Power down */
#define ADF4156_PD                  ((x & 0x1) << 5)
/* CP Three State */
#define ADF4156_CP_Z(x)             ((x & 0x1) << 4)
/* Counter Reset */
#define ADF4156_CNT_RST(x)          ((x & 0x1) << 3)
/* Control bits */
#define ADF4156_R3_CTRL             0x03

/* CLK DIV Register R4 */
/* CLK DIV Mode */
#define ADF4156_CLK_DIV_OFF         (0x00 << 19)
#define ADF4156_CLK_FAST_LOCK       (0x01 << 19)
#define ADF4156_CLK_RESYNC_EN       (0x02 << 19)
/* CLK DIV VALUE */
#define ADF4156_CLK_DIV_VAL(x)      ((x & 0xFFF) << 7)
/* Control bits */
#define ADF4156_R4_CTRL             0x04

/* GPIO */
#define ADF4156_LE_OUT              no_os_gpio_direction_output(dev->gpio_le,  \
			            NO_OS_GPIO_HIGH);
#define ADF4156_LE_LOW              no_os_gpio_set_value(dev->gpio_le,         \
			            NO_OS_GPIO_LOW)
#define ADF4156_LE_HIGH             no_os_gpio_set_value(dev->gpio_le,         \
			            NO_OS_GPIO_HIGH)

#define ADF4156_MUX_OUT             no_os_gpio_direction_output(dev->gpio_mux, \
			            NO_OS_GPIO_HIGH);
#define ADF4156_MUX_LOW             no_os_gpio_set_value(dev->gpio_mux,        \
			            NO_OS_GPIO_LOW)
#define ADF4156_MUX_HIGH            no_os_gpio_set_value(dev->gpio_mux,        \
			            NO_OS_GPIO_HIGH)

#define ADF4156_CE_OUT              no_os_gpio_direction_output(dev->gpio_ce,  \
			            NO_OS_GPIO_HIGH);
#define ADF4156_CE_LOW              no_os_gpio_set_value(dev->gpio_ce,         \
			            NO_OS_GPIO_LOW)
#define ADF4156_CE_HIGH             no_os_gpio_set_value(dev->gpio_ce,         \
			            NO_OS_GPIO_HIGH)

#define ADF4156_LE2_OUT             no_os_gpio_direction_output(dev->gpio_le2, \
			            NO_OS_GPIO_HIGH);
#define ADF4156_LE2_LOW             no_os_gpio_set_value(dev->gpio_le2,        \
			            NO_OS_GPIO_LOW)
#define ADF4156_LE2_HIGH            no_os_gpio_set_value(dev->gpio_le2,        \
			            NO_OS_GPIO_HIGH)

#define ADF4156_CE2_OUT             no_os_gpio_direction_output(dev->gpio_ce2, \
			            NO_OS_GPIO_HIGH);
#define ADF4156_CE2_LOW             no_os_gpio_set_value(dev->gpio_ce2,        \
			            NO_OS_GPIO_LOW)
#define ADF4156_CE2_HIGH            no_os_gpio_set_value(dev->gpio_ce2,        \
			            NO_OS_GPIO_HIGH)

/* Specifications */
#define ADF4156_MAX_OUT_FREQ        6200          /* MHz */
#define ADF4156_MIN_OUT_FREQ        500           /* MHz */
#define ADF4156_MAX_FREQ_45_PRESC   3000          /* MHz */
#define ADF4156_MAX_FREQ_PFD        32000000      /* Hz */
#define ADF4156_MAX_FREQ_REFIN      250000000     /* Hz */
#define ADF4156_MIN_FREQ_REFIN      10000000      /* Hz */
#define ADF4156_MAX_MODULUS         4095          /* (2^12)-1 */
#define ADF4156_MAX_R_CNT           32
#define ADF4156_MAX_REG_VAL         0x1FFFFFFF

/* Registers */
#define ADF4156_REG0                0
#define ADF4156_REG1                1
#define ADF4156_REG2                2
#define ADF4156_REG3                3
#define ADF4156_REG4                4

/******************************************************************************/
/**************************** Types Declarations ******************************/
/******************************************************************************/
/* Supported devices */
/*typedef enum {
    ID_ADF4156,
    ID_ADF4157,
} ADF4156_type;
*/
/***************************************************************************//**
 * @brief The `adf4156_platform_data` structure is used to configure the ADF4156
 * frequency synthesizer device. It contains parameters for the input
 * clock frequency, channel spacing, and flags to enable or disable
 * specific features like the reference doubler and divide-by-2.
 * Additionally, it holds user-defined settings for various registers
 * (R0, R2, R3, R4) that control the device's operation.
 *
 * @param clkin The input clock frequency in Hz.
 * @param channel_spacing The channel spacing frequency in Hz.
 * @param ref_doubler_en Flag to enable or disable the reference doubler.
 * @param ref_div2_en Flag to enable or disable the reference divide-by-2.
 * @param r0_user_settings User-defined settings for register R0.
 * @param r2_user_settings User-defined settings for register R2.
 * @param r3_user_settings User-defined settings for register R3.
 * @param r4_user_settings User-defined settings for register R4.
 ******************************************************************************/
struct adf4156_platform_data {
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint8_t		ref_doubler_en;
	uint8_t		ref_div2_en;
	uint32_t	r0_user_settings;
	uint32_t	r2_user_settings;
	uint32_t	r3_user_settings;
	uint32_t	r4_user_settings;
};

/***************************************************************************//**
 * @brief The `adf4156_state` structure is used to maintain the state of the
 * ADF4156 frequency synthesizer device. It includes configuration
 * parameters such as the phase frequency detector frequency, R-counter,
 * fractional and integer values for frequency setting, and modulus
 * value. Additionally, it holds an array of register values that
 * represent the current configuration of the device. This structure is
 * essential for managing the device's operation and ensuring accurate
 * frequency synthesis.
 *
 * @param pdata Pointer to adf4156_platform_data structure containing platform-
 * specific settings.
 * @param fpfd Phase Frequency Detector frequency in Hz.
 * @param r_cnt R-counter value used in frequency synthesis.
 * @param r0_fract Fractional part of the frequency setting.
 * @param r0_int Integer part of the frequency setting.
 * @param r2_mod Modulus value used in frequency synthesis.
 * @param reg_val Array holding the actual register values for the device.
 ******************************************************************************/
struct adf4156_state {
	struct adf4156_platform_data	*pdata;
	uint32_t	fpfd;           /* Phase Frequency Detector */
	uint16_t	r_cnt;          /* R-counter */
	uint32_t	r0_fract;       /* Fractional value */
	uint32_t	r0_int;         /* Integer value */
	uint32_t	r2_mod;         /* Modulus value */
	uint32_t	reg_val[5];     /* Actual register value */
};

/***************************************************************************//**
 * @brief The `adf4156_dev` structure is a compound data type used to represent
 * and manage the state of an ADF4156 device, which is a frequency
 * synthesizer. It includes pointers to SPI and GPIO descriptors for
 * communication and control, as well as a nested structure that holds
 * the device's configuration and operational parameters. This structure
 * is essential for interfacing with the ADF4156 hardware, allowing for
 * initialization, configuration, and frequency setting operations.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_le Pointer to a GPIO descriptor for the LE (Latch Enable) pin.
 * @param gpio_ce Pointer to a GPIO descriptor for the CE (Chip Enable) pin.
 * @param adf4156_st Structure containing the state and settings of the ADF4156
 * device.
 ******************************************************************************/
struct adf4156_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_le;
	struct no_os_gpio_desc	*gpio_ce;
	/* Device Settings */
	struct adf4156_state	adf4156_st;
};

/***************************************************************************//**
 * @brief The `adf4156_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the ADF4156 device.
 * It includes SPI initialization parameters and GPIO configurations for
 * line enable and chip enable, which are essential for establishing
 * communication and control with the device.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_le Contains the initialization parameters for the GPIO line
 * enable.
 * @param gpio_ce Contains the initialization parameters for the GPIO chip
 * enable.
 ******************************************************************************/
struct adf4156_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_le;
	struct no_os_gpio_init_param	gpio_ce;
};

/******************************************************************************/
/************************** Functions Declarations ****************************/
/******************************************************************************/
/* Initialize the SPI communication with the device. */
/***************************************************************************//**
 * @brief This function initializes the ADF4156 device by allocating necessary
 * resources and setting up the SPI and GPIO interfaces as specified in
 * the initialization parameters. It must be called before any other
 * operations on the ADF4156 device to ensure proper configuration. The
 * function configures the device registers with default settings and
 * prepares the device for operation. If the initialization fails, it
 * returns an error code, and the device pointer will not be valid.
 *
 * @param device A pointer to a pointer of type struct adf4156_dev. This will be
 * allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A struct adf4156_init_param containing the initialization
 * parameters for the SPI and GPIO interfaces. This includes
 * configuration details necessary for setting up
 * communication with the ADF4156 device.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. The device pointer is set to a valid device structure on
 * success.
 ******************************************************************************/
int8_t adf4156_init(struct adf4156_dev **device,
		    struct adf4156_init_param init_param);

/* Free the resources allocated by adf4156_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an ADF4156
 * device when it is no longer needed. This function should be called to
 * clean up after a successful initialization with `adf4156_init`. It
 * ensures that all associated SPI and GPIO resources are properly freed,
 * and the memory allocated for the device structure is released. Failure
 * to call this function may result in resource leaks.
 *
 * @param dev A pointer to an `adf4156_dev` structure representing the device to
 * be removed. Must not be null. The function will handle null
 * pointers gracefully by not attempting to free resources, but this
 * is not recommended as it may indicate a logic error in the calling
 * code.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a non-zero value indicates an error occurred during
 * the removal of SPI or GPIO resources.
 ******************************************************************************/
int32_t adf4156_remove(struct adf4156_dev *dev);

/* Transmits 32 bits on SPI. */
/***************************************************************************//**
 * @brief This function is used to send a 32-bit value to the ADF4156 device
 * over SPI communication. It should be called when you need to configure
 * or update the device's settings. The function requires a valid device
 * structure that has been initialized with the SPI descriptor. It
 * handles the transmission of the value by breaking it into four bytes
 * and sending them sequentially. The function returns a status
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to an adf4156_dev structure representing the device.
 * This must be a valid, initialized device structure with a properly
 * configured SPI descriptor. The caller retains ownership.
 * @param value A 32-bit unsigned integer representing the data to be
 * transmitted to the device. The value is split into four bytes
 * for transmission.
 * @return Returns an int8_t status code: 0 for success, or -1 if the
 * transmission fails.
 ******************************************************************************/
int8_t adf4156_set(struct adf4156_dev *dev,
		   uint32_t value);

/* Increases the R counter value until the PFD frequency is smaller than
ADF4351_MAX_FREQ_PFD. */
/***************************************************************************//**
 * @brief This function is used to increment the R counter value of the ADF4156
 * device until the Phase Frequency Detector (PFD) frequency is less than
 * or equal to the maximum allowed frequency. It should be called when
 * configuring the device to ensure that the PFD frequency does not
 * exceed the specified limit, which is crucial for proper device
 * operation. The function modifies the R counter and updates the PFD
 * frequency accordingly.
 *
 * @param dev A pointer to an adf4156_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param r_cnt An integer representing the initial R counter value. It is
 * incremented by the function until the PFD frequency is within
 * the allowed range.
 * @return Returns the adjusted R counter value as an integer.
 ******************************************************************************/
int32_t adf4156_tune_r_cnt(struct adf4156_dev *dev,
			   int32_t r_cnt);

/* Computes the greatest common divider of two numbers. */
uint32_t gcd(uint32_t x, uint32_t y);

/* Sets the ADF4156 output frequency. */
/***************************************************************************//**
 * @brief This function configures the ADF4156 device to output a specified
 * frequency, given in MHz. It should be called after the device has been
 * properly initialized. The function checks if the requested frequency
 * is within the allowable range and adjusts internal settings
 * accordingly. If the frequency is outside the valid range or if the
 * reference clock frequency is invalid, the function returns an error.
 * The function also handles prescaler settings based on the frequency
 * and ensures the internal counters are set correctly to achieve the
 * desired output frequency.
 *
 * @param dev A pointer to an initialized adf4156_dev structure. Must not be
 * null, and the device must be properly initialized before calling
 * this function.
 * @param freq The desired output frequency in MHz. Must be within the range of
 * 500 MHz to 6200 MHz. Frequencies outside this range will result
 * in an error return.
 * @return Returns the actual frequency set on the device in MHz if successful,
 * or -1 if an error occurs (e.g., invalid frequency or reference clock
 * settings).
 ******************************************************************************/
double adf4156_set_freq(struct adf4156_dev *dev,
			double freq);

#endif /* _ADF4156_H_ */

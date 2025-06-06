/***************************************************************************//**
*   @file   ADF4157.h
*   @brief  Header file of ADF4157 Driver for Microblaze processor.
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
#ifndef _ADF4157_H_
#define _ADF4157_H_

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
#define ADF4157_MUXOUT_Z            (0x00 << 27)
#define ADF4157_MUXOUT_DVdd         (0x01 << 27)
#define ADF4157_MUXOUT_DGND         (0x02 << 27)
#define ADF4157_MUXOUT_RDIV         (0x03 << 27)
#define ADF4157_MUXOUT_NDIV         (0x04 << 27)
#define ADF4157_MUXOUT_ALOCK        (0x05 << 27)
#define ADF4157_MUXOUT_DLOCK        (0x06 << 27)
#define ADF4157_MUXOUT_SDO          (0x07 << 27)
#define ADF4157_MUXOUT_CLKDIV       (0x0A << 27)
#define ADF4157_MUXOUT_FLS          (0x0C << 27)
#define ADF4157_MUXOUT_RDIV2        (0x0D << 27)
#define ADF4157_MUXOUT_NDIV2        (0x0E << 27)
#define ADF4157_MUXOUT_RESERVED     (0x0F << 27)
/* 12-Bit Integer Value INT */
#define ADF4157_INT_VAL(x)          ((x & 0xFFF) << 15)
/* 12-Bit Fractional Value FRAC */
#define ADF4157_FRAC_VAL_MSB(x)     ((x & 0x1FFE000) >> 10)
/* Control bits */
#define ADF4157_R0_CTRL             0x00

/* LSB FRAC Register R1 */
#define ADF4157_FRAC_VAL_LSB(x)     ((x & 0x1FFF) << 15)
#define ADF4157_R1_CTRL             0x01

/* MOD/R Register R2 */
/* Noise Mode */
#define ADF4157_LOW_NOISE           (0x00 << 29)
#define ADF4157_LOW_SPUR            (0x03 << 29)
/* CSR EN */
#define ADF4157_CSR_EN(x)           ((x & 0x1) << 28)
/* Current setting */
#define ADF4157_CURR_SET(x)         ((x & 0xF) << 24)
/* Prescaler */
#define ADF4157_PRESCALER(x)        ((x & 0x1) << 22)
/* RDIV2 */
#define ADF4157_RDIV2(x)            ((x & 0x1) << 21)
/* Reference Doubler */
#define ADF4157_REF_DBL(x)          ((x & 0x1) << 20)
/* 5-Bit R-Counter */
#define ADF4157_R_CNT(x)            ((x & 0x1F) << 15)
/* Control bits */
#define ADF4157_R2_CTRL             0x02

/* Function Register R3 */
/* Sigma Delta Reset */
#define ADF4157_SIG_DEL_RST(x)      ((x & 0x1) << 14)
/* LDP */
#define ADF4157_LDP(x)              ((x & 0x1) << 7)
/* PD Polarity */
#define ADF4157_PD_POL(x)           ((x & 0x1)  << 6)
/* Power down */
#define ADF4157_PD                  ((x & 0x1) << 5)
/* CP Three State */
#define ADF4157_CP_Z(x)             ((x & 0x1) << 4)
/* Counter Reset */
#define ADF4157_CNT_RST(x)          ((x & 0x1) << 3)
/* Control bits */
#define ADF4157_R3_CTRL             0x03

/* CLK DIV Register R4 */
/* NEG BLEED Current*/
#define ADF4157_NEG_BLEED_CUR(x)    ((x && 0x3) << 23)
/* CLK DIV Mode */
#define ADF4157_CLK_DIV_OFF         (0x00 << 19)
#define ADF4157_CLK_FAST_LOCK       (0x01 << 19)
#define ADF4157_CLK_RESYNC_EN       (0x02 << 19)
/* CLK DIV VALUE */
#define ADF4157_CLK_DIV_VAL(x)      ((x & 0xFFF) << 7)
/* Control bits */
#define ADF4157_R4_CTRL             0x04

/* GPIO */
#define ADF4157_LE_OUT              no_os_gpio_direction_output(dev->gpio_le,  \
			            NO_OS_GPIO_HIGH);
#define ADF4157_LE_LOW              no_os_gpio_set_value(dev->gpio_le,         \
			            NO_OS_GPIO_LOW)
#define ADF4157_LE_HIGH             no_os_gpio_set_value(dev->gpio_le,         \
			            NO_OS_GPIO_HIGH)

#define ADF4157_MUX_OUT             no_os_gpio_direction_output(dev->gpio_mux, \
			            NO_OS_GPIO_HIGH);
#define ADF4157_MUX_LOW             no_os_gpio_set_value(dev->gpio_mux,        \
			            NO_OS_GPIO_LOW)
#define ADF4157_MUX_HIGH            no_os_gpio_set_value(dev->gpio_mux,        \
			            NO_OS_GPIO_HIGH)

#define ADF4157_CE_OUT              no_os_gpio_direction_output(dev->gpio_ce,  \
			            NO_OS_GPIO_HIGH);
#define ADF4157_CE_LOW              no_os_gpio_set_value(dev->gpio_ce,         \
			            NO_OS_GPIO_LOW)
#define ADF4157_CE_HIGH             no_os_gpio_set_value(dev->gpio_ce,         \
			            NO_OS_GPIO_HIGH)

#define ADF4157_LE2_OUT             no_os_gpio_direction_output(dev->gpio_le2, \
			            NO_OS_GPIO_HIGH);
#define ADF4157_LE2_LOW             no_os_gpio_set_value(dev->gpio_le2,        \
			            NO_OS_GPIO_LOW)
#define ADF4157_LE2_HIGH            no_os_gpio_set_value(dev->gpio_le2,        \
			            NO_OS_GPIO_HIGH)

#define ADF4157_CE2_OUT             no_os_gpio_direction_output(dev->gpio_ce2, \
			            NO_OS_GPIO_HIGH);
#define ADF4157_CE2_LOW             no_os_gpio_set_value(dev->gpio_ce2,        \
			            NO_OS_GPIO_LOW)
#define ADF4157_CE2_HIGH            no_os_gpio_set_value(dev->gpio_ce2,        \
			            NO_OS_GPIO_HIGH)

/* Specifications */
#define ADF4157_MAX_OUT_FREQ        6000          /* MHz */
#define ADF4157_MIN_OUT_FREQ        500           /* MHz */
#define ADF4157_MAX_FREQ_45_PRESC   3000          /* MHz */
#define ADF4157_MAX_FREQ_PFD        32000000      /* Hz */
#define ADF4157_MAX_FREQ_REFIN      300000000     /* Hz */
#define ADF4157_MIN_FREQ_REFIN      10000000      /* Hz */
#define ADF4157_FIXED_MODULUS       33554432      /* 2^25 */
#define ADF4157_MAX_R_CNT           32
#define ADF4157_MAX_REG_VAL         0x1FFFFFFF

/* Registers */
#define ADF4157_REG0                0
#define ADF4157_REG1                1
#define ADF4157_REG2                2
#define ADF4157_REG3                3
#define ADF4157_REG4                4

/******************************************************************************/
/**************************** Types Declarations ******************************/
/***************************************************************************//**
 * @brief The `adf4157_platform_data` structure is used to configure the ADF4157
 * frequency synthesizer device. It contains fields for setting the input
 * clock frequency and enabling or disabling specific features such as
 * the reference doubler and divide-by-2. Additionally, it allows for
 * user-defined settings for several control registers, providing
 * flexibility in configuring the device's operation.
 *
 * @param clkin The input clock frequency in Hz.
 * @param ref_doubler_en Enables or disables the reference doubler.
 * @param ref_div2_en Enables or disables the reference divide-by-2.
 * @param r0_user_settings User-defined settings for register R0.
 * @param r2_user_settings User-defined settings for register R2.
 * @param r3_user_settings User-defined settings for register R3.
 * @param r4_user_settings User-defined settings for register R4.
 ******************************************************************************/
struct adf4157_platform_data {
	uint32_t	clkin;
	uint8_t		ref_doubler_en;
	uint8_t		ref_div2_en;
	uint32_t	r0_user_settings;
	uint32_t	r2_user_settings;
	uint32_t	r3_user_settings;
	uint32_t	r4_user_settings;
};

/***************************************************************************//**
 * @brief The `adf4157_state` structure is used to maintain the state of the
 * ADF4157 frequency synthesizer device. It includes configuration
 * parameters such as the phase frequency detector frequency (`fpfd`),
 * R-counter value (`r_cnt`), and both integer and fractional components
 * of the frequency setting (`r0_int` and `r0_fract`). Additionally, it
 * holds the modulus value (`r2_mod`) and channel spacing for frequency
 * synthesis, as well as an array of register values (`reg_val`) that
 * represent the current configuration of the device. The structure also
 * contains a pointer to `adf4157_platform_data`, which provides
 * platform-specific settings.
 *
 * @param pdata Pointer to adf4157_platform_data structure containing platform-
 * specific settings.
 * @param fpfd Phase Frequency Detector frequency in Hz.
 * @param r_cnt R-counter value used in frequency synthesis.
 * @param r0_fract Fractional part of the frequency setting.
 * @param r0_int Integer part of the frequency setting.
 * @param r2_mod Modulus value for frequency synthesis.
 * @param channel_spacing Spacing between channels in frequency synthesis.
 * @param reg_val Array holding the actual register values for the device.
 ******************************************************************************/
struct adf4157_state {
	struct adf4157_platform_data	*pdata;
	uint32_t			fpfd;               /* Phase Frequency Detector */
	uint16_t			r_cnt;              /* R-counter */
	uint32_t			r0_fract;           /* Fractional value */
	uint32_t			r0_int;             /* Integer value */
	uint32_t			r2_mod;             /* Modulus value */
	float				channel_spacing;    /* Channel spacing */
	uint32_t			reg_val[5];         /* Actual register value */
};

/***************************************************************************//**
 * @brief The `adf4157_dev` structure is a compound data type used to represent
 * and manage the state of an ADF4157 frequency synthesizer device. It
 * includes pointers to SPI and GPIO descriptors for communication and
 * control, as well as a nested `adf4157_state` structure that contains
 * detailed device settings and operational parameters. This structure is
 * essential for interfacing with the ADF4157 device, allowing for
 * configuration and control of its frequency synthesis capabilities.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_le Pointer to a GPIO descriptor for latch enable control.
 * @param gpio_ce Pointer to a GPIO descriptor for chip enable control.
 * @param adf4157_st Holds the state and configuration settings of the ADF4157
 * device.
 ******************************************************************************/
struct adf4157_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_le;
	struct no_os_gpio_desc	*gpio_ce;
	/* Device Settings */
	struct		adf4157_state adf4157_st;
};

/***************************************************************************//**
 * @brief The `adf4157_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the ADF4157 device.
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
struct adf4157_init_param {
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
 * @brief This function initializes the ADF4157 device by setting up the
 * necessary GPIO and SPI interfaces based on the provided initialization
 * parameters. It must be called before any other operations on the
 * ADF4157 device to ensure proper configuration. The function allocates
 * memory for the device structure and configures the device registers
 * with default and user-specified settings. If the initialization is
 * successful, a pointer to the device structure is returned; otherwise,
 * an error code is returned. Ensure that the device is properly removed
 * using `adf4157_remove` to free allocated resources.
 *
 * @param device A double pointer to a `struct adf4157_dev` where the
 * initialized device instance will be stored. Must not be null.
 * The caller takes ownership of the allocated memory and is
 * responsible for freeing it.
 * @param init_param A `struct adf4157_init_param` containing the initialization
 * parameters for the SPI and GPIO interfaces. Must be
 * properly configured before calling this function.
 * @return Returns 0 on successful initialization, or a negative error code if
 * initialization fails.
 ******************************************************************************/
int8_t adf4157_init(struct adf4157_dev **device,
		    struct adf4157_init_param init_param);

/* Free the resources allocated by adf4157_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an ADF4157
 * device when it is no longer needed. This function should be called to
 * clean up after a successful initialization with `adf4157_init`. It
 * ensures that the SPI and GPIO resources are properly released and the
 * memory allocated for the device structure is freed. The function
 * returns a status code indicating the success or failure of the
 * resource deallocation process.
 *
 * @param dev A pointer to an `adf4157_dev` structure representing the device to
 * be removed. This pointer must not be null, and the device must
 * have been previously initialized with `adf4157_init`. If the
 * pointer is invalid or null, the behavior is undefined.
 * @return Returns an `int32_t` status code. A value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the
 * resource deallocation process.
 ******************************************************************************/
int32_t adf4157_remove(struct adf4157_dev *dev);

/* Transmits 32 bits on SPI. */
/***************************************************************************//**
 * @brief This function is used to send a 32-bit value to the ADF4157 device
 * over SPI communication. It should be called when there is a need to
 * update the device's configuration or control registers with a new
 * value. The function requires a valid device structure that has been
 * initialized and configured for SPI communication. It handles the
 * necessary GPIO operations to ensure proper data transmission. The
 * function returns a status indicating the success or failure of the
 * operation.
 *
 * @param dev A pointer to an adf4157_dev structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null. The caller retains ownership.
 * @param value A 32-bit unsigned integer representing the data to be
 * transmitted to the device. The value is split into bytes and
 * sent over SPI.
 * @return Returns an int8_t status code: 0 for success, or -1 if the
 * transmission fails.
 ******************************************************************************/
int8_t adf4157_set(struct adf4157_dev *dev,
		   uint32_t value);

/* Increases the R counter value until the PFD frequency is smaller than
ADF4157_MAX_FREQ_PFD. */
/***************************************************************************//**
 * @brief This function is used to increment the R counter value of the ADF4157
 * device until the Phase Frequency Detector (PFD) frequency is less than
 * or equal to the maximum allowed frequency, ADF4157_MAX_FREQ_PFD. It is
 * typically called when configuring the device to ensure that the PFD
 * frequency remains within operational limits. The function must be
 * called with a valid device structure that has been properly
 * initialized. It modifies the R counter and updates the PFD frequency
 * accordingly.
 *
 * @param dev A pointer to an initialized adf4157_dev structure. Must not be
 * null. The function uses this structure to access device-specific
 * settings and state.
 * @param r_cnt An integer representing the initial R counter value. The
 * function will increment this value as needed to adjust the PFD
 * frequency.
 * @return Returns the adjusted R counter value as an integer, which ensures the
 * PFD frequency is within the specified limit.
 ******************************************************************************/
int32_t adf4157_tune_r_cnt(struct adf4157_dev *dev,
			   int32_t r_cnt);

/* Computes the greatest common divider of two numbers. */
uint32_t gcd(uint32_t x, uint32_t y);

/* Sets the ADF4157 output frequency. */
/***************************************************************************//**
 * @brief This function configures the ADF4157 device to output a specified
 * frequency within the allowable range. It should be called after the
 * device has been properly initialized. The function checks if the
 * requested frequency is within the valid range and adjusts the device's
 * settings accordingly. If the frequency is outside the permissible
 * range or if the reference input frequency is invalid, the function
 * returns an error. The function modifies the device's internal state to
 * achieve the desired frequency output.
 *
 * @param dev A pointer to an initialized adf4157_dev structure representing the
 * device. Must not be null.
 * @param freq The desired output frequency in MHz. Must be between
 * ADF4157_MIN_OUT_FREQ (500 MHz) and ADF4157_MAX_OUT_FREQ (6000
 * MHz). Values outside this range will result in an error return.
 * @return Returns the actual frequency set on the device in MHz, or -1 if an
 * error occurs due to invalid input parameters.
 ******************************************************************************/
double adf4157_set_freq(struct adf4157_dev *dev,
			double freq);

#endif /* _ADF4157_H_ */

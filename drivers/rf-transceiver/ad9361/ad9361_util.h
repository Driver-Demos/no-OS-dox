/***************************************************************************//**
 *   @file   ad9361_util.h
 *   @brief  AD9361 Header file of Util driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
#ifndef __AD9361_UTIL_H__
#define __AD9361_UTIL_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "ad9361.h"
#include "common.h"
#include "app_config.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define CLK_IGNORE_UNUSED						NO_OS_BIT(3)
#define CLK_GET_RATE_NOCACHE					NO_OS_BIT(6)

#if defined(HAVE_VERBOSE_MESSAGES)
#define dev_err(dev, format, ...)		({printf(format, ## __VA_ARGS__);printf("\n"); })
#define dev_warn(dev, format, ...)		({printf(format, ## __VA_ARGS__);printf("\n"); })
#if defined(HAVE_DEBUG_MESSAGES)
#define dev_dbg(dev, format, ...)		({printf(format, ## __VA_ARGS__);printf("\n"); })
#else
#define dev_dbg(dev, format, ...)	({ if (0) printf(format, ## __VA_ARGS__); })
#endif
#define printk(format, ...)			printf(format, ## __VA_ARGS__)
#else
#define dev_err(dev, format, ...)	({ if (0) printf(format, ## __VA_ARGS__); })
#define dev_warn(dev, format, ...)	({ if (0) printf(format, ## __VA_ARGS__); })
#define dev_dbg(dev, format, ...)	({ if (0) printf(format, ## __VA_ARGS__); })
#define printk(format, ...)			({ if (0) printf(format, ## __VA_ARGS__); })
#endif

/***************************************************************************//**
 * @brief The `device` structure is defined as an empty struct in the provided
 * code, indicating that it currently does not contain any members or
 * fields. It serves as a placeholder or base structure, potentially for
 * future expansion or to be used as a base type for other structures,
 * such as `spi_device`, which includes a `device` struct as a member.
 *
 ******************************************************************************/
struct device {
};

/***************************************************************************//**
 * @brief The 'spi_device' structure is a simple data structure used to
 * represent an SPI (Serial Peripheral Interface) device. It contains a
 * 'dev' member, which is a structure representing the associated device,
 * and an 'id_no' member, which is a unique identifier for the SPI
 * device. This structure is typically used in systems where multiple SPI
 * devices are managed, allowing each device to be uniquely identified
 * and associated with its corresponding hardware device structure.
 *
 * @param dev A member of type 'struct device' representing the device
 * associated with the SPI device.
 * @param id_no A uint8_t representing the identifier number of the SPI device.
 ******************************************************************************/
struct spi_device {
	struct device	dev;
	uint8_t 		id_no;
};

/***************************************************************************//**
 * @brief The `axiadc_state` structure is used to encapsulate the state of an
 * AXI ADC interface, specifically for the AD9361 RF transceiver. It
 * contains a pointer to the `ad9361_rf_phy` structure, which holds the
 * configuration and state of the RF physical layer, and a
 * `pcore_version` field that indicates the version of the programmable
 * core being used. This structure is essential for managing and
 * interfacing with the ADC hardware in a coherent manner.
 *
 * @param phy A pointer to an ad9361_rf_phy structure, representing the RF
 * physical layer configuration.
 * @param pcore_version A 32-bit unsigned integer representing the version of
 * the pcore.
 ******************************************************************************/
struct axiadc_state {
	struct ad9361_rf_phy	*phy;
	uint32_t				pcore_version;
};

/***************************************************************************//**
 * @brief The `axiadc_chip_info` structure is used to store information about an
 * ADC chip, specifically its name and the number of channels it
 * supports. This structure is likely used in the context of configuring
 * or managing ADC chips within a larger system, providing a way to
 * identify the chip and understand its capabilities in terms of channel
 * count.
 *
 * @param name A pointer to a character array representing the name of the ADC
 * chip.
 * @param num_channels An integer representing the number of channels the ADC
 * chip supports.
 ******************************************************************************/
struct axiadc_chip_info {
	char		*name;
	int32_t		num_channels;
};

/***************************************************************************//**
 * @brief The `axiadc_converter` structure is designed to represent an ADC
 * converter in a system, encapsulating both the chip-specific
 * information and a set of scratch registers for temporary data
 * handling. The `chip_info` member points to a `axiadc_chip_info`
 * structure, which provides details about the ADC chip, such as its name
 * and the number of channels it supports. The `scratch_reg` array serves
 * as a temporary storage space, allowing for efficient data manipulation
 * and processing within the ADC converter's operations.
 *
 * @param chip_info A pointer to an axiadc_chip_info structure, which contains
 * information about the ADC chip.
 * @param scratch_reg An array of 16 32-bit unsigned integers used as scratch
 * registers for temporary data storage.
 ******************************************************************************/
struct axiadc_converter {
	struct axiadc_chip_info	*chip_info;
	uint32_t				scratch_reg[16];
};

#ifdef WIN32
#include "basetsd.h"
typedef SSIZE_T ssize_t;
#define strsep(s, ct)				0
#define snprintf(s, n, format, ...)	0
#define __func__ __FUNCTION__
#endif

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is used to prepare and enable a clock represented by the
 * `no_os_clk` structure. It should be called when a clock needs to be
 * activated before use. The function expects a valid pointer to a
 * `no_os_clk` structure, and it will return immediately if the pointer
 * is null. This function is typically used in hardware initialization
 * sequences where clock management is necessary.
 *
 * @param clk A pointer to a `no_os_clk` structure representing the clock to be
 * prepared and enabled. The pointer must not be null, as passing a
 * null pointer will result in no operation being performed.
 * @return Returns 0 to indicate successful preparation and enabling of the
 * clock.
 ******************************************************************************/
int32_t clk_prepare_enable(struct no_os_clk *clk);
/***************************************************************************//**
 * @brief Use this function to obtain the current rate of a specific clock
 * source associated with an AD9361 RF PHY device. It is essential to
 * provide valid pointers to both the AD9361 RF PHY structure and the
 * reference clock scale structure. The function determines the clock
 * rate based on the source specified in the reference clock scale
 * structure. Ensure that the structures are properly initialized and
 * configured before calling this function to avoid undefined behavior.
 *
 * @param phy A pointer to a struct ad9361_rf_phy, representing the AD9361 RF
 * PHY device. Must not be null and should be properly initialized.
 * @param clk_priv A pointer to a struct refclk_scale, which specifies the clock
 * source for which the rate is to be retrieved. Must not be
 * null and should be properly initialized with a valid source.
 * @return Returns the clock rate as a uint32_t value. If the source is invalid
 * or not handled, the function returns 0.
 ******************************************************************************/
uint32_t clk_get_rate(struct ad9361_rf_phy *phy,
		      struct refclk_scale *clk_priv);
/***************************************************************************//**
 * @brief This function is used to set the desired clock rate for a specific
 * clock source within the AD9361 RF PHY structure. It should be called
 * when a change in the clock rate is required for a particular clock
 * source. The function handles various clock sources differently,
 * adjusting the rate accordingly and ensuring that the clock rates are
 * recalculated and updated throughout the system. It is important to
 * ensure that the `phy` and `clk_priv` structures are properly
 * initialized before calling this function. The function returns 0 on
 * successful execution.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the RF PHY
 * context. Must not be null and should be properly initialized
 * before use.
 * @param clk_priv A pointer to a `refclk_scale` structure that specifies the
 * clock source and scaling information. Must not be null and
 * should be properly initialized.
 * @param rate The desired clock rate to set, specified as a 32-bit unsigned
 * integer. The function will attempt to set the clock to this rate,
 * adjusting as necessary based on the clock source.
 * @return Returns 0 on success, indicating the clock rate was set successfully.
 ******************************************************************************/
int32_t clk_set_rate(struct ad9361_rf_phy *phy,
		     struct refclk_scale *clk_priv,
		     uint32_t rate);
/***************************************************************************//**
 * @brief Use this function to compute the integer square root of a given non-
 * negative integer. It is particularly useful when you need the largest
 * integer whose square is less than or equal to the input value. The
 * function handles input values of zero and one as special cases,
 * returning the input value itself. It is designed to work with 32-bit
 * unsigned integers, and the input must be within this range.
 *
 * @param x A 32-bit unsigned integer representing the non-negative number for
 * which the integer square root is to be calculated. The function
 * expects a valid non-negative integer within the range of 0 to
 * 2^32-1. Special cases for 0 and 1 are handled by returning the input
 * value directly.
 * @return Returns a 32-bit unsigned integer representing the largest integer
 * whose square is less than or equal to the input value.
 ******************************************************************************/
uint32_t int_sqrt(uint32_t x);
/***************************************************************************//**
 * @brief Use this function to determine the position of the highest set bit in
 * a positive integer, effectively computing the integer base-2
 * logarithm. This function is useful in scenarios where you need to
 * determine the power of two that is less than or equal to the given
 * number. The input must be a positive integer; behavior is undefined
 * for non-positive values. The function returns the logarithm as an
 * integer, which corresponds to the zero-based index of the highest set
 * bit.
 *
 * @param x A positive integer for which the base-2 logarithm is to be
 * calculated. Must be greater than zero. The function does not handle
 * non-positive values, and passing such values results in undefined
 * behavior.
 * @return Returns the integer base-2 logarithm of the input, which is the index
 * of the highest set bit in the binary representation of the input.
 ******************************************************************************/
int32_t ilog2(int32_t x);
/***************************************************************************//**
 * @brief Use this function to determine the position of the first bit set to 1
 * in a given 32-bit unsigned integer. This is useful in scenarios where
 * you need to quickly identify the least significant bit that is set.
 * The function returns the zero-based index of the first set bit,
 * starting from the least significant bit. If the input word is zero,
 * the function will return 32, indicating no bits are set.
 *
 * @param word A 32-bit unsigned integer representing the word to be examined.
 * The function expects a valid 32-bit integer and will return 32 if
 * no bits are set.
 * @return Returns a 32-bit unsigned integer representing the zero-based index
 * of the first set bit. If no bits are set, it returns 32.
 ******************************************************************************/
uint32_t find_first_bit(uint32_t word);
/***************************************************************************//**
 * @brief Use this function to convert a long integer error code into a pointer
 * type, which can be useful for returning error codes in functions that
 * typically return pointers. This function is particularly useful in
 * systems where error handling is done through pointer checks, allowing
 * the caller to distinguish between valid pointers and error codes.
 * Ensure that the error code does not conflict with valid memory
 * addresses to avoid misinterpretation.
 *
 * @param error A long integer representing the error code to be converted. The
 * value should be chosen such that it does not overlap with valid
 * memory addresses, as this could lead to incorrect error
 * handling.
 * @return Returns a void pointer that represents the given error code.
 ******************************************************************************/
/***************************************************************************//**
 * @brief `ERR_PTR` is a function that takes a long integer representing an
 * error code and returns a pointer. This function is typically used to
 * encode error values as pointers, which can be useful in certain
 * programming contexts where functions return pointers and need to
 * indicate errors.
 *
 * @details `ERR_PTR` is used to convert an error code into a pointer, allowing
 * functions that return pointers to also signal errors.
 ******************************************************************************/
void * ERR_PTR(long error);

#endif

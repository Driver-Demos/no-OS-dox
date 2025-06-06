/***************************************************************************//**
 *   @file   AD5755.h
 *   @brief  Header file of AD5755 Driver. This driver supporting the following
 *           devices: AD5755, AD5755-1 and AD5757
 *   @author Istvan Csomortani (istvan.csomortani@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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
#ifndef __AD5755_H__
#define __AD5755_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/******************* Macros and Constants Definitions *************************/
/******************************************************************************/

/******************************************************************************/
/******************************** AD5755 **************************************/
/******************************************************************************/

/* LDAC */
#define AD5755_LDAC_OUT        no_os_gpio_direction_output(dev->gpio_ldac,  \
			       NO_OS_GPIO_HIGH);
#define AD5755_LDAC_LOW        no_os_gpio_set_value(dev->gpio_ldac,         \
			       NO_OS_GPIO_LOW)
#define AD5755_LDAC_HIGH       no_os_gpio_set_value(dev->gpio_ldac,         \
			       NO_OS_GPIO_HIGH)

/* RESET */
#define AD5755_RESET_OUT       no_os_gpio_direction_output(dev->gpio_rst,  \
			       NO_OS_GPIO_HIGH);
#define AD5755_RESET_LOW       no_os_gpio_set_value(dev->gpio_rst,         \
			       NO_OS_GPIO_LOW)
#define AD5755_RESET_HIGH      no_os_gpio_set_value(dev->gpio_rst,         \
			       NO_OS_GPIO_HIGH)

/* CLEAR */
#define AD5755_CLEAR_OUT        no_os_gpio_direction_output(dev->gpio_clr, \
			        NO_OS_GPIO_HIGH);
#define AD5755_CLEAR_LOW        no_os_gpio_set_value(dev->gpio_clr,        \
			        NO_OS_GPIO_LOW)
#define AD5755_CLEAR_HIGH       no_os_gpio_set_value(dev->gpio_clr,        \
			        NO_OS_GPIO_HIGH)

/* POC */
#define AD5755_POC_OUT          no_os_gpio_direction_output(dev->gpio_poc, \
			        NO_OS_GPIO_HIGH);
#define AD5755_POC_LOW          no_os_gpio_set_value(dev->gpio_poc,        \
			        NO_OS_GPIO_LOW)
#define AD5755_POC_HIGH         no_os_gpio_set_value(dev->gpio_poc,        \
			        NO_OS_GPIO_HIGH)

/* Input Shift Register Contents for a Write Operation. */
#define AD5755_ISR_WRITE            (0ul << 23)           /* R/nW */
#define AD5755_ISR_DUT_AD1(x)       (((x) & 0x1) << 22)   /* Device AddrBit1*/
#define AD5755_ISR_DUT_AD0(x)       (((x) & 0x1) << 21)   /* Device AddrBit0*/
#define AD5755_ISR_DREG(x)          (((x) & 0x7) << 18)   /* Register AddrBits*/
#define AD5755_ISR_DAC_AD(x)        (((x) & 0x3) << 16)   /* Channel AddrBits */
#define AD5755_ISR_DATA(x)          ((x) & 0xFFFF)        /* Data Bits*/

/* Nop operation code. */
#define AD5755_ISR_NOP              0x1CE000

/* AD5755_ISR_DREG(x) options. (Register addresses) */
#define AD5755_DREG_WR_DAC          0
#define AD5755_DREG_WR_GAIN         2
#define AD5755_DREG_WR_GAIN_ALL     3
#define AD5755_DREG_WR_OFFSET       4
#define AD5755_DREG_WR_OFFSET_ALL   5
#define AD5755_DREG_WR_CLR_CODE     6
#define AD5755_DREG_WR_CTRL_REG     7

/* AD5755_ISR_DAC_AD(x) options. (Channel addresses) */
#define AD5755_DAC_A            0
#define AD5755_DAC_B            1
#define AD5755_DAC_C            2
#define AD5755_DAC_D            3

/* Gain register definition. */
#define AD5755_GAIN_ADJUSTMENT(x)       ((x) & 0xFFFF)

/* Offset register definition. */
#define AD5755_OFFSET_ADJUSTMENT(x)     ((x) & 0xFFFF)

/* Clear Code Register definition. */
#define AD5755_CLEAR_CODE(x)            ((x) & 0xFFFF)

/* Control Register definition. */
#define AD5755_CTRL_CREG(x)             (((x) & 0x7) << 13)
#define AD5755_CTRL_DATA(x)             ((x) & 0x1FFF)

/* AD5755_CTRL_CREG(x) options. */
#define AD5755_CREG_SLEW        0 // Slew rate control register(one per channel)
#define AD5755_CREG_MAIN        1 // Main control register
#define AD5755_CREG_DAC         2 // DAC control register(one per channel)
#define AD5755_CREG_DC_DC       3 // DC-to-dc control register
#define AD5755_CREG_SOFT        4 // Software register

/* Slew Rate Control Register definition. */
#define AD5755_SLEW_SREN            (1 << 12)
#define AD5755_SLEW_SR_CLOCK(x)     (((x) & 0xF) << 3)
#define AD5755_SLEW_SR_STEP(x)      (((x) & 0x7) << 0)

/* AD5755_SLEW_SR_CLOCK(x) options. */
#define AD5755_SR_CLK_64K       0
#define AD5755_SR_CLK_32k       1
#define AD5755_SR_CLK_16k       2
#define AD5755_SR_CLK_8K        3
#define AD5755_SR_CLK_4K        4
#define AD5755_SR_CLK_2K        5
#define AD5755_SR_CLK_1K        6
#define AD5755_SR_CLK_500       7
#define AD5755_SR_CLK_250       8
#define AD5755_SR_CLK_125       9
#define AD5755_SR_CLK_64        10
#define AD5755_SR_CLK_32        11
#define AD5755_SR_CLK_16        12
#define AD5755_SR_CLK_8         13
#define AD5755_SR_CLK_4         14
#define AD5755_SR_CLK_0_5       15

/* AD5755_SLEW_SR_STEP(x) options. */
#define AD5755_STEP_1       0
#define AD5755_STEP_2       1
#define AD5755_STEP_4       2
#define AD5755_STEP_16      3
#define AD5755_STEP_32      4
#define AD5755_STEP_64      5
#define AD5755_STEP_128     6
#define AD5755_STEP_256     7

/* Main Control Register definition. */
#define AD5755_MAIN_POC             (1 << 12)
#define AD5755_MAIN_STATREAD        (1 << 11)
#define AD5755_MAIN_EWD             (1 <<10)
#define AD5755_MAIN_WD(x)           (((x) & 0x3) << 8)
#define AD5755_MAIN_SHTCCTLIM(x)    (((x) & 0x1) << 6)
#define AD5755_MAIN_OUTEN_ALL       (1 << 5)
#define AD5755_MAIN_DCDC_ALL        (1 << 4)

/* AD5755_MAIN_WD(x) options. */
#define AD5755_WD_5MS               0 // 5 ms timeout period
#define AD5755_WD_10MS              1 // 10 ms timeout period
#define AD5755_WD_100MS             2 // 100 ms timeout period
#define AD5755_WD_200MS             3 // 200 ms timeout period

/* AD5755_MAIN_SHTCCTLIM(x) options. */
#define AD5755_LIMIT_16_MA          0 // 16 mA (default)
#define AD5755_LIMIT_8_MA           1 // 8 mA

/* DAC Control Register definition. */
#define AD5755_DAC_INT_ENABLE       (1 << 8)
#define AD5755_DAC_CLR_EN           (1 << 7)
#define AD5755_DAC_OUTEN            (1 << 6)
#define AD5755_DAC_RSET             (1 << 5)
#define AD5755_DAC_DC_DC            (1 << 4)
#define AD5755_DAC_OVRNG            (1 << 3)
#define AD5755_DAC_R(x)             ((x) & 0x7)

/* AD5755_DAC_R(x) options. */
#define AD5755_R_0_5_V              0 // 0 V to 5 V voltage range (default)
#define AD5755_R_0_10_V             1 // 0 V to 10 V voltage range
#define AD5755_R_M5_P5_V            2 // -5 V to +5 V voltage range
#define AD5755_R_M10_P10_V          3 // -10 V to 10 V voltage range
#define AD5755_R_4_20_MA            4 // 4 mA to 20 mA current range
#define AD5755_R_0_20_MA            5 // 0 mA to 20 mA current range
#define AD5755_R_0_24_MA            6 // 0 mA to 24 mA current range

/* DC-to-DC Control Register definition. */
#define AD5755_DC_DC_COMP           (1 << 6)
#define AD5755_DC_DC_PHASE(x)       (((x) & 0x3) << 4)
#define AD5755_DC_DC_FREQ(x)        (((x) & 0x3) << 2)
#define AD5755_DC_DC_MAX_V(x)       (((x) & 0x3) << 0)

/* AD5755_DC_DC_PHASE(x) options. */
#define AD5755_PHASE_ALL_DC_DC      0 // all dc-dc converters clock on same edge
#define AD5755_PHASE_AB_CD          1 // Ch A,B clk same edge, C,D opposite edge
#define AD5755_PHASE_AC_BD          2 // Ch A,C clk same edge, B,D opposite edge
#define AD5755_PHASE_A_B_C_D_90     3 // A,B,C,D clock 90 degree out of phase

/* AD5755_DC_DC_FREQ(x) options. */
#define AD5755_FREQ_250_HZ          0 // 250 +/- 10% kHz
#define AD5755_FREQ_410_HZ          1 // 410 +/- 10% kHz
#define AD5755_FREQ_650_HZ          2 // 650 +/- 10% kHz

/* AD5755_DC_DC_MAX_V(x) options. */
#define AD5755_MAX_23V          0 // 23 V + 1 V/-1.5 V (default)
#define AD5755_MAX_24_5V        1 // 24.5 V +/- 1 V
#define AD5755_MAX_27V          2 // 27 V +/- 1 V
#define AD5755_MAX_29_5V        3 // 29.5 V +/- 1V

/* Software Register definition. */
#define AD5755_SOFT_USER_BIT        (1 << 12)
#define AD5755_SOFT_RESET_CODE(x)   ((x) & 0xFFF)

/* AD5755_SOFT_RESET_CODE(x) options. */
#define AD5755_RESET_CODE       0x555 // Performs a reset of the AD5755.
#define AD5755_SPI_CODE         0x195 // If watchdog is enabled, 0x195 must be
// written to the software register within
// the programmed timeout period.

/* Input Shift Register Contents for a Read Operation. */
#define AD5755_ISR_READ             (1 << 23)
/* Same as Input Shift Register Contents for a Write Operation. */
/*
#define AD5755_ISR_DUT_AD1(x)       (((x) & 0x1) << 22)
#define AD5755_ISR_DUT_AD0(x)       (((x) & 0x1) << 21)
*/
#define AD5755_ISR_RD(x)            (((x) & 0x1F) << 16)

/* AD5755_ISR_RD(x) options. (Read address decoding) */
#define AD5755_RD_DATA_REG(x)           (((x) & 0x3) + 0)
#define AD5755_RD_CTRL_REG(x)           (((x) & 0x3) + 4)
#define AD5755_RD_GAIN_REG(x)           (((x) & 0x3) + 8)
#define AD5755_RD_OFFSET_REG(x)         (((x) & 0x3) + 12)
#define AD5755_RD_CODE_REG(x)           (((x) & 0x3) + 16)
#define AD5755_RD_SR_CTRL_REG(x)        (((x) & 0x3) + 20)
#define AD5755_RD_STATUS_REG            24
#define AD5755_RD_MAIN_CTRL_REG         25
#define AD5755_RD_Dc_DC_CTRL_REG        26

/* Status Register definition. */
/* channelA = 0 ... channelD = 3 */
#define AD5755_STATUS_DC_DC(x)          (1 << (12 + (x)))
#define AD5755_STATUS_USER_BIT          (1 << 11)
#define AD5755_STATUS_PEC_ERROR         (1 << 10)
#define AD5755_STATUS_RAMP_ACTIVE       (1 << 9)
#define AD5755_STATUS_OVER_TEMP         (1 << 8)
/* channelA = 0 ... channelD = 3 */
#define AD5755_STATUS_VOUT_FAULT(x)     (1 << (4 + (x)))
#define AD5755_STATUS_IOUT_FAULT(x)     (1 << (0 + (x)))

#define AD5755_CRC_POLYNOMIAL   0x07    // P(x)=x^8+x^2+x^1+1 = 100000111
#define AD5755_CRC_CHECK_CODE   0x00

/*****************************************************************************/
/************************** Types Declarations *******************************/
/*****************************************************************************/
/***************************************************************************//**
 * @brief The `ad5755_setup` structure is designed to store configuration
 * settings for the AD5755 device, which are applied during
 * initialization. It includes fields for setting the logic state of
 * external pins, enabling or disabling packet error checking,
 * configuring power-on conditions, and setting short-circuit limits.
 * Additionally, it allows for the selection of current sense resistors,
 * enabling voltage overrange, and configuring various parameters of the
 * dc-to-dc converter, such as compensation, phase, frequency, and
 * maximum voltage. This structure is essential for defining the
 * operational parameters of the AD5755 device.
 *
 * @param pin_ad0state Reflects the logic state of the external pin AD0, with a
 * range of 0 to 1.
 * @param pin_ad1state Reflects the logic state of the external pin AD1, with a
 * range of 0 to 1.
 * @param enable_packet_error_check Enables or disables Packet Error Checking
 * during SPI transfers, with a range of 0 to
 * 1.
 * @param poc_bit Determines the state of the voltage output channels during
 * normal operation based on the POC hardware pin, with a range
 * of 0 to 1.
 * @param stat_readbit Enables or disables status readback during a write, with
 * a range of 0 to 1.
 * @param sht_cc_lim_bit Sets a programmable short-circuit limit on the VOUT_x
 * pin, with options for 16 mA or 8 mA.
 * @param rset_bits Selects between an internal or external current sense
 * resistor for the selected DAC channel, with a range of 0 to
 * 1.
 * @param ovrng_bits Enables 20% overrange on the voltage output channel, with a
 * range of 0 to 1.
 * @param dc_dc_comp_bit Selects between an internal and external compensation
 * resistor for the dc-to-dc converter, with a range of 0
 * to 1.
 * @param dc_dc_phase_bit User-programmable dc-to-dc converter phase between
 * channels, with a range of 0 to 3.
 * @param dc_dc_freq_bit Sets the dc-to-dc switching frequency, with a range of
 * 0 to 2.
 * @param dc_dc_max_vbit Specifies the maximum allowed VBOOST_x voltage supplied
 * by the dc-to-dc converter, with a range of 0 to 3.
 ******************************************************************************/
struct ad5755_setup {
	/** Reflects the logic state of the external pin AD0. Range 0..1 */
	uint8_t pin_ad0state;
	/** Reflects the logic state of the external pin AD1. Range 0..1 */
	uint8_t pin_ad1state;
	/** Enables/Disables the Packet Error Checking that is used during all
	 * SPI transfers. Range 0..1
	 */
	uint8_t enable_packet_error_check;

	/** Power-On Condition. Determines the state of the voltage output
	 * channels during normal operation.
	 * 0 - The output goes to the value set by the POC hardware pin when
	 * the voltage output is not enabled;
	 * 1 - The output goes to the opposite value of the POC hardware pin
	 * if the voltage output is not enabled.
	 */
	uint8_t poc_bit;
	/** Enables/Disables status readback during a write. Range 0..1 */
	uint8_t stat_readbit;
	/** Programmable short-circuit limit on the VOUT_x pin in the event
	 * of a short-circuit condition: 0 - 16 mA
	 *                               1 - 8 mA
	 */
	uint8_t sht_cc_lim_bit;

	/** Selects an internal or external current sense resistor for the
	 * selected DAC channel: 0 - selects the external resistor
	 *                       1 - selects the internal resistor
	 */
	uint8_t rset_bits[4];
	/** Enables 20% overrange on voltage output channel only. Range 0..1 */
	uint8_t ovrng_bits[4];

	/** Selects between an internal and external compensation resistor
	 * for the dc-to-dc converter. Range 0..1
	 */
	uint8_t dc_dc_comp_bit;
	/** User programmable dc-to-dc converter phase (between channels).
	 *  Range 0..3
	 */
	uint8_t dc_dc_phase_bit;
	/** DC-to-dc switching frequency. Range 0..2 */
	uint8_t dc_dc_freq_bit;
	/** Maximum allowed VBOOST_x voltage supplied by the dc-to-dc
	 * converter. Range 0..3
	 */
	uint8_t dc_dc_max_vbit;
};

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5755_type_t` is an enumeration that defines the different types
 * of devices supported by the AD5755 driver, specifically the AD5755,
 * AD5755-1, and AD5757. This enumeration is used to identify and
 * differentiate between these device types within the driver code,
 * allowing for appropriate handling and configuration of each specific
 * device.
 *
 * @param ID_AD5755 Represents the AD5755 device type.
 * @param ID_AD5755_1 Represents the AD5755-1 device type.
 * @param ID_AD5757 Represents the AD5757 device type.
 ******************************************************************************/
enum ad5755_type_t {
	ID_AD5755,
	ID_AD5755_1,
	ID_AD5757,
};

/***************************************************************************//**
 * @brief The `ad5755_dev` structure is designed to encapsulate the necessary
 * components and configurations for interfacing with an AD5755 device,
 * which is a digital-to-analog converter (DAC). It includes pointers to
 * SPI and GPIO descriptors for communication and control, as well as a
 * pointer to a setup structure that holds device-specific settings. The
 * structure also contains an enumeration to specify the exact type of
 * AD5755 device being used, allowing for flexible support of different
 * models within the AD5755 family.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param gpio_rst Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_clr Pointer to a GPIO descriptor for the clear pin.
 * @param gpio_poc Pointer to a GPIO descriptor for the POC pin.
 * @param p_ad5755_st Pointer to a structure containing device settings.
 * @param this_device Enumeration indicating the specific AD5755 device type.
 ******************************************************************************/
struct ad5755_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_ldac;
	struct no_os_gpio_desc	*gpio_rst;
	struct no_os_gpio_desc	*gpio_clr;
	struct no_os_gpio_desc	*gpio_poc;
	/* Device Settings */
	struct ad5755_setup *p_ad5755_st;
	enum ad5755_type_t this_device;
};

/***************************************************************************//**
 * @brief The `ad5755_init_param` structure is used to encapsulate the
 * initialization parameters required to set up an AD5755 device. It
 * includes SPI initialization parameters, GPIO initialization parameters
 * for various control pins (LDAC, RST, CLR, POC), and a field to specify
 * the specific type of AD5755 device. This structure is essential for
 * configuring the device's communication and control interfaces before
 * it is used in an application.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_ldac Holds the initialization parameters for the LDAC GPIO pin.
 * @param gpio_rst Holds the initialization parameters for the RST GPIO pin.
 * @param gpio_clr Holds the initialization parameters for the CLR GPIO pin.
 * @param gpio_poc Holds the initialization parameters for the POC GPIO pin.
 * @param this_device Specifies the type of AD5755 device being used.
 ******************************************************************************/
struct ad5755_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_ldac;
	struct no_os_gpio_init_param	gpio_rst;
	struct no_os_gpio_init_param	gpio_clr;
	struct no_os_gpio_init_param	gpio_poc;
	/* Device Settings */
	enum ad5755_type_t this_device;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes the AD5755 device by setting up the
 * necessary GPIO and SPI interfaces, configuring the device's control
 * registers, and powering up all DAC channels. It should be called
 * before any other operations on the AD5755 device to ensure proper
 * setup. The function allocates memory for the device structure and
 * configures the device based on the provided initialization parameters.
 * It returns a status code indicating success or failure of the
 * initialization process.
 *
 * @param device A pointer to a pointer of type `struct ad5755_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad5755_init_param` containing
 * initialization parameters for the device, including SPI and
 * GPIO configurations. The structure must be properly
 * populated before calling this function.
 * @return Returns an int8_t status code: 0 for success, or a negative value
 * indicating an error during initialization.
 ******************************************************************************/
int8_t ad5755_init(struct ad5755_dev **device,
		   struct ad5755_init_param init_param);

/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD5755 device after it is no longer needed. It
 * ensures that the SPI and GPIO descriptors are removed and the memory
 * allocated for the device structure is freed. This function should be
 * called only after the device has been initialized and used, to prevent
 * resource leaks. It returns a status code indicating the success or
 * failure of the resource deallocation process.
 *
 * @param dev A pointer to an ad5755_dev structure representing the device to be
 * removed. Must not be null. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns an int32_t status code, where 0 indicates success and a non-
 * zero value indicates an error occurred during resource deallocation.
 ******************************************************************************/
int32_t ad5755_remove(struct ad5755_dev *dev);

/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in a specific
 * register of the AD5755 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function communicates with the device over SPI and returns the
 * register value. If packet error checking is enabled and a CRC error is
 * detected, the function will return -1 to indicate an error. This
 * function is useful for monitoring and debugging purposes, allowing you
 * to verify the current configuration or status of the device.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to read from. Must be a
 * valid register address for the AD5755 device.
 * @return Returns the 16-bit value of the specified register, or -1 if a CRC
 * error is detected when packet error checking is enabled.
 ******************************************************************************/
int32_t ad5755_get_register_value(struct ad5755_dev *dev,
				  uint8_t register_address);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the AD5755 device, targeting a particular channel. It is essential
 * for configuring the device's registers to control its behavior. The
 * function must be called with a valid device structure that has been
 * properly initialized. It supports optional packet error checking and
 * can return a status register value if status readback is enabled. This
 * function is typically used in device setup or configuration routines.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to write to. Valid values
 * depend on the device's register map.
 * @param channel The channel to which the register write is directed. Valid
 * values are typically 0 to 3, corresponding to channels A to D.
 * @param register_value The 16-bit value to write to the specified register.
 * Must be within the range of 0 to 65535.
 * @return Returns a 16-bit status register value if status readback is enabled;
 * otherwise, the return value is undefined.
 ******************************************************************************/
uint16_t ad5755_set_register_value(struct ad5755_dev *dev,
				   uint8_t register_address,
				   uint8_t channel,
				   uint16_t register_value);

/***************************************************************************//**
 * @brief Use this function to reset the AD5755 device to its default state via
 * software control. This is typically necessary when you want to ensure
 * the device is in a known state before starting configuration or
 * operation. The function should be called when the device is
 * initialized and ready to accept commands. It does not return any value
 * and does not provide feedback on the success of the reset operation.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device to reset. This pointer must not be null, and the device
 * must be properly initialized before calling this function.
 * @return None
 ******************************************************************************/
void ad5755_software_reset(struct ad5755_dev *dev);

/***************************************************************************//**
 * @brief This function enables or disables the watchdog timer on the AD5755
 * device and sets its timeout period. It should be used to ensure that
 * the device is periodically serviced to prevent it from entering a
 * fault state. The function must be called with a valid device structure
 * that has been initialized. The timeout period is specified in
 * predefined intervals, and the function will configure the device's
 * main control register accordingly.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param wtd_enable A uint8_t value where 0 disables the watchdog timer and 1
 * enables it. Values outside this range may lead to undefined
 * behavior.
 * @param timeout A uint8_t value specifying the timeout period for the watchdog
 * timer. Valid values are 0 (5 ms), 1 (10 ms), 2 (100 ms), and 3
 * (200 ms). Invalid values may result in incorrect
 * configuration.
 * @return None
 ******************************************************************************/
void ad5755_watch_dog_setup(struct ad5755_dev *dev,
			    uint8_t wtd_enable,
			    uint8_t timeout);

/***************************************************************************//**
 * @brief Use this function to service the watchdog timer on an AD5755 device,
 * ensuring it does not time out. This function should be called
 * periodically within the configured timeout period of the watchdog
 * timer to prevent it from triggering a reset or other protective
 * actions. It is essential to have the watchdog timer enabled and
 * properly configured before using this function.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * memory.
 * @return None
 ******************************************************************************/
void ad5755_feed_watch_dog_timer(struct ad5755_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure one of the control registers of the
 * AD5755 device for a specific channel. This function is typically
 * called after initializing the device and when you need to update the
 * control settings for a particular channel. Ensure that the device is
 * properly initialized before calling this function to avoid undefined
 * behavior.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param ctrl_reg_address The address of the control register to configure.
 * Valid values are defined by the AD5755_CTRL_CREG(x)
 * options.
 * @param channel The channel number to configure. Valid values are 0 to 3,
 * corresponding to channels A to D.
 * @param reg_value The value to write to the control register. This is a 16-bit
 * value that configures the register as needed.
 * @return None
 ******************************************************************************/
void ad5755_set_control_registers(struct ad5755_dev *dev,
				  uint8_t  ctrl_reg_address,
				  uint8_t  channel,
				  uint16_t reg_value);

/***************************************************************************//**
 * @brief Use this function to calculate an 8-bit CRC checksum for a given data
 * buffer using a specific polynomial. This function is typically used to
 * verify data integrity during communication with devices that support
 * CRC error checking. Ensure that the data buffer is valid and the
 * number of bytes specified is correct to avoid incorrect CRC results.
 *
 * @param data A pointer to the data buffer for which the CRC is to be
 * calculated. Must not be null, and the buffer should contain at
 * least 'bytes_number' bytes.
 * @param bytes_number The number of bytes in the data buffer to include in the
 * CRC calculation. Must be a non-zero value.
 * @return Returns an 8-bit CRC checksum calculated over the specified data
 * buffer.
 ******************************************************************************/
uint8_t ad5755_check_crc(uint8_t* data,
			 uint8_t bytes_number);

/***************************************************************************//**
 * @brief Use this function to power up or down the DC-to-DC converter, DAC, and
 * internal amplifiers for a specific channel on the AD5755 device. This
 * function is typically called when you need to enable or disable the
 * output of a channel, such as during initialization or when changing
 * the operational state of the device. Ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5755_dev structure representing the device
 * instance. Must not be null.
 * @param channel The channel to be configured, specified as an integer. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param pwr_status An integer indicating the desired power state. A non-zero
 * value enables the power, while zero disables it.
 * @return None
 ******************************************************************************/
void ad5755_set_channel_power(struct ad5755_dev *dev,
			      uint8_t channel,
			      uint8_t pwr_status);

/***************************************************************************//**
 * @brief Use this function to configure the output range of a specific channel
 * on the AD5755 device. This function must be called after the device
 * has been initialized and is typically used when you need to change the
 * output voltage or current range of a channel. It ensures that the
 * channel is set to the desired range and enables the output. The
 * function handles setting the output code to zero or midscale as
 * appropriate for the selected range.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure, typically ranging from 0 to
 * 3, corresponding to channels A to D.
 * @param range The desired output range for the channel, specified using
 * predefined macros such as AD5755_R_0_5_V, AD5755_R_0_10_V, etc.
 * Invalid range values may result in undefined behavior.
 * @return None
 ******************************************************************************/
void ad5755_set_channel_range(struct ad5755_dev *dev,
			      uint8_t channel,
			      uint8_t range);

/***************************************************************************//**
 * @brief This function configures whether a specific channel on the AD5755
 * device will clear its output when the CLEAR pin is activated. It is
 * typically used to control the behavior of the device in response to
 * external clear signals. This function should be called after the
 * device has been initialized and the channel has been set up. The
 * function does not return a value, and it modifies the control register
 * of the specified channel based on the provided parameters.
 *
 * @param dev A pointer to an ad5755_dev structure representing the device
 * instance. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param channel An unsigned 8-bit integer specifying the channel to configure.
 * Valid values are typically 0 to 3, corresponding to the
 * available channels on the device.
 * @param clear_en An unsigned 8-bit integer indicating whether to enable (non-
 * zero value) or disable (zero value) the clear functionality
 * for the specified channel.
 * @return None
 ******************************************************************************/
void ad5755_channel_clear_enable(struct ad5755_dev *dev,
				 uint8_t channel,
				 uint8_t clear_en);

/***************************************************************************//**
 * @brief This function is used to configure the digital slew rate control for a
 * specific channel on the AD5755 device. It allows enabling or disabling
 * the slew rate control, setting the update frequency, and defining the
 * step size for the slew rate. This function should be called when
 * precise control over the rate of change of the output signal is
 * required, such as in applications where sudden changes in output could
 * cause issues. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad5755_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param channel An integer specifying the channel to configure. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param sr_en An integer to enable or disable the slew rate control. Use 1 to
 * enable and 0 to disable.
 * @param updt_freq An integer specifying the update frequency for the slew
 * rate. Valid values range from 0 to 15, corresponding to
 * predefined frequency options.
 * @param step_size An integer specifying the step size for the slew rate. Valid
 * values range from 0 to 7, corresponding to predefined step
 * size options.
 * @return None
 ******************************************************************************/
void ad5755_slew_rate_ctrl(struct ad5755_dev *dev,
			   int8_t channel,
			   int8_t sr_en,
			   int8_t updt_freq,
			   int8_t step_size);

/***************************************************************************//**
 * @brief Use this function to set the desired output voltage on a specific
 * channel of the AD5755 device. It is essential to ensure that the
 * device is properly initialized and configured before calling this
 * function. The function calculates the necessary DAC value to achieve
 * the specified voltage, taking into account the channel's offset, gain,
 * and range settings. It then writes this value to the device. The
 * function returns the actual voltage set, which may differ slightly
 * from the requested voltage due to device characteristics and range
 * limitations.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to set the voltage on. Valid values are 0
 * to 3, corresponding to channels A to D.
 * @param voltage The desired output voltage to set on the specified channel.
 * The valid range depends on the channel's configured range,
 * which should be set prior to calling this function.
 * @return Returns the actual voltage set on the channel, which may differ from
 * the requested voltage due to device characteristics and range
 * limitations.
 ******************************************************************************/
float ad5755_set_voltage(struct ad5755_dev *dev,
			 uint8_t channel,
			 float voltage);

/***************************************************************************//**
 * @brief Use this function to configure the output current for a specific
 * channel on an AD5755 device. This function is typically called after
 * initializing the device and setting the desired channel range. It
 * calculates the appropriate DAC value based on the provided current and
 * writes it to the device, ensuring the output current matches the
 * specified value as closely as possible. The function returns the
 * actual current set, which may differ slightly due to device
 * characteristics and range settings.
 *
 * @param dev A pointer to an initialized ad5755_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to set the current for. Valid values are 0
 * to 3, corresponding to channels A to D.
 * @param m_acurrent The desired output current in milliamperes. The valid range
 * depends on the channel's configured range, typically
 * between 0 mA and 24 mA.
 * @return Returns the actual current set on the channel in milliamperes, which
 * may differ slightly from the requested current.
 ******************************************************************************/
float ad5755_set_current(struct ad5755_dev *dev,
			 uint8_t channel,
			 float m_acurrent);

#endif // __AD5755_H__

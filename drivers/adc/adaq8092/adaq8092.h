/***************************************************************************//**
 *   @file   adaq8092.h
 *   @brief  Header file of ADAQ8092 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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
#ifndef __ADAQ8092_H__
#define __ADAQ8092_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* SPI commands */
#define ADAQ8092_SPI_READ          	NO_OS_BIT(7)
#define ADAQ8092_ADDR(x)		((x) & 0xFF)

/* ADAQ8092 Register Map */
#define ADAQ8092_REG_RESET		0x00
#define ADAQ8092_REG_POWERDOWN		0x01
#define ADAQ8092_REG_TIMING		0x02
#define ADAQ8092_REG_OUTPUT_MODE	0x03
#define ADAQ8092_REG_DATA_FORMAT	0x04

/* ADAQ8092_REG_RESET Bit Definition */
#define ADAQ8092_RESET			NO_OS_BIT(7)

/* ADAQ8092_REG_POWERDOWN Bit Definition */
#define ADAQ8092_POWERDOWN_MODE		NO_OS_GENMASK(1, 0)

/* ADAQ8092_REG_TIMING Bit Definition */
#define ADAQ8092_CLK_INVERT		NO_OS_BIT(3)
#define ADAQ8092_CLK_PHASE		NO_OS_GENMASK(2, 1)
#define ADAQ8092_CLK_DUTYCYCLE		NO_OS_BIT(0)

/* ADAQ8092_REG_OUTPUT_MODE Bit Definition */
#define ADAQ8092_ILVDS			NO_OS_GENMASK(6, 4)
#define ADAQ8092_TERMON			NO_OS_BIT(3)
#define ADAQ8092_OUTOFF			NO_OS_BIT(2)
#define ADAQ8092_OUTMODE		NO_OS_GENMASK(1, 0)

/* ADAQ8092_REG_DATA_FORMAT Bit Definition */
#define ADAQ8092_OUTTEST		NO_OS_GENMASK(5, 3)
#define ADAQ8092_ABP			NO_OS_BIT(2)
#define ADAQ8092_RAND			NO_OS_BIT(1)
#define ADAQ8092_TWOSCOMP		NO_OS_BIT(0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* ADAQ8092 Power Down Modes */
/***************************************************************************//**
 * @brief The `adaq8092_powerdown_modes` enumeration defines the various power-
 * down modes available for the ADAQ8092 device, allowing for different
 * levels of power conservation by controlling the operational state of
 * its channels. This enum is used to manage the power state of the
 * device, providing options for normal operation, partial power-down
 * with one or both channels in nap mode, and a full sleep mode to
 * minimize power consumption.
 *
 * @param ADAQ8092_NORMAL_OP Represents the normal operation mode of the
 * ADAQ8092 device.
 * @param ADAQ8092_CH1_NORMAL_CH2_NAP Indicates that channel 1 is in normal
 * operation while channel 2 is in nap mode.
 * @param ADAQ8092_CH1_CH2_NAP Specifies that both channel 1 and channel 2 are
 * in nap mode.
 * @param ADAQ8092_SLEEP Denotes that the device is in sleep mode.
 ******************************************************************************/
enum adaq8092_powerdown_modes {
	ADAQ8092_NORMAL_OP,
	ADAQ8092_CH1_NORMAL_CH2_NAP,
	ADAQ8092_CH1_CH2_NAP,
	ADAQ8092_SLEEP
};

/* ADAQ8092 Output Clock Invert */
/***************************************************************************//**
 * @brief The `adaq8092_clk_invert` enumeration defines the possible clock
 * polarity modes for the ADAQ8092 device. It allows the user to specify
 * whether the clock signal should be in its normal state or inverted,
 * which can be crucial for synchronizing the device's operations with
 * other components in a system.
 *
 * @param ADAQ8092_CLK_POL_NORMAL Represents the normal clock polarity mode.
 * @param ADAQ8092_CLK_POL_INVERTED Represents the inverted clock polarity mode.
 ******************************************************************************/
enum adaq8092_clk_invert {
	ADAQ8092_CLK_POL_NORMAL,
	ADAQ8092_CLK_POL_INVERTED
};

/* ADAQ8092 Output Clock Phase Delay Bits */
/***************************************************************************//**
 * @brief The `adaq8092_clk_phase_delay` enumeration defines the possible phase
 * delay settings for the clock output of the ADAQ8092 device. It allows
 * the user to select between no delay, a 45-degree delay, a 90-degree
 * delay, or a 180-degree delay, which can be used to adjust the timing
 * of the clock signal for synchronization purposes.
 *
 * @param ADAQ8092_NO_DELAY Represents no delay in the clock phase.
 * @param ADAQ8092_CLKOUT_DELAY_45DEG Represents a 45-degree delay in the clock
 * phase.
 * @param ADAQ8092_CLKOUT_DELAY_90DEG Represents a 90-degree delay in the clock
 * phase.
 * @param ADAQ8092_CLKOUT_DELAY_180DEG Represents a 180-degree delay in the
 * clock phase.
 ******************************************************************************/
enum adaq8092_clk_phase_delay {
	ADAQ8092_NO_DELAY,
	ADAQ8092_CLKOUT_DELAY_45DEG,
	ADAQ8092_CLKOUT_DELAY_90DEG,
	ADAQ8092_CLKOUT_DELAY_180DEG
};

/*ADAQ8092 Clock Duty Cycle Stabilizer */
/***************************************************************************//**
 * @brief The `adaq8092_clk_dutycycle` enumeration defines the possible states
 * for the clock duty cycle stabilizer in the ADAQ8092 device. It
 * provides two options: turning the stabilizer off or on, which can be
 * used to control the stability of the clock signal's duty cycle in the
 * device's operation.
 *
 * @param ADAQ8092_CLK_DC_STABILIZER_OFF Represents the state where the clock
 * duty cycle stabilizer is turned off.
 * @param ADAQ8092_CLK_DC_STABILIZER_ON Represents the state where the clock
 * duty cycle stabilizer is turned on.
 ******************************************************************************/
enum adaq8092_clk_dutycycle {
	ADAQ8092_CLK_DC_STABILIZER_OFF,
	ADAQ8092_CLK_DC_STABILIZER_ON,
};

/* ADAQ8092 LVDS Output Current */
/***************************************************************************//**
 * @brief The `adaq8092_lvds_out_current` enumeration defines various levels of
 * LVDS (Low Voltage Differential Signaling) output current settings for
 * the ADAQ8092 device. Each enumerator corresponds to a specific current
 * value, allowing the user to configure the output current of the
 * device's LVDS interface to match the requirements of the connected
 * system or application. This configuration is crucial for ensuring
 * signal integrity and compatibility with different load conditions.
 *
 * @param ADAQ8092_3M5A Represents an LVDS output current of 3.5 mA.
 * @param ADAQ8092_4MA Represents an LVDS output current of 4 mA.
 * @param ADAQ8092_4M5A Represents an LVDS output current of 4.5 mA.
 * @param ADAQ8092_3MA Represents an LVDS output current of 3 mA.
 * @param ADAQ8092_2M5A Represents an LVDS output current of 2.5 mA.
 * @param ADAQ8092_2M1A Represents an LVDS output current of 2.1 mA.
 * @param ADAQ8092_1M75 Represents an LVDS output current of 1.75 mA.
 ******************************************************************************/
enum adaq8092_lvds_out_current {
	ADAQ8092_3M5A = 0,
	ADAQ8092_4MA = 1,
	ADAQ8092_4M5A = 2,
	ADAQ8092_3MA = 4,
	ADAQ8092_2M5A = 5,
	ADAQ8092_2M1A = 6,
	ADAQ8092_1M75 = 7
};

/* ADAQ8092 LVDS Internal Termination */
/***************************************************************************//**
 * @brief The `adaq8092_internal_term` enumeration defines the possible states
 * for the internal termination of the ADAQ8092 device. It provides two
 * options: `ADAQ8092_TERM_OFF` and `ADAQ8092_TERM_ON`, which are used to
 * control whether the internal termination is disabled or enabled,
 * respectively. This setting is crucial for configuring the electrical
 * characteristics of the device's output interface.
 *
 * @param ADAQ8092_TERM_OFF Represents the state where the internal termination
 * is turned off.
 * @param ADAQ8092_TERM_ON Represents the state where the internal termination
 * is turned on.
 ******************************************************************************/
enum adaq8092_internal_term {
	ADAQ8092_TERM_OFF,
	ADAQ8092_TERM_ON
};

/* ADAQ8092 Digital Output */
/***************************************************************************//**
 * @brief The `adaq8092_dout_enable` enumeration defines the possible states for
 * enabling or disabling the digital output of the ADAQ8092 device. It
 * provides two states: `ADAQ8092_DOUT_ON` for enabling the digital
 * output and `ADAQ8092_DOUT_OFF` for disabling it. This enumeration is
 * used to control the digital output functionality of the device,
 * allowing for easy toggling between the enabled and disabled states.
 *
 * @param ADAQ8092_DOUT_ON Represents the state where the digital output is
 * enabled.
 * @param ADAQ8092_DOUT_OFF Represents the state where the digital output is
 * disabled.
 ******************************************************************************/
enum adaq8092_dout_enable {
	ADAQ8092_DOUT_ON,
	ADAQ8092_DOUT_OFF
};

/* ADAQ8092 Digital Output Mode */
/***************************************************************************//**
 * @brief The `adaq8092_dout_modes` enumeration defines the different digital
 * output modes available for the ADAQ8092 device. These modes determine
 * how the digital output data is transmitted, either using CMOS or LVDS
 * signaling, and at what rate (full or double). This allows for
 * flexibility in interfacing the ADAQ8092 with different digital systems
 * depending on the required data rate and signaling standard.
 *
 * @param ADAQ8092_FULL_RATE_CMOS Represents the full rate CMOS digital output
 * mode.
 * @param ADAQ8092_DOUBLE_RATE_LVDS Represents the double rate LVDS digital
 * output mode.
 * @param ADAQ8092_DOUBLE_RATE_CMOS Represents the double rate CMOS digital
 * output mode.
 ******************************************************************************/
enum adaq8092_dout_modes {
	ADAQ8092_FULL_RATE_CMOS,
	ADAQ8092_DOUBLE_RATE_LVDS,
	ADAQ8092_DOUBLE_RATE_CMOS
};

/* ADAQ8092 Digital Test Pattern */
/***************************************************************************//**
 * @brief The `adaq8092_out_test_modes` enumeration defines various digital
 * output test patterns for the ADAQ8092 device. These test modes are
 * used to verify the digital output functionality by applying specific
 * bit patterns such as all ones, all zeros, checkerboard, and
 * alternating patterns. Each mode is associated with a specific integer
 * value that can be used to configure the device's output for testing
 * purposes.
 *
 * @param ADAQ8092_TEST_OFF Represents the test mode where no test pattern is
 * applied (value 0).
 * @param ADAQ8092_TEST_ONES Represents the test mode where all bits are set to
 * one (value 1).
 * @param ADAQ8092_TEST_ZEROS Represents the test mode where all bits are set to
 * zero (value 3).
 * @param ADAQ8092_TEST_CHECKERBOARD Represents the test mode with a
 * checkerboard pattern (value 5).
 * @param ADAQ8092_TEST_ALTERNATING Represents the test mode with an alternating
 * pattern (value 7).
 ******************************************************************************/
enum adaq8092_out_test_modes {
	ADAQ8092_TEST_OFF = 0,
	ADAQ8092_TEST_ONES = 1,
	ADAQ8092_TEST_ZEROS = 3,
	ADAQ8092_TEST_CHECKERBOARD = 5,
	ADAQ8092_TEST_ALTERNATING = 7
};

/* ADAQ8092 Alternate Bit Polarity Mode */
/***************************************************************************//**
 * @brief The `adaq8092_alt_bit_pol` enumeration defines the possible states for
 * the alternate bit polarity mode in the ADAQ8092 device. This mode can
 * be either off or on, allowing the user to control whether the
 * alternate bit polarity feature is enabled or disabled, which can be
 * useful for specific data output configurations.
 *
 * @param ADAQ8092_ALT_BIT_POL_OFF Represents the state where alternate bit
 * polarity is turned off.
 * @param ADAQ8092_ALT_BIT_POL_ON Represents the state where alternate bit
 * polarity is turned on.
 ******************************************************************************/
enum adaq8092_alt_bit_pol {
	ADAQ8092_ALT_BIT_POL_OFF,
	ADAQ8092_ALT_BIT_POL_ON
};

/* ADAQ8092 Data Output Randomizer*/
/***************************************************************************//**
 * @brief The `adaq8092_data_rand` enumeration defines the possible states for
 * the data output randomizer feature of the ADAQ8092 device. This
 * feature can be toggled between being off (`ADAQ8092_DATA_RAND_OFF`)
 * and on (`ADAQ8092_DATA_RAND_ON`), allowing for the randomization of
 * data output to potentially improve data integrity or security in
 * certain applications.
 *
 * @param ADAQ8092_DATA_RAND_OFF Represents the state where the data output
 * randomizer is turned off.
 * @param ADAQ8092_DATA_RAND_ON Represents the state where the data output
 * randomizer is turned on.
 ******************************************************************************/
enum adaq8092_data_rand {
	ADAQ8092_DATA_RAND_OFF,
	ADAQ8092_DATA_RAND_ON
};

/* ADAQ8092 Twos Complement Mode */
/***************************************************************************//**
 * @brief The `adaq8092_twoscomp` enumeration defines the data output format
 * options for the ADAQ8092 device, specifically allowing the selection
 * between offset binary and two's complement formats. This is crucial
 * for interpreting the digital output data correctly depending on the
 * application requirements.
 *
 * @param ADAQ8092_OFFSET_BINARY Represents the offset binary data format.
 * @param ADAQ8092_TWOS_COMPLEMENT Represents the two's complement data format.
 ******************************************************************************/
enum adaq8092_twoscomp {
	ADAQ8092_OFFSET_BINARY,
	ADAQ8092_TWOS_COMPLEMENT
};

/***************************************************************************//**
 * @brief The `adaq8092_init_param` structure is used to define the
 * initialization parameters for the ADAQ8092 device. It includes
 * pointers to SPI and GPIO initialization parameters, as well as various
 * configuration settings for power down modes, clock settings, LVDS
 * output current, digital output modes, and data formatting options.
 * This structure is essential for setting up the ADAQ8092 device with
 * the desired operational parameters before use.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param gpio_adc_pd1_param Pointer to the GPIO initialization parameters for
 * ADC power down 1.
 * @param gpio_adc_pd2_param Pointer to the GPIO initialization parameters for
 * ADC power down 2.
 * @param gpio_en_1p8_param Pointer to the GPIO initialization parameters for
 * enabling 1.8V.
 * @param gpio_par_ser_param Pointer to the GPIO initialization parameters for
 * parallel/serial mode.
 * @param pd_mode Specifies the power down mode of the ADAQ8092.
 * @param clk_pol_mode Specifies the clock polarity mode of the ADAQ8092.
 * @param clk_phase_mode Specifies the clock phase delay mode of the ADAQ8092.
 * @param clk_dc_mode Specifies the clock duty cycle stabilizer mode of the
 * ADAQ8092.
 * @param lvds_cur_mode Specifies the LVDS output current mode of the ADAQ8092.
 * @param lvds_term_mode Specifies the LVDS internal termination mode of the
 * ADAQ8092.
 * @param dout_en Specifies whether the digital output is enabled or disabled.
 * @param dout_mode Specifies the digital output mode of the ADAQ8092.
 * @param test_mode Specifies the digital output test pattern mode of the
 * ADAQ8092.
 * @param alt_bit_pol_en Specifies whether alternate bit polarity is enabled.
 * @param data_rand_en Specifies whether data output randomization is enabled.
 * @param twos_comp Specifies whether the data format is two's complement or
 * offset binary.
 ******************************************************************************/
struct adaq8092_init_param {
	/** Device communication descriptor */
	struct no_os_spi_init_param 	*spi_init;
	struct no_os_gpio_init_param	*gpio_adc_pd1_param;
	struct no_os_gpio_init_param	*gpio_adc_pd2_param;
	struct no_os_gpio_init_param	*gpio_en_1p8_param;
	struct no_os_gpio_init_param	*gpio_par_ser_param;
	enum adaq8092_powerdown_modes	pd_mode;
	enum adaq8092_clk_invert	clk_pol_mode;
	enum adaq8092_clk_phase_delay	clk_phase_mode;
	enum adaq8092_clk_dutycycle	clk_dc_mode;
	enum adaq8092_lvds_out_current	lvds_cur_mode;
	enum adaq8092_internal_term	lvds_term_mode;
	enum adaq8092_dout_enable	dout_en;
	enum adaq8092_dout_modes	dout_mode;
	enum adaq8092_out_test_modes	test_mode;
	enum adaq8092_alt_bit_pol	alt_bit_pol_en;
	enum adaq8092_data_rand		data_rand_en;
	enum adaq8092_twoscomp		twos_comp;
};

/***************************************************************************//**
 * @brief The `adaq8092_dev` structure is a comprehensive representation of the
 * ADAQ8092 device, encapsulating all necessary descriptors and
 * configuration settings required for its operation. It includes
 * pointers to SPI and GPIO descriptors for communication and control, as
 * well as various enumerations that define the device's power, clock,
 * LVDS, and output modes. This structure is essential for managing the
 * device's state and behavior, allowing for precise control over its
 * operational parameters.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param gpio_adc_pd1 Pointer to the GPIO descriptor for ADC power down control
 * 1.
 * @param gpio_adc_pd2 Pointer to the GPIO descriptor for ADC power down control
 * 2.
 * @param gpio_en_1p8 Pointer to the GPIO descriptor for enabling 1.8V power.
 * @param gpio_par_ser Pointer to the GPIO descriptor for parallel/serial mode
 * control.
 * @param pd_mode Specifies the power down mode of the device.
 * @param clk_pol_mode Specifies the clock polarity mode.
 * @param clk_phase_mode Specifies the clock phase delay mode.
 * @param clk_dc_mode Specifies the clock duty cycle stabilizer mode.
 * @param lvds_cur_mode Specifies the LVDS output current mode.
 * @param lvds_term_mode Specifies the LVDS internal termination mode.
 * @param dout_en Specifies whether digital outputs are enabled.
 * @param dout_mode Specifies the digital output mode.
 * @param test_mode Specifies the digital output test pattern mode.
 * @param alt_bit_pol_en Specifies whether alternate bit polarity is enabled.
 * @param data_rand_en Specifies whether data output randomization is enabled.
 * @param twos_comp Specifies the two's complement mode for data output.
 ******************************************************************************/
struct adaq8092_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	struct no_os_gpio_desc		*gpio_adc_pd1;
	struct no_os_gpio_desc		*gpio_adc_pd2;
	struct no_os_gpio_desc		*gpio_en_1p8;
	struct no_os_gpio_desc		*gpio_par_ser;
	enum adaq8092_powerdown_modes	pd_mode;
	enum adaq8092_clk_invert	clk_pol_mode;
	enum adaq8092_clk_phase_delay	clk_phase_mode;
	enum adaq8092_clk_dutycycle	clk_dc_mode;
	enum adaq8092_lvds_out_current	lvds_cur_mode;
	enum adaq8092_internal_term	lvds_term_mode;
	enum adaq8092_dout_enable	dout_en;
	enum adaq8092_dout_modes	dout_mode;
	enum adaq8092_out_test_modes	test_mode;
	enum adaq8092_alt_bit_pol	alt_bit_pol_en;
	enum adaq8092_data_rand		data_rand_en;
	enum adaq8092_twoscomp		twos_comp;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief This function is used to read a specific register from the ADAQ8092
 * device. It requires a valid device structure and a register address to
 * read from. The function will store the read data into the provided
 * memory location pointed to by reg_data. It is essential that the
 * device has been properly initialized before calling this function. The
 * function will return an error code if the read operation fails,
 * otherwise it returns 0 on success.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the device's address space.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adaq8092_read(struct adaq8092_dev *dev, uint8_t reg_addr,
		  uint8_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief Use this function to write a byte of data to a specific register on
 * the ADAQ8092 device. This function is typically used to configure the
 * device by setting various control registers. It requires a valid
 * device structure that has been properly initialized and a valid
 * register address. The function communicates with the device over SPI
 * and returns an error code if the write operation fails.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the ADAQ8092.
 * @param reg_data The data byte to write to the specified register.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the result of the SPI write operation.
 ******************************************************************************/
int adaq8092_write(struct adaq8092_dev *dev, uint8_t reg_addr,
		   uint8_t reg_data);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the
 * ADAQ8092 device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets the new bits as
 * specified by the reg_data parameter. This function is useful when only
 * certain bits of a register need to be changed without affecting the
 * other bits. It must be called with a valid device structure and
 * register address. The function returns an error code if the read or
 * write operation fails.
 *
 * @param dev Pointer to an adaq8092_dev structure representing the device. Must
 * not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADAQ8092 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, after applying
 * the mask. Only the bits specified by the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int adaq8092_update_bits(struct adaq8092_dev *dev, uint8_t reg_addr,
			 uint8_t mask, uint8_t reg_data);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the ADAQ8092 device by initializing its
 * communication interfaces and configuring it according to the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper setup. The function
 * allocates memory for the device structure and initializes SPI and GPIO
 * interfaces. It also configures various device modes such as power
 * down, clock polarity, and output modes. If any initialization step
 * fails, the function cleans up allocated resources and returns an error
 * code. The caller is responsible for providing valid initialization
 * parameters and handling the device pointer.
 *
 * @param device A pointer to a pointer of type `struct adaq8092_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adaq8092_init_param` containing
 * initialization parameters for the device. Must be properly
 * populated with valid SPI and GPIO initialization parameters
 * and device mode settings.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures that no resources are leaked.
 ******************************************************************************/
int adaq8092_init(struct adaq8092_dev **device,
		  struct adaq8092_init_param init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADAQ8092 device instance. It should be called when the device is no
 * longer needed to ensure that all associated resources, such as SPI and
 * GPIO descriptors, are correctly freed. This function must be called
 * after the device has been initialized and used, and it is important to
 * ensure that the `dev` parameter is valid and not null before calling
 * this function. Failure to call this function may result in resource
 * leaks.
 *
 * @param dev A pointer to an `adaq8092_dev` structure representing the device
 * to be removed. This pointer must not be null, and the structure
 * should have been previously initialized and used. The function
 * will handle invalid pointers by returning an error code.
 * @return Returns 0 on success or a negative error code if any of the resource
 * removals fail.
 ******************************************************************************/
int adaq8092_remove(struct adaq8092_dev *dev);

/* Set the device powerodown mode. */
/***************************************************************************//**
 * @brief This function configures the power-down mode of the ADAQ8092 device,
 * allowing the user to manage power consumption based on operational
 * needs. It should be called when a change in the device's power state
 * is required, such as transitioning to a low-power mode. The function
 * must be called with a valid device structure and a valid power-down
 * mode. If the operation is successful, the device's internal state is
 * updated to reflect the new mode. The function returns an error code if
 * the operation fails, which can be used for error handling.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type adaq8092_powerdown_modes indicating the
 * desired power-down mode. Must be a valid mode defined in the
 * enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_pd_mode(struct adaq8092_dev *dev,
			 enum adaq8092_powerdown_modes mode);

/* Get the device powerdown mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current power-down mode setting of
 * an ADAQ8092 device. It is useful for checking the device's power
 * state, especially in applications where power management is critical.
 * The function should be called only after the device has been properly
 * initialized. It does not modify the device state or any input
 * parameters.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before calling this function. If the
 * pointer is invalid, the behavior is undefined.
 * @return Returns the current power-down mode as an `enum
 * adaq8092_powerdown_modes` value, indicating the device's power state.
 ******************************************************************************/
enum adaq8092_powerdown_modes adaq8092_get_pd_mode(struct adaq8092_dev *dev);

/* Set the clock polarity mode. */
/***************************************************************************//**
 * @brief This function configures the clock polarity mode of the ADAQ8092
 * device, which determines the phase relationship between the clock
 * signal and the data. It should be called when the clock polarity needs
 * to be adjusted, typically during device initialization or
 * configuration changes. The function requires a valid device structure
 * and a clock polarity mode. It returns an error code if the operation
 * fails, ensuring that the device's state is not altered in case of an
 * error.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_clk_invert specifying the desired
 * clock polarity mode. Valid values are ADAQ8092_CLK_POL_NORMAL and
 * ADAQ8092_CLK_POL_INVERTED.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_clk_pol_mode(struct adaq8092_dev *dev,
			      enum adaq8092_clk_invert mode);

/* Get the clock polarity mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current clock polarity mode setting of
 * an ADAQ8092 device. This function is useful when you need to verify or
 * log the current configuration of the device's clock polarity. It is
 * expected that the device has been properly initialized before calling
 * this function. The function does not modify the device state or any
 * input parameters.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before calling this function.
 * @return Returns an `enum adaq8092_clk_invert` value indicating the current
 * clock polarity mode of the device.
 ******************************************************************************/
enum adaq8092_clk_invert adaq8092_get_clk_pol_mode(struct adaq8092_dev *dev);

/* Set the clock phase delay mode. */
/***************************************************************************//**
 * @brief This function configures the clock phase delay mode of the ADAQ8092
 * device, which affects the timing of the clock output. It should be
 * called when you need to adjust the clock phase delay to match specific
 * application requirements. The function updates the device's internal
 * register to reflect the new mode and stores the mode in the device
 * structure for future reference. It is important to ensure that the
 * device has been properly initialized before calling this function. If
 * the operation fails, an error code is returned.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type adaq8092_clk_phase_delay representing the
 * desired clock phase delay mode. Valid values are
 * ADAQ8092_NO_DELAY, ADAQ8092_CLKOUT_DELAY_45DEG,
 * ADAQ8092_CLKOUT_DELAY_90DEG, and ADAQ8092_CLKOUT_DELAY_180DEG.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adaq8092_set_clk_phase_mode(struct adaq8092_dev *dev,
				enum adaq8092_clk_phase_delay mode);

/* Get the clock phase delay mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current clock phase delay mode
 * setting of an ADAQ8092 device. It is typically called after the device
 * has been initialized and configured, to verify or utilize the current
 * clock phase delay setting. The function requires a valid device
 * structure pointer, which must have been previously initialized. It
 * does not modify the device state or any input parameters.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure. This parameter
 * must not be null, and the device must be properly initialized
 * before calling this function. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns the current clock phase delay mode as an enum
 * adaq8092_clk_phase_delay value, indicating the specific delay
 * setting.
 ******************************************************************************/
enum adaq8092_clk_phase_delay adaq8092_get_clk_phase_mode(
	struct adaq8092_dev *dev);

/* Set the clock duty cycle stabilizer mode. */
/***************************************************************************//**
 * @brief This function configures the clock duty cycle stabilizer mode of the
 * ADAQ8092 device. It should be called when you need to enable or
 * disable the clock duty cycle stabilizer feature. The function updates
 * the device's internal state to reflect the new mode. It is important
 * to ensure that the device has been properly initialized before calling
 * this function. The function returns an error code if the operation
 * fails, allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_clk_dutycycle indicating the
 * desired clock duty cycle stabilizer mode. Valid values are
 * ADAQ8092_CLK_DC_STABILIZER_OFF and ADAQ8092_CLK_DC_STABILIZER_ON.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_clk_dc_mode(struct adaq8092_dev *dev,
			     enum adaq8092_clk_dutycycle mode);

/* Get the clock duty cycle stabilizer mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current setting of the clock duty
 * cycle stabilizer mode for the ADAQ8092 device. This function is useful
 * for verifying the current configuration of the device's clock
 * settings. It should be called only after the device has been properly
 * initialized. The function does not modify any device settings or
 * state.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before calling this function.
 * @return Returns the current clock duty cycle stabilizer mode as an `enum
 * adaq8092_clk_dutycycle` value.
 ******************************************************************************/
enum adaq8092_clk_dutycycle adaq8092_get_clk_dc_mode(struct adaq8092_dev *dev);

/* Set the LVDS output current mode. */
/***************************************************************************//**
 * @brief This function configures the LVDS output current mode of the ADAQ8092
 * device. It should be called when you need to adjust the LVDS current
 * settings to match your system requirements. The function updates the
 * device's register to reflect the new mode and stores the mode in the
 * device structure for future reference. It is important to ensure that
 * the device is properly initialized before calling this function. If
 * the operation fails, an error code is returned.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_lvds_out_current representing the
 * desired LVDS output current mode. Valid values are ADAQ8092_3M5A,
 * ADAQ8092_4MA, ADAQ8092_4M5A, ADAQ8092_3MA, ADAQ8092_2M5A,
 * ADAQ8092_2M1A, and ADAQ8092_1M75.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_lvds_cur_mode(struct adaq8092_dev *dev,
			       enum adaq8092_lvds_out_current mode);

/* Get the LVDS output current mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current setting of the LVDS output
 * current mode from the ADAQ8092 device. It is typically called after
 * the device has been initialized and configured to verify or monitor
 * the current LVDS output current setting. The function requires a valid
 * device structure pointer and will return the current mode as an
 * enumerated value. Ensure that the device pointer is not null before
 * calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns an `enum adaq8092_lvds_out_current` value representing the
 * current LVDS output current mode.
 ******************************************************************************/
enum adaq8092_lvds_out_current adaq8092_get_lvds_cur_mode(
	struct adaq8092_dev *dev);

/* Set the LVDS internal temination mode. */
/***************************************************************************//**
 * @brief This function configures the LVDS internal termination mode of the
 * ADAQ8092 device. It should be called when you need to enable or
 * disable the internal termination of the LVDS outputs. Ensure that the
 * device is properly initialized before calling this function. The
 * function updates the device's configuration and stores the new mode in
 * the device structure. It returns an error code if the operation fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_internal_term indicating the
 * desired LVDS termination mode. Valid values are ADAQ8092_TERM_OFF
 * and ADAQ8092_TERM_ON.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_lvds_term_mode(struct adaq8092_dev *dev,
				enum adaq8092_internal_term mode);

/* Get the LVDS internal temination device mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current LVDS internal termination
 * mode setting of the ADAQ8092 device. It is useful for checking the
 * device's configuration regarding its LVDS termination state. This
 * function should be called only after the device has been properly
 * initialized and configured. It does not modify any device settings or
 * state.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before calling this function.
 * @return Returns an `enum adaq8092_internal_term` value indicating the current
 * LVDS internal termination mode of the device.
 ******************************************************************************/
enum adaq8092_internal_term adaq8092_get_lvds_term_mode(
	struct adaq8092_dev *dev);

/* Set digital outputs. */
/***************************************************************************//**
 * @brief This function configures the digital output enable mode of the
 * ADAQ8092 device. It should be called when you need to enable or
 * disable the digital outputs of the device. The function updates the
 * device's internal state to reflect the new mode. It is important to
 * ensure that the device is properly initialized before calling this
 * function. The function returns an error code if the operation fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_dout_enable indicating the desired
 * digital output enable mode. Valid values are ADAQ8092_DOUT_ON and
 * ADAQ8092_DOUT_OFF.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_dout_en(struct adaq8092_dev *dev,
			 enum adaq8092_dout_enable mode);

/* Get digital outputs. */
/***************************************************************************//**
 * @brief Use this function to obtain the current state of the digital output
 * enable setting for the ADAQ8092 device. This function is useful for
 * checking whether the digital outputs are currently enabled or
 * disabled. It should be called only after the device has been properly
 * initialized. The function does not modify any device state or
 * configuration.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. Must not be null. The function will not perform any
 * action if this parameter is invalid.
 * @return Returns an `enum adaq8092_dout_enable` value indicating whether the
 * digital outputs are enabled (`ADAQ8092_DOUT_ON`) or disabled
 * (`ADAQ8092_DOUT_OFF`).
 ******************************************************************************/
enum adaq8092_dout_enable adaq8092_get_dout_en(struct adaq8092_dev *dev);

/* Set the digital output mode. */
/***************************************************************************//**
 * @brief Use this function to configure the digital output mode of the ADAQ8092
 * device. It should be called when you need to change the output mode to
 * one of the predefined modes. Ensure that the device has been properly
 * initialized before calling this function. The function updates the
 * device's internal state to reflect the new mode and returns an error
 * code if the operation fails.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_dout_modes specifying the desired
 * digital output mode. Valid values are ADAQ8092_FULL_RATE_CMOS,
 * ADAQ8092_DOUBLE_RATE_LVDS, and ADAQ8092_DOUBLE_RATE_CMOS.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adaq8092_set_dout_mode(struct adaq8092_dev *dev,
			   enum adaq8092_dout_modes mode);

/* Get the digital output mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current digital output mode setting of
 * an ADAQ8092 device. This function is useful for verifying the current
 * configuration of the device's digital output mode, which can be one of
 * several predefined modes. It is important to ensure that the device
 * has been properly initialized before calling this function to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before use.
 * @return Returns an `enum adaq8092_dout_modes` value indicating the current
 * digital output mode of the device.
 ******************************************************************************/
enum adaq8092_dout_modes adaq8092_get_dout_mode(struct adaq8092_dev *dev);

/* Set digital output test pattern mode. */
/***************************************************************************//**
 * @brief This function configures the ADAQ8092 device to use a specified
 * digital output test pattern mode. It should be called when you need to
 * change the test mode of the device's digital outputs. The function
 * requires a valid device structure and a test mode enumeration value.
 * It updates the device's internal state to reflect the new test mode.
 * If the operation fails, an error code is returned. Ensure the device
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enumeration value of type adaq8092_out_test_modes specifying
 * the desired test pattern mode. Valid values are defined in the
 * adaq8092_out_test_modes enum.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adaq8092_set_test_mode(struct adaq8092_dev *dev,
			   enum adaq8092_out_test_modes mode);

/* Get digital output test pattern mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current digital output test
 * pattern mode of the ADAQ8092 device. It is useful for verifying the
 * test mode configuration of the device. The function should be called
 * when the device is properly initialized and configured. It does not
 * modify the device state or any input parameters.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. Must not be null. The function will not modify the
 * contents of this structure.
 * @return Returns the current test mode as an `enum adaq8092_out_test_modes`
 * value, indicating the digital output test pattern mode.
 ******************************************************************************/
enum adaq8092_out_test_modes adaq8092_get_test_mode(struct adaq8092_dev *dev);

/* Set the alternate bit polarity mode. */
/***************************************************************************//**
 * @brief Use this function to configure the alternate bit polarity mode of the
 * ADAQ8092 device. This function should be called when you need to
 * change the bit polarity mode to either enable or disable alternate bit
 * polarity. Ensure that the device is properly initialized before
 * calling this function. The function updates the device's internal
 * state and the corresponding register to reflect the new mode. It
 * returns an error code if the operation fails, which should be checked
 * to ensure successful configuration.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type adaq8092_alt_bit_pol indicating the desired
 * alternate bit polarity mode. Valid values are
 * ADAQ8092_ALT_BIT_POL_OFF and ADAQ8092_ALT_BIT_POL_ON.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adaq8092_set_alt_pol_en(struct adaq8092_dev *dev,
			    enum adaq8092_alt_bit_pol mode);

/* Get the alternate bit polarity mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current setting of the alternate
 * bit polarity mode for the ADAQ8092 device. It is typically called
 * after the device has been initialized and configured, to verify or
 * utilize the current alternate bit polarity configuration. The function
 * requires a valid device structure pointer, which must have been
 * previously initialized. It does not modify the device state or
 * configuration.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. This pointer must not be null, and the structure must
 * be properly initialized before calling this function. If the
 * pointer is invalid, the behavior is undefined.
 * @return Returns an `enum adaq8092_alt_bit_pol` value indicating the current
 * alternate bit polarity mode setting of the device.
 ******************************************************************************/
enum adaq8092_alt_bit_pol adaq8092_get_alt_pol_en(struct adaq8092_dev *dev);

/* Set the data output randomizer mode. */
/***************************************************************************//**
 * @brief This function configures the data output randomizer mode of the
 * ADAQ8092 device, which can be used to enable or disable data
 * randomization. It should be called when you need to change the
 * randomization setting of the device's data output. The function
 * requires a valid device structure and a mode specifying whether to
 * enable or disable the randomizer. It updates the device's internal
 * state to reflect the new mode. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adaq8092_data_rand indicating the desired
 * randomizer mode. Valid values are ADAQ8092_DATA_RAND_OFF and
 * ADAQ8092_DATA_RAND_ON.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adaq8092_set_data_rand_en(struct adaq8092_dev *dev,
			      enum adaq8092_data_rand mode);

/* Get the data output randomizer mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current setting of the data output
 * randomizer mode for the ADAQ8092 device. This function is useful when
 * you need to verify or log the current configuration of the device. It
 * should be called only after the device has been properly initialized.
 * The function does not modify any device settings or state.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. Must not be null. The function will not perform any
 * action if this parameter is invalid.
 * @return Returns an `enum adaq8092_data_rand` value indicating whether the
 * data output randomizer is enabled or disabled.
 ******************************************************************************/
enum adaq8092_data_rand adaq8092_get_data_rand_en(struct adaq8092_dev *dev);

/* Set the Tows Complement mode. */
/***************************************************************************//**
 * @brief Use this function to configure the ADAQ8092 device to operate in
 * either offset binary or two's complement mode. This setting affects
 * how the device interprets and outputs digital data. It is essential to
 * call this function after initializing the device and before starting
 * data acquisition to ensure the data format is correctly set. The
 * function updates the device's internal state to reflect the selected
 * mode. If the operation fails, an error code is returned, and the mode
 * is not changed.
 *
 * @param dev A pointer to an initialized adaq8092_dev structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type adaq8092_twoscomp indicating the desired
 * data format mode. Valid values are ADAQ8092_OFFSET_BINARY and
 * ADAQ8092_TWOS_COMPLEMENT.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adaq8092_set_twos_comp(struct adaq8092_dev *dev,
			   enum adaq8092_twoscomp mode);

/* Get the Tows Complement mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current two's complement mode
 * setting of the ADAQ8092 device. It should be called when you need to
 * verify or utilize the current data format configuration of the device.
 * Ensure that the device has been properly initialized before calling
 * this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized `adaq8092_dev` structure representing
 * the device. Must not be null. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns the current two's complement mode as an `enum
 * adaq8092_twoscomp`, indicating whether the device is set to offset
 * binary or two's complement mode.
 ******************************************************************************/
enum adaq8092_twoscomp adaq8092_get_twos_comp(struct adaq8092_dev *dev);

#endif /* __ADAQ8092_H__ */

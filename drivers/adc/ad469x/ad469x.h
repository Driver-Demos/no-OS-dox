/***************************************************************************//**
 *   @file   ad469x.h
 *   @brief  Header file for ad469x Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2020-22(c) Analog Devices, Inc.
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

#ifndef SRC_AD469X_H_
#define SRC_AD469X_H_

// **** Note for User: SPI Standard/Engine selection **** //
/* By default the SPI Engine Framework is used for communicating with eval board.
 * Uncomment the "USE_STANDARD_SPI" macro to enable the standard SPI.
 * framework.
 * */
//#define USE_STANDARD_SPI

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>

#if !defined(USE_STANDARD_SPI)
#include "spi_engine.h"
#include "clk_axi_clkgen.h"
#include "no_os_pwm.h"
#else
#include "no_os_spi.h"
#endif

#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* AD469x registers */
#define AD469x_REG_IF_CONFIG_A		0x000
#define AD469x_REG_IF_CONFIG_B		0x001
#define AD469x_REG_DEVICE_TYPE		0x003
#define AD469x_REG_DEVICE_ID_L		0x004
#define AD469x_REG_DEVICE_ID_H		0x005
#define AD469x_REG_SCRATCH_PAD		0x00A
#define AD469x_REG_VENDOR_L		0x00C
#define AD469x_REG_VENDOR_H		0x00D
#define AD469x_REG_LOOP_MODE		0x00E
#define AD469x_REG_IF_CONFIG_C		0x010
#define AD469x_REG_IF_STATUS		0x011
#define AD469x_REG_STATUS		0x014
#define AD469x_REG_ALERT_STATUS1	0x015
#define AD469x_REG_ALERT_STATUS2	0x016
#define AD469x_REG_ALERT_STATUS3	0x017
#define AD469x_REG_ALERT_STATUS4	0x018
#define AD469x_REG_CLAMP_STATUS1	0x01A
#define AD469x_REG_CLAMP_STATUS2	0x01B
#define AD469x_REG_SETUP		0x020
#define AD469x_REG_REF_CTRL		0x021
#define AD469x_REG_SEQ_CTRL		0x022
#define AD469x_REG_AC_CTRL		0x023
#define AD469x_REG_STD_SEQ_CONFIG	0x024
#define AD469x_REG_GPIO_CTRL		0x026
#define AD469x_REG_GP_MODE		0x027
#define AD469x_REG_GPIO_STATE		0x028
#define AD469x_REG_TEMP_CTRL		0x029
#define AD469x_REG_CONFIG_IN(x)		((x & 0x0F) | 0x30)
#define AD469x_REG_THRESHOLD_UB(x)  ((x << 1) | 0x40)
#define AD469x_REG_THRESHOLD_LB(x)  ((x << 1) | 0x60)
#define AD469x_REG_HYST_IN(x)		((x << 1) | 0x80)
#define AD469x_REG_OFFSET_IN(x)		((x << 1) | 0xA0)
#define AD469x_REG_GAIN_IN(x)       ((x << 1) | 0x0C0)
#define AD469x_REG_AS_SLOT(x)		((x & 0x7F) | 0x100)

/* 5-bit SDI Conversion Mode Commands */
#define AD469x_CMD_REG_CONFIG_MODE		(0x0A << 3)
#define AD469x_CMD_SEL_TEMP_SNSOR_CH		(0x0F << 3)
#define AD469x_CMD_CONFIG_CH_SEL(x)		((0x10 | (0x0F & x)) << 3)

/* Status Register Mask */
#define AD469x_REG_STATUS_RESET_MASK     (0x01 << 5)

/* AD469x_REG_SETUP */
#define AD469x_SETUP_IF_MODE_MASK		(0x01 << 2)
#define AD469x_SETUP_IF_MODE_CONV		(0x01 << 2)
#define AD469x_SETUP_CYC_CTRL_MASK		(0x01 << 1)
#define AD469x_SETUP_CYC_CTRL_SINGLE(x)		((x & 0x01) << 1)

/* AD469x_REG_REF_CTRL */
#define AD469x_REG_REF_VREF_SET_MASK		(0x07 << 2)
#define AD469x_REG_REF_VREF_SET(x)          ((x & 0x07) << 2)
#define AD469x_REG_REF_VREF_REFHIZ_MASK		(0x07 << 1)
#define AD469x_REG_REF_VREF_REFHIZ(x)		((x & 0x01) << 1)
#define AD469x_REG_REF_VREF_REFBUF_MASK		0x01
#define AD469x_REG_REF_VREF_REFBUF(x)		(x & 0x01)

/* AD469x_REG_GP_MODE */
#define AD469x_GP_MODE_BUSY_GP_EN_MASK		(0x01 << 1)
#define AD469x_GP_MODE_BUSY_GP_EN(x)		((x & 0x01) << 1)
#define AD469x_GP_MODE_BUSY_GP_SEL_MASK		(0x01 << 4)
#define AD469x_GP_MODE_BUSY_GP_SEL(x)		((x & 0x01) << 4)

/* AD469x_REG_SEQ_CTRL */
#define AD469x_SEQ_CTRL_STD_SEQ_EN_MASK		(0x01 << 7)
#define AD469x_SEQ_CTRL_STD_SEQ_EN(x)		((x & 0x01) << 7)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK	(0x7f << 0)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS(x)		((x & 0x7f) << 0)

/* AD469x_REG_TEMP_CTRL */
#define AD469x_REG_TEMP_CTRL_TEMP_EN_MASK	(0x01 << 0)
#define AD469x_REG_TEMP_CTRL_TEMP_EN(x)		((x & 0x01) << 0)

/* AD469x_REG_AS_SLOT */
#define AD469x_REG_AS_SLOT_INX(x)		((x & 0x0f) << 0)

/* AD469x_REG_IF_CONFIG_C */
#define AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK	(0x01 << 5)
#define AD469x_REG_IF_CONFIG_C_MB_STRICT(x)	((x & 0x01) << 5)

/* AD469x_REG_CONFIG_INn */
#define AD469x_REG_CONFIG_IN_OSR_MASK		(0x03 << 0)
#define AD469x_REG_CONFIG_IN_OSR(x)		((x & 0x03) << 0)
#define AD469x_REG_CONFIG_IN_HIZ_EN_MASK	(0x01 << 3)
#define AD469x_REG_CONFIG_IN_HIZ_EN(x)		((x & 0x01) << 3)
#define AD469x_REG_CONFIG_IN_PAIR_MASK		(0x03 << 4)
#define AD469x_REG_CONFIG_IN_PAIR(x)		((x & 0x03) << 4)
#define AD469x_REG_CONFIG_IN_MODE_MASK		(0x01 << 6)
#define AD469x_REG_CONFIG_IN_MODE(x)		((x & 0x01) << 6)
#define AD469x_REG_CONFIG_IN_TD_EN_MASK		(0x01 << 7)
#define AD469x_REG_CONFIG_IN_TD_EN(x)		((x & 0x01) << 7)

#define AD469x_CHANNEL(x)			(NO_OS_BIT(x) & 0xFFFF)
#define AD469x_CHANNEL_NO			16
#define AD469x_SLOTS_NO				0x80

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ad469x_channel_sequencing` enumeration defines the different
 * modes of channel sequencing available for the AD469x device. It
 * includes options for single and two cycle reads, as well as standard
 * and advanced sequencing modes, allowing for flexible configuration of
 * how channels are read and processed.
 *
 * @param AD469x_single_cycle Represents a single cycle read mode.
 * @param AD469x_two_cycle Represents a two cycle read mode.
 * @param AD469x_standard_seq Represents a standard mode for sequencing through
 * channels.
 * @param AD469x_advanced_seq Represents an advanced mode for sequencing through
 * channels.
 ******************************************************************************/
enum ad469x_channel_sequencing {
	/** Single cycle read */
	AD469x_single_cycle,
	/** Two cycle read */
	AD469x_two_cycle,
	/** Sequence trough channels, standard mode */
	AD469x_standard_seq,
	/** Sequence trough channels, advanced mode */
	AD469x_advanced_seq,
};

/***************************************************************************//**
 * @brief The `ad469x_busy_gp_sel` enumeration defines the possible general
 * purpose pin selections for indicating the busy state of the AD469x
 * device. It provides two options, `AD469x_busy_gp0` and
 * `AD469x_busy_gp3`, which correspond to the busy signal being output on
 * either general purpose pin 0 or pin 3, respectively. This allows for
 * flexible configuration of the busy signal output based on the specific
 * hardware setup or application requirements.
 *
 * @param AD469x_busy_gp0 Represents the busy state on general purpose pin 0.
 * @param AD469x_busy_gp3 Represents the busy state on general purpose pin 3.
 ******************************************************************************/
enum ad469x_busy_gp_sel {
	/** Busy on gp0 */
	AD469x_busy_gp0 = 0,
	/** Busy on gp3 */
	AD469x_busy_gp3 = 1,
};

/***************************************************************************//**
 * @brief The `ad469x_reg_access` enumeration defines the modes of register
 * access for the AD469x device, allowing for either byte or word access.
 * This is crucial for configuring how data is read from or written to
 * the device's registers, impacting the granularity and efficiency of
 * data transactions.
 *
 * @param AD469x_BYTE_ACCESS Represents byte-level access to the registers.
 * @param AD469x_WORD_ACCESS Represents word-level access to the registers.
 ******************************************************************************/
enum ad469x_reg_access {
	AD469x_BYTE_ACCESS,
	AD469x_WORD_ACCESS,
};

/***************************************************************************//**
 * @brief The `ad469x_supported_dev_ids` enumeration defines a set of constants
 * representing the supported device IDs for the AD469x series of
 * devices. Each enumerator corresponds to a specific model within the
 * series, allowing the software to identify and handle different models
 * appropriately. This enumeration is crucial for ensuring compatibility
 * and correct operation of the driver with the specific AD469x device in
 * use.
 *
 * @param ID_AD4695 Represents the device ID for the AD4695 model.
 * @param ID_AD4696 Represents the device ID for the AD4696 model.
 * @param ID_AD4697 Represents the device ID for the AD4697 model.
 * @param ID_AD4698 Represents the device ID for the AD4698 model.
 ******************************************************************************/
enum ad469x_supported_dev_ids {
	ID_AD4695,
	ID_AD4696,
	ID_AD4697,
	ID_AD4698,
};

/***************************************************************************//**
 * @brief The `ad469x_osr_ratios` enumeration defines the supported oversampling
 * ratios for the AD469x series of devices. Oversampling is a technique
 * used to improve the resolution and signal-to-noise ratio of the
 * analog-to-digital conversion process by sampling the input signal at a
 * higher rate than the Nyquist rate and then averaging the results. This
 * enumeration provides four distinct oversampling ratios: 1, 4, 16, and
 * 64, allowing users to select the appropriate level of oversampling
 * based on their specific application requirements.
 *
 * @param AD469x_OSR_1 Represents an oversampling ratio of 1.
 * @param AD469x_OSR_4 Represents an oversampling ratio of 4.
 * @param AD469x_OSR_16 Represents an oversampling ratio of 16.
 * @param AD469x_OSR_64 Represents an oversampling ratio of 64.
 ******************************************************************************/
enum ad469x_osr_ratios {
	AD469x_OSR_1,
	AD469x_OSR_4,
	AD469x_OSR_16,
	AD469x_OSR_64
};

/***************************************************************************//**
 * @brief The `ad469x_pin_pairing` enumeration defines the possible
 * configurations for pairing input pins in the AD469x device. This
 * allows for different reference configurations, such as referencing to
 * ground, using a common reference, or pairing even and odd channels
 * together. These configurations are used to set up the input channels
 * for the device's standard sequencer mode, providing flexibility in how
 * the inputs are processed and measured.
 *
 * @param AD469x_INx_REF_GND Represents a pin pairing option where the input is
 * referenced to ground.
 * @param AD469x_INx_COM Represents a pin pairing option where the input is
 * common.
 * @param AD469x_INx_EVEN_ODD Represents a pin pairing option where even and odd
 * channels are paired.
 ******************************************************************************/
enum ad469x_pin_pairing {
	AD469x_INx_REF_GND,
	AD469x_INx_COM,
	AD469x_INx_EVEN_ODD
};

/***************************************************************************//**
 * @brief The `ad469x_ref_set` enumeration defines a set of constants
 * representing different reference input voltage ranges for the AD469x
 * device. Each enumerator corresponds to a specific voltage range,
 * allowing the device to be configured for various reference input
 * levels. This is crucial for setting the appropriate reference voltage
 * range for the device's operation, ensuring accurate analog-to-digital
 * conversion.
 *
 * @param AD469x_2P4_2P75 Represents a reference input range from 2.4V to 2.75V.
 * @param AD469x_2P75_3P25 Represents a reference input range from 2.75V to
 * 3.25V.
 * @param AD469x_3P25_3P75 Represents a reference input range from 3.25V to
 * 3.75V.
 * @param AD469x_3P75_4P5 Represents a reference input range from 3.75V to 4.5V.
 * @param AD469x_4P5_5P1 Represents a reference input range from 4.5V to 5.1V.
 ******************************************************************************/
enum ad469x_ref_set {
	AD469x_2P4_2P75,
	AD469x_2P75_3P25,
	AD469x_3P25_3P75,
	AD469x_3P75_4P5,
	AD469x_4P5_5P1,
};

/***************************************************************************//**
 * @brief The `ad469x_ain_high_z` enumeration defines the possible states for
 * the analog input high impedance mode in the AD469x device. This mode
 * can be either enabled or disabled, allowing for control over the input
 * impedance characteristics of the device's analog inputs. This setting
 * is crucial for applications where input impedance needs to be managed
 * to match specific circuit requirements or to minimize loading effects.
 *
 * @param AD469x_AIN_HIGH_Z_DISABLE Represents the state where the analog input
 * high impedance mode is disabled.
 * @param AD469x_AIN_HIGH_Z_ENABLE Represents the state where the analog input
 * high impedance mode is enabled.
 ******************************************************************************/
enum ad469x_ain_high_z {
	AD469x_AIN_HIGH_Z_DISABLE,
	AD469x_AIN_HIGH_Z_ENABLE,
};

/***************************************************************************//**
 * @brief The `ad469x_init_param` structure is used to define the initialization
 * parameters for the AD469x device, which includes configurations for
 * SPI communication, GPIO settings, clock generation, and device-
 * specific settings such as channel sequencing and oversampling ratios.
 * It provides a comprehensive setup for initializing the device with
 * options for standard and advanced sequencer modes, as well as
 * additional features like temperature measurement and extended
 * initialization capabilities.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param offload_init_param Pointer to SPI module offload initialization
 * parameters.
 * @param trigger_pwm_init Pointer to PWM generator initialization parameters.
 * @param clkgen_init Pointer to clock generator initialization parameters for
 * HDL design.
 * @param axi_clkgen_rate Clock generator rate in Hz.
 * @param gpio_resetn Pointer to RESET GPIO initialization parameters.
 * @param gpio_convst Pointer to CONVST GPIO initialization parameters.
 * @param gpio_busy Pointer to BUSY GPIO initialization parameters.
 * @param reg_access_speed Register access speed in Hz.
 * @param reg_data_width Width of the register data in bits.
 * @param capture_data_width Width of the capture data in bits.
 * @param dev_id Device ID from supported device IDs.
 * @param std_seq_pin_pairing Pin pairing option in standard sequencer mode.
 * @param ch_sequence Channel sequencing mode.
 * @param std_seq_osr OSR resolution for all channels in standard sequencer
 * mode.
 * @param adv_seq_osr_resol Array of OSR resolutions for each channel in
 * advanced sequencer mode.
 * @param dcache_invalidate_range Function pointer to invalidate data cache for
 * a given address range.
 * @param num_data_ch Number of data channels to enable.
 * @param temp_enabled Boolean indicating if temperature is enabled for
 * sequencers.
 * @param enable_extended_init Boolean indicating if extended initialization is
 * enabled.
 ******************************************************************************/
struct ad469x_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_init;
#if !defined(USE_STANDARD_SPI)
	/* SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	/* PWM generator init structure */
	struct no_os_pwm_init_param	*trigger_pwm_init;
	/* Clock gen for hdl design init structure */
	struct axi_clkgen_init	*clkgen_init;
	/* Clock generator rate */
	uint32_t		axi_clkgen_rate;
#endif
	/** RESET GPIO initialization structure. */
	struct no_os_gpio_init_param	*gpio_resetn;
	/** CONVST GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_convst;
	/** BUSY GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_busy;
	/* Register access speed */
	uint32_t		reg_access_speed;
	/* Register data width */
	uint8_t		reg_data_width;
	/* Capture data width */
	uint8_t		capture_data_width;
	/* Device Settings */
	enum ad469x_supported_dev_ids dev_id;
	/* Pin Pairing option in standard sequencer mode */
	enum ad469x_pin_pairing std_seq_pin_pairing;
	/* Channel sequencing mode */
	enum ad469x_channel_sequencing ch_sequence;
	/** OSR resolution corresponding to all channels, when standard
	 * sequencer is selected. */
	enum ad469x_osr_ratios std_seq_osr;
	/** OSR resolution corresponding to each channel, when advanced
	 * sequencer is selected. */
	enum ad469x_osr_ratios adv_seq_osr_resol[AD469x_CHANNEL_NO];
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	/** Number of data channels to enable */
	uint8_t num_data_ch;
	/** Temperature enabled for standard and advanced sequencer if set. */
	bool temp_enabled;
	/** enable extended init */
	bool enable_extended_init;
};

/***************************************************************************//**
 * @brief The `ad469x_dev` structure represents a device configuration for the
 * AD469x series, encapsulating various parameters and descriptors
 * necessary for device operation. It includes SPI communication
 * descriptors, GPIO handlers for control signals, and settings for
 * channel sequencing and oversampling. The structure supports both
 * standard and advanced sequencer modes, with options for temperature
 * measurement and data cache management. It is designed to interface
 * with the AD469x series of analog-to-digital converters, providing a
 * comprehensive setup for device initialization and operation.
 *
 * @param spi_desc Pointer to the SPI descriptor used for communication.
 * @param clkgen Pointer to the clock generator structure for HDL design (used
 * if not using standard SPI).
 * @param trigger_pwm_desc Pointer to the PWM generator descriptor for trigger
 * conversion (used if not using standard SPI).
 * @param offload_init_param Pointer to the SPI engine offload initialization
 * parameters (used if not using standard SPI).
 * @param reg_access_speed Specifies the speed of register access in Hz.
 * @param reg_data_width Width of the register data in bits.
 * @param capture_data_width Width of the capture data in bits.
 * @param dev_id Identifier for the supported device type.
 * @param gpio_resetn Pointer to the GPIO descriptor for the RESET pin.
 * @param gpio_convst Pointer to the GPIO descriptor for the CONVST pin.
 * @param gpio_busy Pointer to the GPIO descriptor for the BUSY pin.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a given address range.
 * @param ch_sequence Current channel sequencing mode.
 * @param std_seq_pin_pairing Pin pairing option in standard sequencer mode.
 * @param std_seq_osr OSR resolution for all channels in standard sequencer
 * mode.
 * @param adv_seq_osr_resol Array of OSR resolutions for each channel in
 * advanced sequencer mode.
 * @param ch_slots Array representing channel slots for the advanced sequencer.
 * @param temp_enabled Boolean indicating if temperature measurement is enabled.
 * @param num_slots Number of active channel slots in advanced sequencer mode.
 * @param num_data_ch Number of data channels to enable.
 ******************************************************************************/
struct ad469x_dev {
	/* SPI descriptor */
	struct no_os_spi_desc		*spi_desc;
#if !defined(USE_STANDARD_SPI)
	/* Clock gen for hdl design structure */
	struct axi_clkgen	*clkgen;
	/* Trigger conversion PWM generator descriptor */
	struct no_os_pwm_desc		*trigger_pwm_desc;
	/* SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
#endif
	/* Register access speed */
	uint32_t		reg_access_speed;
	/* Register data width */
	uint8_t		reg_data_width;
	/* Capture data width */
	uint8_t		capture_data_width;
	/* Device Settings */
	enum ad469x_supported_dev_ids dev_id;
	/** RESET GPIO handler. */
	struct no_os_gpio_desc	*gpio_resetn;
	/** CONVST GPIO descriptor */
	struct no_os_gpio_desc *gpio_convst;
	/** BUSY GPIO descriptor */
	struct no_os_gpio_desc *gpio_busy;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	/** Current channel sequence */
	enum ad469x_channel_sequencing ch_sequence;
	/* Pin Pairing option in standard sequencer mode */
	enum ad469x_pin_pairing std_seq_pin_pairing;
	/** OSR resolution corresponding to all channels, when standard
	 * sequencer is selected. */
	enum ad469x_osr_ratios std_seq_osr;
	/** OSR resolution corresponding to each channel, when advanced
	 * sequencer is selected. */
	enum ad469x_osr_ratios adv_seq_osr_resol[AD469x_CHANNEL_NO];
	/** Channel slots for advanced sequencer */
	uint8_t ch_slots[AD469x_SLOTS_NO];
	/** Temperature enabled for standard and advanced sequencer if set. */
	bool temp_enabled;
	/** Number of active channel slots, for advanced sequencer */
	uint8_t num_slots;
	/** Number of data channels to enable */
	uint8_t num_data_ch;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Read device register. */
/***************************************************************************//**
 * @brief This function reads a register from the AD469x device using SPI
 * communication. It is typically used to retrieve configuration or
 * status information from the device. The function requires a valid
 * device structure, a register address, and a pointer to store the read
 * data. It must be called with a properly initialized device structure.
 * The function handles both standard and non-standard SPI
 * configurations, adjusting the transfer width and speed as necessary.
 * It returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. It is a 16-bit value
 * representing the register's location.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t error code: 0 on success, or a negative error code
 * on failure.
 ******************************************************************************/
int32_t ad469x_spi_reg_read(struct ad469x_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data);

/* Write device register */
/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register of an AD469x device using SPI communication. It requires a
 * valid device structure that has been properly initialized. The
 * function is typically called when there is a need to configure or
 * modify the settings of the AD469x device by writing to its registers.
 * It is important to ensure that the device is ready for communication
 * and that the register address and data are correctly specified. The
 * function returns an error code if the write operation fails, allowing
 * the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null and should be initialized before calling this
 * function.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. The address should be within the valid range of the
 * device's register map.
 * @param reg_data The 8-bit data to be written to the specified register. This
 * is the value that will be stored in the register.
 * @return Returns an int32_t error code, where 0 indicates success and a non-
 * zero value indicates an error occurred during the SPI write
 * operation.
 ******************************************************************************/
int32_t ad469x_spi_reg_write(struct ad469x_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data);

/* Read from device using a mask */
/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD469x
 * device, applies a mask to the read value, and stores the result in the
 * provided data pointer. It is useful for retrieving specific bits from
 * a register without affecting other bits. The function must be called
 * with a valid device structure and a non-null data pointer. It returns
 * an error code if the read operation fails.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD469x device.
 * @param mask A bitmask to apply to the read register value. Determines which
 * bits are extracted.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t ad469x_spi_read_mask(struct ad469x_dev *dev,
			     uint16_t reg_addr,
			     uint8_t mask,
			     uint8_t *data);

/* Write to device using a mask */
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * AD469x device by applying a mask and writing new data. It is useful
 * when only certain bits of a register need to be updated without
 * affecting the other bits. The function first reads the current value
 * of the register, applies the mask to clear the bits to be modified,
 * and then writes the new data. It should be called when the device is
 * properly initialized and ready for SPI communication. The function
 * returns an error code if the read or write operation fails.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD469x device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the masked bits of the register.
 * Only the bits corresponding to the mask will be updated.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int32_t ad469x_spi_write_mask(struct ad469x_dev *dev,
			      uint16_t reg_addr,
			      uint8_t mask,
			      uint8_t data);

/* Read data from device */
/***************************************************************************//**
 * @brief This function retrieves a specified number of samples from a given
 * channel of the AD469x device and stores them in the provided buffer.
 * It is essential to ensure that the device is properly initialized
 * before calling this function. The function supports both standard SPI
 * and SPI engine frameworks, and the appropriate framework should be
 * selected based on the build configuration. The channel parameter must
 * be within the valid range of data channels or set to access the
 * temperature sensor channel. The buffer must be large enough to hold
 * the requested number of samples, and the samples parameter indicates
 * how many samples to read. The function returns an error code if the
 * operation fails, which should be checked by the caller.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to read from. Must be less than or equal to
 * the number of data channels in the device, or equal to the
 * number of data channels to select the temperature sensor
 * channel.
 * @param buf A pointer to a buffer where the read data will be stored. Must not
 * be null and should be large enough to hold 'samples' number of
 * samples.
 * @param samples The number of samples to read from the device. Must be a
 * positive integer.
 * @return Returns an int32_t error code: 0 on success, or a negative value on
 * failure.
 ******************************************************************************/
int32_t ad469x_read_data(struct ad469x_dev *dev,
			 uint8_t channel,
			 uint32_t *buf,
			 uint16_t samples);

/* Read from device when converter has the channel sequencer activated */
/***************************************************************************//**
 * @brief This function is used to read a specified number of data samples from
 * an AD469x device into a provided buffer. It is particularly useful
 * when the device is configured with a channel sequencer. The function
 * must be called with a properly initialized device structure and a
 * buffer large enough to hold the requested number of samples. The
 * number of samples read is determined by the number of slots configured
 * in the device and whether temperature reading is enabled. If the
 * device is in advanced sequencing mode, additional processing is
 * applied to each sample. The function returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param buf A pointer to a buffer where the read samples will be stored. The
 * buffer must be large enough to hold 'samples' multiplied by the
 * number of slots plus one if temperature is enabled. Must not be
 * null.
 * @param samples The number of samples to read per slot. Must be a positive
 * integer.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad469x_seq_read_data(struct ad469x_dev *dev,
			     uint32_t *buf,
			     uint32_t samples);

/* Set channel sequence */
/***************************************************************************//**
 * @brief This function sets the channel sequencing mode of an AD469x device to
 * one of the predefined modes: single cycle, two cycle, standard
 * sequence, or advanced sequence. It should be called when you need to
 * change the way channels are sequenced during data acquisition. The
 * function requires a valid device structure and a sequencing mode. It
 * updates the device's internal state to reflect the new sequencing
 * mode. If an invalid sequencing mode is provided, the function returns
 * an error. This function must be called after the device has been
 * properly initialized.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param seq An enum value of type ad469x_channel_sequencing indicating the
 * desired channel sequencing mode. Valid values are
 * AD469x_single_cycle, AD469x_two_cycle, AD469x_standard_seq, and
 * AD469x_advanced_seq. Invalid values result in an error return.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad469x_set_channel_sequence(struct ad469x_dev *dev,
				    enum ad469x_channel_sequencing seq);

/* Configure standard sequencer enabled channels */
/***************************************************************************//**
 * @brief This function sets up the standard sequencer on the AD469x device to
 * include the channels specified by the channel mask. It should be
 * called when configuring the device for standard sequence operations.
 * The function writes the channel mask to the appropriate registers,
 * enabling the specified channels for sequencing. The number of active
 * slots is updated based on the number of channels enabled. This
 * function must be called with a valid device structure and a channel
 * mask that specifies the channels to be included in the sequence.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param ch_mask A 16-bit mask where each bit represents a channel. A bit set
 * to 1 includes the corresponding channel in the sequence. The
 * mask should be within the valid range for the device, and
 * invalid bits are ignored.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad469x_std_sequence_ch(struct ad469x_dev *dev,
			       uint16_t ch_mask);

/* Configure advanced sequencer number of slots */
/***************************************************************************//**
 * @brief This function sets the number of slots in the advanced sequencer mode
 * for an AD469x device. It should be used when configuring the device to
 * operate in advanced sequencing mode, allowing the user to specify how
 * many slots are active. The function must be called with a valid device
 * structure that has been properly initialized. The number of slots is
 * adjusted internally by subtracting one from the provided value before
 * being written to the device. If the operation is successful, the
 * device's internal state is updated to reflect the new number of slots.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param num_slots The desired number of slots for the advanced sequencer.
 * Valid values are from 1 to 128. A value of 0 will result in
 * no slots being configured.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad469x_adv_sequence_set_num_slots(struct ad469x_dev *dev,
		uint8_t num_slots);

/* Advanced sequencer, assign channel to a slot */
/***************************************************************************//**
 * @brief This function is used to configure the advanced sequencer of an AD469x
 * device by assigning a specific channel to a designated slot. It should
 * be called when setting up the advanced sequencing mode, allowing
 * precise control over which channels are sampled in each slot. The
 * function requires a valid device structure and expects the slot and
 * channel numbers to be within their respective valid ranges. It returns
 * an error code if the operation fails, ensuring that the caller can
 * handle any issues that arise during the configuration process.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param slot An 8-bit unsigned integer representing the slot number to
 * configure. Must be within the valid range of slots supported by
 * the device.
 * @param channel An 8-bit unsigned integer representing the channel number to
 * assign to the slot. Must be within the valid range of channels
 * supported by the device.
 * @return Returns an int32_t value, where 0 indicates success and a non-zero
 * value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t ad469x_adv_sequence_set_slot(struct ad469x_dev *dev,
				     uint8_t slot,
				     uint8_t channel);

/* Enable temperature read at the end of the sequence, for standard and */
/***************************************************************************//**
 * @brief This function is used to enable the temperature reading feature in the
 * AD469x device's sequence. It should be called when the user wants to
 * include temperature data in the sequence of readings from the device.
 * The function modifies the device's internal state to enable
 * temperature measurement, and it must be called with a valid device
 * structure that has been properly initialized. If the operation is
 * successful, the device's temperature reading capability will be
 * activated, and the function will return a success code. If an error
 * occurs, the function will return an error code indicating the failure.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad469x_sequence_enable_temp(struct ad469x_dev *dev);

/* Disable temperature read at the end of the sequence, for standard and */
/***************************************************************************//**
 * @brief Use this function to disable the temperature reading at the end of a
 * sequence for the AD469x device. This function should be called when
 * temperature data is no longer needed in the sequence, or to optimize
 * performance by excluding unnecessary data. Ensure that the device is
 * properly initialized before calling this function. The function
 * modifies the device's internal state to reflect the change in
 * temperature reading configuration.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if communication with the
 * device fails.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad469x_sequence_disable_temp(struct ad469x_dev *dev);

/* Configure over sampling ratio in advanced sequencer mode */
/***************************************************************************//**
 * @brief This function sets the oversampling ratio for a specified channel when
 * the device is operating in advanced sequencer mode. It should be used
 * only when the channel sequencing mode is set to advanced, as it
 * returns an error if the device is in single or two-cycle modes. The
 * function also updates the internal resolution settings for the channel
 * and adjusts the capture data width to accommodate the maximum data
 * width. It is important to ensure that the channel index is within the
 * valid range of enabled data channels to avoid errors.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null.
 * @param ch The channel index for which the oversampling ratio is to be set.
 * Must be less than the number of enabled data channels; otherwise,
 * the function returns an error.
 * @param ratio The desired oversampling ratio, specified as an enum
 * ad469x_osr_ratios value. Determines the oversampling setting for
 * the specified channel.
 * @return Returns 0 on success, or a negative error code if the device is not
 * in advanced sequencer mode or if the channel index is invalid.
 ******************************************************************************/
int32_t ad469x_adv_seq_osr(struct ad469x_dev *dev, uint16_t ch,
			   enum ad469x_osr_ratios ratio);

/* Configure over sampling ratio in standard sequencer mode */
/***************************************************************************//**
 * @brief This function sets the oversampling ratio for all channels when the
 * device is operating in standard sequencer mode. It should be called
 * only when the channel sequence is not set to single or two cycle
 * modes. The function updates the device configuration to reflect the
 * specified oversampling ratio, which can affect the resolution and data
 * capture width. It is important to ensure that the device is properly
 * initialized and in a compatible sequencing mode before calling this
 * function.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null, and the device should be initialized and configured
 * for standard sequencer mode.
 * @param ratio An enum value of type ad469x_osr_ratios representing the desired
 * oversampling ratio. Valid values are AD469x_OSR_1, AD469x_OSR_4,
 * AD469x_OSR_16, and AD469x_OSR_64.
 * @return Returns 0 on success, or a negative error code if the device is in an
 * incompatible mode or if there is a communication failure.
 ******************************************************************************/
int32_t ad469x_std_seq_osr(struct ad469x_dev *dev,
			   enum ad469x_osr_ratios ratio);

/* Configure the pairing option in standard sequencer mode */
/***************************************************************************//**
 * @brief This function sets the pin pairing configuration for the AD469x device
 * when operating in standard sequencer mode. It should be called after
 * the device has been initialized and before starting any data
 * acquisition that relies on specific pin pairing settings. The function
 * updates the device's internal configuration to reflect the selected
 * pin pairing option, which can affect how input channels are paired and
 * read. Proper pin pairing is essential for accurate data acquisition in
 * certain applications.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param pin_pair An enum value of type ad469x_pin_pairing specifying the
 * desired pin pairing configuration. Valid values are defined
 * in the ad469x_pin_pairing enumeration.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad469x_std_pin_pairing(struct ad469x_dev *dev,
			       enum ad469x_pin_pairing pin_pair);

/* Configure the busy indicator to the output on specified pin */
/***************************************************************************//**
 * @brief This function is used to configure the AD469x device to output a busy
 * indicator signal on a specified general-purpose pin. It should be
 * called when there is a need to monitor the busy state of the device
 * through an external pin. The function requires a valid device
 * structure and a selection of the general-purpose pin where the busy
 * signal will be output. It is important to ensure that the device is
 * properly initialized before calling this function. The function will
 * return an error code if the configuration fails.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param gp_sel An enum value of type ad469x_busy_gp_sel indicating the
 * general-purpose pin selection for the busy signal. Valid values
 * are AD469x_busy_gp0 and AD469x_busy_gp3.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad469x_set_busy(struct ad469x_dev *dev,
			enum ad469x_busy_gp_sel gp_sel);

/* Enter conversion mode */
/***************************************************************************//**
 * @brief This function is used to set the AD469x device into conversion mode,
 * which is necessary for performing analog-to-digital conversions. It
 * should be called after the device has been properly initialized and
 * configured. The function modifies the device's setup register to
 * enable conversion mode, ensuring that the device is ready to perform
 * conversions. It is important to ensure that the device is not in a
 * reset state and that all necessary configurations are completed before
 * calling this function.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. This
 * pointer must not be null, and the device must be initialized
 * before calling this function. The caller retains ownership of the
 * memory.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad469x_enter_conversion_mode(struct ad469x_dev *dev);

/* Exit conversion mode */
/***************************************************************************//**
 * @brief Use this function to transition the AD469x device from conversion mode
 * to register configuration mode. This is typically necessary when you
 * need to reconfigure the device settings after data acquisition. Ensure
 * that the device is properly initialized before calling this function.
 * The function handles both standard SPI and SPI engine frameworks,
 * depending on the configuration. It returns an error code if the
 * operation fails, which should be checked to ensure successful mode
 * transition.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the mode transition.
 ******************************************************************************/
int32_t ad469x_exit_conversion_mode(struct ad469x_dev *dev);

/* Reset with AD469x device */
/***************************************************************************//**
 * @brief This function performs a hardware reset on the AD469x device by
 * toggling the reset GPIO pin. It should be used when the device needs
 * to be reset to its default state, such as during initialization or
 * error recovery. The function requires a valid device structure and
 * assumes that the GPIO for reset has been properly configured. It
 * returns an error code if the reset operation fails at any step,
 * ensuring that the caller can handle such failures appropriately.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. This
 * must be a valid, initialized device structure with a configured
 * reset GPIO. The function will return an error if this parameter is
 * null or if the GPIO operations fail.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int32_t ad469x_reset_dev(struct ad469x_dev *dev);

/* Configures the AD469x device */
/***************************************************************************//**
 * @brief This function sets up the AD469x device according to the provided
 * configuration parameters. It must be called after the device has been
 * initialized and before any data acquisition operations. The function
 * configures various aspects of the device, such as register access
 * mode, busy pin selection, oversampling ratio, pin pairing, and channel
 * sequencing. It also optionally enables temperature measurement if
 * specified. The function returns an error code if any configuration
 * step fails, allowing the caller to handle configuration errors
 * appropriately.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device to be configured. Must not be null.
 * @param config_desc A pointer to an ad469x_init_param structure containing the
 * desired configuration settings. Must not be null and
 * should be properly initialized with valid configuration
 * parameters.
 * @return Returns 0 on success or a negative error code if any configuration
 * step fails.
 ******************************************************************************/
int32_t ad469x_config(struct ad469x_dev *dev,
		      struct ad469x_init_param *config_desc);

/* Get Reference */
/***************************************************************************//**
 * @brief Use this function to obtain the current reference voltage setting of
 * an AD469x device. It is essential to ensure that the `device`
 * parameter is a valid, initialized pointer to an `ad469x_dev` structure
 * before calling this function. The function will populate the `ref_set`
 * parameter with the current reference setting, which is an enumerated
 * value of type `ad469x_ref_set`. This function should be called when
 * you need to verify or log the current reference configuration of the
 * device. If the `device` parameter is null, the function will return an
 * error code.
 *
 * @param device A pointer to an `ad469x_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ref_set A pointer to an `ad469x_ref_set` enum where the current
 * reference setting will be stored. Must not be null. The caller
 * retains ownership.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if there is a failure in reading the reference setting.
 ******************************************************************************/
int32_t ad469x_get_reference(struct ad469x_dev *device,
			     enum ad469x_ref_set *ref_set);

/* Set reference */
/***************************************************************************//**
 * @brief This function configures the reference voltage setting for an AD469x
 * device. It should be called when you need to change the reference
 * voltage range of the device, which is specified by the `ref_set`
 * parameter. Ensure that the `device` parameter points to a valid and
 * initialized `ad469x_dev` structure before calling this function. The
 * function returns an error code if the operation fails, which can be
 * used for error handling.
 *
 * @param device A pointer to an `ad469x_dev` structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param ref_set An enumeration value of type `ad469x_ref_set` specifying the
 * desired reference voltage range. The value must be one of the
 * predefined options in the `ad469x_ref_set` enum.
 * @return Returns an `int32_t` error code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int32_t ad469x_set_reference(struct ad469x_dev *device,
			     enum ad469x_ref_set ref_set);

/* Configure analog input high Z mode */
/***************************************************************************//**
 * @brief This function is used to set the high impedance mode for a specific
 * analog input channel on an AD469x device. It is typically called when
 * configuring the device's input channels to ensure they are set to the
 * desired impedance state. The function requires a valid device
 * structure and a channel number, which must be within the range of
 * available channels. The high impedance mode can be enabled or disabled
 * using the appropriate enumeration value. The function returns an error
 * code if the operation fails, which can be used for error handling.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param ch The channel number to configure. Must be within the valid range of
 * channels supported by the device.
 * @param status An enumeration value of type ad469x_ain_high_z indicating
 * whether to enable or disable high impedance mode for the
 * specified channel.
 * @return Returns an int32_t error code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad469x_configure_ain_high_z(struct ad469x_dev *dev,
				    uint8_t ch,
				    enum ad469x_ain_high_z status);

/* Get the status of analog input high Z mode */
/***************************************************************************//**
 * @brief This function is used to check whether a specific analog input channel
 * on the AD469x device is configured in high impedance mode. It is
 * typically called when there is a need to verify the configuration of
 * the input channels, especially in applications where input impedance
 * is critical. The function requires a valid device structure and a
 * channel number, and it outputs the status through a provided pointer.
 * It must be called with a properly initialized device structure, and
 * the channel number should be within the valid range for the device.
 * The function will return an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad469x_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number to query. Must be within the valid range of
 * channels supported by the device.
 * @param status A pointer to an ad469x_ain_high_z enum where the high impedance
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad469x_get_ain_high_z_status(struct ad469x_dev *dev,
				     uint8_t ch,
				     enum ad469x_ain_high_z *status);

/* Get the number of channels that are enabled */
/***************************************************************************//**
 * @brief This function is used to determine the number of data channels
 * currently enabled on the specified AD469x device. It should be called
 * when you need to know how many channels are active, which can include
 * an additional channel if temperature sensing is enabled. The function
 * requires a valid device structure and a pointer to store the result.
 * It returns an error if the device structure is null.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null. If null, the function returns an error.
 * @param num_channels A pointer to a uint8_t where the number of enabled
 * channels will be stored. The caller must provide a valid
 * memory location.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int32_t ad469x_get_num_channels(struct ad469x_dev *dev,
				uint8_t *num_channels);

/* check if channel is a temperature channel */
/***************************************************************************//**
 * @brief This function determines whether a specified channel on the AD469x
 * device is configured as a temperature channel. It should be called
 * when you need to verify the channel type, particularly in applications
 * where temperature monitoring is involved. The function requires a
 * valid device structure and a channel number to operate correctly. If
 * the device structure is null, the function will return false and print
 * an error message. The function returns true if the temperature channel
 * is enabled and the specified channel matches the device's temperature
 * channel number.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device. Must
 * not be null. If null, the function returns false and prints an
 * error message.
 * @param channel An 8-bit unsigned integer representing the channel number to
 * check. It should be within the valid range of channels for the
 * device.
 * @return Returns a boolean value: true if the specified channel is a
 * temperature channel and temperature monitoring is enabled; false
 * otherwise.
 ******************************************************************************/
bool ad469x_is_temp_channel(struct ad469x_dev *dev,
			    uint8_t channel);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up and initializes an AD469x device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * allocates memory for the device structure, configures SPI
 * communication, and sets up GPIOs and other device-specific settings.
 * It also performs a device reset and verifies communication by writing
 * and reading a test value. If extended initialization is enabled,
 * additional configuration is performed. The function handles errors by
 * cleaning up resources and returns a negative value if initialization
 * fails.
 *
 * @param device A pointer to a pointer of type `struct ad469x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad469x_init_param` containing the
 * initialization parameters. Must not be null and should be
 * properly configured with valid settings for the device.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int32_t ad469x_init(struct ad469x_dev **device,
		    struct ad469x_init_param *init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function is used to properly release all resources associated
 * with an AD469x device, including SPI, GPIO, and optionally PWM and
 * clock generator resources, depending on the configuration. It should
 * be called when the device is no longer needed to ensure that all
 * allocated resources are freed and any associated hardware is properly
 * shut down. The function must be called with a valid device structure
 * that was previously initialized. If the device pointer is null, the
 * function returns an error code immediately. The function returns an
 * error code if any of the resource removal operations fail, allowing
 * the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an ad469x_dev structure representing the device to be
 * removed. Must not be null. The function returns an error code if
 * this parameter is null.
 * @return Returns 0 on success, or a negative error code if any resource
 * removal operation fails.
 ******************************************************************************/
int32_t ad469x_remove(struct ad469x_dev *dev);

#endif /* SRC_AD469X_H_ */

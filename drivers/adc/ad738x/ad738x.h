/***************************************************************************//**
 *   @file   ad738x.h
 *   @brief  Header file for AD738x Driver.
 *   @author SPopa (stefan.popa@analog.com)
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#ifndef SRC_AD738X_H_
#define SRC_AD738X_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_util.h"
#include "clk_axi_clkgen.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/*
 * AD738X registers definition
 */
#define AD738X_REG_NOP                  0x00
#define AD738X_REG_CONFIG1              0x01
#define AD738X_REG_CONFIG2              0x02
#define AD738X_REG_ALERT                0x03
#define AD738X_REG_ALERT_LOW_TH         0x04
#define AD738X_REG_ALERT_HIGH_TH        0x05

/*
 * AD738X_REG_CONFIG1
 */
#define AD738X_CONFIG1_OS_MODE_MSK      NO_OS_BIT(9)
#define AD738X_CONFIG1_OS_MODE(x)       (((x) & 0x1) << 9)
#define AD738X_CONFIG1_OSR_MSK          NO_OS_GENMASK(8, 6)
#define AD738X_CONFIG1_OSR(x)           (((x) & 0x7) << 6)
#define AD738X_CONFIG1_CRC_W_MSK        NO_OS_BIT(5)
#define AD738X_CONFIG1_CRC_W(x)         (((x) & 0x1) << 5)
#define AD738X_CONFIG1_CRC_R_MSK        NO_OS_BIT(4)
#define AD738X_CONFIG1_CRC_R(x)         (((x) & 0x1) << 4)
#define AD738X_CONFIG1_ALERTEN_MSK      NO_OS_BIT(3)
#define AD738X_CONFIG1_ALERTEN(x)       (((x) & 0x1) << 3)
#define AD738X_CONFIG1_RES_MSK          NO_OS_BIT(2)
#define AD738X_CONFIG1_RES(x)           (((x) & 0x1) << 2)
#define AD738X_CONFIG1_REFSEL_MSK       NO_OS_BIT(1)
#define AD738X_CONFIG1_REFSEL(x)        (((x) & 0x1) << 1)
#define AD738X_CONFIG1_PMODE_MSK        NO_OS_BIT(0)
#define AD738X_CONFIG1_PMODE(x)         (((x) & 0x1) << 0)

/*
 * AD738X_REG_CONFIG2
 */
#define AD738X_CONFIG2_SDO2_MSK         NO_OS_BIT(8)
#define AD738X_CONFIG2_SDO2(x)          (((x) & 0x1) << 8)
#define AD738X_CONFIG2_SDO4_MSK         NO_OS_GENMASK(9, 8)
#define AD738X_CONFIG2_SDO4(x)          (((x) & 0x3) << 8)
#define AD738X_CONFIG2_RESET_MSK        NO_OS_GENMASK(7, 0)
#define AD738X_CONFIG2_RESET(x)         (((x) & 0xFF) << 0)

/*
 * AD738X_REG_ALERT_LOW_TH
 */
#define AD738X_ALERT_LOW_MSK            NO_OS_GENMASK(11, 0)
#define AD738X_ALERT_LOW(x)             (((x) & 0xFFF) << 0)

/*
 * AD738X_REG_ALERT_HIGH_TH
 */
#define AD738X_ALERT_HIGH_MSK           NO_OS_GENMASK(11, 0)
#define AD738X_ALERT_HIGH(x)            (((x) & 0xFFF) << 0)

/* Write to register x */
#define AD738X_REG_WRITE(x)             ((1 << 7) | ((x & 0x7) << 4))
/* Read from register x */
#define AD738X_REG_READ(x)              ((x & 0x7) << 4)

#define AD738X_FLAG_STANDARD_SPI_DMA    NO_OS_BIT(0)
#define AD738X_FLAG_OFFLOAD             NO_OS_BIT(1)
/*****************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad738x_conv_mode` enumeration defines the possible conversion
 * modes for the AD738x device, specifying whether the device operates in
 * a two-wire or one-wire communication mode. This setting is crucial for
 * configuring the communication protocol used by the device to transmit
 * data.
 *
 * @param TWO_WIRE_MODE Represents a conversion mode using two wires for
 * communication.
 * @param ONE_WIRE_MODE Represents a conversion mode using a single wire for
 * communication.
 ******************************************************************************/
enum ad738x_conv_mode {
	TWO_WIRE_MODE,
	ONE_WIRE_MODE
};

/***************************************************************************//**
 * @brief The `ad738x_os_mode` enumeration defines the oversampling modes
 * available for the AD738x device. It includes two modes:
 * `NORMAL_OS_MODE`, which is the standard oversampling mode, and
 * `ROLLING_OS_MODE`, which provides a rolling oversampling capability.
 * These modes are used to configure the oversampling behavior of the
 * device, affecting how data is sampled and processed.
 *
 * @param NORMAL_OS_MODE Represents the normal oversampling mode.
 * @param ROLLING_OS_MODE Represents the rolling oversampling mode.
 ******************************************************************************/
enum ad738x_os_mode {
	NORMAL_OS_MODE,
	ROLLING_OS_MODE
};

/***************************************************************************//**
 * @brief The `ad738x_os_ratio` enumeration defines the possible oversampling
 * ratios for the AD738x device, allowing for different levels of data
 * averaging to improve signal quality. It includes options for disabling
 * oversampling as well as specific ratios ranging from 2 to 32.
 *
 * @param OSR_DISABLED Represents the state where oversampling is disabled.
 * @param OSR_X2 Represents an oversampling ratio of 2.
 * @param OSR_X4 Represents an oversampling ratio of 4.
 * @param OSR_X8 Represents an oversampling ratio of 8.
 * @param OSR_X16 Represents an oversampling ratio of 16.
 * @param OSR_X32 Represents an oversampling ratio of 32.
 ******************************************************************************/
enum ad738x_os_ratio {
	OSR_DISABLED,
	OSR_X2,
	OSR_X4,
	OSR_X8,
	OSR_X16,
	OSR_X32,
};

/***************************************************************************//**
 * @brief The `ad738x_resolution` enumeration defines the possible resolution
 * settings for the AD738x device, allowing the user to select between
 * 16-bit and 18-bit resolutions. This setting is crucial for determining
 * the precision of the analog-to-digital conversion performed by the
 * device.
 *
 * @param RES_16_BIT Represents a 16-bit resolution setting for the AD738x
 * device.
 * @param RES_18_BIT Represents an 18-bit resolution setting for the AD738x
 * device.
 ******************************************************************************/
enum ad738x_resolution {
	RES_16_BIT,
	RES_18_BIT
};

/***************************************************************************//**
 * @brief The `ad738x_reset_type` is an enumeration that defines the types of
 * reset operations available for the AD738x device. It includes two
 * options: `SOFT_RESET`, which allows for a reset through software
 * commands, and `HARD_RESET`, which involves a physical reset of the
 * device. This enumeration is used to specify the reset method when
 * performing a reset operation on the AD738x device.
 *
 * @param SOFT_RESET Represents a software reset operation for the AD738x
 * device.
 * @param HARD_RESET Represents a hardware reset operation for the AD738x
 * device.
 ******************************************************************************/
enum ad738x_reset_type {
	SOFT_RESET,
	HARD_RESET
};

/***************************************************************************//**
 * @brief The `ad738x_pwd_mode` enumeration defines the power-down modes
 * available for the AD738x device, which include normal and full power-
 * down modes. These modes are used to manage the power consumption of
 * the device, with the normal mode allowing for quicker wake-up times
 * and the full mode providing maximum power savings.
 *
 * @param NORMAL_PWDM Represents the normal power-down mode for the AD738x
 * device.
 * @param FULL_PWDM Represents the full power-down mode for the AD738x device.
 ******************************************************************************/
enum ad738x_pwd_mode {
	NORMAL_PWDM,
	FULL_PWDM
};

/***************************************************************************//**
 * @brief The `ad738x_ref_sel` enumeration defines the reference selection
 * options for the AD738x device, allowing the user to choose between
 * using an internal or external reference voltage. This selection is
 * crucial for configuring the device's reference voltage source, which
 * can impact the accuracy and stability of the analog-to-digital
 * conversion process.
 *
 * @param INT_REF Represents the internal reference selection for the AD738x
 * device.
 * @param EXT_REF Represents the external reference selection for the AD738x
 * device.
 ******************************************************************************/
enum ad738x_ref_sel {
	INT_REF,
	EXT_REF
};

/***************************************************************************//**
 * @brief The `ad738x_dev` structure is a comprehensive data structure used to
 * manage and configure the AD738x device, which is an analog-to-digital
 * converter. It includes pointers to various hardware descriptors such
 * as SPI, clock generator, and PWM, which are essential for device
 * communication and control. The structure also holds device-specific
 * settings like conversion mode, reference selection, and resolution,
 * allowing for flexible configuration. Additionally, it provides a
 * function pointer for cache management and a flags field for extended
 * configuration options, making it a central component in the device's
 * driver implementation.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param offload_init_param Pointer to SPI module offload initialization
 * parameters.
 * @param clkgen Pointer to the clock generator structure.
 * @param pwm_desc Pointer to the PWM descriptor.
 * @param conv_mode Conversion mode setting for the device.
 * @param ref_sel Reference selection for the device, either internal or
 * external.
 * @param ref_voltage_mv Reference voltage in millivolts.
 * @param resolution Resolution setting for the device, either 16-bit or 18-bit.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a given address range.
 * @param flags Flags for additional configuration options.
 ******************************************************************************/
struct ad738x_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/** SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	struct axi_clkgen *clkgen;
	struct no_os_pwm_desc *pwm_desc;

	/* Device Settings */
	enum ad738x_conv_mode 	conv_mode;
	enum ad738x_ref_sel		ref_sel;
	uint32_t		ref_voltage_mv;
	enum ad738x_resolution 	resolution;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	uint32_t flags;
};

/***************************************************************************//**
 * @brief The `ad738x_init_param` structure is used to initialize the AD738x
 * device, encapsulating various parameters required for SPI
 * communication, clock generation, and device-specific settings. It
 * includes pointers to initialization parameters for SPI, clock
 * generator, and PWM, as well as settings for conversion mode, reference
 * selection, and reference voltage. Additionally, it provides a function
 * pointer for cache invalidation and a flags field for extra
 * configuration options.
 *
 * @param spi_param Pointer to SPI initialization parameters.
 * @param clkgen_init Pointer to AXI clock generator initialization parameters.
 * @param axi_clkgen_rate Clock rate for the AXI clock generator.
 * @param offload_init_param Pointer to SPI engine offload initialization
 * parameters.
 * @param pwm_init Pointer to PWM initialization parameters.
 * @param conv_mode Conversion mode for the AD738x device.
 * @param ref_sel Reference selection for the AD738x device.
 * @param ref_voltage_mv Reference voltage in millivolts.
 * @param dcache_invalidate_range Function pointer to invalidate data cache for
 * a given address range.
 * @param flags Flags for additional configuration options.
 ******************************************************************************/
struct ad738x_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_param;
	struct axi_clkgen_init *clkgen_init;
	uint32_t axi_clkgen_rate;
	/** SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	struct no_os_pwm_init_param *pwm_init;

	/* Device Settings */
	enum ad738x_conv_mode	conv_mode;
	enum ad738x_ref_sel		ref_sel;
	uint32_t		ref_voltage_mv;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	uint32_t flags;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function sets up the AD738x device using the provided
 * initialization parameters, configuring necessary hardware interfaces
 * and device settings. It must be called before any other operations on
 * the device to ensure proper setup. The function allocates resources
 * and initializes the device's SPI, clock generator, and PWM components,
 * among others. If initialization fails at any step, it cleans up and
 * returns an error code. Successful initialization results in a ready-
 * to-use device structure, which is returned via the provided pointer.
 *
 * @param device A pointer to a pointer of type `struct ad738x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad738x_init_param` containing
 * initialization parameters for the device. Must not be null
 * and should be properly configured before calling this
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and cleans up any allocated resources.
 ******************************************************************************/
int32_t ad738x_init(struct ad738x_dev **device,
		    struct ad738x_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD738x
 * device instance. It should be called when the device is no longer
 * needed, typically at the end of its lifecycle, to ensure proper
 * cleanup and avoid resource leaks. This function must be called only
 * after the device has been successfully initialized with `ad738x_init`.
 * It handles the removal of associated SPI, clock generator, and PWM
 * resources, and frees the memory allocated for the device structure.
 *
 * @param dev A pointer to an `ad738x_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid device instance initialized by `ad738x_init`. The function
 * will handle invalid pointers by returning an error code.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error occurred during the removal of resources.
 ******************************************************************************/
int32_t ad738x_remove(struct ad738x_dev *dev);
/***************************************************************************//**
 * @brief This function is used to read a 16-bit value from a specified register
 * of the AD738x device using SPI communication. It requires a valid
 * device structure that has been initialized and configured for SPI
 * communication. The function reads the register specified by the
 * address and stores the result in the provided data pointer. It is
 * important to ensure that the device is properly initialized before
 * calling this function, and the data pointer must not be null as it
 * will be used to store the read value. The function returns an error
 * code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD738x device.
 * @param reg_data A pointer to a uint16_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int32_t ad738x_spi_reg_read(struct ad738x_dev *dev,
			    uint8_t reg_addr,
			    uint16_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to write a 12-bit data value to a specific
 * register on the AD738x device using SPI communication. It is typically
 * called when configuration or control of the device is required. The
 * function requires a valid device structure that has been initialized
 * and configured for SPI communication. The register address and data to
 * be written must be provided. The function returns an error code if the
 * SPI communication fails, allowing the caller to handle such errors
 * appropriately.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null, and the SPI descriptor within must be
 * properly configured.
 * @param reg_addr The address of the register to which data will be written.
 * Must be a valid register address as defined by the device's
 * register map.
 * @param reg_data The 12-bit data to be written to the specified register. The
 * data is masked and shifted appropriately to fit the
 * register's format.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in SPI communication.
 ******************************************************************************/
int32_t ad738x_spi_reg_write(struct ad738x_dev *dev,
			     uint8_t reg_addr,
			     uint16_t reg_data);
/***************************************************************************//**
 * @brief This function retrieves a single conversion result from the AD738x
 * device using SPI communication. It should be called when a conversion
 * result is needed from the device. The function requires a valid device
 * structure that has been initialized and configured for the desired
 * conversion mode. The conversion mode affects the length of the data
 * read. The function writes the conversion result into the provided
 * buffer. It returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null. The device should be properly configured
 * before calling this function.
 * @param adc_data A pointer to a uint32_t variable where the conversion result
 * will be stored. Must not be null. The function writes the
 * conversion data to this location.
 * @return Returns an int32_t error code indicating the success or failure of
 * the SPI communication. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad738x_spi_single_conversion(struct ad738x_dev *dev,
				     uint32_t *adc_data);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * AD738x device by applying a mask and writing new data. It is useful
 * when only certain bits of a register need to be updated without
 * affecting the other bits. The function first reads the current value
 * of the register, applies the mask to clear the bits to be modified,
 * and then writes the new data to those bits. It should be called when
 * the device is properly initialized and the register address is valid.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD738x device.
 * @param mask A 32-bit mask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the masked bits of the register.
 * Only the bits specified by the mask will be updated.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating a failure in reading or writing the register.
 ******************************************************************************/
int32_t ad738x_spi_write_mask(struct ad738x_dev *dev,
			      uint8_t reg_addr,
			      uint32_t mask,
			      uint16_t data);
/***************************************************************************//**
 * @brief Use this function to configure the conversion mode of the AD738x
 * device, which determines the data output format. It should be called
 * after the device has been initialized and before starting any data
 * conversion operations. The function modifies the device's
 * configuration register to set the desired mode. Ensure that the device
 * pointer is valid and that the mode is a valid enumeration value.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null.
 * @param mode An enumeration value of type ad738x_conv_mode indicating the
 * desired conversion mode. Valid values are TWO_WIRE_MODE and
 * ONE_WIRE_MODE.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad738x_set_conversion_mode(struct ad738x_dev *dev,
				   enum ad738x_conv_mode mode);
/***************************************************************************//**
 * @brief This function is used to reset the AD738x device, either through a
 * soft or hard reset, depending on the specified reset type. It should
 * be called when a reset of the device is necessary, such as during
 * initialization or to recover from an error state. The function
 * requires a valid device structure and a reset type, and it
 * communicates with the device over SPI to perform the reset. Ensure
 * that the device has been properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before use.
 * @param reset An enum value of type ad738x_reset_type indicating the type of
 * reset to perform. Valid values are SOFT_RESET and HARD_RESET.
 * @return Returns an int32_t status code indicating success or failure of the
 * reset operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad738x_reset(struct ad738x_dev *dev,
		     enum ad738x_reset_type reset);
/***************************************************************************//**
 * @brief Use this function to set the oversampling mode, oversampling ratio,
 * and resolution for an AD738x device. This function should be called
 * after the device has been initialized and before starting any data
 * acquisition processes that require specific oversampling settings. It
 * modifies the device's configuration registers to apply the specified
 * settings. Ensure that the device pointer is valid and that the mode,
 * ratio, and resolution parameters are within their respective
 * enumerated types.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null.
 * @param os_mode An enum ad738x_os_mode value specifying the oversampling mode.
 * Valid values are NORMAL_OS_MODE and ROLLING_OS_MODE.
 * @param os_ratio An enum ad738x_os_ratio value specifying the oversampling
 * ratio. Valid values range from OSR_DISABLED to OSR_X32.
 * @param res An enum ad738x_resolution value specifying the resolution. Valid
 * values are RES_16_BIT and RES_18_BIT.
 * @return Returns an int32_t indicating success (0) or a negative error code if
 * the configuration fails.
 ******************************************************************************/
int32_t ad738x_oversampling_config(struct ad738x_dev *dev,
				   enum ad738x_os_mode os_mode,
				   enum ad738x_os_ratio os_ratio,
				   enum ad738x_resolution res);
/***************************************************************************//**
 * @brief This function configures the power-down mode of the AD738x device,
 * allowing the user to switch between normal and full power-down modes.
 * It should be called when the device needs to be put into a low-power
 * state, either to conserve energy or to prepare for a shutdown. The
 * function requires a valid device structure and a power-down mode
 * enumeration value. It is important to ensure that the device has been
 * properly initialized before calling this function to avoid undefined
 * behavior.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before use.
 * @param pmode An enumeration value of type ad738x_pwd_mode indicating the
 * desired power-down mode. Valid values are NORMAL_PWDM and
 * FULL_PWDM.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad738x_power_down_mode(struct ad738x_dev *dev,
			       enum ad738x_pwd_mode pmode);
/***************************************************************************//**
 * @brief This function configures the AD738x device to use either an internal
 * or external reference voltage source. It should be called when the
 * reference source needs to be set or changed, typically during device
 * initialization or configuration. The function requires a valid device
 * structure and a reference selection enumeration value. It returns an
 * error code if the operation fails, which can occur if the device
 * structure is not properly initialized.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param ref_sel An enumeration value of type ad738x_ref_sel indicating the
 * desired reference voltage source. Valid values are INT_REF for
 * internal reference and EXT_REF for external reference.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad738x_reference_sel(struct ad738x_dev *dev,
			     enum ad738x_ref_sel ref_sel);
/***************************************************************************//**
 * @brief This function retrieves a specified number of conversion samples from
 * the AD738x device and stores them in the provided buffer. It supports
 * different data acquisition modes, including offload and standard SPI
 * DMA, based on the device's configuration flags. The function should be
 * called when the device is properly initialized and configured for data
 * reading. It handles the reading process according to the device's
 * current mode and returns an error code if the reading fails at any
 * point.
 *
 * @param dev A pointer to an initialized ad738x_dev structure representing the
 * device. Must not be null. The device should be configured for data
 * reading.
 * @param buf A pointer to a buffer where the read conversion data will be
 * stored. Must not be null and should have enough space to hold
 * 'samples' number of 32-bit integers.
 * @param samples The number of conversion samples to read. Must be a positive
 * integer. The function will attempt to read this many samples
 * from the device.
 * @return Returns 0 on success, or a negative error code if the reading process
 * fails.
 ******************************************************************************/
int32_t ad738x_read_data(struct ad738x_dev *dev,
			 uint32_t *buf,
			 uint16_t samples);
#endif /* SRC_AD738X_H_ */

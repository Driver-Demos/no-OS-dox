/***************************************************************************//**
 *   @file   ad463x.h
 *   @brief  Header file of AD463x Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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
#ifndef AD463X_H_
#define AD463x_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"
#include "spi_engine.h"
#include "clk_axi_clkgen.h"
#include "no_os_pwm.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Types Declarations *************************/
/******************************************************************************/

/* Register addresses */
#define AD463X_REG_INTERFACE_CONFIG_A	0x00
#define AD463X_REG_INTERFACE_CONFIG_B	0x01
#define AD463X_REG_DEVICE_CONFIG	0x02
#define AD463X_REG_CHIP_TYPE		0x03
#define AD463X_REG_PRODUCT_ID_L		0x04
#define AD463X_REG_PRODUCT_ID_H		0x05
#define AD463X_REG_CHIP_GRADE		0x06
#define AD463X_REG_SCRATCH_PAD		0x0A
#define AD463X_REG_SPI_REVISION		0x0B
#define AD463X_REG_VENDOR_L		0x0C
#define AD463X_REG_VENDOR_H		0x0D
#define AD463X_REG_STREAM_MODE		0x0E
#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_AVG			0x15
#define AD463X_REG_OFFSET_BASE		0x16
#define AD463X_REG_OFFSET_X0_0		0x16
#define AD463X_REG_OFFSET_X0_1		0x17
#define AD463X_REG_OFFSET_X0_2		0x18
#define AD463X_REG_OFFSET_X1_0		0x19
#define AD463X_REG_OFFSET_X1_1		0x1A
#define AD463X_REG_OFFSET_X1_2		0x1B
#define AD463X_REG_GAIN_BASE		0x1C
#define AD463X_REG_GAIN_X0_LSB		0x1C
#define AD463X_REG_GAIN_X0_MSB		0x1D
#define AD463X_REG_GAIN_X1_LSB		0x1E
#define AD463X_REG_GAIN_X1_MSB		0x1F
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_REG_IO			0x22
#define AD463X_REG_PAT0			0x23
#define AD463X_REG_PAT1			0x24
#define AD463X_REG_PAT2			0x25
#define AD463X_REG_PAT3			0x26
#define AD463X_REG_DIG_DIAG		0x34
#define AD463X_REG_DIG_ERR		0x35
/* INTERFACE_CONFIG_A */
#define AD463X_CFG_SW_RESET		(NO_OS_BIT(7) | NO_OS_BIT(0))
#define AD463X_CFG_SDO_ENABLE		NO_OS_BIT(4)
/* MODES */
#define AD463X_SW_RESET_MSK		(NO_OS_BIT(7) | NO_OS_BIT(0))
#define AD463X_LANE_MODE_MSK		(NO_OS_BIT(7) | NO_OS_BIT(6))
#define AD463X_CLK_MODE_MSK		(NO_OS_BIT(5) | NO_OS_BIT(4))
#define AD463X_DDR_MODE_MSK		NO_OS_BIT(3)
#define AD463X_SDR_MODE			0x00
#define AD463X_DDR_MODE			NO_OS_BIT(3)
#define AD463X_OUT_DATA_MODE_MSK	(NO_OS_BIT(2) | NO_OS_BIT(1) | NO_OS_BIT(0))
#define AD463X_24_DIFF			0x00
#define AD463X_16_DIFF_8_COM		0x01
#define AD463X_24_DIFF_8_COM		0x02
#define AD463X_30_AVERAGED_DIFF		0x03
#define AD463X_32_PATTERN		0x04
/* EXIT_CFG_MD */
#define AD463X_EXIT_CFG_MODE		NO_OS_BIT(0)
/* CHANNEL */
#define AD463X_CHANNEL_0		0x00
#define AD463X_CHANNEL_1		0x01
/* OFFSET */
#define AD463X_OFFSET_0			0x00
#define AD463X_OFFSET_1			0x01
#define AD463X_OFFSET_2			0x02
/* GAIN */
#define AD463X_GAIN_LSB			0x00
#define AD463X_GAIN_MSB			0x01
/* LANE MODE */
#define AD463X_ONE_LANE_PER_CH		0x00
#define AD463X_TWO_LANES_PER_CH		NO_OS_BIT(6)
#define AD463X_FOUR_LANES_PER_CH	NO_OS_BIT(7)
#define AD463X_SHARED_TWO_CH		(NO_OS_BIT(6) | NO_OS_BIT(7))
/* CLK MODE */
#define AD463X_SPI_COMPATIBLE_MODE	0x00
#define AD463X_ECHO_CLOCK_MODE		NO_OS_BIT(4)
#define AD463X_CLOCK_MASTER_MODE	NO_OS_BIT(5)
/* POWER MODE */
#define AD463X_NORMAL_MODE 		0x00
#define AD463X_LOW_POWER_MODE		(NO_OS_BIT(1) | NO_OS_BIT(0))
/* AVG */
#define AD463X_AVG_FILTER_RESET		NO_OS_BIT(7)
#define AD463X_CONFIG_TIMING		0x2000
#define AD463X_REG_READ_DUMMY		0x00
#define AD463X_REG_WRITE        	0x00
#define AD463X_REG_READ		    	NO_OS_BIT(7)
#define AD463X_REG_CHAN_OFFSET(ch, pos)	(AD463X_REG_OFFSET_BASE + (3 * ch) + pos)
#define AD463X_REG_CHAN_GAIN(ch, pos)	(AD463X_REG_GAIN_BASE + (2 * ch) + pos)
/* IO */
#define AD463X_DRIVER_STRENGTH_MASK	NO_OS_BIT(0)
#define AD463X_NORMAL_OUTPUT_STRENGTH	0x00
#define AD463X_DOUBLE_OUTPUT_STRENGTH	NO_OS_BIT(1)
/* OUT_DATA_PAT */
#define AD463X_OUT_DATA_PAT		0x5A5A0F0F

#define AD463X_TRIGGER_PULSE_WIDTH_NS	0x0A

#define AD463X_GAIN_MAX_VAL_SCALED	19997

/***************************************************************************//**
 * @brief The `ad463x_id` enumeration defines a set of constants representing
 * different device types within the AD463x series, as well as other
 * related devices like the AD4030 and ADAQ4224. Each enumerator
 * corresponds to a specific model and configuration of the device,
 * allowing for easy identification and selection of the device type in
 * the code. This enumeration is typically used in device initialization
 * and configuration processes to specify which device is being
 * interfaced with.
 *
 * @param ID_AD4630_24 Represents the AD4630-24 device.
 * @param ID_AD4630_20 Represents the AD4630-20 device.
 * @param ID_AD4630_16 Represents the AD4630-16 device.
 * @param ID_AD4631_24 Represents the AD4631-24 device.
 * @param ID_AD4631_20 Represents the AD4631-20 device.
 * @param ID_AD4631_16 Represents the AD4631-16 device.
 * @param ID_AD4632_24 Represents the AD4632-24 device.
 * @param ID_AD4632_20 Represents the AD4632-20 device.
 * @param ID_AD4632_16 Represents the AD4632-16 device.
 * @param ID_AD4030 Represents the AD4030 device.
 * @param ID_ADAQ4224 Represents the ADAQ4224 device.
 ******************************************************************************/
enum ad463x_id {
	/** AD4630-24 device */
	ID_AD4630_24,
	/** AD4630-20 device */
	ID_AD4630_20,
	/** AD4630-16 device */
	ID_AD4630_16,
	/** AD4631-24 device */
	ID_AD4631_24,
	/** AD4631-20 device */
	ID_AD4631_20,
	/** AD4631-16 device */
	ID_AD4631_16,
	/** AD4632-24 device */
	ID_AD4632_24,
	/** AD4632-20 device */
	ID_AD4632_20,
	/** AD4632-16 device */
	ID_AD4632_16,
	/** AD4030 device */
	ID_AD4030,
	/** ADAQ4224 device */
	ID_ADAQ4224,
};

/***************************************************************************//**
 * @brief The `ad463x_pgia_gain` enumeration defines a set of constants
 * representing different programmable gain amplifier (PGIA) gain
 * settings for the AD463x series of devices. Each constant corresponds
 * to a specific gain ratio of output voltage to input voltage, allowing
 * for flexible adjustment of signal amplification based on application
 * requirements.
 *
 * @param AD463X_GAIN_0_33 Represents a gain setting where the output voltage is
 * 0.33 times the input voltage.
 * @param AD463X_GAIN_0_56 Represents a gain setting where the output voltage is
 * 0.56 times the input voltage.
 * @param AD463X_GAIN_2_22 Represents a gain setting where the output voltage is
 * 2.22 times the input voltage.
 * @param AD463X_GAIN_6_67 Represents a gain setting where the output voltage is
 * 6.67 times the input voltage.
 ******************************************************************************/
enum ad463x_pgia_gain {
	/** Vout/Vin = 0.33  */
	AD463X_GAIN_0_33 = 0,
	/** Vout/Vin = 0.56  */
	AD463X_GAIN_0_56 = 1,
	/** Vout/Vin = 2.22  */
	AD463X_GAIN_2_22 = 2,
	/** Vout/Vin = 6.67  */
	AD463X_GAIN_6_67 = 3,
};

/***************************************************************************//**
 * @brief The `ad463x_init_param` structure is used to initialize the AD463x
 * device, encapsulating various configuration parameters such as SPI,
 * GPIO, PWM, and clock generator settings. It includes fields for
 * device-specific settings like data width, lane mode, clock mode, and
 * data rate, as well as function pointers for cache management. This
 * structure is essential for setting up the device's communication
 * interfaces and operational modes before use.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_resetn Pointer to GPIO initialization parameters for reset.
 * @param gpio_cnv Pointer to GPIO initialization parameters for conversion.
 * @param gpio_pgia_a0 Pointer to GPIO initialization parameters for PGIA A0.
 * @param gpio_pgia_a1 Pointer to GPIO initialization parameters for PGIA A1.
 * @param trigger_pwm_init Pointer to PWM initialization parameters for trigger.
 * @param offload_init_param Pointer to SPI engine offload initialization
 * parameters.
 * @param clkgen_init Pointer to clock generator initialization structure.
 * @param axi_clkgen_rate Clock generator rate in Hz.
 * @param reg_access_speed Register access speed in Hz.
 * @param device_id Enum representing the device ID.
 * @param reg_data_width Width of register data in bits.
 * @param lane_mode Mode of data lanes.
 * @param clock_mode Mode of clock operation.
 * @param data_rate Data rate mode.
 * @param vref Reference voltage in millivolts.
 * @param output_mode Mode of output data.
 * @param spi_dma_enable Boolean to enable SPI DMA.
 * @param offload_enable Boolean to enable SPI engine offload.
 * @param dcache_invalidate_range Function pointer to invalidate data cache for
 * a given address range.
 ******************************************************************************/
struct ad463x_init_param {
	/** SPI */
	struct no_os_spi_init_param *spi_init;
	/** GPIO */
	struct no_os_gpio_init_param *gpio_resetn;
	struct no_os_gpio_init_param *gpio_cnv;
	struct no_os_gpio_init_param *gpio_pgia_a0;
	struct no_os_gpio_init_param *gpio_pgia_a1;
	/** PWM */
	struct no_os_pwm_init_param *trigger_pwm_init;
	/** SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	/** Clock gen for hdl design init structure */
	struct axi_clkgen_init *clkgen_init;
	/** Clock generator rate */
	uint32_t axi_clkgen_rate;
	/** Register access speed */
	uint32_t reg_access_speed;
	/** Device id */
	enum ad463x_id device_id;
	/** Register data width */
	uint8_t reg_data_width;
	/** Lane Mode */
	uint8_t lane_mode;
	/** Clock Mode */
	uint8_t clock_mode;
	/** Data Rate Mode */
	uint8_t data_rate;
	/** Reference voltage */
	int32_t vref;
	/** Output Mode */
	uint8_t output_mode;
	/** enable spi dma */
	bool spi_dma_enable;
	/** enable spi engine offload */
	bool offload_enable;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/***************************************************************************//**
 * @brief The `ad463x_dev` structure is a comprehensive representation of the
 * AD463x device configuration and state, encapsulating all necessary
 * components for its operation. It includes descriptors for SPI, GPIO,
 * and PWM interfaces, as well as parameters for SPI engine offload and
 * clock generation. The structure also holds device-specific settings
 * such as register access speed, data width, and operational modes,
 * along with configuration for PGIA and data capture. Additionally, it
 * provides function pointers for cache management, making it a central
 * entity for managing the AD463x device's functionality and integration
 * into a system.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_resetn Pointer to the GPIO descriptor for the reset pin.
 * @param gpio_cnv Pointer to the GPIO descriptor for the conversion pin.
 * @param gpio_pgia_a0 Pointer to the GPIO descriptor for the PGIA A0 pin.
 * @param gpio_pgia_a1 Pointer to the GPIO descriptor for the PGIA A1 pin.
 * @param trigger_pwm_desc Pointer to the PWM descriptor for triggering.
 * @param offload_init_param Pointer to the SPI engine offload initialization
 * parameters.
 * @param clkgen Pointer to the clock generator structure for HDL design.
 * @param reg_access_speed Speed of register access in Hz.
 * @param device_id Identifier for the specific AD463x device variant.
 * @param reg_data_width Width of the register data in bits.
 * @param read_bytes_no Number of bytes to be read from the device.
 * @param capture_data_width Width of the data capture in bits.
 * @param lane_mode Mode of data lane configuration.
 * @param clock_mode Mode of clock configuration.
 * @param data_rate Data rate mode of the device.
 * @param vref Reference voltage for the device.
 * @param pgia_idx Index for the PGIA setting.
 * @param scale_table Table of available scales for the device.
 * @param output_mode Mode of output data configuration.
 * @param real_bits_precision Precision of the ADC in bits.
 * @param has_pgia Boolean indicating if PGIA is available.
 * @param spi_dma_enable Boolean indicating if SPI DMA is enabled.
 * @param offload_enable Boolean indicating if SPI engine offload is enabled.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a given address range.
 ******************************************************************************/
struct ad463x_dev {
	/** SPI */
	struct no_os_spi_desc *spi_desc;
	/** GPIO */
	struct no_os_gpio_desc *gpio_resetn;
	struct no_os_gpio_desc *gpio_cnv;
	struct no_os_gpio_desc *gpio_pgia_a0;
	struct no_os_gpio_desc *gpio_pgia_a1;
	/** PWM */
	struct no_os_pwm_desc *trigger_pwm_desc;
	/** SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	/** Clock gen for hdl design structure */
	struct axi_clkgen *clkgen;
	/** Register access speed */
	uint32_t reg_access_speed;
	/** Device id */
	enum ad463x_id device_id;
	/** Register data width */
	uint8_t reg_data_width;
	/** Number of bytes to be read */
	uint8_t read_bytes_no;
	/** Capture data width */
	uint8_t capture_data_width;
	/** Lane Mode */
	uint8_t lane_mode;
	/** Clock Mode */
	uint8_t clock_mode;
	/** Data Rate Mode */
	uint8_t data_rate;
	/** Reference voltage */
	int32_t vref;
	/** pgia index */
	uint8_t pgia_idx;
	/** available scales */
	int32_t scale_table[4][2];
	/** Output Mode */
	uint8_t output_mode;
	/** ADC precision in bits */
	uint8_t real_bits_precision;
	/** pgia availability */
	bool has_pgia;
	/** enable spi dma */
	bool spi_dma_enable;
	/** enable spi engine offload */
	bool offload_enable;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function is used to read a specific register from an AD463x
 * device using SPI communication. It requires a valid device structure
 * that has been properly initialized and configured for SPI
 * communication. The function reads the register specified by the
 * address and stores the result in the provided data pointer. It is
 * essential to ensure that the device is correctly initialized before
 * calling this function to avoid communication errors. The function
 * returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to be read. Valid register
 * addresses are defined by the device's register map.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t error code indicating the success or failure of
 * the SPI read operation.
 ******************************************************************************/
int32_t ad463x_spi_reg_read(struct ad463x_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register of an AD463x device using SPI communication. It is typically
 * called when there is a need to configure or modify the settings of the
 * device by writing to its registers. The function requires a valid
 * device structure that has been properly initialized, a register
 * address, and the data to be written. It is important to ensure that
 * the device is in a state that allows register writing, and that the
 * register address is within the valid range for the device. The
 * function returns an integer status code indicating the success or
 * failure of the operation.
 *
 * @param dev A pointer to an ad463x_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership.
 * @param reg_addr A 16-bit unsigned integer specifying the address of the
 * register to write to. The address must be within the valid
 * range of the device's register map.
 * @param reg_data An 8-bit unsigned integer representing the data to be written
 * to the specified register. The data should be formatted
 * according to the register's requirements.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad463x_spi_reg_write(struct ad463x_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data);

/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD463x
 * device, applies a mask to the read value, and returns the masked
 * result. It is useful when only specific bits of a register are of
 * interest. The function must be called with a valid device structure
 * and a valid register address. The caller must ensure that the `data`
 * pointer is not null, as it will be used to store the masked result. If
 * the read operation fails, the function returns an error code.
 *
 * @param dev A pointer to an initialized `ad463x_dev` structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD463x device.
 * @param mask A bitmask to apply to the read register value. Determines which
 * bits of the register are of interest.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t ad463x_spi_reg_read_masked(struct ad463x_dev *dev,
				   uint16_t reg_addr,
				   uint8_t mask,
				   uint8_t *data);

/***************************************************************************//**
 * @brief This function allows writing to a specific register of the AD463x
 * device by first reading the current register value, applying a mask to
 * clear specific bits, and then setting those bits to the provided data.
 * It is useful for modifying only certain bits of a register without
 * affecting the others. The function must be called with a valid device
 * structure and register address. It handles reading and writing errors
 * by returning a non-zero error code.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be written to. Must be a valid
 * register address for the AD463x device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The data to be written to the masked bits of the register. Only
 * bits corresponding to the mask will be written.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int32_t ad463x_spi_reg_write_masked(struct ad463x_dev *dev,
				    uint16_t reg_addr,
				    uint8_t mask,
				    uint8_t data);

/***************************************************************************//**
 * @brief This function configures the power mode of the AD463x device to either
 * normal or low power mode. It should be called when a change in power
 * consumption is required, such as during different operational states
 * of the device. The function requires a valid device structure and a
 * mode value that specifies the desired power mode. If an invalid mode
 * is provided, the function returns an error code. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the desired power mode. Valid values
 * are AD463X_NORMAL_MODE and AD463X_LOW_POWER_MODE. Invalid values
 * result in an error return.
 * @return Returns 0 on success, or -1 if the mode is invalid.
 ******************************************************************************/
int32_t ad463x_set_pwr_mode(struct ad463x_dev *dev, uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the average frame length of the AD463x device
 * based on the specified mode. It should be called when you need to
 * adjust the data output mode of the device, either to normal mode or to
 * an averaged mode with a specific frame length. The function requires a
 * valid device structure and a mode value. If the mode is set to normal,
 * the function resets the average filter. For averaged modes, the mode
 * value must be between 1 and 16, inclusive. The function returns an
 * error code if the mode is invalid or if any register write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the desired mode. Use
 * AD463X_NORMAL_MODE for normal mode, or a value between 1 and 16
 * for averaged modes. Invalid values result in an error.
 * @return Returns an int32_t error code: 0 on success, or a negative value on
 * failure.
 ******************************************************************************/
int32_t ad463x_set_avg_frame_len(struct ad463x_dev *dev, uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to configure the drive strength of the AD463x device
 * to either normal or double output strength. This function should be
 * called when you need to adjust the output drive capability of the
 * device, which may be necessary depending on the load conditions or
 * specific application requirements. Ensure that the device has been
 * properly initialized before calling this function. The function
 * validates the input mode and returns an error if the mode is invalid.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the desired drive strength mode.
 * Valid values are AD463X_NORMAL_OUTPUT_STRENGTH and
 * AD463X_DOUBLE_OUTPUT_STRENGTH. If an invalid mode is provided,
 * the function returns an error.
 * @return Returns 0 on success, or a negative error code if the mode is invalid
 * or if there is a failure in setting the drive strength.
 ******************************************************************************/
int32_t ad463x_set_drive_strength(struct ad463x_dev *dev, uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to exit the register configuration mode of an AD463x
 * device. It should be called after any necessary configuration changes
 * have been made in configuration mode. If the device is set to offload
 * mode, the function will also configure the SPI transfer width and
 * speed. Ensure that the device structure is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if any SPI operations fail.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad463x_exit_reg_cfg_mode(struct ad463x_dev *dev);

/***************************************************************************//**
 * @brief This function configures the gain for a specific channel on the AD463x
 * device. It should be used when you need to adjust the gain settings
 * for a particular channel, which is identified by its index. The
 * function requires a valid device structure and a gain value within the
 * permissible range. If the gain value is outside the allowed range, the
 * function returns an error. This function must be called after the
 * device has been properly initialized and configured for operation.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param ch_idx The index of the channel for which the gain is to be set. Valid
 * channel indices depend on the specific device configuration.
 * @param gain The desired gain value, which must be between 0 and
 * AD463X_GAIN_MAX_VAL_SCALED inclusive. If the gain is outside this
 * range, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the gain value is
 * invalid or if there is a failure in writing to the device registers.
 ******************************************************************************/
int32_t ad463x_set_ch_gain(struct ad463x_dev *dev, uint8_t ch_idx,
			   uint64_t gain);

/***************************************************************************//**
 * @brief This function sets the offset for a specific channel on the AD463x
 * device by writing the provided offset value to the appropriate
 * registers. It is typically used to calibrate or adjust the channel's
 * baseline measurement. The function must be called with a valid device
 * structure and a channel index within the supported range. It returns
 * an error code if the operation fails, which can occur if the device is
 * not properly initialized or if communication with the device fails.
 *
 * @param dev A pointer to an initialized ad463x_dev structure representing the
 * device. Must not be null.
 * @param ch_idx The index of the channel for which the offset is being set.
 * Valid values depend on the specific device configuration and
 * should be within the range of available channels.
 * @param offset The offset value to be set for the specified channel. It is a
 * 32-bit unsigned integer representing the offset to apply.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad463x_set_ch_offset(struct ad463x_dev *dev, uint8_t ch_idx,
			     uint32_t offset);

/***************************************************************************//**
 * @brief This function is used to read a specified number of samples from an
 * AD463x device and store them in a provided buffer. It should be called
 * when data acquisition from the device is required. The function
 * supports different data acquisition modes, including offload and DMA,
 * based on the device configuration. It is essential to ensure that the
 * device is properly initialized and configured before calling this
 * function. The function handles invalid device pointers by returning an
 * error code.
 *
 * @param dev A pointer to an ad463x_dev structure representing the device. Must
 * not be null. If null, the function returns an error code.
 * @param buf A pointer to a buffer where the read samples will be stored. The
 * buffer must be large enough to hold the specified number of
 * samples.
 * @param samples The number of samples to read from the device. Must be a
 * positive integer.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad463x_read_data(struct ad463x_dev *dev,
			 uint32_t *buf,
			 uint16_t samples);

/***************************************************************************//**
 * @brief This function initializes an AD463x device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the device's SPI
 * interface, GPIOs, and other configurations based on the parameters. It
 * handles various modes and configurations, such as clock and data rate
 * modes, and ensures the device is ready for operation. If
 * initialization fails at any step, it cleans up allocated resources and
 * returns an error code.
 *
 * @param device A pointer to a pointer of type `struct ad463x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad463x_init_param` containing
 * initialization parameters. Must not be null. The structure
 * should be properly filled with valid configuration settings
 * for the device.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and does not allocate the device structure.
 ******************************************************************************/
int32_t ad463x_init(struct ad463x_dev **device,
		    struct ad463x_init_param *init_param);

/***************************************************************************//**
 * @brief This function determines the closest Programmable Gain Instrumentation
 * Amplifier (PGIA) gain index for a given integer and fractional gain,
 * reference voltage, and precision. It is used to configure the gain
 * settings of the AD463x device. The function requires a valid pointer
 * to store the resulting gain index. If the calculated gain exceeds the
 * maximum allowable value, the function returns an error. This function
 * should be called when precise gain configuration is needed for the
 * device.
 *
 * @param gain_int The integer part of the desired gain. It should be a non-
 * negative integer.
 * @param gain_fract The fractional part of the desired gain, expressed in
 * nanounits. It should be a non-negative integer.
 * @param vref The reference voltage used in the gain calculation. It should be
 * a positive integer.
 * @param precision The precision factor used in the calculation, typically
 * representing the number of bits. It should be a non-negative
 * integer.
 * @param gain_idx A pointer to an enum ad463x_pgia_gain where the calculated
 * gain index will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the gain index
 * cannot be calculated due to invalid input or exceeding maximum gain.
 ******************************************************************************/
int32_t ad463x_calc_pgia_gain(int32_t gain_int, int32_t gain_fract,
			      int32_t vref,
			      int32_t precision,
			      enum ad463x_pgia_gain *gain_idx);

/***************************************************************************//**
 * @brief Use this function to configure the Programmable Gain Instrumentation
 * Amplifier (PGIA) gain of an AD463x device. This function should be
 * called only if the device supports PGIA, as indicated by the
 * `has_pgia` field in the device structure. The function requires a
 * valid device structure and a gain index within the supported range. It
 * updates the device's gain setting and configures the corresponding
 * GPIO pins. Ensure the device is properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an `ad463x_dev` structure representing the device.
 * Must not be null. The device must support PGIA, as indicated by
 * the `has_pgia` field.
 * @param gain_idx An `enum ad463x_pgia_gain` value representing the desired
 * gain setting. Must be between 0 and 3, inclusive. Invalid
 * values result in an error.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or unsupported operations.
 ******************************************************************************/
int32_t ad463x_set_pgia_gain(struct ad463x_dev *dev,
			     enum ad463x_pgia_gain gain_idx);

/***************************************************************************//**
 * @brief This function is used to release all resources allocated for an AD463x
 * device, including SPI, GPIO, PWM, and clock generator components. It
 * should be called when the device is no longer needed to ensure proper
 * cleanup and avoid resource leaks. The function must be called with a
 * valid device structure that was previously initialized. If the device
 * structure is null, the function returns an error code. It also handles
 * the optional removal of the clock generator if offload is enabled.
 *
 * @param dev A pointer to an ad463x_dev structure representing the device to be
 * removed. Must not be null. If null, the function returns an error
 * code.
 * @return Returns 0 on successful removal of all resources, or a negative error
 * code if any resource removal fails.
 ******************************************************************************/
int32_t ad463x_remove(struct ad463x_dev *dev);


/***************************************************************************//**
 * @brief This function is used to switch the AD463x device into configuration
 * register mode, allowing for subsequent configuration changes. It
 * should be called when the device needs to be reconfigured, and it is
 * essential that the device structure is properly initialized and not
 * null before calling this function. The function communicates with the
 * device over SPI, and any issues with the SPI communication will result
 * in an error code being returned.
 *
 * @param dev A pointer to an initialized 'ad463x_dev' structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an SPI communication error.
 ******************************************************************************/
int32_t ad463x_enter_config_mode(struct ad463x_dev *dev);

#endif // AD463X_H_

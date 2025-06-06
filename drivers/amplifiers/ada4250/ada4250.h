/***************************************************************************//**
 *   @file   ada4250.h
 *   @brief  Header file for ada4250 Driver.
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

#ifndef ADA4250_H_
#define ADA4250_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* ADA4250 Register Map */
#define ADA4250_REG_GAIN_MUX        0x00
#define ADA4250_REG_REFBUF_EN       0x01
#define ADA4250_REG_RESET           0x02
#define ADA4250_REG_SNSR_CAL_VAL    0x04
#define ADA4250_REG_SNSR_CAL_CNFG   0x05
#define ADA4250_REG_DIE_REV         0x18
#define ADA4250_REG_CHIP_ID1        0x19
#define ADA4250_REG_CHIP_ID2        0x1a

/* ADA4250_REG_GAIN_MUX Map */
#define ADA4250_GAIN_MUX_MSK        NO_OS_GENMASK(2, 0)
#define ADA4250_GAIN_MUX(x)         no_os_field_prep(ADA4250_GAIN_MUX_MSK, x)

/* ADA4250_REG_REFBUF Map */
#define ADA4250_REFBUF_MSK          NO_OS_BIT(0)
#define ADA4250_REFBUF(x)           no_os_field_prep(ADA4250_REFBUF_MSK, x)

/* ADA4250_REG_REFBUFF_EN Bit Definition */
#define ADA4250_BUF_ENABLE          0x01
#define ADA4250_BUF_DISABLE         0x00

/* ADA4250_REG_RESET Map */
#define ADA4250_RESET_MSK           NO_OS_BIT(0)
#define ADA4250_RESET(x)            no_os_field_prep(ADA4250_RESET_MSK, x)

/* ADA4250_REG_RESET Bit Definition */
#define ADA4250_RESET_ENABLE        0x01
#define ADA4250_RESET_DISABLE       0x00

/* ADA4250_REG_SNSR_CAL_VAL Map */
#define ADA4250_SNSR_CAL_VAL_MSK    NO_OS_GENMASK(7, 0)
#define ADA4250_SNSR_CAL_VAL(x)     no_os_field_prep(ADA4250_SNSR_CAL_VAL_MSK, x)

/* ADA4250_REG_SNSR_CAL_CNFG Bit Definition */
#define ADA4250_BIAS_SET_MSK        NO_OS_GENMASK(3, 2)
#define ADA4250_BIAS_SET(x)         no_os_field_prep(ADA4250_BIAS_SET_MSK, x)
#define ADA4250_RANGE_SET_MSK       NO_OS_GENMASK(1, 0)
#define ADA4250_RANGE_SET(x)        no_os_field_prep(ADA4250_RANGE_SET_MSK, x)

/* Specifications */
#define ADA4250_SPI_WRITE_CMD		0x0
#define ADA4250_BUFF_SIZE_BYTES     2
#define ADA4250_SPI_READ_CMD		NO_OS_BIT(7)
#define ADA4250_DIE_REV             0x0
#define ADA4250_CHIP_ID             0x4250

/* ADA4250 Extra Definitions */
#define ADA4250_SPI_DUMMY_DATA		0x00

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ada4250_id` enumeration defines identifiers for different ADA
 * devices, specifically the ADA4230 and ADA4250. This enumeration is
 * used to distinguish between these two devices within the driver code,
 * allowing for device-specific operations and configurations.
 *
 * @param ADA4230 Represents the ADA4230 device identifier.
 * @param ADA4250 Represents the ADA4250 device identifier.
 ******************************************************************************/
enum ada4250_id {
	ADA4230,
	ADA4250,
};

/***************************************************************************//**
 * @brief The `ada4250_bias` enumeration defines the possible bias settings for
 * the ADA4250 device, which include disabling the bias, using a bandgap
 * reference, or using the AVDD supply voltage. This enumeration is used
 * to configure the bias setting of the device, which can affect its
 * performance and power consumption.
 *
 * @param ADA4250_BIAS_DISABLE Represents the state where the bias is disabled.
 * @param ADA4250_BIAS_BANDGAP_REF Represents the state where the bias is set to
 * use the bandgap reference.
 * @param ADA4250_BIAS_AVDD Represents the state where the bias is set to use
 * the AVDD supply voltage.
 ******************************************************************************/
enum ada4250_bias {
	ADA4250_BIAS_DISABLE,
	ADA4250_BIAS_BANDGAP_REF,
	ADA4250_BIAS_AVDD
};

/***************************************************************************//**
 * @brief The `ada4250_offset_range` enumeration defines four distinct offset
 * ranges used for sensor offset trimming in the ADA4250 device. Each
 * enumerator corresponds to a specific range setting that can be applied
 * to adjust the sensor's offset, allowing for precise calibration and
 * optimization of the sensor's performance.
 *
 * @param ADA4250_RANGE1 Represents the first offset range for sensor offset
 * trimming.
 * @param ADA4250_RANGE2 Represents the second offset range for sensor offset
 * trimming.
 * @param ADA4250_RANGE3 Represents the third offset range for sensor offset
 * trimming.
 * @param ADA4250_RANGE4 Represents the fourth offset range for sensor offset
 * trimming.
 ******************************************************************************/
enum ada4250_offset_range {
	ADA4250_RANGE1,
	ADA4250_RANGE2,
	ADA4250_RANGE3,
	ADA4250_RANGE4,
};

/***************************************************************************//**
 * @brief The `ada4250_gain` enumeration defines a set of constants representing
 * different gain settings for the ADA4250 device. Each enumerator
 * corresponds to a specific gain factor, allowing the user to select the
 * desired amplification level for the device's operation. This
 * enumeration is used to configure the gain of the ADA4250, which is a
 * precision instrumentation amplifier, to suit various application
 * requirements.
 *
 * @param ADA4250_GAIN_1 Represents a gain setting of 1.
 * @param ADA4250_GAIN_2 Represents a gain setting of 2.
 * @param ADA4250_GAIN_4 Represents a gain setting of 4.
 * @param ADA4250_GAIN_8 Represents a gain setting of 8.
 * @param ADA4250_GAIN_16 Represents a gain setting of 16.
 * @param ADA4250_GAIN_32 Represents a gain setting of 32.
 * @param ADA4250_GAIN_64 Represents a gain setting of 64.
 * @param ADA4250_GAIN_128 Represents a gain setting of 128.
 ******************************************************************************/
enum ada4250_gain {
	ADA4250_GAIN_1,
	ADA4250_GAIN_2,
	ADA4250_GAIN_4,
	ADA4250_GAIN_8,
	ADA4250_GAIN_16,
	ADA4250_GAIN_32,
	ADA4250_GAIN_64,
	ADA4250_GAIN_128,
};

/***************************************************************************//**
 * @brief The `ada4250_bandwidth` enumeration defines the available bandwidth
 * modes for the ADA4250 device, allowing the user to select between low
 * and high bandwidth settings. This enumeration is used to configure the
 * bandwidth mode of the ADA4250, which can affect the performance
 * characteristics of the device in terms of speed and signal processing
 * capabilities.
 *
 * @param ADA4250_BANDWIDTH_LOW Represents the low bandwidth mode for the
 * ADA4250 device.
 * @param ADA4250_BANDWIDTH_HIGH Represents the high bandwidth mode for the
 * ADA4250 device.
 ******************************************************************************/
enum ada4250_bandwidth {
	ADA4250_BANDWIDTH_LOW,
	ADA4250_BANDWIDTH_HIGH
};

/***************************************************************************//**
 * @brief The `ada4250_power_mode` enumeration defines the different power modes
 * available for the ADA4250 device, which include normal, sleep, and
 * shutdown modes. These modes allow the device to operate at different
 * power levels, optimizing for either performance or power savings
 * depending on the application's requirements.
 *
 * @param ADA4250_POWER_NORMAL Represents the normal power mode of the ADA4250
 * device.
 * @param ADA4250_POWER_SLEEP Represents the sleep power mode of the ADA4250
 * device.
 * @param ADA4250_POWER_SHUTDOWN Represents the shutdown power mode of the
 * ADA4250 device.
 ******************************************************************************/
enum ada4250_power_mode {
	ADA4250_POWER_NORMAL,
	ADA4250_POWER_SLEEP,
	ADA4250_POWER_SHUTDOWN,
};

/***************************************************************************//**
 * @brief The `ada4250_init_param` structure is used to define the
 * initialization parameters for the ADA4250 device. It includes various
 * configuration settings such as device ID, SPI and GPIO initialization
 * parameters, AVDD voltage, reference buffer enablement, gain, bias,
 * bandwidth, and offset calibration. This structure is essential for
 * setting up the ADA4250 device with the desired operational parameters
 * before use.
 *
 * @param device_id Specifies the device ID using the ada4250_id enumeration.
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_g2_param Pointer to GPIO initialization parameters for G2.
 * @param gpio_g1_param Pointer to GPIO initialization parameters for G1.
 * @param gpio_g0_param Pointer to GPIO initialization parameters for G0.
 * @param gpio_bw_param Pointer to GPIO initialization parameters for bandwidth
 * control.
 * @param gpio_bufen_param Pointer to GPIO initialization parameters for buffer
 * enable.
 * @param gpio_slp Pointer to GPIO initialization parameters for sleep mode.
 * @param gpio_shtdwn Pointer to GPIO initialization parameters for shutdown
 * mode.
 * @param avdd_v Specifies the AVDD voltage in millivolts.
 * @param refbuf_en Boolean flag to enable or disable the reference buffer.
 * @param gain Specifies the gain value using the ada4250_gain enumeration.
 * @param bias Specifies the bias setting using the ada4250_bias enumeration.
 * @param bandwidth Specifies the bandwidth setting using the ada4250_bandwidth
 * enumeration.
 * @param offset_nv Specifies the offset calibration value in nanovolts.
 ******************************************************************************/
struct ada4250_init_param {
	/* Device ID */
	enum ada4250_id device_id;
	/* SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/* GPIO Initialization parameters */
	struct no_os_gpio_init_param	*gpio_g2_param;
	struct no_os_gpio_init_param	*gpio_g1_param;
	struct no_os_gpio_init_param	*gpio_g0_param;
	struct no_os_gpio_init_param	*gpio_bw_param;
	struct no_os_gpio_init_param	*gpio_bufen_param;
	struct no_os_gpio_init_param	*gpio_slp;
	struct no_os_gpio_init_param	*gpio_shtdwn;
	/* AVDD value in milliVolts */
	int32_t avdd_v;
	/* Reference Buffer Enable */
	bool refbuf_en;
	/* Gain Value */
	enum ada4250_gain gain;
	/* Bias Set */
	enum ada4250_bias bias;
	/* Bandwidth Value */
	enum ada4250_bandwidth bandwidth;
	/* Offset Calibration Value */
	int32_t offset_nv;
};

/***************************************************************************//**
 * @brief The `ada4250_dev` structure is a comprehensive descriptor for the
 * ADA4250 device, encapsulating all necessary parameters for its
 * operation and configuration. It includes identifiers for the device
 * and its communication interfaces, such as SPI and GPIO descriptors,
 * which are essential for hardware interaction. The structure also holds
 * configuration settings like gain, offset range, bias, bandwidth, and
 * power mode, allowing for precise control over the device's
 * performance. Additionally, it includes voltage and calibration
 * settings to ensure accurate operation within specified parameters.
 *
 * @param device_id Specifies the device ID of the ADA4250.
 * @param spi_desc Holds the SPI descriptor for communication.
 * @param gpio_g2 Descriptor for GPIO pin G2.
 * @param gpio_g1 Descriptor for GPIO pin G1.
 * @param gpio_g0 Descriptor for GPIO pin G0.
 * @param gpio_bw Descriptor for GPIO pin controlling bandwidth.
 * @param gpio_bufen Descriptor for GPIO pin enabling the buffer.
 * @param gpio_slp Descriptor for GPIO pin controlling sleep mode.
 * @param gpio_shtdwn Descriptor for GPIO pin controlling shutdown mode.
 * @param avdd_v Specifies the AVDD voltage in millivolts.
 * @param refbuf_en Indicates if the reference buffer is enabled.
 * @param gain Specifies the gain setting of the device.
 * @param offset_range Specifies the offset range setting.
 * @param bias Specifies the bias setting of the device.
 * @param bandwidth Specifies the bandwidth setting of the device.
 * @param power_mode Specifies the power mode of the device.
 * @param offset_nv Specifies the offset calibration value in nanovolts.
 ******************************************************************************/
struct ada4250_dev {
	/* Device ID */
	enum ada4250_id device_id;
	/* SPI Initialization parameters */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO Descriptors */
	struct no_os_gpio_desc	*gpio_g2;
	struct no_os_gpio_desc	*gpio_g1;
	struct no_os_gpio_desc	*gpio_g0;
	struct no_os_gpio_desc	*gpio_bw;
	struct no_os_gpio_desc	*gpio_bufen;
	struct no_os_gpio_desc	*gpio_slp;
	struct no_os_gpio_desc	*gpio_shtdwn;
	/* AVDD value in milliVolts */
	int32_t avdd_v;
	/* Reference Buffer Enable */
	bool refbuf_en;
	/* Gain Value */
	enum ada4250_gain gain;
	/* Offset Range */
	enum ada4250_offset_range offset_range;
	/* Bias Set */
	enum ada4250_bias bias;
	/* Bandwidth Value */
	enum ada4250_bandwidth bandwidth;
	/* Power Mode */
	enum ada4250_power_mode power_mode;
	/* Offset Calibration Value in nV*/
	int32_t offset_nv;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * of the ADA4250 device via SPI communication. It should be called when
 * there is a need to configure or modify the settings of the ADA4250 by
 * writing to its registers. The function requires a valid device
 * descriptor and will return an error if the device ID does not match
 * the expected ADA4250 ID. It is important to ensure that the device has
 * been properly initialized before calling this function.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * This must not be null and must be properly initialized with a
 * valid device ID of ADA4250.
 * @param reg_addr The address of the register to which the data will be
 * written. It is an 8-bit unsigned integer representing a valid
 * register address within the ADA4250.
 * @param data The data byte to be written to the specified register. It is an
 * 8-bit unsigned integer.
 * @return Returns an int32_t value. A negative error code is returned if the
 * device ID is invalid or if the SPI write operation fails. Otherwise,
 * it returns 0 on success.
 ******************************************************************************/
int32_t ada4250_write(struct ada4250_dev *dev, uint8_t reg_addr,
		      uint8_t data);

/***************************************************************************//**
 * @brief This function reads a byte of data from a specified register of the
 * ADA4250 device using SPI communication. It should be called when you
 * need to retrieve the current value of a register from the device. The
 * function requires a valid device descriptor and a register address to
 * read from. It is important to ensure that the device ID in the
 * descriptor matches the ADA4250 before calling this function. If the
 * device ID is incorrect, the function will return an error. The
 * function writes the read data into the provided data pointer.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and must have a valid device_id set to ADA4250.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the ADA4250.
 * @param data A pointer to a uint8_t where the read data will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the device ID is
 * incorrect or if the SPI read operation fails.
 ******************************************************************************/
int32_t ada4250_read(struct ada4250_dev *dev, uint8_t reg_addr,
		     uint8_t *data);

/* ADA4250 Register Update */
/***************************************************************************//**
 * @brief This function is used to update a specific register of the ADA4250
 * device by applying a mask to the current register value and then
 * writing new data. It should be called when a specific bit or bits in a
 * register need to be modified without affecting other bits. The
 * function requires that the device is correctly initialized and that
 * the device ID matches ADA4250. If the device ID is incorrect, the
 * function returns an error. It reads the current value of the register,
 * applies the mask, and writes the modified value back to the register.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and must have a valid device_id set to ADA4250.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADA4250 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected by the data
 * parameter.
 * @param data The new data to be written to the register, masked by the mask
 * parameter. Only bits corresponding to 1s in the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code on failure, such as if
 * the device ID is incorrect or if a read/write operation fails.
 ******************************************************************************/
int32_t ada4250_update(struct ada4250_dev *dev, uint8_t reg_addr,
		       uint8_t mask, uint8_t data);

/* ADA4250 Device Descriptor Update */
/***************************************************************************//**
 * @brief This function reads specific registers from the ADA4250 device and
 * updates the corresponding fields in the device descriptor structure.
 * It should be called when there is a need to synchronize the software
 * representation of the device state with the actual hardware state. The
 * function assumes that the device has been properly initialized and is
 * ready for communication. It handles errors by returning a non-zero
 * value if any register read operation fails.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * This must not be null, and the structure should be properly
 * initialized before calling this function. The function will update
 * the fields within this structure based on the current register
 * values of the device.
 * @return Returns 0 on success, or a negative error code if a register read
 * operation fails.
 ******************************************************************************/
int32_t ada4250_update_desc(struct ada4250_dev *dev);

/* Software Reset */
/***************************************************************************//**
 * @brief This function is used to perform a software reset on an ADA4250
 * device, which is necessary to reinitialize the device to its default
 * state. It should be called when a reset of the device is required,
 * such as after a configuration change or to recover from an error
 * state. The function checks if the device ID matches the expected
 * ADA4250 ID before proceeding. It attempts to reset the device by
 * writing to the reset register and waits for the reset to complete. If
 * the reset does not complete within a predefined timeout, an error is
 * returned. This function must be called with a valid device descriptor
 * that has been properly initialized.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and must be initialized with the correct device
 * ID for the ADA4250. If the device ID is incorrect, the function
 * returns an error.
 * @return Returns 0 on success, -ENODEV if the device ID is incorrect, or
 * -ETIME if the reset operation times out. Other negative error codes
 * may be returned if reading or updating the device registers fails.
 ******************************************************************************/
int32_t ada4250_soft_reset(struct ada4250_dev *dev);

/* Set Reference Buffer */
/***************************************************************************//**
 * @brief This function is used to enable or disable the reference buffer on an
 * ADA4250 device. It should be called when you need to change the
 * reference buffer state, typically during device configuration or
 * reconfiguration. The function requires a valid device descriptor and
 * will return an error if the device ID is not recognized. It updates
 * the device's internal state to reflect the new reference buffer
 * setting.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and should be properly initialized with a valid
 * device ID.
 * @param refbuf A boolean value indicating whether to enable (true) or disable
 * (false) the reference buffer.
 * @return Returns 0 on success, a negative error code if the device ID is
 * unrecognized, or if an error occurs during the update process.
 ******************************************************************************/
int32_t ada4250_en_refbuf(struct ada4250_dev *dev, bool refbuf);

/* Set Current Bias */
/***************************************************************************//**
 * @brief Use this function to configure the current bias setting of an ADA4250
 * device. It must be called with a valid device descriptor that has been
 * initialized and verified to be of type ADA4250. The function updates
 * the device's bias setting and returns an error code if the device ID
 * is incorrect or if the update operation fails. This function is
 * essential for adjusting the bias to match specific application
 * requirements.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and must have a device_id of ADA4250. The caller
 * retains ownership.
 * @param bias An enum ada4250_bias value representing the desired bias setting.
 * Valid values are ADA4250_BIAS_DISABLE, ADA4250_BIAS_BANDGAP_REF,
 * and ADA4250_BIAS_AVDD.
 * @return Returns 0 on success. Returns -ENODEV if the device ID is not
 * ADA4250, or a negative error code if the update operation fails.
 ******************************************************************************/
int32_t ada4250_set_bias(struct ada4250_dev *dev, enum ada4250_bias bias);

/* Set gain */
/***************************************************************************//**
 * @brief This function configures the gain setting of an ADA4250 device. It
 * should be called when a change in the gain setting is required. The
 * function requires a valid device descriptor and a gain value from the
 * predefined gain enumeration. It updates the device's gain setting and
 * returns an error code if the operation fails. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null. The device should be initialized before use.
 * @param gain An enum ada4250_gain value representing the desired gain setting.
 * Must be a valid value from the ada4250_gain enumeration.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ada4250_set_gain(struct ada4250_dev *dev, enum ada4250_gain gain);

/* Set offset value */
/***************************************************************************//**
 * @brief This function configures the offset for the ADA4250 device, which is
 * essential for calibrating the sensor's output. It must be called with
 * a valid device descriptor and a desired offset value. The function
 * checks if the device is correctly identified as ADA4250 and if the
 * bias is enabled. It calculates the appropriate offset range and raw
 * offset value based on the current gain setting. If the offset is
 * within the permissible range, it updates the device's offset
 * configuration. This function should be used after initializing the
 * device and setting the gain and bias parameters.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null and must be initialized with a valid ADA4250
 * device.
 * @param offset The desired offset value in nanovolts. It should be within the
 * range supported by the current gain setting of the device. If
 * the offset is out of range, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the device is not
 * recognized, the bias is disabled, or the offset is out of range.
 ******************************************************************************/
int32_t ada4250_set_offset(struct ada4250_dev *dev, int64_t offset);

/* Set bandwidth mode */
/***************************************************************************//**
 * @brief This function configures the bandwidth mode of the ADA4250 device to
 * either low or high bandwidth. It should be called when a change in the
 * bandwidth setting is required. The function requires a valid device
 * descriptor and a specified bandwidth mode. If the device descriptor is
 * null, the function returns an error. The function also returns an
 * error if an invalid bandwidth mode is provided. Upon successful
 * execution, the device's bandwidth setting is updated accordingly.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param bw An enum ada4250_bandwidth value specifying the desired bandwidth
 * mode. Valid values are ADA4250_BANDWIDTH_LOW and
 * ADA4250_BANDWIDTH_HIGH. Invalid values result in an error.
 * @return Returns 0 on success. Returns a negative error code if the device
 * descriptor is null, if the bandwidth mode is invalid, or if there is
 * a failure in setting the GPIO value.
 ******************************************************************************/
int32_t ada4250_set_bandwidth(struct ada4250_dev *dev,
			      enum ada4250_bandwidth bw);

/* Set sleep/shutdown mode */
/***************************************************************************//**
 * @brief Use this function to change the power mode of the ADA4250 device to
 * either sleep or shutdown. This function should be called when you need
 * to reduce power consumption by putting the device into a low-power
 * state. Ensure that the device has been properly initialized before
 * calling this function. The function will return an error if the device
 * pointer is null or if an invalid power mode is specified. It updates
 * the device's internal state to reflect the new power mode.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param pwrmode An enum value of type ada4250_power_mode specifying the
 * desired power mode. Valid values are ADA4250_POWER_SLEEP and
 * ADA4250_POWER_SHUTDOWN. Invalid values result in an error.
 * @return Returns 0 on success, -ENOMEM if the device pointer is null, -EINVAL
 * if the power mode is invalid, or a negative error code if setting the
 * GPIO value fails.
 ******************************************************************************/
int32_t ada4250_set_slp_shtdwn_mode(struct ada4250_dev *dev,
				    enum ada4250_power_mode pwrmode);

/* Set normal mode */
/***************************************************************************//**
 * @brief This function transitions the ADA4250 device from either sleep or
 * shutdown mode to normal power mode. It should be called when the
 * device needs to be activated from a low-power state. The function
 * requires a valid device descriptor and handles the necessary GPIO
 * operations to change the power state. If the device was in shutdown
 * mode, it can optionally reconfigure the device to its previous state
 * or update the driver configurations based on the `reconfig` parameter.
 * The function returns an error code if the device descriptor is null or
 * if any GPIO operation fails.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device.
 * Must not be null. The function will return an error if this
 * parameter is null.
 * @param reconfig A boolean indicating whether to reconfigure the device to its
 * previous state after waking from shutdown mode. If true, the
 * device is reconfigured; if false, the driver configurations
 * are updated.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor is null or if a GPIO operation fails.
 ******************************************************************************/
int32_t ada4250_set_normal_mode(struct ada4250_dev *dev, bool reconfig);

/* ADA4250 Initialization */
/***************************************************************************//**
 * @brief This function initializes an ADA4250 device using the provided
 * initialization parameters. It sets up the device's configuration,
 * including GPIO and SPI interfaces, and verifies the device's identity.
 * This function must be called before any other operations on the
 * ADA4250 device. It handles both ADA4250 and ADA4230 device types,
 * configuring the appropriate interfaces and settings based on the
 * device ID. The function returns an error code if initialization fails
 * at any step, such as memory allocation failure, GPIO setup failure, or
 * incorrect device ID verification.
 *
 * @param device A pointer to a pointer of type `struct ada4250_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure upon successful initialization.
 * @param init_param A pointer to a `struct ada4250_init_param` containing the
 * initialization parameters for the device. This includes
 * device ID, SPI and GPIO initialization parameters, AVDD
 * voltage, reference buffer enable flag, gain, bias,
 * bandwidth, and offset calibration value. The caller retains
 * ownership of this structure, and it must not be null.
 * @return Returns an `int32_t` error code: 0 on success, or a negative error
 * code on failure, indicating the type of error encountered during
 * initialization.
 ******************************************************************************/
int32_t ada4250_init(struct ada4250_dev **device,
		     struct ada4250_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADA4250 device when it is no longer needed. This function should be
 * called to clean up after using the device to prevent resource leaks.
 * It handles the deallocation of GPIO and SPI resources based on the
 * device configuration. Ensure that the device pointer is valid and
 * initialized before calling this function.
 *
 * @param dev A pointer to an ada4250_dev structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns 0 on successful deallocation of all resources. If any
 * resource deallocation fails, a negative error code is returned
 * indicating the failure.
 ******************************************************************************/
int32_t ada4250_remove(struct ada4250_dev *dev);

#endif /* ADA4250_H_ */

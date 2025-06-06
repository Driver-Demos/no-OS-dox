/***************************************************************************//**
 *   @file   ad7799.h
 *   @brief  Header file of AD7798/AD7799 Driver.
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
#ifndef AD7799_H_
#define AD7799_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Types Declarations *************************/
/******************************************************************************/

/*AD7799 Registers*/
#define AD7799_REG_COMM		0x0 /* Communications Register(WO, 8-bit) */
#define AD7799_REG_STAT	    	0x0 /* Status Register (RO, 8-bit) */
#define AD7799_REG_MODE	    	0x1 /* Mode Register (RW, 16-bit) */
#define AD7799_REG_CONF	    	0x2 /* Configuration Register (RW, 16-bit)*/
#define AD7799_REG_DATA	    	0x3 /* Data Register (RO, 16-/24-bit) */
#define AD7799_REG_ID	    	0x4 /* ID Register (RO, 8-bit) */
#define AD7799_REG_IO	    	0x5 /* IO Register (RO, 8-bit) */
#define AD7799_REG_OFFSET   	0x6 /* Offset Register (RW, 24-bit) */
#define AD7799_REG_FULLSCALE	0x7 /* Full-Scale Register (RW, 24-bit) */

/* AD7799 Polarity */
#define AD7799_BIPOLAR 		0x0 /* Bipolar bit */
#define AD7799_UNIPOLAR		0x1 /* Unipolar bit */

/* Communications Register Bit Designations (AD7799_REG_COMM) */
#define AD7799_COMM_WEN		0x80 /* Write Enable */
#define AD7799_COMM_WRITE	0x00 /* Write Operation */
#define AD7799_COMM_READ    	0x40 /* Read Operation */
#define AD7799_COMM_ADDR(x)	(((x) & 0x7) << 3) /* Register Address */
#define AD7799_COMM_CREAD	0x04 /* Continuous Read */

/* Status Register Bit Designations (AD7799_REG_STAT) */
#define AD7799_STAT_RDY		0x80 /* Ready */
#define AD7799_STAT_ERR		0x40 /* Error (Overrange, Underrange) */
#define AD7799_STAT_CH3		0x04 /* Channel 3 */
#define AD7799_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7799_STAT_CH1		(1 << 0) /* Channel 1 */

/* Mode Register Bit Designations (AD7799_REG_MODE) */
#define AD7799_MODE_SEL(x)	(((x) & 0x7) << 13) /* Operation Mode Select */
#define AD7799_MODE_PSW(x)	0x1000 /* Power Switch Control Bit */
#define AD7799_MODE_RATE(x)	((x) & 0xF) /* Filter Update Rate */

/* AD7799_MODE_SEL(x) options */
#define AD7799_MODE_CONT	 0x0 /* Continuous Conversion Mode */
#define AD7799_MODE_SINGLE	 0x1 /* Single Conversion Mode */
#define AD7799_MODE_IDLE	 0x2 /* Idle Mode */
#define AD7799_MODE_PWRDN	 0x3 /* Power-Down Mode */
#define AD7799_MODE_CAL_INT_ZERO 0x4 /* Internal Zero-Scale Calibration */
#define AD7799_MODE_CAL_INT_FULL 0x5 /* Internal Full-Scale Calibration */
#define AD7799_MODE_CAL_SYS_ZERO 0x6 /* System Zero-Scale Calibration */
#define AD7799_MODE_CAL_SYS_FULL 0x7 /* System Full-Scale Calibration */

/* Configuration Register Bit Designations (AD7799_REG_CONF) */
#define AD7799_CONF_BO_EN	 0x2000 /* Burnout Current */
#define AD7799_CONF_POLARITY(x)  (((x) & 0x1) << 12) /* Unipolar/Bipolar */
#define AD7799_CONF_GAIN(x)	 (((x) & 0x7) << 8) /* Gain Select */
#define AD7799_CONF_REFDET(x)    (((x) & 0x1) << 5) /* Reference detect */
#define AD7799_CONF_BUF		 0x10 /* Buffered Mode Enable */
#define AD7799_CONF_CHAN(x)	 ((x) & 0x7) /* Channel select */

/* AD7799_CONF_GAIN(x) options */
#define AD7799_GAIN_1       	 0x0
#define AD7799_GAIN_2       	 0x1
#define AD7799_GAIN_4       	 0x2
#define AD7799_GAIN_8       	 0x3
#define AD7799_GAIN_16      	 0x4
#define AD7799_GAIN_32      	 0x5
#define AD7799_GAIN_64      	 0x6
#define AD7799_GAIN_128     	 0x7

/* AD7799 Register size */
#define AD7799_REG_SIZE_1B	 0x1
#define AD7799_REG_SIZE_2B	 0x2
#define AD7799_REG_SIZE_3B	 0x3

/* AD7799_CONF_REFDET(x) options */
#define AD7799_REFDET_ENA   	 0x1
#define AD7799_REFDET_DIS   	 0x0

/* AD7799_CONF_CHAN(x) options */
#define AD7799_CH_AIN1P_AIN1M	 0x0 /* AIN1(+) - AIN1(-) */
#define AD7799_CH_AIN2P_AIN2M	 0x1 /* AIN2(+) - AIN2(-) */
#define AD7799_CH_AIN3P_AIN3M	 0x2 /* AIN3(+) - AIN3(-) */
#define AD7799_CH_AIN1M_AIN1M	 0x3 /* AIN1(-) - AIN1(-) */
#define AD7799_CH_AVDD_MONITOR	 0x7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7799_REG_ID) */
#define AD7799_ID_MASK		 0xF

/* AD7799 Configuration Mask */
#define AD7799_REG_MASK		 0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7799_REG_IO) */
#define AD7799_IOEN		 0x40
#define AD7799_IO1(x)		 (((x) & 0x1) << 4)
#define AD7799_IO2(x)		 (((x) & 0x1) << 5)

/* AD7799 Timeout */
#define AD7799_TIMEOUT		 0xFFFF

/* AD7799 Reset Sequence */
#define AD7799_RESET_DATA	 0xFF

/***************************************************************************//**
 * @brief The `ad7799_type` enumeration defines constants for identifying the
 * type of device, specifically distinguishing between the AD7798 and
 * AD7799 devices. Each enumerator is associated with a unique
 * hexadecimal value, allowing for easy identification and
 * differentiation of the device types within the driver code.
 *
 * @param ID_AD7798 Represents the AD7798 device with a value of 0x8.
 * @param ID_AD7799 Represents the AD7799 device with a value of 0x9.
 ******************************************************************************/
enum ad7799_type {
	/** AD7798 device*/
	ID_AD7798 = 0x8,
	/** AD7799 device */
	ID_AD7799 = 0x9
};

/***************************************************************************//**
 * @brief The `ad7799_precision` enumeration defines the precision levels for
 * the ADC channels of the AD7799 device, allowing the user to specify
 * whether the precision should be in millivolts (mV) or microvolts (uV).
 * This is crucial for applications requiring different levels of
 * measurement accuracy and resolution.
 *
 * @param AD7799_PRECISION_MV Represents ADC channel precision in millivolts.
 * @param AD7799_PRECISION_UV Represents ADC channel precision in microvolts.
 ******************************************************************************/
enum ad7799_precision {
	/** ADC channel precision in mV */
	AD7799_PRECISION_MV,
	/** ADC channel precision in uV */
	AD7799_PRECISION_UV,
};

/***************************************************************************//**
 * @brief The `ad7799_dev` structure is a comprehensive representation of an
 * AD7798/AD7799 device, encapsulating all necessary parameters for its
 * operation. It includes a pointer to an SPI descriptor for
 * communication, the chip type to distinguish between AD7798 and AD7799,
 * and a pointer to the register size configuration. The structure also
 * holds the gain setting, polarity mode (unipolar or bipolar), reference
 * voltage in millivolts, and the precision of the ADC channel, which can
 * be in millivolts or microvolts. This structure is essential for
 * managing the configuration and operation of the AD7798/AD7799 ADC
 * devices.
 *
 * @param spi_desc Pointer to the SPI descriptor used for communication.
 * @param chip_type Indicates the type of chip, either AD7798 or AD7799.
 * @param reg_size Pointer to the register size configuration.
 * @param gain Specifies the gain setting for the ADC.
 * @param polarity Determines if the ADC operates in unipolar or bipolar mode.
 * @param vref_mv Reference voltage in millivolts used by the ADC.
 * @param precision Specifies the ADC channel precision, either in mV or uV.
 ******************************************************************************/
struct ad7799_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc *spi_desc;
	/** Chip type (AD7798/AD7799) */
	uint8_t chip_type;
	/** Register size */
	const uint8_t *reg_size;
	/** Gain */
	uint8_t gain;
	/** Unipolar/Bipolar Coding */
	bool polarity;
	/** Reference Voltage in mV */
	uint32_t vref_mv;
	/** ADC channel precision (mV/uV) **/
	enum ad7799_precision precision;
};

/***************************************************************************//**
 * @brief The `ad7799_init_param` structure is used to define the initialization
 * parameters for the AD7798/AD7799 devices. It includes settings for SPI
 * communication, chip type selection, gain configuration, polarity mode,
 * reference voltage, and ADC precision. This structure is essential for
 * configuring the ADC device before use, ensuring that it operates with
 * the desired parameters for specific applications.
 *
 * @param spi_init SPI Initialization parameters.
 * @param chip_type Specifies the chip type, either AD7798 or AD7799.
 * @param gain Defines the gain setting for the ADC.
 * @param polarity Indicates whether the ADC operates in unipolar or bipolar
 * mode.
 * @param vref_mv Specifies the reference voltage in millivolts.
 * @param precision Determines the ADC channel precision, either in millivolts
 * or microvolts.
 ******************************************************************************/
struct ad7799_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param spi_init;
	/** Chip type (AD7798/AD7799) */
	enum ad7799_type chip_type;
	/** Gain */
	uint8_t gain;
	/** Unipolar/Bipolar Coding */
	bool polarity;
	/** Reference Voltage in mV */
	uint32_t vref_mv;
	/** ADC channel precision (mV/uV) **/
	enum ad7799_precision precision;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief This function reads data from a specified register of the AD7799
 * device and stores the result in the provided output parameter. It is
 * essential to ensure that the device is properly initialized and
 * configured before calling this function. The function communicates
 * with the device over SPI, and the caller must ensure that the SPI
 * descriptor within the device structure is valid. The function will
 * return an error code if the SPI communication fails.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null, and the SPI descriptor within
 * must be valid.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address as defined by the device.
 * @param reg_data A pointer to a uint32_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success or -1 if an error occurs during SPI
 * communication. The read data is stored in the location pointed to by
 * reg_data.
 ******************************************************************************/
int32_t ad7799_read(struct ad7799_dev *device, uint8_t reg_addr,
		    uint32_t *reg_data);

/* Write device register */
/***************************************************************************//**
 * @brief This function writes a 32-bit data value to a specified register of
 * the AD7799 device. It is used to configure or control the device by
 * updating its registers. The function requires a valid device structure
 * that has been initialized and a valid register address. The register
 * address must correspond to one of the defined registers in the AD7799
 * device. The function handles the data transfer over SPI and returns an
 * error code if the write operation fails. It is important to ensure
 * that the device is ready and properly initialized before calling this
 * function.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address defined for the AD7799.
 * @param reg_data The 32-bit data to write to the specified register. The data
 * will be truncated or padded based on the register size.
 * @return Returns 0 on success or -1 if the SPI write operation fails.
 ******************************************************************************/
int32_t ad7799_write(struct ad7799_dev *device, uint8_t reg_addr,
		     uint32_t reg_data);

/* Software reset of the device. */
/***************************************************************************//**
 * @brief Use this function to reset the AD7799 device to its default state by
 * sending a reset command via SPI. This is typically done to ensure the
 * device is in a known state before configuration or operation. The
 * function should be called when the device is not actively being used
 * for conversions, as it will interrupt any ongoing operations. Ensure
 * that the device structure is properly initialized and the SPI
 * descriptor is valid before calling this function.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device to reset. Must not be null. The function assumes the
 * SPI descriptor within this structure is valid and properly
 * configured.
 * @return Returns an int32_t value indicating the success or failure of the
 * reset operation. A non-negative value typically indicates success,
 * while a negative value indicates an error occurred during the SPI
 * communication.
 ******************************************************************************/
int32_t ad7799_reset(struct ad7799_dev *device);

/* Set the device mode. */
/***************************************************************************//**
 * @brief Use this function to configure the AD7799 device to operate in a
 * specific mode, such as continuous conversion or power-down. This
 * function must be called after the device has been initialized. It
 * reads the current mode register, modifies it to set the desired mode,
 * and writes it back to the device. The function ensures the device is
 * ready after setting the mode. If any operation fails, it returns an
 * error code.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param mode A uint8_t value representing the desired mode. Valid values are
 * defined by AD7799_MODE_SEL(x) options, such as AD7799_MODE_CONT
 * for continuous conversion mode.
 * @return Returns 0 on success, or -1 if an error occurs during reading,
 * writing, or checking device readiness.
 ******************************************************************************/
int32_t ad7799_set_mode(struct ad7799_dev *device, uint8_t mode);

/* Select the ADC channel. */
/***************************************************************************//**
 * @brief This function is used to select the active ADC channel on an AD7799
 * device. It must be called with a valid device structure and a channel
 * identifier to configure the device to read from the specified channel.
 * The function reads the current configuration, modifies the channel
 * selection bits, and writes the updated configuration back to the
 * device. It is important to ensure that the device is properly
 * initialized before calling this function. The function returns an
 * error code if the read or write operation fails.
 *
 * @param device A pointer to an ad7799_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch An 8-bit unsigned integer representing the channel to select. Valid
 * values are typically defined by the device's channel configuration
 * options. Invalid values may result in undefined behavior.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7799_set_channel(struct ad7799_dev *device, uint8_t ch);

/* Read specific ADC channel. */
/***************************************************************************//**
 * @brief This function reads data from a specified channel of the AD7799
 * device. It should be called when you need to obtain the current data
 * from a specific channel. Before calling this function, ensure that the
 * device has been properly initialized and is ready for communication.
 * The function sets the device to single conversion mode, waits for the
 * device to be ready, and then reads the data from the specified
 * channel. If any step fails, the function returns an error code.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param ch The channel number to read from. Valid channel numbers depend on
 * the specific device configuration.
 * @param reg_data A pointer to a uint32_t where the read data will be stored.
 * Must not be null.
 * @return Returns 0 on success, or -1 if an error occurs during channel
 * selection, mode setting, device readiness check, or data reading.
 ******************************************************************************/
int32_t ad7799_get_channel(struct ad7799_dev *device, uint8_t ch,
			   uint32_t *reg_data);

/* Read specific ADC channel data with the specified precision. */
/***************************************************************************//**
 * @brief Use this function to read data from a specific ADC channel of the
 * AD7798/AD7799 device and scale it according to the device's reference
 * voltage and precision settings. This function should be called when
 * you need to obtain a scaled measurement from the ADC. Ensure that the
 * device is properly initialized and configured before calling this
 * function. The function handles both unipolar and bipolar
 * configurations, scaling the data accordingly. It returns an error code
 * if the channel reading fails.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the ADC device. Must not be null.
 * @param ch The channel number to read from. Valid values depend on the device
 * configuration and should correspond to a configured channel.
 * @param data_scaled A pointer to an int32_t where the scaled data will be
 * stored. Must not be null.
 * @return Returns an int32_t error code: 0 on success, or a negative error code
 * if the channel reading fails. The scaled data is written to the
 * location pointed to by data_scaled.
 ******************************************************************************/
int32_t ad7799_read_channel(struct ad7799_dev *device, uint8_t ch,
			    int32_t *data_scaled);

/* Set the ADC gain. */
/***************************************************************************//**
 * @brief Use this function to configure the gain setting of the AD7799 ADC
 * device. It must be called with a valid device structure and a gain
 * value that corresponds to the desired amplification level. This
 * function reads the current configuration, modifies the gain bits, and
 * writes the updated configuration back to the device. It should be
 * called only after the device has been properly initialized. If the
 * read or write operation fails, the function returns an error code.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param gain A uint8_t value representing the desired gain setting. Valid
 * values are from 0 to 7, corresponding to gain levels of 1, 2, 4,
 * 8, 16, 32, 64, and 128 respectively. Invalid values may result in
 * undefined behavior.
 * @return Returns 0 on success or -1 if an error occurs during the read or
 * write operation.
 ******************************************************************************/
int32_t ad7799_set_gain(struct ad7799_dev *device, uint8_t gain);

/* Get the ADC gain. */
/***************************************************************************//**
 * @brief Use this function to obtain the current gain configuration of an
 * AD7799 device. It is essential to call this function after the device
 * has been properly initialized and configured. The function reads the
 * configuration register of the device to extract the gain setting,
 * which is then provided to the caller through the output parameter.
 * This function is useful for verifying the current gain setting or for
 * debugging purposes. Ensure that the device pointer is valid and that
 * the gain pointer is not null before calling this function.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param gain A pointer to a uint8_t where the current gain setting will be
 * stored. Must not be null.
 * @return Returns 0 on success, with the gain value stored in the provided
 * pointer. Returns -1 if an error occurs during the read operation.
 ******************************************************************************/
int32_t ad7799_get_gain(struct ad7799_dev *device, uint8_t *gain);

/* Enable or disable the reference detect function. */
/***************************************************************************//**
 * @brief Use this function to control the reference detect feature of the
 * AD7799 device, which can be enabled or disabled based on the provided
 * parameter. This function should be called when you need to change the
 * reference detection state, typically during device configuration or
 * calibration. Ensure that the device is properly initialized before
 * calling this function. The function reads the current configuration,
 * modifies the reference detect bit, and writes the updated
 * configuration back to the device. If the read or write operation
 * fails, the function returns an error code.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param ref_en A uint8_t value indicating whether to enable (1) or disable (0)
 * the reference detect function. Values outside this range may
 * lead to undefined behavior.
 * @return Returns 0 on success or -1 if an error occurs during the read or
 * write operation.
 ******************************************************************************/
int32_t ad7799_set_refdet(struct ad7799_dev *device, uint8_t ref_en);

/* Set ADC polarity. */
/***************************************************************************//**
 * @brief This function configures the polarity mode of the AD7799 device,
 * allowing the user to select between unipolar and bipolar operation. It
 * should be called when the device is initialized and before starting
 * any conversions that depend on the polarity setting. The function
 * reads the current configuration, modifies the polarity bits, and
 * writes the updated configuration back to the device. It is important
 * to ensure that the device pointer is valid and properly initialized
 * before calling this function. If the read or write operation fails,
 * the function returns an error code.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null.
 * @param polarity A uint8_t value representing the desired polarity mode. Valid
 * values are AD7799_BIPOLAR for bipolar mode and
 * AD7799_UNIPOLAR for unipolar mode. Invalid values may result
 * in undefined behavior.
 * @return Returns 0 on success or -1 if an error occurs during the read or
 * write operation.
 ******************************************************************************/
int32_t ad7799_set_polarity(struct ad7799_dev *device, uint8_t polarity);

/* Check the status of the device. */
/***************************************************************************//**
 * @brief This function checks the readiness of the AD7799 device by polling its
 * status register. It should be called to ensure the device is ready
 * before attempting data operations. The function will return
 * immediately if the device is ready, or after a timeout period if it is
 * not. It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param device A pointer to an initialized ad7799_dev structure representing
 * the device. Must not be null. The function will return an error
 * if the device is not properly initialized or if communication
 * fails.
 * @return Returns 0 if the device is ready, or -1 if the device is not ready
 * after the timeout period or if an error occurs during communication.
 ******************************************************************************/
int32_t ad7799_dev_ready(struct ad7799_dev *device);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes an AD7798 or AD7799 device using the
 * provided initialization parameters. It allocates memory for the device
 * structure, configures the SPI interface, and sets initial device
 * parameters such as gain and polarity. The function must be called
 * before any other operations on the device. If initialization fails at
 * any step, the function returns an error code and the device pointer is
 * not valid. Ensure that the `init_param` structure is correctly
 * populated with valid values before calling this function.
 *
 * @param device A pointer to a pointer of type `struct ad7799_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad7799_init_param` containing the
 * initialization parameters for the device. Must not be null
 * and must be correctly populated with valid values for chip
 * type, gain, polarity, reference voltage, and precision.
 * @return Returns 0 on successful initialization. Returns a negative error code
 * if initialization fails, in which case the device pointer is not
 * valid.
 ******************************************************************************/
int32_t ad7799_init(struct ad7799_dev **device,
		    const struct ad7799_init_param *init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief Use this function to properly shut down and clean up resources
 * associated with an AD7798/AD7799 device. It should be called when the
 * device is no longer needed, ensuring that any allocated resources are
 * freed and the SPI interface is properly closed. This function must be
 * called after the device has been initialized and used, to prevent
 * resource leaks.
 *
 * @param device A pointer to an `ad7799_dev` structure representing the device
 * to be removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-zero value indicates an error occurred during the
 * removal process.
 ******************************************************************************/
int32_t ad7799_remove(struct ad7799_dev *device);

#endif /* AD7799_H_ */

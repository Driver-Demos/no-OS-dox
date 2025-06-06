/***************************************************************************//**
 *   @file   ADXL362.h
 *   @brief  Header file of ADXL362 Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
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

#ifndef __ADXL362_H__
#define __ADXL362_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************************* ADXL362 ************************************/
/******************************************************************************/

/* ADXL362 communication commands */
#define ADXL362_WRITE_REG               0x0A
#define ADXL362_READ_REG                0x0B
#define ADXL362_WRITE_FIFO              0x0D

/* Registers */
#define ADXL362_REG_DEVID_AD            0x00
#define ADXL362_REG_DEVID_MST           0x01
#define ADXL362_REG_PARTID              0x02
#define ADXL362_REG_REVID               0x03
#define ADXL362_REG_XDATA               0x08
#define ADXL362_REG_YDATA               0x09
#define ADXL362_REG_ZDATA               0x0A
#define ADXL362_REG_STATUS              0x0B
#define ADXL362_REG_FIFO_L              0x0C
#define ADXL362_REG_FIFO_H              0x0D
#define ADXL362_REG_XDATA_L             0x0E
#define ADXL362_REG_XDATA_H             0x0F
#define ADXL362_REG_YDATA_L             0x10
#define ADXL362_REG_YDATA_H             0x11
#define ADXL362_REG_ZDATA_L             0x12
#define ADXL362_REG_ZDATA_H             0x13
#define ADXL362_REG_TEMP_L              0x14
#define ADXL362_REG_TEMP_H              0x15
#define ADXL362_REG_SOFT_RESET          0x1F
#define ADXL362_REG_THRESH_ACT_L        0x20
#define ADXL362_REG_THRESH_ACT_H        0x21
#define ADXL362_REG_TIME_ACT            0x22
#define ADXL362_REG_THRESH_INACT_L      0x23
#define ADXL362_REG_THRESH_INACT_H      0x24
#define ADXL362_REG_TIME_INACT_L        0x25
#define ADXL362_REG_TIME_INACT_H        0x26
#define ADXL362_REG_ACT_INACT_CTL       0x27
#define ADXL362_REG_FIFO_CTL            0x28
#define ADXL362_REG_FIFO_SAMPLES        0x29
#define ADXL362_REG_INTMAP1             0x2A
#define ADXL362_REG_INTMAP2             0x2B
#define ADXL362_REG_FILTER_CTL          0x2C
#define ADXL362_REG_POWER_CTL           0x2D
#define ADXL362_REG_SELF_TEST           0x2E

/* ADXL362_REG_STATUS definitions */
#define ADXL362_STATUS_ERR_USER_REGS    (1 << 7)
#define ADXL362_STATUS_AWAKE            (1 << 6)
#define ADXL362_STATUS_INACT            (1 << 5)
#define ADXL362_STATUS_ACT              (1 << 4)
#define ADXL362_STATUS_FIFO_OVERRUN     (1 << 3)
#define ADXL362_STATUS_FIFO_WATERMARK   (1 << 2)
#define ADXL362_STATUS_FIFO_RDY         (1 << 1)
#define ADXL362_STATUS_DATA_RDY         (1 << 0)

/* ADXL362_REG_ACT_INACT_CTL definitions */
#define ADXL362_ACT_INACT_CTL_LINKLOOP(x)   (((x) & 0x3) << 4)
#define ADXL362_ACT_INACT_CTL_INACT_REF     (1 << 3)
#define ADXL362_ACT_INACT_CTL_INACT_EN      (1 << 2)
#define ADXL362_ACT_INACT_CTL_ACT_REF       (1 << 1)
#define ADXL362_ACT_INACT_CTL_ACT_EN        (1 << 0)

/* ADXL362_ACT_INACT_CTL_LINKLOOP(x) options */
#define ADXL362_MODE_DEFAULT            0
#define ADXL362_MODE_LINK               1
#define ADXL362_MODE_LOOP               3

/* ADXL362_REG_FIFO_CTL */
#define ADXL362_FIFO_CTL_AH             (1 << 3)
#define ADXL362_FIFO_CTL_FIFO_TEMP      (1 << 2)
#define ADXL362_FIFO_CTL_FIFO_MODE(x)   (((x) & 0x3) << 0)

/* ADXL362_FIFO_CTL_FIFO_MODE(x) options */
#define ADXL362_FIFO_DISABLE            0
#define ADXL362_FIFO_OLDEST_SAVED       1
#define ADXL362_FIFO_STREAM             2
#define ADXL362_FIFO_TRIGGERED          3

/* ADXL362_REG_INTMAP1 */
#define ADXL362_INTMAP1_INT_LOW         (1 << 7)
#define ADXL362_INTMAP1_AWAKE           (1 << 6)
#define ADXL362_INTMAP1_INACT           (1 << 5)
#define ADXL362_INTMAP1_ACT             (1 << 4)
#define ADXL362_INTMAP1_FIFO_OVERRUN    (1 << 3)
#define ADXL362_INTMAP1_FIFO_WATERMARK  (1 << 2)
#define ADXL362_INTMAP1_FIFO_READY      (1 << 1)
#define ADXL362_INTMAP1_DATA_READY      (1 << 0)

/* ADXL362_REG_INTMAP2 definitions */
#define ADXL362_INTMAP2_INT_LOW         (1 << 7)
#define ADXL362_INTMAP2_AWAKE           (1 << 6)
#define ADXL362_INTMAP2_INACT           (1 << 5)
#define ADXL362_INTMAP2_ACT             (1 << 4)
#define ADXL362_INTMAP2_FIFO_OVERRUN    (1 << 3)
#define ADXL362_INTMAP2_FIFO_WATERMARK  (1 << 2)
#define ADXL362_INTMAP2_FIFO_READY      (1 << 1)
#define ADXL362_INTMAP2_DATA_READY      (1 << 0)

/* ADXL362_REG_FILTER_CTL definitions */
#define ADXL362_FILTER_CTL_RANGE(x)     (((x) & 0x3) << 6)
#define ADXL362_FILTER_CTL_RES          (1 << 5)
#define ADXL362_FILTER_CTL_HALF_BW      (1 << 4)
#define ADXL362_FILTER_CTL_EXT_SAMPLE   (1 << 3)
#define ADXL362_FILTER_CTL_ODR(x)       (((x) & 0x7) << 0)

/* ADXL362_FILTER_CTL_RANGE(x) options */
#define ADXL362_RANGE_2G                0 /* +/-2 g */
#define ADXL362_RANGE_4G                1 /* +/-4 g */
#define ADXL362_RANGE_8G                2 /* +/-8 g */

/* ADXL362_FILTER_CTL_ODR(x) options */
#define ADXL362_ODR_12_5_HZ             0 /* 12.5 Hz */
#define ADXL362_ODR_25_HZ               1 /* 25 Hz */
#define ADXL362_ODR_50_HZ               2 /* 50 Hz */
#define ADXL362_ODR_100_HZ              3 /* 100 Hz */
#define ADXL362_ODR_200_HZ              4 /* 200 Hz */
#define ADXL362_ODR_400_HZ              5 /* 400 Hz */

/* ADXL362_REG_POWER_CTL definitions */
#define ADXL362_POWER_CTL_RES           (1 << 7)
#define ADXL362_POWER_CTL_EXT_CLK       (1 << 6)
#define ADXL362_POWER_CTL_LOW_NOISE(x)  (((x) & 0x3) << 4)
#define ADXL362_POWER_CTL_WAKEUP        (1 << 3)
#define ADXL362_POWER_CTL_AUTOSLEEP     (1 << 2)
#define ADXL362_POWER_CTL_MEASURE(x)    (((x) & 0x3) << 0)

/* ADXL362_POWER_CTL_LOW_NOISE(x) options */
#define ADXL362_NOISE_MODE_NORMAL       0
#define ADXL362_NOISE_MODE_LOW          1
#define ADXL362_NOISE_MODE_ULTRALOW     2

/* ADXL362_POWER_CTL_MEASURE(x) options */
#define ADXL362_MEASURE_STANDBY         0
#define ADXL362_MEASURE_ON              2

/* ADXL362_REG_SELF_TEST */
#define ADXL362_SELF_TEST_ST            (1 << 0)

/* ADXL362 device information */
#define ADXL362_DEVICE_AD               0xAD
#define ADXL362_DEVICE_MST              0x1D
#define ADXL362_PART_ID                 0xF2

/* ADXL362 Reset settings */
#define ADXL362_RESET_KEY               0x52

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxl362_dev` structure is designed to represent an ADXL362
 * accelerometer device in a software application. It contains a pointer
 * to a SPI descriptor, which is essential for establishing and managing
 * SPI communication with the device, and an 8-bit field to store the
 * selected measurement range, allowing the user to configure the
 * sensitivity of the accelerometer.
 *
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 * @param selected_range An 8-bit unsigned integer representing the measurement
 * range of the device.
 ******************************************************************************/
struct adxl362_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc	*spi_desc;
	/** Measurement Range: */
	uint8_t		selected_range;
};

/***************************************************************************//**
 * @brief The `adxl362_init_param` structure is designed to encapsulate the
 * initialization parameters necessary for setting up an ADXL362 device,
 * specifically focusing on the SPI communication interface. It contains
 * a single member, `spi_init`, which is a structure that provides the
 * necessary configuration details for initializing the SPI interface,
 * ensuring that the device can communicate effectively with the host
 * system.
 *
 * @param spi_init This member is a structure that holds the SPI initialization
 * parameters required for setting up the SPI communication with
 * the ADXL362 device.
 ******************************************************************************/
struct adxl362_init_param {
	/** SPI Initialization structure. */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the ADXL362 accelerometer device for use by
 * initializing the necessary hardware interfaces and verifying the
 * device's identity. It must be called before any other operations on
 * the device to ensure proper configuration. The function allocates
 * memory for the device structure and initializes the SPI communication
 * based on the provided parameters. If the initialization is successful,
 * the device is set to a default measurement range of +/- 2g. The caller
 * is responsible for managing the memory of the device structure, which
 * should be freed using the appropriate function when no longer needed.
 *
 * @param device A pointer to a pointer of type `struct adxl362_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adxl362_init_param` containing
 * the initialization parameters for the SPI interface. The
 * caller retains ownership of this structure.
 * @return Returns 0 on successful initialization, or -1 if an error occurs,
 * such as memory allocation failure or device identity mismatch.
 ******************************************************************************/
int32_t adxl362_init(struct adxl362_dev **device,
		     struct adxl362_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release the resources associated with an
 * ADXL362 device that were previously allocated during initialization.
 * It should be called when the device is no longer needed to ensure
 * proper cleanup and to prevent resource leaks. The function must be
 * called with a valid device structure that was successfully
 * initialized. It is important to note that after calling this function,
 * the device structure should not be used unless it is re-initialized.
 *
 * @param dev A pointer to an adxl362_dev structure representing the device to
 * be removed. This must not be null and should point to a valid,
 * initialized device structure. Passing an invalid or null pointer
 * may result in undefined behavior.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t adxl362_remove(struct adxl362_dev *dev);

/***************************************************************************//**
 * @brief This function is used to write a specified value to a register on the
 * ADXL362 device via SPI communication. It is typically called when
 * configuring the device or updating its settings. The function requires
 * a valid device structure, the value to be written, the address of the
 * register, and the number of bytes to write. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. The function does not return a value, and any errors during
 * SPI communication are not reported back to the caller.
 *
 * @param dev A pointer to an adxl362_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param register_value The value to be written to the register. It is a 16-bit
 * value, but only the least significant bytes as
 * specified by bytes_number will be used.
 * @param register_address The address of the register to which the value will
 * be written. It is an 8-bit value representing a valid
 * register address on the ADXL362.
 * @param bytes_number The number of bytes to write from the register_value to
 * the register. Valid values are 1 or 2, corresponding to
 * the number of bytes to be written.
 * @return None
 ******************************************************************************/
void adxl362_set_register_value(struct adxl362_dev *dev,
				uint16_t register_value,
				uint8_t  register_address,
				uint8_t  bytes_number);

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from consecutive
 * registers of the ADXL362 device, starting from a given register
 * address. It is typically used to retrieve multiple bytes of data in a
 * single operation, which can be more efficient than reading each
 * register individually. The function requires a valid device structure
 * and a buffer to store the read data. It is important to ensure that
 * the buffer is large enough to hold the requested number of bytes. The
 * function does not perform any validation on the input parameters, so
 * the caller must ensure that the register address and byte count are
 * within valid ranges for the device.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param read_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold
 * 'bytes_number' bytes.
 * @param register_address The starting register address from which to begin
 * reading. Must be a valid register address for the
 * ADXL362 device.
 * @param bytes_number The number of bytes to read from the device. Must be a
 * positive integer and should not exceed the size of the
 * read_data buffer.
 * @return The read_data buffer is populated with the data read from the
 * device's registers.
 ******************************************************************************/
void adxl362_get_register_value(struct adxl362_dev *dev,
				uint8_t *read_data,
				uint8_t  register_address,
				uint8_t  bytes_number);

/***************************************************************************//**
 * @brief This function retrieves a specified number of bytes from the FIFO
 * buffer of an ADXL362 device and stores them in a provided buffer. It
 * is typically used when you need to access data that has been
 * accumulated in the FIFO buffer. The function requires a valid device
 * structure and a pre-allocated buffer to store the retrieved data.
 * Ensure that the device has been properly initialized before calling
 * this function. The function does not perform any validation on the
 * input parameters, so the caller must ensure that the buffer is large
 * enough to hold the requested number of bytes.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param buffer A pointer to a pre-allocated buffer where the FIFO data will be
 * stored. Must not be null and should be large enough to hold
 * 'bytes_number' bytes.
 * @param bytes_number The number of bytes to read from the FIFO buffer. Should
 * be a positive integer and not exceed the size of the
 * buffer.
 * @return None
 ******************************************************************************/
void adxl362_get_fifo_value(struct adxl362_dev *dev,
			    uint8_t *buffer,
			    uint16_t bytes_number);

/***************************************************************************//**
 * @brief This function performs a software reset of the ADXL362 accelerometer
 * device, which is useful for reinitializing the device to its default
 * state. It should be called when the device needs to be reset due to
 * configuration changes or unexpected behavior. The function requires a
 * valid device structure that has been initialized using the appropriate
 * initialization function. It is important to ensure that the device is
 * not in use or in a critical operation when this reset is performed, as
 * it will reset all settings to their default values.
 *
 * @param dev A pointer to an initialized 'adxl362_dev' structure representing
 * the device to be reset. This parameter must not be null, and the
 * device must have been successfully initialized prior to calling
 * this function. If the pointer is invalid, the behavior is
 * undefined.
 * @return None
 ******************************************************************************/
void adxl362_software_reset(struct adxl362_dev *dev);

/***************************************************************************//**
 * @brief This function configures the ADXL362 accelerometer to either standby
 * or measurement mode based on the specified power mode. It should be
 * called when there is a need to change the device's operational state,
 * such as before starting measurements or to conserve power. The device
 * must be properly initialized before calling this function. The
 * function modifies the power control register of the device to reflect
 * the desired power mode.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param pwr_mode A uint8_t value representing the desired power mode. Valid
 * values are 0 for standby mode and 2 for measurement mode.
 * Invalid values may result in undefined behavior.
 * @return None
 ******************************************************************************/
void adxl362_set_power_mode(struct adxl362_dev *dev,
			    uint8_t pwr_mode);

/***************************************************************************//**
 * @brief This function configures the measurement range of the ADXL362
 * accelerometer device. It should be called when you need to change the
 * sensitivity of the device to different g-ranges, such as +/-2g, +/-4g,
 * or +/-8g. The function must be called with a valid device structure
 * that has been initialized using `adxl362_init`. The range is specified
 * by the `g_range` parameter, which determines the sensitivity and
 * dynamic range of the accelerometer readings. This function updates the
 * device's internal register to reflect the new range and also updates
 * the `selected_range` field in the device structure to indicate the
 * current range setting.
 *
 * @param dev A pointer to an `adxl362_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param g_range An 8-bit unsigned integer specifying the desired g-range.
 * Valid values are 0, 1, or 2, corresponding to +/-2g, +/-4g,
 * and +/-8g respectively. Invalid values may result in undefined
 * behavior.
 * @return None
 ******************************************************************************/
void adxl362_set_range(struct adxl362_dev *dev,
		       uint8_t g_range);

/***************************************************************************//**
 * @brief This function sets the output data rate (ODR) for the ADXL362
 * accelerometer, which determines how frequently the device samples and
 * outputs data. It should be called after the device has been
 * initialized and before data collection begins to ensure the desired
 * sampling rate is set. The function modifies the device's filter
 * control register to update the ODR. Users must ensure that the
 * provided output rate is within the valid range of the device's
 * supported ODR values.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param out_rate A uint8_t value representing the desired output data rate.
 * Valid values are typically defined by the device's
 * specifications, such as 0 for 12.5 Hz, 1 for 25 Hz, etc.
 * Values outside the supported range may result in undefined
 * behavior.
 * @return None
 ******************************************************************************/
void adxl362_set_output_rate(struct adxl362_dev *dev,
			     uint8_t out_rate);

/***************************************************************************//**
 * @brief Use this function to obtain the raw acceleration data from the ADXL362
 * device along the X, Y, and Z axes. This function should be called when
 * you need to retrieve the current acceleration values for processing or
 * analysis. Ensure that the device has been properly initialized and is
 * in the correct measurement mode before calling this function. The
 * function will populate the provided pointers with the latest
 * acceleration data.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to an int16_t variable where the X-axis acceleration data
 * will be stored. Must not be null.
 * @param y A pointer to an int16_t variable where the Y-axis acceleration data
 * will be stored. Must not be null.
 * @param z A pointer to an int16_t variable where the Z-axis acceleration data
 * will be stored. Must not be null.
 * @return None
 ******************************************************************************/
void adxl362_get_xyz(struct adxl362_dev *dev,
		     int16_t *x,
		     int16_t *y,
		     int16_t *z);

/***************************************************************************//**
 * @brief Use this function to obtain the current acceleration values in g-units
 * for the x, y, and z axes from the ADXL362 accelerometer. This function
 * should be called when you need real-time acceleration data in a human-
 * readable format. Ensure that the device has been properly initialized
 * and configured before calling this function. The function reads raw
 * data from the device, converts it to g-units based on the selected
 * measurement range, and stores the results in the provided float
 * pointers.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to a float where the x-axis acceleration in g-units will
 * be stored. Must not be null.
 * @param y A pointer to a float where the y-axis acceleration in g-units will
 * be stored. Must not be null.
 * @param z A pointer to a float where the z-axis acceleration in g-units will
 * be stored. Must not be null.
 * @return None
 ******************************************************************************/
void adxl362_get_g_xyz(struct adxl362_dev *dev,
		       float* x,
		       float* y,
		       float* z);

/***************************************************************************//**
 * @brief This function retrieves the current temperature reading from the
 * ADXL362 sensor and returns it as a floating-point value in degrees
 * Celsius. It should be called when a temperature measurement is needed
 * from the device. The function requires a valid device structure that
 * has been properly initialized. Ensure that the device is in a state
 * where temperature readings are available before calling this function.
 *
 * @param dev A pointer to an initialized 'adxl362_dev' structure representing
 * the device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the device is properly configured for
 * temperature reading.
 * @return Returns the temperature in degrees Celsius as a float. If the device
 * is not properly initialized or configured, the behavior is undefined.
 ******************************************************************************/
float adxl362_read_temperature(struct adxl362_dev *dev);

/***************************************************************************//**
 * @brief This function sets up the FIFO (First In, First Out) feature of the
 * ADXL362 accelerometer device. It allows the user to specify the FIFO
 * mode, set a watermark level for FIFO interrupts, and enable or disable
 * temperature readings in the FIFO. This function should be called after
 * the device has been initialized and is typically used to manage data
 * buffering and interrupt generation based on FIFO conditions. Proper
 * configuration of the FIFO can help in efficiently handling data
 * collection and processing in applications requiring buffered data
 * acquisition.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value specifying the FIFO mode. Valid values are
 * ADXL362_FIFO_DISABLE, ADXL362_FIFO_OLDEST_SAVED,
 * ADXL362_FIFO_STREAM, and ADXL362_FIFO_TRIGGERED.
 * @param water_mark_lvl A uint16_t value representing the FIFO watermark level.
 * This value determines the number of samples that
 * trigger a watermark interrupt.
 * @param en_temp_read A uint8_t value indicating whether temperature readings
 * should be included in the FIFO. A non-zero value enables
 * temperature readings, while zero disables them.
 * @return None
 ******************************************************************************/
void adxl362_fifo_setup(struct adxl362_dev *dev,
			uint8_t  mode,
			uint16_t water_mark_lvl,
			uint8_t  en_temp_read);

/***************************************************************************//**
 * @brief This function sets up the activity detection feature of the ADXL362
 * accelerometer. It configures the motion threshold and activity timer,
 * enabling the activity interrupt. The function allows the user to
 * choose between a referenced or absolute configuration for activity
 * detection. It must be called with a valid device structure that has
 * been initialized. This function is typically used in applications
 * where motion detection is required, such as in wearable devices or
 * motion-triggered systems.
 *
 * @param dev A pointer to an initialized adxl362_dev structure. Must not be
 * null. The caller retains ownership.
 * @param ref_or_abs A uint8_t value indicating the configuration mode: 0 for
 * absolute, 1 for referenced. Invalid values may result in
 * undefined behavior.
 * @param threshold A uint16_t value representing the motion threshold. The
 * value is clamped to 11 bits (0 to 0x7FF). Values outside
 * this range are automatically adjusted.
 * @param time A uint8_t value specifying the activity timer duration. Must be a
 * valid 8-bit value.
 * @return None
 ******************************************************************************/
void adxl362_setup_activity_detection(struct adxl362_dev *dev,
				      uint8_t  ref_or_abs,
				      uint16_t threshold,
				      uint8_t  time);

/***************************************************************************//**
 * @brief This function sets up the inactivity detection feature on the ADXL362
 * accelerometer device. It configures the motion threshold and
 * inactivity timer, enabling the inactivity interrupt with either a
 * referenced or absolute configuration. This function should be called
 * after the device has been initialized and is typically used to detect
 * periods of inactivity based on the specified threshold and time
 * parameters. Proper configuration of these parameters is essential for
 * accurate inactivity detection.
 *
 * @param dev A pointer to an initialized adxl362_dev structure representing the
 * device. Must not be null.
 * @param ref_or_abs A uint8_t value indicating whether to use referenced (1) or
 * absolute (0) inactivity detection. Invalid values may lead
 * to undefined behavior.
 * @param threshold A uint16_t value representing the inactivity threshold.
 * Valid range is 0 to 2047.
 * @param time A uint16_t value specifying the inactivity time period. Must be a
 * valid non-negative integer.
 * @return None
 ******************************************************************************/
void adxl362_setup_inactivity_detection(struct adxl362_dev *dev,
					uint8_t  ref_or_abs,
					uint16_t threshold,
					uint16_t time);

#endif /* __ADXL362_H__ */

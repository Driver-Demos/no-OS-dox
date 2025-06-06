/***************************************************************************//**
 *   @file   adxl345.h
 *   @brief  Header file of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
#ifndef __ADXL345_H__
#define __ADXL345_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/******************************** ADXL345 *************************************/
/******************************************************************************/

/* Options for communicating with the device. */
#define ADXL345_SPI_COMM       0
#define ADXL345_I2C_COMM       1

/* I2C address of the device */
#define ADXL345_ADDRESS		0x1D

/* SPI commands */
#define ADXL345_SPI_READ        (1 << 7)
#define ADXL345_SPI_WRITE       (0 << 7)
#define ADXL345_SPI_MB          (1 << 6)

/* ADXL345 Register Map */
#define	ADXL345_DEVID           0x00 // R   Device ID.
#define ADXL345_THRESH_TAP      0x1D // R/W Tap threshold.
#define ADXL345_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_DUR             0x21 // R/W Tap duration.
#define ADXL345_LATENT          0x22 // R/W Tap latency.
#define ADXL345_WINDOW          0x23 // R/W Tap window.
#define ADXL345_THRESH_ACT      0x24 // R/W Activity threshold.
#define ADXL345_THRESH_INACT    0x25 // R/W Inactivity threshold.
#define ADXL345_TIME_INACT      0x26 // R/W Inactivity time.
#define ADXL345_ACT_INACT_CTL   0x27 // R/W Axis enable control for activity
// and inactivity detection.
#define ADXL345_THRESH_FF       0x28 // R/W Free-fall threshold.
#define ADXL345_TIME_FF         0x29 // R/W Free-fall time.
#define ADXL345_TAP_AXES        0x2A // R/W Axis control for tap/double tap.
#define ADXL345_ACT_TAP_STATUS  0x2B // R   Source of tap/double tap.
#define ADXL345_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_FIFO_STATUS     0x39 // R   FIFO status.
#define ADXL345_TAP_SIGN        0x3A // R   Sign and source for single tap/double tap.
#define ADXL345_ORIENT_CONF     0x3B // R/W Orientation configuration.
#define ADXL345_ORIENT          0x3C // R   Orientation status.

/* ADXL345_ACT_INACT_CTL definition */
#define ADXL345_ACT_ACDC        (1 << 7)
#define ADXL345_ACT_X_EN        (1 << 6)
#define ADXL345_ACT_Y_EN        (1 << 5)
#define ADXL345_ACT_Z_EN        (1 << 4)
#define ADXL345_INACT_ACDC      (1 << 3)
#define ADXL345_INACT_X_EN      (1 << 2)
#define ADXL345_INACT_Y_EN      (1 << 1)
#define ADXL345_INACT_Z_EN      (1 << 0)

/* ADXL345_TAP_AXES definition */
#define ADXL345_SUPPRESS        (1 << 3)
#define ADXL345_TAP_X_EN        (1 << 2)
#define ADXL345_TAP_Y_EN        (1 << 1)
#define ADXL345_TAP_Z_EN        (1 << 0)

/* ADXL345_ACT_TAP_STATUS definition */
#define ADXL345_ACT_X_SRC       (1 << 6)
#define ADXL345_ACT_Y_SRC       (1 << 5)
#define ADXL345_ACT_Z_SRC       (1 << 4)
#define ADXL345_ASLEEP          (1 << 3)
#define ADXL345_TAP_X_SRC       (1 << 2)
#define ADXL345_TAP_Y_SRC       (1 << 1)
#define ADXL345_TAP_Z_SRC       (1 << 0)

/* ADXL345_BW_RATE definition */
#define ADXL345_LOW_POWER       (1 << 4)
#define ADXL345_RATE(x)         ((x) & 0xF)

/* ADXL345_POWER_CTL definition */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE definition */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)
#define ADXL345_ORIENTATION     (1 << 0)

/* ADXL345_DATA_FORMAT definition */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)

/* ADXL345_RANGE(x) options */
#define ADXL345_RANGE_PM_2G     0
#define ADXL345_RANGE_PM_4G     1
#define ADXL345_RANGE_PM_8G     2
#define ADXL345_RANGE_PM_16G    3

/* ADXL345_FIFO_CTL definition */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_FIFO_MODE(x) options */
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3

/* ADXL345_FIFO_STATUS definition */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345_ORIENT_CONF definition */
#define ADXL345_INT_ORIENT(x)   (((x) & 0x1) << 7)
#define ADXL345_DEAD_ZONE(x)    (((x) & 0x7) << 4)
#define ADXL345_INT_3D(x)       (((x) & 0x1) << 3)
#define ADXL345_DIVISOR(x)      ((x) & 7)

/* ADXL345 ID */
#define ADXL345_ID              0xE5
/* ADXL346 ID */
#define ADXL346_ID              0xE6

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_SCALE_FACTOR    0.0039

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxl345_type` enumeration defines the supported device types for
 * the ADXL345 driver, specifically identifying the ADXL345 and ADXL346
 * accelerometer models. This enumeration is used to specify the type of
 * device being interfaced with, allowing the driver to handle device-
 * specific operations and configurations.
 *
 * @param ID_ADXL345 Represents the ADXL345 device type.
 * @param ID_ADXL346 Represents the ADXL346 device type.
 ******************************************************************************/
enum adxl345_type {
	ID_ADXL345,
	ID_ADXL346,
};

/***************************************************************************//**
 * @brief The `adxl345_dead_zone_angle` enumeration defines a set of constants
 * representing specific dead zone angles used in the ADXL345
 * accelerometer's orientation detection feature. These angles are used
 * to configure the sensitivity of the device to changes in orientation,
 * allowing for precise control over how the device responds to different
 * tilt angles.
 *
 * @param DEGREES_5_1 Represents a dead zone angle of 5.1 degrees.
 * @param DEGREES_10_2 Represents a dead zone angle of 10.2 degrees.
 * @param DEGREES_15_2 Represents a dead zone angle of 15.2 degrees.
 * @param DEGREES_20_4 Represents a dead zone angle of 20.4 degrees.
 * @param DEGREES_25_5 Represents a dead zone angle of 25.5 degrees.
 * @param DEGREES_30_8 Represents a dead zone angle of 30.8 degrees.
 * @param DEGREES_36_1 Represents a dead zone angle of 36.1 degrees.
 * @param DEGREES_41_4 Represents a dead zone angle of 41.4 degrees.
 ******************************************************************************/
enum adxl345_dead_zone_angle {
	DEGREES_5_1,
	DEGREES_10_2,
	DEGREES_15_2,
	DEGREES_20_4,
	DEGREES_25_5,
	DEGREES_30_8,
	DEGREES_36_1,
	DEGREES_41_4,
};

/***************************************************************************//**
 * @brief The `adxl345_divisor_bandwidth` enumeration defines a set of constants
 * used to specify the divisor values for configuring the bandwidth of
 * the ADXL345 accelerometer. Each enumerator corresponds to a specific
 * output data rate (ODR) divisor, which affects the bandwidth and power
 * consumption of the device. This enumeration is used to select the
 * appropriate bandwidth setting when configuring the ADXL345 for
 * different applications.
 *
 * @param ODR_DIV_9 Represents a divisor for a specific bandwidth setting in the
 * ADXL345 device.
 * @param ODR_DIV_22 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_50 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_100 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_200 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_400 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_800 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 * @param ODR_DIV_1600 Represents a divisor for a specific bandwidth setting in
 * the ADXL345 device.
 ******************************************************************************/
enum adxl345_divisor_bandwidth {
	ODR_DIV_9,
	ODR_DIV_22,
	ODR_DIV_50,
	ODR_DIV_100,
	ODR_DIV_200,
	ODR_DIV_400,
	ODR_DIV_800,
	ODR_DIV_1600
};

/***************************************************************************//**
 * @brief The `adxl345_dev` structure is designed to encapsulate the
 * configuration and communication details for an ADXL345 or ADXL346
 * accelerometer device. It includes descriptors for both I2C and SPI
 * communication, allowing flexibility in how the device is interfaced.
 * The structure also holds information about the device type,
 * communication method, measurement range, and resolution settings,
 * which are essential for configuring and operating the accelerometer in
 * various applications.
 *
 * @param i2c_desc Pointer to an I2C descriptor for communication.
 * @param spi_desc Pointer to an SPI descriptor for communication.
 * @param dev_type Specifies the device type, either ADXL345 or ADXL346.
 * @param communication_type Indicates the communication type, either SPI or
 * I2C.
 * @param selected_range Defines the measurement range of the device.
 * @param full_resolution_set Indicates whether full resolution is enabled or
 * disabled.
 ******************************************************************************/
struct adxl345_dev {
	/** I2C Descriptor */
	struct no_os_i2c_desc	*i2c_desc;
	/** SPI Descriptor */
	struct no_os_spi_desc	*spi_desc;
	/** Device type ADXL345 or 346 */
	enum adxl345_type	dev_type;
	/** Device Communication type: ADXL345_SPI_COMM, ADXL345_I2C_COMM */
	uint8_t		communication_type;
	/** Measurement range */
	uint8_t		selected_range;
	/** Enable/Disable Full Resolution */
	uint8_t		full_resolution_set;
};

/***************************************************************************//**
 * @brief The `adxl345_init_param` structure is used to hold the initialization
 * parameters for the ADXL345 device, which is a digital accelerometer.
 * It includes configurations for both I2C and SPI communication
 * interfaces, allowing the user to specify the desired communication
 * protocol. The structure also specifies the device type (either ADXL345
 * or ADXL346), the communication type (SPI or I2C), the measurement
 * range, and whether full resolution mode is enabled. This structure is
 * essential for setting up the device before it can be used for data
 * acquisition.
 *
 * @param i2c_init I2C Initialization structure.
 * @param spi_init SPI Initialization structure.
 * @param dev_type Device type ADXL345 or 346.
 * @param communication_type Device Communication type: ADXL345_SPI_COMM,
 * ADXL345_I2C_COMM.
 * @param selected_range Measurement range.
 * @param full_resolution_set Enable/Disable Full Resolution.
 ******************************************************************************/
struct adxl345_init_param {
	/** I2C Initialization structure. */
	struct no_os_i2c_init_param	i2c_init;
	/** SPI Initialization structure. */
	struct no_os_spi_init_param	spi_init;
	/** Device type ADXL345 or 346 */
	enum adxl345_type		dev_type;
	/** Device Communication type: ADXL345_SPI_COMM, ADXL345_I2C_COMM */
	uint8_t		communication_type;
	/** Measurement range */
	uint8_t		selected_range;
	/** Enable/Disable Full Resolution */
	uint8_t		full_resolution_set;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function retrieves the value stored in a specific register of the
 * ADXL345 accelerometer device. It should be used when you need to
 * access configuration or data registers of the device. The function
 * requires a valid device structure that has been properly initialized
 * and configured for either SPI or I2C communication. The register
 * address must be within the valid range of the device's register map.
 * The function returns the value of the register, and it is important to
 * ensure that the device is in a state where reading from the register
 * is appropriate.
 *
 * @param dev A pointer to an adxl345_dev structure representing the device.
 * This must be initialized and configured for communication. Must
 * not be null.
 * @param register_address The address of the register to read from. Must be a
 * valid register address as defined in the ADXL345
 * register map.
 * @return Returns the 8-bit value read from the specified register.
 ******************************************************************************/
uint8_t adxl345_get_register_value(struct adxl345_dev *dev,
				   uint8_t register_address);

/***************************************************************************//**
 * @brief Use this function to set a specific register on the ADXL345 device to
 * a desired value. This function requires the device to be properly
 * initialized and configured for either SPI or I2C communication. It is
 * essential to ensure that the device structure is correctly set up with
 * the appropriate communication type before calling this function. This
 * function does not return a value, and it assumes that the provided
 * register address and value are valid for the ADXL345 device.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null, and the communication_type field must be
 * correctly set to either ADXL345_SPI_COMM or ADXL345_I2C_COMM.
 * @param register_address The address of the register to be written to. Must be
 * a valid register address for the ADXL345 device.
 * @param register_value The value to write to the specified register. Must be a
 * valid value for the register being addressed.
 * @return None
 ******************************************************************************/
void adxl345_set_register_value(struct adxl345_dev *dev,
				uint8_t register_address,
				uint8_t register_value);

/***************************************************************************//**
 * @brief This function initializes the ADXL345 device by setting up the
 * communication interface and verifying the device's presence. It should
 * be called before any other operations on the device to ensure proper
 * setup. The function allocates memory for the device structure and
 * configures the communication type based on the provided initialization
 * parameters. It also sets the default measurement range and resolution
 * settings. If the device is not detected or memory allocation fails,
 * the function returns an error code.
 *
 * @param device A pointer to a pointer of type `struct adxl345_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adxl345_init_param` containing
 * initialization parameters such as communication type and
 * device type. The structure must be properly populated
 * before calling the function.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails.
 ******************************************************************************/
int32_t adxl345_init(struct adxl345_dev **device,
		     struct adxl345_init_param init_param);

/***************************************************************************//**
 * @brief This function should be called to properly release resources allocated
 * for an ADXL345 device after it is no longer needed. It handles the
 * cleanup of communication descriptors based on the communication type
 * (SPI or I2C) and frees the memory associated with the device
 * structure. It is important to call this function to prevent resource
 * leaks. Ensure that the device structure was previously initialized and
 * is valid before calling this function.
 *
 * @param dev A pointer to an adxl345_dev structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. The function will handle invalid
 * communication types by attempting to remove both SPI and I2C
 * descriptors.
 * @return Returns an int32_t indicating the success or failure of the resource
 * removal process. A non-zero return value indicates an error occurred
 * during the removal of the communication descriptor.
 ******************************************************************************/
int32_t adxl345_remove(struct adxl345_dev *dev);

/***************************************************************************//**
 * @brief This function configures the ADXL345 accelerometer to either standby
 * or measurement mode based on the provided power mode parameter. It
 * should be called when there is a need to change the device's power
 * state, such as before starting measurements or to conserve power when
 * measurements are not needed. The device must be properly initialized
 * before calling this function. The function modifies the power control
 * register of the device to reflect the desired power mode.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before use.
 * @param pwr_mode A uint8_t value representing the desired power mode.
 * Typically, 0 for standby mode and 1 for measurement mode.
 * Invalid values may result in undefined behavior.
 * @return None
 ******************************************************************************/
void adxl345_set_power_mode(struct adxl345_dev *dev,
			    uint8_t pwr_mode);

/***************************************************************************//**
 * @brief This function retrieves the raw acceleration data for the X, Y, and Z
 * axes from an ADXL345 device. It must be called with a properly
 * initialized `adxl345_dev` structure, which specifies the communication
 * type (SPI or I2C) and the necessary descriptors. The function writes
 * the acceleration data into the provided integer pointers for each
 * axis. It is essential to ensure that the device is in the correct mode
 * to provide valid data before calling this function. The function does
 * not handle invalid device structures or null pointers, so these must
 * be managed by the caller.
 *
 * @param dev A pointer to an `adxl345_dev` structure representing the device.
 * It must be properly initialized and configured for either SPI or
 * I2C communication. The caller retains ownership.
 * @param x A pointer to an `int16_t` where the raw X-axis acceleration data
 * will be stored. Must not be null.
 * @param y A pointer to an `int16_t` where the raw Y-axis acceleration data
 * will be stored. Must not be null.
 * @param z A pointer to an `int16_t` where the raw Z-axis acceleration data
 * will be stored. Must not be null.
 * @return None
 ******************************************************************************/
void adxl345_get_xyz(struct adxl345_dev *dev,
		     int16_t* x,
		     int16_t* y,
		     int16_t* z);

/***************************************************************************//**
 * @brief This function retrieves the raw accelerometer data from the ADXL345
 * device and converts it into g-forces for each axis. It should be
 * called when you need the current acceleration values in g, which are
 * useful for applications requiring precise motion detection or
 * orientation sensing. Ensure that the device has been properly
 * initialized and configured before calling this function. The function
 * writes the converted values to the provided pointers for the x, y, and
 * z axes.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to a float where the x-axis acceleration in g will be
 * stored. Must not be null.
 * @param y A pointer to a float where the y-axis acceleration in g will be
 * stored. Must not be null.
 * @param z A pointer to a float where the z-axis acceleration in g will be
 * stored. Must not be null.
 * @return None
 ******************************************************************************/
void adxl345_get_g_xyz(struct adxl345_dev *dev,
		       float* x,
		       float* y,
		       float* z);

/***************************************************************************//**
 * @brief This function is used to configure the tap detection feature of the
 * ADXL345 accelerometer. It allows the user to specify the type of tap
 * detection (single or double), the axes on which to detect taps, and
 * various timing and threshold parameters that define the tap detection
 * behavior. This function should be called after the device has been
 * initialized and is in a suitable power mode. It modifies the device's
 * internal registers to enable or disable tap detection and to set the
 * parameters for tap detection events.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param tap_type A bitmask indicating the type of tap detection to enable
 * (e.g., single or double tap). Valid values are defined by the
 * ADXL345_SINGLE_TAP and ADXL345_DOUBLE_TAP constants.
 * @param tap_axes A bitmask specifying which axes to enable for tap detection.
 * Valid values are combinations of ADXL345_TAP_X_EN,
 * ADXL345_TAP_Y_EN, and ADXL345_TAP_Z_EN.
 * @param tap_dur The maximum time that an event must be above the threshold to
 * qualify as a tap event. Expressed in units of 625 µs.
 * @param tap_latent The wait time from the detection of a tap event to the
 * start of the time window during which a possible second tap
 * event can be detected. Expressed in units of 1.25 ms.
 * @param tap_window The amount of time after the expiration of the latency time
 * during which a second valid tap can begin. Expressed in
 * units of 1.25 ms.
 * @param tap_thresh The threshold value for tap detection. Expressed in g's,
 * where 1 g is 62.5 mg.
 * @param tap_int A bitmask indicating which interrupt pins to map the tap
 * detection events to. Valid values are defined by the
 * ADXL345_INT_MAP constants.
 * @return None
 ******************************************************************************/
void adxl345_set_tap_detection(struct adxl345_dev *dev,
			       uint8_t tap_type,
			       uint8_t tap_axes,
			       uint8_t tap_dur,
			       uint8_t tap_latent,
			       uint8_t tap_window,
			       uint8_t tap_thresh,
			       uint8_t tap_int);

/***************************************************************************//**
 * @brief This function is used to enable or disable activity detection on the
 * ADXL345 accelerometer. It allows the user to specify which axes to
 * monitor, whether to use AC or DC coupling, the activity threshold, and
 * the interrupt mapping. This function should be called after the device
 * has been initialized and is typically used to configure the device for
 * specific motion detection applications. Proper configuration of the
 * parameters is necessary to ensure accurate activity detection.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param act_on_off A uint8_t value indicating whether activity detection is
 * enabled (non-zero) or disabled (zero).
 * @param act_axes A uint8_t bitmask specifying which axes (X, Y, Z) to enable
 * for activity detection. Valid values are combinations of
 * ADXL345_ACT_X_EN, ADXL345_ACT_Y_EN, and ADXL345_ACT_Z_EN.
 * @param act_ac_dc A uint8_t value indicating whether to use AC
 * (ADXL345_ACT_ACDC set) or DC coupling (ADXL345_ACT_ACDC
 * cleared) for activity detection.
 * @param act_thresh A uint8_t value representing the threshold for activity
 * detection. The value is device-specific and should be set
 * according to the desired sensitivity.
 * @param act_int A uint8_t value specifying the interrupt mapping for activity
 * detection. Typically, this is a bitmask where ADXL345_ACTIVITY
 * is set to map the interrupt to a specific pin.
 * @return None
 ******************************************************************************/
void adxl345_set_activity_detection(struct adxl345_dev *dev,
				    uint8_t act_on_off,
				    uint8_t act_axes,
				    uint8_t act_ac_dc,
				    uint8_t act_thresh,
				    uint8_t act_int);

/***************************************************************************//**
 * @brief Use this function to enable or disable inactivity detection on the
 * ADXL345 accelerometer, specifying the axes to monitor, the mode of
 * operation, the threshold, the time duration for inactivity, and the
 * interrupt mapping. This function should be called after the device has
 * been initialized and is in a suitable power mode. It modifies the
 * device's registers to set the desired inactivity detection parameters,
 * which can trigger an interrupt if configured. Ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param inact_on_off A uint8_t value indicating whether inactivity detection
 * is enabled (non-zero) or disabled (zero).
 * @param inact_axes A uint8_t bitmask specifying which axes (X, Y, Z) to
 * monitor for inactivity. Valid values are combinations of
 * ADXL345_INACT_X_EN, ADXL345_INACT_Y_EN, and
 * ADXL345_INACT_Z_EN.
 * @param inact_ac_dc A uint8_t value specifying the mode of operation for
 * inactivity detection: AC (non-zero) or DC (zero).
 * @param inact_thresh A uint8_t value representing the threshold for inactivity
 * detection. The valid range is device-specific and should
 * be set according to the desired sensitivity.
 * @param inact_time A uint8_t value specifying the time duration for which
 * inactivity must be detected before triggering an interrupt.
 * The valid range is device-specific.
 * @param inact_int A uint8_t value indicating the interrupt mapping for
 * inactivity detection. This determines which interrupt pin
 * will be used if inactivity is detected.
 * @return None
 ******************************************************************************/
void adxl345_set_inactivity_detection(struct adxl345_dev *dev,
				      uint8_t inact_on_off,
				      uint8_t inact_axes,
				      uint8_t inact_ac_dc,
				      uint8_t inact_thresh,
				      uint8_t inact_time,
				      uint8_t inact_int);

/***************************************************************************//**
 * @brief This function enables or disables the free-fall detection feature on
 * an ADXL345 device, allowing the user to specify the threshold, time,
 * and interrupt settings for free-fall events. It should be called when
 * the device is initialized and ready for configuration. The function
 * modifies the device's registers to set the free-fall threshold and
 * time, and configures the interrupt mapping and enabling based on the
 * provided parameters. It is important to ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param ff_on_off A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) free-fall detection.
 * @param ff_thresh A uint8_t value representing the free-fall threshold. Valid
 * range is device-specific and should be set according to the
 * desired sensitivity.
 * @param ff_time A uint8_t value representing the free-fall time. Valid range
 * is device-specific and should be set according to the desired
 * duration for free-fall detection.
 * @param ff_int A uint8_t value indicating the interrupt mapping for free-fall
 * detection. Should be set according to the desired interrupt
 * configuration.
 * @return None
 ******************************************************************************/
void adxl345_set_free_fall_detection(struct adxl345_dev *dev,
				     uint8_t ff_on_off,
				     uint8_t ff_thresh,
				     uint8_t ff_time,
				     uint8_t ff_int);

/***************************************************************************//**
 * @brief This function enables or disables orientation detection on an ADXL345
 * device, allowing the user to specify various parameters such as
 * interrupt mapping, 3D detection, dead zone angle, and divisor
 * bandwidth. It should be called only for devices of type ADXL346, as it
 * will not perform any action for other device types. This function is
 * useful for applications that require orientation-based interrupts and
 * should be used after the device has been properly initialized.
 *
 * @param dev A pointer to an adxl345_dev structure representing the device.
 * Must not be null and must be initialized with the correct device
 * type (ID_ADXL346) before calling this function.
 * @param orient_int A uint8_t value specifying the interrupt mapping for
 * orientation detection. Valid values depend on the specific
 * interrupt configuration desired.
 * @param orient_on_off A uint8_t value indicating whether orientation detection
 * is enabled (non-zero) or disabled (zero).
 * @param int_3d A uint8_t value indicating whether 3D orientation detection is
 * enabled (non-zero) or disabled (zero).
 * @param dead_zone An enum adxl345_dead_zone_angle value specifying the dead
 * zone angle for orientation detection. Must be one of the
 * predefined enum values.
 * @param divisor An enum adxl345_divisor_bandwidth value specifying the divisor
 * bandwidth for orientation detection. Must be one of the
 * predefined enum values.
 * @return None
 ******************************************************************************/
void adxl345_set_orientation_detection(struct adxl345_dev *dev,
				       uint8_t orient_int,
				       uint8_t orient_on_off,
				       uint8_t int_3d,
				       enum adxl345_dead_zone_angle dead_zone,
				       enum adxl345_divisor_bandwidth divisor);

/***************************************************************************//**
 * @brief This function is used to apply offset calibration to the X, Y, and Z
 * axes of an ADXL345 accelerometer device. It should be called when you
 * need to adjust the zero-g offset for each axis to correct for any bias
 * in the sensor readings. This is typically done after initializing the
 * device and before taking precise measurements. The function writes the
 * specified offset values to the corresponding registers of the device,
 * allowing for fine-tuning of the sensor's output.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param x_offset The offset value to be applied to the X-axis. It is an 8-bit
 * unsigned integer, typically ranging from 0 to 255.
 * @param y_offset The offset value to be applied to the Y-axis. It is an 8-bit
 * unsigned integer, typically ranging from 0 to 255.
 * @param z_offset The offset value to be applied to the Z-axis. It is an 8-bit
 * unsigned integer, typically ranging from 0 to 255.
 * @return None
 ******************************************************************************/
void adxl345_set_offset(struct adxl345_dev *dev,
			uint8_t x_offset,
			uint8_t y_offset,
			uint8_t z_offset);

/***************************************************************************//**
 * @brief Use this function to configure the measurement range and resolution
 * mode of an ADXL345 device. This function should be called after
 * initializing the device and before starting measurements. It updates
 * the device's data format register to reflect the desired range and
 * resolution settings. The function does not perform any validation on
 * the input parameters, so ensure that the provided range and resolution
 * values are within the valid options for the device.
 *
 * @param dev A pointer to an initialized adxl345_dev structure representing the
 * device. Must not be null.
 * @param g_range The desired measurement range, specified as a 2-bit value.
 * Valid values are 0 (±2g), 1 (±4g), 2 (±8g), and 3 (±16g).
 * @param full_res A flag indicating whether full resolution mode is enabled.
 * Use 1 to enable full resolution and 0 to disable it.
 * @return None
 ******************************************************************************/
void adxl345_set_range_resolution(struct adxl345_dev *dev,
				  uint8_t g_range,
				  uint8_t full_res);

#endif	/* __ADXL345_H__ */

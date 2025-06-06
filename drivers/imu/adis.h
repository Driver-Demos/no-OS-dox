/***************************************************************************//**
 *   @file   adis.h
 *   @brief  Implementation of adis.h
 *   @author RBolboac (ramona.bolboaca@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
 ******************************************************************************/

#ifndef __ADIS_H__
#define __ADIS_H__

#ifdef TEST
#define STATIC
#else
#define STATIC static
#endif

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no_os_spi.h"
#include "no_os_util.h"
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ADIS_4_BYTES_SIZE	4
#define ADIS_2_BYTES_SIZE	2
#define ADIS_1_BYTE_SIZE	1

#define ADIS_SYNC_DEFAULT	0
#define ADIS_SYNC_DIRECT	1
#define ADIS_SYNC_SCALED	2
#define ADIS_SYNC_OUTPUT	3
#define ADIS_SYNC_PULSE		5

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adis_device_id` enumeration defines a set of constants
 * representing different device IDs for various models and variants of
 * ADIS devices. Each enumerator corresponds to a specific model or
 * variant, allowing for easy identification and selection of a
 * particular device within the ADIS family. This enumeration is useful
 * in applications where specific device configurations or operations are
 * required based on the device type.
 *
 * @param ADIS16465_1 Represents the first variant of the ADIS16465 device.
 * @param ADIS16465_2 Represents the second variant of the ADIS16465 device.
 * @param ADIS16465_3 Represents the third variant of the ADIS16465 device.
 * @param ADIS16467_1 Represents the first variant of the ADIS16467 device.
 * @param ADIS16467_2 Represents the second variant of the ADIS16467 device.
 * @param ADIS16467_3 Represents the third variant of the ADIS16467 device.
 * @param ADIS16470 Represents the ADIS16470 device.
 * @param ADIS16475_1 Represents the first variant of the ADIS16475 device.
 * @param ADIS16475_2 Represents the second variant of the ADIS16475 device.
 * @param ADIS16475_3 Represents the third variant of the ADIS16475 device.
 * @param ADIS16477_1 Represents the first variant of the ADIS16477 device.
 * @param ADIS16477_2 Represents the second variant of the ADIS16477 device.
 * @param ADIS16477_3 Represents the third variant of the ADIS16477 device.
 * @param ADIS16500 Represents the ADIS16500 device.
 * @param ADIS16501 Represents the ADIS16501 device.
 * @param ADIS16505_1 Represents the first variant of the ADIS16505 device.
 * @param ADIS16505_2 Represents the second variant of the ADIS16505 device.
 * @param ADIS16505_3 Represents the third variant of the ADIS16505 device.
 * @param ADIS16507_1 Represents the first variant of the ADIS16507 device.
 * @param ADIS16507_2 Represents the second variant of the ADIS16507 device.
 * @param ADIS16507_3 Represents the third variant of the ADIS16507 device.
 * @param ADIS16545_1 Represents the first variant of the ADIS16545 device.
 * @param ADIS16545_2 Represents the second variant of the ADIS16545 device.
 * @param ADIS16545_3 Represents the third variant of the ADIS16545 device.
 * @param ADIS16547_1 Represents the first variant of the ADIS16547 device.
 * @param ADIS16547_2 Represents the second variant of the ADIS16547 device.
 * @param ADIS16547_3 Represents the third variant of the ADIS16547 device.
 * @param ADIS16550 Represents the ADIS16550 device.
 * @param ADIS16575_2 Represents the second variant of the ADIS16575 device.
 * @param ADIS16575_3 Represents the third variant of the ADIS16575 device.
 * @param ADIS16576_2 Represents the second variant of the ADIS16576 device.
 * @param ADIS16576_3 Represents the third variant of the ADIS16576 device.
 * @param ADIS16577_2 Represents the second variant of the ADIS16577 device.
 * @param ADIS16577_3 Represents the third variant of the ADIS16577 device.
 ******************************************************************************/
enum adis_device_id {
	ADIS16465_1,
	ADIS16465_2,
	ADIS16465_3,
	ADIS16467_1,
	ADIS16467_2,
	ADIS16467_3,
	ADIS16470,
	ADIS16475_1,
	ADIS16475_2,
	ADIS16475_3,
	ADIS16477_1,
	ADIS16477_2,
	ADIS16477_3,
	ADIS16500,
	ADIS16501,
	ADIS16505_1,
	ADIS16505_2,
	ADIS16505_3,
	ADIS16507_1,
	ADIS16507_2,
	ADIS16507_3,
	ADIS16545_1,
	ADIS16545_2,
	ADIS16545_3,
	ADIS16547_1,
	ADIS16547_2,
	ADIS16547_3,
	ADIS16550,
	ADIS16575_2,
	ADIS16575_3,
	ADIS16576_2,
	ADIS16576_3,
	ADIS16577_2,
	ADIS16577_3,
};

/***************************************************************************//**
 * @brief The `adis_chan_type` enumeration defines the different types of
 * channels supported by the ADIS device, including accelerometer,
 * gyroscope, temperature, delta angle, and delta velocity channels. This
 * enumeration is used to specify the type of data channel being accessed
 * or configured in the ADIS device, facilitating the management and
 * processing of sensor data.
 *
 * @param ADIS_ACCL_CHAN Represents the accelerometer channel.
 * @param ADIS_GYRO_CHAN Represents the gyroscope channel.
 * @param ADIS_TEMP_CHAN Represents the temperature channel.
 * @param ADIS_DELTAANGL_CHAN Represents the delta angle channel.
 * @param ADIS_DELTAVEL_CHAN Represents the delta velocity channel.
 ******************************************************************************/
enum adis_chan_type {
	ADIS_ACCL_CHAN,
	ADIS_GYRO_CHAN,
	ADIS_TEMP_CHAN,
	ADIS_DELTAANGL_CHAN,
	ADIS_DELTAVEL_CHAN,
};

/***************************************************************************//**
 * @brief The `adis_axis_type` is an enumeration that defines the supported axes
 * for a device, specifically the X, Y, and Z axes. This enumeration is
 * used to specify or identify the axis of interest in various operations
 * or configurations within the ADIS device context.
 *
 * @param ADIS_X_AXIS Represents the X-axis in the enumeration.
 * @param ADIS_Y_AXIS Represents the Y-axis in the enumeration.
 * @param ADIS_Z_AXIS Represents the Z-axis in the enumeration.
 ******************************************************************************/
enum adis_axis_type {
	ADIS_X_AXIS,
	ADIS_Y_AXIS,
	ADIS_Z_AXIS,
};


/***************************************************************************//**
 * @brief The `adis_diag_flags` structure is a bitfield that maps various
 * diagnostic flags for an ADIS sensor device, providing a comprehensive
 * status of potential errors and operational states. Each member of the
 * structure represents a specific diagnostic flag, such as sensor
 * initialization failures, communication errors, and hardware faults,
 * allowing for detailed monitoring and troubleshooting of the device's
 * health and functionality. This structure is crucial for ensuring the
 * reliability and accuracy of the sensor's data by enabling the
 * detection and reporting of any anomalies or failures in real-time.
 *
 * @param snsr_init_failure Indicates a sensor initialization failure.
 * @param data_path_overrun Indicates a data path overrun event.
 * @param fls_mem_update_failure Indicates a flash memory update failure.
 * @param spi_comm_err Indicates an SPI communication error.
 * @param standby_mode Indicates the device is in standby mode.
 * @param snsr_failure Indicates a general sensor failure.
 * @param mem_failure Indicates a memory failure.
 * @param clk_err Indicates a clock error.
 * @param gyro1_failure Indicates a failure in gyroscope 1.
 * @param gyro2_failure Indicates a failure in gyroscope 2.
 * @param accl_failure Indicates an accelerometer failure.
 * @param x_axis_gyro_failure Indicates a failure in the X-axis gyroscope.
 * @param y_axis_gyro_failure Indicates a failure in the Y-axis gyroscope.
 * @param z_axis_gyro_failure Indicates a failure in the Z-axis gyroscope.
 * @param x_axis_accl_failure Indicates a failure in the X-axis accelerometer.
 * @param y_axis_accl_failure Indicates a failure in the Y-axis accelerometer.
 * @param z_axis_accl_failure Indicates a failure in the Z-axis accelerometer.
 * @param aduc_mcu_fault Indicates a fault in the ADuC microcontroller.
 * @param config_calib_crc_error Indicates a CRC error in configuration or
 * calibration.
 * @param overrange Indicates an overrange event.
 * @param temp_err Indicates a temperature error.
 * @param power_supply_failure Indicates a power supply failure.
 * @param boot_memory_failure Indicates a boot memory failure.
 * @param reg_nvm_err Indicates a register NVM error.
 * @param wdg_timer_flag Indicates a watchdog timer flag.
 * @param int_proc_supply_err Indicates an internal processor supply error.
 * @param ext_5v_supply_err Indicates an external 5V supply error.
 * @param int_snsr_supply_err Indicates an internal sensor supply error.
 * @param int_reg_err Indicates an internal regulator error.
 * @param checksum_err Indicates a checksum error.
 * @param fls_mem_wr_cnt_exceed Indicates that the flash memory write count has
 * been exceeded.
 ******************************************************************************/
struct adis_diag_flags {
	/** Sensor initialization failure. */
	uint8_t snsr_init_failure	: 1;
	/** Data path overrun bit. */
	uint8_t data_path_overrun	: 1;
	/** Flash memory update failure. */
	uint8_t fls_mem_update_failure	: 1;
	/** SPI communication error. */
	uint8_t spi_comm_err		: 1;
	/** Standby mode. */
	uint8_t standby_mode		: 1;
	/** Sensor failure. */
	uint8_t snsr_failure		: 1;
	/** Memory failure. */
	uint8_t mem_failure		: 1;
	/** Clock error. */
	uint8_t clk_err			: 1;
	/** Gyroscope 1 failure. */
	uint8_t gyro1_failure		: 1;
	/** Gyroscope 2 failure. */
	uint8_t gyro2_failure		: 1;
	/** Accelerometer failure. */
	uint8_t accl_failure		: 1;
	/** X-Axis gyroscope failure. */
	uint8_t x_axis_gyro_failure	: 2;
	/** Y-Axis gyroscope failure. */
	uint8_t y_axis_gyro_failure	: 2;
	/** Z-Axis gyroscope failure. */
	uint8_t z_axis_gyro_failure	: 2;
	/** X-Axis accelerometer failure. */
	uint8_t x_axis_accl_failure	: 2;
	/** Y-Axis accelerometer failure. */
	uint8_t y_axis_accl_failure	: 2;
	/** Z-Axis accelerometer failure. */
	uint8_t z_axis_accl_failure	: 2;
	/** ADuC microcontroller fault. */
	uint8_t aduc_mcu_fault		: 1;
	/** Configuration and/or calibration CRC error. */
	uint8_t config_calib_crc_error	: 1;
	/** Overrange event occurred. */
	uint8_t overrange		: 1;
	/** Temperature error. */
	uint8_t temp_err		: 1;
	/** Power supply failure. */
	uint8_t power_supply_failure	: 1;
	/** Power supply failure. */
	uint8_t boot_memory_failure	: 1;
	/** Register NVM error. */
	uint8_t reg_nvm_err		: 1;
	/** Watchdog timer flag. */
	uint8_t wdg_timer_flag		: 1;
	/** Internal processor supply error. */
	uint8_t int_proc_supply_err	: 1;
	/** External 5V supply error. */
	uint8_t ext_5v_supply_err	: 1;
	/** Internal sensor supply error. */
	uint8_t int_snsr_supply_err	: 1;
	/** Internal regulator error. */
	uint8_t int_reg_err		: 1;
	/** Checksum error.  */
	uint8_t checksum_err		: 1;
	/** Flash memory write count exceeded. */
	uint8_t fls_mem_wr_cnt_exceed	: 1;
};

/***************************************************************************//**
 * @brief The `adis_temp_flags` structure is a bitfield that represents various
 * temperature flags for accelerometers and gyroscopes across different
 * axes. Each member of the structure is a single bit flag that indicates
 * a temperature condition for a specific combination of axes and sensor
 * types, allowing for efficient storage and retrieval of temperature
 * status information in a compact form.
 *
 * @param accl_temp_z_x Accelerometer temperature flag for z-axis and x-axis.
 * @param accl_temp_y_z Accelerometer temperature flag for y-axis and z-axis.
 * @param accl_temp_x_y Accelerometer temperature flag for x-axis and y-axis.
 * @param gyro2_temp_z Gyroscope2 temperature flag for z-axis.
 * @param gyro1_temp_z Gyroscope1 temperature flag for z-axis.
 * @param gyro2_temp_y Gyroscope2 temperature flag for y-axis.
 * @param gyro1_temp_y Gyroscope1 temperature flag for y-axis.
 * @param gyro2_temp_x Gyroscope2 temperature flag for x-axis.
 * @param gyro1_temp_x Gyroscope1 temperature flag for x-axis.
 ******************************************************************************/
struct adis_temp_flags {
	/** Accelerometer temperature flag for z-axis and x-axis. */
	uint8_t accl_temp_z_x	: 1;
	/** Accelerometer temperature flag for y-axis and z-axis. */
	uint8_t accl_temp_y_z	: 1;
	/** Accelerometer temperature flag for x-axis and y-axis. */
	uint8_t accl_temp_x_y	: 1;
	/** Gyroscope2 temperature flag for z-axis. */
	uint8_t	gyro2_temp_z	: 1;
	/** Gyroscope1 temperature flag for z-axis. */
	uint8_t gyro1_temp_z	: 1;
	/** Gyroscope2 temperature flag for y-axis. */
	uint8_t	gyro2_temp_y	: 1;
	/** Gyroscope1 temperature flag for y-axis. */
	uint8_t gyro1_temp_y	: 1;
	/** Gyroscope2 temperature flag for x-axis. */
	uint8_t	gyro2_temp_x	: 1;
	/** Gyroscope1 temperature flag for x-axis. */
	uint8_t gyro1_temp_x	: 1;
};

/***************************************************************************//**
 * @brief The `adis_scale_fractional` structure is used to represent a
 * fractional scale format where the scale is defined as the ratio of a
 * dividend to a divisor. This structure is typically used in contexts
 * where precise scaling of values is required, such as in sensor data
 * processing, where the scale factor is represented as a fraction to
 * maintain accuracy.
 *
 * @param dividend Scale dividend.
 * @param divisor Scale divisor.
 ******************************************************************************/
struct adis_scale_fractional {
	/** Scale dividend. */
	uint32_t dividend;
	/** Scale divisor. */
	uint32_t divisor;
};

/***************************************************************************//**
 * @brief The `adis_scale_fractional_log2` structure is used to represent a
 * fractional scale in a logarithmic base-2 format, where the scale is
 * calculated as the dividend divided by 2 raised to the power of the
 * `power` member. This structure is useful in scenarios where scaling
 * factors need to be represented in a compact form, particularly when
 * dealing with logarithmic scaling in digital signal processing or
 * sensor data interpretation.
 *
 * @param dividend Scale dividend.
 * @param power Scale 2's power.
 ******************************************************************************/
struct adis_scale_fractional_log2 {
	/** Scale dividend. */
	uint32_t dividend;
	/** Scale 2's power. */
	uint32_t power;
};


/***************************************************************************//**
 * @brief The `adis_burst_data` structure is designed to store burst data from
 * an ADIS sensor, capturing both temperature and motion data. It
 * includes fields for temperature, data counter, gyroscope, and
 * accelerometer readings, each split into lower and upper 16-bit
 * segments to accommodate the full 32-bit data from the sensor. This
 * structure is essential for handling high-speed data acquisition from
 * the sensor, allowing for efficient processing and analysis of the
 * sensor's output.
 *
 * @param temp_lsb Lower 16 bits of the temperature data.
 * @param temp_msb Upper 16 bits of the temperature data.
 * @param data_cntr_lsb Lower 16 bits of the data counter.
 * @param data_cntr_msb Upper 16 bits of the data counter.
 * @param x_gyro_lsb Lower 16 bits of the X-axis gyroscope data.
 * @param x_gyro_msb Upper 16 bits of the X-axis gyroscope data.
 * @param y_gyro_lsb Lower 16 bits of the Y-axis gyroscope data.
 * @param y_gyro_msb Upper 16 bits of the Y-axis gyroscope data.
 * @param z_gyro_lsb Lower 16 bits of the Z-axis gyroscope data.
 * @param z_gyro_msb Upper 16 bits of the Z-axis gyroscope data.
 * @param x_accel_lsb Lower 16 bits of the X-axis accelerometer data.
 * @param x_accel_msb Upper 16 bits of the X-axis accelerometer data.
 * @param y_accel_lsb Lower 16 bits of the Y-axis accelerometer data.
 * @param y_accel_msb Upper 16 bits of the Y-axis accelerometer data.
 * @param z_accel_lsb Lower 16 bits of the Z-axis accelerometer data.
 * @param z_accel_msb Upper 16 bits of the Z-axis accelerometer data.
 ******************************************************************************/
struct adis_burst_data {
	uint16_t temp_lsb;
	uint16_t temp_msb;
	uint16_t data_cntr_lsb;
	uint16_t data_cntr_msb;
	uint16_t x_gyro_lsb;
	uint16_t x_gyro_msb;
	uint16_t y_gyro_lsb;
	uint16_t y_gyro_msb;
	uint16_t z_gyro_lsb;
	uint16_t z_gyro_msb;
	uint16_t x_accel_lsb;
	uint16_t x_accel_msb;
	uint16_t y_accel_lsb;
	uint16_t y_accel_msb;
	uint16_t z_accel_lsb;
	uint16_t z_accel_msb;
};

/***************************************************************************//**
 * @brief The `adis_dev` structure is a comprehensive descriptor for an ADIS
 * device, encapsulating all necessary components for device
 * communication and configuration. It includes descriptors for SPI and
 * GPIO interfaces, chip-specific information, and diagnostic and
 * temperature flags. The structure also maintains the current device ID,
 * register map page, and buffers for SPI transactions. Additionally, it
 * holds configuration flags for FIFO and burst modes, clock frequencies,
 * and a lock status to restrict device configuration. This structure is
 * essential for managing and interacting with ADIS devices, providing a
 * centralized representation of the device's state and settings.
 *
 * @param spi_desc SPI descriptor used for SPI communication.
 * @param gpio_reset GPIO descriptor used to handle the reset pin.
 * @param info Specific chip information.
 * @param diag_flags Current diagnosis flags values.
 * @param temp_flags Current temperature flags values.
 * @param dev_id Current device id, specified by the user.
 * @param current_page Current page to be accessed in register map.
 * @param tx Transmit buffer used in SPI transactions.
 * @param rx Receive buffer used in SPI transactions.
 * @param int_clk Internal clock frequency in Hertz.
 * @param ext_clk External clock frequency in Hertz.
 * @param fifo_enabled Set to true if device fifo is enabled.
 * @param burst32 Set to true if device burst32 is enabled.
 * @param burst_sel Burst data selection: 0 for accel/gyro data; 1 for delta
 * angle/ delta velocity data.
 * @param is_locked Device is locked, only data readings are allowed, no
 * configuration allowed.
 ******************************************************************************/
struct adis_dev {
	/** SPI descriptor used for SPI communication. */
	struct no_os_spi_desc		*spi_desc;
	/** GPIO descriptor used to handle the reset pin. */
	struct no_os_gpio_desc		*gpio_reset;
	/** Specific chip information. */
	const struct adis_chip_info  	*info;
	/** Current diagnosis flags values. */
	struct adis_diag_flags 		diag_flags;
	/** Current temperature flags values. */
	struct adis_temp_flags 		temp_flags;
	/** Current device id, specified by the user */
	enum adis_device_id		dev_id;
	/** Current page to be accessed in register map. */
	uint32_t			current_page;
	/** Transmit buffer used in SPI transactions. */
	uint8_t				tx[12];
	/** Receive buffer used in SPI transactions. */
	uint8_t				rx[8];
	/** Internal clock frequency in Hertz. */
	uint32_t 			int_clk;
	/** External clock frequency in Hertz. */
	uint32_t 			ext_clk;
	/** Set to true if device fifo is enabled. */
	bool				fifo_enabled;
	/** Set to true if device burst32 is enabled. */
	bool				burst32;
	/** Burst data selection: 0 for accel/gyro data; 1 for delta angle/ delta velocity data. */
	uint8_t				burst_sel;
	/** Device is locked, only data readings are allowed, no configuration allowed. */
	bool				is_locked;
};

/***************************************************************************//**
 * @brief The `adis_init_param` structure is used to define the initialization
 * parameters for an ADIS device. It includes pointers to chip-specific
 * information, SPI and GPIO initialization parameters, and configuration
 * settings such as external clock frequency and synchronization mode.
 * Additionally, it allows the user to specify the device ID,
 * facilitating the setup and configuration of the ADIS device during the
 * initialization phase.
 *
 * @param info Pointer to chip-specific information.
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_reset Pointer to GPIO initialization parameter for the reset pin.
 * @param ext_clk External clock frequency in Hertz for initialization.
 * @param sync_mode Desired synchronization mode for initialization.
 * @param dev_id Device ID specified by the user.
 ******************************************************************************/
struct adis_init_param {
	/* Chip specific information. */
	const struct adis_chip_info *info;
	/** SPI initialization parameters. */
	struct no_os_spi_init_param 	*spi_init;
	/** GPIO initialization parameter for reset pin. */
	struct no_os_gpio_init_param	*gpio_reset;
	/** External clock frequency in Hertz to be configured at initialization
	 *  phase.
	 */
	uint32_t 			ext_clk;
	/** Desired synchronization mode to be configured at initialization
	 *  phase.
	 */
	uint32_t			sync_mode;
	/** Device id, specified by the user  */
	enum adis_device_id		dev_id;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to initialize an ADIS device by allocating
 * necessary resources and configuring its parameters. It must be called
 * before any other operations on the device can be performed. The
 * initialization requires a valid `adis_init_param` structure, which
 * contains essential information such as the device ID, SPI
 * initialization parameters, GPIO reset pin configuration, external
 * clock frequency, and synchronization mode. If any of the required
 * parameters are invalid or if resource allocation fails, the function
 * will return an error code. It is important to ensure that the `adis`
 * pointer is not null and that the device is properly configured before
 * calling this function.
 *
 * @param adis A pointer to a pointer of `struct adis_dev`. This will be set to
 * point to the initialized device structure. Must not be null.
 * @param ip A pointer to an `adis_init_param` structure containing
 * initialization parameters. Must not be null and must contain valid
 * information, including a non-null `info` field.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure (e.g., -EINVAL for invalid parameters, -ENOMEM for memory
 * allocation failure).
 ******************************************************************************/
int adis_init(struct adis_dev **adis, const struct adis_init_param *ip);
/***************************************************************************//**
 * @brief This function is used to safely remove an ADIS device and free
 * associated resources. It should be called when the device is no longer
 * needed, typically during cleanup or shutdown procedures. The function
 * checks if the provided device pointer is valid before attempting to
 * remove the GPIO and SPI descriptors associated with the device. If the
 * device pointer is null, the function will simply return without
 * performing any actions.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device
 * to be removed. This pointer must not be null; if it is null, the
 * function will return immediately without any action. The caller
 * is responsible for ensuring that the device is no longer in use
 * before calling this function.
 * @return This function does not return a value and does not modify any input
 * parameters.
 ******************************************************************************/
void adis_remove(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is intended to be called after the ADIS device has been
 * initialized. It performs a series of startup operations, including
 * resetting the device and running a self-test to ensure proper
 * functionality. If a GPIO reset pin is configured, it will set the pin
 * high for a specified duration before proceeding. The function will
 * return an error code if any of the operations fail, allowing the
 * caller to handle initialization issues appropriately.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device to be
 * initialized. This pointer must not be null and should point to a
 * valid device structure that has been properly initialized.
 * @return Returns an integer indicating the status of the startup process. A
 * return value of 0 indicates success, while a negative value indicates
 * an error occurred during the startup sequence.
 ******************************************************************************/
int adis_initial_startup(struct adis_dev *adis);

/***************************************************************************//**
 * @brief This function is used to read a specified number of bytes from a
 * register of the ADIS device. It must be called after the device has
 * been initialized. The `reg` parameter specifies the register address
 * to read from, while `val` is a pointer to a variable where the read
 * value will be stored. The `size` parameter determines how many bytes
 * to read, which can be either 2 or 4 bytes. If the specified size is
 * invalid, the function will return an error code. Additionally, if the
 * register belongs to a different page than the currently selected page,
 * the function will automatically switch to the correct page before
 * reading.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param reg The register address to read from. Must be a valid register
 * address.
 * @param val Pointer to a variable where the read value will be stored. Must
 * not be null.
 * @param size The number of bytes to read from the register. Valid values are 2
 * or 4. If an invalid size is provided, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_reg(struct adis_dev *adis,  uint32_t reg, uint32_t *val,
		  uint32_t size);
/***************************************************************************//**
 * @brief This function is used to write a specified value to a register of the
 * ADIS device. It should be called after the device has been initialized
 * and is not locked, except when performing a software reset. The
 * function checks if the device is locked; if it is, writing to any
 * register other than the software reset register is prohibited. The
 * size of the data being written can be specified, and it must be one of
 * the predefined sizes (1, 2, or 4 bytes). If an invalid size is
 * provided, the function will return an error. The function also handles
 * page changes automatically if the target register belongs to a
 * different page than the currently selected one.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param reg The address of the register to write to. Valid register addresses
 * depend on the specific device.
 * @param value The value to write to the register. The value must be
 * appropriate for the register being written.
 * @param size The number of bytes to write (1, 2, or 4). Must be one of the
 * predefined sizes; otherwise, an error is returned.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int adis_write_reg(struct adis_dev *adis, uint32_t reg, uint32_t value,
		   uint32_t size);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * device. It should be called after the device has been properly
 * initialized. The function reads the current value of the specified
 * register, applies the provided mask and value to update the desired
 * bits, and then writes the modified value back to the register. If the
 * read operation fails, the function will return an error code. It is
 * important to ensure that the size parameter matches the expected size
 * of the register being accessed.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param reg The register address to be updated. Valid register addresses
 * depend on the specific device.
 * @param mask A bitmask indicating which bits to update. Must be a valid mask
 * for the specified register.
 * @param val The new value to set for the bits indicated by the mask. Must be
 * compatible with the mask.
 * @param size The size of the register in bytes. Valid values are typically 1,
 * 2, or 4.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_update_bits_base(struct adis_dev *adis, uint32_t reg,
			  const uint32_t mask, const uint32_t val, uint8_t size);

/***************************************************************************//**
 * @brief This function is intended to be called to retrieve the current
 * diagnostic status of the device, which includes various error flags
 * related to sensor operation. It should be invoked after the device has
 * been initialized and is ready for operation. The function reads the
 * diagnostic status from the device's register and updates the provided
 * `diag_flags` structure with the current status. If the read operation
 * fails, an error code is returned, and the `diag_flags` structure
 * remains unchanged.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param diag_flags Pointer to an `adis_diag_flags` structure where the
 * diagnostic flags will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_stat(struct adis_dev *adis,
			struct adis_diag_flags *diag_flags);

/***************************************************************************//**
 * @brief This function retrieves the temperature flags from the specified
 * device and updates the provided `temp_flags` structure with the
 * results. It should be called after the device has been properly
 * initialized. If the device is not ready or if there is an error
 * reading the register, the function will return an error code. The
 * caller must ensure that the `temp_flags` pointer is valid and points
 * to a properly allocated `adis_temp_flags` structure.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param temp_flags Pointer to an `adis_temp_flags` structure where the
 * temperature flags will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_temp_flags(struct adis_dev *adis,
			 struct adis_temp_flags *temp_flags);

/***************************************************************************//**
 * @brief This function is used to read the sensor initialization failure status
 * from the device's diagnostic flags. It should be called after the
 * device has been initialized and is ready for operation. The function
 * will populate the provided pointer with the value of the sensor
 * initialization failure flag, which indicates whether the sensor has
 * encountered an initialization failure. If the device is not properly
 * initialized or if there is an error reading the diagnostic status, the
 * function will return a negative error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param snsr_init_failure Pointer to a `uint32_t` where the sensor
 * initialization failure status will be stored. Caller
 * retains ownership and must ensure it points to a
 * valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_snsr_init_failure(struct adis_dev *adis,
				     uint32_t *snsr_init_failure);
/***************************************************************************//**
 * @brief This function is used to read the data path overrun error flag from
 * the diagnostic status of the device. It should be called after the
 * device has been initialized and is operational. The function retrieves
 * the current state of the data path overrun error and stores it in the
 * provided pointer. If the device is not properly initialized or if
 * there is an error reading the diagnostic status, the function will
 * return an error code. It is important to ensure that the pointer
 * provided for the error flag is valid and not null.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param data_path_overrun_err Pointer to a `uint32_t` where the data path
 * overrun error flag will be stored. Must not be
 * null.
 * @return Returns 0 on success, indicating that the data path overrun error
 * flag has been successfully read and stored. On failure, it returns a
 * negative error code.
 ******************************************************************************/
int adis_read_diag_data_path_overrun(struct adis_dev *adis,
				     uint32_t *data_path_overrun_err);
/***************************************************************************//**
 * @brief This function is used to retrieve the status of the flash memory
 * update failure diagnostic flag from the device. It should be called
 * after the device has been properly initialized. The function will read
 * the diagnostic status and store the result in the provided pointer. If
 * the device is not initialized or if there is an error reading the
 * diagnostic status, the function will return an error code. It is
 * important to ensure that the pointer provided for the output is valid
 * and points to a memory location that can store the result.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param fls_mem_update_failure Pointer to a `uint32_t` where the result will
 * be stored. Must not be null; the function will
 * write the diagnostic flag value to this
 * location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_fls_mem_update_failure(struct adis_dev *adis,
		uint32_t *fls_mem_update_failure);
/***************************************************************************//**
 * @brief This function is used to read the SPI communication error flag from
 * the device's diagnostic status. It should be called after the device
 * has been initialized and is ready for operation. The function
 * retrieves the current state of the SPI communication error flag and
 * stores it in the provided pointer. If the device is not properly
 * initialized or if there is an error reading the diagnostic status, the
 * function will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param spi_comm_err Pointer to a `uint32_t` where the SPI communication error
 * flag will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_spi_comm_err(struct adis_dev *adis, uint32_t *spi_comm_err);
/***************************************************************************//**
 * @brief This function is used to retrieve the standby mode status of the
 * device. It should be called after the device has been properly
 * initialized. The function reads the diagnostic status register and
 * updates the provided pointer with the standby mode value. If the read
 * operation fails, an error code is returned, and the output parameter
 * is not modified.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param standby_mode Pointer to a `uint32_t` where the standby mode value will
 * be stored. Caller retains ownership and must ensure it is
 * not null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_standby_mode(struct adis_dev *adis,
				uint32_t *standby_mode);
/***************************************************************************//**
 * @brief This function is used to retrieve the sensor failure diagnostic flag
 * from the device. It should be called after the device has been
 * properly initialized and configured. The function will read the
 * diagnostic status and update the provided pointer with the sensor
 * failure flag value. If the read operation fails, an error code will be
 * returned, and the output parameter will remain unchanged.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param snsr_failure Pointer to a `uint32_t` where the sensor failure flag
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_diag_snsr_failure(struct adis_dev *adis, uint32_t *snsr_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the memory failure diagnostic flag
 * from the device. It should be called after the device has been
 * properly initialized and configured. The function will read the
 * diagnostic status and update the provided pointer with the memory
 * failure flag value. If the device is not ready or if there is an error
 * reading the diagnostic status, the function will return an error code.
 * It is important to ensure that the pointer provided for the
 * `mem_failure` parameter is valid and points to a writable memory
 * location.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param mem_failure Pointer to a `uint32_t` where the memory failure flag will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_mem_failure(struct adis_dev *adis, uint32_t *mem_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the clock error diagnostic flag from
 * the device. It should be called after the device has been properly
 * initialized. The function will read the diagnostic status and update
 * the provided pointer with the clock error flag value. If the device is
 * not initialized or if there is an error reading the diagnostic status,
 * the function will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to a valid initialized device.
 * @param clk_err Pointer to a `uint32_t` where the clock error flag will be
 * stored. Must not be null. The function will write the clock
 * error status to this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_clk_err(struct adis_dev *adis, uint32_t *clk_err);
/***************************************************************************//**
 * @brief This function is used to retrieve the gyroscope 1 failure diagnostic
 * flag from the device. It should be called after the device has been
 * properly initialized and configured. The function will read the
 * diagnostic status and update the provided pointer with the value of
 * the gyroscope 1 failure flag. If the device is not ready or if there
 * is an error in reading the diagnostic status, the function will return
 * an error code. It is important to ensure that the pointer provided for
 * the `gyro1_failure` parameter is valid and points to a memory location
 * that can store the result.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param gyro1_failure Pointer to a `uint32_t` where the gyroscope 1 failure
 * flag will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_gyro1_failure(struct adis_dev *adis,
				 uint32_t *gyro1_failure);
/***************************************************************************//**
 * @brief This function retrieves the gyroscope 2 failure status from the
 * diagnostic flags of the specified ADIS device. It should be called
 * after the device has been initialized and is ready for operation. The
 * function will populate the provided pointer with the gyroscope 2
 * failure status, which indicates whether a failure has occurred. If the
 * device is not properly initialized or if there is an error reading the
 * diagnostic status, the function will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param gyro2_failure Pointer to a `uint32_t` where the gyroscope 2 failure
 * status will be stored. Caller retains ownership and must
 * ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_gyro2_failure(struct adis_dev *adis,
				 uint32_t *gyro2_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the accelerometer failure status
 * from the device's diagnostic flags. It should be called after the
 * device has been properly initialized and configured. The function will
 * populate the provided pointer with the value of the accelerometer
 * failure flag, which indicates whether a failure has occurred. If the
 * device is not ready or if there is an error in reading the diagnostic
 * status, the function will return a negative error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param accl_failure Pointer to a `uint32_t` where the accelerometer failure
 * status will be stored. Must not be null. The value will
 * be set to 1 if a failure is detected, or 0 if there is no
 * failure.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_accl_failure(struct adis_dev *adis, uint32_t *accl_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the failure status of the X-axis
 * gyroscope from the device's diagnostic flags. It should be called
 * after the device has been initialized and is ready for operation. The
 * function will read the diagnostic status and update the provided
 * pointer with the failure status. If the device is not properly
 * initialized or if there is an error reading the diagnostic status, the
 * function will return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param x_axis_gyro_failure Pointer to a `uint32_t` where the X-axis gyroscope
 * failure status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_x_axis_gyro_failure(struct adis_dev *adis,
				       uint32_t *x_axis_gyro_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the failure status of the Y-axis
 * gyroscope from the device's diagnostic flags. It should be called
 * after the device has been properly initialized and configured. The
 * function will read the diagnostic status and update the provided
 * pointer with the Y-axis gyroscope failure value. If the device is not
 * ready or an error occurs during the read operation, the function will
 * return an error code, allowing the caller to handle the situation
 * appropriately.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param y_axis_gyro_failure Pointer to a `uint32_t` where the Y-axis gyroscope
 * failure status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_y_axis_gyro_failure(struct adis_dev *adis,
				       uint32_t *y_axis_gyro_failure);
/***************************************************************************//**
 * @brief This function retrieves the failure status of the Z-axis gyroscope
 * from the device's diagnostic flags. It should be called after the
 * device has been initialized and is ready for operation. The function
 * will update the value pointed to by the `z_axis_gyro_failure`
 * parameter with the current failure status. If the device is not
 * properly initialized or if there is an error reading the diagnostic
 * status, the function will return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param z_axis_gyro_failure Pointer to a `uint32_t` variable where the Z-axis
 * gyroscope failure status will be stored. Caller
 * retains ownership and must ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_z_axis_gyro_failure(struct adis_dev *adis,
				       uint32_t *z_axis_gyro_failure);
/***************************************************************************//**
 * @brief This function retrieves the failure status of the X-axis accelerometer
 * from the device's diagnostic flags. It should be called after the
 * device has been properly initialized and configured. The function will
 * update the value pointed to by the `x_axis_accl_failure` parameter
 * with the current failure status. If the device is not ready or if
 * there is an error reading the diagnostic status, the function will
 * return a negative error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param x_axis_accl_failure Pointer to a `uint32_t` where the X-axis
 * accelerometer failure status will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_x_axis_accl_failure(struct adis_dev *adis,
				       uint32_t *x_axis_accl_failure);
/***************************************************************************//**
 * @brief This function retrieves the failure status of the Y-axis accelerometer
 * from the device's diagnostic flags. It should be called after the
 * device has been properly initialized. The function will update the
 * value pointed to by the `y_axis_accl_failure` parameter with the
 * current status. If the device is not ready or an error occurs while
 * reading the diagnostic status, the function will return a negative
 * error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param y_axis_accl_failure Pointer to a `uint32_t` where the Y-axis
 * accelerometer failure status will be stored.
 * Caller retains ownership and must ensure it is not
 * null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_y_axis_accl_failure(struct adis_dev *adis,
				       uint32_t *y_axis_accl_failure);
/***************************************************************************//**
 * @brief This function retrieves the failure status of the Z-axis accelerometer
 * from the device's diagnostic flags. It should be called after the
 * device has been properly initialized and configured. The function will
 * update the value pointed to by the `z_axis_accl_failure` parameter
 * with the current failure status. If the device is not ready or an
 * error occurs while reading the diagnostic status, the function will
 * return an error code, allowing the caller to handle it appropriately.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param z_axis_accl_failure A pointer to a `uint32_t` variable where the
 * Z-axis accelerometer failure status will be
 * stored. Caller retains ownership and must ensure
 * it is not null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_z_axis_accl_failure(struct adis_dev *adis,
				       uint32_t *z_axis_accl_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the fault status of the ADuC
 * microcontroller from the device's diagnostic flags. It should be
 * called after the device has been properly initialized. The function
 * will read the diagnostic status and update the provided pointer with
 * the fault status. If the device is not initialized or if there is an
 * error reading the diagnostic status, the function will return an error
 * code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param aduc_mcu_fault Pointer to a `uint32_t` where the fault status will be
 * stored. Must not be null. The function will write the
 * fault status to this location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_aduc_mcu_fault(struct adis_dev *adis,
				  uint32_t *aduc_mcu_fault);
/***************************************************************************//**
 * @brief This function is used to retrieve the configuration and calibration
 * CRC error status from the device's diagnostic flags. It should be
 * called after the device has been initialized and is ready for
 * operation. The function will read the diagnostic status and update the
 * provided pointer with the CRC error flag value. If the device is not
 * properly initialized or if there is an error reading the diagnostic
 * status, the function will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param config_calib_crc_error Pointer to a `uint32_t` where the CRC error
 * flag value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_config_calib_crc_error(struct adis_dev *adis,
		uint32_t *config_calib_crc_error);
/***************************************************************************//**
 * @brief This function is used to retrieve the overrange diagnostic flag from
 * the device. It should be called after the device has been properly
 * initialized and configured. The function will read the diagnostic
 * status and update the provided pointer with the overrange flag value.
 * If the read operation fails, an error code will be returned, and the
 * value pointed to by the `overrange` parameter will remain unchanged.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param overrange Pointer to a `uint32_t` where the overrange flag value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_overrange(struct adis_dev *adis,
			     uint32_t *overrange);
/***************************************************************************//**
 * @brief This function is used to retrieve the temperature error diagnostic
 * flag from the device. It should be called after the device has been
 * properly initialized. The function will read the diagnostic status and
 * update the provided pointer with the value of the temperature error
 * flag. If the device is not ready or if there is an error reading the
 * diagnostic status, the function will return an error code. Ensure that
 * the pointer provided for the temperature error flag is valid and
 * points to a writable memory location.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param temp_err Pointer to a `uint32_t` where the temperature error flag will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_temp_err(struct adis_dev *adis,
			    uint32_t *temp_err);
/***************************************************************************//**
 * @brief This function is used to read the power supply failure diagnostic flag
 * from the device. It should be called after the device has been
 * initialized and is ready for operation. The function retrieves the
 * current state of the power supply failure flag and stores it in the
 * provided pointer. If the device is not properly initialized or if
 * there is an error reading the diagnostic status, the function will
 * return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param power_supply_failure Pointer to a `uint32_t` where the power supply
 * failure flag will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_power_supply_failure(struct adis_dev *adis,
					uint32_t *power_supply_failure);
/***************************************************************************//**
 * @brief This function is used to retrieve the boot memory failure status from
 * the device's diagnostic flags. It should be called after the device
 * has been initialized and is ready for operation. The function will
 * populate the provided pointer with the boot memory failure status,
 * which indicates whether a failure has occurred during the boot
 * process. If the device is not properly initialized or if there is an
 * error reading the diagnostic status, the function will return a
 * negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param boot_memory_failure Pointer to a `uint32_t` where the boot memory
 * failure status will be stored. Caller retains
 * ownership and must ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_diag_boot_memory_failure(struct adis_dev *adis,
				       uint32_t *boot_memory_failure);
/***************************************************************************//**
 * @brief This function retrieves the value of the register NVM error flag from
 * the device's diagnostic status. It should be called after the device
 * has been properly initialized and configured. The function will read
 * the diagnostic status and update the provided pointer with the value
 * of the register NVM error flag. If the read operation fails, an error
 * code will be returned, indicating the failure reason.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param reg_nvm_err Pointer to a `uint32_t` where the register NVM error value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_diag_reg_nvm_err(struct adis_dev *adis,
			       uint32_t *reg_nvm_err);
/***************************************************************************//**
 * @brief This function retrieves the watchdog timer flag from the device's
 * diagnostic status. It should be called after the device has been
 * properly initialized and is ready for operation. The function will
 * read the diagnostic status and update the provided pointer with the
 * value of the watchdog timer flag. If the read operation fails, an
 * error code will be returned, indicating the type of failure
 * encountered.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param wdg_timer_flag Pointer to a `uint32_t` where the watchdog timer flag
 * value will be stored. Caller retains ownership and must
 * ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_wdg_timer_flag(struct adis_dev *adis,
				  uint32_t *wdg_timer_flag);
/***************************************************************************//**
 * @brief This function is used to retrieve the internal processor supply error
 * flag from the device's diagnostic status. It should be called after
 * the device has been initialized and is ready for operation. The
 * function will read the diagnostic status and update the provided
 * pointer with the value of the internal processor supply error flag. If
 * the read operation fails, an error code will be returned, indicating
 * the failure reason.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param int_proc_supply_err Pointer to a `uint32_t` where the internal
 * processor supply error flag value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_diag_int_proc_supply_err(struct adis_dev *adis,
				       uint32_t *int_proc_supply_err);
/***************************************************************************//**
 * @brief This function is used to read the external 5V supply error flag from
 * the device's diagnostic status. It should be called after the device
 * has been initialized and is ready for operation. The function
 * retrieves the error status and stores it in the provided pointer. If
 * the device is not properly initialized or if there is an error reading
 * the diagnostic status, the function will return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param ext_5v_supply_err Pointer to a `uint32_t` where the external 5V supply
 * error flag will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_ext_5v_supply_err(struct adis_dev *adis,
				     uint32_t *ext_5v_supply_err);
/***************************************************************************//**
 * @brief This function is used to read the internal sensor supply error flag
 * from the device's diagnostic status. It should be called after the
 * device has been initialized and is ready for operation. The function
 * retrieves the value of the internal sensor supply error flag and
 * stores it in the provided output parameter. If the device is not
 * properly initialized or if there is an error reading the diagnostic
 * status, the function will return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device.
 * @param int_snsr_supply_err A pointer to a `uint32_t` variable where the
 * internal sensor supply error flag value will be
 * stored. This pointer must not be null. The
 * function will write the flag value to this
 * variable.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_int_snsr_supply_err(struct adis_dev *adis,
				       uint32_t *int_snsr_supply_err);
/***************************************************************************//**
 * @brief This function is used to retrieve the internal register error flags
 * from the device's diagnostic status. It should be called after the
 * device has been initialized and is ready for operation. The function
 * will read the diagnostic status and populate the `int_reg_err` pointer
 * with the corresponding error flags. If the read operation fails, an
 * error code will be returned, allowing the caller to handle the error
 * appropriately.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param int_reg_err Pointer to a `uint32_t` where the internal register error
 * flags will be stored. Caller retains ownership and must
 * ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_diag_int_reg_err(struct adis_dev *adis,
			       uint32_t *int_reg_err);
/***************************************************************************//**
 * @brief This function retrieves the current value of the checksum error flag
 * from the device's diagnostic flags. It should be called after the
 * device has been initialized and is operational. The function directly
 * writes the retrieved value into the provided pointer, allowing the
 * caller to check for any checksum errors that may have occurred during
 * operation.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device
 * instance. Must not be null and should point to a valid
 * initialized device.
 * @param checksum_err A pointer to a `uint32_t` variable where the checksum
 * error flag value will be stored. Must not be null; the
 * function will write the checksum error value to this
 * location.
 * @return The function does not return a value. Instead, it writes the checksum
 * error flag value directly to the location pointed to by the
 * `checksum_err` parameter.
 ******************************************************************************/
void adis_read_diag_checksum_err(struct adis_dev *adis, uint32_t *checksum_err);
/***************************************************************************//**
 * @brief This function retrieves the status of the flash memory write count
 * exceeded diagnostic flag from the specified device. It should be
 * called after the device has been properly initialized and configured.
 * The function will write the value of the diagnostic flag to the
 * provided pointer. If the pointer is null, the behavior is undefined,
 * and the function may not execute correctly.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param fls_mem_wr_cnt_exceed A pointer to a `uint32_t` variable where the
 * function will store the value of the flash
 * memory write count exceeded flag. Must not be
 * null.
 * @return The function does not return a value. It writes the status of the
 * flash memory write count exceeded flag to the variable pointed to by
 * `fls_mem_wr_cnt_exceed`.
 ******************************************************************************/
void adis_read_diag_fls_mem_wr_cnt_exceed(struct adis_dev *adis,
		uint32_t *fls_mem_wr_cnt_exceed);

/***************************************************************************//**
 * @brief This function is used to retrieve the raw gyroscope data for the
 * x-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. If the FIFO mode
 * is enabled, it will first disable it before reading the data. The
 * function will return an error code if the read operation fails,
 * allowing the caller to handle any issues that may arise during the
 * data retrieval.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device
 * from which to read the gyroscope data. This pointer must not be
 * null and should point to a valid initialized device.
 * @param x_gyro A pointer to an `int32_t` variable where the read gyroscope
 * data will be stored. This pointer must not be null; otherwise,
 * the function will not perform the read operation.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_x_gyro(struct adis_dev *adis, int32_t *x_gyro);
/***************************************************************************//**
 * @brief This function is used to retrieve the raw gyroscope data specifically
 * for the Y axis from the ADIS device. It should be called after the
 * device has been properly initialized and configured. If the FIFO
 * (First In, First Out) feature is enabled, it will be disabled before
 * reading the data. The function will return an error code if the read
 * operation fails, allowing the caller to handle any issues that arise
 * during the data retrieval process.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the Y gyroscope data. This pointer must not be null
 * and should point to a valid initialized device.
 * @param y_gyro A pointer to an `int32_t` variable where the read Y gyroscope
 * data will be stored. This pointer must not be null, and the
 * caller retains ownership of the memory it points to.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_y_gyro(struct adis_dev *adis, int32_t *y_gyro);
/***************************************************************************//**
 * @brief This function is used to retrieve the raw gyroscope data specifically
 * for the z-axis from the ADIS device. It should be called after the
 * device has been properly initialized and configured. If the device's
 * FIFO is enabled, it will be disabled before reading the z-gyro data.
 * The function will return an error code if the read operation fails,
 * allowing the caller to handle any issues that may arise during the
 * data retrieval process.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the z-gyro data. This pointer must not be null and
 * should point to a valid initialized device.
 * @param z_gyro A pointer to an `int32_t` variable where the read z-gyro data
 * will be stored. This pointer must not be null, and the caller
 * retains ownership of the memory it points to.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_z_gyro(struct adis_dev *adis, int32_t *z_gyro);

/***************************************************************************//**
 * @brief This function is used to retrieve the raw acceleration data along the
 * x-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. If the device's
 * FIFO is enabled, it will be temporarily disabled before reading the
 * data. The function will return an error code if the read operation
 * fails, allowing the caller to handle any issues that may arise.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the acceleration data. This pointer must not be
 * null and should point to a valid initialized device.
 * @param x_accl A pointer to an `int32_t` variable where the read acceleration
 * data will be stored. This pointer must not be null; otherwise,
 * the function will not perform the read operation.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_x_accl(struct adis_dev *adis, int32_t *x_accl);
/***************************************************************************//**
 * @brief This function is used to retrieve the raw acceleration data along the
 * Y-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. If the device's
 * FIFO is enabled, it will be disabled before reading the data. The
 * function will return an error code if the read operation fails,
 * allowing the caller to handle any issues that may arise.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the Y-axis acceleration data. This pointer must not
 * be null and should point to a valid initialized device.
 * @param y_accl A pointer to an `int32_t` variable where the read Y-axis
 * acceleration data will be stored. This pointer must not be
 * null; the function will write the retrieved data to the
 * location it points to.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_y_accl(struct adis_dev *adis, int32_t *y_accl);
/***************************************************************************//**
 * @brief This function is used to retrieve the raw acceleration data along the
 * Z-axis from the specified device. It should be called after the device
 * has been properly initialized. If the FIFO mode is enabled, it will
 * first disable it before reading the data. The function will return an
 * error code if the read operation fails, allowing the caller to handle
 * any issues that may arise during the data retrieval.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the Z-axis acceleration. This pointer must not be
 * null and should point to a valid initialized device.
 * @param z_accl A pointer to an `int32_t` variable where the read Z-axis
 * acceleration value will be stored. This pointer must not be
 * null; otherwise, the function will not perform the read
 * operation.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_z_accl(struct adis_dev *adis, int32_t *z_accl);

/***************************************************************************//**
 * @brief This function is used to retrieve the temperature output from the
 * specified device. It should be called after the device has been
 * properly initialized. If the device's FIFO is enabled, it will be
 * disabled before reading the temperature. The function will return an
 * error code if the read operation fails, and the temperature value will
 * be stored in the provided output parameter.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param temp_out Pointer to an `int32_t` where the temperature output will be
 * stored. Caller retains ownership and must ensure it is valid.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_temp_out(struct adis_dev *adis, int32_t *temp_out);

/***************************************************************************//**
 * @brief This function is used to retrieve the current time stamp from the
 * device, which is essential for synchronizing data readings. It should
 * be called after the device has been properly initialized and
 * configured. The function expects a valid pointer to a `uint32_t`
 * variable where the time stamp will be stored. If the provided pointer
 * is null or if the device is not ready, the function may return an
 * error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid initialized device.
 * @param time_stamp A pointer to a `uint32_t` variable where the time stamp
 * will be stored. This pointer must not be null; otherwise,
 * the function will not be able to write the time stamp
 * value.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_time_stamp(struct adis_dev *adis, uint32_t *time_stamp);
/***************************************************************************//**
 * @brief This function retrieves the current data counter value from the
 * specified ADIS device. It should be called after the device has been
 * properly initialized and configured. The data counter is typically
 * used to track the number of data samples collected by the device. If
 * the provided pointer for the data counter is null, the function will
 * not perform the read operation and may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the data counter. This pointer must not be null and
 * should point to a valid initialized device.
 * @param data_cntr A pointer to a `uint32_t` variable where the read data
 * counter value will be stored. This pointer must not be null;
 * otherwise, the function will not perform the read operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_data_cntr(struct adis_dev *adis, uint32_t *data_cntr);

/***************************************************************************//**
 * @brief This function is used to retrieve the delta angle data for the x-axis
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized. If the device's FIFO is enabled, it
 * will be disabled before reading the data. The function will return an
 * error code if the read operation fails, which can occur due to various
 * reasons such as communication issues with the device.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the delta angle data. This pointer must not be null
 * and should point to a valid initialized device.
 * @param x_deltang A pointer to an `int32_t` variable where the read delta
 * angle data will be stored. This pointer must not be null;
 * the function will write the retrieved data to the location
 * it points to.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_x_deltang(struct adis_dev *adis, int32_t *x_deltang);
/***************************************************************************//**
 * @brief This function is used to retrieve the delta angle measurement on the
 * Y-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. If the FIFO
 * (First In, First Out) feature is enabled on the device, it will be
 * disabled temporarily during the read operation to ensure accurate data
 * retrieval. The function will return an error code if the read
 * operation fails, allowing the caller to handle any issues that may
 * arise.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the delta angle. This pointer must not be null and
 * should point to a valid initialized device.
 * @param y_deltang A pointer to an `int32_t` variable where the read delta
 * angle value will be stored. This pointer must not be null;
 * the function will write the retrieved value to this
 * location.
 * @return Returns an integer indicating the success or failure of the read
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_y_deltang(struct adis_dev *adis, int32_t *y_deltang);
/***************************************************************************//**
 * @brief This function is used to retrieve the delta angle measurement on the
 * z-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. If the FIFO
 * feature is enabled on the device, it will be disabled before reading
 * the data. The function will return an error code if the read operation
 * fails, which can occur if the device is not ready or if there are
 * communication issues.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param z_deltang Pointer to an `int32_t` variable where the read delta angle
 * value will be stored. Must not be null; the function will
 * not write to this pointer if it is null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_z_deltang(struct adis_dev *adis, int32_t *z_deltang);

/***************************************************************************//**
 * @brief This function is used to retrieve the delta velocity data for the
 * x-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized. If the device's FIFO is enabled,
 * it will be disabled before reading the data. The function will return
 * an error code if the read operation fails, allowing the caller to
 * handle any issues that arise during the data retrieval.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the delta velocity data. This pointer must not be
 * null and should point to a valid initialized device.
 * @param x_deltvel A pointer to an `int32_t` variable where the read delta
 * velocity data will be stored. This pointer must not be null;
 * otherwise, the function will not be able to store the
 * result.
 * @return Returns an integer value indicating the success or failure of the
 * read operation. A return value of 0 indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_read_x_deltvel(struct adis_dev *adis, int32_t *x_deltvel);
/***************************************************************************//**
 * @brief This function is used to retrieve the delta velocity measurement along
 * the Y axis from the specified ADIS device. It should be called after
 * the device has been properly initialized. If the FIFO mode is enabled,
 * it will first disable it before reading the value. The function will
 * return an error code if the read operation fails, which can occur if
 * the device is not ready or if there are communication issues.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the delta velocity. This pointer must not be null
 * and should point to a valid initialized device.
 * @param y_deltvel A pointer to an `int32_t` variable where the read delta
 * velocity value will be stored. This pointer must not be
 * null; otherwise, the function will not perform the read
 * operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_y_deltvel(struct adis_dev *adis, int32_t *y_deltvel);
/***************************************************************************//**
 * @brief This function is used to retrieve the delta velocity measurement along
 * the z-axis from the specified ADIS device. It should be called after
 * the device has been properly initialized. If the FIFO mode is enabled,
 * it will first disable it before reading the value. The function will
 * return an error code if the read operation fails, which can occur if
 * the device is not ready or if there are communication issues.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param z_deltvel Pointer to an `int32_t` where the read delta velocity value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_z_deltvel(struct adis_dev *adis, int32_t *z_deltvel);

/***************************************************************************//**
 * @brief This function is used to retrieve the current sample count from the
 * output FIFO of the ADIS device. It should be called after the device
 * has been initialized and configured properly. The function expects a
 * valid pointer to a `uint32_t` variable where the FIFO count will be
 * stored. If the provided pointer is null, the function will not perform
 * the read operation and may return an error code. It is important to
 * ensure that the device is in a state where it can provide the FIFO
 * count, as calling this function under inappropriate conditions may
 * lead to undefined behavior.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid initialized device.
 * @param fifo_cnt A pointer to a `uint32_t` variable where the FIFO sample
 * count will be stored. This pointer must not be null;
 * otherwise, the function will not perform the read operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_fifo_cnt(struct adis_dev *adis, uint32_t *fifo_cnt);
/***************************************************************************//**
 * @brief This function is used to retrieve the checksum of the current sample
 * SPI transaction from the specified `adis` device. It should be called
 * after the device has been properly initialized and configured. The
 * checksum is useful for verifying the integrity of the data received
 * from the device. If the `checksum` pointer is null, the function will
 * not perform any operation and will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should be initialized before calling this
 * function.
 * @param checksum Pointer to a `uint32_t` where the checksum value will be
 * stored. Must not be null; if it is null, the function will
 * return an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_spi_chksum(struct adis_dev *adis, uint32_t *checksum);

/***************************************************************************//**
 * @brief This function is used to retrieve the current bias value for the
 * gyroscope on the x-axis from the specified device. It should be called
 * after the device has been properly initialized and configured. The
 * function will write the retrieved bias value into the provided
 * pointer. If the device is not initialized or if the pointer is null,
 * the function may return an error code.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device
 * from which the bias is to be read. This pointer must not be null
 * and should point to a valid initialized device.
 * @param xg_bias A pointer to an `int32_t` variable where the read bias value
 * will be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the reason for the failure.
 ******************************************************************************/
int adis_read_xg_bias(struct adis_dev *adis, int32_t *xg_bias);
/***************************************************************************//**
 * @brief This function is used to set the bias correction value for the
 * gyroscope on the X-axis. It should be called after the device has been
 * initialized and is ready for configuration. The input value represents
 * the bias correction to be applied, which can help improve the accuracy
 * of gyroscope measurements. If the provided bias value is invalid, the
 * function will handle it appropriately, ensuring that the device state
 * remains consistent.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param xg_bias The bias value to be written, represented as a 32-bit signed
 * integer. Valid values depend on the specific device's range
 * for bias correction.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_xg_bias(struct adis_dev *adis, int32_t xg_bias);
/***************************************************************************//**
 * @brief This function is used to retrieve the current bias value of the
 * gyroscope on the Y-axis from the specified device. It should be called
 * after the device has been properly initialized and configured. The
 * function expects a valid pointer to an `int32_t` variable where the
 * bias value will be stored. If the provided pointer is null or if the
 * device is not initialized correctly, the function may return an error
 * code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the Y-axis gyroscope bias is to be read. This pointer must
 * not be null and should point to a valid initialized device.
 * @param yg_bias A pointer to an `int32_t` variable where the Y-axis gyroscope
 * bias will be stored. This pointer must not be null; otherwise,
 * the function will return an error.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_yg_bias(struct adis_dev *adis, int32_t *yg_bias);
/***************************************************************************//**
 * @brief This function is used to set the gyroscope bias for the Y-axis in an
 * ADIS device. It should be called after the device has been properly
 * initialized and configured. The function takes an integer value
 * representing the bias to be written. If the provided `adis` pointer is
 * null or if the writing operation fails, the function will return an
 * error code, indicating the failure.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param yg_bias An integer value representing the Y-axis gyroscope bias to be
 * written. The valid range for this value is device-specific and
 * should be determined from the device's documentation.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_yg_bias(struct adis_dev *adis, int32_t yg_bias);
/***************************************************************************//**
 * @brief This function is used to retrieve the gyroscope bias value for the
 * z-axis from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function
 * expects a valid pointer to an `int32_t` variable where the retrieved
 * bias value will be stored. If the provided pointer is null or if the
 * device is not properly initialized, the function may return an error
 * code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the z-axis gyroscope bias is to be read. This pointer must
 * not be null and should point to a valid initialized device.
 * @param zg_bias A pointer to an `int32_t` variable where the z-axis gyroscope
 * bias will be stored. This pointer must not be null; otherwise,
 * the function will not be able to store the result.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_zg_bias(struct adis_dev *adis, int32_t *zg_bias);
/***************************************************************************//**
 * @brief This function is used to set the bias value for the Z-axis gyroscope
 * in the ADIS device. It should be called after the device has been
 * properly initialized. The `zg_bias` parameter represents the bias
 * value to be written, and it is important to ensure that this value is
 * within the acceptable range for the device. If the function encounters
 * an error during the write operation, it will return a negative error
 * code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and must point to a valid device
 * that has been initialized.
 * @param zg_bias An integer value representing the Z-axis gyroscope bias to be
 * written. The value should be within the valid range specified
 * by the device's documentation. If the value is out of range,
 * the function may return an error.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_zg_bias(struct adis_dev *adis, int32_t zg_bias);

/***************************************************************************//**
 * @brief This function is used to retrieve the X-axis acceleration bias from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized. The function expects a valid pointer to an
 * `adis_dev` structure representing the device and a pointer to an
 * `int32_t` variable where the bias value will be stored. If the
 * provided pointers are null or if the device is not initialized
 * correctly, the function may return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to a valid initialized device.
 * @param xa_bias Pointer to an `int32_t` variable where the X-axis acceleration
 * bias will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_xa_bias(struct adis_dev *adis, int32_t *xa_bias);
/***************************************************************************//**
 * @brief This function is used to set the X-axis acceleration bias for the
 * specified ADIS device. It should be called after the device has been
 * properly initialized. The function expects a valid pointer to an
 * `adis_dev` structure, which represents the device, and an integer
 * value representing the bias to be set. If the provided device pointer
 * is null, the function will return an error code. It is important to
 * ensure that the bias value is within the acceptable range for the
 * specific device to avoid unexpected behavior.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param xa_bias An integer value representing the X-axis acceleration bias to
 * be set. The valid range for this value depends on the specific
 * device specifications.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_write_xa_bias(struct adis_dev *adis, int32_t xa_bias);
/***************************************************************************//**
 * @brief This function is used to retrieve the Y-axis acceleration bias from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The function expects a valid
 * pointer to an `int32_t` variable where the Y-axis bias value will be
 * stored. If the provided pointer is null or if the device is not
 * properly initialized, the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the Y-axis bias. This pointer must not be null and
 * should point to a valid initialized device.
 * @param ya_bias A pointer to an `int32_t` variable where the Y-axis bias value
 * will be stored. This pointer must not be null; otherwise, the
 * function will not be able to write the bias value.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_ya_bias(struct adis_dev *adis, int32_t *ya_bias);
/***************************************************************************//**
 * @brief This function is used to set the Y-axis acceleration bias for the
 * specified ADIS device. It should be called after the device has been
 * properly initialized. The function takes an `adis_dev` structure
 * pointer and an integer value representing the bias. If the provided
 * `adis` pointer is null, the function will return an error. The bias
 * value should be within the acceptable range defined by the device
 * specifications; otherwise, the function may not behave as expected.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param ya_bias The Y-axis acceleration bias value to be written. This value
 * should be within the valid range for the device; otherwise,
 * the function may return an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_ya_bias(struct adis_dev *adis, int32_t ya_bias);
/***************************************************************************//**
 * @brief This function is used to retrieve the Z-axis acceleration bias from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The function expects a valid
 * pointer to an `int32_t` variable where the retrieved bias value will
 * be stored. If the provided pointer is null or if the device is not
 * properly initialized, the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the Z-axis bias is to be read. This pointer must not be
 * null and should point to a valid initialized device.
 * @param za_bias A pointer to an `int32_t` variable where the Z-axis bias value
 * will be stored. This pointer must not be null; otherwise, the
 * function will not be able to store the result.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_za_bias(struct adis_dev *adis, int32_t *za_bias);
/***************************************************************************//**
 * @brief This function is used to set the z-axis acceleration bias for the
 * device. It should be called after the device has been properly
 * initialized. The function takes an `adis_dev` structure pointer and an
 * integer value representing the z-axis bias. If the provided `adis`
 * pointer is null, the function will return an error. It is important to
 * ensure that the device is in a state that allows writing to its
 * registers.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param za_bias The z-axis acceleration bias value to be written. This is a
 * 32-bit signed integer. The function does not impose specific
 * value constraints, but the caller should ensure that the value
 * is within the acceptable range for the device.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_write_za_bias(struct adis_dev *adis, int32_t za_bias);

/***************************************************************************//**
 * @brief This function is used to retrieve the current scale adjustment value
 * for the gyroscope on the x-axis from the specified device. It should
 * be called after the device has been properly initialized and
 * configured. The function expects a valid pointer to an `int32_t`
 * variable where the scale value will be stored. If the provided pointer
 * is null or if the device is not initialized, the function may return
 * an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device.
 * @param xg_scale A pointer to an `int32_t` variable where the scale adjustment
 * value will be stored. This pointer must not be null;
 * otherwise, the function will return an error.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the reason for the failure.
 ******************************************************************************/
int adis_read_xg_scale(struct adis_dev *adis, int32_t *xg_scale);
/***************************************************************************//**
 * @brief This function is used to set the gyroscope scale for the x-axis in an
 * ADIS device. It should be called after the device has been initialized
 * and is ready for configuration. The `xg_scale` parameter specifies the
 * desired scale value, which is expected to be within the valid range
 * for the device. If the provided scale value is out of range, the
 * function may return an error code indicating the failure to write the
 * value.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and must point to a valid device
 * that has been initialized.
 * @param xg_scale An integer value representing the desired gyroscope scale for
 * the x-axis. The valid range for this value depends on the
 * specific device and should be checked in the device's
 * documentation. If the value is out of range, the function
 * will handle it by returning an error code.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_xg_scale(struct adis_dev *adis, int32_t xg_scale);
/***************************************************************************//**
 * @brief This function is used to retrieve the scale value of the gyroscope on
 * the Y-axis from the specified device. It should be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to an `int32_t` variable where the scale value will be stored.
 * If the provided pointer is null, the function will not perform the
 * read operation and may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the scale value is to be read. This pointer must not be
 * null and should point to a valid initialized device.
 * @param yg_scale A pointer to an `int32_t` variable where the Y-axis gyroscope
 * scale value will be stored. This pointer must not be null;
 * otherwise, the function will not perform the read operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_yg_scale(struct adis_dev *adis, int32_t *yg_scale);
/***************************************************************************//**
 * @brief This function is used to set the scale factor for the Y-axis gyroscope
 * in the ADIS device. It should be called after the device has been
 * initialized and is ready for configuration. The function expects a
 * valid pointer to an `adis_dev` structure, which represents the device,
 * and an integer value representing the desired scale. If the provided
 * scale value is out of the acceptable range, the function may return an
 * error code, indicating that the operation was unsuccessful.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param yg_scale An integer value representing the desired scale for the
 * Y-axis gyroscope. The valid range for this value is
 * implementation-specific, and the function may return an error
 * if the value is outside the acceptable limits.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_yg_scale(struct adis_dev *adis, int32_t yg_scale);
/***************************************************************************//**
 * @brief This function is used to retrieve the scale factor for the z-axis
 * gyroscope of the specified device. It should be called after the
 * device has been properly initialized. The caller must ensure that the
 * `zg_scale` pointer is valid and points to a memory location where the
 * scale value can be stored. If the function encounters an error during
 * the read operation, it will return a negative error code, indicating
 * the type of failure.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which the gyroscope scale is to be read. This pointer must not be
 * null and should point to a valid initialized device.
 * @param zg_scale Pointer to an `int32_t` variable where the read scale value
 * will be stored. This pointer must not be null; otherwise, the
 * function will not perform the read operation.
 * @return Returns 0 on success, indicating that the scale value has been
 * successfully read and stored in the provided pointer. A negative
 * value indicates an error occurred during the read operation.
 ******************************************************************************/
int adis_read_zg_scale(struct adis_dev *adis, int32_t *zg_scale);
/***************************************************************************//**
 * @brief This function is used to set the scale factor for the Z-axis gyroscope
 * of the specified device. It should be called after the device has been
 * properly initialized. The `zg_scale` parameter must be within the
 * valid range for the device, and the function will return an error code
 * if the value is out of bounds or if the device is not ready for
 * configuration.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null and must point to a valid initialized device.
 * @param zg_scale An integer value representing the desired scale for the
 * Z-axis gyroscope. The valid range for this value is device-
 * specific; refer to the device documentation for details. If
 * the value is out of range, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_zg_scale(struct adis_dev *adis, int32_t zg_scale);
/***************************************************************************//**
 * @brief This function is used to retrieve the scale factor for the X-axis
 * acceleration from the specified device. It should be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to an `adis_dev` structure representing the device and a
 * pointer to an `int32_t` variable where the scale value will be stored.
 * If the provided pointers are null or if the device is not initialized,
 * the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null and must point to a valid initialized device.
 * @param xa_scale A pointer to an `int32_t` variable where the X-axis
 * acceleration scale will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_xa_scale(struct adis_dev *adis, int32_t *xa_scale);
/***************************************************************************//**
 * @brief This function is used to set the scale factor for the X-axis
 * acceleration measurement of the device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `xa_scale` parameter specifies the desired scale value, which is
 * expected to be within the valid range for the device. If the provided
 * value is out of range or if the device is not properly initialized,
 * the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been initialized.
 * @param xa_scale An integer value representing the desired scale for the
 * X-axis acceleration. The valid range for this value is
 * implementation-specific; refer to the device documentation
 * for details. If the value is invalid, the function will
 * return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_xa_scale(struct adis_dev *adis, int32_t xa_scale);
/***************************************************************************//**
 * @brief This function is used to retrieve the Y-axis acceleration scale from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The function expects a valid
 * pointer to an `int32_t` variable where the retrieved scale value will
 * be stored. If the provided pointer is null or if the device is not
 * properly initialized, the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the Y-axis acceleration scale is to be read. This pointer
 * must not be null and should point to a valid initialized device.
 * @param ya_scale A pointer to an `int32_t` variable where the Y-axis
 * acceleration scale will be stored. This pointer must not be
 * null; otherwise, the function will not be able to store the
 * result.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_ya_scale(struct adis_dev *adis, int32_t *ya_scale);
/***************************************************************************//**
 * @brief This function is used to set the scale factor for the Y-axis
 * acceleration measurement in the ADIS device. It should be called after
 * the device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and an integer value representing the desired
 * scale. If the provided scale value is out of the acceptable range, the
 * function may return an error code, indicating that the write operation
 * was unsuccessful.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param ya_scale The desired scale factor for the Y-axis acceleration. This is
 * an integer value, and the function will handle invalid values
 * by returning an error code.
 * @return Returns an integer indicating the success or failure of the write
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_ya_scale(struct adis_dev *adis, int32_t ya_scale);
/***************************************************************************//**
 * @brief This function is used to retrieve the Z-axis acceleration scale from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The function expects a valid
 * pointer to an `int32_t` variable where the retrieved scale value will
 * be stored. If the provided pointer is null or if the device is not
 * properly initialized, the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the Z-axis scale is to be read. This pointer must not be
 * null and should point to a valid initialized device.
 * @param za_scale A pointer to an `int32_t` variable where the Z-axis scale
 * value will be stored. This pointer must not be null;
 * otherwise, the function will not be able to store the result.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_za_scale(struct adis_dev *adis, int32_t *za_scale);
/***************************************************************************//**
 * @brief This function is used to set the Z-axis acceleration scale for the
 * specified device. It should be called after the device has been
 * initialized and is ready for configuration. The function expects a
 * valid pointer to an `adis_dev` structure and an integer value
 * representing the desired scale. If the provided scale value is out of
 * the acceptable range, the function may return an error code,
 * indicating that the write operation was unsuccessful.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param za_scale An integer value representing the desired Z-axis acceleration
 * scale. The valid range for this value should be defined by
 * the device specifications. If the value is out of range, the
 * function will handle it appropriately by returning an error
 * code.
 * @return Returns an integer indicating the success or failure of the write
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_za_scale(struct adis_dev *adis, int32_t za_scale);

/***************************************************************************//**
 * @brief This function is used to retrieve the current state of the FIFO enable
 * bit for the specified ADIS device. It should be called after the
 * device has been initialized and configured. The function will write
 * the value of the FIFO enable bit to the provided pointer. If the
 * pointer is null, the function will not perform any operation and may
 * return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device.
 * @param fifo_en A pointer to a `uint32_t` variable where the FIFO enable bit
 * value will be stored. This pointer must not be null;
 * otherwise, the function will not perform any operation.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fifo_en(struct adis_dev *adis, uint32_t *fifo_en);
/***************************************************************************//**
 * @brief This function is used to configure the FIFO (First In, First Out)
 * feature of the device. It should be called after the device has been
 * initialized and before any data operations are performed. The
 * `fifo_en` parameter determines whether the FIFO is enabled (1) or
 * disabled (0). If an invalid value is provided, the function will
 * return an error code, and the FIFO state will remain unchanged.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param fifo_en An integer value that specifies the FIFO enable state: 1 to
 * enable and 0 to disable. Valid values are 0 and 1. If an
 * invalid value is provided, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fifo_en(struct adis_dev *adis, uint32_t fifo_en);
/***************************************************************************//**
 * @brief This function is used to read the FIFO overflow status from the
 * specified `adis` device. It should be called after the device has been
 * properly initialized and configured. The function will update the
 * value pointed to by `fifo_overflow` with the current overflow status.
 * If the `adis` device is not initialized or if the pointer to
 * `fifo_overflow` is null, the function may return an error code. It is
 * important to ensure that the device is in a valid state before calling
 * this function to avoid unexpected behavior.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param fifo_overflow Pointer to a `uint32_t` variable where the overflow
 * status will be stored. Must not be null; if it is null,
 * the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_fifo_overflow(struct adis_dev *adis, uint32_t *fifo_overflow);
/***************************************************************************//**
 * @brief This function is used to set the FIFO overflow value for the specified
 * ADIS device. It should be called after the device has been properly
 * initialized. The function expects a valid pointer to an `adis_dev`
 * structure, which represents the device, and a 32-bit unsigned integer
 * representing the FIFO overflow value. If the provided device pointer
 * is null, the function will not perform any operation.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fifo_overflow A 32-bit unsigned integer representing the FIFO overflow
 * value. Valid values depend on the specific device's
 * requirements.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fifo_overflow(struct adis_dev *adis, uint32_t fifo_overflow);
/***************************************************************************//**
 * @brief This function is used to read the FIFO watermark interrupt enable
 * setting from the specified `adis` device. It should be called after
 * the device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` instance and a pointer to a `uint32_t`
 * variable where the result will be stored. If the provided pointers are
 * null or if the device is not initialized, the function may return an
 * error code.
 *
 * @param adis Pointer to the `struct adis_dev` instance representing the
 * device. Must not be null and should point to a valid initialized
 * device.
 * @param fifo_wm_int_en Pointer to a `uint32_t` variable where the FIFO
 * watermark interrupt enable setting will be stored. Must
 * not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_fifo_wm_int_en(struct adis_dev *adis, uint32_t *fifo_wm_int_en);
/***************************************************************************//**
 * @brief This function is used to enable or disable the FIFO watermark
 * interrupt for the specified device. It should be called after the
 * device has been initialized and configured. The `fifo_wm_int_en`
 * parameter determines whether the interrupt is enabled or disabled. If
 * the value is set to a non-zero value, the interrupt will be enabled;
 * otherwise, it will be disabled. It is important to ensure that the
 * device is in a valid state before calling this function to avoid
 * unexpected behavior.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fifo_wm_int_en An unsigned 32-bit integer that specifies the FIFO
 * watermark interrupt enable state. Valid values are
 * typically 0 (disable) or 1 (enable). The function will
 * handle invalid values by treating them as 0 (disable).
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fifo_wm_int_en(struct adis_dev *adis, uint32_t fifo_wm_int_en);
/***************************************************************************//**
 * @brief This function is used to read the FIFO watermark interrupt polarity
 * setting from the specified ADIS device. It should be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the read value will be stored. If the
 * provided pointers are null or if the device is not initialized, the
 * function may return an error code.
 *
 * @param adis Pointer to the `struct adis_dev` representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param fifo_wm_int_pol Pointer to a `uint32_t` variable where the FIFO
 * watermark interrupt polarity value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fifo_wm_int_pol(struct adis_dev *adis, uint32_t *fifo_wm_int_pol);
/***************************************************************************//**
 * @brief This function is used to configure the FIFO watermark interrupt
 * polarity for the specified device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the desired interrupt polarity. If the provided device pointer is null
 * or invalid, the function will return an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fifo_wm_int_pol A 32-bit unsigned integer representing the desired
 * FIFO watermark interrupt polarity. The valid range of
 * values depends on the specific device's configuration.
 * The function will return an error if the value is out
 * of range.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fifo_wm_int_pol(struct adis_dev *adis, uint32_t fifo_wm_int_pol);
/***************************************************************************//**
 * @brief This function retrieves the current FIFO watermark level from the
 * specified ADIS device. It should be called after the device has been
 * initialized and is ready for communication. The watermark level
 * indicates the threshold at which the FIFO will trigger an interrupt or
 * other action. If the provided pointer for the watermark level is null,
 * the function will not perform any operation and may return an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which to read the FIFO watermark level. Must not be null.
 * @param fifo_wm_lvl Pointer to a `uint32_t` variable where the FIFO watermark
 * level will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fifo_wm_lvl(struct adis_dev *adis, uint32_t *fifo_wm_lvl);
/***************************************************************************//**
 * @brief This function is used to set the FIFO watermark level for the
 * specified device. It should be called after the device has been
 * initialized and is ready for configuration. The watermark level
 * determines the threshold at which the FIFO will trigger an interrupt
 * or other action when the number of samples reaches this level. Ensure
 * that the provided level is within the valid range for the specific
 * device, as invalid values may result in an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fifo_wm_lvl The desired FIFO watermark level as a 32-bit unsigned
 * integer. The valid range for this value depends on the
 * specific device configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fifo_wm_lvl(struct adis_dev *adis, uint32_t fifo_wm_lvl);

/***************************************************************************//**
 * @brief This function is used to retrieve the current value of the filter size
 * variable B from the specified ADIS device. It should be called after
 * the device has been properly initialized and configured. The function
 * expects a valid pointer to a `struct adis_dev` representing the device
 * and a pointer to a `uint32_t` variable where the retrieved filter size
 * will be stored. If the provided pointers are null or if the device is
 * not properly initialized, the function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param filt_size_var_b Pointer to a `uint32_t` variable where the filter size
 * variable B will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_filt_size_var_b(struct adis_dev *adis, uint32_t *filt_size_var_b);
/***************************************************************************//**
 * @brief This function is used to configure the filter size variable B for the
 * specified device. It should be called after the device has been
 * initialized and is ready for configuration. The function checks if the
 * provided filter size variable B exceeds the maximum allowed value for
 * the device; if it does, an error is returned. After successfully
 * writing the value, the function introduces a delay to allow the device
 * to process the update, ensuring that the configuration takes effect.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param filt_size_var_b The desired filter size variable B value to be set. It
 * must be within the range of 0 to the maximum value
 * defined by the device's specifications. If the value
 * exceeds this maximum, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_filt_size_var_b(struct adis_dev *adis, uint32_t filt_size_var_b);
/***************************************************************************//**
 * @brief This function retrieves the low-pass filter frequency for a specified
 * channel and axis of an ADIS device. It should be called after the
 * device has been initialized. The function ignores the specified
 * channel and axis parameters, as the low-pass filter setting applies to
 * all measurements. If the device's specific read function is not
 * available, it will read the filter size variable and use it to
 * determine the frequency. If the filter size is out of range, an error
 * will be returned.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param chan An enumeration value of type `adis_chan_type` representing the
 * channel type. Valid values are defined in the `adis_chan_type`
 * enum.
 * @param axis An enumeration value of type `adis_axis_type` representing the
 * axis type. Valid values are defined in the `adis_axis_type` enum.
 * @param freq Pointer to a `uint32_t` where the filter frequency will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 * The frequency value is written to the location pointed to by `freq`.
 ******************************************************************************/
int adis_read_lpf(struct adis_dev *adis, enum adis_chan_type chan,
		  enum adis_axis_type axis, uint32_t *freq);
/***************************************************************************//**
 * @brief This function is used to set the low-pass filter frequency for the
 * device, which affects the filtering of sensor measurements. It should
 * be called after the device has been initialized. The frequency
 * parameter must be a valid value that corresponds to the supported
 * filter frequencies. If the provided frequency exceeds the maximum
 * supported frequency, it will be clamped to the highest available
 * frequency. The function does not differentiate between channels or
 * axes, as the filter setting applies globally to all measurements.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null.
 * @param chan An enumeration value of type `adis_chan_type` representing the
 * channel type. Valid values include `ADIS_ACCL_CHAN`,
 * `ADIS_GYRO_CHAN`, `ADIS_TEMP_CHAN`, `ADIS_DELTAANGL_CHAN`, and
 * `ADIS_DELTAVEL_CHAN`. The function ignores this parameter.
 * @param axis An enumeration value of type `adis_axis_type` representing the
 * axis type. Valid values include `ADIS_X_AXIS`, `ADIS_Y_AXIS`, and
 * `ADIS_Z_AXIS`. The function ignores this parameter.
 * @param freq A 32-bit unsigned integer representing the desired filter
 * frequency. Must be a valid frequency supported by the device. If
 * the frequency is invalid (too high), it will be clamped to the
 * maximum supported frequency.
 * @return Returns an integer indicating the result of the operation. A return
 * value of 0 indicates success, while a negative value indicates an
 * error.
 ******************************************************************************/
int adis_write_lpf(struct adis_dev *adis, enum adis_chan_type chan,
		   enum adis_axis_type axis, uint32_t freq);
/***************************************************************************//**
 * @brief This function is used to retrieve the gyroscope measurement range from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized. The function expects a valid pointer to a
 * `struct adis_dev` representing the device and a pointer to a
 * `uint32_t` variable where the measurement range will be stored. If the
 * provided pointers are null or if the device is not initialized
 * correctly, the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device from
 * which the gyroscope measurement range is to be read. This pointer
 * must not be null and should point to a valid initialized device.
 * @param gyro_meas_range A pointer to a `uint32_t` variable where the gyroscope
 * measurement range will be stored. This pointer must
 * not be null; otherwise, the function will not be able
 * to write the result.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_gyro_meas_range(struct adis_dev *adis, uint32_t *gyro_meas_range);

/***************************************************************************//**
 * @brief This function is used to retrieve the FIR filter enable status for the
 * x-axis gyroscope of the specified device. It should be called after
 * the device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the filter enable status will be
 * stored. If the provided pointers are null or if the device is not
 * initialized, the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. This
 * pointer must not be null and should point to a valid initialized
 * device.
 * @param fir_en_xg A pointer to a `uint32_t` variable where the FIR filter
 * enable status will be stored. This pointer must not be null.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the status of the operation.
 ******************************************************************************/
int adis_read_fir_en_xg(struct adis_dev *adis, uint32_t *fir_en_xg);
/***************************************************************************//**
 * @brief This function is used to configure the filter enable setting for the
 * x-axis gyroscope in an ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `fir_en_xg` parameter specifies the desired filter enable setting,
 * which is typically a bitmask representing various filter options. If
 * the provided `adis` pointer is null or if the `fir_en_xg` value is
 * invalid, the function may return an error code, indicating that the
 * operation could not be completed.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_en_xg Unsigned 32-bit integer representing the filter enable
 * setting for the x-axis gyroscope. Valid values depend on the
 * specific filter options supported by the device.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_xg(struct adis_dev *adis, uint32_t fir_en_xg);
/***************************************************************************//**
 * @brief This function is used to retrieve the filter enable status for the
 * Y-axis gyroscope of the specified device. It should be called after
 * the device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the filter enable status will be
 * stored. If the provided pointers are null, the function will not
 * perform any operation and may return an error.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. Must
 * not be null.
 * @param fir_en_yg A pointer to a `uint32_t` variable where the filter enable
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_en_yg(struct adis_dev *adis, uint32_t *fir_en_yg);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter enable setting for
 * the Y-axis gyroscope of the ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the desired FIR filter enable value. If the provided `adis` pointer is
 * null, the function will return an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_en_yg A 32-bit unsigned integer representing the FIR filter enable
 * value for the Y-axis gyroscope. Valid values depend on the
 * specific device configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_yg(struct adis_dev *adis, uint32_t fir_en_yg);
/***************************************************************************//**
 * @brief This function is used to retrieve the FIR filter enable status for the
 * z-axis gyroscope of the specified device. It should be called after
 * the device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the result will be stored. If the
 * provided pointers are null, the function will not perform any
 * operation and may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. Must
 * not be null.
 * @param fir_en_zg A pointer to a `uint32_t` variable where the FIR filter
 * enable status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_en_zg(struct adis_dev *adis, uint32_t *fir_en_zg);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter enable setting for
 * the Z-axis gyroscope of the ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `fir_en_zg` parameter specifies the enable setting, which typically
 * consists of a bitmask that determines whether the filter is enabled or
 * disabled. It is important to ensure that the device is not locked
 * before calling this function, as configuration changes are not allowed
 * when the device is in a locked state.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param fir_en_zg A 32-bit unsigned integer representing the FIR filter enable
 * setting for the Z-axis gyroscope. The valid range depends on
 * the specific configuration of the device. The function will
 * handle invalid values by clamping or ignoring them as per
 * the device's specifications.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_zg(struct adis_dev *adis, uint32_t fir_en_zg);
/***************************************************************************//**
 * @brief This function is used to retrieve the FIR filter enable status for the
 * x-axis accelerometer of the specified device. It should be called
 * after the device has been properly initialized. The function expects a
 * valid pointer to a `struct adis_dev` representing the device and a
 * pointer to a `uint32_t` variable where the result will be stored. If
 * the device is not initialized or if the pointers provided are null,
 * the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. This
 * pointer must not be null and must point to a valid initialized
 * device.
 * @param fir_en_xa A pointer to a `uint32_t` variable where the FIR filter
 * enable status will be stored. This pointer must not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_fir_en_xa(struct adis_dev *adis, uint32_t *fir_en_xa);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter enable setting for
 * the x-axis accelerometer of the device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the FIR filter enable value. If the provided `adis` pointer is null,
 * or if the function encounters an error during the write operation, it
 * will return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param fir_en_xa A 32-bit unsigned integer representing the FIR filter enable
 * value for the x-axis accelerometer. Valid values depend on
 * the specific device's configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_xa(struct adis_dev *adis, uint32_t fir_en_xa);
/***************************************************************************//**
 * @brief This function is used to retrieve the filter enable status for the
 * Y-axis accelerometer of the specified device. It should be called
 * after the device has been properly initialized. The function expects a
 * valid pointer to a `struct adis_dev` representing the device and a
 * pointer to a `uint32_t` variable where the filter enable status will
 * be stored. If the provided pointers are null or if the device is not
 * initialized, the function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and must point to a valid initialized device.
 * @param fir_en_ya Pointer to a `uint32_t` variable where the filter enable
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_en_ya(struct adis_dev *adis, uint32_t *fir_en_ya);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter enable setting for
 * the Y-axis accelerometer of the device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the desired FIR filter enable value. If the provided device pointer is
 * null, the function will return an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_en_ya A 32-bit unsigned integer representing the FIR filter enable
 * value for the Y-axis accelerometer. Valid values depend on
 * the specific device configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_ya(struct adis_dev *adis, uint32_t fir_en_ya);
/***************************************************************************//**
 * @brief This function is used to retrieve the FIR filter enable status for the
 * z-axis accelerometer of the specified device. It should be called
 * after the device has been properly initialized. The function expects a
 * valid pointer to a `struct adis_dev` representing the device and a
 * pointer to a `uint32_t` variable where the enable status will be
 * stored. If the device is not initialized or if the pointers provided
 * are null, the function may return an error.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. This
 * pointer must not be null and should point to a valid initialized
 * device.
 * @param fir_en_za A pointer to a `uint32_t` variable where the FIR filter
 * enable status will be stored. This pointer must not be null.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the status of the read operation.
 ******************************************************************************/
int adis_read_fir_en_za(struct adis_dev *adis, uint32_t *fir_en_za);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter enable setting for
 * the z-axis accelerometer of the device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the FIR filter enable value. If the provided device pointer is null,
 * the function will return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_en_za 32-bit unsigned integer representing the FIR filter enable
 * value for the z-axis accelerometer. Valid values depend on
 * the specific device configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_en_za(struct adis_dev *adis, uint32_t fir_en_za);
/***************************************************************************//**
 * @brief This function is used to retrieve the current filter bank selection
 * for the x-axis gyroscope of the ADIS device. It should be called after
 * the device has been properly initialized and configured. The function
 * will populate the provided pointer with the filter bank selection
 * value. If the provided pointer is null, the function will not perform
 * any operation and may return an error code. It is important to handle
 * the return value to check for any potential errors during the read
 * operation.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_bank_sel_xg Pointer to a `uint32_t` variable where the filter bank
 * selection value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_bank_sel_xg(struct adis_dev *adis, uint32_t *fir_bank_sel_xg);
/***************************************************************************//**
 * @brief This function is used to set the filter bank selection for the x-axis
 * gyroscope in an ADIS device. It should be called after the device has
 * been initialized and is ready for configuration. The `fir_bank_sel_xg`
 * parameter specifies the desired filter bank selection value. It is
 * important to ensure that the value provided is within the valid range
 * expected by the device, as invalid values may lead to undefined
 * behavior or errors.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_bank_sel_xg Unsigned 32-bit integer representing the filter bank
 * selection for the x-axis gyroscope. The valid range of
 * this value is determined by the device specifications.
 * Caller retains ownership.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_bank_sel_xg(struct adis_dev *adis, uint32_t fir_bank_sel_xg);
/***************************************************************************//**
 * @brief This function is used to retrieve the filter bank selection value for
 * the Y-axis gyroscope from the specified ADIS device. It should be
 * called after the device has been properly initialized. The function
 * expects a valid pointer to a `struct adis_dev` representing the device
 * and a pointer to a `uint32_t` variable where the filter bank selection
 * value will be stored. If the provided pointers are null or if the
 * device is not initialized, the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device. Must
 * not be null and must point to a valid initialized device.
 * @param fir_bank_sel_yg A pointer to a `uint32_t` variable where the filter
 * bank selection value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_bank_sel_yg(struct adis_dev *adis, uint32_t *fir_bank_sel_yg);
/***************************************************************************//**
 * @brief This function is used to set the filter bank selection for the Y-axis
 * gyroscope in an ADIS device. It should be called after the device has
 * been initialized and is ready for configuration. The function expects
 * a valid `adis` device pointer and a `fir_bank_sel_yg` value that
 * specifies the desired filter bank selection. If the provided `adis`
 * pointer is null, or if the operation fails, the function will return
 * an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_bank_sel_yg Unsigned 32-bit integer representing the filter bank
 * selection for the Y-axis gyroscope. Valid values
 * depend on the specific device's filter bank
 * configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_bank_sel_yg(struct adis_dev *adis, uint32_t fir_bank_sel_yg);
/***************************************************************************//**
 * @brief This function is used to retrieve the FIR filter bank selection value
 * for the Z-axis gyroscope from the specified ADIS device. It should be
 * called after the device has been properly initialized. The function
 * expects a valid pointer to a `struct adis_dev` representing the device
 * and a pointer to a `uint32_t` variable where the result will be
 * stored. If the provided pointers are null or if the device is not
 * initialized, the function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param fir_bank_sel_zg Pointer to a `uint32_t` variable where the FIR filter
 * bank selection value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_bank_sel_zg(struct adis_dev *adis, uint32_t *fir_bank_sel_zg);
/***************************************************************************//**
 * @brief This function is used to configure the filter bank selection for the
 * z-axis gyroscope in an ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the desired filter bank selection. If the provided `adis` pointer is
 * null, or if the operation fails, the function will return an error
 * code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param fir_bank_sel_zg A 32-bit unsigned integer representing the filter bank
 * selection for the z-axis gyroscope. Valid values
 * depend on the specific device configuration. The
 * function will return an error if the value is invalid.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_bank_sel_zg(struct adis_dev *adis, uint32_t fir_bank_sel_zg);
/***************************************************************************//**
 * @brief This function is used to retrieve the current filter bank selection
 * for the X-axis accelerometer of the ADIS device. It should be called
 * after the device has been properly initialized and configured. The
 * function expects a valid pointer to a `uint32_t` variable where the
 * selected filter bank value will be stored. If the provided pointer is
 * null or if the device is not initialized, the function may return an
 * error code, indicating the failure to read the filter bank selection.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device.
 * @param fir_bank_sel_xa A pointer to a `uint32_t` variable where the filter
 * bank selection value will be stored. This pointer must
 * not be null; otherwise, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_fir_bank_sel_xa(struct adis_dev *adis, uint32_t *fir_bank_sel_xa);
/***************************************************************************//**
 * @brief This function is used to set the filter bank selection for the x-axis
 * accelerometer in an ADIS device. It should be called after the device
 * has been initialized and configured. The provided filter bank
 * selection value must be valid according to the device's
 * specifications. If an invalid value is provided, the function may
 * return an error code, indicating that the operation was unsuccessful.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_bank_sel_xa Unsigned 32-bit integer representing the filter bank
 * selection for the x-axis accelerometer. The valid
 * range and specific values depend on the device's
 * documentation. The function will return an error if
 * the value is invalid.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_bank_sel_xa(struct adis_dev *adis, uint32_t fir_bank_sel_xa);
/***************************************************************************//**
 * @brief This function is used to retrieve the filter bank selection value for
 * the Y-axis accelerometer from the specified device. It should be
 * called after the device has been properly initialized and configured.
 * The function expects a valid pointer to a `struct adis_dev`
 * representing the device and a pointer to a `uint32_t` variable where
 * the filter bank selection value will be stored. If the provided
 * pointers are null or if the device is not initialized, the function
 * may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param fir_bank_sel_ya Pointer to a `uint32_t` variable where the filter bank
 * selection value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_bank_sel_ya(struct adis_dev *adis, uint32_t *fir_bank_sel_ya);
/***************************************************************************//**
 * @brief This function is used to set the filter bank selection for the Y-axis
 * accelerometer in an ADIS device. It should be called after the device
 * has been initialized and is ready for configuration. The
 * `fir_bank_sel_ya` parameter specifies the desired filter bank
 * selection value. It is important to ensure that the value provided is
 * within the valid range expected by the device, as invalid values may
 * lead to undefined behavior or errors.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param fir_bank_sel_ya The filter bank selection value for the Y-axis
 * accelerometer. This value should be within the valid
 * range defined by the device specifications. The
 * function does not handle out-of-range values and may
 * result in an error if an invalid value is provided.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_fir_bank_sel_ya(struct adis_dev *adis, uint32_t fir_bank_sel_ya);
/***************************************************************************//**
 * @brief This function is used to retrieve the filter bank selection value for
 * the z-axis accelerometer from the specified device. It should be
 * called after the device has been properly initialized. The caller must
 * ensure that the `adis` pointer is valid and points to an initialized
 * `adis_dev` structure. The function will write the retrieved filter
 * bank selection value into the memory location pointed to by
 * `fir_bank_sel_za`. If the function encounters an error during the read
 * operation, it will return a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to a valid initialized device.
 * @param fir_bank_sel_za Pointer to a `uint32_t` variable where the filter bank
 * selection value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_fir_bank_sel_za(struct adis_dev *adis, uint32_t *fir_bank_sel_za);
/***************************************************************************//**
 * @brief This function is used to configure the FIR filter bank selection for
 * the Z-axis accelerometer in an ADIS device. It should be called after
 * the device has been initialized and is ready for configuration. The
 * function expects a valid pointer to an `adis_dev` structure, which
 * represents the device, and a 32-bit unsigned integer that specifies
 * the desired filter bank selection. If the provided `adis` pointer is
 * null, or if the operation fails, the function will return an error
 * code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param fir_bank_sel_za A 32-bit unsigned integer representing the filter bank
 * selection for the Z-axis accelerometer. Valid values
 * depend on the specific device configuration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_fir_bank_sel_za(struct adis_dev *adis, uint32_t fir_bank_sel_za);
/***************************************************************************//**
 * @brief This function is used to read a specific FIR filter coefficient from
 * bank A of the ADIS device. It should be called after the device has
 * been properly initialized. The `coef_idx` parameter specifies the
 * index of the coefficient to be read, which must be within the valid
 * range defined by the device's maximum coefficient index. If the index
 * is out of range, the function will return an error code. The value of
 * the coefficient will be written to the location pointed to by the
 * `coef` parameter.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to read. Valid values are from 0
 * to the maximum index defined by the device. If the index is
 * greater than the maximum, the function will return an error.
 * @param coef Pointer to a `uint32_t` where the read coefficient value will be
 * stored. Caller retains ownership and must ensure it points to a
 * valid memory location.
 * @return Returns 0 on success, or a negative error code if the index is
 * invalid.
 ******************************************************************************/
int adis_read_fir_coef_bank_a(struct adis_dev *adis, uint8_t coef_idx,
			      uint32_t *coef);
/***************************************************************************//**
 * @brief This function is used to write a specified FIR coefficient to bank A
 * of the ADIS device. It should be called after the device has been
 * properly initialized. The `coef_idx` parameter must be within the
 * valid range defined by the device's maximum FIR coefficient index. If
 * the index exceeds this maximum, the function will return an error. It
 * is important to ensure that the device is not locked before attempting
 * to write coefficients, as this may prevent configuration changes.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to write. Valid values are from
 * 0 to the maximum index defined in the device's information
 * structure. If the index is greater than the maximum, the
 * function will return an error.
 * @param coef The FIR coefficient value to write. This is a 32-bit unsigned
 * integer. The function does not impose specific constraints on the
 * value, but it should be within the expected range for the
 * application.
 * @return Returns 0 on success, or a negative error code if the `coef_idx` is
 * invalid.
 ******************************************************************************/
int adis_write_fir_coef_bank_a(struct adis_dev *adis, uint8_t coef_idx,
			       uint32_t coef);
/***************************************************************************//**
 * @brief This function is used to read a specific FIR filter coefficient from
 * bank B of the ADIS device. It should be called after the device has
 * been properly initialized. The `coef_idx` parameter specifies the
 * index of the coefficient to read, which must be within the valid range
 * defined by the device's maximum coefficient index. If the index is out
 * of range, the function will return an error code. The value of the
 * coefficient will be written to the location pointed to by the `coef`
 * parameter, which must not be null.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to read. Valid values are from 0
 * to the maximum index defined by the device. If the index is
 * greater than the maximum, the function will return an error.
 * @param coef Pointer to a `uint32_t` where the read coefficient value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_fir_coef_bank_b(struct adis_dev *adis, uint8_t coef_idx,
			      uint32_t *coef);
/***************************************************************************//**
 * @brief This function is used to write a specified FIR coefficient to bank B
 * of the ADIS device. It should be called after the device has been
 * properly initialized. The `coef_idx` parameter must be within the
 * valid range defined by the device's maximum coefficient index,
 * otherwise, the function will return an error. It is important to
 * ensure that the device is not locked before attempting to write to the
 * coefficient bank, as writes may be restricted in that state.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the coefficient to write, which must be less than or
 * equal to the maximum coefficient index defined in the
 * device's information. If it exceeds this limit, the function
 * will return an error.
 * @param coef The coefficient value to write, represented as a 32-bit unsigned
 * integer. This value will be written directly to the specified
 * coefficient index.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_fir_coef_bank_b(struct adis_dev *adis, uint8_t coef_idx,
			       uint32_t coef);
/***************************************************************************//**
 * @brief This function is used to retrieve a specific FIR filter coefficient
 * from bank C of the ADIS device. It should be called after the device
 * has been properly initialized. The `coef_idx` parameter specifies the
 * index of the coefficient to read, which must be within the valid range
 * defined by the device's maximum coefficient index. If the index is out
 * of range, the function will return an error code. The retrieved
 * coefficient value will be stored in the memory location pointed to by
 * the `coef` parameter.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to read. Valid values are from 0
 * to the maximum index defined in the device's information
 * structure. If the index is greater than the maximum, an error
 * is returned.
 * @param coef Pointer to a `uint32_t` where the read coefficient value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the index is
 * invalid.
 ******************************************************************************/
int adis_read_fir_coef_bank_c(struct adis_dev *adis, uint8_t coef_idx,
			      uint32_t *coef);
/***************************************************************************//**
 * @brief This function is used to write a FIR coefficient to a specific index
 * in the coefficient bank of an ADIS device. It should be called after
 * the device has been properly initialized. The `coef_idx` parameter
 * must be within the valid range defined by the device's maximum
 * coefficient index; otherwise, the function will return an error. The
 * `coef` parameter represents the coefficient value to be written. It is
 * important to ensure that the device is not in a locked state, as this
 * may prevent writing operations.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the coefficient to write, which must be less than or
 * equal to the maximum index defined in the device's
 * information. If it exceeds this limit, the function will
 * return an error.
 * @param coef The coefficient value to write, represented as a 32-bit unsigned
 * integer. This value will be written to the specified coefficient
 * index.
 * @return Returns 0 on success, or a negative error code if the `coef_idx` is
 * invalid.
 ******************************************************************************/
int adis_write_fir_coef_bank_c(struct adis_dev *adis, uint8_t coef_idx,
			       uint32_t coef);
/***************************************************************************//**
 * @brief This function is used to read a specific FIR filter coefficient from
 * bank D of the ADIS device. It should be called after the device has
 * been properly initialized. The `coef_idx` parameter specifies the
 * index of the coefficient to be read, and it must be within the valid
 * range defined by the device's maximum coefficient index. If the index
 * is out of range, the function will return an error code. The value of
 * the coefficient will be written to the location pointed to by the
 * `coef` parameter.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to read. Valid values are from 0
 * to the maximum index defined in the device's information
 * structure. If the index exceeds this range, the function will
 * return an error.
 * @param coef Pointer to a `uint32_t` where the read coefficient value will be
 * stored. Caller retains ownership and must ensure it points to a
 * valid memory location.
 * @return Returns 0 on success, or a negative error code if the index is
 * invalid.
 ******************************************************************************/
int adis_read_fir_coef_bank_d(struct adis_dev *adis, uint8_t coef_idx,
			      uint32_t *coef);
/***************************************************************************//**
 * @brief This function is used to write a FIR coefficient to a specified index
 * in the coefficient bank of an ADIS device. It should be called after
 * the device has been properly initialized. The `coef_idx` parameter
 * must be within the valid range defined by the device's maximum
 * coefficient index; otherwise, the function will return an error. It is
 * important to ensure that the device is not in a locked state, as this
 * may prevent writing to the registers.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the coefficient to write, which must be less than or
 * equal to the maximum index defined in the device's
 * information structure. If it exceeds this limit, the function
 * will return an error.
 * @param coef The coefficient value to write, which is a 32-bit unsigned
 * integer. This value will be written to the specified coefficient
 * index.
 * @return Returns 0 on success, or a negative error code if the `coef_idx` is
 * invalid.
 ******************************************************************************/
int adis_write_fir_coef_bank_d(struct adis_dev *adis, uint8_t coef_idx,
			       uint32_t coef);

/***************************************************************************//**
 * @brief This function is used to retrieve the current data ready selection
 * setting from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function will
 * populate the provided pointer with the data ready selection value,
 * which indicates the source of the data ready signal. If the provided
 * pointer is null or if the device is not initialized, the function may
 * return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the data ready selection. This pointer must not be
 * null and should point to a valid initialized device.
 * @param dr_selection A pointer to a `uint32_t` variable where the data ready
 * selection value will be stored. This pointer must not be
 * null; if it is null, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_dr_selection(struct adis_dev *adis, uint32_t *dr_selection);
/***************************************************************************//**
 * @brief This function is used to configure the data ready selection for the
 * ADIS device. It should be called after the device has been initialized
 * and is ready for configuration. The `dr_selection` parameter specifies
 * the desired data ready selection value, which must be a valid encoded
 * value as defined by the device specifications. If the provided value
 * is invalid or if there is an error during the write operation, the
 * function will return a negative error code. A successful call will
 * result in the device being updated with the new data ready selection.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param dr_selection The data ready selection value to be written. This value
 * must be a valid encoded value as per the device's
 * specifications.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_dr_selection(struct adis_dev *adis, uint32_t dr_selection);
/***************************************************************************//**
 * @brief This function retrieves the current data ready polarity setting from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The caller must ensure that
 * the `adis` pointer is valid and points to an initialized `adis_dev`
 * structure. The `dr_polarity` pointer must not be null, as it will be
 * used to store the retrieved value. If the function encounters an error
 * during the read operation, it will return a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which to read the data ready polarity. Must not be null and must
 * point to a valid initialized device.
 * @param dr_polarity Pointer to a `uint32_t` variable where the data ready
 * polarity value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_dr_polarity(struct adis_dev *adis, uint32_t *dr_polarity);
/***************************************************************************//**
 * @brief This function is used to configure the data ready polarity for the
 * specified device. It should be called after the device has been
 * initialized. The `dr_polarity` parameter must be either 0 or 1,
 * representing the desired polarity setting. If an invalid value is
 * provided, the function will return an error code without modifying the
 * device state. After a successful write operation, the function
 * introduces a delay to ensure the device registers are updated
 * accordingly.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param dr_polarity The desired data ready polarity setting, which must be
 * either 0 or 1. If a value greater than 1 is provided, the
 * function will return an error code.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if the write operation fails.
 ******************************************************************************/
int adis_write_dr_polarity(struct adis_dev *adis, uint32_t dr_polarity);
/***************************************************************************//**
 * @brief This function is used to retrieve the current data ready enable
 * setting from the specified ADIS device. It should be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to a `uint32_t` variable where the retrieved value will be
 * stored. If the provided pointer is null, the function will not perform
 * the read operation and may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the data ready enable value. This pointer must not
 * be null.
 * @param dr_enable A pointer to a `uint32_t` variable where the data ready
 * enable value will be stored. This pointer must not be null;
 * otherwise, the function will not perform the read operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_dr_enable(struct adis_dev *adis, uint32_t *dr_enable);
/***************************************************************************//**
 * @brief This function is used to configure the data ready feature of the
 * device, allowing the user to enable or disable it based on the
 * provided parameter. It should be called after the device has been
 * initialized and is ready for configuration. The `dr_enable` parameter
 * must be either 0 (disabled) or 1 (enabled). If an invalid value is
 * provided, the function will return an error code without modifying the
 * device state. Additionally, after successfully writing the
 * configuration, the function introduces a small delay to ensure the
 * device registers are updated accordingly.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param dr_enable An integer value that specifies the desired state of the
 * data ready feature. Valid values are 0 (disable) and 1
 * (enable). If the value is greater than 1, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the input value is
 * invalid or if the write operation fails.
 ******************************************************************************/
int adis_write_dr_enable(struct adis_dev *adis, uint32_t dr_enable);
/***************************************************************************//**
 * @brief This function is used to retrieve the current synchronization
 * selection setting from the specified ADIS device. It should be called
 * after the device has been properly initialized and configured. The
 * function will write the retrieved value into the provided pointer. If
 * the pointer is null or if the device is not properly initialized, the
 * function may return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which to read the synchronization selection. Must not be null and
 * should point to a valid initialized device.
 * @param sync_selection Pointer to a `uint32_t` variable where the
 * synchronization selection value will be stored. Must
 * not be null; if it is null, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_sync_selection(struct adis_dev *adis, uint32_t *sync_selection);
/***************************************************************************//**
 * @brief This function is used to set the synchronization selection mode for
 * the ADIS device. It should be called after the device has been
 * initialized and is ready for configuration. The `sync_selection`
 * parameter must be a valid value defined by the device's
 * specifications. If the provided value is invalid, the function will
 * return an error code. After writing the selection, the function
 * introduces a small delay to ensure the device registers the change.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param sync_selection An unsigned 32-bit integer representing the
 * synchronization selection mode. Valid values are
 * defined by the device's specifications. The function
 * will return an error if an invalid value is provided.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_sync_selection(struct adis_dev *adis, uint32_t sync_selection);
/***************************************************************************//**
 * @brief This function is used to retrieve the current synchronization polarity
 * setting from the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function
 * expects a valid pointer to a `struct adis_dev` representing the device
 * and a pointer to a `uint32_t` variable where the synchronization
 * polarity value will be stored. If the provided pointers are null or if
 * the device is not initialized, the function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and must point to a valid initialized device.
 * @param sync_polarity Pointer to a `uint32_t` where the synchronization
 * polarity value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_sync_polarity(struct adis_dev *adis, uint32_t *sync_polarity);
/***************************************************************************//**
 * @brief This function is used to configure the synchronization polarity of the
 * ADIS device. It should be called after the device has been initialized
 * and before any data operations are performed. The `sync_polarity`
 * parameter must be either 0 or 1, where values outside this range will
 * result in an error. If the function is called with an invalid
 * `sync_polarity`, it will return an error code. After successfully
 * setting the synchronization polarity, the function introduces a small
 * delay to ensure the device registers the change.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device.
 * Must not be null, and the caller retains ownership.
 * @param sync_polarity An integer representing the desired synchronization
 * polarity, which must be either 0 or 1. Values greater
 * than 1 are considered invalid and will result in an
 * error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if the write operation fails.
 ******************************************************************************/
int adis_write_sync_polarity(struct adis_dev *adis, uint32_t sync_polarity);
/***************************************************************************//**
 * @brief This function retrieves the current synchronization mode of the
 * specified device. It should be called after the device has been
 * initialized and configured. The function will write the
 * synchronization mode value to the provided pointer. If the device does
 * not support a specific read function for synchronization mode, it will
 * fall back to a default method of reading the value. Ensure that the
 * pointer for the synchronization mode is valid and points to a memory
 * location that can store the result.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null.
 * @param sync_mode A pointer to a `uint32_t` where the synchronization mode
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_sync_mode(struct adis_dev *adis, uint32_t *sync_mode);
/***************************************************************************//**
 * @brief This function is used to configure the synchronization mode and
 * external clock frequency for the ADIS device. It should be called
 * after the device has been initialized. The `sync_mode` parameter must
 * be within the valid range defined by the device's capabilities, and if
 * it is set to a mode that requires an external clock, the `ext_clk`
 * must also fall within the specified frequency limits for that mode. If
 * the provided values are invalid, the function will return an error
 * code. Additionally, if the synchronization mode is set to 'scaled',
 * the function will adjust the internal sample rate based on the
 * external clock frequency.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param sync_mode The desired synchronization mode, which must be one of the
 * predefined modes (e.g., `ADIS_SYNC_DEFAULT`,
 * `ADIS_SYNC_DIRECT`, etc.). Must be less than or equal to the
 * maximum supported synchronization mode.
 * @param ext_clk The external clock frequency in Hertz. This parameter is only
 * relevant if the `sync_mode` requires an external clock. It
 * must be within the frequency limits defined for the selected
 * `sync_mode`.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if the operation fails.
 ******************************************************************************/
int adis_write_sync_mode(struct adis_dev *adis, uint32_t sync_mode,
			 uint32_t ext_clk);
/***************************************************************************//**
 * @brief This function is used to retrieve the current alarm selection
 * configuration from the specified ADIS device. It should be called
 * after the device has been properly initialized and configured. The
 * function will write the retrieved alarm selection value into the
 * provided pointer. If the pointer is null or if the device is not
 * properly initialized, the function may return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which to read the alarm selection. Must not be null and should
 * point to a valid initialized device.
 * @param alarm_selection Pointer to a `uint32_t` variable where the alarm
 * selection value will be stored. Must not be null; if
 * it is null, the function will return an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_alarm_selection(struct adis_dev *adis, uint32_t *alarm_selection);
/***************************************************************************//**
 * @brief This function is used to configure the alarm selection settings for
 * the ADIS device. It should be called after the device has been
 * initialized and is ready for configuration. The function will write
 * the specified alarm selection value to the appropriate register, and
 * it is important to ensure that the value is valid according to the
 * device's specifications. If the write operation fails, an error code
 * will be returned, indicating the nature of the failure. Additionally,
 * a small delay is introduced after the write operation to allow the
 * device to process the change.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param alarm_selection A 32-bit unsigned integer representing the alarm
 * selection configuration. The valid range and specific
 * values depend on the device's documentation. If an
 * invalid value is provided, the function may return an
 * error code.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_alarm_selection(struct adis_dev *adis, uint32_t alarm_selection);
/***************************************************************************//**
 * @brief This function retrieves the current alarm polarity setting from the
 * specified ADIS device. It should be called after the device has been
 * properly initialized and configured. The caller must ensure that the
 * `adis` pointer is valid and points to an initialized `adis_dev`
 * structure. The `alarm_polarity` pointer must not be null, as it will
 * be used to store the retrieved value. If the function encounters an
 * error during the read operation, it will return a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device from
 * which to read the alarm polarity. This pointer must not be null
 * and should point to a valid, initialized device.
 * @param alarm_polarity Pointer to a `uint32_t` variable where the alarm
 * polarity value will be stored. This pointer must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_alarm_polarity(struct adis_dev *adis, uint32_t *alarm_polarity);
/***************************************************************************//**
 * @brief This function is used to configure the alarm polarity for the
 * specified device. It should be called after the device has been
 * initialized. The `alarm_polarity` parameter must be either 0 or 1,
 * where 0 typically represents active low and 1 represents active high.
 * If an invalid value is provided, the function will return an error
 * code. After successfully writing the alarm polarity, the function
 * introduces a small delay to ensure the device registers the change.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param alarm_polarity An unsigned 32-bit integer representing the desired
 * alarm polarity. Valid values are 0 or 1. If a value
 * greater than 1 is provided, the function will return an
 * error code.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if the write operation fails.
 ******************************************************************************/
int adis_write_alarm_polarity(struct adis_dev *adis, uint32_t alarm_polarity);
/***************************************************************************//**
 * @brief This function retrieves the current alarm enable status from the
 * specified device. It should be called after the device has been
 * properly initialized. The function expects a valid pointer to a
 * `struct adis_dev` representing the device and a pointer to a
 * `uint32_t` variable where the alarm enable status will be stored. If
 * the device is not initialized or if the provided pointers are invalid,
 * the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device from
 * which to read the alarm enable status. This pointer must not be
 * null and should point to a valid initialized device.
 * @param alarm_enable A pointer to a `uint32_t` variable where the alarm enable
 * status will be stored. This pointer must not be null;
 * otherwise, the function will not be able to write the
 * status.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adis_read_alarm_enable(struct adis_dev *adis, uint32_t *alarm_enable);
/***************************************************************************//**
 * @brief This function is used to configure the alarm feature of the device. It
 * should be called after the device has been initialized. The
 * `alarm_enable` parameter determines whether the alarm is enabled (1)
 * or disabled (0). If an invalid value is provided (greater than 1), the
 * function will return an error. After successfully writing the value,
 * the function introduces a small delay to ensure the device registers
 * the change.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param alarm_enable An integer value that specifies the alarm state: 0 to
 * disable or 1 to enable the alarm. Must be either 0 or 1;
 * any other value will result in an error.
 * @return Returns 0 on success, or a negative error code if the input value is
 * invalid or if the write operation fails.
 ******************************************************************************/
int adis_write_alarm_enable(struct adis_dev *adis, uint32_t alarm_enable);
/***************************************************************************//**
 * @brief This function is used to retrieve the direction configuration of a
 * specified GPIO pin on the device. It should be called after the device
 * has been properly initialized. The `dio_nb` parameter specifies which
 * GPIO pin to read, and it must be in the range of 0 to 3. If an invalid
 * pin number is provided, the function will return an error code. The
 * direction value will be written to the `dir` pointer, which must not
 * be null.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param dio_nb The GPIO pin number to read, which must be between 0 and 3
 * inclusive. If it is greater than 3, the function will return an
 * error.
 * @param dir Pointer to a `uint32_t` where the direction value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int adis_read_gpio_dir(struct adis_dev *adis, uint8_t dio_nb, uint32_t *dir);
/***************************************************************************//**
 * @brief This function is used to set the direction of a specified GPIO pin on
 * the device. It should be called after the device has been initialized.
 * The `dio_nb` parameter specifies which GPIO pin to configure, and the
 * `dir` parameter indicates the desired direction, where 0 typically
 * represents input and 1 represents output. If the specified pin number
 * is out of range (greater than 3) or if the direction value is invalid
 * (greater than 1), the function will return an error code. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param dio_nb The GPIO pin number to configure, which must be in the range of
 * 0 to 3. If the value is out of range, the function will return
 * an error.
 * @param dir The desired direction for the GPIO pin, where 0 typically means
 * input and 1 means output. If the value is greater than 1, the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_gpio_dir(struct adis_dev *adis, uint8_t dio_nb, uint32_t dir);
/***************************************************************************//**
 * @brief This function is used to read the logical level (high or low) of a
 * specified GPIO pin on the device. It should be called after the device
 * has been properly initialized. The function expects the GPIO pin
 * number to be in the range of 0 to 3, inclusive. If an invalid pin
 * number is provided, the function will return an error code. The level
 * read from the GPIO pin is written to the provided pointer, which must
 * not be null.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param dio_nb The GPIO pin number to read, which must be between 0 and 3. If
 * the value is outside this range, the function will return an
 * error.
 * @param level Pointer to a `uint32_t` where the read level will be stored. The
 * caller retains ownership of this pointer, and it must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_gpio_lvl(struct adis_dev *adis, uint8_t dio_nb, uint32_t *level);
/***************************************************************************//**
 * @brief This function is used to set the level of a specified GPIO pin on the
 * device. It should be called after the device has been properly
 * initialized. The function expects the GPIO number to be in the range
 * of 0 to 3 and the level to be either 0 or 1. If the provided GPIO
 * number or level is out of range, the function will return an error
 * code. It is important to ensure that the device is ready to accept
 * GPIO level changes before invoking this function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param dio_nb The GPIO number to be set, which must be between 0 and 3. If
 * the value is outside this range, the function will return an
 * error.
 * @param level The desired level to set for the GPIO pin, which must be either
 * 0 or 1. If the value is not 0 or 1, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_gpio_lvl(struct adis_dev *adis, uint8_t dio_nb, uint32_t level);
/***************************************************************************//**
 * @brief This function is used to retrieve the internal sensor bandwidth
 * setting from the specified device. It should be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the bandwidth value will be stored. If
 * the provided pointers are null or if the device is not initialized
 * correctly, the function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param sens_bw Pointer to a `uint32_t` variable where the sensor bandwidth
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_sens_bw(struct adis_dev *adis, uint32_t *sens_bw);
/***************************************************************************//**
 * @brief This function is used to configure the sensor bandwidth of the
 * specified device. It should be called after the device has been
 * initialized and is ready for configuration. The function writes the
 * specified bandwidth value to the device and waits for a defined period
 * to ensure the update is applied. If the provided bandwidth value is
 * invalid or if the device is not properly initialized, the function may
 * return an error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param sens_bw The desired sensor bandwidth value to be set. This value
 * should be within the valid range defined by the device
 * specifications. If the value is out of range, the function
 * will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_sens_bw(struct adis_dev *adis, uint32_t sens_bw);
/***************************************************************************//**
 * @brief This function retrieves the current status of the accelerometer FIR
 * filter enable bit. It should be called after the device has been
 * initialized and is ready for communication. The caller must ensure
 * that the `adis` pointer is valid and points to an initialized
 * `adis_dev` structure. The function will write the retrieved status
 * into the provided `accl_fir_enable` pointer, which must not be null.
 * If the operation fails, an error code will be returned.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to an initialized device.
 * @param accl_fir_enable Pointer to a `uint32_t` where the FIR filter enable
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_accl_fir_enable(struct adis_dev *adis, uint32_t *accl_fir_enable);
/***************************************************************************//**
 * @brief This function is used to configure the accelerometer FIR filter by
 * setting its enable state. It should be called after the device has
 * been initialized and is ready for configuration. The `accl_fir_enable`
 * parameter determines whether the FIR filter is enabled or disabled. If
 * an invalid value is provided, the function will handle it gracefully,
 * ensuring that the device remains in a valid state.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param accl_fir_enable An unsigned 32-bit integer that specifies the desired
 * state of the FIR filter (enabled or disabled). Valid
 * values are typically 0 (disabled) or 1 (enabled). The
 * function will handle any invalid values appropriately.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_accl_fir_enable(struct adis_dev *adis, uint32_t accl_fir_enable);
/***************************************************************************//**
 * @brief This function is used to retrieve the current status of the gyroscope
 * FIR filter enable bit. It should be called after the device has been
 * properly initialized and configured. The function will populate the
 * provided pointer with the enable status, which indicates whether the
 * FIR filter is currently enabled or disabled. It is important to ensure
 * that the pointer passed to this function is valid and points to a
 * memory location that can store the result.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid device that has been initialized.
 * @param gyro_fir_enable A pointer to a `uint32_t` variable where the function
 * will store the FIR filter enable status. This pointer
 * must not be null and should point to a valid memory
 * location.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the status of the read operation.
 ******************************************************************************/
int adis_read_gyro_fir_enable(struct adis_dev *adis, uint32_t *gyro_fir_enable);
/***************************************************************************//**
 * @brief This function is used to configure the gyroscope FIR filter by
 * enabling or disabling it based on the provided parameter. It should be
 * called after the device has been initialized and is ready for
 * configuration. The `gyro_fir_enable` parameter determines whether the
 * FIR filter is active or not, and it is important to ensure that the
 * device is not locked before making this configuration. If an invalid
 * value is provided, the function will handle it appropriately, ensuring
 * that the device state remains consistent.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null and the caller retains ownership.
 * @param gyro_fir_enable A 32-bit unsigned integer that specifies the FIR
 * filter enable state. Valid values are typically 0
 * (disable) or 1 (enable). The function will handle any
 * invalid values gracefully.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_gyro_fir_enable(struct adis_dev *adis, uint32_t gyro_fir_enable);
/***************************************************************************//**
 * @brief This function is used to read the point of percussion alignment value
 * from the device. It should be called after the device has been
 * properly initialized and configured. The function expects a valid
 * pointer to a `uint32_t` variable where the read value will be stored.
 * If the provided pointer is null, the function will not perform the
 * read operation and may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid device that has been initialized.
 * @param pt_of_perc_algnmt A pointer to a `uint32_t` variable where the point
 * of percussion alignment value will be stored. This
 * pointer must not be null; otherwise, the function
 * will not perform the read operation.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the reason for the failure.
 ******************************************************************************/
int adis_read_pt_of_perc_algnmt(struct adis_dev *adis,
				uint32_t *pt_of_perc_algnmt);
/***************************************************************************//**
 * @brief This function is used to set the point of percussion alignment for the
 * specified device. It should be called after the device has been
 * properly initialized. The function will write the provided value to
 * the appropriate register and introduce a delay to ensure the register
 * update is completed. If the provided value is invalid or if there is
 * an error during the write operation, the function will return an error
 * code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param pt_of_perc_algnmt The point of percussion alignment value to be
 * written. This value should be within the valid range
 * defined by the device specifications. The function
 * will handle invalid values by returning an error
 * code.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_pt_of_perc_algnmt(struct adis_dev *adis,
				 uint32_t pt_of_perc_algnmt);
/***************************************************************************//**
 * @brief This function retrieves the linear acceleration compensation value
 * from the specified device. It should be called after the device has
 * been properly initialized and configured. The caller must ensure that
 * the `adis` pointer is valid and points to an initialized `adis_dev`
 * structure. The `linear_accl_comp` pointer must not be null, as it will
 * be used to store the retrieved value. If the function encounters an
 * error during the read operation, it will return a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to an initialized device.
 * @param linear_accl_comp Pointer to a `uint32_t` variable where the linear
 * acceleration compensation value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_linear_accl_comp(struct adis_dev *adis,
			       uint32_t *linear_accl_comp);
/***************************************************************************//**
 * @brief This function is used to set the linear acceleration compensation
 * value for the specified device. It should be called after the device
 * has been properly initialized. The function will write the provided
 * compensation value to the device's register and introduce a delay to
 * ensure the register update is completed. If the provided value is
 * invalid or if there is an error during the write operation, the
 * function will return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param linear_accl_comp The linear acceleration compensation value to be
 * written. This value should be a valid 32-bit unsigned
 * integer.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_linear_accl_comp(struct adis_dev *adis,
				uint32_t linear_accl_comp);
/***************************************************************************//**
 * @brief This function is used to read the current burst selection setting from
 * the specified device. It should be called after the device has been
 * initialized and is ready for communication. The burst selection value
 * indicates which type of burst data the device will output. If the
 * provided pointer for the burst selection is null, the function will
 * return an error. Additionally, if the read operation fails, an error
 * code will be returned.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param burst_sel Pointer to a `uint32_t` variable where the burst selection
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_burst_sel(struct adis_dev *adis, uint32_t *burst_sel);
/***************************************************************************//**
 * @brief This function is used to set the burst selection mode for the ADIS
 * device, which determines the type of data that will be collected
 * during burst operations. It should be called after the device has been
 * initialized and before any burst data reads are performed. The
 * function introduces a delay to ensure that the device has time to
 * process the new setting. If the provided burst selection value is
 * invalid, the function will return an error code, and the device's
 * burst selection will remain unchanged.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param burst_sel The burst selection value to be written. Valid values depend
 * on the device specifications and should be checked against
 * the documentation. The function will return an error if the
 * value is invalid.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_burst_sel(struct adis_dev *adis, uint32_t burst_sel);
/***************************************************************************//**
 * @brief This function is used to read burst data from the specified device. It
 * should be called after the device has been properly initialized. The
 * function retrieves the burst data and updates the internal state of
 * the device based on the retrieved value. If the read operation fails,
 * an error code is returned, indicating the type of failure. It is
 * important to ensure that the device is ready for reading before
 * invoking this function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param burst32 Pointer to a `uint32_t` variable where the burst data will be
 * stored. Caller retains ownership and must ensure it is valid.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_burst32(struct adis_dev *adis, uint32_t *burst_size);
/***************************************************************************//**
 * @brief This function is used to configure the burst mode of the device by
 * writing a 32-bit value. It should be called after the device has been
 * initialized. The `burst32` parameter determines whether the burst mode
 * is enabled or disabled, where a value of 1 enables it and a value of 0
 * disables it. After writing the value, the function introduces a delay
 * to allow the device to process the update. If the provided `adis`
 * pointer is null or if the write operation fails, the function will
 * return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param burst32 A 32-bit unsigned integer representing the burst mode setting.
 * Valid values are 0 (disabled) and 1 (enabled). The function
 * will handle invalid values by clamping them to these valid
 * options.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_burst32(struct adis_dev *adis, uint32_t burst_size);
/***************************************************************************//**
 * @brief This function retrieves the current timestamp from the specified
 * device. It should be called after the device has been properly
 * initialized and configured. The timestamp is stored in the memory
 * location pointed to by the `timestamp32` parameter. If the device is
 * not ready or if an error occurs during the read operation, the
 * function will return an error code, allowing the caller to handle the
 * situation appropriately.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been initialized.
 * @param timestamp32 A pointer to a `uint32_t` variable where the read
 * timestamp will be stored. This pointer must not be null,
 * and the caller retains ownership of the memory it points
 * to.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_timestamp32(struct adis_dev *adis, uint32_t *timestamp32);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit timestamp value to the
 * specified device. It should be called after the device has been
 * properly initialized. The function ensures that the timestamp is
 * written correctly and introduces a delay to allow the device to
 * process the update. If the provided `adis` pointer is null or if the
 * write operation fails, an error code will be returned.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param timestamp32 A 32-bit unsigned integer representing the timestamp to be
 * written. Valid values are any 32-bit unsigned integer.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_timestamp32(struct adis_dev *adis, uint32_t timestamp32);
/***************************************************************************//**
 * @brief This function is used to retrieve the 4kHz synchronization value from
 * the specified ADIS device. It should be called after the device has
 * been properly initialized and configured. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the synchronization value will be
 * stored. If the device is not initialized or if the provided pointers
 * are invalid, the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` representing the device from
 * which to read the synchronization value. Must not be null and
 * should point to a valid initialized device.
 * @param sync_4khz A pointer to a `uint32_t` variable where the read
 * synchronization value will be stored. Must not be null; if
 * it is null, the function will not write any value.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int adis_read_sync_4khz(struct adis_dev *adis, uint32_t *sync_4khz);
/***************************************************************************//**
 * @brief This function is used to set the synchronization frequency of the
 * device to either 2000 Hz or 4000 Hz based on the provided parameter.
 * It should be called after the device has been initialized and
 * configured. The function updates the internal clock frequency
 * accordingly and introduces a delay to ensure the register update is
 * completed. If an invalid value is provided, the function will return
 * an error code, and the internal state of the device will remain
 * unchanged.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param sync_4khz An unsigned 32-bit integer that specifies the desired
 * synchronization frequency. Valid values are 0 (2000 Hz) or 1
 * (4000 Hz). Any other value will result in an error.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adis_write_sync_4khz(struct adis_dev *adis, uint32_t sync_4khz);

/***************************************************************************//**
 * @brief This function is used to retrieve the current external clock frequency
 * multiplier from the specified ADIS device. It should be called after
 * the device has been properly initialized. The function expects a valid
 * pointer to a `uint32_t` variable where the retrieved multiplier value
 * will be stored. If the provided pointer is null or if the device is
 * not initialized correctly, the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the clock frequency multiplier. This pointer must
 * not be null and should point to a valid initialized device.
 * @param up_scale A pointer to a `uint32_t` variable where the external clock
 * frequency multiplier will be stored. This pointer must not be
 * null; otherwise, the function will not be able to write the
 * value.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_up_scale(struct adis_dev *adis, uint32_t *up_scale);
/***************************************************************************//**
 * @brief This function is used to set the external clock frequency multiplier
 * for the device. It should be called after the device has been
 * initialized and is ready for configuration. If the device is in
 * SYNC_SCALED synchronization mode, the function ensures that the
 * product of the external clock frequency and the multiplier is within
 * the specified sampling clock limits. If the conditions are not met, an
 * error is returned. It is important to handle the return value properly
 * to ensure that the operation was successful.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null.
 * @param up_scale A 32-bit unsigned integer representing the external clock
 * frequency multiplier. Valid values depend on the device's
 * specifications. If the device is in SYNC_SCALED mode, the
 * function checks that the resulting clock frequency is within
 * the allowed limits.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_up_scale(struct adis_dev *adis, uint32_t up_scale);

/***************************************************************************//**
 * @brief This function is used to retrieve the current decimation rate setting
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized and configured. The function will write
 * the retrieved decimation rate value into the provided pointer. If the
 * pointer is null or if the device is not properly initialized, the
 * function may return an error.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device
 * from which the decimation rate is to be read. This pointer must
 * not be null and should point to a valid initialized device.
 * @param dec_rate A pointer to a `uint32_t` variable where the read decimation
 * rate will be stored. This pointer must not be null;
 * otherwise, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_dec_rate(struct adis_dev *adis, uint32_t *dec_rate);
/***************************************************************************//**
 * @brief This function is used to set the decimation rate for the device, which
 * controls the rate at which data is sampled. It should be called after
 * the device has been initialized and before any data acquisition
 * begins. The function checks if the provided decimation rate exceeds
 * the maximum allowed value for the device; if it does, an error is
 * returned. After successfully writing the decimation rate, the function
 * introduces a delay to allow the device to update its settings.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null.
 * @param dec_rate The desired decimation rate to be set. It must be less than
 * or equal to the maximum decimation rate supported by the
 * device. If the value exceeds this limit, the function will
 * return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_write_dec_rate(struct adis_dev *adis, uint32_t dec_rate);

/***************************************************************************//**
 * @brief This function is used to read the time bias control value from the
 * specified ADIS device. It should be called after the device has been
 * properly initialized and configured. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the read value will be stored. If the
 * provided pointers are null or if the device is not initialized, the
 * function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param bias_corr_tbc Pointer to a `uint32_t` variable where the time bias
 * control value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_tbc(struct adis_dev *adis, uint32_t *bias_corr_tbc);
/***************************************************************************//**
 * @brief This function is used to set the time bias control value for the
 * specified device. It should be called after the device has been
 * properly initialized. The input value must be within the valid range
 * defined by the device's maximum bias correction time constant. If the
 * provided value exceeds this maximum, the function will return an error
 * code, indicating that the input is invalid.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * Must not be null.
 * @param bias_corr_tbc A 32-bit unsigned integer representing the time bias
 * control value to be set. It must be less than or equal
 * to the maximum bias correction time constant defined in
 * the device's information structure.
 * @return Returns 0 on success, or a negative error code if the input value is
 * invalid.
 ******************************************************************************/
int adis_write_bias_corr_tbc(struct adis_dev *adis, uint32_t bias_corr_tbc);
/***************************************************************************//**
 * @brief This function is used to read the status of the bias correction enable
 * bit for the X-axis gyroscope in an ADIS device. It should be called
 * after the device has been properly initialized. The function will
 * populate the provided pointer with the current value of the bias
 * correction enable bit, which indicates whether bias correction is
 * enabled or disabled. If the provided pointer is null, the function
 * will not perform the read operation and may return an error.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_xg Pointer to a `uint32_t` where the bias correction
 * enable status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_xg(struct adis_dev *adis, uint32_t *bias_corr_en_xg);
/***************************************************************************//**
 * @brief This function is used to configure the bias correction feature for the
 * X-axis gyroscope in the ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `bias_corr_en_xg` parameter determines whether the bias correction is
 * enabled or disabled. If an invalid value is provided, the function
 * will handle it gracefully, ensuring that the device remains in a valid
 * state.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_xg A 32-bit unsigned integer that specifies the bias
 * correction enable state. Valid values are typically 0
 * (disable) or 1 (enable). The function will handle out-
 * of-range values appropriately.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_bias_corr_en_xg(struct adis_dev *adis, uint32_t bias_corr_en_xg);
/***************************************************************************//**
 * @brief This function is used to retrieve the current state of the Y-axis
 * gyroscope bias correction enable bit from the device. It should be
 * called after the device has been properly initialized. The function
 * will write the retrieved value into the provided pointer. If the
 * pointer is null or if the device is not initialized, the function may
 * return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null and should be initialized before calling this
 * function.
 * @param bias_corr_en_yg A pointer to a `uint32_t` variable where the function
 * will store the Y-axis gyroscope bias correction enable
 * bit value. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_yg(struct adis_dev *adis, uint32_t *bias_corr_en_yg);
/***************************************************************************//**
 * @brief This function is used to enable or disable the bias correction for the
 * Y-axis gyroscope in the ADIS device. It should be called after the
 * device has been initialized and configured. The `bias_corr_en_yg`
 * parameter determines whether the bias correction is enabled or
 * disabled. If the value is not valid, the function will handle it
 * appropriately, ensuring that the device state remains consistent.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device.
 * Must not be null.
 * @param bias_corr_en_yg A 32-bit unsigned integer that specifies the bias
 * correction enable state for the Y-axis gyroscope.
 * Valid values are typically 0 (disable) or 1 (enable).
 * The function will handle invalid values gracefully.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_bias_corr_en_yg(struct adis_dev *adis, uint32_t bias_corr_en_yg);
/***************************************************************************//**
 * @brief This function is used to retrieve the status of the Z-axis gyroscope
 * bias correction enable bit from the device. It should be called after
 * the device has been properly initialized. The function will write the
 * retrieved value into the provided pointer. If the pointer is null, the
 * function will not perform any operation and may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param bias_corr_en_zg A pointer to a `uint32_t` variable where the function
 * will store the retrieved bias correction enable
 * status. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_zg(struct adis_dev *adis, uint32_t *bias_corr_en_zg);
/***************************************************************************//**
 * @brief This function is used to configure the bias correction feature for the
 * Z-axis gyroscope of the ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `bias_corr_en_zg` parameter determines whether the bias correction is
 * enabled or disabled. If an invalid value is provided, the function
 * will handle it gracefully, ensuring that the device remains in a safe
 * state.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_zg A 32-bit unsigned integer that specifies the bias
 * correction enable state. Valid values are typically 0
 * (disable) or 1 (enable). The function will handle out-
 * of-range values appropriately.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_bias_corr_en_zg(struct adis_dev *adis, uint32_t bias_corr_en_zg);
/***************************************************************************//**
 * @brief This function is used to retrieve the current state of the bias
 * correction enable bit for the X-axis accelerometer. It should be
 * called after the device has been properly initialized. The function
 * will write the retrieved value into the provided pointer. If the
 * pointer is null or if the device is not initialized correctly, the
 * function may return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param bias_corr_en_xa Pointer to a `uint32_t` variable where the bias
 * correction enable state will be stored. Must not be
 * null; if it is null, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_xa(struct adis_dev *adis, uint32_t *bias_corr_en_xa);
/***************************************************************************//**
 * @brief This function is used to configure the bias correction feature for the
 * X-axis accelerometer in the ADIS device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `bias_corr_en_xa` parameter determines whether the bias correction is
 * enabled or disabled. If an invalid value is provided, the function
 * will handle it gracefully, ensuring that the device state remains
 * consistent.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_xa A 32-bit unsigned integer that specifies the bias
 * correction enable state. Valid values are typically 0
 * (disable) or 1 (enable). The function will handle any
 * invalid values appropriately.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_bias_corr_en_xa(struct adis_dev *adis, uint32_t bias_corr_en_xa);
/***************************************************************************//**
 * @brief This function is used to read the bias correction enable status for
 * the Y-axis accelerometer in an ADIS device. It should be called after
 * the device has been properly initialized. The function will populate
 * the provided pointer with the current enable status, which indicates
 * whether bias correction is active or not. If the provided pointer is
 * null, the function will not perform any operation and may return an
 * error code. It is important to handle the return value to check for
 * any potential errors during the read operation.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param bias_corr_en_ya A pointer to a `uint32_t` variable where the bias
 * correction enable status will be stored. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_ya(struct adis_dev *adis, uint32_t *bias_corr_en_ya);
/***************************************************************************//**
 * @brief This function is used to configure the bias correction feature for the
 * Y-axis accelerometer of the device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `bias_corr_en_ya` parameter determines whether the bias correction is
 * enabled or disabled. It is important to ensure that the device is not
 * locked before calling this function, as configuration changes are not
 * allowed when the device is locked.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_ya A 32-bit unsigned integer that specifies the bias
 * correction enable state. Valid values are typically 0
 * (disable) or 1 (enable). The function will handle
 * invalid values by clamping or ignoring them as per the
 * device's specifications.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_bias_corr_en_ya(struct adis_dev *adis, uint32_t bias_corr_en_ya);
/***************************************************************************//**
 * @brief This function is used to read the status of the Z-axis accelerometer
 * bias correction enable bit from the device. It should be called after
 * the device has been properly initialized. The function will populate
 * the provided pointer with the value of the bias correction enable bit,
 * which indicates whether the bias correction for the Z-axis
 * accelerometer is enabled or disabled. If the provided pointer is null,
 * the function will not perform any operation and will return an error
 * code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_za Pointer to a `uint32_t` variable where the bias
 * correction enable status will be stored. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_bias_corr_en_za(struct adis_dev *adis, uint32_t *bias_corr_en_za);
/***************************************************************************//**
 * @brief This function is used to enable or disable the bias correction for the
 * Z-axis accelerometer in the ADIS device. It should be called after the
 * device has been initialized and configured. The function takes a
 * pointer to the `adis_dev` structure, which represents the device, and
 * a 32-bit integer that specifies the desired state of the bias
 * correction enable bit. If the provided pointer is null, the function
 * will not perform any operation and will return an error code.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param bias_corr_en_za 32-bit integer representing the desired state of the
 * Z-axis accelerometer bias correction enable bit. Valid
 * values are typically 0 (disable) or 1 (enable). The
 * function will handle invalid values by writing them as
 * is, but the expected values should be adhered to for
 * correct operation.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int adis_write_bias_corr_en_za(struct adis_dev *adis, uint32_t bias_corr_en_za);

/***************************************************************************//**
 * @brief This function is intended to be called when a bias correction update
 * is required for the device. It should be invoked after the device has
 * been properly initialized and configured. The function will execute
 * the necessary command to update the bias correction settings in the
 * device. It is important to ensure that the device is in a state that
 * allows for this operation, as calling it in an inappropriate state may
 * lead to undefined behavior.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been initialized. If the pointer is null or points to an
 * uninitialized device, the function may not behave as expected.
 * @return Returns an integer value indicating the success or failure of the
 * command execution. A return value of 0 typically indicates success,
 * while a negative value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int adis_cmd_bias_corr_update(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function should be called when there is a need to revert the
 * device to its factory calibration settings, typically after a
 * calibration process or if the device is not performing as expected. It
 * is important to ensure that the device is properly initialized before
 * calling this function. The function will write to the appropriate
 * register to initiate the restoration process and will block for a
 * specified duration to allow the operation to complete. If the write
 * operation fails, an error code will be returned.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been initialized. If the pointer is invalid or the
 * device is not initialized, the function may return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_cmd_fact_calib_restore(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to initiate a self-test procedure for the
 * sensor. It should be called when the sensor is properly initialized
 * and ready for operation. The function writes to the appropriate
 * register to trigger the self-test and then waits for a specified
 * duration to allow the test to complete. It is important to ensure that
 * the sensor is in a state that allows for self-testing, as calling this
 * function in an inappropriate state may lead to undefined behavior.
 *
 * @param adis Pointer to an `adis_dev` structure representing the sensor
 * device. This pointer must not be null and should point to a valid
 * device that has been initialized properly.
 * @return Returns 0 on success, indicating that the self-test was initiated
 * successfully. On failure, it returns a negative error code indicating
 * the type of error encountered.
 ******************************************************************************/
int adis_cmd_snsr_self_test(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to initiate a flash memory update for the
 * device. It must be called after the device has been properly
 * initialized. The function writes to the appropriate register to
 * trigger the update and then waits for a specified duration to allow
 * the update process to complete. After the delay, it reads the flash
 * memory write counter to ensure the update was successful. If the
 * update fails, the function will return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and must point to a valid device
 * that has been initialized.
 * @return Returns an integer value indicating the status of the operation. A
 * return value of 0 indicates success, while a negative value indicates
 * an error occurred during the update process.
 ******************************************************************************/
int adis_cmd_fls_mem_update(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to perform a flash memory test on the specified
 * device. It should be called after the device has been properly
 * initialized. The function writes to the flash memory and then waits
 * for a specified duration to allow the operation to complete. If the
 * write operation fails, an error code is returned. It is important to
 * ensure that the device is in a suitable state for performing this
 * operation to avoid unexpected behavior.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device to
 * be tested. This pointer must not be null and should point to a
 * valid device that has been initialized.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_cmd_fls_mem_test(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to clear the FIFO buffer of the device, ensuring
 * that any data stored in the buffer is discarded. It should be called
 * when the user wants to reset the FIFO state, typically before starting
 * a new data acquisition session. The function expects that the device
 * has been properly initialized and is ready for command execution.
 * Calling this function when the device is not initialized or in an
 * invalid state may lead to undefined behavior.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been initialized. If the pointer is null or points to an
 * uninitialized device, the behavior of the function is undefined.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the command
 * execution.
 ******************************************************************************/
int adis_cmd_fifo_flush(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to initiate a software reset of the device,
 * which is typically required to restore the device to a known state. It
 * should be called when the device is in an operational state and may be
 * necessary after certain configuration changes or when troubleshooting
 * issues. The function will wait for a specified duration to allow the
 * reset process to complete before returning. It is important to ensure
 * that the device is not locked before calling this function, as a
 * locked device may not accept configuration changes.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device to
 * be reset. This pointer must not be null, and the caller retains
 * ownership of the structure. If the device is locked, the function
 * will not perform the reset.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int adis_cmd_sw_res(struct adis_dev *adis);
/***************************************************************************//**
 * @brief This function is used to lock the device, preventing any further
 * configuration changes while allowing data readings. It should be
 * called after the device has been initialized and is ready for
 * operation. If the device is already locked, subsequent calls will have
 * no effect. The function will return an error code if the write
 * operation to the device fails.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device to
 * be locked. This pointer must not be null. If the pointer is null,
 * the function will not perform any operation and will return an
 * error code.
 * @return Returns 0 on success, indicating that the device has been
 * successfully locked. If an error occurs during the write operation, a
 * negative error code will be returned.
 ******************************************************************************/
int adis_cmd_write_lock(struct adis_dev *adis);

/***************************************************************************//**
 * @brief This function is used to retrieve the processor revision of the ADIS
 * device. It should be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * adis_dev` that represents the device and a pointer to a `uint32_t`
 * variable where the processor revision will be stored. If the device is
 * not initialized or if the provided pointers are invalid, the function
 * may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and should point to a valid initialized device.
 * @param proc_rev Pointer to a `uint32_t` where the processor revision will be
 * stored. Must not be null; if it is null, the function will
 * not write the revision.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_proc_rev(struct adis_dev *adis, uint32_t *proc_rev);
/***************************************************************************//**
 * @brief This function retrieves the firmware revision of the specified ADIS
 * device. It should be called after the device has been initialized and
 * is ready for communication. The caller must ensure that the `adis`
 * pointer is valid and points to an initialized `adis_dev` structure.
 * The function will write the firmware revision value to the location
 * pointed to by the `firm_rev` parameter, which must not be null. If the
 * operation fails, the function will return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read the firmware revision. This pointer must not be
 * null and should point to a valid, initialized device.
 * @param firm_rev A pointer to a `uint32_t` variable where the firmware
 * revision will be stored. This pointer must not be null;
 * otherwise, the function will not be able to write the
 * firmware revision.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_firm_rev(struct adis_dev *adis, uint32_t *firm_rev);
/***************************************************************************//**
 * @brief This function is used to retrieve the firmware revision data from the
 * specified ADIS device. It should be called after the device has been
 * properly initialized. The function expects a valid pointer to a
 * `uint32_t` variable where the firmware revision will be stored. If the
 * provided pointer is null, the function will not perform the read
 * operation and may return an error code. It is important to check the
 * return value to ensure that the read operation was successful.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the firmware revision is to be read. This pointer must not
 * be null and should point to a valid initialized device.
 * @param firm_d A pointer to a `uint32_t` variable where the firmware revision
 * data will be stored. This pointer must not be null; otherwise,
 * the function will not perform the read operation.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_firm_d(struct adis_dev *adis, uint32_t *firm_d);
/***************************************************************************//**
 * @brief This function retrieves the firmware revision month from the specified
 * ADIS device. It should be called after the device has been properly
 * initialized and configured. The caller must ensure that the `adis`
 * pointer is valid and points to an initialized `adis_dev` structure.
 * The function will write the retrieved month value into the memory
 * location pointed to by the `firm_m` parameter. If the operation fails,
 * an error code will be returned.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to a valid initialized device.
 * @param firm_m Pointer to a `uint32_t` where the firmware month will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_firm_m(struct adis_dev *adis, uint32_t *firm_m);
/***************************************************************************//**
 * @brief This function retrieves the firmware year from the specified ADIS
 * device. It should be called after the device has been properly
 * initialized. The caller must ensure that the `adis` pointer is valid
 * and points to an initialized `adis_dev` structure. The `firm_y`
 * pointer must not be null, as it will be used to store the retrieved
 * firmware year. If the function encounters an error during the read
 * operation, it will return a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to an initialized device.
 * @param firm_y Pointer to a `uint32_t` variable where the firmware year will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_firm_y(struct adis_dev *adis, uint32_t *firm_y);
/***************************************************************************//**
 * @brief This function is used to retrieve the boot revision of the ADIS
 * device. It should be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * adis_dev` that represents the device and a pointer to a `uint32_t`
 * variable where the boot revision will be stored. If the provided
 * pointers are valid, the function will populate the boot revision
 * value; otherwise, it may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and must point to a valid initialized device.
 * @param boot_rev Pointer to a `uint32_t` where the boot revision will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_boot_rev(struct adis_dev *adis, uint32_t *boot_rev);
/***************************************************************************//**
 * @brief This function is used to retrieve the product identification number
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized. The function expects a valid pointer to
 * a `struct adis_dev` representing the device and a pointer to a
 * `uint32_t` variable where the product ID will be stored. If the device
 * is not initialized or if the provided pointers are invalid, the
 * function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` that represents the device from
 * which the product ID will be read. This pointer must not be null
 * and should point to a valid initialized device.
 * @param prod_id A pointer to a `uint32_t` variable where the product ID will
 * be stored. This pointer must not be null, and the caller
 * retains ownership of the memory it points to.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_prod_id(struct adis_dev *adis, uint32_t *prod_id);
/***************************************************************************//**
 * @brief This function retrieves the serial number of the specified device. It
 * should be called after the device has been properly initialized. The
 * caller must ensure that the `adis` pointer is valid and points to an
 * initialized `adis_dev` structure. The `serial_num` pointer must not be
 * null, as it will be used to store the retrieved serial number. If the
 * function encounters an error during the read operation, it will return
 * a negative error code.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to an initialized device.
 * @param serial_num Pointer to a `uint32_t` where the serial number will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_serial_num(struct adis_dev *adis, uint32_t *serial_num);
/***************************************************************************//**
 * @brief This function retrieves the lot number associated with the ADIS
 * device. It should be called after the device has been initialized and
 * is ready for communication. The caller must ensure that the `lot_num`
 * pointer is valid and points to a memory location where the lot number
 * can be stored. If the function encounters an error during the read
 * operation, it will return a negative value, indicating the type of
 * error that occurred.
 *
 * @param adis A pointer to the `adis_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device.
 * @param lot_num A pointer to a `uint32_t` variable where the lot number will
 * be stored. This pointer must not be null and should point to a
 * valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_lot_num(struct adis_dev *adis, uint32_t *lot_num);

/***************************************************************************//**
 * @brief This function is used to read the value from the user scratch register
 * 1 of the ADIS device. It should be called after the device has been
 * properly initialized and configured. The function expects a valid
 * pointer to a `struct adis_dev` representing the device and a pointer
 * to a `uint32_t` variable where the read value will be stored. If the
 * provided pointers are null or if the device is not initialized, the
 * function may return an error code.
 *
 * @param adis Pointer to a `struct adis_dev` representing the device. Must not
 * be null and must point to a valid initialized device.
 * @param usr_scr_1 Pointer to a `uint32_t` variable where the read value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_read_usr_scr_1(struct adis_dev *adis, uint32_t *usr_scr_1);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to the user scratch
 * register 1 of the specified ADIS device. It should be called after the
 * device has been properly initialized. The function will return an
 * error code if the provided `adis` pointer is null or if the write
 * operation fails for any reason.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param usr_scr_1 A 32-bit unsigned integer value to be written to the user
 * scratch register 1. Valid values are any 32-bit unsigned
 * integer.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_usr_scr_1(struct adis_dev *adis, uint32_t usr_scr_1);
/***************************************************************************//**
 * @brief This function is used to read the value of the user scratch register 2
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized and configured. The function expects a
 * valid pointer to a `struct adis_dev` representing the device and a
 * pointer to a `uint32_t` variable where the read value will be stored.
 * If the provided pointers are null or if the device is not initialized,
 * the function may return an error code.
 *
 * @param adis A pointer to a `struct adis_dev` representing the device from
 * which to read. Must not be null and should point to a valid
 * initialized device.
 * @param usr_scr_2 A pointer to a `uint32_t` variable where the read value will
 * be stored. Must not be null; if it is null, the function
 * will not perform the read operation.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the reason for the failure.
 ******************************************************************************/
int adis_read_usr_scr_2(struct adis_dev *adis, uint32_t *usr_scr_2);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to the user scratch
 * register 2 of the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function will
 * return an error code if the provided `adis` pointer is null or if the
 * write operation fails for any reason. Ensure that the device is not
 * locked before attempting to write to the register, as writes are not
 * permitted when the device is in a locked state.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param usr_scr_2 A 32-bit unsigned integer value to be written to the user
 * scratch register 2. Valid values are determined by the
 * specific application requirements.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_usr_scr_2(struct adis_dev *adis, uint32_t usr_scr_2);
/***************************************************************************//**
 * @brief This function is used to read the value of the user scratch register 3
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized and configured. The function will
 * populate the provided pointer with the value read from the register.
 * If the pointer is null or if the device is not initialized, the
 * function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read. This pointer must not be null and should point to
 * a valid initialized device.
 * @param usr_scr_3 A pointer to a `uint32_t` variable where the read value will
 * be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_usr_scr_3(struct adis_dev *adis, uint32_t *usr_scr_3);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to the user scratch
 * register 3 of the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function does
 * not perform any validation on the `adis` pointer; therefore, it must
 * not be null. If the write operation fails, an error code will be
 * returned, indicating the nature of the failure.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null.
 * @param usr_scr_3 The 32-bit value to be written to the user scratch register
 * 3. Valid values are determined by the device specifications.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_usr_scr_3(struct adis_dev *adis, uint32_t usr_scr_3);
/***************************************************************************//**
 * @brief This function is used to read the value of the user scratch register 4
 * from the specified ADIS device. It should be called after the device
 * has been properly initialized and configured. The function will
 * populate the provided pointer with the value read from the register.
 * If the pointer is null or if the device is not initialized correctly,
 * the function may return an error code.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which to read. This pointer must not be null and should point to
 * a valid initialized device.
 * @param usr_scr_4 A pointer to a `uint32_t` variable where the read value will
 * be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adis_read_usr_scr_4(struct adis_dev *adis, uint32_t *usr_scr_4);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to the user scratch
 * register 4 of the specified ADIS device. It should be called after the
 * device has been properly initialized and configured. The function will
 * return an error code if the provided `adis` pointer is null or if the
 * write operation fails for any reason. Ensure that the device is not
 * locked before attempting to write to the register, as writes may be
 * restricted when the device is in a locked state.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param usr_scr_4 The 32-bit value to be written to the user scratch register
 * 4. Valid values are determined by the specific application
 * requirements.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_write_usr_scr_4(struct adis_dev *adis, uint32_t usr_scr_4);

/***************************************************************************//**
 * @brief This function is used to retrieve the current value of the flash
 * memory write cycle counter from the specified device. It should be
 * called after the device has been properly initialized. The function
 * will update the value pointed to by the `fls_mem_wr_cntr` parameter
 * with the current count. If the read operation is successful, the
 * function returns 0; otherwise, it returns a negative error code.
 * Additionally, if the retrieved counter exceeds the maximum allowed
 * value, a diagnostic flag will be set to indicate this condition.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device from
 * which the counter is to be read. This pointer must not be null
 * and should point to a valid initialized device.
 * @param fls_mem_wr_cntr A pointer to a `uint32_t` variable where the function
 * will store the retrieved flash memory write cycle
 * counter value. This pointer must not be null and the
 * caller retains ownership of the memory.
 * @return Returns 0 on success, or a negative error code on failure. The value
 * pointed to by `fls_mem_wr_cntr` is updated with the current flash
 * memory write cycle counter.
 ******************************************************************************/
int adis_read_fls_mem_wr_cntr(struct adis_dev *adis, uint32_t *fls_mem_wr_cntr);

/***************************************************************************//**
 * @brief This function retrieves a specific FIR filter coefficient from the
 * device. It should be called after the device has been initialized and
 * configured. The `coef_idx` parameter specifies which coefficient to
 * read, and it must be within the valid range defined by the device's
 * maximum coefficient index. If the index is out of range, the function
 * will return an error code. The retrieved coefficient value is stored
 * in the memory location pointed to by the `coef` parameter.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to read. Valid values are from 0
 * to the maximum index defined in the device's information
 * structure. If the index exceeds this range, the function will
 * return an error.
 * @param coef Pointer to a `uint32_t` where the read coefficient value will be
 * stored. Caller retains ownership and must ensure this pointer is
 * valid and points to allocated memory.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_read_fir_coef(struct adis_dev *adis, uint8_t coef_idx, uint32_t *coef);
/***************************************************************************//**
 * @brief This function is used to write a specific FIR filter coefficient to
 * the device. It should be called after the device has been properly
 * initialized. The `coef_idx` parameter specifies which coefficient to
 * write, and it must be within the valid range defined by the device's
 * maximum coefficient index. If the provided index exceeds this maximum,
 * the function will return an error. The `coef` parameter represents the
 * value to be written as the coefficient. It is important to ensure that
 * the device is ready for configuration before calling this function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param coef_idx Index of the FIR coefficient to write. Valid values are from
 * 0 to the maximum index defined in the device's information
 * structure. If the index is out of range, the function will
 * return an error.
 * @param coef The coefficient value to write. This is a 32-bit unsigned
 * integer. The function does not impose specific constraints on the
 * value, but it should be within the expected range for the device.
 * @return Returns 0 on success, or a negative error code if the index is
 * invalid.
 ******************************************************************************/
int adis_write_fir_coef(struct adis_dev *adis, uint8_t coef_idx, uint32_t coef);

/***************************************************************************//**
 * @brief This function is used to read burst data from an ADIS device, which
 * includes gyroscope, accelerometer, and temperature data. It should be
 * called after the device has been properly initialized. The function
 * allows for configuration of burst reading modes, including 32-bit or
 * 16-bit data formats, and the selection of specific data types to read.
 * It also supports options for FIFO data handling and CRC checksum
 * verification. If the device does not support the requested burst mode
 * or if invalid parameters are provided, the function will return an
 * error code. The caller should ensure that the `data` structure is
 * properly allocated before calling this function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param data Pointer to the `adis_burst_data` structure where the read data
 * will be stored. Must not be null.
 * @param burst32 Boolean indicating whether to use 32-bit burst mode. Valid
 * values are true or false.
 * @param burst_sel Selection for the type of burst data to read. Valid values
 * depend on the device's capabilities.
 * @param fifo_pop Boolean indicating whether to pop data from the FIFO. Valid
 * values are true or false.
 * @param crc_check Boolean indicating whether to perform CRC check on the
 * received data. Valid values are true or false.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure.
 ******************************************************************************/
int adis_read_burst_data(struct adis_dev *adis, struct adis_burst_data *data,
			 bool burst32, uint8_t burst_sel, bool fifo_pop, bool crc_check);

/***************************************************************************//**
 * @brief This function is used to set the external clock frequency for the
 * device. It should be called after the device has been initialized and
 * is typically used when the synchronization mode is not set to default
 * or output. If the synchronization mode is set to a specific mode, the
 * provided clock frequency must be within the limits defined for that
 * mode. If the clock frequency is outside these limits, an error will be
 * returned. In other synchronization modes, the clock frequency can be
 * set without restrictions.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param clk_freq The desired external clock frequency in Hertz. Must be a
 * positive integer. If the synchronization mode is not default
 * or output, the frequency must be within the specified limits
 * for the current mode.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_update_ext_clk_freq(struct adis_dev *adis, uint32_t clk_freq);

/***************************************************************************//**
 * @brief This function retrieves the synchronization clock frequency of the
 * ADIS device in Hertz. It should be called after the device has been
 * initialized and configured. The function checks the current
 * synchronization mode and assigns the appropriate clock frequency to
 * the provided pointer. If the synchronization mode is set to default or
 * output, the internal clock frequency is used; otherwise, the external
 * clock frequency is assigned. Ensure that the `clk_freq` pointer is
 * valid and points to a location where the frequency can be stored.
 *
 * @param adis A pointer to an `adis_dev` structure representing the device.
 * Must not be null.
 * @param clk_freq A pointer to a `uint32_t` variable where the clock frequency
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_get_sync_clk_freq(struct adis_dev *adis, uint32_t *clk_freq);

/***************************************************************************//**
 * @brief This function retrieves the scale of the gyroscope in fractional
 * format, which is represented by a dividend and a divisor. It should be
 * called after the device has been properly initialized. Both input
 * parameters must be valid and non-null; otherwise, the function will
 * return an error code. The retrieved scale can be used for further
 * calculations or conversions related to gyroscope data.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param anglvel_scale Pointer to a `adis_scale_fractional` structure where the
 * scale will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_get_anglvel_scale(struct adis_dev *adis,
			   struct adis_scale_fractional *anglvel_scale);

/***************************************************************************//**
 * @brief This function retrieves the acceleration scale from the specified ADIS
 * device and stores it in the provided `accl_scale` structure. It must
 * be called with a valid `adis` device descriptor and a non-null
 * `accl_scale` pointer. If either of these parameters is invalid, the
 * function will return an error code. The retrieved scale is represented
 * as a fraction, with the dividend and divisor populated in the
 * `accl_scale` structure.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device.
 * Must not be null.
 * @param accl_scale Pointer to a `adis_scale_fractional` structure where the
 * acceleration scale will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_get_accl_scale(struct adis_dev *adis,
			struct adis_scale_fractional *accl_scale);

/***************************************************************************//**
 * @brief This function retrieves the scale factor for delta angle measurements
 * from the specified ADIS device. It must be called after the device has
 * been initialized and configured properly. The function expects valid
 * pointers for both the `adis` device structure and the
 * `deltaangl_scale` structure, which will be populated with the scale
 * information. If either pointer is null, the function will return an
 * error code. The retrieved scale is represented as a fractional value
 * in the form of a dividend and a power of two.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device.
 * Must not be null.
 * @param deltaangl_scale Pointer to a `adis_scale_fractional_log2` structure
 * where the scale information will be stored. Must not
 * be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int adis_get_deltaangl_scale(struct adis_dev *adis,
			     struct adis_scale_fractional_log2 *deltaangl_scale);

/***************************************************************************//**
 * @brief This function retrieves the scale for delta velocity measurements from
 * the specified ADIS device. It must be called after the device has been
 * properly initialized. The caller should ensure that the `adis` and
 * `deltavelocity_scale` parameters are valid and not null. If either
 * parameter is null, the function will return an error code. The
 * retrieved scale is stored in the `deltavelocity_scale` structure,
 * which contains both the dividend and the power of two for the scale.
 *
 * @param adis Pointer to the `adis_dev` structure representing the ADIS device.
 * Must not be null.
 * @param deltavelocity_scale Pointer to a `adis_scale_fractional_log2`
 * structure where the delta velocity scale will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int adis_get_deltavelocity_scale(struct adis_dev *adis,
				 struct adis_scale_fractional_log2 *deltavelocity_scale);

/***************************************************************************//**
 * @brief This function retrieves the temperature scale of the ADIS device in a
 * fractional format, represented by a dividend and divisor. It should be
 * called after the device has been initialized and is ready for
 * operation. If either the `adis` or `temp_scale` parameters are null,
 * the function will return an error code. The caller must ensure that
 * the `temp_scale` structure is properly allocated before passing it to
 * the function.
 *
 * @param adis Pointer to the `adis_dev` structure representing the device. Must
 * not be null.
 * @param temp_scale Pointer to an `adis_scale_fractional` structure where the
 * temperature scale will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adis_get_temp_scale(struct adis_dev *adis,
			struct adis_scale_fractional *temp_scale);

/***************************************************************************//**
 * @brief This function is used to obtain the temperature offset value from the
 * specified device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to an `adis_dev`
 * structure and a pointer to an integer where the temperature offset
 * will be stored. If any of the input pointers are null or if the device
 * does not support getting the offset, the function will return an error
 * code. It is important to check the return value to ensure that the
 * operation was successful.
 *
 * @param adis Pointer to an `adis_dev` structure representing the device. Must
 * not be null and must point to a valid initialized device.
 * @param temp_offset Pointer to an integer where the temperature offset will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adis_get_temp_offset(struct adis_dev *adis, int *temp_offset);
#endif

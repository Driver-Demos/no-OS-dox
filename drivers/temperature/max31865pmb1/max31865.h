/***************************************************************************//**
 *   @file   max31865.h
 *   @brief  Header File of MAX31865 Driver.
 *   @author JSanBuen (jose.sanbuenaventura@analog.com)
 *   @author MSosa (marcpaolo.sosa@analog.com)
********************************************************************************
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
*******************************************************************************/

#ifndef __MAX31865_H__
#define __MAX31865_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define MAX31865_READ_MASK  			0x7F
#define MAX31865_WRITE_MASK 			0x80

#define MAX31865_CONFIG_BIAS 			0x80
#define MAX31865_CONFIG_MODEAUTO 		0x40
#define MAX31865_CONFIG_MODEOFF 		0xBF
#define MAX31865_CONFIG_1SHOT 			0x20
#define MAX31865_CONFIG_3WIRE 			0x10
#define MAX31865_CONFIG_2_4WIRE 		0xEF
#define MAX31865_CONFIG_CLRFAULT_MASK 	0xD3
#define MAX31865_CONFIG_FAULTSTAT 		0x02
#define MAX31865_CONFIG_FILT50HZ 		0x01
#define MAX31865_CONFIG_FILT60HZ 		0xFE

#define MAX31865_CONFIG_REG 			0x00
#define MAX31865_RTDMSB_REG 			0x01
#define MAX31865_RTDLSB_REG 			0x02
#define MAX31865_HFAULTMSB_REG 			0x03
#define MAX31865_HFAULTLSB_REG 			0x04
#define MAX31865_LFAULTMSB_REG 			0x05
#define MAX31865_LFAULTLSB_REG 			0x06
#define MAX31865_FAULTSTAT_REG 			0x07

/***************************************************************************//**
 * @brief The `max31865_dev` structure is a descriptor for the MAX31865 RTD-to-
 * Digital Converter, encapsulating the necessary configuration and
 * communication parameters. It includes a pointer to an SPI descriptor
 * for communication, flags for filter and wiring configuration, and a
 * delay parameter for temperature conversion. This structure is
 * essential for managing the device's settings and operations in a
 * software application.
 *
 * @param comm_desc A pointer to a no_os_spi_desc structure used for SPI
 * communication.
 * @param is_filt_50 A boolean indicating if the 50Hz filter is enabled.
 * @param is_odd_wire A boolean indicating if the sensor is configured for an
 * odd number of wires.
 * @param t_rc_delay An integer representing the delay time for temperature
 * reading conversion.
 ******************************************************************************/
struct max31865_dev {
	struct no_os_spi_desc *comm_desc;
	bool is_filt_50;
	bool is_odd_wire;
	int t_rc_delay;
};

/***************************************************************************//**
 * @brief The `max31865_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the MAX31865 device.
 * It includes the SPI initialization parameters necessary for
 * establishing communication with the device, as well as the reference
 * resistance value used in RTD (Resistance Temperature Detector)
 * measurements. This structure is essential for configuring the device
 * to accurately measure temperature using an RTD sensor.
 *
 * @param spi_init Holds the SPI initialization parameters for communication.
 * @param rtd_rc Represents the reference resistance value for the RTD sensor.
 ******************************************************************************/
struct max31865_init_param {
	struct no_os_spi_init_param spi_init;
	float rtd_rc;
};

/***************************************************************************//**
 * @brief This function initializes a MAX31865 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * descriptor and sets up the SPI communication based on the given
 * parameters. If the initialization is successful, the device descriptor
 * is returned through the provided pointer. The function handles invalid
 * input parameters by returning an error code, and it ensures that the
 * device is properly set up with a default delay if no RC time constant
 * is specified.
 *
 * @param device A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated memory and is responsible for freeing it using
 * max31865_remove.
 * @param init_param A pointer to a max31865_init_param structure containing the
 * initialization parameters. Must not be null. If null, the
 * function returns an error code.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid parameters,
 * -ENOMEM for memory allocation failure).
 ******************************************************************************/
int max31865_init(struct max31865_dev **, struct max31865_init_param *);

/***************************************************************************//**
 * @brief Use this function to release all resources allocated for a MAX31865
 * device when it is no longer needed. This function should be called to
 * clean up after a device has been initialized and used, ensuring that
 * any associated communication descriptors are properly removed and
 * memory is freed. It is important to pass a valid device pointer;
 * otherwise, the function will return an error. This function is
 * typically called during the shutdown or cleanup phase of an
 * application.
 *
 * @param device A pointer to a max31865_dev structure representing the device
 * to be removed. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if there is an error removing the communication descriptor.
 ******************************************************************************/
int max31865_remove(struct max31865_dev *);

/***************************************************************************//**
 * @brief This function updates a register on the MAX31865 device by either
 * applying a bitwise OR or AND operation with a specified update value.
 * It is used to modify the configuration or status of the device by
 * writing to its registers. The function must be called with a valid
 * device structure that has been initialized, and the register address
 * must be within the valid range for the device. The operation mode (OR
 * or AND) is determined by the boolean parameter, allowing flexible
 * register updates. The function returns an error code if the read or
 * write operation fails.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the MAX31865 device.
 * @param reg_update The value to be used for updating the register. It is
 * applied to the current register value using a bitwise
 * operation.
 * @param or_mask A boolean indicating the type of bitwise operation to perform.
 * If true, a bitwise OR is performed; if false, a bitwise AND is
 * performed.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the read or write operation fails.
 ******************************************************************************/
int max31865_reg_update(struct max31865_dev *, uint8_t, uint8_t, bool);

/***************************************************************************//**
 * @brief Use this function to read a specific register from the MAX31865
 * device. It is essential to ensure that the device has been properly
 * initialized before calling this function. The function reads the value
 * from the specified register address and stores it in the provided
 * buffer. It is important to pass a valid register address within the
 * range of defined registers for the MAX31865. If the register address
 * is invalid, the function will return an error code. This function is
 * useful for retrieving configuration or status information from the
 * device.
 *
 * @param device A pointer to an initialized max31865_dev structure. This must
 * not be null, and the device must be properly initialized before
 * use.
 * @param reg_addr The address of the register to read from. Must be within the
 * range of valid register addresses for the MAX31865 (0x00 to
 * 0x07). If the address is outside this range, the function
 * returns an error.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the register
 * address is invalid or if the SPI transfer fails.
 ******************************************************************************/
int max31865_read(struct max31865_dev *, uint8_t, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to write a specific value to a register on the
 * MAX31865 device. It is essential to ensure that the register address
 * is valid and writable; otherwise, the function will return an error.
 * This function should be called only after the device has been properly
 * initialized. It does not allow writing to certain registers, such as
 * the RTD MSB and LSB registers, and will return an error if an attempt
 * is made to write to these addresses. The function communicates with
 * the device over SPI, and the caller must ensure that the device
 * structure is correctly set up with a valid SPI descriptor.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * Must not be null and should be properly initialized with a
 * valid SPI descriptor.
 * @param reg_addr The address of the register to write to. Must be between
 * MAX31865_CONFIG_REG and MAX31865_FAULTSTAT_REG, excluding
 * MAX31865_RTDMSB_REG and MAX31865_RTDLSB_REG. Invalid
 * addresses will result in an error.
 * @param reg_data The data to write to the specified register. This is an 8-bit
 * value.
 * @return Returns 0 on success, or a negative error code if the register
 * address is invalid or if the SPI transfer fails.
 ******************************************************************************/
int max31865_write(struct max31865_dev *, uint8_t, uint8_t);

/** Read fault register value */
int max31865_read_fault(struct max31865_dev *, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to clear any fault conditions that have been
 * detected on the MAX31865 device. This is typically necessary after a
 * fault has been detected and addressed, allowing the device to resume
 * normal operation. The function must be called with a valid device
 * descriptor that has been properly initialized. It returns an error
 * code if the operation fails, which can occur if the device is not
 * properly configured or if communication with the device fails.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * This must be a valid, initialized device descriptor. The
 * function will return an error if this parameter is null or if
 * the device is not properly initialized.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int max31865_clear_fault(struct max31865_dev *);

/***************************************************************************//**
 * @brief This function is used to control the bias voltage of the MAX31865
 * device, which is necessary for accurate temperature measurements. It
 * should be called when the device is initialized and ready for
 * configuration. Enabling the bias voltage is typically required before
 * starting a temperature conversion. Disabling it can be used to save
 * power when the device is not actively measuring. The function updates
 * the device's configuration register to reflect the desired bias state.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param bias_en A boolean value indicating whether to enable (true) or disable
 * (false) the bias voltage. Any non-boolean value is considered
 * invalid and may lead to undefined behavior.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int max31865_enable_bias(struct max31865_dev *, bool);

/***************************************************************************//**
 * @brief Use this function to control the automatic conversion mode of a
 * MAX31865 device. This function should be called when you need to
 * enable or disable the automatic conversion feature, which allows the
 * device to continuously convert temperature readings without manual
 * intervention. Ensure that the device has been properly initialized
 * before calling this function. The function modifies the device's
 * configuration register to reflect the desired mode.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param auto_conv_en A boolean value indicating whether to enable (true) or
 * disable (false) automatic conversion mode.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int max31865_auto_convert(struct max31865_dev *, bool);

/***************************************************************************//**
 * @brief This function is used to set the filter frequency of the MAX31865
 * device to either 50Hz or 60Hz, depending on the provided parameter. It
 * should be called when there is a need to change the noise rejection
 * frequency of the device, which can be useful in environments with
 * specific power line frequencies. The function updates the device's
 * configuration register to reflect the chosen filter setting. It is
 * important to ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param device A pointer to a max31865_dev structure representing the device.
 * Must not be null, and the device should be initialized before
 * use.
 * @param filt_en A boolean value indicating whether to enable the 50Hz filter
 * (true) or the 60Hz filter (false).
 * @return Returns 0 on success or a negative error code if the register update
 * fails.
 ******************************************************************************/
int max31865_enable_50Hz(struct max31865_dev *, bool);

/***************************************************************************//**
 * @brief Use this function to configure the high and low fault detection
 * thresholds for a MAX31865 device. This is typically done to define the
 * temperature range within which the device should operate without
 * triggering a fault. The function must be called with a valid device
 * structure that has been initialized using `max31865_init`. The
 * thresholds are specified as 16-bit unsigned integers, and the function
 * will write these values to the appropriate registers on the device. If
 * the function encounters an error during the write operations, it will
 * return a non-zero error code.
 *
 * @param device A pointer to a `max31865_dev` structure representing the
 * device. Must not be null and should be initialized prior to
 * calling this function.
 * @param lower A 16-bit unsigned integer representing the lower fault
 * threshold. This value is written to the device's lower threshold
 * registers.
 * @param upper A 16-bit unsigned integer representing the upper fault
 * threshold. This value is written to the device's upper threshold
 * registers.
 * @return Returns 0 on success, or a non-zero error code if a write operation
 * fails.
 ******************************************************************************/
int max31865_set_threshold(struct max31865_dev *, uint16_t, uint16_t);

/***************************************************************************//**
 * @brief Use this function to obtain the current lower threshold value set in
 * the MAX31865 device, which is used for fault detection. This function
 * should be called when you need to verify or utilize the lower
 * threshold setting of the device. Ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the read operation fails, indicating that the
 * threshold value could not be retrieved.
 *
 * @param device A pointer to an initialized `max31865_dev` structure
 * representing the MAX31865 device. Must not be null. The caller
 * retains ownership.
 * @param low_threshold A pointer to a `uint16_t` where the lower threshold
 * value will be stored. Must not be null. The function
 * writes the retrieved threshold value to this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max31865_get_lower_threshold(struct max31865_dev*, uint16_t *);

/***************************************************************************//**
 * @brief Use this function to obtain the current upper threshold value set in
 * the MAX31865 device. This function is typically called when you need
 * to verify or log the threshold settings of the device. It requires a
 * valid device structure that has been initialized and a pointer to a
 * variable where the threshold value will be stored. Ensure that the
 * device is properly configured and communication is established before
 * calling this function. The function will return an error code if the
 * read operation fails.
 *
 * @param device A pointer to an initialized 'max31865_dev' structure
 * representing the device. Must not be null. The caller retains
 * ownership.
 * @param up_threshold A pointer to a uint16_t variable where the upper
 * threshold value will be stored. Must not be null. The
 * function writes the threshold value to this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max31865_get_upper_threshold(struct max31865_dev*, uint16_t *);

/***************************************************************************//**
 * @brief Use this function to set the wiring configuration of the MAX31865
 * device to either 3-wire or 2/4-wire mode, depending on the RTD sensor
 * being used. This function should be called after initializing the
 * device with `max31865_init`. The function updates the device's
 * configuration register to reflect the selected wiring mode. It is
 * important to ensure that the `device` parameter is a valid,
 * initialized pointer to a `max31865_dev` structure. The function
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param device A pointer to a `max31865_dev` structure representing the
 * device. Must not be null and should be initialized using
 * `max31865_init` before calling this function. The caller
 * retains ownership.
 * @param is_odd_wire A boolean value indicating the wiring configuration. Set
 * to `true` for 3-wire configuration, or `false` for
 * 2/4-wire configuration.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int max31865_set_wires(struct max31865_dev *, bool);

/***************************************************************************//**
 * @brief Use this function to read the RTD (Resistance Temperature Detector)
 * value from a MAX31865 device. It should be called when you need to
 * obtain the current RTD measurement. The function first clears any
 * existing faults, enables the bias voltage, and performs a one-shot
 * conversion. It then reads the RTD value from the device's registers
 * and stores it in the provided memory location. The function assumes
 * that the device has been properly initialized and configured. It
 * handles both 50Hz and 60Hz filter settings, introducing a delay based
 * on the configuration. The function returns an error code if any
 * operation fails, and it disables the bias voltage before returning.
 *
 * @param device A pointer to an initialized max31865_dev structure representing
 * the device. Must not be null. The function will interact with
 * this device to perform the RTD read operation.
 * @param rtd_reg A pointer to a uint16_t variable where the RTD value will be
 * stored. Must not be null. The function writes the RTD value to
 * this location if successful.
 * @return Returns 0 on success or a negative error code if any step in the
 * process fails.
 ******************************************************************************/
int max31865_read_rtd(struct max31865_dev *, uint16_t *);

#endif // __MAX31865_H__

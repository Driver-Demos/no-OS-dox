/***************************************************************************//**
 *   @file   ltc7841.h
 *   @brief  Header file of LTC7841 Driver.
 *   @author Marvin Cabuenas (marvinneil.cabuenas@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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
#ifndef __LTC7841_H__
#define __LTC7841_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "no_os_i2c.h"
#include "no_os_irq.h"
#include "no_os_print_log.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_units.h"
#include "no_os_alloc.h"

/** X is set based on the pull configuration of the ADDR pin */
#define TWO_BYTES_LENGTH                       2
#define REPEATED_START_MODE                    1
#define I2C_WRITE_LENGTH_FOR_READ              1
#define I2C_DEFAULT_ARRAY_SIZE                 2
#define STOP_MODE                              0
#define LTC7841_DEFAULT_ADDRESS_0              0x5A
#define LTC7841_DEFAULT_ADDRESS_1              0x5B
#define LTC7841_TOTAL_REGISTERS                0x16
#define LTC7841_BYTE_LENGTH                    1
#define LTC7841_WORD_LENGTH                    2
#define LTC7841_I2C_WR_FRAME_SIZE              3
/* This denotes the max voltage output of LTC7841 in millivolts*/
#define LTC7841_MAX_VOUT                       48000
#define LTC7841_7_BIT_ADDRESS_MASK             0x7F
#define LTC7841_OPERATION_OFF                  0x00
#define LTC7841_OPERATION_TURN_ON              0x80
#define LTC7841_OPERATION_MARGIN_LOW           0x98
#define LTC7841_OPERATION_MARGIN_HIGH          0xA8
/* Slew rate multiplier*/
#define LTC7841_NOMINAL_SLEW_RATE              0b00
#define LTC7841_SLOW_SLEW_RATE                 0b01
#define LTC7841_FAST_SLEW_RATE                 0b11
/* READ TEMPERATURE 1*/
#define LTC7841_INTERNAL_DIE_TEMPERATURE       0b0
#define LTC7841_TSNS_PIN_VOLTAGE               0b1
#define READ_TEMPERATURE_1_BIT                 2
/* MFR MARGIN MAX VALUE*/
#define MFR_MAX_VALUE                          0x1FF
/* MFR VOUT INITIALIZE TO HALF */
#define MFR_VOUT_INIT                          0x0FF
#define WRITE_ONLY_REGISTERS_NUMBER            2
#define ECOMM                                  9

/***************************************************************************//**
 * @brief The `ltc7841_write_only_registers` is a global constant array of type
 * `uint8_t` that holds the register addresses of the LTC7841 device that
 * are write-only. The size of this array is defined by the macro
 * `WRITE_ONLY_REGISTERS_NUMBER`, which is set to 2. This array is used
 * to store the specific register addresses that can only be written to,
 * not read from, in the LTC7841 device.
 *
 * @details This variable is used to identify and manage the write-only
 * registers of the LTC7841 device in the driver implementation.
 ******************************************************************************/
extern const uint8_t ltc7841_write_only_registers[WRITE_ONLY_REGISTERS_NUMBER];

/* LTC7841 registers list */
#define  LTC7841_OPERATION                     1
#define  LTC7841_VOUT_MODE                     0x20
#define  LTC7841_STATUS_WORD                   0x79
#define  LTC7841_READ_VIN                      0x88
#define  LTC7841_READ_IIN                      0x89
#define  LTC7841_READ_VOUT                     0x8B
#define  LTC7841_READ_IOUT                     0x8C
#define  LTC7841_READ_TEMPERATURE_1            0x8D
#define  LTC7841_PMBUS_REVISION                0x98
#define  LTC7841_MFR_IOUT_PEAK                 0xD7
#define  LTC7841_MFR_VOUT_PEAK                 0xDD
#define  LTC7841_MFR_VIN_PEAK                  0xDE
#define  LTC7841_MFR_TEMEPRATURE1_PEAK         0xDF
#define  LTC7841_MFR_IIN_PEAK                  0xE1
#define  LTC7841_MFR_CLEAR_PEAKS               0xE3
#define  LTC7841_MFR_VOUT_MARGIN_HIGH          0xE5
#define  LTC7841_MFR_SPECIAL_ID                0xE7
#define  LTC7841_MFR_VOUT_COMMAND              0xE8
#define  LTC7841_MFR_CONFIG                    0xE9
#define  LTC7841_MFR_VOUT_MARGIN_LOW           0xED
#define  LTC7841_MFR_RAIL_ADDRESS              0xFA
#define  LTC7841_MFR_RESET                     0xFD

/***************************************************************************//**
 * @brief The `ltc7841_status_word_bit_placement` is an enumeration that defines
 * specific bit positions for various status indicators related to the
 * LTC7841 device. Each enumerator represents a distinct status
 * condition, such as communication failure, temperature fault, or power
 * good status, and is associated with a specific bit position within a
 * status word. This enumeration is used to interpret the status word of
 * the LTC7841, allowing for easy identification and handling of
 * different operational states and fault conditions.
 *
 * @param LTC7841_COMMUNICATION_FAILURE Represents a communication failure
 * status with a bit position of 1.
 * @param LTC7841_TEMPERATURE_FAULT Indicates a temperature fault with a bit
 * position of 2.
 * @param LTC7841_OUTPUT_OVERVOLTAGE_FAULT Denotes an output overvoltage fault
 * with a bit position of 5.
 * @param LTC7841_OFF Indicates the device is off with a bit position of 6.
 * @param LTC7841_PGOOD Represents a power good status with a bit position of
 * 11.
 * @param LTC7841_OUTPUT_VOLTAGE_FAULT Indicates an output voltage fault with a
 * bit position of 15.
 ******************************************************************************/
enum ltc7841_status_word_bit_placement {
	LTC7841_COMMUNICATION_FAILURE =    1,
	LTC7841_TEMPERATURE_FAULT =        2,
	LTC7841_OUTPUT_OVERVOLTAGE_FAULT = 5,
	LTC7841_OFF =                      6,
	LTC7841_PGOOD =                    11,
	LTC7841_OUTPUT_VOLTAGE_FAULT =     15
};

/***************************************************************************//**
 * @brief The `ltc7841_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the LTC7841 device.
 * It includes a pointer to an I2C initialization parameter structure
 * (`comm_param`) for configuring the I2C communication interface, and an
 * IRQ initialization parameter structure (`irq_param`) for setting up
 * the interrupt request controller. This structure is essential for
 * initializing the device descriptor and ensuring proper communication
 * and interrupt handling for the LTC7841.
 *
 * @param comm_param Pointer to an I2C initialization parameter structure.
 * @param irq_param Structure containing IRQ controller initialization
 * parameters.
 ******************************************************************************/
struct ltc7841_init_param {
	/* I2C */
	struct no_os_i2c_init_param *comm_param;
	/* IRQ controller */
	struct no_os_irq_init_param irq_param;
};

/***************************************************************************//**
 * @brief The `ltc7841_desc` structure is a device descriptor for the LTC7841,
 * which is a power management IC. It contains pointers to an I2C
 * descriptor and an IRQ controller descriptor, facilitating
 * communication and interrupt handling for the device. This structure is
 * essential for managing the device's operations, such as reading and
 * writing register values, initializing the device, and handling various
 * operational commands.
 *
 * @param comm_desc Pointer to an I2C descriptor for communication.
 * @param irq_desc Pointer to an IRQ controller descriptor for handling
 * interrupts.
 ******************************************************************************/
struct ltc7841_desc {
	/* I2C */
	struct no_os_i2c_desc *comm_desc;
	/* IRQ controller */
	struct no_os_irq_ctrl_desc *irq_desc;
};
/***************************************************************************//**
 * @brief Use this function to read a specific register value from the LTC7841
 * device via I2C communication. It requires a valid device descriptor
 * and a command byte that specifies the register to read. The function
 * will store the read data in the provided buffer. Ensure that the
 * device is properly initialized before calling this function. If the
 * command corresponds to a register with a size of zero, the function
 * will return an error. The function handles I2C communication errors
 * and returns appropriate error codes.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before use.
 * @param cmd A uint8_t value representing the command byte that specifies the
 * register to read. Must correspond to a valid register command.
 * @param data A pointer to a uint8_t buffer where the read data will be stored.
 * Must not be null and should have enough space to store the
 * register data.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if the register size is zero or an I2C communication error
 * occurs.
 ******************************************************************************/
int ltc7841_reg_read(struct ltc7841_desc *desc, uint8_t cmd, uint8_t * data);

/***************************************************************************//**
 * @brief Use this function to write a 16-bit value to a specific register of
 * the LTC7841 device via I2C communication. It is essential to ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function handles both byte and word length registers
 * based on the command provided. It returns an error code if the write
 * operation fails, which can be used for error handling in the
 * application.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. Must not be null.
 * @param cmd The command or register address to which the value will be
 * written. It should be a valid register address for the LTC7841.
 * @param value The 16-bit value to write to the specified register. The
 * function will handle the conversion to the appropriate byte or
 * word length based on the register's requirements.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error during the I2C write operation.
 ******************************************************************************/
int ltc7841_reg_write(struct ltc7841_desc *desc, uint8_t cmd, uint16_t value);

/***************************************************************************//**
 * @brief This function sets up an LTC7841 device descriptor by allocating
 * memory for it and initializing the I2C communication based on the
 * provided parameters. It should be called before any other operations
 * on the LTC7841 device to ensure proper setup. The function also sets
 * the manufacturer's VOUT command to a default value and turns on the
 * device. If any step fails, the function will clean up and return an
 * error code. Ensure that the `init_param` is properly configured before
 * calling this function.
 *
 * @param device A pointer to a pointer of type `struct ltc7841_desc`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ltc7841_init_param` containing
 * initialization parameters for the I2C communication. Must
 * be properly initialized before calling this function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, `device` points to a valid, initialized `ltc7841_desc`
 * structure.
 ******************************************************************************/
int ltc7841_init(struct ltc7841_desc ** device,
		 struct ltc7841_init_param * init_param);

/***************************************************************************//**
 * @brief Use this function to properly release resources allocated for an
 * LTC7841 device descriptor when it is no longer needed. This function
 * should be called to clean up after using the device, ensuring that any
 * associated I2C communication resources are also released. It is
 * important to pass a valid descriptor to avoid undefined behavior. If
 * the descriptor is null, the function will return an error code
 * indicating invalid input.
 *
 * @param desc A pointer to an `ltc7841_desc` structure representing the device
 * descriptor to be removed. Must not be null. If null, the function
 * returns an error code.
 * @return Returns 0 on successful removal and resource deallocation. If the
 * descriptor is null, returns -EINVAL. If an error occurs during I2C
 * resource removal, the function returns the corresponding error code.
 ******************************************************************************/
int ltc7841_remove(struct ltc7841_desc *);
/***************************************************************************//**
 * @brief Use this function to reset the LTC7841 device, which is typically
 * necessary when you want to restore the device to its default state or
 * recover from an error condition. This function should be called when
 * the device is in a stable state and not actively processing critical
 * operations, as it will reset the device's configuration. Ensure that
 * the device descriptor has been properly initialized before calling
 * this function.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. This parameter must not be null, and the structure
 * should be properly set up before calling this function. If the
 * descriptor is invalid, the function will return an error code.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ltc7841_reset(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to update the rail address of an LTC7841 device. It
 * is typically called when there is a need to reconfigure the device's
 * I2C address for communication purposes. The function must be called
 * with a valid device descriptor that has been properly initialized. The
 * address is masked to ensure it fits within the 7-bit I2C address
 * space. The function returns an integer status code indicating success
 * or failure of the operation.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device.
 * This must be a valid, initialized descriptor and must not be
 * null.
 * @param addr A uint8_t value representing the new I2C address for the rail.
 * The address is masked to fit within the 7-bit address range.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a non-zero value indicates an error occurred during
 * the operation.
 ******************************************************************************/
int ltc7841_change_rail_address(struct ltc7841_desc *desc, uint8_t addr);
/***************************************************************************//**
 * @brief Use this function to turn off the LTC7841 device, which is typically
 * part of a power management system. This function should be called when
 * the device needs to be powered down safely. Ensure that the device
 * descriptor has been properly initialized before calling this function.
 * The function communicates with the device over I2C to issue the turn-
 * off command, and it returns an integer status code indicating the
 * success or failure of the operation.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. This parameter must not be null, and the structure
 * should be properly set up with valid I2C communication parameters
 * before calling this function. If the descriptor is invalid or not
 * initialized, the function may return an error code.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int ltc7841_turn_off(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to power on the LTC7841 device. It should be called
 * when the device needs to be activated after initialization. Ensure
 * that the device descriptor is properly initialized before calling this
 * function. This function communicates with the device over I2C to
 * change its operational state to 'on'. It is important to handle the
 * return value to check for any communication errors.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. Must not be null. The caller retains ownership and is
 * responsible for ensuring it is valid.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, indicating a communication error or invalid input.
 ******************************************************************************/
int ltc7841_turn_on(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief This function is used to set the LTC7841 device to operate in a margin
 * low mode, which typically involves adjusting the output voltage to a
 * predefined lower margin level. It should be called when the device
 * needs to be configured to this specific operational state. The
 * function requires a valid device descriptor, which must be properly
 * initialized before calling this function. It returns an integer status
 * code indicating the success or failure of the operation.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device
 * descriptor. This must be a valid, initialized descriptor, and
 * must not be null. The caller retains ownership of this pointer.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ltc7841_operation_margin_low(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief This function configures the LTC7841 device to operate in margin high
 * mode, which is typically used for testing or adjusting the output
 * voltage to a higher level than nominal. It should be called when the
 * device is already initialized and ready for operation. The function
 * communicates with the device over I2C to set the appropriate register
 * value. Ensure that the device descriptor is valid and properly
 * initialized before calling this function.
 *
 * @param desc A pointer to a valid and initialized `ltc7841_desc` structure
 * representing the device. This parameter must not be null, and the
 * caller retains ownership of the memory.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error in communication or invalid
 * parameters.
 ******************************************************************************/
int ltc7841_operation_margin_high(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief This function sets the LTC7841 device to operate with the nominal slew
 * rate, which is a predefined setting for the device's output voltage
 * change rate. It should be called when the device is initialized and
 * ready for configuration changes. The function reads the current
 * configuration, modifies it to set the nominal slew rate, and writes it
 * back to the device. It is important to ensure that the device
 * descriptor is valid and properly initialized before calling this
 * function.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device
 * descriptor. This must be a valid, initialized descriptor, and
 * must not be null. The function will return an error if the
 * descriptor is invalid or if communication with the device fails.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as due to communication errors.
 ******************************************************************************/
int ltc7841_change_to_nominal_slew_rate(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to configure the LTC7841 device to operate with a
 * slow slew rate, which may be desirable for specific applications
 * requiring reduced electromagnetic interference or smoother voltage
 * transitions. This function should be called when the device is already
 * initialized and operational. It reads the current configuration,
 * modifies it to set the slow slew rate, and writes it back to the
 * device. Ensure that the device descriptor is valid and properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. Must not be null. The function will return an error
 * if the descriptor is invalid or if communication with the device
 * fails.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as due to communication errors.
 ******************************************************************************/
int ltc7841_change_to_slow_slew_rate(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to configure the LTC7841 device to operate in fast
 * slew rate mode, which may be necessary for applications requiring
 * rapid voltage changes. This function should be called when the device
 * is already initialized and ready for configuration changes. It reads
 * the current configuration, modifies it to enable fast slew rate, and
 * writes it back to the device. Ensure that the device descriptor is
 * valid and properly initialized before calling this function.
 *
 * @param desc A pointer to a valid and initialized `ltc7841_desc` structure
 * representing the device. Must not be null. The function will
 * return an error if the descriptor is invalid or if communication
 * with the device fails.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as due to communication errors.
 ******************************************************************************/
int ltc7841_change_to_fast_slew_rate(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to configure the LTC7841 device to read the internal
 * die temperature instead of any other temperature source. This function
 * should be called when the user needs to monitor the internal
 * temperature of the device for thermal management or diagnostics.
 * Ensure that the device descriptor is properly initialized before
 * calling this function. The function reads the current configuration,
 * modifies it to select the internal die temperature, and writes it
 * back. It returns an error code if the read or write operation fails.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. Must not be null. The function will return an error
 * if the descriptor is invalid or if communication with the device
 * fails.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ltc7841_change_to_internal_die_temperature(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief This function configures the LTC7841 device to read voltage from the
 * TSNS pin by modifying the MFR_CONFIG register. It should be called
 * when the user needs to switch the temperature sensing source to the
 * TSNS pin. The function requires a valid device descriptor and will
 * return an error code if the read or write operation fails. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param desc A pointer to a valid ltc7841_desc structure representing the
 * device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the descriptor is initialized before
 * use.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ltc7841_change_to_tsns_pin_voltage(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief This function is used to set a new voltage output level for the
 * LTC7841 device by writing to the appropriate register. It should be
 * called when there is a need to adjust the output voltage of the
 * device. The function requires a valid device descriptor and a voltage
 * level that does not exceed the maximum allowable value. If the voltage
 * level is within the permissible range, the function attempts to write
 * the new value to the device and returns the result of this operation.
 * If the voltage level is invalid, the function returns an error code.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param vout_level A 16-bit unsigned integer representing the desired output
 * voltage level. Must be less than or equal to MFR_MAX_VALUE
 * (0x1FF). If the value is invalid, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input voltage level is invalid.
 ******************************************************************************/
int ltc7841_change_vout_command(struct ltc7841_desc *desc, uint16_t vout_level);
/***************************************************************************//**
 * @brief This function sets the MFR VOUT margin low value for the LTC7841
 * device to the specified level. It should be used when you need to
 * adjust the voltage output margin low setting of the device. The
 * function requires a valid device descriptor and a voltage level that
 * does not exceed the maximum allowable value. If the voltage level is
 * within the acceptable range, the function writes the value to the
 * device; otherwise, it returns an error.
 *
 * @param desc A pointer to an ltc7841_desc structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param vout_level A 16-bit unsigned integer representing the desired VOUT
 * margin low level. Must be less than or equal to
 * MFR_MAX_VALUE (0x1FF). If the value exceeds this limit, the
 * function returns an error.
 * @return Returns 0 on success, or a negative error code if the vout_level is
 * invalid or if there is a failure in writing to the device.
 ******************************************************************************/
int ltc7841_change_margin_low_command(struct ltc7841_desc *desc,
				      uint16_t vout_level);
/***************************************************************************//**
 * @brief Use this function to set the MFR VOUT margin high value for an LTC7841
 * device. This function should be called when you need to adjust the
 * voltage output margin high setting of the device. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function will only proceed if the provided voltage level
 * is within the acceptable range, otherwise, it will return an error.
 *
 * @param desc A pointer to an initialized ltc7841_desc structure representing
 * the device. Must not be null.
 * @param vout_level A 16-bit unsigned integer representing the desired voltage
 * level for the MFR VOUT margin high setting. Must be less
 * than or equal to MFR_MAX_VALUE (0x1FF). If the value
 * exceeds this limit, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the input voltage
 * level is invalid or if there is a failure in writing to the device.
 ******************************************************************************/
int ltc7841_change_margin_high_command(struct ltc7841_desc *desc,
				       uint16_t vout_level);
/***************************************************************************//**
 * @brief Use this function to reset the peak data values stored in the LTC7841
 * device, which may include peak voltage, current, and temperature
 * readings. This function is typically called when you need to clear
 * historical peak data, for instance, after logging or before starting a
 * new measurement cycle. Ensure that the device descriptor is properly
 * initialized before calling this function. The function communicates
 * with the device over I2C to perform the operation and returns an error
 * code if the operation fails.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. Must not be null. The caller retains ownership of
 * this structure. If the pointer is invalid or uninitialized, the
 * function will return an error code.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int ltc7841_mfr_clear_peaks(struct ltc7841_desc *desc);
/***************************************************************************//**
 * @brief Use this function to clear specific fault and status bits in the
 * LTC7841 device's status word register. This is typically done to reset
 * the status after handling or acknowledging the conditions that set
 * these bits. The function should be called when you need to clear the
 * communication failure, temperature fault, output overvoltage fault,
 * and output voltage fault bits. Ensure that the device descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized `ltc7841_desc` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before calling this function. If the
 * descriptor is invalid, the function will not perform the
 * operation and will return an error code.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int ltc7841_clear_status_word_bits(struct ltc7841_desc *desc);
#endif /* LTC7841_H */

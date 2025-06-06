/***************************************************************************//**
 *   @file   tmc7300.h
 *   @brief  Header file for the TMC7300 driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef _TMC7300_H_
#define _TMC7300_H_

#include <stdint.h>
#include "no_os_uart.h"
#include "no_os_gpio.h"

#define TMC7300_BRIDGE_NUM			2
#define TMC7300_DUTY_MAX_VALUE			511
#define TMC7300_SENDDELAY_MAX_VALUE		15

#define TMC7300_GCONF_REG		0x00
#define TMC7300_IFCNT_REG		0x02
#define TMC7300_SLAVECONF_REG		0x03
#define TMC7300_IOIN_REG		0x03
#define TMC7300_CURRENT_LIMIT_REG	0x10
#define TMC7300_PWM_AB_REG		0x22
#define TMC7300_CHOPCONF_REG		0x6C
#define TMC7300_DRV_STATUS_REG		0x6F
#define TMC7300_PWMCONF_REG		0x70

#define TMC7300_PAR_MODE_MASK	NO_OS_BIT(2)
#define TMC7300_PWM_DIRECT_MASK NO_OS_BIT(0)
#define TMC7300_DRV_ENABLE_MASK NO_OS_BIT(0)
#define TMC7300_SLAVECONF_MASK 	NO_OS_GENMASK(11, 8)
#define TMC7300_IRUN_MASK 	NO_OS_GENMASK(12, 8)
#define TMC7300_PWM_B_MASK	NO_OS_GENMASK(24, 16)
#define TMC7300_PWM_FREQ_MASK	NO_OS_GENMASK(17, 16)
#define TMC7300_FREEWHEEL_MASK	NO_OS_GENMASK(21, 20)
#define TMC7300_BLANK_TIME_MASK	NO_OS_GENMASK(16, 15)

#define TMC7300_LI_MASK(bridge)	(bridge == 0) ? NO_OS_BIT(6) : \
					NO_OS_BIT(7)

#define TMC7300_PWM_MASK(bridge)	(bridge == 0) ? NO_OS_GENMASK(8, 0) : \
					NO_OS_GENMASK(24, 16)

/***************************************************************************//**
 * @brief The `tmc7300_bridge` enumeration defines two constants,
 * `TMC7300_BRIDGE_A` and `TMC7300_BRIDGE_B`, which are used to identify
 * and differentiate between the two motor bridges available in the
 * TMC7300 driver. This enumeration is essential for functions that need
 * to specify which bridge to control or query, allowing for precise
 * management of the motor driver hardware.
 *
 * @param TMC7300_BRIDGE_A Represents the first bridge in the TMC7300 driver.
 * @param TMC7300_BRIDGE_B Represents the second bridge in the TMC7300 driver.
 ******************************************************************************/
enum tmc7300_bridge {
	TMC7300_BRIDGE_A,
	TMC7300_BRIDGE_B,
};

/***************************************************************************//**
 * @brief The `tmc7300_standstill_mode` enumeration defines the possible modes
 * for the TMC7300 motor driver when the motor is in a standstill state.
 * It includes options for freewheeling, low-side braking, and high-side
 * braking, allowing for different methods of handling the motor's
 * behavior when it is not actively driven.
 *
 * @param TMC7300_FREEWHEELING Represents the freewheeling mode with a value of
 * 1.
 * @param TMC7300_BREAK_LS Represents the low-side braking mode.
 * @param TMC7300_BREAK_HS Represents the high-side braking mode.
 ******************************************************************************/
enum tmc7300_standstill_mode {
	TMC7300_FREEWHEELING = 1,
	TMC7300_BREAK_LS,
	TMC7300_BREAK_HS,
};

/***************************************************************************//**
 * @brief The `tmc7300_pwm_freq` enumeration defines a set of constants
 * representing different PWM frequency settings for the TMC7300 driver.
 * Each enumerator corresponds to a specific frequency division factor,
 * allowing the user to select the desired PWM frequency for motor
 * control applications.
 *
 * @param TMC7300_PWM_FREQ_2_1024 Represents a PWM frequency setting of 2/1024.
 * @param TMC7300_PWM_FREQ_2_683 Represents a PWM frequency setting of 2/683.
 * @param TMC7300_PWM_FREQ_2_512 Represents a PWM frequency setting of 2/512.
 * @param TMC7300_PWM_FREQ_2_410 Represents a PWM frequency setting of 2/410.
 ******************************************************************************/
enum tmc7300_pwm_freq {
	TMC7300_PWM_FREQ_2_1024,
	TMC7300_PWM_FREQ_2_683,
	TMC7300_PWM_FREQ_2_512,
	TMC7300_PWM_FREQ_2_410,
};

/***************************************************************************//**
 * @brief The `tmc7300_blank_time` enumeration defines a set of constants
 * representing different blank time settings for the TMC7300 driver.
 * These settings are used to configure the duration of the blanking
 * time, which is a period during which the comparator output is ignored
 * to prevent false triggering due to switching noise. The available
 * options are 16, 24, 32, and 40, allowing for flexibility in tuning the
 * driver's response to noise.
 *
 * @param TMC7300_BLANK_TIME_16 Represents a blank time setting of 16.
 * @param TMC7300_BLANK_TIME_24 Represents a blank time setting of 24.
 * @param TMC7300_BLANK_TIME_32 Represents a blank time setting of 32.
 * @param TMC7300_BLANK_TIME_40 Represents a blank time setting of 40.
 ******************************************************************************/
enum tmc7300_blank_time {
	TMC7300_BLANK_TIME_16,
	TMC7300_BLANK_TIME_24,
	TMC7300_BLANK_TIME_32,
	TMC7300_BLANK_TIME_40,
};

/***************************************************************************//**
 * @brief The `tmc7300_motor_dir` enumeration defines the possible directions
 * for motor rotation in the TMC7300 driver, specifically indicating
 * whether the motor should rotate clockwise (CW) or counterclockwise
 * (CCW). This enumeration is used to control the direction of the motor
 * in applications utilizing the TMC7300 motor driver.
 *
 * @param TMC7300_DIR_CW Represents the clockwise direction for motor rotation.
 * @param TMC7300_DIR_CCW Represents the counterclockwise direction for motor
 * rotation.
 ******************************************************************************/
enum tmc7300_motor_dir {
	TMC7300_DIR_CW,
	TMC7300_DIR_CCW,
};

/***************************************************************************//**
 * @brief The `_tmc7300_drv_status` structure is a bit-field representation of
 * the driver status for the TMC7300 motor driver. It contains various
 * flags that indicate the status of the driver, such as overtemperature
 * warnings, short circuit conditions, and load indicators. The structure
 * uses bit-fields to efficiently pack these status flags into a compact
 * form, with additional reserved bits for future expansion or alignment
 * purposes.
 *
 * @param otpw Overtemperature pre-warning flag.
 * @param ot Overtemperature flag.
 * @param s2ga Short to ground A flag.
 * @param s2gb Short to ground B flag.
 * @param s2vsa Short to supply A flag.
 * @param s2vsb Short to supply B flag.
 * @param lia Load indicator A flag.
 * @param lib Load indicator B flag.
 * @param t120 Temperature threshold 120°C flag.
 * @param t150 Temperature threshold 150°C flag.
 * @param reserved Reserved bits for future use or alignment.
 ******************************************************************************/
struct _tmc7300_drv_status {
	uint8_t otpw : 1;
	uint8_t ot : 1;
	uint8_t s2ga : 1;
	uint8_t s2gb : 1;
	uint8_t s2vsa : 1;
	uint8_t s2vsb : 1;
	uint8_t lia : 1;
	uint8_t lib : 1;
	uint8_t t120: 1;
	uint8_t t150: 1;
	uint32_t reserved: 22;
};

/***************************************************************************//**
 * @brief The `tmc7300_drv_status` is a union that encapsulates the status of
 * the TMC7300 driver. It provides two ways to access the driver status:
 * as a structured set of individual status bits through the `bits`
 * member, or as a single 32-bit integer value through the `val` member.
 * This design allows for flexible access to the driver's status,
 * enabling both detailed bit-level inspection and efficient bulk
 * operations.
 *
 * @param bits A structure containing individual status bits for the TMC7300
 * driver.
 * @param val A 32-bit unsigned integer representing the combined status value.
 ******************************************************************************/
union tmc7300_drv_status {
	struct _tmc7300_drv_status bits;
	uint32_t val;
};

/***************************************************************************//**
 * @brief The `_tmc7300_ioin` structure is a bit-field representation of the
 * input and status register for the TMC7300 motor driver. It includes
 * various control and status bits such as enable, standby, address
 * configuration, diagnostic, UART status, mode input, and comparator
 * results. The structure is designed to efficiently pack these bits into
 * a compact form, allowing for easy manipulation and access to the
 * TMC7300's input and status information.
 *
 * @param en Enable bit for the TMC7300.
 * @param nstdby Standby bit for the TMC7300.
 * @param ad0 Address bit 0 for device configuration.
 * @param ad1 Address bit 1 for device configuration.
 * @param diag Diagnostic bit for the TMC7300.
 * @param uart_on Indicates if UART is enabled.
 * @param uart_input Indicates if UART input is active.
 * @param mode_input Indicates the mode input state.
 * @param a2 Additional input bit 2.
 * @param a1 Additional input bit 1.
 * @param comp_a1a2 Comparator result for inputs A1 and A2.
 * @param comp_b1b2 Comparator result for inputs B1 and B2.
 * @param reserved Reserved bits for future use.
 * @param version Version number of the TMC7300.
 ******************************************************************************/
struct _tmc7300_ioin {
	uint8_t en : 1;
	uint8_t nstdby : 1;
	uint8_t ad0 : 1;
	uint8_t ad1 : 1;
	uint8_t diag : 1;
	uint8_t uart_on : 1;
	uint8_t uart_input : 1;
	uint8_t mode_input : 1;
	uint8_t a2 : 1;
	uint8_t a1 : 1;
	uint8_t comp_a1a2 : 1;
	uint8_t comp_b1b2 : 1;
	uint16_t reserved : 12;
	uint8_t version : 8;
};

/***************************************************************************//**
 * @brief The `tmc7300_ioin` union is designed to encapsulate the input/output
 * state and configuration of the TMC7300 driver. It provides a flexible
 * way to access the individual bit fields through the `bits` structure,
 * which includes flags for enabling, standby, address pins, diagnostic
 * status, UART settings, and mode inputs, among others. Alternatively,
 * the entire 32-bit value can be accessed directly via the `val` member,
 * allowing for efficient reading and writing of the complete register
 * state.
 *
 * @param bits A structure containing individual bit fields representing various
 * input/output states and configurations.
 * @param val A 32-bit unsigned integer representing the combined value of all
 * bit fields in the structure.
 ******************************************************************************/
union tmc7300_ioin {
	struct _tmc7300_ioin bits;
	uint32_t val;
};

/***************************************************************************//**
 * @brief The `tmc7300_bridge_priv` structure is used to encapsulate the private
 * data for controlling a motor bridge in the TMC7300 driver. It contains
 * fields for setting the PWM duty cycle and the motor direction, which
 * are essential for managing the motor's speed and rotational direction.
 * This structure is part of the broader TMC7300 driver framework, which
 * facilitates motor control through various configurations and settings.
 *
 * @param pwm_duty Represents the PWM duty cycle for the motor control.
 * @param motor_dir Specifies the direction of the motor, either clockwise or
 * counterclockwise.
 ******************************************************************************/
struct tmc7300_bridge_priv {
	uint8_t pwm_duty;
	enum tmc7300_motor_dir motor_dir;
};

/***************************************************************************//**
 * @brief The `tmc7300_desc` structure is designed to encapsulate the runtime
 * state for the TMC7300 motor driver. It includes configuration details
 * such as the device address, communication descriptors for UART and
 * GPIO, and operational parameters like motor drive state and current
 * limit. Additionally, it holds PWM settings for the output bridges,
 * allowing for precise control of motor operations. This structure is
 * essential for managing the TMC7300's functionality in embedded
 * systems.
 *
 * @param addr Device address configured by the AD0 and AD1 pins.
 * @param comm_desc Initialized UART descriptor, requiring asynchronous_rx to be
 * set to 1.
 * @param en_gpio Initialized GPIO descriptor for the EN (bridge driver enable)
 * signal.
 * @param vio_gpio Initialized GPIO descriptor for the VIO signal.
 * @param motor_drive Indicates whether the motor is actively driven or in
 * freewheeling/passive braking mode.
 * @param irun Current limit setting for the motor.
 * @param bridge_priv Array of PWM settings for the output bridges, with a size
 * defined by TMC7300_BRIDGE_NUM.
 ******************************************************************************/
struct tmc7300_desc {
	/** Device address configured by the AD0 and AD1 pins */
	uint32_t addr;
	/** Initialized UART descriptor. Has to have asynchronous_rx = 1 */
	struct no_os_uart_desc *comm_desc;
	/** Initialized GPIO descriptor for the EN (bridge driver enable) signal */
	struct no_os_gpio_desc *en_gpio;
	/** Initialized GPIO descriptor for the VIO signal */
	struct no_os_gpio_desc *vio_gpio;

	/**
	 * Whether the motor is actively driven or
	 * freewheeling/passive braking is applied
	 */
	bool motor_drive;
	/* Current limit setting */
	uint8_t irun;
	/** PWM settings for the output bridges */
	struct tmc7300_bridge_priv bridge_priv[TMC7300_BRIDGE_NUM];
};

/***************************************************************************//**
 * @brief The `tmc7300_init_param` structure is used to initialize the TMC7300
 * motor driver. It contains configuration parameters such as the device
 * address, communication descriptor, and GPIO descriptors for enabling
 * the bridge driver and VIO signal. Additionally, it includes a flag to
 * enable parallel mode, allowing for single motor drive using both
 * bridges. This structure is essential for setting up the TMC7300 driver
 * with the necessary hardware and communication settings.
 *
 * @param addr Device address configured by the AD0 and AD1 pins.
 * @param comm_desc Initialized UART descriptor, which must have asynchronous_rx
 * set to 1.
 * @param en_gpio Initialized GPIO descriptor for the EN (bridge driver enable)
 * signal.
 * @param vio_gpio Initialized GPIO descriptor for the VIO signal.
 * @param parallel_mode Boolean flag to enable single motor drive using both
 * bridges.
 ******************************************************************************/
struct tmc7300_init_param {
	/** Device address configured by the AD0 and AD1 pins */
	uint32_t addr;
	/** Initialized UART descriptor. Has to have asynchronous_rx = 1 */
	struct no_os_uart_desc *comm_desc;
	/** Initialized GPIO descriptor for the EN (bridge driver enable) signal */
	struct no_os_gpio_desc *en_gpio;
	/** Initialized GPIO descriptor for the VIO signal */
	struct no_os_gpio_desc *vio_gpio;
	/** Enable single motor drive using both bridges */
	bool parallel_mode;
};

/***************************************************************************//**
 * @brief This function is used to read the value of a register from a TMC7300
 * device using a UART communication interface. It requires a valid
 * descriptor for the TMC7300 device, the address of the register to be
 * read, and a pointer to store the read value. The function must be
 * called with a properly initialized descriptor, and the pointer for
 * storing the value must not be null. It handles communication errors
 * and checks for data integrity using a CRC. If the descriptor is null,
 * the function returns an error indicating the device is not available.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. Must not be null. The descriptor should be properly
 * initialized before calling this function.
 * @param addr The address of the register to be read. It is a 32-bit unsigned
 * integer representing the register address within the TMC7300
 * device.
 * @param val A pointer to a 32-bit unsigned integer where the read register
 * value will be stored. Must not be null. The function writes the
 * read value to this location if successful.
 * @return Returns 0 on success, or a negative error code on failure. The read
 * value is stored in the location pointed to by 'val' if successful.
 ******************************************************************************/
int tmc7300_reg_read(struct tmc7300_desc *, uint32_t, uint32_t *);

/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to a specific register
 * of the TMC7300 motor driver. It requires a valid descriptor for the
 * TMC7300 device, which includes communication and configuration
 * details. The function should be called when a register value needs to
 * be updated, and it handles the communication over UART to perform the
 * write operation. It is important to ensure that the descriptor is
 * properly initialized and not null before calling this function, as a
 * null descriptor will result in an error. The function also manages the
 * UART response flushing internally.
 *
 * @param desc A pointer to a tmc7300_desc structure that must be initialized
 * and not null. It contains the communication descriptor and device
 * address necessary for the operation. If null, the function
 * returns an error.
 * @param addr A 32-bit unsigned integer representing the address of the
 * register to be written. The address should be a valid register
 * address for the TMC7300 device.
 * @param val A 32-bit unsigned integer representing the value to be written to
 * the specified register. The value should be within the valid range
 * for the register being accessed.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when the descriptor is null or a communication error
 * occurs.
 ******************************************************************************/
int tmc7300_reg_write(struct tmc7300_desc *, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief This function is used to modify a specific field within a register of
 * the TMC7300 device by applying a bitmask. It first reads the current
 * value of the register, applies the mask to clear the field, and then
 * sets the field to the new value. This function should be called when
 * you need to change specific settings in a register without affecting
 * other fields. It requires a valid TMC7300 descriptor and assumes that
 * the device is properly initialized. The function returns an error code
 * if the read or write operation fails.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. Must not be null and should be properly initialized
 * before calling this function.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the TMC7300 device.
 * @param mask A bitmask indicating the field to be updated within the register.
 * The mask should be non-zero and correctly aligned with the field
 * to be modified.
 * @param val The new value to set in the specified field. The value should be
 * within the range allowed by the mask.
 * @return Returns 0 on success, or a negative error code if the register read
 * or write operation fails.
 ******************************************************************************/
int tmc7300_reg_update(struct tmc7300_desc *, uint32_t, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief Use this function to determine if the output of a specified bridge on
 * the TMC7300 driver is current limited. It should be called when you
 * need to monitor the load status of a bridge, which can be useful for
 * diagnostics or performance monitoring. Ensure that the `desc`
 * parameter is properly initialized before calling this function. The
 * function will populate the provided `val` pointer with the load
 * indicator status, which is specific to the selected bridge. Handle any
 * non-zero return values as errors, which indicate that the read
 * operation was unsuccessful.
 *
 * @param desc A pointer to a `tmc7300_desc` structure that must be initialized
 * before use. It represents the TMC7300 device context.
 * @param bridge An enum value of type `tmc7300_bridge` specifying which bridge
 * (A or B) to query for the load indicator status.
 * @param val A pointer to a `uint32_t` where the load indicator status will be
 * stored. Must not be null, and the caller is responsible for
 * managing the memory.
 * @return Returns 0 on success, with the load indicator status stored in `val`.
 * Returns a non-zero error code if the operation fails.
 ******************************************************************************/
int tmc7300_get_load_indicator(struct tmc7300_desc *, enum tmc7300_bridge,
			       uint32_t *);

/***************************************************************************//**
 * @brief Use this function to enable or disable the TMC7300 bridge driver. It
 * is essential to ensure that the `desc` parameter is a valid,
 * initialized descriptor before calling this function. The function will
 * attempt to set the GPIO value associated with the enable signal and
 * update the driver enable register. If the descriptor is null, the
 * function will return an error. This function is typically used to
 * control the operational state of the motor driver, allowing for
 * dynamic enabling or disabling as needed.
 *
 * @param desc A pointer to a `tmc7300_desc` structure that must be initialized
 * and not null. This structure contains the necessary configuration
 * and state information for the TMC7300 device.
 * @param enable A boolean value where `true` enables the bridge driver and
 * `false` disables it.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when the descriptor is null or a GPIO operation fails.
 ******************************************************************************/
int tmc7300_drv_enable(struct tmc7300_desc *, bool);

/***************************************************************************//**
 * @brief Use this function to configure the PWM duty cycle for a specific
 * bridge on the TMC7300 motor driver. The function sets the duty cycle
 * and determines the motor direction based on the sign of the duty
 * parameter. It should be called with a valid descriptor and bridge
 * identifier. The duty cycle must not exceed the maximum allowed value,
 * and negative values indicate a counter-clockwise direction. This
 * function is typically used after initializing the TMC7300 device and
 * when you need to control motor speed and direction.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. Must not be null, and should be properly initialized
 * before calling this function.
 * @param bridge An enum value of type tmc7300_bridge indicating which bridge (A
 * or B) to configure. Must be a valid bridge identifier.
 * @param duty An int32_t value representing the desired PWM duty cycle. The
 * value must be within the range of -TMC7300_DUTY_MAX_VALUE to
 * TMC7300_DUTY_MAX_VALUE. Values greater than
 * TMC7300_DUTY_MAX_VALUE will result in an error.
 * @return Returns 0 on success, or a negative error code if the duty value is
 * invalid or if there is a failure in updating the register.
 ******************************************************************************/
int tmc7300_set_pwm_duty(struct tmc7300_desc *, enum tmc7300_bridge, int32_t);

/***************************************************************************//**
 * @brief This function configures the delay for the UART access response of the
 * TMC7300 device. It should be used when there is a need to adjust the
 * communication timing with the device. The function must be called with
 * a valid descriptor and a delay value that is even and does not exceed
 * the maximum allowed value. If the delay value is invalid, the function
 * returns an error code.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the device.
 * Must not be null, and the structure should be properly
 * initialized before calling this function.
 * @param delay An 8-bit unsigned integer representing the desired delay. Must
 * be an even number and not exceed TMC7300_SENDDELAY_MAX_VALUE. If
 * the value is invalid, the function returns an error.
 * @return Returns 0 on success or a negative error code if the delay value is
 * invalid.
 ******************************************************************************/
int tmc7300_set_send_delay(struct tmc7300_desc *, uint8_t);

/***************************************************************************//**
 * @brief Use this function to configure the current limit for the TMC7300 motor
 * driver. It should be called when you need to adjust the current limit
 * to a specific value, which is crucial for controlling the motor's
 * power consumption and protecting the hardware. The function requires a
 * valid descriptor and a current limit value within the specified range.
 * It returns an error if the value is out of range or if the register
 * write operation fails. Ensure the descriptor is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. Must not be null and should be properly initialized
 * before use. The caller retains ownership.
 * @param val A 32-bit unsigned integer representing the desired current limit
 * setting. Valid values are between 0 and 31 inclusive. If the value
 * is greater than 31, the function returns an error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid input).
 ******************************************************************************/
int tmc7300_set_current_limit(struct tmc7300_desc *, uint32_t);

/***************************************************************************//**
 * @brief This function retrieves the valid write access counter from the
 * TMC7300 device, which is useful for monitoring communication
 * integrity. It should be called when you need to verify the number of
 * successful write operations to the device. Ensure that the `desc`
 * parameter is properly initialized before calling this function. The
 * function writes the counter value to the location pointed to by
 * `ifcnt`. If the read operation fails, the function returns an error
 * code.
 *
 * @param desc A pointer to a `tmc7300_desc` structure that must be initialized
 * and represents the TMC7300 device. It must not be null.
 * @param ifcnt A pointer to a `uint8_t` where the function will store the read
 * counter value. It must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int tmc7300_get_ifcnt(struct tmc7300_desc *, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to obtain the current driver status from a TMC7300
 * device. It should be called when you need to check the status of the
 * driver, such as fault conditions or operational states. Ensure that
 * the `tmc7300_desc` structure is properly initialized before calling
 * this function. The function will populate the provided
 * `tmc7300_drv_status` union with the status information. If the
 * function fails, it will return a non-zero error code.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the TMC7300
 * device. This must be initialized and must not be null. The
 * function will not modify this structure.
 * @param status A pointer to a `tmc7300_drv_status` union where the driver
 * status will be stored. This must not be null. The function will
 * write the status information to this location.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int tmc7300_get_drv_status(struct tmc7300_desc *, union tmc7300_drv_status *);

/***************************************************************************//**
 * @brief This function retrieves the current value of the IOIN register from a
 * TMC7300 device and stores it in the provided `tmc7300_ioin` union. It
 * is typically used to obtain the current input status and configuration
 * of the device. The function must be called with a valid and
 * initialized `tmc7300_desc` structure, which represents the device
 * context. The `ioin` parameter must point to a valid `tmc7300_ioin`
 * union where the register value will be stored. The function returns an
 * error code if the read operation fails.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the device
 * context. This must be a valid and initialized descriptor for the
 * TMC7300 device. The caller retains ownership.
 * @param ioin A pointer to a `tmc7300_ioin` union where the IOIN register value
 * will be stored. Must not be null. The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the register read
 * operation fails.
 ******************************************************************************/
int tmc7300_get_ioin(struct tmc7300_desc *, union tmc7300_ioin *);

/***************************************************************************//**
 * @brief Use this function to configure the standstill mode of the TMC7300
 * motor driver, which determines the behavior of the motor when it is
 * not actively driven. This function should be called when you need to
 * change the motor's behavior to either freewheeling or braking modes.
 * Ensure that the `desc` parameter is a valid, initialized descriptor
 * for the TMC7300 device. The function will return an error if the
 * descriptor is null or if there is a failure in writing to the device
 * registers.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the TMC7300
 * device. Must not be null. The caller retains ownership.
 * @param mode An enumeration value of type `tmc7300_standstill_mode` indicating
 * the desired standstill mode. Valid values are
 * `TMC7300_FREEWHEELING`, `TMC7300_BREAK_LS`, and
 * `TMC7300_BREAK_HS`.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or if there is a failure in updating the device registers.
 ******************************************************************************/
int tmc7300_set_standstill_mode(struct tmc7300_desc *,
				enum tmc7300_standstill_mode);

/***************************************************************************//**
 * @brief Use this function to configure the PWM frequency of a TMC7300 motor
 * driver. It is essential to call this function after initializing the
 * TMC7300 device to ensure the PWM operates at the desired frequency.
 * The function updates the PWM frequency setting in the device's
 * configuration register. Ensure that the `desc` parameter is a valid
 * and initialized descriptor for the TMC7300 device. The function
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the TMC7300
 * device. This must be a valid, initialized descriptor, and must
 * not be null. The caller retains ownership.
 * @param freq An enumerated value of type `tmc7300_pwm_freq` representing the
 * desired PWM frequency. Valid values are defined in the
 * `tmc7300_pwm_freq` enum.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during the operation.
 ******************************************************************************/
int tmc7300_set_pwm_freq(struct tmc7300_desc *, enum tmc7300_pwm_freq);

/***************************************************************************//**
 * @brief Use this function to obtain the current PWM frequency setting of the
 * TMC7300 device. It is essential to call this function after the device
 * has been properly initialized and configured. The function reads the
 * PWM configuration register and extracts the frequency setting, which
 * is then returned through the provided pointer. This function is useful
 * for verifying the current PWM frequency setting or for debugging
 * purposes.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the TMC7300
 * device. This must be a valid, initialized descriptor and must not
 * be null. The function will not modify the contents of this
 * structure.
 * @param freq A pointer to an `enum tmc7300_pwm_freq` where the current PWM
 * frequency setting will be stored. This pointer must not be null.
 * If the function fails, the value pointed to by `freq` is not
 * modified.
 * @return Returns 0 on success, indicating that the PWM frequency was
 * successfully retrieved and stored in the location pointed to by
 * `freq`. If an error occurs, a non-zero error code is returned, and
 * the value pointed to by `freq` is not modified.
 ******************************************************************************/
int tmc7300_get_pwm_freq(struct tmc7300_desc *, enum tmc7300_pwm_freq *);

/***************************************************************************//**
 * @brief Use this function to retrieve the current blank time setting of the
 * TMC7300 device. This function should be called when you need to know
 * the blank time configuration, which is a parameter affecting the
 * device's operation. Ensure that the `desc` parameter is properly
 * initialized before calling this function. The function will store the
 * retrieved blank time in the provided `time` parameter. It returns an
 * error code if the read operation fails.
 *
 * @param desc A pointer to a `tmc7300_desc` structure representing the TMC7300
 * device. This must be initialized and not null. The function will
 * not modify this structure.
 * @param time A pointer to an `enum tmc7300_blank_time` where the function will
 * store the retrieved blank time value. This must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int tmc7300_get_blank_time(struct tmc7300_desc *, enum tmc7300_blank_time *);

/***************************************************************************//**
 * @brief This function configures the blank time for the comparator in the
 * TMC7300 driver, which is a critical parameter for motor control
 * applications. It should be called when you need to adjust the blanking
 * time to match specific motor characteristics or application
 * requirements. Ensure that the TMC7300 descriptor is properly
 * initialized before calling this function. The function updates the
 * relevant register with the specified blank time setting.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. This must be initialized and not null, as it provides the
 * context for the operation.
 * @param time An enum value of type tmc7300_blank_time, specifying the desired
 * blank time setting. Valid values are TMC7300_BLANK_TIME_16,
 * TMC7300_BLANK_TIME_24, TMC7300_BLANK_TIME_32, and
 * TMC7300_BLANK_TIME_40.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the register update process.
 ******************************************************************************/
int tmc7300_set_blank_time(struct tmc7300_desc *, enum tmc7300_blank_time);

/***************************************************************************//**
 * @brief This function configures the PWM duty cycle and the motor direction
 * for a specified bridge on the TMC7300 driver. It should be used when
 * you need to control the speed and direction of a motor connected to
 * the TMC7300. The function requires a valid descriptor for the TMC7300
 * device, and it must be called after the device has been properly
 * initialized. The duty cycle is specified as a value between 0 and 255,
 * representing 0% to 100% of the maximum duty cycle. The direction can
 * be set to clockwise or counterclockwise. The function updates the
 * internal state of the descriptor to reflect the new settings. If the
 * operation fails, a non-zero error code is returned.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device. Must not be null and should be initialized before calling
 * this function. The caller retains ownership.
 * @param bridge An enum value of type tmc7300_bridge indicating which bridge (A
 * or B) to configure. Must be a valid bridge identifier.
 * @param duty An 8-bit unsigned integer representing the PWM duty cycle,
 * ranging from 0 to 255. Values outside this range may result in
 * undefined behavior.
 * @param dir An enum value of type tmc7300_motor_dir specifying the motor
 * direction, either TMC7300_DIR_CW (clockwise) or TMC7300_DIR_CCW
 * (counterclockwise). Must be a valid direction identifier.
 * @return Returns 0 on success or a non-zero error code if the operation fails.
 ******************************************************************************/
int tmc7300_set_pwm_duty_dir(struct tmc7300_desc *, enum tmc7300_bridge,
			     uint8_t, enum tmc7300_motor_dir);

/***************************************************************************//**
 * @brief This function sets up a TMC7300 device descriptor based on the
 * provided initialization parameters. It must be called before any other
 * operations on the TMC7300 device. The function configures GPIOs and
 * updates device registers to prepare the device for operation. It
 * requires valid initialization parameters, including a communication
 * descriptor and GPIO descriptors. If initialization fails, it returns
 * an error code and ensures that any allocated resources are freed.
 *
 * @param desc A pointer to a pointer where the initialized TMC7300 descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated descriptor upon successful initialization.
 * @param param A pointer to a tmc7300_init_param structure containing
 * initialization parameters. Must not be null and must include a
 * valid communication descriptor and GPIO descriptors. If invalid,
 * the function returns -EINVAL.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL for invalid parameters or -ENOMEM if memory allocation
 * fails.
 ******************************************************************************/
int tmc7300_init(struct tmc7300_desc **, struct tmc7300_init_param *);

/***************************************************************************//**
 * @brief Use this function to release all resources allocated for a TMC7300
 * descriptor when it is no longer needed. This function should be called
 * to clean up after a TMC7300 descriptor has been initialized and used,
 * ensuring that any associated GPIO resources are properly freed. It is
 * important to pass a valid descriptor to avoid undefined behavior. If
 * the descriptor is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param desc A pointer to a tmc7300_desc structure representing the TMC7300
 * device to be removed. Must not be null. If null, the function
 * returns -ENODEV.
 * @return Returns 0 on success, or -ENODEV if the provided descriptor is null.
 ******************************************************************************/
int tmc7300_remove(struct tmc7300_desc *);

#endif
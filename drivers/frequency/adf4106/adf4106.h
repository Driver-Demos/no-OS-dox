/**************************************************************************//**
*   @file   adf4106.h
*   @brief  Header file of ADF4106 driver. This driver supporting the following
*           devices : ADF4001, ADF4002, ADF4106
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*****************************************************************************/
#ifndef __ADF4106_H__
#define __ADF4106_H__

/*****************************************************************************/
/****************************** Include Files ********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/*****************************************************************************/
/*  Device specific MACROs                                                   */
/*****************************************************************************/
/* GPIOs */
#define ADF4106_LE_OUT                      no_os_gpio_direction_output(dev->gpio_le,  \
			                    NO_OS_GPIO_HIGH)
#define ADF4106_LE_LOW                      no_os_gpio_set_value(dev->gpio_le,         \
			                    NO_OS_GPIO_LOW)
#define ADF4106_LE_HIGH                     no_os_gpio_set_value(dev->gpio_le,         \
			                    NO_OS_GPIO_HIGH)

#define ADF4106_CE_OUT                      no_os_gpio_direction_output(dev->gpio_ce,  \
			                    NO_OS_GPIO_HIGH)
#define ADF4106_CE_LOW                      no_os_gpio_set_value(dev->gpio_ce,         \
			                    NO_OS_GPIO_LOW)
#define ADF4106_CE_HIGH                     no_os_gpio_set_value(dev->gpio_ce,         \
			                    NO_OS_GPIO_HIGH)

#define ADF4106_LE2_OUT                     no_os_gpio_direction_output(dev->gpio_le2, \
			                    NO_OS_GPIO_HIGH)
#define ADF4106_LE2_LOW                     no_os_gpio_set_value(dev->gpio_le2,        \
			                    NO_OS_GPIO_LOW)
#define ADF4106_LE2_HIGH                    no_os_gpio_set_value(dev->gpio_le2,        \
			                    NO_OS_GPIO_HIGH)

#define ADF4106_CE2_OUT                     no_os_gpio_direction_output(dev->gpio_ce2, \
			                    NO_OS_GPIO_HIGH)
#define ADF4106_CE2_LOW                     no_os_gpio_set_value(dev->gpio_ce2,        \
			                    NO_OS_GPIO_LOW)
#define ADF4106_CE2_HIGH                    no_os_gpio_set_value(dev->gpio_ce2,        \
			                    NO_OS_GPIO_HIGH)

/* Control Bits */
#define ADF4106_CTRL_MASK                   0x3

#define ADF4106_CTRL_R_COUNTER              0          /* Reference Counter */
#define ADF4106_CTRL_N_COUNTER              1          /* N Counter */
#define ADF4106_CTRL_FUNCTION_LATCH         2          /* Function Latch*/
#define ADF4106_CTRL_INIT_LATCH             3          /* Initialization Latch*/

/* Reference Counter Latch */

#define ADF4106_R_COUNTER_OFFSET            2
#define ADF4106_R_COUNTER_MASK              0x3FFFul
#define ADF4106_R_COUNTER(x)                ((x) & ADF4106_R_COUNTER_MASK) \
                                            << ADF4106_R_COUNTER_OFFSET
/* Anti-backlash Pulse Width options */
#define ADF4106_R_ABP_OFFSET                16
#define ADF4106_R_ABP_MASK                  0x3
#define ADF4106_R_ABP(x)                    ((x) & ADF4106_R_ABP_MASK) \
                                            << ADF4106_R_ABP_OFFSET
#define ADF4106_R_ABP_2_9NS                 0
#define ADF4106_R_ABP_1_3NS                 1
#define ADF4106_R_ABP_6_0NS                 2

/* Test Mode Bits */
#define ADF4106_R_TMB_OFFSET                18
#define ADF4106_R_TMB_MASK                  0x1
#define ADF4106_R_TMB(x)                    ((x) & ADF4106_R_TMB_MASK) \
                                            << ADF4106_R_TMB_OFFSET
#define ADF4106_R_TMB_NORMAL                0
/* Lock Detect Precision */
#define ADF4106_R_LDP_OFFSET                20
#define ADF4106_R_LDP_MASK                  0x1
#define ADF4106_R_LDP(x)                    ((x) & ADF4106_R_LDP_MASK) \
                                            <<  ADF4106_R_LDP_OFFSET
#define ADF4106_R_LDP_3                     0
#define ADF4106_R_LDP_5                     1

/* N Counter Latch */
#define ADF4106_N_COUNTER_A_OFFSET          2
#define ADF4106_N_COUNTER_A_MASK            0x3F
#define ADF4106_N_COUNTER_A(x)              ((x) & ADF4106_N_COUNTER_A_MASK) \
                                            << ADF4106_N_COUNTER_A_OFFSET
#define ADF4106_N_COUNTER_B_OFFSET          8
#define ADF4106_N_COUNTER_B_MASK            0x1FFF
#define ADF4106_N_COUNTER_B(x)              ((x) & ADF4106_N_COUNTER_B_MASK) \
                                            << ADF4106_N_COUNTER_B_OFFSET
/* Charge Pump Gain Settings */
#define ADF4106_N_CP_OFFSET                 21
#define ADF4106_N_CP_MASK                   0x1
#define ADF4106_N_CP(x)                     ((x) & ADF4106_N_CP_MASK) \
                                            << ADF4106_N_CP_OFFSET
#define ADF4106_N_CP_GAIN_1                 0
#define ADF4106_N_CP_GAIN_2                 1

/********* Function and Initialization register offsets and masks ************/
/* Counter Reset Bit */
#define ADF4106_CR_OFFSET                   2
#define ADF4106_CR_MASK                     0x1ul
#define ADF4106_CR(x)                       ((x) & ADF4106_CR_MASK) \
                                            << ADF4106_CR_OFFSET
/* Power Down Bit 1 */
#define ADF4106_PD1_OFFSET                  3
#define ADF4106_PD1_MASK                    0x1
#define ADF4106_PD1(x)                      ((x) & ADF4106_PD1_MASK) \
                                            << ADF4106_PD1_OFFSET
/* Muxout Control */
#define ADF4106_MUXOUT_OFFSET               4
#define ADF4106_MUXOUT_MASK                 0x7
#define ADF4106_MUXOUT(x)                   ((x) & ADF4106_MUXOUT_MASK) \
                                            << ADF4106_MUXOUT_OFFSET
/* Phase Detector Polarity */
#define ADF4106_PDPOL_OFFSET                7
#define ADF4106_PDPOL_MASK                  0x1
#define ADF4106_PDPOL(x)                    ((x) & ADF4106_PDPOL_MASK) \
                                            << ADF4106_PDPOL_OFFSET
/* Charge Pump Output*/
#define ADF4106_CP_OFFSET                   8
#define ADF4106_CP_MASK                     0x1
#define ADF4106_CP(x)                       ((x) & ADF4106_CP_MASK) \
                                            << ADF4106_CP_OFFSET
/* Fast-lock Mode */
#define ADF4106_FASTLOCK_OFFSET             9
#define ADF4106_FASTLOCK_MASK               0x3
#define ADF4106_FASTLOCK(x)                 ((x) & ADF4106_FASTLOCK_MASK) \
                                            << ADF4106_FASTLOCK_OFFSET
/* Timer Counter Control */
#define ADF4106_TCC_OFFSET                  11
#define ADF4106_TCC_MASK                    0xF
#define ADF4106_TCC(x)                      ((x) & ADF4106_TCC_MASK) \
                                            << ADF4106_TCC_OFFSET
/* Current Setting Position */
#define ADF4106_CS1_OFFSET                  15
#define ADF4106_CS1_MASK                    0x7
#define ADF4106_CS1(x)                      ((x) << ADF4106_CS1_OFFSET)
#define ADF4106_CS2_OFFSET                  18
#define ADF4106_CS2_MASK                    0x7
#define ADF4106_CS2(x)                      ((x) << ADF4106_CS2_OFFSET)

/* Synchronous or asynchronous power down*/
#define ADF4106_PD2_OFFSET                  21
#define ADF4106_PD2_MASK                    0x1
#define ADF4106_PD2(x)                      ((x) & ADF4106_PD2_MASK) \
                                            << ADF4106_PD2_OFFSET
/* Prescaler value */
#define ADF4106_PS_OFFSET                   22
#define ADF4106_PS_MASK                     0x3
#define ADF4106_PS(x)                       ((x) & ADF4106_PS_MASK) \
                                            << ADF4106_PS_OFFSET

/* Counter Reset Bit Definition */
#define ADF4106_CR_NORMAL                 0
#define ADF4106_CR_RESET                  1
/* Power Down Bit 1 Definition */
#define ADF4106_PD1_NORMAL                0
#define ADF4106_PD1_POWER_DOWN            1
/* Muxout Control Definition */
#define ADF4106_MUXOUT_3STATE             0
#define ADF4106_MUXOUT_DLOCK_DETECT       1
#define ADF4106_MUXOUT_NDIV_OUTPUT        2
#define ADF4106_MUXOUT_AVDD               3
#define ADF4106_MUXOUT_RDIV_OUTPUT        4
#define ADF4106_MUXOUT_NCH_OPENDRAIN      5
#define ADF4106_MUXOUT_SERIAL_OUTPUT      6
#define ADF4106_MUXOUT_DGND               7
/* Phase Detector Polarity Definition */
#define ADF4106_PDPOL_NEGATIVE            0
#define ADF4106_PDPOL_POSITIVE            1
/* Charge Pump Output Definition */
#define ADF4106_CP_NORMAL                 0
#define ADF4106_CP_THREE_STATE            1
/* Fast-lock Mode Definition */
#define ADF4106_FASTLOCK_DISABLE          0
#define ADF4106_FASTLOCK_MODE1            1
#define ADF4106_FASTLOCK_MODE2            3
/* Timer Counter Control Definition */
#define ADF4106_TCC_3                     0
#define ADF4106_TCC_7                     1
#define ADF4106_TCC_11                    2
#define ADF4106_TCC_15                    3
#define ADF4106_TCC_19                    4
#define ADF4106_TCC_23                    5
#define ADF4106_TCC_27                    6
#define ADF4106_TCC_31                    7
#define ADF4106_TCC_35                    8
#define ADF4106_TCC_39                    9
#define ADF4106_TCC_43                    10
#define ADF4106_TCC_47                    11
#define ADF4106_TCC_51                    12
#define ADF4106_TCC_55                    13
#define ADF4106_TCC_59                    14
#define ADF4106_TCC_63                    15
/* Current Settings Definitions */
#define ADF4106_CS_0_62                   0
#define ADF4106_CS_1_25                   1
#define ADF4106_CS_1_87                   2
#define ADF4106_CS_2_5                    3
#define ADF4106_CS_3_12                   4
#define ADF4106_CS_3_75                   5
#define ADF4106_CS_4_37                   6
#define ADF4106_CS_5_0                    7
/* Synchronous or asynchronous power down Definition */
#define ADF4106_ASYNC_PWD                 0
#define ADF4106_SYNC_PWD                  1
/* Prescaler value Definition */
#define ADF4106_PS_8_9                    0
#define ADF4106_PS_16_17                  1
#define ADF4106_PS_32_33                  2
#define ADF4106_PS_64_65                  3

/* Default prescaler for ADF4001 and ADF4002 */
#define ADF4106_PRESCALE(x)                 (8 << (x))

/*****************************************************************************/
/************************** Types Declarations *******************************/
/*****************************************************************************/

/**
*   @struct adf4106_settings_t
*   @brief store the value of all the latch and the input
*                               reference frequency
*/

/***************************************************************************//**
 * @brief The `adf4106_settings_t` structure is designed to store configuration
 * settings for the ADF4106 device, which is a frequency synthesizer. It
 * includes fields for setting reference input frequency, phase frequency
 * detector (PFD) maximum frequency, and various counters and control
 * bits that determine the operation of the device, such as anti-backlash
 * pulse width, lock detect precision, and charge pump settings. This
 * structure allows for detailed control over the device's operation,
 * including power down modes, fast lock modes, and prescaler values,
 * making it suitable for precise frequency synthesis applications.
 *
 * @param ref_in Reference Input Frequency.
 * @param pfd_max PFD max frequency.
 * @param ref_counter The initial value of the 14-bit Reference Counter
 * register.
 * @param anti_backlash_width The width of the anti-backlash pulse to minimize
 * phase noise and reference spurs.
 * @param test_mode_bits Should be set to zero for Normal operation.
 * @param lock_detect_precision Determines the number of consecutive cycles of
 * phase delay before lock detect is set.
 * @param a_n_counter A 6-bit counter supported at ADF4106.
 * @param b_n_counter A 13-bit counter.
 * @param cp_gain Determines which charge pump current settings are used.
 * @param counter_reset Resets the R and N counters.
 * @param power_down1 Activates power down mode.
 * @param muxout_control The type of the MUXOUT output.
 * @param phase_detector_pol The polarity of the Phase Detector.
 * @param cp_type The type of the Charge Pump output.
 * @param fast_lock_mode Sets the desired Fast Lock Mode.
 * @param timer_counter_control Duration the secondary charge pump current is
 * active before reverting to the primary current.
 * @param current_setting1 Used when the RF output is stable and the system is
 * in a static state.
 * @param current_setting2 Used when the system is dynamic and in a state of
 * change.
 * @param power_down2 Defines the type of the power down.
 * @param prescaler_value The value of the prescaler.
 ******************************************************************************/
struct adf4106_settings_t {

	/** Reference Input Frequency */
	uint32_t ref_in;

	/** PFD max frequency */
	uint32_t pfd_max;

	/** The initial value of the 14 bit Reference Counter register */
	uint16_t ref_counter : 14;
	/** The width of the anti-backlash pulse, this pulse
	 * ensures that no dead zone is in the PFD transfer function and minimizes
	 * phase noise and reference spurs.
	 */
	uint8_t anti_backlash_width : 2;
	/** Should be set to zero for Normal operation */
	uint8_t test_mode_bits : 1;
	/** determines the number of consecutive cycles of phase
	 * delay, that must occur before lock detect is set
	 */
	uint8_t lock_detect_precision : 1;

	/* N Latch */
	/** a 6 bits counter is supported at ADF4106 */
	uint8_t a_n_counter : 6;
	/** a 13 bits counter */
	uint16_t b_n_counter : 13;
	/** determines which charge pump current settings is used */
	uint8_t cp_gain : 1;

	/* Functional/Initialization latch */
	/** resets the R and N counters */
	uint8_t counter_reset : 1;
	/** activate power down mode */
	uint8_t power_down1 : 1;
	/** the type of the MUXOUT output */
	uint8_t muxout_control : 3;
	/** the polarity of the Phase Detector */
	uint8_t phase_detector_pol : 1;
	/** the type of the Charge Pump output */
	uint8_t cp_type : 1;
	/** set the desired Fast Lock Mode */
	uint8_t fast_lock_mode : 2;
	/** how long will be the secondary charge pump current
	 * active, before reverting to the primary current
	 */
	uint8_t timer_counter_control : 4;
	/** is used when the RF output is stable and the system is
	 * in static state
	 */
	uint8_t current_setting1 : 3;
	/** is meant to be used when the system is dynamic and in a
	 * state of change (i.e., when a new output frequency is programmed)
	 */
	uint8_t current_setting2 : 3;
	/** define the type of the power down */
	uint8_t power_down2 : 1;
	/** the value of the prescaler */
	uint8_t prescaler_value : 2;

};

/* Supported devices */
/***************************************************************************//**
 * @brief The `adf4106_type_t` is an enumeration that defines identifiers for
 * different types of devices supported by the ADF4106 driver,
 * specifically the ADF4001, ADF4002, and ADF4106. This enumeration is
 * used to specify the type of device being interfaced with, allowing the
 * driver to handle device-specific operations accordingly.
 *
 * @param ID_ADF4001 Represents the identifier for the ADF4001 device.
 * @param ID_ADF4002 Represents the identifier for the ADF4002 device.
 * @param ID_ADF4106 Represents the identifier for the ADF4106 device.
 ******************************************************************************/
enum adf4106_type_t {
	ID_ADF4001,
	ID_ADF4002,
	ID_ADF4106
};

/* Initialization methods */
/***************************************************************************//**
 * @brief The `adf4106_init_t` is an enumeration that defines the different
 * initialization methods available for the ADF4106 device. It includes
 * options for initializing the device using a latch, a CE pin, or a
 * counter reset, providing flexibility in how the device can be set up
 * for operation.
 *
 * @param INIT_LATCH Represents the initialization method using the latch.
 * @param INIT_CEPIN Represents the initialization method using the CE pin.
 * @param INIT_COUNTER_RESET Represents the initialization method using the
 * counter reset.
 ******************************************************************************/
enum adf4106_init_t {
	INIT_LATCH,
	INIT_CEPIN,
	INIT_COUNTER_RESET
};

/***************************************************************************//**
 * @brief The `adf4106_chip_info` structure is used to store frequency-related
 * parameters for the ADF4106 device, specifically the maximum and
 * minimum frequencies for both the Voltage-Controlled Oscillator (VCO)
 * and the Phase Frequency Detector (PFD). This information is crucial
 * for configuring and operating the ADF4106 device within its specified
 * frequency limits.
 *
 * @param vco_max_frequency Represents the maximum frequency of the Voltage-
 * Controlled Oscillator (VCO) in hertz.
 * @param pfd_max_frequency Indicates the maximum frequency of the Phase
 * Frequency Detector (PFD) in hertz.
 * @param vco_min_frequency Specifies the minimum frequency of the Voltage-
 * Controlled Oscillator (VCO) in hertz.
 * @param pfd_min_frequency Denotes the minimum frequency of the Phase Frequency
 * Detector (PFD) in hertz.
 ******************************************************************************/
struct adf4106_chip_info {
	uint64_t vco_max_frequency;
	uint32_t pfd_max_frequency;
	uint32_t vco_min_frequency;
	uint32_t pfd_min_frequency;
};

/***************************************************************************//**
 * @brief The `adf4106_dev` structure is a comprehensive representation of an
 * ADF4106 device, encapsulating all necessary components for its
 * operation. It includes SPI and GPIO descriptors for communication and
 * control, device-specific settings, and internal buffers for managing
 * latch data. This structure is essential for configuring and
 * interfacing with the ADF4106, a frequency synthesizer, and supports
 * various device types and initialization methods.
 *
 * @param spi_desc Pointer to an SPI descriptor for communication.
 * @param gpio_le Pointer to a GPIO descriptor for latch enable control.
 * @param gpio_ce Pointer to a GPIO descriptor for chip enable control.
 * @param gpio_le2 Pointer to a second GPIO descriptor for latch enable control.
 * @param gpio_ce2 Pointer to a second GPIO descriptor for chip enable control.
 * @param chip_info Contains information about the chip's frequency
 * capabilities.
 * @param adf4106_st Stores settings related to the ADF4106 device
 * configuration.
 * @param this_device Specifies the type of ADF4106 device being used.
 * @param r_latch Internal buffer for the reference counter latch.
 * @param n_latch Internal buffer for the N counter latch.
 * @param f_latch Internal buffer for the function latch.
 * @param i_latch Internal buffer for the initialization latch.
 ******************************************************************************/
struct adf4106_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_le;
	struct no_os_gpio_desc	*gpio_ce;
	struct no_os_gpio_desc	*gpio_le2;
	struct no_os_gpio_desc	*gpio_ce2;
	/* Device Settings */
	struct adf4106_chip_info chip_info;
	struct adf4106_settings_t adf4106_st;
	enum adf4106_type_t this_device;
	/* Internal buffers for each latch */
	uint32_t r_latch;
	uint32_t n_latch;
	uint32_t f_latch;
	uint32_t i_latch;
};

/***************************************************************************//**
 * @brief The `adf4106_init_param` structure is used to encapsulate all the
 * necessary initialization parameters for setting up an ADF4106 device.
 * It includes SPI and GPIO initialization parameters, device type,
 * initialization method, and specific device settings. This structure is
 * essential for configuring the ADF4106 device to operate correctly
 * within a system, ensuring that all communication and control
 * interfaces are properly initialized.
 *
 * @param spi_init Holds the SPI initialization parameters.
 * @param gpio_le Holds the GPIO initialization parameters for the LE pin.
 * @param gpio_ce Holds the GPIO initialization parameters for the CE pin.
 * @param gpio_le2 Holds the GPIO initialization parameters for the second LE
 * pin.
 * @param gpio_ce2 Holds the GPIO initialization parameters for the second CE
 * pin.
 * @param this_device Specifies the type of ADF4106 device being used.
 * @param init_method Specifies the initialization method for the device.
 * @param adf4106_st Holds the settings for the ADF4106 device.
 ******************************************************************************/
struct adf4106_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param		gpio_le;
	struct no_os_gpio_init_param		gpio_ce;
	struct no_os_gpio_init_param		gpio_le2;
	struct no_os_gpio_init_param		gpio_ce2;
	/* Device Settings */
	enum adf4106_type_t this_device;
	enum adf4106_init_t init_method;
	struct adf4106_settings_t adf4106_st;
};

/*****************************************************************************/
/*  Functions Prototypes                                                     */
/*****************************************************************************/
/* Initialize the communication with the device */
/***************************************************************************//**
 * @brief This function sets up the ADF4106 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function handles the initialization
 * of SPI and GPIO interfaces and configures the device based on the
 * specified initialization method. It returns a status code indicating
 * success or failure, which should be checked by the caller to ensure
 * the device is ready for use.
 *
 * @param device A pointer to a pointer of type `struct adf4106_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct adf4106_init_param` containing
 * the initialization parameters for the device, including SPI
 * and GPIO configurations, device type, and initialization
 * method. All fields must be properly set before calling the
 * function.
 * @return Returns an int8_t status code: 0 for success, or a negative value for
 * failure. The `device` pointer is updated to point to the initialized
 * device structure on success.
 ******************************************************************************/
int8_t adf4106_init(struct adf4106_dev **device,
		    struct adf4106_init_param init_param);

/* Free the resources allocated by adf4106_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADF4106 device after it is no longer needed. This function should be
 * called to clean up and free memory, as well as to ensure that all
 * associated SPI and GPIO resources are correctly released. It is
 * important to call this function to prevent resource leaks in your
 * application.
 *
 * @param dev A pointer to an adf4106_dev structure representing the device to
 * be removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource removal process. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t adf4106_remove(struct adf4106_dev *dev);

/* Update register function */
/***************************************************************************//**
 * @brief This function is used to update one of the internal latches of the
 * ADF4106 device with the provided latch data. It should be called when
 * there is a need to modify the configuration of the device, such as
 * changing the reference counter, N counter, function latch, or
 * initialization latch. The function requires a valid device structure
 * and latch data that specifies which latch to update. It communicates
 * with the device via SPI and generates a load pulse to apply the
 * changes.
 *
 * @param dev A pointer to an adf4106_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param latch_data A 32-bit unsigned integer containing the data to be written
 * to the latch. The lower two bits determine the latch type
 * to update. Invalid latch types may result in undefined
 * behavior.
 * @return None
 ******************************************************************************/
void adf4106_update_latch(struct adf4106_dev *dev,
			  uint32_t latch_data);

/* Return the value of a desired latch */
/***************************************************************************//**
 * @brief This function retrieves the value of a specified latch from the
 * ADF4106 device, which is useful for reading the current configuration
 * or status of the device. It requires a valid device structure and a
 * latch type identifier. The function should be called only after the
 * device has been properly initialized. If an invalid latch type is
 * provided, the function returns -1, indicating an error.
 *
 * @param dev A pointer to an initialized adf4106_dev structure representing the
 * device. Must not be null.
 * @param latch_type An unsigned 8-bit integer specifying the type of latch to
 * read. Valid values are ADF4106_CTRL_R_COUNTER,
 * ADF4106_CTRL_N_COUNTER, ADF4106_CTRL_FUNCTION_LATCH, and
 * ADF4106_CTRL_INIT_LATCH. If an invalid value is provided,
 * the function returns -1.
 * @return Returns a 32-bit unsigned integer representing the value of the
 * specified latch, or -1 if the latch type is invalid.
 ******************************************************************************/
uint32_t adf4106_read_latch(struct adf4106_dev *dev,
			    uint8_t latch_type);

/* PLL initialization functions */
/***************************************************************************//**
 * @brief This function configures the ADF4106 device by programming its
 * initialization, R, and N latches using the latch method. It should be
 * called to set up the device before any frequency setting operations.
 * The function assumes that the device structure has been properly
 * initialized and configured with the desired settings. It does not
 * return any value and does not handle invalid input explicitly, so the
 * caller must ensure that the provided device structure is valid and
 * correctly set up.
 *
 * @param dev A pointer to an adf4106_dev structure representing the device to
 * be initialized. Must not be null and should be properly
 * initialized with the desired settings before calling this
 * function. The caller retains ownership of the structure.
 * @return None
 ******************************************************************************/
void adf4106_init_latch_method(struct adf4106_dev *dev);
/***************************************************************************//**
 * @brief This function configures the ADF4106 device by programming its
 * function, R counter, and N counter latches, using the CE pin method to
 * control the device's power state. It should be called to initialize
 * the device after it has been properly configured and before any
 * frequency setting operations. The function ensures the device is
 * powered down initially, programs the necessary latches, and then
 * powers the device back up, allowing the input buffer bias to
 * stabilize. This function must be called with a valid device structure
 * that has been initialized with the desired settings.
 *
 * @param dev A pointer to an adf4106_dev structure representing the device to
 * be initialized. This structure must be properly initialized with
 * the desired settings before calling this function. The pointer
 * must not be null.
 * @return None
 ******************************************************************************/
void adf4106_init_cepin_method(struct adf4106_dev *dev);
/***************************************************************************//**
 * @brief This function configures the ADF4106 device by programming its latches
 * using the counter reset method. It should be called to initialize the
 * device after it has been properly set up with the necessary SPI and
 * GPIO configurations. This method involves enabling and then disabling
 * the counter reset to ensure the device is correctly initialized. It is
 * essential to ensure that the `dev` structure is fully initialized and
 * valid before calling this function.
 *
 * @param dev A pointer to an `adf4106_dev` structure representing the device to
 * be initialized. This structure must be properly initialized and
 * must not be null. The function assumes ownership of the structure
 * for the duration of the call.
 * @return None
 ******************************************************************************/
void adf4106_init_counte_reset_method(struct adf4106_dev *dev);

/* Set the frequency to a desired value */
/***************************************************************************//**
 * @brief This function sets the frequency of the ADF4106 device to the
 * specified value, adjusting it within the device's allowable frequency
 * range if necessary. It should be called when a new frequency is
 * required for the device operation. The function ensures that the
 * frequency is within the device's minimum and maximum VCO frequency
 * limits, and it calculates the appropriate counter values to achieve
 * the desired frequency. The function must be called with a valid device
 * structure that has been properly initialized.
 *
 * @param dev A pointer to an initialized adf4106_dev structure representing the
 * device. Must not be null.
 * @param frequency The desired frequency in Hz. It will be clamped to the
 * device's minimum and maximum VCO frequency range if it falls
 * outside these limits.
 * @return Returns the actual frequency set on the device, which may differ from
 * the requested frequency if clamping was applied.
 ******************************************************************************/
uint64_t adf4106_set_frequency(struct adf4106_dev *dev,
			       uint64_t frequency);

#endif // __ADF4106_H__

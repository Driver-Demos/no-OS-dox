/**************************************************************************//**
*   @file   adf4153.h
*   @brief  Header file of adf4153 driver.
*
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
#ifndef __ADF4153_H__
#define __ADF4153_H__

/*****************************************************************************/
/****************************** Include Files ********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/*****************************************************************************/
/*  Device specific MACROs                                                   */
/*****************************************************************************/
/* GPIOs */
#define ADF4153_LE_OUT                      no_os_gpio_direction_output(dev->gpio_le,  \
			                    NO_OS_GPIO_HIGH)
#define ADF4153_LE_LOW                      no_os_gpio_set_value(dev->gpio_le,         \
			                    NO_OS_GPIO_LOW)
#define ADF4153_LE_HIGH                     no_os_gpio_set_value(dev->gpio_le,         \
			                    NO_OS_GPIO_HIGH)

#define ADF4153_CE_OUT                      no_os_gpio_direction_output(dev->gpio_ce,  \
			                    NO_OS_GPIO_HIGH)
#define ADF4153_CE_LOW                      no_os_gpio_set_value(dev->gpio_ce,         \
			                    NO_OS_GPIO_LOW)
#define ADF4153_CE_HIGH                     no_os_gpio_set_value(dev->gpio_ce,         \
			                    NO_OS_GPIO_HIGH)

#define ADF4153_LE2_OUT                     no_os_gpio_direction_output(dev->gpio_le2, \
			                    NO_OS_GPIO_HIGH)
#define ADF4153_LE2_LOW                     no_os_gpio_set_value(dev->gpio_le2,        \
			                    NO_OS_GPIO_LOW)
#define ADF4153_LE2_HIGH                    no_os_gpio_set_value(dev->gpio_le2,        \
			                    NO_OS_GPIO_HIGH)

#define ADF4153_CE2_OUT                     no_os_gpio_direction_output(dev->gpio_ce2, \
			                    NO_OS_GPIO_HIGH)
#define ADF4153_CE2_LOW                     no_os_gpio_set_value(dev->gpio_ce2,        \
			                    NO_OS_GPIO_LOW)
#define ADF4153_CE2_HIGH                    no_os_gpio_set_value(dev->gpio_ce2,        \
			                    NO_OS_GPIO_HIGH)

/* Control Bits */
#define ADF4153_CTRL_MASK                   0x3

#define ADF4153_CTRL_N_DIVIDER              0          /* N Divider Register */
#define ADF4153_CTRL_R_DIVIDER              1          /* R Divider Register */
#define ADF4153_CTRL_CONTROL                2          /* Control Register */
#define ADF4153_CTRL_NOISE_SPUR             3          /* Noise and Spur Reg*/

/* N Divider Register */

/* 12-bit fractional value */
#define ADF4153_R0_FRAC_OFFSET              2
#define ADF4153_R0_FRAC_MASK                0xFFFul
#define ADF4153_R0_FRAC(x)                  ((x) & ADF4153_R0_FRAC_MASK) \
                                              << ADF4153_R0_FRAC_OFFSET
/* 9-bit integer value */
#define ADF4153_R0_INT_OFFSET               14
#define ADF4153_R0_INT_MASK                 0x1FFul
#define ADF4153_R0_INT(x)                   ((x) & ADF4153_R0_INT_MASK) \
                                              << ADF4153_R0_INT_OFFSET

/* Fast-Lock */
#define ADF4153_R0_FASTLOCK_OFFSET          23
#define ADF4153_R0_FASTLOCK_MASK            0x1
#define ADF4153_R0_FASTLOCK(x)              ((x) & ADF4153_R0_FASTLOCK_MASK) \
                                              << ADF4153_R0_FASTLOCK_OFFSET

/* R Divider Register */

/* 12-bit interpolator modulus value */
#define ADF4153_R1_MOD_OFFSET               2
#define ADF4153_R1_MOD_MASK                 0xFFFul
#define ADF4153_R1_MOD(x)                   ((x) & ADF4153_R1_MOD_MASK) \
                                              << ADF4153_R1_MOD_OFFSET
/* 4-bit R Counter */
#define ADF4153_R1_RCOUNTER_OFFSET          14
#define ADF4153_R1_RCOUNTER_MASK            0xFul
#define ADF4153_R1_RCOUNTER(x)              ((x) & ADF4153_R1_RCOUNTER_MASK) \
                                              << ADF4153_R1_RCOUNTER_OFFSET
/* Prescale */
#define ADF4153_R1_PRESCALE_OFFSET          18
#define ADF4153_R1_PRESCALE_MASK            0x1ul
#define ADF4153_R1_PRESCALE(x)              ((x) & ADF4153_R1_PRESCALE_MASK) \
                                              << ADF4153_R1_PRESCALE_OFFSET
/* MUXOUT */
#define ADF4153_R1_MUXOUT_OFFSET            20
#define ADF4153_R1_MUXOUT_MASK              0x7
#define ADF4153_R1_MUXOUT(x)                ((x) & ADF4153_R1_MUXOUT_MASK) \
                                              << ADF4153_R1_MUXOUT_OFFSET
/* Load Control */
#define ADF4153_R1_LOAD_OFFSET              23
#define ADF4153_R1_LOAD_MASK                0x1
#define ADF4153_R1_LOAD(x)                  ((x) & ADF4153_R1_LOAD_MASK) \
                                              << ADF4153_R1_LOAD_OFFSET

/* Control Register */

/* Counter Reset */
#define ADF4153_R2_COUNTER_RST_OFFSET       2
#define ADF4153_R2_COUNTER_RST_MASK         0x1ul
#define ADF4153_R2_COUNTER_RST(x)           ((x) & ADF4153_R2_COUNTER_RST_MASK)\
                                               << ADF4153_R2_COUNTER_RST_OFFSET
/* CP Three-State */
#define ADF4153_R2_CP_3STATE_OFFSET         3
#define ADF4153_R2_CP_3STATE_MASK           0x1
#define ADF4153_R2_CP_3STATE(x)             ((x) & ADF4153_R2_CP_3STATE_MASK) \
                                               << ADF4153_R2_CP_3STATE_OFFSET
/* Power-down */
#define ADF4153_R2_POWER_DOWN_OFFSET        4
#define ADF4153_R2_POWER_DOWN_MASK          0x1
#define ADF4153_R2_POWER_DOWN(x)            ((x) & ADF4153_R2_POWER_DOWN_MASK) \
                                               <<   ADF4153_R2_POWER_DOWN_OFFSET
/* LDP */
#define ADF4153_R2_LDP_OFFSET               5
#define ADF4153_R2_LDP_MASK                 0x1
#define ADF4153_R2_LDP(x)                   ((x) & ADF4153_R2_LDP_MASK) \
                                               << ADF4153_R2_LDP_OFFSET
/* PD Polarity */
#define ADF4153_R2_PD_POL_OFFSET            6
#define ADF4153_R2_PD_POL_MASK              0x1
#define ADF4153_R2_PD_POL(x)                ((x) & ADF4153_R2_PD_POL_MASK) \
                                               << ADF4153_R2_PD_POL_OFFSET
/* CP Current Settings and CP/2 */
#define ADF4153_R2_CP_CURRENT_OFFSET        7
#define ADF4153_R2_CP_CURRENT_MASK          0xF
#define ADF4153_R2_CP_CURRENT(x)            ((x) & ADF4153_R2_CP_CURRENT_MASK) \
                                               << ADF4153_R2_CP_CURRENT_OFFSET
/* Reference doubler */
#define ADF4153_R2_REF_DOUBLER_OFFSET       11
#define ADF4153_R2_REF_DOUBLER_MASK         0x1
#define ADF4153_R2_REF_DOUBLER(x)           ((x) & ADF4153_R2_REF_DOUBLER_MASK)\
                                              << ADF4153_R2_REF_DOUBLER_OFFSET
/* Resync */
#define ADF4153_R2_RESYNC_OFFSET            12
#define ADF4153_R2_RESYNC_MASK              0x7
#define ADF4153_R2_RESYNC(x)                ((x) & ADF4153_R2_RESYNC_MASK) \
                                              << ADF4153_R2_RESYNC_OFFSET

/* Noise and spur register */

/* Noise and spur mode */
#define ADF4153_R3_NOISE_SPURG_MASK         0x3C4
#define ADF4153_R3_NOISE_SPURG(x)           ( (((x) << 0x2) & 0x7) | \
                                              (((x) >> 0x1) << 0x6) ) &\
                                               ADF4153_R3_NOISE_SPURG_MASK

/* Fast-Lock definitions */
#define ADF4153_FASTLOCK_DISABLED           0
#define ADF4153_FASTLOCK_ENABLED            1
/* Prescale definitions */
#define ADF4153_PRESCALER_4_5               0
#define ADF4153_PRESCALER_8_9               1
/* Muxout definitions */
#define ADF4153_MUXOUT_THREESTATE           0
#define ADF4153_MUXOUT_DIGITAL_LOCK         1
#define ADF4153_MUXOUT_NDIV_OUTPUT          2
#define ADF4153_MUXOUT_LOGICHIGH            3
#define ADF4153_MUXOUT_RDIV_OUTPUT          4
#define ADF4153_MUXOUT_ANALOG_LOCK          5
#define ADF4153_MUXOUT_FASTLOCK             6
#define ADF4153_MUXOUT_LOGICLOW             7
/* Load Control definitions */
#define ADF4153_LOAD_NORMAL                 0
#define ADF4153_LOAD_RESYNC                 1
/* Counter Reset Definitions */
#define ADF4153_CR_DISABLED                 0
#define ADF4153_CR_ENABLED                  1
/* CP Three-state definitions */
#define ADF4153_CP_DISABLED                 0
#define ADF4153_CP_THREE_STATE              1
/* Power-down definitions */
#define ADF4153_PD_DISABLED                 0
#define ADF4153_PD_ENABLED                  1
/* LDP definitions */
#define ADF4153_LDP_24                      0
#define ADF4153_LDP_40                      1
/* PD Polarity definitions */
#define ADF4153_PD_POL_NEGATIV              0
#define ADF4153_PD_POL_POSITIVE             1
/* CR Current Settings definitions */
#define ADF4153_CP_CURRENT_0_63             0
#define ADF4153_CP_CURRENT_1_25             1
#define ADF4153_CP_CURRENT_1_88             2
#define ADF4153_CP_CURRENT_2_50             3
#define ADF4153_CP_CURRENT_3_13             4
#define ADF4153_CP_CURRENT_3_75             5
#define ADF4153_CP_CURRENT_4_38             6
#define ADF4153_CP_CURRENT_5_00             7
#define ADF4153_CP2_CURRENT_0_31            8
#define ADF4153_CP2_CURRENT_0_63            9
#define ADF4153_CP2_CURRENT_0_94            10
#define ADF4153_CP2_CURRENT_1_25            11
#define ADF4153_CP2_CURRENT_1_57            12
#define ADF4153_CP2_CURRENT_1_88            13
#define ADF4153_CP2_CURRENT_2_19            14
#define ADF4153_CP2_CURRENT_2_50            15

/* Reference doubler definition */
#define ADF4153_REF_DOUBLER_DIS             0
#define ADF4153_REF_DOUBLER_EN              1
/* Noise and Spur mode definitions */
#define ADF4153_LOW_SPUR_MODE               0b00000
#define ADF4153_LOW_NOISE_SPUR              0b11100
#define ADF4153_LOWEST_NOISE                0b11111

/*****************************************************************************/
/************************** Types Declarations *******************************/
/*****************************************************************************/

/**
*   @struct adf41053_settings_t
*   @brief store the value of all the latch and the input
*                               reference frequency
*/

/***************************************************************************//**
 * @brief The `adf4153_settings_t` structure is used to store configuration
 * settings for the ADF4153 frequency synthesizer, including reference
 * input frequency, channel spacing, and various divider and control
 * settings. It includes fields for fractional and integer values for the
 * N divider, modulus and R counter for the R divider, and several
 * control bits for fast-lock, prescaler, multiplexer output, and charge
 * pump settings. This structure allows for detailed configuration of the
 * synthesizer's operation, enabling precise control over frequency
 * synthesis and performance optimization.
 *
 * @param ref_in Reference input frequency for the device.
 * @param channel_spacing Defines the channel resolution or spacing.
 * @param frac_value 12-bit value controlling the fractional interpolator.
 * @param int_value 9-bit value determining the overall division factor.
 * @param fastlock Enables fast-lock when set to logic high.
 * @param mod_value 12-bit fractional modulus for PFD frequency to channel step
 * resolution.
 * @param r_counter 4-bit counter dividing the input reference frequency for the
 * PFD.
 * @param prescaler Determines the overall division ratio with INT, FRAC, and
 * MOD counters.
 * @param muxout 3-bit on-chip multiplexer selection bits.
 * @param load_control Controls resync delay of the Sigma-Delta when set to
 * logic high.
 * @param counter_reset Resets the R and N counters.
 * @param cp_three_state Puts the charge pump into three-state mode when set to
 * 1.
 * @param power_down Activates power down mode.
 * @param ldp Controls lock detect precision.
 * @param pd_polarity Sets phase detector polarity.
 * @param cp_current Charge pump current settings for loop filter design.
 * @param ref_doubler Enables REFin doubler for active edges at PFD input.
 * @param resync Defines time between two resyncs; disables phase resync if
 * zero.
 * @param noise_spur Optimizes design for spurious or phase noise performance.
 ******************************************************************************/
struct adf4153_settings_t {

	/* Reference Input Frequency*/
	uint32_t ref_in;
	/* Channel resolution or Channel spacing */
	uint32_t channel_spacing;

	/* N Divider */
	/** these 12 bits control what is loaded as the FRAC value into
	 * the fractional interpolator.
	 */
	uint16_t frac_value : 12;
	/** these nine bits control what is loaded as the INT value, this
	 * is used to determine the overall division factor.
	 */
	uint16_t int_value : 9;
	/** when set to logic high fast-lock is enabled */
	uint8_t fastlock : 1;

	/* R Divider */
	/** set the fractional modulus, this is the ratio of the PFD
	 * frequency to the channel step resolution on the RF output
	 */
	uint16_t mod_value : 12;
	/** the r counter allows the input reference frequency to
	* be divided down to produce the reference clock to phase
	* frequency detector
	*/
	uint8_t r_counter : 4;
	/** the dual-modulus prescaler, along with the INT, FRAC and MOD
	 * counters, determines the overall division ratio from the RFin
	 * to PFD input
	 */
	uint8_t prescaler : 1;
	/** the on chip multiplexer selection bits */
	uint8_t muxout : 3;
	/** when set to logic high the value being programmed in the
	 * modulus is not loaded into the modulus. Instead, it sets the
	 * resync delay of the Sigma-Delta.
	 */
	uint8_t load_control : 1;

	/* Control Register */
	/** resets the R and N counters */
	uint8_t counter_reset : 1;
	/** puts the charge pump into three-state mode when programmed
	 * to 1
	 */
	uint8_t cp_three_state : 1;
	/** activate power down mode */
	uint8_t power_down : 1;
	/** lock detect precision */
	uint8_t ldp : 1;
	/** phase detector polarity */
	uint8_t pd_polarity : 1;
	/** Charge Pump Current settings, this should be set to the charge
	 * pump current that the loop filter is designed with
	 */
	uint8_t cp_current : 4;
	/** REFin Doubler, when the doubler is enabled, both the rising
	 * and falling edges of REFin become active edges at the PFD input
	 */
	uint8_t ref_doubler : 1;
	/** define the time between two resync, if it is zero, than
	 * the phase resync feature is disabled
	 */
	uint8_t resync : 4;

	/* Noise and Spur register */
	/** allows the user to optimize a design either for improved
	 * spurious performance or for improved phase noise performance
	 */
	uint8_t noise_spur : 5;

};

/***************************************************************************//**
 * @brief The `adf4153_dev` structure is a comprehensive representation of the
 * ADF4153 device, encapsulating all necessary components for its
 * operation. It includes SPI and GPIO descriptors for communication and
 * control, as well as a settings structure for device configuration. The
 * structure also defines frequency limits for RF input and VCO output,
 * and maintains internal buffers for various register values,
 * facilitating the management of the device's operational parameters.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_le Pointer to the GPIO descriptor for latch enable control.
 * @param gpio_ce Pointer to the GPIO descriptor for chip enable control.
 * @param gpio_le2 Pointer to the second GPIO descriptor for latch enable
 * control.
 * @param gpio_ce2 Pointer to the second GPIO descriptor for chip enable
 * control.
 * @param adf4153_st Structure containing device settings and configuration.
 * @param adf4153_rfin_min_frq Minimum RF input frequency limit.
 * @param adf4153_rfin_max_frq Maximum RF input frequency limit.
 * @param adf4153_pfd_max_frq Maximum phase frequency detector frequency.
 * @param adf4153_vco_min_frq Minimum VCO output frequency limit.
 * @param adf4153_vco_max_frq Maximum VCO output frequency limit.
 * @param adf4153_mod_max Maximum interpolator modulus value.
 * @param r0 Internal buffer for the N Divider Register value.
 * @param r1 Internal buffer for the R Divider Register value.
 * @param r2 Internal buffer for the Control Register value.
 * @param r3 Internal buffer for the Noise and Spur Register value.
 ******************************************************************************/
struct adf4153_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_le;
	struct no_os_gpio_desc	*gpio_ce;
	struct no_os_gpio_desc	*gpio_le2;
	struct no_os_gpio_desc	*gpio_ce2;
	/* Device Settings */
	struct adf4153_settings_t adf4153_st;
	/* RF input frequency limits */
	uint32_t adf4153_rfin_min_frq;
	uint32_t adf4153_rfin_max_frq;
	/* Maximum PFD frequency */
	uint32_t adf4153_pfd_max_frq;
	/* VCO out frequency limits */
	uint32_t adf4153_vco_min_frq;
	uint64_t adf4153_vco_max_frq;
	/* maximum interpolator modulus value */
	uint16_t adf4153_mod_max;

	/* Internal buffers for each latch */
	uint32_t r0;               /* the actual value of N Divider Register */
	uint32_t r1;               /* the actual value of R Divider Register */
	uint32_t r2;               /* the actual value of Control Register */
	uint32_t r3;               /* the actual value of Noise and Spur Reg*/
};

/***************************************************************************//**
 * @brief The `adf4153_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the ADF4153 device. It
 * includes SPI and GPIO initialization parameters, as well as specific
 * device settings encapsulated in the `adf4153_settings_t` structure.
 * This structure is essential for configuring the device's communication
 * interfaces and operational parameters before use.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_le Holds the initialization parameters for the first GPIO line
 * enable.
 * @param gpio_ce Holds the initialization parameters for the first GPIO chip
 * enable.
 * @param gpio_le2 Holds the initialization parameters for the second GPIO line
 * enable.
 * @param gpio_ce2 Holds the initialization parameters for the second GPIO chip
 * enable.
 * @param adf4153_st Contains the device-specific settings for the ADF4153.
 ******************************************************************************/
struct adf4153_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_le;
	struct no_os_gpio_init_param	gpio_ce;
	struct no_os_gpio_init_param	gpio_le2;
	struct no_os_gpio_init_param	gpio_ce2;
	/* Device Settings */
	struct adf4153_settings_t adf4153_st;
};

/*****************************************************************************/
/*  Functions Prototypes                                                     */
/*****************************************************************************/
/* Initialize the communication with the device */
/***************************************************************************//**
 * @brief This function sets up the ADF4153 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device. The function initializes SPI communication and configures GPIO
 * pins required for device operation. It also sets default frequency
 * limits and other device-specific settings. If the initialization
 * fails, the function returns an error code, and the device pointer is
 * not valid.
 *
 * @param device A pointer to a pointer of type `struct adf4153_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adf4153_init_param` containing
 * initialization parameters for the device, including SPI and
 * GPIO configurations. The caller retains ownership of this
 * structure.
 * @return Returns 0 on success or a negative error code if initialization
 * fails. On success, the `device` pointer is set to point to a newly
 * allocated and initialized `adf4153_dev` structure.
 ******************************************************************************/
int8_t adf4153_init(struct adf4153_dev **device,
		    struct adf4153_init_param init_param);

/* Free the resources allocated by adf4153_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADF4153 device instance. It should be called when the device is no
 * longer needed, typically after all operations with the device are
 * complete. This function ensures that all associated SPI and GPIO
 * resources are freed, and the device structure is deallocated. It is
 * important to call this function to prevent resource leaks in the
 * system.
 *
 * @param dev A pointer to the ADF4153 device structure to be removed. Must not
 * be null. The function will handle the deallocation of resources
 * associated with this device.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during resource
 * deallocation.
 ******************************************************************************/
int32_t adf4153_remove(struct adf4153_dev *dev);

/* Update register function */
/***************************************************************************//**
 * @brief This function updates a specific latch of the ADF4153 device with the
 * provided data and sends the updated data to the device via SPI. It
 * should be called whenever there is a need to modify the configuration
 * of the ADF4153 device. The function also generates a load pulse to
 * ensure the new data is latched correctly. It is important to ensure
 * that the device has been properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an adf4153_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param latch_data A 32-bit unsigned integer containing the data to be written
 * to the latch. The lower two bits determine which latch is
 * updated, and the rest of the bits contain the data to be
 * written.
 * @return None
 ******************************************************************************/
void adf4153_update_latch(struct adf4153_dev *dev,
			  uint32_t latch_data);

/* Return the value of a desired latch */
/***************************************************************************//**
 * @brief Use this function to retrieve the current value of a specific latch
 * from the ADF4153 device. It is useful for reading back the
 * configuration of the device's internal registers. The function
 * requires a valid device structure and a latch type identifier. If an
 * invalid latch type is provided, the function returns -1, indicating an
 * error. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an adf4153_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param latch_type An unsigned 8-bit integer specifying the latch type to
 * read. Valid values are ADF4153_CTRL_N_DIVIDER,
 * ADF4153_CTRL_R_DIVIDER, ADF4153_CTRL_CONTROL, and
 * ADF4153_CTRL_NOISE_SPUR. Invalid values result in a return
 * value of -1.
 * @return Returns a 32-bit unsigned integer representing the value of the
 * specified latch. Returns -1 if the latch type is invalid.
 ******************************************************************************/
uint32_t adf4153_read_latch(struct adf4153_dev *dev,
			    uint8_t latch_type);

/* Set the frequency to a desired value */
/***************************************************************************//**
 * @brief This function configures the ADF4153 device to output a specified
 * frequency, adjusting internal settings to achieve the closest possible
 * frequency within the device's capabilities. It should be called after
 * the device has been initialized. The function ensures the requested
 * frequency is within the allowable range, clamping it to the nearest
 * valid value if necessary. It then calculates and sets the appropriate
 * divider values to achieve the desired frequency, updating the device's
 * internal registers accordingly.
 *
 * @param dev A pointer to an initialized adf4153_dev structure representing the
 * device. Must not be null.
 * @param frequency The desired output frequency in Hz. It should be within the
 * device's VCO frequency range, specified by
 * adf4153_vco_min_frq and adf4153_vco_max_frq. If the
 * frequency is outside this range, it will be clamped to the
 * nearest valid value.
 * @return Returns the actual frequency set on the device, which may differ
 * slightly from the requested frequency due to internal limitations.
 ******************************************************************************/
uint64_t adf4153_set_frequency(struct adf4153_dev *dev,
			       uint64_t frequency);

/* Return the value of the channel spacing */
/***************************************************************************//**
 * @brief Use this function to obtain the current channel spacing value
 * configured in the ADF4153 device. This function is useful when you
 * need to verify or utilize the channel spacing setting in your
 * application. Ensure that the device has been properly initialized
 * before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized adf4153_dev structure. This parameter
 * must not be null, and the device must be properly initialized
 * before use. If the pointer is invalid, the behavior is undefined.
 * @return Returns the current channel spacing value as a 32-bit unsigned
 * integer.
 ******************************************************************************/
uint32_t adf4153_get_channel_spacing(struct adf4153_dev *dev);

#endif // __ADF4153_H__

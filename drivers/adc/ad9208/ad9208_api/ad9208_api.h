/**
 * \file ad9208.h
 *
 * \brief AD9208 API interface header file
 *
 * This file contains all the publicly exposed methods and data structures to
 * interface with the AD9208 API.
 *
 * Release 1.0.X
 *
 * Copyright(c) 2017 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
 */
#ifndef __AD9208API_H__
#define __AD9208API_H__

#include "api_def.h"
#include "no_os_util.h"

/***************************************************************************//**
 * @brief The `ad9208_adc_data_frmt_t` is an enumeration that defines the data
 * format types for the AD9208 ADC, specifically distinguishing between
 * real and complex data formats. This enumeration is used to configure
 * the ADC's data output format, allowing users to select between real
 * and complex data representations based on their application
 * requirements.
 *
 * @param AD9208_DATA_FRMT_REAL Represents real data format with a value of 0x0.
 * @param AD9208_DATA_FRMT_COMPLEX Represents complex data format.
 ******************************************************************************/
typedef enum {
	AD9208_DATA_FRMT_REAL = 0x0,  /**< Real Data*/
	AD9208_DATA_FRMT_COMPLEX      /**< Complex Data */
} ad9208_adc_data_frmt_t;

/***************************************************************************//**
 * @brief The `ad9208_adc_ch_t` is an enumeration that defines the possible ADC
 * channel selections for the AD9208 device. It allows for specifying
 * whether no channel, a specific channel (A or B), or all channels are
 * selected for operations. This enumeration is used in various API
 * functions to configure and manage the ADC channels on the AD9208.
 *
 * @param AD9208_ADC_CH_NONE Represents no ADC channel selected.
 * @param AD9208_ADC_CH_A Represents ADC Channel A.
 * @param AD9208_ADC_CH_B Represents ADC Channel B.
 * @param AD9208_ADC_CH_ALL Represents all ADC channels selected.
 ******************************************************************************/
typedef enum {
	AD9208_ADC_CH_NONE = 0x0,   /**< No ADC Channel*/
	AD9208_ADC_CH_A,	    /**< ADC Channel A */
	AD9208_ADC_CH_B,	    /**< ADC Channel B*/
	AD9208_ADC_CH_ALL	    /**< ALL ADC Channels*/
} ad9208_adc_ch_t;

/***************************************************************************//**
 * @brief The `ad9208_adc_scale_range_t` is an enumeration that defines the
 * possible full-scale voltage ranges for the AD9208 ADC. Each enumerator
 * corresponds to a specific differential voltage peak-to-peak (Vpp)
 * value, allowing the user to select the appropriate scale for their
 * application. This is crucial for configuring the ADC to match the
 * expected input signal levels, ensuring optimal performance and
 * accuracy.
 *
 * @param AD9208_ADC_SCALE_2P04_VPP Represents a full-scale range of 2.04 Vpp
 * differential.
 * @param AD9208_ADC_SCALE_1P13_VPP Represents a full-scale range of 1.13 Vpp
 * differential.
 * @param AD9208_ADC_SCALE_1P25_VPP Represents a full-scale range of 1.25 Vpp
 * differential.
 * @param AD9208_ADC_SCALE_1P7_VPP Represents a full-scale range of 1.70 Vpp
 * differential.
 * @param AD9208_ADC_SCALE_1P81_VPP Represents a full-scale range of 1.81 Vpp
 * differential.
 * @param AD9208_ADC_SCALE_1P93_VPP Represents a full-scale range of 1.93 Vpp
 * differential.
 ******************************************************************************/
typedef enum {
	AD9208_ADC_SCALE_2P04_VPP = 0,	/**< 2.04 Vpp Differential*/
	AD9208_ADC_SCALE_1P13_VPP,	/**< 1.13 Vpp Differential */
	AD9208_ADC_SCALE_1P25_VPP,	/**< 1.25 Vpp Differential */
	AD9208_ADC_SCALE_1P7_VPP,	/**< 1.70 Vpp Differential*/
	AD9208_ADC_SCALE_1P81_VPP,	/**< 1.81 Vpp Differential*/
	AD9208_ADC_SCALE_1P93_VPP	/**< 1.93 Vpp Differential*/
} ad9208_adc_scale_range_t;

/***************************************************************************//**
 * @brief The `ad9208_adc_buff_curr_t` is an enumeration that defines various
 * buffer current settings for the AD9208 ADC input buffer. Each
 * enumerator represents a specific current value in microamperes (uA)
 * that can be set for the buffer, allowing for optimization of the ADC's
 * performance based on the input signal characteristics. This
 * configuration is crucial for managing the signal integrity and power
 * consumption of the ADC.
 *
 * @param AD9208_ADC_BUFF_CURR_400_UA Buffer Current set to 400 uA.
 * @param AD9208_ADC_BUFF_CURR_500_UA Buffer Current set to 500 uA.
 * @param AD9208_ADC_BUFF_CURR_600_UA Buffer Current set to 600 uA.
 * @param AD9208_ADC_BUFF_CURR_700_UA Buffer Current set to 700 uA.
 * @param AD9208_ADC_BUFF_CURR_800_UA Buffer Current set to 800 uA.
 * @param AD9208_ADC_BUFF_CURR_1000_UA Buffer Current set to 1000 uA.
 ******************************************************************************/
typedef enum {
	AD9208_ADC_BUFF_CURR_400_UA = 0x4,   /**< Buffer Current set to 400 uA*/
	AD9208_ADC_BUFF_CURR_500_UA = 0x9,   /**< Buffer Current set to 500 uA*/
	AD9208_ADC_BUFF_CURR_600_UA = 0x1E,  /**< Buffer Current set to 600 uA*/
	AD9208_ADC_BUFF_CURR_700_UA = 0x23,  /**< Buffer Current set to 700 uA*/
	AD9208_ADC_BUFF_CURR_800_UA = 0x28,  /**< Buffer Current set to 800 uA*/
	AD9208_ADC_BUFF_CURR_1000_UA = 0x32, /**< Buffer Current set to 1000 uA*/
} ad9208_adc_buff_curr_t;

/***************************************************************************//**
 * @brief The `ad9208_adc_nco_mode_t` is an enumeration that defines the various
 * modes of operation for the Numerically Controlled Oscillator (NCO) in
 * the AD9208 ADC. It includes modes for variable intermediate frequency
 * (IF), zero IF, a test mode, and an invalid mode to handle erroneous
 * configurations. This enumeration is used to configure the NCO mode for
 * digital down-conversion in the AD9208 device.
 *
 * @param AD9208_ADC_NCO_VIF Variable IF Mode.
 * @param AD9208_ADC_NCO_ZIF Zero IF Mode.
 * @param AD9208_ADC_NCO_TEST Test Mode.
 * @param AD9208_ADC_NCO_INVLD Invalid NCO Mode.
 ******************************************************************************/
typedef enum {
	AD9208_ADC_NCO_VIF = 0,	  /**< Variable IF Mode*/
	AD9208_ADC_NCO_ZIF = 1,	  /**< Zero IF Mode */
	AD9208_ADC_NCO_TEST = 3,  /**< Test Mode*/
	AD9208_ADC_NCO_INVLD = 4  /**< Invalid NCO Mode*/
} ad9208_adc_nco_mode_t;

/***************************************************************************//**
 * @brief The `ad9208_pdn_mode_t` is an enumeration that defines the power-down
 * modes for the AD9208 device. It includes three modes: `AD9208_POWERUP`
 * for normal operation, `AD9208_STANDBY` for standby mode, and
 * `AD9208_POWERDOWN` for full power-down. These modes are used to
 * control the power state of the AD9208 device, allowing for efficient
 * power management depending on the operational requirements.
 *
 * @param AD9208_POWERUP Normal operational power-up mode.
 * @param AD9208_STANDBY Standby mode power-up.
 * @param AD9208_POWERDOWN Full power-down mode.
 ******************************************************************************/
typedef enum {
	AD9208_POWERUP = 0x0,	/**< Normal Operational Powerup*/
	AD9208_STANDBY = 0x2,	/**< Standby Mode Powerup */
	AD9208_POWERDOWN = 0x3	/**< Full Powerdown Mode*/
} ad9208_pdn_mode_t;

/***************************************************************************//**
 * @brief The `ad9208_jesd_serdes_pll_flg_t` is an enumeration that defines
 * status flags for the SERDES PLL in the AD9208 device. It includes
 * flags to indicate whether the PLL is locked or if it has lost lock,
 * which are critical for monitoring the stability and performance of the
 * PLL in high-speed data transmission applications.
 *
 * @param AD9208_PLL_LOCK_STAT Serdes PLL lock Status Flag.
 * @param AD9208_PLL_LOSSLOCK Serdes PLL Lost Lock Status Flag.
 ******************************************************************************/
typedef enum {
	AD9208_PLL_LOCK_STAT = 0x8, /**< Serdes PLL lock Status Flag*/
	AD9208_PLL_LOSSLOCK = 0x4   /**< Serdes PLL Lost Lock Status Flag*/
} ad9208_jesd_serdes_pll_flg_t;

/***************************************************************************//**
 * @brief The `ad9208_handle_t` structure is a comprehensive data structure used
 * to manage the initialization, configuration, and operation of the
 * AD9208 device. It includes pointers to user-defined data and several
 * function pointers for handling SPI access, reset pin control, delay
 * operations, and hardware initialization and de-initialization.
 * Additionally, it stores the ADC clock frequency, which is crucial for
 * the device's operation within a specified range. This structure serves
 * as a central reference for interfacing with the AD9208 API,
 * facilitating various device operations and configurations.
 *
 * @param user_data Void pointer to user defined data for HAL initialization.
 * @param dev_xfer Function pointer to HAL SPI access function.
 * @param reset_pin_ctrl Function pointer to HAL RESETB pin control function.
 * @param delay_us Function pointer to HAL delay function.
 * @param hw_open Function pointer to HAL initialization function.
 * @param hw_close Function pointer to HAL de-initialization function.
 * @param adc_clk_freq_hz ADC clock frequency in Hz, valid range 2.5GHz to
 * 3.1GHz.
 ******************************************************************************/
typedef struct {
	void *user_data;	/**< Void pointer to user defined data for HAL initialization */
	spi_xfer_t dev_xfer;	/**< Function Pointer to HAL SPI access function*/
	reset_pin_ctrl_t reset_pin_ctrl;
	/**< Function Point to HAL RESETB Pin Ctrl Function*/
	delay_us_t delay_us;	/**< Function Pointer to HAL delay function*/
	hw_open_t hw_open;	/**< Function Pointer to HAL initialization function*/
	hw_close_t hw_close;	/**< Function Pointer to HAL de-initialization function*/
	uint64_t adc_clk_freq_hz;   /**< ADC Clock Frequency in Hz. Valid range 2.5GHz to 3.1 GHz */
} ad9208_handle_t;

/***************************************************************************//**
 * @brief This function initializes the AD9208 device and must be called before
 * any other API functions. It sets up the internal API state and memory,
 * and optionally initializes hardware resources if the `hw_open`
 * function pointer in the handle is not null. This function is essential
 * for preparing the device for further operations and should be followed
 * by a reset to ensure all SPI registers are set to default values.
 * Proper initialization is crucial for the correct functioning of
 * subsequent API calls.
 *
 * @param h A pointer to an `ad9208_handle_t` structure, which must not be null.
 * This structure should have valid function pointers for SPI transfer
 * (`dev_xfer`), delay (`delay_us`), and optionally hardware open
 * (`hw_open`). If any required pointer is null, the function returns
 * an error.
 * @return Returns an integer status code: `API_ERROR_OK` on success, or an
 * error code indicating the type of failure, such as invalid handle or
 * function pointer, or hardware initialization failure.
 ******************************************************************************/
int ad9208_init(ad9208_handle_t *h);

/***************************************************************************//**
 * @brief This function should be called as the final step in the AD9208 device
 * lifecycle to properly de-initialize the device and release any
 * associated resources. It ensures that any hardware resources allocated
 * during initialization are properly closed if the `hw_close` function
 * pointer in the handle is not null. This function must not be called if
 * the handle is null, as it will return an error. After calling this
 * function, no other API functions should be invoked on the device.
 *
 * @param h A pointer to the AD9208 device reference handle. It must not be
 * null, and the caller retains ownership. If the handle is null, the
 * function returns an error indicating an invalid handle pointer.
 * @return Returns an integer status code: `API_ERROR_OK` on success,
 * `API_ERROR_INVALID_HANDLE_PTR` if the handle is null, or
 * `API_ERROR_HW_CLOSE` if the hardware close operation fails.
 ******************************************************************************/
int ad9208_deinit(ad9208_handle_t *h);

/***************************************************************************//**
 * @brief This function resets the AD9208 device, either through a hardware pin
 * or via SPI register, depending on the `hw_reset` parameter. It should
 * be used to ensure the device is in a known state, typically after
 * initialization. If a hardware reset is requested, the function will
 * control the reset pin and may introduce a delay if a delay function is
 * provided. The function requires a valid device handle and will return
 * an error if the handle is null or if the reset pin control function is
 * not set when a hardware reset is requested.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * handle should be properly initialized before calling this function.
 * @param hw_reset A uint8_t value indicating the type of reset. A value of 1
 * triggers a hardware reset, while 0 triggers a software reset.
 * If a hardware reset is requested, the reset pin control
 * function must be set in the handle.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors, such as invalid handle or
 * parameter issues.
 ******************************************************************************/
int ad9208_reset(ad9208_handle_t *h, uint8_t hw_reset);

/***************************************************************************//**
 * @brief This function configures the operation of the external power down pin
 * on the AD9208 device, allowing it to control the power down or standby
 * modes based on the pin's signal. It can also disable the pin's effect
 * on power status. This function should be used when you need to manage
 * the power state of the AD9208 via an external pin. Ensure the device
 * handle is valid and initialized before calling this function.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null,
 * and the device should be properly initialized before use.
 * @param pin_enable A uint8_t value indicating whether the power down pin is
 * enabled (1) or disabled (0). Values other than 0 or 1 are
 * considered invalid and will result in an error.
 * @param pin_mode An ad9208_pdn_mode_t value specifying the power down mode
 * controlled by the pin if enabled. Valid values are
 * AD9208_STANDBY and AD9208_POWERDOWN. Any other value is
 * invalid and will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if parameters are invalid.
 ******************************************************************************/
int ad9208_set_pdn_pin_mode(ad9208_handle_t *h,
			    uint8_t pin_enable, ad9208_pdn_mode_t pin_mode);

/***************************************************************************//**
 * @brief This function configures the input clock settings for the AD9208
 * device by setting the input clock frequency and the clock divider. It
 * must be called with a valid device handle after the device has been
 * initialized. The input clock frequency must be within the range of 2.5
 * GHz to 6 GHz, and the resulting ADC clock frequency, calculated as the
 * input clock frequency divided by the divider, must be between 2.5 GHz
 * and 3.5 GHz. The divider can only be set to 1, 2, or 4. If any of
 * these conditions are not met, the function will return an error. This
 * function updates the device's internal state to reflect the new ADC
 * clock frequency.
 *
 * @param h A pointer to the AD9208 device handle. Must not be null. The handle
 * should be properly initialized before calling this function.
 * @param clk_freq_hz The input clock frequency in hertz. Must be between 2.5
 * GHz and 6 GHz. If outside this range, the function returns
 * an error.
 * @param div The clock divider value. Must be 1, 2, or 4. If an invalid value
 * is provided, the function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code if the handle is invalid or parameters are out of range.
 ******************************************************************************/
int ad9208_set_input_clk_cfg(ad9208_handle_t *h,
			     uint64_t clk_freq_hz, uint8_t div);

/***************************************************************************//**
 * @brief This function is used to control the duty cycle stabilizer (DCS) for
 * the input clock of the AD9208 device. It should be called when you
 * need to enable or disable the DCS feature, which can be beneficial
 * when a stable clock duty cycle is required but not inherently provided
 * by the clock source. The function must be called with a valid device
 * handle and a boolean-like enable parameter. It is important to ensure
 * that the device handle is properly initialized before calling this
 * function. The function will return an error if the handle is invalid
 * or if the enable parameter is out of range.
 *
 * @param h A pointer to an ad9208_handle_t structure representing the device
 * handle. Must not be null. The caller retains ownership.
 * @param en A uint8_t value representing the enable setting for the DCS. Valid
 * values are 0 (disable) and 1 (enable). Values outside this range
 * will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int ad9208_set_input_clk_duty_cycle_stabilizer(ad9208_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief Use this function to obtain the current ADC clock frequency from an
 * initialized AD9208 device handle. It is essential to ensure that the
 * device handle is valid and initialized before calling this function.
 * The function will store the ADC clock frequency in the provided
 * pointer. This function should be called when you need to verify or
 * utilize the ADC clock frequency for further processing or
 * configuration.
 *
 * @param h A pointer to an initialized AD9208 device handle. Must not be null.
 * If null, the function returns an error code indicating an invalid
 * handle.
 * @param adc_clk_freq_hz A pointer to a uint64_t variable where the ADC clock
 * frequency will be stored. Must not be null. If null,
 * the function returns an error code indicating an
 * invalid parameter.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the adc_clk_freq_hz pointer is null.
 ******************************************************************************/
int ad9208_get_adc_clk_freq(ad9208_handle_t *h, uint64_t *adc_clk_freq_hz);

/***************************************************************************//**
 * @brief This function is used to specify which ADC channels on the AD9208
 * device should be active for subsequent configuration operations. It
 * must be called with a valid device handle and a channel selection
 * value before any channel-specific configuration functions are invoked.
 * The function ensures that the handle is not null and that the channel
 * selection is within the valid range, returning an error if these
 * conditions are not met.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param ch A bitwise representation of the ADC channels to be activated, using
 * values from the ad9208_adc_ch_t enumeration. Valid values include
 * AD9208_ADC_CH_NONE, AD9208_ADC_CH_A, AD9208_ADC_CH_B, and
 * AD9208_ADC_CH_ALL. If an invalid value is provided, the function
 * returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null,
 * API_ERROR_INVALID_PARAM if the channel selection is invalid, or other
 * error codes from the underlying register write operation.
 ******************************************************************************/
int ad9208_adc_set_channel_select(ad9208_handle_t *h, uint8_t ch);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC channel selection from
 * the AD9208 device. It should be called when you need to verify which
 * ADC channels are currently active. Ensure that the device handle is
 * properly initialized before calling this function. The function will
 * return an error if the handle or the channel pointer is null, or if
 * there is an issue with SPI communication.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The handle should be properly initialized before use.
 * @param ch A pointer to a uint8_t variable where the current ADC channel
 * selection will be stored. Must not be null. The function will write
 * the channel selection to this location.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter, or SPI communication failure.
 ******************************************************************************/
int ad9208_adc_get_channel_select(ad9208_handle_t *h, uint8_t *ch);

/***************************************************************************//**
 * @brief This function configures the power-down mode of the AD9208 ADC
 * channels, allowing the user to select between normal operation,
 * standby, or full power-down modes. It should be used in conjunction
 * with the channel selection API to target specific channels. The
 * function must be called with a valid device handle and a power-down
 * mode within the defined range. It returns an error if the handle is
 * null or if the mode is invalid.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param mode Desired power-down mode for the ADC channels. Must be a valid
 * value from the ad9208_pdn_mode_t enumeration. If the mode is
 * greater than AD9208_POWERDOWN, the function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the mode is invalid.
 ******************************************************************************/
int ad9208_adc_set_ch_pdn_mode(ad9208_handle_t *h, ad9208_pdn_mode_t mode);

/***************************************************************************//**
 * @brief This function adjusts the phase of the ADC clock for the AD9208 device
 * by a specified number of half-cycle delays. It should be used when
 * precise clock phase alignment is required for the ADC operation. The
 * function must be called with a valid device handle and a phase
 * adjustment value within the allowed range. It is important to ensure
 * that the device handle is properly initialized before calling this
 * function. The function will return an error if the handle is invalid
 * or if the phase adjustment value is out of range.
 *
 * @param h A pointer to the AD9208 device reference handle. This handle must be
 * valid and properly initialized before calling the function. The
 * function will return an error if this parameter is null.
 * @param phase_adj An 8-bit unsigned integer representing the number of half-
 * cycle delays to apply to the ADC clock. Valid values range
 * from 0 to 15, where 0 means no delay and 15 means a 7.5
 * input clock cycle delay. If the value is out of this range,
 * the function will return an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is invalid, or
 * API_ERROR_INVALID_PARAM if the phase adjustment value is out of
 * range.
 ******************************************************************************/
int ad9208_adc_set_clk_phase(ad9208_handle_t *h, uint8_t phase_adj);

/**
 * \brief Set AD9208 ADC Input Configuration
 *
 * The AD9208 analog inputs are internally biased to the comon mode voltage.
 * For AC coupled applications an external Vref may be used.
 * For DC coupled applications the internal common mode buffer may be disabled.
 * And its recommended to export the internal common mode voltage reference
 * to the input buffer via the Vref pin. Refer to the Datasheet for more details.
 * Finally the internal voltage reference may be adjusted to modify the full-scale
 * voltage.
 * Refer to the Datasheet for full details.
 *
 * \param h                  Pointer to the AD9208 device reference handle.
 * \param analog_input_mode  ADC Analog Input mode. Enumerated by signal_coupling_t
 *                           COUPLING_AC - AC Coupled Mode
 *                           COUPLING_DC-  DC Coupled Mode
 * \param ext_vref           External Voltage Reference mode of AC Coupled Applications.
 *                           This parameter shall be ignored in DC Coupled Applications
 *                           0 - AC Coupled Mode
 *                           1-  DC Coupled Mode
 * \param full_scale_range   Full Scale Voltage settings.
 *                           Valid options are defined by ad9208_adc_scale_range_t
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR   Invalid reference handle
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_SPI_XFER SPI XFER Function Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */

/***************************************************************************//**
 * @brief This function configures the analog input mode, external voltage
 * reference, and full-scale range for the AD9208 ADC. It should be
 * called after initializing the device and before starting any data
 * acquisition. The function requires a valid device handle and checks
 * for valid parameter combinations, such as ensuring that DC coupling
 * does not use an external voltage reference. Invalid parameters or a
 * null handle will result in an error code.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param analog_input_mode Specifies the ADC analog input mode, either AC or DC
 * coupling, as defined by the signal_coupling_t
 * enumeration. Must not be COUPLING_UNKNOWN.
 * @param ext_vref Specifies whether to use an external voltage reference in AC
 * coupled mode. Valid values are 0 (internal) or 1 (external).
 * Ignored in DC coupled mode.
 * @param full_scale_range Specifies the full-scale voltage range for the ADC,
 * as defined by the ad9208_adc_scale_range_t
 * enumeration.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle, invalid
 * parameters).
 ******************************************************************************/
int ad9208_adc_set_input_cfg(ad9208_handle_t *h,
			     signal_coupling_t analog_input_mode,
			     uint8_t ext_vref,
			     ad9208_adc_scale_range_t full_scale_range);

/***************************************************************************//**
 * @brief This function configures the input buffer currents for the AD9208 ADC,
 * optimizing the buffer settings for the common mode reference and
 * setting the optimal VCM buffer. It should be called after initializing
 * the AD9208 device and before starting any ADC operations that depend
 * on specific buffer configurations. The function requires valid buffer
 * current settings for the N-input, P-input, and VCM buffers, and it
 * will return an error if any of these settings are invalid or if the
 * device handle is null.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param buff_curr_n N-input buffer current setting. Must be a valid value from
 * the ad9208_adc_buff_curr_t enumeration. Invalid values
 * will result in an error.
 * @param buff_curr_p P-input buffer current setting. Must be a valid value from
 * the ad9208_adc_buff_curr_t enumeration. Invalid values
 * will result in an error.
 * @param vcm_buff VCM buffer current setting. Must be a valid value from the
 * ad9208_adc_buff_curr_t enumeration. Invalid values will
 * result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle or
 * parameter).
 ******************************************************************************/
int ad9208_adc_set_input_buffer_cfg(ad9208_handle_t *h,
				    ad9208_adc_buff_curr_t buff_curr_n,
				    ad9208_adc_buff_curr_t buff_curr_p,
				    ad9208_adc_buff_curr_t vcm_buff);

/***************************************************************************//**
 * @brief This function allows the user to enable or disable the digital filter
 * that removes DC offset from the ADC output of the AD9208 device. It
 * should be called when the user needs to adjust the DC offset
 * calibration setting. The function requires a valid device handle and a
 * boolean-like enable parameter. It must be called after the device has
 * been properly initialized. If the handle is null or the enable
 * parameter is not 0 or 1, the function will return an error.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param en A uint8_t value indicating whether to enable (1) or disable (0) the
 * DC offset calibration filter. Values other than 0 or 1 are
 * considered invalid and will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int ad9208_adc_set_dc_offset_filt_en(ad9208_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief This function sets the operational mode of the AD9208 ADC by
 * specifying the number of carrier frequency channels to be used. It
 * should be called after initializing the device and before starting any
 * data acquisition to ensure the ADC is configured correctly for the
 * intended application. The function requires a valid device handle and
 * a channel mode value that specifies the number of channels. Invalid
 * handle pointers or channel mode values outside the allowed range will
 * result in an error.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param fc_ch Specifies the number of carrier frequency channels. Valid values
 * are 0, 1, 2, and 4. The value 3 is not allowed and will result
 * in an error. Values greater than AD9208_NOF_FC_MAX are also
 * invalid.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null,
 * API_ERROR_INVALID_PARAM for invalid channel mode values, or other
 * error codes for SPI access failures.
 ******************************************************************************/
int ad9208_adc_set_fc_ch_mode(ad9208_handle_t *h, uint8_t fc_ch);

/***************************************************************************//**
 * @brief This function configures the decimation mode of the AD9208 ADC, which
 * is essential for adjusting the ADC's sampling rate relative to the
 * desired output sample rate. It should be used when you need to change
 * the decimation ratio to match specific application requirements. The
 * function must be called with a valid device handle and a decimation
 * ratio within the supported range. It is important to ensure that the
 * device handle is properly initialized before calling this function. If
 * the handle is null or the decimation configuration is invalid, the
 * function will return an error.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership and is responsible for ensuring it
 * points to a valid, initialized device.
 * @param dcm The desired ADC decimation ratio. Valid range is 1 to 48. If the
 * value is outside this range, the function will return an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code if the handle is invalid or the decimation configuration fails.
 ******************************************************************************/
int ad9208_adc_set_dcm_mode(ad9208_handle_t *h, uint8_t dcm);

/***************************************************************************//**
 * @brief This function configures the data format for both input and output of
 * the AD9208 ADC, allowing selection between real and complex data
 * formats. It should be used when the data format needs to be specified
 * for the ADC's operation. The function must be called with a valid
 * device handle, and the data format parameters must be within the
 * defined enumeration range. If the handle is null or the data format
 * parameters are invalid, the function will return an error code.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param ip_data_frmt The desired input data format, either real or complex,
 * specified by the ad9208_adc_data_frmt_t enumeration.
 * Values outside this enumeration will result in an error.
 * @param op_data_frmt The desired output data format, either real or complex,
 * specified by the ad9208_adc_data_frmt_t enumeration.
 * Values outside this enumeration will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle or
 * parameter).
 ******************************************************************************/
int ad9208_adc_set_data_format(ad9208_handle_t *h,
			       ad9208_adc_data_frmt_t ip_data_frmt,
			       ad9208_adc_data_frmt_t op_data_frmt);

/***************************************************************************//**
 * @brief This function allows the user to enable or disable the synchronized
 * update mode for the AD9208 ADC. Synchronized update mode affects how
 * DDC NCO parameters and programmable filter registers are updated,
 * allowing for either instantaneous or synchronized updates. This
 * function should be used when there is a need to control the timing of
 * updates to these parameters, which can be triggered by a SPI bit or a
 * GPIO pin. It is important to ensure that the device handle is valid
 * before calling this function.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null,
 * as a null handle will result in an API_ERROR_INVALID_HANDLE_PTR
 * error.
 * @param en A uint8_t value indicating whether to enable (1) or disable (0) the
 * synchronized update mode. Values other than 0 or 1 will result in
 * an API_ERROR_INVALID_PARAM error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int ad9208_adc_set_sync_update_mode_enable(ad9208_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief This function configures the input data sources for the I and Q data
 * channels of a specified Digital Down Converter (DDC) channel in the
 * AD9208 device. It is essential to ensure that the device handle is
 * valid and that the DDC channel index is within the permissible range
 * before calling this function. The function should be used when there
 * is a need to change the data source for the I and Q channels, which
 * can be either ADC Channel A or B. Proper error handling should be
 * implemented to manage invalid parameters or handle pointers.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param ddc_ch The DDC channel index to configure. Valid range is 0 to 3. If
 * out of range, the function returns an error.
 * @param i_data_src Specifies the ADC channel source for the I data channel.
 * Must be either AD9208_ADC_CH_A or AD9208_ADC_CH_B. Invalid
 * values result in an error.
 * @param q_data_src Specifies the ADC channel source for the Q data channel.
 * Must be either AD9208_ADC_CH_A or AD9208_ADC_CH_B. Invalid
 * values result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure, such as API_ERROR_INVALID_HANDLE_PTR or
 * API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_adc_set_ddc_ip_mux(ad9208_handle_t *h,
			      uint8_t ddc_ch, ad9208_adc_ch_t i_data_src,
			      ad9208_adc_ch_t q_data_src);

/***************************************************************************//**
 * @brief This function configures the decimation filter for a specified DDC
 * channel on the AD9208 device. It should be used when you need to
 * adjust the decimation rate for a particular DDC channel to match the
 * desired output sample rate. Ensure that the device handle is valid and
 * that the DDC channel and decimation values are within their respective
 * valid ranges before calling this function. The function will return an
 * error if the handle is null or if any parameters are out of range.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param ddc_ch The DDC channel to configure. Valid range is 0 to 3.
 * @param dcm The decimation rate to set for the DDC channel. Valid range is 1
 * to 48.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle or
 * parameter).
 ******************************************************************************/
int ad9208_adc_set_ddc_dcm(ad9208_handle_t *h, uint8_t ddc_ch, uint8_t dcm);

/***************************************************************************//**
 * @brief Use this function to configure the Numerically Controlled Oscillator
 * (NCO) mode for a specific Digital Down Converter (DDC) channel on the
 * AD9208 device. This function should be called after initializing the
 * device and selecting the appropriate DDC channel. It is important to
 * ensure that the handle is valid and that the DDC channel and mode
 * parameters are within their valid ranges. Invalid parameters will
 * result in an error.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param ddc_ch The DDC channel to configure. Valid range is 0 to 3. Values
 * outside this range will result in an API_ERROR_INVALID_PARAM
 * error.
 * @param mode The desired NCO mode for the specified DDC channel. Must be a
 * valid value from the ad9208_adc_nco_mode_t enumeration, excluding
 * AD9208_ADC_NCO_INVLD. Invalid values will result in an
 * API_ERROR_INVALID_PARAM error.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameters.
 ******************************************************************************/
int ad9208_adc_set_ddc_nco_mode(ad9208_handle_t *h,
				uint8_t ddc_ch, ad9208_adc_nco_mode_t mode);

/***************************************************************************//**
 * @brief This function sets the frequency translation word (FTW) and optional
 * modulation words (MAW and MBW) for a specified DDC channel in the
 * AD9208 device. It should be used when configuring the NCO for
 * frequency translation in digital down-conversion applications. The
 * function requires a valid device handle and a DDC channel index within
 * the supported range. If modulation words are non-zero, they are also
 * configured. The function must be called after initializing the device
 * and selecting the appropriate DDC channel. It returns an error code if
 * the handle is invalid or if the DDC channel index is out of range.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param ddc_ch Index of the DDC channel to configure. Valid range is 0 to 3.
 * An invalid index results in an error.
 * @param ftw 48-bit frequency tuning word for the NCO. Represents the desired
 * frequency translation.
 * @param mod_a 48-bit modulation word A for the NCO. Used for additional
 * frequency modulation if non-zero.
 * @param mod_b 48-bit modulation word B for the NCO. Used for additional
 * frequency modulation if non-zero.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure.
 ******************************************************************************/
int ad9208_adc_set_ddc_nco_ftw(ad9208_handle_t *h, uint8_t ddc_ch,
			       uint64_t ftw, uint64_t mod_a, uint64_t mod_b);

/***************************************************************************//**
 * @brief This function sets the Numerically Controlled Oscillator (NCO) for a
 * specified Digital Down Converter (DDC) channel in the AD9208 device,
 * using the provided carrier frequency. It must be called with a valid
 * device handle and after the device has been initialized. The function
 * checks if the carrier frequency is within the valid range, which is
 * between -adc_clk_freq_hz/2 and adc_clk_freq_hz/2, and ensures the DDC
 * channel is within the allowed range. If the carrier frequency is a
 * power of two relative to the ADC clock frequency, the function
 * configures the NCO in Integer mode; otherwise, it uses Programmable
 * Modulus mode. The function returns an error code if any parameter is
 * invalid or if the configuration fails.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param ddc_ch The DDC channel to configure. Valid range is 0 to 3.
 * @param carrier_freq_hz The desired carrier frequency in Hz. Must be non-zero
 * and within the range of -adc_clk_freq_hz/2 to
 * adc_clk_freq_hz/2.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure.
 ******************************************************************************/
int ad9208_adc_set_ddc_nco(ad9208_handle_t *h, uint8_t ddc_ch,
			   const int64_t carrier_freq_hz);

/***************************************************************************//**
 * @brief This function configures the phase offset of the Numerically
 * Controlled Oscillator (NCO) for a specified Digital Down-Converter
 * (DDC) channel on the AD9208 device. It is used to adjust the phase of
 * the NCO, which is crucial for applications requiring precise phase
 * alignment. The function must be called with a valid device handle and
 * a DDC channel within the supported range. It is important to ensure
 * that the device handle is properly initialized before calling this
 * function. Invalid parameters will result in an error.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param ddc_ch The DDC channel on which to set the NCO phase offset. Valid
 * range is 0 to 3. If the value is out of range, the function
 * returns an error.
 * @param po The desired 48-bit NCO phase offset. This value is used to set the
 * phase offset for the specified DDC channel.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure, such as API_ERROR_INVALID_HANDLE_PTR or
 * API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_adc_set_ddc_nco_phase(ad9208_handle_t *h, uint8_t ddc_ch,
				 uint64_t po);

/***************************************************************************//**
 * @brief Use this function to configure the gain for a specific Digital Down-
 * Converter (DDC) channel on the AD9208 device. It allows setting the
 * gain to either 0 dB or 6 dB for the selected DDC channel. This
 * function should be called after initializing the AD9208 device and
 * selecting the appropriate DDC channel. Ensure that the handle is valid
 * and the DDC channel index is within the supported range. Invalid
 * parameters will result in an error.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param ddc_ch The DDC channel index to configure. Valid range is 0 to 3.
 * @param gain_db Desired gain setting for the DDC channel. Must be either 0 or
 * 6.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., invalid handle or parameters).
 ******************************************************************************/
int ad9208_adc_set_ddc_gain(ad9208_handle_t *h,
			    uint8_t ddc_ch, uint8_t gain_db);
/***************************************************************************//**
 * @brief This function is used to perform a soft reset of the Digital
 * Downconverter (DDC) NCOs in the AD9208 device, which may be necessary
 * for synchronization purposes. It should be called when a reset of the
 * NCOs is required, such as after configuration changes. The function
 * requires a valid AD9208 device handle and will return an error if the
 * handle is null. It also handles any necessary delays if a delay
 * function is provided in the handle.
 *
 * @param h A pointer to an ad9208_handle_t structure representing the AD9208
 * device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the handle is valid.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g.,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or other error
 * codes for SPI access failures).
 ******************************************************************************/
int ad9208_adc_set_ddc_nco_reset(ad9208_handle_t *h);
/***************************************************************************//**
 * @brief This function configures the JESD interface parameters for the AD9208
 * device based on the provided JESD link parameters. It calculates and
 * sets the appropriate lane rate and updates the configuration registers
 * accordingly. This function should be called after initializing the
 * AD9208 device and setting the input clock configuration. It ensures
 * that the lane rate is within the valid range and returns an error if
 * the parameters are invalid or if the handle is null.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * handle should be properly initialized before calling this function.
 * @param jesd_param The desired JESD link parameters for configuring the JESD
 * interface. This includes parameters like M, N, L, etc.,
 * which must be within the supported ranges of the AD9208.
 * @param lane_rate_kbps Pointer to a uint64_t variable where the calculated
 * lane rate in kbps will be stored. Can be set to NULL if
 * the lane rate is not needed by the caller.
 * @return Returns an integer status code. API_ERROR_OK on success, or an error
 * code indicating the type of failure, such as invalid handle or
 * parameter.
 ******************************************************************************/
int ad9208_jesd_set_if_config(ad9208_handle_t *h,
			      jesd_param_t jesd_param,
			      uint64_t *lane_rate_kbps);

/***************************************************************************//**
 * @brief Use this function to read back the current configuration of the JESD
 * interface parameters for the AD9208 device. This function is useful
 * for verifying the current settings of the JESD interface after
 * configuration or for diagnostic purposes. It requires a valid device
 * handle and a pointer to a `jesd_param_t` structure where the
 * parameters will be stored. Ensure that the device handle is properly
 * initialized before calling this function.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. If
 * null, the function returns an error.
 * @param jesd_param Pointer to a `jesd_param_t` structure where the current
 * JESD interface parameters will be stored. Must not be null.
 * If null, the function returns an error.
 * @return Returns an integer status code. `API_ERROR_OK` indicates success,
 * while other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int ad9208_jesd_get_cfg_param(ad9208_handle_t *h, jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function configures the mapping of physical JESD lanes to logical
 * lanes for the AD9208 device. It is used to route the physical lanes to
 * the desired logical lanes in the JESD datalink layer. The function
 * must be called with a valid device handle and valid lane numbers. It
 * returns an error if the handle is null or if the lane numbers are out
 * of the valid range. This function should be used after initializing
 * the device and before starting data transmission.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param logical_lane The logical lane number to which the physical lane should
 * be mapped. Valid range is 0 to 7. If out of range, the
 * function returns an error.
 * @param physical_lane The physical lane number to be mapped to the logical
 * lane. Valid range is 0 to 7. If out of range, the
 * function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., API_ERROR_INVALID_HANDLE_PTR or
 * API_ERROR_INVALID_PARAM).
 ******************************************************************************/
int ad9208_jesd_set_lane_xbar(ad9208_handle_t *h,
			      uint8_t logical_lane, uint8_t physical_lane);

/***************************************************************************//**
 * @brief This function retrieves the current mapping of physical to logical
 * lanes in the JESD datalink layer of the AD9208 device. It is useful
 * for verifying the lane configuration after it has been set. The
 * function must be called with a valid device handle and a pre-allocated
 * array to store the mapping. Ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. If
 * null, the function returns an API_ERROR_INVALID_HANDLE_PTR error.
 * @param phy_log_map Pointer to an array of 8 uint8_t elements where the lane
 * mapping will be stored. Each element represents a physical
 * lane (0-7), and the value represents the logical lane
 * assigned to that physical lane. Must not be null. If null,
 * the function returns an API_ERROR_INVALID_PARAM error.
 * @return Returns API_ERROR_OK on success, or an error code if the handle is
 * invalid or if there is a parameter error.
 ******************************************************************************/
int ad9208_jesd_get_lane_xbar(ad9208_handle_t *h, uint8_t *phy_log_map);

/***************************************************************************//**
 * @brief This function is used to control the scrambler feature of the JESD
 * link in the AD9208 device. It should be called when configuring the
 * JESD interface to either enable or disable data scrambling, which can
 * help in reducing electromagnetic interference and improving data
 * integrity. The function must be called with a valid device handle, and
 * the scrambler can only be enabled or disabled (binary choice). Ensure
 * that the device handle is properly initialized before calling this
 * function.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership and is responsible for ensuring it points
 * to a valid, initialized device structure.
 * @param en A uint8_t value indicating whether to enable (1) or disable (0) the
 * JESD scrambler. Values other than 0 or 1 are considered invalid and
 * will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int ad9208_jesd_enable_scrambler(ad9208_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief This function is used to control the JESD link state on the AD9208
 * device, enabling or disabling it based on the provided parameter. It
 * should be called when you need to start or stop JESD transmission.
 * Ensure that the device handle is valid before calling this function.
 * The function will return an error if the handle is null or if the
 * enable parameter is not within the valid range.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * The caller retains ownership.
 * @param en A uint8_t value indicating whether to enable (1) or disable (0) the
 * JESD link. Valid values are 0 and 1. If the value is greater than
 * 1, the function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int ad9208_jesd_enable_link(ad9208_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief Use this function to obtain the current status of the SERDES PLL in
 * the AD9208 device. It is essential to ensure that the device handle is
 * valid and initialized before calling this function. The function will
 * populate the provided pointer with the PLL status, which includes lock
 * and lost lock status flags. This function is useful for monitoring the
 * PLL status to ensure proper operation of the JESD interface.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null and
 * should be properly initialized before use. If null, the function
 * returns an API_ERROR_INVALID_HANDLE_PTR error.
 * @param pll_status Pointer to a uint8_t variable where the PLL status will be
 * stored. Must not be null. If null, the function returns an
 * API_ERROR_INVALID_PARAM error.
 * @return Returns API_ERROR_OK on success, or an error code indicating the type
 * of failure, such as API_ERROR_INVALID_HANDLE_PTR or
 * API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_jesd_get_pll_status(ad9208_handle_t *h, uint8_t *pll_status);

/***************************************************************************//**
 * @brief This function configures the JESD synchronization subclass mode for
 * the AD9208 device. It should be used when setting up the JESD
 * interface to ensure the correct subclass is selected for the
 * application. The function must be called with a valid device handle
 * and a subclass value within the supported range. It is important to
 * ensure that the device handle is properly initialized before calling
 * this function. The function will return an error if the handle is null
 * or if the subclass value is invalid.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership and is responsible for ensuring it points
 * to a valid, initialized device structure.
 * @param subclass Desired JESD subclass mode. Valid values are 0 for Subclass 0
 * and 1 for Subclass 1. Values outside this range will result
 * in an API_ERROR_INVALID_PARAM error.
 * @return Returns API_ERROR_OK on success. Returns API_ERROR_INVALID_HANDLE_PTR
 * if the handle is null, or API_ERROR_INVALID_PARAM if the subclass is
 * invalid. Other error codes may be returned if register read or write
 * operations fail.
 ******************************************************************************/
int ad9208_jesd_subclass_set(ad9208_handle_t *h, uint8_t subclass);

/***************************************************************************//**
 * @brief This function configures the SYSREF synchronization mode for the
 * AD9208 JESD interface. It should be used to set the desired SYSREF
 * mode, which can be none, one-shot, or continuous. The function must be
 * called with a valid device handle and a valid mode. It is important to
 * ensure that the handle is initialized and valid before calling this
 * function. The function will return an error if the handle is null or
 * if the mode is invalid.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership and is responsible for ensuring it is a
 * valid, initialized handle.
 * @param mode The desired SYSREF synchronization mode. Must be a valid value of
 * type jesd_sysref_mode_t, and less than SYSREF_MON. Invalid values
 * will result in an error.
 * @param sysref_count Number of initial SYSREF signals to ignore in one-shot
 * mode. Valid range is 0-15. This parameter is only
 * relevant when mode is SYSREF_ONESHOT.
 * @return Returns an integer status code. API_ERROR_OK on success, or an error
 * code indicating the type of failure, such as
 * API_ERROR_INVALID_HANDLE_PTR or API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_jesd_syref_mode_set(ad9208_handle_t *h,
			       jesd_sysref_mode_t mode, uint8_t sysref_count);

/***************************************************************************//**
 * @brief This function configures the SYSREF signal capture settings for the
 * AD9208 JESD interface, allowing the user to specify the edge
 * transitions and skew windows for SYSREF signal capture. It is
 * essential to ensure that the handle is valid and that the parameters
 * are within their specified ranges before calling this function. This
 * function should be used when setting up the JESD interface to ensure
 * proper synchronization and signal capture.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null,
 * as a null handle will result in an API_ERROR_INVALID_HANDLE_PTR
 * error.
 * @param sysref_edge_sel Specifies the transition on which the SYSREF signal is
 * valid. Valid values are 0 for LOW to HIGH transition
 * and 1 for HIGH to LOW transition. Values outside this
 * range will result in an API_ERROR_INVALID_PARAM error.
 * @param clk_edge_sel Specifies the clock edge for SYSREF capture. Valid values
 * are 0 for rising clock edge and 1 for falling clock edge.
 * Values outside this range will result in an
 * API_ERROR_INVALID_PARAM error.
 * @param neg_window_skew Specifies the skew in clock cycles for the negative
 * window during which the SYSREF signal is ignored.
 * Valid range is 0 to 3. Values outside this range will
 * result in an API_ERROR_INVALID_PARAM error.
 * @param pos_window_skew Specifies the skew in clock cycles for the positive
 * window during which the SYSREF signal is ignored.
 * Valid range is 0 to 3. Values outside this range will
 * result in an API_ERROR_INVALID_PARAM error.
 * @return Returns API_ERROR_OK on success, or an error code indicating the type
 * of failure, such as API_ERROR_INVALID_HANDLE_PTR or
 * API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_jesd_syref_config_set(ad9208_handle_t *h,
				 uint8_t sysref_edge_sel, uint8_t clk_edge_sel,
				 uint8_t neg_window_skew,
				 uint8_t pos_window_skew);

/***************************************************************************//**
 * @brief This function is used to obtain the current SYSREF status, which
 * includes hold, setup, and phase status, for monitoring the JESD
 * interface of the AD9208 device. It should be called when you need to
 * check the synchronization status of the SYSREF signal. Ensure that the
 * AD9208 device has been properly initialized before calling this
 * function. The function requires valid pointers for all output
 * parameters and will return an error if any of the pointers are null.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * handle should be properly initialized before use.
 * @param hold_status Pointer to a uint8_t variable where the SYSREF hold status
 * will be stored. Must not be null.
 * @param setup_status Pointer to a uint8_t variable where the SYSREF setup
 * status will be stored. Must not be null.
 * @param phase_status Pointer to a uint8_t variable where the SYSREF phase
 * status will be stored. Must not be null.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int ad9208_jesd_syref_status_get(ad9208_handle_t *h,
				 uint8_t *hold_status, uint8_t *setup_status,
				 uint8_t *phase_status);

/***************************************************************************//**
 * @brief This function configures the SYSREF timestamp settings for the AD9208
 * JESD interface. It should be used when you need to enable or disable
 * timestamping of SYSREF signals, control the bit used for timestamping,
 * and set the delay for the timestamp in sample clock cycles. Ensure
 * that the handle is valid and initialized before calling this function.
 * Invalid parameters will result in an error.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null.
 * @param timestamp_en Enable or disable timestamp mode. Valid values are 0
 * (disable) and 1 (enable).
 * @param control_bit Control bit for SYSREF timestamp control. Valid values are
 * 0, 1, or 2.
 * @param delay Delay of SYSREF timestamp in sample clock cycles. Must be within
 * the valid range defined by the device.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., API_ERROR_INVALID_HANDLE_PTR,
 * API_ERROR_INVALID_PARAM).
 ******************************************************************************/
int ad9208_jesd_sysref_timestamp_set(ad9208_handle_t *h,
				     uint8_t timestamp_en, uint8_t control_bit,
				     uint8_t delay);
/***************************************************************************//**
 * @brief This function adjusts the Local Multi-Frame Clock (LMFC) phase offset
 * in frame clock cycles for the JESD interface of the AD9208 device. It
 * should be used when configuring the JESD synchronization settings to
 * ensure proper alignment of the data frames. The function must be
 * called with a valid device handle and a valid offset value. It is
 * important to ensure that the device handle is initialized and that the
 * offset value is within the permissible range for the specific JESD
 * configuration.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * handle should be properly initialized before calling this function.
 * @param offset The desired LMFC phase offset in frame clock cycles. The value
 * must be within the valid range defined by the device's JESD
 * configuration. If the offset is out of range, the function
 * returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the offset is invalid.
 ******************************************************************************/
int ad9208_jesd_syref_lmfc_offset_set(ad9208_handle_t *h, uint8_t offset);

/***************************************************************************//**
 * @brief This function retrieves the overrange status of the AD9208 ADC,
 * indicating whether the input signal has exceeded the ADC's range, and
 * then clears the overrange detection signal. It should be called when
 * you need to check for overrange conditions in the ADC input. Ensure
 * that the AD9208 device has been properly initialized before calling
 * this function. The function requires valid pointers for both the
 * device handle and the status output parameter.
 *
 * @param h A pointer to the AD9208 device reference handle. Must not be null.
 * If null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param status A pointer to a uint8_t variable where the overrange status will
 * be stored. Must not be null. If null, the function returns
 * API_ERROR_INVALID_PARAM.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int ad9208_adc_get_overange_status(ad9208_handle_t *h, uint8_t *status);

/**
 * \brief  Configure The Fast Detect Overange Signal Thresholds
 *
 * Configure the parameters, the upper and lower threshold levels and the dwell time,
 * that govern the triggering of the fast overange detection signal.
 *
 * \param h             Pointer to the AD9208 device reference handle.
 * \param upper_dbfs    A 13 bit value representing the Upper magnitude threshold level
 *                      in DBFS.
 *
 * \param lower_dbfs    A 13 bit value representing the lower magnitude threshold level
 *                      in DBFS.
 *
 * \param dwell_time    A 13 bit value representing the Upper magnitude threshold level
 *                      in DBFS.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_SPI_XFER SPI XFER Function Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */

/***************************************************************************//**
 * @brief This function configures the upper and lower magnitude threshold
 * levels and the dwell time for the fast overrange detection signal in
 * the AD9208 ADC. It should be used when you need to set specific
 * thresholds for detecting overrange conditions in the ADC input. Ensure
 * that the handle is valid and that the threshold and dwell time values
 * are within their respective maximum limits before calling this
 * function.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null, as
 * a null handle will result in an API_ERROR_INVALID_HANDLE_PTR error.
 * @param upper_dbfs A 13-bit value representing the upper magnitude threshold
 * level in dBFS. Must not exceed FD_THRESHOLD_MAG_DBFS_MAX,
 * or an API_ERROR_INVALID_PARAM error will be returned.
 * @param lower_dbfs A 13-bit value representing the lower magnitude threshold
 * level in dBFS. Must not exceed FD_THRESHOLD_MAG_DBFS_MAX,
 * or an API_ERROR_INVALID_PARAM error will be returned.
 * @param dwell_time A 13-bit value representing the dwell time in clock cycles.
 * Must not exceed FD_DWELL_CLK_CYCLES_MAX, or an
 * API_ERROR_INVALID_PARAM error will be returned.
 * @return Returns API_ERROR_OK on success, or an error code on failure, such as
 * API_ERROR_INVALID_HANDLE_PTR or API_ERROR_INVALID_PARAM.
 ******************************************************************************/
int ad9208_adc_set_fd_thresholds(ad9208_handle_t *h,
				 uint16_t upper_dbfs, uint16_t lower_dbfs,
				 uint16_t dwell_time);
/***************************************************************************//**
 * @brief Use this function to control the state of the AD9208 ADC's diode-based
 * temperature sensors, which can output voltages to the VREF pin. This
 * function should be called when you need to enable or disable the
 * temperature sensor functionality, especially in applications where
 * temperature monitoring is required. Ensure that the VREF pin is not
 * being used for ADC reference voltage in DC coupled applications or as
 * an external VREF in AC coupled modes, as this would conflict with the
 * temperature sensor usage. The function must be called with a valid
 * device handle and a proper enable/disable flag.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. If
 * null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param en Enable control for the temperature sensor diodes. Valid values are
 * 0 (disable) and 1 (enable). If the value is greater than 1, the
 * function returns API_ERROR_INVALID_PARAM.
 * @return Returns API_ERROR_OK on success, or an error code indicating the type
 * of failure.
 ******************************************************************************/
int ad9208_adc_temp_sensor_set_enable(ad9208_handle_t *h, uint8_t en);
/***************************************************************************//**
 * @brief Use this function to obtain the product type, identification, and
 * revision data from an AD9208 device. It is essential to ensure that
 * the device handle is valid and properly initialized before calling
 * this function. The function will populate the provided `chip_id`
 * structure with the relevant identification data. This function should
 * be called when you need to verify the identity or version of the
 * AD9208 device in use. Handle any error codes returned to ensure
 * successful data retrieval.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null and
 * should point to a valid, initialized device handle.
 * @param chip_id Pointer to a variable of type `adi_chip_id_t` where the chip
 * identification data will be stored. Must not be null.
 * @return Returns an integer status code. `API_ERROR_OK` indicates success,
 * while other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int ad9208_get_chip_id(ad9208_handle_t *h, adi_chip_id_t *chip_id);

/***************************************************************************//**
 * @brief This function writes an 8-bit data value to a specified SPI register
 * address on the AD9208 device. It requires a valid device handle and a
 * properly configured SPI transfer function. The function should be
 * called when a register write operation is needed, and it returns an
 * error code if the operation fails due to an invalid handle, transfer
 * function, or SPI transfer error.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null and
 * should have a valid SPI transfer function configured.
 * @param address The 16-bit SPI address of the AD9208 device register to which
 * the data will be written. Valid address range depends on the
 * device specification.
 * @param data The 8-bit data value to be written to the specified SPI register
 * address.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle, invalid
 * transfer function, or SPI transfer error).
 ******************************************************************************/
int ad9208_register_write(ad9208_handle_t *h,
			  const uint16_t address, const uint8_t data);

/***************************************************************************//**
 * @brief This function reads an 8-bit value from a specified SPI register
 * address on the AD9208 device. It requires a valid device handle and a
 * pointer to store the read data. The function should be called after
 * the device has been properly initialized. It handles errors related to
 * invalid handles and SPI transfer failures.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null and
 * should be properly initialized before use.
 * @param address The 16-bit SPI address from which the data will be read. The
 * address should be within the valid range for the device's
 * register map.
 * @param data Pointer to an 8-bit variable where the read data will be stored.
 * Must not be null.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is invalid,
 * API_ERROR_INVALID_XFER_PTR if the transfer function is invalid, and
 * API_ERROR_SPI_XFER if the SPI transfer fails.
 ******************************************************************************/
int ad9208_register_read(ad9208_handle_t *h,
			 const uint16_t address, uint8_t *data);
/***************************************************************************//**
 * @brief Use this function to obtain the major, minor, and release candidate
 * (RC) revision numbers of the AD9208 API. This function is useful for
 * verifying the version of the API being used. It requires valid
 * pointers for each of the revision number outputs and will return an
 * error if any of these pointers are null.
 *
 * @param h Pointer to the AD9208 device reference handle. The handle must be
 * valid and properly initialized before calling this function.
 * @param rev_major Pointer to a uint8_t variable where the major revision
 * number will be stored. Must not be null.
 * @param rev_minor Pointer to a uint8_t variable where the minor revision
 * number will be stored. Must not be null.
 * @param rev_rc Pointer to a uint8_t variable where the RC revision number will
 * be stored. Must not be null.
 * @return Returns an integer indicating success or failure. Returns
 * API_ERROR_OK on success, or API_ERROR_INVALID_PARAM if any of the
 * revision number pointers are null.
 ******************************************************************************/
int ad9208_get_revision(ad9208_handle_t *h, uint8_t *rev_major,
			uint8_t *rev_minor, uint8_t *rev_rc);

/***************************************************************************//**
 * @brief This function configures the full-scale input voltage range for the
 * AD9208 ADC. It should be called after initializing the device with a
 * valid handle. The function requires a valid scale range enumeration
 * value to set the desired full-scale range. If the handle is null or
 * the scale range is invalid, the function returns an error. This
 * function is essential for adjusting the ADC's input sensitivity to
 * match the expected signal levels.
 *
 * @param h Pointer to the AD9208 device reference handle. Must not be null. The
 * caller retains ownership.
 * @param full_scale_range Enumeration value specifying the desired full-scale
 * input voltage range. Valid values are defined by
 * ad9208_adc_scale_range_t. If an invalid value is
 * provided, the function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the scale range is invalid.
 ******************************************************************************/
int ad9208_adc_set_input_scale(ad9208_handle_t *h,
			       ad9208_adc_scale_range_t full_scale_range);

/***************************************************************************//**
 * @brief Use this function to obtain the current decimation rate configured in
 * the AD9208 device. It is essential to ensure that the device handle is
 * valid before calling this function. The function reads the decimation
 * configuration from the device and maps it to a known decimation rate.
 * If the handle is invalid or the decimation configuration is not
 * recognized, the function will return an error code.
 *
 * @param h A pointer to an ad9208_handle_t structure representing the device
 * handle. Must not be null. If null, the function returns
 * API_ERROR_INVALID_HANDLE_PTR.
 * @param dcm A pointer to a uint8_t variable where the decimation rate will be
 * stored. The caller must ensure this pointer is valid and not null.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the decimation configuration is not
 * recognized.
 ******************************************************************************/
int ad9208_get_decimation(ad9208_handle_t *h, uint8_t *dcm);

#endif /* !__AD9208API_H__ */

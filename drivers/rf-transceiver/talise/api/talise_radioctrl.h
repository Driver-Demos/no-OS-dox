/**
 * \file talise_radioctrl.h
 * \brief Contains Talise related function prototypes for talise_radioctrl.c
 *
 *  Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_RADIOCTRL_H_
#define TALISE_RADIOCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_radioctrl_types.h"
#include "talise_tx_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to load a 4096-element byte array into the
 * stream processor data memory of a Talise device. The byte array should
 * be obtained from a binary stream processor file provided by Analog
 * Devices. This function must be called after the device has been
 * initialized and the PLL lock status has been verified. It is essential
 * for powering up and down various signal chains correctly. The function
 * handles several recovery actions in case of errors, such as resetting
 * the SPI or logging issues.
 *
 * @param device A pointer to the taliseDevice_t structure containing the device
 * settings. This parameter must not be null, and the caller
 * retains ownership.
 * @param binary A pointer to a 4096-element byte array containing the parsed
 * stream processor binary file. This parameter must not be null,
 * and the caller retains ownership.
 * @return Returns a uint32_t value indicating the recovery action required:
 * TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI,
 * or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_loadStreamFromBinary(taliseDevice_t *device, uint8_t *binary);

/***************************************************************************//**
 * @brief This function configures the Talise ARM to use specific GPIO pins for
 * TDD control signals, allowing the baseband processor (BBP) to drive
 * these signals via GPIO. It should be called after loading the ARM
 * processor with the binary using TALISE_loadArmFromBinary() and before
 * entering the radioOn state during initialization. The function can be
 * called again to change GPIO assignments while the ARM is in the
 * radioOff state. It also sets the GPIO pin direction for any enabled
 * pins. Ensure that the ARM binary is loaded and the device is in the
 * appropriate state before calling this function.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param armGpio Pointer to a structure that specifies which GPIO pins and
 * settings to use for each ARM GPIO signal. Must not be null.
 * Invalid or conflicting configurations will result in error
 * handling.
 * @return Returns a uint32_t indicating the result of the operation, with
 * specific values indicating success or various error conditions.
 ******************************************************************************/
uint32_t TALISE_setArmGpioPins(taliseDevice_t *device,
			       taliseArmGpioConfig_t *armGpio);


/***************************************************************************//**
 * @brief This function configures the radio control pin mode for the Talise
 * device, allowing the selection of GPIO pins for controlling Tx, Rx,
 * and ORx signaling. It should be called after the device has been fully
 * initialized and the ARM and GPIO have been configured. The function is
 * intended for use in the radio off state. It validates the provided pin
 * configuration masks and returns an error if they are invalid. The
 * function writes the configuration to the device, and any errors during
 * this process are handled and returned as part of the function's return
 * value.
 *
 * @param device Pointer to the Talise data structure containing device
 * settings. Must not be null.
 * @param pinOptionsMask A bitmask comprised of OR'd taliseRadioCtlCfg1_t
 * enumerated types for radio control configuration
 * register 1. Valid values are combinations of
 * TAL_TXRX_PIN_MODE, TAL_ORX_PIN_MODE,
 * TAL_ORX_USES_RX_PINS, TAL_ORX_SEL,
 * TAL_ORX_SINGLE_CHANNEL, and TAL_ORX_ENAB_SEL_PIN.
 * Invalid bitmask settings will result in an error.
 * @param orxEnGpioPinSel A taliseRadioCtlCfg2_t enumerated type selecting one
 * of three possible GPIO pin pair combinations for ORx
 * enable. Valid values are TAL_ORX1ORX2_PAIR_01_SEL,
 * TAL_ORX1ORX2_PAIR_45_SEL, TAL_ORX1ORX2_PAIR_89_SEL,
 * and TAL_ORX1ORX2_PAIR_NONE_SEL. Invalid values will
 * result in an error.
 * @return Returns a uint32_t indicating the result of the operation, with
 * possible values including TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, and TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_setRadioCtlPinMode(taliseDevice_t *device,
				   uint8_t pinOptionsMask, taliseRadioCtlCfg2_t orxEnGpioPinSel);

/***************************************************************************//**
 * @brief This function retrieves the current configuration of the radio control
 * pin mode for the Talise device, including the Tx, Rx, and ORx control
 * bit mask and the GPIO pin selection for ORx enable. It can be called
 * at any time after the device has been fully initialized and the ARM
 * and GPIO have been configured, regardless of whether the radio is in
 * the on or off state. The function requires valid pointers for the
 * output parameters and will return an error if any of these pointers
 * are null.
 *
 * @param device Pointer to the Talise device structure containing settings.
 * Must not be null.
 * @param pinOptionsMask Pointer to a uint8_t where the function will store the
 * bitmask of radio control configuration options. Must
 * not be null.
 * @param orxEnGpioPinSel Pointer to a taliseRadioCtlCfg2_t where the function
 * will store the GPIO pin selection for ORx enable. Must
 * not be null.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values indicating success or various error conditions.
 ******************************************************************************/
uint32_t TALISE_getRadioCtlPinMode(taliseDevice_t *device,
				   uint8_t *pinOptionsMask, taliseRadioCtlCfg2_t *orxEnGpioPinSel);


/***************************************************************************//**
 * @brief This function sets the Observation Receiver (ORx) Local Oscillator
 * (LO) source configuration, allowing the user to choose between using
 * the RFPLL or AuxPLL for the ORx path. It is useful for applications
 * where the AuxLO is needed to offset the ORx receive data from the Tx
 * data, potentially improving calibration performance. The function
 * should be called after loading the ARM and before running
 * initialization calibrations. It can be used in the ARM READY or Radio
 * off state. The function also allows for real-time control of the ORx
 * LO source via a GPIO pin, if configured. Note that changes to the LO
 * source take effect only when the ORx channel is powered up.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param orxLoCfg Pointer to a structure containing the desired ORx LO
 * configuration settings. Must not be null. The gpioSelect
 * field must be either TAL_GPIO_INVALID or a valid GPIO pin
 * (TAL_GPIO_00 to TAL_GPIO_15).
 * @return Returns a uint32_t value indicating the result of the operation, with
 * possible values including TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and other error codes
 * for specific failures.
 ******************************************************************************/
uint32_t TALISE_setOrxLoCfg(taliseDevice_t *device,
			    const taliseOrxLoCfg_t *orxLoCfg);

/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of the
 * Observation Receiver (ORx) Local Oscillator (LO) settings, which were
 * previously set using the TALISE_setOrxLoCfg() function. This function
 * can be called anytime after the ORx LO configuration has been set, and
 * it is valid to call it in both the ARM READY (before initialization
 * calibrations) and Radio Off states. Ensure that the device is in a
 * valid state before calling this function to avoid errors.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param orxLoCfg Pointer to a taliseOrxLoCfg_t structure where the current ORx
 * LO configuration will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or error codes indicating specific issues encountered during
 * execution.
 ******************************************************************************/
uint32_t TALISE_getOrxLoCfg(taliseDevice_t *device, taliseOrxLoCfg_t *orxLoCfg);

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to power up the enabled Rx and Tx signal chains
 * and start ARM tracking calibrations by transitioning the radio state
 * to Radio ON. It should be called after the device has been fully
 * initialized, including PLL configuration and lock verification, Multi-
 * Chip Sync (MCS), and JESD204B link setup. The function ensures that
 * the device is ready for operation by checking command completion and
 * handling any errors that may occur during the process.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null, and the device should be fully
 * initialized before calling this function.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success, or
 * various error codes indicating specific recovery actions required.
 ******************************************************************************/
uint32_t TALISE_radioOn(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function transitions the radio state from Radio On to Radio Off,
 * halting ARM tracking calibrations and ignoring GPIO control pins for
 * TxEnable/RxEnable. It ensures that the receive and transmit chains
 * remain powered down until the radio is turned back on using the
 * appropriate function. It should be called after the radio has been
 * turned on previously. The function returns a status code indicating
 * the success of the operation or any required recovery actions.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null. The function will handle errors
 * internally if the device is not properly initialized.
 * @return Returns a uint32_t status code indicating the result of the
 * operation, which may include success or specific recovery actions
 * required.
 ******************************************************************************/
uint32_t TALISE_radioOff(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function retrieves the current state of the ARM radio, which can
 * be in one of several states such as POWERUP, READY, IDLE, or RADIO ON.
 * It should be called after the device has been fully initialized. The
 * function requires a valid pointer to store the radio state, and it
 * will return an error if the pointer is null. The radio state is
 * returned as a 32-bit value, although currently only the lower 8 bits
 * are used.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param radioStatus Pointer to a uint32_t where the current ARM radio state
 * will be stored. Must not be null. If null, the function
 * returns an error.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. The radio state is written to the location pointed to by
 * radioStatus if the function succeeds.
 ******************************************************************************/
uint32_t TALISE_getRadioState(taliseDevice_t *device, uint32_t *radioStatus);

/***************************************************************************//**
 * @brief This function is used to enable or disable the specified Rx/ORx and Tx
 * channels in the transceiver when the device is in the radioOn state.
 * It should be called after the device has been initialized and the
 * stream processor has been loaded. The function does not support
 * enabling Rx and ORx channels simultaneously, nor does it support
 * enabling both ORx1 and ORx2 at the same time unless the ORx profile
 * has a lower bandwidth that does not require ADC stitching. The
 * function checks for valid channel configurations and returns an error
 * if invalid parameters are provided or if the channels have not been
 * initialized.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @param rxOrxChannel Specifies the desired Rx or ORx channel to enable. Valid
 * values are defined in the taliseRxORxChannels_t
 * enumeration. Invalid values result in an error.
 * @param txChannel Specifies the desired Tx channel to enable. Valid values are
 * defined in the taliseTxChannels_t enumeration. Invalid
 * values result in an error.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and
 * TALACT_ERR_RESET_SPI for SPI reset required.
 ******************************************************************************/
uint32_t TALISE_setRxTxEnable(taliseDevice_t *device,
			      taliseRxORxChannels_t rxOrxChannel, taliseTxChannels_t txChannel);

/***************************************************************************//**
 * @brief This function is used to read the current state of the Rx/ORx and Tx
 * channel enable signals in the transceiver. It should be called after
 * the device has been initialized and the stream processor has been
 * loaded. The function can be used after entering the radioOn state, as
 * during radioOff all transmit and receive chains are forced off. It is
 * important to ensure that the pointers provided for the channel outputs
 * are not null, as the function will return an error if they are.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param rxOrxChannel Pointer to a variable where the current enabled Rx/ORx
 * channels will be stored. Must not be null.
 * @param txChannel Pointer to a variable where the current enabled Tx channels
 * will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include TALACT_NO_ACTION for success, or error
 * codes indicating specific issues such as null parameters or SPI
 * errors.
 ******************************************************************************/
uint32_t TALISE_getRxTxEnable(taliseDevice_t *device,
			      taliseRxORxChannels_t *rxOrxChannel, taliseTxChannels_t *txChannel);

/***************************************************************************//**
 * @brief This function configures the mapping of Tx channels to ORx channels
 * for external LOL initialization and tracking calibrations. It should
 * be called after the ARM has been initialized and loaded, but before
 * running initialization calibrations. The function allows enabling Tx
 * calibrations and specifies which Tx channel is observable on each ORx
 * channel. It is important to ensure that the parameters are valid, as
 * invalid mappings will result in an error.
 *
 * @param device Pointer to the Talise device structure containing settings.
 * Must not be null.
 * @param txCalEnable Enables Tx calibrations when set to 1; disables when set
 * to 0. This bit is ignored if oRx1Map or oRx2Map is set to
 * TAL_MAP_NONE.
 * @param oRx1Map Specifies the mapping of Tx channels to ORx1 using
 * taliseTxToOrxMapping_t. Valid values are TAL_MAP_NONE,
 * TAL_MAP_TX1_ORX, and TAL_MAP_TX2_ORX. Invalid values result in
 * an error.
 * @param oRx2Map Specifies the mapping of Tx channels to ORx2 using
 * taliseTxToOrxMapping_t. Valid values are TAL_MAP_NONE,
 * TAL_MAP_TX1_ORX, and TAL_MAP_TX2_ORX. Invalid values result in
 * an error.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. Possible values include TALACT_NO_ACTION for success and
 * various error codes for different failure conditions.
 ******************************************************************************/
uint32_t TALISE_setTxToOrxMapping(taliseDevice_t *device, uint8_t txCalEnable,
				  taliseTxToOrxMapping_t oRx1Map, taliseTxToOrxMapping_t oRx2Map);

/***************************************************************************//**
 * @brief This function configures the RF PLL to a specified local oscillator
 * frequency in Hz. It must be called after the ARM has been initialized
 * and while the device is in the radioOff state. The function checks if
 * initialization calibrations are in progress and returns an error if
 * they are. It also validates the frequency against the bandwidth of the
 * Tx, Rx, and ORx profiles, returning an error if the frequency is out
 * of range. The function handles different PLL names and ensures the
 * correct PLL is configured. It is important to ensure that the device
 * is properly initialized and in the correct state before calling this
 * function.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param pllName Name of the PLL to configure, based on the enumerated types in
 * taliseRfPllName_t. Must be a valid PLL name; otherwise, an
 * error is returned.
 * @param rfPllLoFrequency_Hz Desired RF LO frequency in Hz. Must be within
 * valid range for the configured profiles;
 * otherwise, an error is returned.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for parameter errors, and other error
 * codes for specific issues.
 ******************************************************************************/
uint32_t TALISE_setRfPllFrequency(taliseDevice_t *device,
				  taliseRfPllName_t pllName, uint64_t rfPllLoFrequency_Hz);

/***************************************************************************//**
 * @brief Use this function to obtain the current frequency of a specified RF
 * PLL in Hz. It is essential to ensure that the device has been
 * initialized and the PLLs are configured before calling this function.
 * For the RF_PLL and AUX_PLL, the ARM firmware must be loaded and
 * running to read back the frequencies. The function will return an
 * error if the specified PLL name is invalid or if the frequency pointer
 * is null.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param pllName Specifies the PLL whose frequency is to be read. Valid values
 * are TAL_RF_PLL, TAL_AUX_PLL, and TAL_CLK_PLL. An invalid value
 * will result in an error.
 * @param rfPllLoFrequency_Hz Pointer to a uint64_t where the current frequency
 * of the specified PLL will be stored. Must not be
 * null.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success and various error codes for failure.
 ******************************************************************************/
uint32_t TALISE_getRfPllFrequency(taliseDevice_t *device,
				  taliseRfPllName_t pllName, uint64_t *rfPllLoFrequency_Hz);

/***************************************************************************//**
 * @brief This function returns the lock status of the Talise PLLs by updating
 * the provided pointer with a bitwise representation of the lock status
 * for the CLK PLL, RF PLL, and AUX PLL. It should be called after the
 * PLLs have been configured and are operational. The function does not
 * modify the device state but requires a valid pointer to store the lock
 * status.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param pllLockStatus Pointer to a uint8_t where the lock status of the PLLs
 * will be stored. The 3 LSBs represent the lock status of
 * the CLK PLL, RF PLL, and AUX PLL, respectively. Must not
 * be null; otherwise, the function returns an error.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation, with specific error codes for parameter checks and SPI
 * errors.
 ******************************************************************************/
uint32_t TALISE_getPllsLockStatus(taliseDevice_t *device,
				  uint8_t *pllLockStatus);

/***************************************************************************//**
 * @brief This function configures the loop filter settings for the RF PLL,
 * which affects the loop bandwidth and stability of the PLL. It should
 * be called after the ARM has been initialized and while the device is
 * in the radioOff state. After setting the loop filter, the RF PLL
 * frequency must be set using the TALISE_setRfPllFrequency function to
 * apply the new configuration. This function is deprecated and
 * TALISE_setPllLoopFilter should be used instead for more flexibility.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param loopBandwidth_kHz Desired loop bandwidth in kHz. Valid range is
 * between 50kHz and 750kHz. Invalid values will result
 * in an error.
 * @param stability Factor impacting noise and stability of the loop filter.
 * Valid range is between 3 and 15. Lower values decrease
 * stability and increase noise rejection. Invalid values will
 * result in an error.
 * @return Returns a uint32_t indicating the recovery action required, such as
 * TALACT_NO_ACTION for success or various error codes for different
 * failure conditions.
 ******************************************************************************/
uint32_t TALISE_setRfPllLoopFilter(taliseDevice_t *device,
				   uint16_t loopBandwidth_kHz, uint8_t stability);

/***************************************************************************//**
 * @brief Use this function to obtain the current loop bandwidth and stability
 * factor settings for the RF PLL loop filter. It is essential to call
 * this function after the device has been initialized and the RF_PLL has
 * been configured. The ARM firmware must be loaded and running for this
 * function to execute successfully. This function is deprecated, and it
 * is recommended to use TALISE_getPllLoopFilter for accessing both RF
 * and Aux PLL loop filter settings.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param loopBandwidth_kHz Pointer to a variable where the loop bandwidth of
 * the RFPLL in kHz will be stored. Must not be null.
 * @param stability Pointer to a variable where the stability factor of the loop
 * filter will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_CHECK_TIMER, TALACT_ERR_RESET_SPI,
 * TALACT_ERR_RESET_ARM, and TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_getRfPllLoopFilter(taliseDevice_t *device,
				   uint16_t *loopBandwidth_kHz, uint8_t *stability);

/***************************************************************************//**
 * @brief This function configures the loop filter settings for either the RFPLL
 * or AuxPLL by specifying the desired loop bandwidth and stability
 * factor. It should be called after the ARM has been initialized and
 * while the device is in the radioOff state. The function must be
 * followed by a call to set the RF PLL frequency to apply the new
 * configuration. The loop bandwidth must be between 50 kHz and 750 kHz,
 * and the stability factor must be between 3 and 15. Invalid parameters
 * will result in an error.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param pllName Specifies the PLL to configure, either TAL_RF_PLL or
 * TAL_AUX_PLL. Invalid values will result in an error.
 * @param loopBandwidth_kHz Desired loop bandwidth in kHz, must be between 50
 * and 750. Values outside this range will result in an
 * error.
 * @param stability Stability factor for the loop filter, must be between 3 and
 * 15. Values outside this range will result in an error.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, or various error codes for parameter validation failures.
 ******************************************************************************/
uint32_t TALISE_setPllLoopFilter(taliseDevice_t *device,
				 taliseRfPllName_t pllName, uint16_t loopBandwidth_kHz, uint8_t stability);

/***************************************************************************//**
 * @brief This function is used to obtain the current loop bandwidth and
 * stability factor for either the RF or Aux PLL. It should be called
 * after the device has been initialized and the desired PLL configured,
 * with the ARM firmware loaded and running. The function requires valid
 * pointers for the output parameters and will return an error if any
 * input parameters are invalid.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param pllName Specifies the PLL (RF or Aux) for which the loop filter
 * settings are to be retrieved. Must be either TAL_RF_PLL or
 * TAL_AUX_PLL.
 * @param loopBandwidth_kHz Pointer to a variable where the loop bandwidth in
 * kHz will be stored. Must not be null.
 * @param stability Pointer to a variable where the stability factor will be
 * stored. Must not be null.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or
 * command execution issues.
 ******************************************************************************/
uint32_t TALISE_getPllLoopFilter(taliseDevice_t *device,
				 taliseRfPllName_t pllName, uint16_t *loopBandwidth_kHz, uint8_t *stability);

/***************************************************************************//**
 * @brief This function configures the Observation Receiver (ORx) Local
 * Oscillator (LO) source to use either the RF PLL or the Aux PLL. It
 * should be called after the device has been fully initialized and the
 * initial calibrations have been completed. The function is intended to
 * be used in the radio off state. Changing the LO source does not take
 * effect until the ORx channel is powered up. If the LO source is
 * changed while the ORx is active, the change will only take effect
 * after the ORx is disabled and re-enabled.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param orxLoSource Specifies the desired ORx LO source. Must be a valid
 * taliseObsRxLoSource_t value, either TAL_OBSLO_RF_PLL or
 * TAL_OBSLO_AUX_PLL. Invalid values result in an error.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for invalid parameters, and other error codes indicating specific
 * issues.
 ******************************************************************************/
uint32_t TALISE_setOrxLoSource(taliseDevice_t *device,
			       taliseObsRxLoSource_t orxLoSource);

/***************************************************************************//**
 * @brief Use this function to determine the current local oscillator source for
 * the observation receivers ORx1 and ORx2, which can be either the RF
 * PLL or the Aux PLL. This function should be called after the device
 * has been fully initialized and the initial calibrations have been
 * completed. It can be used in both the radio On and Off states. Ensure
 * that the pointers provided for the LO sources are not null, as the
 * function will return an error if they are.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param orx1LoSource Pointer to a variable where the function will store the
 * current LO source for ORx1. Must not be null.
 * @param orx2LoSource Pointer to a variable where the function will store the
 * current LO source for ORx2. Must not be null.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for null parameter errors, and
 * TALACT_ERR_RESET_SPI for SPI-related errors.
 ******************************************************************************/
uint32_t TALISE_getOrxLoSource(taliseDevice_t *device,
			       taliseObsRxLoSource_t *orx1LoSource, taliseObsRxLoSource_t *orx2LoSource);

/***************************************************************************//**
 * @brief This function sets the frequency hopping configuration for the Talise
 * device, including the trigger GPIO pin and the frequency range for
 * hopping. It must be called after the device and ARM processor have
 * been fully initialized and can only be executed in the radio Off
 * state. The function ensures that the frequency hopping minimum
 * frequency is greater than the maximum frequency and checks the
 * validity of the GPIO pin used for triggering. It also updates the
 * device's internal state with the new frequency range settings.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param fhmConfig Pointer to the FHM configuration data structure. Must not be
 * null. The structure should contain valid GPIO pin and
 * frequency range values. Invalid GPIO pins or frequency
 * ranges will result in an error.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for parameter errors, and other error
 * codes for specific issues.
 ******************************************************************************/
uint32_t TALISE_setFhmConfig(taliseDevice_t *device,
			     taliseFhmConfig_t *fhmConfig);

/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of the frequency
 * hopping mode, including the trigger GPIO pin and the frequency range
 * settings. It is essential to ensure that the `fhmConfig` parameter is
 * not null before calling this function, as a null pointer will result
 * in an error. This function is typically called to verify or log the
 * current frequency hopping settings in a Talise device.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param fhmConfig Pointer to a `taliseFhmConfig_t` structure where the current
 * FHM configuration will be stored. Must not be null.
 * @return Returns a `uint32_t` value indicating the success or failure of the
 * operation, with specific codes for different types of errors or
 * warnings.
 ******************************************************************************/
uint32_t TALISE_getFhmConfig(taliseDevice_t *device,
			     taliseFhmConfig_t *fhmConfig);

/***************************************************************************//**
 * @brief Use this function to enable or disable frequency hopping mode,
 * configure MCS synchronization, and set the initial frequency for
 * hopping. It must be called after the device is fully initialized, the
 * ARM processor is initialized, and the frequency hopping configuration
 * is set using TALISE_setFhmConfig(). This function can only be called
 * when the device is in the Radio On state. Ensure that frequency
 * hopping mode is disabled before switching modes.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param fhmMode Pointer to a taliseFhmMode_t structure containing frequency
 * hop mode settings. Must not be null. The structure should be
 * properly configured with valid frequency and mode settings.
 * @return Returns a uint32_t value indicating the result of the operation, with
 * specific values representing different recovery actions or success.
 ******************************************************************************/
uint32_t TALISE_setFhmMode(taliseDevice_t *device, taliseFhmMode_t *fhmMode);

/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of the frequency
 * hopping mode, including the status of frequency hopping enablement,
 * MCS synchronization, and exit mode settings. This function is useful
 * for verifying the current operational state of the frequency hopping
 * mode. It is important to ensure that the `fhmMode` parameter is not
 * null before calling this function, as a null pointer will result in an
 * error.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param fhmMode Pointer to a structure where the current frequency hop mode
 * settings will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for different failure modes.
 ******************************************************************************/
uint32_t TALISE_getFhmMode(taliseDevice_t *device, taliseFhmMode_t *fhmMode);

/***************************************************************************//**
 * @brief This function is used to set the next RF PLL LO hop frequency in both
 * GPIO and non-GPIO modes. In non-GPIO mode, it sends a command to the
 * ARM processor to trigger a frequency hop. In GPIO mode, the user must
 * trigger the hop by sending a low to high pulse on the assigned GPIO
 * after calling this function. It should be called after the device is
 * fully initialized, the ARM processor is initialized, and the FHM
 * configuration and mode are set. This function can only be called in
 * the Radio ON state.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param nextRfPllFrequency_Hz The next RF PLL frequency in Hz for frequency
 * hopping. Must be within the valid frequency
 * range specified by the device's FHM
 * configuration. If the frequency is outside this
 * range, the function will handle it as an invalid
 * parameter.
 * @return Returns a uint32_t indicating the recovery action required. Possible
 * values include TALACT_NO_ACTION for successful completion, or other
 * values indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_setFhmHop(taliseDevice_t *device,
			  uint64_t nextRfPllFrequency_Hz);

/***************************************************************************//**
 * @brief Use this function to obtain the current RF PLL frequency when the
 * device is operating in fast frequency hopping mode. This function is
 * useful for monitoring the frequency changes during hopping operations.
 * It should be called when the device is in frequency hopping mode, and
 * the ARM processor is initialized. Ensure that the device is properly
 * configured for frequency hopping before calling this function.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param fhmRfPllFrequency_Hz Pointer to a uint64_t where the current RF PLL
 * frequency in Hz will be stored. Must not be null.
 * @return Returns a uint32_t indicating the status of the operation. Possible
 * values include TALACT_NO_ACTION for success, or other error codes
 * indicating specific issues.
 ******************************************************************************/
uint32_t TALISE_getFhmRfPllFrequency(taliseDevice_t *device,
				     uint64_t *fhmRfPllFrequency_Hz);

/***************************************************************************//**
 * @brief This function is used to obtain the current status of the Fast Hopping
 * Mode (FHM) from the device. It should be called when the user needs to
 * check the FHM status, which includes error statuses and hop
 * statistics. The function requires a valid device structure and a non-
 * null pointer to a taliseFhmStatus_t structure where the status will be
 * stored. If the fhmStatus parameter is null, the function will return
 * an error. This function is useful for monitoring and debugging FHM
 * operations.
 *
 * @param device A pointer to the taliseDevice_t structure containing the device
 * settings. The caller must ensure this pointer is valid and
 * properly initialized.
 * @param fhmStatus A pointer to a taliseFhmStatus_t structure where the FHM
 * status will be stored. This pointer must not be null, as the
 * function will return an error if it is.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. The return value can be used to determine if any recovery
 * actions are needed.
 ******************************************************************************/
uint32_t TALISE_getFhmStatus(taliseDevice_t *device,
			     taliseFhmStatus_t *fhmStatus);

/***************************************************************************//**
 * @brief Use this function to enable or disable the external local oscillator
 * (LO) output and set the division factor for the output frequency. This
 * function is essential when you need to adjust the external LO output
 * frequency based on the RFPLL VCO frequency, which ranges from 6 GHz to
 * 12 GHz. Ensure that the external LO input mode is not enabled, as it
 * conflicts with the external LO output. Call this function after
 * setting the RFPLL frequency if the internal LO dividers change, to
 * maintain the correct external LO output frequency.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param enableExtLoOutput A boolean value (0 or 1) indicating whether to
 * enable (1) or disable (0) the external LO output
 * buffer.
 * @param extLoOutDivide Specifies the division factor for the external LO
 * output frequency. Must be one of the predefined values:
 * TAL_EXTLO_RFPLLVCO_DIV2, TAL_EXTLO_RFPLLVCO_DIV4,
 * TAL_EXTLO_RFPLLVCO_DIV8, TAL_EXTLO_RFPLLVCO_DIV16,
 * TAL_EXTLO_RFPLLVCO_DIV32, or TAL_EXTLO_RFPLLVCO_DIV64.
 * Invalid values result in an error.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values for success or various error conditions.
 ******************************************************************************/
uint32_t TALISE_setExtLoOutCfg(taliseDevice_t *device,
			       uint8_t enableExtLoOutput, taliseExtLoDiv_t extLoOutDivide);

/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of the external
 * local oscillator (LO) output, including whether it is enabled and the
 * divider setting used. This function is useful for verifying the
 * current state of the external LO output configuration. It should be
 * called after the device has been initialized and configured. Ensure
 * that the pointers provided for output parameters are valid and non-
 * null to avoid parameter errors.
 *
 * @param device Pointer to the Talise device structure containing settings.
 * Must not be null.
 * @param enableExtLoOutput Pointer to a uint8_t where the function will store
 * the enable status of the external LO output. Must
 * not be null.
 * @param extLoOutDivide Pointer to a taliseExtLoDiv_t where the function will
 * store the external LO output divider setting. Must not
 * be null.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation, with specific error codes for parameter errors or SPI
 * communication issues.
 ******************************************************************************/
uint32_t TALISE_getExtLoOutCfg(taliseDevice_t *device,
			       uint8_t *enableExtLoOutput, taliseExtLoDiv_t *extLoOutDivide);
/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif /* TALISE_RADIOCTRL_H_ */

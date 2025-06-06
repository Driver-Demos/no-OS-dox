/**
 * \file talise_tx.h
 * \brief Contains Talise transmit related function prototypes for
 *        talise_tx.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_TX_H_
#define TALISE_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_tx_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief Use this function to adjust the RF output attenuation for a specific
 * transmit channel on a Talise device. It can be called any time after
 * the device has been initialized. The function requires a valid
 * transmit channel and an attenuation value within the specified range.
 * Invalid parameters will result in an error, and the function will
 * return a corresponding recovery action code. This function does not
 * alter the state of the device beyond setting the attenuation.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param txChannel Specifies the transmit channel to set the attenuation for.
 * Must be either TAL_TX1 or TAL_TX2. Invalid values will
 * result in an error.
 * @param txAttenuation_mdB The desired attenuation in milli-dB. Must be in the
 * range 0 to 41950. Values outside this range will
 * result in an error.
 * @return Returns a uint32_t indicating the recovery action required. Possible
 * values include TALACT_NO_ACTION for success, TALACT_WARN_RESET_LOG
 * for a log reset warning, TALACT_ERR_CHECK_PARAM for a parameter
 * error, and TALACT_ERR_RESET_SPI for a SPI reset error.
 ******************************************************************************/
uint32_t TALISE_setTxAttenuation(taliseDevice_t *device,
				 taliseTxChannels_t txChannel, uint16_t txAttenuation_mdB);

/***************************************************************************//**
 * @brief This function retrieves the current attenuation setting for either the
 * Tx1 or Tx2 RF channel in milli-dB. It can be used in both SPI mode and
 * pin-controlled Tx attenuation mode. The function should be called when
 * the Tx data path is powered up to ensure the read value is valid. If
 * the Tx data path is powered down, the last active attenuation setting
 * will be returned. The function requires a valid device pointer and a
 * valid Tx channel, and it writes the attenuation value to the provided
 * pointer.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txChannel Specifies the Tx channel to read from. Must be either
 * TAL_TX1 or TAL_TX2. Invalid values result in an error.
 * @param txAttenuation_mdB Pointer to a uint16_t where the attenuation value in
 * milli-dB will be stored. Must not be null.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values for success or various error conditions.
 ******************************************************************************/
uint32_t TALISE_getTxAttenuation(taliseDevice_t *device,
				 taliseTxChannels_t txChannel, uint16_t *txAttenuation_mdB);


/***************************************************************************//**
 * @brief Use this function to adjust the DAC full scale current boost, which
 * affects the Tx output power. It should be called before loading the
 * Talise ARM processor. The function requires a valid `dacFullScale`
 * parameter, which must be one of the predefined boost levels. If the
 * ARM processor is running, the function will not execute and will
 * return an error. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null, and the device should be initialized before use.
 * @param dacFullScale An enum value of type `taliseDacFullScale_t` representing
 * the desired DAC boost level. Valid values are
 * `TAL_DACFS_0DB` and `TAL_DACFS_3DB`. Invalid values will
 * result in an error.
 * @return Returns a `uint32_t` indicating the status of the operation. Possible
 * return values include `TALACT_NO_ACTION` for success, or error codes
 * indicating specific issues such as invalid parameters or the ARM
 * processor being active.
 ******************************************************************************/
uint32_t TALISE_setDacFullScale(taliseDevice_t *device,
				taliseDacFullScale_t dacFullScale);

/***************************************************************************//**
 * @brief This function is used to enable or disable a digital numerically
 * controlled oscillator (NCO) to generate a continuous wave (CW) test
 * tone on the Tx1 and Tx2 RF outputs of the Talise device. It forces the
 * Tx attenuation to maximum analog output power with a 6dB digital back-
 * off to prevent clipping in the digital filter. When the NCO is
 * disabled, the function sets the Tx attenuation to SPI mode. It is
 * important to ensure that no other API functions are called that change
 * the Tx attenuation mode while using this function. This function
 * should be called after the device has been properly initialized and
 * configured.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txNcoTestToneCfg Pointer to a taliseTxNcoTestToneCfg_t structure
 * containing the configuration for the Tx NCO test
 * tone. Must not be null. The structure should specify
 * whether to enable the NCO and the desired tone
 * frequencies for Tx1 and Tx2, which must be within the
 * range of -Fs/2 to Fs/2, where Fs is the sampling
 * frequency.
 * @return Returns a uint32_t value indicating the result of the operation,
 * which can be a recovery action code such as TALACT_NO_ACTION for
 * success, or an error code if an invalid parameter is detected.
 ******************************************************************************/
uint32_t TALISE_enableTxNco(taliseDevice_t *device,
			    taliseTxNcoTestToneCfg_t *txNcoTestToneCfg);

/***************************************************************************//**
 * @brief This function is used to enable or disable the Tx NCO shifter, which
 * generates a test continuous wave (CW) tone on the Tx1 and Tx2 RF
 * outputs. It allows for independent configuration of each NCO,
 * including frequency and phase adjustments. The function should be
 * called after device initialization and when the Tx profile is valid.
 * It ensures that the Tx attenuation is set to maximum analog output
 * power with a 6dB digital back-off to prevent clipping. The function
 * handles invalid parameters by returning specific error codes.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txNcoShiftCfg Pointer to a taliseTxNcoShifterCfg_t structure
 * containing the configuration for the Tx NCO shifter.
 * Must not be null. The frequency values must be within
 * the range of -Fs/2 to Fs/2, where Fs is the Tx input
 * rate.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or other
 * issues.
 ******************************************************************************/
uint32_t TALISE_txNcoShifterSet(taliseDevice_t *device,
				taliseTxNcoShifterCfg_t *txNcoTestToneCfg);

/***************************************************************************//**
 * @brief Use this function to set up the GPIO input pins and step size for
 * controlling the attenuation of the Tx signal chain via pin control. It
 * allows the baseband processor to adjust the gain by sending high
 * pulses to the specified increment or decrement pins. This function
 * should be called after device initialization and is applicable for
 * either Tx1 or Tx2 channels. Ensure that the `txAttenCtrlPin` structure
 * is correctly populated before calling this function.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txChannel Specifies the Tx channel to configure, either TAL_TX1 or
 * TAL_TX2. Invalid channels will result in an error.
 * @param txAttenCtrlPin Pointer to a taliseTxAttenCtrlPin_t structure that
 * configures the Tx attenuation pin control. Must not be
 * null. The stepSize field must be within the valid range
 * (0 to 31).
 * @return Returns a uint32_t indicating the status of the operation. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for invalid parameters, and TALACT_ERR_RESET_SPI for SPI reset
 * required.
 ******************************************************************************/
uint32_t TALISE_setTxAttenCtrlPin(taliseDevice_t *device,
				  taliseTxChannels_t txChannel, taliseTxAttenCtrlPin_t *txAttenCtrlPin);

/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of GPIO inputs
 * and gain steps for controlling Tx attenuation via pins for a specified
 * Tx channel. This function is useful for verifying or debugging the
 * current pin control settings. It must be called with a valid Tx
 * channel and a non-null pointer for the configuration structure. The
 * function will return an error if the channel is invalid or if the
 * pointer is null.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txChannel Specifies the Tx channel to read the configuration from.
 * Must be either TAL_TX1 or TAL_TX2.
 * @param txAttenCtrlPin Pointer to a taliseTxAttenCtrlPin_t structure where the
 * configuration will be stored. Must not be null.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. Possible values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and
 * TALACT_ERR_RESET_SPI for SPI errors.
 ******************************************************************************/
uint32_t TALISE_getTxAttenCtrlPin(taliseDevice_t *device,
				  taliseTxChannels_t txChannel, taliseTxAttenCtrlPin_t *txAttenCtrlPin);

/***************************************************************************//**
 * @brief This function sets up the PA Protection feature on the Talise device,
 * enabling the measurement of Tx sample power and configuring thresholds
 * for error flagging. It must be called after the device has been
 * initialized and initial calibrations have been completed. The function
 * does not enable automatic Tx attenuation adjustment; this requires a
 * separate function call. It is important to ensure that the provided
 * configuration parameters are within valid ranges to avoid errors.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txPaProtectCfg Pointer to a taliseTxPaProtectCfg_t structure
 * containing the configuration settings for the PA
 * Protection feature. Must not be null. The structure's
 * fields must adhere to specific constraints, such as
 * avgDuration not exceeding 14, peakCount not exceeding
 * 30 or 31 depending on silicon revision, and power
 * thresholds being non-zero and within specified
 * maximums.
 * @return Returns a uint32_t value indicating the result of the operation,
 * where specific values correspond to different recovery actions or
 * success.
 ******************************************************************************/
uint32_t TALISE_setPaProtectionCfg(taliseDevice_t *device,
				   taliseTxPaProtectCfg_t *txPaProtectCfg);

/***************************************************************************//**
 * @brief Use this function to read the current configuration of the PA
 * Protection feature from the Talise device. This function should be
 * called after the device has been initialized and the PA Protection
 * configuration has been set using TALISE_setPaProtectionCfg(). It
 * provides the current settings, including power thresholds and
 * attenuation steps, which are crucial for monitoring and managing the
 * power amplifier's protection. Ensure that the provided structure
 * pointer is valid and non-null to avoid errors.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null. The caller retains ownership.
 * @param txPaProtectCfg Pointer to a taliseTxPaProtectCfg_t structure where the
 * PA Protection configuration will be stored. Must not be
 * null. The function will populate this structure with
 * the current configuration settings.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and
 * TALACT_ERR_RESET_SPI for SPI-related errors.
 ******************************************************************************/
uint32_t TALISE_getPaProtectionCfg(taliseDevice_t *device,
				   taliseTxPaProtectCfg_t *txPaProtectCfg);

/***************************************************************************//**
 * @brief This function is intended to control whether the PA power protection
 * feature can automatically adjust the Tx attenuation when the Tx sample
 * power exceeds predefined thresholds. However, due to a hardware issue,
 * this functionality is currently not implemented, and calling this
 * function will not enable the control of Tx attenuation based on PA
 * protection error flags. It should be called after completing normal
 * Talise initialization and initial calibrations, and after configuring
 * PA protection using TALISE_setPaProtectionCfg().
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param enableTxAttenCtrl A uint8_t value where 1 is intended to allow PA
 * protection to reduce Tx output power when thresholds
 * are exceeded, and 0 is to disable this control.
 * However, this parameter currently has no effect due
 * to a hardware issue.
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM,
 * TALACT_ERR_RESET_SPI, or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_enablePaProtection(taliseDevice_t *device,
				   uint8_t enableTxAttenCtrl);

/***************************************************************************//**
 * @brief Use this function to obtain the most recent average IQ sample power
 * for either the Tx1 or Tx2 channel. This function should be called
 * after the device has been properly initialized and the PA Protection
 * configuration has been set. It is important to ensure that the
 * `channelPower` pointer is not null before calling this function, as it
 * will store the read power value. The function will return an error if
 * an invalid channel is specified or if the `channelPower` pointer is
 * null.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param txChannel Specifies the Tx channel to read from. Must be either
 * TAL_TX1 or TAL_TX2. Invalid values will result in an error.
 * @param channelPower Pointer to a uint16_t where the function will store the
 * read power value. Must not be null.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values indicating success or the type of error encountered.
 ******************************************************************************/
uint32_t TALISE_getTxSamplePower(taliseDevice_t *device,
				 taliseTxChannels_t txChannel, uint16_t *channelPower);

/***************************************************************************//**
 * @brief This function retrieves the error flags for the PA Protection feature,
 * indicating if the Tx1 or Tx2 sample power exceeded the configured
 * thresholds. It should be called after initializing the Talise device
 * and configuring the PA Protection settings. The function provides
 * immediate feedback on power concerns by reading the error flags, which
 * can also trigger a GP interrupt if configured. Ensure that the device
 * is properly initialized and the PA Protection is configured before
 * calling this function.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null.
 * @param errorFlags Pointer to a uint8_t where the error flags will be stored.
 * Bit0=1 if Tx1 exceeded the threshold, Bit1=1 if Tx2
 * exceeded the threshold. Must not be null.
 * @return Returns a uint32_t indicating the recovery action required. Possible
 * values include TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM,
 * TALACT_ERR_RESET_SPI, or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_getPaProtectErrorFlags(taliseDevice_t *device,
				       uint8_t *errorFlags);

/***************************************************************************//**
 * @brief Use this function to clear the PA Protection error flags for both Tx1
 * and Tx2 channels after they have been read using the
 * TALISE_getPaProtectErrorFlags function. This function is necessary to
 * reset the error state and should be called after handling any PA
 * protection errors. It is important to ensure that the device has been
 * properly initialized and configured with PA protection settings before
 * calling this function. If the PA protection error flags are routed to
 * the GP interrupt pin, it is recommended to use the GP interrupt
 * handler function instead to clear the error flags.
 *
 * @param device Pointer to the Talise device's data structure. Must not be
 * null. The function will handle invalid pointers by returning an
 * error code.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, and TALACT_NO_ACTION,
 * which indicate different recovery actions or successful completion.
 ******************************************************************************/
uint32_t TALISE_clearPaProtectErrorFlags(taliseDevice_t *device);

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

#endif /* TALISE_TX_H_ */

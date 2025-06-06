/**
* \file
* \brief Contains ADRV9001 transmit related function prototypes for
*        adi_adrv9001_tx.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the FPGA9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TX_H_
#define _ADI_ADRV9001_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* System header files */

/* ADI specific header files */
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_types.h"

/***************************************************************************//**
 * @brief This function is used to set the transmission attenuation settings for
 * a specific channel on the ADRV9001 device. It should be called when
 * the channel is in either the STANDBY or CALIBRATED state, and the
 * attenuation mode must not be set to
 * ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC. This function is
 * essential for adjusting the signal strength output of the device,
 * which can be critical for maintaining signal integrity and meeting
 * regulatory requirements.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null, and the device should be
 * properly initialized before calling this function.
 * @param channel An enum value of type adi_common_ChannelNumber_e indicating
 * the Tx channel to configure. Must be a valid channel number
 * supported by the device.
 * @param config A pointer to an adi_adrv9001_TxAttenuationConfig_t structure
 * containing the desired attenuation configuration. Must not be
 * null, and the structure should be properly populated with valid
 * configuration data.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_TxAttenuationConfig_t *config);

/***************************************************************************//**
 * @brief This function is used to retrieve the current transmission (Tx)
 * attenuation configuration for a specified channel on the ADRV9001
 * device. It should be called when the channel is in either the STANDBY
 * or CALIBRATED state. The function provides the current settings for
 * the Tx attenuation, including whether the transmission is disabled on
 * PLL unlock, the attenuation mode, and the step size of the
 * attenuation. This information is returned through the provided
 * configuration structure.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the attenuation
 * configuration. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to a structure where the current Tx attenuation
 * configuration will be stored. Must not be null. The function
 * populates this structure with the current configuration
 * details.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_Inspect(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenuationConfig_t *config);

/***************************************************************************//**
 * @brief This function configures the attenuation control mode for a specified
 * transmission channel on the ADRV9001 device. It should be used when
 * the channel is in either the STANDBY or CALIBRATED state. The function
 * does not support setting the mode to
 * ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC directly; if this mode
 * is specified, it will default to
 * ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI. This function is
 * essential for managing how the device controls attenuation, which can
 * be crucial for maintaining signal integrity and performance.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param channel The transmission channel for which to set the attenuation
 * control mode. It must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param mode The desired attenuation control mode, specified by
 * adi_adrv9001_TxAttenuationControlMode_e. If set to
 * ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC, it will be
 * internally changed to
 * ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_AttenuationMode_Set(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenuationControlMode_e mode);

/***************************************************************************//**
 * @brief Use this function to obtain the current attenuation control mode for a
 * specific Tx channel on the ADRV9001 device. This function is useful
 * for verifying the current mode of operation, especially when the
 * channel state is PRIMED, as it will always return the mode as SPI in
 * that state. Ensure that the device and channel are properly
 * initialized before calling this function. The function will return a
 * status code indicating success or the necessary action to recover from
 * an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before use.
 * @param channel The Tx channel for which to get the attenuation control mode.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param mode Pointer to a variable where the current Tx attenuation mode will
 * be stored. Must not be null.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_AttenuationMode_Get(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_TxAttenuationControlMode_e *mode);

/***************************************************************************//**
 * @brief This function sets the transmission attenuation for a specified
 * channel on the ADRV9001 device. It should be used when you need to
 * adjust the signal strength output from the device. The function
 * requires the device to be in a state where the attenuation can be
 * modified, such as STANDBY, CALIBRATED, PRIMED, or RF_ENABLED,
 * depending on the current attenuation mode. The attenuation value is
 * specified in milli-dB and must adhere to specific resolution and range
 * constraints based on the device's output signaling mode. The function
 * ensures that the attenuation mode is temporarily set to SPI for the
 * operation and restores it afterward. It is important to ensure that
 * the device is properly initialized and in a valid state before calling
 * this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to set the attenuation. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param attenuation_mdB The desired attenuation in milli-dB. Must be a
 * multiple of the resolution (50 or 500 mdB) and within
 * the range of 0 to 41950 mdB, depending on the output
 * signaling mode. Invalid values will result in an
 * error.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_Set(adi_adrv9001_Device_t *adrv9001,
                                        adi_common_ChannelNumber_e channel,
                                        uint16_t attenuation_mdB);

/***************************************************************************//**
 * @brief This function is used to obtain the current transmission attenuation
 * level, in milli-dB, for a specified channel on the ADRV9001 device. It
 * should be called after the device has been initialized and the
 * attenuation table has been loaded. The function is particularly useful
 * for monitoring and adjusting the transmission power levels in real-
 * time applications. Note that during transitions between RF_ENABLED and
 * PRIMED states, the attenuation may be temporarily adjusted to protect
 * the analog front-end, which can result in unexpected values during TDD
 * operation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param channel The Tx channel for which to get the attenuation. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param attenuation_mdB Pointer to a uint16_t where the current attenuation in
 * milli-dB will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The attenuation value is written
 * to the location pointed to by attenuation_mdB.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_Get(adi_adrv9001_Device_t *adrv9001,
                                        adi_common_ChannelNumber_e channel,
                                        uint16_t *attenuation_mdB);

/***************************************************************************//**
 * @brief This function allows you to enable or disable the Tx output power
 * boost for a specified channel on the ADRV9001 device. Enabling the
 * boost increases the output power by 3dB, but it may degrade linearity.
 * The boost is disabled by default. It is important to note that
 * enabling the power boost after calibrations requires running
 * calibrations again to avoid performance degradation. This function
 * should be called when the channel state is STANDBY and the attenuation
 * mode is not ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * number.
 * @param boostEnable Boolean value indicating whether to enable (true) or
 * disable (false) the Tx output power boost.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_OutputPowerBoost_Set(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             bool boostEnable);

/***************************************************************************//**
 * @brief This function is used to check whether the Tx output power boost is
 * currently enabled for a specified channel on the ADRV9001 device. It
 * is useful for verifying the current configuration of the device's
 * transmission settings. The function requires a valid device context
 * and a specified channel. The boost status is returned through a
 * pointer, which must not be null. This function should be called when
 * the user needs to confirm the power boost setting without altering it.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param channel The Tx channel of interest, specified as an enumeration of
 * type adi_common_ChannelNumber_e. Must be a valid channel
 * number.
 * @param boostEnabled A pointer to a boolean variable where the function will
 * store the boost status. Must not be null, as it is used
 * to return the result.
 * @return Returns an integer code indicating success or the required action to
 * recover, with the boost status written to the provided boolean
 * pointer.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_OutputPowerBoost_Get(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             bool *boostEnabled);

/***************************************************************************//**
 * @brief This function writes the attenuation table for the specified Tx
 * channels, allowing the full table to be loaded in a single call or in
 * multiple calls by specifying an index offset. It should be called
 * after device initialization. All unused table entries should be set to
 * maximum attenuation to ensure deterministic output when an
 * unconfigured index is selected.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channelMask An OR'd combination of adi_common_ChannelNumber_e values
 * specifying the Tx channels for which to write the
 * attenuation table.
 * @param indexOffset The starting index in the attenuation table where writing
 * begins. Valid range is 0 to 839.
 * @param attenTableRows Array of attenuation table entries to write. Must not
 * be null.
 * @param arraySize The number of attenuation table rows to write. Valid range
 * is 1 to 840.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_AttenuationTable_Write(adi_adrv9001_Device_t *adrv9001,
                                               uint32_t channelMask,
                                               uint32_t indexOffset,
                                               adi_adrv9001_TxAttenTableRow_t attenTableRows[],
                                               uint32_t arraySize);

/***************************************************************************//**
 * @brief This function retrieves a subset of the attenuation table entries for
 * a specified Tx channel, starting from a given index offset. It can be
 * used to read the entire table in one call or in multiple calls by
 * specifying different index offsets. The function should be called
 * after device initialization. It is important to ensure that the
 * `attenTableRows` array is large enough to hold the requested number of
 * entries, as specified by `arraySize`. The function will populate the
 * `attenTableRows` array with the read entries and optionally return the
 * number of entries read through `numAttenIndicesRead`. If
 * `numAttenIndicesRead` is not needed, it can be set to NULL.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel The Tx channel from which to read the attenuation table. Must
 * be a valid channel number.
 * @param indexOffset The starting index in the attenuation table from which to
 * begin reading. Valid range is 0 to 839.
 * @param attenTableRows Array to store the attenuation table entries read. Must
 * not be null and should have enough space to hold
 * `arraySize` entries.
 * @param arraySize The number of attenuation table rows to read. Determines the
 * size of `attenTableRows`. Must be between 1 and 840.
 * @param numAttenIndicesRead Pointer to store the number of attenuation indices
 * actually read. Can be null if this information is
 * not needed.
 * @return Returns an integer code indicating success or the required action to
 * recover. The `attenTableRows` array is populated with the read
 * entries, and `numAttenIndicesRead` is updated with the number of
 * entries read if it is not null.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_AttenuationTable_Read(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              uint32_t indexOffset,
                                              adi_adrv9001_TxAttenTableRow_t attenTableRows[],
                                              uint32_t arraySize,
                                              uint16_t *numAttenIndicesRead);

/***************************************************************************//**
 * @brief This function sets up the Power Amplifier (PA) protection for a
 * specified transmission (Tx) channel on the ADRV9001 device. It enables
 * the measurement of Tx sample power but does not automatically adjust
 * Tx attenuation. This function should be called after completing the
 * normal ADRV9001 initialization and initial calibrations. It is
 * essential for ensuring the safe operation of the PA by monitoring
 * power levels and preventing damage due to excessive power.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to configure PA protection. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param config A pointer to the desired PA protection configuration structure.
 * Must not be null. The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_PaProtection_Configure(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_TxPaProtectCfg_t *config);

/***************************************************************************//**
 * @brief Use this function to retrieve the current Power Amplifier (PA)
 * protection configuration for a specified transmission (Tx) channel on
 * the ADRV9001 device. This function should be called after the device
 * has been properly initialized and calibration has been completed. It
 * provides the current settings for average power, peak power, and
 * average-to-peak ratio protection features, which are essential for
 * ensuring the safe operation of the PA. The function does not modify
 * the device state but populates the provided configuration structure
 * with the current settings.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the PA protection
 * configuration. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to a structure where the current PA protection
 * configuration will be stored. Must not be null. The function
 * populates this structure with the current configuration values.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_PaProtection_Inspect(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_TxPaProtectCfg_t *config);

/***************************************************************************//**
 * @brief This function is used to configure the internal tone generation for a
 * specified transmit (Tx) channel on the ADRV9001 device. It should be
 * called when the channel is in the CALIBRATED state. The function sets
 * parameters such as enabling the tone, its amplitude, and frequency.
 * This is useful for testing and calibration purposes where a known tone
 * is required on the Tx path. Ensure that the device is properly
 * initialized and the channel is in the correct state before calling
 * this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should point to a valid, initialized device
 * context.
 * @param channel The Tx channel number to configure. It must be a valid channel
 * number as defined by the adi_common_ChannelNumber_e
 * enumeration.
 * @param tone A pointer to a structure of type
 * adi_adrv9001_TxInternalToneGeneration_t containing the tone
 * configuration parameters. This must not be null and should be
 * properly initialized with the desired tone settings.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_InternalToneGeneration_Configure(adi_adrv9001_Device_t *adrv9001,
                                                         adi_common_ChannelNumber_e channel,
                                                         adi_adrv9001_TxInternalToneGeneration_t *tone);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the Tx NCO
 * test tone for a specified channel. It is applicable when the channel
 * is in any of the CALIBRATED, PRIMED, or RF_ENABLED states. This
 * function is useful for verifying the current settings of the internal
 * tone generation, such as its enable status, amplitude, and frequency.
 * Ensure that the device is properly initialized and the channel is in a
 * valid state before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the NCO test tone
 * frequency. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param tone Pointer to a structure where the current NCO tone configuration
 * will be stored. Must not be null. The structure will be populated
 * with the enable status, amplitude, and frequency of the tone.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_InternalToneGeneration_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                       adi_common_ChannelNumber_e channel,
                                                       adi_adrv9001_TxInternalToneGeneration_t *tone);


/***************************************************************************//**
 * @brief This function is used to configure the slew rate limiter for a
 * specified transmit (Tx) channel on the ADRV9001 device. It should be
 * called when the channel is in the STANDBY state. The function applies
 * the provided configuration settings to the slew rate limiter, which
 * can affect the signal's rise and fall times. Proper configuration of
 * the slew rate limiter is essential for optimizing signal performance
 * and ensuring compliance with system requirements.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to configure the slew rate limiter.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to the desired slew rate limiter configuration. Must
 * not be null. The caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_SlewRateLimiter_Configure(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SlewRateLimiterCfg_t *config);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the slew
 * rate limiter for a specified Tx channel. It is useful for verifying
 * the current settings of the slew rate limiter. This function can be
 * called when the channel is in any of the following states: STANDBY,
 * CALIBRATED, PRIMED, or RF_ENABLED. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the slew rate limiter
 * configuration. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to a adi_adrv9001_SlewRateLimiterCfg_t structure where
 * the current configuration will be stored. Must not be null. The
 * function populates this structure with the current
 * configuration.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_SlewRateLimiter_Inspect(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_SlewRateLimiterCfg_t *config);

/***************************************************************************//**
 * @brief This function is used to configure the Power Amplifier (PA) ramp
 * settings for a specified transmission (Tx) channel on the ADRV9001
 * device. It should be called when the channel is in one of the
 * following states: STANDBY, CALIBRATED, or PRIMED. The function
 * requires a valid PA ramp configuration structure, which includes
 * settings such as the auxiliary DAC channel selection, trigger delays,
 * and the ramp lookup table. The GPIO source must not be unassigned,
 * even if the trigger select is not set to GPIO. This function returns a
 * status code indicating success or the necessary action to recover from
 * an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel to configure, specified as an enumeration of
 * type adi_common_ChannelNumber_e. Must be a valid channel
 * number.
 * @param paRampCfg Pointer to a structure of type adi_adrv9001_PaRampCfg_t
 * containing the PA ramp configuration settings. Must not be
 * null. The caller retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_PaRamp_Configure(adi_adrv9001_Device_t *adrv9001,
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_PaRampCfg_t *paRampCfg);

/***************************************************************************//**
 * @brief This function retrieves the current PA ramp configuration for a
 * specified Tx channel on the ADRV9001 device. It should be called when
 * the channel is in one of the following states: STANDBY, CALIBRATED, or
 * PRIMED. The function fills the provided configuration structure with
 * the current settings, including the ramp clock frequency, LUT values,
 * and other parameters. It is important to ensure that the device is
 * properly initialized before calling this function. The function does
 * not modify the device state or configuration, but it does require a
 * valid device context and channel specification.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the PA ramp configuration.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param paRampCfg A pointer to a structure where the PA ramp configuration
 * will be stored. Must not be null. The function populates
 * this structure with the current configuration.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_PaRamp_Inspect(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_ChannelNumber_e channel,
                                       adi_adrv9001_PaRampCfg_t *paRampCfg);

/***************************************************************************//**
 * @brief This function sets up the transmission (Tx) attenuation control via
 * GPIO pins for a specified channel on the ADRV9001 device. It should be
 * used when the channel is in the CALIBRATED state and the attenuation
 * mode is not set to ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC. The
 * function configures the GPIO pins to control the attenuation increment
 * and decrement, allowing for dynamic adjustment of the Tx signal
 * strength. It is important to ensure that the device is properly
 * initialized and in the correct state before calling this function.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null, and the device should be
 * properly initialized.
 * @param channel An enum value of type adi_common_ChannelNumber_e indicating
 * the Tx channel to configure. Valid values are typically
 * channel 1 or 2.
 * @param config A pointer to an adi_adrv9001_TxAttenuationPinControlCfg_t
 * structure containing the desired configuration for the Tx
 * attenuation pin control. Must not be null, and should be
 * properly populated with valid configuration data.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_PinControl_Configure(adi_adrv9001_Device_t *adrv9001,
                                                         adi_common_ChannelNumber_e channel,
                                                         adi_adrv9001_TxAttenuationPinControlCfg_t *config);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the Tx
 * attenuation control via GPIO pins for a specified channel. This
 * function is useful for verifying the current settings of the
 * attenuation control pins. It should be called when the channel is in a
 * CALIBRATED state, and it is not supported in TX_DIRECT_FM_FSK mode.
 * The function will populate the provided configuration structure with
 * the current settings, including the step size and the GPIO pins used
 * for incrementing and decrementing attenuation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to inspect the attenuation
 * configuration. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to a structure where the current Tx attenuation pin
 * control configuration will be stored. Must not be null. The
 * caller provides this structure and retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_Attenuation_PinControl_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                       adi_common_ChannelNumber_e channel,
                                                       adi_adrv9001_TxAttenuationPinControlCfg_t *config);

/***************************************************************************//**
 * @brief This function adjusts the NCO frequency for a specified Tx channel to
 * correct for small deviations in the local oscillator frequency. It
 * should be used when the channel is in one of the CALIBRATED, PRIMED,
 * or RF_ENABLED states. The function allows for immediate frequency
 * changes or updates at the start of the next available frame, depending
 * on the 'immediate' parameter. Ensure that the Tx data path profile is
 * set to ADI_ADRV9001_TX_DP_IQDMDUC_MODE2 before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to set the NCO frequency. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param frequencyOffset_Hz The desired offset frequency in Hz. Can be positive
 * or negative to adjust the frequency up or down.
 * @param immediate Boolean flag indicating whether to change the frequency
 * immediately (true) or at the start of the next available
 * frame (false).
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_FrequencyCorrection_Set(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                int32_t frequencyOffset_Hz,
                                                bool immediate);

/***************************************************************************//**
 * @brief Use this function to enable or disable the loopback from the Rx
 * datapath to the Tx datapath for a specified channel on the ADRV9001
 * device. This is useful for hardware debugging and allows verification
 * of the datapath using an RF signal applied to the Rx port and observed
 * at the Tx port without requiring SSI configuration. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel for which to set the datapath loopback. Valid
 * values are ADI_CHANNEL_1 and ADI_CHANNEL_2. If an invalid
 * channel is provided, the function will not perform the
 * operation.
 * @param loopbackEnable Boolean flag indicating whether to enable (true) or
 * disable (false) the Tx datapath loopback.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Tx_DataPath_Loopback_Set(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              bool loopbackEnable);


#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_TX_H_ */

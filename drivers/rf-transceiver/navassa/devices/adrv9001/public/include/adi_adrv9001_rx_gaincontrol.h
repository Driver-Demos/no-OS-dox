/**
 * \file
 * \brief Gain Control
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RX_GAINCONTROL_H_
#define _ADI_ADRV9001_RX_GAINCONTROL_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief Use this function to set the desired gain control mode for a specific
 * Rx channel on the ADRV9001 device. It is essential to call this
 * function before initiating Rx operations to ensure the correct gain
 * control mode is applied. The function supports different gain control
 * modes, including SPI, PIN, and AUTO, and adjusts the device settings
 * accordingly. Proper validation of input parameters is performed, and
 * the function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param channel The Rx channel for which to set the gain control mode. Must be
 * a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param gainCtrlMode The desired gain control mode to set. Must be a valid
 * mode as defined by adi_adrv9001_RxGainControlMode_e.
 * Invalid modes will result in an error.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_Mode_Set(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_RxGainControlMode_e mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current gain control mode for a
 * specified Rx channel on the ADRV9001 device. It should be called when
 * there is a need to verify or inspect the gain control mode
 * configuration of a channel. The function requires a valid device
 * context and channel number, and it outputs the current gain control
 * mode through a pointer. It is important to ensure that the device and
 * channel parameters are correctly initialized before calling this
 * function to avoid unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param channel The Rx channel for which the gain control mode is to be read.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param mode Pointer to a variable where the current gain control mode will be
 * stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_Mode_Get(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_RxGainControlMode_e *mode);
    
/***************************************************************************//**
 * @brief This function configures the Automatic Gain Control (AGC) settings for
 * a specified Rx channel on the ADRV9001 device. It should be called
 * when the channel is in one of the following states: CALIBRATED,
 * PRIMED, or RF_ENABLED. This function does not enable AGC mode; after
 * configuration, the user must call adi_adrv9001_Rx_GainCtrlMode_Set()
 * to activate the desired gain control mode. It is important to ensure
 * that the device and channel parameters are correctly initialized
 * before calling this function to avoid unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to configure AGC. Must be a valid
 * channel number as defined by adi_common_ChannelNumber_e.
 * @param agcCfg Pointer to the AGC configuration structure to apply. Must not
 * be null. The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_GainControlCfg_t *agcCfg);

/***************************************************************************//**
 * @brief Use this function to retrieve the current AGC configuration for a
 * specific Rx channel on the ADRV9001 device. It is essential to call
 * this function when the channel is in one of the following states:
 * CALIBRATED, PRIMED, or RF_ENABLED. This function provides detailed AGC
 * settings, which can be useful for diagnostics or configuration
 * verification. Ensure that the device and channel parameters are valid
 * before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized.
 * @param channel The Rx Channel for which to inspect the AGC configuration.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param agcCfg Pointer to a structure where the current AGC configuration will
 * be stored. Must not be null, and the caller is responsible for
 * allocating this structure.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_Inspect(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_GainControlCfg_t *agcCfg);

/***************************************************************************//**
 * @brief This function configures the Automatic Gain Control (AGC) by setting
 * the minimum and maximum gain indexes for a specified Rx channel on the
 * ADRV9001 device. It is essential to ensure that the channel is
 * correctly specified and that the gain indexes are within the valid
 * range before calling this function. This function should be used when
 * you need to define the operational gain range for AGC, typically
 * before starting Rx operations. The function returns a status code
 * indicating success or the necessary action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx Channel for which to set the min and max gain indices.
 * Must be a valid channel number.
 * @param minGainIndex The lower limit of the gain index. Valid range is from 0
 * to 255. Values outside this range may result in an error.
 * @param maxGainIndex The upper limit of the gain index. Valid range is from 0
 * to 255. Values outside this range may result in an error.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Set(adi_adrv9001_Device_t *adrv9001,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint8_t minGainIndex,
                                                        uint8_t maxGainIndex);
    
/***************************************************************************//**
 * @brief Use this function to obtain the current minimum and maximum gain
 * indexes configured for the Automatic Gain Control (AGC) of a specified
 * Rx channel on the ADRV9001 device. This function is useful for
 * inspecting the gain limits that are currently set, which can be
 * critical for debugging or monitoring the AGC behavior. It should be
 * called when the device is in a state where the channel is accessible,
 * and the gain indexes are relevant to the operation. Ensure that the
 * pointers provided for the gain indexes are valid and that the device
 * context is properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param channel The Rx Channel from which to get the min and max gain indices.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param minGainIndex Pointer to a uint8_t where the function will store the
 * lower limit of the gain index. Must not be null.
 * @param maxGainIndex Pointer to a uint8_t where the function will store the
 * upper limit of the gain index. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_MinMaxGainIndex_Get(adi_adrv9001_Device_t *adrv9001,
                                                        adi_common_ChannelNumber_e channel,
                                                        uint8_t *minGainIndex,
                                                        uint8_t *maxGainIndex);

/***************************************************************************//**
 * @brief Use this function to reset the Automatic Gain Control (AGC) state
 * machine for a specific Rx channel on the ADRV9001 device. This
 * operation is typically performed when you need to reinitialize the AGC
 * state, for example, after a configuration change or to recover from an
 * error state. Upon reset, the gain will revert to its maximum value.
 * This function should be called when the channel is in a state that
 * allows for AGC reset, such as when the channel is not actively
 * processing data.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx Channel for which to reset the AGC state machine. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_Reset(adi_adrv9001_Device_t *adrv9001, adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief This function sets up the gain control for a specified Rx channel
 * using pin mode. It should be called before initiating Rx operations to
 * ensure the gain control is configured correctly. The function
 * configures the minimum and maximum gain indices, the increment and
 * decrement step sizes, and the GPIO pins used for gain control. It is
 * important to ensure that the device is properly initialized and that
 * the channel is valid before calling this function. The function
 * returns a status code indicating success or the necessary action to
 * recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx Channel for which to configure pin gain control. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to the desired pin gain control configuration. Must not
 * be null. The configuration includes parameters like min/max
 * gain index and GPIO pins for increment/decrement.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_PinMode_Configure(adi_adrv9001_Device_t *adrv9001, 
                                                      adi_common_ChannelNumber_e channel,
                                                      adi_adrv9001_RxGainControlPinCfg_t *config);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the pin
 * mode gain control for a specified Rx channel on the ADRV9001 device.
 * This function is useful for verifying the current settings of the gain
 * control pins, including the minimum and maximum gain indices, and the
 * increment and decrement step sizes. It should be called when the
 * channel is in a state of CALIBRATED, PRIMED, or RF_ENABLED. The
 * function requires a valid device context and channel number, and it
 * outputs the configuration details into the provided configuration
 * structure.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to inspect the pin gain control
 * configuration. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param config Pointer to a structure where the current pin gain control
 * configuration will be stored. Must not be null. The caller
 * provides the memory for this structure.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The configuration details are
 * written to the provided config structure.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainControl_PinMode_Inspect(adi_adrv9001_Device_t *adrv9001, 
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_RxGainControlPinCfg_t *config);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_GAINCONTROL_H_ */

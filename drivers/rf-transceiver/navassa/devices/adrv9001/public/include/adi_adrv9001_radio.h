/**
 * \file
 * \brief Contains ADRV9001 related function prototypes for adi_adrv9001_radioctrl.c
 *
 *  ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RADIO_H_
#define _ADI_ADRV9001_RADIO_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adrv9001_init_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Minimum supported carrier frequency 
 */
#define ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ 25000000llu    /* 25 MHz */
/**
 * \brief Maximum supported carrier frequency
 */
#define ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ 6000000000llu  /* 6 GHz */
/**
 * \brief Maximum supported intermediate frequency
 */
#define ADI_ADRV9001_INTERMEDIATE_FREQUENCY_MAX_HZ 20000000  /* 20MHz */
/***************************************************************************//**
 * @brief This function sets the carrier frequency for a specified channel on
 * the ADRV9001 device. It should be called when the channel is in either
 * the STANDBY or CALIBRATED state. The function requires a valid device
 * context, port, channel, and carrier configuration. It communicates
 * with the device via a mailbox command to apply the configuration.
 * Proper error handling is performed, and the function returns a status
 * code indicating success or the necessary action to recover from an
 * error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port for the device.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * for the specified port.
 * @param carrier Pointer to an adi_adrv9001_Carrier_t structure containing the
 * desired carrier configuration. Must not be null, and the
 * carrier frequency should be within the supported range of 25
 * MHz to 6 GHz.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Carrier_Configure(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_Carrier_t *carrier);

/***************************************************************************//**
 * @brief This function is used to obtain the current carrier frequency and
 * related configuration for a specified channel on the ADRV9001 device.
 * It should be called when the channel is in any of the following
 * states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. The function fills
 * the provided `carrier` structure with the current carrier frequency
 * and intermediate frequency, if applicable. It is important to ensure
 * that the `carrier` parameter is a valid pointer to a
 * `adi_adrv9001_Carrier_t` structure, as this is where the function will
 * store the retrieved data.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port The port of interest, specified as an `adi_common_Port_e` enum
 * value. Must be a valid port.
 * @param channel The channel of interest, specified as an
 * `adi_common_ChannelNumber_e` enum value. Must be a valid
 * channel.
 * @param carrier Pointer to a `adi_adrv9001_Carrier_t` structure where the
 * current carrier configuration will be stored. Must not be
 * null.
 * @return Returns an `int32_t` code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Carrier_Inspect(adi_adrv9001_Device_t *adrv9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_Carrier_t *carrier);

/***************************************************************************//**
 * @brief This function is used to set the configuration parameters for a
 * specified PLL in the ADRV9001 device. It should be called before
 * running the initialization calibration when all channels are in the
 * STANDBY state. This ensures that the PLL is correctly configured
 * before any further operations are performed. The function requires a
 * valid device context, PLL identifier, and configuration parameters. It
 * returns a status code indicating success or the necessary action to
 * recover from an error.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null and should be properly initialized
 * before calling this function. The caller retains ownership.
 * @param pllId An identifier for the PLL of interest. It must be a valid value
 * of type adi_adrv9001_Pll_e, representing the specific PLL to
 * configure.
 * @param pllConfig A pointer to a structure containing the desired PLL
 * configuration parameters. This must not be null and should
 * be properly populated with valid configuration data before
 * calling the function. The caller retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
    int32_t adi_adrv9001_Radio_Pll_Configure(adi_adrv9001_Device_t *adrv9001,
                                             adi_adrv9001_Pll_e pllId,
                                             adi_adrv9001_PllConfig_t *pllConfig);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration settings of
 * a specified PLL in the ADRV9001 device. It should be called when the
 * channel state is any of STANDBY, CALIBRATED, PRIMED, or RF_ENABLED.
 * The function fills the provided pllConfig structure with the PLL's
 * calibration and power settings. It is important to ensure that the
 * adrv9001 device context is properly initialized and that the pllConfig
 * pointer is valid and non-null before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param pllId The identifier of the PLL of interest. Must be a valid
 * adi_adrv9001_Pll_e enumeration value.
 * @param pllConfig Pointer to a structure where the PLL configuration will be
 * stored. Must not be null. The structure is filled with the
 * PLL's calibration and power settings.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
    int32_t adi_adrv9001_Radio_Pll_Inspect(adi_adrv9001_Device_t *adrv9001,
                                           adi_adrv9001_Pll_e pllId,
                                           adi_adrv9001_PllConfig_t *pllConfig);
/***************************************************************************//**
 * @brief This function checks the lock status of a specified PLL in the
 * ADRV9001 device. It should be called after the PLLs have been
 * configured and are operational. The function provides the lock status
 * through an output parameter, indicating whether the PLL is locked or
 * not. It returns a status code indicating success or the required
 * action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param pll The PLL of interest, specified as an enum value of type
 * adi_adrv9001_Pll_e. Must be a valid PLL identifier.
 * @param locked Output parameter that will be set to true if the PLL is locked,
 * false otherwise. Must not be null.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_PllStatus_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_Pll_e pll, bool *locked);

/***************************************************************************//**
 * @brief This function is used to set the enable mode for a specific channel on
 * the ADRV9001 device, allowing the user to switch between SPI and pin
 * mode control for Rx, ORx, and Tx signal chains. It should be called
 * after the device has been initialized and the stream processor has
 * been loaded. The function requires valid port and channel parameters,
 * and it returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid
 * channel.
 * @param mode The desired enable mode, specified as an
 * adi_adrv9001_ChannelEnableMode_e enum value. Must be a valid
 * mode.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_ChannelEnableMode_Set(adi_adrv9001_Device_t *adrv9001,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 adi_adrv9001_ChannelEnableMode_e mode);

/***************************************************************************//**
 * @brief This function retrieves the current enable mode of a specified channel
 * on the ADRV9001 device. It should be called after the device has been
 * initialized and the stream processor has been loaded. The function
 * requires specifying the port and channel of interest, and it outputs
 * the current enable mode through a pointer. It is important to ensure
 * that the provided pointers are valid and that the function is called
 * in the correct sequence of operations to avoid undefined behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Valid values are typically ADI_RX or ADI_TX.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Valid values are
 * typically ADI_CHANNEL_1 or ADI_CHANNEL_2.
 * @param mode Pointer to an adi_adrv9001_ChannelEnableMode_e variable where the
 * current enable mode will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_ChannelEnableMode_Get(adi_adrv9001_Device_t *adrv9001,
                                                 adi_common_Port_e port,
                                                 adi_common_ChannelNumber_e channel,
                                                 adi_adrv9001_ChannelEnableMode_e *mode);

/***************************************************************************//**
 * @brief This function retrieves the current state of the ARM radio from the
 * ADRV9001 device. It should be used after the device has been
 * initialized, the ARM binary has been loaded, and the PLLs have been
 * configured. The function provides detailed information about the
 * system state, monitor mode state, MCS state, boot state, and channel
 * states for both Rx and Tx channels. It is essential for monitoring and
 * debugging the radio's operational status.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param radioState Pointer to a structure where the current radio state will
 * be stored. Must not be null. The caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_State_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_RadioState_t *radioState);
    
/***************************************************************************//**
 * @brief This function is used to obtain the current state of a specified
 * channel on a given port of the ADRV9001 device. It should be called
 * after the device has been initialized, the ARM binary has been loaded,
 * and the PLLs have been configured. The function provides the state of
 * the channel, which can be useful for determining the channel's
 * readiness for further operations or transitions. It is important to
 * ensure that the `adrv9001` context and the `channelState` pointer are
 * valid before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port The port of interest, specified as an `adi_common_Port_e`
 * enumeration value.
 * @param channel The channel number of interest, specified as an
 * `adi_common_ChannelNumber_e` enumeration value.
 * @param channelState Pointer to an `adi_adrv9001_ChannelState_e` variable
 * where the current state of the channel will be stored.
 * Must not be null.
 * @return Returns an `int32_t` code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_State_Get(adi_adrv9001_Device_t *adrv9001, 
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel,
                                             adi_adrv9001_ChannelState_e *channelState);

/***************************************************************************//**
 * @brief This function is used to transition a specified radio channel between
 * the CALIBRATED and PRIMED states. It is typically called when
 * preparing a channel for RF operation or when reverting it back to a
 * non-operational state. The function requires a valid device context
 * and specifies the port and channel to be primed or unprimed. It is
 * important to ensure that the channel is in a state where transitioning
 * to PRIMED is valid before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port for the device.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * for the specified port.
 * @param prime Boolean value indicating whether to prime (true) or unprime
 * (false) the channel.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_Prime(adi_adrv9001_Device_t *adrv9001,
                                         adi_common_Port_e port,
                                         adi_common_ChannelNumber_e channel,
                                         bool prime);

/***************************************************************************//**
 * @brief Use this function to transition specified radio channels between the
 * CALIBRATED and PRIMED states. It is essential to ensure that each
 * channel is in the appropriate state before calling this function:
 * channels must be in the CALIBRATED state to be primed and in the
 * PRIMED state to be unprimed. This function is typically used when
 * preparing channels for RF operation or returning them to a non-
 * operational state. The function requires a valid device context and
 * arrays of ports and channels, with a specified length indicating the
 * number of channel-port combinations. The operation is controlled by
 * the 'prime' parameter, which determines whether the channels are
 * primed or unprimed.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param ports Array of ports corresponding to the channels to be primed or
 * unprimed. The array must have at least 'length' elements.
 * @param channels Array of channels corresponding to the ports to be primed or
 * unprimed. The array must have at least 'length' elements.
 * @param length The number of port/channel combinations to process. Must be
 * greater than zero and correspond to the size of the 'ports' and
 * 'channels' arrays.
 * @param prime Boolean value indicating whether to prime (true) or unprime
 * (false) the channels.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channels_Prime(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_Port_e ports[],
                                          adi_common_ChannelNumber_e channels[],
                                          uint32_t length,
                                          bool prime);

/***************************************************************************//**
 * @brief This function is used to transition a specified channel from the
 * PRIMED state to the RF ENABLED state, or vice versa, depending on the
 * 'enable' parameter. It is applicable only when the channel is in SPI
 * enable mode, which should be set using
 * adi_adrv9001_ChannelEnableModeSet(). The function should be called
 * after the device has been initialized and the stream processor loaded.
 * It is important to ensure that the channel is in the correct state
 * before calling this function to avoid unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value.
 * @param enable Boolean value indicating whether to enable (true) or disable
 * (false) RF for the channel.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_EnableRf(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel,
                                            bool enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable RF operation for a set of
 * specified channels on the ADRV9001 device, provided they are in SPI
 * enable mode. It transitions channels from the PRIMED state to the RF
 * ENABLED state when enabling, or from the RF ENABLED state to the
 * PRIMED state when disabling. The function must be called with channels
 * that are already in the PRIMED state for enabling, or in the RF
 * ENABLED state for disabling. It is important to ensure that the
 * channels are configured in SPI mode before calling this function, as
 * it will not operate correctly otherwise. This function is typically
 * used after channel initialization and configuration, and before
 * starting or stopping RF operations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param ports Array of ports corresponding to the channels to be enabled or
 * disabled. Each element must be a valid adi_common_Port_e value.
 * @param channels Array of channels corresponding to the ports to be enabled or
 * disabled. Each element must be a valid
 * adi_common_ChannelNumber_e value.
 * @param length Number of port/channel combinations to process. Must match the
 * length of the ports and channels arrays.
 * @param enable Boolean flag indicating whether to enable (true) or disable
 * (false) RF for the specified channels.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channels_EnableRf(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_Port_e ports[],
                                             adi_common_ChannelNumber_e channels[],
                                             uint32_t length,
                                             bool enable);

/***************************************************************************//**
 * @brief This function powers down a specified channel on the ADRV9001 device
 * while preserving its initial calibration information. It should be
 * called when the channel is in the CALIBRATED state. After powering
 * down, the channel will remain in the CALIBRATED state, and it must be
 * powered up again using the appropriate function before it can be
 * primed or used further. This function is useful for managing power
 * consumption and operational states of the device's channels.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, as it provides the context for the operation.
 * @param port The port of interest, specified as an enumeration of type
 * adi_common_Port_e. It identifies which port the channel belongs
 * to.
 * @param channel The channel of interest, specified as an enumeration of type
 * adi_common_ChannelNumber_e. It identifies which channel on the
 * specified port to power down.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_PowerDown(adi_adrv9001_Device_t *adrv9001,
                                             adi_common_Port_e port,
                                             adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief This function is used to power down a set of specified radio channels
 * on the ADRV9001 device, preserving their initial calibration
 * information. It should be called when the channels are in the
 * CALIBRATED state. After execution, the channels will remain in the
 * CALIBRATED state, and they must be powered up again before they can be
 * primed. This function is useful for managing power consumption and
 * operational states of the device's channels.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param ports Array of ports corresponding to the channels to be powered down.
 * Each index corresponds to a port/channel combination.
 * @param channels Array of channels corresponding to the ports to be powered
 * down. Each index corresponds to a port/channel combination.
 * @param length The number of port/channel combinations to power down. Must
 * match the length of the ports and channels arrays.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channels_PowerDown(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_Port_e ports[],
                                              adi_common_ChannelNumber_e channels[],
                                              uint32_t length);

/***************************************************************************//**
 * @brief This function powers up a specified radio channel on the ADRV9001
 * device, transitioning it to the CALIBRATED state while preserving
 * initial calibration information. It should be used when a channel
 * needs to be activated after being powered down, ensuring that the
 * channel is ready for further operations such as priming or enabling
 * RF. The function must be called with valid port and channel
 * identifiers, and the device context must be properly initialized.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device must be initialized.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port identifier.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * identifier.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_PowerUp(adi_adrv9001_Device_t *adrv9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief This function powers up the specified radio channels on the ADRV9001
 * device, transitioning them to the CALIBRATED state while preserving
 * initial calibration information. It should be used when channels need
 * to be activated after being powered down, ensuring they are ready for
 * further operations such as priming or enabling RF. The function
 * requires valid port and channel combinations, and the length parameter
 * must accurately reflect the number of these combinations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param ports Array of ports corresponding to the channels to be powered up.
 * Each index corresponds to a channel in the channels array. Must
 * not be null.
 * @param channels Array of channels to be powered up. Each index corresponds to
 * a port in the ports array. Must not be null.
 * @param length The number of port/channel combinations to power up. Must match
 * the length of the ports and channels arrays.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channels_PowerUp(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_Port_e ports[],
                                            adi_common_ChannelNumber_e channels[],
                                            uint32_t length);

/***************************************************************************//**
 * @brief Use this function to transition a specified channel to the CALIBRATED
 * state from any valid state. This is typically done to prepare the
 * channel for further operations that require it to be in the CALIBRATED
 * state. The function should be called when the channel is not in the
 * STANDBY state, as attempting to transition from STANDBY will result in
 * an error. If the channel is already in the CALIBRATED state, the
 * function will have no effect.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param port The port that the channel refers to. Must be a valid port
 * identifier.
 * @param channel The channel of the specified port to transition to the
 * CALIBRATED state. Must be a valid channel identifier.
 * @return Returns an integer code indicating success or the required action to
 * recover. If the channel is in STANDBY, an error is reported.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_ToCalibrated(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief Use this function to transition a specified channel to the PRIMED
 * state, which is necessary for enabling RF operations. The channel must
 * be in the CALIBRATED state before calling this function. If the
 * channel is in the STANDBY state, an error is reported, and the
 * function advises using the adi_adrv9001_InitCals_Run() function to
 * move to the CALIBRATED state first. This function does not perform any
 * action if the channel is already in the PRIMED state.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e
 * enumeration value.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enumeration value.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. If the channel is in the STANDBY state, an error code is
 * returned.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_ToPrimed(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_Port_e port,
                                            adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief Use this function to transition a specified channel on the ADRV9001
 * device to the RF ENABLED state. This function should be called when
 * the channel is in the CALIBRATED state, as it will first prime the
 * channel if necessary. If the channel is already in the RF ENABLED
 * state, the function will return without making changes. Ensure that
 * the channel is not in the STANDBY state before calling this function,
 * as it requires the channel to be at least in the CALIBRATED state.
 * This function is typically used after initialization and calibration
 * processes are complete, and the channel is ready for RF operation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port for the device.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * for the specified port.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. If the channel is in the STANDBY
 * state, an error is reported.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_ToRfEnabled(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_Port_e port,
                                               adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief This function is used to transition a specified channel on a given
 * port to a desired state, such as CALIBRATED, PRIMED, or RF ENABLED. It
 * should be called when a channel needs to change its operational state,
 * ensuring that the transition is valid from the current state. The
 * function performs necessary validations and expects the device context
 * to be properly initialized. It is important to handle the return value
 * to check for successful state transition or required recovery actions.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. Caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum value.
 * @param state The desired state to transition the channel to, specified as an
 * adi_adrv9001_ChannelState_e enum value. Must be a valid state
 * such as CALIBRATED, PRIMED, or RF ENABLED.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_Channel_ToState(adi_adrv9001_Device_t *adrv9001,
                                           adi_common_Port_e port,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_ChannelState_e state);

/***************************************************************************//**
 * @brief This function is used to configure the loop filter settings for a
 * specified PLL in the ADRV9001 device. It should be called when the
 * channel state is in STANDBY, typically before running initialization
 * calibrations. The function requires a valid device context, the PLL
 * identifier, and a configuration structure detailing the desired loop
 * filter settings. Proper validation of inputs is performed, and the
 * function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param pll The PLL identifier for which the loop filter is to be configured.
 * Must be a valid PLL enumeration value.
 * @param pllLoopFilterConfig Pointer to a structure containing the desired loop
 * filter configuration parameters. Must not be null.
 * The caller retains ownership.
 * @return Returns an integer status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_PllLoopFilter_Set(adi_adrv9001_Device_t *adrv9001,
                                             adi_adrv9001_Pll_e pll,
                                             adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig);

/***************************************************************************//**
 * @brief This function is used to obtain the current loop filter configuration
 * for a specified PLL in the ADRV9001 device. It should be called when
 * the channel state is any of STANDBY, CALIBRATED, PRIMED, or
 * RF_ENABLED. The function fills the provided configuration structure
 * with the current settings, allowing the caller to inspect the loop
 * filter parameters. It is important to ensure that the device context
 * and configuration pointers are valid before calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * Must not be null. The caller retains ownership.
 * @param pll The PLL of interest, specified as an adi_adrv9001_Pll_e
 * enumeration value. Must be a valid PLL identifier.
 * @param pllLoopFilterConfig A pointer to an adi_adrv9001_PllLoopFilterCfg_t
 * structure where the current loop filter
 * configuration will be stored. Must not be null.
 * The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_PllLoopFilter_Get(adi_adrv9001_Device_t *adrv9001,
                                             adi_adrv9001_Pll_e pll,
                                             adi_adrv9001_PllLoopFilterCfg_t *pllLoopFilterConfig);

/***************************************************************************//**
 * @brief This function is used to obtain the mailbox channel value
 * corresponding to a specific port and channel combination. It is useful
 * when interacting with the ADRV9001 device's mailbox system, which
 * requires channel identification in this format. The function expects
 * valid port and channel identifiers as inputs and returns a value that
 * represents the mailbox channel. This function should be used when a
 * precise mailbox channel value is needed for further operations or
 * configurations.
 *
 * @param port The port of interest, specified as an adi_common_Port_e
 * enumeration. It must be a valid port identifier for the ADRV9001
 * device.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enumeration. It must be a valid
 * channel identifier for the ADRV9001 device.
 * @return Returns a uint8_t value representing the mailbox channel
 * corresponding to the given port and channel combination.
 ******************************************************************************/
uint8_t adi_adrv9001_Radio_MailboxChannel_Get(adi_common_Port_e port, adi_common_ChannelNumber_e channel);

/***************************************************************************//**
 * @brief This function is used to convert a set of port and channel
 * combinations into a corresponding mailbox channel mask. It is useful
 * when you need to determine the channel mask for multiple port/channel
 * pairs. The function iterates over the provided arrays of ports and
 * channels, and constructs a mask that represents the specified
 * combinations. Ensure that the arrays are of the same length and that
 * the length parameter accurately reflects the number of elements in
 * these arrays.
 *
 * @param ports An array of ports, where each element specifies a port (e.g.,
 * ADI_RX, ADI_TX, ADI_ORX). The array must not be null and should
 * have at least 'length' elements.
 * @param channels An array of channels, where each element specifies a channel
 * (e.g., ADI_CHANNEL_1, ADI_CHANNEL_2). The array must not be
 * null and should have at least 'length' elements.
 * @param length The number of port/channel combinations to process. Must be a
 * non-negative integer and should not exceed the actual size of
 * the ports and channels arrays.
 * @return Returns a uint8_t value representing the mailbox channel mask for the
 * specified port/channel combinations.
 ******************************************************************************/
uint8_t adi_adrv9001_Radio_MailboxChannelMask_Get(adi_common_Port_e ports[],
                                                  adi_common_ChannelNumber_e channels[],
                                                  uint32_t length);

/***************************************************************************//**
 * @brief This function sets the enablement delays for a specified channel on
 * the ADRV9001 device. It should be called when the channel is in either
 * the STANDBY or CALIBRATED state. The function configures the timing
 * parameters that control how the channel transitions between different
 * states. It is important to ensure that the device context and delay
 * settings are correctly initialized before calling this function. The
 * function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum
 * value. Must be a valid port for the device.
 * @param channel The channel number of interest, specified as an
 * adi_common_ChannelNumber_e enum value. Must be a valid channel
 * for the specified port.
 * @param delays Pointer to a structure containing the desired delay settings.
 * Must not be null. The caller retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_ChannelEnablementDelays_Configure(adi_adrv9001_Device_t *adrv9001,
                                                             adi_common_Port_e port,
                                                             adi_common_ChannelNumber_e channel,
                                                             adi_adrv9001_ChannelEnablementDelays_t *delays);

/***************************************************************************//**
 * @brief This function retrieves the current enablement delay settings for a
 * specified channel on the ADRV9001 device. It should be used when the
 * channel is in either the PRIMED or RF_ENABLED state. The function
 * populates the provided delays structure with the current delay values,
 * allowing the caller to inspect the timing parameters associated with
 * channel enablement. This is useful for verifying or debugging the
 * timing configuration of the device.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e
 * enumeration value. Must be a valid port for the device.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enumeration value. Must be a valid
 * channel for the specified port.
 * @param delays Pointer to an adi_adrv9001_ChannelEnablementDelays_t structure
 * where the delay settings will be stored. Must not be null. The
 * caller provides the memory for this structure.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_ChannelEnablementDelays_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                           adi_common_Port_e port,
                                                           adi_common_ChannelNumber_e channel,
                                                           adi_adrv9001_ChannelEnablementDelays_t *delays);

/***************************************************************************//**
 * @brief This function transitions all channels from the CALIBRATED state to
 * the MCS ready substate. It should be called when all channels are in
 * the CALIBRATED state and ready to be moved to the MCS_READY substate.
 * This is typically part of a multi-chip synchronization process. The
 * function ensures that the necessary command is sent to the device and
 * waits for the operation to complete before returning. It is important
 * to ensure that the device context is properly initialized and that the
 * channels are in the correct state before calling this function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure containing settings. Must not be null. The function
 * will perform validation on this parameter and expects it to
 * be properly initialized.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_ToMcsReady(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to obtain the Logen Divider value for a
 * specified PLL (either RF1 or RF2) in the ADRV9001 device. It should be
 * called when the channel state is any of CALIBRATED, PRIMED, or
 * RF_ENABLED. The Logen Divider is calculated based on the RF Logen
 * divide mode and ratio, and the result is stored in the provided output
 * parameter. This function is useful for applications that need to
 * understand or verify the frequency division settings of the RF PLLs.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, as it provides the context for the operation.
 * @param pll The PLL of interest, specified as an enum value of type
 * adi_adrv9001_Pll_e. Valid values are those representing RF1 and
 * RF2 PLLs.
 * @param RfLogenDivider Pointer to a uint32_t where the calculated Logen
 * Divider value will be stored. Must not be null. The
 * function will write the result to this location.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The calculated Logen Divider value
 * is output through the RfLogenDivider parameter.
 ******************************************************************************/
int32_t adi_adrv9001_Radio_RfLogenDivider_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_Pll_e pll, uint32_t *RfLogenDivider);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RADIO_H_ */

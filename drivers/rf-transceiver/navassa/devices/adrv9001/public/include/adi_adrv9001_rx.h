/**
 * \file
 * \brief Functions for configuring receiver (Rx) features
 *
 * Copyright 2015 - 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RX_H_
#define _ADI_ADRV9001_RX_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_common_error_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to load or reconfigure the gain table for
 * specified Rx channels. It should be called after device initialization
 * and before running initialization calibrations if a custom gain table
 * is needed. The function allows for partial gain table loads, which can
 * be useful in scenarios with memory constraints or when loading
 * multiple regions. Users must ensure that the gain table rows and
 * indices are correctly specified, and that the device context is
 * properly initialized before calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param port The desired Rx or ORx port. Must be a valid port enumeration
 * value.
 * @param channel The Rx channel for which to write the gain table. Must be a
 * valid channel enumeration value.
 * @param gainIndexOffset The starting gain index from which the gain table is
 * written. Must be within the valid range of gain
 * indices.
 * @param gainTableRows Array of gain table row entries to write. Must not be
 * null and should have at least 'arraySize' elements.
 * @param arraySize The number of gain table rows to write. Must be greater than
 * zero and within the limits of the gain table size.
 * @param lnaConfig Pointer to the desired LNA configuration. Must not be null
 * if external LNA is present.
 * @param gainTableType The type of gain table to load. Must be a valid gain
 * table type enumeration value.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainTable_Write(adi_adrv9001_Device_t *adrv9001,
                                        adi_common_Port_e port,
                                        adi_common_ChannelNumber_e channel,
                                        uint8_t  gainIndexOffset,
                                        adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                        uint32_t arraySize,
                                        adi_adrv9001_RxLnaConfig_t *lnaConfig,
                                        adi_adrv9001_RxGainTableType_e gainTableType);

/***************************************************************************//**
 * @brief Use this function to retrieve the current gain table settings from the
 * ADRV9001 device for a specified Rx channel. It reads the gain table
 * entries starting from a given gain index offset and stores them in the
 * provided array. This function can be called anytime after the device
 * has been initialized. It is useful for verifying or inspecting the
 * gain table configuration. Ensure that the provided array is large
 * enough to hold the desired number of gain table entries. If the actual
 * number of gain indices read is not needed, you can pass NULL for the
 * numGainIndicesRead parameter.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel The Rx channel from which to read the gain table. Must be a
 * valid channel number.
 * @param gainIndexOffset The gain index from which the gain table read should
 * start. Must be within the valid range of gain indices.
 * @param gainTableRows Array to store the read gain table row entries. Must not
 * be null and should have enough space to store the
 * entries.
 * @param arraySize The size of the gainTableRows array, indicating the maximum
 * number of gain table rows to read. Must be greater than
 * zero.
 * @param numGainIndicesRead Pointer to store the actual number of gain indices
 * read. Can be null if this information is not
 * needed.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainTable_Read(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_ChannelNumber_e channel,
                                       uint8_t  gainIndexOffset,
                                       adi_adrv9001_RxGainTableRow_t gainTableRows[],
                                       uint32_t arraySize,
                                       uint16_t *numGainIndicesRead);

/***************************************************************************//**
 * @brief Use this function to manually set the gain index for a specific Rx
 * channel on the ADRV9001 device. This function is applicable when the
 * gain control mode is set to SPI. The gain index must be within the
 * valid range defined by the gain table's minimum and maximum indices.
 * If the provided gain index is out of range, the function will return
 * an error. The new gain setting will only take effect when the channel
 * is in a state where clocks are enabled, such as during transitions to
 * PRIMED or RF_ENABLED states.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to set the gain. Must be a valid
 * channel number as defined by adi_common_ChannelNumber_e.
 * @param gainIndex The gain table index to set for the specified channel. Must
 * be within the valid range of the gain table's minimum and
 * maximum indices.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_Gain_Set(adi_adrv9001_Device_t *adrv9001,
                                 adi_common_ChannelNumber_e channel,
                                 uint8_t gainIndex);

/***************************************************************************//**
 * @brief Use this function to obtain the current Automatic Gain Control (AGC)
 * gain index for a specified Rx channel. It is useful for monitoring the
 * gain settings applied to the channel. This function should be called
 * when the channel is in any of the following states: STANDBY,
 * CALIBRATED, PRIMED, or RF_ENABLED. However, note that gain indices are
 * only tracked after the channel state is RF_ENABLED. The function
 * returns the gain index as it was the last time the clocks were
 * enabled.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel from which to read the gain. Must be a valid
 * channel number as defined by adi_common_ChannelNumber_e.
 * @param gainIndex Pointer to a uint8_t where the current gain index will be
 * stored. Must not be null. The function writes the gain index
 * to this location.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_Gain_Get(adi_adrv9001_Device_t *adrv9001,
                                 adi_common_ChannelNumber_e channel,
                                 uint8_t *gainIndex);

/***************************************************************************//**
 * @brief Use this function to obtain the Received Signal Strength Indicator
 * (RSSI) for a specific Rx channel. It should be called when the channel
 * state is RF_ENABLED. This function provides the RSSI measurement in
 * milli-decibels (mdB) and is useful for monitoring signal strength.
 * Ensure that the device is properly initialized and the channel is in
 * the correct state before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized.
 * @param channel The Rx channel for which the RSSI status is to be read. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param rxRssiPower_mdB Pointer to a uint32_t where the measured Rx RSSI power
 * will be stored, in milli-decibels (mdB). Must not be
 * null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_Rssi_Read(adi_adrv9001_Device_t *adrv9001,
                                  adi_common_ChannelNumber_e channel,
                                  uint32_t *rxRssiPower_mdB);

/***************************************************************************//**
 * @brief Use this function to obtain the decimated power measurement of a
 * specified Rx channel in millidecibels full scale (mdBFS). It is
 * suitable for runtime power monitoring of the Rx channel. The function
 * requires a valid device context and channel number, and it outputs the
 * power measurement to a provided pointer. The resolution of the
 * measurement is 250 mdB, and the dynamic range is 60 dB. If the
 * receiver is disabled during measurement, a value of 200000 mdBFS is
 * returned. Ensure the channel is within the valid range before calling
 * this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel Enum value of type adi_common_ChannelNumber_e specifying the
 * Rx channel. Must be ADI_CHANNEL_1 or ADI_CHANNEL_2.
 * @param rxDecPower_mdBFS Pointer to a uint16_t where the decimated power in
 * mdBFS will be stored. Must not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. On success, rxDecPower_mdBFS is updated with the decimated
 * power value.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_DecimatedPower_Get(adi_adrv9001_Device_t *adrv9001,
                                           adi_common_ChannelNumber_e channel,
                                           uint16_t *rxDecPower_mdBFS);

/***************************************************************************//**
 * @brief Use this function to set the desired Rx interface gain control
 * configuration for a specific channel on the ADRV9001 device. This
 * function should be called when the channel is in the CALIBRATED state.
 * It is essential for configuring the gain control parameters, which can
 * affect the performance of the receiver. Ensure that the device and
 * channel parameters are valid and that the configuration structure is
 * properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized.
 * @param channel The Rx channel for which to configure the interface gain
 * control. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param rxInterfaceGainConfig Pointer to the desired Rx interface gain control
 * configuration structure. Must not be null and
 * should be properly initialized with the desired
 * settings.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_Configure(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                adi_adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainConfig);

/***************************************************************************//**
 * @brief This function sets the interface gain for a specified Rx channel on
 * the ADRV9001 device. It should be called when the channel is in the
 * RF_ENABLED state and the gain control mode is set to manual. This
 * function is useful for adjusting the gain settings of the receiver
 * interface to optimize signal reception. Ensure that the device and
 * channel are properly initialized and configured before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which the interface gain is to be set. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param gain The gain value to be set for the specified Rx channel. Must be a
 * valid value as defined by adi_adrv9001_RxInterfaceGain_e.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_Set(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_RxInterfaceGain_e gain);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the Rx
 * interface gain control for a specified channel. It provides details
 * about the gain control mode, update timing, and other related
 * settings. This function is useful for verifying the current gain
 * control settings and ensuring they are configured as expected. It must
 * be called when the channel is in one of the following states:
 * CALIBRATED, PRIMED, or RF_ENABLED.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel The Rx channel for which to inspect the interface gain control
 * configuration. Must be a valid channel number.
 * @param rxInterfaceGainConfig Pointer to a structure where the current Rx
 * interface gain control configuration will be
 * stored. Must not be null.
 * @param gainTableType Pointer to a variable where the current gain table type
 * will be stored. Must not be null.
 * @return Returns an integer code indicating success or the required action to
 * recover. The output parameters are updated with the current
 * configuration values.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_Inspect(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_RxInterfaceGainCtrl_t *rxInterfaceGainConfig,
                                              adi_adrv9001_RxGainTableType_e *gainTableType);

/***************************************************************************//**
 * @brief Use this function to obtain the current interface gain setting for a
 * specific Rx channel on the ADRV9001 device. It is essential to ensure
 * that the channel is in one of the following states before calling this
 * function: CALIBRATED, PRIMED, or RF_ENABLED. This function is useful
 * for monitoring or debugging purposes to verify the gain settings
 * applied to the channel.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null.
 * @param channel An enum value of type adi_common_ChannelNumber_e specifying
 * the Rx channel from which to read the interface gain. Must be
 * a valid channel number.
 * @param gain A pointer to an adi_adrv9001_RxInterfaceGain_e variable where the
 * current Rx interface gain will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_Get(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_RxInterfaceGain_e *gain);
/***************************************************************************//**
 * @brief This function sets the seed gain for the Rx interface gain, which will
 * be applied at the rising edge of the associated GPIO for the next
 * frame. It should be used when the channel is in any of the CALIBRATED,
 * PRIMED, or RF_ENABLED states. The function is applicable only when the
 * control mode is set to
 * ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC. It is important to
 * ensure that the device and channel parameters are valid before calling
 * this function.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized.
 * @param channel The Rx channel for which the interface gain is to be
 * configured. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param seedGain The gain value to be seeded for the next frame. Must be a
 * valid value of type adi_adrv9001_RxInterfaceGain_e.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_SeedGain_Set(adi_adrv9001_Device_t *adrv9001,
	                                                adi_common_ChannelNumber_e channel,
	                                                adi_adrv9001_RxInterfaceGain_e seedGain);

/***************************************************************************//**
 * @brief Use this function to obtain the current seed gain value for the
 * specified Rx channel, which will be applied at the next frame by a
 * rising edge on the associated GPIO. This function is applicable when
 * the channel is in any of the CALIBRATED, PRIMED, or RF_ENABLED states.
 * It is important to ensure that the device is properly initialized and
 * the channel is in a valid state before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel The Rx channel from which to read the interface gain. Must be
 * a valid channel number.
 * @param seedGain Pointer to a variable where the seed gain value will be
 * stored. Must not be null.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_SeedGain_Get( adi_adrv9001_Device_t *adrv9001,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_RxInterfaceGain_e *seedGain);

/***************************************************************************//**
 * @brief Use this function to obtain the gain value at the end of the previous
 * frame for a specified Rx channel. This value is latched by a falling
 * edge on the associated GPIO. The function should be called when the
 * channel is in one of the following states: CALIBRATED, PRIMED, or
 * RF_ENABLED. It is important to ensure that the device context and
 * channel parameters are valid before calling this function.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device context. Must not be null.
 * @param channel An enum value of type adi_common_ChannelNumber_e specifying
 * the Rx channel from which to read the interface gain. Valid
 * values are typically ADI_CHANNEL_1 or ADI_CHANNEL_2.
 * @param endOfFrameGain A pointer to an adi_adrv9001_RxInterfaceGain_e variable
 * where the end-of-frame gain will be stored. Must not be
 * null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_InterfaceGain_EndOfFrameGain_Get(   adi_adrv9001_Device_t *adrv9001,
	                                                        adi_common_ChannelNumber_e channel,
	                                                        adi_adrv9001_RxInterfaceGain_e *endOfFrameGain);
	
/***************************************************************************//**
 * @brief Use this function to adjust the NCO frequency for a specified Rx
 * channel to correct small deviations in the local oscillator (LO)
 * frequency. This function is typically called when there is a need to
 * fine-tune the frequency offset for a channel. The frequency correction
 * can be applied immediately or at the start of the next available
 * frame, depending on the 'immediate' parameter. Ensure that the device
 * context is properly initialized before calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be null
 * and should be properly initialized before use.
 * @param channel The Rx channel for which to set the NCO frequency. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param frequencyOffset_Hz The desired frequency offset in Hertz. This value
 * is used to adjust the NCO frequency.
 * @param immediate Boolean flag indicating whether the frequency change should
 * be applied immediately (true) or at the start of the next
 * available frame (false).
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_FrequencyCorrection_Set(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                int32_t frequencyOffset_Hz,
                                                bool immediate);


/***************************************************************************//**
 * @brief This function enables or disables the dynamic switching between Low
 * Power and High Power ADCs for a specified Rx channel. It should be
 * called when the channel is in either the STANDBY or CALIBRATED state.
 * This function is useful for optimizing power consumption based on the
 * operational requirements of the channel. Ensure that the device
 * context and channel are correctly specified before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx Channel for which to set the enabledness of the ADC
 * dynamic switch. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param enable A boolean flag indicating whether to enable (true) or disable
 * (false) dynamic switching between Low Power and High Power
 * ADCs.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_AdcSwitchEnable_Set(adi_adrv9001_Device_t *adrv9001, 
                                            adi_common_ChannelNumber_e channel,
                                            bool enable);

/***************************************************************************//**
 * @brief This function is used to check whether the dynamic switching between
 * Low Power and High Power ADCs is enabled for a specific Rx channel. It
 * should be called when the channel is in any of the states: STANDBY,
 * CALIBRATED, PRIMED, or RF_ENABLED. The function provides the current
 * enabled state through a boolean output parameter. It is useful for
 * verifying the configuration of ADC dynamic switching in the system.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param channel The Rx Channel for which to get the current enabledness of the
 * ADC dynamic switch. Must be a valid channel number as defined
 * by adi_common_ChannelNumber_e.
 * @param enable Pointer to a boolean where the current enabled state of the ADC
 * dynamic switch will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_AdcSwitchEnable_Get(adi_adrv9001_Device_t *adrv9001, 
                                            adi_common_ChannelNumber_e channel,
                                            bool *enable);

/***************************************************************************//**
 * @brief Use this function to configure the ADC dynamic switch settings for a
 * specified Rx channel on the ADRV9001 device. This function should be
 * called when the channel state is CALIBRATED. It allows the user to set
 * the desired configuration for dynamic switching between low power and
 * high power ADCs, which can optimize power consumption and performance
 * based on the application's requirements. Ensure that the device and
 * channel are properly initialized and in the correct state before
 * calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to configure the ADC dynamic switch.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param switchConfig Pointer to the desired ADC dynamic switch configuration.
 * Must not be null. The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_AdcSwitch_Configure(adi_adrv9001_Device_t *adrv9001, 
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_AdcSwitchCfg_t *switchConfig);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the ADC
 * dynamic switch for a specified Rx channel. It is useful for verifying
 * the current ADC switch settings during various operational states.
 * This function can be called when the channel is in any of the
 * following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. Ensure
 * that the device context and channel are correctly specified before
 * calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx Channel for which to inspect the ADC dynamic switch
 * settings. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param switchConfig Pointer to an adi_adrv9001_AdcSwitchCfg_t structure where
 * the current ADC dynamic switch configuration will be
 * stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The switchConfig parameter is
 * populated with the current ADC dynamic switch settings.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_AdcSwitch_Inspect(adi_adrv9001_Device_t *adrv9001, 
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_AdcSwitchCfg_t *switchConfig);

/***************************************************************************//**
 * @brief Use this function to obtain the current ADC type for a given channel
 * on the ADRV9001 device. It is applicable when the channel is in any of
 * the following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. This
 * function is useful for checking the ADC configuration during runtime
 * to ensure the correct ADC type is being used for the channel.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param channel The channel number for which the ADC type is to be retrieved.
 * It must be a valid channel number as defined by the
 * adi_common_ChannelNumber_e enumeration.
 * @param adcType Pointer to a variable where the current ADC type will be
 * stored. Must not be null, as it is used to return the result
 * of the function.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error. The adcType parameter is updated with the
 * current ADC type on success.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_AdcType_Get(adi_adrv9001_Device_t *adrv9001, 
                                    adi_common_ChannelNumber_e channel,
                                    adi_adrv9001_AdcType_e *adcType);

/***************************************************************************//**
 * @brief This function configures the GPIO pins to route the gain indices for
 * the specified Rx channel on the ADRV9001 device. It should be called
 * when the channel state is CALIBRATED, and the GPIO pin levels will
 * reflect the channel gain index only when the channel is in the
 * RF_ENABLED state. This function is useful for applications that
 * require external monitoring or control of the gain index via GPIO.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param channel The Rx channel for which to configure the GPIO routing. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param gainIndexPinCfgchannel Pointer to the desired GPIO pin configuration
 * for routing the gain index. Must not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_GainIndex_Gpio_Configure(adi_adrv9001_Device_t *adrv9001,
                                                 adi_common_ChannelNumber_e channel,
                                                 adi_adrv9001_GainIndexPinCfg_t *gainIndexPinCfg);

/***************************************************************************//**
 * @brief This function is used to configure the RX port switching settings for
 * the ADRV9001 device. It should be called when the device is in the
 * STANDBY state to ensure proper configuration. The function takes a
 * configuration structure that specifies the desired settings for the RX
 * port switching, including frequency ranges and enabling options.
 * Proper validation of the input parameters is performed, and the
 * function returns a status code indicating success or the required
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param switchConfig Pointer to the RX port switch configuration structure.
 * Must not be null. The structure should be properly
 * initialized with the desired configuration settings
 * before calling this function.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_PortSwitch_Configure(adi_adrv9001_Device_t *adrv9001, 
                                             adi_adrv9001_RxPortSwitchCfg_t *switchConfig);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the RX port
 * switching for a specified ADRV9001 device. It is useful for verifying
 * the current settings of the RX port switch, especially after
 * configuration changes. This function can be called when the channel
 * state is any of STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. Ensure
 * that the device and switchConfig pointers are valid before calling
 * this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param switchConfig Pointer to a structure where the current RX port switch
 * configuration will be stored. Must not be null. The
 * caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_PortSwitch_Inspect(adi_adrv9001_Device_t *adrv9001, 
                                           adi_adrv9001_RxPortSwitchCfg_t *switchConfig);

/***************************************************************************//**
 * @brief This function configures the external Low Noise Amplifier (LNA) for a
 * specified RX channel on the ADRV9001 device. It should be called when
 * the channel is in the STANDBY state. The function sets up the LNA
 * configuration based on the provided parameters, including the gain
 * table type. Proper configuration of the LNA is essential for optimal
 * receiver performance, especially in environments with varying signal
 * strengths. This function must be used with care to ensure that the LNA
 * settings are compatible with the overall system design and
 * requirements.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param channel The RX channel to configure, specified as an enum of type
 * adi_common_ChannelNumber_e. Must be a valid channel number for
 * the device.
 * @param lnaConfig Pointer to the desired LNA configuration structure of type
 * adi_adrv9001_RxLnaConfig_t. Must not be null and should be
 * properly populated with the desired configuration settings.
 * @param gainTableType The gain table type to be used, specified as an enum of
 * type adi_adrv9001_RxGainTableType_e. Determines the gain
 * table settings applied during configuration.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_ExternalLna_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_RxLnaConfig_t *lnaConfig,
                                              adi_adrv9001_RxGainTableType_e gainTableType);

/***************************************************************************//**
 * @brief This function configures the digital gain delay for the external Low
 * Noise Amplifier (LNA) on a specified RX channel. It should be used
 * when the channel is in either the STANDBY or CALIBRATED state. The
 * function adjusts the gain delay by a fixed value to ensure proper
 * timing alignment. It is important to ensure that the device is
 * properly initialized and the channel is in the correct state before
 * calling this function to avoid unexpected behavior.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized.
 * @param channel The RX channel to configure, specified as an enum of type
 * adi_common_ChannelNumber_e. Must be a valid channel number.
 * @param lnaDigitalGainDelay The desired LNA digital gain delay, specified as a
 * uint16_t. The value is adjusted internally by a
 * fixed amount.
 * @param gainTableType The type of gain table loaded during ADRV9001
 * initialization, specified as an enum of type
 * adi_adrv9001_RxGainTableType_e. Must be a valid gain
 * table type.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_ExternalLna_DigitalGainDelay_Set(adi_adrv9001_Device_t *adrv9001,
                                                         adi_common_ChannelNumber_e channel,
                                                         uint16_t lnaDigitalGainDelay,
                                                         adi_adrv9001_RxGainTableType_e gainTableType);

/***************************************************************************//**
 * @brief Use this function to obtain the current digital gain delay setting for
 * the external LNA of a specified RX channel. It is applicable in any
 * channel state, including STANDBY, CALIBRATED, PRIMED, and RF_ENABLED.
 * This function is useful for verifying the current configuration of the
 * LNA digital gain delay, which can be critical for ensuring optimal
 * signal processing and gain control in the receiver path.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the device operation.
 * @param channel Specifies the RX channel for which the LNA digital gain delay
 * is to be retrieved. Must be a valid channel number as defined
 * by adi_common_ChannelNumber_e.
 * @param lnaDigitalGainDelay Pointer to a uint16_t where the current LNA
 * digital gain delay will be stored. Must not be
 * null, as it is used to return the delay value to
 * the caller.
 * @return Returns an int32_t code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_ExternalLna_DigitalGainDelay_Get(adi_adrv9001_Device_t *adrv9001,
                                                         adi_common_ChannelNumber_e channel,
                                                         uint16_t *lnaDigitalGainDelay);
	
/***************************************************************************//**
 * @brief Use this function to set the LOID (Local Oscillator Interference
 * Detection) configuration for a specific Rx channel on the ADRV9001
 * device. This function should be called when the channel is in either
 * the STANDBY or CALIBRATED state. It is essential for setting up the
 * LOID parameters, which include enabling the feature and setting the
 * threshold levels. Proper configuration of these settings is crucial
 * for effective interference detection.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should be properly initialized before calling
 * the function. The caller retains ownership.
 * @param channel An enumeration value of type adi_common_ChannelNumber_e
 * specifying the Rx channel to configure. Must be a valid
 * channel number supported by the device.
 * @param loidConfig A pointer to an adi_adrv9001_RxrfdcLoidCfg_t structure
 * containing the desired LOID configuration settings. This
 * must not be null and should be properly populated with
 * valid configuration data before calling the function. The
 * caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_Loid_Configure(adi_adrv9001_Device_t *adrv9001,
                                   adi_common_ChannelNumber_e channel,
		                           adi_adrv9001_RxrfdcLoidCfg_t *loidConfig);


/***************************************************************************//**
 * @brief Use this function to retrieve the current LOID (Local Oscillator ID)
 * configuration for a specified Rx channel on the ADRV9001 device. This
 * function is useful for verifying the LOID settings after they have
 * been configured. It can be called when the channel is in any of the
 * following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. Ensure
 * that the device context and channel are correctly specified before
 * calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to inspect the LOID settings. Must be
 * a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param loidConfig Pointer to a structure where the current LOID configuration
 * will be stored. Must not be null. The function will
 * populate this structure with the LOID settings.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Rx_Loid_Inspect(adi_adrv9001_Device_t *adrv9001,
                                   adi_common_ChannelNumber_e channel,
		                           adi_adrv9001_RxrfdcLoidCfg_t *loidConfig);
	
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_H_ */

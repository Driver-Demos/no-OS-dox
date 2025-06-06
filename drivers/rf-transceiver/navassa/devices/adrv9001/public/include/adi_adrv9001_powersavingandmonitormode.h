/**
 * \file
 * \brief Contains ADRV9001 Power saving and Monitor mode related function prototypes for adi_adrv9001_PowerSavingAndMonitorMode.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_H_
#define _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_H_

#include "adi_adrv9001_powersavingandmonitormode_types.h"
#include "adi_common_error_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to set the power saving configuration for a
 * specific channel on the ADRV9001 device. It should be called when the
 * device's Rx and Tx channels are in any of the CALIBRATED, PRIMED, or
 * RF_ENABLE states. The function sends a high-priority mailbox command
 * to the device to apply the specified power saving settings. It is
 * important to ensure that the device context and configuration
 * parameters are valid before calling this function to avoid errors.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before use.
 * @param channel The channel number for which to configure the power saving
 * settings. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param powerSavingCfg Pointer to the desired power saving configuration
 * structure. Must not be null and should be properly
 * populated with the desired settings before calling the
 * function.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_ChannelPowerSaving_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                            adi_common_ChannelNumber_e channel,
                                                                            adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerSavingCfg_t *powerSavingCfg);

/***************************************************************************//**
 * @brief Use this function to retrieve the current power saving configuration
 * for a specific channel on the ADRV9001 device. It is essential to
 * ensure that both Rx and Tx channels are in one of the CALIBRATED,
 * PRIMED, or RF_ENABLE states before calling this function. This
 * function provides the current power saving settings by writing them
 * into the provided configuration structure. It is useful for verifying
 * the current power saving state of a channel.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param channel The channel number for which to inspect the power saving
 * settings. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param powerSavingCfg Pointer to a structure where the current power saving
 * configuration will be stored. Must not be null. The
 * function writes the current configuration into this
 * structure.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_ChannelPowerSaving_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                                          adi_common_ChannelNumber_e channel,
                                                                          adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerSavingCfg_t *powerSavingCfg);

/***************************************************************************//**
 * @brief This function is used to set up the monitor mode configuration for all
 * channels on the ADRV9001 device. It should be called when the device
 * is in any of the STANDBY, CALIBRATED, PRIMED, or RF_ENABLE states, and
 * after the ADI_ADRV9001_GPIO_SIGNAL_MON_ENABLE_SPS signal has been
 * assigned to a pin using adi_adrv9001_gpio_Configure. The function
 * sends configuration data to the device via a high-priority mailbox
 * command, ensuring that the monitor mode settings are applied
 * correctly. It is important to ensure that the device is in the correct
 * state before calling this function to avoid unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeCfg Pointer to the desired monitor mode configuration
 * structure. Must not be null. The caller retains
 * ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_SystemPowerSavingAndMonitorMode_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                                         adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerSavingAndMonitorModeCfg_t *monitorModeCfg);

/***************************************************************************//**
 * @brief This function retrieves the current monitor mode configuration for all
 * channels in the ADRV9001 device. It should be called when the user
 * needs to verify or log the current monitor mode settings. The function
 * requires that all channels are in one of the STANDBY, CALIBRATED,
 * PRIMED, or RF_ENABLE states before it is called. It outputs the
 * current configuration into the provided structure, allowing the caller
 * to examine the settings.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeCfg Pointer to a structure where the current monitor mode
 * configuration will be stored. Must not be null. The
 * caller provides this structure and retains ownership.
 * @return Returns an integer code indicating success or the required action to
 * recover. The monitorModeCfg structure is populated with the current
 * settings on success.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_SystemPowerSavingAndMonitorMode_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                                                       adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerSavingAndMonitorModeCfg_t *monitorModeCfg);

/***************************************************************************//**
 * @brief This function configures the system power saving mode for the ADRV9001
 * device by setting the desired power down mode. It is a high-priority
 * mailbox command and should be used when you need to adjust the power
 * saving settings of the entire system. Ensure that the ADRV9001 device
 * context is properly initialized before calling this function. The
 * function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized
 * before use. The caller retains ownership.
 * @param mode The desired system power saving power down mode, specified as an
 * enumeration of type
 * adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e. The
 * function expects a valid mode value; invalid values may result in
 * undefined behavior.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_SystemPowerSavingMode_Set(adi_adrv9001_Device_t *adrv9001,
                                                                         adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current power saving mode of the
 * ADRV9001 device. It should be called when you need to check the power
 * saving state of the system. The function requires a valid device
 * context and a pointer to store the retrieved mode. It returns a status
 * code indicating success or the necessary action to recover from an
 * error.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * Must not be null. The caller retains ownership.
 * @param mode A pointer to a variable where the current system power saving
 * mode will be stored. Must not be null. The function writes the
 * current mode to this location.
 * @return Returns an integer status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_SystemPowerSavingMode_Get(adi_adrv9001_Device_t *adrv9001,
                                                                         adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e *mode);

/***************************************************************************//**
 * @brief This function is used to configure the monitor mode pattern for the
 * ADRV9001 device. It loads a specified pattern into the device's input
 * FIFO and verifies each pattern by reading it back to ensure it is
 * correctly written. This function should be called after preparing the
 * device with
 * adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Prepare. It
 * is important to ensure that the device is in a suitable state before
 * calling this function to avoid errors. The function returns a code
 * indicating success or the required action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModePattern Pointer to the desired monitor mode pattern
 * configuration. Must not be null. The pattern length
 * should not exceed the FIFO capacity.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Pattern_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                             adi_adrv9001_PowerSavingAndMonitorMode_MonitorModePatternCfg_t *monitorModePattern);

/***************************************************************************//**
 * @brief This function sets up the monitor mode vector configuration for the
 * ADRV9001 device, allowing for specific monitoring behaviors to be
 * defined. It should be used when the device is in a state that allows
 * direct register access, and the desired monitor mode vector
 * configuration is ready to be applied. The function requires a valid
 * device context and a properly initialized monitor mode vector
 * configuration structure. It returns a status code indicating success
 * or the necessary action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeVector Pointer to the desired monitor mode vector
 * configuration. Must not be null and should be
 * properly initialized before calling the function.
 * The caller retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Vector_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                            adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg_t *monitorModeVector);

/***************************************************************************//**
 * @brief This function is used to inspect the current monitor mode vector
 * settings of the ADRV9001 device. It should be called when you need to
 * retrieve the current configuration of the monitor mode vector. The
 * function requires a valid device context and a pointer to a
 * configuration structure where the monitor mode vector will be stored.
 * Ensure that the device is properly initialized and in a suitable state
 * before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeVector Pointer to a structure where the current monitor
 * mode vector will be stored. Must not be null. The
 * function will populate this structure with the
 * current settings.
 * @return Returns an integer code indicating success or the required action to
 * recover. The monitorModeVector structure is populated with the
 * current settings on success.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Vector_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                                          adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg_t *monitorModeVector);

/***************************************************************************//**
 * @brief This function sets the RSSI configuration for the monitor mode of the
 * ADRV9001 device. It should be called when the channel is in STANDBY,
 * CALIBRATED, or PRIMED state. The function configures parameters such
 * as the number of measurements to average, the start period for
 * measurements, the duration of each measurement, and the detection
 * threshold. It is important to ensure that the device is properly
 * initialized and in a valid state before calling this function to avoid
 * unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeRssiCfg Pointer to the desired monitor mode RSSI
 * configuration settings. Must not be null. The
 * caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Rssi_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                          adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg_t *monitorModeRssiCfg);

/***************************************************************************//**
 * @brief This function retrieves the current configuration settings for the
 * monitor mode RSSI of the ADRV9001 device. It should be called when the
 * channel is in any of the STANDBY, CALIBRATED, PRIMED, or RF_ENABLED
 * states. The function populates the provided configuration structure
 * with the current settings, allowing the caller to inspect the RSSI
 * configuration. It is important to ensure that the device context and
 * configuration structure pointers are valid before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param monitorModeRssiCfg Pointer to a structure where the current monitor
 * mode RSSI configuration will be stored. Must not be
 * null. The caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Rssi_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                                        adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg_t *monitorModeRssiCfg);

/***************************************************************************//**
 * @brief This function is used to prepare the ADRV9001 device for running the
 * Rx DMR path delay calibration. It must be called before configuring
 * the monitor mode pattern using
 * adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_Pattern_Configure.
 * The function operates on Rx channel 1 and requires the channel state
 * to be in the CALIBRATED state. It sends a high-priority mailbox
 * command to the device to set up the necessary configuration for the
 * calibration process.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership of the memory.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Prepare(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function triggers the Rx DMR Path Delay calibration for the
 * ADRV9001 device, which is used to find the peak correlation between a
 * waveform pattern and a sync vector. It should be called when the
 * channel state is CALIBRATED, and after the pattern and vector have
 * been loaded. The function operates on Rx channel 1 and requires a
 * timeout value to specify how long to wait for the calibration to
 * complete. If the calibration fails to find a sync, the timeout will
 * apply. The function also provides an error flag to indicate any issues
 * during the initial calibrations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param timeout_ms Timeout value in milliseconds to wait for the calibration
 * to complete. A typical value is 1000 ms.
 * @param initCalsError Pointer to a 3-bit error flag that indicates errors
 * during initial calibrations. '0' indicates no error.
 * Must not be null.
 * @return Returns a code indicating success or the required action to recover.
 * The initCalsError parameter is updated with any calibration errors.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Run(adi_adrv9001_Device_t *adrv9001,
                                                                       uint32_t timeout_ms,
                                                                       uint8_t *initCalsError);

/***************************************************************************//**
 * @brief This function is used to calibrate the Rx DMR path delay for the
 * ADRV9001 device in monitor mode. It should be called after preparing
 * the device and configuring the monitor mode pattern and vector. The
 * function waits for the calibration to complete within a specified
 * timeout and provides error information and the path delay result. It
 * is essential to ensure that the device is in the CALIBRATED state
 * before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param monitorModePattern Pointer to the desired monitor mode pattern
 * configuration. Must not be null.
 * @param monitorModeVector Pointer to the desired monitor mode vector
 * configuration. Must not be null.
 * @param timeout_ms Timeout in milliseconds to wait for the calibration to
 * complete. A typical value is 1000 ms.
 * @param initCalsError Pointer to a 3-bit error flag that indicates any errors
 * during initial calibrations. Must not be null.
 * @param pathDelay Pointer to store the result of the DMR path delay
 * calibration, ranging from 0 to 2047 samples. Must not be
 * null.
 * @return Returns a code indicating success or the required action to recover.
 * The path delay and any calibration errors are output through the
 * provided pointers.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Calibrate(adi_adrv9001_Device_t *adrv9001,
                                                                             adi_adrv9001_PowerSavingAndMonitorMode_MonitorModePatternCfg_t *monitorModePattern,
                                                                             adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg_t *monitorModeVector,
                                                                             uint32_t timeout_ms,
                                                                             uint8_t *initCalsError,
                                                                             uint32_t *pathDelay);

/***************************************************************************//**
 * @brief This function is used to obtain the Rx DMR path delay, which is the
 * latency of the ADRV9001 receiver when detecting a sync symbol as per
 * the Digital Mobile Radio (DMR) standard. It should be called after
 * successfully running the Rx DMR Path Delay calibration using
 * adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Run. The
 * function operates on Rx channel 1 and requires the channel to be in a
 * CALIBRATED state. The result is used to configure the Monitor DMR
 * Search.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param pathDelay Pointer to a uint32_t where the function will store the
 * result of the DMR path delay calibration, ranging from 0 to
 * 2047 samples. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_RxDmrPd_Get(adi_adrv9001_Device_t *adrv9001,
                                                                       uint32_t *pathDelay);

/***************************************************************************//**
 * @brief This function sets up the Digital Mobile Radio (DMR) sync search
 * configuration for the ADRV9001 device. It should be called when the
 * channel is in either the STANDBY or CALIBRATED state. The function
 * configures the device with the specified DMR search settings, which
 * are crucial for the monitor mode operation. It is important to ensure
 * that the device context and configuration structure are properly
 * initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param dmrSearchCfg Pointer to the desired DMR search settings structure.
 * Must not be null. The caller retains ownership and is
 * responsible for ensuring the structure is correctly
 * populated with valid configuration data.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_powerSavingAndMonitorMode_MonitorMode_DmrSearch_Configure(adi_adrv9001_Device_t *adrv9001,
                                                                               adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeDmrSearchCfg_t *dmrSearchCfg);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_H_ */

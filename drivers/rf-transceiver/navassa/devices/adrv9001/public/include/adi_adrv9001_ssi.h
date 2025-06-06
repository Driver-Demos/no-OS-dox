/**
 * \file
 * \brief Contains ADRV9001 receive related function prototypes for adi_adrv9001_ssi.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_SSI_H_
#define _ADI_ADRV9001_SSI_H_

#include "adi_adrv9001_ssi_types.h"
#include "adi_common_error_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to set up the Serial Synchronous Interface (SSI)
 * test mode for a specified receive (Rx) channel on the ADRV9001 device.
 * It is essential to call this function when you need to test the SSI
 * interface under different conditions, such as using different data
 * formats or test patterns. The function requires a valid device context
 * and configuration parameters, including the channel, SSI type, data
 * format, and test mode configuration. It returns a status code
 * indicating success or the necessary action to recover from an error.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel to configure. Must be a valid channel number as
 * defined by adi_common_ChannelNumber_e.
 * @param ssiType The type of SSI interface, either CMOS or LVDS, as defined by
 * adi_adrv9001_SsiType_e.
 * @param dataFormat The data format of the SSI interface, either I-data or
 * I/Q-data, as defined by adi_adrv9001_SsiDataFormat_e.
 * @param ssiTestModeConfig Pointer to the desired SSI test mode configuration.
 * Must not be null. The caller retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Rx_TestMode_Configure(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SsiType_e ssiType,
                                               adi_adrv9001_SsiDataFormat_e dataFormat,
                                               adi_adrv9001_RxSsiTestModeCfg_t *ssiTestModeConfig);

/***************************************************************************//**
 * @brief This function sets up the Serial Synchronous Interface (SSI) test mode
 * for a specified transmit (Tx) channel on the ADRV9001 device. It is
 * used to configure the SSI interface to operate in a test mode, which
 * can be useful for debugging and validation purposes. The function
 * requires the device context, channel number, SSI type, data format,
 * and a configuration structure specifying the desired test mode. It
 * returns a status code indicating success or the necessary recovery
 * action. This function should be called when the device is in a state
 * that allows configuration changes, and the caller must ensure that the
 * provided parameters are valid and appropriate for the current device
 * setup.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Tx channel to configure. Must be a valid channel number as
 * defined by adi_common_ChannelNumber_e.
 * @param ssiType The type of SSI interface, either CMOS or LVDS, as defined by
 * adi_adrv9001_SsiType_e.
 * @param dataFormat The data format of the SSI interface, either I-data or
 * I/Q-data, as defined by adi_adrv9001_SsiDataFormat_e.
 * @param ssiTestModeConfig Pointer to a structure defining the desired SSI test
 * mode configuration. Must not be null. The caller
 * retains ownership.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Tx_TestMode_Configure(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SsiType_e ssiType,
                                               adi_adrv9001_SsiDataFormat_e dataFormat,
                                               adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig);

/***************************************************************************//**
 * @brief This function is used to inspect the status of the SSI test mode for a
 * specified Tx channel on the ADRV9001 device. It should be called after
 * configuring the SSI test mode to verify the current status, including
 * any data errors, strobe alignment errors, and FIFO status. The
 * function requires a valid device context and configuration parameters,
 * and it outputs the status into a provided structure. It returns a
 * status code indicating success or the necessary action to recover from
 * an error.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. Caller retains ownership.
 * @param channel The Tx channel for which to inspect SSI test mode status. Must
 * be a valid channel number.
 * @param ssiType The type of SSI interface, either CMOS or LVDS. Must be a
 * valid enum value.
 * @param dataFormat The data format of the SSI interface, specifying I-data or
 * I/Q-data. Must be a valid enum value.
 * @param ssiTestModeConfig Pointer to the SSI test mode configuration
 * structure. Must not be null. Caller retains
 * ownership.
 * @param ssiTestModeStatus Pointer to the structure where the current SSI test
 * mode status will be stored. Must not be null. Caller
 * retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(adi_adrv9001_Device_t *adrv9001,
                                                    adi_common_ChannelNumber_e channel,
                                                    adi_adrv9001_SsiType_e ssiType,
                                                    adi_adrv9001_SsiDataFormat_e dataFormat,
                                                    adi_adrv9001_TxSsiTestModeCfg_t *ssiTestModeConfig,
                                                    adi_adrv9001_TxSsiTestModeStatus_t *ssiTestModeStatus);

/***************************************************************************//**
 * @brief This function is used to enable or disable the loopback from the
 * transmit (Tx) SSI interface to the receive (Rx) SSI interface for a
 * specified channel on the ADRV9001 device. It is applicable for both
 * CMOS and LVDS SSI types. The function should be called when
 * configuring the SSI interface loopback settings, and it requires a
 * valid device context and channel specification. The function performs
 * validation on the input parameters and returns a status code
 * indicating success or the necessary action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to set the SSI interface loopback.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param ssiType The type of SSI interface, either CMOS or LVDS, as defined by
 * adi_adrv9001_SsiType_e.
 * @param loopbackEnable Boolean flag indicating whether to enable (true) or
 * disable (false) the loopback.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Loopback_Set(adi_adrv9001_Device_t *adrv9001,
                                      adi_common_ChannelNumber_e channel,
                                      adi_adrv9001_SsiType_e ssiType,
                                      bool loopbackEnable);

/**
 * \brief This function programs the SSI delay configuration in ADRV9001 device through SPI.
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in] adrv9001          Context variable - Pointer to the ADRV9001 device data structure
 * \param[in] ssiType           LVDS or CMOS mode
 * \param[in] ssiCalibration    The desired SSI calibration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */

/***************************************************************************//**
 * @brief This function sets the SSI delay configuration for the ADRV9001 device
 * using SPI communication. It should be used when you need to adjust the
 * delay settings for the SSI interface, which can be either LVDS or
 * CMOS. The function requires a valid device context and a calibration
 * configuration structure. It is important to ensure that the device is
 * properly initialized before calling this function. The function will
 * return a status code indicating success or the necessary action to
 * recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before use.
 * @param ssiType Specifies the type of SSI interface, either LVDS or CMOS. Must
 * be a valid value of type adi_adrv9001_SsiType_e.
 * @param ssiCalibration Pointer to the desired SSI calibration configuration.
 * Must not be null and should contain valid calibration
 * settings.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Delay_Configure(adi_adrv9001_Device_t *adrv9001,
                                         adi_adrv9001_SsiType_e ssiType,
                                         adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration);

/**
 * \brief This function gets the SSI delay configuration from ADRV9001 device through SPI.
 *
 * \note Message type: \ref timing_direct "Direct register acccess"
 *
 * \param[in]  adrv9001         Context variable - Pointer to the ADRV9001 device data structure
 * \param[in]  ssiType          LVDS or CMOS mode
 * \param[out] ssiCalibration   The current SSI calibration
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */

/***************************************************************************//**
 * @brief This function is used to obtain the current SSI (Serial Synchronous
 * Interface) delay configuration from an ADRV9001 device. It is
 * essential to call this function when you need to verify or log the
 * current delay settings for either LVDS or CMOS SSI types. The function
 * requires a valid device context and an initialized calibration
 * configuration structure to store the retrieved settings. It is
 * important to ensure that the device is properly initialized and
 * configured before calling this function to avoid unexpected results.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param ssiType Specifies the type of SSI interface, either LVDS or CMOS. The
 * function will retrieve the delay configuration corresponding
 * to this type.
 * @param ssiCalibration A pointer to an adi_adrv9001_SsiCalibrationCfg_t
 * structure where the current SSI calibration settings
 * will be stored. Must not be null, and should be
 * allocated by the caller.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Delay_Inspect(adi_adrv9001_Device_t *adrv9001,
                                       adi_adrv9001_SsiType_e ssiType,
                                       adi_adrv9001_SsiCalibrationCfg_t *ssiCalibration);

/***************************************************************************//**
 * @brief This function configures the power down mode for a specified channel
 * on the ADRV9001 device, applicable only when the channel is in the
 * STANDBY state. It is important to note that the power down enablement
 * is valid only for LVDS interfaces, as CMOS interfaces always have
 * power down disabled. This function should be used when it is necessary
 * to manage power consumption by disabling unused SSI pads.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param port The port associated with the channel. Must be a valid port
 * identifier as defined by adi_common_Port_e.
 * @param channel The channel for which to set the power down mode. Must be a
 * valid channel number as defined by adi_common_ChannelNumber_e.
 * @param powerDownMode The desired power down mode. Must be a valid mode as
 * defined by adi_adrv9001_SsiPowerDown_e.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_PowerDown_Set(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel,
                                       adi_adrv9001_SsiPowerDown_e powerDownMode);

/***************************************************************************//**
 * @brief This function is used to obtain the current power down mode of a
 * specified channel on the ADRV9001 device. It should be called when the
 * channel is in any of the following states: STANDBY, CALIBRATED,
 * PRIMED, or RF_ENABLED. The function requires a valid device context
 * and will output the power down mode through the provided pointer. It
 * is important to ensure that the device and channel parameters are
 * correctly specified to avoid errors.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device context. Must not be null.
 * @param port An adi_common_Port_e value specifying the port associated with
 * the channel. Must be a valid port identifier.
 * @param channel An adi_common_ChannelNumber_e value indicating the channel for
 * which to get the power down mode. Must be a valid channel
 * number.
 * @param powerDownMode A pointer to an adi_adrv9001_SsiPowerDown_e variable
 * where the current power down mode will be stored. Must
 * not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_PowerDown_Get(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_Port_e port,
                                       adi_common_ChannelNumber_e channel,
                                       adi_adrv9001_SsiPowerDown_e *powerDownMode);

/***************************************************************************//**
 * @brief This function is used to configure the SSI Rx gain selection for a
 * specified channel on the ADRV9001 device. It should be called when the
 * channel is in either the STANDBY or CALIBRATED state. The function
 * requires a valid configuration structure to be provided, which
 * specifies the desired gain selection settings. This operation is
 * performed through a mailbox command, and the function returns a status
 * code indicating success or the necessary action to recover from an
 * error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param channel The Rx channel for which to configure SSI Rx gain selection.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param ssiRxGainSelectCfg Pointer to a configuration structure of type
 * adi_adrv9001_SsiRxGainSelectCfg_t containing the
 * desired SSI Rx gain selection settings. Must not be
 * null, and should be properly populated with valid
 * configuration data.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Rx_GainSelect_Configure(adi_adrv9001_Device_t *adrv9001,
                                                 adi_common_ChannelNumber_e channel,
                                                 adi_adrv9001_SsiRxGainSelectCfg_t *ssiRxGainSelectCfg);

/***************************************************************************//**
 * @brief This function retrieves the current SSI Rx gain selection
 * configuration for a specified channel on the ADRV9001 device. It
 * should be used when the channel is in any of the following states:
 * STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. The function populates the
 * provided configuration structure with the current settings, allowing
 * the caller to inspect the gain selection parameters. It is important
 * to ensure that the device context and channel parameters are valid
 * before calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The Rx channel for which to inspect the SSI Rx gain selection.
 * Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param ssiRxGainSelectCfg A pointer to a structure where the current SSI Rx
 * gain selection will be stored. Must not be null.
 * The function populates this structure with the gain
 * selection data.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_Ssi_Rx_GainSelect_Inspect(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_SsiRxGainSelectCfg_t *ssiRxGainSelectCfg);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_SSI_H_ */

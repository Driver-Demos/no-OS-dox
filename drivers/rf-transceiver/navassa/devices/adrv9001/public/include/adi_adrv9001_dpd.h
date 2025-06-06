/**
 * \file
 * \brief Contains ADRV9001 Digital Pre-Distortion (DPD) related function prototypes
 *
 * Copyright 2019-2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_DPD_H_
#define _ADI_ADRV9001_DPD_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_dpd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to configure the initial Digital Pre-Distortion
 * (DPD) settings for a specified channel on the ADRV9001 device. It must
 * be called when the channel is in the STANDBY state and the profile
 * indicates that an external loopback for the channel exists. This
 * function is typically invoked as part of the device initialization
 * process. Proper configuration of DPD is crucial for optimizing the
 * performance of the transmission path by compensating for non-
 * linearities in the power amplifier.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param channel The channel to configure, specified as an enumeration of type
 * adi_common_ChannelNumber_e. Must be a valid channel number.
 * @param dpdConfig Pointer to the desired DPD configuration structure of type
 * adi_adrv9001_DpdInitCfg_t. Must not be null. The caller
 * retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_Initial_Configure(adi_adrv9001_Device_t *adrv9001, 
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_DpdInitCfg_t *dpdConfig);

/***************************************************************************//**
 * @brief Use this function to retrieve the current pre-initialization
 * calibration settings of the Digital Pre-Distortion (DPD) for a
 * specified channel on the ADRV9001 device. This function is useful for
 * verifying the initial DPD configuration before further operations. It
 * requires a valid device context and channel number, and it outputs the
 * current DPD configuration into the provided structure. Ensure that the
 * device is properly initialized and the channel is in a suitable state
 * before calling this function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param channel The channel to inspect. Must be a valid channel number as
 * defined by adi_common_ChannelNumber_e.
 * @param dpdConfig Pointer to a structure where the current DPD configuration
 * will be stored. Must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_Initial_Inspect(adi_adrv9001_Device_t *adrv9001, 
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_DpdInitCfg_t *dpdConfig);

/***************************************************************************//**
 * @brief This function is used to configure the Digital Pre-Distortion (DPD)
 * settings for a specified channel on the ADRV9001 device. It should be
 * called when you need to set or update the DPD configuration
 * parameters. The function requires the channel to be in a state that
 * allows configuration changes, and it resets the DPD operation as part
 * of the configuration process. This function is typically used during
 * device initialization or when reconfiguring the DPD settings.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null, and the caller retains ownership.
 * @param channel The channel to configure, specified as an enumeration value of
 * type adi_common_ChannelNumber_e. It must be a valid channel
 * number for the device.
 * @param dpdConfig A pointer to the desired DPD configuration structure of type
 * adi_adrv9001_DpdCfg_t. This must not be null, and the caller
 * retains ownership. The structure should be properly
 * initialized with the desired configuration values before
 * calling the function.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_Configure(adi_adrv9001_Device_t *adrv9001,
                                   adi_common_ChannelNumber_e channel,
                                   adi_adrv9001_DpdCfg_t *dpdConfig);

/***************************************************************************//**
 * @brief Use this function to retrieve the current Digital Pre-Distortion (DPD)
 * settings for a specified channel on the ADRV9001 device. This function
 * is useful for verifying the DPD configuration that is currently
 * applied. It requires a valid device context and a channel number to
 * specify which channel's configuration to inspect. The function outputs
 * the DPD configuration into the provided dpdConfig structure. Ensure
 * that the dpdConfig pointer is valid and that the device context is
 * properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function.
 * @param channel The channel number to inspect. Must be a valid channel number
 * as defined by adi_common_ChannelNumber_e.
 * @param dpdConfig Pointer to a structure where the current DPD configuration
 * will be stored. Must not be null and should point to a valid
 * adi_adrv9001_DpdCfg_t structure.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if an error occurs.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_Inspect(adi_adrv9001_Device_t *adrv9001,
                                 adi_common_ChannelNumber_e channel,
                                 adi_adrv9001_DpdCfg_t *dpdConfig);
    
/***************************************************************************//**
 * @brief This function sets the Digital Pre-Distortion (DPD) coefficients for a
 * specified channel on the ADRV9001 device, allowing for a "hot start"
 * with previously saved coefficients instead of starting from unity
 * coefficients. It should be called when the channel state is
 * CALIBRATED. This function is useful for restoring a known good state
 * of DPD operation quickly. The function communicates with the device
 * using a mailbox command and returns a status code indicating success
 * or the required action to recover.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param channel The channel of interest to configure. Must be a valid channel
 * number as defined by adi_common_ChannelNumber_e.
 * @param coefficients Pointer to the DPD coefficients to set. Must not be null.
 * The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_coefficients_Set(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_DpdCoefficients_t *coefficients);

/***************************************************************************//**
 * @brief This function is used to obtain the most recent Digital Pre-Distortion
 * (DPD) coefficients for a given channel on the ADRV9001 device. It
 * should be called when the channel is in the CALIBRATED state to ensure
 * valid data retrieval. The function updates the provided coefficients
 * structure with the latest values, which can be used for analysis or to
 * restore a previous state. It is important to ensure that the device
 * context and channel parameters are valid before calling this function
 * to avoid errors.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null and should be properly initialized
 * before calling the function. The caller retains ownership.
 * @param channel The channel number for which the DPD coefficients are to be
 * retrieved. It must be a valid channel number as defined by the
 * adi_common_ChannelNumber_e enumeration.
 * @param coefficients A pointer to an adi_adrv9001_DpdCoefficients_t structure
 * where the retrieved coefficients will be stored. This
 * must not be null and should be allocated by the caller
 * before the function is called. The function will update
 * this structure with the latest coefficients.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_coefficients_Get(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_DpdCoefficients_t *coefficients);

/***************************************************************************//**
 * @brief This function retrieves the captured Digital Pre-Distortion (DPD) data
 * for a specified channel from the ADRV9001 device. It must be called
 * when the channel state is either CALIBRATED or PRIMED. The function
 * reads both the I and Q components of the transmitted (Tx) and external
 * loopback (ELB) data, storing them in the provided arrays. The length
 * of these arrays is specified by the caller, and the function supports
 * optional auto-increment of the ARM register address. This function is
 * typically used to analyze or process the captured DPD data for further
 * operations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param channel The channel of interest, specified as an
 * adi_common_ChannelNumber_e enum. Valid values are typically
 * channel 1 or 2.
 * @param iData_tx Array to store the 18-bit I values of the captured Tx data.
 * Must be pre-allocated by the caller with a size of at least
 * 'length'.
 * @param qData_tx Array to store the 18-bit Q values of the captured Tx data.
 * Must be pre-allocated by the caller with a size of at least
 * 'length'.
 * @param iData_elb Array to store the 18-bit I values of the captured ELB data.
 * Must be pre-allocated by the caller with a size of at least
 * 'length'.
 * @param qData_elb Array to store the 18-bit Q values of the captured ELB data.
 * Must be pre-allocated by the caller with a size of at least
 * 'length'.
 * @param length The number of data points to read into each array. Must be a
 * positive integer.
 * @param autoIncrement Boolean flag indicating whether to enable auto-increment
 * of the ARM register address during data read.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The arrays iData_tx, qData_tx, iData_elb, and qData_elb are
 * populated with the captured data.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_CaptureData_Read(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          int32_t iData_tx[],
                                          int32_t qData_tx[],
                                          int32_t iData_elb[],
                                          int32_t qData_elb[],
                                          uint32_t length,
                                          bool autoIncrement);

/***************************************************************************//**
 * @brief This function is used to configure the Digital Pre-Distortion (DPD)
 * Frequency Hopping (FH) frequency regions for a specified transmission
 * channel on the ADRV9001 device. It should be called when the channel
 * is in either the STANDBY or CALIBRATED state. The function takes an
 * array of frequency regions and their count, which are then applied to
 * the specified channel. This configuration is necessary for setting up
 * the DPD FH regions before starting frequency hopping operations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param channel The transmission channel to configure. Must be a valid channel
 * number as defined by adi_common_ChannelNumber_e.
 * @param dpdFhRegions Array of DPD FH frequency regions to configure. Must not
 * be null and should contain valid frequency region data.
 * @param size The number of DPD FH regions to configure. Must be greater than
 * zero and should not exceed the maximum supported regions.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_fh_regions_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                              uint32_t size);

/***************************************************************************//**
 * @brief Use this function to retrieve the current DPD frequency hopping (FH)
 * regions for a specified transmission channel on the ADRV9001 device.
 * This function is typically called when the channel is in either the
 * STANDBY or CALIBRATED state. It reads the frequency regions into the
 * provided array, allowing inspection of the current configuration.
 * Ensure that the `dpdFhRegions` array is large enough to hold the
 * number of regions specified by `size`, with a maximum of 7 regions.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param channel The transmission channel of interest, specified as an
 * enumeration of type `adi_common_ChannelNumber_e`. Must be a
 * valid channel number.
 * @param dpdFhRegions Array to store the inspected DPD FH frequency regions.
 * Must not be null and should be pre-allocated with enough
 * space to hold `size` elements. The function writes the
 * frequency regions into this array.
 * @param size The number of DPD FH regions to inspect. Must be a positive
 * integer, with a maximum value of 7.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_dpd_fh_regions_Inspect(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                            uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_DPD_H_ */

/**
 * \file
 * \brief Contains ADRV9001 calibration related function prototypes for
 *        adi_adrv9001_cals.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_CALS_H_
#define _ADI_ADRV9001_CALS_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */
/***************************************************************************//**
 * @brief This function executes the initialization calibrations for the
 * ADRV9001 device. It should be called after the device has been
 * initialized and the RF PLL is confirmed to be locked. The function
 * uses the provided calibration settings and waits for the specified
 * timeout period for the calibrations to complete. It also provides an
 * error flag to indicate any issues encountered during the process. This
 * function is essential for ensuring the device is properly calibrated
 * before use.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param initCals Pointer to the InitCals structure specifying which
 * calibrations to run. 'calMode' must not be
 * ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY. Must not be null.
 * @param timeout_ms A timeout value in milliseconds to wait for the
 * calibrations to complete. Must be a positive integer.
 * @param errorFlag A pointer to a 3-bit error flag that helps identify any
 * errors during initial calibrations. '0' indicates no error.
 * Must not be null.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover. The errorFlag is updated to reflect any
 * errors encountered.
 ******************************************************************************/
int32_t adi_adrv9001_cals_InitCals_Run(adi_adrv9001_Device_t *adrv9001,
                                       adi_adrv9001_InitCals_t *initCals,
                                       uint32_t timeout_ms,
                                       uint8_t *errorFlag);

/***************************************************************************//**
 * @brief Use this function to initialize an `adi_adrv9001_InitCals_t` structure
 * with default calibration settings, enabling all initialization
 * calibrations. This function should be called before running any
 * initialization calibrations to ensure that the calibration structure
 * is properly configured. It is important to provide a valid pointer to
 * an `adi_adrv9001_InitCals_t` structure, as the function will populate
 * it with default values. This function does not perform any actions
 * that require recovery.
 *
 * @param initCals Pointer to an `adi_adrv9001_InitCals_t` structure that will
 * be populated with default calibration settings. Must not be
 * null, as the function will write default values to this
 * structure.
 * @return Returns `ADI_COMMON_ACT_NO_ACTION` indicating successful completion
 * without any required recovery actions.
 ******************************************************************************/
int32_t adi_adrv9001_cals_InitCalsBuildDefault(adi_adrv9001_InitCals_t *initCals);

/***************************************************************************//**
 * @brief Use this function to enable specific tracking calibrations on the
 * ADRV9001 device for the specified channels. This function should be
 * called when the channel state is either STANDBY or CALIBRATED. It
 * configures the device to continuously run the specified tracking
 * calibrations, which are provided as a bitmask. This function is
 * essential for maintaining optimal performance of the device by
 * ensuring that the necessary calibrations are active.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param trackingCals Pointer to a structure specifying which tracking
 * calibrations to enable. Each mask is an OR'd combination
 * of 'adi_adrv9001_TrackingCalibrations_e' enum values.
 * Must not be null.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_Tracking_Set(adi_adrv9001_Device_t *adrv9001,
                                       adi_adrv9001_TrackingCals_t *trackingCals);

/***************************************************************************//**
 * @brief Use this function to obtain the current tracking calibration bitmask
 * information from the ADRV9001 device. It should be called when the
 * channel state is any of STANDBY, CALIBRATED, PRIMED, or RF_ENABLED.
 * This function is useful for verifying which tracking calibrations are
 * currently active on the device. Ensure that the device context is
 * properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function.
 * @param trackingCals Pointer to a structure where the current tracking
 * calibration bitmask information will be stored. Must not
 * be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_Tracking_Get(adi_adrv9001_Device_t *adrv9001,
                                       adi_adrv9001_TrackingCals_t *trackingCals);

/***************************************************************************//**
 * @brief This function is used to perform external path delay calibrations on
 * the ADRV9001 device for a specified channel. It should be called when
 * the channel state is in CALIBRATED, and the
 * 'ADI_ADRV9001_INIT_CAL_TX_LB_PD' calibration must have been enabled
 * during the initial calibrations. The function waits for the
 * calibration to complete within a specified timeout period and provides
 * an error flag to indicate any issues encountered during the process.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param channel The channel for which to run external path delay calibrations.
 * Must be a valid channel number.
 * @param timeout_ms Timeout value in milliseconds to wait for the calibrations
 * to complete. Must be a positive integer.
 * @param initCalsError Pointer to a 3-bit error flag that indicates any errors
 * during initial calibrations. Must not be null.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_ExternalPathDelay_Run(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t timeout_ms,
                                                uint8_t *initCalsError);

/***************************************************************************//**
 * @brief This function is used to measure the difference in path delays between
 * the internal loopback (ILB) and external loopback (ELB) for a
 * specified channel on the ADRV9001 device. It should be called when the
 * channel is in the CALIBRATED state. The function calculates the delay
 * in tenths of a nanosecond and returns the result in picoseconds. This
 * measurement is useful for calibration purposes to ensure accurate
 * signal timing.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param channel The channel for which to measure the external path delay. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param externalPathDelay_ps Pointer to a uint32_t where the measured path
 * delay difference will be stored in picoseconds.
 * Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_ExternalMinusInternalPathDelay_Measure(adi_adrv9001_Device_t *adrv9001,
                                                                 adi_common_ChannelNumber_e channel,
                                                                 uint32_t *externalPathDelay_ps);

/***************************************************************************//**
 * @brief This function is used to perform external path delay calibrations on a
 * specified channel of the ADRV9001 device and retrieve the measurement
 * result of the difference in path delays between internal and external
 * loopback paths. It should be called when the channel is in a
 * CALIBRATED state, and the initial calibration for TX loopback path
 * delay must have been enabled previously. The function waits for the
 * calibration to complete within a specified timeout period and provides
 * an error flag to indicate any issues encountered during the process.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param channel The channel for which to run external path delay calibrations.
 * Must be a valid channel number.
 * @param timeout_ms Timeout value in milliseconds to wait for the calibrations
 * to complete. Must be a positive integer.
 * @param initCalsError Pointer to a 3-bit error flag that indicates any errors
 * during initial calibrations. Must not be null.
 * @param externalPathDelay_ps Pointer to store the measurement result of the
 * difference in path delays (in picoseconds)
 * between internal and external loopback paths.
 * Must not be null.
 * @return Returns a code indicating success or the required action to recover.
 * The external path delay measurement is stored in the provided pointer
 * if successful.
 ******************************************************************************/
int32_t adi_adrv9001_cals_ExternalPathDelay_Calibrate(adi_adrv9001_Device_t *adrv9001,
                                                      adi_common_ChannelNumber_e channel,
                                                      uint32_t timeout_ms,
                                                      uint8_t *initCalsError,
                                                      uint32_t *externalPathDelay_ps);

/***************************************************************************//**
 * @brief Use this function to set the external path delay for a specific
 * channel on the ADRV9001 device. This function should be called when
 * the channel is in either the STANDBY or CALIBRATED state. It is
 * important to ensure that the external path delay value is within the
 * valid range of 0 to 6553500 picoseconds. The function returns a status
 * code indicating success or the necessary action to recover from an
 * error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized.
 * @param channel The channel number for which the external path delay is being
 * set. Must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param externalPathDelay_ps The desired external path delay in picoseconds.
 * Valid range is 0 to 6553500 picoseconds. Values
 * outside this range may result in an error.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_ExternalPathDelay_Set(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t externalPathDelay_ps);

/***************************************************************************//**
 * @brief This function is used to obtain the current external path delay for a
 * specified channel on the ADRV9001 device. It should be called when the
 * channel is in one of the following states: STANDBY, CALIBRATED,
 * PRIMED, or RF_ENABLED. The function writes the delay value, measured
 * in picoseconds, to the provided output parameter. It is important to
 * ensure that the device context and channel parameters are valid before
 * calling this function to avoid unexpected behavior.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null and should be properly initialized
 * before calling the function.
 * @param channel The channel number for which the external path delay is to be
 * retrieved. It must be a valid channel number as defined by the
 * adi_common_ChannelNumber_e enumeration.
 * @param externalPathDelay_ps A pointer to a uint32_t where the external path
 * delay will be stored. This must not be null, and
 * the caller is responsible for allocating memory
 * for this variable.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_ExternalPathDelay_Get(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t *externalPathDelay_ps);

/***************************************************************************//**
 * @brief Use this function to obtain the internal path delay values in
 * nanoseconds for a specified port and channel on the ADRV9001 device.
 * This function is typically called when you need to assess or verify
 * the internal path delays for calibration or diagnostic purposes.
 * Ensure that the device is properly initialized and configured before
 * calling this function. The function fills the provided array with
 * delay values, where the first element corresponds to the main profile,
 * and subsequent elements are reserved for future use with dynamic
 * profile switching.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param port Specifies the port (RX or TX) for which the internal path delay
 * is being queried. Must be a valid adi_common_Port_e value.
 * @param channel Specifies the channel number within the port. Must be a valid
 * adi_common_ChannelNumber_e value.
 * @param internalPathDelays_ns Array to store the internal path delay values in
 * nanoseconds. The caller must ensure the array is
 * large enough to hold the number of delays
 * specified by 'length'. Must not be null.
 * @param length The number of elements in the internalPathDelays_ns array. Must
 * be at least 1 and no more than 6.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The internalPathDelays_ns array is populated with delay
 * values on success.
 ******************************************************************************/
int32_t adi_adrv9001_cals_InternalPathDelay_Get(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_Port_e port,
                                                adi_common_ChannelNumber_e channel,
                                                uint32_t internalPathDelays_ns[],
                                                uint32_t length);

/***************************************************************************//**
 * @brief Use this function to obtain the carrier frequencies of all channels
 * that were used in the previous successful initialization calibration
 * of the ADRV9001 device. This function should be called when the device
 * is in any of the following states: STANDBY, CALIBRATED, PRIMED, or
 * RF_ENABLED. It is important to ensure that the `carrierFrequencies_Hz`
 * array has sufficient length to store the frequencies for all channels,
 * with a maximum of four entries (Rx1, Rx2, Tx1, Tx2).
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * Must not be null.
 * @param carrierFrequencies_Hz An array to store the carrier frequencies in
 * Hertz. The caller must ensure the array is large
 * enough to hold up to four frequencies.
 * @param length The number of entries in the `carrierFrequencies_Hz` array.
 * Must be at least 1 and at most 4.
 * @return Returns an integer code indicating success or the required action to
 * recover. The `carrierFrequencies_Hz` array is populated with the
 * carrier frequencies of the channels used in the last initialization
 * calibration.
 ******************************************************************************/
int32_t adi_adrv9001_cals_LastInitCal_CarrierFrequency_Get(adi_adrv9001_Device_t *adrv9001,
                                                           uint64_t carrierFrequencies_Hz[],
                                                           uint32_t length);

/***************************************************************************//**
 * @brief This function is used to perform dynamic profile calibrations on the
 * ADRV9001 device. It should be called when the device is in either the
 * STANDBY or CALIBRATED state. The function iterates over an array of
 * dynamic profiles, applying each one and running the specified initial
 * calibrations. It is essential to ensure that the 'calMode' in the
 * initCals parameter is not set to ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY.
 * The function also requires a timeout value to specify how long to wait
 * for the calibrations to complete. An error flag is provided to capture
 * any errors that occur during the calibration process.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param initCals Pointer to the InitCals structure which specifies which
 * calibrations to run. 'calMode' must not be
 * ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY. Must not be null.
 * @param timeout_ms A timeout value in milliseconds to wait for the
 * calibrations to complete. Must be a positive integer.
 * @param errorFlag A 3-bit error flag that helps identify any errors during
 * initial calibrations. '0' indicates no error. Must not be
 * null.
 * @param dynamicProfile An array of dynamic profile parameters. Must not be
 * null and should have a length of up to 6.
 * @param length Length of the dynamicProfile array. Must be a positive integer,
 * with a maximum value of 6.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_cals_Dynamic_profiles_calibrate(adi_adrv9001_Device_t *adrv9001,
                                                     adi_adrv9001_InitCals_t *initCals,
                                                     uint32_t timeout_ms,
                                                     uint8_t *errorFlag,
                                                     adi_adrv9000_DynamicProfile_t dynamicProfile[],
                                                     uint32_t length);

/***************************************************************************//**
 * @brief This function is used to obtain the number of unique initialization
 * calibrations that are enabled for a given ADRV9001 device
 * configuration. It should be called when the channel state is
 * CALIBRATED. The function populates the provided calNumbers structure
 * with the number of unique enabled calibrations and their respective
 * identifiers. It requires calibration bit masks for both channel 1 and
 * channel 2 to determine which calibrations are enabled. The function
 * returns a status code indicating success or the necessary action to
 * recover from an error.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param calNumbers Pointer to a structure that will be populated with the
 * number of unique enabled calibrations and their
 * identifiers. Must not be null.
 * @param maskChannel1 Calibration bit mask for channel 1. Determines which
 * calibrations are considered for channel 1.
 * @param maskChannel2 Calibration bit mask for channel 2. Determines which
 * calibrations are considered for channel 2.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
	int32_t adi_adrv9001_cals_InitCals_WarmBoot_UniqueEnabledCals_Get(adi_adrv9001_Device_t *device,
		                                                        adi_adrv9001_Warmboot_CalNumbers_t *calNumbers,
																uint32_t maskChannel1,
																uint32_t maskChannel2);
/***************************************************************************//**
 * @brief This function is used to read the initialization calibration
 * coefficients required for a warmboot operation from the ADRV9001
 * device. It should be called when the device is in a CALIBRATED state.
 * The function retrieves coefficients for the enabled calibrations
 * specified by the channel masks and stores them in the provided
 * `savedCals` structure. This is useful for saving the current
 * calibration state, which can be restored later to reduce
 * initialization time during a warmboot.
 *
 * @param device A pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param savedCals A pointer to an `adi_adrv9001_Warmboot_Coeff_t` structure
 * where the retrieved coefficients will be stored. Must not be
 * null.
 * @param maskChannel1 A 32-bit mask specifying which calibrations to retrieve
 * for channel 1. Each bit represents a different
 * calibration.
 * @param maskChannel2 A 32-bit mask specifying which calibrations to retrieve
 * for channel 2. Each bit represents a different
 * calibration.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
 int32_t adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_MaxArray_Get(adi_adrv9001_Device_t *device,
		adi_adrv9001_Warmboot_Coeff_t *savedCals,
		uint32_t maskChannel1,
		uint32_t maskChannel2);
	
/***************************************************************************//**
 * @brief This function is used to read the unique initialization calibration
 * coefficients required for a warm boot of the ADRV9001 device. It
 * should be called when the device is in a CALIBRATED state. The
 * function retrieves coefficients based on the specified calibration bit
 * masks for two channels and stores them in the provided memory block.
 * Ensure that the memory block pointed to by `memStartAddress` is
 * properly allocated to hold the required number of bytes before calling
 * this function.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param memStartAddress Pointer to the start address of a memory block
 * allocated to hold the warm boot coefficients. Must not
 * be null and should be large enough to store the
 * coefficients.
 * @param maskChannel1 Calibration bit mask for channel 1. Determines which
 * calibrations are considered for channel 1.
 * @param maskChannel2 Calibration bit mask for channel 2. Determines which
 * calibrations are considered for channel 2.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
	int32_t adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_UniqueArray_Get(adi_adrv9001_Device_t *device,
		uint8_t *memStartAddress,
		uint32_t maskChannel1,
		uint32_t maskChannel2);

/***************************************************************************//**
 * @brief This function is used to write the initialization calibration
 * coefficients required for a warm boot of the ADRV9001 device. It
 * should be called when the device is in the STANDBY state. The function
 * takes calibration coefficients and writes them to the device for the
 * specified channels, as indicated by the provided bit masks. This is
 * useful for restoring calibration settings after a device reset or
 * power cycle.
 *
 * @param device A pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param savedCals A pointer to a structure containing the coefficients for the
 * enabled calibrations. Must not be null. The caller retains
 * ownership.
 * @param maskChannel1 A bitmask indicating which calibrations to apply for
 * channel 1. Each bit represents a different calibration.
 * @param maskChannel2 A bitmask indicating which calibrations to apply for
 * channel 2. Each bit represents a different calibration.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
	int32_t adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_MaxArray_Set(adi_adrv9001_Device_t *device,
																adi_adrv9001_Warmboot_Coeff_t *savedCals,
																uint32_t maskChannel1,
																uint32_t maskChannel2);
/***************************************************************************//**
 * @brief This function is used to write the initialization calibration
 * coefficients required for a warm boot from a pre-allocated memory
 * block. It should be called when the device is in the STANDBY state.
 * The function uses the provided memory block to write coefficients for
 * the specified channels based on the given calibration bit masks. It is
 * important to ensure that the memory block is correctly allocated and
 * contains the necessary data before calling this function.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param memStartAddress Pointer to the start address of the memory block that
 * has been allocated and contains the
 * warmbootMemoryNumBytes. The caller retains ownership
 * and must ensure it is valid.
 * @param maskChannel1 Calibration bit mask for channel 1. Determines which
 * calibrations are applied to channel 1.
 * @param maskChannel2 Calibration bit mask for channel 2. Determines which
 * calibrations are applied to channel 2.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
	int32_t adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_UniqueArray_Set(adi_adrv9001_Device_t *device,
		uint8_t *memStartAddress,
		uint32_t maskChannel1,
		uint32_t maskChannel2);
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_CALS_H_ */


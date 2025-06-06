/**
 * \file
 * \brief Contains ADRV9001 Frequency Hopping function prototypes
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_FH_H_
#define _ADI_ADRV9001_FH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_adrv9001_fh_types.h"

/***************************************************************************//**
 * @brief This function is used to configure the frequency hopping settings on
 * an ADRV9001 device. It should be called when the device is in the
 * STANDBY state, and it requires a valid configuration structure to be
 * provided. The function sets up various parameters related to frequency
 * hopping, such as mode, gain, and frequency ranges, and writes these
 * settings to the device. It also configures GPIOs if necessary, based
 * on the provided configuration. This function is essential for enabling
 * frequency hopping capabilities on the device and must be used before
 * any frequency hopping operations are performed.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should point to a properly initialized device
 * context.
 * @param fhConfig A pointer to a structure containing the frequency hopping
 * configuration settings to be applied. This must not be null
 * and should be populated with valid configuration data before
 * calling the function.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_fh_Configure(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_FhCfg_t  *fhConfig);

/***************************************************************************//**
 * @brief This function retrieves the current frequency hopping configuration
 * from the ADRV9001 device and populates the provided configuration
 * structure with these settings. It can be called at any time after the
 * device has been initialized, allowing users to inspect the current
 * frequency hopping setup. This is useful for verifying the
 * configuration or for debugging purposes. The function requires a valid
 * device context and a pointer to a configuration structure where the
 * data will be stored.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param fhConfig A pointer to an adi_adrv9001_FhCfg_t structure that will be
 * populated with the frequency hopping configuration settings.
 * This must not be null. The caller is responsible for
 * allocating and managing the memory for this structure.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_Configuration_Inspect(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_FhCfg_t *fhConfig);

/**
 * \brief Load a frequency hopping table into ARM memory
 * 
 * A frequency hopping table is made up by an array of hop frame information
 * defined by adi_adrv9001_FhHopFrame_t
 * 
 * \note Message type: \ref timing_prioritymailbox "High-priority mailbox command"
 *       Maximum table size is 64
 *
 * \pre This function can be called by the user anytime after initialization.
 *      mode should be set to the same as that set in fhConfig when calling 
 *      adi_adrv9001_fh_Configure
 * 
 * \param[in] adrv9001      Context variable - Pointer to the ADRV9001 device data structure
 * \param[in] mode          Frequency hopping mode
 * \param[in] hopSignal     Hop signal to configure appropriate tableId
 * \param[in] tableId       FH_HOP_TABLE_A or FH_HOP_TABLE_B. Used for ping-pong hop tables.
 * \param[in] hopTable      Array of hop frame information to write as the frequency hopping table
 * \param[in] hopTableSize  Size of hopTable; Number of hop frames to write
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
/* This writes the buffer directly (it's too big for the mailbox), then uses mailbox to message ARM */
/***************************************************************************//**
 * @brief This function is used to load a frequency hopping table, consisting of
 * an array of hop frame information, into the ARM memory of the ADRV9001
 * device. It should be called after the device has been initialized and
 * the frequency hopping mode has been configured using the same mode
 * parameter. The function supports two hop tables, identified by
 * tableId, which can be used for ping-pong operations. The maximum size
 * of the hop table is 64 entries. The function writes the hop table data
 * directly to ARM memory, bypassing the mailbox due to size constraints.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param mode Specifies the frequency hopping mode. Must match the mode set in
 * the device configuration.
 * @param hopSignal Specifies the hop signal to configure the appropriate
 * tableId. Valid values are defined by
 * adi_adrv9001_FhHopSignal_e.
 * @param tableId Specifies which hop table to configure, either FH_HOP_TABLE_A
 * or FH_HOP_TABLE_B. Used for ping-pong hop tables.
 * @param hopTable Array of hop frame information to be written as the frequency
 * hopping table. Must not be null and should have at least
 * hopTableSize elements.
 * @param hopTableSize Number of hop frames to write. Must be less than or equal
 * to 64.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_Static_Configure(adi_adrv9001_Device_t *adrv9001,
                                                  adi_adrv9001_FhMode_e mode,
                                                  adi_adrv9001_FhHopSignal_e hopSignal,
                                                  adi_adrv9001_FhHopTable_e tableId, 
                                                  adi_adrv9001_FhHopFrame_t hopTable[],
                                                  uint32_t hopTableSize);

/**
 * \brief Inspect frequency hopping table in ARM memory
 *
 * This function reads back a frequency hopping table from ARM memory and loads it into hopTable. 
 * The frequency hopping table to read back is specified by a tableId
 * 
 * \note Message type: \ref timing_prioritymailbox "Mailbox command"
 *
 * \pre This function can be called by the user anytime after initialization.
 *
 * \param[in]  adrv9001        Context variable - Pointer to the ADRV9001 device data structure
 * \param[in]  hopSignal       Hop signal to inspect appropriate tableId
 * \param[in]  tableId         FH_HOP_TABLE_A or FH_HOP_TABLE_B. Used for ping-pong hop tables
 * \param[out] hopTable        Read back array of hopTable which will be updated with the read back values
 * \param[in]  hopTableSize    Size of hopTable; Max number of hop frames to read back
 * \param[out] numEntriesRead  Actual number of hop table entries read back from ARM. Pass null if this info is not needed

 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
/* This writes the buffer directly (it's too big for the mailbox), then uses mailbox to trigger the DMA table generation. */
/***************************************************************************//**
 * @brief This function reads a frequency hopping table from ARM memory and
 * populates the provided hopTable array with the retrieved data. It is
 * used to inspect the current state of a frequency hopping table
 * specified by tableId and hopSignal. The function can be called anytime
 * after initialization, and it is important to ensure that the hopTable
 * array is large enough to hold the data being read. If the number of
 * entries read exceeds the size of hopTable, an error is reported. The
 * actual number of entries read can be optionally returned via
 * numEntriesRead.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param hopSignal Specifies the hop signal to inspect the appropriate tableId.
 * Must be a valid adi_adrv9001_FhHopSignal_e value.
 * @param tableId Specifies which frequency hopping table to inspect, either
 * FH_HOP_TABLE_A or FH_HOP_TABLE_B. Must be a valid
 * adi_adrv9001_FhHopTable_e value.
 * @param hopTable Array to be populated with the read back hop table data. Must
 * be large enough to hold the data being read.
 * @param hopTableSize The maximum number of hop frames that can be stored in
 * hopTable. Must be greater than zero.
 * @param numEntriesRead Pointer to a uint32_t where the actual number of hop
 * table entries read will be stored. Can be null if this
 * information is not needed.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The hopTable array is populated with the read back values,
 * and numEntriesRead is updated with the number of entries read if it
 * is not null.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_Inspect(adi_adrv9001_Device_t *adrv9001,
                                         adi_adrv9001_FhHopSignal_e hopSignal,
                                         adi_adrv9001_FhHopTable_e tableId, 
                                         adi_adrv9001_FhHopFrame_t hopTable[],
                                         uint32_t hopTableSize,
                                         uint32_t *numEntriesRead);

/***************************************************************************//**
 * @brief This function is used to set the active frequency hopping table in the
 * ADRV9001 device, allowing the user to switch between predefined tables
 * for frequency hopping operations. It should be called when the device
 * is in a state that allows for configuration changes, typically after
 * initialization and when the channel is in standby. The function
 * requires a valid device context and specific identifiers for the hop
 * signal and table. It returns a status code indicating success or the
 * need for corrective action.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be properly initialized before
 * calling this function.
 * @param hopSignal An enumeration value of type adi_adrv9001_FhHopSignal_e,
 * indicating the hop signal to configure. Must be a valid hop
 * signal supported by the device.
 * @param tableId An enumeration value of type adi_adrv9001_FhHopTable_e,
 * specifying which hop table to activate (FH_HOP_TABLE_A or
 * FH_HOP_TABLE_B). Must be within the valid range for table
 * identifiers.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_Set(adi_adrv9001_Device_t *adrv9001,
                                     adi_adrv9001_FhHopSignal_e hopSignal,
                                     adi_adrv9001_FhHopTable_e tableId);

/***************************************************************************//**
 * @brief This function is used to determine which frequency hopping table,
 * either FH_HOP_TABLE_A or FH_HOP_TABLE_B, is currently active for a
 * given hop signal. It should be called when the user needs to verify or
 * log the current table in use. The function requires that frequency
 * hopping is enabled in the device profile and that the device is
 * properly initialized. It communicates with the device through a
 * mailbox command to retrieve the table ID.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null, and the device should be initialized and configured for
 * frequency hopping.
 * @param hopSignal Specifies the hop signal for which the table ID is to be
 * retrieved. It must be a valid adi_adrv9001_FhHopSignal_e
 * enumeration value.
 * @param tableId A pointer to an adi_adrv9001_FhHopTable_e variable where the
 * current table ID will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The tableId output parameter is
 * updated with the current table ID.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_Get(adi_adrv9001_Device_t *adrv9001,
                                     adi_adrv9001_FhHopSignal_e hopSignal,
                                     adi_adrv9001_FhHopTable_e *tableId);

/***************************************************************************//**
 * @brief This function is used to obtain detailed information about a specific
 * frequency hopping frame, either the current or the upcoming one, based
 * on the provided frame index. It is essential to call this function
 * after the device has been initialized and frequency hopping is
 * enabled. The function requires a valid device context and will
 * populate the provided hop frame structure with the relevant data. It
 * is important to ensure that the hop frame pointer is not null before
 * calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must be
 * a valid, initialized context and must not be null.
 * @param fhHopSignal Specifies the frequency hopping signal to retrieve frame
 * information from. Must be within the range of defined hop
 * signals (e.g., ADI_ADRV9001_FH_HOP_SIGNAL_1 to
 * ADI_ADRV9001_FH_HOP_SIGNAL_2). Invalid values will result
 * in a range check error.
 * @param frameIndex Indicates which frame's information to retrieve, either the
 * current or upcoming frame. Must be within the defined range
 * (e.g., ADI_ADRV9001_FHFRAMEINDEX_CURRENT_FRAME to
 * ADI_ADRV9001_FHFRAMEINDEX_UPCOMING_FRAME). Invalid values
 * will result in a range check error.
 * @param hopFrame A pointer to a structure where the hop frame information will
 * be stored. This pointer must not be null, and the caller
 * retains ownership of the memory.
 * @return Returns an integer code indicating success or the type of error
 * encountered. The hopFrame structure is populated with the retrieved
 * data on success.
 ******************************************************************************/
int32_t adi_adrv9001_fh_FrameInfo_Inspect(adi_adrv9001_Device_t *adrv9001,
		                                  adi_adrv9001_FhHopSignal_e fhHopSignal,
                                          adi_adrv9001_FhFrameIndex_e frameIndex,
                                          adi_adrv9001_FhHopFrame_t *hopFrame);

/***************************************************************************//**
 * @brief Use this function to toggle a frequency hopping signal on the ADRV9001
 * device. This function should be called when the channel is in the
 * PRIMED RF_ENABLED state. It is used to trigger the hop signal either
 * through SPI or mailbox, depending on the configuration. Ensure that
 * the device is properly initialized and configured for frequency
 * hopping before invoking this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should point to a valid, initialized device
 * context.
 * @param hopSignal An enumeration value of type adi_adrv9001_FhHopSignal_e
 * indicating which hop signal to toggle. The value must be a
 * valid hop signal enumeration defined for the device.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_Hop(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_FhHopSignal_e hopSignal);

/***************************************************************************//**
 * @brief This function prepares a SPI packed frequency hopping table that can
 * be used by an FPGA to dynamically load frequency hopping tables into
 * the ADRV9001 ARM memory based on TDD signal activity. It should be
 * called after the device has been initialized and is suitable for use
 * when dynamic frequency hopping is required. The function does not send
 * the SPI transactions itself but generates the necessary data for
 * external use. Ensure that the hopTableSize is exactly twice the
 * numberHopsPerDynamicLoad, as this is a precondition for correct
 * operation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param mode Specifies the frequency hopping mode. Must match the mode set
 * during configuration.
 * @param hopSignal Indicates which hop signal to configure. Determines the
 * appropriate table ID.
 * @param hopTable Array of hop frame information to be written as the frequency
 * hopping table. Must not be null.
 * @param hopTableSize The size of the hopTable array. Must be twice the
 * numberHopsPerDynamicLoad.
 * @param numberHopsPerDynamicLoad Specifies the number of hops to be loaded in
 * real time. Determines the size of each half
 * of the hopTable.
 * @param spiPackedFhTable Output buffer that will contain the SPI formatted
 * frequency hopping table. Must be large enough to hold
 * the packed data.
 * @param length The length of the spiPackedFhTable array. Must be sufficient to
 * store the packed data.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_Dynamic_Configure(adi_adrv9001_Device_t *adrv9001,
                                                   adi_adrv9001_FhMode_e mode,
                                                   adi_adrv9001_FhHopSignal_e hopSignal,
                                                   adi_adrv9001_FhHopFrame_t hopTable[],
                                                   uint32_t hopTableSize,
                                                   adi_adrv9001_FhPerDynamicLoad_e numberHopsPerDynamicLoad,
                                                   uint8_t spiPackedFhTable[],
                                                   uint32_t length);

/***************************************************************************//**
 * @brief This function calculates the number of SPI packed bytes required for a
 * dynamic frequency hopping table load based on the specified number of
 * hops per dynamic load. It should be called after the device has been
 * initialized and is useful for determining the memory requirements for
 * frequency hopping operations. The function does not modify the device
 * state or configuration.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param numberHopsPerDynamicLoad Specifies the number of hops to be loaded in
 * real time. Must be a valid value defined by
 * adi_adrv9001_FhPerDynamicLoad_e.
 * @param bytesPerTable Pointer to a uint32_t where the function will store the
 * calculated number of SPI packed bytes per table. Must
 * not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_HopTable_BytesPerTable_Get(adi_adrv9001_Device_t *adrv9001,
                                                   adi_adrv9001_FhPerDynamicLoad_e numberHopsPerDynamicLoad,
                                                   uint32_t *bytesPerTable);
/***************************************************************************//**
 * @brief This function sets the offset frequencies for the Rx1 and Rx2 channels
 * in a frequency hopping configuration. It should be called when the
 * channel state is RF_ENABLED, and it updates the NCO with the specified
 * offset frequencies upon a subsequent GPIO pulse. This function is
 * useful for applications requiring precise frequency adjustments during
 * frequency hopping operations.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param hopSignal Specifies the hop signal associated with the Rx port of
 * interest. Must be a valid adi_adrv9001_FhHopSignal_e value.
 * @param rx1OffsetFrequencyHz The offset frequency in Hz for the Rx1 channel.
 * It should be a valid 32-bit integer representing
 * the desired frequency offset.
 * @param rx2OffsetFrequencyHz The offset frequency in Hz for the Rx2 channel.
 * It should be a valid 32-bit integer representing
 * the desired frequency offset.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_fh_RxOffsetFrequency_Set(adi_adrv9001_Device_t *adrv9001, 
		                                      adi_adrv9001_FhHopSignal_e hopSignal, 
		                                      int32_t  rx1OffsetFrequencyHz, 
		                                      int32_t  rx2OffsetFrequencyHz);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_FH_H_ */

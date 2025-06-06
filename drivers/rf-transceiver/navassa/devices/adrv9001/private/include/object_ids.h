/*! 
 * @brief    Object IDs
 *
 * @details  This file contains object IDs of the system
 */
/*******************************************************************************
  Copyright(c) 2018 Analog Devices, Inc. All Rights Reserved. This software is
  proprietary & confidential to Analog Devices, Inc. and its licensors. By using
  this software you agree to the terms of the associated Analog Devices License
  Agreement.
********************************************************************************/
#ifndef __OBJECT_IDS_H__
#define __OBJECT_IDS_H__

/* This file is open to the API, so it's better to be moved to top level of the 
   project, or a folder that holds all the API headers. 
*/


/*! 
 *  @addtogroup  mailbox
 *  @ingroup drivers
 *  @{
 */

/*! Bit position and mask of the group id */
#define BITP_GROUP_ID  (5u)
#define BITM_GROUP_ID  (0xE0u)
   
/*! Bit position and mask of the item id */
#define BITP_ITEM_ID   (0u)
#define BITM_ITEM_ID   (0x1Fu)

/***************************************************************************//**
 * @brief The `objectGroupId_e` is an enumeration that categorizes object IDs
 * into distinct groups based on their functionality and access
 * permissions. Each enumerator represents a specific group of objects,
 * such as initialization calibration algorithms, tracking calibration
 * algorithms, or system objects. The enumeration is designed to use 3
 * bits in the object ID for the group ID, allowing for a maximum of 8
 * groups. This structure helps in organizing and managing different
 * types of objects within a system, ensuring that they are accessed and
 * utilized appropriately according to their group classification.
 *
 * @param OBJ_GROUP_ID_NOT_USED Represents a group ID that is not used.
 * @param OBJ_GROUP_ID_IC Represents the group ID for initialization calibration
 * algorithm objects.
 * @param OBJ_GROUP_ID_TC Represents the group ID for tracking calibration
 * algorithm objects.
 * @param OBJ_GROUP_ID_GO Represents the group ID for objects that can only be
 * accessed by GET command.
 * @param OBJ_GROUP_ID_GS Represents the group ID for objects that can be
 * accessed by GET and SET commands.
 * @param OBJ_GROUP_ID_CFG Represents the group ID for objects that can be
 * accessed by GET or SET command with OBJID_GS_CONFIG
 * target ID.
 * @param OBJ_GROUP_ID_DRV Represents the group ID for driver objects.
 * @param OBJ_GROUP_ID_SYS Represents the group ID for system objects.
 ******************************************************************************/
typedef enum {
    OBJ_GROUP_ID_NOT_USED = 0x0, /*!< NOT USED */
    OBJ_GROUP_ID_IC       = 0x1, /*!< Group for init cal algorithm objects */
    OBJ_GROUP_ID_TC       = 0x2, /*!< Group for tracking cal algorithm objects */
    OBJ_GROUP_ID_GO       = 0x3, /*!< Group for objects that can only be accessed by GET command */
    OBJ_GROUP_ID_GS       = 0x4, /*!< Group for objects that can be accessed by GET and SET command */
    OBJ_GROUP_ID_CFG      = 0x5, /*!< Group for objects that can be accessed by GET or SET command with OBJID_GS_CONFIG target ID*/
    OBJ_GROUP_ID_DRV      = 0x6, /*!< Group for driver objects */
    OBJ_GROUP_ID_SYS      = 0x7, /*!< Group for system objects */
    
    /* We use 3 bits in the object ID for group ID so the maximum number of 
       group is 8. Therefore, do NOT create >7 group IDs in this enum. */
    
}objectGroupId_e;

/*! Macros to convert between Object Id, Object Group Id and Item Id */   
#define GENERATE_OBJID(groupId, itemId) ((((groupId) << BITP_GROUP_ID) & BITM_GROUP_ID) + (((itemId) << BITP_ITEM_ID) & BITM_ITEM_ID))
#define GET_GROUPID_FROM_OBJID(objId)   ((objectGroupId_e)(((uint8_t)(objId) & BITM_GROUP_ID) >> BITP_GROUP_ID))
#define GET_ITEMID_FROM_OBJID(objId)    (((uint8_t)(objId) & BITM_ITEM_ID) >> BITP_ITEM_ID)
   

/***************************************************************************//**
 * @brief The `objectId_e` is an enumeration that defines a comprehensive set of
 * object IDs used within a system to uniquely identify various
 * components such as initialization algorithms, tracking algorithms,
 * system sub-modules, and driver modules. Each object ID is an 8-bit
 * number, where the most significant 3 bits represent the group ID and
 * the least significant 5 bits represent the item ID within that group.
 * This structure is crucial for categorizing and managing different
 * system components, facilitating command arguments in mailboxes, and
 * aiding in error code identification. The enumeration includes object
 * IDs for initial calibration, tracking calibration, system operations,
 * and driver functionalities, each associated with specific tasks or
 * configurations within the system.
 *
 * @param OBJID_NOT_USED Represents an unused object ID with a value of 0x0UL.
 * @param OBJID_IC_TX_QEC Object ID for TX QEC Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x0u.
 * @param OBJID_IC_TX_LOL Object ID for TX LOL Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x1u.
 * @param OBJID_IC_TX_LBPD Object ID for TX Loopback Path Delay Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and item
 * ID 0x2u.
 * @param OBJID_IC_TX_DCC Object ID for TX External LO Duty Cycle Correction
 * Initial Calibration, generated with OBJ_GROUP_ID_IC
 * and item ID 0x3u.
 * @param OBJID_IC_TX_BBAF Object ID for TX BBAF Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x4u.
 * @param OBJID_IC_TX_BBAF_GD Object ID for TX BBAF Gain Delay Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and
 * item ID 0x5u.
 * @param OBJID_IC_TX_ATTD Object ID for TX Attenuation Delay Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and item
 * ID 0x6u.
 * @param OBJID_IC_TX_DAC Object ID for TX DAC Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x7u.
 * @param OBJID_IC_TX_PATH_DELAY Object ID for TX Path Delay Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and
 * item ID 0x8u.
 * @param OBJID_IC_RX_HPADC_RC Object ID for RX HP ADC Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID 0x9u.
 * @param OBJID_IC_RX_HPADC_FLASH Object ID for RX HP ADC Flash Initial
 * Calibration, generated with OBJ_GROUP_ID_IC
 * and item ID 0xAu.
 * @param OBJID_IC_RX_HPADC_DAC Object ID for RX HP ADC DAC Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID 0xBu.
 * @param OBJID_IC_RX_DCC Object ID for RX Duty Cycle Correction Initial
 * Calibration for External LO, generated with
 * OBJ_GROUP_ID_IC and item ID 0xCu.
 * @param OBJID_IC_RX_LPADC Object ID for RX LP ADC Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID 0xDu.
 * @param OBJID_IC_RX_TIA_CUTOFF Object ID for RX TIA Cutoff Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and
 * item ID 0xEu.
 * @param OBJID_IC_RX_TIA_FINE Object ID for RX TIA Fine Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID 0xFu.
 * @param OBJID_IC_RX_TCAL Object ID for RX T-Cal Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x10u.
 * @param OBJID_IC_RX_FIIC Object ID for RX FIIC Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x11u.
 * @param OBJID_IC_RX_ILB_LOD Object ID for RX Internal Loopback LO Delay
 * Initial Calibration, generated with
 * OBJ_GROUP_ID_IC and item ID 0x12u.
 * @param OBJID_IC_RX_RFDC_OFFSET Object ID for RX RFDC Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID
 * 0x13u.
 * @param OBJID_IC_RX_GPD Object ID for RX Gain Path Delay Initial Calibration,
 * generated with OBJ_GROUP_ID_IC and item ID 0x14u.
 * @param OBJID_IC_RX_DMR_PD Object ID for DMR Path Delay One-Time Initial
 * Calibration, generated with OBJ_GROUP_ID_IC and
 * item ID 0x15u.
 * @param OBJID_IC_PLL Object ID for PLL Initial Calibration, generated with
 * OBJ_GROUP_ID_IC and item ID 0x16u.
 * @param OBJID_IC_AUXPLL Object ID for AUX PLL Initial Calibration, generated
 * with OBJ_GROUP_ID_IC and item ID 0x17u.
 * @param OBJID_TC_TX_QEC Object ID for TX QEC Tracking Calibration, generated
 * with OBJ_GROUP_ID_TC and item ID 0x0u.
 * @param OBJID_TC_TX_LOL Object ID for TX LOL Tracking Calibration, generated
 * with OBJ_GROUP_ID_TC and item ID 0x1u.
 * @param OBJID_TC_TX_LBPD Object ID for TX Loopback Path Delay Tracking
 * Calibration, generated with OBJ_GROUP_ID_TC and item
 * ID 0x2u.
 * @param OBJID_TC_TX_PAC Object ID for TX PA Correction Tracking Calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0x3u.
 * @param OBJID_TC_TX_DPD Object ID for TX DPD Tracking Calibration, generated
 * with OBJ_GROUP_ID_TC and item ID 0x4u.
 * @param OBJID_TC_TX_CLGC Object ID for TX Close Loop Gain Control Tracking
 * Calibration, generated with OBJ_GROUP_ID_TC and item
 * ID 0x5u.
 * @param OBJID_TC_TX_RSV1 Object ID for reserved TX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0x6u.
 * @param OBJID_TC_TX_RSV2 Object ID for reserved TX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0x7u.
 * @param OBJID_TC_RX_HD2 Object ID for RX HD2 Tracking Calibration, generated
 * with OBJ_GROUP_ID_TC and item ID 0x8u.
 * @param OBJID_TC_RX_WBPOLY Object ID for RX Wideband Poly Tracking
 * Calibration, generated with OBJ_GROUP_ID_TC and
 * item ID 0x9u.
 * @param OBJID_TC_RX_RSV1 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0xAu.
 * @param OBJID_TC_RX_RSV2 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0xBu.
 * @param OBJID_TC_ORX_WBPOLY Object ID for ORX Wideband Poly Tracking
 * Calibration, generated with OBJ_GROUP_ID_TC and
 * item ID 0xCu.
 * @param OBJID_TC_ORX_RSV1 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0xDu.
 * @param OBJID_TC_ORX_RSV2 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0xEu.
 * @param OBJID_TC_ELB_WBPOLY Object ID for ELB Wideband Poly Tracking
 * Calibration, generated with OBJ_GROUP_ID_TC and
 * item ID 0xFu.
 * @param OBJID_TC_ELB_RSV1 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0x10u.
 * @param OBJID_TC_ELB_RSV2 Object ID for reserved RX tracking calibration,
 * generated with OBJ_GROUP_ID_TC and item ID 0x11u.
 * @param OBJID_TC_HW_RSV1 Object ID for RFDC HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x12u.
 * @param OBJID_TC_HW_RX_BBDC Object ID for RFDC HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x13u.
 * @param OBJID_TC_HW_RX_RFDC Object ID for RFDC HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x14u.
 * @param OBJID_TC_HW_RX_FIC Object ID for NBFIC HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x15u.
 * @param OBJID_TC_HW_RX_AGC Object ID for AGC HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x16u.
 * @param OBJID_TC_HW_RX_RSSI Object ID for RSSI HW tracing algorithm, generated
 * with OBJ_GROUP_ID_TC and item ID 0x17u.
 * @param OBJID_GO_TEMP_SENSOR Object ID for temperature sensor, generated with
 * OBJ_GROUP_ID_GO and item ID 0x0u.
 * @param OBJID_GO_RSSI Object ID for RSSI, generated with OBJ_GROUP_ID_GO and
 * item ID 0x1u.
 * @param OBJID_GO_RSSI_XCORR Object ID for RSSI/cross-correlator, generated
 * with OBJ_GROUP_ID_GO and item ID 0x2u.
 * @param OBJID_GO_CAL_STATUS Object ID for Calibration Status, generated with
 * OBJ_GROUP_ID_GO and item ID 0x3u.
 * @param OBJID_GO_RUN_INIT_STATUS Object ID for run initial calibration status,
 * generated with OBJ_GROUP_ID_GO and item ID
 * 0x4u.
 * @param OBJID_GO_INIT_CAL_DONE Object ID for Initial Calibration done,
 * generated with OBJ_GROUP_ID_GO and item ID
 * 0x5u.
 * @param OBJID_GO_TX_PATH_DELAY_READ Object ID for reading TX path delay,
 * generated with OBJ_GROUP_ID_GO and item ID
 * 0x6u.
 * @param OBJID_GO_RX_PATH_DELAY_READ Object ID for reading RX path delay,
 * generated with OBJ_GROUP_ID_GO and item ID
 * 0x7u.
 * @param OBJID_GO_GET_POWER_SAVING_CONFIG Object ID for reading power saving
 * configuration, generated with
 * OBJ_GROUP_ID_GO and item ID 0x8u.
 * @param OBJID_GO_GET_MONITOR_CONFIG Object ID for reading Monitor Mode
 * configuration, generated with
 * OBJ_GROUP_ID_GO and item ID 0x9u.
 * @param OBJID_GO_ILB_ELB_DIFF_MEASUREMENT Object ID for reading ILB-ELB path
 * delay measurements, generated with
 * OBJ_GROUP_ID_GO and item ID 0xAu.
 * @param OBJID_GO_GET_CURRENT_ADC_TYPE Object ID for getting the type of the
 * current ADC being used, generated with
 * OBJ_GROUP_ID_GO and item ID 0xBu.
 * @param OBJID_GO_GET_FH_HOP_TABLE_SELECT Object ID for getting the current hop
 * table selection, generated with
 * OBJ_GROUP_ID_GO and item ID 0xCu.
 * @param OBJID_GO_GET_FH_HOP_TABLE Object ID for getting a hop table, generated
 * with OBJ_GROUP_ID_GO and item ID 0xDu.
 * @param OBJID_GO_GET_FREQ_HOP_FRAME_INFO Object ID for getting hop frame
 * information, generated with
 * OBJ_GROUP_ID_GO and item ID 0xEu.
 * @param OBJID_GO_CARRIER_FREQUENCY_OF_PREVIOUS_INITCAL Object ID for getting
 * carrier frequency of
 * previous initial
 * calibration, generated
 * with OBJ_GROUP_ID_GO
 * and item ID 0xFu.
 * @param OBJID_GS_GPIO_CTRL Object ID for GPIO control, generated with
 * OBJ_GROUP_ID_GS and item ID 0x0u.
 * @param OBJID_GS_ORX_TX_CTRL Object ID for ORX TX select, generated with
 * OBJ_GROUP_ID_GS and item ID 0x1u.
 * @param OBJID_GS_TX_ATTEN Object ID for TX attenuation, generated with
 * OBJ_GROUP_ID_GS and item ID 0x2u.
 * @param OBJID_GS_CHANNEL_CARRIER_FREQUENCY Object ID for Channel carrier
 * frequency, generated with
 * OBJ_GROUP_ID_GS and item ID 0x3u.
 * @param OBJID_GS_PERFORM_ADC_SWITCH Object ID for requesting ADC switching,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0x4u.
 * @param OBJID_GS_EXT_PATH_DELAY Object ID for external path delay, generated
 * with OBJ_GROUP_ID_GS and item ID 0x5u.
 * @param OBJID_GS_TRACKCALS_CTRL Tracking calibrations control commands,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0x6u.
 * @param OBJID_GS_TRACKCALS_PENDING Tracking calibrations pending, generated
 * with OBJ_GROUP_ID_GS and item ID 0x7u.
 * @param OBJID_GS_TRACKCALS_ENABLE Tracking calibrations enable/disable,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0x8u.
 * @param OBJID_GS_ARM_FORCE_EXCEPTION ARM force exception, generated with
 * OBJ_GROUP_ID_GS and item ID 0x9u.
 * @param OBJID_GS_PARITY_ERROR_CHECK Enable or Disable parity error checking,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0xAu.
 * @param OBJID_GS_PLL_LOOPFILTER Object ID for PLL loop filter, generated with
 * OBJ_GROUP_ID_GS and item ID 0xBu.
 * @param OBJID_GS_DEBUG_LOOPFILTER Used to debug the Loopfilter algorithm,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0xCu.
 * @param OBJID_GS_DEVICE_PROFILE Set device profile, generated with
 * OBJ_GROUP_ID_GS and item ID 0xDu.
 * @param OBJID_GS_BBDC_ENABLE Disable or enable BBDC on the ORX channel,
 * generated with OBJ_GROUP_ID_GS and item ID 0xEu.
 * @param OBJID_GS_CONFIG Config a data structure, generated with
 * OBJ_GROUP_ID_GS and item ID 0xFu.
 * @param OBJID_GS_SRL_CONTROL Slew Rate Limiter control, generated with
 * OBJ_GROUP_ID_GS and item ID 0x10u.
 * @param OBJID_GS_SLICER_CONTROL Slicer Control, generated with OBJ_GROUP_ID_GS
 * and item ID 0x11u.
 * @param OBJID_GS_SLICER_GAIN Slicer Gain, generated with OBJ_GROUP_ID_GS and
 * item ID 0x12u.
 * @param OBJID_GS_SYSTEM_CONFIG Set system config, generated with
 * OBJ_GROUP_ID_GS and item ID 0x13u.
 * @param OBJID_GS_CLOCK_ENABLE Enable/Disable clock, generated with
 * OBJ_GROUP_ID_GS and item ID 0x14u.
 * @param OBJID_GS_RAM_DRIVE_AND_CAPTURE_CONFIG Configure RAM data drive and
 * capture, generated with
 * OBJ_GROUP_ID_GS and item ID
 * 0x15u.
 * @param OBJID_GS_RAM_DRIVE_AND_CAPTURE_START Start/stop RAM data drive and
 * capture, generated with
 * OBJ_GROUP_ID_GS and item ID
 * 0x16u.
 * @param OBJID_GS_BBDC_TEST_DISABLE Force BBDC on or off at any profile, for
 * testing use only, generated with
 * OBJ_GROUP_ID_GS and item ID 0x17u.
 * @param OBJID_GS_RX_FREQ_CORRECTION Rx frequency correction, generated with
 * OBJ_GROUP_ID_GS and item ID 0x18u.
 * @param OBJID_GS_INTERNAL_DEBUG Internal debug commands, generated with
 * OBJ_GROUP_ID_GS and item ID 0x19u.
 * @param OBJID_GS_DYNAMIC_PROFILE Preload dynamic components of device profile,
 * generated with OBJ_GROUP_ID_GS and item ID
 * 0x1Au.
 * @param OBJID_GS_FREQ_HOP_CONFIGURE Frequency Hopping commands, generated with
 * OBJ_GROUP_ID_GS and item ID 0x1Bu.
 * @param OBJID_GS_TDD_TIMING_PARAMS Set TDD Timing parameters, generated with
 * OBJ_GROUP_ID_GS and item ID 0x1Cu.
 * @param OBJID_GS_LOID Set LO ID command parameters, generated with
 * OBJ_GROUP_ID_GS and item ID 0x1Du.
 * @param OBJID_CFG_A0 Object ID for configuration A0, generated with
 * OBJ_GROUP_ID_CFG and item ID 0x0u.
 * @param OBJID_CFG_RADIO_EVENT Object ID for Radio events module, generated
 * with OBJ_GROUP_ID_CFG and item ID 0x1u.
 * @param OBJID_CFG_INITIAL_CALS Object ID for Initial Calibration framework
 * configuration, generated with OBJ_GROUP_ID_CFG
 * and item ID 0x2u.
 * @param OBJID_CFG_CAL_SCHEDULER Object ID for Calibration scheduler, generated
 * with OBJ_GROUP_ID_CFG and item ID 0x3u.
 * @param OBJID_CFG_HM Object ID for HM Timer Control, generated with
 * OBJ_GROUP_ID_CFG and item ID 0x4u.
 * @param OBJID_CFG_PARITY_ERROR_CHECK Configurable objects for memory refresh,
 * generated with OBJ_GROUP_ID_CFG and item
 * ID 0x5u.
 * @param OBJID_CFG_ADC_SWITCHING Object ID for ADC switching configuration,
 * generated with OBJ_GROUP_ID_CFG and item ID
 * 0x6u.
 * @param OBJID_CFG_TRACKING_CALS Set tracking calibration framework
 * configuration, generated with OBJ_GROUP_ID_CFG
 * and item ID 0x7u.
 * @param OBJID_CFG_DPD_LUT_INITIALIZATION LUT Initialization GET/SET, generated
 * with OBJ_GROUP_ID_CFG and item ID
 * 0x8u.
 * @param OBJID_CFG_DPD_FH_REGIONS Configure the DPD Model based on external
 * algorithm, generated with OBJ_GROUP_ID_CFG
 * and item ID 0x9u.
 * @param OBJID_CFG_BBDC Configure BBDC Parameters, generated with
 * OBJ_GROUP_ID_CFG and item ID 0xAu.
 * @param OBJID_CFG_TELEM Configure the telemetry logging for short or long
 * term, generated with OBJ_GROUP_ID_CFG and item ID
 * 0xBu.
 * @param OBJID_CFG_DPD_PRE_INIT_CAL Configure DPD before initial calibrations,
 * generated with OBJ_GROUP_ID_CFG and item ID
 * 0xCu.
 * @param OBJID_CFG_MONITOR_RSSI Configure RSSI settings used in monitor mode,
 * generated with OBJ_GROUP_ID_CFG and item ID
 * 0xDu.
 * @param OBJID_CFG_MONITOR_DMR_SEARCH Configure DMR search settings used in
 * monitor mode, generated with
 * OBJ_GROUP_ID_CFG and item ID 0xEu.
 * @param OBJID_CFG_LSSI_PADS_WHEN_NOT_USED Configure LSSI pads option when LSSI
 * is not in use, generated with
 * OBJ_GROUP_ID_CFG and item ID 0xFu.
 * @param OBJID_CFG_TX_INTERNAL_TONE_GENERATION Configure TX internal tone
 * generation, generated with
 * OBJ_GROUP_ID_CFG and item ID
 * 0x10u.
 * @param OBJID_CFG_PLL_CONFIG Configure PLL, generated with OBJ_GROUP_ID_CFG
 * and item ID 0x11u.
 * @param OBJID_CFG_RX_PORT_SWITCHING Configure RX port switching, generated
 * with OBJ_GROUP_ID_CFG and item ID 0x12u.
 * @param OBJID_CFG_GPIO_DEBUG_IN_STREAM Configure GPIO debug in stream,
 * generated with OBJ_GROUP_ID_CFG and
 * item ID 0x13u.
 * @param OBJID_CFG_CLK_PLL_TYPE Configure clock PLL type, generated with
 * OBJ_GROUP_ID_CFG and item ID 0x14u.
 * @param OBJID_CFG_RX_GAIN_OVER_SSI Configure RX slicer gain or AGC gain, and
 * gain index delay for 32-bit SSI, generated
 * with OBJ_GROUP_ID_CFG and item ID 0x15u.
 * @param OBJID_CFG_SPI_MASTER_CONFIG Configure SPI master to perform read/write
 * transaction with slave device, generated
 * with OBJ_GROUP_ID_CFG and item ID 0x16u.
 * @param OBJID_CFG_REFERENCE_TIMER_CONFIG Configure Reference Timer, generated
 * with OBJ_GROUP_ID_CFG and item ID
 * 0x17u.
 * @param OBJID_CFG_REFERENCE_TIMER_START Start Reference Timer, generated with
 * OBJ_GROUP_ID_CFG and item ID 0x18u.
 * @param OBJID_DRV_NCO Object ID for NCO driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x0u.
 * @param OBJID_DRV_STREAM Object ID for Stream driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x1u.
 * @param OBJID_DRV_PFIR Object ID for PFIR driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x2u.
 * @param OBJID_DRV_SSI Object ID for SSI driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x3u.
 * @param OBJID_DRV_RXGAIN Object ID for Rx Gain driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x4u.
 * @param OBJID_DRV_RXQECHW Object ID for RxQEC driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x5u.
 * @param OBJID_DRV_TXATTEN Object ID for TX ATTEN driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x6u.
 * @param OBJID_DRV_TXQEC Object ID for TxQEC driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x7u.
 * @param OBJID_DRV_DATAPATH Object ID for Datapath driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x8u.
 * @param OBJID_DRV_PLL Object ID for PLL driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x9u.
 * @param OBJID_DRV_LOGEN Object ID for Logen driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0xAu.
 * @param OBJID_DRV_SCCG Object ID for SCCG driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0xBu.
 * @param OBJID_DRV_BBDC Object ID for BBDC driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0xCu.
 * @param OBJID_DRV_DMA Object ID for DMA driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0xDu.
 * @param OBJID_DRV_LDO Object ID for LDO driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0xEu.
 * @param OBJID_DRV_TX_PD Object ID for TX Predistorter Calibration, generated
 * with OBJ_GROUP_ID_DRV and item ID 0xFu.
 * @param OBJID_DRV_RAM_DRIVE_CAPTURE Object ID for RAM Drive/Capture driver,
 * generated with OBJ_GROUP_ID_DRV and item
 * ID 0x10u.
 * @param OBJID_DRV_MEM_POWER_CTRL Object ID for Memory power control driver,
 * generated with OBJ_GROUP_ID_DRV and item ID
 * 0x11u.
 * @param OBJID_DRV_EFUSE Object ID for Efuse driver, generated with
 * OBJ_GROUP_ID_DRV and item ID 0x12u.
 * @param OBJID_SYS_BOOTUP Object ID for System bootup, generated with
 * OBJ_GROUP_ID_SYS and item ID 0x0u.
 * @param OBJID_SYS_INIT_CAL Object ID for System initial calibration wrapper,
 * generated with OBJ_GROUP_ID_SYS and item ID 0x1u.
 * @param OBJID_SYS_TRACKING_CAL Object ID for System tracking calibration
 * wrapper, generated with OBJ_GROUP_ID_SYS and
 * item ID 0x2u.
 * @param OBJID_SYS_CAL_FRAMEWORK Object ID for System calibration task
 * framework, generated with OBJ_GROUP_ID_SYS and
 * item ID 0x3u.
 * @param OBJID_SYS_CONTROL_TASK Object ID for System control task, generated
 * with OBJ_GROUP_ID_SYS and item ID 0x4u.
 * @param OBJID_SYS_MAILBOX Object ID for System mailbox, generated with
 * OBJ_GROUP_ID_SYS and item ID 0x5u.
 * @param OBJID_SYS_SCHEDULER Object ID for System scheduler, generated with
 * OBJ_GROUP_ID_SYS and item ID 0x6u.
 * @param OBJID_SYS_CHANNEL Object ID for System channel module, generated with
 * OBJ_GROUP_ID_SYS and item ID 0x7u.
 * @param OBJID_SYS_RESOURCE_MANAGER Object ID for System resource manager,
 * generated with OBJ_GROUP_ID_SYS and item ID
 * 0x8u.
 * @param OBJID_SYS_POWER_SAVING_MANAGER Object ID for System power saving
 * manager, generated with
 * OBJ_GROUP_ID_SYS and item ID 0x9u.
 * @param OBJID_SYS_MONITOR_MODE Object ID for System monitor mode, generated
 * with OBJ_GROUP_ID_SYS and item ID 0xAu.
 * @param OBJID_SYS_FREQUENCY_HOPPING Object ID for System frequency hopping,
 * generated with OBJ_GROUP_ID_SYS and item
 * ID 0xBu.
 * @param OBJID_SYS_MISC Object ID for Miscellaneous system object, generated
 * with OBJ_GROUP_ID_SYS and item ID 0x1Fu.
 ******************************************************************************/
typedef enum
{
    /* The reason to skip 0 is: 
       - in error code, the object id is used as MSB 8-bit, so a "0x0" object id 
         may generate an error code of "0x0", which is actually defined as NO_ERROR
  
       - in mailbox command, a command payload is usually "0" if it's not used, 
         so if we get a "0" payload, it's not easy to tell if it's an object or
         it's not set by BBIC.   
    */
    OBJID_NOT_USED               = 0x0UL,    
   
    /* initial calibration objects in OBJ_GROUP_ID_IC group
       1-to-1 mapping to IDs defined in initAlgoId_e
    */
    OBJID_IC_TX_QEC              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x0u),   /*!< 0x20: Object ID for TX QEC Init Cal */
    OBJID_IC_TX_LOL              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x1u),   /*!< 0x21: Object ID for TX LOL Init Cal */
    OBJID_IC_TX_LBPD             = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x2u),   /*!< 0x22: Object ID for TX Loopback path delay Init Cal */
    OBJID_IC_TX_DCC              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x3u),   /*!< 0x23: Object ID for TX ext LO duty Cycle Correction initcal */
    OBJID_IC_TX_BBAF             = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x4u),   /*!< 0x24: Object ID for TX BBAF Init Cal */
    OBJID_IC_TX_BBAF_GD          = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x5u),   /*!< 0x25: Object ID for TX BBAF Gain Delay Init Cal */
    OBJID_IC_TX_ATTD             = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x6u),   /*!< 0x26: Object ID for TX Attenuation Delay Init Cal */
    OBJID_IC_TX_DAC              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x7u),   /*!< 0x27: Object ID for TX DAC Init Cal */
    OBJID_IC_TX_PATH_DELAY       = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x8u),   /*!< 0x28: Object ID for TX Path Delay Init Cal */
    
    OBJID_IC_RX_HPADC_RC         = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x9u),   /*!< 0x29: Object ID for RX HP ADC Init Cal */
    OBJID_IC_RX_HPADC_FLASH      = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xAu),   /*!< 0x2A: Object ID for RX HP ADC Flash Init Cal */
    OBJID_IC_RX_HPADC_DAC        = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xBu),   /*!< 0x2B: Object ID for RX HP ADC DAC Init Cal */
    OBJID_IC_RX_DCC              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xCu),   /*!< 0x2C: Object ID for RX duty cycle correction Init Cal for ext LO */
    OBJID_IC_RX_LPADC            = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xDu),   /*!< 0x2D: Object ID for RX LP ADC Init Cal */
    OBJID_IC_RX_TIA_CUTOFF       = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xEu),   /*!< 0x2E: Object ID for RX TIA Cutoff Init Cal */
    OBJID_IC_RX_TIA_FINE         = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0xFu),   /*!< 0x2F: Object ID for RX TIA Fine Init Cal */
    OBJID_IC_RX_TCAL             = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x10u),  /*!< 0x30: Object ID for RX T-Cal Init Cal */
    OBJID_IC_RX_FIIC             = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x11u),  /*!< 0x31: Object ID for RX FIIC Init Cal */
    OBJID_IC_RX_ILB_LOD          = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x12u),  /*!< 0x32: Object ID for RX Internal Loopback LO Delay Init Cal */
    OBJID_IC_RX_RFDC_OFFSET      = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x13u),  /*!< 0x33: Object ID for RX RFDC Init Cal */
    OBJID_IC_RX_GPD              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x14u),  /*!< 0x34: Object ID for RX Gain Path Delay Init Cal */
    OBJID_IC_RX_DMR_PD           = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x15u),  /*!< 0x35: Object ID for DMR Path Delay one-time Init Cal */

    OBJID_IC_PLL                 = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x16u),  /*!< 0x36: Object ID for PLL Init Cal */
    OBJID_IC_AUXPLL              = GENERATE_OBJID(OBJ_GROUP_ID_IC, 0x17u),  /*!< 0x37: Object ID for AUX PLL Init Cal */


    /* tracking calibration objects in OBJ_GROUP_ID_TC group
       1-to-1 mapping to IDs defined in trackingAlgoId_e
    */
    OBJID_TC_TX_QEC              = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x0u),   /*!< 0x40: Object ID for TX QEC Tracking Cal */
    OBJID_TC_TX_LOL              = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x1u),   /*!< 0x41: Object ID for TX LOL Tracking Cal */
    OBJID_TC_TX_LBPD             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x2u),   /*!< 0x42: Object ID for TX Loopback Path Delay Tracking Cal */
    OBJID_TC_TX_PAC              = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x3u),   /*!< 0x43: Object ID for TX PA Correction Tracking Cal */
    OBJID_TC_TX_DPD              = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x4u),   /*!< 0x44: Object ID for TX DPD Tracking Cal */
    OBJID_TC_TX_CLGC             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x5u),   /*!< 0x45: Object ID for TX Close Loop Gain Control Tracking Cal */
    OBJID_TC_TX_RSV1             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x6u),   /*!< 0x46: Object ID for reserved TX tracking cal */
    OBJID_TC_TX_RSV2             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x7u),   /*!< 0x47: Object ID for reserved TX tracking cal */
    
    OBJID_TC_RX_HD2              = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x8u),   /*!< 0x48: Object ID for RX HD2 Tracking Cal */
    OBJID_TC_RX_WBPOLY           = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x9u),   /*!< 0x49: Object ID for RX Wideband Poly Tracking Cal */
    
    OBJID_TC_RX_RSV1             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xAu),   /*!< 0x4A: Object ID for reserved RX tracking cal */
    OBJID_TC_RX_RSV2             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xBu),   /*!< 0x4B: Object ID for reserved RX tracking cal */
    
    OBJID_TC_ORX_WBPOLY          = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xCu),   /*!< 0x4C: Object ID for ORX Wideband Poly Tracking Cal */
    OBJID_TC_ORX_RSV1            = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xDu),   /*!< 0x4D: Object ID for reserved RX tracking cal */
    OBJID_TC_ORX_RSV2            = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xEu),   /*!< 0x4E: Object ID for reserved RX tracking cal */
    
    OBJID_TC_ELB_WBPOLY          = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0xFu),   /*!< 0x4F: Object ID for ELB Wideband Poly Tracking Cal */    
    OBJID_TC_ELB_RSV1            = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x10u),  /*!< 0x50: Object ID for reserved RX tracking cal */
    OBJID_TC_ELB_RSV2            = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x11u),  /*!< 0x51: Object ID for reserved RX tracking cal */

    OBJID_TC_HW_RSV1             = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x12u),  /*!< 0x52: Object ID for RFDC HW tracing algorithm */
    OBJID_TC_HW_RX_BBDC          = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x13u),  /*!< 0x53: Object ID for RFDC HW tracing algorithm */
    OBJID_TC_HW_RX_RFDC          = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x14u),  /*!< 0x54: Object ID for RFDC HW tracing algorithm */
    OBJID_TC_HW_RX_FIC           = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x15u),  /*!< 0x55: Object ID for NBFIC HW tracing algorithm */ 
    OBJID_TC_HW_RX_AGC           = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x16u),  /*!< 0x56: Object ID for AGC HW tracing algorithm */  
    OBJID_TC_HW_RX_RSSI          = GENERATE_OBJID(OBJ_GROUP_ID_TC, 0x17u),  /*!< 0x57: Object ID for RSSI HW tracing algorithm */
    
    /* get only objects in OBJ_GROUP_ID_GO group */
    OBJID_GO_TEMP_SENSOR              = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x0u),   /*!< 0x60: Object ID for temp sensor   */
    OBJID_GO_RSSI                     = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x1u),   /*!< 0x61: Object ID for RSSI          */
    OBJID_GO_RSSI_XCORR               = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x2u),   /*!< 0x62: Object ID for RSSI/cross-correlator  */
    OBJID_GO_CAL_STATUS               = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x3u),   /*!< 0x63: Object ID for Cal Status    */
    OBJID_GO_RUN_INIT_STATUS          = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x4u),   /*!< 0x64: Object ID for run init cal status */
    OBJID_GO_INIT_CAL_DONE            = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x5u),   /*!< 0x65: Object ID for Init Cal done */
    OBJID_GO_TX_PATH_DELAY_READ       = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x6u),   /*!< 0x66: Object ID for reading TX path delay */
    OBJID_GO_RX_PATH_DELAY_READ       = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x7u),   /*!< 0x67: Object ID for reading RX path delay */
    OBJID_GO_GET_POWER_SAVING_CONFIG  = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x8u),   /*!< 0x68: Object ID for reading power saving confg*/
    OBJID_GO_GET_MONITOR_CONFIG       = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0x9u),   /*!< 0x69: Object ID for reading Monitor Mode config*/    
    OBJID_GO_ILB_ELB_DIFF_MEASUREMENT = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xAu),   /*!< 0x6A: Object ID for reading ILB-ELB path delay measurements */    
    OBJID_GO_GET_CURRENT_ADC_TYPE     = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xBu),   /*!< 0x6B: Object ID for getting the type of the current adc being used */        
    OBJID_GO_GET_FH_HOP_TABLE_SELECT  = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xCu),   /*!< 0x6C: Object ID for getting the current hop table selection */
    OBJID_GO_GET_FH_HOP_TABLE         = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xDu),   /*!< 0x6D: Object ID for getting a hop table */
    OBJID_GO_GET_FREQ_HOP_FRAME_INFO  = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xEu),   /*!< 0x6E: Object ID for getting hop frame information */
    OBJID_GO_CARRIER_FREQUENCY_OF_PREVIOUS_INITCAL = GENERATE_OBJID(OBJ_GROUP_ID_GO, 0xFu),   /*!< 0x6F: Object ID for getting carrier frequency of previous init cal */
    
    /* get and set objects in OBJ_GROUP_ID_GS group */
    OBJID_GS_GPIO_CTRL                 = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x0u),     /*!< 0x80: Object ID for GPIO control  */
    OBJID_GS_ORX_TX_CTRL               = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x1u),     /*!< 0x81: Object ID for ORX TX select */
    OBJID_GS_TX_ATTEN                  = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x2u),     /*!< 0x82: Object ID for Tx atten      */
    OBJID_GS_CHANNEL_CARRIER_FREQUENCY = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x3u),     /*!< 0x83: Object ID for Channel carrier frequency */
    OBJID_GS_PERFORM_ADC_SWITCH        = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x4u),     /*!< 0x84: Object ID for requesting ADC switching  */
    OBJID_GS_EXT_PATH_DELAY            = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x5u),     /*!< 0x85: Object ID for external path delay */
    OBJID_GS_TRACKCALS_CTRL            = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x6u),     /*!< 0x86: Tracking cals control commands */
    OBJID_GS_TRACKCALS_PENDING         = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x7u),     /*!< 0x87: Tracking cals pending       */
    OBJID_GS_TRACKCALS_ENABLE          = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x8u),     /*!< 0x88: Tracking cals enable/disable*/
    OBJID_GS_ARM_FORCE_EXCEPTION       = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x9u),     /*!< 0x89: ARM force exception         */
    OBJID_GS_PARITY_ERROR_CHECK        = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xAu),     /*!< 0x8A: Enable or Disable parity error checking */
    OBJID_GS_PLL_LOOPFILTER            = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xBu),     /*!< 0x8B: Object ID for PLL loop filter */
    OBJID_GS_DEBUG_LOOPFILTER          = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xCu),     /*!< 0x8C: Used to debug the Loopfilter algorithm */
    OBJID_GS_DEVICE_PROFILE            = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xDu),     /*!< 0x8D: Set device profile */
    OBJID_GS_BBDC_ENABLE               = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xEu),     /*!< 0x8E: Disable or enable BBDC on the ORX channel */
    OBJID_GS_CONFIG                    = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0xFu),     /*!< 0x8F: config a data structure */
    OBJID_GS_SRL_CONTROL               = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x10u),    /*!< 0x90: Slew Rate Limiter control */
    OBJID_GS_SLICER_CONTROL            = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x11u),    /*!< 0x91: Slicer Control */
    OBJID_GS_SLICER_GAIN               = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x12u),    /*!< 0x92: Slicer Gain */
    OBJID_GS_SYSTEM_CONFIG             = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x13u),    /*!< 0x93: Set system config */
    OBJID_GS_CLOCK_ENABLE              = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x14u),    /*!< 0x94: Enable/Disable clock */
    OBJID_GS_RAM_DRIVE_AND_CAPTURE_CONFIG   = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x15u),    /*!< 0x95: Configure  RAM data drive and capture */
    OBJID_GS_RAM_DRIVE_AND_CAPTURE_START    = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x16u),    /*!< 0x96: Start/stop RAM data drive and capture */
    OBJID_GS_BBDC_TEST_DISABLE        = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x17u),    /*!< 0x97: Force BBDC on or off at any profile, for testing use only */
    OBJID_GS_RX_FREQ_CORRECTION       = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x18u),    /*!< 0x98: Rx frequency correction */
    OBJID_GS_INTERNAL_DEBUG           = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x19u),    /*!< 0x99: Internal debug commands */
    OBJID_GS_DYNAMIC_PROFILE          = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x1Au),    /*!< 0x9A: Preload dynamic components of device profile */
    OBJID_GS_FREQ_HOP_CONFIGURE       = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x1Bu),    /*!< 0x9B: Frequency Hopping commands */
    OBJID_GS_TDD_TIMING_PARAMS        = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x1Cu),    /*!< 0x9C: Set TDD Timing parameters */
    OBJID_GS_LOID                      = GENERATE_OBJID(OBJ_GROUP_ID_GS, 0x1Du),    /*!< 0x9d: set LO ID command parameters */

    /* Configuration objects ids in OBJ_GROUP_ID_CFG group 
       1-to-1 mapping to IDs defined in cfgId_e
    */
    OBJID_CFG_A0                      = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x0u),    /*!< 0xA0: xxx        */
    OBJID_CFG_RADIO_EVENT             = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x1u),    /*!< 0xA1: Radio events module         */
    OBJID_CFG_INITIAL_CALS            = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x2u),    /*!< 0xA2: Initial Calibration framework configuration */
    OBJID_CFG_CAL_SCHEDULER           = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x3u),    /*!< 0xA3: Calibration scheduler */
    OBJID_CFG_HM                      = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x4u),    /*!< 0xA4: HM Timer Control */
    OBJID_CFG_PARITY_ERROR_CHECK      = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x5u),    /*!< 0xA5: Configurable objects for memory refresh */
    OBJID_CFG_ADC_SWITCHING           = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x6u),    /*!< 0xA6: xxx */
    OBJID_CFG_TRACKING_CALS           = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x7u),    /*!< 0xA7: Set tracking cal framework configuration */
    OBJID_CFG_DPD_LUT_INITIALIZATION  = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x8u),    /*!< 0xA8: LUT Initialization GET/SET */
    OBJID_CFG_DPD_FH_REGIONS          = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x9u),    /*!< 0xA9: Configure the DPD Model based on external algorithm */
    OBJID_CFG_BBDC                    = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xAu),    /*!< 0xAA: Configure BBDC Parameters */
    OBJID_CFG_TELEM                   = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xBu),    /*!< 0xAB: Configure the telemetry logging for short or long term */
    OBJID_CFG_DPD_PRE_INIT_CAL        = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xCu),    /*!< 0xAC: Configure DPD before initial calibrations */
    OBJID_CFG_MONITOR_RSSI            = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xDu),    /*!< 0xAD: Configure RSSI settings used in monitor mode */
    OBJID_CFG_MONITOR_DMR_SEARCH      = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xEu),    /*!< 0xAE: Configure DMR search settings used in monitor mode */
    OBJID_CFG_LSSI_PADS_WHEN_NOT_USED = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0xFu),    /*!< 0xAF: Configure LSSI pads option when LSSI is not in use */
    OBJID_CFG_TX_INTERNAL_TONE_GENERATION = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x10u),   /*!< 0xB0: Configure TX internal tone generation */
    OBJID_CFG_PLL_CONFIG              = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x11u),   /*!< 0xB1: Configure PLL */
    OBJID_CFG_RX_PORT_SWITCHING       = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x12u),   /*!< 0xB2: Configure RX port switching */
    OBJID_CFG_GPIO_DEBUG_IN_STREAM    = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x13u),   /*!< 0xB3: Configure GPIO debug in stream */
    OBJID_CFG_CLK_PLL_TYPE            = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x14u),   /*!< 0xB4: Configure clock PLL type */
    OBJID_CFG_RX_GAIN_OVER_SSI        = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x15u),   /*!< 0xB5: Configure RX slicer gain or agc gain, and gain index delay for 32-bit SSI */
    OBJID_CFG_SPI_MASTER_CONFIG       = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x16u),   /*!< 0xB6: Configure SPI master to perform read/write transaction with slave device */
    OBJID_CFG_REFERENCE_TIMER_CONFIG  = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x17u),   /*!< 0xB7: Configure Reference Timer */
    OBJID_CFG_REFERENCE_TIMER_START   = GENERATE_OBJID(OBJ_GROUP_ID_CFG, 0x18u),   /*!< 0xB8: Start Reference Timer */

    /* Driver objects in OBJ_GROUP_ID_DRV group */
    OBJID_DRV_NCO                     = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x0u),    /*!< 0xC0: NCO driver      */
    OBJID_DRV_STREAM                  = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x1u),    /*!< 0xC1: Stream driver   */
    OBJID_DRV_PFIR                    = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x2u),    /*!< 0xC2: PFIR driver     */
    OBJID_DRV_SSI                     = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x3u),    /*!< 0xC3: SSI driver      */
    OBJID_DRV_RXGAIN                  = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x4u),    /*!< 0xC4: Rx Gain driver  */
    OBJID_DRV_RXQECHW                 = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x5u),    /*!< 0xC5: RxQEC driver    */
    OBJID_DRV_TXATTEN                 = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x6u),    /*!< 0xC6: TX ATTEN driver */
    OBJID_DRV_TXQEC                   = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x7u),    /*!< 0xC7: TxQEC driver    */
    OBJID_DRV_DATAPATH                = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x8u),    /*!< 0xC8: Datapath driver */
    OBJID_DRV_PLL                     = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x9u),    /*!< 0xC9: Pll driver      */
    OBJID_DRV_LOGEN                   = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xAu),    /*!< 0xCA: Logen driver    */
    OBJID_DRV_SCCG                    = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xBu),    /*!< 0xCB: SCCG driver     */
    OBJID_DRV_BBDC                    = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xCu),    /*!< 0xCC: BBDC driver     */
    OBJID_DRV_DMA                     = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xDu),    /*!< 0xCD: DMA driver      */
    OBJID_DRV_LDO                     = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xEu),    /*!< 0xCE: LDO driver      */
    OBJID_DRV_TX_PD                   = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0xFu),    /*!< 0xCF: TX Predistorter Cal */
    OBJID_DRV_RAM_DRIVE_CAPTURE       = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x10u),   /*!< 0xD0: RAM Drive/Capture driver */
    OBJID_DRV_MEM_POWER_CTRL          = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x11u),   /*!< 0xD1: Memory power control driver */
    OBJID_DRV_EFUSE                   = GENERATE_OBJID(OBJ_GROUP_ID_DRV, 0x12u),   /*!< 0xD2: Efuse driver */

    /* System objects in OBJ_GROUP_ID_SYS group */
    OBJID_SYS_BOOTUP                  = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x0u),    /*!< 0xE0: System bootup */
    OBJID_SYS_INIT_CAL                = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x1u),    /*!< 0xE1: System init cal wrapper */
    OBJID_SYS_TRACKING_CAL            = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x2u),    /*!< 0xE2: System tracking cal wrapper */
    OBJID_SYS_CAL_FRAMEWORK           = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x3u),    /*!< 0xE3: System cal task framework */
    OBJID_SYS_CONTROL_TASK            = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x4u),    /*!< 0xE4: System control task */
    OBJID_SYS_MAILBOX                 = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x5u),    /*!< 0xE5: System mailbox */
    OBJID_SYS_SCHEDULER               = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x6u),    /*!< 0xE6: System scheduler */
    OBJID_SYS_CHANNEL                 = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x7u),    /*!< 0xE7: System channel module */
    OBJID_SYS_RESOURCE_MANAGER        = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x8u),    /*!< 0xE8: System resource manager */
    OBJID_SYS_POWER_SAVING_MANAGER    = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x9u),    /*!< 0xE9: System power saving manager */
    OBJID_SYS_MONITOR_MODE            = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0xAu),    /*!< 0xEA: System monitor mode */
    OBJID_SYS_FREQUENCY_HOPPING       = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0xBu),    /*!< 0xEB: System frequency hopping */
    OBJID_SYS_MISC                    = GENERATE_OBJID(OBJ_GROUP_ID_SYS, 0x1Fu),   /*!< 0xFF: Miscellaneous object */
    
} objectId_e;

/*! @} */
#endif /* __OBJECT_IDS_H__*/

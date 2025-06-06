/*!
 * @brief     API header file
 *            This file contains all the publicly exposed methods and data
 *            structures to interface with API.
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup ADI_AD9083
 * @{
 */
#ifndef __ADI_AD9083__
#define __ADI_AD9083__


/*============= G R O U P S ============*/
// Define groups to section APIs
/*!

* @defgroup dev_init    1.0 Device Initialization & Clock Configuration
* @brief    The initialize API functions are used to retrieve the API revision
*           for information, perform a soft reset initialize the device structure,
*           and set up all the necessary internal clocks of the chip based on the
*           desired mode of operation. These are system high level APIs that can 
*           be called alone to set up the part for the init and clock blocks without
*           needing to use any lower-level API functions
*
* @defgroup dev_config    1.1 Device Init/Deinit and Hardware Platform API
* @ingroup  dev_init
* @brief    These API functions are mid-level hardware init/deinit API calls that can be   
*           implemented as desired to get additional chip and platform information.
*
* @defgroup clock_config    1.2 Block Level Clock API
* @ingroup  dev_init
* @brief    These API functions are mid-level clock config block API calls underneath 
*           the top system high level clock configuration API function call listed in Section 1.0. 
*           These API methods typically are not necessary to call directly unless specific 
*           modification is needed by the user.
*
* @defgroup vco_adc_config    2.0 VCO ADC Configuration
* @brief    This VCO ADC API function is used to set up all of the necessary settings for
*           the VCO ADC block configuration. This is a system high level API function that
*           can be called alone and does not require any additional API calls directly to set up 
*           the VCO ADC functions and features.  All lower level calculations and setup needed
*           are covered by the mid-level VCO ADC config block helper APIs listed in Section 2.1.
*
* @defgroup vco_adc_helper    2.1 VCO ADC Block Helper APIs
* @brief    These API functions are mid-level VCO ADC config block API calls underneath
*           the top system high level VCO ADC configuration API function call listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific 
*           modification is needed by the user.
*
* @defgroup adc_dp_config    3.0 ADC Rx Digital Datapath Configuration
* @brief    This ADC Rx Digital Datapath API function is used to set up all of the necessary settings for
*           the ADC Rx digital datapath blocks configuration. This is a system high level API function that
*           can be called alone and does not require any additional API calls directly to set up 
*           the ADC digital datapath functions and features.  All lower level calculations and setup needed
*           are covered by the mid-level VCO ADC config block helper APIs listed in Section 3.1.
*
* @defgroup adc_dp_helper    3.1 ADC Rx Digital Datapath Helper APIs
* @brief    These API functions are mid-level ADC Rx digital datapath config block API calls
*           underneath the top system high level ADC Rx digital datapath configuration API function
*           call listed in Section 3.0. These API methods typically are not necessary to call 
*           directly unless specific modification is needed by the user.
*
* @defgroup adc_link_setup    4.0 SERDES Link Establishment & Monitoring
* @brief    This SERDES Link Establishment API function is used to set up all of the necessary settings for
*           the ADC SERDES blocks configuration. This is a system high level API function that
*           can be called alone and does not require any additional API calls directly to set up 
*           the SERDES link functions and features.  All lower level calculations and setup needed
*           are covered by the mid-level SERDES config block helper APIs listed in Section 4.1.
*
* @defgroup adc_serdes_helper    4.1 SERDES Helper APIs
* @brief    These API functions are mid-level SERDES link config block API calls underneath 
*           the top system high level SERDES link configuration API function call listed in 
*           Section 4.0. These API methods typically are not necessary to call directly unless 
*           specific modification is needed by the user.
****
*
* @defgroup appendix    Appendix: Additional Digital Feature Blocks
* @brief    This Appendix section includes additional API function calls for other features
*           and functional blocks that exist in the ADC that are not covered in previous sections.
*
* @defgroup appdx_tm    A1.0 Test Modes
* @ingroup  appendix
* @brief    These API methods cover various test modes inside the ADC. These functions are not
*           required to be called in order to fully set up the ADC chip in a working state.
*
* @defgroup appdx_tempsense    A2.0 Temperature Sensor
* @ingroup  appendix
* @brief    These API methods set up and configure the on-chip temperature sensor.
*
* @defgroup appdx_spi       A3.0  SPI Controls
* @ingroup  appendix
* @brief    These API methods set up and configure lower-level SPI calls not directly needed to 
*           configure the setup of the ADC.
*/

/*============= I N C L U D E S ============*/
#include "adi_cms_api_common.h"

/*============= D E F I N E S ==============*/
#define R1
#define AD9083_ID                     0x9083
#define AD9083_REF_CLK_FREQ_HZ_MIN    50000000ull     /*!< 50 MHz */
#define AD9083_REF_CLK_FREQ_HZ_MAX    500000000ull    /*!< 500MHz */
#define AD9083_ADC_CLK_FREQ_HZ_MIN    1000000000ull   /*!< 1  GHz */
#define AD9083_ADC_CLK_FREQ_HZ_MAX    2000000000ull   /*!< 2  GHz */

#define AD9083_JESD_SER_COUNT         4

/***************************************************************************//**
 * @brief The `adi_ad9083_reset_e` is an enumeration that defines the different
 * types of reset operations available for the AD9083 device. It includes
 * options for performing a soft reset, a hard reset, and combinations of
 * reset with initialization. This enumeration is used to specify the
 * desired reset operation when interacting with the device's API,
 * allowing for flexible control over the device's reset behavior.
 *
 * @param AD9083_SOFT_RESET Represents a soft reset operation.
 * @param AD9083_HARD_RESET Represents a hard reset operation.
 * @param AD9083_SOFT_RESET_AND_INIT Represents a soft reset followed by
 * initialization.
 * @param AD9083_HARD_RESET_AND_INIT Represents a hard reset followed by
 * initialization.
 ******************************************************************************/
typedef enum {
    AD9083_SOFT_RESET = 0,                            /*!< Soft reset */
    AD9083_HARD_RESET = 1,                            /*!< Hard reset */
    AD9083_SOFT_RESET_AND_INIT = 2,                   /*!< Soft reset then init */
    AD9083_HARD_RESET_AND_INIT = 3                    /*!< Hard reset then init */
}adi_ad9083_reset_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_low_power_mode_e` is an enumeration that defines the
 * different power modes available for the AD9083 device. It includes
 * three modes: `AD9083_POWER_ON` for full operation, `AD9083_STANDBY`
 * for reduced power consumption with some blocks still active, and
 * `AD9083_FULL_POWER_DOWN` for minimal power usage with most blocks
 * powered down. This enumeration is used to manage the power state of
 * the device, optimizing for either performance or power savings as
 * needed.
 *
 * @param AD9083_POWER_ON Full on mode where all blocks are active.
 * @param AD9083_STANDBY Standby mode where some blocks remain active.
 * @param AD9083_FULL_POWER_DOWN Full power down mode where almost all blocks
 * are inactive.
 ******************************************************************************/
typedef enum {
    AD9083_POWER_ON = 0,                              /*!< Full on */
    AD9083_STANDBY = 1,                               /*!< Some of blocks are keep running */
    AD9083_FULL_POWER_DOWN = 2,                       /*!< Almost all of blocks are held in power down */
}adi_ad9083_low_power_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_adc_pd_pin_mode_e` is an enumeration that defines the
 * different power-down modes for the AD9083 ADC's power-down pin. Each
 * enumerator specifies a different level of power-down functionality,
 * ranging from disabling the power-down pin entirely to disabling the
 * ADC clock, VTI bias, and master bias block. This allows for flexible
 * power management of the ADC depending on the desired operational
 * state.
 *
 * @param AD9083_ADC_PD_DISABLE Power down pin disabled.
 * @param AD9083_ADC_PD_CLK Power down pin disables clock path only.
 * @param AD9083_ADC_PD_CLK_VTI Power down pin disables ADC clock and VTI bias.
 * @param AD9083_ADC_PD_CLK_VTI_MASTER_BIAS Power down pin disables ADC clock,
 * VTI bias, and ADC master bias block.
 ******************************************************************************/
typedef enum {
    AD9083_ADC_PD_DISABLE = 0,                        /*!< Power down pin disabled */
    AD9083_ADC_PD_CLK = 1,                            /*!< Power down pin disables clock path only */
    AD9083_ADC_PD_CLK_VTI = 2,                        /*!< Power down pin disables adc clock and vti bias */
    AD9083_ADC_PD_CLK_VTI_MASTER_BIAS = 3,            /*!< Power down pin disables adc clock, vti bias and adc masterbias block */
}adi_ad9083_adc_pd_pin_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_adc_term_res_e` is an enumeration that defines the
 * possible termination resistance settings for the AD9083 ADC. It
 * includes options for open termination, as well as specific resistances
 * of 200 Ohms and 100 Ohms. This enumeration is used to configure the
 * termination resistance of the ADC input, which can affect signal
 * integrity and performance.
 *
 * @param AD9083_ADC_TERM_RES_OPEN Represents an open termination resistance.
 * @param AD9083_ADC_TERM_RES_200 Represents a termination resistance of 200
 * Ohms.
 * @param AD9083_ADC_TERM_RES_100 Represents a termination resistance of 100
 * Ohms.
 ******************************************************************************/
typedef enum {
    AD9083_ADC_TERM_RES_OPEN = 0,                     /*!< Open */
    AD9083_ADC_TERM_RES_200 = 1,                      /*!< 200 Ohm */
    AD9083_ADC_TERM_RES_100 = 2,                      /*!< 100 Ohm */
}adi_ad9083_adc_term_res_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_cic_dec_rate_e` is an enumeration that defines the
 * possible decimation rates for the CIC (Cascaded Integrator-Comb)
 * filter in the AD9083 device. It provides three options for decimation:
 * by 4, by 8, and by 16, allowing the user to select the appropriate
 * rate for their application needs.
 *
 * @param AD9083_CIC_DEC_4 Decimation by 4.
 * @param AD9083_CIC_DEC_8 Decimation by 8.
 * @param AD9083_CIC_DEC_16 Decimation by 16.
 ******************************************************************************/
typedef enum {
    AD9083_CIC_DEC_4 = 0,                             /*!< Decimation by 4 */
    AD9083_CIC_DEC_8 = 1,                             /*!< Decimation by 8 */
    AD9083_CIC_DEC_16 = 2,                            /*!< Decimation by 16 */
}adi_ad9083_cic_dec_rate_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_j_dec_rate_e` is an enumeration that defines various
 * decimation rates for the AD9083 device. Each enumerator represents a
 * specific decimation factor, allowing the user to select how much the
 * input signal should be decimated. This is useful in digital signal
 * processing applications where reducing the sample rate is necessary to
 * meet system requirements or to reduce data throughput.
 *
 * @param AD9083_J_DEC_1 Bypass decimation.
 * @param AD9083_J_DEC_4 Decimation by 4.
 * @param AD9083_J_DEC_8 Decimation by 8.
 * @param AD9083_J_DEC_16 Decimation by 16.
 * @param AD9083_J_DEC_12 Decimation by 12.
 * @param AD9083_J_DEC_24 Decimation by 24.
 * @param AD9083_J_DEC_10 Decimation by 10.
 * @param AD9083_J_DEC_20 Decimation by 20.
 * @param AD9083_J_DEC_30 Decimation by 30.
 * @param AD9083_J_DEC_40 Decimation by 40.
 * @param AD9083_J_DEC_60 Decimation by 60.
 ******************************************************************************/
typedef enum {
    AD9083_J_DEC_1 = 0,                               /*!< Bypass */
    AD9083_J_DEC_4 = 1,                               /*!< Decimation by 4 */
    AD9083_J_DEC_8 = 2,                               /*!< Decimation by 8 */
    AD9083_J_DEC_16 = 3,                              /*!< Decimation by 16 */
    AD9083_J_DEC_12 = 6,                              /*!< Decimation by 12 */
    AD9083_J_DEC_24 = 7,                              /*!< Decimation by 24 */
    AD9083_J_DEC_10 = 9,                              /*!< Decimation by 10 */
    AD9083_J_DEC_20 = 10,                             /*!< Decimation by 20 */
    AD9083_J_DEC_30 = 14,                             /*!< Decimation by 30 */
    AD9083_J_DEC_40 = 11,                             /*!< Decimation by 40 */
    AD9083_J_DEC_60 = 15,                             /*!< Decimation by 60 */
}adi_ad9083_j_dec_rate_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_h_dec_rate_e` is an enumeration that defines various
 * decimation rates for the AD9083 device. Each enumerator represents a
 * specific decimation factor, which is used to reduce the sampling rate
 * of a signal by the specified factor. This enumeration is part of the
 * configuration settings for the AD9083, allowing users to select the
 * appropriate decimation rate for their application needs.
 *
 * @param AD9083_H_DEC_1 Decimation by 1.
 * @param AD9083_H_DEC_12 Decimation by 12.
 * @param AD9083_H_DEC_14 Decimation by 14.
 * @param AD9083_H_DEC_16 Decimation by 16.
 * @param AD9083_H_DEC_18 Decimation by 18.
 * @param AD9083_H_DEC_24 Decimation by 24.
 * @param AD9083_H_DEC_28 Decimation by 28.
 * @param AD9083_H_DEC_32 Decimation by 32.
 * @param AD9083_H_DEC_36 Decimation by 36.
 ******************************************************************************/
typedef enum {
    AD9083_H_DEC_1  = 1,                              /*!< Decimation by 1  */
    AD9083_H_DEC_12 = 12,                             /*!< Decimation by 12 */
    AD9083_H_DEC_14 = 14,                             /*!< Decimation by 14 */
    AD9083_H_DEC_16 = 16,                             /*!< Decimation by 16 */
    AD9083_H_DEC_18 = 18,                             /*!< Decimation by 18 */
    AD9083_H_DEC_24 = 24,                             /*!< Decimation by 24 */
    AD9083_H_DEC_28 = 28,                             /*!< Decimation by 28 */
    AD9083_H_DEC_32 = 32,                             /*!< Decimation by 32 */
    AD9083_H_DEC_36 = 36,                             /*!< Decimation by 36 */
}adi_ad9083_h_dec_rate_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_g_dec_rate_e` is an enumeration that defines the
 * possible decimation rates for the G decimation stage in the AD9083
 * device. It includes options for no decimation (NA), decimation by 8,
 * and decimation by 16, allowing for flexible configuration of the
 * device's data processing capabilities.
 *
 * @param AD9083_G_DEC_NA Represents a non-applicable or default state with a
 * value of 0.
 * @param AD9083_G_DEC_8 Represents a decimation rate of 8 with a value of 8.
 * @param AD9083_G_DEC_16 Represents a decimation rate of 16 with a value of 16.
 ******************************************************************************/
typedef enum {
    AD9083_G_DEC_NA = 0,                              /*!< NA */
    AD9083_G_DEC_8  = 8,                              /*!< Decimation by 8  */
    AD9083_G_DEC_16 = 16,                             /*!< Decimation by 16 */
}adi_ad9083_g_dec_rate_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_jesd_rx_prbs_test_data_src_e` is an enumeration that
 * defines the possible sources of test data for the JESD RX PRBS
 * (Pseudo-Random Binary Sequence) test in the AD9083 device. It allows
 * the selection between using lane data or sample data as the test data
 * source, which is crucial for testing and validating the integrity of
 * data transmission in JESD interfaces.
 *
 * @param AD9083_JESD_RX_PRBS_TEST_DATA_SRC_LANE Lane Data As Test Data Source.
 * @param AD9083_JESD_RX_PRBS_TEST_DATA_SRC_SAMPLE Sample Data As Test Data
 * Source (M0 Only).
 ******************************************************************************/
typedef enum {
    AD9083_JESD_RX_PRBS_TEST_DATA_SRC_LANE = 0x0,     /*!< Lane Data As Test Data Source */
    AD9083_JESD_RX_PRBS_TEST_DATA_SRC_SAMPLE = 0x1    /*!< Sample Data As Test Data Source (M0 Only) */
}adi_ad9083_jesd_rx_prbs_test_data_src_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_jesd_rx_prbs_test_mode_e` is an enumeration that
 * defines the different test modes available for the JESD RX PRBS
 * (Pseudo-Random Binary Sequence) testing in the AD9083 device. It
 * includes options to disable the test mode or to enable specific PRBS
 * test patterns such as PRBS7, PRBS9, PRBS15, and PRBS31, which are used
 * to verify the integrity of data transmission over the JESD interface.
 *
 * @param AD9083_JESD_RX_PRBS_TEST_MODE_OFF Disable PRBS Test Mode.
 * @param AD9083_JESD_RX_PRBS_TEST_MODE_PRBS7 Enable PRBS7 test mode.
 * @param AD9083_JESD_RX_PRBS_TEST_MODE_PRBS9 Enable PRBS9 test mode.
 * @param AD9083_JESD_RX_PRBS_TEST_MODE_PRBS15 Enable PRBS15 test mode.
 * @param AD9083_JESD_RX_PRBS_TEST_MODE_PRBS31 Enable PRBS31 test mode.
 ******************************************************************************/
typedef enum {
    AD9083_JESD_RX_PRBS_TEST_MODE_OFF = 0x0,          /*!< Disable PRBS Test Mode */
    AD9083_JESD_RX_PRBS_TEST_MODE_PRBS7 = 0x1,        /*!< PRBS7 */
    AD9083_JESD_RX_PRBS_TEST_MODE_PRBS9 = 0x2,        /*!< PRBS9 */
    AD9083_JESD_RX_PRBS_TEST_MODE_PRBS15 = 0x3,       /*!< PRBS15 */
    AD9083_JESD_RX_PRBS_TEST_MODE_PRBS31 = 0x4        /*!< PRBS31 */
}adi_ad9083_jesd_rx_prbs_test_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_jesd_tx_test_data_src_e` is an enumeration that
 * defines the possible sources of test data for the JESD transmitter in
 * the AD9083 device. It allows the selection of different data sources
 * such as sample data, PHY data, or scrambler input data to be used as
 * test data, facilitating various testing and diagnostic operations
 * within the JESD transmission path.
 *
 * @param AD9083_JESD_TX_TEST_DATA_SAMPLE Sample Data As Test Data Source.
 * @param AD9083_JESD_TX_TEST_DATA_PHY PHY Data As Test Data Source.
 * @param AD9083_JESD_TX_TEST_DATA_SCRAMBLER_INPUT Scrambler Input Data As Data
 * Source.
 ******************************************************************************/
typedef enum {
    AD9083_JESD_TX_TEST_DATA_SAMPLE = 0x0,            /*!< Sample Data As Test Data Source */
    AD9083_JESD_TX_TEST_DATA_PHY = 0x1,               /*!< PHY Data As Test Data Source */
    AD9083_JESD_TX_TEST_DATA_SCRAMBLER_INPUT = 0x2    /*!< Scrambler Input Data As Data Source */
}adi_ad9083_jesd_tx_test_data_src_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_jesd_tx_test_mode_e` is an enumeration that defines
 * various test modes for the JESD transmitter in the AD9083 device. Each
 * enumerator represents a specific test mode that can be used to verify
 * the functionality and performance of the JESD transmitter. These modes
 * include disabling the test mode, using checker board patterns, word
 * toggling, and various pseudo-random binary sequence (PN) modes, as
 * well as user-defined data patterns. This enumeration is crucial for
 * testing and validating the JESD transmitter's operation in different
 * scenarios.
 *
 * @param AD9083_JESD_TX_TEST_MODE_DISABLED Disable Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_CHECKER_BOARD Checker Board Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_WORD_TOGGLE Word Toggle Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_PN31 PN31 Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_PN15 PN15 Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_PN7 PN7 Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_RAMP Ramp Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_USER_REPEAT Repeated User Data Test Mode.
 * @param AD9083_JESD_TX_TEST_MODE_USER_SINGLE Single Time User Data Test Mode.
 ******************************************************************************/
typedef enum {
    AD9083_JESD_TX_TEST_MODE_DISABLED = 0x0,          /*!< Disable Test Mode */
    AD9083_JESD_TX_TEST_MODE_CHECKER_BOARD = 0x1,     /*!< Checker Board Test Mode */
    AD9083_JESD_TX_TEST_MODE_WORD_TOGGLE = 0x2,       /*!< Word Toggle Test Mode */
    AD9083_JESD_TX_TEST_MODE_PN31 = 0x3,              /*!< PN31 Test Mode */
    AD9083_JESD_TX_TEST_MODE_PN15 = 0x5,              /*!< PN15 Test Mode */
    AD9083_JESD_TX_TEST_MODE_PN7 = 0x7,               /*!< PN7  Test Mode */
    AD9083_JESD_TX_TEST_MODE_RAMP = 0x8,              /*!< Ramp Test Mode */
    AD9083_JESD_TX_TEST_MODE_USER_REPEAT = 0xE,       /*!< Repeated User Data Test Mode */
    AD9083_JESD_TX_TEST_MODE_USER_SINGLE = 0xF        /*!< Single Time User Data Test Mode */
}adi_ad9083_jesd_tx_test_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_rx_tmode_resolution_e` is an enumeration that defines
 * different resolution modes for the I/Q channel in the AD9083 device.
 * Each enumerator represents a specific bit resolution, ranging from
 * 12-bit to 16-bit, which can be used to configure the test mode
 * resolution of the ADC. This allows for flexibility in testing and
 * optimizing the ADC's performance based on the desired resolution.
 *
 * @param AD9083_RX_TMODE_RES16 Resolution 16-bit.
 * @param AD9083_RX_TMODE_RES15 Resolution 15-bit.
 * @param AD9083_RX_TMODE_RES14 Resolution 14-bit.
 * @param AD9083_RX_TMODE_RES13 Resolution 13-bit.
 * @param AD9083_RX_TMODE_RES12 Resolution 12-bit.
 ******************************************************************************/
typedef enum {
    AD9083_RX_TMODE_RES16 = 0,                        /*!< Resolution 16-bit */
    AD9083_RX_TMODE_RES15 = 1,                        /*!< Resolution 15-bit */
    AD9083_RX_TMODE_RES14 = 2,                        /*!< Resolution 14-bit */
    AD9083_RX_TMODE_RES13 = 3,                        /*!< Resolution 13-bit */
    AD9083_RX_TMODE_RES12 = 4                         /*!< Resolution 12-bit */
}adi_ad9083_rx_tmode_resolution_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_datapath_mode_e` is an enumeration that defines
 * various data path configurations for the AD9083 device. Each
 * enumerator represents a specific sequence of processing blocks that
 * the ADC data passes through, such as CIC (Cascaded Integrator-Comb),
 * NCO (Numerically Controlled Oscillator), and decimation stages (J, G,
 * H), before reaching the output. This allows for flexible configuration
 * of the ADC's digital signal processing path to suit different
 * application requirements.
 *
 * @param AD9083_DATAPATH_ADC_CIC ADC -> CIC -> output.
 * @param AD9083_DATAPATH_ADC_CIC_NCO_J ADC -> CIC -> NCO -> J -> output.
 * @param AD9083_DATAPATH_ADC_CIC_J ADC -> CIC -> J -> output.
 * @param AD9083_DATAPATH_ADC_J ADC -> J -> output.
 * @param AD9083_DATAPATH_ADC_CIC_NCO_G ADC -> CIC -> NCO -> G -> output.
 * @param AD9083_DATAPATH_ADC_CIC_NCO_G_H ADC -> CIC -> NCO -> G -> H output.
 ******************************************************************************/
typedef enum {
    AD9083_DATAPATH_ADC_CIC         = 1,              /*!< ADC -> CIC -> output */
    AD9083_DATAPATH_ADC_CIC_NCO_J   = 2,              /*!< ADC -> CIC -> NCO -> J -> output */
    AD9083_DATAPATH_ADC_CIC_J       = 3,              /*!< ADC -> CIC -> J -> output */
    AD9083_DATAPATH_ADC_J           = 4,              /*!< ADC -> J -> output */
    AD9083_DATAPATH_ADC_CIC_NCO_G   = 5,              /*!< ADC -> CIC -> NCO -> G -> output */
    AD9083_DATAPATH_ADC_CIC_NCO_G_H = 6               /*!< ADC -> CIC -> NCO -> G -> H output */
}adi_ad9083_datapath_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_ser_swing_e` is an enumeration that defines the
 * possible swing settings for the JESD serializer in the AD9083 device.
 * Each enumerator represents a specific voltage swing level, measured in
 * millivolts (mV), that can be configured for the serializer. This
 * allows for the adjustment of the output signal strength to match the
 * requirements of the connected system or to optimize performance under
 * different conditions.
 *
 * @param AD9083_SER_SWING_1000 1000 mV Swing
 * @param AD9083_SER_SWING_850 850 mV Swing
 * @param AD9083_SER_SWING_750 750 mV Swing
 * @param AD9083_SER_SWING_500 500 mV Swing
 ******************************************************************************/
typedef enum {
    AD9083_SER_SWING_1000 = 0,                        /*!< 1000 mV Swing */
    AD9083_SER_SWING_850 = 1,                         /*!< 850 mV Swing */
    AD9083_SER_SWING_750 = 2,                         /*!< 750 mV Swing */
    AD9083_SER_SWING_500 = 3                          /*!< 500 mV Swing */
}adi_ad9083_ser_swing_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_ser_pre_emp_e` is an enumeration that defines the pre-
 * emphasis settings for the JESD serializer in the AD9083 device. It
 * provides three levels of pre-emphasis: 0 dB, 3 dB, and 6 dB, which are
 * used to adjust the signal strength to compensate for signal loss over
 * transmission lines. This setting is crucial for optimizing signal
 * integrity in high-speed data transmission applications.
 *
 * @param AD9083_SER_PRE_EMP_0DB Represents a 0 dB pre-emphasis setting.
 * @param AD9083_SER_PRE_EMP_3DB Represents a 3 dB pre-emphasis setting.
 * @param AD9083_SER_PRE_EMP_6DB Represents a 6 dB pre-emphasis setting.
 ******************************************************************************/
typedef enum {
    AD9083_SER_PRE_EMP_0DB = 0,                       /*!< 0 db Pre-Emphasis */
    AD9083_SER_PRE_EMP_3DB = 1,                       /*!< 3 db Pre-Emphasis */
    AD9083_SER_PRE_EMP_6DB = 2                        /*!< 6 db Pre-Emphasis */
}adi_ad9083_ser_pre_emp_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_ser_post_emp_e` is an enumeration that defines various
 * post-emphasis settings for a JESD serializer. Each enumerator
 * corresponds to a specific level of post-emphasis, measured in decibels
 * (dB), which can be applied to the serializer's output signal to
 * improve signal integrity over transmission lines. This is particularly
 * useful in high-speed data communication to compensate for signal
 * degradation.
 *
 * @param AD9083_SER_POST_EMP_0DB Represents a 0 dB post-emphasis setting.
 * @param AD9083_SER_POST_EMP_3DB Represents a 3 dB post-emphasis setting.
 * @param AD9083_SER_POST_EMP_6DB Represents a 6 dB post-emphasis setting.
 * @param AD9083_SER_POST_EMP_9DB Represents a 9 dB post-emphasis setting.
 * @param AD9083_SER_POST_EMP_12DB Represents a 12 dB post-emphasis setting.
 ******************************************************************************/
typedef enum {
    AD9083_SER_POST_EMP_0DB = 0,                      /*!< 0 db Post-Emphasis */
    AD9083_SER_POST_EMP_3DB = 1,                      /*!< 3 db Post-Emphasis */
    AD9083_SER_POST_EMP_6DB = 2,                      /*!< 6 db Post-Emphasis */
    AD9083_SER_POST_EMP_9DB = 3,                      /*!< 9 db Post-Emphasis */
    AD9083_SER_POST_EMP_12DB = 4                      /*!< 12 db Post-Emphasis */
}adi_ad9083_ser_post_emp_e;

/***************************************************************************//**
 * @brief The `adi_ad9083_indv_ser_settings_t` structure is used to define
 * individual serializer settings for the JESD interface in the AD9083
 * device. It includes settings for swing, pre-emphasis, and post-
 * emphasis, which are critical for configuring the signal integrity and
 * performance of the serializer lanes. These settings allow for fine-
 * tuning of the serializer's output characteristics to match the
 * requirements of the specific application or system configuration.
 *
 * @param swing_setting Specifies the swing setting for the JESD serializer.
 * @param pre_emp_setting Specifies the pre-emphasis setting for the JESD
 * serializer.
 * @param post_emp_setting Specifies the post-emphasis setting for the JESD
 * serializer.
 ******************************************************************************/
typedef struct {
    adi_ad9083_ser_swing_e    swing_setting;
    adi_ad9083_ser_pre_emp_e  pre_emp_setting;
    adi_ad9083_ser_post_emp_e post_emp_setting;
}adi_ad9083_indv_ser_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9083_ser_settings_t` structure is designed to encapsulate
 * the settings for the JESD serializer in the AD9083 device. It includes
 * an array of individual serializer lane settings, allowing for
 * configuration of up to four lanes, each with its own swing, pre-
 * emphasis, and post-emphasis settings. Additionally, it contains a
 * transmit invert mask, which is used to control the inversion of the
 * transmitted data across these lanes. This structure is crucial for
 * configuring the serializer's behavior to match the specific
 * requirements of the application and the connected hardware.
 *
 * @param indv_ser_lane_settings An array of individual serializer lane
 * settings, each of type
 * adi_ad9083_indv_ser_settings_t, for 4 lanes.
 * @param tx_invert_mask A uint8_t value representing a mask for inverting the
 * transmit data.
 ******************************************************************************/
typedef struct {
    adi_ad9083_indv_ser_settings_t indv_ser_lane_settings[4];
    uint8_t                      tx_invert_mask;
}adi_ad9083_ser_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9083_prbs_test_t` structure is used to store the results of
 * a PRBS (Pseudo-Random Binary Sequence) test in the AD9083 device. It
 * contains fields to track the error count during the test
 * (`phy_prbs_err_cnt`), the pass status of the test (`phy_prbs_pass`),
 * and the source error count (`phy_src_err_cnt`). This structure is
 * essential for evaluating the integrity and performance of data
 * transmission in the device.
 *
 * @param phy_prbs_err_cnt PRBS Test Error Count.
 * @param phy_prbs_pass PRBS Test Status.
 * @param phy_src_err_cnt PRBS Test Source Error Count.
 ******************************************************************************/
typedef struct {
    uint32_t phy_prbs_err_cnt;                       /*!< PRBS Test Error Count */
    uint8_t  phy_prbs_pass;                          /*!< PRBS Test Status */
    uint8_t  phy_src_err_cnt;                        /*!< PRBS Test Source Error Count */
}adi_ad9083_prbs_test_t;

/***************************************************************************//**
 * @brief The `adi_ad9083_hal_t` structure is a hardware abstraction layer (HAL)
 * for the AD9083 device, encapsulating configuration settings and
 * function pointers for SPI communication, signal configuration, and
 * hardware control. It includes members for user data, SPI interface
 * configurations, signal types, and function pointers for various
 * hardware operations such as SPI transfers, delays, initialization, de-
 * initialization, logging, and pin control. This structure serves as an
 * interface between the device and the underlying hardware, allowing for
 * flexible and customizable hardware interactions.
 *
 * @param user_data A pointer to user-defined data.
 * @param sdo Configures the SPI interface for 3/4 wire mode.
 * @param msb Configures the SPI interface for MSB/LSB bit order.
 * @param addr_inc Configures the SPI interface address increment.
 * @param syncoutb Specifies the desired signal type for the SYNCOUTB signal.
 * @param sysref Specifies the desired input coupling for the SysRef signal.
 * @param spi_xfer Function pointer to the HAL SPI access function.
 * @param delay_us Function pointer to the HAL delay function.
 * @param hw_open Function pointer to the HAL initialization function.
 * @param hw_close Function pointer to the HAL de-initialization function.
 * @param log_write Function pointer to the HAL log write function.
 * @param tx_en_pin_ctrl Function pointer to the HAL TX_ENABLE pin control
 * function.
 * @param reset_pin_ctrl Function pointer to the HAL RESETB pin control
 * function.
 ******************************************************************************/
typedef struct {
    void *                    user_data;
    adi_cms_spi_sdo_config_e  sdo;                   /*!< SPI interface 3/4 wire mode configuration */
    adi_cms_spi_msb_config_e  msb;                   /*!< SPI interface MSB/LSB bit order configuration */
    adi_cms_spi_addr_inc_e    addr_inc;              /*!< SPI interface address increment configuration */
    adi_cms_signal_type_e     syncoutb;              /*!< Desired Signal type for SYNCOUTB signal */
    adi_cms_signal_coupling_e sysref;                /*!< Desired Input coupling for SysRef signal */

    adi_spi_xfer_t            spi_xfer;              /*!< Function Pointer to HAL SPI access function */
    adi_delay_us_t            delay_us;              /*!< Function Pointer to HAL delay function */
    adi_hw_open_t             hw_open;               /*!< Function Pointer to HAL initialization function */
    adi_hw_close_t            hw_close;              /*!< Function Pointer to HAL De-initialization function */
    adi_log_write_t           log_write;             /*!< Function Pointer to HAL log write function */
    adi_tx_en_pin_ctrl_t      tx_en_pin_ctrl;        /*!< Function Pointer to HAL TX_ENABLE Pin Control function */
    adi_reset_pin_ctrl_t      reset_pin_ctrl;        /*!< Function Pointer to HAL RESETB Pin Control Function */
}adi_ad9083_hal_t;

/***************************************************************************//**
 * @brief The `adi_ad9083_info_t` structure is used to store essential frequency
 * information for the AD9083 device, specifically the device clock input
 * frequency and the ADC clock frequency. These frequencies are critical
 * for configuring and operating the device within its specified limits,
 * ensuring proper functionality and performance. The structure provides
 * a straightforward way to encapsulate these two frequency parameters,
 * which are fundamental to the device's clock configuration and
 * operation.
 *
 * @param dev_freq_hz Device Clock Input frequency in KHz, with a valid range
 * from 50MHz to 1GHz.
 * @param adc_freq_hz ADC Clock Frequency in KHz, with a valid range from 1GHz
 * to 2GHz.
 ******************************************************************************/
typedef struct {
    uint32_t dev_freq_hz;                            /*!< Device Clock Input frequency in KHz. Valid range 50MHz to 1GHz */
    uint32_t adc_freq_hz;                            /*!< ADC Clock Frequency in KHz. Valid range 1GHz to 2GHz */
}adi_ad9083_info_t;

/***************************************************************************//**
 * @brief The `adi_ad9083_device_t` structure is a composite data type that
 * encapsulates the essential information required to interface with and
 * manage an AD9083 device. It includes `hal_info`, which provides the
 * necessary hardware abstraction layer details for communication and
 * control, and `dev_info`, which stores internal device-specific
 * information like clock frequencies. This structure is central to the
 * device's initialization, configuration, and operation within the API.
 *
 * @param hal_info Contains hardware abstraction layer information for the
 * device.
 * @param dev_info Holds internal information about the device, such as clock
 * frequencies.
 ******************************************************************************/
typedef struct {
    adi_ad9083_hal_t  hal_info;
    adi_ad9083_info_t dev_info;
}adi_ad9083_device_t;

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/*===== 1 . 0   D E V I C E   I N I T   &  C L O C K I N G =====*/
/***************************************************************************//**
 * @brief Use this function to obtain the major, minor, and release candidate
 * revision numbers of the API associated with the specified device. This
 * function is essential for verifying compatibility and ensuring that
 * the correct version of the API is being used. It must be called with
 * valid pointers for all parameters, and the device must be properly
 * initialized before calling this function.
 *
 * @param device Pointer to the device structure. Must not be null. The caller
 * retains ownership.
 * @param rev_major Pointer to a variable where the major revision number will
 * be stored. Must not be null.
 * @param rev_minor Pointer to a variable where the minor revision number will
 * be stored. Must not be null.
 * @param rev_rc Pointer to a variable where the release candidate revision
 * number will be stored. Must not be null.
 * @return Returns an integer status code. Returns API_CMS_ERROR_OK on success,
 * or a negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_api_revision_get(adi_ad9083_device_t *device, uint8_t *rev_major, uint8_t *rev_minor, uint8_t *rev_rc);

/***************************************************************************//**
 * @brief This function is used to reset the AD9083 device, either through a
 * soft or hard reset, with optional initialization. It should be called
 * when a reset of the device is required, such as during initialization
 * or error recovery. The function requires a valid device structure and
 * a reset operation type. It handles invalid parameters by returning an
 * error code. The function may perform a reset via SPI or hardware pin,
 * and optionally reinitialize the device, depending on the operation
 * specified.
 *
 * @param device A pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param operation An enum value of type adi_ad9083_reset_e indicating the type
 * of reset operation to perform. Valid values are
 * AD9083_SOFT_RESET, AD9083_HARD_RESET,
 * AD9083_SOFT_RESET_AND_INIT, and AD9083_HARD_RESET_AND_INIT.
 * Invalid values result in an error return.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_reset(adi_ad9083_device_t *device, adi_ad9083_reset_e operation);

/***************************************************************************//**
 * @brief This function initializes the AD9083 device, setting up necessary
 * configurations such as SPI, checking power status, and logging system
 * information. It should be called to prepare the device for operation
 * after it has been instantiated. The function assumes that the device
 * pointer is valid and non-null. It logs the system's endianness and
 * data type sizes, and performs a series of checks and configurations to
 * ensure the device is ready for use.
 *
 * @param device Pointer to an adi_ad9083_device_t structure representing the
 * device to be initialized. Must not be null. The function will
 * return an error if this pointer is invalid.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_init(adi_ad9083_device_t *device);

/***************************************************************************//**
 * @brief This function is used to configure the clock circuitry of the AD9083
 * device based on the desired ADC and reference clock frequencies. It
 * must be called after the device has been initialized. The function
 * checks the validity of the provided clock frequencies and configures
 * the device accordingly. If the reference clock frequency is greater
 * than or equal to the maximum allowed, the PLL is bypassed. The
 * function returns an error if the clock frequencies are outside the
 * valid range or if the PLL fails to lock.
 *
 * @param device Pointer to the device structure. Must not be null.
 * @param adc_clk_hz Desired ADC clock frequency in Hz. Must be between 1 GHz
 * and 2 GHz. Invalid values result in an error return.
 * @param ref_clk_hz Reference clock frequency in Hz. Must be between 50 MHz and
 * 500 MHz if the PLL is used. If greater than or equal to 500
 * MHz, the PLL is bypassed. Invalid values result in an error
 * return.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure, such as invalid parameters or PLL not
 * locking.
 ******************************************************************************/
int32_t adi_ad9083_device_clock_config_set(adi_ad9083_device_t *device, uint64_t adc_clk_hz, uint64_t ref_clk_hz);







/*===== 1 . 1   D E V I C E   I N I T / D E I N I T  &  H W  P L A T F O R M =====*/
/***************************************************************************//**
 * @brief Use this function to obtain the chip type, product ID, product grade,
 * and device revision information from the specified device. This
 * function is essential for verifying the identity and version of the
 * device being used. It must be called with valid pointers to both the
 * device structure and the chip ID structure. The function will return
 * an error if any of the pointers are null, ensuring that the caller
 * provides valid memory locations for the output data.
 *
 * @param device Pointer to the device structure. Must not be null. The caller
 * retains ownership and is responsible for ensuring the device is
 * properly initialized before calling this function.
 * @param chip_id Pointer to a variable of type adi_cms_chip_id_t where the chip
 * identification data will be stored. Must not be null. The
 * caller is responsible for allocating memory for this structure
 * before calling the function.
 * @return Returns an integer status code. Returns API_CMS_ERROR_OK on success,
 * or a negative error code if the operation fails. The chip_id
 * structure is populated with the chip identification data on success.
 ******************************************************************************/
int32_t adi_ad9083_device_chip_id_get(adi_ad9083_device_t *device, adi_cms_chip_id_t *chip_id);

/***************************************************************************//**
 * @brief This function is used to obtain the silicon DIE ID of the specified
 * device. It is essential to ensure that both the device and the id
 * pointer are valid and not null before calling this function. The
 * function will return an error if either pointer is null. This function
 * is typically used to verify the identity of the device in use, which
 * can be crucial for debugging or logging purposes.
 *
 * @param device Pointer to the device structure. Must not be null. The caller
 * retains ownership.
 * @param id Pointer to a uint8_t variable where the DIE ID will be stored. Must
 * not be null. The caller retains ownership.
 * @return Returns an int32_t indicating success or failure. A non-negative
 * value indicates success, while a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9083_device_die_id_get(adi_ad9083_device_t *device, uint8_t *id);

/***************************************************************************//**
 * @brief Use this function to initialize and open the hardware platform
 * associated with the given device. It is essential to ensure that the
 * device pointer is not null before calling this function. This function
 * should be called before performing any hardware-specific operations on
 * the device. If the operation is successful, the function returns a
 * success code; otherwise, it returns an error code indicating the
 * failure.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The function will return an error if
 * this pointer is null.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure.
 ******************************************************************************/
 int32_t adi_ad9083_device_hw_open(adi_ad9083_device_t *device);

/***************************************************************************//**
 * @brief This function is used to close the hardware platform associated with
 * the given device. It should be called when the device is no longer
 * needed or before shutting down the system to ensure proper resource
 * management. The function requires a valid device structure and will
 * return an error if the device pointer is null. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param device A pointer to the adi_ad9083_device_t structure representing the
 * device to be closed. Must not be null. The function will return
 * an error if this parameter is invalid.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of API_CMS_ERROR_OK indicates success,
 * while a negative value indicates failure.
 ******************************************************************************/
 int32_t adi_ad9083_device_hw_close(adi_ad9083_device_t *device);

/***************************************************************************//**
 * @brief This function is used to de-initialize the AD9083 device, performing
 * both a hardware and software reset to return the device to its default
 * state. It should be called when the device is no longer needed or
 * before re-initializing it to ensure a clean state. The function
 * requires a valid device structure pointer and will return an error if
 * the pointer is null. It is important to ensure that any ongoing
 * operations with the device are completed or safely stopped before
 * calling this function.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_deinit(adi_ad9083_device_t *device);

/***************************************************************************//**
 * @brief This function configures the SPI interface settings for the AD9083
 * device based on the parameters specified in the device's hardware
 * abstraction layer (HAL) structure. It should be called to ensure the
 * SPI interface is set up correctly before performing any SPI
 * transactions with the device. The function checks for a null device
 * pointer and logs the function call. It returns an error code if the
 * configuration fails.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The function will return an error if
 * this pointer is null.
 * @return Returns an int32_t status code: API_CMS_ERROR_OK if the configuration
 * is successful, or a negative error code if it fails.
 ******************************************************************************/
int32_t adi_ad9083_device_spi_config(adi_ad9083_device_t *device);

/***************************************************************************//**
 * @brief This function writes an 8-bit value to a specified SPI register
 * address on the device. It is used to configure or control the device
 * by setting specific register values. The function must be called with
 * a valid device pointer, and the device should be properly initialized
 * before calling this function. It returns an error code if the
 * operation fails, which can be used to diagnose issues with the SPI
 * communication or device state.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null, and the device should be initialized
 * before use.
 * @param addr The 16-bit address of the SPI register to which the data will be
 * written. The address must be within the valid range of the
 * device's register map.
 * @param data The 8-bit data value to be written to the specified SPI register
 * address.
 * @return Returns an int32_t error code. A non-negative value indicates
 * success, while a negative value indicates failure, with specific
 * error codes providing details on the type of failure.
 ******************************************************************************/
int32_t adi_ad9083_device_spi_register_set(adi_ad9083_device_t *device, uint16_t addr, uint8_t data);

/***************************************************************************//**
 * @brief Use this function to read an 8-bit value from a specific SPI register
 * of the AD9083 device. It requires a valid device structure and a non-
 * null pointer for storing the read data. This function is typically
 * used when you need to retrieve configuration or status information
 * from the device. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the device structure representing the AD9083 device.
 * Must not be null.
 * @param addr The 16-bit address of the SPI register to read from. Valid range
 * depends on the device's register map.
 * @param data Pointer to an 8-bit variable where the read value will be stored.
 * Must not be null.
 * @return Returns an integer status code. A non-negative value indicates
 * success, while a negative value indicates failure.
 ******************************************************************************/
int32_t adi_ad9083_device_spi_register_get(adi_ad9083_device_t *device, uint16_t addr, uint8_t *data);

/***************************************************************************//**
 * @brief This function is used to verify the power status of the AD9083 device
 * by checking various power supply registers. It should be called to
 * ensure that all necessary power supplies are active before proceeding
 * with further device operations. If any power supply is found to be
 * off, an error message is logged. This function must be called with a
 * valid device structure pointer, and it assumes that the device has
 * been properly initialized. It returns a status code indicating success
 * or failure of the operation.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. If null, the function will return an
 * error.
 * @return Returns an integer status code. API_CMS_ERROR_OK indicates success,
 * while a negative value indicates failure, such as a null pointer or
 * an error in reading power status registers.
 ******************************************************************************/
int32_t adi_ad9083_device_power_status_check(adi_ad9083_device_t *device);








/*===== 1 . 2   B L O C K  L E V E L  C L O C K  A P I =====*/
/***************************************************************************//**
 * @brief This function checks the lock status of the phase-locked loop (PLL) in
 * the specified device. It is useful for determining whether the PLL has
 * successfully locked, which is a critical step in ensuring stable clock
 * operation. The function must be called with a valid device structure
 * that has been properly initialized. It writes the lock status to the
 * provided status pointer, where the status is represented by specific
 * bits indicating fast and slow lock status. Ensure that the status
 * pointer is not null to avoid undefined behavior.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param status A pointer to a uint8_t variable where the PLL lock status will
 * be stored. Must not be null. The function writes the lock
 * status to this location.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. Returns API_CMS_ERROR_OK on success, or a negative error
 * code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_pll_lock_status_get(adi_ad9083_device_t *device, uint8_t *status);

/***************************************************************************//**
 * @brief This function is used to control the PLL test output of the AD9083
 * device. It should be called when there is a need to enable or disable
 * the PLL test output for testing purposes. The function requires a
 * valid device structure pointer and an enable flag. It returns an error
 * if the device pointer is null, ensuring that the function is only
 * called with a properly initialized device structure.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null, as a null pointer will result in an
 * error.
 * @param enable A uint8_t value where 0 means disable and 1 means enable the
 * PLL test output. Any other values are treated as invalid and
 * will not perform any action.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_device_pll_test_enable_set(adi_ad9083_device_t *device, uint8_t enable);







/*===== 2 . 0   V C O  A D C  C O N F I G U R A T I O N =====*/
/***************************************************************************//**
 * @brief This function is used to configure the ADC settings of the AD9083
 * device, including setting the low-pass filter cutoff frequency, input
 * full-scale voltage, termination resistance, enabling the high-
 * performance path, setting the noise backoff, and specifying the
 * maximum input frequency. It must be called with a valid device
 * structure and appropriate parameter values within specified ranges.
 * The function performs parameter validation and returns an error code
 * if any parameter is out of range. It is essential to ensure that the
 * device is properly initialized before calling this function.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param vmax The differential peak-to-peak input full-scale voltage in
 * millivolts. Valid range is 500 to 2000 mV.
 * @param fc The low-pass filter -3dB frequency in Hz. Must be a valid frequency
 * value for the device.
 * @param rterm Termination resistance setting. Acceptable values are 0 (open),
 * 1 (200 ohm), or 2 (100 ohm).
 * @param en_hp Enable high-performance path. Acceptable values are 0 (disable)
 * or 1 (enable).
 * @param backoff The backoff in dB in terms of noise. Valid range is 0 to 18000
 * dB.
 * @param finmax The maximum input frequency in Hz. Must be a valid frequency
 * value for the device.
 * @return Returns an int32_t error code: API_CMS_ERROR_OK on success, or a
 * negative value indicating failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_adc_config_set(adi_ad9083_device_t *device, uint32_t vmax, uint32_t fc, uint8_t rterm, uint8_t en_hp, uint32_t backoff, uint64_t finmax);







/*===== 2 . 1   V C O  A D C  H E L P E R  A P I S =====*/
/***************************************************************************//**
 * @brief This function is used to configure the Voltage-to-Current (VTI)
 * settings for the AD9083 device, which involves setting parameters
 * related to the low-pass filter, input full-scale, and other
 * operational modes. It should be called when specific VTI
 * configurations are needed for the device's operation. The function
 * requires valid input parameters and will return an error code if any
 * parameter is invalid or if the operation fails.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param fc The -3dB frequency of the low-pass filter. Must be a valid
 * frequency value.
 * @param vmax The differential peak-to-peak input full-scale voltage. Must be a
 * valid voltage value.
 * @param en_hp A flag to enable (1) or disable (0) the high-pass replica path
 * in the VCO ADC. Must be either 0 or 1.
 * @param orc1mode Indicates if the ORC1 method is enabled. Must be a valid mode
 * value.
 * @param backoff The backoff in dB in terms of noise. Must be a valid backoff
 * value.
 * @param finmax The maximum input frequency. Must be a valid frequency value.
 * @param kgain Pointer to a uint32_t where the calculated gain value will be
 * stored. Must not be null.
 * @param bcenter_os Pointer to a uint8_t where the calculated center offset
 * value will be stored. Must not be null.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_adc_vti_set(adi_ad9083_device_t *device, uint32_t fc, uint32_t vmax, uint8_t en_hp, 
    uint32_t orc1mode, uint32_t backoff, uint64_t finmax, uint32_t *kgain, uint8_t *bcenter_os);

/***************************************************************************//**
 * @brief This function is used to calibrate the ADC of the AD9083 device by
 * setting specific calibration parameters. It should be called when the
 * ADC requires calibration to ensure accurate performance. The function
 * requires a valid device structure and specific calibration values for
 * the ring oscillator center current, KGAIN register, and BCENTER
 * offset. It is important to ensure that the device pointer is not null
 * before calling this function. The function returns an error code if
 * any operation fails during the calibration process.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param gcenter A uint16_t value representing the ring oscillator center
 * current. Valid range is not specified, but it should be within
 * the device's operational limits.
 * @param kgain A uint32_t value representing the KGAIN register value. Valid
 * range is not specified, but it should be within the device's
 * operational limits.
 * @param bcenter_os A uint8_t value representing the BCENTER offset. Valid
 * range is not specified, but it should be within the
 * device's operational limits.
 * @return Returns an int32_t error code: API_CMS_ERROR_OK on success, or a
 * negative value indicating failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_adc_cal_set(adi_ad9083_device_t *device, uint16_t gcenter, uint32_t kgain, uint8_t bcenter_os);







/*===== 3 . 0   A D C  R X  D A T A P A T H  C O N F I G U R A T I O N =====*/
/***************************************************************************//**
 * @brief This function sets up the ADC Rx digital datapath configuration for
 * the specified device, allowing for various modes and decimation
 * settings. It should be called when configuring the ADC for specific
 * signal processing requirements. The function requires a valid device
 * pointer and mode, and it expects decimation and NCO frequency arrays
 * to be properly initialized. Invalid parameters will result in an error
 * return code.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param mode Specifies the datapath mode using adi_ad9083_datapath_mode_e.
 * Must be a valid mode enumeration value.
 * @param dec Array of four uint8_t values representing decimation rates for
 * CIC, J, G, and H stages. Values must be within valid ranges for
 * each stage.
 * @param nco_freq_hz Array of three uint64_t values representing NCO
 * frequencies in Hz. Must be properly initialized with
 * desired frequencies.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_datapath_config_set(adi_ad9083_device_t *device, adi_ad9083_datapath_mode_e mode, uint8_t dec[4], uint64_t nco_freq_hz[3]);







/*===== 3 . 1   A D C  R X  D A T A P A T H  H E L P E R  A P I S =====*/
/***************************************************************************//**
 * @brief This function is used to control the state of the ADC Rx digital
 * datapath within the AD9083 device. It can be called to either enable
 * or disable the datapath, depending on the operational requirements.
 * This function should be used when configuring the device for different
 * modes of operation, ensuring that the datapath is only active when
 * needed. It is important to ensure that the `device` parameter is a
 * valid pointer to an initialized `adi_ad9083_device_t` structure before
 * calling this function.
 *
 * @param device Pointer to the `adi_ad9083_device_t` structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param enable A `uint8_t` value where 0 disables and 1 enables the ADC Rx
 * digital datapath. Values other than 0 or 1 are invalid and may
 * result in undefined behavior.
 * @return Returns an `int32_t` indicating success with `API_CMS_ERROR_OK` or an
 * error code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_datapath_enable_set(adi_ad9083_device_t *device, uint8_t enable);

/***************************************************************************//**
 * @brief This function retrieves and calculates the total decimation factor for
 * the ADC Rx datapath of the AD9083 device. It should be called when the
 * total decimation factor is needed for configuration or analysis
 * purposes. The function requires a valid device structure and a pointer
 * to store the resulting decimation factor. It handles null pointers and
 * invalid parameters by returning an error code. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param device Pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param total_dec Pointer to a uint16_t variable where the total decimation
 * factor will be stored. Must not be null. The function writes
 * the calculated decimation factor to this location.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_datapath_total_dec_get(adi_ad9083_device_t *device, uint16_t *total_dec);

/***************************************************************************//**
 * @brief This function configures the test mode resolution for the I and Q
 * channels of the ADC in the AD9083 device. It should be used when you
 * need to set specific resolutions for testing purposes. The function
 * requires a valid device pointer and resolution values for both I and Q
 * channels. It returns an error code if the device pointer is null or if
 * there is an issue setting the resolutions.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param i_resolution The desired test mode resolution for the I channel,
 * specified as an adi_ad9083_rx_tmode_resolution_e enum
 * value. Valid values range from 0 (16-bit) to 4 (12-bit).
 * @param q_resolution The desired test mode resolution for the Q channel,
 * specified as an adi_ad9083_rx_tmode_resolution_e enum
 * value. Valid values range from 0 (16-bit) to 4 (12-bit).
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_test_mode_res_set(adi_ad9083_device_t *device, adi_ad9083_rx_tmode_resolution_e i_resolution, adi_ad9083_rx_tmode_resolution_e q_resolution);

/***************************************************************************//**
 * @brief This function configures the test mode type for the I and Q channels
 * of the ADC in the AD9083 device. It should be used when you need to
 * set specific test modes for these channels, typically for testing or
 * diagnostic purposes. The function requires a valid device structure
 * pointer and specific test mode types for both I and Q channels. It is
 * important to ensure that the device pointer is not null before calling
 * this function, as a null pointer will result in an error. The function
 * returns a status code indicating success or failure of the operation.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param i_type Unsigned 8-bit integer representing the test mode type for the
 * I channel. Valid values depend on the specific test modes
 * supported by the device.
 * @param q_type Unsigned 8-bit integer representing the test mode type for the
 * Q channel. Valid values depend on the specific test modes
 * supported by the device.
 * @return Returns an int32_t status code. API_CMS_ERROR_OK indicates success,
 * while a negative value indicates failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_test_mode_type_set(adi_ad9083_device_t *device, uint8_t i_type, uint8_t q_type);

/***************************************************************************//**
 * @brief This function configures the ADC test mode selection for all channels
 * on the specified device. It should be used when you need to enable or
 * disable test modes across all channels simultaneously. The function
 * must be called with a valid device pointer, and it assumes that the
 * device has been properly initialized. The enable parameter determines
 * whether the test mode is activated or deactivated. The function
 * returns an error code if the operation fails, which should be checked
 * to ensure successful execution.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null, and the device should be initialized
 * before calling this function. If null, the function will return
 * an error.
 * @param enable A uint8_t value indicating whether to enable (non-zero value)
 * or disable (zero) the test mode for all channels. Values
 * outside the range of 0 or 1 are treated as enabling the test
 * mode.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. API_CMS_ERROR_OK is returned on success, and a negative
 * error code is returned on failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_test_mode_sel_all_set(adi_ad9083_device_t *device, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the test mode for the ADC
 * in the AD9083 device. It should be called when you need to switch the
 * ADC into a test mode for diagnostic or validation purposes. The
 * function requires a valid device structure pointer and a flag
 * indicating whether to enable or disable the test mode. It is important
 * to ensure that the device pointer is not null before calling this
 * function, as a null pointer will result in an error. The function
 * returns a status code indicating success or failure of the operation.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param enable A uint8_t value where 0 disables the test mode and 1 enables
 * it. Values outside this range may result in undefined behavior.
 * @return Returns an int32_t status code. API_CMS_ERROR_OK indicates success,
 * while a negative value indicates failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_test_mode_enable_set(adi_ad9083_device_t *device, uint8_t enable);

/***************************************************************************//**
 * @brief This function sets the frequency tuning word (FTW) for a specified
 * Numerically Controlled Oscillator (NCO) within the AD9083 device. It
 * is used to adjust the frequency output of the NCO by specifying the
 * FTW value. The function must be called with a valid device pointer and
 * a valid NCO index (0, 1, or 2). If an invalid index is provided, the
 * function returns an error. This function should be used when precise
 * frequency control of the NCO is required in the device's operation.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param index The index of the NCO to configure. Valid values are 0, 1, or 2.
 * If an invalid index is provided, the function returns an error.
 * @param ftw The frequency tuning word to set for the specified NCO. It is an
 * 8-bit value.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_rx_nco_ftw_set (adi_ad9083_device_t *device, uint8_t index, uint8_t ftw);

/***************************************************************************//**
 * @brief This function configures the AD9083 device to operate in a specified
 * low power mode. It should be used when there is a need to reduce power
 * consumption by switching the device to standby or full power down
 * modes. The function must be called with a valid device pointer and a
 * mode value that is within the defined range of the
 * adi_ad9083_low_power_mode_e enumeration. If the mode is invalid, the
 * function will return an error. This function is typically used in
 * power management scenarios where different operational states of the
 * device are required.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null, as the function will return an error
 * if it is.
 * @param mode Specifies the desired low power mode for the device. Must be a
 * valid value from the adi_ad9083_low_power_mode_e enumeration. If
 * the mode is greater than AD9083_FULL_POWER_DOWN, the function
 * will return an error.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_device_low_power_mode_set(adi_ad9083_device_t *device, adi_ad9083_low_power_mode_e mode);

/***************************************************************************//**
 * @brief Use this function to obtain the current ADC clock frequency from the
 * device. It is essential to ensure that both the `device` and
 * `adc_clk_hz` pointers are valid and not null before calling this
 * function. This function is typically used when you need to verify or
 * log the current ADC clock frequency for diagnostic or configuration
 * purposes. The function will return an error if any of the input
 * pointers are null.
 *
 * @param device Pointer to the `adi_ad9083_device_t` structure representing the
 * device. Must not be null.
 * @param adc_clk_hz Pointer to a `uint64_t` variable where the ADC clock
 * frequency in Hz will be stored. Must not be null.
 * @return Returns an `int32_t` indicating success or failure.
 * `API_CMS_ERROR_OK` is returned on success, and a negative value
 * indicates an error.
 ******************************************************************************/
int32_t adi_ad9083_rx_adc_clk_freq_get(adi_ad9083_device_t *device, uint64_t *adc_clk_hz);










/*===== 4 . 0   S E R D E S  L I N K  =====*/
/***************************************************************************//**
 * @brief This function sets up and starts the JESD transmitter (JTx) link for
 * the AD9083 device. It should be called after the device has been
 * initialized and configured with the necessary clock settings. The
 * function configures various JESD parameters, powers up the necessary
 * components, and enables the JTx link. It is essential to ensure that
 * the `device` and `jtx_param` pointers are valid and properly
 * initialized before calling this function. The function returns an
 * error code if any step in the setup process fails.
 *
 * @param device Pointer to an `adi_ad9083_device_t` structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param jtx_param Pointer to an `adi_cms_jesd_param_t` structure containing
 * the JESD transmitter parameters. Must not be null. The
 * caller retains ownership.
 * @return Returns an `int32_t` indicating success or failure.
 * `API_CMS_ERROR_OK` is returned on success, while a negative value
 * indicates an error.
 ******************************************************************************/
int32_t adi_ad9083_jtx_startup(adi_ad9083_device_t *device, adi_cms_jesd_param_t *jtx_param);








/*===== 4 . 1   S E R D E S  H E L P E R  A P I S  =====*/
/***************************************************************************//**
 * @brief Use this function to obtain the current status of the JESD TX link for
 * the specified device. It provides detailed status information,
 * including QBF status, frame sync status, PLL lock status, phase
 * establishment, and invalid mode indication. This function must be
 * called with valid pointers for both the device and status parameters.
 * It is essential to ensure that the device is properly initialized
 * before calling this function. The function will return an error code
 * if any issues are encountered during execution.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param status Pointer to a uint16_t variable where the status will be stored.
 * Must not be null. The function writes the status information to
 * this location.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_link_status_get(adi_ad9083_device_t *device, uint16_t *status);

/***************************************************************************//**
 * @brief Use this function to check if the JESD transmitter LCPLL is locked. It
 * is essential to ensure that the device and the pointer to store the
 * lock status are valid before calling this function. This function
 * should be called when you need to verify the lock status of the LCPLL,
 * which is crucial for ensuring proper operation of the JESD
 * transmitter.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The function will return an error if
 * this pointer is null.
 * @param pll_locked Pointer to a uint8_t variable where the lock status will be
 * stored. Must not be null. The function will return an error
 * if this pointer is null.
 * @return Returns an int32_t indicating success or failure. A return value of
 * API_CMS_ERROR_OK indicates success, while a negative value indicates
 * an error.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_lcpll_status_get(adi_ad9083_device_t *device, uint8_t *pll_locked);

/***************************************************************************//**
 * @brief This function is used to configure the JESD TX link settings on the
 * specified device using the provided JESD parameters. It must be called
 * with a valid device structure and a JESD parameter structure that
 * specifies the desired configuration. The function performs parameter
 * validation and will return an error if the parameters are invalid,
 * such as when the number of lanes exceeds the supported maximum. This
 * function is typically used during the setup phase of the device to
 * ensure the JESD link is configured correctly before data transmission.
 *
 * @param device A pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param jesd_param A pointer to the adi_cms_jesd_param_t structure containing
 * the JESD configuration parameters. Must not be null and
 * must have valid values, particularly jesd_l must not exceed
 * 4. The caller retains ownership.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the configuration fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_link_config_set(adi_ad9083_device_t *device, adi_cms_jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function sets the crossbar mapping for the JESD Tx lanes on the
 * specified device. It is used to map logical lanes to physical lanes,
 * allowing for flexible lane configuration. The function must be called
 * with a valid device pointer and an array of four logical lane indices.
 * It is important to ensure that the device has been properly
 * initialized before calling this function. The function will return an
 * error code if the device pointer is null or if any internal
 * configuration step fails.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param logic_lanes Array of four uint8_t values representing the logical lane
 * indices to be mapped. Each value should be within the
 * valid range for logical lanes.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_lanes_xbar_set(adi_ad9083_device_t *device, uint8_t logic_lanes[4]);

/***************************************************************************//**
 * @brief This function is used to enable a specified number of JESD TX lanes on
 * the AD9083 device. It should be called when configuring the JESD
 * interface to ensure the correct lanes are powered on. The function
 * requires a valid device pointer and a lane number indicating how many
 * lanes to enable, starting from lane 0. The function will return an
 * error if the lane number exceeds the maximum supported lanes, which is
 * 4. It is important to ensure the device is properly initialized before
 * calling this function.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param lane_num An unsigned 8-bit integer specifying the number of lanes to
 * enable, starting from lane 0. Valid values are 0 to 4. If the
 * value exceeds 4, the function returns an error.
 * @return Returns an int32_t indicating success or failure. API_CMS_ERROR_OK is
 * returned on success, and API_CMS_ERROR_INVALID_PARAM is returned if
 * lane_num is greater than 4.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_lanes_enable_set(adi_ad9083_device_t *device, uint8_t lane_num);

/***************************************************************************//**
 * @brief This function is used to enable the JESD TX PHY for a specific lane on
 * the AD9083 device. It should be called when you need to activate the
 * JESD transmission on a particular lane. The function requires a valid
 * device structure and a lane number between 1 and 4. If the lane number
 * is outside this range, the function will return an error. Ensure that
 * the device pointer is not null before calling this function.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param lane_num The lane number to enable, valid values are 1, 2, 3, or 4. If
 * an invalid lane number is provided, the function returns an
 * error.
 * @return Returns an int32_t indicating success or failure. API_CMS_ERROR_OK is
 * returned on success, and API_CMS_ERROR_INVALID_PARAM is returned if
 * the lane number is invalid.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_phy_enable_set(adi_ad9083_device_t *device, uint8_t lane_num);

/***************************************************************************//**
 * @brief This function configures the JESD TX lane IDs for the specified
 * device. It should be called when setting up the JESD interface to
 * ensure that each lane is correctly identified. The function requires a
 * valid device pointer and an array of four lane IDs. It is important to
 * ensure that the device pointer is not null before calling this
 * function. The function will return an error code if any operation
 * fails during the configuration process.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param lid Array of four uint8_t values representing the lane IDs to be set.
 * Each value corresponds to a specific lane.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_lane_id_set(adi_ad9083_device_t *device, uint8_t lid[4]);

/***************************************************************************//**
 * @brief Use this function to disable all JESD TX lanes on the specified
 * device. This is typically done when the lanes are no longer needed or
 * before reconfiguring them. The function must be called with a valid
 * device pointer, and it will return an error code if the operation
 * fails. Ensure that the device is properly initialized before calling
 * this function.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The function will return an error if
 * this pointer is null.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or a
 * negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_lanes_disable_all_set(adi_ad9083_device_t *device);












/*===== A 1 . 0   T E S T  M O D E S   =====*/
/***************************************************************************//**
 * @brief This function is used to initiate a Pseudo-Random Binary Sequence
 * (PRBS) test on the JESD transmitter physical layer of the AD9083
 * device. It is typically used for testing and validation purposes to
 * ensure the integrity of data transmission. The function requires a
 * valid device structure and a PRBS pattern identifier. It should be
 * called when the device is properly initialized and configured. The
 * function will return an error if the PRBS pattern is invalid or if the
 * device pointer is null.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param prbs_pattern An enum value of type adi_cms_jesd_prbs_pattern_e
 * indicating the PRBS pattern to use. Valid values are
 * PRBS7, PRBS15, and PRBS31. Invalid values will result in
 * an error.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_phy_prbs_test(adi_ad9083_device_t *device, adi_cms_jesd_prbs_pattern_e prbs_pattern);

/***************************************************************************//**
 * @brief This function is used to control the continuous D21.5 test mode on a
 * specific JESD transmitter lane of the AD9083 device. It is typically
 * used in testing scenarios to verify the integrity of the JESD link.
 * The function requires a valid device pointer and a lane identifier
 * within the range of available lanes. The enable parameter determines
 * whether the test mode is activated or deactivated. The function must
 * be called with a properly initialized device structure.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param lane_id The identifier of the lane to configure. Valid values are
 * typically 0 to 3, corresponding to the available lanes on the
 * device.
 * @param enable A boolean value where 0 disables and 1 enables the continuous
 * D21.5 test mode for the specified lane.
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_jesd_tx_continuous_d215_enable_set(adi_ad9083_device_t *device, uint8_t lane_id, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the ADC test mode by setting user-defined I
 * and Q patterns. It is typically used for testing and validation
 * purposes to ensure the ADC is functioning correctly. The function
 * requires a valid device structure pointer and two arrays of 8 elements
 * each for the I and Q patterns. It must be called with a properly
 * initialized device structure. If the device pointer is null, the
 * function will not proceed and will return an error. The function
 * returns an integer status code indicating success or failure.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param i_pattern An array of 8 uint16_t values representing the I pattern to
 * be set. The array must be properly initialized before
 * calling the function.
 * @param q_pattern An array of 8 uint16_t values representing the Q pattern to
 * be set. The array must be properly initialized before
 * calling the function.
 * @return Returns an int32_t status code. API_CMS_ERROR_OK indicates success,
 * while a negative value indicates failure.
 ******************************************************************************/
int32_t adi_ad9083_rx_test_mode_usr_pattern_set(adi_ad9083_device_t *device, uint16_t i_pattern[8], uint16_t q_pattern[8]);









/*===== A 2 . 0   T E M P E R A T U R E  S E N S O R   =====*/
/***************************************************************************//**
 * @brief Use this function to control the operational state of the on-chip
 * temperature sensor. It is typically called when configuring the device
 * for temperature monitoring or when disabling the sensor to conserve
 * power. Ensure that the device pointer is valid before calling this
 * function. The function does not perform any action if the device
 * pointer is null.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param enable A uint8_t value where 0 disables the temperature sensor and any
 * non-zero value enables it.
 * @return Returns an int32_t indicating success with API_CMS_ERROR_OK or an
 * error code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_device_temp_sensor_enable_set(adi_ad9083_device_t *device, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the temperature sensor mode on the
 * AD9083 device. It should be called when you need to change the mode of
 * the temperature sensor, which can be set to default, 1x diode, 20x
 * diode, or GND. Ensure that the device pointer is valid before calling
 * this function. The function will return an error code if the operation
 * fails.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param sel An unsigned 8-bit integer representing the temperature sensor
 * mode. Valid values are 0 (Default), 1 (1x diode), 2 (20x diode),
 * and 3 (GND).
 * @return Returns an int32_t indicating success (API_CMS_ERROR_OK) or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9083_device_temp_sensor_sel_set(adi_ad9083_device_t *device, uint8_t sel);

/***************************************************************************//**
 * @brief Use this function to calculate the temperature of the device based on
 * the voltage differences across two diodes. It requires the device to
 * be properly initialized and the temperature sensor to be enabled. The
 * function computes the temperature in milli-degrees Celsius and stores
 * it in the provided output parameter. Ensure that the input voltages
 * are in millivolts to get accurate results.
 *
 * @param device Pointer to the device structure. Must not be null. The device
 * should be initialized before calling this function.
 * @param vbe_1 Voltage across the first diode in millivolts. Must be a valid
 * integer representing the voltage in mV.
 * @param vbe_2 Voltage across the second diode in millivolts. Must be a valid
 * integer representing the voltage in mV.
 * @param temp Pointer to an integer where the calculated temperature in milli-
 * degrees Celsius will be stored. Must not be null.
 * @return Returns an integer status code. Returns API_CMS_ERROR_OK on success,
 * or a negative error code on failure.
 ******************************************************************************/
int32_t adi_ad9083_device_temp_get(adi_ad9083_device_t *device, int32_t vbe_1, int32_t vbe_2, int32_t *temp);








/*===== A 3 . 0   S P I  C O N T R O L S   =====*/
/***************************************************************************//**
 * @brief This function writes an 8-bit value to a specified SPI register
 * address within the PLL cbus of the device. It is used when there is a
 * need to configure or modify the PLL settings of the device through
 * direct register access. The function must be called with a valid
 * device pointer, and it will return an error if the device pointer is
 * null. This function is typically used in scenarios where low-level
 * control over the PLL configuration is required.
 *
 * @param device Pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param addr 8-bit address of the SPI register to which the data will be
 * written. Valid range is 0 to 255.
 * @param data 8-bit data value to be written to the specified SPI register
 * address. Valid range is 0 to 255.
 * @return Returns an int32_t indicating success or failure. A non-negative
 * value indicates success, while a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9083_device_cbuspll_register_set(adi_ad9083_device_t *device, uint8_t addr, uint8_t  data);

/***************************************************************************//**
 * @brief This function reads an 8-bit value from a specified address in the PLL
 * cbus register of the AD9083 device. It is essential to ensure that
 * both the device and data pointers are valid and not null before
 * calling this function. The function is typically used when there is a
 * need to access specific configuration or status information from the
 * PLL cbus registers. It returns an error code if the operation fails,
 * which can be used for error handling.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param addr An 8-bit unsigned integer representing the address of the PLL
 * cbus register to read from.
 * @param data A pointer to an 8-bit unsigned integer where the read value will
 * be stored. Must not be null.
 * @return Returns an int32_t indicating success or failure of the operation. A
 * non-negative value indicates success, while a negative value
 * indicates an error.
 ******************************************************************************/
int32_t adi_ad9083_device_cbuspll_register_get(adi_ad9083_device_t *device, uint8_t addr, uint8_t* data);
















/*===== x . x         =====*/
























#ifdef __cplusplus
}
#endif

#endif /* __ADI_AD9083_H__ */
/*! @} */
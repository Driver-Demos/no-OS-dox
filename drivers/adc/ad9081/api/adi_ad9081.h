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
 * @addtogroup ADI_AD9081
 * @{
 */
#ifndef __ADI_AD9081_H__
#define __ADI_AD9081_H__

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
****
*
* @defgroup tx_setup    2.0 Transmit Path Setup
* @brief    The transmit path setup API functions are used to set up the DAC datapaths
*           based on the desired mode of operation chosen for the transmit outputs.
*           Options for using an NCO test mode (without SERDES interface in use) as
*           well as normal JESD204B/C operation are enabled through this group of APIs.
*           These API functions are system high level APIs that can be called alone and
*           do not require any additional API calls directly to complete basic set up
*           the DAC transmit datapath functions and features.  All lower level calculations
*           and setup needed are covered by the mid-level transmit path individual block
*           helper APIs listed in Section 2.1 to Section 2.7.
*
* @defgroup tx_txen_setup    2.1 Transmit TxEn
* @ingroup  tx_setup
* @brief    These API functions are mid-level TxEn block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_dac_analog_core    2.2 Transmit DAC Analog Core
* @ingroup  tx_setup
* @brief    These API functions are mid-level DAC analog core block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_ch_gain_setup    2.3 Transmit Channelizer Gain
* @ingroup  tx_setup
* @brief    These API functions are mid-level channelizer gain block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_dp_setup    2.4 Transmit Datapath Setup
* @ingroup  tx_setup
* @brief    These API functions are mid-level Tx digital datapath block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_nco_setup    2.5 Transmit Path NCOs
* @ingroup  tx_setup
* @brief    These API functions are mid-level transmit channel and main NCO block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_nco_ffh_setup    2.5.1 Transmit Path FFH
* @ingroup  tx_nco_setup
* @brief    These API functions are mid-level fast frequency hopping (FFH) block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.


* @defgroup tx_pa_protect_setup    2.6 Transmit Path PA Protection
* @ingroup  tx_setup
* @brief    These API functions are mid-level Tx PA protection block API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup tx_helper_api    2.7 Transmit Path Helper APIs
* @ingroup  tx_setup
* @brief    These API functions are mid-level additional helper Tx path API calls underneath the top
*           system high level transmit path setup API function calls listed in Section 2.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.

****
*
* @defgroup rx_setup    3.0 Receive Path Setup
* @brief    The receive path setup API funcitons are used to set up the ADC datapaths
*           based on the desired mode of operation chosen for the receive inputs.
*           These API functions are system high level APIs that can be called alone and
*           do not require any additional API calls directly to complete basic set up
*           the ADC receive datapath functions and features.  All lower level calculations
*           and setup needed are covered by the mid-level receive path individual block
*           helper APIs listed in Section 3.1 through Section 3.7.
*
* @defgroup rx_dp_setup    3.1 Receive Datapath Setup
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx digital datapath block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_fd_setup    3.2 Receive Fast Detect
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx fast detect block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_pfilt_setup    3.3 Receive Programmable Filter
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx programmable filter block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_pfilt_low_level_setup    3.3.1 Low Level Receive Programmable Filter API
* @ingroup  rx_pfilt_setup
* @brief    These API functions are low-level Rx programmable filter block API calls underneath the
*           mid-level receive path setup API function calls listed in Section 3.3.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_nco_setup    3.4 Receive Path NCOs
* @ingroup  rx_setup
* @brief    These API functions are mid-level receive channel and main NCO block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_nco_delay_setup    3.4.1 Receive Path NCO Delay API
* @ingroup  rx_nco_setup
* @brief    These API functions are low-level Rx NCO delay block API calls underneath the
*           mid-level receive NCO setup API function calls listed in Section 3.4.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_decim_c2r_setup    3.5 Receive Path Decimation and C2R
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx datapath decimation & complex to real (C2R)
*           block API calls underneath the top system high level receive path setup API function
*           calls listed in Section 3.0. These API methods typically are not necessary to call
*           directly unless specific modification is needed by the user.
*
* @defgroup rx_fine_coarse_gain_setup    3.6 Receive Path Gain
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx channel and main gain block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_helper_api    3.7 Receive Path Helper APIs
* @ingroup  rx_setup
* @brief    These API functions are mid-level addition Rx path helper block API calls underneath the top
*           system high level receive path setup API function calls listed in Section 3.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup rx_power_savings 3.8 Receive Path Power Savings APIs
* @ingroup  rx_setup
* @brief    These API functions are mid-level Rx dynamic power savings APIs underneath the top system
*           high level receive path setup API function calls listed in Section 3.0. These API methods
*           are typically not necessary unless the user desires internal signal control to maximize power
*           saving options.
*
****
* @defgroup link_setup  4.0 SERDES Link Establishment & Monitoring
* @brief    The SERDES link setup API funcitons are used to set up the JESD204B/C links
*           based on the desired mode of operation chosen for both transmit and receive paths.
*           These API functions are system high level APIs that can be called alone and
*           do not require any additional API calls directly to complete basic set up
*           the SERDES link functions and features.  All lower level calculations
*           and setup needed are covered by the mid-level SERDES JRX and JTX individual block
*           helper APIs listed in Section 4.1 through Section 4.2.
*
* @defgroup dac_link_setup    4.1 SERDES Receiver Link Setup
* @ingroup  link_setup
* @brief    These API functions are mid-level JRX SERDES block API calls underneath the top
*           system high level JRX SERDES API function calls listed in Section 4.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
* @defgroup adc_link_setup    4.2 SERDES Transmitter Link Setup
* @ingroup  link_setup
* @brief    These API functions are mid-level JTX SERDES block API calls underneath the top
*           system high level JTX SERDES API function calls listed in Section 4.0.
*           These API methods typically are not necessary to call directly unless specific
*           modification is needed by the user.
*
****
*
* @defgroup appendix    Appendix: Additional Digital Feature Blocks
* @brief    This Appendix section includes additional API function calls for other features
*           and functional blocks that exist in the MxFE that are not covered in previous sections.
*
* @defgroup appdx_serdes_tm    A1.0 SERDES Link Test Modes
* @ingroup  appendix
* @brief    These API functions are used to set up various SERDES link test modes available.
*
* @defgroup appdx_serdes_jrx_tm    A1.1 JRX SERDES Link Test Modes
* @ingroup  appendix
* @brief    The JRX SERDES link test mode API functions are used to set up the SERDES
*           test modes for debugging the JESD204B/C link between FPGA (transmitter)
*           and DAC datapath (receiver).
*
* @defgroup appdx_serdes_jtx_tm    A1.2 JTX SERDES Link Test Modes
* @ingroup  appendix
* @brief    The JTX SERDES link test mode API functions are used to set up the SERDES
*           test modes for debugging the JESD204B/C link between ADC (transmitter)
*           datapath and FPGA (receiver)
*
* @defgroup appdx_mcs    A2.0 Multi-chip Sync & Subclass 1
* @ingroup  appendix
* @brief    These API functions are used to set up various SERDES multi-chip sync and
*           Subclass 1 functions available.
*
* @defgroup appdx_irqs    A3.0 Interrupt Service Requests
* @ingroup  appendix
* @brief    These API functions are used to set up various IRQs available.
*
* @defgroup appdx_irq_dac_dp    A3.1 DAC Datapath IRQs
* @ingroup  appendix
* @brief    These API functions are used to set up transmit path DAC datapath IRQs available.
*
* @defgroup appdx_irq_adc_dp    A3.2 ADC Datapath IRQs
* @ingroup  appendix
* @brief    These API functions are used to set up receive path ADC datapath IRQs available.
*
* @defgroup appdx_irq_serdes_dp    A3.3 SERDES Configuration IRQs
* @ingroup  appendix
* @brief    These API functions are used to set up SERDES IRQs available.
*
* @defgroup appdx_loopback    A4.0  Loopback Test Modes
* @ingroup  appendix
* @brief    These API functions are used to set up various loopback test modes available.
*
* @defgroup appdx_rx_adc_smon    A5.0  ADC Signal Monitoring
* @ingroup  appendix
* @brief    These API functions are used to set up ADC signal monitoring fuctions available.
*
* @defgroup appdx_spi       A6.0  SPI Controls
* @ingroup  appendix
* @brief    These API methods set up and configure lower-level SPI calls not directly needed to
*           configure the setup of the MxFE.
*/

/*============= I N C L U D E S ============*/
#include "adi_cms_api_common.h"

/*============= D E F I N E S ==============*/
#define AD9081_ID 0x9081

#define AD9082_ID 0x9082
#define AD9082_ADC_CLK_FREQ_HZ_MAX 6300000000ULL
#define AD9081_DAC_CLK_FREQ_HZ_MIN 2900000000ULL
#define AD9081_DAC_CLK_FREQ_HZ_MAX 12000000000ULL
#define AD9081_ADC_CLK_FREQ_HZ_MIN 1450000000ULL
#define AD9081_ADC_CLK_FREQ_HZ_MAX 4000000000ULL
#define AD9081_REF_CLK_FREQ_HZ_MIN 25000000ULL
#define AD9081_REF_CLK_FREQ_HZ_MAX 3000000000ULL
#define AD9081_JESDRX_204C_CAL_THRESH 16000000000ULL
#define AD9081_JESD_SER_COUNT 8
#define AD9081_JESD_DESER_COUNT 8

#define AD9081_USE_FLOATING_TYPE 0
#define AD9081_USE_SPI_BURST_MODE 0

/*============= ENUMS ==============*/

/***************************************************************************//**
 * @brief The `adi_ad9081_dac_select_e` is an enumeration that defines constants
 * for selecting different Digital-to-Analog Converters (DACs) in the
 * AD9081 device, allowing for easy reference to specific DACs or all
 * DACs in the system.
 *
 * @param AD9081_DAC_NONE Represents the absence of any DAC.
 * @param AD9081_DAC_0 Represents the first DAC (DAC0).
 * @param AD9081_DAC_1 Represents the second DAC (DAC1).
 * @param AD9081_DAC_2 Represents the third DAC (DAC2).
 * @param AD9081_DAC_3 Represents the fourth DAC (DAC3).
 * @param AD9081_DAC_ALL Represents all available DACs.
 ******************************************************************************/
typedef enum {
	AD9081_DAC_NONE = 0x0, /*!< No DAC */
	AD9081_DAC_0 = 0x1, /*!< DAC0 */
	AD9081_DAC_1 = 0x2, /*!< DAC1 */
	AD9081_DAC_2 = 0x4, /*!< DAC2 */
	AD9081_DAC_3 = 0x8, /*!< DAC3 */
	AD9081_DAC_ALL = 0x0F /*!< ALL DACs */
} adi_ad9081_dac_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_dac_channel_select_e` is an enumeration that defines
 * constants for selecting DAC channels in the AD9081 device, allowing
 * for easy reference to specific channels or the option to select all
 * channels.
 *
 * @param AD9081_DAC_CH_NONE Represents no channel selected.
 * @param AD9081_DAC_CH_0 Represents channel 0.
 * @param AD9081_DAC_CH_1 Represents channel 1.
 * @param AD9081_DAC_CH_2 Represents channel 2.
 * @param AD9081_DAC_CH_3 Represents channel 3.
 * @param AD9081_DAC_CH_4 Represents channel 4.
 * @param AD9081_DAC_CH_5 Represents channel 5.
 * @param AD9081_DAC_CH_6 Represents channel 6.
 * @param AD9081_DAC_CH_7 Represents channel 7.
 * @param AD9081_DAC_CH_ALL Represents all channels selected.
 ******************************************************************************/
typedef enum {
	AD9081_DAC_CH_NONE = 0x00, /*!< No Channel */
	AD9081_DAC_CH_0 = 0x01, /*!< Channel 0 */
	AD9081_DAC_CH_1 = 0x02, /*!< Channel 1 */
	AD9081_DAC_CH_2 = 0x04, /*!< Channel 2 */
	AD9081_DAC_CH_3 = 0x08, /*!< Channel 3 */
	AD9081_DAC_CH_4 = 0x10, /*!< Channel 4 */
	AD9081_DAC_CH_5 = 0x20, /*!< Channel 5 */
	AD9081_DAC_CH_6 = 0x40, /*!< Channel 6 */
	AD9081_DAC_CH_7 = 0x80, /*!< Channel 7 */
	AD9081_DAC_CH_ALL = 0xFF /*!< ALL Channels */
} adi_ad9081_dac_channel_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_dac_dp_select_e` is an enumeration that defines
 * various options for selecting the digital-to-analog converter (DAC)
 * datapaths in the AD9081 device, allowing for the specification of
 * different configurations for the DAC operation.
 *
 * @param AD9081_DAC_DP_NONE Represents no coarse DUC and no main DAC datapath.
 * @param AD9081_DAC_DP_0 Represents coarse DUC 0 and main DAC0 datapath.
 * @param AD9081_DAC_DP_1 Represents coarse DUC 1 and main DAC1 datapath.
 * @param AD9081_DAC_DP_2 Represents coarse DUC 2 and main DAC2 datapath.
 * @param AD9081_DAC_DP_3 Represents coarse DUC 3 and main DAC3 datapath.
 * @param AD9081_DAC_DP_ALL Represents all coarse DUCs and all main DAC
 * datapaths.
 ******************************************************************************/
typedef enum {
	AD9081_DAC_DP_NONE = 0x0, /*!< No Coarse DUC /No Main DAC Datapath */
	AD9081_DAC_DP_0 = 0x1, /*!< Coarse DUC 0/ Main DAC0 Datapath */
	AD9081_DAC_DP_1 = 0x2, /*!< Coarse DUC 1/ Main DAC1 Datapath */
	AD9081_DAC_DP_2 = 0x4, /*!< Coarse DUC 2/ Main DAC2 Datapath */
	AD9081_DAC_DP_3 = 0x8, /*!< Coarse DUC 3/ Main DAC3 Datapath */
	AD9081_DAC_DP_ALL = 0x0F /*!< ALL Coarse DUC /ALL Main DAC Datapths */
} adi_ad9081_dac_dp_select_e;

/***************************************************************************//**
 * @brief `adi_ad9081_dac_pair_select_e` is an enumeration that defines
 * constants for selecting different pairs of Digital-to-Analog
 * Converters (DACs) in the AD9081 device, allowing for easy reference to
 * specific DAC groupings in the code.
 *
 * @param AD9081_DAC_PAIR_NONE Represents no DAC group selected.
 * @param AD9081_DAC_PAIR_0_1 Represents the selection of DAC0 and DAC1.
 * @param AD9081_DAC_PAIR_2_3 Represents the selection of DAC2 and DAC3.
 * @param AD9081_DAC_PAIR_ALL Represents the selection of all DAC groups.
 ******************************************************************************/
typedef enum {
	AD9081_DAC_PAIR_NONE = 0x00, /*!< No Group */
	AD9081_DAC_PAIR_0_1 = 0x01, /*!< Group 0 (DAC0 & DAC1) */
	AD9081_DAC_PAIR_2_3 = 0x02, /*!< Group 1 (DAC2 & DAC3) */
	AD9081_DAC_PAIR_ALL = 0x03, /*!< All Groups */
} adi_ad9081_dac_pair_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_dac_mod_mux_mode_e` is an enumeration that defines
 * various modes for multiplexing digital-to-analog converters (DACs) in
 * the AD9081 device, specifying how input signals are routed to the DAC
 * outputs.
 *
 * @param AD9081_DAC_MUX_MODE_0 Represents the first DAC multiplexing mode.
 * @param AD9081_DAC_MUX_MODE_1 Represents the second DAC multiplexing mode.
 * @param AD9081_DAC_MUX_MODE_2 Represents the third DAC multiplexing mode.
 * @param AD9081_DAC_MUX_MODE_3 Represents the fourth DAC multiplexing mode.
 ******************************************************************************/
typedef enum {
	AD9081_DAC_MUX_MODE_0 =
		0x00, /*!<  I0.Q0 -> DAC0, Complex I1.Q1 -> DAC1 */
	AD9081_DAC_MUX_MODE_1 =
		0x01, /*!< (I0 + I1) / 2 -> DAC0, (Q0 + Q1) / 2 -> DAC1, Data Path NCOs Bypassed */
	AD9081_DAC_MUX_MODE_2 =
		0x02, /*!< I0 -> DAC0, Q0 -> DAC1, Datapath0 NCO Bypassed, Datapath1 Unused */
	AD9081_DAC_MUX_MODE_3 =
		0x03, /*!< (I0 + I1) / 2 -> DAC0, DAC1 Output Tied To Midscale */
} adi_ad9081_dac_mod_mux_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_select_e` is an enumeration that defines constants
 * for selecting different ADCs in the AD9081 device, allowing for easy
 * reference to specific ADCs or all ADCs in the system.
 *
 * @param AD9081_ADC_NONE Represents the absence of any ADC.
 * @param AD9081_ADC_0 Represents the first ADC (ADC0).
 * @param AD9081_ADC_1 Represents the second ADC (ADC1).
 * @param AD9081_ADC_2 Represents the third ADC (ADC2).
 * @param AD9081_ADC_3 Represents the fourth ADC (ADC3).
 * @param AD9081_ADC_ALL Represents all available ADCs.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_NONE = 0x0, /*!< No ADC */
	AD9081_ADC_0 = 0x1, /*!< ADC0 */
	AD9081_ADC_1 = 0x2, /*!< ADC1 */
	AD9081_ADC_2 = 0x4, /*!< ADC2 */
	AD9081_ADC_3 = 0x8, /*!< ADC3 */
	AD9081_ADC_ALL = 0x0F /*!< ALL ADCs */
} adi_ad9081_adc_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_nco_mode_e` is an enumeration that defines various
 * modes for the NCO (Numerically Controlled Oscillator) in the AD9081
 * ADC, allowing for clear representation of different operational states
 * such as Variable IF Mode, Zero IF Mode, and others, including an
 * invalid state.
 *
 * @param AD9081_ADC_NCO_VIF Represents Variable IF Mode.
 * @param AD9081_ADC_NCO_ZIF Represents Zero IF Mode.
 * @param AD9081_ADC_NCO_FS_4_IF Represents Fs/4 Hz IF Mode.
 * @param AD9081_ADC_NCO_TEST Represents Test Mode.
 * @param AD9081_ADC_NCO_INVALID Represents an Invalid NCO Mode.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_NCO_VIF = 0, /*!< Variable IF Mode */
	AD9081_ADC_NCO_ZIF = 1, /*!< Zero IF Mode */
	AD9081_ADC_NCO_FS_4_IF = 2, /*!< Fs/4 Hz IF Mode */
	AD9081_ADC_NCO_TEST = 3, /*!< Test Mode */
	AD9081_ADC_NCO_INVALID = 4 /*!< Invalid NCO Mode */
} adi_ad9081_adc_nco_mode_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_adc_to_cddc_xbar_e` is an enumeration that defines
 * different operational modes for ADCs, specifically distinguishing
 * between real and complex modes for both quad and dual ADC
 * configurations.
 *
 * @param AD9081_ADC_4_ADC_REAL_MODE Represents the Quad ADC Real Mode.
 * @param AD9081_ADC_4_ADC_COMP_MODE Represents the Quad ADC Complex Mode.
 * @param AD9081_ADC_2_ADC_REAL_MODE Represents the Dual ADC Real Mode.
 * @param AD9081_ADC_2_ADC_COMP_MODE Represents the Dual ADC Complex Mode.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_4_ADC_REAL_MODE = 0, /*!< Quad ADC Real Mode */
	AD9081_ADC_4_ADC_COMP_MODE = 1, /*!< Quad ADC Complex Mode */
	AD9081_ADC_2_ADC_REAL_MODE = 2, /*!< Dual ADC Real Mode */
	AD9081_ADC_2_ADC_COMP_MODE = 3 /*!< Dual ADC Complex MOde */
} adi_ad9081_adc_adc_to_cddc_xbar_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_cddc_to_fddc0_xbar_e` is an enumeration that
 * defines two constants representing the crossbar connections from two
 * ADC channels (CDDC0 and CDDC1) to a common FDDC0 output, facilitating
 * the selection of the appropriate ADC input for further processing.
 *
 * @param AD9081_ADC_CDDC0_TO_FDDC0 Represents the mapping from CDDC0 to FDDC0
 * for ADC channel 0.
 * @param AD9081_ADC_CDDC1_TO_FDDC0 Represents the mapping from CDDC1 to FDDC0
 * for ADC channel 1.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC0_TO_FDDC0 = 0,
	AD9081_ADC_CDDC1_TO_FDDC0 = 1
} adi_ad9081_adc_cddc_to_fddc0_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc1_xbar_e` is an enumeration that defines
 * two constants representing the crossbar connections from the CDDC
 * (Channel Data Digital Converter) to the FDDC (Frequency Domain Digital
 * Converter) for the AD9081 ADC, allowing for specific routing
 * configurations.
 *
 * @param AD9081_ADC_CDDC0_TO_FDDC1 Represents the mapping from CDDC0 to FDDC1.
 * @param AD9081_ADC_CDDC1_TO_FDDC1 Represents the mapping from CDDC1 to FDDC1.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC0_TO_FDDC1 = 0,
	AD9081_ADC_CDDC1_TO_FDDC1 = 2
} adi_ad9081_adc_cddc_to_fddc1_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc2_xbar_e` is an enumeration that defines
 * two constants representing the crossbar connections from two different
 * ADC channels (CDDC0 and CDDC1) to a common output (FDDC2),
 * facilitating the selection of data routing in a digital signal
 * processing context.
 *
 * @param AD9081_ADC_CDDC0_TO_FDDC2 Represents the mapping from CDDC0 to FDDC2.
 * @param AD9081_ADC_CDDC1_TO_FDDC2 Represents the mapping from CDDC1 to FDDC2.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC0_TO_FDDC2 = 0,
	AD9081_ADC_CDDC1_TO_FDDC2 = 4
} adi_ad9081_adc_cddc_to_fddc2_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc3_xbar_e` is an enumeration that defines
 * two constants representing the crossbar connections from two different
 * CDDC (Channel Digital Down Converter) inputs to a single FDDC
 * (Frequency Domain Digital Converter) output, facilitating the
 * selection of input channels for signal processing.
 *
 * @param AD9081_ADC_CDDC0_TO_FDDC3 Represents the mapping from CDDC0 to FDDC3.
 * @param AD9081_ADC_CDDC1_TO_FDDC3 Represents the mapping from CDDC1 to FDDC3.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC0_TO_FDDC3 = 0,
	AD9081_ADC_CDDC1_TO_FDDC3 = 8
} adi_ad9081_adc_cddc_to_fddc3_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc4_xbar_e` is an enumeration that defines
 * two constants representing the crossbar connections from ADC CDDC2 and
 * CDDC3 to FDDC4, facilitating the configuration of signal routing in
 * the AD9081 ADC.
 *
 * @param AD9081_ADC_CDDC2_TO_FDDC4 Represents the mapping from CDDC2 to FDDC4.
 * @param AD9081_ADC_CDDC3_TO_FDDC4 Represents the mapping from CDDC3 to FDDC4.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC2_TO_FDDC4 = 0,
	AD9081_ADC_CDDC3_TO_FDDC4 = 0x10
} adi_ad9081_adc_cddc_to_fddc4_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc5_xbar_e` is an enumeration that defines
 * two constants representing different mappings from CDDC (Channel Data
 * Digital Converter) to FDDC (Frequency Domain Digital Converter) for
 * the AD9081 ADC, allowing for easy reference to these specific
 * configurations in the code.
 *
 * @param AD9081_ADC_CDDC2_TO_FDDC5 Represents the mapping from CDDC2 to FDDC5.
 * @param AD9081_ADC_CDDC3_TO_FDDC5 Represents the mapping from CDDC3 to FDDC5.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC2_TO_FDDC5 = 0,
	AD9081_ADC_CDDC3_TO_FDDC5 = 0x20
} adi_ad9081_adc_cddc_to_fddc5_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc6_xbar_e` is an enumeration that defines
 * two constants representing the crossbar connections from ADC CDDC
 * channels to FDDC6, allowing for easy reference to these specific
 * mappings in the code.
 *
 * @param AD9081_ADC_CDDC2_TO_FDDC6 Represents the mapping from CDDC2 to FDDC6.
 * @param AD9081_ADC_CDDC3_TO_FDDC6 Represents the mapping from CDDC3 to FDDC6.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC2_TO_FDDC6 = 0,
	AD9081_ADC_CDDC3_TO_FDDC6 = 0x40
} adi_ad9081_adc_cddc_to_fddc6_xbar_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_cddc_to_fddc7_xbar_e` is an enumeration that defines
 * two constants representing different mappings from ADC CDDC channels
 * to the FDDC7 output, facilitating the configuration of signal routing
 * in the AD9081 ADC.
 *
 * @param AD9081_ADC_CDDC2_TO_FDDC7 Represents the mapping from CDDC2 to FDDC7.
 * @param AD9081_ADC_CDDC3_TO_FDDC7 Represents the mapping from CDDC3 to FDDC7.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC2_TO_FDDC7 = 0,
	AD9081_ADC_CDDC3_TO_FDDC7 = 0x80
} adi_ad9081_adc_cddc_to_fddc7_xbar_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_coarse_ddc_select_e` is an enumeration that
 * defines constants for selecting different coarse digital down
 * converters (DDCs) in the AD9081 ADC, allowing for easy reference to
 * specific DDC configurations in the code.
 *
 * @param AD9081_ADC_CDDC_NONE Represents no coarse DDC selection.
 * @param AD9081_ADC_CDDC_0 Represents selection of coarse DDC 0.
 * @param AD9081_ADC_CDDC_1 Represents selection of coarse DDC 1.
 * @param AD9081_ADC_CDDC_2 Represents selection of coarse DDC 2.
 * @param AD9081_ADC_CDDC_3 Represents selection of coarse DDC 3.
 * @param AD9081_ADC_CDDC_ALL Represents selection of all coarse DDCs.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_CDDC_NONE = 0x00, /*!< No COARSE DDC */
	AD9081_ADC_CDDC_0 = 0x01, /*!< COARSE DDC 0 */
	AD9081_ADC_CDDC_1 = 0x02, /*!< COARSE DDC 1 */
	AD9081_ADC_CDDC_2 = 0x04, /*!< COARSE DDC 2 */
	AD9081_ADC_CDDC_3 = 0x08, /*!< COARSE DDC 3 */
	AD9081_ADC_CDDC_ALL = 0x0F /*!< ALL COARSE DDCs */
} adi_ad9081_adc_coarse_ddc_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_fine_ddc_select_e` is an enumeration that defines
 * constants for selecting fine digital down converters (DDCs) in the
 * AD9081 ADC, allowing for the specification of individual DDCs or all
 * DDCs.
 *
 * @param AD9081_ADC_FDDC_NONE Represents no fine digital down converter (DDC).
 * @param AD9081_ADC_FDDC_0 Represents fine DDC 0.
 * @param AD9081_ADC_FDDC_1 Represents fine DDC 1.
 * @param AD9081_ADC_FDDC_2 Represents fine DDC 2.
 * @param AD9081_ADC_FDDC_3 Represents fine DDC 3.
 * @param AD9081_ADC_FDDC_4 Represents fine DDC 4.
 * @param AD9081_ADC_FDDC_5 Represents fine DDC 5.
 * @param AD9081_ADC_FDDC_6 Represents fine DDC 6.
 * @param AD9081_ADC_FDDC_7 Represents fine DDC 7.
 * @param AD9081_ADC_FDDC_ALL Represents all fine DDCs.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_FDDC_NONE = 0x00, /*!< No FINE DDC */
	AD9081_ADC_FDDC_0 = 0x01, /*!< FINE DDC 0 */
	AD9081_ADC_FDDC_1 = 0x02, /*!< FINE DDC 1 */
	AD9081_ADC_FDDC_2 = 0x04, /*!< FINE DDC 2 */
	AD9081_ADC_FDDC_3 = 0x08, /*!< FINE DDC 3 */
	AD9081_ADC_FDDC_4 = 0x10, /*!< FINE DDC 4 */
	AD9081_ADC_FDDC_5 = 0x20, /*!< FINE DDC 5 */
	AD9081_ADC_FDDC_6 = 0x40, /*!< FINE DDC 6 */
	AD9081_ADC_FDDC_7 = 0x80, /*!< FINE DDC 7 */
	AD9081_ADC_FDDC_ALL = 0xFF /*!< ALL FINE DDCs */
} adi_ad9081_adc_fine_ddc_select_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_coarse_ddc_dcm_e` is an enumeration that defines
 * various decimation factors for the AD9081 ADC's coarse digital
 * downconverter (DDC), allowing for flexible configuration of the
 * decimation process.
 *
 * @param AD9081_CDDC_DCM_1 Represents a decimation factor of 1.
 * @param AD9081_CDDC_DCM_2 Represents a decimation factor of 2.
 * @param AD9081_CDDC_DCM_3 Represents a decimation factor of 3.
 * @param AD9081_CDDC_DCM_4 Represents a decimation factor of 4.
 * @param AD9081_CDDC_DCM_6 Represents a decimation factor of 6.
 * @param AD9081_CDDC_DCM_8 Represents a decimation factor of 8.
 * @param AD9081_CDDC_DCM_9 Represents a decimation factor of 9.
 * @param AD9081_CDDC_DCM_12 Represents a decimation factor of 12.
 * @param AD9081_CDDC_DCM_16 Represents a decimation factor of 16.
 * @param AD9081_CDDC_DCM_18 Represents a decimation factor of 18.
 * @param AD9081_CDDC_DCM_24 Represents a decimation factor of 24.
 * @param AD9081_CDDC_DCM_36 Represents a decimation factor of 36.
 ******************************************************************************/
typedef enum {
	AD9081_CDDC_DCM_1 = 0xC, /*!< Decimate by 1 */
	AD9081_CDDC_DCM_2 = 0x0, /*!< Decimate by 2 */
	AD9081_CDDC_DCM_3 = 0x8, /*!< Decimate by 3 */
	AD9081_CDDC_DCM_4 = 0x1, /*!< Decimate by 4 */
	AD9081_CDDC_DCM_6 = 0x5, /*!< Decimate by 6 */
	AD9081_CDDC_DCM_8 = 0x2, /*!< Decimate by 8 */
	AD9081_CDDC_DCM_9 = 0x9, /*!< Decimate by 9 */
	AD9081_CDDC_DCM_12 = 0x6, /*!< Decimate by 12 */
	AD9081_CDDC_DCM_16 = 0x3, /*!< Decimate by 16 */
	AD9081_CDDC_DCM_18 = 0xA, /*!< Decimate by 18 */
	AD9081_CDDC_DCM_24 = 0x7, /*!< Decimate by 24 */
	AD9081_CDDC_DCM_36 = 0xB /*!< Decimate by 36 */
} adi_ad9081_adc_coarse_ddc_dcm_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_fine_ddc_dcm_e` is an enumeration that defines
 * various decimation factors for the AD9081 ADC fine digital
 * downconverter, allowing for selection of specific decimation rates
 * ranging from 1 to 24.
 *
 * @param AD9081_FDDC_DCM_1 Represents a decimation factor of 1.
 * @param AD9081_FDDC_DCM_2 Represents a decimation factor of 2.
 * @param AD9081_FDDC_DCM_3 Represents a decimation factor of 3.
 * @param AD9081_FDDC_DCM_4 Represents a decimation factor of 4.
 * @param AD9081_FDDC_DCM_6 Represents a decimation factor of 6.
 * @param AD9081_FDDC_DCM_8 Represents a decimation factor of 8.
 * @param AD9081_FDDC_DCM_12 Represents a decimation factor of 12.
 * @param AD9081_FDDC_DCM_16 Represents a decimation factor of 16.
 * @param AD9081_FDDC_DCM_24 Represents a decimation factor of 24.
 ******************************************************************************/
typedef enum {
	AD9081_FDDC_DCM_1 = 0x8, /*!< Decimate by 1 */
	AD9081_FDDC_DCM_2 = 0x0, /*!< Decimate by 2 */
	AD9081_FDDC_DCM_3 = 0x4, /*!< Decimate by 3 */
	AD9081_FDDC_DCM_4 = 0x1, /*!< Decimate by 4 */
	AD9081_FDDC_DCM_6 = 0x5, /*!< Decimate by 6 */
	AD9081_FDDC_DCM_8 = 0x2, /*!< Decimate by 8 */
	AD9081_FDDC_DCM_12 = 0x6, /*!< Decimate by 12 */
	AD9081_FDDC_DCM_16 = 0x3, /*!< Decimate by 16 */
	AD9081_FDDC_DCM_24 = 0x7, /*!< Decimate by 24 */
} adi_ad9081_adc_fine_ddc_dcm_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_fine_ddc_converter_e` is an enumeration that
 * defines constants for the I and Q components of multiple Fine Digital
 * Down Converters (FDDCs) in the AD9081 ADC, allowing for easy reference
 * to these components in code.
 *
 * @param AD9081_FDDC_0_I Represents the I component of FDDC0.
 * @param AD9081_FDDC_0_Q Represents the Q component of FDDC0.
 * @param AD9081_FDDC_1_I Represents the I component of FDDC1.
 * @param AD9081_FDDC_1_Q Represents the Q component of FDDC1.
 * @param AD9081_FDDC_2_I Represents the I component of FDDC2.
 * @param AD9081_FDDC_2_Q Represents the Q component of FDDC2.
 * @param AD9081_FDDC_3_I Represents the I component of FDDC3.
 * @param AD9081_FDDC_3_Q Represents the Q component of FDDC3.
 * @param AD9081_FDDC_4_I Represents the I component of FDDC4.
 * @param AD9081_FDDC_4_Q Represents the Q component of FDDC4.
 * @param AD9081_FDDC_5_I Represents the I component of FDDC5.
 * @param AD9081_FDDC_5_Q Represents the Q component of FDDC5.
 * @param AD9081_FDDC_6_I Represents the I component of FDDC6.
 * @param AD9081_FDDC_6_Q Represents the Q component of FDDC6.
 * @param AD9081_FDDC_7_I Represents the I component of FDDC7.
 * @param AD9081_FDDC_7_Q Represents the Q component of FDDC7.
 ******************************************************************************/
typedef enum {
	AD9081_FDDC_0_I = 0x0, /*!< FDDC0 I */
	AD9081_FDDC_0_Q = 0x1, /*!< FDDC0 Q */
	AD9081_FDDC_1_I = 0x2, /*!< FDDC1 I */
	AD9081_FDDC_1_Q = 0x3, /*!< FDDC1 Q */
	AD9081_FDDC_2_I = 0x4, /*!< FDDC2 I */
	AD9081_FDDC_2_Q = 0x5, /*!< FDDC2 Q */
	AD9081_FDDC_3_I = 0x6, /*!< FDDC3 I */
	AD9081_FDDC_3_Q = 0x7, /*!< FDDC3 Q */
	AD9081_FDDC_4_I = 0x8, /*!< FDDC4 I */
	AD9081_FDDC_4_Q = 0x9, /*!< FDDC4 Q */
	AD9081_FDDC_5_I = 0xA, /*!< FDDC5 I */
	AD9081_FDDC_5_Q = 0xB, /*!< FDDC5 Q */
	AD9081_FDDC_6_I = 0xC, /*!< FDDC6 I */
	AD9081_FDDC_6_Q = 0xD, /*!< FDDC6 Q */
	AD9081_FDDC_7_I = 0xE, /*!< FDDC7 I */
	AD9081_FDDC_7_Q = 0xF, /*!< FDDC7 Q */
} adi_ad9081_adc_fine_ddc_converter_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_nyquist_zone_e` is an enumeration that defines two
 * constants representing the Nyquist zones for the AD9081 ADC,
 * specifically distinguishing between odd and even zones.
 *
 * @param AD9081_ADC_NYQUIST_ZONE_ODD Represents the odd Nyquist zone.
 * @param AD9081_ADC_NYQUIST_ZONE_EVEN Represents the even Nyquist zone.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_NYQUIST_ZONE_ODD = 0x00, /*!< Odd  Zone */
	AD9081_ADC_NYQUIST_ZONE_EVEN = 0x01 /*!< Even Zone */
} adi_ad9081_adc_nyquist_zone_e;

/***************************************************************************//**
 * @brief `adi_ad9081_adc_pfir_ctl_page_e` is an enumeration that defines
 * constants for different PFIR ADC pairs, allowing for easy reference to
 * specific pairs in the code.
 *
 * @param AD9081_ADC_PFIR_ADC_PAIR0 Represents the PFIR ADC Pair0.
 * @param AD9081_ADC_PFIR_ADC_PAIR1 Represents the PFIR ADC Pair1.
 * @param AD9081_ADC_PFIR_ADC_PAIR_ALL Represents all PFIR ADC pairs.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_PFIR_ADC_PAIR0 = 0x01, /*!< PFIR ADC Pair0 */
	AD9081_ADC_PFIR_ADC_PAIR1 = 0x02, /*!< PFIR ADC Pair1 */
	AD9081_ADC_PFIR_ADC_PAIR_ALL = 0x03 /*!< PFIR ADC Pair All */
} adi_ad9081_adc_pfir_ctl_page_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_pfir_coeff_page_e` is an enumeration that defines
 * constants for different PFIR coefficient pages used in the AD9081 ADC,
 * allowing for easy reference to specific pages in the configuration.
 *
 * @param AD9081_ADC_PFIR_COEFF_PAGE0 Represents PFIR Coefficient Page0.
 * @param AD9081_ADC_PFIR_COEFF_PAGE1 Represents PFIR Coefficient Page1.
 * @param AD9081_ADC_PFIR_COEFF_PAGE2 Represents PFIR Coefficient Page2.
 * @param AD9081_ADC_PFIR_COEFF_PAGE3 Represents PFIR Coefficient Page3.
 * @param AD9081_ADC_PFIR_COEFF_PAGE_ALL Represents all PFIR Coefficient Pages.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_PFIR_COEFF_PAGE0 = 0x01, /*!< PFIR Coefficient Page0 */
	AD9081_ADC_PFIR_COEFF_PAGE1 = 0x02, /*!< PFIR Coefficient Page1 */
	AD9081_ADC_PFIR_COEFF_PAGE2 = 0x04, /*!< PFIR Coefficient Page2 */
	AD9081_ADC_PFIR_COEFF_PAGE3 = 0x08, /*!< PFIR Coefficient Page3 */
	AD9081_ADC_PFIR_COEFF_PAGE_ALL = 0x0F /*!< PFIR Coefficient Page All */
} adi_ad9081_adc_pfir_coeff_page_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_pfir_i_mode_e` is an enumeration that defines
 * various modes for the ADC PFIR (Polyphase Finite Impulse Response)
 * filter, allowing for different configurations such as disabled, real,
 * and complex tap filters.
 *
 * @param AD9081_ADC_PFIR_I_MODE_DISABLE Represents the disabled state with
 * filters bypassed.
 * @param AD9081_ADC_PFIR_I_MODE_REAL_N4 Represents a real N/4 tap filter mode.
 * @param AD9081_ADC_PFIR_I_MODE_REAL_N2 Represents a real N/2 tap filter mode.
 * @param AD9081_ADC_PFIR_I_MODE_MATRIX Represents an N/4 tap matrix mode.
 * @param AD9081_ADC_PFIR_I_MODE_COMPLEX_FULL Represents a full complex N/3 tap
 * filter mode.
 * @param AD9081_ADC_PFIR_I_MODE_COMPLEX_HALF Represents a half complex mode
 * using two N/2 tap filters.
 * @param AD9081_ADC_PFIR_I_MODE_REAL_N Represents a real N tap filter mode.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_PFIR_I_MODE_DISABLE =
		0x0, /*!< Disabled (filters bypassed) */
	AD9081_ADC_PFIR_I_MODE_REAL_N4 = 0x1, /*!< Real N/4 tap filter */
	AD9081_ADC_PFIR_I_MODE_REAL_N2 = 0x2, /*!< Real N/2 tap filter */
	AD9081_ADC_PFIR_I_MODE_MATRIX = 0x4, /*!< N/4 tap matrix mode */
	AD9081_ADC_PFIR_I_MODE_COMPLEX_FULL = 0x5, /*!< N/3 tap full complex */
	AD9081_ADC_PFIR_I_MODE_COMPLEX_HALF =
		0x6, /*!< Half complex using 2 N/2 tap filters */
	AD9081_ADC_PFIR_I_MODE_REAL_N = 0x7 /*!< Real N tap filter */
} adi_ad9081_adc_pfir_i_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_pfir_q_mode_e` is an enumeration that defines
 * various modes for the ADC PFIR Q filter configuration, allowing
 * selection between different filter types such as disabled, real,
 * complex, and matrix modes, each represented by a unique constant
 * value.
 *
 * @param AD9081_ADC_PFIR_Q_MODE_DISABLE Represents the disabled state with
 * filters bypassed.
 * @param AD9081_ADC_PFIR_Q_MODE_REAL_N4 Indicates a real N/4 tap filter mode.
 * @param AD9081_ADC_PFIR_Q_MODE_REAL_N2 Indicates a real N/2 tap filter mode.
 * @param AD9081_ADC_PFIR_Q_MODE_MATRIX Represents the N/4 tap matrix mode.
 * @param AD9081_ADC_PFIR_Q_MODE_COMPLEX_FULL Indicates a full complex N/3 tap
 * filter mode.
 * @param AD9081_ADC_PFIR_Q_MODE_COMPLEX_HALF Represents a half complex mode
 * using two N/2 tap filters.
 * @param AD9081_ADC_PFIR_Q_MODE_REAL_N Indicates a real N tap filter mode.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_PFIR_Q_MODE_DISABLE =
		0x0, /*!< Disabled (filters bypassed) */
	AD9081_ADC_PFIR_Q_MODE_REAL_N4 = 0x1, /*!< Real N/4 tap filter */
	AD9081_ADC_PFIR_Q_MODE_REAL_N2 = 0x2, /*!< Real N/2 tap filter */
	AD9081_ADC_PFIR_Q_MODE_MATRIX = 0x4, /*!< N/4 tap matrix mode */
	AD9081_ADC_PFIR_Q_MODE_COMPLEX_FULL = 0x5, /*!< N/3 tap full complex */
	AD9081_ADC_PFIR_Q_MODE_COMPLEX_HALF =
		0x6, /*!< Half complex using 2 N/2 tap filters */
	AD9081_ADC_PFIR_Q_MODE_REAL_N = 0x7 /*!< Real N tap filter */
} adi_ad9081_adc_pfir_q_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_pfir_gain_e` is an enumeration that defines
 * various gain levels for the AD9081 ADC PFIR, allowing for easy
 * reference to specific gain values such as -12dB, -6dB, 0dB, 6dB, and
 * 12dB.
 *
 * @param AD9081_ADC_PFIR_GAIN_N12DB Represents a gain of -12dB.
 * @param AD9081_ADC_PFIR_GAIN_N6DB Represents a gain of -6dB.
 * @param AD9081_ADC_PFIR_GAIN_0DB Represents a gain of 0dB.
 * @param AD9081_ADC_PFIR_GAIN_P6DB Represents a gain of 6dB.
 * @param AD9081_ADC_PFIR_GAIN_P12DB Represents a gain of 12dB.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_PFIR_GAIN_N12DB = 0x6, /*!< -12dB */
	AD9081_ADC_PFIR_GAIN_N6DB = 0x7, /*!<  -6dB */
	AD9081_ADC_PFIR_GAIN_0DB = 0x0, /*!<   0dB */
	AD9081_ADC_PFIR_GAIN_P6DB = 0x1, /*!<   6dB */
	AD9081_ADC_PFIR_GAIN_P12DB = 0x2 /*!<  12dB */
} adi_ad9081_adc_pfir_gain_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_adc_bypass_mode_e` is an enumeration that defines
 * three distinct modes for the ADC bypass functionality, allowing the
 * selection of the main receive datapath, full bandwidth mode, or test
 * mode.
 *
 * @param AD9081_ADC_MAIN_DP_MODE Represents the main receive datapath mode.
 * @param AD9081_ADC_FBW_MODE Represents the full bandwidth mode bypass
 * datapath.
 * @param AD9081_ADC_TEST_MODE Represents the test mode bypass datapath.
 ******************************************************************************/
typedef enum {
	AD9081_ADC_MAIN_DP_MODE = 0x0, /*!< Main receive datapath */
	AD9081_ADC_FBW_MODE = 0x1, /*!< Full bandwidth mode bypass datapath */
	AD9081_ADC_TEST_MODE = 0x2 /*!< Test mode bypass datapath */
} adi_ad9081_adc_bypass_mode_e;

/***************************************************************************//**
 * @brief `adi_ad9081_jesd_link_select_e` is an enumeration that defines
 * constants for selecting different JESD links in the AD9081 device,
 * allowing for easy reference to specific link configurations.
 *
 * @param AD9081_LINK_NONE Represents the absence of any link.
 * @param AD9081_LINK_0 Represents the first link.
 * @param AD9081_LINK_1 Represents the second link.
 * @param AD9081_LINK_ALL Represents all available links.
 ******************************************************************************/
typedef enum {
	AD9081_LINK_NONE = 0x0, /*!< No Link */
	AD9081_LINK_0 = 0x1, /*!< Link 0 */
	AD9081_LINK_1 = 0x2, /*!< Link 1 */
	AD9081_LINK_ALL = 0x3 /*!< All Links */
} adi_ad9081_jesd_link_select_e;

/***************************************************************************//**
 * @brief `adi_ad9081_jesd_rx_prbs_test_data_src_e` is an enumeration that
 * defines two constants for selecting the source of test data in the
 * AD9081 JESD RX interface, allowing the user to choose between lane
 * data and sample data.
 *
 * @param AD9081_JESD_RX_PRBS_TEST_DATA_SRC_LANE Represents lane data as the
 * test data source.
 * @param AD9081_JESD_RX_PRBS_TEST_DATA_SRC_SAMPLE Represents sample data as the
 * test data source, applicable
 * only for M0.
 ******************************************************************************/
typedef enum {
	AD9081_JESD_RX_PRBS_TEST_DATA_SRC_LANE =
		0x0, /*!< Lane Data As Test Data Source */
	AD9081_JESD_RX_PRBS_TEST_DATA_SRC_SAMPLE =
		0x1 /*!< Sample Data As Test Data Source (M0 Only) */
} adi_ad9081_jesd_rx_prbs_test_data_src_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_jesd_rx_prbs_test_mode_e` is an enumeration that
 * defines various modes for the PRBS (Pseudo-Random Binary Sequence)
 * test in the AD9081 JESD receiver, allowing users to enable or disable
 * the test mode and select from different PRBS lengths.
 *
 * @param AD9081_JESD_RX_PRBS_TEST_MODE_OFF Represents the disabled state of the
 * PRBS Test Mode.
 * @param AD9081_JESD_RX_PRBS_TEST_MODE_PRBS7 Represents the PRBS7 test mode.
 * @param AD9081_JESD_RX_PRBS_TEST_MODE_PRBS9 Represents the PRBS9 test mode.
 * @param AD9081_JESD_RX_PRBS_TEST_MODE_PRBS15 Represents the PRBS15 test mode.
 * @param AD9081_JESD_RX_PRBS_TEST_MODE_PRBS31 Represents the PRBS31 test mode.
 ******************************************************************************/
typedef enum {
	AD9081_JESD_RX_PRBS_TEST_MODE_OFF = 0x0, /*!< Disable PRBS Test Mode */
	AD9081_JESD_RX_PRBS_TEST_MODE_PRBS7 = 0x1, /*!< PRBS7 */
	AD9081_JESD_RX_PRBS_TEST_MODE_PRBS9 = 0x2, /*!< PRBS9 */
	AD9081_JESD_RX_PRBS_TEST_MODE_PRBS15 = 0x3, /*!< PRBS15 */
	AD9081_JESD_RX_PRBS_TEST_MODE_PRBS31 = 0x4 /*!< PRBS31 */
} adi_ad9081_jesd_rx_prbs_test_mode_e;

/***************************************************************************//**
 * @brief `adi_ad9081_jesd_tx_test_data_src_e` is an enumeration that defines
 * different sources of test data for the AD9081 JESD transmitter,
 * allowing the selection of sample data, PHY data, or scrambler input
 * data as the source.
 *
 * @param AD9081_JESD_TX_TEST_DATA_SAMPLE Represents sample data as the test
 * data source.
 * @param AD9081_JESD_TX_TEST_DATA_PHY Represents PHY data as the test data
 * source.
 * @param AD9081_JESD_TX_TEST_DATA_SCRAMBLER_INPUT Represents scrambler input
 * data as the test data source.
 ******************************************************************************/
typedef enum {
	AD9081_JESD_TX_TEST_DATA_SAMPLE =
		0x0, /*!< Sample Data As Test Data Source */
	AD9081_JESD_TX_TEST_DATA_PHY = 0x1, /*!< PHY Data As Test Data Source */
	AD9081_JESD_TX_TEST_DATA_SCRAMBLER_INPUT =
		0x2 /*!< Scrambler Input Data As Data Source */
} adi_ad9081_jesd_tx_test_data_src_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_jesd_tx_test_mode_e` is an enumeration that defines
 * various test modes for the AD9081 JESD transmitter, allowing the
 * selection of different testing patterns such as checkerboard, word
 * toggle, and pseudo-random sequences.
 *
 * @param AD9081_JESD_TX_TEST_MODE_DISABLED Represents the disabled state of the
 * test mode.
 * @param AD9081_JESD_TX_TEST_MODE_CHECKER_BOARD Indicates the checkerboard test
 * mode.
 * @param AD9081_JESD_TX_TEST_MODE_WORD_TOGGLE Specifies the word toggle test
 * mode.
 * @param AD9081_JESD_TX_TEST_MODE_PN31 Denotes the PN31 test mode.
 * @param AD9081_JESD_TX_TEST_MODE_PN15 Represents the PN15 test mode.
 * @param AD9081_JESD_TX_TEST_MODE_PN7 Indicates the PN7 test mode.
 * @param AD9081_JESD_TX_TEST_MODE_RAMP Specifies the ramp test mode.
 * @param AD9081_JESD_TX_TEST_MODE_USER_REPEAT Denotes the repeated user data
 * test mode.
 * @param AD9081_JESD_TX_TEST_MODE_USER_SINGLE Represents the single time user
 * data test mode.
 ******************************************************************************/
typedef enum {
	AD9081_JESD_TX_TEST_MODE_DISABLED = 0x0, /*!< Disable Test Mode */
	AD9081_JESD_TX_TEST_MODE_CHECKER_BOARD =
		0x1, /*!< Checker Board Test Mode */
	AD9081_JESD_TX_TEST_MODE_WORD_TOGGLE =
		0x2, /*!< Word Toggle Test Mode */
	AD9081_JESD_TX_TEST_MODE_PN31 = 0x3, /*!< PN31 Test Mode */
	AD9081_JESD_TX_TEST_MODE_PN15 = 0x5, /*!< PN15 Test Mode */
	AD9081_JESD_TX_TEST_MODE_PN7 = 0x7, /*!< PN7  Test Mode */
	AD9081_JESD_TX_TEST_MODE_RAMP = 0x8, /*!< Ramp Test Mode */
	AD9081_JESD_TX_TEST_MODE_USER_REPEAT =
		0xE, /*!< Repeated User Data Test Mode */
	AD9081_JESD_TX_TEST_MODE_USER_SINGLE =
		0xF /*!< Single Time User Data Test Mode */
} adi_ad9081_jesd_tx_test_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_test_mode_e` is an enumeration that defines various
 * test modes for the AD9081 device, allowing for different operational
 * configurations such as normal operation, various scale tests, and
 * specific pattern sequences.
 *
 * @param AD9081_TMODE_OFF Represents normal operation mode.
 * @param AD9081_TMODE_MIDSCALE Represents midscale short test mode.
 * @param AD9081_TMODE_POS_FULL Represents positive full-scale test mode.
 * @param AD9081_TMODE_NEG_FULL Represents negative full-scale test mode.
 * @param AD9081_TMODE_ALT_CHECKER Represents alternating checkerboard test
 * mode.
 * @param AD9081_TMODE_PN23 Represents PN23 sequence test mode.
 * @param AD9081_TMODE_PN9 Represents PN9 sequence test mode.
 * @param AD9081_TMODE_1_0_TOGG Represents 1/0 word toggle test mode.
 * @param AD9081_TMODE_USER_PAT Represents user-defined pattern test mode.
 * @param AD9081_TMODE_PN7 Represents PN7 sequence test mode.
 * @param AD9081_TMODE_PN15 Represents PN15 sequence test mode.
 * @param AD9081_TMODE_PN31 Represents PN31 sequence test mode.
 * @param AD9081_TMODE_RAMP Represents ramp output test mode.
 ******************************************************************************/
typedef enum {
	AD9081_TMODE_OFF = 0x0, /*!< Normal Operation */
	AD9081_TMODE_MIDSCALE = 0x1, /*!< Midscale Short */
	AD9081_TMODE_POS_FULL = 0x2, /*!< Positive Full-Scale, 0x7FFF */
	AD9081_TMODE_NEG_FULL = 0x3, /*!< Negative Full-Scale, 0x8000 */
	AD9081_TMODE_ALT_CHECKER =
		0x4, /*!< Alternating Checker Board, 0x5555-0xAAAA */
	AD9081_TMODE_PN23 = 0x5, /*!< PN23 Sequence */
	AD9081_TMODE_PN9 = 0x6, /*!< PN9  Sequence */
	AD9081_TMODE_1_0_TOGG = 0x7, /*!< 1/0 Word Toggle, 0x0000-0xFFFF */
	AD9081_TMODE_USER_PAT = 0x8, /*!< User Pattern Test Mode */
	AD9081_TMODE_PN7 = 0x9, /*!< PN7 Sequence */
	AD9081_TMODE_PN15 = 0xA, /*!< PN15 Sequence */
	AD9081_TMODE_PN31 = 0xB, /*!< PN31 Sequence */
	AD9081_TMODE_RAMP = 0xF /*!< Ramp Output */
} adi_ad9081_test_mode_e;

/***************************************************************************//**
 * @brief `adi_ad9081_reset_e` is an enumeration that defines various reset
 * operations for the AD9081 device, allowing for soft and hard resets,
 * as well as options for initialization following these resets.
 *
 * @param AD9081_SOFT_RESET Represents a soft reset operation.
 * @param AD9081_HARD_RESET Represents a hard reset operation.
 * @param AD9081_SOFT_RESET_AND_INIT Represents a soft reset followed by
 * initialization.
 * @param AD9081_HARD_RESET_AND_INIT Represents a hard reset followed by
 * initialization.
 ******************************************************************************/
typedef enum {
	AD9081_SOFT_RESET = 0, /*!< Soft Reset */
	AD9081_HARD_RESET = 1, /*!< Hard Reset */
	AD9081_SOFT_RESET_AND_INIT = 2, /*!< Soft Reset Then Init */
	AD9081_HARD_RESET_AND_INIT = 3 /*!< Hard Reset Then Init */
} adi_ad9081_reset_e;

/***************************************************************************//**
 * @brief `adi_ad9081_deser_mode_e` is an enumeration that defines three modes
 * of operation for the AD9081 device, allowing the user to specify
 * whether the device operates at full, half, or quarter rate.
 *
 * @param AD9081_FULL_RATE Represents full rate operation.
 * @param AD9081_HALF_RATE Represents half rate operation.
 * @param AD9081_QUART_RATE Represents quarter rate operation.
 ******************************************************************************/
typedef enum {
	AD9081_FULL_RATE = 0, /*!< Full rate operation */
	AD9081_HALF_RATE = 1, /*!< Half rate operation */
	AD9081_QUART_RATE = 2 /*!< Quarter rate operation */
} adi_ad9081_deser_mode_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_prbs_test_t` structure is designed to hold the results
 * of a PRBS (Pseudo-Random Binary Sequence) test, including the total
 * error count, the test status indicating whether it passed or failed,
 * and the count of source errors encountered during the test.
 *
 * @param phy_prbs_err_cnt Counts the number of errors detected during the PRBS
 * test.
 * @param phy_prbs_pass Indicates the pass or fail status of the PRBS test.
 * @param phy_src_err_cnt Counts the number of source errors detected during the
 * PRBS test.
 ******************************************************************************/
typedef struct {
	uint32_t phy_prbs_err_cnt; /*!< PRBS Test Error Count */
	uint8_t phy_prbs_pass; /*!< PRBS Test Status */
	uint8_t phy_src_err_cnt; /*!< PRBS Test Source Error Count */
} adi_ad9081_prbs_test_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_spo_t` structure is a simple data type that holds two
 * 8-bit unsigned integers, `left_spo` and `right_spo`, which represent
 * the good SPO values for the left and right channels, respectively.
 *
 * @param left_spo Represents the left good SPO value.
 * @param right_spo Represents the right good SPO value.
 ******************************************************************************/
typedef struct {
	uint8_t left_spo; /*!< Left good SPO */
	uint8_t right_spo; /*!< Right good SPO */
} adi_ad9081_spo_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_ser_swing_e` is an enumeration that defines different
 * voltage swing levels for the AD9081 serializer, allowing for clear and
 * type-safe representation of swing values in the code.
 *
 * @param AD9081_SER_SWING_1000 Represents a 1000 mV swing.
 * @param AD9081_SER_SWING_850 Represents an 850 mV swing.
 * @param AD9081_SER_SWING_750 Represents a 750 mV swing.
 * @param AD9081_SER_SWING_500 Represents a 500 mV swing.
 ******************************************************************************/
typedef enum {
	AD9081_SER_SWING_1000 = 0, /*!< 1000 mV Swing */
	AD9081_SER_SWING_850 = 1, /*!< 850 mV Swing */
	AD9081_SER_SWING_750 = 2, /*!< 750 mV Swing */
	AD9081_SER_SWING_500 = 3 /*!< 500 mV Swing */
} adi_ad9081_ser_swing_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_ser_pre_emp_e` is an enumeration that defines three
 * levels of pre-emphasis for the AD9081 serializer, allowing for 0 dB, 3
 * dB, and 6 dB settings, which are used to adjust signal integrity in
 * high-speed data transmission.
 *
 * @param AD9081_SER_PRE_EMP_0DB Represents a 0 dB pre-emphasis level.
 * @param AD9081_SER_PRE_EMP_3DB Represents a 3 dB pre-emphasis level.
 * @param AD9081_SER_PRE_EMP_6DB Represents a 6 dB pre-emphasis level.
 ******************************************************************************/
typedef enum {
	AD9081_SER_PRE_EMP_0DB = 0, /*!< 0 db Pre-Emphasis */
	AD9081_SER_PRE_EMP_3DB = 1, /*!< 3 db Pre-Emphasis */
	AD9081_SER_PRE_EMP_6DB = 2 /*!< 6 db Pre-Emphasis */
} adi_ad9081_ser_pre_emp_e;

/***************************************************************************//**
 * @brief The `adi_ad9081_ser_post_emp_e` is an enumeration that defines various
 * levels of post-emphasis for the AD9081 serializer, allowing for the
 * selection of specific emphasis levels ranging from 0 dB to 12 dB.
 *
 * @param AD9081_SER_POST_EMP_0DB Represents a 0 dB post-emphasis level.
 * @param AD9081_SER_POST_EMP_3DB Represents a 3 dB post-emphasis level.
 * @param AD9081_SER_POST_EMP_6DB Represents a 6 dB post-emphasis level.
 * @param AD9081_SER_POST_EMP_9DB Represents a 9 dB post-emphasis level.
 * @param AD9081_SER_POST_EMP_12DB Represents a 12 dB post-emphasis level.
 ******************************************************************************/
typedef enum {
	AD9081_SER_POST_EMP_0DB = 0, /*!< 0 db Post-Emphasis */
	AD9081_SER_POST_EMP_3DB = 1, /*!< 3 db Post-Emphasis */
	AD9081_SER_POST_EMP_6DB = 2, /*!< 6 db Post-Emphasis */
	AD9081_SER_POST_EMP_9DB = 3, /*!< 9 db Post-Emphasis */
	AD9081_SER_POST_EMP_12DB = 4 /*!< 12 db Post-Emphasis */
} adi_ad9081_ser_post_emp_e;
/***************************************************************************//**
 * @brief The `adi_ad9081_cal_mode_e` is an enumeration that defines three
 * calibration modes for the AD9081 device, allowing the user to select
 * between running the calibration, running and saving coefficients, or
 * bypassing the calibration process.
 *
 * @param AD9081_CAL_MODE_RUN Represents the mode to run 204C QR Calibration.
 * @param AD9081_CAL_MODE_RUN_AND_SAVE Represents the mode to run 204C QR
 * Calibration and save CTLE Coefficients.
 * @param AD9081_CAL_MODE_BYPASS Represents the mode to bypass 204C QR
 * Calibration and load CTLE Coefficients.
 ******************************************************************************/
typedef enum {
	AD9081_CAL_MODE_RUN = 0, /*!< Run 204C QR Calibration*/
	AD9081_CAL_MODE_RUN_AND_SAVE =
		1, /*!< Run 204C QR Calibration and save CTLE Coefficients*/
	AD9081_CAL_MODE_BYPASS =
		2 /*!< Bypass 204C QR Calibration and load CTLE Coefficients*/
} adi_ad9081_cal_mode_e;
/***************************************************************************//**
 * @brief The `adi_ad9081_ser_lane_settings_t` structure is used to configure
 * the settings for a serial lane in the AD9081 device, encapsulating
 * parameters for swing, pre-emphasis, and post-emphasis, which are
 * essential for optimizing signal integrity.
 *
 * @param swing_setting Specifies the swing setting for the lane.
 * @param pre_emp_setting Defines the pre-emphasis setting for the lane.
 * @param post_emp_setting Indicates the post-emphasis setting for the lane.
 ******************************************************************************/
typedef struct {
	adi_ad9081_ser_swing_e swing_setting;
	adi_ad9081_ser_pre_emp_e pre_emp_setting;
	adi_ad9081_ser_post_emp_e post_emp_setting;
} adi_ad9081_ser_lane_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_ser_settings_t` structure is designed to hold
 * configuration settings for the AD9081 serializer, including an array
 * of lane settings, an invert mask for lane inversion, and a mapping of
 * physical lanes to logical lanes, facilitating the deserialization
 * process.
 *
 * @param lane_settings An array of `adi_ad9081_ser_lane_settings_t` structures
 * for lane configuration.
 * @param invert_mask A bitmask to indicate which lanes should be inverted.
 * @param lane_mapping A 2D array mapping physical lanes to logical lanes for
 * deserialization.
 ******************************************************************************/
typedef struct {
	adi_ad9081_ser_lane_settings_t lane_settings[8];
	uint8_t invert_mask;
	uint8_t lane_mapping
		[2]
		[8]; /*Deserialise Lane Mapping, Map Virtual Converter to Physical Lane, index is physical, value is logical lane*/
} adi_ad9081_ser_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_des_settings_t` structure is designed to hold
 * configuration settings for the AD9081 deserializer, including masks
 * for calibration and inversion, CTLE filter selections, calibration
 * mode flags, CTLE coefficients for each lane, and a mapping of logical
 * to physical lanes.
 *
 * @param boost_mask Mask to enable calibration boost mode per lane.
 * @param invert_mask Mask for lane inversion.
 * @param ctle_filter Array for equaliser CTLE filter selection.
 * @param cal_mode Flag to configure the calibration run.
 * @param ctle_coeffs 2D array for per lane CTLE coefficient settings.
 * @param lane_mapping 2D array for deserializing lane mapping from virtual to
 * physical lanes.
 ******************************************************************************/
typedef struct {
	uint8_t boost_mask; /*Calibration boost mode enable Mask,Set per Lane,Enable (Set to 1) if the channels insertion loss is greater than 10 dB*/
	uint8_t invert_mask; /*Lane Inversion Mask*/
	uint8_t ctle_filter
		[8]; /*Equaliser CTLE Filter Selection, Range 0 - 4, based on Jesd IL, Pick lower setting for Higher Insertion loss*/
	adi_ad9081_cal_mode_e cal_mode; /*Flag to configure calibration run */
	uint8_t ctle_coeffs[8][4]; /*Per lane CTLE coefficient settings */
	uint8_t lane_mapping
		[2]
		[8]; /*Deserialise Lane Mapping, Map Virtual Converter to Physical Lane, index is logical lane, value is physical lane*/
} adi_ad9081_des_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_serdes_settings_t` structure encapsulates the
 * configuration settings for both the transmitter and receiver
 * components of a JESD204 interface, specifically holding the serializer
 * and deserializer settings as defined by the
 * `adi_ad9081_ser_settings_t` and `adi_ad9081_des_settings_t` types.
 *
 * @param ser_settings Settings for the Jesd Tx Serializer.
 * @param des_settings Settings for the Jesd Rx Deserializer.
 ******************************************************************************/
typedef struct {
	adi_ad9081_ser_settings_t ser_settings; /*! Jesd Tx Serializer Settings */
	adi_ad9081_des_settings_t
		des_settings; /*! Jesd Rx Deserializer Settings */
} adi_ad9081_serdes_settings_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_jtx_conv_sel_t` structure is designed to hold the
 * indices for multiple JTX virtual converters, specifically from 0 to
 * 15, allowing for the selection and management of these converters in a
 * system.
 *
 * @param virtual_converter0_index Index for JTX virtual converter0.
 * @param virtual_converter1_index Index for JTX virtual converter1.
 * @param virtual_converter2_index Index for JTX virtual converter2.
 * @param virtual_converter3_index Index for JTX virtual converter3.
 * @param virtual_converter4_index Index for JTX virtual converter4.
 * @param virtual_converter5_index Index for JTX virtual converter5.
 * @param virtual_converter6_index Index for JTX virtual converter6.
 * @param virtual_converter7_index Index for JTX virtual converter7.
 * @param virtual_converter8_index Index for JTX virtual converter8.
 * @param virtual_converter9_index Index for JTX virtual converter9.
 * @param virtual_convertera_index Index for JTX virtual converter10.
 * @param virtual_converterb_index Index for JTX virtual converter11.
 * @param virtual_converterc_index Index for JTX virtual converter12.
 * @param virtual_converterd_index Index for JTX virtual converter13.
 * @param virtual_convertere_index Index for JTX virtual converter14.
 * @param virtual_converterf_index Index for JTX virtual converter15.
 ******************************************************************************/
typedef struct {
	uint8_t virtual_converter0_index; /*! Index for JTX virtual converter0  */
	uint8_t virtual_converter1_index; /*! Index for JTX virtual converter1  */
	uint8_t virtual_converter2_index; /*! Index for JTX virtual converter2  */
	uint8_t virtual_converter3_index; /*! Index for JTX virtual converter3  */
	uint8_t virtual_converter4_index; /*! Index for JTX virtual converter4  */
	uint8_t virtual_converter5_index; /*! Index for JTX virtual converter5  */
	uint8_t virtual_converter6_index; /*! Index for JTX virtual converter6  */
	uint8_t virtual_converter7_index; /*! Index for JTX virtual converter7  */
	uint8_t virtual_converter8_index; /*! Index for JTX virtual converter8  */
	uint8_t virtual_converter9_index; /*! Index for JTX virtual converter9  */
	uint8_t virtual_convertera_index; /*! Index for JTX virtual converter10 */
	uint8_t virtual_converterb_index; /*! Index for JTX virtual converter11 */
	uint8_t virtual_converterc_index; /*! Index for JTX virtual converter12 */
	uint8_t virtual_converterd_index; /*! Index for JTX virtual converter13 */
	uint8_t virtual_convertere_index; /*! Index for JTX virtual converter14 */
	uint8_t virtual_converterf_index; /*! Index for JTX virtual converter15 */
} adi_ad9081_jtx_conv_sel_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_hal_t` structure is designed to encapsulate various
 * configurations and function pointers necessary for the hardware
 * abstraction layer (HAL) of the AD9081 device, including user data, SPI
 * interface settings, and control functions for initialization, logging,
 * and pin management.
 *
 * @param user_data Pointer to connect customer data related to this device.
 * @param sdo SPI interface 3/4 wire mode configuration.
 * @param msb SPI interface MSB/LSB bit order configuration.
 * @param addr_inc SPI interface address increment configuration.
 * @param spi_xfer Function pointer to HAL SPI access function.
 * @param delay_us Function pointer to HAL delay function.
 * @param hw_open Function pointer to HAL initialization function.
 * @param hw_close Function pointer to HAL de-initialization function.
 * @param log_write Function pointer to HAL log write function.
 * @param tx_en_pin_ctrl Function pointer to HAL tx_enable pin control function.
 * @param reset_pin_ctrl Function pointer to HAL reset# pin control function.
 ******************************************************************************/
typedef struct {
	void *user_data; /*!< Pointer to connect customer data related to this device */

	adi_cms_spi_sdo_config_e
		sdo; /*!< SPI interface 3/4 wire mode configuration */
	adi_cms_spi_msb_config_e
		msb; /*!< SPI interface MSB/LSB bit order configuration */
	adi_cms_spi_addr_inc_e
		addr_inc; /*!< SPI interface address increment configuration */

	adi_spi_xfer_t
		spi_xfer; /*!< Function pointer to hal spi access function */
	adi_delay_us_t delay_us; /*!< Function pointer to hal delay function */
	adi_hw_open_t
		hw_open; /*!< Function pointer to hal initialization function */
	adi_hw_close_t
		hw_close; /*!< Function pointer to hal de-initialization function */
	adi_log_write_t
		log_write; /*!< Function pointer to hal log write function */
	adi_tx_en_pin_ctrl_t
		tx_en_pin_ctrl; /*!< Function pointer to hal tx_enable pin control function */
	adi_reset_pin_ctrl_t
		reset_pin_ctrl; /*!< Function pointer to hal reset# pin control function */
} adi_ad9081_hal_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_info_t` structure is designed to hold various
 * configuration and operational parameters for the AD9081 device,
 * including clock frequencies for the device, DAC, and ADC, as well as
 * device revision, product ID, and JESD RX lane rate.
 *
 * @param dev_freq_hz Device clock frequency in Hz.
 * @param dac_freq_hz DAC clock frequency in Hz.
 * @param adc_freq_hz ADC clock frequency in Hz.
 * @param dev_rev Device revision, represented as an integer.
 * @param prod_id Product identifier.
 * @param jesd_rx_lane_rate JESD RX link lane rate.
 ******************************************************************************/
typedef struct {
	uint64_t dev_freq_hz; /*!< Device clock frequency in Hz */
	uint64_t dac_freq_hz; /*!< DAC clock frequency in Hz */
	uint64_t adc_freq_hz; /*!< ADC clock frequency in Hz */
	uint8_t dev_rev; /*!< Device revision, 0:r0, 1:r1, 2:r1r, 3:r2 */
	uint8_t prod_id;
	uint64_t jesd_rx_lane_rate; /*!< jrx link lane rate */
} adi_ad9081_info_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_clk_t` structure is designed to encapsulate the
 * configuration and control of clock sources related to system reference
 * operations in the AD9081 device. It includes a pointer to the clock
 * source, a function pointer for system reference control, and an
 * enumeration to specify the synchronization mode, allowing for flexible
 * and efficient management of clock signals.
 *
 * @param sysref_clk Pointer to the clock source related to system reference
 * control.
 * @param sysref_ctrl Function pointer for controlling the system reference.
 * @param sysref_mode Enumeration for configuring the system reference
 * synchronization mode.
 ******************************************************************************/
typedef struct {
	void *sysref_clk; /*!< Clk source related to sysref ctrl */
	adi_sysref_ctrl_t
		sysref_ctrl; /*!< Function pointer to sysref control function */
	adi_cms_jesd_sysref_mode_e
		sysref_mode; /*!< sysref synchronization mode configuration */
} adi_ad9081_clk_t;

/***************************************************************************//**
 * @brief The `adi_ad9081_device_t` structure is a composite data type that
 * encapsulates various components necessary for managing an AD9081
 * device, including hardware abstraction, device information,
 * serializer/deserializer settings, and clock configurations.
 *
 * @param hal_info Contains hardware abstraction layer information for the
 * device.
 * @param dev_info Holds device-specific information and configuration.
 * @param serdes_info Stores settings related to the serializer/deserializer
 * configuration.
 * @param clk_info Includes clock-related information for the device operation.
 ******************************************************************************/
typedef struct {
	adi_ad9081_hal_t hal_info;
	adi_ad9081_info_t dev_info;
	adi_ad9081_serdes_settings_t serdes_info;
	adi_ad9081_clk_t clk_info;
} adi_ad9081_device_t;

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/*===== 1 . 0   D E V I C E   I N I T   &  C L O C K I N G =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the major, minor, and release
 * candidate (RC) version numbers of the API associated with a specific
 * device. It should be called after the device has been properly
 * initialized. The function expects valid pointers for the revision
 * parameters, and it will return an error if any of the pointers are
 * null. Upon successful execution, the provided pointers will be
 * populated with the respective revision values.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rev_major A pointer to a `uint8_t` where the major revision number
 * will be stored. Must not be null.
 * @param rev_minor A pointer to a `uint8_t` where the minor revision number
 * will be stored. Must not be null.
 * @param rev_rc A pointer to a `uint8_t` where the release candidate number
 * will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the revision
 * information has been successfully retrieved.
 ******************************************************************************/
int32_t adi_ad9081_device_api_revision_get(adi_ad9081_device_t *device,
					   uint8_t *rev_major,
					   uint8_t *rev_minor, uint8_t *rev_rc);

/***************************************************************************//**
 * @brief This function is used to reset the AD9081 device, either through a
 * soft or hard reset, depending on the specified operation. It must be
 * called with a valid `device` pointer that has been properly
 * initialized. The function can perform a soft reset, a hard reset, or
 * both, with an optional initialization following the reset. If an
 * invalid operation is provided, or if the `device` pointer is null, the
 * function will return an error. It is important to ensure that the
 * device is in a state that allows for a reset before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be reset. Must not be null.
 * @param operation An enumeration value of type `adi_ad9081_reset_e` that
 * specifies the type of reset to perform. Valid values include
 * `AD9081_SOFT_RESET`, `AD9081_HARD_RESET`,
 * `AD9081_SOFT_RESET_AND_INIT`, and
 * `AD9081_HARD_RESET_AND_INIT`. If an invalid value is
 * provided, the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful reset, or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_device_reset(adi_ad9081_device_t *device,
				adi_ad9081_reset_e operation);

/***************************************************************************//**
 * @brief This function is used to initialize the AD9081 device, preparing it
 * for operation. It must be called after the device structure has been
 * allocated and before any other device operations are performed. The
 * function checks for a null pointer and logs the API version
 * information. It also configures the SPI communication settings,
 * verifies the ability to read and write to an 8-bit register, and
 * checks the power status of the device. If any of these checks fail, an
 * error code is returned, indicating the specific issue.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be initialized. Must not be null; otherwise, the
 * function will return an error.
 * @return Returns an integer error code indicating the success or failure of
 * the initialization process. A return value of `API_CMS_ERROR_OK`
 * indicates successful initialization.
 ******************************************************************************/
int32_t adi_ad9081_device_init(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to set the clock frequencies for the DAC, ADC,
 * and reference clock of the device. It must be called after the device
 * has been initialized and before any data processing occurs. The
 * function validates the provided clock frequencies against predefined
 * limits and ensures that the reference clock is within acceptable
 * bounds. If any of the parameters are invalid, the function will return
 * an error code. Additionally, it performs several internal checks and
 * configurations to enable the digital logic and clock settings,
 * ensuring that the device operates correctly with the specified clock
 * frequencies.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dac_clk_hz The desired DAC clock frequency in Hertz. Must be within
 * the range defined by `AD9081_DAC_CLK_FREQ_HZ_MIN` and
 * `AD9081_DAC_CLK_FREQ_HZ_MAX`.
 * @param adc_clk_hz The desired ADC clock frequency in Hertz. Must be within
 * the range defined by `AD9081_ADC_CLK_FREQ_HZ_MIN` and
 * `AD9081_ADC_CLK_FREQ_HZ_MAX` based on the product ID. Must
 * not be zero.
 * @param ref_clk_hz The reference clock frequency in Hertz. Must be within the
 * range defined by `AD9081_REF_CLK_FREQ_HZ_MIN` and
 * `AD9081_REF_CLK_FREQ_HZ_MAX`. If it exceeds the maximum,
 * the PLL will be disabled.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the configuration fails.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_config_set(adi_ad9081_device_t *device,
					 uint64_t dac_clk_hz,
					 uint64_t adc_clk_hz,
					 uint64_t ref_clk_hz);

/*===== 1 . 1   D E V I C E   I N I T / D E I N I T  &  H W  P L A T F O R M =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the chip ID details from the specified
 * device. It must be called after the device has been properly
 * initialized. The function retrieves various identifiers, including the
 * chip type, product ID, product grade, and device revision, and
 * populates the provided `chip_id` structure with this information. If
 * either the `device` or `chip_id` pointers are null, the function will
 * return an error. Additionally, if any register read operation fails,
 * the function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null. If null, the function will return
 * an error.
 * @param chip_id A pointer to an `adi_cms_chip_id_t` structure where the chip
 * ID information will be stored. Must not be null. If null, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the chip ID
 * information has been successfully retrieved and stored in the
 * `chip_id` structure. If an error occurs during the process, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_device_chip_id_get(adi_ad9081_device_t *device,
				      adi_cms_chip_id_t *chip_id);

/***************************************************************************//**
 * @brief This function is used to obtain the laminate ID from an AD9081 device.
 * It must be called with a valid `device` pointer that has been properly
 * initialized. The `id` parameter must also be a valid pointer to a
 * buffer where the laminate ID will be stored. If either pointer is
 * null, the function will return an error without modifying the output
 * buffer. This function is typically called when the laminate ID is
 * needed for identification or configuration purposes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param id A pointer to a buffer where the laminate ID will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t adi_ad9081_device_laminate_id_get(adi_ad9081_device_t *device,
					  uint8_t *id);

/***************************************************************************//**
 * @brief This function is used to obtain the die ID of a specified AD9081
 * device. It must be called with a valid device pointer that has been
 * properly initialized. The function will write the die ID into the
 * provided `id` buffer, which must also be valid and allocated by the
 * caller. If either the device pointer or the id pointer is null, the
 * function will return an error without modifying any output. This
 * function is typically called during device initialization or
 * configuration to verify the identity of the hardware.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and should point to a valid
 * initialized device.
 * @param id A pointer to a `uint8_t` buffer where the die ID will be stored.
 * Must not be null and should point to a valid memory location with
 * sufficient space to hold the die ID.
 * @return Returns the die ID of the device as an integer value. If successful,
 * the die ID is written to the memory location pointed to by `id`. If
 * there is an error (e.g., null pointer), the function returns an error
 * code.
 ******************************************************************************/
int32_t adi_ad9081_device_die_id_get(adi_ad9081_device_t *device, uint8_t *id);

/***************************************************************************//**
 * @brief This function is used to establish a connection to the hardware
 * interface of the specified device. It must be called with a valid
 * device pointer that has been properly initialized. If the device
 * pointer is null, the function will return an error without attempting
 * to open the hardware interface. Upon successful execution, it prepares
 * the device for further operations. It is important to handle the
 * return value appropriately to ensure that the hardware interface is
 * opened successfully.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be opened. Must not be null; otherwise, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` if the hardware interface is opened
 * successfully, or an error code indicating the failure reason if the
 * operation is unsuccessful.
 ******************************************************************************/
int32_t adi_ad9081_device_hw_open(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function should be called to properly close the hardware
 * interface associated with a device after it has been used. It is
 * essential to ensure that the `device` parameter is valid and not null
 * before invoking this function. Failing to do so will result in an
 * immediate return with an error code. This function is typically called
 * during the cleanup phase of an application to release resources
 * associated with the device.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be closed. Must not be null; otherwise, the
 * function will return an error code.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of 0 typically indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_hw_close(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to safely deinitialize an AD9081 device,
 * ensuring that it is properly reset before being released or
 * reconfigured. It must be called when the device is no longer needed,
 * typically during shutdown or before reinitialization. The function
 * performs both a hardware and a software reset to ensure that the
 * device is in a clean state. It is important to ensure that the
 * `device` parameter is valid and not null before calling this function,
 * as passing a null pointer will result in an immediate error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be deinitialized. Must not be null; passing a
 * null pointer will trigger an error.
 * @return Returns `API_CMS_ERROR_OK` on successful deinitialization, or an
 * error code if the reset operations fail.
 ******************************************************************************/
int32_t adi_ad9081_device_deinit(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to configure the Serial Peripheral Interface
 * (SPI) settings for the specified device. It must be called after the
 * device has been properly initialized and before any SPI communication
 * occurs. The function checks the configuration parameters of the device
 * and sets the appropriate register values accordingly. If the provided
 * device pointer is null, the function will return an error. It is
 * important to ensure that the device is in a valid state before
 * invoking this function to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be configured. Must not be null; otherwise, the
 * function will return an error.
 * @return Returns an integer value indicating the success or failure of the
 * configuration operation. A return value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_spi_config(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to configure a specific register of the AD9081
 * device by writing a value to it. It must be called with a valid
 * `device` pointer that has been properly initialized. If the `device`
 * pointer is null, the function will return an error without making any
 * changes. The `addr` parameter specifies the register address to be
 * written to, and the `data` parameter is the value to be written. It is
 * important to ensure that the address is valid for the device to avoid
 * undefined behavior. The function will return an error code if the
 * operation fails, allowing the caller to handle the error
 * appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr The address of the register to be set. Must be a valid register
 * address for the AD9081 device.
 * @param data The value to write to the specified register. Must be a valid
 * 8-bit value.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_device_spi_register_set(adi_ad9081_device_t *device,
					   uint16_t addr, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to read a value from a specific register of the
 * AD9081 device. It must be called with a valid `device` pointer that
 * has been properly initialized and a valid `data` pointer where the
 * retrieved value will be stored. If either pointer is null, the
 * function will return an error without performing any read operation.
 * The `addr` parameter specifies the register address to read from, and
 * it should be within the valid range defined by the device's register
 * map. This function is typically used in scenarios where configuration
 * or status information needs to be obtained from the device.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr The address of the register to read from. It should be within the
 * valid range of register addresses for the AD9081 device.
 * @param data A pointer to a `uint8_t` where the read value will be stored.
 * Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the value has
 * been successfully retrieved. If an error occurs, a negative error
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_spi_register_get(adi_ad9081_device_t *device,
					   uint16_t addr, uint8_t *data);

/*===== 1 . 2   B L O C K  L E V E L  C L O C K  A P I =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the current phase-locked loop (PLL)
 * lock status of the specified device. It should be called after the
 * device has been properly initialized. The function checks the PLL lock
 * status and updates the provided status pointer with the result, where
 * the least significant bits indicate the lock status. If the device or
 * status pointer is null, the function will return an error. It is
 * important to ensure that the pointers passed to this function are
 * valid to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the PLL lock status
 * will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the PLL lock
 * status has been successfully retrieved and stored in the provided
 * status pointer.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_pll_lock_status_get(adi_ad9081_device_t *device,
						  uint8_t *status);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ACLK receiver in the
 * specified device. It should be called after the device has been
 * properly initialized. The `enable` parameter determines the state of
 * the ACLK receiver, where a value of 1 enables it and a value of 0
 * disables it. If the `device` pointer is null or if the `enable`
 * parameter is not within the valid range, the function will handle
 * these cases by returning an appropriate error code. It is important to
 * check the return value to ensure that the operation was successful.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A uint8_t value that indicates whether to enable (1) or disable
 * (0) the ACLK receiver. Valid values are 0 and 1; any other
 * value will result in an error.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_device_aclk_receiver_enable_set(adi_ad9081_device_t *device,
						   uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the clock divider for the ADC in
 * the device. It should be called after the device has been properly
 * initialized. The `div` parameter specifies the desired divider ratio,
 * which must be greater than zero. If the provided `device` pointer is
 * null, the function will return an error without making any changes. It
 * is important to ensure that the divider ratio is within the valid
 * range to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param div An 8-bit unsigned integer representing the divider ratio. Must be
 * greater than zero.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_clk_div_set(adi_ad9081_device_t *device, uint8_t div);

/***************************************************************************//**
 * @brief This function is used to control the state of the ADC clock in the
 * device. It should be called after the device has been properly
 * initialized. The `enable` parameter determines whether the ADC clock
 * is turned on or off. If the function is called with a null `device`
 * pointer, it will return an error. Additionally, if there is an error
 * during the operation, it will also return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the ADC clock.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_clk_enable_set(adi_ad9081_device_t *device,
				      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to control the main automatic clock generation
 * feature of the device. It should be called after the device has been
 * properly initialized. The `enable` parameter determines whether the
 * clock generation is turned on or off. If the `device` pointer is null,
 * the function will return an error without making any changes. It is
 * important to ensure that the device is in a valid state before
 * invoking this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the automatic clock generation.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_main_auto_clk_gen_enable(adi_ad9081_device_t *device,
						   uint8_t enable);

/*===== 2 . 0   T R A N S M I T  P A T H  S E T U P =====*/
/***************************************************************************//**
 * @brief This function is used to initiate a NCO (Numerically Controlled
 * Oscillator) test on the AD9081 device. It must be called after the
 * device has been properly initialized. The function configures the data
 * path and enables the test mode based on the provided interpolation
 * settings. It is important to ensure that the `device` parameter is not
 * null, as passing a null pointer will result in an error. The function
 * also handles enabling or disabling test tones based on the channel
 * interpolation value, and it applies a DC offset to the test tones. If
 * any of the internal operations fail, the function will return an error
 * code.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param main_interp Main interpolation factor, valid values depend on the
 * device specifications.
 * @param chan_interp Channel interpolation factor, valid values depend on the
 * device specifications.
 * @param dac_chan Array of 4 channel indices for DAC configuration. Must not be
 * null.
 * @param main_shift Array of 4 main shift values for DAC configuration. Must
 * not be null.
 * @param chan_shift Array of 8 channel shift values for DAC configuration. Must
 * not be null.
 * @param dc_offset DC offset value to be applied to the test tones. Valid range
 * depends on the device specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t
adi_ad9081_device_startup_nco_test(adi_ad9081_device_t *device,
				   uint8_t main_interp, uint8_t chan_interp,
				   uint8_t dac_chan[4], int64_t main_shift[4],
				   int64_t chan_shift[8], uint16_t dc_offset);

/***************************************************************************//**
 * @brief This function is used to initialize the transmit path of the AD9081
 * device, configuring the necessary parameters for operation. It must be
 * called after the device has been properly initialized and before any
 * data transmission occurs. The function expects valid pointers for the
 * device and JESD parameters, and it will return an error if any of the
 * pointers are null. Additionally, it is important to ensure that the
 * interpolation values and shifts are set according to the device
 * specifications to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be configured. Must not be null.
 * @param main_interp An 8-bit integer representing the main interpolation
 * factor. Valid values depend on the device specifications.
 * @param chan_interp An 8-bit integer representing the channel interpolation
 * factor. Valid values depend on the device specifications.
 * @param dac_chan An array of 4 8-bit integers specifying the DAC channels to
 * be used. Must not be null.
 * @param main_shift An array of 4 64-bit integers representing the main shift
 * values for the DAC channels. Must not be null.
 * @param chan_shift An array of 8 64-bit integers representing the channel
 * shift values. Must not be null.
 * @param jesd_param A pointer to an `adi_cms_jesd_param_t` structure containing
 * JESD parameters. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on successful startup, or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_device_startup_tx(adi_ad9081_device_t *device,
				     uint8_t main_interp, uint8_t chan_interp,
				     uint8_t dac_chan[4], int64_t main_shift[4],
				     int64_t chan_shift[8],
				     adi_cms_jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function is used to configure the NCO gains for each of the DAC
 * channels in the specified device. It must be called after the device
 * has been properly initialized. The function expects an array of gains,
 * where each element corresponds to a specific DAC channel. If the
 * provided `device` pointer is null, the function will return an error.
 * Additionally, if any of the gain settings are invalid, the function
 * will handle the error accordingly, ensuring that all channels are
 * configured correctly before returning.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param gains An array of 8 `uint16_t` values representing the NCO gains for
 * each DAC channel. The caller retains ownership of this array.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration of all DAC
 * channels, or an error code if any operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_gains_set(adi_ad9081_device_t *device,
					 uint16_t gains[8]);

/***************************************************************************//**
 * @brief This function configures the modulation multiplexing mode for a
 * specified DAC pair in the AD9081 device. It must be called with a
 * valid `device` pointer that has been properly initialized. The
 * `dac_pair` parameter specifies which DAC pair to configure, and it
 * must not be zero. The `mode` parameter must be within the valid range
 * of 0 to 3. If any of these parameters are invalid, the function will
 * return an error. After setting the mode, the function also resets the
 * DAC mode switch group to a default state.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dac_pair Specifies the DAC pair to configure. Must not be zero.
 * @param mode The modulation multiplexing mode to set, which must be between 0
 * and 3 inclusive.
 * @return Returns an integer status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t
adi_ad9081_dac_modulation_mux_mode_set(adi_ad9081_device_t *device,
				       adi_ad9081_dac_pair_select_e dac_pair,
				       adi_ad9081_dac_mod_mux_mode_e mode);

/***************************************************************************//**
 * @brief This function is used to enable or disable complex modulation for a
 * specified Digital-to-Analog Converter (DAC) pair in the AD9081 device.
 * It must be called with a valid `device` pointer that has been properly
 * initialized. The `dac_pair` parameter specifies which DAC pair to
 * configure, and the `enable` parameter determines whether to enable (1)
 * or disable (0) the complex modulation. If the `enable` value is not 0
 * or 1, or if the `dac_pair` is invalid, the function will return an
 * error. It is important to ensure that the device is not null before
 * calling this function, as passing a null pointer will also result in
 * an error.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dac_pair Specifies the DAC pair to configure. Must be a valid value;
 * passing 0x0 is considered invalid.
 * @param enable A boolean value (0 or 1) indicating whether to enable or
 * disable complex modulation. Values outside this range are
 * invalid.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or internal errors.
 ******************************************************************************/
int32_t adi_ad9081_dac_complex_modulation_enable_set(
	adi_ad9081_device_t *device, adi_ad9081_dac_pair_select_e dac_pair,
	uint8_t enable);

/*===== 2 . 1   T R A N S M I T  T X E N =====*/
/***************************************************************************//**
 * @brief This function is used to control the transmit state machine for one or
 * more Digital-to-Analog Converters (DACs) in the specified device. It
 * should be called after the device has been properly initialized and
 * configured. The `dacs` parameter allows selection of which DACs to
 * enable or disable, while the `enable` parameter determines the state
 * (enabled or disabled). If an invalid pointer is provided for the
 * `device`, or if an error occurs while selecting a DAC or setting the
 * state machine, the function will handle these cases appropriately. It
 * is important to ensure that the `dacs` parameter correctly represents
 * the DACs intended for control.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask representing the DACs to be enabled or disabled. Valid
 * values are combinations of `AD9081_DAC_0`, `AD9081_DAC_1`,
 * `AD9081_DAC_2`, and `AD9081_DAC_3`. Invalid values will be
 * ignored.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the transmit state machine for the selected
 * DACs.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs during
 * execution, an appropriate error code will be returned.
 ******************************************************************************/
int32_t
adi_ad9081_dac_tx_enable_state_machine_enable_set(adi_ad9081_device_t *device,
						  uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the SPI transmission for
 * one or more Digital-to-Analog Converters (DACs) associated with the
 * specified device. It should be called after the device has been
 * properly initialized. The `dacs` parameter allows you to specify which
 * DACs to modify, and the `enable` parameter determines whether to
 * enable or disable the transmission. If the `device` pointer is null,
 * the function will return an error. Additionally, if any DAC selection
 * or setting operation fails, the function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to enable or disable. Valid
 * values are combinations of `AD9081_DAC_0`, `AD9081_DAC_1`,
 * `AD9081_DAC_2`, and `AD9081_DAC_3`. Each bit corresponds to a
 * DAC.
 * @param enable A boolean value where 1 enables SPI transmission and 0 disables
 * it. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_spi_as_tx_en_set(adi_ad9081_device_t *device,
					uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the transmit functionality
 * of specific Digital-to-Analog Converters (DACs) in the device. It
 * should be called after the device has been properly initialized. The
 * `dacs` parameter specifies which DACs to modify, and the `enable`
 * parameter determines whether to enable (non-zero) or disable (zero)
 * the transmit functionality. If the `device` pointer is null, the
 * function will return an error. The function iterates through the
 * specified DACs and applies the enable/disable setting accordingly,
 * ensuring that each DAC is configured correctly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to enable or disable. Valid
 * values are combinations of `AD9081_DAC_0`, `AD9081_DAC_1`,
 * `AD9081_DAC_2`, and `AD9081_DAC_3`.
 * @param enable A boolean value where a non-zero value enables the DACs and
 * zero disables them.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_dac_tx_enable_set(adi_ad9081_device_t *device, uint8_t dacs,
				     uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the DAC GPIO to enable or disable
 * the transmission (TX) functionality. It must be called with a valid
 * `device` pointer that has been properly initialized. The `enable`
 * parameter determines whether the TX functionality is enabled (1) or
 * disabled (0). If the `enable` parameter is set to a value greater than
 * 1, the function will return an error. Additionally, if the `device`
 * pointer is null, the function will also return an error. It is
 * important to ensure that the device is ready for configuration before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable An 8-bit unsigned integer that specifies the TX enable state.
 * Valid values are 0 (disable) and 1 (enable). Values greater
 * than 1 will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_gpio_as_tx_en_set(adi_ad9081_device_t *device,
					 uint8_t enable);

/*===== 2 . 2   T R A N S M I T  D A C  A N A L O G  C O R E  =====*/
/***************************************************************************//**
 * @brief This function is used to control the power state of the Digital-to-
 * Analog Converters (DACs) in the specified device. It should be called
 * after the device has been properly initialized. The `dacs` parameter
 * specifies which DACs to power up or down, and the `enable` parameter
 * determines whether to power them up (1) or down (0). If an invalid
 * `device` pointer is provided, or if the `dacs` or `enable` parameters
 * are out of range, the function will return an error. It is important
 * to ensure that the `dacs` parameter does not exceed the maximum
 * allowed value, and that `enable` is either 0 or 1.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to power up or down. Valid values
 * are within the range defined by `AD9081_DAC_ALL`. If the value
 * exceeds this range, an error is returned.
 * @param enable A flag indicating whether to power up (1) or power down (0) the
 * specified DACs. Must be either 0 or 1; otherwise, an error is
 * returned.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating the operation was
 * completed successfully. If an error occurs, a negative error code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_power_up_set(adi_ad9081_device_t *device, uint8_t dacs,
				    uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the full-scale current for one or more
 * Digital-to-Analog Converters (DACs) in the device. It should be called
 * after the device has been properly initialized and before any DAC
 * output is expected. The function validates the specified full-scale
 * current against design recommendations, issuing warnings if the value
 * is outside the recommended range of 7mA to 40mA. If the `rerun_cal`
 * parameter is set, it will trigger a calibration process after setting
 * the current. It is important to ensure that the `device` pointer is
 * not null, and the `dacs` parameter should specify which DACs to
 * configure. Invalid parameters will result in error handling as defined
 * in the implementation.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param uA The desired full-scale current in microamperes (uA). Valid range is
 * 7000 to 40000. Values outside this range will trigger warnings.
 * @param rerun_cal A flag indicating whether to rerun calibration after setting
 * the current. Valid values are 0 (no calibration) and 1
 * (rerun calibration). Values greater than 1 are considered
 * invalid.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if an error occurs during execution.
 ******************************************************************************/
int32_t adi_ad9081_dac_fsc_set(adi_ad9081_device_t *device, uint8_t dacs,
			       uint32_t uA, uint8_t rerun_cal);

/*===== 2 . 3   T R A N S M I T  C H A N N E L I Z E R  G A I N =====*/
/***************************************************************************//**
 * @brief This function is used to configure the NCO gain for one or more DAC
 * channels of the device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `channels` parameter
 * specifies which DAC channels to configure, and the `gain` parameter
 * sets the desired gain value. The function will return an error if the
 * `gain` exceeds the maximum allowed value of 0x0FFF. If any of the
 * specified channels are invalid or if the device pointer is null, the
 * function will handle these cases gracefully by returning an
 * appropriate error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param channels A bitmask indicating which DAC channels to configure. Each
 * bit corresponds to a channel, where `AD9081_DAC_CH_0`
 * represents the first channel.
 * @param gain A 16-bit unsigned integer representing the NCO gain to be set.
 * Valid values range from 0 to 0x0FFF. If the value exceeds this
 * range, the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_gain_set(adi_ad9081_device_t *device,
					uint8_t channels, uint16_t gain);

/*===== 2 . 4   T R A N S M I T  D A T A P A T H  S E T U P  =====*/
/***************************************************************************//**
 * @brief This function configures the interpolation settings for the Digital-
 * to-Analog Converter (DAC) in the specified device. It must be called
 * after the device has been properly initialized. The function takes two
 * interpolation parameters: `main_interp` for the main interpolation
 * setting and `ch_interp` for the channel interpolation setting. It is
 * important to ensure that the `device` pointer is valid and not null
 * before calling this function. If either interpolation value is out of
 * the expected range, the function will handle the error appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param main_interp An 8-bit unsigned integer representing the main
 * interpolation setting. Valid values depend on the device
 * specifications.
 * @param ch_interp An 8-bit unsigned integer representing the channel
 * interpolation setting. Valid values depend on the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * interpolation settings were successfully applied. If an error occurs
 * during the operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_interpolation_set(adi_ad9081_device_t *device,
					 uint8_t main_interp,
					 uint8_t ch_interp);

/***************************************************************************//**
 * @brief This function configures the crossbar settings for the specified
 * Digital-to-Analog Converters (DACs) in the device. It should be called
 * after the device has been properly initialized and configured. The
 * function takes a bitmask of DACs and a channel number, applying the
 * configuration to each selected DAC. If an invalid DAC bitmask is
 * provided, or if the device pointer is null, the function will return
 * an error. It is important to ensure that the channel number is valid
 * for the selected DACs, as improper values may lead to undefined
 * behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * defined by the `AD9081_DAC_ALL` constant. If the value exceeds
 * this constant, an error will be returned.
 * @param channel An integer representing the channel to be configured. The
 * valid range depends on the specific DAC configuration and
 * should be verified against the device's specifications.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration, or an error
 * code indicating the type of failure if the operation was
 * unsuccessful.
 ******************************************************************************/
int32_t adi_ad9081_dac_xbar_set(adi_ad9081_device_t *device, uint8_t dacs,
				uint8_t channel);

/***************************************************************************//**
 * @brief This function configures the XOR settings for the specified Digital-
 * to-Analog Converters (DACs) in the device. It must be called after the
 * device has been properly initialized. The `dacs` parameter specifies
 * which DACs to configure, while the `enable` parameter determines
 * whether to enable or disable the XOR functionality. If the `device`
 * pointer is null, the function will return an error. The function also
 * handles enabling dual SPI communication for the DACs, ensuring that
 * the settings are applied correctly. It is important to note that the
 * function may return an error if any of the underlying operations fail.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`. The function will ignore bits corresponding
 * to DACs that are not specified.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the XOR functionality for the specified DACs.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_data_xor_set(adi_ad9081_device_t *device, uint8_t dacs,
				    uint8_t enable);

/*===== 2 . 5   T R A N S M I T  P A T H  N C O S =====*/
/***************************************************************************//**
 * @brief This function configures the NCO frequency shift for specified DACs
 * and channels. It must be called after the device has been properly
 * initialized and configured. The function checks for null pointers and
 * validates that the DAC frequency is non-zero before proceeding. If the
 * specified DACs or channels are not valid, the function will not
 * perform any operations. The NCO frequency is calculated based on the
 * provided shift in Hertz and the device's DAC frequency.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs Specifies which DACs to configure. Valid values are defined by
 * the `AD9081_DAC_*` constants, with `AD9081_DAC_NONE` indicating
 * no DACs are selected.
 * @param channels Specifies which channels to configure. Valid values are
 * defined by the `AD9081_DAC_CH_*` constants, with
 * `AD9081_DAC_CH_NONE` indicating no channels are selected.
 * @param nco_shift_hz The desired NCO frequency shift in Hertz. This value can
 * be any 64-bit signed integer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_set(adi_ad9081_device_t *device, uint8_t dacs,
				   uint8_t channels, int64_t nco_shift_hz);

/***************************************************************************//**
 * @brief This function configures the Numerically Controlled Oscillator (NCO)
 * settings for the Digital-to-Analog Converter (DAC) associated with the
 * specified device. It should be called after the device has been
 * properly initialized. The function allows the user to set the
 * frequency shift, offset, and enable a test tone for specified DACs and
 * channels. If the provided `device` pointer is null, the function will
 * return an error. Additionally, if the specified DACs or channels are
 * not valid, the function will handle these cases gracefully by
 * returning appropriate error codes.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * defined by the `AD9081_DAC_*` constants.
 * @param channels A bitmask indicating which channels to configure. Valid
 * values are defined by the `AD9081_DAC_CH_*` constants.
 * @param shift_hz The frequency shift in Hertz to be applied by the NCO. This
 * value can be any valid 64-bit integer.
 * @param offset The offset value for the test tone, represented as a 16-bit
 * unsigned integer. This value should be within the range of 0 to
 * 65535.
 * @param test_tone_en A flag to enable or disable the test tone. A value of 1
 * enables the test tone, while 0 disables it.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_set(adi_ad9081_device_t *device, uint8_t dacs,
			       uint8_t channels, int64_t shift_hz,
			       uint16_t offset, uint8_t test_tone_en);

/***************************************************************************//**
 * @brief This function is used to enable or disable the Numerically Controlled
 * Oscillator (NCO) for specific Digital-to-Analog Converters (DACs) and
 * their associated channels in the AD9081 device. It must be called with
 * a valid `device` pointer that has been properly initialized. The
 * `dacs` parameter specifies which DACs to configure, while the
 * `channels` parameter indicates which channels to affect. The `enable`
 * parameter determines whether to turn the NCO on (non-zero value) or
 * off (zero value). If invalid parameters are provided, such as a null
 * `device` pointer or out-of-range `dacs`, the function will return an
 * error without making any changes.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Bitmask indicating which DACs to enable or disable. Valid values
 * are defined by the `AD9081_DAC_ALL` constant.
 * @param channels Bitmask indicating which channels to enable or disable. Valid
 * values are defined by the `AD9081_DAC_CH_0` constant.
 * @param enable Integer value indicating whether to enable (non-zero) or
 * disable (zero) the NCO.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if an operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_enable_set(adi_ad9081_device_t *device,
					  uint8_t dacs, uint8_t channels,
					  uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to reset the Numerically Controlled Oscillator
 * (NCO) for a specific channel as well as the main NCO in the device. It
 * should be called when there is a need to reinitialize the NCOs, such
 * as after a configuration change or to recover from an error state. The
 * function requires a valid `device` pointer, which must not be null. If
 * the `device` pointer is null, the function will return an error.
 * Additionally, the function will return an error if the underlying
 * hardware abstraction layer fails to set the reset values.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param chan_nco_reset A `uint8_t` value indicating whether to reset the
 * channel NCO. Valid values are 0 (do not reset) or 1
 * (reset).
 * @param main_nco_reset A `uint8_t` value indicating whether to reset the main
 * NCO. Valid values are 0 (do not reset) or 1 (reset).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_reset_set(adi_ad9081_device_t *device,
					 uint8_t chan_nco_reset,
					 uint8_t main_nco_reset);

/***************************************************************************//**
 * @brief This function configures the NCO phase offset for one or more Digital-
 * to-Analog Converters (DACs) and their respective channels. It must be
 * called with a valid `device` pointer that has been properly
 * initialized. The `dacs` parameter specifies which DACs to configure,
 * while `dac_phase_offset` sets the phase offset for those DACs.
 * Similarly, the `channels` parameter indicates which channels to
 * configure, and `ch_phase_offset` sets the phase offset for those
 * channels. If any of the parameters are invalid, the function will
 * return an error without making any changes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * defined by the `AD9081_DAC_ALL` constant. If the value exceeds
 * this constant, an error is returned.
 * @param dac_phase_offset The phase offset value for the selected DACs. The
 * valid range is determined by the device
 * specifications.
 * @param channels A bitmask indicating which channels to configure. Valid
 * values are defined by the `AD9081_DAC_CH_0` constant.
 * @param ch_phase_offset The phase offset value for the selected channels. The
 * valid range is determined by the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * parameter is invalid or if an operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_phase_offset_set(adi_ad9081_device_t *device,
						uint8_t dacs,
						uint16_t dac_phase_offset,
						uint8_t channels,
						uint16_t ch_phase_offset);

/***************************************************************************//**
 * @brief This function configures the NCO frequency tuning word (FTW) and the
 * accumulator parameters (modulus and delta) for specified Digital-to-
 * Analog Converters (DACs) and channels. It must be called after the
 * device has been properly initialized. The function checks that the sum
 * of the accumulator modulus and delta does not exceed a specified
 * limit, returning an error if this condition is violated. It is
 * important to ensure that the DACs and channels specified are valid and
 * that the device pointer is not null. The function may have side
 * effects on the device state, particularly in terms of the DAC and
 * channel selections.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Bitmask representing the DACs to configure. Valid values are
 * defined by the `AD9081_DAC_0` constants.
 * @param channels Bitmask representing the channels to configure. Valid values
 * are defined by the `AD9081_DAC_CH_0` constants.
 * @param ftw The frequency tuning word to set for the DACs. Must be a valid
 * 64-bit unsigned integer.
 * @param acc_modulus The accumulator modulus value. Must be a 64-bit unsigned
 * integer less than (1ULL << 48) when added to `acc_delta`.
 * @param acc_delta The accumulator delta value. Must be a 64-bit unsigned
 * integer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_ftw_set(adi_ad9081_device_t *device,
				       uint8_t dacs, uint8_t channels,
				       uint64_t ftw, uint64_t acc_modulus,
				       uint64_t acc_delta);
/***************************************************************************//**
 * @brief This function is used to obtain the frequency tuning word (FTW) and
 * accumulator parameters for a specified Digital-to-Analog Converter
 * (DAC) in the AD9081 device. It must be called after the device has
 * been properly initialized and configured. The function checks if the
 * specified DAC is valid and retrieves the FTW, as well as the
 * accumulator modulus and delta values if accumulator functionality is
 * enabled. If the accumulator is not enabled, the modulus and delta
 * values will be set to zero. It is important to ensure that the
 * `device` pointer is not null before calling this function.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Specifies which DAC to query. Valid values depend on the number
 * of DACs supported by the device.
 * @param ftw Pointer to a `uint64_t` where the frequency tuning word will be
 * stored. Caller retains ownership and must ensure it points to a
 * valid memory location.
 * @param acc_modulus Pointer to a `uint64_t` where the accumulator modulus will
 * be stored. Caller retains ownership and must ensure it
 * points to a valid memory location.
 * @param acc_delta Pointer to a `uint64_t` where the accumulator delta will be
 * stored. Caller retains ownership and must ensure it points
 * to a valid memory location.
 * @return Returns `API_CMS_ERROR_OK` on success. If any error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_nco_ftw_get(adi_ad9081_device_t *device,
					    uint8_t dacs, uint64_t *ftw,
					    uint64_t *acc_modulus,
					    uint64_t *acc_delta);

/***************************************************************************//**
 * @brief This function is used to obtain the frequency tuning word (FTW) and
 * accumulator settings for a specific DAC channel in the AD9081 device.
 * It must be called after the device has been properly initialized. The
 * function checks if the provided `device` pointer is valid and selects
 * the specified `channels`. If the accumulator modulus is enabled, it
 * retrieves the corresponding values for `acc_modulus` and `acc_delta`.
 * It is important to ensure that the `ftw`, `acc_modulus`, and
 * `acc_delta` pointers are valid and allocated by the caller, as the
 * function will write the retrieved values to these locations.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param channels Specifies the DAC channel to retrieve settings for. Valid
 * values depend on the device configuration.
 * @param ftw Pointer to a `uint64_t` where the frequency tuning word will be
 * stored. Must not be null.
 * @param acc_modulus Pointer to a `uint64_t` where the accumulator modulus will
 * be stored if enabled. Can be null if not needed.
 * @param acc_delta Pointer to a `uint64_t` where the accumulator delta will be
 * stored if enabled. Can be null if not needed.
 * @return Returns `API_CMS_ERROR_OK` on success. If any error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_channel_nco_ftw_get(adi_ad9081_device_t *device,
					       uint8_t channels, uint64_t *ftw,
					       uint64_t *acc_modulus,
					       uint64_t *acc_delta);

/*===== 2 . 5 . 1   T R A N S M I T  P A T H  F F H =====*/
/***************************************************************************//**
 * @brief This function configures the NCO hopf frequency tuning word for one or
 * more Digital-to-Analog Converters (DACs) in the specified device. It
 * must be called after the device has been properly initialized and
 * configured. The function expects a valid `hopf_index` between 1 and
 * 31, inclusive, and will return an error if the index is out of this
 * range. The `dacs` parameter allows selection of which DACs to
 * configure, and the function will apply the tuning word to each
 * selected DAC. If any parameter is invalid or if an error occurs during
 * the configuration process, the function will return an appropriate
 * error code.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Bitmask indicating which DACs to configure. Valid values are
 * determined by the defined DAC constants.
 * @param hopf_index Index for the hopf frequency tuning word, must be between 1
 * and 31. An out-of-range value will result in an error.
 * @param hopf_ftw The frequency tuning word to set for the hopf. Must be a
 * valid 32-bit unsigned integer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_nco_hopf_ftw_set(adi_ad9081_device_t *device,
						 uint8_t dacs,
						 uint8_t hopf_index,
						 uint32_t hopf_ftw);

/***************************************************************************//**
 * @brief This function configures the NCO Hopf mode for one or more Digital-to-
 * Analog Converters (DACs) associated with the specified device. It
 * should be called after the device has been properly initialized and
 * configured. The `dacs` parameter allows selection of which DACs to
 * configure, while the `hopf_mode` parameter specifies the desired mode
 * of operation. The function will return an error if the device pointer
 * is null or if any of the DAC selection or mode setting operations
 * fail. It is important to ensure that the `dacs` parameter correctly
 * represents the DACs to be configured, as invalid selections may lead
 * to unexpected behavior.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Bitmask representing the DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0` shifted left by 0 to 3. The
 * function will ignore any bits that do not correspond to valid
 * DACs.
 * @param hopf_mode An 8-bit value representing the Hopf mode. Valid values are
 * 0 (phase continuous switch), 1 (phase in-continuous switch),
 * or 2 (phase coherent switch).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during DAC selection or mode setting.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_nco_hopf_mode_set(adi_ad9081_device_t *device,
						  uint8_t dacs,
						  uint8_t hopf_mode);

/***************************************************************************//**
 * @brief This function configures the NCO HOPF selection for one or more
 * Digital-to-Analog Converters (DACs) associated with the specified
 * device. It must be called after the device has been properly
 * initialized. The `dacs` parameter allows selection of multiple DACs,
 * while the `hopf_index` specifies which HOPF to select, with valid
 * values ranging from 0 to 31. If the `device` pointer is null or if the
 * `hopf_index` is out of range, the function will return an error. The
 * function will attempt to set the HOPF selection for each DAC specified
 * in `dacs`, and any errors encountered during this process will also be
 * returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Each bit
 * corresponds to a DAC (e.g., bit 0 for DAC 0). Valid values are 0
 * to 15, where each bit can be set to indicate selection.
 * @param hopf_index An index for the HOPF selection, which must be in the range
 * of 0 to 31. If the value is greater than 31, the function
 * will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if an error occurs during the configuration
 * process.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_nco_hopf_select_set(adi_ad9081_device_t *device,
						    uint8_t dacs,
						    uint8_t hopf_index);

/***************************************************************************//**
 * @brief This function is used to configure the glitch-free operation of the
 * DAC NCO for specified DACs in the device. It must be called with a
 * valid `device` pointer that has been properly initialized. The `dacs`
 * parameter specifies which DACs to configure, and the `enable`
 * parameter determines whether to enable (1) or disable (0) the glitch-
 * free feature. If `enable` is set to a value other than 0 or 1, the
 * function will return an error. The function will iterate through the
 * specified DACs and apply the configuration, ensuring that the
 * operation is performed without glitches.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dacs Bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0` shifted left by 0 to 3.
 * @param enable Integer value to enable (1) or disable (0) glitch-free
 * operation. Must be either 0 or 1; otherwise, an error is
 * returned.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * configuration was applied without errors.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_nco_hopf_gpio_no_glitch_en_set(
	adi_ad9081_device_t *device, uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the GPIO settings related to the
 * DAC NCO Hop feature of the `adi_ad9081_device_t`. It must be called
 * after the device has been properly initialized. The `enable` parameter
 * determines whether the GPIO is enabled (1) or disabled (0). If
 * `enable` is set to 1, several internal registers are configured
 * accordingly. It is important to ensure that the `device` pointer is
 * not null and that `enable` is either 0 or 1; otherwise, the function
 * will return an error without making any changes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A uint8_t value that specifies whether to enable (1) or disable
 * (0) the GPIO. Valid values are 0 or 1; any other value will
 * result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or internal errors.
 ******************************************************************************/
int32_t
adi_ad9081_dac_duc_main_nco_hopf_gpio_as_hop_en_set(adi_ad9081_device_t *device,
						    uint8_t enable);

/*===== 2 . 6   T R A N S M I T  P A T H  P A  P R O T E C T I O N =====*/
/***************************************************************************//**
 * @brief This function is used to enable or disable the soft off gain feature
 * for one or more Digital-to-Analog Converters (DACs) in the specified
 * device. It should be called after the device has been properly
 * initialized. The `dacs` parameter allows selection of which DACs to
 * modify, and the `enable` parameter determines whether to enable or
 * disable the feature. If the `device` pointer is null, the function
 * will return an error. It is important to ensure that the specified
 * DACs are valid and that the device is in a state that allows this
 * operation.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to modify. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`. Invalid values will be ignored.
 * @param enable A boolean value where 1 enables the soft off gain and 0
 * disables it. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_off_gain_enable_set(adi_ad9081_device_t *device,
						uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the soft off gain feature
 * for one or more Digital-to-Analog Converters (DACs) in the specified
 * device. It should be called after the device has been properly
 * initialized. The `dacs` parameter allows selection of which DACs to
 * modify, and the `enable` parameter determines whether to enable or
 * disable the feature. If an invalid device pointer is provided, the
 * function will return an error. It is important to ensure that the
 * `dacs` parameter correctly represents the DACs intended for
 * modification, as the function will iterate through the specified DACs
 * and apply the changes accordingly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to modify. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`. Invalid values will be ignored.
 * @param enable A boolean value where 1 enables the soft off gain and 0
 * disables it. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_off_new_gain_enable_set(adi_ad9081_device_t *device,
						    uint8_t dacs,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the soft off gain ramp rate for one
 * or more Digital-to-Analog Converters (DACs) in the specified device.
 * It should be called after the device has been properly initialized and
 * configured. The function allows the user to specify which DACs to
 * configure using a bitmask, and the ramp rate is defined by the `rate`
 * parameter. If an invalid device pointer is provided, the function will
 * return an error. Additionally, if the DAC selection fails or if there
 * is an error setting the ramp rate, the function will also return an
 * error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`. Invalid values will be ignored.
 * @param rate An 8-bit value representing the ramp rate to be set. The valid
 * range for this parameter should be defined by the user, as it is
 * not specified in the function.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the device
 * pointer is null, DAC selection fails, or setting the ramp rate
 * encounters an error.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_off_gain_ramp_rate_set(adi_ad9081_device_t *device,
						   uint8_t dacs, uint8_t rate);

/***************************************************************************//**
 * @brief This function is used to control the soft off feature of the DACs in
 * the device. It should be called after the device has been properly
 * initialized. The `dacs` parameter specifies which DACs to modify, and
 * the `enable` parameter determines whether to enable or disable the
 * soft off feature. If an invalid pointer is provided for the `device`,
 * the function will return an error. Additionally, if any DAC selection
 * or register setting fails, the function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to modify. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param enable A 16-bit value that specifies whether to enable (non-zero) or
 * disable (zero) the soft off feature.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during DAC selection or register setting.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_off_enable_set(adi_ad9081_device_t *device,
					   uint8_t dacs, uint16_t enable);

/***************************************************************************//**
 * @brief This function is used to control the soft on feature of the DACs in
 * the device. It should be called after the device has been properly
 * initialized. The `dacs` parameter specifies which DACs to enable or
 * disable, and the `enable` parameter determines the state to set. If an
 * invalid pointer is provided for the `device`, the function will return
 * an error. The function will iterate through the specified DACs and
 * apply the enable state accordingly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to enable or disable. Valid
 * values are combinations of `AD9081_DAC_0`, `AD9081_DAC_1`,
 * `AD9081_DAC_2`, and `AD9081_DAC_3`.
 * @param enable A boolean value where 1 enables and 0 disables the soft on
 * feature for the specified DACs.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during execution.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_on_enable_set(adi_ad9081_device_t *device,
					  uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the long power amplifier (PA) settings for
 * one or more digital-to-analog converters (DACs) in the device. It must
 * be called with a valid `device` pointer that has been properly
 * initialized. The function allows enabling or disabling the long PA,
 * setting the averaging time, and defining the average threshold for the
 * specified DACs. If any of the parameters are invalid or if the device
 * pointer is null, the function will handle these cases gracefully,
 * returning an appropriate error code. It is important to ensure that
 * the `dacs` parameter correctly represents the DACs to be configured,
 * as the function iterates through the possible DACs to apply the
 * settings.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param enable A boolean value indicating whether to enable (1) or disable (0)
 * the long PA.
 * @param averaging_time An 8-bit value representing the averaging time. Valid
 * range is typically defined by the device
 * specifications.
 * @param average_threshold A 16-bit value that sets the threshold for
 * averaging. Must be within the valid range defined by
 * the device specifications.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_dac_long_pa_set(adi_ad9081_device_t *device, uint8_t dacs,
				   uint8_t enable, uint8_t averaging_time,
				   uint16_t average_threshold);

/***************************************************************************//**
 * @brief This function is used to obtain the long power amplifier (PA) power
 * settings for one or more digital-to-analog converters (DACs)
 * associated with the specified device. It should be called after the
 * device has been properly initialized and configured. The `dacs`
 * parameter allows the selection of which DACs to query, and the results
 * are written to the `power` output parameter. If the function
 * encounters any errors, such as null pointers or issues with DAC
 * selection, it will return an error code. It is important to ensure
 * that the `power` pointer is valid and points to a memory location that
 * can hold the result.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to query. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param power A pointer to a `uint16_t` variable where the retrieved power
 * setting will be stored. Must not be null.
 * @return Returns an error code indicating the success or failure of the
 * operation. On success, the `power` variable will contain the long PA
 * power setting for the selected DACs.
 ******************************************************************************/
int32_t adi_ad9081_dac_long_pa_power_get(adi_ad9081_device_t *device,
					 uint8_t dacs, uint16_t *power);

/***************************************************************************//**
 * @brief This function is used to configure the short power amplifier (PA)
 * settings for one or more digital-to-analog converters (DACs) in the
 * device. It must be called with a valid `device` pointer that has been
 * properly initialized. The function allows enabling or disabling the
 * short PA, setting the averaging time, and defining the average
 * threshold for the specified DACs. If any of the parameters are invalid
 * or if the device pointer is null, the function will handle the error
 * accordingly. It is important to ensure that the `dacs` parameter
 * correctly represents the DACs to be configured, as the function will
 * iterate through the possible DACs and apply the settings only to those
 * that are specified.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`. Invalid values will be ignored.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the short PA. Must be either 0 or 1.
 * @param averaging_time An 8-bit value representing the averaging time setting.
 * Must be within the valid range defined by the device
 * specifications.
 * @param average_threshold A 16-bit value representing the threshold for
 * averaging. Must be within the valid range defined by
 * the device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the settings
 * were applied successfully. If an error occurs during the
 * configuration process, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_short_pa_set(adi_ad9081_device_t *device, uint8_t dacs,
				    uint8_t enable, uint8_t averaging_time,
				    uint16_t average_threshold);

/***************************************************************************//**
 * @brief This function is used to obtain the short power amplifier power level
 * for one or more digital-to-analog converters (DACs) associated with
 * the specified device. It must be called with a valid `device` pointer
 * that has been properly initialized. The `dacs` parameter allows the
 * selection of which DACs to query, and it should be a bitwise
 * combination of DAC identifiers. The function will populate the `power`
 * output parameter with the retrieved power level. If any of the
 * provided parameters are invalid or if an error occurs during the
 * operation, the function will handle it gracefully and return an
 * appropriate error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitwise combination of DAC identifiers indicating which DACs to
 * query. Valid values are defined by the `AD9081_DAC_0` constant
 * and its shifts.
 * @param power A pointer to a `uint16_t` variable where the retrieved power
 * level will be stored. Caller retains ownership and must ensure
 * it points to a valid memory location.
 * @return Returns an error code indicating the success or failure of the
 * operation. On success, the `power` parameter will contain the short
 * PA power level for the selected DACs.
 ******************************************************************************/
int32_t adi_ad9081_dac_short_pa_power_get(adi_ad9081_device_t *device,
					  uint8_t dacs, uint16_t *power);

/***************************************************************************//**
 * @brief This function configures the DAC rotation mode for the specified
 * device. It must be called with a valid `device` pointer that has been
 * properly initialized. The `mode` parameter determines the behavior of
 * the DAC during rotation, specifically enabling or disabling automatic
 * transitions for JESD and data path. If the `device` pointer is null,
 * the function will return an error without making any changes. It is
 * important to ensure that the device is ready for configuration before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error is returned.
 * @param mode A `uint8_t` value that specifies the rotation mode. Valid values
 * depend on the specific configuration options defined for the
 * device. The function will handle invalid values by returning an
 * error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the rotation
 * mode has been set successfully. If an error occurs, a negative error
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_rotation_mode_set(adi_ad9081_device_t *device,
					 uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the GPIO settings of the device to enable or
 * disable the power amplifiers. It should be called after the device has
 * been properly initialized. The `enable` parameter determines whether
 * the power amplifiers are enabled (1) or disabled (0). If the `enable`
 * parameter is set to a value greater than 1, the function will return
 * an error. Additionally, if the `device` pointer is null, the function
 * will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable An 8-bit unsigned integer that indicates whether to enable (1)
 * or disable (0) the power amplifiers. Valid values are 0 or 1;
 * any value greater than 1 will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or other issues.
 ******************************************************************************/
int32_t adi_ad9081_dac_gpio_as_pa_en_set(adi_ad9081_device_t *device,
					 uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the Digital Signal
 * Amplifier (DSA) for one or more Digital-to-Analog Converters (DACs) in
 * the device. It should be called after the device has been properly
 * initialized. The `dacs` parameter specifies which DACs to modify, and
 * the `enable` parameter determines whether to enable (non-zero value)
 * or disable (zero value) the DSA. If an invalid pointer is provided for
 * the `device`, or if an error occurs while selecting a DAC or setting
 * the DSA configuration, the function will handle these cases
 * appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to enable or disable. Valid
 * values are combinations of `AD9081_DAC_0`, `AD9081_DAC_1`,
 * `AD9081_DAC_2`, and `AD9081_DAC_3`. Each bit corresponds to a
 * DAC.
 * @param enable A value indicating whether to enable (non-zero) or disable
 * (zero) the DSA. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs during
 * execution, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_dsa_enable_set(adi_ad9081_device_t *device,
					       uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the Digital Up Converter (DUC) settings for
 * specified Digital-to-Analog Converters (DACs) in the device. It should
 * be called after the device has been properly initialized and
 * configured. The function allows the user to set various parameters
 * such as the DUC code, cutover, boost, and gain for the selected DACs.
 * It is important to ensure that the `device` pointer is valid and not
 * null before calling this function. If any of the DACs specified are
 * invalid or if there are errors during the configuration process, the
 * function will handle these gracefully by returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param code A value representing the DUC code to be set. The valid range is
 * determined by the device specifications.
 * @param cutover A value representing the cutover setting. The valid range is
 * determined by the device specifications.
 * @param boost A value representing the boost setting. The valid range is
 * determined by the device specifications.
 * @param gain A value representing the gain setting. The valid range is
 * determined by the device specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_dsa_set(adi_ad9081_device_t *device,
					uint8_t dacs, uint8_t code,
					uint8_t cutover, uint8_t boost,
					uint16_t gain);

/*===== 2 . 7   T X  P A T H  H E L P E R  A P I =====*/
/***************************************************************************//**
 * @brief This function is used to compute the NCO (Numerically Controlled
 * Oscillator) frequency tuning word based on the specified frequency and
 * shift value. It must be called with a valid `device` pointer that has
 * been properly initialized. The function will return an error if the
 * `device` pointer is null or if the underlying calculation fails. The
 * computed tuning word and additional parameters are returned via the
 * provided pointers, which must not be null.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param freq The desired frequency for the NCO, specified as a 64-bit unsigned
 * integer. Valid values depend on the device specifications.
 * @param nco_shift A 64-bit signed integer representing the NCO shift value.
 * This value can be positive or negative.
 * @param ftw A pointer to a 64-bit unsigned integer where the calculated
 * frequency tuning word will be stored. Must not be null.
 * @param a A pointer to a 64-bit unsigned integer where an additional
 * calculated value will be stored. Must not be null.
 * @param b A pointer to a 64-bit unsigned integer where another additional
 * calculated value will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * calculation was performed successfully. If an error occurs, the
 * function will return an error code, and the output parameters may not
 * be modified.
 ******************************************************************************/
int32_t adi_ad9081_device_calc_nco_ftw(adi_ad9081_device_t *device,
				       uint64_t freq, int64_t nco_shift,
				       uint64_t *ftw, uint64_t *a, uint64_t *b);

/***************************************************************************//**
 * @brief This function is used to specify which Digital-to-Analog Converters
 * (DACs) should be selected for operation. It must be called after the
 * device has been properly initialized. The `dacs` parameter allows for
 * the selection of one or more DACs, and it is important to ensure that
 * the value provided is within the valid range. If an invalid value is
 * provided or if the `device` pointer is null, the function will handle
 * these cases appropriately by returning an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask representing the DACs to select. Valid values must not
 * exceed `AD9081_DAC_ALL`. If an invalid value is provided, the
 * function will return an error.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A successful operation will return a non-negative
 * value, while an error will be indicated by a negative return value.
 ******************************************************************************/
int32_t adi_ad9081_dac_select_set(adi_ad9081_device_t *device, uint8_t dacs);

/***************************************************************************//**
 * @brief This function is used to configure which Digital-to-Analog Converter
 * (DAC) channels are active for a given device. It must be called with a
 * valid device pointer that has been properly initialized. The
 * `channels` parameter specifies which DAC channels to select, and it
 * should be within the valid range defined by the constant
 * `AD9081_DAC_CH_ALL`. If the provided `channels` value exceeds this
 * range, the function will return an error. It is important to ensure
 * that the device pointer is not null before calling this function, as
 * it will handle null pointer checks internally.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param channels A uint8_t value representing the DAC channels to select.
 * Valid values are in the range from 0 to `AD9081_DAC_CH_ALL`.
 * If the value exceeds this range, the function will return an
 * error.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A successful operation will return a non-negative
 * value, while an error will be indicated by a negative return value.
 ******************************************************************************/
int32_t adi_ad9081_dac_chan_select_set(adi_ad9081_device_t *device,
				       uint8_t channels);

/***************************************************************************//**
 * @brief This function is used to configure the Digital-to-Analog Converter
 * (DAC) settings for the AD9081 device. It must be called after the
 * device has been properly initialized. The function takes two
 * parameters: the DAC selection and the channel selection, which
 * determine which DACs and channels are active. If either parameter is
 * invalid or if the device pointer is null, the function will handle
 * these cases gracefully by returning an error code. It is important to
 * ensure that the device pointer is not null before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A `uint8_t` value representing the DACs to be selected. Valid
 * values depend on the specific DAC configuration of the device.
 * @param channels A `uint8_t` value representing the channels to be selected.
 * Valid values depend on the specific channel configuration of
 * the device.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_select_set(adi_ad9081_device_t *device, uint8_t dacs,
				      uint8_t channels);

/***************************************************************************//**
 * @brief This function is used to enable or disable the DC test tone for one or
 * more Digital-to-Analog Converter (DAC) channels in the
 * `adi_ad9081_device_t`. It must be called with a valid device pointer
 * that has been properly initialized. The `channels` parameter specifies
 * which DAC channels to modify, and the `enable` parameter determines
 * whether to turn the test tone on (1) or off (0). If an invalid
 * `enable` value is provided (anything other than 0 or 1), the function
 * will return an error. The function will iterate through the specified
 * channels and apply the requested setting to each valid channel.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param channels A bitmask indicating which DAC channels to enable or disable
 * the test tone for. Valid values are combinations of
 * `AD9081_DAC_CH_0` shifted left by channel indices.
 * @param enable An integer that specifies whether to enable (1) or disable (0)
 * the test tone. Must be either 0 or 1; otherwise, an error is
 * returned.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors. If an error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_dc_test_tone_en_set(adi_ad9081_device_t *device,
					   uint8_t channels, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the DC offset for one or more
 * Digital-to-Analog Converter (DAC) channels in the
 * `adi_ad9081_device_t`. It should be called after the device has been
 * properly initialized. The `channels` parameter allows you to specify
 * which DAC channels to configure, and the `offset` parameter sets the
 * desired DC offset value. If the `device` pointer is null, the function
 * will return an error. Additionally, if any errors occur while
 * selecting the channel or setting the offset, the function will also
 * return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param channels A bitmask indicating which DAC channels to configure. Valid
 * values are combinations of `AD9081_DAC_CH_0` through
 * `AD9081_DAC_CH_7`. If no channels are selected, no action is
 * taken.
 * @param offset A 16-bit unsigned integer representing the DC offset value to
 * set. The valid range for this value is implementation-specific,
 * but it should be within the acceptable range for the DAC.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during channel selection or offset setting.
 ******************************************************************************/
int32_t adi_ad9081_dac_dc_test_tone_offset_set(adi_ad9081_device_t *device,
					       uint8_t channels,
					       uint16_t offset);

/***************************************************************************//**
 * @brief This function is used to control the DC test tone feature for one or
 * more Digital-to-Analog Converters (DACs) in the device. It must be
 * called with a valid `device` pointer that has been properly
 * initialized. The `dacs` parameter specifies which DACs to configure,
 * and the `enable` parameter determines whether to turn the test tone on
 * (non-zero value) or off (zero value). The function iterates through
 * the specified DACs and applies the setting to each one that is
 * selected. If any errors occur during the operation, they will be
 * handled internally, and the function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Valid values are
 * combinations of `AD9081_DAC_0`, `AD9081_DAC_1`, `AD9081_DAC_2`,
 * and `AD9081_DAC_3`.
 * @param enable A value indicating whether to enable (non-zero) or disable
 * (zero) the DC test tone. Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_main_dc_test_tone_en_set(adi_ad9081_device_t *device,
						    uint8_t dacs,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the DC test tone offset for one or
 * more Digital-to-Analog Converters (DACs) in the specified device. It
 * should be called after the device has been properly initialized and
 * configured. The function accepts a bitmask indicating which DACs to
 * configure, allowing for multiple DACs to be set simultaneously. If an
 * invalid pointer is provided for the device, or if an error occurs
 * while selecting a DAC or setting the offset, the function will handle
 * these cases gracefully by returning an error code. It is important to
 * ensure that the offset value is within the valid range expected by the
 * device.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param dacs A bitmask indicating which DACs to configure. Each bit
 * corresponds to a DAC (e.g., bit 0 for DAC 0). Valid values are 0
 * to 15, where each bit represents a DAC.
 * @param offset A 16-bit unsigned integer representing the DC test tone offset.
 * The valid range for this value depends on the specific DAC
 * configuration and should be checked against the device's
 * specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t
adi_ad9081_dac_duc_main_dc_test_tone_offset_set(adi_ad9081_device_t *device,
						uint8_t dacs, uint16_t offset);

/***************************************************************************//**
 * @brief This function is used to configure the skew adjustment for one or more
 * Digital-to-Analog Converter (DAC) channels in the AD9081 device. It
 * should be called after the device has been properly initialized. The
 * `channels` parameter allows the selection of specific DAC channels,
 * while the `skew` parameter defines the amount of skew adjustment to
 * apply. If the `device` pointer is null, the function will return an
 * error. Additionally, if any channel selection or skew setting fails,
 * the function will also return an error, ensuring that the caller is
 * informed of any issues during the operation.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param channels A bitmask representing the DAC channels to configure. Valid
 * values are combinations of `AD9081_DAC_CH_0` shifted left by
 * 0 to 7, allowing selection of multiple channels.
 * @param skew An 8-bit value representing the skew adjustment to apply. Valid
 * values depend on the specific requirements of the DAC channels.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_chan_skew_set(adi_ad9081_device_t *device,
					 uint8_t channels, uint8_t skew);

/*===== 3 . 0   R E C E I V E  P A T H  S E T U P =====*/
/***************************************************************************//**
 * @brief This function is used to initialize and start the RX device,
 * configuring various parameters such as DDC settings and JESD link
 * configurations. It must be called after the device has been properly
 * initialized. The function checks the provided DDC configurations and
 * adjusts the settings accordingly, ensuring that any DCM settings are
 * respected. If both coarse and fine DDCs are disabled, the function
 * will configure the device for full bandwidth mode. It is important to
 * handle any errors returned by the function, as they indicate issues
 * with the configuration or initialization process.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device to be started. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDCs to enable. Valid values
 * are 0 to 15, where each bit represents a DDC.
 * @param fddcs A bitmask indicating which fine DDCs to enable. Valid values are
 * 0 to 255, where each bit represents a DDC.
 * @param cddc_shift An array of 4 integers representing the shift values for
 * the coarse DDCs. Must not be null.
 * @param fddc_shift An array of 8 integers representing the shift values for
 * the fine DDCs. Must not be null.
 * @param cddc_dcm An array of 4 bytes indicating the DCM settings for the
 * coarse DDCs. Must not be null.
 * @param fddc_dcm An array of 8 bytes indicating the DCM settings for the fine
 * DDCs. Must not be null.
 * @param cc2r_en An array of 4 bytes indicating whether to enable CC2R for the
 * coarse DDCs. Must not be null.
 * @param fc2r_en An array of 8 bytes indicating whether to enable FC2R for the
 * fine DDCs. Must not be null.
 * @param jesd_param An array of 2 `adi_cms_jesd_param_t` structures containing
 * JESD parameters. Must not be null.
 * @param jesd_conv_sel An array of 2 `adi_ad9081_jtx_conv_sel_t` values
 * indicating the JESD conversion selection. Must not be
 * null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * startup, while any negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_startup_rx(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t fddcs,
	int64_t cddc_shift[4], int64_t fddc_shift[8], uint8_t cddc_dcm[4],
	uint8_t fddc_dcm[8], uint8_t cc2r_en[4], uint8_t fc2r_en[8],
	adi_cms_jesd_param_t jesd_param[2],
	adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2]);

/***************************************************************************//**
 * @brief This function configures the NCO mode for one or more coarse digital
 * down converters (DDCs) associated with the specified device. It should
 * be called after the device has been properly initialized and
 * configured. The `cddcs` parameter allows selection of which DDCs to
 * configure, while the `nco_mode` parameter specifies the desired NCO
 * mode. If an invalid NCO mode is provided, the function will return an
 * error. Additionally, if the `device` pointer is null, the function
 * will also return an error. It is important to ensure that the selected
 * DDCs are valid and that the device is in a state that allows for this
 * configuration.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to configure. Each bit
 * corresponds to a DDC, with valid values ranging from 0 to 15.
 * @param nco_mode The desired NCO mode, represented by
 * `adi_ad9081_adc_nco_mode_e`. Must be a valid mode; otherwise,
 * an error will be returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or device state.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_coarse_nco_mode_set(adi_ad9081_device_t *device,
				       uint8_t cddcs,
				       adi_ad9081_adc_nco_mode_e nco_mode);

/***************************************************************************//**
 * @brief This function configures the crossbar settings for the ADC and CDDC in
 * the specified device. It must be called after the device has been
 * properly initialized. The function expects valid crossbar values for
 * both the ADC to CDDC and CDDC to FDDC mappings. If any of the provided
 * parameters are invalid or if the device pointer is null, the function
 * will return an error. It is important to ensure that the device is not
 * null before calling this function to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_cddc_xbar A `uint8_t` value representing the ADC to CDDC crossbar
 * configuration. Valid values depend on the specific
 * device configuration.
 * @param cddc_fddc_xbar A `uint8_t` value representing the CDDC to FDDC
 * crossbar configuration. Valid values depend on the
 * specific device configuration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_xbar_set(adi_ad9081_device_t *device,
				uint8_t adc_cddc_xbar, uint8_t cddc_fddc_xbar);

/***************************************************************************//**
 * @brief This function configures the feedback selection for the JESD
 * transmitter on the specified device. It should be called after the
 * device has been properly initialized and configured. The `links`
 * parameter determines which JESD links are affected by this
 * configuration, and the `converters` parameter specifies the feedback
 * selection value. If the `device` pointer is null or if an invalid link
 * is specified, the function will return an error. It is important to
 * ensure that the appropriate links are selected based on the
 * application requirements.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`. If no valid links are set, the function
 * will not perform any configuration.
 * @param converters A 16-bit unsigned integer representing the feedback
 * selection value. The valid range depends on the specific
 * application and device configuration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_fbw_sel_set(adi_ad9081_device_t *device,
				       adi_ad9081_jesd_link_select_e links,
				       uint16_t converters);
/***************************************************************************//**
 * @brief This function is used to set the frame bandwidth configuration for the
 * JESD204B transmitter on the specified device. It must be called after
 * the device has been properly initialized and configured. The function
 * takes into account the selected links and the JESD204B parameters
 * provided in the `jesd_m` array. It is important to ensure that the
 * `jesd_m` values are within the expected ranges to avoid configuration
 * errors. If the device pointer is null or if any internal error occurs
 * during the configuration, the function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD204B links to configure. Valid values
 * are defined in the enumeration.
 * @param jesd_m An array of two `uint8_t` values representing the JESD204B
 * parameters for the configuration. Each value should be in the
 * range of 0 to 3 for proper operation.
 * @return Returns an integer error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * successful configuration.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_fbw_config_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t jesd_m[2]);

/***************************************************************************//**
 * @brief This function configures the Nyquist zone for a selected ADC channel
 * in the `adi_ad9081_device_t`. It must be called after the device has
 * been properly initialized. The function modifies the Nyquist zone
 * setting based on the provided `zone` parameter, which can be either
 * odd or even. It is important to ensure that the `device` pointer is
 * valid and not null before calling this function. If the specified
 * `zone` is invalid or if there are issues with the device
 * communication, the function will handle these errors appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_sel An enumeration value of type `adi_ad9081_adc_select_e` that
 * specifies which ADC channels to configure. Valid values depend
 * on the specific ADC configuration.
 * @param zone An enumeration value of type `adi_ad9081_adc_nyquist_zone_e` that
 * specifies the desired Nyquist zone. Valid values are
 * `AD9081_ADC_NYQUIST_ZONE_ODD` and `AD9081_ADC_NYQUIST_ZONE_EVEN`.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the Nyquist
 * zone has been set successfully. If an error occurs during the
 * operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_nyquist_zone_set(adi_ad9081_device_t *device,
					adi_ad9081_adc_select_e adc_sel,
					adi_ad9081_adc_nyquist_zone_e zone);

/***************************************************************************//**
 * @brief This function configures the fine gain setting for the ADC digital
 * down converter (DDC) of the specified device. It should be called
 * after the device has been properly initialized and configured. The
 * function allows setting the fine gain for multiple DDCs, as specified
 * by the `fddcs` parameter. If an invalid gain value is provided, or if
 * the device pointer is null, the function will handle these cases
 * gracefully by returning an error. It is important to ensure that the
 * `fddcs` parameter correctly represents the DDCs to be configured.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which DDCs to configure. Each bit
 * corresponds to a DDC, where a set bit indicates that the
 * corresponding DDC should be configured.
 * @param gain The fine gain value to set for the specified DDCs. Valid values
 * are 0 or 1. If the value is outside this range, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the fine gain
 * has been set successfully. If an error occurs during the operation,
 * an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_gain_set(adi_ad9081_device_t *device,
					 uint8_t fddcs, uint8_t gain);

/***************************************************************************//**
 * @brief This function configures the DC coupling data inversion setting for
 * the specified ADC channel in the device. It must be called after the
 * device has been properly initialized. The `enable` parameter
 * determines whether the data inversion is activated or deactivated for
 * the selected ADC channels specified by `adc_sel`. If `enable` is set
 * to a value greater than 1, the function will return an error. The
 * function also triggers a user-defined ADC calibration setting and
 * initiates a data transfer after updating the inversion setting.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_sel An enumeration value of type `adi_ad9081_adc_select_e` that
 * specifies which ADC channels to configure. Valid values depend
 * on the specific ADC channels available in the device.
 * @param enable A uint8_t value that indicates whether to enable (1) or disable
 * (0) the data inversion. Must be either 0 or 1; any other value
 * will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t
adi_ad9081_adc_data_inversion_dc_coupling_set(adi_ad9081_device_t *device,
					      adi_ad9081_adc_select_e adc_sel,
					      uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the offset timing calibration for specified
 * ADC channels in the device. It must be called after the device has
 * been properly initialized. The `adc_sel` parameter determines which
 * ADC channels are affected by the calibration setting, while the
 * `enable` parameter specifies whether to enable or disable the
 * calibration for those channels. If `enable` is set to a value greater
 * than 1, the function will return an error. The function also triggers
 * a data transfer to apply the changes. It is important to ensure that
 * the `device` pointer is not null before calling this function.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_sel A bitmask of type `adi_ad9081_adc_select_e` indicating which
 * ADC channels to configure. Valid values depend on the specific
 * ADC channels available in the device.
 * @param enable A uint8_t value that indicates whether to enable (1) or disable
 * (0) the offset timing calibration. Must be 0 or 1; any other
 * value will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_adc_offset_timing_calibration_set(adi_ad9081_device_t *device,
					     adi_ad9081_adc_select_e adc_sel,
					     uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the offset calibration settings for the
 * specified ADC channels in the device. It must be called after the
 * device has been properly initialized. The `enable` parameter
 * determines whether to enable or disable the offset calibration for the
 * selected ADC channels, specified by `adc_sel`. If `enable` is set to a
 * value greater than 1, the function will return an error. The function
 * also ensures that the necessary registers are updated to reflect the
 * new calibration settings, and it triggers a data transfer to apply the
 * changes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_sel An enumeration value of type `adi_ad9081_adc_select_e` that
 * specifies which ADC channels to configure. Valid values depend
 * on the specific ADC channels available in the device.
 * @param enable A uint8_t value that indicates whether to enable (1) or disable
 * (0) the offset calibration. Must be 0 or 1; values greater than
 * 1 will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_offset_calibration_set(adi_ad9081_device_t *device,
					      adi_ad9081_adc_select_e adc_sel,
					      uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the gain calibration settings for the
 * specified ADC channels in the device. It should be called after the
 * device has been properly initialized. The `adc_sel` parameter
 * determines which ADC channels are affected by the gain calibration
 * setting, while the `enable` parameter specifies whether to enable or
 * disable the calibration for those channels. If `enable` is set to a
 * value other than 0 or 1, the function will return an error. The
 * function also triggers a data transfer to apply the changes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_sel An enumeration value of type `adi_ad9081_adc_select_e` that
 * specifies which ADC channels to configure. Valid values depend
 * on the specific ADC channels available in the device.
 * @param enable A uint8_t value that indicates whether to enable (1) or disable
 * (0) gain calibration for the selected ADC channels. Must be
 * either 0 or 1; any other value will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_gain_calibration_set(adi_ad9081_device_t *device,
					    adi_ad9081_adc_select_e adc_sel,
					    uint8_t enable);

/*===== 3 . 1   R E C E I V E  D A T A P A T H  S E T U P =====*/
/***************************************************************************//**
 * @brief This function is used to configure the ADC settings of the AD9081
 * device. It should be called after the device has been properly
 * initialized. The function allows the user to set various parameters
 * such as channel digital down-converter settings, shifts, and
 * decimation settings. It is important to ensure that the provided
 * pointers are valid and that the arrays for shifts and decimation
 * settings are of the correct size. If any parameter is invalid, the
 * function will return an error code.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device to be configured. Must not be null.
 * @param cddcs Channel digital down-converter settings. Valid values depend on
 * the specific configuration of the ADC.
 * @param fddcs Frequency digital down-converter settings. Valid values depend
 * on the specific configuration of the ADC.
 * @param cddc_shift Array of 4 integers representing the shifts for the channel
 * digital down-converters. Must not be null and must have
 * exactly 4 elements.
 * @param fddc_shift Array of 8 integers representing the shifts for the
 * frequency digital down-converters. Must not be null and
 * must have exactly 8 elements.
 * @param cddc_dcm Array of 4 integers representing the decimation settings for
 * the channel digital down-converters. Must not be null and
 * must have exactly 4 elements.
 * @param fddc_dcm Array of 8 integers representing the decimation settings for
 * the frequency digital down-converters. Must not be null and
 * must have exactly 8 elements.
 * @param cc2r_en Array of 4 integers indicating whether channel-to-receiver is
 * enabled for each channel. Must not be null and must have
 * exactly 4 elements.
 * @param fc2r_en Array of 8 integers indicating whether frequency-to-receiver
 * is enabled for each frequency channel. Must not be null and
 * must have exactly 8 elements.
 * @return Returns an error code indicating the success or failure of the
 * configuration. A return value of `API_CMS_ERROR_OK` indicates
 * successful configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_config(adi_ad9081_device_t *device, uint8_t cddcs,
			      uint8_t fddcs, int64_t cddc_shift[4],
			      int64_t fddc_shift[8], uint8_t cddc_dcm[4],
			      uint8_t fddc_dcm[8], uint8_t cc2r_en[4],
			      uint8_t fc2r_en[8]);

/***************************************************************************//**
 * @brief This function is used to configure the ADC bypass mode for a specified
 * device. It should be called after the device has been properly
 * initialized. The function will return an error if the device pointer
 * is null or if the configuration fails. It is important to ensure that
 * the device is in a state that allows for configuration changes when
 * this function is called.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be configured. Must not be null. The function
 * will return an error if this parameter is invalid.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration, or an error
 * code if the configuration fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_bypass_config(adi_ad9081_device_t *device);

/*===== 3 . 2   R E C E I V E  F A S T  D E T E C T =====*/
/***************************************************************************//**
 * @brief This function configures the fault detection thresholds for specified
 * ADCs in the device. It should be called after the device has been
 * properly initialized and is ready for configuration. The function
 * allows setting both low and high thresholds, as well as dwell
 * thresholds for two ADCs. It is important to ensure that the `adcs`
 * parameter correctly specifies which ADCs to configure, as invalid
 * selections may lead to undefined behavior. The function will return an
 * error if the device pointer is null or if any of the internal
 * operations fail.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure (0-3). Each bit
 * corresponds to an ADC, where bit 0 is ADC0, bit 1 is ADC1, etc.
 * @param low_thresh The lower threshold for fault detection for the first ADC.
 * Valid range is implementation-specific.
 * @param up_thresh The upper threshold for fault detection for the first ADC.
 * Valid range is implementation-specific.
 * @param low_thresh2 The lower threshold for fault detection for the second
 * ADC. Valid range is implementation-specific.
 * @param up_thresh2 The upper threshold for fault detection for the second ADC.
 * Valid range is implementation-specific.
 * @param low_dwell_thresh The lower dwell threshold for the first ADC. Valid
 * range is implementation-specific.
 * @param low_dwell_thresh2 The lower dwell threshold for the second ADC. Valid
 * range is implementation-specific.
 * @param up_dwell_thresh The upper dwell threshold for the second ADC. Valid
 * range is implementation-specific.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd_thresh_set(adi_ad9081_device_t *device, uint8_t adcs,
				     uint16_t low_thresh, uint16_t up_thresh,
				     uint16_t low_thresh2, uint16_t up_thresh2,
				     uint16_t low_dwell_thresh,
				     uint16_t low_dwell_thresh2,
				     uint16_t up_dwell_thresh);

/***************************************************************************//**
 * @brief This function configures the force settings for the ADCs in the
 * device. It should be called after the device has been properly
 * initialized. The `adcs` parameter specifies which ADCs to configure,
 * while `enable` determines whether the ADCs should use the forced value
 * or operate normally. The `value` parameter sets the forced value to be
 * used when `enable` is set. If any of the parameters are invalid,
 * appropriate error handling will occur, and the function will return an
 * error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC.
 * @param enable A flag indicating whether to enable the forced value (1) or
 * allow normal operation (0). Valid values are 0 or 1.
 * @param value The value to be used when the forced mode is enabled. Must be
 * within the acceptable range defined by the device
 * specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd0_force_set(adi_ad9081_device_t *device, uint8_t adcs,
				     uint8_t enable, uint8_t value);

/***************************************************************************//**
 * @brief This function configures the force settings for the ADC FD1 of the
 * specified device. It should be called after the device has been
 * properly initialized. The function allows enabling or disabling the
 * forced value for the ADC, which can be useful for testing or specific
 * operational modes. It is important to ensure that the `adcs` parameter
 * correctly specifies which ADCs to configure, and that the `device`
 * pointer is valid and not null. If any of the parameters are invalid or
 * if an error occurs during the configuration, the function will return
 * an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC.
 * @param enable A value indicating whether to enable (1) or disable (0) the
 * forced value for the ADC. Must be either 0 or 1.
 * @param value The value to be forced to the ADC when enabled. This should be a
 * valid value as per the ADC's specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd1_force_set(adi_ad9081_device_t *device, uint8_t adcs,
				     uint8_t enable, uint8_t value);

/***************************************************************************//**
 * @brief This function is used to control the fast data output feature of the
 * ADCs in the device. It should be called after the device has been
 * properly initialized. The `adcs` parameter specifies which ADCs to
 * configure, and the `enable` parameter determines whether to enable or
 * disable the fast data output for the selected ADCs. If the `device`
 * pointer is null, the function will return an error. The function will
 * iterate through the specified ADCs and apply the configuration
 * accordingly, ensuring that the fast data output is only modified for
 * valid ADCs.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC0, 1 for
 * ADC1, etc.).
 * @param enable A boolean value where 0 disables and any non-zero value enables
 * the fast data output. This parameter is used to set the desired
 * state of the fast data output.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd_en_set(adi_ad9081_device_t *device, uint8_t adcs,
				 uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to control the fast data pins of the ADC by
 * enabling or disabling them based on GPIO input. It should be called
 * after initializing the `adi_ad9081_device_t` structure. The `adcs`
 * parameter specifies which ADCs to configure, and the `enable`
 * parameter determines whether to enable or disable the FD pins. If the
 * `device` pointer is null, the function will return an error. The
 * function iterates over the specified ADCs and applies the
 * configuration accordingly, ensuring that only valid ADCs are affected.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC (0-3). Valid values are 0 to 15.
 * @param enable A boolean value where 0 disables and any non-zero value enables
 * the FD pins. Valid values are 0 or 1.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd_en_via_gpio_set(adi_ad9081_device_t *device,
					  uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the ADC multiplexer selection for a given
 * device by specifying which source ADCs are to be routed to the
 * destination ADCs. It must be called with a valid `device` pointer that
 * has been properly initialized. The function iterates through the bits
 * of the `src_adc` parameter to determine which source ADCs are
 * selected, and for each selected source, it sets the corresponding
 * destination ADC using the `dst_adc` parameter. If any of the
 * parameters are invalid or if the device pointer is null, the function
 * will return an error. It is important to ensure that the device is in
 * a valid state before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param src_adc A bitmask indicating which source ADCs are selected. Valid
 * values are from 0 to 15, where each bit represents an ADC.
 * @param dst_adc A bitmask indicating which destination ADCs are to be
 * selected. Valid values are from 0 to 15, where each bit
 * represents an ADC.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd0_mux_sel_set(adi_ad9081_device_t *device,
				       uint8_t src_adc, uint8_t dst_adc);

/***************************************************************************//**
 * @brief This function configures the ADC multiplexing selection for a
 * specified device, allowing the user to route signals from one ADC to
 * another. It should be called after the device has been properly
 * initialized. The function takes two parameters: `src_adc`, which
 * specifies the source ADCs to be selected, and `dst_adc`, which
 * indicates the destination ADC. The function will return an error if
 * the `device` pointer is null or if any of the ADC selection operations
 * fail. It is important to ensure that the values provided for `src_adc`
 * and `dst_adc` are within the valid range to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param src_adc A bitmask indicating which source ADCs to select. Valid values
 * are from 0 to 15, where each bit represents an ADC. The
 * function will handle invalid values by returning an error.
 * @param dst_adc A bitmask indicating the destination ADC. Valid values are
 * from 0 to 15, where each bit represents an ADC. The function
 * will handle invalid values by returning an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the selection process.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd1_mux_sel_set(adi_ad9081_device_t *device,
				       uint8_t src_adc, uint8_t dst_adc);

/***************************************************************************//**
 * @brief This function configures the operational mode of the ADC FD0 for
 * specified ADCs in the device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `adcs` parameter
 * specifies which ADCs to configure, allowing for multiple ADCs to be
 * set in a single call. The `mode` parameter determines the operational
 * behavior of FD0, with specific modes utilizing different threshold
 * settings. If an invalid `device` pointer is provided, the function
 * will return an error. Additionally, if the specified ADCs are not
 * valid, the function will handle the error gracefully.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must be initialized before
 * use.
 * @param adcs A bitmask indicating which ADCs to configure (0-3). Each bit
 * corresponds to an ADC, where a bit value of 1 indicates that the
 * respective ADC should be configured.
 * @param mode An 8-bit value that specifies the function mode for FD0. Valid
 * values are 0 or 1, where 0 uses both upper and lower thresholds,
 * and 1 uses only the upper threshold.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd0_function_mode_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the operational mode of specified ADCs in the
 * device. It should be called after the device has been properly
 * initialized and is ready for configuration. The `adcs` parameter
 * allows selection of one or more ADCs, while the `mode` parameter
 * determines the specific operational mode for the selected ADCs. It is
 * important to ensure that the `device` pointer is valid and not null
 * before calling this function. If an invalid ADC selection is made or
 * if the device is not properly initialized, the function will handle
 * errors gracefully.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC1, 1 for
 * ADC2, etc.). If no bits are set, no ADCs will be configured.
 * @param mode An 8-bit value representing the operational mode for the selected
 * ADCs. Valid values are 0 or 1, where 0 uses a specific threshold
 * and 1 uses an alternative threshold.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd1_function_mode_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t mode);

/*===== 3 . 3   R E C E I V E  P R O G R A M M A B L E  F I L T E R =====*/
/***************************************************************************//**
 * @brief This function is used to clear the PFIR coefficients for the specified
 * control pages of an `adi_ad9081_device_t`. It must be called with a
 * valid device pointer that has been properly initialized. The
 * `ctl_pages` parameter determines which control pages will have their
 * coefficients cleared, and the `clear` parameter indicates whether to
 * perform the clearing operation. If an invalid device pointer is
 * provided, the function will return an error. It is important to ensure
 * that the device is in a state that allows for this operation before
 * calling the function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to clear. Valid values depend on the specific
 * implementation of the enumeration.
 * @param clear A uint8_t value indicating whether to clear the coefficients
 * (non-zero value) or not (zero value). Valid values are 0 or 1.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_clear_set(adi_ad9081_device_t *device,
				    adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				    uint8_t clear);

/***************************************************************************//**
 * @brief This function configures the quad mode for the ADC PFIR (Programmable
 * Finite Impulse Response) on specified control pages of the
 * `adi_ad9081_device_t`. It must be called with a valid device pointer
 * and appropriate control page flags. The `enable` parameter determines
 * whether to enable or disable the quad mode. If the `device` pointer is
 * null, the function will return an error. The function iterates over
 * the specified control pages and applies the configuration accordingly,
 * ensuring that any errors encountered during the process are handled
 * appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` indicating which control
 * pages to configure. Valid values depend on the specific
 * control pages defined in the enumeration.
 * @param enable A uint8_t value that specifies whether to enable (non-zero) or
 * disable (zero) the quad mode.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_quad_mode_set(adi_ad9081_device_t *device,
				  adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				  uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the PFIR I mode for the ADC in the specified
 * control pages. It must be called with a valid `device` pointer that
 * has been properly initialized. The `ctl_pages` parameter determines
 * which control pages will be affected, and the `i_mode` parameter
 * specifies the desired PFIR I mode. If the `device` pointer is null,
 * the function will return an error. It is important to ensure that the
 * control pages specified in `ctl_pages` are valid, as the function will
 * attempt to set the PFIR I mode for each specified page.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to configure. Valid values depend on the
 * specific implementation of the enumeration.
 * @param i_mode An enumeration value of type `adi_ad9081_adc_pfir_i_mode_e`
 * that specifies the PFIR I mode to set. Valid values depend on
 * the specific implementation of the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the PFIR I
 * mode has been set successfully. If an error occurs during the
 * operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_i_mode_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				       adi_ad9081_adc_pfir_i_mode_e i_mode);

/***************************************************************************//**
 * @brief This function configures the PFIR Q mode for the specified ADC control
 * pages of the `adi_ad9081_device_t`. It must be called with a valid
 * device pointer and appropriate control pages. The function iterates
 * over the specified control pages and applies the Q mode setting to
 * each active page. If the `device` pointer is null, the function will
 * return an error. It is important to ensure that the control pages
 * provided are valid and that the device has been properly initialized
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` indicating which control
 * pages to configure. Valid values depend on the specific
 * implementation of the enumeration.
 * @param q_mode An enumeration value of type `adi_ad9081_adc_pfir_q_mode_e`
 * representing the desired PFIR Q mode. Valid values depend on
 * the specific implementation of the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the failure reason if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_q_mode_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				       adi_ad9081_adc_pfir_q_mode_e q_mode);

/***************************************************************************//**
 * @brief This function configures the PFIR coefficient load selection for the
 * ADC in the specified device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `ctl_pages` parameter
 * determines which control pages to update, and the `sel` parameter
 * specifies the selection value for the coefficient load. If the
 * `device` pointer is null or if any errors occur during the operation,
 * the function will return an error code. It is important to ensure that
 * the control pages specified in `ctl_pages` are valid and that the
 * selection value is within the acceptable range.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to update. Valid values depend on the specific
 * implementation of the enumeration.
 * @param sel An 8-bit unsigned integer representing the selection value for the
 * coefficient load. The valid range is typically 0 to 255, but
 * specific valid values depend on the context of use.
 * @return Returns an integer error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_load_sel_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				       uint8_t sel);

/***************************************************************************//**
 * @brief This function is used to configure the PFIR (Polyphase Finite Impulse
 * Response) control page for the ADC in the `adi_ad9081_device_t`
 * device. It must be called after the device has been properly
 * initialized. If the provided `device` pointer is null, the function
 * will return an error. The function should be called with a valid
 * `page` value that corresponds to the desired PFIR control page. If an
 * error occurs during the register setting process, it will be returned
 * to the caller.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param page An enumeration value of type `adi_ad9081_adc_pfir_ctl_page_e`
 * that specifies the PFIR control page to set. The value must be
 * valid as per the defined enumeration.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, indicating that
 * the PFIR control page has been set without errors.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_ctl_page_set(adi_ad9081_device_t *device,
					 adi_ad9081_adc_pfir_ctl_page_e page);

/***************************************************************************//**
 * @brief This function is used to configure the PFIR coefficient transfer for
 * the specified ADC control pages of the `adi_ad9081_device_t`. It must
 * be called with a valid device pointer that has been properly
 * initialized. The `ctl_pages` parameter determines which control pages
 * will be affected, and the `enable` parameter specifies whether to
 * enable or disable the coefficient transfer. If the `device` pointer is
 * null, the function will return an error. The function will iterate
 * over the specified control pages and apply the settings accordingly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to configure. Valid values depend on the
 * defined enumeration.
 * @param enable A uint8_t value that indicates whether to enable (non-zero) or
 * disable (zero) the PFIR coefficient transfer.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_xfer_set(adi_ad9081_device_t *device,
				   adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				   uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the PFIR (Polyphase Finite Impulse Response)
 * gain settings for the ADC in the specified device. It should be called
 * after the device has been properly initialized and is ready for
 * configuration. The function allows for setting gains for two control
 * pages, and it will only apply changes for the pages specified in the
 * `ctl_pages` parameter. If an invalid `device` pointer is provided, the
 * function will return an error without making any changes. It is
 * important to ensure that the specified gains are within valid ranges
 * as defined by the API.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages A bitmask of control pages to configure. Valid values are
 * defined in the `adi_ad9081_adc_pfir_ctl_page_e` enumeration.
 * @param ix_gain The gain value for the X channel, specified as an enumeration
 * value from `adi_ad9081_adc_pfir_gain_e`. Must be within the
 * valid range defined by the enumeration.
 * @param iy_gain The gain value for the Y channel, specified as an enumeration
 * value from `adi_ad9081_adc_pfir_gain_e`. Must be within the
 * valid range defined by the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_i_gain_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				       adi_ad9081_adc_pfir_gain_e ix_gain,
				       adi_ad9081_adc_pfir_gain_e iy_gain);

/***************************************************************************//**
 * @brief This function configures the Q gain settings for the ADC's PFIR
 * (Polyphase Finite Impulse Response) filter. It should be called after
 * initializing the `device` and when the appropriate control pages are
 * selected. The function allows for setting gains for two channels,
 * specified by the `ctl_pages` parameter. If the specified control page
 * is not valid or if the `device` pointer is null, the function will
 * return an error. It is important to ensure that the gains provided are
 * within acceptable ranges as defined by the API.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to configure. Valid values depend on the
 * specific implementation.
 * @param qx_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the gain for the Qx channel. Must be a valid gain
 * value as defined by the API.
 * @param qy_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the gain for the Qy channel. Must be a valid gain
 * value as defined by the API.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_q_gain_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				       adi_ad9081_adc_pfir_gain_e qx_gain,
				       adi_ad9081_adc_pfir_gain_e qy_gain);

/***************************************************************************//**
 * @brief This function is used to configure the gain settings for the ADC's
 * PFIR (Post Filter Interpolation Rate) on a specified device. It must
 * be called with a valid `device` pointer that has been properly
 * initialized. The function sets both the I and Q gain values, which are
 * essential for accurate signal processing. If any of the gain
 * parameters are invalid or if the device pointer is null, the function
 * will handle these cases appropriately by returning an error. It is
 * important to ensure that the device is ready for configuration before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies the control
 * page to be used. Valid values are defined in the
 * enumeration.
 * @param ix_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the I channel gain. Valid values are defined in
 * the enumeration.
 * @param iy_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the I channel gain. Valid values are defined in
 * the enumeration.
 * @param qx_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the Q channel gain. Valid values are defined in
 * the enumeration.
 * @param qy_gain An enumeration value of type `adi_ad9081_adc_pfir_gain_e`
 * representing the Q channel gain. Valid values are defined in
 * the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or a null device pointer.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_gain_set(adi_ad9081_device_t *device,
				     adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				     adi_ad9081_adc_pfir_gain_e ix_gain,
				     adi_ad9081_adc_pfir_gain_e iy_gain,
				     adi_ad9081_adc_pfir_gain_e qx_gain,
				     adi_ad9081_adc_pfir_gain_e qy_gain);

/*===== 3 . 3 . 1   L O W  L E V E L  R E C E I V E  P R O G R A M M A B L E  F I L T E R  A P I  =====*/
/***************************************************************************//**
 * @brief This function is used to configure the PFIR (Polyphase Finite Impulse
 * Response) coefficient page for the ADC in the specified device. It
 * must be called after the device has been properly initialized. If the
 * `device` pointer is null, the function will return an error without
 * making any changes. The `page` parameter specifies which coefficient
 * page to set, and it should be a valid enumerated value from
 * `adi_ad9081_adc_pfir_coeff_page_e`. This function is essential for
 * ensuring that the ADC operates with the correct filter settings.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error is returned.
 * @param page An enumerated value of type `adi_ad9081_adc_pfir_coeff_page_e`
 * that specifies the PFIR coefficient page to set. The value must
 * be valid as defined in the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * coefficient page has been set successfully. If an error occurs, a
 * negative error code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_page_set(adi_ad9081_device_t *device,
				   adi_ad9081_adc_pfir_coeff_page_e page);

/***************************************************************************//**
 * @brief This function configures the digital input selection for the ADC PFIR
 * (Polyphase Finite Impulse Response) filter. It should be called after
 * initializing the `adi_ad9081_device_t` structure and before using the
 * ADC. The function takes control pages to determine which settings to
 * apply, and it modifies the input selection for both I and Q channels.
 * If the provided `device` pointer is null, the function will return an
 * error. Ensure that the `ctl_pages` parameter is valid and that the
 * selection indices for I and Q are within acceptable ranges.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param ctl_pages Control pages to select which settings to apply. Valid
 * values are defined in the `adi_ad9081_adc_pfir_ctl_page_e`
 * enumeration.
 * @param i_sel Selection index for the I channel. Must be within the valid
 * range for input selection.
 * @param q_sel Selection index for the Q channel. Must be within the valid
 * range for input selection.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_din_select_set(adi_ad9081_device_t *device,
				   adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				   uint8_t i_sel, uint8_t q_sel);

/***************************************************************************//**
 * @brief This function configures the output selection for the ADC PFIR
 * (Polyphase Finite Impulse Response) filter in the specified device. It
 * should be called after the device has been properly initialized and
 * configured. The function takes a bitmask of ADCs to select, allowing
 * multiple ADCs to be configured simultaneously. If an invalid device
 * pointer is provided, the function will return an error. Additionally,
 * if any of the ADC selection or output selection operations fail, the
 * function will return an error code, ensuring that the caller is aware
 * of any issues that arise during the configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates that the
 * respective ADC should be configured.
 * @param out_sel An 8-bit value representing the output selection for the PFIR.
 * Valid values depend on the specific output options available
 * for the device.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_dout_select_set(adi_ad9081_device_t *device,
					    uint8_t adcs, uint8_t out_sel);

/***************************************************************************//**
 * @brief This function configures the Polyphase Finite Impulse Response (PFIR)
 * mode for both the I and Q channels of the ADC in the specified device.
 * It must be called with a valid `device` pointer that has been properly
 * initialized. The function will return an error if the `device` pointer
 * is null or if there are issues setting the modes for either channel.
 * It is important to ensure that the control pages and modes provided
 * are valid and appropriate for the device's current configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies the control
 * pages to be used. Valid values depend on the device's
 * configuration.
 * @param i_mode An enumeration value of type `adi_ad9081_adc_pfir_i_mode_e`
 * that specifies the PFIR mode for the I channel. Valid values
 * are defined in the enumeration.
 * @param q_mode An enumeration value of type `adi_ad9081_adc_pfir_q_mode_e`
 * that specifies the PFIR mode for the Q channel. Valid values
 * are defined in the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the failure reason if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_mode_set(adi_ad9081_device_t *device,
				     adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				     adi_ad9081_adc_pfir_i_mode_e i_mode,
				     adi_ad9081_adc_pfir_q_mode_e q_mode);

/***************************************************************************//**
 * @brief This function configures the half-complex delay for the ADC PFIR of
 * the specified device. It should be called after the device has been
 * properly initialized and is ready for configuration. The function
 * accepts control pages to specify which PFIR settings to modify, and
 * applies the specified delay value. If the provided `device` pointer is
 * null, the function will return an error. Additionally, if any errors
 * occur during the setting of control pages or the delay, the function
 * will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to modify. Valid values depend on the defined
 * enumeration.
 * @param delay An 8-bit unsigned integer representing the delay setting. The
 * valid range is typically from 0 to 255, but specific valid
 * values should be confirmed in the device documentation.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while any negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_half_complex_delay_set(
	adi_ad9081_device_t *device, adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
	uint8_t delay);

/***************************************************************************//**
 * @brief This function configures the high cutoff program delay for the
 * specified ADC PFIR coefficient pages of the `adi_ad9081_device_t`. It
 * must be called with a valid device pointer and a valid coefficient
 * page selection. The `delay` parameter specifies the delay value to be
 * set. If the `device` pointer is null, the function will return an
 * error. The function iterates through the specified coefficient pages
 * and applies the delay setting to each active page. It is important to
 * ensure that the coefficient pages provided are valid, as invalid
 * selections may lead to errors during execution.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param coeff_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_coeff_page_e` that specifies which
 * coefficient pages to configure. Valid values are
 * determined by the enumeration definition.
 * @param delay An 8-bit unsigned integer representing the delay value to be
 * set. The valid range is implementation-specific, but typically
 * should be within 0 to 255. The function will handle out-of-range
 * values by returning an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_hc_prog_delay_set(
	adi_ad9081_device_t *device,
	adi_ad9081_adc_pfir_coeff_page_e coeff_pages, uint8_t delay);

/***************************************************************************//**
 * @brief This function configures the Variable Length Encoding (VLE)
 * coefficient for the ADC PFIR (Post Filter Interpolation Rate) in the
 * specified control pages of the device. It must be called with a valid
 * `device` pointer and appropriate control pages specified. The function
 * iterates over the control pages and applies the VLE coefficient
 * setting for each active page. If the `device` pointer is null or if
 * any internal operation fails, the function will return an error code.
 * It is important to ensure that the device is properly initialized
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` indicating which control
 * pages to configure. Valid values depend on the specific
 * implementation of the enumeration.
 * @param vle An 8-bit unsigned integer representing the VLE coefficient to be
 * set. The valid range is 0 to 255.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_vle_set(adi_ad9081_device_t *device,
				  adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
				  uint8_t vle);

/***************************************************************************//**
 * @brief This function is used to configure the coefficient page selection for
 * the ADC PFIR in the specified device. It must be called with a valid
 * `device` pointer that has been properly initialized. The `ctl_pages`
 * parameter determines which control pages are affected, and the `sel`
 * parameter specifies the selection value for the coefficient page. If
 * the `device` pointer is null or if any of the operations fail,
 * appropriate error handling will occur. It is important to ensure that
 * the function is called in the correct context where the device is
 * ready for configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param ctl_pages An enumeration value of type
 * `adi_ad9081_adc_pfir_ctl_page_e` that specifies which
 * control pages to set. Valid values depend on the defined
 * enumeration.
 * @param sel An 8-bit unsigned integer representing the selection value for the
 * coefficient page. The valid range is 0 to 255.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * coefficient page selection was set successfully. If an error occurs
 * during the operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_rd_coeff_page_sel_set(
	adi_ad9081_device_t *device, adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
	uint8_t sel);

/***************************************************************************//**
 * @brief This function is used to validate the PFIR coefficients for the ADC
 * based on the number of taps specified. It should be called after
 * ensuring that the `device` is properly initialized and before using
 * the coefficients in further processing. The function checks for the
 * presence and arrangement of 16-bit and 12-bit coefficients within the
 * provided array, ensuring that they conform to the expected format
 * based on the number of taps. If the validation fails, an error code is
 * returned, indicating the nature of the failure.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the ADC device. Must not be null.
 * @param ntaps An 8-bit unsigned integer representing the number of taps, which
 * must be one of the following values: 48, 64, 96, or 192. Invalid
 * values will result in an error.
 * @param coeffs An array of 192 16-bit unsigned integers representing the
 * coefficients to be validated. The caller retains ownership of
 * this array, and it must not be null.
 * @return Returns an integer indicating the result of the validation:
 * `API_CMS_ERROR_OK` if the coefficients are valid, or an error code
 * indicating the type of validation failure.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_coeff_validate(adi_ad9081_device_t *device,
					   uint8_t ntaps, uint16_t coeffs[192]);

/***************************************************************************//**
 * @brief This function is used to configure the PFIR (Polyphase Finite Impulse
 * Response) coefficients for the ADC in the specified device. It must be
 * called after the device has been properly initialized and configured.
 * The function takes a coefficient page and an index to specify which
 * coefficient to set. It is important to ensure that the index is within
 * the valid range for the specified coefficient page. If any of the
 * parameters are invalid or if there is an error during the register
 * write operations, the function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param coeff_pages An enumerator of type `adi_ad9081_adc_pfir_coeff_page_e`
 * that specifies the coefficient page to be used. Valid
 * values are defined in the enumeration.
 * @param index An 8-bit unsigned integer representing the index of the
 * coefficient to set. Must be within the valid range for the
 * specified coefficient page.
 * @param coeff A 16-bit unsigned integer representing the coefficient value to
 * set. This value will be split into two 8-bit values for storage.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation was not successful.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeff_set(adi_ad9081_device_t *device,
			      adi_ad9081_adc_pfir_coeff_page_e coeff_pages,
			      uint8_t index, uint16_t coeff);

/***************************************************************************//**
 * @brief This function is used to configure the programmable finite impulse
 * response (PFIR) coefficients for the ADC in the specified device. It
 * must be called after the device has been properly initialized and
 * before any data processing occurs. The function expects a valid
 * pointer to the device structure, a specified coefficient page, and an
 * array of coefficients. If any of the parameters are invalid,
 * appropriate error handling will occur, and the function will return an
 * error code. It is important to ensure that the coefficients array
 * contains at least 192 elements, as this is the expected size for the
 * PFIR coefficients.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param coeff_pages An enumerated value of type
 * `adi_ad9081_adc_pfir_coeff_page_e` that specifies which
 * coefficient page to set. Valid values are defined in the
 * enumeration.
 * @param coeffs A pointer to an array of 16-bit unsigned integers representing
 * the PFIR coefficients. The array must contain at least 192
 * elements. Must not be null.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates that the
 * coefficients were successfully set.
 ******************************************************************************/
int32_t
adi_ad9081_adc_pfir_coeffs_set(adi_ad9081_device_t *device,
			       adi_ad9081_adc_pfir_coeff_page_e coeff_pages,
			       uint16_t *coeffs);

/***************************************************************************//**
 * @brief This function is used to configure the programmable finite impulse
 * response (PFIR) settings for the ADC in the device. It must be called
 * after the device has been properly initialized. The function sets
 * various parameters including the control pages, coefficient pages,
 * input and output modes, gains, and the coefficients themselves. It is
 * important to ensure that the `coeffs` array is of the correct size as
 * specified by `coeffs_size`. If any parameter is invalid or if the
 * device pointer is null, the function will return an error code. The
 * function also handles the loading and transferring of coefficients,
 * ensuring that the settings are applied correctly.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param ctl_pages Control page selection for the PFIR configuration. Valid
 * values are defined in `adi_ad9081_adc_pfir_ctl_page_e`.
 * @param coeff_pages Coefficient page selection for the PFIR configuration.
 * Valid values are defined in
 * `adi_ad9081_adc_pfir_coeff_page_e`.
 * @param i_mode Input mode for the I channel. Valid values are defined in
 * `adi_ad9081_adc_pfir_i_mode_e`.
 * @param q_mode Input mode for the Q channel. Valid values are defined in
 * `adi_ad9081_adc_pfir_q_mode_e`.
 * @param ix_gain Gain setting for the I channel's X component. Valid values are
 * defined in `adi_ad9081_adc_pfir_gain_e`.
 * @param iy_gain Gain setting for the I channel's Y component. Valid values are
 * defined in `adi_ad9081_adc_pfir_gain_e`.
 * @param qx_gain Gain setting for the Q channel's X component. Valid values are
 * defined in `adi_ad9081_adc_pfir_gain_e`.
 * @param qy_gain Gain setting for the Q channel's Y component. Valid values are
 * defined in `adi_ad9081_adc_pfir_gain_e`.
 * @param coeff_load_sel Selector for loading coefficients. Must be a valid
 * value.
 * @param coeffs Pointer to an array of coefficients to be set. Must not be null
 * and should have a size defined by `coeffs_size`.
 * @param coeffs_size The number of coefficients to be set. Must be greater than
 * zero.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_pfir_config_set(
	adi_ad9081_device_t *device, adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
	adi_ad9081_adc_pfir_coeff_page_e coeff_pages,
	adi_ad9081_adc_pfir_i_mode_e i_mode,
	adi_ad9081_adc_pfir_q_mode_e q_mode, adi_ad9081_adc_pfir_gain_e ix_gain,
	adi_ad9081_adc_pfir_gain_e iy_gain, adi_ad9081_adc_pfir_gain_e qx_gain,
	adi_ad9081_adc_pfir_gain_e qy_gain, uint8_t coeff_load_sel,
	uint16_t *coeffs, uint8_t coeffs_size);

/*===== 3 . 4   R E C E I V E  P A T H  N C O S =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the frequency tuning word and modulus
 * values for the coarse NCO of a specified DDC. It must be called after
 * the device has been properly initialized and configured. The function
 * expects valid pointers for the output parameters, which will be
 * populated with the retrieved values. If any of the provided pointers
 * are null, the function will return an error. Additionally, if the
 * specified `cddc` is invalid or if any internal operations fail, the
 * function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddc An 8-bit unsigned integer representing the coarse DDC index.
 * Valid values depend on the specific device configuration.
 * @param ftw A pointer to a 64-bit unsigned integer where the frequency tuning
 * word will be stored. Must not be null.
 * @param modulus_a A pointer to a 64-bit unsigned integer where the first
 * modulus value will be stored. Must not be null.
 * @param modulus_b A pointer to a 64-bit unsigned integer where the second
 * modulus value will be stored. Must not be null.
 * @return Returns an error code indicating the success or failure of the
 * operation. On success, the output parameters `ftw`, `modulus_a`, and
 * `modulus_b` are populated with the corresponding values.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_ftw_get(adi_ad9081_device_t *device,
					      uint8_t cddc, uint64_t *ftw,
					      uint64_t *modulus_a,
					      uint64_t *modulus_b);

/***************************************************************************//**
 * @brief This function configures the coarse NCO phase offset for a specified
 * ADC DDC channel. It must be called after initializing the
 * `adi_ad9081_device_t` structure and before using the DDC
 * functionality. The function expects a valid device pointer and a
 * channel identifier. If the provided device pointer is null, the
 * function will return an error. Additionally, the function will select
 * the specified DDC channel before setting the phase offset. It is
 * important to ensure that the offset value is within the valid range
 * for the hardware, as invalid values may lead to undefined behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A `uint8_t` value representing the DDC channel to configure.
 * Valid values depend on the specific hardware configuration.
 * @param offset A `uint64_t` value representing the phase offset to set. The
 * valid range for this value is determined by the hardware
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_coarse_nco_phase_offset_set(adi_ad9081_device_t *device,
					       uint8_t cddcs, uint64_t offset);

/***************************************************************************//**
 * @brief This function is used to enable or disable the coarse Digital Down
 * Converter (DDC) Numerically Controlled Oscillator (NCO) in the
 * specified device. It must be called with a valid device pointer that
 * has been properly initialized. The `cddcs` parameter determines the
 * state of the coarse DDC NCO, where valid values should be specified
 * according to the device's requirements. If the device pointer is null,
 * the function will return an error. It is important to check the return
 * value to ensure that the operation was successful.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A `uint8_t` value that specifies the enable/disable state of the
 * coarse DDC NCO. Valid values depend on the device's
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_enable_set(adi_ad9081_device_t *device,
						 uint8_t cddcs);

/***************************************************************************//**
 * @brief This function is used to configure the coarse NCO channel selection
 * for the ADC DDC in the specified device. It should be called after the
 * device has been properly initialized. The function iterates through
 * the coarse DDC channels specified by `cddcs`, and for each active
 * channel, it sets the corresponding NCO channel selection to the
 * specified `channel`. If the `device` pointer is null, the function
 * will return an error. It is important to ensure that the values for
 * `cddcs` and `channel` are within valid ranges to avoid unexpected
 * behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDC channels to configure.
 * Valid values are determined by the defined constants for coarse
 * DDC channels.
 * @param channel An 8-bit value representing the channel selection for the NCO.
 * Valid values are expected to be in the range of 0 to 15.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_channel_selection_set(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t channel);

/***************************************************************************//**
 * @brief This function configures the coarse NCO channel selection for the ADC
 * device using GPIO pins. It should be called after the device has been
 * properly initialized. The function iterates through the specified
 * channels and applies the selection based on the provided `cddcs`
 * parameter. The `mode` parameter determines the control method for the
 * selected channels. If any of the input parameters are invalid, the
 * function will return an error code, and no changes will be made to the
 * device configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the ADC device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDC channels to select. Valid
 * values are defined by the `AD9081_ADC_CDDC_0` constants.
 * @param mode An 8-bit value that specifies the mode of operation for the
 * selected channels. Valid values depend on the specific control
 * requirements of the device.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_channel_select_via_gpio_set(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the update index for a coarse NCO
 * channel in the AD9081 device. It should be called after the device has
 * been properly initialized and configured. The function iterates
 * through the coarse digital downconverter (DDC) channels specified by
 * the `cddcs` parameter, applying the update index defined by the
 * `channel` parameter. If the `device` pointer is null, the function
 * will return an error. It is important to ensure that the `cddcs`
 * parameter correctly represents the active DDC channels, as only those
 * will be updated.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDC channels to update. Valid
 * values are determined by the defined constants for the DDC
 * channels.
 * @param channel The update index to set for the specified channels. This value
 * should be within the valid range defined by the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_channel_update_index_set(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t channel);

/***************************************************************************//**
 * @brief This function is used to configure the fine Digital Down Converter
 * (DDC) selection for the ADC in the specified device. It must be called
 * with a valid `device` pointer that has been properly initialized. The
 * `fddcs` parameter specifies the fine DDC selection and must be within
 * the valid range defined by the `AD9081_ADC_FDDC_ALL` constant. If the
 * provided `fddcs` value is invalid or if the `device` pointer is null,
 * the function will return an error. This function is essential for
 * configuring the ADC's signal processing capabilities.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs An 8-bit unsigned integer representing the fine DDC selection.
 * Valid values must not exceed `AD9081_ADC_FDDC_ALL`. If an
 * invalid value is provided, the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the fine DDC
 * selection has been set successfully. If an error occurs, a negative
 * error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_select_set(adi_ad9081_device_t *device,
					   uint8_t fddcs);

/***************************************************************************//**
 * @brief This function is used to reset the fine DDC for the specified ADC
 * channels in the `adi_ad9081_device_t` device. It should be called when
 * a reset of the fine DDC is required, typically during initialization
 * or reconfiguration of the device. The function expects a valid device
 * pointer and a bitmask indicating which DDCs to reset. If the provided
 * device pointer is null, the function will return an error. The
 * function performs a series of operations for each DDC specified in the
 * bitmask, including selecting the DDC, setting a soft reset, waiting
 * for a brief period, and then clearing the reset. It is important to
 * ensure that the device is properly initialized before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which fine DDCs to reset. Each bit
 * corresponds to a DDC channel, where a value of 1 indicates that
 * the respective DDC should be reset.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, indicating that
 * the reset operation was completed without errors.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_reset_set(adi_ad9081_device_t *device,
					  uint8_t fddcs);

/***************************************************************************//**
 * @brief This function is used to enable or disable fine synchronization for
 * specific digital down converters (DDCs) in an AD9081 device. It should
 * be called after the device has been properly initialized. The `fddcs`
 * parameter allows the selection of which DDCs to configure, and the
 * `enable` parameter determines whether to enable or disable
 * synchronization. If the `device` pointer is null, the function will
 * return an error. It is important to ensure that the `fddcs` parameter
 * correctly represents the DDCs intended for configuration, as invalid
 * values may lead to unexpected behavior.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param fddcs Bitmask representing the DDCs to configure. Each bit corresponds
 * to a DDC (0-7). Valid values are 0 to 255.
 * @param enable Boolean value indicating whether to enable (non-zero) or
 * disable (zero) fine synchronization.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during execution.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_sync_enable_set(adi_ad9081_device_t *device,
						uint8_t fddcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the coarse Numerically Controlled Oscillator
 * (NCO) frequency for the ADC Digital Down Converter (DDC) based on the
 * specified shift in Hertz. It must be called with a valid `device`
 * pointer that has been properly initialized. The function calculates
 * the frequency tuning word (FTW) based on the current ADC frequency and
 * the desired shift. If the input parameters are invalid, appropriate
 * error codes will be returned, and no changes will be made to the
 * device configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param cddcs An 8-bit unsigned integer representing the coarse DDC channel
 * selection. Valid values depend on the specific device
 * configuration.
 * @param cddc_shift_hz A 64-bit signed integer representing the desired
 * frequency shift in Hertz. This value can be positive or
 * negative, allowing for frequency adjustments in both
 * directions.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_set(adi_ad9081_device_t *device,
					  uint8_t cddcs, int64_t cddc_shift_hz);

/***************************************************************************//**
 * @brief This function configures the coarse NCO frequency tuning word and
 * modulus values for the specified ADC DDC. It must be called after
 * initializing the device and before using the DDC functionality. The
 * function validates the input parameters to ensure they are within
 * acceptable ranges, specifically that the upper 16 bits of the `ftw`,
 * `modulus_a`, and `modulus_b` parameters are zero. If any of these
 * parameters are invalid, the function will return an error.
 * Additionally, it sets the phase offset for the specified coarse DDC
 * channels based on the `cddcs` parameter, which indicates which
 * channels to configure.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDC channels to configure.
 * Valid values are defined by the `AD9081_ADC_CDDC_*` constants.
 * @param ftw The frequency tuning word to set. Must be a 48-bit value, meaning
 * the upper 16 bits must be zero.
 * @param modulus_a The modulus value for the first channel. Must be a 48-bit
 * value, meaning the upper 16 bits must be zero.
 * @param modulus_b The modulus value for the second channel. Must be a 48-bit
 * value, meaning the upper 16 bits must be zero.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_ftw_set(adi_ad9081_device_t *device,
					      uint8_t cddcs, uint64_t ftw,
					      uint64_t modulus_a,
					      uint64_t modulus_b);

/***************************************************************************//**
 * @brief This function is used to configure the fine NCO (Numerically
 * Controlled Oscillator) channel selection for the ADC (Analog-to-
 * Digital Converter) by utilizing GPIO (General Purpose Input/Output)
 * pins. It should be called after ensuring that the `device` has been
 * properly initialized. The function iterates through the specified FDDC
 * (Fine Digital Down Converter) channels, and for each active channel,
 * it sets the corresponding NCO channel selection mode. It is important
 * to note that if the `device` pointer is null, the function will return
 * an error. Additionally, if any internal operation fails, the function
 * will also return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which FDDC channels are to be selected.
 * Valid values are from 0 to 255, where each bit represents a
 * channel.
 * @param mode An 8-bit value representing the mode of operation for the
 * selected channels. Valid values depend on the specific mode
 * configurations defined in the API.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_channel_select_via_gpio_set(
	adi_ad9081_device_t *device, uint8_t fddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the update index for the fine
 * Numerically Controlled Oscillator (NCO) of the specified ADC Digital
 * Down Converter (DDC) channels. It should be called after initializing
 * the `device` and before using the DDC channels. The function processes
 * the `fddcs` parameter to determine which channels to update, and it
 * will only affect those channels that are enabled. If the `device`
 * pointer is null, the function will return an error. Additionally, if
 * any internal operations fail, the function will also return an error,
 * ensuring that the caller is aware of any issues that arise during the
 * update process.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which ADC DDC channels to update. Each bit
 * corresponds to a channel, where a set bit indicates that the
 * channel is enabled.
 * @param channel An index representing the update index to set for the
 * specified channels. Valid values depend on the specific
 * implementation and should be within the expected range for the
 * DDC configuration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_channel_update_index_set(
	adi_ad9081_device_t *device, uint8_t fddcs, uint8_t channel);

/***************************************************************************//**
 * @brief This function configures the fine NCO mode for the specified ADC
 * digital down converters (DDCs) in the device. It should be called
 * after the device has been properly initialized and configured. The
 * `fddcs` parameter allows selection of which DDCs to configure, while
 * the `nco_mode` parameter specifies the desired NCO mode. It is
 * important to ensure that the `nco_mode` is valid; otherwise, the
 * function will return an error. If the `device` pointer is null or if
 * any errors occur during the configuration process, appropriate error
 * handling will be triggered.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param fddcs Bitmask indicating which DDCs to configure. Each bit corresponds
 * to a DDC, where a set bit indicates that the respective DDC
 * should be configured.
 * @param nco_mode The desired NCO mode, represented by
 * `adi_ad9081_adc_nco_mode_e`. Must be a valid mode; otherwise,
 * an error will be returned.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration, or an error
 * code if an error occurs during the process.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_fine_nco_mode_set(adi_ad9081_device_t *device, uint8_t fddcs,
				     adi_ad9081_adc_nco_mode_e nco_mode);

/***************************************************************************//**
 * @brief This function is used to enable or disable the fine Digital Down
 * Converter (DDC) Numerically Controlled Oscillator (NCO) in the
 * specified device. It must be called with a valid `device` pointer that
 * has been properly initialized. The `fddcs` parameter determines the
 * state of the fine DDC NCO, where valid values should be specified
 * according to the device's requirements. If the `device` pointer is
 * null, the function will return an error without making any changes. It
 * is important to check the return value to ensure that the operation
 * was successful.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A `uint8_t` value that specifies the enable/disable state of the
 * fine DDC NCO. Valid values depend on the device's
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_enable_set(adi_ad9081_device_t *device,
					       uint8_t fddcs);

/***************************************************************************//**
 * @brief This function configures the fine NCO channel selection for the
 * specified ADC digital down converters (DDCs) in the device. It should
 * be called after the device has been properly initialized and
 * configured. The `fddcs` parameter allows selection of multiple DDCs,
 * and the `channel` parameter specifies which channel to select for each
 * DDC. If either parameter is invalid or if the device pointer is null,
 * the function will handle the error appropriately. It is important to
 * ensure that the `fddcs` value corresponds to valid DDCs, as the
 * function will iterate through the bits of this value to apply the
 * selection.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param fddcs A bitmask indicating which ADC DDCs to configure. Each bit
 * corresponds to a DDC, with valid values being from 0 to 255.
 * @param channel The channel number to select for the specified DDCs. Valid
 * values depend on the device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * completed without errors. If an error occurs during the operation, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_channel_selection_set(
	adi_ad9081_device_t *device, uint8_t fddcs, uint8_t channel);

/***************************************************************************//**
 * @brief This function configures the fine overall decimation for the ADC
 * digital down converters (DDCs) specified by the `fddcs` parameter. It
 * must be called with a valid `device` pointer that has been properly
 * initialized. The `fddcs` parameter allows for the selection of
 * multiple DDCs, and the function will apply the specified `dcm` value
 * to each selected DDC. If any of the provided parameters are invalid,
 * the function will handle the error gracefully and return an
 * appropriate error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must be initialized before
 * calling this function.
 * @param fddcs A bitmask indicating which ADC DDCs to configure. Each bit
 * corresponds to a DDC, with valid values ranging from 0 to 255
 * (0x00 to 0xFF). If no bits are set, no DDCs will be configured.
 * @param dcm The decimation value to set for the selected DDCs. This value
 * should be within the valid range defined by the device
 * specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_overall_dcm_set(adi_ad9081_device_t *device,
						uint8_t fddcs, uint8_t dcm);

/***************************************************************************//**
 * @brief This function configures the fine Numerically Controlled Oscillator
 * (NCO) frequency for the ADC Digital Down Converter (DDC) based on the
 * specified parameters. It should be called after the device has been
 * properly initialized and configured. The function processes multiple
 * fine DDC channels as indicated by the `fddcs` parameter, and it
 * computes the necessary frequency tuning word (FTW) based on the
 * provided `fddc_shift_hz`. If any of the parameters are invalid or if
 * the device is not initialized, the function will return an error. It
 * is important to ensure that the device pointer is valid and not null
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which fine DDC channels to configure. Each
 * bit corresponds to a specific DDC channel.
 * @param fddc_shift_hz The frequency shift in Hertz to be applied to the fine
 * DDC. This value can be any valid 64-bit integer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_set(adi_ad9081_device_t *device,
					uint8_t fddcs, int64_t fddc_shift_hz);

/***************************************************************************//**
 * @brief This function configures the fine NCO (Numerically Controlled
 * Oscillator) frequency tuning word and its associated modulus values
 * for a specified digital downconverter (DDC). It must be called after
 * initializing the device and selecting the appropriate DDC. The
 * function validates the input parameters to ensure they are within
 * acceptable ranges, specifically that the upper 16 bits of the `ftw`,
 * `modulus_a`, and `modulus_b` parameters are zero. If any parameter is
 * invalid, the function will return an error. Additionally, it handles
 * communication with the device via SPI, and any SPI transfer errors
 * will also result in an error return.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param fddcs An 8-bit identifier for the DDC to configure. Valid values
 * depend on the specific device configuration.
 * @param ftw A 64-bit frequency tuning word. The upper 16 bits must be zero;
 * otherwise, an error is returned.
 * @param modulus_a A 64-bit modulus value for the NCO. The upper 16 bits must
 * be zero; otherwise, an error is returned.
 * @param modulus_b A 64-bit modulus value for the NCO. The upper 16 bits must
 * be zero; otherwise, an error is returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure (e.g., invalid parameters or SPI transfer
 * errors).
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_ftw_set(adi_ad9081_device_t *device,
					    uint8_t fddcs, uint64_t ftw,
					    uint64_t modulus_a,
					    uint64_t modulus_b);

/***************************************************************************//**
 * @brief This function is used to obtain the fine NCO frequency tuning word and
 * the associated modulus values for a specified fine digital
 * downconverter (DDC) in an AD9081 device. It must be called after the
 * device has been properly initialized and configured. The function
 * expects valid pointers for the output parameters, which will be
 * populated with the retrieved values. If any of the input pointers are
 * null, the function will return an error. Additionally, if the
 * specified DDC index is invalid or if there are issues during the
 * retrieval process, appropriate error codes will be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddc An 8-bit unsigned integer representing the index of the fine DDC
 * to query. Valid values depend on the specific device
 * configuration.
 * @param ftw A pointer to a 64-bit unsigned integer where the fine NCO
 * frequency tuning word will be stored. Must not be null.
 * @param modulus_a A pointer to a 64-bit unsigned integer where the first
 * modulus value will be stored. Must not be null.
 * @param modulus_b A pointer to a 64-bit unsigned integer where the second
 * modulus value will be stored. Must not be null.
 * @return Returns an integer status code indicating success or failure of the
 * operation. On success, the output parameters will contain the
 * retrieved values.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_ftw_get(adi_ad9081_device_t *device,
					    uint8_t fddc, uint64_t *ftw,
					    uint64_t *modulus_a,
					    uint64_t *modulus_b);

/***************************************************************************//**
 * @brief This function configures the fine NCO phase offset for a specified
 * fine digital down converter (DDC) in the ADC. It must be called after
 * initializing the `device` and selecting the appropriate DDC using the
 * `fddcs` parameter. The `offset` parameter specifies the desired phase
 * offset value. If the `device` pointer is null or if an error occurs
 * during the selection of the DDC or the SPI transfer, the function will
 * return an error code. It is important to ensure that the `offset`
 * value is within the valid range for phase offsets.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs An 8-bit unsigned integer representing the fine DDC selection.
 * Valid values depend on the specific DDCs available in the
 * device.
 * @param offset A 64-bit unsigned integer representing the phase offset. The
 * valid range is determined by the device specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_fine_nco_phase_offset_set(adi_ad9081_device_t *device,
					     uint8_t fddcs, uint64_t offset);

/***************************************************************************//**
 * @brief This function configures the GPIO mapping for the specified device
 * using the provided `ffh` array, which defines the mapping for various
 * GPIO pins. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to an
 * `adi_ad9081_device_t` structure and an array of six `uint8_t` values
 * representing the GPIO configuration. If the provided device pointer is
 * null, the function will return an error. Additionally, if any of the
 * operations to set the GPIO mapping fail, the function will return an
 * error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param ffh An array of six `uint8_t` values that specify the GPIO mapping.
 * The values at indices 2 to 7 correspond to specific GPIO pins. The
 * caller retains ownership of the array.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_ffh_sel_to_gpio_mapping_set(adi_ad9081_device_t *device,
					       uint8_t ffh[6]);

/***************************************************************************//**
 * @brief This function configures the coarse chip transfer settings for the ADC
 * Digital Down Converter (DDC) in the specified device. It should be
 * called after the device has been properly initialized. The function
 * processes the `cddcs` parameter to determine which DDCs to configure,
 * and it updates the relevant registers accordingly. If the `device`
 * pointer is null, the function will return an error. It is important to
 * ensure that the `cddcs` value is within the valid range to avoid
 * unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDCs to configure. Valid
 * values are determined by the defined constants for the ADC DDCs.
 * The function will ignore bits that are not set.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_chip_xfer_set(adi_ad9081_device_t *device,
						uint8_t cddcs);

/***************************************************************************//**
 * @brief This function is used to obtain the transfer status of the coarse
 * digital downconverter (DDC) chip for a specified device. It should be
 * called after the device has been properly initialized and configured.
 * The function checks the status for up to four coarse DDC channels,
 * indicated by the `cddcs` parameter. If the transfer is complete, the
 * status will be updated accordingly. It is important to ensure that the
 * `device` pointer is not null before calling this function, as it will
 * return an error if it is. Additionally, the `status` pointer must be
 * valid and allocated by the caller to receive the transfer status.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDC channels to check. Valid
 * values are determined by the defined constants for the coarse
 * DDC channels.
 * @param status A pointer to a `uint8_t` variable where the transfer status
 * will be stored. Caller retains ownership and must ensure it is
 * not null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while other values indicate an error occurred during the operation.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_coarse_chip_xfer_status_get(adi_ad9081_device_t *device,
					       uint8_t cddcs, uint8_t *status);

/***************************************************************************//**
 * @brief This function configures the fine DDC chip transfer settings for the
 * specified device. It should be called after the device has been
 * properly initialized and is ready for configuration. The function
 * processes the `fddcs` parameter to determine which fine DDC channels
 * to configure, and it updates the relevant registers accordingly. If
 * the `device` pointer is null or if any internal operation fails, the
 * function will handle these cases gracefully by returning an error
 * code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param fddcs A bitmask indicating which fine DDC channels to configure. Each
 * bit corresponds to a specific channel, where a value of 1
 * indicates that the channel should be configured.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_chip_xfer_set(adi_ad9081_device_t *device,
					      uint8_t fddcs);

/***************************************************************************//**
 * @brief This function is used to obtain the transfer status of the fine
 * digital downconverter (DDC) for specific channels in the AD9081
 * device. It should be called after the device has been properly
 * initialized and configured. The function checks the status for up to 8
 * channels, as specified by the `fddcs` parameter. If the transfer is
 * complete, the status will indicate this; otherwise, it will show that
 * the transfer is not requested or not completed. It is important to
 * ensure that the `device` pointer is valid and not null before calling
 * this function, as passing a null pointer will result in an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which fine DDC channels to check. Each bit
 * corresponds to a channel (0-7). Valid values are 0 to 255.
 * @param status A pointer to a `uint8_t` where the transfer status will be
 * stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the status has
 * been successfully retrieved. If an error occurs, an appropriate error
 * code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_fine_chip_xfer_status_get(adi_ad9081_device_t *device,
					     uint8_t fddcs, uint8_t *status);

/***************************************************************************//**
 * @brief This function is intended to be called to synchronize the Numerically
 * Controlled Oscillator (NCO) of the AD9081 device. It should be invoked
 * after the device has been properly initialized and configured. The
 * function checks the device revision and performs specific register
 * settings based on that revision. If the provided `device` pointer is
 * null, the function will return an error. It is important to ensure
 * that the device is in a valid state before calling this function to
 * avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null. If the pointer is null, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful synchronization, or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_pre(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is intended to be called to synchronize the Numerically
 * Controlled Oscillator (NCO) of the AD9081 device. It should be invoked
 * after the device has been properly initialized and configured. The
 * function checks for a null pointer to the device structure and returns
 * an error if the pointer is invalid. If the device revision is 3, it
 * performs specific register configurations to ensure proper
 * synchronization. It is important to handle any potential errors
 * returned by the function, particularly in the context of device
 * initialization and configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful synchronization, or an error
 * code if an error occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_post(adi_ad9081_device_t *device);

/*===== 3 . 4   R E C E I V E  P A T H  N C O  D E L A Y  A P I S  =====*/
/***************************************************************************//**
 * @brief This function configures the fine status selection for the ADC digital
 * down converter (DDC) in the specified device. It should be called
 * after the device has been properly initialized. The function allows
 * the user to specify which fine DDCs are active and their corresponding
 * I and Q status selections. If the `device` pointer is null, the
 * function will return an error. The function iterates through the fine
 * DDC selections, applying the specified I and Q status selections only
 * for the active DDCs.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs An enumeration value of type `adi_ad9081_adc_fine_ddc_select_e`
 * that specifies which fine DDCs are to be configured. Valid
 * values depend on the defined enumeration.
 * @param i_status_adc An enumeration value of type `adi_ad9081_adc_select_e`
 * that specifies the I status selection for the active fine
 * DDCs. Valid values depend on the defined enumeration.
 * @param q_status_adc An enumeration value of type `adi_ad9081_adc_select_e`
 * that specifies the Q status selection for the active fine
 * DDCs. Valid values depend on the defined enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_fine_status_sel_set(adi_ad9081_device_t *device,
				       adi_ad9081_adc_fine_ddc_select_e fddcs,
				       adi_ad9081_adc_select_e i_status_adc,
				       adi_ad9081_adc_select_e q_status_adc);

/***************************************************************************//**
 * @brief This function configures the cycle delay selection for one or more
 * ADCs in the device. It should be called after the device has been
 * properly initialized and configured. The `adcs` parameter specifies
 * which ADCs to configure, and the `cd_index` parameter determines the
 * specific cycle delay setting to apply. If the `device` pointer is
 * null, the function will return an error. Additionally, if any of the
 * ADC selection or HAL bitfield setting operations fail, the function
 * will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates the ADC is
 * selected.
 * @param cd_index An index representing the desired cycle delay setting. Valid
 * values depend on the specific configuration of the ADCs.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_cycle_delay_selection_set(adi_ad9081_device_t *device,
						 uint8_t adcs,
						 uint8_t cd_index);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ADC cycle delay feature
 * for one or more ADCs in the device. It should be called after the
 * device has been properly initialized. The `adcs` parameter specifies
 * which ADCs to configure, and the `enable` parameter determines whether
 * to enable or disable the cycle delay. If an invalid pointer is
 * provided for the `device`, or if an error occurs while selecting the
 * ADC or setting the configuration, the function will handle these cases
 * appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC (0-3). Valid values are 0 to 15.
 * @param enable A boolean value where 1 enables the cycle delay and 0 disables
 * it. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_cycle_delay_enable_set(adi_ad9081_device_t *device,
					      uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ADC cycle delay feature
 * for specified ADCs on the device. It should be called after the device
 * has been properly initialized. The `adcs` parameter allows selection
 * of which ADCs to configure, and the `enable` parameter determines
 * whether to enable or disable the cycle delay. It is important to
 * ensure that the `device` pointer is not null before calling this
 * function, as passing a null pointer will result in an error. The
 * function will iterate through the specified ADCs and apply the
 * configuration accordingly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates the ADC
 * should be affected.
 * @param enable A boolean value (0 or 1) indicating whether to disable (0) or
 * enable (1) the ADC cycle delay.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_cycle_delay_enable_via_gpio_set(adi_ad9081_device_t *device,
					       uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the cycle delay for up to four ADCs in the
 * `adi_ad9081_device_t` device. It should be called after the device has
 * been properly initialized and configured. The `adcs` parameter
 * specifies which ADCs to configure, and the `delay` array provides the
 * delay values for each selected ADC. If any of the specified ADCs are
 * invalid or if the `device` pointer is null, the function will return
 * an error. It is important to ensure that the `delay` array contains
 * valid values, as the function will attempt to set these values for
 * each selected ADC.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC0, 1 for
 * ADC1, etc.).
 * @param delay An array of four `uint8_t` values representing the delay
 * settings for the selected ADCs. The array must have exactly four
 * elements, and the function will set the delay for each ADC that
 * is indicated by the `adcs` bitmask.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs during the
 * configuration process, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_cycle_delay_set(adi_ad9081_device_t *device,
				       uint8_t adcs, uint8_t delay[4]);

/***************************************************************************//**
 * @brief This function configures the fractional delay status selection for the
 * I and Q channels of the ADC in the specified device. It must be called
 * after the device has been properly initialized. If the `device`
 * pointer is null, the function will return an error. The function sets
 * the status selections for both I and Q channels, and it is important
 * to ensure that valid selections are provided. If any of the selections
 * are invalid, the function will return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param i_status_sel An enumeration value of type `adi_ad9081_adc_select_e`
 * representing the I channel status selection. Valid values
 * depend on the specific implementation of the enumeration.
 * @param q_status_sel An enumeration value of type `adi_ad9081_adc_select_e`
 * representing the Q channel status selection. Valid values
 * depend on the specific implementation of the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_fractional_delay_status_selection_set(
	adi_ad9081_device_t *device, adi_ad9081_adc_select_e i_status_sel,
	adi_ad9081_adc_select_e q_status_sel);

/***************************************************************************//**
 * @brief This function configures the fractional delay selection for the ADC in
 * the specified device. It must be called with a valid `device` pointer
 * that has been properly initialized. The `fd_index` parameter
 * determines which fractional delay setting to apply, and it should be
 * within the valid range defined by the device specifications. If the
 * `device` pointer is null or if an error occurs during the setting
 * process, the function will handle these cases appropriately, returning
 * an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must be initialized before
 * calling this function.
 * @param fd_index An 8-bit unsigned integer representing the fractional delay
 * index to set. Valid values depend on the device
 * specifications; passing an invalid value may result in an
 * error.
 * @return Returns an integer error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t
adi_ad9081_adc_fractional_delay_selection_set(adi_ad9081_device_t *device,
					      uint8_t fd_index);

/***************************************************************************//**
 * @brief This function is used to enable or disable the fractional delay down-
 * sampling feature of the ADC in the specified device. It should be
 * called after the device has been properly initialized. If the `device`
 * pointer is null, the function will return an error without making any
 * changes. The `enable` parameter determines whether the feature is
 * turned on or off, and it is expected to be either 0 (disable) or 1
 * (enable). It is important to check the return value to ensure that the
 * operation was successful.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A uint8_t value that specifies whether to enable (1) or disable
 * (0) the fractional delay down-sampling feature. Valid values
 * are 0 and 1.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed successfully. If an error occurs, a negative error code
 * is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_fractional_delay_down_sample_enable_set(
	adi_ad9081_device_t *device, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the fractional delay
 * feature for the ADC in the specified device. It must be called with a
 * valid `device` pointer that has been properly initialized. If the
 * `device` pointer is null, the function will return an error. The
 * `enable` parameter determines whether the fractional delay is
 * activated (when set to a non-zero value) or deactivated (when set to
 * zero). It is important to ensure that the device is in a state that
 * allows for this configuration change.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the fractional delay feature.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors.
 ******************************************************************************/
int32_t adi_ad9081_adc_fractional_delay_enable_set(adi_ad9081_device_t *device,
						   uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the fractional delay
 * feature of the ADC through a GPIO pin. It should be called after the
 * device has been properly initialized. The `enable` parameter
 * determines whether the feature is activated (when set to a non-zero
 * value) or deactivated (when set to zero). It is important to ensure
 * that the `device` pointer is valid and not null before calling this
 * function, as passing a null pointer will result in an immediate error.
 * The function will return an error code if the operation fails for any
 * reason.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the fractional delay feature.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_adc_fractional_delay_enable_via_gpio_set(adi_ad9081_device_t *device,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the fractional delay settings for the ADC in
 * the specified device. It must be called after the device has been
 * properly initialized. The `delay` parameter should contain four values
 * representing the fractional delays for different channels. If any of
 * the provided values are invalid or if the `device` pointer is null,
 * the function will return an error. It is important to ensure that the
 * device is ready for configuration before invoking this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param delay An array of four `uint8_t` values representing the fractional
 * delays for the ADC channels. Each value should be within the
 * valid range for fractional delays. The caller retains ownership
 * of the array.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_fractional_delay_set(adi_ad9081_device_t *device,
					    uint8_t delay[4]);

/***************************************************************************//**
 * @brief This function configures the fractional delay input/output multiplexer
 * for the specified device. It should be called after the device has
 * been properly initialized. The `mux` parameter determines which coarse
 * digital downconverter (DDC) mixer is connected to the fractional delay
 * I/O: a value of 0 connects it to the coarse DDC0 mixer, while a value
 * of 1 connects it to the coarse DDC3 mixer. If the `device` pointer is
 * null, the function will return an error. Additionally, if the
 * operation fails, an error code will be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mux An 8-bit unsigned integer that selects the multiplexer
 * configuration. Valid values are 0 or 1, where 0 connects to the
 * coarse DDC0 mixer and 1 connects to the coarse DDC3 mixer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_fractional_delay_io_mux_set(adi_ad9081_device_t *device,
						   uint8_t mux);

/*===== 3 . 5   R E C E I V E  P A T H  D E C I M A T I O N  &  C 2 R =====*/
/***************************************************************************//**
 * @brief This function configures the decimation ratio for the ADC chip
 * associated with the specified device. It must be called with a valid
 * `device` pointer that has been properly initialized. The `links`
 * parameter determines which JESD links (0 or 1) will be configured, and
 * the `dcm` parameter specifies the desired decimation ratio. If the
 * specified `links` do not correspond to any active links, the function
 * will not perform any configuration. It is important to ensure that the
 * `dcm` value is within the acceptable range for the device, as invalid
 * values may lead to errors during execution.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD links to configure. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`. The function will only
 * configure the links that are specified.
 * @param dcm An 8-bit unsigned integer representing the decimation ratio to be
 * set. The valid range for this value should be defined in the
 * device's specifications. Invalid values may result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or internal errors.
 ******************************************************************************/
int32_t adi_ad9081_adc_chip_dcm_ratio_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t dcm);

/***************************************************************************//**
 * @brief This function is used to obtain the decimation ratio of the ADC chip
 * for specified JESD links. It must be called after the device has been
 * properly initialized. The function checks which links are selected and
 * retrieves the corresponding decimation ratio, writing the result to
 * the provided pointer. If the `device` pointer is null or if any errors
 * occur during the process, appropriate error handling will be
 * triggered. It is important to ensure that the `dcm` pointer is valid
 * and points to a location where the result can be stored.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to query. Valid values are `AD9081_LINK_0` and
 * `AD9081_LINK_1`.
 * @param dcm A pointer to a `uint8_t` where the decimation ratio will be
 * stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, a negative
 * error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_chip_dcm_ratio_get(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t *dcm);

/***************************************************************************//**
 * @brief This function configures the coarse Digital Clock Manager (DCM)
 * settings for the specified ADC Digital Down Converters (DDCs). It
 * should be called after initializing the `device` and before using the
 * DDCs. The function processes up to four DDCs based on the `cddcs`
 * parameter, applying the specified DCM setting to each active DDC. If
 * the `device` pointer is null, the function will return an error. It is
 * important to ensure that the `cddcs` parameter correctly represents
 * the DDCs to be configured.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which ADC DDCs to configure. Each bit
 * corresponds to a DDC, where a value of 1 indicates that the DDC
 * is active.
 * @param dcm An enumeration value of type `adi_ad9081_adc_coarse_ddc_dcm_e`
 * that specifies the DCM setting to apply. Must be a valid
 * enumeration value.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the configuration process.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_dcm_set(adi_ad9081_device_t *device,
					  uint8_t cddcs,
					  adi_ad9081_adc_coarse_ddc_dcm_e dcm);

/***************************************************************************//**
 * @brief This function configures the coarse C2R settings for the ADC digital
 * down converter (DDC) in the specified device. It should be called
 * after the device has been properly initialized. The function iterates
 * over the coarse decimation digital down converter settings specified
 * by `cddcs`, applying the C2R enable setting `c2r_en` for each active
 * coarse decimation setting. If the `device` pointer is null, the
 * function will return an error. It is important to ensure that the
 * `cddcs` parameter correctly represents the desired coarse decimation
 * settings.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask representing the coarse decimation digital down
 * converter settings. Valid values are defined by the
 * `AD9081_ADC_CDDC_0` constants.
 * @param c2r_en A value indicating whether to enable or disable the C2R
 * setting. Typically a binary value (0 or 1).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_c2r_set(adi_ad9081_device_t *device,
					  uint8_t cddcs, uint8_t c2r_en);

/***************************************************************************//**
 * @brief This function configures the fine Digital Clock Management (DCM)
 * settings for the ADC Digital Down Converter (DDC) of the specified
 * device. It should be called after the device has been properly
 * initialized and configured. The function allows for the selection of
 * multiple fine DDCs, and it modifies the bypass settings based on the
 * provided DCM value. If an invalid device pointer is provided, the
 * function will return an error. Additionally, it handles errors that
 * may occur during the configuration process, ensuring that the caller
 * is informed of any issues.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which fine DDCs to configure. Each bit
 * corresponds to a fine DDC, with valid values ranging from 0 to
 * 255.
 * @param dcm An enumeration value of type `adi_ad9081_adc_fine_ddc_dcm_e` that
 * specifies the DCM setting. Valid values are defined in the
 * enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_dcm_set(adi_ad9081_device_t *device,
					uint8_t fddcs,
					adi_ad9081_adc_fine_ddc_dcm_e dcm);

/***************************************************************************//**
 * @brief This function configures the fine C2R setting for the ADC DDC of the
 * specified device. It should be called after the device has been
 * properly initialized. The function expects a valid device pointer and
 * a valid FDDC selection. The `c2r_en` parameter must be either 0 or 1,
 * where 1 enables the C2R feature. If the provided `device` pointer is
 * null or if `c2r_en` is out of range, the function will return an
 * error. The function iterates through the FDDC settings, applying the
 * configuration for each enabled FDDC.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask representing the FDDC channels to configure. Each bit
 * corresponds to a specific FDDC channel.
 * @param c2r_en An integer that enables (1) or disables (0) the C2R feature.
 * Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the configuration.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_c2r_set(adi_ad9081_device_t *device,
					uint8_t fddcs, uint8_t c2r_en);

/*===== 3 . 6   R E C E I V E  P A T H  G A I N =====*/
/***************************************************************************//**
 * @brief This function configures the coarse gain for the ADC Digital Down
 * Converter (DDC) of the specified device. It should be called after the
 * device has been properly initialized and configured. The function
 * expects a valid device pointer and a gain value of either 0 or 1. If
 * the gain value is invalid or if the device pointer is null, the
 * function will handle these errors appropriately. The function operates
 * on up to four DDC channels, applying the specified gain to each
 * selected channel.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which DDC channels to configure. Each bit
 * corresponds to a DDC channel.
 * @param gain The coarse gain value to set for the selected DDC channels. Valid
 * values are 0 or 1. If the value is greater than 1, the function
 * will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_gain_set(adi_ad9081_device_t *device,
					   uint8_t cddcs, uint8_t gain);

/*===== 3 . 7   R E C E I V E  P A T H  H E L P E R  A P I S =====*/
/***************************************************************************//**
 * @brief This function is used to reset the digital datapath of the ADC in the
 * `adi_ad9081_device_t` context. It should be called when a reset of the
 * ADC's digital processing is required, typically during initialization
 * or recovery from an error state. The function expects a valid device
 * pointer and will return an error if the pointer is null. It is
 * important to ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the ADC device. Must not be null; the function will return an
 * error if it is.
 * @param reset A `uint8_t` value indicating the reset state. Valid values are
 * typically 0 (no reset) or 1 (reset). The function will handle
 * invalid values by treating them as a no-operation.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the reset
 * command was successfully issued.
 ******************************************************************************/
int32_t adi_ad9081_adc_digital_datapath_reset_set(adi_ad9081_device_t *device,
						  uint8_t reset);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC clock frequency from
 * the specified device. It must be called after the device has been
 * properly initialized. The function expects a valid pointer to an
 * `adi_ad9081_device_t` structure and a pointer to a `uint64_t` variable
 * where the frequency will be stored. If either pointer is null, the
 * function will return an error without modifying the output variable.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @param adc_clk_hz A pointer to a `uint64_t` variable where the ADC clock
 * frequency will be stored. Must not be null; otherwise, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the ADC clock
 * frequency has been successfully retrieved and stored in the provided
 * variable.
 ******************************************************************************/
int32_t adi_ad9081_adc_clk_freq_get(adi_ad9081_device_t *device,
				    uint64_t *adc_clk_hz);

/***************************************************************************//**
 * @brief This function configures the ADC coarse digital downconverter crossbar
 * for the specified device. It must be called with a valid device
 * pointer that has been properly initialized. If the device pointer is
 * null, the function will return an error. The `adc_cddc_xbar` parameter
 * specifies the desired crossbar configuration and should be within the
 * valid range defined by the device specifications. Calling this
 * function with an invalid configuration may result in an error being
 * returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must point to a valid
 * initialized device.
 * @param adc_cddc_xbar An 8-bit unsigned integer representing the ADC coarse
 * digital downconverter crossbar configuration. The valid
 * range is defined by the device specifications; passing
 * an invalid value may lead to an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * configuration was set successfully. If an error occurs, a negative
 * error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_adc2cddc_xbar_set(adi_ad9081_device_t *device,
					 uint8_t adc_cddc_xbar);

/***************************************************************************//**
 * @brief This function configures the crossbar settings for the CDDC to FDDC
 * mapping in the specified device. It must be called with a valid device
 * pointer that has been properly initialized. If the device pointer is
 * null, the function will return an error without making any changes.
 * The `cddc_fddc_xbar` parameter should be set to a valid value that
 * corresponds to the desired crossbar configuration. The function will
 * return an error if the underlying hardware access fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must point to a valid
 * initialized device.
 * @param cddc_fddc_xbar A `uint8_t` value representing the crossbar
 * configuration. Valid values depend on the specific
 * configuration options defined in the device's
 * documentation.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_cddc2fddc_xbar_set(adi_ad9081_device_t *device,
					  uint8_t cddc_fddc_xbar);

/***************************************************************************//**
 * @brief This function is used to obtain the crossbar settings for the ADC and
 * CDDC from the specified device. It must be called with a valid
 * `device` pointer, and the output parameters `adc_cddc_xbar` and
 * `cddc_fddc_xbar` must also be valid pointers. If any of these pointers
 * are null, the function will return an error. The function is expected
 * to be called after the device has been properly initialized. It
 * retrieves the configuration values from the device's registers and
 * populates the provided pointers with the corresponding data.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adc_cddc_xbar A pointer to a `uint8_t` where the ADC crossbar
 * configuration will be stored. Must not be null.
 * @param cddc_fddc_xbar A pointer to a `uint8_t` where the CDDC crossbar
 * configuration will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any of the
 * input pointers are null or if there is an error retrieving the
 * configuration values.
 ******************************************************************************/
int32_t adi_ad9081_adc_xbar_get(adi_ad9081_device_t *device,
				uint8_t *adc_cddc_xbar,
				uint8_t *cddc_fddc_xbar);

/***************************************************************************//**
 * @brief This function is used to determine the corresponding CDDC value for a
 * specified FDDC input. It must be called with a valid `device` pointer
 * that has been properly initialized. The function retrieves a value
 * from the device's hardware registers and processes it to find the
 * appropriate CDDC. If the input `fddc` is valid, the function will
 * write the resulting CDDC value to the provided pointer. It is
 * important to ensure that the `cddc` pointer is not null before calling
 * this function, as it will be dereferenced to store the result.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddc A uint8_t value representing the FDDC input. Valid values are
 * defined by the `AD9081_ADC_FDDC_0` macro and its shifts.
 * @param cddc A pointer to a uint8_t where the resulting CDDC value will be
 * stored. Caller retains ownership and must ensure this pointer is
 * not null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_xbar_find_cddc(adi_ad9081_device_t *device, uint8_t fddc,
				      uint8_t *cddc);

/***************************************************************************//**
 * @brief This function configures the coarse digital downconverter (DDC)
 * selection for the ADC in the specified device. It must be called after
 * the device has been properly initialized. The function checks for null
 * pointers and validates the `cddcs` parameter to ensure it is within an
 * acceptable range. If the input is invalid or if there are issues
 * accessing the device's registers, appropriate error handling will
 * occur. It is important to ensure that the device is not null and that
 * the `cddcs` value does not exceed the defined maximum.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs An 8-bit unsigned integer representing the coarse DDC selection.
 * Valid values must be less than or equal to
 * `AD9081_ADC_CDDC_ALL`. If the value exceeds this limit, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation was not successful.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_select_set(adi_ad9081_device_t *device,
					     uint8_t cddcs);

/***************************************************************************//**
 * @brief This function is used to initiate a coarse reset for the ADC Digital
 * Down Converter (DDC) on the specified device. It should be called when
 * a reset of the DDC is required, typically during initialization or
 * when reconfiguring the DDC settings. The function expects a valid
 * device pointer and will perform a series of operations to reset the
 * specified coarse DDCs based on the provided mask. If any of the
 * specified DDCs are invalid or if the device pointer is null, the
 * function will handle these cases appropriately by returning an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which coarse DDCs to reset. Each bit
 * corresponds to a DDC; valid values are 0 to 15, where each bit
 * set to 1 indicates a DDC to be reset.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while any negative value indicates an error occurred during
 * the reset process.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_reset_set(adi_ad9081_device_t *device,
					    uint8_t cddcs);

/***************************************************************************//**
 * @brief This function is used to enable or disable coarse synchronization for
 * specific digital down converters (DDCs) in the AD9081 device. It
 * should be called after the device has been properly initialized and
 * configured. The `cddcs` parameter specifies which DDCs to modify, and
 * the `enable` parameter determines whether to enable or disable
 * synchronization. If the `device` pointer is null, the function will
 * return an error. The function iterates through the specified DDCs and
 * applies the synchronization setting accordingly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which DDCs to modify. Each bit corresponds
 * to a DDC, where a set bit indicates that the respective DDC
 * should be affected.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) coarse synchronization for the specified DDCs.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_sync_enable_set(adi_ad9081_device_t *device,
						  uint8_t cddcs,
						  uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the coarse synchronization value
 * for one or more ADC Digital Down Converters (DDCs) in the device. It
 * should be called after the device has been properly initialized and
 * configured. The function iterates over the specified DDCs, applying
 * the synchronization value to each active DDC. If the `device` pointer
 * is null, the function will return an error without making any changes.
 * It is important to ensure that the `cddcs` parameter correctly
 * represents the DDCs to be configured, as only those specified will be
 * affected.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which ADC DDCs to configure. Each bit
 * corresponds to a DDC, where a set bit indicates that the
 * respective DDC should be configured.
 * @param val The synchronization value to be set for the selected DDCs. This
 * value should be within the valid range defined by the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_sync_next_set(adi_ad9081_device_t *device,
						uint8_t cddcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the coarse trigger NCO
 * reset for specific digital down converters (DDCs) in the AD9081
 * device. It should be called after the device has been properly
 * initialized. The `cddcs` parameter allows selection of which DDCs to
 * configure, and the `enable` parameter determines whether the reset is
 * enabled or disabled. If an invalid pointer is provided for the
 * `device`, the function will return an error. The function handles
 * multiple DDCs by iterating through the specified `cddcs` bits,
 * applying the configuration to each selected DDC.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A bitmask indicating which DDCs to configure. Each bit
 * corresponds to a DDC, with valid values being from 0 to 3.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the coarse trigger NCO reset.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_trig_nco_reset_enable_set(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ADC clock output for a
 * specified device. It must be called with a valid device pointer that
 * has been properly initialized. If the device pointer is null, the
 * function will return an error. The `enable` parameter determines
 * whether the clock output is activated (when set to a non-zero value)
 * or deactivated (when set to zero). It is important to ensure that this
 * function is called in the appropriate context where the device is
 * ready for configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the ADC clock output.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors.
 ******************************************************************************/
int32_t adi_ad9081_adc_clk_out_enable_set(adi_ad9081_device_t *device,
					  uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the output voltage swing of the ADC clock in
 * the specified device. It should be called after the device has been
 * properly initialized. The `swing_mv` parameter determines the desired
 * voltage swing in millivolts, which must be within the range of -1000
 * to 1000 mV. If the provided value is outside this range, the function
 * will return an error. The function will also return an error if the
 * `device` pointer is null. Upon successful execution, the voltage swing
 * is set according to the specified value.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param swing_mv An integer representing the desired voltage swing in
 * millivolts. Valid range is from -1000 to 1000 mV. Values
 * outside this range will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the input
 * parameters are invalid or if an error occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_clk_out_voltage_swing_set(adi_ad9081_device_t *device,
						 int16_t swing_mv);

/***************************************************************************//**
 * @brief This function configures the GPIO pin that corresponds to the ADC's
 * FD_EN signal for the specified device. It must be called after the
 * device has been properly initialized. The `fd_en_gpio` parameter
 * should be within the valid range of GPIO mappings, which are defined
 * as 2 through 8 for specific GPIO pins and 7 and 8 for sync inputs. If
 * the provided `device` pointer is null or if an error occurs during the
 * configuration, the function will handle these cases appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fd_en_gpio An 8-bit unsigned integer representing the GPIO pin mapping
 * for the FD_EN signal. Valid values are 2 to 8,
 * corresponding to specific GPIO pins and sync inputs. If an
 * invalid value is provided, the function will return an
 * error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the GPIO
 * mapping was set successfully. If an error occurs, an appropriate
 * error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_fd_en_to_gpio_mapping_set(adi_ad9081_device_t *device,
						 uint8_t fd_en_gpio);

/***************************************************************************//**
 * @brief This function configures the GPIO pins used for enabling the ADC
 * receive channels in the `adi_ad9081_device_t`. It must be called after
 * the device has been initialized and before any ADC operations are
 * performed. The function expects valid GPIO pin numbers for the RX
 * enable signals, which should be within the specified range. If the
 * provided `device` pointer is null, the function will return an error.
 * Additionally, if the GPIO values are invalid, the function will also
 * return an error, ensuring that the configuration is safe and correct.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rxen1_gpio The GPIO pin number for the first RX enable signal. Valid
 * values are typically between 2 and 8, corresponding to
 * specific GPIOs. The function will return an error if this
 * value is out of range.
 * @param rxen3_gpio The GPIO pin number for the third RX enable signal. Valid
 * values are typically between 2 and 8, corresponding to
 * specific GPIOs. The function will return an error if this
 * value is out of range.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or device state.
 ******************************************************************************/
int32_t adi_ad9081_adc_rx_en_to_gpio_mapping_set(adi_ad9081_device_t *device,
						 uint8_t rxen1_gpio,
						 uint8_t rxen3_gpio);

/***************************************************************************//**
 * @brief This function is used to configure the user-defined test patterns for
 * the ADC in the `adi_ad9081_device_t`. It should be called after the
 * device has been properly initialized. The function takes two arrays of
 * 8 elements each, representing the in-phase (I) and quadrature (Q)
 * components of the test patterns. It is important to ensure that the
 * provided arrays are valid and not null, as passing a null pointer will
 * result in an error. The function will write the specified patterns to
 * the appropriate registers, and any errors encountered during this
 * process will be returned to the caller.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param i_pattern An array of 16 `uint8_t` values representing the in-phase
 * test pattern. The caller retains ownership of this array.
 * @param q_pattern An array of 16 `uint8_t` values representing the quadrature
 * test pattern. The caller retains ownership of this array.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_test_mode_usr_pattern_set(adi_ad9081_device_t *device,
						 uint8_t i_pattern[16],
						 uint8_t q_pattern[16]);

/***************************************************************************//**
 * @brief This function is used to configure the test mode for the ADC
 * converters in the device. It should be called after the device has
 * been properly initialized. The function sets the test mode for both
 * the in-phase (I) and quadrature (Q) channels, allowing for various
 * testing scenarios. If both modes are set to 'off', the function
 * disables the test mode for all converters. It is important to ensure
 * that the `device` parameter is valid and not null before calling this
 * function, as passing a null pointer will result in an error.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param i_mode The test mode for the in-phase channel, represented by the
 * `adi_ad9081_test_mode_e` enumeration. Valid values are defined
 * in the enumeration.
 * @param q_mode The test mode for the quadrature channel, represented by the
 * `adi_ad9081_test_mode_e` enumeration. Valid values are defined
 * in the enumeration.
 * @param links The JESD link selection, represented by the
 * `adi_ad9081_jesd_link_select_e` enumeration. Valid values are
 * defined in the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_test_mode_config_set(
	adi_ad9081_device_t *device, adi_ad9081_test_mode_e i_mode,
	adi_ad9081_test_mode_e q_mode, adi_ad9081_jesd_link_select_e links);

/***************************************************************************//**
 * @brief This function is used to configure the trigger programming delay for
 * the ADC in the `adi_ad9081_device_t`. It must be called after the
 * device has been properly initialized. The `delay` parameter specifies
 * the desired delay value, which should be within the valid range
 * defined by the device specifications. If the `device` pointer is null,
 * the function will return an error without making any changes. It is
 * important to handle any potential errors returned by this function to
 * ensure proper operation.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param delay An 8-bit unsigned integer representing the trigger programming
 * delay. The valid range for this value should be defined in the
 * device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the delay has
 * been set successfully. If an error occurs, an appropriate error code
 * is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_trig_prog_delay_set(adi_ad9081_device_t *device,
					   uint8_t delay);

/***************************************************************************//**
 * @brief This function is used to configure the rising edge trigger for the ADC
 * in the `adi_ad9081_device_t` context. It should be called after the
 * device has been properly initialized. The `enable` parameter
 * determines whether the rising edge trigger is activated or
 * deactivated. If the `device` pointer is null, the function will return
 * an error without making any changes. It is important to ensure that
 * the device is ready for configuration before invoking this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that specifies whether to enable (non-zero)
 * or disable (zero) the rising edge trigger.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_trig_rise_edge_enable_set(adi_ad9081_device_t *device,
						 uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the master trigger
 * functionality of the ADC in the specified device. It should be called
 * after the device has been properly initialized. If the `device`
 * pointer is null, the function will return an error without making any
 * changes. The `enable` parameter determines whether the master trigger
 * is activated (when set to a non-zero value) or deactivated (when set
 * to zero). It is important to ensure that the device is in a valid
 * state before calling this function to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that specifies whether to enable (non-zero)
 * or disable (zero) the master trigger.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed successfully.
 ******************************************************************************/
int32_t adi_ad9081_adc_master_trig_enable_set(adi_ad9081_device_t *device,
					      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the loopback master trigger
 * feature of the AD9081 device. It should be called after the device has
 * been properly initialized. If the `device` pointer is null, the
 * function will return an error without making any changes. The `enable`
 * parameter determines whether the loopback master trigger is activated
 * (when set to a non-zero value) or deactivated (when set to zero). It
 * is important to ensure that the device is in a valid state before
 * calling this function to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that specifies whether to enable (non-zero)
 * or disable (zero) the loopback master trigger.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed successfully.
 ******************************************************************************/
int32_t
adi_ad9081_adc_loopback_master_trig_enable_set(adi_ad9081_device_t *device,
					       uint8_t enable);

/*===== 3 . 8   R E C E I V E  P A T H   P O W E R  S A V I N G S =====*/
/***************************************************************************//**
 * @brief This function configures the power-down control settings for the ADC
 * channels of the specified device. It should be called after the device
 * has been properly initialized. The parameters control the enable state
 * for different ADC channels, and they must be set to either 0 or 1. If
 * any parameter is invalid or if the device pointer is null, the
 * function will return an error. It is important to ensure that the
 * device is not null and that the parameters are within the valid range
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rxen0_0f_ctrl_en Control enable for ADC channel 0 (0 or 1). Must be 0
 * or 1; otherwise, an error is returned.
 * @param rxengp0_0f_ctrl_en Control enable for ADC channel 0 GP (0 or 1). Must
 * be 0 or 1; otherwise, an error is returned.
 * @param rxen0_0s_ctrl_en Control enable for ADC channel 0 (0 or 1). Must be 0
 * or 1; otherwise, an error is returned.
 * @param rxengp0_0s_ctrl_en Control enable for ADC channel 0 GP (0 or 1). Must
 * be 0 or 1; otherwise, an error is returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * parameter is invalid or if the device pointer is null.
 ******************************************************************************/
int32_t adi_ad9081_adc_adc0_rxen_pwdn_ctrl_set(adi_ad9081_device_t *device,
					       uint8_t rxen0_0f_ctrl_en,
					       uint8_t rxengp0_0f_ctrl_en,
					       uint8_t rxen0_0s_ctrl_en,
					       uint8_t rxengp0_0s_ctrl_en);

/***************************************************************************//**
 * @brief This function configures the power-down control settings for the ADC
 * channels of the device. It must be called after the device has been
 * properly initialized. The parameters control the enable state for
 * different ADC channels, and they should be set to either 0 (disabled)
 * or 1 (enabled). If any parameter is outside this range or if the
 * `device` pointer is null, the function will return an error. It is
 * important to handle the return value to ensure that the settings were
 * applied successfully.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rxen1_1f_ctrl_en Controls the enable state for the first channel of
 * the ADC. Valid values are 0 or 1.
 * @param rxengp1_1f_ctrl_en Controls the enable state for the first gain path
 * of the ADC. Valid values are 0 or 1.
 * @param rxen1_1s_ctrl_en Controls the enable state for the second channel of
 * the ADC. Valid values are 0 or 1.
 * @param rxengp1_1s_ctrl_en Controls the enable state for the second gain path
 * of the ADC. Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * parameter is invalid or if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_adc1_rxen_pwdn_ctrl_set(adi_ad9081_device_t *device,
					       uint8_t rxen1_1f_ctrl_en,
					       uint8_t rxengp1_1f_ctrl_en,
					       uint8_t rxen1_1s_ctrl_en,
					       uint8_t rxengp1_1s_ctrl_en);

/***************************************************************************//**
 * @brief This function configures the power-down control settings for specific
 * ADC channels in the device. It must be called after the device has
 * been properly initialized. The parameters control the enable state for
 * two different ADC configurations, and it is important to ensure that
 * the values provided are either 0 or 1. If any parameter is invalid or
 * if the device pointer is null, the function will handle these cases
 * gracefully by returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rxen0_2f_ctrl_en Controls the enable state for the RXEN0_2F channel.
 * Valid values are 0 (disabled) or 1 (enabled). If the
 * value is outside this range, an error will be
 * returned.
 * @param rxengp0_2f_ctrl_en Controls the enable state for the RXENGP0_2F
 * channel. Valid values are 0 (disabled) or 1
 * (enabled). If the value is outside this range, an
 * error will be returned.
 * @param rxen0_2s_ctrl_en Controls the enable state for the RXEN0_2S channel.
 * Valid values are 0 (disabled) or 1 (enabled). If the
 * value is outside this range, an error will be
 * returned.
 * @param rxengp0_2s_ctrl_en Controls the enable state for the RXENGP0_2S
 * channel. Valid values are 0 (disabled) or 1
 * (enabled). If the value is outside this range, an
 * error will be returned.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates that the
 * settings were successfully applied.
 ******************************************************************************/
int32_t adi_ad9081_adc_adc2_rxen_pwdn_ctrl_set(adi_ad9081_device_t *device,
					       uint8_t rxen0_2f_ctrl_en,
					       uint8_t rxengp0_2f_ctrl_en,
					       uint8_t rxen0_2s_ctrl_en,
					       uint8_t rxengp0_2s_ctrl_en);

/***************************************************************************//**
 * @brief This function configures the power-down control settings for specific
 * ADC channels in the device. It must be called after the device has
 * been properly initialized. Each control parameter determines whether
 * the corresponding ADC channel is enabled or disabled, and they should
 * be set to either 0 (disabled) or 1 (enabled). If any of the parameters
 * are set to invalid values, the function will return an error. It is
 * important to ensure that the `device` pointer is not null before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param rxen1_3f_ctrl_en Control enable for ADC channel 1 in 3F mode. Valid
 * values are 0 or 1.
 * @param rxengp1_3f_ctrl_en Control enable for ADC channel GP1 in 3F mode.
 * Valid values are 0 or 1.
 * @param rxen1_3s_ctrl_en Control enable for ADC channel 1 in 3S mode. Valid
 * values are 0 or 1.
 * @param rxengp1_3s_ctrl_en Control enable for ADC channel GP1 in 3S mode.
 * Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * parameter is invalid or if an internal error occurs.
 ******************************************************************************/
int32_t adi_ad9081_adc_adc3_rxen_pwdn_ctrl_set(adi_ad9081_device_t *device,
					       uint8_t rxen1_3f_ctrl_en,
					       uint8_t rxengp1_3f_ctrl_en,
					       uint8_t rxen1_3s_ctrl_en,
					       uint8_t rxengp1_3s_ctrl_en);

/***************************************************************************//**
 * @brief This function is used to configure the receive engine control settings
 * of the ADC. It must be called with a valid `device` pointer that has
 * been properly initialized. The function sets the polarity, SPI enable
 * state, and receive enable state for the RX engine. If any of the
 * parameters are invalid or if the `device` pointer is null, the
 * function will return an error. It is important to ensure that the
 * device is ready for configuration before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param spi_en A `uint8_t` value that enables or disables the SPI interface
 * for the RX engine. Valid values are 0 (disable) or 1 (enable).
 * @param pol A `uint8_t` value that sets the polarity for the RX engine. Valid
 * values are implementation-specific, typically 0 or 1.
 * @param rxen A `uint8_t` value that enables or disables the RX engine. Valid
 * values are 0 (disable) or 1 (enable).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any of the
 * parameters are invalid or if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxengp0_ctrl_set(adi_ad9081_device_t *device,
					uint8_t spi_en, uint8_t pol,
					uint8_t rxen);

/***************************************************************************//**
 * @brief This function is used to configure the RX engine control settings of
 * the specified device. It must be called with a valid device pointer
 * that has been properly initialized. The function sets the SPI enable
 * state, polarity, and RX enable state based on the provided parameters.
 * If any of the parameters are invalid or if the device pointer is null,
 * the function will handle these cases by returning an appropriate error
 * code. It is important to ensure that the device is ready for
 * configuration before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be configured. Must not be null.
 * @param spi_en A `uint8_t` value indicating whether SPI is enabled (1) or
 * disabled (0). Valid values are 0 or 1.
 * @param pol A `uint8_t` value representing the polarity setting for the RX
 * engine. Valid values depend on the specific configuration
 * requirements.
 * @param rxen A `uint8_t` value indicating whether the RX engine is enabled (1)
 * or disabled (0). Valid values are 0 or 1.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while any other value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxengp1_ctrl_set(adi_ad9081_device_t *device,
					uint8_t spi_en, uint8_t pol,
					uint8_t rxen);

/***************************************************************************//**
 * @brief This function configures the selection settings for the ADC RX engine
 * of the device. It must be called with a valid `device` pointer that
 * has been properly initialized. The function sets various parameters
 * including the CDDC, FDDC, ADC, JTX, and JTX PHY selections. If any of
 * the parameters are invalid or if the device pointer is null, the
 * function will handle these cases by returning an error code. It is
 * important to ensure that the device is in a suitable state for
 * configuration before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A `uint8_t` value representing the CDDC selection. Valid values
 * depend on the specific configuration of the device.
 * @param fddcs A `uint8_t` value representing the FDDC selection. Valid values
 * depend on the specific configuration of the device.
 * @param adcs A `uint8_t` value representing the ADC selection. Valid values
 * depend on the specific configuration of the device.
 * @param jtx A `uint8_t` value representing the JTX selection. Valid values
 * depend on the specific configuration of the device.
 * @param jtx_phy A `uint8_t` value representing the JTX PHY selection. Valid
 * values depend on the specific configuration of the device.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxengp0_sel_set(adi_ad9081_device_t *device,
				       uint8_t cddcs, uint8_t fddcs,
				       uint8_t adcs, uint8_t jtx,
				       uint8_t jtx_phy);

/***************************************************************************//**
 * @brief This function configures the ADC RX engine parameters for a specified
 * device. It must be called after the device has been properly
 * initialized. The function sets various selection parameters, including
 * the CDDC, FDDC, ADC, JTX, and JTX PHY values. If any of the parameters
 * are invalid or if the device pointer is null, the function will handle
 * these cases by returning an error code. It is important to ensure that
 * the device is not null before calling this function to avoid
 * unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param cddcs A `uint8_t` value representing the CDDC selection. Valid values
 * depend on the specific configuration of the device.
 * @param fddcs A `uint8_t` value representing the FDDC selection. Valid values
 * depend on the specific configuration of the device.
 * @param adcs A `uint8_t` value representing the ADC selection. Valid values
 * depend on the specific configuration of the device.
 * @param jtx A `uint8_t` value representing the JTX selection. Valid values
 * depend on the specific configuration of the device.
 * @param jtx_phy A `uint8_t` value representing the JTX PHY selection. Valid
 * values depend on the specific configuration of the device.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxengp1_sel_set(adi_ad9081_device_t *device,
				       uint8_t cddcs, uint8_t fddcs,
				       uint8_t adcs, uint8_t jtx,
				       uint8_t jtx_phy);

/***************************************************************************//**
 * @brief This function is used to configure the RXEN0 control settings of an
 * ADC device. It must be called after the device has been properly
 * initialized. The function sets various parameters including the
 * transmit enable signal, the polarity of the RXEN0 signal, and the SPI
 * enable signal. It is important to ensure that the `device` pointer is
 * valid and not null before calling this function, as passing a null
 * pointer will result in an error. The function will return an error
 * code if any of the internal operations fail.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the ADC device. Must not be null.
 * @param use_txen A `uint8_t` value indicating whether to use the transmit
 * enable signal. Valid values are 0 or 1.
 * @param spi_en A `uint8_t` value indicating whether the SPI interface is
 * enabled. Valid values are 0 or 1.
 * @param rxen0_pol A `uint8_t` value representing the polarity of the RXEN0
 * signal. Valid values are 0 or 1.
 * @param rxen0 A `uint8_t` value indicating the state of the RXEN0 signal.
 * Valid values are 0 or 1.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxen0_ctrl_set(adi_ad9081_device_t *device,
				      uint8_t use_txen, uint8_t spi_en,
				      uint8_t rxen0_pol, uint8_t rxen0);

/***************************************************************************//**
 * @brief This function is used to set various control parameters for the RXEN1
 * functionality of the ADC. It must be called with a valid `device`
 * pointer that has been properly initialized. The function configures
 * the use of the TXEN signal, the SPI enable state, the polarity of the
 * RXEN1 signal, and the RXEN1 state itself. If any of the parameters are
 * invalid or if the `device` pointer is null, the function will return
 * an error. It is important to ensure that the device is ready for
 * configuration before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the ADC device. Must not be null.
 * @param use_txen A `uint8_t` indicating whether to use the TXEN signal (1 for
 * enabled, 0 for disabled). Valid values are 0 or 1.
 * @param spi_en A `uint8_t` indicating whether the SPI interface is enabled (1
 * for enabled, 0 for disabled). Valid values are 0 or 1.
 * @param rxen1_pol A `uint8_t` representing the polarity of the RXEN1 signal (1
 * for active high, 0 for active low). Valid values are 0 or 1.
 * @param rxen1 A `uint8_t` indicating the state of the RXEN1 signal (1 for
 * enabled, 0 for disabled). Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any of the
 * parameters are invalid or if the device pointer is null.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxen1_ctrl_set(adi_ad9081_device_t *device,
				      uint8_t use_txen, uint8_t spi_en,
				      uint8_t rxen1_pol, uint8_t rxen1);

/***************************************************************************//**
 * @brief This function configures the selection settings for the ADC RXEN0
 * based on the provided parameters. It must be called with a valid
 * `device` pointer that has been properly initialized. The function will
 * return an error if the `device` pointer is null or if any of the
 * selection settings fail to be applied. It is important to ensure that
 * the values for `cddcs`, `fddcs`, `adcs`, `jtx`, and `jtx_phy` are
 * within the expected range for the specific application, as invalid
 * values may lead to undefined behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs Selection value for the CDDC. Valid values depend on the
 * specific configuration of the device.
 * @param fddcs Selection value for the FDDC. Valid values depend on the
 * specific configuration of the device.
 * @param adcs Selection value for the ADC core. Valid values depend on the
 * specific configuration of the device.
 * @param jtx Selection value for JTX links. Valid values depend on the specific
 * configuration of the device.
 * @param jtx_phy Selection value for JTX PHY lanes. Valid values depend on the
 * specific configuration of the device.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any of the
 * selection settings could not be applied.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxen0_sel_set(adi_ad9081_device_t *device, uint8_t cddcs,
				     uint8_t fddcs, uint8_t adcs, uint8_t jtx,
				     uint8_t jtx_phy);

/***************************************************************************//**
 * @brief This function configures the selection parameters for the ADC RXEN1
 * interface of the device. It should be called after the device has been
 * properly initialized. The function takes multiple selection parameters
 * that determine the configuration of the ADC and its associated links.
 * If any of the input parameters are invalid or if the device pointer is
 * null, the function will handle these cases appropriately by returning
 * an error code. It is important to ensure that the device is not null
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cddcs A `uint8_t` value representing the selection for the CDDC. Valid
 * values depend on the specific configuration of the device.
 * @param fddcs A `uint8_t` value representing the selection for the FDDC. Valid
 * values depend on the specific configuration of the device.
 * @param adcs A `uint8_t` value representing the selection for the ADC core.
 * Valid values depend on the specific configuration of the device.
 * @param jtx A `uint8_t` value representing the selection for the JTX links.
 * Valid values are typically 0 or 1, corresponding to the enabled or
 * disabled state.
 * @param jtx_phy A `uint8_t` value representing the selection for the JTX
 * physical lanes. Valid values depend on the specific
 * configuration of the device.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_rxen1_sel_set(adi_ad9081_device_t *device, uint8_t cddcs,
				     uint8_t fddcs, uint8_t adcs, uint8_t jtx,
				     uint8_t jtx_phy);

/*===== 4 . 0   S E R D E S  L I N K  =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the current lock status of the JESD
 * PLL in the specified device. It should be called after the device has
 * been properly initialized. The function expects a valid pointer to a
 * `device` structure and a pointer to a `jesd_pll_status` variable where
 * the lock status will be stored. If the `device` pointer is null, the
 * function will return an error without modifying the `jesd_pll_status`.
 * It is important to ensure that the `jesd_pll_status` pointer is valid
 * and points to a location that can hold the status value.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param jesd_pll_status A pointer to a `uint8_t` variable where the lock
 * status will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the lock
 * status has been successfully retrieved. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_pll_lock_status_get(adi_ad9081_device_t *device,
					    uint8_t *jesd_pll_status);
/***************************************************************************//**
 * @brief This function is used to enable or disable specific JESD links on the
 * `adi_ad9081_device_t`. It should be called after the device has been
 * properly initialized. The `links` parameter allows selection of which
 * links to modify, and the `link_en` parameter determines whether to
 * enable (1) or disable (0) the selected links. If an invalid value is
 * provided for `link_en`, the function will return an error.
 * Additionally, if the `device` pointer is null, the function will also
 * return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which links to enable or disable. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param link_en A uint8_t value that specifies the enable state for the
 * selected links. Must be either 0 (disable) or 1 (enable). Any
 * other value will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or other issues.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_enable_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t link_en);
/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD RX link for a
 * specified device. It should be called after the device has been
 * properly initialized. The `links` parameter determines which link(s)
 * to modify, and the `link_en` parameter specifies whether to enable (1)
 * or disable (0) the link(s). If the `device` pointer is null, the
 * function will return an error. It is important to ensure that the
 * device is in a valid state before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link(s) to enable or disable. Valid
 * values depend on the specific implementation of the enumeration.
 * @param link_en A `uint8_t` value indicating the desired state of the link(s).
 * A value of 1 enables the link(s), while a value of 0 disables
 * them.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_link_enable_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t link_en);

/***************************************************************************//**
 * @brief This function is used to perform calibration of the JESD RX interface
 * for the specified device. It should be called after the device has
 * been properly initialized and configured. The function allows for a
 * forced calibration reset and can enable background calibration if
 * specified. It is important to ensure that the device is in a suitable
 * state for calibration, as the function checks the core status and may
 * wait for previous calibration tasks to complete. The calibration
 * process may vary depending on the device revision, and certain
 * parameters like `boost_mask` and `run_bg_cal` can influence the
 * calibration behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be calibrated. Must not be null.
 * @param force_cal_reset A flag indicating whether to force a calibration
 * reset. Valid values are 0 (no reset) and 1 (force
 * reset).
 * @param boost_mask A mask that specifies the equalizer boost settings. The
 * valid range depends on the device specifications.
 * @param run_bg_cal A flag indicating whether to run background calibration.
 * Valid values are 0 (do not run) and 1 (run background
 * calibration).
 * @return Returns an integer status code indicating the result of the
 * calibration operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while other values indicate various error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_calibrate_204c(adi_ad9081_device_t *device,
					  uint8_t force_cal_reset,
					  uint8_t boost_mask,
					  uint8_t run_bg_cal);

/***************************************************************************//**
 * @brief This function is used to configure and synchronize the NCO
 * (Numerically Controlled Oscillator) of the device in either master or
 * slave mode. It must be called after the device has been properly
 * initialized. The function sets various synchronization parameters,
 * including the trigger source and GPIO index, and it handles both
 * coarse and fine synchronization. It is important to ensure that the
 * `device` pointer is valid and not null before calling this function.
 * If any of the internal configuration steps fail, the function will
 * return an error code.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param is_master A flag indicating whether the device operates in master mode
 * (non-zero value) or slave mode (zero value). Valid values
 * are 0 or 1.
 * @param trigger_src An integer representing the trigger source for
 * synchronization. The valid range depends on the specific
 * implementation and should be defined in the documentation.
 * @param gpio_index An integer specifying the GPIO index to be used for
 * synchronization. The valid range is typically defined by
 * the hardware specifications.
 * @param extra_lmfc_num An integer representing an additional LMFC number for
 * synchronization. The valid range should be defined in
 * the hardware documentation.
 * @return Returns an error code indicating the success or failure of the
 * synchronization process. A return value of `API_CMS_ERROR_OK`
 * indicates successful execution.
 ******************************************************************************/
int32_t adi_ad9081_adc_nco_master_slave_sync(adi_ad9081_device_t *device,
					     uint8_t is_master,
					     uint8_t trigger_src,
					     uint8_t gpio_index,
					     uint8_t extra_lmfc_num);

/***************************************************************************//**
 * @brief This function is used to pause the background calibration process of
 * the device. It should be called when the calibration needs to be
 * temporarily halted, typically during system adjustments or
 * maintenance. The function checks the device's revision to determine
 * the appropriate register address for the operation. It will wait for a
 * maximum of 50 iterations to confirm that the receiver is idle before
 * successfully pausing the calibration. If the receiver does not become
 * idle within this timeframe, an error is logged, and the function
 * returns an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates that the
 * calibration was successfully paused, while other values indicate
 * different error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_cal_bg_cal_pause(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function initiates the background calibration process for the
 * JESD interface of the specified device. It must be called with a valid
 * `device` pointer that has been properly initialized. If the device
 * pointer is null, the function will return an error. The function sets
 * the appropriate register address based on the device revision and
 * triggers the calibration process. It is important to ensure that the
 * device is in a state that allows calibration before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null. If the pointer is null, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful initiation of the
 * calibration process. If an error occurs during the operation, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_cal_bg_cal_start(adi_ad9081_device_t *device);

/*===== 4 . 1   S E R D E S  R E C E I V E R  L I N K  =====*/
/***************************************************************************//**
 * @brief This function is used to power down the JESD RX subsystem of the
 * device. It should be called when the JESD RX functionality is no
 * longer needed, such as during device shutdown or when switching to a
 * different operational mode. The function expects a valid device
 * pointer and will return an error if the pointer is null. It performs
 * several operations to ensure that the RX subsystem is properly powered
 * down, and it is important to check the return value to confirm that
 * the operation was successful.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, indicating that
 * the JESD RX subsystem has been powered down. If an error occurs
 * during the operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_power_down_des(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to obtain the validity status of the JESD RX
 * configuration for a specified device. It must be called after the
 * device has been properly initialized. The function checks for null
 * pointers and will return an error if either the `device` or
 * `cfg_valid` parameters are null. The `cfg_valid` parameter will be
 * updated to indicate whether the configuration is valid or not. It is
 * important to ensure that the device is ready for configuration status
 * retrieval before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param cfg_valid A pointer to a `uint8_t` variable where the configuration
 * validity status will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * configuration status was retrieved successfully.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_config_status_get(adi_ad9081_device_t *device,
					     uint8_t *cfg_valid);

/***************************************************************************//**
 * @brief This function configures the synchronization mode for one or more JESD
 * links on the specified device. It must be called after the device has
 * been properly initialized. The `links` parameter determines which JESD
 * links are affected, and the `sync_mode` parameter specifies the
 * desired synchronization mode. If an invalid `device` pointer is
 * provided, the function will return an error. Additionally, if the
 * specified links are not valid or if there are issues setting the
 * synchronization mode, appropriate error codes will be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`. The function will only configure the links
 * specified in this bitmask.
 * @param sync_mode An 8-bit unsigned integer representing the synchronization
 * mode to be set. The valid range of values depends on the
 * specific implementation and should be defined in the
 * relevant documentation.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_sync_mode_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t sync_mode);

/***************************************************************************//**
 * @brief This function is used to set up the JESD RX link configuration for a
 * specified device. It must be called after the device has been properly
 * initialized. The function takes into account the parameters provided
 * in `jesd_param`, which dictate the specific JESD204 settings,
 * including link mode and dual link configuration. It is important to
 * ensure that the `device` and `jesd_param` pointers are not null before
 * calling this function, as it will return an error if they are. The
 * function also handles various configurations based on the JESD version
 * specified in `jesd_param`, and it may produce warnings if the
 * configuration is not found in the expected table.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD links to configure. Valid values
 * depend on the specific implementation.
 * @param jesd_param A pointer to an `adi_cms_jesd_param_t` structure containing
 * the JESD configuration parameters. Must not be null. The
 * structure must be properly initialized with valid values
 * before calling this function.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * successful configuration.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_link_config_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   adi_cms_jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function is used to initialize the JESD RX interface for the
 * specified device. It must be called after the device has been properly
 * initialized and configured. The function sets up the lane crossbar,
 * powers up the necessary components, calculates the bit rate, and
 * starts the JESD PLL and deserializer. It is important to ensure that
 * the `device` pointer is valid and not null before calling this
 * function. If any of the operations fail, the function will return an
 * error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD links to bring up. Valid values depend
 * on the specific implementation.
 * @param lanes An 8-bit unsigned integer representing the number of lanes to be
 * used. Valid values are typically between 1 and 8, depending on
 * the device capabilities.
 * @return Returns an error code of type `int32_t`. A return value of
 * `API_CMS_ERROR_OK` indicates success, while any other value indicates
 * an error occurred during the operation.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_bring_up(adi_ad9081_device_t *device,
				    adi_ad9081_jesd_link_select_e links,
				    uint8_t lanes);

/***************************************************************************//**
 * @brief This function is used to configure the JESD RX link selection for a
 * specified device. It must be called with a valid device pointer that
 * has been properly initialized. If the device pointer is null, the
 * function will return an error without making any changes. The `links`
 * parameter specifies which JESD link to select and must be a valid
 * value from the `adi_ad9081_jesd_link_select_e` enumeration. The
 * function will return an error if the link selection operation fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error is returned.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies the desired JESD RX link. Must be a valid
 * selection from the enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_link_select_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links);

/***************************************************************************//**
 * @brief This function configures the crossbar mapping for the JESD RX lanes of
 * the specified device. It should be called after the device has been
 * properly initialized and before any data transfer occurs. The
 * `logical_lanes` array must contain valid lane mappings for each of the
 * 8 lanes, and the function will return an error if any of the mappings
 * are invalid. It is important to ensure that the `device` pointer is
 * not null before calling this function, as it will result in an
 * immediate error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to configure. Valid values are
 * defined in the enumeration.
 * @param logical_lanes An array of 8 `uint8_t` values representing the logical
 * lane mappings. Each value should be within the valid
 * range for lane indices. The caller retains ownership of
 * this array.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any of the
 * lane mappings are invalid or if the device pointer is null.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_lanes_xbar_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t logical_lanes[8]);

/***************************************************************************//**
 * @brief This function configures the mapping between physical and logical
 * lanes for the JESD RX interface of the specified device. It must be
 * called after the device has been properly initialized. The function
 * allows for the selection of one or more JESD links, and it will set
 * the specified physical lane to the corresponding logical lane. If the
 * provided lane numbers exceed their valid ranges, the function will
 * return an error. It is important to ensure that the `device` pointer
 * is not null before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param physical_lane An 8-bit unsigned integer representing the physical lane
 * number. Valid values are 0 to 7.
 * @param logical_lane An 8-bit unsigned integer representing the logical lane
 * number. Valid values are 0 to 7.
 * @return Returns `API_CMS_ERROR_OK` on success. If any parameter is invalid or
 * if an error occurs during the configuration, an appropriate error
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_lane_xbar_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t physical_lane,
					 uint8_t logical_lane);

/***************************************************************************//**
 * @brief This function configures the descrambler for the specified JESD links
 * on the given device. It must be called after the device has been
 * properly initialized. The `links` parameter allows selection of one or
 * both JESD links, while the `dsr_en` parameter enables or disables the
 * descrambler. If an invalid link is specified or if `dsr_en` is not 0
 * or 1, the function will return an error. It is important to ensure
 * that the device pointer is not null before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`,
 * `AD9081_LINK_1`, or both.
 * @param dsr_en A `uint8_t` value that enables (1) or disables (0) the
 * descrambler. Must be either 0 or 1; otherwise, the function
 * will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or device state.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_descrambler_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t dsr_en);

/***************************************************************************//**
 * @brief This function is used to configure the inversion state of a specified
 * logical lane for JESD RX links. It must be called with a valid
 * `device` pointer that has been properly initialized. The `links`
 * parameter specifies which JESD link(s) to configure, and at least one
 * link must be selected. The `logical_lane` parameter must be in the
 * range of 0 to 7, representing the logical lanes available. The
 * `invert_en` parameter determines whether the lane inversion is enabled
 * (1) or disabled (0). If any parameter is invalid, the function will
 * return an error code without making any changes.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to configure. Must be a
 * valid combination of `adi_ad9081_jesd_link_select_e` values.
 * @param logical_lane The logical lane number to configure, which must be in
 * the range of 0 to 7. If out of range, the function will
 * return an error.
 * @param invert_en A flag indicating whether to enable (1) or disable (0) lane
 * inversion. Must be either 0 or 1; otherwise, an error will
 * be returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any input
 * parameters are invalid or if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_lane_invert_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t logical_lane,
					   uint8_t invert_en);

/***************************************************************************//**
 * @brief This function configures the synchronization mode for the JESD RX
 * SYNCA signal in the specified device. It should be called after the
 * device has been properly initialized. The `mode` parameter determines
 * the synchronization type, which can be either CMOS or LVDS. If the
 * provided `device` pointer is null, the function will return an error.
 * It is important to ensure that the `mode` parameter is either 0 or 1,
 * as any other value may lead to undefined behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mode An 8-bit unsigned integer representing the synchronization mode.
 * Valid values are 0 for CMOS and 1 for LVDS. Any other value may
 * result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * synchronization mode has been set successfully. If an error occurs, a
 * negative error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_synca_mode_set(adi_ad9081_device_t *device,
					  uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the synchronization mode for the JESD RX
 * interface of the specified device. It must be called with a valid
 * device pointer that has been properly initialized. The `mode`
 * parameter determines the synchronization signaling type, where a value
 * of 0 sets the mode to CMOS and a value of 1 sets it to LVDS. If the
 * provided device pointer is null, the function will return an error
 * without making any changes. It is important to ensure that the device
 * is in a state that allows for mode changes before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mode An 8-bit unsigned integer specifying the synchronization mode.
 * Valid values are 0 for CMOS and 1 for LVDS. Any other value will
 * be treated as invalid.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the
 * synchronization mode has been set successfully. If an error occurs,
 * an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_syncb_mode_set(adi_ad9081_device_t *device,
					  uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to control the power state of the JESD RX SYNC A
 * driver in the specified device. It should be called when there is a
 * need to power down the driver, typically during device shutdown or
 * when the driver is not in use. The function expects a valid device
 * pointer and will return an error if the pointer is null. The powerdown
 * parameter determines whether the driver is powered down (1) or powered
 * up (0). It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; the function will return an error
 * if it is.
 * @param powerdown A `uint8_t` value indicating the desired power state of the
 * JESD RX SYNC A driver. A value of 1 powers down the driver,
 * while a value of 0 powers it up.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the power
 * state has been set successfully. If an error occurs, an appropriate
 * error code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_synca_driver_powerdown_set(adi_ad9081_device_t *device,
					      uint8_t powerdown);

/***************************************************************************//**
 * @brief This function is used to control the power state of the JESD RX syncb
 * driver in the `adi_ad9081_device_t`. It should be called when there is
 * a need to power down the syncb driver, typically during device
 * shutdown or when the driver is not in use. The function expects a
 * valid device pointer and will return an error if the pointer is null.
 * It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @param powerdown A `uint8_t` value indicating the desired power down state.
 * Valid values are typically 0 (power up) or 1 (power down).
 * The function will handle invalid values by treating them as
 * valid inputs without specific error handling.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the power down
 * state has been set successfully. If an error occurs during the
 * operation, an appropriate error code will be returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_syncb_driver_powerdown_set(adi_ad9081_device_t *device,
					      uint8_t powerdown);

/***************************************************************************//**
 * @brief This function configures the LMFC (Lane Multi-Frame Clock) delay for
 * one or more JESD links associated with the specified device. It must
 * be called after the device has been properly initialized. The function
 * allows for setting the delay for either link 0 or link 1, or both,
 * depending on the provided `links` parameter. If an invalid `device`
 * pointer is passed, the function will return an error. Additionally, if
 * the specified links are not valid or if the delay value is out of
 * acceptable range, the function will handle these cases by returning an
 * appropriate error code.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`,
 * `AD9081_LINK_1`, or a combination of both.
 * @param delay A 16-bit unsigned integer representing the LMFC delay to be set.
 * The valid range for this value should be defined by the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_lmfc_delay_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint16_t delay);

/***************************************************************************//**
 * @brief This function configures the RX run calibration mask for the specified
 * AD9081 device. It must be called after the device has been properly
 * initialized. The function checks the device revision and applies the
 * mask accordingly. If the device pointer is null, the function will
 * return an error. It is important to ensure that the mask value is
 * valid for the device revision being used.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mask A `uint8_t` value representing the calibration mask to be set.
 * Valid values depend on the device's specifications.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_run_cal_mask_set(adi_ad9081_device_t *device,
					    uint8_t mask);

/***************************************************************************//**
 * @brief This function configures the RX boost mask for the specified AD9081
 * device, which affects the gain settings of the receiver. It must be
 * called after the device has been properly initialized and is ready for
 * configuration. The function checks the device revision to determine
 * the appropriate register to modify. If the provided `device` pointer
 * is null, the function will return an error. It is important to ensure
 * that the `mask` value is within the valid range for the device's RX
 * boost settings.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error is returned.
 * @param mask A `uint8_t` value representing the RX boost mask to be set. The
 * valid range of this mask depends on the device specifications;
 * invalid values may lead to undefined behavior or errors.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_boost_mask_set(adi_ad9081_device_t *device,
					  uint8_t mask);

/***************************************************************************//**
 * @brief This function is used to obtain the current link status of the JESD RX
 * interface for a specified device. It must be called after the device
 * has been properly initialized. The function checks the status of
 * multiple links and updates the provided status pointer with a bitmask
 * indicating the link states. If any of the links are not operational,
 * the corresponding bits in the status will be cleared. It is important
 * to ensure that the `device` and `status` parameters are not null
 * before calling this function, as it will return an error if they are.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which links to check. Valid values depend on the
 * specific implementation.
 * @param status A pointer to a `uint16_t` where the link status will be stored.
 * Must not be null; the function will return an error if it is.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while any other value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_link_status_get(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint16_t *status);

/***************************************************************************//**
 * @brief This function is used to set the Continuous Time Linear Equalizer
 * (CTLE) configuration for the JESD receiver. It must be called with a
 * valid `device` pointer that has been properly initialized. The `lanes`
 * parameter specifies which lanes to configure, and the `il_db`
 * parameter determines the insertion loss in decibels. If `il_db` is
 * below a certain threshold, different register values are set compared
 * to when it is above that threshold. The function also toggles the link
 * enable, which is necessary for certain silicon revisions. It is
 * important to handle any potential errors returned by the function, as
 * it will return an error code if any of the register settings fail.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param lanes A `uint8_t` value indicating which lanes to configure. Valid
 * values depend on the specific hardware configuration.
 * @param il_db A `uint8_t` value representing the insertion loss in decibels.
 * Must be within the valid range defined by the hardware
 * specifications.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_ctle_config_set(adi_ad9081_device_t *device,
					   uint8_t lanes, uint8_t il_db);

/***************************************************************************//**
 * @brief This function configures the GPIO pin to operate as a SYNC1 output for
 * the specified device. It should be called after the device has been
 * properly initialized. The `mode` parameter determines the specific
 * SYNC1 output configuration, with valid values being 0 for `link1_sync`
 * and 1 for `link1_sync with diff mode`. If an invalid `mode` is
 * provided or if the `device` pointer is null, the function will return
 * an error. It is important to ensure that the device is not null before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mode An 8-bit unsigned integer that specifies the SYNC1 output
 * configuration. Valid values are 0 and 1. If the value is greater
 * than 1, the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the GPIO
 * configuration was set successfully. If an error occurs, a negative
 * error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_gpio_as_sync1_out_set(adi_ad9081_device_t *device,
					     uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to obtain the manual configuration settings for
 * the Continuous Time Linear Equalizer (CTLE) associated with a specific
 * lane of the device. It must be called after the device has been
 * properly initialized and configured. The function retrieves the CTLE
 * coefficients for the specified lane and stores them in the device's
 * internal structure. If the provided `device` pointer is null or if an
 * error occurs during the retrieval of the coefficients, the function
 * will handle these cases appropriately, ensuring that the caller is
 * informed of any issues.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lane An 8-bit unsigned integer representing the lane number for which
 * the CTLE settings are to be retrieved. Valid values are typically
 * in the range of 0 to the maximum number of lanes supported by the
 * device.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while other values indicate specific error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_ctle_manual_config_get(adi_ad9081_device_t *device,
						  uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to configure the Continuous Time Linear
 * Equalizer (CTLE) settings for a specific lane of the JESD RX
 * interface. It must be called after the device has been properly
 * initialized and configured. The function expects a valid `device`
 * pointer and a valid `lane` number. If the `lane` is out of range or if
 * the `device` pointer is null, the function will handle these errors
 * gracefully. It is important to ensure that the CTLE coefficients for
 * the specified lane are set correctly in the device's settings before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lane An 8-bit unsigned integer representing the lane number to
 * configure. Valid values are typically in the range of 0 to 3,
 * depending on the device's configuration. If the value is out of
 * range, the function will handle the error.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration. If an error
 * occurs during the configuration process, an appropriate error code
 * will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_ctle_manual_config_set(adi_ad9081_device_t *device,
						  uint8_t lane);

/*===== 4 . 2   S E R D E S  T R A N S M I T T E R  L I N K  =====*/
/***************************************************************************//**
 * @brief This function is used to power down the JESD transmitter in the
 * specified device. It should be called when the transmitter is no
 * longer needed, such as during device shutdown or when switching to a
 * different operational mode. Before calling this function, ensure that
 * the `device` parameter is properly initialized and not null. If the
 * function encounters a null pointer or any internal error during
 * execution, it will return an error code, indicating the failure.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, or an error code
 * if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_power_down_ser(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to obtain the current status of the JESD204B
 * transmitter link for a specified device. It must be called after the
 * device has been properly initialized and configured. The function
 * checks various internal states and registers to compile the link
 * status, which is then returned through the provided status pointer. It
 * is important to ensure that both the device and status parameters are
 * valid and not null before calling this function, as passing null
 * pointers will result in an error. Additionally, the function may
 * return an error code if any internal operations fail.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD204B link to query. Valid values depend
 * on the specific implementation.
 * @param status A pointer to a `uint16_t` where the link status will be stored.
 * Must not be null.
 * @return Returns an integer error code indicating the success or failure of
 * the operation. On success, the `status` parameter is populated with
 * the current link status.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_status_get(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint16_t *status);

/***************************************************************************//**
 * @brief This function configures the synchronization mode for one or more JESD
 * links on the specified device. It must be called after the device has
 * been properly initialized and before any data transmission occurs. The
 * function checks which links are selected and applies the
 * synchronization mode accordingly. If an invalid device pointer is
 * provided, or if there are errors during the link selection or register
 * setting, the function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`. The function will only configure the links
 * that are set in this bitmask.
 * @param sync_mode An 8-bit unsigned integer representing the desired
 * synchronization mode. The valid range of values depends on
 * the specific synchronization modes supported by the device.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_sync_mode_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t sync_mode);

/***************************************************************************//**
 * @brief This function is used to configure the JESD204B transmitter link
 * settings for a specified device. It must be called after the device
 * has been properly initialized and before any data transmission occurs.
 * The function takes a pointer to the device structure, a selection of
 * links to configure, and a pointer to a structure containing JESD204B
 * parameters. It is important to ensure that the provided parameters are
 * valid; otherwise, the function may return an error. The function also
 * handles various internal configurations and may affect the state of
 * the device, including enabling or disabling specific links.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device to configure. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD links to configure. Valid values
 * depend on the specific device capabilities.
 * @param jesd_param A pointer to an array of `adi_cms_jesd_param_t` structures
 * containing the JESD204B configuration parameters. Must not
 * be null and should contain valid settings for the specified
 * links.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * configuration, while other values indicate specific error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_config_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   adi_cms_jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function is used to initialize and bring up the JESD204 interface
 * for a specified device. It must be called after the device has been
 * properly initialized. The function configures the JESD204 links and
 * lanes based on the provided parameters. If any of the input parameters
 * are invalid, appropriate error handling will occur, ensuring that the
 * function does not proceed with invalid configurations.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD204 links to bring up. Valid values are
 * defined in the enumeration.
 * @param lanes An 8-bit unsigned integer representing the number of lanes to be
 * used. Valid values depend on the device specifications.
 * @param jesd_conv_sel An array of two `adi_ad9081_jtx_conv_sel_t` values that
 * specify the converter selection for the JESD204
 * interface. The caller retains ownership of this array.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates that
 * the JESD204 interface was successfully brought up.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_bring_up(adi_ad9081_device_t *device,
				    adi_ad9081_jesd_link_select_e links,
				    uint8_t lanes,
				    adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2]);

/***************************************************************************//**
 * @brief This function is used to configure which JESD link will be used for
 * transmission in the `adi_ad9081_device_t`. It must be called after the
 * device has been properly initialized. The function checks for a null
 * pointer for the `device` parameter and will return an error if the
 * pointer is invalid. It is important to ensure that the `links`
 * parameter corresponds to a valid JESD link selection, as invalid
 * values may lead to errors.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; the caller retains ownership.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies the JESD link to select. Must be a valid link
 * selection; invalid values may result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the link
 * selection was set successfully. If an error occurs, a negative error
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_select_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links);

/***************************************************************************//**
 * @brief This function configures the crossbar mapping for the JESD TX lanes of
 * the specified device. It should be called after the device has been
 * properly initialized and before any data transmission occurs. The
 * function takes an array of logical lane mappings, which must be
 * provided for all 8 lanes. If any of the parameters are invalid,
 * appropriate error codes will be returned, and no changes will be made
 * to the device configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to configure. Valid values are
 * defined in the enumeration.
 * @param logical_lanes An array of 8 `uint8_t` values representing the logical
 * lane mappings. The caller retains ownership of this
 * array, and it must not be null.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while other values indicate specific error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lanes_xbar_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t logical_lanes[8]);

/***************************************************************************//**
 * @brief This function is used to configure the mapping between logical and
 * physical lanes for JESD transmission in the specified device. It must
 * be called with a valid `device` pointer that has been properly
 * initialized. The `links` parameter specifies which JESD links to
 * configure, and at least one of the specified links must be valid. The
 * `physical_lane` and `logical_lane` parameters must be in the range of
 * 0 to 7. If any of these parameters are invalid, the function will
 * return an error. This function may have side effects on the device's
 * configuration, so it should be used with caution.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * indicating which JESD links to configure. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param physical_lane An 8-bit unsigned integer representing the physical lane
 * number. Must be in the range of 0 to 7.
 * @param logical_lane An 8-bit unsigned integer representing the logical lane
 * number. Must be in the range of 0 to 7.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if any
 * parameter is invalid or if an internal operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lane_xbar_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t physical_lane,
					 uint8_t logical_lane);

/***************************************************************************//**
 * @brief This function configures the scrambler for the specified JESD links on
 * the given device. It must be called after the device has been properly
 * initialized. The `links` parameter determines which JESD links will
 * have their scrambler settings applied. The `scr_en` parameter enables
 * or disables the scrambler; it should be either 0 (disabled) or 1
 * (enabled). If an invalid `scr_en` value is provided, the function will
 * return an error. Additionally, if the `device` pointer is null, the
 * function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param scr_en A uint8_t value that enables (1) or disables (0) the scrambler.
 * Must be either 0 or 1; any other value will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_scrambler_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t scr_en);

/***************************************************************************//**
 * @brief This function is used to configure the inversion state of a physical
 * lane in the JESD transmitter of the `adi_ad9081_device_t`. It must be
 * called after the device has been properly initialized. The function
 * allows the user to specify which JESD link to configure and whether to
 * enable or disable lane inversion. It is important to ensure that the
 * `physical_lane` parameter is within the valid range of 0 to 7 and that
 * the `invert_en` parameter is either 0 or 1. If invalid parameters are
 * provided, the function will return an error without making any
 * changes.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param physical_lane An integer representing the physical lane number to
 * configure. Valid values are from 0 to 7. If the value is
 * greater than 7, an error is returned.
 * @param invert_en An integer indicating whether to enable (1) or disable (0)
 * lane inversion. Must be either 0 or 1; otherwise, an error
 * is returned.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs due to
 * invalid parameters or other issues, an appropriate error code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lane_invert_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t physical_lane,
					   uint8_t invert_en);

/***************************************************************************//**
 * @brief This function is used to initialize the JESD transmitter for the
 * specified device. It must be called after the device has been properly
 * initialized and configured. The function toggles power down bits, sets
 * drive slice offsets, configures swing and equalization settings for
 * each lane, and resets the PHY. It is important to ensure that the
 * `lanes` parameter is within the valid range to avoid undefined
 * behavior. The function includes delays to allow for hardware
 * stabilization, and it will return an error code if any operation
 * fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lanes A bitmask indicating which lanes to power up. Valid values are
 * between 0 and 0xFF, where each bit represents a lane. The
 * function will handle invalid values by returning an error.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * startup.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_startup_ser(adi_ad9081_device_t *device,
				       uint8_t lanes);

/***************************************************************************//**
 * @brief This function is used to configure the power-down state of a JESD TX
 * lane in the AD9081 device. It should be called after the device has
 * been properly initialized. The `physical_lane` parameter specifies
 * which lane to configure, and it must be in the range of 0 to 7. The
 * `power_down` parameter indicates whether to power down the lane (1) or
 * keep it active (0). If either parameter is invalid, the function will
 * return an error. It is important to ensure that the `device` pointer
 * is not null before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param physical_lane An 8-bit unsigned integer representing the lane number
 * to configure. Valid values are 0 to 7.
 * @param power_down An 8-bit unsigned integer indicating the power state. Valid
 * values are 0 (active) or 1 (power down).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lane_force_pd_set(adi_ad9081_device_t *device,
					     uint8_t physical_lane,
					     uint8_t power_down);

/***************************************************************************//**
 * @brief This function is used to configure the converter selection for a
 * specified JESD link on the `adi_ad9081_device_t`. It must be called
 * after the device has been properly initialized. The function takes a
 * link selection and a converter index, and it updates the converter
 * selection value accordingly. If the provided `conv_index` is greater
 * than 15, the function will return an error. Additionally, it updates
 * the chip decimation based on the current configuration of the selected
 * link. It is important to ensure that the `device` pointer is not null
 * before calling this function.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to configure. Valid values
 * are `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param conv_index Index of the converter to be set, which must be in the
 * range of 0 to 15.
 * @param val Value to set for the converter selection. The valid range for this
 * value is implementation-specific.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_conv_sel_set(adi_ad9081_device_t *device,
					adi_ad9081_jesd_link_select_e links,
					uint8_t conv_index, uint8_t val);

/***************************************************************************//**
 * @brief This function configures the selection of control bits for the JESD
 * transmitter associated with the specified device. It must be called
 * after the device has been properly initialized and configured. The
 * function allows the user to specify which bits to select for two
 * different JESD links, and it will only apply the settings for the
 * links that are enabled. If an invalid device pointer is provided, the
 * function will return an error. Additionally, if any of the link
 * selections or bit selections fail, the function will return an error
 * code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * indicating which JESD links to configure. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param bit0_sel A `uint8_t` value representing the selection for control bit
 * 0. Valid values depend on the specific configuration of the
 * device.
 * @param bit1_sel A `uint8_t` value representing the selection for control bit
 * 1. Valid values depend on the specific configuration of the
 * device.
 * @param bit2_sel A `uint8_t` value representing the selection for control bit
 * 2. Valid values depend on the specific configuration of the
 * device.
 * @return Returns an `int32_t` error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_ctrl_bit_sel_set(adi_ad9081_device_t *device,
					    adi_ad9081_jesd_link_select_e links,
					    uint8_t bit0_sel, uint8_t bit1_sel,
					    uint8_t bit2_sel);

/***************************************************************************//**
 * @brief This function configures the output format for the JESD links of the
 * specified device. It must be called after the device has been properly
 * initialized. The function allows selection of multiple links, and the
 * format can be set to either 2's complement, offset binary, or gray
 * code. If an invalid link is specified or if the device pointer is
 * null, the function will handle these cases gracefully by returning an
 * error. It is important to ensure that the format value is within the
 * valid range before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of `adi_ad9081_jesd_link_select_e` values indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param format An 8-bit unsigned integer representing the desired output
 * format. Valid values are 0 for 2's complement, 1 for offset
 * binary, and 2 for gray code.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_format_sel_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t format);

/***************************************************************************//**
 * @brief This function configures the resolution of the JESD transmitter for
 * the specified links on the device. It must be called after the device
 * has been properly initialized. The `links` parameter determines which
 * JESD links are affected, and the `resolution` parameter specifies the
 * desired resolution, which must be between 8 and 16 bits inclusive. If
 * an invalid resolution is provided or if the device pointer is null,
 * the function will return an error. The function will apply the
 * resolution setting to each selected link.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param resolution An 8-bit unsigned integer representing the desired
 * resolution in bits. Valid values are between 8 and 16
 * inclusive. If the value is outside this range, an error
 * will be returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_res_sel_set(adi_ad9081_device_t *device,
				       adi_ad9081_jesd_link_select_e links,
				       uint8_t resolution);

/***************************************************************************//**
 * @brief This function configures the fractional delay converter selection for
 * the specified JESD links on the device. It must be called after the
 * device has been properly initialized. The `links` parameter determines
 * which JESD links are affected, and the `converters` parameter
 * specifies the converter selection. If an invalid pointer is provided
 * for the `device`, the function will return an error. Additionally, if
 * the specified links are not valid or if there are issues setting the
 * link selection or the fractional delay, appropriate error codes will
 * be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are combinations of
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param converters A 16-bit unsigned integer representing the selection of
 * fractional delay converters. The valid range is determined
 * by the device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_fractional_delay_converter_selection_set(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	uint16_t converters);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD TX conversion test
 * mode for specified links on the device. It must be called with a valid
 * `device` pointer that has been properly initialized. The `links`
 * parameter allows selection of one or more JESD links to configure, and
 * the `enable` parameter determines whether to enable (non-zero value)
 * or disable (zero value) the test mode. If an invalid link is specified
 * or if the `device` pointer is null, the function will handle these
 * cases gracefully by returning an error. It is important to ensure that
 * the device is in a state that allows configuration changes when this
 * function is called.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask of `adi_ad9081_jesd_link_select_e` values indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`. Multiple links can be selected using
 * bitwise OR.
 * @param enable A 16-bit integer where a non-zero value enables the test mode
 * and zero disables it. No specific range is enforced, but only
 * the zero and non-zero values are meaningful.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_conv_test_mode_enable_set(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	uint16_t enable);

/***************************************************************************//**
 * @brief This function configures the LID (Link Identifier) for the specified
 * lanes of the JESD transmitter links. It must be called with a valid
 * `device` pointer that has been properly initialized. The `links`
 * parameter determines which JESD links are affected, and the `lane`
 * parameter specifies which lane's LID configuration to set. Valid lane
 * values are from 0 to 7. If an invalid lane is provided or if the
 * `device` pointer is null, the function will return an error. This
 * function should be used when configuring the JESD interface to ensure
 * proper communication.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to configure. Valid values
 * are `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param lane The lane number for which to set the LID configuration. Valid
 * values are 0 to 7. If the value is greater than 7, an error is
 * returned.
 * @param val The value to set for the LID configuration. Must be within the
 * valid range defined by the hardware specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lid_cfg_set(adi_ad9081_device_t *device,
				       adi_ad9081_jesd_link_select_e links,
				       uint8_t lane, uint8_t val);

/***************************************************************************//**
 * @brief This function configures the LIDs (Logical Identifiers) for the JESD
 * transmitter associated with the specified device. It must be called
 * after the device has been properly initialized and configured. The
 * function expects an array of 8 LIDs, which are used to identify the
 * data streams. If any of the provided LIDs are invalid or if the device
 * pointer is null, the function will handle these cases appropriately by
 * returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to configure. Valid values are
 * defined in the enumeration.
 * @param lids An array of 8 `uint8_t` values representing the LIDs to be
 * configured. The caller retains ownership of this array, and it
 * must not be null.
 * @return Returns an integer error code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * successful configuration.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lids_cfg_set(adi_ad9081_device_t *device,
					adi_ad9081_jesd_link_select_e links,
					uint8_t lids[8]);

/***************************************************************************//**
 * @brief This function is used to reset the JESD204 transmit link of the
 * specified device. It should be called when a reset of the link is
 * necessary, such as during initialization or recovery from an error
 * state. The function expects a valid device pointer and a reset
 * parameter that indicates whether to perform the reset (1) or not (0).
 * If the device pointer is null or the reset parameter is invalid, the
 * function will handle these cases appropriately by returning an error.
 * It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param reset An 8-bit unsigned integer indicating the reset state. Valid
 * values are 0 (no reset) and 1 (perform reset). Any value outside
 * this range is considered invalid.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the reset
 * operation was initiated successfully. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_reset(adi_ad9081_device_t *device,
				      uint8_t reset);

/***************************************************************************//**
 * @brief This function is used to control the on-chip termination feature for
 * the SYNC A signal in the `adi_ad9081_device_t`. It should be called
 * after the device has been properly initialized. The `enable` parameter
 * determines whether the termination is activated or deactivated. If the
 * `device` pointer is null, the function will return an error without
 * making any changes. It is important to ensure that the device is in a
 * valid state before calling this function to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the on-chip termination. Valid values are 0
 * (disable) and any non-zero value (enable).
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_synca_onchip_term_enable(adi_ad9081_device_t *device,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to control the on-chip termination feature for
 * the SYNCB signal in the `adi_ad9081_device_t`. It should be called
 * after the device has been properly initialized. The `enable` parameter
 * determines whether the termination is activated or deactivated. If the
 * `device` pointer is null, the function will return an error without
 * making any changes. It is important to ensure that the device is in a
 * valid state before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the on-chip termination. Valid values are 0
 * (disable) and any non-zero value (enable).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_syncb_onchip_term_enable(adi_ad9081_device_t *device,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to force a digital reset on one or both JESD
 * links of the specified device. It should be called when a reset of the
 * digital link is required, typically during initialization or recovery
 * from an error state. The function expects a valid device pointer and
 * will return an error if the pointer is null. The reset operation is
 * performed on the specified links, and a delay of 100 milliseconds is
 * introduced after the reset to ensure the operation completes. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to reset. Valid values are `AD9081_LINK_0` and
 * `AD9081_LINK_1`. Multiple links can be selected using bitwise
 * OR.
 * @param reset A `uint8_t` value indicating the reset state. A value of 1
 * forces a reset, while 0 disables the reset.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_tx_force_digital_reset_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t reset);

/***************************************************************************//**
 * @brief This function configures the LMFC (Lane Multi-Frame Clock) delay for
 * one or more JESD links associated with the specified device. It must
 * be called after the device has been properly initialized. The `links`
 * parameter allows the selection of which JESD links to configure, and
 * the `delay` parameter specifies the desired delay value. If an invalid
 * `device` pointer is provided, the function will return an error.
 * Additionally, if the specified links are not valid or if there are
 * issues setting the delay, appropriate error codes will be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`. The function will only configure the links
 * that are specified in this bitmask.
 * @param delay An unsigned 16-bit integer representing the LMFC delay to be
 * set. The valid range for this value is implementation-specific,
 * and the function will handle out-of-range values by returning an
 * error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure if the operation could not be completed.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_lmfc_delay_set(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint16_t delay);

/*=====    A P P E N D I X  =====*/
/***************************************************************************//**
 * @brief This function is used to obtain the maximum and minimum temperature
 * readings from the specified device. It should be called after the
 * device has been properly initialized. The function checks the device
 * revision; if the revision is not supported, a warning is logged, and
 * the temperature values are not retrieved. The caller must ensure that
 * the `device` pointer is valid and not null, as passing a null pointer
 * will result in an immediate return with an error. The function
 * populates the `max` and `min` parameters with the respective
 * temperature values, which are expressed in a specific format.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param max A pointer to an `int16_t` where the maximum temperature value will
 * be stored. Caller retains ownership and must ensure it is not
 * null.
 * @param min A pointer to an `int16_t` where the minimum temperature value will
 * be stored. Caller retains ownership and must ensure it is not
 * null.
 * @return Returns an integer status code indicating success or failure of the
 * operation. On success, `max` and `min` will contain the maximum and
 * minimum temperature values, respectively.
 ******************************************************************************/
int32_t adi_ad9081_device_get_temperature(adi_ad9081_device_t *device,
					  int16_t *max, int16_t *min);

/*===== A 1 . 0   S E R D E S  L I N K  T E S T  M O D E S   =====*/

/*===== A 1 . 1   J R X  S E R D E S  L I N K  T E S T  M O D E S   =====*/
/***************************************************************************//**
 * @brief This function is used to start a pseudo-random binary sequence (PRBS)
 * test on the JESD RX PHY of the specified device. It must be called
 * after the device has been properly initialized. The function takes a
 * PRBS pattern and a duration in milliseconds for which the test will
 * run. If an invalid PRBS pattern is provided, the function will return
 * an error. The function also clears any previous error counts before
 * starting the test and updates the error count during the specified
 * duration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param prbs_pattern An enumeration value of type
 * `adi_cms_jesd_prbs_pattern_e` that specifies the PRBS
 * pattern to use. Valid values are PRBS7, PRBS9, PRBS15,
 * and PRBS31. An invalid value will result in an error.
 * @param time_ms An unsigned integer representing the duration of the PRBS test
 * in milliseconds. Must be a positive value.
 * @return Returns an integer indicating the status of the operation. A return
 * value of `API_CMS_ERROR_OK` indicates success, while other values
 * indicate specific error conditions.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_phy_prbs_test(adi_ad9081_device_t *device,
				 adi_cms_jesd_prbs_pattern_e prbs_pattern,
				 uint32_t time_ms);

/***************************************************************************//**
 * @brief This function is used to obtain the results of the PRBS (Pseudo-Random
 * Binary Sequence) test for a specific lane of the device. It should be
 * called after the PRBS test has been initiated and is intended for use
 * in diagnostic or validation scenarios. The function expects a valid
 * `device` pointer and a non-null `prbs_rx_result` pointer to store the
 * results. If either pointer is null, the function will return an error.
 * The results include the error count and pass/fail status of the PRBS
 * test, which are populated in the `prbs_rx_result` structure.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lane An 8-bit unsigned integer representing the lane number for which
 * the PRBS test results are requested. Valid values are typically 0
 * to 3, depending on the device configuration.
 * @param prbs_rx_result A pointer to an `adi_ad9081_prbs_test_t` structure
 * where the PRBS test results will be stored. Must not be
 * null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the PRBS test
 * results have been successfully retrieved and stored in the provided
 * structure. If an error occurs during the operation, an appropriate
 * error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_phy_prbs_test_result_get(
	adi_ad9081_device_t *device, uint8_t lane,
	adi_ad9081_prbs_test_t *prbs_rx_result);

/***************************************************************************//**
 * @brief This function is used to disable the pseudo-random binary sequence
 * (PRBS) test mode for the JESD RX PHY of the specified device. It
 * should be called when the PRBS test is no longer needed, typically
 * after testing is complete. The function expects a valid device pointer
 * and will return an error if the pointer is null. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the PRBS test
 * mode has been successfully disabled.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_phy_prbs_test_disable_set(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to start a pseudo-random binary sequence (PRBS)
 * test on the JESD receiver of the specified device. It must be called
 * after the device has been properly initialized. The function
 * configures the PRBS pattern based on the provided pattern type and
 * enables the PRBS test for the specified lane. The test runs for the
 * duration specified by the `time_sec` parameter, during which it clears
 * and updates the PRBS error count. If an invalid PRBS pattern is
 * provided, the function will return an error. It is important to ensure
 * that the `device` pointer is not null before calling this function.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param prbs_pattern The PRBS pattern to be used for the test, which can be
 * one of the following: `PRBS7`, `PRBS9`, `PRBS15`,
 * `PRBS23`, or `PRBS31`. An invalid value will result in an
 * error.
 * @param lane The lane number on which to perform the PRBS test. Valid values
 * depend on the device specifications.
 * @param time_sec The duration in seconds for which the PRBS test will run.
 * Must be a positive integer.
 * @return Returns `API_CMS_ERROR_OK` on successful initiation of the PRBS test.
 * If an error occurs during the process, an appropriate error code is
 * returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_sample_prbs_test(adi_ad9081_device_t *device,
				    adi_cms_jesd_prbs_pattern_e prbs_pattern,
				    uint8_t lane, uint32_t time_sec);

/***************************************************************************//**
 * @brief This function is used to obtain the results of the PRBS (Pseudo-Random
 * Binary Sequence) test for the JESD RX interface. It should be called
 * after the PRBS test has been initiated and is typically used for
 * diagnostic purposes to check the integrity of the received data. The
 * function expects a valid `device` pointer and will return error flags
 * and counts of errors detected in both the in-phase and quadrature
 * components. If any of the provided pointers are null, or if there are
 * issues accessing the device registers, the function will handle these
 * errors appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param error_flag A pointer to a `uint8_t` where the error flag will be
 * stored. Must not be null.
 * @param error_count_i A pointer to a `uint32_t` where the count of in-phase
 * errors will be stored. Must not be null.
 * @param error_count_q A pointer to a `uint32_t` where the count of quadrature
 * errors will be stored. Must not be null.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates successful
 * retrieval of the test results.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_sample_prbs_test_result_get(
	adi_ad9081_device_t *device, uint8_t *error_flag,
	uint32_t *error_count_i, uint32_t *error_count_q);

/***************************************************************************//**
 * @brief This function is used to find the best Serial Peripheral Output (SPO)
 * values for a specified JESD RX lane by sweeping through potential
 * values and testing them against a specified PRBS pattern. It must be
 * called after the device has been properly initialized and configured.
 * The function will modify the values pointed to by `left_spo` and
 * `right_spo` to reflect the best SPO values found during the sweep. If
 * the provided `deser_mode` is invalid, the function will return an
 * error. Additionally, if any of the input pointers are null, the
 * function will return an error without performing any operations.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param lane The JESD RX lane number to be configured. Valid values depend on
 * the specific device configuration.
 * @param prbs_pattern The PRBS pattern to be used for testing. Must be a valid
 * value from the `adi_cms_jesd_prbs_pattern_e` enumeration.
 * @param deser_mode The deserialization mode to be used. Must be a valid value
 * from the `adi_ad9081_deser_mode_e` enumeration, such as
 * `AD9081_HALF_RATE` or `AD9081_QUART_RATE`.
 * @param prbs_delay_sec The delay in seconds for the PRBS test. Must be a non-
 * negative integer.
 * @param left_spo Pointer to a `uint8_t` where the optimal left SPO value will
 * be stored. Must not be null.
 * @param right_spo Pointer to a `uint8_t` where the optimal right SPO value
 * will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if any operation fails during the sweep
 * process.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_spo_sweep(adi_ad9081_device_t *device, uint8_t lane,
				     adi_cms_jesd_prbs_pattern_e prbs_pattern,
				     adi_ad9081_deser_mode_e deser_mode,
				     uint32_t prbs_delay_sec, uint8_t *left_spo,
				     uint8_t *right_spo);

/***************************************************************************//**
 * @brief This function is used to initiate a vertical eye scan on the JESD RX
 * interface of the specified device. It must be called after the device
 * has been properly initialized. The function takes a direction and a
 * lane as parameters, which determine the scan's orientation and the
 * specific lane to be scanned, respectively. The function will wait for
 * the scan to complete, with a maximum wait time defined, and will log
 * an error if the scan does not complete in time. It is important to
 * ensure that the `device` pointer is valid and not null before calling
 * this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param direction A `uint8_t` value indicating the scan direction. Valid
 * values are typically 0 or 1, representing different scan
 * orientations.
 * @param lane A `uint8_t` value specifying the lane to be scanned. Valid values
 * depend on the device's configuration, typically ranging from 0 to
 * the maximum number of lanes supported.
 * @return Returns `API_CMS_ERROR_OK` on successful completion of the scan, or
 * an error code if the operation fails or times out.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_qr_vertical_eye_scan(adi_ad9081_device_t *device,
						uint8_t direction,
						uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to execute a two-dimensional eye scan on a
 * specified JESD RX lane of the device. It should be called after the
 * device has been properly initialized and configured. The function
 * modifies the provided `eye_scan_data` array to store the results of
 * the scan, which includes various measurements based on the settings of
 * the lane. It is important to ensure that the `device` pointer is valid
 * and not null before calling this function, as it will return an error
 * if the pointer is invalid. The function handles various internal
 * configurations and restores the device state upon completion.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lane An 8-bit unsigned integer representing the lane number to be
 * scanned. Valid values are typically within the range of available
 * lanes for the device.
 * @param eye_scan_data An array of 96 elements where the results of the eye
 * scan will be stored. The caller retains ownership of
 * this array, and it must be allocated before calling the
 * function.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success, while any negative value indicates an error occurred during
 * the execution.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_qr_two_dim_eye_scan(adi_ad9081_device_t *device,
					       uint8_t lane,
					       uint16_t eye_scan_data[96]);

/***************************************************************************//**
 * @brief This function is used to conduct a vertical eye scan on a JESD RX
 * interface, which helps in determining the optimal voltage level for
 * signal integrity. It should be called after the device has been
 * properly initialized and configured. The function modifies the
 * `good_mv` parameter to reflect the best voltage level found during the
 * scan. It is important to ensure that the `device` pointer is valid and
 * that the `lane` parameter corresponds to a valid lane. The function
 * will return an error if any of the internal operations fail, and it
 * will reset the voltage level to 0mV after the scan.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param direction A `uint8_t` indicating the direction of the sweep; valid
 * values are 0 (positive) or 1 (negative).
 * @param lane A `uint8_t` specifying the lane to be scanned; must be within the
 * valid range for the device.
 * @param good_mv Pointer to a `uint8_t` where the optimal voltage level (in mV)
 * will be stored. Caller retains ownership and must ensure it
 * points to a valid memory location.
 * @param prbs_pattern An `adi_cms_jesd_prbs_pattern_e` enumeration value
 * representing the PRBS pattern to be used during the test.
 * @param prbs_delay_ms A `uint32_t` specifying the delay in milliseconds for
 * the PRBS test; must be a non-negative value.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the scan was
 * completed without errors. If an error occurs during the operation, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_hr_vertical_eye_scan(
	adi_ad9081_device_t *device, uint8_t direction, uint8_t lane,
	uint8_t *good_mv, adi_cms_jesd_prbs_pattern_e prbs_pattern,
	uint32_t prbs_delay_ms);

/***************************************************************************//**
 * @brief This function is used to conduct a two-dimensional eye scan on a
 * specified JESD RX lane of the device. It should be called after the
 * device has been properly initialized and configured. The function
 * evaluates the eye scan by adjusting the vertical and horizontal
 * settings and collecting results in the provided `eye_scan_data` array.
 * The function handles both left and right horizontal scans, storing
 * results for each position tested. If the input parameters are invalid,
 * such as a null device pointer or out-of-range lane number, the
 * function will return an error without modifying the output data.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param lane The lane number to perform the eye scan on. Valid values are
 * typically 0 to the maximum number of lanes supported by the
 * device. An out-of-range value will result in an error.
 * @param prbs_pattern The PRBS pattern to be used for the test. This should be
 * a valid enumeration value from
 * `adi_cms_jesd_prbs_pattern_e`.
 * @param prbs_delay_ms The delay in milliseconds for the PRBS test. This should
 * be a non-negative integer.
 * @param eye_scan_data An array of at least 192 elements where the results of
 * the eye scan will be stored. The caller retains
 * ownership of this array.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the eye scan
 * was completed without errors. If an error occurs during execution, an
 * appropriate error code will be returned, and the contents of
 * `eye_scan_data` may be partially filled with results.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_hr_two_dim_eye_scan(
	adi_ad9081_device_t *device, uint8_t lane,
	adi_cms_jesd_prbs_pattern_e prbs_pattern, uint32_t prbs_delay_ms,
	uint16_t eye_scan_data[192]);

/*===== A 1 . 2   J T X  S E R D E S  L I N K  T E S T  M O D E S   =====*/
/***************************************************************************//**
 * @brief This function is used to start a checkerboard test pattern on the JESD
 * transmitter of the specified device. It should be called after the
 * device has been properly initialized and configured. The function
 * checks for null pointers and will return an error if the provided
 * device pointer is invalid. It is important to ensure that the correct
 * link and data source are specified, as these parameters determine the
 * behavior of the test. The function will return an error code if the
 * test initiation fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to use for the test. Valid values
 * depend on the device configuration.
 * @param data_source An enumeration value of type
 * `adi_ad9081_jesd_tx_test_data_src_e` that indicates the
 * source of the test data. Valid values are defined in the
 * enumeration.
 * @return Returns `API_CMS_ERROR_OK` on successful initiation of the
 * checkerboard test. If an error occurs, a negative error code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_checker_board_test(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	adi_ad9081_jesd_tx_test_data_src_e data_source);

/***************************************************************************//**
 * @brief This function is used to initiate a word toggle test mode for the JESD
 * transmitter of the specified device. It should be called after the
 * device has been properly initialized and configured. The function
 * checks for a null pointer for the device parameter and will return an
 * error if the device is not valid. It is important to ensure that the
 * correct link and data source are specified, as these parameters
 * determine the behavior of the test. The function will return an error
 * code if the test initiation fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; the caller retains ownership.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to use for the test. Valid values
 * are defined in the enumeration.
 * @param data_source An enumeration value of type
 * `adi_ad9081_jesd_tx_test_data_src_e` that specifies the
 * source of the test data. Valid values are defined in the
 * enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the failure reason.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_word_toggle_test(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	adi_ad9081_jesd_tx_test_data_src_e data_source);

/***************************************************************************//**
 * @brief This function is used to start a ramp test on the JESD transmitter of
 * the specified device. It should be called after the device has been
 * properly initialized and configured. The function checks for null
 * pointers and will return an error if the provided device pointer is
 * invalid. It is important to ensure that the correct link and data
 * source are specified, as these parameters determine the behavior of
 * the test. The function will return an error code if the test
 * generation fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; the caller retains ownership.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to use for the test. Valid values
 * are defined in the enumeration.
 * @param data_source An enumeration value of type
 * `adi_ad9081_jesd_tx_test_data_src_e` that indicates the
 * source of test data. Valid values are defined in the
 * enumeration.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the ramp test
 * has been initiated successfully. If an error occurs during the test
 * generation, an appropriate error code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_tx_ramp_test(adi_ad9081_device_t *device,
			     adi_ad9081_jesd_link_select_e links,
			     adi_ad9081_jesd_tx_test_data_src_e data_source);

/***************************************************************************//**
 * @brief This function is used to configure the JESD transmitter to repeatedly
 * send user-defined test data. It must be called after the device has
 * been properly initialized. The function expects a valid `device`
 * pointer and will return an error if the pointer is null. The user must
 * specify which JESD links to use and provide a source for the test
 * data. The `data` array must contain exactly 9 bytes of data, which
 * will be written to specific registers in the JESD transmitter. If any
 * of the specified links are invalid or if there are issues writing to
 * the registers, the function will return an error code.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Specifies which JESD links to configure. Valid values are
 * defined in the `adi_ad9081_jesd_link_select_e` enumeration.
 * @param data_source Specifies the source of the test data. Valid values are
 * defined in the `adi_ad9081_jesd_tx_test_data_src_e`
 * enumeration.
 * @param data Array of 9 bytes containing the user-defined test data. Must not
 * be null and must contain exactly 9 elements.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_repeat_user_data_test(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	adi_ad9081_jesd_tx_test_data_src_e data_source, uint8_t data[9]);

/***************************************************************************//**
 * @brief This function is used to start a pseudo-random binary sequence (PRBS)
 * test on the JESD transmitter of the specified device. It must be
 * called after the device has been properly initialized. The function
 * takes a link selection and a PRBS pattern as parameters, and it will
 * configure the transmitter to generate the specified test pattern. If
 * an invalid PRBS pattern is provided, the function will return an
 * error. It is important to ensure that the `device` pointer is not null
 * before calling this function.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to configure. Valid values are
 * defined in the enumeration.
 * @param prbs_pattern An enumeration value of type
 * `adi_cms_jesd_prbs_pattern_e` that specifies the PRBS
 * pattern to use. Valid values include `PRBS7`, `PRBS15`,
 * and `PRBS31`. If an invalid pattern is provided, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or other issues.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_tx_phy_prbs_test(adi_ad9081_device_t *device,
				 adi_ad9081_jesd_link_select_e links,
				 adi_cms_jesd_prbs_pattern_e prbs_pattern);

/***************************************************************************//**
 * @brief This function is used to initiate a pseudo-random binary sequence
 * (PRBS) test pattern for the JESD transmitter of the specified device.
 * It must be called after the device has been properly initialized and
 * configured. The function takes a PRBS pattern type as an argument,
 * which determines the specific PRBS sequence to generate. If an invalid
 * pattern is provided, the function will return an error. It is
 * important to ensure that the `device` pointer is not null before
 * calling this function, as it will result in an immediate error return.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to use. Valid values depend on
 * the device configuration.
 * @param prbs_pattern An enumeration value of type
 * `adi_cms_jesd_prbs_pattern_e` that specifies the PRBS
 * pattern to generate. Valid values include `PRBS7`,
 * `PRBS15`, and `PRBS31`. If an invalid value is provided,
 * the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or other issues.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_sample_data_prbs_test(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	adi_cms_jesd_prbs_pattern_e prbs_pattern);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ILAS test mode for one
 * or more JESD links on the specified device. It must be called with a
 * valid `device` pointer that has been properly initialized. The `links`
 * parameter allows the selection of which JESD links to configure, and
 * the `enable` parameter determines whether to turn the test mode on (1)
 * or off (0). If an invalid link is specified or if the device pointer
 * is null, the function will handle these cases gracefully by returning
 * an error. It is important to ensure that the device is in a state that
 * allows configuration changes when this function is called.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to configure. Valid values are `AD9081_LINK_0`
 * and `AD9081_LINK_1`.
 * @param enable A `uint8_t` value where 1 enables the ILAS test mode and 0
 * disables it. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_ilas_test_mode_enable_set(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable continuous transmission for
 * specific JESD links on the `adi_ad9081_device_t`. It should be called
 * after the device has been properly initialized and configured. The
 * function takes a `links` parameter that specifies which JESD links to
 * modify, and a `lane_id` to indicate the specific lane for the
 * operation. The `enable` parameter determines whether to enable (non-
 * zero value) or disable (zero value) the continuous transmission. If
 * the specified `device` pointer is null, the function will return an
 * error. Additionally, if the specified `links` do not correspond to
 * valid JESD links, the function will not perform any operations for
 * those links.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to modify. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`. The function will only
 * affect the specified links.
 * @param lane_id Identifier for the specific lane to modify. Valid values
 * depend on the device configuration.
 * @param enable A boolean value where a non-zero value enables continuous
 * transmission and zero disables it.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors. If an error occurs during the
 * operation, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_continuous_d215_enable_set(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	uint8_t lane_id, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the RPAT (Random Pattern)
 * feature for a specific JESD link on the device. It must be called with
 * a valid `device` pointer that has been properly initialized. The
 * `links` parameter specifies which JESD link to configure, while
 * `lane_id` identifies the specific lane within that link. The `enable`
 * parameter determines whether to enable (non-zero value) or disable
 * (zero value) the RPAT feature. If the `device` pointer is null, the
 * function will return an error. It is important to ensure that the
 * device is in a state that allows configuration changes when this
 * function is called.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies the JESD link to configure. Valid values depend
 * on the specific device configuration.
 * @param lane_id A `uint8_t` representing the lane ID within the specified JESD
 * link. Valid values are typically in the range of 0 to the
 * maximum number of lanes supported by the device.
 * @param enable A `uint8_t` that indicates whether to enable (non-zero) or
 * disable (zero) the RPAT feature.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_rpat_enable_set(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t lane_id, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD204B transmit
 * pattern for a specified lane on a given device. It must be called
 * after the device has been properly initialized. The `device` parameter
 * must not be null, and the `links` parameter should specify the
 * appropriate JESD link. The `lane_id` must be within the valid range
 * for the specified link, and the `enable` parameter determines whether
 * the pattern is enabled (non-zero value) or disabled (zero value). If
 * any of the parameters are invalid, the function will return an error.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link to configure. Valid values depend
 * on the device's configuration.
 * @param lane_id An 8-bit integer representing the lane ID. Must be within the
 * valid range for the specified link.
 * @param enable An 8-bit integer that indicates whether to enable (non-zero) or
 * disable (zero) the transmit pattern.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_jspat_enable_set(adi_ad9081_device_t *device,
					    adi_ad9081_jesd_link_select_e links,
					    uint8_t lane_id, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JTS pattern for a
 * specific lane in a JESD link of the `adi_ad9081_device_t`. It must be
 * called with a valid device pointer that has been properly initialized.
 * The function checks for null pointers and will return an error if the
 * device pointer is invalid. The `links` parameter specifies which JESD
 * link to configure, while `lane_id` indicates the specific lane within
 * that link. The `enable` parameter determines whether to enable (non-
 * zero value) or disable (zero value) the JTS pattern. It is important
 * to ensure that the lane ID is within the valid range for the specified
 * link.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Specifies the JESD link to configure. Valid values are defined
 * in the `adi_ad9081_jesd_link_select_e` enumeration.
 * @param lane_id The ID of the lane to configure. Must be within the valid
 * range for the specified link.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the JTS pattern.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_tx_jtspat_enable_set(adi_ad9081_device_t *device,
				     adi_ad9081_jesd_link_select_e links,
				     uint8_t lane_id, uint8_t enable);

/*===== A 2 . 0   M U L T I C H I P  S Y N C  &  S U B C L A S S  1   =====*/
/***************************************************************************//**
 * @brief This function is intended to be called to initiate a one-shot
 * synchronization process for the JESD interface of the specified
 * device. It should be invoked after the device has been properly
 * initialized and configured. The function checks the current state of
 * the device and performs a series of register writes to set up the
 * synchronization. It is important to ensure that the device pointer is
 * valid and not null before calling this function. The function will log
 * warnings if the synchronization process does not complete as expected,
 * and it will return an error code if the synchronization fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device to be synchronized. Must not be null.
 * @param subclass An enumeration value of type `adi_cms_jesd_subclass_e` that
 * specifies the subclass of the JESD interface. Valid values
 * depend on the specific implementation and configuration of
 * the device.
 * @return Returns an error code indicating the result of the synchronization
 * process. If the synchronization is successful, it returns
 * `API_CMS_ERROR_OK`. If the synchronization fails, it returns
 * `API_CMS_ERROR_JESD_SYNC_NOT_DONE`.
 ******************************************************************************/
int32_t adi_ad9081_jesd_oneshot_sync(adi_ad9081_device_t *device,
				     adi_cms_jesd_subclass_e subclass);

/***************************************************************************//**
 * @brief This function is used to configure the SYSREF phase of the DAC in the
 * specified device. It must be called after the device has been properly
 * initialized. The phase parameter represents the phase of the measured
 * SYSREF event in DAC clock units. If the provided device pointer is
 * null, the function will return an error. Additionally, if the phase
 * value is outside the valid range, an error will also be returned. It
 * is important to ensure that the device is ready for configuration
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param phase A 16-bit unsigned integer representing the SYSREF phase in DAC
 * clock units. Valid values depend on the specific DAC
 * configuration; invalid values will result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_sysref_phase_set(adi_ad9081_device_t *device,
					uint16_t phase);

/***************************************************************************//**
 * @brief This function configures the sample type for the SYSREF signal in the
 * DAC. It should be called after the device has been properly
 * initialized. The `sample_type` parameter determines how the SYSREF
 * signal is sampled: a value of 0 indicates that it is sampled by the
 * reference clock followed by the high-speed clock, while a value of 1
 * indicates direct sampling by the high-speed clock. If the `device`
 * pointer is null, the function will return an error. It is important to
 * ensure that the device is ready for configuration before calling this
 * function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null.
 * @param sample_type An 8-bit unsigned integer that specifies the sample type.
 * Valid values are 0 or 1. Any other value will be handled
 * as an error.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the sample
 * type has been set successfully. If an error occurs, a negative error
 * code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_sysref_sample_type_set(adi_ad9081_device_t *device,
					      uint8_t sample_type);

/***************************************************************************//**
 * @brief This function is used to reset the Numerically Controlled Oscillator
 * (NCO) of the DAC. It should be called when the NCO needs to be re-
 * initialized, typically after a configuration change or to recover from
 * an error state. The function expects a valid `device` pointer, which
 * must not be null. If the pointer is null, the function will return an
 * error without performing any operations. The function performs a
 * sequence of operations to start and then stop the NCO synchronization,
 * ensuring that the NCO is properly reset.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null; otherwise, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, indicating that
 * the NCO reset operation was completed without errors. If an error
 * occurs during the operation, an appropriate error code will be
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_reset_set(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to enable or disable the NCO synchronization
 * reset feature via SYSREF for a specified device. It should be called
 * after the device has been properly initialized. The `enable` parameter
 * determines whether the synchronization reset is activated or
 * deactivated. It is important to ensure that the `device` pointer is
 * valid and points to an initialized `adi_ad9081_device_t` structure
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must point to a valid
 * initialized device.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the NCO synchronization reset. Valid values
 * are 0 (disable) or 1 (enable).
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A value of 0 typically indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t
adi_ad9081_dac_nco_sync_reset_via_sysref_set(adi_ad9081_device_t *device,
					     uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the NCO synchronization system
 * reference mode for a specified device. It should be called after the
 * device has been properly initialized. The mode parameter determines
 * the specific synchronization behavior, and it is essential to ensure
 * that the mode value is valid as per the device's specifications.
 * Calling this function with an invalid device pointer or an unsupported
 * mode may lead to undefined behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and should point to a valid
 * initialized device.
 * @param mode An 8-bit unsigned integer representing the desired
 * synchronization mode. Valid values depend on the device's
 * specifications. The function does not handle invalid mode values,
 * and passing an unsupported mode may result in an error.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_sync_sysref_mode_set(adi_ad9081_device_t *device,
						uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the synchronization source for the DAC's NCO
 * (Numerically Controlled Oscillator). It must be called after the
 * device has been properly initialized. The `align_source` parameter
 * determines the synchronization method, which can be one of several
 * predefined options. If an invalid `align_source` is provided, the
 * function will return an error. It is important to ensure that the
 * `device` pointer is not null before calling this function, as it will
 * result in an error if it is.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null.
 * @param align_source An integer value that specifies the synchronization
 * source. Valid values are 0 to 4, where each value
 * corresponds to a specific synchronization method. If the
 * value is outside this range, the function will return an
 * error.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration of the
 * synchronization source. If an error occurs, a negative error code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_sync_set(adi_ad9081_device_t *device,
				    uint8_t align_source);

/***************************************************************************//**
 * @brief This function is used to configure the NCO (Numerically Controlled
 * Oscillator) mode of the DAC (Digital-to-Analog Converter) in either
 * master or slave mode. It should be called after the device has been
 * properly initialized and configured. The mode parameter determines the
 * synchronization behavior of the NCO, which is crucial for applications
 * requiring precise timing and frequency control. Ensure that the mode
 * value is valid to avoid unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null.
 * @param mode A `uint8_t` value representing the NCO mode. Valid values depend
 * on the specific implementation but typically include options for
 * master and slave modes. Invalid values may result in an error.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of zero typically indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_master_slave_mode_set(adi_ad9081_device_t *device,
						 uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the NCO GPIO output for a specific
 * index on the device. It should be called after the device has been
 * properly initialized. The `gpio_index` parameter specifies which GPIO
 * pin to configure, and the `output` parameter determines the output
 * state. It is important to ensure that the `gpio_index` is within the
 * valid range for the device, as invalid indices may lead to undefined
 * behavior. This function does not modify the state of the device if the
 * parameters are invalid.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param gpio_index An index for the GPIO pin to be configured. Valid values
 * depend on the specific device configuration. Must be within
 * the acceptable range; otherwise, the function may not
 * behave as expected.
 * @param output A value representing the desired output state for the GPIO pin.
 * Typically, this is a binary value (0 or 1). The function will
 * set the GPIO output accordingly.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of 0 typically indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_master_slave_gpio_set(adi_ad9081_device_t *device,
						 uint8_t gpio_index,
						 uint8_t output);

/***************************************************************************//**
 * @brief This function is used to configure the NCO (Numerically Controlled
 * Oscillator) trigger source for the DAC (Digital-to-Analog Converter)
 * in the specified device. It should be called after the device has been
 * properly initialized and configured. The function allows the user to
 * select the source of the NCO trigger, which is essential for
 * synchronizing the DAC operation. It is important to ensure that the
 * `device` parameter is valid and that the `source` parameter is within
 * the acceptable range, as invalid values may lead to undefined
 * behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must point to a valid
 * initialized device.
 * @param source An 8-bit unsigned integer representing the trigger source.
 * Valid values depend on the specific implementation and should
 * be checked against the device's documentation. Invalid values
 * may result in an error.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t
adi_ad9081_dac_nco_master_slave_trigger_source_set(adi_ad9081_device_t *device,
						   uint8_t source);

/***************************************************************************//**
 * @brief This function is used to configure the extra LMFC (Low Master Frame
 * Clock) number for the NCO (Numerically Controlled Oscillator) in a
 * device operating in master or slave mode. It should be called after
 * the device has been properly initialized and configured. The function
 * expects a valid `device` pointer and a `num` value that specifies the
 * extra LMFC number. If the provided `device` pointer is null or if
 * `num` is outside the valid range, the function may return an error
 * code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param num An 8-bit unsigned integer representing the extra LMFC number.
 * Valid values depend on the specific device configuration; invalid
 * values may result in an error.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t
adi_ad9081_dac_nco_master_slave_extra_lmfc_num_set(adi_ad9081_device_t *device,
						   uint8_t num);

/***************************************************************************//**
 * @brief This function is used to configure the NCO (Numerically Controlled
 * Oscillator) synchronization trigger for the DAC (Digital-to-Analog
 * Converter) in the specified device. It should be called after the
 * device has been properly initialized to ensure that the NCO can be
 * synchronized correctly. If the provided device pointer is null, the
 * function will handle this gracefully, returning an error code. It is
 * important to ensure that the device is in a state that allows for NCO
 * configuration before invoking this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null; otherwise, the function will
 * return an error code.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of 0 typically indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t
adi_ad9081_dac_nco_master_slave_trigger_set(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to configure and synchronize the DAC NCO
 * (Numerically Controlled Oscillator) in either master or slave mode. It
 * must be called after initializing the `adi_ad9081_device_t` structure
 * and before any DAC operations. The function sets various parameters
 * such as the master/slave mode, trigger source, GPIO index, and an
 * additional LMFC number. It also performs a synchronization reset to
 * ensure the NCO is properly aligned. If the `is_master` parameter is
 * set to indicate master mode, it will also trigger the synchronization
 * process. Invalid parameters will result in an error being returned.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param is_master Indicates whether the device operates in master (1) or slave
 * (0) mode. Valid values are 0 or 1.
 * @param trigger_src Specifies the trigger source for synchronization. Valid
 * values depend on the specific implementation and should be
 * defined in the documentation.
 * @param gpio_index Index of the GPIO to be used for synchronization. Must be
 * within the valid range of GPIO indices supported by the
 * device.
 * @param extra_lmfc_num An additional LMFC number for configuration. Must be
 * within the valid range defined by the device
 * specifications.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, or an error code
 * indicating the type of failure if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_dac_nco_master_slave_sync(adi_ad9081_device_t *device,
					     uint8_t is_master,
					     uint8_t trigger_src,
					     uint8_t gpio_index,
					     uint8_t extra_lmfc_num);

/***************************************************************************//**
 * @brief This function is used to control the SYSREF functionality of the
 * device, allowing the user to enable or disable the SYSREF signal as
 * needed. It should be called after the device has been properly
 * initialized. If the `device` pointer is null, the function will return
 * an error without making any changes. The `enable` parameter determines
 * whether the SYSREF functionality is turned on (when set to a non-zero
 * value) or off (when set to zero). It is important to ensure that the
 * device is in a state that allows for this operation, as improper usage
 * may lead to unexpected behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the SYSREF functionality.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the SYSREF
 * functionality has been set as requested.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_enable_set(adi_ad9081_device_t *device,
					  uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the SYSREF input mode for the JESD interface
 * of the specified device. It must be called after the device has been
 * properly initialized. The function allows enabling or disabling the
 * receiver and capture functionality, and it sets the coupling mode to
 * either AC or DC. If an invalid `input_mode` is provided, the function
 * will return an error. It is important to ensure that the device
 * pointer is not null before calling this function.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param enable_receiver A boolean value indicating whether to enable the
 * SYSREF receiver (1 to enable, 0 to disable).
 * @param enable_capture A boolean value indicating whether to enable SYSREF
 * capture (1 to enable, 0 to disable).
 * @param input_mode An enumeration value of type `adi_cms_signal_coupling_e`
 * that specifies the coupling mode. Valid values are
 * `COUPLING_AC` for AC coupling and `COUPLING_DC` for DC
 * coupling. Must be a valid value; otherwise, an error is
 * returned.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * parameter is provided or if any internal operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_input_mode_set(
	adi_ad9081_device_t *device, uint8_t enable_receiver,
	uint8_t enable_capture, adi_cms_signal_coupling_e input_mode);

/***************************************************************************//**
 * @brief This function is used to configure the SYSREF input settings of the
 * specified device, which is essential for proper synchronization in
 * data acquisition systems. It must be called after the device has been
 * initialized and before any data acquisition begins. The function
 * checks for valid parameters and will return an error if any of the
 * input values are out of range or if the coupling mode or signal type
 * is invalid. It is important to ensure that the coupling mode and
 * signal type are compatible, as certain combinations are required for
 * successful configuration.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param coupling_mode Specifies the coupling mode for the SYSREF input, which
 * must be either `COUPLING_AC` or `COUPLING_DC`. Invalid
 * values will result in an error.
 * @param signal_type Indicates the type of signal being used, which must not be
 * `SIGNAL_UNKNOWN`. Other valid types include `SIGNAL_CMOS`,
 * `SIGNAL_LVDS`, `SIGNAL_CML`, and `SIGNAL_LVPECL`.
 * @param sysref_single_end_p An 8-bit unsigned integer representing the single-
 * ended positive SYSREF input. Must be in the range
 * of 0 to 15.
 * @param sysref_single_end_n An 8-bit unsigned integer representing the single-
 * ended negative SYSREF input. Must be in the range
 * of 0 to 15.
 * @return Returns `API_CMS_ERROR_OK` on successful configuration. If an error
 * occurs due to invalid parameters or incompatible settings, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_sync_sysref_input_config_set(
	adi_ad9081_device_t *device, adi_cms_signal_coupling_e coupling_mode,
	adi_cms_signal_type_e signal_type, uint8_t sysref_single_end_p,
	uint8_t sysref_single_end_n);

/***************************************************************************//**
 * @brief This function configures the ADC SYSREF resynchronization mode for the
 * specified device. It should be called after the device has been
 * properly initialized. The `mode` parameter determines the
 * synchronization behavior, where a value of 0 sets the timestamp mode
 * and a value of 1 sets the resync mode. If the provided `device`
 * pointer is null, the function will return an error. It is important to
 * ensure that the device is ready to accept this configuration before
 * calling the function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mode An 8-bit unsigned integer that specifies the synchronization
 * mode. Valid values are 0 for timestamp mode and 1 for resync
 * mode.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_sysref_resync_mode_set(adi_ad9081_device_t *device,
					      uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the rising edge of the ADC SYSREF
 * signal for the specified device. It should be called after the device
 * has been properly initialized. The `enable` parameter determines
 * whether the rising edge is enabled or disabled. If the `device`
 * pointer is null, the function will return an error without making any
 * changes. It is important to ensure that the device is in a valid state
 * before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that specifies whether to enable (non-zero)
 * or disable (zero) the rising edge of the SYSREF signal.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_sysref_rise_edge_enable_set(adi_ad9081_device_t *device,
						   uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the counts of risk violations for
 * SYSREF setup and hold times from the specified device. It must be
 * called with a valid `device` pointer that has been properly
 * initialized. The function also requires non-null pointers for
 * `setup_risk_violation` and `hold_risk_violation`, which will be
 * populated with the respective counts of risk violations. If any of the
 * pointers are null, the function will return an error. The counts
 * represent the number of '1' bits in the respective registers,
 * indicating the risk level for setup and hold times.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param setup_risk_violation A pointer to a `uint8_t` where the setup risk
 * violation count will be stored. Must not be null.
 * @param hold_risk_violation A pointer to a `uint8_t` where the hold risk
 * violation count will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the risk
 * violation counts have been successfully retrieved and stored in the
 * provided pointers.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_setup_hold_get(adi_ad9081_device_t *device,
					      uint8_t *setup_risk_assessment,
					      uint8_t *hold_risk_assessment);

/***************************************************************************//**
 * @brief This function configures the fine and superfine delay settings for the
 * SYSREF input of the device. It must be called after the device has
 * been properly initialized. The `enable` parameter determines which
 * delay settings are active: 0 disables delays, 1 enables fine delay, 2
 * enables superfine delay, and 3 enables both. If the `enable` parameter
 * is set to 1 or 3, the `fine_delay` parameter must be provided, and if
 * set to 2 or 3, the `superfine_delay` parameter must be provided.
 * Invalid values for `enable` will result in an error, and passing a
 * null `device` pointer will also trigger an error.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param enable Controls the activation of delay settings. Valid values are 0
 * (disabled), 1 (fine delay), 2 (superfine delay), and 3 (both).
 * Values greater than 3 will result in an error.
 * @param fine_delay Specifies the fine delay setting. This parameter is only
 * relevant if `enable` is 1 or 3. The maximum effective
 * setting is 0x2F.
 * @param superfine_delay Specifies the superfine delay setting. This parameter
 * is only relevant if `enable` is 2 or 3. The maximum
 * setting corresponds to approximately 4ps (255 x 16
 * fs).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_fine_superfine_delay_set(
	adi_ad9081_device_t *device, uint8_t enable, uint8_t fine_delay,
	uint8_t superfine_delay);

/***************************************************************************//**
 * @brief This function configures the number of edges for the SYSREF signal in
 * the JESD204 interface of the device. It must be called after the
 * device has been properly initialized and before any data transmission
 * occurs. If the `device` pointer is null, the function will return an
 * error without making any changes. Additionally, the function will
 * handle any errors that occur during the setting of the SYSREF count,
 * ensuring that the caller is informed of any issues.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param edges An 8-bit unsigned integer representing the number of edges for
 * the SYSREF signal. Valid values depend on the specific
 * requirements of the JESD204 interface; invalid values may result
 * in an error being returned.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the SYSREF
 * count has been set successfully. If an error occurs, a negative error
 * code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_count_set(adi_ad9081_device_t *device,
					 uint8_t edges);

/***************************************************************************//**
 * @brief This function configures the average number of SYSREF pulses for the
 * specified device. It should be called after the device has been
 * properly initialized. The `pulses` parameter determines how many
 * SYSREF pulses to average, and it must be within the range of 0 to 7.
 * If an invalid value is provided, the function will return an error.
 * Additionally, if the `device` pointer is null, the function will also
 * return an error, ensuring that the caller handles device
 * initialization correctly.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param pulses An 8-bit unsigned integer specifying the number of SYSREF
 * pulses to average. Valid values are from 0 to 7. If the value
 * exceeds 7, the function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the input
 * parameters are invalid or if an error occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_average_set(adi_ad9081_device_t *device,
					   uint8_t pulses);

/***************************************************************************//**
 * @brief This function is used to obtain the current SYSREF phase value from
 * the specified device. It must be called after the device has been
 * properly initialized. The function writes a strobe to trigger an
 * update of the SYSREF phase value and then reads the phase from the
 * device's registers. It is important to ensure that both the `device`
 * and `sysref_phase` parameters are valid and not null before calling
 * this function, as passing null pointers will result in an error. The
 * function will return an error code if the read operation fails.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device from which to retrieve the SYSREF phase. Must not be
 * null.
 * @param sysref_phase A pointer to a `uint16_t` variable where the retrieved
 * SYSREF phase value will be stored. Must not be null.
 * @return Returns an error code indicating the success or failure of the
 * operation. On success, the SYSREF phase value is stored in the
 * location pointed to by `sysref_phase`.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_monitor_phase_get(adi_ad9081_device_t *device,
						 uint16_t *sysref_phase);

/***************************************************************************//**
 * @brief This function is used to check for any alignment errors related to the
 * LMFC (Local Multi-Frame Clock) in the JESD204 interface of the AD9081
 * device. It should be called after the device has been properly
 * initialized. The function expects valid pointers for both the device
 * and the output parameter. If either pointer is null, the function will
 * return an error without performing any further operations. The output
 * parameter will be updated with the alignment error status, which can
 * be used to determine if there are issues with the LMFC alignment.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lmfc_align_err A pointer to a `uint8_t` variable where the LMFC
 * alignment error status will be stored. Must not be
 * null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the LMFC
 * alignment error status has been successfully retrieved.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_sysref_monitor_lmfc_align_error_get(adi_ad9081_device_t *device,
						    uint8_t *lmfc_align_err);

/***************************************************************************//**
 * @brief This function is used to configure the SYSREF error window threshold
 * for the JESD204 interface in the `adi_ad9081_device_t`. It should be
 * called after the device has been properly initialized. The
 * `sysref_error_window` parameter defines the allowable error window for
 * SYSREF alignment, and it must be within the range of 0 to 127. If the
 * provided value exceeds this range, the function will return an error.
 * Additionally, if the `device` pointer is null, the function will also
 * return an error. This function is essential for ensuring proper
 * synchronization in JESD204 applications.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param sysref_error_window An 8-bit unsigned integer that specifies the
 * SYSREF error window threshold. Valid values are
 * from 0 to 127. If the value exceeds 127, the
 * function will return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code indicating
 * the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_monitor_lmfc_align_threshold_set(
	adi_ad9081_device_t *device, uint8_t sysref_error_window);

/***************************************************************************//**
 * @brief This function is used to enable or disable the SYSREF interrupt for
 * the specified device. It should be called after the device has been
 * properly initialized. If the `device` pointer is null, the function
 * will return an error without making any changes. The `enable`
 * parameter determines whether the interrupt is enabled (non-zero value)
 * or disabled (zero value). It is important to ensure that the device is
 * in a valid state before calling this function to avoid unexpected
 * behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that specifies whether to enable (non-zero)
 * or disable (zero) the SYSREF interrupt.
 * @return Returns `API_CMS_ERROR_OK` on success. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_irq_enable_set(adi_ad9081_device_t *device,
					      uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the SYSREF jitter multiplexer pin for the
 * specified device. It must be called with a valid `device` pointer that
 * has been properly initialized. The `pin` parameter determines which of
 * the two available pins (0 or 1) will be used for the SYSREF jitter
 * output. If an invalid pin value is provided, the function will return
 * an error. It is important to ensure that the device is not null before
 * calling this function, as passing a null pointer will result in an
 * immediate error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param pin An 8-bit unsigned integer representing the pin to be set. Valid
 * values are 0 or 1. If the value is not 0 or 1, the function will
 * return an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or other issues.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_irq_jitter_mux_set(adi_ad9081_device_t *device,
						  uint8_t pin);

/***************************************************************************//**
 * @brief This function is used to check if the oneshot synchronization process
 * has been completed for a specified device. It should be called after
 * the synchronization process has been initiated. The function expects a
 * valid device pointer and a pointer to a variable where the
 * synchronization status will be stored. If the synchronization is not
 * finished, an error message will be logged. It is important to ensure
 * that both pointers provided are not null to avoid runtime errors.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param sync_done A pointer to a `uint8_t` variable where the synchronization
 * status will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the status has
 * been retrieved. If the synchronization is not finished, the value
 * pointed to by `sync_done` will not be equal to 1.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_sysref_oneshot_sync_done_get(adi_ad9081_device_t *device,
					     uint8_t *sync_done);

/***************************************************************************//**
 * @brief This function is used to compute the LMFC frequency for a given DAC
 * clock and interpolation settings. It must be called with a valid
 * `jesd_param` structure that contains the JESD204B parameters,
 * specifically `jesd_s` and `jesd_k`. The `lmfc_freq` parameter must not
 * be null, as it will be used to store the calculated frequency. If
 * `lmfc_freq` is null, the function will return an error code. The
 * function performs the calculation using the provided DAC clock, main
 * interpolation, and channel interpolation values, and it is important
 * to ensure that these values are set correctly to avoid incorrect
 * frequency calculations.
 *
 * @param dac_clk The DAC clock frequency in Hz. Must be a positive value.
 * @param main_interp The main interpolation factor. Must be a positive integer.
 * @param ch_interp The channel interpolation factor. Must be a positive
 * integer.
 * @param jesd_param A pointer to an `adi_cms_jesd_param_t` structure containing
 * JESD204B parameters. Must not be null.
 * @param lmfc_freq A pointer to a variable where the calculated LMFC frequency
 * will be stored. Must not be null.
 * @return Returns an error code indicating the success or failure of the
 * operation. On success, `lmfc_freq` is updated with the calculated
 * frequency.
 ******************************************************************************/
int32_t adi_ad9081_sync_calc_jrx_lmfc_lemc(uint64_t dac_clk,
					   uint8_t main_interp,
					   uint8_t ch_interp,
					   adi_cms_jesd_param_t *jesd_param,
					   uint64_t *lmfc_freq);

/***************************************************************************//**
 * @brief This function is used to compute the LMFC (Lane Multi-Frame Clock)
 * frequency based on the ADC clock and various parameters related to the
 * JESD interface. It should be called after initializing the JESD
 * parameters and before using the LMFC frequency in further processing.
 * The function expects valid JESD parameters and DCM values; if the
 * `lmfc_freq` pointer is null, it will return an error. The function
 * handles both single and dual link configurations, calculating the GCD
 * of the LMFC frequencies for each link when applicable.
 *
 * @param adc_clk The ADC clock frequency in Hz. Must be a positive value.
 * @param cddc_dcm An array of 4 elements representing the coarse DCM values for
 * the ADC DDC. Must not be null.
 * @param fddc_dcm An array of 8 elements representing the fine DCM values for
 * the ADC DDC. Must not be null.
 * @param links An enumeration value indicating the JESD link selection. Must be
 * a valid value from the `adi_ad9081_jesd_link_select_e`
 * enumeration.
 * @param jesd_param An array of 2 `adi_cms_jesd_param_t` structures containing
 * JESD parameters. Must not be null and should contain valid
 * configurations.
 * @param lmfc_freq A pointer to a uint64_t where the calculated LMFC frequency
 * will be stored. Must not be null; otherwise, an error will
 * be returned.
 * @return Returns an error code indicating success or failure. On success, the
 * calculated LMFC frequency is stored in the location pointed to by
 * `lmfc_freq`.
 ******************************************************************************/
int32_t adi_ad9081_sync_calc_jtx_lmfc_lemc(uint64_t adc_clk,
					   uint8_t cddc_dcm[4],
					   uint8_t fddc_dcm[8],
					   adi_ad9081_jesd_link_select_e links,
					   adi_cms_jesd_param_t jesd_param[2],
					   uint64_t *lmfc_freq);

/***************************************************************************//**
 * @brief This function is used to configure the SYSREF frequency for
 * synchronization in a device. It must be called after the device has
 * been properly initialized. The function calculates the SYSREF
 * frequency based on the provided DAC and ADC clock frequencies,
 * interpolation settings, and JESD parameters. It is important to ensure
 * that all pointer parameters are valid and not null before calling this
 * function, as it performs null checks and will return an error if any
 * are found to be invalid.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param sysref_freq A pointer to a `uint64_t` where the calculated SYSREF
 * frequency will be stored. Must not be null.
 * @param dac_clk The DAC clock frequency in Hz. Must be a positive value.
 * @param adc_clk The ADC clock frequency in Hz. Must be a positive value.
 * @param main_interp The main interpolation factor, which should be a valid
 * value as per device specifications.
 * @param ch_interp The channel interpolation factor, which should be a valid
 * value as per device specifications.
 * @param cddc_dcm An array of 4 `uint8_t` values representing the CDDC DCM
 * settings. Caller retains ownership.
 * @param fddc_dcm An array of 8 `uint8_t` values representing the FDDC DCM
 * settings. Caller retains ownership.
 * @param jtx_links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * indicating the selected JESD links.
 * @param jrx_param A pointer to an `adi_cms_jesd_param_t` structure containing
 * the JESD RX parameters. Must not be null.
 * @param jtx_param An array of 2 `adi_cms_jesd_param_t` structures containing
 * the JESD TX parameters. Caller retains ownership.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the SYSREF
 * frequency has been successfully set. If any input parameters are
 * invalid, the function will return an error code.
 ******************************************************************************/
int32_t adi_ad9081_sync_sysref_frequency_set(
	adi_ad9081_device_t *device, uint64_t *sysref_freq, uint64_t dac_clk,
	uint64_t adc_clk, uint8_t main_interp, uint8_t ch_interp,
	uint8_t cddc_dcm[4], uint8_t fddc_dcm[8],
	adi_ad9081_jesd_link_select_e jtx_links,
	adi_cms_jesd_param_t *jrx_param, adi_cms_jesd_param_t jtx_param[2]);

/***************************************************************************//**
 * @brief This function is used to obtain the phase difference information for
 * the JESD receiver links of the specified device. It must be called
 * with a valid `device` pointer and a non-null `jrx_phase_diff` pointer
 * to store the retrieved phase difference value. The function allows
 * selection of either link 0 or link 1, and it will return an error if
 * the provided link selection is invalid or if any of the pointers are
 * null. It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * that specifies which JESD link(s) to query. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`. If an invalid value is
 * provided, the function will not perform any operations.
 * @param jrx_phase_diff A pointer to a `uint8_t` where the phase difference
 * value will be stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or internal errors.
 ******************************************************************************/
int32_t
adi_ad9081_sync_jrx_tpl_phase_diff_get(adi_ad9081_device_t *device,
				       adi_ad9081_jesd_link_select_e links,
				       uint8_t *jrx_phase_diff);

/*===== A 3 . 0   I R Q S   =====*/

/*===== A 3 . 1   D A C  D P  I R Q S   =====*/
/***************************************************************************//**
 * @brief This function is used to enable or disable the DAC interrupts for a
 * specified device. It must be called with a valid device pointer that
 * has been properly initialized. If the device pointer is null, the
 * function will return an error without making any changes. The `enable`
 * parameter determines which interrupts to enable or disable, and it
 * should be set according to the specific requirements of the
 * application. It is important to ensure that this function is called in
 * the appropriate context where interrupt handling is safe.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A 64-bit integer that specifies which interrupts to enable or
 * disable. The valid range depends on the specific interrupt
 * configuration of the device.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the interrupts
 * have been successfully enabled or disabled. If an error occurs, an
 * appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_irqs_enable_set(adi_ad9081_device_t *device,
				       uint64_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the current interrupt request (IRQ)
 * status from the DAC device. It must be called after the device has
 * been properly initialized. The function expects a valid pointer to a
 * `device` structure and a pointer to a `status` variable where the IRQ
 * status will be stored. If either pointer is null, the function will
 * return an error. The IRQ status is represented as a 64-bit value, and
 * the function will populate the `status` variable with this value upon
 * successful execution.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the DAC device. Must not be null.
 * @param status A pointer to a 64-bit integer where the IRQ status will be
 * stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the IRQ status
 * has been successfully retrieved and stored in the provided `status`
 * variable. If an error occurs, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_irqs_status_get(adi_ad9081_device_t *device,
				       uint64_t *status);

/***************************************************************************//**
 * @brief This function is used to clear specific interrupt request (IRQ)
 * statuses for the DAC associated with the provided device. It should be
 * called when the application needs to reset the IRQ status after
 * handling an interrupt. The `device` parameter must be a valid pointer
 * to an initialized `adi_ad9081_device_t` structure. If the pointer is
 * null, the function will return an error. The `clear` parameter
 * specifies which IRQ statuses to clear, and it should be a valid
 * bitmask corresponding to the IRQs defined in the device's
 * documentation.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error will be
 * returned.
 * @param clear A 64-bit unsigned integer representing the IRQ statuses to
 * clear. The valid values depend on the specific IRQs defined for
 * the device.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the IRQ
 * statuses have been cleared. If an error occurs, an appropriate error
 * code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_irqs_status_clr(adi_ad9081_device_t *device,
				       uint64_t clear);

/*===== A 3 . 2   A D C  D P  I R Q S   =====*/

/*===== A 3 . 3   S E R D E S  I R Q S   =====*/
/***************************************************************************//**
 * @brief This function is used to enable or disable the CRC interrupt for the
 * specified JESD links on the `adi_ad9081_device_t`. It should be called
 * after the device has been properly initialized. The `links` parameter
 * allows selection of one or both JESD links, and the `enable` parameter
 * determines whether the interrupt is enabled (non-zero value) or
 * disabled (zero value). If an invalid `device` pointer is provided, the
 * function will return an error. It is important to ensure that the
 * specified links are valid and that the device is in a state that
 * allows configuration.
 *
 * @param device Pointer to the `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to configure. Valid values
 * are `AD9081_LINK_0`, `AD9081_LINK_1`, or a combination of both.
 * @param enable A uint8_t value that specifies whether to enable (non-zero) or
 * disable (zero) the CRC interrupt.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_crc_irq_enable(adi_ad9081_device_t *device,
				       adi_ad9081_jesd_link_select_e links,
				       uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the CRC interrupt request (IRQ) status
 * for the specified JESD links of the device. It must be called after
 * the device has been properly initialized. The function checks the
 * specified links and retrieves the corresponding status based on the
 * device revision. If the provided `device` pointer is null, the
 * function will return an error. The `status` parameter must point to a
 * valid memory location where the function can store the retrieved
 * status. If the specified links are not valid or if there are issues in
 * retrieving the status, appropriate error codes will be returned.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to check. Valid values are `AD9081_LINK_0` and
 * `AD9081_LINK_1`.
 * @param status A pointer to a `uint8_t` where the function will store the IRQ
 * status. Must point to a valid memory location.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while other values indicate specific error conditions.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_crc_irq_status_get(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links,
					   uint8_t *status);

/***************************************************************************//**
 * @brief This function is used to clear the CRC interrupt for the specified
 * JESD links on the device. It should be called when the application
 * needs to reset the interrupt status after handling a CRC error. The
 * function expects a valid `device` pointer and a bitmask indicating
 * which links to clear. It is important to ensure that the device is
 * properly initialized before calling this function. If an invalid link
 * is specified or if the device pointer is null, the function will
 * handle these cases gracefully by returning an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to clear. Valid values are `AD9081_LINK_0` and
 * `AD9081_LINK_1`. If no valid links are specified, the function
 * will not perform any operations.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the interrupt
 * has been cleared. If an error occurs during the operation, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_crc_irq_clr(adi_ad9081_device_t *device,
				    adi_ad9081_jesd_link_select_e links);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD RX 204C MB
 * interrupt for specified links on the `adi_ad9081_device_t`. It must be
 * called with a valid device pointer that has been properly initialized.
 * The `links` parameter allows selection of which JESD links to
 * configure, and the `enable` parameter determines whether the interrupt
 * is enabled (non-zero) or disabled (zero). If an invalid link is
 * specified or if the device pointer is null, the function will handle
 * these cases gracefully by returning an error. It is important to
 * ensure that the device revision is compatible with the operation, as
 * different revisions may have different register settings.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to enable or disable. Valid
 * values are `AD9081_LINK_0` and `AD9081_LINK_1`. If no valid link
 * is specified, the function will not perform any action.
 * @param enable A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) the interrupt. This value must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails due to invalid parameters or device state.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_mb_irq_enable(adi_ad9081_device_t *device,
				      adi_ad9081_jesd_link_select_e links,
				      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the IRQ status for the JESD receiver
 * links of the specified device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `links` parameter
 * allows the caller to specify which JESD links to query, and the
 * function will populate the `status` pointer with the IRQ status
 * information. The function handles multiple link selections and
 * retrieves the appropriate status based on the device revision. If the
 * `device` pointer or `status` pointer is null, the function will return
 * an error. It is important to ensure that the device is in a valid
 * state before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * indicating which JESD links to query. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param status A pointer to a `uint8_t` where the IRQ status will be stored.
 * Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the IRQ status
 * has been successfully retrieved. If an error occurs, a negative error
 * code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_mb_irq_status_get(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t *status);

/***************************************************************************//**
 * @brief This function is used to clear the JESD RX 204C MB interrupt for
 * specified links on the device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `links` parameter
 * allows the caller to specify which JESD links to clear the interrupt
 * for, and it can include multiple links. The function checks the device
 * revision and performs the appropriate actions based on the revision.
 * If an invalid `device` pointer is provided, the function will return
 * an error. Additionally, if the specified links do not correspond to
 * valid configurations, the function will handle these cases gracefully.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must be initialized before
 * calling this function.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to clear the interrupt for. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`. The function can handle
 * multiple links specified in the bitmask.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the interrupt
 * has been cleared. If an error occurs during the operation, an
 * appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_204c_mb_irq_clr(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD RX 204C SH
 * interrupt for specified links on the `adi_ad9081_device_t`. It must be
 * called with a valid device pointer that has been properly initialized.
 * The `links` parameter allows selection of which JESD links to
 * configure, and the `enable` parameter determines whether the interrupt
 * is enabled (non-zero) or disabled (zero). If an invalid link is
 * specified or if the device pointer is null, the function will handle
 * these cases gracefully by returning an error. It is important to
 * ensure that the device revision is compatible with the operation, as
 * different revisions may have different register settings.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param links Bitmask indicating which JESD links to configure. Valid values
 * are `AD9081_LINK_0` and `AD9081_LINK_1`. If no valid link is
 * specified, the function will return an error.
 * @param enable A uint8_t value that indicates whether to enable (non-zero) or
 * disable (zero) the interrupt. Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_sh_irq_enable(adi_ad9081_device_t *device,
				      adi_ad9081_jesd_link_select_e links,
				      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the interrupt status for the JESD RX
 * 204C interface of the specified device. It must be called with a valid
 * `device` pointer that has been properly initialized. The `links`
 * parameter specifies which JESD links to query, and the function will
 * update the `status` pointer with the retrieved interrupt information.
 * The function handles two links, and the behavior may vary depending on
 * the device revision. If an invalid `device` pointer is provided, the
 * function will return an error. Additionally, if the specified links
 * are not valid, the function will not perform any operations related to
 * those links.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links An enumeration value of type `adi_ad9081_jesd_link_select_e`
 * indicating which JESD links to query. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`.
 * @param status A pointer to a `uint8_t` where the interrupt status will be
 * stored. Caller retains ownership and must ensure it points to a
 * valid memory location.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the status has
 * been successfully retrieved. If an error occurs, an appropriate error
 * code will be returned.
 ******************************************************************************/
int32_t
adi_ad9081_jesd_rx_204c_sh_irq_status_get(adi_ad9081_device_t *device,
					  adi_ad9081_jesd_link_select_e links,
					  uint8_t *status);

/***************************************************************************//**
 * @brief This function is used to clear the JESD RX 204C short interrupt for
 * specified links on the device. It must be called with a valid `device`
 * pointer that has been properly initialized. The `links` parameter
 * allows the caller to specify which JESD links to clear the interrupt
 * for, and it can include multiple links. The function checks the device
 * revision and performs the necessary operations to clear the interrupt
 * for each specified link. If an invalid `device` pointer is provided,
 * or if any operation fails, the function will return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must be initialized before
 * calling this function.
 * @param links A bitmask of type `adi_ad9081_jesd_link_select_e` indicating
 * which JESD links to clear the interrupt for. Valid values are
 * `AD9081_LINK_0` and `AD9081_LINK_1`. The function handles
 * invalid values by ignoring them.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_204c_sh_irq_clr(adi_ad9081_device_t *device,
					   adi_ad9081_jesd_link_select_e links);

/*===== A 4 . 0   L O O P B A C K  T E S T  M O D E S   =====*/
/***************************************************************************//**
 * @brief This function configures the JESD loopback mode of the specified
 * device. It must be called with a valid device pointer that has been
 * properly initialized. The mode parameter determines the specific
 * loopback configuration to be applied. If the device pointer is null,
 * the function will return an error without making any changes. It is
 * important to ensure that the device is in a state that allows for mode
 * changes before invoking this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null; otherwise, an error is returned.
 * @param mode An 8-bit unsigned integer representing the desired loopback mode.
 * The valid range of values for this parameter should be defined in
 * the device's documentation. Invalid values may result in an error
 * being returned.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the loopback
 * mode has been set successfully. If an error occurs, a negative error
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_loopback_mode_set(adi_ad9081_device_t *device,
					  uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the direct loopback settings for the
 * specified device. It must be called with a valid device pointer that
 * has been properly initialized. The `mode` parameter determines the
 * type of loopback operation, while the `mapping` parameter specifies
 * the mapping configuration. If either parameter is invalid or if the
 * device pointer is null, the function will handle the error
 * appropriately. It is important to ensure that the device is in a state
 * that allows for loopback configuration before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param mode A `uint8_t` value representing the loopback mode. Valid values
 * depend on the device specifications.
 * @param mapping A `uint8_t` value representing the mapping configuration.
 * Valid values depend on the device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_device_direct_loopback_set(adi_ad9081_device_t *device,
					      uint8_t mode, uint8_t mapping);

/*===== A 5 . 0   A D C  S I G N A L  M O N I T O R I N G    =====*/
/***************************************************************************//**
 * @brief This function configures the GPIO pin used for enabling the ADC status
 * monitor feature on the specified device. It should be called after the
 * device has been properly initialized. The `smon_en_gpio` parameter
 * determines which GPIO pin will be used for this purpose, with specific
 * values corresponding to different GPIOs or disable options. If an
 * invalid GPIO value is provided, the function will handle it gracefully
 * by returning an error code. It is important to ensure that the
 * `device` pointer is valid and not null before calling this function.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param smon_en_gpio An 8-bit unsigned integer specifying the GPIO mapping for
 * the ADC status monitor enable. Valid values are 0
 * (disable), 1 (enable), or 2-8 for specific GPIO pins
 * (6-10, syncinb1_p, syncinb1_n). Invalid values will
 * result in an error.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_en_to_gpio_mapping_set(adi_ad9081_device_t *device,
						   uint8_t smon_en_gpio);

/***************************************************************************//**
 * @brief This function is used to enable or disable the clock for one or more
 * ADCs in the device. It should be called after the device has been
 * properly initialized. The `adcs` parameter specifies which ADCs to
 * modify, and the `enable` parameter determines whether to turn the
 * clock on or off. If the `device` pointer is null, the function will
 * return an error. Additionally, if any of the ADC selection or hardware
 * register operations fail, the function will also return an error.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to enable or disable. Each bit
 * corresponds to an ADC, with the least significant bit
 * representing ADC 0.
 * @param enable A boolean value where a non-zero value enables the clock and
 * zero disables it.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an invalid
 * input is provided or if an operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_clk_enable_set(adi_ad9081_device_t *device,
					   uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the SFRAMER mode for one or
 * more ADCs in the device. It should be called after the device has been
 * properly initialized. The `adcs` parameter specifies which ADCs to
 * configure, and the `enable` parameter determines whether to enable or
 * disable the mode. If an invalid pointer is provided for the `device`,
 * or if an error occurs while selecting the ADC or setting the mode, the
 * function will return an error code. It is important to ensure that the
 * `adcs` parameter only includes valid ADC indices.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC (0-3). Valid values are 0 to 15.
 * @param enable A boolean value indicating whether to enable (non-zero) or
 * disable (zero) the SFRAMER mode.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_sframer_mode_enable_set(adi_ad9081_device_t *device,
						    uint8_t adcs,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to obtain the frame counter values for one or
 * more ADCs associated with the specified device. It should be called
 * after the device has been properly initialized. The `adcs` parameter
 * allows the selection of which ADCs to query, and the results are
 * written to the `fcnt` pointer. It is important to ensure that the
 * `fcnt` pointer is valid and points to a writable memory location. If
 * an invalid device pointer is provided or if there are errors during
 * the selection or retrieval process, the function will handle these
 * gracefully by returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to query. Each bit corresponds to
 * an ADC, where a value of 1 indicates the ADC is selected.
 * @param fcnt A pointer to a `uint8_t` where the frame counter value will be
 * stored. Must point to a valid memory location.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_frame_counter_get(adi_ad9081_device_t *device,
					      uint8_t adcs, uint8_t *fcnt);

/***************************************************************************//**
 * @brief This function configures the monitoring period for one or more ADCs in
 * the device. It should be called after the device has been properly
 * initialized and before any monitoring operations are performed. The
 * `adcs` parameter allows selection of multiple ADCs, and the `period`
 * parameter specifies the desired monitoring period. If the `device`
 * pointer is null, the function will return an error. Additionally, if
 * any of the ADC selection or setting operations fail, the function will
 * also return an error.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC (0-3). Valid values are from 0 to 15.
 * @param period The monitoring period to set for the selected ADCs. Must be a
 * valid non-negative integer.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_period_set(adi_ad9081_device_t *device,
				       uint8_t adcs, uint32_t period);

/***************************************************************************//**
 * @brief This function is used to obtain the status of one or more ADCs
 * specified by the `adcs` parameter. It must be called with a valid
 * `device` pointer that has been properly initialized. The `status`
 * parameter is expected to point to a valid memory location where the
 * function will store the retrieved status information. If the `adcs`
 * parameter includes invalid ADC selections, the function will handle
 * these gracefully, ensuring that only valid ADCs are queried. It is
 * important to ensure that the `status` pointer is not null before
 * calling this function, as it will result in an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to query. Each bit corresponds to
 * an ADC, where a value of 1 indicates the ADC should be included
 * in the status retrieval.
 * @param status A pointer to a `uint32_t` where the status will be stored. Must
 * not be null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the status has
 * been successfully retrieved. If an error occurs during the operation,
 * an appropriate error code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_status_get(adi_ad9081_device_t *device,
				       uint8_t adcs, uint32_t *status);

/***************************************************************************//**
 * @brief This function configures the monitoring thresholds for specified ADCs
 * in the device. It should be called after the device has been properly
 * initialized and is ready for configuration. The function takes a
 * bitmask to select which ADCs to configure, allowing for multiple ADCs
 * to be set in a single call. It is important to ensure that the
 * thresholds are within valid ranges, as invalid values may lead to
 * undefined behavior. The function will return an error if the device
 * pointer is null or if any of the operations to set the thresholds
 * fail.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates that the
 * respective ADC should be configured.
 * @param thresh_low The lower threshold value for the ADC monitoring. Must be
 * within the valid range defined by the device
 * specifications.
 * @param thresh_high The upper threshold value for the ADC monitoring. Must be
 * greater than `thresh_low` and within the valid range
 * defined by the device specifications.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_thresh_set(adi_ad9081_device_t *device,
				       uint8_t adcs, uint16_t thresh_low,
				       uint16_t thresh_high);

/***************************************************************************//**
 * @brief This function is used to update the status monitoring of specified
 * ADCs in the device. It should be called after the device has been
 * properly initialized and configured. The `adcs` parameter allows the
 * caller to specify which ADCs to update, using a bitmask where each bit
 * corresponds to an ADC. The `update` parameter indicates the new status
 * value to be set. If the `device` pointer is null or if any of the
 * specified ADCs are invalid, the function will handle these cases
 * gracefully by returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to update. Each bit corresponds
 * to an ADC (0-3). Valid values are 0 to 15.
 * @param update A value representing the new status to be set for the selected
 * ADCs. The valid range depends on the specific status values
 * defined for the ADCs.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_status_update_set(adi_ad9081_device_t *device,
					      uint8_t adcs, uint8_t update);

/***************************************************************************//**
 * @brief This function is used to configure the status read selection for one
 * or more ADCs in the device. It should be called after the device has
 * been properly initialized. The `adcs` parameter specifies which ADCs
 * to configure, and the `select` parameter determines the selection
 * setting for those ADCs. If the `device` pointer is null, the function
 * will return an error. Additionally, if any errors occur during the
 * configuration of the ADCs, the function will return an error code,
 * ensuring that the caller is informed of any issues.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates that the
 * respective ADC should be configured.
 * @param select An 8-bit value representing the selection setting for the
 * specified ADCs. Valid values depend on the specific
 * configuration options available for the ADCs.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates that the
 * operation was successful.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_status_read_select_set(adi_ad9081_device_t *device,
						   uint8_t adcs,
						   uint8_t select);

/***************************************************************************//**
 * @brief This function is used to enable or disable the peak detector feature
 * for one or more ADCs in the device. It should be called after the
 * device has been properly initialized. The `adcs` parameter specifies
 * which ADCs to configure, and the `enable` parameter determines whether
 * to enable (non-zero value) or disable (zero value) the peak detector.
 * If an invalid pointer is provided for the `device`, or if an error
 * occurs while selecting the ADC or setting the configuration, the
 * function will handle these cases appropriately. It is important to
 * ensure that the `adcs` parameter only includes valid ADC indices.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC (0-3). Valid values are from 0 to 15.
 * @param enable A value indicating whether to enable (non-zero) or disable
 * (zero) the peak detector. Must be either 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating that the operation
 * was completed without errors.
 ******************************************************************************/
int32_t
adi_ad9081_adc_smon_peak_detector_enable_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the JLINK selection for one or more
 * ADCs in the device. It should be called after the device has been
 * properly initialized. The `adcs` parameter specifies which ADCs to
 * configure, and the `select` parameter determines the JLINK selection
 * value. If the `device` pointer is null, the function will return an
 * error. The function handles multiple ADCs by iterating through the
 * specified bits in `adcs`, and it will only configure those ADCs that
 * are indicated by the corresponding bits being set.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC0, 1 for
 * ADC1, etc.).
 * @param select An 8-bit value representing the JLINK selection. The valid
 * range is implementation-specific, but it should be within the
 * expected operational parameters of the device.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_jlink_select_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t select);

/***************************************************************************//**
 * @brief This function configures the GPIO output for status monitoring of
 * specified ADCs. It should be called after initializing the
 * `adi_ad9081_device_t` structure and before using the ADCs for data
 * acquisition. The `adcs` parameter allows selection of which ADCs to
 * configure, while the `enable` parameter determines whether the GPIO
 * output is enabled or disabled. If the `device` pointer is null, the
 * function will return an error. Additionally, if any errors occur
 * during the configuration of the ADCs or GPIO settings, the function
 * will return an error code.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param adcs Bitmask indicating which ADCs to configure (0-3). Each bit
 * corresponds to an ADC, where bit 0 is ADC 0, bit 1 is ADC 1, and
 * so on.
 * @param enable Boolean value indicating whether to enable (non-zero) or
 * disable (zero) the GPIO output for the selected ADCs.
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_output_via_gpio_set(adi_ad9081_device_t *device,
						uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the SFRAMER feature for one
 * or more ADCs in the device. It should be called after the device has
 * been properly initialized. The `adcs` parameter specifies which ADCs
 * to configure, and the `enable` parameter determines whether to enable
 * (non-zero value) or disable (zero value) the SFRAMER. If an invalid
 * pointer is provided for the `device`, the function will return an
 * error. Additionally, if an error occurs while selecting an ADC or
 * setting the SFRAMER state, the function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC0, 1 for
 * ADC1, etc.).
 * @param enable A value indicating whether to enable (non-zero) or disable
 * (zero) the SFRAMER. Valid values are 0 or 1.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during execution.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_sframer_enable_set(adi_ad9081_device_t *device,
					       uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the SFRAMER mode for one or more ADCs in the
 * device. It should be called after the device has been properly
 * initialized and configured. The `adcs` parameter specifies which ADCs
 * to configure, and the `mode` parameter defines the desired SFRAMER
 * mode. If the `device` pointer is null, the function will return an
 * error. Additionally, if an invalid ADC selection is made or if there
 * are issues setting the mode, the function will also return an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Valid values are
 * from 0 to 15, where each bit represents an ADC (0 for ADC0, 1 for
 * ADC1, etc.). If no bits are set, no ADCs will be configured.
 * @param mode An 8-bit value representing the desired SFRAMER mode. The valid
 * range of modes should be defined in the device's documentation.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_sframer_mode_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function configures the input selection for specified ADCs in the
 * device. It should be called after the device has been properly
 * initialized. The `adcs` parameter allows the selection of one or more
 * ADCs, while the `select` parameter specifies the input source for
 * those ADCs. If either parameter is invalid, the function will handle
 * the error gracefully. It is important to ensure that the `device`
 * pointer is not null before calling this function.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure. Each bit
 * corresponds to an ADC, where a value of 1 indicates selection.
 * Valid values are 0 to 15.
 * @param select An 8-bit value representing the input source selection for the
 * specified ADCs. Valid values depend on the specific input
 * sources available in the device.
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if an error
 * occurs during the operation.
 ******************************************************************************/
int32_t
adi_ad9081_adc_smon_sframer_input_select_set(adi_ad9081_device_t *device,
					     uint8_t adcs, uint8_t select);

/***************************************************************************//**
 * @brief This function is used to enable or disable the synchronization feature
 * for one or more ADCs in the device. It should be called after the
 * device has been properly initialized. The `adcs` parameter specifies
 * which ADCs to modify, using a bitmask where each bit corresponds to an
 * ADC. The `enable` parameter determines whether to enable (non-zero
 * value) or disable (zero value) the synchronization feature. If an
 * invalid pointer is provided for the `device`, or if an error occurs
 * while selecting the ADC or setting the synchronization feature, the
 * function will handle these cases appropriately.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to enable or disable
 * synchronization for. Valid values are from 0 to 15, where each
 * bit represents an ADC.
 * @param enable A value indicating whether to enable (non-zero) or disable
 * (zero) synchronization. Valid values are 0 or 1.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of `API_CMS_ERROR_OK` indicates
 * success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_sync_enable_set(adi_ad9081_device_t *device,
					    uint8_t adcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function configures the next synchronization mode for one or more
 * ADCs in the device. It should be called after the device has been
 * properly initialized and configured. The `adcs` parameter specifies
 * which ADCs to configure, and the `mode` parameter determines the
 * synchronization mode (0 for continuous mode, 1 for next sync mode). If
 * an invalid pointer is provided for the `device`, or if an error occurs
 * while setting the ADC selection or synchronization mode, the function
 * will handle these cases gracefully by returning an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param adcs A bitmask indicating which ADCs to configure (0-3). Each bit
 * corresponds to an ADC, where bit 0 represents ADC 0, bit 1
 * represents ADC 1, and so on.
 * @param mode An integer representing the synchronization mode. Valid values
 * are 0 (continuous mode) and 1 (next sync mode).
 * @return Returns an error code indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success.
 ******************************************************************************/
int32_t adi_ad9081_adc_smon_next_sync_mode_set(adi_ad9081_device_t *device,
					       uint8_t adcs, uint8_t mode);

/*===== A 6 . 0   S P I  C O N T R O L S   =====*/
/***************************************************************************//**
 * @brief This function is used to enable or disable the DAC SPI interface of
 * the specified device. It must be called with a valid `device` pointer
 * that has been properly initialized. The `enable` parameter determines
 * whether the SPI interface is activated (when set to a non-zero value)
 * or deactivated (when set to zero). If the `device` pointer is null,
 * the function will return an error without making any changes. It is
 * important to ensure that this function is called in the appropriate
 * context where the device is ready for configuration.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param enable A `uint8_t` value that indicates whether to enable (non-zero)
 * or disable (zero) the DAC SPI interface. Valid values are 0
 * (disable) or any non-zero value (enable).
 * @return Returns `API_CMS_ERROR_OK` on success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_dac_spi_enable_set(adi_ad9081_device_t *device,
				      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure a specific register in the CBUSJRX
 * interface of the device. It should be called after the device has been
 * properly initialized. The function allows the user to specify the
 * register address, the data to be written, and the lane to which the
 * register belongs. It is important to ensure that the `device` pointer
 * is valid and not null before calling this function. If invalid
 * parameters are provided, the function may return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr The address of the register to be set. Valid values depend on the
 * specific device's register map.
 * @param data The data value to write to the specified register. Valid values
 * depend on the register's specifications.
 * @param lane The lane number for the register operation. Valid values are
 * typically defined by the device's architecture.
 * @return Returns an integer status code indicating the success or failure of
 * the operation.
 ******************************************************************************/
int32_t adi_ad9081_device_cbusjrx_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data,
					       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to read a specific register value from the CBUS
 * JRX of the device. It should be called after the device has been
 * properly initialized. The `addr` parameter specifies the register
 * address to read from, while the `data` parameter is a pointer to a
 * buffer where the retrieved value will be stored. The `lane` parameter
 * indicates which lane to access. If the provided `data` pointer is
 * null, the function will not perform the read operation, and the return
 * value will indicate an error.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr The address of the register to read. Valid values depend on the
 * device's register map.
 * @param data A pointer to a `uint8_t` where the register value will be stored.
 * Must not be null.
 * @param lane The lane number to access. Valid values are typically defined by
 * the device specifications.
 * @return Returns a 32-bit integer indicating the success or failure of the
 * operation. A non-negative value typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_cbusjrx_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data,
					       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to configure a specific register in the CBUS JTX
 * interface of the `adi_ad9081_device_t` device. It should be called
 * after the device has been properly initialized. The function allows
 * the user to specify the register address, the data to be written, and
 * the lane to which the register belongs. It is important to ensure that
 * the `device` pointer is valid and that the `addr`, `data`, and `lane`
 * parameters are within acceptable ranges to avoid undefined behavior.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and must point to a valid
 * initialized device.
 * @param addr The address of the register to be set. This should be within the
 * valid range of register addresses for the device.
 * @param data The value to write to the specified register. This should be a
 * valid value as per the register's specifications.
 * @param lane The lane number for the register operation. This should be within
 * the valid range of lanes supported by the device.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_cbusjtx_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data,
					       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to obtain the value stored in a specific CBUS
 * JTX register of the device. It should be called after the device has
 * been properly initialized. The function takes an address to specify
 * which register to read from, and it requires a pointer to a buffer
 * where the retrieved data will be stored. The lane parameter allows
 * selection of the specific lane for the operation. It is important to
 * ensure that the `data` pointer is valid and points to a memory
 * location that can hold the result.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr The address of the CBUS JTX register to read. Valid values depend
 * on the specific device's register map.
 * @param data A pointer to a `uint8_t` where the register value will be stored.
 * Must not be null.
 * @param lane The lane number to read from. Valid values depend on the device's
 * configuration.
 * @return Returns the status of the operation as an `int32_t`. A value of 0
 * typically indicates success, while a non-zero value indicates an
 * error.
 ******************************************************************************/
int32_t adi_ad9081_device_cbusjtx_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data,
					       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to configure the CBUS PLL by writing a specific
 * value to a designated register. It should be called after the device
 * has been properly initialized to ensure that the device is ready to
 * accept register configurations. The function takes a device handle, a
 * register address, and the data to be written. It is important to
 * ensure that the address corresponds to a valid register for the CBUS
 * PLL; otherwise, the behavior is undefined.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null and should point to a valid
 * initialized device.
 * @param addr The address of the register to be set. This should be within the
 * valid range of register addresses for the CBUS PLL.
 * @param data The value to write to the specified register. This should be a
 * valid value as per the register's specifications.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_device_cbuspll_register_set(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to read the value of a specific register from
 * the CBUS PLL of the `adi_ad9081_device_t` device. It should be called
 * after the device has been properly initialized. The `addr` parameter
 * specifies the register address to read from, and the `data` parameter
 * is a pointer to a location where the retrieved value will be stored.
 * It is important to ensure that the `data` pointer is not null before
 * calling this function, as passing a null pointer will lead to
 * undefined behavior. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param addr A `uint8_t` value representing the address of the register to
 * read. Valid addresses depend on the specific device
 * configuration.
 * @param data A pointer to a `uint8_t` where the register value will be stored.
 * Must not be null.
 * @return Returns an `int32_t` status code indicating the result of the
 * operation, where a value of 0 typically indicates success.
 ******************************************************************************/
int32_t adi_ad9081_device_cbuspll_register_get(adi_ad9081_device_t *device,
					       uint8_t addr, uint8_t *data);

/*=====   E X T R A  L O W  L E V E L  A P I S   =====*/

/* FIXME */
/***************************************************************************//**
 * @brief This function is used to initiate a test data generation process for
 * the JESD transmitter of the specified device. It must be called after
 * the device has been properly initialized. The function allows the user
 * to select specific JESD links, choose a data source, and set the test
 * mode. It is important to ensure that the `device` parameter is not
 * null before calling this function, as it will return an error if it
 * is. The function handles multiple links, and if any link is selected,
 * it will configure the test settings accordingly. If any operation
 * fails during the configuration, an error will be returned.
 *
 * @param device A pointer to the `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param links A bitmask of `adi_ad9081_jesd_link_select_e` values indicating
 * which JESD links to configure. Valid values depend on the
 * device's capabilities.
 * @param data_source An enumeration value of type
 * `adi_ad9081_jesd_tx_test_data_src_e` that specifies the
 * source of the test data. Must be a valid enumeration
 * value.
 * @param test_mode An enumeration value of type
 * `adi_ad9081_jesd_tx_test_mode_e` that specifies the mode of
 * the test. Must be a valid enumeration value.
 * @return Returns `API_CMS_ERROR_OK` on successful execution, or an error code
 * if any operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_gen_test(adi_ad9081_device_t *device,
			    adi_ad9081_jesd_link_select_e links,
			    adi_ad9081_jesd_tx_test_data_src_e data_source,
			    adi_ad9081_jesd_tx_test_mode_e test_mode);

/***************************************************************************//**
 * @brief This function is used to convert a signed integer into its two's
 * complement representation based on the specified bit length. It should
 * be called with a valid `device` pointer and a valid `bit_length` that
 * defines the size of the output. The function checks if the
 * `input_value` is within the valid range for the specified
 * `bit_length`, and if it is not, it returns an error code. The
 * resulting two's complement value is stored in the `data` pointer
 * provided by the caller, which must not be null.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param input_value The signed integer value to be converted. It must be
 * within the range of -2^(bit_length-1) to
 * 2^(bit_length-1)-1.
 * @param bit_length The number of bits for the two's complement representation.
 * Valid values are typically between 1 and 32.
 * @param data Pointer to a `uint8_t` where the resulting two's complement value
 * will be stored. Caller retains ownership and must ensure this
 * pointer is not null.
 * @return Returns `API_CMS_ERROR_OK` on success, indicating the two's
 * complement value has been successfully written to `data`. If the
 * `input_value` is out of the valid range, it returns
 * `API_CMS_ERROR_INVALID_PARAM`.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_gen_2s_comp(adi_ad9081_device_t *device,
				       int8_t input_value, uint8_t bit_length,
				       uint8_t *data);

/***************************************************************************//**
 * @brief This function configures the Serial Peripheral Interface (SPI) output
 * for a specific lane of the device. It must be called with a valid
 * `device` pointer that has been properly initialized. The `lane`
 * parameter specifies which lane to configure, and it should be within
 * the valid range of lanes supported by the device. The `spo` parameter
 * determines the SPI output setting and is masked to ensure it remains
 * within acceptable limits. If any of the parameters are invalid, the
 * function will handle the error appropriately and return an error code.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param lane An 8-bit unsigned integer representing the lane to configure.
 * Valid values depend on the device's specifications.
 * @param spo An 8-bit unsigned integer representing the SPI output setting. The
 * value is masked to the lower 7 bits, so valid values are 0 to 127.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while other values indicate specific error conditions.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_spo_set(adi_ad9081_device_t *device, uint8_t lane,
				   uint8_t spo);

/***************************************************************************//**
 * @brief This function is used to configure the fine synchronization next value
 * for one or more digital down converters (DDCs) associated with the
 * specified device. It should be called after the device has been
 * properly initialized and configured. The function iterates through the
 * specified DDCs, applying the provided value to each active DDC. If the
 * `device` pointer is null, the function will return an error.
 * Additionally, if any internal operation fails, an error will be
 * returned, indicating the failure.
 *
 * @param device A pointer to an `adi_ad9081_device_t` structure representing
 * the device. Must not be null.
 * @param fddcs A bitmask indicating which DDCs to configure. Each bit
 * corresponds to a DDC, where a set bit indicates that the DDC is
 * active.
 * @param val The value to set for the fine synchronization next. This value
 * should be within the valid range expected by the device.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of `API_CMS_ERROR_OK` indicates success,
 * while any other value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_sync_next_set(adi_ad9081_device_t *device,
					      uint8_t fddcs, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* __ADI_AD9081_H__ */
/*! @} */
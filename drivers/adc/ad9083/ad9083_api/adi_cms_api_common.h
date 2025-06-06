/*!
 * @brief     Common API definitions header file.
 *            This file contains all common API definitions.
 *
 * @copyright copyright(c) 2020 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup ADI_API_COMMON
 * @{
 */
#ifndef __ADI_CMS_API_COMMON_H__
#define __ADI_CMS_API_COMMON_H__

/*============= I N C L U D E S ============*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "adi_cms_api_config.h"

/*============= D E F I N E S ==============*/
/***************************************************************************//**
 * @brief The `adi_cms_error_e` is an enumeration that defines a set of error
 * codes used in the Analog Devices API to indicate various error
 * conditions. Each enumerator represents a specific error scenario, such
 * as general errors, invalid parameters, hardware-related issues, and
 * specific operational failures like PLL not locking or SPI transfer
 * errors. These error codes are used to facilitate error handling and
 * debugging in applications using the API.
 *
 * @param API_CMS_ERROR_OK No Error.
 * @param API_CMS_ERROR_ERROR General Error.
 * @param API_CMS_ERROR_NULL_PARAM Null parameter.
 * @param API_CMS_ERROR_SPI_SDO Wrong value assigned to the SDO in device
 * structure.
 * @param API_CMS_ERROR_INVALID_HANDLE_PTR Device handler pointer is invalid.
 * @param API_CMS_ERROR_INVALID_XFER_PTR Invalid pointer to the SPI xfer
 * function assigned.
 * @param API_CMS_ERROR_INVALID_DELAYUS_PTR Invalid pointer to the delay_us
 * function assigned.
 * @param API_CMS_ERROR_INVALID_PARAM Invalid parameter passed.
 * @param API_CMS_ERROR_INVALID_RESET_CTRL_PTR Invalid pointer to the reset
 * control function assigned.
 * @param API_CMS_ERROR_NOT_SUPPORTED Not supported.
 * @param API_CMS_ERROR_VCO_OUT_OF_RANGE The VCO is out of range.
 * @param API_CMS_ERROR_PLL_NOT_LOCKED PLL is not locked.
 * @param API_CMS_ERROR_DLL_NOT_LOCKED DLL is not locked.
 * @param API_CMS_ERROR_MODE_NOT_IN_TABLE JESD Mode not in table.
 * @param API_CMS_ERROR_FTW_LOAD_ACK FTW acknowledge not received.
 * @param API_CMS_ERROR_NCO_NOT_ENABLED The NCO is not enabled.
 * @param API_CMS_ERROR_INIT_SEQ_FAIL Initialization sequence failed.
 * @param API_CMS_ERROR_TEST_FAILED Test failed.
 * @param API_CMS_ERROR_SPI_XFER SPI transfer error.
 * @param API_CMS_ERROR_TX_EN_PIN_CTRL TX enable function error.
 * @param API_CMS_ERROR_RESET_PIN_CTRL HW reset function error.
 * @param API_CMS_ERROR_EVENT_HNDL Event handling error.
 * @param API_CMS_ERROR_HW_OPEN HW open function error.
 * @param API_CMS_ERROR_HW_CLOSE HW close function error.
 * @param API_CMS_ERROR_LOG_OPEN Log open error.
 * @param API_CMS_ERROR_LOG_WRITE Log write error.
 * @param API_CMS_ERROR_LOG_CLOSE Log close error.
 * @param API_CMS_ERROR_DELAY_US Delay error.
 ******************************************************************************/
typedef enum {
    API_CMS_ERROR_OK                     =  0,  /*!< No Error */
    API_CMS_ERROR_ERROR                  = -1,  /*!< General Error  */
    API_CMS_ERROR_NULL_PARAM             = -2,  /*!< Null parameter */
    API_CMS_ERROR_SPI_SDO                = -10, /*!< Wrong value assigned to the SDO in device structure */
    API_CMS_ERROR_INVALID_HANDLE_PTR     = -11, /*!< Device handler pointer is invalid */
    API_CMS_ERROR_INVALID_XFER_PTR       = -12, /*!< Invalid pointer to the SPI xfer function assigned */
    API_CMS_ERROR_INVALID_DELAYUS_PTR    = -13, /*!< Invalid pointer to the delay_us function assigned */
    API_CMS_ERROR_INVALID_PARAM          = -14, /*!< Invalid parameter passed */
    API_CMS_ERROR_INVALID_RESET_CTRL_PTR = -15, /*!< Invalid pointer to the reset control function assigned */
    API_CMS_ERROR_NOT_SUPPORTED          = -16, /*!< Not supported */
    API_CMS_ERROR_VCO_OUT_OF_RANGE       = -20, /*!< The VCO is out of range */
    API_CMS_ERROR_PLL_NOT_LOCKED         = -21, /*!< PLL is not locked */
    API_CMS_ERROR_DLL_NOT_LOCKED         = -22, /*!< DLL is not locked */
    API_CMS_ERROR_MODE_NOT_IN_TABLE      = -23, /*!< JESD Mode not in table */
    API_CMS_ERROR_FTW_LOAD_ACK           = -30, /*!< FTW acknowledge not received */
    API_CMS_ERROR_NCO_NOT_ENABLED        = -31, /*!< The NCO is not enabled */
    API_CMS_ERROR_INIT_SEQ_FAIL          = -40, /*!< Initialization sequence failed */
    API_CMS_ERROR_TEST_FAILED            = -50, /*!< Test failed */
    API_CMS_ERROR_SPI_XFER               = -60, /*!< SPI transfer error */
    API_CMS_ERROR_TX_EN_PIN_CTRL         = -62, /*!< TX enable function error */
    API_CMS_ERROR_RESET_PIN_CTRL         = -63, /*!< HW reset function error */
    API_CMS_ERROR_EVENT_HNDL             = -64, /*!< Event handling error */
    API_CMS_ERROR_HW_OPEN                = -65, /*!< HW open function error */
    API_CMS_ERROR_HW_CLOSE               = -66, /*!< HW close function error */
    API_CMS_ERROR_LOG_OPEN               = -67, /*!< Log open error */
    API_CMS_ERROR_LOG_WRITE              = -68, /*!< Log write error */
    API_CMS_ERROR_LOG_CLOSE              = -69, /*!< Log close error */
    API_CMS_ERROR_DELAY_US               = -70  /*!< Delay error */
} adi_cms_error_e;

/***************************************************************************//**
 * @brief The `adi_cms_log_type_e` is an enumeration that defines various types
 * of log messages that can be generated by the system. Each enumerator
 * represents a specific category of log message, such as error, warning,
 * or informational messages related to SPI or API operations. The
 * enumeration allows for selective logging by enabling specific log
 * types or all log types at once, facilitating debugging and monitoring
 * of system operations.
 *
 * @param ADI_CMS_LOG_NONE Represents no log type selected with a value of
 * 0x0000.
 * @param ADI_CMS_LOG_ERR Represents an error message log type with a value of
 * 0x0001.
 * @param ADI_CMS_LOG_WARN Represents a warning message log type with a value of
 * 0x0002.
 * @param ADI_CMS_LOG_MSG Represents a tips info message log type with a value
 * of 0x0004.
 * @param ADI_CMS_LOG_SPI Represents an SPI read/write info message log type
 * with a value of 0x0010.
 * @param ADI_CMS_LOG_API Represents an API info message log type with a value
 * of 0x0020.
 * @param ADI_CMS_LOG_ALL Represents all log types selected with a value of
 * 0xFFFF.
 ******************************************************************************/
typedef enum {
    ADI_CMS_LOG_NONE = 0x0000,                  /*!< all not selected */
    ADI_CMS_LOG_ERR  = 0x0001,                  /*!< error message */
    ADI_CMS_LOG_WARN = 0x0002,                  /*!< warning message */
    ADI_CMS_LOG_MSG  = 0x0004,                  /*!< tips info message */
    ADI_CMS_LOG_SPI  = 0x0010,                  /*!< spi r/w info message */
    ADI_CMS_LOG_API  = 0x0020,                  /*!< api info message */
    ADI_CMS_LOG_ALL  = 0xFFFF                   /*!< all selected */
} adi_cms_log_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_chip_id_t` structure is used to store identification data
 * for an ADI device. It includes fields for the chip type, product ID,
 * product grade, and device revision, which collectively provide a
 * unique identifier for the specific device model and version. This
 * structure is essential for distinguishing between different devices
 * and their respective versions in a system.
 *
 * @param chip_type Chip Type Code.
 * @param prod_id Product ID Code.
 * @param prod_grade Product Grade Code.
 * @param dev_revision Device Revision.
 ******************************************************************************/
typedef struct {
    uint8_t  chip_type;                         /*!< Chip Type Code */
    uint16_t prod_id;                           /*!< Product ID Code */
    uint8_t  prod_grade;                        /*!< Product Grade Code */
    uint8_t  dev_revision;                      /*!< Device Revision */
}adi_cms_chip_id_t;

/***************************************************************************//**
 * @brief The `adi_cms_reg_data_t` structure is used to represent a register
 * access operation, containing both the address of the register and the
 * value to be written or read. This structure is typically used in
 * contexts where register-level manipulation is required, such as
 * configuring hardware devices or reading their status.
 *
 * @param reg Register address as a 16-bit unsigned integer.
 * @param val Register value as an 8-bit unsigned integer.
 ******************************************************************************/
typedef struct {    
    uint16_t reg;                               /*!< Register address */
    uint8_t  val;                               /*!< Register value */
}adi_cms_reg_data_t;                    

/***************************************************************************//**
 * @brief The `adi_cms_spi_sdo_config_e` is an enumeration that defines the
 * configuration settings for SPI (Serial Peripheral Interface) modes,
 * specifically focusing on the SDO (Serial Data Out) and SDIO (Serial
 * Data Input/Output) configurations. It provides options for selecting
 * between no SPI mode, a 4-wire SPI mode with SDO active, and a 3-wire
 * SPI mode with SDIO active. The enumeration also includes a boundary
 * marker to denote the maximum configuration value.
 *
 * @param SPI_NONE Represents a test configuration with no SPI mode active.
 * @param SPI_SDO Indicates that the SDO (Serial Data Out) is active, applicable
 * for 4-wire SPI configurations.
 * @param SPI_SDIO Indicates that the SDIO (Serial Data Input/Output) is active,
 * applicable for 3-wire SPI configurations.
 * @param SPI_CONFIG_MAX Serves as a boundary marker for the enumeration,
 * ensuring it is the last element.
 ******************************************************************************/
typedef enum {                  
    SPI_NONE = 0,                               /*!< Keep this for test */
    SPI_SDO = 1,                                /*!< SDO  active, 4-wire only */
    SPI_SDIO = 2,                               /*!< SDIO active, 3-wire only */
    SPI_CONFIG_MAX = 3                          /*!< Keep it last */
}adi_cms_spi_sdo_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_spi_msb_config_e` is an enumeration that defines the bit
 * order configuration for SPI (Serial Peripheral Interface)
 * communication. It provides two options: `SPI_MSB_LAST` for LSB first
 * and `SPI_MSB_FIRST` for MSB first, allowing users to specify the order
 * in which bits are transmitted over the SPI interface.
 *
 * @param SPI_MSB_LAST Represents the configuration for LSB (Least Significant
 * Bit) first in SPI communication.
 * @param SPI_MSB_FIRST Represents the configuration for MSB (Most Significant
 * Bit) first in SPI communication.
 ******************************************************************************/
typedef enum {
    SPI_MSB_LAST  = 0,                          /*!< LSB first */
    SPI_MSB_FIRST = 1                           /*!< MSB first */
}adi_cms_spi_msb_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_spi_addr_inc_e` is an enumeration that defines the modes
 * for SPI address increment settings. It provides two options:
 * `SPI_ADDR_DEC_AUTO` for automatic address decrementing and
 * `SPI_ADDR_INC_AUTO` for automatic address incrementing. This
 * enumeration is used to configure how the SPI address should behave
 * during data transfers, either incrementing or decrementing
 * automatically.
 *
 * @param SPI_ADDR_DEC_AUTO Represents an auto-decremented SPI address mode with
 * a value of 0.
 * @param SPI_ADDR_INC_AUTO Represents an auto-incremented SPI address mode with
 * a value of 1.
 ******************************************************************************/
typedef enum {
    SPI_ADDR_DEC_AUTO = 0,                      /*!< auto decremented */
    SPI_ADDR_INC_AUTO = 1                       /*!< auto incremented */
}adi_cms_spi_addr_inc_e;

/***************************************************************************//**
 * @brief The `adi_cms_driver_mode_config_e` is an enumeration that defines the
 * driver operation modes for a device, specifically distinguishing
 * between open drain and CMOS modes. This enumeration is used to
 * configure the electrical characteristics of the driver output,
 * allowing for flexibility in interfacing with different types of
 * circuits.
 *
 * @param OPEN_DRAIN_MODE Represents the open drain mode with a value of 0.
 * @param CMOS_MODE Represents the CMOS mode with a value of 1.
 ******************************************************************************/
typedef enum
{
    OPEN_DRAIN_MODE = 0,
    CMOS_MODE = 1
}adi_cms_driver_mode_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_impedance_type_e` is an enumeration that defines
 * different types of signal impedance configurations for a device. It
 * provides options to either disable the internal resistor or select
 * between a 100-ohm or 50-ohm internal resistor. Additionally, it
 * includes an option for an unknown impedance type, which can be used as
 * a default or error state.
 *
 * @param ADI_CMS_NO_INTERNAL_RESISTOR Represents the option to disable the
 * internal resistor.
 * @param ADI_CMS_INTERNAL_RESISTOR_100_OHM Represents the option to use an
 * internal 100-ohm resistor.
 * @param ADI_CMS_INTERNAL_RESISTOR_50_OHM Represents the option to use an
 * internal 50-ohm resistor.
 * @param ADI_CMS_INTERNAL_RESISTOR_UNKNOWN Represents an unknown impedance
 * type.
 ******************************************************************************/
typedef enum {
    ADI_CMS_NO_INTERNAL_RESISTOR = 0,           /*!< disable internal resistor */
    ADI_CMS_INTERNAL_RESISTOR_100_OHM = 1,      /*!< internal 100ohm resistor */
    ADI_CMS_INTERNAL_RESISTOR_50_OHM = 2,       /*!< internal  50ohm resistor */
    ADI_CMS_INTERNAL_RESISTOR_UNKNOWN =3        /*!< unknown type */
}adi_cms_signal_impedance_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_type_e` is an enumeration that defines various
 * types of signal standards used in communication systems. It includes
 * CMOS, LVDS, CML, LVPECL, and an unknown signal type, providing a way
 * to categorize and handle different signal types within the software.
 *
 * @param SIGNAL_CMOS CMOS signal.
 * @param SIGNAL_LVDS LVDS signal.
 * @param SIGNAL_CML CML signal.
 * @param SIGNAL_LVPECL LVPECL signal.
 * @param SIGNAL_UNKNOWN Unknown signal type.
 ******************************************************************************/
typedef enum {
    SIGNAL_CMOS = 0,                            /*!< CMOS signal */
    SIGNAL_LVDS = 1,                            /*!< LVDS signal */
    SIGNAL_CML = 2,                             /*!< CML  signal */
    SIGNAL_LVPECL = 3,                          /*!< LVPECL signal */
    SIGNAL_UNKNOWN = 4                          /*!< UNKNOW signal */
}adi_cms_signal_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_coupling_e` is an enumeration that defines
 * different types of signal coupling modes. It includes three possible
 * values: `COUPLING_AC` for AC coupled signals, `COUPLING_DC` for DC
 * signals, and `COUPLING_UNKNOWN` for cases where the coupling type is
 * not known. This enumeration is used to specify the coupling mode of a
 * signal in applications involving signal processing or communication
 * systems.
 *
 * @param COUPLING_AC Represents an AC coupled signal with a value of 0.
 * @param COUPLING_DC Represents a DC signal with a value of 1.
 * @param COUPLING_UNKNOWN Represents an unknown coupling type with a value of
 * 2.
 ******************************************************************************/
typedef enum {
    COUPLING_AC = 0,                            /*!< AC coupled signal */
    COUPLING_DC = 1,                            /*!< DC signal */
    COUPLING_UNKNOWN = 2                        /*!< UNKNOWN coupling */
}adi_cms_signal_coupling_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_link_e` is an enumeration that defines the possible
 * states of JESD links in a system. It provides options to specify no
 * link, a specific link (either link 0 or link 1), or all links. This
 * enumeration is useful for configuring and managing JESD link settings
 * in applications that require precise control over JESD interfaces.
 *
 * @param JESD_LINK_NONE Represents no JESD link.
 * @param JESD_LINK_0 Represents JESD link 0.
 * @param JESD_LINK_1 Represents JESD link 1.
 * @param JESD_LINK_ALL Represents all JESD links.
 ******************************************************************************/
typedef enum {
    JESD_LINK_NONE = 0,                         /*!< JESD link none  */
    JESD_LINK_0 = 1,                            /*!< JESD link 0 */
    JESD_LINK_1 = 2,                            /*!< JESD link 1 */
    JESD_LINK_ALL = 3                           /*!< All JESD links  */
}adi_cms_jesd_link_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_syncoutb_e` is an enumeration that defines constants
 * for different SYNCOUTB output signals used in JESD (Joint Electron
 * Device Engineering Council) interfaces. It provides symbolic names for
 * specific SYNCOUTB signals, allowing for easier reference and
 * manipulation of these signals in the code. The enumeration includes
 * individual signals as well as a collective representation of all
 * signals.
 *
 * @param SYNCOUTB_0 Represents the SYNCOUTB 0 signal with a value of 0x0.
 * @param SYNCOUTB_1 Represents the SYNCOUTB 1 signal with a value of 0x1.
 * @param SYNCOUTB_ALL Represents all SYNCOUTB signals with a value of 0xFF.
 ******************************************************************************/
typedef enum {
    SYNCOUTB_0 = 0x0,                           /*!< SYNCOUTB 0 */
    SYNCOUTB_1 = 0x1,                           /*!< SYNCOUTB 1 */
    SYNCOUTB_ALL = 0xFF                         /*!< ALL SYNCOUTB SIGNALS */
}adi_cms_jesd_syncoutb_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_sysref_mode_e` is an enumeration that defines the
 * different modes of SYSREF synchronization for JESD (Joint Electron
 * Device Engineering Council) interfaces. It includes modes for no
 * SYSREF support, one-shot SYSREF, continuous synchronization, and
 * monitoring, as well as an invalid mode to handle erroneous states.
 *
 * @param SYSREF_NONE No SYSREF Support.
 * @param SYSREF_ONESHOT ONE-SHOT SYSREF.
 * @param SYSREF_CONT Continuous SysRef sync.
 * @param SYSREF_MON SYSREF monitor mode.
 * @param SYSREF_MODE_INVALID Invalid SYSREF mode.
 ******************************************************************************/
typedef enum {
    SYSREF_NONE = 0,                            /*!< No SYSREF Support */
    SYSREF_ONESHOT = 1,                         /*!< ONE-SHOT SYSREF */
    SYSREF_CONT = 2,                            /*!< Continuous SysRef sync. */
    SYSREF_MON = 3,                             /*!< SYSREF monitor mode */
    SYSREF_MODE_INVALID = 4
}adi_cms_jesd_sysref_mode_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_prbs_pattern_e` is an enumeration that defines
 * various Pseudo-Random Binary Sequence (PRBS) patterns used in JESD
 * (Joint Electron Device Engineering Council) interfaces. These patterns
 * are used for testing and validation of data transmission integrity
 * over JESD links. The enumeration includes several standard PRBS
 * patterns such as PRBS7, PRBS9, PRBS15, PRBS23, and PRBS31, as well as
 * a state to indicate that PRBS is not in use (PRBS_NONE). The PRBS_MAX
 * value is used to represent the total number of PRBS pattern types
 * defined in this enumeration.
 *
 * @param PRBS_NONE Represents the state where PRBS is turned off.
 * @param PRBS7 Represents the PRBS7 pattern.
 * @param PRBS9 Represents the PRBS9 pattern.
 * @param PRBS15 Represents the PRBS15 pattern.
 * @param PRBS23 Represents the PRBS23 pattern.
 * @param PRBS31 Represents the PRBS31 pattern.
 * @param PRBS_MAX Indicates the number of PRBS pattern types available.
 ******************************************************************************/
typedef enum {
    PRBS_NONE = 0,                              /*!< PRBS   off */
    PRBS7  = 1,                                 /*!< PRBS7  pattern */
    PRBS9  = 2,                                 /*!< PRBS9  pattern */
    PRBS15 = 3,                                 /*!< PRBS15 pattern */
    PRBS23 = 4,                                 /*!< PRBS23 pattern */
    PRBS31 = 5,                                 /*!< PRBS31 pattern */
    PRBS_MAX = 6                                /*!< Number of member */
}adi_cms_jesd_prbs_pattern_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_subclass_e` is an enumeration that defines the
 * available JESD subclass modes, which are used to specify the
 * operational subclass of a JESD interface. It includes two valid
 * subclasses, 0 and 1, and an invalid subclass option for error handling
 * or default states.
 *
 * @param JESD_SUBCLASS_0 Represents JESD Subclass 0.
 * @param JESD_SUBCLASS_1 Represents JESD Subclass 1.
 * @param JESD_SUBCLASS_INVALID Represents an invalid JESD Subclass.
 ******************************************************************************/
typedef enum {
    JESD_SUBCLASS_0 = 0,                        /*!< JESD SUBCLASS 0 */
    JESD_SUBCLASS_1 = 1,                        /*!< JESD SUBCLASS 1 */
    JESD_SUBCLASS_INVALID = 2
}adi_cms_jesd_subclass_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_param_t` structure defines the parameters for
 * configuring a JESD (Joint Electron Device Engineering Council)
 * interface, which is used for high-speed serial data communication. It
 * includes fields for specifying the number of lanes, octets per frame,
 * converters, and samples, as well as various configuration parameters
 * such as converter resolution, scrambling, and mode settings. This
 * structure is essential for setting up and managing the JESD interface
 * in devices that require precise control over data transmission
 * characteristics.
 *
 * @param jesd_l Number of lanes.
 * @param jesd_f Number of octets in a frame.
 * @param jesd_m Number of converters.
 * @param jesd_s Number of samples.
 * @param jesd_hd High Density flag.
 * @param jesd_k Number of frames for a multi-frame.
 * @param jesd_n Converter resolution.
 * @param jesd_np Bit packing sample.
 * @param jesd_cf Parameter CF.
 * @param jesd_cs Parameter CS.
 * @param jesd_did Device ID DID.
 * @param jesd_bid Bank ID BID.
 * @param jesd_lid0 Lane ID for lane 0.
 * @param jesd_subclass Subclass.
 * @param jesd_scr Scramble enable flag.
 * @param jesd_duallink Link mode (single/dual).
 * @param jesd_jesdv Version (0:204A, 1:204B, 2:204C).
 * @param jesd_mode_id JESD mode ID.
 * @param jesd_mode_c2r_en JESD mode C2R enable flag.
 * @param jesd_mode_s_sel JESD mode S value.
 ******************************************************************************/
typedef struct {
    uint8_t jesd_l;                             /*!< No of lanes */
    uint8_t jesd_f;                             /*!< No of octets in a frame */
    uint8_t jesd_m;                             /*!< No of converters */
    uint8_t jesd_s;                             /*!< No of samples */
    uint8_t jesd_hd;                            /*!< High Density */
    uint16_t jesd_k;                            /*!< No of frames for a multi-frame */
    uint8_t jesd_n;                             /*!< Converter resolution */
    uint8_t jesd_np;                            /*!< Bit packing sample */
    uint8_t jesd_cf;                            /*!< Parameter CF */
    uint8_t jesd_cs;                            /*!< Parameter CS */
    uint8_t jesd_did;                           /*!< Device ID DID */
    uint8_t jesd_bid;                           /*!< Bank ID.  BID */
    uint8_t jesd_lid0;                          /*!< Lane ID for lane0 */
    uint8_t jesd_subclass;                      /*!< Subclass */
    uint8_t jesd_scr;                           /*!< Scramble enable */
    uint8_t jesd_duallink;                      /*!< Link mode (single/dual) */
    uint8_t jesd_jesdv;                         /*!< Version (0:204A, 1:204B, 2:204C) */
    uint8_t jesd_mode_id;                       /*!< JESD mode ID */
    uint8_t jesd_mode_c2r_en;                   /*!< JESD mode C2R enable */
    uint8_t jesd_mode_s_sel;                    /*!< JESD mode S value */
}adi_cms_jesd_param_t;

/**
 * @brief  Platform dependent SPI access functions.
 *         
 * @param  in_data     Pointer to array with the data to be sent on the SPI
 * @param  out_data    Pointer to array where the data to which the SPI will be written
 * @param  size_bytes  The size in bytes allocated for each of the in_data and out_data arrays.
 *                     bit[31:28]: 0000-8bit reg data, 0001-16bit reg data, 0010-32bit reg data
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 *
 * @note   in_data and out_data arrays are of same size.
 */
typedef int32_t(*adi_spi_xfer_t)(void *user_data, uint8_t *in_data, uint8_t *out_data, uint32_t size_bytes);

/**
 * @brief  Delay for specified number of microseconds. Platform Dependant.
 *         Performs a blocking or sleep delay for the specified time in microseconds
 *         The implementation of this function is platform dependent and
 *         is required for correct operation of the API.
 *
 * @param  us time to delay/sleep in microseconds.
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_delay_us_t)(void *user_data, uint32_t us);

/**
 * @brief  Write log message. Platform Dependant.
 *
 * @param  user_data  A void pointer to a client defined structure containing any
 *                    parameters/settings that may be required by the function
 *                    to write log messages for the ADI Device.
 * @param  log_type   @see adi_cms_log_type_e
 * @param  message    Format string
 * @param  argp       Variable message
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_log_write_t)(void *user_data, int32_t log_type, const char *message, va_list argp);

/**
 * @brief  Write log message. Platform Dependant.
 *
 * @param  user_data  A void pointer to a client defined structure containing any
 *                    parameters/settings that may be required by the function
 *                    to write log messages for the ADI Device.
 * @param  log_type   @see adi_cms_log_type_e
 * @param  message    Message string
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_log_write_s_t)(void *user_data, int32_t log_type, char *message);

/**
 * @brief  Platform hardware initialization for the ADL5580 Device
 *         This function shall initialize all external hardware resources required by
 *         the ADI Device and API for correct functionality as per the 
 *         target platform.
 *         For example initialization of SPI, GPIO resources, clocks etc.
 *         
 * @param  user_data  A void pointer to a client defined structure containing any
 *                    parameters/settings that may be required by the function
 *                    to initialize the hardware for the ADI Device.
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_hw_open_t)(void *user_data);

/**
 * @brief  Closes any platform hardware resources for device.
 *         This function shall close or shutdown all external hardware resources 
 *         required by the ADL5580 Device and API for correct functionality 
 *         as per the target platform.
 *         For example initialization of SPI, GPIO resources, clocks etc.
 *         It should close and free any resources assigned in the hw_open_t function.
 *
 * @param  user_data  A void pointer to a client defined structure containing any
 *                    parameters/settings that may be required by the function
 *                    to close/shutdown the hardware for the ADI Device.
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_hw_close_t)(void *user_data);

/**
 * @brief  Client Event Handler
 *
 * @param  event A uint16_t value representing the event that occurred.
 * @param  ref   A uint8_t value indicating the reference for that event if any.
 *               For example 0 if even occurred on lane 0.
 * @param  data  A void pointer to any user data that may pertain to that event.
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_event_handler_t)(uint16_t event, uint8_t ref, void* data);

/**
 * @brief  tx_enable pin control function
 *
 * @param  user_data A void pointer to a client defined structure containing
 *                   any parameters/settings that may be required by the function
 *                   to control the hardware for the ADI Device TX_ENABLE PIN.
 * @param  enable    A uint8_t value indicating the desired enable/disable
 *                   setting for the tx_enable pin.
 *                   A value of 1 indicates TX_ENABLE pin is set HIGH
 *                   A value of 0 indicates TX_ENABLE pin is set LOW
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_tx_en_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * @brief  reset pin control function
 *
 * @param  user_data  A void pointer to a client defined structure containing
 *                    any parameters/settings that may be required by the function
 *                    to control the hardware for the ADI Device RESETB PIN.
 * @param  enable     A uint8_t value indicating the desired enable/disable
 *                    reset via the ADI device RESETB pin.
 *                    A value of 1 indicates RESETB pin is set LOW
 *                    A value of 0 indicates RESETB pin is set HIGH
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_reset_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * @brief   Control function for GPIO write.
 *
 * @param   user_data   A void pointer to a client defined structure containing
 *                      any parameters/settings that may be required by the function
 *                      to control the hardware for the ADI Device RESETB PIN.
 * @param   gpio        A uint32_t GPIO index used for identification. See enum "adi_adl5580_gpio_e".
 * @param   value       A uint32_t value indicating the desired high/low state for GPIO.
 *                      A value of 1 indicates GPIO pin is set HIGH
 *                      A value of 0 indicates GPIO pin is set LOW
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_gpio_write_t) (void *user_data, uint32_t gpio, uint32_t value);

/**
 * @brief   Control function for GPIO write.
 *
 * @param   user_data   A void pointer to a client defined structure containing
 *                      any parameters/settings that may be required by the function
 *                      to control the hardware for the ADI Device RESETB PIN.
 * @param   gpio        A uint32_t GPIO index used for identification. See enum "adi_adl5580_gpio_e".
 * @param   value       A uint32_t integer pointer indicating the readback state of GPIO.
 *                      A value of 1 indicates GPIO pin is set HIGH
 *                      A value of 0 indicates GPIO pin is set LOW
 * @Note    Depending on the platform, reading a GPIO which is set as an OUTPUT may result in changing the GPIO state.
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t(*adi_gpio_read_t)(void *user_data, uint32_t gpio, uint32_t *value);


#endif /* __ADI_API_COMMON_H__ */
/*! @} */
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
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/delay.h>
#include <linux/gcd.h>

static inline int32_t adi_api_utils_is_power_of_two(uint64_t x)
{
	return (u32) is_power_of_2(x);
}

static inline int32_t adi_api_utils_gcd(int32_t u, int32_t v)
{
	return (u32) gcd(u, v);
}

static inline uint32_t adi_api_utils_log2(uint32_t a)
{
	uint8_t b = 0;

	while (a >>= 1)
		b++;

	return b; /* log2(a) , only for power of 2 numbers */
}

#else
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

/***************************************************************************//**
 * @brief The function `adi_api_utils_is_power_of_two` checks if a given 64-bit
 * unsigned integer is a power of two.
 *
 * @param x A 64-bit unsigned integer to be checked if it is a power of two.
 * @return Returns 1 if the input number is a power of two, otherwise returns 0.
 ******************************************************************************/
static inline int32_t adi_api_utils_is_power_of_two(uint64_t x)
{
	if (x == 0)
		return 0;

	while (x != 1) {
		if (x % 2 != 0)
			return 0;
		x = x / 2;
	}

	return 1;
}

/***************************************************************************//**
 * @brief The `adi_api_utils_gcd` function calculates the greatest common
 * divisor (GCD) of two integers using a simple iterative method.
 *
 * @param u An integer representing the first number for which the GCD is to be
 * calculated.
 * @param v An integer representing the second number for which the GCD is to be
 * calculated.
 * @return The function returns an integer representing the greatest common
 * divisor of the two input integers.
 ******************************************************************************/
static inline int32_t adi_api_utils_gcd(int32_t u, int32_t v)
{
	uint32_t div;
	uint32_t common_div = 1;

	if ((u == 0) || (v == 0))
		return (((u) > (v)) ? (u) : (v));

	for (div = 1; (div <= u) && (div <= v); div++)
		if (!(u % div) && !(v % div))
			common_div = div;

	return common_div;
}

/***************************************************************************//**
 * @brief The `adi_api_utils_log2` function calculates the base-2 logarithm of a
 * given power of two number.
 *
 * @param a A 32-bit unsigned integer representing a power of two number for
 * which the base-2 logarithm is to be calculated.
 * @return The function returns an 8-bit unsigned integer representing the
 * base-2 logarithm of the input number `a`, assuming `a` is a power of
 * two.
 ******************************************************************************/
static inline uint32_t adi_api_utils_log2(uint32_t a)
{
	uint8_t b = 0;

	while (a >>= 1)
		b++;

	return b; /* log2(a) , only for power of 2 numbers */
}

#endif
#include "adi_cms_api_config.h"

/*============= D E F I N E S ==============*/
/***************************************************************************//**
 * @brief The `adi_cms_error_e` is an enumeration that defines a comprehensive
 * set of error codes used in the Analog Devices API to indicate various
 * error conditions. Each enumerator represents a specific error
 * scenario, ranging from general errors, invalid parameters, and pointer
 * issues, to more specific hardware-related errors such as SPI transfer
 * errors, PLL lock failures, and function control errors. This
 * enumeration is crucial for error handling and debugging within the
 * API, providing clear and specific error codes that can be used to
 * diagnose and resolve issues effectively.
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
 * @param API_CMS_ERROR_JESD_PLL_NOT_LOCKED PD STBY function error.
 * @param API_CMS_ERROR_JESD_SYNC_NOT_DONE JESD_SYNC_NOT_DONE.
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
 * @param API_CMS_ERROR_PD_STBY_PIN_CTRL PD STBY function error.
 * @param API_CMS_ERROR_SYSREF_CTRL SYSREF enable function error.
 ******************************************************************************/
typedef enum {
	API_CMS_ERROR_OK = 0, /*!< No Error */
	API_CMS_ERROR_ERROR = -1, /*!< General Error  */
	API_CMS_ERROR_NULL_PARAM = -2, /*!< Null parameter */
	API_CMS_ERROR_SPI_SDO =
		-10, /*!< Wrong value assigned to the SDO in device structure */
	API_CMS_ERROR_INVALID_HANDLE_PTR =
		-11, /*!< Device handler pointer is invalid */
	API_CMS_ERROR_INVALID_XFER_PTR =
		-12, /*!< Invalid pointer to the SPI xfer function assigned */
	API_CMS_ERROR_INVALID_DELAYUS_PTR =
		-13, /*!< Invalid pointer to the delay_us function assigned */
	API_CMS_ERROR_INVALID_PARAM = -14, /*!< Invalid parameter passed */
	API_CMS_ERROR_INVALID_RESET_CTRL_PTR =
		-15, /*!< Invalid pointer to the reset control function assigned */
	API_CMS_ERROR_NOT_SUPPORTED = -16, /*!< Not supported */
	API_CMS_ERROR_VCO_OUT_OF_RANGE = -20, /*!< The VCO is out of range */
	API_CMS_ERROR_PLL_NOT_LOCKED = -21, /*!< PLL is not locked */
	API_CMS_ERROR_DLL_NOT_LOCKED = -22, /*!< DLL is not locked */
	API_CMS_ERROR_MODE_NOT_IN_TABLE = -23, /*!< JESD Mode not in table */
	API_CMS_ERROR_JESD_PLL_NOT_LOCKED = -24, /*!< PD STBY function error */
	API_CMS_ERROR_JESD_SYNC_NOT_DONE = -25, /*!< JESD_SYNC_NOT_DONE */
	API_CMS_ERROR_FTW_LOAD_ACK = -30, /*!< FTW acknowledge not received */
	API_CMS_ERROR_NCO_NOT_ENABLED = -31, /*!< The NCO is not enabled */
	API_CMS_ERROR_INIT_SEQ_FAIL =
		-40, /*!< Initialization sequence failed */
	API_CMS_ERROR_TEST_FAILED = -50, /*!< Test failed */
	API_CMS_ERROR_SPI_XFER = -60, /*!< SPI transfer error */
	API_CMS_ERROR_TX_EN_PIN_CTRL = -62, /*!< TX enable function error */
	API_CMS_ERROR_RESET_PIN_CTRL = -63, /*!< HW reset function error */
	API_CMS_ERROR_EVENT_HNDL = -64, /*!< Event handling error */
	API_CMS_ERROR_HW_OPEN = -65, /*!< HW open function error */
	API_CMS_ERROR_HW_CLOSE = -66, /*!< HW close function error */
	API_CMS_ERROR_LOG_OPEN = -67, /*!< Log open error */
	API_CMS_ERROR_LOG_WRITE = -68, /*!< Log write error */
	API_CMS_ERROR_LOG_CLOSE = -69, /*!< Log close error */
	API_CMS_ERROR_DELAY_US = -70, /*!< Delay error */
	API_CMS_ERROR_PD_STBY_PIN_CTRL = -71, /*!< PD STBY function error */
	API_CMS_ERROR_SYSREF_CTRL = -72 /*!< SYSREF enable function error */

} adi_cms_error_e;

/***************************************************************************//**
 * @brief The `adi_cms_log_type_e` is an enumeration that defines various log
 * message types for a system, allowing for categorization of log
 * messages into none, error, warning, informational, SPI-related, API-
 * related, or all-inclusive categories. Each log type is associated with
 * a specific hexadecimal value, enabling bitwise operations for log
 * filtering and selection.
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
	ADI_CMS_LOG_NONE = 0x0000, /*!< all not selected */
	ADI_CMS_LOG_ERR = 0x0001, /*!< error message */
	ADI_CMS_LOG_WARN = 0x0002, /*!< warning message */
	ADI_CMS_LOG_MSG = 0x0004, /*!< tips info message */
	ADI_CMS_LOG_SPI = 0x0010, /*!< spi r/w info message */
	ADI_CMS_LOG_API = 0x0020, /*!< api info message */
	ADI_CMS_LOG_ALL = 0xFFFF /*!< all selected */
} adi_cms_log_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_chip_id_t` structure is used to store identification data
 * for an ADI device. It includes fields for the chip type, product ID,
 * product grade, and device revision, which collectively provide a
 * unique identifier for the specific device model and version. This
 * structure is essential for distinguishing between different devices
 * and their respective versions in a system that utilizes multiple ADI
 * components.
 *
 * @param chip_type Chip Type Code.
 * @param prod_id Product ID Code.
 * @param prod_grade Product Grade Code.
 * @param dev_revision Device Revision.
 ******************************************************************************/
typedef struct {
	uint8_t chip_type; /*!< Chip Type Code */
	uint16_t prod_id; /*!< Product ID Code */
	uint8_t prod_grade; /*!< Product Grade Code */
	uint8_t dev_revision; /*!< Device Revision */
} adi_cms_chip_id_t;

/***************************************************************************//**
 * @brief The `adi_cms_reg_data_t` structure is used to represent a register and
 * its corresponding value in a device. It contains two members: `reg`,
 * which holds the address of the register, and `val`, which holds the
 * value to be written to or read from that register. This structure is
 * typically used in contexts where direct manipulation of device
 * registers is required, such as in low-level hardware interfacing or
 * driver development.
 *
 * @param reg Register address.
 * @param val Register value.
 ******************************************************************************/
typedef struct {
	uint16_t reg; /*!< Register address */
	uint8_t val; /*!< Register value */
} adi_cms_reg_data_t;

/***************************************************************************//**
 * @brief The `adi_cms_spi_sdo_config_e` is an enumeration that defines
 * different SPI (Serial Peripheral Interface) configurations for a
 * device. It includes options for no configuration (`SPI_NONE`), a
 * 4-wire configuration with SDO active (`SPI_SDO`), and a 3-wire
 * configuration with SDIO active (`SPI_SDIO`). The `SPI_CONFIG_MAX` is
 * used as a boundary marker to indicate the maximum configuration value.
 * This enumeration is useful for setting and managing the SPI
 * communication mode in devices that support multiple SPI
 * configurations.
 *
 * @param SPI_NONE Represents no SPI configuration, used for testing.
 * @param SPI_SDO Indicates SDO is active, applicable for 4-wire SPI
 * configuration.
 * @param SPI_SDIO Indicates SDIO is active, applicable for 3-wire SPI
 * configuration.
 * @param SPI_CONFIG_MAX Represents the maximum value for SPI configuration,
 * used as a boundary marker.
 ******************************************************************************/
typedef enum {
	SPI_NONE = 0, /*!< Keep this for test */
	SPI_SDO = 1, /*!< SDO  active, 4-wire only */
	SPI_SDIO = 2, /*!< SDIO active, 3-wire only */
	SPI_CONFIG_MAX = 3 /*!< Keep it last */
} adi_cms_spi_sdo_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_spi_msb_config_e` is an enumeration that defines the bit
 * order configuration for SPI communication. It allows the user to
 * specify whether the most significant bit (MSB) or the least
 * significant bit (LSB) is transmitted first during SPI data transfers.
 * This configuration is crucial for ensuring compatibility with
 * different SPI devices that may require different bit orderings.
 *
 * @param SPI_MSB_LAST Represents the configuration where the least significant
 * bit (LSB) is transmitted first.
 * @param SPI_MSB_FIRST Represents the configuration where the most significant
 * bit (MSB) is transmitted first.
 ******************************************************************************/
typedef enum {
	SPI_MSB_LAST = 0, /*!< LSB first */
	SPI_MSB_FIRST = 1 /*!< MSB first */
} adi_cms_spi_msb_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_spi_addr_inc_e` is an enumeration that defines the modes
 * for SPI address increment settings. It provides two options:
 * `SPI_ADDR_DEC_AUTO` for automatic address decrementing and
 * `SPI_ADDR_INC_AUTO` for automatic address incrementing. This
 * enumeration is used to configure how the SPI address should be
 * adjusted automatically during operations.
 *
 * @param SPI_ADDR_DEC_AUTO Represents an auto-decremented SPI address mode with
 * a value of 0.
 * @param SPI_ADDR_INC_AUTO Represents an auto-incremented SPI address mode with
 * a value of 1.
 ******************************************************************************/
typedef enum {
	SPI_ADDR_DEC_AUTO = 0, /*!< auto decremented */
	SPI_ADDR_INC_AUTO = 1 /*!< auto incremented */
} adi_cms_spi_addr_inc_e;

/***************************************************************************//**
 * @brief The `adi_cms_driver_mode_config_e` is an enumeration that defines the
 * driver operation modes for a device, specifically indicating whether
 * the device operates in open-drain mode or CMOS mode. This enumeration
 * is used to configure the electrical characteristics of the device's
 * output drivers, allowing for flexibility in interfacing with different
 * types of circuits.
 *
 * @param OPEN_DRAIN_MODE Represents the open-drain mode configuration with a
 * value of 0.
 * @param CMOS_MODE Represents the CMOS mode configuration with a value of 1.
 ******************************************************************************/
typedef enum {
	OPEN_DRAIN_MODE = 0,
	CMOS_MODE = 1
} adi_cms_driver_mode_config_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_impedance_type_e` is an enumeration that defines
 * different types of signal impedance configurations for a device. It
 * includes options for no internal resistor, a 100-ohm internal
 * resistor, a 50-ohm internal resistor, and an unknown resistor type.
 * This enumeration is used to specify the impedance characteristics of a
 * signal path within the device, which can be critical for ensuring
 * proper signal integrity and matching in various applications.
 *
 * @param ADI_CMS_NO_INTERNAL_RESISTOR Represents the absence of an internal
 * resistor.
 * @param ADI_CMS_INTERNAL_RESISTOR_100_OHM Represents an internal resistor with
 * a value of 100 ohms.
 * @param ADI_CMS_INTERNAL_RESISTOR_50_OHM Represents an internal resistor with
 * a value of 50 ohms.
 * @param ADI_CMS_INTERNAL_RESISTOR_UNKNOWN Represents an unknown type of
 * internal resistor.
 ******************************************************************************/
typedef enum {
	ADI_CMS_NO_INTERNAL_RESISTOR = 0, /*!< disable internal resistor */
	ADI_CMS_INTERNAL_RESISTOR_100_OHM = 1, /*!< internal 100ohm resistor */
	ADI_CMS_INTERNAL_RESISTOR_50_OHM = 2, /*!< internal  50ohm resistor */
	ADI_CMS_INTERNAL_RESISTOR_UNKNOWN = 3 /*!< unknown type */
} adi_cms_signal_impedance_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_type_e` is an enumeration that defines various
 * types of signal standards used in communication systems. Each
 * enumerator represents a specific signal type, such as CMOS, LVDS, CML,
 * LVPECL, and an unknown type, with corresponding integer values. This
 * enumeration is useful for categorizing and handling different signal
 * types within the software, allowing for clear and consistent signal
 * type management.
 *
 * @param SIGNAL_CMOS CMOS signal type represented by the value 0.
 * @param SIGNAL_LVDS LVDS signal type represented by the value 1.
 * @param SIGNAL_CML CML signal type represented by the value 2.
 * @param SIGNAL_LVPECL LVPECL signal type represented by the value 3.
 * @param SIGNAL_UNKNOWN Unknown signal type represented by the value 4.
 ******************************************************************************/
typedef enum {
	SIGNAL_CMOS = 0, /*!< CMOS signal */
	SIGNAL_LVDS = 1, /*!< LVDS signal */
	SIGNAL_CML = 2, /*!< CML  signal */
	SIGNAL_LVPECL = 3, /*!< LVPECL signal */
	SIGNAL_UNKNOWN = 4 /*!< UNKNOW signal */
} adi_cms_signal_type_e;

/***************************************************************************//**
 * @brief The `adi_cms_signal_coupling_e` is an enumeration that defines the
 * possible coupling modes for a signal. It includes three members:
 * `COUPLING_AC` for AC coupled signals, `COUPLING_DC` for DC signals,
 * and `COUPLING_UNKNOWN` for cases where the coupling type is not known.
 * This enumeration is used to specify the type of signal coupling in
 * various applications, providing a clear and standardized way to handle
 * different signal coupling scenarios.
 *
 * @param COUPLING_AC Represents an AC coupled signal with a value of 0.
 * @param COUPLING_DC Represents a DC signal with a value of 1.
 * @param COUPLING_UNKNOWN Represents an unknown coupling type with a value of
 * 2.
 ******************************************************************************/
typedef enum {
	COUPLING_AC = 0, /*!< AC coupled signal */
	COUPLING_DC = 1, /*!< DC signal */
	COUPLING_UNKNOWN = 2 /*!< UNKNOWN coupling */
} adi_cms_signal_coupling_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_link_e` is an enumeration that defines the possible
 * states of JESD links in a system. It provides options to specify no
 * link, individual links (0 or 1), or all links, which can be used to
 * configure or query the status of JESD links in a device.
 *
 * @param JESD_LINK_NONE Represents no JESD link.
 * @param JESD_LINK_0 Represents JESD link 0.
 * @param JESD_LINK_1 Represents JESD link 1.
 * @param JESD_LINK_ALL Represents all JESD links.
 ******************************************************************************/
typedef enum {
	JESD_LINK_NONE = 0, /*!< JESD link none  */
	JESD_LINK_0 = 1, /*!< JESD link 0 */
	JESD_LINK_1 = 2, /*!< JESD link 1 */
	JESD_LINK_ALL = 3 /*!< All JESD links  */
} adi_cms_jesd_link_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_syncoutb_e` is an enumeration that defines the
 * possible SYNCOUTB output signals for a JESD interface. It includes
 * options for individual SYNCOUTB signals (0 and 1) as well as a
 * collective option for all SYNCOUTB signals. This enumeration is used
 * to specify which SYNCOUTB signal(s) are active or being referenced in
 * a given context, providing a clear and concise way to manage SYNCOUTB
 * signal configurations in the system.
 *
 * @param SYNCOUTB_0 Represents the SYNCOUTB 0 signal.
 * @param SYNCOUTB_1 Represents the SYNCOUTB 1 signal.
 * @param SYNCOUTB_ALL Represents all SYNCOUTB signals.
 ******************************************************************************/
typedef enum {
	SYNCOUTB_0 = 0x0, /*!< SYNCOUTB 0 */
	SYNCOUTB_1 = 0x1, /*!< SYNCOUTB 1 */
	SYNCOUTB_ALL = 0xFF /*!< ALL SYNCOUTB SIGNALS */
} adi_cms_jesd_syncoutb_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_sysref_mode_e` is an enumeration that defines
 * various modes for SYSREF synchronization in a JESD (Joint Electron
 * Device Engineering Council) interface. It includes modes for no SYSREF
 * support, one-shot SYSREF, continuous synchronization, monitoring mode,
 * and an invalid mode for error handling or uninitialized states. This
 * enumeration is used to configure and manage the SYSREF signal behavior
 * in JESD systems, which is crucial for synchronizing data converters in
 * high-speed data transmission applications.
 *
 * @param SYSREF_NONE No SYSREF Support.
 * @param SYSREF_ONESHOT ONE-SHOT SYSREF.
 * @param SYSREF_CONT Continuous SysRef sync.
 * @param SYSREF_MON SYSREF monitor mode.
 * @param SYSREF_MODE_INVALID Invalid SYSREF mode.
 ******************************************************************************/
typedef enum {
	SYSREF_NONE = 0, /*!< No SYSREF Support */
	SYSREF_ONESHOT = 1, /*!< ONE-SHOT SYSREF */
	SYSREF_CONT = 2, /*!< Continuous SysRef sync. */
	SYSREF_MON = 3, /*!< SYSREF monitor mode */
	SYSREF_MODE_INVALID = 4
} adi_cms_jesd_sysref_mode_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_prbs_pattern_e` is an enumeration that defines
 * various Pseudo-Random Binary Sequence (PRBS) patterns used in JESD
 * (Joint Electron Device Engineering Council) interfaces. These patterns
 * are used for testing and validation of data transmission systems,
 * ensuring that the communication link is functioning correctly by
 * generating a known sequence of bits. The enumeration includes several
 * standard PRBS patterns, such as PRBS7, PRBS9, PRBS15, PRBS23, and
 * PRBS31, each differing in the length of the sequence and the
 * complexity of the pattern. The `PRBS_NONE` member indicates that no
 * PRBS pattern is applied, while `PRBS_MAX` serves as a boundary marker
 * for the number of patterns defined.
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
	PRBS_NONE = 0, /*!< PRBS   off */
	PRBS7 = 1, /*!< PRBS7  pattern */
	PRBS9 = 2, /*!< PRBS9  pattern */
	PRBS15 = 3, /*!< PRBS15 pattern */
	PRBS23 = 4, /*!< PRBS23 pattern */
	PRBS31 = 5, /*!< PRBS31 pattern */
	PRBS_MAX = 6 /*!< Number of member */
} adi_cms_jesd_prbs_pattern_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_subclass_e` is an enumeration that defines the
 * available JESD subclass modes, which are used to specify the
 * operational subclass of a JESD interface. It includes two valid
 * subclasses, 0 and 1, and an invalid subclass option for error handling
 * or undefined states.
 *
 * @param JESD_SUBCLASS_0 Represents JESD Subclass 0.
 * @param JESD_SUBCLASS_1 Represents JESD Subclass 1.
 * @param JESD_SUBCLASS_INVALID Represents an invalid JESD Subclass.
 ******************************************************************************/
typedef enum {
	JESD_SUBCLASS_0 = 0, /*!< JESD SUBCLASS 0 */
	JESD_SUBCLASS_1 = 1, /*!< JESD SUBCLASS 1 */
	JESD_SUBCLASS_INVALID = 2
} adi_cms_jesd_subclass_e;

/***************************************************************************//**
 * @brief The `adi_cms_jesd_param_t` structure defines various parameters for
 * configuring a JESD (Joint Electron Device Engineering Council)
 * interface, which is used in high-speed data converters. It includes
 * fields for specifying the number of lanes, octets in a frame,
 * converters, samples, and frames for a multi-frame, as well as settings
 * for converter resolution, bit packing, and various JESD-specific
 * parameters such as device ID, bank ID, lane ID, subclass, and
 * scrambling options. This structure is essential for setting up and
 * managing the JESD interface in devices that require precise data
 * transmission configurations.
 *
 * @param jesd_l No of lanes.
 * @param jesd_f No of octets in a frame.
 * @param jesd_m No of converters.
 * @param jesd_s No of samples.
 * @param jesd_hd High Density.
 * @param jesd_k No of frames for a multi-frame.
 * @param jesd_n Converter resolution.
 * @param jesd_np Bit packing sample.
 * @param jesd_cf Parameter CF.
 * @param jesd_cs Parameter CS.
 * @param jesd_did Device ID DID.
 * @param jesd_bid Bank ID BID.
 * @param jesd_lid0 Lane ID for lane0.
 * @param jesd_subclass Subclass.
 * @param jesd_scr Scramble enable.
 * @param jesd_duallink Link mode (single/dual).
 * @param jesd_jesdv Version (0:204A, 1:204B, 2:204C).
 * @param jesd_mode_id JESD mode ID.
 * @param jesd_mode_c2r_en JESD mode C2R enable.
 * @param jesd_mode_s_sel JESD mode S value.
 ******************************************************************************/
typedef struct {
	uint8_t jesd_l; /*!< No of lanes */
	uint8_t jesd_f; /*!< No of octets in a frame */
	uint8_t jesd_m; /*!< No of converters */
	uint8_t jesd_s; /*!< No of samples */
	uint8_t jesd_hd; /*!< High Density */
	uint16_t jesd_k; /*!< No of frames for a multi-frame */
	uint8_t jesd_n; /*!< Converter resolution */
	uint8_t jesd_np; /*!< Bit packing sample */
	uint8_t jesd_cf; /*!< Parameter CF */
	uint8_t jesd_cs; /*!< Parameter CS */
	uint8_t jesd_did; /*!< Device ID DID */
	uint8_t jesd_bid; /*!< Bank ID.  BID */
	uint8_t jesd_lid0; /*!< Lane ID for lane0 */
	uint8_t jesd_subclass; /*!< Subclass */
	uint8_t jesd_scr; /*!< Scramble enable */
	uint8_t jesd_duallink; /*!< Link mode (single/dual) */
	uint8_t jesd_jesdv; /*!< Version (0:204A, 1:204B, 2:204C) */
	uint8_t jesd_mode_id; /*!< JESD mode ID */
	uint8_t jesd_mode_c2r_en; /*!< JESD mode C2R enable */
	uint8_t jesd_mode_s_sel; /*!< JESD mode S value */
} adi_cms_jesd_param_t;

/***************************************************************************//**
 * @brief The `adi_cms_chip_op_mode_t` is an enumeration that defines the
 * operational modes of a chip, specifically indicating whether the chip
 * is using only the transmit (Tx) path, only the receive (Rx) path, or
 * both paths simultaneously. This enumeration is useful for configuring
 * the chip's operation mode in applications where different
 * communication paths are required.
 *
 * @param TX_ONLY Chip using Tx path only.
 * @param RX_ONLY Chip using Rx path only.
 * @param TX_RX_ONLY Chip using both Tx and Rx paths.
 ******************************************************************************/
typedef enum {
	TX_ONLY = 1, /*!< Chip using Tx path only */
	RX_ONLY = 2, /*!< Chip using Rx path only */
	TX_RX_ONLY = 3 /*!< Chip using Tx + Rx both paths */
} adi_cms_chip_op_mode_t;

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
typedef int32_t (*adi_spi_xfer_t)(void *user_data, uint8_t *in_data,
				  uint8_t *out_data, uint32_t size_bytes);

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
typedef int32_t (*adi_delay_us_t)(void *user_data, uint32_t us);

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
typedef int32_t (*adi_log_write_t)(void *user_data, int32_t log_type,
				   const char *message, va_list argp);

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
typedef int32_t (*adi_log_write_s_t)(void *user_data, int32_t log_type,
				     char *message);

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
typedef int32_t (*adi_hw_open_t)(void *user_data);

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
typedef int32_t (*adi_hw_close_t)(void *user_data);

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
typedef int32_t (*adi_event_handler_t)(uint16_t event, uint8_t ref, void *data);

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
typedef int32_t (*adi_tx_en_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * @brief  pd_stby pin control function
 *
 * @param  user_data A void pointer to a client defined structure containing
 *                   any parameters/settings that may be required by the function
 *                   to control the hardware for the ADI Device PD_STBY PIN.
 * @param  enable    A uint8_t value indicating the desired enable/disable
 *                   setting for the pd_stby pin.
 *                   A value of 1 indicates pd_stby pin is set HIGH
 *                   A value of 0 indicates pd_stby pin is set LOW
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t (*adi_pd_stby_pin_ctrl_t)(void *user_data, uint8_t enable);

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
typedef int32_t (*adi_reset_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * @brief sysref control function
 *
 * @param sysref_clk   A void pointer to a structure containing the clock source
 *                     required by the function to control the hardware sysref control.
 *
 * @return 0 for success
 * @return Any non-zero value indicates an error
 */
typedef int32_t (*adi_sysref_ctrl_t)(void *sysref_clk);

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
typedef int32_t (*adi_gpio_write_t)(void *user_data, uint32_t gpio,
				    uint32_t value);

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
typedef int32_t (*adi_gpio_read_t)(void *user_data, uint32_t gpio,
				   uint32_t *value);

#endif /* __ADI_API_COMMON_H__ */
/*! @} */

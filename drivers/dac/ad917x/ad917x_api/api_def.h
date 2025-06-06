/**
 * \file ad917x_api/api_def.h
 *
 * \brief API definitions header file.
 *
 * This file contains all common API definitions.
 *
 * Copyright(c) 2016 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
 */
#ifndef __API_DEF_H__
#define __API_DEF_H__
#include <stdint.h>
#define ADI_POW2_48 ((uint64_t)1u<<48)
#define ADI_MAXUINT48 (ADI_POW2_48 - 1)

#define ADI_POW2_32 ((uint64_t)1u<<32)
#define ADI_MAXUINT32 (ADI_POW2_32 - 1)
#define ADI_MAXUINT24 (0xFFFFFF)
#define ADI_GET_BYTE(w, p) (uint8_t)(((w) >> (p)) & 0xFF)
#define ALL -1

#define U64MSB 0x8000000000000000ull

/**
 * \brief Platform dependent SPI access functions.
 *
 *
 * \param indata  Pointer to array with the data to be sent on the SPI
 * \param outdata Pointer to array where the data to which the SPI will be written
 * \param size_bytes The size in bytes allocated for each of the indata and outdata arrays.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note indata and outdata arrays are of same size.
 */
typedef int32_t(*spi_xfer_t)(void *user_data, uint8_t *indata, uint8_t *outdata,
			     int32_t size_bytes);

/**
 * \brief Delay for specified number of microseconds. Platform Dependant.
 *
 * Performs a blocking or sleep delay for the specified time in microseconds
 * The implementation of this function is platform dependant and
 * is required for correct operation of the API.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \param us - time to delay/sleep in microseconds.
 */
typedef int32_t(*delay_us_t)(void *user_data, uint32_t us);

/**
 * \brief Platform hardware initialisation for the AD9164 Device
 *
 * This function shall initialize all external hardware resources required by
 * the ADI Device and API for correct functionatlity as per the
 * target platform.
 * For example initialisation of SPI, GPIO resources, clocks etc.
 *
 *
 * \param *user_data - A void pointer to a client defined structure containing any
 *             parameters/settings that may be required by the function
 *             to initialise the hardware for the ADI Device.
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 */
typedef int(*hw_open_t)(void *user_data);

/**
 * \brief Closes any platform hardware resources for the AD9164 Device
 *
 * This function shall close or shutdown all external hardware resources
 * required by the AD9164 Device and API for correct functionatlity
 * as per the target platform.
 * For example initialisation of SPI, GPIO resources, clocks etc.
 * It should close and free any resources assigned in the hw_open_t function.
 *
 * \param *user_data - A void pointer to a client defined structure containing any
 * 				parameters/settings that may be required by the function
 * 				to close/shutdown the hardware for the ADI Device.
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 */
typedef int(*hw_close_t)(void *user_data);
/**
 * \brief Client  Event Handler
 *
 *
 * \param event	A uint16_t value representing the event that occurred.
 * \param ref   A uint8_t value indicating the reference for that event if any.
 * 				For example 0 if even occured on lane 0.
 * \param data  A void pointer to any user data that may pertain to that event.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*event_handler_t)(uint16_t event, uint8_t ref, void *data);

/**
 * \brief TX_ENABLE PIN CONTROL FUNCTION
 *
 *
 * \param *user_data  A void pointer to a client defined structure containing
 * 					any parameters/settings that may be required by the function
 * 					to control the hardware for the ADI Device TX_ENABLE PIN.
 * \param enable   A uint8_t value indicating the desired enable/disable
 * 					setting for the tx_enable pin.
 * 					A value of 1 indicates TX_ENABLE pin is set HIGH
 * 					A value of 0 indicates TX_ENABLE pin is set LOW
 *
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*tx_en_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * \brief RESETB PIN CONTROL FUNCTION
 *
 *
 * \param *user_data  A void pointer to a client defined structure containing
 * 					any parameters/settings that may be required by the function
 * 					to control the hardware for the ADI Device RESETB PIN.
 * \param enable   A uint8_t value indicating the desired enable/disable
 * 					reset via the ADI device RESETB pin.
 * 					A value of 1 indicates RESETB pin is set LOW
 * 					A value of 0 indicates RESETB pin is set HIGH
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*reset_pin_ctrl_t)(void *user_data, uint8_t enable);

/***************************************************************************//**
 * @brief The `adi_reg_data` structure is used to represent a register in a
 * device, containing both the address of the register and the value to
 * be written to or read from it. This structure is typically used in the
 * context of device communication, where registers are accessed to
 * configure or retrieve information from hardware components.
 *
 * @param reg A 16-bit unsigned integer representing the register address.
 * @param val An 8-bit unsigned integer representing the register value.
 ******************************************************************************/
struct adi_reg_data {
	/** Register address */
	uint16_t reg;
	/** Register value */
	uint8_t  val;
};

/***************************************************************************//**
 * @brief The `adi_chip_id_t` structure is used to encapsulate identification
 * information for an ADI device, including the chip type, product ID,
 * product grade, and device revision. This structure is essential for
 * distinguishing between different versions and types of devices,
 * allowing for appropriate handling and configuration within the API.
 *
 * @param chip_type A uint8_t representing the chip type code.
 * @param prod_id A uint16_t representing the product ID code.
 * @param prod_grade A uint8_t representing the product grade code.
 * @param dev_revision A uint8_t representing the device revision.
 ******************************************************************************/
typedef struct {
	/** Chip Type Code */
	uint8_t chip_type;
	/** Product ID Code */
	uint16_t prod_id;
	/** Product Grade Code*/
	uint8_t prod_grade;
	/** Device Revision */
	uint8_t dev_revision;
} adi_chip_id_t;


/***************************************************************************//**
 * @brief The `spi_sdo_config_t` is an enumeration that defines the
 * configuration options for the SPI interface's data output pins. It
 * includes options for setting the SPI SDO pin for 4-wire communication
 * and the SPI SDIO pin for 3-wire communication, as well as a default
 * invalid option for error checking. This enum is used to configure the
 * SPI interface in a way that matches the hardware requirements of the
 * connected device.
 *
 * @param SPI_NONE Represents an invalid SPI configuration, used for error
 * checking.
 * @param SPI_SDO Activates the SPI SDO (Serial Data Output) pin for 4-wire SPI
 * communication.
 * @param SPI_SDIO Activates the SPI SDIO (Serial Data Input-Output) pin for
 * 3-wire SPI communication.
 * @param SPI_CONFIG_MAX Marks the end of the enum, used for boundary checking.
 ******************************************************************************/
typedef enum {
	/* Keep this invalid value as 0, so the API can test for wrong setting.*/
	SPI_NONE = 0,
	/* Set SPI SDO (Serial Data Output) pin as active.
	 * This is in case the 4-wire SPI is needed.*/
	SPI_SDO = 1,
	/* Set SPI SDIO (Serial Data Input-Output) pin as active.
	 * This is in case the 3-wire SPI is needed.*/
	SPI_SDIO = 2,
	/* Keep it last. */
	SPI_CONFIG_MAX = 3
} spi_sdo_config_t;

/***************************************************************************//**
 * @brief The `signal_type_t` is an enumeration that defines different types of
 * signal types used in the system. It includes three possible values:
 * `SIGNAL_CMOS` for CMOS signal type, `SIGNAL_LVDS` for LVDS signal
 * type, and `SIGNAL_UNKNOWN` for any undefined or unknown signal type.
 * This enumeration is useful for categorizing and handling different
 * signal types within the application.
 *
 * @param SIGNAL_CMOS Represents the CMOS signal type with a value of 0.
 * @param SIGNAL_LVDS Represents the LVDS signal type.
 * @param SIGNAL_UNKNOWN Represents an unknown or undefined signal type.
 ******************************************************************************/
typedef enum {
	SIGNAL_CMOS = 0, /**CMOS SIGNAL TYPE */
	SIGNAL_LVDS,	 /**LVDS SIGNAL TYPE */
	SIGNAL_UNKNOWN	 /**UNKNOW/UNDEFINED SIGNAL TYPE */
} signal_type_t;

/***************************************************************************//**
 * @brief The `signal_coupling_t` is an enumeration that defines the types of
 * signal coupling modes available, specifically AC, DC, and an unknown
 * or undefined state. This data structure is used to categorize the
 * coupling mode of a signal, which is crucial for signal processing and
 * interfacing in electronic systems.
 *
 * @param COUPLING_AC Represents an AC coupled signal.
 * @param COUPLING_DC Represents a DC signal.
 * @param COUPLING_UNKNOWN Represents an unknown or undefined coupling.
 ******************************************************************************/
typedef enum {
	COUPLING_AC = 0,	 /**AC COUPLED SIGNAL */
	COUPLING_DC,	 	 /**DC SIGNAL  */
	COUPLING_UNKNOWN	 /**UNKNOWN/UNDEFINED COUPLING */
} signal_coupling_t;

/***************************************************************************//**
 * @brief The `jesd_link_t` is an enumeration that defines constants for
 * identifying different JESD links in a system. It includes specific
 * values for individual links (JESD_LINK_0 and JESD_LINK_1) as well as a
 * value representing all links (JESD_LINK_ALL). This enumeration is
 * useful for configuring or referencing specific JESD links in
 * applications that utilize the JESD204 interface standard.
 *
 * @param JESD_LINK_0 Represents JESD LINK 0 with a value of 0x0.
 * @param JESD_LINK_1 Represents JESD LINK 1 with a value of 0x1.
 * @param JESD_LINK_ALL Represents all JESD LINKS with a value of 0xFF.
 ******************************************************************************/
typedef enum {
	JESD_LINK_0 = 0x0,     /**< JESD LINK 0 */
	JESD_LINK_1 = 0x1,     /**< JESD LINK 1 */
	JESD_LINK_ALL = 0xFF   /**< ALL JESD LINKS */
} jesd_link_t;

/***************************************************************************//**
 * @brief The `jesd_syncoutb_t` is an enumeration that defines the possible
 * SYNCOUTB output signals for a JESD interface. It includes specific
 * values for individual SYNCOUTB signals (0 and 1) as well as a value
 * representing all SYNCOUTB signals. This enumeration is used to
 * configure or identify the SYNCOUTB signal state in a JESD interface
 * setup.
 *
 * @param SYNCOUTB_0 Represents the SYNCOUTB signal 0 with a value of 0x0.
 * @param SYNCOUTB_1 Represents the SYNCOUTB signal 1 with a value of 0x1.
 * @param SYNCOUTB_ALL Represents all SYNCOUTB signals with a value of 0xFF.
 ******************************************************************************/
typedef enum {
	SYNCOUTB_0 = 0x0,     /**< SYNCOUTB 0 */
	SYNCOUTB_1 = 0x1,     /**< SYNCOUTB 1 */
	SYNCOUTB_ALL = 0xFF   /**< ALL SYNCOUTB SIGNALS */
} jesd_syncoutb_t;

/***************************************************************************//**
 * @brief The `jesd_sysref_mode_t` is an enumeration that defines the different
 * modes of SYSREF synchronization for JESD interfaces. It includes modes
 * for no support, one-shot synchronization, continuous synchronization,
 * monitoring, and an invalid mode to handle erroneous settings. This
 * enumeration is used to configure and manage the SYSREF signal behavior
 * in JESD systems.
 *
 * @param SYSREF_NONE No SYSREF Support.
 * @param SYSREF_ONESHOT ONE-SHOT SYSREF.
 * @param SYSREF_CONT Continuous Sysref Synchronisation.
 * @param SYSREF_MON SYSREF monitor Mode.
 * @param SYSREF_MODE_INVLD Invalid SYSREF mode.
 ******************************************************************************/
typedef enum {
	SYSREF_NONE,     /**< No SYSREF Support */
	SYSREF_ONESHOT,  /**< ONE-SHOT SYSREF */
	SYSREF_CONT,     /**< Continuous Sysref Synchronisation */
	SYSREF_MON,      /**< SYSREF monitor Mode */
	SYSREF_MODE_INVLD
} jesd_sysref_mode_t;

/***************************************************************************//**
 * @brief The `jesd_param_t` structure is a compound data type used to
 * encapsulate various parameters related to the JESD (Joint Electron
 * Device Engineering Council) interface, which is a standard for high-
 * speed serial data communication. This structure includes fields for
 * lane parameters, octet parameters, converter parameters, sample
 * parameters, high-density parameters, multiframe parameters, converter
 * resolution, bit packing, device ID, bank ID, lane ID, and versioning,
 * all of which are essential for configuring and managing JESD
 * interfaces in electronic devices.
 *
 * @param jesd_L JESD Lane Param L.
 * @param jesd_F JESD Octet Param F.
 * @param jesd_M JESD Converter Param M.
 * @param jesd_S JESD No of Sample Param S.
 * @param jesd_HD JESD High Density Param HD.
 * @param jesd_K JESD multiframe Param K.
 * @param jesd_N JESD Converter Resolution Param N.
 * @param jesd_NP JESD Bit Packing Sample NP.
 * @param jesd_CF JESD Param CF.
 * @param jesd_CS JESD Param CS.
 * @param jesd_DID JESD Device ID Param DID.
 * @param jesd_BID JESD Bank ID Param BID.
 * @param jesd_LID0 JESD Lane ID for Lane 0 Param LIDO.
 * @param jesd_JESDV JESD Version.
 ******************************************************************************/
typedef struct {
	uint8_t jesd_L;     /**< JESD Lane Param L. */
	uint8_t jesd_F;     /**< JESD Octet Param F. */
	uint8_t jesd_M;     /**< JESD Converter Param M. */
	uint8_t jesd_S;     /**< JESD No of Sample Param S. */


	uint8_t jesd_HD;	/** JESD High Density Param HD.*/
	uint8_t jesd_K;     /**< JESD multiframe Param K.  */
	uint8_t jesd_N;     /**< JESD Converter Resolution Param N.  */
	uint8_t jesd_NP;    /**< JESD Bit Packing Sample NP. */
	uint8_t jesd_CF;    /**< JESD Param CF.  */
	uint8_t jesd_CS;    /**< JESD Param CS.  */

	uint8_t jesd_DID;   /**< JESD Device ID Param DID. */
	uint8_t jesd_BID;   /**< JESD Bank ID. Param BID */
	uint8_t jesd_LID0;  /**< JESD Lane ID for Lane 0 Param LIDO*/
	uint8_t jesd_JESDV; /**< JESD Version */
} jesd_param_t;

/***************************************************************************//**
 * @brief The `jesd_prbs_pattern_t` is an enumeration that defines various
 * Pseudo-Random Binary Sequence (PRBS) patterns used in JESD (Joint
 * Electron Device Engineering Council) interfaces. These patterns, such
 * as PRBS7, PRBS15, and PRBS31, are commonly used for testing and
 * validation of high-speed data communication links by simulating random
 * data. The enumeration also includes a `PRBS_NONE` option to indicate
 * that no PRBS pattern is applied, and a `PRBS_MAX` to represent the
 * count of patterns available in the enumeration.
 *
 * @param PRBS_NONE Indicates that the PRBS (Pseudo-Random Binary Sequence) is
 * turned off.
 * @param PRBS7 Represents the PRBS7 pattern, a specific type of pseudo-random
 * binary sequence.
 * @param PRBS15 Represents the PRBS15 pattern, another type of pseudo-random
 * binary sequence.
 * @param PRBS31 Represents the PRBS31 pattern, a longer pseudo-random binary
 * sequence.
 * @param PRBS_MAX Denotes the number of members in this enumeration.
 ******************************************************************************/
typedef enum {
	PRBS_NONE, /**< PRBS OFF */
	PRBS7,     /**< PRBS7 pattern */
	PRBS15,    /**< PRBS15 pattern */
	PRBS31,    /**< PRBS31 pattern */
	PRBS_MAX   /**< Number of member in this enum */
} jesd_prbs_pattern_t;

#endif /* !__API_DEF_H__ */

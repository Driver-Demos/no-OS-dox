/**
 * \file AD917x.h
 *
 * \brief AD917X API interface header file
 *
 * This file contains all the publicly exposed methods and data structures to
 * interface with the AD917X API.
 *
 * Release 1.1.X
 *
 * Copyright(c) 2017 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
 */

#ifndef __AD917XAPI_H__
#define __AD917XAPI_H__

#include "api_def.h"
#include "no_os_util.h"

/** \addtogroup API
 *  @{
 */

/***************************************************************************//**
 * @brief The `ad917x_dds_select_t` is an enumeration that defines the selection
 * options for the Direct Digital Synthesis (DDS) paths in the AD917X
 * device. It provides two options: `AD917X_DDSM` for selecting the main
 * DDS path and `AD917X_DDSC` for selecting the channel DDS path. This
 * enumeration is used to configure which DDS path is active in the
 * device's operation.
 *
 * @param AD917X_DDSM Represents the Main DDS selection with a value of 0.
 * @param AD917X_DDSC Represents the Channel DDS selection with a value of 1.
 ******************************************************************************/
typedef enum {
	/** Main DDS */
	AD917X_DDSM = 0,
	/** Channel DDS */
	AD917X_DDSC = 1
} ad917x_dds_select_t;

/***************************************************************************//**
 * @brief The `ad917x_dac_select_t` is an enumeration that defines the selection
 * of DACs (Digital-to-Analog Converters) for the AD917X series. It
 * provides options to select either no DAC, DAC0, or DAC1, which are
 * used to configure and control the specific DAC channels in the AD917X
 * device. This enumeration is crucial for specifying which DAC is being
 * targeted for operations such as configuration, data transmission, or
 * other DAC-specific functionalities.
 *
 * @param AD917X_DAC_NONE Represents no DAC selected with a value of 0.
 * @param AD917X_DAC0 Represents DAC0 selected with a value of 1.
 * @param AD917X_DAC1 Represents DAC1 selected with a value of 2.
 ******************************************************************************/
typedef enum {
	/** No DAC */
	AD917X_DAC_NONE = 0,
	/** DAC0 */
	AD917X_DAC0 = 1,
	/** DAC1 */
	AD917X_DAC1 = 2
} ad917x_dac_select_t;

/***************************************************************************//**
 * @brief The `ad917x_channel_select_t` is an enumeration that defines constants
 * for selecting different channels in the AD917X device. Each channel is
 * represented by a bit position, allowing for bitwise operations to
 * select multiple channels simultaneously. This enumeration is used to
 * specify which channel or channels are active or being configured in
 * the device.
 *
 * @param AD917X_CH_NONE Represents no channel selected with a value of 0.
 * @param AD917X_CH_0 Represents Channel 0 with a value derived from
 * NO_OS_BIT(0).
 * @param AD917X_CH_1 Represents Channel 1 with a value derived from
 * NO_OS_BIT(1).
 * @param AD917X_CH_2 Represents Channel 2 with a value derived from
 * NO_OS_BIT(2).
 * @param AD917X_CH_3 Represents Channel 3 with a value derived from
 * NO_OS_BIT(3).
 * @param AD917X_CH_4 Represents Channel 4 with a value derived from
 * NO_OS_BIT(4).
 * @param AD917X_CH_5 Represents Channel 5 with a value derived from
 * NO_OS_BIT(5).
 ******************************************************************************/
typedef enum {
	/** No Channel */
	AD917X_CH_NONE = 0,
	/** Channel 0 */
	AD917X_CH_0 = NO_OS_BIT(0),
	/** Channel 1 */
	AD917X_CH_1 = NO_OS_BIT(1),
	/** Channel 2 */
	AD917X_CH_2 = NO_OS_BIT(2),
	/** Channel 3 */
	AD917X_CH_3 = NO_OS_BIT(3),
	/** Channel 4 */
	AD917X_CH_4 = NO_OS_BIT(4),
	/** Channel 5 */
	AD917X_CH_5 = NO_OS_BIT(5)
} ad917x_channel_select_t;

/***************************************************************************//**
 * @brief The `ad917x_jesd_link_stat_t` structure is used to represent the
 * status of the JESD interface link for the AD917X device. It contains
 * four 8-bit fields, each representing a specific status for all JESD
 * lanes: code group synchronization, frame synchronization, checksum
 * validity, and initial lane synchronization. These fields are bitwise
 * indicators, allowing for the monitoring and management of the JESD
 * link's operational status.
 *
 * @param code_grp_sync_stat Bit wise Code Group Sync Status for all JESD Lanes.
 * @param frame_sync_stat Bit wise Frame Sync Status for all JESD Lanes.
 * @param good_checksum_stat Bit wise Good Checksum Status for all JESD Lanes.
 * @param init_lane_sync_stat Bit wise Initial Lane Sync Status for all JESD
 * Lanes.
 ******************************************************************************/
typedef struct {
	/** Bit wise Code Group Sync Status for all JESD Lanes*/
	uint8_t code_grp_sync_stat;
	/** Bit wise Frame Sync Status for all JESD Lanes*/
	uint8_t frame_sync_stat;
	/** Bit wise Good Checksum Status for all JESD Lanes*/
	uint8_t good_checksum_stat;
	/** Bit wise Initial Lane Sync Status for all JESD Lanes*/
	uint8_t init_lane_sync_stat;
} ad917x_jesd_link_stat_t;

/***************************************************************************//**
 * @brief The `ad917x_jesd_serdes_pll_flg_t` is an enumeration that defines
 * various status flags related to the SERDES PLL (Phase-Locked Loop) in
 * the AD917X device. Each enumerator represents a specific status
 * condition of the PLL, such as lock status, regulator readiness, VCO
 * calibration status, and loss of lock threshold. These flags are used
 * to monitor and control the operational state of the SERDES PLL,
 * ensuring proper synchronization and functionality within the device's
 * JESD interface.
 *
 * @param AD917X_PLL_LOCK_STAT Serdes PLL lock Status Flag.
 * @param AD917X_PLL_REG_RDY Serdes PLL Regulator RDY Status Flag.
 * @param AD917X_PLL_CAL_STAT Serdes PLL VCO Calibration Status Flag.
 * @param AD917X_PLL_LOSSLOCK Serdes PLL Upper Calibration Threshold flag.
 ******************************************************************************/
typedef enum {
	AD917X_PLL_LOCK_STAT = 0x1, /**< Serdes PLL lock Status Flag*/
	AD917X_PLL_REG_RDY = 0x2,   /**< Serdes PLL Regulator RDY Status Flag*/
	AD917X_PLL_CAL_STAT = 0x4,  /**< Serdes PLL VCO Calibration Status Flag*/
	AD917X_PLL_LOSSLOCK = 0x8  /**< Serdes PLL Upper Calibration Threshold flag*/
} ad917x_jesd_serdes_pll_flg_t;

/***************************************************************************//**
 * @brief The `ad917x_handle_t` structure is a comprehensive data structure used
 * to manage and configure the AD917X device, which is a high-performance
 * digital-to-analog converter (DAC). It encapsulates various
 * configuration parameters and function pointers necessary for hardware
 * abstraction layer (HAL) operations, such as SPI communication, delay
 * handling, and control of specific hardware pins. This structure allows
 * for flexible initialization and de-initialization of the device, as
 * well as setting up the DAC's clock frequency and signal types, making
 * it integral to the device's API for efficient and effective operation.
 *
 * @param user_data Void pointer to user defined data for HAL initialization.
 * @param sdo DAC SPI interface configuration.
 * @param syncoutb Desired Signal type for SYNCOUTB signal.
 * @param sysref Desired Input coupling for sysref signal.
 * @param dac_freq_hz DAC Clock Frequency in Hz, valid range 2.9GHz to 12GHz.
 * @param dev_xfer Function Pointer to HAL SPI access function.
 * @param delay_us Function Pointer to HAL delay function.
 * @param tx_en_pin_ctrl Function Pointer to HAL TX_ENABLE Pin Ctrl function.
 * @param reset_pin_ctrl Function Point to HAL RESETB Pin Ctrl Function.
 * @param hw_open Function Pointer to HAL initialization function.
 * @param hw_close Function Pointer to HAL de-initialization function.
 ******************************************************************************/
typedef struct {
	void *user_data;        /**< Void pointer to user defined data for HAL initialization */
	spi_sdo_config_t sdo;      /**< DAC SPI interface configuration*/
	signal_type_t syncoutb;    /**< Desired Signal type for SYNCOUTB signal*/
	signal_coupling_t sysref;  /**< Desired Input coupling for sysref signal*/
	uint64_t dac_freq_hz;   /**< DAC Clock Frequency in Hz. Valid range 2.9GHz to 12GHz*/
	spi_xfer_t dev_xfer;    /**< Function Pointer to HAL SPI access function*/
	delay_us_t delay_us;    /**< Function Pointer to HAL delay function*/
	tx_en_pin_ctrl_t
	tx_en_pin_ctrl; /**< Function Pointer to HAL TX_ENABLE Pin Ctrl function*/
	reset_pin_ctrl_t
	reset_pin_ctrl; /**< Function Point to HAL RESETB Pin Ctrl Function*/
	hw_open_t hw_open;      /**< Function Pointer to HAL initialization function*/
	hw_close_t
	hw_close;    /**< Function Pointer to HAL de-initialization function*/
} ad917x_handle_t;

/***************************************************************************//**
 * @brief This function initializes the AD917X device and must be called before
 * any other API functions. It sets up the necessary internal states and
 * configurations required for the device to operate. If the `hw_open`
 * function pointer in the handle is not null, it will be called to
 * perform any hardware-specific initialization. This function checks for
 * valid pointers and configurations, returning specific error codes if
 * any issues are detected. It is recommended to call the reset function
 * after initialization to ensure all settings are at their default
 * values.
 *
 * @param h A pointer to an `ad917x_handle_t` structure, which must not be null.
 * This structure should be properly configured with valid function
 * pointers and settings before calling this function. Invalid pointers
 * or configurations will result in specific error codes.
 * @return Returns an `int32_t` indicating the success or failure of the
 * initialization. Possible return values include `API_ERROR_OK` for
 * success, or various error codes indicating specific issues with the
 * handle or configuration.
 ******************************************************************************/
int32_t ad917x_init(ad917x_handle_t *h);

/***************************************************************************//**
 * @brief This function should be called as the final step in the lifecycle of
 * the AD917X device to properly de-initialize it. It ensures that any
 * hardware resources associated with the device are released. The
 * function checks if the provided handle is valid and, if a hardware
 * close function is specified in the handle, it calls this function to
 * perform any necessary hardware de-initialization. It is important to
 * ensure that no other API functions are called after this function, as
 * it signifies the end of the device's operational lifecycle.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * the handle is invalid, the function returns an error. The caller
 * retains ownership of the handle.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_HW_CLOSE if the hardware close operation fails.
 ******************************************************************************/
int32_t ad917x_deinit(ad917x_handle_t *h);

/***************************************************************************//**
 * @brief This function performs a reset of the AD917X device, either through a
 * hardware pin or via SPI register, depending on the `hw_reset`
 * parameter. It should be called to reset all SPI registers to their
 * default values and initiate the required initialization sequence. The
 * function must be called with a valid device handle and after the
 * device has been initialized. It handles both hardware and software
 * reset scenarios, ensuring the device is properly configured post-
 * reset.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * handle should be properly initialized before calling this function.
 * @param hw_reset A parameter indicating the type of reset: 1 for hardware
 * reset, 0 for software reset. Values other than 0 or 1 are
 * considered invalid and will result in an error.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. Possible return values include API_ERROR_OK for
 * success, API_ERROR_INVALID_HANDLE_PTR for a null handle,
 * API_ERROR_INVALID_PARAM for invalid parameters, and other error codes
 * for specific failures during the reset process.
 ******************************************************************************/
int32_t ad917x_reset(ad917x_handle_t *h, uint8_t hw_reset);

/***************************************************************************//**
 * @brief Use this function to obtain the chip type, product ID, product grade,
 * and device revision of the AD917X device. It is essential to call this
 * function after initializing the device with `ad917x_init`. Ensure that
 * both the device handle and the chip ID pointer are valid and not null
 * before calling this function. The function will return an error if the
 * handle or chip ID pointer is invalid, or if there is a failure in
 * reading the necessary registers.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership and is responsible for ensuring it points
 * to a valid, initialized device.
 * @param chip_id Pointer to a variable of type `adi_chip_id_t` where the chip
 * identification data will be stored. Must not be null. The
 * function will populate this structure with the chip type,
 * product ID, product grade, and device revision.
 * @return Returns an integer status code: `API_ERROR_OK` on success,
 * `API_ERROR_INVALID_HANDLE_PTR` if the handle is null,
 * `API_ERROR_INVALID_PARAM` if the chip ID pointer is null, or an error
 * code if a register read fails.
 ******************************************************************************/
int32_t ad917x_get_chip_id(ad917x_handle_t *h, adi_chip_id_t *chip_id);


/***************************************************************************//**
 * @brief This function writes an 8-bit data value to a specified SPI register
 * address on the AD917X device. It requires a valid device handle and a
 * properly configured SPI transfer function. The function should be
 * called when a register write operation is needed, and it returns an
 * error code if the handle or transfer function is invalid, or if the
 * SPI transfer fails.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null and
 * must have a valid SPI transfer function configured.
 * @param address The 16-bit SPI address of the AD917X device register to which
 * the data will be written.
 * @param data The 8-bit value to be written to the specified SPI register
 * address.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null,
 * API_ERROR_INVALID_XFER_PTR if the transfer function is null, or
 * API_ERROR_SPI_XFER if the SPI transfer fails.
 ******************************************************************************/
int32_t ad917x_register_write(ad917x_handle_t *h,
			      const uint16_t address, const uint8_t data);
/***************************************************************************//**
 * @brief This function reads an 8-bit value from a specified SPI register
 * address on the AD917X device. It requires a valid device handle and a
 * pointer to store the read data. The function should be called after
 * the device has been properly initialized. It returns an error if the
 * handle or the transfer function pointer is invalid, or if the SPI
 * transfer fails.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null and
 * should be properly initialized before calling this function.
 * @param address The 16-bit SPI address from which the data will be read. The
 * address should be within the valid range for the device's SPI
 * register map.
 * @param data Pointer to an 8-bit variable where the read data will be stored.
 * Must not be null.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null,
 * API_ERROR_INVALID_XFER_PTR if the transfer function pointer is null,
 * or API_ERROR_SPI_XFER if the SPI transfer fails.
 ******************************************************************************/
int32_t ad917x_register_read(ad917x_handle_t *h,
			     const uint16_t address, uint8_t *data);
/***************************************************************************//**
 * @brief Use this function to obtain the major, minor, and release candidate
 * (RC) revision numbers of the AD917X API. This function requires valid
 * pointers for each revision number parameter to store the respective
 * values. It is essential to ensure that these pointers are not null
 * before calling the function, as null pointers will result in an error.
 * The function returns an error code indicating success or failure, with
 * specific error codes for invalid parameters.
 *
 * @param h Pointer to the AD917X device reference handle. The handle must be
 * properly initialized before calling this function.
 * @param rev_major Pointer to a uint8_t variable where the major revision
 * number will be stored. Must not be null.
 * @param rev_minor Pointer to a uint8_t variable where the minor revision
 * number will be stored. Must not be null.
 * @param rev_rc Pointer to a uint8_t variable where the RC revision number will
 * be stored. Must not be null.
 * @return Returns an int32_t error code: API_ERROR_OK on success, or
 * API_ERROR_INVALID_PARAM if any of the revision number pointers are
 * null.
 ******************************************************************************/
int32_t ad917x_get_revision(ad917x_handle_t *h, uint8_t *rev_major,
			    uint8_t *rev_minor, uint8_t *rev_rc);


/***************************************************************************//**
 * @brief This function configures the on-chip DAC PLL of the AD917X device,
 * allowing it to generate a DAC clock internally or use an external
 * clock. It should be called when setting up the DAC clock
 * configuration. The function requires valid parameters for the PLL
 * enable flag, M divider, N divider, and VCO divider. It returns an
 * error if the handle is null or if any parameter is out of its valid
 * range. Ensure the device is initialized before calling this function.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param dac_pll_en Enable flag for internal DAC clock generation. Valid values
 * are 0 (disable) and 1 (enable).
 * @param m_div Reference clock pre-divider. Valid range is 1 to 4.
 * @param n_div VCO feedback divider ratio. Valid range is 2 to 50.
 * @param vco_div VCO divider for the desired DAC clock. Valid range is 1 to 3.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., API_ERROR_INVALID_HANDLE_PTR,
 * API_ERROR_INVALID_PARAM).
 ******************************************************************************/
int32_t ad917x_set_dac_pll_config(ad917x_handle_t *h, uint8_t dac_pll_en,
				  uint8_t m_div, uint8_t n_div, uint8_t vco_div);
/***************************************************************************//**
 * @brief This function configures the DAC clock frequency for the AD917X
 * device, which can be directly applied or generated using a reference
 * clock. It must be called after initializing the device and before any
 * operations that depend on the DAC clock frequency. The function
 * ensures the frequency is within the valid range and applies necessary
 * initialization sequences. It returns an error if the handle is invalid
 * or if the frequency is out of range.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param dac_clk_freq_hz The desired DAC clock frequency in Hz. Valid range is
 * 2.9 GHz to 12 GHz for AD9172 and AD9173, and 2.9 GHz
 * to 6 GHz for AD9171. The function returns an error if
 * the frequency is out of range.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle or
 * parameter).
 ******************************************************************************/
int32_t ad917x_set_dac_clk_freq(ad917x_handle_t *h,
				uint64_t dac_clk_freq_hz);

/***************************************************************************//**
 * @brief This function retrieves the current DAC clock frequency from the
 * AD917X device and stores it in the provided variable. It should be
 * called when the user needs to know the current operating frequency of
 * the DAC clock. The function requires a valid device handle and a non-
 * null pointer to store the frequency value. It returns an error if the
 * handle or the pointer is invalid.
 *
 * @param h A pointer to an ad917x_handle_t structure representing the device
 * handle. Must not be null. If null, the function returns
 * API_ERROR_INVALID_HANDLE_PTR.
 * @param dac_clk_freq_hz A pointer to a uint64_t variable where the DAC clock
 * frequency in Hz will be stored. Must not be null. If
 * null, the function returns API_ERROR_INVALID_PARAM.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. API_ERROR_OK is returned on success, while specific error
 * codes are returned for invalid parameters.
 ******************************************************************************/
int32_t ad917x_get_dac_clk_freq(ad917x_handle_t *h,
				uint64_t *dac_clk_freq_hz);
/***************************************************************************//**
 * @brief Use this function to check the lock status of the DAC's PLL and DLL in
 * the AD917X device. This function is useful for verifying that the DAC
 * clock is stable and properly configured. It should be called after the
 * DAC clock has been set up. The function can return the lock status of
 * both the PLL and DLL, but you can choose to ignore either by passing
 * NULL for the respective parameter. Ensure that the device handle is
 * valid before calling this function.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * null, the function returns an error.
 * @param pll_lock_stat Pointer to a uint8_t where the DAC PLL lock status will
 * be stored. Can be null if PLL status is not needed. If
 * not null, it will be set to 0 if the PLL is not locked,
 * or 1 if it is locked.
 * @param dll_lock_stat Pointer to a uint8_t where the DAC DLL lock status will
 * be stored. Can be null if DLL status is not needed. If
 * not null, it will be set to 0 if the DLL is not locked,
 * or 1 if it is locked.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code if the handle is invalid or if a register read fails.
 ******************************************************************************/
int32_t ad917x_get_dac_clk_status(ad917x_handle_t *h,
				  uint8_t *pll_lock_stat, uint8_t *dll_lock_stat);

/***************************************************************************//**
 * @brief This function configures the output clock divider for the AD917X
 * device, which is essential for setting the desired output clock
 * frequency. It should be called after the device has been properly
 * initialized. The function requires a valid device handle and a divider
 * value within the specified range. If the handle is null or the divider
 * value is out of range, the function returns an error. This function
 * does not modify any other device settings or states.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param l_div Output clock divider setting. Valid range is 1 to 4. If the
 * value is outside this range, the function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the divider value is out of range.
 ******************************************************************************/
int32_t ad917x_set_clkout_config(ad917x_handle_t *h, uint8_t l_div);

/***************************************************************************//**
 * @brief This function sets the desired DAC clock frequency for the AD917X
 * device, with the option to enable or disable the internal PLL for
 * clock generation. It should be used when configuring the DAC clock
 * source and frequency, either directly from an external clock or
 * generated internally using a reference clock. The function requires a
 * valid device handle and checks for valid frequency ranges and PLL
 * settings. It returns an error code if any parameter is invalid or if
 * the configuration fails.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param dac_clk_freq_hz Desired DAC clock frequency in Hz. Must be within the
 * valid range for the specific AD917X model.
 * @param dac_pll_en Enable flag for internal DAC clock generation. Must be 0
 * (disabled) or 1 (enabled).
 * @param ref_clk_freq_hz Reference clock frequency in Hz used for internal DAC
 * clock generation. Must be within the valid range if
 * dac_pll_en is 1; otherwise, it should be set to 0.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t ad917x_set_dac_clk(ad917x_handle_t *h,
			   uint64_t dac_clk_freq_hz, uint8_t dac_pll_en, uint64_t ref_clk_freq_hz);

/***************************************************************************//**
 * @brief This function sets up the JESD interface parameters and the digital
 * datapath interpolation mode for the AD917X device. It should be used
 * when configuring the device for data transmission over the JESD
 * interface. The function requires a valid device handle and specific
 * configuration parameters. It returns an error if the configuration is
 * unsupported or if any parameters are invalid. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param dual_en Dual Link enable setting. Valid values are 0 for Single Link
 * Mode and 1 for Dual Link Mode. Values greater than 1 are
 * invalid and will result in an error.
 * @param jesd_mode Desired JESD link mode. Valid range is 0 to 21. Values
 * outside this range are invalid and will result in an error.
 * @param ch_intpl Desired channel interpolation. Valid range is 1 to 12. Values
 * outside this range are invalid and will result in an error.
 * @param dp_intpl Desired main DAC datapath interpolation. Valid range is 1 to
 * 10. Values outside this range are invalid and will result in
 * an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle, invalid
 * parameter).
 ******************************************************************************/
int32_t ad917x_jesd_config_datapath(ad917x_handle_t *h, uint8_t dual_en,
				    uint8_t jesd_mode, uint8_t ch_intpl, uint8_t dp_intpl);

/***************************************************************************//**
 * @brief This function checks the validity of the current JESD and
 * interpolation mode configuration for the AD917X device. It should be
 * called after configuring the JESD interface to verify if the settings
 * are valid. The function requires a valid device handle and a pointer
 * to store the configuration status. It returns an error if the handle
 * or the pointer is invalid.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param cfg_valid Pointer to a uint8_t variable where the configuration status
 * will be stored. Must not be null. If null, the function
 * returns API_ERROR_INVALID_PARAM.
 * @return Returns API_ERROR_OK if the operation is successful. The cfg_valid
 * parameter is set to 1 if the configuration is valid, or 0 if it is
 * invalid.
 ******************************************************************************/
int32_t ad917x_jesd_get_cfg_status(ad917x_handle_t *h, uint8_t *cfg_valid);

/***************************************************************************//**
 * @brief This function is used to read back all the currently configured JESD
 * interface parameters for the AD917X device. It should be called when
 * you need to verify or log the current JESD settings. Ensure that the
 * AD917X device has been properly initialized before calling this
 * function. The function will populate the provided `jesd_param`
 * structure with the current JESD configuration values. If the handle or
 * the parameter pointer is invalid, the function will return an error
 * code.
 *
 * @param h A pointer to the AD917X device reference handle. Must not be null.
 * If null, the function returns an error code indicating an invalid
 * handle.
 * @param jesd_param A pointer to a `jesd_param_t` structure where the current
 * JESD interface parameters will be stored. Must not be null.
 * If null, the function returns an error code indicating an
 * invalid parameter.
 * @return Returns an integer status code: `API_ERROR_OK` on success, or an
 * error code if the handle or parameter pointer is invalid.
 ******************************************************************************/
int32_t ad917x_jesd_get_cfg_param(ad917x_handle_t *h,
				  jesd_param_t *jesd_param);

/***************************************************************************//**
 * @brief This function is used to control the power state of the SYSREF input
 * interface on the AD917X device. It should be called when you need to
 * enable or disable the SYSREF input, which is crucial for synchronizing
 * the device with an external system. The function requires a valid
 * device handle and a control flag to specify the desired state. It is
 * important to ensure that the handle is not null and the control flag
 * is either 0 or 1, as invalid inputs will result in an error.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param en Enable control for the SYSREF input interface. Valid values are 0
 * (disable) and 1 (enable). Values outside this range will result in
 * an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int32_t ad917x_jesd_set_sysref_enable(ad917x_handle_t *h, uint8_t en);
/***************************************************************************//**
 * @brief This function checks whether the SYSREF input interface of the AD917X
 * device is enabled or disabled. It should be called when you need to
 * verify the current state of the SYSREF input. Ensure that the device
 * handle is properly initialized before calling this function. The
 * function will return an error if the handle or the output parameter is
 * null.
 *
 * @param h A pointer to an initialized AD917X device handle. Must not be null.
 * If null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param en A pointer to a uint8_t variable where the enable status of the
 * SYSREF input will be stored. Must not be null. If null, the
 * function returns API_ERROR_INVALID_PARAM.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors.
 ******************************************************************************/
int32_t ad917x_jesd_get_sysref_enable(ad917x_handle_t *h, uint8_t *en);


/***************************************************************************//**
 * @brief This function configures the LMFC delay and variance for a specified
 * JESD link on the AD917X device. It should be used when you need to
 * adjust the dynamic link latency and variable delay buffer for JESD
 * link setup. Ensure that the device handle is valid and initialized
 * before calling this function. The function will return an error if the
 * handle is null, or if the delay or variance values exceed their
 * respective maximums. It also checks if the specified link is valid.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param link Specifies the target JESD link to configure. Valid values are
 * JESD_LINK_0, JESD_LINK_1, or JESD_LINK_ALL.
 * @param delay Dynamic link latency for LMFC Rx in PCLK cycles. Valid range is
 * 0 to 63.
 * @param var Variable delay buffer in PCLK cycles. Valid range is 0 to 12.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle, invalid
 * parameter).
 ******************************************************************************/
int32_t ad917x_jesd_set_lmfc_delay(ad917x_handle_t *h, jesd_link_t link,
				   uint8_t delay, uint8_t var);
/***************************************************************************//**
 * @brief This function configures and enables or disables the SYNCOUTB output
 * signal for the AD917X device. It should be used when you need to
 * control the SYNCOUTB signal for synchronization purposes. The function
 * requires a valid device handle and specific SYNCOUTB signal selection.
 * It is important to ensure that the handle is properly initialized
 * before calling this function. Invalid parameters will result in an
 * error.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param syncoutb Target SYNCOUTB signal to configure. Valid values are
 * SYNCOUTB_0, SYNCOUTB_1, or SYNCOUTB_ALL.
 * @param en Enable or disable the SYNCOUTB signal. Valid values are 0 (disable)
 * or 1 (enable).
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is invalid, or
 * API_ERROR_INVALID_PARAM if parameters are invalid.
 ******************************************************************************/
int32_t ad917x_jesd_set_syncoutb_enable(ad917x_handle_t *h,
					jesd_syncoutb_t syncoutb, uint8_t en);

/***************************************************************************//**
 * @brief This function is used to control the scrambler feature of the JESD
 * interface on the AD917X device. It should be called when you need to
 * enable or disable scrambling for data transmission over the JESD
 * interface. The function must be called with a valid device handle, and
 * the scrambler can only be enabled or disabled (binary state). Ensure
 * that the device handle is properly initialized before calling this
 * function. The function will return an error if the handle is invalid
 * or if the enable parameter is outside the expected range.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param en Enable control for JESD Scrambler. Valid values are 0 (disable) and
 * 1 (enable). Any other value will result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or
 * API_ERROR_INVALID_PARAM if the enable parameter is invalid.
 ******************************************************************************/
int32_t ad917x_jesd_set_scrambler_enable(ad917x_handle_t *h, uint8_t en);

/***************************************************************************//**
 * @brief This function configures the mapping between logical and physical
 * lanes in the JESD interface of the AD917X device. It is used to route
 * physical JESD lanes to desired logical lanes, which is essential for
 * proper data alignment and transmission. The function should be called
 * after initializing the device and before enabling the JESD interface.
 * Ensure that the logical and physical lane indices are within the valid
 * range, as specified by the device's capabilities. Invalid handle or
 * parameter values will result in an error.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param logical_lane The logical lane index to which the physical lane should
 * be mapped. Valid range is 0 to LANE_INDEX_MAX.
 * @param physical_lane The physical lane index that is to be mapped to the
 * specified logical lane. Valid range is 0 to
 * LANE_INDEX_MAX.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameters.
 ******************************************************************************/
int32_t ad917x_jesd_set_lane_xbar(ad917x_handle_t *h,
				  uint8_t logical_lane, uint8_t physical_lane);

/***************************************************************************//**
 * @brief This function retrieves the current mapping of physical JESD lanes to
 * logical lanes for the AD917X device. It should be called when you need
 * to verify or utilize the current lane configuration. Ensure that the
 * device handle is valid and initialized before calling this function.
 * The function will populate the provided array with the mapping data,
 * where each element corresponds to a physical lane and its value
 * represents the logical lane it is mapped to.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * the handle is invalid, the function returns an error.
 * @param phy_log_map Pointer to an array of 8 uint8_t elements. This array will
 * be populated with the mapping of physical lanes to logical
 * lanes. Must not be null. If null, the function returns an
 * error.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int32_t ad917x_jesd_get_lane_xbar(ad917x_handle_t *h, uint8_t *phy_log_map);

/***************************************************************************//**
 * @brief This function is used to invert or un-invert a specified logical lane
 * in the JESD interface of the AD917X device. It is useful for adjusting
 * the signal routing of SERDIN signals. The function must be called with
 * a valid device handle and logical lane identifier. The logical lane
 * must be within the range of 0 to 7, and the invert parameter should be
 * either 0 or 1. If the handle is null or parameters are out of range,
 * the function returns an error.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param logical_lane Logical lane ID to be inverted, ranging from 0 to 7.
 * Values outside this range result in an error.
 * @param invert Desired invert status for the logical lane. Set to 1 to invert,
 * or 0 to de-invert. Values other than 0 or 1 result in an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code if the handle is invalid or parameters are out of range.
 ******************************************************************************/
int32_t ad917x_jesd_invert_lane(ad917x_handle_t *h,
				uint8_t logical_lane, uint8_t invert);
/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD interface datapath
 * on the AD917X device. It should be called with a valid device handle
 * after the device has been initialized. The function allows specifying
 * which lanes to enable or disable via a bitmask, and whether to run the
 * equalization routine. It is important to ensure that the parameters
 * are within the valid range to avoid errors.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param lanes_msk 8-bit mask indicating which JESD lanes to enable or disable.
 * Each bit represents a lane.
 * @param run_cal Flag to indicate whether to run the JESD interface
 * equalization routine. Valid values are 0 or 1.
 * @param en Enable control for the JESD interface. Valid values are 0 (disable)
 * or 1 (enable).
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure.
 ******************************************************************************/
int32_t ad917x_jesd_enable_datapath(ad917x_handle_t *h,
				    uint8_t lanes_msk, uint8_t run_cal, uint8_t en);
/***************************************************************************//**
 * @brief Use this function to obtain the current status of the SERDES PLL for
 * the AD917X device. It requires a valid device handle and a pointer to
 * store the PLL status. The function should be called after the device
 * has been initialized. It returns an error code if the handle or the
 * status pointer is invalid, or if there is an issue with SPI
 * communication.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * handle should be properly initialized before calling this function.
 * @param pll_status Pointer to a uint8_t variable where the PLL status will be
 * stored. Must not be null. The status will include various
 * flags indicating the lock, regulator, calibration, and loss
 * lock status of the SERDES PLL.
 * @return Returns an integer error code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int32_t ad917x_jesd_get_pll_status(ad917x_handle_t *h, uint8_t *pll_status);

/***************************************************************************//**
 * @brief This function is used to enable or disable a specific JESD link on the
 * AD917X device. It should be called after the JESD transmitter link is
 * enabled and ready, and the SERDES PLL is locked. The function requires
 * a valid device handle and appropriate link and enable parameters. It
 * returns an error if the handle is invalid or if the parameters are out
 * of range.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null.
 * @param link Specifies the target JESD link to enable or disable. Must be
 * within valid range or JESD_LINK_ALL.
 * @param en Enable control for the JESD link. 0 to disable, 1 to enable. Values
 * outside this range are considered invalid.
 * @return Returns API_ERROR_OK on success, or an error code if the handle is
 * invalid or parameters are out of range.
 ******************************************************************************/
int32_t ad917x_jesd_enable_link(ad917x_handle_t *h,
				jesd_link_t link, uint8_t en);

/***************************************************************************//**
 * @brief This function is used to obtain the current status of a specified JESD
 * link on the AD917x device. It should be called when you need to verify
 * the synchronization and integrity of the JESD link. The function
 * requires a valid device handle and a pointer to a structure where the
 * link status will be stored. It is important to ensure that the device
 * handle and the link status pointer are not null before calling this
 * function, as null values will result in an error. The function will
 * populate the provided structure with various status indicators, such
 * as code group sync, frame sync, good checksum, and initial lane sync
 * status for the specified link.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. The
 * caller retains ownership.
 * @param link The JESD link for which the status is being queried. Must be a
 * valid link identifier.
 * @param link_status Pointer to a structure of type ad917x_jesd_link_stat_t
 * where the link status will be stored. Must not be null.
 * The caller retains ownership.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors, such as invalid handle or
 * parameter.
 ******************************************************************************/
int32_t ad917x_jesd_get_link_status(ad917x_handle_t *h,
				    jesd_link_t link, ad917x_jesd_link_stat_t *link_status);

/***************************************************************************//**
 * @brief This function sets the page index for a specified DAC and channel on
 * the AD917X device. It is typically used to configure the device to
 * target specific DACs and channels for subsequent operations. The
 * function must be called with a valid device handle, and it is expected
 * that the handle has been properly initialized. If the handle is null,
 * the function will return an error. The DAC and channel parameters
 * allow for selecting specific or multiple DACs and channels by using
 * predefined constants or bitwise OR operations.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param dac DAC number to select. Valid values are AD917X_DAC_NONE,
 * AD917X_DAC0, AD917X_DAC1, or a combination using bitwise OR.
 * @param channel Channel number to select. Valid values are AD917X_CH_NONE,
 * AD917X_CH_0 to AD917X_CH_5, or a combination using bitwise OR.
 * @return Returns an integer status code: API_ERROR_OK on success, or
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null.
 ******************************************************************************/
int32_t ad917x_set_page_idx(ad917x_handle_t *h, const uint32_t dac,
			    const uint32_t channel);

/***************************************************************************//**
 * @brief Use this function to obtain the current page indices for the DAC and
 * channel in the AD917x device. It is essential to ensure that the
 * device handle and the pointers for DAC and channel are valid before
 * calling this function. This function is typically used to verify or
 * log the current configuration of the device. It must be called after
 * the device has been properly initialized.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * null, the function returns an API_ERROR_INVALID_HANDLE_PTR error.
 * @param dac Pointer to an integer where the selected DAC number will be
 * stored. Must not be null. If null, the function returns an
 * API_ERROR_INVALID_PARAM error.
 * @param channel Pointer to an integer where the selected channel number will
 * be stored. Must not be null. If null, the function returns an
 * API_ERROR_INVALID_PARAM error.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other codes indicate specific errors such as invalid handle or
 * parameter.
 ******************************************************************************/
int32_t ad917x_get_page_idx(ad917x_handle_t *h, int32_t *dac, int32_t *channel);

/***************************************************************************//**
 * @brief Use this function to set the gain for a specific channel on the AD917x
 * device. It is essential to ensure that the device handle is valid
 * before calling this function. The gain value is applied to the channel
 * currently selected by the CHANNEL_PAGE register. This function should
 * be called after the device has been properly initialized and
 * configured. If the handle is invalid, the function will return an
 * error.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. The
 * caller retains ownership.
 * @param gain The gain value to be set for the channel. It is a 16-bit unsigned
 * integer. The valid range is determined by the device
 * specifications.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure, such as
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null.
 ******************************************************************************/
int32_t ad917x_set_channel_gain(ad917x_handle_t *h, const uint16_t gain);

/***************************************************************************//**
 * @brief Use this function to obtain the current scalar gain value for a
 * channel on the AD917x device. It is essential to ensure that the
 * device handle is valid and initialized before calling this function.
 * The function will store the gain value in the provided pointer if
 * successful. This function should be called when you need to read the
 * gain setting for diagnostic or configuration purposes.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null and
 * should be properly initialized before use.
 * @param gain Pointer to a uint16_t variable where the gain value will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. Possible return values include API_ERROR_OK for success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, and
 * API_ERROR_INVALID_PARAM if the gain pointer is null.
 ******************************************************************************/
int32_t ad917x_get_channel_gain(ad917x_handle_t *h, uint16_t *gain);


/***************************************************************************//**
 * @brief This function configures the amplitude of the DC calibration tone for
 * both the I and Q paths of the AD917x device. It should be used when
 * there is a need to adjust the calibration tone amplitude for device
 * calibration purposes. The function requires a valid device handle and
 * a 16-bit amplitude value. It is important to ensure that the device
 * handle is properly initialized before calling this function. If the
 * handle is null, the function will return an error indicating an
 * invalid handle pointer.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. The
 * caller retains ownership and is responsible for ensuring it is a
 * valid, initialized handle.
 * @param amp A 16-bit unsigned integer representing the desired amplitude of
 * the DC calibration tone. The value is split across two registers,
 * with the lower 8 bits written first, followed by the upper 8 bits.
 * @return Returns an integer status code. API_ERROR_OK indicates success, while
 * other values indicate specific errors, such as
 * API_ERROR_INVALID_HANDLE_PTR for a null handle.
 ******************************************************************************/
int32_t ad917x_set_dc_cal_tone_amp(ad917x_handle_t *h, const uint16_t amp);

/***************************************************************************//**
 * @brief Use this function to obtain the phase offset values for the main
 * datapath and/or channel datapath NCOs of the AD917x device. It
 * requires a valid device handle and pointers to store the phase
 * offsets. The function can be used to query either DACs, channels, or
 * both, depending on the provided selection. Ensure that the device
 * handle is initialized and valid before calling this function. The
 * function will return an error if any of the input pointers are null or
 * if the handle is invalid.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null and
 * should be properly initialized.
 * @param dacs Specifies which DAC NCO phase offsets to retrieve. Can be
 * AD917X_DAC0, AD917X_DAC1, or both using a bitwise OR.
 * AD917X_DAC_NONE indicates no DAC selection.
 * @param dacs_po Pointer to a uint16_t where the DAC NCO phase offset will be
 * stored. Must not be null.
 * @param channels Specifies which channel NCO phase offsets to retrieve. Can be
 * a combination of AD917X_CH_0 to AD917X_CH_5 using a bitwise
 * OR. AD917X_CH_NONE indicates no channel selection.
 * @param ch_po Pointer to a uint16_t where the channel NCO phase offset will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating success or an error code. On success,
 * the phase offsets are stored in the provided pointers.
 ******************************************************************************/
int32_t ad917x_nco_get_phase_offset(ad917x_handle_t *h,
				    const ad917x_dac_select_t dacs, uint16_t *dacs_po,
				    const ad917x_channel_select_t channels, uint16_t *ch_po);

/***************************************************************************//**
 * @brief Use this function to configure the phase offset for the Numerically
 * Controlled Oscillator (NCO) in the main datapath and/or channel
 * datapath of the AD917x device. This function should be called after
 * initializing the device and selecting the appropriate DACs and
 * channels. It is important to ensure that the handle is valid and that
 * the DACs and channels specified are supported by the device. The
 * function will return an error if the handle is null or if there is an
 * issue with SPI communication.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null.
 * @param dacs Specifies the DAC(s) for which the phase offset is to be set.
 * Valid values are AD917X_DAC0, AD917X_DAC1, or a combination using
 * bitwise OR.
 * @param dacs_po The phase offset value for the selected DAC NCO(s). It is a
 * 16-bit value.
 * @param channels Specifies the channel(s) for which the phase offset is to be
 * set. Valid values are AD917X_CH_0 to AD917X_CH_5, or a
 * combination using bitwise OR.
 * @param ch_po The phase offset value for the selected channel NCO(s). It is a
 * 16-bit value.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., API_ERROR_INVALID_HANDLE_PTR for a null
 * handle).
 ******************************************************************************/
int32_t ad917x_nco_set_phase_offset(ad917x_handle_t *h,
				    const ad917x_dac_select_t dacs, const uint16_t dacs_po,
				    const ad917x_channel_select_t channels, const uint16_t ch_po);

/***************************************************************************//**
 * @brief This function configures the frequency tuning word (FTW) and
 * optionally the accumulator modulus and delta for the Numerically
 * Controlled Oscillator (NCO) in the AD917x device. It is used to set
 * the frequency and phase characteristics of the NCO. The function must
 * be called with a valid device handle and after the device has been
 * properly initialized. The DDS selection must be either the main or
 * channel DDS. If both accumulator modulus and delta are non-zero, they
 * will be configured; otherwise, they are ignored. The function returns
 * an error code if the handle is invalid, the DDS selection is
 * incorrect, or if there is a failure in the register read/write
 * operations.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. The
 * caller retains ownership.
 * @param dds Specifies the DDS to configure. Must be either AD917X_DDSM (main
 * DDS) or AD917X_DDSC (channel DDS). Invalid values result in an
 * error.
 * @param ftw The frequency tuning word to set. It is a 48-bit value.
 * @param acc_modulus The accumulator modulus value. If set to zero, it is
 * ignored.
 * @param acc_delta The accumulator delta value. If set to zero, it is ignored.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure.
 ******************************************************************************/
int32_t ad917x_nco_set_ftw(ad917x_handle_t *h,
			   const ad917x_dds_select_t nco,
			   const uint64_t ftw,
			   const uint64_t acc_modulus,
			   const uint64_t acc_delta);

/***************************************************************************//**
 * @brief This function is used to obtain the Frequency Tuning Word (FTW),
 * accumulator modulus, and delta values for a specified Numerically
 * Controlled Oscillator (NCO) in the AD917x device. It requires a valid
 * device handle and a DDS selection to specify which NCO to query. The
 * function must be called after the device has been properly
 * initialized. If the accumulator modulus and delta values are not
 * needed, their respective pointers can be set to NULL. The function
 * returns an error code if the handle is invalid, the DDS selection is
 * incorrect, or if any required pointer is NULL.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null.
 * Caller retains ownership.
 * @param dds Specifies the DDS to query. Must be either AD917X_DDSM or
 * AD917X_DDSC. Invalid values result in an error.
 * @param ftw Pointer to a uint64_t where the FTW value will be stored. Must not
 * be null.
 * @param acc_modulus Pointer to a uint64_t where the accumulator modulus will
 * be stored. Can be null if not needed.
 * @param acc_delta Pointer to a uint64_t where the accumulator delta will be
 * stored. Can be null if not needed.
 * @return Returns an int32_t error code. API_ERROR_OK on success, or an error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t ad917x_nco_get_ftw(ad917x_handle_t *h,
			   const ad917x_dds_select_t nco,
			   uint64_t *ftw,
			   uint64_t *acc_modulus,
			   uint64_t *acc_delta);

/***************************************************************************//**
 * @brief This function configures the Numerically Controlled Oscillator (NCO)
 * for the AD917x device to generate a specified carrier frequency and
 * amplitude. It can be used to set up the NCO for either DACs or
 * channels, enabling test tones and calibration as needed. The function
 * must be called with a valid device handle and appropriate frequency
 * and amplitude values. It is important to ensure that the carrier
 * frequency is within the valid range relative to the DAC frequency and
 * is not zero. The function handles both integer and modulus NCO modes
 * based on the frequency relationship to the DAC frequency.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null.
 * @param dacs Specifies which DAC NCO to configure. Can be AD917X_DAC0,
 * AD917X_DAC1, or both.
 * @param channels Specifies which channel NCO to configure. Can be a
 * combination of AD917X_CH_0 to AD917X_CH_5.
 * @param carrier_freq_hz Desired carrier frequency in Hz. Must be within the
 * range of -DAC frequency/2 to DAC frequency/2 and not
 * zero.
 * @param amplitude Desired amplitude value for the NCO output.
 * @param dc_test_tone_en Flag to enable or disable the DC test tone. Non-zero
 * to enable, zero to disable.
 * @param ddsm_cal_dc_input_en Flag to enable or disable the main datapath test
 * tone. Non-zero to enable, zero to disable.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure.
 ******************************************************************************/
int32_t ad917x_nco_set(ad917x_handle_t *h,
		       const ad917x_dac_select_t dacs,
		       const ad917x_channel_select_t channels,
		       const int64_t carrier_freq_hz,
		       const uint16_t amplitude,
		       int32_t dc_test_tone_en,
		       int32_t ddsm_cal_dc_input_en);

/***************************************************************************//**
 * @brief This function is used to enable or disable Numerically Controlled
 * Oscillators (NCOs) for specified DACs and channels on the AD917x
 * device. It should be called when you need to control which NCOs are
 * active, allowing for precise frequency control in digital signal
 * processing applications. The function disables all NCOs initially and
 * then enables only those specified by the parameters. It is important
 * to ensure that the device handle is properly initialized before
 * calling this function.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null and
 * should be properly initialized before use.
 * @param dacs Specifies which DAC NCOs to enable. Can be a combination of
 * AD917X_DAC0 and AD917X_DAC1, or AD917X_DAC_NONE to disable all.
 * @param channels Specifies which channel NCOs to enable. Can be a combination
 * of AD917X_CH_0 to AD917X_CH_5, or AD917X_CH_NONE to disable
 * all.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code indicating the type of failure (e.g., invalid handle or
 * parameter).
 ******************************************************************************/
int32_t ad917x_nco_enable(ad917x_handle_t *h,
			  const ad917x_dac_select_t dacs,
			  const ad917x_channel_select_t channels);

/***************************************************************************//**
 * @brief This function is used to enable or disable the main DAC calibration DC
 * input for the AD917x device. It should be called with a valid device
 * handle after the device has been initialized. The function modifies
 * the internal register settings to reflect the desired enable status.
 * It returns an error if the handle is invalid or if there is a failure
 * in reading or writing to the device registers.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. The
 * caller retains ownership.
 * @param ddsm_cal_dc_input_en Integer flag to enable or disable the calibration
 * DC input. Use 1 to enable and 0 to disable.
 * Invalid values are treated as 0 (disabled).
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code on failure (e.g., API_ERROR_INVALID_HANDLE_PTR for a null
 * handle).
 ******************************************************************************/
int32_t ad917x_ddsm_cal_dc_input_set(ad917x_handle_t *h,
				     int32_t ddsm_cal_dc_input_en);

/***************************************************************************//**
 * @brief This function checks whether the main DAC calibration DC input is
 * enabled or disabled. It should be called when you need to verify the
 * current status of the calibration DC input for the main DAC. Ensure
 * that the AD917X device has been properly initialized before calling
 * this function. The function will return an error if the provided
 * handle or output pointer is null.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param ddsm_cal_dc_input_en Pointer to an integer where the function will
 * store the result: 0 if disabled, 1 if enabled.
 * Must not be null. If null, the function returns
 * API_ERROR_INVALID_PARAM.
 * @return Returns API_ERROR_OK if successful, or an error code if the handle or
 * parameter is invalid.
 ******************************************************************************/
int32_t ad917x_ddsm_cal_dc_input_get(ad917x_handle_t *h,
				     int32_t *ddsm_cal_dc_input_en);

/***************************************************************************//**
 * @brief This function is used to enable or disable the DC test tone on the
 * AD917X device. It should be called when you need to configure the
 * device to output a DC test tone for testing or calibration purposes.
 * The function requires a valid device handle and an enable flag to
 * specify whether the test tone should be enabled or disabled. It
 * returns an error code if the handle is invalid or if there is an issue
 * with the SPI communication.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param dc_test_tone_en Integer flag to enable or disable the DC test tone. A
 * non-zero value enables the test tone, while zero
 * disables it.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, or other error
 * codes for SPI communication failures.
 ******************************************************************************/
int32_t ad917x_dc_test_tone_set(ad917x_handle_t *h, int32_t dc_test_tone_en);

/***************************************************************************//**
 * @brief Use this function to check whether the DC test tone is enabled on the
 * AD917X device. It requires a valid device handle and a pointer to an
 * integer where the status will be stored. The function must be called
 * after the device has been properly initialized. If the handle or the
 * pointer is null, the function will return an error. This function does
 * not modify the device state, only reads the current configuration.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. If
 * null, the function returns API_ERROR_INVALID_HANDLE_PTR.
 * @param dc_test_tone_en Pointer to an integer where the DC test tone enable
 * status will be stored. Must not be null. If null, the
 * function returns API_ERROR_INVALID_PARAM.
 * @return Returns API_ERROR_OK on success, or an error code if the handle or
 * parameter is invalid.
 ******************************************************************************/
int32_t ad917x_dc_test_tone_get(ad917x_handle_t *h, int32_t *dc_test_tone_en);

/***************************************************************************//**
 * @brief Use this function to obtain the frequency of a specific channel's
 * Numerically Controlled Oscillator (NCO) in Hertz. It is essential to
 * ensure that the device handle is valid and that the channel is one of
 * the supported channels (0 to 5). The function will store the frequency
 * in the provided pointer if the inputs are valid. This function should
 * be called after the device has been properly initialized.
 *
 * @param h Pointer to the AD917x device reference handle. Must not be null. If
 * null, the function returns an error.
 * @param channel Specifies the channel number for which the NCO frequency is to
 * be retrieved. Valid values are AD917X_CH_0 to AD917X_CH_5. If
 * an invalid channel is provided, the function returns an error.
 * @param carrier_freq_hz Pointer to a 64-bit integer where the frequency result
 * will be stored. Must not be null. If null, the
 * function returns an error.
 * @return Returns an integer status code: API_ERROR_OK on success, or an error
 * code if the handle is invalid, the channel is out of range, or the
 * frequency pointer is null.
 ******************************************************************************/
int32_t ad917x_nco_channel_freq_get(ad917x_handle_t *h,
				    ad917x_channel_select_t channel,
				    int64_t *carrier_freq_hz);

/***************************************************************************//**
 * @brief Use this function to obtain the frequency of the main DAC Numerically
 * Controlled Oscillator (NCO) for a specified DAC channel. It is
 * essential to ensure that the device handle is valid and that the DAC
 * selection is either AD917X_DAC0 or AD917X_DAC1. The function will
 * store the frequency in the provided pointer if the inputs are valid.
 * This function should be called after the device has been properly
 * initialized.
 *
 * @param h Pointer to the AD917X device reference handle. Must not be null. The
 * caller retains ownership.
 * @param dac Specifies the DAC channel for which the NCO frequency is to be
 * retrieved. Valid values are AD917X_DAC0 and AD917X_DAC1. Invalid
 * values result in an error.
 * @param carrier_freq_hz Pointer to a 64-bit integer where the frequency will
 * be stored. Must not be null.
 * @return Returns an integer status code: API_ERROR_OK on success,
 * API_ERROR_INVALID_HANDLE_PTR if the handle is null, and
 * API_ERROR_INVALID_PARAM for invalid DAC selection or null frequency
 * pointer.
 ******************************************************************************/
int32_t ad917x_nco_main_freq_get(ad917x_handle_t *h,
				 ad917x_dac_select_t dac,
				 int64_t *carrier_freq_hz);

/** @} */

#endif /* !__AD917XAPI_H__ */

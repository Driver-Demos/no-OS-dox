/***************************************************************************//**
 *   @file   altera_adxcvr.h
 *   @brief  Driver for the Altera ADXCVR Configuration.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef ALTERA_ADXCVR_H_
#define ALTERA_ADXCVR_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* XCVR Registers */

#define XCVR_REG_ARBITRATION				0x000
#define XCVR_ARBITRATION_MASK				0xFF
#define XCVR_ARBITRATION_GET_AVMM			0x02
#define XCVR_ARBITRATION_RELEASE_AVMM_CALIB	0x01
#define XCVR_ARBITRATION_RELEASE_AVMM		0x03

#define XCVR_REG_CALIB_ATX_PLL_EN		0x100
#define XCVR_CALIB_ATX_PLL_EN_MASK		0x01
#define XCVR_CALIB_ATX_PLL_EN			0x01

#define XCVR_REG_CAPAB_ATX_PLL_STAT		0x280
#define XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK	0x02
#define XCVR_CAPAB_ATX_PLL_CAL_DONE		0x00


#define XCVR_REG_CALIB_PMA_EN			0x100
#define XCVR_CALIB_TX_TERM_VOD_MASK		0x20
#define XCVR_CALIB_TX_TERM_VOD_EN		0x20
#define XCVR_CALIB_CMU_CDR_PLL_EN_MASK	0x02
#define XCVR_CALIB_CMU_CDR_PLL_EN		0x02

#define XCVR_REG_CAPAB_PMA				0x281
#define XCVR_CAPAB_RX_CAL_BUSY_EN_MASK	0x20
#define XCVR_CAPAB_RX_CAL_BUSY_EN		0x20
#define XCVR_CAPAB_RX_CAL_BUSY_DIS		0x00
#define XCVR_CAPAB_RX_CAL_BUSY_MASK		0x02
#define XCVR_CAPAB_RX_CAL_DONE			0x00
#define XCVR_CAPAB_TX_CAL_BUSY_EN_MASK	0x10
#define XCVR_CAPAB_TX_CAL_BUSY_EN		0x10
#define XCVR_CAPAB_TX_CAL_BUSY_DIS		0x00
#define XCVR_CAPAB_TX_CAL_BUSY_MASK		0x01
#define XCVR_CAPAB_TX_CAL_DONE			0x00

#define XCVR_REG_RATE_SWITCH_FLAG				0x166
#define XCVR_RATE_SWITCH_FLAG_MASK				0x80
#define XCVR_RATE_SWITCH_FLAG_RATE_SWITCH		0x00
#define XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH	0x80

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `adxcvr` structure is designed to encapsulate the configuration
 * and state of an Altera transceiver, including its name, base
 * addresses, operational mode (transmit or receive), and various rate
 * settings. It provides a comprehensive representation of the
 * transceiver's configuration, allowing for initialization, rate
 * adjustments, and calibration operations. The structure is integral to
 * managing the transceiver's functionality within a system, ensuring
 * proper setup and operation.
 *
 * @param name A constant character pointer to the name of the transceiver.
 * @param base A 32-bit unsigned integer representing the base address of the
 * transceiver.
 * @param is_transmit A boolean indicating if the transceiver is in transmit
 * mode.
 * @param lanes_per_link A 32-bit unsigned integer specifying the number of
 * lanes per link.
 * @param adxcfg_base An array of four 32-bit unsigned integers for the base
 * addresses of the ADX configuration.
 * @param atx_pll_base A 32-bit unsigned integer for the base address of the ATX
 * PLL.
 * @param reset_counter A 32-bit unsigned integer used as a counter for resets.
 * @param lane_rate_khz A 32-bit unsigned integer representing the lane rate in
 * kHz.
 * @param parent_rate_khz A 32-bit unsigned integer representing the parent rate
 * in kHz.
 * @param initial_recalc A boolean indicating if an initial recalculation is
 * needed.
 ******************************************************************************/
struct adxcvr {
	const char *name;
	uint32_t base;
	bool is_transmit;
	uint32_t lanes_per_link;
	uint32_t adxcfg_base[4];
	uint32_t atx_pll_base;
	uint32_t reset_counter;
	uint32_t lane_rate_khz;
	uint32_t parent_rate_khz;
	bool initial_recalc;
};

/***************************************************************************//**
 * @brief The `adxcvr_init` structure is used to initialize an ADXCVR (Altera
 * Transceiver) instance, providing essential configuration parameters
 * such as the name, base addresses for the transceiver and its
 * configuration, and the operational rates for the lanes and parent
 * clock. This structure is crucial for setting up the transceiver's
 * initial state before it is used in communication tasks.
 *
 * @param name A constant character pointer to the name of the ADXCVR instance.
 * @param base A 32-bit unsigned integer representing the base address of the
 * ADXCVR.
 * @param adxcfg_base An array of four 32-bit unsigned integers representing the
 * base addresses for ADXCVR configuration.
 * @param atx_pll_base A 32-bit unsigned integer representing the base address
 * of the ATX PLL.
 * @param lane_rate_khz A 32-bit unsigned integer representing the lane rate in
 * kilohertz.
 * @param parent_rate_khz A 32-bit unsigned integer representing the parent rate
 * in kilohertz.
 ******************************************************************************/
struct adxcvr_init {
	const char *name;
	uint32_t base;
	uint32_t adxcfg_base[4];
	uint32_t atx_pll_base;
	uint32_t lane_rate_khz;
	uint32_t parent_rate_khz;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
int32_t adxcvr_init(struct adxcvr **ad_xcvr,
		    const struct adxcvr_init *init);
int32_t adxcvr_remove(struct adxcvr *xcvr);
void adxcvr_post_lane_rate_change(struct adxcvr *xcvr,
				  unsigned int lane_rate_khz);
/***************************************************************************//**
 * @brief This function is used to determine the closest supported rate to a
 * desired rate for a given transceiver. It should be called when you
 * need to adjust the rate to one that is supported by the hardware. The
 * function distinguishes between transmit and receive modes based on the
 * transceiver's configuration and rounds the rate accordingly. It is
 * important to ensure that the transceiver structure is properly
 * initialized before calling this function.
 *
 * @param xcvr A pointer to an initialized 'struct adxcvr' representing the
 * transceiver. Must not be null.
 * @param rate_khz The desired rate in kilohertz to be rounded. Must be a
 * positive integer.
 * @return Returns the nearest supported rate in kilohertz as an int32_t. If the
 * input is invalid, the behavior is undefined.
 ******************************************************************************/
int32_t adxcvr_round_rate(struct adxcvr *xcvr, uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function configures the data rate of a given transceiver to the
 * specified rate in kilohertz. It should be used when there is a need to
 * change the operating data rate of the transceiver, either for
 * transmission or reception, depending on the transceiver's
 * configuration. The function must be called with a valid transceiver
 * structure that has been properly initialized. The behavior of the
 * function depends on whether the transceiver is set for transmission or
 * reception, as indicated by the `is_transmit` field in the `adxcvr`
 * structure.
 *
 * @param xcvr A pointer to a `struct adxcvr` representing the transceiver. This
 * structure must be initialized and must not be null. The function
 * uses the `is_transmit` field to determine whether to configure
 * the transceiver for transmission or reception.
 * @param rate_khz The desired data rate in kilohertz. This value should be
 * within the supported range of the transceiver. Invalid values
 * may result in an error return.
 * @return Returns an `int32_t` indicating success or failure. A non-zero return
 * value indicates an error occurred while setting the rate.
 ******************************************************************************/
int32_t adxcvr_set_rate(struct adxcvr *xcvr,
			uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function recalculates the data rate for a given transceiver
 * instance, determining whether to use the transmit or receive path
 * based on the transceiver's configuration. It should be called when the
 * data rate needs to be updated or verified, and it assumes that the
 * transceiver has been properly initialized. The function distinguishes
 * between transmit and receive modes using the `is_transmit` flag within
 * the transceiver structure.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * This must not be null and should point to a valid, initialized
 * transceiver instance. The function uses the `is_transmit` field
 * to determine the appropriate recalculation method.
 * @return Returns the recalculated data rate in kHz as a 32-bit unsigned
 * integer.
 ******************************************************************************/
uint32_t adxcvr_recalc_rate(struct adxcvr *xcvr);

/***************************************************************************//**
 * @brief This function is used to prepare the ADXCVR transceiver for a lane
 * rate change by asserting a reset signal. It should be called before
 * any lane rate change operation to ensure that the transceiver is in a
 * known state. The function manages multiple re-configuration requests
 * by maintaining a reset counter, ensuring that the reset remains
 * asserted until all requests are completed. This function must be
 * called with a valid `adxcvr` structure that has been properly
 * initialized.
 *
 * @param xcvr A pointer to a valid `adxcvr` structure representing the
 * transceiver. This parameter must not be null, and the structure
 * should be initialized before calling this function. The function
 * increments an internal reset counter within this structure.
 * @return None
 ******************************************************************************/
void adxcvr_pre_lane_rate_change(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief This function should be called after a lane rate change has been
 * initiated to finalize the configuration of the transceiver with the
 * new lane rate. It updates the lane rate of the transceiver and
 * performs any necessary finalization steps to ensure the transceiver
 * operates correctly at the new rate. This function assumes that the
 * transceiver has been properly initialized and that any preconditions
 * for changing the lane rate have been met.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * Must not be null, and the transceiver should be properly
 * initialized before calling this function.
 * @param lane_rate_khz An unsigned integer representing the new lane rate in
 * kilohertz. It should be a valid rate that the
 * transceiver can support.
 * @return None
 ******************************************************************************/
void adxcvr_post_lane_rate_change(struct adxcvr *xcvr,
				  unsigned int lane_rate);

/***************************************************************************//**
 * @brief This function is used to acquire arbitration for the ATX PLL
 * associated with a given transceiver. It is typically called when
 * exclusive access to the ATX PLL is required, such as before performing
 * configuration or calibration operations. The function must be called
 * with a valid transceiver structure that has been properly initialized.
 * It is important to ensure that arbitration is released after the
 * necessary operations are completed to avoid deadlocks or resource
 * contention.
 *
 * @param xcvr A pointer to an adxcvr structure representing the transceiver.
 * This structure must be initialized and must not be null. The
 * caller retains ownership of the structure.
 * @return None
 ******************************************************************************/
void atx_pll_acquire_arbitration(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief This function is used to release the arbitration lock on the ATX PLL
 * associated with the specified transceiver. It can optionally perform a
 * calibration during the release process. This function should be called
 * when the ATX PLL is no longer needed to be locked for exclusive
 * access, and the caller wants to ensure that the PLL is either left in
 * a calibrated state or not, depending on the `calibrate` parameter. It
 * is important to ensure that the transceiver structure is properly
 * initialized before calling this function.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * This must be a valid, non-null pointer, and the structure should
 * be properly initialized before use.
 * @param calibrate A boolean value indicating whether to perform calibration
 * upon releasing arbitration. If true, calibration is
 * performed; if false, it is not.
 * @return None
 ******************************************************************************/
void atx_pll_release_arbitration(struct adxcvr *xcvr,
				 bool calibrate);
/***************************************************************************//**
 * @brief Use this function to write a 32-bit value to a specific register of
 * the ATX PLL associated with the given transceiver. This function is
 * typically used when configuring or modifying the settings of the ATX
 * PLL. Ensure that the `xcvr` parameter is a valid pointer to an
 * initialized `adxcvr` structure before calling this function. The
 * function does not perform any error checking on the register address
 * or the value being written, so it is the caller's responsibility to
 * ensure that these are valid and appropriate for the intended
 * operation.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * Must not be null and should be properly initialized before use.
 * @param reg A 32-bit unsigned integer representing the register address within
 * the ATX PLL to which the value will be written. The caller must
 * ensure this is a valid register address.
 * @param val A 32-bit unsigned integer representing the value to be written to
 * the specified register. The caller must ensure this value is
 * appropriate for the register being accessed.
 * @return Returns 0 on successful execution. The function does not perform any
 * error checking and always returns 0.
 ******************************************************************************/
int32_t atx_pll_write(struct adxcvr *xcvr, uint32_t reg, uint32_t val);
/***************************************************************************//**
 * @brief This function retrieves the value from a specified register of the ATX
 * PLL associated with the given transceiver structure. It is typically
 * used when there is a need to access the current configuration or
 * status of the ATX PLL. The function requires a valid transceiver
 * structure and a register address to read from. The result is stored in
 * the provided output parameter. It is important to ensure that the
 * transceiver structure is properly initialized before calling this
 * function.
 *
 * @param xcvr A pointer to a valid 'struct adxcvr' instance representing the
 * transceiver. Must not be null.
 * @param reg The register address to read from, specified as a 32-bit unsigned
 * integer. The value should correspond to a valid register within
 * the ATX PLL.
 * @param val A pointer to a 32-bit unsigned integer where the read value will
 * be stored. Must not be null.
 * @return Returns 0 on success. The value read from the register is stored in
 * the location pointed to by 'val'.
 ******************************************************************************/
int32_t atx_pll_read(struct adxcvr *xcvr, uint32_t reg, uint32_t *val);
/***************************************************************************//**
 * @brief This function is used to modify a specific register within the ATX PLL
 * configuration of an ADXCVR device. It reads the current value of the
 * register, applies a mask to clear specific bits, and then sets those
 * bits to the new value provided. This function is typically used when
 * precise control over the PLL configuration is required, such as during
 * initialization or reconfiguration of the transceiver. It is important
 * to ensure that the `xcvr` parameter is a valid and initialized ADXCVR
 * structure before calling this function.
 *
 * @param xcvr A pointer to an initialized `adxcvr` structure representing the
 * transceiver. Must not be null.
 * @param reg The register address within the ATX PLL to be updated. Must be a
 * valid register address.
 * @param mask A bitmask indicating which bits in the register should be cleared
 * before setting the new value. Must be a valid bitmask for the
 * register.
 * @param val The new value to be set in the register after applying the mask.
 * Must be within the valid range for the register.
 * @return None
 ******************************************************************************/
void atx_pll_update(struct adxcvr *xcvr, uint32_t reg,
		    uint32_t mask, uint32_t val);
/***************************************************************************//**
 * @brief Use this function to verify whether the ATX PLL calibration has
 * completed successfully. It should be called after initiating a
 * calibration process to ensure that the PLL is properly calibrated. The
 * function waits for a maximum of 100 milliseconds, checking the
 * calibration status at 10-millisecond intervals. If the calibration is
 * successful within this period, the function returns 0; otherwise, it
 * returns 1, indicating a failure. This function is useful in scenarios
 * where reliable PLL calibration is critical for system stability.
 *
 * @param xcvr A pointer to an adxcvr structure representing the transceiver.
 * Must not be null. The caller retains ownership of this structure.
 * The function will read from this structure to determine the
 * calibration status.
 * @return Returns 0 if the ATX PLL calibration is successful within the timeout
 * period, or 1 if it fails.
 ******************************************************************************/
int32_t atx_pll_calibration_check(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief This function is used to perform the TX termination and Vod
 * calibration on all lanes of a given transceiver. It should be called
 * when the transceiver requires calibration to ensure proper
 * transmission characteristics. The function iterates over each lane,
 * acquiring and releasing arbitration as needed, and updates the
 * necessary registers to enable calibration. It checks for calibration
 * errors and returns a cumulative error status. This function assumes
 * that the transceiver structure is properly initialized and that the
 * number of lanes is correctly set in the `lanes_per_link` field.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * to be calibrated. Must not be null. The structure should be
 * properly initialized with valid configuration data, including the
 * number of lanes.
 * @return Returns an `int32_t` value representing the cumulative error status
 * of the calibration process across all lanes. A return value of 0
 * indicates success, while any non-zero value indicates an error
 * occurred during calibration.
 ******************************************************************************/
int32_t xcvr_calib_tx(struct adxcvr *xcvr);

/***************************************************************************//**
 * @brief This function is used to acquire arbitration for a specific lane
 * within the ADXCVR configuration, which is necessary for performing
 * certain operations that require exclusive access to the lane's
 * resources. It should be called when a lane-specific operation is about
 * to be performed, ensuring that no other operations interfere with the
 * process. The function must be called with a valid ADXCVR structure and
 * a valid lane index. It is important to ensure that the lane index is
 * within the range of configured lanes for the given ADXCVR instance.
 *
 * @param xcvr A pointer to an adxcvr structure representing the transceiver
 * configuration. Must not be null, and should be properly
 * initialized before calling this function. The caller retains
 * ownership.
 * @param lane An unsigned 32-bit integer representing the lane index for which
 * arbitration is to be acquired. Must be within the valid range of
 * lanes configured in the adxcvr structure.
 * @return None
 ******************************************************************************/
void adxcfg_acquire_arbitration(struct adxcvr *xcvr,
				uint32_t lane);
/***************************************************************************//**
 * @brief This function is used to release the arbitration for a specific lane
 * of the ADXCVR, optionally performing a calibration as part of the
 * release process. It should be called when the user needs to relinquish
 * control over a lane after completing necessary operations. The
 * function requires a valid ADXCVR structure and a lane index within the
 * valid range. The calibration parameter determines whether a
 * calibration is performed during the release. This function does not
 * return a value and assumes that the provided lane index is valid.
 *
 * @param xcvr A pointer to an adxcvr structure representing the ADXCVR
 * instance. Must not be null, and the structure should be properly
 * initialized before calling this function.
 * @param lane An unsigned 32-bit integer representing the lane index for which
 * arbitration is to be released. Must be within the valid range of
 * lanes for the given ADXCVR instance.
 * @param calibrate A boolean value indicating whether to perform calibration
 * during the release process. If true, calibration is
 * performed; if false, it is not.
 * @return None
 ******************************************************************************/
void adxcfg_release_arbitration(struct adxcvr *xcvr,
				uint32_t lane, bool calibrate);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to a specific register
 * of a specified lane within the ADXCVR configuration. It is typically
 * called when there is a need to configure or modify the settings of a
 * transceiver lane. The function requires a valid ADXCVR structure, a
 * lane index, a register address, and the value to be written. It is
 * important to ensure that the lane index is within the valid range
 * supported by the ADXCVR instance. The function does not perform any
 * error checking on the inputs and assumes that the caller provides
 * valid parameters.
 *
 * @param xcvr A pointer to an adxcvr structure representing the ADXCVR
 * instance. Must not be null, and should be properly initialized
 * before calling this function.
 * @param lane An unsigned 32-bit integer specifying the lane index. Must be
 * within the range of lanes supported by the ADXCVR instance.
 * @param reg An unsigned 32-bit integer representing the register address to
 * write to. The address is multiplied by 4 internally to calculate
 * the byte offset.
 * @param val An unsigned 32-bit integer representing the value to be written to
 * the specified register.
 * @return Returns 0 on successful execution. No error checking is performed, so
 * the return value does not indicate success or failure of the write
 * operation.
 ******************************************************************************/
int32_t adxcfg_write(struct adxcvr *xcvr, uint32_t lane, uint32_t reg,
		     uint32_t val);
/***************************************************************************//**
 * @brief This function reads a 32-bit value from a specified configuration
 * register of a given lane in an ADXCVR device. It is typically used to
 * retrieve configuration settings or status information from the device.
 * The function requires a valid ADXCVR device structure and a valid lane
 * index. The caller must ensure that the `val` pointer is not null, as
 * it will be used to store the read value. The function assumes that the
 * lane index and register address are within valid ranges for the
 * device.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the ADXCVR
 * device. Must not be null.
 * @param lane The index of the lane from which to read the register. Must be
 * within the valid range of lanes for the device.
 * @param reg The address of the register to read, specified as a 32-bit
 * unsigned integer. Must be a valid register address for the device.
 * @param val A pointer to a 32-bit unsigned integer where the read value will
 * be stored. Must not be null.
 * @return Returns 0 on success. The read value is stored in the location
 * pointed to by `val`.
 ******************************************************************************/
int32_t adxcfg_read(struct adxcvr *xcvr, uint32_t lane,
		    uint32_t reg, uint32_t *val);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register associated
 * with a particular lane in the ADXCVR configuration. It reads the
 * current value of the register, applies a mask to clear specific bits,
 * and then sets these bits to the provided value. This function is
 * typically used when precise control over register fields is required,
 * such as during configuration or calibration processes. It is important
 * to ensure that the `xcvr` structure is properly initialized before
 * calling this function.
 *
 * @param xcvr A pointer to an initialized `adxcvr` structure representing the
 * transceiver configuration. Must not be null.
 * @param lane The lane number for which the register update is to be performed.
 * Must be within the valid range of lanes for the given
 * transceiver.
 * @param reg The register address within the lane's configuration space to be
 * updated.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param val The new value to be written to the masked bits of the register.
 * Only the bits specified by the mask will be updated.
 * @return None
 ******************************************************************************/
void adxcfg_update(struct adxcvr *xcvr, uint32_t lane,
		   uint32_t reg, uint32_t mask, uint32_t val);
/***************************************************************************//**
 * @brief This function checks whether the calibration process for a specified
 * lane of a transceiver is complete. It can be used to verify either the
 * transmission (TX) or reception (RX) calibration status, depending on
 * the 'tx' parameter. The function waits for a maximum of 100
 * milliseconds for the calibration to complete, checking the status at
 * regular intervals. It is important to ensure that the transceiver
 * structure is properly initialized before calling this function. The
 * function returns immediately if the calibration is successful within
 * the timeout period, otherwise, it indicates a failure.
 *
 * @param xcvr A pointer to an initialized 'adxcvr' structure representing the
 * transceiver. Must not be null.
 * @param lane The lane number to check for calibration. Must be a valid lane
 * index for the given transceiver.
 * @param tx A boolean indicating whether to check the TX (true) or RX (false)
 * calibration status.
 * @return Returns 0 if the calibration is successful within the timeout period,
 * or 1 if it fails.
 ******************************************************************************/
int32_t adxcfg_calibration_check(struct adxcvr *xcvr, uint32_t lane,
				 bool tx);
#endif


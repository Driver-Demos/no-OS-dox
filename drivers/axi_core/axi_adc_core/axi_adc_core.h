/***************************************************************************//**
 *   @file   axi_adc_core.h
 *   @brief  Driver for the Analog Devices AXI-ADC-CORE module.
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
#ifndef AXI_ADC_CORE_H_
#define AXI_ADC_CORE_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AXI_ADC_REG_RSTN		0x0040
#define AXI_ADC_MMCM_RSTN		NO_OS_BIT(1)
#define AXI_ADC_RSTN			NO_OS_BIT(0)

#define AXI_ADC_REG_CNTRL		0x0044
#define AXI_ADC_R1_MODE			NO_OS_BIT(2)
#define AXI_ADC_DDR_EDGESEL		NO_OS_BIT(1)
#define AXI_ADC_PIN_MODE		NO_OS_BIT(0)

#define AXI_ADC_REG_CNTRL_3		0x004C
#define AXI_ADC_CRC_EN			NO_OS_BIT(8)

#define AXI_ADC_REG_CLK_FREQ		0x0054
#define AXI_ADC_CLK_FREQ(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_ADC_TO_CLK_FREQ(x)		(((x) >> 0) & 0xFFFFFFFF)

#define AXI_ADC_REG_CLK_RATIO		0x0058
#define AXI_ADC_CLK_RATIO(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_ADC_TO_CLK_RATIO(x)		(((x) >> 0) & 0xFFFFFFFF)

#define AXI_ADC_REG_STATUS		0x005C
#define AXI_ADC_MUX_PN_ERR		NO_OS_BIT(3)
#define AXI_ADC_MUX_PN_OOS		NO_OS_BIT(2)
#define AXI_ADC_MUX_OVER_RANGE		NO_OS_BIT(1)
#define AXI_ADC_STATUS			NO_OS_BIT(0)

#define AXI_ADC_REG_DELAY_CNTRL	0x0060
#define ADC_DELAY_SEL			NO_OS_BIT(17)
#define ADC_DELAY_RWN			NO_OS_BIT(16)
#define ADC_DELAY_ADDRESS(x) 		(((x) & 0xFF) << 8)
#define ADC_TO_DELAY_ADDRESS(x) 	(((x) >> 8) & 0xFF)
#define ADC_DELAY_WDATA(x)		(((x) & 0x1F) << 0)
#define ADC_TO_DELAY_WDATA(x)		(((x) >> 0) & 0x1F)

#define AXI_ADC_REG_CHAN_CNTRL(c)	(0x0400 + (c) * 0x40)
#define AXI_ADC_PN_SEL			NO_OS_BIT(10)
#define AXI_ADC_IQCOR_ENB		NO_OS_BIT(9)
#define AXI_ADC_DCFILT_ENB		NO_OS_BIT(8)
#define AXI_ADC_FORMAT_SIGNEXT		NO_OS_BIT(6)
#define AXI_ADC_FORMAT_TYPE		NO_OS_BIT(5)
#define AXI_ADC_FORMAT_ENABLE		NO_OS_BIT(4)
#define AXI_ADC_PN23_TYPE		NO_OS_BIT(1)
#define AXI_ADC_ENABLE			NO_OS_BIT(0)

#define AXI_ADC_REG_CHAN_STATUS(c)	(0x0404 + (c) * 0x40)
#define AXI_ADC_PN_ERR			NO_OS_BIT(2)
#define AXI_ADC_PN_OOS			NO_OS_BIT(1)
#define AXI_ADC_OVER_RANGE		NO_OS_BIT(0)

#define AXI_ADC_REG_CHAN_CNTRL_1(c)	(0x0410 + (c) * 0x40)
#define AXI_ADC_DCFILT_OFFSET(x)	(((x) & 0xFFFFL) << 16)
#define AXI_ADC_TO_DCFILT_OFFSET(x)	(((x) >> 16) & 0xFFFF)
#define AXI_ADC_DCFILT_COEFF(x)		(((x) & 0xFFFF) << 0)
#define AXI_ADC_TO_DCFILT_COEFF(x)	(((x) >> 0) & 0xFFFF)

#define AXI_ADC_REG_CHAN_CNTRL_2(c)	(0x0414 + (c) * 0x40)
#define AXI_ADC_IQCOR_COEFF_1(x)	(((x) & 0xFFFFL) << 16)
#define AXI_ADC_TO_IQCOR_COEFF_1(x)	(((x) >> 16) & 0xFFFF)
#define AXI_ADC_IQCOR_COEFF_2(x)	(((x) & 0xFFFF) << 0)
#define AXI_ADC_TO_IQCOR_COEFF_2(x)	(((x) >> 0) & 0xFFFF)

#define AXI_ADC_REG_CHAN_CNTRL_3(c)	(0x0418 + (c) * 0x40)
#define AXI_ADC_ADC_PN_SEL(x)		(((x) & 0xF) << 16)
#define AXI_ADC_TO_ADC_PN_SEL(x)	(((x) >> 16) & 0xF)
#define AXI_ADC_ADC_DATA_SEL(x)		(((x) & 0xF) << 0)
#define AXI_ADC_TO_ADC_DATA_SEL(x)	(((x) >> 0) & 0xF)

#define AXI_ADC_REG_DELAY(l)		(0x0800 + (l) * 0x4)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `axi_adc` structure is a descriptor for an AXI ADC device,
 * encapsulating essential configuration parameters such as device name,
 * base addresses, number of channels, clock frequency, and channel mask.
 * This structure is used to manage and interact with the ADC hardware,
 * providing a means to configure and control the ADC's operation within
 * a system.
 *
 * @param name A pointer to a string representing the device name.
 * @param base The base address of the ADC device.
 * @param slave_base The base address for the slave channels of the ADC.
 * @param num_channels The number of channels available in the ADC.
 * @param num_slave_channels The number of slave channels available in the ADC.
 * @param clock_hz The clock frequency of the ADC in hertz.
 * @param mask A bitmask representing the active channels of the ADC.
 ******************************************************************************/
struct axi_adc {
	/** Device Name */
	const char *name;
	/** Base Address */
	uint32_t base;
	/** Slave Base Address */
	uint32_t slave_base;
	/** Number of channels */
	uint8_t	num_channels;
	/** Number of slave channels */
	uint8_t	num_slave_channels;
	/** AXI ADC Clock */
	uint64_t clock_hz;
	/** AXI ADC Channel Mask*/
	uint32_t mask;
};

/***************************************************************************//**
 * @brief The `axi_adc_init` structure is used to define the initialization
 * parameters for an AXI ADC device. It includes information about the
 * device's name, base address, number of channels, and similar details
 * for any associated slave device. This structure is essential for
 * setting up the ADC device correctly before it can be used for data
 * acquisition.
 *
 * @param name A pointer to a string representing the device name.
 * @param base The base address of the ADC device.
 * @param num_channels The number of channels available in the ADC device.
 * @param slave_base The base address of the slave ADC device.
 * @param num_slave_channels The number of channels available in the slave ADC
 * device.
 ******************************************************************************/
struct axi_adc_init {
	/** Device Name */
	const char *name;
	/** Base Address */
	uint32_t base;
	/** Number of channels */
	uint8_t	num_channels;
	/** Slave Base Address */
	uint32_t slave_base;
	/** Number of slave channels */
	uint8_t	num_slave_channels;
};

/***************************************************************************//**
 * @brief The `axi_adc_pn_sel` enumeration defines a set of constants
 * representing different pseudo-random noise (PN) sequences used in the
 * Analog Devices AXI-ADC-CORE module. Each enumerator corresponds to a
 * specific PN sequence type, which is used for testing and calibration
 * purposes in ADC systems. The values assigned to each enumerator are
 * used to select the appropriate PN sequence in the ADC configuration.
 *
 * @param AXI_ADC_PN9 Represents the PN9 sequence with a value of 0.
 * @param AXI_ADC_PN23A Represents the PN23A sequence with a value of 1.
 * @param AXI_ADC_PN7 Represents the PN7 sequence with a value of 4.
 * @param AXI_ADC_PN15 Represents the PN15 sequence with a value of 5.
 * @param AXI_ADC_PN23 Represents the PN23 sequence with a value of 6.
 * @param AXI_ADC_PN31 Represents the PN31 sequence with a value of 7.
 * @param AXI_ADC_PN_CUSTOM Represents a custom PN sequence with a value of 9.
 * @param AXI_ADC_PN_RAMP_NIBBLE Represents a ramp nibble sequence with a value
 * of 10.
 * @param AXI_ADC_PN_RAMP_16 Represents a ramp 16 sequence with a value of 11.
 * @param AXI_ADC_PN_END Marks the end of the PN sequence enumeration with a
 * value of 12.
 ******************************************************************************/
enum axi_adc_pn_sel {
	AXI_ADC_PN9 = 0,
	AXI_ADC_PN23A = 1,
	AXI_ADC_PN7 = 4,
	AXI_ADC_PN15 = 5,
	AXI_ADC_PN23 = 6,
	AXI_ADC_PN31 = 7,
	AXI_ADC_PN_CUSTOM = 9,
	AXI_ADC_PN_RAMP_NIBBLE = 10,
	AXI_ADC_PN_RAMP_16 = 11,
	AXI_ADC_PN_END = 12,
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function starts the initialization process for an AXI ADC device
 * by allocating memory for the device descriptor and setting its initial
 * parameters based on the provided initialization structure. It should
 * be called before any other operations on the ADC device and must be
 * followed by a call to complete the initialization. The function
 * ensures that the device descriptor is properly set up with the
 * necessary base addresses and channel information.
 *
 * @param adc_core A pointer to a pointer of type `struct axi_adc`. This will be
 * set to point to the newly allocated and initialized ADC
 * device descriptor. Must not be null.
 * @param init A pointer to a `struct axi_adc_init` containing the
 * initialization parameters for the ADC device. Must not be null
 * and should be properly populated with valid base addresses and
 * channel information.
 * @return Returns 0 on successful initialization, or -1 if memory allocation
 * fails.
 ******************************************************************************/
int32_t axi_adc_init_begin(struct axi_adc **adc_core,
			   const struct axi_adc_init *init);
/***************************************************************************//**
 * @brief This function finalizes the initialization process of an AXI ADC
 * device by verifying its status and calculating its clock frequency. It
 * should be called after the initial setup of the ADC has been
 * performed. The function checks the status register to ensure there are
 * no errors before proceeding. If the status is valid, it reads the
 * clock frequency and ratio registers to compute the ADC's clock
 * frequency, which is then stored in the device descriptor. If the
 * status check fails, the function returns an error code.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * This must be a valid, non-null pointer to a properly initialized
 * `axi_adc` structure. The function will read from and write to this
 * structure.
 * @return Returns 0 on successful completion of initialization, or -1 if there
 * are status errors.
 ******************************************************************************/
int32_t axi_adc_init_finish(struct axi_adc *adc);
/** AXI ADC Main Initialization */
int32_t axi_adc_init(struct axi_adc **adc_core,
		     const struct axi_adc_init *init);
/***************************************************************************//**
 * @brief This function is used to free the resources allocated for an AXI ADC
 * instance. It should be called when the ADC instance is no longer
 * needed, to ensure that all associated memory is properly released.
 * This function must be called after the ADC instance has been
 * initialized and used, to prevent memory leaks. It is important to
 * ensure that the `adc` parameter is valid and points to a previously
 * initialized AXI ADC instance.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC instance
 * to be removed. Must not be null and should point to a valid,
 * initialized AXI ADC instance. Passing an invalid or null pointer
 * may result in undefined behavior.
 * @return Returns 0 to indicate successful deallocation of resources.
 ******************************************************************************/
int32_t axi_adc_remove(struct axi_adc *adc);
/***************************************************************************//**
 * @brief Use this function to read a 32-bit value from a specified register
 * address of an AXI ADC device. It is essential to ensure that the
 * `axi_adc` structure is properly initialized before calling this
 * function. The function does not perform any error checking on the
 * input parameters, so the caller must ensure that the register address
 * is valid and that the `reg_data` pointer is not null. This function is
 * typically used when direct access to the ADC's register values is
 * required for configuration or status checking.
 *
 * @param adc A pointer to an initialized `axi_adc` structure representing the
 * ADC device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the ADC device.
 * @param reg_data A pointer to a uint32_t variable where the read register
 * value will be stored. Must not be null.
 * @return Returns 0 on success. The value read from the register is stored in
 * the location pointed to by `reg_data`.
 ******************************************************************************/
int32_t axi_adc_read(struct axi_adc *adc,
		     uint32_t reg_addr,
		     uint32_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit data value to a specific
 * register address of an AXI ADC device. It is typically called when
 * there is a need to configure or control the ADC by setting specific
 * register values. The function requires a valid AXI ADC device
 * descriptor and assumes that the device has been properly initialized
 * before use. It does not perform any error checking on the input
 * parameters and always returns 0, indicating successful execution.
 *
 * @param adc A pointer to an `axi_adc` structure representing the AXI ADC
 * device. This must not be null and should point to a valid,
 * initialized device descriptor.
 * @param reg_addr A 32-bit unsigned integer specifying the register address
 * within the AXI ADC device where the data will be written. The
 * address should be within the valid range of the device's
 * register map.
 * @param reg_data A 32-bit unsigned integer representing the data to be written
 * to the specified register address.
 * @return Always returns 0, indicating successful execution.
 ******************************************************************************/
int32_t axi_adc_write(struct axi_adc *adc,
		      uint32_t reg_addr,
		      uint32_t reg_data);
/***************************************************************************//**
 * @brief This function configures the pseudo-random noise (PN) sequence for a
 * specific channel of an AXI ADC device. It is used to select the
 * desired PN sequence type for a given channel, which can be useful for
 * testing and calibration purposes. The function must be called with a
 * valid ADC device descriptor and a channel number that is within the
 * range of available channels on the device. The selected PN sequence
 * must be one of the predefined types in the `axi_adc_pn_sel`
 * enumeration. The function assumes that the ADC device has been
 * properly initialized before calling.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * Must not be null, and the device should be initialized before
 * calling this function.
 * @param chan The channel number for which the PN sequence is to be set. It
 * should be within the range of available channels on the ADC
 * device.
 * @param sel An enumeration value of type `axi_adc_pn_sel` representing the
 * desired PN sequence. Must be a valid value from the
 * `axi_adc_pn_sel` enum.
 * @return Returns 0 on success. No other return values are specified.
 ******************************************************************************/
int32_t axi_adc_set_pnsel(struct axi_adc *adc,
			  uint32_t chan,
			  enum axi_adc_pn_sel sel);
/***************************************************************************//**
 * @brief Use this function to monitor the Pseudo-Random Noise (PN) sequence of
 * an AXI ADC device to detect any errors. It enables the PN sequence
 * monitoring for all channels of the specified ADC, waits for a
 * specified delay, and then checks for any errors in the PN sequence.
 * This function should be called when you need to verify the integrity
 * of the PN sequence output by the ADC. It is important to ensure that
 * the ADC is properly initialized before calling this function. The
 * function returns an error code if any channel reports a PN sequence
 * error after the delay.
 *
 * @param adc A pointer to an initialized 'struct axi_adc' representing the ADC
 * device to be monitored. Must not be null.
 * @param sel An 'enum axi_adc_pn_sel' value specifying the PN sequence type to
 * monitor. Must be a valid enumeration value.
 * @param delay_ms A 'uint32_t' specifying the delay in milliseconds to wait
 * before checking for errors. Must be a non-negative value.
 * @return Returns 0 if no errors are detected in the PN sequence; returns -1 if
 * any channel reports a PN sequence error.
 ******************************************************************************/
int32_t axi_adc_pn_mon(struct axi_adc *adc,
		       enum axi_adc_pn_sel sel,
		       uint32_t delay_ms);
/***************************************************************************//**
 * @brief Use this function to obtain the current sampling frequency of a
 * specific channel in an AXI ADC device. This function is typically
 * called after the ADC has been initialized and configured. It
 * calculates the sampling frequency based on internal clock frequency
 * and ratio registers. Ensure that the `adc` pointer is valid and that
 * the `sampling_freq` pointer is not null to store the result. The
 * function does not perform any validation on the channel number, so it
 * is the caller's responsibility to ensure the channel is valid for the
 * given ADC device.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * Must not be null.
 * @param chan The channel number for which the sampling frequency is requested.
 * The caller must ensure this is a valid channel for the ADC
 * device.
 * @param sampling_freq A pointer to a `uint64_t` where the calculated sampling
 * frequency will be stored. Must not be null.
 * @return Returns 0 on success. The calculated sampling frequency is stored in
 * the location pointed to by `sampling_freq`.
 ******************************************************************************/
int32_t axi_adc_get_sampling_freq(struct axi_adc *adc,
				  uint32_t chan,
				  uint64_t *sampling_freq);
/***************************************************************************//**
 * @brief This function configures the input/output delay for a specified lane
 * of an AXI ADC device. It is typically used to adjust timing
 * characteristics for signal integrity or synchronization purposes. The
 * function must be called with a valid `axi_adc` device structure that
 * has been properly initialized. The `lane` parameter specifies which
 * interface line's delay is being set, and the `val` parameter
 * determines the delay value to be applied. This function does not
 * perform any validation on the input parameters, so it is the caller's
 * responsibility to ensure they are within valid ranges.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * Must not be null and should be initialized before calling this
 * function. The caller retains ownership.
 * @param lane An unsigned 32-bit integer specifying the interface line for
 * which the delay is being set. The valid range depends on the
 * specific ADC configuration.
 * @param val An unsigned 32-bit integer representing the delay value to be set.
 * The valid range is determined by the ADC's capabilities and
 * requirements.
 * @return None
 ******************************************************************************/
void axi_adc_idelay_set(struct axi_adc *adc,
			uint32_t lane,
			uint32_t val);
/***************************************************************************//**
 * @brief This function configures the input/output delay for a specified number
 * of lanes on an AXI ADC device. It should be used when there is a need
 * to adjust the timing of signals for multiple lanes simultaneously. The
 * function requires that the ADC core version is 10 or higher;
 * otherwise, it returns an error. It is important to ensure that the ADC
 * device is properly initialized before calling this function. The
 * function will attempt to set the delay for each lane and verify the
 * operation by reading back the delay value. If the read-back value does
 * not match the expected delay, a message is printed indicating the
 * discrepancy.
 *
 * @param adc A pointer to an initialized 'struct axi_adc' representing the ADC
 * device. Must not be null.
 * @param no_of_lanes The number of lanes for which the delay should be set.
 * Must be a positive integer.
 * @param delay The delay value to be set for each lane. The valid range depends
 * on the ADC hardware specifications.
 * @return Returns 0 on success, or -1 if the ADC core version is unsupported.
 ******************************************************************************/
int32_t axi_adc_delay_set(struct axi_adc *adc,
			  uint32_t no_of_lanes,
			  uint32_t delay);
/***************************************************************************//**
 * @brief This function is used to calibrate the delay settings of an AXI ADC
 * device by utilizing a specific pseudo-random noise (PN) sequence. It
 * should be called when precise delay calibration is required for the
 * ADC lanes to ensure accurate data capture. The function iterates over
 * possible delay values to find the optimal setting that minimizes
 * errors, and it sets the ADC to this delay. It is important to ensure
 * that the ADC device is properly initialized before calling this
 * function. If the calibration fails, the function returns an error
 * code.
 *
 * @param adc A pointer to an initialized 'struct axi_adc' representing the ADC
 * device. Must not be null.
 * @param no_of_lanes The number of lanes to calibrate. Must be a valid number
 * of lanes supported by the ADC.
 * @param sel An enumeration value of type 'enum axi_adc_pn_sel' specifying the
 * PN sequence to use for calibration. Must be a valid PN sequence
 * supported by the ADC.
 * @return Returns 0 on successful calibration, or -1 if the calibration fails.
 ******************************************************************************/
int32_t axi_adc_delay_calibrate(struct axi_adc *core,
				uint32_t no_of_lanes,
				enum axi_adc_pn_sel sel);
/***************************************************************************//**
 * @brief Use this function to set the phase calibration for a specific channel
 * of an AXI ADC device. It is typically called when you need to adjust
 * the phase alignment of the ADC channel to match specific requirements
 * or to compensate for known phase errors. Ensure that the ADC device is
 * properly initialized before calling this function. The function does
 * not perform any validation on the input values, so it is the caller's
 * responsibility to provide appropriate values for the calibration.
 *
 * @param adc A pointer to an initialized `axi_adc` structure representing the
 * ADC device. Must not be null.
 * @param chan The channel number for which the phase calibration is to be set.
 * It should be a valid channel index within the range supported by
 * the ADC device.
 * @param val An integer representing the primary phase calibration value. The
 * specific range and meaning depend on the ADC's calibration
 * requirements.
 * @param val2 An integer representing the secondary phase calibration value.
 * The specific range and meaning depend on the ADC's calibration
 * requirements.
 * @return Returns an `int32_t` status code indicating success or failure of the
 * operation. A non-zero value typically indicates an error.
 ******************************************************************************/
int32_t axi_adc_set_calib_phase(struct axi_adc *adc,
				uint32_t chan,
				int32_t val,
				int32_t val2);
/***************************************************************************//**
 * @brief Use this function to obtain the current phase calibration values for a
 * specific channel of an AXI ADC device. This function is typically
 * called after the ADC has been initialized and configured, and when
 * there is a need to verify or utilize the phase calibration settings.
 * It is important to ensure that the `adc` parameter is a valid,
 * initialized AXI ADC device structure before calling this function.
 *
 * @param adc A pointer to an initialized `struct axi_adc` representing the AXI
 * ADC device. Must not be null.
 * @param chan The channel number for which the phase calibration values are to
 * be retrieved. Must be a valid channel index for the given ADC
 * device.
 * @param val A pointer to an `int32_t` where the first phase calibration value
 * will be stored. Must not be null.
 * @param val2 A pointer to an `int32_t` where the second phase calibration
 * value will be stored. Must not be null.
 * @return Returns an `int32_t` status code indicating success or failure of the
 * operation. On success, `val` and `val2` are populated with the phase
 * calibration values.
 ******************************************************************************/
int32_t axi_adc_get_calib_phase(struct axi_adc *adc,
				uint32_t chan,
				int32_t *val,
				int32_t *val2);
/***************************************************************************//**
 * @brief This function is used to set the calibration scale for a specified
 * channel of an AXI ADC device. It should be called when you need to
 * adjust the scale calibration for a particular channel, typically
 * during the setup or tuning phase of using the ADC. The function
 * requires a valid AXI ADC device descriptor and a channel number within
 * the range supported by the device. The scale is defined by two integer
 * values, which must be provided by the caller. The function returns an
 * integer status code indicating success or failure of the operation.
 *
 * @param adc A pointer to an axi_adc structure representing the ADC device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param chan The channel number for which the scale calibration is to be set.
 * Must be within the valid range of channels supported by the ADC
 * device.
 * @param val An integer representing the first part of the scale calibration
 * value. The specific range and meaning depend on the ADC
 * configuration.
 * @param val2 An integer representing the second part of the scale calibration
 * value. The specific range and meaning depend on the ADC
 * configuration.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t axi_adc_set_calib_scale(struct axi_adc *adc,
				uint32_t chan,
				int32_t val,
				int32_t val2);
/***************************************************************************//**
 * @brief This function is used to obtain the scale calibration values for a
 * specific channel of an AXI ADC device. It is typically called after
 * the ADC has been initialized and configured, and when there is a need
 * to verify or utilize the current scale calibration settings for a
 * channel. The function requires valid pointers for the output
 * parameters to store the retrieved calibration values. It is important
 * to ensure that the `adc` parameter is a valid and initialized AXI ADC
 * device structure before calling this function.
 *
 * @param adc A pointer to an initialized `axi_adc` structure representing the
 * ADC device. Must not be null.
 * @param chan The channel number for which the scale calibration values are to
 * be retrieved. Must be a valid channel index for the given ADC
 * device.
 * @param val A pointer to an `int32_t` where the first part of the scale
 * calibration value will be stored. Must not be null.
 * @param val2 A pointer to an `int32_t` where the second part of the scale
 * calibration value will be stored. Must not be null.
 * @return Returns an `int32_t` status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t axi_adc_get_calib_scale(struct axi_adc *adc,
				uint32_t chan,
				int32_t *val,
				int32_t *val2);
/***************************************************************************//**
 * @brief This function is used to set the bias calibration for a specified
 * channel of an AXI ADC device. It should be called when there is a need
 * to adjust the DC offset for a particular channel to ensure accurate
 * signal processing. The function requires a valid AXI ADC device
 * descriptor and a channel number within the range of available
 * channels. The bias value is specified by the `val` parameter, while
 * `val2` is currently unused and should be set to zero. The function
 * assumes that the ADC device has been properly initialized before
 * calling this function.
 *
 * @param adc A pointer to an `axi_adc` structure representing the AXI ADC
 * device. Must not be null and should point to a valid, initialized
 * ADC device.
 * @param chan The channel number for which the bias calibration is to be set.
 * Must be within the range of available channels for the ADC
 * device.
 * @param val The bias value to be set for the specified channel. It is a 32-bit
 * integer, but only the lower 16 bits are used.
 * @param val2 An additional parameter for future use. Currently, it should be
 * set to zero.
 * @return Returns 0 on success, indicating that the bias calibration was set
 * successfully.
 ******************************************************************************/
int32_t axi_adc_set_calib_bias(struct axi_adc *adc,
			       uint32_t chan,
			       int32_t val,
			       int32_t val2);
/***************************************************************************//**
 * @brief Use this function to obtain the current bias calibration values for a
 * specific channel of an AXI ADC device. This function is typically
 * called after the ADC has been initialized and configured. It reads the
 * calibration offset for the specified channel and stores it in the
 * provided output parameter. Ensure that the `adc` pointer is valid and
 * that the channel number is within the range supported by the ADC
 * device. The function does not modify the second output parameter
 * `val2`, so it can be ignored or used for other purposes.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * Must not be null and should point to a properly initialized ADC
 * instance.
 * @param chan The channel number for which to retrieve the bias calibration.
 * Must be within the valid range of channels supported by the ADC
 * device.
 * @param val A pointer to an `int32_t` where the function will store the
 * retrieved bias calibration value. Must not be null.
 * @param val2 A pointer to an `int32_t` that is not used by this function. It
 * can be null or ignored.
 * @return Returns 0 on success. The bias calibration value is stored in the
 * location pointed to by `val`.
 ******************************************************************************/
int32_t axi_adc_get_calib_bias(struct axi_adc *adc,
			       uint32_t chan,
			       int32_t *val,
			       int32_t *val2);
/***************************************************************************//**
 * @brief Use this function to modify the active channels of an AXI ADC device
 * by providing a new channel mask. It should be called when there is a
 * need to change which channels are active, based on the provided mask.
 * The function updates the internal state of the ADC device to reflect
 * the new active channels. It is important to ensure that the `adc`
 * parameter is a valid, initialized AXI ADC device structure. The
 * function returns immediately if the provided mask is the same as the
 * current mask, avoiding unnecessary operations. If the mask differs, it
 * iterates over the channels, updating their control registers to enable
 * or disable them according to the mask. The function handles errors by
 * returning a non-zero value if any read or write operation fails.
 *
 * @param adc A pointer to an `axi_adc` structure representing the ADC device.
 * Must not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param mask A 32-bit unsigned integer representing the new channel mask. Each
 * bit corresponds to a channel, where a set bit indicates the
 * channel should be active. The function will update the ADC's
 * active channels based on this mask.
 * @return Returns 0 on success, or a negative error code if a read or write
 * operation fails.
 ******************************************************************************/
int32_t axi_adc_update_active_channels(struct axi_adc *adc, uint32_t mask);
#endif

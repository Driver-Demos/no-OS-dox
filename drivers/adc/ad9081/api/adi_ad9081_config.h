/*!
 * @brief     device configuration header
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup ADI_AD9081_INTERNAL_CONFIG
 * @{
 */
#ifndef __ADI_AD9081_CONFIG_H__
#define __ADI_AD9081_CONFIG_H__

/*============= D E F I N E S ==============*/

/*============= I N C L U D E S ============*/
#include "adi_ad9081.h"
#include "adi_ad9081_bf_impala_tc.h"
#include "adi_ad9081_bf_jrxa_des.h"
#include "adi_ad9081_bf_jtx_dual_link.h"
#include "adi_ad9081_bf_jtx_qbf_ad9081.h"
#include "adi_ad9081_bf_lcpll_28nm.h"
#include "adi_ad9081_bf_main.h"
#include "adi_ad9081_bf_nb_coarse_nco.h"
#include "adi_ad9081_bf_nb_ddc_dformat.h"
#include "adi_ad9081_bf_nb_fine_nco.h"
#include "adi_ad9081_bf_rx_paging.h"
#include "adi_ad9081_bf_ser_phy.h"
#include "adi_ad9081_bf_spi_only_up.h"
#include "adi_ad9081_bf_ad9081.h"

/*============= D E F I N E S ==============*/
#if (defined(__STDC_VERSION__) && __STDC_VERSION__ == 199901L)
#define __FUNCTION_NAME__ __func__
#else
#define __FUNCTION_NAME__ __FUNCTION__
#endif

#define AD9081_API_REV 0x00010500
#define AD9081_API_HW_RESET_LOW 600000
#define AD9081_API_RESET_WAIT 500000
#define AD9081_PLL_LOCK_TRY 75
#define AD9081_PLL_LOCK_WAIT 20000
#define AD9081_JESD_CAL_BOOT_WAIT 250000
#define AD9081_JESD_MAN_CAL_WAIT 200000
#define AD9081_JESD_RX_204C_CAL_WAIT 500000
#define AD9081_JESD_FG_CAL_WAIT 200000
#define AD9081_JESD_BG_CAL_WAIT 10000
#define AD9081_SERDES_RST_WAIT 50000
#define AD9081_DESER_MODE_204B_BR_TRESH 8000000000ULL
#define AD9081_DESER_MODE_204C_BR_TRESH 16000000000ULL
#define AD9081_IL_CTLE_UPPER_DB_THRESH 10

/* var error report */
#define AD9081_MSG_REPORT(var, comment)                                        \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_MSG, API_CMS_ERROR_OK, \
				    __FILE__, __FUNCTION_NAME__, __LINE__,     \
				    #var, comment)
#define AD9081_WARN_REPORT(var, comment)                                       \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_WARN,                  \
				    API_CMS_ERROR_OK, __FILE__,                \
				    __FUNCTION_NAME__, __LINE__, #var,         \
				    comment)
#define AD9081_ERROR_REPORT(error, var, comment)                               \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_ERR, error, __FILE__,  \
				    __FUNCTION_NAME__, __LINE__, #var,         \
				    comment)

/* log report */
#define AD9081_LOG_FUNC()                                                      \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_API, "%s(...)",           \
				 __FUNCTION_NAME__)
#define AD9081_LOG_SPIR(addr, data)                                            \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_SPI,                      \
				 "ad9081: r@%.4x = %.2x", addr, data)
#define AD9081_LOG_SPIW(addr, data)                                            \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_SPI,                      \
				 "ad9081: w@%.4x = %.2x", addr, data)
#define AD9081_LOG_SPIR32(addr, data)                                          \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_SPI,                      \
				 "ad9081: r32@%.4x = %.8x", addr, data)
#define AD9081_LOG_SPIW32(addr, data)                                          \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_SPI,                      \
				 "ad9081: w32@%.4x = %.8x", addr, data)
#define AD9081_LOG_VAR(type, msg, ...)                                         \
	adi_ad9081_hal_log_write(device, type, msg, ##__VA_ARGS__)
#define AD9081_LOG_MSG(msg)                                                    \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_MSG, msg)
#define AD9081_LOG_WARN(msg)                                                   \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_WARN, msg)
#define AD9081_LOG_ERR(msg)                                                    \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_ERR, msg)

/* var error check */
#define AD9081_ERROR_RETURN(r)                                                 \
	{                                                                      \
		if (r != API_CMS_ERROR_OK) {                                   \
			return r;                                              \
		}                                                              \
	}
#define AD9081_NULL_POINTER_RETURN(p)                                          \
	{                                                                      \
		if (p == NULL) {                                               \
			AD9081_ERROR_REPORT(API_CMS_ERROR_NULL_PARAM, p,       \
					    "Null pointer passed.");           \
			return API_CMS_ERROR_NULL_PARAM;                       \
		}                                                              \
	}
#define AD9081_INVALID_PARAM_RETURN(r)                                         \
	{                                                                      \
		if (r) {                                                       \
			AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, r,    \
					    "Invalid param passed.");          \
			return API_CMS_ERROR_INVALID_PARAM;                    \
		}                                                              \
	}
#define AD9081_INVALID_PARAM_WARN(r)                                           \
	{                                                                      \
		if (r) {                                                       \
			AD9081_WARN_REPORT(r, "Invalid param passed.");        \
		}                                                              \
	}

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to enable or disable the on-chip Phase-Locked
 * Loop (PLL) of the device. It should be called when configuring the
 * device's clock settings, specifically when switching between PLL-
 * generated clocks and reference clock sources. The `device` parameter
 * must point to a valid device handler structure, and the `pll_en`
 * parameter determines whether the PLL is enabled (1) or disabled (0).
 * If the function is called with a null `device` pointer, it will return
 * an error. Additionally, if any internal operations fail, the function
 * will return an appropriate error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param pll_en Enable signal for PLL. Valid values are 0 (disable PLL) and 1
 * (enable PLL).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, returns a failure
 * code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_pll_enable_set(adi_ad9081_device_t *device,
					     uint8_t pll_en);

/***************************************************************************//**
 * @brief This function is used to set up the on-chip PLL dividers, which
 * generate device clocks for DAC and ADC cores based on a provided
 * reference clock. It must be called after the device has been
 * initialized and before any clock-dependent operations. The function
 * performs several internal configurations and resets before applying
 * the specified divider settings. If any of the parameters are invalid,
 * the function will return an error code, and the device state will
 * remain unchanged.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param ref_div Input reference clock divider. Valid range is 1-3. If set to
 * 3, additional reset operations are performed.
 * @param m_div PLL M-Divider. Valid values are implementation-specific.
 * @param pll_div PLL output divider. Valid range is 1-3.
 * @param fb_div PLL Feedback divider. Valid range is 1-63.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_pll_div_set(adi_ad9081_device_t *device,
					  uint8_t ref_div, uint8_t m_div,
					  uint8_t pll_div, uint8_t fb_div);

/***************************************************************************//**
 * @brief This function is used to configure the Phase-Locked Loop (PLL)
 * settings for the device based on the desired DAC and ADC clock
 * frequencies, as well as the reference clock frequency. It must be
 * called after the device has been properly initialized and before any
 * operations that depend on the PLL being configured. The function will
 * attempt to find suitable divider settings for the PLL; if it fails to
 * do so, it will return an error. It is important to ensure that the
 * provided clock frequencies are within the specified valid ranges to
 * avoid configuration errors.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dac_clk_hz Desired DAC clock frequency in Hz. Valid range is 1.5 GHz
 * to 12 GHz.
 * @param adc_clk_hz Desired ADC clock frequency in Hz. Valid range is 2 GHz to
 * 6 GHz.
 * @param ref_clk_hz Reference clock frequency in Hz. Valid range is 100 MHz to
 * 2 GHz.
 * @return Returns API_CMS_ERROR_OK upon successful configuration of the PLL. If
 * the configuration fails, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_pll_startup(adi_ad9081_device_t *device,
					  uint64_t dac_clk_hz,
					  uint64_t adc_clk_hz,
					  uint64_t ref_clk_hz);

/***************************************************************************//**
 * @brief This function is used to set the up dividers for the device based on
 * the desired DAC clock frequency. It must be called after the device
 * has been properly initialized. The `dac_clk_hz` parameter specifies
 * the desired DAC clock frequency in Hertz, which must be within the
 * valid range of 1.5 GHz to 12 GHz. If the provided frequency is outside
 * this range, the function will handle it gracefully, ensuring that the
 * device's configuration remains stable.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dac_clk_hz Desired DAC Clock Frequency in Hz. Valid range is 1.5 GHz
 * to 12 GHz. The function will handle values outside this
 * range appropriately.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_device_clk_up_div_set(adi_ad9081_device_t *device,
					 uint64_t dac_clk_hz);

/***************************************************************************//**
 * @brief This function retrieves the laminate ID from the device's register and
 * stores it in the location pointed to by the provided pointer. It
 * should be called after the device has been properly initialized. Both
 * input parameters must not be null; otherwise, the function will return
 * an error code. The function is designed to handle cases where the
 * device or ID pointer is null by returning an appropriate error code.
 *
 * @param device Pointer to the device handler structure. Must not be null; if
 * null, the function returns an error code.
 * @param id Pointer to a location where the laminate ID will be stored. Must
 * not be null; if null, the function returns an error code.
 * @return Returns API_CMS_ERROR_OK upon success. If there is an error (e.g.,
 * null pointer), a failure code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_laminate_id_get(adi_ad9081_device_t *device,
					  uint8_t *id);

/***************************************************************************//**
 * @brief This function retrieves the die ID from the specified device and
 * stores it in the location pointed to by the `id` parameter. It should
 * be called after the device has been properly initialized. If either
 * the `device` or `id` pointers are null, the function will return an
 * error code without attempting to access the device. Ensure that the
 * `id` pointer points to a valid memory location capable of storing the
 * die ID.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param id Pointer to a location where the die ID will be stored. Must not be
 * null.
 * @return Returns API_CMS_ERROR_OK upon success. If an error occurs, a failure
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_die_id_get(adi_ad9081_device_t *device, uint8_t *id);

/***************************************************************************//**
 * @brief This function is used to verify the operational status of the power
 * supplies for the DAC, Clock, ADC0, and ADC1 components of the device.
 * It should be called after the device has been initialized to ensure
 * that all necessary power supplies are active. If any power supply is
 * found to be off, an error will be logged, and a failure code will be
 * returned. It is important to handle the return value appropriately to
 * ensure that the device is functioning correctly.
 *
 * @param device Pointer to the device handler structure. Must not be null;
 * otherwise, a null pointer error will be returned.
 * @return Returns API_CMS_ERROR_OK if all power supplies are on; otherwise, a
 * failure code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_power_status_check(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to verify the read and write capabilities of an
 * 8-bit register in the device. It should be called after the device has
 * been properly initialized. The function attempts to write specific
 * values to the register and then reads them back to ensure the values
 * match. If the readback does not match the written value, an error is
 * logged, and a failure code is returned. It is important to handle the
 * return value appropriately to check for success or failure.
 *
 * @param device Pointer to the device handler structure. Must not be null;
 * otherwise, a null pointer error will be reported and a failure
 * code will be returned.
 * @return Returns API_CMS_ERROR_OK upon success. If any read or write operation
 * fails, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_reg8_access_check(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to verify the read and write capabilities of a
 * specific 32-bit register in the device. It should be called after the
 * device has been properly initialized. The function attempts to write
 * specific test values to the register and then reads back the values to
 * ensure they match. If any readback does not match the expected value,
 * an error is logged, and a failure code is returned. It is important to
 * handle the return value appropriately to determine if the test was
 * successful or if there were issues with the register access.
 *
 * @param device Pointer to the device handler structure. Must not be null;
 * otherwise, a null pointer error will be reported and a failure
 * code returned.
 * @return Returns API_CMS_ERROR_OK upon success. If any read/write operation
 * fails, it returns a specific error code indicating the failure.
 ******************************************************************************/
int32_t adi_ad9081_device_reg32_access_check(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function should be called to ensure that the device has
 * successfully completed its pre-clock boot sequence. It checks the core
 * status to confirm that the boot process has reached the expected
 * state, and it verifies that the device revision is supported. If any
 * issues are detected during the boot process, appropriate error
 * messages are logged. It is important to call this function after the
 * device has been initialized and before any further operations are
 * performed. The function will return an error code if the device is not
 * properly configured or if the boot sequence fails.
 *
 * @param device Pointer to the device handler structure. Must not be null. If
 * null is passed, the function will return an error code
 * indicating a null parameter.
 * @return Returns API_CMS_ERROR_OK upon success. If any error occurs during the
 * boot verification process, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_boot_pre_clock(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function should be called to ensure that the device has completed
 * its boot process after the clock has been initialized. It checks the
 * device revision to determine if the boot loader's power-up sequence
 * can be bypassed, verifies that the boot process is complete, and
 * checks that the core is ready to run application code. Additionally,
 * it ensures that the clock switch has been completed and performs
 * necessary writes to enable the ADC SPI registers. It is important to
 * call this function after the device has been powered on and the clock
 * has been set up.
 *
 * @param device Pointer to the device handler structure. Must not be null; if
 * null, the function will return an error.
 * @return Returns API_CMS_ERROR_OK upon successful completion of the boot
 * process. If any checks fail or if an error occurs during the process,
 * a failure code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_device_boot_post_clock(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to configure the NCO synchronization mode of the
 * device, allowing it to operate in different modes: disabled, master,
 * or slave. It is essential to call this function after the device has
 * been properly initialized. The mode parameter must be set to one of
 * the predefined values: 0 to disable synchronization, 1 to set the
 * device as the master, or 2 to set it as a slave. Passing an invalid
 * mode value will result in an error being returned.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param mode Specifies the NCO synchronization mode. Valid values are 0
 * (disable), 1 (master), and 2 (slave). Passing any other value
 * will result in an error.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_mode_set(adi_ad9081_device_t *device,
					    uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the source that will trigger the
 * NCO synchronization in a master-slave setup. It should be called after
 * initializing the device and before starting any synchronization
 * processes. The `source` parameter determines the trigger source, which
 * can be set to use either the system reference (sysref) or the rising
 * or falling edges of the LMFC signal. If an invalid `source` value is
 * provided, the function will return an error code without modifying the
 * device state.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param source Specifies the trigger source for NCO synchronization. Valid
 * values are 0 (sysref), 1 (lmfc rising edge), and 2 (lmfc
 * falling edge). The function will return an error if an invalid
 * value is provided.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t
adi_ad9081_device_nco_sync_trigger_source_set(adi_ad9081_device_t *device,
					      uint8_t source);

/***************************************************************************//**
 * @brief This function configures the specified GPIO pin for NCO
 * synchronization by writing a value to the appropriate bitfield. It
 * should be called when the device is properly initialized and ready for
 * configuration. The `gpio_index` parameter determines which GPIO pin to
 * configure, and the `output` parameter specifies the value to write. If
 * the `gpio_index` is out of range (greater than 5) or if the `device`
 * pointer is null, the function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param gpio_index GPIO identifier, valid range is 0 to 5. If out of range,
 * the function will return an error.
 * @param output Value to write to the GPIO bitfield; if greater than 0, 10 is
 * written, otherwise 11 is written.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_gpio_set(adi_ad9081_device_t *device,
					    uint8_t gpio_index, uint8_t output);

/***************************************************************************//**
 * @brief This function is used to configure the number of extra LMFC cycles to
 * delay before issuing an NCO reset in Master-Slave synchronization
 * mode. It should only be called when the NCO synchronization mode is
 * set to master and the trigger source is not zero. Ensure that the
 * device is properly initialized before calling this function, as
 * passing a null pointer will result in an error. The function will
 * return an error code if the input value is invalid.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param num The number of extra LMFC cycles to set. Valid values are typically
 * within the range defined by the device specifications. The
 * function will return an error if the value is invalid.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t
adi_ad9081_device_nco_sync_extra_lmfc_num_set(adi_ad9081_device_t *device,
					      uint8_t num);

/***************************************************************************//**
 * @brief This function is used to configure the synchronization mode for the
 * NCOs (Numerically Controlled Oscillators) in the device, specifically
 * how they respond to the SYSREF signal. It should be called after the
 * device has been properly initialized. The `mode` parameter determines
 * the synchronization behavior: 0 for immediate synchronization upon
 * receiving SYSREF, 1 for synchronization at the next rising edge of the
 * LMFC (Local Multi-Frequency Clock), and 2 for synchronization at the
 * next falling edge of the LMFC. It is important to ensure that the
 * `device` pointer is valid and not null before calling this function,
 * as passing a null pointer will result in an error.
 *
 * @param device Pointer to the device handler structure. Must not be null; the
 * caller retains ownership.
 * @param mode Synchronization mode for the NCOs. Valid values are 0
 * (immediate), 1 (next LMFC rising edge), and 2 (next LMFC falling
 * edge). Passing an invalid value may result in an error.
 * @return Returns API_CMS_ERROR_OK upon success. If an error occurs, a failure
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_sysref_mode_set(adi_ad9081_device_t *device,
						   uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to control the reset state of the NCO
 * (Numerically Controlled Oscillator) in the device. It should be called
 * when you need to reset the NCO, which is typically done in
 * synchronization with the SYSREF signal. The `enable` parameter
 * determines whether to trigger the reset (when set to 1) or to clear
 * the reset state (when set to 0). It is important to ensure that the
 * `device` pointer is valid and not null before calling this function,
 * as passing a null pointer will result in an error.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param enable An integer value that indicates the reset state: 1 to trigger a
 * reset of the NCO, 0 to clear the reset state. Valid values are
 * 0 and 1.
 * @return Returns API_CMS_ERROR_OK upon success. If an error occurs, a failure
 * code is returned.
 ******************************************************************************/
int32_t
adi_ad9081_device_nco_sync_reset_via_sysref_set(adi_ad9081_device_t *device,
						uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to initiate the synchronization process for the
 * NCO in a master-slave configuration. It should be called when the
 * device is properly initialized and configured for NCO synchronization.
 * The function is self-clearing, meaning that the trigger will
 * automatically reset after it has been processed. It is important to
 * ensure that the `device` pointer is valid and not null before calling
 * this function.
 *
 * @param device Pointer to the device handler structure. Must not be null. If
 * the pointer is null, the function will return an error code
 * indicating a null parameter.
 * @return Returns API_CMS_ERROR_OK upon successful triggering of the
 * synchronization. If an error occurs, a failure code will be returned.
 ******************************************************************************/
int32_t adi_ad9081_device_nco_sync_trigger_set(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to enable or disable the digital logic
 * components of the device, which include the JESD digital, digital
 * clock generator, and digital data path. It should be called when
 * configuring the device's digital functionalities, and the `enable`
 * parameter determines whether these components are activated (1) or
 * deactivated (0). It is important to ensure that the `device` pointer
 * is valid and not null before calling this function, as passing a null
 * pointer will result in an error. The function will return a success
 * code if the operation is successful, or an error code if it fails.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param enable A flag to enable or disable digital logic. Valid values are 0
 * (disable) and 1 (enable).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code.
 ******************************************************************************/
int32_t adi_ad9081_device_digital_logic_enable_set(adi_ad9081_device_t *device,
						   uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the dual SPI mode for the device,
 * allowing access to specific control registers for DAC and ADC
 * channels. It should be called when the device is initialized and ready
 * for configuration. The `duals` parameter specifies which dual
 * configuration to enable, while the `enable` parameter determines
 * whether to enable or disable the dual SPI mode. It is important to
 * ensure that the `device` pointer is valid and not null before calling
 * this function, as passing a null pointer will result in an error.
 * Additionally, the function will return an error code if the operation
 * fails.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param duals Specifies which dual configuration to enable: 0x1 for dual0, 0x2
 * for dual1, and 0x3 for both. Valid values are 0x1, 0x2, or 0x3.
 * @param enable Determines whether to enable (1) or disable (0) the dual SPI
 * mode.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_d2a_dual_spi_enable_set(adi_ad9081_device_t *device,
					       uint8_t duals, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to initialize the specified Digital-to-Analog
 * Converters (DACs) by enabling various clock controls and
 * configurations necessary for their operation. It should be called
 * after the device has been properly initialized and configured. The
 * `dacs` parameter allows the caller to specify which DACs to start up
 * using a bitmask, where each bit corresponds to a specific DAC. If an
 * invalid pointer is passed for the `device` parameter, the function
 * will return an error code. Additionally, if any internal configuration
 * fails, the function will return an appropriate error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dacs Bitmask indicating which DACs to start up. Valid values are
 * 0b0000 to 0b1111, where each bit represents a DAC (0 for DAC 0, 1
 * for DAC 1, etc.).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, returns a failure
 * code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_dac_dll_startup(adi_ad9081_device_t *device, uint8_t dacs);

/***************************************************************************//**
 * @brief This function configures the frequency tuning word (FTW) for selected
 * digital-to-analog converters (DACs) and their respective channels. It
 * should be called after initializing the device and before starting any
 * DAC operations. The function expects valid DAC and channel selections,
 * and it will handle multiple DACs and channels as specified by the
 * input parameters. If any of the parameters are invalid or if the
 * device pointer is null, appropriate error codes will be returned.
 * Additionally, the function will log warnings if the FTW update does
 * not occur as expected.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dacs Bitmask indicating which DACs to configure. Each bit corresponds
 * to a DAC (0-3). Valid values are 0b0000 to 0b1111.
 * @param channels Bitmask indicating which channels to configure. Each bit
 * corresponds to a channel (0-7). Valid values are 0b00000000
 * to 0b11111111.
 * @param ftw The frequency tuning word, a 64-bit unsigned integer. Must be
 * within the valid range for the DACs being configured.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns an error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_dac_duc_nco_ftw0_set(adi_ad9081_device_t *device,
					uint8_t dacs, uint8_t channels,
					uint64_t ftw);

/***************************************************************************//**
 * @brief This function is used to control the soft off gain block for one or
 * more DACs in the device. It should be called when you want to enable
 * or disable the soft off gain functionality, which is necessary for
 * ramping the gain up or down smoothly. The `device` parameter must be a
 * valid pointer to an initialized device structure. The `dacs` parameter
 * specifies which DACs to affect using a bitmask, where each bit
 * corresponds to a DAC (0 for DAC 0, 1 for DAC 1, etc.). The `enable`
 * parameter determines whether to enable (1) or disable (0) the soft off
 * gain block. If an invalid DAC bitmask is provided, the function will
 * handle it gracefully without causing undefined behavior.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dacs Bitmask indicating which DACs to affect (0bXXXX). Each bit
 * corresponds to a DAC: Bit 0 for DAC 0, Bit 1 for DAC 1, etc.
 * Valid values are 0 to 0xF.
 * @param enable Flag to enable (1) or disable (0) the soft off gain block. Must
 * be either 0 or 1.
 * @return Returns API_CMS_ERROR_OK upon success. If an error occurs, a failure
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_soft_off_gain_enable_set(adi_ad9081_device_t *device,
						uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable the shuffling of the most
 * significant bit (MSB) and intermediate significant bit (ISB) segments
 * for the specified Digital-to-Analog Converters (DACs). It should be
 * called after the device has been properly initialized and configured.
 * The `dacs` parameter allows the user to specify which DACs to affect,
 * using a bitmask where each bit corresponds to a DAC. The `enable`
 * parameter determines whether shuffling is enabled (1) or disabled (0).
 * If an invalid value is provided for `dacs`, the function will handle
 * it gracefully without causing undefined behavior.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dacs Bitmask indicating which DACs to affect. Valid values are 0bXXXX,
 * where each bit corresponds to a DAC (Bit 3: DAC 3, Bit 2: DAC 2,
 * Bit 1: DAC 1, Bit 0: DAC 0).
 * @param enable Flag to enable (1) or disable (0) shuffling. Must be either 0
 * or 1.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_dac_shuffle_enable_set(adi_ad9081_device_t *device,
					  uint8_t dacs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the scrambling and de-scrambling of
 * DAC data for specified DACs. It should be called after the device has
 * been properly initialized. The `dacs` parameter allows the user to
 * specify which DACs to affect, using a bitmask where each bit
 * corresponds to a DAC. The `enable` parameter determines whether
 * scrambling and de-scrambling are enabled (1) or disabled (0). If an
 * invalid `device` pointer is provided, the function will return an
 * error code without making any changes.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dacs Bitmask indicating which DACs to affect, where each bit
 * corresponds to a DAC (0 for DAC 0, 1 for DAC 1, etc.). Valid
 * values are 0b0000 to 0b1111.
 * @param enable Flag to enable (1) or disable (0) scrambling and de-scrambling.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, returns a failure
 * code indicating the error.
 ******************************************************************************/
int32_t adi_ad9081_dac_data_xor_set(adi_ad9081_device_t *device, uint8_t dacs,
				    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to specify which ADCs will be affected by
 * subsequent configuration changes. It should be called when you need to
 * update settings for specific ADCs, and the `adcs` parameter allows you
 * to select multiple ADCs at once using a bitmask. The function expects
 * a valid pointer to a device handler structure, and it will return an
 * error if the pointer is null. If the `adcs` parameter is outside the
 * valid range, the function will handle it gracefully by returning an
 * error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param adcs Bitmask indicating which ADCs to select. Valid values are 0bXXXX,
 * where each bit corresponds to an ADC (Bit 3: ADC 3, Bit 2: ADC 2,
 * Bit 1: ADC 1, Bit 0: ADC 0).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_select_set(adi_ad9081_device_t *device, uint8_t adcs);

/***************************************************************************//**
 * @brief This function is used to enable or disable the analog registers for
 * specified ADC cores in the device. It should be called after the
 * device has been properly initialized. The `adc_cores` parameter allows
 * selection of which ADC cores to enable or disable, with valid values
 * being 0x01 for ADC0, 0x02 for ADC1, or 0x03 for both. The `enable`
 * parameter determines whether to enable (1) or disable (0) the selected
 * ADC cores. If an invalid pointer is passed for the `device` parameter,
 * the function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param adc_cores Specifies which ADC cores to enable or disable. Valid values
 * are 0x01 (ADC0), 0x02 (ADC1), or 0x03 (both).
 * @param enable Indicates whether to enable (1) or disable (0) the selected ADC
 * cores.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_core_analog_regs_enable_set(adi_ad9081_device_t *device,
						   uint8_t adc_cores,
						   uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to initialize and boot the specified ADC cores
 * of the device. It must be called after the device has been properly
 * initialized and configured. The `adc_cores` parameter determines which
 * ADC cores to boot, with valid values being 0x01 for ADC0, 0x02 for
 * ADC1, or 0x03 to boot both cores. If an invalid value is provided, the
 * function will return an error code. Additionally, if the device
 * pointer is null, it will also return an error. After calling this
 * function, it is important to check the core status to ensure that the
 * boot process has completed successfully.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param adc_cores Specifies which ADC cores to boot. Valid values are 0x01
 * (ADC0), 0x02 (ADC1), or 0x03 (both). Invalid values will
 * result in an error.
 * @return Returns API_CMS_ERROR_OK upon successful booting of the ADC cores, or
 * an error code if the operation fails.
 ******************************************************************************/
int32_t adi_ad9081_adc_core_setup(adi_ad9081_device_t *device,
				  uint8_t adc_cores);

/***************************************************************************//**
 * @brief This function is used to configure the analog input buffer settings
 * for the specified ADC cores in the device. It must be called after the
 * device has been initialized and before any data acquisition is
 * performed. The function takes into account the number of ADC cores
 * being used and the desired coupling type (AC or DC). If the parameters
 * are invalid, the function will return an error code. It is important
 * to ensure that the `device` pointer is not null before calling this
 * function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param adc_cores Specifies which ADC cores to configure. Valid values are
 * 0x01 for ADC0, 0x02 for ADC1, and 0x03 for both. Invalid
 * values will result in an error.
 * @param coupling Specifies the coupling type for the ADC input buffer. Valid
 * values are defined in the `adi_cms_signal_coupling_e`
 * enumeration, which includes AC_COUPLED and DC_COUPLED.
 * Invalid values will result in an error.
 * @return Returns API_CMS_ERROR_OK on success. On failure, it returns an error
 * code indicating the type of error encountered.
 ******************************************************************************/
int32_t
adi_ad9081_adc_analog_input_buffer_set(adi_ad9081_device_t *device,
				       uint8_t adc_cores,
				       adi_cms_signal_coupling_e coupling);

/***************************************************************************//**
 * @brief This function is used to power up specific ADC cores in the device by
 * booting them up. It should be called when the device is initialized
 * and ready for configuration. The `adcs` parameter allows the caller to
 * specify which ADCs to power up using a bitmask, where each bit
 * corresponds to an ADC core. The `enable` parameter indicates whether
 * to power up (1) or power down (0) the specified ADCs. If an invalid
 * value is provided for `adcs`, the function will handle it gracefully,
 * ensuring that only valid ADCs are affected.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param adcs Bitmask indicating which ADCs to power up. Valid values are
 * 0bXXXX, where each bit corresponds to an ADC (0 for ADC0, 1 for
 * ADC1, etc.).
 * @param enable Indicates whether to power up (1) or power down (0) the
 * specified ADCs. Must be either 0 or 1.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_power_up_set(adi_ad9081_device_t *device, uint8_t adcs,
				    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to decode the coarse decimation values for the
 * ADC's digital down converter (DDC). It should be called with a valid
 * `cddc_dcm` value representing the desired decimation configuration.
 * The function will return a corresponding decoded value based on the
 * input. If an invalid `cddc_dcm` value is provided, the function
 * defaults to returning a value of 1.
 *
 * @param cddc_dcm An enumerated value representing the coarse DDC decimation
 * configuration. Valid values are defined in the
 * `adi_ad9081_adc_coarse_ddc_dcm_e` enumeration. Must not be
 * null.
 * @return Returns a decoded decimation value as an 8-bit unsigned integer,
 * which corresponds to the input `cddc_dcm` value.
 ******************************************************************************/
uint8_t
adi_ad9081_adc_ddc_coarse_dcm_decode(adi_ad9081_adc_coarse_ddc_dcm_e cddc_dcm);

/***************************************************************************//**
 * @brief This function is used to decode the fine decimation values for the
 * ADC's digital down converter (DDC). It should be called with a valid
 * `fddc_dcm` value that corresponds to the desired decimation setting.
 * The function will return a decoded value based on the input, which can
 * be used for further configuration of the ADC. If an invalid `fddc_dcm`
 * value is provided, the function will return a default value of 1.
 *
 * @param fddc_dcm An enumerated value representing the fine DDC decimation
 * setting. Valid values include `AD9081_FDDC_DCM_2`,
 * `AD9081_FDDC_DCM_3`, `AD9081_FDDC_DCM_4`,
 * `AD9081_FDDC_DCM_6`, `AD9081_FDDC_DCM_8`,
 * `AD9081_FDDC_DCM_12`, `AD9081_FDDC_DCM_16`, and
 * `AD9081_FDDC_DCM_24`. The function expects a valid
 * enumeration; if an invalid value is passed, the function will
 * return a default decoded value of 1.
 * @return Returns a `uint8_t` value representing the decoded fine DDC
 * decimation value. The returned value will be one of the predefined
 * values corresponding to the input enumeration, or 1 if the input is
 * invalid.
 ******************************************************************************/
uint8_t
adi_ad9081_adc_ddc_fine_dcm_decode(adi_ad9081_adc_fine_ddc_dcm_e fddc_dcm);

/***************************************************************************//**
 * @brief This function is used to map specific GPIO pins to the programmable
 * filter and delay settings of the device. It should be called after the
 * device has been properly initialized. The `sel0` and `sel1` parameters
 * determine which GPIO pins are selected for the programmable filter
 * settings. If either `sel0` or `sel1` is outside the valid range, the
 * function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param sel0 Selects the GPIO for PERI_I_SEL21. Valid values are 0x02 to 0x08,
 * corresponding to specific GPIO pins. If an invalid value is
 * provided, the function will return an error.
 * @param sel1 Selects the GPIO for PERI_I_SEL22. Valid values are the same as
 * for sel0. If an invalid value is provided, the function will
 * return an error.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_adc_fdelay_cdelay_pfir_sel_to_gpio_mapping_set(
	adi_ad9081_device_t *device, uint8_t sel0, uint8_t sel1);

/***************************************************************************//**
 * @brief This function is used to enable or disable the common frequency
 * hopping feature for all coarse DDC NCOs in the ADCs. It should be
 * called when configuring the ADCs, particularly when frequency hopping
 * is desired for signal processing. The `device` parameter must point to
 * a valid device handler structure, and the `enable` parameter should be
 * set to 1 to enable frequency hopping or 0 to disable it. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param enable Enable signal for frequency hopping. Valid values are 0
 * (disable) and 1 (enable).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_common_hop_en_set(adi_ad9081_device_t *device,
					 uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the update mode for the coarse
 * digital down converters (DDCs) in the device. It allows the user to
 * specify which DDCs to affect and how the phase increment and phase
 * offset values are updated. The function must be called with a valid
 * `device` pointer, and the `cddcs` parameter should specify the DDCs to
 * be updated using a bitmask. The `mode` parameter determines whether
 * the updates are instantaneous or synchronized with a chip transfer
 * signal. If an invalid `device` pointer is provided, the function will
 * return an error.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to affect. Valid values are
 * 0bXXXX, where each bit corresponds to a DDC (0-3).
 * @param mode Update mode for the DDCs. Valid values are 0 for instantaneous
 * updates and 1 for synchronized updates.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_nco_channel_update_mode_set(
	adi_ad9081_device_t *device, uint8_t cddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the GPIO chip transfer mode for
 * specified coarse DDCs. It should be called when the
 * `ddc0_phase_update_mode` is set to '1'. The function allows the user
 * to specify which coarse DDCs to affect and the mode of operation for
 * updating phase increment and phase offset values. The mode can either
 * be synchronous updates when the chip transfer bit is set high or based
 * on a GPIO pin transition. It is important to ensure that the `device`
 * parameter is valid and not null before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to affect. Each bit
 * corresponds to a DDC (0-3). Valid values are 0b0000 to 0b1111.
 * @param mode Mode of operation for updating phase values. Valid values are 0
 * (synchronous update) or 1 (GPIO-based update).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_coarse_gpio_chip_xfer_mode_set(adi_ad9081_device_t *device,
						  uint8_t cddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to control the frequency hopping behavior of the
 * coarse digital down converters (DDCs) based on a trigger signal. It
 * should be called after the device has been properly initialized and
 * configured. The `cddcs` parameter allows the user to specify which
 * coarse DDCs to affect, while the `enable` parameter determines whether
 * the frequency hopping is dependent on the trigger signal (enabled) or
 * operates independently (disabled). It is important to ensure that the
 * `device` pointer is valid and not null before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to affect, where each bit
 * corresponds to a specific DDC (0 for DDC 0, 1 for DDC 1, etc.).
 * Valid values are from 0b0000 to 0b1111.
 * @param enable Flag to enable or disable trigger signal frequency hopping. Set
 * to 1 to enable (hopping based on trigger signal) or 0 to
 * disable (independent of trigger signal).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned if an error occurs.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_trig_hop_en_set(adi_ad9081_device_t *device,
						  uint8_t cddcs,
						  uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to configure the dithering settings for the
 * coarse digital down converters (DDCs) in the device. It should be
 * called after the device has been properly initialized and configured.
 * The `cddcs` parameter allows the user to specify which coarse DDCs to
 * affect, while the `amp_dither_en` and `phase_dither_en` parameters
 * control the enabling or disabling of amplitude and phase dithering,
 * respectively. It is important to ensure that the values provided for
 * `cddcs`, `amp_dither_en`, and `phase_dither_en` are valid, as invalid
 * values may lead to undefined behavior or failure in setting the
 * desired configuration.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to affect. Each bit
 * corresponds to a specific DDC (0-3). Valid values are 0b0000 to
 * 0b1111.
 * @param amp_dither_en Enable signal for amplitude dithering. 0 to enable, 1 to
 * disable.
 * @param phase_dither_en Enable signal for phase dithering. 0 to enable, 1 to
 * disable.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code
 * indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_dither_en_set(adi_ad9081_device_t *device,
						uint8_t cddcs,
						uint8_t amp_dither_en,
						uint8_t phase_dither_en);

/***************************************************************************//**
 * @brief This function is used to configure the Profile Select Word (PSW) for
 * specified coarse Digital Down Converters (DDCs). It should be called
 * when you need to set the rollover point for the Profile Select Timer,
 * which is crucial for managing channel selection in the device. The
 * function expects a valid `device` pointer and a valid `cddcs` value
 * that specifies which DDCs to affect. The `psw` parameter should be a
 * valid 64-bit value representing the Profile Select Word. If the
 * `device` pointer is null or if any of the parameters are invalid, the
 * function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param cddcs Bitmask indicating which coarse DDCs to affect. Valid values are
 * 0b0000 to 0b1111, where each bit corresponds to a DDC (0-3).
 * @param psw Profile Select Word, a 64-bit unsigned integer. Must be a valid
 * value.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_coarse_psw_set(adi_ad9081_device_t *device,
					  uint8_t cddcs, uint64_t psw);

/***************************************************************************//**
 * @brief This function is used to enable or disable the trigger NCO reset for
 * specified fine DDCs in the device. It should be called when
 * configuring the fine DDCs, particularly when synchronization or reset
 * behavior is required. The `device` parameter must point to a valid
 * device handler structure, and the `fddcs` parameter specifies which
 * fine DDCs to affect using a bitmask. The `enable` parameter determines
 * whether to enable (1) or disable (0) the trigger NCO reset. If the
 * `device` pointer is null, the function will return an error. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param fddcs Bitmask indicating which fine DDCs to affect (0bXXXXXXXX, where
 * each bit corresponds to a fine DDC). Valid values are from 0 to
 * 0xFF.
 * @param enable Flag to enable (1) or disable (0) the trigger NCO reset.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the error.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_trig_nco_reset_enable_set(
	adi_ad9081_device_t *device, uint8_t fddcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to set the update mode for fine NCO channels in
 * the specified device. It should be called after the device has been
 * properly initialized. The `fddcs` parameter allows selection of which
 * fine DDC channels to update, while the `mode` parameter determines how
 * the phase increment and phase offset values are updated. The function
 * handles multiple channels, and if an invalid `device` pointer is
 * provided, it will return an error. It is important to ensure that the
 * `fddcs` parameter correctly represents the channels intended for
 * modification.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param fddcs Bitmask indicating which fine DDC channels to update. Valid
 * values are 0 to 0xFF, where each bit represents a channel.
 * @param mode Update mode for the channels. Valid values are 0 (instantaneous
 * update) or 1 (synchronous update).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_nco_channel_update_mode_set(
	adi_ad9081_device_t *device, uint8_t fddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to configure the GPIO chip transfer mode for
 * specified fine digital down converters (DDCs). It should be called
 * after the device has been properly initialized. The `fddcs` parameter
 * allows selection of which fine DDCs to configure, while the `mode`
 * parameter determines the update behavior for phase increment and phase
 * offset values. If the `mode` is set to 0, updates occur synchronously
 * when the chip transfer bit is set high. If set to 1, updates occur
 * based on a GPIO pin transition from low to high. It is important to
 * ensure that the `device` pointer is valid and not null before calling
 * this function.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param fddcs Bitmask indicating which fine DDCs to configure. Each bit
 * corresponds to a fine DDC (0-7). Valid values are 0b00000000 to
 * 0b11111111.
 * @param mode Specifies the chip transfer mode. Valid values are 0 (synchronous
 * update) or 1 (GPIO-based update).
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t
adi_ad9081_adc_ddc_fine_gpio_chip_xfer_mode_set(adi_ad9081_device_t *device,
						uint8_t fddcs, uint8_t mode);

/***************************************************************************//**
 * @brief This function is used to control the frequency hopping behavior of
 * fine digital down converters (DDCs) in the device. It should be called
 * after the device has been properly initialized. The `fddcs` parameter
 * allows selection of which fine DDCs to configure, while the `enable`
 * parameter determines whether frequency hopping is enabled (1) or
 * disabled (0). If an invalid `fddcs` value is provided, the function
 * will not perform any operations on the DDCs.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param fddcs Bitmask indicating which fine DDCs to affect, where each bit
 * corresponds to a specific DDC (0-7). Valid values are 0b00000000
 * to 0b11111111.
 * @param enable Flag to enable (1) or disable (0) frequency hopping for the
 * selected fine DDCs.
 * @return Returns API_CMS_ERROR_OK upon success. If an error occurs, a failure
 * code is returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_trig_hop_en_set(adi_ad9081_device_t *device,
						uint8_t fddcs, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to enable or disable amplitude and phase
 * dithering for specific fine digital down converters (DDCs) in the
 * device. It should be called after the device has been properly
 * initialized and configured. The `fddcs` parameter allows selection of
 * which fine DDCs to affect, while `amp_dither_en` and `phase_dither_en`
 * control the enabling or disabling of amplitude and phase dithering,
 * respectively. If invalid values are provided for the dithering
 * parameters, the function will handle them gracefully, ensuring that
 * the device remains in a valid state.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param fddcs Bitmask indicating which fine DDCs to affect. Each bit
 * corresponds to a specific fine DDC (0-7). Valid values are 0x00
 * to 0xFF.
 * @param amp_dither_en Enable signal for amplitude dithering. 0 to enable, 1 to
 * disable.
 * @param phase_dither_en Enable signal for phase dithering. 0 to enable, 1 to
 * disable.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_adc_ddc_fine_dither_en_set(adi_ad9081_device_t *device,
					      uint8_t fddcs,
					      uint8_t amp_dither_en,
					      uint8_t phase_dither_en);

/***************************************************************************//**
 * @brief This function is used to configure the masking of unused channels in
 * the JESD interface of the device. It should be called when you need to
 * enable or disable specific converters for the selected links. The
 * `links` parameter allows you to specify which link(s) to configure,
 * while the `conv_index` parameter selects the specific converter to
 * mask. The `val` parameter determines whether to mask (1) or unmask (0)
 * the specified converter. It is important to ensure that the `device`
 * pointer is valid and not null, and that the `conv_index` is within the
 * valid range of 0 to 15.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param links Specifies the link(s) to set the mask for. Valid values are 0x1
 * for AD9081_LINK_0, 0x2 for AD9081_LINK_1, and 0x3 for both.
 * @param conv_index Index of the converter to set the mask for. Valid range is
 * 0 to 15.
 * @param val Value to set for the mask. 0 to unmask the converter, 1 to mask
 * it.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_conv_mask_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t conv_index, uint8_t val);

/***************************************************************************//**
 * @brief This function is used to configure the virtual converters for the
 * specified JESD links in the AD9081 device. It must be called after the
 * device has been properly initialized and before any data transmission
 * occurs. The function allows the user to specify which virtual
 * converters are active for each link, as well as the number of
 * converters in use. If the provided parameters are invalid, such as a
 * null device pointer, the function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param links Selects the JESD links to configure. Valid values are 0x1 for
 * AD9081_LINK_0, 0x2 for AD9081_LINK_1, or 0x3 for both.
 * @param jesd_conv_sel Array of virtual converter selection structures for each
 * link. Must have a size of at least 2.
 * @param jesd_m Array indicating the number of used virtual converters for each
 * link. Must have a size of at least 2.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, returns a failure
 * code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_link_conv_sel_set(
	adi_ad9081_device_t *device, adi_ad9081_jesd_link_select_e links,
	adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2], uint8_t jesd_m[2]);

/***************************************************************************//**
 * @brief This function retrieves the status of the Phase-Locked Loop (PLL) for
 * the specified device. It should be called after the device has been
 * properly initialized and configured. The function checks if the PLL is
 * locked and stores the result in the provided pointer. If the input
 * pointer is null, the function will return an error. It is important to
 * ensure that the device is ready and operational before calling this
 * function to avoid unexpected results.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param pll_locked Pointer to a location where the PLL lock status will be
 * stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK upon success, or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_pll_status_get(adi_ad9081_device_t *device,
					  uint8_t *pll_locked);

/***************************************************************************//**
 * @brief This function is used to initialize the JESD deserializers for the
 * specified device. It must be called after the device has been properly
 * initialized and configured. The `deser_mode` parameter determines the
 * operational mode of the deserializer, which can be full rate, half
 * rate, or quarter rate. It is important to ensure that the device
 * pointer is not null before calling this function, as passing a null
 * pointer will result in an error. The function may also return error
 * codes if the device is not ready or if the configuration settings are
 * invalid.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param deser_mode Specifies the deserializer mode of operation. Valid values
 * are 0 for full rate, 1 for half rate, and 2 for quarter
 * rate.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns a
 * failure code indicating the type of error encountered.
 ******************************************************************************/
int32_t adi_ad9081_jesd_rx_startup_des(adi_ad9081_device_t *device,
				       adi_ad9081_deser_mode_e deser_mode);

/***************************************************************************//**
 * @brief This function is used to identify an available DFormat output that can
 * be assigned to virtual converters that are not currently in use. It
 * should be called after the virtual converters have been configured,
 * and it will return a bitmask indicating which DFormat outputs are
 * available. The function takes into account the number of virtual
 * converters in use, specified by the `jesd_m` parameter, and will only
 * consider outputs that are not already assigned. If all outputs are in
 * use, the function will return a bitmask indicating the first available
 * output.
 *
 * @param jesd_conv_sel Pointer to a structure that contains the indices of the
 * virtual converters. This structure must not be null and
 * should point to valid memory containing the indices of
 * the virtual converters.
 * @param jesd_m An 8-bit unsigned integer representing the number of virtual
 * converters currently in use. Valid values range from 0 to 16,
 * inclusive. If this value exceeds the number of available
 * virtual converters, the behavior is undefined.
 * @return Returns a 16-bit unsigned integer bitmask representing the unused
 * DFormat outputs. Each bit in the mask corresponds to a DFormat
 * output, where a set bit indicates that the output is available for
 * assignment.
 ******************************************************************************/
uint16_t adi_ad9081_jesd_find_dformat_out_nc(
	adi_ad9081_jtx_conv_sel_t const *jesd_conv_sel, uint8_t jesd_m);

/***************************************************************************//**
 * @brief This function is used to determine the highest common unused DFormat
 * output for the specified JESD links, which is essential when
 * configuring virtual converters in a JESD interface. It should be
 * called after the virtual converters have been selected and the number
 * of used converters has been defined. The function takes into account
 * the active DFormat outputs for each link and returns the highest
 * unused output. If both links are inactive, the function will return a
 * default value. Ensure that the input parameters are valid to avoid
 * unexpected behavior.
 *
 * @param links Specifies which JESD links to consider for determining the
 * unused DFormat output. Valid values are 0x1 for AD9081_LINK_0,
 * 0x2 for AD9081_LINK_1, and 0x3 for both links.
 * @param jesd_conv_sel An array of two `adi_ad9081_jtx_conv_sel_t` structures
 * that represent the virtual converter selections for each
 * link. The caller retains ownership of this array.
 * @param jesd_m An array of two `uint8_t` values indicating the number of used
 * virtual converters for each link. The values must be non-
 * negative and should not exceed the maximum number of virtual
 * converters supported.
 * @return Returns the highest common unused DFormat output as a `uint8_t`. If
 * no unused output is found, a default value is returned.
 ******************************************************************************/
uint8_t
adi_ad9081_jesd_determine_common_nc(adi_ad9081_jesd_link_select_e links,
				    adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2],
				    uint8_t jesd_m[2]);

/***************************************************************************//**
 * @brief This function is used to enable or disable the D2A center
 * functionality in the device. It should be called when configuring the
 * device's digital logic settings. The `device` parameter must point to
 * a valid device structure, and the `enable` parameter should be set to
 * 1 to enable or 0 to disable the D2A center functionality. If the
 * `device` pointer is null, the function will return an error code. It
 * is important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the device structure. Must not be null.
 * @param enable An integer value that indicates whether to enable (1) or
 * disable (0) the D2A center functionality. Valid values are 0
 * and 1.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, a failure code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9081_jesd_sysref_d2acenter_enable_set(adi_ad9081_device_t *device,
						    uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to control the hardware synchronization
 * reference (SYSREF) for the device. It should be called when the device
 * is properly initialized and ready for synchronization operations. The
 * function checks for null pointers and ensures that the necessary clock
 * information is available before proceeding. If any of the required
 * pointers are null, an error is returned. It is important to handle the
 * return value appropriately to ensure that the synchronization control
 * is successful.
 *
 * @param device Pointer to the device handler structure. Must not be null. If
 * the pointer is null, the function will return an error code
 * indicating a null parameter.
 * @return Returns API_CMS_ERROR_OK upon success. If there is an error during
 * the operation, an appropriate failure code is returned.
 ******************************************************************************/
int32_t adi_ad9081_sync_sysref_ctrl(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to compute the JTX BR log2 ratio based on the
 * specified chip operating mode and JESD parameters. It should be called
 * after initializing the device and before starting data transmission.
 * The function handles both TX only and RX only modes, adjusting the
 * output values based on the provided lane rates. If the input
 * parameters are invalid, such as null pointers or out-of-range values,
 * the function will return an error code.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param chip_op_mode Specifies the chip operating mode, which can be TX_ONLY,
 * RX_ONLY, or TX_RX_ONLY. Valid values are defined in the
 * `adi_cms_chip_op_mode_t` enumeration.
 * @param jesd_param Pointer to a structure containing JESD parameters. Must not
 * be null.
 * @param links Specifies which AD9081 links to set, with valid values being
 * AD9081_LINK_0, AD9081_LINK_1, or both.
 * @param jtx_lane_rate Array of two 64-bit integers representing the lane rates
 * in bps for the specified links. Must contain valid lane
 * rates.
 * @param jtx_brr Array of two 8-bit integers where the calculated JTX BR log2
 * ratio values will be stored. The caller retains ownership.
 * @return Returns API_CMS_ERROR_OK upon success. Otherwise, it returns an error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_jesd_tx_calc_br_ratio(adi_ad9081_device_t *device,
					 adi_cms_chip_op_mode_t chip_op_mode,
					 adi_cms_jesd_param_t *jesd_param,
					 adi_ad9081_jesd_link_select_e links,
					 uint64_t jtx_lane_rate[2],
					 uint8_t jtx_brr[2]);

#if AD9081_USE_FLOATING_TYPE > 0
/***************************************************************************//**
 * @brief This function is used to compute the frequency tuning word (FTW) for
 * the NCO based on the desired frequency and NCO shift. It should be
 * called with a valid `device` pointer that has been properly
 * initialized. The `freq` parameter must be non-zero, as a zero
 * frequency is considered invalid and will result in an error. The
 * function also computes two additional values, `a` and `b`, which are
 * used in conjunction with the FTW. The function handles both positive
 * and negative NCO shifts, adjusting the FTW and the values of `a` and
 * `b` accordingly. If any input parameters are invalid, appropriate
 * error codes will be returned.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param freq Desired frequency in Hz. Must be non-zero.
 * @param nco_shift NCO shift value in Hz. Can be positive or negative.
 * @param ftw Pointer to a uint64_t where the calculated frequency tuning word
 * will be stored. Caller retains ownership.
 * @param a Pointer to a uint64_t where the calculated value 'a' will be stored.
 * Caller retains ownership.
 * @param b Pointer to a uint64_t where the calculated value 'b' will be stored.
 * Caller retains ownership.
 * @return Returns API_CMS_ERROR_OK upon success. If any input parameters are
 * invalid, an appropriate error code is returned.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_nco_ftw_f(adi_ad9081_device_t *device, double freq,
				      double nco_shift, uint64_t *ftw,
				      uint64_t *a, uint64_t *b);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ADI_AD9081_CONFIG_H__ */

/*! @} */
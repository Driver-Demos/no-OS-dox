/*!
 * @brief     device configuration header
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup ADI_AD9083_CONFIG_H
 * @{
 */
#ifndef __ADI_AD9083_CONFIG_H__
#define __ADI_AD9083_CONFIG_H__

/*============= I N C L U D E S ============*/
#include "adi_ad9083.h"
#include "adi_ad9083_bf_datapath.h"
#include "adi_ad9083_bf_dig_scan.h"
#include "adi_ad9083_bf_eng.h"
#include "adi_ad9083_bf_g_h_sync.h"
#include "adi_ad9083_bf_jtx_ip.h"
#include "adi_ad9083_bf_jtx_qbf.h"
#include "adi_ad9083_bf_lcpll_28nm_r1.h"
#include "adi_ad9083_bf_ser_phy.h"
#include "adi_ad9083_bf_spi.h"
#include "adi_ad9083_bf_top_clk_reg.h"
#include "adi_ad9083_bf_top_ref_reg.h"
#include "adi_ad9083_bf_vcoadc_lut.h"
#include "adi_ad9083_bf_vcoadc_mem_map.h"

/*============= D E F I N E S ==============*/
#if (defined(__STDC_VERSION__) && __STDC_VERSION__ == 199901L)
#define __FUNCTION_NAME__ __func__
#else
#define __FUNCTION_NAME__ __FUNCTION__
#endif

/* error report */
#define AD9083_MSG_REPORT(var, comment)                                        \
  adi_ad9083_hal_error_report(device, ADI_CMS_LOG_MSG, API_CMS_ERROR_OK,       \
                              __FILE__, __FUNCTION_NAME__, __LINE__, #var,     \
                              comment)
#define AD9083_WARN_REPORT(var, comment)                                       \
  adi_ad9083_hal_error_report(device, ADI_CMS_LOG_WARN, API_CMS_ERROR_OK,      \
                              __FILE__, __FUNCTION_NAME__, __LINE__, #var,     \
                              comment)
#define AD9083_ERROR_REPORT(error, var, comment)                               \
  adi_ad9083_hal_error_report(device, ADI_CMS_LOG_ERR, error, __FILE__,        \
                              __FUNCTION_NAME__, __LINE__, #var, comment)

/* log report */
#define AD9083_LOG_FUNC()                                                      \
  adi_ad9083_hal_log_write(device, ADI_CMS_LOG_MSG, "%s(...)",                 \
                           __FUNCTION_NAME__)
#define AD9083_LOG_VAR(type, msg, ...)                                         \
  adi_ad9083_hal_log_write(device, type, msg, ##__VAR_ARGS__)
#define AD9083_LOG_MSG(msg)                                                    \
  adi_ad9083_hal_log_write(device, ADI_CMS_LOG_MSG, msg)

/* error check */
#define AD9083_ERROR_RETURN(r)                                                 \
  {                                                                            \
    if (r != API_CMS_ERROR_OK) {                                               \
      return r;                                                                \
    }                                                                          \
  }
#define AD9083_NULL_POINTER_RETURN(p)                                          \
  {                                                                            \
    if (p == NULL) {                                                           \
      AD9083_ERROR_REPORT(API_CMS_ERROR_NULL_PARAM, p,                         \
                          "Null pointer passed.");                             \
      return API_CMS_ERROR_NULL_PARAM;                                         \
    }                                                                          \
  }
#define AD9083_INVALID_PARAM_RETURN(r)                                         \
  {                                                                            \
    if (r) {                                                                   \
      AD9083_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, r,                      \
                          "Invalid param passed.");                            \
      return API_CMS_ERROR_INVALID_PARAM;                                      \
    }                                                                          \
  }
#define AD9083_INVALID_PARAM_WARN(r)                                           \
  {                                                                            \
    if (r) {                                                                   \
      AD9083_WARN_REPORT(r, "Invalid param passed.");                          \
    }                                                                          \
  }

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function verifies the ability to read from and write to an 8-bit
 * register on the specified device. It is used to ensure that the
 * device's register access mechanism is functioning correctly. The
 * function must be called with a valid device pointer, and it performs a
 * series of read and write operations to test the register access. If
 * the test fails, an error code is returned. This function is typically
 * used during device initialization or diagnostics to confirm proper
 * communication with the device.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device to be tested. Must not be null. If a null pointer is
 * passed, the function returns an error code indicating a null
 * parameter.
 * @return Returns an integer status code. API_CMS_ERROR_OK is returned if the
 * register access test passes successfully. If the test fails or if a
 * null device pointer is provided, an appropriate error code is
 * returned.
 ******************************************************************************/
int32_t adi_ad9083_device_reg8_access_check(adi_ad9083_device_t *device);
/***************************************************************************//**
 * @brief This function verifies whether all necessary power supplies for the
 * AD9083 device are active. It should be called to ensure that the
 * device is properly powered before performing operations that require
 * full functionality. The function logs an error message if any power
 * supply is not on, which can be useful for debugging power-related
 * issues. It is important to ensure that the `device` parameter is a
 * valid, non-null pointer to an initialized `adi_ad9083_device_t`
 * structure before calling this function.
 *
 * @param device A pointer to an `adi_ad9083_device_t` structure representing
 * the device to check. Must not be null. The caller retains
 * ownership of the memory. If the pointer is null, the function
 * returns an error code indicating a null parameter.
 * @return Returns an integer status code. Returns `API_CMS_ERROR_OK` if all
 * power supplies are on, or an error code if any power supply is off or
 * if a null pointer is passed.
 ******************************************************************************/
int32_t adi_ad9083_device_power_status_check(adi_ad9083_device_t *device);

#ifdef __cplusplus
}
#endif

#endif /* __ADI_AD9083_CONFIG_H__ */

/*! @} */
/***************************************************************************//**
 *   @file   ades1754.h
 *   @brief  Header file for the ADES1754 Driver
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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
#ifndef __ADES1754_H__
#define __ADES1754_H__

#include "no_os_spi.h"
#include "no_os_uart.h"
#include "no_os_util.h"
#include "no_os_units.h"

#define ADES1754_VERSION_REG			0x00
#define ADES1754_ADDRESS_REG			0x01
#define ADES1754_STATUS1_REG			0x02
#define ADES1754_STATUS2_REG			0x03
#define ADES1754_STATUS3_REG			0x04
#define ADES1754_FMEA1_REG			0x05
#define ADES1754_FMEA2_REG			0x06
#define ADES1754_ALRTSUM_REG			0x07
#define ADES1754_ALRTOVCELL_REG			0x08
#define ADES1754_ALRTUVCELL_REG			0x09
#define ADES1754_MINMAXCELL_REG			0x0A
#define ADES1754_ALRTAUXPRTCT_REG		0x0B
#define ADES1754_ALRTUAUXOV_REG			0x0C
#define ADES1754_ALRTAUXUV_REG			0x0D
#define ADES1754_ALRTCOMPOVR_REG		0x0E
#define ADES1754_ALRTCOMPUVR_REG		0x0F
#define ADES1754_ALRTCOMPAUXOVR_REG		0x10
#define ADES1754_ALRTCOMPAUXUV_REG		0x11
#define ADES1754_ALRTBALSWRE_REG		0x12
#define ADES1754_SWACTION_REG			0x13
#define ADES1754_DEVCFG_REG			0x14
#define ADES1754_DEVCFG2_REG			0x15
#define ADES1754_AUXGPIOCFG_REG			0x16
#define ADES1754_GPIOCFG_REG			0x17
#define ADES1754_PACKCFG_REG			0x18

#define ADES1754_ALRTIRQEN_REG			0x19
#define ADES1754_ALRTOVEN_REG			0x1A
#define ADES1754_ALRTUVEN_REG			0x1B
#define ADES1754_ALRTAUXOVEN_REG		0x1C
#define ADES1754_ALRTAUXUVEN_REG		0x1D
#define ADES1754_ALRTCALTST_REG			0x1E

#define ADES1754_OVTHCLR_REG			0x1F
#define ADES1754_OVTHSET_REG			0x20
#define ADES1754_UVTHCLR_REG			0x21
#define ADES1754_UVTHSET_REG			0x22
#define ADES1754_MSMTCH_REG			0x23
#define ADES1754_BIPOVTHCLR_REG			0x24
#define ADES1754_BIPOVTHSET_REG			0x25
#define ADES1754_BIPUVTHCLR_REG			0x26
#define ADES1754_BIPUVTHSET_REG			0x27
#define ADES1754_BLKOVTHCLR_REG			0x28
#define ADES1754_BLKOVTHSET_REG			0x29
#define ADES1754_BLKUVTHCLR_REG			0x2A
#define ADES1754_BLKUVTHSET_REG			0x2B
#define ADES1754_AUXROVTHCLR_REG		0x30
#define ADES1754_AUXROVTHSET_REG		0x31
#define ADES1754_AUXRUVTHCLR_REG		0x32
#define ADES1754_AUXRUVTHSET_REG		0x33
#define ADES1754_AUXAOVTHCLR_REG		0x34
#define ADES1754_AUXAOVTHSET_REG		0x35
#define ADES1754_AUXAUVTHCLR_REG		0x36
#define ADES1754_AUXAUVTHSET_REG		0x37
#define ADES1754_COMPOVTH_REG			0x38
#define ADES1754_COMPUVTH_REG			0x39
#define ADES1754_COMPAUXROVTH_REG		0x3A
#define ADES1754_COMPAUXRUVTH_REG		0x3B
#define ADES1754_COMPAUXAOVTH_REG		0x3C
#define ADES1754_COMPAUXAUVTH_REG		0x3D

#define ADES1754_COMPOPNTH_REG			0x3E
#define ADES1754_COMPAUXROPNTH_REG		0x3F
#define ADES1754_COMPAUXOPNTH_REG		0x40
#define ADES1754_COMPACCOVTH_REG		0x41
#define ADES1754_COMPACCUVTH_REG		0x42
#define ADES1754_BALSHRTTHR_REG			0x43
#define ADES1754_BALLOWTHR_REG			0x44
#define ADES1754_BALHIGHTHR_REG			0x45

#define ADES1754_CELLN_REG(n)			(0x47 + (n))
#define ADES1754_BLOCK_REG			0x55

#define ADES1754_TOTAL_REG			0x56
#define ADES1754_DIAG1_REG			0x57
#define ADES1754_DIAG2_REG			0x58
#define ADES1754_AUX0_REG			0x59
#define ADES1754_AUX1_REG			0x5A
#define ADES1754_AUX2_REG			0x5B
#define ADES1754_AUX3_REG			0x5C
#define ADES1754_AUX4_REG			0x5D
#define ADES1754_AUX5_REG			0x5E

#define ADES1754_POLARITYCTRL_REG		0x5F
#define ADES1754_AUXREFCTRL_REG			0x60
#define ADES1754_AUXTIME_REG			0x61
#define ADES1754_ACQCFG_REG			0x62
#define ADES1754_BALSWDLY_REG			0x63

#define ADES1754_MEASUREEN1_REG			0x64
#define ADES1754_MEASUREEN2_REG			0x65
#define ADES1754_SCANCTRL_REG			0x66

#define ADES1754_ADCTEST1A_REG			0x67
#define ADES1754_ADCTEST1B_REG			0x68
#define ADES1754_ADCTEST2A_REG			0x69
#define ADES1754_ADCTEST2B_REG			0x6A
#define ADES1754_DIAGCFG_REG			0x6B
#define ADES1754_CTSTCFG_REG			0x6C
#define ADES1754_AUXTSTCFG_REG			0x6D
#define ADES1754_DIAGGENCFG_REG			0x6E

#define ADES1754_BALSWCTRL_REG			0x6F
#define ADES1754_BALEXP1_REG			0x70
#define ADES1754_BALEXP2_REG			0x71
#define ADES1754_BALEXP3_REG			0x72
#define ADES1754_BALEXP4_REG			0x73
#define ADES1754_BALEXP5_REG			0x74
#define ADES1754_BALEXP6_REG			0x75
#define ADES1754_BALEXP7_REG			0x76
#define ADES1754_BALEXP8_REG			0x77
#define ADES1754_BALEXP9_REG			0x78
#define ADES1754_BALEXP10_REG			0x79
#define ADES1754_BALEXP11_REG			0x7A
#define ADES1754_BALEXP12_REG			0x7B
#define ADES1754_BALEXP13_REG			0x7C
#define ADES1754_BALEXP14_REG			0x7D
#define ADES1754_BALAUTOUVTH_REG		0x7E
#define ADES1754_BALDLYCTRL_REG			0x7F
#define ADES1754_BALCTRL_REG			0x80
#define ADES1754_BALSTAT_REG			0x81
#define ADES1754_BALUVSTAT_REG			0x82
#define ADES1754_BALDATA_REG			0x83

#define ADES1754_I2CPNTR_REG			0x84
#define ADES1754_I2CWDATA1_REG			0x85
#define ADES1754_I2CWDATA2_REG			0x86
#define ADES1754_I2CRDATA1_REG			0x87
#define ADES1754_I2CRDATA2_REG			0x88
#define ADES1754_I2CCFG_REG			0x89
#define ADES1754_I2CSTAT_REG			0x8A
#define ADES1754_I2CSEND_REG			0x8B

#define ADES1754_ID1_REG			0x8C
#define ADES1754_ID2_REG			0x8D
#define ADES1754_ID3_REG			0x8E
#define ADES1754_OTP3_REG			0x8F
#define ADES1754_OTP4_REG			0x90
#define ADES1754_OTP5_REG			0x91
#define ADES1754_OTP6_REG			0x92
#define ADES1754_OTP7_REG			0x93
#define ADES1754_OTP8_REG			0x94
#define ADES1754_OTP9_REG			0x95
#define ADES1754_OTP10_REG			0x96
#define ADES1754_OTP11_REG			0x97
#define ADES1754_OTP12_REG			0x98

#define ADES1754_PREAMBLE_BYTE			0x15
#define ADES1754_STOP_BYTE			0x54
#define ADES1754_HELLO_ALL_BYTE			0x57

#define ADES1754_DEV_ADDR_MAX			0x1F
#define ADES1754_WR_ALL				0x02
#define ADES1754_RD_ALL				0x03
#define ADES1754_DEFAULT_DC_BYTE_SEED		0x00
#define ADES1754_FILL_BYTE_C2			0xC2
#define ADES1754_FILL_BYTE_D3			0xD3

#define ADES1754_WR_MASK			NO_OS_BIT(2)
#define ADES1754_RD_MASK			ADES1754_WR_MASK | NO_OS_BIT(0)
#define ADES1754_BL_MASK			NO_OS_GENMASK(2, 1)
#define ADES1754_RW_ADDR_MASK			NO_OS_GENMASK(7, 3)

#define ADES1754_LSB_MASK			NO_OS_GENMASK(7, 0)
#define ADES1754_MSB_MASK			NO_OS_GENMASK(15, 8)
#define ADES1754_LOWER_NIBBLE_MASK		NO_OS_GENMASK(3, 0)
#define ADES1754_UPPER_NIBBLE_MASK		NO_OS_GENMASK(7, 4)

#define ADES1754_ADC_METHOD_MASK		NO_OS_BIT(1)
#define ADES1754_SCAN_MODE_MASK			NO_OS_GENMASK(8, 6)
#define ADES1754_SCAN_REQUEST_MASK		NO_OS_BIT(0)
#define ADES1754_IIR_MASK			NO_OS_GENMASK(15, 13)
#define ADES1754_ALRTFILT_MASK			NO_OS_BIT(2)
#define ADES1754_AMENDFILT_MASK			NO_OS_BIT(1)
#define ADES1754_RDFILT_MASK			NO_OS_BIT(0)
#define ADES1754_IIR_SCAN_MASK			NO_OS_GENMASK(11, 9)
#define ADES1754_DBLBUFEN_MASK			NO_OS_BIT(0)
#define ADES1754_CBMODE_MASK			NO_OS_GENMASK(13, 11)
#define ADES1754_CBMEASEN_MASK			NO_OS_GENMASK(1, 0)
#define ADES1754_CBCALDLY_MASK			NO_OS_GENMASK(2, 0)
#define ADES1754_UARTCFG_MASK			NO_OS_GENMASK(15, 14)
#define ADES1754_CELL_POLARITY_MASK		NO_OS_BIT(15)

/***************************************************************************//**
 * @brief The `ades1754_id` enumeration defines a set of constants that
 * represent unique identifiers for different ADES175x series devices.
 * These identifiers are used to distinguish between various models
 * within the series, allowing for specific handling and configuration of
 * each device type in the ADES1754 driver.
 *
 * @param ID_ADES1754 Represents the identifier for the ADES1754 device.
 * @param ID_ADES1755 Represents the identifier for the ADES1755 device.
 * @param ID_ADES1756 Represents the identifier for the ADES1756 device.
 * @param ID_ADES1751 Represents the identifier for the ADES1751 device.
 * @param ID_ADES1752 Represents the identifier for the ADES1752 device.
 ******************************************************************************/
enum ades1754_id {
	ID_ADES1754,
	ID_ADES1755,
	ID_ADES1756,
	ID_ADES1751,
	ID_ADES1752,
};

/***************************************************************************//**
 * @brief The `ades1754_scan_method` enumeration defines the different scanning
 * methods available for the ADES1754 device, which are used to determine
 * how the device performs its scanning operations. The two methods
 * provided are the pyramid method and the ramp method, each offering a
 * distinct approach to scanning.
 *
 * @param ADES1754_PYRAMID_METHOD Represents the pyramid scan method for the
 * ADES1754 device.
 * @param ADES1754_RAMP_METHOD Represents the ramp scan method for the ADES1754
 * device.
 ******************************************************************************/
enum ades1754_scan_method {
	ADES1754_PYRAMID_METHOD,
	ADES1754_RAMP_METHOD,
};

/***************************************************************************//**
 * @brief The `ades1754_scan_mode` enumeration defines various modes for
 * scanning operations in the ADES1754 device, each mode corresponding to
 * a specific configuration or operation such as using the ADC,
 * comparator, or performing calibration and balancing tasks. This
 * enumeration is crucial for selecting the appropriate scanning behavior
 * based on the operational requirements of the device.
 *
 * @param ADES1754_SCAN_ADC Represents a scan mode that uses the ADC.
 * @param ADES1754_SCAN_COMP Represents a scan mode that uses the comparator.
 * @param ADES1754_SCAN_ADC_COMP Represents a scan mode that uses both ADC and
 * comparator.
 * @param ADES1754_ON_DEMAND_CALIB Represents a scan mode for on-demand
 * calibration.
 * @param ADES1754_BALANCING_SW_SHRT Represents a scan mode for balancing with a
 * short switch.
 * @param ADES1754_BALANCING_SW_OPEN Represents a scan mode for balancing with
 * an open switch.
 * @param ADES1754_CELL_SENSE_OPEN_ODDS Represents a scan mode for sensing open
 * odd-numbered cells.
 * @param ADES1754_CELL_SENSE_OPEN_EVENS Represents a scan mode for sensing open
 * even-numbered cells.
 ******************************************************************************/
enum ades1754_scan_mode {
	ADES1754_SCAN_ADC,
	ADES1754_SCAN_COMP,
	ADES1754_SCAN_ADC_COMP,
	ADES1754_ON_DEMAND_CALIB,
	ADES1754_BALANCING_SW_SHRT,
	ADES1754_BALANCING_SW_OPEN,
	ADES1754_CELL_SENSE_OPEN_ODDS,
	ADES1754_CELL_SENSE_OPEN_EVENS
};

/***************************************************************************//**
 * @brief The `ades1754_cell_polarity` enumeration defines the possible polarity
 * configurations for cells in the ADES1754 device. It includes two
 * values: `ADES1754_UNIPOLAR` for unipolar configuration and
 * `ADES1754_BIPOLAR` for bipolar configuration. This enumeration is used
 * to specify the polarity setting for the device's cell measurement
 * operations.
 *
 * @param ADES1754_UNIPOLAR Represents a unipolar cell polarity configuration.
 * @param ADES1754_BIPOLAR Represents a bipolar cell polarity configuration.
 ******************************************************************************/
enum ades1754_cell_polarity {
	ADES1754_UNIPOLAR,
	ADES1754_BIPOLAR,
};

/***************************************************************************//**
 * @brief The `ades1754_iir_filter_coef` enumeration defines a set of constants
 * representing different coefficients for an Infinite Impulse Response
 * (IIR) filter used in the ADES1754 device. These coefficients determine
 * the filter's response characteristics, with values ranging from 0.125
 * to 1.000, where 1.000 effectively disables the filter. This
 * enumeration is used to configure the IIR filter settings in the
 * device, allowing for various levels of signal smoothing or filtering.
 *
 * @param ADES1754_IIR_0_125 Represents an IIR filter coefficient of 0.125.
 * @param ADES1754_IIR_0_250 Represents an IIR filter coefficient of 0.250.
 * @param ADES1754_IIR_0_375 Represents an IIR filter coefficient of 0.375.
 * @param ADES1754_IIR_0_500 Represents an IIR filter coefficient of 0.500.
 * @param ADES1754_IIR_0_625 Represents an IIR filter coefficient of 0.625.
 * @param ADES1754_IIR_0_750 Represents an IIR filter coefficient of 0.750.
 * @param ADES1754_IIR_0_875 Represents an IIR filter coefficient of 0.875.
 * @param ADES1754_IIR_1_000_OFF Represents an IIR filter coefficient of 1.000,
 * effectively turning the filter off.
 ******************************************************************************/
enum ades1754_iir_filter_coef {
	ADES1754_IIR_0_125,
	ADES1754_IIR_0_250,
	ADES1754_IIR_0_375,
	ADES1754_IIR_0_500,
	ADES1754_IIR_0_625,
	ADES1754_IIR_0_750,
	ADES1754_IIR_0_875,
	ADES1754_IIR_1_000_OFF,
};

/***************************************************************************//**
 * @brief The `ades1754_buffer_mode` enumeration defines two modes of buffer
 * operation for the ADES1754 device: single buffer and double buffer.
 * This enumeration is used to configure how data is buffered within the
 * device, allowing for either a single buffer or a double buffer setup,
 * which can affect data handling and processing efficiency.
 *
 * @param ADES1754_SINGLE_BUFF Represents a single buffer mode.
 * @param ADES1754_DOUBLE_BUFF Represents a double buffer mode.
 ******************************************************************************/
enum ades1754_buffer_mode {
	ADES1754_SINGLE_BUFF,
	ADES1754_DOUBLE_BUFF,
};

/***************************************************************************//**
 * @brief The `ades1754_alert` enumeration defines a set of alert conditions for
 * the ADES1754 device, which is likely a battery management or
 * monitoring system. Each enumerator represents a specific type of alert
 * condition, such as over-voltage or under-voltage, for different
 * components like cells, blocks, or auxiliary inputs. This enumeration
 * is used to identify and handle various alert scenarios that the device
 * might encounter during operation.
 *
 * @param ADES1754_CELL_OV Represents an over-voltage alert for a cell.
 * @param ADES1754_CELL_UV Represents an under-voltage alert for a cell.
 * @param ADES1754_BIPOLAR_OV Represents an over-voltage alert for a bipolar
 * configuration.
 * @param ADES1754_BIPOLAR_UV Represents an under-voltage alert for a bipolar
 * configuration.
 * @param ADES1754_BLOCK_OV Represents an over-voltage alert for a block.
 * @param ADES1754_BLOCK_UV Represents an under-voltage alert for a block.
 * @param ADES1754_CELL_MISMATCH Indicates a mismatch alert between cells.
 * @param ADES1754_AUXIN_OV Represents an over-voltage alert for auxiliary
 * input.
 * @param ADES1754_AUXIN_UV Represents an under-voltage alert for auxiliary
 * input.
 ******************************************************************************/
enum ades1754_alert {
	ADES1754_CELL_OV,
	ADES1754_CELL_UV,
	ADES1754_BIPOLAR_OV,
	ADES1754_BIPOLAR_UV,
	ADES1754_BLOCK_OV,
	ADES1754_BLOCK_UV,
	ADES1754_CELL_MISMATCH,
	ADES1754_AUXIN_OV,
	ADES1754_AUXIN_UV,
};

/***************************************************************************//**
 * @brief The `ades1754_bal_mode` enumeration defines various modes of operation
 * for balancing in the ADES1754 driver. It includes modes for both
 * manual and automatic balancing, with options for different time
 * granularities (seconds and minutes) and group balancing capabilities.
 * This enumeration is used to configure the balancing behavior of the
 * ADES1754 device, allowing for flexible management of balancing
 * operations.
 *
 * @param ADES1754_BAL_DISABLED Represents the disabled state for balancing
 * mode.
 * @param ADES1754_BAL_EMERGENCY Represents the emergency state for balancing
 * mode.
 * @param ADES1754_BAL_MANUAL_SEC Represents manual balancing mode with second-
 * level granularity.
 * @param ADES1754_BAL_MANUAL_MIN Represents manual balancing mode with minute-
 * level granularity.
 * @param ADES1754_BAL_AUTO_SEC Represents automatic balancing mode with second-
 * level granularity.
 * @param ADES1754_BAL_AUTO_MIN Represents automatic balancing mode with minute-
 * level granularity.
 * @param ADES1754_BAL_AUTO_GROUP_SEC Represents automatic group balancing mode
 * with second-level granularity.
 * @param ADES1754_BAL_AUTO_GROUP_MIN Represents automatic group balancing mode
 * with minute-level granularity.
 ******************************************************************************/
enum ades1754_bal_mode {
	ADES1754_BAL_DISABLED,
	ADES1754_BAL_EMERGENCY,
	ADES1754_BAL_MANUAL_SEC,
	ADES1754_BAL_MANUAL_MIN,
	ADES1754_BAL_AUTO_SEC,
	ADES1754_BAL_AUTO_MIN,
	ADES1754_BAL_AUTO_GROUP_SEC,
	ADES1754_BAL_AUTO_GROUP_MIN,
};

/***************************************************************************//**
 * @brief The `ades1754_bal_meas` enumeration defines the measurement states for
 * the ADES1754 device, providing options to enable or disable all
 * measurements, or to enable ADC calibration while disabling CBUVTHR.
 * This enum is used to configure the measurement behavior of the device,
 * allowing for flexible control over its operational modes.
 *
 * @param ADES1754_ALL_MEAS_DISABLED Represents a state where all measurements
 * are disabled.
 * @param ADES1754_ADC_CAL_ON_CBUVTHR_OFF Indicates that ADC calibration is on
 * and CBUVTHR is off.
 * @param ADES1754_ALL_MEAS_ENABLED Represents a state where all measurements
 * are enabled.
 ******************************************************************************/
enum ades1754_bal_meas {
	ADES1754_ALL_MEAS_DISABLED = 1,
	ADES1754_ADC_CAL_ON_CBUVTHR_OFF,
	ADES1754_ALL_MEAS_ENABLED,
};

/***************************************************************************//**
 * @brief The `ades1754_bal_calib` enumeration defines various states for
 * balancing calibration cycles in the ADES1754 driver. Each enumerator
 * represents a specific number of cycles for the balancing calibration
 * process, allowing the user to select the desired calibration cycle
 * count or disable it entirely. This is useful for configuring the
 * balancing behavior of the ADES1754 device.
 *
 * @param ADES1754_BAL_CALIB_DISABLED Represents the state where balancing
 * calibration is disabled.
 * @param ADES1754_BAL_2_CYCLE Represents a balancing calibration cycle of 2.
 * @param ADES1754_BAL_4_CYCLE Represents a balancing calibration cycle of 4.
 * @param ADES1754_BAL_8_CYCLE Represents a balancing calibration cycle of 8.
 * @param ADES1754_BAL_12_CYCLE Represents a balancing calibration cycle of 12.
 * @param ADES1754_BAL_16_CYCLE Represents a balancing calibration cycle of 16.
 * @param ADES1754_BAL_24_CYCLE Represents a balancing calibration cycle of 24.
 ******************************************************************************/
enum ades1754_bal_calib {
	ADES1754_BAL_CALIB_DISABLED,
	ADES1754_BAL_2_CYCLE,
	ADES1754_BAL_4_CYCLE,
	ADES1754_BAL_8_CYCLE,
	ADES1754_BAL_12_CYCLE,
	ADES1754_BAL_16_CYCLE,
	ADES1754_BAL_24_CYCLE,
};

/***************************************************************************//**
 * @brief The `ades1754_init_param` structure is used to initialize the ADES1754
 * device, containing parameters necessary for setting up communication
 * and identifying the device. It includes a pointer to UART
 * initialization parameters, an enumeration for device identification, a
 * boolean to indicate the use of a UART bridge, and two 8-bit unsigned
 * integers for device address and the number of devices.
 *
 * @param uart_param Pointer to a structure containing UART initialization
 * parameters.
 * @param id Enumeration value representing the specific ADES1754 device ID.
 * @param uart_bridge Boolean flag indicating if a UART bridge is used.
 * @param dev_addr 8-bit unsigned integer representing the device address.
 * @param no_dev 8-bit unsigned integer representing the number of devices.
 ******************************************************************************/
struct ades1754_init_param {
	struct no_os_uart_init_param *uart_param;
	enum ades1754_id id;
	bool uart_bridge;
	uint8_t dev_addr;
	uint8_t no_dev;
};

/***************************************************************************//**
 * @brief The `ades1754_desc` structure is used to encapsulate the configuration
 * and state information for an ADES1754 device. It includes a pointer to
 * a UART descriptor for communication, an identifier for the specific
 * device variant, and flags for UART bridging and device activity.
 * Additionally, it holds configuration settings such as the scanning
 * mode, cell polarity, device address, and the number of devices in the
 * system. This structure is essential for managing and interfacing with
 * the ADES1754 device in a system.
 *
 * @param uart_desc Pointer to a UART descriptor for communication.
 * @param id Identifier for the specific ADES1754 device variant.
 * @param uart_bridge Boolean flag indicating if UART bridging is enabled.
 * @param scan_mode Specifies the scanning mode of the device.
 * @param cell_polarity Defines the cell polarity configuration.
 * @param dev_addr Device address for communication.
 * @param no_dev Number of devices in the system.
 * @param alive Boolean flag indicating if the device is active or responsive.
 ******************************************************************************/
struct ades1754_desc {
	struct no_os_uart_desc *uart_desc;
	enum ades1754_id id;
	bool uart_bridge;
	enum ades1754_scan_mode scan_mode;
	enum ades1754_cell_polarity cell_polarity;
	uint8_t dev_addr;
	uint8_t no_dev;
	bool alive;
};

/***************************************************************************//**
 * @brief This function is used to send a 'hello' command to all devices
 * connected in the ADES1754 network, which can be useful for network
 * discovery or initialization purposes. It requires a valid descriptor
 * that has been properly initialized and configured. The function
 * handles communication over UART, either directly or through a bridge,
 * depending on the configuration in the descriptor. It returns an error
 * code if the communication fails, ensuring that the caller can handle
 * such cases appropriately.
 *
 * @param desc A pointer to a struct ades1754_desc that must be initialized and
 * configured with the necessary UART settings and device
 * information. The structure must not be null, and it should
 * reflect the current state of the network configuration.
 * @return Returns 0 on success, or a negative error code if the UART
 * communication fails.
 ******************************************************************************/
int ades1754_hello_all(struct ades1754_desc *desc);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register of the ADES1754 device. It requires a valid device descriptor
 * and the target register address. The function supports both UART and
 * non-UART communication modes, automatically handling the data
 * transmission format based on the descriptor's configuration. It is
 * essential to ensure that the device descriptor is properly initialized
 * and that the register address is within the valid range for the
 * device. The function returns an error code if the write operation
 * fails.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure. This must
 * not be null and should be properly configured to represent the
 * target device.
 * @param reg The register address to which the data will be written. It should
 * be a valid register address as defined by the device's register
 * map.
 * @param data The 16-bit data to be written to the specified register. The data
 * is split into two bytes for transmission.
 * @return Returns 0 on success or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int ades1754_write_dev(struct ades1754_desc *desc, uint8_t reg, uint16_t data);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specified
 * register across all connected ADES1754 devices. It should be called
 * when a uniform update to a register is required for all devices in the
 * network. The function requires a valid descriptor that has been
 * initialized and configured to use either a UART bridge or a direct
 * connection. The function handles data transmission and ensures data
 * integrity using a CRC check. It returns an error code if the write
 * operation fails.
 *
 * @param desc A pointer to an initialized ades1754_desc structure. This must
 * not be null and should be properly configured to represent the
 * devices to be written to.
 * @param reg The register address to which the data will be written. It should
 * be a valid register address as defined in the ADES1754 register
 * map.
 * @param data The 16-bit data to be written to the specified register. The data
 * is split into two bytes for transmission.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the write operation fails.
 ******************************************************************************/
int ades1754_write_all(struct ades1754_desc *desc, uint8_t reg, uint16_t data);

/***************************************************************************//**
 * @brief This function is used to read a 16-bit value from a specific register
 * of the ADES1754 device. It requires a valid device descriptor and the
 * register address from which the data is to be read. The function
 * supports both UART bridge and non-bridge communication modes. It
 * performs error checking, including CRC validation, to ensure data
 * integrity. The function must be called with a properly initialized
 * device descriptor, and the data pointer must not be null. In case of
 * communication errors or invalid CRC, the function returns a negative
 * error code.
 *
 * @param desc A pointer to a struct ades1754_desc that describes the device.
 * Must be properly initialized and not null. The caller retains
 * ownership.
 * @param reg A uint8_t representing the register address to read from. Must be
 * a valid register address as defined by the device.
 * @param data A pointer to a uint16_t where the read data will be stored. Must
 * not be null. The function writes the read value to this location.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., communication error, invalid
 * CRC).
 ******************************************************************************/
int ades1754_read_dev(struct ades1754_desc *desc, uint8_t reg, uint16_t *data);

/***************************************************************************//**
 * @brief This function reads data from all devices connected to the ADES1754
 * using the specified register. It is intended for use when multiple
 * devices are connected and data needs to be retrieved from each one.
 * The function requires a valid descriptor that has been initialized and
 * configured for the devices. It handles communication through UART, and
 * the data is returned through the provided pointer. The function
 * performs error checking and will return an error code if communication
 * fails or if the data integrity check fails.
 *
 * @param desc A pointer to a struct ades1754_desc that describes the device
 * configuration and communication settings. Must not be null and
 * should be properly initialized before calling this function.
 * @param reg A uint8_t value representing the register from which to read data.
 * Must be a valid register address as defined in the ADES1754
 * register map.
 * @param data A pointer to a uint16_t array where the read data will be stored.
 * The array must be large enough to hold data for all devices
 * specified in the descriptor. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * communication errors or data integrity check failures.
 ******************************************************************************/
int ades1754_read_all(struct ades1754_desc *desc, uint8_t reg, uint16_t *data);

/***************************************************************************//**
 * @brief This function is used to read a block of data from a specified
 * register of the ADES1754 device. It requires a valid device descriptor
 * and the block and register addresses to read from. The function
 * supports reading either a single or double-sized block of data,
 * depending on the 'double_size' parameter. It is important to ensure
 * that the device descriptor is properly initialized and that the 'data'
 * pointer is valid and has enough space to store the read data. The
 * function returns an error code if the read operation fails or if the
 * data integrity check fails.
 *
 * @param desc A pointer to a 'struct ades1754_desc' that describes the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param block The block address from which to read. Must be a valid block
 * address for the ADES1754 device.
 * @param reg The register address within the block to read from. Must be a
 * valid register address for the ADES1754 device.
 * @param data A pointer to a uint16_t buffer where the read data will be
 * stored. Must not be null and should have enough space to store
 * the data, especially if 'double_size' is true.
 * @param double_size A boolean indicating whether to read a double-sized block
 * of data. If true, the function reads and stores two data
 * values; otherwise, it reads a single data value.
 * @return Returns 0 on success, a negative error code on failure, or if data
 * integrity checks fail.
 ******************************************************************************/
int ades1754_read_block(struct ades1754_desc *desc, uint8_t block, uint8_t reg,
			uint16_t *data, bool double_size);

/***************************************************************************//**
 * @brief This function is used to modify a specific register of the ADES1754
 * device by applying a mask and a new value. It first reads the current
 * value of the register, applies the mask to clear specific bits, and
 * then sets the new value in those bits. This function should be called
 * when there is a need to update specific bits of a register without
 * affecting other bits. It is important to ensure that the device
 * descriptor is properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param reg The register address to be updated. Must be a valid register
 * address for the ADES1754 device.
 * @param mask A 16-bit mask indicating which bits of the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param val The new value to be written to the masked bits of the register.
 * Must fit within the masked bits.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ades1754_update_dev(struct ades1754_desc *desc, uint8_t reg, uint16_t mask,
			uint16_t val);

/***************************************************************************//**
 * @brief Use this function to configure the ADC scan method of an ADES1754
 * device. This function should be called when you need to change the
 * scan method to either pyramid or ramp, as defined by the
 * `ades1754_scan_method` enumeration. Ensure that the device descriptor
 * is properly initialized before calling this function. The function
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param desc A pointer to an `ades1754_desc` structure representing the
 * device. Must not be null and should be properly initialized
 * before use. The caller retains ownership.
 * @param scan_method An `ades1754_scan_method` enumeration value specifying the
 * desired ADC scan method. Valid values are
 * `ADES1754_PYRAMID_METHOD` and `ADES1754_RAMP_METHOD`.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int ades1754_set_adc_method(struct ades1754_desc *desc,
			    enum ades1754_scan_method scan_method);

/***************************************************************************//**
 * @brief Use this function to change the scan mode of an ADES1754 device to a
 * specified mode. This function should be called when you need to alter
 * the scanning behavior of the device, such as switching between
 * different types of scans or calibration modes. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function updates the device's scan mode register and modifies the
 * descriptor to reflect the new mode. If the operation fails, an error
 * code is returned.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null. The caller retains
 * ownership.
 * @param mode An `ades1754_scan_mode` enumeration value specifying the desired
 * scan mode. Must be a valid enumeration value.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ades1754_switch_scan_mode(struct ades1754_desc *desc,
			      enum ades1754_scan_mode mode);

/***************************************************************************//**
 * @brief This function configures the cell polarity of the ADES1754 device by
 * writing the specified polarity setting to the device's polarity
 * control register. It should be called when the cell polarity needs to
 * be updated, ensuring that the device descriptor is properly
 * initialized beforehand. The function updates the descriptor with the
 * new polarity setting and returns an error code if the write operation
 * fails.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null, and the device should
 * be properly initialized before calling this function.
 * @param cell_polarity An `ades1754_cell_polarity` enum value indicating the
 * desired cell polarity setting. Valid values are
 * `ADES1754_UNIPOLAR` and `ADES1754_BIPOLAR`.
 * @return Returns 0 on success or a negative error code if the write operation
 * to the device fails.
 ******************************************************************************/
int ades1754_set_cell_pol(struct ades1754_desc *desc,
			  enum ades1754_cell_polarity cell_polarity);

/***************************************************************************//**
 * @brief This function is used to start a scan on the ADES1754 device, which
 * can be configured to include measurement operations. It requires a
 * valid device descriptor and a cell mask to specify which cells to
 * include in the scan. The function should be called when a scan is
 * needed, and it will configure the device accordingly. It is important
 * to ensure that the cell mask is within the valid range before calling
 * this function, as invalid values will result in an error.
 *
 * @param desc A pointer to a valid ades1754_desc structure representing the
 * device. Must not be null.
 * @param meas A boolean value indicating whether measurement should be included
 * in the scan. True to include measurement, false otherwise.
 * @param cell_mask A 16-bit mask specifying which cells to include in the scan.
 * Valid values are within the range defined by
 * NO_OS_GENMASK(14, 1). Values outside this range will result
 * in an error.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for an invalid cell mask.
 ******************************************************************************/
int ades1754_start_scan(struct ades1754_desc *desc, bool meas,
			uint16_t cell_mask);

/***************************************************************************//**
 * @brief Use this function to obtain the voltage measurement of a specific cell
 * in the ADES1754 device. It requires a valid device descriptor and the
 * cell number to be queried. The function will store the voltage data in
 * the provided pointer. Ensure that the device has been properly
 * initialized and that the cell number is within the valid range for the
 * device. The function will handle the sign extension if the cell
 * polarity is set to bipolar. It returns an error code if the read
 * operation fails.
 *
 * @param desc A pointer to a valid ades1754_desc structure representing the
 * device. Must not be null.
 * @param cell_nb The number of the cell to read the voltage from. Must be
 * within the valid range of cells for the device.
 * @param cell_voltage A pointer to an int32_t where the cell voltage will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ades1754_get_cell_data(struct ades1754_desc *desc, uint8_t cell_nb,
			   int32_t *cell_voltage);

/***************************************************************************//**
 * @brief This function configures the Infinite Impulse Response (IIR) filter
 * coefficient for the ADES1754 device. It should be called when you need
 * to adjust the filtering characteristics of the device's signal
 * processing. Ensure that the device descriptor is properly initialized
 * before calling this function. The function modifies the device's
 * configuration register to set the specified IIR filter coefficient.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param coef An `ades1754_iir_filter_coef` enumeration value specifying the
 * desired IIR filter coefficient. Valid values are defined in the
 * `ades1754_iir_filter_coef` enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ades1754_set_iir(struct ades1754_desc *desc,
		     enum ades1754_iir_filter_coef coef);

/***************************************************************************//**
 * @brief This function sets the IIR (Infinite Impulse Response) control
 * settings for the ADES1754 device by configuring alert filtering,
 * accumulation, and output filtering options. It should be called when
 * you need to adjust these specific filter settings on the device.
 * Ensure that the device descriptor is properly initialized before
 * calling this function. The function modifies the device's scan control
 * register to apply the specified filter settings.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param alrtfilt A boolean value indicating whether alert filtering is enabled
 * (true) or disabled (false).
 * @param acc A boolean value indicating whether accumulation is enabled (true)
 * or disabled (false).
 * @param output A boolean value indicating whether output filtering is enabled
 * (true) or disabled (false).
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ades1754_set_iir_ctrl(struct ades1754_desc *desc, bool alrtfilt, bool acc,
			  bool output);

/***************************************************************************//**
 * @brief Use this function to configure the buffer mode of an ADES1754 device,
 * which determines how data buffering is handled. This function should
 * be called when you need to switch between single and double buffering
 * modes, depending on your application's requirements. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param mode An `ades1754_buffer_mode` enumeration value specifying the
 * desired buffer mode. Valid values are `ADES1754_SINGLE_BUFF` and
 * `ADES1754_DOUBLE_BUFF`.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ades1754_set_buffer_mode(struct ades1754_desc *desc,
			     enum ades1754_buffer_mode mode);

/***************************************************************************//**
 * @brief This function configures the threshold for a specific alert type on
 * the ADES1754 device. It should be used when you need to set or update
 * the threshold values for various alert conditions such as over-voltage
 * or under-voltage. The function requires a valid device descriptor and
 * an alert type to be specified. It is important to ensure that the
 * descriptor is properly initialized before calling this function. The
 * function will return an error if an invalid alert type is provided.
 *
 * @param desc A pointer to a struct ades1754_desc, representing the device
 * descriptor. Must be initialized and not null. The caller retains
 * ownership.
 * @param alert An enum ades1754_alert value specifying the type of alert for
 * which the threshold is being set. Must be a valid alert type
 * defined in the enum.
 * @param thr A uint16_t value representing the threshold to be set for the
 * specified alert. The value is masked and prepared for writing to
 * the device.
 * @return Returns 0 on success or a negative error code on failure, such as
 * when an invalid alert type is provided.
 ******************************************************************************/
int ades1754_set_alert_thr(struct ades1754_desc *desc,
			   enum ades1754_alert alert,
			   uint16_t thr);

/***************************************************************************//**
 * @brief Use this function to check whether a specific alert condition is
 * currently enabled on the ADES1754 device. It must be called with a
 * valid device descriptor and a valid alert type. The function will
 * populate the provided boolean pointer with the alert status,
 * indicating if the alert is active. Ensure that the device descriptor
 * is properly initialized before calling this function. If an invalid
 * alert type is provided, the function will return an error.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param alert An `ades1754_alert` enumeration value specifying the alert type
 * to check. Must be a valid alert type.
 * @param enable A pointer to a boolean variable where the alert status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an invalid alert
 * type is specified or if there is a communication error with the
 * device.
 ******************************************************************************/
int ades1754_get_alert(struct ades1754_desc *desc, enum ades1754_alert alert,
		       bool *enable);

/***************************************************************************//**
 * @brief Use this function to configure the balancing mode of the ADES1754
 * device. It is essential to ensure that the device descriptor is
 * properly initialized before calling this function. The function
 * modifies the balancing control register to set the desired mode. This
 * operation is critical for applications that require specific balancing
 * behavior, such as battery management systems. Ensure that the mode
 * parameter is valid to avoid unexpected behavior.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null, and the device should
 * be properly initialized before use.
 * @param mode An enumeration value of type `ades1754_bal_mode` specifying the
 * desired balancing mode. Must be a valid mode defined in the
 * `ades1754_bal_mode` enum.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * that the balancing mode was not set.
 ******************************************************************************/
int ades1754_set_balancing_mode(struct ades1754_desc *desc,
				enum ades1754_bal_mode mode);

/***************************************************************************//**
 * @brief This function sets the balancing measurement mode on the ADES1754
 * device, which is used to control how the device performs measurements
 * related to cell balancing. It should be called when you need to change
 * the measurement mode to one of the predefined modes specified by the
 * `ades1754_bal_meas` enumeration. Ensure that the device descriptor is
 * properly initialized before calling this function. The function
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param desc A pointer to an `ades1754_desc` structure representing the device
 * descriptor. This must be a valid, non-null pointer to a properly
 * initialized descriptor.
 * @param meas An `ades1754_bal_meas` enumeration value specifying the desired
 * balancing measurement mode. The value must be one of the
 * predefined modes in the `ades1754_bal_meas` enum.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ades1754_set_balancing_meas(struct ades1754_desc *desc,
				enum ades1754_bal_meas meas);

/***************************************************************************//**
 * @brief Use this function to configure the balancing calibration mode of an
 * ADES1754 device. This function should be called when you need to
 * adjust the calibration settings for cell balancing operations. Ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function modifies the calibration settings based on the
 * provided calibration mode.
 *
 * @param desc A pointer to an initialized `ades1754_desc` structure
 * representing the device. Must not be null.
 * @param calib An `ades1754_bal_calib` enumeration value specifying the desired
 * calibration mode. Valid values are defined in the
 * `ades1754_bal_calib` enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ades1754_set_balancing_calib(struct ades1754_desc *desc,
				 enum ades1754_bal_calib calib);

/***************************************************************************//**
 * @brief This function sets up and initializes a descriptor for the ADES1754
 * device, preparing it for communication. It must be called before any
 * other operations on the device. The function allocates memory for the
 * descriptor and initializes the UART communication based on the
 * provided parameters. It also performs a basic communication check with
 * the device. If initialization fails at any step, appropriate error
 * codes are returned, and resources are cleaned up.
 *
 * @param desc A pointer to a pointer where the initialized descriptor will be
 * stored. Must not be null. The caller is responsible for managing
 * the memory of the descriptor after initialization.
 * @param init_param A pointer to a structure containing initialization
 * parameters such as UART settings, device address, and
 * device ID. Must not be null. The device address must not
 * exceed ADES1754_DEV_ADDR_MAX.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid parameters, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int ades1754_init(struct ades1754_desc **desc,
		  struct ades1754_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly clean up and release all resources
 * associated with an ADES1754 device descriptor when it is no longer
 * needed. This function should be called to prevent resource leaks after
 * the device has been initialized and used. It ensures that any
 * allocated memory and associated UART resources are freed. The function
 * must be called with a valid descriptor that was previously
 * initialized, and it is expected to be called only once per descriptor.
 *
 * @param desc A pointer to a valid `ades1754_desc` structure. This must not be
 * null and should point to a descriptor that was successfully
 * initialized. The function will handle freeing the memory, so the
 * caller should not use the descriptor after this call.
 * @return Returns 0 to indicate successful removal and resource deallocation.
 * No other values are returned.
 ******************************************************************************/
int ades1754_remove(struct ades1754_desc *desc);

#endif /* __ADES1754_H__ */

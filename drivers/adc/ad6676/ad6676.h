/***************************************************************************//**
* @file ad6676.h
* @brief Header file of AD6676 Driver.
* @authors Dragos Bogdan (dragos.bogdan@analog.com)
* @authors Andrei Grozav (andrei.grozav@analog.com)
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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
#ifndef AD6676_H_
#define AD6676_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD6676_SPI_CONFIG		0x000
#define AD6676_DEVICE_CONFIG		0x002
#define AD6676_CHIP_TYPE		0x003
#define AD6676_CHIP_ID0			0x004
#define AD6676_CHIP_ID1			0x005
#define AD6676_GRADE_REVISION		0x006
#define AD6676_VENDOR_ID0		0x00C
#define AD6676_VENDOR_ID1		0x00D
#define AD6676_PCBL_DONE		0x0FE

/* CONFIGURATION SETTINGS */
#define AD6676_FADC_0			0x100
#define AD6676_FADC_1			0x101
#define AD6676_FIF_0			0x102
#define AD6676_FIF_1			0x103
#define AD6676_BW_0			0x104
#define AD6676_BW_1			0x105
#define AD6676_LEXT			0x106
#define AD6676_MRGN_L			0x107
#define AD6676_MRGN_U			0x108
#define AD6676_MRGN_IF			0x109
#define AD6676_XSCALE_1			0x10A

/* BP SD ADC CALIBRATION/PROFILE */
#define AD6676_CAL_CTRL			0x115
#define AD6676_CAL_CMD			0x116
#define AD6676_CAL_DONE			0x117
#define AD6676_ADC_CONFIG		0x118
#define AD6676_FORCE_END_CAL		0x11A

/* DIGITAL SIGNAL PATH */
#define AD6676_DEC_MODE			0x140
#define AD6676_MIX1_TUNING		0x141
#define AD6676_MIX2_TUNING		0x142
#define AD6676_MIX1_INIT		0x143
#define AD6676_MIX2_INIT_LSB		0x144
#define AD6676_MIX2_INIT_MSB		0x145
#define AD6676_DP_CTRL			0x146

/* POWER CONTROL */
#define AD6676_STANDBY			0x150
#define AD6676_PD_DIG			0x151
#define AD6676_PD_PIN_CTRL		0x152
#define AD6676_STBY_DAC			0x250

/* ATTENUATOR */
#define AD6676_ATTEN_MODE		0x180
#define AD6676_ATTEN_VALUE_PIN0		0x181
#define AD6676_ATTEN_VALUE_PIN1		0x182
#define AD6676_ATTEN_INIT		0x183
#define AD6676_ATTEN_CTL		0x184

/* ADC RESET CONTROL */
#define AD6676_ADCRE_THRH		0x188
#define AD6676_ADCRE_PULSE_LEN		0x189
#define AD6676_ATTEN_STEP_RE		0x18A
#define AD6676_TIME_PER_STEP		0x18B

/* PEAK DETECTOR AND AGC FLAG CONTROL */
#define AD6676_ADC_UNSTABLE		0x18F
#define AD6676_PKTHRH0_LSB		0x193
#define AD6676_PKTHRH0_MSB		0x194
#define AD6676_PKTHRH1_LSB		0x195
#define AD6676_PKTHRH1_MSB		0x196
#define AD6676_LOWTHRH_LSB		0x197
#define AD6676_LOWTHRH_MSB		0x198
#define AD6676_DWELL_TIME_MANTISSA	0x199
#define AD6676_DWELL_TIME_EXP		0x19A
#define AD6676_FLAG0_SEL		0x19B
#define AD6676_FLAG1_SEL		0x19C
#define AD6676_EN_FLAG			0x19E

/* GPIO CONFIGURATION */
#define AD6676_FORCE_GPIO		0x1B0
#define AD6676_FORCE_GPIO_OUT		0x1B1
#define AD6676_FORCE_GPIO_VAL		0x1B2
#define AD6676_READ_GPO			0x1B3
#define AD6676_READ_GPI			0x1B4

/* AD6676 JESD204B INTERFACE */
#define AD6676_DID			0x1C0
#define AD6676_BID			0x1C1
#define AD6676_L			0x1C3
#define AD6676_F			0x1C4
#define AD6676_K			0x1C5
#define AD6676_M			0x1C6
#define AD6676_S			0x1C9
#define AD6676_HD			0x1CA
#define AD6676_RES1			0x1CB
#define AD6676_RES2			0x1CC
#define AD6676_LID0			0x1D0
#define AD6676_LID1			0x1D1
#define AD6676_FCHK0			0x1D8
#define AD6676_FCHK1			0x1D9
#define AD6676_EN_LFIFO			0x1E0
#define AD6676_SWAP			0x1E1
#define AD6676_LANE_PD			0x1E2
#define AD6676_MIS1			0x1E3
#define AD6676_SYNC_PIN			0x1E4
#define AD6676_TEST_GEN			0x1E5
#define AD6676_KF_ILAS			0x1E6
#define AD6676_SYNCB_CTRL		0x1E7
#define AD6676_MIX_CTRL			0x1E8
#define AD6676_K_OFFSET			0x1E9
#define AD6676_SYSREF			0x1EA
#define AD6676_SER1			0x1EB
#define AD6676_SER2			0x1EC

#define AD6676_CLKSYN_ENABLE		0x2A0
#define AD6676_CLKSYN_INT_N_LSB		0x2A1
#define AD6676_CLKSYN_INT_N_MSB		0x2A2
#define AD6676_CLKSYN_LOGEN		0x2A5
#define AD6676_CLKSYN_KVCO_VCO		0x2A9
#define AD6676_CLKSYN_VCO_BIAS		0x2AA
#define AD6676_CLKSYN_VCO_CAL		0x2AB
#define AD6676_CLKSYN_I_CP		0x2AC
#define AD6676_CLKSYN_CP_CAL		0x2AD
#define AD6676_CLKSYN_VCO_VAR		0x2B7
#define AD6676_CLKSYN_R_DIV		0x2BB
#define AD6676_CLKSYN_STATUS		0x2BC
#define AD6676_JESDSYN_STATUS		0x2DC

#define AD6676_SHUFFLE_THREG0		0x342
#define AD6676_SHUFFLE_THREG1		0x343

/*
 * AD6676_SPI_CONFIG
 */

#define SPI_CONF_SW_RESET		(0x81)
#define SPI_CONF_SDIO_DIR		(0x18)


/*
 * AD6676_CLKSYN_STATUS, AD6676_JESDSYN_STATUS
 */
#define SYN_STAT_PLL_LCK		(1 << 3)
#define SYN_STAT_VCO_CAL_BUSY		(1 << 1)
#define SYN_STAT_CP_CAL_DONE		(1 << 0)

/*
 * AD6676_DP_CTRL
 */
#define DP_CTRL_OFFSET_BINARY		(1 << 0)
#define DP_CTRL_TWOS_COMPLEMENT		(0 << 0)

/*
 * AD6676_TEST_GEN
 */
#define TESTGENMODE_OFF			0x0
#define TESTGENMODE_ALT_CHECKERBOARD	0x1
#define TESTGENMODE_ONE_ZERO_TOGGLE	0x2
#define TESTGENMODE_PN23_SEQ		0x3
#define TESTGENMODE_PN9_SEQ		0x4
#define TESTGENMODE_REP_USER_PAT	0x5
#define TESTGENMODE_SING_USER_PAT	0x6
#define TESTGENMODE_RAMP		0x7
#define TESTGENMODE_MOD_RPAT		0x8
#define TESTGENMODE_JSPAT		0x10
#define TESTGENMODE_JTSPAT		0x11

/*
 * AD6676_CLKSYN_R_DIV
 */

#define R_DIV(x)			((x) << 6)
#define CLKSYN_R_DIV_SYSREF_CTRL	(1 << 3)
#define CLKSYN_R_DIV_CLKIN_IMPED	(1 << 2)
#define CLKSYN_R_DIV_RESERVED		0x31

/*
 * AD6676_CLKSYN_ENABLE
 */
#define EN_EXT_CK			(1 << 7)
#define EN_ADC_CK			(1 << 6)
#define EN_SYNTH			(1 << 5)
#define EN_VCO_PTAT			(1 << 4)
#define EN_VCO_ALC			(1 << 3)
#define EN_VCO				(1 << 2)
#define EN_OVER_IDE_CAL			(1 << 1)
#define EN_OVER_IDE			(1 << 0)

/*
 * AD6676_CLKSYN_LOGEN
 */

#define RESET_CAL			(1 << 3)

/*
 * AD6676_CLKSYN_VCO_CAL
 */

#define INIT_ALC_VALUE(x)		((x) << 4)
#define ALC_DIS				(1 << 3)

/*
 * AD6676_CLKSYN_CP_CAL
 */
#define CP_CAL_EN			(1 << 7)

/*
 * AD6676_DEC_MODE
 */

#define DEC_32				1
#define DEC_24				2
#define DEC_16				3
#define DEC_12				4

/*
 * AD6676_CAL_CMD
 */

#define XCMD3				(1 << 7)
#define XCMD2				(1 << 6)
#define XCMD1				(1 << 5)
#define XCMD0				(1 << 4)
#define RESON1_CAL      		(1 << 3)
#define FLASH_CAL       		(1 << 2)
#define INIT_ADC        		(1 << 1)
#define TUNE_ADC			(1 << 0)

/*
 * AD6676_SYNCB_CTRL
 */
#define PD_SYSREF_RX			(1 << 3)
#define LVDS_SYNCB			(1 << 2)

/* AD6676_L */
#define SCR				(1 << 7)

/* AD6676_FORCE_END_CAL */
#define FORCE_END_CAL			(1 << 0)

/* AD6676_CAL_DONE */
#define CAL_DONE			(1 << 0)

#define MHz 				1000000UL
#define MIN_FADC			2000000000ULL /* SPS */
#define MIN_FADC_INT_SYNTH		2925000000ULL /* SPS REVISIT */
#define MAX_FADC			3200000000ULL /* SPS */

#define MIN_FIF				70000000ULL /* Hz */
#define MAX_FIF				450000000ULL /* Hz */

#define MIN_BW				20000000ULL /* Hz */
#define MAX_BW				160000000ULL /* Hz */

#define CHIP_ID1_AD6676			0x03
#define CHIP_ID0_AD6676			0xBB

/***************************************************************************//**
 * @brief The `ad6676_init_param` structure is used to initialize and configure
 * the AD6676 device, a wideband IF receiver. It contains various
 * parameters such as clock rates, bandwidth settings, decimation
 * factors, and JESD204B interface configurations. This structure allows
 * for detailed customization of the device's operation, including
 * enabling or disabling features like external clock usage, SPI
 * interface configuration, and shuffler control. Additionally, it
 * includes SPI initialization parameters to facilitate communication
 * with the device.
 *
 * @param ref_clk Reference clock rate in Hz.
 * @param f_adc_hz ADC frequency in Hz.
 * @param f_if_hz Intermediate frequency in Hz.
 * @param bw_hz Bandwidth in Hz.
 * @param bw_margin_low_mhz Lower bandwidth margin in MHz.
 * @param bw_margin_high_mhz Higher bandwidth margin in MHz.
 * @param bw_margin_if_mhz Intermediate frequency bandwidth margin in MHz.
 * @param decimation Decimation factor.
 * @param ext_l External inductance in nH.
 * @param attenuation Attenuation level.
 * @param scale Fullscale adjustment.
 * @param use_extclk Flag to enable external clock.
 * @param spi3wire Flag to set SPI interface to 3 or 4 wires.
 * @param shuffle_ctrl Shuffler control setting.
 * @param shuffle_thresh Shuffler threshold setting.
 * @param scrambling_en Flag to enable JESD scrambling.
 * @param lvds_syncb Flag to enable JESD LVDS SYNCB.
 * @param sysref_pd Flag to enable JESD powerdown SYSREF.
 * @param n_lanes Number of JESD lanes.
 * @param frames_per_multiframe Number of frames per multiframe.
 * @param m JESD parameter M.
 * @param spi_init SPI initialization parameters.
 ******************************************************************************/
struct ad6676_init_param {
	uint32_t 	ref_clk; // reference_clk rate Hz
	uint32_t	f_adc_hz; // adc frequency Hz
	uint32_t	f_if_hz; // intermediate frequency hz
	uint32_t	bw_hz; // bandwidth Hz;
	uint8_t		bw_margin_low_mhz;
	uint8_t		bw_margin_high_mhz;
	int8_t		bw_margin_if_mhz;
	uint8_t		decimation; // decimation
	uint8_t		ext_l; // external inductance l_nh
	uint8_t		attenuation; //
	uint8_t		scale; // fullscale adjust
	uint8_t		use_extclk; // use external clk enable
	uint8_t		spi3wire; // set device spi intereface 3/4 wires
	// shuffle
	uint8_t		shuffle_ctrl; // shuffler control
	uint8_t		shuffle_thresh; // shuffler threshold
	// jesd
	uint8_t 	scrambling_en; // jesd_scrambling_enable
	uint8_t 	lvds_syncb; // jesd_use_lvds_syncb_enable
	uint8_t 	sysref_pd; // jesd_powerdown_sysref_enable
	uint8_t 	n_lanes; // lanes
	uint8_t 	frames_per_multiframe;
	uint64_t	m;
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/***************************************************************************//**
 * @brief The `ad6676_dev` structure is a simple data structure designed to
 * encapsulate the SPI communication descriptor for the AD6676 device. It
 * contains a single member, `spi_desc`, which is a pointer to a
 * `no_os_spi_desc` structure. This structure is essential for managing
 * the SPI interface, which is crucial for configuring and controlling
 * the AD6676 device, a high-speed analog-to-digital converter. The
 * `ad6676_dev` structure is used in various functions to perform
 * operations such as reading from and writing to the device, setting up
 * the device, and updating its configuration.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct ad6676_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI read from device. */
/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD6676 device using SPI communication. It is essential to ensure that
 * the device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. The read value is stored in the
 * provided memory location pointed to by reg_data. This function is
 * typically used when there is a need to retrieve configuration or
 * status information from the device.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. Must
 * not be null, and the device must be properly initialized.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address for the AD6676 device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad6676_spi_read(struct ad6676_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);

/* SPI write to device. */
/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specified
 * register on the AD6676 device using the SPI interface. It is typically
 * called when there is a need to configure or modify the settings of the
 * device by writing to its registers. The function requires a valid
 * device structure that has been properly initialized and configured for
 * SPI communication. It is important to ensure that the register address
 * is within the valid range for the device. The function returns an
 * integer status code indicating the success or failure of the write
 * operation.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. This
 * must be a valid, initialized device structure with an SPI
 * descriptor. Must not be null.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. The address should be within the valid range of the
 * device's register map.
 * @param reg_data The 8-bit data to be written to the specified register. This
 * is the value that will be stored in the register.
 * @return Returns an int32_t status code from the SPI write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t ad6676_spi_write(struct ad6676_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the AD6676 device by initializing it with the
 * provided configuration parameters. It must be called before any other
 * operations on the device to ensure proper setup. The function
 * configures the device's SPI interface, checks the device ID, and sets
 * various operational parameters such as clock source, frequency,
 * bandwidth, and decimation. It also performs necessary calibrations and
 * checks for PLL lock status. If the setup is successful, the function
 * returns a non-negative value and provides a pointer to the initialized
 * device structure. If any step fails, it returns a negative error code.
 *
 * @param device A pointer to a pointer of type `struct ad6676_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A structure of type `struct ad6676_init_param` containing
 * initialization parameters such as reference clock rate, ADC
 * frequency, bandwidth, and other configuration settings. The
 * values must be within valid ranges as specified in the
 * structure definition.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t ad6676_setup(struct ad6676_dev **device,
		     struct ad6676_init_param init_param);

/* Reconfigure device for other target frequency and bandwidth and
 * recalibrate. */
/***************************************************************************//**
 * @brief This function is used to update the configuration of an AD6676 device
 * with new target frequency and bandwidth parameters, followed by a
 * recalibration process. It should be called whenever there is a need to
 * change the operational parameters of the device after it has been
 * initialized. The function ensures that the input parameters are within
 * valid ranges by clamping them to acceptable values. It is important to
 * ensure that the device has been properly initialized before calling
 * this function. The function returns an error code if any of the
 * operations fail, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device to be
 * updated. Must not be null, and the device must be initialized
 * prior to calling this function.
 * @param init_param A pointer to an ad6676_init_param structure containing the
 * new configuration parameters. The structure's fields will
 * be clamped to valid ranges where necessary. Must not be
 * null.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error occurred during the update or
 * calibration process.
 ******************************************************************************/
int32_t ad6676_update(struct ad6676_dev *dev,
		      struct ad6676_init_param *init_param);

/* Set attenuation in decibels or disable attenuator. */
/***************************************************************************//**
 * @brief This function configures the attenuation level of the AD6676 device by
 * writing the specified attenuation value to the device's registers. It
 * should be called when you need to adjust the signal attenuation for
 * the device. The function ensures that the attenuation value is clamped
 * within the valid range of 0 to 27 dB before applying it. This function
 * must be called with a valid device structure and initialization
 * parameters that have been properly set up. It does not handle invalid
 * device pointers or uninitialized parameters.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param init_param A pointer to an ad6676_init_param structure containing the
 * initialization parameters, including the desired
 * attenuation level. The attenuation value will be clamped
 * between 0 and 27 dB. Must not be null. The caller retains
 * ownership.
 * @return Returns 0 on success. The attenuation value in init_param is clamped
 * and written to the device.
 ******************************************************************************/
int32_t ad6676_set_attenuation(struct ad6676_dev *dev,
			       struct ad6676_init_param *init_param);

/* Set the target IF frequency. */
/***************************************************************************//**
 * @brief This function configures the target intermediate frequency (IF) for
 * the AD6676 device using the specified initialization parameters. It
 * should be called when you need to set or update the IF frequency of
 * the device. The function ensures that the provided IF frequency is
 * clamped within the valid range before applying it. It is important to
 * ensure that the device has been properly initialized before calling
 * this function to avoid undefined behavior.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param init_param A pointer to an ad6676_init_param structure containing the
 * desired IF frequency in the f_if_hz field. The frequency is
 * clamped between MIN_FIF and MAX_FIF before being set. Must
 * not be null.
 * @return Returns an int32_t indicating success or failure of the operation. A
 * non-zero value indicates an error.
 ******************************************************************************/
int32_t ad6676_set_fif(struct ad6676_dev *dev,
		       struct ad6676_init_param *init_param);

/* Get the target IF frequency. */
/***************************************************************************//**
 * @brief This function retrieves the current target intermediate frequency (IF)
 * for the AD6676 device by reading specific tuning registers and
 * performing calculations based on the provided initialization
 * parameters. It is typically used to verify or obtain the current IF
 * setting after the device has been configured. The function requires a
 * valid device structure and initialization parameters to perform the
 * calculation correctly.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param init_param A pointer to an ad6676_init_param structure containing
 * initialization parameters, including the ADC frequency and
 * a divisor. Must not be null. The caller retains ownership.
 * @return Returns the calculated target IF frequency as a 64-bit unsigned
 * integer.
 ******************************************************************************/
uint64_t ad6676_get_fif(struct ad6676_dev *dev,
			struct ad6676_init_param *init_param);

/* Perform an interface test. */
/***************************************************************************//**
 * @brief This function is used to perform an interface test on the AD6676
 * device by setting it to a specified test mode. It is typically used to
 * verify the communication and functionality of the device during
 * development or troubleshooting. The function requires a valid device
 * structure and a test mode value, which determines the type of test to
 * be performed. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad6676_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * use.
 * @param test_mode A 32-bit unsigned integer specifying the test mode to set on
 * the device. Valid values are defined by the device's test
 * mode specifications.
 * @return Returns 0 on success, indicating the test mode was set successfully.
 ******************************************************************************/
int32_t ad6676_test(struct ad6676_dev *dev,
		    uint32_t test_mode);
#endif

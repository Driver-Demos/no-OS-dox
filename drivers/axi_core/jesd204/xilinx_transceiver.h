/***************************************************************************//**
 *   @file   xilinx_transceiver.h
 *   @brief  Driver for the Xilinx High-speed transceiver dynamic
 *           reconfiguration.
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
#ifndef XILINX_TRANSCEIVER_H_
#define XILINX_TRANSCEIVER_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/************************ Macros and Types Declarations ***********************/
/******************************************************************************/
#define AXI_PCORE_VER(major, minor, letter)	((major << 16) | (minor << 8) | letter)
#define AXI_PCORE_VER_MAJOR(version)	(((version) >> 16) & 0xff)
#define AXI_PCORE_VER_MINOR(version)	((version >> 8) & 0xff)
#define AXI_PCORE_VER_LETTER(version)	(version & 0xff)

#define	AXI_REG_VERSION			0x0000
#define AXI_VERSION(x)			(((x) & 0xffffffff) << 0)
#define AXI_VERSION_IS(x, y, z)		((x) << 16 | (y) << 8 | (z))
#define AXI_VERSION_MAJOR(x)		((x) >> 16)

#define AXI_REG_FPGA_INFO		0x001C
#define AXI_REG_FPGA_VOLTAGE		0x0140

#define AXI_INFO_FPGA_TECH(info)        ((info) >> 24)
#define AXI_INFO_FPGA_FAMILY(info)      (((info) >> 16) & 0xff)
#define AXI_INFO_FPGA_SPEED_GRADE(info)	(((info) >> 8) & 0xff)
#define AXI_INFO_FPGA_DEV_PACKAGE(info)	((info) & 0xff)
#define AXI_INFO_FPGA_VOLTAGE(val)      ((val) & 0xffff)

/***************************************************************************//**
 * @brief The `xilinx_xcvr_type` enumeration defines a set of constants
 * representing different types of Xilinx high-speed transceivers. Each
 * enumerator corresponds to a specific transceiver type, identified by a
 * unique integer value, which is used to distinguish between various
 * generations and models of Xilinx transceivers, such as the 7 Series
 * GTX2 and UltraScale GTH3, GTH4, and GTY4.
 *
 * @param XILINX_XCVR_TYPE_S7_GTX2 Represents the Xilinx 7 Series GTX2
 * transceiver type with a value of 2.
 * @param XILINX_XCVR_TYPE_US_GTH3 Represents the Xilinx UltraScale GTH3
 * transceiver type with a value of 5.
 * @param XILINX_XCVR_TYPE_US_GTH4 Represents the Xilinx UltraScale GTH4
 * transceiver type with a value of 8.
 * @param XILINX_XCVR_TYPE_US_GTY4 Represents the Xilinx UltraScale GTY4
 * transceiver type with a value of 9.
 ******************************************************************************/
enum xilinx_xcvr_type {
	XILINX_XCVR_TYPE_S7_GTX2 = 2,
	XILINX_XCVR_TYPE_US_GTH3 = 5,
	XILINX_XCVR_TYPE_US_GTH4 = 8,
	XILINX_XCVR_TYPE_US_GTY4 = 9,
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr_legacy_type` is an enumeration that defines legacy
 * types for Xilinx high-speed transceivers. It includes identifiers for
 * different generations and models of transceivers, such as Series 7
 * GTX2 and UltraScale GTH3, GTH4, and GTY4. This enum is used to
 * categorize and manage different legacy transceiver types within the
 * Xilinx transceiver driver, facilitating compatibility and
 * configuration management for older transceiver models.
 *
 * @param XILINX_XCVR_LEGACY_TYPE_S7_GTX2 Represents the legacy type for the
 * Series 7 GTX2 transceiver.
 * @param XILINX_XCVR_LEGACY_TYPE_US_GTH3 Represents the legacy type for the
 * UltraScale GTH3 transceiver.
 * @param XILINX_XCVR_LEGACY_TYPE_US_GTH4 Represents the legacy type for the
 * UltraScale GTH4 transceiver.
 * @param XILINX_XCVR_LEGACY_TYPE_US_GTY4 Represents the legacy type for the
 * UltraScale GTY4 transceiver,
 * explicitly set to 4.
 ******************************************************************************/
enum xilinx_xcvr_legacy_type {
	XILINX_XCVR_LEGACY_TYPE_S7_GTX2,
	XILINX_XCVR_LEGACY_TYPE_US_GTH3,
	XILINX_XCVR_LEGACY_TYPE_US_GTH4,
	XILINX_XCVR_LEGACY_TYPE_US_GTY4 = 4,
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr_refclk_ppm` enumeration defines a set of constants
 * representing different levels of parts per million (ppm) deviation for
 * a reference clock in a Xilinx transceiver. This is used to specify the
 * allowable frequency deviation of the reference clock, which is
 * critical for maintaining the accuracy and stability of high-speed data
 * transmission in Xilinx transceivers.
 *
 * @param PM_200 Represents a reference clock with a parts per million (ppm)
 * deviation of 200.
 * @param PM_700 Represents a reference clock with a parts per million (ppm)
 * deviation of 700.
 * @param PM_1250 Represents a reference clock with a parts per million (ppm)
 * deviation of 1250.
 ******************************************************************************/
enum xilinx_xcvr_refclk_ppm {
	PM_200,
	PM_700,
	PM_1250,
};

/***************************************************************************//**
 * @brief The `axi_fgpa_technology` enumeration defines a set of constants
 * representing different generations or technologies of FPGA devices. It
 * is used to categorize FPGA devices into specific technology types,
 * such as Series 7, UltraScale, and UltraScale+, with an additional
 * option for unknown technology. This enumeration is useful for software
 * that needs to handle different FPGA technologies differently, allowing
 * for technology-specific optimizations or configurations.
 *
 * @param AXI_FPGA_TECH_UNKNOWN Represents an unknown FPGA technology.
 * @param AXI_FPGA_TECH_SERIES7 Represents the Series 7 FPGA technology.
 * @param AXI_FPGA_TECH_ULTRASCALE Represents the UltraScale FPGA technology.
 * @param AXI_FPGA_TECH_ULTRASCALE_PLUS Represents the UltraScale+ FPGA
 * technology.
 ******************************************************************************/
enum axi_fgpa_technology {
	AXI_FPGA_TECH_UNKNOWN = 0,
	AXI_FPGA_TECH_SERIES7,
	AXI_FPGA_TECH_ULTRASCALE,
	AXI_FPGA_TECH_ULTRASCALE_PLUS,
};

/***************************************************************************//**
 * @brief The `axi_fpga_family` enumeration defines a set of constants
 * representing different families of Xilinx FPGAs. Each enumerator
 * corresponds to a specific FPGA family, such as Artix, Kintex, Virtex,
 * and Zynq, with an additional option for unknown family types. This
 * enumeration is used to categorize and identify the family variant of
 * an FPGA device within the context of the Xilinx transceiver driver.
 *
 * @param AXI_FPGA_FAMILY_UNKNOWN Represents an unknown FPGA family.
 * @param AXI_FPGA_FAMILY_ARTIX Represents the Artix family of FPGAs.
 * @param AXI_FPGA_FAMILY_KINTEX Represents the Kintex family of FPGAs.
 * @param AXI_FPGA_FAMILY_VIRTEX Represents the Virtex family of FPGAs.
 * @param AXI_FPGA_FAMILY_ZYNQ Represents the Zynq family of FPGAs.
 ******************************************************************************/
enum axi_fpga_family {
	AXI_FPGA_FAMILY_UNKNOWN = 0,
	AXI_FPGA_FAMILY_ARTIX,
	AXI_FPGA_FAMILY_KINTEX,
	AXI_FPGA_FAMILY_VIRTEX,
	AXI_FPGA_FAMILY_ZYNQ,
};

/***************************************************************************//**
 * @brief The `axi_fpga_speed_grade` enumeration defines various speed grades
 * for an FPGA, each associated with a specific integer value. These
 * speed grades are used to categorize the performance capabilities of
 * FPGA devices, ranging from unknown to specific grades like 1, 1L, 1H,
 * 1HV, 1LV, 2, 2L, 2LV, and 3. This enumeration is useful for
 * identifying and managing FPGA configurations based on their speed
 * capabilities.
 *
 * @param AXI_FPGA_SPEED_UNKNOWN Represents an unknown FPGA speed grade with a
 * value of 0.
 * @param AXI_FPGA_SPEED_1 Represents FPGA speed grade 1 with a value of 10.
 * @param AXI_FPGA_SPEED_1L Represents FPGA speed grade 1L with a value of 11.
 * @param AXI_FPGA_SPEED_1H Represents FPGA speed grade 1H with a value of 12.
 * @param AXI_FPGA_SPEED_1HV Represents FPGA speed grade 1HV with a value of 13.
 * @param AXI_FPGA_SPEED_1LV Represents FPGA speed grade 1LV with a value of 14.
 * @param AXI_FPGA_SPEED_2 Represents FPGA speed grade 2 with a value of 20.
 * @param AXI_FPGA_SPEED_2L Represents FPGA speed grade 2L with a value of 21.
 * @param AXI_FPGA_SPEED_2LV Represents FPGA speed grade 2LV with a value of 22.
 * @param AXI_FPGA_SPEED_3 Represents FPGA speed grade 3 with a value of 30.
 ******************************************************************************/
enum axi_fpga_speed_grade {
	AXI_FPGA_SPEED_UNKNOWN	= 0,
	AXI_FPGA_SPEED_1	= 10,
	AXI_FPGA_SPEED_1L	= 11,
	AXI_FPGA_SPEED_1H	= 12,
	AXI_FPGA_SPEED_1HV	= 13,
	AXI_FPGA_SPEED_1LV	= 14,
	AXI_FPGA_SPEED_2	= 20,
	AXI_FPGA_SPEED_2L	= 21,
	AXI_FPGA_SPEED_2LV	= 22,
	AXI_FPGA_SPEED_3	= 30,
};

/***************************************************************************//**
 * @brief The `axi_fpga_dev_pack` is an enumeration that defines various FPGA
 * device package types. Each enumerator represents a specific package
 * type, which is used to identify the physical packaging of an FPGA
 * device. This enumeration is useful in contexts where the package type
 * needs to be specified or checked, such as in configuration or
 * initialization routines for FPGA-based systems.
 *
 * @param AXI_FPGA_DEV_UNKNOWN Represents an unknown FPGA device package.
 * @param AXI_FPGA_DEV_RF Represents the RF device package.
 * @param AXI_FPGA_DEV_FL Represents the FL device package.
 * @param AXI_FPGA_DEV_FF Represents the FF device package.
 * @param AXI_FPGA_DEV_FB Represents the FB device package.
 * @param AXI_FPGA_DEV_HC Represents the HC device package.
 * @param AXI_FPGA_DEV_FH Represents the FH device package.
 * @param AXI_FPGA_DEV_CS Represents the CS device package.
 * @param AXI_FPGA_DEV_CP Represents the CP device package.
 * @param AXI_FPGA_DEV_FT Represents the FT device package.
 * @param AXI_FPGA_DEV_FG Represents the FG device package.
 * @param AXI_FPGA_DEV_SB Represents the SB device package.
 * @param AXI_FPGA_DEV_RB Represents the RB device package.
 * @param AXI_FPGA_DEV_RS Represents the RS device package.
 * @param AXI_FPGA_DEV_CL Represents the CL device package.
 * @param AXI_FPGA_DEV_SF Represents the SF device package.
 * @param AXI_FPGA_DEV_BA Represents the BA device package.
 * @param AXI_FPGA_DEV_FA Represents the FA device package.
 ******************************************************************************/
enum axi_fpga_dev_pack {
	AXI_FPGA_DEV_UNKNOWN = 0,
	AXI_FPGA_DEV_RF,
	AXI_FPGA_DEV_FL,
	AXI_FPGA_DEV_FF,
	AXI_FPGA_DEV_FB,
	AXI_FPGA_DEV_HC,
	AXI_FPGA_DEV_FH,
	AXI_FPGA_DEV_CS,
	AXI_FPGA_DEV_CP,
	AXI_FPGA_DEV_FT,
	AXI_FPGA_DEV_FG,
	AXI_FPGA_DEV_SB,
	AXI_FPGA_DEV_RB,
	AXI_FPGA_DEV_RS,
	AXI_FPGA_DEV_CL,
	AXI_FPGA_DEV_SF,
	AXI_FPGA_DEV_BA,
	AXI_FPGA_DEV_FA,
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr` structure is designed to encapsulate the
 * configuration parameters for a Xilinx high-speed transceiver,
 * facilitating dynamic reconfiguration. It includes fields for
 * specifying the transceiver type, reference clock deviation, encoding
 * scheme, and associated adxcvr structure. Additionally, it holds
 * information about the FPGA's version, technology, family, speed grade,
 * device package, and operating voltage. The structure also defines the
 * nominal operating frequency ranges for the CPLL/QPLL voltage-
 * controlled oscillators (VCOs), which are critical for ensuring the
 * transceiver operates within its specified limits.
 *
 * @param type Specifies the type of Xilinx transceiver.
 * @param refclk_ppm Indicates the reference clock parts per million (ppm)
 * deviation.
 * @param encoding Defines the encoding scheme used, such as 8B10B or 66B64B.
 * @param ad_xcvr Pointer to an associated adxcvr structure.
 * @param version Holds the version information of the transceiver.
 * @param tech Specifies the FPGA technology or generation.
 * @param family Indicates the family variant of the FPGA device.
 * @param speed_grade Represents the speed grade of the FPGA.
 * @param dev_package Specifies the device package type.
 * @param voltage Indicates the operating voltage of the FPGA.
 * @param vco0_min Minimum frequency for VCO0 in kHz.
 * @param vco0_max Maximum frequency for VCO0 in kHz.
 * @param vco1_min Minimum frequency for VCO1 in kHz.
 * @param vco1_max Maximum frequency for VCO1 in kHz.
 ******************************************************************************/
struct xilinx_xcvr {
	enum xilinx_xcvr_type type;
	enum xilinx_xcvr_refclk_ppm refclk_ppm;
	uint32_t encoding;
	struct adxcvr *ad_xcvr;
	uint32_t version;
	enum axi_fgpa_technology tech;
	enum axi_fpga_family family;
	enum axi_fpga_speed_grade speed_grade;
	enum axi_fpga_dev_pack dev_package;
	uint32_t voltage;

	// CPLL / QPLL nominal operating ranges
	uint32_t vco0_min; // kHz
	uint32_t vco0_max; // kHz
	uint32_t vco1_min; // kHz
	uint32_t vco1_max; // kHz
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr_drp_ops` structure defines a set of operations for
 * dynamic reconfiguration of Xilinx high-speed transceivers,
 * specifically focusing on read and write operations to the DRP (Dynamic
 * Reconfiguration Port). It contains two function pointers, `write` and
 * `read`, which are used to perform write and read operations on the
 * transceiver's DRP, allowing for configuration and data retrieval from
 * specific registers within the transceiver.
 *
 * @param write A function pointer for writing data to a specified DRP port and
 * register.
 * @param read A function pointer for reading data from a specified DRP port and
 * register.
 ******************************************************************************/
struct xilinx_xcvr_drp_ops {
	int (*write)(struct adxcvr *xcvr, unsigned int drp_port,
		     unsigned int reg, unsigned int val);
	int (*read)(struct adxcvr *xcvr, unsigned int drp_port,
		    unsigned int reg, unsigned int *val);
};

/***************************************************************************//**
 * @brief The `clk_ops` structure defines a set of callback operations for
 * managing hardware clocks, intended to be implemented by clock drivers.
 * It includes function pointers for enabling and disabling clocks
 * atomically, recalculating and rounding clock rates, and setting new
 * clock rates. These operations are crucial for ensuring that clocks are
 * managed efficiently and correctly, particularly in systems where
 * clocks need to be adjusted dynamically based on hardware requirements.
 * The structure is designed to be used in conjunction with the `clk_*`
 * API, allowing for a separation of concerns between clock management
 * and clock usage in drivers.
 *
 * @param enable A function pointer to enable the clock atomically without
 * sleeping.
 * @param disable A function pointer to disable the clock atomically without
 * sleeping.
 * @param recalc_rate A function pointer to recalculate the clock rate by
 * querying hardware, using the parent rate as input.
 * @param round_rate A function pointer to determine the closest supported rate
 * to a target rate, with the parent rate as an input/output
 * parameter.
 * @param set_rate A function pointer to change the clock rate, using the target
 * rate and parent rate as inputs.
 ******************************************************************************/
struct clk_ops {
	int (*enable)(struct adxcvr *xcvr);
	int (*disable)(struct adxcvr *xcvr);
	unsigned long (*recalc_rate)(struct adxcvr *xcvr,
				     unsigned long parent_rate);
	long (*round_rate)(struct adxcvr *xcvr,
			   unsigned long rate,
			   unsigned long parent_rate);
	int (*set_rate)(struct adxcvr *xcvr,
			unsigned long rate,
			unsigned long parent_rate);
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr_cpll_config` structure is used to hold configuration
 * parameters for the Channel Phase-Locked Loop (CPLL) in Xilinx high-
 * speed transceivers. It includes fields for setting the reference clock
 * division and two feedback division factors, which are critical for
 * determining the operating frequency and stability of the CPLL. This
 * configuration is essential for ensuring that the transceiver operates
 * correctly at the desired data rates.
 *
 * @param refclk_div Specifies the reference clock division factor.
 * @param fb_div_N1 Specifies the first feedback division factor.
 * @param fb_div_N2 Specifies the second feedback division factor.
 ******************************************************************************/
struct xilinx_xcvr_cpll_config {
	uint32_t refclk_div;
	uint32_t fb_div_N1;
	uint32_t fb_div_N2;
};

/***************************************************************************//**
 * @brief The `xilinx_xcvr_qpll_config` structure is used to configure the Quad
 * PLL (QPLL) settings for a Xilinx high-speed transceiver. It includes
 * parameters for reference clock division, feedback division, band
 * selection, and full rate configuration, which are essential for
 * setting up the QPLL to achieve the desired operational characteristics
 * in high-speed data transmission applications.
 *
 * @param refclk_div Specifies the reference clock division factor.
 * @param fb_div Specifies the feedback division factor.
 * @param band Indicates the band setting for the QPLL.
 * @param qty4_full_rate Represents the full rate setting for the QPLL.
 ******************************************************************************/
struct xilinx_xcvr_qpll_config {
	uint32_t refclk_div;
	uint32_t fb_div;
	uint32_t band;
	uint32_t qty4_full_rate;
};

/* Encoding */
#define ENC_8B10B		810
#define ENC_66B64B		6664

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to configure the Clock Data Recovery (CDR)
 * circuit of a Xilinx transceiver, which is essential for proper data
 * synchronization and recovery in high-speed communication systems. It
 * should be called when setting up or modifying the CDR settings for a
 * specific transceiver type. The function requires a valid transceiver
 * structure and specific configuration parameters. It returns an error
 * code if the transceiver type is unsupported or if any configuration
 * parameter is invalid.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver
 * to be configured. Must not be null and should be properly
 * initialized with the transceiver type and other relevant
 * parameters.
 * @param drp_port A `uint32_t` representing the DRP port number to be used for
 * configuration. Must be a valid port number for the given
 * transceiver.
 * @param lane_rate A `uint32_t` specifying the lane rate in Hz. This parameter
 * is used for certain transceiver types and should be within
 * the supported range for the transceiver.
 * @param out_div A `uint32_t` representing the output divider value. This
 * parameter is used to set the output clock division and must be
 * valid for the transceiver type.
 * @param lpm_enable A `bool` indicating whether Low Power Mode (LPM) is
 * enabled. This parameter is applicable for certain
 * transceiver types and affects power consumption and
 * performance.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the transceiver type is unsupported or if any configuration
 * parameter is invalid.
 ******************************************************************************/
int xilinx_xcvr_configure_cdr(struct xilinx_xcvr *xcvr,
			      uint32_t drp_port, uint32_t lane_rate, uint32_t out_div,
			      bool lpm_enable);
/***************************************************************************//**
 * @brief This function configures a Xilinx transceiver to operate in either
 * Low-Power mode (LPM) or Decision Feedback Equalization (DFE) mode. It
 * should be used when there is a need to switch the operational mode of
 * the transceiver based on performance or power requirements. The
 * function is applicable to specific transceiver types, and it is
 * important to ensure that the transceiver type supports the desired
 * mode. The function must be called with a valid transceiver structure
 * and a designated DRP port.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver
 * to be configured. Must not be null and should be properly
 * initialized with the transceiver type and other relevant
 * parameters.
 * @param drp_port A `uint32_t` representing the DRP port number to be used for
 * configuration. It should be a valid port number for the given
 * transceiver.
 * @param lpm A `bool` indicating the desired mode: `true` for Low-Power mode
 * (LPM) and `false` for Decision Feedback Equalization (DFE).
 * @return Returns 0 on success, indicating the mode was set without errors.
 ******************************************************************************/
int xilinx_xcvr_configure_lpm_dfe_mode(struct xilinx_xcvr *xcvr,
				       uint32_t drp_port, bool lpm);


/***************************************************************************//**
 * @brief This function calculates the necessary configuration parameters for
 * the Channel PLL (CPLL) of a Xilinx transceiver based on the provided
 * reference clock and lane rate frequencies. It should be used when
 * setting up or adjusting the CPLL to achieve a desired lane rate. The
 * function requires a valid `xilinx_xcvr` structure and expects the
 * reference clock and lane rate to be specified in kilohertz. If
 * successful, it populates the provided configuration structure and
 * output divider with the calculated values. The function returns an
 * error code if it fails to find a suitable configuration.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null.
 * @param refclk_khz The reference clock frequency in kilohertz. Must be a
 * positive integer.
 * @param lane_rate_khz The desired lane rate frequency in kilohertz. Must be a
 * positive integer.
 * @param conf A pointer to a `struct xilinx_xcvr_cpll_config` where the
 * calculated CPLL configuration will be stored. Can be null if
 * configuration data is not needed.
 * @param out_div A pointer to a `uint32_t` where the calculated output divider
 * will be stored. Can be null if the output divider is not
 * needed.
 * @return Returns 0 on success, or a negative error code if a suitable
 * configuration cannot be found.
 ******************************************************************************/
int xilinx_xcvr_calc_cpll_config(struct xilinx_xcvr *xcvr,
				 uint32_t refclk_khz, uint32_t lane_rate_khz,
				 struct xilinx_xcvr_cpll_config *conf, uint32_t *out_div);
/***************************************************************************//**
 * @brief This function retrieves the current Channel PLL (CPLL) configuration
 * for a given Xilinx transceiver. It is used to obtain the configuration
 * parameters of the CPLL, which are necessary for understanding or
 * modifying the transceiver's operation. The function must be called
 * with a valid transceiver structure and is specific to certain
 * transceiver types. If the transceiver type is unsupported, the
 * function returns an error. This function is typically used in
 * scenarios where the current CPLL settings need to be read for
 * diagnostics or further configuration.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * This must not be null and should be properly initialized with a
 * supported transceiver type.
 * @param drp_port A `uint32_t` representing the DRP port number. It specifies
 * which port's configuration to read and must be a valid port
 * number for the transceiver.
 * @param conf A pointer to a `struct xilinx_xcvr_cpll_config` where the CPLL
 * configuration will be stored. This must not be null and should
 * point to a valid memory location.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if any other error occurs.
 ******************************************************************************/
int xilinx_xcvr_cpll_read_config(struct xilinx_xcvr *xcvr,
				 uint32_t drp_port, struct xilinx_xcvr_cpll_config *conf);
/***************************************************************************//**
 * @brief This function is used to configure the Channel PLL (CPLL) of a Xilinx
 * transceiver with the provided settings. It should be called when you
 * need to apply a new CPLL configuration to a transceiver of a supported
 * type. The function requires a valid transceiver structure and
 * configuration settings. It returns an error if the transceiver type is
 * not supported.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver
 * to be configured. Must not be null and should be properly
 * initialized with a supported type.
 * @param drp_port A `uint32_t` representing the DRP port number to which the
 * configuration will be applied. Must be a valid port number
 * for the transceiver.
 * @param conf A pointer to a `struct xilinx_xcvr_cpll_config` containing the
 * desired CPLL configuration settings. Must not be null and should
 * be properly initialized with valid configuration values.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is not supported or if any other error occurs.
 ******************************************************************************/
int xilinx_xcvr_cpll_write_config(struct xilinx_xcvr *xcvr,
				  uint32_t drp_port, const struct xilinx_xcvr_cpll_config *conf);
/***************************************************************************//**
 * @brief This function calculates the lane rate for a Channel PLL based on the
 * provided reference clock frequency, CPLL configuration, and output
 * divider. It is used to determine the effective data rate of a
 * transceiver lane. The function requires valid configuration parameters
 * and non-zero divider values to perform the calculation. If any of the
 * dividers are zero, the function returns zero, indicating an invalid
 * configuration.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver
 * instance. The caller retains ownership and it must not be null.
 * @param refclk_hz The reference clock frequency in hertz. It must be a
 * positive integer.
 * @param conf A pointer to a `struct xilinx_xcvr_cpll_config` containing the
 * CPLL configuration parameters. The caller retains ownership and
 * it must not be null. The `refclk_div` field must be non-zero.
 * @param out_div The output divider value. It must be a non-zero positive
 * integer.
 * @return Returns the calculated lane rate in kilohertz as an integer. Returns
 * zero if the configuration is invalid due to zero dividers.
 ******************************************************************************/
int xilinx_xcvr_cpll_calc_lane_rate(struct xilinx_xcvr *xcvr,
				    uint32_t refclk_hz, const struct xilinx_xcvr_cpll_config *conf,
				    uint32_t out_div);

/***************************************************************************//**
 * @brief This function calculates the necessary configuration parameters for a
 * Xilinx Quad PLL based on the provided system clock selection,
 * reference clock frequency, and desired lane rate. It is used to
 * determine the appropriate settings for the PLL to achieve the
 * specified lane rate. The function must be called with a valid
 * transceiver structure and is expected to fill the provided
 * configuration structure and output divider if a valid configuration is
 * found. If the configuration cannot be determined, the function returns
 * an error code.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null and should be properly initialized with the
 * transceiver type and VCO frequency ranges.
 * @param sys_clk_sel A `uint32_t` representing the system clock selection.
 * Valid values depend on the specific transceiver
 * configuration.
 * @param refclk_khz A `uint32_t` representing the reference clock frequency in
 * kHz. Must be a positive value.
 * @param lane_rate_khz A `uint32_t` representing the desired lane rate in kHz.
 * Must be a positive value.
 * @param conf A pointer to a `struct xilinx_xcvr_qpll_config` where the
 * calculated configuration will be stored. Can be null if the
 * configuration is not needed.
 * @param out_div A pointer to a `uint32_t` where the output divider value will
 * be stored. Can be null if the output divider is not needed.
 * @return Returns 0 on success with the configuration parameters filled in
 * `conf` and `out_div` if provided. Returns a negative error code if
 * the configuration cannot be determined.
 ******************************************************************************/
int xilinx_xcvr_calc_qpll_config(struct xilinx_xcvr *xcvr, uint32_t sys_clk_sel,
				 uint32_t refclk_khz, uint32_t lane_rate_khz,
				 struct xilinx_xcvr_qpll_config *conf, uint32_t *out_div);
/***************************************************************************//**
 * @brief This function retrieves the current configuration of the Quad PLL
 * (QPLL) for a specified Xilinx transceiver. It should be used when you
 * need to obtain the QPLL settings for a transceiver of type S7 GTX2, US
 * GTH3, US GTH4, or US GTY4. The function requires a valid transceiver
 * structure and configuration structure to store the results. It returns
 * an error code if the transceiver type is unsupported or if any other
 * error occurs during the read operation.
 *
 * @param xcvr A pointer to a struct xilinx_xcvr representing the transceiver.
 * Must not be null and should be properly initialized with the
 * transceiver type.
 * @param drp_port A uint32_t representing the DRP port number. It should be a
 * valid port number for the transceiver.
 * @param sys_clk_sel A uint32_t representing the system clock selection. This
 * parameter is used for certain transceiver types and should
 * be set according to the transceiver's requirements.
 * @param conf A pointer to a struct xilinx_xcvr_qpll_config where the QPLL
 * configuration will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if an error occurs.
 ******************************************************************************/
int xilinx_xcvr_qpll_read_config(struct xilinx_xcvr *xcvr,
				 uint32_t drp_port, uint32_t sys_clk_sel, struct xilinx_xcvr_qpll_config *conf);
/***************************************************************************//**
 * @brief This function is used to configure the Quad PLL (QPLL) of a Xilinx
 * transceiver based on the specified configuration parameters. It should
 * be called when you need to set up or modify the QPLL settings for a
 * transceiver, particularly after initializing the transceiver
 * structure. The function requires a valid transceiver type and
 * configuration structure, and it handles different transceiver types by
 * delegating to specific configuration functions. If the transceiver
 * type is not supported, the function returns an error code.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver
 * to be configured. Must not be null and should be properly
 * initialized with a valid transceiver type.
 * @param sys_clk_sel A `uint32_t` value representing the system clock
 * selection. The valid range depends on the specific
 * transceiver type and configuration.
 * @param drp_port A `uint32_t` value indicating the DRP port to be used for
 * configuration. Must be a valid port number for the
 * transceiver.
 * @param conf A pointer to a `struct xilinx_xcvr_qpll_config` containing the
 * desired QPLL configuration parameters. Must not be null and
 * should be filled with valid configuration data.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if any other error occurs.
 ******************************************************************************/
int xilinx_xcvr_qpll_write_config(struct xilinx_xcvr *xcvr,
				  uint32_t sys_clk_sel, uint32_t drp_port,
				  const struct xilinx_xcvr_qpll_config *conf);
/***************************************************************************//**
 * @brief This function calculates the lane rate for a Xilinx transceiver based
 * on the provided reference clock frequency, QPLL configuration, and
 * output divider. It is used when configuring the transceiver to
 * determine the effective lane rate that will be achieved with the given
 * settings. The function requires valid QPLL configuration parameters
 * and non-zero values for the reference clock divider and output
 * divider. If any of these values are zero, the function returns zero,
 * indicating an invalid configuration.
 *
 * @param xcvr A pointer to a struct xilinx_xcvr representing the transceiver.
 * The caller retains ownership and it must not be null.
 * @param refclk_hz The reference clock frequency in hertz. It must be a
 * positive integer.
 * @param conf A pointer to a struct xilinx_xcvr_qpll_config containing the QPLL
 * configuration parameters. The caller retains ownership and it
 * must not be null. The refclk_div field must be non-zero.
 * @param out_div The output divider value. It must be a non-zero positive
 * integer.
 * @return Returns the calculated lane rate in kHz as an integer. Returns 0 if
 * the configuration is invalid (e.g., if refclk_div or out_div is
 * zero).
 ******************************************************************************/
int xilinx_xcvr_qpll_calc_lane_rate(struct xilinx_xcvr *xcvr,
				    uint32_t refclk_hz, const struct xilinx_xcvr_qpll_config *conf,
				    uint32_t out_div);

/***************************************************************************//**
 * @brief This function retrieves the TX and RX output divider values for a
 * specified Xilinx transceiver and DRP port. It is used to obtain the
 * current configuration of the transceiver's output dividers, which are
 * essential for determining the data rate and clock settings. The
 * function must be called with a valid transceiver structure and DRP
 * port. It returns an error code if the transceiver type is unsupported
 * or if any other error occurs during the read operation.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null and should be properly initialized with a
 * supported transceiver type.
 * @param drp_port A `uint32_t` representing the DRP port number. It should be a
 * valid port number for the given transceiver.
 * @param rx_out_div A pointer to a `uint32_t` where the RX output divider value
 * will be stored. Must not be null.
 * @param tx_out_div A pointer to a `uint32_t` where the TX output divider value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if an error occurs during the read operation.
 ******************************************************************************/
int xilinx_xcvr_read_out_div(struct xilinx_xcvr *xcvr, uint32_t drp_port,
			     uint32_t *rx_out_div, uint32_t *tx_out_div);
/***************************************************************************//**
 * @brief This function sets the output divider values for both the transmit
 * (TX) and receive (RX) paths of a Xilinx transceiver. It should be used
 * when configuring the transceiver's output clock dividers to achieve
 * the desired data rate. The function requires a valid `xilinx_xcvr`
 * structure, which must be properly initialized with the transceiver
 * type. The function handles different transceiver types and returns an
 * error if the type is unsupported. It is important to ensure that the
 * `xilinx_xcvr` structure and the divider values are correctly set
 * before calling this function.
 *
 * @param xcvr A pointer to a `xilinx_xcvr` structure representing the
 * transceiver. This must be initialized and must not be null. The
 * structure should contain the transceiver type and other relevant
 * configuration parameters.
 * @param drp_port A 32-bit unsigned integer representing the DRP port number.
 * This specifies which port to configure and must be a valid
 * port number for the transceiver.
 * @param rx_out_div A 32-bit signed integer specifying the RX output divider
 * value. This value should be within the valid range for the
 * specific transceiver type.
 * @param tx_out_div A 32-bit signed integer specifying the TX output divider
 * value. This value should be within the valid range for the
 * specific transceiver type.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if any other error occurs.
 ******************************************************************************/
int xilinx_xcvr_write_out_div(struct xilinx_xcvr *xcvr, uint32_t drp_port,
			      int32_t rx_out_div, int32_t tx_out_div);

/***************************************************************************//**
 * @brief This function sets the RX_CLK25_DIV value for a specified Xilinx
 * transceiver, which is used to configure the clock division for the
 * receiver. It should be called when you need to adjust the clock
 * division settings for a transceiver operating in a specific mode. The
 * function requires a valid transceiver structure and a division value
 * within the range of 1 to 32. If the division value is outside this
 * range or the transceiver type is unsupported, the function returns an
 * error.
 *
 * @param xcvr A pointer to a struct xilinx_xcvr representing the transceiver.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param drp_port A uint32_t representing the DRP port number. It specifies
 * which port to configure, and should be a valid port number
 * for the transceiver.
 * @param div A uint32_t representing the division value. Must be between 1 and
 * 32 inclusive. Values outside this range will result in an error.
 * @return Returns 0 on success. Returns -EINVAL if the division value is out of
 * range or the transceiver type is unsupported.
 ******************************************************************************/
int xilinx_xcvr_write_rx_clk25_div(struct xilinx_xcvr *xcvr,
				   uint32_t drp_port, uint32_t div);
/***************************************************************************//**
 * @brief This function sets the TX_CLK25_DIV value for a specified Xilinx
 * transceiver, which is used to configure the clock division for the
 * transmitter. It should be called when you need to adjust the clock
 * division ratio for the transmitter to match specific requirements. The
 * function requires a valid transceiver structure and a division value
 * within the range of 1 to 32. If the division value is outside this
 * range or if the transceiver type is unsupported, the function returns
 * an error. Ensure that the transceiver is properly initialized before
 * calling this function.
 *
 * @param xcvr A pointer to a struct xilinx_xcvr representing the transceiver.
 * Must not be null and should be properly initialized.
 * @param drp_port A uint32_t representing the DRP port number. It specifies
 * which port to configure but has no specific constraints
 * mentioned.
 * @param div A uint32_t representing the division value. Must be between 1 and
 * 32 inclusive. Values outside this range result in an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int xilinx_xcvr_write_tx_clk25_div(struct xilinx_xcvr *xcvr,
				   uint32_t drp_port, uint32_t div);

/***************************************************************************//**
 * @brief This function is used to obtain the PRBS (Pseudo-Random Binary
 * Sequence) generator test pattern control setting for a specified
 * Xilinx transceiver. It is applicable for different types of
 * transceivers, such as S7 GTX2, US GTH3, US GTH4, and US GTY4. The
 * function can perform a lookup in either a forward or reverse manner
 * based on the `reverse_lu` parameter. It should be called with a valid
 * `xilinx_xcvr` structure that represents the transceiver configuration.
 * If the transceiver type is not supported or the PRBS value is not
 * found, the function returns an error code.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` that represents the
 * transceiver configuration. Must not be null and should be
 * properly initialized with a supported transceiver type.
 * @param prbs A `uint32_t` value representing the PRBS pattern to look up. The
 * valid range depends on the transceiver type and the lookup
 * direction.
 * @param reverse_lu A `bool` indicating whether to perform a reverse lookup. If
 * true, the function attempts to map the PRBS index to a
 * pattern; otherwise, it maps a pattern to an index.
 * @return Returns an integer representing the PRBS control setting if
 * successful, or a negative error code if the transceiver type is
 * unsupported or the PRBS value is not found.
 ******************************************************************************/
int xilinx_xcvr_prbsel_enc_get(struct xilinx_xcvr *xcvr,
			       uint32_t prbs, bool reverse_lu);

/***************************************************************************//**
 * @brief This function is used to obtain the Pseudo-Random Binary Sequence
 * (PRBS) error count from a specified Xilinx transceiver. It should be
 * called when you need to monitor or diagnose the error rate of a
 * transceiver channel. The function requires a valid transceiver
 * structure and a DRP port number. It returns the error count through a
 * pointer, which must not be null. The function handles different
 * transceiver types and returns an error code if the transceiver type is
 * unsupported or if any read operation fails.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null and should be properly initialized with a
 * supported transceiver type.
 * @param drp_port A `uint32_t` representing the DRP port number. Must be a
 * valid port number for the transceiver.
 * @param cnt A pointer to a `uint32_t` where the error count will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the transceiver
 * type is unsupported or if a read operation fails.
 ******************************************************************************/
int xilinx_xcvr_prbs_err_cnt_get(struct xilinx_xcvr *xcvr,
				 uint32_t drp_port, uint32_t *cnt);

/***************************************************************************//**
 * @brief This function sets the programmable divider ratio for both the RX and
 * TX paths of a Xilinx transceiver. It should be used when you need to
 * configure the RX and TX rates dynamically. The function requires a
 * valid `xilinx_xcvr` structure and is only applicable to transceivers
 * of type `XILINX_XCVR_TYPE_US_GTY4`. If the transceiver type is not
 * supported, the function returns an error. Ensure that the
 * `xilinx_xcvr` structure is properly initialized before calling this
 * function.
 *
 * @param xcvr A pointer to a `xilinx_xcvr` structure representing the
 * transceiver. Must not be null and should be properly initialized.
 * @param drp_port A 32-bit unsigned integer specifying the DRP port to be used.
 * Valid range depends on the specific hardware configuration.
 * @param rx_rate A 32-bit signed integer specifying the RX rate divider. The
 * value should be within the valid range for the specific
 * transceiver.
 * @param tx_rate A 32-bit signed integer specifying the TX rate divider. The
 * value should be within the valid range for the specific
 * transceiver.
 * @return Returns 0 on success or a negative error code if the transceiver type
 * is not supported.
 ******************************************************************************/
int xilinx_xcvr_write_prog_div_rate(struct xilinx_xcvr *xcvr,
				    uint32_t drp_port, int32_t rx_rate, int32_t tx_rate);

/***************************************************************************//**
 * @brief This function sets the programmable divider ratios for both RX and TX
 * paths of a Xilinx transceiver. It should be used when you need to
 * configure the divider settings for specific transceiver types, such as
 * US_GTH3, US_GTH4, or US_GTY4. The function requires a valid
 * transceiver structure and a DRP port identifier. It returns an error
 * code if the transceiver type is unsupported.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null and should be properly initialized with a
 * supported type.
 * @param drp_port A `uint32_t` representing the DRP port number. It should be a
 * valid port number for the transceiver.
 * @param rx_prog_div An `int32_t` specifying the RX programmable divider ratio.
 * The value should be within the valid range for the
 * specific transceiver type.
 * @param tx_prog_div An `int32_t` specifying the TX programmable divider ratio.
 * The value should be within the valid range for the
 * specific transceiver type.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the transceiver type is unsupported or if any other error
 * occurs.
 ******************************************************************************/
int xilinx_xcvr_write_prog_div(struct xilinx_xcvr *xcvr,
			       uint32_t drp_port, int32_t rx_prog_div, int32_t tx_prog_div);

/***************************************************************************//**
 * @brief This function is used to enable or disable the asynchronous gearbox
 * feature on a specified Xilinx transceiver. It should be called when
 * configuring the transceiver for specific operational modes that
 * require asynchronous gearbox functionality. The function requires a
 * valid transceiver structure and a DRP port number. It supports
 * specific transceiver types, and will return an error if the
 * transceiver type is not supported. Ensure that the transceiver
 * structure is properly initialized before calling this function.
 *
 * @param xcvr A pointer to a `struct xilinx_xcvr` representing the transceiver.
 * Must not be null and should be properly initialized with a
 * supported transceiver type.
 * @param drp_port A `uint32_t` representing the DRP port number. Must be a
 * valid port number for the transceiver.
 * @param en A `bool` indicating whether to enable (`true`) or disable (`false`)
 * the asynchronous gearbox.
 * @return Returns 0 on success for supported transceiver types, or -EINVAL if
 * the transceiver type is not supported.
 ******************************************************************************/
int xilinx_xcvr_write_async_gearbox_en(struct xilinx_xcvr *xcvr,
				       uint32_t drp_port, bool en);

#endif

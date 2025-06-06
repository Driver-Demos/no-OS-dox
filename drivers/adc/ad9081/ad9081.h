/***************************************************************************//**
 *   @file   ad9081.h
 *   @brief  Header file of AD9081 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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
#ifndef AD9081_H_
#define AD9081_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "no_os_clk.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "adi_ad9081.h"
#include "jesd204.h"

/******************************************************************************/
/********************** Macros and Types Declarations *************************/
/******************************************************************************/
#define MAX_NUM_MAIN_DATAPATHS	4
#define MAX_NUM_CHANNELIZER	8

/***************************************************************************//**
 * @brief The `ad9081_jesd_link` structure is designed to encapsulate the
 * configuration and state of a JESD link in the AD9081 device. It
 * includes fields for determining whether the link is a receiver,
 * storing JESD parameters, and configuring the JESD204 link.
 * Additionally, it manages phase adjustments, logical lane mappings, and
 * converter selections. The structure also provides detailed information
 * about the lane rate, both in bits per second and kilobits per second,
 * including a calibrated rate. This structure is crucial for managing
 * the JESD interface in high-speed data converter applications.
 *
 * @param is_jrx Indicates if the link is a JESD receiver.
 * @param jesd_param Holds JESD parameters using the adi_cms_jesd_param_t type.
 * @param jesd204_link Represents the JESD204 link configuration.
 * @param jrx_tpl_phase_adjust Stores the phase adjustment for the JRX TPL.
 * @param logiclane_mapping Array mapping logical lanes, with a size of 8.
 * @param link_converter_select Array for selecting link converters, with a size
 * of 16.
 * @param lane_rate Specifies the lane rate in bits per second.
 * @param lane_rate_kbps Specifies the lane rate in kilobits per second.
 * @param lane_cal_rate_kbps Specifies the calibrated lane rate in kilobits per
 * second.
 ******************************************************************************/
struct ad9081_jesd_link {
	bool is_jrx;
	adi_cms_jesd_param_t jesd_param;
	struct jesd204_link jesd204_link;
	uint32_t jrx_tpl_phase_adjust;
	uint8_t logiclane_mapping[8];
	uint8_t link_converter_select[16];
	uint64_t lane_rate;
	unsigned long lane_rate_kbps;
	unsigned long lane_cal_rate_kbps;
};

/***************************************************************************//**
 * @brief The `dac_settings_cache` structure is designed to store the gain
 * settings for a digital-to-analog converter (DAC) across multiple
 * channelizers. It contains a single member, `chan_gain`, which is an
 * array that holds the gain values for each channelizer, allowing for
 * efficient access and modification of these settings. This structure is
 * likely used in the context of configuring or caching DAC settings in a
 * system that supports multiple channelizers, as indicated by the use of
 * the `MAX_NUM_CHANNELIZER` constant to define the array size.
 *
 * @param chan_gain An array of uint16_t values representing the gain settings
 * for each channelizer, with a size defined by
 * MAX_NUM_CHANNELIZER.
 ******************************************************************************/
struct dac_settings_cache {
	uint16_t chan_gain[MAX_NUM_CHANNELIZER];
};

/***************************************************************************//**
 * @brief The `ad9081_phy` structure is a comprehensive data structure used to
 * manage and configure the AD9081 device, which is a high-performance
 * mixed-signal front-end. It includes pointers to various hardware
 * descriptors such as SPI, GPIO, and clock interfaces, as well as
 * configuration parameters for JESD204 links, synchronization, and
 * signal processing paths for both transmission (TX) and reception (RX).
 * The structure supports detailed configuration of digital-to-analog
 * converters (DACs) and analog-to-digital converters (ADCs), including
 * interpolation, decimation, frequency shifts, and gain settings. It
 * also manages device initialization and operational states, making it a
 * central component for controlling the AD9081's functionality in
 * complex signal processing applications.
 *
 * @param spi_desc Pointer to SPI descriptor for communication.
 * @param gpio_reset Pointer to GPIO descriptor for reset control.
 * @param ms_sync_en_gpio Pointer to GPIO descriptor for master-slave
 * synchronization enable.
 * @param jesd_rx_clk Pointer to clock descriptor for JESD RX clock.
 * @param jesd_tx_clk Pointer to clock descriptor for JESD TX clock.
 * @param dev_clk Pointer to clock descriptor for device clock.
 * @param jdev Pointer to JESD204 device structure.
 * @param sync_ms_gpio_num GPIO number for master-slave synchronization.
 * @param sysref_coupling_ac_en Boolean to enable AC coupling for SYSREF.
 * @param sysref_cmos_input_en Boolean to enable CMOS input for SYSREF.
 * @param sysref_cmos_single_end_term_pos Positive termination for single-ended
 * CMOS SYSREF.
 * @param sysref_cmos_single_end_term_neg Negative termination for single-ended
 * CMOS SYSREF.
 * @param ad9081 ADI AD9081 device structure.
 * @param jrx_link_tx Array of JESD links for JRX transmission.
 * @param jtx_link_rx Array of JESD links for JTX reception.
 * @param multidevice_instance_count Count of multidevice instances.
 * @param config_sync_01_swapped Boolean to indicate if sync pins 0 and 1 are
 * swapped.
 * @param config_sync_0a_cmos_en Boolean to enable CMOS for sync 0a.
 * @param lmfc_delay Delay for LMFC in DAC clock cycles.
 * @param nco_sync_ms_extra_lmfc_num Extra LMFC number for NCO sync.
 * @param nco_sync_direct_sysref_mode_en Boolean to enable direct SYSREF mode
 * for NCO sync.
 * @param sysref_average_cnt_exp Exponent for SYSREF average count.
 * @param sysref_continuous_dis Boolean to disable continuous SYSREF mode.
 * @param is_initialized Boolean to indicate if the device is initialized.
 * @param tx_disable Boolean to disable transmission.
 * @param rx_disable Boolean to disable reception.
 * @param dac_frequency_hz Frequency of the DAC in hertz.
 * @param tx_main_interp Interpolation factor for main TX datapaths.
 * @param tx_dac_chan_xbar Crossbar configuration for TX DAC channels.
 * @param tx_dac_chan_xbar_1x_non1x Crossbar configuration for 1x and non-1x TX
 * DAC channels.
 * @param tx_main_shift Frequency shift for main TX datapaths.
 * @param tx_chan_interp Interpolation factor for TX channelizers.
 * @param tx_chan_shift Frequency shift for TX channelizers.
 * @param tx_dac_fsc Full-scale current for TX DACs.
 * @param tx_ffh_hopf_via_gpio_en Boolean to enable FFH HOPF via GPIO.
 * @param dac_cache Cache for DAC settings.
 * @param adc_dcm Decimation factor for ADC.
 * @param adc_frequency_hz Frequency of the ADC in hertz.
 * @param rx_nyquist_zone Nyquist zone for RX.
 * @param rx_cddc_shift Frequency shift for RX CDDC.
 * @param adc_main_decimation Decimation factor for main ADC datapaths.
 * @param rx_cddc_dcm Decimation factor for RX CDDC.
 * @param rx_fddc_mxr_if Mixer interface for RX FDDC.
 * @param rx_cddc_c2r Complex to real conversion for RX CDDC.
 * @param rx_cddc_select Selection for RX CDDC.
 * @param rx_fddc_shift Frequency shift for RX FDDC.
 * @param adc_chan_decimation Decimation factor for ADC channelizers.
 * @param rx_fddc_dcm Decimation factor for RX FDDC.
 * @param rx_fddc_c2r Complex to real conversion for RX FDDC.
 * @param rx_cddc_gain_6db_en Enable 6dB gain for RX CDDC.
 * @param rx_fddc_gain_6db_en Enable 6dB gain for RX FDDC.
 * @param rx_fddc_select Selection for RX FDDC.
 * @param rx_cddc_nco_channel_select_mode NCO channel select mode for RX CDDC.
 * @param rx_ffh_gpio_mux_sel GPIO multiplexer selection for RX FFH.
 ******************************************************************************/
struct ad9081_phy {
	struct no_os_spi_desc		*spi_desc;
	struct no_os_gpio_desc		*gpio_reset;
	struct no_os_gpio_desc		*ms_sync_en_gpio;
	struct no_os_clk		*jesd_rx_clk;
	struct no_os_clk		*jesd_tx_clk;
	struct no_os_clk		*dev_clk;
	struct jesd204_dev		*jdev;
	uint8_t		sync_ms_gpio_num;
	bool		sysref_coupling_ac_en;
	bool 		sysref_cmos_input_en;
	uint8_t 	sysref_cmos_single_end_term_pos;
	uint8_t 	sysref_cmos_single_end_term_neg;
	adi_ad9081_device_t	ad9081;
	struct ad9081_jesd_link	jrx_link_tx[2];
	struct ad9081_jesd_link	jtx_link_rx[2];
	uint32_t	multidevice_instance_count;
	bool		config_sync_01_swapped;
	bool		config_sync_0a_cmos_en;
	uint32_t	lmfc_delay;
	uint32_t	nco_sync_ms_extra_lmfc_num;
	bool		nco_sync_direct_sysref_mode_en;
	uint32_t	sysref_average_cnt_exp;
	bool		sysref_continuous_dis;
	bool		is_initialized;
	bool		tx_disable;
	bool		rx_disable;
	/* TX */
	uint64_t	dac_frequency_hz;
	/* The 4 DAC Main Datapaths */
	uint32_t	tx_main_interp;
	uint8_t		tx_dac_chan_xbar[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		tx_dac_chan_xbar_1x_non1x[MAX_NUM_MAIN_DATAPATHS];
	int64_t		tx_main_shift[MAX_NUM_MAIN_DATAPATHS];
	/* The 8 DAC Channelizers */
	uint32_t	tx_chan_interp;
	int64_t		tx_chan_shift[MAX_NUM_CHANNELIZER];
	uint32_t	tx_dac_fsc[MAX_NUM_MAIN_DATAPATHS];
	bool		tx_ffh_hopf_via_gpio_en;
	struct dac_settings_cache	dac_cache;
	/* RX */
	uint32_t	adc_dcm[2];
	uint64_t 	adc_frequency_hz;
	uint32_t	rx_nyquist_zone[MAX_NUM_MAIN_DATAPATHS];
	/* The 4 ADC Main Datapaths */
	int64_t		rx_cddc_shift[MAX_NUM_MAIN_DATAPATHS];
	uint32_t	adc_main_decimation[MAX_NUM_MAIN_DATAPATHS];
	uint8_t 	rx_cddc_dcm[MAX_NUM_MAIN_DATAPATHS];
	uint8_t 	rx_fddc_mxr_if[MAX_NUM_CHANNELIZER];
	uint8_t 	rx_cddc_c2r[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_cddc_select;
	/* The 8 ADC Channelizers */
	int64_t		rx_fddc_shift[MAX_NUM_CHANNELIZER];
	uint32_t	adc_chan_decimation[MAX_NUM_CHANNELIZER];
	uint8_t		rx_fddc_dcm[MAX_NUM_CHANNELIZER];
	uint8_t 	rx_fddc_c2r[MAX_NUM_CHANNELIZER];
	uint8_t		rx_cddc_gain_6db_en[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_fddc_gain_6db_en[MAX_NUM_CHANNELIZER];
	uint8_t 	rx_fddc_select;
	uint8_t		rx_cddc_nco_channel_select_mode[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_ffh_gpio_mux_sel[6];
};

/***************************************************************************//**
 * @brief The `link_init_param` structure is used to initialize and configure a
 * JESD204 link for a device, containing various parameters such as
 * device ID, frame and multiframe configurations, converter settings,
 * and link modes. It includes arrays for logical lane mapping and link
 * converter selection, as well as a phase adjustment parameter for TPL,
 * making it essential for setting up the data transmission
 * characteristics of the device.
 *
 * @param device_id A unique identifier for the device.
 * @param octets_per_frame Specifies the number of octets per frame.
 * @param frames_per_multiframe Indicates the number of frames per multiframe.
 * @param samples_per_converter_per_frame Defines the number of samples per
 * converter per frame.
 * @param high_density A flag indicating if high density mode is enabled.
 * @param converter_resolution Specifies the resolution of the converter.
 * @param bits_per_sample Indicates the number of bits per sample.
 * @param converters_per_device Specifies the number of converters per device.
 * @param control_bits_per_sample Defines the number of control bits per sample.
 * @param lanes_per_device Indicates the number of lanes per device.
 * @param subclass Specifies the subclass of the link.
 * @param link_mode Defines the mode of the link.
 * @param dual_link A flag indicating if dual link mode is enabled.
 * @param version Specifies the version of the link.
 * @param logical_lane_mapping An array mapping logical lanes.
 * @param link_converter_select An array for selecting link converters.
 * @param tpl_phase_adjust Specifies the phase adjustment for TPL.
 ******************************************************************************/
struct link_init_param {
	uint32_t	device_id;
	uint32_t	octets_per_frame;
	uint32_t	frames_per_multiframe;
	uint32_t	samples_per_converter_per_frame;
	uint32_t	high_density;
	uint32_t	converter_resolution;
	uint32_t	bits_per_sample;
	uint32_t	converters_per_device;
	uint32_t	control_bits_per_sample;
	uint32_t	lanes_per_device;
	uint32_t	subclass;
	uint32_t	link_mode;
	uint32_t	dual_link;
	uint32_t	version;
	uint8_t		logical_lane_mapping[8];
	/* JTX */
	uint8_t		link_converter_select[16];
	/* JRX */
	uint32_t	tpl_phase_adjust;
};

/***************************************************************************//**
 * @brief The `ad9081_init_param` structure is a comprehensive configuration
 * data structure used to initialize the AD9081 device, which is a high-
 * performance mixed-signal front-end. It includes parameters for SPI and
 * GPIO initialization, clock configurations, synchronization settings,
 * and various operational flags for both the transmitter (TX) and
 * receiver (RX) paths. The structure supports detailed configuration of
 * the DAC and ADC datapaths, including interpolation, decimation, NCO
 * frequency shifts, and gain settings. It also includes settings for
 * JESD204 links, allowing for flexible and precise control over the
 * device's data conversion and transmission capabilities.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_reset Pointer to GPIO reset initialization parameters.
 * @param ms_sync_enable Pointer to GPIO master-slave sync enable initialization
 * parameters.
 * @param dev_clk Pointer to the device clock configuration.
 * @param jesd_rx_clk Pointer to the JESD receiver clock configuration.
 * @param jesd_tx_clk Pointer to the JESD transmitter clock configuration.
 * @param master_slave_sync_gpio_num GPIO number for master-slave
 * synchronization.
 * @param sysref_coupling_ac_en Flag to enable AC coupling for SYSREF.
 * @param sysref_cmos_input_enable Flag to enable CMOS input for SYSREF.
 * @param sysref_cmos_single_end_term_pos Positive termination for single-ended
 * CMOS SYSREF.
 * @param sysref_cmos_single_end_term_neg Negative termination for single-ended
 * CMOS SYSREF.
 * @param multidevice_instance_count Count of multidevice instances.
 * @param jesd_sync_pins_01_swap_enable Flag to enable swapping of JESD sync
 * pins 0 and 1.
 * @param config_sync_0a_cmos_enable Flag to enable CMOS configuration for sync
 * 0a.
 * @param lmfc_delay_dac_clk_cycles LMFC delay in DAC clock cycles.
 * @param nco_sync_ms_extra_lmfc_num Extra LMFC number for NCO sync in master-
 * slave mode.
 * @param nco_sync_direct_sysref_mode_enable Flag to enable direct SYSREF mode
 * for NCO sync.
 * @param sysref_average_cnt_exp Exponent for SYSREF averaging count.
 * @param continuous_sysref_mode_disable Flag to disable continuous SYSREF mode.
 * @param tx_disable Flag to disable the transmitter.
 * @param rx_disable Flag to disable the receiver.
 * @param dac_frequency_hz Frequency of the DAC in hertz.
 * @param tx_main_interpolation Interpolation factor for the main TX datapath.
 * @param tx_main_nco_frequency_shift_hz NCO frequency shift for the main TX
 * datapath.
 * @param tx_dac_channel_crossbar_select Crossbar selection for TX DAC channels.
 * @param tx_maindp_dac_1x_non1x_crossbar_select Crossbar selection for 1x and
 * non-1x TX DAC channels.
 * @param tx_full_scale_current_ua Full-scale current for TX DAC channels in
 * microamperes.
 * @param tx_channel_interpolation Interpolation factor for TX channelizers.
 * @param tx_channel_nco_frequency_shift_hz NCO frequency shift for TX
 * channelizers.
 * @param tx_channel_gain Gain for TX channelizers.
 * @param jrx_link_tx Array of pointers to link initialization parameters for
 * JRX TX.
 * @param adc_frequency_hz Frequency of the ADC in hertz.
 * @param nyquist_zone Nyquist zone for the main RX datapath.
 * @param rx_main_nco_frequency_shift_hz NCO frequency shift for the main RX
 * datapath.
 * @param rx_main_decimation Decimation factor for the main RX datapath.
 * @param rx_main_complex_to_real_enable Flag to enable complex-to-real
 * conversion for the main RX datapath.
 * @param rx_main_digital_gain_6db_enable Flag to enable 6dB digital gain for
 * the main RX datapath.
 * @param rx_main_enable Flag to enable the main RX datapath.
 * @param rx_channel_nco_frequency_shift_hz NCO frequency shift for RX
 * channelizers.
 * @param rx_channel_decimation Decimation factor for RX channelizers.
 * @param rx_channel_complex_to_real_enable Flag to enable complex-to-real
 * conversion for RX channelizers.
 * @param rx_channel_nco_mixer_mode Mixer mode for RX channelizers.
 * @param rx_channel_digital_gain_6db_enable Flag to enable 6dB digital gain for
 * RX channelizers.
 * @param rx_channel_enable Flag to enable RX channelizers.
 * @param rx_cddc_nco_channel_select_mode NCO channel select mode for RX CDDC.
 * @param rx_ffh_gpio_mux_selection GPIO mux selection for RX FFH.
 * @param jtx_link_rx Array of pointers to link initialization parameters for
 * JTX RX.
 ******************************************************************************/
struct ad9081_init_param {
	struct no_os_spi_init_param	*spi_init;
	struct no_os_gpio_init_param	*gpio_reset;
	struct no_os_gpio_init_param	*ms_sync_enable;
	struct no_os_clk	*dev_clk;
	struct no_os_clk	*jesd_rx_clk;
	struct no_os_clk	*jesd_tx_clk;
	uint8_t		master_slave_sync_gpio_num;
	bool		sysref_coupling_ac_en;
	bool		sysref_cmos_input_enable;
	uint8_t 	sysref_cmos_single_end_term_pos;
	uint8_t 	sysref_cmos_single_end_term_neg;
	uint32_t	multidevice_instance_count;
	bool		jesd_sync_pins_01_swap_enable;
	bool 		config_sync_0a_cmos_enable;
	uint32_t	lmfc_delay_dac_clk_cycles;
	uint32_t	nco_sync_ms_extra_lmfc_num;
	bool		nco_sync_direct_sysref_mode_enable;
	uint32_t	sysref_average_cnt_exp;
	bool		continuous_sysref_mode_disable;
	bool		tx_disable;
	bool		rx_disable;
	/* TX */
	uint64_t	dac_frequency_hz;
	/* The 4 DAC Main Datapaths */
	uint32_t	tx_main_interpolation;
	int64_t		tx_main_nco_frequency_shift_hz[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		tx_dac_channel_crossbar_select[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		tx_maindp_dac_1x_non1x_crossbar_select[MAX_NUM_MAIN_DATAPATHS];
	uint32_t	tx_full_scale_current_ua[MAX_NUM_MAIN_DATAPATHS];
	/* The 8 DAC Channelizers */
	uint32_t	tx_channel_interpolation;
	int64_t		tx_channel_nco_frequency_shift_hz[MAX_NUM_CHANNELIZER];
	uint16_t	tx_channel_gain[MAX_NUM_CHANNELIZER];
	struct link_init_param	*jrx_link_tx[2];
	/* RX */
	uint64_t 	adc_frequency_hz;
	uint32_t	nyquist_zone[MAX_NUM_MAIN_DATAPATHS];
	/* The 4 ADC Main Datapaths */
	int64_t		rx_main_nco_frequency_shift_hz[MAX_NUM_MAIN_DATAPATHS];
	uint32_t	rx_main_decimation[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_main_complex_to_real_enable[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_main_digital_gain_6db_enable[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_main_enable[MAX_NUM_MAIN_DATAPATHS];
	/* The 8 ADC Channelizers */
	int64_t		rx_channel_nco_frequency_shift_hz[MAX_NUM_CHANNELIZER];
	uint32_t	rx_channel_decimation[MAX_NUM_CHANNELIZER];
	uint8_t		rx_channel_complex_to_real_enable[MAX_NUM_CHANNELIZER];
	uint8_t		rx_channel_nco_mixer_mode[MAX_NUM_CHANNELIZER];
	uint8_t		rx_channel_digital_gain_6db_enable[MAX_NUM_CHANNELIZER];
	uint8_t		rx_channel_enable[MAX_NUM_CHANNELIZER];
	uint8_t		rx_cddc_nco_channel_select_mode[MAX_NUM_MAIN_DATAPATHS];
	uint8_t		rx_ffh_gpio_mux_selection[6];
	struct link_init_param	*jtx_link_rx[2];
};

/* ffh: 2 - gpio6, 3 - gpio7, 4 - gpio8, 5 - gpio9, 6 - gpio10, 7 - syncinb1_p, 8 - syncinb1_n */

#define AD9081_PERI_SEL_GPIO6		2
#define AD9081_PERI_SEL_GPIO7		3
#define AD9081_PERI_SEL_GPIO8		4
#define AD9081_PERI_SEL_GPIO9		5
#define AD9081_PERI_SEL_GPIO10		6
#define AD9081_PERI_SEL_SYNCINB1_P	7
#define AD9081_PERI_SEL_SYNCINB1_N	8

#define AD9081_FFH_CHAN_SEL_REG_MODE		0 /* 0:  Register Map control (Use ddc_nco_regmap_chan_sel) */
#define AD9081_FFH_CHAN_SEL_1GPIO_MODE		1 /* 1:  profile_pins[0]     is used. Pin level control {3'b0, profile_pins[0]} */
#define AD9081_FFH_CHAN_SEL_2GPIO_MODE		2 /* 2:  profile_pins[1 :0] are used. Pin level control {2'b0, profile_pins[1:0]} */
#define AD9081_FFH_CHAN_SEL_3GPIO_MODE		3 /* 3:  profile_pins[2 :0] are used. Pin level control {1'b0, profile_pins[2:0]} */
#define AD9081_FFH_CHAN_SEL_4GPIO_MODE		4 /* 4:  profile_pins[3 :0] are used. Pin level control { profile_pins[3:0]} */
#define AD9081_FFH_CHAN_SEL_GPIO0_EDGE_MODE	8 /* 8:  profile_pins[0] Pin edge control- increment internal counter when rising edge of profile_pins[0] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO1_EDGE_MODE	9 /* 9:  profile_pins[1] Pin edge control- increment internal counter when rising edge of profile_pins[1] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO2_EDGE_MODE	10 /* 10: profile_pins[2] Pin edge control- increment internal counter when rising edge of profile_pins[2] Pin. */
#define AD9081_FFH_CHAN_SEL_GPIO3_EDGE_MODE	11 /* 11: profile_pins[3] Pin edge control- increment internal counter when rising edge of profile_pins[3] Pin. */
#define AD9081_FFH_CHAN_SEL_FHT_EXP_MODE	12 /* 12: FHT expire based control - increment internal counter when FHT is expired. */

/*
 * JESD204-FSM defines
 */
#define DEFRAMER_LINK0_TX 0
#define DEFRAMER_LINK1_TX 1
#define FRAMER_LINK0_RX 2
#define FRAMER_LINK1_RX 3

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD9081 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the necessary hardware
 * interfaces, configures the device according to the initialization
 * parameters, and verifies the device identity. If initialization fails
 * at any step, it performs cleanup and returns an error code. Ensure
 * that the `init_param` structure is correctly populated before calling
 * this function.
 *
 * @param dev A pointer to a pointer of type `struct ad9081_phy`. This will be
 * allocated and initialized by the function. The caller must ensure
 * this pointer is valid and will receive ownership of the allocated
 * structure upon successful initialization.
 * @param init_param A pointer to a `struct ad9081_init_param` containing the
 * initialization parameters. This must be properly
 * initialized by the caller before calling the function. The
 * structure includes configuration for SPI, GPIO, and clock
 * settings, among others.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and performs cleanup of any allocated resources.
 ******************************************************************************/
int32_t ad9081_init(struct ad9081_phy **device,
		    const struct ad9081_init_param *init_param);
/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly remove an AD9081 device instance and
 * free associated resources. It should be called when the device is no
 * longer needed, ensuring that all allocated resources such as GPIO and
 * SPI descriptors are released. This function must be called only after
 * the device has been initialized and used, to prevent resource leaks.
 *
 * @param device A pointer to an ad9081_phy structure representing the device to
 * be removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource removal process. A non-zero return value indicates an error
 * occurred during the removal of resources.
 ******************************************************************************/
int32_t ad9081_remove(struct ad9081_phy *device);
/* Work function. */
void ad9081_work_func(struct ad9081_phy *phy);
#endif

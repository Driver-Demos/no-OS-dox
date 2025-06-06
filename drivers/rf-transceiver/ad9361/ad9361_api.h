/***************************************************************************//**
 *   @file   ad9361_api.h
 *   @brief  Header file of AD9361 API Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#ifndef AD9361_API_H_
#define AD9361_API_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad9361_util.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `AD9361_InitParam` structure is a comprehensive configuration
 * structure used to initialize the AD9361 transceiver. It contains a
 * wide array of parameters that control various aspects of the device,
 * including device selection, reference clock settings, base
 * configuration options, LO control, rate and bandwidth settings, RF
 * port control, gain control, and more. This structure allows for
 * detailed customization of the AD9361's operation, enabling users to
 * tailor the transceiver's behavior to specific application
 * requirements. It also includes function pointers for external LO clock
 * management and optional ADC/DAC initialization parameters, providing
 * flexibility for integration into different system architectures.
 *
 * @param dev_sel Specifies the device selection through an enumeration.
 * @param reference_clk_rate Defines the reference clock rate in Hz.
 * @param two_rx_two_tx_mode_enable Enables or disables the 2RX-2TX mode.
 * @param one_rx_one_tx_mode_use_rx_num Specifies which RX number to use in
 * 1RX-1TX mode.
 * @param one_rx_one_tx_mode_use_tx_num Specifies which TX number to use in
 * 1RX-1TX mode.
 * @param frequency_division_duplex_mode_enable Enables or disables frequency
 * division duplex mode.
 * @param frequency_division_duplex_independent_mode_enable Enables or disables
 * frequency division
 * duplex independent
 * mode.
 * @param tdd_use_dual_synth_mode_enable Enables or disables TDD dual
 * synthesizer mode.
 * @param tdd_skip_vco_cal_enable Enables or disables skipping VCO calibration
 * in TDD mode.
 * @param tx_fastlock_delay_ns Specifies the TX fastlock delay in nanoseconds.
 * @param rx_fastlock_delay_ns Specifies the RX fastlock delay in nanoseconds.
 * @param rx_fastlock_pincontrol_enable Enables or disables RX fastlock pin
 * control.
 * @param tx_fastlock_pincontrol_enable Enables or disables TX fastlock pin
 * control.
 * @param external_rx_lo_enable Enables or disables the use of an external RX
 * local oscillator.
 * @param external_tx_lo_enable Enables or disables the use of an external TX
 * local oscillator.
 * @param dc_offset_tracking_update_event_mask Specifies the event mask for DC
 * offset tracking updates.
 * @param dc_offset_attenuation_high_range Defines the high range attenuation
 * for DC offset.
 * @param dc_offset_attenuation_low_range Defines the low range attenuation for
 * DC offset.
 * @param dc_offset_count_high_range Specifies the high range count for DC
 * offset.
 * @param dc_offset_count_low_range Specifies the low range count for DC offset.
 * @param split_gain_table_mode_enable Enables or disables split gain table
 * mode.
 * @param trx_synthesizer_target_fref_overwrite_hz Specifies the target
 * frequency reference overwrite
 * for the synthesizer in Hz.
 * @param qec_tracking_slow_mode_enable Enables or disables QEC tracking slow
 * mode.
 * @param ensm_enable_pin_pulse_mode_enable Enables or disables ENSM pin pulse
 * mode.
 * @param ensm_enable_txnrx_control_enable Enables or disables ENSM TXNRX
 * control.
 * @param rx_synthesizer_frequency_hz Specifies the RX synthesizer frequency in
 * Hz.
 * @param tx_synthesizer_frequency_hz Specifies the TX synthesizer frequency in
 * Hz.
 * @param tx_lo_powerdown_managed_enable Enables or disables managed power down
 * for TX LO.
 * @param rx_path_clock_frequencies Array specifying RX path clock frequencies.
 * @param tx_path_clock_frequencies Array specifying TX path clock frequencies.
 * @param rf_rx_bandwidth_hz Specifies the RF RX bandwidth in Hz.
 * @param rf_tx_bandwidth_hz Specifies the RF TX bandwidth in Hz.
 * @param rx_rf_port_input_select Selects the RX RF port input.
 * @param tx_rf_port_input_select Selects the TX RF port input.
 * @param tx_attenuation_mdB Specifies the TX attenuation in milli-dB.
 * @param update_tx_gain_in_alert_enable Enables or disables TX gain update in
 * alert mode.
 * @param xo_disable_use_ext_refclk_enable Enables or disables the use of an
 * external reference clock when XO is
 * disabled.
 * @param dcxo_coarse_and_fine_tune Array for DCXO coarse and fine tuning.
 * @param clk_output_mode_select Selects the clock output mode.
 * @param gc_rx1_mode Specifies the gain control mode for RX1.
 * @param gc_rx2_mode Specifies the gain control mode for RX2.
 * @param gc_adc_large_overload_thresh Specifies the ADC large overload
 * threshold.
 * @param gc_adc_ovr_sample_size Specifies the ADC overload sample size.
 * @param gc_adc_small_overload_thresh Specifies the ADC small overload
 * threshold.
 * @param gc_dec_pow_measurement_duration Specifies the duration for power
 * measurement in decimation.
 * @param gc_dig_gain_enable Enables or disables digital gain control.
 * @param gc_lmt_overload_high_thresh Specifies the high threshold for LMT
 * overload.
 * @param gc_lmt_overload_low_thresh Specifies the low threshold for LMT
 * overload.
 * @param gc_low_power_thresh Specifies the low power threshold.
 * @param gc_max_dig_gain Specifies the maximum digital gain.
 * @param gc_use_rx_fir_out_for_dec_pwr_meas_enable Enables or disables the use
 * of RX FIR output for
 * decimation power
 * measurement.
 * @param mgc_dec_gain_step Specifies the MGC decrement gain step.
 * @param mgc_inc_gain_step Specifies the MGC increment gain step.
 * @param mgc_rx1_ctrl_inp_enable Enables or disables MGC control input for RX1.
 * @param mgc_rx2_ctrl_inp_enable Enables or disables MGC control input for RX2.
 * @param mgc_split_table_ctrl_inp_gain_mode Specifies the MGC split table
 * control input gain mode.
 * @param agc_adc_large_overload_exceed_counter Specifies the counter for ADC
 * large overload exceedance.
 * @param agc_adc_large_overload_inc_steps Specifies the increment steps for ADC
 * large overload.
 * @param agc_adc_lmt_small_overload_prevent_gain_inc_enable Enables or disables
 * prevention of gain
 * increase on small
 * overload.
 * @param agc_adc_small_overload_exceed_counter Specifies the counter for ADC
 * small overload exceedance.
 * @param agc_dig_gain_step_size Specifies the digital gain step size for AGC.
 * @param agc_dig_saturation_exceed_counter Specifies the counter for digital
 * saturation exceedance.
 * @param agc_gain_update_interval_us Specifies the gain update interval in
 * microseconds.
 * @param agc_immed_gain_change_if_large_adc_overload_enable Enables or disables
 * immediate gain
 * change on large ADC
 * overload.
 * @param agc_immed_gain_change_if_large_lmt_overload_enable Enables or disables
 * immediate gain
 * change on large LMT
 * overload.
 * @param agc_inner_thresh_high Specifies the inner high threshold for AGC.
 * @param agc_inner_thresh_high_dec_steps Specifies the decrement steps for
 * inner high threshold.
 * @param agc_inner_thresh_low Specifies the inner low threshold for AGC.
 * @param agc_inner_thresh_low_inc_steps Specifies the increment steps for inner
 * low threshold.
 * @param agc_lmt_overload_large_exceed_counter Specifies the counter for large
 * LMT overload exceedance.
 * @param agc_lmt_overload_large_inc_steps Specifies the increment steps for
 * large LMT overload.
 * @param agc_lmt_overload_small_exceed_counter Specifies the counter for small
 * LMT overload exceedance.
 * @param agc_outer_thresh_high Specifies the outer high threshold for AGC.
 * @param agc_outer_thresh_high_dec_steps Specifies the decrement steps for
 * outer high threshold.
 * @param agc_outer_thresh_low Specifies the outer low threshold for AGC.
 * @param agc_outer_thresh_low_inc_steps Specifies the increment steps for outer
 * low threshold.
 * @param agc_attack_delay_extra_margin_us Specifies the extra margin for AGC
 * attack delay in microseconds.
 * @param agc_sync_for_gain_counter_enable Enables or disables synchronization
 * for gain counter.
 * @param fagc_dec_pow_measuremnt_duration Specifies the duration for power
 * measurement in fast AGC.
 * @param fagc_state_wait_time_ns Specifies the state wait time for fast AGC in
 * nanoseconds.
 * @param fagc_allow_agc_gain_increase Enables or disables AGC gain increase in
 * fast AGC low power mode.
 * @param fagc_lp_thresh_increment_time Specifies the increment time for low
 * power threshold in fast AGC.
 * @param fagc_lp_thresh_increment_steps Specifies the increment steps for low
 * power threshold in fast AGC.
 * @param fagc_lock_level_lmt_gain_increase_en Enables or disables LMT gain
 * increase at lock level in fast
 * AGC.
 * @param fagc_lock_level_gain_increase_upper_limit Specifies the upper limit
 * for gain increase at lock
 * level in fast AGC.
 * @param fagc_lpf_final_settling_steps Specifies the final settling steps for
 * LPF in fast AGC.
 * @param fagc_lmt_final_settling_steps Specifies the final settling steps for
 * LMT in fast AGC.
 * @param fagc_final_overrange_count Specifies the final overrange count in fast
 * AGC.
 * @param fagc_gain_increase_after_gain_lock_en Enables or disables gain
 * increase after gain lock in fast
 * AGC.
 * @param fagc_gain_index_type_after_exit_rx_mode Specifies the gain index type
 * after exiting RX mode in fast
 * AGC.
 * @param fagc_use_last_lock_level_for_set_gain_en Enables or disables the use
 * of last lock level for
 * setting gain in fast AGC.
 * @param fagc_rst_gla_stronger_sig_thresh_exceeded_en Enables or disables reset
 * GLA on stronger signal
 * threshold exceedance in
 * fast AGC.
 * @param fagc_optimized_gain_offset Specifies the optimized gain offset in fast
 * AGC.
 * @param fagc_rst_gla_stronger_sig_thresh_above_ll Specifies the stronger
 * signal threshold above lock
 * level for reset GLA in fast
 * AGC.
 * @param fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en Enables or disables
 * reset GLA on energy
 * lost signal threshold
 * exceedance in fast
 * AGC.
 * @param fagc_rst_gla_engergy_lost_goto_optim_gain_en Enables or disables reset
 * GLA to optimized gain on
 * energy lost in fast AGC.
 * @param fagc_rst_gla_engergy_lost_sig_thresh_below_ll Specifies the energy
 * lost signal threshold
 * below lock level for
 * reset GLA in fast AGC.
 * @param fagc_energy_lost_stronger_sig_gain_lock_exit_cnt Specifies the count
 * for energy lost
 * stronger signal gain
 * lock exit in fast
 * AGC.
 * @param fagc_rst_gla_large_adc_overload_en Enables or disables reset GLA on
 * large ADC overload in fast AGC.
 * @param fagc_rst_gla_large_lmt_overload_en Enables or disables reset GLA on
 * large LMT overload in fast AGC.
 * @param fagc_rst_gla_en_agc_pulled_high_en Enables or disables reset GLA when
 * AGC is pulled high in fast AGC.
 * @param fagc_rst_gla_if_en_agc_pulled_high_mode Specifies the mode for reset
 * GLA if AGC is pulled high in
 * fast AGC.
 * @param fagc_power_measurement_duration_in_state5 Specifies the power
 * measurement duration in
 * state 5 for fast AGC.
 * @param fagc_large_overload_inc_steps Specifies the increment steps for large
 * overload in fast AGC.
 * @param rssi_delay Specifies the delay for RSSI measurement.
 * @param rssi_duration Specifies the duration for RSSI measurement.
 * @param rssi_restart_mode Specifies the restart mode for RSSI measurement.
 * @param rssi_unit_is_rx_samples_enable Enables or disables RSSI unit as RX
 * samples.
 * @param rssi_wait Specifies the wait time for RSSI measurement.
 * @param aux_adc_decimation Specifies the decimation rate for auxiliary ADC.
 * @param aux_adc_rate Specifies the rate for auxiliary ADC.
 * @param aux_dac_manual_mode_enable Enables or disables manual mode for
 * auxiliary DAC.
 * @param aux_dac1_default_value_mV Specifies the default value for auxiliary
 * DAC1 in mV.
 * @param aux_dac1_active_in_rx_enable Enables or disables auxiliary DAC1 in RX
 * mode.
 * @param aux_dac1_active_in_tx_enable Enables or disables auxiliary DAC1 in TX
 * mode.
 * @param aux_dac1_active_in_alert_enable Enables or disables auxiliary DAC1 in
 * alert mode.
 * @param aux_dac1_rx_delay_us Specifies the RX delay for auxiliary DAC1 in
 * microseconds.
 * @param aux_dac1_tx_delay_us Specifies the TX delay for auxiliary DAC1 in
 * microseconds.
 * @param aux_dac2_default_value_mV Specifies the default value for auxiliary
 * DAC2 in mV.
 * @param aux_dac2_active_in_rx_enable Enables or disables auxiliary DAC2 in RX
 * mode.
 * @param aux_dac2_active_in_tx_enable Enables or disables auxiliary DAC2 in TX
 * mode.
 * @param aux_dac2_active_in_alert_enable Enables or disables auxiliary DAC2 in
 * alert mode.
 * @param aux_dac2_rx_delay_us Specifies the RX delay for auxiliary DAC2 in
 * microseconds.
 * @param aux_dac2_tx_delay_us Specifies the TX delay for auxiliary DAC2 in
 * microseconds.
 * @param temp_sense_decimation Specifies the decimation rate for temperature
 * sensing.
 * @param temp_sense_measurement_interval_ms Specifies the measurement interval
 * for temperature sensing in
 * milliseconds.
 * @param temp_sense_offset_signed Specifies the signed offset for temperature
 * sensing.
 * @param temp_sense_periodic_measurement_enable Enables or disables periodic
 * measurement for temperature
 * sensing.
 * @param ctrl_outs_enable_mask Specifies the enable mask for control outputs.
 * @param ctrl_outs_index Specifies the index for control outputs.
 * @param elna_settling_delay_ns Specifies the settling delay for external LNA
 * in nanoseconds.
 * @param elna_gain_mdB Specifies the gain for external LNA in milli-dB.
 * @param elna_bypass_loss_mdB Specifies the bypass loss for external LNA in
 * milli-dB.
 * @param elna_rx1_gpo0_control_enable Enables or disables GPO0 control for RX1
 * external LNA.
 * @param elna_rx2_gpo1_control_enable Enables or disables GPO1 control for RX2
 * external LNA.
 * @param elna_gaintable_all_index_enable Enables or disables gain table for all
 * index in external LNA.
 * @param digital_interface_tune_skip_mode Specifies the tune skip mode for
 * digital interface.
 * @param digital_interface_tune_fir_disable Enables or disables FIR tuning for
 * digital interface.
 * @param pp_tx_swap_enable Enables or disables TX swap for parallel port.
 * @param pp_rx_swap_enable Enables or disables RX swap for parallel port.
 * @param tx_channel_swap_enable Enables or disables TX channel swap.
 * @param rx_channel_swap_enable Enables or disables RX channel swap.
 * @param rx_frame_pulse_mode_enable Enables or disables RX frame pulse mode.
 * @param two_t_two_r_timing_enable Enables or disables 2T2R timing.
 * @param invert_data_bus_enable Enables or disables data bus inversion.
 * @param invert_data_clk_enable Enables or disables data clock inversion.
 * @param fdd_alt_word_order_enable Enables or disables alternate word order in
 * FDD mode.
 * @param invert_rx_frame_enable Enables or disables RX frame inversion.
 * @param fdd_rx_rate_2tx_enable Enables or disables RX rate 2TX in FDD mode.
 * @param swap_ports_enable Enables or disables port swapping.
 * @param single_data_rate_enable Enables or disables single data rate.
 * @param lvds_mode_enable Enables or disables LVDS mode.
 * @param half_duplex_mode_enable Enables or disables half duplex mode.
 * @param single_port_mode_enable Enables or disables single port mode.
 * @param full_port_enable Enables or disables full port mode.
 * @param full_duplex_swap_bits_enable Enables or disables bit swapping in full
 * duplex mode.
 * @param delay_rx_data Specifies the delay for RX data.
 * @param rx_data_clock_delay Specifies the delay for RX data clock.
 * @param rx_data_delay Specifies the delay for RX data.
 * @param tx_fb_clock_delay Specifies the delay for TX feedback clock.
 * @param tx_data_delay Specifies the delay for TX data.
 * @param lvds_bias_mV Specifies the LVDS bias in mV.
 * @param lvds_rx_onchip_termination_enable Enables or disables on-chip
 * termination for LVDS RX.
 * @param rx1rx2_phase_inversion_en Enables or disables phase inversion between
 * RX1 and RX2.
 * @param lvds_invert1_control Specifies the control for LVDS inversion 1.
 * @param lvds_invert2_control Specifies the control for LVDS inversion 2.
 * @param gpo_manual_mode_enable Enables or disables manual mode for GPO.
 * @param gpo_manual_mode_enable_mask Specifies the enable mask for GPO manual
 * mode.
 * @param gpo0_inactive_state_high_enable Enables or disables high inactive
 * state for GPO0.
 * @param gpo1_inactive_state_high_enable Enables or disables high inactive
 * state for GPO1.
 * @param gpo2_inactive_state_high_enable Enables or disables high inactive
 * state for GPO2.
 * @param gpo3_inactive_state_high_enable Enables or disables high inactive
 * state for GPO3.
 * @param gpo0_slave_rx_enable Enables or disables slave RX for GPO0.
 * @param gpo0_slave_tx_enable Enables or disables slave TX for GPO0.
 * @param gpo1_slave_rx_enable Enables or disables slave RX for GPO1.
 * @param gpo1_slave_tx_enable Enables or disables slave TX for GPO1.
 * @param gpo2_slave_rx_enable Enables or disables slave RX for GPO2.
 * @param gpo2_slave_tx_enable Enables or disables slave TX for GPO2.
 * @param gpo3_slave_rx_enable Enables or disables slave RX for GPO3.
 * @param gpo3_slave_tx_enable Enables or disables slave TX for GPO3.
 * @param gpo0_rx_delay_us Specifies the RX delay for GPO0 in microseconds.
 * @param gpo0_tx_delay_us Specifies the TX delay for GPO0 in microseconds.
 * @param gpo1_rx_delay_us Specifies the RX delay for GPO1 in microseconds.
 * @param gpo1_tx_delay_us Specifies the TX delay for GPO1 in microseconds.
 * @param gpo2_rx_delay_us Specifies the RX delay for GPO2 in microseconds.
 * @param gpo2_tx_delay_us Specifies the TX delay for GPO2 in microseconds.
 * @param gpo3_rx_delay_us Specifies the RX delay for GPO3 in microseconds.
 * @param gpo3_tx_delay_us Specifies the TX delay for GPO3 in microseconds.
 * @param low_high_gain_threshold_mdB Specifies the low-high gain threshold for
 * TX monitor in milli-dB.
 * @param low_gain_dB Specifies the low gain for TX monitor in dB.
 * @param high_gain_dB Specifies the high gain for TX monitor in dB.
 * @param tx_mon_track_en Enables or disables DC tracking for TX monitor.
 * @param one_shot_mode_en Enables or disables one-shot mode for TX monitor.
 * @param tx_mon_delay Specifies the delay for TX monitor.
 * @param tx_mon_duration Specifies the duration for TX monitor.
 * @param tx1_mon_front_end_gain Specifies the front-end gain for TX1 monitor.
 * @param tx2_mon_front_end_gain Specifies the front-end gain for TX2 monitor.
 * @param tx1_mon_lo_cm Specifies the LO common mode for TX1 monitor.
 * @param tx2_mon_lo_cm Specifies the LO common mode for TX2 monitor.
 * @param gpio_resetb GPIO parameter for reset control.
 * @param gpio_sync GPIO parameter for sync control.
 * @param gpio_cal_sw1 GPIO parameter for calibration switch 1.
 * @param gpio_cal_sw2 GPIO parameter for calibration switch 2.
 * @param spi_param SPI initialization parameters.
 * @param ad9361_rfpll_ext_recalc_rate Function pointer for recalculating RFPLL
 * external rate.
 * @param ad9361_rfpll_ext_round_rate Function pointer for rounding RFPLL
 * external rate.
 * @param ad9361_rfpll_ext_set_rate Function pointer for setting RFPLL external
 * rate.
 * @param rx_adc_init Pointer to RX ADC initialization structure.
 * @param tx_dac_init Pointer to TX DAC initialization structure.
 ******************************************************************************/
typedef struct {
	/* Device selection */
	enum dev_id	dev_sel;
	/* Reference Clock */
	uint32_t	reference_clk_rate;
	/* Base Configuration */
	uint8_t		two_rx_two_tx_mode_enable;	/* adi,2rx-2tx-mode-enable */
	uint8_t		one_rx_one_tx_mode_use_rx_num;	/* adi,1rx-1tx-mode-use-rx-num */
	uint8_t		one_rx_one_tx_mode_use_tx_num;	/* adi,1rx-1tx-mode-use-tx-num */
	uint8_t		frequency_division_duplex_mode_enable;	/* adi,frequency-division-duplex-mode-enable */
	uint8_t		frequency_division_duplex_independent_mode_enable;	/* adi,frequency-division-duplex-independent-mode-enable */
	uint8_t		tdd_use_dual_synth_mode_enable;	/* adi,tdd-use-dual-synth-mode-enable */
	uint8_t		tdd_skip_vco_cal_enable;		/* adi,tdd-skip-vco-cal-enable */
	uint32_t	tx_fastlock_delay_ns;	/* adi,tx-fastlock-delay-ns */
	uint32_t	rx_fastlock_delay_ns;	/* adi,rx-fastlock-delay-ns */
	uint8_t		rx_fastlock_pincontrol_enable;	/* adi,rx-fastlock-pincontrol-enable */
	uint8_t		tx_fastlock_pincontrol_enable;	/* adi,tx-fastlock-pincontrol-enable */
	uint8_t		external_rx_lo_enable;	/* adi,external-rx-lo-enable */
	uint8_t		external_tx_lo_enable;	/* adi,external-tx-lo-enable */
	uint8_t		dc_offset_tracking_update_event_mask;	/* adi,dc-offset-tracking-update-event-mask */
	uint8_t		dc_offset_attenuation_high_range;	/* adi,dc-offset-attenuation-high-range */
	uint8_t		dc_offset_attenuation_low_range;	/* adi,dc-offset-attenuation-low-range */
	uint8_t		dc_offset_count_high_range;			/* adi,dc-offset-count-high-range */
	uint8_t		dc_offset_count_low_range;			/* adi,dc-offset-count-low-range */
	uint8_t		split_gain_table_mode_enable;	/* adi,split-gain-table-mode-enable */
	uint32_t	trx_synthesizer_target_fref_overwrite_hz;	/* adi,trx-synthesizer-target-fref-overwrite-hz */
	uint8_t		qec_tracking_slow_mode_enable;	/* adi,qec-tracking-slow-mode-enable */
	/* ENSM Control */
	uint8_t		ensm_enable_pin_pulse_mode_enable;	/* adi,ensm-enable-pin-pulse-mode-enable */
	uint8_t		ensm_enable_txnrx_control_enable;	/* adi,ensm-enable-txnrx-control-enable */
	/* LO Control */
	uint64_t	rx_synthesizer_frequency_hz;	/* adi,rx-synthesizer-frequency-hz */
	uint64_t	tx_synthesizer_frequency_hz;	/* adi,tx-synthesizer-frequency-hz */
	uint8_t		tx_lo_powerdown_managed_enable;	/* adi,tx-lo-powerdown-managed-enable */
	/* Rate & BW Control */
	uint32_t	rx_path_clock_frequencies[6];	/* adi,rx-path-clock-frequencies */
	uint32_t	tx_path_clock_frequencies[6];	/* adi,tx-path-clock-frequencies */
	uint32_t	rf_rx_bandwidth_hz;	/* adi,rf-rx-bandwidth-hz */
	uint32_t	rf_tx_bandwidth_hz;	/* adi,rf-tx-bandwidth-hz */
	/* RF Port Control */
	uint32_t	rx_rf_port_input_select;	/* adi,rx-rf-port-input-select */
	uint32_t	tx_rf_port_input_select;	/* adi,tx-rf-port-input-select */
	/* TX Attenuation Control */
	int32_t		tx_attenuation_mdB;	/* adi,tx-attenuation-mdB */
	uint8_t		update_tx_gain_in_alert_enable;	/* adi,update-tx-gain-in-alert-enable */
	/* Reference Clock Control */
	uint8_t		xo_disable_use_ext_refclk_enable;	/* adi,xo-disable-use-ext-refclk-enable */
	uint32_t	dcxo_coarse_and_fine_tune[2];	/* adi,dcxo-coarse-and-fine-tune */
	uint32_t	clk_output_mode_select;		/* adi,clk-output-mode-select */
	/* Gain Control */
	uint8_t		gc_rx1_mode;	/* adi,gc-rx1-mode */
	uint8_t		gc_rx2_mode;	/* adi,gc-rx2-mode */
	uint8_t		gc_adc_large_overload_thresh;	/* adi,gc-adc-large-overload-thresh */
	uint8_t		gc_adc_ovr_sample_size;	/* adi,gc-adc-ovr-sample-size */
	uint8_t		gc_adc_small_overload_thresh;	/* adi,gc-adc-small-overload-thresh */
	uint16_t	gc_dec_pow_measurement_duration;	/* adi,gc-dec-pow-measurement-duration */
	uint8_t		gc_dig_gain_enable;	/* adi,gc-dig-gain-enable */
	uint16_t	gc_lmt_overload_high_thresh;	/* adi,gc-lmt-overload-high-thresh */
	uint16_t	gc_lmt_overload_low_thresh;	/* adi,gc-lmt-overload-low-thresh */
	uint8_t		gc_low_power_thresh;	/* adi,gc-low-power-thresh */
	uint8_t		gc_max_dig_gain;	/* adi,gc-max-dig-gain */
	uint8_t		gc_use_rx_fir_out_for_dec_pwr_meas_enable;	/* adi,gc-use-rx-fir-out-for-dec-pwr-meas-enable */
	/* Gain MGC Control */
	uint8_t		mgc_dec_gain_step;	/* adi,mgc-dec-gain-step */
	uint8_t		mgc_inc_gain_step;	/* adi,mgc-inc-gain-step */
	uint8_t		mgc_rx1_ctrl_inp_enable;	/* adi,mgc-rx1-ctrl-inp-enable */
	uint8_t		mgc_rx2_ctrl_inp_enable;	/* adi,mgc-rx2-ctrl-inp-enable */
	uint8_t		mgc_split_table_ctrl_inp_gain_mode;	/* adi,mgc-split-table-ctrl-inp-gain-mode */
	/* Gain AGC Control */
	uint8_t		agc_adc_large_overload_exceed_counter;	/* adi,agc-adc-large-overload-exceed-counter */
	uint8_t		agc_adc_large_overload_inc_steps;	/* adi,agc-adc-large-overload-inc-steps - Name is misleading should be dec-steps*/
	uint8_t		agc_adc_lmt_small_overload_prevent_gain_inc_enable;	/* adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable */
	uint8_t		agc_adc_small_overload_exceed_counter;	/* adi,agc-adc-small-overload-exceed-counter */
	uint8_t		agc_dig_gain_step_size;	/* adi,agc-dig-gain-step-size */
	uint8_t		agc_dig_saturation_exceed_counter;	/* adi,agc-dig-saturation-exceed-counter */
	uint32_t	agc_gain_update_interval_us; /* adi,agc-gain-update-interval-us */
	uint8_t		agc_immed_gain_change_if_large_adc_overload_enable;	/* adi,agc-immed-gain-change-if-large-adc-overload-enable */
	uint8_t		agc_immed_gain_change_if_large_lmt_overload_enable;	/* adi,agc-immed-gain-change-if-large-lmt-overload-enable */
	uint8_t		agc_inner_thresh_high;	/* adi,agc-inner-thresh-high */
	uint8_t		agc_inner_thresh_high_dec_steps;	/* adi,agc-inner-thresh-high-dec-steps */
	uint8_t		agc_inner_thresh_low;	/* adi,agc-inner-thresh-low */
	uint8_t		agc_inner_thresh_low_inc_steps;	/* adi,agc-inner-thresh-low-inc-steps */
	uint8_t		agc_lmt_overload_large_exceed_counter;	/* adi,agc-lmt-overload-large-exceed-counter */
	uint8_t		agc_lmt_overload_large_inc_steps;	/* adi,agc-lmt-overload-large-inc-steps */
	uint8_t		agc_lmt_overload_small_exceed_counter;	/* adi,agc-lmt-overload-small-exceed-counter */
	uint8_t		agc_outer_thresh_high;	/* adi,agc-outer-thresh-high */
	uint8_t		agc_outer_thresh_high_dec_steps;	/* adi,agc-outer-thresh-high-dec-steps */
	uint8_t		agc_outer_thresh_low;	/* adi,agc-outer-thresh-low */
	uint8_t		agc_outer_thresh_low_inc_steps;	/* adi,agc-outer-thresh-low-inc-steps */
	uint32_t	agc_attack_delay_extra_margin_us;	/* adi,agc-attack-delay-extra-margin-us */
	uint8_t		agc_sync_for_gain_counter_enable;	/* adi,agc-sync-for-gain-counter-enable */
	/* Fast AGC */
	uint32_t	fagc_dec_pow_measuremnt_duration;	/* adi,fagc-dec-pow-measurement-duration */
	uint32_t	fagc_state_wait_time_ns;	/* adi,fagc-state-wait-time-ns */
	/* Fast AGC - Low Power */
	uint8_t		fagc_allow_agc_gain_increase;	/* adi,fagc-allow-agc-gain-increase-enable */
	uint32_t	fagc_lp_thresh_increment_time;	/* adi,fagc-lp-thresh-increment-time */
	uint32_t	fagc_lp_thresh_increment_steps;	/* adi,fagc-lp-thresh-increment-steps */
	/* Fast AGC - Lock Level (Lock Level is set via slow AGC inner high threshold) */
	uint8_t		fagc_lock_level_lmt_gain_increase_en;	/* adi,fagc-lock-level-lmt-gain-increase-enable */
	uint32_t	fagc_lock_level_gain_increase_upper_limit;	/* adi,fagc-lock-level-gain-increase-upper-limit */
	/* Fast AGC - Peak Detectors and Final Settling */
	uint32_t	fagc_lpf_final_settling_steps;	/* adi,fagc-lpf-final-settling-steps */
	uint32_t	fagc_lmt_final_settling_steps;	/* adi,fagc-lmt-final-settling-steps */
	uint32_t	fagc_final_overrange_count;	/* adi,fagc-final-overrange-count */
	/* Fast AGC - Final Power Test */
	uint8_t		fagc_gain_increase_after_gain_lock_en;	/* adi,fagc-gain-increase-after-gain-lock-enable */
	/* Fast AGC - Unlocking the Gain */
	uint32_t	fagc_gain_index_type_after_exit_rx_mode;	/* adi,fagc-gain-index-type-after-exit-rx-mode */
	uint8_t		fagc_use_last_lock_level_for_set_gain_en;	/* adi,fagc-use-last-lock-level-for-set-gain-enable */
	uint8_t		fagc_rst_gla_stronger_sig_thresh_exceeded_en;	/* adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable */
	uint32_t	fagc_optimized_gain_offset;	/* adi,fagc-optimized-gain-offset */
	uint32_t	fagc_rst_gla_stronger_sig_thresh_above_ll;	/* adi,fagc-rst-gla-stronger-sig-thresh-above-ll */
	uint8_t		fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en;	/* adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable */
	uint8_t		fagc_rst_gla_engergy_lost_goto_optim_gain_en;	/* adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable */
	uint32_t	fagc_rst_gla_engergy_lost_sig_thresh_below_ll;	/* adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll */
	uint32_t	fagc_energy_lost_stronger_sig_gain_lock_exit_cnt;	/* adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt */
	uint8_t		fagc_rst_gla_large_adc_overload_en;	/* adi,fagc-rst-gla-large-adc-overload-enable */
	uint8_t		fagc_rst_gla_large_lmt_overload_en;	/* adi,fagc-rst-gla-large-lmt-overload-enable */
	uint8_t		fagc_rst_gla_en_agc_pulled_high_en;	/* adi,fagc-rst-gla-en-agc-pulled-high-enable */
	uint32_t	fagc_rst_gla_if_en_agc_pulled_high_mode;	/* adi,fagc-rst-gla-if-en-agc-pulled-high-mode */
	uint32_t	fagc_power_measurement_duration_in_state5;	/* adi,fagc-power-measurement-duration-in-state5 */
	uint32_t	fagc_large_overload_inc_steps;	/* adi,fagc-adc-large-overload-inc-steps - Name is misleading should be dec-steps */
	/* RSSI Control */
	uint32_t	rssi_delay;	/* adi,rssi-delay */
	uint32_t	rssi_duration;	/* adi,rssi-duration */
	uint8_t		rssi_restart_mode;	/* adi,rssi-restart-mode */
	uint8_t		rssi_unit_is_rx_samples_enable;	/* adi,rssi-unit-is-rx-samples-enable */
	uint32_t	rssi_wait;	/* adi,rssi-wait */
	/* Aux ADC Control */
	uint32_t	aux_adc_decimation;	/* adi,aux-adc-decimation */
	uint32_t	aux_adc_rate;	/* adi,aux-adc-rate */
	/* AuxDAC Control */
	uint8_t		aux_dac_manual_mode_enable;	/* adi,aux-dac-manual-mode-enable */
	uint32_t	aux_dac1_default_value_mV;	/* adi,aux-dac1-default-value-mV */
	uint8_t		aux_dac1_active_in_rx_enable;	/* adi,aux-dac1-active-in-rx-enable */
	uint8_t		aux_dac1_active_in_tx_enable;	/* adi,aux-dac1-active-in-tx-enable */
	uint8_t		aux_dac1_active_in_alert_enable;	/* adi,aux-dac1-active-in-alert-enable */
	uint32_t	aux_dac1_rx_delay_us;	/* adi,aux-dac1-rx-delay-us */
	uint32_t	aux_dac1_tx_delay_us;	/* adi,aux-dac1-tx-delay-us */
	uint32_t	aux_dac2_default_value_mV;	/* adi,aux-dac2-default-value-mV */
	uint8_t		aux_dac2_active_in_rx_enable;	/* adi,aux-dac2-active-in-rx-enable */
	uint8_t		aux_dac2_active_in_tx_enable;	/* adi,aux-dac2-active-in-tx-enable */
	uint8_t		aux_dac2_active_in_alert_enable;	/* adi,aux-dac2-active-in-alert-enable */
	uint32_t	aux_dac2_rx_delay_us;	/* adi,aux-dac2-rx-delay-us */
	uint32_t	aux_dac2_tx_delay_us;	/* adi,aux-dac2-tx-delay-us */
	/* Temperature Sensor Control */
	uint32_t	temp_sense_decimation;	/* adi,temp-sense-decimation */
	uint16_t	temp_sense_measurement_interval_ms;	/* adi,temp-sense-measurement-interval-ms */
	int8_t		temp_sense_offset_signed;	/* adi,temp-sense-offset-signed */
	uint8_t		temp_sense_periodic_measurement_enable;	/* adi,temp-sense-periodic-measurement-enable */
	/* Control Out Setup */
	uint8_t		ctrl_outs_enable_mask;	/* adi,ctrl-outs-enable-mask */
	uint8_t		ctrl_outs_index;	/* adi,ctrl-outs-index */
	/* External LNA Control */
	uint32_t	elna_settling_delay_ns;	/* adi,elna-settling-delay-ns */
	uint32_t	elna_gain_mdB;	/* adi,elna-gain-mdB */
	uint32_t	elna_bypass_loss_mdB;	/* adi,elna-bypass-loss-mdB */
	uint8_t		elna_rx1_gpo0_control_enable;	/* adi,elna-rx1-gpo0-control-enable */
	uint8_t		elna_rx2_gpo1_control_enable;	/* adi,elna-rx2-gpo1-control-enable */
	uint8_t		elna_gaintable_all_index_enable;	/* adi,elna-gaintable-all-index-enable */
	/* Digital Interface Control */
	uint8_t		digital_interface_tune_skip_mode;	/* adi,digital-interface-tune-skip-mode */
	uint8_t		digital_interface_tune_fir_disable;	/* adi,digital-interface-tune-fir-disable */
	uint8_t		pp_tx_swap_enable;	/* adi,pp-tx-swap-enable */
	uint8_t		pp_rx_swap_enable;	/* adi,pp-rx-swap-enable */
	uint8_t		tx_channel_swap_enable;	/* adi,tx-channel-swap-enable */
	uint8_t		rx_channel_swap_enable;	/* adi,rx-channel-swap-enable */
	uint8_t		rx_frame_pulse_mode_enable;	/* adi,rx-frame-pulse-mode-enable */
	uint8_t		two_t_two_r_timing_enable;	/* adi,2t2r-timing-enable */
	uint8_t		invert_data_bus_enable;	/* adi,invert-data-bus-enable */
	uint8_t		invert_data_clk_enable;	/* adi,invert-data-clk-enable */
	uint8_t		fdd_alt_word_order_enable;	/* adi,fdd-alt-word-order-enable */
	uint8_t		invert_rx_frame_enable;	/* adi,invert-rx-frame-enable */
	uint8_t		fdd_rx_rate_2tx_enable;	/* adi,fdd-rx-rate-2tx-enable */
	uint8_t		swap_ports_enable;	/* adi,swap-ports-enable */
	uint8_t		single_data_rate_enable;	/* adi,single-data-rate-enable */
	uint8_t		lvds_mode_enable;	/* adi,lvds-mode-enable */
	uint8_t		half_duplex_mode_enable;	/* adi,half-duplex-mode-enable */
	uint8_t		single_port_mode_enable;	/* adi,single-port-mode-enable */
	uint8_t		full_port_enable;	/* adi,full-port-enable */
	uint8_t		full_duplex_swap_bits_enable;	/* adi,full-duplex-swap-bits-enable */
	uint32_t	delay_rx_data;	/* adi,delay-rx-data */
	uint32_t	rx_data_clock_delay;	/* adi,rx-data-clock-delay */
	uint32_t	rx_data_delay;	/* adi,rx-data-delay */
	uint32_t	tx_fb_clock_delay;	/* adi,tx-fb-clock-delay */
	uint32_t	tx_data_delay;	/* adi,tx-data-delay */
	uint32_t	lvds_bias_mV;	/* adi,lvds-bias-mV */
	uint8_t		lvds_rx_onchip_termination_enable;	/* adi,lvds-rx-onchip-termination-enable */
	uint8_t		rx1rx2_phase_inversion_en;	/* adi,rx1-rx2-phase-inversion-enable */
	uint8_t		lvds_invert1_control;	/* adi,lvds-invert1-control */
	uint8_t		lvds_invert2_control;	/* adi,lvds-invert2-control */
	/* GPO Control */
	uint8_t		gpo_manual_mode_enable;			/* adi,gpo-manual-mode-enable */
	uint32_t	gpo_manual_mode_enable_mask;	/* adi,gpo-manual-mode-enable-mask */
	uint8_t		gpo0_inactive_state_high_enable;	/* adi,gpo0-inactive-state-high-enable */
	uint8_t		gpo1_inactive_state_high_enable;	/* adi,gpo1-inactive-state-high-enable */
	uint8_t		gpo2_inactive_state_high_enable;	/* adi,gpo2-inactive-state-high-enable */
	uint8_t		gpo3_inactive_state_high_enable;	/* adi,gpo3-inactive-state-high-enable */
	uint8_t		gpo0_slave_rx_enable;	/* adi,gpo0-slave-rx-enable */
	uint8_t		gpo0_slave_tx_enable;	/* adi,gpo0-slave-tx-enable */
	uint8_t		gpo1_slave_rx_enable;	/* adi,gpo1-slave-rx-enable */
	uint8_t		gpo1_slave_tx_enable;	/* adi,gpo1-slave-tx-enable */
	uint8_t		gpo2_slave_rx_enable;	/* adi,gpo2-slave-rx-enable */
	uint8_t		gpo2_slave_tx_enable;	/* adi,gpo2-slave-tx-enable */
	uint8_t		gpo3_slave_rx_enable;	/* adi,gpo3-slave-rx-enable */
	uint8_t		gpo3_slave_tx_enable;	/* adi,gpo3-slave-tx-enable */
	uint8_t		gpo0_rx_delay_us;	/* adi,gpo0-rx-delay-us */
	uint8_t		gpo0_tx_delay_us;	/* adi,gpo0-tx-delay-us */
	uint8_t		gpo1_rx_delay_us;	/* adi,gpo1-rx-delay-us */
	uint8_t		gpo1_tx_delay_us;	/* adi,gpo1-tx-delay-us */
	uint8_t		gpo2_rx_delay_us;	/* adi,gpo2-rx-delay-us */
	uint8_t		gpo2_tx_delay_us;	/* adi,gpo2-tx-delay-us */
	uint8_t		gpo3_rx_delay_us;	/* adi,gpo3-rx-delay-us */
	uint8_t		gpo3_tx_delay_us;	/* adi,gpo3-tx-delay-us */
	/* Tx Monitor Control */
	uint32_t	low_high_gain_threshold_mdB;	/* adi,txmon-low-high-thresh */
	uint32_t	low_gain_dB;	/* adi,txmon-low-gain */
	uint32_t	high_gain_dB;	/* adi,txmon-high-gain */
	uint8_t		tx_mon_track_en;	/* adi,txmon-dc-tracking-enable */
	uint8_t		one_shot_mode_en;	/* adi,txmon-one-shot-mode-enable */
	uint32_t	tx_mon_delay;	/* adi,txmon-delay */
	uint32_t	tx_mon_duration;	/* adi,txmon-duration */
	uint32_t	tx1_mon_front_end_gain;	/* adi,txmon-1-front-end-gain */
	uint32_t	tx2_mon_front_end_gain;	/* adi,txmon-2-front-end-gain */
	uint32_t	tx1_mon_lo_cm;	/* adi,txmon-1-lo-cm */
	uint32_t	tx2_mon_lo_cm;	/* adi,txmon-2-lo-cm */
	/* GPIO definitions */
	struct no_os_gpio_init_param	gpio_resetb;	/* reset-gpios */
	/* MCS Sync */
	struct no_os_gpio_init_param	gpio_sync;	/* sync-gpios */
	struct no_os_gpio_init_param	gpio_cal_sw1;	/* cal-sw1-gpios */
	struct no_os_gpio_init_param	gpio_cal_sw2;	/* cal-sw2-gpios */

	struct no_os_spi_init_param	spi_param;

	/* External LO clocks */
	uint32_t	(*ad9361_rfpll_ext_recalc_rate)(struct refclk_scale *clk_priv);
	int32_t	(*ad9361_rfpll_ext_round_rate)(struct refclk_scale *clk_priv,
					       uint32_t rate);
	int32_t	(*ad9361_rfpll_ext_set_rate)(struct refclk_scale *clk_priv,
					     uint32_t rate);
#ifndef AXI_ADC_NOT_PRESENT
	struct axi_adc_init	*rx_adc_init;
	struct axi_dac_init	*tx_dac_init;
#endif
} AD9361_InitParam;

/***************************************************************************//**
 * @brief The AD9361_RXFIRConfig structure is used to configure the receive FIR
 * filter settings for the AD9361 transceiver. It includes parameters for
 * selecting the receive channel, setting the receive gain and decimation
 * factor, and defining the FIR filter coefficients and their size.
 * Additionally, it specifies the receive path clock frequencies and the
 * receive bandwidth, allowing for detailed customization of the receive
 * signal processing chain.
 *
 * @param rx Specifies the receive channel selection, with possible values 1, 2,
 * or 3 (both).
 * @param rx_gain Defines the receive gain with possible values of -12, -6, 0,
 * or 6.
 * @param rx_dec Indicates the receive decimation factor, which can be 1, 2, or
 * 4.
 * @param rx_coef An array of 128 coefficients used for the receive FIR filter.
 * @param rx_coef_size Specifies the number of coefficients used in the rx_coef
 * array.
 * @param rx_path_clks An array of 6 elements representing the receive path
 * clock frequencies.
 * @param rx_bandwidth Defines the receive bandwidth in Hz.
 ******************************************************************************/
typedef struct {
	uint32_t	rx;				/* 1, 2, 3(both) */
	int32_t		rx_gain;		/* -12, -6, 0, 6 */
	uint32_t	rx_dec;			/* 1, 2, 4 */
	int16_t		rx_coef[128];
	uint8_t		rx_coef_size;
	uint32_t	rx_path_clks[6];
	uint32_t	rx_bandwidth;
} AD9361_RXFIRConfig;

/***************************************************************************//**
 * @brief The `AD9361_TXFIRConfig` structure is used to configure the
 * transmission finite impulse response (FIR) filter settings for the
 * AD9361 transceiver. It includes parameters for selecting the
 * transmission path, setting the gain and interpolation factor, and
 * defining the filter coefficients and their size. Additionally, it
 * specifies the clock frequencies for the transmission path and the
 * overall transmission bandwidth, allowing for detailed customization of
 * the transmission characteristics.
 *
 * @param tx Specifies the transmission path, with possible values 1, 2, or 3
 * (both).
 * @param tx_gain Defines the transmission gain, with possible values -6 or 0.
 * @param tx_int Indicates the transmission interpolation factor, with possible
 * values 1, 2, or 4.
 * @param tx_coef An array of 128 coefficients for the transmission filter.
 * @param tx_coef_size Specifies the number of coefficients used in the
 * transmission filter.
 * @param tx_path_clks An array of 6 clock frequencies for the transmission
 * path.
 * @param tx_bandwidth Defines the transmission bandwidth.
 ******************************************************************************/
typedef struct {
	uint32_t	tx;				/* 1, 2, 3(both) */
	int32_t		tx_gain;		/* -6, 0 */
	uint32_t	tx_int;			/* 1, 2, 4 */
	int16_t		tx_coef[128];
	uint8_t		tx_coef_size;
	uint32_t	tx_path_clks[6];
	uint32_t	tx_bandwidth;
} AD9361_TXFIRConfig;

/***************************************************************************//**
 * @brief The `ad9361_ensm_mode` enumeration defines the various operational
 * modes of the Enable State Machine (ENSM) for the AD9361 device. Each
 * mode corresponds to a specific state or operation that the device can
 * be in, such as transmitting, receiving, or being in an alert state.
 * This enumeration is crucial for controlling the state transitions of
 * the AD9361, allowing for flexible and precise management of its
 * operational states.
 *
 * @param ENSM_MODE_TX Represents the transmit mode of the Enable State Machine.
 * @param ENSM_MODE_RX Represents the receive mode of the Enable State Machine.
 * @param ENSM_MODE_ALERT Represents the alert mode of the Enable State Machine.
 * @param ENSM_MODE_FDD Represents the frequency division duplex mode of the
 * Enable State Machine.
 * @param ENSM_MODE_WAIT Represents the wait mode of the Enable State Machine.
 * @param ENSM_MODE_SLEEP Represents the sleep mode of the Enable State Machine.
 * @param ENSM_MODE_PINCTRL Represents the pin control mode of the Enable State
 * Machine.
 * @param ENSM_MODE_PINCTRL_FDD_INDEP Represents the pin control mode with
 * frequency division duplex independence of
 * the Enable State Machine.
 ******************************************************************************/
enum ad9361_ensm_mode {
	ENSM_MODE_TX,
	ENSM_MODE_RX,
	ENSM_MODE_ALERT,
	ENSM_MODE_FDD,
	ENSM_MODE_WAIT,
	ENSM_MODE_SLEEP,
	ENSM_MODE_PINCTRL,
	ENSM_MODE_PINCTRL_FDD_INDEP,
};

#define ENABLE		1
#define DISABLE		0

#define RX1			0
#define RX2			1

#define TX1			0
#define TX2			1

#define A_BALANCED	0
#define B_BALANCED	1
#define C_BALANCED	2
#define A_N			3
#define A_P			4
#define B_N			5
#define B_P			6
#define C_N			7
#define C_P			8
#define TX_MON1		9
#define TX_MON2		10
#define TX_MON1_2	11

#define TXA			0
#define TXB			1

#define MODE_1x1	1
#define MODE_2x2	2

#define HIGHEST_OSR	0
#define NOMINAL_OSR	1

#define INT_LO		0
#define EXT_LO		1

#define ON			0
#define OFF			1

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the AD9361 part. */
/***************************************************************************//**
 * @brief This function is used to initialize the AD9361 RF transceiver with the
 * specified parameters. It must be called before any other operations on
 * the transceiver can be performed. The `init_param` structure must be
 * properly populated with valid configuration values, including device
 * selection and clock rates. If memory allocation fails at any point
 * during initialization, the function will return an error code. The
 * caller is responsible for ensuring that the provided pointers are
 * valid and that the memory for the `ad9361_phy` pointer is properly
 * managed after initialization.
 *
 * @param ad9361_phy A pointer to a pointer of type `struct ad9361_rf_phy`. This
 * will be set to point to the initialized transceiver
 * structure. Must not be null.
 * @param init_param A pointer to an `AD9361_InitParam` structure containing the
 * initialization parameters. Must not be null and must be
 * properly populated with valid values.
 * @return Returns 0 on successful initialization, or a negative error code
 * indicating the type of failure (e.g., -ENOMEM for memory allocation
 * failure, -ENODEV for unsupported device ID).
 ******************************************************************************/
int32_t ad9361_init(struct ad9361_rf_phy **ad9361_phy,
		    AD9361_InitParam *init_param);
/* Free the allocated resources. */
/***************************************************************************//**
 * @brief This function should be called to release all resources associated
 * with the `ad9361_rf_phy` structure after it is no longer needed. It is
 * essential to invoke this function to prevent memory leaks and ensure
 * proper cleanup of hardware resources. The function assumes that the
 * `phy` parameter has been properly initialized and is not null. It will
 * handle the deallocation of various components associated with the
 * AD9361 device, including GPIOs and SPI interfaces.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure that represents the
 * AD9361 device. This pointer must not be null and should point to a
 * valid initialized instance. If an invalid pointer is provided, the
 * behavior is undefined.
 * @return Returns 0 on success, indicating that resources were successfully
 * freed. If there are issues during the cleanup process, the return
 * value may not reflect the error, as the function does not provide
 * specific error codes.
 ******************************************************************************/
int32_t ad9361_remove(struct ad9361_rf_phy *phy);
/* Set the Enable State Machine (ENSM) mode. */
/***************************************************************************//**
 * @brief This function is used to configure the Enable State Machine (ENSM)
 * mode of the AD9361 device. It should be called after the device has
 * been initialized and is ready for operation. The `mode` parameter
 * determines the specific state to which the ENSM should transition,
 * such as TX, RX, ALERT, FDD, WAIT, SLEEP, or specific pin control
 * modes. If an invalid mode is provided, the function will return an
 * error code. It is important to ensure that the device is in a state
 * that allows for mode changes, as certain modes may have specific
 * requirements or restrictions.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null and the caller retains ownership.
 * @param mode An unsigned integer representing the desired ENSM mode. Valid
 * values are defined in the `ad9361_ensm_mode` enumeration. If an
 * invalid value is provided, the function will return an error
 * code.
 * @return Returns a status code indicating success (0) or an error code (e.g.,
 * -EINVAL for invalid input).
 ******************************************************************************/
int32_t ad9361_set_en_state_machine_mode(struct ad9361_rf_phy *phy,
		uint32_t mode);
/* Get the Enable State Machine (ENSM) mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current mode of the Enable State
 * Machine (ENSM) for the specified `ad9361_rf_phy` instance. It should
 * be called after the device has been initialized and configured. The
 * function will write the current mode to the provided pointer, which
 * must not be null. If the ENSM state is not recognized, the function
 * will return an error code. It is important to ensure that the `phy`
 * parameter is valid and properly initialized before calling this
 * function.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the device.
 * Must not be null and should be initialized before use.
 * @param mode A pointer to a `uint32_t` where the current ENSM mode will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the ENSM state is
 * unrecognized.
 ******************************************************************************/
int32_t ad9361_get_en_state_machine_mode(struct ad9361_rf_phy *phy,
		uint32_t *mode);
/* Set the receive RF gain for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to configure the receive RF gain for a specific
 * channel in the AD9361 RF transceiver. It should be called after the
 * transceiver has been initialized and is ready for configuration. The
 * gain value is specified in decibels (dB) and can be adjusted to
 * optimize the signal quality. It is important to ensure that the
 * specified channel is valid and that the gain value is within the
 * acceptable range for the device. If the function is called with
 * invalid parameters, it may return an error code.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver. This parameter must not be null and is owned by the
 * caller.
 * @param ch An 8-bit unsigned integer representing the channel number (0 or 1)
 * for which the gain is being set. Valid values are 0 for RX1 and 1
 * for RX2.
 * @param gain_db An integer representing the desired gain in decibels. The
 * valid range for this parameter is typically defined by the
 * hardware specifications, and the function may return an error
 * if the value is out of range.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_rx_rf_gain(struct ad9361_rf_phy *phy, uint8_t ch,
			      int32_t gain_db);
/* Get current receive RF gain for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to obtain the current RF gain setting for a
 * specified receive channel in the AD9361 device. It should be called
 * after the device has been properly initialized and configured. The
 * function expects a valid pointer to an `ad9361_rf_phy` structure
 * representing the device, a channel identifier, and a pointer to an
 * integer where the gain value will be stored. If the specified channel
 * is invalid or if the device is not properly initialized, the function
 * may return an error code.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param ch An 8-bit unsigned integer representing the channel number (0 or 1).
 * Must be within the valid range of channels supported by the device.
 * @param gain_db A pointer to an integer where the gain value in dB will be
 * stored. Must not be null; the function will write the gain
 * value to this location.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_get_rx_rf_gain(struct ad9361_rf_phy *phy, uint8_t ch,
			      int32_t *gain_db);
/* Set the RX RF bandwidth. */
/***************************************************************************//**
 * @brief This function is used to configure the receive RF bandwidth of the
 * specified `phy` device. It should be called after the device has been
 * initialized and is ready for configuration. The `bandwidth_hz`
 * parameter must be a valid bandwidth value as determined by the
 * device's specifications. If the requested bandwidth is the same as the
 * current setting, no action is taken. The function will return an error
 * code if the provided bandwidth is invalid.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param bandwidth_hz The desired RX RF bandwidth in Hertz. Must be a valid
 * value as per the device specifications. If an invalid
 * value is provided, it will be adjusted to a valid range.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t bandwidth_hz);
/* Get the RX RF bandwidth. */
/***************************************************************************//**
 * @brief This function is used to obtain the current receive radio frequency
 * bandwidth setting of the specified `ad9361_rf_phy` instance. It should
 * be called after the device has been properly initialized and
 * configured. The function writes the bandwidth value in Hertz to the
 * location pointed to by the `bandwidth_hz` parameter. It is important
 * to ensure that the `bandwidth_hz` pointer is not null before calling
 * this function, as passing a null pointer will lead to undefined
 * behavior.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the device
 * instance. This must not be null and should point to a valid
 * initialized device.
 * @param bandwidth_hz A pointer to a `uint32_t` variable where the current RX
 * RF bandwidth will be stored. This pointer must not be
 * null.
 * @return Returns 0 on success, indicating that the bandwidth has been
 * successfully retrieved and stored in the provided pointer. If the
 * function fails, it may return an error code, but this is not
 * specified in the provided API.
 ******************************************************************************/
int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t *bandwidth_hz);
/* Set the RX sampling frequency. */
/***************************************************************************//**
 * @brief This function is used to configure the receive sampling frequency of
 * the AD9361 device. It should be called after the device has been
 * initialized and before starting any data transmission or reception.
 * The function takes a frequency value in Hertz and updates the internal
 * settings accordingly. If the provided frequency is invalid or if the
 * device is not properly initialized, the function may return an error
 * code.
 *
 * @param phy A pointer to the `struct ad9361_rf_phy` representing the AD9361
 * device. Must not be null.
 * @param sampling_freq_hz The desired RX sampling frequency in Hertz. This
 * value must be within the valid range supported by the
 * device. If the value is out of range, the function
 * will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t sampling_freq_hz);
/* Get current RX sampling frequency. */
/***************************************************************************//**
 * @brief This function is used to obtain the current receive sampling frequency
 * of the AD9361 device. It should be called after the device has been
 * properly initialized and configured. The function writes the sampling
 * frequency in Hertz to the provided pointer. If the pointer is null,
 * the behavior is undefined, and the function may not execute correctly.
 *
 * @param phy A pointer to the `struct ad9361_rf_phy` representing the AD9361
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param sampling_freq_hz A pointer to a `uint32_t` where the function will
 * store the current RX sampling frequency in Hertz.
 * This pointer must not be null; otherwise, the
 * function may not behave as expected.
 * @return Returns 0 on success, indicating that the sampling frequency was
 * successfully retrieved and stored in the provided pointer. If the
 * function fails, it may return an error code, but specific error
 * handling is not detailed in the API.
 ******************************************************************************/
int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t *sampling_freq_hz);
/* Set the RX LO frequency. */
/***************************************************************************//**
 * @brief This function is used to configure the receive local oscillator (LO)
 * frequency for the specified RF PHY device. It should be called after
 * the device has been properly initialized. The frequency is specified
 * in Hertz and must be within the valid range supported by the device.
 * If an invalid frequency is provided, the function will return an error
 * code, indicating the failure to set the frequency.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF PHY
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param lo_freq_hz The desired local oscillator frequency in Hertz. This value
 * must be within the valid operational range of the device.
 * If the value is out of range, the function will return an
 * error code.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *phy, uint64_t lo_freq_hz);
/* Get current RX LO frequency. */
/***************************************************************************//**
 * @brief This function is used to obtain the current local oscillator frequency
 * for the RX path of the AD9361 device. It should be called after the
 * device has been properly initialized and configured. The function
 * expects a valid pointer to a `struct ad9361_rf_phy` instance, which
 * represents the device context, and a pointer to a `uint64_t` variable
 * where the frequency will be stored. If the provided pointers are
 * valid, the function will write the frequency in Hertz to the location
 * pointed to by `lo_freq_hz`.
 *
 * @param phy A pointer to a `struct ad9361_rf_phy` that represents the device
 * context. Must not be null.
 * @param lo_freq_hz A pointer to a `uint64_t` variable where the RX local
 * oscillator frequency will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the frequency has been
 * successfully retrieved and stored in `lo_freq_hz`. If the function
 * fails, it may return a negative error code, but specific error
 * handling is not detailed in the API.
 ******************************************************************************/
int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *phy, uint64_t *lo_freq_hz);
/* Switch between internal and external LO. */
/***************************************************************************//**
 * @brief This function is used to configure the local oscillator (LO) source
 * for the receiver. It should be called after the initialization of the
 * `ad9361_rf_phy` structure. The `int_ext` parameter determines whether
 * to use the internal or external LO. If the device selected is
 * `AD9363A` and the external LO is requested, an error will be returned.
 * It is important to ensure that the `phy` pointer is valid and properly
 * initialized before calling this function.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure that holds the device
 * context. Must not be null and should be initialized before use.
 * @param int_ext An 8-bit integer that specifies the LO source: `INT_LO` (0)
 * for internal and `EXT_LO` (1) for external. If `EXT_LO` is
 * selected while the device is `AD9363A`, the function will
 * return an error.
 * @return Returns 0 on success, or -1 if an error occurs, such as when
 * attempting to set the external LO on an unsupported device.
 ******************************************************************************/
int32_t ad9361_set_rx_lo_int_ext(struct ad9361_rf_phy *phy, uint8_t int_ext);
/* Get the RSSI for the selected channel. */
/***************************************************************************//**
 * @brief This function retrieves the Received Signal Strength Indicator (RSSI)
 * for a specified receive channel. It should be called after the device
 * has been properly initialized and configured. The `rssi` structure
 * must be allocated by the caller and should not be null. The function
 * sets the `ant` field of the `rssi` structure based on the channel
 * number provided, and it assumes a duration of 1 for the measurement.
 * If the channel number is out of range, the behavior is undefined.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param ch The channel number for which to retrieve the RSSI. Valid values are
 * 0 and 1, corresponding to the available receive channels.
 * @param rssi A pointer to a `rf_rssi` structure where the RSSI value will be
 * stored. The caller must allocate this structure and ensure it is
 * not null.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the RSSI retrieval process.
 ******************************************************************************/
int32_t ad9361_get_rx_rssi(struct ad9361_rf_phy *phy, uint8_t ch,
			   struct rf_rssi *rssi);
/* Set the gain control mode for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to configure the gain control mode for a
 * specific receive channel in the AD9361 RF transceiver. It should be
 * called after initializing the transceiver and before starting any data
 * reception. The function expects a valid channel index and a gain
 * control mode value. If the provided channel index is out of range, the
 * function will not perform any operation, ensuring that the internal
 * state remains consistent.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver. Must not be null.
 * @param ch The index of the receive channel (0 or 1). Must be within the valid
 * range of channels.
 * @param gc_mode The gain control mode to set. Valid values depend on the
 * specific modes supported by the hardware.
 * @return Returns 0 on success, indicating that the gain control mode has been
 * set successfully.
 ******************************************************************************/
int32_t ad9361_set_rx_gain_control_mode(struct ad9361_rf_phy *phy, uint8_t ch,
					uint8_t gc_mode);
/* Get the gain control mode for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to obtain the current gain control mode for a
 * specified receive channel of the AD9361 device. It should be called
 * after the device has been initialized and configured. The function
 * expects a valid pointer to a `struct ad9361_rf_phy` that represents
 * the device, a channel index that specifies which receive channel to
 * query, and a pointer to a variable where the gain control mode will be
 * stored. If the channel index is out of bounds, the behavior is
 * undefined.
 *
 * @param phy A pointer to a `struct ad9361_rf_phy` representing the device.
 * Must not be null.
 * @param ch An 8-bit unsigned integer representing the channel index (0 for
 * RX1, 1 for RX2). Valid values are 0 and 1.
 * @param gc_mode A pointer to an 8-bit unsigned integer where the gain control
 * mode will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the gain control mode has been
 * successfully retrieved and stored in the provided pointer.
 ******************************************************************************/
int32_t ad9361_get_rx_gain_control_mode(struct ad9361_rf_phy *phy, uint8_t ch,
					uint8_t *gc_mode);
/* Set the RX FIR filter configuration. */
/***************************************************************************//**
 * @brief This function configures the receive FIR filter settings for the
 * specified RF PHY device. It should be called after initializing the
 * device and before starting any data reception. The configuration
 * includes parameters such as the decimation factor, gain, and filter
 * coefficients. Ensure that the provided configuration structure is
 * properly populated with valid values, as invalid configurations may
 * lead to undefined behavior.
 *
 * @param phy A pointer to the `struct ad9361_rf_phy` representing the RF PHY
 * device. Must not be null.
 * @param fir_cfg An `AD9361_RXFIRConfig` structure containing the FIR filter
 * configuration. The fields within this structure must be set to
 * valid values, including `rx_dec` (1, 2, or 4), `rx_gain` (-12,
 * -6, 0, or 6), and `rx_coef` (an array of filter coefficients).
 * The `rx_coef_size` must reflect the number of coefficients
 * provided.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the configuration process.
 ******************************************************************************/
int32_t ad9361_set_rx_fir_config(struct ad9361_rf_phy *phy,
				 AD9361_RXFIRConfig fir_cfg);
/* Get the RX FIR filter configuration. */
/***************************************************************************//**
 * @brief This function is used to obtain the configuration settings for the RX
 * FIR filter of a specified channel. It should be called after the
 * AD9361 device has been initialized and configured. The function reads
 * the necessary filter parameters from the device and populates the
 * provided `AD9361_RXFIRConfig` structure with the current filter
 * coefficients, gain, and other relevant settings. It is important to
 * ensure that the `rx_ch` parameter is within the valid range, as
 * invalid values may lead to undefined behavior or errors.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the device
 * instance. Must not be null.
 * @param rx_ch The RX channel number, which should be 0 or 1. Values outside
 * this range may result in an error.
 * @param fir_cfg Pointer to an `AD9361_RXFIRConfig` structure where the filter
 * configuration will be stored. Caller retains ownership and
 * must ensure it is valid.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the operation.
 ******************************************************************************/
int32_t ad9361_get_rx_fir_config(struct ad9361_rf_phy *phy, uint8_t rx_ch,
				 AD9361_RXFIRConfig *fir_cfg);
/* Enable/disable the RX FIR filter. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the receive finite impulse
 * response (FIR) filter in the AD9361 RF transceiver. It should be
 * called after the transceiver has been initialized and configured. The
 * `en_dis` parameter determines whether the FIR filter is enabled (1) or
 * disabled (0). If the requested state is already set, the function will
 * return immediately without making any changes. If an invalid state is
 * requested, the function will revert to the previous state and return
 * an error code.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver. Must not be null.
 * @param en_dis A uint8_t value indicating the desired state of the RX FIR
 * filter: 1 to enable, 0 to disable. Valid values are 0 and 1.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t en_dis);
/* Get the status of the RX FIR filter. */
/***************************************************************************//**
 * @brief This function retrieves the status of the RX FIR filter, indicating
 * whether it is enabled or disabled. It should be called after the
 * initialization of the `ad9361_rf_phy` structure to ensure that the
 * device is properly set up. The function writes the status to the
 * provided pointer, which must not be null. If the pointer is null, the
 * behavior is undefined.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a `uint8_t` where the function will store the FIR
 * filter status (enabled or disabled). Must not be null.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved.
 ******************************************************************************/
int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t *en_dis);
/* Enable/disable the RX RFDC Tracking. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the RX RFDC tracking
 * feature of the AD9361 device. It should be called after the device has
 * been properly initialized. The `en_dis` parameter determines whether
 * the tracking is enabled (1) or disabled (0). If the requested state is
 * already set, the function will return immediately without making any
 * changes. The function may also trigger additional internal state
 * changes related to tracking control.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the device.
 * Must not be null.
 * @param en_dis A uint8_t value that specifies the desired state for RFDC
 * tracking: 1 to enable, 0 to disable. Invalid values will be
 * ignored, and the function will return without making changes.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_rfdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis);
/* Get the status of the RX RFDC Tracking. */
/***************************************************************************//**
 * @brief This function retrieves the current status of the RX RFDC Tracking
 * feature, indicating whether it is enabled or disabled. It should be
 * called after the initialization of the `ad9361_rf_phy` structure to
 * ensure that the device is properly set up. The function modifies the
 * value pointed to by the `en_dis` parameter to reflect the current
 * state of the RFDC Tracking. It is important to ensure that the `phy`
 * parameter is not null before calling this function.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a `uint8_t` variable where the status of the RX
 * RFDC Tracking will be stored. The caller retains ownership of
 * this variable, and it must not be null.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved.
 ******************************************************************************/
int32_t ad9361_get_rx_rfdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis);
/* Enable/disable the RX BasebandDC Tracking. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the RX BasebandDC tracking
 * feature of the AD9361 device. It should be called after the device has
 * been initialized and configured. The function checks if the requested
 * state is different from the current state before making any changes.
 * If the state is already set to the requested value, it returns
 * immediately without making any modifications. This helps to avoid
 * unnecessary operations and potential side effects.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param en_dis A `uint8_t` value that indicates whether to enable (1) or
 * disable (0) the RX BasebandDC tracking. Valid values are 0 and
 * 1.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_bbdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis);
/* Get the status of the RX BasebandDC Tracking. */
/***************************************************************************//**
 * @brief This function is used to obtain the current status of the RX
 * BasebandDC Tracking feature, which can be either enabled or disabled.
 * It should be called after the initialization of the `ad9361_rf_phy`
 * structure to ensure that the device is properly set up. The function
 * writes the status to the provided pointer, allowing the caller to
 * check whether the tracking is currently active or not. It is important
 * to ensure that the pointer passed to this function is not null to
 * avoid undefined behavior.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure that represents the
 * device. This must not be null and should point to a valid
 * initialized instance.
 * @param en_dis A pointer to a `uint8_t` variable where the function will store
 * the status of the RX BasebandDC Tracking. This pointer must not
 * be null.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved. If there is an error, a negative value may be returned,
 * but specific error handling is not detailed in the API.
 ******************************************************************************/
int32_t ad9361_get_rx_bbdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis);
/* Enable/disable the RX Quadrature Tracking. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the RX Quadrature Tracking
 * feature of the AD9361 RF PHY. It should be called after the
 * initialization of the device and before starting any data processing.
 * If the requested state is already set, the function will return
 * immediately without making any changes. The function may have side
 * effects on the tracking control mechanism, which is responsible for
 * maintaining signal integrity.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the RF PHY
 * device. Must not be null.
 * @param en_dis A `uint8_t` value that indicates whether to enable (1) or
 * disable (0) the RX Quadrature Tracking. Valid values are 0 and
 * 1.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_rx_quad_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis);
/* Get the status of the RX Quadrature Tracking. */
/***************************************************************************//**
 * @brief This function retrieves the current status of the RX Quadrature
 * Tracking feature, indicating whether it is enabled or disabled. It
 * should be called after the initialization of the `ad9361_rf_phy`
 * structure to ensure that the device is properly set up. The function
 * writes the status to the provided pointer, which must not be null. If
 * the pointer is null, the behavior is undefined.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. This must not be null and should be initialized before
 * calling this function.
 * @param en_dis A pointer to a `uint8_t` variable where the status of the RX
 * Quadrature Tracking will be stored. This pointer must not be
 * null; otherwise, the function's behavior is undefined.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved. If there is an error, a negative value may be returned,
 * but specific error codes are not defined in the provided API.
 ******************************************************************************/
int32_t ad9361_get_rx_quad_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis);
/* Set the RX RF input port. */
/***************************************************************************//**
 * @brief This function is used to configure the RX RF input port for the
 * specified `phy` device. It should be called after the device has been
 * initialized and before any data transmission or reception occurs. The
 * `mode` parameter determines which RF input port to select, and it is
 * essential to ensure that the selected mode is valid. If an invalid
 * mode is provided, the function may return an error code, indicating
 * that the operation was unsuccessful.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. This pointer must not be null and should point to a valid
 * initialized instance.
 * @param mode An unsigned integer representing the RF input port selection
 * mode. Valid values depend on the specific implementation and
 * should be checked against the device's documentation. The
 * function will return an error if an invalid mode is provided.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_rx_rf_port_input(struct ad9361_rf_phy *phy, uint32_t mode);
/* Get the selected RX RF input port. */
/***************************************************************************//**
 * @brief This function retrieves the currently selected RX RF input port from
 * the specified `ad9361_rf_phy` structure. It should be called after the
 * initialization of the `ad9361_rf_phy` instance to ensure that the
 * structure is properly set up. The function writes the selected mode to
 * the provided pointer, which must not be null. If the pointer is null,
 * the behavior is undefined.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure that represents the
 * device. Must not be null.
 * @param mode A pointer to a `uint32_t` variable where the selected RX RF input
 * port will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the operation was successful.
 * If there is an error, a negative value may be returned, but specific
 * error codes are not defined in the provided API.
 ******************************************************************************/
int32_t ad9361_get_rx_rf_port_input(struct ad9361_rf_phy *phy, uint32_t *mode);
/* Store RX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to store a specific RX fastlock profile for the
 * AD9361 device. It should be called after the device has been properly
 * initialized and configured. The `profile` parameter specifies which
 * profile to store, and it is important to ensure that the profile
 * number is valid. If the provided `phy` pointer is null or if the
 * profile number is out of range, the function will return an error
 * code.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param profile An unsigned integer representing the profile number to store.
 * Valid values depend on the device's configuration and should
 * be within the acceptable range defined by the device
 * specifications.
 * @return Returns a status code indicating success or failure of the operation.
 * A return value of 0 typically indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_rx_fastlock_store(struct ad9361_rf_phy *phy, uint32_t profile);
/* Recall RX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to recall a specific RX fastlock profile
 * identified by the `profile` parameter. It should be called after the
 * AD9361 device has been properly initialized and configured. The
 * function will attempt to restore the settings associated with the
 * specified profile, allowing for quick reconfiguration of the RX
 * settings. If the provided `phy` pointer is null or if the profile
 * number is invalid, the function may return an error code.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. Must not be null.
 * @param profile An unsigned integer representing the profile number to recall.
 * Valid values depend on the number of profiles supported by the
 * device; passing an invalid profile number may result in an
 * error.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A return value of 0 typically indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_rx_fastlock_recall(struct ad9361_rf_phy *phy, uint32_t profile);
/* Load RX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to load a previously stored RX fastlock profile
 * into the AD9361 device. It should be called after the device has been
 * properly initialized and configured. The `profile` parameter specifies
 * which profile to load, and the `values` parameter is a pointer to an
 * array containing the values associated with that profile. It is
 * important to ensure that the `values` array is valid and properly
 * allocated before calling this function. If the function encounters an
 * invalid `phy` pointer or an invalid profile number, it will return an
 * error code.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param profile An unsigned integer representing the profile number to load.
 * Valid profile numbers depend on the specific implementation
 * and configuration of the device.
 * @param values A pointer to an array of bytes that contains the values for the
 * specified profile. This array must be allocated by the caller
 * and should not be null.
 * @return Returns a 32-bit integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_rx_fastlock_load(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values);
/* Save RX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to save the current RX fastlock profile to a
 * specified profile index. It should be called after the RX fastlock
 * settings have been configured and when the user wants to store these
 * settings for future use. The function expects a valid pointer to an
 * `ad9361_rf_phy` structure, a profile index, and a pointer to an array
 * of values that represent the fastlock settings. If the provided
 * profile index is out of range or if the `values` pointer is null, the
 * function will handle these cases appropriately.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the RX PHY
 * device. Must not be null.
 * @param profile An unsigned integer representing the profile index where the
 * fastlock settings will be saved. Valid range is
 * implementation-defined, and the function will handle out-of-
 * range values.
 * @param values A pointer to an array of bytes containing the fastlock settings
 * to be saved. Must not be null.
 * @return Returns an integer status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad9361_rx_fastlock_save(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values);
/* Power down the RX Local Oscillator. */
/***************************************************************************//**
 * @brief This function is used to power down the RX Local Oscillator (LO) of
 * the AD9361 device. It should be called when the RX LO needs to be
 * disabled, typically to save power or during device shutdown
 * procedures. The `option` parameter determines whether to turn the LO
 * off or keep it on. It is important to ensure that this function is
 * called in a valid state of the device, as powering down the LO while
 * the device is in use may lead to undefined behavior.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. This pointer must not be null and should point to a valid
 * initialized device instance.
 * @param option An 8-bit unsigned integer that specifies the power state of the
 * RX LO. A value of 1 indicates that the LO should be powered
 * down (OFF), while a value of 0 indicates that it should remain
 * powered up (ON). Invalid values will be clamped to the nearest
 * valid option.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_rx_lo_powerdown(struct ad9361_rf_phy *phy, uint8_t option);
/* Get the RX Local Oscillator power status. */
/***************************************************************************//**
 * @brief This function retrieves the power status of the RX Local Oscillator
 * (LO) for the specified `phy` device. It should be called after the
 * device has been properly initialized. The `option` parameter will be
 * updated to indicate whether the RX LO is powered down or not, with
 * possible values being `ON` or `OFF`. It is important to ensure that
 * the `phy` pointer is valid and points to an initialized
 * `ad9361_rf_phy` structure before calling this function.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the device.
 * Must not be null and should be initialized before use.
 * @param option A pointer to a `uint8_t` variable where the power status will
 * be stored. The caller retains ownership and must ensure it
 * points to a valid memory location.
 * @return Returns 0 on success, indicating that the power status has been
 * successfully retrieved and stored in the `option` parameter.
 ******************************************************************************/
int32_t ad9361_get_rx_lo_power(struct ad9361_rf_phy *phy, uint8_t *option);
/* Set the transmit attenuation for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit attenuation level for
 * a specific channel in the AD9361 RF transceiver. It should be called
 * after the device has been initialized and is ready for configuration.
 * The attenuation value is specified in millidecibels (mDb) and can be
 * adjusted to control the output power of the transmitter. It is
 * important to ensure that the specified channel is valid; otherwise,
 * the function may return an error. The function may also have side
 * effects on the output signal quality if the attenuation is set outside
 * of the supported range.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver instance. Must not be null.
 * @param ch The channel number for which the attenuation is to be set. Valid
 * values are typically 0 or 1, corresponding to the available
 * transmit channels.
 * @param attenuation_mdb The desired attenuation level in millidecibels (mDb).
 * This value should be within the supported range for
 * the device; otherwise, the function may return an
 * error.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_tx_attenuation(struct ad9361_rf_phy *phy, uint8_t ch,
				  uint32_t attenuation_mdb);
/* Get current transmit attenuation for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to obtain the current transmit attenuation value
 * in millidecibels (mDb) for a specified channel of the AD9361 RF PHY
 * device. It should be called after the device has been properly
 * initialized and configured. The function expects a valid pointer to a
 * `struct ad9361_rf_phy` representing the device, a channel index that
 * must be within the valid range, and a pointer to a variable where the
 * attenuation value will be stored. If the function encounters an error,
 * it will return a negative value, while a successful call will return 0
 * and populate the provided pointer with the attenuation value.
 *
 * @param phy A pointer to a `struct ad9361_rf_phy` that represents the
 * initialized RF PHY device. Must not be null.
 * @param ch An unsigned 8-bit integer representing the channel index. Valid
 * values are typically 0 or 1, depending on the device configuration.
 * @param attenuation_mdb A pointer to a 32-bit unsigned integer where the
 * retrieved attenuation value will be stored. The caller
 * retains ownership of this variable, and it must not be
 * null.
 * @return Returns 0 on success, indicating that the attenuation value has been
 * successfully retrieved and stored in the provided pointer. If an
 * error occurs, a negative value is returned.
 ******************************************************************************/
int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *phy, uint8_t ch,
				  uint32_t *attenuation_mdb);
/* Set the TX RF bandwidth. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit RF bandwidth of the
 * specified `ad9361_rf_phy` instance. It should be called after the
 * initialization of the device and before starting any transmission. The
 * function validates the requested bandwidth and updates the
 * configuration only if the new bandwidth differs from the current
 * setting. If an invalid bandwidth is provided, it will be adjusted to a
 * valid value, ensuring that the function operates safely without
 * causing errors.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the RF
 * device. Must not be null.
 * @param bandwidth_hz The desired transmit RF bandwidth in Hertz. Valid values
 * depend on the specific capabilities of the device. If an
 * invalid value is provided, it will be validated and
 * adjusted accordingly.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t bandwidth_hz);
/* Get the TX RF bandwidth. */
/***************************************************************************//**
 * @brief This function is used to obtain the current transmit RF bandwidth
 * setting of the specified `ad9361_rf_phy` instance. It should be called
 * after the initialization of the `ad9361_rf_phy` structure to ensure
 * that the bandwidth value is valid. The function writes the bandwidth
 * value in Hertz to the provided pointer. If the pointer is null, the
 * behavior is undefined.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure that represents the RF
 * physical layer. This pointer must not be null and should point to
 * a valid initialized instance.
 * @param bandwidth_hz A pointer to a `uint32_t` variable where the current
 * transmit RF bandwidth will be stored. This pointer must
 * not be null; otherwise, the function's behavior is
 * undefined.
 * @return Returns 0 on success, indicating that the bandwidth has been
 * successfully retrieved and stored in the provided pointer. If an
 * error occurs, a negative error code may be returned, but specific
 * error handling is not detailed in the API.
 ******************************************************************************/
int32_t ad9361_get_tx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t *bandwidth_hz);
/* Set the TX sampling frequency. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit sampling frequency of
 * the AD9361 device. It should be called after the device has been
 * initialized and is ready for configuration. The function takes a
 * desired sampling frequency in Hertz and adjusts the internal clock
 * settings accordingly. If the provided frequency is invalid or if the
 * device is not properly initialized, the function may return an error
 * code. It is important to ensure that the sampling frequency is within
 * the supported range for the device.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. This pointer must not be null and should point to a valid
 * initialized device instance.
 * @param sampling_freq_hz The desired transmit sampling frequency in Hertz.
 * This value must be within the valid range supported
 * by the device. If the value is out of range, the
 * function will return an error code.
 * @return Returns a status code indicating success or failure. A negative value
 * indicates an error occurred during the operation.
 ******************************************************************************/
int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t sampling_freq_hz);
/* Get current TX sampling frequency. */
/***************************************************************************//**
 * @brief This function is used to obtain the current transmit sampling
 * frequency of the AD9361 device. It should be called after the device
 * has been properly initialized and configured. The function writes the
 * sampling frequency in Hertz to the provided pointer. If the pointer is
 * null, the behavior is undefined, and the function may not execute
 * correctly.
 *
 * @param phy A pointer to the `struct ad9361_rf_phy` representing the AD9361
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param sampling_freq_hz A pointer to a `uint32_t` where the function will
 * store the current TX sampling frequency in Hertz.
 * This pointer must not be null; otherwise, the
 * function may not behave as expected.
 * @return Returns 0 on success, indicating that the sampling frequency has been
 * successfully retrieved and stored in the provided pointer.
 ******************************************************************************/
int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t *sampling_freq_hz);
/* Set the TX LO frequency. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit local oscillator (LO)
 * frequency for the specified RF PHY device. It should be called after
 * the device has been properly initialized and configured. The frequency
 * is specified in Hertz and must be within the valid range supported by
 * the device. If an invalid frequency is provided, the function will
 * return an error code indicating the failure.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF PHY
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param lo_freq_hz The desired transmit LO frequency in Hertz. This value must
 * be within the valid operational range of the device. If the
 * value is out of range, the function will return an error
 * code.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *phy, uint64_t lo_freq_hz);
/* Get current TX LO frequency. */
/***************************************************************************//**
 * @brief This function is used to obtain the current transmit local oscillator
 * (LO) frequency in hertz. It should be called after the AD9361 device
 * has been properly initialized and configured. The caller must ensure
 * that the `phy` parameter points to a valid `struct ad9361_rf_phy`
 * instance. The `lo_freq_hz` parameter must not be null, as it will be
 * used to store the retrieved frequency value. If the function is called
 * with an invalid `phy` pointer, it may lead to undefined behavior.
 *
 * @param phy A pointer to a `struct ad9361_rf_phy` that represents the AD9361
 * device. This must not be null and should point to a valid
 * initialized instance.
 * @param lo_freq_hz A pointer to a `uint64_t` variable where the function will
 * store the current TX LO frequency in hertz. This must not
 * be null.
 * @return Returns 0 on success, indicating that the frequency was successfully
 * retrieved and stored in `lo_freq_hz`. If an error occurs, a negative
 * value may be returned, but specific error codes are not detailed in
 * the API.
 ******************************************************************************/
int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *phy, uint64_t *lo_freq_hz);
/* Switch between internal and external LO. */
/***************************************************************************//**
 * @brief This function is used to configure the local oscillator (LO) source
 * for the transmitter. It should be called after the initialization of
 * the `ad9361_rf_phy` structure. The `int_ext` parameter determines
 * whether to use the internal or external LO. If the device selected is
 * `AD9363A` and the external LO is requested, an error will be returned.
 * It is important to ensure that the `phy` pointer is valid and properly
 * initialized before calling this function.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure that holds the device
 * context. Must not be null and should be initialized before use.
 * @param int_ext An 8-bit integer that specifies the LO source: `INT_LO` for
 * internal and `EXT_LO` for external. Valid values are 0
 * (internal) and 1 (external). If `EXT_LO` is selected for an
 * `AD9363A` device, the function will return an error.
 * @return Returns 0 on success, or -1 if an error occurs, such as when
 * attempting to use an external LO with an unsupported device.
 ******************************************************************************/
int32_t ad9361_set_tx_lo_int_ext(struct ad9361_rf_phy *phy, uint8_t int_ext);
/* Set the TX FIR filter configuration. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit FIR filter settings
 * for the AD9361 device. It should be called after initializing the
 * device and before starting transmission to ensure the filter settings
 * are applied. The function expects a valid pointer to the
 * `ad9361_rf_phy` structure and a properly populated
 * `AD9361_TXFIRConfig` structure. If the provided configuration is
 * invalid, the function may return an error code.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. Must not be null.
 * @param fir_cfg The `AD9361_TXFIRConfig` structure containing the FIR filter
 * configuration parameters. The structure must be properly
 * initialized before passing it to the function.
 * @return Returns a status code indicating success or failure of the operation.
 * A return value of 0 indicates success, while a negative value
 * indicates an error.
 ******************************************************************************/
int32_t ad9361_set_tx_fir_config(struct ad9361_rf_phy *phy,
				 AD9361_TXFIRConfig fir_cfg);
/* Get the TX FIR filter configuration. */
/***************************************************************************//**
 * @brief This function is used to obtain the configuration settings for the
 * transmit FIR filter of a specified channel. It should be called after
 * the AD9361 device has been initialized and configured. The function
 * reads the current FIR filter settings from the device and populates
 * the provided `fir_cfg` structure with the relevant coefficients and
 * parameters. If the specified channel is invalid or if there are issues
 * reading from the device, the function will return an error code.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. This must not be null and should point to a valid
 * initialized device.
 * @param tx_ch An 8-bit unsigned integer representing the transmit channel
 * number. Valid values are 0 and 1, corresponding to the first and
 * second transmit channels, respectively. The function will adjust
 * this value internally, so it should be within the expected
 * range.
 * @param fir_cfg A pointer to an `AD9361_TXFIRConfig` structure where the
 * function will store the retrieved FIR filter configuration.
 * This pointer must not be null.
 * @return Returns 0 on success, indicating that the configuration was
 * successfully retrieved and stored in `fir_cfg`. If an error occurs
 * during the operation, a negative error code is returned.
 ******************************************************************************/
int32_t ad9361_get_tx_fir_config(struct ad9361_rf_phy *phy, uint8_t tx_ch,
				 AD9361_TXFIRConfig *fir_cfg);
/* Enable/disable the TX FIR filter. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the transmit finite impulse
 * response (FIR) filter in the AD9361 RF transceiver. It should be
 * called after the initialization of the transceiver and before any
 * transmission occurs. The `en_dis` parameter determines whether the FIR
 * filter is enabled (1) or disabled (0). If the requested state is
 * already set, the function will return immediately without making any
 * changes. If an invalid state is provided, the function will revert to
 * the default enabled state.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver. Must not be null.
 * @param en_dis An 8-bit integer indicating the desired state of the TX FIR
 * filter. Valid values are 0 (disable) and 1 (enable). Caller
 * retains ownership.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t en_dis);
/* Get the status of the TX FIR filter. */
/***************************************************************************//**
 * @brief This function retrieves the status of the TX FIR filter, indicating
 * whether it is enabled or disabled. It should be called after the
 * initialization of the `ad9361_rf_phy` structure to ensure that the
 * device is properly set up. The function writes the status to the
 * provided pointer, which must not be null. If the pointer is null, the
 * behavior is undefined.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param en_dis A pointer to a `uint8_t` where the function will store the FIR
 * filter status. Must not be null.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved.
 ******************************************************************************/
int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t *en_dis);
/* Get the TX RSSI for the selected channel. */
/***************************************************************************//**
 * @brief This function retrieves the transmit Received Signal Strength
 * Indicator (RSSI) for a specified channel of the AD9361 RF PHY device.
 * It should be called after the device has been properly initialized and
 * configured. The function expects a valid channel number (0 or 1) and
 * will return an error if an invalid channel is provided. The RSSI value
 * is returned in decibels multiplied by 1000, and the caller must ensure
 * that the pointer for the RSSI output is valid and not null.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF PHY
 * device. Must not be null.
 * @param ch The channel number for which to retrieve the RSSI, which must be
 * either 0 or 1. Any other value will result in an error.
 * @param rssi_db_x_1000 A pointer to a `uint32_t` where the RSSI value will be
 * stored. The caller retains ownership of this pointer,
 * and it must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs,
 * such as an invalid channel number.
 ******************************************************************************/
int32_t ad9361_get_tx_rssi(struct ad9361_rf_phy *phy, uint8_t ch,
			   uint32_t *rssi_db_x_1000);
/* Set the TX RF output port. */
/***************************************************************************//**
 * @brief This function is used to configure the transmit RF output port for the
 * specified `phy` device. It should be called after the device has been
 * properly initialized. The `mode` parameter determines which RF output
 * port to select, and it is essential to ensure that the value provided
 * is valid. If an invalid mode is specified, the function may return an
 * error code, indicating that the operation could not be completed.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param mode An unsigned integer representing the desired TX RF output port
 * mode. Valid values depend on the specific implementation and
 * should be defined in the associated documentation. The function
 * will return an error if an invalid mode is provided.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_set_tx_rf_port_output(struct ad9361_rf_phy *phy, uint32_t mode);
/* Get the selected TX RF output port. */
/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the TX RF
 * output port for the specified `phy` device. It should be called after
 * the device has been properly initialized. The function writes the
 * selected output port mode to the provided pointer. If the pointer is
 * null, the behavior is undefined.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param mode A pointer to a `uint32_t` variable where the selected TX RF
 * output port mode will be stored. Caller retains ownership and
 * must ensure it is not null.
 * @return Returns 0 on success, indicating that the mode has been successfully
 * retrieved.
 ******************************************************************************/
int32_t ad9361_get_tx_rf_port_output(struct ad9361_rf_phy *phy,
				     uint32_t *mode);
/* Enable/disable the auto calibration. */
/***************************************************************************//**
 * @brief This function is used to control the auto calibration feature of the
 * transmitter in the AD9361 RF transceiver. It should be called after
 * the initialization of the `ad9361_rf_phy` structure. The `en_dis`
 * parameter determines whether the auto calibration is enabled (1) or
 * disabled (0). If an invalid value is provided, the function will
 * default to disabling the feature. It is important to ensure that the
 * `phy` pointer is valid and points to an initialized `ad9361_rf_phy`
 * structure before calling this function.
 *
 * @param phy Pointer to an initialized `ad9361_rf_phy` structure. Must not be
 * null.
 * @param en_dis An 8-bit integer that indicates whether to enable (1) or
 * disable (0) the auto calibration feature. Valid values are 0
 * and 1.
 * @return Returns 0 on success, indicating that the operation was completed
 * successfully. No other return values are defined.
 ******************************************************************************/
int32_t ad9361_set_tx_auto_cal_en_dis(struct ad9361_rf_phy *phy,
				      uint8_t en_dis);
/* Get the status of the auto calibration flag. */
/***************************************************************************//**
 * @brief This function retrieves the current status of the auto calibration
 * feature for the specified `ad9361_rf_phy` instance. It should be
 * called after the initialization of the `ad9361_rf_phy` structure to
 * ensure that the device is properly set up. The function writes the
 * status to the provided pointer, which will indicate whether auto
 * calibration is enabled or disabled. It is important to ensure that the
 * pointer passed to this function is not null, as this will lead to
 * undefined behavior.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the device
 * instance. Must not be null.
 * @param en_dis A pointer to a `uint8_t` variable where the status of the auto
 * calibration will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the status was successfully
 * retrieved.
 ******************************************************************************/
int32_t ad9361_get_tx_auto_cal_en_dis(struct ad9361_rf_phy *phy,
				      uint8_t *en_dis);
/* Store TX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to store a transmit fastlock profile for the
 * AD9361 RF transceiver. It should be called after the transceiver has
 * been properly initialized and configured. The profile can be used
 * later to quickly restore the transmit settings, which is useful for
 * applications requiring rapid state changes. Ensure that the `phy`
 * parameter is valid and points to an initialized `ad9361_rf_phy`
 * structure. The function may return an error code if the operation
 * fails.
 *
 * @param phy Pointer to an `ad9361_rf_phy` structure representing the
 * initialized RF transceiver. Must not be null.
 * @param profile An unsigned integer representing the profile index to store.
 * Valid values depend on the implementation but typically range
 * from 0 to a predefined maximum. The function will handle out-
 * of-range values by returning an error.
 * @return Returns a status code indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_tx_fastlock_store(struct ad9361_rf_phy *phy, uint32_t profile);
/* Recall TX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to restore a specific transmit fastlock profile
 * for the AD9361 RF PHY device. It should be called after the device has
 * been properly initialized and configured. The `profile` parameter
 * specifies which fastlock profile to recall, and it is important to
 * ensure that the profile number is valid and corresponds to a profile
 * that has been previously stored. If an invalid profile number is
 * provided, the function may return an error code.
 *
 * @param phy A pointer to the `struct ad9361_rf_phy` representing the RF PHY
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param profile An unsigned integer representing the profile number to recall.
 * Valid values depend on the number of profiles that can be
 * stored, which should be defined in the device's documentation.
 * If an invalid profile number is provided, the function will
 * handle it by returning an error code.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_tx_fastlock_recall(struct ad9361_rf_phy *phy, uint32_t profile);
/* Load TX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to load a transmit fastlock profile into the
 * AD9361 device. It should be called after the device has been properly
 * initialized and configured. The `profile` parameter specifies which
 * profile to load, and the `values` parameter should point to an array
 * containing the fastlock values corresponding to that profile. It is
 * important to ensure that the `values` array is valid and contains the
 * expected number of elements for the specified profile. If the function
 * encounters an invalid `phy` pointer or an invalid profile number, it
 * will return an error code.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the device.
 * Must not be null.
 * @param profile An unsigned integer representing the profile number to load.
 * Valid profile numbers depend on the specific implementation
 * and configuration of the device.
 * @param values Pointer to an array of bytes containing the fastlock values.
 * Must not be null and should be of sufficient size to hold the
 * values for the specified profile.
 * @return Returns a status code indicating success or failure of the operation.
 * A return value of 0 indicates success, while a negative value
 * indicates an error.
 ******************************************************************************/
int32_t ad9361_tx_fastlock_load(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values);
/* Save TX fastlock profile. */
/***************************************************************************//**
 * @brief This function is used to save a TX fastlock profile for the specified
 * `profile` index. It should be called after the AD9361 device has been
 * properly initialized and configured. The `values` parameter should
 * point to a buffer that contains the data to be saved for the fastlock
 * profile. It is important to ensure that the `phy` parameter is not
 * null and that the `profile` index is within a valid range. If the
 * function encounters an invalid `phy` pointer or an out-of-range
 * `profile`, it will return an error code.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. Must not be null.
 * @param profile An unsigned integer representing the profile index to save.
 * Valid range is implementation-defined; ensure it is within the
 * acceptable limits for the device.
 * @param values Pointer to a buffer containing the values to be saved for the
 * fastlock profile. Must not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_tx_fastlock_save(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values);
/* Power down the TX Local Oscillator. */
/***************************************************************************//**
 * @brief This function is used to control the power state of the transmit local
 * oscillator in the AD9361 RF transceiver. It should be called when
 * there is a need to disable the TX local oscillator, typically to save
 * power or during certain operational modes. The `option` parameter
 * determines whether to turn the oscillator off or on, with a value of 1
 * indicating power down and 0 indicating power up. It is important to
 * ensure that this function is called in the appropriate state of the
 * device to avoid unintended behavior.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the RF
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param option An 8-bit unsigned integer that specifies the desired state of
 * the TX local oscillator. A value of 1 will power down the
 * oscillator, while a value of 0 will power it up. This parameter
 * must be either 0 or 1.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_tx_lo_powerdown(struct ad9361_rf_phy *phy, uint8_t option);
/* Get the TX Local Oscillator power status. */
/***************************************************************************//**
 * @brief This function retrieves the power status of the TX Local Oscillator.
 * It should be called after the initialization of the `ad9361_rf_phy`
 * structure to ensure that the device is properly set up. The function
 * will write the power status to the provided `option` pointer,
 * indicating whether the TX Local Oscillator is powered down or not. It
 * is important to ensure that the `option` pointer is not null before
 * calling this function.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param option A pointer to a `uint8_t` where the power status will be stored.
 * Must not be null.
 * @return Returns 0 on success, indicating that the power status was
 * successfully retrieved and stored in the `option` parameter.
 ******************************************************************************/
int32_t ad9361_get_tx_lo_power(struct ad9361_rf_phy *phy, uint8_t *option);
/* Set the RX and TX path rates. */
/***************************************************************************//**
 * @brief This function configures the clock rates for both the receive (RX) and
 * transmit (TX) paths of the AD9361 device. It should be called after
 * the device has been initialized and before any data transmission or
 * reception occurs. The function will update the RF bandwidth settings
 * based on the provided clock rates. If the provided clock rates are
 * invalid or if the device is not properly initialized, the function may
 * return an error code.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param rx_path_clks A pointer to an array of 6 `uint32_t` values representing
 * the RX path clock rates. Caller retains ownership. Must
 * not be null.
 * @param tx_path_clks A pointer to an array of 6 `uint32_t` values representing
 * the TX path clock rates. Caller retains ownership. Must
 * not be null.
 * @return Returns a non-negative integer on success, or a negative error code
 * if the operation fails.
 ******************************************************************************/
int32_t ad9361_set_trx_path_clks(struct ad9361_rf_phy *phy,
				 uint32_t *rx_path_clks, uint32_t *tx_path_clks);
/* Get the RX and TX path rates. */
/***************************************************************************//**
 * @brief This function is used to obtain the clock rates for both the receive
 * (RX) and transmit (TX) paths of the AD9361 device. It should be called
 * after the device has been initialized and configured. The function
 * populates the provided pointers with the respective clock rates, which
 * can be used for further processing or configuration. It is important
 * to ensure that the pointers passed for the RX and TX path clocks are
 * not null, as this will lead to undefined behavior.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param rx_path_clks A pointer to a `uint32_t` array where the RX path clock
 * rates will be stored. Must not be null.
 * @param tx_path_clks A pointer to a `uint32_t` array where the TX path clock
 * rates will be stored. Must not be null.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9361_get_trx_path_clks(struct ad9361_rf_phy *phy,
				 uint32_t *rx_path_clks, uint32_t *tx_path_clks);
/* Set the number of channels mode. */
/***************************************************************************//**
 * @brief This function configures the number of channels mode for the AD9361
 * device. It should be called after the device has been initialized and
 * before any data transmission or reception occurs. The `no_ch_mode`
 * parameter determines the mode of operation: a value of 1 sets the
 * device to a single channel mode, while a value of 2 sets it to a dual
 * channel mode. If an invalid value is provided, the function will
 * return an error code, and no changes will be made to the device
 * configuration.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the
 * device. Must not be null.
 * @param no_ch_mode An 8-bit unsigned integer indicating the number of channels
 * mode. Valid values are 1 for single channel mode and 2 for
 * dual channel mode. Any other value will result in an error.
 * @return Returns 0 on success, or a negative error code if an invalid
 * `no_ch_mode` is provided.
 ******************************************************************************/
int32_t ad9361_set_no_ch_mode(struct ad9361_rf_phy *phy, uint8_t no_ch_mode);
/* Do multi chip synchronization. */
/***************************************************************************//**
 * @brief This function is used to synchronize multiple AD9361 devices, ensuring
 * that the master and slave devices operate in harmony. It must be
 * called after both devices have been initialized and configured. If
 * either device is of type AD9363, the function will return an error, as
 * MCS is not supported for that device. The function modifies the state
 * of the devices during the synchronization process, and it is important
 * to ensure that the devices are in a suitable state for synchronization
 * before calling this function.
 *
 * @param phy_master Pointer to the master `ad9361_rf_phy` structure. Must not
 * be null and should point to a valid initialized device.
 * @param phy_slave Pointer to the slave `ad9361_rf_phy` structure. Must not be
 * null and should point to a valid initialized device.
 * @return Returns 0 on success, or -1 if synchronization is not supported for
 * the specified devices.
 ******************************************************************************/
int32_t ad9361_do_mcs(struct ad9361_rf_phy *phy_master,
		      struct ad9361_rf_phy *phy_slave);
/* Enable/disable the TRX FIR filters. */
/***************************************************************************//**
 * @brief This function is used to enable or disable the Transmit and Receive
 * FIR filters in the AD9361 RF transceiver. It should be called when the
 * user wants to change the filter state, typically after initializing
 * the device and configuring the necessary parameters. The `en_dis`
 * parameter determines whether the filters are enabled (1) or disabled
 * (0). If the filters are already in the desired state, the function
 * will return without making any changes. If an invalid state is
 * detected during the operation, the function will revert the filters to
 * a bypass state.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF
 * transceiver. Must not be null.
 * @param en_dis An 8-bit integer that indicates whether to enable (1) or
 * disable (0) the FIR filters. Valid values are 0 and 1.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_set_trx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t en_dis);
/* Set the OSR rate governor. */
/***************************************************************************//**
 * @brief This function is used to configure the OSR (Oversampling Rate) rate
 * governor for the specified `phy` instance. It should be called when
 * you need to enable or disable the rate governor based on the desired
 * operating conditions. The `rate_gov` parameter determines whether the
 * rate governor is enabled (non-zero value) or disabled (zero value). It
 * is important to ensure that the `phy` pointer is valid and properly
 * initialized before calling this function.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure representing the
 * physical layer device. Must not be null and should be properly
 * initialized before use.
 * @param rate_gov An unsigned integer that specifies the state of the rate
 * governor. A value of 0 disables the rate governor, while any
 * non-zero value enables it. Valid values are 0 or any positive
 * integer.
 * @return Returns 0 on success, indicating that the operation was completed
 * successfully. No other values are returned.
 ******************************************************************************/
int32_t ad9361_set_trx_rate_gov(struct ad9361_rf_phy *phy, uint32_t rate_gov);
/* Get the OSR rate governor. */
/***************************************************************************//**
 * @brief This function is used to obtain the current value of the OSR rate
 * governor from the specified `ad9361_rf_phy` structure. It should be
 * called after the initialization of the `ad9361_rf_phy` instance to
 * ensure that the structure is properly set up. The function writes the
 * retrieved value into the provided pointer, which must not be null. If
 * the pointer is null, the behavior is undefined.
 *
 * @param phy A pointer to an `ad9361_rf_phy` structure that represents the
 * device instance. Must not be null.
 * @param rate_gov A pointer to a `uint32_t` variable where the OSR rate
 * governor value will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the value was successfully
 * retrieved and stored in the provided pointer.
 ******************************************************************************/
int32_t ad9361_get_trx_rate_gov(struct ad9361_rf_phy *phy, uint32_t *rate_gov);
/* Perform the selected calibration. */
/***************************************************************************//**
 * @brief This function is used to execute a specific calibration routine on the
 * AD9361 RF PHY device. It should be called after the device has been
 * properly initialized and configured. The calibration type is specified
 * by the `cal` parameter, and the `arg` parameter can be used to provide
 * additional context or settings for the calibration process. It is
 * important to ensure that the `phy` pointer is valid and points to an
 * initialized `ad9361_rf_phy` structure. The function may return an
 * error code if the calibration cannot be performed, such as if the
 * device is not in a suitable state.
 *
 * @param phy Pointer to an initialized `ad9361_rf_phy` structure. Must not be
 * null.
 * @param cal Specifies the type of calibration to perform. Valid values depend
 * on the specific calibration routines supported by the device.
 * @param arg An integer that provides additional parameters for the
 * calibration. The valid range and meaning depend on the specific
 * calibration type.
 * @return Returns a status code indicating the success or failure of the
 * calibration operation. A return value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_do_calib(struct ad9361_rf_phy *phy, uint32_t cal, int32_t arg);
/* Load and enable TRX FIR filters configurations. */
/***************************************************************************//**
 * @brief This function is used to load and enable the transmit and receive FIR
 * filter configurations for the AD9361 device. It should be called after
 * initializing the device and before starting any data transmission or
 * reception. The function expects valid configurations for both the RX
 * and TX FIR filters, and it will set the filter bandwidths accordingly.
 * If either configuration is invalid, the function will handle it
 * gracefully, ensuring that the filter settings are only applied if they
 * are valid. The function also updates the internal state to reflect the
 * validity of the filter settings.
 *
 * @param phy Pointer to the `ad9361_rf_phy` structure representing the device.
 * Must not be null.
 * @param rx_fir_cfg Configuration structure for the RX FIR filter. Must contain
 * valid settings for the RX path, including coefficients and
 * bandwidth.
 * @param tx_fir_cfg Configuration structure for the TX FIR filter. Must contain
 * valid settings for the TX path, including coefficients and
 * bandwidth.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad9361_trx_load_enable_fir(struct ad9361_rf_phy *phy,
				   AD9361_RXFIRConfig rx_fir_cfg,
				   AD9361_TXFIRConfig tx_fir_cfg);
/* Do DCXO coarse tuning. */
/***************************************************************************//**
 * @brief This function is used to adjust the coarse tuning of the DCXO (Digital
 * Controlled Crystal Oscillator) in the AD9361 RF PHY device. It should
 * be called after the device has been properly initialized and
 * configured. The `coarse` parameter specifies the coarse tuning value,
 * which must be within the valid range defined by the hardware
 * specifications. If the provided value is invalid, the function may not
 * perform the tuning as expected, and the behavior is undefined. It is
 * important to ensure that the device is in a suitable state for tuning
 * before invoking this function.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the RF PHY
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param coarse An unsigned integer representing the coarse tuning value for
 * the DCXO. The valid range for this value is determined by the
 * hardware specifications. Providing a value outside this range
 * may lead to undefined behavior.
 * @return Returns an integer value indicating the success or failure of the
 * tuning operation. A return value of 0 typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t ad9361_do_dcxo_tune_coarse(struct ad9361_rf_phy *phy,
				   uint32_t coarse);
/* Do DCXO fine tuning. */
/***************************************************************************//**
 * @brief This function is used to adjust the fine tuning of the DCXO (Digital
 * Controlled Crystal Oscillator) in the AD9361 RF PHY device. It should
 * be called after the device has been properly initialized and
 * configured. The `fine` parameter specifies the fine tuning value,
 * which is applied to the DCXO. It is important to ensure that the `phy`
 * parameter is valid and points to an initialized `ad9361_rf_phy`
 * structure. The function will return an integer status code indicating
 * success or failure of the tuning operation.
 *
 * @param phy Pointer to an initialized `ad9361_rf_phy` structure. Must not be
 * null.
 * @param fine The fine tuning value for the DCXO. Valid values depend on the
 * specific tuning range supported by the hardware.
 * @return Returns a status code indicating the result of the tuning operation,
 * where a value of 0 typically indicates success.
 ******************************************************************************/
int32_t ad9361_do_dcxo_tune_fine(struct ad9361_rf_phy *phy,
				 uint32_t fine);
/* Get the temperature. */
/***************************************************************************//**
 * @brief This function is used to obtain the current temperature reading from
 * the AD9361 device. It should be called after the device has been
 * properly initialized. The temperature value is written to the location
 * pointed to by the `temp` parameter. It is important to ensure that the
 * `temp` pointer is not null before calling this function, as passing a
 * null pointer will lead to undefined behavior.
 *
 * @param phy A pointer to the `ad9361_rf_phy` structure representing the AD9361
 * device. This pointer must not be null and should point to a valid
 * initialized device.
 * @param temp A pointer to an `int32_t` variable where the temperature value
 * will be stored. This pointer must not be null; otherwise, the
 * function will not operate correctly.
 * @return Returns 0 on success, indicating that the temperature has been
 * successfully retrieved and stored in the provided `temp` variable.
 ******************************************************************************/
int32_t ad9361_get_temperature(struct ad9361_rf_phy *phy,
			       int32_t *temp);
#endif

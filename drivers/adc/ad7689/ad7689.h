/***************************************************************************//**
 *   @file   ad7689.h
 *   @brief  Header file for the ad7689 driver
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#ifndef AD7689_H_
#define AD7689_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"

#define AD7689_CFG_CFG_MSK      NO_OS_BIT(13)
#define AD7689_CFG_INCC_MSK     NO_OS_GENMASK(12,10)
#define AD7689_CFG_INX_MSK      NO_OS_GENMASK(9,7)
#define AD7689_CFG_BW_MSK       NO_OS_BIT(6)
#define AD7689_CFG_REF_MSK      NO_OS_GENMASK(5,3)
#define AD7689_CFG_SEQ_MSK      NO_OS_GENMASK(2,1)
#define AD7689_CFG_RB_MSK       NO_OS_BIT(0)

/***************************************************************************//**
 * @brief The `ad7689_device_id` enumeration defines a set of constants
 * representing different models of PulSAR ADCs, each with specific bit
 * resolution, channel count, and sampling rate characteristics. This
 * enumeration is used to identify and differentiate between various ADC
 * devices within the AD7689 series, facilitating the selection and
 * configuration of the appropriate ADC model in software applications.
 *
 * @param ID_AD7689 Represents a 16-Bit, 8-Channel, 250 kSPS PulSAR ADC.
 * @param ID_AD7682 Represents a 16-Bit, 4-Channel, 250 kSPS PulSAR ADC.
 * @param ID_AD7949 Represents a 14-Bit, 8-Channel, 250 kSPS PulSAR ADC.
 * @param ID_AD7699 Represents a 16-Bit, 8-Channel, 500 kSPS PulSAR ADC.
 ******************************************************************************/
enum ad7689_device_id {
	/** 16-Bit, 8-Channel, 250 kSPS PulSAR ADC */
	ID_AD7689,
	/** 16-Bit, 4-Channel, 250 kSPS PulSAR ADC */
	ID_AD7682,
	/** 14-Bit, 8-Channel, 250 kSPS PulSAR ADC */
	ID_AD7949,
	/** 16-Bit, 8-Channel, 500 kSPS PulSAR ADC */
	ID_AD7699,
};

/***************************************************************************//**
 * @brief The `ad7689_incc` enumeration defines the input channel configuration
 * options for the AD7689 ADC device. It includes various modes such as
 * bipolar differential pairs, bipolar referenced to a common voltage, a
 * temperature sensor mode, and unipolar configurations with different
 * reference points. These configurations determine how the input
 * channels are referenced and processed by the ADC, allowing for
 * flexible adaptation to different signal types and measurement
 * requirements.
 *
 * @param AD7689_BIPOLAR_DIFFERENTIAL_PAIRS Bipolar differential pairs; INx−
 * referenced to VREF/2 ± 0.1 V.
 * @param AD7689_BIPOLAR_COM Bipolar; INx referenced to COM = V REF /2 ± 0.1 V.
 * @param AD7689_TEMPERATURE_SENSOR Temperature sensor.
 * @param AD7689_UNIPOLAR_DIFFERENTIAL_PAIRS Unipolar differential pairs; INx−
 * referenced to GND ± 0.1 V.
 * @param AD7689_UNIPOLAR_COM Unipolar, INx referenced to COM = GND ± 0.1 V.
 * @param AD7689_UNIPOLAR_GND Unipolar, INx referenced to GND.
 ******************************************************************************/
enum ad7689_incc {
	/** Bipolar differential pairs; INx− referenced to VREF/2 ± 0.1 V. */
	AD7689_BIPOLAR_DIFFERENTIAL_PAIRS = 0x1,
	/** Bipolar; INx referenced to COM = V REF /2 ± 0.1 V. */
	AD7689_BIPOLAR_COM,
	/** Temperature sensor. */
	AD7689_TEMPERATURE_SENSOR,
	/** Unipolar differential pairs; INx− referenced to GND ± 0.1 V. */
	AD7689_UNIPOLAR_DIFFERENTIAL_PAIRS = 0x5,
	/** Unipolar, INx referenced to COM = GND ± 0.1 V. */
	AD7689_UNIPOLAR_COM,
	/** Unipolar, INx referenced to GND. */
	AD7689_UNIPOLAR_GND
};

/***************************************************************************//**
 * @brief The `ad7689_bw` enumeration defines the bandwidth selection options
 * for the AD7689 device, allowing users to choose between a reduced
 * bandwidth with additional noise filtering or full bandwidth operation.
 * This selection impacts the noise performance and throughput of the
 * device, providing flexibility in optimizing for different application
 * requirements.
 *
 * @param AD7689_BW_QUARTER Represents 1/4 of the bandwidth, using an additional
 * series resistor to further limit noise, requiring
 * maximum throughput to be reduced to 1/4.
 * @param AD7689_BW_FULL Represents full bandwidth without additional noise
 * limiting.
 ******************************************************************************/
enum ad7689_bw {
	/** 1⁄4 of BW, uses an additional series resistor to further bandwidth limit the noise. Maximum throughput must be reduced to 1⁄4. */
	AD7689_BW_QUARTER,
	/** Full bandwidth. */
	AD7689_BW_FULL
};

/***************************************************************************//**
 * @brief The `ad7689_ref` enumeration defines various reference and buffer
 * configurations for the AD7689 ADC device. It allows selection between
 * internal and external reference voltages, with options to enable or
 * disable the internal buffer and temperature sensor. This configuration
 * is crucial for setting up the ADC's reference voltage and operational
 * mode, impacting the precision and functionality of the device.
 *
 * @param AD7689_REF_INTERNAL_2p5V Internal reference and temperature sensor
 * enabled with a 2.5 V buffered output.
 * @param AD7689_REF_INTERNAL_4p096V Internal reference and temperature sensor
 * enabled with a 4.096 V buffered output.
 * @param AD7689_REF_EXTERNAL_TEMP Uses an external reference with the
 * temperature sensor enabled and internal
 * buffer disabled.
 * @param AD7689_REF_EXTERNAL_TEMP_IBUF Uses an external reference with both the
 * internal buffer and temperature sensor
 * enabled.
 * @param AD7689_REF_EXTERNAL Uses an external reference with internal
 * reference, buffer, and temperature sensor
 * disabled.
 * @param AD7689_REF_IBUF Uses an external reference with the internal buffer
 * enabled, but internal reference and temperature sensor
 * disabled.
 ******************************************************************************/
enum ad7689_ref {
	/** Internal reference and temperature sensor enabled. REF = 2.5 V buffered output. */
	AD7689_REF_INTERNAL_2p5V,
	/** Internal reference and temperature sensor enabled. REF = 4.096 V buffered output. */
	AD7689_REF_INTERNAL_4p096V,
	/** Use external reference. Temperature sensor enabled. Internal buffer disabled. */
	AD7689_REF_EXTERNAL_TEMP,
	/** Use external reference. Internal buffer and temperature sensor enabled. */
	AD7689_REF_EXTERNAL_TEMP_IBUF,
	/** Use external reference. Internal reference, internal buffer, and temperature sensor disabled. */
	AD7689_REF_EXTERNAL = 0x6,
	/** Use external reference. Internal buffer enabled. Internal reference and temperature sensor disabled. */
	AD7689_REF_IBUF
};

/***************************************************************************//**
 * @brief The `ad7689_seq` enumeration defines the configuration options for the
 * channel sequencer in the AD7689 ADC driver. It allows the user to
 * specify whether the sequencer is disabled, if the configuration should
 * be updated during a sequence, or if the sequencer should scan through
 * all input channels and optionally measure temperature. This provides
 * flexibility in how the ADC channels are read and configured during
 * operation.
 *
 * @param AD7689_SEQ_DISABLE Disables the sequencer.
 * @param AD7689_SEQ_UPDATE_CFG Updates configuration during the sequence.
 * @param AD7689_SEQ_SCAN_ALL_THEN_TEMP Scans from IN0 to INX, then measures
 * temperature.
 * @param AD7689_SEQ_SCAN_ALL Scans from IN0 to INX.
 ******************************************************************************/
enum ad7689_seq {
	/** Disable sequencer. */
	AD7689_SEQ_DISABLE,
	/** Update configuration during sequence. */
	AD7689_SEQ_UPDATE_CFG,
	/** Scan IN0 to INX, then temperature. */
	AD7689_SEQ_SCAN_ALL_THEN_TEMP,
	/** Scan IN0 to INX. */
	AD7689_SEQ_SCAN_ALL
};

/***************************************************************************//**
 * @brief The `ad7689_config` structure is used to configure various settings of
 * the AD7689 ADC, including input channel configuration, channel
 * selection for sequencing, low-pass filter bandwidth, reference and
 * buffer settings, and channel sequencer configuration. It also includes
 * a boolean flag to determine if the configuration register should be
 * read back. This structure is essential for setting up the ADC's
 * operational parameters before data acquisition.
 *
 * @param incc Specifies the input channel configuration using the ad7689_incc
 * enumeration.
 * @param inx Defines the INX channel selection for the sequencer, iterating
 * from IN0 to INX.
 * @param bw Selects the low-pass filter bandwidth using the ad7689_bw
 * enumeration.
 * @param ref Determines the reference/buffer selection using the ad7689_ref
 * enumeration.
 * @param seq Configures the channel sequencer using the ad7689_seq enumeration.
 * @param rb Indicates whether to read back the CFG register, represented as a
 * boolean.
 ******************************************************************************/
struct ad7689_config {
	/** Input channel configuration */
	enum ad7689_incc incc;
	/** INX channel selection (sequencer iterates from IN0 to INX) */
	uint8_t inx;
	/** Low-pass filter bandwidth selection */
	enum ad7689_bw bw;
	/**  Reference/buffer selection */
	enum ad7689_ref ref;
	/** Channel sequencer configuration */
	enum ad7689_seq seq;
	/** Read back the CFG register */
	bool rb;
};

/***************************************************************************//**
 * @brief The `ad7689_init_param` structure is used to initialize an AD7689
 * device, encapsulating the necessary parameters for device
 * identification, ADC configuration, and SPI communication setup. It
 * includes a device ID to specify which AD7689 variant is being used, a
 * configuration structure for setting ADC-specific parameters such as
 * input channel configuration and reference selection, and SPI
 * initialization parameters to establish communication with the device.
 *
 * @param id Specifies the device ID using the ad7689_device_id enumeration.
 * @param config Holds ADC specific parameters using the ad7689_config
 * structure.
 * @param spi_init Contains SPI initialization parameters using the
 * no_os_spi_init_param structure.
 ******************************************************************************/
struct ad7689_init_param {
	/** Device ID */
	enum ad7689_device_id id;
	/** ADC specific parameters */
	struct ad7689_config config;
	/** SPI initialization parameters */
	struct no_os_spi_init_param spi_init;
};

/***************************************************************************//**
 * @brief The `ad7689_dev` structure is designed to encapsulate the necessary
 * information and configurations for managing an AD7689 device. It
 * includes a device name, a device ID to identify the specific model of
 * the AD7689 series, and an array of configuration structures to handle
 * current and future configurations. Additionally, it holds a pointer to
 * an SPI descriptor, which is essential for communication with the
 * device over the SPI protocol. This structure is central to
 * initializing, configuring, and operating the AD7689 ADC device in a
 * software application.
 *
 * @param name A constant character pointer to the device name string.
 * @param id An enumeration representing the device ID.
 * @param configs An array of two ad7689_config structures, where configs[1] is
 * currently in use and configs[0] will be used in the next
 * transaction.
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor.
 ******************************************************************************/
struct ad7689_dev {
	/** Device name string */
	const char *name;
	/** Device ID */
	enum ad7689_device_id id;
	/** AD7689 configs (configs[1] - in use, configs[0] - will be in use during next transaction) */
	struct ad7689_config configs[2];
	/** SPI descriptor*/
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief This function sets up an AD7689 device instance based on the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration and
 * communication setup. The function allocates memory for the device
 * structure, initializes the SPI interface, and configures the device
 * according to the specified settings. It handles invalid device IDs by
 * returning an error and ensures that resources are properly cleaned up
 * in case of initialization failure. The caller is responsible for
 * providing valid initialization parameters and managing the lifecycle
 * of the device structure.
 *
 * @param dev A pointer to a pointer of type struct ad7689_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a struct ad7689_init_param containing the
 * initialization parameters for the device. Must not be null
 * and must contain a valid device ID and SPI initialization
 * parameters.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., invalid
 * parameters or memory allocation failure).
 ******************************************************************************/
int32_t ad7689_init(struct ad7689_dev **dev,
		    struct ad7689_init_param *init_param);
/***************************************************************************//**
 * @brief This function updates the configuration of an AD7689 device with the
 * settings provided in the `config` parameter. It should be called
 * whenever a change in the device's operational parameters is required,
 * such as modifying the input channel configuration, bandwidth, or
 * reference settings. The function requires a valid device structure and
 * a configuration structure, both of which must be properly initialized
 * before calling. It is important to ensure that the device is not in
 * use by other operations when updating the configuration to avoid
 * conflicts.
 *
 * @param dev A pointer to an `ad7689_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param config A pointer to an `ad7689_config` structure containing the new
 * configuration settings. This structure must be properly
 * initialized with valid configuration values. The caller retains
 * ownership and must ensure it is not null.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad7689_write_config(struct ad7689_dev *dev,
			    struct ad7689_config *config);
/***************************************************************************//**
 * @brief This function is used to obtain the current configuration settings of
 * an AD7689 device. It should be called when you need to verify or
 * utilize the current configuration parameters of the device. The
 * function requires a valid device structure and a configuration
 * structure where the current settings will be stored. It is important
 * to ensure that the device has been properly initialized before calling
 * this function. The function will return an error code if the operation
 * fails, which should be checked to ensure successful execution.
 *
 * @param dev A pointer to an initialized ad7689_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param config A pointer to an ad7689_config structure where the current
 * configuration will be stored. Must not be null. The caller
 * retains ownership and is responsible for managing the memory.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad7689_read_config(struct ad7689_dev *dev,
			   struct ad7689_config *config);
/***************************************************************************//**
 * @brief This function retrieves a specified number of samples from the AD7689
 * device and stores them in the provided data buffer. It should be
 * called after the device has been properly initialized and configured.
 * The function will attempt to read the number of samples specified by
 * nb_samples, storing each sample in the data array. If the data pointer
 * is null, the function will return an error. If nb_samples is zero, the
 * function will return immediately without reading any samples. This
 * function is useful for acquiring data from the ADC for further
 * processing or analysis.
 *
 * @param dev A pointer to an initialized ad7689_dev structure representing the
 * device from which samples are to be read. Must not be null.
 * @param data A pointer to a buffer where the read samples will be stored. Must
 * not be null. The buffer should be large enough to hold nb_samples
 * of uint16_t data.
 * @param nb_samples The number of samples to read from the device. If zero, the
 * function returns immediately without reading any samples.
 * @return Returns 0 on success, a negative error code on failure, or -EINVAL if
 * the data pointer is null.
 ******************************************************************************/
int32_t ad7689_read(struct ad7689_dev *dev, uint16_t *data,
		    uint32_t nb_samples);
/***************************************************************************//**
 * @brief Use this function to properly remove an AD7689 device instance and
 * free associated resources. It should be called when the device is no
 * longer needed to ensure that all allocated resources are released.
 * This function must be called with a valid device instance that was
 * previously initialized. If the provided device pointer is null, the
 * function will return an error code indicating invalid input.
 *
 * @param dev A pointer to the ad7689_dev structure representing the device
 * instance to be removed. Must not be null. If null, the function
 * returns an error code.
 * @return Returns 0 on successful removal and deallocation of the device.
 * Returns a negative error code if the input is invalid.
 ******************************************************************************/
int32_t ad7689_remove(struct ad7689_dev *dev);

#endif

/***************************************************************************//**
 *   @file   axi_dac_core.h
 *   @brief  Driver for the Analog Devices AXI-DAC-CORE module.
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
#ifndef AXI_DAC_CORE_H_
#define AXI_DAC_CORE_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `axi_iface` enumeration defines the possible bus types that can be
 * used with the AXI DAC interface. It currently supports two values:
 * `AXI_DAC_BUS_TYPE_NONE`, indicating no specific bus type, and
 * `AXI_DAC_BUS_TYPE_QSPI`, indicating the use of a QSPI bus. This
 * enumeration is used to configure the bus type for the AXI DAC, which
 * is a critical component in determining how data is communicated with
 * the DAC hardware.
 *
 * @param AXI_DAC_BUS_TYPE_NONE Represents a state where no bus type is
 * specified for the AXI DAC.
 * @param AXI_DAC_BUS_TYPE_QSPI Represents a state where the QSPI bus type is
 * used for the AXI DAC.
 ******************************************************************************/
enum axi_iface {
	AXI_DAC_BUS_TYPE_NONE,
	AXI_DAC_BUS_TYPE_QSPI,
};

/***************************************************************************//**
 * @brief The `axi_io_mode` enumeration defines the different I/O modes
 * available for the AXI DAC interface, specifically focusing on the
 * types of Serial Peripheral Interface (SPI) configurations that can be
 * used. This includes standard SPI, Dual SPI, and Quad SPI modes, which
 * determine how data is communicated between the DAC and other
 * components in the system. These modes are crucial for configuring the
 * data transfer method and optimizing the performance of the DAC in
 * various applications.
 *
 * @param AXI_DAC_IO_MODE_SPI Represents the SPI mode for AXI DAC I/O
 * operations.
 * @param AXI_DAC_IO_MODE_DSPI Represents the Dual SPI mode for AXI DAC I/O
 * operations.
 * @param AXI_DAC_IO_MODE_QSPI Represents the Quad SPI mode for AXI DAC I/O
 * operations.
 ******************************************************************************/
enum axi_io_mode {
	AXI_DAC_IO_MODE_SPI,
	AXI_DAC_IO_MODE_DSPI,
	AXI_DAC_IO_MODE_QSPI,
};

/***************************************************************************//**
 * @brief The `axi_dac` structure is a descriptor for an AXI DAC device,
 * encapsulating essential configuration parameters such as the device
 * name, base address, number of channels, clock frequency, and bus type.
 * It also includes a pointer to an array of `axi_dac_channel`
 * structures, allowing for detailed manual configuration of each DAC
 * channel. This structure is fundamental for initializing and managing
 * the operation of the AXI DAC within a system, providing a
 * comprehensive interface for setting up and controlling the DAC's
 * functionality.
 *
 * @param name A pointer to a constant character string representing the device
 * name.
 * @param base A 32-bit unsigned integer representing the base address of the
 * device.
 * @param num_channels An 8-bit unsigned integer indicating the number of
 * channels in the DAC.
 * @param clock_hz A 64-bit unsigned integer representing the clock frequency of
 * the DAC in hertz.
 * @param channels A pointer to an array of `axi_dac_channel` structures for
 * manual configuration of DAC channels.
 * @param bus_type A 32-bit unsigned integer representing the type of bus used
 * by the DAC IP.
 ******************************************************************************/
struct axi_dac {
	/** Device Name */
	const char *name;
	/** Base Address */
	uint32_t base;
	/** Number of channels */
	uint8_t	num_channels;
	/** AXI DAC Clock */
	uint64_t clock_hz;
	/** DAC channels manual configuration */
	struct axi_dac_channel *channels;
	/** DAC IP bus type */
	uint32_t bus_type;
};

/***************************************************************************//**
 * @brief The `axi_dac_init` structure is used to initialize an AXI DAC device,
 * providing essential configuration parameters such as the device name,
 * base address, number of channels, and the bus type. It also includes a
 * pointer to a configuration array for the DAC channels and specifies
 * the effective DAC rate. This structure is crucial for setting up the
 * DAC hardware interface and ensuring proper communication and data
 * transfer between the DAC and the controlling system.
 *
 * @param name A pointer to a constant character string representing the device
 * name.
 * @param base A 32-bit unsigned integer representing the base address of the
 * device.
 * @param num_channels An 8-bit unsigned integer indicating the number of
 * channels in the DAC.
 * @param channels A pointer to an array of `axi_dac_channel` structures for
 * manual configuration of DAC channels.
 * @param rate An 8-bit unsigned integer representing the effective DAC rate.
 * @param bus_type A 32-bit unsigned integer indicating the type of bus used by
 * the DAC IP.
 ******************************************************************************/
struct axi_dac_init {
	/** Device Name */
	const char *name;
	/** Base Address */
	uint32_t base;
	/** Number of channels */
	uint8_t	num_channels;
	/** DAC channels manual configuration */
	struct axi_dac_channel *channels;
	/** The effective DAC rate */
	uint8_t rate;
	/** DAC IP bus type */
	uint32_t bus_type;
};

/***************************************************************************//**
 * @brief The `axi_dac_data_sel` enumeration defines various data source options
 * for the AXI DAC module, allowing the selection of different input data
 * types such as DDS, SED, DMA, zero, and various pseudo-random noise
 * sequences. This enumeration is used to configure the data source for
 * the DAC channels, enabling flexibility in signal generation and
 * testing scenarios.
 *
 * @param AXI_DAC_DATA_SEL_DDS Selects the Direct Digital Synthesis (DDS) data
 * source.
 * @param AXI_DAC_DATA_SEL_SED Selects the SED (Signal Error Detection) data
 * source.
 * @param AXI_DAC_DATA_SEL_DMA Selects the Direct Memory Access (DMA) data
 * source.
 * @param AXI_DAC_DATA_SEL_ZERO Selects a zero data source, effectively
 * outputting zero.
 * @param AXI_DAC_DATA_SEL_PN7 Selects a pseudo-random noise source with a 7-bit
 * sequence.
 * @param AXI_DAC_DATA_SEL_PN15 Selects a pseudo-random noise source with a
 * 15-bit sequence.
 * @param AXI_DAC_DATA_SEL_PN23 Selects a pseudo-random noise source with a
 * 23-bit sequence.
 * @param AXI_DAC_DATA_SEL_PN31 Selects a pseudo-random noise source with a
 * 31-bit sequence.
 * @param AXI_DAC_DATA_SEL_LB Selects a loopback data source.
 * @param AXI_DAC_DATA_SEL_PNXX Selects a pseudo-random noise source with a
 * custom sequence.
 ******************************************************************************/
enum axi_dac_data_sel {
	AXI_DAC_DATA_SEL_DDS,
	AXI_DAC_DATA_SEL_SED,
	AXI_DAC_DATA_SEL_DMA,
	AXI_DAC_DATA_SEL_ZERO,
	AXI_DAC_DATA_SEL_PN7,
	AXI_DAC_DATA_SEL_PN15,
	AXI_DAC_DATA_SEL_PN23,
	AXI_DAC_DATA_SEL_PN31,
	AXI_DAC_DATA_SEL_LB,
	AXI_DAC_DATA_SEL_PNXX,
};

/***************************************************************************//**
 * @brief The `axi_dac_channel` structure is used to define the configuration
 * parameters for a single channel of an AXI DAC. It includes settings
 * for two DDS tones, such as frequency, phase, and scale, allowing for
 * dual-tone generation. The structure also includes a field for pattern
 * data used in debugging and a selection field to determine the data
 * source for the channel. This structure is essential for configuring
 * the output characteristics of a DAC channel in applications involving
 * signal generation and processing.
 *
 * @param dds_frequency_0 Specifies the frequency for the first DDS tone in
 * hertz.
 * @param dds_phase_0 Specifies the phase for the first DDS tone in milli-
 * degrees.
 * @param dds_scale_0 Specifies the scale for the first DDS tone in micro units.
 * @param dds_frequency_1 Specifies the frequency for the second DDS tone in
 * hertz.
 * @param dds_phase_1 Specifies the phase for the second DDS tone in milli-
 * degrees.
 * @param dds_scale_1 Specifies the scale for the second DDS tone in micro
 * units.
 * @param dds_dual_tone Indicates whether dual tone is used for this channel,
 * set to 0x0 for single tone.
 * @param pat_data Used for SED/debug purposes.
 * @param sel Specifies the data selection mode using the axi_dac_data_sel
 * enumeration.
 ******************************************************************************/
struct axi_dac_channel {
	uint32_t dds_frequency_0;       // in hz (1000*1000 for MHz)
	uint32_t dds_phase_0;           // in milli(?) angles (90*1000 for 90 degrees = pi/2)
	int32_t dds_scale_0;            // in micro units (1.0*1000*1000 is 1.0)
	uint32_t dds_frequency_1;       // in hz (1000*1000 for MHz)
	uint32_t dds_phase_1;           // in milli(?) angles (90*1000 for 90 degrees = pi/2)
	int32_t dds_scale_1;            // in micro units (1.0*1000*1000 is 1.0)
	uint32_t dds_dual_tone;         // if using single tone for this channel, set to 0x0
	uint32_t pat_data;              // if using SED/debug that sort of thing
	enum axi_dac_data_sel sel;      // set to one of the enumerated type above.
};

/***************************************************************************//**
 * @brief The `sine_lut` is a constant array of 128 unsigned 16-bit integers. It
 * is likely used as a lookup table for sine wave values, providing
 * precomputed values to optimize performance in applications requiring
 * sine wave generation or processing.
 *
 * @details This variable is used to provide quick access to sine wave values,
 * reducing computational overhead in digital signal processing tasks.
 ******************************************************************************/
extern const uint16_t sine_lut[128];

/***************************************************************************//**
 * @brief `sine_lut_iq` is a global constant array of 1024 unsigned 32-bit
 * integers. It is likely used as a lookup table for sine wave values in
 * IQ (In-phase and Quadrature) format, which is common in digital signal
 * processing applications.
 *
 * @details This variable is used to provide precomputed sine wave values for
 * efficient signal generation or processing in the AXI DAC core
 * module.
 ******************************************************************************/
extern const uint32_t sine_lut_iq[1024];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function initializes the initial setup for an AXI DAC device by
 * allocating memory for the device descriptor and setting its properties
 * based on the provided initialization structure. It should be called
 * before any other operations on the DAC device and is typically
 * followed by a call to `axi_dac_init_finish` to complete the
 * initialization process. The function ensures that the `dac_core`
 * pointer is updated to point to the newly allocated and initialized
 * device descriptor. If memory allocation fails, the function returns an
 * error code.
 *
 * @param dac_core A pointer to a pointer of type `struct axi_dac`. This
 * parameter must not be null, and upon successful execution, it
 * will point to the newly allocated and initialized AXI DAC
 * device descriptor. The caller is responsible for managing the
 * memory of the pointed-to structure.
 * @param init A pointer to a constant `struct axi_dac_init` that contains the
 * initialization parameters for the DAC device. This parameter must
 * not be null and should be properly initialized with valid values
 * before calling the function.
 * @return Returns 0 on successful initialization, or -1 if memory allocation
 * fails.
 ******************************************************************************/
int32_t axi_dac_init_begin(struct axi_dac **dac_core,
			   const struct axi_dac_init *init);
/***************************************************************************//**
 * @brief This function finalizes the initialization process of an AXI DAC
 * device by verifying its status and calculating the effective clock
 * frequency. It should be called after the initial setup of the DAC has
 * been performed using other initialization functions. The function
 * checks the status register to ensure the device is ready and
 * calculates the clock frequency based on the device's configuration. If
 * the status check fails, the function returns an error code. Successful
 * completion of this function indicates that the DAC is ready for
 * operation.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device to
 * be initialized. This pointer must not be null, and the structure
 * should be properly initialized with relevant device information
 * before calling this function.
 * @return Returns 0 on successful initialization, or -1 if the status check
 * fails, indicating an error in the initialization process.
 ******************************************************************************/
int32_t axi_dac_init_finish(struct axi_dac *dac);
/** AXI DAC Main Initialization */
int32_t axi_dac_init(struct axi_dac **dac_core,
		     const struct axi_dac_init *init);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a
 * previously initialized AXI DAC device. It should be called when the
 * DAC is no longer needed to ensure that memory is properly freed. This
 * function must be called after the DAC has been initialized and used,
 * and it is the caller's responsibility to ensure that the `dac`
 * parameter is valid and not null. Failure to call this function may
 * result in memory leaks.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device to
 * be removed. Must not be null. The caller retains ownership of the
 * pointer, but the memory it points to will be freed by this
 * function.
 * @return Returns 0 on successful deallocation of resources. The function does
 * not modify any input parameters.
 ******************************************************************************/
int32_t axi_dac_remove(struct axi_dac *dac);
/***************************************************************************//**
 * @brief This function sets the data source for a specific channel of the AXI
 * DAC or for all channels if a negative channel number is provided. It
 * is used to configure the data path for the DAC, allowing selection
 * from predefined data sources such as DDS, SED, or DMA. This function
 * must be called after the DAC has been initialized. It writes the
 * selected data source to the control register of the specified
 * channel(s) and synchronizes the DAC configuration.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan An integer specifying the channel number to configure. If
 * negative, all channels are configured with the same data source.
 * @param sel An enum 'axi_dac_data_sel' value indicating the data source to
 * select. Valid values are defined in the 'axi_dac_data_sel'
 * enumeration.
 * @return Returns 0 on success. The function does not modify the input
 * parameters.
 ******************************************************************************/
int32_t axi_dac_set_datasel(struct axi_dac *dac,
			    int32_t chan,
			    enum axi_dac_data_sel sel);
/***************************************************************************//**
 * @brief Use this function to configure the frequency of a specific Direct
 * Digital Synthesis (DDS) channel on an AXI DAC device. This function
 * should be called when you need to update the frequency of a channel
 * after the DAC has been initialized. Ensure that the `dac` pointer is
 * valid and that the specified channel index is within the range of
 * available channels. The frequency is specified in hertz and should be
 * a positive value. The function does not perform any error checking on
 * the input parameters, so invalid inputs may lead to undefined
 * behavior.
 *
 * @param dac A pointer to an initialized `axi_dac` structure representing the
 * DAC device. Must not be null.
 * @param chan The index of the channel to configure. Must be within the range
 * of available channels on the DAC.
 * @param freq_hz The desired frequency in hertz for the specified channel. Must
 * be a positive value.
 * @return Returns 0 on success. The function does not perform error checking,
 * so the return value does not indicate parameter validation.
 ******************************************************************************/
int32_t axi_dac_dds_set_frequency(struct axi_dac *dac,
				  uint32_t chan, uint32_t freq_hz);
/***************************************************************************//**
 * @brief This function is used to obtain the current Direct Digital Synthesis
 * (DDS) frequency for a specific channel of the AXI DAC device. It is
 * essential to call this function when you need to verify or utilize the
 * frequency setting of a channel in your application. The function
 * requires a valid AXI DAC device descriptor and a channel number within
 * the range of available channels. The frequency is returned through a
 * pointer, and the function will return an error code if the operation
 * fails.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the AXI
 * DAC device. Must not be null.
 * @param chan The channel number for which the frequency is to be retrieved.
 * Must be within the range of available channels in the DAC.
 * @param freq A pointer to a uint32_t where the frequency will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int32_t axi_dac_dds_get_frequency(struct axi_dac *dac,
				  uint32_t chan, uint32_t *freq);
/***************************************************************************//**
 * @brief Use this function to configure the phase of a specific Direct Digital
 * Synthesis (DDS) channel on an AXI DAC device. This function is
 * typically called after the DAC has been initialized and when you need
 * to adjust the phase of the output signal for a particular channel. The
 * phase is specified in milli-degrees, and the function ensures that the
 * phase is correctly set by writing to the appropriate registers. It is
 * important to ensure that the `dac` pointer is valid and that the
 * specified channel exists on the device.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device.
 * Must not be null and should point to a properly initialized DAC
 * device.
 * @param chan The channel number for which the phase is to be set. Must be a
 * valid channel number for the given DAC device.
 * @param phase The desired phase in milli-degrees. The value should be within
 * the range that the DAC device supports for phase settings.
 * @return Returns 0 on success, indicating that the phase was set successfully.
 ******************************************************************************/
int32_t axi_dac_dds_set_phase(struct axi_dac *dac,
			      uint32_t chan, uint32_t phase);
/***************************************************************************//**
 * @brief This function is used to obtain the current phase setting of a
 * specific Direct Digital Synthesis (DDS) channel on an AXI DAC device.
 * It is typically called when there is a need to read back the phase
 * configuration for monitoring or verification purposes. The function
 * requires a valid `axi_dac` device descriptor and a channel number
 * within the range of available channels on the device. The phase value
 * is returned in milli-degrees through the provided pointer. The
 * function assumes that the DAC has been properly initialized and
 * configured before calling. It returns 0 on success, indicating that
 * the phase was successfully retrieved.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device.
 * Must not be null and should point to a valid, initialized DAC
 * device descriptor.
 * @param chan The channel number for which the phase is to be retrieved. Must
 * be within the range of available channels on the DAC device.
 * @param phase A pointer to a uint32_t where the phase value will be stored.
 * Must not be null. The phase is returned in milli-degrees.
 * @return Returns 0 on success, indicating the phase was successfully
 * retrieved. The phase value is written to the location pointed to by
 * `phase`.
 ******************************************************************************/
int32_t axi_dac_dds_get_phase(struct axi_dac *dac,
			      uint32_t chan, uint32_t *phase);
/***************************************************************************//**
 * @brief This function sets the Direct Digital Synthesis (DDS) scale for a
 * specified channel on the AXI DAC device. It is used to adjust the
 * amplitude of the signal generated by the DAC. The function must be
 * called with a valid DAC device descriptor and a channel number within
 * the range supported by the device. The scale is specified in micro
 * units, where 1,000,000 represents a full-scale value. Negative values
 * are allowed and will be processed accordingly. The function ensures
 * that the scale is clamped to a maximum of 1,999,000 micro units. It is
 * important to ensure that the DAC has been properly initialized before
 * calling this function.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan The channel number for which the scale is to be set. Must be
 * within the range of available channels on the DAC device.
 * @param scale_micro_units The desired scale in micro units, where 1,000,000
 * represents full scale. Negative values are allowed
 * and will be processed as such. Values are clamped to
 * a maximum of 1,999,000 micro units.
 * @return Returns 0 on success. The function does not modify the input
 * parameters.
 ******************************************************************************/
int32_t axi_dac_dds_set_scale(struct axi_dac *dac,
			      uint32_t chan,
			      int32_t scale_micro_units);
/***************************************************************************//**
 * @brief Use this function to obtain the current scale factor of a Direct
 * Digital Synthesis (DDS) channel in a DAC device, expressed in micro
 * units. This function is typically called when you need to read back
 * the scale setting for a specific channel to verify or log the current
 * configuration. Ensure that the `dac` structure is properly initialized
 * and that the `chan` parameter corresponds to a valid channel within
 * the DAC device. The function writes the scale factor to the provided
 * pointer `scale_micro_units`. It is expected that the
 * `scale_micro_units` pointer is valid and non-null.
 *
 * @param dac A pointer to an initialized `axi_dac` structure representing the
 * DAC device. Must not be null.
 * @param chan The channel number for which the scale factor is to be retrieved.
 * Must be a valid channel index within the DAC device.
 * @param scale_micro_units A pointer to an `int32_t` where the scale factor in
 * micro units will be stored. Must not be null.
 * @return Returns 0 on success. The scale factor is written to the location
 * pointed to by `scale_micro_units`.
 ******************************************************************************/
int32_t axi_dac_dds_get_scale(struct axi_dac *dac,
			      uint32_t chan,
			      int32_t *scale_micro_units);
/***************************************************************************//**
 * @brief This function configures the buffer for an AXI DAC device by writing
 * interleaved I and Q data from the provided buffer to the specified
 * address. It should be called when you need to update the DAC's data
 * buffer with new values. The function assumes that the buffer contains
 * interleaved I and Q samples and that the buffer size is an even
 * number, as it processes two samples at a time. Ensure that the DAC
 * device is properly initialized before calling this function.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param address The base address where the buffer data will be written. It
 * should be a valid address for the DAC device.
 * @param buff A pointer to an array of uint16_t containing interleaved I and Q
 * samples. Must not be null and should have at least 'buff_size'
 * elements.
 * @param buff_size The number of elements in the buffer. Must be an even
 * number, as the function processes two elements (I and Q) at
 * a time.
 * @return Returns 0 on success. No other return values are specified.
 ******************************************************************************/
int32_t axi_dac_set_buff(struct axi_dac *dac,
			 uint32_t address,
			 uint16_t *buff,
			 uint32_t buff_size);
/***************************************************************************//**
 * @brief This function sets up the DAC to output a sine wave by writing a
 * predefined sine lookup table to the specified memory address. It
 * should be called when the DAC is initialized and ready to be
 * configured for sine wave generation. The function supports both dual
 * and quad channel configurations, automatically adjusting the data
 * writing process based on the number of channels. The caller must
 * ensure that the DAC structure is properly initialized and that the
 * address provided is valid and writable. The function returns the
 * length of the data written, which is dependent on the number of
 * channels in the DAC.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param address A 32-bit unsigned integer representing the base memory address
 * where the sine lookup table will be written. Must be a valid
 * and writable address.
 * @return Returns a 32-bit unsigned integer representing the length of the data
 * written to the DAC, calculated based on the number of channels.
 ******************************************************************************/
uint32_t axi_dac_set_sine_lut(struct axi_dac *dac,
			      uint32_t address);
/***************************************************************************//**
 * @brief This function is used to obtain the scale calibration values for a
 * specific channel of an AXI DAC device. It is typically called when
 * there is a need to verify or utilize the current calibration settings
 * of the DAC channel. The function requires a valid AXI DAC device
 * descriptor and a channel number within the range of available
 * channels. The retrieved calibration values are stored in the provided
 * integer pointers. Ensure that the pointers are valid and that the DAC
 * device has been properly initialized before calling this function.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan The channel number for which the calibration scale values are to
 * be retrieved. Must be within the range of available channels on
 * the DAC device.
 * @param val A pointer to an integer where the first calibration scale value
 * will be stored. Must not be null.
 * @param val2 A pointer to an integer where the second calibration scale value
 * will be stored. Must not be null.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. The calibration scale values are written to the provided
 * pointers 'val' and 'val2'.
 ******************************************************************************/
int32_t axi_dac_dds_get_calib_scale(struct axi_dac *dac,
				    uint32_t chan,
				    int32_t *val,
				    int32_t *val2);
/***************************************************************************//**
 * @brief Use this function to obtain the current phase calibration values for a
 * specific channel of an AXI DAC device. This function is typically
 * called when you need to verify or utilize the phase calibration
 * settings of a DAC channel. Ensure that the DAC device has been
 * properly initialized before calling this function. The function will
 * populate the provided pointers with the calibration values, which can
 * then be used for further processing or analysis.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan The channel number for which the phase calibration values are to
 * be retrieved. Must be a valid channel index for the given DAC
 * device.
 * @param val A pointer to an int32_t where the first phase calibration value
 * will be stored. Must not be null.
 * @param val2 A pointer to an int32_t where the second phase calibration value
 * will be stored. Must not be null.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t axi_dac_dds_get_calib_phase(struct axi_dac *dac,
				    uint32_t chan,
				    int32_t *val,
				    int32_t *val2);
/***************************************************************************//**
 * @brief Use this function to set the calibration scale for a specific channel
 * of an AXI DAC device. This function is typically called when you need
 * to adjust the output scale of a DAC channel to match a desired
 * calibration standard. It is important to ensure that the DAC device
 * has been properly initialized before calling this function. The
 * function does not perform any validation on the input values, so it is
 * the caller's responsibility to provide appropriate calibration values.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan The channel number to calibrate. Must be a valid channel index
 * for the given DAC device.
 * @param val The primary calibration scale value to set. The specific range and
 * meaning depend on the DAC's calibration requirements.
 * @param val2 An additional calibration scale value, used in conjunction with
 * 'val'. The specific range and meaning depend on the DAC's
 * calibration requirements.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t axi_dac_dds_set_calib_scale(struct axi_dac *dac,
				    uint32_t chan,
				    int32_t val,
				    int32_t val2);
/***************************************************************************//**
 * @brief This function is used to set the calibration phase for a specified
 * channel of an AXI DAC device. It is typically called when precise
 * phase adjustments are needed for signal generation. The function
 * requires a valid AXI DAC device descriptor and a channel number within
 * the range of available channels. The phase calibration values are
 * provided as two integer parameters, which allow for fine-tuning of the
 * phase. It is important to ensure that the DAC device has been properly
 * initialized before calling this function.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param chan The channel number to calibrate. Must be within the range of
 * available channels on the DAC device.
 * @param val An integer representing the primary phase calibration value. The
 * valid range depends on the specific DAC configuration.
 * @param val2 An integer representing the secondary phase calibration value.
 * The valid range depends on the specific DAC configuration.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t axi_dac_dds_set_calib_phase(struct axi_dac *dac,
				    uint32_t chan,
				    int32_t val,
				    int32_t val2);
/***************************************************************************//**
 * @brief This function is used to load a specified amount of custom IQ data
 * into the AXI DAC at a given memory address. It is typically called
 * when there is a need to transmit custom data through the DAC channels.
 * The function assumes that the DAC has been properly initialized and
 * configured before use. It writes the provided data to all available
 * transmission channels and synchronizes the DAC to ensure the data is
 * ready for transmission. The function does not handle invalid input
 * parameters, so it is the caller's responsibility to ensure that the
 * inputs are valid and that the DAC is in a suitable state for data
 * loading.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param custom_data_iq A pointer to an array of 32-bit unsigned integers
 * containing the custom IQ data to be loaded. Must not be
 * null.
 * @param custom_tx_count The number of IQ data entries to load. Must be a non-
 * negative integer.
 * @param address The memory address where the custom data will be loaded. Must
 * be a valid address for the DAC's memory space.
 * @return Returns 0 on successful data loading. No error codes are returned for
 * invalid inputs.
 ******************************************************************************/
int32_t axi_dac_load_custom_data(struct axi_dac *dac,
				 const uint32_t *custom_data_iq,
				 uint32_t custom_tx_count,
				 uint32_t address);
/***************************************************************************//**
 * @brief This function is used to configure the data setup for an AXI DAC
 * device by setting the frequency, phase, and scale for each channel
 * based on the provided configuration. It should be called after the DAC
 * has been initialized and before starting data transmission. If the
 * `channels` field in the `axi_dac` structure is not null, it uses the
 * specified channel configurations; otherwise, it applies default
 * settings. This function assumes that the `dac` parameter is a valid
 * pointer to an initialized `axi_dac` structure.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device.
 * Must not be null and should be properly initialized before calling
 * this function. The function does not handle null pointers and
 * expects the `num_channels` field to be correctly set.
 * @return Returns 0 on successful configuration. The function does not handle
 * errors explicitly and assumes valid input.
 ******************************************************************************/
int32_t axi_dac_data_setup(struct axi_dac *dac);
/***************************************************************************//**
 * @brief This function reads data from a specified register of an AXI DAC
 * device using the QSPI bus type. It should be called when you need to
 * retrieve data from a specific register address of a DAC device that is
 * configured to use the QSPI bus. The function requires that the DAC
 * device is already initialized and configured to use the QSPI bus type.
 * It performs a read operation and stores the result in the provided
 * memory location. The function returns an error code if the bus type is
 * not QSPI or if any other error occurs during the read operation.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. The device must be configured to use the QSPI bus type.
 * Must not be null.
 * @param reg_addr The register address from which to read data. It is a 32-bit
 * unsigned integer specifying the target register within the
 * DAC device.
 * @param reg_data A pointer to a 32-bit unsigned integer where the read data
 * will be stored. Must not be null, and the caller is
 * responsible for allocating memory for this pointer.
 * @param data_size The size of the data to read, specified in bytes. It is an
 * 8-bit unsigned integer and should be a valid size for the
 * data being read.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if the bus type is not QSPI.
 ******************************************************************************/
int32_t axi_dac_bus_read(struct axi_dac *dac,
			 uint32_t reg_addr,
			 uint32_t *reg_data,
			 uint8_t data_size);
/***************************************************************************//**
 * @brief This function writes a given data value to a specified register
 * address on an AXI DAC device using the QSPI bus interface. It should
 * be used when the DAC is configured to use the QSPI bus type. The
 * function handles data sizes of either 8 or 16 bits, and it ensures
 * that the write operation is completed before returning. It is
 * important to ensure that the `dac` structure is properly initialized
 * and configured to use the QSPI bus before calling this function. The
 * function returns an error code if the bus type is not QSPI or if the
 * write operation fails.
 *
 * @param dac A pointer to an initialized `axi_dac` structure. Must not be null
 * and must be configured to use the QSPI bus type.
 * @param reg_addr The register address to which the data will be written. Must
 * be a valid register address for the DAC.
 * @param reg_data The data to be written to the specified register. The data
 * size is determined by the `data_size` parameter.
 * @param data_size The size of the data to be written, either 1 for 8-bit or 2
 * for 16-bit data. Invalid sizes will result in incorrect
 * behavior.
 * @return Returns 0 on success, or a negative error code if the bus type is not
 * QSPI or if the write operation fails.
 ******************************************************************************/
int32_t axi_dac_bus_write(struct axi_dac *dac,
			  uint32_t reg_addr,
			  uint32_t reg_data,
			  uint8_t data_size);
/***************************************************************************//**
 * @brief This function is used to configure the AXI DAC to operate in either
 * single-data-rate (SDR) or double-data-rate (DDR) mode. It should be
 * called when there is a need to switch the data rate mode of the DAC,
 * typically during initialization or reconfiguration of the DAC
 * settings. The function does not perform any validation on the input
 * parameters, so it is the caller's responsibility to ensure that the
 * `dac` pointer is valid and properly initialized before calling this
 * function. The function always returns 0, indicating successful
 * execution.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device.
 * Must not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) DDR mode for the DAC.
 * @return The function returns an integer value of 0, indicating successful
 * execution.
 ******************************************************************************/
int32_t axi_dac_set_ddr(struct axi_dac *dac,
			bool enable);
/***************************************************************************//**
 * @brief This function configures the I/O mode of the specified AXI DAC device.
 * It should be called when there is a need to change the communication
 * mode of the DAC, such as switching between SPI, DSPI, or QSPI modes.
 * The function requires a valid AXI DAC device descriptor and a valid
 * I/O mode enumeration value. It is expected that the DAC device has
 * been properly initialized before calling this function. The function
 * does not perform any error checking on the input parameters and
 * assumes they are valid.
 *
 * @param dac A pointer to an axi_dac structure representing the DAC device.
 * Must not be null and should point to a valid, initialized AXI DAC
 * device descriptor.
 * @param mode An enumeration value of type axi_io_mode specifying the desired
 * I/O mode. Valid values are AXI_DAC_IO_MODE_SPI,
 * AXI_DAC_IO_MODE_DSPI, and AXI_DAC_IO_MODE_QSPI.
 * @return Returns 0 on success. The function does not perform any error
 * checking and assumes inputs are valid.
 ******************************************************************************/
int32_t axi_dac_set_io_mode(struct axi_dac *dac,
			    enum axi_io_mode mode);
/***************************************************************************//**
 * @brief Use this function to control the data stream mode of an AXI DAC
 * device. It allows enabling or disabling the data stream functionality,
 * which is essential for managing how data is processed and output by
 * the DAC. This function should be called when you need to change the
 * data stream state, typically after initializing the DAC and before
 * starting data transmission. Ensure that the `dac` parameter is a
 * valid, initialized pointer to an `axi_dac` structure.
 *
 * @param dac A pointer to an `axi_dac` structure representing the DAC device.
 * Must not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the data stream mode.
 * @return Always returns 0, indicating successful execution.
 ******************************************************************************/
int32_t axi_dac_set_data_stream(struct axi_dac *dac,
				bool enable);
/***************************************************************************//**
 * @brief This function sets the data transfer address for the AXI DAC, which is
 * used as a sample register address when the DAC is configured or as a
 * stream start address when the finite state machine (FSM) is in stream
 * state. It should be called when configuring the DAC for data transfer
 * operations. The function assumes that the DAC has been properly
 * initialized before calling.
 *
 * @param dac A pointer to an initialized 'struct axi_dac' representing the DAC
 * device. Must not be null.
 * @param address A 32-bit unsigned integer representing the address to be set
 * for data transfer. The valid range is determined by the DAC's
 * addressable space.
 * @return Returns 0 on success, indicating the address was set without error.
 ******************************************************************************/
int32_t axi_dac_data_transfer_addr(struct axi_dac *dac,
				   uint32_t address);
/***************************************************************************//**
 * @brief Use this function to configure the data format of the AXI DAC to
 * either 8-bit or 16-bit symbols. This function should be called when
 * you need to change the data format for the DAC operation. It is
 * important to ensure that the format parameter is either 8 or 16, as
 * these are the only supported formats. If an unsupported format is
 * provided, the function will return an error code.
 *
 * @param dac A pointer to an axi_dac structure representing the DAC device.
 * Must not be null, and the DAC should be properly initialized
 * before calling this function.
 * @param format An integer specifying the desired data format. Valid values are
 * 8 and 16. If the value is not 8 or 16, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the format is
 * invalid.
 ******************************************************************************/
int32_t axi_dac_data_format_set(struct axi_dac *dac,
				int format);

#endif

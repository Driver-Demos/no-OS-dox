/*******************************************************************************
 *   @file   spi_engine.h
 *   @brief  Header file of SPI Engine core features.
 *   @author Sergiu Cuciurean (sergiu.cuciurean@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
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

#ifndef SPI_ENGINE_H
#define SPI_ENGINE_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "xilinx_spi.h"
#include "spi_engine_private.h"
#include "axi_dmac.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define OFFLOAD_DISABLED		0x00
#define OFFLOAD_TX_EN			NO_OS_BIT(0)
#define OFFLOAD_RX_EN			NO_OS_BIT(1)
#define OFFLOAD_TX_RX_EN		OFFLOAD_TX_EN | OFFLOAD_RX_EN

#define SPI_ENGINE_MSG_QUEUE_END	0xFFFFFFFF

/* Spi engine commands */
#define	WRITE(no_bytes)			((SPI_ENGINE_INST_TRANSFER << 12) |\
	(SPI_ENGINE_INSTRUCTION_TRANSFER_W << 8) | no_bytes)

#define	READ(no_bytes)			((SPI_ENGINE_INST_TRANSFER << 12) |\
	(SPI_ENGINE_INSTRUCTION_TRANSFER_R << 8) | no_bytes)

#define	WRITE_READ(no_bytes)		((SPI_ENGINE_INST_TRANSFER << 12) |\
	(SPI_ENGINE_INSTRUCTION_TRANSFER_RW << 8) | no_bytes)

#define SLEEP(time)			SPI_ENGINE_CMD_SLEEP(time & 0xF)

/* The delay and chip select parmeters are passed over the engine descriptor
and will be used inside the function call */
#define CS_HIGH				SPI_ENGINE_CMD_ASSERT(0x03, 0xFF)
#define CS_LOW				SPI_ENGINE_CMD_ASSERT(0x03, 0x00)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `spi_engine_init_param` structure is used to define the
 * initialization parameters for an SPI engine. It includes configuration
 * settings such as the reference clock frequency, the type of SPI
 * implementation, and the base address of the HDL core. Additionally, it
 * specifies the delay between chip select toggling and the start of the
 * serial clock, the data width for SPI transfers, and the idle state of
 * the SDO line when the chip select is inactive or during read-only
 * operations. This structure is essential for setting up the SPI engine
 * with the correct parameters for operation.
 *
 * @param ref_clk_hz SPI engine reference clock frequency in hertz.
 * @param type Type of SPI implementation, defined by the enum xil_spi_type.
 * @param spi_engine_baseaddr Base address where the HDL core is located.
 * @param cs_delay Delay between the chip select toggle and the start of the
 * serial clock.
 * @param data_width Width of one SPI transfer in bits.
 * @param sdo_idle_state Output state of SDO when chip select is inactive or
 * during read-only transfers.
 ******************************************************************************/
struct spi_engine_init_param {
	/** SPI engine reference clock */
	uint32_t	ref_clk_hz;
	/** Type of implementation */
	enum xil_spi_type	type;
	/** Base address where the HDL core is situated */
	uint32_t 		spi_engine_baseaddr;
	/** Delay between the CS toggle and the start of SCLK */
	uint32_t		cs_delay;
	/** Data with of one SPI transfer ( in bits ) */
	uint8_t			data_width;
	/**  output of SDO when CS is inactive or read-only transfers */
	uint8_t			sdo_idle_state;
};


/***************************************************************************//**
 * @brief The `spi_engine_desc` structure represents an SPI engine device,
 * encapsulating various parameters and configurations necessary for SPI
 * communication. It includes references to clock settings, DMAC pointers
 * for handling data transmission and reception, and configuration
 * settings for transfer modes and data widths. The structure also
 * specifies base addresses for the SPI engine and DMAC cores, as well as
 * timing parameters such as chip select delay and clock division. This
 * comprehensive structure is designed to facilitate efficient and
 * flexible SPI operations, supporting both transmission and reception
 * with configurable data widths and offload capabilities.
 *
 * @param ref_clk_hz SPI engine reference clock frequency in hertz.
 * @param type Specifies the type of SPI implementation.
 * @param offload_tx_dma Pointer to a DMAC used for transmission.
 * @param offload_rx_dma Pointer to a DMAC used for reception.
 * @param cyclic Transfer mode for the Tx DMAC.
 * @param offload_config Configuration for offload module transfer direction:
 * TX, RX, or both.
 * @param offload_tx_len Number of words to be sent by the module.
 * @param offload_rx_len Number of words to be received by the module.
 * @param spi_engine_baseaddr Base address of the HDL core.
 * @param rx_dma_baseaddr Base address of the RX DMAC core.
 * @param tx_dma_baseaddr Base address of the TX DMAC core.
 * @param cs_delay Delay between chip select toggle and start of SCLK.
 * @param clk_div Clock divider used for transmission delays.
 * @param data_width Data width of one SPI transfer in bits.
 * @param max_data_width Maximum data width supported by the engine.
 * @param sdo_idle_state Output state of SDO when CS is inactive or during read-
 * only transfers.
 ******************************************************************************/
struct spi_engine_desc {
	/** SPI engine reference clock */
	uint32_t	ref_clk_hz;
	/** Type of implementation */
	enum xil_spi_type	type;
	/** Pointer to a DMAC used in transmission */
	struct axi_dmac		*offload_tx_dma;
	/** Pointer to a DMAC used in reception */
	struct axi_dmac		*offload_rx_dma;
	/** Transfer mode for Tx DMAC */
	enum cyclic_transfer cyclic;
	/** Offload's module transfer direction : TX, RX or both */
	uint8_t			offload_config;
	/** Number of words that the module has to send */
	uint8_t			offload_tx_len;
	/** Number of words that the module has to receive */
	uint8_t			offload_rx_len;
	/** Base address where the HDL core is situated */
	uint32_t		spi_engine_baseaddr;
	/** Base address where the RX DMAC core is situated */
	uint32_t		rx_dma_baseaddr;
	/** Base address where the TX DMAC core is situated */
	uint32_t		tx_dma_baseaddr;
	/** Delay between the CS toggle and the start of SCLK */
	uint8_t			cs_delay;
	/** Clock divider used in transmission delays */
	uint32_t		clk_div;
	/** Data with of one SPI transfer ( in bits ) */
	uint8_t			data_width;
	/** The maximum data width supported by the engine */
	uint8_t 		max_data_width;
	/**  output of SDO when CS is inactive or read-only transfers */
	uint8_t			sdo_idle_state;
};


/***************************************************************************//**
 * @brief The `spi_engine_offload_init_param` structure is used to initialize
 * the offload module of the SPI engine. It contains parameters such as
 * the base addresses for the RX and TX DMAC cores, DMAC flags which
 * default to DMA_CYCLIC if not specified, and the configuration for the
 * offload module's transfer direction, which can be set to transmit
 * (TX), receive (RX), or both.
 *
 * @param rx_dma_baseaddr Base address where the RX DMAC core is situated.
 * @param tx_dma_baseaddr Base address where the TX DMAC core is situated.
 * @param dma_flags DMAC flags, defaulting to DMA_CYCLIC if not initialized.
 * @param offload_config Offload's module transfer direction: TX, RX, or both.
 ******************************************************************************/
struct spi_engine_offload_init_param {
	/** Base address where the RX DMAC core is situated */
	uint32_t	rx_dma_baseaddr;
	/** Base address where the TX DMAC core is situated */
	uint32_t	tx_dma_baseaddr;
	/** DMAC flags - if not initialized, the default value is DMA_CYCLIC */
	uint32_t	dma_flags;
	/** Offload's module transfer direction : TX, RX or both */
	uint8_t		offload_config;
};

/***************************************************************************//**
 * @brief The `spi_engine_offload_message` structure is designed to encapsulate
 * the details of an offload message for the SPI engine. It includes
 * pointers to command buffers and data, as well as addresses for
 * transmission and reception, facilitating the management of SPI data
 * transfers in an offload context. This structure is crucial for
 * handling the specifics of SPI communication, such as the commands to
 * be executed and the data to be transmitted or received, thereby
 * enabling efficient and organized SPI operations.
 *
 * @param commands Pointer to the SPI engine commands buffer that will be sent.
 * @param no_commands Length of the SPI engine commands buffer.
 * @param commands_data Pointer of the data that will be sent over SPI.
 * @param tx_addr The address where the data that will be transmitted is
 * situated.
 * @param rx_addr The address where the data that will be received is situated.
 ******************************************************************************/
struct spi_engine_offload_message {
	/** Pointer to the SPI engine commands buffer that will be sent.
	 * The available commands can be found at:
	 * https://wiki.analog.com/resources/fpga/peripherals/spi_engine/
	*/
	uint32_t *commands;
	/** Length of the SPI engine commands buffer */
	uint32_t no_commands;
	/** Pointer of the data that will be sent over spi */
	uint32_t *commands_data;
	/** The address where the data that will be transmitted is situated */
	uint32_t tx_addr;
	/** The address where the data that will be received is situated */
	uint32_t rx_addr;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief `xil_spi_ops` is a constant external variable of type `struct
 * no_os_spi_platform_ops`. It is used to define the platform-specific
 * operations for the SPI engine on Xilinx platforms.
 *
 * @details This variable is used to provide a set of function pointers for SPI
 * operations specific to Xilinx hardware, facilitating the integration
 * of the SPI engine with Xilinx systems.
 ******************************************************************************/
extern const struct no_os_spi_platform_ops xil_spi_ops;

/* Write SPI Engine's axi registers */
int32_t spi_engine_write(struct spi_engine_desc *desc,
			 uint32_t reg_addr,
			 uint32_t reg_data);

/* Read SPI Engine's axi registers */
int32_t spi_engine_read(struct spi_engine_desc *desc,
			uint32_t reg_addr,
			uint32_t *reg_data);
/* Initialize the SPI engine device */
int32_t spi_engine_init(struct no_os_spi_desc **desc,
			const struct no_os_spi_init_param *param);

/* Write and read data over SPI using the SPI engine */
int32_t spi_engine_write_and_read(struct no_os_spi_desc *desc,
				  uint8_t *data,
				  uint16_t bytes_number);

/* Free the resources used by the SPI engine device */
int32_t spi_engine_remove(struct no_os_spi_desc *desc);

/* Initialize the SPI engine offload module */
int32_t spi_engine_offload_init(struct no_os_spi_desc *desc,
				const struct spi_engine_offload_init_param *param);

/* Write and read data over SPI using the offload module */
int32_t spi_engine_offload_transfer(struct no_os_spi_desc *desc,
				    struct spi_engine_offload_message msg,
				    uint32_t no_samples);

/* Set SPI transfer width */
int32_t spi_engine_set_transfer_width(struct no_os_spi_desc *desc,
				      uint8_t data_wdith);

/* Set SPI transfer speed */
void spi_engine_set_speed(struct no_os_spi_desc *desc,
			  uint32_t speed_hz);

#endif // SPI_ENGINE_H

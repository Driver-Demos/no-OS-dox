/***************************************************************************//**
 *   @file   axi_dmac.h
 *   @brief  Driver for the Analog Devices AXI-DMAC core.
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
#ifndef AXI_DMAC_H_
#define AXI_DMAC_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AXI_DMAC_REG_IRQ_MASK		0x80
#define AXI_DMAC_REG_IRQ_PENDING	0x84
#define AXI_DMAC_IRQ_SOT			NO_OS_BIT(0)
#define AXI_DMAC_IRQ_EOT			NO_OS_BIT(1)

#define AXI_DMAC_REG_INTF_DESC		0x010
#define AXI_DMAC_DMA_BPB_DEST		NO_OS_GENMASK(3,0)
#define AXI_DMAC_DMA_TYPE_DEST		NO_OS_GENMASK(5,4)
#define AXI_DMAC_DMA_BPB_SRC		NO_OS_GENMASK(11,8)
#define AXI_DMAC_DMA_TYPE_SRC		NO_OS_GENMASK(13,12)
//Define macro for src and dest

#define AXI_DMAC_REG_CTRL			0x400
#define AXI_DMAC_CTRL_ENABLE		NO_OS_BIT(0)
#define AXI_DMAC_CTRL_DISABLE		0u
#define AXI_DMAC_CTRL_PAUSE			NO_OS_BIT(1)

#define AXI_DMAC_REG_TRANSFER_ID		0x404
#define AXI_DMAC_REG_TRANSFER_SUBMIT	0x408
#define AXI_DMAC_TRANSFER_SUBMIT		NO_OS_BIT(0)
#define AXI_DMAC_QUEUE_FULL				NO_OS_BIT(0)
#define AXI_DMAC_REG_FLAGS				0x40c
#define AXI_DMAC_REG_DEST_ADDRESS		0x410
#define AXI_DMAC_REG_SRC_ADDRESS		0x414
#define AXI_DMAC_REG_X_LENGTH			0x418
#define AXI_DMAC_REG_Y_LENGTH			0x41c
#define AXI_DMAC_REG_DEST_STRIDE		0x420
#define AXI_DMAC_REG_SRC_STRIDE			0x424
#define AXI_DMAC_REG_TRANSFER_DONE		0x428

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `use_irq` enumeration defines two possible states for interrupt
 * usage in a system: `IRQ_DISABLED` and `IRQ_ENABLED`. This enum is used
 * to specify whether interrupts are enabled or disabled, providing a
 * simple mechanism to control interrupt behavior in the context of the
 * AXI-DMAC driver.
 *
 * @param IRQ_DISABLED Represents the state where interrupts are disabled, with
 * a value of 0.
 * @param IRQ_ENABLED Represents the state where interrupts are enabled, with a
 * value of 1.
 ******************************************************************************/
enum use_irq {
	IRQ_DISABLED = 0,
	IRQ_ENABLED = 1
};

/***************************************************************************//**
 * @brief The `dma_direction` enumeration defines the possible directions for
 * Direct Memory Access (DMA) operations, specifying whether data is
 * transferred from a device to memory, from memory to a device, or
 * between memory locations. It also includes an option for an invalid
 * direction, which can be used to signify an uninitialized or erroneous
 * state.
 *
 * @param INVALID_DIR Represents an invalid or uninitialized DMA direction.
 * @param DMA_DEV_TO_MEM Indicates a DMA transfer direction from device to
 * memory.
 * @param DMA_MEM_TO_DEV Indicates a DMA transfer direction from memory to
 * device.
 * @param DMA_MEM_TO_MEM Indicates a DMA transfer direction from memory to
 * memory.
 ******************************************************************************/
enum dma_direction {
	INVALID_DIR = 0,
	DMA_DEV_TO_MEM = 1,
	DMA_MEM_TO_DEV = 2,
	DMA_MEM_TO_MEM = 3
};

/***************************************************************************//**
 * @brief The `dma_flags` enumeration defines a set of flags used to control and
 * configure DMA (Direct Memory Access) operations. Each flag represents
 * a specific feature or behavior of the DMA transfer, such as cyclic
 * transfers, marking the last transfer, or enabling partial status
 * reporting. These flags are typically used to modify the behavior of
 * DMA operations in a hardware or software context, providing
 * flexibility in how data is transferred between memory and devices.
 *
 * @param DMA_CYCLIC Indicates that the DMA transfer is cyclic.
 * @param DMA_LAST Marks the last transfer in a sequence.
 * @param DMA_PARTIAL_REPORTING_EN Enables partial reporting of the DMA transfer
 * status.
 ******************************************************************************/
enum dma_flags {
	DMA_CYCLIC = 1,
	DMA_LAST = 2,
	DMA_PARTIAL_REPORTING_EN = 4
};

// Could be transformed to a bool
/***************************************************************************//**
 * @brief The `cyclic_transfer` enum is a simple enumeration used to define
 * whether a data transfer should be cyclic or not. It contains two
 * possible values: `NO`, indicating a non-cyclic transfer, and `CYCLIC`,
 * indicating a cyclic transfer. This enum is likely used in the context
 * of configuring DMA (Direct Memory Access) operations to specify if the
 * transfer should automatically restart upon completion.
 *
 * @param NO Represents a non-cyclic transfer with a value of 0.
 * @param CYCLIC Represents a cyclic transfer with a value of 1.
 ******************************************************************************/
enum cyclic_transfer {
	NO = 0,
	CYCLIC = 1
};

/***************************************************************************//**
 * @brief The `axi_dma_transfer` structure is used to define the parameters for
 * a Direct Memory Access (DMA) transfer operation in the Analog Devices
 * AXI-DMAC core. It includes fields for specifying the size of the
 * transfer, the source and destination addresses, and whether the
 * transfer is cyclic. Additionally, it contains a volatile boolean to
 * indicate the completion status of the transfer, allowing for efficient
 * monitoring of the transfer's progress.
 *
 * @param size Specifies the size of the data transfer in bytes.
 * @param transfer_done Indicates whether the data transfer has been completed.
 * @param cyclic Specifies if the transfer is cyclic or not, using the
 * cyclic_transfer enum.
 * @param src_addr Holds the source address for the data transfer.
 * @param dest_addr Holds the destination address for the data transfer.
 ******************************************************************************/
struct axi_dma_transfer {
	uint32_t size;
	volatile bool transfer_done;
	enum cyclic_transfer cyclic;
	uint32_t src_addr;
	uint32_t dest_addr;
};

/***************************************************************************//**
 * @brief The `axi_dmac` structure is designed to manage and configure the
 * Analog Devices AXI-DMAC core, which facilitates direct memory access
 * (DMA) operations. It includes fields for identifying the DMAC
 * instance, configuring interrupt options, specifying data transfer
 * directions, and managing transfer properties such as cyclic mode,
 * maximum length, and data widths. Additionally, it maintains the state
 * of the current transfer through a nested `axi_dma_transfer` structure
 * and tracks sub-transfer properties like initial address, remaining
 * size, and next source and destination addresses.
 *
 * @param name A constant character pointer to the name of the DMAC instance.
 * @param base A 32-bit unsigned integer representing the base address of the
 * DMAC.
 * @param irq_option An enumeration indicating whether interrupts are enabled or
 * disabled.
 * @param direction An enumeration specifying the direction of data transfer.
 * @param hw_cyclic A boolean indicating if hardware cyclic mode is enabled.
 * @param max_length A 32-bit unsigned integer representing the maximum transfer
 * length.
 * @param width_dst A 32-bit unsigned integer for the destination data width.
 * @param width_src A 32-bit unsigned integer for the source data width.
 * @param transfer A volatile structure representing the current DMA transfer
 * details.
 * @param init_addr A 32-bit unsigned integer for the initial address of the
 * transfer.
 * @param remaining_size A 32-bit unsigned integer for the remaining size of the
 * transfer.
 * @param next_src_addr A 32-bit unsigned integer for the next source address in
 * the transfer.
 * @param next_dest_addr A 32-bit unsigned integer for the next destination
 * address in the transfer.
 ******************************************************************************/
struct axi_dmac {
	const char *name;
	uint32_t base;
	enum use_irq irq_option;
	enum dma_direction direction;
	bool hw_cyclic;
	uint32_t max_length;
	uint32_t width_dst;
	uint32_t width_src;
	volatile struct axi_dma_transfer transfer;
	//Current sub-transfer properties
	uint32_t init_addr;
	uint32_t remaining_size;
	uint32_t next_src_addr;
	uint32_t next_dest_addr;
};

/***************************************************************************//**
 * @brief The `axi_dmac_init` structure is used to initialize an instance of the
 * AXI Direct Memory Access Controller (DMAC) by specifying its name,
 * base address, and interrupt configuration. This structure is essential
 * for setting up the DMAC hardware parameters before it can be used for
 * data transfer operations.
 *
 * @param name A constant character pointer representing the name of the DMAC
 * instance.
 * @param base A 32-bit unsigned integer representing the base address of the
 * DMAC.
 * @param irq_option An enumeration indicating whether interrupts are enabled or
 * disabled for the DMAC.
 ******************************************************************************/
struct axi_dmac_init {
	const char *name;
	uint32_t base;
	enum use_irq irq_option;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is designed to handle interrupts for device-to-memory
 * transfers using the AXI-DMAC core. It should be called in the context
 * of an interrupt service routine when a DMA transfer is in progress.
 * The function processes start-of-transfer (SOT) and end-of-transfer
 * (EOT) interrupts, managing the transfer of data by setting up
 * subsequent transfers if necessary. It is crucial that the `instance`
 * parameter points to a valid `axi_dmac` structure that has been
 * properly initialized and configured for a device-to-memory transfer.
 * The function updates the internal state of the `axi_dmac` structure,
 * including the remaining size of the transfer and the next destination
 * address, and marks the transfer as complete when finished.
 *
 * @param instance A pointer to an `axi_dmac` structure representing the DMA
 * controller instance. Must not be null and should be properly
 * initialized before calling this function. The function
 * assumes ownership of this pointer for the duration of the
 * call.
 * @return None
 ******************************************************************************/
void axi_dmac_dev_to_mem_isr(void *instance);
/***************************************************************************//**
 * @brief This function is designed to handle interrupts that occur during
 * memory-to-device DMA transfers using the AXI-DMAC core. It should be
 * called in the context of an interrupt service routine when an
 * interrupt is triggered for a DMA transfer. The function processes the
 * start-of-transfer (SOT) and end-of-transfer (EOT) interrupts, managing
 * the transfer state and preparing the next transfer if necessary. It is
 * crucial that the `instance` parameter points to a valid `axi_dmac`
 * structure that has been properly initialized before calling this
 * function.
 *
 * @param instance A pointer to an `axi_dmac` structure representing the DMA
 * controller instance. Must not be null and should be properly
 * initialized before use. The function assumes ownership of
 * this pointer for the duration of the call.
 * @return None
 ******************************************************************************/
void axi_dmac_mem_to_dev_isr(void *instance);
/***************************************************************************//**
 * @brief This function is designed to handle interrupts for memory-to-memory
 * DMA transfers using the AXI-DMAC core. It should be called within an
 * interrupt service routine context when a DMA transfer interrupt
 * occurs. The function processes the start-of-transfer (SOT) and end-of-
 * transfer (EOT) interrupts, managing the transfer of data between
 * memory locations. It requires a valid pointer to an `axi_dmac`
 * instance, which must be properly initialized before calling this
 * function. The function updates the DMA transfer state and addresses,
 * and it triggers subsequent transfers as needed. It is essential for
 * managing ongoing DMA operations and ensuring data is transferred
 * correctly.
 *
 * @param instance A pointer to an `axi_dmac` structure representing the DMA
 * controller instance. Must not be null and should be properly
 * initialized before use. The function assumes ownership of
 * this pointer for the duration of the call.
 * @return None
 ******************************************************************************/
void axi_dmac_mem_to_mem_isr(void *instance);
void axi_dmac_write_isr(void *instance);
/***************************************************************************//**
 * @brief This function reads a 32-bit value from a specified register address
 * of the AXI-DMAC device. It is typically used to retrieve configuration
 * or status information from the device. The function requires a valid
 * AXI-DMAC device structure and a register address to read from. The
 * result is stored in the provided output parameter. Ensure that the
 * `dmac` pointer is properly initialized and that `reg_data` is not null
 * before calling this function.
 *
 * @param dmac A pointer to an initialized `axi_dmac` structure representing the
 * AXI-DMAC device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the AXI-DMAC device's address space.
 * @param reg_data A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success. The value read from the register is stored in
 * the location pointed to by `reg_data`.
 ******************************************************************************/
int32_t axi_dmac_read(struct axi_dmac *dmac, uint32_t reg_addr,
		      uint32_t *reg_data);
/***************************************************************************//**
 * @brief Use this function to write a 32-bit value to a specific register of
 * the AXI-DMAC device. This function is typically called when
 * configuring or controlling the AXI-DMAC hardware. It is important to
 * ensure that the `dmac` parameter is a valid pointer to an initialized
 * `axi_dmac` structure before calling this function. The function does
 * not perform any error checking on the register address or data, so it
 * is the caller's responsibility to ensure that the provided register
 * address is valid for the specific AXI-DMAC device being used.
 *
 * @param dmac A pointer to an `axi_dmac` structure representing the AXI-DMAC
 * device. Must not be null and should be properly initialized
 * before use.
 * @param reg_addr The address of the register within the AXI-DMAC device to
 * which the data will be written. It should be a valid register
 * address for the device.
 * @param reg_data The 32-bit data to be written to the specified register. Any
 * 32-bit value is valid.
 * @return Returns 0 to indicate successful execution. No error codes are
 * returned.
 ******************************************************************************/
int32_t axi_dmac_write(struct axi_dmac *dmac, uint32_t reg_addr,
		       uint32_t reg_data);
/***************************************************************************//**
 * @brief This function checks the status of a DMA transfer associated with the
 * given `axi_dmac` instance and indicates whether the transfer is
 * complete. It should be called when you need to verify the completion
 * status of a DMA operation. The function requires a valid `axi_dmac`
 * instance and a non-null pointer to a boolean variable where the result
 * will be stored. The function does not handle invalid input parameters,
 * so ensure that the `dmac` pointer is properly initialized and `rdy` is
 * not null before calling this function.
 *
 * @param dmac A pointer to an `axi_dmac` structure representing the DMA
 * controller. Must be a valid, initialized instance.
 * @param rdy A pointer to a boolean variable where the function will store the
 * result. Must not be null.
 * @return Returns 0 on success, and the boolean pointed to by `rdy` is set to
 * true if the transfer is complete, false otherwise.
 ******************************************************************************/
int32_t axi_dmac_is_transfer_ready(struct axi_dmac *dmac, bool *rdy);
int32_t axi_dmac_init(struct axi_dmac **adc_core,
		      const struct axi_dmac_init *init);
/***************************************************************************//**
 * @brief This function is used to properly release and clean up resources
 * associated with a given AXI DMAC instance. It should be called when
 * the DMAC instance is no longer needed, ensuring that any allocated
 * memory is freed. The function must be called with a valid pointer to
 * an initialized `axi_dmac` structure. If the provided pointer is null,
 * the function will return an error code, indicating that no action was
 * taken. This function is essential for preventing memory leaks in
 * applications that utilize the AXI DMAC driver.
 *
 * @param dmac A pointer to an `axi_dmac` structure representing the DMAC
 * instance to be removed. Must not be null. If null, the function
 * returns an error code and does nothing.
 * @return Returns 0 on successful removal and resource deallocation, or -1 if
 * the input pointer is null.
 ******************************************************************************/
int32_t axi_dmac_remove(struct axi_dmac *dmac);
/***************************************************************************//**
 * @brief This function is used to start a Direct Memory Access (DMA) transfer
 * with the given parameters. It should be called when a transfer needs
 * to be initiated using the AXI-DMAC core. The function checks for valid
 * transfer parameters and ensures that the DMA controller is properly
 * configured before starting the transfer. It handles different transfer
 * directions and cyclic transfer modes, and it will return an error if
 * the configuration is not supported or if the addresses are not
 * properly aligned. The function must be called with a valid `axi_dmac`
 * instance and a properly configured `axi_dma_transfer` structure.
 *
 * @param dmac A pointer to an `axi_dmac` structure representing the DMA
 * controller. Must not be null. The structure should be initialized
 * and configured before calling this function.
 * @param dma_transfer A pointer to an `axi_dma_transfer` structure containing
 * the transfer parameters such as size, cyclic mode, source
 * address, and destination address. Must not be null. The
 * size must be greater than zero for a transfer to occur,
 * and addresses must be aligned according to the data path
 * width.
 * @return Returns 0 on success, or -1 on error if the transfer mode is not
 * supported, addresses are not aligned, or the DMA queue is full.
 ******************************************************************************/
int32_t axi_dmac_transfer_start(struct axi_dmac *dmac,
				struct axi_dma_transfer *dma_transfer);
/***************************************************************************//**
 * @brief This function is used to wait for the completion of a DMA transfer
 * initiated on the specified AXI DMAC device. It should be called after
 * starting a transfer to ensure that the transfer has completed
 * successfully. The function will block until the transfer is done or
 * the specified timeout period has elapsed. If the transfer does not
 * complete within the timeout period, the function returns an error.
 * This function handles both interrupt-driven and polling modes based on
 * the configuration of the DMAC device.
 *
 * @param dmac A pointer to an initialized 'struct axi_dmac' representing the
 * DMAC device. Must not be null. The DMAC device should be properly
 * configured and a transfer should be in progress.
 * @param timeout_ms The maximum time to wait for the transfer to complete, in
 * milliseconds. Must be a positive integer. If the transfer
 * does not complete within this time, the function returns an
 * error.
 * @return Returns 0 if the transfer completes successfully within the timeout
 * period, or -1 if the timeout is reached before completion.
 ******************************************************************************/
int32_t axi_dmac_transfer_wait_completion(struct axi_dmac *dmac,
		uint32_t timeout_ms);
/***************************************************************************//**
 * @brief Use this function to halt an active DMA transfer associated with the
 * specified AXI DMAC instance. It is typically called when a transfer
 * needs to be stopped prematurely or when cleaning up resources. Ensure
 * that the `dmac` parameter is a valid, initialized pointer to an
 * `axi_dmac` structure before calling this function. This function does
 * not return a value and does not provide feedback on whether the
 * transfer was successfully stopped.
 *
 * @param dmac A pointer to an `axi_dmac` structure representing the DMA
 * controller instance. Must not be null and should be properly
 * initialized before use.
 * @return None
 ******************************************************************************/
void axi_dmac_transfer_stop(struct axi_dmac *dmac);

#endif

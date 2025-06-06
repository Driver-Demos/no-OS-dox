/***************************************************************************//**
 *   @file   ssd_1306.h
 *   @brief  Header file for ssd_1306 Driver.
 *   @author Andrei Porumb (andrei.porumb@analog.com)
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

#ifndef SSD_1306_H
#define SSD_1306_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "display.h"
#include "no_os_gpio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ssd_1306_extra` structure is designed to hold additional
 * parameters required for the operation of the SSD1306 display
 * controller. It includes initialization parameters and descriptors for
 * both GPIO pins (Data/Command and RESET) and the SPI interface, which
 * are essential for configuring and managing the communication with the
 * display hardware.
 *
 * @param dc_pin_ip Pointer to the initial parameters for the Data/Command GPIO
 * pin.
 * @param reset_pin_ip Pointer to the initial parameters for the RESET GPIO pin.
 * @param dc_pin Pointer to the descriptor for the Data/Command GPIO pin.
 * @param reset_pin Pointer to the descriptor for the RESET GPIO pin.
 * @param spi_ip Pointer to the initial parameters for the SPI interface.
 * @param spi_desc Pointer to the descriptor for the SPI interface.
 ******************************************************************************/
typedef struct ssd_1306_extra {
	/** Data/Command pin gpio initial param */
	struct no_os_gpio_init_param	   *dc_pin_ip;
	/** RESET pin gpio initial param */
	struct no_os_gpio_init_param     *reset_pin_ip;
	/** Data/Command pin gpio desc */
	struct no_os_gpio_desc	       *dc_pin;
	/** RESET pin gpio desc*/
	struct no_os_gpio_desc     	   *reset_pin;
	/* SPI initial param */
	struct no_os_spi_init_param      *spi_ip;
	/* SPI descriptor*/
	struct no_os_spi_desc	           *spi_desc;
} ssd_1306_extra;

/***************************************************************************//**
 * @brief The `ssd1306_ops` is a constant structure of type
 * `display_controller_ops` that provides a set of operations for
 * controlling the SSD1306 display. This structure is likely to contain
 * function pointers that define the operations that can be performed on
 * the display, such as initialization, turning the display on or off,
 * moving the cursor, printing characters, and removing the display
 * resources.
 *
 * @details This variable is used to interface with the SSD1306 display by
 * providing a standardized set of operations for display management.
 ******************************************************************************/
extern const struct display_controller_ops ssd1306_ops;
/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the SSD1306 display device for operation by
 * initializing the necessary SPI and GPIO interfaces. It must be called
 * before any other operations on the display are performed. The function
 * configures the display's command and reset pins, and sends
 * initialization commands to prepare the display for use. It is
 * essential to ensure that the `device` parameter is properly configured
 * with valid SPI and GPIO initialization parameters before calling this
 * function. If any initialization step fails, the function returns an
 * error code.
 *
 * @param device A pointer to a `struct display_dev` that must be properly
 * initialized with valid `ssd_1306_extra` parameters. The `extra`
 * field of this structure should contain valid SPI and GPIO
 * initialization parameters. The pointer must not be null.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during the initialization process.
 ******************************************************************************/
int32_t ssd_1306_init(struct display_dev *device);

/***************************************************************************//**
 * @brief Use this function to turn the SSD1306 display on or off. It should be
 * called after the display has been properly initialized using the
 * appropriate initialization function. The function sends a command to
 * the display to change its power state based on the provided parameter.
 * Ensure that the device structure is correctly set up and that the
 * display is ready to receive commands. This function returns an error
 * code if the operation fails, which can occur if there is an issue with
 * the GPIO or SPI communication.
 *
 * @param device A pointer to a struct display_dev representing the display
 * device. This must be a valid, initialized device structure, and
 * must not be null.
 * @param on_off A uint8_t value where a non-zero value turns the display on,
 * and zero turns it off. The value is interpreted as a boolean.
 * @return Returns an int32_t value: 0 on success, or a negative error code if
 * the operation fails.
 ******************************************************************************/
int32_t ssd_1306_display_on_off(struct display_dev *device, uint8_t on_off);

/*  Moves cursor to desired row/column. */
/***************************************************************************//**
 * @brief Use this function to position the cursor on the SSD1306 display to a
 * specific row and column before performing operations like printing
 * characters. It is essential to ensure that the display device has been
 * properly initialized before calling this function. The function will
 * attempt to set the cursor position and return an error code if the
 * operation fails. This function is typically used in conjunction with
 * other display operations to control where text or graphics are
 * rendered on the screen.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null and should be properly initialized
 * before use.
 * @param row An 8-bit unsigned integer specifying the row to move the cursor
 * to. Valid values depend on the display's row configuration.
 * @param column An 8-bit unsigned integer specifying the column to move the
 * cursor to. Valid values depend on the display's column
 * configuration.
 * @return Returns an int32_t value: 0 on success, or a negative error code if
 * the operation fails.
 ******************************************************************************/
int32_t ssd_1306_move_cursor(struct display_dev *device, uint8_t row,
			     uint8_t column);

/***************************************************************************//**
 * @brief This function is used to display a single ASCII character at a
 * specified row and column on an SSD1306 display. It should be called
 * after the display has been initialized and is typically used to update
 * the display with new text content. The function moves the cursor to
 * the specified position and writes the character data to the display.
 * It is important to ensure that the row and column values are within
 * the valid range for the display to avoid unexpected behavior. The
 * function returns an error code if the operation fails, which can occur
 * if the cursor cannot be moved or if there is an issue with the display
 * communication.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null and should be properly initialized
 * before calling this function.
 * @param ascii An 8-bit unsigned integer representing the ASCII value of the
 * character to be printed. Only valid ASCII values should be used.
 * @param row An 8-bit unsigned integer specifying the row position on the
 * display where the character will be printed. Must be within the
 * valid range of rows for the display.
 * @param column An 8-bit unsigned integer specifying the column position on the
 * display where the character will be printed. Must be within the
 * valid range of columns for the display.
 * @return Returns an int32_t value indicating success (0) or failure (-1) of
 * the operation.
 ******************************************************************************/
int32_t ssd_1306_print_ascii(struct display_dev *device, uint8_t ascii,
			     uint8_t row, uint8_t column);

/***************************************************************************//**
 * @brief Use this function to clean up and release all resources associated
 * with an SSD1306 display device when it is no longer needed. This
 * function should be called to ensure that all GPIO and SPI resources
 * are properly freed, preventing resource leaks. It is important to call
 * this function only after the device has been initialized and used, and
 * before the program terminates or the device is re-initialized.
 *
 * @param device A pointer to a `struct display_dev` representing the SSD1306
 * device. This must not be null and should point to a valid,
 * initialized display device structure. The function will access
 * the `extra` field of this structure to release associated
 * resources.
 * @return Returns 0 on successful removal of all resources. Returns -1 if any
 * of the resource removals fail.
 ******************************************************************************/
int32_t ssd_1306_remove(struct display_dev *device);

#endif

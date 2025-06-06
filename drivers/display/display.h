/***************************************************************************//**
 *   @file   display.h
 *   @brief  Header file for display Driver.
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

#ifndef DISPLAY_H
#define DISPLAY_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `display_dev` structure is a descriptor for a display device,
 * encapsulating essential attributes such as the number of columns and
 * rows, and providing a mechanism to interface with the display
 * controller through a set of operations defined in
 * `display_controller_ops`. It also allows for the inclusion of extra
 * parameters specific to the device, enabling flexibility and
 * extensibility in handling various display hardware configurations.
 *
 * @param cols_nb Represents the number of columns in the display.
 * @param rows_nb Represents the number of rows in the display.
 * @param controller_ops Pointer to a structure containing function pointers for
 * display controller operations.
 * @param extra Pointer to additional device-specific parameters.
 ******************************************************************************/
struct display_dev {
	/** Number of columns */
	uint8_t                    cols_nb;
	/** Number of rows */
	uint8_t                    rows_nb;
	/** DISPLAY controller specific ops */
	const struct display_controller_ops *controller_ops;
	/**  Display extra parameters (device specific) */
	void		               *extra;
};

/***************************************************************************//**
 * @brief The `display_init_param` structure is used to define the initial
 * parameters required to set up a display device. It includes the number
 * of columns and rows for the display, a pointer to a set of operations
 * specific to the display controller, and a pointer to any additional
 * parameters that may be needed for device-specific configurations. This
 * structure is essential for initializing a display device with the
 * correct settings and operations.
 *
 * @param cols_nb Specifies the number of columns in the display.
 * @param rows_nb Specifies the number of rows in the display.
 * @param controller_ops Pointer to a structure containing display controller
 * specific operations.
 * @param extra Pointer to additional device-specific parameters.
 ******************************************************************************/
struct display_init_param {
	/** Number of columns */
	uint8_t                    cols_nb;
	/** Number of rows */
	uint8_t                    rows_nb;
	/** DISPLAY controller specific ops */
	const struct display_controller_ops *controller_ops;
	/**  Display extra parameters (device specific) */
	void		               *extra;
};

/***************************************************************************//**
 * @brief The `display_controller_ops` structure is a collection of function
 * pointers that define operations specific to a display controller.
 * These operations include initializing the controller, turning the
 * display on or off, moving the cursor, printing a character, and
 * removing resources. This structure allows for the abstraction of
 * display controller functionalities, enabling different implementations
 * to be used interchangeably by providing the appropriate function
 * pointers.
 *
 * @param init Function pointer to initialize the display controller.
 * @param display_on_off Function pointer to turn the display on or off.
 * @param move_cursor Function pointer to move the cursor on the display.
 * @param print_char Function pointer to print a character at a specified
 * position on the display.
 * @param remove Function pointer to remove resources allocated by the display
 * device.
 ******************************************************************************/
struct display_controller_ops {
	/** Initialize controller for display usage. */
	int32_t (*init)(struct display_dev *);
	/** Turn display on/off */
	int32_t (*display_on_off)(struct display_dev *, uint8_t);
	/** Move cursor */
	int32_t (*move_cursor)(struct display_dev *, uint8_t, uint8_t);
	/** Print character by ascii number */
	int32_t (*print_char)(struct display_dev *, uint8_t, uint8_t,
			      uint8_t);
	/** Removes resources allocated by device */
	int32_t (*remove)(struct display_dev *);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up a display device using the provided
 * initialization parameters. It must be called before any other
 * operations on the display device. The function allocates memory for
 * the device descriptor and initializes the display controller using the
 * specified operations. If the initialization is successful, the device
 * pointer is updated to point to the newly created display device. The
 * function returns an error code if the input parameters are invalid or
 * if memory allocation or controller initialization fails.
 *
 * @param device A pointer to a pointer of type `struct display_dev`. This must
 * not be null. On successful initialization, it will point to the
 * newly allocated display device structure. The caller is
 * responsible for managing the memory of this structure.
 * @param param A pointer to a `struct display_init_param` containing the
 * initialization parameters for the display device. This must not
 * be null and should include valid column and row numbers,
 * controller operations, and any extra parameters required by the
 * specific display.
 * @return Returns 0 on successful initialization. Returns a negative error code
 * if the input parameters are invalid, memory allocation fails, or the
 * controller initialization fails.
 ******************************************************************************/
int32_t display_init(struct display_dev **device,
		     const struct display_init_param *param);

/* Frees the resources allocated by display_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a display
 * device that was previously initialized. It should be called when the
 * display device is no longer needed to ensure proper cleanup and avoid
 * memory leaks. The function requires a valid display device pointer and
 * will return an error if the pointer is null or if the removal
 * operation fails.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device to be removed. Must not be null. If null, the function
 * returns -EINVAL. The caller retains ownership of the pointer,
 * but the resources it points to will be freed.
 * @return Returns 0 on successful removal of the device. Returns -EINVAL if the
 * device pointer is null, or -1 if the controller's remove operation
 * fails.
 ******************************************************************************/
int32_t display_remove(struct display_dev *device);

/***************************************************************************//**
 * @brief Use this function to power on the display associated with the given
 * device. It must be called with a valid display device that has been
 * properly initialized. This function is typically used when you want to
 * activate the display after it has been turned off or after
 * initialization. Ensure that the device parameter is not null before
 * calling this function, as passing a null pointer will result in an
 * error.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device to be turned on. Must not be null. The caller retains
 * ownership of the device structure. If the pointer is null, the
 * function returns an error code.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if the operation fails.
 ******************************************************************************/
int32_t display_on(struct display_dev *device);

/***************************************************************************//**
 * @brief Use this function to turn off a display device that has been
 * previously initialized. It is important to ensure that the device
 * parameter is not null before calling this function, as passing a null
 * pointer will result in an error. This function is typically used when
 * the display is no longer needed or before shutting down the system to
 * conserve power.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device to be turned off. Must not be null. If null, the
 * function returns an error code.
 * @return Returns 0 on success, or a negative error code if the device
 * parameter is null or if the operation fails.
 ******************************************************************************/
int32_t display_off(struct display_dev *device);

/***************************************************************************//**
 * @brief Use this function to position the cursor on a display device at a
 * specific row and column, which is necessary before performing
 * operations like printing characters. The function requires a valid
 * display device that has been initialized and supports cursor movement
 * operations. It is important to ensure that the specified row and
 * column are within the bounds of the display's dimensions. If the
 * device parameter is null, the function will return an error code.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null. The device should be properly
 * initialized and support cursor movement operations.
 * @param row An unsigned 8-bit integer specifying the row position to move the
 * cursor to. Should be within the valid range of rows for the
 * display.
 * @param column An unsigned 8-bit integer specifying the column position to
 * move the cursor to. Should be within the valid range of columns
 * for the display.
 * @return Returns an int32_t value. A negative error code is returned if the
 * device is null or if the operation fails; otherwise, a non-negative
 * value indicates success.
 ******************************************************************************/
int32_t display_move_cursor(struct display_dev *device, uint8_t row,
			    uint8_t column);

/***************************************************************************//**
 * @brief Use this function to clear all characters from the display,
 * effectively resetting it to a blank state. This function should be
 * called when you need to remove all existing content from the display.
 * It requires a valid display device descriptor and will return an error
 * if the device is not properly initialized. Ensure that the display
 * device is correctly set up before calling this function to avoid
 * errors.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null. The device should be properly
 * initialized before calling this function. If the device is
 * null, the function returns an error.
 * @return Returns 0 on success. If the device is null, returns -EINVAL. If an
 * error occurs while clearing the display, returns -1.
 ******************************************************************************/
int32_t display_clear(struct display_dev *device);

/***************************************************************************//**
 * @brief This function is used to print a string on a display device starting
 * from a specified row and column. It requires a valid display device
 * descriptor and a non-null message string. The function will attempt to
 * print each character of the string sequentially, moving to the next
 * line if the end of a row is reached. It is important to ensure that
 * the starting position is within the bounds of the display's
 * dimensions. The function returns an error code if the device or
 * message is invalid, or if printing fails at any point.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null. The device should be properly
 * initialized before calling this function.
 * @param msg A pointer to a null-terminated string to be printed on the
 * display. Must not be null.
 * @param row The starting row position on the display where the string will
 * begin to print. Must be within the valid range of the display's
 * rows.
 * @param column The starting column position on the display where the string
 * will begin to print. Must be within the valid range of the
 * display's columns.
 * @return Returns 0 on success, -EINVAL if the device or message is null, or -1
 * if printing fails.
 ******************************************************************************/
int32_t display_print_string(struct display_dev *device, char *msg,
			     uint8_t row, uint8_t column);

/***************************************************************************//**
 * @brief This function is used to print a single character at a specified row
 * and column on a display device. It requires a valid display device
 * that has been properly initialized. The function will not perform any
 * action if the device parameter is null, and it will return an error
 * code in such cases. This function is useful when precise control over
 * the display content is needed, such as updating a specific character
 * without affecting the rest of the display.
 *
 * @param device A pointer to a display_dev structure representing the display
 * device. Must not be null. The device should be initialized
 * before calling this function.
 * @param chr The character to be printed on the display. It is expected to be a
 * valid ASCII character.
 * @param row The row position on the display where the character will be
 * printed. It should be within the valid range of rows supported by
 * the display.
 * @param column The column position on the display where the character will be
 * printed. It should be within the valid range of columns
 * supported by the display.
 * @return Returns an int32_t value. A non-negative value indicates success,
 * while a negative value indicates an error, such as -EINVAL if the
 * device is null.
 ******************************************************************************/
int32_t display_print_char(struct display_dev *device, char chr,
			   uint8_t row, uint8_t column);

#endif

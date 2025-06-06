/***************************************************************************//**
 *   @file   nhd_c12832a1z.h
 *   @brief  Header file of nhd_c12832a1z
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
 ******************************************************************************/

#ifndef __NHD_C12832A1Z_H__
#define __NHD_C12832A1Z_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define NHD_C12832A1Z_RST_ON     	0U
#define NHD_C12832A1Z_RST_OFF    	1U
#define NHD_C12832A1Z_DC_DATA    	1U
#define NHD_C12832A1Z_DC_CMD     	0U
#define NHD_C12832A1Z_DISP_ON    	0xAFU
#define NHD_C12832A1Z_DISP_OFF   	0xAEU
#define NDH_C12832A1Z_ADC_NORMAL	0xA0U
#define NDH_C12832A1Z_ADC_REVERSE	0xA1U
#define NDH_C12832A1Z_COM_NORMAL	0xC0U
#define NDH_C12832A1Z_COM_REVERSE	0xC8U
#define NDH_C12832A1Z_LCD_BIAS		0xA2U
#define NDH_C12832A1Z_PWR_CTRL		0x2FU
#define NDH_C12832A1Z_RES_RATIO		0x21U
#define NDH_C12832A1Z_ELECTRIC_VOL	0x81U
#define NDH_C12832A1Z_ELECTRIC_VAL	0x20U

#define NHD_C12832A1Z_REVERSE		0x1
#define NHD_C12832A1Z_BLINK		0x2

#define NHD_C12832A1Z_BLINK_INTERVAL	500

#define FB_FLUSH_DELAY			30

#define NR_COLUMNS			128
#define NR_PAGES			4
#define NR_CHAR				64
#define PAGE_START_ADDR			0xB0
#define DISPLAY_START_OFFSET		0x40

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `nhd_c12832a1z_dev` structure is designed to represent a device
 * interface for the NHD-C12832A1Z display module. It encapsulates the
 * necessary hardware descriptors required for controlling the display,
 * including GPIO descriptors for the Data/Command and RESET pins, as
 * well as an SPI descriptor for communication. This structure is
 * essential for managing the low-level operations of the display,
 * allowing for command and data transmission, as well as hardware
 * resets.
 *
 * @param dc_pin Pointer to a GPIO descriptor for the Data/Command pin.
 * @param reset_pin Pointer to a GPIO descriptor for the RESET pin.
 * @param spi_desc Pointer to an SPI descriptor for SPI communication.
 ******************************************************************************/
struct nhd_c12832a1z_dev {
	/** Data/Command pin gpio desc */
	struct no_os_gpio_desc		*dc_pin;
	/** RESET pin gpio desc*/
	struct no_os_gpio_desc     	*reset_pin;
	/* SPI descriptor*/
	struct no_os_spi_desc		*spi_desc;
};

/***************************************************************************//**
 * @brief The `nhd_c12832a1z_init_param` structure is used to define the
 * initialization parameters for the nhd_c12832a1z device, which is a
 * type of display. It includes pointers to the initial configuration
 * parameters for the GPIO pins used for Data/Command and RESET
 * operations, as well as the SPI interface, which are essential for
 * setting up the communication and control of the display hardware.
 *
 * @param dc_pin_ip Pointer to the initial parameters for the Data/Command GPIO
 * pin.
 * @param reset_pin_ip Pointer to the initial parameters for the RESET GPIO pin.
 * @param spi_ip Pointer to the initial parameters for the SPI interface.
 ******************************************************************************/
struct nhd_c12832a1z_init_param {
	/** Data/Command pin gpio initial param */
	struct no_os_gpio_init_param	*dc_pin_ip;
	/** RESET pin gpio initial param */
	struct no_os_gpio_init_param	*reset_pin_ip;
	/* SPI initial param */
	struct no_os_spi_init_param 	*spi_ip;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* nhd_c12832a1z write command */
/***************************************************************************//**
 * @brief Use this function to send a command to the NHD-C12832A1Z display. It
 * must be called with a valid device structure that has been properly
 * initialized, including valid SPI and GPIO descriptors. The function
 * sets the display to command mode before sending the command over SPI.
 * It is essential to ensure that the device structure is not null and
 * that the SPI and GPIO descriptors are correctly configured before
 * calling this function. If the device is not properly initialized, the
 * function will return an error code.
 *
 * @param dev A pointer to an nhd_c12832a1z_dev structure representing the
 * display device. This must be a valid, non-null pointer with
 * initialized SPI and GPIO descriptors. If the SPI or GPIO
 * descriptors are null, the function returns -EINVAL.
 * @param cmd An 8-bit unsigned integer representing the command to be sent to
 * the display. The command should be a valid command for the
 * NHD-C12832A1Z display.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as -EINVAL for invalid input parameters.
 ******************************************************************************/
int nhd_c12832a1z_write_cmd(struct nhd_c12832a1z_dev *dev, uint8_t cmd);

/* nhd_c12832a1z write data */
/***************************************************************************//**
 * @brief Use this function to send a single byte of data to the NHD-C12832A1Z
 * display. It is essential to ensure that the device structure is
 * properly initialized and that both the SPI descriptor and the
 * data/command GPIO pin are set up before calling this function. This
 * function should be called when you need to update the display with new
 * data. If the device is not correctly initialized, or if the SPI or
 * GPIO operations fail, the function will return an error code.
 *
 * @param dev A pointer to an initialized nhd_c12832a1z_dev structure. This
 * structure must have valid spi_desc and dc_pin members. The caller
 * retains ownership and must ensure it is not null.
 * @param data A uint8_t value representing the data byte to be written to the
 * display. There are no specific constraints on the value of this
 * byte.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as -EINVAL if the device is not properly initialized.
 ******************************************************************************/
int nhd_c12832a1z_write_data(struct nhd_c12832a1z_dev *dev, uint8_t data);

/* nhd_c12832a1z print string on LCD */
/***************************************************************************//**
 * @brief Use this function to display a string on the NHD-C12832A1Z LCD screen.
 * It must be called with a valid device structure that has been properly
 * initialized. The function will attempt to print the provided string
 * starting from the current cursor position. If the string is too long
 * to fit on the display, it will be truncated to fit within the
 * available space. The function handles the necessary commands to update
 * the display and returns an error code if any command fails.
 *
 * @param dev A pointer to an initialized nhd_c12832a1z_dev structure
 * representing the LCD device. Must not be null.
 * @param msg A pointer to a null-terminated string to be displayed on the LCD.
 * The string should not exceed the display's width; otherwise, it
 * will be truncated. Must not be null.
 * @return Returns 0 on success or a negative error code if a command fails
 * during the display update process.
 ******************************************************************************/
int nhd_c12832a1z_print_string(struct nhd_c12832a1z_dev *dev, char *msg);

/* nhd_c12832a1z clear LCD */
/***************************************************************************//**
 * @brief Use this function to clear all content from the NHD-C12832A1Z LCD
 * display, effectively setting all pixels to off. This function should
 * be called when you need to reset the display to a blank state. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function communicates with the
 * display hardware to turn off the display, clear each page and column,
 * and then turn the display back on. If any command fails during this
 * process, the function will return an error code.
 *
 * @param dev A pointer to an initialized nhd_c12832a1z_dev structure
 * representing the LCD device. Must not be null. The caller retains
 * ownership of this structure. If the pointer is invalid or the
 * device is not properly initialized, the function will return an
 * error.
 * @return Returns 0 on success. If an error occurs during communication with
 * the display, a non-zero error code is returned.
 ******************************************************************************/
int nhd_c12832a1z_clear_lcd(struct nhd_c12832a1z_dev *dev);

/***************************************************************************//**
 * @brief This function sets up the nhd_c12832a1z device for use by initializing
 * the necessary SPI and GPIO interfaces based on the provided
 * initialization parameters. It must be called before any other
 * operations on the display. The function allocates memory for the
 * device structure and configures the display's initial state. If any
 * initialization step fails, the function will clean up any resources
 * that were successfully allocated before the failure and return an
 * error code. The caller is responsible for managing the memory of the
 * device structure and should call the corresponding remove function to
 * free resources when the device is no longer needed.
 *
 * @param device A pointer to a pointer of nhd_c12832a1z_dev structure. This
 * will be allocated and initialized by the function. Must not be
 * null.
 * @param init_param A structure containing initialization parameters for the
 * SPI and GPIO interfaces. Must be properly initialized
 * before calling this function.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * which step of the initialization failed.
 ******************************************************************************/
int nhd_c12832a1z_init(struct nhd_c12832a1z_dev **device,
		       struct nhd_c12832a1z_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * nhd_c12832a1z device when it is no longer needed. This function should
 * be called to clean up after the device has been initialized and used,
 * ensuring that all allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks. The
 * function will return an error code if any of the resource removals
 * fail, allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an nhd_c12832a1z_dev structure representing the
 * device to be removed. This pointer must not be null, and the
 * device must have been previously initialized. The function will
 * attempt to free the resources associated with this device.
 * @return Returns 0 on success, or a negative error code if any resource
 * removal fails.
 ******************************************************************************/
int nhd_c12832a1z_remove(struct nhd_c12832a1z_dev *dev);

#endif //__NHD_C12832A1Z_H__

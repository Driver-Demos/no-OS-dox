/***************************************************************************//**
 *   @file   ADP5589.h
 *   @brief  Header file of ADP5589 Driver.
 *   @author Mihai Bancisor (Mihai.Bancisor@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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
#ifndef __ADP5589_H__
#define __ADP5589_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/************************** ADP5589 Definitions *******************************/
/******************************************************************************/

#define PMOD_IOXP_J1		0	// J1 port of PmodIOXP
#define PMOD_IOXP_J2		1	// J2 port of PmodIOXP
#define ADP5589_ADDRESS		0x34	// I2C ADDRESS
#define ADP5589_ID		0x10	// Manufacturer ID

/* Register address definitions */
#define ADP5589_ADR_ID               	0x00
#define ADP5589_ADR_INT_STATUS       	0x01
#define ADP5589_ADR_STATUS           	0x02
#define ADP5589_ADR_FIFO1            	0x03
#define ADP5589_ADR_FIFO2            	0x04
#define ADP5589_ADR_FIFO3            	0x05
#define ADP5589_ADR_FIFO4            	0x06
#define ADP5589_ADR_FIFO5            	0x07
#define ADP5589_ADR_FIFO6            	0x08
#define ADP5589_ADR_FIFO7            	0x09
#define ADP5589_ADR_FIFO8            	0x0A
#define ADP5589_ADR_FIFO9            	0x0B
#define ADP5589_ADR_FIFO10           	0x0C
#define ADP5589_ADR_FIFO11           	0x0D
#define ADP5589_ADR_FIFO12           	0x0E
#define ADP5589_ADR_FIFO13           	0x0F
#define ADP5589_ADR_FIFO14           	0x10
#define ADP5589_ADR_FIFO15           	0x11
#define ADP5589_ADR_FIFO16           	0x12
#define ADP5589_ADR_GPI_INT_STATUS_A   	0x13
#define ADP5589_ADR_GPI_INT_STATUS_B   	0x14
#define ADP5589_ADR_GPI_INT_STATUS_C   	0x15
#define ADP5589_ADR_GPI_STATUS_A       	0x16
#define ADP5589_ADR_GPI_STATUS_B       	0x17
#define ADP5589_ADR_GPI_STATUS_C       	0x18
#define ADP5589_ADR_RPULL_CONFIG_A   	0x19
#define ADP5589_ADR_RPULL_CONFIG_B   	0x1A
#define ADP5589_ADR_RPULL_CONFIG_C   	0x1B
#define ADP5589_ADR_RPULL_CONFIG_D   	0x1C
#define ADP5589_ADR_RPULL_CONFIG_E	0x1D
#define ADP5589_ADR_GPI_INT_LEVEL_A	0x1E
#define ADP5589_ADR_GPI_INT_LEVEL_B	0x1F
#define ADP5589_ADR_GPI_INT_LEVEL_C    	0x20
#define ADP5589_ADR_GPI_EVENT_EN_A     	0x21
#define ADP5589_ADR_GPI_EVENT_EN_B     	0x22
#define ADP5589_ADR_GPI_EVENT_EN_C     	0x23
#define ADP5589_ADR_GPI_INTERRUPT_EN_A 	0x24
#define ADP5589_ADR_GPI_INTERRUPT_EN_B 	0x25
#define ADP5589_ADR_GPI_INTERRUPT_EN_C 	0x26
#define ADP5589_ADR_DEBOUNCE_DIS_A     	0x27
#define ADP5589_ADR_DEBOUNCE_DIS_B     	0x28
#define ADP5589_ADR_DEBOUNCE_DIS_C     	0x29
#define ADP5589_ADR_GPO_DATA_OUT_A     	0x2A
#define ADP5589_ADR_GPO_DATA_OUT_B     	0x2B
#define ADP5589_ADR_GPO_DATA_OUT_C     	0x2C
#define ADP5589_ADR_GPO_OUT_MODE_A     	0x2D
#define ADP5589_ADR_GPO_OUT_MODE_B     	0x2E
#define ADP5589_ADR_GPO_OUT_MODE_C     	0x2F
#define ADP5589_ADR_GPIO_DIRECTION_A 	0x30
#define ADP5589_ADR_GPIO_DIRECTION_B   	0x31
#define ADP5589_ADR_GPIO_DIRECTION_C	0x32
#define ADP5589_ADR_UNLOCK1          	0x33
#define ADP5589_ADR_UNLOCK2          	0x34
#define ADP5589_ADR_EXT_LOCK_EVENT   	0x35
#define ADP5589_ADR_UNLOCK_TIMERS	0x36
#define ADP5589_ADR_LOCK_CFG          	0x37
#define ADP5589_ADR_RESET1_EVENT_A     	0x38
#define ADP5589_ADR_RESET1_EVENT_B     	0x39
#define ADP5589_ADR_RESET1_EVENT_C     	0x3A
#define ADP5589_ADR_RESET2_EVENT_A     	0x3B
#define ADP5589_ADR_RESET2_EVENT_B     	0x3C
#define ADP5589_ADR_RESET_CFG         	0x3D
#define ADP5589_ADR_PWM_OFFT_LOW       	0x3E
#define ADP5589_ADR_PWM_OFFT_HIGH      	0x3F
#define ADP5589_ADR_PWM_ONT_LOW        	0x40
#define ADP5589_ADR_PWM_ONT_HIGH      	0x41
#define ADP5589_ADR_PWM_CFG           	0x42
#define ADP5589_ADR_CLOCK_DIV_CFG      	0x43
#define ADP5589_ADR_LOGIC_1_CFG        	0x44
#define ADP5589_ADR_LOGIC_2_CFG        	0x45
#define ADP5589_ADR_LOGIC_FF_CFG       	0x46
#define ADP5589_ADR_LOGIC_INT_EVENT    	0x47
#define ADP5589_ADR_POLL_TIME_CFG      	0x48
#define ADP5589_ADR_PIN_CONFIG_A       	0x49
#define ADP5589_ADR_PIN_CONFIG_B       	0x4A
#define ADP5589_ADR_PIN_CONFIG_C       	0x4B
#define ADP5589_ADR_PIN_CONFIG_D       	0x4C
#define ADP5589_ADR_GENERAL_CFG_B      	0x4D
#define ADP5589_ADR_INT_EN            	0x4E


/* Register Bit Mask Definitions. */
/* ID Register bits 0x00. */
#define ADP5589_ID_MAN_ID  			(0xF0)
#define ADP5589_ID_REV_ID  			(0x0F)
/* INT_STATUS Register bits 0x01. */
#define ADP5589_INT_STATUS_EVENT_INT   		(1 << 0)
#define ADP5589_INT_STATUS_GPI_INT     		(1 << 1)
#define ADP5589_INT_STATUS_OVERFLOW_INT 	(1 << 2)
#define ADP5589_INT_STATUS_LOCK_INT   		(1 << 3)
#define ADP5589_INT_STATUS_LOGIC1_INT  	 	(1 << 4)
#define ADP5589_INT_STATUS_LOGIC2_INT  		(1 << 5)
/* STATUS Register bits 0x02. */
#define ADP5589_STATUS_EC(x)			(((x) & 0x1F) << 0)
#define ADP5589_STATUS_LOCK_STAT  	 	(1 << 5)
#define ADP5589_STATUS_LOGIC1_STAT  	  	(1 << 6)
#define ADP5589_STATUS_LOGIC2_STAT  	 	(1 << 7)
/* INT_EN Register bits 0x4E. */
#define ADP5589_INT_EN_EVENT_IEN   		(1 << 0)
#define ADP5589_INT_EN_GPI_IEN     		(1 << 1)
#define ADP5589_INT_EN_OVERFLOW_IEN 		(1 << 2)
#define ADP5589_INT_EN_LOCK_IEN   		(1 << 3)
#define ADP5589_INT_EN_LOGIC1_INT  		(1 << 4)
#define ADP5589_INT_EN_LOGIC2_INT  		(1 << 5)
/* GENERAL_CFG_B Register bits  0x4D. */
#define ADP5589_GENERAL_CFG_B_RST_CFG   	(1 << 0)
#define ADP5589_GENERAL_CFG_B_INT_CFG   	(1 << 1)
#define ADP5589_GENERAL_CFG_B_LCK_TRK_GPI   	(1 << 3)
#define ADP5589_GENERAL_CFG_B_LCK_TRK_LOGIC  	(1 << 4)
#define ADP5589_GENERAL_CFG_B_CORE_FREQ(x)  	(((x) & 0x03) << 5)
#define ADP5589_GENERAL_CFG_B_OSC_EN  		(1 << 7)
/* PIN_CONFIG_D Register bits 0x4C. */
#define ADP5589_PIN_CONFIG_D_R0_EXTEND  	(1 << 0)
#define ADP5589_PIN_CONFIG_D_C9_EXTEND  	(1 << 1)
#define ADP5589_PIN_CONFIG_D_R3_EXTEND(x)	(((x) & 0x03) << 2)
#define ADP5589_PIN_CONFIG_D_C6_EXTEND	  	(1 << 4)
#define ADP5589_PIN_CONFIG_D_R4_EXTEND	  	(1 << 5)
#define ADP5589_PIN_CONFIG_D_C4_EXTEND	  	(1 << 6)
#define ADP5589_PIN_CONFIG_D_PULL_SELECT  	(1 << 7)
/* GPI_STATUS_A Register bits 0x16. */
#define ADP5589_GPI_STATUS_GPI_1_STAT 		(1 << 0)
#define ADP5589_GPI_STATUS_GPI_2_STAT 		(1 << 1)
#define ADP5589_GPI_STATUS_GPI_3_STAT 		(1 << 2)
#define ADP5589_GPI_STATUS_GPI_4_STAT 		(1 << 3)
#define ADP5589_GPI_STATUS_GPI_5_STAT 		(1 << 4)
#define ADP5589_GPI_STATUS_GPI_6_STAT 		(1 << 5)
#define ADP5589_GPI_STATUS_GPI_7_STAT 		(1 << 6)
#define ADP5589_GPI_STATUS_GPI_8_STAT 		(1 << 7)
/* GPI_STATUS_B Register bits 0x17. */
#define ADP5589_GPI_STATUS_GPI_9_STAT  		(1 << 0)
#define ADP5589_GPI_STATUS_GPI_10_STAT  	(1 << 1)
#define ADP5589_GPI_STATUS_GPI_11_STAT  	(1 << 2)
#define ADP5589_GPI_STATUS_GPI_12_STAT  	(1 << 3)
#define ADP5589_GPI_STATUS_GPI_13_STAT  	(1 << 4)
#define ADP5589_GPI_STATUS_GPI_14_STAT  	(1 << 5)
#define ADP5589_GPI_STATUS_GPI_15_STAT  	(1 << 6)
#define ADP5589_GPI_STATUS_GPI_16_STAT  	(1 << 7)
/* GPI_STATUS_C Register bits 0x18. */
#define ADP5589_GPI_STATUS_GPI_17_STAT  	(1 << 0)
#define ADP5589_GPI_STATUS_GPI_18_STAT  	(1 << 1)
#define ADP5589_GPI_STATUS_GPI_19_STAT  	(1 << 2)
/* GPI_EVENT_EN_A Register bits 0x21. */
#define ADP5589_GPI_EVENT_EN_GPI_1_STAT  	(1 << 0)
#define ADP5589_GPI_EVENT_EN_GPI_2_STAT  	(1 << 1)
#define ADP5589_GPI_EVENT_EN_GPI_3_STAT  	(1 << 2)
#define ADP5589_GPI_EVENT_EN_GPI_4_STAT  	(1 << 3)
#define ADP5589_GPI_EVENT_EN_GPI_5_STAT  	(1 << 4)
#define ADP5589_GPI_EVENT_EN_GPI_6_STAT  	(1 << 5)
#define ADP5589_GPI_EVENT_EN_GPI_7_STAT  	(1 << 6)
#define ADP5589_GPI_EVENT_EN_GPI_8_STAT  	(1 << 7)
/* GPI_EVENT_EN_B Register bits 0x22. */
#define ADP5589_GPI_EVENT_EN_GPI_9_STAT  	(1 << 0)
#define ADP5589_GPI_EVENT_EN_GPI_10_STAT  	(1 << 1)
#define ADP5589_GPI_EVENT_EN_GPI_11_STAT  	(1 << 2)
#define ADP5589_GPI_EVENT_EN_GPI_12_STAT  	(1 << 3)
#define ADP5589_GPI_EVENT_EN_GPI_13_STAT  	(1 << 4)
#define ADP5589_GPI_EVENT_EN_GPI_14_STAT  	(1 << 5)
#define ADP5589_GPI_EVENT_EN_GPI_15_STAT  	(1 << 6)
#define ADP5589_GPI_EVENT_EN_GPI_16_STAT  	(1 << 7)
/* GPI_EVENT_EN_C Register bits 0x23. */
#define ADP5589_GPI_EVENT_EN_GPI_17_STAT  	(1 << 0)
#define ADP5589_GPI_EVENT_EN_GPI_18_STAT  	(1 << 1)
#define ADP5589_GPI_EVENT_EN_GPI_19_STAT  	(1 << 2)

/* UNLOCK1 Register bits 0x33. */
#define ADP5589_UNLOCK1_UNLOCK1_STATE	  	(1 << 7)
#define ADP5589_UNLOCK1_UNLOCK1_UNLOCK1(x) 	(((x) & 0x7F) << 0)
/* UNLOCK2 Register bits 0x34. */
#define ADP5589_UNLOCK2_UNLOCK2_STATE	  	(1 << 7)
#define ADP5589_UNLOCK2_UNLOCK2_UNLOCK2(x) 	(((x) & 0x7F) << 0)
/* EXT_LOCK_EVENT Register bits 0x35. */
#define ADP5589_EXT_LOCK_EXT_LOCK_STATE		(1 << 7)
#define ADP5589_EXT_LOCK_EXT_LOCK_EVENT(x) 	(((x) & 0x7F) << 0)
/* UNLOCK_TIMERS Register bits 0x36. */
#define ADP5589_UNLOCK_TIMERS_INT_MASK_TIMER(x)	(((x) & 0xF8) << 3)
#define ADP5589_UNLOCK_TIMERS_UNLOCK_TIMER(x)  	(((x) & 0x07) << 0)
/* UNLOCK_TIMER bits. */
#define ADP5589_UNLOCK_TIMER_DIS	        0
#define ADP5589_UNLOCK_TIMER_1SEC	        1
#define ADP5589_UNLOCK_TIMER_2SEC	        2
#define ADP5589_UNLOCK_TIMER_3SEC	        3
#define ADP5589_UNLOCK_TIMER_4SEC	        4
#define ADP5589_UNLOCK_TIMER_5SEC	        5
#define ADP5589_UNLOCK_TIMER_6SEC	        6
#define ADP5589_UNLOCK_TIMER_7SEC	        7
/* INT_MASK_TIMER bits. */
#define ADP5589_INT_MASTER_TIMER_DIS	        0
#define ADP5589_INT_MASTER_TIMER_1SEC	        1
#define ADP5589_INT_MASTER_TIMER_2SEC	        2
#define ADP5589_INT_MASTER_TIMER_30SEC	        (0X1E)
#define ADP5589_INT_MASTER_TIMER_31SEC	        (0X1F)
/* LOCK_CFG Register bits 0x37. */
#define ADP5589_LOCK_CFG_LOCK_EN		(1 << 0)
/* RESET_CFG Register bits 0x3D. */
#define ADP5589_RESET_CFG_RESET_PULSE_WIDTH(x)	(((x) & 0x03) << 0)
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME(x)	(((x) & 0x07) << 2)
#define ADP5589_RESET_CFG_RST_PASSTHRU_EN	(1 << 5)
#define ADP5589_RESET_CFG_RESET1_POL		(1 << 6)
#define ADP5589_RESET_CFG_RESET2_POL		(1 << 7)
/* RESET_TRIGGER_TIME bits. */
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_IMMED	0
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_1D0SEC	1
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_1D5SEC	2
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_2D0SEC	3
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_2D5SEC	4
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_3D0SEC	5
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_3D5SEC	6
#define ADP5589_RESET_CFG_RESET_TRIGGER_TIME_4D0SEC	7
/* RESET_PULSE_WIDTH bits. */
#define ADP5589_RESET_CFG_RESET_PULSE_WIDTH_500US	0
#define ADP5589_RESET_CFG_RESET_PULSE_WIDTH_1MS		1
#define ADP5589_RESET_CFG_RESET_PULSE_WIDTH_2MS		2
#define ADP5589_RESET_CFG_RESET_PULSE_WIDTH_10MS	3
/* PWM_CFG Register bits 0x42. */
#define ADP5589_PWM_CFG_PWM_EN				(1 << 0)
#define ADP5589_PWM_CFG_PWM_MODE			(1 << 1)
#define ADP5589_PWM_CFG_PWM_IN_AND			(1 << 2)
/* CLOCK_DIV_CFG Register bits 0x43. */
#define ADP5589_CLOCK_DIV_CFG_CLK_INV			(1 << 6)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV(x)		(((x) & 0x1F) << 1)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_EN		(1 << 0)
/* CLK_DIV bits. */
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_DIV1		(0X00)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_DIV2		(0X01)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_DIV3		(0X02)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_DIV4		(0X03)
#define ADP5589_CLOCK_DIV_CFG_CLK_DIV_DIV32		(0X1F)
/* LOGIC_1_CFG Register bits 0x44. */
#define ADP5589_LOGIC_1_CFG_LOGIC1_SEL(x)		(((x) & 0x07) << 0)
#define ADP5589_LOGIC_1_LA1_INV				(1 << 3)
#define ADP5589_LOGIC_1_LB1_INV				(1 << 4)
#define ADP5589_LOGIC_1_LC1_INV				(1 << 5)
#define ADP5589_LOGIC_1_LY1_INV				(1 << 6)
/* LOGIC1_SEL bits. */
#define ADP5589_LOGIC_CFG_LOGIC_SEL_OFF			(0x00)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_AND			(0x01)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_OR			(0x02)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_XOR			(0x03)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_FF			(0x04)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_IN_LA		(0x05)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_IN_LB		(0x06)
#define ADP5589_LOGIC_CFG_LOGIC_SEL_IN_LC		(0x07)
/* LOGIC_2_CFG Register bits 0x45. */
#define ADP5589_LOGIC_2_CFG_LOGIC2_SEL(x)		(((x) & 0x07) << 0)
#define ADP5589_LOGIC_2_LA2_INV				(1 << 3)
#define ADP5589_LOGIC_2_LB2_INV				(1 << 4)
#define ADP5589_LOGIC_2_LC2_INV				(1 << 5)
#define ADP5589_LOGIC_2_LY2_INV				(1 << 6)
#define ADP5589_LOGIC_2_LY1_CASCADE			(1 << 7)
/* LOGIC_FF_CFG Register bits 0x46. */
#define ADP5589_LOGIC_FF_CFG_FF1_CLR			(1 << 0)
#define ADP5589_LOGIC_FF_CFG_FF1_SET			(1 << 1)
#define ADP5589_LOGIC_FF_CFG_FF2_CLR			(1 << 2)
#define ADP5589_LOGIC_FF_CFG_FF2_SET			(1 << 3)
/* LOGIC_INT_EVENT_EN Register bits 0x47. */
#define ADP5589_LOGIC_INT_EVENT_EN_LOGIC1_INT_LEVEL	(1 << 0)
#define ADP5589_LOGIC_INT_EVENT_EN_LOGIC1_EVENT_EN	(1 << 1)
#define ADP5589_LOGIC_INT_EVENT_EN_LY1_DBNC_DIS		(1 << 2)
#define ADP5589_LOGIC_INT_EVENT_EN_LOGIC2_INT_LEVEL	(1 << 3)
#define ADP5589_LOGIC_INT_EVENT_EN_LOGIC2_EVENT_EN	(1 << 4)
#define ADP5589_LOGIC_INT_EVENT_EN_LY2_DBNC_DIS		(1 << 5)
/* POLL_TIME_CFG Register bits 0x48. */
#define ADP5589_POLL_TIME_CFG_KEY_POLL_TIME(x)		(((x) & 0x03) << 0)
/* KEY_POLL_TIME bits. */
#define ADP5589_POLL_TIME_CFG_KEY_POLL_TIME_10MS	(0x00)
#define ADP5589_POLL_TIME_CFG_KEY_POLL_TIME_20MS	(0x01)
#define ADP5589_POLL_TIME_CFG_KEY_POLL_TIME_30MS	(0x02)
#define ADP5589_POLL_TIME_CFG_KEY_POLL_TIME_40MS	(0x03)

#define ADP5589_EVENT_KEY_RELEASED                      0
#define ADP5589_EVENT_KEY_PRESSED                       1

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adp5589_dev` structure is a simple data structure used to
 * represent a device instance for the ADP5589 driver, specifically
 * managing the I2C communication interface. It contains a single member,
 * `i2c_desc`, which is a pointer to a `no_os_i2c_desc` structure,
 * encapsulating the details necessary for I2C communication with the
 * ADP5589 device.
 *
 * @param i2c_desc Pointer to a structure describing the I2C interface.
 ******************************************************************************/
struct adp5589_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
};

/***************************************************************************//**
 * @brief The `adp5589_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the ADP5589 device,
 * specifically focusing on the I2C communication interface. It contains
 * a single member, `i2c_init`, which is a structure that holds the
 * necessary parameters for initializing the I2C interface, ensuring that
 * the device can communicate effectively over the I2C bus.
 *
 * @param i2c_init This member is a structure of type no_os_i2c_init_param, used
 * to initialize I2C communication parameters.
 ******************************************************************************/
struct adp5589_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
};


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to set a specific register on the ADP5589 device to
 * a desired value. This function is typically called when configuring
 * the device or updating its settings. Ensure that the device has been
 * properly initialized before calling this function. The function
 * communicates with the device over I2C, so the I2C interface must be
 * correctly set up and operational. It does not return any value, and
 * any errors in communication are not reported through this function.
 *
 * @param dev A pointer to an adp5589_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param register_address The address of the register to be written to. It is
 * an 8-bit unsigned integer, and valid addresses are
 * defined by the device's register map.
 * @param register_value The value to write to the specified register. It is an
 * 8-bit unsigned integer.
 * @return None
 ******************************************************************************/
void adp5589_set_register_value(struct adp5589_dev *dev,
				uint8_t register_address,
				uint8_t register_value);

/***************************************************************************//**
 * @brief This function retrieves the value stored in a specific register of the
 * ADP5589 device using I2C communication. It is typically used when you
 * need to read the current configuration or status from the device.
 * Ensure that the device has been properly initialized before calling
 * this function. The function requires a valid device descriptor and a
 * register address to operate correctly. If the register address is
 * invalid, the behavior is undefined.
 *
 * @param dev A pointer to an adp5589_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param register_address The address of the register to read from. Must be a
 * valid register address as defined by the ADP5589
 * device specifications.
 * @return Returns the 8-bit value read from the specified register.
 ******************************************************************************/
uint8_t adp5589_get_register_value(struct adp5589_dev *dev,
				   uint8_t register_address);

/***************************************************************************//**
 * @brief This function sets up the ADP5589 device by initializing the I2C
 * communication interface and verifying the presence of the device. It
 * should be called before any other operations on the ADP5589 to ensure
 * the device is properly configured and ready for use. The function
 * allocates memory for the device structure and configures the internal
 * oscillator and clock frequency. It is important to check the return
 * value to ensure successful initialization, as a negative return value
 * indicates an error, such as memory allocation failure or device not
 * being detected.
 *
 * @param device A pointer to a pointer of type `struct adp5589_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct adp5589_init_param` containing
 * initialization parameters for the I2C interface. This must
 * be properly configured before calling the function.
 * @return Returns 0 on success, or a negative value if an error occurs, such as
 * memory allocation failure or device detection failure.
 ******************************************************************************/
int8_t adp5589_init(struct adp5589_dev **device,
		    struct adp5589_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release all resources allocated for an
 * ADP5589 device, typically after the device is no longer needed. It
 * should be called to clean up after a successful initialization with
 * `adp5589_init`. The function ensures that the I2C descriptor
 * associated with the device is properly removed and the memory
 * allocated for the device structure is freed. This function must be
 * called to prevent memory leaks in applications that use the ADP5589
 * driver.
 *
 * @param dev A pointer to an `adp5589_dev` structure representing the device to
 * be removed. This pointer must not be null, and it must point to a
 * valid device structure that was previously initialized. If the
 * pointer is invalid, the behavior is undefined.
 * @return Returns an `int32_t` indicating the result of the I2C removal
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the I2C removal
 * process.
 ******************************************************************************/
int32_t adp5589_remove(struct adp5589_dev *dev);

/***************************************************************************//**
 * @brief This function is used to configure the ADP5589 device to enable its
 * PWM generator in continuous mode. It should be called after the device
 * has been properly initialized using the appropriate initialization
 * function. This function sets specific register values to enable the
 * PWM functionality, ensuring that the device is ready to generate PWM
 * signals. It is important to ensure that the device structure provided
 * is valid and properly initialized before calling this function.
 *
 * @param dev A pointer to an adp5589_dev structure representing the device.
 * This must be a valid, non-null pointer to a device that has been
 * initialized. The caller retains ownership of this structure.
 * @return None
 ******************************************************************************/
void adp5589_init_pwm(struct adp5589_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure the PWM (Pulse Width Modulation) on and
 * off times for an ADP5589 device. This function should be called after
 * the device has been properly initialized and the PWM generator has
 * been set up. It allows you to specify the duration for which the PWM
 * signal is high (on time) and low (off time). Ensure that the device
 * pointer is valid and that the PWM times are within the acceptable
 * range for your application.
 *
 * @param dev A pointer to an initialized adp5589_dev structure representing the
 * device. Must not be null.
 * @param pwm_off_time The duration for which the PWM signal is low, specified
 * as a 16-bit unsigned integer. The valid range depends on
 * the specific application requirements.
 * @param pwm_on_time The duration for which the PWM signal is high, specified
 * as a 16-bit unsigned integer. The valid range depends on
 * the specific application requirements.
 * @return None
 ******************************************************************************/
void adp5589_set_pwm(struct adp5589_dev *dev,
		     uint16_t pwm_off_time,
		     uint16_t pwm_on_time);

/***************************************************************************//**
 * @brief This function configures the direction of GPIO pins on the ADP5589
 * device by writing a specified value to a given register. It is
 * typically used to set pins as either inputs or outputs, depending on
 * the application requirements. The function must be called with a valid
 * device structure that has been properly initialized. It is important
 * to ensure that the register address and value provided are appropriate
 * for the desired configuration.
 *
 * @param dev A pointer to an initialized adp5589_dev structure representing the
 * device. Must not be null.
 * @param reg The register address where the direction configuration will be
 * written. Must be a valid register address for GPIO direction.
 * @param val The value to be written to the specified register, determining the
 * direction of the GPIO pins. Valid values depend on the specific
 * register and desired configuration.
 * @return None
 ******************************************************************************/
void adp5589_gpio_direction(struct adp5589_dev *dev,
			    uint8_t reg,
			    uint8_t val);

/***************************************************************************//**
 * @brief Use this function to retrieve the current state of a pin or group of
 * pins from the ADP5589 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function reads the value from the specified register and returns it,
 * allowing the caller to determine the pin states. This function is
 * useful for monitoring pin status in applications where pin state
 * changes need to be detected or logged.
 *
 * @param dev A pointer to an initialized adp5589_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before use.
 * @param reg The register address from which to read the pin state. Must be a
 * valid register address as defined by the ADP5589 device
 * specifications.
 * @return Returns an 8-bit unsigned integer representing the state of the pins
 * in the specified register.
 ******************************************************************************/
uint8_t adp5589_get_pin_state(struct adp5589_dev *dev,
			      uint8_t reg);

/***************************************************************************//**
 * @brief Use this function to change the state of a specific pin on the ADP5589
 * device. This function is typically called when you need to control the
 * output state of a pin, such as setting it high or low. Ensure that the
 * device has been properly initialized before calling this function. The
 * function does not perform any validation on the input parameters, so
 * it is the caller's responsibility to ensure that the provided register
 * address and state are valid for the intended operation.
 *
 * @param dev A pointer to an initialized adp5589_dev structure representing the
 * device. Must not be null.
 * @param reg The register address of the pin whose state is to be set. It
 * should be a valid register address for the ADP5589 device.
 * @param state The desired state to set for the specified pin. Typically, this
 * would be a value representing high or low state.
 * @return None
 ******************************************************************************/
void adp5589_set_pin_state(struct adp5589_dev *dev,
			   uint8_t reg,
			   uint8_t state);

/***************************************************************************//**
 * @brief This function configures the ADP5589 device to initialize the keyboard
 * decoder for a specified Pmod port. It should be called after the
 * device has been properly initialized and is ready for configuration.
 * The function sets up the row and column configurations based on the
 * provided Pmod port, either configuring rows and columns 0-3 or 4-7.
 * This setup is necessary for the device to correctly interpret key
 * presses from the connected keypad.
 *
 * @param dev A pointer to an adp5589_dev structure representing the device.
 * Must not be null, and the device should be initialized before
 * calling this function.
 * @param pmod_port A uint8_t value indicating the Pmod port to configure. Valid
 * values are 0 or 1, corresponding to different row and column
 * configurations. If an invalid value is provided, the
 * behavior is undefined.
 * @return None
 ******************************************************************************/
void adp5589_init_key(struct adp5589_dev *dev,
		      uint8_t pmod_port);

/***************************************************************************//**
 * @brief This function is used to decode a key event from the Pmod-KYPD based
 * on the provided register value, event type, and Pmod port. It is
 * typically called when a key event is detected to determine which key
 * was pressed or released. The function requires the register value to
 * be adjusted according to the Pmod port and event type before decoding.
 * It returns a character representing the key, or 'x' if the key cannot
 * be decoded.
 *
 * @param reg The register value associated with the key event. It must be
 * adjusted according to the Pmod port and event type before
 * decoding.
 * @param event_type Specifies the type of key event, such as key pressed or
 * released. Valid values are ADP5589_EVENT_KEY_PRESSED and
 * ADP5589_EVENT_KEY_RELEASED.
 * @param pmod_port Indicates the Pmod port being used, either PMOD_IOXP_J1 or
 * PMOD_IOXP_J2. This affects the adjustment of the register
 * value.
 * @return Returns a uint8_t character representing the decoded key, or 'x' if
 * the key cannot be decoded.
 ******************************************************************************/
uint8_t adp5589_key_decode(uint8_t reg,
			   uint8_t event_type,
			   uint8_t pmod_port);

/***************************************************************************//**
 * @brief This function is used to lock the ADP5589 device, requiring a password
 * to unlock it. It should be called when you want to secure the device
 * from unauthorized access. The function configures the device to
 * require a specific sequence of events as a password for unlocking. It
 * must be called with valid event codes and a valid Pmod port
 * identifier. The function will block until the lock status is
 * confirmed.
 *
 * @param dev A pointer to an initialized adp5589_dev structure. Must not be
 * null. The caller retains ownership.
 * @param first_event An 8-bit unsigned integer representing the first event in
 * the unlock sequence. Must be a valid event code.
 * @param second_event An 8-bit unsigned integer representing the second event
 * in the unlock sequence. Must be a valid event code.
 * @param pmod_port An 8-bit unsigned integer indicating the Pmod port (e.g.,
 * PMOD_IOXP_J1 or PMOD_IOXP_J2). Must be a valid port
 * identifier.
 * @return None
 ******************************************************************************/
void adp5589_key_lock(struct adp5589_dev *dev,
		      uint8_t first_event,
		      uint8_t second_event,
		      uint8_t pmod_port);

#endif	/* __ADP5589_H__ */

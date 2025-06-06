#ifndef _ADI_ADRV9001_COMMON_TYPES_H_
#define _ADI_ADRV9001_COMMON_TYPES_H_

/***************************************************************************//**
 * @brief The `adi_common_ChannelNumber_e` is an enumeration that defines
 * channel numbers for a system, where each channel is represented by a
 * unique hexadecimal value. These values are designed to be maskable,
 * allowing for the combination of multiple channels using bitwise
 * operations. This feature is useful in scenarios where multiple
 * channels need to be specified simultaneously.
 *
 * @param ADI_CHANNEL_1 Represents channel 1 with a maskable value of 0x1.
 * @param ADI_CHANNEL_2 Represents channel 2 with a maskable value of 0x2.
 ******************************************************************************/
typedef enum adi_common_ChannelNumber
{
    ADI_CHANNEL_1 = 0x1, /* In hex to indicate maskable value */
    ADI_CHANNEL_2 = 0x2
} adi_common_ChannelNumber_e;

/***************************************************************************//**
 * @brief The `adi_common_Port_e` is an enumeration that defines various port
 * types used in a system, each associated with a unique non-maskable
 * integer value. This enumeration is used to specify different ports
 * such as receive, transmit, observation receive, internal loopback, and
 * external loopback, and it is important to note that these values
 * cannot be combined using bitwise OR operations.
 *
 * @param ADI_RX Represents the receive port with a non-maskable value of 0.
 * @param ADI_TX Represents the transmit port with a non-maskable value of 1.
 * @param ADI_ORX Represents the observation receive port with a non-maskable
 * value of 2.
 * @param ADI_ILB Represents the internal loopback port with a non-maskable
 * value of 3.
 * @param ADI_ELB Represents the external loopback port with a non-maskable
 * value of 4.
 ******************************************************************************/
typedef enum adi_common_Port
{
    ADI_RX = 0, /* In decimal to indicate NON-maskable value */
    ADI_TX = 1,
    ADI_ORX = 2,
    ADI_ILB = 3,
    ADI_ELB = 4
} adi_common_Port_e;

#endif
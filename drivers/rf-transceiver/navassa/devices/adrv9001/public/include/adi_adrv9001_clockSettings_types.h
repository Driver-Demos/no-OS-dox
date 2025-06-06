/**
 * \file
 * \brief Type definitions for ADRV9001 clock settings
 * \copyright Analog Devices Inc. 2019. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_CLOCKSETTINGS_TYPES_H_
#define _ADI_ADRV9001_CLOCKSETTINGS_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

/***************************************************************************//**
 * @brief The `adi_adrv9001_HsDiv_e` is an enumeration that defines the possible
 * settings for the high-speed clock divider in the ADRV9001 device. It
 * allows the selection of different division factors (4, 6, or 8) for
 * the high-speed clock, which is crucial for configuring the clocking
 * architecture of the device to meet specific application requirements.
 *
 * @param ADI_ADRV9001_HSDIV_4 High speed clock divide by 4 setting.
 * @param ADI_ADRV9001_HSDIV_6 High speed clock divide by 6 setting.
 * @param ADI_ADRV9001_HSDIV_8 High speed clock divide by 8 setting.
 ******************************************************************************/
typedef enum adi_adrv9001_HsDiv
{
    ADI_ADRV9001_HSDIV_4 = 0x0, /*!< High speed clock divide by 4 setting */
    ADI_ADRV9001_HSDIV_6 = 0x1, /*!< High speed clock divide by 6 setting */
    ADI_ADRV9001_HSDIV_8 = 0x2  /*!< High speed clock divide by 8 setting */
} adi_adrv9001_HsDiv_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ClkPllMode_e` is an enumeration that defines the
 * available modes for the Clock Phase-Locked Loop (PLL) in the ADRV9001
 * device, specifically for driving the High Speed Digital Clock. It
 * provides two modes: High Performance (HP) and Low Power (LP), allowing
 * users to select the appropriate mode based on their performance and
 * power consumption requirements.
 *
 * @param ADI_ADRV9001_CLK_PLL_HP_MODE Clock PLL High Performance Mode.
 * @param ADI_ADRV9001_CLK_PLL_LP_MODE Clock PLL Low Power Mode.
 ******************************************************************************/
typedef enum adi_adrv9001_ClkPllMode
{
    ADI_ADRV9001_CLK_PLL_HP_MODE = 0x0, /*!< Clock PLL HP Mode */
    ADI_ADRV9001_CLK_PLL_LP_MODE = 0x1, /*!< Clock PLL LP Mode */
} adi_adrv9001_ClkPllMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ComponentPowerLevel_e` is an enumeration that
 * defines different power level settings for components in the ADRV9001
 * system. It provides three distinct power levels: low, medium, and
 * high, which can be used to configure the power consumption and
 * performance characteristics of various components within the system.
 *
 * @param ADI_ADRV9001_COMPONENT_POWER_LEVEL_LOW Represents a low power level
 * setting for a component.
 * @param ADI_ADRV9001_COMPONENT_POWER_LEVEL_MEDIUM Represents a medium power
 * level setting for a
 * component.
 * @param ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH Represents a high power level
 * setting for a component.
 ******************************************************************************/
typedef enum
{
    ADI_ADRV9001_COMPONENT_POWER_LEVEL_LOW = 0,  /*!< Low power */
    ADI_ADRV9001_COMPONENT_POWER_LEVEL_MEDIUM,   /*!< Medium power */
    ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH      /*!< High power */
} adi_adrv9001_ComponentPowerLevel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_InternalClock_Divisor_e` is an enumeration that
 * defines the possible settings for dividing the internal clock
 * frequency in the ADRV9001 device. Each enumerator represents a
 * specific division factor, ranging from 1 to 9, allowing the internal
 * clock to be divided by the specified value to achieve the desired
 * clock frequency for various operational needs.
 *
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_1 Clock divide by 1 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_2 Clock divide by 2 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_3 Clock divide by 3 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_4 Clock divide by 4 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_5 Clock divide by 5 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_6 Clock divide by 6 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_7 Clock divide by 7 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_8 Clock divide by 8 setting.
 * @param ADI_ADRV9001_INTERNAL_CLOCK_DIV_9 Clock divide by 9 setting.
 ******************************************************************************/
typedef enum adi_adrv9001_InternalClock_Divisor
{
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_1 = 0x1, /*!< Clock divide by 1 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_2 = 0x2, /*!< Clock divide by 2 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_3 = 0x3, /*!< Clock divide by 3 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_4 = 0x4, /*!< Clock divide by 4 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_5 = 0x5, /*!< Clock divide by 5 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_6 = 0x6, /*!< Clock divide by 6 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_7 = 0x7, /*!< Clock divide by 7 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_8 = 0x8, /*!< Clock divide by 8 setting */
    ADI_ADRV9001_INTERNAL_CLOCK_DIV_9 = 0x9, /*!< Clock divide by 9 setting */
} adi_adrv9001_InternalClock_Divisor_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllLoMode_e` is an enumeration that defines the
 * possible modes for setting the Local Oscillator (LO) source in the
 * ADRV9001 device. It includes options for using internal RFPLL LO
 * frequencies (LO1 and LO2) as outputs, as well as using external LO
 * sources either as outputs or inputs to generate the LO frequency. This
 * enumeration is crucial for configuring the LO path in RF applications,
 * allowing for flexibility in choosing between internal and external LO
 * sources.
 *
 * @param ADI_ADRV9001_INT_LO1 Internal RFPLL LO1 frequency will output from
 * ADRV9001.
 * @param ADI_ADRV9001_INT_LO2 Internal RFPLL LO2 frequency will output from
 * ADRV9001.
 * @param ADI_ADRV9001_EXT_LO_OUTPUT External RFPLL LO in ADRV9001 to generate
 * the LO frequency.
 * @param ADI_ADRV9001_EXT_LO_INPUT1 Provided external LO1 in ADRV9001 to
 * generate the LO frequency.
 * @param ADI_ADRV9001_EXT_LO_INPUT2 Provided external LO2 in ADRV9001 to
 * generate the LO frequency.
 ******************************************************************************/
typedef enum adi_adrv9001_PllLoMode
{
    ADI_ADRV9001_INT_LO1 = 0,       /*!< Internal RFPLL LO1 frequency will output from ADRV9001 */
    ADI_ADRV9001_INT_LO2 = 0,       /*!< Internal RFPLL LO2 frequency will output from ADRV9001 */
    ADI_ADRV9001_EXT_LO_OUTPUT = 1, /*!< External RFPLL LO in ADRV9001 to generate the LO frequency */
    ADI_ADRV9001_EXT_LO_INPUT1 = 2, /*!< Provided external LO1 in ADRV9001 to generate the LO frequency */
    ADI_ADRV9001_EXT_LO_INPUT2 = 3  /*!< Provided external LO2 in ADRV9001 to generate the LO frequency */
} adi_adrv9001_PllLoMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ExtLoType_e` is an enumeration that defines the
 * types of external local oscillators (LO) that can be used with the
 * ADRV9001 device. It provides two options: differential and single-
 * ended, allowing users to specify the configuration of the external LO
 * connection.
 *
 * @param ADI_ADRV9001_EXT_LO_DIFFERENTIAL Represents a differential external
 * local oscillator type with a value of
 * 0.
 * @param ADI_ADRV9001_EXT_LO_SINGLE_ENDED Represents a single-ended external
 * local oscillator type with a value of
 * 1.
 ******************************************************************************/
typedef enum adi_adrv9001_ExtLoType
{
    ADI_ADRV9001_EXT_LO_DIFFERENTIAL = 0,
    ADI_ADRV9001_EXT_LO_SINGLE_ENDED = 1

} adi_adrv9001_ExtLoType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxRfInputSel_e` is an enumeration that defines the
 * possible options for selecting the RF input path for the ADRV9001
 * device. It provides two options, `ADI_ADRV9001_RX_A` and
 * `ADI_ADRV9001_RX_B`, which correspond to different RF input paths that
 * can be used for receiving signals. This enumeration is used to
 * configure which RF input path is active in the device's operation.
 *
 * @param ADI_ADRV9001_RX_A Represents the selection of RX path A.
 * @param ADI_ADRV9001_RX_B Represents the selection of RX path B.
 ******************************************************************************/
typedef enum adi_adrv9001_RxRfInputSel
{
    ADI_ADRV9001_RX_A = 0,
    ADI_ADRV9001_RX_B = 1

} adi_adrv9001_RxRfInputSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RfPllMcs_e` is an enumeration that defines the modes
 * for RFPLL phase synchronization in the ADRV9001 device. It provides
 * options to either disable synchronization, enable it only during
 * initialization, or enable it both during initialization and
 * continuously track it. This allows for flexibility in managing the
 * phase synchronization of the RFPLL, which can be crucial for
 * applications requiring precise phase alignment.
 *
 * @param ADI_ADRV9001_RFPLLMCS_NOSYNC Disable RFPLL phase synchronization.
 * @param ADI_ADRV9001_RFPLLMCS_INIT_AND_SYNC Enable RFPLL phase sync init only.
 * @param ADI_ADRV9001_RFPLLMCS_INIT_AND_CONTTRACK Enable RFPLL phase sync init
 * and track continuously.
 ******************************************************************************/
typedef enum adi_adrv9001_RfPllMcs
{
    ADI_ADRV9001_RFPLLMCS_NOSYNC = 0,            /*!< Disable RFPLL phase synchronization */
    ADI_ADRV9001_RFPLLMCS_INIT_AND_SYNC = 1,     /*!< Enable RFPLL phase sync init only */
    ADI_ADRV9001_RFPLLMCS_INIT_AND_CONTTRACK = 2 /*!< Enable RFPLL phase sync init and track continuously */
} adi_adrv9001_RfPllMcs_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_LoSel_e` enumeration defines the possible local
 * oscillator (LO) sources that can be selected for use in the ADRV9001
 * device. It provides options to choose between two primary LOs (LO1 and
 * LO2) and an auxiliary LO, allowing for flexible configuration of the
 * LO source for the Rx/Tx mixers in the device.
 *
 * @param ADI_ADRV9001_LOSEL_LO1 Represents the selection of LO1 for the local
 * oscillator.
 * @param ADI_ADRV9001_LOSEL_LO2 Represents the selection of LO2 for the local
 * oscillator.
 * @param ADI_ADRV9001_LOSEL_AUXLO Represents the selection of an auxiliary
 * local oscillator.
 ******************************************************************************/
typedef enum adi_adrv9001_LoSel
{
    ADI_ADRV9001_LOSEL_LO1 = 1,
    ADI_ADRV9001_LOSEL_LO2 = 2,
    ADI_ADRV9001_LOSEL_AUXLO = 3
} adi_adrv9001_LoSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_LoDividerMode_e` is an enumeration that defines the
 * available modes for the Local Oscillator (LO) divider in the ADRV9001
 * device. It provides two options: a high-performance mode and a low-
 * power mode, allowing users to select the appropriate mode based on
 * their performance or power consumption requirements.
 *
 * @param ADI_ADRV9001_LO_DIV_MODE_HIGH_PERFORMANCE Represents a high-
 * performance mode for the LO
 * divider.
 * @param ADI_ADRV9001_LO_DIV_MODE_LOW_POWER Represents a low-power mode for the
 * LO divider.
 ******************************************************************************/
typedef enum adi_adrv9001_LoDividerMode
{
    ADI_ADRV9001_LO_DIV_MODE_HIGH_PERFORMANCE = 0,
    ADI_ADRV9001_LO_DIV_MODE_LOW_POWER
} adi_adrv9001_LoDividerMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_LoGenPower_e` is an enumeration that defines the
 * power level settings for the Local Oscillator (LO) generation in the
 * ADRV9001 device. It provides two options: using the RFPLL LDO or an
 * off-chip power source. This enumeration is used to configure the power
 * source for the LO generation, which is crucial for the operation of
 * the RF components in the device.
 *
 * @param ADI_ADRV9001_LOGENPOWER_RFPLL_LDO Represents the power level setting
 * for the RFPLL LDO.
 * @param ADI_ADRV9001_LOGENPOWER_OFFCHIP Represents the power level setting for
 * off-chip power.
 ******************************************************************************/
typedef enum adi_adrv9001_LoGenPower
{
    ADI_ADRV9001_LOGENPOWER_RFPLL_LDO = 1,
    ADI_ADRV9001_LOGENPOWER_OFFCHIP = 2
} adi_adrv9001_LoGenPower_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ClockSettings_t` structure is designed to manage and
 * configure the clock settings for the ADRV9001 device. It includes
 * various parameters such as device clock frequency, clock PLL settings,
 * and dividers for internal clocks. The structure also handles power
 * levels for different components, external LO configurations, and RF
 * PLL synchronization modes. This comprehensive configuration allows for
 * precise control over the clocking and LO generation aspects of the
 * ADRV9001, ensuring optimal performance and flexibility in various
 * operational scenarios.
 *
 * @param deviceClock_kHz Device clock frequency in kHz.
 * @param clkPllVcoFreq_daHz CLKPLL VCO frequency in daHz (dekaHertz, 10^1).
 * @param clkPllHsDiv CLKPLL high speed clock divider.
 * @param clkPllMode CLKPLL Mode.
 * @param clk1105Div CLK1105 clock divider.
 * @param armClkDiv ARM Clock divider.
 * @param armPowerSavingClkDiv ARM clock divider used by FW for power saving;
 * ranges from 1 to 256.
 * @param refClockOutEnable Reference clock out enable; False: Disable, True:
 * Enable.
 * @param auxPllPower AUXPLL power level.
 * @param clkPllPower CLKPLL power level.
 * @param padRefClkDrv Output Clock Buffer Drive (valid 0-3).
 * @param extLo1OutFreq_kHz EXT LO1 output frequency in kHz.
 * @param extLo2OutFreq_kHz EXT LO2 output frequency in kHz.
 * @param rfPll1LoMode Internal LO generation for RF LO1; internal LO can be
 * output, or external LO can be input.
 * @param rfPll2LoMode Internal LO generation for RF LO2; internal LO can be
 * output, or external LO can be input.
 * @param ext1LoType External LO1 type; 0 = Differential, 1 = Single Ended.
 * @param ext2LoType External LO2 type; 0 = Differential, 1 = Single Ended.
 * @param rx1RfInputSel RX1 RF input selection; 0 = Rx1A, 1 = Rx1B.
 * @param rx2RfInputSel RX2 RF input selection; 0 = Rx2A, 1 = Rx2B.
 * @param extLo1Divider External LO1 Output divider (valid 2 to 1022).
 * @param extLo2Divider External LO2 Output divider (valid 2 to 1022).
 * @param rfPllPhaseSyncMode RF PLL phase synchronization mode; adds extra time
 * to lock RF PLL when frequency changes.
 * @param rx1LoSelect RX1 LO select; 1 = LO1, 2 = LO2.
 * @param rx2LoSelect RX2 LO select; 1 = LO1, 2 = LO2.
 * @param tx1LoSelect TX1 LO select; 1 = LO1, 2 = LO2.
 * @param tx2LoSelect TX2 LO select; 1 = LO1, 2 = LO2.
 * @param rx1LoDivMode RX1 LO divider mode select; 0: BEST_PHASE_NOISE, 1:
 * BEST_POWER_SAVING.
 * @param rx2LoDivMode RX2 LO divider mode select; 0: BEST_PHASE_NOISE, 1:
 * BEST_POWER_SAVING.
 * @param tx1LoDivMode TX1 LO divider mode select; 0: BEST_PHASE_NOISE, 1:
 * BEST_POWER_SAVING.
 * @param tx2LoDivMode TX2 LO divider mode select; 0: BEST_PHASE_NOISE, 1:
 * BEST_POWER_SAVING.
 * @param loGen1Select LoGen1 selection; 0 = RFPLL1_LDO, 1 = OffChip.
 * @param loGen2Select LoGen2 selection; 0 = RFPLL2_LDO, 1 = OffChip.
 ******************************************************************************/
typedef struct adi_adrv9001_ClockSettings
{
    uint32_t deviceClock_kHz;                         /*!< Device clock frequency in kHz */
    uint32_t clkPllVcoFreq_daHz;                      /*!< CLKPLL VCO frequency in daHz (dekaHertz, 10^1) */
    adi_adrv9001_HsDiv_e  clkPllHsDiv;                /*!< CLKPLL high speed clock divider */
    adi_adrv9001_ClkPllMode_e  clkPllMode;            /*!< CLKPLL Mode */
    adi_adrv9001_InternalClock_Divisor_e  clk1105Div; /*!< CLK1105 clock divider */
    adi_adrv9001_InternalClock_Divisor_e  armClkDiv;  /*!< ARM Clock divider */
    uint16_t armPowerSavingClkDiv;                    /*!< B0: ARM clock divider used by FW for power saving; Ranges from 1 to 256 */
    bool refClockOutEnable;                           /*!< Reference clock out enable; False: Disable reference clock out, True: Enable reference clock out */
    adi_adrv9001_ComponentPowerLevel_e  auxPllPower;  /*!< AUXPLL power level */
    adi_adrv9001_ComponentPowerLevel_e  clkPllPower;  /*!< CLKPLL power level */
    uint8_t padRefClkDrv;                             /*!< Output Clock Buffer Drive (valid 0-3) */
    uint32_t extLo1OutFreq_kHz;                       /*!< EXT LO1 output frequency in kHz */
    uint32_t extLo2OutFreq_kHz;                       /*!< EXT LO2 output frequency in kHz */
    adi_adrv9001_PllLoMode_e rfPll1LoMode;            /*!< internal LO generation for RF LO1, internal LO can be output, or external LO can be input. */
    adi_adrv9001_PllLoMode_e rfPll2LoMode;            /*!< internal LO generation for RF LO2, internal LO can be output, or external LO can be input. */
    adi_adrv9001_ExtLoType_e ext1LoType;              /*!< 0 = Differential, 1= Single Ended */
    adi_adrv9001_ExtLoType_e ext2LoType;              /*!< 0 = Differential, 1= Single Ended */
    adi_adrv9001_RxRfInputSel_e  rx1RfInputSel;       /*!< 0 = Rx1A, 1 = Rx1B */
    adi_adrv9001_RxRfInputSel_e  rx2RfInputSel;       /*!< 0 = Rx2A, 1 = Rx2B */
    uint16_t extLo1Divider;					          /*!< External LO1 Output divider (valid 2 to 1022) */
    uint16_t extLo2Divider;					          /*!< External LO2 Output divider (valid 2 to 1022) */
    adi_adrv9001_RfPllMcs_e rfPllPhaseSyncMode;       /*!< Set RF PLL phase synchronization mode. Adds extra time to lock RF PLL when PLL frequency changed. See enum for options */
    adi_adrv9001_LoSel_e rx1LoSelect;		          /*!< Rx1 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e rx2LoSelect;		          /*!< Rx2 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e tx1LoSelect;		          /*!< Tx1 LO select: 1 = LO1, 2 = LO2  */
    adi_adrv9001_LoSel_e tx2LoSelect;		          /*!< Tx2 LO select: 1 = LO1, 2 = LO2  */

    adi_adrv9001_LoDividerMode_e rx1LoDivMode;        /*!< RX1 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e rx2LoDivMode;        /*!< RX2 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e tx1LoDivMode;        /*!< TX1 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */
    adi_adrv9001_LoDividerMode_e tx2LoDivMode;        /*!< TX2 LO divider mode select: 0: BEST_PHASE_NOISE; 1: BEST_POWER_SAVING */

    adi_adrv9001_LoGenPower_e loGen1Select;		      /*!< D0 - LoGen1 Sel: 0 = RFPLL1_LDO, 1 = OffChip */
    adi_adrv9001_LoGenPower_e loGen2Select;		      /*!< D1 - LoGen2 Sel: 0 = RFPLL2_LDO, 1 = OffChip */
} adi_adrv9001_ClockSettings_t;

#endif /* _ADI_ADRV9001_CLOCKSETTINGS_TYPES_H_ */

/***************************************************************************//**
 *   @file   bia_measurement.h
 *   @brief  Header of Bio Impedance Measurement utility.
 *   @author Kister Jimenez (kister.jimenez@analog.com)
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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
#ifndef BIA_MEASUREMENT_H_
#define BIA_MEASUREMENT_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ad5940.h"

#define MAXSWEEP_POINTS 100 /* Need to know how much buffer is needed to save RTIA calibration result */

/*
  Note: this example will use SEQID_0 as measurment sequence, and use SEQID_1 as init sequence.
  SEQID_3 is used for calibration.
 */

/***************************************************************************//**
 * @brief The `AppBiaCfg_Type` structure is a comprehensive configuration data
 * structure used for bio-impedance measurement applications. It contains
 * various fields that define the operational parameters and settings for
 * the AD5940 device, including sequence addresses, clock frequencies,
 * power modes, and calibration settings. The structure also includes
 * fields for managing the data acquisition process, such as FIFO
 * thresholds, output data rates, and excitation signal parameters.
 * Additionally, it supports sweep function control and maintains
 * internal state information for ongoing measurements and calibrations.
 * This structure is essential for configuring and controlling the bio-
 * impedance measurement process, ensuring accurate and efficient data
 * collection.
 *
 * @param SeqStartAddr Initialization sequence start address in SRAM of AD5940.
 * @param MaxSeqLen Limit the maximum sequence length.
 * @param SeqStartAddrCal Measurement sequence start address in SRAM of AD5940.
 * @param MaxSeqLenCal Limit the maximum calibration sequence length.
 * @param bParamsChanged Indicates if parameters have changed, triggering
 * sequence regeneration.
 * @param bImpedanceReadMode Determines if both voltage and current are read in
 * sequence.
 * @param ReDoRtiaCal Flag to indicate the need for RTIA calibration.
 * @param SysClkFreq The real frequency of the system clock.
 * @param WuptClkFreq The clock frequency of the Wakeup Timer, typically 32kHz.
 * @param AdcClkFreq The real frequency of the ADC clock.
 * @param FifoThresh FIFO threshold, should be a multiple of 4.
 * @param BiaODR Output Data Rate in Hz, determining the period of the Wakeup
 * Timer.
 * @param NumOfData Number of data points to collect before stopping, or -1 to
 * never stop.
 * @param SinFreq Frequency of the excitation signal.
 * @param RcalVal Rcal value in Ohms.
 * @param PwrMod Control chip power mode (Low Power/High Power).
 * @param DacVoltPP Peak-to-peak voltage of DAC output in mV.
 * @param ExcitBufGain Excitation buffer gain selection.
 * @param HsDacGain High-speed DAC gain selection.
 * @param HsDacUpdateRate DAC update rate, determined by System Clock divided by
 * a value between 7 and 255.
 * @param ADCPgaGain PGA Gain selection for ADC.
 * @param ADCSinc3Osr SINC3 oversampling ratio selection.
 * @param ADCSinc2Osr SINC2 oversampling ratio selection.
 * @param HstiaRtiaSel Internal RTIA selection.
 * @param CtiaSel CTIA selection in pF.
 * @param DftNum DFT number.
 * @param DftSrc DFT source.
 * @param HanWinEn Enable Hanning window.
 * @param SweepCfg Configuration for sweep function control.
 * @param SweepCurrFreq Current frequency in the sweep.
 * @param SweepNextFreq Next frequency in the sweep.
 * @param RtiaCurrValue Calibrated RTIA value for the current frequency.
 * @param RtiaCalTable Table of calibrated RTIA values.
 * @param FreqofData Frequency of the latest data sampled.
 * @param BiaInited Indicates if the program has run for the first time and
 * generated sequence commands.
 * @param InitSeqInfo Information about the initialization sequence.
 * @param MeasureSeqInfo Information about the measurement sequence.
 * @param StopRequired Flag to stop the measurement sequence after FIFO is
 * ready.
 * @param FifoDataCount Count of how many times impedance has been measured.
 ******************************************************************************/
typedef struct {
	/* Common configurations for all kinds of Application. */
	uint32_t SeqStartAddr;    /* Initialaztion sequence start address in SRAM of AD5940  */
	uint32_t MaxSeqLen;       /* Limit the maximum sequence.   */
	uint32_t SeqStartAddrCal; /* Measurment sequence start address in SRAM of AD5940 */
	uint32_t MaxSeqLenCal;
	/* Application related parameters */
	//bool bBioElecBoard;     /* The code is same for BioElec board and AD5941Sens1 board. No changes are needed */
	bool bParamsChanged;       /* Indicate to generate sequence again. It's auto cleared by AppBiaInit */
	bool bImpedanceReadMode; /* Read Voltage and current in sequence if True, otherwise, measure only voltage. */
	bool ReDoRtiaCal;     /* Set this flag to bTRUE when there is need to do calibration. */
	float SysClkFreq;            /* The real frequency of system clock */
	float WuptClkFreq;           /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
	float AdcClkFreq;            /* The real frequency of ADC clock */
	uint32_t FifoThresh;         /* FIFO threshold. Should be N*4 */
	float BiaODR;                /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and sample frequency decides the maxim ODR. */
	int32_t NumOfData;           /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
	float SinFreq;               /* Frequency of excitation signal */
	float RcalVal;               /* Rcal value in Ohm */
	uint32_t PwrMod;             /* Control Chip power mode(LP/HP) */
	float DacVoltPP;             /* Final excitation voltage is DAC_VOLTpp*DAC_PGA*EXCIT_GAIN, DAC_PGA= 1 or 0.2, EXCIT_GAIN=2 or 0.25. DAC output voltage in mV peak to peak. Maximum value is 800mVpp. Peak to peak voltage  */
	uint32_t ExcitBufGain;       /* Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */
	uint32_t HsDacGain;          /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */
	uint32_t HsDacUpdateRate;    /* DAC update rate is SystemCLoock/Divider. The available value is 7 to 255. Set to 7 for better perfomance */
	uint32_t ADCPgaGain;         /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */
	uint8_t ADCSinc3Osr;         /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
	uint8_t ADCSinc2Osr;         /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
	uint32_t HstiaRtiaSel;       /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
	uint32_t CtiaSel;            /* Select CTIA in pF unit from 0 to 31pF */

	uint32_t DftNum;   /* DFT number */
	uint32_t DftSrc;   /* DFT Source */
	bool HanWinEn; /* Enable Hanning window */

	/* Sweep Function Control */
	SoftSweepCfg_Type SweepCfg;
	/* Private variables for internal usage */
	float SweepCurrFreq;
	float SweepNextFreq;
	float RtiaCurrValue[2];                 /* Calibrated Rtia value of current frequency */
	float RtiaCalTable[MAXSWEEP_POINTS][2]; /* Calibrated Rtia Value table */
	float FreqofData;                       /* The frequency of latest data sampled */
	bool BiaInited;                     /* If the program run firstly, generated sequence commands */
	SEQInfo_Type InitSeqInfo;
	SEQInfo_Type MeasureSeqInfo;
	bool StopRequired;  /* After FIFO is ready, stop the measurment sequence */
	uint32_t FifoDataCount; /* Count how many times impedance have been measured */
	/* End */
} AppBiaCfg_Type;

#define BIACTRL_START 0
#define BIACTRL_STOPNOW 1
#define BIACTRL_STOPSYNC 2
#define BIACTRL_GETFREQ 3  /* Get Current frequency of returned data from ISR */
#define BIACTRL_SHUTDOWN 4 /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

/***************************************************************************//**
 * @brief This function is used to obtain a pointer to the current configuration
 * of the Bio Impedance Application. It is typically called when there is
 * a need to access or modify the configuration settings of the
 * application. The function requires a valid pointer to a pointer
 * variable where the address of the configuration will be stored. It is
 * important to ensure that the provided pointer is not null, as the
 * function will return an error code if it is. This function does not
 * modify the configuration itself, only provides access to it.
 *
 * @param pCfg A pointer to a pointer variable where the address of the current
 * configuration will be stored. Must not be null. If null, the
 * function returns an error code.
 * @return Returns 0 on success, indicating the configuration pointer was
 * successfully retrieved. Returns -EINVAL if the provided pointer is
 * null.
 ******************************************************************************/
int AppBiaGetCfg(void *pCfg);
/***************************************************************************//**
 * @brief This function sets up the Bio Impedance Application by configuring the
 * necessary hardware components and initializing the sequencer. It must
 * be called before any bio impedance measurements are performed. The
 * function requires a valid device structure and a buffer for sequence
 * generation. It handles calibration and configuration changes
 * automatically. Ensure that the device is properly powered and
 * connected before calling this function.
 *
 * @param dev A pointer to an ad5940_dev structure representing the device to be
 * initialized. Must not be null.
 * @param pBuffer A pointer to a buffer used for sequence generation. Must not
 * be null and should have sufficient size as specified by
 * BufferSize.
 * @param BufferSize The size of the buffer pointed to by pBuffer. Must be
 * greater than zero.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int AppBiaInit(struct ad5940_dev *dev, uint32_t *pBuffer, uint32_t nBufferSize);
/***************************************************************************//**
 * @brief This function processes the interrupt for bio-impedance measurement by
 * reading data from the FIFO buffer of the AD5940 device and storing it
 * in the provided buffer. It should be called when an interrupt
 * indicating data availability is received. The function requires that
 * the bio-impedance application has been initialized and that the
 * provided buffer and count pointers are valid. It manages the wake-up
 * and sleep states of the device and updates the count of data read. If
 * the application is not initialized or if invalid pointers are
 * provided, the function returns an error code.
 *
 * @param dev A pointer to the ad5940_dev structure representing the device.
 * Must not be null.
 * @param pBuff A pointer to a buffer where the read data will be stored. Must
 * not be null.
 * @param pCount A pointer to a uint32_t variable that initially contains the
 * maximum number of data items the buffer can hold. It will be
 * updated with the actual number of data items read. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code on failure. The error
 * codes include -EINVAL for invalid parameters, -EAGAIN if the
 * application is not initialized, and other negative values for device-
 * specific errors.
 ******************************************************************************/
int AppBiaISR(struct ad5940_dev *dev, void *pBuff, uint32_t *pCountd);
/***************************************************************************//**
 * @brief This function manages the bio-impedance measurement process by
 * starting, stopping, synchronizing, retrieving frequency, or shutting
 * down the measurement system based on the control command provided. It
 * should be used to control the measurement state of the bio-impedance
 * application, ensuring that the device is properly initialized before
 * starting measurements. The function handles different control
 * commands, each affecting the measurement process differently, such as
 * starting or stopping the measurement immediately or synchronously,
 * retrieving the current frequency, or shutting down the system. It is
 * important to handle the return values appropriately to manage errors
 * or specific conditions like uninitialized states.
 *
 * @param dev A pointer to an ad5940_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param BcmCtrl An integer control command that dictates the action to be
 * performed. Valid values are BIACTRL_START, BIACTRL_STOPNOW,
 * BIACTRL_STOPSYNC, BIACTRL_GETFREQ, and BIACTRL_SHUTDOWN.
 * Invalid values result in no action.
 * @param pPara A pointer used to store output data for certain control
 * commands, specifically BIACTRL_GETFREQ. It can be null if not
 * required for the command being executed.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * issues such as device wake-up failure or uninitialized states.
 ******************************************************************************/
int AppBiaCtrl(struct ad5940_dev *dev, int32_t BcmCtrl, void *pPara);
/***************************************************************************//**
 * @brief This function processes an array of 18-bit integers stored in a 32-bit
 * format, sign-extending each element to a full 32-bit signed integer.
 * It is typically used when handling data that originates from a source
 * providing 18-bit two's complement values, ensuring that the sign is
 * correctly propagated in the conversion to a 32-bit integer. The
 * function must be called with a valid pointer to an array and a length
 * indicating the number of elements to process. It modifies the array in
 * place, so the caller must ensure that the array is properly allocated
 * and that the length does not exceed the actual number of elements in
 * the array.
 *
 * @param pData A pointer to an array of 32-bit unsigned integers, where each
 * element contains an 18-bit two's complement value. The pointer
 * must not be null, and the array must be large enough to hold at
 * least 'nLen' elements. The caller retains ownership of the
 * array.
 * @param nLen The number of elements in the array to process. It must be a non-
 * negative integer and should not exceed the actual number of
 * elements in the array pointed to by 'pData'.
 * @return None
 ******************************************************************************/
void signExtend18To32(uint32_t *const pData, uint16_t nLen);
/***************************************************************************//**
 * @brief This function calculates the impedance using voltage and current data
 * provided in a specific format. It is intended for use in bio-impedance
 * measurement applications where the input data represents real and
 * imaginary components of voltage and current. The function must be
 * called with a valid pointer to an array of four 32-bit unsigned
 * integers, which represent the real and imaginary parts of voltage and
 * current, respectively. The function returns a complex impedance value
 * as a structure containing real and imaginary components. Ensure that
 * the input data is correctly formatted and that the pointer is not null
 * before calling this function.
 *
 * @param pData A pointer to an array of four uint32_t values representing the
 * real and imaginary parts of voltage and current. The array must
 * be formatted as [VRe, VIm, IRe, IIm]. The pointer must not be
 * null, and the caller retains ownership of the data.
 * @return Returns a fImpCar_Type structure containing the computed real and
 * imaginary components of the impedance.
 ******************************************************************************/
fImpCar_Type computeImpedance(uint32_t *const pData);

#endif /* BIA_MEASUREMENT_H_ */

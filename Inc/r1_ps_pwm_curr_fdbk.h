
/**
  ******************************************************************************
  * @file    r1_ps_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_ps_pwm_curr_fdbk component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup r1_ps_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R1_PS_PWMCURRFDBK_H
#define R1_PS_PWMCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r1_ps_pwm_curr_fdbk
  * @{
  */

/* Exported constants --------------------------------------------------------*/
#define NONE      ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))
#define DAC_MODE  ((uint8_t)(0x03))

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Current feedback component parameters structure definition for 1 Shunt configurations. Common to F30X, F7XX, G0XX, C0XX, G4XX and L4XX MCUs.
 *
 */
typedef struct
{
  /* HW IP involved -----------------------------*/
  ADC_TypeDef *ADCx;                       /*!< ADC peripheral to be used. */
  TIM_TypeDef *TIMx;                       /*!< Timer used for PWM generation. */
  DMA_TypeDef *DMAx;                       /*!< DMA used to access memory. */
  uint32_t DMAChannelX;                    /*!< DMA channel used to modify CCR on the fly. */

 /* PWM generation parameters --------------------------------------------------*/

  uint16_t TMin;                           /*!< Minimum time after which the ADC is allowed to trigger a conversion. */
  uint16_t TSample;                        /*!< Sampling time expressed in number of TIM clocks. */
  uint16_t hTADConv;                       /*!< Time recquired for the ADC to perform a conversion after sampling. Expressed in number of TIM clocks. */

  /* DAC settings --------------------------------------------------------------*/
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t IChannel;                        /*!< ADC Channel used for conversion. */
  uint8_t  RepetitionCounter;              /*!< It expresses the number of PWM periods to be elapsed before compare
                                                registers are updated again. In particular:
                                                @f$ RepetitionCounter\ =\ (2\times PWM\ Periods)\ -\ 1 @f$ */
  /* Internal COMP settings ----------------------------------------------------*/
  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  FreqRatio;                      /*!< It is used in case of dual MC to synchronize TIM1 and TIM8. It has
                                                effect only on the second instanced object and must be equal to the
                                                ratio between the two PWM frequencies (higher/lower).
                                                Supported values are 1, 2 or 3. */
  uint8_t  IsHigherFreqTim;                /*!< When FreqRatio is greather than 1 this param is used to indicate if
                                                this instance is the one with the highest frequency. Allowed value are:
                                                HIGHER_FREQ or LOWER_FREQ. */

} R1_Params_t;

/**
  * @brief  This structure is used to handle an instance of the
  *         Current feedback component for 1 Shunt configurations. Common to F30X, F7XX, G0XX, G4XX and L4XX MCUs.
  */
typedef struct
{
  PWMC_Handle_t _Super;                    /*!< Offset of current sensing network. */
  uint16_t DmaBuffCCR[8];                  /*!< Buffer used for PWM phase shift points. */
  uint16_t DmaBuffCCR_latch[8];            /*!< Buffer used to latch PWM phase shift points. */
  uint32_t PhaseOffset;                    /*!< Offset of Phase current sensing network. */
  uint16_t aShiftval[3];                   /*!< Shift value to be applied. */
  uint16_t Half_PWMPeriod;                 /* Half PWM Period in timer clock counts. */
  uint16_t CntSmp1;                        /*!< First sampling point express in timer counts. */
  uint16_t CntSmp2;                        /*!< Second sampling point express in timer counts. */
  uint8_t sampCur1;                        /*!< Current sampled in the first sampling point. */
  uint8_t sampCur2;                        /*!< Current sampled in the second sampling point. */
  int16_t CurrAOld;                        /*!< Previous measured value of phase A current. */
  int16_t CurrBOld;                        /*!< Previous measured value of phase B current. */
  volatile uint8_t  Index;                 /*!< Number of conversions performed during the calibration phase. */
  uint8_t iflag;
  uint8_t TCCnt;

  bool UpdateFlagBuffer;                   /*!< Buffered version of Timer update IT flag. */
  volatile bool TCDoneFlag;                /*!< This flag is used to indicate that last DMA TC of the period is done. */
  /* Should be there */
  R1_Params_t const * pParams_str;

} PWMC_R1_Handle_t;

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in single shunt topology using STM32 MCU and ADC.
  */
void R1_Init(PWMC_R1_Handle_t *pHandle);

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R1_SwitchOffPWM(PWMC_Handle_t *pHdl);

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R1_SwitchOnPWM(PWMC_Handle_t *pHdl);

/*
  * Turns on low sides switches.
  */
void R1_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks);

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *pStator_Currents);

/*
  * Contains the TIMx Update event interrupt.
  */
void *R1_TIMx_UP_IRQHandler(PWMC_R1_Handle_t *pHandle);

/*
  * Contains the TIMx Break1/2 event interrupt.
  */
void *R1_OCP_IRQHandler(PWMC_R1_Handle_t *pHandle);

/*
  * Contains the TIMx Break1/2 event interrupt.
  */
void *R1_OVP_IRQHandler(PWMC_R1_Handle_t *pHandle);

/*
  * Stores into the the handler the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor.
  */
void R1_CurrentReadingCalibration(PWMC_Handle_t *pHdl);

/*
  * Implementation of the single shunt algorithm to setup the
  * TIM1 register and DMA buffers values for the next PWM period.
  */
uint16_t R1_CalcDutyCycles(PWMC_Handle_t *pHdl);

/*
  * Contains the motor DMAx TC interrupt request.
  */
void *R1_DMAx_TC_IRQHandler(PWMC_R1_Handle_t *pHandle);

/*
  * Contains the motor DMAx HT interrupt request.
  */
void *R1_DMAx_HT_IRQHandler(PWMC_R1_Handle_t *pHandle);

/*
  * Stores in the handler the calibrated offsets.
  */
void R1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets stored in the handler.
  */
void R1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*R1_PS_F30X_PWMCURRFDBK_H*/

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

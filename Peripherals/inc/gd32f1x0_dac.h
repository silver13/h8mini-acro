/**
  ******************************************************************************
  * @file    gd32f1x0_dac.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   DAC header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_DAC_H
#define __GD32F1X0_DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"
 
/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup DAC
  * @{
  */

/** @defgroup DAC_Exported_Types
  * @{
  */

/** 
  * @brief  DAC Init structure definition
  */
typedef struct
{
    uint32_t DAC_Trigger;               /*!< external trigger of the selected DAC channel.
                                             This parameter can be a value of @ref DAC_Trigger */
    uint32_t DAC_OutputBuffer;          /*!< Specifies whether the DAC channel output buffer is enabled or disabled
                                             This parameter can be a value of @ref DAC_OutputBuffer */
}DAC_InitPara;

/** @defgroup DAC_Exported_Constants
  * @{
  */

/** @defgroup DAC_Trigger
  * @{
  */
#define DAC_TRIGGER_NONE                ((uint32_t)0x00000000) /*!< no trigger */
#define DAC_TRIGGER_T2_TRGO             ((uint32_t)0x00000024) /*!< TIMER2 TRGO */
#define DAC_TRIGGER_T3_TRGO             ((uint32_t)0x0000000C) /*!< TIMER3 TRGO */
#define DAC_TRIGGER_T6_TRGO             ((uint32_t)0x00000004) /*!< TIMER6 TRGO */
#define DAC_TRIGGER_T15_TRGO            ((uint32_t)0x0000001C) /*!< TIMER15 TRGO */
#define DAC_TRIGGER_EXT_IT9             ((uint32_t)0x00000034) /*!< EXTI Line9 event */
#define DAC_TRIGGER_SOFTWARE            ((uint32_t)0x0000003C) /*!< software trigger */

/**
  * @}
  */

/** @defgroup DAC_OutputBuffer
  * @{
  */

#define DAC_OUTPUTBUFFER_ENABLE         ((uint32_t)0x00000000)
#define DAC_OUTPUTBUFFER_DISABLE        DAC_CTLR_BOFF

/**
  * @}
  */

/** @defgroup DAC_data_alignment
  * @{
  */
#define DAC_ALIGN_12B_R                 ((uint32_t)0x00000000)
#define DAC_ALIGN_12B_L                 ((uint32_t)0x00000004)
#define DAC_ALIGN_8B_R                  ((uint32_t)0x00000008)

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions
  * @{
  */
void DAC_DeInit(DAC_InitPara* DAC_InitParaStruct);
void DAC_Init(DAC_InitPara* DAC_InitParaStruct);
void DAC_Enable(TypeState NewValue);
void DAC_SoftwareTrigger_Enable(TypeState NewValue);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
uint16_t DAC_GetDataOutputValue(void);
void DAC_DMA_Enable(TypeState NewValue);
void DAC_INTConfig(TypeState NewValue);
TypeState DAC_GetBitState(void);
void DAC_ClearBitState(void);
TypeState DAC_GetINTState(void);
void DAC_Clear_INTPendingBit(void);

#ifdef __cplusplus
}
#endif

#endif /*__GD32F1X0_DAC_H */

/**
  * @}
  */

/**
  * @}
  */
/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

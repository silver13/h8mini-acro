/**
  ******************************************************************************
  * @file    gd32f1x0_cmp.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CMP functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_cmp.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup CMP 
  * @brief CMP driver modules
  * @{
  */ 

/** @defgroup CMP_Private_Defines
  * @{
  */

/* CMP_CSR register Mask */
#define CMP_CSR_CLEAR_MASK              ((uint32_t)0x00003FFE)

/**
  * @}
  */

/** @defgroup CMP_Private_Functions
  * @{
  */

/**
  * @brief  Reset the CMP.
  * @param  None
  * @retval None
  */
void CMP_DeInit(void)
{
    /* Initial CSR register with reset value */
    CMP->CSR = ((uint32_t)0x00000000);
}

/**
  * @brief  Initialize the CMP parameters.
  * @param  CMP_Channel: the comparator channel.
  *   This parameter can be one of the following values:
  *     @arg CMP_CHANNEL_CMP1: CMP1
  *     @arg CMP_CHANNEL_CMP2: CMP2
  * @param  CMP_InitParaStruct: the sturct CMP_InitPara pointer.
  * @retval None
  */
void CMP_Init(uint32_t CMP_Channel, CMP_InitPara* CMP_InitParaStruct)
{
    uint32_t temp = 0;
    
    temp = CMP->CSR;
    temp &= (uint32_t) ~(CMP_CSR_CLEAR_MASK<<CMP_Channel);
    
    /* The CMP initial parameters */
    temp |= (uint32_t)((CMP_InitParaStruct->CMP_InvertingInput |
                        CMP_InitParaStruct->CMP_Output |
                        CMP_InitParaStruct->CMP_OutputPolarity |
                        CMP_InitParaStruct->CMP_Hysteresis |
                        CMP_InitParaStruct->CMP_OperatingMode)<<CMP_Channel);
    
    /* CMP_CSR */
    CMP->CSR = temp;  
}

/**
  * @brief  Initial the sturct CMP_InitPara
  * @param  CMP_InitParaStruct: the CMP_InitPara struct pointer
  * @retval None
  */
void CMP_ParaInit(CMP_InitPara* CMP_InitParaStruct)
{
    CMP_InitParaStruct->CMP_InvertingInput = CMP_INVERTINGINPUT_1_4VREFINT;
    CMP_InitParaStruct->CMP_Output = CMP_OUTPUT_NONE;
    CMP_InitParaStruct->CMP_OutputPolarity = CMP_OUTPUTPOLARITY_NONINVERTED;
    CMP_InitParaStruct->CMP_Hysteresis = CMP_HYSTERESIS_NO;
    CMP_InitParaStruct->CMP_OperatingMode = CMP_OPERATINGMODE_VERYLOWSPEED;
}

/**
  * @brief  Enable or disable the CMP channel.
  * @param  CMP_Channel: the CMP channel.
  *   This parameter can be one of the following values:
  *     @arg CMP_CHANNEL_CMP1: CMP1 channel
  *     @arg CMP_CHANNEL_CMP2: CMP2 channel
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CMP_Enable(uint32_t CMP_Channel, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable CMP_CSR_CMPEN control bit */
        CMP->CSR |= (uint32_t) (1<<CMP_Channel);
    }
    else
    {
        /* Disable CMP_CSR_CMPEN control bit */
        CMP->CSR &= (uint32_t)(~((uint32_t)1<<CMP_Channel));
    }
}

/**
  * @brief  Enable or disable the CMP switch.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CMP_Switch_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable CMP_CSR_CMP1SW control bit */
        CMP->CSR |= (uint32_t) (CMP_CSR_CMP1SW);
    }
    else
    {
        /* Disable CMP_CSR_CMP1SW control bit */
        CMP->CSR &= (uint32_t)(~CMP_CSR_CMP1SW);
    }
}

/**
  * @brief  The output level of CMP.
  * @param  CMP_Channel: the CMP channel.
  *   This parameter can be one of the following values:
  *     @arg CMP_CHANNEL_CMP1: CMP1 channel
  *     @arg CMP_CHANNEL_CMP2: CMP2 channel  
  * @retval The output level.
  */
uint32_t CMP_GetOutputLevel(uint32_t CMP_Channel)
{
    if ((CMP->CSR & (CMP_CSR_CMP1O<<CMP_Channel)) != 0)
    {
        /* High level */
        return CMP_OUTPUTLEVEL_HIGH;
    }
    else
    {
        /* Low level */
        return CMP_OUTPUTLEVEL_LOW;
    }
}

/**
  * @brief  Enable or disable the CMP window.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CMP_Window_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the CMP_CSR_WNDEN control bit */
        CMP->CSR |= (uint32_t) CMP_CSR_WNDEN;
    }
    else
    {
        /* Disable the CMP_CSR_WNDEN control bit */
        CMP->CSR &= (uint32_t)(~CMP_CSR_WNDEN);
    }
}

/**
  * @brief  Lock the CMP.
  * @param  CMP_Channel: the CMP channel.
  *   This parameter can be a value of the following values:
  *     @arg CMP_CHANNEL_CMP1: CMP1 channel
  *     @arg CMP_CHANNEL_CMP2: CMP2 channel
  * @retval None
  */
void CMP_LockConfig(uint32_t CMP_Channel)
{
    /* Lock the CMP */
    CMP->CSR |= (uint32_t) (CMP_CSR_CMP1LK<<CMP_Channel);
}

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

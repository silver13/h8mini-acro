/**
  ******************************************************************************
  * @file    gd32f1x0_dac.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   DAC functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_dac.h"
#include "gd32f1x0_rcc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup DAC 
  * @brief DAC driver modules
  * @{
  */

/** @defgroup DAC_Private_Defines
  * @{
  */

/* Private define ------------------------------------------------------------*/
/* CTLR register bits mask */
#define CTLR_BITS_CLEAR                 ((uint32_t)0x0000003E)
/* DHR registers offsets */
#define DHR12R_OFFSET                   ((uint32_t)0x00000008)
/* DOR register offset */
#define DOR_OFFSET                      ((uint32_t)0x0000002C)

/**
  * @}
  */

/** @defgroup DAC_Private_Functions
  * @{
  */

/**
  * @brief  Deinitialize the DAC peripheral registers.
  * @param  DAC_InitStruct: DAC_InitTypeDef structure that contains the
  *   configuration information for the selected DAC channel.
  * @retval None
  */
void DAC_DeInit(DAC_InitPara* DAC_InitParaStruct)
{
    /* Enable DAC reset state */
    RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_DACRST, ENABLE);
    /* Release DAC from reset state */
    RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_DACRST, DISABLE);
    /* Initialize the DAC_Trigger member */
    DAC_InitParaStruct->DAC_Trigger = DAC_TRIGGER_NONE;
    /* Initialize the DAC_OutputBuffer member */
    DAC_InitParaStruct->DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
}

/**
  * @brief  Initialize the DAC peripheral.
  * @param  DAC_InitStruct: DAC_InitTypeDef structure that contains the
  *   configuration information for the specified DAC channel.
  * @retval None
  */
void DAC_Init(DAC_InitPara* DAC_InitParaStruct)
{
    uint32_t temp1 = 0, temp2 = 0;
    /* DAC CTLR Configuration */
    /* Get the DAC CTLR value */
    temp1 = DAC->CTLR;

    /* Clear BOFF, TEN, TSEL bits */
    temp1 &= ~(CTLR_BITS_CLEAR);

    /* Configure for the DAC channel: buffer output, trigger */
    /* Set TSEL and TEN bits according to DAC_Trigger value */
    /* Set BOFF bit according to DAC_OutputBuffer value */
    temp2 = (DAC_InitParaStruct->DAC_Trigger | DAC_InitParaStruct->DAC_OutputBuffer);

    /* Calculate CTLR register value depending on DAC_Channel */
    temp1 |= temp2;

    /* Write to DAC CTLR */
    DAC->CTLR = temp1;
}

/**
  * @brief  Enable or disable the DAC channel.
  * @param  NewValue: Alternative state of the DAC channel. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the selected DAC channel */
        DAC->CTLR |= DAC_CTLR_DACEN ;
    }
    else
    {
        /* Disable the selected DAC channel */
        DAC->CTLR &= (~DAC_CTLR_DACEN);
    }
}

/**
  * @brief  Enable or disable the selected DAC channel software trigger.
  * @param  NewValue: Alternative state of the selected DAC channel software trigger.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_SoftwareTrigger_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable software trigger for DAC channel1 */
        DAC->SWTR |= (uint32_t)DAC_SWTR_SWTR;
    }
    else
    {
        /* Disable software trigger for DAC channel1 */
        DAC->SWTR &= ~((uint32_t)DAC_SWTR_SWTR);
    }
}

/**
  * @brief  Set the specified data holding register value for DAC channel1.
  * @param  DAC_Align: Specifies the data alignment for DAC channel1.
  *   This parameter can be one of the following values:
  *     @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *     @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *     @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data: Data to be loaded in the selected data holding register.
  * @retval None
  */
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
{
    __IO uint32_t temp = 0;

    temp = (uint32_t)DAC_BASE; 
    temp += DHR12R_OFFSET + DAC_Align;

    /* Set the DAC channel1 selected data holding register */
    *(__IO uint32_t *) temp = Data;
}

/**
  * @brief  Return the last data output value of DAC channel1.
  * @retval The DAC channel1 data output value.
  */
uint16_t DAC_GetDataOutputValue(void)
{
    __IO uint32_t temp = 0;

    temp = (uint32_t) DAC_BASE;
    temp += DOR_OFFSET;

    /* Returns the DAC channel data output register value */
    return (uint16_t) (*(__IO uint32_t*) temp);
}

/**
  * @brief  Enable or disable DAC channel1 DMA request.
  *         When enabled DMA1 is generated when an external trigger (EXTI Line9,
  *         TIMER2, TIMER3, TIMER6 or TIMER15  but not a software trigger) occurs
  * @param  NewValue: Alternative state of the selected DAC channel DMA request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_DMA_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the selected DAC channel DMA request */
        DAC->CTLR |= DAC_CTLR_DMAEN;
    }
    else
    {
        /* Disable the selected DAC channel DMA request */
        DAC->CTLR &= (~DAC_CTLR_DMAEN);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  Enable or disable the specified DAC interrupts.
  * @param  NewValue: Alternative state of the specified DAC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_INTConfig(TypeState NewValue)  
{
    if (NewValue != DISABLE)
    {
        /* Enable the selected DAC interrupts */
        DAC->CTLR |=  DAC_STR_DMAUDR;
    }
    else
    {
        /* Disable the selected DAC interrupts */
        DAC->CTLR &= (~(uint32_t)DAC_STR_DMAUDR);
    }
}

/**
  * @brief  Check the specified DAC flag.
  * @retval The new state of DAC_FLAG (SET or RESET).
  */
TypeState DAC_GetBitState(void)
{
    TypeState state = RESET;
    
    /* Check the status of DAC DMAUDR flag */
    if ((DAC->STR & DAC_STR_DMAUDR) != (uint8_t)RESET)
    {
        /* DAC_FLAG is set */
        state = SET;
    }
    else
    {
        /* DAC_FLAG is reset */
        state = RESET;
    }
    /* Return the DAC_FLAG status */
    return state;
}

/**
  * @brief  Clear the DAC channel1's DMAUDR flags.
  * @retval None
  */
void DAC_ClearBitState(void)
{
    /* Clear the DAC DMAUDR flags */
    DAC->STR = DAC_STR_DMAUDR;
}

/**
  * @brief  Check the DAC DMAUDR interrupt.
  * @retval The new state of DAC_IT (SET or RESET).
  */
TypeState DAC_GetINTState(void)
{
    /* Check the status of the specified DAC interrupt */
    if (((DAC->STR & DAC_STR_DMAUDR) != (uint32_t)RESET) && (DAC->CTLR & DAC_STR_DMAUDR))
    {
        /* DAC_IT is set */
        return SET;
    }
    else
    {
        /* DAC_IT is reset */
        return RESET;
    }
}

/**
  * @brief  Clear the DAC channel1's interrupt pending bits.
  * @retval None
  */
void DAC_ClearINTPendingBit(void)
{
    /* Clear DAC_IT_DMAUDR interrupt pending bits */
    DAC->STR = DAC_STR_DMAUDR;
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

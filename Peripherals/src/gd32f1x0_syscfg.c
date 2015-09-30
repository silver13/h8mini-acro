/**
  ******************************************************************************
  * @file    gd32f1x0_syscfg.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   SYSCFG functions of the firmware library.
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_syscfg.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup SYSCFG 
  * @brief SYSCFG driver modules
  * @{
  */

/** @defgroup SYSCFG_Private_Defines
  * @{
  */
#define        EXTI_SS_MASK            0x0F
#define        EXTI_SS_STEP            0x04
#define        EXTI_PIN_MASK           0x03
#define        EXTI_SS_OFFSET          0x02

/**
  * @}
  */

/** @defgroup SYSCFG_Private_Functions
  * @{
  */ 

/**
  * @brief  Reset the SYSCFG registers.
  * @param  None
  * @retval None
  * @note   BOOT_MODE bits are read only.
  */
void SYSCFG_DeInit(void)
{  
    SYSCFG->R1 = 0;  
    SYSCFG->EXTISS[0] = 0;
    SYSCFG->EXTISS[1] = 0;
    SYSCFG->EXTISS[2] = 0;
    SYSCFG->EXTISS[3] = 0;
    SYSCFG->R2 = 0;
}

/**
  * @brief  Enable or disable the DMA channels remapping
  * @param  SYSCFG_dma_remap: Specify the DMA channels to remap.
  *   This parameter can be any of the following values:
  *     @arg SYSCFG_R1_TIM17_DMA_RMP: Remap TIM17 CH1 and UP DMA requests to channel2(defaut channel1) 
  *     @arg SYSCFG_R1_TIM16_DMA_RMP: Remap TIM16 CH1 and UP DMA requests to channel4(defaut channel3) 
  *     @arg SYSCFG_R1_USART1_RX_DMA_RMP: Remap USART1 Rx DMA request to channel5(default channel3)
  *     @arg SYSCFG_R1_USART1_TX_DMA_RMP: Remap USART1 Tx DMA request to channel4(default channel2)
  *     @arg SYSCFG_R1_ADC_DMA_RMP: Remap ADC1 DMA requests from channel1 to channel2
  * @param  NewValue: DMA remapping state to configure.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SYSCFG_DMARemap_Config(uint32_t SYSCFG_dma_remap, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        SYSCFG->R1 |= (uint32_t)SYSCFG_dma_remap;
    }
    else
    {
        SYSCFG->R1 &= (uint32_t)(~SYSCFG_dma_remap);
    }
}

/**
  * @brief  Enable or disable PB9 high current capability
  * @param  NewValue: PB9 high current state to configure. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SYSCFG_HighCurrent_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {    
        SYSCFG->R1 |= (uint32_t)SYSCFG_R1_PB9_HCCE;
    }
    else
    {   
        SYSCFG->R1 &= (uint32_t)(~SYSCFG_R1_PB9_HCCE);
    }
}

/**
  * @brief  Configure the GPIO pin as EXTI Line
  * @param  exti_port: Specify the GPIO port used in EXTI
  *   This parameter can be  any of the following values:
  *     @arg EXTI_SOURCE_GPIOA
  *     @arg EXTI_SOURCE_GPIOB
  *     @arg EXTI_SOURCE_GPIOC
  *     @arg EXTI_SOURCE_GPIOD
  *     @arg EXTI_SOURCE_GPIOF 
  * @param  exti_pin: Specify the EXTI line
  *   This parameter can be  any of the following values:
  *     @arg EXTI_SOURCE_PIN0
  *     @arg EXTI_SOURCE_PIN1
  *     @arg EXTI_SOURCE_PIN2
  *     @arg EXTI_SOURCE_PIN3
  *     @arg EXTI_SOURCE_PIN4
  *     @arg EXTI_SOURCE_PIN5
  *     @arg EXTI_SOURCE_PIN6
  *     @arg EXTI_SOURCE_PIN7
  *     @arg EXTI_SOURCE_PIN8
  *     @arg EXTI_SOURCE_PIN9
  *     @arg EXTI_SOURCE_PIN10
  *     @arg EXTI_SOURCE_PIN11
  *     @arg EXTI_SOURCE_PIN12
  *     @arg EXTI_SOURCE_PIN13
  *     @arg EXTI_SOURCE_PIN14
  *     @arg EXTI_SOURCE_PIN15
  * @retval None
  */
void SYSCFG_EXTILine_Config(uint8_t exti_port, uint8_t exti_pin)
{
    uint32_t temp = 0x00;

    temp = ((uint32_t)EXTI_SS_MASK) << (EXTI_SS_STEP * (exti_pin & (uint8_t)EXTI_PIN_MASK));
    SYSCFG->EXTISS[exti_pin >> EXTI_SS_OFFSET] &= ~temp;
    SYSCFG->EXTISS[exti_pin >> EXTI_SS_OFFSET] |= (((uint32_t)exti_port) << (EXTI_SS_STEP * (exti_pin & (uint8_t)EXTI_PIN_MASK)));
}

/**
  * @brief  Connect TIM1/15/16/17 break input to the selected parameter
  * @param  SYSCFG_lock: Specify the parameter to be connected
  *   This parameter can be any of the following values
  *     @arg SYSCFG_R2_LVD_LOCK: LVD interrupt connected to the break input
  *     @arg SYSCFG_R2_SRAM_PARITY_ERROR_LOCK: SRAM_PARITY check error connected to the break input
  *     @arg SYSCFG_R2_LOCKUP_LOCK: Cortex-M3 Lockup output connected to the break input
  * @retval None
  */
void SYSCFG_Lock_Config(uint32_t SYSCFG_lock)
{
    SYSCFG->R2 |= (uint32_t) SYSCFG_lock;
}

/**
  * @brief  Check if the specified flag in SYSCFG_R2 is set or not.
  * @param  SYSCFG_flag: Specify the flag in SYSCFG_R2 to check.
  *   This parameter can be any of the following values:
  *     @arg SYSCFG_R2_SRAM_PCEF: SRAM parity check error flag.
  * @retval The SYSCFG_flag state returned (SET or RESET).
  */
TypeState SYSCFG_GetBitState(uint32_t SYSCFG_flag)
{
    if ((SYSCFG->R2 & SYSCFG_R2_SRAM_PCEF) != (uint32_t)RESET)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the flag in SYSCFG_R2.
  * @param  SYSCFG_flag: Specify the flag in SYSCFG_R2 to clear.
  *   This parameter can be any of the following values:
  *     @arg SYSCFG_R2_SRAM_PCEF: SRAM parity check error flag.
  * @retval None
  */
void SYSCFG_ClearBitState(uint32_t SYSCFG_flag)
{
    SYSCFG->R2 |= (uint32_t) SYSCFG_flag;
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

/************************ (C) COPYRIGHT 2014 GIGADEVICE*****END OF FILE****/

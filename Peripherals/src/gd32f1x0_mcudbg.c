/**
  ******************************************************************************
  * @file    gd32f1x0_mcudbg.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   MCUDBG functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_mcudbg.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup MCUDBG 
  * @brief MCUDBG driver modules
  * @{
  */ 

/** @defgroup MCUDBG_Private_Defines
  * @{
  */
#define IDCODE_DEVID_MASK    ((uint32_t)0x00000FFF)

/**
  * @}
  */

/** @defgroup MCUDBG_Private_Functions
  * @{
  */

/**
  * @brief  Return the device revision identifier.
  * @param  None
  * @retval Device revision identifier
  */
uint32_t MCUDBG_GetREVID(void)
{
   return(MCUDBG->IDR >> 16);
}

/**
  * @brief  Return the device identifier.
  * @param  None
  * @retval Device identifier
  */
uint32_t MCUDBG_GetDEVID(void)
{
   return(MCUDBG->IDR & IDCODE_DEVID_MASK);
}

/**
  * @brief  Configure APB1 peripheral behavior in Debug mode.
  * @param  MCUDBG_APB1Periph: APB1 peripheral.
  *   This parameter can be any combination of the following values:
  *     @arg MCUDBG_SLEEP_HOLD: Keep debugger connection during SLEEP mode              
  *     @arg MCUDBG_DEEPSLEEP_HOLD: Keep debugger connection during DEEPSLEEP mode               
  *     @arg MCUDBG_STANDBY_HOLD: Keep debugger connection during STANDBY mode            
  *     @arg MCUDBG_IWDG_HOLD: Debug IWDG kept when Core is halted          
  *     @arg MCUDBG_WWDG_HOLD: Debug WWDG kept when Core is halted          
  *     @arg MCUDBG_TIMER1_HOLD: TIMER1 counter kept when Core is halted
  *     @arg MCUDBG_TIMER2_HOLD: TIMER2 counter kept when Core is halted
  *     @arg MCUDBG_TIMER3_HOLD: TIMER3 counter kept when Core is halted
  *     @arg MCUDBG_I2C1_HOLD: Hold I2C1 SMBUS when Core is halted
  *     @arg MCUDBG_I2C2_HOLD: Hold I2C2 SMBUS when Core is halted
  *     @arg MCUDBG_I2C3_HOLD: Hold I2C3 SMBUS when Core is halted
  *     @arg MCUDBG_TIMER6_HOLD: Hold TIMER6 counter when Core is halted
  *     @arg MCUDBG_TIMER14_HOLD: Hold TIMER14 counter when Core is halted
  * @param  NewValue: new value of the specified APB1 peripheral in Debug mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void MCUDBG_APB1PeriphConfig(uint32_t MCUDBG_APB1Periph, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        MCUDBG->APB1FZ |= MCUDBG_APB1Periph;
    }
    else
    {
        MCUDBG->APB1FZ &= ~MCUDBG_APB1Periph;
    }
}

/**
  * @brief  Configure APB2 peripheral behavior when the MCU is in Debug mode.
  * @param  MCUDBG_APB2Periph:  APB2 peripheral.
  *   This parameter can be any combination of the following values:
  *     @arg MCUDBG_RTC_HOLD: Hold RTC Calendar and Wakeup counter when Core is halted.
  *     @arg MCUDBG_TIMER15_HOLD: Hold TIMER15 counter when Core is halted
  *     @arg MCUDBG_TIMER16_HOLD: Hold TIMER16 counter when Core is halted
  *     @arg MCUDBG_TIMER17_HOLD: Hold TIMER17 counter when Core is halted
  * @param  NewValue: new value of the specified APB2 peripheral in Debug mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void MCUDBG_APB2PeriphConfig(uint32_t MCUDBG_APB2Periph, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        MCUDBG->APB2FZ |= MCUDBG_APB2Periph;
    }
    else
    {
        MCUDBG->APB2FZ &= ~MCUDBG_APB2Periph;
    }
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

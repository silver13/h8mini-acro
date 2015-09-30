/**
  ******************************************************************************
  * @file    gd32f1x0_mcudbg.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   MCUDBG header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_MCUDBG_H
#define __GD32F1X0_MCUDBG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup MCUDBG
  * @{
  */

/** @defgroup MCUDBG_Exported_Constants
  * @{
  */
/* MCUDBG_CTLR1 */
#define MCUDBG_SLEEP_HOLD               ((uint32_t)0x00000001)
#define MCUDBG_DEEPSLEEP_HOLD           ((uint32_t)0x00000002)
#define MCUDBG_STANDBY_HOLD             ((uint32_t)0x00000004)
#define MCUDBG_IWDG_HOLD                ((uint32_t)0x00000100)
#define MCUDBG_WWDG_HOLD                ((uint32_t)0x00000200)
#define MCUDBG_TIMER1_HOLD              ((uint32_t)0x00000400)
#define MCUDBG_TIMER2_HOLD              ((uint32_t)0x00000800)
#define MCUDBG_TIMER3_HOLD              ((uint32_t)0x00001000)
#define MCUDBG_I2C1_HOLD                ((uint32_t)0x00008000)
#define MCUDBG_I2C2_HOLD                ((uint32_t)0x00010000)
#define MCUDBG_I2C3_HOLD                ((uint32_t)0x00020000)
#define MCUDBG_TIMER6_HOLD              ((uint32_t)0x00080000)
#define MCUDBG_TIMER14_HOLD             ((uint32_t)0x08000000)
/* MCUDBG_CTLR2 */
#define MCUDBG_RTC_HOLD                 ((uint32_t)0x00000400)
#define MCUDBG_TIMER15_HOLD             ((uint32_t)0x00010000)   
#define MCUDBG_TIMER16_HOLD             ((uint32_t)0x00020000)   
#define MCUDBG_TIMER17_HOLD             ((uint32_t)0x00040000)

/**
  * @}
  */ 

/** @defgroup DBGMCU_Exported_Functions
  * @{
  */
  
uint32_t MCUDBG_GetREVID(void);
uint32_t MCUDBG_GetDEVID(void);
void MCUDBG_APB1PeriphConfig(uint32_t MCUDBG_APB1Periph, TypeState NewValue);
void MCUDBG_APB2PeriphConfig(uint32_t MCUDBG_APB2Periph, TypeState NewValue);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_MCUDBG_H */

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

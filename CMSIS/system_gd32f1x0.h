/**
  ******************************************************************************
  * @file    system_gd32f1x0.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup GD32F1x0_system
  * @{
  */  
  
/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_GD32F1X0_H
#define __SYSTEM_GD32F1X0_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup GD32F1x0_System_Includes
  * @{
  */

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Exported_types
  * @{
  */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Exported_Functions
  * @{
  */
extern void System_Init(void);
extern void SystemCoreClock_Update(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_GD32F1X0_H */

/**
  * @}
  */
  
/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

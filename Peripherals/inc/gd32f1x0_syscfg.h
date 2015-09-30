/**
  ******************************************************************************
  * @file    gd32f1x0_syscfg.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   SYSCFG header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_SYSCFG_H
#define __GD32F1X0_SYSCFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup SYSCFG
  * @{
  */

/** @defgroup SYSCFG_Exported_Constants
  * @{
  */ 

/** @defgroup SYSCFG_EXTI_SOURCE_Port_Def
  * @{
  */ 
#define EXTI_SOURCE_GPIOA                               ((uint8_t)0x00)
#define EXTI_SOURCE_GPIOB                               ((uint8_t)0x01)
#define EXTI_SOURCE_GPIOC                               ((uint8_t)0x02)
#define EXTI_SOURCE_GPIOD                               ((uint8_t)0x03)
#define EXTI_SOURCE_GPIOF                               ((uint8_t)0x05)

/**
  * @}
  */

/** @defgroup SYSCFG_EXTI_SOURCE_Pin_Def
  * @{
  */ 
#define EXTI_SOURCE_PIN0                                ((uint8_t)0x00)
#define EXTI_SOURCE_PIN1                                ((uint8_t)0x01)
#define EXTI_SOURCE_PIN2                                ((uint8_t)0x02)
#define EXTI_SOURCE_PIN3                                ((uint8_t)0x03)
#define EXTI_SOURCE_PIN4                                ((uint8_t)0x04)
#define EXTI_SOURCE_PIN5                                ((uint8_t)0x05)
#define EXTI_SOURCE_PIN6                                ((uint8_t)0x06)
#define EXTI_SOURCE_PIN7                                ((uint8_t)0x07)
#define EXTI_SOURCE_PIN8                                ((uint8_t)0x08)
#define EXTI_SOURCE_PIN9                                ((uint8_t)0x09)
#define EXTI_SOURCE_PIN10                               ((uint8_t)0x0A)
#define EXTI_SOURCE_PIN11                               ((uint8_t)0x0B)
#define EXTI_SOURCE_PIN12                               ((uint8_t)0x0C)
#define EXTI_SOURCE_PIN13                               ((uint8_t)0x0D)
#define EXTI_SOURCE_PIN14                               ((uint8_t)0x0E)
#define EXTI_SOURCE_PIN15                               ((uint8_t)0x0F)

/**
  * @}
  */

/** @defgroup SYSCFG_BOOT_Mode_Def
  * @{
  */ 
#define SYSCFG_FLASH_BOOT                               ((uint32_t)0x00)
#define SYSCFG_SYSTEMMEMORY_BOOT                        SYSCFG_R1_BOOT_MODE_0
#define SYSCFG_SRAM_BOOT                                (SYSCFG_R1_BOOT_MODE_0 | SYSCFG_R1_BOOT_MODE_1)

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup SYSCFG_Exported_Functions
  * @{
  */
void SYSCFG_DeInit(void);
void SYSCFG_DMARemap_Config(uint32_t SYSCFG_dma_remap, TypeState NewValue);
void SYSCFG_HighCurrent_Enable(TypeState NewValue);
void SYSCFG_EXTILine_Config(uint8_t exti_port, uint8_t exti_pin);
void SYSCFG_Lock_Config(uint32_t SYSCFG_lock);
TypeState SYSCFG_GetBitState(uint32_t SYSCFG_flag);
void SYSCFG_ClearBitState(uint32_t SYSCFG_flag);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__GD32F1X0_SYSCFG_H */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

/**
  ******************************************************************************
  * @file    gd32f1x0_gpio.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   GPIO header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_GPIO_H
#define __GD32F1X0_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/** @defgroup GPIO_Exported_Types
  * @{
  */

/** @defgroup GPIO_Mode_enumeration 
  * @{
  */
typedef enum
{
    GPIO_MODE_IN   = 0x00,
    GPIO_MODE_OUT  = 0x01,
    GPIO_MODE_AF   = 0x02,
    GPIO_MODE_AN   = 0x03
}GPIO_ModePara;

/**
  * @}
  */

/** @defgroup Output_type_enumeration
  * @{
  */
typedef enum
{
    GPIO_OTYPE_PP = 0x00,
    GPIO_OTYPE_OD = 0x01
}GPIO_OTypePara;

/**
  * @}
  */

/** @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
typedef enum
{
    GPIO_SPEED_10MHZ  = 0x01,
    GPIO_SPEED_2MHZ   = 0x02,
    GPIO_SPEED_50MHZ  = 0x03
}GPIO_SpeedPara;

/**
  * @}
  */

/** @defgroup GPIO_Pull-Up_Pull-Down_enumeration 
  * @{
  */
typedef enum
{
    GPIO_PUPD_NOPULL     = 0x00,
    GPIO_PUPD_PULLUP     = 0x01,
    GPIO_PUPD_PULLDOWN   = 0x02
}GPIO_PuPdPara;

/**
  * @}
  */

/** @defgroup Bit_State_enumeration
  * @{
  */
typedef enum
{ 
    Bit_RESET = 0,
    Bit_SET
}BitState;

/**
  * @}
  */

/** 
  * @brief  GPIO Initial Parameters
  */
typedef struct
{
    uint32_t GPIO_Pin;              /*!< The GPIO pins to be configured. choose several from @ref GPIO_pins_define */
    GPIO_ModePara GPIO_Mode;        /*!< The operating mode for the selected pins. choose one from @ref GPIO_ModePara   */
    GPIO_SpeedPara GPIO_Speed;      /*!< The speed for the selected pins.choose one from @ref GPIO_SpeedPara  */
    GPIO_OTypePara GPIO_OType;      /*!< The operating output type for the selected pins.choose one from @ref GPIO_OTypePara  */
    GPIO_PuPdPara GPIO_PuPd;        /*!< The operating Pull-up/Pull down for the selected pins.choose one from @ref GPIO_PuPdPara   */
}GPIO_InitPara;

/**
  * @}
  */

/** @defgroup GPIO_Exported_Constants
  * @{
  */

/** @defgroup GPIO_pins_define 
  * @{
  */
#define GPIO_PIN_0                  ((uint16_t)0x0001)
#define GPIO_PIN_1                  ((uint16_t)0x0002)
#define GPIO_PIN_2                  ((uint16_t)0x0004)
#define GPIO_PIN_3                  ((uint16_t)0x0008)
#define GPIO_PIN_4                  ((uint16_t)0x0010)
#define GPIO_PIN_5                  ((uint16_t)0x0020)
#define GPIO_PIN_6                  ((uint16_t)0x0040)
#define GPIO_PIN_7                  ((uint16_t)0x0080)
#define GPIO_PIN_8                  ((uint16_t)0x0100)
#define GPIO_PIN_9                  ((uint16_t)0x0200)
#define GPIO_PIN_10                 ((uint16_t)0x0400)
#define GPIO_PIN_11                 ((uint16_t)0x0800)
#define GPIO_PIN_12                 ((uint16_t)0x1000)
#define GPIO_PIN_13                 ((uint16_t)0x2000)
#define GPIO_PIN_14                 ((uint16_t)0x4000)
#define GPIO_PIN_15                 ((uint16_t)0x8000)
#define GPIO_PIN_ALL                ((uint16_t)0xFFFF)

/**
  * @}
  */

/** @defgroup GPIO_Pin_sources 
  * @{
  */
#define GPIO_PINSOURCE0             ((uint8_t)0x00)
#define GPIO_PINSOURCE1             ((uint8_t)0x01)
#define GPIO_PINSOURCE2             ((uint8_t)0x02)
#define GPIO_PINSOURCE3             ((uint8_t)0x03)
#define GPIO_PINSOURCE4             ((uint8_t)0x04)
#define GPIO_PINSOURCE5             ((uint8_t)0x05)
#define GPIO_PINSOURCE6             ((uint8_t)0x06)
#define GPIO_PINSOURCE7             ((uint8_t)0x07)
#define GPIO_PINSOURCE8             ((uint8_t)0x08)
#define GPIO_PINSOURCE9             ((uint8_t)0x09)
#define GPIO_PINSOURCE10            ((uint8_t)0x0A)
#define GPIO_PINSOURCE11            ((uint8_t)0x0B)
#define GPIO_PINSOURCE12            ((uint8_t)0x0C)
#define GPIO_PINSOURCE13            ((uint8_t)0x0D)
#define GPIO_PINSOURCE14            ((uint8_t)0x0E)
#define GPIO_PINSOURCE15            ((uint8_t)0x0F)

/**
  * @}
  */

/** @defgroup GPIO_Alternate_function_selection_define 
  * @{
  */

#define GPIO_AF_0                   ((uint8_t)0x00) 
#define GPIO_AF_1                   ((uint8_t)0x01)
#define GPIO_AF_2                   ((uint8_t)0x02)
#define GPIO_AF_3                   ((uint8_t)0x03)
#define GPIO_AF_4                   ((uint8_t)0x04)
#define GPIO_AF_5                   ((uint8_t)0x05)
#define GPIO_AF_6                   ((uint8_t)0x06)
#define GPIO_AF_7                   ((uint8_t)0x07)

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions
  * @{
  */
void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitPara* GPIO_InitParaStruct);
void GPIO_ParaInit(GPIO_InitPara* GPIO_InitParaStruct);
void GPIO_PinLock(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint8_t GPIO_ReadInputBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitState BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_GPIO_H */

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

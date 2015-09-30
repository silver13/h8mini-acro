/**
  ******************************************************************************
  * @file    gd32f1x0_tsi.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   TSI firmware library header file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_TSI_H
#define __GD32F1X0_TSI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup TSI
  * @{
  */

/** @defgroup TSI_Exported_Types
  * @{
  */

/** 
  * @brief  TSI Group Enum
  */
typedef enum
{
    TSI_GROUP1,                 /*!< Group 1 */
    TSI_GROUP2,                 /*!< Group 2 */
    TSI_GROUP3,                 /*!< Group 3 */
    TSI_GROUP4,                 /*!< Group 4 */
    TSI_GROUP5,                 /*!< Group 5 */
    TSI_GROUP6,                 /*!< Group 6 */
}TSI_GROUP_TypeDef;


/** 
  * @brief  TSI Pin Enum 
  */
typedef enum
{
    TSI_PIN1,                   /*!< Pin 1 */
    TSI_PIN2,                   /*!< Pin 2 */
    TSI_PIN3,                   /*!< Pin 3 */
    TSI_PIN4,                   /*!< Pin 4 */
}TSI_PIN_TypeDef;

/** 
  * @brief  TSI Spread Spectrum Prescaler
  */
typedef enum
{
    ECCLK_DIV_0  = 0x01,        /*!< fHCLK   */
    ECCLK_DIV_1  = 0x02,        /*!< fHCLK/2 */
}ECCLKDIV_TypeDef;

/** 
  * @brief  TSI Init structure definition  
  */

typedef struct
{
    uint32_t              TSI_TriggerMode;        /*!< Trigger mode selection, 
                                                         a value of @ref TSI_Trigger_Mode. */
    uint32_t              TSI_EageSelect;         /*!< Trigger Edge selection, 
                                                         a value of @ref TSI_Trigger_Edge. */
    uint32_t              TSI_MaxCycle;           /*!< Max cycle number of a sequence , 
                                                         a value of @ref TSI_Max_Cycle. */
    uint32_t              TSI_PulsePrescaler;     /*!< CTCLK clock division factor, 
                                                         a value of @ref Pulse_Generator_Prescaler. */
    uint32_t              TSI_pulseLow;           /*!< Charge Transfer State Duration Time, 
                                                         a value of @ref Charge_Transfer_State_Duration_Time. */
    uint32_t              TSI_pulseHigh;          /*!< Charge State Duration Time, 
                                                         a value of @ref Charge_State_Duration_Time. */
} TSI_BaseInitPara;

/** 
  * @brief  TSI Group Init structure definition  
  */
typedef struct
{
    TSI_GROUP_TypeDef     TSI_Group;              /*!< Group to init , 
                                                          a value of @ref TSI Group Enum TSI Group Enum. */
    uint32_t              TSI_SamplePin;          /*!< Selected the pin as Sample pin in a group , 
                                                          a value of @ref TSI_Group_Channel_define. */     
    uint32_t              TSI_ChannelPin;         /*!< Selected the pin as Channel pin in a group , 
                                                          a value of @ref TSI_Group_Channel_define . */
}TSI_GroupInitPara;

/** 
  * @brief  TSI SpreadSpectrum Init structure definition  
  */
typedef struct
{
    uint32_t              TSI_SSPrescaler;        /*!< Spread spectrum prescaler, 
                                                           a value of @ref TSI Spread Spectrum Prescaler . */     
    uint32_t              TSI_SSDeviation;        /*!< Spread spectrum deviation, 
                                                           a value of @ref TSI_Extend_Charge_State_Maximum_Duration_Time . */
}TSI_SpreadSpectrumInitPara ;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup TSI_Exported_Constants
  * @{
  */

/** @defgroup TSI_Group_Channel_define 
  * @{
  */

#define TSI_GROUP1_PIN1                           ((uint32_t)0x00000001)   /*!< Group 1 PIN 1 */ 
#define TSI_GROUP1_PIN2                           ((uint32_t)0x00000002)   /*!< Group 1 PIN 2 */ 
#define TSI_GROUP1_PIN3                           ((uint32_t)0x00000004)   /*!< Group 1 PIN 3 */ 
#define TSI_GROUP1_PIN4                           ((uint32_t)0x00000008)   /*!< Group 1 PIN 4 */ 
                                                  
#define TSI_GROUP2_PIN1                           ((uint32_t)0x00000010)   /*!< Group 2 PIN 1 */ 
#define TSI_GROUP2_PIN2                           ((uint32_t)0x00000020)   /*!< Group 2 PIN 2 */ 
#define TSI_GROUP2_PIN3                           ((uint32_t)0x00000040)   /*!< Group 2 PIN 3 */ 
#define TSI_GROUP2_PIN4                           ((uint32_t)0x00000080)   /*!< Group 2 PIN 4 */ 
                                                  
#define TSI_GROUP3_PIN1                           ((uint32_t)0x00000100)   /*!< Group 3 PIN 1 */ 
#define TSI_GROUP3_PIN2                           ((uint32_t)0x00000200)   /*!< Group 3 PIN 2 */ 
#define TSI_GROUP3_PIN3                           ((uint32_t)0x00000400)   /*!< Group 3 PIN 3 */ 
#define TSI_GROUP3_PIN4                           ((uint32_t)0x00000800)   /*!< Group 3 PIN 4 */ 
                                                  
#define TSI_GROUP4_PIN1                           ((uint32_t)0x00001000)   /*!< Group 4 PIN 1 */ 
#define TSI_GROUP4_PIN2                           ((uint32_t)0x00002000)   /*!< Group 4 PIN 2 */ 
#define TSI_GROUP4_PIN3                           ((uint32_t)0x00004000)   /*!< Group 4 PIN 3 */ 
#define TSI_GROUP4_PIN4                           ((uint32_t)0x00008000)   /*!< Group 4 PIN 4 */ 
                                                  
#define TSI_GROUP5_PIN1                           ((uint32_t)0x00010000)   /*!< Group 5 PIN 1 */ 
#define TSI_GROUP5_PIN2                           ((uint32_t)0x00020000)   /*!< Group 5 PIN 2 */ 
#define TSI_GROUP5_PIN3                           ((uint32_t)0x00040000)   /*!< Group 5 PIN 3 */ 
#define TSI_GROUP5_PIN4                           ((uint32_t)0x00080000)   /*!< Group 5 PIN 4 */ 
                                                  
#define TSI_GROUP6_PIN1                           ((uint32_t)0x00100000)   /*!< Group 6 PIN 1 */ 
#define TSI_GROUP6_PIN2                           ((uint32_t)0x00200000)   /*!< Group 6 PIN 2 */ 
#define TSI_GROUP6_PIN3                           ((uint32_t)0x00400000)   /*!< Group 6 PIN 3 */ 
#define TSI_GROUP6_PIN4                           ((uint32_t)0x00800000)   /*!< Group 6 PIN 4 */ 

/**
  * @}
  */


/** @defgroup  TSI_Max_Cycle
  * @{
  */

#define TSI_MCN_255_CYCLE                         ((uint32_t)0x00000000)
#define TSI_MCN_511_CYCLE                         ((uint32_t)0x00000020)
#define TSI_MCN_1023_CYCLE                        ((uint32_t)0x00000040)
#define TSI_MCN_2047_CYCLE                        ((uint32_t)0x00000060)
#define TSI_MCN_4095_CYCLE                        ((uint32_t)0x00000080)
#define TSI_MCN_8191_CYCLE                        ((uint32_t)0x000000A0)
#define TSI_MCN_16383_CYCLE                       ((uint32_t)0x000000C0)

/**
  * @}
  */

/** @defgroup  Pulse_Generator_Prescaler
  * @{
  */

#define TSI_CTCDIV_1                              ((uint32_t)0x00000000)
#define TSI_CTCDIV_2                              ((uint32_t)0x00001000)
#define TSI_CTCDIV_4                              ((uint32_t)0x00002000)
#define TSI_CTCDIV_8                              ((uint32_t)0x00003000)
#define TSI_CTCDIV_16                             ((uint32_t)0x00004000)
#define TSI_CTCDIV_32                             ((uint32_t)0x00005000)
#define TSI_CTCDIV_64                             ((uint32_t)0x00006000)
#define TSI_CTCDIV_128                            ((uint32_t)0x00007000)

/**
  * @}
  */

/** @defgroup  TSI_Extend_Charge_State_Maximum_Duration_Time
  * @{
  */
#define TSI_ECDT_1ECCLK                           ((uint32_t)0x00000000) 
#define TSI_ECDT_2ECCLK                           ((uint32_t)0x00020000) 
#define TSI_ECDT_3ECCLK                           ((uint32_t)0x00040000) 
#define TSI_ECDT_4ECCLK                           ((uint32_t)0x00060000) 
#define TSI_ECDT_5ECCLK                           ((uint32_t)0x00080000) 
#define TSI_ECDT_6ECCLK                           ((uint32_t)0x000A0000) 
#define TSI_ECDT_7ECCLK                           ((uint32_t)0x000C0000) 
#define TSI_ECDT_8ECCLK                           ((uint32_t)0x000E0000) 
#define TSI_ECDT_9ECCLK                           ((uint32_t)0x00100000) 
#define TSI_ECDT_10ECCLK                          ((uint32_t)0x00110000) 
#define TSI_ECDT_16ECCLK                          ((uint32_t)0x001E0000) 
#define TSI_ECDT_32ECCLK                          ((uint32_t)0x003E0000) 
#define TSI_ECDT_64ECCLK                          ((uint32_t)0x007E0000) 
#define TSI_ECDT_128ECCLK                         ((uint32_t)0x00FE0000) 

/**
  * @}
  */

/** @defgroup  Charge_Transfer_State_Duration_Time
  * @{
  */
#define TSI_CTDT_1CTCLK                           ((uint32_t)0x00000000)  
#define TSI_CTDT_2CTCLK                           ((uint32_t)0x01000000)  
#define TSI_CTDT_3CTCLK                           ((uint32_t)0x02000000)  
#define TSI_CTDT_4CTCLK                           ((uint32_t)0x03000000)  
#define TSI_CTDT_5CTCLK                           ((uint32_t)0x04000000)  
#define TSI_CTDT_6CTCLK                           ((uint32_t)0x05000000)  
#define TSI_CTDT_7CTCLK                           ((uint32_t)0x06000000)  
#define TSI_CTDT_8CTCLK                           ((uint32_t)0x07000000)  
#define TSI_CTDT_9CTCLK                           ((uint32_t)0x08000000)  
#define TSI_CTDT_10CTCLK                          ((uint32_t)0x09000000)  
#define TSI_CTDT_11CTCLK                          ((uint32_t)0x0A000000)  
#define TSI_CTDT_12CTCLK                          ((uint32_t)0x0B000000)  
#define TSI_CTDT_13CTCLK                          ((uint32_t)0x0C000000)  
#define TSI_CTDT_14CTCLK                          ((uint32_t)0x0D000000)  
#define TSI_CTDT_15CTCLK                          ((uint32_t)0x0E000000)  
#define TSI_CTDT_16CTCLK                          ((uint32_t)0x0F000000)  

/**
  * @}
  */
                                                           
/** @defgroup  Charge_State_Duration_Time
  * @{
  */
#define TSI_CDT_1CTCLK                            ((uint32_t)0x00000000)
#define TSI_CDT_2CTCLK                            ((uint32_t)0x10000000)
#define TSI_CDT_3CTCLK                            ((uint32_t)0x20000000)
#define TSI_CDT_4CTCLK                            ((uint32_t)0x30000000)
#define TSI_CDT_5CTCLK                            ((uint32_t)0x40000000)
#define TSI_CDT_6CTCLK                            ((uint32_t)0x50000000)
#define TSI_CDT_7CTCLK                            ((uint32_t)0x60000000)
#define TSI_CDT_8CTCLK                            ((uint32_t)0x70000000)
#define TSI_CDT_9CTCLK                            ((uint32_t)0x80000000)
#define TSI_CDT_10CTCLK                           ((uint32_t)0x90000000)
#define TSI_CDT_11CTCLK                           ((uint32_t)0xA0000000)
#define TSI_CDT_12CTCLK                           ((uint32_t)0xB0000000)
#define TSI_CDT_13CTCLK                           ((uint32_t)0xC0000000)
#define TSI_CDT_14CTCLK                           ((uint32_t)0xD0000000)
#define TSI_CDT_15CTCLK                           ((uint32_t)0xE0000000)
#define TSI_CDT_16CTCLK                           ((uint32_t)0xF0000000)
                                                          
/**                                                       
  * @}                                                    
  */                                                      

/** @defgroup TSI_Trigger_Mode
  * @{
  */
#define TSI_SOFTWARE_TRIGGLER                     ((uint32_t)0x00000000)   /*!< Software Trigger Mode . */ 
#define TSI_HARDWARE_TRIGGLER                     TSI_CTLR_TM              /*!< Hardware Trigger Mode . */       

/**
  * @}
  */


/** @defgroup  TSI_Trigger_Edge
  * @{
  */

#define TSI_RISING_EDGE_TRIGGLER                  ((uint32_t)0x00000000)   /*!< Rising edge and high level Trigger  . */ 
#define TSI_FALLING_EDGE_TRIGGLER                 TSI_CTLR_ES              /*!< Falling edge Trigger only . */       

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup TSI_Exported_Functions
  * @{
  */

/* TSI management *************************************************************/
void TSI_DeInit( void );
void TSI_BaseInit( TSI_BaseInitPara* TSI_BaseInitParaStruct );
void TSI_BaseStructInit( TSI_BaseInitPara * TSI_BaseInitParaStruct );
void TSI_GroupInit( TSI_GroupInitPara * TSI_GroupInitStruct );
void TSI_SpreadSpectrumInit( TSI_SpreadSpectrumInitPara * TSI_SpreadSpectrumInitParaStruct );
void TSI_Enable( TypeState NewValue );
void TSI_StartAcquisition( TypeState NewValue );
void TSI_TriggerModeConfig( TypeState TSI_TriggerMode );
void TSI_TriggerEdgeConfig( TypeState TSI_TriggerEdge );
void TSI_IOModeConfig( TypeState NewValue );
void TSI_SetMaxCycle( uint32_t TSI_MCN );
void TSI_PulseGenDIV( uint32_t TSI_CTCDIV );
void TSI_ECCLKDivConfig( ECCLKDIV_TypeDef TSI_ECDIV );
void TSI_ExtendChargeConfig( TypeState NewValue );
void TSI_ExtendChargeMaxTime( uint32_t TSI_ECDT );
void TSI_SetChargeTransferTime( uint32_t TSI_CTDT );
void TSI_SetChargeTime( uint32_t TSI_CDT );

void TSI_INTConfig( uint8_t TSI_INT, TypeState NewValue );
void TSI_ClearIntBitState( uint8_t TSI_INT );
TypeState TSI_GetIntBitState(uint8_t TSI_FLAG);

/* TSI Pin management ***************************************************/
void TSI_PinHysteresisConfig( uint32_t TSI_Pin , TypeState NewValue );
void TSI_AnalogModeConfig( uint32_t TSI_Pin , TypeState NewValue );
void TSI_SampleModeConfig( uint32_t TSI_Pin , TypeState NewValue );
void TSI_ChannelModeConfig( uint32_t TSI_Pin , TypeState NewValue );

/* TSI Group management ***************************************************/
void TSI_GroupSampleConfig( TSI_GROUP_TypeDef TSI_GROUP , TypeState NewValue );
TypeState TSI_GetGroupSampleState( TSI_GROUP_TypeDef TSI_GROUP );
uint16_t  TSI_GetGroupSampleCycle( TSI_GROUP_TypeDef TSI_GROUP );

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_TSI_H */

/**
  * @}
  */

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

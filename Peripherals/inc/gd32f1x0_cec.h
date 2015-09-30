/**
  ******************************************************************************
  * @file    gd32f1x0_cec.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CEC header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_CEC_H
#define __GD32F1X0_CEC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup CEC
  * @{
  */

/** @defgroup CEC_Exported_Types
  * @{
  */

/** 
  * @brief  CEC Initial Parameters
  */

typedef struct
{
    uint32_t CEC_SFT;                   /*!< The CEC Signal Free Time, choose one from @ref CEC_Signal_Free_Time */
    uint32_t CEC_RXTOL;                 /*!< The CEC Reception Tolerance, choose one from @ref CEC_RxTolerance */
    uint32_t CEC_BRESTOP;               /*!< Stop reception when detected BRE, choose one from @ref CEC_BRE_Stop_Reception */
    uint32_t CEC_BREGEN;                /*!< Generate Error-Bit when detected BRE, choose one from @ref CEC_Bit_Rising_Error_Generation */
    uint32_t CEC_LBPEGEN;               /*!< Generate Error-Bit when detected LBPE, choose one from @ref CEC_Long_Bit_Error_Generation */
    uint32_t CEC_BCNG;                  /*!< Do not generate Error-Bit in broadcast, choose one from @ref CEC_Broadcast_No_Generation */
    uint32_t CEC_SFTOPT;                /*!< The CEC Signal Free Time option, choose one from @ref CEC_SFT_Option */
}CEC_InitPara;

/**
  * @}
  */
  
/** @defgroup CEC_Exported_Constants
  * @{
  */

/** @defgroup CEC_Signal_Free_Time
  * @{
  */
#define CEC_SFT_STD                     ((uint32_t)0x00000000)  /*!< Signal Free Time Standard   */
#define CEC_SFT_1T                      ((uint32_t)0x00000001)  /*!< SFT 1.5 data bit periods    */
#define CEC_SFT_2T                      ((uint32_t)0x00000002)  /*!< SFT 2.5 data bit periods    */
#define CEC_SFT_3T                      ((uint32_t)0x00000003)  /*!< SFT 3.5 data bit periods    */
#define CEC_SFT_4T                      ((uint32_t)0x00000004)  /*!< SFT 4.5 data bit periods    */
#define CEC_SFT_5T                      ((uint32_t)0x00000005)  /*!< SFT 5.5 data bit periods    */
#define CEC_SFT_6T                      ((uint32_t)0x00000006)  /*!< SFT 6.5 data bit periods    */
#define CEC_SFT_7T                      ((uint32_t)0x00000007)  /*!< SFT 7.5 data bit periods    */

/**
  * @}
  */

/** @defgroup CEC_RxTolerance
  * @{
  */
#define CEC_RXTOL_STD                   ((uint32_t)0x00000000)  /*!< Standard bit timing tolerance        */
#define CEC_RXTOL_EXD                   CEC_SR_RXTOL            /*!< Extended bit timing tolerance        */

/**
  * @}
  */

/** @defgroup CEC_BRE_Stop_Reception
  * @{
  */
#define CEC_BRESTOP_OFF                 ((uint32_t)0x00000000)  /*!< Do not stop reception on bit Rising Error   */
#define CEC_BRESTOP_ON                  CEC_SR_BRESTP           /*!< Stop reception on bit Rising Error          */

/**
  * @}
  */

/** @defgroup CEC_Bit_Rising_Error_Generation
  * @{
  */
#define CEC_BREGEN_OFF                  ((uint32_t)0x00000000)  /*!< Do not generate Error-Bit on bit Rising Error */
#define CEC_BREGEN_ON                   CEC_SR_BREGEN           /*!< Generate Error-Bit on bit Rising Error        */

/**
  * @}
  */

/** @defgroup CEC_Long_Bit_Error_Generation
  * @{
  */
#define CEC_LBPEGEN_OFF                 ((uint32_t)0x00000000)  /*!< Do not generate Error-Bit on Long Bit Period Error */
#define CEC_LBPEGEN_ON                  CEC_SR_LBPEGEN          /*!< Generate Error-Bit on Long Bit Period Error        */

/**
  * @}
  */

/** @defgroup CEC_Broadcast_No_Generation
  * @{
  */

#define CEC_BCNG_OFF                    ((uint32_t)0x00000000)  /*!< Generate Error-Bit in Broadcast         */
#define CEC_BCNG_ON                     CEC_SR_BCNG             /*!< Do not generate Error-Bit in Broadcast  */

/**
  * @}
  */

/** @defgroup CEC_SFT_Option
  * @{
  */
#define CEC_SFTOPT_SOM                  ((uint32_t)0x00000000)  /*!< SFT counter starts when SOM is set                   */
#define CEC_SFTOPT_LAST                 CEC_SR_SFTOPT           /*!< SFT counter starts after the end of the last message */

/**
  * @}
  */

/** @defgroup CEC_Interrupt_Configuration_definition
  * @{
  */
#define CEC_INT_TAE                     CEC_IER_TAEIE
#define CEC_INT_TE                      CEC_IER_TEIE
#define CEC_INT_TU                      CEC_IER_TUIE
#define CEC_INT_TEND                    CEC_IER_TENDIE
#define CEC_INT_TBR                     CEC_IER_TBRIE
#define CEC_INT_ARBLST                  CEC_IER_ARBLSTIE
#define CEC_INT_RAE                     CEC_IER_RAEIE
#define CEC_INT_LBPE                    CEC_IER_LBPEIE
#define CEC_INT_SBPE                    CEC_IER_SBPEIE
#define CEC_INT_BRE                     CEC_IER_BREIE
#define CEC_INT_RO                      CEC_IER_ROIE
#define CEC_INT_REND                    CEC_IER_RENDIE
#define CEC_INT_RBR                     CEC_IER_RBRIE

/**
  * @}
  */

/** @defgroup CEC_ISTR_register_flags_definition
  * @{
  */
#define CEC_FLAG_TAE                    CEC_ISTR_TAE
#define CEC_FLAG_TE                     CEC_ISTR_TE
#define CEC_FLAG_TU                     CEC_ISTR_TU
#define CEC_FLAG_TEND                   CEC_ISTR_TEND
#define CEC_FLAG_TBR                    CEC_ISTR_TBR
#define CEC_FLAG_ARBLST                 CEC_ISTR_ARBLST
#define CEC_FLAG_RAE                    CEC_ISTR_RAE
#define CEC_FLAG_LBPE                   CEC_ISTR_LBPE
#define CEC_FLAG_SBPE                   CEC_ISTR_SBPE
#define CEC_FLAG_BRE                    CEC_ISTR_BRE
#define CEC_FLAG_RO                     CEC_ISTR_RO
#define CEC_FLAG_REND                   CEC_ISTR_REND
#define CEC_FLAG_RBR                    CEC_ISTR_RBR

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup CEC_Exported_Functions
  * @{
  */

void CEC_DeInit(void);
void CEC_Init(CEC_InitPara* CEC_InitParaStruct);
void CEC_ParaInit(CEC_InitPara* CEC_InitParaStruct);
void CEC_Enable(TypeState NewValue);
void CEC_ListenModeEnable(TypeState NewValue);
void CEC_SetOwnAddress(uint8_t CEC_OwnAddress);
void CEC_ClearAllOwnAddress(void);
void CEC_SendData(uint8_t Data);
uint8_t CEC_ReceiveData(void);
void CEC_StartOfMessage(void);
void CEC_EndOfMessage(void);
void CEC_INTConfig(uint32_t CEC_INT, TypeState NewValue);
TypeState CEC_GetBitState(uint32_t CEC_FLAG);
void CEC_ClearBitState(uint32_t CEC_FLAG);
TypeState CEC_GetIntBitState(uint32_t CEC_INT);
void CEC_ClearIntBitState(uint32_t CEC_INT);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_CEC_H */

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

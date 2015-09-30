/**
  ******************************************************************************
  * @file    gd32f1x0_usart.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USART header file of the firmware library.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USART_H
#define __GD32F1X0_USART_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USART
  * @{
  */

/** @defgroup USART_Exported_Types
  * @{
  */

/** 
  * @brief USART Initial Parameters
  */
typedef struct
{
    uint32_t USART_BRR;                   /*!< the USART communication baud rate configuration.
                                             The baud-rate divider (USARTDIV) has the following relationship to the system clock: 
                                             In case of oversampling by 16, the equation is:
                                             USARTDIV=  UCLK/(16¡ÁBaud Rate)
                                             In case of oversampling by 8, the equation is:
                                             USARTDIV=  UCLK/(8¡ÁBaud Rate) */
    uint32_t USART_WL;                    /*!< Set by USART_CTLR1_WL   Word length 0: 8 Data bits,
                                                                                 1: 9 Data bits */
    uint32_t USART_STBits;                /*!< Stop bits configuration */                         

    uint32_t USART_Parity;                /*!< Set by USART_CTLR1_PCEN */

    uint32_t USART_RxorTx;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled. */

    uint32_t USART_HardwareFlowControl;  /*!< Specifies wether the hardware flow control mode is enabled
                                            or disabled.This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitPara;

/** 
  * @brief  USART Clock Init Structure definition
  */ 

typedef struct
{
    uint32_t USART_CKEN;                  /*!< USART clock enabled this parameter can be a value of @ref USART_CKEN */     
                                         
    uint32_t USART_CPL;                   /*!< Clock polarity of Steady state this parameter can be a value of @ref USART_USART_CKEN */
                                        
    uint32_t USART_CPH;                   /*!< Clock phase this parameter can be a value of @ref USART_Clock_Phase */
                                   
    uint32_t USART_LBCP;                  /*!< Last bit clock pulse this parameter can be a value of @ref USART_Last_Bit */                                                                     
} USART_ClockInitPara;

/**
  * @}
  */
  
/** @defgroup USART_Exported_Constants
  * @{
  */ 

/** @defgroup USART_WL 
  * @{
  */ 
#define USART_WL_8B                                                    ((uint32_t)0x00000000)
#define USART_WL_9B                                                    USART_CTLR1_WL

/**
  * @}
  */ 

/** @defgroup USART_STBits 
  * @{
  */ 
#define USART_STBITS_1                                                 ((uint32_t)0x00000000)
#define USART_STBITS_2                                                 USART_CTLR2_STB_1
#define USART_STBITS_1_5                                               (USART_CTLR2_STB_0 | USART_CTLR2_STB_1)

/**
  * @}
  */ 

/** @defgroup USART_Parity 
  * @{
  */
#define USART_PARITY_RESET                                             ((uint32_t)0x00000000)
#define USART_PARITY_SETEVEN                                           USART_CTLR1_PCEN
#define USART_PARITY_SETODD                                            (USART_CTLR1_PCEN | USART_CTLR1_PM) 

/**
  * @}
  */ 

/** @defgroup USART_RxorTx 
  * @{
  */ 
#define USART_RXORTX_RX                                                USART_CTLR1_REN
#define USART_RXORTX_TX                                                USART_CTLR1_TEN

/**
  * @}
  */ 

/** @defgroup USART_Hardware_Flow_Control 
  * @{
  */ 
#define USART_HARDWAREFLOWCONTROL_NONE                                 ((uint32_t)0x00000000)
#define USART_HARDWAREFLOWCONTROL_RTS                                  USART_CTLR3_RTSEN
#define USART_HARDWAREFLOWCONTROL_CTS                                  USART_CTLR3_CTSEN
#define USART_HARDWAREFLOWCONTROL_RTS_CTS                              (USART_CTLR3_RTSEN | USART_CTLR3_CTSEN)

/**
  * @}
  */ 

/** @defgroup USART_USART_CKEN 
  * @{
  */ 
#define USART_CKEN_RESET                                               ((uint32_t)0x00000000)
#define USART_CKEN_SET                                                 USART_CTLR2_CKEN

/**
  * @}
  */ 

/** @defgroup USART_Clock_Polarity 
  * @{
  */
#define USART_CPL_LOW                                                  ((uint32_t)0x00000000)
#define USART_CPL_HIGH                                                 USART_CTLR2_CPL

/**
  * @}
  */ 

/** @defgroup USART_Clock_Phase
  * @{
  */
#define USART_CPH_1EDGE                                                ((uint32_t)0x00000000)
#define USART_CPH_2EDGE                                                USART_CTLR2_CPH

/**
  * @}
  */

/** @defgroup USART_Last_Bit
  * @{
  */
#define USART_LBCP_DISABLE                                             ((uint32_t)0x00000000)
#define USART_LBCP_ENABLE                                              USART_CTLR2_LBCP

/**
  * @}
  */
  
/** @defgroup USART_DMA_Requests 
  * @{
  */
#define USART_DMAREQ_TX                                                USART_CTLR3_DENT
#define USART_DMAREQ_RX                                                USART_CTLR3_DENR

/**
  * @}
  */ 

/** @defgroup USART_DMA_Recception_Error
  * @{
  */
#define USART_DMAONERROR_ENABLE                                        ((uint32_t)0x00000000)
#define USART_DMAONERROR_DISABLE                                       USART_CTLR3_DDRE

/**
  * @}
  */ 

/** @defgroup USART_MuteMode_WakeUp_methods
  * @{
  */
#define USART_WAKEUP_IDLELINE                                          ((uint32_t)0x00000000)
#define USART_WAKEUP_ADDRESSMARK                                       USART_CTLR1_WM

/**
  * @}
  */

/** @defgroup USART_Address_Detection
  * @{
  */ 
#define USART_ADDRESSLENGTH_4B                                         ((uint32_t)0x00000000)
#define USART_ADDRESSLENGTH_7B                                         USART_CTLR2_ADDM

/**
  * @}
  */ 

/** @defgroup USART_StopMode_WakeUp_methods 
  * @{
  */ 
#define USART_WAKEUPSOURCE_ADDRESSMATCH                                ((uint32_t)0x00000000)
#define USART_WAKEUPSOURCE_STARTBIT                                    USART_CTLR3_WUM_1
#define USART_WAKEUPSOURCE_RBNE                                        (USART_CTLR3_WUM_0 | USART_CTLR3_WUM_1)

/**
  * @}
  */ 

/** @defgroup USART_LIN_Break_Detection_Length 
  * @{
  */
#define USART_LINBREAKDETECTLENGTH_10B                                 ((uint32_t)0x00000000)
#define USART_LINBREAKDETECTLENGTH_11B                                 USART_CTLR2_LBDL

/**
  * @}
  */

/** @defgroup USART_IrDA_Low_Power 
  * @{
  */
#define USART_IRDAMODE_LOWPOWER                                        USART_CTLR3_IRLP
#define USART_IRDAMODE_NORMAL                                          ((uint32_t)0x00000000)

/**
  * @}
  */ 

/** @defgroup USART_DE_Polarity 
  * @{
  */
#define USART_DEPOLARITY_HIGH                                          ((uint32_t)0x00000000)
#define USART_DEPOLARITY_LOW                                           USART_CTLR3_DEP

/**
  * @}
  */ 

/** @defgroup USART_Inversion_Pins 
  * @{
  */
#define USART_INVPIN_TX                                                USART_CTLR2_TINV
#define USART_INVPIN_RX                                                USART_CTLR2_RINV

/**
  * @}
  */ 

/** @defgroup USART_AutoBaudRate_Mode 
  * @{
  */
#define USART_AUTOBAUDRATE_STARTBIT                                    ((uint32_t)0x00000000)
#define USART_AUTOBAUDRATE_FALLINGEDGE                                 USART_CTLR2_ABDM_0

/**
  * @}
  */ 

/** @defgroup USART_OVR_DETECTION
  * @{
  */
#define USART_OVRDETECTION_ENABLE                                      ((uint32_t)0x00000000)
#define USART_OVRDETECTION_DISABLE                                     USART_CTLR3_OVRD

/**
  * @}
  */ 
  
/** @defgroup USART_Cmd 
  * @{
  */
#define USART_REQUEST_ABDCMD                                           USART_CMD_ABDCMD
#define USART_REQUEST_SBKCMD                                           USART_CMD_SBKCMD
#define USART_REQUEST_MMRQ                                             USART_CMD_MMCMD
#define USART_REQUEST_RXFCMD                                           USART_CMD_RXFCMD
#define USART_REQUEST_TXFCMD                                           USART_CMD_TXFCMD

/**
  * @}
  */ 

/** @defgroup USART_Flags 
  * @{
  */
#define USART_FLAG_REA                                                 USART_STR_REA
#define USART_FLAG_TEA                                                 USART_STR_TEA
#define USART_FLAG_WUF                                                 USART_STR_WUF
#define USART_FLAG_RWU                                                 USART_STR_RWU
#define USART_FLAG_SBF                                                 USART_STR_SBF
#define USART_FLAG_AMF                                                 USART_STR_AMF
#define USART_FLAG_BSY                                                 USART_STR_BSY
#define USART_FLAG_ABDF                                                USART_STR_ABDF
#define USART_FLAG_ABDE                                                USART_STR_ABDE
#define USART_FLAG_EBF                                                 USART_STR_EBF
#define USART_FLAG_RTF                                                 USART_STR_RTF
#define USART_FLAG_CTS                                                 USART_STR_CTS 
#define USART_FLAG_CTSF                                                USART_STR_CTSF
#define USART_FLAG_LBDF                                                USART_STR_LBDF
#define USART_FLAG_TBE                                                 USART_STR_TBE
#define USART_FLAG_TC                                                  USART_STR_TC
#define USART_FLAG_RBNE                                                USART_STR_RBNE
#define USART_FLAG_IDLEF                                               USART_STR_IDLEF
#define USART_FLAG_ORE                                                 USART_STR_ORE
#define USART_FLAG_NE                                                  USART_STR_NE
#define USART_FLAG_FE                                                  USART_STR_FE
#define USART_FLAG_PE                                                  USART_STR_PE

/**
  * @}
  */ 

/** @defgroup USART_Interrupt_definition 
  * @brief USART Interrupt definition
  * USART_INT possible values
  * Elements values convention: 0xZZZZYYXX
  *   XX: Position of the corresponding Interrupt
  *   YY: Register index
  *   ZZZZ: Flag position
  * @{
  */
#define USART_INT_WU                                                   ((uint32_t)0x00140316)
#define USART_INT_AM                                                   ((uint32_t)0x0011010E)
#define USART_INT_EB                                                   ((uint32_t)0x000C011B)
#define USART_INT_RT                                                   ((uint32_t)0x000B011A)
#define USART_INT_PE                                                   ((uint32_t)0x00000108)
#define USART_INT_TBE                                                  ((uint32_t)0x00070107)
#define USART_INT_TC                                                   ((uint32_t)0x00060106)
#define USART_INT_RBNE                                                 ((uint32_t)0x00050105)
#define USART_INT_IDLEF                                                ((uint32_t)0x00040104)
#define USART_INT_LBDF                                                 ((uint32_t)0x00080206)
#define USART_INT_CTSF                                                 ((uint32_t)0x0009030A)
#define USART_INT_ERIE                                                  ((uint32_t)0x00000300) 
#define USART_INT_ORE                                                  ((uint32_t)0x00030300)
#define USART_INT_NE                                                   ((uint32_t)0x00020300)
#define USART_INT_FE                                                   ((uint32_t)0x00010300)

/**
  * @}
  */
  
/**
  * @}
  */
  
/** @defgroup USART_Exported_Functions
  * @{
  */
void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitPara* USART_InitParaStruct);
void USART_ParaInit(USART_TypeDef* USARTx,USART_InitPara* USART_InitParaStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitPara* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitPara* USART_ClockInitParaStruct);
void USART_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_TransferDirection_Enable(USART_TypeDef* USARTx, uint32_t USART_Direction, TypeState NewValue);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_OneSamplingBit_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_MSBFirst_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_DataInvert_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_InvPin_Enable(USART_TypeDef* USARTx, uint32_t USART_InvertPin, TypeState NewValue);
void USART_SWPFunction_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_ReceiverTimeOut_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_SetReceiveTimeOut_Threshold(USART_TypeDef* USARTx, uint32_t USART_ReceiverTimeOutThreshold);
void USART_DataSend(USART_TypeDef* USARTx,uint16_t Data);
uint16_t USART_DataReceive(USART_TypeDef* USARTx);
void USART_AutoBaudRateDect_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_AutoBaudRateDectMode_Set(USART_TypeDef* USARTx, uint32_t USART_AutoBaudRate);
void USART_Address(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_MuteMode_Enable(USART_TypeDef* USARTx, TypeState NewState);
void USART_MuteModeWakeUp_Set(USART_TypeDef* USARTx, uint32_t USART_WakeUp);
void USART_AddressDetection_Set(USART_TypeDef* USARTx, uint32_t USART_AddressLength);
void USART_SetLINBDLength(USART_TypeDef* USARTx,uint32_t USART_LINBreakDetectLength);
void USART_LIN_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_HalfDuplex_Enable(USART_TypeDef* USARTx, TypeState NewValue);
void USART_GuardTime_Set(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SmartCard_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_SmartCardNACK_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_AutoRetryNumber_Set(USART_TypeDef* USARTx,uint8_t USART_AutoNumber);
void USART_BlockLength_Set(USART_TypeDef* USARTx,uint8_t USART_BlockLength);
void USART_IrDA_Set(USART_TypeDef* USARTx,uint32_t USART_IrDAMode);
void USART_IrDA_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_DE_Enable(USART_TypeDef* USARTx,TypeState NewValue);
void USART_DriverEnablePolarity_Set(USART_TypeDef* USARTx, uint32_t USART_DEPly);
void USART_DEATime_Set(USART_TypeDef* USARTx, uint32_t USART_DEATime);
void USART_DEDTime_Set(USART_TypeDef* USARTx, uint32_t USART_DEDTime);
void USART_DMA_Enable(USART_TypeDef* USARTx, uint32_t USART_DMAEnable, TypeState NewValue);
void USART_DMARE_Set(USART_TypeDef* USARTx, uint32_t USART_DMARE);

void USART_DEEPSLEEPModeEnable(USART_TypeDef* USARTx, TypeState NewState);
void USART_DEEPSLEEPModeWakeUpSourceConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUpSource);

void USART_INT_Set(USART_TypeDef* USARTx, uint32_t USART_INT, TypeState NewValue);
void USART_Request_Enable(USART_TypeDef* USARTx, uint32_t USART_Cmd, TypeState NewValue);
void USART_OverRunFunc_Set(USART_TypeDef* USARTx, uint32_t USART_OVRD);
TypeState USART_GetBitState(USART_TypeDef* USARTx, uint32_t USART_FLAG);
void USART_ClearBitState(USART_TypeDef* USARTx, uint32_t USART_FLAG);
TypeState USART_GetIntBitState(USART_TypeDef* USARTx, uint32_t USART_INT);
void USART_ClearIntBitState(USART_TypeDef* USARTx, uint32_t USART_INT);

#ifdef __cplusplus
}
#endif

#endif /*__GD32F1X0_USART_H */
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

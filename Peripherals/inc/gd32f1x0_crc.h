/**
  ******************************************************************************
  * @file    gd32f1x0_crc.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CRC header file of the firmware library.
  ******************************************************************************
  */

#ifndef __GD32F1X0_CRC_H
#define __GD32F1X0_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup CRC
  * @{
  */

/** @defgroup CRC_Exported_Constants
  * @{
  */
#define CRC_REVERSE_INPUT_DATA_NOT              ((uint32_t)0x00000000)
#define CRC_REVERSE_INPUT_DATA_BYTE             CRC_CTLR_REV_I_0
#define CRC_REVERSE_INPUT_DATA_HALFWORD         CRC_CTLR_REV_I_1
#define CRC_REVERSE_INPUT_DATA_WORD             CRC_CTLR_REV_I

/**
  * @}
  */
/** @defgroup CRC_Exported_Functions
  * @{
  */

void CRC_DeInit(void);
void CRC_ResetDTR(void);
void CRC_ReverseInputData_Config(uint32_t CRC_reverse_input_data);
void CRC_ReverseOutputData_Enable(TypeState NewValue);
void CRC_WriteIDTR(uint32_t CRC_init_data);
uint32_t CRC_CalcSingleData(uint32_t CRC_data);
uint32_t CRC_CalcDataFlow(uint32_t pbuffer[], uint32_t buffer_length);
uint32_t CRC_ReadDTR(void);
void CRC_WriteFDTR(uint8_t CRC_fdtr);
uint8_t CRC_ReadFDTR(void);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /*__GD32F1X0_CRC_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2014 GIGADEVICE*****END OF FILE****/

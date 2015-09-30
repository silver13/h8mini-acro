/**
  ******************************************************************************
  * @file    gd32f1x0_crc.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CRC functions of the firmware library.
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_crc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup CRC 
  * @brief CRC driver modules
  * @{
  */

/** @defgroup CRC_Private_Functions
  * @{
  */

/**
  * @brief  Reset the CRC registers
  * @param  None
  * @retval None
  */
void CRC_DeInit(void)
{
    CRC->IDTR = 0xFFFFFFFF;

    CRC->DTR = 0xFFFFFFFF;

    CRC->FDTR = 0x00;

    CRC->CTLR = CRC_CTLR_RESET;
}

/**
  * @brief  Reset CRC DTR register to the value of IDTR register.
  * @param  None
  * @retval None
  */
void CRC_ResetDTR(void)
{
    CRC->CTLR = CRC_CTLR_RESET;
}

/**
  * @brief  Configure the reverse operation of input data.
  * @param  CRC_reverse_input_data: Specify the reverse operation of input data.
  *   This parameter can be any of the following values:
  *     @arg CRC_REVERSE_INPUT_DATA_NOT: input data is not reversed before calculation
  *     @arg CRC_REVERSE_INPUT_DATA_BYTE: input data is reversed on 8 bits before calculation
  *     @arg CRC_REVERSE_INPUT_DATA_HALFWORD:input data is reversed on 16 bits before calculation
  *     @arg CRC_REVERSE_INPUT_DATA_WORD: input data is reversed on 32 bits before calculation
  * @retval None
  */
void CRC_ReverseInputData_Config(uint32_t CRC_reverse_input_data)
{
    uint32_t temp_ctlr = 0;

    temp_ctlr = CRC->CTLR;
    /* Reset REV_I bits */
    temp_ctlr &= (uint32_t)(~CRC_CTLR_REV_I);
    temp_ctlr |= (uint32_t)CRC_reverse_input_data;
    CRC->CTLR = (uint32_t)temp_ctlr;
}

/**
  * @brief  Enable or disable the reverse operation of output data.
  * @param  NewValue: reverse operation state to configure.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CRC_ReverseOutputData_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        CRC->CTLR |= CRC_CTLR_REV_O;
    }
    else
    {
        CRC->CTLR &= (uint32_t)~((uint32_t)CRC_CTLR_REV_O);
    }
}

/**
  * @brief  Write the IDTR register.
  * @param  CRC_init_data: initial value of the CRC calculation 
  * @retval None
  */
void CRC_WriteIDTR(uint32_t CRC_init_data)
{
    CRC->IDTR = CRC_init_data;
}

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data.
  * @param  CRC_data: data to compute its CRC value
  * @retval 32-bit CRC value
  */
uint32_t CRC_CalcSingleData(uint32_t CRC_data)
{
    CRC->DTR = CRC_data;
    
    return (CRC->DTR);
}

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data array.
  * @param  pbuffer[]: pointer to the data array
  * @param  buffer_length: length of the data array
  * @retval 32-bit CRC value
  */
uint32_t CRC_CalcDataFlow(uint32_t pbuffer[], uint32_t buffer_length)
{
    uint32_t index = 0;

    for(index = 0; index < buffer_length; index++)
    {
        CRC->DTR = pbuffer[index];
    }
    return (CRC->DTR);
}

/**
  * @brief  Read current CRC value.
  * @param  None
  * @retval 32-bit CRC value
  */
uint32_t CRC_ReadDTR(void)
{
    return (CRC->DTR);
}

/**
  * @brief  Write an 8-bit data in FDTR.
  * @param  CRC_fdtr: 8-bit data to write
  * @retval None
  */
void CRC_WriteFDTR(uint8_t CRC_fdtr)
{
    CRC->FDTR = CRC_fdtr;
}

/**
  * @brief  Read the 8-bit data stored in FDTR
  * @param  None
  * @retval 8-bit data
  */
uint8_t CRC_ReadFDTR(void)
{
    return (CRC->FDTR);
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

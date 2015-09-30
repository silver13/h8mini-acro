/**
  ******************************************************************************
  * @file    gd32f1x0_cec.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CEC functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_cec.h"
#include "gd32f1x0_rcc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup CEC 
  * @brief CEC driver modules
  * @{
  */

/** @defgroup CEC_Private_Defines
  * @{
  */
#define BROADCAST_ADDRESS      ((uint32_t)0x0000000F) /*!< Broadcast address */
#define SR_CLEAR_MASK          ((uint32_t)0x7FFFFE00) /*!< SR register Mask */

/**
  * @}
  */

/** @defgroup CEC_Private_Functions
  * @{
  */

/**
  * @brief  Reset the CEC peripheral.
  * @param  None
  * @retval None
  */
void CEC_DeInit(void)
{
    RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_CECRST, ENABLE);
    RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_CECRST, DISABLE);
}

/**
  * @brief  Initialize the CEC peripheral.
  * @param  CEC_InitParaStruct: The structuer contains configuration information.
  * @retval None
  */
void CEC_Init(CEC_InitPara* CEC_InitParaStruct)
{
    uint32_t temp_sr = 0;

    /* Get the CEC SR */
    temp_sr = CEC->SR;

    /* Clear SR bits except own address*/
    temp_sr &= SR_CLEAR_MASK;

    /* Configure the CEC SR register */
    temp_sr |= (CEC_InitParaStruct->CEC_SFT | CEC_InitParaStruct->CEC_RXTOL |
                CEC_InitParaStruct->CEC_BRESTOP  | CEC_InitParaStruct->CEC_BREGEN |
                CEC_InitParaStruct->CEC_LBPEGEN| CEC_InitParaStruct->CEC_BCNG |
                CEC_InitParaStruct->CEC_SFTOPT);

    /* Write to CEC SR register */
    CEC->SR = temp_sr;
}

/**
  * @brief  Initial CEC_InitParameter members.
  * @param  CEC_InitParameter : pointer to a CEC_InitPara structure.
  * @retval None
  */
void CEC_ParaInit(CEC_InitPara* CEC_InitParaStruct)
{
    /* Reset CEC init structure parameters values */
    CEC_InitParaStruct->CEC_SFT = CEC_SFT_STD;
    CEC_InitParaStruct->CEC_RXTOL = CEC_RXTOL_STD;
    CEC_InitParaStruct->CEC_BRESTOP = CEC_BRESTOP_OFF;
    CEC_InitParaStruct->CEC_BREGEN = CEC_BREGEN_OFF;
    CEC_InitParaStruct->CEC_LBPEGEN = CEC_LBPEGEN_OFF;
    CEC_InitParaStruct->CEC_BCNG = CEC_BCNG_OFF;
    CEC_InitParaStruct->CEC_SFTOPT = CEC_SFTOPT_SOM;
}

/**
  * @brief  Enable or disable the CEC peripheral.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CEC_Enable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        CEC->CTLR |= CEC_CTLR_CECON;
    }
    else
    {
        CEC->CTLR &= ~CEC_CTLR_CECON;
    }
}

/**
  * @brief  Enable or disable the CEC Listen Mode.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CEC_ListenModeEnable(TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        CEC->SR |= CEC_SR_LSTN;
    }
    else
    {
        CEC->SR &= ~CEC_SR_LSTN;
    }
}

/**
  * @brief  Set the CEC Own Address.
  * @param  CEC_OwnAddress: The CEC own address.
  * @retval None
  */
void CEC_SetOwnAddress(uint8_t CEC_OwnAddress)
{
    CEC->SR |= 1 <<(CEC_OwnAddress + 16);
}

/**
  * @brief  Clear all of the CEC Own Address.
  * @param  None
  * @retval None
  */
void CEC_ClearAllOwnAddress(void)
{
    CEC->SR &= ~CEC_SR_OADR;
}

/**
  * @brief  Send a Data by the CEC peripheral.
  * @param  Data: Data to be Send.
  * @retval None
  */
void CEC_SendData(uint8_t Data)
{
    CEC->TDTR = Data;
}

/**
  * @brief  Return the received data by the CEC peripheral.
  * @param  None
  * @retval The value of the received data.
  */
uint8_t CEC_ReceiveData(void)
{
    return (uint8_t)(CEC->RDTR);
}

/**
  * @brief  Start a new message transfer.
  * @param  None
  * @retval None
  */
void CEC_StartOfMessage(void)
{
    CEC->CTLR |= CEC_CTLR_SOM; 
}

/**
  * @brief  Set the EOM bit, Stop transmits after next data.
  * @param  None.
  * @retval None
  */
void CEC_EndOfMessage(void)
{
    CEC->CTLR |= CEC_CTLR_EOM;
}

/**
  * @brief  Enable or disable the CEC interrupts.
  * @param  CEC_INT: specifies the CEC interrupt source. Select one of the follwing values:
  *     @arg CEC_INT_TAE: Transfer acknowledge Error.
  *     @arg CEC_INT_TE: Transfer Error.
  *     @arg CEC_INT_TU: Transfer-Buffer Underrun.
  *     @arg CEC_INT_TEND: Transmission successful end.
  *     @arg CEC_INT_TBR: Transfer-Byte data Request.
  *     @arg CEC_INT_ARBLST: Arbitration Lost.
  *     @arg CEC_INT_RAE: Receive Acknowledge Error.
  *     @arg CEC_INT_LBPE: Long bit period Error.
  *     @arg CEC_INT_SBPE: Short bit period Error.
  *     @arg CEC_INT_BRE: Bit Rising Error.
  *     @arg CEC_INT_RO: Receive Overrun.
  *     @arg CEC_INT_REND: End Of Reception.
  *     @arg CEC_INT_RBR: Rx-Byte Data Received 
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void CEC_INTConfig(uint32_t CEC_INT, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        CEC->IER |= CEC_INT;
    }
    else
    {
        CEC->IER &= ~CEC_INT;
    }
}

/**
  * @brief  Check whether the CEC flag is set or not.
  * @param  CEC_FLAG: specifies the CEC flag. Select one of the follwing values:
  *     @arg CEC_FLAG_TAE: Transfer acknowledge Error.
  *     @arg CEC_FLAG_TE: Transfer Error.
  *     @arg CEC_FLAG_TU: Transfer-Buffer Underrun.
  *     @arg CEC_FLAG_TEND: Transmission successful end.
  *     @arg CEC_FLAG_TBR: Transfer-Byte data Request.
  *     @arg CEC_FLAG_ARBLST: Arbitration Lost.
  *     @arg CEC_FLAG_RAE: Receive Acknowledge Error.
  *     @arg CEC_FLAG_LBPE: Long bit period Error.
  *     @arg CEC_FLAG_SBPE: Short bit period Error.
  *     @arg CEC_FLAG_BRE: Bit Rising Error.
  *     @arg CEC_FLAG_RO: Receive Overrun.
  *     @arg CEC_FLAG_REND: End Of Reception.
  *     @arg CEC_FLAG_RBR: Rx-Byte Data Received 
  * @retval The new state of CEC_FLAG
  */
TypeState CEC_GetBitState(uint32_t CEC_FLAG) 
{
    /* Check the status of the specified CEC flag */
    if ((CEC->ISTR & CEC_FLAG) != (uint16_t)RESET)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the CEC flags.
  * @param  CEC_FLAG: specifies the CEC flag. Select one of the follwing values:
  *     @arg CEC_FLAG_TAE: Transfer acknowledge Error.
  *     @arg CEC_FLAG_TE: Transfer Error.
  *     @arg CEC_FLAG_TU: Transfer-Buffer Underrun.
  *     @arg CEC_FLAG_TEND: Transmission successful end.
  *     @arg CEC_FLAG_TBR: Transfer-Byte data Request.
  *     @arg CEC_FLAG_ARBLST: Arbitration Lost.
  *     @arg CEC_FLAG_RAE: Receive Acknowledge Error.
  *     @arg CEC_FLAG_LBPE: Long bit period Error.
  *     @arg CEC_FLAG_SBPE: Short bit period Error.
  *     @arg CEC_FLAG_BRE: Bit Rising Error.
  *     @arg CEC_FLAG_RO: Receive Overrun.
  *     @arg CEC_FLAG_REND: End Of Reception.
  *     @arg CEC_FLAG_RBR: Rx-Byte Data Received.
  * @retval None
  */
void CEC_ClearBitState(uint32_t CEC_FLAG)
{
    CEC->ISTR = CEC_FLAG;
}

/**
  * @brief  Check whether the interrupt flag is set or not.
  * @param  CEC_INT: specifies the CEC interrupt source. Select one of the follwing values:
  *     @arg CEC_INT_TAE: Transfer acknowledge Error.
  *     @arg CEC_INT_TE: Transfer Error.
  *     @arg CEC_INT_TU: Transfer-Buffer Underrun.
  *     @arg CEC_INT_TEND: Transmission successful end.
  *     @arg CEC_INT_TBR: Transfer-Byte data Request.
  *     @arg CEC_INT_ARBLST: Arbitration Lost.
  *     @arg CEC_INT_RAE: Receive Acknowledge Error.
  *     @arg CEC_INT_LBPE: Long bit period Error.
  *     @arg CEC_INT_SBPE: Short bit period Error.
  *     @arg CEC_INT_BRE: Bit Rising Error.
  *     @arg CEC_INT_RO: Receive Overrun.
  *     @arg CEC_INT_REND: End Of Reception.
  *     @arg CEC_INT_RBR: Rx-Byte Data Received.
  * @retval The new state of CEC_INT.
  */
TypeState CEC_GetIntBitState(uint32_t CEC_INT)
{
    if (((CEC->ISTR & CEC_INT) != (uint32_t)RESET) && (CEC->IER & CEC_INT))
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the CEC interrupt pending bits.
  * @param  CEC_INT: specifies the CEC interrupt source. Select one of the follwing values:
  *     @arg CEC_INT_TAE: Transfer acknowledge Error.
  *     @arg CEC_INT_TE: Transfer Error.
  *     @arg CEC_INT_TU: Transfer-Buffer Underrun.
  *     @arg CEC_INT_TEND: Transmission successful end.
  *     @arg CEC_INT_TBR: Transfer-Byte data Request.
  *     @arg CEC_INT_ARBLST: Arbitration Lost.
  *     @arg CEC_INT_RAE: Receive Acknowledge Error.
  *     @arg CEC_INT_LBPE: Long bit period Error.
  *     @arg CEC_INT_SBPE: Short bit period Error.
  *     @arg CEC_INT_BRE: Bit Rising Error.
  *     @arg CEC_INT_RO: Receive Overrun.
  *     @arg CEC_INT_REND: End Of Reception.
  *     @arg CEC_INT_RBR: Rx-Byte Data Received.
  * @retval None
  */
void CEC_ClearIntBitState(uint32_t CEC_INT)
{
    CEC->ISTR = CEC_INT;
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

/**
  ******************************************************************************
  * @file    gd32f1x0_usart.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USART functions of the firmware library.
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_usart.h"
#include "gd32f1x0_rcc.h"
  
/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup USART 
  * @brief USART driver modules
  * @{
  */ 

/** @defgroup USART_Private_Defines
  * @{
  */
/* USART Interrupts mask */
#define INT_MASK                                        ((uint32_t)0x000000FF)

/* USART CTLR1 initializes bit mask */
#define CTLR1_CLEAR_MASK                                ((uint32_t)(0X0000160C))

/* USART CTLR2 CLOCK initializes bit mask */
#define CTLR2_CLOCK_CLEAR_MASK                          ((uint32_t)(0x00000F00))

/* USART CTLR3 CLOCK initializes bit mask */
#define CTLR3_CLEAR_MASK                                ((uint32_t)(0x00000300))

/**
  * @}
  */

/** @defgroup USART_Private_Functions
  * @{
  */
  
/**
  * @brief  Reset the USART peripheral.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  NewValue: new value of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_DeInit(USART_TypeDef* USARTx)
{
    if (USARTx == USART1)
    {
        RCC_APB2PeriphReset_Enable(RCC_APB2PERIPH_USART1, ENABLE);
        RCC_APB2PeriphReset_Enable(RCC_APB2PERIPH_USART1, DISABLE);
    }
    else
    {
        if (USARTx == USART2)
        {
            RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_USART2, ENABLE);
            RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_USART2, DISABLE);
        }
    }
}

/**
  * @brief  Initialize the USART peripheral interface parameters.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_InitParaStruct: the struct USART_InitPara pointer.
  * @retval None
  */
void USART_Init(USART_TypeDef* USARTx, USART_InitPara* USART_InitParaStruct)
{
    uint32_t divider = 0, apbclock = 0, temp = 0;
    RCC_ClocksPara RCC_ClocksState;
    
    USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_UEN);
    /* Initialize CTLR2 */ 
    temp = USARTx->CTLR2;
    /* Reset stop bits then set it use USART_STBits */
    temp &= ~((uint32_t)USART_CTLR2_STB);
    temp |= (uint32_t)USART_InitParaStruct->USART_STBits;
    
    USARTx->CTLR2 = temp;
    
    /*Initialize CTLR1*/
    temp = USARTx->CTLR1;
    /* Reset WL, PCEN, PM, TEN and REN bits */
    temp &= ~((uint32_t)CTLR1_CLEAR_MASK);
    
    /* According to USART_WL,USART_Parity,USART_RxorTx to configure the CTLR1 */
    temp |= (uint32_t)USART_InitParaStruct->USART_WL | USART_InitParaStruct->USART_Parity |
    USART_InitParaStruct->USART_RxorTx;
    USARTx->CTLR1 = temp;
    
    /*Initialize CTLR3*/
    temp = USARTx->CTLR3;
    /* Reset CTSEN and RTSEN bits */
    temp &= ~((uint32_t)CTLR3_CLEAR_MASK);
    
    /* According to USART_HardwareFlowControl to configure the CTLR3 */
    temp |= USART_InitParaStruct->USART_HardwareFlowControl;
    USARTx->CTLR3 = temp;
    
    /*Initialize USART BRR*/
    RCC_GetClocksFreq(&RCC_ClocksState);
    
    if (USARTx == USART1)
    {
        apbclock = RCC_ClocksState.USART1CLK_Frequency;
    }
    else
    {
        apbclock = RCC_ClocksState.APB1_Frequency;
    }
    
    /* Get integer of baud-rate divide and raction of baud-rate divider */
    if ((USARTx->CTLR1 & USART_CTLR1_OM) != 0)
    {
        /* When Oversampling mode is 8 Samples,OM = 1, USARTDIV [3:1] = BRR [2:0], BRR [3] must be reset */
        divider = (uint32_t)((2*apbclock) / ((USART_InitParaStruct->USART_BRR)));
        temp    = (uint32_t)((2*apbclock) % ((USART_InitParaStruct->USART_BRR)));
    }
    else 
    {
        /* When Oversampling mode is 16 Samples,OM = 1, USARTDIV [3:0] = BRR [3:0] */
        divider = (uint32_t)((apbclock) / ((USART_InitParaStruct->USART_BRR)));
        temp    = (uint32_t)((apbclock) % ((USART_InitParaStruct->USART_BRR)));
    }
    /* Round the divider : if fractional part i greater than 0.5 increment divider */
    if (temp >=  (USART_InitParaStruct->USART_BRR) / 2)
    {
        divider++;
    } 
    
    /* Implement the divider in case Oversampling mode is 8 Samples */
    if ((USARTx->CTLR1 & USART_CTLR1_OM) != 0)
    {
        /* Get the LSB of divider and shift it to the right by 1 bit */
        temp = (divider & (uint16_t)0x000F) >> 1;
    
        /* Update the divider value */
        divider = (divider & (uint16_t)0xFFF0) | temp;
    }  
    USARTx->BRR = (uint16_t)divider;
}

/**
  * @brief  Initial the struct USART_InitPara
  * @param  USARTx: where x can be 1 or 2 to select the USART peripheral.
  * @param  USART_InitParaStruct: the struct USART_InitPara pointer
  * @retval None
  */
void USART_ParaInit(USART_TypeDef* USARTx,USART_InitPara* USART_InitParaStruct)
{
    /* USART_InitStruct members default value */
    USART_InitParaStruct->USART_BRR = 9600;
    USART_InitParaStruct->USART_WL   =USART_WL_8B;
    USART_InitParaStruct->USART_STBits =USART_STBITS_1;
    USART_InitParaStruct->USART_Parity =USART_PARITY_RESET;
    USART_InitParaStruct->USART_RxorTx =USART_RXORTX_RX | USART_RXORTX_TX;
    USART_InitParaStruct->USART_HardwareFlowControl =USART_HARDWAREFLOWCONTROL_NONE;
}   

/**
  * @brief  Initialize the USARTx peripheral Clock according to the 
  *         specified parameters in the USART_ClockInitStruct.
  * @param  USARTx: where x can be 1 or 2 to select the USART peripheral.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *   structure that contains the configuration information for the specified 
  *   USART peripheral.  
  * @retval None
  */
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitPara* USART_ClockInitStruct)
{
    uint32_t temp = 0;
    
    temp = USARTx->CTLR2;
    /* Clear CKEN, CPL, CPH, LBCP and SSM bits */
    temp &= ~((uint32_t)CTLR2_CLOCK_CLEAR_MASK);
    
    /* Reset hen set it usethe USART Clock, CPL, CPH, LBCP */
    temp |= (uint32_t)(USART_ClockInitStruct->USART_CKEN | USART_ClockInitStruct->USART_CPL | 
                       USART_ClockInitStruct->USART_CPH | USART_ClockInitStruct->USART_LBCP);
    /* Write to USART CTLR2 */
    USARTx->CTLR2 = temp;
}

/**
  * @brief  Initial the struct USART_ClockInitPara.
  * @param  USART_ClockInitParaStruct: the struct USART_ClockInitPara pointer
  * @retval None
  */
void USART_ClockStructInit(USART_ClockInitPara* USART_ClockInitParaStruct)
{
    /*Reset USART_ClockInitStruct members default value */
    USART_ClockInitParaStruct->USART_CKEN = USART_CKEN_RESET;
    USART_ClockInitParaStruct->USART_CPL = USART_CPL_LOW;
    USART_ClockInitParaStruct->USART_CPH = USART_CPH_1EDGE;
    USART_ClockInitParaStruct->USART_LBCP = USART_LBCP_DISABLE;
}

/**
  * @brief  Enable or disable the specified USART peripheral.
  * @param  USARTx:  the USART peripheral where x can be 1 or 2.
  * @param  NewValue: new value of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* Enable or disable the specified USART peripheral by setting the UEN bit in the CTLR1 register */ 
    if (NewValue!= DISABLE)
    {
        USARTx->CTLR1 |= USART_CTLR1_UEN;
    }
    else
    {
        USARTx->CTLR1 &= (uint32_t)~((uint32_t)USART_CTLR1_UEN);
    }
}

/**
  * @brief  Enable or disable the USART's transmitter or receiver.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_Direction: specifies the USART direction.
  *   This parameter can be any:
  *     @arg USART_RXORTX_TX: USART Transmitter
  *     @arg USART_RXORTX_RX: USART Receiver
  * @param  NewValue: new value of the USART transfer direction.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_TransferDirection_Enable(USART_TypeDef* USARTx, uint32_t USART_Direction, TypeState NewValue)
{
    /* By setting the TEN and/or REN bits in the USART CTLR1 register to set the USART transfer direction*/
    if (NewValue != DISABLE)
    {
        USARTx->CTLR1 |= USART_Direction;
    }
    else
    {
        USARTx->CTLR1 &= ~USART_Direction;
    }
}

/**
  * @brief  Enable or disable the USART's 8x oversampling mode.
  * @param  USARTx: the USART peripheral where x can be 1 or 2 .
  * @param  NewValue: new value of the USART 8x oversampling mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_OverSampling8_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the OM bit in the CTLR1 register enable or disable the USART's 8x oversampling mode */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR1 |= USART_CTLR1_OM;
    }
    else
    {
        USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_OM);
    }
}  

/**
  * @brief  Enable or disable the USART's one sampling bit method.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  NewValue: new value of the USART one sampling bit method.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function has to be called before calling USART_Enable() function.  
  * @retval None
  */
void USART_OneSamplingBit_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the OSBM bit in the CTLR3 register enableor disable the USART's one sampling bit method  */  
    if (NewValue != DISABLE)
    {
        USARTx->CTLR3 |= USART_CTLR3_OSBM;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_OSBM);
    }
}

/**
  * @brief  Enable or disable the USART's most significant bit first.
  * @param  USARTx: the USART peripheral where x can be 1 or 2 .
  * @param  NewValue: new value of the USART most significant bit first
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function has to be called before calling USART_Enable() function.
  * @retval None
  */
void USART_MSBFirst_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the MSBF bit in the CTLR2 register enable or disable the USART's most significant bit first*/  
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |= USART_CTLR2_MSBF;
    }
    else
    {
        USARTx->CTLR2 &= ~((uint32_t)USART_CTLR2_MSBF);
    }
}

/**
  * @brief  Enable or disable data bit level inversion.
  * @param  USARTx: where x can be 1 or 2 to select the USART peripheral.
  * @param  NewValue: new levels of USART data.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_DataInvert_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the DINV bit in the CTLR2 register enable or disable data bit level inversion */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |= USART_CTLR2_DINV;
    }
    else
    {
        USARTx->CTLR2 &= (uint32_t)~((uint32_t)USART_CTLR2_DINV);
    }
}

/**
  * @brief  Enable or disable the Pin signal values inversion.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_InvertPin:  The USART pin is inverted.
  *   This parameter can be :
  *     @arg USART_INVPIN_TX: USART Tx pin pin signal values inversion.
  *     @arg USART_INVPIN_RX: USART Rx pin signal values inversion.
  * @param  NewValue: new value 0f the USART pin.
  *   This parameter can be:
  *     @arg ENABLE: pin signal values are inverted (Vdd =0, Gnd =1).
  *     @arg DISABLE: pin signal values are not inverted (Vdd =1, Gnd =0).
  * @retval None
  */
void USART_InvPin_Enable(USART_TypeDef* USARTx, uint32_t USART_InvertPin, TypeState NewValue)
{
    /* By setting the TINV ,RINV in the USART CTLR2 register enable or disable the Pin signal values inversion */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |=USART_InvertPin;
    }
    else
    {
        USARTx->CTLR2 &= ~USART_InvertPin;
    }
}

/**
  * @brief  Enable or disable Swap TX/RX pins functions.
  * @param  USARTx: the USART peripheral where x can be 1 or 2 .
  * @param  NewValue: new value of the USARTx TX/RX pins.
  *   This parameter can be:
  *     @arg ENABLE: The TX and RX pins functions are swapped.
  *     @arg DISABLE: TX/RX pins are not swapped 
  * @retval None
  */
void USART_SWPFunction_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the STRP bit in the CTLR2 register enable or disable Swap TX/RX pins functions */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |= USART_CTLR2_STRP;
    }
    else
    {
        USARTx->CTLR2 &= (uint32_t)~((uint32_t)USART_CTLR2_STRP);
    }
}

/**
  * @brief  Enable or disable the receiver TimeOut function of USART1.
  * @param  USARTx: where USARTx is USART1 
  * @param  NewValue: new value of the USARTx receiver TimeOut function .
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ReceiverTimeOut_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    /* By setting the RTEN bit in the CTLR2 register enable or disable the receiver TimeOut function of USART1 */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |= USART_CTLR2_RTEN;
    }
    else
    {
        USARTx->CTLR2 &= ~((uint32_t)USART_CTLR2_RTEN);
    }
}

/**
  * @brief  Set the receiver TimeOut threshold.
  * @param  USARTx: where USARTx is USART1
  * @param  USART_ReceiverTimeOutThreshold: the Receiver Time Out Threshold value.
  * @retval None
  */
void USART_SetReceiveTimeOut_Threshold(USART_TypeDef* USARTx,uint32_t USART_ReceiverTimeOutThreshold)
{    
    /* Clear and set the receiver TimeOut threshold */
    USARTx->RTR &= ~((uint32_t)USART_RTR_RT);
    
    USARTx->RTR |= USART_ReceiverTimeOutThreshold;
}

/**
  * @brief  Set the system clock prescaler.
  * @param  USARTx: where USARTx is USART1.
  * @param  USART_Prescaler: specifies the prescaler clock.
  * @note   This function has to be called before calling USART_Enable() function.
  * @retval None
  */
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler)
{ 
    /* Clear and set the USART prescaler */
    USARTx->GTPR &= USART_GTPR_GT;
    
    USARTx->GTPR |= USART_Prescaler;
}

/**
  * @brief  Send data by the USART peripheral.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  Data: the data will be sent.
  * @retval None
  */
void USART_DataSend(USART_TypeDef* USARTx,uint16_t Data)
{
    USARTx->TDTR = (Data & (uint16_t)0x01FF);
}
   
/**
  * @brief  Receive data by the USART peripheral.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @retval The received data.
  */
uint16_t USART_DataReceive(USART_TypeDef* USARTx)
{
    return (uint16_t)(USARTx->RDTR & (uint16_t)0x01FF);
}
 
/**
  * @brief  Enable or disable the Auto Baud Rate detection.
  * @param  USARTx: Where USARTx is USART1 .
  * @param  NewValue: new value of the USART1 auto baud rate.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_AutoBaudRateDect_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /*By setting the ABDEN bit in the CTLR2 register */
        USARTx->CTLR2 |= USART_CTLR2_ABDEN;
    }
    else
    {
        USARTx->CTLR2 &= ~((uint32_t)USART_CTLR2_ABDEN);         
    }
}

/**
  * @brief  Set USART auto baud rate detection mode.
  * @param  USARTx: Where USARTx is USART1.
  * @param  USART_AutoBaudRate: USART auto baud rate detection mode.
  * @retval None
  */
void USART_AutoBaudRateDectMode_Set(USART_TypeDef* USARTx, uint32_t USART_AutoBaudRate)
{
    USARTx->CTLR2 &= (uint32_t)(~(USART_CTLR2_ABDM_0) |~(USART_CTLR2_ABDM_1));
    USARTx->CTLR2 |= USART_AutoBaudRate;
}

/**
  * @brief  Set the address of the USART terminal.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
void USART_Address(USART_TypeDef* USARTx, uint8_t USART_Address)
{
    /* Clear the USART address and Set the USART terminal*/
    USARTx->CTLR2 &= (uint32_t)(~(USART_CTLR2_ADDR));
    
    USARTx->CTLR2 |=((uint32_t)USART_Address << (uint32_t)0x18);
}

/**
  * @brief  Enable or disable the USART's mute mode.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  NewValue: the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_MuteMode_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the MEN bit in the CTLR1 register enable or disable the USART's mute mode*/
    if (NewValue != DISABLE)
    {
        USARTx->CTLR1 |= USART_CTLR1_MEN;
    }
    else
    {
        USARTx->CTLR1 &=~((uint32_t)USART_CTLR1_MEN);
    }
}

/**
  * @brief  Set the USART WakeUp method from mute mode.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_WakeUp
  *   This parameter can be:
  *     @arg USART_WAKEUP_IDLELINE
  *     @arg USART_WAKEUP_ADDRESSMARK
  * @retval None
  */
void USART_MuteModeWakeUp_Set(USART_TypeDef* USARTx, uint32_t USART_WakeUp)
{
    USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_WM);
    USARTx->CTLR1 |= USART_WakeUp;
}

/**
  * @brief  Set the the USART Address detection length.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_AddressLength
  *   This parameter can be:
  *     @arg USART_ADDRESSLENGTH_4B
  *     @arg USART_ADDRESSLENGTH_7B
  * @retval None
  */
void USART_AddressDetection_Set(USART_TypeDef* USARTx, uint32_t USART_AddressLength)
{
    USARTx->CTLR2 &= ~((uint32_t)USART_CTLR2_ADDM);
    USARTx->CTLR2 |= USART_AddressLength;
}

/**
  * @brief  Set the USART LIN Break detection length.
  * @param  USARTx: Where USARTx is USART1.
  * @param  USART_LINBreakDetectLength
  *   This parameter can be:
  *     @arg USART_LINBREAKDETECTLENGTH_10B
  *     @arg USART_LINBREAKDETECTLENGTH_11B
  * @retval None
  */
void USART_SetLINBDLength(USART_TypeDef* USARTx,uint32_t USART_LINBreakDetectLength)
{
    USARTx->CTLR2 &= ~((uint32_t)USART_CTLR2_LBDL);
    USARTx->CTLR2 |= USART_LINBreakDetectLength;
}

/**
  * @brief  Enable or disable the USART's LIN mode.
  * @param  USARTx: Where USARTx is USART1.
  * @param  NewValue: the USART LIN mode value.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_LIN_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    /* By setting the LINEN bit in the CTLR2 register enable or disable the USART's LIN mode */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR2 |= USART_CTLR2_LMEN;
    }
    else
    {
        USARTx->CTLR2 &=~((uint32_t)USART_CTLR2_LMEN);
    }
}

/**
  * @brief  Enable or disable the Half-duplex mode.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  NewValue: the USART Half-duplex mode value.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_HalfDuplex_Enable(USART_TypeDef* USARTx, TypeState NewValue)
{
    /* By setting the HDEN bit in the CTLR3 register enable or disable the Half-duplex mode */ 
    if (NewValue  != DISABLE)
    {
        USARTx->CTLR3 |= USART_CTLR3_HDEN;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_HDEN);
    }
}

/**
  * @brief  Set the the USART guard time.
  * @param  USARTx: Where USARTx is USART1.
  * @param  USART_GuardTime: the USART guard time.
  * @retval None
  */
void USART_GuardTime_Set(USART_TypeDef* USARTx, uint8_t USART_GuardTime)
{    
    /* Set the USART guard time */
    USARTx->GTPR &= USART_GTPR_PSC;
    
    USARTx->GTPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}

/**
  * @brief  Enable or disable the USART's Smart Card mode.
  * @param  USARTx: the USART peripheral is USART1
  * @param  NewValue: the Smart Card mode value.
  *   This parameter can be: ENABLE or DISABLE.      
  * @retval None
  */
void USART_SmartCard_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    /* By setting the SCEN bit in the CTLR3 register enable or disable the USART's Smart Card mode */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR3 |= USART_CTLR3_SCEN;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_SCEN);
    }
}

/**
  * @brief  Enable or disable NACK transmission.
  * @param  USARTx: the USART peripheral is USART1
  * @param  NewValue: the NACK transmission state.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_SmartCardNACK_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    /* By setting the NACK bit in the CTLR3 register Enable or disable NACK transmission */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR3 |= USART_CTLR3_NACK;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_NACK);
    }
}

/**
  * @brief  Set Smartcard auto-retry number.
  * @param  USARTx: the USART peripheral is USART1  
  * @param  USART_AutoNumber: the Smart Card auto retry number.
  * @retval None
  */
void USART_AutoRetryNumber_Set(USART_TypeDef* USARTx,uint8_t USART_AutoNumber)
{    
    /* Set the USART auto retry count */
    USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_SCRTNUM);
    USARTx->CTLR3 |= ((uint32_t)USART_AutoNumber << 0x11);
}

/**
  * @brief  Set the Smart Card Block length.
  * @param  USARTx: the USART peripheral is USART1  
  * @param  USART_BlockLength: the Smart Card block length.
  * @retval None
  */
void USART_BlockLength_Set(USART_TypeDef* USARTx,uint8_t USART_BlockLength)
{    
    /* Set the Smart Card Block length */
    USARTx->RTR &= ~((uint32_t)USART_RTR_BL);
    USARTx->RTR |= ((uint32_t)USART_BlockLength << 0x18);
}

/**
  * @brief  Set the  USART1 IrDA low-power.
  * @param  USARTx: the USART peripheral is USART1  
  * @param  USART_IrDAMode
  *   This parameter can be:
  *     @arg USART_IRDAMODE_LOWPOWER
  *     @arg USART_IRDAMODE_NORMAL
  * @retval None
  */
void USART_IrDA_Set(USART_TypeDef* USARTx,uint32_t USART_IrDAMode)
{
    USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_IRLP);
    USARTx->CTLR3 |= USART_IrDAMode;
}

/**
  * @brief  Enable or disable the USART's IrDA interface.
  * @param  USARTx: the USART peripheral is USART1. 
  * @param  NewValue: the IrDA mode value.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_IrDA_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    /* By setting the IREN bit in the CTLR3 register enable or disable the USART's IrDA interface */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR3 |= USART_CTLR3_IREN;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_IREN);
    }
}

/**
  * @brief  Enable or disable the Driver enable mode .
  * @param  USARTx: the USART peripheral where x can be 1 or 2. 
  * @param  NewValue: the Driver enable mode value.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_DE_Enable(USART_TypeDef* USARTx,TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* By setting the DEM bit in the CTLR3 register enable or disable the Driver enable mode */
        USARTx->CTLR3 |= USART_CTLR3_DEM;
    }
    else
    {
        USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_DEM);
    }
}

/**
  * @brief  Set Driver enable polarity mode
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_DEPly
  *   This parameter can be:
  *     @arg USART_DEPOLARITY_HIGH
  *     @arg USART_DEPOLARITY_LOW
  * @retval None
  */
void USART_DriverEnablePolarity_Set(USART_TypeDef* USARTx, uint32_t USART_DEPly)
{
    USARTx->CTLR3 &=~((uint32_t)USART_CTLR3_DEP);
    USARTx->CTLR3 |= USART_DEPly;
}

/**
  * @brief  Set the specified RS485 DE assertion time
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_DEATime: define the time between the activation of the
  *   DE (Driver Enable) signal and the beginning of the start bit.
  * @retval None
  */
void USART_DEATime_Set(USART_TypeDef* USARTx, uint32_t USART_DEATime)
{
    /* Set the new value for the DE assertion time */
    USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_DEA);
    
    USARTx->CTLR1 |=((uint32_t)USART_DEATime << 0x15);
}

/**
  * @brief  Set the specified RS485 DE deassertion time
  * @param  USARTx: where x can be 1 or 2 to select the USART peripheral.
  * @param  USART_DEDTime: define the time between the end of the last stop bit,
  *   in a transmitted message, and the de-activation of the DE (Driver Enable) signal
  * @retval None
  */
void USART_DEDTime_Set(USART_TypeDef* USARTx, uint32_t USART_DEDTime)
{
    /* Set the new value for the DE deassertion time */
    USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_DED);
    
    USARTx->CTLR1 |=((uint32_t)USART_DEDTime << 0x10);
}

/**
  * @brief  DMA enable for transmission or reception.
  * @param  USARTx: the USART peripheral where x can be 1 or 2 .
  * @param  USART_DMAEnable: the DMA request.
  *   This parameter can be:
  *     @arg USART_DMAREQ_RX
  *     @arg USART_DMAREQ_TX
  * @param  NewValue: the DMA Request sources state.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_DMA_Enable(USART_TypeDef* USARTx, uint32_t USART_DMAEnable, TypeState NewValue)
{
    /* Enable or disable the DMA transfer for transmission or reception by setting the DENT and/or
    DENR bits in the USART CTLR3 register */
    if (NewValue != DISABLE)
    {
        USARTx->CTLR3 |= USART_DMAEnable;
    }
    else
    {
        USARTx->CTLR3 &= ~USART_DMAEnable;
    }
}

/**
  * @brief  Enable or disable the USART's DMA on reception error.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_DMARE: the DMA state on reception error 
  *   This parameter can be:
  *     @arg USART_DMAONERROR_ENABLE
  *     @arg USART_DMAONERROR_DISABLE.
  * @retval None
  */
void USART_DMARE_Set(USART_TypeDef* USARTx, uint32_t USART_DMARE)
{
    /* Clear the DMA Reception error detection bit */
    USARTx->CTLR3 &= (uint32_t)~((uint32_t)USART_CTLR3_DDRE);
    /* Set the new value for the DMA Reception error detection bit */
    USARTx->CTLR3 |= USART_DMARE;
}

 /**
  * @brief  Enables or disables the specified USART peripheral in Deep-sleep Mode.
  * @param  USARTx: where x can be 1 to select the USART peripheral.
  * @param  NewState: new state of the USARTx peripheral state in stop mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function has to be called when USART clock is set to HSI or LSE.
  * @retval None
  */
void USART_DEEPSLEEPModeEnable(USART_TypeDef* USARTx, TypeState NewState)
{
    if (NewState != DISABLE)
    {
        /* Enable the selected USART in DEEP-SLEEP mode by setting the UESM bit in the CTLR1
        register */
        USARTx->CTLR1 |= USART_CTLR1_UESM;
    }
    else
    {
        /* Disable the selected USART in DEEP-SLEEP mode by clearing the UE bit in the CTLR1
        register */
        USARTx->CTLR1 &= ~((uint32_t)USART_CTLR1_UESM);
    }
}

/**
  * @brief  Selects the USART WakeUp method form Deep-sleep mode.
  * @param  USARTx: where x can be 1 to select the USART peripheral.
  * @param  USART_WakeUpSource: specifies the selected USART wakeup method.
  *   This parameter can be one of the following values:
  *     @arg USART_WAKEUPSOURCE_ADDRESSMATCH: WUM active on address match.
  *     @arg USART_WAKEUPSOURCE_STARTBIT: WUM active on Start bit detection.
  *     @arg USART_WAKEUPSOURCE_RBNE: WUM active on RBNE.
  * @note   This function has to be called before calling USART_Enable() function.
  * @retval None
  */
void USART_DEEPSLEEPModeWakeUpSourceConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUpSource)
{
    USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_WUM);
    USARTx->CTLR3 |= USART_WakeUpSource;
}

/**
  * @brief  Enable or disable USART interrupt.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_USART_INT: the USART interrupt
      This parameter can be one of the following values:
         @arg USART_INT_WU                                                   
         @arg USART_INT_AM                                                   
         @arg USART_INT_EB                                                  
         @arg USART_INT_RT                                                   
         @arg USART_INT_PE                                                   
         @arg USART_INT_TBE                                                  
         @arg USART_INT_TC                                                   
         @arg USART_INT_RBNE                                                 
         @arg USART_INT_IDLEF                                                
         @arg USART_INT_LBDF                                                 
         @arg USART_INT_CTSF                                                  
         @arg USART_INT_ERIE                                                                                                     
  * @param  NewValue: the USART interrupt State.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_INT_Set(USART_TypeDef* USARTx, uint32_t USART_INT, TypeState NewValue)
{
    uint32_t  intpos , usartreg=0;
    uint32_t usartxbase = 0;
    
    usartxbase = (uint32_t)USARTx;
    
    /* Get the USART register index and the interrupt position */
    usartreg = USART_INT& ((uint32_t)0x0000FF00); 
    intpos = USART_INT & ((uint32_t)0x000000FF);
    
    /* Get the interrupt from which register: CTLR1,CTLR2 OR CTLR3 */
    if (usartreg == 0x00000300) 
    {
        usartxbase += 0x08;
    }
    else if (usartreg == 0x00000200) 
    {
        usartxbase += 0x04;
    }
    else 
    {
    }
    if (NewValue != DISABLE)
    {
        *(__IO uint32_t*)usartxbase  |= (((uint32_t)0x01) << intpos);
    }
    else
    {
        *(__IO uint32_t*)usartxbase &= ~(((uint32_t)0x01) << intpos);
    }
}

/**
  * @brief  Enable or disable the USART's Request.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_Cmd: the USART request.
      This parameter can be one of the following values:
         @arg USART_REQUEST_ABDCMD
         @arg USART_REQUEST_SBKCMD
         @arg USART_REQUEST_MMRQ
         @arg USART_REQUEST_RXFCMD
         @arg USART_REQUEST_TXFCMD 
  * @param  NewValue: new state of the DMA interface on reception error occurs.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_Request_Enable(USART_TypeDef* USARTx, uint32_t USART_Cmd, TypeState NewValue)
{
    /* Enable or disable the USART ReQuest by setting the dedicated request bit in the CMDregister. */
    if (NewValue != DISABLE)
    {
        USARTx->CMD |= USART_Cmd;
    }
    else
    {
        USARTx->CMD &= (uint32_t)~USART_Cmd;
    }
}

/**
  * @brief  Enable or disable the USART's Overrun functionality.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_OVRD: the  OVRD bit state in case of Overrun error.
  * @retval None
  */
void USART_OverRunFunc_Set(USART_TypeDef* USARTx, uint32_t USART_OVRD)
{
    /* Clear the OVRD bit and set a new value */
    USARTx->CTLR3 &= ~((uint32_t)USART_CTLR3_OVRD); 
    USARTx->CTLR3 |= USART_OVRD;
}

/**
  * @brief  Detect the bit flag of STR register.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_FLAG: the flag of STR register.
       This parameter can be one of the following values:
  *      @arg USART_FLAG_REA:  Receive Enable acknowledge flag.
  *      @arg USART_FLAG_TEA:  Transmit Enable acknowledge flag.
  *      @arg USART_FLAG_WUF:  Wake up flag.
  *      @arg USART_FLAG_RWU:  Receive Wake up .
  *      @arg USART_FLAG_SBF:  Send Break flag.
  *      @arg USART_FLAG_AMF:  Character match flag.
  *      @arg USART_FLAG_BSY:  Busy flag.
  *      @arg USART_FLAG_ABDF: Auto baud rate flag.
  *      @arg USART_FLAG_ABRE: Auto baud rate error flag.
  *      @arg USART_FLAG_EBF:  End of block flag.
  *      @arg USART_FLAG_RTF:  Receive time out flag.
  *      @arg USART_FLAG_CTS:  Inverted nCTS input bit status.
  *      @arg USART_FLAG_CTSF: CTS Change flag.
  *      @arg USART_FLAG_LBDF: LIN Break detection flag.
  *      @arg USART_FLAG_TBE:  Transmit data register empty flag.
  *      @arg USART_FLAG_TC:   Transmission Complete flag.
  *      @arg USART_FLAG_RBNE: Receive data register not empty flag.
  *      @arg USART_FLAG_IDLEF:Idle Line detection flag.
  *      @arg USART_FLAG_ORE:  OverRun Error flag.
  *      @arg USART_FLAG_NE:   Noise Error flag.
  *      @arg USART_FLAG_FE:   Framing Error flag.
  *      @arg USART_FLAG_PE:   Parity Error flag.
  * @retval The new state of USART_FLAG (SET or RESET).
  */
TypeState USART_GetBitState(USART_TypeDef* USARTx, uint32_t USART_FLAG)
{
    if ((USARTx->STR & USART_FLAG) != (uint16_t)RESET)
    {
        return  SET;
    }
    else
    {
        return  RESET;
    }
}

/**
  * @brief  Clear the bit flag of SCR register.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_FLAG: the flag of SCR register.
       This parameter can be one of the following values:
  *      @arg USART_FLAG_WUF:  Wake up flag .
  *      @arg USART_FLAG_AMF:  Character match flag.
  *      @arg USART_FLAG_EBF:  End of block flag.
  *      @arg USART_FLAG_RTF:  Receive time out flag.
  *      @arg USART_FLAG_CTSF: CTS Change flag.
  *      @arg USART_FLAG_LBDF: LIN Break detection flag.
  *      @arg USART_FLAG_TC:   Transmission Complete flag.
  *      @arg USART_FLAG_IDLEF:Idle Line detection flag.
  *      @arg USART_FLAG_ORE:  OverRun Error flag.
  *      @arg USART_FLAG_NE:   Noise Error flag.
  *      @arg USART_FLAG_FE:   Framing Error flag.
  *      @arg USART_FLAG_PE:   Parity Error flag.
  * @retval None
  */
void USART_ClearBitState(USART_TypeDef* USARTx, uint32_t USART_FLAG)
{
    USARTx->SCR = USART_FLAG;
}

/**
  * @brief  Detect the interrupt bit flag.
  * @param  USARTx: the USART peripheral where x can be 1 or 2.
  * @param  USART_INT: the USART interrupt bit flag.
      This parameter can be one of the following values:
         @arg USART_INT_WU                                                   
         @arg USART_INT_AM                                                   
         @arg USART_INT_EB                                                  
         @arg USART_INT_RT                                                   
         @arg USART_INT_PE                                                   
         @arg USART_INT_TBE                                                  
         @arg USART_INT_TC                                                   
         @arg USART_INT_RBNE                                                 
         @arg USART_INT_IDLEF                                                
         @arg USART_INT_LBDF                                                 
         @arg USART_INT_CTSF                                                  
         @arg USART_INT_ORE                                                  
         @arg USART_INT_NE                                                   
         @arg USART_INT_FE
  * @retval The new state of USART_INT (SET or RESET).
  */
TypeState USART_GetIntBitState(USART_TypeDef* USARTx, uint32_t USART_INT)
{
    uint32_t bitpos = 0, itmask = 0, usartreg = 0;
    
    /* Get the USART register index and the interrupt position */
    usartreg = (((uint16_t)USART_INT) >> 0x08);
    
    itmask = (USART_INT)&(INT_MASK);
    
    itmask = (uint32_t)0x01 << itmask;
    
    if (usartreg == 0x01)
    {
        itmask &= USARTx->CTLR1;
    }
    else
    {
        if (usartreg == 0x02) 
        {
            itmask &= USARTx->CTLR2;
        }
        else 
        {
            itmask &= USARTx->CTLR3;
        }
    }
    
    bitpos = USART_INT >> 0x10;
    bitpos = (uint32_t)0x01 << bitpos;
    bitpos &= USARTx->STR;
    if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
    {
        return  SET;
    }
    else
    {
        return  RESET;
    }
}

/**
  * @brief  Clears the interrupt bit flag.
  * @param  USARTx: the USART peripheral where x can be 1 or 2 to select.
  * @param  USART_INT: the interrupt pending bit.
      This parameter can be one of the following values:
         @arg USART_INT_WU                                                   
         @arg USART_INT_AM                                                   
         @arg USART_INT_EB                                                  
         @arg USART_INT_RT                                                   
         @arg USART_INT_PE                                                                                                     
         @arg USART_INT_TC                                                                                                    
         @arg USART_INT_IDLEF                                                
         @arg USART_INT_LBDF                                                 
         @arg USART_INT_CTSF                                                  
         @arg USART_INT_ORE                                                  
         @arg USART_INT_NE                                                   
         @arg USART_INT_FE
  * @retval None
  */
void USART_ClearIntBitState(USART_TypeDef* USARTx, uint32_t USART_INT)
{
    uint32_t bitpos = 0, itmask = 0;
    
    bitpos = USART_INT >> 0x10;
    itmask = ((uint32_t)0x01 << (uint32_t)bitpos);
    USARTx->SCR = (uint32_t)itmask;
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

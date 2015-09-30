/**
  ******************************************************************************
  * @file    gd32f1x0_spi.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   SPI functions of the firmware library.  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_spi.h"
#include "gd32f1x0_rcc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup SPI 
  * @brief SPI driver modules
  * @{
  */ 

/** @defgroup SPI_Private_Defines
  * @{
  */

/* SPI registers Masks */
#define CTLR1_CLEAR_Mask        ((uint16_t)0x3040)
#define I2SCTLR_CLEAR_Mask      ((uint16_t)0xF040)

/**
  * @}
  */

/** @defgroup SPI_Private_Functions
  * @{
  */

/**
  * @brief  Reset the SPIx and the I2Sx peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @retval None
  */
void SPI_I2S_DeInit(SPI_TypeDef* SPIx)
{
    if (SPIx == SPI1)
    {
        /* Reset SPI1 and I2S1 peripheral*/
        RCC_APB2PeriphReset_Enable(RCC_APB2PERIPH_SPI1RST, ENABLE);
        RCC_APB2PeriphReset_Enable(RCC_APB2PERIPH_SPI1RST, DISABLE);
    }
    else if (SPIx == SPI2)
    {
        /* Reset SPI2 */
        RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_SPI2RST, ENABLE);
        RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_SPI2RST, DISABLE);
    }
    else if( SPIx == SPI3)
    {
        /* Reset SPI3 and I2S3 peripheral */
        RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_SPI3RST, ENABLE);
        RCC_APB1PeriphReset_Enable(RCC_APB1PERIPH_SPI3RST, DISABLE);
    }
}

/**
  * @brief  Initialize the SPIx peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_InitParaStruct: The structuer contains configuration information.
  * @retval None
  */
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitPara* SPI_InitParaStruct)
{
    uint16_t temp_ctrl1 = 0;
    
    /* Configure SPIx CTRL1 according to the SPI_InitParaStruct */
    temp_ctrl1 = SPIx->CTLR1;
    temp_ctrl1 &= CTLR1_CLEAR_Mask;
    temp_ctrl1 |= (uint16_t)((uint32_t)SPI_InitParaStruct->SPI_TransType | SPI_InitParaStruct->SPI_Mode |
                  SPI_InitParaStruct->SPI_FrameFormat | SPI_InitParaStruct->SPI_SCKPL |  
                  SPI_InitParaStruct->SPI_SCKPH | SPI_InitParaStruct->SPI_SWNSSEN |  
                  SPI_InitParaStruct->SPI_PSC | SPI_InitParaStruct->SPI_FirstBit);
    SPIx->CTLR1 = temp_ctrl1;
    
    /* Configure SPIx CRC Polynomial */
    SPIx->CPR = SPI_InitParaStruct->SPI_CRCPOL;
    
    /* Configure the I2SSEL bit in I2SCTLR register as SPI mode */
    SPIx->I2SCTLR &= ~SPI_I2SCTLR_I2SSEL;

}

/**
  * @brief  Initialize the I2Sx peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1 or 3.
  * @param  I2S_InitParaStruct: The structuer contains configuration information.
  * @retval None
  */
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitPara* I2S_InitParaStruct)
{
    uint16_t temp_i2sctrl = 0, div = 2, of = 0;
    uint32_t temp = 0;
    RCC_ClocksPara RCC_Clocks;
    
    /* SPIx I2SCTLR & I2SCKP Configuration */
    /* Deinit I2SCTLR I2SCKP register */
    SPIx->I2SCTLR &= I2SCTLR_CLEAR_Mask; 
    SPIx->I2SCKP = 0x0002;
    
    /* Default config of the prescaler*/
    if(I2S_InitParaStruct->I2S_AudioFreq == I2S_AUDIOFREQ_DEFAULT)
    {
        of = (uint16_t)0;
        div = (uint16_t)2;   
    }
    else
    {
        /* Get system clock */
        RCC_GetClocksFreq(&RCC_Clocks);
        temp = RCC_Clocks.CK_SYS_Frequency;  
        
        /* Calculate the prescaler depending on the MCLK output state and the data format with a flaoting point. */
        if(I2S_InitParaStruct->I2S_MCKOE == I2S_MCK_ENABLE)
        {
            temp = (uint16_t)(((((temp / 256) * 10) / I2S_InitParaStruct->I2S_AudioFreq)) + 5);
        }
        else
        {
            if(I2S_InitParaStruct->I2S_FrameFormat == I2S_FRAMEFORMAT_DL16b_CL16b)
            {
                temp = (uint16_t)(((((temp / 32) *10 ) / I2S_InitParaStruct->I2S_AudioFreq)) + 5);    
            }
            else
            {
                temp = (uint16_t)(((((temp / 64) *10 ) / I2S_InitParaStruct->I2S_AudioFreq)) + 5);
            }
        }

        /* Remove the flaoting point */
        temp = temp / 10;     
        of = (uint16_t)(temp & (uint16_t)0x0001);
        div = (uint16_t)((temp - of) / 2);
        of = (uint16_t) (of << 8);
    }
    
    /* Inappropriate prescaler, Set the default values */
    if ((div < 1) || (div > 0xFF))
    {
        div = 2;
        of = 0;
    }
    
    /* Configure SPIx I2SCKP */
    SPIx->I2SCKP = (uint16_t)(div | (uint16_t)(of | (uint16_t)I2S_InitParaStruct->I2S_MCKOE));  
    
    /* Configure SPIx I2SCTLR according to the I2S_InitParaStruct */
    temp_i2sctrl = SPIx->I2SCTLR;
    temp_i2sctrl |= (uint16_t)(SPI_I2SCTLR_I2SSEL | (uint16_t)(I2S_InitParaStruct->I2S_Mode | \
                  (uint16_t)(I2S_InitParaStruct->I2S_STD | (uint16_t)(I2S_InitParaStruct->I2S_FrameFormat | \
                  (uint16_t)I2S_InitParaStruct->I2S_CKPL))));
    SPIx->I2SCTLR = temp_i2sctrl;

}

/**
  * @brief  Initial SPI_InitParaStruct members.
  * @param  SPI_InitParaStruct : pointer to a SPI_InitPara structure.
  * @retval None
  */
void SPI_ParaInit(SPI_InitPara* SPI_InitParaStruct)
{
    /* Reset SPI init structure parameters values */
    SPI_InitParaStruct->SPI_TransType = SPI_TRANSTYPE_FULLDUPLEX;
    SPI_InitParaStruct->SPI_Mode = SPI_MODE_SLAVE;
    SPI_InitParaStruct->SPI_FrameFormat = SPI_FRAMEFORMAT_8BIT;
    SPI_InitParaStruct->SPI_SCKPL = SPI_SCKPL_LOW;
    SPI_InitParaStruct->SPI_SCKPH = SPI_SCKPH_1EDGE;
    SPI_InitParaStruct->SPI_SWNSSEN = SPI_SWNSS_HARD;
    SPI_InitParaStruct->SPI_PSC = SPI_PSC_2;
    SPI_InitParaStruct->SPI_FirstBit = SPI_FIRSTBIT_MSB;
    SPI_InitParaStruct->SPI_CRCPOL = 7;
}

/**
  * @brief  Initial I2S_InitParaStruct members.
  * @param  I2S_InitParaStruct : pointer to a I2S_InitPara structure.
  * @retval None
  */
void I2S_ParaInit(I2S_InitPara* I2S_InitParaStruct)
{
    /* Reset I2S init structure parameters values */
    I2S_InitParaStruct->I2S_Mode = I2S_MODE_SLAVETX;
    I2S_InitParaStruct->I2S_STD = I2S_STD_PHILLIPS;
    I2S_InitParaStruct->I2S_FrameFormat = I2S_FRAMEFORMAT_DL16b_CL16b;
    I2S_InitParaStruct->I2S_MCKOE = I2S_MCK_DISABLE;
    I2S_InitParaStruct->I2S_AudioFreq = I2S_AUDIOFREQ_DEFAULT;
    I2S_InitParaStruct->I2S_CKPL = I2S_CKPL_LOW;
}

/**
  * @brief  Enable or disable the SPI peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void SPI_Enable(SPI_TypeDef* SPIx, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        SPIx->CTLR1 |= SPI_CTLR1_SPIEN;
    }
    else
    {
        SPIx->CTLR1 &= ~SPI_CTLR1_SPIEN;
    }
}

/**
  * @brief  Enable or disable the I2S peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1 or 3.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void I2S_Enable(SPI_TypeDef* SPIx, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        SPIx->I2SCTLR |= SPI_I2SCTLR_I2SEN;
    }
    else
    {
        SPIx->I2SCTLR &= ~SPI_I2SCTLR_I2SEN;
    }
}

/**
  * @brief  Enable or disable the SPI or I2S interrupts.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_INT: specifies the SPI or I2S interrupt source. Select one of the follwing values :
  *     @arg SPI_I2S_INT_TBE: Tx buffer empty interrupt mask
  *     @arg SPI_I2S_INT_RBNE: Rx buffer not empty interrupt mask
  *     @arg SPI_I2S_INT_ERR: Error interrupt mask 
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void SPI_I2S_INTConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_INT, TypeState NewValue)
{
    uint16_t intmask = 0 ;
    
    /* Get the interrupt enable bit in CTRL2 */
    intmask = (uint16_t)1 << (uint16_t)(SPI_I2S_INT >> 4);
    
    /* Enable or disable the selected interrupt */
    if (NewValue != DISABLE)
    {
        SPIx->CTLR2 |= intmask;
    }
    else
    {
        SPIx->CTLR2 &= (uint16_t)~intmask;
    }
}

/**
  * @brief  Enable or disable the SPIx or I2Sx DMA transfer request.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_DMAReq: Select the SPI or I2S DMA transfer request. Select one of the follwing values :
  *     @arg SPI_I2S_DMA_Tx: Tx buffer DMA transfer request
  *     @arg SPI_I2S_DMA_Rx: Rx buffer DMA transfer request 
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void SPI_I2S_DMA_Enable(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        SPIx->CTLR2 |= SPI_I2S_DMAReq;
    }
    else
    {
        SPIx->CTLR2 &= (uint16_t)~SPI_I2S_DMAReq;
    }
}

/**
  * @brief  Send a Data by the SPIx or I2Sx peripheral.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  Data : Data to be Send.
  * @retval None
  */
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
    SPIx->DTR = Data;
}

/**
  * @brief  Return the received data by the SPIx or I2Sx peripheral. 
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @retval The value of the received data.
  */
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx)
{
    return SPIx->DTR;
}

/**
  * @brief  NSS pin internal software management.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_SWNSS: specifies the SPI NSS internal state. Select one of the follwing values :
  *     @arg SPI_SWNSS_SET: Set NSS pin internally
  *     @arg SPI_SWNSS_RESET: Reset NSS pin internally
  * @retval None
  */
void SPI_SWNSSConfig(SPI_TypeDef* SPIx, uint16_t SPI_SWNSS)
{
    if (SPI_SWNSS != SPI_SWNSS_RESET)
    {
        /* Set NSS pin */
        SPIx->CTLR1 |= SPI_CTLR1_SWNSS;
    }
    else
    {
        /* Reset NSS pin */
        SPIx->CTLR1 &= ~SPI_CTLR1_SWNSS;
    }
}

/**
  * @brief  Enable or disable the NSS output.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void SPI_NSSDRV(SPI_TypeDef* SPIx, TypeState NewValue)
{ 
    if (NewValue != DISABLE)
    {
        SPIx->CTLR2 |= SPI_CTLR2_NSSDRV;
    }
    else
    {
        SPIx->CTLR2 &= ~SPI_CTLR2_NSSDRV;
    }
}

/**
  * @brief  Configure the data frame format.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_FrameFormat: Select the data frame format. Select one of the follwing values :
  *     @arg SPI_FRAMEFORMAT_16BIT: Set data frame format to 16bit
  *     @arg SPI_FRAMEFORMAT_8BIT: Set data frame format to 8bit
  * @retval None
  */
void SPI_FrameFormatConfig(SPI_TypeDef* SPIx, uint16_t SPI_FrameFormat)
{
    /* Clear FF16 bit */
    SPIx->CTLR1 &= (uint16_t)~SPI_FRAMEFORMAT_16BIT;
    /* Set new FF16 bit value */
    SPIx->CTLR1 |= SPI_FrameFormat;
}

/**
  * @brief  Send the SPIx CRC value.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @retval None
  */
void SPI_SendCRCNext(SPI_TypeDef* SPIx)
{
    /* Enable the CRC transmission */
    SPIx->CTLR1 |= SPI_CTLR1_CRCNT;
}

/**
  * @brief  Enable or disable the CRC calculation.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void SPI_CRC_Enable(SPI_TypeDef* SPIx, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the CRC calculation */
        SPIx->CTLR1 |= SPI_CTLR1_CRCEN;
    }
    else
    {
        /* Disable the CRC calculation */
        SPIx->CTLR1 &= ~SPI_CTLR1_CRCEN;
    }
}

/**
  * @brief  Get the transmit or the receive CRC register value.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_CRC: Select the transmit or the receive CRC register. Select one of the follwing values :
  *     @arg SPI_CRC_TX: Selects Tx CRC register
  *     @arg SPI_CRC_RX: Selects Rx CRC register
  * @retval The selected CRC register value.
  */
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC)
{
    if (SPI_CRC != SPI_CRC_RX)
    {
        /* Transmit CRC value */
        return SPIx->TCR;
    }
    else
    {
        /* Receive CRC value */
        return SPIx->RCR;
    }
}

/**
  * @brief  Get the CRC Polynomial value.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @retval The CRC Polynomial value.
  */
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx)
{
    return SPIx->CPR;
}

/**
  * @brief  Select the transfer direction in bidirectional mode.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_BDOE: The transfer direction in bi-directional mode. Select one of the follwing values :
  *     @arg SPI_BDOE_TX: Selects Tx transmission direction
  *     @arg SPI_BDOE_RX: Selects Rx receive direction 
  * @retval None
  */
void SPI_BDOEConfig(SPI_TypeDef* SPIx, uint16_t SPI_BDOE)
{
    if (SPI_BDOE == SPI_BDOE_TX)
    {
        /* Transmit only mode*/
        SPIx->CTLR1 |= SPI_BDOE_TX;
    }
    else
    {
        /* Receive only mode */
        SPIx->CTLR1 &= SPI_BDOE_RX;
    }
}

/**
  * @brief  Check whether the flag is set or not.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_FLAG: Select the flag. Select one of the follwing values :
  *     @arg SPI_FLAG_TBE: Transmit buffer empty flag.
  *     @arg SPI_FLAG_RBNE: Receive buffer not empty flag.
  *     @arg SPI_FLAG_BSY: Busy flag.
  *     @arg SPI_FLAG_OVR: Overrun flag.
  *     @arg SPI_FLAG_MODF: Mode Fault flag.
  *     @arg SPI_FLAG_CRCERR: CRC Error flag.
  *     @arg I2S_FLAG_TBE: Transmit buffer empty flag.
  *     @arg I2S_FLAG_RBNE: Receive buffer not empty flag.
  *     @arg I2S_FLAG_BSY: Busy flag.
  *     @arg I2S_FLAG_OVR: Overrun flag.
  *     @arg I2S_FLAG_UDR: Underrun Error flag.
  *     @arg I2S_FLAG_CHSIDE: Channel Side flag.
  * @retval The new state of SPI_I2S_FLAG.
  */
TypeState SPI_I2S_GetBitState(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG)
{
    /* Check the status of the selected flag */
    if ((SPIx->STR & SPI_I2S_FLAG) != (uint16_t)RESET)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the flag, only used for clear CRCERR flag.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_FLAG: Select the flag. This parametric only can be SPI_FLAG_CRCERR.
  * @note
  *   The other flags are cleared by software sequences:  
  *   - OVR (OverRun error) flag is cleared by software sequence: a read 
  *     operation to SPI_DTR register (SPI_I2S_ReceiveData()) followed by a read 
  *     operation to SPI_STR register (SPI_I2S_GetBitState()).
  *   - UDR (UnderRun error) flag is cleared by a read operation to 
  *     SPI_STR register (SPI_I2S_GetBitState()).
  *   - MODF (Mode Fault) flag is cleared by software sequence: a read/write 
  *     operation to SPI_STR register (SPI_I2S_GetBitState()) followed by a 
  *     write operation to SPI_CTLR1 register (SPI_Enable() to enable the SPI).
  * @retval None
  */
void SPI_I2S_ClearBitState(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG)
{
    SPIx->STR = (uint16_t)~SPI_I2S_FLAG;
}

/**
  * @brief  Check whether interrupt has occurred.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_INT: Select the SPI/I2S interrupt. Select one of the follwing values :
  *     @arg SPI_I2S_INT_TBE: Transmit buffer empty interrupt.
  *     @arg SPI_I2S_INT_RBNE: Receive buffer not empty interrupt.
  *     @arg SPI_I2S_INT_OVR: Overrun interrupt.
  *     @arg SPI_INT_MODF: Mode Fault interrupt.
  *     @arg SPI_INT_CRCERR: CRC Error interrupt.
  *     @arg I2S_INT_UDR: Underrun Error interrupt. 
  * @retval The new state of SPI_I2S_INT.
  */
TypeState SPI_I2S_GetIntBitState(SPI_TypeDef* SPIx, uint8_t SPI_I2S_INT)
{
    uint16_t intposition = 0, intmask = 0;
    /* Get the interrupt pending bit and enable bit */
    intposition = 0x01 << (SPI_I2S_INT & 0x0F);
    intmask = 0x01 << (SPI_I2S_INT >> 4);
    
    /* Check the status of the interrupt */
    if (((SPIx->STR & intposition) != (uint16_t)RESET) && (SPIx->CTLR2 & intmask))
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the SPIx or I2S interrupt pending bit, only uesd for clear CRCERR interrupt.
  * @param  SPIx: the SPI/I2S peripheral where x can be 1..3.
  * @param  SPI_I2S_INT: Select the SPI or I2S interrupt. This parametric only can be SPI_INT_CRCERR.
  * @note
  *   The other flags are cleared by software sequences:  
  *   - OVR (OverRun error) flag is cleared by software sequence: a read 
  *     operation to SPI_DTR register (SPI_I2S_ReceiveData()) followed by a read 
  *     operation to SPI_STR register (SPI_I2S_GetBitState()).
  *   - UDR (UnderRun error) flag is cleared by a read operation to 
  *     SPI_STR register (SPI_I2S_GetBitState()).
  *   - MODF (Mode Fault) flag is cleared by software sequence: a read/write 
  *     operation to SPI_STR register (SPI_I2S_GetBitState()) followed by a 
  *     write operation to SPI_CTLR1 register (SPI_Enable() to enable the SPI).   
  * @retval None
  */
void SPI_I2S_ClearIntBitState(SPI_TypeDef* SPIx, uint8_t SPI_I2S_INT)
{
    uint16_t itpos = 0;

    /* Clear the select interrupt pending bit. */
    itpos = 0x01 << (SPI_I2S_INT & 0x0F);
    SPIx->STR = (uint16_t)~itpos;
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

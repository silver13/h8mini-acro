/**
  ******************************************************************************
  * @file    gd32f1x0_dma.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   DMA functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_dma.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup DMA 
  * @brief DMA driver modules
  * @{
  */ 

/** @defgroup DMA_Private_Defines
  * @{
  */

/* DMA Channel config registers Masks */
#define CTLRx_CLEAR_MASK                ((uint32_t)0xFFFF800F) 

/* DMA Reset registers mask */
#define  DMA_REGISTERS_RESET            ((uint32_t)0x00000000)

/* DMA Reset registers mask */
#define  DMA_INIT_RESET                 ((uint32_t)0x00000000)

/* DMA1 Channelx interrupt  bit masks */
#define DMA1_CHANNEL1_INT_MASK          ((uint32_t)(DMA_IFR_GIF1 | DMA_IFR_TCIF1 | DMA_IFR_HTIF1 | DMA_IFR_ERRIF1))
#define DMA1_CHANNEL2_INT_MASK          ((uint32_t)(DMA_IFR_GIF2 | DMA_IFR_TCIF2 | DMA_IFR_HTIF2 | DMA_IFR_ERRIF2))
#define DMA1_CHANNEL3_INT_MASK          ((uint32_t)(DMA_IFR_GIF3 | DMA_IFR_TCIF3 | DMA_IFR_HTIF3 | DMA_IFR_ERRIF3))
#define DMA1_CHANNEL4_INT_MASK          ((uint32_t)(DMA_IFR_GIF4 | DMA_IFR_TCIF4 | DMA_IFR_HTIF4 | DMA_IFR_ERRIF4))
#define DMA1_CHANNEL5_INT_MASK          ((uint32_t)(DMA_IFR_GIF5 | DMA_IFR_TCIF5 | DMA_IFR_HTIF5 | DMA_IFR_ERRIF5))
#define DMA1_CHANNEL6_INT_MASK          ((uint32_t)(DMA_IFR_GIF6 | DMA_IFR_TCIF6 | DMA_IFR_HTIF6 | DMA_IFR_ERRIF6))
#define DMA1_CHANNEL7_INT_MASK          ((uint32_t)(DMA_IFR_GIF7 | DMA_IFR_TCIF7 | DMA_IFR_HTIF7 | DMA_IFR_ERRIF7))

/**
  * @}
  */

/** @defgroup DMA_Private_Functions 
  * @{
  */

/** @defgroup DMA_Group1 Initialization and Configuration functions
  * @brief    Initialization and Configuration functions
  * @{
  */

/**
  * @brief  Deinitialize the DMAy Channelx registers
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @retval None
  */
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx)
{
    /* Disable the selected DMAy Channelx */
    DMAy_Channelx->CTLRx &= (uint16_t)(~DMA_CTLRx_CHEN);
    
    /* Reset DMAy Channelx control register */
    DMAy_Channelx->CTLRx  = DMA_REGISTERS_RESET;
    
    /* Reset DMAy Channelx remaining bytes register */
    DMAy_Channelx->RCNTx = DMA_REGISTERS_RESET;
    
    /* Reset DMAy Channelx peripheral address register */
    DMAy_Channelx->PBARx  = DMA_REGISTERS_RESET;
    
    /* Reset DMAy Channelx memory address register */
    DMAy_Channelx->MBARx = DMA_REGISTERS_RESET;
    
    if (DMAy_Channelx == DMA1_CHANNEL1)
    {
        /* Clear interrupt bits of DMA1 Channel1 */
        DMA1->ICR |= DMA1_CHANNEL1_INT_MASK;
    }
    else if (DMAy_Channelx == DMA1_CHANNEL2)
    {
        /* Clear interrupt bits of DMA1 Channel2 */
        DMA1->ICR |= DMA1_CHANNEL2_INT_MASK;
    }
    else if (DMAy_Channelx == DMA1_CHANNEL3)
    {
        /* Clear interrupt bits of DMA1 Channel3 */
        DMA1->ICR |= DMA1_CHANNEL3_INT_MASK;
    }
    else if (DMAy_Channelx == DMA1_CHANNEL4)
    {
        /* Clear interrupt bits of DMA1 Channel4 */
        DMA1->ICR |= DMA1_CHANNEL4_INT_MASK;
    }
    else if (DMAy_Channelx == DMA1_CHANNEL5) 
    {
        /* Clear interrupt bits of DMA1 Channel5 */
        DMA1->ICR |= DMA1_CHANNEL5_INT_MASK;
    }
    else if (DMAy_Channelx == DMA1_CHANNEL6) 
    {
        /* Clear interrupt bits of DMA1 Channel6 */
        DMA1->ICR |= DMA1_CHANNEL6_INT_MASK;
    }
    
    else if(DMAy_Channelx == DMA1_CHANNEL7) 
    {
        /* Clear interrupt bits of DMA1 Channel7 */
        DMA1->ICR |= DMA1_CHANNEL7_INT_MASK;
    }
}

/**
  * @brief  Initialize the DMAy Channelx according to the DMA_InitParaStruct.
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @param  DMA_InitParaStruct: contain the configuration information for the specified DMA Channel.
  * @retval None
  */
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitPara* DMA_InitParaStruct)
{
    uint32_t temp = 0;
    
    /* Get the DMAy_Channelx CTLRx value */
    temp = DMAy_Channelx->CTLRx;
    
    /* Clear MEMTOMEM, PRIO, MSIZE, PSIZE, MNAGA, PNAGA, CIRC and DIR bits */
    temp &= CTLRx_CLEAR_MASK;
    
    /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
    /* Set MEMTOMEM, PRIO, MSIZE, PSIZE, MNAGA, PNAGA, CIRC and DIR bits according to DMA_InitParaStruct */
    temp |= DMA_InitParaStruct->DMA_DIR | DMA_InitParaStruct->DMA_Mode |
            DMA_InitParaStruct->DMA_PeripheralInc | DMA_InitParaStruct->DMA_MemoryInc |
            DMA_InitParaStruct->DMA_PeripheralDataSize | DMA_InitParaStruct->DMA_MemoryDataSize |
            DMA_InitParaStruct->DMA_Priority | DMA_InitParaStruct->DMA_MTOM;

    /* Write to DMAy Channelx CTLRx */
    DMAy_Channelx->CTLRx = temp;
    
    /* Write to DMAy Channelx RCNTx */
    DMAy_Channelx->RCNTx = DMA_InitParaStruct->DMA_BufferSize;
    
    /* Write to DMAy Channelx PBARx */
    DMAy_Channelx->PBARx = DMA_InitParaStruct->DMA_PeripheralBaseAddr;
    
    /* Write to DMAy Channelx MBARx */
    DMAy_Channelx->MBARx = DMA_InitParaStruct->DMA_MemoryBaseAddr;
}

/**
  * @brief  Set each DMA_InitParaStruct member to its default value.
  * @param  DMA_InitParaStruct: The structure pointer to DMA_InitParaStruct will be initialized.
  * @retval None
  */
void DMA_ParaInit(DMA_InitPara* DMA_InitParaStruct)
{
    /*-------------- Reset DMA init structure parameters values ------------------*/
    DMA_InitParaStruct->DMA_PeripheralBaseAddr = DMA_INIT_RESET;
    
    DMA_InitParaStruct->DMA_MemoryBaseAddr = DMA_INIT_RESET;
    
    DMA_InitParaStruct->DMA_DIR = DMA_DIR_PERIPHERALSRC;
    
    DMA_InitParaStruct->DMA_BufferSize = DMA_INIT_RESET;
    
    DMA_InitParaStruct->DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    
    DMA_InitParaStruct->DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    
    DMA_InitParaStruct->DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    
    DMA_InitParaStruct->DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    
    DMA_InitParaStruct->DMA_Mode = DMA_MODE_NORMAL;
    
    DMA_InitParaStruct->DMA_Priority = DMA_PRIORITY_LOW;
    
    DMA_InitParaStruct->DMA_MTOM = DMA_MEMTOMEM_DISABLE;
}

/**
  * @brief  Enable or disable the DMAy Channelx.
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @param  NewValue: new state of the DMAy Channelx. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA_Enable(DMA_Channel_TypeDef* DMAy_Channelx, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the DMAy Channelx */
        DMAy_Channelx->CTLRx |= DMA_CTLRx_CHEN;
    }
    else
    {
        /* Disable the DMAy Channelx */
        DMAy_Channelx->CTLRx &= (uint16_t)(~DMA_CTLRx_CHEN);
    }
}

/**
  * @}
  */

/** @defgroup DMA_Group2 The Remaining Counter functions       
 *  @brief   Remaining Counter functions 
 *
  * @{
  */

/**
  * @brief  Set the number of the remaining counter in the current DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @param  DataNumber: The number of the remaining counter in the current DMAy Channelx transfer.
  * @retval None.
  */
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber)
{
    /* Write to DMAy Channelx RCNTx */
    DMAy_Channelx->RCNTx = DataNumber;
}

/**
  * @brief  Return the number of remaining counter in the current DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @retval The number of remaining counter in the current DMAy Channelx transfer.
  */
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx)
{
    /* Return the number of remaining counter for DMAy Channelx */
    return ((uint16_t)(DMAy_Channelx->RCNTx));
}

/**
  * @}
  */

/** @defgroup DMA_Group3 Interrupts and flags management functions
  * @brief   Interrupts and flags management functions 
  * @{
  */

/**
  * @brief  Enable or disable the DMAy Channelx interrupts.
  * @param  DMAy_Channelx: where y:[1] to select the DMA and x:[1,7] to select the DMA Channel.
  * @param  DMA_INT: specify the DMA interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg DMA_INT_TC:  Transfer complete interrupt mask
  *     @arg DMA_INT_HT:  Half transfer interrupt mask
  *     @arg DMA_INT_ERR: Transfer error interrupt mask
  * @param  NewValue: new state of the DMA interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA_INTConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_INT, TypeState NewValue)
{
    if (NewValue != DISABLE)
    {
        /* Enable the DMA interrupts */
        DMAy_Channelx->CTLRx |= DMA_INT;
    }
    else
    {
        /* Disable the DMA interrupts */
        DMAy_Channelx->CTLRx &= ~DMA_INT;
    }
}

/**
  * @brief  Check whether the DMAy Channelx flag is set or not.
  * @param  DMA_FLAG: specify the flag to check.
  *   This parameter can be one of the following values:
  *     @arg DMA1_FLAG_GL1:  DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1:  DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1:  DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_ERR1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2:  DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2:  DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2:  DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_ERR2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3:  DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3:  DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3:  DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_ERR3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4:  DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4:  DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4:  DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_ERR4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5:  DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5:  DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5:  DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_ERR5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6:  DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6:  DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6:  DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_ERR6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7:  DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7:  DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7:  DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_ERR7: DMA1 Channel7 transfer error flag.      
  * @retval The new value of DMA_FLAG (SET or RESET).
  */
TypeState DMA_GetBitState(uint32_t DMA_FLAG)
{
    /* Check the status of the DMA flag */
    if ((DMA1->IFR & DMA_FLAG) != (uint32_t)RESET)
    {
        /* DMA_FLAG is set */
        return SET;
    }
    else
    {
        /* DMA_FLAG is reset */
        return RESET;
    }

}

/**
  * @brief  Clear the DMAy Channelx's bit flags.
  * @param  DMA_FLAG: specify the flag to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_FLAG_GL1:  DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1:  DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1:  DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_ERR1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2:  DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2:  DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2:  DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_ERR2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3:  DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3:  DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3:  DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_ERR3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4:  DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4:  DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4:  DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_ERR4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5:  DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5:  DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5:  DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_ERR5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6:  DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6:  DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6:  DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_ERR6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7:  DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7:  DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7:  DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_ERR7: DMA1 Channel7 transfer error flag.
  * @retval None
  */
void DMA_ClearBitState(uint32_t DMA_FLAG)
{
    /* Clear the DMA flags */
    DMA1->ICR = DMA_FLAG;
}

/**
  * @brief  Check whether the DMAy Channelx interrupt has occurred or not.
  * @param  DMA_INT: specifies the DMA interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg DMA1_INT_GL1:  DMA1 Channel1 global interrupt.
  *     @arg DMA1_INT_TC1:  DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_INT_HT1:  DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_INT_ERR1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_INT_GL2:  DMA1 Channel2 global interrupt.
  *     @arg DMA1_INT_TC2:  DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_INT_HT2:  DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_INT_ERR2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_INT_GL3:  DMA1 Channel3 global interrupt.
  *     @arg DMA1_INT_TC3:  DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_INT_HT3:  DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_INT_ERR3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_INT_GL4:  DMA1 Channel4 global interrupt.
  *     @arg DMA1_INT_TC4:  DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_INT_HT4:  DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_INT_ERR4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_INT_GL5:  DMA1 Channel5 global interrupt.
  *     @arg DMA1_INT_TC5:  DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_INT_HT5:  DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_INT_ERR5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_INT_GL6:  DMA1 Channel6 global interrupt.
  *     @arg DMA1_INT_TC6:  DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_INT_HT6:  DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_INT_ERR6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_INT_GL7:  DMA1 Channel7 global interrupt.
  *     @arg DMA1_INT_TC7:  DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_INT_HT7:  DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_INT_ERR7: DMA1 Channel7 transfer error interrupt.
  * @retval The new value of DMA_INT (SET or RESET).
  */
TypeState DMA_GetIntBitState(uint32_t DMA_INT)
{
    /* Check the status of the DMA interrupt */
    if((DMA1->IFR & DMA_INT) != (uint32_t)RESET)
    {
        /* DMA_INT is set */
        return SET;
    }
    else
    {
        /* DMA_INT is reset */
        return RESET;
    }
}

/**
  * @brief  Clear the DMAy Channelx's interrupt bits.
  * @param  DMA_INT: specify the DMA interrupt bit to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_INT_GL1:  DMA1 Channel1 global interrupt.
  *     @arg DMA1_INT_TC1:  DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_INT_HT1:  DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_INT_ERR1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_INT_GL2:  DMA1 Channel2 global interrupt.
  *     @arg DMA1_INT_TC2:  DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_INT_HT2:  DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_INT_ERR2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_INT_GL3:  DMA1 Channel3 global interrupt.
  *     @arg DMA1_INT_TC3:  DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_INT_HT3:  DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_INT_ERR3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_INT_GL4:  DMA1 Channel4 global interrupt.
  *     @arg DMA1_INT_TC4:  DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_INT_HT4:  DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_INT_ERR4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_INT_GL5:  DMA1 Channel5 global interrupt.
  *     @arg DMA1_INT_TC5:  DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_INT_HT5:  DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_INT_ERR5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_INT_GL6:  DMA1 Channel6 global interrupt.
  *     @arg DMA1_INT_TC6:  DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_INT_HT6:  DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_INT_ERR6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_INT_GL7:  DMA1 Channel7 global interrupt.
  *     @arg DMA1_INT_TC7:  DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_INT_HT7:  DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_INT_ERR7: DMA1 Channel7 transfer error interrupt.     
  * @retval None
  */
void DMA_ClearIntBitState(uint32_t DMA_INT)
{
    /* Clear the DMA interrupt bits */
    DMA1->ICR = DMA_INT;
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

/**
 ******************************************************************************
  * @file    gd32f1x0_tsi.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   TSI functions of the firmware library.
 ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_tsi.h"
#include "gd32f1x0_rcc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup TSI
  * @brief TSI driver modules
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup TSI_Exported_Functions
  * @{
  */

/**
  * @brief  Deinitialize the TSI.
  * @param  None
  * @retval None
  */
void TSI_DeInit( void )
{
    /* Enable TSI reset state */
    RCC->AHBRCR |= RCC_AHBPERIPH_TSIRST;
    
    /* Release TSI from reset state */
    RCC->AHBRCR &= ~RCC_AHBPERIPH_TSIRST; 
}

/**
  * @brief  Initialize the TSI.
  * @param  TSI_InitParaStruct: the sturct TSI_InitPara pointer.
  * @retval None
  */
void TSI_BaseInit( TSI_BaseInitPara* TSI_BaseInitParaStruct )
{
    uint32_t temp = 0;
    
    temp  = TSI_BaseInitParaStruct->TSI_TriggerMode    |
            TSI_BaseInitParaStruct->TSI_EageSelect     |
            TSI_BaseInitParaStruct->TSI_MaxCycle       |
            TSI_BaseInitParaStruct->TSI_PulsePrescaler |
            TSI_BaseInitParaStruct->TSI_pulseLow       |
            TSI_BaseInitParaStruct->TSI_pulseHigh  ;

    /* Initialize the TSI and enable the TSI */
    TSI->CTLR = temp | TSI_CTLR_TSIEN;
}

/**
  * @brief  Fill each TSI_BaseInitPara Struct member with a default value.
  * @param  TSI_BaseInitStruct: the sturct TSI_BaseInitPara pointer.
  * @retval None
  */
void TSI_BaseStructInit( TSI_BaseInitPara * TSI_BaseInitParaStruct )
{
    /* fill the default value */
    TSI_BaseInitParaStruct->TSI_PulsePrescaler = TSI_CTCDIV_1    ;
    TSI_BaseInitParaStruct->TSI_pulseLow       = TSI_CTDT_2CTCLK ;
    TSI_BaseInitParaStruct->TSI_pulseHigh      = TSI_CDT_2CTCLK  ;
    TSI_BaseInitParaStruct->TSI_TriggerMode    = TSI_SOFTWARE_TRIGGLER ;
    TSI_BaseInitParaStruct->TSI_EageSelect     = TSI_RISING_EDGE_TRIGGLER ;
    TSI_BaseInitParaStruct->TSI_MaxCycle       = TSI_MCN_4095_CYCLE ;
}

/**
  * @brief  Initialize the specified Group 
  * @param  TSI_GroupInitStruct: the sturct TSI_GroupInitPara pointer.
  * @retval None
  */
void TSI_GroupInit( TSI_GroupInitPara * TSI_GroupInitParaStruct )
{
    TSI->GCTLR |= 1 << TSI_GroupInitParaStruct->TSI_Group ; 
    TSI->SPR    = TSI_GroupInitParaStruct->TSI_SamplePin ; 
    TSI->CPR    = TSI_GroupInitParaStruct->TSI_ChannelPin ; 
}

/**
  * @brief  Configure the TSI Spread Spectrum function.
  * @param  TSI_SpreadSpectrumInitParaStruct : pointer to a TSI_SpreadSpectrumInitPara structure that
  *         contains the information to configure the Spread Spectrum.
  * @retval None
  */
void TSI_SpreadSpectrumInit( TSI_SpreadSpectrumInitPara * TSI_SpreadSpectrumInitParaStruct )
{
    TSI->CTLR = TSI_SpreadSpectrumInitParaStruct->TSI_SSPrescaler |
                TSI_SpreadSpectrumInitParaStruct->TSI_SSDeviation |
                TSI_CTLR_ECEN;
}

/**
  * @brief  Enable or disable the TSI interface.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_Enable( TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Enable the TSI interface */
        TSI->CTLR |= TSI_CTLR_TSIEN;
    }
    else
    {
        /* Disable the TSI interface */
        TSI->CTLR &= ~TSI_CTLR_TSIEN;
    }
}

/**
  * @brief  Start a new acquisition.
  * @param  NewValue: ENABLE or DISABLE.
  *     @arg ENABLE : Start a new acquisition.
  *     @arg DISABLE: Cancel the on-going acquisition.
  * @retval None
  */
void TSI_StartAcquisition( TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Start a new acquisition */
        TSI->CTLR |= TSI_CTLR_TSIST; 
    }
    else
    {
        /* Cancel the on-going acquisition */
        TSI->CTLR &= ~TSI_CTLR_TSIST;
    }
}

/**
  * @brief  Acquisition Trigger Mode Configure.
  * @param  TSI_TriggerMode: new value of the TSI interface.
  *   This value will be:
  *     @arg TSI_SOFTWARE_TRIGGLER: Software Trigger Mode
  *     @arg TSI_HARDWARE_TRIGGLER: Hardware Trigger Mode
  * @retval None
  */
void TSI_TriggerModeConfig( TypeState TSI_TriggerMode )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        uint32_t temp = TSI->CTLR;
        
        /* Clear Trigger Mode */
        temp &= ~TSI_CTLR_TM;
        
        temp |= TSI_TriggerMode;
        
        TSI->CTLR = temp;
    }
}

/**
  * @brief  Configure the hardware trigger edge mode.
  * @param  TSI_TriggerEdge: new value of the TSI interface.
  *   This value will be:
  *     @arg TSI_RISING_EDGE_TRIGGLER : Rising edge and high level
  *     @arg TSI_FALLING_EDGE_TRIGGLER: Falling edge only
  * @retval None
  */
void TSI_TriggerEdgeConfig( TypeState TSI_TriggerEdge )
{
    uint32_t temp = TSI->CTLR;
    
    /* Clear edge Selected */
    temp &= ~TSI_CTLR_ES;
    
    temp |= TSI_TriggerEdge; 
    
    TSI->CTLR = temp;
}

/**
  * @brief  Configure the I/O Default mode.
  * @param  NewValue: new value of the TSI interface.
  *   This value will be:
  *     @arg ENABLE : I/Os are in input floating
  *     @arg DISABLE: I/Os are forced to output push-pull low
  * @retval None
  */
void TSI_IOModeConfig( TypeState NewValue )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        if ( NewValue != DISABLE )
        {
            /* Input floating */
            TSI->CTLR |= TSI_CTLR_PINMOD; 
        }
        else
        {
            /* Forced to output push-pull low */
            TSI->CTLR &= ~TSI_CTLR_PINMOD;
        }
    }
}

/**
  * @brief  Set the TSI charge-transfer Max Cycle.
  * @param  TSI_MCN: new value of the TSI interface.
  *   This value will be:
  *     @arg TSI_MCN_255_CYCLE
  *     @arg TSI_MCN_511_CYCLE
  *     @arg TSI_MCN_1023_CYCLE
  *     @arg TSI_MCN_2047_CYCLE
  *     @arg TSI_MCN_4095_CYCLE
  *     @arg TSI_MCN_8191_CYCLE
  *     @arg TSI_MCN_16383_CYCLE
  * @retval None
  */
void TSI_SetMaxCycle( uint32_t TSI_MCN )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        uint32_t temp = TSI->CTLR;
        
        /* Clear Max Cycle */
        temp &= ~TSI_CTLR_MCN; 
        
        /* Set the Max Cycle */
        temp |= TSI_MCN;
        
        TSI->CTLR = temp;
    }
}


/**
  * @brief  Configure the TSI pulse generator prescaler.
  * @param  TSI_CTCDIV: CTCLK Clock Division value.
  *   This value will be:
  *     @arg TSI_CTCDIV_1
  *     @arg TSI_CTCDIV_2
  *     @arg TSI_CTCDIV_4
  *     @arg TSI_CTCDIV_8
  *     @arg TSI_CTCDIV_16
  *     @arg TSI_CTCDIV_32
  *     @arg TSI_CTCDIV_64
  *     @arg TSI_CTCDIV_128
  * @retval None
  */
void TSI_PulseGenDIV( uint32_t TSI_CTCDIV )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        /* Clear the CTCLK Clock Division value */
        TSI->CTLR &= ~TSI_CTLR_CTCDIV; 
        
        /* Set the CTCLK Clock Division value  */
        TSI->CTLR |= TSI_CTCDIV;
    }
}

/**
  * @brief  Configure the TSI Spread spectrum prescaler.
  * @param  TSI_ECDIV: ECCLK clock division factor.
  *     @arg ECCLK_DIV_0 fHClk
  *     @arg ECCLK_DIV_1 fHClk /2
  * @retval None
  */
void TSI_ECCLKDivConfig( ECCLKDIV_TypeDef TSI_ECDIV )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        if( TSI_ECDIV != ECCLK_DIV_0 )
        {
            /* Set the ECCLK Clock Division value */
            TSI->CTLR |= TSI_CTLR_ECDIV;
        }
        else
        {
            /* Clear the ECCLK Clock Division value */
            TSI->CTLR &= TSI_CTLR_ECDIV; 
        }
    }
}

/**
  * @brief  Configure the TSI Spread spectrum enable.
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_ExtendChargeConfig( TypeState NewValue )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        if ( NewValue != DISABLE )
        {
            TSI->CTLR |= TSI_CTLR_ECEN;
        }
        else
        {
            TSI->CTLR &= TSI_CTLR_ECEN; 
        }    
    }
}

/**
  * @brief  Configure the TSI Spread spectrum deviation.
  * @param  TSI_ECDT: Spread spectrum deviation.
  *   This value will be:
  *     @arg TSI_ECDT_1ECCLK      Spread spectrum deviation   1x tSSCLK
  *     @arg TSI_ECDT_2ECCLK      Spread spectrum deviation   2x tSSCLK
  *     @arg TSI_ECDT_3ECCLK      Spread spectrum deviation   3x tSSCLK
  *     @arg TSI_ECDT_4ECCLK      Spread spectrum deviation   4x tSSCLK
  *     @arg TSI_ECDT_5ECCLK      Spread spectrum deviation   5x tSSCLK
  *     @arg TSI_ECDT_6ECCLK      Spread spectrum deviation   6x tSSCLK
  *     @arg TSI_ECDT_7ECCLK      Spread spectrum deviation   7x tSSCLK
  *     @arg TSI_ECDT_8ECCLK      Spread spectrum deviation   8x tSSCLK
  *     @arg TSI_ECDT_9ECCLK      Spread spectrum deviation   9x tSSCLK
  *     @arg TSI_ECDT_10ECCLK     Spread spectrum deviation  10x tSSCLK
  *     @arg TSI_ECDT_16ECCLK     Spread spectrum deviation  16x tSSCLK
  *     @arg TSI_ECDT_32ECCLK     Spread spectrum deviation  32x tSSCLK
  *     @arg TSI_ECDT_64ECCLK     Spread spectrum deviation  64x tSSCLK
  *     @arg TSI_ECDT_128ECCLK    Spread spectrum deviation 128x tSSCLK
  * @retval None
  */
void TSI_ExtendChargeMaxTime( uint32_t TSI_ECDT )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        uint32_t temp = TSI->CTLR;
        
        /* Clear the Extend Charge State Maximum Duration Time */
        temp &= ~TSI_CTLR_ECDT; 
        
        /* Set the Extend Charge State Maximum Duration Time */
        temp |= TSI_ECDT;
        
        TSI->CTLR = temp;
    }
}

/**
  * @brief  Configure the TSI Charge Transfer State Duration Time
  * @param  TSI_CTDT: Charge Transfer State Duration Time
  *   This value will be:
  *     @arg TSI_CTDT_1CTCLK   Charge Transfer State Duration Time 1 ¡Á tCTCLK 
  *     @arg TSI_CTDT_2CTCLK   Charge Transfer State Duration Time 2 ¡Á tCTCLK 
  *     @arg TSI_CTDT_3CTCLK   Charge Transfer State Duration Time 3 ¡Á tCTCLK 
  *     @arg TSI_CTDT_4CTCLK   Charge Transfer State Duration Time 4 ¡Á tCTCLK 
  *     @arg TSI_CTDT_5CTCLK   Charge Transfer State Duration Time 5 ¡Á tCTCLK 
  *     @arg TSI_CTDT_6CTCLK   Charge Transfer State Duration Time 6 ¡Á tCTCLK 
  *     @arg TSI_CTDT_7CTCLK   Charge Transfer State Duration Time 7 ¡Á tCTCLK 
  *     @arg TSI_CTDT_8CTCLK   Charge Transfer State Duration Time 8 ¡Á tCTCLK 
  *     @arg TSI_CTDT_9CTCLK   Charge Transfer State Duration Time 9 ¡Á tCTCLK 
  *     @arg TSI_CTDT_10CTCLK  Charge Transfer State Duration Time 10 ¡Á tCTCLK 
  *     @arg TSI_CTDT_11CTCLK  Charge Transfer State Duration Time 11 ¡Á tCTCLK 
  *     @arg TSI_CTDT_12CTCLK  Charge Transfer State Duration Time 12 ¡Á tCTCLK 
  *     @arg TSI_CTDT_13CTCLK  Charge Transfer State Duration Time 13 ¡Á tCTCLK 
  *     @arg TSI_CTDT_14CTCLK  Charge Transfer State Duration Time 14 ¡Á tCTCLK 
  *     @arg TSI_CTDT_15CTCLK  Charge Transfer State Duration Time 15 ¡Á tCTCLK 
  *     @arg TSI_CTDT_16CTCLK  Charge Transfer State Duration Time 16 ¡Á tCTCLK 
  * @retval None
  */
void TSI_SetChargeTransferTime( uint32_t TSI_CTDT )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        uint32_t temp = TSI->CTLR;
        
        /* Clear the Extend Charge State Maximum Duration Time */
        temp &= ~TSI_CTLR_CTDT; 
        
        /* Set the Extend Charge State Maximum Duration Time  */
        temp |= TSI_CTDT;
        
        TSI->CTLR = temp;
    }
}

/**
  * @brief  Configure the TSI Charge State Duration Time
  * @param  TSI_CDT: Charge State Duration Time.
  *   This value will be:
  *     @arg TSI_CDT_1CTCLK   Charge State Duration Time 1 ¡Á tCTCLK
  *     @arg TSI_CDT_2CTCLK   Charge State Duration Time 2 ¡Á tCTCLK
  *     @arg TSI_CDT_3CTCLK   Charge State Duration Time 3 ¡Á tCTCLK
  *     @arg TSI_CDT_4CTCLK   Charge State Duration Time 4 ¡Á tCTCLK
  *     @arg TSI_CDT_5CTCLK   Charge State Duration Time 5 ¡Á tCTCLK
  *     @arg TSI_CDT_6CTCLK   Charge State Duration Time 6 ¡Á tCTCLK
  *     @arg TSI_CDT_7CTCLK   Charge State Duration Time 7 ¡Á tCTCLK
  *     @arg TSI_CDT_8CTCLK   Charge State Duration Time 8 ¡Á tCTCLK
  *     @arg TSI_CDT_9CTCLK   Charge State Duration Time 9 ¡Á tCTCLK
  *     @arg TSI_CDT_10CTCLK  Charge State Duration Time 10 ¡Á tCTCLK
  *     @arg TSI_CDT_11CTCLK  Charge State Duration Time 11 ¡Á tCTCLK
  *     @arg TSI_CDT_12CTCLK  Charge State Duration Time 12 ¡Á tCTCLK
  *     @arg TSI_CDT_13CTCLK  Charge State Duration Time 13 ¡Á tCTCLK
  *     @arg TSI_CDT_14CTCLK  Charge State Duration Time 14 ¡Á tCTCLK
  *     @arg TSI_CDT_15CTCLK  Charge State Duration Time 15 ¡Á tCTCLK
  *     @arg TSI_CDT_16CTCLK  Charge State Duration Time 16 ¡Á tCTCLK   
  * @retval None
  */
void TSI_SetChargeTime( uint32_t TSI_CDT )
{
    if( ( TSI->CTLR & TSI_CTLR_TSIST ) == RESET )
    {
        uint32_t temp = TSI->CTLR;
        
        /* Clear the Charge State Duration Time */
        temp &= ~TSI_CTLR_CDT; 
        
        /* Set the Charge State Duration Time  */
        temp |= TSI_CDT;
        
        TSI->CTLR = temp;
    }
}

/**
  * @brief  Enable or disable the specified TSI interrupts.
  * @param  TSI_INT: Specifies TSI interrupt sources. 
  *   This parameter can be the following values:
  *     @arg TSI_IER_CRCFIE: Max Count Error interrupt mask
  *     @arg TSI_IER_MCEIE : End of acquisition interrupt mask 
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_INTConfig( uint8_t TSI_INT, TypeState NewValue )
{
    uint8_t temp_it = 0;
    
    /* TSI INT old state */
    temp_it = (uint8_t)TSI_INT;
    
    if ( NewValue != DISABLE )
    {
        /* Enable the selected TSI interrupts */
        TSI->IER |= temp_it;
    }
    else
    {
        /* Disable the selected TSI interrupts */
        TSI->IER &= (~(uint32_t)temp_it);
    }
}

/**
  * @brief  Clear the TSI's interrupt pending bits.
  * @param  TSI_INT: Specifies TSI interrupt sources. 
  *   This parameter can be the following values:
  *     @arg TSI_CEFR_CMCE : Max Count Error interrupt 
  *     @arg TSI_CEFR_CCTCF: End of acquisition interrupt 
  * @retval None
  */
void TSI_ClearIntBitState( uint8_t TSI_INT )
{
    /* Clear TSI interrupt pending bits */
    TSI->CEFR |= (uint8_t) TSI_INT;
}

/**
  * @brief  Check the TSI flag.
  * @param  TSI_FLAG: the flag to check. 
  *   This parameter can be the following values:
  *     @arg TSI_STR_CTCF: Max Count Error interrupt 
  *     @arg TSI_STR_MCE : End of acquisition interrupt 
  * @retval TSI_FLAG state( SET or RESET ).
  */
TypeState TSI_GetIntBitState(uint8_t TSI_FLAG)
{
    /* Check the specified TSI flag state */
    if (( TSI->STR & TSI_FLAG ) != (uint8_t)RESET )
    {
        /* TSI_FLAG is set */
        return SET;
    }
    else
    {
        /* TSI_FLAG is reset */
        return RESET;
    }
}

/**
  * @brief  Configure the TSI Pin Schmitt trigger hysteresis mode.
  * @param  TSI_Pin: TSI Pin choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1_PIN1: TSI Group 1 Pin 1
  *     @arg TSI_GROUP1_PIN2: TSI Group 1 Pin 2
  *     @arg TSI_GROUP1_PIN3: TSI Group 1 Pin 3
  *     @arg TSI_GROUP1_PIN4: TSI Group 1 Pin 4
  *     @arg TSI_GROUP2_PIN1: TSI Group 2 Pin 1
  *     @arg TSI_GROUP2_PIN2: TSI Group 2 Pin 2
  *     @arg TSI_GROUP2_PIN3: TSI Group 2 Pin 3
  *     @arg TSI_GROUP2_PIN4: TSI Group 2 Pin 4
  *     @arg TSI_GROUP3_PIN1: TSI Group 3 Pin 1
  *     @arg TSI_GROUP3_PIN2: TSI Group 3 Pin 2
  *     @arg TSI_GROUP3_PIN3: TSI Group 3 Pin 3
  *     @arg TSI_GROUP3_PIN4: TSI Group 3 Pin 4
  *     @arg TSI_GROUP4_PIN1: TSI Group 4 Pin 1
  *     @arg TSI_GROUP4_PIN2: TSI Group 4 Pin 2
  *     @arg TSI_GROUP4_PIN3: TSI Group 4 Pin 3
  *     @arg TSI_GROUP4_PIN4: TSI Group 4 Pin 4
  *     @arg TSI_GROUP5_PIN1: TSI Group 5 Pin 1
  *     @arg TSI_GROUP5_PIN2: TSI Group 5 Pin 2
  *     @arg TSI_GROUP5_PIN3: TSI Group 5 Pin 3
  *     @arg TSI_GROUP5_PIN4: TSI Group 5 Pin 4
  *     @arg TSI_GROUP6_PIN1: TSI Group 6 Pin 1
  *     @arg TSI_GROUP6_PIN2: TSI Group 6 Pin 2
  *     @arg TSI_GROUP6_PIN3: TSI Group 6 Pin 3
  *     @arg TSI_GROUP6_PIN4: TSI Group 6 Pin 4
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_PinHysteresisConfig( uint32_t TSI_Pin , TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Enable the selected pin Schmitt trigger hysteresis mode */
        TSI->PHMR |= TSI_Pin;
    }
    else
    {
        /* Disable the selected pin Schmitt trigger hysteresis mode */
        TSI->PHMR &= (~(uint32_t)TSI_Pin );
    }
}

/**
  * @brief  Configure the TSI Pin Analog Mode configure.
  * @param  TSI_Pin: TSI Pin choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1_PIN1: TSI Group 1 Pin 1
  *     @arg TSI_GROUP1_PIN2: TSI Group 1 Pin 2
  *     @arg TSI_GROUP1_PIN3: TSI Group 1 Pin 3
  *     @arg TSI_GROUP1_PIN4: TSI Group 1 Pin 4
  *     @arg TSI_GROUP2_PIN1: TSI Group 2 Pin 1
  *     @arg TSI_GROUP2_PIN2: TSI Group 2 Pin 2
  *     @arg TSI_GROUP2_PIN3: TSI Group 2 Pin 3
  *     @arg TSI_GROUP2_PIN4: TSI Group 2 Pin 4
  *     @arg TSI_GROUP3_PIN1: TSI Group 3 Pin 1
  *     @arg TSI_GROUP3_PIN2: TSI Group 3 Pin 2
  *     @arg TSI_GROUP3_PIN3: TSI Group 3 Pin 3
  *     @arg TSI_GROUP3_PIN4: TSI Group 3 Pin 4
  *     @arg TSI_GROUP4_PIN1: TSI Group 4 Pin 1
  *     @arg TSI_GROUP4_PIN2: TSI Group 4 Pin 2
  *     @arg TSI_GROUP4_PIN3: TSI Group 4 Pin 3
  *     @arg TSI_GROUP4_PIN4: TSI Group 4 Pin 4
  *     @arg TSI_GROUP5_PIN1: TSI Group 5 Pin 1
  *     @arg TSI_GROUP5_PIN2: TSI Group 5 Pin 2
  *     @arg TSI_GROUP5_PIN3: TSI Group 5 Pin 3
  *     @arg TSI_GROUP5_PIN4: TSI Group 5 Pin 4
  *     @arg TSI_GROUP6_PIN1: TSI Group 6 Pin 1
  *     @arg TSI_GROUP6_PIN2: TSI Group 6 Pin 2
  *     @arg TSI_GROUP6_PIN3: TSI Group 6 Pin 3
  *     @arg TSI_GROUP6_PIN4: TSI Group 6 Pin 4
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_AnalogModeConfig( uint32_t TSI_Pin , TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Enable the selected pin Analog Mode */
        TSI->ASWR |= TSI_Pin;
    }
    else
    {
        /* Disable the selected pin Analog Mode */
        TSI->ASWR &= (~(uint32_t)TSI_Pin );
    }                   
}

/**
  * @brief  Enable the TSI Pin sample Mode.
  * @param  TSI_Pin: TSI Pin choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1_PIN1: TSI Group 1 Pin 1
  *     @arg TSI_GROUP1_PIN2: TSI Group 1 Pin 2
  *     @arg TSI_GROUP1_PIN3: TSI Group 1 Pin 3
  *     @arg TSI_GROUP1_PIN4: TSI Group 1 Pin 4
  *     @arg TSI_GROUP2_PIN1: TSI Group 2 Pin 1
  *     @arg TSI_GROUP2_PIN2: TSI Group 2 Pin 2
  *     @arg TSI_GROUP2_PIN3: TSI Group 2 Pin 3
  *     @arg TSI_GROUP2_PIN4: TSI Group 2 Pin 4
  *     @arg TSI_GROUP3_PIN1: TSI Group 3 Pin 1
  *     @arg TSI_GROUP3_PIN2: TSI Group 3 Pin 2
  *     @arg TSI_GROUP3_PIN3: TSI Group 3 Pin 3
  *     @arg TSI_GROUP3_PIN4: TSI Group 3 Pin 4
  *     @arg TSI_GROUP4_PIN1: TSI Group 4 Pin 1
  *     @arg TSI_GROUP4_PIN2: TSI Group 4 Pin 2
  *     @arg TSI_GROUP4_PIN3: TSI Group 4 Pin 3
  *     @arg TSI_GROUP4_PIN4: TSI Group 4 Pin 4
  *     @arg TSI_GROUP5_PIN1: TSI Group 5 Pin 1
  *     @arg TSI_GROUP5_PIN2: TSI Group 5 Pin 2
  *     @arg TSI_GROUP5_PIN3: TSI Group 5 Pin 3
  *     @arg TSI_GROUP5_PIN4: TSI Group 5 Pin 4
  *     @arg TSI_GROUP6_PIN1: TSI Group 6 Pin 1
  *     @arg TSI_GROUP6_PIN2: TSI Group 6 Pin 2
  *     @arg TSI_GROUP6_PIN3: TSI Group 6 Pin 3
  *     @arg TSI_GROUP6_PIN4: TSI Group 6 Pin 4
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_SampleModeConfig( uint32_t TSI_Pin , TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Enable the selected pin as Sample pin */
        TSI->SPR |= TSI_Pin;
    }
    else
    {
        /* Disable the selected  pin as Sample pin */
        TSI->SPR &= (~(uint32_t)TSI_Pin);
    }     
}

/**
  * @brief  Configure the TSI Pin Channel Mode configure.
  * @param  TSI_Pin: TSI Pin choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1_PIN1: TSI Group 1 Pin 1
  *     @arg TSI_GROUP1_PIN2: TSI Group 1 Pin 2
  *     @arg TSI_GROUP1_PIN3: TSI Group 1 Pin 3
  *     @arg TSI_GROUP1_PIN4: TSI Group 1 Pin 4
  *     @arg TSI_GROUP2_PIN1: TSI Group 2 Pin 1
  *     @arg TSI_GROUP2_PIN2: TSI Group 2 Pin 2
  *     @arg TSI_GROUP2_PIN3: TSI Group 2 Pin 3
  *     @arg TSI_GROUP2_PIN4: TSI Group 2 Pin 4
  *     @arg TSI_GROUP3_PIN1: TSI Group 3 Pin 1
  *     @arg TSI_GROUP3_PIN2: TSI Group 3 Pin 2
  *     @arg TSI_GROUP3_PIN3: TSI Group 3 Pin 3
  *     @arg TSI_GROUP3_PIN4: TSI Group 3 Pin 4
  *     @arg TSI_GROUP4_PIN1: TSI Group 4 Pin 1
  *     @arg TSI_GROUP4_PIN2: TSI Group 4 Pin 2
  *     @arg TSI_GROUP4_PIN3: TSI Group 4 Pin 3
  *     @arg TSI_GROUP4_PIN4: TSI Group 4 Pin 4
  *     @arg TSI_GROUP5_PIN1: TSI Group 5 Pin 1
  *     @arg TSI_GROUP5_PIN2: TSI Group 5 Pin 2
  *     @arg TSI_GROUP5_PIN3: TSI Group 5 Pin 3
  *     @arg TSI_GROUP5_PIN4: TSI Group 5 Pin 4
  *     @arg TSI_GROUP6_PIN1: TSI Group 6 Pin 1
  *     @arg TSI_GROUP6_PIN2: TSI Group 6 Pin 2
  *     @arg TSI_GROUP6_PIN3: TSI Group 6 Pin 3
  *     @arg TSI_GROUP6_PIN4: TSI Group 6 Pin 4
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_ChannelModeConfig( uint32_t TSI_Pin , TypeState NewValue )
{
    if ( NewValue != DISABLE )
    {
        /* Enable the selected pin as Channel pin */
        TSI->CPR |= TSI_Pin;
    }
    else
    {
        /* Disable the selected pin as Channel pin */
        TSI->CPR &= (~(uint32_t)TSI_Pin );
    }
}

/**
  * @brief  Enable the TSI Group .
  * @param  TSI_GROUP: TSI GROUP choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1 : TSI Group 1 
  *     @arg TSI_GROUP2 : TSI Group 2 
  *     @arg TSI_GROUP3 : TSI Group 3 
  *     @arg TSI_GROUP4 : TSI Group 4 
  *     @arg TSI_GROUP5 : TSI Group 5 
  *     @arg TSI_GROUP6 : TSI Group 6 
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None
  */
void TSI_GroupSampleConfig( TSI_GROUP_TypeDef TSI_GROUP , TypeState NewValue )
{
    uint8_t temp = 0;

    /* TSI Group choose */
    temp = (uint8_t)( 1 << TSI_GROUP );

    if ( NewValue != DISABLE )
    {
        /* Enable the selected GROUP */
        TSI->GCTLR |= temp;
    }
    else
    {
        /* Disable the selected GROUP */
        TSI->GCTLR &= (~(uint32_t)temp);
    }
}

/**
  * @brief  Get the TSI Group Sample State.
  * @param  TSI_GROUP: TSI GROUP choose
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1 : TSI Group 1 
  *     @arg TSI_GROUP2 : TSI Group 2 
  *     @arg TSI_GROUP3 : TSI Group 3 
  *     @arg TSI_GROUP4 : TSI Group 4 
  *     @arg TSI_GROUP5 : TSI Group 5 
  *     @arg TSI_GROUP6 : TSI Group 6 
  * @retval TSI GROUP Sample complete or not. 
  */
TypeState TSI_GetGroupSampleState( TSI_GROUP_TypeDef TSI_GROUP )
{
    uint32_t temp = 0;

    /* TSI Group choose */
    temp = (uint32_t)( 1 << ( 16 + TSI_GROUP ) );

    /* Check the specified TSI Group Sample complete state */
    if (( TSI->GCTLR & temp ) != (uint8_t)RESET )
    {
        /* TSI Sample complete state is set */
        return SET;
    }
    else
    {
        /* TSI Sample complete state is reset */
        return RESET;
    }
}


/**
  * @brief  Return the TSI GROUP Sample cycle number.
  * @param  TSI_GROUP: TSI GROUP choose.
  *   This parameter can be as follows:
  *     @arg TSI_GROUP1 : TSI Group 1 
  *     @arg TSI_GROUP2 : TSI Group 2 
  *     @arg TSI_GROUP3 : TSI Group 3 
  *     @arg TSI_GROUP4 : TSI Group 4 
  *     @arg TSI_GROUP5 : TSI Group 5 
  *     @arg TSI_GROUP6 : TSI Group 6 
  * @retval Sample cycle number.
  */
uint16_t TSI_GetGroupSampleCycle( TSI_GROUP_TypeDef TSI_GROUP )
{
    /* Return TSI conversion data */
    return ( uint16_t) TSI->GXCYCNR[ TSI_GROUP ];
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

/**
  ******************************************************************************
  * @file    gd32f1x0_FMC.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the FMC peripheral:
  *            - FMC Memory Programming
  *            - Option Bytes Programming
  *            - Interrupts and flags management  
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_fmc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup FMC 
  * @brief FMC driver modules
  * @{
  */ 
 
/** @defgroup FMC_Private_Functions
  * @{
  */ 

/** @defgroup FMC_Group1 FMC Memory Programming functions
 *  @brief    FMC Memory Programming functions
 *
@verbatim   
 ===============================================================================
                ##### FMC Memory Programming functions #####
 ===============================================================================

    [..] The FMC Memory Programming functions, includes the following functions:
       (+) void FMC_Unlock(void);
       (+) void FMC_Lock(void);
       (+) FMC_State FMC_ErasePage(uint32_t Page_Address);
       (+) FMC_State FMC_MassErase(void);
       (+) FMC_State FMC_ProgramWord(uint32_t Address, uint32_t Data);

    [..] Any operation of erase or program should follow these steps:
       
       (#) Call the FMC_Unlock() function to unlock the FMC operation
       (#) Call erase or program data
       (#) Call the FMC_Lock() to lock the FMC operation

@endverbatim
  * @{
  */

/**
  * @brief  Unlock the main FMC operation.
  * @param  None
  * @retval None
  */
void FMC_Unlock(void)
{
    if((FMC->CMR & FMC_CMR_LK) != RESET)
    {
        /* Write the FMC key */
        FMC->UKEYR = FMC_KEY1;
        FMC->UKEYR = FMC_KEY2;
    }
}

/**
  * @brief  Lock the main FMC operation.
  * @param  None
  * @retval None
  */
void FMC_Lock(void)
{
    /* Set the LOCK bit*/
    FMC->CMR |= FMC_CMR_LK;
}

/**
  * @brief  Erase a page.
  * @param  Page_Address: The page address to be erased.
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_ErasePage(uint32_t Page_Address)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    { 
        /* Start page erase */
        FMC->CMR |= FMC_CMR_PE;
        FMC->AR  = Page_Address;
        FMC->CMR |= FMC_CMR_START;
    
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
    
        /* Reset the PE bit */
        FMC->CMR &= ~FMC_CMR_PE;
    }
    /* Return the FMC state  */
    return temp_state;
}

/**
  * @brief  Erase all main FMC.
  * @param  None
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_MassErase(void)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {
        /* Start chip erase */
        FMC->CMR |= FMC_CMR_ME;
        FMC->CMR |= FMC_CMR_START;    
    
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        /* Reset the MER bit */
        FMC->CMR &= ~FMC_CMR_ME;
    }

    /* Return the FMC state  */
    return temp_state;
}


/**
  * @brief  Program a word at the corresponding address.
  * @param  Address: The address to be programmed.
  * @param  Data: The data to be programmed.
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_ProgramWord(uint32_t Address, uint32_t Data)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {
        /* Set the PG bit to start program */
        FMC->CMR |= FMC_CMR_PG;
  
        *(__IO uint32_t*)Address = Data;

        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
    
        /* Reset the PG bit */
        FMC->CMR &= ~FMC_CMR_PG;
    } 
  
    /* Return the FMC state */
    return temp_state;
}

/**
  * @}
  */
  
/** @defgroup FMC_Group2 Option Bytes Programming functions
 *  @brief   Option Bytes Programming functions 
 *
@verbatim   
 ===============================================================================
                ##### Option Bytes Programming functions #####
 ===============================================================================

    [..] The FMC_Option Bytes Programming_functions, includes the following functions:
       (+) void FMC_OB_Unlock(void);
       (+) void FMC_OB_Lock(void);
       (+) void FMC_OB_Reset(void);
       (+) FMC_State FMC_OB_Erase(void);
       (+) FMC_State FMC_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
       (+) FMC_State FMC_OB_RDPConfig(uint8_t OB_RDP);
       (+) FMC_State FMC_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_DEEPSLEEP, uint8_t OB_STDBY);
       (+) FMC_State FMC_OB_BOOTConfig(uint8_t OB_BOOT1);
       (+) FMC_State FMC_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG);
       (+) FMC_State FMC_OB_WriteUser(uint8_t OB_USER);
       (+) FMC_ProgramOptionByteData(uint32_t Address, uint8_t Data);
       (+) uint8_t FMC_OB_GetUser(void);
       (+) uint32_t FMC_OB_GetWRP(void);
       (+) FlagStatus FMC_OB_GetRDP(void);

    [..] Any operation of erase or program should follow these steps:

   (#) Call the FMC_OB_Unlock() function to enable the Option Bytes registers access

   (#) Call one or several functions to program the desired option bytes 
      (++) FMC_State FMC_OB_RDPConfig(uint8_t OB_RDP) => to set the desired read Protection Level
      (++) FMC_State FMC_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState) 
           => to Enable/Disable the desired sector write protection
      (++) FMC_State FMC_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_DEEPSLEEP, uint8_t OB_STDBY) 
           => to configure the user option Bytes: IWDG, DEEPSLEEP and the Standby.
      (++) FMC_State FMC_OB_BOOTConfig(uint8_t OB_BOOT1)
           => to set or reset BOOT1 
      (++) FMC_State FMC_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG) 
           => to enable or disable the VDDA Analog Monitoring 			 
      (++) You can write all User Options bytes at once using a single function
           by calling FMC_State FMC_OB_WriteUser(uint8_t OB_USER)
      (++) FMC_ProgramOptionByteData(uint32_t Address, uint8_t Data) to program the 
           two half word in the option bytes

   (#) Once all needed option bytes to be programmed are correctly written, call the
      FMC_OB_Launch(void) function to launch the Option Bytes programming process.

   (#) Call the FMC_OB_Lock() to disable the Option Bytes registers access (recommended
      to protect the option Bytes against possible unwanted operations)

@endverbatim
  * @{
  */

/**
  * @brief  Unlock the option byte operation
  * @param  None
  * @retval None
  */
void FMC_OB_Unlock(void)
{
    if((FMC->CMR & FMC_CMR_OBWE) == RESET)
    { 
        /* Write the FMC key */
        FMC->OBKEYR = FMC_KEY1;
        FMC->OBKEYR = FMC_KEY2;
    }
}

/**
  * @brief  Lock the option byte operation.
  * @param  None
  * @retval None
  */
void FMC_OB_Lock(void)
{
    /* Reset the OBWE bit */
    FMC->CMR &= ~FMC_CMR_OBWE;
}

/**
  * @brief  Generate a system reset to reload the option byte.
  * @param  None
  * @retval None
  */
void FMC_OB_Reset(void)
{
    /* Set the OPTR bit */
    FMC->CMR |= FMC_CMR_OPTR;
}

/**
  * @brief  Erase the FMC option byte.
  * @param  None
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_Erase(void)
{
    uint16_t temp_rdp = RDP_Level_0;

    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

    /* Check the ReadOut Protection Option Byte */
    if(FMC_OB_GetRDP() != RESET)
    {
        temp_rdp = 0x00;  
    }

    if(temp_state == FMC_READY)
    {   
        /* Start erase the option byte */
        FMC->CMR |= FMC_CMR_OBER;
        FMC->CMR |= FMC_CMR_START;

        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
    
        if(temp_state == FMC_READY)
        {
            /* Reset the OPER bit */
            FMC->CMR &= ~FMC_CMR_OBER;
       
            /* Set the OBPG bit */
            FMC->CMR |= FMC_CMR_OBPG;

            /* Set default RDP level */
            OB->RDP = (uint16_t)temp_rdp; 

            /* Wait for the FMC ready */
            temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
 
            if(temp_state != FMC_TIMEOUT_ERR)
            {
                /* Reset the OBPG bit */
                FMC->CSR &= ~FMC_CMR_OBPG;
            }
        }
        else
        {
            if (temp_state != FMC_TIMEOUT_ERR)
            {
                /* Reset the OBPG bit */
                FMC->CMR &= ~FMC_CMR_OBPG;
            }
        }  
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program Write protect Byte
  * @param  OB_WRP: specify the address of the pages to be write protected.
  *   The legal parameter can be:
  *     @arg WRP_sector0 ... WRP_sector15
  *     @arg WRP_Allsectors
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_EnableWRP(uint32_t OB_WRP)
{
    uint16_t temp_WRP0, temp_WRP1;

    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

    OB_WRP = (uint32_t)(~OB_WRP);
    temp_WRP0 = (uint16_t)(OB_WRP & OB_WRP0_WRP0);
    temp_WRP1 = (uint16_t)((OB_WRP & OB_WRP0_nWRP0) >> 8);

    if(temp_state == FMC_READY)
    {
        /* Set the OBPG bit*/
        FMC->CMR |= FMC_CMR_OBPG;

        if(temp_WRP0 != 0xFF)
        {
            OB->WRP0 = temp_WRP0;
      
            /* Wait for the FMC ready */
            temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
        }
        if((temp_state == FMC_READY) && (temp_WRP1 != 0xFF))
        {
            OB->WRP1 = temp_WRP1;
      
            /* Wait for the FMC ready */
            temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
        }
        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    } 
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Config the Read Out Protection bit.
  * @param  FMC_ReadProtection_Level: The Read Out Protection level.
  *   This parameter can be:
  *     @arg RDP_Level_0: No protection
  *     @arg RDP_Level_1: Read Outprotection of the memory
  *     @arg RDP_Level_2: Chip protection
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_RDPConfig(uint8_t OB_RDP)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

    if(temp_state == FMC_READY)
    {
        FMC->CMR |= FMC_CMR_OBER;
        FMC->CMR |= FMC_CMR_START;
    
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
    
        if(temp_state == FMC_READY)
        {
            /* Reset the OBER bit */
            FMC->CMR &= ~FMC_CMR_OBER;
      
            /* Enable the Option Bytes Programming */
            FMC->CMR |= FMC_CMR_OBPG;
       
            OB->RDP = OB_RDP;

            /* Wait for the FMC ready */
            temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT); 
    
            if(temp_state != FMC_TIMEOUT_ERR)
            {
                /* Reset the OBPG bit */
                FMC->CMR &= ~FMC_CMR_OBPG;
            }
        }
        else
        {
            if(temp_state != FMC_TIMEOUT_ERR)
            {
                /* Reset the OBER Bit */
                FMC->CMR &= ~FMC_CMR_OBER;
            }
        }
    }  
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program the FMC User Option Byte: IWDG_SW / RST_DEEPSLEEP / RST_STDBY.
  * @param  OB_IWDG: Config the WDG mode
  *     @arg OB_IWDG_SW: Software WDG selected
  *     @arg OB_IWDG_HW: Hardware WDG selected
  * @param  OB_DEEPSLEEP: Config Reset event when entering DEEPSLEEP mode.
  *     @arg OB_DEEPSLEEP_NoRST: No reset generated when entering in DEEPSLEEP
  *     @arg OB_DEEPSLEEP_RST: Reset generated when entering in DEEPSLEEP
  * @param  OB_STDBY: Config Reset event when entering Standby mode.
  *     @arg OB_STDBY_NoRST: No reset generated when entering in STANDBY
  *     @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State  FMC_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_DEEPSLEEP, uint8_t OB_STDBY)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {
        /* Set the OBPG bit*/
        FMC->CMR |= FMC_CMR_OBPG; 

        OB->USER = (uint16_t)((uint16_t)(OB_IWDG | OB_DEEPSLEEP) | (uint16_t)(OB_STDBY | 0xF8));
    
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program the BOOT1 option bit.
  * @param  OB_BOOT1: The legal parameter can be one of the following value:
  *     @arg OB_BOOT1_RESET
  *     @arg OB_BOOT1_SET
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_BOOTConfig(uint8_t OB_BOOT1)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {  
        /* Set the OBPG bit*/
        FMC->CMR |= FMC_CMR_OBPG;

        OB->USER = OB_BOOT1 | 0xEF;
  
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program the VDDA Power source bit.
  * @param  OB_VDDA_ANALOG: Selects the analog monitoring on VDDA Power source.
  *   This parameter can be one of the following values:
  *     @arg OB_VDDA_ANALOG_ON: Analog monitoring on VDDA Power source ON
  *     @arg OB_VDDA_ANALOG_OFF: Analog monitoring on VDDA Power source OFF
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {  
        /* Set the OBPG bit*/
        FMC->CMR |= FMC_CMR_OBPG; 

        OB->USER = OB_VDDA_ANALOG | 0xDF;
  
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program the SRAM parity bit.
  * @param  OB_SRAM_Parity: Set or Reset the SRAM parity bit.
  *   This legal parameter can be :
  *     @arg OB_SRAM_PARITY_SET: Set SRAM parity.
  *     @arg OB_SRAM_PARITY_RESET: Reset SRAM parity.
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_SRAMParityConfig(uint8_t OB_SRAM_Parity)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {  
        /* Set the OBPG bit*/
        FMC->CMR |= FMC_CMR_OBPG; 

        OB->USER = OB_SRAM_Parity | 0xBF;
  
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Program the FMC User Option Byte.
  * @param  OB_USER: Select all user option byte
  *   The legal parameter is a combination of the following values:
  *     @arg OB_IWDG_SW / OB_IWDG_HW
  *     @arg OB_DEEPSLEEP_NoRST / OB_DEEPSLEEP_RST
  *     @arg OB_STDBY_NoRST / OB_STDBY_RST
  *     @arg OB_BOOT1_RESET / OB_BOOT1_SET
  *     @arg OB_VDDA_ANALOG_ON / OB_VDDA_ANALOG_OFF 
  *     @arg OB_SRAM_PARITY_SET / OB_SRAM_PARITY_RESET 
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_OB_WriteUser(uint8_t OB_USER)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
  
    if(temp_state == FMC_READY)
    {
        /* Set the OBPG bit */
        FMC->CMR |= FMC_CMR_OBPG; 

        OB->USER = OB_USER | 0x88;
  
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OBPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }    
    /* Return the FMC state */
    return temp_state;

}

/**
  * @brief  Program Option Byte Data.
  * @param  Address: The Option Byte address to be programmed.
  *   The legal parameter can be 0x1FFFF804 or 0x1FFFF806. 
  * @param  Data: The Byte to be programmed.
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_ProgramOptionByteData(uint32_t Address, uint8_t Data)
{
    FMC_State temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);

    if(temp_state == FMC_READY)
    {
        /* SET the OPTPG bit */
        FMC->CMR |= FMC_CMR_OBPG; 
        *(__IO uint16_t*)Address = Data;
    
        /* Wait for the FMC ready */
        temp_state = FMC_WaitReady(FMC_TIMEOUT_COUNT);
    
        if(temp_state != FMC_TIMEOUT_ERR)
        {
            /* Reset the OPTPG bit */
            FMC->CMR &= ~FMC_CMR_OBPG;
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Get the FMC User Option Byte.
  * @param  None
  * @retval The FMC User Option Byte.
  */
uint8_t FMC_OB_GetUser(void)
{
    return (uint8_t)(FMC->OPTR >> 8);
}

/**
  * @brief  Get the FMC Write Protection Option Byte.
  * @param  None
  * @retval The FMC Write Protection Option Byte
  */
uint32_t FMC_OB_GetWRP(void)
{
    return (uint32_t)(FMC->WPR);
}

/**
  * @brief  Check whether the FMC Read out Protection Status is SET or RESET.
  * @param  None
  * @retval FMC ReadOut Protection state
  */
TypeState FMC_OB_GetRDP(void)
{
    TypeState RDPState = RESET;
  
    if ((uint8_t)(FMC->OPTR & (FMC_OPTR_PLEVEL1 | FMC_OPTR_PLEVEL2)) != RESET)
    {
        RDPState = SET;
    }
    else
    {
        RDPState = RESET;
    }
    return RDPState;
}

/**
  * @}
  */

/** @defgroup FMC_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
             ##### Interrupts and flags management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Enable or disable the corresponding FMC interrupt source.
  * @param  FMC_INT: The FMC interrupt source to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg FMC_INT_EOP: FMC end of programming Interrupt
  *     @arg FMC_INT_ERR: FMC Error Interrupt
  * @param  NewValue: ENABLE or DISABLE.
  * @retval None 
  */
void FMC_INTConfig(uint32_t FMC_INT, TypeState NewValue)
{
    if(NewValue != DISABLE)
    {
        /* Enable the interrupt sources */
        FMC->CMR |= FMC_INT;
    }
    else
    {
        /* Disable the interrupt sources */
        FMC->CMR &= ~(uint32_t)FMC_INT;
    }
}

/**
  * @brief  Check whether the FMC flag is SET or RESET.
  * @param  FMC_FLAG: the corresponding FMC flag.
  *   The legal parameter can be:
  *     @arg FMC_FLAG_BSY:  FMC BUSY flag
  *     @arg FMC_FLAG_PERR: FMC Programming error flag flag
  *     @arg FMC_FLAG_WERR: FMC Write protection error flag
  *     @arg FMC_FLAG_EOP:  FMC End of Programming flag
  * @retval The state of the FMC flag.
  */
TypeState FMC_GetBitState(uint32_t FMC_FLAG)
{
    if((FMC->CSR & FMC_FLAG) != (uint32_t)RESET)
    {
        return  SET;
    }
    /* Return the state of corresponding FMC flag */
    return RESET; 
}

/**
  * @brief  Clear the FMC pending flag.
  * @param  FMC_FLAG: clear the corresponding FMC flag.
  *     @arg FMC_FLAG_PERR: Programming error flag flag
  *     @arg FMC_FLAG_WERR: Write protection error flag
  *     @arg FMC_FLAG_EOP:  End of Programming flag
  * @retval None
  */
void FMC_ClearBitState(uint32_t FMC_FLAG)
{
    /* Clear the flags */
    FMC->CSR = FMC_FLAG;
}

/**
  * @brief  Return the FMC state.
  * @param  None
  * @retval FMC state: FMC_READY,  FMC_BSY,  FMC_WRPERR, or FMC_PGERR
  */
FMC_State FMC_GetState(void)
{
    FMC_State temp_state = FMC_READY;
  
    if((FMC->CSR & FMC_CSR_BUSY) == FMC_CSR_BUSY) 
    {
        temp_state = FMC_BSY;
    }
    else 
    {  
        if((FMC->CSR & (uint32_t)FMC_CSR_WPEF)!= (uint32_t)0x00)
        { 
            temp_state = FMC_WRPERR;
        }
        else 
        {
            if((FMC->CSR & (uint32_t)(FMC_CSR_PGEF)) != (uint32_t)0x00)
            {
                temp_state = FMC_PGERR; 
            }
        }
    }
    /* Return the FMC state */
    return temp_state;
}

/**
  * @brief  Check whether FMC is ready or not.
  * @param  Timeout: Count of loop
  * @retval FMC state: FMC_READY, FMC_BSY, FMC_WRPERR, FMC_PGERR or FMC_TIMEOUT_ERR.
  */
FMC_State FMC_WaitReady(uint32_t uCount)
{ 
    FMC_State temp_state = FMC_BSY;
  
    /* Wait for FMC ready */
    do
    {
        /* Get FMC state */
        temp_state = FMC_GetState();
        uCount--;
    }while((temp_state == FMC_BSY) && (uCount != 0x00));
  
    if(temp_state == FMC_BSY)
    {
        temp_state = FMC_TIMEOUT_ERR;
    }
    /* Return the FMC state */
    return temp_state;
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

/************************ (C) COPYRIGHT GIGADEVICE *****END OF FILE****/

/**
  ******************************************************************************
  * @file    system_gd32f1x0.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup GD32F1x0_system
  * @{
  */  
  
/** @addtogroup GD32F1x0_System_Private_Includes
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Private_Variables
  * @{
  */

#define RCC_GCFGR_FREQ_DIVIDE_RESET     (uint32_t)0x083F000C    /* Reset SCS[1:0], AHBPS[3:0], APB1PS[2:0],APB2PS[2:0],
                                                                   ADCPS[1:0],USBPS[1:0],CKOTUSRC[2:0], CKOUTDIV, and PLLDV bits*/
#define RCC_GCCR_CLK_ENABLE_RESET       (uint32_t)0xFEF6FFFF    /* Reset HSEEN, CKMEN and PLLEN bits */
#define RCC_GCCR_HSEBPS_RESET           (uint32_t)0xFFFBFFFF    /* Reset HSEBPS bit */
#define RCC_GCFGR_CLK_SELECT_RESET      (uint32_t)0xF7C0FFFF    /* Reset PLLSEL, PLLPREDV and PLLMF[3:0] bits */
#define RCC_GCFGR2_HSEPREDV1_RESET      (uint32_t)0xFFFFFFF0    /* Reset HSEPREDV1[3:0] bits */
#define RCC_GCFGR3_RESET                (uint32_t)0xFFFFFEBC    /* Reset USARTSEL[1:0], I2CSEL, CECSEL and ADCSEL bits */
#define RCC_GCCR2_HSI14_RESET           (uint32_t)0xFFFFFFFE    /* Reset HSI14 bit */
#define RCC_GCIR_DISABLE                (uint32_t)0x00000000    /* Disable all interrupts */
#define RCC_GCFGR_PLLMF3_0              (uint32_t)0x003C0000    /* PLLMF[3:0] Bits */

//#define SYSCLK_FREQ_HSE    HSE_VALUE 
//#define SYSCLK_FREQ_24MHz  24000000  
//#define SYSCLK_FREQ_36MHz  36000000 
//#define SYSCLK_FREQ_48MHz  48000000 
//#define SYSCLK_FREQ_56MHz  56000000 
#define SYSCLK_FREQ_72MHz  72000000

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Private_Variables
  * @{
  */

/*******************************************************************************
*  System Clock Select
*******************************************************************************/
#ifdef SYSCLK_FREQ_HSE
    uint32_t SystemCoreClock         = SYSCLK_FREQ_HSE;          
#elif defined SYSCLK_FREQ_24MHz
    uint32_t SystemCoreClock         = SYSCLK_FREQ_24MHz;        
#elif defined SYSCLK_FREQ_36MHz
    uint32_t SystemCoreClock         = SYSCLK_FREQ_36MHz;       
#elif defined SYSCLK_FREQ_48MHz
    uint32_t SystemCoreClock         = SYSCLK_FREQ_48MHz;
#elif defined SYSCLK_FREQ_56MHz
    uint32_t SystemCoreClock         = SYSCLK_FREQ_56MHz;
#elif defined SYSCLK_FREQ_72MHz
    uint32_t SystemCoreClock         = SYSCLK_FREQ_72MHz;        
#else
    uint32_t SystemCoreClock         = HSI_VALUE;                      /*!< HSI is selected as system clock source */
#endif

__I uint8_t AHBPrescTableList[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Private_FunctionPrototypes
  * @{
  */

static void Set_SysClock(void);

#ifdef SYSCLK_FREQ_HSE
  static void Set_SysClockToHSE(void);
#elif defined SYSCLK_FREQ_24MHz
  static void Set_SysClockTo24(void);
#elif defined SYSCLK_FREQ_36MHz
  static void Set_SysClockTo36(void);
#elif defined SYSCLK_FREQ_48MHz
  static void Set_SysClockTo48(void);
#elif defined SYSCLK_FREQ_56MHz
  static void Set_SysClockTo56(void);  
#elif defined SYSCLK_FREQ_72MHz
  static void Set_SysClockTo72(void);
#endif

/**
  * @}
  */

/** @addtogroup GD32F1x0_System_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the Embedded Flash Interface, the PLL and set the 
  *         SystemCoreClock variable.
  * @param  None
  * @retval None
  */
void System_Init (void)
{
    /* Set HSIEN bit */
    RCC->GCCR |= RCC_GCCR_HSIEN;
    
    /* Reset SCS[1:0], AHBPS[3:0], APB1PS[2:0],APB2PS[2:0],
       ADCPS[1:0],USBPS[1:0],CKOTUSRC[2:0], CKOUTDIV, and PLLDV bits*/
    RCC->GCFGR &= RCC_GCFGR_FREQ_DIVIDE_RESET;
    
    /* Reset HSEEN, CKMEN and PLLEN bits */
    RCC->GCCR &= RCC_GCCR_CLK_ENABLE_RESET;
    
    /* Reset HSEBPS bit */
    RCC->GCCR &= RCC_GCCR_HSEBPS_RESET;
    
    /* Reset PLLSEL, PLLPREDV and PLLMF[3:0] bits */
    RCC->GCFGR &= RCC_GCFGR_CLK_SELECT_RESET;
    
    /* Reset HSEPREDV1[3:0] bits */
    RCC->GCFGR2 &= RCC_GCFGR2_HSEPREDV1_RESET;
    
    /* Reset USARTSEL[1:0], CECSEL and ADCSEL bits */
    RCC->GCFGR3 &= RCC_GCFGR3_RESET;
    
    /* Reset HSI14 bit */
    RCC->GCCR2 &= RCC_GCCR2_HSI14_RESET;
    
    /* Disable all interrupts */
    RCC->GCIR = RCC_GCIR_DISABLE;
    
    /* Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
    Set_SysClock();
}

/**
  * @brief  Update SystemCoreClock according to RCC Register Values
  * @note   Every time the core clock (HCLK) changes, you must call this function
  *         to update the SystemCoreClock variable values. Otherwise, any configuration
  *         based on this variable will be wrong.
  * @param  None
  * @retval None
  */
void SystemCoreClock_Update (void)
{
    uint32_t temp = 0, pllmf = 0, pllmf4=0,pllselect = 0, prediv1factor = 0, presc = 0;

    /* Get SYSCLK source -------------------------------------------------------*/
    temp = RCC->GCFGR & RCC_GCFGR_SCSS;
    
    switch (temp)
    {
    case 0x00:  /* HSI selected as system clock */
        SystemCoreClock = HSI_VALUE;
        break;
    case 0x04:  /* HSE selected as system clock */
        SystemCoreClock = HSE_VALUE;
        break;
    case 0x08:
        /* PLL selected as system clock */
        /* Get PLL clock source and multiplication factor ----------------------*/
        /* Get PLLMF[3:0] */
        pllmf = RCC->GCFGR & RCC_GCFGR_PLLMF3_0;
        /* Get PLLMF[4] */
        pllmf4 = RCC->GCFGR & RCC_GCFGR_PLLMF_4;
        pllmf4 = (( pllmf4 >> 27)*15);
        
        pllmf = ( pllmf >> 18) + pllmf4+ 2;
        
        pllselect = RCC->GCFGR & RCC_GCFGR_PLLSEL;
        if (pllselect == 0x00)
        {
            /* HSI clock divided by two used as PLL clock entry */
            SystemCoreClock = (HSI_VALUE >> 1) * pllmf;
        }
        else
        {
            /* Get the HSEPREDV division factor */
            prediv1factor = (RCC->GCFGR2 & RCC_GCFGR2_HSEPREDV1) + 1;
            /* HSE clock used as PREDIV1 clock entry */
            SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmf; 
        }
        break;
    default: 
        /* HSI selected as system clock */
        SystemCoreClock = HSI_VALUE;
        break;
    }
    /* Calculate HCLK frequency ----------------*/
    /* Get AHB prescaler */
    presc = AHBPrescTableList[((RCC->GCFGR & RCC_GCFGR_AHBPS) >> 4)];
    /* AHB frequency */
    SystemCoreClock >>= presc;  
}

/**
  * @brief  Configure the System clock frequency, AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   If none of the define below is enabled, the HSI is selected as System clock
  *         source (default after reset)
  * @param  None
  * @retval None
  */
static void Set_SysClock(void)
{
#ifdef SYSCLK_FREQ_HSE
    Set_SysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
    Set_SysClockTo24();
#elif defined SYSCLK_FREQ_36MHz
    Set_SysClockTo36();
#elif defined SYSCLK_FREQ_48MHz
    Set_SysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
    Set_SysClockTo56();  
#elif defined SYSCLK_FREQ_72MHz
    Set_SysClockTo72();
#endif
}

#ifdef SYSCLK_FREQ_HSE
/**
  * @brief  HSE is chosen as the system clock source and configurate AHB/APB2
  *         /APB1 prescalers.
  * @note   This function should be called only after reset.
  * @param  None
  * @retval None
  */
static void Set_SysClockToHSE(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
    
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached  */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }  

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB= SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* HSE is chosen as the system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_HSE;    
        
        /* Wait untill HSE is selected as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)0x04)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
         configuration. To deal with the error,the user can add here some code  */
    }  
}

#elif defined SYSCLK_FREQ_24MHz

/**
  * @brief  Set the System clock frequency to 24MHz and configurate AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   This function should be called only after the RCC clock configuration
  *         is reset to the default reset value .
  * @param  None
  * @retval None
  */
static void Set_SysClockTo24(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
  
    /* SYSCLK, AHB, APB1,APB2 configuration ----------------------------------------*/
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }  

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB = SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* PLL multiplication factor is 3 , PLL = HSE * 3 = 72 MHz */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_PLLSEL | RCC_GCFGR_PLLPREDV | RCC_GCFGR_PLLMF));
        RCC->GCFGR |= (uint32_t)(RCC_GCFGR_PLLSEL_HSEPREDIV | RCC_GCFGR_PLLPREDV_PREDIV1 | RCC_GCFGR_PLLMF3);
        
        /* PLL Enable */
        RCC->GCCR |= RCC_GCCR_PLLEN;
        
        /* Wait untill PLL is ready */
        while((RCC->GCCR & RCC_GCCR_PLLSTB) == 0)
        {
        }
        
        /* PLL is chosen as system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_PLL;    
        
        /* Wait untill PLL is chosen as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
         configuration. To deal with the error,the user can add here some code  */
    }  
}

#elif defined SYSCLK_FREQ_36MHz

/**
  * @brief  Set the System clock frequency to 36MHz and configurate AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   This function should be called only after the RCC clock configuration
  *         is reset to the default reset value .
  * @param  None
  * @retval None
  */
static void Set_SysClockTo36(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
  
    /* SYSCLK, AHB, APB1, APB2 configuration ----------------------------------------*/
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));
    
    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }  

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB = SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* PLL multiplication factor is 9 , PLL = HSE/2 * 9 = 36 MHz */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_PLLSEL | RCC_GCFGR_PLLPREDV | RCC_GCFGR_PLLMF));
        RCC->GCFGR |= (uint32_t)(RCC_GCFGR_PLLSEL_HSEPREDIV | RCC_GCFGR_PLLPREDV_PREDIV1_DIV2 | RCC_GCFGR_PLLMF9);
        
        /* PLL Enable */
        RCC->GCCR |= RCC_GCCR_PLLEN;
        
        /* Wait untill PLL is ready */
        while((RCC->GCCR & RCC_GCCR_PLLSTB) == 0)
        {
        }
        
        /* PLL is chosen as system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_PLL;    
        
        /* Wait untill PLL is chosen as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
        configuration. To deal with the error,the user can add here some code  */
    }  
}

#elif defined SYSCLK_FREQ_48MHz

/**
  * @brief  Set the System clock frequency to 48MHZ and configurate AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   This function should be called only after the RCC clock configuration
  *         is reset to the default reset value .
  * @param  None
  * @retval None
  */
static void Set_SysClockTo48(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
  
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }  

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB= SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* PLL multiplication factor is 6 , PLL = HSE * 6 = 48 MHz */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_PLLSEL | RCC_GCFGR_PLLPREDV | RCC_GCFGR_PLLMF));
        RCC->GCFGR |= (uint32_t)(RCC_GCFGR_PLLSEL_HSEPREDIV | RCC_GCFGR_PLLPREDV_PREDIV1 | RCC_GCFGR_PLLMF6);         
        
        /* PLL Enable */
        RCC->GCCR |= RCC_GCCR_PLLEN;
        
        /* Wait untill PLL is ready */
        while((RCC->GCCR & RCC_GCCR_PLLSTB) == 0)
        {
        }
        
        /* PLL is chosen as system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_PLL;    
        
        /* Wait untill PLL is chosen as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
        configuration. To deal with the error,the user can add here some code  */
    }  
}

#elif defined SYSCLK_FREQ_56MHz

/**
  * @brief  Set the System clock frequency to 56MHz and configurate AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   This function should be called only after the RCC clock configuration
  *         is reset to the default reset value .
  * @param  None
  * @retval None
  */
static void Set_SysClockTo56(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
  
    /* SYSCLK, AHB, APB1,APB2 configuration ----------------------------------------*/
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB = SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* PLL multiplication factor is 7 , PLL = HSE * 7 = 56 MHz */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_PLLSEL | RCC_GCFGR_PLLPREDV | RCC_GCFGR_PLLMF));
        RCC->GCFGR |= (uint32_t)(RCC_GCFGR_PLLSEL_HSEPREDIV | RCC_GCFGR_PLLPREDV_PREDIV1 | RCC_GCFGR_PLLMF7);
        
        /* PLL Enable */
        RCC->GCCR |= RCC_GCCR_PLLEN;
        
        /* Wait untill PLL is ready */
        while((RCC->GCCR & RCC_GCCR_PLLSTB) == 0)
        {
        }
        
        /* PLL is chosen as system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_PLL;    
        
        /* Wait untill PLL is chosen as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
        configuration. To deal with the error,the user can add here some code  */
    }  
}


#elif defined SYSCLK_FREQ_72MHz

/**
  * @brief  Set the System clock frequency to 72MHz and configurate AHB/APB1/APB2 prescalers and Flash
  *         settings.
  * @note   This function should be called only after the RCC clock configuration
  *         is reset to the default reset value .
  * @param  None
  * @retval None
  */
static void Set_SysClockTo72(void)
{
    __IO uint32_t tmp_StartUpCounter = 0, tmp_HSEStatus = 0;
  
    /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/
    /* HSE Enable */
    RCC->GCCR |= ((uint32_t)RCC_GCCR_HSEEN);
 
    /* Wait untill HSE is ready ,but exit if Time out is reached */
    do
    {
        tmp_HSEStatus = RCC->GCCR & RCC_GCCR_HSESTB;
        tmp_StartUpCounter++;  
    } while((tmp_HSEStatus == 0) && (tmp_StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->GCCR & RCC_GCCR_HSESTB) != RESET)
    {
        tmp_HSEStatus = (uint32_t)0x01;
    }
    else
    {
        tmp_HSEStatus = (uint32_t)0x00;
    }  

    if (tmp_HSEStatus == (uint32_t)0x01)
    {
        /* AHB = SYSCLK not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_AHBPS_DIV1;
        
        /* APB2 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB2PS_DIV1;
        
        /* APB1 = AHB not divided */
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_APB1PS_DIV1;
        
        /* PLL multiplication factor is 9 , PLL = HSE * 9 = 72 MHz */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_PLLSEL | RCC_GCFGR_PLLPREDV | RCC_GCFGR_PLLMF));
        RCC->GCFGR |= (uint32_t)(RCC_GCFGR_PLLSEL_HSEPREDIV | RCC_GCFGR_PLLPREDV_PREDIV1 | RCC_GCFGR_PLLMF9);
        
        /* PLL Enable */
        RCC->GCCR |= RCC_GCCR_PLLEN;
        
        /* Wait untill PLL is ready */
        while((RCC->GCCR & RCC_GCCR_PLLSTB) == 0)
        {
        }
        
        /* PLL is chosen as system clock source */
        RCC->GCFGR &= (uint32_t)((uint32_t)~(RCC_GCFGR_SCS));
        RCC->GCFGR |= (uint32_t)RCC_GCFGR_SCS_PLL;    
        
        /* Wait untill PLL is chosen as system clock source */
        while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL)
        {
        }
    }
    else
    { /* If HSE fails to start, the application will have wrong clock 
        configuration. To deal with the error,the user can add here some code  */
    }  
}

#endif

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

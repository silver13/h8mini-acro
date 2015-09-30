/**
  ******************************************************************************
  * @file    gd32f1x0_rcc.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   RCC header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_RCC_H
#define __GD32F1X0_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup RCC
  * @{
  */

/** @defgroup RCC_Exported_Types
  * @{
  */

/** 
  * @brief  RCC Initial Parameters
  */
typedef struct
{
  uint32_t CK_SYS_Frequency;            /*!< The frequency of the CK_SYS.     */
  uint32_t AHB_Frequency;               /*!< The frequency of the AHB.        */
  uint32_t APB1_Frequency;              /*!< The frequency of the APB1.       */
  uint32_t APB2_Frequency;              /*!< The frequency of the APB2.       */
  uint32_t ADCCLK_Frequency;            /*!< The frequency of the ADCCLK.     */
  uint32_t CECCLK_Frequency;            /*!< The frequency of the CECCLK.     */
  uint32_t USART1CLK_Frequency;         /*!< The frequency of the USART1CLK.  */
}RCC_ClocksPara;

/**
  * @}
  */

/** @defgroup RCC_Exported_Constants
  * @{
  */

/** @defgroup RCC_HSE_configuration 
  * @{
  */
#define RCC_HSE_OFF                         ((uint8_t)0x00)
#define RCC_HSE_ON                          ((uint8_t)0x01)
#define RCC_HSE_BYPASS                      ((uint8_t)0x05)

/**
  * @}
  */ 
 
/** @defgroup RCC_PLL_Clock_Source 
  * @{
  */
#define RCC_PLLSOURCE_HSI_DIV2              RCC_GCFGR_PLLSEL_HSI_DIV2
#define RCC_PLLSOURCE_HSEPREDIV             RCC_GCFGR_PLLSEL_HSEPREDIV
 
/**
  * @}
  */ 

/** @defgroup RCC_PLL_Multiplication_Factor 
  * @{
  */
#define RCC_PLLMUL_2                        RCC_GCFGR_PLLMF2
#define RCC_PLLMUL_3                        RCC_GCFGR_PLLMF3
#define RCC_PLLMUL_4                        RCC_GCFGR_PLLMF4
#define RCC_PLLMUL_5                        RCC_GCFGR_PLLMF5
#define RCC_PLLMUL_6                        RCC_GCFGR_PLLMF6
#define RCC_PLLMUL_7                        RCC_GCFGR_PLLMF7
#define RCC_PLLMUL_8                        RCC_GCFGR_PLLMF8
#define RCC_PLLMUL_9                        RCC_GCFGR_PLLMF9
#define RCC_PLLMUL_10                       RCC_GCFGR_PLLMF10
#define RCC_PLLMUL_11                       RCC_GCFGR_PLLMF11
#define RCC_PLLMUL_12                       RCC_GCFGR_PLLMF12
#define RCC_PLLMUL_13                       RCC_GCFGR_PLLMF13
#define RCC_PLLMUL_14                       RCC_GCFGR_PLLMF14
#define RCC_PLLMUL_15                       RCC_GCFGR_PLLMF15
#define RCC_PLLMUL_16                       RCC_GCFGR_PLLMF16
#define RCC_PLLMUL_17                       RCC_GCFGR_PLLMF17
#define RCC_PLLMUL_18                       RCC_GCFGR_PLLMF18
#define RCC_PLLMUL_19                       RCC_GCFGR_PLLMF19
#define RCC_PLLMUL_20                       RCC_GCFGR_PLLMF20
#define RCC_PLLMUL_21                       RCC_GCFGR_PLLMF21
#define RCC_PLLMUL_22                       RCC_GCFGR_PLLMF22
#define RCC_PLLMUL_23                       RCC_GCFGR_PLLMF23
#define RCC_PLLMUL_24                       RCC_GCFGR_PLLMF24
#define RCC_PLLMUL_25                       RCC_GCFGR_PLLMF25
#define RCC_PLLMUL_26                       RCC_GCFGR_PLLMF26
#define RCC_PLLMUL_27                       RCC_GCFGR_PLLMF27
#define RCC_PLLMUL_28                       RCC_GCFGR_PLLMF28
#define RCC_PLLMUL_29                       RCC_GCFGR_PLLMF29
#define RCC_PLLMUL_30                       RCC_GCFGR_PLLMF30
#define RCC_PLLMUL_31                       RCC_GCFGR_PLLMF31
#define RCC_PLLMUL_32                       RCC_GCFGR_PLLMF32

/**
  * @}
  */

/** @defgroup RCC_HSEPREDV1_division_factor
  * @{
  */
#define  RCC_HSEPREDV1_DIV1                RCC_GCFGR2_HSEPREDV1_DIV1
#define  RCC_HSEPREDV1_DIV2                RCC_GCFGR2_HSEPREDV1_DIV2
#define  RCC_HSEPREDV1_DIV3                RCC_GCFGR2_HSEPREDV1_DIV3
#define  RCC_HSEPREDV1_DIV4                RCC_GCFGR2_HSEPREDV1_DIV4
#define  RCC_HSEPREDV1_DIV5                RCC_GCFGR2_HSEPREDV1_DIV5
#define  RCC_HSEPREDV1_DIV6                RCC_GCFGR2_HSEPREDV1_DIV6
#define  RCC_HSEPREDV1_DIV7                RCC_GCFGR2_HSEPREDV1_DIV7
#define  RCC_HSEPREDV1_DIV8                RCC_GCFGR2_HSEPREDV1_DIV8
#define  RCC_HSEPREDV1_DIV9                RCC_GCFGR2_HSEPREDV1_DIV9
#define  RCC_HSEPREDV1_DIV10               RCC_GCFGR2_HSEPREDV1_DIV10
#define  RCC_HSEPREDV1_DIV11               RCC_GCFGR2_HSEPREDV1_DIV11
#define  RCC_HSEPREDV1_DIV12               RCC_GCFGR2_HSEPREDV1_DIV12
#define  RCC_HSEPREDV1_DIV13               RCC_GCFGR2_HSEPREDV1_DIV13
#define  RCC_HSEPREDV1_DIV14               RCC_GCFGR2_HSEPREDV1_DIV14
#define  RCC_HSEPREDV1_DIV15               RCC_GCFGR2_HSEPREDV1_DIV15
#define  RCC_HSEPREDV1_DIV16               RCC_GCFGR2_HSEPREDV1_DIV16

/**
  * @}
  */
 
/** @defgroup RCC_System_Clock_Source 
  * @{
  */
#define RCC_SYSCLKSOURCE_HSI                RCC_GCFGR_SCS_HSI
#define RCC_SYSCLKSOURCE_HSE                RCC_GCFGR_SCS_HSE
#define RCC_SYSCLKSOURCE_PLLCLK             RCC_GCFGR_SCS_PLL

/**
  * @}
  */

/** @defgroup RCC_AHB_Clock_Source
  * @{
  */
#define RCC_SYSCLK_DIV1                     RCC_GCFGR_AHBPS_DIV1
#define RCC_SYSCLK_DIV2                     RCC_GCFGR_AHBPS_DIV2
#define RCC_SYSCLK_DIV4                     RCC_GCFGR_AHBPS_DIV4
#define RCC_SYSCLK_DIV8                     RCC_GCFGR_AHBPS_DIV8
#define RCC_SYSCLK_DIV16                    RCC_GCFGR_AHBPS_DIV16
#define RCC_SYSCLK_DIV64                    RCC_GCFGR_AHBPS_DIV64
#define RCC_SYSCLK_DIV128                   RCC_GCFGR_AHBPS_DIV128
#define RCC_SYSCLK_DIV256                   RCC_GCFGR_AHBPS_DIV256
#define RCC_SYSCLK_DIV512                   RCC_GCFGR_AHBPS_DIV512

/**
  * @}
  */ 

/** @defgroup RCC_APB_Clock_Source
  * @{
  */
#define RCC_APB1AHB_DIV1                    RCC_GCFGR_APB1PS_DIV1
#define RCC_APB1AHB_DIV2                    RCC_GCFGR_APB1PS_DIV2
#define RCC_APB1AHB_DIV4                    RCC_GCFGR_APB1PS_DIV4
#define RCC_APB1AHB_DIV8                    RCC_GCFGR_APB1PS_DIV8
#define RCC_APB1AHB_DIV16                   RCC_GCFGR_APB1PS_DIV116

#define RCC_APB2AHB_DIV1                    RCC_GCFGR_APB2PS_DIV1
#define RCC_APB2AHB_DIV2                    RCC_GCFGR_APB2PS_DIV2
#define RCC_APB2AHB_DIV4                    RCC_GCFGR_APB2PS_DIV4
#define RCC_APB2AHB_DIV8                    RCC_GCFGR_APB2PS_DIV8
#define RCC_APB2AHB_DIV16                   RCC_GCFGR_APB2PS_DIV116

/**
  * @}
  */
  
/** @defgroup RCC_ADC_clock_source 
  * @{
  */
#define RCC_ADCCLK_HSI14                    ((uint32_t)0x00000000)
#define RCC_ADCCLK_APB2_DIV2                ((uint32_t)0x01000000)
#define RCC_ADCCLK_APB2_DIV4                ((uint32_t)0x01004000)
#define RCC_ADCCLK_APB2_DIV6                ((uint32_t)0x01008000)
#define RCC_ADCCLK_APB2_DIV8                ((uint32_t)0x0100C000)

/**
  * @}
  */
  
/** @defgroup RCC_USB_clock_source 
  * @{
  */
#define  RCC_USBCLK_PLL_DIV1                RCC_GCFGR_USBPS_Div1
#define  RCC_USBCLK_PLL_DIV1_5              RCC_GCFGR_USBPS_Div1_5
#define  RCC_USBCLK_PLL_DIV2                RCC_GCFGR_USBPS_Div2
#define  RCC_USBCLK_PLL_DIV2_5              RCC_GCFGR_USBPS_Div2_5

/**
  * @}
  */

/** @defgroup RCC_CEC_clock_source 
  * @{
  */
#define RCC_CECCLK_HSI_DIV244                RCC_GCFGR3_CECSEL_HSI_DIV244
#define RCC_CECCLK_LSE                       RCC_GCFGR3_CECSEL_LSE

/**
  * @}
  */

/** @defgroup RCC_USART_clock_source 
  * @{
  */
#define RCC_USART1CLK_APB2                  RCC_GCFGR3_USART1SEL_APB2
#define RCC_USART1CLK_CK_SYS                RCC_GCFGR3_USART1SEL_CK_SYS
#define RCC_USART1CLK_LSE                   RCC_GCFGR3_USART1SEL_LSE
#define RCC_USART1CLK_HSI                   RCC_GCFGR3_USART1SEL_HSI

/**
  * @}
  */
       
/** @defgroup RCC_Interrupt_Source
  * @{
  */
#define RCC_INT_LSISTB                      ((uint8_t)0x01)
#define RCC_INT_LSESTB                      ((uint8_t)0x02)
#define RCC_INT_HSISTB                      ((uint8_t)0x04)
#define RCC_INT_HSESTB                      ((uint8_t)0x08)
#define RCC_INT_PLLSTB                      ((uint8_t)0x10)
#define RCC_INT_HSI14STB                    ((uint8_t)0x20)
#define RCC_INT_CKM                         ((uint8_t)0x80)

/**
  * @}
  */
  
/** @defgroup RCC_LSE_Configuration
  * @{
  */
#define RCC_LSE_OFF                         ((uint32_t)0x00000000)
#define RCC_LSE_EN                          RCC_BDCR_LSEEN
#define RCC_LSE_BYPASS                      ((uint32_t)(RCC_BDCR_LSEEN | RCC_BDCR_LSEBPS))

/**
  * @}
  */

/** @defgroup RCC_RTC_Clock_Source
  * @{
  */
#define RCC_RTCCLKSource_LSE                RCC_BDCR_RTCSEL_LSE
#define RCC_RTCCLKSource_LSI                RCC_BDCR_RTCSEL_LSI
#define RCC_RTCCLKSource_HSE_DIV32          RCC_BDCR_RTCSEL_HSE

/**
  * @}
  */

/** @defgroup RCC_LSE_Drive_Configuration 
  * @{
  */
#define RCC_LSEDRIVE_LOW                    ((uint32_t)0x00000000)
#define RCC_LSEDRIVE_MEDIUMLOE              RCC_BDCR_LSEDRI_0
#define RCC_LSEDRIVE_MEDIUMHIGH             RCC_BDCR_LSEDRI_1
#define RCC_LSEDRIVE_HIGH                   RCC_BDCR_LSEDRI

/**
  * @}
  */
  
/** @defgroup RCC_AHB_Peripherals 
  * @{
  */
#define RCC_AHBPERIPH_GPIOA                 RCC_AHBCCR_PAEN
#define RCC_AHBPERIPH_GPIOB                 RCC_AHBCCR_PBEN
#define RCC_AHBPERIPH_GPIOC                 RCC_AHBCCR_PCEN
#define RCC_AHBPERIPH_GPIOD                 RCC_AHBCCR_PDEN
#define RCC_AHBPERIPH_GPIOF                 RCC_AHBCCR_PFEN
#define RCC_AHBPERIPH_TSI                   RCC_AHBCCR_TSIEN
#define RCC_AHBPERIPH_CRC                   RCC_AHBCCR_CRCEN
#define RCC_AHBPERIPH_FMC                   RCC_AHBCCR_FMCEN
#define RCC_AHBPERIPH_SRAM                  RCC_AHBCCR_SRAMEN
#define RCC_AHBPERIPH_DMA1                  RCC_AHBCCR_DMA1EN

/**
  * @}
  */
  
/** @defgroup RCC_AHB_Peripherals_RST 
  * @{
  */
#define  RCC_AHBPERIPH_GPIOARST             RCC_AHBRCR_PARST
#define  RCC_AHBPERIPH_GPIOBRST             RCC_AHBRCR_PBRST                
#define  RCC_AHBPERIPH_GPIOCRST             RCC_AHBRCR_PCRST             
#define  RCC_AHBPERIPH_GPIODRST             RCC_AHBRCR_PDRST        
#define  RCC_AHBPERIPH_GPIOFRST             RCC_AHBRCR_PFRST            
#define  RCC_AHBPERIPH_TSIRST               RCC_AHBRCR_TSIRST     

/**
  * @}
  */

/** @defgroup RCC_APB2_Peripherals 
  * @{
  */
#define RCC_APB2PERIPH_CFG                  RCC_APB2CCR_CFGEN
#define RCC_APB2PERIPH_ADC1                 RCC_APB2CCR_ADC1EN
#define RCC_APB2PERIPH_TIMER1               RCC_APB2CCR_TIMER1EN
#define RCC_APB2PERIPH_SPI1                 RCC_APB2CCR_SPI1EN
#define RCC_APB2PERIPH_USART1               RCC_APB2CCR_USART1EN
#define RCC_APB2PERIPH_TIMER15              RCC_APB2CCR_TIMER15EN
#define RCC_APB2PERIPH_TIMER16              RCC_APB2CCR_TIMER16EN
#define RCC_APB2PERIPH_TIMER17              RCC_APB2CCR_TIMER17EN

/**
  * @}
  */

/** @defgroup RCC_APB2_Peripherals_RST 
  * @{
  */
#define  RCC_APB2PERIPH_CFGRST              RCC_APB2RCR_CFGRST
#define  RCC_APB2PERIPH_ADC1RST             RCC_APB2RCR_ADC1RST
#define  RCC_APB2PERIPH_TIMER1RST           RCC_APB2RCR_TIMER1RST
#define  RCC_APB2PERIPH_SPI1RST             RCC_APB2RCR_SPI1RST
#define  RCC_APB2PERIPH_USART1RST           RCC_APB2RCR_USART1RST
#define  RCC_APB2PERIPH_TIMER15RST          RCC_APB2RCR_TIMER15RST
#define  RCC_APB2PERIPH_TIMER16RST          RCC_APB2RCR_TIMER16RST
#define  RCC_APB2PERIPH_TIMER17RST          RCC_APB2RCR_TIMER17RST

/**
  * @}
  */ 

/** @defgroup RCC_APB1_Peripherals 
  * @{
  */
#define RCC_APB1PERIPH_TIMER2               RCC_APB1CCR_TIMER2EN
#define RCC_APB1PERIPH_TIMER3               RCC_APB1CCR_TIMER3EN
#define RCC_APB1PERIPH_TIMER6               RCC_APB1CCR_TIMER6EN
#define RCC_APB1PERIPH_TIMER14              RCC_APB1CCR_TIMER14EN
#define RCC_APB1PERIPH_WWDG                 RCC_APB1CCR_WWDGEN
#define RCC_APB1PERIPH_SPI2                 RCC_APB1CCR_SPI2EN
#define RCC_APB1PERIPH_SPI3                 RCC_APB1CCR_SPI3EN
#define RCC_APB1PERIPH_USART2               RCC_APB1CCR_USART2EN
#define RCC_APB1PERIPH_I2C1                 RCC_APB1CCR_I2C1EN
#define RCC_APB1PERIPH_I2C2                 RCC_APB1CCR_I2C2EN
#define RCC_APB1PERIPH_USB                  RCC_APB1CCR_USBEN
#define RCC_APB1PERIPH_PWR                  RCC_APB1CCR_PWREN
#define RCC_APB1PERIPH_DAC                  RCC_APB1CCR_DACEN
#define RCC_APB1PERIPH_CEC                  RCC_APB1CCR_CECEN

/**
  * @}
  */ 

/** @defgroup RCC_APB1_Peripherals_RST 
  * @{
  */
#define  RCC_APB1PERIPH_TIMER2RST           RCC_APB1RCR_TIMER2RST               
#define  RCC_APB1PERIPH_TIMER3RST           RCC_APB1RCR_TIMER3RST             
#define  RCC_APB1PERIPH_TIMER6RST           RCC_APB1RCR_TIMER6RST               
#define  RCC_APB1PERIPH_TIMER14RST          RCC_APB1RCR_TIMER14RST              
#define  RCC_APB1PERIPH_WWDGRST             RCC_APB1RCR_WWDGRST           
#define  RCC_APB1PERIPH_SPI2RST             RCC_APB1RCR_SPI2RST           
#define  RCC_APB1PERIPH_SPI3RST             RCC_APB1RCR_SPI3RST           
#define  RCC_APB1PERIPH_USART2RST           RCC_APB1RCR_USART2RST          
#define  RCC_APB1PERIPH_I2C1RST             RCC_APB1RCR_I2C1RST              
#define  RCC_APB1PERIPH_I2C2RST             RCC_APB1RCR_I2C2RST                
#define  RCC_APB1PERIPH_USBRST              RCC_APB1RCR_USBRST                 
#define  RCC_APB1PERIPH_PWRRST              RCC_APB1RCR_PWRRST                
#define  RCC_APB1PERIPH_DACRST              RCC_APB1RCR_DACRST                
#define  RCC_APB1PERIPH_CECRST              RCC_APB1RCR_CECRST                 

/**
  * @}
  */ 

/** @defgroup RCC_ACCRPeripherals_I2C3 
  * @{
  */
#define RCC_ACCRPERIPH_I2C3                 RCC_ACCR_I2C3EN

/**
  * @}
  */ 

/** @defgroup RCC_ARCR_Peripherals_I2C3_RST 
  * @{
  */
#define RCC_ARCRPERIPH_I2C3RST              RCC_ARCR_I2C3RST 

/**
  * @}
  */

/** @defgroup RCC_CK_OUT_Clock_Source
  * @{
  */
#define  RCC_CKOUTSRC_NOCLOCK               RCC_GCFGR_CKOUTSEL_NoClock
#define  RCC_CKOUTSRC_HSI14                 RCC_GCFGR_CKOUTSEL_HSI14
#define  RCC_CKOUTSRC_LSI                   RCC_GCFGR_CKOUTSEL_LSI
#define  RCC_CKOUTSRC_LSE                   RCC_GCFGR_CKOUTSEL_LSE
#define  RCC_CKOUTSRC_SYSCLK                RCC_GCFGR_CKOUTSEL_SYSCLK
#define  RCC_CKOUTSRC_HSI                   RCC_GCFGR_CKOUTSEL_HSI
#define  RCC_CKOUTSRC_HSE                   RCC_GCFGR_CKOUTSEL_HSE
#define  RCC_CKOUTSRC_PLLCLK_DIV2           RCC_GCFGR_CKOUTSEL_PLL_DIV2
#define  RCC_CKOUTSRC_PLLCLK_DIV1           RCC_GCFGR_CKOUTSEL_PLL_DIV1

#define  RCC_CKOUTDIV_1                     RCC_GCFGR_CKOUTDIV_1
#define  RCC_CKOUTDIV_2                     RCC_GCFGR_CKOUTDIV_2
#define  RCC_CKOUTDIV_4                     RCC_GCFGR_CKOUTDIV_4
#define  RCC_CKOUTDIV_8                     RCC_GCFGR_CKOUTDIV_8
#define  RCC_CKOUTDIV_16                    RCC_GCFGR_CKOUTDIV_16
#define  RCC_CKOUTDIV_32                    RCC_GCFGR_CKOUTDIV_32
#define  RCC_CKOUTDIV_64                    RCC_GCFGR_CKOUTDIV_64
#define  RCC_CKOUTDIV_128                   RCC_GCFGR_CKOUTDIV_128

/**
  * @}
  */

/** @defgroup RCC_Flag 
  * @{
  */
/* The flag to check is in GCCR register */
#define  RCC_FLAG_HSISTB                    ((uint8_t)0x01)
#define  RCC_FLAG_HSESTB                    ((uint8_t)0x11)
#define  RCC_FLAG_PLLSTB                    ((uint8_t)0x19)

/* The flag to check is in BDCR register */
#define  RCC_FLAG_LSESTB                    ((uint8_t)0x21)

 /* The flag to check is in GCSR register */
#define  RCC_FLAG_LSISTB                    ((uint8_t)0x41)

#define  RCC_FLAG_V12RST                    ((uint8_t)0x57)
#define  RCC_FLAG_OBLRST                    ((uint8_t)0x59)
#define  RCC_FLAG_EPRST                     ((uint8_t)0x5A)
#define  RCC_FLAG_POPDRST                   ((uint8_t)0x5B)
#define  RCC_FLAG_SWRST                     ((uint8_t)0x5C)
#define  RCC_FLAG_IWDGRST                   ((uint8_t)0x5D)
#define  RCC_FLAG_WWDGRST                   ((uint8_t)0x5E)
#define  RCC_FLAG_LPRST                     ((uint8_t)0x5F)

/* The flag to check is in GCCR2 register */
#define  RCC_FLAG_HSI14STB                  ((uint8_t)0x61)

/**
  * @}
  */

/** @defgroup RCC_DEEPSLEEP_VC_VOL
  * @{
  */
#define  RCC_KERNEL_VOL1_2                  RCC_DEEPSLEEP_VC1_2
#define  RCC_KERNEL_VOL1_1                  RCC_DEEPSLEEP_VC1_1
#define  RCC_KERNEL_VOL1_0                  RCC_DEEPSLEEP_VC1_0
#define  RCC_KERNEL_VOL0_9                  RCC_DEEPSLEEP_VC0_9

/**
  * @}
  */

/** @defgroup RCC_PDR_S
  * @{
  */
#define  RCC_POWERDOWNVOL2_6                RCC_PDR_S2_6
#define  RCC_POWERDOWNVOL1_8                RCC_PDR_S1_8

/**
  * @}
  */
  
/**
  * @}
  */
  
/** @defgroup RCC_Exported_Functions
  * @{
  */
/* Reset the RCC clock configuration to the default reset state */
void RCC_DeInit(void);

/* Internal/external clocks, PLL, CKM and CK_OUT configuration functions */
void RCC_HSEConfig(uint8_t RCC_HSE);
TypeState RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSI_Enable(TypeState NewValue);
void RCC_AdjustHSI14CalibrationValue(uint8_t HSI14CalibrationValue);
void RCC_HSI14_Enable(TypeState NewValue);
void RCC_LSEConfig(uint32_t RCC_LSE);
void RCC_LSEDriveConfig(uint32_t RCC_LSEDrive);
void RCC_LSI_Enable(TypeState NewValue);
void RCC_PLLConfig(uint32_t RCC_PLLSelect, uint32_t RCC_PLLMF);
void RCC_PLL_Enable(TypeState NewValue);
void RCC_HSEPREDVConfig(uint32_t RCC_HSEPREDV1_Div);
void RCC_HSEClockMonitor_Enable(TypeState NewValue);
void RCC_CKOUTSRCConfig(uint32_t RCC_CKOUTSRC, uint32_t RCC_CKOUTDIV);

/* System, AHB, APB1 and APB2 busses clocks configuration functions */
void RCC_CK_SYSConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetCK_SYSSource(void);
void RCC_AHBConfig(uint32_t RCC_CK_SYSDiv);
void RCC_APB1Config(uint32_t RCC_APB1);
void RCC_APB2Config(uint32_t RCC_APB2);
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK);
void RCC_USBCLKConfig(uint32_t RCC_USBCLK);
void RCC_CECCLKConfig(uint32_t RCC_CECCLK);
void RCC_USARTCLKConfig(uint32_t RCC_USARTCLK);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_GetClocksFreq(RCC_ClocksPara* RCC_Clocks);

/* Peripheral clocks configuration functions */

void RCC_AHBPeriphClock_Enable(uint32_t RCC_AHBPeriph, TypeState NewValue);
void RCC_APB2PeriphClock_Enable(uint32_t RCC_APB2Periph, TypeState NewValue);
void RCC_APB1PeriphClock_Enable(uint32_t RCC_APB1Periph, TypeState NewValue);
void RCC_ACCRPeriphClock_Enable(uint32_t RCC_ACCRPeriph, TypeState NewValue);
void RCC_RTCCLK_Enable(TypeState NewValue);

void RCC_AHBPeriphReset_Enable(uint32_t RCC_AHBPeriphRST, TypeState NewValue);
void RCC_APB2PeriphReset_Enable(uint32_t RCC_APB2PeriphRST, TypeState NewValue);
void RCC_APB1PeriphReset_Enable(uint32_t RCC_APB1PeriphRST, TypeState NewValue);
void RCC_ARCRPeriphReset_Enable(uint32_t RCC_ARCRPeriphRST, TypeState NewValue);
void RCC_BackupReset_Enable(TypeState NewValue);

/* Interrupts and flags management functions */
void RCC_INTConfig(uint8_t RCC_INT, TypeState NewValue);
TypeState RCC_GetBitState(uint8_t RCC_FLAG);
void RCC_ClearBitState(void);
TypeState RCC_GetIntBitState(uint8_t RCC_INT);
void RCC_ClearIntBitState(uint8_t RCC_INT);

void RCC_UnlockPower(void);
void RCC_KERNELVOLConfig(uint32_t RCC_KERNEL_VOL);
void RCC_POWERDOWNVOLConfig(uint32_t RCC_POWERDOWNVOL);
#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_RCC_H */

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

/**
  ******************************************************************************
  * @file    gd32f1x0_gpio.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   GPIO functions of the firmware library.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_gpio.h"
#include "gd32f1x0_rcc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup GPIO 
  * @brief GPIO driver modules
  * @{
  */

/** @defgroup GPIO_Private_Functions 
  * @{
  */

/**
  * @brief  Reset the GPIOx peripheral.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
    if(GPIOx == GPIOA)
    {
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOARST, ENABLE);
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOARST, DISABLE);
    }
    else if(GPIOx == GPIOB)
    {
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOBRST, ENABLE);
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOBRST, DISABLE);
    }
    else if(GPIOx == GPIOC)
    {
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOCRST, ENABLE);
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOCRST, DISABLE);
    }
    else if(GPIOx == GPIOD)
    {
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIODRST, ENABLE);
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIODRST, DISABLE);
    }
    else if(GPIOx == GPIOF)
    {
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOFRST, ENABLE);
        RCC_AHBPeriphReset_Enable(RCC_AHBPERIPH_GPIOFRST, DISABLE);
    }
}

/**
  * @brief  Initialize the GPIOx peripheral.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_InitParaStruct: The structuer contains configuration information.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitPara* GPIO_InitParaStruct)
{
    uint32_t pin = 0x00;

    for (pin = 0x00; pin < 0x10; pin++)
    {
        if(((GPIO_InitParaStruct->GPIO_Pin) & (((uint32_t)0x01) << pin))!=0)
        {
            if ((GPIO_InitParaStruct->GPIO_Mode == GPIO_MODE_OUT) || (GPIO_InitParaStruct->GPIO_Mode == GPIO_MODE_AF))
            {
                /* Speed configuration */
                GPIOx->OSPD &= ~(GPIO_OSPD_OSPD0 << (pin * 2));
                GPIOx->OSPD |= ((uint32_t)(GPIO_InitParaStruct->GPIO_Speed) << (pin * 2));

                /* Output type configuration */
                GPIOx->OMODE &= ~((GPIO_OMODE_OM0) << ((uint16_t)pin));
                GPIOx->OMODE |= (uint16_t)(((uint16_t)GPIO_InitParaStruct->GPIO_OType) << ((uint16_t)pin));
            }

            /* Pull-up Pull-down configuration */
            GPIOx->PUPD &= ~(GPIO_PUPD_PUPD0 << ((uint16_t)pin * 2));
            GPIOx->PUPD |= (((uint32_t)GPIO_InitParaStruct->GPIO_PuPd) << (pin * 2));
            
            /* GPIO mode configuration */
            GPIOx->CTLR  &= ~(GPIO_CTLR_CTLR0 << (pin * 2));
            GPIOx->CTLR |= (((uint32_t)GPIO_InitParaStruct->GPIO_Mode) << (pin * 2));
        }
    }
}

/**
  * @brief  Initial GPIO_InitParameter members.
  * @param  GPIO_InitParaStruct : pointer to a GPIO_InitPara structure.
  * @retval None
  */
void GPIO_ParaInit(GPIO_InitPara* GPIO_InitParaStruct)
{
    /* Reset GPIO init structure parameters values */
    GPIO_InitParaStruct->GPIO_Pin   = GPIO_PIN_ALL;
    GPIO_InitParaStruct->GPIO_Mode  = GPIO_MODE_IN;
    GPIO_InitParaStruct->GPIO_Speed = GPIO_SPEED_2MHZ;
    GPIO_InitParaStruct->GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitParaStruct->GPIO_PuPd  = GPIO_PUPD_NOPULL;
}

/**
  * @brief  Lock GPIO Pins configuration.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_PinLock(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    __IO uint32_t temp_lock = 0x00010000;

    temp_lock |= GPIO_Pin;
    /* Lock key writing sequence*/
    GPIOx->LOCKR = temp_lock;
    GPIOx->LOCKR =  GPIO_Pin;
    GPIOx->LOCKR = temp_lock;
    temp_lock = GPIOx->LOCKR;
    temp_lock = GPIOx->LOCKR;
}

/**
  * @brief  Read the select input port.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @retval The input port pin value.
  */
uint8_t GPIO_ReadInputBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if ((GPIOx->DIR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        return (uint8_t)Bit_SET;
    }
    else
    {
        return (uint8_t)Bit_RESET;
    }
}

/**
  * @brief  Read the select input data.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @retval The input data value.
  */
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
    return ((uint16_t)GPIOx->DIR);
}

/**
  * @brief  Read the specified output data port bit.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @retval The output port pin value.
  */
uint8_t GPIO_ReadOutputBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if ((GPIOx->DOR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        return (uint8_t)Bit_SET;
    }
    else
    {
        return (uint8_t)Bit_RESET;
    }
}

/**
  * @brief  Read the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
    return ((uint16_t)GPIOx->DOR);
}

/**
  * @brief  Set the selected data port bits.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BOR = GPIO_Pin;
}

/**
  * @brief  Clear the selected data port bits.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BCR = GPIO_Pin;
}

/**
  * @brief  Set or clear the selected data port bit.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_Pin: where pin can be (GPIO_PIN_0..GPIO_PIN_15) to select the GPIO peripheral.
  * @param  BitVal: specifies the state of the port.Select one of the follwing values :
  *     @arg Bit_RESET: clear the port pin
  *     @arg Bit_SET: set the port pin
  * @retval None
  */
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitState BitVal)
{
    if (BitVal != Bit_RESET)
    {
        GPIOx->BOR = GPIO_Pin;
    }
    else
    {
        GPIOx->BCR = GPIO_Pin;
    }
}

/**
  * @brief  Write data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
    GPIOx->DOR = PortVal;
}

/**
  * @brief  Write data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral.
  * @param  GPIO_PinSource: This parameter can be GPIO_PINSOURCEx where x can be (0..15).
  * @param  GPIO_AF: selects the pin to used as Alternate function. This parameter can be GPIO_AF_x where x can be 0 to 7.\
             This parameter can be one of the following value:
  *            @arg GPIO_AF_0: EVENTOUT, TIMER15, SPI1, I2S1, TIMER17,MCO, SWDAT, SWCLK, TIMER14,
  *                            USART1, CEC, IR_OUT, SPI2 
  *            @arg GPIO_AF_1: USART1, USART2, CEC, TIMER3, IR_OUT, EVENTOUT, I2C1, I2C2, TIMER15 
  *            @arg GPIO_AF_2: TIMER2, TIMER1, EVENTOUT, TIMER16, TIMER17
  *            @arg GPIO_AF_3: TSI, I2C1, TIMER15, EVENTOUT 
  *            @arg GPIO_AF_4: TIMER14, I2C2, I2C1, USART2
  *            @arg GPIO_AF_5: TIMER16, TIMER17
  *            @arg GPIO_AF_6: EVENTOUT, SPI2 
  *            @arg GPIO_AF_7: COMP1_OUT, COMP2_OUT 
  * @retval None
  */ 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
    uint32_t temp = 0x00;
    uint32_t temp_2 = 0x00;

    temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
    GPIOx->AFS[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
    temp_2 = GPIOx->AFS[GPIO_PinSource >> 0x03] | temp;
    GPIOx->AFS[GPIO_PinSource >> 0x03] = temp_2;
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

;/**
;  ******************************************************************************
;  * @file    startup_gd32f1x0.s
;  * @author  MCU SD
;  * @version V1.0.1   
;  * @date    6-Sep-2014
;  * @brief   GD32F1x0 startup code.
;  ******************************************************************************
;  */

;/* <<< Use Configuration Wizard in Context Menu >>>                                                        */

; Amount of memory (in bytes) allocated for Stack and Heap
; Tailor those values to your application needs
;// <h> Stack Configuration
;//   <o> Stack Size (in Bytes) <0-8192:8>
;// </h>
Stack_Size          EQU     0x400

                    AREA    STACK, NOINIT, READWRITE, ALIGN = 3
Stack_Mem           SPACE   Stack_Size
__initial_sp

;// <h> Heap Configuration
;//   <o>  Heap Size (in Bytes) <0-8192:8>
;// </h>
Heap_Size           EQU     0x400

                    AREA    HEAP, NOINIT, READWRITE, ALIGN = 3
__heap_base
Heap_Mem            SPACE   Heap_Size
__heap_limit


                    PRESERVE8
                    THUMB

; Vector table entries with the exceptions ISR address
                    AREA    RESET, DATA, READONLY
                    EXPORT  __Vectors
                    EXPORT  __Vectors_End
                    EXPORT  __Vectors_Size

__Vectors           DCD     __initial_sp               ; Top of Stack
                    DCD     Reset_Handler              ; Reset Handler
                    DCD     NMI_Handler                ; NMI Handler
                    DCD     HardFault_Handler          ; Hard Fault Handler
                    DCD     MemManage_Handler          ; MPU Fault Handler
                    DCD     BusFault_Handler           ; Bus Fault Handler
                    DCD     UsageFault_Handler         ; Usage Fault Handler
                    DCD     0                          ; Reserved
                    DCD     0                          ; Reserved
                    DCD     0                          ; Reserved
                    DCD     0                          ; Reserved
                    DCD     SVC_Handler                ; SVCall Handler
                    DCD     DebugMon_Handler           ; Debug Monitor Handler
                    DCD     0                          ; Reserved
                    DCD     PendSV_Handler             ; PendSV Handler
                    DCD     SysTick_Handler            ; SysTick Handler

                    ; External Interrupts
                    DCD     WWDG_IRQHandler                ; Window Watchdog
                    DCD     LVD_IRQHandler                 ; LVD through EXTI Line detect
                    DCD     RTC_IRQHandler                 ; RTC through EXTI Line
                    DCD     FMC_IRQHandler                 ; FMC
                    DCD     RCC_IRQHandler                 ; RCC
                    DCD     EXTI0_1_IRQHandler             ; EXTI Line 0 and 1
                    DCD     EXTI2_3_IRQHandler             ; EXTI Line 2 and 3
                    DCD     EXTI4_15_IRQHandler            ; EXTI Line 4 to 15
                    DCD     TS_IRQHandler                  ; TS
                    DCD     DMA1_Channel1_IRQHandler       ; DMA1 Channel 1
                    DCD     DMA1_Channel2_3_IRQHandler     ; DMA1 Channel 2 and Channel 3
                    DCD     DMA1_Channel4_5_IRQHandler     ; DMA1 Channel 4 and Channel 5
                    DCD     ADC1_CMP_IRQHandler            ; ADC1, CMP1 and CMP2 
                    DCD     TIM1_BRK_UP_TRG_COM_IRQHandler ; TIM1 Break, Update, Trigger and Commutation
                    DCD     TIM1_CC_IRQHandler             ; TIM1 Capture Compare
                    DCD     TIM2_IRQHandler                ; TIM2
                    DCD     TIM3_IRQHandler                ; TIM3
                    DCD     TIM6_DAC_IRQHandler            ; TIM6 and DAC
                    DCD     0                              ; Reserved
                    DCD     TIM14_IRQHandler               ; TIM14
                    DCD     TIM15_IRQHandler               ; TIM15
                    DCD     TIM16_IRQHandler               ; TIM16
                    DCD     TIM17_IRQHandler               ; TIM17
                    DCD     I2C1_EV_IRQHandler             ; I2C1 Event
                    DCD     I2C2_EV_IRQHandler             ; I2C2 Event
                    DCD     SPI1_IRQHandler                ; SPI1
                    DCD     SPI2_IRQHandler                ; SPI2
                    DCD     USART1_IRQHandler              ; USART1
                    DCD     USART2_IRQHandler              ; USART2
                    DCD     0                              ; Reserved
                    DCD     CEC_IRQHandler                 ; CEC
                    DCD     0                              ; Reserved
                    DCD     I2C1_ER_IRQHandler             ; I2C1 Error
                    DCD     0                              ; Reserved
                    DCD     I2C2_ER_IRQHandler             ; I2C2 Error
                    DCD     I2C3_EV_IRQHandler             ; I2C3 Event
                    DCD     I2C3_ER_IRQHandler             ; I2C3 Error
                    DCD     USB_LP_IRQHandler              ; USB Low  Priority 
                    DCD     USB_HP_IRQHandler              ; USB High Priority
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     USBWakeUp_IRQHandler           ; USB Wakeup from suspend
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     DMA1_Channel6_7_IRQHandler     ; DMA1 Channel 6 and Channel 7
                    DCD     0                              ; Reserved
                    DCD     0                              ; Reserved
                    DCD     SPI3_IRQHandler                ; SPI3

                    SPACE   0x4A



__Vectors_End

__Vectors_Size      EQU  __Vectors_End - __Vectors

                    AREA    |.text|, CODE, READONLY

; Reset handler routine
Reset_Handler       PROC
                    EXPORT  Reset_Handler                   [WEAK]
                    IMPORT  __main
                    IMPORT  System_Init  
                    LDR     R0, =System_Init
                    BLX     R0
                    LDR     R0, =__main
                    BX      R0
                    ENDP

; Dummy Exception Handlers
NMI_Handler         PROC
                    EXPORT  NMI_Handler                     [WEAK]
                    B       .
                    ENDP

HardFault_Handler   PROC
                    EXPORT  HardFault_Handler               [WEAK]
                    B       .
                    ENDP

MemManage_Handler   PROC
                    EXPORT  MemManage_Handler               [WEAK]
                    B       .
                    ENDP

BusFault_Handler    PROC
                    EXPORT  BusFault_Handler                [WEAK]
                    B       .
                    ENDP

UsageFault_Handler  PROC
                    EXPORT  UsageFault_Handler              [WEAK]
                    B       .
                    ENDP

SVC_Handler         PROC
                    EXPORT  SVC_Handler                     [WEAK]
                    B       .
                    ENDP

DebugMon_Handler    PROC
                    EXPORT  DebugMon_Handler                [WEAK]
                    B       .
                    ENDP

PendSV_Handler      PROC
                    EXPORT  PendSV_Handler                  [WEAK]
                    B       .
                    ENDP

SysTick_Handler     PROC
                    EXPORT  SysTick_Handler                 [WEAK]
                    B       .
                    ENDP

Default_Handler     PROC
                    EXPORT  WWDG_IRQHandler                 [WEAK]
                    EXPORT  LVD_IRQHandler                  [WEAK]
                    EXPORT  RTC_IRQHandler                  [WEAK]
                    EXPORT  FMC_IRQHandler                  [WEAK]
                    EXPORT  RCC_IRQHandler                  [WEAK]
                    EXPORT  EXTI0_1_IRQHandler              [WEAK]
                    EXPORT  EXTI2_3_IRQHandler              [WEAK]
                    EXPORT  EXTI4_15_IRQHandler             [WEAK]
                    EXPORT  TS_IRQHandler                   [WEAK]
                    EXPORT  DMA1_Channel1_IRQHandler        [WEAK]
                    EXPORT  DMA1_Channel2_3_IRQHandler      [WEAK]
                    EXPORT  DMA1_Channel4_5_IRQHandler      [WEAK]
                    EXPORT  DMA1_Channel6_7_IRQHandler      [WEAK]
                    EXPORT  ADC1_CMP_IRQHandler             [WEAK]
                    EXPORT  TIM1_BRK_UP_TRG_COM_IRQHandler  [WEAK]
                    EXPORT  TIM1_CC_IRQHandler              [WEAK]
                    EXPORT  TIM2_IRQHandler                 [WEAK]
                    EXPORT  TIM3_IRQHandler                 [WEAK]
                    EXPORT  TIM6_DAC_IRQHandler             [WEAK]
                    EXPORT  TIM14_IRQHandler                [WEAK]
                    EXPORT  TIM15_IRQHandler                [WEAK]
                    EXPORT  TIM16_IRQHandler                [WEAK]
                    EXPORT  TIM17_IRQHandler                [WEAK]
                    EXPORT  I2C1_EV_IRQHandler              [WEAK]
                    EXPORT  I2C2_EV_IRQHandler              [WEAK]
                    EXPORT  SPI1_IRQHandler                 [WEAK]
                    EXPORT  SPI2_IRQHandler                 [WEAK]
                    EXPORT  USART1_IRQHandler               [WEAK]
                    EXPORT  USART2_IRQHandler               [WEAK]
                    EXPORT  CEC_IRQHandler                  [WEAK]
                    EXPORT  I2C1_ER_IRQHandler              [WEAK]
                    EXPORT  I2C2_ER_IRQHandler              [WEAK]
                    EXPORT  I2C3_EV_IRQHandler              [WEAK]
                    EXPORT  I2C3_ER_IRQHandler              [WEAK]
                    EXPORT  SPI3_IRQHandler                 [WEAK]
                    EXPORT  USBWakeUp_IRQHandler            [WEAK]
                    EXPORT  USB_HP_IRQHandler               [WEAK]
                    EXPORT  USB_LP_IRQHandler               [WEAK]

WWDG_IRQHandler
LVD_IRQHandler
RTC_IRQHandler
FMC_IRQHandler
RCC_IRQHandler
EXTI0_1_IRQHandler
EXTI2_3_IRQHandler
EXTI4_15_IRQHandler
TS_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_3_IRQHandler
DMA1_Channel4_5_IRQHandler
DMA1_Channel6_7_IRQHandler
ADC1_CMP_IRQHandler 
TIM1_BRK_UP_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM6_DAC_IRQHandler
TIM14_IRQHandler
TIM15_IRQHandler
TIM16_IRQHandler
TIM17_IRQHandler
I2C1_EV_IRQHandler
I2C2_EV_IRQHandler

SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
CEC_IRQHandler   

I2C1_ER_IRQHandler
I2C2_ER_IRQHandler
I2C3_EV_IRQHandler
I2C3_ER_IRQHandler
SPI3_IRQHandler
USBWakeUp_IRQHandler
USB_HP_IRQHandler
USB_LP_IRQHandler
                B       .
                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

;/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

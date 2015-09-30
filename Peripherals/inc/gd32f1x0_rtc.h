/**
  ******************************************************************************
  * @file    gd32f1x0_rtc.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   RTC header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_RTC_H
#define __GD32F1X0_RTC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup RTC
  * @{
  */ 

/** 
  * @brief  RTC Init and config structure def
  */ 
typedef struct
{
    uint16_t RTC_Prescaler_Asyn;    /*!< Specify the RTC Asynchronous Prescaler value.
                                         This parameter must be no more than 0x7F */

    uint16_t RTC_Prescaler_Syn;     /*!< Specify the RTC Synchronous Prescaler value.
                                         This parameter must be no more than 0x1FFF */

    uint32_t RTC_AM_PM;             /*!< Specify the RTC AM/PM value.
                                         This parameter can be any value in @ref RTC_AM_PM_Def */ 

    uint8_t RTC_Hour;               /*!< Specify the RTC Hour value.
                                         This parameter must be 0-12 or 0-23 depending on the time notation chose. */

    uint8_t RTC_Minute;             /*!< Specify the RTC Minute value.
                                         This parameter must be 0-59. */

    uint8_t RTC_Second;             /*!< Specify the RTC Second value.
                                         This parameter must be 0-59. */

    uint8_t RTC_Year;               /*!< Specify the RTC Year value.
                                         This parameter must be 0-99. */

    uint8_t RTC_WeekDay;            /*!< Specify the RTC WeekDay value.
                                         This parameter can be any value in @ref RTC_WeekDay_Def */

    uint8_t RTC_Month;              /*!< Specify the RTC Month value.
                                         This parameter can be any value in @ref RTC_Month_Def */ 

    uint8_t RTC_Date;               /*!< Specify the RTC Date value.
                                         This parameter must be 1-31. */ 

    uint32_t RTC_Hour_Format;       /*!< Specify the RTC Time Notation.
                                         This parameter can be any value in @ref RTC_Hour_Format_Def */
}RTC_InitPara;    

/** 
  * @brief  RTC Alarm structure def  
  */
typedef struct
{
    uint32_t RTC_Alarm_Mask;        /*!< Specify the RTC Alarm Mask.
                                         This parameter can be any value in @ref RTC_Alarm_Mask_Def */       

    uint32_t RTC_Alarm_Weekday_Sel; /*!< Specify the RTC Alarm is on Date or WeekDay.
                                         This parameter can be any value in @ref RTC_Alarm_Date_Def */

    uint8_t RTC_Alarm_Date;         /*!< Specify the RTC Alarm Date or WeekDay.
                                         This parameter must be 1-31 if the Alarm Date is chose.
                                         This parameter can be any value in @ref RTC_WeekDay_Def 
                                         if the Alarm WeekDay is chose. */

    uint32_t RTC_AM_PM;             /*!< Specify the RTC Alarm AM/PM value.
                                         This parameter can be any value in @ref RTC_AM_PM_Def */ 

    uint8_t RTC_Hour;               /*!< Specify the RTC Alarm Hour value.
                                         This parameter must be 0-12 or 0-23 depending on the time notation chose. */

    uint8_t RTC_Minute;             /*!< Specify the RTC Alarm Minute value.
                                         This parameter must be 0-59. */

    uint8_t RTC_Second;             /*!< Specify the RTC Alarm Second value.
                                         This parameter must be 0-59. */
}RTC_AlarmPara;

/** 
  * @brief  RTC Time-stamp structure def  
  */
typedef struct
{
    uint32_t RTC_AM_PM;             /*!< The RTC Time-stamp AM/PM value.*/

    uint8_t RTC_Hour;               /*!< The RTC Time-stamp Hour value */

    uint8_t RTC_Minute;             /*!< The RTC Time-stamp Minute value */

    uint8_t RTC_Second;             /*!< The RTC Time-stamp Second value */

    uint8_t RTC_WeekDay;            /*!< The RTC Time-stamp WeekDay value */

    uint8_t RTC_Month;              /*!< The RTC Time-stamp Month value */

    uint8_t RTC_Date;               /*!< The RTC Time-stamp Date value */  

}RTC_TimeStampPara;    

/** 
  * @brief  RTC Tamper structure def  
  */
typedef struct
{
    uint32_t RTC_Tamper_Source;              /*!< Specify the RTC Tamper Source.
                                                  This parameter can be any value in @ref RTC_Tamper_Source_Def */   

    uint32_t RTC_Tamper_Filter;              /*!< Specify the RTC Tamper Consecutive Samples Needed During a Voltage Level Detection. 
                                                  This parameter can be any value in @ref RTC_Tamper_Filter_Def  */   

    TypeState RTC_Tamper_Precharge;          /*!< Specify the RTC Tamper Precharge Feature During a Voltage Level Detection.
                                                  This parameter can be DISABLE or ENABLE */

    uint32_t RTC_Tamper_Charge_Duration;     /*!< Specify the RTC Tamper Precharge Duration if Precharge Feature is enabled.
                                                  This parameter can be any value in @ref RTC_Tamper_Charge_Duration_Def  */   

    uint32_t RTC_Tamper_Sample_Frequency;    /*!< Specify the RTC Tamper Sampling Frequency During a Voltage Level Detection.  
                                                  This parameter can be any value in @ref RTC_Tamper_Sample_Frequency_Def  */   

    TypeState RTC_Tamper_Timestamp;          /*!< Specify the RTC Tamper Time-stamp Feature.
                                                  This parameter can be DISABLE or ENABLE */

    uint32_t RTC_Tamper_Trigger;             /*!< Specify the RTC Tamper Trigger.  
                                                  This parameter can be any value in @ref RTC_Tamper_Trigger_Def  */  

}RTC_TamperPara;    

/** @defgroup RTC_Exported_Constants
  * @{
  */ 

/** @defgroup RTC_WeekDay_Def
  * @{
  */ 
#define    RTC_MON                                      ((uint8_t)0x01)
#define    RTC_TUE                                      ((uint8_t)0x02)
#define    RTC_WED                                      ((uint8_t)0x03)
#define    RTC_THU                                      ((uint8_t)0x04)
#define    RTC_FRI                                      ((uint8_t)0x05)
#define    RTC_SAT                                      ((uint8_t)0x06)
#define    RTC_SUN                                      ((uint8_t)0x07)

/**
  * @}
  */ 

/** @defgroup RTC_Month_Def
  * @{
  */ 
#define RTC_JAN                                         ((uint8_t)0x01)
#define RTC_FEB                                         ((uint8_t)0x02)
#define RTC_MAR                                         ((uint8_t)0x03)
#define RTC_APR                                         ((uint8_t)0x04)
#define RTC_MAY                                         ((uint8_t)0x05)
#define RTC_JUN                                         ((uint8_t)0x06)
#define RTC_JUL                                         ((uint8_t)0x07)
#define RTC_AUG                                         ((uint8_t)0x08)
#define RTC_SEP                                         ((uint8_t)0x09)
#define RTC_OCT                                         ((uint8_t)0x10)
#define RTC_NOV                                         ((uint8_t)0x11)
#define RTC_DEC                                         ((uint8_t)0x12)

/**
  * @}
  */

/** @defgroup RTC_Hour_Format_Def
  * @{
  */ 
#define RTC_24HOUR                                      ((uint32_t)0x00000000)
#define RTC_12HOUR                                      RTC_CTLR_FMT

/**
  * @}
  */ 

/** @defgroup RTC_AM_PM_Def
  * @{
  */ 
#define RTC_AM                                          ((uint32_t)0x00000000)
#define RTC_PM                                          RTC_TR_PM

/**
  * @}
  */ 
    
/** @defgroup RTC_Alarm_Mask_Def
  * @{
  */ 
#define RTC_ALARM_NONE_MASK                             ((uint32_t)0x00000000)
#define RTC_ALARM_DATE_MASK                             RTC_ALRMR_MSK4 
#define RTC_ALARM_HOUR_MASK                             RTC_ALRMR_MSK3
#define RTC_ALARM_MINUTE_MASK                           RTC_ALRMR_MSK2
#define RTC_ALARM_SECOND_MASK                           RTC_ALRMR_MSK1
#define RTC_ALARM_ALL_MASK                              RTC_ALRMR_MSK4|RTC_ALRMR_MSK3|RTC_ALRMR_MSK2|RTC_ALRMR_MSK1

/**
  * @}
  */ 

/** @defgroup RTC_Alarm_Date_Def
  * @{
  */ 
#define RTC_ALARM_DATE_SELECTED                         ((uint32_t)0x00000000)  
#define RTC_ALARM_WEEKDAY_SELECTED                      RTC_ALRMR_WDSEL    

/**
  * @}
  */ 

/** @defgroup RTC_Alarm_Mask_SubSecond_Def
  * @{
  */ 
#define RTC_ALARM_MASK_SS14_0                           ((uint32_t)0x00) 
#define RTC_ALARM_MASK_SS14_1                           RTC_ALARMSSR_MASKSS_0
#define RTC_ALARM_MASK_SS14_2                           RTC_ALARMSSR_MASKSS_1
#define RTC_ALARM_MASK_SS14_3                           RTC_ALARMSSR_MASKSS_1|RTC_ALARMSSR_MASKSS_0
#define RTC_ALARM_MASK_SS14_4                           RTC_ALARMSSR_MASKSS_2
#define RTC_ALARM_MASK_SS14_5                           RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_0 
#define RTC_ALARM_MASK_SS14_6                           RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_1
#define RTC_ALARM_MASK_SS14_7                           RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_1|RTC_ALARMSSR_MASKSS_0
#define RTC_ALARM_MASK_SS14_8                           RTC_ALARMSSR_MASKSS_3
#define RTC_ALARM_MASK_SS14_9                           RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_0 
#define RTC_ALARM_MASK_SS14_10                          RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_1 
#define RTC_ALARM_MASK_SS14_11                          RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_1|RTC_ALARMSSR_MASKSS_0 
#define RTC_ALARM_MASK_SS14_12                          RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_2
#define RTC_ALARM_MASK_SS14_13                          RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_0 
#define RTC_ALARM_MASK_SS14                             RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_1
#define RTC_ALARM_MASK_NONE                             RTC_ALARMSSR_MASKSS_3|RTC_ALARMSSR_MASKSS_2|RTC_ALARMSSR_MASKSS_1|RTC_ALARMSSR_MASKSS_0

/**
  * @}
  */ 
    
/** @defgroup RTC_Store_Format_Def
  * @{
  */ 
#define RTC_STORE_NORMAL                                ((uint32_t)0x000000000)
#define RTC_STORE_BCD                                   ((uint32_t)0x000000001)

/**
  * @}
  */ 

/** @defgroup RTC_Time_Stamp_Edge_Def
  * @{
  */ 
#define RTC_TS_RISING_EDGE                              ((uint32_t)0x00000000)
#define RTC_TS_FALLING_EDGE                             RTC_CTLR_TSEDGE

/**
  * @}
  */ 

/** @defgroup RTC_Tamper_Source_Def
  * @{
  */ 
#define RTC_TAMPER1                                     RTC_TAFCR_TAP1EN
#define RTC_TAMPER2                                     RTC_TAFCR_TAP2EN

/**
  * @}
  */

/** @defgroup RTC_Tamper_Filter_Def 
  * @{
  */ 
#define RTC_EDGE_DETECTION                              ((uint32_t)0x00000000)
#define RTC_LEVEL_DETECTION_2S                          RTC_TAFCR_FLIT_0                     /*!< Tamper event is detected after 2 consecutive samples at the active level */
#define RTC_LEVEL_DETECTION_4S                          RTC_TAFCR_FLIT_1                     /*!< Tamper event is detected after 4 consecutive samples at the active level */ 
#define RTC_LEVEL_DETECTION_8S                          RTC_TAFCR_FLIT_0 | RTC_TAFCR_FLIT_1  /*!< Tamper event is detected after 8 consecutive samples at the active level */

/**
  * @}
  */ 

/** @defgroup RTC_Tamper_Charge_Duration_Def
  * @{
  */ 
#define RTC_TAMPER_CHARGE_DURATION_1C                   ((uint32_t)0x00000000)  
#define RTC_TAMPER_CHARGE_DURATION_2C                   RTC_TAFCR_PRCH_0 
#define RTC_TAMPER_CHARGE_DURATION_4C                   RTC_TAFCR_PRCH_1  
#define RTC_TAMPER_CHARGE_DURATION_8C                   RTC_TAFCR_PRCH_0 | RTC_TAFCR_PRCH_1  

/**
  * @}
  */

/** @defgroup RTC_Tamper_Sample_Frequency_Def
  * @{
  */ 
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV32768     ((uint32_t)0x00000000) 
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV16384     RTC_TAFCR_FREQ_0 
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV8192      RTC_TAFCR_FREQ_1
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV4096      RTC_TAFCR_FREQ_1 | RTC_TAFCR_FREQ_0
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV2048      RTC_TAFCR_FREQ_2
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV1024      RTC_TAFCR_FREQ_2 | RTC_TAFCR_FREQ_0 
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV512       RTC_TAFCR_FREQ_2 | RTC_TAFCR_FREQ_1 
#define RTC_TAMPER_SAMPLE_FREQUENCY_RTCCLK_DIV256       RTC_TAFCR_FREQ_2 | RTC_TAFCR_FREQ_1 | RTC_TAFCR_FREQ_0

/**
  * @}
  */

/** @defgroup RTC_Tamper_Trigger_Def
  * @{
  */ 
#define RTC_TAMPER_TRIGGER_EDGE_RISING                  ((uint32_t)0x00000000)
#define RTC_TAMPER_TRIGGER_EDGE_FALLING                 RTC_TAFCR_TAP1TRIG
#define RTC_TAMPER_TRIGGER_LEVEL_LOW                    ((uint32_t)0x00000000)
#define RTC_TAMPER_TRIGGER_LEVEL_HIGH                   RTC_TAFCR_TAP1TRIG

/**
  * @}
  */ 

/** @defgroup RTC_Interrupt_Def
  * @{
  */ 
#define RTC_INT_TS                                      RTC_CTLR_TSIE
#define RTC_INT_ALRA                                    RTC_CTLR_AIE
#define RTC_INT_TAMP                                    RTC_TAFCR_TAPIE

/**
  * @}
  */ 

/** @defgroup RTC_Alter_Output_Source_Def
  * @{
  */
#define RTC_512HZ                                       RTC_CTLR_COEN 
#define RTC_1HZ                                         RTC_CTLR_COEN | RTC_CTLR_COS
#define RTC_ALARM_HIGH                                  RTC_CTLR_OS_0 
#define RTC_ALARM_LOW                                   RTC_CTLR_OS_0 | RTC_CTLR_OPOL

/**
  * @}
  */ 

/** @defgroup RTC_Alarm_Output_Mode_Def 
  * @{
  */ 
#define RTC_ALARM_OUTPUT_OD                             ((uint32_t)0x00000000)
#define RTC_ALARM_OUTPUT_PP                             RTC_TAFCR_ALARMOUTTYPE

/**
  * @}
  */ 

/** @defgroup RTC_Calibration_Window_Def
  * @{
  */ 
#define RTC_CALIBRATION_WINDOW_32S                      ((uint32_t)0x00000000)  /*!< 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz */
#define RTC_CALIBRATION_WINDOW_16S                      RTC_CCR_CWND16          /*!< 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz */
#define RTC_CALIBRATION_WINDOW_8S                       RTC_CCR_CWND8           /*!< 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz */

/**
  * @}
  */ 

/** @defgroup RTC_Calibration_Plus_Def
  * @{
  */ 
#define RTC_CALIBRATION_PLUS_SET                        RTC_CCR_CALP
#define RTC_CALIBRATION_PLUS_RESET                      ((uint32_t)0x00000000) 
/**
  * @}
  */ 

/** @defgroup RTC_Shift_Add1s_Def
  * @{
  */ 
#define RTC_SHIFT_ADD1S_RESET                           ((uint32_t)0x00000000)
#define RTC_SHIFT_ADD1S_SET                             RTC_SHIFTCTLR_A1S

/**
  * @}
  */ 

/**
  * @}
  */ 

/** @defgroup RTC_Exported_Functions
  * @{
  */
TypeState RTC_DeInit(void);
TypeState RTC_Config(RTC_InitPara* RTC_config,uint32_t RTC_input_format);
TypeState RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
TypeState RTC_WaitRSF_ToSetAgain(void);
void RTC_GetTimeDate(RTC_InitPara* RTC_config,uint32_t RTC_output_format);
uint32_t RTC_GetSubSecond(void);
void RTC_SetAlarm(RTC_AlarmPara* RTC_alarm_time, uint32_t RTC_input_format);
void RTC_SetAlarmSubSecond(uint32_t RTC_alarm_m_ss,  uint32_t RTC_alarm_ss);
TypeState RTC_Alarm_Enable(TypeState NewValue);
void RTC_GetAlarm(RTC_AlarmPara* RTC_alarm_time, uint32_t RTC_output_format);
uint32_t RTC_GetAlarmSubSecond(void);
void RTC_TimeStamp_Enable(uint32_t RTC_TS_edge, TypeState NewValue);
void RTC_GetTimeStamp(RTC_TimeStampPara* RTC_time_stamp,uint32_t RTC_output_format);
uint32_t RTC_GetTimeStampSubSecond(void);
void RTC_Tamper_Enable(RTC_TamperPara* RTC_tamper , TypeState NewValue);
void RTC_INT_Enable(uint32_t RTC_int, TypeState NewValue);
TypeState RTC_GetBitState(uint32_t RTC_flag);
void RTC_ClearBitState(uint32_t RTC_flag);
void RTC_AlterOutput_Config(uint32_t RTC_alter_output_source, uint32_t RTC_alarm_output_mode);
void RTC_AchieveDST(uint32_t RTC_dst);
void RTC_BypassShadowRegisters_Enable(TypeState NewValue);
TypeState RTC_RefClockDetection_Enable(TypeState NewValue);
TypeState RTC_Shift_Config(uint32_t RTC_shift_add_1s, uint32_t RTC_shift_minus_subsec);
TypeState RTC_Calibration_Config(uint32_t  RTC_calib_window, uint32_t RTC_calib_plus, uint32_t RTC_calib_minus);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /*__GD32F1X0_RTC_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT 2014 GIGADEVICE*****END OF FILE****/

/**
  ******************************************************************************
  * @file    gd32f1x0_rtc.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   RTC functions of the firmware library. 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_rtc.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @defgroup RTC 
  * @brief RTC driver modules
  * @{
  */ 

/** @defgroup RTC_Private_Defines
  * @{
  */
#define INITF_TIMEOUT                               ((uint32_t) 0x00004000)
#define RSF_TIMEOUT                                 ((uint32_t) 0x00008000)
#define RECALPF_TIMEOUT                             ((uint32_t) 0x00001000)
#define SHPF_TIMEOUT                                ((uint32_t) 0x00001000)
#define AWF_TIMEOUT                                 ((uint32_t) 0x00008000)

#define RTC_KEY1                                    0xCA
#define RTC_KEY2                                    0x53
#define RTC_LOCK                                    0xFF

#define RTC_HOUR_POS                                16
#define RTC_MINUTE_POS                              8
#define RTC_MONTH_POS                               8
#define RTC_DATE_POS                                24
#define RTC_WEEKDAY_POS                             13
#define RTC_YEAR_POS                                16
#define RTC_PRESC_ASYN_POS                          16
#define RTC_TMPER_TRI_POS                           1

/**
  * @}
  */

/** @defgroup RTC_Private_Functions
  * @{
  */
static uint8_t RTC_normal2BCD(uint8_t Value);
static uint8_t RTC_BCD2normal(uint8_t Value);

/**
  * @brief  Reset most of the RTC registers.
  * @note   Time stamp registers and SSR are read only, backup domain registers are kept.
  * @param  None
  * @retval An TypeState enumeration value:
  *          - SUCCESS: Reset RTC registers successfully.
  *          - ERROR:  Reset RTC registers failed.
  */
TypeState RTC_DeInit(void)
{
    TypeState error_status = ERROR;

    /* this register is not under w-protection. */
    RTC->TAFCR = 0x00000000;

    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    /* this register can modified outside the init mode */
    RTC->CTLR        &= (uint32_t)0x00000000;

    /* enter init mode */
    error_status = RTC_EnterInitMode();

    if(error_status != ERROR)
    {  
        /* need reset BYPSHAD first */        
        RTC->TR        = (uint32_t)0x00000000;
        RTC->DR        = (uint32_t)0x00002101;

        RTC->PSCR      = (uint32_t)0x007F00FF;

        /* exit init mode */
        RTC->STR = (uint32_t)0x00000007;
        /* need AWF set first or modified in init mode */
        RTC->ALRMR    = (uint32_t)0x00000000;
        /* need reset AE first or modified in init mode */
        RTC->ALARMSSR  = (uint32_t)0x00000000;

        /* this register can modified outside the init mode */
        /* need reset SHPF first */ 
        RTC->SHIFTCTLR    = (uint32_t)0x00000000;       
        RTC->CCR       = (uint32_t)0x00000000;

        error_status = RTC_WaitRSF_ToSetAgain();  
    }

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Initialize and configure the RTC registers .
  * @param  RTC_InitParaStruct: pointer to a RTC_InitPara structure that contains 
  *         the initialization and configuration information for the RTC peripheral.
  * @param  RTC_input_format: specify the format of the input parameters.
  *   This parameter can be  any of the following values:
  *     @arg RTC_STORE_NORMAL:  Binary format 
  *     @arg RTC_STORE_BCD:  BCD format
  * @retval An TypeState enumeration value:
  *          - SUCCESS: Initialize and configure the RTC registers successfully.
  *          - ERROR:  Initialize and configure the RTC registers failed.  
  */
TypeState RTC_Config(RTC_InitPara* RTC_InitParaStruct,uint32_t RTC_input_format)
{
    TypeState error_status = ERROR;
    uint32_t temp_tr = 0, temp_dr = 0;

    /* convert the input format of time configuration structure if needed */
    if (RTC_input_format == RTC_STORE_NORMAL)
    {    
        RTC_InitParaStruct->RTC_Hour = RTC_normal2BCD(RTC_InitParaStruct->RTC_Hour);
        RTC_InitParaStruct->RTC_Minute = RTC_normal2BCD(RTC_InitParaStruct->RTC_Minute);
        RTC_InitParaStruct->RTC_Second = RTC_normal2BCD(RTC_InitParaStruct->RTC_Second);
        RTC_InitParaStruct->RTC_Year = RTC_normal2BCD(RTC_InitParaStruct->RTC_Year);
        RTC_InitParaStruct->RTC_Month = RTC_normal2BCD(RTC_InitParaStruct->RTC_Month);
        RTC_InitParaStruct->RTC_Date = RTC_normal2BCD(RTC_InitParaStruct->RTC_Date);
    }

    temp_tr = (RTC_InitParaStruct->RTC_AM_PM| \
                        ((uint32_t)(RTC_InitParaStruct->RTC_Hour) << RTC_HOUR_POS) | \
                        ((uint32_t)(RTC_InitParaStruct->RTC_Minute) << RTC_MINUTE_POS) | \
                        ((uint32_t)RTC_InitParaStruct->RTC_Second) 
                        ); 

    temp_dr = (((uint32_t)(RTC_InitParaStruct->RTC_Year) << RTC_YEAR_POS) | \
                        ((uint32_t)(RTC_InitParaStruct->RTC_WeekDay) << RTC_WEEKDAY_POS)| \
                        ((uint32_t)(RTC_InitParaStruct->RTC_Month) << RTC_MONTH_POS) | \
                        ((uint32_t)RTC_InitParaStruct->RTC_Date) 
                        ); 
    
    /* step1: Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    /* step2: enter initializaiton mode */
    error_status = RTC_EnterInitMode();

    if(error_status != ERROR)
    {
        RTC->PSCR = (((uint32_t)(RTC_InitParaStruct->RTC_Prescaler_Asyn) << RTC_PRESC_ASYN_POS)| \
                                ((uint32_t)RTC_InitParaStruct->RTC_Prescaler_Syn));

        RTC->TR = (uint32_t)temp_tr;

        RTC->DR = (uint32_t)temp_dr;


        RTC->CTLR &= ((uint32_t)~(RTC_CTLR_FMT));

        RTC->CTLR |=  RTC_InitParaStruct->RTC_Hour_Format;
        /* step3: exit initialization mode */  
        RTC_ExitInitMode();
        /* step4: wait the RSF flag to set again */          
        error_status = RTC_WaitRSF_ToSetAgain();
    }

    /* step5:  Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Enter the RTC Initialization mode.
  * @param  None
  * @retval An TypeState enumeration value:
  *          - SUCCESS: RTC is aready in Init mode.
  *          - ERROR: Enter the RTC Init mode failed.
  */
TypeState RTC_EnterInitMode(void)
{
    uint32_t temp_index = 0;
    uint32_t flag_status = RESET;

    /* check if the initialization mode has been entered already */
    if ((RTC->STR & RTC_STR_INITF) == (uint32_t)RESET)
    {
        RTC->STR |= RTC_STR_INIT;
        /* wait for the INITF flag to set */
        for(temp_index=0x0;temp_index<INITF_TIMEOUT;temp_index++)
        {
            flag_status = RTC->STR & RTC_STR_INITF;

            if(flag_status!=(uint32_t)RESET)
                break;
        }

        if (flag_status!=(uint32_t)RESET)
        {
            return SUCCESS;
        }
        else
        {
            return ERROR;
        }
    }
    else
    {
        return SUCCESS;
    }
}

/**
  * @brief  Exit the RTC Initialization mode.
  * @param  None
  * @retval None
  */
void RTC_ExitInitMode(void)
{
    RTC->STR &= (uint32_t)~RTC_STR_INIT;
}

/**
  * @brief  Wait until the RTC Time and Date registers are synchronized with APB clock.
  * @param  None
  * @retval An TypeState enumeration value:
  *          - SUCCESS: RTC_TR and RTC_DR are updated to the correct value.
  *          - ERROR: The RTC_TR and RTC_DR are not correct.
  */
TypeState RTC_WaitRSF_ToSetAgain(void)
{
    __IO uint32_t temp_index = 0;
    TypeState error_status = ERROR;
    uint32_t flag_status = RESET;

    if ((RTC->CTLR & RTC_CTLR_BYPSHAD) == (uint32_t)RESET)
    {
        /* Disable the w-protection */
        RTC->WPR = RTC_KEY1;
        RTC->WPR = RTC_KEY2;

        /* clear RSF flag first */
        RTC->STR &= (uint32_t)(~RTC_STR_RSF);

        /* wait RSF flag to set again */
        for(temp_index=0x0;temp_index<RSF_TIMEOUT;temp_index++)
        {
            flag_status = RTC->STR & RTC_STR_RSF;

            if(flag_status!=(uint32_t)RESET)
                break;
        }

        if (flag_status!=(uint32_t)RESET)  
        {
            error_status = SUCCESS;
        }
        else
        {
            error_status = ERROR;
        }

        /* Enable the w-protection */
        RTC->WPR = RTC_LOCK;
    }
    /* reading bypass the shadow registers */
    else 
    {
        error_status = SUCCESS;
    }

    return error_status;
}

/**
  * @brief  Get the RTC current Time and Date.
  * @param  RTC_InitParaStruct: pointer to a RTC_InitPara structure that contains 
  *         the current time and Date.
  * @param  RTC_output_format: specify the format of the output parameters.
  *   This parameter can be  any of the following values:
  *     @arg RTC_STORE_NORMAL:  Binary format 
  *     @arg RTC_STORE_BCD:  BCD format 
  * @retval None
  */
void RTC_GetTimeDate(RTC_InitPara* RTC_InitParaStruct,uint32_t RTC_output_format)
{
    uint32_t temp_tr = 0, temp_dr = 0, temp_pscr = 0, temp_ctlr = 0;

    temp_tr = (uint32_t)(RTC->TR );   
    temp_dr = (uint32_t)(RTC->DR );
    temp_pscr = (uint32_t)(RTC->PSCR);
    temp_ctlr = (uint32_t)(RTC->CTLR);
    /* construct the time configuration structure */
    RTC_InitParaStruct->RTC_Prescaler_Asyn = (uint16_t)((temp_pscr & RTC_PSCR_PREDIV_A)>>RTC_PRESC_ASYN_POS);
    RTC_InitParaStruct->RTC_Prescaler_Syn = (uint16_t)(temp_pscr & RTC_PSCR_PREDIV_S);

    RTC_InitParaStruct->RTC_AM_PM = (uint32_t)(temp_tr & RTC_TR_PM);  
    RTC_InitParaStruct->RTC_Hour = (uint8_t)((temp_tr & (RTC_TR_HT | RTC_TR_HU)) >> RTC_HOUR_POS);
    RTC_InitParaStruct->RTC_Minute = (uint8_t)((temp_tr & (RTC_TR_MNT | RTC_TR_MNU)) >>RTC_MINUTE_POS);
    RTC_InitParaStruct->RTC_Second = (uint8_t)(temp_tr & (RTC_TR_ST | RTC_TR_SU));
    RTC_InitParaStruct->RTC_Year = (uint8_t)((temp_dr & (RTC_DR_YT | RTC_DR_YU)) >> RTC_YEAR_POS);
    RTC_InitParaStruct->RTC_WeekDay = (uint8_t)((temp_dr & (RTC_DR_WDU)) >> RTC_WEEKDAY_POS); 
    RTC_InitParaStruct->RTC_Month = (uint8_t)((temp_dr & (RTC_DR_MT | RTC_DR_MU)) >> RTC_MONTH_POS);
    RTC_InitParaStruct->RTC_Date = (uint8_t)(temp_dr & (RTC_DR_DT | RTC_DR_DU));

    RTC_InitParaStruct->RTC_Hour_Format = (uint32_t)(temp_ctlr & RTC_CTLR_FMT);
    /* convert the output format of time configuration structure if needed */
    if (RTC_output_format == RTC_STORE_NORMAL)
    {
        RTC_InitParaStruct->RTC_Hour = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Hour);
        RTC_InitParaStruct->RTC_Minute = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Minute);
        RTC_InitParaStruct->RTC_Second = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Second);
        RTC_InitParaStruct->RTC_Year = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Year);
        RTC_InitParaStruct->RTC_Month = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Month);
        RTC_InitParaStruct->RTC_Date = (uint8_t)RTC_BCD2normal(RTC_InitParaStruct->RTC_Date);
    }
}

/**
  * @brief  Get the RTC current SubSecond value.
  * @param  None
  * @retval RTC current SubSecond value.
  */
uint32_t RTC_GetSubSecond(void)
{
    uint32_t temp_ssr = 0;
    /* reading RTC_SSR will lock RTC_TR and RTC_DR automatically, if BYPSHAD bit is reset */
    temp_ssr = (uint32_t)(RTC->SSR);
    /* read RTC_DR to unlock the 3 shadow registers */
    (void) (RTC->DR);

    return temp_ssr;
}

/**
  * @brief  Set the RTC Alarm 
  * @param  RTC_alarm_time: pointer to a RTC_AlarmPara structure that 
  *         contains the alarm configuration parameters. 
  * @param  RTC_input_format: specify the format of the input parameters.
  *   This parameter can be  any of the following values:
  *     @arg RTC_STORE_NORMAL:  Binary format 
  *     @arg RTC_STORE_BCD:  BCD format
  * @retval None
  */
void RTC_SetAlarm(RTC_AlarmPara* RTC_alarm_time, uint32_t RTC_input_format)
{
    uint32_t temp_alarm = 0;

    /* convert the input format of alarm structure if needed */
    if (RTC_input_format == RTC_STORE_NORMAL)
    {
        RTC_alarm_time->RTC_Alarm_Date = RTC_normal2BCD(RTC_alarm_time->RTC_Alarm_Date);
        RTC_alarm_time->RTC_Hour = RTC_normal2BCD(RTC_alarm_time->RTC_Hour);
        RTC_alarm_time->RTC_Minute = RTC_normal2BCD(RTC_alarm_time->RTC_Minute) ;
        RTC_alarm_time->RTC_Second = RTC_normal2BCD(RTC_alarm_time->RTC_Second) ;  
    }

    temp_alarm = (RTC_alarm_time->RTC_Alarm_Mask| \
                                RTC_alarm_time->RTC_Alarm_Weekday_Sel| \
                                ((uint32_t)(RTC_alarm_time->RTC_Alarm_Date) << RTC_DATE_POS) | \
                                RTC_alarm_time->RTC_AM_PM| \
                                ((uint32_t)(RTC_alarm_time->RTC_Hour) << RTC_HOUR_POS) | \
                                ((uint32_t)(RTC_alarm_time->RTC_Minute) << RTC_MINUTE_POS) | \
                                ((uint32_t)RTC_alarm_time->RTC_Second)); 

    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    RTC->ALRMR = (uint32_t)temp_alarm;

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Set the SubSecond value and mask of RTC Alarm 
  * @param  RTC_alarm_m_ss:  specify the SubSecond Mask.
  *   This parameter can be any of the following values:
  *     @arg  RTC_ALARM_MASK_SS14_0
  *     @arg  RTC_ALARM_MASK_SS14_1
  *     @arg  RTC_ALARM_MASK_SS14_2
  *     @arg  RTC_ALARM_MASK_SS14_3
  *     @arg  RTC_ALARM_MASK_SS14_4
  *     @arg  RTC_ALARM_MASK_SS14_5
  *     @arg  RTC_ALARM_MASK_SS14_6
  *     @arg  RTC_ALARM_MASK_SS14_7
  *     @arg  RTC_ALARM_MASK_SS14_8
  *     @arg  RTC_ALARM_MASK_SS14_9
  *     @arg  RTC_ALARM_MASK_SS14_10
  *     @arg  RTC_ALARM_MASK_SS14_11
  *     @arg  RTC_ALARM_MASK_SS14_12
  *     @arg  RTC_ALARM_MASK_SS14_13
  *     @arg  RTC_ALARM_MASK_SS14
  *     @arg  RTC_ALARM_MASK_NONE 
  * @param  RTC_alarm_ss: specify the SubSecond value.
  *   This parameter can be a value between 0 and 0x00007FFF.
  * @retval None
  */
void RTC_SetAlarmSubSecond(uint32_t RTC_alarm_m_ss,  uint32_t RTC_alarm_ss)
{
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;  

    RTC->ALARMSSR = RTC_alarm_ss |RTC_alarm_m_ss;  

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Enable or disable the RTC Alarm.
  * @param  NewValue: alarm state to configure
  *   This parameter can be: ENABLE or DISABLE.
  * @retval An TypeState enumeration value:
  *          - SUCCESS: enable or disable the RTC Alarm succeed
  *          - ERROR: enable or disable the RTC Alarm failed 
  */
TypeState RTC_Alarm_Enable(TypeState NewValue)
{
    __IO uint32_t temp_index = 0;
    TypeState error_status = ERROR;
    uint32_t flag_status = RESET;

    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;
    /* configure the state of alarm */
    if (NewValue == ENABLE)
    {
        RTC->CTLR |= RTC_CTLR_AE;
        error_status = SUCCESS;
    }
    else
    { 
        RTC->CTLR &= (uint32_t)~RTC_CTLR_AE;  
        /* check if the AWF flag is set after the alarm is disabled */
        for(temp_index=0x0;temp_index<AWF_TIMEOUT;temp_index++)
        {
            flag_status = RTC->STR & RTC_STR_AWF;

            if(flag_status!=(uint32_t)RESET)
                break;
        }

        if (flag_status!=(uint32_t)RESET)  
        {
            error_status = SUCCESS;
        }
        else
        {
            error_status = ERROR;
        }
    } 

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Get the RTC Alarm
  * @param  RTC_alarm_time: pointer to a RTC_AlarmPara structure that 
  *         contains the alarm configuration parameters. 
  * @param  RTC_output_format: specify the format of the output parameters.
  *   This parameter can be any of the following values:
  *     @arg RTC_STORE_NORMAL: Binary format 
  *     @arg RTC_STORE_BCD: BCD format
  * @retval None
  */
void RTC_GetAlarm(RTC_AlarmPara* RTC_alarm_time, uint32_t RTC_output_format)
{
    uint32_t temp_alm = 0;

    /* get the value of RTC_ALRMR register */
    temp_alm = (uint32_t)(RTC->ALRMR);

    /* construct the alarm structure */
    RTC_alarm_time->RTC_Alarm_Mask = temp_alm & RTC_ALARM_ALL_MASK; 
    RTC_alarm_time->RTC_Alarm_Weekday_Sel = temp_alm & RTC_ALRMR_WDSEL;
    RTC_alarm_time->RTC_Alarm_Date = (uint8_t)((temp_alm & (RTC_ALRMR_DT | RTC_ALRMR_DU)) >> RTC_DATE_POS);
    RTC_alarm_time->RTC_AM_PM = temp_alm & RTC_ALRMR_PM;
    RTC_alarm_time->RTC_Hour = (uint8_t)((temp_alm & (RTC_ALRMR_HT | RTC_ALRMR_HU)) >> RTC_HOUR_POS);
    RTC_alarm_time->RTC_Minute = (uint8_t)((temp_alm & (RTC_ALRMR_MNT | RTC_ALRMR_MNU)) >> RTC_MINUTE_POS);
    RTC_alarm_time->RTC_Second = (uint8_t)(temp_alm & (RTC_ALRMR_ST | RTC_ALRMR_SU));

    /* convert the output format of alarm structure if needed */
    if (RTC_output_format == RTC_STORE_NORMAL)
    { 
        RTC_alarm_time->RTC_Alarm_Date = RTC_BCD2normal(RTC_alarm_time->RTC_Alarm_Date);
        RTC_alarm_time->RTC_Hour = RTC_BCD2normal(RTC_alarm_time->RTC_Hour);
        RTC_alarm_time->RTC_Minute = RTC_BCD2normal(RTC_alarm_time-> RTC_Minute);
        RTC_alarm_time->RTC_Second= RTC_BCD2normal(RTC_alarm_time->RTC_Second);
    }  
}

/**
  * @brief  Get the SubSecond value of  RTC Alarm.
  * @param  None
  * @retval RTC Alarm SubSecond
  */
uint32_t RTC_GetAlarmSubSecond(void)
{
    uint32_t temp_almssr= 0;

    temp_almssr = (uint32_t)((RTC->ALARMSSR) & RTC_ALARMSSR_SS);

    return (temp_almssr);
}

/**
  * @brief  Enable or Disable the RTC Time-stamp 
  * @param  RTC_TS_edge: Specify the pin edge on which the Time-stamp is detected.
  *   This parameter can be any of the following values:
  *     @arg RTC_TS_RISING_EDGE: the rising edge activates the time-stamp event 
  *     @arg RTC_TS_FALLING_EDGE: the falling edge activate the time-stamp event 
  * @param  NewValue: Time-stamp state to configure.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RTC_TimeStamp_Enable(uint32_t RTC_TS_edge, TypeState NewValue)
{
    uint32_t temp_ctlr = 0;

    /* Get the RTC_CTLR register and clear the bits to be configured */
    temp_ctlr = (uint32_t)(RTC->CTLR & (uint32_t)~(RTC_CTLR_TSEDGE | RTC_CTLR_TSEN));

    /* Get the new configuration */
    if (NewValue != DISABLE)
    {
        temp_ctlr |= (uint32_t)(RTC_TS_edge | RTC_CTLR_TSEN);
    }
    else
    {
        temp_ctlr |= (uint32_t)(RTC_TS_edge);
    }
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    RTC->CTLR = (uint32_t)temp_ctlr;

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Get the RTC time-stamp time and date.
  * @param  RTC_time_stamp: pointer to a RTC_TimeStampPara structure that will 
  *         contains the time-stamp time and date. 
  * @param  RTC_output_format: specify the format of the output parameters.
  *   This parameter can be any of the following value:
  *     @arg RTC_STORE_NORMAL: Binary format 
  *     @arg RTC_STORE_BCD: BCD format  
  * @retval None
  */
void RTC_GetTimeStamp(RTC_TimeStampPara* RTC_time_stamp,uint32_t RTC_output_format)
{
    uint32_t temp_tstr = 0, temp_tsdr = 0;

    /* get the value of time_stamp registers */
    temp_tstr = (uint32_t)(RTC->TSTR );
    temp_tsdr = (uint32_t)(RTC->TSDR );
    /* construct the time-stamp structure */
    RTC_time_stamp->RTC_AM_PM = temp_tstr & RTC_TSTR_PM;
    RTC_time_stamp->RTC_Hour = (uint8_t)((temp_tstr & (RTC_TSTR_HT | RTC_TSTR_HU)) >> RTC_HOUR_POS);
    RTC_time_stamp->RTC_Minute = (uint8_t)((temp_tstr & (RTC_TSTR_MNT | RTC_TSTR_MNU)) >> RTC_MINUTE_POS);
    RTC_time_stamp->RTC_Second = (uint8_t)(temp_tstr & (RTC_TSTR_ST | RTC_TSTR_SU));

    RTC_time_stamp->RTC_WeekDay = (uint8_t)((temp_tsdr & (RTC_TSDR_WDU)) >> RTC_WEEKDAY_POS);
    RTC_time_stamp->RTC_Month = (uint8_t)((temp_tsdr & (RTC_TSDR_MT | RTC_TSDR_MU)) >> RTC_MONTH_POS);
    RTC_time_stamp->RTC_Date = (uint8_t)(temp_tsdr & (RTC_TSDR_DT | RTC_TSDR_DU));

    if (RTC_output_format == RTC_STORE_NORMAL)
    {
        /* convert the output format of time-stamp structure if needed */
        RTC_time_stamp->RTC_Hour = (uint8_t)RTC_BCD2normal(RTC_time_stamp->RTC_Hour);
        RTC_time_stamp->RTC_Minute = (uint8_t)RTC_BCD2normal(RTC_time_stamp->RTC_Minute);
        RTC_time_stamp->RTC_Second = (uint8_t)RTC_BCD2normal(RTC_time_stamp->RTC_Second);
        RTC_time_stamp->RTC_Month = (uint8_t)RTC_BCD2normal(RTC_time_stamp->RTC_Month);
        RTC_time_stamp->RTC_Date = (uint8_t)RTC_BCD2normal(RTC_time_stamp->RTC_Date);
    }
}

/**
  * @brief  Get the RTC time-stamp SubSecond value.
  * @param  None
  * @retval RTC time-stamp SubSecond value.
  */
uint32_t RTC_GetTimeStampSubSecond(void)
{
    return (uint32_t)(RTC->TSSSR);
}

/**
  * @brief  Enable or disable the RTC Tamper.
  * @param  RTC_tamper: pointer to a RTC_TamperPara structure that contains 
  *         the configuration information for the RTC Tamper.
  * @param  NewValue: tamper state to configure
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RTC_Tamper_Enable(RTC_TamperPara* RTC_tamper , TypeState NewValue)
{
    /* Disable tamper */
    RTC->TAFCR &= (uint32_t)~(RTC_tamper->RTC_Tamper_Source); 

    if(NewValue != DISABLE)
    {
        /* tamper filter must be used when the tamper source is voltage level detection */
        RTC->TAFCR &= (uint32_t)~RTC_TAFCR_FLIT;
        /* the tamper source is voltage level detection */
        if(RTC_tamper->RTC_Tamper_Filter != RTC_EDGE_DETECTION ) 
        {
            RTC->TAFCR &= (uint32_t)~(RTC_TAFCR_DISPU | RTC_TAFCR_PRCH | RTC_TAFCR_FREQ | RTC_TAFCR_FLIT);

            /* check if the tamper pin need precharge before sampling the voltage level */
            if(RTC_tamper->RTC_Tamper_Precharge == DISABLE)
            {
                RTC->TAFCR |=  (uint32_t)RTC_TAFCR_DISPU;    
            }
            else
            {
                /* configure the precharge duration */
                RTC->TAFCR |= (uint32_t)(RTC_tamper->RTC_Tamper_Charge_Duration);
            }
            /* configure the sampling frequency */
            RTC->TAFCR |= (uint32_t)(RTC_tamper->RTC_Tamper_Sample_Frequency);

            /* configure the tamper filter */
            RTC->TAFCR |= (uint32_t)(RTC_tamper->RTC_Tamper_Filter);
        }
        RTC->TAFCR &= (uint32_t)~RTC_TAFCR_TAPTS;  
        if(RTC_tamper->RTC_Tamper_Timestamp!= DISABLE)
        {        
            /* the tamper event also cause a time-stamp event */
            RTC->TAFCR |= (uint32_t)RTC_TAFCR_TAPTS ;
        }
        /* configure the tamper trigger */
        RTC->TAFCR &= ((uint32_t)~((RTC_tamper->RTC_Tamper_Source) << RTC_TMPER_TRI_POS ));    
        if(RTC_tamper->RTC_Tamper_Trigger!=RTC_TAMPER_TRIGGER_EDGE_RISING)
        {
            RTC->TAFCR |= (uint32_t)((RTC_tamper->RTC_Tamper_Source)<< RTC_TMPER_TRI_POS );  
        }    
        /* enable tamper */
        RTC->TAFCR |=  (uint32_t)(RTC_tamper->RTC_Tamper_Source); 
    }
}

/**
  * @brief  Enable or disable the specified RTC interrupts.
  * @param  RTC_int: specify the RTC interrupt sources 
  *   This parameter can be any combination of the following value:
  *     @arg RTC_INT_TS:  Time Stamp event interrupt
  *     @arg RTC_INT_ALRA:  Alarm event interrupt 
  *     @arg RTC_INT_TAMP: Tamper event interrupt 
  * @param  NewValue:  RTC interrupt state to configure
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RTC_INT_Enable(uint32_t RTC_int, TypeState NewValue)
{  
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    if (NewValue == ENABLE)
    {  
        /* Enable the interrupts in RTC_CTLR register */
        RTC->CTLR |= (uint32_t)(RTC_int & ~RTC_TAFCR_TAPIE);
        /* Enable the interrupts in RTC_TAFCR register */
        RTC->TAFCR |= (uint32_t)(RTC_int & RTC_TAFCR_TAPIE);
    }
    else
    {  
        /* Disable the interrupts in RTC_CTLR register */
        RTC->CTLR &= (uint32_t)~(RTC_int & (uint32_t)~RTC_TAFCR_TAPIE);
        /* Disable the interrupts in RTC_TAFCR register */
        RTC->TAFCR &= (uint32_t)~(RTC_int & RTC_TAFCR_TAPIE);
    }

    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK; 
}

/**
  * @brief  Check if the specified flag in RTC_STR is set or not.
  * @param  RTC_flag: specify the flag in RTC_STR to check.
  *   This parameter can be any of the following values:      
  *     @arg RTC_STR_RECALPF: Recalibration pending flag
  *     @arg RTC_STR_TAMP2F: Tamper 2 event flag   
  *     @arg RTC_STR_TAMP1F: Tamper 1 event flag
  *     @arg RTC_STR_TSOVF: Time-stamp overflow event flag
  *     @arg RTC_STR_TSF: Time-stamp event flag
  *     @arg RTC_STR_AF: Alarm event flag
  *     @arg RTC_STR_INITF: Initialization mode event flag
  *     @arg RTC_STR_RSF: Time and date registers synchronized event flag
  *     @arg RTC_STR_INITS: Year parameter configured event flag
  *     @arg RTC_STR_SHPF: Shift operation pending flag
  *     @arg RTC_STR_AWF: Alarm writen available flag
  * @retval The RTC_flag state returned(SET or RESET).
  */
TypeState RTC_GetBitState(uint32_t RTC_flag)
{
    uint32_t temp_str = 0;
    temp_str = RTC->STR;
    if ((temp_str & RTC_flag) != (uint32_t)RESET)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/**
  * @brief  Clear the flags in RTC_STR.
  * @param  RTC_flag: specify the flags in RTC_STR to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_STR_TAMP2F: Tamper 2 event flag
  *     @arg RTC_STR_TAMP1F: Tamper 1 event flag 
  *     @arg RTC_STR_TSOVF: Time Stamp Overflow event flag 
  *     @arg RTC_STR_TSF: Time Stamp event flag
  *     @arg RTC_STR_AF: Alarm event flag
  *     @arg RTC_STR_RSF: Time and Date Registers Synchronized event flag
  * @retval None
  */
void RTC_ClearBitState(uint32_t RTC_flag)
{
    RTC->STR &= (uint32_t)(~RTC_flag);  
}

/**
  * @brief  Configure the RTC alternate output source
  * @param  RTC_alter_output_source: Specify which signal will be output
  *   This parameter can be any of the following:
  *     @arg RTC_512HZ: when the LSE freqency is 32768Hz and the RTC_PSCR 
  *          is the default value, the output pin output 512Hz signal
  *     @arg RTC_1HZ: when the LSE freqency is 32768Hz and the RTC_PSCR 
  *          is the default value, the output pin output 1Hz signal
  *     @arg RTC_ALARM_HIGH: when the  Alarm flag is set, the output pin is high 
  *     @arg RTC_ALARM_LOW    : when the  Alarm flag is set, the output pin is low 
  * @param  RTC_alarm_output_mode: Specify the output pin (PC13) mode when output alarm signal   
  *   This parameter can be any of the following:
  *     @arg RTC_ALARM_OUTPUT_OD : Open Drain mode   
  *     @arg RTC_ALARM_OUTPUT_PP : Push Pull mode 
  * @retval None
  */
void RTC_AlterOutput_Config(uint32_t RTC_alter_output_source, uint32_t RTC_alarm_output_mode)
{
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    RTC->CTLR &= (uint32_t)~(RTC_CTLR_COEN | RTC_CTLR_OS | RTC_CTLR_OPOL | RTC_CTLR_COS);

    RTC->CTLR |= (uint32_t)(RTC_alter_output_source);
    /* alarm output */
    if(RTC_alter_output_source & RTC_CTLR_OS_0 != RESET)
    {
        RTC->TAFCR &= (uint32_t)~(RTC_TAFCR_ALARMOUTTYPE);
        RTC->TAFCR |= (uint32_t)(RTC_alarm_output_mode);  
    }
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Achieve the daylight saving time by adding or substracting one hour from the current time .
  * @param  RTC_dst: hour adjustment value. 
  *   This parameter can be any of the following values:
  *     @arg RTC_CTLR_S1H: Substract one hour
  *     @arg RTC_CTLR_A1H: Add one hour
  * @retval None
  */
void RTC_AchieveDST(uint32_t RTC_dst)
{
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    RTC->CTLR |= (uint32_t)(RTC_dst);
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Enable or Disable the RTC bypass shadow registers function.
  * @param  NewValue: the Bypass Shadow Registers function state to configure.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
*/
void RTC_BypassShadowRegisters_Enable(TypeState NewValue)
{ 
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;

    if (NewValue != DISABLE)
    {
        RTC->CTLR |= (uint8_t)RTC_CTLR_BYPSHAD;
    }
    else
    {
        RTC->CTLR &= (uint8_t)~RTC_CTLR_BYPSHAD;
    }
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;
}

/**
  * @brief  Enable or disable the RTC reference clock detection function.
  * @param  NewValue: the RTC reference clock detection state to configure. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval An TypeState enumeration value:
  *          - SUCCESS: enable the RTC reference clock detection succeeded
  *          - ERROR: enable the RTC reference clock detection failed  
  */
TypeState RTC_RefClockDetection_Enable(TypeState NewValue)
{
    TypeState error_status = ERROR;
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;    

    /* enter the initialization mode */
    error_status = RTC_EnterInitMode();

    if(error_status != ERROR)
    {  
        if (NewValue != DISABLE)
        {      
            RTC->CTLR |= RTC_CTLR_REFCKON;
        }
        else
        {     
            RTC->CTLR &= ~RTC_CTLR_REFCKON;
        }  
        /* exit the initialization mode */
        RTC_ExitInitMode();  
    }    
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Configure the Shift function control register
  * @param  RTC_shift_add_1s: Add 1 second to current time or not 
  *   This parameter can be any of the following values :
  *     @arg RTC_SHIFT_ADD1S_SET: Add 1 second to current time. 
  *     @arg RTC_SHIFT_ADD1S_RESET: No effect.
  * @param  RTC_shift_minus_subsec: The number of SubSecond to minus from current time.
  *   This parameter can be any value from 0 to 0x7FFF.
  * @retval An TypeState enumeration value:
  *          - SUCCESS: Configure the RTC Shift register succeeded.
  *          - ERROR: Configure the RTC Shift register failed.
*/
TypeState RTC_Shift_Config(uint32_t RTC_shift_add_1s, uint32_t RTC_shift_minus_subsec)
{
    uint32_t temp_index = 0;
    TypeState error_status = ERROR;
    uint32_t flag_status = RESET;
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;
    /* check if a shift operation is ongoing */    
    for(temp_index=0x0;temp_index<SHPF_TIMEOUT;temp_index++)
    {
        flag_status = RTC->STR & RTC_STR_SHPF;

        if(flag_status ==(uint32_t)RESET)
            break;
    }

    if(flag_status ==(uint32_t)RESET)  
    { 
        /* check if the function of reference clock detection is disabled */
        if((RTC->CTLR & RTC_CTLR_REFCKON) == RESET)
        {
            RTC->SHIFTCTLR = (uint32_t)(RTC_shift_add_1s | RTC_shift_minus_subsec);
            error_status = RTC_WaitRSF_ToSetAgain(); 
        }
        else
        {
            error_status = ERROR;
        }
    }
    else
    {
        error_status = ERROR;
    }
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Configure the Calibration configure register.
  * @param  RTC_calib_window: Select the Calibration Window.
  *   This parameter can be any of the following values:
  *     @arg RTC_CALIBRATION_WINDOW_32S : The calibration window is 32s when RTC clock is 32768Hz.
  *     @arg RTC_CALIBRATION_WINDOW_16S : The calibration window is 16s when RTC clock is 32768Hz.
  *     @arg RTC_CALIBRATION_WINDOW_8S : The calibration window is 8s when RTC clock is 32768Hz.
  * @param  RTC_calib_plus: Add RTC clock or not
  *   This parameter can be any of the following values:
  *     @arg RTC_CALIBRATION_PLUS_SET: Add one RTC clock every 2048 RTC clock.
  *     @arg RTC_CALIBRATION_PLUS_RESET: No RTC clock are added.
  * @param  RTC_calib_minus: the RTC clock to minus during the calibration window
  *   This parameter can be any value from 0 to 0x000001FF.
  * @retval An TypeState enumeration value:
  *          - SUCCESS: Configure the RTC Calib register succeeded.
  *          - ERROR: Configure the RTC Calib register failed.
*/
TypeState RTC_Calibration_Config(uint32_t  RTC_calib_window, uint32_t RTC_calib_plus, uint32_t RTC_calib_minus)
{
    uint32_t temp_index = 0;
    TypeState error_status = ERROR;
    uint32_t flag_status = RESET;
    /* Disable the w-protection */
    RTC->WPR = RTC_KEY1;
    RTC->WPR = RTC_KEY2;    
    /* check if a calibration operation is ongoing */    
    for(temp_index=0x0;temp_index<RECALPF_TIMEOUT;temp_index++)
    {
        flag_status = RTC->STR & RTC_STR_RECALPF;
        if(flag_status ==(uint32_t)RESET)
            break;
    }
    if(flag_status==(uint32_t)RESET)  
    { 
        RTC->CCR = (RTC_calib_window | RTC_calib_plus | RTC_calib_minus);
        error_status = SUCCESS;
    }
    else
    {
        error_status = ERROR;
    }
    /* Enable the w-protection */
    RTC->WPR = RTC_LOCK;

    return error_status;
}

/**
  * @brief  Convert Binary format to BCD format.
  * @param  Value: data to be converted.
  * @retval Converted data
  */
static uint8_t RTC_normal2BCD(uint8_t data)
{
    uint8_t bcd_high = 0;

    while (data >= 10)
    {
        bcd_high++;
        data -= 10;
    }

    return  ((uint8_t)(bcd_high << 4) | data);
}

/**
  * @brief  Convert from  BCD format to Binary format.
  * @param  Value: data to be converted.
  * @retval Converted data
  */
static uint8_t RTC_BCD2normal(uint8_t data)
{
    uint8_t temp = 0;
    temp = ((uint8_t)(data & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (temp + (data & (uint8_t)0x0F));
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

/************************ (C) COPYRIGHT 2014 GIGADEVICE*****END OF FILE****/

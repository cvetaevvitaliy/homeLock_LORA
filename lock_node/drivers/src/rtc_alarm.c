#include "rtc_alarm.h"
#include "main.h"
#include "process.h"
#include "string.h"

RTC_HandleTypeDef hrtc;
/* RTC init function */
void init_RTC(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    while(1);
  }

    /**Initialize RTC and set the Time and Date 
    */
  //if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x31F2){
		sTime.Hours = INIT_HOUR;
		sTime.Minutes = INIT_MINITUE;
		sTime.Seconds = INIT_SECOND;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		{
			while(1);
		}

		sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
		sDate.Month = INTI_MONTH;
		sDate.Date = INIT_DAY;
		sDate.Year = INIT_YEAR;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
		{
			while(1);
		}

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x3F2);
  //}
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = INIT_ALARM_HOUR;
  sAlarm.AlarmTime.Minutes = INIT_ALARM_MINUTE;
  sAlarm.AlarmTime.Seconds = 0x00;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    while(1);
  }
}
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{

  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }

}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(RTC_IRQn);

  }
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */

}
void RTC_IRQHandler(void){
  /* USER CODE BEGIN RTC_IRQn 0 */

  /* USER CODE END RTC_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_IRQn 1 */

  /* USER CODE END RTC_IRQn 1 */
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	uint8_t send_buf[130] = {UPDATE_TIME_REQ};
	alarm_valid = 1;
	if(alarm_valid == 1){
		uint8_t len = isEmptyRecord();
		uint8_t sendBufLen = sizeof(Record)*len;
		if(len != 0xFF){
			memcpy(send_buf, record, sendBufLen);
			sendToServer(send_buf, sendBufLen, 0xE0 , 1);
			delAllRecord();
		}else{
			send_buf[0] = UPDATE_TIME_REQ;
			sendToServer(send_buf, 1, UPDATE_TIME_CMD, 1);			
		}
		alarm_valid = 0;
	}
}

void calDateTime(uint8_t* calTime){
	RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
	
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
		sTime.Hours = calTime[3];
		sTime.Minutes = calTime[4];
		sTime.Seconds = calTime[5];
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		{
			while(1);
		}

		sDate.WeekDay = RTC_WEEKDAY_MONDAY; //not care
		sDate.Month = calTime[1];
		sDate.Date = calTime[2];
		sDate.Year = calTime[0];

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
		{
			while(1);
		}

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
	}
	
}
void setAlarm(uint8_t hour, uint8_t minute){
  RTC_AlarmTypeDef sAlarm;
	/**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = hour;
  sAlarm.AlarmTime.Minutes = minute;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    while(1);
  }
	
}

//------    end      -------

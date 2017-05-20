/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    21-Nov-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"


#include "low_power_conf.h"
#include "profile_application.h"

#include "stm32xx_timerserver.h" 
#include "stm32xx_hal_app_rtc.h"
#include "stm32l4xx_hal_def.h"

#include "cube_hal.h"

#define BLE_CURRENT_PROFILE_ROLES  (HID_DEVICE) 
#define LOW_POWER_ENABLED 0
/* Exported defines -----------------------------------------------------------*/
#define HCLK_32MHZ 0 /* can be set to 1 only for STM32L053xx */
#define HCLK_80MHZ 0 /* can be set to 1 only for STM32L476RG */ /* SO: TO BE VERIFIED FOR L4. Do not use by now. */
#define HCLK_84MHZ 0 /* can be set to 1 only for STM32F401xE */

/**
 * RTC cloc divider
 */
#define WUCKSEL_DIVIDER (3)		/**< Tick is (LSI speed clock/2) */
#define RTC_ASYNCH_PRESCALER (1)
#define RTC_SYNCH_PRESCALER (0xFFFF)

volatile uint8_t send_measurement = 0;

tProfileApplContext profileApplContext; /* Profile Application Context */

RTC_HandleTypeDef hrtc;  /* RTC handler declaration */
extern volatile uint8_t send_measurement;
#if (BLE_CURRENT_PROFILE_ROLES & HID_DEVICE)
extern volatile uint8_t skip_hid_tick_inc;
extern volatile uint8_t hid_tick;
#endif
static void Init_RTC(void);

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;

extern TIM_HandleTypeDef TimHandle;
extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Exported Variables -------------------------------------------------------------*/

uint32_t ConnectionBleStatus  =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef TimCCHandle;

uint8_t bdaddr[6];

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t HCI_ProcessEvent=0;
static volatile uint32_t SendEnv         =0;

SensorAxes_t GYR_Value;

int xValue,yValue;
//uint8_t CDC_Fill_Buffer(uint8_t* Buf, uint32_t TotalLen){}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t low_power_enabled;
  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
#if LOW_POWER_ENABLED
  low_power_enabled = TRUE;
#else
  low_power_enabled = FALSE;
#endif
  
  /* Configure the System clock */
  SystemClock_Config();

  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();

  InitTargetPlatform(TARGET_SENSORTILE);
      
  /* Configure the system in Low Power Mode */
  if (low_power_enabled) {
    SystemPower_Config();
  }
  
  /* Initialize the Profile Application Context's Data Structures */
  Osal_MemSet(&profileApplContext,0,sizeof(tProfileApplContext));

  /* Configure the RTC */
  Init_RTC();  
  TIMER_Init(&hrtc);
  TIMER_Create(eTimerModuleID_BlueNRG_Profile_App, &(profileApplContext.profileTimer_Id), eTimerMode_Repeated, 0);
    
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
  
  /* Set current BlueNRG profile (HRM, HTM, GS, ...) */
  BNRG_Set_Current_profile();
  
  /* Initialize the BlueNRG Profile */
  /* set tx power and security parameters (common to all profiles) */
  BNRG_Profiles_Init(); 
  /* low level profile initialization (profile specific) */
  profileApplContext.initProfileFunc();
  
  BLE_Profile_Write_DeviceState(APPL_UNINITIALIZED);
  



  while(1){
    
    /* Read the Gyro values */
    BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);
    xValue = -(GYR_Value.AXIS_Z /1500);
    yValue = GYR_Value.AXIS_X/1500;    
//    xValue = GYR_Value.AXIS_X /1500;
//    yValue = GYR_Value.AXIS_Z/1500;
    HCI_Process();
    profileApplContext.profileStateMachineFunc();
    if (Profile_Process_Q() == 0x00)
    {
      if (low_power_enabled)
      {
        if (profileApplContext.profileApplicationProcessFunc() != 0) {
          LPM_Mode_Request(eLPM_MAIN_LOOP_PROCESSES, eLPM_Mode_RUN);
        }else{
          LPM_Mode_Request(eLPM_MAIN_LOOP_PROCESSES, eLPM_Mode_LP_Stop);
        }
      }
      else
      {
        profileApplContext.profileApplicationProcessFunc();
      }
    }
//    LPM_Enter_Mode();
    
  } /* end while(1) */
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimHandle)) {
    CDC_TIM_PeriodElapsedCallback(htim);
  }
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
    case BNRG_SPI_EXTI_PIN:
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: STLBLE_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif


/**
  * @brief  Initialize RTC block
  *
  * @note
  * @param  None
  * @retval None
  */
static void Init_RTC(void)
{
  /* Initialize the HW - 37Khz LSI being used */
  /* Enable the LSI clock */
  __HAL_RCC_LSI_ENABLE();
  
  /* Enable power module clock */
  __PWR_CLK_ENABLE();
  
  /* Enable acces to the RTC registers */
  HAL_PWR_EnableBkUpAccess();
  
  /**
   *  Write twice the value to flush the APB-AHB bridge
   *  This bit shall be written in the register before writing the next one
   */
  HAL_PWR_EnableBkUpAccess();

  /* Select LSI as RTC Input */
  __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
  
  /* Enable RTC */
  __HAL_RCC_RTC_ENABLE();
  
  hrtc.Instance = RTC;                  /**< Define instance */
  hrtc.Lock = HAL_UNLOCKED;             /**< Initialize lock */
  hrtc.State = HAL_RTC_STATE_READY;     /**< Initialize state */
  
  /**
  * Bypass the shadow register
  */
  HAL_RTCEx_EnableBypassShadow(&hrtc);
  
  /**
  * Set the Asynchronous prescaler
  */
  hrtc.Init.AsynchPrediv = RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = RTC_SYNCH_PRESCALER;
  HAL_RTC_Init(&hrtc);
  
  /* Disable Write Protection */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  
  HAL_APP_RTC_Set_Wucksel(&hrtc, WUCKSEL_DIVIDER);  /**< Tick timer is 55us */
  
  /* Wait for LSI to be stable */
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == 0);
  
  return;
}

void TIMER_Notification(eTimerProcessID_t eTimerProcessID, uint8_t ubTimerID, pf_TIMER_TimerCallBack_t pfTimerCallBack)
{
  uint32_t uwPRIMASK_Bit = __get_PRIMASK();	/**< backup PRIMASK bit */;
  
  switch (eTimerProcessID)
  {
  case eTimerModuleID_BlueNRG_Profile_App:
#if !((BLE_CURRENT_PROFILE_ROLES & ALERT_NOTIFICATION_SERVER) || (BLE_CURRENT_PROFILE_ROLES & HID_DEVICE))
    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);    
    __disable_irq();  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/

    send_measurement++;
    __set_PRIMASK(uwPRIMASK_Bit);  /**< Restore PRIMASK bit*/
#endif
#if (BLE_CURRENT_PROFILE_ROLES & HID_DEVICE)
    if(skip_hid_tick_inc == 0) {
      /* Clear Wake Up Flag */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      __disable_irq();  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/

      hid_tick++;
      __set_PRIMASK(uwPRIMASK_Bit);  /**< Restore PRIMASK bit*/
    }
#endif
    break;
  default:
    if (pfTimerCallBack != 0)
    {
      pfTimerCallBack();
    }
    break;
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

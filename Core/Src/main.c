/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void Breathing_light(int);
void Water_light(int);
void ADC_V(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define LED_MAX 3
#define USE_BRIGHTNESS 1
int t=0;

        uint8_t LED_Data[LED_MAX][4];
        uint8_t LED_Mod[LED_MAX][4];
        uint16_t raw;
        char msg2[50];

        int datasentflag = 0;


void HAL_TIM_PWM_PulseFinshedCallback(TIM_HandleTypeDef *htim)
{
        HAL_TIM_PWM_Stop_DMA(&htim3,TIM_CHANNEL_1);
        datasentflag = 1;

}

        void Set_LED (int LEDnum,int Green,int Red,int Blue)
        {

                LED_Data[LEDnum][0] = LEDnum;
                LED_Data[LEDnum][1] = Green;
                LED_Data[LEDnum][2] = Red;
                LED_Data[LEDnum][3] = Blue;

        }

        void Set_Brightness(int brightness) {
            float factor = (float)brightness / 45.0f; // 计算亮度系数
            for(int i = 0; i < LED_MAX; i++) {
                for(int j = 1; j < 4; j++) {
                    // 根据亮度系数调整颜色�??
                    LED_Mod[i][j] = (uint8_t)(LED_Data[i][j] * factor);
                }
            }
        }


uint16_t pwmData[24*LED_MAX+240];
void WS2812_Send(void) {
    uint32_t indx = 0;
    uint32_t color;

    for(int i = 0; i < LED_MAX; i++) {
        color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

        for(int i = 23; i >= 0; i--) {
            if(color & (1 << i)) {
                pwmData[indx] = 65;
            } else {
                pwmData[indx] = 25;
            }
            indx++;
        }
    }

    for(int i = 0; i < 240; i++) {
        pwmData[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
        datasentflag = 1;
    }
}
void Breathing_light(int speed) {
    for(int i = 0; i < 45; i++) {
        Set_Brightness(i);
        WS2812_Send();
        HAL_Delay(speed);
    }
    for(int i = 45; i > 0; i--) {
        Set_Brightness(i);
        WS2812_Send();
        HAL_Delay(speed);
    }
    WS2812_Send();
    HAL_Delay(speed);
}

//speed为变换速度，数字越小越快
void Water_light(int speed) {

        t++;
        if (t % 3 == 0) {
            Set_LED(0, 255, 0, 0);
            Set_LED(1, 0, 255, 0);
            Set_LED(2, 0, 0, 255);
        } else if (t % 3 == 1) {
            Set_LED(1, 255, 0, 0);
            Set_LED(2, 0, 255, 0);
            Set_LED(0, 0, 0, 255);
        } else {
            Set_LED(0, 255, 0, 0);
            Set_LED(2, 0, 255, 0);
            Set_LED(1, 0, 0, 255);
        }
        Set_Brightness(50);
        WS2812_Send();
        HAL_Delay(speed);

}

void  ADC_V(void)
    {
                              // 开始ADC采样
                              HAL_ADC_Start(&hadc1);
                              HAL_ADC_PollForConversion(&hadc1,300);
                             // 获取ADC转换结果并计算电压
                              raw = HAL_ADC_GetValue(&hadc1);
                              float vin = raw*(3.3/4096);
                           // 创建包含电压值的字符串消息
                              sprintf(msg2,"vol = %.2f\r\n",vin);
                            // 通过串口发送电压值信息
                              HAL_UART_Transmit(&huart1,(uint8_t *)msg2,strlen(msg2),300);
                              HAL_Delay(100);
                               if (vin < 1) // 小于1v
                        {

                             Set_LED(0, 255, 0, 0);
                             Set_LED(1, 255, 0, 0);
                             Set_LED(2, 255, 0, 0);// 绿光
                             Set_Brightness(50);
                                WS2812_Send();
                        }
                        else if(vin < 2)//1~2
                        {
                                 Set_LED(0, 0, 0, 255);
                                 Set_LED(1,0, 0, 255);
                                 Set_LED(2, 0, 0, 255);// 蓝光
                                 Set_Brightness(50);
                                    WS2812_Send();
                        }
                        else// >2
                        {
                                Set_LED(0,0,255, 0);
                                Set_LED(1,0, 255,0);
                                Set_LED(2,0,255,0);// 红光
                               Set_Brightness(50);
                                  WS2812_Send();
                       }
                    HAL_Delay(50);

        }

/*void send (int Green,int Red,int Blue)
{
                        uint32_t data=(Green<<16)|(Red<<8)|Blue;

        for(int i=23;i>=0;i--){

                if(data&(1<<i)) pwmData[i]=60;
                else pwmData[i]=30;


        }
        HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_1,(uint32_t *)pwmData,24);


        }*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //（）说要有光，于是便有了光
   for(int i=0;i<=LED_MAX;i++){
	 if(i%3==1) Set_LED(i,255,0,0);
	 else if(i%3==2) Set_LED(i,0,255,0);
	 else if (i%3==0)  Set_LED(i,255,255,266);
  }

   Set_Brightness(50);
   WS2812_Send();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
          //三种灯效，想用那个，删掉前面的“//”就行，1和2可以组合用,speed为变换速度，数字越小越快
	  	  	//Breathing_light(10);
          //Water_light(100) ;
           // ADC_V();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

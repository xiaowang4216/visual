/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_GPIO.h"
#include "BSP_USART.h"
#include "BSP_spi.h"
#include "stdio.h"	
#include "tftlcd.h"
#include "lora_cfg.h"
#include "mpu6050.h"
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
u8 rx2_buf;
u8 gear=48;

short x;
short y;
short z;
float temp=0;
short ax=0;
short ay=0;
short az=0;
float temp_z_angle=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	char str[50] = {0};
	u8 t;
	char tbuf[40];
	
	
//LoRa模块配置参数
//设备参数初始化(具体设备参数见lora_cfg.h定义)
_LoRa_CFG LoRa_CFG=
{
	.addr = LORA_ADDR,       //设备地址
	.power = LORA_POWER,     //发射功率
	.chn = LORA_CHN,         //信道
	.wlrate = LORA_RATE,     //空中速率
	.wltime = LORA_WLTIME,   //睡眠时间
	.mode = LORA_MODE,       //工作模式
	.mode_sta = LORA_STA,    //发送状态
	.bps = LORA_TTLBPS ,     //波特率设置
	.parity = LORA_TTLPAR    //校验位设置
};

void LoRa_Set(void)
{
	u8 sendbuf[20];
	u8 lora_addrh,lora_addrl=0;
	
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET); //进入配置模式
	HAL_Delay(40);

//	huart2.Instance = USART2;
//	huart2.Init.BaudRate = LORA_TTLBPS_115200;
//	UART_SetConfig(&huart2);
//	printf("lora_set begin\n\r");
	
	lora_addrh =  (LoRa_CFG.addr>>8)&0xff;
	lora_addrl = LoRa_CFG.addr&0xff;
	sprintf((char*)sendbuf,"AT+ADDR=%02x,%02x\n\r",lora_addrh,lora_addrl);//设置设备地址
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+WLRATE=%d,%d\n\r",LoRa_CFG.chn,LoRa_CFG.wlrate);//设置信道和空中速率
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+TPOWER=%d\n\r",LoRa_CFG.power);//设置发射功率
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+CWMODE=%d\n\r",LoRa_CFG.mode);//设置工作模式
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+TMODE=%d\n\r",LoRa_CFG.mode_sta);//设置发送状态
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+WLTIME=%d\n\r",LoRa_CFG.wltime);//设置睡眠时间
	printf("%s",sendbuf);
	HAL_Delay(100);
	sprintf((char*)sendbuf,"AT+UART=%d,%d\n\r",LoRa_CFG.bps,LoRa_CFG.parity);//设置串口波特率、数据校验位
	printf("%s",sendbuf);
	HAL_Delay(100);

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET); //退出配置模式
	HAL_Delay(40);

//	usart3_set(LoRa_CFG.bps,LoRa_CFG.parity);//返回通信,更新通信串口配置(波特率、数据校验位)

//	huart2.Init.BaudRate = 9600;
//	HAL_UART_Init(&huart2);
	

}	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	u8 color = 0;
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART1_UART_Init();
  MX_SPI6_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//  printf("begin\n\r");
  HAL_UART_Receive_IT(&huart2, &rx2_buf, 1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	MPU_Init();

	__HAL_SPI_ENABLE(&hspi6);
	LCD_Init();
	LCD_Clear(BLACK); 		//清屏	
	BACK_COLOR=BLACK;
	POINT_COLOR=CYAN;	
	LCD_ShowString(0, 0, 160, 12, 12, "TFTLCD 240*135");
	LoRa_Set();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(color%7)
		{
			case 0:
				LED_R(0);
				LED_G(1);
				LED_B(1);
				break;
			case 1:
				LED_R(1);
				LED_G(0);
				LED_B(1);
				break;
			case 2:
				LED_R(1);
				LED_G(1);
				LED_B(0);
				break;
			case 3:
				LED_R(0);
				LED_G(0);
				LED_B(1);
				break;
			case 4:
				LED_R(0);
				LED_G(1);
				LED_B(0);
				break;
			case 5:
				LED_R(1);
				LED_G(0);
				LED_B(0);
				break;
			case 6:
				LED_R(0);
				LED_G(0);
				LED_B(0);
				break;
			default:
				break;
		}
		color++;
	
		t++;
		sprintf(str,"[t]%4d",t);
		LCD_ShowString(0, 15, 240, 12, 12, str);
		sprintf(str,"[rx2_buf]%d",(u8)gear);
		LCD_ShowString(0, 30, 240, 12, 12, str);
		
//		MPU_Get_Gyroscope(&x,&y,&z);
//		HAL_Delay(100);
//		sprintf(str,"x=%8x,y=%8x,z=%8x",x,y,z);
//		LCD_Fill(0,60,239,74,BLACK);
//		LCD_ShowString(0, 60, 240, 12, 12, str);
//		sprintf(str,"x=%f,y=%f,z=%f",x*2.0f/32768,y*2.0f/32768,z*2.0f/32768);
//		LCD_Fill(0,75,239,89,BLACK);
//		LCD_ShowString(0, 75, 240, 12, 12, str);
//		temp=MPU_Get_Temperature();
//		HAL_Delay(100);
//		sprintf(str,"[temp]%f",temp);
//		LCD_Fill(0,90,239,104,BLACK);
//		LCD_ShowString(0, 90, 240, 12, 12, str);
		
		MPU_Get_Accelerometer(&ax,&ay,&az);
		HAL_Delay(100);
		sprintf(str,"[original]ax=%d,ay=%d,az=%d",ax,ay,az);
		LCD_Fill(0,45,239,59,BLACK);
		LCD_ShowString(0,45, 240, 12, 12, str);
		sprintf(str,"[stress]x=%.2fg,y=%.2fg,z=%.2fg",(ax-X_ACCEL_OFFSET)*2.0f/32768,(ay-Y_ACCEL_OFFSET)*2.0f/32768,(az-Z_ACCEL_OFFSET)*2.0f/32768);
		LCD_Fill(0,60,239,74,BLACK);
		LCD_ShowString(0, 60, 240, 12, 12, str);
		temp_z_angle=(az-Z_ACCEL_OFFSET)*2.0f/32768;
		if(temp_z_angle>1)
			temp_z_angle=1;
		sprintf(str,"[angle]x=%.2f,y=%.2f,z=%.2f",acos((ax-X_ACCEL_OFFSET)*2.0f/32768)*57.29577,acos((ay-Y_ACCEL_OFFSET)*2.0f/32768)*57.29577,acos(temp_z_angle)*57.29577);
		LCD_Fill(0,75,239,89,BLACK);
		LCD_ShowString(0, 75, 240, 12, 12, str);
		sprintf(str,"[test]a=%f",acos(1));
		LCD_ShowString(0, 90, 240, 12, 12, str);
		if(gear==0x00)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18479);	//停止
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18479);	//停止
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,18479);	//停止
		}
		else if(gear==0x04)//直行
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18649);	//3档，速度合适，可以稍微加大
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18649);	//3档，速度合适，可以稍微加大
		}
		else if(gear==0x10)//左转
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18559);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18699);
		}
		else if(gear==0x20)//右转
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18699);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18559);
		}
		else if(gear==0x14)//左转幅度小一点
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18559);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18659);
		}
		else if(gear==0x24)//右转幅度小一点
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18659);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,18559);
		}
//		HAL_Delay(500);						
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
	* @brief          串口接收中断回调函数
  * @param[in]     	huart 串口序号
  * @retval         none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance==USART2)
//	{
	if(huart==&huart2)
	{
		gear=rx2_buf;
		HAL_UART_Receive_IT(&huart2, &rx2_buf, 1);	
		
	}
	

}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

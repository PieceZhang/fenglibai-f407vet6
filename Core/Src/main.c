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

#define LED_ON() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define LED2_ON() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_TOGGLE() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MotorSet(int16_t angle1_pwm, int16_t angle2_pwm);
int16_t PID_angle1(float angle1_cur, float angle1_set);
int16_t PID_angle2(float angle2_cur, float angle2_set);
void _motorset(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ABS(x)		((x>0)? x: -x) 

uint8_t MPUflag=0;
uint8_t MPUBuffer[22];
float MPU_a[3]={0};
float MPU_angle[3]={0};

int16_t angle1_motor=0, angle2_motor=0;

volatile float angle1_p=0, angle1_i=0, angle1_d=340;
volatile float angle2_p=0, angle2_i=0, angle2_d=340;
volatile float angle1_lasterror = 0, angle2_lasterror = 0;
volatile float angle1_sumerror = 0, angle2_sumerror = 0;
volatile float angle1_deadband = 0.1, angle2_deadband = 0.1;

volatile uint8_t mode=0;
float angle1_set=0, angle2_set=0;
volatile uint32_t counter1=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	const uint8_t SendBuffer[4]={0x03,0xFC,0xFC,0x03};  // 帧头
	
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

/* 	USART1: MPU9250 115200 DMA
		USART2: BLE 115200 DMA
		USART3: PC 256000 */
	
	My_UART_DMA_Init();
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);  // 上电先拉低油门
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
	HAL_Delay(2000);	
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	const float T = 1850.4;  // 单摆周期 (ms)  T =2pi*sqrt(L / g)
	mode = 5;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		LED2_OFF();
		
		if(mode==0)
		{
			if(MPUflag)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
				MPUflag = 0;
				/* 虚拟示波器 */
//				HAL_UART_Transmit(&huart3, (uint8_t *)&SendBuffer[0], 1, 1);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&SendBuffer[1], 1, 1);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&MPU_angle, sizeof(float)*2, 1);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&SendBuffer[2], 1, 1);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&SendBuffer[3], 1, 1);
			}
		}
		else if(mode==1)
		{
			uint8_t status=0;
			angle1_p=0, angle1_i=0, angle1_d=700, angle1_deadband=0.1;
			angle2_p=5.5, angle2_i=1, angle2_d=350, angle2_deadband=0.2;
			angle2_set = 12;
			counter1 = 0;
			while(1)  // counter1 <= 375
			{
				if(MPUflag)
				{
					angle1_motor = -PID_angle1(MPU_angle[0], 0);
					if(status==0)
					{
						angle2_motor = -PID_angle2(MPU_angle[1], angle2_set);
						if(MPU_angle[1] > 0 && ABS(MPU_angle[1]-angle2_set) < angle2_deadband) status=1;
					}
					else if(status==1)
					{
						angle2_motor = -PID_angle2(MPU_angle[1], -angle2_set);
						if(MPU_angle[1] < 0 && ABS(MPU_angle[1]-angle2_set) < angle2_deadband) status=0;
					}
					MotorSet(angle1_motor, angle2_motor);
					MPUflag = 0;
				}
			}
		}
		else if(mode==2)
		{
			uint8_t status=0;
			uint8_t length=20;
			/*30cm: 7deg  40cm: 10deg  50cm: 12deg  60cm: 14deg*/
			angle2_set = 90 - atan(100/length)*180/3.1415926;
			angle1_p=0, angle1_i=0, angle1_d=700, angle1_deadband=0.1;
			angle2_p=5.5, angle2_i=1, angle2_d=350, angle2_deadband=0.2;
			counter1 = 0;
			while(1)  // counter1 <= 375
			{
				if(MPUflag)
				{
					angle1_motor = -PID_angle1(MPU_angle[0], 0);
					if(status==0)
					{
						angle2_motor = -PID_angle2(MPU_angle[1], angle2_set);
						if(MPU_angle[1] > 0 && ABS(MPU_angle[1]-angle2_set) < angle2_deadband) status=1;
					}
					else if(status==1)
					{
						angle2_motor = -PID_angle2(MPU_angle[1], -angle2_set);
						if(MPU_angle[1] < 0 && ABS(MPU_angle[1]-angle2_set) < angle2_deadband) status=0;
					}
					MotorSet(angle1_motor, angle2_motor);
					MPUflag = 0;
				}
			}
		}
		else if(mode==3)
		{
		
		}
		else if(mode==4)
		{
			if(MPUflag)
			{
				angle1_p=0, angle1_i=0, angle1_d=500, angle1_deadband=0.2;
				angle2_p=0, angle2_i=0, angle2_d=500, angle1_deadband=0.2;
				angle1_motor = -PID_angle1(MPU_angle[0], 0);
				angle2_motor = -PID_angle2(MPU_angle[1], 0);
				MotorSet(angle1_motor, angle2_motor);
				MPUflag = 0;
			}
		}
		else if(mode==5)
		{
			uint32_t cnt=0;
			uint8_t RoundDir = 0;
			float R=15;  // 地面划线长度
			float norm=0, omega, phase, A;
			angle1_p=6.5, angle1_i=1, angle1_d=420, angle1_deadband=0.1;  // 6 2
			angle2_p=6.5, angle2_i=1, angle2_d=420, angle2_deadband=0.1;
			while(1)
			{
				if(MPUflag)
				{
					cnt+=5;
					norm = cnt/T;
					omega = 2.0f*3.1415926f*norm;
					A = atan(R/100)*180.0f/3.1415926f;
					if(RoundDir == 0) phase=3.1415926f/2.0f;  // 顺时针
					else if(RoundDir == 1) phase=3.1415926*3/2;  // 逆时针
					angle1_set = A*sin(omega+0.1f);
					angle2_set = A*sin(omega+phase);
					angle1_motor = -PID_angle1(MPU_angle[0], angle1_set);
					angle2_motor = -PID_angle2(MPU_angle[1], angle2_set);
					MotorSet(angle1_motor, angle2_motor);
					MPUflag = 0;
				}
			}
		}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define ANGLE1_I_MAX 3
#define ANGLE2_I_MAX 3

int16_t PID_angle1(float angle1_cur, float angle1_set)
{
	/* input:   angle1(MPU)
	 * output:  PWM */
	float iError, dError;
	static float out;
	iError = angle1_cur - angle1_set;
	angle1_sumerror += iError;
	if(angle1_sumerror > ANGLE1_I_MAX)
			angle1_sumerror = ANGLE1_I_MAX;
	else if(angle1_sumerror < -ANGLE1_I_MAX)
			angle1_sumerror = -ANGLE1_I_MAX;
	dError = iError - angle1_lasterror;
	angle1_lasterror = iError;
	if(ABS(iError) > angle1_deadband)
		out = (int16_t)(angle1_p * iError + angle1_i * angle1_sumerror + angle1_d * dError);
	return out;
}
int16_t PID_angle2(float angle2_cur, float angle2_set)
{
	/* input:   angle2(MPU)
	 * output:  PWM */
	float iError, dError;
	static float out;
	iError = angle2_cur - angle2_set;
	angle2_sumerror += iError;
	if(angle2_sumerror > ANGLE2_I_MAX)
			angle2_sumerror = ANGLE2_I_MAX;
	else if(angle2_sumerror < -ANGLE2_I_MAX)
			angle2_sumerror = -ANGLE2_I_MAX;
	dError = iError - angle2_lasterror;
	angle2_lasterror = iError;
	if(ABS(iError) > angle2_deadband)
		out = (int16_t)(angle2_p * iError + angle2_i * angle2_sumerror + angle2_d * dError);
	return out;
}

#define ANGLE1_MAX 80
#define ANGLE2_MAX 80
#define PWM_INIT 1060
#define PWM1_MAX PWM_INIT+ANGLE2_MAX
#define PWM2_MAX PWM_INIT+ANGLE1_MAX
#define PWM3_MAX PWM_INIT+ANGLE2_MAX
#define PWM4_MAX PWM_INIT+ANGLE1_MAX
void _motorset(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
	if(m1 > PWM1_MAX) m1=PWM1_MAX;
	else if(m1 < PWM_INIT) m1=PWM_INIT;
	if(m2 > PWM2_MAX) m2=PWM2_MAX;
	else if(m2 < PWM_INIT) m2=PWM_INIT;
	if(m3 > PWM3_MAX) m3=PWM3_MAX;
	else if(m3 < PWM_INIT) m3=PWM_INIT;
	if(m4 > PWM4_MAX) m4=PWM4_MAX;
	else if(m4 < PWM_INIT) m4=PWM_INIT;
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4);	
}

void MotorSet(int16_t _angle1_motor, int16_t _angle2_motor)
{
	uint16_t m1, m2, m3, m4;
	
	if(_angle1_motor>ANGLE1_MAX) _angle1_motor=ANGLE1_MAX;
	else if(_angle1_motor<-ANGLE1_MAX) _angle1_motor=-ANGLE1_MAX;
	if(_angle2_motor>ANGLE2_MAX) _angle2_motor=ANGLE2_MAX;
	else if(_angle2_motor<-ANGLE2_MAX) _angle2_motor=-ANGLE2_MAX;
	
	m1=PWM_INIT-_angle2_motor, m3=PWM_INIT+_angle2_motor;
	m2=PWM_INIT-_angle1_motor, m4=PWM_INIT+_angle1_motor;
	_motorset(m1, m2, m3, m4);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t i=0, counter500ms=0, status=0;
	if (htim == (&htim2))  // 40ms
	{
		i++;
		if(i>=25)
		{
			LED_TOGGLE();
			i=0;
		}
		counter500ms++;
		if(counter500ms>=7)
		{
			char buf[40]={'0'};
			if(status==0)
			{
				sprintf(buf, "Angle1:%+6.2f\r\n", MPU_angle[0]);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&buf[0], 15);
				status = 1;
			}
			else if(status==1)
			{
				sprintf(buf, "Angle2:%+6.2f\r\n", MPU_angle[1]);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&buf[0], 15);
				status = 0;
			}
			sprintf(buf, "Angle2:%+6.2f\r\n", MPU_angle[1]);
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&buf[0], 15);
			counter500ms = 0;
		}
//		counter1++;
	}
}

void USART1_IRQHandler(void)
{
	uint8_t errorcounter;

  HAL_UART_IRQHandler(&huart1);
	My_UART_RESET_RX_CALLBACK(&huart1);
	
	if(MPUBuffer[21]!=0)
	{
		errorcounter=0;
		if(MPUBuffer[10]==(uint8_t)(MPUBuffer[0]+MPUBuffer[1]+MPUBuffer[2]+MPUBuffer[3]+MPUBuffer[4]+
										MPUBuffer[5]+MPUBuffer[6]+MPUBuffer[7]+MPUBuffer[8]+MPUBuffer[9])&&MPUBuffer[0]!=0&&MPUBuffer[1]==0x51)
		{
			MPU_a[0] = ((short)(MPUBuffer[3]<<8 | MPUBuffer[2]))/32768.0f*16*9.8f;     
			MPU_a[1] = ((short)(MPUBuffer[5]<<8 | MPUBuffer[4]))/32768.0f*16*9.8f;      
			MPU_a[2] = ((short)(MPUBuffer[7]<<8 | MPUBuffer[6]))/32768.0f*16*9.8f;  
		}
		else errorcounter++;
		if(MPUBuffer[10+11]==(uint8_t)(MPUBuffer[0+11]+MPUBuffer[1+11]+MPUBuffer[2+11]+MPUBuffer[3+11]+MPUBuffer[4+11]+
										MPUBuffer[5+11]+MPUBuffer[6+11]+MPUBuffer[7+11]+MPUBuffer[8+11]+MPUBuffer[9+11])&&MPUBuffer[0+11]!=0&&MPUBuffer[1+11]==0x53)
		{
			MPU_angle[0] = ((short)(MPUBuffer[3+11]<<8| MPUBuffer[2+11]))/32768.0f*180; 
			MPU_angle[1] = ((short)(MPUBuffer[5+11]<<8| MPUBuffer[4+11]))/32768.0f*180;   	
			MPU_angle[2] = ((short)(MPUBuffer[7+11]<<8| MPUBuffer[6+11]))/32768.0f*180;
			MPU_angle[0] += 1.6f;  // 修正安装误差
		}
		else errorcounter++;
		
		if(errorcounter>1) //纠错
		{
			__HAL_DMA_DISABLE(huart1.hdmarx);  //关闭通道
			huart1.hdmarx->Instance->NDTR=sizeof(MPUBuffer); //写寄存器 CNDTR：DMA传输剩余字节数（当通道开启时只读）
			__HAL_DMA_ENABLE(huart1.hdmarx);   //开启通道
		}
		else  // 无错
			MPUflag = 1, LED2_ON();
	}
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
//    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
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

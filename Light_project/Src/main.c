
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

//Defines
#define umbral_luz 200

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//Variables

int i;
volatile uint32_t ADC_buffer[4]; // Buffer que guarda los valores analógicos que se leen desde la DMA
int Intensidad = 0; // Valor del duty cycle que representará la intensidad a la que ilumina el LED seleccionado
bool R, L, U, D; // boolean variables para definir posición del joystick
volatile uint32_t Joystick_V, Joystick_H, LDR, Potentiometer; // Valores analógicos de los ejes X e Y del Joystic respectivamente y de la LDR y el potenciómetro
uint32_t num; // variable para representar en el display
int Red_light, Green_light, Blue_light;
volatile bool flag = 0;
//Funciones

void map ()// funcion encargada de traducir el valor analógico leído del Joystick a valores de dcha, izq, arriba y abajo o lo que se precise
{
	R = L = U = D = 0;
	if (ADC_buffer[0] < 800)
		R = 1;
	if (ADC_buffer[0] > 1200)
		L = 1;
	if (ADC_buffer[1] < 800)
		D = 1;
	if (ADC_buffer[1] > 1200)
		U = 1;
}

void Display_control (uint32_t P) // Decodifica el valor del potenciómetro y lo representa en el display
{
		num = P%10; //asocia al valor de la lectura analógica un número entre 0 y 97
//		switch (num)
//		{
//			case 0:		// Display = 0xFC; //Respresenta el número 0 en el display ("0b11111100")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);								
//				break;				
//			}
//			case 1:		// Display = 0x60; //Respresenta el número 0 en el display ("0b01100000")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);	
//				break;				
//			}
//			case 2:		// Display = 0xDA; //Respresenta el número 0 en el display ("0b11011010")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);	
//				break;				
//			}
//			case 3:		// Display = 0xF2; //Respresenta el número 0 en el display ("0b11110010")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);	
//				break;				
//			}
//			case 4:		// Display = 0x66; //Respresenta el número 0 en el display ("0b01100110")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_RESET);	
//				break;				
//			}
//			case 5:		// Display = 0xB6; //Respresenta el número 0 en el display ("0b10110110")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_RESET);	
//				break;				
//			}
//			case 6:		// Display = 0xBE; //Respresenta el número 0 en el display ("0b10111110")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_RESET);	
//				break;				
//			}
//			case 7:		// Display = 0xE0; //Respresenta el número 0 en el display ("0b11100000")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);	
//				break;				
//			}
//			case 8:		// Display = 0xFE; //Respresenta el número 0 en el display ("0b11111110")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_RESET);	
//								break;				
//			}
//			case 9:		// Display = 0xE6; //Respresenta el número 0 en el display ("0b11100110")
//			{
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_RESET);	
//								break;
//			}								
//		}
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  GPIO_PIN_SET);	
}

void enable_lights(bool EN)
{
		if (EN)
		{
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Intensidad);
		}
		else
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void RGB_Control ()
{
				map();// mapea el valor del joystick
				if(U)
				{
						Red_light = 100 - (Joystick_V%100);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Red_light);
				}
				if(D)
				{
						Green_light = Joystick_V%100;
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Green_light);
				}
				if(R)
				{
						Blue_light = Joystick_H- (Joystick_V%100);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Blue_light);
				}
				if(L)
				{
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
				}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
			flag = !flag;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
			Joystick_V = ADC_buffer [0];
			Joystick_H = ADC_buffer [1];
			LDR = ADC_buffer [2];
			Potentiometer = ADC_buffer [3];
	
			Intensidad = ((Potentiometer)/100)%100;
							if(LDR < umbral_luz)
				{
						enable_lights(false); // en caso de que haya más luz de la esperada apaga las luces
				}
				else
				{
						enable_lights(true); // en caso de que haya más luz de la esperada enciende las luces
				}
				
				// Display_control(Intensidad); // le pasa el valor del potenciómetro a la funciçon para poder decodificar el valor y representarlo en el display
				
				if(flag == 0)
				{
						RGB_Control();
				}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	// Inicialización de los canales del temporizador
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_ADC_Start_DMA (&hadc1, ADC_buffer, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, c_Pin|d_Pin|e_Pin|f_Pin 
                          |g_Pin|a_Pin|b_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : c_Pin d_Pin e_Pin f_Pin 
                           g_Pin a_Pin b_Pin */
  GPIO_InitStruct.Pin = c_Pin|d_Pin|e_Pin|f_Pin 
                          |g_Pin|a_Pin|b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_setter_Pin */
  GPIO_InitStruct.Pin = RGB_setter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RGB_setter_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define tV_25   1.34f	// Напряжение (в вольтах) на датчике при температуре 25 °C.
#define tSlope  0.0043f	// изменение напряжения (в вольтах) при изменении температуры на градус.
#define Vref    3.3f	// Образцовое напряжение АЦП (в вольтах).

#define ADC_SAMPLES 50	// Кол-во отсчётов которые нужно накопить для усреднения
#define CHANNELS_CNT 6	// Кол-во кактивных аналов АЦП участвующих в получении значений

#define UART_MS_TIMEOUT 1000	// Период в мс выдачи информации по UART1

typedef enum {	// Для хранения состояния кнопок BTN1, BTN2
	NO_PRESS,
	PRESS,
	RELEASE
} BTN_STATE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Буфер для получения определённого кол-ва значений со всех активных каналов АЦП
// (*2) для того чтобы можно было использовать прерывания по половине заполнения буфера и по целому буферу
uint16_t adc_buffer[ADC_SAMPLES * CHANNELS_CNT * 2] = {0};

// Для хранения усреднённых значений АЦП по 5 каналам и температурному датчику
float adc1_v, adc2_v, adc3_v, adc4_v, adc5_v;
float adc_temperature;
int ten_temperature = 0;	// Заглушка с температурой (по заданию)

uint8_t but1_shift_reg = 0, but2_shift_reg = 0;	// Сдвиговые регистры для отслеживания нажатий\отпусканий кнопок
BTN_STATE btn1_state = NO_PRESS, btn2_state = NO_PRESS;	// Текущее состояние кнопок

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Усреднение значений АЦП из половины кольцевого буфера
void averaging_adc_values(uint16_t *buffer) {

    uint32_t sum[6] = {0,0,0,0,0,0};	// Для целочисленных сумм значений соответствующих каналов

    for (int i = 0; i < ADC_SAMPLES; i++) {
    	sum[0] += buffer[i * CHANNELS_CNT];	// Суммируем значения полученные от канала за все отсчёты
    	sum[1] += buffer[1 + i * CHANNELS_CNT];
    	sum[2] += buffer[2 + i * CHANNELS_CNT];
    	sum[3] += buffer[3 + i * CHANNELS_CNT];
    	sum[4] += buffer[4 + i * CHANNELS_CNT];
    	sum[5] += buffer[5 + i * CHANNELS_CNT];
    }

    adc1_v = (float)((sum[0] / ADC_SAMPLES)*(Vref / 0xFFF));	// Для получения значений в диапазоне 0В - 3.3В
    adc2_v = (float)((sum[1] / ADC_SAMPLES)*(Vref / 0xFFF));
    adc3_v = (float)((sum[2] / ADC_SAMPLES)*(Vref / 0xFFF));
    adc4_v = (float)((sum[3] / ADC_SAMPLES)*(Vref / 0xFFF));
    adc5_v = (float)((sum[4] / ADC_SAMPLES)*(Vref / 0xFFF));

    adc_temperature = (float)((sum[5] / ADC_SAMPLES)*(Vref / 0xFFF));	// Для получения значения напряжения на температурном датчике
    adc_temperature = (tV_25 - adc_temperature)/tSlope + 25;   			// Температура в градусах (формула пересчёта найдена на форумах)
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t str_size;	// Для подсчёта текущей	 длины строки для вывода на LCD1602
	int size;	// Для подсчёта текущей	 длины строки UART
	uint32_t now = 0, then = 0;	// Для отсчёта интервалов времени
	char buf[100];	// буфер для UART-строки с запасом по кол-ву символов
	char lcd_str[16];	// строка для отображения на LCD1602

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	LCD_init();


	LCD_SetPos(0, 0);	// Перевод курсора на 0 символ 0 строки (отсчёт с нуля)
	sprintf(lcd_str, "%3ddg %2.1fv", 23, 1.2);
	str_size = strlen(lcd_str);
	if (str_size < 16)
		lcd_str[str_size] = 0;
	LCD_String(lcd_str);

	LCD_SetPos(0, 1);	// Перевод курсора на 0 символ 1 строки
	sprintf(lcd_str, "%3ddg %2.1fv", 45, 7.8);
	str_size = strlen(lcd_str);
	if (str_size < 16)
		lcd_str[str_size] = 0;
	LCD_String(lcd_str);


	// Разрешён ScanConversionMode для того чтобы можно было опрашивать несколько каналов АЦП подряд
	// В качестве триггера для запуска АЦП выступают прерывания от Таймера3
	HAL_ADCEx_Calibration_Start(&hadc1);	// Калибровка
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_SAMPLES * CHANNELS_CNT * 2);	// Запуск АЦП с сохранением отсчётов через DMA в кольцевой буфер
	HAL_TIM_Base_Start_IT(&htim1);	// Таймер для отсчёта 5мс (опрос кнопок BUT1, BUT2 в прерывании)
	HAL_TIM_Base_Start_IT(&htim3);	// Таймер для отсчёта 100мкс(10КГц) для запуска АЦП

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Асинхронный вывод нажатий\отпусканий кнопок
	if (btn1_state != NO_PRESS)
	{
		if (btn1_state == PRESS)
			sprintf(buf, "BUT1 pressed\n");
		else
			sprintf(buf, "BUT1 released\n");
		size = strlen(buf);
		btn1_state = NO_PRESS;
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, size, 100);
	}

	if (btn2_state != NO_PRESS)
	{
		if (btn2_state == PRESS)
			sprintf(buf, "BUT2 pressed\n");
		else
			sprintf(buf, "BUT2 released\n");
		size = strlen(buf);
		btn2_state = NO_PRESS;
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, size, 100);
	}

	// Получаем кол-во прошедших со старта мс
	now = HAL_GetTick();

	// Вывод усреднённых значений по UART каждую секунду
	// >= используется на случай если вывод по UART состояния кнопок займёт лишние мс
	if ((now - then) >= UART_MS_TIMEOUT)
	{
		then = now;

		sprintf(buf, "ts = %2ddg, tt = %1ddg, u1 = %2.1fV, u2 = %2.1fV, u3 = %2.1fV, u4 = %2.1fV, u5 = %2.1fV\n",
						(int)adc_temperature, ten_temperature, adc1_v, adc2_v, adc3_v, adc4_v, adc5_v);
		size = strlen(buf);
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, size, 50);

		// Отладочная мигалка
		HAL_GPIO_TogglePin(led_pc13_GPIO_Port, led_pc13_Pin);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_pc13_GPIO_Port, led_pc13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led_pb0_Pin|led_pb1_Pin|led_pb12_Pin|D4_Pin
                          |D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_pc13_Pin */
  GPIO_InitStruct.Pin = led_pc13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_pc13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_pb0_Pin led_pb1_Pin led_pb12_Pin D7_Pin */
  GPIO_InitStruct.Pin = led_pb0_Pin|led_pb1_Pin|led_pb12_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT1_Pin BUT2_Pin */
  GPIO_InitStruct.Pin = BUT1_Pin|BUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Колбэк прерываний от Таймеров
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Предназначен для обработки кнопок PB14(BUT1), PB15(BUT2)
	if (htim->Instance == TIM1)
	{
		HAL_GPIO_TogglePin(led_pb0_GPIO_Port, led_pb0_Pin);	// Отладочная мигалка

		// Для BUT1
		but1_shift_reg <<= 1;	// Сдвигаем регистр на 1 разряд влево
		if (HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin) == GPIO_PIN_RESET)	// Опрашиваем пин к которому подключена кнопка1
			but1_shift_reg |= 1;	// Если в этот момент кнопка нажата - задвигаем в младший разряд единичку

		if (but1_shift_reg == 0x0F)
			btn1_state = PRESS;	// При такой комбинации в сдвиговом регистре фиксируется нажатие на кнопку
		else if (but1_shift_reg == 0xF0)
			btn1_state = RELEASE;	// При такой комбинации в сдвиговом регистре фиксируется отпускание кнопки

		// Для BUT2
		but2_shift_reg <<= 1;
		if (HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin) == GPIO_PIN_RESET)	// Опрашиваем пин к которому подключена кнопка2
			but2_shift_reg |= 1;

		if (but2_shift_reg == 0x0F)
			btn2_state = PRESS;
		else if (but2_shift_reg == 0xF0)
			btn2_state = RELEASE;
	}

	// Отслеживается момент запуска преобразований на АЦП
	else if (htim->Instance == TIM3)
		HAL_GPIO_TogglePin(led_pb1_GPIO_Port,  led_pb1_Pin);
}


// Прерывания по половине заполнения кольцевого буфера DMA значениями АЦП
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
    	HAL_GPIO_TogglePin(led_pb12_GPIO_Port,  led_pb12_Pin);
    	averaging_adc_values(&adc_buffer[0]);	// Усреднеине значений из первой половины буфера
    }
}


// Прерывания по полному заполнению кольцевого буфера DMA значениями АЦП
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
    	HAL_GPIO_TogglePin(led_pb12_GPIO_Port,  led_pb12_Pin);
    	averaging_adc_values(&adc_buffer[ADC_SAMPLES * CHANNELS_CNT]);	// Усреднеине значений из второй половины буфера
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

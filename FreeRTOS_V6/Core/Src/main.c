/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;

/* Definitions for Throttle */
osThreadId_t ThrottleHandle;
const osThreadAttr_t Throttle_attributes = {
  .name = "Throttle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Brake */
osThreadId_t BrakeHandle;
const osThreadAttr_t Brake_attributes = {
  .name = "Brake",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Gear */
osThreadId_t GearHandle;
const osThreadAttr_t Gear_attributes = {
  .name = "Gear",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Data */
osThreadId_t DataHandle;
const osThreadAttr_t Data_attributes = {
  .name = "Data",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ThrottleQueue */
osMessageQueueId_t ThrottleQueueHandle;
const osMessageQueueAttr_t ThrottleQueue_attributes = {
  .name = "ThrottleQueue"
};
/* Definitions for BrakeQueue */
osMessageQueueId_t BrakeQueueHandle;
const osMessageQueueAttr_t BrakeQueue_attributes = {
  .name = "BrakeQueue"
};
/* Definitions for GearQueue */
osMessageQueueId_t GearQueueHandle;
const osMessageQueueAttr_t GearQueue_attributes = {
  .name = "GearQueue"
};
/* Definitions for SpeedQueue */
osMessageQueueId_t SpeedQueueHandle;
const osMessageQueueAttr_t SpeedQueue_attributes = {
  .name = "SpeedQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
void ThrottleInput(void *argument);
void BrakeInput(void *argument);
void GearSelection(void *argument);
void DataProcessing(void *argument);
void OutputDisplay(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t throttle;
uint16_t brake;
uint16_t throttleMap;
uint16_t brakeMap;
char gear = 'N';

int brakeMapReceived = 0;
int throttleMapReceived = 0;
int gearReceived = 0;
int DesiredSpeedReceived = 0;

int DesiredSpeed = 0;
int MapDesiredSpeed = 0;
int AverageBrakePercentage = 0;
int AverageThrottlePercentage = 0;
int AverageBrakePercentageTotal = 0;
int AverageThrottlePercentageTotal = 0;
int AverageSpeed = 0;
int AverageSpeedTotal = 0;
int finalSpeed = 0;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
HAL_ADC_Start(&hadc2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ThrottleQueue */
  ThrottleQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &ThrottleQueue_attributes);

  /* creation of BrakeQueue */
  BrakeQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &BrakeQueue_attributes);

  /* creation of GearQueue */
  GearQueueHandle = osMessageQueueNew (16, sizeof(char), &GearQueue_attributes);

  /* creation of SpeedQueue */
  SpeedQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &SpeedQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Throttle */
  ThrottleHandle = osThreadNew(ThrottleInput, NULL, &Throttle_attributes);

  /* creation of Brake */
  BrakeHandle = osThreadNew(BrakeInput, NULL, &Brake_attributes);

  /* creation of Gear */
  GearHandle = osThreadNew(GearSelection, NULL, &Gear_attributes);

  /* creation of Data */
  DataHandle = osThreadNew(DataProcessing, NULL, &Data_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(OutputDisplay, NULL, &Display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ThrottleInput */
/**
  * @brief  Function implementing the Throttle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ThrottleInput */
void ThrottleInput(void *argument)

{  /* USER CODE BEGIN 5 */
	uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
		{
		    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
		}

	//print on LCD for debugging
/*
	Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
	Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
	Lcd_HandleTypeDef lcd;
	lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);
*/
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_PollForConversion(&hadc1,1000);
	  throttle = HAL_ADC_GetValue(&hadc1);
	  throttleMap = MAP(throttle, 0,4020,0,100);
	  if (throttleMap < 15)
	  {
		  throttleMap = 0;
	  }
	  //print on LCD for debugging
/*
	  Lcd_cursor(&lcd, 0,0);
	  Lcd_string(&lcd, "throttle value");
	  Lcd_cursor(&lcd, 1,0);
	  Lcd_int(&lcd,throttleMap);
*/
	  osMessageQueuePut(ThrottleQueueHandle, &throttleMap,0,200);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_BrakeInput */
/**
* @brief Function implementing the Brake thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BrakeInput */
void BrakeInput(void *argument)
{
	 /* USER CODE BEGIN BrakeInput */
		uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
		{
		    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
		}
		 //print on LCD for debugging
/*
		Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
		Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
		Lcd_HandleTypeDef lcd;
		lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);
*/
	  /* Infinite loop */
	  for(;;)
	  {
		  HAL_ADC_PollForConversion(&hadc2,1000);
		  brake = HAL_ADC_GetValue(&hadc2);
		  brakeMap = MAP(brake, 0,4020,0,100);
		  if (brakeMap < 15)
		  	  {
		  		  brakeMap = 0;
		  	  }
/*
		  //print on LCD for debugging
		  Lcd_cursor(&lcd, 0,0);
		  Lcd_string(&lcd, "brake 2 value");
		  Lcd_cursor(&lcd, 1,0);
		  Lcd_int(&lcd,brakeMap);
*/

		  osMessageQueuePut(BrakeQueueHandle, &brakeMap,0,200);
	    osDelay(1);
	  }
	  /* USER CODE END BrakeInput */
}

/* USER CODE BEGIN Header_GearSelection */
/**
* @brief Function implementing the Gear thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GearSelection */
void GearSelection(void *argument)
{
	/* USER CODE BEGIN GearSelection */
/*
	Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
	  Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
	  Lcd_HandleTypeDef lcd;
	  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);
*/
	  /* Infinite loop */
	  for(;;)
	  {
		  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)) //if bit0 == 1(which is bit4 in DIP) means car is parked
		 	  {
		 		  gear = 'P';
				  /*Lcd_cursor(&lcd, 1,0);
				  Lcd_string(&lcd, "P");*/
		 	  }
		 	  else //car is moving
		 	  {
		 		  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)) //bit1 means car is moving
		 		  {
		 			  gear = '2';
		 			 /*Lcd_cursor(&lcd, 1,0);
		 			 Lcd_string(&lcd, "2");*/
		 		  }
		 		  else
		 		  {
		 			  gear = '1';
		 			 /*Lcd_cursor(&lcd, 1,0);
		 			 Lcd_string(&lcd, "1");*/
		 		  }
		 	  }
/*
		  Lcd_cursor(&lcd, 0,0);
		  Lcd_string(&lcd, "gear value");
*/

		  osMessageQueuePut(GearQueueHandle,&gear,0,250);
	    osDelay(1);
	  }
	  /* USER CODE END GearSelection */
}

/* USER CODE BEGIN Header_DataProcessing */
/**
* @brief Function implementing the Data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataProcessing */
void DataProcessing(void *argument)
{
	uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
	{
		return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
	}
/*
	  Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
	  Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
	  Lcd_HandleTypeDef lcd;
	  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);
*/

	  for(;;)
	  {
		AverageBrakePercentageTotal = 0;
		AverageThrottlePercentageTotal = 0;
		brakeMapReceived = 0;
		throttleMapReceived = 0;

		osMessageQueueGet(GearQueueHandle,&gearReceived,NULL,250); //get gear

		  for (int i = 0; i < 16; i++) //cuantas iteraciones
		  {
			  osMessageQueueGet(BrakeQueueHandle,&brakeMapReceived,NULL,250); //get brakequeue
			  osMessageQueueGet(ThrottleQueueHandle, &throttleMapReceived,NULL,250); //get throttle
			  AverageBrakePercentageTotal = AverageBrakePercentageTotal + brakeMapReceived;
			  AverageThrottlePercentageTotal = AverageThrottlePercentageTotal + throttleMapReceived;

		  }
		  AverageBrakePercentage = AverageBrakePercentageTotal / 16;
		  AverageThrottlePercentage = AverageThrottlePercentageTotal / 16;
/*
 PRINT LCD FOR DEBUGGING AVERAGE VALUES OF BRAKE AND THROTTLE
		  Lcd_cursor(&lcd, 0,0);
		  Lcd_int(&lcd, AverageBrakePercentage);
		  Lcd_string(&lcd, "   ");
		  Lcd_cursor(&lcd, 1,0);
		  Lcd_int(&lcd, AverageThrottlePercentage);
		  Lcd_string(&lcd, "   ");
*/

		  if (gearReceived == 'P' || gearReceived == 'N')
		  {
			  DesiredSpeed = DesiredSpeed - 400;
			  if (DesiredSpeed <= 0)
			  {
				  DesiredSpeed = 0;
			  }
		  }
		  else if (gearReceived == '1')
		  {
			  if (DesiredSpeed >= 80000)
			  {
				  DesiredSpeed = 80000;
			  }

			  if(AverageBrakePercentage > 0 && AverageThrottlePercentage > 0)
			  {
				  DesiredSpeed = DesiredSpeed;

			  }
			  //ACCELERATION
			  else if(AverageBrakePercentage <= 0 && AverageThrottlePercentage > 0)
			  {
					  MapDesiredSpeed = MAP(AverageThrottlePercentage, 0, 100, 0, 1600);
					  DesiredSpeed = DesiredSpeed + MapDesiredSpeed;
			  }
			  //BRAKING
			  else if(AverageBrakePercentage > 0 && AverageThrottlePercentage <= 0)
			  {
				  if (DesiredSpeed <= 0)
				  {
					  DesiredSpeed = 0;
				  }
				  else if (AverageBrakePercentage > 0)
				  {
					  MapDesiredSpeed = MAP(AverageBrakePercentage, 0, 100, 0, 3200);
					  DesiredSpeed = DesiredSpeed - MapDesiredSpeed;
				  }
			  }
		  }
		  else if (gearReceived == '2')
		  {
			  if (DesiredSpeed >= 200000)
			  {
				  DesiredSpeed = 200000;
			  }

			  if(AverageBrakePercentage > 0 && AverageThrottlePercentage > 0)
			  {
				  DesiredSpeed = DesiredSpeed;
			  }
			  else if(AverageBrakePercentage <= 0 && AverageThrottlePercentage > 0)
			  {
				  MapDesiredSpeed = MAP(AverageThrottlePercentage, 0, 100, 0, 3200);
				  DesiredSpeed = DesiredSpeed + MapDesiredSpeed;
			  }
			  //BRAKING
			  else if(AverageBrakePercentage > 0 && AverageThrottlePercentage <= 0)
			  {
				  if (DesiredSpeed <= 0)
				  {
					  DesiredSpeed = 0;
				  }
				  else if (AverageBrakePercentage > 0)
				  {
					  MapDesiredSpeed = MAP(AverageBrakePercentage, 0, 100, 0, 3200);
					  DesiredSpeed = DesiredSpeed - MapDesiredSpeed;
				  }
			  }
		  }
/*
		  //PRINT LCD FOR DEBUGGING AVERAGE VALUES OF BRAKE AND THROTTLE
		  	  	  Lcd_cursor(&lcd, 0,0);
				  Lcd_string(&lcd, "speed");
				  Lcd_cursor(&lcd, 1,0);
		  		  Lcd_int(&lcd, DesiredSpeed);
		  		  Lcd_string(&lcd, "          ");
*/
		  osMessageQueuePut(SpeedQueueHandle,&DesiredSpeed,0,250);

		  osDelay(1);
	  }
	  /* USER CODE END DataProcessing */
}

/* USER CODE BEGIN Header_OutputDisplay */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputDisplay */
void OutputDisplay(void *argument)
{

		  Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
		  Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
		  Lcd_HandleTypeDef lcd;
		  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);

	  for(;;)
	  {
		  AverageSpeedTotal = 0;
		  for (int i = 0; i < 16; i++)
		  {
			  osMessageQueueGet(SpeedQueueHandle,&DesiredSpeedReceived,NULL,250);
			  AverageSpeedTotal = AverageSpeedTotal + DesiredSpeedReceived;

		  }
		  AverageSpeed = AverageSpeedTotal / 16;
		  finalSpeed = AverageSpeed / 1000;

		  Lcd_cursor(&lcd, 0,0);
		  Lcd_string(&lcd, "final speed ");
		  Lcd_cursor(&lcd, 1,0);
		  Lcd_int(&lcd,finalSpeed);
		  Lcd_string(&lcd, "          ");


		  osDelay(1);
	  }
	  /* USER CODE END OutputDisplay */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

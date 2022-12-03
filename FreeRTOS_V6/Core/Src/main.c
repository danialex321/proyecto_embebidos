#include "main.h"
#include "cmsis_os.h"
#include "lcd.h"

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

// Defining variables
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();

  // Initializing ADC and PWM channel
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  osKernelInitialize();

  /* Create the queue(s) */
  /* creation of ThrottleQueue */
  ThrottleQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &ThrottleQueue_attributes);
  /* creation of BrakeQueue */
  BrakeQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &BrakeQueue_attributes);
  /* creation of GearQueue */
  GearQueueHandle = osMessageQueueNew (16, sizeof(char), &GearQueue_attributes);
  /* creation of SpeedQueue */
  SpeedQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &SpeedQueue_attributes);
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
  /* Start scheduler */
  osKernelStart();
  while (1)
  {
  }
}

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
  ADC_ChannelConfTypeDef sConfig = {0};
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
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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

void ThrottleInput(void *argument)
{
	// Function used for mapping the pot to the range 0-100
	uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
		{
		    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
		}

  for(;;)
  {
	  HAL_ADC_PollForConversion(&hadc1,1000);
	  throttle = HAL_ADC_GetValue(&hadc1);
	  throttleMap = MAP(throttle, 0,4020,0,100);
	  if (throttleMap < 15)
	  {
		  throttleMap = 0;
	  }
	  osMessageQueuePut(ThrottleQueueHandle, &throttleMap,0,200);
    osDelay(10);
  }
}

void BrakeInput(void *argument)
{
	// Function used for mapping the pot to the range 0-100
	uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
		{
		    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
		}
	  for(;;)
	  {
		  HAL_ADC_PollForConversion(&hadc2,1000);
		  brake = HAL_ADC_GetValue(&hadc2);
		  brakeMap = MAP(brake, 0,4020,0,100);
		  if (brakeMap < 15)
		  	  {
		  		  brakeMap = 0;
		  	  }
		  osMessageQueuePut(BrakeQueueHandle, &brakeMap,0,200);
	    osDelay(10);
	  }
}

void GearSelection(void *argument)
{
	  for(;;)
	  {
		  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)) //if bit0 == 1(which is bit4 in DIP) means car is parked
		 	  {
		 		  gear = 'P';
		 	  }
		 	  else //car is moving
		 	  {
		 		  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)) //bit1 means car is moving
		 		  {
		 			  gear = '2';
		 		  }
		 		  else
		 		  {
		 			  gear = '1';
		 		  }
		 	  }
		  osMessageQueuePut(GearQueueHandle,&gear,0,250);
	    osDelay(50);
	  }
}

void DataProcessing(void *argument)
{
	// Function used for mapping the throttle and brake percentages to the according speed increase|decrease
	uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
	{
		return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
	}

	for(;;)
	  {
		AverageBrakePercentageTotal = 0;
		AverageThrottlePercentageTotal = 0;
		brakeMapReceived = 0;
		throttleMapReceived = 0;

		osMessageQueueGet(GearQueueHandle,&gearReceived,NULL,250); //get gear

		  for (int i = 0; i < 16; i++) //getting 16 values from queue
		  {
			  osMessageQueueGet(BrakeQueueHandle,&brakeMapReceived,NULL,250); //get brake
			  osMessageQueueGet(ThrottleQueueHandle, &throttleMapReceived,NULL,250); //get throttle
			  AverageBrakePercentageTotal = AverageBrakePercentageTotal + brakeMapReceived;
			  AverageThrottlePercentageTotal = AverageThrottlePercentageTotal + throttleMapReceived;
		  }
		  AverageBrakePercentage = AverageBrakePercentageTotal / 16;
		  AverageThrottlePercentage = AverageThrottlePercentageTotal / 16;

		  //Conditionals according to gear and input
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
		  osMessageQueuePut(SpeedQueueHandle,&DesiredSpeed,0,250);
		  osDelay(100);
	  }
}

void OutputDisplay(void *argument)
{
	  //LCD Ports config
	  Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
	  Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
	  Lcd_HandleTypeDef lcd;
	  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);

	  // Function used for mapping the speed to the PWM range
	  uint16_t MAP(uint16_t au32_IN, uint16_t au32_INmin, uint16_t au32_INmax, uint16_t au32_OUTmin, uint16_t au32_OUTmax)
		{
			return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
		}

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

		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, MAP(finalSpeed,0,220,0,65535));

		  osDelay(500);
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

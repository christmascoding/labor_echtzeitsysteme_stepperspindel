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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h" // Include the header for TimerCallbackFunction_t
#include "stdio.h"
#include "LibL6474.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// fic
void vStepperPulseTask(void* pvParameters);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
extern void initialise_stdlib_abstraction( void );

void vApplicationMallocFailedHook( void )
{
  taskDISABLE_INTERRUPTS();
  __asm volatile( "bkpt #0" );
  for (;;) {;}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  taskDISABLE_INTERRUPTS();
  __asm volatile( "bkpt #0" );
  for (;;) {;}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --------------------------------------------------------------------------------------------------------------------
static int CapabilityFunc( int argc, char** argv, void* ctx )
// --------------------------------------------------------------------------------------------------------------------
{
	(void)argc;
	(void)argv;
	(void)ctx;
	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\nOK",
	    0, // has spindle
		0, // has spindle status
		0, // has stepper
		0, // has stepper move relative
		0, // has stepper move speed
		0, // has stepper move async
		0, // has stepper status
		0, // has stepper refrun
		0, // has stepper refrun timeout
		0, // has stepper refrun skip
		0, // has stepper refrun stay enabled
		0, // has stepper reset
		0, // has stepper position
		0, // has stepper config
		0, // has stepper config torque
		0, // has stepper config throvercurr
		0, // has stepper config powerena
		0, // has stepper config stepmode
		0, // has stepper config timeoff
		0, // has stepper config timeon
		0, // has stepper config timefast
		0, // has stepper config mmperturn
		0, // has stepper config posmax
		0, // has stepper config posmin
		0, // has stepper config posref
		0, // has stepper config stepsperturn
		0  // has stepper cancel
	);
	return 0;
}

//static int ConsoleWriteStream_ToStdErr(void* pContext, const char* pBuffer, int num)
//{
//	(void)pContext;
//	extern int _write( int file, char* ptr, int len );
//	return _write(2, pBuffer, num);
//}
/* USER CODE END 0 */
// Custom functions for stepper
static void* StepLibraryMalloc(unsigned int size)
{
  return pvPortMalloc(size); // Use FreeRTOS memory allocation
}

static void StepLibraryFree(const void* const pMem)
{
  vPortFree((void*)pMem); // Use FreeRTOS memory deallocation
}
static int StepDriverSpiTransfer(void* pIO, char* pRX, const char* pTX, unsigned int length)
{
  for (unsigned int i = 0; i < length; i++) {
    HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_RESET); // Select the SPI device
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&pTX[i], (uint8_t*)&pRX[i], 1, HAL_MAX_DELAY) != HAL_OK) {
      HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device
      return -1; // Error during SPI transfer
    }
    HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device
    //code for 800ns delay lol

    // Calculate the clock speed in Hz
    uint32_t clockSpeedHz = HAL_RCC_GetSysClockFreq();

    // Calculate the number of NOPs needed for 800ns delay
    // Each NOP takes 1 clock cycle, so calculate cycles for 800ns
    uint32_t nopsNeeded = (clockSpeedHz / 1000000000) * 800;  

    // Perform the NOPs
    for (volatile uint32_t i = 0; i < nopsNeeded + 1; i++) { //+1 for safety (better wait longer!)
      __NOP(); // idle machine
    }
  }
  return 0; // Success
}

  /*(void)pIO; // Unused in this implementation
  //HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_RESET); // Select the SPI device

  if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)pTX, (uint8_t*)pRX, length, HAL_MAX_DELAY) != HAL_OK)
  {
    //HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device
    return -1; // Error du ring SPI transfer
  }

  HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device
  return 0; // Success*/

static void StepDriverReset(void* pGPO, const int ena)
{
  (void)pGPO; // Unused in this implementation
  HAL_GPIO_WritePin(STEP_RSTN_GPIO_Port, STEP_RSTN_Pin, ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StepLibraryDelay(unsigned int ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms)); // Delay using FreeRTOS
}

typedef struct {
  void* pPWM;
  int dir;
  unsigned int numPulses;
  void (*doneClb)(L6474_Handle_t);
  L6474_Handle_t h;
  TaskHandle_t taskHandle; // task handle so it can be cancelled
} StepperTaskArgs_t;

static int StepTimerCancelAsync(void* pPWM)
{
    StepperTaskArgs_t* args = (StepperTaskArgs_t*)pPWM; // Cast pPWM to StepperTaskArgs_t*

    if (args && args->taskHandle) {
        // Suspend the task to ensure it doesn't execute further
        vTaskSuspend(args->taskHandle);

        // Delete the task
        vTaskDelete(args->taskHandle);

        // Free the memory allocated for the task arguments
        vPortFree(args);

        return 0; // Success
    }

    return -1; // Task was not running or invalid arguments
}

void vStepperPulseTask(void* pvParameters) {
    StepperTaskArgs_t* args = (StepperTaskArgs_t*)pvParameters;

    // Store the current task handle in the arguments
    args->taskHandle = xTaskGetCurrentTaskHandle();

    /* Set direction (beispielhaft über pGPO)
    if (args->h.pGPO) {
        // Pseudocode: setDirection(args->h.pGPO, args->dir);
    }*/

    // Pulse loop
    for (unsigned int i = 0; i < args->numPulses; ++i) {
        // Set STEP_PULSE pin high
        HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms High

        // Set STEP_PULSE pin low
        HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms Low
    }

    // Call the done callback
    if (args->doneClb) {
        args->doneClb(args->h);
    }

    // Free memory
    vPortFree(args);

    vTaskDelete(NULL);
}

static int StepTimerAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h) {
    StepperTaskArgs_t* args = pvPortMalloc(sizeof(StepperTaskArgs_t));
    if (!args) return -1;  // malloc failed

    args->pPWM      = pPWM;
    args->dir       = dir;
    args->numPulses = numPulses;
    args->doneClb   = doneClb;
    args->h         = h;
    args->taskHandle = NULL; // Initialize the task handle to NULL

    BaseType_t res = xTaskCreate(
        vStepperPulseTask,
        "StepperTask",
        configMINIMAL_STACK_SIZE + 128,
        args,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    return (res == pdPASS) ? 0 : -1;
}

static int StepSynchronous(void* pPWM, int dir, unsigned int numPulses) {
  (void)pPWM; // Unused in this implementation

  // Set direction pin
  HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // Generate pulses
  for (unsigned int i = 0; i < numPulses; ++i) {
    // Set STEP_PULSE pin high
    HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_SET);
    StepLibraryDelay(1); // 1 ms High

    // Set STEP_PULSE pin low
    HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);
    StepLibraryDelay(1); // 1 ms Low
  }

  return 0; // Success
}

void StepperTask(void *pvParameters)
{
    // Allocate memory for the StepperTaskArgs_t structure
    StepperTaskArgs_t* stepperArgs = pvPortMalloc(sizeof(StepperTaskArgs_t));
    if (!stepperArgs) {
        printf("Failed to allocate memory for StepperTaskArgs_t\r\n");
        Error_Handler();
    }

    // Initialize the StepperTaskArgs_t structure
    stepperArgs->pPWM = NULL; // Placeholder for PWM context, if needed
    stepperArgs->dir = 0;     // Default direction
    stepperArgs->numPulses = 0;
    stepperArgs->doneClb = NULL;
    stepperArgs->h = NULL;
    stepperArgs->taskHandle = NULL;

    // Pass all function pointers required by the stepper library
    // to a separate platform abstraction structure
    L6474x_Platform_t p;
    p.malloc     = StepLibraryMalloc;
    p.free       = StepLibraryFree;
    p.transfer   = StepDriverSpiTransfer;
    p.reset      = StepDriverReset;
    p.sleep      = StepLibraryDelay;
    //p.stepAsync  = StepTimerAsync;
    p.step       = StepSynchronous;
    //p.cancelStep = StepTimerCancelAsync;

    // Now create the handle, passing the stepperArgs as the pPWM parameter
    L6474_Handle_t h = L6474_CreateInstance(&p, NULL, NULL, stepperArgs);

    if (h == NULL) {
        printf("Failed to create L6474 instance\r\n");
        vPortFree(stepperArgs); // Free memory if instance creation fails
        Error_Handler();
    } else {
        printf("Stepper motor instance created\r\n");
    }

    stepperArgs->h = h; // Store the handle in the arguments structure

    int result = 0;

    // Create base parameter structure
    L6474_BaseParameter_t baseParam = {
        .stepMode   = smMICRO8,        // Gute Balance zwischen Auflösung und Drehmoment
        .OcdTh      = ocdth1125mA,     // Ca. 1.125 A als Überstromgrenze
        .TimeOnMin  = 10,              // µs – Beispielwert, ggf. durch Tests optimieren
        .TimeOffMin = 15,              // µs – Beispielwert, ggf. durch Tests optimieren
        .TorqueVal  = 80,              // 80% des max. Drehmoments
        .TFast      = 5                // µs – Schaltzeitoptimierung
    };

    // Set default base parameters
    result |= L6474_SetBaseParameter(&baseParam);

    // Initialize the driver with the base parameters
    result |= L6474_Initialize(h, &baseParam);
    result |= L6474_SetPowerOutputs(h, 1);

    // In case we have no error, we can enable the drivers
    // and then we step a bit
    if (result == 0) {
        result |= L6474_StepIncremental(h, 1000);
        if (result == 0) {
            printf("Stepper motor moved 1000 steps\r\n");
        } else {
            printf("Error during step operation\r\n");
        }
    } else {
        // Error handling
        printf("Error during initialization: %d\r\n", result);
        Error_Handler();
    }

    // Free the memory for stepperArgs after use
    vPortFree(stepperArgs);

    // Delete the task after initialization
    vTaskDelete(NULL);
}


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  // Create the task
  if (xTaskCreate(StepperTask, "StepperTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
      printf("Failed to create StepperTask\r\n");
      Error_Handler();
  }

  printf("Hallo Welt\r\n");
  (void)CapabilityFunc;
  vTaskStartScheduler();
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Period = 4499;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, STEP_RSTN_Pin|STEP_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPINDLE_ENA_L_Pin|SPINDLE_ENA_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USR_BUTTON_Pin */
  GPIO_InitStruct.Pin = USR_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USR_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPINDLE_SI_R_Pin */
  GPIO_InitStruct.Pin = SPINDLE_SI_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPINDLE_SI_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_RSTN_Pin */
  GPIO_InitStruct.Pin = STEP_RSTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_RSTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_DIR_Pin */
  GPIO_InitStruct.Pin = STEP_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STEP_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_FLAG_Pin */
  GPIO_InitStruct.Pin = STEP_FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STEP_FLAG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPINDLE_ENA_L_Pin SPINDLE_ENA_R_Pin */
  GPIO_InitStruct.Pin = SPINDLE_ENA_L_Pin|SPINDLE_ENA_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_SPI_CS_Pin */
  GPIO_InitStruct.Pin = STEP_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STEP_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_PULSE_Pin */
  GPIO_InitStruct.Pin = STEP_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REFERENCE_MARK_Pin LIMIT_SWITCH_Pin */
  GPIO_InitStruct.Pin = REFERENCE_MARK_Pin|LIMIT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPINDLE_SI_L_Pin */
  GPIO_InitStruct.Pin = SPINDLE_SI_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPINDLE_SI_L_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vAssertCalled( const char * const pcFileName, unsigned long ulLine )
{
volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Parameters are not used. */
    ( void ) ulLine;
    ( void ) pcFileName;

    taskENTER_CRITICAL();
    {
        /* You can step out of this function to debug the assertion by using
        the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
        value. */
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
        }
    }
    taskEXIT_CRITICAL();
}

int __stdout_put_char(int ch)
{
	uint8_t val = ch;
	while((huart3.Instance->ISR & UART_FLAG_TXE) == 0);
	huart3.Instance->TDR = val;
	while((huart3.Instance->ISR & UART_FLAG_TC) == 0);
	return 0;
}

int __stdin_get_char(void)
{
	if (huart3.Instance->ISR & UART_FLAG_ORE)
		huart3.Instance->ICR = UART_CLEAR_OREF;

	if (huart3.Instance->ISR & UART_FLAG_NE)
		huart3.Instance->ICR = UART_CLEAR_NEF;

	if (huart3.Instance->ISR & UART_FLAG_FE)
		huart3.Instance->ICR = UART_CLEAR_FEF;

	if ((huart3.Instance->ISR & UART_FLAG_RXNE) == 0) return -1;
	return huart3.Instance->RDR;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
	portENTER_CRITICAL();

	printf("HAL_ASSERT: %s:::%u\r\n", (char*)file, (unsigned int)line);
	assert(0);

	portEXIT_CRITICAL();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

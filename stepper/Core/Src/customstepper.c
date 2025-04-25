#include "customstepper.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stddef.h>  // for NULL


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
    if (HAL_SPI_TransmitReceive(&hspi1, pTX+i, pRX+i,(uint16_t)1, HAL_MAX_DELAY) != HAL_OK) {
      HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device
      return -1; // Error during SPI transfer
    }
    HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET); // Deselect the SPI device

  }
  return 0; // Success
}


static void StepDriverReset(void* pGPO, const int ena)
{
  (void)pGPO; // Unused in this implementation
  HAL_GPIO_WritePin(STEP_RSTN_GPIO_Port, STEP_RSTN_Pin, !ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StepLibraryDelay(unsigned int ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms)); // Delay using FreeRTOS
}

/*typedef struct {
  void* pPWM;
  int dir;
  unsigned int numPulses;
  void (*doneClb)(L6474_Handle_t);
  L6474_Handle_t h;
  TaskHandle_t taskHandle; // task handle so it can be cancelled
} StepperTaskArgs_t;*/

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
    L6474_Handle_t h = L6474_CreateInstance(&p, NULL, NULL, NULL);

    if (h == NULL) {
        printf("Failed to create L6474 instance\r\n");
        vPortFree(stepperArgs); // Free memory if instance creation fails
        Error_Handler();
    } else {
        printf("Stepper motor instance created\r\n");
    }

    //stepperArgs->h = h; // Store the handle in the arguments structure

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
    result |= L6474_ResetStandBy(h);
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

#include "FreeRTOS.h"   
#include "task.h"           
#include "timers.h"        
#include "stdio.h"          
#include "LibL6474.h"       
#include "main.h"    
#include <stddef.h>  // for NULL
#include "customstepper.h"

extern SPI_HandleTypeDef hspi1;
extern L6474_Handle_t stepperHandle; 

extern SPI_HandleTypeDef hspi1;

int stepspermm = 100; //steps/mm for the stepper motor
// Function to calculate the number of steps needed to move a certain distance
static int CalcAbsolute(double position, double currentPosition, int stepsPerMm) {
  double deltaPosition = position - currentPosition; // Calculate the difference in position
  return (int)(deltaPosition * stepsPerMm);         // Convert to steps
}

// Function to calculate the number of steps for a relative move
static int CalcRelative(double relativePosition, int stepsPerMm) {
  return (int)(relativePosition * stepsPerMm);      // Convert relative position to steps
}

//static int ConsoleWriteStream_ToStdErr(void* pContext, const char* pBuffer, int num)
//{
//	(void)pContext;
//	extern int _write( int file, char* ptr, int len );
//	return _write(2, pBuffer, num);
//}
/* USER CODE END 0 */
typedef enum
// --------------------------------------------------------------------------------------------------------------------
{
  cctNONE                = 0x00,
  cctSTART               = 0x01,
  cctSTOP                = 0x02,
  cctMOVE_RELATIVE       = 0x03,
  cctMOVE_ABSOLUTE       = 0x04,
  cctMOVE_WITH_SPEED     = 0x05,
  cctREFERENCE_ENABLE    = 0x06,
  cctREFERENCE_TIMEOUT   = 0x07,
  cctREFERENCE_SKIP      = 0x08,
  cctPOSITION            = 0x09,
  cctCANCEL              = 0x0A,
  cctSTATUS              = 0x0B,
  cctCONFIG_POWER_ENABLE = 0x0C,
  cctCONFIG_PARAMETER    = 0x0D
} CtrlCommandType_t;

typedef struct CtrlCommand
// --------------------------------------------------------------------------------------------------------------------
{
  struct
  {
    int requestID; // Unique request ID
    CtrlCommandType_t type; // Command type
  } head;
  struct
  {
    union
    {
      struct
      {
        double relativePosition; // stepper move <RelPos> -r
      } asMoveRelative;

      struct
      {
        double absolutePosition; // stepper move <AbsPos> -a
      } asMoveAbsolute;

      struct
      {
        double absolutePosition; // stepper move <AbsPos> -s <Speed_in_mm/min>
        float speed;
      } asMoveWithSpeed;

      struct
      {
        int enablePowerOutputs; // stepper reference -e
      } asReferenceEnable;

      struct
      {
        int timeout; // stepper reference -t <time>
      } asReferenceWithTimeout;

      struct
      {
        int skipReference; // stepper reference -s
      } asReferenceSkip;

      struct
      {
        int position; // stepper position
      } asPosition;

      struct
      {
        int cancelOperation; // stepper cancel
      } asCancel;

      struct
      {
        int status; // stepper status
      } asStatus;

      struct
      {
        int powerEnable; // stepper config powerena [-v 0|1]
        int value;
      } asConfigPowerEnable;

      struct
      {
        char parameter[32]; // stepper config <parameter> [-v <value>]
        int value;
      } asConfigParameter;

    } args;
    SemaphoreHandle_t syncEvent; // Synchronization event
  } request;
  StepCommandResponse_t* response; // Response structure
} StepperCtrlCommand_t;


// Stepper Console Function
// --------------------------------------------------------------------------------------------------------------------
static int StepperConsoleFunction(int argc, char** argv, void* ctx)
// --------------------------------------------------------------------------------------------------------------------
{
	//possible commands are
	// stepper move (absolutePos) ->
	// stepper move (relativePos) -r
	// stepper move (absPos) -s (speed in mm/min)
	// stepper reference  -> referenzfahrt
	// stepper reset  -> reset and re-initialize stepper


  StepperTaskArgs_t* args = (StepperTaskArgs_t*)ctx; // Cast ctx to StepperTaskArgs_t*
  L6474_Handle_t h = args->h; // Access the stepper handle
	StepCommandResponse_t response = { 0 };
	StepperCtrlCommand_t cmd;

  //register command like this: CONSOLE_RegisterCommand(consoleHandle, "stepper", "Control the stepper motor", StepperConsoleFunction, stepperArgs);

	cmd.response       = &response;
	cmd.head.requestID = h->nextRequestID;
	h->nextRequestID += 1;
  // First decode the subcommand and all arguments
  if (argc == 0) {
    // No arguments provided
    printf("invalid number of arguments\r\nFAIL");
    response->code = -1;
  }

  // Handle "move" command
  if (strcmp(argv[0], "move") == 0) {
    if (argc < 2) {
        printf("Missing position argument for move command\r\nFAIL\r\n");
        return -1;
    }

    double position = atof(argv[1]); // Parse position as double

    if (strcmp(argv[2], "-r") == 0) {
        // Relative move
        cmd.head.type = cctMOVE_RELATIVE;
        cmd.request.args.asMoveRelative.relativePosition = position;
    } else if (strcmp(argv[2], "-a") == 0) {
        // Absolute move
        cmd.head.type = cctMOVE_ABSOLUTE;
        cmd.request.args.asMoveAbsolute.absolutePosition = position;
    } else if (strcmp(argv[2], "-s") == 0) {
        // Move with speed
        if (argc < 4) {
            printf("Missing speed argument for move command\r\nFAIL\r\n");
            return -1;
        }
        cmd.head.type = cctMOVE_WITH_SPEED;
        cmd.request.args.asMoveWithSpeed.absolutePosition = position;
        cmd.request.args.asMoveWithSpeed.speed = atof(argv[3]); // Parse speed as double
    } else {
        printf("Invalid subcommand for move command\r\nFAIL\r\n");
        return -1;
    }
}
  // Handle "reference" command
  // Handle "reference" command
else if (strcmp(argv[0], "reference") == 0) {
  if (argc == 1) {
      cmd.head.type = cctREFERENCE_ENABLE; // Matches "stepper reference"
      cmd.request.args.asReferenceEnable.enablePowerOutputs = 0; // No enabled power outputs -> only reference command
  } else if (argc >= 2) {
      if (strcmp(argv[1], "-e") == 0) {
          cmd.head.type = cctREFERENCE_ENABLE; // Matches "stepper reference -e"
          cmd.request.args.asReferenceEnable.enablePowerOutputs = 1;
      } else if (strcmp(argv[1], "-t") == 0) {
          if (argc < 3) {
              // Missing timeout value
              printf("missing timeout value for reference command\r\nFAIL");
              return -1;
          }
          cmd.head.type = cctREFERENCE_TIMEOUT; // Matches "stepper reference -t <time>"
          cmd.request.args.asReferenceWithTimeout.timeout = atoi(argv[2]);
      } else if (strcmp(argv[1], "-s") == 0) {
          cmd.head.type = cctREFERENCE_SKIP; // Matches "stepper reference -s"
          cmd.request.args.asReferenceSkip.skipReference = 1;
      } else {
          // Invalid subcommand
          printf("invalid subcommand for reference\r\nFAIL");
          return -1;
      }
  }
}
  // Handle "position" command
  else if (strcmp(argv[0], "position") == 0) {
    cmd.head.type = cctPOSITION; // Matches "stepper position"
  } 
  // Handle "cancel" command
  else if (strcmp(argv[0], "cancel") == 0) {
    cmd.head.type = cctCANCEL; // Matches "stepper cancel"
    cmd.request.args.asCancel.cancelOperation = 1;
  } 
  // Handle "status" command
  else if (strcmp(argv[0], "status") == 0) {
    cmd.head.type = cctSTATUS; // Matches "stepper status"
  } 
  // Handle "config" command
  else if (strcmp(argv[0], "config") == 0) {
    if (argc < 2) {
      // Missing parameter
      printf("missing parameter for config command\r\nFAIL");
      return -1;
    }

    if (strcmp(argv[1], "powerena") == 0) {
      cmd.head.type = cctCONFIG_POWER_ENABLE; // Matches "stepper config powerena [-v 0|1]"

      if (argc > 2 && strcmp(argv[2], "-v") == 0) {
        if (argc < 4) {
          // Missing value for powerena
          printf("missing value for powerena config\r\nFAIL");
          return -1;
        }
        cmd.request.args.asConfigPowerEnable.powerEnable = atoi(argv[3]);
      }
    } else {
      cmd.head.type = cctCONFIG_PARAMETER; // Matches "stepper config <parameter> [-v <value>]"
      strncpy(cmd.request.args.asConfigParameter.parameter, argv[1], sizeof(cmd.request.args.asConfigParameter.parameter) - 1);

      if (argc > 2 && strcmp(argv[2], "-v") == 0) {
        if (argc < 4) {
          // Missing value for parameter
          printf("missing value for parameter config\r\nFAIL");
          return -1;
        }
        cmd.request.args.asConfigParameter.value = atoi(argv[3]);
      }
    }
  } 
  // Handle invalid command
  else {
    printf("passed invalid subcommand\r\nFAIL");
    return -1;
  }
  // Execute the command based on cmd.head.type
switch (cmd.head.type) {
  case cctMOVE_RELATIVE:
      // Handle relative move
      printf("Moving by relative position: %.2f\r\n", cmd.request.args.asMoveRelative.relativePosition);
      {
          int steps = CalcRelative(cmd.request.args.asMoveRelative.relativePosition, stepspermm);
          if (L6474_StepIncremental(h, steps) == 0) {
              printf("Relative move command executed successfully\r\nOK\r\n");
          } else {
              printf("Error executing relative move command\r\nFAIL\r\n");
              response.code = -1;
          }
      }
      break;

  case cctMOVE_ABSOLUTE:
      // Handle absolute move
      printf("Moving to absolute position: %.2f\r\n", cmd.request.args.asMoveAbsolute.absolutePosition);
      {
          int steps = CalcAbsolute(cmd.request.args.asMoveAbsolute.absolutePosition, 0.0, stepspermm); // Assuming currentPosition is 0.0 for now
          if (L6474_StepIncremental(h, steps) == 0) {
              printf("Move command executed successfully\r\nOK\r\n");
          } else {
              printf("Error executing move command\r\nFAIL\r\n");
              response.code = -1;
          }
      }
      break;

  case cctMOVE_WITH_SPEED:
      // Handle move with speed
      printf("Moving to position: %.2f at speed: %.2f mm/min\r\n",
             cmd.request.args.asMoveWithSpeed.absolutePosition,
             cmd.request.args.asMoveWithSpeed.speed);
      // Add logic to handle speed if needed
      {
          int steps = CalcAbsolute(cmd.request.args.asMoveWithSpeed.absolutePosition, 0.0, stepspermm); // Assuming currentPosition is 0.0 for now
          if (L6474_StepIncremental(h, steps) == 0) {
              printf("Move with speed command executed successfully\r\nOK\r\n");
          } else {
              printf("Error executing move with speed command\r\nFAIL\r\n");
          }
      }
      break;

      case cctREFERENCE_ENABLE: 
      {
        printf("Starting homing procedure...\r\n");
        // Move towards the limit switch (home position)
        while (HAL_GPIO_ReadPin(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin) == GPIO_PIN_SET) {
        if (L6474_StepIncremental(h, -1) != 0) { // Move one step in the negative direction
            printf("Error during homing\r\nFAIL\r\n");
            return -1;
        }
        }

        // Set the current position to 0.0 (home position)
        args->currentPosition = 0.0;
        h->currentPosition = 0.0; // Update the handle's current position
        printf("Homing complete: Home position set to 0.0 mm\r\n");

        // Move away from the limit switch slightly to avoid re-triggering
        if (L6474_StepIncremental(h, stepspermm) != 0) { // Move 1 mm away
        printf("Error moving away from limit switch\r\nFAIL\r\n");
        return -1;
        }step

        // Move towards the reference switch (max position)
        int stepsToMax = 0;
        while (HAL_GPIO_ReadPin(REFERENCE_MARK_GPIO_Port, REFERENCE_MARK_Pin) == GPIO_PIN_SET) {
        // Move one step at a time using L6474_StepIncremental
        if (L6474_StepIncremental(h, 1) != 0) { // Move one step in the positive direction
            printf("Error during homing\r\nFAIL\r\n");
            return -1;
        }
        stepsToMax++;
        }

        // Calculate and set the MAXPOS
        stepperArgs->MAXPOS = (double)stepsToMax / stepspermm;
        h->MAXPOS = stepperArgs->MAXPOS; // Update the handle's MAXPOS
        stepperArgs->currentPosition = stepperArgs->MAXPOS; // Set current position to MAXPOS
        h->currentPosition = stepperArgs->MAXPOS; // Update the handle's current position
        printf("Homing complete: Max position set to %.2f mm\r\n", stepperArgs->MAXPOS);

        // Move away from the reference switch slightly to avoid re-triggering
        if (L6474_StepIncremental(h, -stepspermm) != 0) { // Move 1 mm away
        printf("Error moving away from reference switch\r\nFAIL\r\n");
        return -1;
        }

        // Mark the stepper as homed
        stepperArgs->homed = true;
        h->homed = true; // Update the handle's homed status
        printf("Homing procedure completed successfully\r\nOK\r\n");
        break;
        }
  case cctREFERENCE_TIMEOUT:
      // Handle reference with timeout
      printf("Starting reference with timeout: %d ms\r\n", cmd.request.args.asReferenceWithTimeout.timeout);
      // Add logic to handle timeout
      printf("Reference with timeout completed\r\nOK\r\n");
      break;

  case cctREFERENCE_SKIP:
      // Handle reference skip
      printf("Skipping reference\r\n");
      // Add logic to skip reference
      printf("Reference skipped successfully\r\nOK\r\n");
      break;

  case cctPOSITION:
      // Handle position query
      printf("Querying current position\r\n");
      // Add logic to get the current position
      printf("Current position: %d\r\nOK\r\n", 0); // Replace 0 with actual position
      break;

  case cctCANCEL:
      // Handle cancel operation
      printf("Cancelling current operation\r\n");
      // Add logic to cancel the current operation
      printf("Operation cancelled successfully\r\nOK\r\n");
      break;

  case cctSTATUS:
      // Handle status query
      printf("Querying stepper status\r\n");
      // Add logic to get the status
      printf("Stepper is running: %d\r\n", 1); // Replace 1 with actual running status
      printf("Current speed: %.2f mm/min\r\nOK\r\n", 500.0f); // Replace 500.0f with actual speed
      break;

  case cctCONFIG_POWER_ENABLE:
      // Handle power enable configuration
      printf("Setting power enable to: %d\r\n", cmd.request.args.asConfigPowerEnable.powerEnable);
      // Add logic to enable/disable power
      printf("Power enable set successfully\r\nOK\r\n");
      break;

  case cctCONFIG_PARAMETER:
      // Handle parameter configuration
      printf("Setting parameter: %s to value: %d\r\n",
             cmd.request.args.asConfigParameter.parameter,
             cmd.request.args.asConfigParameter.value);
      // Add logic to configure the parameter
      printf("Parameter configured successfully\r\nOK\r\n");
      break;

  default:
      // Handle unknown command
      printf("Unknown command type\r\nFAIL\r\n");
      break;
}
  /*
	// now pass the request to the controller
	cmd.request.syncEvent = GetCommandEvent(h);

	if ( pdPASS != xQueueSend( h->cmdQueue, &cmd, -1 ) )
	{
		ReleaseCommandEvent(h, cmd.request.syncEvent );
		return -1;
	}

	xSemaphoreTake( cmd.request.syncEvent, -1 );
	ReleaseCommandEvent(h, cmd.request.syncEvent );

	// now decode the result in case there is one
	if ( response.code == 0 )
	{
		if ( cmd.head.type == cctSTATUS )
		{
			printf("%d\r\n", !!cmd.response->args.asStatus.running);
			printf("%d\r\n", (int)cmd.response->args.asStatus.speed);
		}
		printf("OK");
	}
	else
	{
		printf("error returned\r\nFAIL");
	}

	// now back to console*/
	return response.code;
}


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

int StepTimerCancelAsync(void* pPWM)
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

int StepTimerAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h) {
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

int StepSynchronous(void* pPWM, int dir, unsigned int numPulses) {
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
    stepperArgs->currentPosition = 0.0; // Initialize the current position to 0.0 mm
    stepperArgs->homed = false;        // Initialize as not homed
    stepperArgs->MAXPOS = 0.0;         // Initialize max position to 0.0 mm

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
    stepperHandle = L6474_CreateInstance(&p, NULL, NULL, NULL);

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
    result |= L6474_ResetStandBy(stepperHandle);
    // Initialize the driver with the base parameters
    result |= L6474_Initialize(stepperHandle, &baseParam);
    result |= L6474_SetPowerOutputs(stepperHandle, 1);

    // In case we have no error, we can enable the drivers
    // and then we step a bit
    if (result == 0) {
        result |= L6474_StepIncremental(stepperHandle, 1000);
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

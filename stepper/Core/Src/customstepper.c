#include "FreeRTOS.h"   
#include "task.h"           
#include "timers.h"        
#include "stdio.h"          
#include "LibL6474.h"       
#include "main.h"    
#include <stddef.h>  // for NULL
#include "customstepper.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define HOMING_SPEED 5000
#define DEFAULT_RUN_SPEED 4000

extern SPI_HandleTypeDef hspi1;
extern L6474_Handle_t stepperHandle; 

extern StepperTaskArgs_t* stepperArgs;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

int stepspermm = 100; //steps/mm for the stepper motor

L6474x_Platform_t p; //platform needs to be global for async

static int CalcAbsolute(double position, double currentPosition, StepperTaskArgs_t* ctx) {
    double steps_per_mm = (ctx->steps_per_turn * ctx->res) / ctx->mm_per_turn;
    double deltaPosition = position - currentPosition;
    return (int)(deltaPosition * steps_per_mm);
}

static int CalcRelative(double relativePosition, StepperTaskArgs_t* ctx) {
    double steps_per_mm = (ctx->steps_per_turn * ctx->res) / ctx->mm_per_turn;
    return (int)(relativePosition * steps_per_mm);
}

//static int ConsoleWriteStream_ToStdErr(void* pContext, const char* pBuffer, int num)
//{
//	(void)pContext;
//	extern int _write( int file, char* ptr, int len );
//	return _write(2, pBuffer, num);
//}
/* USER CODE END 0 */
//// LETS GO GAMBLING!!!! (ASYNC)
void setSpeed(StepperTaskArgs_t* ctx, int stepsPerSecond) {
    int clk = HAL_RCC_GetHCLKFreq();
    int quotient = clk / (stepsPerSecond * 2); // scale by 2 -> rising/falling edge
    int i = 0;
    while ((quotient / (i + 1)) > 65535) i++;
    __HAL_TIM_SET_PRESCALER(ctx->htim4Handle, i);
    __HAL_TIM_SET_AUTORELOAD(ctx->htim4Handle, (quotient / (i + 1)) - 1);
    ctx->htim4Handle->Instance->CCR4 = ctx->htim4Handle->Instance->ARR / 2;
}

// Start the timer for a given number of pulses (async, chunked if needed)
void startTim1(int pulses) {
	extern StepperTaskArgs_t* stepperArgs;
    int currentPulses = (pulses >= 65535) ? 65535 : pulses;
    stepperArgs->remainingPulses = pulses - currentPulses;

    if (currentPulses > 0) {
        HAL_TIM_OnePulse_Stop_IT(stepperArgs->htim1Handle, TIM_CHANNEL_1);
        __HAL_TIM_SET_AUTORELOAD(stepperArgs->htim1Handle, currentPulses);
        HAL_TIM_GenerateEvent(stepperArgs->htim1Handle, TIM_EVENTSOURCE_UPDATE);
        HAL_TIM_OnePulse_Start_IT(stepperArgs->htim1Handle, TIM_CHANNEL_1);
        __HAL_TIM_ENABLE(stepperArgs->htim1Handle);
    } else {
        if (stepperArgs->doneClb) stepperArgs->doneClb(stepperArgs->h);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
	if ((stepperArgs->doneClb != 0) && ((htim->Instance->SR & (1 << 2)) == 0)) {
		if (stepperArgs->remainingPulses > 0) {
			startTim1(stepperArgs->remainingPulses);
		}
		else {
			stepperArgs->doneClb(stepperArgs->h);
			stepperArgs->currentlyrunning = 0;
		}
	}
}

// Async stepper start (non-blocking)
static int stepAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h) {
  (void)pPWM;
	(void)h;  
  stepperArgs->currentlyrunning = 1;
  stepperArgs->doneClb = doneClb;

    HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, dir);

    startTim1(numPulses);

    return 0;
}

// Cancel async stepper operation

static int stepCancelAsync(void* pPWM)
{
	(void)pPWM;

	if (stepperArgs->currentlyrunning) {
		HAL_TIM_OnePulse_Stop_IT(stepperArgs->htim1Handle, TIM_CHANNEL_1);
		stepperArgs->doneClb(stepperArgs->h);
    stepperArgs->currentlyrunning = 0;
	}

	return 0;
}


// PowerENA Function
static int powerEnable(StepperTaskArgs_t* context, StepperCtrlCommand_t* cmd) {
    int ena = cmd->request.args.asConfigPowerEnable.powerEnable;

    // If no -v argument was given, just print the current state
    if (cmd->head.type == cctCONFIG_POWER_ENABLE && cmd->request.args.asConfigPowerEnable.powerEnable == 0 && cmd->request.args.asConfigPowerEnable.value == 0) {
        printf("%d\r\nOK\r\n", context->powered);
        return 0;
    }

    // Otherwise, set the power output
    if (ena != 0 && ena != 1) {
        printf("Invalid argument for powerena\r\nFAIL\r\n");
        return -1;
    }
    context->powered = ena;
    if (L6474_SetPowerOutputs(context->h, ena) == 0) {
        printf("OK\r\n");
        return 0;
    } else {
        printf("FAIL\r\n");
        return -1;
    }
}
// config parameter hack function
static int configParams(StepperTaskArgs_t* ctx, StepperCtrlCommand_t* cmd) {
    const char* param = cmd->request.args.asConfigParameter.parameter;
    int has_value = cmd->request.args.asConfigParameter.value_set;   
    int value = cmd->request.args.asConfigParameter.value;

    // POWERENA
    if (strcmp(param, "powerena") == 0) {
        if (!has_value) {
            printf("%d\r\nOK\r\n", ctx->powered);
            return 0;
        } else {
            if (value != 0 && value != 1) {
                printf("Invalid argument for powerena\r\nFAIL\r\n");
                return -1;
            }
            ctx->powered = value;
            if (L6474_SetPowerOutputs(ctx->h, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // TORQUE
    else if (strcmp(param, "torque") == 0) {
        if (!has_value) {
            int tval = 0;
            L6474_GetProperty(ctx->h, L6474_PROP_TORQUE, &tval);
            printf("%d\r\nOK\r\n", tval);
            return 0;
        } else {
            if (L6474_SetProperty(ctx->h, L6474_PROP_TORQUE, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // TIMEON
    else if (strcmp(param, "timeon") == 0) {
        if (ctx->powered == 1) return -1;
        if (!has_value) {
            int tval = 0;
            L6474_GetProperty(ctx->h, L6474_PROP_TON, &tval);
            printf("%d\r\nOK\r\n", tval);
            return 0;
        } else {
            if (L6474_SetProperty(ctx->h, L6474_PROP_TON, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // TIMEOFF
    else if (strcmp(param, "timeoff") == 0) {
        if (ctx->powered == 1) return -1;
        if (!has_value) {
            int tval = 0;
            L6474_GetProperty(ctx->h, L6474_PROP_TOFF, &tval);
            printf("%d\r\nOK\r\n", tval);
            return 0;
        } else {
            if (L6474_SetProperty(ctx->h, L6474_PROP_TOFF, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // TIMEFAST
    else if (strcmp(param, "timefast") == 0) {
        if (ctx->powered == 1) return -1;
        if (!has_value) {
            int tval = 0;
            L6474_GetProperty(ctx->h, L6474_PROP_TFAST, &tval);
            printf("%d\r\nOK\r\n", tval);
            return 0;
        } else {
            if (L6474_SetProperty(ctx->h, L6474_PROP_TFAST, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // THROVERCURR
    else if (strcmp(param, "throvercurr") == 0) {
        if (!has_value) {
            int tval = 0;
            L6474_GetProperty(ctx->h, L6474_PROP_OCDTH, &tval);
            printf("%d\r\nOK\r\n", tval);
            return 0;
        } else {
            if (L6474_SetProperty(ctx->h, L6474_PROP_OCDTH, value) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // STEPMODE
    else if (strcmp(param, "stepmode") == 0) {
        if (ctx->powered == 1) return -1;
        if (!has_value) {
            printf("%d\r\nOK\r\n", ctx->res);
            return 0;
        } else {
            int res = value;
            L6474x_StepMode_t step_mode;
            switch (res) {
                case 1: step_mode = smFULL; break;
                case 2: step_mode = smHALF; break;
                case 4: step_mode = smMICRO4; break;
                case 8: step_mode = smMICRO8; break;
                case 16: step_mode = smMICRO16; break;
                default:
                    printf("Invalid step mode\r\nFAIL\r\n");
                    return -1;
            }
            ctx->res = res;
            if (L6474_SetStepMode(ctx->h, step_mode) == 0) {
                printf("OK\r\n");
                return 0;
            } else {
                printf("FAIL\r\n");
                return -1;
            }
        }
    }
    // STEPSPER TURN
    else if (strcmp(param, "stepsperturn") == 0) {
        if (!has_value) {
            printf("%d\r\nOK\r\n", ctx->steps_per_turn);
            return 0;
        } else {
            ctx->steps_per_turn = value;
            printf("OK\r\n");
            return 0;
        }
    }
    // MMPERTURN
    else if (strcmp(param, "mmperturn") == 0) {
        if (!has_value) {
            printf("%f\r\nOK\r\n", ctx->mm_per_turn);
            return 0;
        } else {
            ctx->mm_per_turn = (double)value;
            printf("OK\r\n");
            return 0;
        }
    }
    // POSMIN
    else if (strcmp(param, "posmin") == 0) {
        if (!has_value) {
            printf("%f\r\nOK\r\n", (double)ctx->position_min_steps * ctx->mm_per_turn / (ctx->steps_per_turn * ctx->res));
            return 0;
        } else {
            ctx->position_min_steps = (int)(value * ctx->steps_per_turn * ctx->res / ctx->mm_per_turn);
            printf("OK\r\n");
            return 0;
        }
    }
    // POSMAX
    else if (strcmp(param, "posmax") == 0) {
        if (!has_value) {
            printf("%f\r\nOK\r\n", (double)ctx->position_max_steps * ctx->mm_per_turn / (ctx->steps_per_turn * ctx->res));
            return 0;
        } else {
            ctx->position_max_steps = (int)(value * ctx->steps_per_turn * ctx->res / ctx->mm_per_turn);
            printf("OK\r\n");
            return 0;
        }
    }
    // POSREF
    else if (strcmp(param, "posref") == 0) {
        if (!has_value) {
            printf("%f\r\nOK\r\n", (double)ctx->position_ref_steps * ctx->mm_per_turn / (ctx->steps_per_turn * ctx->res));
            return 0;
        } else {
            ctx->position_ref_steps = (int)(value * ctx->steps_per_turn * ctx->res / ctx->mm_per_turn);
            printf("OK\r\n");
            return 0;
        }
    }
    else {
        printf("Invalid parameter\r\nFAIL\r\n");
        return -1;
    }
}
static int reportStatus()
{
    L6474_Status_t driverStatus;
    L6474_GetStatus(stepperArgs->h, &driverStatus);

    unsigned int statusBits = 0;
    statusBits |= (driverStatus.DIR ? (1 << 0) : 0);
    statusBits |= (driverStatus.HIGHZ ? (1 << 1) : 0);
    statusBits |= (driverStatus.NOTPERF_CMD ? (1 << 2) : 0);
    statusBits |= (driverStatus.OCD ? (1 << 3) : 0);
    statusBits |= (driverStatus.ONGOING ? (1 << 4) : 0);
    statusBits |= (driverStatus.TH_SD ? (1 << 5) : 0);
    statusBits |= (driverStatus.TH_WARN ? (1 << 6) : 0);
    statusBits |= (driverStatus.UVLO ? (1 << 7) : 0);
    statusBits |= (driverStatus.WRONG_CMD ? (1 << 8) : 0);

    printf("0x%01X\r\n", stepperArgs->powered);
    printf("0x%04X\r\n", statusBits);
    printf("%d\r\n", stepperArgs->currentlyrunning);

    return 0;
}
//reset function for a steppah
static int reset(StepperTaskArgs_t* stepper_ctx) {
	L6474_BaseParameter_t param; // recreate baseparam
	param.stepMode = smMICRO16;
	param.OcdTh = ocdth6000mA;
	param.TimeOnMin = 0x29;
	param.TimeOffMin = 0x29;
	param.TorqueVal = 0x26;
	param.TFast = 0x19;

	int result = 0;

	result |= L6474_ResetStandBy(stepper_ctx->h);
	result |= L6474_Initialize(stepper_ctx->h, &param);

	result |= L6474_SetPowerOutputs(stepper_ctx->h, 0);
 //call functions again
	stepper_ctx->powered = 0;
	stepper_ctx->referenced = 0;
	stepper_ctx->currentlyrunning = 0;

	return result;
}


// console function, call with ctx!!!

// Stepper Console Function
// --------------------------------------------------------------------------------------------------------------------
int StepperConsoleFunction(int argc, char** argv, void* ctx)
// --------------------------------------------------------------------------------------------------------------------
{
	//possible commands are
	// stepper move (absolutePos) ->
	// stepper move (relativePos) -r
	// stepper move (absPos) -s (speed in mm/min)
	// stepper reference  -> referenzfahrt
	// stepper reset  -> reset and re-initialize stepper


  StepperTaskArgs_t* args = (StepperTaskArgs_t*)ctx; // Cast ctx to StepperTaskArgs_t*
  L6474_Handle_t handle = args->h; // Access the stepper handle
	StepCommandResponse_t response = { 0 };
	StepperCtrlCommand_t cmd;

  //register command like this: CONSOLE_RegisterCommand(consoleHandle, "stepper", "Control the stepper motor", StepperConsoleFunction, stepperArgs);

	cmd.response       = &response;
  // First decode the subcommand and all arguments
  if (argc == 0) {
    // No arguments provided
    printf("invalid number of arguments\r\nFAIL");
    response.code = -1;
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
        cmd.request.args.asConfigParameter.value_set = 1;
        } else {
        cmd.request.args.asConfigParameter.value_set = 0;
        }
    }
  } 
  else if(strcmp(argv[0], "reset") == 0) {
    // Handle reset command
    cmd.head.type = cctRESET; // Matches "stepper reset"
  }
  // Handle invalid command
  else {
    printf("passed invalid subcommand\r\nFAIL");
    return -1;
  }
  // Execute the command based on cmd.head.type
switch (cmd.head.type) {
  case cctMOVE_RELATIVE: {
    if (args->powered != 1) {
        printf("Stepper not powered\r\n");
        response.code = -1;
        break;
    }
    if (args->referenced != 1) {
        printf("Stepper not referenced\r\n");
        response.code = -1;
        break;
    }
    double rel_mm = cmd.request.args.asMoveRelative.relativePosition;
    int steps = (int)(rel_mm * args->steps_per_turn * args->res / args->mm_per_turn);

    int abs_pos;
    L6474_GetAbsolutePosition(args->h, &abs_pos);
    int resulting_steps = abs_pos + steps;

    if (resulting_steps < args->position_min_steps || resulting_steps > args->position_max_steps) {
        printf("Position out of bounds\r\n");
        response.code = -1;
        break;
    }

    setSpeed(args, DEFAULT_RUN_SPEED); // default speed

    int result = L6474_StepIncremental(args->h, steps);
    if (result == 0) {
        printf("Relative move command executed successfully\r\nOK\r\n");
    } else {
        printf("Error executing relative move command\r\nFAIL\r\n");
        response.code = -1;
    }
    break;
}

case cctMOVE_ABSOLUTE: {
    if (args->powered != 1) {
        printf("Stepper not powered\r\n");
        response.code = -1;
        break;
    }
    if (args->referenced != 1) {
        printf("Stepper not referenced\r\n");
        response.code = -1;
        break;
    }
    double abs_mm = cmd.request.args.asMoveAbsolute.absolutePosition;
    int target_steps = (int)(abs_mm * args->steps_per_turn * args->res / args->mm_per_turn);

    int abs_pos;
    L6474_GetAbsolutePosition(args->h, &abs_pos);
    int steps = target_steps - abs_pos;

    if (target_steps < args->position_min_steps || target_steps > args->position_max_steps) {
        printf("Position out of bounds\r\n");
        response.code = -1;
        break;
    }

    setSpeed(args, DEFAULT_RUN_SPEED); // default speed

    int result = L6474_StepIncremental(args->h, steps);
    if (result == 0) {
        printf("Move command executed successfully\r\nOK\r\n");
    } else {
        printf("Error executing move command\r\nFAIL\r\n");
        response.code = -1;
    }
    break;
}

case cctMOVE_WITH_SPEED: {
    if (args->powered != 1) {
        printf("Stepper not powered\r\n");
        response.code = -1;
        break;
    }
    if (args->referenced != 1) {
        printf("Stepper not referenced\r\n");
        response.code = -1;
        break;
    }
    double abs_mm = cmd.request.args.asMoveWithSpeed.absolutePosition;
    double speed = cmd.request.args.asMoveWithSpeed.speed; // mm/min

    int steps_per_second = (int)(speed * args->steps_per_turn * args->res / (60.0 * args->mm_per_turn));
    if (steps_per_second < 1) {
        printf("Speed too small\r\n");
        response.code = -1;
        break;
    }
    setSpeed(args, steps_per_second);

    int target_steps = (int)(abs_mm * args->steps_per_turn * args->res / args->mm_per_turn);
    int abs_pos;
    L6474_GetAbsolutePosition(args->h, &abs_pos);
    int steps = target_steps - abs_pos;

    if (target_steps < args->position_min_steps || target_steps > args->position_max_steps) {
        printf("Position out of bounds\r\n");
        response.code = -1;
        break;
    }

    int result = L6474_StepIncremental(args->h, steps);
    if (result == 0) {
        printf("Move with speed command executed successfully\r\nOK\r\n");
    } else {
        printf("Error executing move with speed command\r\nFAIL\r\n");
        response.code = -1;
    }
    break;
}
    case cctREFERENCE_TIMEOUT:
    case cctREFERENCE_ENABLE: 
    {
        // Get timeout from command if present, else use args->timeout_ms or 0
        uint32_t timeout_ms = 0;
        if (cmd.head.type == cctREFERENCE_TIMEOUT) {
            timeout_ms = cmd.request.args.asReferenceWithTimeout.timeout;
            args->timeout_ms = timeout_ms;
        } else if (args->timeout_ms > 0) {
            timeout_ms = args->timeout_ms;
        }

        printf("Starting homing procedure...\r\n");
        int result = 0;
        uint32_t start_time = HAL_GetTick();

        // Power on stepper
        result |= L6474_SetPowerOutputs(handle, 1);
        args->powered = 1;
        setSpeed(args,HOMING_SPEED );

        // If at limit switch, move away first
        if (HAL_GPIO_ReadPin(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin) == GPIO_PIN_RESET) {
            printf("At limit switch, moving away before reference run...\r\n");
            L6474_StepIncremental(handle, -5000);
            while (HAL_GPIO_ReadPin(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin) == GPIO_PIN_RESET) {
                if (timeout_ms > 0 && HAL_GetTick() - start_time > timeout_ms) {
                    stepCancelAsync(args);
                    printf("Timeout while moving away from limit switch\r\nFAIL\r\n");
                    return -1;
                }
            }
            stepCancelAsync(args);
        }

        // If already at reference switch, move off it
        if (HAL_GPIO_ReadPin(REFERENCE_MARK_GPIO_Port, REFERENCE_MARK_Pin) == GPIO_PIN_RESET) {
            L6474_StepIncremental(handle, 100000000);
            while (HAL_GPIO_ReadPin(REFERENCE_MARK_GPIO_Port, REFERENCE_MARK_Pin) == GPIO_PIN_RESET) {
                if (timeout_ms > 0 && HAL_GetTick() - start_time > timeout_ms) {
                    stepCancelAsync(args);
                    printf("Timeout while waiting for reference switch\r\nFAIL\r\n");
                    return -1;
                }
            }
            stepCancelAsync(args);
        }

        // Move to reference switch
        L6474_StepIncremental(handle, -1000000000);
        while (HAL_GPIO_ReadPin(REFERENCE_MARK_GPIO_Port, REFERENCE_MARK_Pin) != GPIO_PIN_RESET) {
            if (timeout_ms > 0 && HAL_GetTick() - start_time > timeout_ms) {
                stepCancelAsync(args);
                printf("Timeout while waiting for reference switch\r\nFAIL\r\n");
                return -1;
            }
        }
        stepCancelAsync(args);

        // Set reference position
        L6474_SetAbsolutePosition(handle, 800);
        args->position_min_steps = 800;
        args->position_ref_steps = 800;

        // Move to limit switch from reference
        int step_amt = 0;
        uint32_t track_timer_start = HAL_GetTick();
        L6474_StepIncremental(handle, 1000000000);
        while (HAL_GPIO_ReadPin(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin) != GPIO_PIN_RESET) {
            step_amt++;
            if (timeout_ms > 0 && HAL_GetTick() - start_time > timeout_ms) {
                stepCancelAsync(args);
                printf("Timeout while moving to limit switch\r\nFAIL\r\n");
                return -1;
            }
        }
        uint32_t track_timer_stop = HAL_GetTick();
        stepCancelAsync(args);

        // Get max position
        int abs_pos = 0;
        L6474_GetAbsolutePosition(handle, &abs_pos);
        args->position_max_steps = abs_pos;

        // Optionally calculate mm_per_step, mm_per_sec, etc. if needed

        args->referenced = 1;
        args->homed = true;
        printf("Homing procedure completed successfully\r\nOK\r\n");
        break;
    }
  case cctRESET:
    reset(args);
    args->currentlyrunning = 0;
    args->referenced = 0;
    args->powered = 0;
    printf("Stepper reset\r\nOK\r\n");
    break;
  case cctREFERENCE_SKIP:
      // Handle reference skip
      printf("Skipping reference\r\n");
      // Add logic to skip reference
      args->referenced = 1; // Set referenced flag
      L6474_SetAbsolutePosition(args->h, 0);
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
    powerEnable(args, &cmd);
    break;

  case cctCONFIG_PARAMETER:
      // Call the custom config function
      configParams(args, &cmd);
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
  HAL_GPIO_WritePin(STEP_RSTN_GPIO_Port, STEP_RSTN_Pin, !ena);
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
/*
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
}*/

/*
int StepSynchronous(void* pPWM, int dir, int numPulses) {
	int direction = 1;
	if(numPulses < 0){
		numPulses = -numPulses;
		direction = 0;
	}
	(void)pPWM; // Unused in this implementation

  // Set direction pin
  HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, direction);

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
}*/

void StepperTask(void *pvParameters)
{
    StepperTaskArgs_t* args = (StepperTaskArgs_t*)pvParameters;



    // Initialize hardware
    HAL_GPIO_WritePin(STEP_SPI_CS_GPIO_Port, STEP_SPI_CS_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // Platform abstraction
    L6474x_Platform_t p;
    p.malloc     = StepLibraryMalloc;
    p.free       = StepLibraryFree;
    p.transfer   = StepDriverSpiTransfer;
    p.reset      = StepDriverReset;
    p.sleep      = StepLibraryDelay;
    p.stepAsync  = stepAsync;           // Your async function
    p.cancelStep = stepCancelAsync;     // Your cancel function

    // Create stepper instance
    args->h = L6474_CreateInstance(&p, &hspi1, NULL, &htim1);
    args->htim1Handle = &htim1;
    args->htim4Handle = &htim4;

    // Set default parameters
    args->steps_per_turn = STEPS_PER_TURN;
    args->res = RESOLUTION;
    args->mm_per_turn = MM_PER_TURN;

    args->position_min_steps = 0;
    args->position_max_steps = 100000;
    args->position_ref_steps = 0;

    args->powered = 0;
    args->referenced = 0;
    args->currentlyrunning = 0;
    args->homed = false;
    args->currentPosition = 0.0;
    args->MAXPOS = 0.0;

    // Optionally: initialize driver with base parameters
    L6474_BaseParameter_t baseParam = {
        .stepMode   = smMICRO16,
        .OcdTh      = ocdth1125mA,
        .TimeOnMin  = 10,
        .TimeOffMin = 15,
        .TorqueVal  = 80,
        .TFast      = 5
    };
    int result = 0;

    result |= L6474_SetBaseParameter(&baseParam);
    result |= L6474_ResetStandBy(args->h);
    result |= L6474_Initialize(args->h, &baseParam);
    result |= L6474_SetPowerOutputs(args->h, 0);
    if(result != 0){
    	printf("Critical Error initializing Stepper, result: %d\n", result);
    }
    // JANK SHIT LOL
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Cleanup (never reached in normal operation)
    vTaskDelete(NULL);
}

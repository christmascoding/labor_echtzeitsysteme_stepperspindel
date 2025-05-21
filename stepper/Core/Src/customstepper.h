#ifndef CUSTOMSTEPPER_H
#define CUSTOMSTEPPER_H

#include<stdbool.h>
#include "semphr.h"
#include "LibL6474.h"     // For L6474_Handle_t and related structures
#include "FreeRTOS.h"  // For TaskHandle_t

#ifdef __cplusplus
extern "C" {
#endif

#define STEPS_PER_TURN 200
#define RESOLUTION 16
#define MM_PER_TURN 4

typedef struct StepCommandResponse // need ssome work this is unfinished af
// --------------------------------------------------------------------------------------------------------------------
{
    int code;       // Response code: 0 for success, non-zero for failure
    int requestID;  // Unique ID of the request
    union
    {
        struct
        {
            float speed;   // Speed in mm/min
            int running;   // 1 if the stepper is running, 0 otherwise
        } asStatus;        // Response for "stepper status"

        struct
        {
            int position;  // Current position of the stepper
        } asPosition;      // Response for "stepper position"

        struct
        {
            int powerEnabled; // 1 if power outputs are enabled, 0 otherwise
        } asConfigPowerEnable; // Response for "stepper config powerena"

        struct
        {
            char parameter[32]; // Parameter name
            int value;          // Parameter value
            int value_set; // 1 if value is set, 0 otherwise
        } asConfigParameter;    // Response for "stepper config <parameter>"

        struct
        {
            int timeoutOccurred; // 1 if timeout occurred, 0 otherwise
        } asReferenceWithTimeout; // Response for "stepper reference -t"

        struct
        {
            int skipped; // 1 if reference was skipped, 0 otherwise
        } asReferenceSkip; // Response for "stepper reference -s"

        struct
        {
            int success; // 1 if reference was successful, 0 otherwise
        } asReferenceEnable; // Response for "stepper reference -e"

        struct
        {
            int canceled; // 1 if the operation was canceled, 0 otherwise
        } asCancel; // Response for "stepper cancel"
    } args; // Union for different response types
} StepCommandResponse_t;

// Structure to store stepper task parameters
typedef struct {
    void* pPWM;
    int dir;
    unsigned int numPulses;
    void (*doneClb)(L6474_Handle_t); //callback for fuck sake
    L6474_Handle_t h;
    TaskHandle_t taskHandle;
    double currentPosition;
    bool homed;              // Whether the stepper has been homed
    double MAXPOS;           // Maximum position of the stepper in mm
    int currentlyrunning;
    int powered;
    int referenced;
    int res;           
    int position_min_steps;   // for posmin
    int position_max_steps;   // for posmax
    int position_ref_steps;   // for posref
    int steps_per_turn;     // \/ \/ :(
    int mm_per_turn;        // unfortunately we need these
    TIM_HandleTypeDef* htim1Handle; // i want to kill myself 
    TIM_HandleTypeDef* htim4Handle; // TWO HANDLES POWER OMG 
    int remainingPulses;
} StepperTaskArgs_t;

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
  cctCONFIG_PARAMETER    = 0x0D,
  cctRESET               = 0x0E
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
        int value_set; // 1 if value is set, 0 otherwise
      } asConfigParameter;

    } args;
    SemaphoreHandle_t syncEvent; // Synchronization event
  } request;
  StepCommandResponse_t* response; // Response structure
} StepperCtrlCommand_t;


// Task function to control stepper motor in a FreeRTOS task
void vStepperPulseTask(void* pvParameters);

// Asynchronous stepper pulse function
int StepTimerAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h);

// Cancels an asynchronous step operation
int StepTimerCancelAsync(void* pPWM);

// Synchronous stepping function
int StepSynchronous(void* pPWM, int dir, int numPulses);

// Stepper console function
int StepperConsoleFunction(int argc, char** argv, void* ctx);

// Main stepper task for initialization and default movement
void StepperTask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif // CUSTOMSTEPPER_H


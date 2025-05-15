#ifndef CUSTOMSTEPPER_H
#define CUSTOMSTEPPER_H

#include<stdbool.h>
#include "semphr.h"
#include "LibL6474.h"     // For L6474_Handle_t and related structures
#include "FreeRTOS.h"  // For TaskHandle_t

#ifdef __cplusplus
extern "C" {
#endif

// Structure to store stepper task parameters
typedef struct {
    void* pPWM;
    int dir;
    unsigned int numPulses;
    void (*doneClb)(L6474_Handle_t);
    L6474_Handle_t h;
    TaskHandle_t taskHandle;
    double currentPosition;
    bool homed;              // Whether the stepper has been homed
    double MAXPOS;           // Maximum position of the stepper in mm
} StepperTaskArgs_t;

// Task function to control stepper motor in a FreeRTOS task
void vStepperPulseTask(void* pvParameters);

// Asynchronous stepper pulse function
int StepTimerAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h);

// Cancels an asynchronous step operation
int StepTimerCancelAsync(void* pPWM);

// Synchronous stepping function
int StepSynchronous(void* pPWM, int dir, unsigned int numPulses);

// Stepper console function
int StepperConsoleFunction(int argc, char** argv, void* ctx);

// Main stepper task for initialization and default movement
void StepperTask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif // CUSTOMSTEPPER_H

typedef struct StepCommandResponse
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

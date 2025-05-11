#ifndef CUSTOMSTEPPER_H
#define CUSTOMSTEPPER_H

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
} StepperTaskArgs_t;

// Task function to control stepper motor in a FreeRTOS task
void vStepperPulseTask(void* pvParameters);

// Asynchronous stepper pulse function
int StepTimerAsync(void* pPWM, int dir, unsigned int numPulses, void (*doneClb)(L6474_Handle_t), L6474_Handle_t h);

// Cancels an asynchronous step operation
int StepTimerCancelAsync(void* pPWM);

// Synchronous stepping function
int StepSynchronous(void* pPWM, int dir, unsigned int numPulses);

// Main stepper task for initialization and default movement
void StepperTask(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif // CUSTOMSTEPPER_H

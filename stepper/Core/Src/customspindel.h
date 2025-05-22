#ifndef CUSTOMSPINDEL_H
#define CUSTOMSPINDEL_H

#include "LibL6474.h"     // For L6474_Handle_t and related structures
#include "FreeRTOS.h"     // For TaskHandle_t
#include "Spindle.h"

#ifdef __cplusplus
extern "C" {
#endif

void SpindleTask(void* pvParameters);
//void init_spindle(ConsoleHandle_t console_handle, TIM_HandleTypeDef tim_handle);



#ifdef __cplusplus
}
#endif

#endif // CUSTOMSPINDEL_H
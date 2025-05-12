#ifndef CUSTOMSPINDEL_H
#define CUSTOMSPINDEL_H

#include "LibL6474.h"     // For L6474_Handle_t and related structures
#include "FreeRTOS.h"     // For TaskHandle_t
#include "Spindle.h"

#ifdef __cplusplus
extern "C" {
#endif

void SpindleTask(void* pvParameters);



#ifdef __cplusplus
}
#endif

#endif // CUSTOMSPINDEL_H

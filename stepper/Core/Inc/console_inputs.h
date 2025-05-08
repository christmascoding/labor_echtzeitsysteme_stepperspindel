#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h" // Include the header for TimerCallbackFunction_t
#include "stdio.h"
#include "LibL6474.h"
#include "ConsoleConfig.h"


// example: stepper move 1000 -r 

typedef struct {
    char main_command[64];
    char subcommand[64];
    int param;
    char flag[1];
} ConsoleArg_t;

typedef struct {
    ConsoleArg_t args[16];
    int count;
} ConsoleArgs_t;

int ParseConsoleInput(const char* input, ConsoleArgs_t* parsedArgs);
void ConsoleInputTask(void *pvParameters);
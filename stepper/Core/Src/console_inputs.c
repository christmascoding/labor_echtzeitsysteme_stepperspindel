#include <console_inputs.h>

// Function to parse console input into key-value pairs
int ParseConsoleInput(const char* input, ConsoleArgs_t* parsedArgs) {
    if (!input || !parsedArgs) return -1;


    return 0;
}

void ConsoleInputTask(void *pvParameters)
{
    // Allocate memory for the StepperTaskArgs_t structure
    ConsoleArg_t* ConsoleInputArgs = pvPortMalloc(sizeof(ConsoleArg_t));
    if (!ConsoleInputArgs) {
        printf("Failed to allocate memory for ConsoleArg_t\r\n");
        Error_Handler();
    }
    else {
        printf("console input instance created\r\n");
    }

    while(1) {
    }
    
    vPortFree(ConsoleInputArgs);
    vTaskDelete(NULL);
}
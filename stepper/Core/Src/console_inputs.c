#include <console_inputs.h>

int CapabilityFunc(int argc, char** argv, void* ctx)
{
    printf("sdfndsjkfcndsjkcnksjd");
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

    ConsoleHandle_t c = CONSOLE_CreateInstance( 4*configMINIMAL_STACK_SIZE, configMAX_PRIORITIES - 5  );

    if (CONSOLE_RegisterCommand(c, "spindle", "Moves the spindle", SpindleConsoleFunction, NULL) == 0) {
      printf("Spindle command registered successfully.\n");
    } else {
      printf("Failed to register spindle command.\n");
    }

    CONSOLE_RegisterCommand(c, "capability", "prints a specified string of capability bits",
    CapabilityFunc, NULL);

    while(1) {
    }
    
    vPortFree(ConsoleInputArgs);
    vTaskDelete(NULL);
}
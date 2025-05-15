#include "customspindel.h"
#include "main.h"


extern ConsoleHandle_t c;

typedef struct{
	int direction;
	TIM_HandleTypeDef *pwm;
}SpindleContext;

extern TIM_HandleTypeDef htim2;
extern ConsoleHandle_t c;

void SpindleSetDirection(SpindleHandle_t h, void* context, int direction){
	(void)h;
	SpindleContext* ctx = (SpindleContext*) context;
	ctx->direction = direction;
}

void SpindleSetDutyCycle(SpindleHandle_t h, void* context, float dutyCycle){
	(void)h;
	SpindleContext* ctx = (SpindleContext*) context;

	int ar = TIM2->ARR;

	if(ctx->direction){
		TIM2->CCR3 = 0;
		TIM2->CCR4 = (int)((float)ar * dutyCycle);
	}
	else{
		TIM2->CCR3 = (int)((float)ar * dutyCycle);
		TIM2->CCR4 = 0;
	}
}
void SpindleEnaPWM(SpindleHandle_t h, void* context, int ena){
	(void)h;
	SpindleContext* ctx = (SpindleContext*) context;

	if(ena){
		HAL_TIM_PWM_Start(ctx->pwm, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(ctx->pwm, TIM_CHANNEL_4);
		HAL_GPIO_WritePin(SPINDLE_ENA_L_GPIO_Port, SPINDLE_ENA_L_Pin, 1);
		HAL_GPIO_WritePin(SPINDLE_ENA_R_GPIO_Port, SPINDLE_ENA_R_Pin, 1);
	}
	else{
		HAL_TIM_PWM_Stop(ctx->pwm, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(ctx->pwm, TIM_CHANNEL_4);
		HAL_GPIO_WritePin(SPINDLE_ENA_L_GPIO_Port, SPINDLE_ENA_L_Pin, 0);
		HAL_GPIO_WritePin(SPINDLE_ENA_R_GPIO_Port, SPINDLE_ENA_R_Pin, 0);
	}
}


void SpindleTask(void* pvParameters){
	SpindleContext ctx;

	ctx.direction = 0;
	ctx.pwm = &htim2;
	SpindlePhysicalParams_t s;
	s.maxRPM             =  9000.0f;
	s.minRPM             = -9000.0f;
	s.absMinRPM          =  1600.0f;
	s.setDirection       = SpindleSetDirection;
	s.setDutyCycle       = SpindleSetDutyCycle;
	s.enaPWM             = SpindleEnaPWM;
	s.context            = &ctx;
	SPINDLE_CreateInstance( 4*configMINIMAL_STACK_SIZE, configMAX_PRIORITIES - 3, c, &s);
}
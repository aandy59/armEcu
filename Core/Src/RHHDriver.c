#include "RHHDriver.h"
#include "stm32f4xx_ll_gpio.h"
#include "cmsis_os2.h"

#define RHHDriverEnPinState (GPIOB->ODR & GPIO_ODR_OD3)
#define RHHDriverEnable LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_3) //драйвер РХХ включен/выключен
#define RHHDriverDisable LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_3)

#define RHHDriverDirPinState (GPIOB->ODR & GPIO_ODR_OD5)
#define RHHDriverDir1 LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_5) //драйвер РХХ напрваление
#define RHHDriverDir0 LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_5)

#define RHHDriverStepPinState (GPIOB->ODR & GPIO_ODR_OD4)
#define RHHDriverStepOn LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_4) //драйвер РХХ напрваление
#define RHHDriverStepOff LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_4)

#define MAX_SPEED 1000

struct state getCurrentState(){
	struct state currentState;
	
	currentState.powerState = RHHDriverEnPinState;
	currentState.currentDir = RHHDriverDirPinState;
		
	return currentState;
}
//dir - направление, count - количество шагов, speed - количество шагов в секунду
void stepRHH(bool dir, uint16_t count, uint16_t speed){
	struct state currentState = getCurrentState();
	if(speed > MAX_SPEED){ speed = MAX_SPEED;}
	
		if(dir == 1){
			RHHDriverDir1;
		}else{
			RHHDriverDir0;
		}
	
	RHHDriverEnable;
	for(uint16_t i = 0; i < count; i++){
		RHHDriverStepOn;
		osDelay(1);
		RHHDriverStepOff;
		osDelay(1000/speed);
	}
	RHHDriverStepOff;
	RHHDriverDisable;
}

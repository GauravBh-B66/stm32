/*
 * stm32f303re_gpioDriver.h
 *
 *  Created on: Apr 17, 2024
 *      Author: user_gs
*/

#include "stm32f303re.h"

#ifndef INCLUDE_STM32F303RE_GPIODRIVER_H_
#define INCLUDE_STM32F303RE_GPIODRIVER_H_



#endif /* INCLUDE_STM32F303RE_GPIODRIVER_H_ */

typedef struct {
    uint8_t     pinNumber;
    uint8_t     pinMode;
    uint8_t     pinOutputType;
    uint8_t     pinSpeed;
    uint8_t     pinPullUpDown;
    uint16_t    pinAlternateFunction;
}GPIO_Params_t;


typedef struct{
    GPIO_RegDef_t *pGPIOx;
    GPIO_Params_t paramsGpio;
}GPIO_Handle_t;

//Clock control
void gpioClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en_di);


//Initialize and Deinitialize
void gpioInit(GPIO_Handle_t *pGpioHandle);
void gpioDeinit(GPIO_RegDef_t *pGPIOx);


//Read and write operations
uint8_t gpioPinRead(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t gpioPortRead(GPIO_RegDef_t *pGPIOx);
void gpioPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t set_reset);
void gpioPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t set_reset);
void gpioPinToggle(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

//Interrupt Handling
void gpioIRQConfig(uint8_t irqNumber, uint8_t priority, uint8_t en_di);
void gpoiIRQHandle(uint8_t pinNumber);

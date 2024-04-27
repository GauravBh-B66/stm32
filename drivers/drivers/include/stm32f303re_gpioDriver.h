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

/*
Clock control function
Parameters:
    pGPIOx: Holds address of the GPIO peripheral requested.
    en_di:  1 = Enable, 0 = Disable   
Returns void
*/
void gpioClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en_di);

/*
Initialize the requested GPIO
Parameters
    pGpioHandle: pointer to the GPIO_Handle_t datatype
Returns void
*/
void gpioInit(GPIO_Handle_t *pGpioHandle);

/*
Deinitialize the requested GPIO.
There is a special register dedicated to reset all the registers corresponding to the GPIO.
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested.
Returns void
*/
void gpioDeinit(GPIO_RegDef_t *pGPIOx);


/*
Read pin
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested
    pinNumber: Selected pin
*/
uint8_t gpioPinRead(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);


/*
Read port
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested
*/
uint16_t gpioPortRead(GPIO_RegDef_t *pGPIOx);


/*
Write to pin
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested
    pinNumber: Selected pin
    set_reset: 1 = Set, 0 = Reset
*/
void gpioPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t set_reset);


/*
Write to port
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested
    set_reset: 1 = Set, 0 = Reset
*/
void gpioPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t set_reset);


/*
Toggle requested pin
Parameters
    pGPIOx: Holds address of the GPIO peripheral requested
    pinNumber: Selected pin
*/
void gpioPinToggle(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

//Interrupt Handling
void gpioIRQConfig(uint8_t irqNumber, uint8_t priority, uint8_t en_di);
void gpoiIRQHandle(uint8_t pinNumber);

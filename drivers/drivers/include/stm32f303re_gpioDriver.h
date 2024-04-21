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
/*
 * stm32f303re_gpioDriver.c
 *
 *  Created on: Apr 17, 2024
 *      Author: user_gs
 */


#include "../include/stm32f303re_gpioDriver.h"

void gpioClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en_di){
    if (en_di == ENABLE){
        if (pGPIOx == pGPIOA) { GPIOA_CLOCK_EN(); }
        else if (pGPIOx == pGPIOB) { GPIOB_CLOCK_EN(); }
        else if (pGPIOx == pGPIOC) { GPIOC_CLOCK_EN(); }
        else if (pGPIOx == pGPIOD) { GPIOD_CLOCK_EN(); }
        else if (pGPIOx == pGPIOE) { GPIOE_CLOCK_EN(); }
        else if (pGPIOx == pGPIOF) { GPIOF_CLOCK_EN(); }
        else if (pGPIOx == pGPIOG) { GPIOG_CLOCK_EN(); }
        else if (pGPIOx == pGPIOH) { GPIOH_CLOCK_EN(); }
    }
    else{
        if (pGPIOx == pGPIOA) { GPIOA_CLOCK_DI(); }
        else if (pGPIOx == pGPIOB) { GPIOB_CLOCK_DI(); }
        else if (pGPIOx == pGPIOC) { GPIOC_CLOCK_DI(); }
        else if (pGPIOx == pGPIOD) { GPIOD_CLOCK_DI(); }
        else if (pGPIOx == pGPIOE) { GPIOE_CLOCK_DI(); }
        else if (pGPIOx == pGPIOF) { GPIOF_CLOCK_DI(); }
        else if (pGPIOx == pGPIOG) { GPIOG_CLOCK_DI(); }
        else if (pGPIOx == pGPIOH) { GPIOH_CLOCK_DI(); }
    }
}
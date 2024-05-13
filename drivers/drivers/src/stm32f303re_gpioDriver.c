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

void gpioInit(GPIO_Handle_t *pGpioHandle){

    uint32_t tempReg;

    //Configure modes
    tempReg = 0;
    tempReg = (pGpioHandle->paramsGpio.pinMode << (2*pGpioHandle->paramsGpio.pinNumber));

    pGpioHandle->pGPIOx->MODER &= ~(0x3 <<(2*pGpioHandle->paramsGpio.pinNumber));
    pGpioHandle->pGPIOx->MODER |= tempReg;

    //Configure the speed
    tempReg = 0;
    tempReg = (pGpioHandle->paramsGpio.pinSpeed) << (2*pGpioHandle->paramsGpio.pinNumber);

    pGpioHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2*pGpioHandle->paramsGpio.pinNumber));
    pGpioHandle->pGPIOx->OSPEEDR |= tempReg;
    
    //Configure pull up/down resistors.
    tempReg = 0;
    tempReg = (pGpioHandle->paramsGpio.pinPullUpDown) << (2*pGpioHandle->paramsGpio.pinNumber);
    
    pGpioHandle->pGPIOx->PUPDOWNR &= ~(0x3 <<(2*pGpioHandle->paramsGpio.pinNumber));
    pGpioHandle->pGPIOx->PUPDOWNR |= tempReg;

    //Configure the output type
    tempReg = 0;
    tempReg = (pGpioHandle->paramsGpio.pinOutputType) << (pGpioHandle->paramsGpio.pinNumber);

    pGpioHandle->pGPIOx->OTYPER &= ~(0x1 << pGpioHandle->paramsGpio.pinNumber);
    pGpioHandle->pGPIOx->OTYPER |= tempReg;   
    
    //Configure the alternate functionality
    tempReg = 0;
    if(pGpioHandle->paramsGpio.pinMode == MODE_ALT){
        uint8_t reg, position;
        reg = pGpioHandle->paramsGpio.pinNumber / 8;
        position = pGpioHandle->paramsGpio.pinNumber % 8;

        tempReg = pGpioHandle->paramsGpio.pinAlternateFunction << (4*position);

        
        pGpioHandle->pGPIOx->AFR[reg] &= ~(0xf << (4*position));
        pGpioHandle->pGPIOx->AFR[reg] |= tempReg;
        tempReg = 0;
    } 
}

void gpioDeinit(GPIO_RegDef_t *pGPIOx){
        if (pGPIOx == pGPIOA)      { GPIOA_PORT_RESET(); }
        else if (pGPIOx == pGPIOB) { GPIOB_PORT_RESET(); }
        else if (pGPIOx == pGPIOC) { GPIOC_PORT_RESET(); }
        else if (pGPIOx == pGPIOD) { GPIOD_PORT_RESET(); }
        else if (pGPIOx == pGPIOE) { GPIOE_PORT_RESET(); }
        else if (pGPIOx == pGPIOF) { GPIOF_PORT_RESET(); }
        else if (pGPIOx == pGPIOG) { GPIOG_PORT_RESET(); }
        else if (pGPIOx == pGPIOH) { GPIOH_PORT_RESET(); }
}

uint8_t gpioPinRead(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
    uint8_t data;
    data = (uint8_t)(((pGPIOx->IDR) >> pinNumber) & 0x1);
    return data;
}

uint16_t gpioPortRead(GPIO_RegDef_t *pGPIOx){
    uint16_t data;
    data = (uint16_t)((pGPIOx->IDR) & 0xffff);
    return data;
}


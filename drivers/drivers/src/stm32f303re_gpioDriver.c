/*
 * stm32f303re_gpioDriver.c
 *
 *  Created on: Apr 17, 2024
 *      Author: user_gs
 */


#include "../include/stm32f303re_gpioDriver.h"
#include "../include/stm32f303re.h"


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

    if (pGpioHandle->paramsGpio.pinMode <= MODE_ANALOG){
        //These are the non-interrupt modes. 
        tempReg = 0;
        tempReg = (pGpioHandle->paramsGpio.pinMode << (2*pGpioHandle->paramsGpio.pinNumber));

        pGpioHandle->pGPIOx->MODER &= ~(0x3 <<(2*pGpioHandle->paramsGpio.pinNumber));
        pGpioHandle->pGPIOx->MODER |= tempReg;
    }
    else{
        //These are the interrupt modes.

        //Set the bits corresponding to Rising or Falling edge trigger for given interrupt line.
        if (pGpioHandle->paramsGpio.pinMode == MODE_IT_FT){
            //Set the falling edge trigger for corresponding interrupt line 
            //Reset the rising edge trigger for corresponding interrupt line 
            pEXTI->EXTI_FTSR1 |= (1 << (pGpioHandle->paramsGpio.pinNumber));
            pEXTI->EXTI_RTSR1 &= ~(1 << (pGpioHandle->paramsGpio.pinNumber));
        }
        else if (pGpioHandle->paramsGpio.pinMode == MODE_IT_RT){ 
            //Set the rising edge trigger for corresponding interrupt line 
            //Reset the falling edge trigger for corresponding interrupt line 
            pEXTI->EXTI_RTSR1 |= (1 << (pGpioHandle->paramsGpio.pinNumber));
            pEXTI->EXTI_FTSR1 &= ~(1 << (pGpioHandle->paramsGpio.pinNumber));
        }
        else if (pGpioHandle->paramsGpio.pinMode == MODE_IT_RFT){
            //Set the rising edge trigger for corresponding interrupt line 
            //Set the falling edge trigger for corresponding interrupt line 
            pEXTI->EXTI_RTSR1 |= (1 << (pGpioHandle->paramsGpio.pinNumber));
            pEXTI->EXTI_FTSR1 |= (1 << (pGpioHandle->paramsGpio.pinNumber));
        }

        //Configure SYSCONFIG EXTICR register according to the port and pin specified
        uint8_t regNum, regPos;
        regNum = (pGpioHandle->paramsGpio.pinNumber) / 4; 
        regPos = ((pGpioHandle->paramsGpio.pinNumber) % 4)*4;
        SYSCFG_CLOCK_EN();
        if (pGpioHandle->pGPIOx == pGPIOA){
            pSYSCFG->SYSCFG_EXTICR[regNum] = PORT_CODE(pGpioHandle->pGPIOx) << regPos;  
        }


        //Unmask the interrupt i.e instruct the GPIO to generate an interrupt when triggered.
        pEXTI->EXTI_IMR1 |= (1 << (pGpioHandle->paramsGpio.pinNumber));
    }

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

void gpioPinWrite(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t set_reset){
    if (set_reset == ENABLE){pGPIOx->ODR |= (1 << pinNumber);}
    else{pGPIOx->ODR &= ~(1 << pinNumber);}
}

void gpioPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t value){
    pGPIOx->ODR = value;
}

void gpioPinToggle(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
    pGPIOx->ODR ^= (1 << pinNumber);
}

void gpioIRQConfig(uint8_t irqNumber, uint8_t priority, uint8_t en_di){
    
    //Enabling/disabling the interrupt in processor side based on the input from user.
    if (en_di == ENABLE){
        if (irqNumber <=31){
            *NVIC_ISER0 |= 1 << irqNumber;
        }
        else if (irqNumber <=63){
            *NVIC_ISER1 |= 1 << (irqNumber % 32);
        }
        else if (irqNumber <=95){
            *NVIC_ISER2 |= 1 << (irqNumber % 64);
        }
    }
    else {
        if (irqNumber <=31){
            *NVIC_ICER0 |= 1 << irqNumber;
        }
        else if (irqNumber <=63){
            *NVIC_ICER1 |= 1 << (irqNumber % 32);
        }
        else if (irqNumber <=95){
            *NVIC_ICER2 |= 1 << (irqNumber % 64);
        }
    }

    //Configuration of the priority number
    uint8_t regNum, regPos;
    regNum = irqNumber / 4;
    regPos = (irqNumber % 4)*8;

    *(NVIC_IPR + regNum) |= (priority << (regPos + PRIORITY_BITS_NOT_IMPLEMENTED));
}

void gpoiIRQHandle(uint8_t pinNumber){
    //Clear the Pending Request (PR) bit in EXTi_PR register.
    //The register documentation suggests writing 1 into the bit position to clear the pending request.
    if (pEXTI->EXTI_PR1 & (1 << pinNumber)){
        pEXTI->EXTI_PR1 |= (1 << pinNumber);
    }
}






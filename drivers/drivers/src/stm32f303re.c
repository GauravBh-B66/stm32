/*
 * stm32f303re.c
 *
 *  Created on: May 18, 2024
 *      Author: user_gs
 */
#include "../include/stm32f303re.h"

    GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*)(GPIOA_BASEADDRESS);
    GPIO_RegDef_t *pGPIOB = (GPIO_RegDef_t*)(GPIOB_BASEADDRESS);
    GPIO_RegDef_t *pGPIOC = (GPIO_RegDef_t*)(GPIOC_BASEADDRESS);
    GPIO_RegDef_t *pGPIOD = (GPIO_RegDef_t*)(GPIOD_BASEADDRESS);
    GPIO_RegDef_t *pGPIOE = (GPIO_RegDef_t*)(GPIOE_BASEADDRESS);
    GPIO_RegDef_t *pGPIOF = (GPIO_RegDef_t*)(GPIOF_BASEADDRESS);
    GPIO_RegDef_t *pGPIOG = (GPIO_RegDef_t*)(GPIOG_BASEADDRESS);
    GPIO_RegDef_t *pGPIOH = (GPIO_RegDef_t*)(GPIOH_BASEADDRESS);

    uint8_t PORT_CODE(GPIO_RegDef_t* pGPIOx){
        if (pGPIOx == pGPIOA)      {return 0;}
        else if (pGPIOx == pGPIOB) {return 1;}
        else if (pGPIOx == pGPIOC) {return 2;}
        else if (pGPIOx == pGPIOD) {return 3;}
        else if (pGPIOx == pGPIOE) {return 4;}
        else if (pGPIOx == pGPIOF) {return 5;}
        else if (pGPIOx == pGPIOG) {return 6;}
        else                       {return 0;}
    }

    SpiRegDef_t* pSPI1 = (SpiRegDef_t*)(SPI1_BASEADDRESS);
    SpiRegDef_t* pSPI2 = (SpiRegDef_t*)(SPI2_BASEADDRESS);
    SpiRegDef_t* pSPI3 = (SpiRegDef_t*)(SPI3_BASEADDRESS);
    SpiRegDef_t* pSPI4 = (SpiRegDef_t*)(SPI4_BASEADDRESS);

    RccRegDef_t* pRCC = (RccRegDef_t*)(RCC_BASEADDRESS);

    ExtiRegDef_t* pEXTI = (ExtiRegDef_t*)(EXTI_BASEADDRESS);
    SyscfgRegDef_t* pSYSCFG = (SyscfgRegDef_t*)(SYSCFG_BASEADDRESS);

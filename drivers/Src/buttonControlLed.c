/*
 * ledToggle.c
 *
 *  Created on: May 20, 2024
 *      Author: user_gs
 */

#include "../drivers/include/stm32f303re_gpioDriver.h"

void delay(){
    for (int i = 0; i < 10000; i++);
}

int main(){
    GPIO_Handle_t ledHandle, buttonHandle;

    ledHandle.pGPIOx = pGPIOA;
    ledHandle.paramsGpio.pinNumber     = GPIO_PIN_5;
    ledHandle.paramsGpio.pinMode       = MODE_OUTPUT;
    ledHandle.paramsGpio.pinOutputType = OTYPE_PUSH_PULL;
    ledHandle.paramsGpio.pinSpeed      = O_SPEED_LOW;
    ledHandle.paramsGpio.pinPullUpDown = PULL_DOWN_EN;
   
    buttonHandle.pGPIOx = pGPIOC;
    buttonHandle.paramsGpio.pinNumber     = GPIO_PIN_13;
    buttonHandle.paramsGpio.pinMode       = MODE_INPUT;
    buttonHandle.paramsGpio.pinSpeed      = O_SPEED_LOW;
    buttonHandle.paramsGpio.pinPullUpDown = DISABLE_PUP_PDOWN;

    gpioClockControl(pGPIOA, ENABLE);
    gpioClockControl(pGPIOC, ENABLE);
    gpioInit(&ledHandle);
    gpioInit(&buttonHandle);

    while(1){
        if (gpioPinRead(pGPIOC, GPIO_PIN_13) == RESET){
        	delay();
            gpioPinToggle(pGPIOA, GPIO_PIN_5);
        }

        delay();
    }


}

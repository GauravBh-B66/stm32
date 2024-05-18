/*
 * ledToggle.c
 *
 *  Created on: May 18, 2024
 *      Author: user_gs
 */

#include "../drivers/include/stm32f303re_gpioDriver.h"

void delay(){
    for (int i = 0; i < 500000; i++);
}

int main(){
    GPIO_Handle_t gpioHandle;

    gpioHandle.pGPIOx = pGPIOA;
    gpioHandle.paramsGpio.pinNumber     = GPIO_PIN_5;
    gpioHandle.paramsGpio.pinMode       = MODE_OUTPUT;
    gpioHandle.paramsGpio.pinOutputType = OTYPE_PUSH_PULL;
    gpioHandle.paramsGpio.pinSpeed      = O_SPEED_LOW;
    gpioHandle.paramsGpio.pinPullUpDown = PULL_DOWN_EN;

    gpioClockControl(pGPIOA, ENABLE);
    gpioInit(&gpioHandle);

    while(1){
        gpioPinToggle(pGPIOA, GPIO_PIN_5);
        delay();
    }


}

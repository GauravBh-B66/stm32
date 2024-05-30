/*
 * stm32f303re_gpioDriver.h
 *
 *  Created on: Apr 17, 2024
 *      Author: user_gs
*/


#ifndef INCLUDE_STM32F303RE_GPIODRIVER_H_
#define INCLUDE_STM32F303RE_GPIODRIVER_H_



#include "stm32f303re.h"

//Port Codes
#define CODE_A  0
#define CODE_B  1
#define CODE_C  2
#define CODE_D  3
#define CODE_E  4
#define CODE_F  5
#define CODE_G  6


//GPIO pin definitions
#define GPIO_PIN_0  0
#define GPIO_PIN_1  1
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_4  4
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_9  9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

//GPIO modes
#define MODE_INPUT  0   //Non interrupt
#define MODE_OUTPUT 1   //Non interrupt
#define MODE_ALT    2   //Non interrupt
#define MODE_ANALOG 3   //Non interrupt
#define MODE_IT_FT  4   //Interrupt
#define MODE_IT_RT  5   //Interrupt
#define MODE_IT_RFT 6   //Interrupt

//Output Type
#define OTYPE_PUSH_PULL     0
#define OTYPE_OPEN_DRAIN    1

//Output Speed
#define O_SPEED_LOW     0
#define O_SPEED_MEDIUM  1
#define O_SPEED_HIGH    3

//Pull up/down resistors
#define DISABLE_PUP_PDOWN   0
#define PULL_UP_EN          1
#define PULL_DOWN_EN        2  


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
    value: value to be written into the port
*/
void gpioPortWrite(GPIO_RegDef_t *pGPIOx, uint16_t value);


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

#endif /* INCLUDE_STM32F303RE_GPIODRIVER_H_ */


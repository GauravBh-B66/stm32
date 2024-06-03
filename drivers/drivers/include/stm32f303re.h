/*
 * stm32f303re.h
 *
 *  Created on: Mar 9, 2024
 *  Author: Gaurav Bhattarai
*/
#include <stdint.h>

#ifndef INCLUDE_STM32F303RE_H_
#define INCLUDE_STM32F303RE_H_

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0

//ARM Cortex-M4 specific NVIC Interrupt Set Enable registers (ISER)
#define NVIC_ISER0  (volatile uint32_t*) 0xE000E100U
#define NVIC_ISER1  (volatile uint32_t*) 0xE000E104U
#define NVIC_ISER2  (volatile uint32_t*) 0xE000E108U

//ARM Cortex-M4 specific NVIC Interrupt Clear Enable registers (ICER)
#define NVIC_ICER0  (volatile uint32_t*) 0xE000E180U
#define NVIC_ICER1  (volatile uint32_t*) 0xE000E184U
#define NVIC_ICER2  (volatile uint32_t*) 0xE000E188U

////ARM Cortex-M4 specific NVIC Interrupt Priority registers (IPR)
#define NVIC_IPR    (volatile uint32_t*) 0xE000E400
#define PRIORITY_BITS_NOT_IMPLEMENTED   4


//Addresses of memory components
    //Flash Memory (512KB)          (0x0800 0000 - 0x0807 FFFF)
    #define FLASH_BASEADDRESS       0x08000000U
    //ROM(or system memory) (8KB)   (0x1FFF D800 - 0x1FFF F7FF)
    #define ROM_BASEADDRESS         0x1FFFD800U

    //SRAM (64KB)                   (0x2000 0000 - 0x2000 9FFF)
    #define SRAM_BASEADDRESS        0x20000000U

    //CCM RAM (16KB)                (0x1000 0000 - 0x1000 3FFF)
    #define CCM_SRAM_BASEADDRESS    0x10000000U

//Addresses of buses
    #define AHB1_BASEADDRESS 0x40020000U   // - 0x4002 03FF
    #define AHB2_BASEADDRESS 0x48000000U   // - 0x4800 03FF
    #define AHB3_BASEADDRESS 0x50000000U   // - 0x5000 03FF
    #define AHB4_BASEADDRESS 0x60000000U   // - 0x7FFF FFFF

    #define APB1_BASEADDRESS 0x40000000U   // - 0x4000 03FF
    #define APB2_BASEADDRESS 0x40010000U   // - 0x4001 03FF

//In case of STM32F303RE, the peripheral memory block starts from APB1
    #define PERIPHERAL_BASEADDRESS  APB1_BASEADDRESS

//Addresses of peripherals
    #define DMA1_BASEADDRESS    (AHB1_BASEADDRESS + 0x0000)
    #define DMA2_BASEADDRESS    (AHB1_BASEADDRESS + 0x0400)
    #define RCC_BASEADDRESS     (AHB1_BASEADDRESS + 0x1000)
    #define FLASH_BASEADDRESS   (AHB1_BASEADDRESS + 0x2000)
    #define CRC_BASEADDRESS     (AHB1_BASEADDRESS + 0x3000)
    #define TSC_BASEADDRESS     (AHB1_BASEADDRESS + 0x4000)

    #define GPIOA_BASEADDRESS   (AHB2_BASEADDRESS + 0x0000)
    #define GPIOB_BASEADDRESS   (AHB2_BASEADDRESS + 0x0400)
    #define GPIOC_BASEADDRESS   (AHB2_BASEADDRESS + 0x0800)
    #define GPIOD_BASEADDRESS   (AHB2_BASEADDRESS + 0x0C00)
    #define GPIOE_BASEADDRESS   (AHB2_BASEADDRESS + 0x1000)
    #define GPIOF_BASEADDRESS   (AHB2_BASEADDRESS + 0x1400)
    #define GPIOG_BASEADDRESS   (AHB2_BASEADDRESS + 0x1800)
    #define GPIOH_BASEADDRESS   (AHB2_BASEADDRESS + 0x1C00)

    #define TIM2_BASEADDRESS                (APB1_BASEADDRESS + 0x0000)
    #define TIM3_BASEADDRESS                (APB1_BASEADDRESS + 0x0400)
    #define TIM4_BASEADDRESS                (APB1_BASEADDRESS + 0x0800)
    #define TIM6_BASEADDRESS                (APB1_BASEADDRESS + 0x1000)
    #define TIM7_BASEADDRESS                (APB1_BASEADDRESS + 0x1400)
    #define RTC_BASEADDRESS                 (APB1_BASEADDRESS + 0x2800)
    #define WWDG_BASEADDRESS                (APB1_BASEADDRESS + 0x2C00)
    #define IWDG_BASEADDRESS                (APB1_BASEADDRESS + 0x3000)
    #define I2S2EXT_BASEADDRESS             (APB1_BASEADDRESS + 0x3400)
    #define I2S3EXT_BASEADDRESS             (APB1_BASEADDRESS + 0x4000)
    #define I2S2_BASEADDRESS                (APB1_BASEADDRESS + 0x3800)
    #define I2S3_BASEADDRESS                (APB1_BASEADDRESS + 0x3C00)
    #define SPI2_BASEADDRESS                (APB1_BASEADDRESS + 0x3800)
    #define SPI3_BASEADDRESS                (APB1_BASEADDRESS + 0x3C00)
    #define USART2_BASEADDRESS              (APB1_BASEADDRESS + 0x4400)
    #define USART3_BASEADDRESS              (APB1_BASEADDRESS + 0x4800)
    #define UART4_BASEADDRESS               (APB1_BASEADDRESS + 0x4C00)
    #define UART5_BASEADDRESS               (APB1_BASEADDRESS + 0x5000)
    #define I2C1_BASEADDRESS                (APB1_BASEADDRESS + 0x5400)
    #define I2C2_BASEADDRESS                (APB1_BASEADDRESS + 0x5800)
    #define I2C3_BASEADDRESS                (APB1_BASEADDRESS + 0x7800)
    #define USB_DEVICE_FS_BASEADDRESS       (APB1_BASEADDRESS + 0x5C00)
    #define USB_SRAM_BASEADDRESS            (APB1_BASEADDRESS + 0x6000)
    #define CAN_SRAM_BASEADDRESS            (APB1_BASEADDRESS + 0x6000)
    #define PWR_BASEADDRESS                 (APB1_BASEADDRESS + 0x7000)
    #define DAC1_BASEADDRESS                (APB1_BASEADDRESS + 0x7400)
    #define BXCAN_BASEADDRESS               (APB1_BASEADDRESS + 0x6400)

    #define SYSCFG_BASEADDRESS              (APB2_BASEADDRESS + 0x0000)
    #define COMP_BASEADDRESS                (APB2_BASEADDRESS + 0x0000)
    #define OPAMP_BASEADDRESS               (APB2_BASEADDRESS + 0x0000)
    #define EXTI_BASEADDRESS                (APB2_BASEADDRESS + 0x0400)
    #define TIM1_BASEADDRESS                (APB2_BASEADDRESS + 0x2C00)
    #define SPI1_BASEADDRESS                (APB2_BASEADDRESS + 0x3000)
    #define TIM8_BASEADDRESS                (APB2_BASEADDRESS + 0x3400)
    #define USART1_BASEADDRESS              (APB2_BASEADDRESS + 0x3800)
    #define SPI4_BASEADDRESS                (APB2_BASEADDRESS + 0x3C00)
    #define TIM15_BASEADDRESS               (APB2_BASEADDRESS + 0x4000)
    #define TIM16_BASEADDRESS               (APB2_BASEADDRESS + 0x4400)
    #define TIM17_BASEADDRESS               (APB2_BASEADDRESS + 0x4800)
    #define TIM20_BASEADDRESS               (APB2_BASEADDRESS + 0x5000)

    //ADC1 and ADC3 are master.
    //ADC2 and ADC4 are slaves.
    #define ADC1_BASEADDRESS               (AHB3_BASEADDRESS + 0x0000)
    #define ADC2_BASEADDRESS               (AHB3_BASEADDRESS + 0x0000)
    #define ADC3_BASEADDRESS               (AHB3_BASEADDRESS + 0x0400)
    #define ADC4_BASEADDRESS               (AHB3_BASEADDRESS + 0x0400)


    //Interrupt Request Numbers (From Vector Table)
    #define IRQNUMBER_EXTI0     6
    #define IRQNUMBER_EXTI1     7
    #define IRQNUMBER_EXTI2     8
    #define IRQNUMBER_EXTI3     9
    #define IRQNUMBER_EXTI4     10
    #define IRQNUMBER_EXTI5_9   23
    #define IRQNUMBER_EXTI10_15 40


    //Registers related to GPIOs
    typedef struct{
        volatile uint32_t MODER;        //Port mode register
        volatile uint32_t OTYPER;       //Output type register
        volatile uint32_t OSPEEDR;      //Output speed register
        volatile uint32_t PUPDOWNR;     //Pull-up/pull down register
        volatile uint32_t IDR;          //Input data register
        volatile uint32_t ODR;          //Output data register
        volatile uint32_t BSRR;         //Bit set/reset register
        volatile uint32_t LCKR;         //Configuration lock register
        volatile uint32_t AFR[2];       //Alternate function register
        volatile uint32_t BRR;          //Port bit reset register
    }GPIO_RegDef_t;
    
    extern GPIO_RegDef_t *pGPIOA ;
    extern GPIO_RegDef_t *pGPIOB ;
    extern GPIO_RegDef_t *pGPIOC ;
    extern GPIO_RegDef_t *pGPIOD ;
    extern GPIO_RegDef_t *pGPIOE ;
    extern GPIO_RegDef_t *pGPIOF ;
    extern GPIO_RegDef_t *pGPIOG ;
    extern GPIO_RegDef_t *pGPIOH ;

    extern uint8_t PORT_CODE(GPIO_RegDef_t* );  

    //Registers required for SPI communication:
    typedef struct{
        volatile uint32_t CR1;          //control register 1
        volatile uint32_t CR2;          //control register 2
        volatile uint32_t SR;           //status register
        volatile uint32_t DR;           //data register
        volatile uint32_t CRCPR;        //crc polynomial register
        volatile uint32_t RXCRCR;       //rx crc polynomial register
        volatile uint32_t TXCRCR;       //tx crc polynomial register
        volatile uint32_t I2SCFGR;      //i2s configuration register
        volatile uint32_t I2SPR;        //i2s prescaler register
    }SpiRegDef_t; 

    extern SpiRegDef_t* pSPI1;
    extern SpiRegDef_t* pSPI2;
    extern SpiRegDef_t* pSPI3;
    extern SpiRegDef_t* pSPI4;


    //Registers required for configuring RCC
    typedef struct{
        volatile uint32_t RCC_CR;           //clock control register
        volatile uint32_t RCC_CFGR;         //clock configuration register
        volatile uint32_t RCC_CIR;          //clock interrupt register
        volatile uint32_t RCC_APB2RSTR;     //apb2 peripheral reset register
        volatile uint32_t RCC_APB1RSTR;     //apb1 peripheral reset register
        volatile uint32_t RCC_AHBENR;       //ahb peripheral clock enable register
        volatile uint32_t RCC_APB2ENR;      //apb2 peripheral clock enable register
        volatile uint32_t RCC_APB1ENR;      //apb1 peripheral clock enable register
        volatile uint32_t RCC_BDCR;         //rtc domain control register
        volatile uint32_t RCC_CSR;          //control/status register
        volatile uint32_t RCC_AHBRSTR;      //ahb peripheral reset register
        volatile uint32_t RCC_CFGR2;        //clock configuration register 2
        volatile uint32_t RCC_CFGR3;        //clock configuration register 1
    }RccRegDef_t;
    extern RccRegDef_t* pRCC;

    //Registers required for configuring EXTI peripheral
    typedef struct {
        volatile uint32_t EXTI_IMR1;
        volatile uint32_t EXTI_EMR1;
        volatile uint32_t EXTI_RTSR1;
        volatile uint32_t EXTI_FTSR1;
        volatile uint32_t EXTI_SWIER1;
        volatile uint32_t EXTI_PR1;
        volatile uint32_t EXTI_IMR2;
        volatile uint32_t EXTI_EMR2;
        volatile uint32_t EXTI_RTSR2;
        volatile uint32_t EXTI_FTSR2;
        volatile uint32_t EXTI_SWIER2;
        volatile uint32_t EXTI_PR2;
    }ExtiRegDef_t;
    extern ExtiRegDef_t* pEXTI;


    //Registers required for configuring SYSCFG registers
    typedef struct {
        volatile uint32_t SYSCFG_CFGR1;
        volatile uint32_t SYSCFG_RCR;
        volatile uint32_t SYSCFG_EXTICR[4];
        volatile uint32_t SYSCFG_CFGR2;
                 uint32_t  RESERVED[13];
        volatile uint32_t SYSCFG_CFGR3;
        volatile uint32_t SYSCFG_CFGR4;
    }SyscfgRegDef_t;
    extern SyscfgRegDef_t* pSYSCFG;

    //Clock enable and disable macros for SYSCFG
    #define SYSCFG_CLOCK_EN()      (pRCC->RCC_APB2ENR |= (1 << 0));
    #define SYSCFG_CLOCK_DI()      (pRCC->RCC_APB2ENR &= ~(1 << 0));


    //Clock enable and disable macros for GPIO
    #define GPIOA_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 17));
    #define GPIOB_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 18));
    #define GPIOC_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 19));
    #define GPIOD_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 20));
    #define GPIOE_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 21));
    #define GPIOF_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 22));
    #define GPIOG_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 23));
    #define GPIOH_CLOCK_EN()      (pRCC->RCC_AHBENR |= (1 << 16));

    #define GPIOA_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 17)));
    #define GPIOB_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 18)));
    #define GPIOC_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 19)));
    #define GPIOD_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 20)));
    #define GPIOE_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 21)));
    #define GPIOF_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 22)));
    #define GPIOG_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 23)));
    #define GPIOH_CLOCK_DI()      (pRCC->RCC_AHBENR &= (~(1 << 16)));

    //GPIO Reset Macros
    #define GPIOA_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 17); pRCC->RCC_AHBRSTR &= ~(1 << 17);} while(0)
    #define GPIOB_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 18); pRCC->RCC_AHBRSTR &= ~(1 << 18);} while(0)
    #define GPIOC_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 19); pRCC->RCC_AHBRSTR &= ~(1 << 19);} while(0)
    #define GPIOD_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 20); pRCC->RCC_AHBRSTR &= ~(1 << 20);} while(0)
    #define GPIOE_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 21); pRCC->RCC_AHBRSTR &= ~(1 << 21);} while(0)
    #define GPIOF_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 22); pRCC->RCC_AHBRSTR &= ~(1 << 22);} while(0)
    #define GPIOG_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 23); pRCC->RCC_AHBRSTR &= ~(1 << 23);} while(0)
    #define GPIOH_PORT_RESET()      do {pRCC->RCC_AHBRSTR |= (1 << 16); pRCC->RCC_AHBRSTR &= ~(1 << 16);} while(0)

    //Clock enable and disable macros for SPI
    #define SPI1_CLK_EN()         (pRCC->RCC_APB2ENR |= (1 << 12));
    #define SPI2_CLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 14));
    #define SPI3_CLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 15));
    #define SPI4_CLK_EN()         (pRCC->RCC_APB2ENR |= (1 << 15));

    #define SPI1_CLK_DI()         (pRCC->RCC_APB2ENR &= (~(1 << 12)));
    #define SPI2_CLK_DI()         (pRCC->RCC_APB1ENR &= (~(1 << 14)));
    #define SPI3_CLK_DI()         (pRCC->RCC_APB1ENR &= (~(1 << 15)));
    #define SPI4_CLK_DI()         (pRCC->RCC_APB2ENR &= (~(1 << 15)));


    //Clock enable and disable macros for I2C
    #define I2C1_CLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 21));
    #define I2C2_CLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 22));
    #define I2C3_CLK_EN()         (pRCC->RCC_APB1ENR |= (1 << 30));

    #define I2C1_CLK_DI()         (pRCC->RCC_APB1ENR &= (~(1 << 21)));
    #define I2C2_CLK_DI()         (pRCC->RCC_APB1ENR &= (~(1 << 22)));
    #define I2C3_CLK_DI()         (pRCC->RCC_APB1ENR &= (~(1 << 30)));

    //Clock enable macros for USART
    #define USART1_CLK_EN()       (pRCC->RCC_APB2ENR |= (1 << 14));
    #define USART2_CLK_EN()       (pRCC->RCC_APB1ENR |= (1 << 17));
    #define USART3_CLK_EN()       (pRCC->RCC_APB1ENR |= (1 << 18));

    #define USART1_CLK_DI()       (pRCC->RCC_APB2ENR &= (~(1 << 14)));
    #define USART2_CLK_DI()       (pRCC->RCC_APB1ENR &= (~(1 << 17)));
    #define USART3_CLK_DI()       (pRCC->RCC_APB1ENR &= (~(1 << 18)));

    //Clock enable macros for UART
    #define UART4_CLK_EN()        (pRCC->RCC_APB1ENR |= (1 << 19));
    #define UART5_CLK_EN()        (pRCC->RCC_APB1ENR |= (1 << 20));

    #define UART4_CLK_DI()        (pRCC->RCC_APB1ENR &= (~(1 << 19)));
    #define UART5_CLK_DI()        (pRCC->RCC_APB1ENR &= (~(1 << 20)));

    //Clock enable macros for CAN and USB
    #define CAN_CLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 25));
    #define USB_CLK_EN()          (pRCC->RCC_APB1ENR |= (1 << 23));    

    #define CAN_CLK_DI()          (pRCC->RCC_APB1ENR &= (~(1 << 25)));    
    #define USB_CLK_DI()          (pRCC->RCC_APB1ENR &= (~(1 << 23)));    
    


#endif /* INCLUDE_STM32F303RE_H_ */
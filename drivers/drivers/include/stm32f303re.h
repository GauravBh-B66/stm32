/*
 * stm32f303re.h
 *
 *  Created on: Mar 9, 2024
 *  Author: Gaurav Bhattarai
*/

#ifndef INCLUDE_STM32F303RE_H_
#define INCLUDE_STM32F303RE_H_

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
    #define DMA1_BASEADDRESS            (AHB1_BASEADDRESS + 0x0000)
    #define DMA2_BASEADDRESS            (AHB1_BASEADDRESS + 0x0400)
    #define RCC_BASEADDRESS             (AHB1_BASEADDRESS + 0x1000)
    #define FLASH_INTERFACE_BASEADDRESS (AHB1_BASEADDRESS + 0x2000)
    #define CRC_BASEADDRESS             (AHB1_BASEADDRESS + 0x3000)
    #define TSC_BASEADDRESS             (AHB1_BASEADDRESS + 0x4000)

    #define GPIOA_BASEADDRESS   (AHB2_BASEADDRESS + 0x0000)
    #define GPIOB_BASEADDRESS   (AHB2_BASEADDRESS + 0x0400)
    #define GPIOC_BASEADDRESS   (AHB2_BASEADDRESS + 0x0800)
    #define GPIOD_BASEADDRESS   (AHB2_BASEADDRESS + 0x0C00)
    #define GPIOE_BASEADDRESS   (AHB2_BASEADDRESS + 0x1000)
    #define GPIOF_BASEADDRESS   (AHB2_BASEADDRESS + 0x1400)
    #define GPIOG_BASEADDRESS   (AHB2_BASEADDRESS + 0x1800)
    #define GPIOH_BASEADDRESS   (AHB2_BASEADDRESS + 0x1C00)

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




#endif /* INCLUDE_STM32F303RE_H_ */

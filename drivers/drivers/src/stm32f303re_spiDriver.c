
#ifndef INCLUDE_STM32F303RE_SPIDRIVER_H_
#define INCLUDE_STM32F303RE_SPIDRIVER_H_

#include "../include/stm32f303re_spiDriver.h"
#include "../include/stm32f303re.h"

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0


//Peripheral Clock Setup
void spi_Clock_Control(SpiRegDef_t* pSPIx, uint8_t EN_DI){
    if(EN_DI == ENABLE){
        if (pSPIx == pSPI1){
            SPI1_CLK_EN();
        }
        else if (pSPIx == pSPI2){
            //Enable clock for SPI2
            SPI2_CLK_EN(); 
        }
        else if (pSPIx == pSPI3){
            //Enable clock for SPI3
            SPI3_CLK_EN(); 
        }
        else{
            //Enable clock for SPI4
            SPI4_CLK_EN(); 
        }
    }
    else if(EN_DI == DISABLE){
        if (pSPIx == pSPI1){
            //Enable clock for SPI1
            SPI1_CLK_DI();
        }
        else if (pSPIx == pSPI2){
            //Enable clock for SPI2
            SPI2_CLK_DI();
        }
        else if (pSPIx == pSPI3){
            //Enable clock for SPI3
            SPI3_CLK_DI();
        }
        else{
            //Enable clock for SPI4
            SPI4_CLK_DI();
        }
    }
}

//Initialization and Deinitialization
void SPI_Init(SPI_Handle_t* pSPIHandle){

    uint32_t tempReg;

    //Device mode configuration
    tempReg |= pSPIHandle->paramsSPI.spiDeviceMode << 2;

    //Bus configuration
    if(pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_FULL_DUPLEX){
        tempReg &= ~(1 << 15);                //Clear BIDI Mode   
    }

    else if(pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_HALF_DUPLEX){
        tempReg |= (1 << 15);                //Set BIDI Mode   
    }

    else if (pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_SIMPLE_RX){

        tempReg &= ~(1 << 15);                  //Clear BIDI Mode   
        tempReg |= (1 << 10);                   //Enable RXonly mode.   
    }


    //Clock speed configuration
    tempReg |= pSPIHandle->paramsSPI.spiClockSpeed << 3;

    //CPOL and CPHA configuration
    tempReg |= pSPIHandle->paramsSPI.spiCPOL << 1;
    tempReg |= pSPIHandle->paramsSPI.spiCPHA << 0;

    //Transfer the configured value to the actual register.
    pSPIHandle->pSPI->CR1 = tempReg;

    //Configuration for CR2 register

    //Reset the tempReg.
    tempReg = 0;

    //Data size configuration
    tempReg |= pSPIHandle->paramsSPI.spiDataSize << 8;
    pSPIHandle->pSPI->CR2 = tempReg;
}


//Send and Receive Data
void spiSendData(SPI_Handle_t* pSPI_Handle_t);
void spiReceiveData(SPI_Handle_t* pSPI_Handle_t);



#endif /* INCLUDE_STM32F303RE_SPIDRIVER_H_ */

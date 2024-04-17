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

    uint32_t tempReg = 0;

    //Device mode configuration
    tempReg |= pSPIHandle->paramsSPI.spiDeviceMode << SPI_CR1_MSTR;

    //Bus configuration
    if(pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_FULL_DUPLEX){
        tempReg &= ~(1 << SPI_CR1_BIDI_MODE);                //Clear BIDI Mode   
    }

    else if(pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_HALF_DUPLEX){
        tempReg |= (1 << SPI_CR1_BIDI_MODE);                //Set BIDI Mode   
    }

    else if (pSPIHandle->paramsSPI.spiBusConfig == SPI_BUS_SIMPLE_RX){

        tempReg &= ~(1 << SPI_CR1_BIDI_MODE);                  //Clear BIDI Mode   
        tempReg |= (1 << SPI_CR1_RX_ONLY);                   //Enable RXonly mode.   
    }


    //Clock speed configuration
    tempReg |= pSPIHandle->paramsSPI.spiClockSpeed << SPI_CR1_BAUD;

    //LSB/MSB first configuration
    tempReg |= pSPIHandle->paramsSPI.spiLSBFirst << SPI_CR1_LSB_FIRST;

    //CPOL and CPHA configuration
    tempReg |= pSPIHandle->paramsSPI.spiCPOL << SPI_CR1_CPOL;
    tempReg |= pSPIHandle->paramsSPI.spiCPHA << SPI_CR1_CPHA;

    //Transfer the configured value to the actual register.
    pSPIHandle->pSPI->CR1 = tempReg;

    //Configuration for CR2 register
    //Reset the tempReg.
    tempReg = 0;

    //Data size configuration
    tempReg |= pSPIHandle->paramsSPI.spiDataSize << SPI_CR2_DATA_SIZE;
    pSPIHandle->pSPI->CR2 = tempReg;
}


// //Send and Receive Data
void spi_SendData(SpiRegDef_t* pSPIx, uint8_t *txBuffer, uint32_t nBytes){
    while (nBytes > 0){
        //Check TX buffer
        if(pSPIx->SR & (1 << SPI_STATUS_TXE)){
            //TX buffer is empty

            //Check the data size.
            //Only 8,16 bit data is supported.
            if ((pSPIx->CR2 & 0xf00) == 0xf00){
                pSPIx->DR = *(uint16_t*)txBuffer;
                nBytes--;
                nBytes--;
                (uint16_t*)txBuffer++;
            }
            else if ((pSPIx->CR2 & 0xf00) == 0x700){
                pSPIx->DR = *txBuffer;
                nBytes--;                
                txBuffer++;
            }

        }
    }
}
// void spiReceiveData(SPI_Handle_t* pSPI_Handle_t);

void SPI_Deinit(SPI_Handle_t* pSPI_Handle){
    spi_Clock_Control(pSPI_Handle->pSPI, DISABLE);
}

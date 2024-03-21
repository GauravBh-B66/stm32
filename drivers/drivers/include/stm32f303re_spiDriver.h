#include "stm32f303re.h"

#ifndef INCLUDE_STM32F303RE_SPIDRIVER_H_
#define INCLUDE_STM32F303RE_SPIDRIVER_H_

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0

typedef struct{
    uint8_t spiDeviceMode;
    uint8_t spiClockSpeed;      
    uint8_t spiBusConfig;       //Full duplex, Half duplex, Simplex.
    uint8_t spiDff;
    uint8_t spiCPOL;
    uint8_t spiCPHA;
    uint8_t spiSSM;
}SPI_Params_t;

typedef struct{
    SpiRegDef_t*    pSPI;           //Address of the SPI channel used.
    SPI_Params_t    paramsSPI;      //Configuration parameters for the channel.
}SPI_Handle_t;



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
//Send and Receive Data

void spiSendData();
void spiReceiveData();



#endif /* INCLUDE_STM32F303RE_SPIDRIVER_H_ */

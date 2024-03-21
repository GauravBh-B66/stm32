#include "stm32f303re.h"

#ifndef INCLUDE_STM32F303RE_SPIDRIVER_H_
#define INCLUDE_STM32F303RE_SPIDRIVER_H_

typedef struct{
    uint8_t spiDeviceMode;
    uint8_t spiClockSpeed;
    uint8_t spiBusConfig;
    uint8_t spiDff;
    uint8_t spiCPOL;
    uint8_t spiCPHA;
    uint8_t spiSSM;
}SPI_Params_t;

typedef struct{
    SpiRegDef_t*    pSPI;           //Address of the SPI channel used.
    SPI_Params_t    paramsSPI;      //Configuration parameters for the channel.
}SPI_Handle_t;


void spiSendData();
void spiReceiveData();



#endif /* INCLUDE_STM32F303RE_SPIDRIVER_H_ */

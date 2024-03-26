
#ifndef INCLUDE_STM32F303RE_SPIDRIVER_H_
#define INCLUDE_STM32F303RE_SPIDRIVER_H_

#include "stm32f303re.h"

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0

typedef struct{
    uint8_t spiDeviceMode;      //Master or Slave
    uint8_t spiClockSpeed;      
    uint8_t spiBusConfig;       //Full duplex, Half duplex, Simplex.
    uint8_t spiDataSize;        //Data Size    
    uint8_t spiCPOL;            //Clock Polarity
    uint8_t spiCPHA;            //Clock Phase
    uint8_t spiSSM;             //Software Slave Management
}SPI_Params_t;

typedef struct{
    SpiRegDef_t*    pSPI;           //Address of the SPI channel used.
    SPI_Params_t    paramsSPI;      //Configuration parameters for the channel.
}SPI_Handle_t;


#define SPI_DEVICE_MODE_SLAVE   0
#define SPI_DEVICE_MODE_MASTER  1

#define SPI_BUS_FULL_DUPLEX     4
#define SPI_BUS_HALF_DUPLEX     3
#define SPI_BUS_SIMPLE_TX       2
#define SPI_BUS_SIMPLE_RX       1

#define SPI_SPEED_DIV_2          0
#define SPI_SPEED_DIV_4          1
#define SPI_SPEED_DIV_8          2
#define SPI_SPEED_DIV_16         3
#define SPI_SPEED_DIV_32         4
#define SPI_SPEED_DIV_64         5
#define SPI_SPEED_DIV_128        6
#define SPI_SPEED_DIV_256        7

#define SPI_DATA_SIZE_4        3
#define SPI_DATA_SIZE_5        4
#define SPI_DATA_SIZE_6        5
#define SPI_DATA_SIZE_7        6
#define SPI_DATA_SIZE_8        7
#define SPI_DATA_SIZE_9        8
#define SPI_DATA_SIZE_10       9
#define SPI_DATA_SIZE_11       10
#define SPI_DATA_SIZE_12       11
#define SPI_DATA_SIZE_13       12
#define SPI_DATA_SIZE_14       13
#define SPI_DATA_SIZE_15       14
#define SPI_DATA_SIZE_16       15

#define SPI_CPOL_LO             0
#define SPI_CPOL_HI             1

#define SPI_CPHA_LO             0
#define SPI_CPHA_HI             1

#define SPI_SSM_EN              1
#define SPI_SSM_DI              0




//Peripheral Clock Setup
void spi_Clock_Control(SpiRegDef_t* pSPIx, uint8_t EN_DI);

//Initialization and Deinitialization
void SPI_Init(SPI_Handle_t* pSPIHandle);

//Send and Receive Data
void spiSendData(SPI_Handle_t* pSPI_Handle_t);
void spiReceiveData(SPI_Handle_t* pSPI_Handle_t);



#endif /* INCLUDE_STM32F303RE_SPIDRIVER_H_ */

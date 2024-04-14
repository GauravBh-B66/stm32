
#ifndef INCLUDE_STM32F303RE_SPIDRIVER_H_
#define INCLUDE_STM32F303RE_SPIDRIVER_H_

#include "stm32f303re.h"

#define ENABLE  1
#define DISABLE 0
#define SET     1
#define RESET   0

typedef struct SPI_Params_t{
    uint8_t spiDeviceMode;      //Master or Slave
    uint8_t spiClockSpeed;      
    uint8_t spiBusConfig;       //Full duplex, Half duplex, Simplex.
    uint8_t spiDataSize;        //Data Size    
    uint8_t spiLSBFirst;        //0 for MSB first, 1 for LSB first.  
    uint8_t spiCPOL;            //Clock Polarity
    uint8_t spiCPHA;            //Clock Phase
    uint8_t spiSSM;             //Software Slave Management
} SPI_Params_t;

typedef struct SPI_Handle_t{
    SpiRegDef_t*    pSPI;           //Address of the SPI channel used.
    SPI_Params_t    paramsSPI;      //Configuration parameters for the channel.
} SPI_Handle_t;

//Register Position Macros

//Control Register 1
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BAUD        3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSB_FIRST   7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9       
#define SPI_CR1_RX_ONLY     10
#define SPI_CR1_CRCL        11
#define SPI_CR1_CRC_NEXT    12
#define SPI_CR1_CRC_EN      13
#define SPI_CR1_BIDI_OE     14
#define SPI_CR1_BIDI_MODE   15

//Control Register 2
#define SPI_CR2_RXDMAEN     0     
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_NSSP        3
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7
#define SPI_CR2_DATA_SIZE   8     
#define SPI_CR2_FRXTH       12
#define SPI_CR2_LDMA_RX     13
#define SPI_CR2_LDMA_TX     14

//Status Register
#define SPI_STATUS_RXNE     0
#define SPI_STATUS_TXE      1
#define SPI_STATUS_CHSIDE   2
#define SPI_STATUS_UDR      3
#define SPI_STATUS_CRC_ERR  4
#define SPI_STATUS_MODF     5
#define SPI_STATUS_OVR      6
#define SPI_STATUS_BSY      7
#define SPI_STATUS_FRE      8     0
#define SPI_STATUS_FTLVL    9
#define SPI_STATUS_FRLVL    11




#define SPI_DEVICE_MODE_SLAVE   0
#define SPI_DEVICE_MODE_MASTER  1

#define SPI_BUS_FULL_DUPLEX     1
#define SPI_BUS_HALF_DUPLEX     2
#define SPI_BUS_SIMPLE_RX       3
//TX only mode can be implemented same as FD mode.
//Just leaving the MISO line unconfigured does the trick.
// #define SPI_BUS_SIMPLE_TX       4

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
void spi_Clock_Control(SpiRegDef_t* , uint8_t);

//Initialization and Deinitialization
void SPI_Init(SPI_Handle_t* );
void SPI_Deinit(SPI_Handle_t* );

//Send and Receive Data
void spi_SendData(SpiRegDef_t*, uint8_t*, uint32_t);
void spiReceiveData(SPI_Handle_t* );



#endif /* INCLUDE_STM32F303RE_SPIDRIVER_H_ */

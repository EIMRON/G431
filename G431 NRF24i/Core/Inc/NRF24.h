/*
 * NRF24.h
 *
 *  Created on: Aug 10, 2024
 *      Author: EIMRON
 */

#ifndef INC_NRF24_H_
#define INC_NRF24_H_


#include "main.h"



// Structure definition
typedef struct {
	uint16_t checkbyte;
    uint16_t Joy1X;
    uint16_t Joy1Y;
    uint16_t checksum;
} RemotePackage;



// Declare the myStruct variable as extern
extern RemotePackage myStruct;

// Declare the packedArray variable as extern
extern uint8_t packedArray[sizeof(myStruct)];





/* Commands */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define R_RX_PL_WID   0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP           0xFF
#define ACTIVATE      0x50


/* Configs */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D



//Function prototypes
void NRF24_CMD(uint8_t CMD);
uint8_t NRF24_ReadReg(uint8_t Reg);
void NRF24_WriteReg(uint8_t Reg, uint8_t Value);
void NRF24_Init(void);
void NRF24_Config(void);
//void NRF24_RX_Address(uint8_t RX_Address);
void NRF24_RX_Address(void);
//void NRF24_TX_Address(uint8_t TX_Address);
void NRF24_TX_Address(void);
void NRF24_RX_Mode(void);
void NRF24_TX_Mode(void);
//void NRF24_Transmit(uint8_t* data, uint16_t length);
void NRF24_Transmit(uint8_t array[], int array_size);
void NRF24_TransmitBasic(void);
void NRF24_Receive(RemotePackage *TheStructure, uint8_t size);
uint16_t NRF24_ReceiveBasic(uint8_t size);
void PackIntoArray(RemotePackage *TheStructure, uint8_t array[]);
void UnpackFromArray(uint8_t array[], RemotePackage *TheStructure);
uint16_t CalculateChecksum(RemotePackage TheStructure);
uint8_t VerifyChecksum(RemotePackage *TheStructure);
void FailedPacket(void);




#endif /* INC_NRF24_H_ */

/*
 * NRF24.c
 *
 *  Created on: Aug 10, 2024
 *      Author: EIMRON
 */



#include "stm32g4xx_hal.h"
#include "NRF24.h"
#include "spi.h"

// CE and CSN functions
static inline void NRF24_CE_L() {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

static inline void NRF24_CE_H() {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}

static inline void NRF24_CSN_L() {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
}

static inline void NRF24_CSN_H() {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}


// SPI read/write function
static inline uint8_t NRF24_SPI_Send(uint8_t data) {
    uint8_t result;
    if (HAL_SPI_TransmitReceive(&hspi1, &data, &result, 1, 2000) != HAL_OK) {
        Error_Handler();
    }
    return result;
}


void NRF24_CMD(uint8_t CMD){

    NRF24_CSN_L();
    NRF24_SPI_Send(CMD);
    NRF24_CSN_H();

}

uint8_t NRF24_ReadReg(uint8_t Reg){
    uint8_t value;
    NRF24_CSN_L();
    NRF24_SPI_Send(R_REGISTER | Reg);
    value = NRF24_SPI_Send(NOP);
    NRF24_CSN_H();
    return value;
}



void NRF24_WriteReg(uint8_t Reg, uint8_t Value){
    NRF24_CSN_L();
    NRF24_SPI_Send(W_REGISTER | Reg);
    NRF24_SPI_Send(Value);
    NRF24_CSN_H();
}



void NRF24_Init(void){
	HAL_Delay(5);
	NRF24_WriteReg(SETUP_RETR, 0x5F);
	NRF24_WriteReg(RF_SETUP, 0x03);
	//NRF24_CMD(0x7F);
	NRF24_ReadReg(FEATURE);
    NRF24_CSN_L();
    NRF24_SPI_Send(ACTIVATE);
    NRF24_SPI_Send(0x73);
    NRF24_CSN_H();
	NRF24_ReadReg(FEATURE);
	NRF24_WriteReg(DYNPD, 0x00);
	NRF24_WriteReg(EN_AA, 0x3F);
	NRF24_WriteReg(EN_RXADDR, 0x03);
	NRF24_WriteReg(RX_PW_P0, 0x20);
	NRF24_WriteReg(RX_PW_P1, 0x20);
	NRF24_WriteReg(RX_PW_P2, 0x20);
	NRF24_WriteReg(RX_PW_P3, 0x20);
	NRF24_WriteReg(RX_PW_P4, 0x20);
	NRF24_WriteReg(RX_PW_P5, 0x20);
	NRF24_WriteReg(SETUP_AW, 0x03);
	NRF24_WriteReg(RF_CH, 0x4C);
	NRF24_WriteReg(STATUS, 0x70);
	NRF24_CMD(FLUSH_RX);
	NRF24_CMD(FLUSH_TX);
	NRF24_WriteReg(CONFIG, 0x0C);

	uint8_t featureValue = NRF24_ReadReg(CONFIG);

	    if (featureValue != 0x0C) {
	        Error_Handler();
	    }

	NRF24_WriteReg(CONFIG, 0x0E);
	HAL_Delay(5);

	featureValue = NRF24_ReadReg(CONFIG);

		if (featureValue != 0x0E) {
		        Error_Handler();
		}
    NRF24_ReadReg(RF_SETUP);
    NRF24_WriteReg(RF_SETUP, 0x03);

    //Payload sizes P0 for outgoing P1 for incoming
	NRF24_WriteReg(RX_PW_P0, 0x08); //Change depending on size of payload sent
	NRF24_WriteReg(RX_PW_P1, 0x02);
	NRF24_WriteReg(RX_PW_P2, 0x02);
	NRF24_WriteReg(RX_PW_P3, 0x02);
	NRF24_WriteReg(RX_PW_P4, 0x02);
	NRF24_WriteReg(RX_PW_P5, 0x02);

}



//void NRF24_Config(void)


void NRF24_RX_Address(void){

    NRF24_CSN_L();
    NRF24_SPI_Send(W_REGISTER | RX_ADDR_P1);
    NRF24_SPI_Send(0x31);
    NRF24_SPI_Send(0x4E);
    NRF24_SPI_Send(0x6F);
    NRF24_SPI_Send(0x64);
    NRF24_SPI_Send(0x65);
    NRF24_CSN_H();
    NRF24_WriteReg(EN_RXADDR, 0x02);


}


//void NRF24_TX_Address(uint8_t TX_Address)
void NRF24_TX_Address(void){

    NRF24_CSN_L();
    NRF24_SPI_Send(W_REGISTER | RX_ADDR_P0);
    NRF24_SPI_Send(0x31);
    NRF24_SPI_Send(0x4E);
    NRF24_SPI_Send(0x6F);
    NRF24_SPI_Send(0x64);
    NRF24_SPI_Send(0x65);
    NRF24_CSN_H();

    NRF24_CSN_L();
    NRF24_SPI_Send(W_REGISTER | TX_ADDR);
    NRF24_SPI_Send(0x31);
    NRF24_SPI_Send(0x4E);
    NRF24_SPI_Send(0x6F);
    NRF24_SPI_Send(0x64);
    NRF24_SPI_Send(0x65);
    NRF24_CSN_H();

}



void NRF24_RX_Mode(void){

	NRF24_CE_L();
	NRF24_WriteReg(CONFIG, 0x0F);
	NRF24_WriteReg(STATUS, 0x70);
	NRF24_WriteReg(EN_RXADDR, 0x02);
	NRF24_CE_H();

}

void NRF24_TX_Mode(void){

	NRF24_CE_L();
	NRF24_WriteReg(CONFIG, 0x0E);
	NRF24_ReadReg(EN_RXADDR);
	NRF24_WriteReg(EN_RXADDR, 0x03);

}


/*
bool RF24::txStandBy(uint32_t timeout, bool startTx)
{

    if (startTx) {
        stopListening();
        ce(HIGH);
    }
    uint32_t start = millis();

    while (!(read_register(FIFO_STATUS) & _BV(TX_EMPTY))) {
        if (status & _BV(MAX_RT)) {
            write_register(NRF_STATUS, _BV(MAX_RT));
            ce(LOW); // Set re-transmit
            ce(HIGH);
            if (millis() - start >= timeout) {
                ce(LOW);
                flush_tx();
                return 0;
            }
        }
*/
void NRF24_TransmitBasic(void){

	NRF24_CSN_L();
	NRF24_SPI_Send(W_TX_PAYLOAD);
	NRF24_SPI_Send(0x01);
	NRF24_SPI_Send(0x00);
	NRF24_CSN_H();
	NRF24_CE_H();

	uint8_t Check;
	uint32_t Timer = HAL_GetTick();
	do {
	        NRF24_CSN_L();                 // Pull CSN low
	        Check = NRF24_SPI_Send(NOP);   // Send NOP command and read status
	        NRF24_CSN_H();                 // Pull CSN high to complete the SPI transaction
	    } while (Check == 0x0E && (HAL_GetTick() - Timer) < 100);
	NRF24_CE_L();

	if (Check == 0x0E) {
		Error_Handler();
	    }


	NRF24_WriteReg(STATUS, 0x70);

}


void NRF24_Transmit(uint8_t array[], int array_size) {
    NRF24_CSN_L();
    NRF24_SPI_Send(W_TX_PAYLOAD);

    for (int i = 0; i < array_size; i++) {
        NRF24_SPI_Send(array[i]);
    }

    NRF24_CSN_H();
    NRF24_CE_H();

    uint8_t Check;
    uint32_t Timer = HAL_GetTick();
    do {
        NRF24_CSN_L();                // Pull CSN low
        Check = NRF24_SPI_Send(NOP);  // Send NOP command and read status
        NRF24_CSN_H();                // Pull CSN high to complete the SPI transaction
    } while (Check == 0x0E && (HAL_GetTick() - Timer) < 100);
    NRF24_CE_L();

    if (Check == 0x0E) {
        Error_Handler();
    }

    NRF24_WriteReg(STATUS, 0x70);
}


/*
void NRF24_Transmit(uint8_t* data, uint16_t length) {
    NRF24_CSN_L();
    NRF24_SPI_Send(W_TX_PAYLOAD);

    for (uint16_t i = 0; i < length; i++) {
        NRF24_SPI_Send(data[i]);
    }

    NRF24_CSN_H();
    NRF24_CE_H();

    uint8_t Check;
    uint32_t Timer = HAL_GetTick();
    do {
        NRF24_CSN_L();
        Check = NRF24_SPI_Send(NOP);
        NRF24_CSN_H();
    } while (Check == 0x0E && (HAL_GetTick() - Timer) < 100);

    NRF24_CE_L();

    if (Check == 0x0E) {
        Error_Handler();
    }

    NRF24_WriteReg(STATUS, 0x70);
}
*/


uint16_t NRF24_ReceiveBasic(uint8_t size){

	uint8_t data[2] = {0};
	uint8_t Check;
	uint16_t integer = 0;
	NRF24_CE_H();
	Check = NRF24_ReadReg(FIFO_STATUS);

		if (Check == 0x10) {

			NRF24_CSN_L();
			NRF24_SPI_Send(R_RX_PAYLOAD);

			for (uint8_t i = 0; i < size; i++){

				data[i] = NRF24_SPI_Send(NOP);

			}
			NRF24_CSN_H();
			integer = (data[1] << 8) | data[0];
		}
		return integer;
}

void NRF24_Receive(RemotePackage *TheStructure, uint8_t size) {
    uint8_t data[size];  // Array to hold received data
    uint8_t Check;
    NRF24_CE_H();
    Check = NRF24_ReadReg(FIFO_STATUS);

    if (Check != 0x10) {
        NRF24_CSN_L();
        NRF24_SPI_Send(R_RX_PAYLOAD);

        // Receive the payload
        for (uint8_t i = 0; i < size; i++) {
            data[i] = NRF24_SPI_Send(NOP);
        }

        NRF24_CSN_H();

        // Unpack the received data into the provided structure
        UnpackFromArray(data, TheStructure);
    }
}





 //NEW
RemotePackage myStruct;


uint8_t packedArray[sizeof(myStruct)];


uint16_t CalculateChecksum(RemotePackage TheStructure) {
    uint16_t checksum = 0;
    uint8_t* data = (uint8_t*)&TheStructure;

    // Calculate checksum excluding the checksum field itself
    for (size_t i = 0; i < sizeof(TheStructure) - sizeof(TheStructure.checksum); i++) {
        checksum += data[i];
    }

    return checksum;
}

/*
uint8_t VerifyChecksum(RemotePackage *TheStructure) {
    uint16_t receivedChecksum = TheStructure->checksum;
    TheStructure->checksum = 0;  // Clear the checksum field for recalculation

    uint16_t calculatedChecksum = CalculateChecksum(*TheStructure);

    return (receivedChecksum == calculatedChecksum);  // Return 1 if checksums match, 0 otherwise
}
*/

void PackIntoArray(RemotePackage *TheStructure, uint8_t array[]) {
	TheStructure->checksum = CalculateChecksum(*TheStructure);  // Calculate and set the checksum

    uint8_t* data = (uint8_t*)TheStructure;
    for (uint32_t i = 0; i < sizeof(RemotePackage); i++) {
        array[i] = data[i];  // Pack the structure into the array
    }
}

void UnpackFromArray(uint8_t array[], RemotePackage *TheStructure) {
    // Temporarily store the data to be checked
    RemotePackage temp;
    uint8_t* data = (uint8_t*)&temp;

    for (uint32_t i = 0; i < sizeof(temp); i++) {
        data[i] = array[i];  // Unpack the array into the temporary structure
    }

    // Verify checksum before updating the actual structure
    if (VerifyChecksum(&temp)) {
        *TheStructure = temp;  // Only update the original structure if the checksum is correct
    } else {
        // If checksum fails, you can log an error or handle it accordingly
    	FailedPacket();
    }
}

void FailedPacket(void){
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
}



/*//OLD
// Define the myStruct variable
RemotePackage myStruct;

// Define the packedArray variable
uint8_t packedArray[sizeof(myStruct)];

// Define the PackIntoArray function
void PackIntoArray(RemotePackage s, uint8_t array[]) {
    uint32_t size = sizeof(s);

    // Iterate over each byte in the structure and pack it into the array
    for (uint32_t i = 0; i < size; i++) {
        array[i] = ((uint8_t*)&s)[i];
    }
}

void UnpackFromArray(uint8_t array[], RemotePackage *s) {
    uint32_t size = sizeof(*s);  // Get the size of the structure

    // Iterate over each byte in the array and unpack it into the structure
    for (uint32_t i = 0; i < size; i++) {
        ((uint8_t*)s)[i] = array[i];
    }
}

*/



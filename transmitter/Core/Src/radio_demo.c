#include <memory.h>
#include "support.h"
#include "nrf24.h"
#include "main.h"
#include <stdbool.h>
//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width


#define HEX_CHARS      "0123456789ABCDEF"

#ifdef USE_HAL_DRIVER

extern UART_HandleTypeDef huart2;

/*
void UART_SendChar(char b) {
	HAL_UART_Transmit(&huart2, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
	HAL_UART_Transmit(&huart2, (uint8_t *) string, (uint16_t) strlen(string), 200);
}

void Toggle_LED() {
	//HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}
*/
#else //USE_HAL_DRIVER

void UART_SendChar(char b) {

	while(!LL_USART_IsActiveFlag_TXE(USART2)){};
	LL_USART_TransmitData8(USART2, (uint8_t) b);
}

void UART_SendStr(char *string) {
	for(;(*string) != 0;string++)
	{
		UART_SendChar(* string);
	}
}
void Toggle_LED() {
	LL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}

#endif
/*
void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}
void UART_SendHex8(uint16_t num) {
	UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = (char) (num % 10 + '0'); while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}
*/



uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;
nRF24_TXResult tx_res;

uint32_t i,j,k;

// Length of received payload
uint8_t payload_length;

#define DEMO_RX_SINGLE      0 // Single address receiver (1 pipe)
#define DEMO_RX_MULTI       0 // Multiple address receiver (3 pipes)
#define DEMO_RX_SOLAR       0 // Solar temperature sensor receiver
#define DEMO_TX_SINGLE      0// Single address transmitter (1 pipe)
#define DEMO_TX_MULTI       1 // Multiple address transmitter (3 pipes)
#define DEMO_RX_SINGLE_ESB  0 // Single address receiver with Enhanced ShockBurst (1 pipe)
#define DEMO_TX_SINGLE_ESB  0 // Single address transmitter with Enhanced ShockBurst (1 pipe)
#define DEMO_RX_ESB_ACK_PL  0 // Single address receiver with Enhanced ShockBurst (1 pipe) + payload sent back
#define DEMO_TX_ESB_ACK_PL  0 // Single address transmitter with Enhanced ShockBurst (1 pipe) + payload received in ACK


// Kinda foolproof :)
#if ((DEMO_RX_SINGLE + DEMO_RX_MULTI + DEMO_RX_SOLAR + DEMO_TX_SINGLE + DEMO_TX_MULTI + DEMO_RX_SINGLE_ESB + DEMO_TX_SINGLE_ESB + DEMO_RX_ESB_ACK_PL + DEMO_TX_ESB_ACK_PL) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif


#if ((DEMO_TX_SINGLE) || (DEMO_TX_MULTI) || (DEMO_TX_SINGLE_ESB) || (DEMO_TX_ESB_ACK_PL) )

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
/*
// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

extern nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	UART_SendStr("[");
	UART_SendHex8(status);
	UART_SendStr("] ");

	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}
*/
#endif // DEMO_TX_



void runRadio(void) {
	UART_SendStr("\r\nSTM32L432KC is online.\r\n");

	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	UART_SendStr("nRF24L01+ check: ");

	if (!nRF24_Check()) {
		UART_SendStr("FAIL\r\n");
	}

	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

#if (DEMO_RX_MULTI)

	// This is simple receiver with multiple RX pipes:
    //   - pipe#0 address: "WBC"
    //   - pipe#0 payload: 11 bytes
	//   - pipe#1 address: '0xE7 0x1C 0xE3'
    //   - pipe#1 payload: 5 bytes
    //   - pipe#4 address: '0xE7 0x1C 0xE6' (this is pipe#1 address with different last byte)
    //   - pipe#4 payload: 32 bytes (the maximum payload length)
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

    // The transmitter sends packets of different length to the three different logical addresses,
    // cycling them one after another, that packets comes to different pipes (0, 1 and 4)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure RX PIPE#0
    static const uint8_t nRF24_ADDR0[] = { 'W', 'B', 'C' };
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR0); // program address for RX pipe #0
    nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, 11); // Auto-ACK: disabled, payload length: 11 bytes

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR1[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR1); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

    // Configure RX PIPE#4
    static const uint8_t nRF24_ADDR4[] = { 0xE6 };
    nRF24_SetAddr(nRF24_PIPE4, nRF24_ADDR4); // program address for RX pipe #4
    nRF24_SetRXPipe(nRF24_PIPE4, nRF24_AA_OFF, 32); // Auto-ACK: disabled, payload length: 32 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();


    // The main loop
    while (1) {
    	//
    	// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
    	//
    	// This is far from best solution, but it's ok for testing purposes
    	// More smart way is to use the IRQ pin :)
    	//
    	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
    		// Get a payload from the transceiver
    		pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

    		// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();

			// Print a payload contents to UART
			UART_SendStr("RCV PIPE#");
			UART_SendInt(pipe);
			UART_SendStr(" PAYLOAD:>");
			UART_SendBufHex((char *)nRF24_payload, payload_length);
			UART_SendStr("<\r\n");
    	}
    }

#endif // DEMO_RX_MULTI



#if (DEMO_TX_MULTI)

	// This is simple transmitter (to multiple logic addresses):
	//   - TX addresses and payload lengths:
    //       'WBC', 11 bytes
    //       '0xE7 0x1C 0xE3', 5 bytes
    //       '0xE7 0x1C 0xE6', 32 bytes
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

    // The transmitter sends a data packets to the three logic addresses without Auto-ACK (ShockBurst disabled)
    // The payload length depends on the logic address

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    //static const uint8_t nRF24_ADDR0[] = { 'W', 'B', 'C' };
    //static const uint8_t nRF24_ADDR1[] = { 0xE7, 0x1C, 0xE3 };
    static const uint8_t nRF24_ADDR2[] = { 0xE7, 0x1C, 0xE6 };

    //nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);

    bool once = true;

    // The main loop
    j = 0; pipe = 2;
    while (once) {
    	// Logic address
    	UART_SendStr("ADDR#");
    	UART_SendInt(pipe);

    	// Configure the TX address and payload length
    	/*
    	switch (pipe) {
			case 0:
				// addr #1
				nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR0);
				payload_length = 11;
				break;
			case 1:
				// addr #2
				nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR1);
				payload_length = 5;
				break;
			case 2:
				// addr #3
				nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
				payload_length = 32;
				break;
			default:
				break;
		}
    	*/

    	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
		payload_length = 32;

    	// Prepare data packet
    	for (i = 0; i < payload_length; i++) {
    		nRF24_payload[i] = i;
    		//if (j > 0x000000FF) j = 0;
    	}

    	// Print a payload
    	UART_SendStr(" PAYLOAD:>");
    	UART_SendBufHex((char *)nRF24_payload, payload_length);
    	UART_SendStr("< ... TX: ");

    	// Transmit a packet
    	tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
    	switch (tx_res) {
			case nRF24_TX_SUCCESS:
				UART_SendStr("OK");
				break;
			case nRF24_TX_TIMEOUT:
				UART_SendStr("TIMEOUT");
				break;
			case nRF24_TX_MAXRT:
				UART_SendStr("MAX RETRANSMIT");
				break;
			default:
				UART_SendStr("ERROR");
				break;
		}
    	UART_SendStr("\r\n");

    	// Proceed to next address
    	//pipe++;
    	//if (pipe > 2) {
    	//	pipe = 0;
    	//}
    	once = false;
    	// Wait ~0.5s
    	Delay_ms(500);
    }

#endif // DEMO_TX_MULTI
}

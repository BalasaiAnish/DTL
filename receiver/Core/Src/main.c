/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"

#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t radioInitSuccess = 0;

uint8_t bufRX[20] = {0}; // buffer for received data
uint8_t bufLenRX = 20; // variable to store a length of received payload
uint8_t pipeRX;

char UARTbuf[256];
uint8_t UARTbufLen = 5;

char recChar; // received character
uint8_t Vbat; // battery voltage
uint8_t charConfidence; // boolean value depending on confidence of inference
uint8_t imgRow; // row of image
uint8_t charImg[28][28]; // image of written character

uint8_t txBuf[32] = {0}; // demo
uint8_t txSuccess = 10;

uint32_t c = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void testRadio(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void radioSetup(void);
void receiveData(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  nRF24_CE_L();
  radioInitSuccess = nRF24_Check();

  radioSetup();

  uint8_t rx_buffer[20]={0};
  uint8_t tx_buffer[20]={0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  receiveData(); // also sends data via UART
	  //testRadio();

	  /*
	  UARTbuf[0] = '\0';
	  for(int i = 0; i < 32; i++) sprintf(UARTbuf + strlen(UARTbuf), "%u ", bufRX[i]); // Add all values of bufRX into a buffer to send data, eparated by whitespace
	  UARTbuf[255] =  '\r';

   	  HAL_UART_Transmit(&huart2, (uint8_t *)UARTbuf, 255, 100);
   	  */

	  HAL_UART_Receive(&huart1,rx_buffer,20,100);

	  HAL_UART_Transmit(&huart2,tx_buffer,20,100);

   	  HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
/*
void radioSetup(void){
	// RX/TX disabled
	nRF24_CE_L();

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

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
	//nRF24_DisableAA(0xFF);

	// Set RF channel
	nRF24_SetRFChannel(115);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_250kbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);
	//nRF24_SetAddrWidth(5);
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
}
*/
void radioSetup(void){
	// Set RF channel
	nRF24_SetRFChannel(40);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_2Mbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure RX PIPE
	static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
	nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for pipe
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10); // Auto-ACK: enabled, payload length: 10 bytes

	// Set TX power for Auto-ACK (maximum, to ensure that transmitter will hear ACK reply)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Set operational mode (PRX == receiver)
	nRF24_SetOperationalMode(nRF24_MODE_RX);

	// Clear any pending IRQ flags
	nRF24_ClearIRQFlags();

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	// Enable DPL
	nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);

	nRF24_SetPayloadWithAck(1);

		// Put the transceiver to the RX mode
	nRF24_CE_H();
}

void receiveData(void){
	if (!radioInitSuccess) return;

	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {

		c++;
		pipeRX = nRF24_ReadPayload(bufRX, &bufLenRX); // read a payload to buffer

		nRF24_ClearIRQFlags(); // clear any pending IRQ bits

		// now the buffer bufRX holds received data
		// bufLenRX variable holds a length of received data

		/*
		 On the transmitter side
		txBuf[0] = (uint8_t)writtenChar;
		txBuf[1] = maxPred > THRESHOLD_NN_OUTPUT ? 1 : 0;
		txBuf[2] = getBattery();

		for(int i = 0; i < 28; i++) {
			txBuf[3] = i;
			for (int j = 0; j < 28; j++) txBuf[j + 4] = charImg[i][j];
			nRF24_TransmitPacket(&txBuf[0], 32);
		}
		*/

		HAL_UART_Transmit(&huart2,(uint8_t *) bufRX, strlen(bufRX), 100);
	}
}

void testRadio(void) {
	for(int i = 0; i < 32; i++){
		txBuf[i] = i*2;
	}
	nRF24_SetOperationalMode(nRF24_MODE_TX);

	txSuccess = nRF24_TransmitPacket(txBuf, 32);

	nRF24_SetOperationalMode(nRF24_MODE_RX);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

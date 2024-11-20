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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht.h"
#include "bmp.h"
#include "nrf24.h"
#include <string.h>
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
extern nRF24_TXResult tx_res;

uint8_t txBuf[20];
uint8_t txBufLen = 20;

static const uint8_t nRF24_ADDR2[] = { 0xE7, 0x1C, 0xE6 };

uint8_t radioCheck = 0;
uint8_t txSuccess = 10;

uint32_t analog_vals[2] = {0};
uint16_t ldr_voltage=0,raindrops_voltage=0;

// Buffer for transmission
uint8_t transmit_buffer[32];
uint8_t transmit_buffer_len = 32;

// Read temperature and pressure from BMP180
uint16_t u_temp = 0;
uint32_t u_press = 0;
float temp = 0.0;
float press = 0.0;

DHT11_InitTypeDef dht;
DHT11_StatusTypeDef err;

bmp_calib_t bmp_calib_data = {0};

// Convert float values into byte arrays
union
{
	float f_val;
	uint8_t f_val_buffer[4];
}temp_buffer,press_buffer,dht_temp_buffer,dht_hum_buffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void radioSetup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	ldr_voltage = (uint16_t) analog_vals[0];
	raindrops_voltage = (uint16_t) analog_vals[1];
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_LPTIM2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_LPTIM_TimeOut_Start_IT(&hlptim2,65535,19999);
  HAL_DHT11_Init(&dht, DHT_PIN_GPIO_Port, DHT_PIN_Pin, &htim2);
  read_calibration_data(&hi2c1,&bmp_calib_data);

  nRF24_CE_L();
  radioCheck = nRF24_Check();
  radioSetup();
  //runRadio();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	u_temp = get_uncomp_temp(&hi2c1,&bmp_calib_data);
	u_press = get_uncomp_press(&hi2c1,&bmp_calib_data);

	// Read LDR and Raindrop sensor analog voltages
	HAL_ADC_Start_DMA(&hadc1,analog_vals,2);

	// Read float values
	temp_buffer.f_val = get_comp_temp(u_temp,&bmp_calib_data);
	press_buffer.f_val = get_comp_press(u_temp,u_press,&bmp_calib_data);
	HAL_DHT11_ReadData(&dht);
	dht_temp_buffer.f_val = dht.Temperature;
	dht_hum_buffer.f_val = dht.Humidity;

	// First 16 bytes for float readings
	for(int i=0;i<4;i++)
	{
		transmit_buffer[i] = temp_buffer.f_val_buffer[i];
		transmit_buffer[i+4] = press_buffer.f_val_buffer[i];
		transmit_buffer[i+8] = dht_temp_buffer.f_val_buffer[i];
		transmit_buffer[i+12] = dht_hum_buffer.f_val_buffer[i];
	}

	transmit_buffer[16] = (uint8_t) (ldr_voltage & 0x00FF);
	transmit_buffer[17] = (uint8_t) ((ldr_voltage & 0xFF00)>>8);

	transmit_buffer[18] = (uint8_t) (raindrops_voltage & 0x00FF);
	transmit_buffer[19] = (uint8_t) ((raindrops_voltage & 0xFF00)>>8);

	tx_res = nRF24_TransmitPacket(transmit_buffer, transmit_buffer_len);

	HAL_UART_Transmit(&huart1,transmit_buffer,20,1000);

	//HAL_Delay(100);
	// Not needed due to sleep
	//HAL_Delay(500);
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
void radioSetup(void){
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

	    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);

}

void UART_SendChar(char b) {
	HAL_UART_Transmit(&huart1, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
	HAL_UART_Transmit(&huart1, (uint8_t *) string, (uint16_t) strlen(string), 200);
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

void testRadio(void) {
	/*
	UART_SendStr("ADDR#");
	UART_SendInt(2);
	*/

	//nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
	txBufLen = 20;
	uint8_t txBuf[20]={0};

	// Prepare data packet
	for (int i = 0; i < txBufLen; i++) {
		txBuf[i] = i;
		//if (j > 0x000000FF) j = 0;
	}
	/*
	// Print a payload
	UART_SendStr(" PAYLOAD:>");
	UART_SendBufHex((char *)txBuf, txBufLen);
	UART_SendStr("< ... TX: ");
	*/
	// Transmit a packet
	/*
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT
	*/
	tx_res = nRF24_TransmitPacket(txBuf, txBufLen);
	uint8_t ok_str[] = "ok";
	uint8_t time_str[] = "timeout";
	uint8_t mret_str[] = "max retransmit";
	uint8_t err_str[] = "error";
	switch (tx_res) {
		case nRF24_TX_SUCCESS:
			HAL_UART_Transmit(&huart1,ok_str,sizeof(ok_str),100);
			break;
		case nRF24_TX_TIMEOUT:
			HAL_UART_Transmit(&huart1,time_str,sizeof(time_str),100);
			break;
		case nRF24_TX_MAXRT:
			HAL_UART_Transmit(&huart1,mret_str,sizeof(mret_str),100);
			break;
		default:
			HAL_UART_Transmit(&huart1,err_str,sizeof(err_str),100);
			break;
	}
}



void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Timeout was reached, turn on LED3 */
  HAL_ResumeTick();
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
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

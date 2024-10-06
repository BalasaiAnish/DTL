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
#include "crc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "MPU6050.h"
#include "CalculateAngle.h"
#include "nrf24.h"
#include "preProcessor.h"

static void AI_Init(void);
static void AI_Run(uint8_t *pIn, float *pOut);
static uint8_t argmax(float * values, uint8_t len);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Global Variables


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t radioCheck = 0;
uint8_t txSuccess = 10;

char buf[55];
int buf_len = 0;

//bool nibHasBeenPressedAtleastOnce = false;

ai_handle predChar __attribute__((section(".ram2")));
float aiOutData[1][AI_PREDICTCHAR_OUT_1_CHANNEL] __attribute__((section(".ram2")));
ai_u8 activations[AI_PREDICTCHAR_DATA_ACTIVATIONS_SIZE] __attribute__((section(".ram2")));
ai_buffer *ai_input __attribute__((section(".ram2")));
ai_buffer *ai_output __attribute__((section(".ram2")));

float maxPred, maxInd __attribute__((section(".ram2")));
ai_i8 batch __attribute__((section(".ram2")));
ai_error err __attribute__((section(".ram2")));
const char classNames[36] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
uint8_t predClass __attribute__((section(".ram2")));
char writtenChar __attribute__((section(".ram2")));

nRF24_TXResult tx_res;

//static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};

uint8_t txBuf[32];
uint8_t txBufLen = 32;

static const uint8_t nRF24_ADDR2[] = { 0xE7, 0x1C, 0xE6 };

uint8_t charImgTest[][28] = // same image for running ML tests - just make sure the background is all 0 (black)
{
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 10, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 56, 128, 135, 136, 133, 128, 130, 137, 136, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 43, 128, 137, 131, 69, 0, 0, 0, 0, 38, 130, 137, 133, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 105, 133, 135, 58, 0, 0, 0, 0, 0, 0, 0, 0, 120, 137, 135, 16, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 88, 137, 134, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 99, 137, 134, 1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 3, 137, 131, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 119, 137, 65, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 131, 137, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 46, 137, 65, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 91, 137, 44, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 46, 137, 69, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 56, 137, 58, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 121, 137, 23, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 44, 137, 89, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 130, 133, 134, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 10, 137, 109, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 113, 0, 0, 0, 0, 0, 0, 0, 47, 134, 135, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 113, 0, 0, 0, 0, 0, 90, 129, 136, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 130, 82, 83, 90, 120, 131, 132, 110, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 11, 136, 131, 94, 93, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 113, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 114, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 136, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 97, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define pi 3.1415926536
//#define dt 0.001

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void testRadio(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

//for dt
float currentTime = 0;
float prevTime = 0;
float dt = 0;

//nib coordinates
int m = 0;

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
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  //MPU6050_Initialization();

 /*
	uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
	uint8_t EndMSG[] = "Done! \r\n\r\n";
	int ret;

	HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 100);
		for(int i=1; i<128; i++)
		{
			ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
			if (ret != HAL_OK)
			{
			  buf_len = sprintf(buf, "Not OK: 0x%X\n", i);
			  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
			}
			else if(ret == HAL_OK)
			{
			  buf_len = sprintf(buf, "OK: 0x%X\n", i);
				HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
			}
		}
		HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 100);
	*/
  /*
  	  greenOFF();
  	  //redOFF();
  	  HAL_Delay(2000); // Let the capacitor charge

  	  greenON();
  	  //redON();
  	  HAL_Delay(2000);
  	  greenOFF();
  	  //redOFF();
  	  HAL_Delay(1000);
	*/
  	  nRF24_CE_L();
  	  radioCheck = nRF24_Check();

  	  /*
  	  if(radioCheck == 1) {
  	  	  greenON();
  	  	  HAL_Delay(1000);
  	  	  greenOFF();

  	  	  buf_len = sprintf(buf, "Radio found! ");
  	  	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	    }
  	    else {
  	  	  redON();
  	  	  HAL_Delay(1000);
  	  	  redOFF();

  	  	  buf_len = sprintf(buf, "Radio not found! ");
  	  	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	    }

  	    HAL_Delay(1000);

  	    if(IMUCheck != 1) {
  	  	  greenON();
  	  	  HAL_Delay(1000);
  	  	  greenOFF();

  	  	  buf_len = sprintf(buf, " IMU found!");
  	  	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	    }
  	    else {
  	  	  redON();
  	  	  HAL_Delay(1000);
  	  	  redOFF();

  	  	  buf_len = sprintf(buf, " IMU not found!");
  	  	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	    }
  	    */

	AI_Init();

	resetVars();

	testRadio(); // execution will pause here if radio isn't working

	writtenChar = predictCharTest(); // works fine - just needs a black background

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  testRadio();

	  //m++;
	  currentTime = HAL_GetTick();
	  dt = (currentTime - prevTime)* 1e-3;
	  prevTime = currentTime;

	  MPU6050_Get6AxisRawData(&MPU6050);
	  MPU6050_DataConvert(&MPU6050);
	  CalculateAccAngle(&Angle,&MPU6050);
	  CalculateGyroAngle(&Angle,&MPU6050,dt);
	  CalculateCompliFilter(&Angle,&MPU6050,dt);
	  getTipCoords(dt);

	  if (isNibPressed()) {
		  saveCoordinates();
	  }
	  else if (isNewChar()) {
		  wakeRadio();
		  preProcessor(); // 26 ms - old val
		  writtenChar = predictChar(); // 719 ms - old val
		  //transmitChar(writtenChar); // max retry time: 4*20 = 80 ms
		  resetVars();
	  }
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
void radioSetup(void) {
	nRF24_Init();
	nRF24_SetOperationalMode(nRF24_MODE_TX);
	nRF24_SetRFChannel(TX_CHANNEL);
	nRF24_SetDataRate(nRF24_DR_250kbps);
	nRF24_SetCRCScheme(nRF24_CRC_2byte);
	nRF24_SetAutoRetr(nRF24_ARD_250us, 15);

	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Configure RX PIPE
	nRF24_SetAddrWidth(3);
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address
	nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR); // program address for pipe#0, must be same as TX (for Auto-ACK)
	nRF24_EnableAA(nRF24_PIPE0);

	nRF24_SetPowerMode(nRF24_PWR_UP);

	//nRF24_CE_H();

	nRF24_ClearIRQFlags();
}
*/


void radioSetup(void) {
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

	nRF24_Init();

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

	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);
}


bool isNibPressed(void) {
	return !HAL_GPIO_ReadPin(NibPress_GPIO_Port, NibPress_Pin); // active low
}

bool isNewChar(void) {
	return !HAL_GPIO_ReadPin(ThumbPress_GPIO_Port, ThumbPress_Pin); // active low
}

void transmitChar(char) {

	for(int i = 0; i < 32; i++) txBuf[i] = i;

	txBuf[0] = (uint8_t)writtenChar;
	txBuf[1] = maxPred > THRESHOLD_NN_OUTPUT ? 1 : 0;
	txBuf[2] = getBattery();

	for(int i = 0; i < 28; i++) {
		txBuf[3] = i;
		for (int j = 0; j < 28; j++) txBuf[j + 4] = charImgQuantised[i][j];
		nRF24_TransmitPacket(txBuf, txBufLen);
	}
}

void testRadio(void) {
	for(int i = 0; i < 32; i++) txBuf[i] = i*2;
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR2);

	txSuccess = nRF24_TransmitPacket(txBuf, txBufLen);
}

uint8_t getBattery(void){
	return 1;
}

char predictChar(void) {
	AI_Run(&charImgQuantised[0][0], &aiOutData[0][0]);

	/*
	buf_len = sprintf(buf, "predictChar outputs:\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

	for (uint32_t i = 0; i < AI_PREDICTCHAR_OUT_1_CHANNEL; i++) {
	  buf_len = sprintf(buf, "%8.6f \n", aiOutData[0][0][i]);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
	}
	*/

	predClass = argmax(&aiOutData[0][0], AI_PREDICTCHAR_OUT_1_CHANNEL);

	/*
	if(maxPred > THRESHOLD_NN_OUTPUT)  {
		greenON();
		HAL_Delay(250);
		greenOFF();
	}
	else {
		redON();
		HAL_Delay(250);
		redOFF();
	}
	*/

	buf_len = sprintf(buf, "Predicted character: %c\n", classNames[predClass]);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

    return classNames[predClass];
}

char predictCharTest(void) {
	AI_Run(&charImgTest[0][0], &aiOutData[0][0]);

	predClass = argmax(&aiOutData[0][0], AI_PREDICTCHAR_OUT_1_CHANNEL);

	buf_len = sprintf(buf, "Predicted character: %c\n", classNames[predClass]);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

    return classNames[predClass];
}

static void AI_Init(void)
{
  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_predictchar_create_and_init(&predChar, act_addr, NULL);

  if (err.type != AI_ERROR_NONE) {
    buf_len = sprintf(buf, "ai_predictchar_create error - type=%d code=%d\r\n", err.type, err.code);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

  }
  ai_input = ai_predictchar_inputs_get(predChar, NULL);
  ai_output = ai_predictchar_outputs_get(predChar, NULL);
}

static void AI_Run(uint8_t *pIn, float *pOut)
{
  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_predictchar_run(predChar, ai_input, ai_output);

  if (batch != 1) {
    err = ai_predictchar_get_error(predChar);
    buf_len = sprintf(buf, "ai_predictchar_run error - type=%d code=%d\r\n", err.type, err.code);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  }

}

static uint8_t argmax(float * values, uint8_t len)
{
  maxPred = values[0];
  maxInd = 0;
  for (uint8_t i = 1; i < len; i++) {
    if (values[i] > maxPred) {
      maxPred = values[i];
      maxInd = i;
    }
  }
  return maxInd;
}

void allAsleep(void) {

}

void wakeIMU(void) {

}

void wakeRadio(void) {

}

void greenON(void){
	HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_SET);
}

/*void redON(void){
	HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_SET);
}*/

void greenOFF(void){
	HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_RESET);
}

/*void redOFF(void){
	HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_RESET);
}*/

nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	volatile uint8_t status;

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

	//UART_SendStr("[");
	//UART_SendHex8(status);
	//UART_SendStr("] ");

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

void UART_SendChar(char b) {
	HAL_UART_Transmit(&huart2, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
	HAL_UART_Transmit(&huart2, (uint8_t *) string, (uint16_t) strlen(string), 200);
}

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

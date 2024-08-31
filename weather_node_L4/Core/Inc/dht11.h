/*
 * dht11.h
 *
 *  Created on: Aug 31, 2024
 *      Author: bala
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

typedef enum {
	DHT11_OK = 0,
	DHT11_TIMEOUT,
	DHT11_ERROR,
	DHT11_CHECKSUM_MISMATCH,
	__DHT11_STATUS_TYPEDEF_COUNT__
} DHT11_StatusTypeDef;

typedef struct {
	float Temperature;
	float Humidity;
	DHT11_StatusTypeDef Status;
	uint16_t _Pin;
	GPIO_TypeDef *_GPIOx;
	TIM_HandleTypeDef *_Tim;
} DHT11_InitTypeDef;

/**
  * @brief  Intitalizes the DHT11 Driver.
  * @param	DHT11 instance of a DHT11 driver.
  * @param  GPIOx where x can be (A..G) to select the GPIO peripheral for STM32xxxx family.
  * @param  GPIO_Pin specifies the pin DHT11 is connected.
  * @param 	TIM handler for timer with frequency to count 1us.
  * @retval None
  */
void HAL_DHT11_Init(DHT11_InitTypeDef *DHT11, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, TIM_HandleTypeDef *TIM);

/**
  * @brief  Reads data(Temperature, Humidity) from the DHT11 Driver.
  * @param	DHT11 instance of a DHT11 driver.
  * @retval DHT11_StatusTypeDef
  */
DHT11_StatusTypeDef HAL_DHT11_ReadData(DHT11_InitTypeDef *DHT11);

/**
  * @brief  De-initializes the DHT11 Driver.
  * @param	DHT11 instance of a DHT11 driver.
  * @retval None
  */
void  HAL_DHT11_DeInit(DHT11_InitTypeDef *DHT11);

/**
  * @brief  Get the error message of type DHT11_StatusTypeDef.
  * @param	Status status code.
  * @retval const char* const
  */
const char* const HAL_DHT11_GetErrorMsg(DHT11_StatusTypeDef Status);

/**
  * @brief  Reads Temperature value in Celcius from the DHT11 Driver. Status of the command is stored in DHT11_InitTypeDef::Status.
  * @param	DHT11_InitTypeDef instance of a DHT11 driver.
  * @retval float
  */
float HAL_DHT11_ReadTemperatureC(DHT11_InitTypeDef *DHT11);

/**
  * @brief  Reads Temperature value in Fahrenheit from the DHT11 Driver. Status of the command is stored in DHT11_InitTypeDef::Status.
  * @param	DHT11 instance of a DHT11 driver.
  * @retval float
  */
float HAL_DHT11_ReadTemperatureF(DHT11_InitTypeDef *DHT11);

/**
  * @brief  Reads Humidity value from the DHT11 Driver. Status of the command is stored in DHT11_InitTypeDef::Status.
  * @param	DHT11_InitTypeDef instance of a DHT11 driver.
  * @retval float
  */
float HAL_DHT11_ReadHumidity(DHT11_InitTypeDef *DHT11);

#endif

#endif /* INC_DHT11_H_ */

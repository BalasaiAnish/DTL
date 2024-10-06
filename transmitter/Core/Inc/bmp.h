/*
 * bmp.h
 *
 *  Created on: Aug 30, 2024
 *      Author: bala
 */

#ifndef INC_BMP_H_
#define INC_BMP_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"
#include <stdbool.h>
#include <math.h>

#define BMP_ADDR 0xEE
#define CALIB_START_ADDR 0xAA

#define TEMP_WRITE_VAL 0x2E
#define TEMP_REG_ADDR 0xF4
#define TEMP_REG_ADDR 0xF4
#define PRESS_REG_ADDR 0xF6
#define OSS 0

typedef struct bmp_calib_t
{
	uint16_t AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD;
}bmp_calib_t;



void read_calibration_data(I2C_HandleTypeDef *, bmp_calib_t *);

uint16_t get_uncomp_temp(I2C_HandleTypeDef *, bmp_calib_t *);

uint32_t get_uncomp_press(I2C_HandleTypeDef *, bmp_calib_t *);

float get_comp_temp(uint16_t, bmp_calib_t *);

float get_comp_press(uint16_t, uint16_t, bmp_calib_t *);

#endif /* INC_BMP_H_ */

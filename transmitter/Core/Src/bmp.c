/*
 * bmp.c
 *
 *  Created on: Aug 30, 2024
 *      Author: bala
 */
#include "bmp.h"

void read_calibration_data(I2C_HandleTypeDef *hi2c, bmp_calib_t *bmp_calib_data)
{
	uint8_t calib_data[22];

	HAL_I2C_Mem_Read(hi2c,BMP_ADDR,(uint16_t) CALIB_START_ADDR,1,calib_data,22,1000);

	bmp_calib_data->AC1 = (uint16_t) (calib_data[0] << 8) | (calib_data[1]);
	bmp_calib_data->AC2 = (uint16_t) (calib_data[2] << 8) | (calib_data[3]);
	bmp_calib_data->AC3 = (uint16_t) (calib_data[4] << 8) | (calib_data[5]);
	bmp_calib_data->AC4 = (uint16_t) (calib_data[6] << 8) | (calib_data[7]);
	bmp_calib_data->AC5 = (uint16_t) (calib_data[8] << 8) | (calib_data[9]);
	bmp_calib_data->AC6 = (uint16_t) (calib_data[10] << 8) | (calib_data[11]);

	bmp_calib_data->B1 = (uint16_t) (calib_data[12] << 8) | (calib_data[13]);
	bmp_calib_data->B2 = (uint16_t) (calib_data[14] << 8) | (calib_data[15]);

	bmp_calib_data->MB = (uint16_t) (calib_data[16] << 8) | (calib_data[17]);
	bmp_calib_data->MC = (uint16_t) (calib_data[18] << 8) | (calib_data[19]);
	bmp_calib_data->MD = (uint16_t) (calib_data[20] << 8) | (calib_data[21]);
}

uint16_t get_uncomp_temp(I2C_HandleTypeDef *hi2c, bmp_calib_t *bmp_calib_data)
{
	uint8_t raw_temp_data[2];
	uint8_t temp_write_val = TEMP_WRITE_VAL;

	HAL_I2C_Mem_Write(hi2c,BMP_ADDR,(uint16_t) TEMP_REG_ADDR,1,&temp_write_val,1,1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(hi2c,BMP_ADDR,(uint16_t) TEMP_REG_ADDR,1,raw_temp_data,2,1000);

	uint16_t uncomp_temp = (uint16_t) (raw_temp_data[0] << 8) | raw_temp_data[1];


	float X1 = ((uncomp_temp-bmp_calib_data->AC6) * (bmp_calib_data->AC5/(pow(2,15))));
	float X2 = ((bmp_calib_data->MC*(pow(2,11))) / (X1+bmp_calib_data->MD));
	float B5 = X1+X2;
	float comp_temp = ((B5+8)/(pow(2,4)))/10.0;
	return comp_temp;
}

uint32_t get_uncomp_press(I2C_HandleTypeDef *hi2c, bmp_calib_t *bmp_calib_data)
{
	uint8_t raw_comp_press_data[3];
	uint8_t comp_press_write_data = TEMP_WRITE_VAL + (OSS << 6);

	HAL_I2C_Mem_Write(hi2c,BMP_ADDR,(uint16_t) TEMP_REG_ADDR,1,&comp_press_write_data,1,1000);

	switch (OSS)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}

	HAL_I2C_Mem_Read(hi2c,BMP_ADDR,(uint16_t) PRESS_REG_ADDR,1,raw_comp_press_data,3,1000);

	uint32_t uncomp_press = (((raw_comp_press_data[0]<<16)+(raw_comp_press_data[1]<<8)+raw_comp_press_data[2]) >> (8-OSS));
	return uncomp_press;

}

float get_comp_temp(uint16_t uncomp_temp, bmp_calib_t *bmp_calib_data)
{
	int32_t X1 = ((uncomp_temp-bmp_calib_data->AC6) * (bmp_calib_data->AC5/(pow(2,15))));
	int32_t X2 = ((bmp_calib_data->MC*(pow(2,11))) / (X1+bmp_calib_data->MD));
	int32_t B5 = X1+X2;
	float comp_temp = ((B5+8)/(pow(2,4)))/10.0;
	return comp_temp;
}

float get_comp_press(uint16_t uncomp_press, uint16_t uncomp_temp, bmp_calib_t *bmp_calib_data)
{
	float comp_press = 0.0;
	int32_t X1 = ((uncomp_temp-bmp_calib_data->AC6) * (bmp_calib_data->AC5/(pow(2,15))));
	int32_t X2 = ((bmp_calib_data->MC*(pow(2,11))) / (X1+bmp_calib_data->MD));
	int32_t B5 = X1+X2;
	int32_t B6 = B5-4000;
	X1 = (bmp_calib_data->B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = bmp_calib_data->AC2*B6/(pow(2,11));
	int32_t X3 = X1+X2;
	int32_t B3 = (((bmp_calib_data->AC1*4+X3)<<OSS)+2)/4;
	X1 = bmp_calib_data->AC3*B6/pow(2,13);
	X2 = (bmp_calib_data->B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	uint32_t B4 = bmp_calib_data->AC4*(uint32_t)(X3+32768)/(pow(2,15));
	int32_t B7 = ((uint32_t)uncomp_press-B3)*(50000>>OSS);

	if (B7<0x80000000)
		comp_press = (B7*2)/B4;
	else
		comp_press = (B7/B4)*2;

	X1 = (comp_press/(pow(2,8)))*(comp_press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*comp_press)/(pow(2,16));
	comp_press = comp_press + (X1+X2+3791)/(pow(2,4));
	return comp_press;
}

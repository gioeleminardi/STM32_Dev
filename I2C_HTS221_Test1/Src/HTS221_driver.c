/*
 * HTS221_driver.c
 *
 *  Created on: 07 mag 2018
 *      Author: Gioele Minardi
 */

#include "HTS221_driver.h"

HTS221_Error_et HTS221_read_reg(I2C_HandleTypeDef *handle, uint8_t RegAddr,
		uint16_t NumByteToRead, uint8_t *Data) {

	if (NumByteToRead > 1)
		RegAddr |= 0x80;

	if (HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, RegAddr,
	I2C_MEMADD_SIZE_8BIT, Data, NumByteToRead, HAL_MAX_DELAY) != HAL_OK)
		return HTS221_ERROR;
	else
		return HTS221_OK;
}

HTS221_Error_et HTS221_write_reg(I2C_HandleTypeDef *handle, uint8_t RegAddr,
		uint16_t NumByteToWrite, uint8_t *Data) {

	if (NumByteToWrite > 1)
		RegAddr |= 0x80;

	if (HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, RegAddr,
	I2C_MEMADD_SIZE_8BIT, Data, NumByteToWrite, HAL_MAX_DELAY) != HAL_OK)
		return HTS221_ERROR;
	else
		return HTS221_OK;
}

HTS221_Error_et HTS221_Activate(I2C_HandleTypeDef *handle) {
	uint8_t tmp;

	if (HTS221_read_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	tmp |= HTS221_PD_MASK;

	if (HTS221_write_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	return HTS221_OK;
}

HTS221_Error_et HTS221_DeActivate(I2C_HandleTypeDef *handle) {
	uint8_t tmp;

	if (HTS221_read_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	tmp &= ~HTS221_PD_MASK;

	if (HTS221_write_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	return HTS221_OK;
}

HTS221_Error_et HTS221_Get_DeviceID(I2C_HandleTypeDef *handle,
		uint8_t* deviceid) {
	if (HTS221_read_reg(handle, HTS221_WHO_AM_I_REG, 1, deviceid))
		return HTS221_ERROR;

	return HTS221_OK;
}

HTS221_Error_et HTS221_Set_BduMode(I2C_HandleTypeDef *handle,
		HTS221_State_et status) {
	uint8_t tmp;

	HTS221_assert_param(IS_HTS221_State(status));

	if (HTS221_read_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	tmp &= ~HTS221_BDU_MASK;
	tmp |= ((uint8_t) status) << HTS221_BDU_BIT;

	if (HTS221_write_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	return HTS221_OK;
}

HTS221_Error_et HTS221_Set_Odr(I2C_HandleTypeDef *handle, HTS221_Odr_et odr) {
	uint8_t tmp;

	HTS221_assert_param(IS_HTS221_ODR(odr));

	if (HTS221_read_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	tmp &= ~HTS221_ODR_MASK;
	tmp |= (uint8_t) odr;

	if (HTS221_write_reg(handle, HTS221_CTRL_REG1, 1, &tmp))
		return HTS221_ERROR;

	return HTS221_OK;
}

HTS221_Error_et HTS221_Get_Temperature(I2C_HandleTypeDef *handle,
		int16_t *value) {
	int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	int16_t T0_degC, T1_degC;
	uint8_t buffer[4], tmp;
	float tmp_f;

	if (HTS221_read_reg(handle, HTS221_T0_DEGC_X8, 2, buffer))
		return HTS221_ERROR;
	if (HTS221_read_reg(handle, HTS221_T0_T1_DEGC_H2, 1, &tmp))
		return HTS221_ERROR;

	T0_degC_x8_u16 = (((uint16_t) (tmp & 0x03)) << 8) | ((uint16_t) buffer[0]);
	T1_degC_x8_u16 = (((uint16_t) (tmp & 0x0C)) << 6) | ((uint16_t) buffer[1]);
	T0_degC = T0_degC_x8_u16 >> 3;
	T1_degC = T1_degC_x8_u16 >> 3;

	if (HTS221_read_reg(handle, HTS221_T0_OUT_L, 4, buffer))
		return HTS221_ERROR;

	T0_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	T1_out = (((uint16_t) buffer[3]) << 8) | (uint16_t) buffer[2];

	if (HTS221_read_reg(handle, HTS221_TEMP_OUT_L_REG, 2, buffer))
		return HTS221_ERROR;

	T_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];

	tmp_f = (float) (T_out - T0_out) * (float) (T1_degC - T0_degC)
			/ (float) (T1_out - T0_out) + T0_degC;
	tmp_f *= 10.0f;

	*value = (int16_t) tmp_f;

	return HTS221_OK;
}

HTS221_Error_et HTS221_Get_Humidity(I2C_HandleTypeDef *handle, uint16_t* value)
{
  int16_t H0_T0_out, H1_T0_out, H_T_out;
  int16_t H0_rh, H1_rh;
  uint8_t buffer[2];
  float   tmp_f;

  if(HTS221_read_reg(handle, HTS221_H0_RH_X2, 2, buffer))
    return HTS221_ERROR;
  H0_rh = buffer[0] >> 1;
  H1_rh = buffer[1] >> 1;

  if(HTS221_read_reg(handle, HTS221_H0_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H0_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_read_reg(handle, HTS221_H1_T0_OUT_L, 2, buffer))
    return HTS221_ERROR;
  H1_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  if(HTS221_read_reg(handle, HTS221_HR_OUT_L_REG, 2, buffer))
    return HTS221_ERROR;
  H_T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

  tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
  tmp_f *= 10.0f;

  *value = ( tmp_f > 1000.0f ) ? 1000
           : ( tmp_f <    0.0f ) ?    0
           : ( uint16_t )tmp_f;

  return HTS221_OK;
}

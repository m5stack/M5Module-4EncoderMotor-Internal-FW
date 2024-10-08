/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_ex_H
#define __i2c_ex_H
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define I2C_UPGRADE_BUFFER_LEN  1050

extern volatile uint8_t Receive_Buffer[I2C_UPGRADE_BUFFER_LEN];

extern void i2c1_set_send_data(uint8_t *tx_ptr, uint16_t len);
extern void I2CAddReg(uint8_t reg, uint8_t* buff, uint8_t len, uint8_t bit);
extern void I2CInit(void);
extern uint8_t I2CGetTxState(void);

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

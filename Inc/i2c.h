/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

#define MPU6050_I2C_ADDR (0x69 << 1)
#define MPU6050_WHO_I_AM_ADDR 0x75
#define MPU6050_GYRO_CONFIG_ADDR 0x1B
#define MPU6050_ACCEL_CONFIG_ADDR 0x1C
#define MPU6050_PWR_MGMT_1_ADDR 0x6B
#define MPU6050_FILTER_CONFIG_ADDR 0x1A
#define MPU6050_ACCEL_XOUT_H_ADDR 0x3B
#define MPU6050_ACCEL_XOUT_L_ADDR 0x3C
#define MPU6050_ACCEL_YOUT_H_ADDR 0x3D
#define MPU6050_ACCEL_YOUT_L_ADDR 0x3E
#define MPU6050_ACCEL_ZOUT_H_ADDR 0x3F
#define MPU6050_ACCEL_ZOUT_L_ADDR 0x40
#define MPU6050_GYRO_XOUT_H_ADDR 0x43
#define MPU6050_GYRO_XOUT_L_ADDR 0x44
#define MPU6050_GYRO_YOUT_H_ADDR 0x45
#define MPU6050_GYRO_YOUT_L_ADDR 0x46
#define MPU6050_GYRO_ZOUT_H_ADDR 0x47
#define MPU6050_GYRO_ZOUT_L_ADDR 0x48


/* USER CODE END Private defines */
typedef struct MPU6050_Data
{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} st_MPU6050_Data;

void MX_I2C1_Init(void);
void I2C1_Init(I2C_HandleTypeDef*);

/* USER CODE BEGIN Prototypes */
void MPUConfigure(void);
void MPU_mem_read(uint8_t memAddr);
void MPU_Initialize(void);
void MPU_power_configure(void);
void MPU_Gyro_configure(void);
void MPU_Accel_configure(void);
void MPU_Accel_read(st_MPU6050_Data*);
void MPU_Gyro_read(st_MPU6050_Data*);
void MPU_Data_read(st_MPU6050_Data*);
void MPU_configure(uint8_t memAddr, uint8_t configuration);
void MPU_Filter_FrameSync_configure(void);
void Accel_RollDegreeCal(st_MPU6050_Data*, float*);
void printSensorData(st_MPU6050_Data* data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

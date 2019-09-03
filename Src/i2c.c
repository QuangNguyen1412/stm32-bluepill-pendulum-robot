/**
  ******************************************************************************
  * File Name          : I2C.c
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include <math.h>

#define A_R 16384.0
#define G_R 65.5
#define M_PI 3.141592
#define ACEL_X_OFFSET 0.092
#define ACEL_Y_OFFSET 0.002
#define ACEL_Z_OFFSET 0.03
#define GYRO_Y_OFFSET 998.4
//#define DEBUG 1
I2C_HandleTypeDef hi2c1;
st_MPU6050_Data v_MPU6050_Data;
// This will be modified under function Accel_RollDegreeCal()
uint8_t Acel_Config = (0 << 3);
void printSensorData(st_MPU6050_Data*);
/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = MPU_SCL_Pin|MPU_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
//    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, MPU_SCL_Pin|MPU_SDA_Pin);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/**	@function 	I2C1_Init
	*	@brief			Initialize I2C1 peripherals
	*	@configuration
	*	PB6     ------> I2C1_SCL
	*	PB7     ------> I2C1_SDA
	*	f SCL = 100kHz (you need to know f PCLK1 = 32MHz to calculate f SCL, I2C1->CCR)
	*	Peripheral clock frequency = 32MHz (I2C1->CR2)
	*	TRISE (I2C1->TRISE): 33
	*	Address mode: 7 bit
	*	Mode: Master Transmit and Master Receive
	*	No stress: disable
	*	Duty cycle (I2C1->CCR): 2
	*/
void I2C1_Init(I2C_HandleTypeDef* i2cHandle)
{
  // Enable RCC clock for GPIOB
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  // Set pin PB7(SDA) to alternate out open drain max 10MHz
  GPIOB->CRL |= (0xD << 28);

  // Set pin PB6(SCL) to alternate push pull max 10MHz
  GPIOB->CRL |= (0xD << 24);

  // I2C configure
  // enable RCC clock for i2c1
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Get PCLK
  uint32_t freq = HAL_RCC_GetPCLK1Freq();
  printf("Pclk1: %d\n", freq);

  freq /= 1000000;
  I2C1->CR2 |= freq;          // Set clock to 32MHz
  I2C1->CCR &= ~(1 << 15);    // SM mode
  I2C1->CCR &= ~(1 << 14);    // Duty cycle
  I2C1->CCR |= 0xA0;          // 32 MHz, 100Khz
  I2C1->TRISE = freq + 1;     // Set clock rise time
  I2C1->OAR1 &= ~(1 << 15);   // Set Address mode
  I2C1->CR1 |= (1 << 7);      // Set No stretch disable
  I2C1->CR1 |= 1;             // Enable peripheral

  /* Enable the i2c instance state and code to be used later with HAL library */
  i2cHandle->Instance = I2C1;
	i2cHandle->ErrorCode = HAL_I2C_ERROR_NONE;
  i2cHandle->State = HAL_I2C_STATE_READY;
  i2cHandle->PreviousState = HAL_I2C_MODE_NONE;
  i2cHandle->Mode = HAL_I2C_MODE_NONE;
}


/* USER CODE BEGIN 1 */
/* Read request for a memory of the MPU6050 */
void MPU_mem_read(uint8_t memAddr) 
{
  uint8_t data = 0;
  uint16_t errCode = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("Receive successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }
}

/* Initialize the MPU: Power, Gyro FS, Accel FS */
void MPU_Initialize()
{
  MPU_power_configure();
  MPU_Filter_FrameSync_configure();
  MPU_Accel_configure();
  MPU_Gyro_configure();
}

void MPU_power_configure()
{
  // Clock select PLL with Z axis gyroscope
  // Disable temperature sensor
  // disable sleep mode
  // Disable cycle
  uint8_t data = (0x1 << 7);
  
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  HAL_Delay(100);
  
  data = 0x3;
  uint16_t errCode = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_power_configure: Configure successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("MPU_power_configure: Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }

  errCode = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_power_configure: Read successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }
}

void MPU_Filter_FrameSync_configure()
{
  // Configure filter at mode 4, framesync at mode 0
  uint8_t configuration = 0x1;
  uint8_t data = configuration;
  uint16_t errCode = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_FILTER_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_Filter_FrameSync_configure: Configure successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("MPU_Filter_FrameSync_configure: Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }

  errCode = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_FILTER_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_Filter_FrameSync_configure: Read successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }
}

void MPU_Gyro_configure()
{
  // FS +- 500 */s
  uint8_t data = (0x1 << 3);
  uint16_t errCode = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_Gyro_configure: Configure successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("MPU_Gyro_configure: Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }

  errCode = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  if(errCode == HAL_OK)
  {
    printf("MPU_Gyro_configure: Read successful 0x%X\n", data);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }
}

void MPU_Accel_configure()
{
  // AFS +- 8 g
  uint16_t errCode = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &Acel_Config, 1, 100);
  if(errCode == HAL_OK)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
  {
    printf("MPU_power_configure: Failed %d and error Code %d\n", hi2c1.ErrorCode, errCode);
  }
}

/* Read data from gyro and accel */
void MPU_Data_read(st_MPU6050_Data* data)
{
  MPU_Accel_read(data);
  MPU_Gyro_read(data);
  #ifdef DEBUG
      printSensorData(&v_MPU6050_Data);
  #endif
}

void MPU_Accel_read(st_MPU6050_Data* mpuData)
{
  uint8_t data[6] = {0,0,0,0,0,0};
  uint16_t accel_x, accel_y, accel_z;
  /* Read Accel X */
  if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, 
                              MPU6050_ACCEL_XOUT_H_ADDR, 
                              I2C_MEMADD_SIZE_8BIT, 
                              data, 6, 100) == HAL_OK)
  {
    accel_x = (data[0] << 8) | (data[1]);
    accel_y = ((data[2] << 8) | (data[3]));
    accel_z = ((data[4] << 8) | (data[5]));
    mpuData->accel_x = (float)(accel_x);
    mpuData->accel_y = (float)(accel_y);
    mpuData->accel_z = (float)(accel_z);
  }
}

void MPU_Gyro_read(st_MPU6050_Data* mpuData)
{
  uint8_t data[6] = {0,0,0,0,0,0};
  /* Read Gyro X */
  if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, 
                              MPU6050_GYRO_XOUT_H_ADDR, 
                              I2C_MEMADD_SIZE_8BIT, 
                              data, 6, 100) == HAL_OK)
  {
    mpuData->gyro_x = (data[0] << 8) | (data[1]);
    mpuData->gyro_y = (data[2] << 8) | (data[3]);
    mpuData->gyro_z = (data[4] << 8) | (data[5]);
  }
}

void Accel_RollDegreeCal(st_MPU6050_Data* data, float* acc_roll)
{
  float acc_x, acc_z;
//	float acc_pitch;
	//normalized accelerometer readings. Constructed by taking
	//raw accelerometer readings and diving by accelerometer scaling
	//factor.
  acc_x = (data->accel_x) / A_R - ACEL_X_OFFSET;
	acc_z = (data->accel_z) / A_R + ACEL_Z_OFFSET;
//  acc_pitch = 180 * atan2(acc_y, acc_z) / M_PI;
  // Decide negative or positive roll degree
  // if acc_x < 3 => positive degree; else negative
  // 4 is the maximum value of accelerometer after scaling
  // Accel scale factor is 16384, MPU has 16 bit ADC => 2^16/16384 = 4
  *acc_roll = acc_x <= 2 ? (180.0 * atan2(acc_x, acc_z) / M_PI): -180.0 * atan2(4.0-acc_x, acc_z) / M_PI;
}

void Gyro_RollDegreeCalc(st_MPU6050_Data* data, float* gyroRate)
{
  float gyro_y;
  gyro_y = (data->gyro_y / G_R);
  // 2 is the offset value
  *gyroRate = gyro_y <= 500 ? -gyro_y + 2:-(gyro_y-1000 + 2);
}
void printSensorData(st_MPU6050_Data* data)
{
	float acc_x, acc_y, acc_z;
	float acc_pitch, acc_roll;


	acc_x = (data->accel_x) / A_R - ACEL_X_OFFSET;
	acc_y = (data->accel_y) / A_R - ACEL_Y_OFFSET;
	acc_z = (data->accel_z) / A_R + ACEL_Z_OFFSET;
  
  acc_pitch = 180 * atan2(acc_y, acc_z) / M_PI;
  acc_roll = 180 * atan2(acc_x, acc_z) / M_PI;
  
  // Decide negative or positive roll degree
  // if acc_x < 3 => positive degree; else negative
  // 4 is the maximum value of accelerometer after scaling
  acc_roll = acc_x < 3 ? (180 * atan2(acc_x, acc_z) / M_PI): -180 * atan2(4-acc_x, acc_z) / M_PI;
  printf("Pitch: %f \tRoll: %f \t", acc_pitch, acc_roll);
  printf("AcX=%f \tAcY=%f \tAcZ=%f \t", acc_x, acc_y, acc_z);
  printf("GyX=%f \tGyY=%f \tGyZ=%f\n", data->gyro_x, data->gyro_y, data->gyro_z);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

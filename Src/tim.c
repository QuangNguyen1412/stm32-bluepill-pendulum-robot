/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"
#include "i2c.h"



#define INTERVAL_TIME_MS    0.04
#define TIMER_1_PSC         63999
#define TIMER_1_FREQ        1000
#define TIMER_1_PERIOD      INTERVAL_TIME_MS*TIMER_1_FREQ
#define TIMER_2_PSC_32HZ    0
#define TIMER_2_ARR_20KHZ   1400
#define MIN_SPEED_SCALE     TIMER_2_ARR_20KHZ*1/5
#define MAX_SPEED_SCALE     TIMER_2_ARR_20KHZ
#define BALANCING_POINT     0
#define FORWARDING_POINT    -15.0
#define CONSTANT_KP         11
#define CONSTANT_KI         35
#define CONSTANT_KD         0.1
#define MIN_PWM_CYCLE_FOR_MOTOR     959     // Through observation
#define MAX_PWM_CYCLE_FOR_MOTOR     1050    // Through calculation: max battery, max operation voltage

#define _DIR_FOWARD() do { \
                          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);\
                          GPIOA->ODR |= (0x1 << 7);  \
                         } while(0)

#define _DIR_BACKWARD() do { \
                          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);\
                          GPIOA->ODR &= ~(0x1 << 7);  \
                         } while(0)

#define _SPEED_UPDATE(SPEED) do {\
                              TIM2->CCR3 = SPEED;\
                             } while(0)

#define _MOTOR_PWM_SCALE(PID_OUTPUT)    do  {\
                                            PID_OUTPUT = MIN_PWM_CYCLE_FOR_MOTOR+(PID_OUTPUT/TIMER_2_ARR_20KHZ)*(MAX_PWM_CYCLE_FOR_MOTOR-MIN_PWM_CYCLE_FOR_MOTOR);\
                                            } while(0)
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

volatile float current_error = 0;
volatile float last_error = 0;
volatile float stable_degree = BALANCING_POINT;
volatile float intergral_error = 0;
volatile float derivative_error = 0;
volatile uint8_t Kp = CONSTANT_KP;
volatile float Ki = CONSTANT_KI;
volatile float Kd = CONSTANT_KD;
volatile int16_t duty_cycle =1;

/* For debuging */
volatile int16_t motor_speed = 0;   	// Measured motor speed
float accel_angle = 0;
float gyro_angle = 0;
extern st_MPU6050_Data v_MPU6050_Data;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIMER_1_PSC;     // Timmer runs at 1kHz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIMER_1_PERIOD;          // Update event at 25Hz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;   // Timer run at 16 MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 800;    // PWM at 20kHz
  htim2.Init.Period = 1600;    // PWM at 20kHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void pwmInit()
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Initialize PB10 */  
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  GPIO_InitStruct.Pin = GPIO_PIN_10;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  __HAL_AFIO_REMAP_TIM2_ENABLE();
  /* Init Timer 2 channel 3 */
  TIM2->PSC = TIMER_2_PSC_32HZ;      // Timmer runs at freq 32MHz
  TIM2->ARR = TIMER_2_ARR_20KHZ;    // PWM runs at 2kHz
  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->EGR |= TIM_EGR_UG;
  TIM2->CCMR2 |= (0x6 << 4);  // Configure OC3M
  TIM2->CCMR2 |= (0x1 << 3);  // Configure OC3PE preload enable
  TIM2->CCMR2 |= (0x1 << 2);  // Configure OC3FE fast enable
  TIM2->CCMR2 &= ~(0x3);      // Configure CC3S
  TIM2->CCR3 = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM2->CCER |= (0x1 << 8);   // Enable PWM output
  TIM2->CCER &= ~(0x1 << 1);  // Polarity
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

void TIM1_IRQHandler()
{
//  st_MPU6050_Data data = {0,0,0,0,0,0};
  static float complement_angle;
  static float output = 0;
  MPU_Data_read(&v_MPU6050_Data);
  Accel_RollDegreeCal(&v_MPU6050_Data, &accel_angle);
  Gyro_RollDegreeCalc(&v_MPU6050_Data, &gyro_angle);
  // Complementary filter
  complement_angle = 0.95 * (complement_angle + gyro_angle * INTERVAL_TIME_MS) + 0.05 * accel_angle;
  motor_speed = complement_angle; // Debuggin
  current_error = stable_degree - (motor_speed<<1);
  if (((current_error >= -4) & (current_error <= 4)) |
                              (current_error >= 28) |
                              (current_error <= -28))
  {
    last_error = 0;
    current_error = 0;
    intergral_error = 0;
    derivative_error = 0;
    complement_angle = 0;
//    duty_cycle = 0;
    output = 0;
  }
  else
  { // Calculate PID
    if ((current_error < 0) & (intergral_error > 0))
    { // Ensure the intergral error is negative right away when error is negative
      intergral_error = 0;
    }
    else if ((current_error > 0) & (intergral_error < 0))
    {
      intergral_error = 0;
    }
    intergral_error += (current_error)*INTERVAL_TIME_MS;
    // Clamp the value
    if (intergral_error > MAX_SPEED_SCALE*3)
    {
      intergral_error = MAX_SPEED_SCALE*3;
    }
    else if (intergral_error < -MAX_SPEED_SCALE*3)
    {
      intergral_error = -MAX_SPEED_SCALE*3;
    }
    derivative_error = -(current_error - last_error)/INTERVAL_TIME_MS;
    if (derivative_error > MAX_SPEED_SCALE*3)
    {
      derivative_error = MAX_SPEED_SCALE*3;
    }
    else if (derivative_error < -MAX_SPEED_SCALE*3)
    {
      derivative_error = -MAX_SPEED_SCALE*3;
    }
    output = Kp*current_error + Ki*intergral_error + Kd*derivative_error;
    last_error = current_error;
//    duty_cycle = output;   // Define MAX duty cycle at 30/30
    // Direction

    if (output < 0)
    {
      if (output < -MAX_PWM_CYCLE_FOR_MOTOR)
      {
        output = MAX_PWM_CYCLE_FOR_MOTOR;
      }
      else
      {
        output = -output;
      }
      _DIR_BACKWARD();    // Robot going backward
    }
    else
    {
      if (output > MAX_PWM_CYCLE_FOR_MOTOR) // Define MAX duty cycle at 300 output error
      {
        output = MAX_PWM_CYCLE_FOR_MOTOR;
      }
      _DIR_FOWARD();
    }
    _MOTOR_PWM_SCALE(output);
  }
//  _SPEED_UPDATE(output);
  duty_cycle = output;
  _SPEED_UPDATE(duty_cycle);
  TIM1->SR &= ~(0x1);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

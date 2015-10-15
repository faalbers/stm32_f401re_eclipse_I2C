//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f4xx.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define MPU6050_ADDRESS   0xD0 // 0x68 bit shifted
#define MPU6050_WHO_AM_I  0x75

uint8_t aTxBuffer[] = {MPU6050_WHO_AM_I};
uint8_t aRxBuffer[4];

//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __I2C1_CLK_ENABLE();

  // Configure pin in output push/pull mode
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  I2C_HandleTypeDef I2cHandle;

  I2cHandle.Instance             = I2C1;

  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      = 100000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1     = 0x55;
  HAL_I2C_Init(&I2cHandle);

  if (HAL_I2C_IsDeviceReady(&I2cHandle, (uint16_t)MPU6050_ADDRESS, 2, 10000) == HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }

  HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)MPU6050_ADDRESS, (uint8_t*)aTxBuffer, 1, 10000);
  HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)MPU6050_ADDRESS, (uint8_t*)aRxBuffer, 1, 10000);

  if (HAL_I2C_IsDeviceReady(&I2cHandle, (uint16_t)MPU6050_ADDRESS, 2, 10000) == HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }

  // Infinite loop
  while (1)
    {
       // Add your code here.
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

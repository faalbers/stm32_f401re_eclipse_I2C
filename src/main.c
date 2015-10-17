//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f4xx.h"
#include "mpu6050.h"

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

int
main(int argc, char* argv[])
{
  // At this stage the system clock should have already been configured
  // at high speed.

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __GPIOA_CLK_ENABLE();

  // Configure pin in output push/pull mode
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  InitMPU();

  SetupMPU();

  if (ReadyMPU() == HAL_OK) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }

  //GetAccelMPU();
  //trace_printf("ACCEL_XOUT: 0x%04x\n", ACCEL_XOUT);

  // Infinite loop
  while (1)
    {
      // Add your code here.
      //GetAccelMPU();
      //trace_printf("ACCEL_XOUT_H: 0x%02x\n", ACCEL_XOUT_H);
      //trace_printf("ACCEL_XOUTI: %d\n", -20);
      //trace_printf("ACCEL_XOUT: %d\n", ACCEL_XOUT);
      //trace_printf("ACCEL_XOUTI: %d\n", ACCEL_XOUTI);
      //GetAccelAngles();
      //trace_printf("ACCEL_XANGLE: %d\n", (int)ACCEL_XANGLE);
      //trace_printf("ACCEL_XANGLE: %d\n", (int) (1.4f+1.6f));

      //GetGyroMPU();
      //trace_printf("GYRO_XOUT: %d\n", GYRO_XOUT);
    }
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

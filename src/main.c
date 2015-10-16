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

#define MPU6050_ADDRESS   (0xD0) // 0x68 bit shifted
#define MPU6050_TIMEOUT   (10000)

// Write Only
#define MPU6050_RA_XG_OFFS_TC     (0x00) //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC     (0x01) //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC     (0x02) //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN    (0x03) //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN    (0x04) //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN    (0x05) //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H      (0x06) //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC   (0x07)
#define MPU6050_RA_YA_OFFS_H      (0x08) //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC   (0x09)
#define MPU6050_RA_ZA_OFFS_H      (0x0A) //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC   (0x0B)
#define MPU6050_RA_XG_OFFS_USRH   (0x13) //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL   (0x14)
#define MPU6050_RA_YG_OFFS_USRH   (0x15) //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL   (0x16)
#define MPU6050_RA_ZG_OFFS_USRH   (0x17) //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL   (0x18)
#define MPU6050_RA_SMPLRT_DIV     (0x19)
#define MPU6050_RA_CONFIG         (0x1A)
#define MPU6050_RA_GYRO_CONFIG    (0x1B)
#define MPU6050_RA_ACCEL_CONFIG   (0x1C)
#define MPU6050_RA_FF_THR         (0x1D)
#define MPU6050_RA_FF_DUR         (0x1E)
#define MPU6050_RA_MOT_THR        (0x1F)
#define MPU6050_RA_MOT_DUR        (0x20)
#define MPU6050_RA_ZRMOT_THR      (0x21)
#define MPU6050_RA_ZRMOT_DUR      (0x22)
#define MPU6050_RA_FIFO_EN        (0x23)
#define MPU6050_RA_I2C_MST_CTRL   (0x24)
#define MPU6050_RA_I2C_SLV0_ADDR  (0x25)
#define MPU6050_RA_I2C_SLV0_REG   (0x26)
#define MPU6050_RA_I2C_SLV0_CTRL  (0x27)
#define MPU6050_RA_I2C_SLV1_ADDR  (0x28)
#define MPU6050_RA_I2C_SLV1_REG   (0x29)
#define MPU6050_RA_I2C_SLV1_CTRL  (0x2A)
#define MPU6050_RA_I2C_SLV2_ADDR  (0x2B)
#define MPU6050_RA_I2C_SLV2_REG   (0x2C)
#define MPU6050_RA_I2C_SLV2_CTRL  (0x2D)
#define MPU6050_RA_I2C_SLV3_ADDR  (0x2E)
#define MPU6050_RA_I2C_SLV3_REG   (0x2F)
#define MPU6050_RA_I2C_SLV3_CTRL  (0x30)
#define MPU6050_RA_I2C_SLV4_ADDR  (0x31)
#define MPU6050_RA_I2C_SLV4_REG   (0x32)
#define MPU6050_RA_I2C_SLV4_DO    (0x33)
#define MPU6050_RA_I2C_SLV4_CTRL  (0x34)
#define MPU6050_RA_I2C_SLV4_DI    (0x35)
#define MPU6050_RA_I2C_MST_STATUS (0x36)
#define MPU6050_RA_INT_PIN_CFG    (0x37)
#define MPU6050_RA_INT_ENABLE     (0x38)
#define MPU6050_RA_I2C_SLV0_DO (0x63)
#define MPU6050_RA_I2C_SLV1_DO (0x64)
#define MPU6050_RA_I2C_SLV2_DO (0x65)
#define MPU6050_RA_I2C_SLV3_DO (0x66)
#define MPU6050_RA_I2C_MST_DELAY_CTRL (0x67)
#define MPU6050_RA_SIGNAL_PATH_RESET (0x68)
#define MPU6050_RA_MOT_DETECT_CTRL (0x69)
#define MPU6050_RA_USER_CTRL (0x6A)
#define MPU6050_RA_PWR_MGMT_1 (0x6B)
#define MPU6050_RA_PWR_MGMT_2 (0x6C)
#define MPU6050_RA_BANK_SEL (0x6D)
#define MPU6050_RA_MEM_START_ADDR (0x6E)
#define MPU6050_RA_MEM_R_W (0x6F)
#define MPU6050_RA_DMP_CFG_1 (0x70)
#define MPU6050_RA_DMP_CFG_2 (0x71)
#define MPU6050_RA_FIFO_R_W (0x74)

// Read Only
#define MPU6050_RA_DMP_INT_STATUS (0x39)
#define MPU6050_RA_INT_STATUS     (0x3A)
#define MPU6050_RA_ACCEL_XOUT_H   (0x3B)
#define MPU6050_RA_ACCEL_XOUT_L   (0x3C)
#define MPU6050_RA_ACCEL_YOUT_H   (0x3D)
#define MPU6050_RA_ACCEL_YOUT_L   (0x3E)
#define MPU6050_RA_ACCEL_ZOUT_H   (0x3F)
#define MPU6050_RA_ACCEL_ZOUT_L   (0x40)
#define MPU6050_RA_TEMP_OUT_H     (0x41)
#define MPU6050_RA_TEMP_OUT_L     (0x42)
#define MPU6050_RA_GYRO_XOUT_H    (0x43)
#define MPU6050_RA_GYRO_XOUT_L    (0x44)
#define MPU6050_RA_GYRO_YOUT_H    (0x45)
#define MPU6050_RA_GYRO_YOUT_L    (0x46)
#define MPU6050_RA_GYRO_ZOUT_H    (0x47)
#define MPU6050_RA_GYRO_ZOUT_L    (0x48)
#define MPU6050_RA_EXT_SENS_DATA_00 (0x49)
#define MPU6050_RA_EXT_SENS_DATA_01 (0x4A)
#define MPU6050_RA_EXT_SENS_DATA_02 (0x4B)
#define MPU6050_RA_EXT_SENS_DATA_03 (0x4C)
#define MPU6050_RA_EXT_SENS_DATA_04 (0x4D)
#define MPU6050_RA_EXT_SENS_DATA_05 (0x4E)
#define MPU6050_RA_EXT_SENS_DATA_06 (0x4F)
#define MPU6050_RA_EXT_SENS_DATA_07 (0x50)
#define MPU6050_RA_EXT_SENS_DATA_08 (0x51)
#define MPU6050_RA_EXT_SENS_DATA_09 (0x52)
#define MPU6050_RA_EXT_SENS_DATA_10 (0x53)
#define MPU6050_RA_EXT_SENS_DATA_11 (0x54)
#define MPU6050_RA_EXT_SENS_DATA_12 (0x55)
#define MPU6050_RA_EXT_SENS_DATA_13 (0x56)
#define MPU6050_RA_EXT_SENS_DATA_14 (0x57)
#define MPU6050_RA_EXT_SENS_DATA_15 (0x58)
#define MPU6050_RA_EXT_SENS_DATA_16 (0x59)
#define MPU6050_RA_EXT_SENS_DATA_17 (0x5A)
#define MPU6050_RA_EXT_SENS_DATA_18 (0x5B)
#define MPU6050_RA_EXT_SENS_DATA_19 (0x5C)
#define MPU6050_RA_EXT_SENS_DATA_20 (0x5D)
#define MPU6050_RA_EXT_SENS_DATA_21 (0x5E)
#define MPU6050_RA_EXT_SENS_DATA_22 (0x5F)
#define MPU6050_RA_EXT_SENS_DATA_23 (0x60)
#define MPU6050_RA_MOT_DETECT_STATUS  (0x61)
#define MPU6050_RA_FIFO_COUNTH (0x72)
#define MPU6050_RA_FIFO_COUNTL (0x73)
#define MPU6050_RA_WHO_AM_I (0x75)

I2C_HandleTypeDef I2cHandle;

//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)

uint8_t WriteMPU(uint8_t addr, uint8_t data);
uint8_t ReadMPU(uint8_t addr);
HAL_StatusTypeDef ReadyMPU(void);

void TestMPU(void);
void SetupMPU(void);

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

  I2cHandle.Instance             = I2C1;

  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      = 100000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1     = 0x55;
  HAL_I2C_Init(&I2cHandle);

  SetupMPU();

  if (ReadyMPU() == HAL_OK) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }

  // Infinite loop
  while (1)
    {
       // Add your code here.
    }
}

uint8_t WriteMPU(uint8_t addr, uint8_t data)
{
  uint8_t ret=0;
  uint8_t buf[] = {addr, data};

  __disable_irq();
  if (HAL_I2C_Master_Transmit(&I2cHandle, MPU6050_ADDRESS, buf, 2, MPU6050_TIMEOUT) != HAL_OK) {
    ret=0xFF;
  }
  __enable_irq();
  return ret;
}

uint8_t ReadMPU(uint8_t addr)
{
  uint8_t data=0;

  __disable_irq();
  if (HAL_I2C_Master_Transmit(&I2cHandle, MPU6050_ADDRESS, &addr, 1, MPU6050_TIMEOUT) != HAL_OK) {
    data = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&I2cHandle, MPU6050_ADDRESS, &data, 1, MPU6050_TIMEOUT) != HAL_OK) {
    data = 0xFF;
  }

  error_w:
  __enable_irq();
  return data;
}

HAL_StatusTypeDef ReadyMPU(void)
{
  return HAL_I2C_IsDeviceReady(&I2cHandle, MPU6050_ADDRESS, 2, MPU6050_TIMEOUT);
}

void TestMPU(void)
{
  if (ReadyMPU() == HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }

  ReadMPU(MPU6050_RA_WHO_AM_I);

  if (ReadyMPU() == HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }

}

void SetupMPU(void)
{
  //Sets sample rate to 8000/1+7 = 1000Hz
  WriteMPU(MPU6050_RA_SMPLRT_DIV, 0x07);
  //trace_printf("SMPLRT_DIV: 0x%02x\n", ReadMPU(MPU6050_RA_SMPLRT_DIV));
  //Disable FSync, 256Hz DLPF
  WriteMPU(MPU6050_RA_CONFIG, 0x00);
  //Disable gyro self tests, scale of 500 degrees/s
  WriteMPU(MPU6050_RA_GYRO_CONFIG, 0b00001000);
  //Disable accel self tests, scale of +-2g, no DHPF
  WriteMPU(MPU6050_RA_ACCEL_CONFIG, 0x00);
  //Freefall threshold of |0mg|
  WriteMPU(MPU6050_RA_FF_THR, 0x00);
  //Freefall duration limit of 0
  WriteMPU(MPU6050_RA_FF_DUR, 0x00);
  //Motion threshold of 0mg
  WriteMPU(MPU6050_RA_MOT_THR, 0x00);
  //Motion duration of 0s
  WriteMPU(MPU6050_RA_MOT_DUR, 0x00);
  //Zero motion threshold
  WriteMPU(MPU6050_RA_ZRMOT_THR, 0x00);
  //Zero motion duration threshold
  WriteMPU(MPU6050_RA_ZRMOT_DUR, 0x00);
  //Disable sensor output to FIFO buffer
  WriteMPU(MPU6050_RA_FIFO_EN, 0x00);

  //AUX I2C setup
  //Sets AUX I2C to single master control, plus other config
  WriteMPU(MPU6050_RA_I2C_MST_CTRL, 0x00);
  //Setup AUX I2C slaves
  WriteMPU(MPU6050_RA_I2C_SLV0_ADDR, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV0_REG, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV0_CTRL, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV1_ADDR, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV1_REG, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV1_CTRL, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV2_ADDR, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV2_REG, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV2_CTRL, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV3_ADDR, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV3_REG, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV3_CTRL, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV4_ADDR, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV4_REG, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV4_DO, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV4_CTRL, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV4_DI, 0x00);

  //MPU6050_RA_I2C_MST_STATUS //Read-only
  //Setup INT pin and AUX I2C pass through
  WriteMPU(MPU6050_RA_INT_PIN_CFG, 0x00);
  //Enable data ready interrupt
  WriteMPU(MPU6050_RA_INT_ENABLE, 0x00);

  //Slave out, dont care
  WriteMPU(MPU6050_RA_I2C_SLV0_DO, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV1_DO, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV2_DO, 0x00);
  WriteMPU(MPU6050_RA_I2C_SLV3_DO, 0x00);
  //More slave config
  WriteMPU(MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
  //Reset sensor signal paths
  WriteMPU(MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
  //Motion detection control
  WriteMPU(MPU6050_RA_MOT_DETECT_CTRL, 0x00);
  //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
  WriteMPU(MPU6050_RA_USER_CTRL, 0x00);
  //Sets clock source to gyro reference w/ PLL
  WriteMPU(MPU6050_RA_PWR_MGMT_1, 0b00000010);
  //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
  WriteMPU(MPU6050_RA_PWR_MGMT_2, 0x00);

  //Data transfer to and from the FIFO buffer
  WriteMPU(MPU6050_RA_FIFO_R_W, 0x00);

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

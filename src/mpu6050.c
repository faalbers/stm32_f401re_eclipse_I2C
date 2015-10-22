/*
 * MPU.c
 *
 *  Created on: Oct 16, 2015
 *      Author: frank
 */

#include "mpu6050.h"
#include "diag/Trace.h"

static I2C_HandleTypeDef MPU_I2cHandle;
static int16_t  GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT;

HAL_StatusTypeDef ReadyMPU(void)
{
  return HAL_I2C_IsDeviceReady(&MPU_I2cHandle, MPU_ADDRESS, 2, MPU_TIMEOUT);
}

void ReadMPU(uint8_t addr, uint8_t *data)
{
  *data = 0x00;

  __disable_irq();
  if (HAL_I2C_Master_Transmit(&MPU_I2cHandle, MPU_ADDRESS, &addr, 1, MPU_TIMEOUT) != HAL_OK) {
    *data = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&MPU_I2cHandle, MPU_ADDRESS, data, 1, MPU_TIMEOUT) != HAL_OK) {
    *data = 0xFF;
  }

  error_w:
  __enable_irq();
}

// Read H and L bytes and add them together
// Probably needs Little Endian define
void ReadHLMPU(uint8_t addr, int16_t *data)
{
  ReadMPU(addr,((uint8_t*)data)+1);
  ReadMPU(addr+1,(uint8_t*)data);
}

uint8_t WriteMPU(uint8_t addr, uint8_t data)
{
  uint8_t ret=0;
  uint8_t buf[] = {addr, data};

  __disable_irq();
  if (HAL_I2C_Master_Transmit(&MPU_I2cHandle, MPU_ADDRESS, buf, 2, MPU_TIMEOUT) != HAL_OK) {
    ret=0xFF;
  }
  __enable_irq();
  return ret;
}

void InitMPU(void)
{
  GPIO_InitTypeDef MPU_GPIO_InitStruct;

  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
  */

  /* Peripheral clock enable */
  __GPIOB_CLK_ENABLE();
  __I2C1_CLK_ENABLE();

  MPU_GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  MPU_GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  MPU_GPIO_InitStruct.Pull = GPIO_PULLUP;
  MPU_GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  MPU_GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &MPU_GPIO_InitStruct);

  MPU_I2cHandle.Instance             = I2C1;

  MPU_I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  MPU_I2cHandle.Init.ClockSpeed      = 100000;
  MPU_I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  MPU_I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  MPU_I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  MPU_I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  MPU_I2cHandle.Init.OwnAddress1     = 0x55;
  HAL_I2C_Init(&MPU_I2cHandle);
}

void SetupMPU(void)
{
  SelfTestMPU();
  //uint8_t XG_TEST, YG_TEST, ZG_TEST;

  //return;

  // Set Gyro Full Scale Range to +/-250 deg/sec for self test
  //WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_FULL_SCALE_RANGE_250);

  // Get Gyro Factory Trim Value
  //ReadMPU(MPU_RA_SELF_TEST_X, &XG_TEST);
  //XG_TEST &= 0x1F;
  //ReadMPU(MPU_RA_SELF_TEST_Y, &YG_TEST);
  //YG_TEST &= 0x1F;
  //ReadMPU(MPU_RA_SELF_TEST_Z, &ZG_TEST);
  //ZG_TEST &= 0x1F;

  //trace_printf("XG_TEST: 0x%02x\n", XG_TEST);
  //trace_printf("XG_TEST: %d\n", XG_TEST);

}

void SelfTestMPU(void)
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  //float factoryTrim[6];

  // Configure the accelerometer for self-test
  WriteMPU(MPU_RA_ACCEL_CONFIG, MPU_ACCEL_SELF_TEST_XYZ); // Enable self test on all three axes and set accelerometer range to +/- 8 g
  WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_SELF_TEST_XYZ); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  HAL_Delay(250);  // Delay a while to let the device execute the self-test
  ReadMPU(MPU_RA_SELF_TEST_X, rawData); // X-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_Y, rawData+1); // Y-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_Z, rawData+2); // Z-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_A, rawData+3); // Mixed-axis self-test results

  trace_printf("Self_TEST_X: 0x%02x\n", rawData[0]);
  trace_printf("Self_TEST_Y: 0x%02x\n", rawData[1]);
  trace_printf("Self_TEST_Z: 0x%02x\n", rawData[2]);
  trace_printf("Self_TEST_A: 0x%02x\n", rawData[3]);

  // Extract the acceleration test results first
  selfTest[0] = ((rawData[0] & 0xE0) >> 3) | ((rawData[3] & 0x30) >> 4); // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = ((rawData[1] & 0xE0) >> 3) | ((rawData[3] & 0x0C) >> 2); // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = ((rawData[2] & 0xE0) >> 3) | (rawData[3] & 0x03); // ZA_TEST result is a five-bit unsigned integer
  trace_printf("Self_TEST_AX: 0x%02x\n", selfTest[0]);
  trace_printf("Self_TEST_AY: 0x%02x\n", selfTest[1]);
  trace_printf("Self_TEST_AZ: 0x%02x\n", selfTest[2]);
  /*
  // Extract the gyration test results first
  selfTest[3] = rawData[0] & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1] & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2] & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
  */

  /*
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
  factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
  factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

//  Output self-test results and factory trim calculation if desired
//  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
//  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
//  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
//  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
// To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
  }

  */
}

void SetupMPUOld(void)
{
  //Sets sample rate to 8000/1+7 = 1000Hz
  WriteMPU(MPU_RA_SMPLRT_DIV, 0x07);
  //trace_printf("SMPLRT_DIV: 0x%02x\n", ReadMPU(MPU_RA_SMPLRT_DIV));
  //Disable FSync, 256Hz DLPF
  WriteMPU(MPU_RA_CONFIG, 0x00);
  //Disable gyro self tests, scale of 500 degrees/s
  WriteMPU(MPU_RA_GYRO_CONFIG, 0b00001000);
  //Disable accel self tests, scale of +-2g, no DHPF
  WriteMPU(MPU_RA_ACCEL_CONFIG, 0x00);
  //Freefall threshold of |0mg|
  WriteMPU(MPU_RA_FF_THR, 0x00);
  //Freefall duration limit of 0
  WriteMPU(MPU_RA_FF_DUR, 0x00);
  //Motion threshold of 0mg
  WriteMPU(MPU_RA_MOT_THR, 0x00);
  //Motion duration of 0s
  WriteMPU(MPU_RA_MOT_DUR, 0x00);
  //Zero motion threshold
  WriteMPU(MPU_RA_ZRMOT_THR, 0x00);
  //Zero motion duration threshold
  WriteMPU(MPU_RA_ZRMOT_DUR, 0x00);
  //Disable sensor output to FIFO buffer
  WriteMPU(MPU_RA_FIFO_EN, 0x00);

  //AUX I2C setup
  //Sets AUX I2C to single master control, plus other config
  WriteMPU(MPU_RA_I2C_MST_CTRL, 0x00);
  //Setup AUX I2C slaves
  WriteMPU(MPU_RA_I2C_SLV0_ADDR, 0x00);
  WriteMPU(MPU_RA_I2C_SLV0_REG, 0x00);
  WriteMPU(MPU_RA_I2C_SLV0_CTRL, 0x00);
  WriteMPU(MPU_RA_I2C_SLV1_ADDR, 0x00);
  WriteMPU(MPU_RA_I2C_SLV1_REG, 0x00);
  WriteMPU(MPU_RA_I2C_SLV1_CTRL, 0x00);
  WriteMPU(MPU_RA_I2C_SLV2_ADDR, 0x00);
  WriteMPU(MPU_RA_I2C_SLV2_REG, 0x00);
  WriteMPU(MPU_RA_I2C_SLV2_CTRL, 0x00);
  WriteMPU(MPU_RA_I2C_SLV3_ADDR, 0x00);
  WriteMPU(MPU_RA_I2C_SLV3_REG, 0x00);
  WriteMPU(MPU_RA_I2C_SLV3_CTRL, 0x00);
  WriteMPU(MPU_RA_I2C_SLV4_ADDR, 0x00);
  WriteMPU(MPU_RA_I2C_SLV4_REG, 0x00);
  WriteMPU(MPU_RA_I2C_SLV4_DO, 0x00);
  WriteMPU(MPU_RA_I2C_SLV4_CTRL, 0x00);
  WriteMPU(MPU_RA_I2C_SLV4_DI, 0x00);

  //MPU_RA_I2C_MST_STATUS //Read-only
  //Setup INT pin and AUX I2C pass through
  WriteMPU(MPU_RA_INT_PIN_CFG, 0x00);
  //Enable data ready interrupt
  WriteMPU(MPU_RA_INT_ENABLE, 0x00);

  //Slave out, dont care
  WriteMPU(MPU_RA_I2C_SLV0_DO, 0x00);
  WriteMPU(MPU_RA_I2C_SLV1_DO, 0x00);
  WriteMPU(MPU_RA_I2C_SLV2_DO, 0x00);
  WriteMPU(MPU_RA_I2C_SLV3_DO, 0x00);
  //More slave config
  WriteMPU(MPU_RA_I2C_MST_DELAY_CTRL, 0x00);
  //Reset sensor signal paths
  WriteMPU(MPU_RA_SIGNAL_PATH_RESET, 0x00);
  //Motion detection control
  WriteMPU(MPU_RA_MOT_DETECT_CTRL, 0x00);
  //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
  WriteMPU(MPU_RA_USER_CTRL, 0x00);
  //Sets clock source to gyro reference w/ PLL
  WriteMPU(MPU_RA_PWR_MGMT_1, 0b00000010);
  //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
  WriteMPU(MPU_RA_PWR_MGMT_2, 0x00);

  //Data transfer to and from the FIFO buffer
  WriteMPU(MPU_RA_FIFO_R_W, 0x00);

}

void GetGyroMPU(void)
{
  ReadHLMPU(MPU_RA_GYRO_XOUT_H, &GYRO_XOUT);
  ReadHLMPU(MPU_RA_GYRO_YOUT_H, &GYRO_YOUT);
  ReadHLMPU(MPU_RA_GYRO_ZOUT_H, &GYRO_ZOUT);
}


void GetAccelMPU(void)
{
  /*
  ReadMPU(MPU_RA_ACCEL_XOUT_H, &H_BYTE);
  ReadMPU(MPU_RA_ACCEL_XOUT_L, &L_BYTE);
  ACCEL_XOUT = ((H_BYTE<<8)|L_BYTE);
  ReadMPU(MPU_RA_ACCEL_YOUT_H, &ACCEL_YOUT_H);
  ReadMPU(MPU_RA_ACCEL_YOUT_L, &ACCEL_YOUT_L);
  ReadMPU(MPU_RA_ACCEL_ZOUT_H, &ACCEL_ZOUT_H);
  ReadMPU(MPU_RA_ACCEL_ZOUT_L, &ACCEL_ZOUT_L);

  //ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
  ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
  ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);
  */

  //ACCEL_XOUTI = *(int16_t*)&ACCEL_XOUT;
}

//Converts the already acquired accelerometer data into 3D euler angles
//void GetAccelAngles(void)
//{
//
//  //ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)));
//  //ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)));
//}

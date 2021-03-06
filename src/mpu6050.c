/*
 * MPU.c
 *
 *  Created on: Oct 16, 2015
 *      Author: frank
 */

#include "mpu6050.h"
#include <math.h>
#include "diag/Trace.h"

static I2C_HandleTypeDef MPU_I2cHandle;

HAL_StatusTypeDef ReadyMPU(void)
{
  return HAL_I2C_IsDeviceReady(&MPU_I2cHandle, MPU_ADDRESS, 2, MPU_TIMEOUT);
}

void ReadMPU(uint8_t addr, uint8_t *data, uint16_t bytes)
{
  for (uint16_t ii = 0; ii < bytes; ii++) *(data+ii) = 0x00;

  __disable_irq();
  if (HAL_I2C_Master_Transmit(&MPU_I2cHandle, MPU_ADDRESS, &addr, 1, MPU_TIMEOUT) != HAL_OK)
    goto error_w;
  HAL_I2C_Master_Receive(&MPU_I2cHandle, MPU_ADDRESS, data, bytes, MPU_TIMEOUT);

  error_w:
  __enable_irq();
}

// Read H and L bytes and add them together
// Probably needs Little Endian define
void ReadHLMPU(uint8_t addr, int16_t *data)
{
  ReadMPU(addr,((uint8_t*)data)+1,1);
  ReadMPU(addr+1,(uint8_t*)data,1);
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

HAL_StatusTypeDef InitMPU(void)
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
  if (HAL_I2C_Init(&MPU_I2cHandle) != HAL_OK)
    return HAL_ERROR;

  return SetupMPU();
}

HAL_StatusTypeDef SetupMPU(void)
{
  // Make sure MPU device exists and I2C protocol
  // is working
  if (ReadyMPU() != HAL_OK) return HAL_ERROR;

  // Take MPU out of sleep mode
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_WAKE_UP);

  // Run self test
  if (SelfTestMPU() != HAL_OK) return HAL_ERROR;

  // Calibrate
  CalibrateMPU();

  return HAL_OK;
}

void CalibrateMPU(void)
{
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};;

  GetBiasesMPU(gyro_bias, accel_bias);

  /*
  for (uint8_t ii = 0; ii < 3; ii++) {
    trace_printf("Accel Bias: %d\n", accel_bias[ii]);
    trace_printf("Gyro Bias: %d\n", gyro_bias[ii]);
  }
  */

  SetBiasesMPU(gyro_bias, accel_bias);

  return;
  float dest1[3], dest2[3];
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  int16_t ii, packet_count, fifo_count;
  //int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  int16_t gyro_temp[3] = {0, 0, 0}, accel_temp[3] = {0, 0, 0};
  uint16_t  gyrosensitivity  = 131;   // = 1 deg/sec  ... based on setting of -/+ 250 deg/sec full range
  uint16_t  accelsensitivity = 16384;  // = 1 g ... based on setting of -/+ 2g full range

  // Reset device, reset all registers, clear gyro and accelerometer bias registers
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_DEVICE_RESET);
  HAL_Delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_CLOCK_PLL_X_GYRO);
  WriteMPU(MPU_RA_PWR_MGMT_2, MPU_PWR_MGMT_2_RESET);
  HAL_Delay(200);

  // Configure device for bias calculation
  WriteMPU(MPU_RA_INT_ENABLE, MPU_INT_ENABLE_DISSABLE_ALL); // Disable all interrupts
  WriteMPU(MPU_RA_FIFO_EN, MPU_FIFO_EN_DISSABLE_ALL); // Disable FIFO
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_CLOCK_INTERNAL); // Turn on internal clock source
  WriteMPU(MPU_RA_I2C_MST_CTRL, MPU_I2C_MST_RESET); // Disable I2C master
  WriteMPU(MPU_RA_USER_CTRL, MPU_USER_CTRL_RESET); // Disable FIFO and I2C master modes
  WriteMPU(MPU_RA_USER_CTRL, MPU_USER_CTRL_FIFO_RESET); // Reset FIFO
  HAL_Delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  WriteMPU(MPU_RA_CONFIG, MPU_CONFIG_DLPF_184_188);      // Set low-pass filter to 188 Hz
  WriteMPU(MPU_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz = 1kHz/(1-SMPLRT_DIV)
  WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_FULL_SCALE_RANGE_250);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  WriteMPU(MPU_RA_ACCEL_CONFIG, MPU_ACCEL_FULL_SCALE_RANGE_2); // Set accelerometer full-scale to 2 g, maximum sensitivity

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  WriteMPU(MPU_RA_USER_CTRL, MPU_USER_CTRL_FIFO_EN);   // Enable FIFO
  WriteMPU(MPU_RA_FIFO_EN, (MPU_FIFO_EN_XG | MPU_FIFO_EN_YG | MPU_FIFO_EN_ZG | MPU_FIFO_EN_ACCEL)); // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  HAL_Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  WriteMPU(MPU_RA_FIFO_EN, MPU_FIFO_EN_DISSABLE_ALL); // Disable gyro and accelerometer sensors for FIFO
  ReadHLMPU(MPU_RA_FIFO_COUNTH, &fifo_count); // read FIFO sample count
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  //trace_printf("FIFO count: %d\n", fifo_count);
  //trace_printf("FIFO count: %d\n", packet_count);

  for (ii = 0; ii < packet_count; ii++) {
    // read data for averaging
    ReadMPU(MPU_RA_FIFO_R_W, data,12);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  /*
  for (ii = 0; ii < 3; ii++) {
    trace_printf("Accel Bias: %d\n", accel_bias[ii]);
    trace_printf("Gyro Bias: %d\n", gyro_bias[ii]);
  }
  */

  // Remove 1g gravity from the z-axis accelerometer bias calculation
  // Make sure board is faced upwards
  accel_bias[2] -= (int32_t) accelsensitivity;

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  // Biases are additive, so change sign on calculated average gyro biases
  //NOTE: Don't understand division by 4
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  WriteMPU(MPU_RA_XG_OFFS_USRH, data[0]);
  WriteMPU(MPU_RA_XG_OFFS_USRL, data[1]);
  WriteMPU(MPU_RA_YG_OFFS_USRH, data[2]);
  WriteMPU(MPU_RA_YG_OFFS_USRL, data[3]);
  WriteMPU(MPU_RA_ZG_OFFS_USRH, data[4]);
  WriteMPU(MPU_RA_ZG_OFFS_USRL, data[5]);

    // construct gyro bias in deg/s for later manual subtraction
  //NOTE: Don't uderstand this conversion
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  ReadMPU(MPU_RA_XA_OFFS_USRH, &data[0], 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  ReadMPU(MPU_RA_YA_OFFS_USRH, &data[0], 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  ReadMPU(MPU_RA_ZA_OFFS_USRH, &data[0], 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
  dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
  dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void GetBiasesMPU(int32_t *gyro_bias, int32_t *accel_bias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  int16_t ii, packet_count, fifo_count;
  int16_t gyro_temp[3] = {0, 0, 0}, accel_temp[3] = {0, 0, 0};
  int32_t  accel_sense = MPU_ACCEL_TEST_SENS;  // = 1 g

  // Zero out the biases
  for (ii = 0; ii < 3; ii++) gyro_bias[ii] = accel_bias[ii] = 0;

  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_CLOCK_PLL_X_GYRO);
  WriteMPU(MPU_RA_PWR_MGMT_2, MPU_PWR_MGMT_2_RESET);
  HAL_Delay(200);

  // Configure device for bias calculation
  WriteMPU(MPU_RA_INT_ENABLE, MPU_INT_ENABLE_DISSABLE_ALL); // Disable all interrupts
  WriteMPU(MPU_RA_FIFO_EN, MPU_FIFO_EN_DISSABLE_ALL); // Disable FIFO
  WriteMPU(MPU_RA_PWR_MGMT_1, MPU_PWR_MGMT_1_CLOCK_INTERNAL); // Turn on internal clock source
  WriteMPU(MPU_RA_I2C_MST_CTRL, MPU_I2C_MST_RESET); // Disable I2C master
  WriteMPU(MPU_RA_USER_CTRL, (MPU_USER_CTRL_FIFO_RESET | MPU_USER_CTRL_DMP_RESET)); // Reset FIFO
  HAL_Delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  WriteMPU(MPU_RA_CONFIG, MPU_CONFIG_DLPF_184_188);      // Set low-pass filter to 188 Hz
  WriteMPU(MPU_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz = 1kHz/(1-SMPLRT_DIV)
  WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_TEST_FULL_RANGE);  // Set gyro test full-scale
  WriteMPU(MPU_RA_ACCEL_CONFIG, MPU_ACCEL_TEST_FULL_RANGE); // Set accelerometer test full-scale
  HAL_Delay(200);

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  WriteMPU(MPU_RA_USER_CTRL, MPU_USER_CTRL_FIFO_EN);   // Enable FIFO
  WriteMPU(MPU_RA_FIFO_EN, (MPU_FIFO_EN_XG | MPU_FIFO_EN_YG | MPU_FIFO_EN_ZG | MPU_FIFO_EN_ACCEL)); // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  HAL_Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  WriteMPU(MPU_RA_FIFO_EN, MPU_FIFO_EN_DISSABLE_ALL); // Disable gyro and accelerometer sensors for FIFO
  ReadHLMPU(MPU_RA_FIFO_COUNTH, &fifo_count); // read FIFO sample count
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  // Average samples from the FIFO
  for (ii = 0; ii < packet_count; ii++) {
    // read data for averaging
    ReadMPU(MPU_RA_FIFO_R_W, data,12);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Remove 1g gravity from the z-axis accelerometer bias calculation
  // Make sure board is faced upwards
  if (accel_bias[2] > 0)
    accel_bias[2] -= accel_sense;
  else
    accel_bias[2] += accel_sense;

}

void SetBiasesMPU(int32_t *gyro_bias, int32_t *accel_bias)
{
  SetGyroBiasesMPU(gyro_bias);
  SetAccelBiasesMPU(accel_bias);
}

void SetGyroBiasesMPU(int32_t *gyro_bias)
{
  uint8_t data[6];

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  // Biases are additive, so change sign on calculated average gyro biases
  // Move to H and L
  // The format is in the +- 1000 dps range.
  data[0] = (-(int16_t)gyro_bias[0]  >> 8) & 0xFF;
  data[1] = (-(int16_t)gyro_bias[0])       & 0xFF;
  data[2] = (-(int16_t)gyro_bias[1]  >> 8) & 0xFF;
  data[3] = (-(int16_t)gyro_bias[1])       & 0xFF;
  data[4] = (-(int16_t)gyro_bias[2]  >> 8) & 0xFF;
  data[5] = (-(int16_t)gyro_bias[2])       & 0xFF;

  // Push gyro biases to hardware registers
  WriteMPU(MPU_RA_XG_OFFS_USRH, data[0]);
  WriteMPU(MPU_RA_XG_OFFS_USRL, data[1]);
  WriteMPU(MPU_RA_YG_OFFS_USRH, data[2]);
  WriteMPU(MPU_RA_YG_OFFS_USRL, data[3]);
  WriteMPU(MPU_RA_ZG_OFFS_USRH, data[4]);
  WriteMPU(MPU_RA_ZG_OFFS_USRL, data[5]);

}

void SetAccelBiasesMPU(int32_t *accel_bias)
{
  int16_t accel_bias_offset[3] = {0, 0, 0};
  uint8_t data[6];

  // Read accel offset cancellations registers (factory trim values)
  // The format is in the +-8 G range.
  // In addition, bit 0 of the lower byte must be preserved since it
  // is used for temperature compensation calculations.

  ReadHLMPU(MPU_RA_XA_OFFS_USRH, &accel_bias_offset[0]);
  ReadHLMPU(MPU_RA_YA_OFFS_USRH, &accel_bias_offset[2]);
  ReadHLMPU(MPU_RA_ZA_OFFS_USRH, &accel_bias_offset[4]);

  // bit 0 of the lower byte must be preserved since it is used for
  // temperature compensation calculations.
  // Negate because it is a subtracting value
  accel_bias_offset[0] -= (accel_bias[0] & ~(0x01));
  accel_bias_offset[2] -= (accel_bias[2] & ~(0x01));
  accel_bias_offset[4] -= (accel_bias[4] & ~(0x01));

  // The register is initialized with OTP factory trim values.
  data[0] = (accel_bias_offset[0]  >> 8) & 0xFF;
  data[1] = (accel_bias_offset[0])       & 0xFF;
  data[2] = (accel_bias_offset[1]  >> 8) & 0xFF;
  data[3] = (accel_bias_offset[1])       & 0xFF;
  data[4] = (accel_bias_offset[2]  >> 8) & 0xFF;
  data[5] = (accel_bias_offset[2])       & 0xFF;

  // Push accel biases to hardware registers
  WriteMPU(MPU_RA_XA_OFFS_USRH, data[0]);
  WriteMPU(MPU_RA_XA_OFFS_USRL, data[1]);
  WriteMPU(MPU_RA_YA_OFFS_USRH, data[2]);
  WriteMPU(MPU_RA_YA_OFFS_USRL, data[3]);
  WriteMPU(MPU_RA_ZA_OFFS_USRH, data[4]);
  WriteMPU(MPU_RA_ZA_OFFS_USRL, data[5]);

}

HAL_StatusTypeDef SelfTestMPU(void)
{
  uint8_t i;
  uint8_t rawData[4];
  uint8_t selfTest[6];
  int16_t TestOut[6], Out[6], Str[6];
  float FT[6], St;

  // Set Factory trims
  // Get MPU's self test raw data
  ReadMPU(MPU_RA_SELF_TEST_X, rawData,1); // X-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_Y, rawData+1,1); // Y-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_Z, rawData+2,1); // Z-axis self-test results
  ReadMPU(MPU_RA_SELF_TEST_A, rawData+3,1); // Mixed-axis self-test results


  // Extract the acceleration self test data
  selfTest[0] = ((rawData[0] & 0xE0) >> 3) | ((rawData[3] & 0x30) >> 4); // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = ((rawData[1] & 0xE0) >> 3) | ((rawData[3] & 0x0C) >> 2); // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = ((rawData[2] & 0xE0) >> 3) | (rawData[3] & 0x03); // ZA_TEST result is a five-bit unsigned integer

  // Extract the gyro self test data
  selfTest[3] = rawData[0] & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1] & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2] & 0x1F ; // ZG_TEST result is a five-bit unsigned integer

  // Calculate Factory trim values
  for (i = 0; i < 3; i++) {
    FT[i] = 1392.64f*pow(2.705882353f,(float)(selfTest[i]-1)/30.0f);
  }
  for (i = 3; i < 6; i++) {
    FT[i] = 3275.0f*pow(1.046f,(float)(selfTest[i]-1));
  }

  // Get accelometer and gyro values during test
  // Enable self test on all three axes and set accelerometer range to +/- 8 g
  WriteMPU(MPU_RA_ACCEL_CONFIG, MPU_ACCEL_SELF_TEST_XYZ);
  // Enable self test on all three axes and set gyro range to +/- 250 deg/sec
  WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_SELF_TEST_XYZ);
  // Delay a while to let the device execute the self-test
  HAL_Delay(250);

  ReadHLMPU(MPU_RA_ACCEL_XOUT_H, TestOut);
  ReadHLMPU(MPU_RA_ACCEL_YOUT_H, TestOut+1);
  ReadHLMPU(MPU_RA_ACCEL_ZOUT_H, TestOut+2);
  ReadHLMPU(MPU_RA_GYRO_XOUT_H, TestOut+3);
  ReadHLMPU(MPU_RA_GYRO_YOUT_H, TestOut+4);
  ReadHLMPU(MPU_RA_GYRO_ZOUT_H, TestOut+5);

  // Turn off test mode and get normal values
  // Set accelerometer range to +/- 8 g
  WriteMPU(MPU_RA_ACCEL_CONFIG, MPU_ACCEL_FULL_SCALE_RANGE_8);
  // Set gyro range to +/- 250 deg/sec
  WriteMPU(MPU_RA_GYRO_CONFIG, MPU_GYRO_FULL_SCALE_RANGE_250);

  // Delay a while to let the device execute normally
  HAL_Delay(250);

  ReadHLMPU(MPU_RA_ACCEL_XOUT_H, Out);
  ReadHLMPU(MPU_RA_ACCEL_YOUT_H, Out+1);
  ReadHLMPU(MPU_RA_ACCEL_ZOUT_H, Out+2);
  ReadHLMPU(MPU_RA_GYRO_XOUT_H, Out+3);
  ReadHLMPU(MPU_RA_GYRO_YOUT_H, Out+4);
  ReadHLMPU(MPU_RA_GYRO_ZOUT_H, Out+5);

  for (i = 0; i < 6; i++) {
    Str[i] = TestOut[i]-Out[i];
    if (i == 4) { St = (-Str[i] - FT[i])/FT[i]; }
    else { St = (Str[i] - FT[i])/FT[i]; }
    if ( (int8_t) (St*100.0f) > 14 ) return HAL_ERROR; // 14% as per data sheets
  }

  return HAL_OK;
}

void SetupMPUOld(void)
{
  //Sets sample rate to 8000/1+7 = 1000Hz
  WriteMPU(MPU_RA_SMPLRT_DIV, 0x07);
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
  ReadHLMPU(MPU_RA_GYRO_XOUT_H, &MPU_GYRO_XOUT);
  ReadHLMPU(MPU_RA_GYRO_YOUT_H, &MPU_GYRO_YOUT);
  ReadHLMPU(MPU_RA_GYRO_ZOUT_H, &MPU_GYRO_ZOUT);
}


void GetAccelMPU(void)
{
  ReadHLMPU(MPU_RA_ACCEL_XOUT_H, &MPU_ACCEL_XOUT);
  ReadHLMPU(MPU_RA_ACCEL_YOUT_H, &MPU_ACCEL_YOUT);
  ReadHLMPU(MPU_RA_ACCEL_ZOUT_H, &MPU_ACCEL_ZOUT);
}


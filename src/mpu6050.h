/*
 * MPU.h
 *
 *  Created on: Oct 16, 2015
 *      Author: frank
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx.h"

// Device settings
#define MPU_ADDRESS   (0xD0) // 0x68 bit shifted
#define MPU_TIMEOUT   (10000)

// Write Only
#define MPU_RA_XG_OFFS_TC     (0x00) //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC     (0x01) //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC     (0x02) //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN    (0x03) //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN    (0x04) //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN    (0x05) //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H      (0x06) //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC   (0x07)
#define MPU_RA_YA_OFFS_H      (0x08) //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC   (0x09)
#define MPU_RA_ZA_OFFS_H      (0x0A) //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC   (0x0B)
#define MPU_RA_XG_OFFS_USRH   (0x13) //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL   (0x14)
#define MPU_RA_YG_OFFS_USRH   (0x15) //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL   (0x16)
#define MPU_RA_ZG_OFFS_USRH   (0x17) //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL   (0x18)
#define MPU_RA_SMPLRT_DIV     (0x19)
#define MPU_RA_CONFIG         (0x1A)
#define MPU_RA_GYRO_CONFIG    (0x1B)
#define MPU_RA_ACCEL_CONFIG   (0x1C)
#define MPU_RA_FF_THR         (0x1D)
#define MPU_RA_FF_DUR         (0x1E)
#define MPU_RA_MOT_THR        (0x1F)
#define MPU_RA_MOT_DUR        (0x20)
#define MPU_RA_ZRMOT_THR      (0x21)
#define MPU_RA_ZRMOT_DUR      (0x22)
#define MPU_RA_FIFO_EN        (0x23)
#define MPU_RA_I2C_MST_CTRL   (0x24)
#define MPU_RA_I2C_SLV0_ADDR  (0x25)
#define MPU_RA_I2C_SLV0_REG   (0x26)
#define MPU_RA_I2C_SLV0_CTRL  (0x27)
#define MPU_RA_I2C_SLV1_ADDR  (0x28)
#define MPU_RA_I2C_SLV1_REG   (0x29)
#define MPU_RA_I2C_SLV1_CTRL  (0x2A)
#define MPU_RA_I2C_SLV2_ADDR  (0x2B)
#define MPU_RA_I2C_SLV2_REG   (0x2C)
#define MPU_RA_I2C_SLV2_CTRL  (0x2D)
#define MPU_RA_I2C_SLV3_ADDR  (0x2E)
#define MPU_RA_I2C_SLV3_REG   (0x2F)
#define MPU_RA_I2C_SLV3_CTRL  (0x30)
#define MPU_RA_I2C_SLV4_ADDR  (0x31)
#define MPU_RA_I2C_SLV4_REG   (0x32)
#define MPU_RA_I2C_SLV4_DO    (0x33)
#define MPU_RA_I2C_SLV4_CTRL  (0x34)
#define MPU_RA_I2C_SLV4_DI    (0x35)
#define MPU_RA_I2C_MST_STATUS (0x36)
#define MPU_RA_INT_PIN_CFG    (0x37)
#define MPU_RA_INT_ENABLE     (0x38)
#define MPU_RA_BANK_SEL       (0x6D)
#define MPU_RA_MEM_START_ADDR (0x6E)
#define MPU_RA_MEM_R_W        (0x6F)
#define MPU_RA_DMP_CFG_1      (0x70)
#define MPU_RA_DMP_CFG_2      (0x71)
#define MPU_RA_FIFO_R_W       (0x74)

// Read/Write
#define MPU_RA_SELF_TEST_X    (0x0D)
#define MPU_RA_SELF_TEST_Y    (0x0E)
#define MPU_RA_SELF_TEST_Z    (0x0F)
#define MPU_RA_SELF_TEST_A    (0x10)
#define MPU_RA_I2C_SLV0_DO    (0x63)
#define MPU_RA_I2C_SLV1_DO    (0x64)
#define MPU_RA_I2C_SLV2_DO    (0x65)
#define MPU_RA_I2C_SLV3_DO    (0x66)
#define MPU_RA_I2C_MST_DELAY_CTRL (0x67)
#define MPU_RA_SIGNAL_PATH_RESET (0x68)
#define MPU_RA_MOT_DETECT_CTRL (0x69)
#define MPU_RA_USER_CTRL      (0x6A)
#define MPU_RA_PWR_MGMT_1     (0x6B)
#define MPU_RA_PWR_MGMT_2     (0x6C)

// Read Only
#define MPU_RA_DMP_INT_STATUS (0x39)
#define MPU_RA_INT_STATUS     (0x3A)
#define MPU_RA_ACCEL_XOUT_H   (0x3B)
#define MPU_RA_ACCEL_XOUT_L   (0x3C)
#define MPU_RA_ACCEL_YOUT_H   (0x3D)
#define MPU_RA_ACCEL_YOUT_L   (0x3E)
#define MPU_RA_ACCEL_ZOUT_H   (0x3F)
#define MPU_RA_ACCEL_ZOUT_L   (0x40)
#define MPU_RA_TEMP_OUT_H     (0x41)
#define MPU_RA_TEMP_OUT_L     (0x42)
#define MPU_RA_GYRO_XOUT_H    (0x43)
#define MPU_RA_GYRO_XOUT_L    (0x44)
#define MPU_RA_GYRO_YOUT_H    (0x45)
#define MPU_RA_GYRO_YOUT_L    (0x46)
#define MPU_RA_GYRO_ZOUT_H    (0x47)
#define MPU_RA_GYRO_ZOUT_L    (0x48)
#define MPU_RA_EXT_SENS_DATA_00 (0x49)
#define MPU_RA_EXT_SENS_DATA_01 (0x4A)
#define MPU_RA_EXT_SENS_DATA_02 (0x4B)
#define MPU_RA_EXT_SENS_DATA_03 (0x4C)
#define MPU_RA_EXT_SENS_DATA_04 (0x4D)
#define MPU_RA_EXT_SENS_DATA_05 (0x4E)
#define MPU_RA_EXT_SENS_DATA_06 (0x4F)
#define MPU_RA_EXT_SENS_DATA_07 (0x50)
#define MPU_RA_EXT_SENS_DATA_08 (0x51)
#define MPU_RA_EXT_SENS_DATA_09 (0x52)
#define MPU_RA_EXT_SENS_DATA_10 (0x53)
#define MPU_RA_EXT_SENS_DATA_11 (0x54)
#define MPU_RA_EXT_SENS_DATA_12 (0x55)
#define MPU_RA_EXT_SENS_DATA_13 (0x56)
#define MPU_RA_EXT_SENS_DATA_14 (0x57)
#define MPU_RA_EXT_SENS_DATA_15 (0x58)
#define MPU_RA_EXT_SENS_DATA_16 (0x59)
#define MPU_RA_EXT_SENS_DATA_17 (0x5A)
#define MPU_RA_EXT_SENS_DATA_18 (0x5B)
#define MPU_RA_EXT_SENS_DATA_19 (0x5C)
#define MPU_RA_EXT_SENS_DATA_20 (0x5D)
#define MPU_RA_EXT_SENS_DATA_21 (0x5E)
#define MPU_RA_EXT_SENS_DATA_22 (0x5F)
#define MPU_RA_EXT_SENS_DATA_23 (0x60)
#define MPU_RA_MOT_DETECT_STATUS  (0x61)
#define MPU_RA_FIFO_COUNTH (0x72)
#define MPU_RA_FIFO_COUNTL (0x73)
#define MPU_RA_WHO_AM_I (0x75)

// Settings
#define MPU_ACCEL_FULL_SCALE_RANGE_2   ((uint8_t)0x00) // +/- 2 g
#define MPU_ACCEL_FULL_SCALE_RANGE_4   ((uint8_t)0x08) // +/- 4 g
#define MPU_ACCEL_FULL_SCALE_RANGE_8   ((uint8_t)0x10) // +/- 8 g
#define MPU_ACCEL_FULL_SCALE_RANGE_16  ((uint8_t)0x18) // +/- 16 g

#define MPU_GYRO_FULL_SCALE_RANGE_250   ((uint8_t)0x00) // +/- 250  deg/sec
#define MPU_GYRO_FULL_SCALE_RANGE_500   ((uint8_t)0x08) // +/- 500  deg/sec
#define MPU_GYRO_FULL_SCALE_RANGE_1000  ((uint8_t)0x10) // +/- 1000 deg/sec
#define MPU_GYRO_FULL_SCALE_RANGE_2000  ((uint8_t)0x18) // +/- 2000 deg/sec

#define MPU_ACCEL_SELF_TEST_XYZ         ((uint8_t)0xF0) // Enable self test on all three axes and set accelerometer range to +/- 8 g
#define MPU_GYRO_SELF_TEST_XYZ          ((uint8_t)0xE0) // Enable self test on all three axes and set gyro range to +/- 250 deg/sec

int16_t  MPU_GYRO_XOUT, MPU_GYRO_YOUT, MPU_GYRO_ZOUT;
int16_t  MPU_ACCEL_XOUT, MPU_ACCEL_YOUT, MPU_ACCEL_ZOUT;

// function declarations
HAL_StatusTypeDef ReadyMPU(void);
void ReadMPU(uint8_t addr, uint8_t *data);
void ReadHLMPU(uint8_t addr, int16_t *data);
uint8_t WriteMPU(uint8_t addr, uint8_t data);
HAL_StatusTypeDef InitMPU(void);
HAL_StatusTypeDef SetupMPU(void);
HAL_StatusTypeDef SelfTestMPU(void);
void SetupMPUOld(void);
void GetGyroMPU(void);
void GetAccelMPU(void);

#endif /* MPU6050_H_ */

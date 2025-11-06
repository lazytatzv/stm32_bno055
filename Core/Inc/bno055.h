/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.h
  * @brief          : Header for BNO055 sensor driver
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BNO055_H
#define __BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* BNO055 I2C Address */
#define BNO055_I2C_ADDR         (0x28 << 1)  // ADR pin = LOW
#define BNO055_I2C_ADDR_ALT     (0x29 << 1)  // ADR pin = HIGH

/* BNO055 Register Map */
#define BNO055_CHIP_ID_ADDR     0x00
#define BNO055_CALIB_STAT_ADDR  0x35
#define BNO055_INTR_STAT_ADDR   0x37
#define BNO055_SYS_STAT_ADDR    0x39
#define BNO055_SYS_ERR_ADDR     0x3A

#define BNO055_PAGE_ID_ADDR     0x07
#define BNO055_OPR_MODE_ADDR    0x3D
#define BNO055_PWR_MODE_ADDR    0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_UNIT_SEL_ADDR    0x3B

/* Configuration registers (Page 1) */
#define BNO055_ACC_CONFIG_ADDR  0x08
#define BNO055_MAG_CONFIG_ADDR  0x09
#define BNO055_GYR_CONFIG_0_ADDR 0x0A
#define BNO055_GYR_CONFIG_1_ADDR 0x0B

/* Data registers (Page 0) */
#define BNO055_ACCEL_DATA_X_LSB 0x08  // Accelerometer data X LSB (1 m/s² = 100 LSB)
#define BNO055_ACCEL_DATA_X_MSB 0x09  // Accelerometer data X MSB
#define BNO055_ACCEL_DATA_Y_LSB 0x0A  // Accelerometer data Y LSB
#define BNO055_ACCEL_DATA_Y_MSB 0x0B  // Accelerometer data Y MSB
#define BNO055_ACCEL_DATA_Z_LSB 0x0C  // Accelerometer data Z LSB
#define BNO055_ACCEL_DATA_Z_MSB 0x0D  // Accelerometer data Z MSB
#define BNO055_MAG_DATA_X_LSB   0x0E  // Magnetic field strength data (1 µT = 16 LSB)
#define BNO055_GYRO_DATA_X_LSB  0x14  // Angular velocity data (1 dps = 16 LSB)
#define BNO055_EULER_H_LSB      0x1A  // Euler angles (1 degree = 16 LSB)
#define BNO055_QUAT_DATA_W_LSB  0x20  // Quaternion data (1 quaternion = 2^14 LSB)
#define BNO055_LINEAR_ACCEL_X_LSB 0x28  // Linear acceleration (gravity removed, 1 m/s² = 100 LSB)
#define BNO055_GRAVITY_DATA_X_LSB 0x2E  // Gravity vector (1 m/s² = 100 LSB)

/* Calibration registers */
#define BNO055_CALIB_STAT_ADDR  0x35

/* BNO055 Chip ID */
#define BNO055_ID               0xA0

/* Power modes */
#define BNO055_POWER_MODE_NORMAL   0x00
#define BNO055_POWER_MODE_LOWPOWER 0x01
#define BNO055_POWER_MODE_SUSPEND  0x02

/* Operation modes (OPR_MODE register 0x3D) */
#define BNO055_OPERATION_MODE_CONFIG    0x00  // Configuration mode
#define BNO055_OPERATION_MODE_ACCONLY   0x01  // Accelerometer only
#define BNO055_OPERATION_MODE_MAGONLY   0x02  // Magnetometer only
#define BNO055_OPERATION_MODE_GYROONLY  0x03  // Gyroscope only
#define BNO055_OPERATION_MODE_ACCMAG    0x04  // Accelerometer + Magnetometer
#define BNO055_OPERATION_MODE_ACCGYRO   0x05  // Accelerometer + Gyroscope
#define BNO055_OPERATION_MODE_MAGGYRO   0x06  // Magnetometer + Gyroscope
#define BNO055_OPERATION_MODE_AMG       0x07  // Accelerometer + Magnetometer + Gyroscope (no fusion)
#define BNO055_OPERATION_MODE_IMU       0x08  // Inertial Measurement Unit (Accel + Gyro fusion, no Mag)
#define BNO055_OPERATION_MODE_COMPASS   0x09  // Compass (Accel + Mag fusion, no Gyro)
#define BNO055_OPERATION_MODE_M4G       0x0A  // M4G (Accel + Mag fusion for gaming)
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF 0x0B  // 9DOF with Fast Magnetometer Calibration OFF
#define BNO055_OPERATION_MODE_NDOF      0x0C  // 9DOF fusion (Accel + Mag + Gyro, best accuracy)

/* Data structures */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BNO055_Vector_t;

typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} BNO055_Quaternion_t;

typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} BNO055_CalibStatus_t;

/* Function prototypes */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode);
HAL_StatusTypeDef BNO055_GetChipID(I2C_HandleTypeDef *hi2c, uint8_t *id);

HAL_StatusTypeDef BNO055_ReadAccel(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *accel);
HAL_StatusTypeDef BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *gyro);
HAL_StatusTypeDef BNO055_ReadMag(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *mag);
HAL_StatusTypeDef BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *euler);
HAL_StatusTypeDef BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Quaternion_t *quat);
HAL_StatusTypeDef BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *linear);
HAL_StatusTypeDef BNO055_ReadGravity(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *gravity);

HAL_StatusTypeDef BNO055_GetCalibration(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *calib);

#ifdef __cplusplus
}
#endif

#endif /* __BNO055_H */

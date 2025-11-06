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
#define BNO055_PAGE_ID_ADDR     0x07
#define BNO055_OPR_MODE_ADDR    0x3D
#define BNO055_PWR_MODE_ADDR    0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_UNIT_SEL_ADDR    0x3B

/* Data registers */
#define BNO055_ACCEL_DATA_X_LSB 0x08
#define BNO055_GYRO_DATA_X_LSB  0x14
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_EULER_H_LSB      0x1A
#define BNO055_QUAT_DATA_W_LSB  0x20
#define BNO055_LINEAR_ACCEL_X_LSB 0x28
#define BNO055_GRAVITY_DATA_X_LSB 0x2E

/* Calibration registers */
#define BNO055_CALIB_STAT_ADDR  0x35

/* BNO055 Chip ID */
#define BNO055_ID               0xA0

/* Power modes */
#define BNO055_POWER_MODE_NORMAL   0x00
#define BNO055_POWER_MODE_LOWPOWER 0x01
#define BNO055_POWER_MODE_SUSPEND  0x02

/* Operation modes */
#define BNO055_OPERATION_MODE_CONFIG    0x00
#define BNO055_OPERATION_MODE_ACCONLY   0x01
#define BNO055_OPERATION_MODE_MAGONLY   0x02
#define BNO055_OPERATION_MODE_GYROONLY  0x03
#define BNO055_OPERATION_MODE_ACCMAG    0x04
#define BNO055_OPERATION_MODE_ACCGYRO   0x05
#define BNO055_OPERATION_MODE_MAGGYRO   0x06
#define BNO055_OPERATION_MODE_AMG       0x07
#define BNO055_OPERATION_MODE_IMU       0x08
#define BNO055_OPERATION_MODE_COMPASS   0x09
#define BNO055_OPERATION_MODE_M4G       0x0A
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define BNO055_OPERATION_MODE_NDOF      0x0C

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

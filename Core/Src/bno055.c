/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : BNO055 sensor driver implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bno055.h"

#define BNO055_TIMEOUT 1000

/**
  * @brief  Initialize BNO055 sensor
  * @param  hi2c: pointer to I2C handle
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;
    
    // Wait for sensor to boot
    HAL_Delay(650);
    
    // Check chip ID
    status = BNO055_GetChipID(hi2c, &chip_id);
    if (status != HAL_OK || chip_id != BNO055_ID) {
        return HAL_ERROR;
    }
    
    // Set to config mode
    status = BNO055_SetMode(hi2c, BNO055_OPERATION_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    // Reset
    uint8_t reset_cmd = 0x20;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(650);
    
    // Set power mode to normal
    uint8_t pwr_mode = BNO055_POWER_MODE_NORMAL;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_PWR_MODE_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &pwr_mode, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Set page ID to 0
    uint8_t page_id = 0;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_PAGE_ID_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &page_id, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    
    // Set units (default: m/s^2, deg/s, degrees, Celsius)
    uint8_t unit_sel = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_UNIT_SEL_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &unit_sel, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    
    // Set to NDOF mode (9DOF fusion)
    status = BNO055_SetMode(hi2c, BNO055_OPERATION_MODE_NDOF);
    if (status != HAL_OK) return status;
    HAL_Delay(20);
    
    return HAL_OK;
}

/**
  * @brief  Set operation mode
  * @param  hi2c: pointer to I2C handle
  * @param  mode: operation mode
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    return HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR, 
                              I2C_MEMADD_SIZE_8BIT, &mode, 1, BNO055_TIMEOUT);
}

/**
  * @brief  Get chip ID
  * @param  hi2c: pointer to I2C handle
  * @param  id: pointer to store chip ID
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_GetChipID(I2C_HandleTypeDef *hi2c, uint8_t *id)
{
    return HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_CHIP_ID_ADDR, 
                             I2C_MEMADD_SIZE_8BIT, id, 1, BNO055_TIMEOUT);
}

/**
  * @brief  Read accelerometer data
  * @param  hi2c: pointer to I2C handle
  * @param  accel: pointer to store accelerometer data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadAccel(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *accel)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        accel->x = (int16_t)((buffer[1] << 8) | buffer[0]);
        accel->y = (int16_t)((buffer[3] << 8) | buffer[2]);
        accel->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    return status;
}

/**
  * @brief  Read gyroscope data
  * @param  hi2c: pointer to I2C handle
  * @param  gyro: pointer to store gyroscope data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *gyro)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_GYRO_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        gyro->x = (int16_t)((buffer[1] << 8) | buffer[0]);
        gyro->y = (int16_t)((buffer[3] << 8) | buffer[2]);
        gyro->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    return status;
}

/**
  * @brief  Read magnetometer data
  * @param  hi2c: pointer to I2C handle
  * @param  mag: pointer to store magnetometer data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadMag(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *mag)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_MAG_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        mag->x = (int16_t)((buffer[1] << 8) | buffer[0]);
        mag->y = (int16_t)((buffer[3] << 8) | buffer[2]);
        mag->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    return status;
}

/**
  * @brief  Read Euler angles
  * @param  hi2c: pointer to I2C handle
  * @param  euler: pointer to store Euler angles (heading, roll, pitch)
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *euler)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_EULER_H_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        euler->x = (int16_t)((buffer[1] << 8) | buffer[0]); // Heading
        euler->y = (int16_t)((buffer[3] << 8) | buffer[2]); // Roll
        euler->z = (int16_t)((buffer[5] << 8) | buffer[4]); // Pitch
    }
    
    return status;
}

/**
  * @brief  Read quaternion data
  * @param  hi2c: pointer to I2C handle
  * @param  quat: pointer to store quaternion data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Quaternion_t *quat)
{
    uint8_t buffer[8];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_QUAT_DATA_W_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 8, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        quat->w = (int16_t)((buffer[1] << 8) | buffer[0]);
        quat->x = (int16_t)((buffer[3] << 8) | buffer[2]);
        quat->y = (int16_t)((buffer[5] << 8) | buffer[4]);
        quat->z = (int16_t)((buffer[7] << 8) | buffer[6]);
    }
    
    return status;
}

/**
  * @brief  Read linear acceleration data
  * @param  hi2c: pointer to I2C handle
  * @param  linear: pointer to store linear acceleration data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *linear)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_LINEAR_ACCEL_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        linear->x = (int16_t)((buffer[1] << 8) | buffer[0]);
        linear->y = (int16_t)((buffer[3] << 8) | buffer[2]);
        linear->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    return status;
}

/**
  * @brief  Read gravity vector
  * @param  hi2c: pointer to I2C handle
  * @param  gravity: pointer to store gravity vector data
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_ReadGravity(I2C_HandleTypeDef *hi2c, BNO055_Vector_t *gravity)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_GRAVITY_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        gravity->x = (int16_t)((buffer[1] << 8) | buffer[0]);
        gravity->y = (int16_t)((buffer[3] << 8) | buffer[2]);
        gravity->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    
    return status;
}

/**
  * @brief  Get calibration status
  * @param  hi2c: pointer to I2C handle
  * @param  calib: pointer to store calibration status
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_GetCalibration(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *calib)
{
    uint8_t calib_stat;
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_CALIB_STAT_ADDR, 
                               I2C_MEMADD_SIZE_8BIT, &calib_stat, 1, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        calib->sys = (calib_stat >> 6) & 0x03;
        calib->gyro = (calib_stat >> 4) & 0x03;
        calib->accel = (calib_stat >> 2) & 0x03;
        calib->mag = calib_stat & 0x03;
    }
    
    return status;
}

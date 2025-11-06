/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055.c
  * @brief          : BNO055 sensor driver implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bno055.h"

#define BNO055_TIMEOUT 100

/**
  * @brief  Initialize BNO055 sensor
  * @param  hi2c: pointer to I2C handle
  * @retval HAL status
  */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;
    
    // Wait for sensor boot (can take up to 850ms according to datasheet)
    int timeout = 850;
    while (timeout > 0) {
        status = BNO055_GetChipID(hi2c, &chip_id);
        if (status == HAL_OK && chip_id == BNO055_ID) {
            break;
        }
        HAL_Delay(10);
        timeout -= 10;
    }
    
    if (timeout <= 0 || chip_id != BNO055_ID) {
        return HAL_ERROR;  // Sensor not detected
    }
    
    // Switch to config mode
    status = BNO055_SetMode(hi2c, BNO055_OPERATION_MODE_CONFIG);
    if (status != HAL_OK) return status;
    HAL_Delay(25);
    
    // Reset sensor
    uint8_t reset_cmd = 0x20;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &reset_cmd, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    
    // Wait for reset to complete (minimum 30ms due to power issues)
    HAL_Delay(30);
    
    // Wait until chip ID is readable again after reset
    timeout = 1000;
    while (timeout > 0) {
        status = BNO055_GetChipID(hi2c, &chip_id);
        if (status == HAL_OK && chip_id == BNO055_ID) {
            break;
        }
        HAL_Delay(10);
        timeout -= 10;
    }
    HAL_Delay(50);  // Additional stabilization time
    
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
    
    // Clear system trigger
    uint8_t sys_trigger = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &sys_trigger, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Set units (m/s^2, deg/s, degrees, Celsius)
    uint8_t unit_sel = 0x00;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_UNIT_SEL_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &unit_sel, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    
    // Configure sensor data rates (Page 1)
    page_id = 1;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_PAGE_ID_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &page_id, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Accelerometer Config (0x08):
    // Bit 7-5: Not used
    // Bit 4-2: Bandwidth (011 = 62.5Hz)
    // Bit 1-0: Range (00 = ±2g, 01 = ±4g, 10 = ±8g, 11 = ±16g)
    uint8_t acc_config = 0x0D;  // 0b00001101: 62.5Hz BW, ±4g range
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_ACC_CONFIG_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &acc_config, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Magnetometer Config (0x09):
    // Bit 7-5: Not used
    // Bit 4-3: Output data rate (00 = 2Hz, 01 = 6Hz, 10 = 8Hz, 11 = 10Hz, 
    //                             100 = 15Hz, 101 = 20Hz, 110 = 25Hz, 111 = 30Hz)
    // Bit 2-0: Operation mode (000 = Low power, 001 = Regular, 010 = Enhanced, 011 = High accuracy)
    uint8_t mag_config = 0x1F;  // 0b00011111: 30Hz ODR, High accuracy mode
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_MAG_CONFIG_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &mag_config, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Gyroscope Config 0 (0x0A):
    // Bit 7-6: Not used
    // Bit 5-3: Bandwidth (000 = 523Hz, 001 = 230Hz, 010 = 116Hz, 011 = 47Hz, 
    //                     100 = 23Hz, 101 = 12Hz, 110 = 64Hz, 111 = 32Hz)
    // Bit 2-0: Range (000 = 2000dps, 001 = 1000dps, 010 = 500dps, 
    //                 011 = 250dps, 100 = 125dps)
    uint8_t gyr_config_0 = 0x10;  // 0b00010000: 116Hz BW, 2000dps range
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_GYR_CONFIG_0_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &gyr_config_0, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Gyroscope Config 1 (0x0B):
    // Bit 7-6: Not used
    // Bit 5-0: Not used (auto sleep settings in some modes)
    uint8_t gyr_config_1 = 0x00;  // Normal operation mode
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_GYR_CONFIG_1_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &gyr_config_1, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Back to page 0
    page_id = 0;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_PAGE_ID_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &page_id, 1, BNO055_TIMEOUT);
    if (status != HAL_OK) return status;
    HAL_Delay(10);
    
    // Set to NDOF mode (9DOF fusion with magnetometer)
    // In NDOF mode, all sensors are active and fusion algorithm provides stable output
    // Data update rate: 100Hz for fusion output
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
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &mode, 1, BNO055_TIMEOUT);
    if (status == HAL_OK) {
        HAL_Delay(30);  // Wait for mode switch to complete (as per C++ reference)
    }
    return status;
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
    static BNO055_Vector_t last_valid = {0, 0, 1000};  // Initialize Z to ~9.8m/s² (gravity)
    
    // Explicitly ensure we're on Page 0
    uint8_t page_id = 0;
    HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, BNO055_PAGE_ID_ADDR, 
                      I2C_MEMADD_SIZE_8BIT, &page_id, 1, BNO055_TIMEOUT);
    
    // Read accelerometer data
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
        int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
        
        // Update values (use last valid if current is 0xFFFF/-1)
        accel->x = (x != -1) ? x : last_valid.x;
        accel->y = (y != -1) ? y : last_valid.y;
        accel->z = (z != -1) ? z : last_valid.z;
        
        // Save valid values
        if (x != -1) last_valid.x = x;
        if (y != -1) last_valid.y = y;
        if (z != -1) last_valid.z = z;
    } else {
        // I2C error, use last valid values
        *accel = last_valid;
    }
    
    return HAL_OK;  // Always return OK to keep system running
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
    static BNO055_Vector_t last_valid = {0, 0, 0};
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_GYRO_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
        int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
        
        gyro->x = (x != -1) ? x : last_valid.x;
        gyro->y = (y != -1) ? y : last_valid.y;
        gyro->z = (z != -1) ? z : last_valid.z;
        
        if (x != -1) last_valid.x = x;
        if (y != -1) last_valid.y = y;
        if (z != -1) last_valid.z = z;
    } else {
        *gyro = last_valid;
    }
    
    return HAL_OK;
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
    static BNO055_Vector_t last_valid = {0, 0, 0};
    
    status = HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, BNO055_MAG_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, buffer, 6, BNO055_TIMEOUT);
    
    if (status == HAL_OK) {
        int16_t x = (int16_t)((buffer[1] << 8) | buffer[0]);
        int16_t y = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t z = (int16_t)((buffer[5] << 8) | buffer[4]);
        
        mag->x = (x != -1) ? x : last_valid.x;
        mag->y = (y != -1) ? y : last_valid.y;
        mag->z = (z != -1) ? z : last_valid.z;
        
        if (x != -1) last_valid.x = x;
        if (y != -1) last_valid.y = y;
        if (z != -1) last_valid.z = z;
    } else {
        *mag = last_valid;
    }
    
    return HAL_OK;
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

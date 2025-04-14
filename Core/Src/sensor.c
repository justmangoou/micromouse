/**
 * @file sensor.c
 * @brief MPU6050 sensor implementation.
 */
#include "stdint.h"
#include "sensor.h"

#include "main.h"

const float LSB_GYRO_SENS = 65.5F; // ± 500 °/s

float g_GyroOffset = 0.0F;

/* INTERNAL FUNCTION DECLARATIONS ------------------------------------*/
static void write_8bit(uint32_t reg, uint8_t *data);
static void read(uint32_t reg, uint8_t *data, uint8_t len);

//---------------------------------------------------------------------
// Motor Encoder
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// MPU6050 Sensor
//---------------------------------------------------------------------
bool MPU6050_Init(void) {
    HAL_Delay(100);
    uint8_t check;
    uint8_t data;

    read(MPU6050_REG_WHO_AM_I, &check, 1);

    if (check == 0x68) {
        // power management register 0X6B we should write all 0's to wake the sensor up
        data = 0;
        write_8bit(MPU6050_REG_PWR_MGMT_1, &data);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        data = 0x07; // 1KHz
        write_8bit(MPU6050_REG_SMPLRT_DIV, &data);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
        data = 0x00;
        write_8bit(MPU6050_REG_ACCEL_CONFIG, &data);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> ± 500 °/s
        data = 0x01;
        write_8bit(MPU6050_REG_GYRO_CONFIG, &data);

        return true;
    }

    // Calibrate
    int32_t sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += MPU6050_ReadGyroZ();
        HAL_Delay(5);
    }

    g_GyroOffset = (float)sum / 200.0F / LSB_GYRO_SENS;
    return false;
}

int16_t MPU6050_ReadGyroZ(void) {
    uint8_t buffer[2];
    read(MPU6050_REG_GYRO_ZOUT_H, buffer, 2);
    return (int16_t)(buffer[0] << 8 | buffer[1]);
}

float MPU6050_GetZAngle(void) {
    return (float)MPU6050_ReadGyroZ() / LSB_GYRO_SENS - g_GyroOffset;
}

/* INTERNAL FUNCTIONS ------------------------------------------------*/
static void write_8bit(const uint32_t reg, uint8_t *data) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEF_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT);
}

static void read(const uint32_t reg, uint8_t *data, const uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEF_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
}

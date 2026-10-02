#pragma once 

#include "stdint.h"
#include "ht_i2c.h"
#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <math.h>

#define MPU9250_I2C_ADDR_LOW      0x68
#define MPU9250_I2C_ADDR_HIGH     0x69
#define MPU9250_REG_PWR_MGMT_1    0x6B
#define MPU9250_REG_WHO_AM_I      0x75
#define MPU9250_WHO_AM_I_EXPECTED 0x71
#define MPU9250_REG_INT_PIN_CFG   0x37
#define MPU9250_REG_GYRO_CONFIG   0x1B
#define MPU9250_REG_ACCEL_CONFIG  0x1C
#define MPU9250_REG_ACCEL_XOUT_H  0x3B
#define MPU9250_REG_TEMP_OUT_H    0x41
#define MPU9250_REG_GYRO_XOUT_H   0x43
#define MPU9250_BANK_SEL          0x6D  
#define MPU9250_MEM_START_ADDR    0x6E  
#define MPU9250_MEM_R_W           0x6F  
#define MPU9250_PRGM_START_H      0x70  
#define MPU9250_PRGM_START_L      0x71
#define MPU9250_USER_CTRL         0x6A
#define MPU9250_FIFO_COUNTH       0x72
#define MPU9250_FIFO_COUNTL       0x73
#define MPU9250_FIFO_R_W          0x74

#define AK8963_I2C_ADDR           0x0C
#define AK8963_REG_WIA            0x00
#define AK8963_WIA_EXPECTED       0x48
#define AK8963_REG_CNTL1          0x0A
#define AK8963_REG_HXL            0x03

typedef enum {
    HT_MPU_FILTER_NONE = 0,
    HT_MPU_FILTER_MADGWICK,
    HT_MPU_FILTER_MAHONY,
    HT_MPU_FILTER_DMP
} ht_mpu9250_filter_type_t;

typedef enum {
    HT_MPU_GYRO_FS_250DPS = 0,
    HT_MPU_GYRO_FS_500DPS,
    HT_MPU_GYRO_FS_1000DPS,
    HT_MPU_GYRO_FS_2000DPS
} ht_mpu9250_gyro_fs_t;

typedef enum {
    HT_MPU_ACCEL_2G = 0,
    HT_MPU_ACCEL_4G,
    HT_MPU_ACCEL_8G,
    HT_MPU_ACCEL_16G
} ht_mpu9250_accel_fs_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ht_mpu9250_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} ht_mpu9250_data_t;

typedef struct {
    float q0, q1, q2, q3;
    float beta;
    uint64_t last_update_time;
} ht_mpu9250_ahrs_state_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} ht_mpu9250_euler_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_dev_handle_t i2c_mag;
    ht_mpu9250_filter_type_t filter_type;
    ht_mpu9250_gyro_fs_t gyro_scale;
    ht_mpu9250_accel_fs_t accel_scale;
    ht_mpu9250_ahrs_state_t ahrs;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
} ht_mpu9250_dev_t;

esp_err_t ht_mpu9250_init(ht_mpu9250_dev_t *dev);

esp_err_t ht_mpu9250_check_connection(ht_mpu9250_dev_t *dev, uint8_t *mpu_id, uint8_t *mag_id);

esp_err_t ht_mpu9250_get_accel_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *accel);

esp_err_t ht_mpu9250_get_gyro_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *gyro);

esp_err_t ht_mpu9250_get_mag_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *mag);

esp_err_t ht_mpu9250_get_temp_raw(ht_mpu9250_dev_t *dev, int16_t *temp);

esp_err_t ht_mpu9250_get_accel(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *accel);

esp_err_t ht_mpu9250_get_gyro(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *gyro);

esp_err_t ht_mpu9250_get_mag(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *mag);

esp_err_t ht_mpu9250_get_temp(ht_mpu9250_dev_t *dev, float *temp);

void ht_mpu9250_ahrs_init(ht_mpu9250_dev_t *dev);

esp_err_t ht_mpu9250_get_euler_angles(ht_mpu9250_dev_t *dev, ht_mpu9250_euler_t *euler);

esp_err_t ht_mpu9250_dmp_init(ht_mpu9250_dev_t *dev);

esp_err_t ht_mpu9250_calibrate_gyro(ht_mpu9250_dev_t *dev, uint16_t num_samples);
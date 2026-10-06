/**
 * @file ht_mpu9250.h
 * @brief MPU9250 9-DoF IMU Driver with DMP and AHRS support
 * 
 * Fully featured driver for the InvenSense MPU9250. Supports hardware DMP parsing,
 * Mahony, and Madgwick filter engines for real-time spatial orientation calculation.
 */

#pragma once 

#include "stdint.h"
#include "ht_i2c.h"
#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <math.h>

/* MPU9250 and AK8963 Register Definitions */
#define MPU9250_I2C_ADDR_LOW      0x68
#define MPU9250_I2C_ADDR_HIGH     0x69
#define MPU9250_REG_PWR_MGMT_1    0x6B
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
#define MPU9250_WHO_AM_I_71       0x71
#define MPU9250_WHO_AM_I_70       0x70
#define MPU9250_WHO_AM_I_73       0x73
#define MPU9250_REG_USER_CTRL     0x6A
#define MPU9250_REG_INT_PIN_CFG   0x37
#define MPU9250_REG_WHO_AM_I      0x75

#define AK8963_I2C_ADDR           0x0C
#define AK8963_REG_WIA            0x00
#define AK8963_WIA_EXPECTED       0x48
#define AK8963_REG_CNTL1          0x0A
#define AK8963_REG_HXL            0x03
#define AK8963_WHO_AM_I_48        0x48

/**
 * @brief Available Sensor Fusion / Processing Engines
 */
typedef enum {
    HT_MPU_FILTER_NONE = 0,    /**< Raw data only, no fusion calculation */
    HT_MPU_FILTER_MADGWICK,    /**< Madgwick algorithm (smooth tracking) */
    HT_MPU_FILTER_MAHONY,      /**< Mahony algorithm (aggressive and fast response) */
    HT_MPU_FILTER_DMP          /**< Hardware Digital Motion Processor (reads 28-byte packets) */
} ht_mpu9250_filter_type_t;

/**
 * @brief Gyroscope Full-Scale Range Configuration
 */
typedef enum {
    HT_MPU_GYRO_FS_250DPS = 0,
    HT_MPU_GYRO_FS_500DPS,
    HT_MPU_GYRO_FS_1000DPS,
    HT_MPU_GYRO_FS_2000DPS     /**< Recommended for robotics */
} ht_mpu9250_gyro_fs_t;

/**
 * @brief Accelerometer Full-Scale Range Configuration
 */
typedef enum {
    HT_MPU_ACCEL_2G = 0,       /**< High precision, clips on high impact */
    HT_MPU_ACCEL_4G,
    HT_MPU_ACCEL_8G,
    HT_MPU_ACCEL_16G
} ht_mpu9250_accel_fs_t;

/**
 * @brief Structure to hold raw, unscaled integer data from registers.
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ht_mpu9250_raw_data_t;

/**
 * @brief Structure to hold scaled float data (g for accel, DPS for gyro).
 */
typedef struct {
    float x;
    float y;
    float z;
} ht_mpu9250_data_t;

/**
 * @brief State variables for Software AHRS engines (Quaternions and Timings).
 */
typedef struct {
    float q0, q1, q2, q3;
    float beta;
    uint64_t last_update_time;
} ht_mpu9250_ahrs_state_t;

/**
 * @brief Computed spatial orientation of the sensor.
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} ht_mpu9250_euler_t;

/**
 * @brief Core MPU9250 device configuration and state structure.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;      /**< Handle for the MPU9250 I2C connection */
    i2c_master_dev_handle_t i2c_mag;      /**< Handle for the AK8963 Magnetometer I2C connection */
    ht_mpu9250_filter_type_t filter_type; /**< Active processing engine */
    ht_mpu9250_gyro_fs_t gyro_scale;      /**< Gyroscope sensitivity */
    ht_mpu9250_accel_fs_t accel_scale;    /**< Accelerometer sensitivity */
    ht_mpu9250_ahrs_state_t ahrs;         /**< Internal state for software filters */
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float mag_bias_x;
    float mag_bias_y;
    float mag_bias_z;
} ht_mpu9250_dev_t;

/**
 * @brief Boots up the MPU9250 and AK8963, configures scales, and loads DMP firmware if selected.
 * 
 * @param dev Pointer to the device configuration structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_mpu9250_init(ht_mpu9250_dev_t *dev);

/**
 * @brief Verifies communication with the MPU9250 and internal AK8963.
 * 
 * @param dev Pointer to the device configuration structure.
 * @param mpu_id Output pointer to store the MPU WHO_AM_I value.
 * @param mag_id Output pointer to store the MAG WIA value.
 * @return esp_err_t ESP_OK if connected.
 */
esp_err_t ht_mpu9250_check_connection(ht_mpu9250_dev_t *dev, uint8_t *mpu_id, uint8_t *mag_id);

/**
 * @brief Reads raw integer accelerometer data directly from registers.
 */
esp_err_t ht_mpu9250_get_accel_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *accel);

/**
 * @brief Reads raw integer gyroscope data directly from registers.
 */
esp_err_t ht_mpu9250_get_gyro_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *gyro);

/**
 * @brief Reads raw integer magnetometer data directly from AK8963 registers.
 */
esp_err_t ht_mpu9250_get_mag_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *mag);

/**
 * @brief Reads raw integer temperature data.
 */
esp_err_t ht_mpu9250_get_temp_raw(ht_mpu9250_dev_t *dev, int16_t *temp);

/**
 * @brief Reads and scales accelerometer data to gravity (g).
 */
esp_err_t ht_mpu9250_get_accel(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *accel);

/**
 * @brief Reads, applies bias, and scales gyroscope data to Degrees Per Second (DPS).
 */
esp_err_t ht_mpu9250_get_gyro(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *gyro);

/**
 * @brief Reads, applies bias, and scales magnetometer data to micro-Tesla (uT).
 */
esp_err_t ht_mpu9250_get_mag(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *mag);

/**
 * @brief Reads and converts the internal temperature sensor data to Celsius.
 */
esp_err_t ht_mpu9250_get_temp(ht_mpu9250_dev_t *dev, float *temp);

/**
 * @brief Retrieves the latest computed Euler angles (Roll, Pitch, Yaw).
 * 
 * @note If HT_MPU_FILTER_DMP is selected, this pulls 28-byte packets from the FIFO.
 *       If FIFO is desynchronized, it resets it and returns ESP_ERR_INVALID_STATE.
 *       If a software filter is selected, it updates the state based on dt (delta time).
 * 
 * @param dev Pointer to the device configuration structure.
 * @param euler Pointer to store the calculated Roll, Pitch, and Yaw angles.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if DMP FIFO recovers from corruption.
 */
esp_err_t ht_mpu9250_get_euler_angles(ht_mpu9250_dev_t *dev, ht_mpu9250_euler_t *euler);

/**
 * @brief Calibrates the gyroscope by calculating the static bias.
 * 
 * @warning The sensor must remain completely motionless during this routine.
 * @param dev Pointer to the device configuration structure.
 * @param num_samples Number of readings to average.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_mpu9250_calibrate_gyro(ht_mpu9250_dev_t *dev, uint16_t num_samples);

/**
 * @brief Calibrates the magnetometer to find hard-iron offsets.
 * 
 * @warning The sensor must be rotated in a 3D Figure-8 motion continuously while calibrating.
 * @param dev Pointer to the device configuration structure.
 * @param num_samples Number of readings to record for min/max boundary search.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ht_mpu9250_calibrate_mag(ht_mpu9250_dev_t *dev, uint16_t num_samples);
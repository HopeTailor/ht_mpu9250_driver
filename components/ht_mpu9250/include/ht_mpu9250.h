#pragma once 

#include "stdint.h"
#include "ht_i2c.h"
#include "esp_err.h"

#define MPU9250_I2C_ADDR_LOW      0x68
#define MPU9250_I2C_ADDR_HIGH     0x69
#define MPU9250_REG_PWR_MGMT_1    0x6B
#define MPU9250_REG_WHO_AM_I      0x75
#define MPU9250_WHO_AM_I_EXPECTED 0x71
#define MPU9250_REG_INT_PIN_CFG   0x37

#define AK8963_I2C_ADDR           0x0C
#define AK8963_REG_WIA            0x00
#define AK8963_WIA_EXPECTED       0x48

typedef enum {
    HT_MPU_FILTER_NONE = 0,
    HT_MPU_FILTER_KALMAN,
    HT_MPU_FILTER_MADGWICK,
    HT_MPU_FILTER_MAHONY,
    HT_MPU_FILTER_HW_DMP
} ht_mpu_filter_type_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_dev_handle_t i2c_mag;
    ht_mpu_filter_type_t filter_type;
} ht_mpu9250_dev_t;

esp_err_t ht_mpu9250_init(ht_mpu9250_dev_t *dev);

esp_err_t ht_mpu9250_check_connection(ht_mpu9250_dev_t *dev, uint8_t *mpu_id, uint8_t *mag_id);

#include "ht_mpu9250.h"


static esp_err_t ht_mpu9250_set_gyro_fs(ht_mpu9250_dev_t *dev) {
    uint8_t config_val = dev->gyro_scale << 3;
    return ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_GYRO_CONFIG, &config_val, 1);
}

static esp_err_t ht_mpu9250_set_accel_fs(ht_mpu9250_dev_t *dev) {
    uint8_t config_val = dev->accel_scale << 3;
    return ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_ACCEL_CONFIG, &config_val, 1);
}

static float ht_mpu9250_get_accel_res(ht_mpu_accel_fs_t scale) {
    switch(scale) {
        case HT_MPU_ACCEL_2G: return 16384.0f;
        case HT_MPU_ACCEL_4G: return 8192.0f;
        case HT_MPU_ACCEL_8G: return 4096.0f;
        case HT_MPU_ACCEL_16G: return 2048.0f;
        default: return 16384.0f;
    }
}

static float ht_mpu9250_get_gyro_res(ht_mpu_gyro_fs_t scale) {
    switch(scale) {
        case HT_MPU_GYRO_FS_250DPS:  return 131.0f;
        case HT_MPU_GYRO_FS_500DPS:  return 65.5f;
        case HT_MPU_GYRO_FS_1000DPS: return 32.8f;
        case HT_MPU_GYRO_FS_2000DPS: return 16.4f;
        default: return 131.0f;
    }
}

esp_err_t ht_mpu9250_init(ht_mpu9250_dev_t *dev) {
    uint8_t pwr_val = 0x00;
    esp_err_t err = ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_PWR_MGMT_1, &pwr_val, 1);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t bypass_val = 0x02;
    err = ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_INT_PIN_CFG, &bypass_val, 1);
    if(err != ESP_OK) {
        return err;
    }

    err = ht_mpu9250_set_gyro_fs(dev);
    if(err != ESP_OK) {
        return err;
    }

    err = ht_mpu9250_set_accel_fs(dev);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t mag_config = 0x16;
    err = ht_i2c_write_reg8(dev->i2c_mag, AK8963_REG_CNTL1, &mag_config, 1);
    return err;
}

esp_err_t ht_mpu9250_check_connection(ht_mpu9250_dev_t *dev, uint8_t *mpu_id, uint8_t *mag_id) {
    esp_err_t err = ht_i2c_read_reg8(dev->i2c_dev, MPU9250_REG_WHO_AM_I, mpu_id, 1);
    if(err != ESP_OK) {
        return err;
    }

    if(*mpu_id != MPU9250_WHO_AM_I_EXPECTED) {
        return ESP_ERR_NOT_FOUND;
    }

    err = ht_i2c_read_reg8(dev->i2c_mag, AK8963_REG_WIA, mag_id, 1);
    if(err != ESP_OK) {
        return err;
    }

    if(*mag_id != AK8963_WIA_EXPECTED) {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t ht_mpu9250_get_accel_raw(ht_mpu9250_dev_t *dev, ht_mpu_raw_data_t *accel) {
    uint8_t data[6];
    esp_err_t err = ht_i2c_read_reg8(dev->i2c_dev, MPU9250_REG_ACCEL_XOUT_H, data, 6);
    if(err != ESP_OK) {
        return err;
    }

    accel->x = (int16_t)((data[0] << 8) | data[1]);
    accel->y = (int16_t)((data[3] << 8) | data[2]);
    accel->z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_gyro_raw(ht_mpu9250_dev_t *dev, ht_mpu_raw_data_t *gyro) {
    uint8_t data[6];
    esp_err_t err = ht_i2c_read_reg8(dev->i2c_dev, MPU9250_REG_GYRO_XOUT_H, data, 6);
    if(err != ESP_OK) {
        return err;
    } 

    gyro->x = (int16_t)((data[0] << 8) | data[1]);
    gyro->y = (int16_t)((data[2] << 8) | data[3]);
    gyro->z = (int16_t)((data[4] << 8) | data[5]);
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_mag_raw(ht_mpu9250_dev_t *dev, ht_mpu_raw_data_t *mag){
    uint8_t data[7];
    esp_err_t err = ht_i2c_read_reg8(dev->i2c_mag, AK8963_REG_HXL, data, 7);
    if(err != ESP_OK) {
        return err;
    } 

    mag->x = (int16_t)((data[1] << 8) | data[0]);
    mag->y = (int16_t)((data[3] << 8) | data[2]);
    mag->z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_temp_raw(ht_mpu9250_dev_t *dev, int16_t *temp) {
    uint8_t data[2];
    esp_err_t err = ht_i2c_read_reg8(dev->i2c_dev, MPU9250_REG_TEMP_OUT_H, data, 2);
    if(err != ESP_OK) {
        return err;
    }

    *temp = (int16_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_accel(ht_mpu9250_dev_t *dev, ht_mpu_data_t *accel) {
    ht_mpu_raw_data_t raw;
    esp_err_t err = ht_mpu9250_get_accel_raw(dev, &raw);
    if(err != ESP_OK) {
        return err;
    }

    float res = ht_mpu9250_get_accel_res(dev->accel_scale);
    accel->x = (float)raw.x / res;
    accel->y = (float)raw.y / res;
    accel->z = (float)raw.z / res;
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_gyro(ht_mpu9250_dev_t *dev, ht_mpu_data_t *gyro) {
    ht_mpu_raw_data_t raw;
    esp_err_t err = ht_mpu9250_get_gyro_raw(dev, &raw);
    if(err != ESP_OK) {
        return err;
    }

    float res = ht_mpu9250_get_gyro_res(dev->gyro_scale);
    gyro->x = (float)raw.x / res;
    gyro->y = (float)raw.y / res;
    gyro->z = (float)raw.z / res;
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_mag(ht_mpu9250_dev_t *dev, ht_mpu_data_t *mag) {
    ht_mpu_raw_data_t raw;
    esp_err_t err = ht_mpu9250_get_mag_raw(dev, &raw);
    if(err != ESP_OK) {
        return err;
    }

    mag->x = (float)raw.x * 0.15f;
    mag->y = (float)raw.y * 0.15f;
    mag->z = (float)(-raw.z) * 0.15f;
    return ESP_OK;
}

esp_err_t ht_mpu9250_get_temp(ht_mpu9250_dev_t *dev, float *temp) {
    int16_t raw;
    esp_err_t err = ht_mpu9250_get_temp_raw(dev, &raw);
    if(err != ESP_OK) {
        return err;
    }

    *temp = ((float)raw) / 333.87f + 21.0f;
    return ESP_OK; 
}











#include "ht_mpu9250.h"

esp_err_t ht_mpu9250_init(ht_mpu9250_dev_t *dev) {
    uint8_t pwr_val = 0x00;
    esp_err_t err = ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_PWR_MGMT_1, &pwr_val, 1);
    if(err != ESP_OK) {
        return err;
    }

    uint8_t bypass_val = 0x02;
    err = ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_INT_PIN_CFG, &bypass_val, 1);
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

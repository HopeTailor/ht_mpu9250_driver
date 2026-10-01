#include "ht_mpu9250.h"

static esp_err_t ht_mpu9250_set_gyro_fs(ht_mpu9250_dev_t *dev) {
    uint8_t config_val = dev->gyro_scale << 3;
    return ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_GYRO_CONFIG, &config_val, 1);
}

static esp_err_t ht_mpu9250_set_accel_fs(ht_mpu9250_dev_t *dev) {
    uint8_t config_val = dev->accel_scale << 3;
    return ht_i2c_write_reg8(dev->i2c_dev, MPU9250_REG_ACCEL_CONFIG, &config_val, 1);
}

static float ht_mpu9250_get_accel_res(ht_mpu9250_accel_fs_t scale) {
    switch(scale) {
        case HT_MPU_ACCEL_2G: return 16384.0f;
        case HT_MPU_ACCEL_4G: return 8192.0f;
        case HT_MPU_ACCEL_8G: return 4096.0f;
        case HT_MPU_ACCEL_16G: return 2048.0f;
        default: return 16384.0f;
    }
}

static float ht_mpu9250_get_gyro_res(ht_mpu9250_gyro_fs_t scale) {
    switch(scale) {
        case HT_MPU_GYRO_FS_250DPS:  return 131.0f;
        case HT_MPU_GYRO_FS_500DPS:  return 65.5f;
        case HT_MPU_GYRO_FS_1000DPS: return 32.8f;
        case HT_MPU_GYRO_FS_2000DPS: return 16.4f;
        default: return 131.0f;
    }
}

void ht_mpu9250_ahrs_init(ht_mpu9250_dev_t *dev) {
    dev->ahrs.q0 = 1.0f;
    dev->ahrs.q1 = 0.0f;
    dev->ahrs.q2 = 0.0f;
    dev->ahrs.q3 = 0.0f;

    dev->ahrs.beta = 0.1f;

    dev->ahrs.last_update_time = esp_timer_get_time();
}

static float ht_inv_sqrt(float x) {
    return 1.0f / sqrtf(x); 
}

static void ht_mpu_madgwick_update(ht_mpu9250_dev_t *dev, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
    float q0 = dev->ahrs.q0, q1 = dev->ahrs.q1, q2 = dev->ahrs.q2, q3 = dev->ahrs.q3;
    float beta = dev->ahrs.beta;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return;
    }

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
        recipNorm = ht_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = ht_inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx; _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;

        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = ht_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = ht_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    dev->ahrs.q0 = q0 * recipNorm;
    dev->ahrs.q1 = q1 * recipNorm;
    dev->ahrs.q2 = q2 * recipNorm;
    dev->ahrs.q3 = q3 * recipNorm;
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

esp_err_t ht_mpu9250_get_accel_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *accel) {
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

esp_err_t ht_mpu9250_get_gyro_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *gyro) {
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

esp_err_t ht_mpu9250_get_mag_raw(ht_mpu9250_dev_t *dev, ht_mpu9250_raw_data_t *mag){
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

esp_err_t ht_mpu9250_get_accel(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *accel) {
    ht_mpu9250_raw_data_t raw;
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

esp_err_t ht_mpu9250_get_gyro(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *gyro) {
    ht_mpu9250_raw_data_t raw;
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

esp_err_t ht_mpu9250_get_mag(ht_mpu9250_dev_t *dev, ht_mpu9250_data_t *mag) {
    ht_mpu9250_raw_data_t raw;
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

esp_err_t ht_mpu9250_get_euler_angles(ht_mpu9250_dev_t *dev, ht_mpu9250_euler_t *euler) {
    esp_err_t err;

    if(dev->filter_type == HT_MPU_FILTER_DMP) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    ht_mpu9250_data_t accel, gyro, mag;

    if((err = ht_mpu9250_get_accel(dev, &accel)) != ESP_OK) {
        return err;
    }
    if((err = ht_mpu9250_get_gyro(dev, &accel)) != ESP_OK) {
        return err;
    }
    if((err = ht_mpu9250_get_mag(dev, &accel)) != ESP_OK) {
        return err;
    }

    uint64_t now = esp_timer_get_time();

    float dt = (now - dev->ahrs.last_update_time) / 1000000.0f;
    dev->ahrs.last_update_time = now;

    float gx_rad = gyro.x * (M_PI / 180.0f);
    float gy_rad = gyro.y * (M_PI / 180.0f);
    float gz_rad = gyro.z * (M_PI / 180.0f);

    switch(dev->filter_type) {
        case HT_MPU_FILTER_MADGWICK:
            ht_mpu9250_madgwick_update(dev, accel.x, accel.y, accel.z, gx_rad, gy_rad, gz_rad, mag.x, mag.y, mag.z, dt);
            break;
        case HT_MPU_FILTER_MAHONY:
            ht_mpu9250_mahony_update(dev, accel.x, accel.y, accel.z, gx_rad, gy_rad, gz_rad, mag.x, mag.y, mag.z, dt);
            break;    
        default:
            return ESP_ERR_INVALID_ARG;
    }

    float q0 = dev->ahrs.q0;
    float q1 = dev->ahrs.q1;
    float q2 = dev->ahrs.q2;
    float q3 = dev->ahrs.q3;

    euler->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = fmaxf(fminf(sinp, 1.0f), -1.0f);
    euler->pitch = asinf(sinp) * (180.0f / M_PI);

    euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);

    return ESP_OK;
}










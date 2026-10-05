#include "ht_mpu9250.h"
#include "esp_log.h"

#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "MPU9250";

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C Master...");
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = ht_i2c_bus_init(I2C_MASTER_SDA, I2C_MASTER_SCL, &bus_handle);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C Initialization Failed!");
        return;
    }
    ESP_LOGI(TAG, "I2C Bus Initialized.");

    ht_i2c_scan(bus_handle);

    ht_mpu9250_dev_t mpu_dev = {
        .filter_type = HT_MPU_FILTER_MADGWICK, 
        .gyro_scale = HT_MPU_GYRO_FS_2000DPS,
        .accel_scale = HT_MPU_ACCEL_2G
    };

    ESP_ERROR_CHECK(ht_i2c_add_device(bus_handle, MPU9250_I2C_ADDR_LOW, I2C_MASTER_FREQ_HZ, &mpu_dev.i2c_dev));
    ESP_ERROR_CHECK(ht_i2c_add_device(bus_handle, AK8963_I2C_ADDR, I2C_MASTER_FREQ_HZ, &mpu_dev.i2c_mag));

    uint8_t mpu_id, mag_id;
    if (ht_mpu9250_check_connection(&mpu_dev, &mpu_id, &mag_id) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor not found! Check your wiring.");
        return; 
    }
    ESP_LOGI(TAG, "Sensor Connected! MPU: 0x%02x, MAG: 0x%02x", mpu_id, mag_id);

    ESP_LOGI(TAG, "Initializing Sensor & Processing Engine (Please wait)...");
    ESP_ERROR_CHECK(ht_mpu9250_init(&mpu_dev));
    ESP_LOGI(TAG, "Initialization Complete!");

    ESP_LOGW(TAG, "Keep the sensor absolutely STILL for calibration!");
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    ESP_LOGW(TAG, "Calibrating Gyroscope...");
    ESP_ERROR_CHECK(ht_mpu9250_calibrate_gyro(&mpu_dev, 500));
    ESP_LOGI(TAG, "Calibration Done! Bias X:%.2f Y:%.2f Z:%.2f", mpu_dev.gyro_bias_x, mpu_dev.gyro_bias_y, mpu_dev.gyro_bias_z);

    // ESP_LOGW(TAG, "MAG CALIBRATION: wave the sensor in a FIGURE-8 shape for 15 seconds...");
    // ESP_ERROR_CHECK(ht_mpu9250_calibrate_mag(&mpu_dev, 1500));
    // ESP_LOGI(TAG, "Mag Done! Bias X:%.2f Y:%.2f Z:%.2f", mpu_dev.mag_bias_x, mpu_dev.mag_bias_y, mpu_dev.mag_bias_z);

    // mpu_dev.ahrs.last_update_time = esp_timer_get_time();

    ht_mpu9250_euler_t euler;
    
    while(1) {
        err = ht_mpu9250_get_euler_angles(&mpu_dev, &euler);
        
        if(err == ESP_OK) {
            printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\r", euler.roll, euler.pitch, euler.yaw);
        } 
        else if (err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to read angles: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
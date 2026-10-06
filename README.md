# HT MPU9250 9-DoF IMU Driver for ESP32 (ESP-IDF)

A robust, highly optimized, non-blocking C driver for the MPU9250 (and AK8963 magnetometer) designed specifically for the ESP-IDF framework. It features Real-Time Hardware DMP (Digital Motion Processor) parsing and finely-tuned software AHRS algorithms (Mahony & Madgwick) built for high-speed mobile robotics.

---

## 🚀 Getting Started (Setup)

### 1. Installation
Clone this repository into the `components/` directory of your ESP-IDF project:
```bash
cd components/
git clone [https://github.com/HopeTailor/ht_mpu9250_driver.git](https://github.com/HopeTailor/ht_mpu9250_driver.git)
```
*Note: This driver depends on a custom I2C abstraction layer (`ht_i2c`). Ensure your project has the required I2C read/write functions implemented.*

### 2. Basic Initialization
Here is a minimal setup to get the sensor running and outputting Euler angles:

```c
#include "ht_mpu9250.h"

// 1. Define and configure the device structure
ht_mpu9250_dev_t mpu_dev = {
    .filter_type = HT_MPU_FILTER_DMP,       // Choose the engine (DMP, MAHONY, MADGWICK)
    .gyro_scale  = HT_MPU_GYRO_FS_2000DPS,  // Gyroscope sensitivity
    .accel_scale = HT_MPU_ACCEL_2G          // Accelerometer sensitivity
};

void app_main(void) {
    // 2. Initialize I2C and link devices (Implementation depends on your ht_i2c driver)
    ht_i2c_add_device(bus_handle, MPU9250_I2C_ADDR_LOW, 400000, &mpu_dev.i2c_dev);
    ht_i2c_add_device(bus_handle, AK8963_I2C_ADDR, 400000, &mpu_dev.i2c_mag);

    // 3. Boot up the sensor and selected engine
    ESP_ERROR_CHECK(ht_mpu9250_init(&mpu_dev));

    // 4. Read data inside your RTOS task
    ht_mpu9250_euler_t euler;
    while(1) {
        if (ht_mpu9250_get_euler_angles(&mpu_dev, &euler) == ESP_OK) {
            printf("Roll: %.2f | Pitch: %.2f | Yaw: %.2f\n", euler.roll, euler.pitch, euler.yaw);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
```

---

## ⚙️ Configuration & Data Behavior

How you configure the `ht_mpu9250_dev_t` struct deeply affects the output data. Here is a guide to choosing the right settings for your project:

### 1. The Processing Engine (`filter_type`)
*   **`HT_MPU_FILTER_DMP` (Hardware Engine):** 
    *   **Behavior:** Offloads all complex quaternion calculations to the MPU9250's internal processor. It reads 28-byte packets directly from the hardware FIFO.
    *   **Best for:** Projects requiring low CPU usage on the ESP32 and highly stable, noise-free Roll/Pitch data.
    *   **Note:** Manual calibration functions (like mag figure-8) should generally be avoided in this mode, as the DMP has its own internal baseline tracking.
*   **`HT_MPU_FILTER_MAHONY` (Software Engine):**
    *   **Behavior:** Uses a Proportional (P) controller. In our source code, it is aggressively tuned (`Kp = 10.0`) to instantly trust the accelerometer.
    *   **Best for:** Fast-moving autonomous robots or vehicles where rapid, zero-lag response to physical turns is crucial.
*   **`HT_MPU_FILTER_MADGWICK` (Software Engine):**
    *   **Behavior:** Uses gradient descent optimization. Configured with a `beta` of `0.8`, it provides a smooth and continuous quaternion output.
    *   **Best for:** VR headsets, wearable devices, or platforms needing buttery-smooth tracking.

### 2. Sensor Scales (`accel_scale` & `gyro_scale`)
*   **Accelerometer (2G to 16G):** 
    *   Setting it to `2G` gives you the highest precision (sensitive to tiny tilts) but will clip if the robot experiences heavy impacts or crashes. `8G` or `16G` is better for high-impact environments (e.g., drones).
*   **Gyroscope (250DPS to 2000DPS):**
    *   `2000DPS` (Degrees Per Second) allows the sensor to track extremely fast spins without losing track of its position. `250DPS` provides finer resolution for very slow, delicate movements.

---

## 👏 Acknowledgments & Credits

This driver was built from scratch for the ESP-IDF framework, but its logic, register maps, and algorithms were heavily inspired by the pioneering work of the open-source community:

*   **[Kris Winer's MPU9250 Repository](https://github.com/kriswiner/MPU9250):** For the exhaustive documentation, register mapping, and base initialization sequences of the MPU9250 and AK8963.
*   **[Jeff Rowberg's I2Cdevlib](https://github.com/jrowberg/i2cdevlib):** For the incredible reverse-engineering of the InvenSense DMP firmware and FIFO packet structures.
*   **Sebastian Madgwick & Robert Mahony:** For their brilliant, lightweight AHRS mathematical models that make 9-DoF sensor fusion possible on microcontrollers.

# HT MPU9250 9-DoF IMU Driver for ESP32 (ESP-IDF)

![ESP-IDF](https://img.shields.io/badge/Platform-ESP--IDF-red)
![C](https://img.shields.io/badge/Language-C-blue)
![License](https://img.shields.io/badge/License-MIT-green)

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

### 2. Hardware Wiring
Connect your MPU9250 module to the ESP32 via the I2C bus. Note that the MPU9250 is a 3.3V logic device.

| MPU9250 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Do NOT connect to 5V unless your breakout board has a regulator. |
| **GND** | GND | Common ground. |
| **SDA** | GPIO 21 | Requires a 4.7kΩ or 10kΩ pull-up resistor. |
| **SCL** | GPIO 22 | Requires a 4.7kΩ or 10kΩ pull-up resistor. |

### 3. Basic Initialization
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
    // 2. Initialize I2C and link devices
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
    *   **Note:** Manual calibration functions should generally be avoided in this mode.
*   **`HT_MPU_FILTER_MAHONY` (Software Engine):**
    *   **Behavior:** Uses a Proportional (P) controller. Tuned aggressively (`Kp = 10.0`) to instantly trust the accelerometer.
    *   **Best for:** Fast-moving autonomous robots or vehicles where rapid, zero-lag response to physical turns is crucial.
*   **`HT_MPU_FILTER_MADGWICK` (Software Engine):**
    *   **Behavior:** Uses gradient descent optimization. Configured with a `beta` of `0.8`, providing a smooth and continuous quaternion output.
    *   **Best for:** VR headsets, wearable devices, or platforms needing buttery-smooth tracking.

### 2. Gyroscope Sensitivity (`gyro_scale`)
*   **`HT_MPU_GYRO_FS_250DPS` / `500DPS` (Low Range, High Precision):**
    *   **Behavior:** Maps the sensor's 16-bit ADC to a maximum rotation of 250 or 500 degrees per second. Provides the highest resolution for tiny movements, but "clips" (maxes out) if the sensor spins too fast.
    *   **Best for:** Camera gimbals, robotic arms, and slow-moving platforms where micro-precision is required.
*   **`HT_MPU_GYRO_FS_1000DPS` / `2000DPS` (High Range, Low Precision):**
    *   **Behavior:** Compresses the resolution to track extremely aggressive and violent spins without losing spatial awareness.
    *   **Best for:** Racing rovers, drones, and high-speed autonomous robots that make sudden, sharp turns.

### 3. Accelerometer Sensitivity (`accel_scale`)
*   **`HT_MPU_ACCEL_2G` / `4G` (High Sensitivity):**
    *   **Behavior:** Maximizes the ADC resolution for Earth's 1G gravity vector. It detects the slightest tilts perfectly but will clip under heavy vibrations or impacts.
    *   **Best for:** Self-balancing robots, tilt-sensors, and slow wheeled robots where calculating the exact Roll/Pitch from gravity is the main goal.
*   **`HT_MPU_ACCEL_8G` / `16G` (Impact & Vibration Resistance):**
    *   **Behavior:** Reduces tilt sensitivity to measure heavy linear accelerations and shocks without saturating the sensor.
    *   **Best for:** Systems with high mechanical vibration (like direct-drive chassis), quadcopters, or crash-detection modules.

---

## 🛠️ Troubleshooting & FAQ

*   **`ESP_ERR_INVALID_STATE` when reading Euler angles in DMP mode:**
    *   *Don't panic!* This is a built-in safety feature. It means the FIFO buffer became desynchronized or overflowed. The driver automatically flushed the corrupted buffer, and clean data will be available in the next loop iteration.
*   **Sensor outputs all zeros or fails to initialize:**
    *   Ensure your I2C pull-up resistors are installed. Check your wiring and verify the I2C addresses using a scanner.
*   **Yaw angle drifts constantly:**
    *   If using software filters (Mahony/Madgwick), ensure you have properly executed the 3D Figure-8 magnetometer calibration (`ht_mpu9250_calibrate_mag`) before the main loop.

---

## 👏 Acknowledgments & Credits

This driver was built from scratch for the ESP-IDF framework, but its logic, register maps, and algorithms were heavily inspired by the pioneering work of the open-source community:

*   **[Kris Winer's MPU9250 Repository](https://github.com/kriswiner/MPU9250):** For the exhaustive documentation, register mapping, and base initialization sequences of the MPU9250 and AK8963.
*   **[Jeff Rowberg's I2Cdevlib](https://github.com/jrowberg/i2cdevlib):** For the incredible reverse-engineering of the InvenSense DMP firmware and FIFO packet structures.
*   **Sebastian Madgwick & Robert Mahony:** For their brilliant, lightweight AHRS mathematical models that make 9-DoF sensor fusion possible on microcontrollers.
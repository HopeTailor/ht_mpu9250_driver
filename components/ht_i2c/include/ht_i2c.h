#pragma once

#include "stdint.h"
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t ht_i2c_bus_init(int sda_pin, int scl_pin, i2c_master_bus_handle_t *bus_handle);

esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed, i2c_master_dev_handle_t *dev_handle);

void ht_i2c_scan(i2c_master_bus_handle_t bus_handle) {
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");

    for(uint8_t i = 3; i < 0x78; i++) {
        if(i % 16 == 0) {
            printf("\n%02x:", i);

            esp_err_t err = i2c_master_probe(bus_handle, i, 100);

            if(err == ESP_OK) {
                printf(" %02x", i);
            }
            else {
                printf(" --");
            }
        }
        printf("\n\n");
    }
}

esp_err_t ht_i2c_write_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, const uint8_t *data, size_t len);

esp_err_t ht_i2c_read_reg16(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint8_t *data, size_t len);

esp_err_t ht_i2c_write_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, const uint8_t *data, size_t len);

esp_err_t ht_i2c_read_reg8(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);



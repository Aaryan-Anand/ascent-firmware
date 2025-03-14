/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <rom/ets_sys.h>
#include <string.h>
#include <stdio.h>

#include "manual_spi_bus.h"

#include "driver_w25qxx_interface.h"
#include "driver_w25qxx_basic.h"

#include "driver_bno055.h"

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    /* bno055 testing */
    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C initialized successfully");

    bno_init();

    while (1) {
        // Read accelerometer
        uint8_t x_lsb = readRegister(BNO_ACC_DATA_X_LSB_ADDR);
        uint8_t x_msb = readRegister(BNO_ACC_DATA_X_MSB_ADDR);
        int16_t x_value = (x_msb << 8) | x_lsb;

        uint8_t y_lsb = readRegister(BNO_ACC_DATA_Y_LSB_ADDR);
        uint8_t y_msb = readRegister(BNO_ACC_DATA_Y_MSB_ADDR);
        int16_t y_value = (y_msb << 8) | y_lsb;

        uint8_t z_lsb = readRegister(BNO_ACC_DATA_Z_LSB_ADDR);
        uint8_t z_msb = readRegister(BNO_ACC_DATA_Z_MSB_ADDR);
        int16_t z_value = (z_msb << 8) | z_lsb;

        // Read gyroscope
        uint8_t gx_lsb = readRegister(BNO_GYR_DATA_X_LSB_ADDR);
        uint8_t gx_msb = readRegister(BNO_GYR_DATA_X_MSB_ADDR);
        int16_t gx_value = (gx_msb << 8) | gx_lsb;

        uint8_t gy_lsb = readRegister(BNO_GYR_DATA_Y_LSB_ADDR);
        uint8_t gy_msb = readRegister(BNO_GYR_DATA_Y_MSB_ADDR);
        int16_t gy_value = (gy_msb << 8) | gy_lsb;

        uint8_t gz_lsb = readRegister(BNO_GYR_DATA_Z_LSB_ADDR);
        uint8_t gz_msb = readRegister(BNO_GYR_DATA_Z_MSB_ADDR);
        int16_t gz_value = (gz_msb << 8) | gz_lsb;

        // Read magnetometer
        uint8_t mx_lsb = readRegister(BNO_MAG_DATA_X_LSB_ADDR);
        uint8_t mx_msb = readRegister(BNO_MAG_DATA_X_MSB_ADDR);
        int16_t mx_value = (mx_msb << 8) | mx_lsb;

        uint8_t my_lsb = readRegister(BNO_MAG_DATA_Y_LSB_ADDR);
        uint8_t my_msb = readRegister(BNO_MAG_DATA_Y_MSB_ADDR);
        int16_t my_value = (my_msb << 8) | my_lsb;

        uint8_t mz_lsb = readRegister(BNO_MAG_DATA_Z_LSB_ADDR);
        uint8_t mz_msb = readRegister(BNO_MAG_DATA_Z_MSB_ADDR);
        int16_t mz_value = (mz_msb << 8) | mz_lsb;

        uint8_t sys_cal, gyro_cal, acc_cal, mag_cal;
        bno_getCalib(&sys_cal, &gyro_cal, &acc_cal, &mag_cal);

        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
               x_value, y_value, z_value,      // Accelerometer
               gx_value, gy_value, gz_value,   // Gyroscope
               mx_value, my_value, mz_value,   // Magnetometer
               sys_cal, gyro_cal, acc_cal, mag_cal); // Calibration status

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    /* end bno055*/

    /* flash testing */
    spi_init();

    uint8_t res;
    uint8_t manufacturer;
    uint8_t device_id;

    res = w25qxx_basic_init(W25Q512, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);

    res = w25qxx_basic_get_id((uint8_t *)&manufacturer, (uint8_t *)&device_id);
    w25qxx_interface_debug_print("w25qxx: manufacturer is 0x%02X device id is 0x%02X.\n", manufacturer, device_id);

    // w25qxx_basic_enable_write();

    uint8_t write[] = "Hello, World!";
    uint8_t read[]  = "this no work!";

    printf("Trying to write: %u\n", sizeof(write));

    res = w25qxx_basic_write(0, (uint8_t *)write, sizeof(write));

    res = w25qxx_basic_read(0, (uint8_t *)read, sizeof(read));

    vTaskDelay(1 / portTICK_PERIOD_MS);

    // w25qxx_basic_disable_write();

    w25qxx_basic_deinit();
    /* end of flash testing */

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

// initial driver that reads and writes registers, and initializes the sensor to read data

#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver_bno055.h"

#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_SDA_IO 6        
#define I2C_MASTER_NUM I2C_NUM_0  
#define I2C_MASTER_FREQ_HZ 400000  
#define I2C_MASTER_TX_BUF_DISABLE 0 
#define I2C_MASTER_RX_BUF_DISABLE 0 
#define DEVICE_I2C_ADDRESS 0x28   

static const char *TAG = "I2C_Example";

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 5,
        .scl_io_num = 6,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static uint8_t readRegister(uint8_t reg_addr) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // First, write the register address we want to read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // Then perform a repeated start and read the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, &data, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return data;
}

static esp_err_t writeRegister(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint8_t bno_getMode() {
    uint8_t currentmode = readRegister(BNO_OPR_MODE_ADDR);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    return currentmode;
}

void bno_setMode(opr_mode mode) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    writeRegister(BNO_OPR_MODE_ADDR, CONFIG);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    writeRegister(BNO_OPR_MODE_ADDR, mode);
}

void bno_init() {
    bno_setMode(IMUPLUS);
    ESP_LOGI(TAG,"BNO Initialized!");
}

void app_main() {

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    bno_init();

    while (1) {
        uint8_t value = readRegister(BNO_ACC_DATA_X_LSB_ADDR);
        printf("Register value: 0x%02X\n", value);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
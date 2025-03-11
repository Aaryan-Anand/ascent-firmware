#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 5        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 6        /*!< GPIO number used for I2C master data */
#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define UBLOX_I2C_ADDRESS 0x42     /*!< Slave address of the UBlox device */

static const char *TAG = "I2C_Example";

// FAST MODE Hex Codes
uint8_t fastModeHex1[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 
                         0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x53, 0xCC};
uint8_t fastModeHex2[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 
                         0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x52, 0xC3};

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

static esp_err_t sendHexCode(uint8_t *hexCodes, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (UBLOX_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, hexCodes, length, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmission error: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void tenhzgps() {
    ESP_LOGI(TAG, "Changing CFG-RATE-MEAS to 100ms...");
    sendHexCode(fastModeHex1, sizeof(fastModeHex1));
    sendHexCode(fastModeHex2, sizeof(fastModeHex2));
    ESP_LOGI(TAG, "Done!");
}

int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
   int i = 0;
   fields[i++] = string;
   while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
      *string = '\0';
      fields[i++] = ++string;
   }
   return --i;
}

static void readNMEA() {
    uint8_t data[255];
    char *fields[20];  // Array to store parsed fields
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (UBLOX_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data) - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        data[sizeof(data) - 1] = '\0';  // Null-terminate the string
        
        // Parse each line of NMEA data
        char *line = (char *)data;
        char *next_line;
        
        while ((next_line = strchr(line, '\n')) != NULL) {
            *next_line = '\0';  // Null-terminate this line
            
            // Only parse if it's a valid NMEA sentence (starts with $)
            if (line[0] == '$') {
                int num_fields = parse_comma_delimited_str(line, fields, 20);
                
                // Only print if it's an RMC, GGA, or VTG message. This will ignore all other messages.
                if (strstr(fields[0], "RMC") != NULL || 
                    strstr(fields[0], "GGA") != NULL || 
                    strstr(fields[0], "VTG") != NULL) {
                    printf("Message Type: %s\n", fields[0]);
                    for (int i = 1; i <= num_fields; i++) {
                        printf("Field %d: %s\n", i, fields[i]);
                    }
                    printf("\n");
                }
            }
            
            line = next_line + 1;  // Move to the start of the next line
        }
    } else {
        ESP_LOGE(TAG, "Failed to read from I2C: %s", esp_err_to_name(ret));
    }
}

void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    tenhzgps();
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // let it cook for a lil lol

    while (1) {
        readNMEA();
        vTaskDelay(333 / portTICK_PERIOD_MS);  // read at 3hz
    }
}
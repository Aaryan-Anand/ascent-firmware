#include "main.h"

#ifndef MAIN_H
#include "nvs_interface.h"
#include "esp_err.h"
#include "esp_system.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "flash_interface.h"
#include "beep.h"
#include "driver_buzzer.h"
#include "ascent_r2_hardware_definition.h"
#include "spi_manager.h"

uint8_t uuid[16] = { 0x4, 0x1, 0x7, 0x3, 0x6, 0xe, 0x7, 0x4, 0x5, 0x2, 0x2, 0x0, 0x3, 0x0, 0x0, 0x0 };

static float acc_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float gyr_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float mag_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
static float high_g_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float acc_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float gyr_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float high_g_bias_vector[3] = {0.0f, 0.0f, 0.0f};
#endif

void set_uuid(void) {
    printf("Setting UUID\n");
   nvs_handle_t my_handle = nvs_interface_get_handle();
    esp_err_t err;
    if ((err = nvs_set_blob(my_handle, "uuid", uuid, sizeof(uuid))) != ESP_OK) {
        printf("Failed to set uuid\n");
        printf("%d\n", err);
        esp_restart();
    }
    printf("Set UUID\n");
}

void set_matrices(void) {
    printf("Setting correction matrices\n");
    // if this 0 is changed to a 1 the matrices in the code above will be loaded
    // to the nvs flash, if it is 0 then the matrices from the flash will be
    // written to those variables

    // mats
    esp_err_t err;
    nvs_handle_t my_handle = nvs_interface_get_handle();
    if ((err = nvs_set_blob(my_handle, "acc_mat", acc_correction_matrix, sizeof(acc_correction_matrix))) != ESP_OK) {
        printf("Failed to set acc_correction_matrix\n");
        printf("%d\n", err);
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "gyr_mat", gyr_correction_matrix, sizeof(gyr_correction_matrix)) != ESP_OK) {
        printf("Failed to set gyr_correction_matrix\n");
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "mag_mat", mag_correction_matrix, sizeof(mag_correction_matrix)) != ESP_OK) {
        printf("Failed to set mag_correction_matrix\n");
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "high_g_mat", high_g_correction_matrix, sizeof(high_g_correction_matrix)) != ESP_OK) {
        printf("Failed to set high_g_correction_matrix\n");
        esp_restart();
    }


    // vectors
    if (nvs_set_blob(my_handle, "acc_vec", acc_bias_vector, sizeof(acc_bias_vector)) != ESP_OK) {
        printf("Failed to set acc_bias_vector\n");
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "gyr_vec", gyr_bias_vector, sizeof(gyr_bias_vector)) != ESP_OK) {
        printf("Failed to set gyr_bias_vector\n");
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "mag_vec", mag_bias_vector, sizeof(mag_bias_vector)) != ESP_OK) {
        printf("Failed to set mag_bias_vector\n");
        esp_restart();
    }

    if (nvs_set_blob(my_handle, "high_g_vec", high_g_bias_vector, sizeof(high_g_bias_vector)) != ESP_OK) {
        printf("Failed to set high_g_bias_vector\n");
        esp_restart();
    }

    printf("biases and mats written\n");
}

void config_beep(void) {
    note(NOTE_C, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_E, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_G, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_C, OCTAVE_5, 75);
    vTaskDelay(pdMS_TO_TICKS(500));
    note(NOTE_E, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_C, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_E, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_C, OCTAVE_4, 75);
}

void app_main(void) {
    printf("Configuring your Ascent R2 Board!\n");
    buzzer_init();
    config_beep();
    nvs_interface_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    spi_manager_init(SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
    vTaskDelay(pdMS_TO_TICKS(50));
    flash_flight_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_matrices();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_uuid();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    flash_blank_slate();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    flash_erase_jingle();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Done\n");
}
#include "nvs_interface.h"

#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <rom/ets_sys.h>

static nvs_handle_t my_handle;

void nvs_interface_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // nvs_flash_erase();
        // nvs_flash_init();

        // here the nvs flash has been over run for some reason and needs to be
        // totally erased
        // so we don't lose bank data this throws an error and
        // requires a manual recompile to call the above functions after all
        // data from banks has been saved and the whole external flash chip has
        // been erased.
        // use the above two commented out lines...

        printf("Failed initalize nvs flash\n");

        esp_restart();
    }

    if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) {
        printf("Failed open nvs storage\n");
    }
}

nvs_handle_t nvs_interface_get_handle(void)
{
    return my_handle;
}

void nvs_set_data(char *key, void *data){
    my_handle = nvs_interface_get_handle();
    size_t length = sizeof(data);
    if (nvs_set_blob(my_handle, key, data, length) != ESP_OK) {
        printf("Failed to set %s\n", key);
        esp_restart();
    }
}

void nvs_retreive_data(char *key, void *data){
    my_handle = nvs_interface_get_handle();
    size_t length = sizeof(data);
    if (nvs_get_blob(my_handle, key, data, length) != ESP_OK) {
        printf("Failed to get %s\n", key);
        esp_restart();
    }
}

void nvs_retreive_matrices(float *acc_correction_matrix, float *gyr_correction_matrix, float *mag_correction_matrix, float *high_g_correction_matrix, float *acc_bias_vector, float *gyr_bias_vector, float *mag_bias_vector, float *high_g_bias_vector) {
    my_handle = nvs_interface_get_handle();

    size_t length = sizeof(&acc_correction_matrix);

    // mats
    length = sizeof(&acc_correction_matrix);
    if (nvs_find_key(my_handle, "acc_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "acc_mat", &acc_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load acc_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&gyr_correction_matrix);
    if (nvs_find_key(my_handle, "gyr_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "gyr_mat", &gyr_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load gyr_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&mag_correction_matrix);
    if (nvs_find_key(my_handle, "mag_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "mag_mat", &mag_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load mag_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&high_g_correction_matrix);
    if (nvs_find_key(my_handle, "high_g_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "high_g_mat", &high_g_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load high_g_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }


    // vectors
    length = sizeof(&acc_bias_vector);
    if (nvs_find_key(my_handle, "acc_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "acc_vec", &acc_bias_vector, &length) != ESP_OK) {
            printf("Failed to load acc_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&gyr_bias_vector);
    if (nvs_find_key(my_handle, "gyr_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "gyr_vec", &gyr_bias_vector, &length) != ESP_OK) {
            printf("Failed to load gyr_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&mag_bias_vector);
    if (nvs_find_key(my_handle, "mag_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "mag_vec", &mag_bias_vector, &length) != ESP_OK) {
            printf("Failed to load mag_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(&high_g_bias_vector);
    if (nvs_find_key(my_handle, "high_g_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "high_g_vec", &high_g_bias_vector, &length) != ESP_OK) {
            printf("Failed to load high_g_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }
}

void nvs_retreive_uuid(uint8_t *uuid[16]){
    {
        nvs_handle_t my_handle = nvs_interface_get_handle();
        // if this is a 1 it will write the uuid to the nvs flash
        // if it is 0 it will load the correct value from the nvs flash
        
        if (nvs_find_key(my_handle, "uuid", NULL) == ESP_OK) {
            size_t length = sizeof(&uuid);
            if (nvs_get_blob(my_handle, "uuid", &uuid, &length) != ESP_OK) {
                printf("Failed to load uuid\n");
                esp_restart();
            }
        } else {
            printf("using fallback uuid\n");
        }
    }
}
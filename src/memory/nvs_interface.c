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
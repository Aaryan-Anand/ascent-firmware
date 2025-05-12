#include "datalogging.h"

#include "math.h"
#include "driver_pyro.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "interface_bmp390l.h"
#include "driver_pyro.h"
#include "globals.h"
#include "flight_config.h"  // For POWERED_ALT, APOGEE_MIN, and MAINS_ALT
#include <assert.h>
#include "flightState_manager.h"
#include "neopixel.h"
#include "driver_w25qxx.h"

uint32_t addr = 4096;
static flash_packet fp;
uint32_t sub_addr = 0;

void save_addr() {
    uint8_t res = w25qxx_sector_erase(0);
    res = w25qxx_write(0, &addr, 4);
    printf("Saving %ld rs: %d\n", addr, res);
    uint32_t read;
    res = w25qxx_read(0, &read, 4);
    printf("Read back %ld res: %d\n\n", read, res);
}

void recall_addr() {
    w25qxx_read(0, &addr, 4);
}

void begin_datalogging() {
    addr = 4096;
    uint8_t res;
    res = w25qxx_init();
    if (res) printf("Failed to initalize flash\n");

    if (pyro_continuity(PYRO_CHANNEL_1) && pyro_continuity(PYRO_CHANNEL_2)) {
        printf("DOING CHIP ERASE\n");
        recall_addr();
        // TODO: remove this stupid check
        if (addr == 0xFFFFFFFF || addr == 0) addr = 100*4096;
        uint32_t n = ceil(addr / 4096)+2;
        printf("Addr: %ld, erasing: %ld\n", addr, n);
        addr = 4096;
        // res = w25qxx_chip_erase();
        // if (res) fail_state(FAIL_FLASH_CHIP_ERASE);

        for (int i = 0; i < n; i++) {
            res = w25qxx_sector_erase(i*4096);
            if (res) printf("Failed to erase sector\n");
        }
    } else {
        printf("DUMPING DATA\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        recall_addr();
        // TODO: remove this stupid check
        // if (addr == 0xFFFFFFFF) addr = 100*4096;
        uint32_t n = ceil(addr / 4096)+2;
        printf("Addr: %ld, used: %ld\n", addr, n);
        addr = 4096;
        while (1) {
            w25qxx_read(addr, &fp, sizeof(flash_packet));
            addr += sizeof(flash_packet);

            bool all = true;
            char* buf = (char*) &fp;
            for (int i = 0; i < sizeof(flash_packet); i++) {
                if (buf[i] != 0xFF) all=false;
            }
            if (all) break;
   printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %"PRId64", %f, %f, %f, %lu, %f, %f, %f, %d, %d\n",
                fp.acc_x,
                fp.acc_y,
                fp.acc_z,
                fp.mag_x,
                fp.mag_y,
                fp.mag_z,
                fp.gyr_x,
                fp.gyr_y,
                fp.gyr_z,
                fp.x_accel,
                fp.y_accel,
                fp.z_accel,
                fp.timestamp,
                fp.latitude,
                fp.longitude,
                fp.barometric_agl,
                fp.gps_altitude,
                fp.barometric_velocity,
                fp.average_barometric_velocity,
                fp.acceleration,
                fp.pyro_arm,
                fp.flight_state);
        }

        save_addr();

    //     while (1) {
    //         neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  255) } }, 1);
    //         vTaskDelay(500/portTICK_PERIOD_MS);
    //         neopixel_SetPixel(neopixel, (tNeopixel[]){ { 0, NP_RGB(0, 0,  0) } }, 1);
    //         vTaskDelay(500/portTICK_PERIOD_MS);

    //         vTaskDelay(1000/portTICK_PERIOD_MS);
    //     }
    }
}

void write_flash_packet() {
    fp.acc_x = acc.x;
    fp.acc_y = acc.y;
    fp.acc_z = acc.z;
    fp.mag_x = mag.x;
    fp.mag_y = mag.y;
    fp.mag_z = mag.z;
    fp.gyr_x = gyr.x;
    fp.gyr_y = gyr.y;
    fp.gyr_z = gyr.z;
    fp.x_accel = high_g_acc.x;
    fp.y_accel = high_g_acc.y;
    fp.z_accel = high_g_acc.z;
    // fp.timestamp = start_time;
    fp.latitude = latitude;
    fp.longitude = longitude;
    fp.barometric_agl = barometric_agl;
    fp.gps_altitude = gps_altitude;
    fp.barometric_velocity = barometric_velocity;
    fp.average_barometric_velocity = average_barometric_velocity;
    fp.acceleration = acceleration;
    fp.pyro_arm = pyro_arm;
    fp.flight_state = flight_state;

    w25qxx_write(addr, &fp, sizeof(flash_packet));
    addr += sizeof(flash_packet);
    sub_addr += sizeof(flash_packet);
    if (sub_addr >= 4096) {
        save_addr();
        sub_addr = sub_addr-4096;
    }
}
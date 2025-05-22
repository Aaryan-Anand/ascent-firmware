#include "flash_interface.h"

#include "stdatomic.h"
#include "stdlib.h"
#include "fail.h"
#include "driver_w25qxx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ascent_r2_hardware_definition.h"
#include <rom/ets_sys.h>

#define MAX_SECTORS 16384
#define FLIGHT_LOG_START_ADDR 4096
#define SECTOR_SIZE 4096

static uint32_t addr = FLIGHT_LOG_START_ADDR;
static uint32_t sub_addr = FLIGHT_LOG_START_ADDR;

#define RING_BUFFER_SIZE 50
flash_packet ring_buffer[RING_BUFFER_SIZE];
_Atomic uint32_t write_head = 0;
_Atomic uint32_t read_head = 0;
// SemaphoreHandle_t write_head_semaphore;
// SemaphoreHandle_t read_head_semaphore;
SemaphoreHandle_t head_semaphore;

static void save_addr() {
    uint8_t res = w25qxx_sector_erase(0);
    res = w25qxx_write(0, &addr, 4);
    // printf("Saving %ld rs: %d\n", addr, res);
    uint32_t read;
    res = w25qxx_read(0, &read, 4);
    if (addr != read) {
        // TODO: something has gone very wrong
    }
    // printf("Read back %ld res: %d\n\n", read, res);
}

static void recall_addr() {
    w25qxx_read(0, &addr, 4);
}

void flash_flight_init(void)
{
    uint8_t res;

    res = w25qxx_init();
    if (res) fail_state(FAIL_FLASH_INIT);

    addr = sub_addr = FLIGHT_LOG_START_ADDR;

    // write_head_semaphore = xSemaphoreCreateMutex();
    // read_head_semaphore = xSemaphoreCreateMutex();
    head_semaphore = xSemaphoreCreateMutex();
}

void flash_prepare_for_flight(void) {
    uint8_t res;

    printf("DOING CHIP ERASE\n");
    recall_addr();
    // TODO: remove this stupid check
    if (addr == 0xFFFFFFFF || addr == 0) addr = 100*SECTOR_SIZE;
    uint32_t n = ceil(addr / SECTOR_SIZE)+2;
    // printf("Addr: %ld, erasing: %ld\n", addr, n);
    addr = FLIGHT_LOG_START_ADDR;
    // res = w25qxx_chip_erase();
    // if (res) fail_state(FAIL_FLASH_CHIP_ERASE);

    for (int i = 0; i < n; i++) {
        res = w25qxx_sector_erase(i*4096);
        if (res) fail(FAIL_FLASH_CHIP_ERASE);
    }
}

void flash_dump_to_serial(void) {
    flash_packet fp;

    printf("DUMPING DATA\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    recall_addr();
    // TODO: remove this stupid check
    // if (addr == 0xFFFFFFFF) addr = 100*FLIGHT_LOG_START_ADDR;
    uint32_t n = ceil(addr / SECTOR_SIZE)+2;
    if (n > MAX_SECTORS) n = MAX_SECTORS;

    printf("Addr: %ld, used: %ld\n", addr, n);
    addr = FLIGHT_LOG_START_ADDR;
    while (1) {
        w25qxx_read(addr, &fp, sizeof(flash_packet));
        addr += sizeof(flash_packet);

        bool all = true;
        char* buf = (char*) &fp;
        for (int i = 0; i < sizeof(flash_packet); i++) {
            if (buf[i] != 0xFF) all=false;
        }
        if (all) break;

        printf("%"PRId64",", fp.timestamp);
        printf("%d,", fp.pyro_arm);

        printf("%d,", fp.acc.x);
        printf("%d,", fp.acc.y);
        printf("%d,", fp.acc.z);

        printf("%d,", fp.gyr.x);
        printf("%d,", fp.gyr.y);
        printf("%d,", fp.gyr.z);

        printf("%d,", fp.mag.x);
        printf("%d,", fp.mag.y);
        printf("%d,", fp.mag.z);

        printf("%f,", fp.high_g_acc.x);
        printf("%f,", fp.high_g_acc.y);
        printf("%f,", fp.high_g_acc.z);

        printf("%f,", fp.baro.alt);
        printf("%f,", fp.baro.pressure);
        printf("%f,", fp.baro.temperature);

        printf("%f,", fp.barometric_agl);
        printf("%f,", fp.barometric_velocity);
        printf("%f,", fp.average_barometric_velocity);

        printf("%f,", fp.latitude);
        printf("%f,", fp.longitude);
        printf("%ul,", fp.gps_altitude);
        
        
        printf("%f,", fp.ekf_latitude);
        printf("%f,", fp.ekf_longitude);
        printf("%f,", fp.ekf_altitude);

        printf("%f,", fp.ekf_pitch);
        printf("%f,", fp.ekf_yaw);
        printf("%f,", fp.ekf_roll);
    }

    save_addr();

    printf("FINISHED DUMPING DATA\n");
}

void flash_write_packet(flash_packet *packet) {
    w25qxx_write(addr, packet, sizeof(flash_packet));
    addr += sizeof(flash_packet);
    sub_addr += sizeof(flash_packet);
    if (sub_addr >= SECTOR_SIZE) {
        save_addr();
        sub_addr = sub_addr-SECTOR_SIZE;
    }
}

void flash_queue_packet(flash_packet *packet) {
    while (1) {
        if (xSemaphoreTake(head_semaphore, pdMS_TO_TICKS(MUTEX_TIMEOUT))) {
            if (write_head != read_head) break;

            xSemaphoreGive(head_semaphore);
        }

        ets_delay_us(10);
    }

    memcpy(&ring_buffer[write_head], packet, sizeof(flash_packet));

    if (xSemaphoreTake(head_semaphore, pdMS_TO_TICKS(MUTEX_TIMEOUT))) {
        write_head = (write_head + 1) % RING_BUFFER_SIZE;

        xSemaphoreGive(head_semaphore);
    }
}

void flash_write_queue(int64_t max_time) {
        int64_t start_time = esp_timer_get_time();

        while (1) {
            if (xSemaphoreTake(head_semaphore, pdMS_TO_TICKS(MUTEX_TIMEOUT))) {
                if (read_head == write_head) return;

                xSemaphoreGive(head_semaphore);
            }

            flash_write_packet(&ring_buffer[read_head]);

            if (xSemaphoreTake(head_semaphore, pdMS_TO_TICKS(MUTEX_TIMEOUT))) {
                read_head = (read_head + 1) % RING_BUFFER_SIZE;

                xSemaphoreGive(head_semaphore);
            }

            int64_t delta = esp_timer_get_time() - start_time;
            if (delta > max_time) break;
        }

}
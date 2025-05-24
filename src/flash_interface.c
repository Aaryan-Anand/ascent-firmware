#include "flash_interface.h"

#include "stdatomic.h"
#include "assert.h"
#include "math.h"
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

static uint32_t n = 0;
static uint32_t addr = FLIGHT_LOG_START_ADDR;
static uint32_t sub_addr = FLIGHT_LOG_START_ADDR;

#define RING_BUFFER_SIZE 50
QueueHandle_t flash_packet_queue;

static void save_addr() {
    uint8_t res = w25qxx_sector_erase(0);
    res = w25qxx_write(0, (uint8_t*)&addr, 4);
    // printf("Saving %ld rs: %d\n", addr, res);
    uint32_t read;
    res = w25qxx_read(0, (uint8_t*)&read, 4);
    if (addr != read) {
        // TODO: something has gone very wrong
    }
    // printf("Read back %ld res: %d\n\n", read, res);
}

static void recall_addr() {
    w25qxx_read(0, (uint8_t*)&addr, 4);
}

uint32_t flash_get_addr() {
    recall_addr();
    return addr;
}

void flash_flight_init(void)
{
    uint8_t res;

    res = w25qxx_init();
    if (res) fail(FAIL_FLASH_INIT);

    addr = sub_addr = FLIGHT_LOG_START_ADDR;

    flash_packet_queue = xQueueCreate(RING_BUFFER_SIZE, sizeof(flash_packet));
    assert(flash_packet_queue != NULL);
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
        printf("Erasing %d/%ld\n", i, n);
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
        w25qxx_read(addr, (uint8_t*)&fp, sizeof(flash_packet));
        addr += sizeof(flash_packet);

        bool all = true;
        char* buf = (char*) &fp;
        for (int i = 0; i < sizeof(flash_packet); i++) {
            if (buf[i] != 0xFF) all=false;
        }
        if (all) break;

        printf("%lu,", fp.n);
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
        printf("%lu,", fp.gps_altitude);
        
        
        printf("%f,", fp.ekf_latitude);
        printf("%f,", fp.ekf_longitude);
        printf("%f,", fp.ekf_altitude);

        printf("%f,", fp.ekf_pitch);
        printf("%f,", fp.ekf_yaw);
        printf("%f,", fp.ekf_roll);

        printf("\n");
    }

    save_addr();

    printf("FINISHED DUMPING DATA\n");
}

void flash_write_packet(flash_packet *packet) {
    w25qxx_write(addr, (uint8_t*) packet, sizeof(flash_packet));
    addr += sizeof(flash_packet);
    sub_addr += sizeof(flash_packet);
    if (sub_addr >= SECTOR_SIZE) {
        save_addr();
        sub_addr = sub_addr-SECTOR_SIZE;
    }
}

void flash_queue_packet(flash_packet *packet) {
    packet->n = n++;
    while (xQueueSendToBack(flash_packet_queue, packet, pdMS_TO_TICKS(MUTEX_TIMEOUT)) != pdTRUE) {
        // vTaskDelay(pdMS_TO_TICKS(1)); 
        printf("FLASH PACKET LOST");
    }
}

void flash_write_queue(int64_t max_time) {
    int64_t start = esp_timer_get_time();
    flash_packet packet;

    static UBaseType_t max_count = 0;
    UBaseType_t count = uxQueueMessagesWaiting(flash_packet_queue);
    max_count = count > max_count ? count : max_count;
    printf("ITEMS IN QUEUE: %u, %u\n", count, max_count);

    while ((esp_timer_get_time() - start) < max_time) {
        if (xQueueReceive(flash_packet_queue, &packet, 0) == pdTRUE) {
            flash_write_packet(&packet);
        } else {
            break; // Queue is empty
        }
    }
}
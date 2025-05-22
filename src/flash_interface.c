#include "flash_interface.h"

#include "fail.h"
#include "driver_w25qxx.h"
#include "freertos/FreeRTOS.h"

#define MAX_SECTORS 16384
#define FLIGHT_LOG_START_ADDR 4096
#define SECTOR_SIZE 4096

static uint32_t addr = FLIGHT_LOG_START_ADDR;
static uint32_t sub_addr = FLIGHT_LOG_START_ADDR;

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
        /*
    int64_t timestamp;

    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;

    double x_accel, y_accel, z_accel;

    float latitude;
    float longitude;
    uint32_t gps_altitude;

    float barometric_agl;

    uint8_t pyro_arm;
    uint8_t flight_state;

    float ekf_latitude;
    float ekf_longitude;
    float ekf_altitude;
    float ekf_pitch;
    float ekf_yaw;
    float ekf_roll;

    // printf("Delta: %" PRId64 "us or %ldms or %f\n", delta, time_ms, 1.0f/(time_ms/1000.0f));
    */
        printf("%"PRId64", %d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %lu, %f, %d, %d, %f, %f, %f, %f, %f, %f, %f\n",
            fp.timestamp,

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

            fp.latitude,
            fp.longitude,
            fp.gps_altitude,

            fp.barometric_agl,
            fp.pyro_arm,
            fp.flight_state,

            fp.ekf_latitude,
            fp.ekf_longitude,
            fp.ekf_altitude,
            fp.ekf_pitch,
            fp.ekf_yaw,
            fp.ekf_roll
        );
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
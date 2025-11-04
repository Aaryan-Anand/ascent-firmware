#ifndef GOOBER_H
#define GOOBER_H

#include "stdbool.h"
#include "stdint.h"

#include "flight_config.h"
#include "lora_types.h"

// lora_config_t provided by lora_types.h

// ONE BYTE MESSAGE PAYLOADS
 
#define REQ_PINGPONG         0xFF  // Expect POST_PINGPONG
#define REQ_TELEM            0x66  // Expect POST_TELEM
#define REQ_AUX_ACTIVATE     0x6E  // Expect POST_TELEM
#define REQ_AUX_DEACTIVATE   0x6F  // Expect POST_TELEM
#define REQ_TXLOCK_ACTIVATE  0x78  // Expect POST_TXLOCK_ACTIVATE
#define REQ_REBOOT           0x82  // Expect slave reboot (and a beep)

#define POST_PINGPONG        0x01
#define POST_TXLOCK_ACTIVATE 0x79

// GOOBER Message Types and Structures

typedef enum {
    // POST message types
    MSG_TYPE_POST_PINGPONG = 0xFF,
    MSG_TYPE_POST_TXLOCK_ACTIVATE = 0x83,
    MSG_TYPE_POST_TELEM = 0xCA,
    MSG_TYPE_POST_LOCATE = 0xE3,
    MSG_TYPE_REQ_PINGPONG = 0x01,
    MSG_TYPE_REQ_TELEM = 0x02,
    MSG_TYPE_REQ_AUX_ACTIVATE = 0x0A,
    MSG_TYPE_REQ_AUX_DEACTIVATE = 0x0B,
    MSG_TYPE_REQ_TXLOCK_ACTIVATE = 0x14,
    MSG_TYPE_REQ_REBOOT = 0x1E,
    MSG_TYPE_REQ_POP_APOGEE = 0x69,
    MSG_TYPE_REQ_POP_MAINS = 0x6A,
    MSG_TYPE_REQ_WAKEUP = 0x6B,
    MSG_TYPE_NVS_DUMP = 0x70,
    MSG_TYPE_NVS_DUMP_FLIGHT = 0x71,
    MSG_TYPE_NVS_DUMP_LORA = 0x72,
    MSG_TYPE_NVS_EDIT_FLIGHT = 0x73,
    MSG_TYPE_NVS_EDIT_LORA = 0x74
} goober_msg_type_t;

typedef struct {
    uint32_t timestamp;        // 4 bytes
    int32_t latitude;          // 4 bytes
    int32_t longitude;         // 4 bytes
    float altitude_agl;        // 4 bytes
    float vertical_velocity;   // 4 bytes
    float x_acc;               // 4 bytes
    float gyr_x;               // 4 bytes
    uint8_t pyro_state;        // 1 byte
    uint8_t sats;              // 1 byte
    uint8_t flight_state;      // 1 byte
    uint16_t battery_voltage;  // 2 byte
} goober_post_telemetry_payload_t;

typedef struct {
    int64_t timestamp;         // 8 bytes
    int32_t latitude;          // 4 bytes
    int32_t longitude;         // 4 bytes
} goober_post_locator_payload_t; // total: 16 bytes

typedef struct {
    uint8_t single_byte_payload; // 1 byte payload
} goober_post_single_byte_payload_t;

typedef struct {
    flight_config_t flight_config;
    lora_config_t lora_config;
} nvs_dump_t;

// Union of payloads

typedef union goober_payload {
    goober_post_telemetry_payload_t telemetry;
    goober_post_locator_payload_t   locate;
    goober_post_single_byte_payload_t single_byte;
    flight_config_t flight_config;
    lora_config_t lora_config;
    nvs_dump_t nvs_dump;
    uint8_t raw[ sizeof(goober_post_telemetry_payload_t) ];
} goober_payload_t;

// GOOBER Message Structure

typedef struct goober {
    uint8_t DEV_ID;         // 1 byte
    uint8_t DEV_MODE;       // 1 byte
    uint8_t SEQ_ID;         // 1 byte
    uint8_t MSG_CLS;        // 1 byte

    uint8_t PAYLOAD_SIZE;   // 1 byte
    goober_payload_t payload; // N bytes

} goober_t;

goober_t gooberCreatePacket(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload);

goober_t gooberSlaveResponse(goober_t master_msg, goober_payload_t telemetry);

goober_payload_t gooberCreateTelemetry(int32_t latitude, int32_t longitude, float altitude_agl, float vertical_velocity, float x_acc, float eul_x, float eul_y, float eul_z, float gyr_x, uint8_t sats, uint8_t flight_state);

goober_t gooberParse(uint8_t *rx_buffer, uint8_t rx_buffer_size);

void gooberSerialize(goober_t *packet, uint8_t *tx_buffer, uint8_t tx_buffer_size);

bool is_tx_lock();

bool should_wake_up();

void turn_off_cameras(void);
void turn_off_fan(void);
void fake_tx_lock(void);
void turn_on_cameras(void);
void turn_on_fan(void);
            

void activate_txlock();
void deactivate_txlock();

#endif
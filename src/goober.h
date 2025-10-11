#ifndef GOOBER_H
#define GOOBER_H

#include "stdbool.h"
#include "stdint.h"

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
    MSG_TYPE_REQ_WAKEUP = 0x6B
} goober_msg_type_t;

typedef struct {
    int64_t timestamp;         // 8 bytes
    int32_t latitude;          // 4 bytes
    int32_t longitude;         // 4 bytes
    float altitude_agl;        // 4 bytes
    float vertical_velocity;   // 4 bytes
    float x_acc;               // 4 bytes
    float eul_x;               // 4 bytes
    float eul_y;               // 4 bytes
    float eul_z;               // 4 bytes
    float gyr_x;               // 4 bytes
    uint8_t pyro_state;        // 1 byte
    uint8_t sats;              // 1 byte
    uint8_t flight_state;      // 1 byte
    uint32_t battery_voltage;     // 4 bytes
} goober_post_telemetry_payload_t; // total: 51 bytes

typedef struct {
    int64_t timestamp;         // 8 bytes
    int32_t latitude;          // 4 bytes
    int32_t longitude;         // 4 bytes
} goober_post_locator_payload_t; // total: 16 bytes

typedef struct {
    uint8_t single_byte_payload; // 1 byte payload
} goober_post_single_byte_payload_t;

// Union of payloads

typedef union {
    goober_post_telemetry_payload_t telemetry;
    goober_post_locator_payload_t   locate;
    goober_post_single_byte_payload_t single_byte;
    uint8_t raw[ sizeof(goober_post_telemetry_payload_t) ];
} goober_payload_t;

// GOOBER Message Structure

typedef struct {
    uint8_t DEV_ID;         // 1 byte
    uint8_t DEV_MODE;       // 1 byte
    uint8_t SEQ_ID;         // 1 byte
    uint8_t MSG_CLS;        // 1 byte

    uint8_t PAYLOAD_SIZE;   // 1 byte
    goober_payload_t payload; // N bytes

} goober_t;

goober_t create_packet(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload);

goober_t decode_packet(uint8_t *rx_buffer, uint8_t rx_buffer_size, goober_payload_t telemetry);

goober_t create_response_packet(goober_msg_type_t message_type);

goober_payload_t create_telemetry_payload(int32_t latitude, int32_t longitude, float altitude_agl, float vertical_velocity, float x_acc, float eul_x, float eul_y, float eul_z, float gyr_x, uint8_t sats, uint8_t flight_state);

bool is_tx_lock();
bool should_wake_up();
#endif
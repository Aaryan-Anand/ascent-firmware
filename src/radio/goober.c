#include "goober.h"
#include "driver_pyro.h"
#include "stdatomic.h"
#include "string.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "sensor_manager.h"
#include "driver_buzzer.h"
#include "flash_interface.h"
#include "driver_psu.h"

#define TELEM_PAYLOAD_SIZE 32 // matches sizeof(goober_post_telemetry_payload_t)
#define SLAVE_DEV_ID 0x41

// #define GOOBER_DEBUG

static bool CAMERA_ACTIVE = false;

static bool TXLOCK = false;

_Atomic bool thread_safe_txlock = false;
_Atomic bool thread_safe_should_wakeup = true;

// ASCENT Functions

#define APPO PYRO_CHANNEL_1
#define MAINS PYRO_CHANNEL_2

static bool deploy(pyro_channel_t channel)
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(channel);
        if (cont) {
            pyro_activate(channel, 150*(i+1), 0);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(channel);
            if (!cont) return true;
        }
    }

    return false;
}

void flash_erase_jingle(void) {
    note(NOTE_E, 8, 120);
    note(NOTE_G, 8, 120);
    note(NOTE_C, 7, 200);
    note(NOTE_D, 7, 120);
    note(NOTE_B, 6, 250);
    note(NOTE_E, 7, 400);
}

void turn_on_fan(void) {
    pyro_activate(PYRO_CHANNEL_4,0,1); 
}

void turn_off_fan(void) {
    pyro_activate(PYRO_CHANNEL_4,1,1); 
}

void fake_tx_lock(void) { // DO NOT REMOVE, required to allow flight with only one way coms - Luke
	TXLOCK = flash_prepare_for_flight();
	bmp_aquire_ground();
	flash_erase_jingle();
	atomic_store(&thread_safe_txlock, true);
	printf("Faked tx lock ready to fly.\n");
}

void turn_on_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,0,1); 
    CAMERA_ACTIVE = true;
}

void turn_off_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,1,1); 
    CAMERA_ACTIVE = false;
}

// GOOBER Functions

goober_t gooberCreatePacket(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload) {
    goober_t packet;
    packet.DEV_ID = dev_id;

    packet.DEV_MODE = 0;
    if (is_master) {
		  packet.DEV_MODE |= (1 << 1);
	  }
    if (tx_intent) {
		  packet.DEV_MODE |= (1 << 2);
	  }
    if (tx_lock) {
		  packet.DEV_MODE |= (1 << 3);
	  }

    static uint8_t seq_counter = 0x01;
    packet.SEQ_ID = seq_counter++;
    
    if (seq_counter == 0x00 || seq_counter == 0xFF) {
		seq_counter = 0x01;
	}

    packet.MSG_CLS = (uint8_t)message_class;

    packet.PAYLOAD_SIZE = payload_size;

    packet.payload = *payload;

    return packet;
}

goober_t gooberSlaveResponse(goober_t master_msg, goober_payload_t telemetry) {
	goober_t resp;

	goober_msg_type_t resp_msg_cls = 0;
	goober_payload_t resp_payload;
	uint8_t resp_msg_payload_len = 0;

	switch (master_msg.MSG_CLS) {
		case MSG_TYPE_REQ_TELEM: {
			#ifdef LORA_DEBUG
			printf("Received REQ_TELEM\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_ACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_AUX_ACTIVATE\n");
			#endif
			turn_on_cameras();
			turn_on_fan();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_DEACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_AUX_DEACTIVATE\n");
			#endif
			turn_off_cameras();
			turn_off_fan();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			break;
		}
		case MSG_TYPE_REQ_TXLOCK_ACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_TXLOCK_ACTIVATE\n");
			#endif
			TXLOCK = flash_prepare_for_flight();
			bmp_aquire_ground();
			flash_erase_jingle();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			atomic_store(&thread_safe_txlock, true);
			break;
		}
		case MSG_TYPE_REQ_REBOOT: {
			#ifdef LORA_DEBUG
			printf("Received REQ_REBOOT\n");
			#endif
			esp_restart();
			break;
		}
		case MSG_TYPE_REQ_PINGPONG: {
			#ifdef LORA_DEBUG
			printf("Received REQ_PINGPONG\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_PINGPONG;
			resp_payload.single_byte.single_byte_payload = 0x01; // one byte payloads will need to be revisted. - abdul
			resp_msg_payload_len = 1;
			break;
		}
		case MSG_TYPE_REQ_POP_APOGEE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_POP_APOGEE\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			deploy(APPO);
			break;
		}
		case MSG_TYPE_REQ_POP_MAINS: {
			#ifdef LORA_DEBUG
			printf("Received REQ_POP_MAINS\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;
			deploy(MAINS);
			break;
		}
		case MSG_TYPE_REQ_WAKEUP: { // acts as a toggle! this will need to be revisted. - abdul
			printf("Received REQ_WAKEUP\n");
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_payload = telemetry;
			resp_msg_payload_len = TELEM_PAYLOAD_SIZE;

			vTaskDelay(5000 / portTICK_PERIOD_MS); 

			bool value = atomic_load(&thread_safe_should_wakeup);
			atomic_store(&thread_safe_should_wakeup, !value);

			break;
		}
		default: {
			#ifdef LORA_DEBUG
			printf("Unknown MSG_TYPE: 0x%X\n", request_msg_type);
			#endif
			break;
		}
	}

	if (resp_msg_cls != 0) {
		resp = gooberCreatePacket(SLAVE_DEV_ID, 0, 0, 0, resp_msg_cls, resp_msg_payload_len, &resp_payload);
		resp.SEQ_ID = master_msg.SEQ_ID;
	} else {
		resp = gooberCreatePacket(SLAVE_DEV_ID, 0, 0, 0, MSG_TYPE_POST_TELEM, TELEM_PAYLOAD_SIZE, &telemetry); // this should return an error for obvious reasons, but will transmit telemetry if msg type is not recognized.
		resp.SEQ_ID = master_msg.SEQ_ID;
	}

	return resp;
}

goober_payload_t gooberCreateTelemetry(int32_t latitude, int32_t longitude, float altitude_agl, float vertical_velocity, float x_acc, float eul_x, float eul_y, float eul_z, float gyr_x, uint8_t sats, uint8_t flight_state)
{
	goober_payload_t payload;

	#ifdef LORA_DEBUG
	// printf("Creating telemetry payload\n");
	// printf("timestamp: %lld\n", payload.telemetry.timestamp);
	#endif

    payload.telemetry.timestamp = esp_timer_get_time() / 1000; // convert us to ms

	#ifdef LORA_DEBUG
	// printf("timestamp: %lld\n", payload.telemetry.timestamp);
	#endif
    payload.telemetry.latitude = latitude;
    payload.telemetry.longitude = longitude;
    payload.telemetry.altitude_agl = altitude_agl;
    payload.telemetry.vertical_velocity = vertical_velocity;
    payload.telemetry.x_acc = x_acc;
    payload.telemetry.gyr_x = gyr_x;

	uint8_t pyro_arm = 0;

	if (pyro_continuity(PYRO_CHANNEL_1)) {
		pyro_arm |= (1);
	}
	if (pyro_continuity(PYRO_CHANNEL_2)) {
		pyro_arm |= (1 << 1);
	}
	if (pyro_continuity(PYRO_CHANNEL_3)) {
		pyro_arm |= (1 << 2);
	}
	if (pyro_continuity(PYRO_CHANNEL_4)) {
		pyro_arm |= (1 << 3);
	}

	if (CAMERA_ACTIVE) {
		pyro_arm |= (1 << 4);
	}

	bool sleep_mode = atomic_load(&thread_safe_should_wakeup);
	
	pyro_arm |= (sleep_mode << 5);
	
    payload.telemetry.pyro_state = pyro_arm;
	payload.telemetry.sats = sats;
    payload.telemetry.flight_state = flight_state;
	float voltage = psu_read_battery_voltage();
	payload.telemetry.battery_voltage = (uint16_t)(voltage * 2500);

    return payload;
}

goober_t gooberParse(uint8_t *rx_buffer, uint8_t rx_buffer_size) {
	goober_t receivedPacket;

	receivedPacket.DEV_ID = rx_buffer[0];
	receivedPacket.DEV_MODE = rx_buffer[1];
	receivedPacket.SEQ_ID = rx_buffer[2];
	receivedPacket.MSG_CLS = rx_buffer[3];
	receivedPacket.PAYLOAD_SIZE = rx_buffer[4];

	// Bounds checking to prevent buffer overrun
	uint8_t max_payload_size = sizeof(receivedPacket.payload.raw);
	uint8_t actual_payload_size = (receivedPacket.PAYLOAD_SIZE > max_payload_size) ? max_payload_size : receivedPacket.PAYLOAD_SIZE;
	
	// Ensure we don't read beyond the available buffer
	uint8_t available_bytes = rx_buffer_size - 5; // 5 bytes for header
	if (actual_payload_size > available_bytes) {
		actual_payload_size = available_bytes;
	}

	memcpy(receivedPacket.payload.raw, &rx_buffer[5], actual_payload_size);

	#ifdef GOOBER_DEBUG
	printf("[GOOBER_PARSE] DEV_ID: 0x%02X\n", receivedPacket.DEV_ID);
	printf("[GOOBER_PARSE] DEV_MODE: 0x%02X\n", receivedPacket.DEV_MODE);
	printf("[GOOBER_PARSE] SEQ_ID: 0x%02X\n", receivedPacket.SEQ_ID);
	printf("[GOOBER_PARSE] MSG_CLS: 0x%02X\n", receivedPacket.MSG_CLS);
	printf("[GOOBER_PARSE] PAYLOAD_SIZE: %u (actual: %u)\n", receivedPacket.PAYLOAD_SIZE, actual_payload_size);
	printf("[GOOBER_PARSE] Raw payload: ");
	for (int i = 0; i < actual_payload_size; ++i) {
		printf("%02X ", receivedPacket.payload.raw[i]);
	}
	printf("\n\n");
	#endif

  	return receivedPacket;
}

void gooberSerialize(goober_t *packet, uint8_t *tx_buffer, uint8_t tx_buffer_size) {
    if (packet == NULL || tx_buffer == NULL) return;

    uint8_t serialized_size = 5 + packet->PAYLOAD_SIZE;
    if (tx_buffer_size < serialized_size) return; // not enough space in provided buffer

    tx_buffer[0] = packet->DEV_ID;
    tx_buffer[1] = packet->DEV_MODE;
    tx_buffer[2] = packet->SEQ_ID;
    tx_buffer[3] = packet->MSG_CLS;
    tx_buffer[4] = packet->PAYLOAD_SIZE;

    if (packet->PAYLOAD_SIZE > 0) {
        memcpy(&tx_buffer[5], packet->payload.raw, packet->PAYLOAD_SIZE);
    }
}

bool is_tx_lock() {
	return atomic_load(&thread_safe_txlock);
}

bool should_wake_up() {
	return atomic_load(&thread_safe_should_wakeup);
}

void activate_txlock() {
	atomic_store(&thread_safe_txlock, true);
}
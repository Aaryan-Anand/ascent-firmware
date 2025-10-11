#include "goober.h"
#include "driver_pyro.h"
#include "stdatomic.h"
#include "string.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "sensor_manager.h"


#define TELEM_PACKET_SIZE 51

static bool CAMERA_ACTIVE = false;

static bool TXLOCK = false;

_Atomic bool thread_safe_txlock = false;
_Atomic bool thread_safe_should_wakeup = true;
goober_t recv_packet;

#define APPO PYRO_CHANNEL_1
#define MAINS PYRO_CHANNEL_2

void turn_on_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,0,1); 
    CAMERA_ACTIVE = true;
}

void turn_off_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,1,1); 
    CAMERA_ACTIVE = false;
}

// DO NOT REMOVE, required to allow flight with only one way coms - Luke
void fake_tx_lock(void) {
	TXLOCK = flash_prepare_for_flight();
	bmp_aquire_ground();
	flash_erase_jingle();
	atomic_store(&thread_safe_txlock, true);
	printf("Faked tx lock ready to fly.\n");
}

goober_t create_packet(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload) {
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

goober_t decode_packet(uint8_t *rx_buffer, uint8_t rx_buffer_size) {

	recv_packet.DEV_ID = rx_buffer[0];
	recv_packet.DEV_MODE = rx_buffer[1];
	recv_packet.SEQ_ID = rx_buffer[2];
	recv_packet.MSG_CLS = rx_buffer[3];
	recv_packet.PAYLOAD_SIZE = rx_buffer[4];

	memcpy(recv_packet.payload.raw, &rx_buffer[5], recv_packet.PAYLOAD_SIZE);
  return recv_packet;
}

goober_t create_response_packet(goober_msg_type_t request_msg_type, goober_payload_t telemetry) {
	goober_msg_type_t request_msg_type = recv_packet.MSG_CLS;
	goober_payload_t request_payload;
	memcpy(request_payload.raw, recv_packet.payload.raw, recv_packet.PAYLOAD_SIZE);

	goober_t resp;
	goober_msg_type_t resp_msg_cls =0;
	goober_payload_t resp_msg_payload;
	uint8_t resp_msg_payload_len = 0;

	switch (request_msg_type) {
		case MSG_TYPE_REQ_TELEM: {
			#ifdef LORA_DEBUG
			printf("Received REQ_TELEM\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_ACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_AUX_ACTIVATE\n");
			#endif
			turn_on_cameras();
			turn_on_fan();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_DEACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_AUX_DEACTIVATE\n");
			#endif
			turn_off_cameras();
			turn_off_fan();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_TXLOCK_ACTIVATE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_TXLOCK_ACTIVATE\n");
			#endif
			TXLOCK = flash_prepare_for_flight();
			bmp_aquire_ground();
			flash_erase_jingle();
			resp_msg_cls = POST_TXLOCK_ACTIVATE;
			resp_msg_payload.single_byte.single_byte_payload = 0x79;
			resp_msg_payload_len = 1;
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
			resp_msg_payload.single_byte.single_byte_payload = 0x01;
			resp_msg_payload_len = 1;
			break;
		}
		case MSG_TYPE_REQ_POP_APOGEE: {
			#ifdef LORA_DEBUG
			printf("Received REQ_POP_APOGEE\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			deploy(APPO);
			break;
		}
		case MSG_TYPE_REQ_POP_MAINS: {
			#ifdef LORA_DEBUG
			printf("Received REQ_POP_MAINS\n");
			#endif
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			deploy(MAINS);
			break;
		}
		case MSG_TYPE_REQ_WAKEUP: {
			printf("Received REQ_WAKEUP\n");
			resp_msg_cls = MSG_TYPE_POST_PINGPONG;
			resp_msg_payload.single_byte.single_byte_payload = 0x12;
			resp_msg_payload_len = 1;

			vTaskDelay(5000 / portTICK_PERIOD_MS); 

			//WAKEUP LOGIC GOES HERE @worldwalker2000
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
}

goober_payload_t create_telemetry_payload(int32_t latitude, int32_t longitude, float altitude_agl, float vertical_velocity, float x_acc, float eul_x, float eul_y, float eul_z, float gyr_x, uint8_t sats, uint8_t flight_state) {
	goober_payload_t payload;

	#ifdef LORA_DEBUG
	// printf("Creating telemetry payload\n");
	// printf("timestamp: %lld\n", payload.telemetry.timestamp);
	#endif

    payload.telemetry.timestamp = esp_timer_get_time();
	#ifdef LORA_DEBUG
	// printf("timestamp: %lld\n", payload.telemetry.timestamp);
	#endif
    payload.telemetry.latitude = latitude;
    payload.telemetry.longitude = longitude;
    payload.telemetry.altitude_agl = altitude_agl;
    payload.telemetry.vertical_velocity = vertical_velocity;
    payload.telemetry.x_acc = x_acc;
    payload.telemetry.eul_x = eul_x;
    payload.telemetry.eul_y = eul_y;
    payload.telemetry.eul_z = eul_z;
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
	payload.telemetry.battery_voltage = (uint32_t)(psu_read_battery_voltage() * 100000);

  return payload;
}

bool is_tx_lock() {
	return atomic_load(&thread_safe_txlock);
}

bool should_wake_up() {
	return atomic_load(&thread_safe_should_wakeup);
}

//this is just here for now :p

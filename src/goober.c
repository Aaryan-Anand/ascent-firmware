#include "goober.h"

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

goober_t decode_packet(uint8_t *rx_buffer, uint8_t rx_buffer_size, goober_payload_t telemetry) {
	goober_t recv_packet;

	recv_packet.DEV_ID = rx_buffer[0];
	recv_packet.DEV_MODE = rx_buffer[1];
	recv_packet.SEQ_ID = rx_buffer[2];
	recv_packet.MSG_CLS = rx_buffer[3];
	recv_packet.PAYLOAD_SIZE = rx_buffer[4];

	memcpy(recv_packet.payload.raw, &rx_buffer[5], recv_packet.PAYLOAD_SIZE);
  return recv_packet;
}

goober_t create_response_packet(goober_msg_type_t request_msg_type) {}

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

//this is just here for now :p

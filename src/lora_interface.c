#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "stdlib.h"
#include "stdint.h"
#include "globals.h"
#include "string.h"
#include "driver_psu.h"
#include "driver_pyro.h"
#include "esp_random.h"
#include "esp_random.h"
#include "lora_interface.h"
#include "flash_interface.h"
#include "driver_buzzer.h"
#include "stdatomic.h"
#include "sensor_manager.h"

#define LORA_DEBUG
#define SLAVE_DEV_ID 0x41
#define TELEM_PACKET_SIZE 51
#define LOCATOR_PACKET_SIZE 16

static bool TXLOCK = false;
static bool CAMERA_ACTIVE = false;

_Atomic bool thread_safe_txlock = false;

QueueHandle_t lora_packet_queue;

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

void turn_on_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,0,1); 
    CAMERA_ACTIVE = true;
}

void turn_off_cameras(void) {
    pyro_activate(PYRO_CHANNEL_3,1,1); 
    CAMERA_ACTIVE = false;
}

void turn_on_fan(void) {
    pyro_activate(PYRO_CHANNEL_4,0,1); 
}

void lora_flight_init()
{
 	lora_init();
	
	lora_set_frequency(LORA_FREQ); // 915MHz
	lora_enable_crc();

	int cr = 1;
	int bw = 9;
	int sf = 7;

	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	printf("coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	printf("bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	printf("spreading_factor=%d", sf);

	lora_set_tx_power(17);

	lora_packet_queue = xQueueCreate(1, 51);
	assert(lora_packet_queue != NULL);
}

void lora_queue_packet(goober_payload_t *payload) {
	xQueueOverwrite(lora_packet_queue, payload);
}

void lora_read_latest_queue_packet(goober_payload_t *payload) {
	xQueueReceive(lora_packet_queue, payload, portMAX_DELAY);
}

goober_payload_t create_telemetry_payload(int32_t latitude, int32_t longitude, float altitude_agl, float vertical_velocity, float x_acc, float eul_x, float eul_y, float eul_z, float gyr_x, uint8_t sats, uint8_t flight_state)
{
	goober_payload_t payload;

	// printf("Creating telemetry payload\n");

    payload.telemetry.timestamp = esp_timer_get_time();
	// printf("timestamp: %lld\n", payload.telemetry.timestamp);
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
	
    payload.telemetry.pyro_state = pyro_arm;
	payload.telemetry.sats = sats;
    payload.telemetry.flight_state = flight_state;
	payload.telemetry.battery_voltage = (float)psu_read_battery_voltage();
	printf("Battery voltage: %f\n", payload.telemetry.battery_voltage);

    return payload;
}

goober_t lora_create_packet(uint8_t dev_id, bool is_master, bool tx_intent, bool tx_lock, goober_msg_type_t message_class, uint8_t payload_size, goober_payload_t *payload)
{
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

void lora_transmit_packet(goober_t *packet)
{

    size_t len = 5 + packet->PAYLOAD_SIZE; //5 bytes + payload (n bytes)
    uint8_t *packet_data = malloc(len);
    if (packet_data == NULL) return; //incase of fail

    packet_data[0] = packet->DEV_ID;
    packet_data[1] = packet->DEV_MODE;
    packet_data[2] = packet->SEQ_ID;
    packet_data[3] = packet->MSG_CLS;
    packet_data[4] = packet->PAYLOAD_SIZE;
    memcpy(&packet_data[5], packet->payload.raw, packet->PAYLOAD_SIZE);

    lora_send_packet(packet_data, len);
    free(packet_data);

    int lost = lora_packet_lost();
    if (lost != 0) {
        printf("%d packets lost\n", lost);
    }
}

void lora_process(uint8_t *rx_buffer, uint8_t rx_buffer_size, goober_payload_t telemetry) {
	goober_t recv_packet;

	recv_packet.DEV_ID = rx_buffer[0];
	recv_packet.DEV_MODE = rx_buffer[1];
	recv_packet.SEQ_ID = rx_buffer[2];
	recv_packet.MSG_CLS = rx_buffer[3];
	recv_packet.PAYLOAD_SIZE = rx_buffer[4];

	memcpy(recv_packet.payload.raw, &rx_buffer[5], recv_packet.PAYLOAD_SIZE);

	// RESPONSE CREATION STARTS HERE

	goober_msg_type_t request_msg_type = recv_packet.MSG_CLS;
	goober_payload_t request_payload;
	memcpy(request_payload.raw, recv_packet.payload.raw, recv_packet.PAYLOAD_SIZE);

	goober_t resp;
	goober_msg_type_t resp_msg_cls =0;
	goober_payload_t resp_msg_payload;
	uint8_t resp_msg_payload_len = 0;

	switch (request_msg_type) {
		case MSG_TYPE_REQ_TELEM: {
			// printf("Received REQ_TELEM\n");
			// printf("Received REQ_TELEM\n");
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_ACTIVATE: {
			// printf("Received REQ_AUX_ACTIVATE\n");
			turn_on_cameras();
			// printf("Received REQ_AUX_ACTIVATE\n");
			turn_on_cameras();
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_AUX_DEACTIVATE: {
			turn_off_cameras();
			// printf("Received REQ_AUX_DEACTIVATE\n");
			turn_off_cameras();
			// printf("Received REQ_AUX_DEACTIVATE\n");
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			break;
		}
		case MSG_TYPE_REQ_TXLOCK_ACTIVATE: {
			// printf("Received REQ_TXLOCK_ACTIVATE\n");
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
			// printf("Received REQ_REBOOT\n");
			// printf("Received REQ_REBOOT\n");
			esp_restart();
			break;
		}
		case MSG_TYPE_REQ_PINGPONG: {
			// printf("Received REQ_PINGPONG\n");
			// printf("Received REQ_PINGPONG\n");
			resp_msg_cls = MSG_TYPE_POST_PINGPONG;
			resp_msg_payload.single_byte.single_byte_payload = 0x01;
			resp_msg_payload_len = 1;
			break;
		}
		case MSG_TYPE_REQ_POP_APOGEE: {
			printf("Received REQ_POP_APOGEE\n");
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			deploy(APPO);
			break;
		}
		case MSG_TYPE_REQ_POP_MAINS: {
			printf("Received REQ_POP_MAINS\n");
			resp_msg_cls = MSG_TYPE_POST_TELEM;
			resp_msg_payload = telemetry;
			resp_msg_payload_len = TELEM_PACKET_SIZE;
			deploy(MAINS);
			break;
		}
		default: {
			printf("Unknown MSG_TYPE: 0x%X\n", request_msg_type);
			break;
		}
	}

	// PACKET SENDING STARTS HERE

	if (resp_msg_payload_len != 0) {
		resp = lora_create_packet(SLAVE_DEV_ID,0,0,0,resp_msg_cls,resp_msg_payload_len, &resp_msg_payload);
		// printf("Created packet with the following parameters: \n");
		// printf("MSG_CLS: 0x%X\n", resp_msg_cls);
		// printf("PAYLOAD_SIZE: 0x%X\n", resp_msg_payload_len);
		// printf("SEQ_ID: 0x%X\n", resp.SEQ_ID);
		// printf("DEV_ID: 0x%X\n", resp.DEV_ID);
		// printf("DEV_MODE: 0x%X\n", resp.DEV_MODE);	
		// printf("Created packet with the following parameters: \n");
		// printf("MSG_CLS: 0x%X\n", resp_msg_cls);
		// printf("PAYLOAD_SIZE: 0x%X\n", resp_msg_payload_len);
		// printf("SEQ_ID: 0x%X\n", resp.SEQ_ID);
		// printf("DEV_ID: 0x%X\n", resp.DEV_ID);
		// printf("DEV_MODE: 0x%X\n", resp.DEV_MODE);	
		
	
	resp.SEQ_ID = recv_packet.SEQ_ID;

	printf("Sending packet: ");
	for(int i = 0; i < (5 + 51); i++) {
		printf("%02X ", ((uint8_t*)&resp)[i]);
	}

	lora_transmit_packet(&resp);
	}
}

void slave_lora_task(goober_payload_t *telemetry)
{
	uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
	TickType_t start_time = xTaskGetTickCount(); // Get the current tick count
    
	if (!TXLOCK) {
		bool waiting = true;

		lora_receive(); // put into receive mode

		while(waiting) {
			if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(6)) { // Fixed timeout
				printf("LORA: Timeout waiting for packet after %ldms\n", (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS);
				waiting = false; // Exit the loop after timeout
			} else {
				if(lora_received() != 0) {
					waiting = false;
					int rxLen = lora_receive_packet(buf, sizeof(buf));
					printf("Received packet after %ldms: ", (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS);
					for(int i = 0; i < rxLen; i++) {
						printf("%02X ", buf[i]);
					}
					printf(" ");
					TickType_t process_start_time = xTaskGetTickCount();
					lora_process(buf, rxLen, *telemetry); // 45 ms max
					printf("Took %ldms to process packet\n", (xTaskGetTickCount() - process_start_time) * portTICK_PERIOD_MS);
				} else {
					vTaskDelay(1);
				}
			}
		}

	}
	else if (TXLOCK) {
		goober_t TXLockPacket = lora_create_packet(SLAVE_DEV_ID,0,0,1,MSG_TYPE_POST_TELEM,TELEM_PACKET_SIZE, telemetry);
		lora_transmit_packet(&TXLockPacket);
	}	
}

bool is_tx_lock() {
	return atomic_load(&thread_safe_txlock);
}
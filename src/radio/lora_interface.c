#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "stdlib.h"
#include "stdint.h"
#include "globals.h"
#include "string.h"
#include "lora_interface.h"
#include "flash_interface.h"
#include "stdatomic.h"
#include "driver_pyro.h"

#include "goober.h"

// #define LORA_DEBUG

// Telemetry payload queue -- Always have the latest telemetry payload on hand

QueueHandle_t telemetryPayloadQueue;

void queueLatestTelemetryPayload(goober_payload_t *payload) {
	xQueueOverwrite(telemetryPayloadQueue, payload);
}

void peekLatestTelemetryPayload(goober_payload_t *payload) {
	xQueuePeek(telemetryPayloadQueue, payload, portMAX_DELAY);
}

// LoRa Functionality

esp_err_t lora_flight_init()
{	
	telemetryPayloadQueue = xQueueCreate(1, 41);

 	lora_init();
    if (lora_init() != 1) {
        return ESP_FAIL;
    }

	lora_set_frequency(LORA_FREQ);

	lora_enable_crc();

	int cr = 1;
	int bw = 9;
	int sf = 7;

	lora_set_coding_rate(cr);

	lora_set_bandwidth(bw);

	lora_set_spreading_factor(sf);

    lora_set_tx_power(17);

    printf("\n\n==============================================\n\n");
    printf("Successfully initialized LoRa with the following parameters:\n\n");
    printf("* frequency = %d MHz\n", (int)LORA_FREQ / 1000000);
	printf("* bandwidth = %d\n", bw);
	printf("* coding_rate = %d\n", cr);
	printf("* spreading_factor = %d\n", sf);
    printf("* tx_power = %d\n", 17);
    printf("\n==============================================\n\n");

	return ESP_OK;
}

void lora_transmit_packet(goober_t *packet)
{	
	if (packet == NULL) return; // don't break lol
	
	uint8_t serialized_buffer_length = 5 + packet->PAYLOAD_SIZE;
	uint8_t *serialized_packet = malloc(serialized_buffer_length);

	gooberSerialize(packet, serialized_packet, serialized_buffer_length);

    lora_send_packet(serialized_packet, serialized_buffer_length);

    free(serialized_packet);

    int lost = lora_packet_lost();
    #ifdef LORA_DEBUG
    if (lost != 0) {
        printf("%d packets lost\n", lost);
    }
    #endif
}

int lora_blocking_listen(goober_t *packet, uint8_t timeout){
	if (packet == NULL) return 0; // don't screw up

	int received = 0;
	uint8_t lora_rx_buf[255];
	int rxLen = 0;

	uint64_t start = esp_timer_get_time();
	uint64_t now = start;
	uint64_t end  = start + ((uint64_t)timeout * 1000); // ms to us

	while ((now < end) && received == 0) {
		now = esp_timer_get_time();
		lora_receive();
		if (lora_received()) {
			rxLen = lora_receive_packet(lora_rx_buf, sizeof(lora_rx_buf));
			if (rxLen > 5) { // this could be substited for better logic to avoid wasting time on non-goober packets - abdul
				received = 1;
			}
		}
		vTaskDelay(1);
	}

	if (received) {
		*packet = gooberParse(lora_rx_buf, rxLen);
	}

	return received;
}

uint8_t calc_pyro_arm(void) {
    uint8_t pyro_arm = 0;
    if (pyro_continuity(PYRO_CHANNEL_1)) pyro_arm |= (1);
    if (pyro_continuity(PYRO_CHANNEL_2)) pyro_arm |= (1 << 1);
    if (pyro_continuity(PYRO_CHANNEL_3)) pyro_arm |= (1 << 2);
    if (pyro_continuity(PYRO_CHANNEL_4)) pyro_arm |= (1 << 3);

    return pyro_arm;
}
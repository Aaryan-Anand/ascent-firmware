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
#include "nvs_interface.h"

#include "goober.h"

// Telemetry payload queue -- Always have the latest telemetry payload on hand

static lora_config_t lora_config = {
    
};

void lora_config_init(void) {
    lora_config_t cfg;
    if (nvs_retreive_lora_config(&cfg) != ESP_OK) {
        cfg = (lora_config_t){
            .frequency = 915e6,
			.bandwidth = 9,
			.coding_rate = 1,
			.spreading_factor = 7,
			.tx_power = 17,
			.TDD = 0,
        };
        nvs_set_lora_config(&cfg);
		printf("lora_config not found, setting default\n");
    }
    else {
        printf("lora_config found, setting up LoRa\n");
    }

	lora_set_frequency(cfg.frequency);
	lora_enable_crc();
	lora_set_coding_rate(cfg.coding_rate);
	lora_set_bandwidth(cfg.bandwidth);
	lora_set_spreading_factor(cfg.spreading_factor);
    lora_set_tx_power(cfg.tx_power);

	printf("\n\n==============================================\n\n");
    printf("Successfully initialized LoRa with the following parameters:\n\n");
    printf("* frequency = %d MHz\n", (int)cfg.frequency / 1000000);
	printf("* bandwidth = %d\n", cfg.bandwidth);
	printf("* coding_rate = %d\n", cfg.coding_rate);
	printf("* spreading_factor = %d\n", cfg.spreading_factor);
    printf("* tx_power = %d\n", cfg.tx_power);
    printf("\n==============================================\n\n");
}

void lora_config_set(lora_config_t *cfg) {
    if(cfg->frequency != lora_config.frequency) {
        lora_set_frequency(cfg->frequency);
    }
    if(cfg->bandwidth != lora_config.bandwidth) {
        lora_set_bandwidth(cfg->bandwidth);
    }
    if(cfg->coding_rate != lora_config.coding_rate) {
        lora_set_coding_rate(cfg->coding_rate);
    }
    if(cfg->spreading_factor != lora_config.spreading_factor) {
        lora_set_spreading_factor(cfg->spreading_factor);
    }
    if(cfg->tx_power != lora_config.tx_power) {
        lora_set_tx_power(cfg->tx_power);
    }
	lora_config = *cfg;
	nvs_set_lora_config(&lora_config);
}

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

	lora_config_init();

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
			#ifdef LORA_DEBUG
			printf("received packet of length %d\n", rxLen);
			for (int i = 0; i < rxLen; i++) {	
				printf("%02X ", lora_rx_buf[i]);
			}
			printf("\n");
			#endif
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
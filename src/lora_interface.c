#include "lora_interface.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "stdlib.h"
#include "stdlib.h"
#include "stdint.h"
#include "globals.h"
#include "string.h"
#include "driver_psu.h"

uint8_t packet_data[sizeof(lora_packet_t)];

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
}

void lora_transmit_packet(lora_packet_t *packet)
{
    packet->timestamp = esp_timer_get_time() / 1e3;

    memcpy(packet_data, packet, sizeof(lora_packet_t));
    lora_send_packet(packet_data, sizeof(lora_packet_t));

    // printf("Sent packet at %ld ms: Latitude: %.6f, Longitude: %.6f, GPSAltitude: %ld, Baro Altitude: %f, Baro Velocity: %f, Acceleration: %f, Pyro Arm: %d, Flight State: %d, Battery Voltage: %f\n", packet.timestamp, packet.latitude, packet.longitude, packet.gps_altitude, packet.barometric_agl, packet.barometric_velocity, packet.acceleration, packet.pyro_arm, packet.flight_state, packet.batt_voltage);

    int lost = lora_packet_lost();
    if (lost != 0) {
        printf("%d packets lost", lost);
    }
}
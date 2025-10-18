#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "flash_interface.h"
#include "goober.h"

void validate_esp32(void);
void init_boot_sequence(void);

void update_loop_rate(void);
void low_power_mode_no_gps(void);
void high_power_mode(void);

void beep_pyro_cont(void);

void read_uuid(void);

void primary_preflight(uint32_t *cycle, GPS_data_t *gps_data);
void primary_flight(uint32_t *cycle, GPS_data_t *gps_data, uint8_t *flight_state);
void primary_landed(uint32_t *cycle, GPS_data_t *gps_data, uint8_t *flight_state, bool *flash_erase_next_bank_on_landed_finished, int32_t *resume);
void secondary_flight(uint32_t *cycle, goober_t *rcv_packet, int *rcv, goober_t *rsp_packet, goober_t *txlock_packet);
#endif
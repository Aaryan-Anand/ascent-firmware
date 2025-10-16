#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "flash_interface.h"

void validate_esp32(void);
void init_boot_sequence(void);

void fail_if_barometer_bad(void);

void update_loop_rate(void);
void low_power_mode_no_gps(void);
void high_power_mode(void);

void beep_pyro_cont(void);
uint8_t calc_pyro_arm(void);

void try_to_dump_data();
void print_flash_packet(flash_packet *fp);

#endif
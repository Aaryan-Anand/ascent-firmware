#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "flash_interface.h"

void validate_esp32(void);
void init_boot_sequence(void);

void update_loop_rate(void);
void low_power_mode_no_gps(void);
void high_power_mode(void);

void beep_pyro_cont(void);

#endif
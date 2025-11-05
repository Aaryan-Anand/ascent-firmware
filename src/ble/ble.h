#ifndef BLE_H
#define BLE_H

#include "goober.h"

void ble_init(void);
void ble_stop(void);
goober_t get_goober_packet(goober_t request_packet);

#endif
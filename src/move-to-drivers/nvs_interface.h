#ifndef NVS_INTERFACE
#define NVS_INTERFACE

#include "nvs_flash.h"
#include "nvs.h"

void nvs_interface_init(void);

nvs_handle_t nvs_interface_get_handle(void);

#endif
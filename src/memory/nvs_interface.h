#ifndef NVS_INTERFACE
#define NVS_INTERFACE

#include "nvs_flash.h"
#include "nvs.h"

void nvs_interface_init(void);

nvs_handle_t nvs_interface_get_handle(void);

void nvs_retreive_matrices(float (*acc_correction_matrix)[3], float (*gyr_correction_matrix)[3], float (*mag_correction_matrix)[3], float (*high_g_correction_matrix)[3], float acc_bias_vector[3], float gyr_bias_vector[3], float mag_bias_vector[3], float high_g_bias_vector[3]);
void nvs_retreive_uuid(uint8_t *uuid);
#endif
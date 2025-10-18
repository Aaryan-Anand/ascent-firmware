#ifndef MAIN_H
#define MAIN_H
//MARK: - COPY TO MAIN.H
//Update the UUID and the matricies to your needs
#include "nvs_interface.h"
#include "esp_err.h"
#include "esp_system.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "flash_interface.h"
#include "beep.h"
#include "driver_buzzer.h"
#include "ascent_r2_hardware_definition.h"
#include "spi_manager.h"

//0x4, 0x1, 0x7, 0x3, 0x6, 0xe, 0x7, 0x4, 0x5, 0x2 hex to asci is AsntR followed by it's uuid

uint8_t uuid[16] = { 0x4, 0x1, 0x7, 0x3, 0x6, 0xe, 0x7, 0x4, 0x5, 0x2, 0x2, 0x0, 0x3, 0x0, 0x0, 0x0 };

static float acc_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float gyr_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float mag_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
static float high_g_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

// Bias vectors initialized to zero
static float acc_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float gyr_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float high_g_bias_vector[3] = {0.0f, 0.0f, 0.0f};
#endif
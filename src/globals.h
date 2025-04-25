#ifndef GLOBALS_H
#define GLOBALS_H

#include "sensor_manager.h"
#include "stdint.h"

// Task counters
extern volatile uint32_t sensor_task_counter;
extern volatile uint32_t fsm_task_counter;

// Global variables
extern uint32_t timestamp;
extern float barometric_agl;
extern float barometric_velocity;
extern float average_barometric_velocity;
extern double acceleration;
extern uint8_t pyro_arm;
extern uint8_t flight_state;
extern uint8_t flight_event;
extern double batt_voltage;

// GPS Data
extern int32_t pLatitudeX1e7;
extern int32_t pLongitudeX1e7;
extern int32_t pAltitudeMillimetres;
extern int32_t pRadiusMillimetres;
extern int32_t pAltitudeUncertaintyMillimetres;
extern int32_t pSpeedMillimetresPerSecond;
extern int32_t pSvs;
extern int64_t pTimeUtc;

// Sensor data
extern imu_raw_3d_t acc, gyr, mag;
extern imu_float_3d_t high_g_acc;
extern float bmp_agl;

// Correction matrices (3x3)
extern float acc_correction_matrix[3][3];
extern float gyr_correction_matrix[3][3];
extern float mag_correction_matrix[3][3];
extern float high_g_correction_matrix[3][3];

// Bias vectors (3x1)
extern float acc_bias_vector[3];
extern float gyr_bias_vector[3];
extern float mag_bias_vector[3];
extern float high_g_bias_vector[3];

// Add these declarations to globals.h
extern float bmp_scaling;
extern float bmp_bias;

#endif
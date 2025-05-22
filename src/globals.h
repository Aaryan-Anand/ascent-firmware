#ifndef GLOBALS_H
#define GLOBALS_H

#include "sensor_manager.h"
#include "stdint.h"

// Task counters
extern volatile uint32_t sensor_task_counter;
extern volatile uint32_t fsm_task_counter;

// Global variables
extern uint32_t timestamp;

extern uint8_t pyro_arm;
extern uint8_t flight_state;
extern uint8_t flight_event;

extern double batt_voltage;

// Sensor data
extern imu_raw_3d_t acc, gyr, mag;
extern imu_float_3d_t high_g_acc;
extern baro_double_t baro;

// gps data
extern uint32_t UTCtstamp;
extern int32_t lon;
extern int32_t lat;
extern int32_t gps_altitude;
extern int32_t hMSL;
extern uint8_t fixType;
extern uint8_t numSV;

extern float ekf_latitude;
extern float ekf_longitude;
extern float ekf_altitude;
extern float ekf_pitch;
extern float ekf_yaw;
extern float ekf_roll;

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

extern float barometric_agl;
extern float barometric_velocity;
extern float average_barometric_velocity;


#endif
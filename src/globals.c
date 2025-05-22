#include "globals.h"

uint32_t timestamp;
float latitude;
float longitude;
float barometric_agl;
uint32_t gps_altitude;
float barometric_velocity;
float average_barometric_velocity;
double acceleration;
uint8_t pyro_arm = 0;
uint8_t flight_state;
double batt_voltage = 99.99;

imu_raw_3d_t acc, gyr, mag;
imu_float_3d_t high_g_acc;
baro_double_t baro;

float ekf_latitude;
float ekf_longitude;
float ekf_altitude;
float ekf_pitch;
float ekf_yaw;
float ekf_roll;
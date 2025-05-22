#include "baro.h"

#include "globals.h"
#include "interface_bmp390l.h"

#define HISTORY_SIZE 3
#define VELOCITY_HISTORY_SIZE 10
#define DT 0.01f

static float barometric_agl;
static float barometric_velocity;
static float average_barometric_velocity;

void baro_update(float *agl, float *vel, float *avg_vel)
{
    static float agl_history[HISTORY_SIZE] = {0};  // Store the last 5 AGL readings
    // double pressure_hPa;
    // double temperature;

    static float velocity_samples[VELOCITY_HISTORY_SIZE] = {0};
    static int sample_index = 0;

    // Shift history
    for (int i = HISTORY_SIZE - 1; i > 0; i--) {
        agl_history[i] = agl_history[i - 1];
    }

    // Update with latest AGL
    baro_double_t out;
    bmp390_get_local(&out);
    agl_history[0] = out.alt;
    barometric_agl = agl_history[0];
    // bmp390_read_sensor_data(&pressure_hPa, &temperature);

    // Compute first-order backward finite difference
    if (agl_history[1] != 0) {
        barometric_velocity = barometric_velocity*0.2 + ((agl_history[0] - agl_history[1]) / DT)*0.8;
    }

    // Update velocity samples
    velocity_samples[sample_index] = barometric_velocity;
    sample_index = (sample_index + 1) % VELOCITY_HISTORY_SIZE;

    float sum = 0;
    // Calculate the sum of all samples
    for (int i = 0; i < 10; i++) {
        sum += velocity_samples[i];
    }

    // Calculate the average velocity
    average_barometric_velocity = sum / 10.0;

    // printf("AGL: %f, Pressure: %f, Velocity: %f\n", agl_history[0], pressure_hPa, barometric_velocity);
}
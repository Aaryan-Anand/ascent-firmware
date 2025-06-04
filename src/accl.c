#include "accl.h"

#include "stdbool.h"

static const float alpha = 0.1;
static imu_raw_3d_t acc;
static bool initialized = false;

void accl_update(imu_raw_3d_t new, imu_raw_3d_t* out) {
    if (!initialized) {
        acc = new;
        initialized = true;
    }

    acc.x = (int16_t)((float) acc.x*alpha + (float) new.x*(1-alpha));
    acc.y = (int16_t)((float) acc.y*alpha + (float) new.y*(1-alpha));
    acc.z = (int16_t)((float) acc.z*alpha + (float) new.z*(1-alpha));

    *out = acc;
}
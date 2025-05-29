#include "flight.h"

#include "stdint.h"
#include "math.h"

#include "flight_config.h"
#include "driver_pyro.h"

static uint8_t flight_state = FS_ON_PAD;

#define APPO PYRO_CHANNEL_1
#define MAINS PYRO_CHANNEL_2

void turn_off_cameras(void);

static bool deploy(pyro_channel_t channel)
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(channel);
        if (cont) {
            pyro_activate(channel, 150*(i+1), 0);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(channel);
            if (!cont) return true;
        }
    }

    return false;
}

void flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float xacc
) {
    static int count = 0;

    switch (flight_state) {
        case FS_ON_PAD:
            if (xacc > 3000) {
                count++;
            } else {
                count = 0;
            }

            if (count >= 5) {
                flight_state = IS_TWO_STAGE ? FS_BOOSTER : FS_SUSTAINER;
                count = 0;
            }
            break;

        case FS_BOOSTER:
            if (xacc < 0) {
                flight_state = FS_COAST_BOOSTER;
            }
            break;

        case FS_COAST_BOOSTER:
            if (xacc > 3000) {
                count++;
            } else if (barometric_agl > APOGEE_MIN && average_barometric_velocity < 0 && fabs(xacc) < 100) {
                deploy(APPO);
                flight_state = FS_UNDER_DROGUES;
            } else {
                count = 0;
            }
            

            if (count >= 5) {
                flight_state = FS_SUSTAINER;
                count = 0;
            }
            break;

        case FS_SUSTAINER:
            if (xacc < 0) {
                flight_state = FS_COAST_SUSTAINER;
            }
            break;

        case FS_COAST_SUSTAINER:
            if (barometric_agl > APOGEE_MIN && average_barometric_velocity < 0 && fabs(xacc) < 100) {
                deploy(APPO);
                flight_state = FS_UNDER_DROGUES;
            }
            break;

        case FS_UNDER_DROGUES:
            if (average_barometric_velocity < PANIC_VEL) {
                count++;
            } else {
                count = 0;
            }

            if (barometric_agl < MAINS_ALT || count >= 2) {
                deploy(MAINS);
                flight_state = FS_UNDER_MAINS;
            }
            break;

        case FS_UNDER_MAINS:
            if (fabs(average_barometric_velocity) < 5) {
                count++;
            } else {
                count = 0;
            }

            if (count >= 5) {
                flight_state = FS_LANDED;
            }
            break;

        case FS_LANDED:
            turn_off_cameras();
            break;

        default: break;
    }
}

uint8_t get_flight_state(void) {
    return flight_state;
}
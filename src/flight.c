#include "flight.h"

#include "stdint.h"
#include "math.h"

#include "flight_config.h"
#include "driver_pyro.h"
#include "lora_interface.h"

// static uint8_t flight_state = FS_PREFLIGHT;
static uint8_t flight_state = FS_ON_PAD;

#define APPO PYRO_CHANNEL_1
#define MAINS PYRO_CHANNEL_2

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

// VS code may say that this is an error bc it can't see APPO_GS but it will compile
#define APPO_COND (average_barometric_velocity < 0 && fabs(xacc) < APPO_GS)

bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float xacc
) {
    // this function will be called at 50 Hz durring flight, thus every tick is 20 ms

    static int count1 = 0;
    static int count2 = 0;

    uint8_t pre = flight_state;

    switch (flight_state) {
        case FS_PREFLIGHT:
            if (should_wake_up()) {
                flight_state = FS_ON_PAD;
            }
            break;

        case FS_ON_PAD:
            if (xacc > ENGINE_GS) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = IS_TWO_STAGE ? FS_BOOSTER : FS_SUSTAINER;
            } else if (!should_wake_up() && count1 == 0) {
                flight_state = FS_PREFLIGHT;
            }
            break;

        case FS_BOOSTER:
            if (xacc < 0) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = FS_COAST_BOOSTER;
            }
            break;

        case FS_COAST_BOOSTER:
            if (xacc > ENGINE_GS) {
                count1++;
            } else {
                count1 = 0;
            }

            if (APPO_COND) {
                count2++;
            } else {
                count2 = 0;
            }

            if (count2 >= 5) {
                deploy(APPO);
                flight_state = FS_UNDER_DROGUES;
            } else if (count1 >= 5) {
                flight_state = FS_SUSTAINER;
            }
            break;

        case FS_SUSTAINER:
            if (xacc < 0) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = FS_COAST_SUSTAINER;
            }
            break;

        case FS_COAST_SUSTAINER:
            if (APPO_COND) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                deploy(APPO);
                flight_state = FS_UNDER_DROGUES;
            }
            break;

        case FS_UNDER_DROGUES:
            if (average_barometric_velocity < PANIC_VEL) {
                count1++;
            } else {
                count1 = 0;
            }

            if (barometric_agl < MAINS_ALT) {
                count2++;
            } else {
                count2 = 0;
            }

            if (count1 >= 50 || count2 >= 5) {
                deploy(MAINS);
                flight_state = FS_UNDER_MAINS;
            }
            break;

        case FS_UNDER_MAINS:
            if (barometric_agl < 50 && fabs(average_barometric_velocity) < 2) {
                count1++;
            } else {
                count1 = 0;
            }

            // for 6 seconds
            if (count1 >= 300) {
                flight_state = FS_LANDED;
            }
            break;

        case FS_LANDED:
            break;

        default: break;
    }

    // reset the counters if we changed flight states
    // the next state needs to have the counters at zero
    if (flight_state != pre) {
        count1 = 0;
        count2 = 0;

        printf("Changing flight state, now: %s\n", get_flight_state_name());
    }

    return flight_state != pre;
}

uint8_t get_flight_state(void) {
    return flight_state;
}

const char* get_flight_state_name(void) {
    switch (flight_state) {
    case FS_ON_PAD: return "FS_ON_PAD";
    case FS_BOOSTER: return "FS_BOOSTER";
    case FS_COAST_BOOSTER: return "FS_COAST_BOOSTER";
    case FS_SUSTAINER: return "FS_SUSTAINER";
    case FS_COAST_SUSTAINER: return "FS_COAST_SUSTAINER";
    case FS_UNDER_DROGUES: return "FS_UNDER_DROGUES";
    case FS_UNDER_MAINS: return "FS_UNDER_MAINS";
    case FS_LANDED: return "FS_LANDED";
    case FS_PREFLIGHT: return "FS_PREFLIGHT";
    }
    return "UNKNOWN";
}

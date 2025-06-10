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
#define APO_COND (average_barometric_velocity < 0 && fabs(xacc) < APPO_GS)

bool flight_update(
    float barometric_agl,
    float barometric_velocity,
    float average_barometric_velocity,
    float xacc
) {
    static int count1 = 0;
    static int count2 = 0;

    uint8_t pre = flight_state;

    switch (flight_state) {
        case FS_PREFLIGHT:
            if (should_wake_up()) {
                flight_state = FE_WAKE;
            }
            break;

        case FE_WAKE:
            flight_state = FS_ON_PAD;
            break;

        case FS_ON_PAD:
            if (xacc > ENGINE_GS) {
                count1++;
            } else {
                count1 = 0;
            }

            if (!should_wake_up()) {
                flight_state = FE_SLEEP;
            } else if (count1 >= 5) {
                flight_state = FE_LIFTOFF;
            }
            break;
        case FE_SLEEP:
            flight_state = FS_ON_PAD;
            break;
        case FE_LIFTOFF:
            flight_state = IS_TWO_STAGE ? FS_BOOSTER : FS_SUSTAINER;
            break;

        case FS_BOOSTER:
            if (xacc < 0) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = FE_BURNOUT_BOOSTER;
            }
            break;

        case FE_BURNOUT_BOOSTER:
            if (xacc > ENGINE_GS) {
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

            if (APO_COND) {
                count2++;
            } else {
                count2 = 0;
            }

            if (count2 >= 5) {
                deploy(APPO);
                flight_state = FE_APOGEE;
            } else if (count1 >= 5) {
                flight_state = FE_STAGE_SEPARATION;
            }
            break;
        case FE_STAGE_SEPARATION:
            if (xacc > ENGINE_GS) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = FS_SUSTAINER;
            }
            break;

        case FS_SUSTAINER:
            if (xacc < 0) {
                flight_state = FE_BURNOUT_SUSTAINER;
            }
            break;

        case FE_BURNOUT_SUSTAINER:
            count1++;
            if (count1 >= 3 && xacc < 0) {
                flight_state = FS_COAST_SUSTAINER;
            }
            else if (xacc > 0) {
                flight_state = FS_SUSTAINER;
            }
            break;

        case FS_COAST_SUSTAINER:
            if (APO_COND) {
                flight_state = FE_APOGEE;
            }
            break;

        case FE_APOGEE:
            count1++;
            if (APO_COND && count1 >= 5) {
                deploy(APPO);
                flight_state = FE_DROGUES_DEPLOYED;
            }
            else if (count1 >= 150 && barometric_velocity < 0){
                flight_state = FE_DROGUES_DEPLOYED;
                deploy(APPO);
            }
            break;
            
            

        case FE_DROGUES_DEPLOYED:
            //waiting for charge exhaust to drain from components through depressurization
            count1++;
            if(count1 >= 5){
                if (average_barometric_velocity > DROGUE_DESCENT) {
                    count2++;
                }
                else if (average_barometric_velocity < PANIC_VEL) {
                    count1++;
                } else {
                    count1 = 0;
                }

                // at 50 Hz dt = 0.02 thus 3 seconds is 150 counts as 3/0.02=150
                // we wait 3 seconds so that the raven which has a 2 second delay
                // has a chance to try and pull the drouges out
                // the normal mains condition must be true for 0.1 seconds
                if (count1 >= 150 || count2 >= 5) {
                    flight_state = FE_PANIC;
                }
                else if (count2 >= 5) {
                    flight_state = FS_UNDER_DROGUES;
                }
                break;


            case FS_UNDER_DROGUES:
                
                if (barometric_agl < MAINS_ALT) {
                    count1++;
                } else {
                    count1 = 0;
                }
                if(count1 >= 5) {
                    flight_state = FE_MAIN_DEPLOYED;
                }

            }    
            break;

        case FE_PANIC:
           //PRAY TO GOD THAT THIS NEVER HAPPENS
            deploy(MAINS);
            flight_state = FE_MAIN_DEPLOYED;
            break;
                        
        case FE_MAIN_DEPLOYED:
            if (average_barometric_velocity > MAIN_DESCENT) {
                count1++;
            } else {
                count1 = 0;
            }

            if (count1 >= 5) {
                flight_state = FS_UNDER_MAINS;
            }
            else{
                break; //IF THIS HAPPENS, YOU'RE FUCKED
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
                flight_state = FE_GROUND_HIT;
            }
            break;
        case FE_GROUND_HIT:
            flight_state = FS_LANDED;
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
    case FE_LIFTOFF: return "FE_LIFTOFF";
    case FS_BOOSTER: return "FS_BOOSTER";
    case FE_BURNOUT_BOOSTER: return "FE_BURNOUT_BOOSTER";
    case FS_COAST_BOOSTER: return "FS_COAST_BOOSTER";
    case FE_STAGE_SEPARATION: return "FE_STAGE_SEPARATION";
    case FS_SUSTAINER: return "FS_SUSTAINER";
    case FE_BURNOUT_SUSTAINER: return "FE_BURNOUT_SUSTAINER";
    case FS_COAST_SUSTAINER: return "FS_COAST_SUSTAINER";
    case FE_APOGEE: return "FE_APOGEE";
    case FE_DROGUES_DEPLOYED: return "FE_DROGUES_DEPLOYED";
    case FS_UNDER_DROGUES: return "FS_UNDER_DROGUES";
    case FE_MAIN_DEPLOYED: return "FE_MAIN_DEPLOYED";
    case FE_PANIC: return "FE_PANIC";
    case FS_UNDER_MAINS: return "FS_UNDER_MAINS";
    case FE_GROUND_HIT: return "FE_GROUND_HIT";
    case FS_LANDED: return "FS_LANDED";
    case FS_PREFLIGHT: return "FS_PREFLIGHT";
    }
    return "UNKNOWN";
}

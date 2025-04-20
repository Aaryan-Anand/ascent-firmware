#include "flightmgr_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver_pyro.h"
#include "stdint.h"
#include "globals.h"
#include "math.h"

static void flight_on_pad()
{
    // TODO: also check for accelerometer spike
    if (barometric_agl > POWERED_ALT) {
        flight_state = FS_POWERED_FLIGHT;
        return;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
}

static void flight_powered_flight()
{
    // TODO: use accelerometer to detect motor burn out and switch to coasting state
    // for now just instantly switch to the coasting state
    flight_state = FS_COASTING;
}

static bool deploy_drogues()
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(PYRO_CHANNEL_1);
        if (cont) {
            pyro_activate(PYRO_CHANNEL_1, 150*(i+1));
            vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(PYRO_CHANNEL_1);
            if (!cont) return true;
        }
    }

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

static bool deploy_mains()
{
    bool cont;

    for (int i = 0; i < 2; i++) {
        cont = pyro_continuity(PYRO_CHANNEL_2);
        if (cont) {
            pyro_activate(PYRO_CHANNEL_2, 150*(i+1));
            vTaskDelay(50 / portTICK_PERIOD_MS);
            cont = pyro_continuity(PYRO_CHANNEL_2);
            if (!cont) return true;
        }
    }

#ifdef LED_PYRO
    return true;
#else
    return false;
#endif
}

static void flight_coasting()
{
    if (barometric_agl > APOGEE_MIN && average_barometric_velocity < 0) {
        if (deploy_drogues()) flight_state = FS_UNDER_DROGUES;
        else flight_state = FS_FREEFALL;
        return;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
}

static void flight_under_drogues()
{
    if (barometric_agl < MAINS_ALT) {
        if (deploy_mains()) flight_state = FS_UNDER_MAINS;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
}

static void flight_under_mains()
{
    if (fabs(average_barometric_velocity) < 2) {
        flight_state = FS_LANDED;
        return;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
}

void flight_task(void *pvParameters) {
    flight_state = FS_ON_PAD;
    
    while (1) {
        int sum = 0;
        sum += pyro_continuity(PYRO_CHANNEL_1);
        sum += pyro_continuity(PYRO_CHANNEL_2)*2;
        pyro_arm = sum;

        switch (flight_state) {
            case FS_ON_PAD: flight_on_pad(); break;
            case FS_POWERED_FLIGHT: flight_powered_flight(); break;
            case FS_COASTING: flight_coasting(); break;
            case FS_UNDER_DROGUES: flight_under_drogues(); break;
            case FS_UNDER_MAINS: flight_under_mains(); break;
            case FS_FREEFALL: vTaskDelay(100 / portTICK_PERIOD_MS); break;
            case FS_LANDED: vTaskDelay(100 / portTICK_PERIOD_MS); break;
            default: assert(0); break;
        }
    }
}
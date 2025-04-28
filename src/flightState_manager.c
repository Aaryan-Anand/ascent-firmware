// #include <stdio.h>
// #include <stdbool.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_timer.h"
// #include "driver/gpio.h"
// #include "interface_bmp390l.h"
// #include "driver_pyro.h"
// #include "globals.h"
// #include "flight_config.h"  // For POWERED_ALT, APOGEE_MIN, and MAINS_ALT
// #include <assert.h>
// #include "flightState_manager.h"

// const char* flight_state_to_string(enum FlightState state) {
//     switch (state) {
//         case FS_ON_PAD: return "ON PAD";
//         case FS_POWERED_FLIGHT: return "POWERED FLIGHT";
//         case FS_COAST: return "COAST";
//         case FS_UNDER_DROGUES: return "UNDER DROGUES";
//         case FS_UNDER_MAINS: return "UNDER MAINS";
//         case FS_LANDED: return "LANDED";
//         case FS_FREEFALL: return "FREEFALL";
//         default: return "UNKNOWN";
//     }
// }

// static void flight_on_pad()
// {
//     // TODO: also check for accelerometer spike
//     printf("Acc X = %d", acc.x);
//     if (acc.x > 3000) {
//         flight_state = FS_POWERED_FLIGHT;
//         return;
//     }
// }



// static void flight_powered_flight()
// {
//     printf("Acc X = %d", acc.x);
//     if (acc.x < 0) {
//         flight_state = FS_COAST;
//         return;
//     }
// }

// static bool deploy_drogues()
// {
//     bool cont;

//     for (int i = 0; i < 2; i++) {
//         cont = pyro_continuity(PYRO_CHANNEL_1);
//         if (cont) {
//             pyro_activate(PYRO_CHANNEL_1, 150*(i+1), 0);
//             // vTaskDelay(50 / portTICK_PERIOD_MS);
//             cont = pyro_continuity(PYRO_CHANNEL_1);
//             if (!cont) return true;
//         }
//     }

// #ifdef LED_PYRO
//     return true;
// #else
//     return false;
// #endif
// }

// static bool deploy_mains()
// {
//     bool cont;

//     for (int i = 0; i < 2; i++) {
//         cont = pyro_continuity(PYRO_CHANNEL_2);
//         if (cont) {
//             pyro_activate(PYRO_CHANNEL_2, 150*(i+1), 0);
//             // vTaskDelay(50 / portTICK_PERIOD_MS);
//             cont = pyro_continuity(PYRO_CHANNEL_2);
//             if (!cont) return true;
//         }
//     }

// #ifdef LED_PYRO
//     return true;
// #else
//     return false;
// #endif
// }

// static void flight_coast()
// {
//     printf("AGL = %2f", barometric_agl);
//     if (barometric_agl > APOGEE_MIN && average_barometric_velocity < 0) {
//         if (deploy_drogues()) flight_state = FS_UNDER_DROGUES;
//         else flight_state = FS_FREEFALL;
//         return;
//     }
// }

// static void flight_under_drogues()
// {
//     if (barometric_agl < MAINS_ALT) {
//         if (deploy_mains()) flight_state = FS_UNDER_MAINS;
//     }
// }

// static void flight_under_mains()
// {
//     if (fabs(average_barometric_velocity) < 2) {
//         flight_state = FS_LANDED;
//         return;
//     }
// }

// #define HISTORY_SIZE 3
// #define VELOCITY_HISTORY_SIZE 10
// #define DT 0.01f
// void baro_task()
// {
//     static float agl_history[HISTORY_SIZE] = {0};  // Store the last 5 AGL readings
//     // double pressure_hPa;
//     // double temperature;

//     static float velocity_samples[VELOCITY_HISTORY_SIZE] = {0};
//     static int sample_index = 0;

//     // Shift history
//     for (int i = HISTORY_SIZE - 1; i > 0; i--) {
//         agl_history[i] = agl_history[i - 1];
//     }

//     // Update with latest AGL
//     agl_history[0] = bmp390_barometricAGL();
//     barometric_agl = agl_history[0];
//     // bmp390_read_sensor_data(&pressure_hPa, &temperature);

//     // Compute first-order backward finite difference
//     if (agl_history[1] != 0) {
//         barometric_velocity = barometric_velocity*0.2 + ((agl_history[0] - agl_history[1]) / DT)*0.8;
//     }

//     // Update velocity samples
//     velocity_samples[sample_index] = barometric_velocity;
//     sample_index = (sample_index + 1) % VELOCITY_HISTORY_SIZE;

//     float sum = 0;
//     // Calculate the sum of all samples
//     for (int i = 0; i < 10; i++) {
//         sum += velocity_samples[i];
//     }

//     // Calculate the average velocity
//     average_barometric_velocity = sum / 10.0;

//     // printf("AGL: %f, Pressure: %f, Velocity: %f\n", agl_history[0], pressure_hPa, barometric_velocity);
// }


// void flight_state_manager(void* pvParameters) {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms period = 100Hz
    
//     xLastWakeTime = xTaskGetTickCount();

//     while(1) {
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
//         printf("%s\n", flight_state_to_string(flight_state));
        
//         switch (flight_state) {
//             case FS_ON_PAD: flight_on_pad(); break;
//             case FS_POWERED_FLIGHT: flight_powered_flight(); break;
//             case FS_COAST: flight_coast(); break;
//             case FS_UNDER_DROGUES: flight_under_drogues(); break;
//             case FS_UNDER_MAINS: flight_under_mains(); break;
//             case FS_FREEFALL: break;
//             case FS_LANDED: break;
//             default: assert(0); break;
//         }
//     }
// }
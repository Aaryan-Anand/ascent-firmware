#include "sensor_manager.h"
#include "driver_bno055.h"
#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "interface_bno055.h"
#include "interface_h3lis331dl.h"
#include "ascent_r2_hardware_definition.h"
#include "globals.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "nvs_interface.h"
#include "beep.h"

double groundPressure, groundTemperature, groundAlt;
uint8_t num_readings = 30;

float bmp_scaling = 1.0f;
float bmp_bias = 0.0f;

// Keep these correction matrices and bias vectors
float acc_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

float gyr_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

float mag_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

float high_g_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

float acc_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float gyr_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float mag_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float high_g_bias_vector[3] = {0.0f, 0.0f, 0.0f};

// Remove redundant rotation and calibration helper functions
// (They're now in the interfaces)

void i2c_init(){
    esp_err_t ret1 = i2c_manager_deinit(I2C_NUM_0);
    if (ret1 != ESP_OK) {
        printf("Failed to deinit I2C\n");
        return;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    esp_err_t ret = i2c_manager_init(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ, I2C_MASTER_PORT);
    if (ret != ESP_OK) {
        printf("Failed to initialize I2C\n");
        return;
    }
    
    // Initialize interface mutexes
    bno055_interface_init();
    h3lis331dl_interface_init();
    bmp390_interface_init();
}

static void print_mat_3x3(float* mat)
{
    printf("[%f, %f, %f]\n", mat[0], mat[1], mat[2]);
    printf("[%f, %f, %f]\n", mat[3], mat[4], mat[5]);
    printf("[%f, %f, %f]\n\n", mat[6], mat[7], mat[8]);
}

static void print_vec_3(float* vec)
{
    printf("[%f, %f, %f]\n\n", vec[0], vec[1], vec[2]);
}

static nvs_handle_t my_handle;
void sensor_manager_init() {
    my_handle = nvs_interface_get_handle();


    size_t length = sizeof(acc_correction_matrix);

    // mats
    length = sizeof(acc_correction_matrix);
    if (nvs_find_key(my_handle, "acc_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "acc_mat", acc_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load acc_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(gyr_correction_matrix);
    if (nvs_find_key(my_handle, "gyr_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "gyr_mat", gyr_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load gyr_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(mag_correction_matrix);
    if (nvs_find_key(my_handle, "mag_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "mag_mat", mag_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load mag_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(high_g_correction_matrix);
    if (nvs_find_key(my_handle, "high_g_mat", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "high_g_mat", high_g_correction_matrix, &length) != ESP_OK) {
            printf("Failed to load high_g_correction_matrix\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }


    // vectors
    length = sizeof(acc_bias_vector);
    if (nvs_find_key(my_handle, "acc_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "acc_vec", acc_bias_vector, &length) != ESP_OK) {
            printf("Failed to load acc_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(gyr_bias_vector);
    if (nvs_find_key(my_handle, "gyr_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "gyr_vec", gyr_bias_vector, &length) != ESP_OK) {
            printf("Failed to load gyr_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(mag_bias_vector);
    if (nvs_find_key(my_handle, "mag_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "mag_vec", mag_bias_vector, &length) != ESP_OK) {
            printf("Failed to load mag_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    length = sizeof(high_g_bias_vector);
    if (nvs_find_key(my_handle, "high_g_vec", NULL) == ESP_OK) {
        if (nvs_get_blob(my_handle, "high_g_vec", high_g_bias_vector, &length) != ESP_OK) {
            printf("Failed to load high_g_bias_vector\n");
            esp_restart();
        }
    } else {
        printf("faild to load matix defaulting to matrix in c file\n");
    }

    print_mat_3x3((float*) acc_correction_matrix);
    print_mat_3x3((float*) gyr_correction_matrix);
    print_mat_3x3((float*) mag_correction_matrix);
    print_mat_3x3((float*) high_g_correction_matrix);
    print_vec_3(acc_bias_vector);
    print_vec_3(gyr_bias_vector);
    print_vec_3(mag_bias_vector);
    print_vec_3(high_g_bias_vector);
}

void bmp_aquire_ground() {
    // Calculate and store ground pressure and altitude
    update_ground_pressure(&groundPressure, &groundTemperature, num_readings);
    pressure_to_m(&groundPressure, &groundTemperature, &groundAlt);
    
    // Set the ground altitude in the interface
    bmp390_set_ground_alt(groundAlt);
}

void bmp_flight_init(){
    // Initialize the BMP390 sensor
    bmp390_init(I2C_MASTER_PORT);

    bmp390_osr_settings_t osr_settings = {
        .press_os = BMP390_OVERSAMPLING_2X,
        .temp_os = BMP390_OVERSAMPLING_2X
    };

    bmp390_set_osr(&osr_settings);

    bmp390_odr_t odr_settings = BMP390_ODR_100HZ;

    bmp390_set_odr(odr_settings);

    bmp390_config_t filterconfig = {
        .iir_filter = BMP390_IIR_FILTER_COEFF_63
    };

    bmp390_set_config(&filterconfig);

    printf("BMP Configured!\n");

    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Pass calibration parameters to the BMP interface
    bmp390_set_calibration(bmp_scaling, bmp_bias);
    
    bmp_aquire_ground();
}

void bno_flight_init(){
    bno055_init(I2C_MASTER_PORT);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bno_setoprmode(CONFIG);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_configure_acc(NORMAL, ACC_C_H1000, ACC_C_RANGE_16G);  //Normal power, 1kHz ODR, 16G range
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_set_acc_int(false, false, false, false, false, false, 2); // HG on X/Y/Z
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setinterruptenable(false, false, false, false, false, false, false, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setinterruptmask(false, false, false, false, false, false, false, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // bno_set_acc_int(true, true, true, true, true, true, 2); // HG on X/Y/Z
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // bno_setinterruptenable(false, true, false, false, false, false, false, false);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // bno_setinterruptmask(false, true, false, false, false, false, false, false);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // bno_set_acc_hgtresh(187);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // bno_set_acc_hgduration(10);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setoprmode(AMG);

    //calibrate_gyr_bias_5s();
    // Pass calibration matrices and bias vectors to the BNO interface
    bno055_set_calibration(
        acc_correction_matrix, gyr_correction_matrix, mag_correction_matrix,
        acc_bias_vector, gyr_bias_vector, mag_bias_vector
    );
}

void lis331_flight_init(){
    h3lis331dl_init(I2C_MASTER_PORT);
    
    // Pass calibration matrix and bias vector to the H3LIS331DL interface
    h3lis331dl_set_calibration(high_g_correction_matrix, high_g_bias_vector);

    h3lis331dl_set_power_mode(H3LIS331DL_NORMAL);
    h3lis331dl_set_datarate(H3LIS331DL_DATARATE_1000HZ);
    h3lis331dl_set_axes_config(H3LIS331DL_CONFIG_XYZ);
    h3lis331dl_set_scale(H3LIS331DL_SCALE_100G);
    h3lis331dl_set_endian(H3LIS331DL_BIG_ENDIAN);

    //high-side int detection
    // uint8_t enables_mask = (1 << 1) | (1 << 3); //XHIE and YHIE
    // h3lis331dl_set_int_cfg(H3LIS331DL_INT1, enables_mask, false);//OR mode

    // //Threshold: 2.828g (Net 4G with gravity -> 3g power acceleration)
    // h3lis331dl_set_int_threshold(H3LIS331DL_INT1, 2.828);

    // // Duration: 5 → 5ms @ 1000Hz
    // h3lis331dl_set_int_duration(H3LIS331DL_INT1, 5);

    // // Optional cleanup
    // h3lis331dl_set_int_level(H3LIS331DL_INT_ACTIVE_LOW);
    // h3lis331dl_set_int_pin_mode(H3LIS331DL_INT_PUSH_PULL);
    // h3lis331dl_set_int1_latch(true);
}


#include "stdbool.h"

#include "globals.h"
#include "interface_bmp390l.h"

#define HISTORY_SIZE 3
#define VELOCITY_HISTORY_SIZE 10
#define DT 0.02f

static float barometric_agl;
static float barometric_velocity;
static float average_barometric_velocity;

void baro_update(const baro_double_t * const baro, float *agl, float *vel, float *avg_vel)
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
    agl_history[0] = baro->alt;
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

    *agl = barometric_agl;
    *vel = barometric_velocity;
    *avg_vel = average_barometric_velocity;
}

void calibrate_gyr_bias_5s(void)
{
    const int64_t duration_us = 10 * 1000 * 1000;
    const TickType_t sample_period = pdMS_TO_TICKS(10);

    imu_local_3d_t acc, gyr, mag;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    uint32_t n = 0;

    const int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < duration_us) {
        bno055_get_local(&acc, &gyr, &mag, false);

        sx += (double)gyr.x;
        sy += (double)gyr.y;
        sz += (double)gyr.z;
        n++;

        vTaskDelay(sample_period);
    }

    if (n == 0) return;

    gyr_bias_vector[0] = -(float)(sx / (double)n);
    gyr_bias_vector[1] = -(float)(sy / (double)n);
    gyr_bias_vector[2] = -(float)(sz / (double)n);

    printf("Gyro bias updated (deg/s): bx=%f by=%f bz=%f (N=%u)\n",
           gyr_bias_vector[0], gyr_bias_vector[1], gyr_bias_vector[2], (unsigned)n);
}

void fail_if_barometer_bad() {
    const int n = 100;
    for (int i = 0; i < n; i++) {
        baro_double_t baro_out;
        bmp390_get_local(&baro_out);
        #ifdef BARO_TEST
            printf("Baro test (%d/%d): pressure: %f, temp: %f, alt: %f\n", i+1, n, baro_out.pressure, baro_out.temperature, baro_out.alt);
        #endif
        if (baro_out.pressure <= 0) {
            while (true) {
                error_beep();
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
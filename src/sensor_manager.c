#include "sensor_manager.h"
#include "driver_bno055.h"
#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "ascent_r2_hardware_definition.h"
#include "globals.h"

// Add these variable definitions
imu_raw_3d_t acc, gyr, mag;
imu_float_3d_t high_g_acc;
baro_double_t baro;
double groundPressure, groundTemperature, groundAlt;
uint8_t num_readings = 30;

double baro_alpha = 0.8;

// Add these definitions near the top with other calibration variables
float bmp_scaling = 1.0f;  // Default to no scaling
float bmp_bias = 0.0f;     // Default to no bias

// Correction matrices initialized to identity matrices
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

// Bias vectors initialized to zero
float acc_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float gyr_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float mag_bias_vector[3] = {0.0f, 0.0f, 0.0f};
float high_g_bias_vector[3] = {0.0f, 0.0f, 0.0f};

// Add this constant for the rotation calculations
static const float SQRT2_2 = 0.70710678118f; // sqrt(2)/2 = cos(45°) = sin(45°)

// Helper function to rotate a vector -45 degrees around Z axis
static void rotate_z_45(float* input, float* output) {
    float x = input[0];
    float y = input[1];
    
    // Rotation matrix multiplication for Z axis (-45 degrees)
    output[0] = SQRT2_2 * x + SQRT2_2 * y;   // x' = cos(-45°)x - sin(-45°)y
    output[1] = -SQRT2_2 * x + SQRT2_2 * y;  // y' = sin(-45°)x + cos(-45°)y
    output[2] = input[2];                     // z' = z (unchanged)
}

// Helper function to flip X axis if needed
static void flip_x_if_needed(float* vec, bool flip) {
    if (flip) {
        vec[0] = -vec[0];  // Negate X component
    }
}

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
}

void bmp_flight_init(){
    bmp390_sensorinit();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    update_ground_pressure(&groundPressure, &groundTemperature, num_readings);
    pressure_to_m(&groundPressure, &groundTemperature, &groundAlt);
}

void bno_flight_init(){
    bno055_init(I2C_MASTER_PORT);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bno_configure_acc(NORMAL, ACC_C_H1000, ACC_C_RANGE_16G);  //Normal power, 1kHz ODR, 16G range
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_set_acc_amthres(10);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_set_acc_int(true, true, true, true, true, true, 2); // HG on X/Y/Z
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setinterruptenable(false, true, false, false, false, false, false, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setinterruptmask(false, true, false, false, false, false, false, false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    bno_setoprmode(CONFIG);
    bno_setoprmode(AMG);
}

void lis331_flight_init(){
    h3lis331dl_init(I2C_MASTER_PORT);

    h3lis331dl_set_power_mode(H3LIS331DL_NORMAL);
    h3lis331dl_set_datarate(H3LIS331DL_DATARATE_1000HZ);
    h3lis331dl_set_axes_config(H3LIS331DL_CONFIG_XYZ);
    h3lis331dl_set_scale(H3LIS331DL_SCALE_100G);
    h3lis331dl_set_endian(H3LIS331DL_BIG_ENDIAN);

    //high-side int detection
    uint8_t enables_mask = (1 << 1) | (1 << 3); //XHIE and YHIE
    h3lis331dl_set_int_cfg(H3LIS331DL_INT1, enables_mask, false);//OR mode

    //Threshold: 2.828g (Net 4G with gravity -> 3g power acceleration)
    h3lis331dl_set_int_threshold(H3LIS331DL_INT1, 2.828);

    // Duration: 5 → 5ms @ 1000Hz
    h3lis331dl_set_int_duration(H3LIS331DL_INT1, 5);

    // Optional cleanup
    h3lis331dl_set_int_level(H3LIS331DL_INT_ACTIVE_LOW);
    h3lis331dl_set_int_pin_mode(H3LIS331DL_INT_PUSH_PULL);
    h3lis331dl_set_int1_latch(true);
}

esp_err_t bno_get(imu_raw_3d_t* acc, imu_raw_3d_t* gyr, imu_raw_3d_t* mag) {
    bno_readamg(
        &acc->x, &acc->y, &acc->z,
        &gyr->x, &gyr->y, &gyr->z,
        &mag->x, &mag->y, &mag->z
    );

    return ESP_OK;
}

void lis331_get(imu_float_3d_t* acc) {
    if (acc) {
        h3lis331dl_read_accel(&acc->x, &acc->y, &acc->z);
    }
}

void bmp_get(baro_double_t* baro){
    bmp390_read_sensor_data(&baro->pressure, &baro->temperature);
}

// Helper function to apply 3x3 matrix multiplication and bias addition
static void apply_calibration(float* input, float matrix[3][3], float* bias, float* output) {
    // Matrix multiplication
    output[0] = matrix[0][0] * input[0] + matrix[0][1] * input[1] + matrix[0][2] * input[2];
    output[1] = matrix[1][0] * input[0] + matrix[1][1] * input[1] + matrix[1][2] * input[2];
    output[2] = matrix[2][0] * input[0] + matrix[2][1] * input[1] + matrix[2][2] * input[2];

    // Add bias
    output[0] += bias[0];
    output[1] += bias[1];
    output[2] += bias[2];
}

esp_err_t bno_calib(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out) {
    // Temporary structs for raw readings
    imu_raw_3d_t raw_acc, raw_gyr, raw_mag;
    
    // Get raw sensor data
    esp_err_t ret = bno_get(&raw_acc, &raw_gyr, &raw_mag);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Temporary arrays for floating point calculations
    float acc_float[3], gyr_float[3], mag_float[3];
    float acc_cal[3], gyr_cal[3], mag_cal[3];

    // Convert raw readings to float
    acc_float[0] = (float)raw_acc.x;
    acc_float[1] = (float)raw_acc.y;
    acc_float[2] = (float)raw_acc.z;

    gyr_float[0] = (float)raw_gyr.x;
    gyr_float[1] = (float)raw_gyr.y;
    gyr_float[2] = (float)raw_gyr.z;

    mag_float[0] = (float)raw_mag.x;
    mag_float[1] = (float)raw_mag.y;
    mag_float[2] = (float)raw_mag.z;

    // Apply calibrations
    apply_calibration(acc_float, acc_correction_matrix, acc_bias_vector, acc_cal);
    apply_calibration(gyr_float, gyr_correction_matrix, gyr_bias_vector, gyr_cal);
    apply_calibration(mag_float, mag_correction_matrix, mag_bias_vector, mag_cal);

    // Store calibrated results in output
    acc_out->x = (int16_t)acc_cal[0];
    acc_out->y = (int16_t)acc_cal[1];
    acc_out->z = (int16_t)acc_cal[2];

    gyr_out->x = (int16_t)gyr_cal[0];
    gyr_out->y = (int16_t)gyr_cal[1];
    gyr_out->z = (int16_t)gyr_cal[2];

    mag_out->x = (int16_t)mag_cal[0];
    mag_out->y = (int16_t)mag_cal[1];
    mag_out->z = (int16_t)mag_cal[2];

    return ESP_OK;
}

void lis331_calib(imu_float_3d_t* acc_out) {
    // Temporary struct for raw readings
    imu_float_3d_t raw_acc;
    
    // Get raw sensor data
    lis331_get(&raw_acc);
    
    float acc_float[3], acc_cal[3];

    // Copy raw readings to array
    acc_float[0] = raw_acc.x;
    acc_float[1] = raw_acc.y;
    acc_float[2] = raw_acc.z;

    // Apply calibration
    apply_calibration(acc_float, high_g_correction_matrix, high_g_bias_vector, acc_cal);

    // Store calibrated results
    acc_out->x = acc_cal[0];
    acc_out->y = acc_cal[1];
    acc_out->z = acc_cal[2];
}

esp_err_t bno_local(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out, bool local_up_flipped) {
    // First get calibrated readings
    esp_err_t ret = bno_calib(acc_out, gyr_out, mag_out);
    if (ret != ESP_OK) {
        return ret;
    }

    // Temporary arrays for rotation calculations
    float acc_cal[3], gyr_cal[3], mag_cal[3];
    float acc_rot[3], gyr_rot[3], mag_rot[3];

    // Convert calibrated readings to float for rotation
    acc_cal[0] = (float)acc_out->x;
    acc_cal[1] = (float)acc_out->y;
    acc_cal[2] = (float)acc_out->z;

    gyr_cal[0] = (float)gyr_out->x;
    gyr_cal[1] = (float)gyr_out->y;
    gyr_cal[2] = (float)gyr_out->z;

    mag_cal[0] = (float)mag_out->x;
    mag_cal[1] = (float)mag_out->y;
    mag_cal[2] = (float)mag_out->z;

    // Apply -45° rotation
    rotate_z_45(acc_cal, acc_rot);
    rotate_z_45(gyr_cal, gyr_rot);
    rotate_z_45(mag_cal, mag_rot);

    // Flip X axis if needed
    flip_x_if_needed(acc_rot, local_up_flipped);
    flip_x_if_needed(gyr_rot, local_up_flipped);
    flip_x_if_needed(mag_rot, local_up_flipped);

    // Store rotated results
    acc_out->x = (int16_t)acc_rot[0];
    acc_out->y = (int16_t)acc_rot[1];
    acc_out->z = (int16_t)acc_rot[2];

    gyr_out->x = (int16_t)gyr_rot[0];
    gyr_out->y = (int16_t)gyr_rot[1];
    gyr_out->z = (int16_t)gyr_rot[2];

    mag_out->x = (int16_t)mag_rot[0];
    mag_out->y = (int16_t)mag_rot[1];
    mag_out->z = (int16_t)mag_rot[2];

    return ESP_OK;
}

void lis331_local(imu_float_3d_t* acc_out, bool local_up_flipped) {
    // First get calibrated readings
    lis331_calib(acc_out);

    // Temporary arrays for rotation calculations
    float acc_cal[3], acc_rot[3];

    // Copy calibrated readings to array
    acc_cal[0] = acc_out->x;
    acc_cal[1] = acc_out->y;
    acc_cal[2] = acc_out->z;

    // Apply -45° rotation
    rotate_z_45(acc_cal, acc_rot);

    // Flip X axis if needed
    flip_x_if_needed(acc_rot, local_up_flipped);

    // Store rotated results
    acc_out->x = acc_rot[0];
    acc_out->y = acc_rot[1];
    acc_out->z = acc_rot[2];
}

// Add this new function
void bmp_calib(baro_double_t* baro_out) {
    static baro_double_t baro_prev = baro_out;
    bmp_get(baro_out);
    baro_out->pressure = baro_out->pressure * bmp_scaling + bmp_bias;
    baro_out->pressure = baro_out.pressure*baro_alpha +  baro_prev.pressure*(1-baro_alpha);
    pressure_to_m(&baro_out->pressure, &baro_out->temperature, &baro_out->alt);
}

void bmp_local(baro_double_t* baro_out) {
    bmp_calib(baro_out);
    baro_out->alt = baro_out->alt - groundAlt;
}
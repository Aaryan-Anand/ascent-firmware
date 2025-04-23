#include "sensor_manager.h"
#include "driver_bno055.h"
#include "driver_H3LIS331DL.h"
#include "interface_bmp390l.h"
#include "ascent_r2_hardware_definition.h"
#include "globals.h"

// Add these variable definitions
imu_raw_3d_t acc, gyr, mag;
imu_float_3d_t high_g_acc;

extern volatile bool print_bno_data;

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
    update_ground_pressure();
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

    // Create a queue to handle GPIO interrupt events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Configure GPIO interrupt
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Interrupt on rising edge
        .pin_bit_mask = (1ULL << PIN_BNO055_INT),  // Select BNO055 interrupt pin
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    printf("Initial INT pin state: %d\n", gpio_get_level(PIN_BNO055_INT));

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BNO055_INT, gpio_isr_handler, (void*) PIN_BNO055_INT);
    xTaskCreate(bno_interrupt_task, "bno_interrupt_task", 2048, NULL, 10, NULL);
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

QueueHandle_t gpio_evt_queue = NULL;

void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void bno_interrupt_task(void* arg) {
    uint32_t io_num;
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI("BNO", "Interrupt detected on GPIO %" PRIu32, io_num);
            
            // Toggle printing of BNO data
            print_bno_data = !print_bno_data;
            
            // Clear the interrupt by reading the interrupt status
            bool acc_nm, acc_am, acc_high_g, gyro_drdy, 
                 gyr_high_rate, gyr_am, mag_drdy, acc_drdy;
            bno_getinterruptstatus(&acc_nm, &acc_am, &acc_high_g, &gyro_drdy, 
                                 &gyr_high_rate, &gyr_am, &mag_drdy, &acc_drdy);
        }
    }
}
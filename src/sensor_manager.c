#include <sensor_correction.h>

void bmp_flight_init(){
    bmp390_sensorinit();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    update_ground_pressure();
}

void bno_flight_init(){
    bno055_init(I2C_MASTER_PORT);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bno_setoprmode(CONFIG);
    bno_configure_acc(NORMAL, ACC_C_H1000, ACC_C_RANGE_16G);
    bno_set_acc_hgtresh(187);//3g
    bno_set_acc_hgduration(10);//20ms above 3g
    bno_set_acc_int(true, true, true, false, false, false, 0);//HG on X/Y/Z
    bno_setinterruptenable(false, false, true, false, false, false, false, false);
    bno_setinterruptmask(false, false, true, false, false, false, false, false);
    bno_setoprmode(AMG);
}

void lis331_flight_init(){
    h3lis331dl_init(I2C_MASTER_PORT);

    h3lis331dl_set_power_mode(H3LIS331DL_NORMAL);
    h3lis331dl_set_datarate(H3LIS331DL_DATARATE_1000HZ);
    h3lis331dl_set_axes_config(H3LIS331DL_CONFIG_XY);//Only X and Y
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









static xQueueHandle gpio_evt_queue = NULL;

// ISR handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Task to run on interrupt
static void bno_interrupt_task(void* arg) {
    uint32_t io_num;
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI("BNO", "Interrupt detected on GPIO %d", io_num);
        }
    }
}
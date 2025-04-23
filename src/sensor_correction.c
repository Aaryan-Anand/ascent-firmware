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
    bno_set_acc_hgtresh(187);// ~3g
    bno_set_acc_hgduration(10);// ~20ms above 3g
    bno_set_acc_int(true, true, true, false, false, false, 0);// HG on X/Y/Z
    bno_setinterruptenable(false, false, true, false, false, false, false, false);
    bno_setinterruptmask(false, false, true, false, false, false, false, false);
    bno_setoprmode(AMG);
}

void lis331_flight_init(){
    
}
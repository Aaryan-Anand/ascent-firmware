#include "manual_spi_bus.h"
#include <rom/ets_sys.h>

void spi_init() {
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);

    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void spi_write_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(PIN_NUM_MOSI, (data >> i) & 0x01); // Set MOSI
        gpio_set_level(PIN_NUM_CLK, 1); // Clock high
        ets_delay_us(1);
        gpio_set_level(PIN_NUM_CLK, 0); // Clock low
    }
}

uint8_t spi_read_byte() {
    uint8_t data = 0;
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(PIN_NUM_CLK, 1); // Clock high
        ets_delay_us(1);
        if (gpio_get_level(PIN_NUM_MISO)) { // Read MISO
            data |= (1 << i);
        }
        gpio_set_level(PIN_NUM_CLK, 0); // Clock low
    }

    return data;
}

uint8_t spi_exchange_byte(uint8_t tx) {
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(PIN_NUM_MOSI, (tx >> i) & 0x01); // Set MOSI

        gpio_set_level(PIN_NUM_CLK, 1); // Clock high
        ets_delay_us(1);

        if (gpio_get_level(PIN_NUM_MISO)) { // Read MISO
            rx |= (1 << i);
        }

        gpio_set_level(PIN_NUM_CLK, 0); // Clock low
    }

    return rx;
}

void spi_begin() {
    gpio_set_level(PIN_NUM_CS, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_CS, 0);
}

void spi_end() {
    gpio_set_level(PIN_NUM_CS, 1);
}
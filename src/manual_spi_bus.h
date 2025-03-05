#include <string.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 1
#define PIN_NUM_CLK  3
#define PIN_NUM_CS   4

void spi_init();

void spi_write_byte(uint8_t data);

uint8_t spi_read_byte();

uint8_t spi_exchange_byte(uint8_t tx);

void spi_begin();

void spi_end();
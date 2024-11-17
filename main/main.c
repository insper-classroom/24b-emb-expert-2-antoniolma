#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

// For the BME280
const int SPI0_SCK = 18;
const int SPI0_TX = 19;
const int SPI0_RX = 16;
const int SPI0_CSn = 17;


// ======================================================== BME280 ========================================================

#define READ_BIT 0x80

int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t) dig_P6);
    var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);
    if (var1 == 0)
        return 0;
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;
    var1 = (((int32_t) dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13 ))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t) dig_P8)) >> 13;
    p = (uint32_t)((int32_t) p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}

int32_t compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) *
              ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) *
            ((int32_t) dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI0_CSn, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI0_CSn, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7F;  // Ensure the write bit is 0
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi0, buf, 2);
    cs_deselect();
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= READ_BIT;  // Set the read bit
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    spi_read_blocking(spi0, 0, buf, len);
    cs_deselect();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void read_compensation_parameters() {
    uint8_t buffer[26];

    read_registers(0x88, buffer, 26);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25]; // 0xA1

    read_registers(0xE1, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8); // 0xE1 | 0xE2
    dig_H3 = buffer[2]; // 0xE3
    dig_H4 = (buffer[3] << 4) | (buffer[4] & 0x0F); // 0xE4 | 0xE5[3:0]
    dig_H5 = (buffer[5] << 4) | (buffer[4] >> 4); // 0xE5[7:4] | 0xE6
    dig_H6 = buffer[6]; // 0xE7
}

static void bme280_read_raw(int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[8];

    read_registers(0xF7, buffer, 8);
    *pressure = ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
    // *humidity = ((uint32_t) buffer[6] << 8) | buffer[7];
}

void bme_init() {
    printf("Hello, BME280! Reading raw data from registers via SPI...\n");

    // Initialize SPI0 at 0.5 MHz
    spi_init(spi0, 500 * 1000);
    gpio_set_function(SPI0_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_TX, GPIO_FUNC_SPI);

    // Initialize CS pin
    gpio_init(SPI0_CSn);
    gpio_set_dir(SPI0_CSn, GPIO_OUT);
    gpio_put(SPI0_CSn, 1);
}

// ========================================================= LCD ==========================================================



// ======================================================== Taks ==========================================================

// Baseado no exemplo: https://github.com/raspberrypi/pico-examples/tree/master/spi/bme280_spi
void bme_task(void *p) {
    bme_init();
    
    // Read the chip ID to verify communication
    uint8_t id;
    read_registers(0xD0, &id, 1);
    printf("Chip ID is 0x%x\n", id);

    // Read calibration parameters
    read_compensation_parameters();

    // Configure the sensor
    write_register(0xF2, 0x01);
    write_register(0xF4, 0x27);

    int32_t pressure, temperature;

    while (1) {
        bme280_read_raw(&pressure, &temperature);

        temperature = compensate_temperature(temperature);
        pressure = compensate_pressure(pressure);
        // printf("a\n");
        printf("Pressure = %d Pa\n", pressure);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();
    
    xTaskCreate(bme_task, "BME_TASK 1", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1);
}

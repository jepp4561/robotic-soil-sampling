#include "load_cell.h"

#include <stdint.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define LOAD_CELL_I2C_BUS i2c0
#define LOAD_CELL_I2C_ADDRESS 0x2A

#define LOAD_CELL_SDA_PIN 4
#define LOAD_CELL_SCL_PIN 5

#define LOAD_CELL_I2C_BAUDRATE 400000

#define NAU7802_PU_CTRL 0x00
#define NAU7802_CTRL1 0x01
#define NAU7802_CTRL2 0x02
#define NAU7802_ADC 0x15
#define NAU7802_PGA 0x1B
#define NAU7802_PGA_PWR 0x1C

#define NAU7802_PU_CTRL_PUD 1
#define NAU7802_PU_CTRL_PUA 2
#define NAU7802_PU_CTRL_PUR 3
#define NAU7802_PU_CTRL_CR 5
#define NAU7802_PU_CTRL_CS 4
#define NAU7802_PU_CTRL_RR 0
#define NAU7802_PU_CTRL_AVDDS 7

#define NAU7802_PGA_LDOMODE 6
#define NAU7802_PGA_PWR_PGA_CAP_EN 7

#define NAU7802_ADCO_B2 0x12

#define LOAD_CELL_GAIN_128 0x07
#define LOAD_CELL_LDO_3V3 0x04
#define LOAD_CELL_SPS_80 0x03

int32_t LOAD_CELL_BIAS;
const float LOAD_CELL_CONVERSION_FACTOR = 0.000675f;

static int32_t *buffer;
static uint8_t buffer_size;
static uint8_t buffer_index;
static uint8_t sample_count;
static int64_t buffer_sum;

static bool load_cell_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};

    return i2c_write_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, data, 2, false) == 2;
}

static bool load_cell_read_register(uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, &reg, 1, true) != 1) {
        return false;
    }

    return i2c_read_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, value, 1, false) == 1;
}

static bool load_cell_set_bit(uint8_t reg, uint8_t bit) {
    uint8_t value;

    if (!load_cell_read_register(reg, &value)) {
        return false;
    }

    value |= (uint8_t)(1u << bit);

    return load_cell_write_register(reg, value);
}

static bool load_cell_clear_bit(uint8_t reg, uint8_t bit) {
    uint8_t value;

    if (!load_cell_read_register(reg, &value)) {
        return false;
    }

    value &= (uint8_t)~(1u << bit);

    return load_cell_write_register(reg, value);
}

static bool load_cell_get_bit(uint8_t reg, uint8_t bit) {
    uint8_t value;

    if (!load_cell_read_register(reg, &value)) {
        return false;
    }

    return (value & (uint8_t)(1u << bit)) != 0;
}

static bool load_cell_reset(void) {
    if (!load_cell_set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR)) {
        return false;
    }

    sleep_ms(1);

    return load_cell_clear_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR);
}

static bool load_cell_power_up(void) {
    if (!load_cell_set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD)) {
        return false;
    }

    if (!load_cell_set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUA)) {
        return false;
    }

    for (uint32_t i = 0; i < 100; ++i) {
        if (load_cell_get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUR)) {
            return load_cell_set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_CS);
        }

        sleep_ms(1);
    }

    return false;
}

static bool load_cell_configure(void) {
    uint8_t value;

    if (!load_cell_read_register(NAU7802_CTRL1, &value)) {
        return false;
    }

    value &= 0xC7;
    value |= (uint8_t)(LOAD_CELL_LDO_3V3 << 3);
    value |= LOAD_CELL_GAIN_128;

    if (!load_cell_write_register(NAU7802_CTRL1, value)) {
        return false;
    }

    if (!load_cell_set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_AVDDS)) {
        return false;
    }

    if (!load_cell_read_register(NAU7802_CTRL2, &value)) {
        return false;
    }

    value &= 0x8F;
    value |= (uint8_t)(LOAD_CELL_SPS_80 << 4);

    if (!load_cell_write_register(NAU7802_CTRL2, value)) {
        return false;
    }

    if (!load_cell_read_register(NAU7802_ADC, &value)) {
        return false;
    }

    value |= 0x30;

    if (!load_cell_write_register(NAU7802_ADC, value)) {
        return false;
    }

    if (!load_cell_set_bit(NAU7802_PGA_PWR, NAU7802_PGA_PWR_PGA_CAP_EN)) {
        return false;
    }

    if (!load_cell_clear_bit(NAU7802_PGA, NAU7802_PGA_LDOMODE)) {
        return false;
    }

    sleep_ms(250);

    return true;
}

static bool load_cell_connected(void) {
    uint8_t value;

    return i2c_read_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, &value, 1, false) >= 0;
}

bool load_cell_init(uint8_t requested_buffer_size) {
    if (requested_buffer_size == 0) {
        return false;
    }

    if (buffer != NULL) {
        free(buffer);
        buffer = NULL;
    }

    buffer = calloc(requested_buffer_size, sizeof(int32_t));

    if (buffer == NULL) {
        return false;
    }

    buffer_size = requested_buffer_size;
    buffer_index = 0;
    sample_count = 0;
    buffer_sum = 0;

    i2c_init(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_BAUDRATE);

    gpio_set_function(LOAD_CELL_SDA_PIN, GPIO_FUNC_I2C);

    gpio_set_function(LOAD_CELL_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(LOAD_CELL_SDA_PIN);
    gpio_pull_up(LOAD_CELL_SCL_PIN);

    sleep_ms(10);

    if (!load_cell_connected()) {
        return false;
    }

    if (!load_cell_reset()) {
        return false;
    }

    if (!load_cell_power_up()) {
        return false;
    }

    if (!load_cell_configure()) {
        return false;
    }

    load_cell_calibrate_bias(100);

    return load_cell_connected();
}

bool load_cell_available(void) {
    return load_cell_get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_CR);
}

bool load_cell_update(void) {
    uint8_t data[3];

    if (i2c_write_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, (uint8_t[]){NAU7802_ADCO_B2}, 1, true) != 1) {
        return false;
    }

    if (i2c_read_blocking(LOAD_CELL_I2C_BUS, LOAD_CELL_I2C_ADDRESS, data, 3, false) != 3) {
        return false;
    }

    int32_t value = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2];

    if (value & 0x800000) {
        value |= (int32_t)0xFF000000;
    }

    if (sample_count < buffer_size) {
        buffer[sample_count] = value;
        buffer_sum += value;
        sample_count++;
    } else {
        buffer_sum -= buffer[buffer_index];
        buffer[buffer_index] = value;
        buffer_sum += value;

        buffer_index++;

        if (buffer_index >= buffer_size) {
            buffer_index = 0;
        }
    }

    return true;
}

float load_cell_read_newton(void) {
    return LOAD_CELL_CONVERSION_FACTOR * (load_cell_read_average() - LOAD_CELL_BIAS);
}

int32_t load_cell_read_average(void) {
    if (sample_count == 0) {
        return 0;
    }

    return (int32_t)(buffer_sum / sample_count);
}

uint8_t load_cell_get_sample_count(void) {
    return sample_count;
}

void load_cell_calibrate_bias(uint8_t num_samples) {
    int sum = 0;
    for (int i = 0; i < num_samples; i++) {
        load_cell_update();
        sum += load_cell_read_average();
        sleep_ms(10);
    }
    LOAD_CELL_BIAS = (int32_t)(sum / num_samples);
}

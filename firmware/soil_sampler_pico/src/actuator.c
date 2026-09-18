#include "actuator.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define HALL_COUNTS_PER_METER 23335.0f

#define MAX_ACTUATORS 8

static actuator_t *actuators[MAX_ACTUATORS];
static uint actuator_count = 0;

static actuator_t *find_actuator_by_hall_pin(uint gpio)
{
    for (uint i = 0; i < actuator_count; ++i) {
        if (actuators[i]->hall_pin == gpio) {
            return actuators[i];
        }
    }

    return NULL;
}

static void actuator_hall_callback(uint gpio, uint32_t events)
{
    if ((events & GPIO_IRQ_EDGE_RISE) == 0) {
        return;
    }

    actuator_t *actuator = find_actuator_by_hall_pin(gpio);

    if (actuator == NULL) {
        return;
    }

    if (actuator->current_direction == 1) {
        ++actuator->hall_count;
    } else if (actuator->current_direction == -1) {
        --actuator->hall_count;
    }
}

bool actuator_init(
    actuator_t *actuator,
    uint pwm_pin,
    uint dir_pin,
    uint sleep_pin,
    uint fault_pin,
    uint hall_pin
)
{
    if (actuator == NULL) {
        return false;
    }

    if (actuator_count >= MAX_ACTUATORS) {
        return false;
    }

    actuator->pwm_pin = pwm_pin;
    actuator->dir_pin = dir_pin;
    actuator->sleep_pin = sleep_pin;
    actuator->fault_pin = fault_pin;
    actuator->hall_pin = hall_pin;

    actuator->hall_count = 0;
    actuator->current_direction = 0;
    actuator->enabled = false;
    actuator->hall_counts_per_meter = HALL_COUNTS_PER_METER;

    gpio_init(actuator->pwm_pin);
    gpio_set_dir(actuator->pwm_pin, GPIO_OUT);
    gpio_put(actuator->pwm_pin, 0);

    gpio_init(actuator->dir_pin);
    gpio_set_dir(actuator->dir_pin, GPIO_OUT);
    gpio_put(actuator->dir_pin, 0);

    gpio_init(actuator->sleep_pin);
    gpio_set_dir(actuator->sleep_pin, GPIO_OUT);
    gpio_put(actuator->sleep_pin, 1);
    actuator->enabled = true;

    gpio_init(actuator->fault_pin);
    gpio_set_dir(actuator->fault_pin, GPIO_IN);

    gpio_init(actuator->hall_pin);
    gpio_set_dir(actuator->hall_pin, GPIO_IN);
    gpio_pull_down(actuator->hall_pin);

    actuators[actuator_count] = actuator;
    ++actuator_count;

    gpio_set_irq_enabled_with_callback(
        actuator->hall_pin,
        GPIO_IRQ_EDGE_RISE,
        true,
        &actuator_hall_callback
    );

    return true;
}

void actuator_hall_reset(actuator_t *actuator)
{
    if (actuator == NULL) {
        return;
    }

    uint32_t interrupts = save_and_disable_interrupts();
    actuator->hall_count = 0;
    restore_interrupts(interrupts);
}

void actuator_extend(actuator_t *actuator)
{
    if (actuator == NULL || !actuator->enabled) {
        return;
    }

    actuator->current_direction = 1;

    gpio_put(actuator->dir_pin, 1);
    gpio_put(actuator->pwm_pin, 1);
}

void actuator_retract(actuator_t *actuator)
{
    if (actuator == NULL || !actuator->enabled) {
        return;
    }

    actuator->current_direction = -1;

    gpio_put(actuator->dir_pin, 0);
    gpio_put(actuator->pwm_pin, 1);
}

void actuator_stop(actuator_t *actuator)
{
    if (actuator == NULL) {
        return;
    }

    gpio_put(actuator->pwm_pin, 0);
    actuator->current_direction = 0;
}

void actuator_enable(actuator_t *actuator, bool enable)
{
    if (actuator == NULL) {
        return;
    }

    gpio_put(actuator->sleep_pin, enable ? 1 : 0);
    actuator->enabled = enable;

    if (!enable) {
        actuator_stop(actuator);
    }
}

bool actuator_is_enabled(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return false;
    }

    return actuator->enabled;
}

bool actuator_fault_active(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return false;
    }

    return gpio_get(actuator->fault_pin) == 0;
}

int32_t actuator_get_hall_count(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return 0;
    }

    uint32_t interrupts = save_and_disable_interrupts();
    int32_t count = actuator->hall_count;
    restore_interrupts(interrupts);

    return count;
}

float actuator_get_position(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return 0.0f;
    }

    int32_t count = actuator_get_hall_count(actuator);

    return (float)count / actuator->hall_counts_per_meter;
}

int8_t actuator_get_direction(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return 0;
    }

    return actuator->current_direction;
}
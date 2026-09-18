#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

typedef struct {
    uint pwm_pin;
    uint dir_pin;
    uint sleep_pin;
    uint fault_pin;
    uint hall_pin;

    volatile int32_t hall_count;
    volatile int8_t current_direction;
    volatile bool enabled;

    float hall_counts_per_meter;
} actuator_t;

bool actuator_init(
    actuator_t *actuator,
    uint pwm_pin,
    uint dir_pin,
    uint sleep_pin,
    uint fault_pin,
    uint hall_pin
);

void actuator_hall_reset(actuator_t *actuator);

void actuator_extend(actuator_t *actuator);
void actuator_retract(actuator_t *actuator);
void actuator_stop(actuator_t *actuator);

void actuator_enable(actuator_t *actuator, bool enable);
bool actuator_is_enabled(const actuator_t *actuator);

bool actuator_fault_active(const actuator_t *actuator);

int32_t actuator_get_hall_count(const actuator_t *actuator);
float actuator_get_position(const actuator_t *actuator);
int8_t actuator_get_direction(const actuator_t *actuator);

#endif
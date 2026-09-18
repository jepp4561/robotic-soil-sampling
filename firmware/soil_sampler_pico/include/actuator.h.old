#ifndef SOIL_SAMPLER_ACTUATOR_H
#define SOIL_SAMPLER_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

bool actuator_init(void);
void actuator_hall_reset(void);
void actuator_extend(void);
void actuator_retract(void);
void actuator_stop(void);
void actuator_enable(bool enable);
bool actuator_fault_active(void);
int32_t actuator_get_hall_count(void);
float actuator_get_position(void);
int8_t actuator_get_direction(void);
bool actuator_is_enabled(void);

#endif
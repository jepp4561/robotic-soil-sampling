#ifndef SOIL_SAMPLER_ENVIRONMENT_H
#define SOIL_SAMPLER_ENVIRONMENT_H

#include "soil_sampler_interfaces/msg/actuator_state.h"

void environment_init(void);

void environment_update(void);

void environment_get_state(
    soil_sampler_interfaces__msg__ActuatorState *state);

#endif
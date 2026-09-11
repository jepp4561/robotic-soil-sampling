#include "environment.h"

#include <stdbool.h>

#include "pico/stdlib.h"


static bool initialized = false;

static float temperature_c = 0.0f;
static float humidity_percent = 0.0f;


void environment_init(void)
{
    /*
     * Initialize the temperature/humidity sensor here.
     */

    initialized = true;
}


void environment_update(
    soil_sampler_interfaces__msg__ActuatorState *state)
{
    if (state == NULL) {
        return;
    }

    if (!initialized) {
        return;
    }

    /*
     * Replace these values with the actual sensor
     * implementation.
     */

    (void)temperature_c;
    (void)humidity_percent;
}
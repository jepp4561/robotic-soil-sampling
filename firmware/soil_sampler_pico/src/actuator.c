#include "actuator.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#define HALL_COUNTS_PER_METER 23335.0f

#define MAX_ACTUATORS 8

#define SOFT_START_MIN_DUTY 20
#define SOFT_START_MAX_DUTY 100

// PWM wrap value: 125 MHz / (PWM_WRAP + 1) = ~20 kHz
#define PWM_WRAP 6249

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

    gpio_set_function(actuator->pwm_pin, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);

    actuator->slice_num = pwm_gpio_to_slice_num(actuator->pwm_pin);
    uint channel = pwm_gpio_to_channel(actuator->pwm_pin);

    pwm_init(actuator->slice_num, &config, false);
    pwm_set_chan_level(actuator->slice_num, channel, 0);
    pwm_set_enabled(actuator->slice_num, true);

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
    actuator_soft_start(actuator);
}

void actuator_retract(actuator_t *actuator)
{
    if (actuator == NULL || !actuator->enabled) {
        return;
    }

    actuator->current_direction = -1;

    gpio_put(actuator->dir_pin, 0);
    actuator_soft_start(actuator);
}

void actuator_stop(actuator_t *actuator)
{
    if (actuator == NULL) {
        return;
    }

    pwm_set_chan_level(actuator->slice_num, pwm_gpio_to_channel(actuator->pwm_pin), 0);
    actuator->current_direction = 0;
    actuator->pwm_duty = 0;
}

void actuator_soft_start(actuator_t *actuator)
{
    if (actuator == NULL) {
        return;
    }

    const uint8_t step_size = 10;
    const uint32_t step_delay_ms = 12;
 The level is calculated by multiplying the duty cycle (in percentage) by the PWM wrap value and dividing by 100 to get the actual level for the PWM channel.
    for (uint8_t duty = SOFT_START_MIN_DUTY;
         duty <= SOFT_START_MAX_DUTY;
         duty += step_size)
    {
        actuator->pwm_duty = duty;

        uint16_t level =
            (uint16_t)((duty * PWM_WRAP) / 100);
        // level is the PWM level corresponding to the duty cycle in relation to the frequency of the PWM (PWM_WRAP).

        pwm_set_chan_level(
            actuator->slice_num,
            pwm_gpio_to_channel(actuator->pwm_pin),
            level
        );

        if (duty < SOFT_START_MAX_DUTY) {
            sleep_ms(step_delay_ms);
        }
    }

    actuator->pwm_duty = SOFT_START_MAX_DUTY;

    pwm_set_chan_level(
        actuator->slice_num,
        pwm_gpio_to_channel(actuator->pwm_pin),
        PWM_WRAP
    );
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
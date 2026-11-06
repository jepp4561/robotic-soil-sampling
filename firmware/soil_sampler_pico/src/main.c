#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <soil_sampler_interfaces/msg/actuator_command.h>
#include <soil_sampler_interfaces/msg/actuator_state.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "actuator.h"
#include "load_cell.h"

#define LED_PIN 25

#define AGENT_PING_TIMEOUT_MS 1000
#define AGENT_PING_ATTEMPTS 120

#define LOAD_CELL_PUBLISH_PERIOD_MS 100

#define ACTUATOR_EXTEND 1
#define ACTUATOR_STOP 0
#define ACTUATOR_RETRACT -1

#define ACTUATOR_POSITION_PUBLISH_PERIOD_MS 100

// Calibration timing. The vertical actuator starts retracting first; the horizontal one starts CALIBRATION_STAGGER_MS later. Both then keep retracting for CALIBRATION_TIME_MS before being stopped and zeroed.
#define CALIBRATION_STAGGER_MS 5000
#define CALIBRATION_TIME_MS 10000
#define CALIBRATION_EXTEND_TIME_MS 3000   // vertical extends this long after zeroing, to unload friction
#define CALIBRATION_SETTLE_MS 500         // pause after stopping before biasing the load cell
#define CALIBRATION_LOAD_CELL_SAMPLES 100

// Overload protection. If the vertical actuator is extending and the load cell reads more than MAX_LOAD_N, it is stopped and retracted by OVERLOAD_RETRACT_DISTANCE_M (or until OVERLOAD_RETRACT_TIMEOUT_MS passes, in case the hall sensor fails)
#define MAX_LOAD_N 100.0f
#define OVERLOAD_RETRACT_DISTANCE_M 0.010f
#define OVERLOAD_RETRACT_TIMEOUT_MS 3000

#define VERTICAL_ACTUATOR_DIR_PIN 15
#define VERTICAL_ACTUATOR_PWM_PIN 14
#define VERTICAL_ACTUATOR_SLEEP_PIN 13
#define VERTICAL_ACTUATOR_FAULT_PIN 12
#define VERTICAL_ACTUATOR_HALL_PIN 11
#define VERTICAL_ACTUATOR_CURRENT_SENSE_PIN 10

#define HORIZONTAL_ACTUATOR
#define HORIZONTAL_ACTUATOR_DIR_PIN 16
#define HORIZONTAL_ACTUATOR_PWM_PIN 17
#define HORIZONTAL_ACTUATOR_SLEEP_PIN 18
#define HORIZONTAL_ACTUATOR_FAULT_PIN 19
#define HORIZONTAL_ACTUATOR_HALL_PIN 21
#define HORIZONTAL_ACTUATOR_CURRENT_SENSE_PIN 20

static std_msgs__msg__String debug_message;
static rcl_publisher_t debug_publisher;

static rcl_publisher_t load_cell_publisher;
static rcl_timer_t load_cell_timer;
static std_msgs__msg__Float32 load_cell_message;

#ifdef HORIZONTAL_ACTUATOR
static actuator_t horizontal_actuator;
static rcl_publisher_t horizontal_actuator_state_publisher;
static rcl_subscription_t horizontal_actuator_command_subscription;
static soil_sampler_interfaces__msg__ActuatorCommand horizontal_actuator_command_message;
static soil_sampler_interfaces__msg__ActuatorState horizontal_actuator_state_message;
#endif

static actuator_t vertical_actuator;
static rcl_publisher_t vertical_actuator_state_publisher;
static rcl_subscription_t vertical_actuator_command_subscription;
static soil_sampler_interfaces__msg__ActuatorCommand vertical_actuator_command_message;
static soil_sampler_interfaces__msg__ActuatorState vertical_actuator_state_message;

static rcl_subscription_t calibration_subscription;
static std_msgs__msg__Bool calibration_message;

static rcl_timer_t actuator_position_timer;

// Calibration

typedef enum {
    CALIBRATION_IDLE = 0,
#ifdef HORIZONTAL_ACTUATOR
    CALIBRATION_VERTICAL_STAGGER,   // vertical retracting, waiting to start horizontal
#endif
    CALIBRATION_RETRACTING,         // all actuators retracting until the deadline
    CALIBRATION_EXTENDING,          // vertical extending briefly to release friction load
    CALIBRATION_LOAD_CELL           // actuators stopped, waiting to settle, then bias the load cell
} calibration_state_t;

static calibration_state_t calibration_state = CALIBRATION_IDLE;
static absolute_time_t calibration_deadline;

// Overload protection

typedef enum {
    OVERLOAD_IDLE = 0,
    OVERLOAD_RETRACTING             // vertical actuator backing off after exceeding MAX_LOAD_N
} overload_state_t;

static overload_state_t overload_state = OVERLOAD_IDLE;
static float overload_retract_target = 0.0f;
static absolute_time_t overload_deadline;

static bool overload_active(void) {
    return overload_state != OVERLOAD_IDLE;
}

static void fatal_error(void) {
#ifdef HORIZONTAL_ACTUATOR
    actuator_stop(&horizontal_actuator);
    actuator_enable(&horizontal_actuator, false);
#endif

    actuator_stop(&vertical_actuator);
    actuator_enable(&vertical_actuator, false);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}

static void publish_debug(const char *message) {
    if (!rosidl_runtime_c__String__assign(&debug_message.data, message)) {
        fatal_error();
    }

    rcl_ret_t ret = rcl_publish(&debug_publisher, &debug_message, NULL);
    if (ret != RCL_RET_OK) fatal_error();
}

static bool calibration_active(void) {
    return calibration_state != CALIBRATION_IDLE;
}

static void calibration_start(void) {
    if (calibration_active()) {
        publish_debug("Calibration already running, ignoring command");
        return;
    }

    if (overload_active()) {
        publish_debug("Overload retract in progress, ignoring calibration command");
        return;
    }

    publish_debug("Calibration started");
    publish_debug("Retracting vertical actuator");
    actuator_retract(&vertical_actuator);

#ifdef HORIZONTAL_ACTUATOR
    calibration_deadline = make_timeout_time_ms(CALIBRATION_STAGGER_MS);
    calibration_state = CALIBRATION_VERTICAL_STAGGER;
#else
    calibration_deadline = make_timeout_time_ms(CALIBRATION_TIME_MS);
    calibration_state = CALIBRATION_RETRACTING;
#endif
}

static void calibration_abort(void) {
    if (!calibration_active()) {
        return;
    }

    actuator_stop(&vertical_actuator);
#ifdef HORIZONTAL_ACTUATOR
    actuator_stop(&horizontal_actuator);
#endif

    calibration_state = CALIBRATION_IDLE;
    gpio_put(LED_PIN, 0);
    publish_debug("Calibration aborted");
}

// Call once per main-loop iteration. Never blocks (apart from the load cell bias routine in the final step)
static void calibration_update(void) {
    switch (calibration_state) {
    case CALIBRATION_IDLE:
        break;

#ifdef HORIZONTAL_ACTUATOR
    case CALIBRATION_VERTICAL_STAGGER:
        if (time_reached(calibration_deadline)) {
            publish_debug("Retracting horizontal actuator");
            actuator_retract(&horizontal_actuator);

            calibration_deadline = make_timeout_time_ms(CALIBRATION_TIME_MS);
            calibration_state = CALIBRATION_RETRACTING;
        }
        break;
#endif

    case CALIBRATION_RETRACTING:
        if (time_reached(calibration_deadline)) {
            actuator_stop(&vertical_actuator);
            actuator_hall_reset(&vertical_actuator);

#ifdef HORIZONTAL_ACTUATOR
            actuator_stop(&horizontal_actuator);
            actuator_hall_reset(&horizontal_actuator);
#endif

            publish_debug("Actuators stopped and positions reset");
            publish_debug("Extending vertical actuator to release friction load");

            actuator_extend(&vertical_actuator);

            calibration_deadline = make_timeout_time_ms(CALIBRATION_EXTEND_TIME_MS);
            calibration_state = CALIBRATION_EXTENDING;
        }
        break;

    case CALIBRATION_EXTENDING:
        if (time_reached(calibration_deadline)) {
            actuator_stop(&vertical_actuator);

            publish_debug("Calibrating load cell");

            calibration_deadline = make_timeout_time_ms(CALIBRATION_SETTLE_MS);
            calibration_state = CALIBRATION_LOAD_CELL;
        }
        break;

    case CALIBRATION_LOAD_CELL:
        if (time_reached(calibration_deadline)) {
            load_cell_calibrate_bias(CALIBRATION_LOAD_CELL_SAMPLES);

            publish_debug("Calibration complete");
            calibration_state = CALIBRATION_IDLE;
        }
        break;
    }
}

static void overload_trigger(float load) {
    calibration_abort();

    actuator_stop(&vertical_actuator);

    overload_retract_target = actuator_get_position(&vertical_actuator) - OVERLOAD_RETRACT_DISTANCE_M;
    overload_deadline = make_timeout_time_ms(OVERLOAD_RETRACT_TIMEOUT_MS);
    overload_state = OVERLOAD_RETRACTING;

    actuator_retract(&vertical_actuator);

    char message[64];
    snprintf(message, sizeof(message), "Max load exceeded: %ld N, retracting", (long)load);
    publish_debug(message);
}

static void overload_update(void) {
    if (overload_state != OVERLOAD_RETRACTING) {
        return;
    }

    bool reached = actuator_get_position(&vertical_actuator) <= overload_retract_target;
    bool timed_out = time_reached(overload_deadline);

    if (reached || timed_out) {
        actuator_stop(&vertical_actuator);
        gpio_put(LED_PIN, 0);
        overload_state = OVERLOAD_IDLE;

        publish_debug(reached ? "Overload retract complete" : "Overload retract timed out");
    }
}

// Timers

static void load_cell_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    if (!load_cell_available()) {
        return;
    }

    load_cell_update();
    float load = load_cell_read_newton();
    load_cell_message.data = load;

    if (!overload_active() && actuator_get_direction(&vertical_actuator) == ACTUATOR_EXTEND && load > MAX_LOAD_N) {
        overload_trigger(load);
    }

    rcl_ret_t ret = rcl_publish(&load_cell_publisher, &load_cell_message, NULL);
    if (ret != RCL_RET_OK) fatal_error();
}

static void actuator_position_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

#ifdef HORIZONTAL_ACTUATOR
    horizontal_actuator_state_message.current_direction = actuator_get_direction(&horizontal_actuator);
    horizontal_actuator_state_message.position = actuator_get_position(&horizontal_actuator);
    horizontal_actuator_state_message.enabled = actuator_is_enabled(&horizontal_actuator);
    horizontal_actuator_state_message.fault = actuator_fault_active(&horizontal_actuator);

    rcl_ret_t ret_horizontal = rcl_publish(&horizontal_actuator_state_publisher, &horizontal_actuator_state_message, NULL);
    if (ret_horizontal != RCL_RET_OK) fatal_error();
#endif

    vertical_actuator_state_message.current_direction = actuator_get_direction(&vertical_actuator);
    vertical_actuator_state_message.position = actuator_get_position(&vertical_actuator);
    vertical_actuator_state_message.enabled = actuator_is_enabled(&vertical_actuator);
    vertical_actuator_state_message.fault = actuator_fault_active(&vertical_actuator);

    rcl_ret_t ret_vertical = rcl_publish(&vertical_actuator_state_publisher, &vertical_actuator_state_message, NULL);
    if (ret_vertical != RCL_RET_OK) fatal_error();
}

// Subscription

#ifdef HORIZONTAL_ACTUATOR
static void horizontal_actuator_command_callback(const void *message) {
    const soil_sampler_interfaces__msg__ActuatorCommand *command = (const soil_sampler_interfaces__msg__ActuatorCommand *)message;

    if (calibration_active()) {
        return;
    }

    if (command->direction == ACTUATOR_EXTEND) {
        gpio_put(LED_PIN, 1);
        actuator_extend(&horizontal_actuator);
    } else if (command->direction == ACTUATOR_RETRACT) {
        gpio_put(LED_PIN, 1);
        actuator_retract(&horizontal_actuator);
    } else {
        gpio_put(LED_PIN, 0);
        actuator_stop(&horizontal_actuator);
    }
}
#endif

static void vertical_actuator_command_callback(const void *message) {
    const soil_sampler_interfaces__msg__ActuatorCommand *command = (const soil_sampler_interfaces__msg__ActuatorCommand *)message;

    if (calibration_active() || overload_active()) {
        return;
    }

    if (command->direction == ACTUATOR_EXTEND) {
        gpio_put(LED_PIN, 1);
        actuator_extend(&vertical_actuator);
    } else if (command->direction == ACTUATOR_RETRACT) {
        gpio_put(LED_PIN, 1);
        actuator_retract(&vertical_actuator);
    } else {
        gpio_put(LED_PIN, 0);
        actuator_stop(&vertical_actuator);
    }
}

static void calibration_callback(const void *message) {
    const std_msgs__msg__Bool *command = (const std_msgs__msg__Bool *)message;

    if (command->data) {
        publish_debug("Calibration command received: start");
        calibration_start();
    } else {
        publish_debug("Calibration command received: abort");
        calibration_abort();
    }
}

// Main

int main(void) {
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    if (!load_cell_init(10)) {
        fatal_error();
    }

#ifdef HORIZONTAL_ACTUATOR
    if (!actuator_init(&horizontal_actuator,
            HORIZONTAL_ACTUATOR_PWM_PIN,
            HORIZONTAL_ACTUATOR_DIR_PIN,
            HORIZONTAL_ACTUATOR_SLEEP_PIN,
            HORIZONTAL_ACTUATOR_FAULT_PIN,
            HORIZONTAL_ACTUATOR_HALL_PIN)) {
        fatal_error();
    }
#endif

    if (!actuator_init(&vertical_actuator,
            VERTICAL_ACTUATOR_PWM_PIN,
            VERTICAL_ACTUATOR_DIR_PIN,
            VERTICAL_ACTUATOR_SLEEP_PIN,
            VERTICAL_ACTUATOR_FAULT_PIN,
            VERTICAL_ACTUATOR_HALL_PIN)) {
        fatal_error();
    }

#ifdef HORIZONTAL_ACTUATOR
    actuator_stop(&horizontal_actuator);
#endif
    actuator_stop(&vertical_actuator);

    rcl_ret_t ret = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS);
    if (ret != RCL_RET_OK) fatal_error();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) fatal_error();

    rcl_node_t node;
    ret = rclc_node_init_default(&node, "soil_sampler_pico", "soil_sampler", &support);
    if (ret != RCL_RET_OK) fatal_error();

    if (!std_msgs__msg__String__init(&debug_message)) fatal_error();

    ret = rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "pico_debug");
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_publisher_init_default(&load_cell_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "load_cell_reading");
    if (ret != RCL_RET_OK) fatal_error();

#ifdef HORIZONTAL_ACTUATOR
    ret = rclc_publisher_init_default(&horizontal_actuator_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorState), "horizontal_actuator/state");
    if (ret != RCL_RET_OK) fatal_error();
#endif

    ret = rclc_publisher_init_default(&vertical_actuator_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorState), "vertical_actuator/state");
    if (ret != RCL_RET_OK) fatal_error();

#ifdef HORIZONTAL_ACTUATOR
    ret = rclc_subscription_init_default(&horizontal_actuator_command_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorCommand), "horizontal_actuator/command");
    if (ret != RCL_RET_OK) fatal_error();
#endif

    ret = rclc_subscription_init_default(&vertical_actuator_command_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorCommand), "vertical_actuator/command");
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_subscription_init_default(&calibration_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "calibration_command");
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_timer_init_default2(&load_cell_timer, &support, RCL_MS_TO_NS(LOAD_CELL_PUBLISH_PERIOD_MS), load_cell_timer_callback, true);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_timer_init_default2(&actuator_position_timer, &support, RCL_MS_TO_NS(ACTUATOR_POSITION_PUBLISH_PERIOD_MS), actuator_position_timer_callback, true);
    if (ret != RCL_RET_OK) fatal_error();

    rclc_executor_t executor;

#ifdef HORIZONTAL_ACTUATOR
    uint8_t number_of_handles = 5;
#else
    uint8_t number_of_handles = 4;
#endif
    ret = rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_timer(&executor, &load_cell_timer);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_timer(&executor, &actuator_position_timer);
    if (ret != RCL_RET_OK) fatal_error();

#ifdef HORIZONTAL_ACTUATOR
    ret = rclc_executor_add_subscription(&executor, &horizontal_actuator_command_subscription, &horizontal_actuator_command_message, &horizontal_actuator_command_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();
#endif

    ret = rclc_executor_add_subscription(&executor, &vertical_actuator_command_subscription, &vertical_actuator_command_message, &vertical_actuator_command_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_subscription(&executor, &calibration_subscription, &calibration_message, &calibration_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();

    load_cell_message.data = 0;

#ifdef HORIZONTAL_ACTUATOR
    horizontal_actuator_state_message.current_direction = ACTUATOR_STOP;
    horizontal_actuator_state_message.position = 0;
    horizontal_actuator_state_message.enabled = false;
    horizontal_actuator_state_message.fault = false;
#endif

    vertical_actuator_state_message.current_direction = ACTUATOR_STOP;
    vertical_actuator_state_message.position = 0;
    vertical_actuator_state_message.enabled = false;
    vertical_actuator_state_message.fault = false;

#ifdef HORIZONTAL_ACTUATOR
    horizontal_actuator_command_message.direction = ACTUATOR_STOP;
#endif
    vertical_actuator_command_message.direction = ACTUATOR_STOP;

    publish_debug("Pico initialized and ready");

    while (true) {
        ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (ret != RCL_RET_OK) fatal_error();

        calibration_update();
        overload_update();
    }

    return 0;
}
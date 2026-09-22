#include <stdbool.h>
#include <stdint.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

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

#define CALIBRATION_TIME_MS 10000

#define HORIZONTAL_ACTUATOR_PWM_PIN 14
#define HORIZONTAL_ACTUATOR_DIR_PIN 15
#define HORIZONTAL_ACTUATOR_SLEEP_PIN 13
#define HORIZONTAL_ACTUATOR_FAULT_PIN 12
#define HORIZONTAL_ACTUATOR_HALL_PIN 11

#define VERTICAL_ACTUATOR_PWM_PIN 16
#define VERTICAL_ACTUATOR_DIR_PIN 17
#define VERTICAL_ACTUATOR_SLEEP_PIN 18
#define VERTICAL_ACTUATOR_FAULT_PIN 19
#define VERTICAL_ACTUATOR_HALL_PIN 21


static rcl_publisher_t load_cell_publisher;
static rcl_timer_t load_cell_timer;
static std_msgs__msg__Float32 load_cell_message;

#ifdef horizontal_actuator
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

static void fatal_error(void) {
    #ifdef horizontal_actuator
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

static void calibrate(void) {
    #ifdef horizontal_actuator
    actuator_retract(&horizontal_actuator);
    #endif
    actuator_retract(&vertical_actuator);

    sleep_ms(CALIBRATION_TIME_MS);

    #ifdef horizontal_actuator
    actuator_stop(&horizontal_actuator);
    actuator_hall_reset(&horizontal_actuator);
    #endif
    
    actuator_stop(&vertical_actuator);
    actuator_hall_reset(&vertical_actuator);

    load_cell_calibrate_bias(100);
}

static void load_cell_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    if (!load_cell_available()) {
        return;
    }

    load_cell_update();
    load_cell_message.data = load_cell_read_newton();

    rcl_ret_t ret = rcl_publish(&load_cell_publisher, &load_cell_message, NULL);
    if (ret != RCL_RET_OK) fatal_error();
}

static void actuator_position_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    #ifdef horizontal_actuator
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

#ifdef horizontal_actuator
static void horizontal_actuator_command_callback(const void *message) {
    const soil_sampler_interfaces__msg__ActuatorCommand *command = (const soil_sampler_interfaces__msg__ActuatorCommand *)message;

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
        calibrate();
    }
}

int main(void) {
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    if (!load_cell_init(10)) {
        fatal_error();
    }
    #ifdef horizontal_actuator
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
    #ifdef horizontal_actuator
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

    ret = rclc_publisher_init_default(&load_cell_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "load_cell_reading");
    if (ret != RCL_RET_OK) fatal_error();

    #ifdef horizontal_actuator
    ret = rclc_publisher_init_default(&horizontal_actuator_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorState), "horizontal_actuator/state");
    if (ret != RCL_RET_OK) fatal_error();
    #endif

    ret = rclc_publisher_init_default(&vertical_actuator_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorState), "vertical_actuator/state");
    if (ret != RCL_RET_OK) fatal_error();

    #ifdef horizontal_actuator
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

    uint8_t number_of_handles = 5;
    ret = rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_timer(&executor, &load_cell_timer);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_timer(&executor, &actuator_position_timer);
    if (ret != RCL_RET_OK) fatal_error();

    #ifdef horizontal_actuator
    ret = rclc_executor_add_subscription(&executor, &horizontal_actuator_command_subscription, &horizontal_actuator_command_message, &horizontal_actuator_command_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();
    #endif

    ret = rclc_executor_add_subscription(&executor, &vertical_actuator_command_subscription, &vertical_actuator_command_message, &vertical_actuator_command_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();

    ret = rclc_executor_add_subscription(&executor, &calibration_subscription, &calibration_message, &calibration_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) fatal_error();

    load_cell_message.data = 0;

    #ifdef horizontal_actuator
    horizontal_actuator_state_message.current_direction = ACTUATOR_STOP;
    horizontal_actuator_state_message.position = 0;
    horizontal_actuator_state_message.enabled = false;
    horizontal_actuator_state_message.fault = false;
    #endif

    vertical_actuator_state_message.current_direction = ACTUATOR_STOP;
    vertical_actuator_state_message.position = 0;
    vertical_actuator_state_message.enabled = false;
    vertical_actuator_state_message.fault = false;

    #ifdef horizontal_actuator
    horizontal_actuator_command_message.direction = ACTUATOR_STOP;
    #endif
    vertical_actuator_command_message.direction = ACTUATOR_STOP;

    while (true) {
        ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (ret != RCL_RET_OK) fatal_error();
    }

    return 0;
}

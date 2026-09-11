#include <stdbool.h>
#include <stdint.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

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

static rcl_publisher_t load_cell_publisher;
static rcl_timer_t load_cell_timer;
static std_msgs__msg__Float32 load_cell_message;

static rcl_subscription_t actuator_command_subscription;
static soil_sampler_interfaces__msg__ActuatorCommand actuator_command_message;

static rcl_subscription_t calibration_subscription;
static std_msgs__msg__Bool calibration_message;

static rcl_publisher_t actuator_position_publisher;
static rcl_timer_t actuator_position_timer;
static soil_sampler_interfaces__msg__ActuatorState actuator_position_message;

static void fatal_error(void) {
    actuator_stop();
    actuator_enable(false);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}

static void calibrate(void){
    actuator_retract();
    while(load_cell_read_newton() > -2){
        load_cell_update();
        sleep_ms(10);
    }
    actuator_stop();
    sleep_ms(500);
    actuator_hall_reset();
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

    if (ret != RCL_RET_OK) {
        fatal_error();
    }
}

static void actuator_position_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    actuator_position_message.current_direction = actuator_get_direction();
    actuator_position_message.position = actuator_get_position();
    actuator_position_message.enabled = actuator_is_enabled();
    actuator_position_message.fault = actuator_fault_active();

    rcl_ret_t ret = rcl_publish(&actuator_position_publisher, &actuator_position_message, NULL);

    if (ret != RCL_RET_OK) {
        fatal_error();
    }
}

static void actuator_command_callback(const void *message) {
    const soil_sampler_interfaces__msg__ActuatorCommand *command = (const soil_sampler_interfaces__msg__ActuatorCommand *)message;

    // if (actuator_fault_active()) {
    //     actuator_stop();
    //     return;
    // }

    gpio_put(LED_PIN, 1);
    if (command->direction == ACTUATOR_EXTEND) {
        gpio_put(LED_PIN, 1);
        actuator_extend();
    } else if (command->direction == ACTUATOR_RETRACT) {
        gpio_put(LED_PIN, 1);
        actuator_retract();
    } else {
        gpio_put(LED_PIN, 0);
        actuator_stop();
    }
}

static void calibration_callback(const void *message) {
    const std_msgs__msg__Bool *command = (const std_msgs__msg__Bool *)message;
    if(command->data){
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

    if (!actuator_init()) {
        fatal_error();
    }

    rcl_ret_t ret = rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    rcl_node_t node;
    ret = rclc_node_init_default(&node, "soil_sampler_pico", "soil_sampler", &support);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    // Initialize publishers
    ret = rclc_publisher_init_default(&load_cell_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "load_cell_reading");
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_publisher_init_default(&actuator_position_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorState), "actuator_state");
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    // Initialize subscribers
    ret = rclc_subscription_init_default(&actuator_command_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(soil_sampler_interfaces, msg, ActuatorCommand), "actuator_command");
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_subscription_init_default(&calibration_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "calibration_command");
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    // Initialize timers
    ret = rclc_timer_init_default2(&load_cell_timer, &support, RCL_MS_TO_NS(LOAD_CELL_PUBLISH_PERIOD_MS), load_cell_timer_callback, true);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_timer_init_default2(&actuator_position_timer, &support, RCL_MS_TO_NS(ACTUATOR_POSITION_PUBLISH_PERIOD_MS), actuator_position_timer_callback, true);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    // Initialize executor
    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_executor_add_timer(&executor, &load_cell_timer);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_executor_add_timer(&executor, &actuator_position_timer);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_executor_add_subscription(&executor, &actuator_command_subscription, &actuator_command_message, &actuator_command_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    ret = rclc_executor_add_subscription(&executor, &calibration_subscription, &calibration_message, &calibration_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        fatal_error();
    }

    load_cell_message.data = 0;
    actuator_position_message.current_direction = ACTUATOR_STOP;
    actuator_position_message.position = 0;
    actuator_position_message.enabled = false;
    actuator_position_message.fault = false;
    actuator_command_message.direction = ACTUATOR_STOP;

    actuator_stop();

    while (true) {
        ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        if (ret != RCL_RET_OK) {
            fatal_error();
        }

        float max_force = 100.0;
        if (load_cell_available()) {
            float average = load_cell_read_newton();
            if (average > max_force || average < -max_force) {
                actuator_stop();
            }
        }
    }

    return 0;
}
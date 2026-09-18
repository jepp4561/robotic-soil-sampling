#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Temperature
from std_msgs.msg import Bool, Int32, UInt32, Float32, Float64MultiArray

from soil_sampler_interfaces.msg import ActuatorCommand, ActuatorState


EXTEND = 1
STOP = 0
RETRACT = -1


class SimulatedSoilSamplerPico(Node):

    def __init__(self):
        super().__init__("soil_sampler_pico")

        self.declare_parameter("horizontal_joint_name", "soil_sampler_horizontal_slider")
        self.declare_parameter("vertical_joint_name", "soil_sampler_vertical_slider")
        self.declare_parameter("horizontal_lower_limit", 0.0)
        self.declare_parameter("horizontal_upper_limit", 0.3)
        self.declare_parameter("vertical_lower_limit", 0.0)
        self.declare_parameter("vertical_upper_limit", 0.3)
        self.declare_parameter("horizontal_velocity", 0.01)
        self.declare_parameter("vertical_velocity", 0.01)
        self.declare_parameter("calibration_tolerance", 0.002)
        self.declare_parameter("state_publish_rate", 10.0)

        self.horizontal_joint_name = self.get_parameter("horizontal_joint_name").value
        self.vertical_joint_name = self.get_parameter("vertical_joint_name").value

        self.horizontal_lower_limit = float(self.get_parameter("horizontal_lower_limit").value)
        self.horizontal_upper_limit = float(self.get_parameter("horizontal_upper_limit").value)
        self.vertical_lower_limit = float(self.get_parameter("vertical_lower_limit").value)
        self.vertical_upper_limit = float(self.get_parameter("vertical_upper_limit").value)

        self.horizontal_velocity = abs(float(self.get_parameter("horizontal_velocity").value))
        self.vertical_velocity = abs(float(self.get_parameter("vertical_velocity").value))

        self.calibration_tolerance = abs(float(self.get_parameter("calibration_tolerance").value))

        state_publish_rate = float(self.get_parameter("state_publish_rate").value)

        self.horizontal_direction = STOP
        self.vertical_direction = STOP

        self.horizontal_enabled = True
        self.vertical_enabled = True

        self.horizontal_fault = False
        self.vertical_fault = False

        self.horizontal_position = self.horizontal_lower_limit
        self.vertical_position = self.vertical_lower_limit

        self.have_horizontal_position = False
        self.have_vertical_position = False

        self.calibrating = False
        self._warned_missing_horizontal_joint = False
        self._warned_missing_vertical_joint = False

        self.horizontal_command_sub = self.create_subscription(ActuatorCommand, "horizontal_actuator/command", self.horizontal_command_callback, 10)
        self.vertical_command_sub = self.create_subscription(ActuatorCommand, "vertical_actuator/command", self.vertical_command_callback, 10)
        self.calibration_sub = self.create_subscription(Bool, "calibration_command", self.calibration_callback, 10)
        self.horizontal_state_pub = self.create_publisher(ActuatorState, "horizontal_actuator/state", 10)
        self.vertical_state_pub = self.create_publisher(ActuatorState, "vertical_actuator/state", 10)
        self.load_cell_pub = self.create_publisher(Float32, "load_cell_reading", 10)
        self.vwc_pub = self.create_publisher(Float32, "teros12/volumetric_water_content", 10)
        self.ec_pub = self.create_publisher(Float32, "teros12/electrical_conductivity", 10)
        self.soil_temperature_pub = self.create_publisher(Temperature, "teros12/temperature", 10)
        self.wind_speed_pub = self.create_publisher(Float32, "sen0658/wind_speed", 10)
        self.wind_direction_gear_pub = self.create_publisher(Int32, "sen0658/wind_direction_gear", 10)
        self.wind_direction_pub = self.create_publisher(Float32, "sen0658/wind_direction", 10)
        self.humidity_pub = self.create_publisher(Float32, "sen0658/humidity", 10)
        self.air_temperature_pub = self.create_publisher(Float32, "sen0658/temperature", 10)
        self.noise_pub = self.create_publisher(Float32, "sen0658/noise", 10)
        self.pm2_5_pub = self.create_publisher(Float32, "sen0658/pm2_5", 10)
        self.pm10_pub = self.create_publisher(Float32, "sen0658/pm10", 10)
        self.pressure_pub = self.create_publisher(Float32, "sen0658/pressure", 10)
        self.illumination_pub = self.create_publisher(UInt32, "sen0658/illumination", 10)
        self.rainfall_pub = self.create_publisher(Float32, "sen0658/rainfall", 10)
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.velocity_command_pub = self.create_publisher(Float64MultiArray, "velocity_command", 10)

        self.timer = self.create_timer(
            1.0 / state_publish_rate,
            self.publish_state,
        )

        self.get_logger().info(
            "Simulated soil_sampler_pico started."
        )

    def horizontal_command_callback(
        self,
        msg: ActuatorCommand,
    ):
        if self.calibrating:
            return

        if msg.direction == EXTEND:
            self.horizontal_direction = EXTEND
            self._publish_velocity()

        elif msg.direction == RETRACT:
            if self._horizontal_at_lower_limit():
                self._stop_horizontal_actuator()
                return

            self.horizontal_direction = RETRACT
            self._publish_velocity()

        else:
            self._stop_horizontal_actuator()

    def vertical_command_callback(
        self,
        msg: ActuatorCommand,
    ):
        if self.calibrating:
            return

        if msg.direction == EXTEND:
            self.vertical_direction = EXTEND
            self._publish_velocity()

        elif msg.direction == RETRACT:
            if self._vertical_at_lower_limit():
                self._stop_vertical_actuator()
                return

            self.vertical_direction = RETRACT
            self._publish_velocity()

        else:
            self._stop_vertical_actuator()

    def calibration_callback(self, msg: Bool):
        if not msg.data or self.calibrating:
            return

        self.calibrating = True

        self.horizontal_direction = RETRACT
        self.vertical_direction = RETRACT

        self._publish_velocity()

        self.get_logger().info(
            "Calibration started: retracting actuators"
        )

    def _publish_velocity(self):
        horizontal_velocity = 0.0
        vertical_velocity = 0.0

        if self.horizontal_direction == EXTEND:
            horizontal_velocity = self.horizontal_velocity
        elif self.horizontal_direction == RETRACT:
            horizontal_velocity = -self.horizontal_velocity

        if self.vertical_direction == EXTEND:
            vertical_velocity = self.vertical_velocity
        elif self.vertical_direction == RETRACT:
            vertical_velocity = -self.vertical_velocity

        msg = Float64MultiArray()
        msg.data = [
            horizontal_velocity,
            vertical_velocity,
        ]

        self.velocity_command_pub.publish(msg)

    def _stop_horizontal_actuator(self):
        self.horizontal_direction = STOP
        self._publish_velocity()

    def _stop_vertical_actuator(self):
        self.vertical_direction = STOP
        self._publish_velocity()

    def _stop_all_actuators(self):
        self.horizontal_direction = STOP
        self.vertical_direction = STOP
        self._publish_velocity()

    def _horizontal_at_lower_limit(self):
        return self.horizontal_position <= self.horizontal_lower_limit + self.calibration_tolerance

    def _vertical_at_lower_limit(self):
        return self.vertical_position <= self.vertical_lower_limit + self.calibration_tolerance

    def _horizontal_at_upper_limit(self):
        return self.horizontal_position >= self.horizontal_upper_limit - self.calibration_tolerance

    def _vertical_at_upper_limit(self):
        return self.vertical_position >= self.vertical_upper_limit - self.calibration_tolerance

    def joint_state_callback(self, msg: JointState):
        horizontal_found = (
            self.horizontal_joint_name in msg.name
        )

        vertical_found = (
            self.vertical_joint_name in msg.name
        )

        if not horizontal_found:
            if not self._warned_missing_horizontal_joint:
                self.get_logger().warn(
                    f"Joint '{self.horizontal_joint_name}' "
                    f"not found in incoming JointState "
                    f"(names: {list(msg.name)})."
                )
                self._warned_missing_horizontal_joint = True
        else:
            idx = msg.name.index(self.horizontal_joint_name)
            self.horizontal_position = msg.position[idx]
            self.have_horizontal_position = True

        if not vertical_found:
            if not self._warned_missing_vertical_joint:
                self.get_logger().warn(
                    f"Joint '{self.vertical_joint_name}' "
                    f"not found in incoming JointState "
                    f"(names: {list(msg.name)})."
                )
                self._warned_missing_vertical_joint = True
        else:
            idx = msg.name.index(self.vertical_joint_name)
            self.vertical_position = msg.position[idx]
            self.have_vertical_position = True

        if self.horizontal_direction == RETRACT:
            if self._horizontal_at_lower_limit():
                self.horizontal_position = self.horizontal_lower_limit
                self.horizontal_direction = STOP

        elif self.horizontal_direction == EXTEND:
            if self._horizontal_at_upper_limit():
                self.horizontal_position = self.horizontal_upper_limit
                self.horizontal_direction = STOP

        if self.vertical_direction == RETRACT:
            if self._vertical_at_lower_limit():
                self.vertical_position = self.vertical_lower_limit
                self.vertical_direction = STOP

        elif self.vertical_direction == EXTEND:
            if self._vertical_at_upper_limit():
                self.vertical_position = self.vertical_upper_limit
                self.vertical_direction = STOP

        if self.calibrating:
            if (
                self._horizontal_at_lower_limit()
                and self._vertical_at_lower_limit()
            ):
                self.calibrating = False
                self._stop_all_actuators()
                self.get_logger().info(
                    "Calibration complete"
                )

    def publish_state(self):
        horizontal_state = ActuatorState()
        horizontal_state.current_direction = self.horizontal_direction
        horizontal_state.position = self.horizontal_position
        horizontal_state.enabled = self.horizontal_enabled
        horizontal_state.fault = self.horizontal_fault

        self.horizontal_state_pub.publish(horizontal_state)

        vertical_state = ActuatorState()
        vertical_state.current_direction = self.vertical_direction
        vertical_state.position = self.vertical_position
        vertical_state.enabled = self.vertical_enabled
        vertical_state.fault = self.vertical_fault

        self.vertical_state_pub.publish(vertical_state)

        force_msg = Float32()
        force_msg.data = random.gauss(0.0, 0.4)
        self.load_cell_pub.publish(force_msg)

        vwc_msg = Float32()
        vwc_msg.data = random.gauss(30.0, 20.0)
        self.vwc_pub.publish(vwc_msg)

        ec_msg = Float32()
        ec_msg.data = random.gauss(4.0, 1.0)
        self.ec_pub.publish(ec_msg)

        soil_temperature_msg = Temperature()
        soil_temperature_msg.header.stamp = self.get_clock().now().to_msg()
        soil_temperature_msg.header.frame_id = "virtual_soil_sensor"
        soil_temperature_msg.temperature = random.gauss(20.0, 5.0)
        self.soil_temperature_pub.publish(soil_temperature_msg)

        wind_speed_msg = Float32()
        wind_speed_msg.data = random.gauss(5.0, 2.0)
        self.wind_speed_pub.publish(wind_speed_msg)

        wind_direction_gear_msg = Int32()
        wind_direction_gear_msg.data = round(random.gauss(5.0, 2.0))
        self.wind_direction_gear_pub.publish(wind_direction_gear_msg)

        wind_direction_msg = Float32()
        wind_direction_msg.data = random.gauss(5.0, 2.0)
        self.wind_direction_pub.publish(wind_direction_msg)

        humidity_msg = Float32()
        humidity_msg.data = random.gauss(5.0, 2.0)
        self.humidity_pub.publish(humidity_msg)

        air_temperature_msg = Float32()
        air_temperature_msg.data = random.gauss(5.0, 2.0)
        self.air_temperature_pub.publish(air_temperature_msg)

        noise_msg = Float32()
        noise_msg.data = random.gauss(5.0, 2.0)
        self.noise_pub.publish(noise_msg)

        pm2_5_msg = Float32()
        pm2_5_msg.data = random.gauss(5.0, 2.0)
        self.pm2_5_pub.publish(pm2_5_msg)

        pm10_msg = Float32()
        pm10_msg.data = random.gauss(5.0, 2.0)
        self.pm10_pub.publish(pm10_msg)

        pressure_msg = Float32()
        pressure_msg.data = random.gauss(5.0, 2.0)
        self.pressure_pub.publish(pressure_msg)

        illumination_msg = UInt32()
        illumination_msg.data = max(0, round(random.gauss(5.0, 2.0)))
        self.illumination_pub.publish(illumination_msg)

        rainfall_msg = Float32()
        rainfall_msg.data = random.gauss(5.0, 2.0)
        self.rainfall_pub.publish(rainfall_msg)

    def destroy_node(self):
        self._stop_all_actuators()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedSoilSamplerPico()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray
from soil_sampler_interfaces.msg import ActuatorCommand, ActuatorState


EXTEND = 1
STOP = 0
RETRACT = -1


class SimulatedSoilSamplerPico(Node):
    def __init__(self):
        super().__init__("soil_sampler_pico")

        self.declare_parameter("joint_name", "soil_sampler_slider_1")
        self.declare_parameter("lower_limit", 0.0)
        self.declare_parameter("upper_limit", 0.2)
        self.declare_parameter("velocity", 0.01)
        self.declare_parameter("calibration_tolerance", 0.002)
        self.declare_parameter("load_cell_mean", 0.0)
        self.declare_parameter("load_cell_stddev", 0.5)
        self.declare_parameter("state_publish_rate", 10.0)

        self.joint_name = self.get_parameter("joint_name").value
        self.lower_limit = self.get_parameter("lower_limit").value
        self.upper_limit = self.get_parameter("upper_limit").value
        self.velocity = abs(self.get_parameter("velocity").value)
        self.calibration_tolerance = abs(
            self.get_parameter("calibration_tolerance").value
        )
        self.load_cell_mean = self.get_parameter("load_cell_mean").value
        self.load_cell_stddev = self.get_parameter("load_cell_stddev").value
        state_publish_rate = self.get_parameter("state_publish_rate").value

        self.current_direction = STOP
        self.enabled = True
        self.fault = False
        self.measured_position = self.lower_limit
        self.have_position = False
        self.calibrating = False

        self._warned_missing_joint = False

        self.command_sub = self.create_subscription(
            ActuatorCommand,
            "actuator_command",
            self.command_callback,
            10,
        )

        self.calibration_sub = self.create_subscription(
            Bool,
            "calibration_command",
            self.calibration_callback,
            10,
        )

        self.state_pub = self.create_publisher(
            ActuatorState,
            "actuator_state",
            10,
        )

        self.load_cell_pub = self.create_publisher(
            Float32,
            "load_cell_reading",
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        self.velocity_command_pub = self.create_publisher(
            Float64MultiArray,
            "velocity_command",
            10,
        )

        self.timer = self.create_timer(
            1.0 / state_publish_rate,
            self.publish_state,
        )

        self.get_logger().info(
            "Simulated soil_sampler_pico started."
        )

    def command_callback(self, msg: ActuatorCommand):
        if self.calibrating:
            return

        if msg.direction == EXTEND:
            self.current_direction = EXTEND
            self._publish_velocity(self.velocity)

        elif msg.direction == RETRACT:
            if self._at_lower_limit():
                self._stop_actuator()
                return

            self.current_direction = RETRACT
            self._publish_velocity(-self.velocity)

        else:
            self._stop_actuator()

    def calibration_callback(self, msg: Bool):
        if not msg.data or self.calibrating:
            return

        self.calibrating = True
        self.current_direction = RETRACT

        self._publish_velocity(-self.velocity)

        self.get_logger().info(
            "Calibration started: retracting actuator"
        )

    def _publish_velocity(self, velocity):
        msg = Float64MultiArray()
        msg.data = [velocity]
        self.velocity_command_pub.publish(msg)

    def _stop_actuator(self):
        self.current_direction = STOP
        self._publish_velocity(0.0)

    def _at_lower_limit(self):
        return self.measured_position <= (
            self.lower_limit + self.calibration_tolerance
        )

    def joint_state_callback(self, msg: JointState):
        if self.joint_name not in msg.name:
            if not self._warned_missing_joint:
                self.get_logger().warn(
                    f"Joint '{self.joint_name}' not found in incoming "
                    f"JointState (names: {list(msg.name)})."
                )
                self._warned_missing_joint = True
            return

        idx = msg.name.index(self.joint_name)
        self.measured_position = msg.position[idx]
        self.have_position = True

        if self.current_direction == RETRACT and self._at_lower_limit():
            if self.calibrating:
                self.calibrating = False

                self._stop_actuator()

                self.get_logger().info(
                    "Calibration complete"
                )
            else:
                self._stop_actuator()

    def publish_state(self):
        state_msg = ActuatorState()
        state_msg.current_direction = self.current_direction
        state_msg.position = self.measured_position
        state_msg.enabled = self.enabled
        state_msg.fault = self.fault

        self.state_pub.publish(state_msg)

        force_msg = Float32()
        force_msg.data = random.gauss(self.load_cell_mean, self.load_cell_stddev if self.load_cell_stddev > 0 else self.load_cell_mean)

        self.load_cell_pub.publish(force_msg)

    def destroy_node(self):
        self._publish_velocity(0.0)
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

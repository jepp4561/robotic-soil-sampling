#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, UInt32, Float32, Float64MultiArray
from sensor_msgs.msg import Temperature
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
        self.declare_parameter("state_publish_rate", 10.0)

        self.joint_name = self.get_parameter("joint_name").value
        self.lower_limit = self.get_parameter("lower_limit").value
        self.upper_limit = self.get_parameter("upper_limit").value
        self.velocity = abs(self.get_parameter("velocity").value)
        self.calibration_tolerance = abs(self.get_parameter("calibration_tolerance").value)
        state_publish_rate = self.get_parameter("state_publish_rate").value

        self.current_direction = STOP
        self.enabled = True
        self.fault = False
        self.measured_position = self.lower_limit
        self.have_position = False
        self.calibrating = False
        self._warned_missing_joint = False

        self.command_sub = self.create_subscription(ActuatorCommand, "actuator_command", self.command_callback, 10)
        self.calibration_sub = self.create_subscription(Bool, "calibration_command", self.calibration_callback, 10)
        self.state_pub = self.create_publisher(ActuatorState, "actuator_state", 10)

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
        self.timer = self.create_timer(1.0 / state_publish_rate, self.publish_state)

        self.get_logger().info("Simulated soil_sampler_pico started.")

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

        self.get_logger().info("Calibration started: retracting actuator")

    def _publish_velocity(self, velocity):
        msg = Float64MultiArray()
        msg.data = [velocity]
        self.velocity_command_pub.publish(msg)

    def _stop_actuator(self):
        self.current_direction = STOP
        self._publish_velocity(0.0)

    def _at_lower_limit(self):
        return self.measured_position <= self.lower_limit

    def joint_state_callback(self, msg: JointState):
        if self.joint_name not in msg.name:
            if not self._warned_missing_joint:
                self.get_logger().warn(f"Joint '{self.joint_name}' not found in incoming " f"JointState (names: {list(msg.name)}).")
                self._warned_missing_joint = True
            return

        idx = msg.name.index(self.joint_name)
        self.measured_position = msg.position[idx]
        self.have_position = True

        if self.current_direction == RETRACT and self._at_lower_limit():
            if self.calibrating:
                self.calibrating = False

                self._stop_actuator()

                self.get_logger().info("Calibration complete")
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
        force_msg.data = random.gauss(0.0, 0.4)
        self.load_cell_pub.publish(force_msg)

        vwc_msg = Float32()
        vwc_msg.data = random.gauss(30.0, 20.0)
        self.vwc_pub.publish(vwc_msg)

        ec_msg = Float32()
        ec_msg.data = random.gauss(4.0, 1.0)
        self.ec_pub.publish(ec_msg)

        temperature_msg = Temperature()
        temperature_msg.header.stamp = self.get_clock().now().to_msg()
        temperature_msg.header.frame_id = "virtual_soil_sensor"
        temperature_msg.temperature = random.gauss(20.0, 5.0)
        self.soil_temperature_pub.publish(temperature_msg)

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
        temperature_msg = Float32()
        temperature_msg.data = random.gauss(5.0, 2.0)
        self.air_temperature_pub.publish(temperature_msg)
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

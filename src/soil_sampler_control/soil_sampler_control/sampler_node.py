import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt32

from soil_sampler_interfaces.action import TakeSoilSample
from soil_sampler_interfaces.msg import ActuatorCommand
from soil_sampler_interfaces.msg import ActuatorState
from soil_sampler_interfaces.msg import SoilSample

from .sampler_state_machine import SamplerState
from .sampler_state_machine import SamplerStateMachine
from .sampler_statistics import calculate_circular_statistics
from .sampler_statistics import calculate_mode
from .sampler_statistics import calculate_statistics


EXTEND = 1
STOP = 0
RETRACT = -1


class SoilSamplerNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler")

        self.declare_parameter("maximum_depth", 0.3)
        self.declare_parameter("maximum_horizontal_distance", 0.25)
        self.declare_parameter("maximum_force", 50.0)
        self.declare_parameter("insertion_timeout", 60.0)
        self.declare_parameter("retraction_timeout", 60.0)
        self.declare_parameter("horizontal_extension_timeout", 60.0)
        self.declare_parameter("horizontal_retraction_timeout", 60.0)
        self.declare_parameter("measurement_timeout", 5.0)
        self.declare_parameter("position_threshold", 0.001)

        self.maximum_depth = float(self.get_parameter("maximum_depth").value)
        self.maximum_horizontal_distance = float(self.get_parameter("maximum_horizontal_distance").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.insertion_timeout = float(self.get_parameter("insertion_timeout").value)
        self.retraction_timeout = float(self.get_parameter("retraction_timeout").value)
        self.horizontal_extension_timeout = float(self.get_parameter("horizontal_extension_timeout").value)
        self.horizontal_retraction_timeout = float(self.get_parameter("horizontal_retraction_timeout").value)
        self.measurement_timeout = float(self.get_parameter("measurement_timeout").value)
        self.position_threshold = float(self.get_parameter("position_threshold").value)

        # temporary parameter to enable/disable horizontal actuator for testing purposes
        self.declare_parameter("horizontal_actuator_enabled", True)
        self.horizontal_actuator_enabled_parameter = bool(self.get_parameter("horizontal_actuator_enabled").value)

        self.state_machine = SamplerStateMachine(maximum_depth=self.maximum_depth, maximum_force=self.maximum_force)
        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, TakeSoilSample, "take_sample", self.execute_sample, callback_group=self.callback_group)

        if self.horizontal_actuator_enabled_parameter:
            self.horizontal_actuator_command_publisher = self.create_publisher(ActuatorCommand, "horizontal_actuator/command", 10)
            self.horizontal_actuator_state_subscription = self.create_subscription(ActuatorState, "horizontal_actuator/state", self.horizontal_actuator_state_callback, 10, callback_group=self.callback_group)

        self.vertical_actuator_command_publisher = self.create_publisher(ActuatorCommand, "vertical_actuator/command", 10)
        self.vertical_actuator_state_subscription = self.create_subscription(ActuatorState, "vertical_actuator/state", self.vertical_actuator_state_callback, 10, callback_group=self.callback_group)
        self.force_subscription = self.create_subscription(Float32, "load_cell_reading", self.force_callback, 10, callback_group=self.callback_group)
        self.vwc_subscription = self.create_subscription(Float32, "teros12/volumetric_water_content", self.vwc_callback, 10, callback_group=self.callback_group)
        self.temperature_subscription = self.create_subscription(Temperature, "teros12/temperature", self.temperature_callback, 10, callback_group=self.callback_group)
        self.ec_subscription = self.create_subscription(Float32, "teros12/electrical_conductivity", self.ec_callback, 10, callback_group=self.callback_group)
        self.wind_speed_subscription = self.create_subscription(Float32, "sen0658/wind_speed", self.wind_speed_callback, 10, callback_group=self.callback_group)
        self.wind_direction_gear_subscription = self.create_subscription(Int32, "sen0658/wind_direction_gear", self.wind_direction_gear_callback, 10, callback_group=self.callback_group)
        self.wind_direction_subscription = self.create_subscription(Float32, "sen0658/wind_direction", self.wind_direction_callback, 10, callback_group=self.callback_group)
        self.humidity_subscription = self.create_subscription(Float32, "sen0658/humidity", self.humidity_callback, 10, callback_group=self.callback_group)
        self.air_temperature_subscription = self.create_subscription(Float32, "sen0658/temperature", self.air_temperature_callback, 10, callback_group=self.callback_group)
        self.noise_subscription = self.create_subscription(Float32, "sen0658/noise", self.noise_callback, 10, callback_group=self.callback_group)
        self.pm2_5_subscription = self.create_subscription(Float32, "sen0658/pm2_5", self.pm2_5_callback, 10, callback_group=self.callback_group)
        self.pm10_subscription = self.create_subscription(Float32, "sen0658/pm10", self.pm10_callback, 10, callback_group=self.callback_group)
        self.pressure_subscription = self.create_subscription(Float32, "sen0658/pressure", self.pressure_callback, 10, callback_group=self.callback_group)
        self.illumination_subscription = self.create_subscription(UInt32, "sen0658/illumination", self.illumination_callback, 10, callback_group=self.callback_group)
        self.rainfall_subscription = self.create_subscription(Float32, "sen0658/rainfall", self.rainfall_callback, 10, callback_group=self.callback_group)

        self.vertical_actuator_state: ActuatorState | None = None
        self.horizontal_actuator_state: ActuatorState | None = None
        self.latest_force: float | None = None
        self.latest_vwc: float | None = None
        self.latest_temperature: float | None = None
        self.latest_ec: float | None = None
        self.latest_wind_speed: float | None = None
        self.latest_wind_direction_gear: int | None = None
        self.latest_wind_direction: float | None = None
        self.latest_humidity: float | None = None
        self.latest_air_temperature: float | None = None
        self.latest_noise: float | None = None
        self.latest_pm2_5: float | None = None
        self.latest_pm10: float | None = None
        self.latest_pressure: float | None = None
        self.latest_illumination: int | None = None
        self.latest_rainfall: float | None = None

        self.sampling_active = False

        self.get_logger().info("Soil sampler control node started.")

    def vertical_actuator_state_callback(self, message: ActuatorState) -> None:
        self.vertical_actuator_state = message
        force = self.latest_force if self.latest_force is not None else 0.0
        self.state_machine.update_actuator_measurement(depth=float(message.position), force=force)

    def horizontal_actuator_state_callback(self, message: ActuatorState) -> None:
        self.horizontal_actuator_state = message
        self.state_machine.update_horizontal_position(float(message.position))

    def force_callback(self, message: Float32) -> None:
        self.latest_force = float(message.data)
        depth = float(self.vertical_actuator_state.position) if self.vertical_actuator_state is not None else 0.0
        self.state_machine.update_actuator_measurement(depth=depth, force=self.latest_force)

    def vwc_callback(self, message: Float32) -> None:
        self.latest_vwc = float(message.data)

    def temperature_callback(self, message: Temperature) -> None:
        self.latest_temperature = float(message.temperature)

    def ec_callback(self, message: Float32) -> None:
        self.latest_ec = float(message.data)

    def wind_speed_callback(self, message: Float32) -> None:
        self.latest_wind_speed = float(message.data)

    def wind_direction_gear_callback(self, message: Int32) -> None:
        self.latest_wind_direction_gear = int(message.data)

    def wind_direction_callback(self, message: Float32) -> None:
        self.latest_wind_direction = float(message.data)

    def humidity_callback(self, message: Float32) -> None:
        self.latest_humidity = float(message.data)

    def air_temperature_callback(self, message: Float32) -> None:
        self.latest_air_temperature = float(message.data)

    def noise_callback(self, message: Float32) -> None:
        self.latest_noise = float(message.data)

    def pm2_5_callback(self, message: Float32) -> None:
        self.latest_pm2_5 = float(message.data)

    def pm10_callback(self, message: Float32) -> None:
        self.latest_pm10 = float(message.data)

    def pressure_callback(self, message: Float32) -> None:
        self.latest_pressure = float(message.data)

    def illumination_callback(self, message: UInt32) -> None:
        self.latest_illumination = int(message.data)

    def rainfall_callback(self, message: Float32) -> None:
        self.latest_rainfall = float(message.data)

    def publish_vertical_command(self, direction: int) -> None:
        command = ActuatorCommand()
        command.direction = direction
        self.vertical_actuator_command_publisher.publish(command)

    def publish_horizontal_command(self, direction: int) -> None:
        command = ActuatorCommand()
        command.direction = direction
        self.horizontal_actuator_command_publisher.publish(command)

    def stop_vertical(self) -> None:
        self.publish_vertical_command(STOP)

    def stop_horizontal(self) -> None:
        self.publish_horizontal_command(STOP)

    def stop_all_actuators(self) -> None:
        self.stop_vertical()
        if self.horizontal_actuator_enabled_parameter:
            self.stop_horizontal()

    def vertical_position(self) -> float | None:
        if self.vertical_actuator_state is None:
            return None
        return float(self.vertical_actuator_state.position)

    def horizontal_position(self) -> float | None:
        if self.horizontal_actuator_state is None:
            return None
        return float(self.horizontal_actuator_state.position)

    def vertical_actuator_enabled(self) -> bool:
        if self.vertical_actuator_state is None:
            return False
        return bool(self.vertical_actuator_state.enabled)

    def horizontal_actuator_enabled(self) -> bool:
        if self.horizontal_actuator_state is None:
            return False
        return bool(self.horizontal_actuator_state.enabled)

    def wait_for_vertical_position(self, target_position: float, direction: int, timeout: float, goal_handle, feedback, detect_rock: bool = False) -> tuple[bool, str, bool]:
        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                self.stop_vertical()
                return False, "Sampling action cancelled.", False

            if not self.vertical_actuator_enabled():
                self.stop_vertical()
                return False, "Vertical actuator is disabled.", False

            if detect_rock:
                safe, reason = self.state_machine.insertion_safety_check()

                if not safe:
                    if self.state_machine.measurement.force > self.maximum_force:
                        self.stop_vertical()
                        self.state_machine.rock_detected()
                        self.get_logger().warning(reason)
                        return False, reason, True

                    self.stop_vertical()
                    return False, reason, False

            else:
                safe, reason = self.state_machine.safety_check()
                if not safe:
                    self.stop_vertical()
                    return False, reason, False

            position = self.vertical_position()

            if position is None:
                if time.monotonic() - start_time > self.measurement_timeout:
                    self.stop_vertical()
                    return False, "No vertical actuator state received.", False

                time.sleep(0.05)
                continue

            feedback.current_depth = position
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            feedback.current_state = self.state_machine.state.name
            goal_handle.publish_feedback(feedback)

            if direction > 0:
                target_reached = position >= target_position
            else:
                target_reached = position <= target_position + self.position_threshold

            if target_reached:
                self.get_logger().info(f"Vertical actuator reached target position: {position:.3f} m")
                self.stop_vertical()
                return True, "", False

            if time.monotonic() - start_time > timeout:
                self.stop_vertical()
                return False, "Vertical actuator movement timed out.", False

            time.sleep(0.05)

    def wait_for_horizontal_position(self, target_position: float, direction: int, timeout: float, goal_handle, feedback) -> tuple[bool, str]:
        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                self.stop_horizontal()
                return False, "Sampling action cancelled."

            if not self.horizontal_actuator_enabled():
                self.stop_horizontal()
                return False, "Horizontal actuator is disabled."

            position = self.horizontal_position()

            if position is None:
                if time.monotonic() - start_time > self.measurement_timeout:
                    self.stop_horizontal()
                    return False, "No horizontal actuator state received."

                time.sleep(0.05)
                continue

            feedback.current_depth = self.vertical_position() if self.vertical_position() is not None else 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            feedback.current_state = self.state_machine.state.name
            goal_handle.publish_feedback(feedback)

            if direction > 0:
                target_reached = position >= target_position
            else:
                target_reached = position <= target_position + self.position_threshold

            if target_reached:
                self.get_logger().info(f"Horizontal actuator reached target position: {position:.3f} m")
                self.stop_horizontal()
                return True, ""

            if time.monotonic() - start_time > timeout:
                self.stop_horizontal()
                return False, "Horizontal actuator movement timed out."

            time.sleep(0.05)

    def retract_to_home_after_rock(self, goal_handle, feedback, double_row: bool) -> bool:
        self.state_machine.rock_detected()

        self.stop_vertical()
        self.stop_horizontal()

        feedback.current_state = SamplerState.ROCK_DETECTED.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().warning("Rock detected. Aborting sample and returning actuators to home position.")

        self.state_machine.start_retraction()
        feedback.current_state = SamplerState.RETRACTING.name
        goal_handle.publish_feedback(feedback)

        self.get_logger().info("Retracting vertical actuator after rock detection.")
        self.publish_vertical_command(RETRACT)

        start_time = time.monotonic()

        while True:
            position = self.vertical_position()

            if position is not None and position <= self.position_threshold:
                self.stop_vertical()
                break

            if time.monotonic() - start_time > self.retraction_timeout:
                self.stop_vertical()
                self.get_logger().error("Vertical actuator failed to return to home after rock detection.")
                return False

            time.sleep(0.05)

        self.stop_vertical()

        if double_row:
            self.state_machine.start_horizontal_retraction()

            feedback.current_state = SamplerState.HORIZONTAL_RETRACTING.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            self.get_logger().info("Retracting horizontal actuator after rock detection.")
            self.publish_horizontal_command(RETRACT)

            start_time = time.monotonic()

            while True:
                position = self.horizontal_position()

                if position is not None and position <= self.position_threshold:
                    self.stop_horizontal()
                    break

                if time.monotonic() - start_time > self.horizontal_retraction_timeout:
                    self.stop_horizontal()
                    self.get_logger().error("Horizontal actuator failed to return to home after rock detection.")
                    return False

                time.sleep(0.05)

            self.stop_horizontal()

        return True

    def collect_sample(self, goal_handle, feedback, target_depth: float, dwell_time: float) -> tuple[bool, str, bool, SoilSample | None]:
        vwc_samples: list[float] = []
        soil_temperature_samples: list[float] = []
        ec_samples: list[float] = []
        wind_speed_samples: list[float] = []
        wind_direction_samples: list[float] = []
        humidity_samples: list[float] = []
        air_temperature_samples: list[float] = []
        noise_samples: list[float] = []
        pm2_5_samples: list[float] = []
        pm10_samples: list[float] = []
        pressure_samples: list[float] = []
        wind_direction_gear_samples: list[int] = []
        illumination_samples: list[int] = []
        rainfall_samples: list[float] = []

        self.state_machine.start_insertion()

        feedback.current_state = SamplerState.INSERTING.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"Starting vertical insertion to {target_depth:.3f} m.")

        self.publish_vertical_command(EXTEND)

        success, reason, rock_detected = self.wait_for_vertical_position(target_position=target_depth, direction=EXTEND, timeout=self.insertion_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=True)

        if not success:
            return False, reason, rock_detected, None

        self.state_machine.start_dwell()

        feedback.current_state = SamplerState.DWELLING.name
        feedback.current_depth = target_depth
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"Starting measurement dwell for {dwell_time:.2f} s.")

        sample_timestamp = self.get_clock().now().to_msg()
        dwell_start = time.monotonic()

        while time.monotonic() - dwell_start < dwell_time:
            if goal_handle.is_cancel_requested:
                self.stop_vertical()
                return False, "Sampling action cancelled.", False, None

            safe, reason = self.state_machine.safety_check()

            if not safe:
                self.stop_vertical()
                return False, reason, False, None

            if self.latest_vwc is not None:
                vwc_samples.append(self.latest_vwc)

            if self.latest_temperature is not None:
                soil_temperature_samples.append(self.latest_temperature)

            if self.latest_ec is not None:
                ec_samples.append(self.latest_ec)

            if self.latest_wind_speed is not None:
                wind_speed_samples.append(self.latest_wind_speed)

            if self.latest_wind_direction is not None:
                wind_direction_samples.append(self.latest_wind_direction)

            if self.latest_humidity is not None:
                humidity_samples.append(self.latest_humidity)

            if self.latest_air_temperature is not None:
                air_temperature_samples.append(self.latest_air_temperature)

            if self.latest_noise is not None:
                noise_samples.append(self.latest_noise)

            if self.latest_pm2_5 is not None:
                pm2_5_samples.append(self.latest_pm2_5)

            if self.latest_pm10 is not None:
                pm10_samples.append(self.latest_pm10)

            if self.latest_pressure is not None:
                pressure_samples.append(self.latest_pressure)

            if self.latest_wind_direction_gear is not None:
                wind_direction_gear_samples.append(self.latest_wind_direction_gear)

            if self.latest_illumination is not None:
                illumination_samples.append(self.latest_illumination)

            if self.latest_rainfall is not None:
                rainfall_samples.append(self.latest_rainfall)

            feedback.current_state = SamplerState.DWELLING.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        required_samples = [
            (vwc_samples, "VWC"),
            (soil_temperature_samples, "soil temperature"),
            (ec_samples, "EC"),
            (wind_speed_samples, "wind speed"),
            (wind_direction_samples, "wind direction"),
            (humidity_samples, "humidity"),
            (air_temperature_samples, "air temperature"),
            (noise_samples, "noise"),
            (pm2_5_samples, "PM2.5"),
            (pm10_samples, "PM10"),
            (pressure_samples, "pressure"),
            (wind_direction_gear_samples, "wind direction gear"),
            (illumination_samples, "illumination"),
            (rainfall_samples, "rainfall"),
        ]

        for samples, name in required_samples:
            if not samples:
                return False, f"No {name} measurements received during dwell.", False, None

        self.state_machine.start_retraction()

        feedback.current_state = SamplerState.RETRACTING.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info("Retracting vertical actuator.")

        self.publish_vertical_command(RETRACT)

        success, reason, _ = self.wait_for_vertical_position(target_position=0.0, direction=RETRACT, timeout=self.retraction_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=False)

        if not success:
            return False, reason, False, None

        vwc_statistics = calculate_statistics(vwc_samples)
        soil_temperature_statistics = calculate_statistics(soil_temperature_samples)
        ec_statistics = calculate_statistics(ec_samples)
        wind_speed_statistics = calculate_statistics(wind_speed_samples)
        wind_direction_statistics = calculate_circular_statistics(wind_direction_samples)
        humidity_statistics = calculate_statistics(humidity_samples)
        air_temperature_statistics = calculate_statistics(air_temperature_samples)
        noise_statistics = calculate_statistics(noise_samples)
        pm2_5_statistics = calculate_statistics(pm2_5_samples)
        pm10_statistics = calculate_statistics(pm10_samples)
        pressure_statistics = calculate_statistics(pressure_samples)
        wind_direction_gear = calculate_mode(wind_direction_gear_samples)
        illumination_statistics = calculate_statistics(illumination_samples)
        rainfall_statistics = calculate_statistics(rainfall_samples)

        sample = SoilSample()
        sample.timestamp = sample_timestamp
        sample.depth = target_depth

        sample.teros12.volumetric_water_content_mean = vwc_statistics.mean
        sample.teros12.volumetric_water_content_stddev = vwc_statistics.stddev
        sample.teros12.temperature_mean = soil_temperature_statistics.mean
        sample.teros12.temperature_stddev = soil_temperature_statistics.stddev
        sample.teros12.electrical_conductivity_mean = ec_statistics.mean
        sample.teros12.electrical_conductivity_stddev = ec_statistics.stddev

        sample.sen0658.wind_speed_mean = wind_speed_statistics.mean
        sample.sen0658.wind_speed_stddev = wind_speed_statistics.stddev
        sample.sen0658.wind_direction_gear = wind_direction_gear
        sample.sen0658.wind_direction_mean = wind_direction_statistics.mean
        sample.sen0658.wind_direction_stddev = wind_direction_statistics.stddev
        sample.sen0658.humidity_mean = humidity_statistics.mean
        sample.sen0658.humidity_stddev = humidity_statistics.stddev
        sample.sen0658.temperature_mean = air_temperature_statistics.mean
        sample.sen0658.temperature_stddev = air_temperature_statistics.stddev
        sample.sen0658.noise_mean = noise_statistics.mean
        sample.sen0658.noise_stddev = noise_statistics.stddev
        sample.sen0658.pm2_5_mean = pm2_5_statistics.mean
        sample.sen0658.pm2_5_stddev = pm2_5_statistics.stddev
        sample.sen0658.pm10_mean = pm10_statistics.mean
        sample.sen0658.pm10_stddev = pm10_statistics.stddev
        sample.sen0658.pressure_mean = pressure_statistics.mean
        sample.sen0658.pressure_stddev = pressure_statistics.stddev
        sample.sen0658.illumination = round(illumination_statistics.mean)
        sample.sen0658.rainfall = rainfall_statistics.mean

        return True, "", False, sample

    def execute_sample(self, goal_handle) -> TakeSoilSample.Result:
        request = goal_handle.request

        result = TakeSoilSample.Result()
        feedback = TakeSoilSample.Feedback()

        if self.sampling_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another sampling operation is already active."
            return result

        target_depth = float(request.target_depth)
        dwell_time = float(request.dwell_time)
        double_row = bool(request.double_row)

        if target_depth < 0.0 or target_depth > self.maximum_depth:
            goal_handle.abort()
            result.success = False
            result.message = f"Target depth {target_depth:.3f} m is outside the allowed range 0.0-{self.maximum_depth:.3f} m."
            return result

        if self.vertical_actuator_state is None:
            goal_handle.abort()
            result.success = False
            result.message = "No vertical actuator state received."
            return result

        if double_row and self.horizontal_actuator_state is None:
            goal_handle.abort()
            result.success = False
            result.message = "No horizontal actuator state received."
            return result

        self.sampling_active = True
        self.state_machine.reset()

        try:
            self.get_logger().info(f"Starting soil sampling at {target_depth:.3f} m. double_row={double_row}")

            success, reason, rock_detected, sample = self.collect_sample(goal_handle=goal_handle, feedback=feedback, target_depth=target_depth, dwell_time=dwell_time)

            if not success:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = reason
                    return result

                if rock_detected:
                    recovery_success = self.retract_to_home_after_rock(goal_handle=goal_handle, feedback=feedback, double_row=double_row)

                    goal_handle.abort()
                    result.success = False

                    if recovery_success:
                        result.message = "Rock detected during vertical insertion. Actuators returned to home position."
                    else:
                        result.message = "Rock detected during vertical insertion. Failed to fully return actuators to home position."

                    return result

                goal_handle.abort()
                result.success = False
                result.message = reason
                return result

            result.samples.append(sample)

            if double_row:
                self.state_machine.start_horizontal_extension()
                feedback.current_state = SamplerState.HORIZONTAL_EXTENDING.name
                feedback.current_depth = self.vertical_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)

                self.get_logger().info("Extending horizontal actuator for second row.")

                self.publish_horizontal_command(EXTEND)

                success, reason = self.wait_for_horizontal_position(target_position=self.maximum_horizontal_distance, direction=EXTEND, timeout=self.horizontal_extension_timeout, goal_handle=goal_handle, feedback=feedback)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason

                    self.stop_all_actuators()
                    return result

                success, reason, rock_detected, sample = self.collect_sample(goal_handle=goal_handle, feedback=feedback, target_depth=target_depth, dwell_time=dwell_time)

                if not success:
                    self.stop_vertical()

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = reason
                        self.stop_all_actuators()
                        return result

                    if rock_detected:
                        recovery_success = self.retract_to_home_after_rock(goal_handle=goal_handle, feedback=feedback, double_row=True)

                        goal_handle.abort()
                        result.success = False

                        if recovery_success:
                            result.message = "Rock detected during vertical insertion of the second sample. Actuators returned to home position."
                        else:
                            result.message = "Rock detected during vertical insertion of the second sample. Failed to fully return actuators to home position."

                        return result

                    goal_handle.abort()
                    result.success = False
                    result.message = reason
                    self.stop_all_actuators()
                    return result

                result.samples.append(sample)

                self.state_machine.start_horizontal_retraction()
                feedback.current_state = SamplerState.HORIZONTAL_RETRACTING.name
                self.get_logger().info("Retracting horizontal actuator.")
                self.publish_horizontal_command(RETRACT)

                success, reason = self.wait_for_horizontal_position(target_position=0.0, direction=RETRACT, timeout=self.horizontal_retraction_timeout, goal_handle=goal_handle, feedback=feedback)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason

                    self.stop_all_actuators()
                    return result

            self.state_machine.complete()

            result.success = True
            result.message = f"Double-row sampling completed with {len(result.samples)} samples." if double_row else "Sampling completed with 1 sample."

            feedback.current_state = SamplerState.COMPLETE.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()

            self.get_logger().info(f"Sampling completed with {len(result.samples)} sample(s).")

            return result

        except Exception as exc:
            self.get_logger().error(f"Soil sampling failed: {exc}", exc_info=True)
            self.stop_all_actuators()
            self.state_machine.error()

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = str(exc)
            return result

        finally:
            self.stop_all_actuators()
            self.sampling_active = False

    def destroy_node(self) -> bool:
        self.stop_all_actuators()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SoilSamplerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
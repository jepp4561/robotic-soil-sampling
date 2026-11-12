import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING

import rclpy
if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Illuminance
from std_msgs.msg import Float32, Int32, UInt32

from soil_sampler_interfaces.action import GoToRestDepth, MeasureForceProfile, MoveHorizontal, TakeSoilSample
from soil_sampler_interfaces.msg import ActuatorCommand, ActuatorState, ForceProfileSample, SoilSample

from .force_profile_analysis import MaxRateEstimator
from .force_profile_recorder import ForceProfileRecorder
from .sampler_state_machine import SamplerState, SamplerStateMachine
from .sampler_statistics import calculate_circular_statistics, calculate_mode, calculate_statistics


EXTEND = 1
STOP = 0
RETRACT = -1

STOP_REASON_NONE = MeasureForceProfile.Result.STOP_REASON_NONE
STOP_REASON_FORCE_THRESHOLD = MeasureForceProfile.Result.STOP_REASON_FORCE_THRESHOLD
STOP_REASON_MAXIMUM_DEPTH = MeasureForceProfile.Result.STOP_REASON_MAXIMUM_DEPTH
CLASSIFICATION_NO_ROCK = MeasureForceProfile.Result.CLASSIFICATION_NO_ROCK
CLASSIFICATION_ROCK = MeasureForceProfile.Result.CLASSIFICATION_ROCK


@dataclass
class ForceProfileOutcome:
    stop_reason: int = STOP_REASON_NONE
    sample_count: int = 0
    peak_force: float = 0.0
    max_rate: float = 0.0
FORCE_PROFILE_MESSAGE_TYPE = "soil_sampler_interfaces/msg/ForceProfileSample"
HORIZONTAL_MOVE_VERTICAL_CLEARANCE = 0.01
SOIL_MEASUREMENTS = [
    ("vwc", "VWC"),
    ("temperature", "soil temperature"),
    ("ec", "EC"),
    ("wind_speed", "wind speed"),
    ("wind_direction", "wind direction"),
    ("humidity", "humidity"),
    ("air_temperature", "air temperature"),
    ("noise", "noise"),
    ("pm2_5", "PM2.5"),
    ("pm10", "PM10"),
    ("pressure", "pressure"),
    ("wind_direction_gear", "wind direction gear"),
    ("illuminance", "illuminance"),
    ("rainfall", "rainfall"),
]


class SoilSamplerNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler")

        self.declare_parameter("maximum_depth", 0.301)
        self.declare_parameter("rest_depth", 0.2)
        self.declare_parameter("maximum_horizontal_distance", 0.25)
        self.declare_parameter("maximum_force", 50.0)
        self.declare_parameter("vertical_actuator_timeout", 60.0)
        self.declare_parameter("horizontal_actuator_timeout", 60.0)
        self.declare_parameter("measurement_timeout", 5.0)
        self.declare_parameter("position_threshold", 0.001)
        self.declare_parameter("force_profile_maximum_threshold", 90.0)
        self.declare_parameter("force_profile_bag_directory", "~/soil_sampler_bags")
        self.declare_parameter("force_profile_storage_id", "mcap")
        self.declare_parameter("check_actuator_fault", True)
        self.declare_parameter("pretension_distance", 0.02)
        self.declare_parameter("force_profile_rate_window", 5)
        self.declare_parameter("force_profile_rate_min_span", 0.002)

        self.maximum_depth = float(self.get_parameter("maximum_depth").value)
        self.rest_depth = float(self.get_parameter("rest_depth").value)
        self.maximum_horizontal_distance = float(self.get_parameter("maximum_horizontal_distance").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.vertical_actuator_timeout = float(self.get_parameter("vertical_actuator_timeout").value)
        self.horizontal_actuator_timeout = float(self.get_parameter("horizontal_actuator_timeout").value)
        self.measurement_timeout = float(self.get_parameter("measurement_timeout").value)
        self.position_threshold = float(self.get_parameter("position_threshold").value)
        self.force_profile_maximum_threshold = float(self.get_parameter("force_profile_maximum_threshold").value)
        self.force_profile_bag_directory = os.path.expanduser(str(self.get_parameter("force_profile_bag_directory").value))
        self.force_profile_storage_id = str(self.get_parameter("force_profile_storage_id").value)
        self.check_actuator_fault = bool(self.get_parameter("check_actuator_fault").value)
        self.pretension_distance = max(0.0, float(self.get_parameter("pretension_distance").value))
        self.force_profile_rate_window = max(2, int(self.get_parameter("force_profile_rate_window").value))
        self.force_profile_rate_min_span = float(self.get_parameter("force_profile_rate_min_span").value)

        # temporary parameter to enable/disable horizontal actuator for testing purposes
        self.declare_parameter("horizontal_actuator_enabled", True)
        self.horizontal_actuator_enabled_parameter = bool(self.get_parameter("horizontal_actuator_enabled").value)

        self.state_machine = SamplerStateMachine(maximum_depth=self.maximum_depth, maximum_force=self.maximum_force)
        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, TakeSoilSample, "take_sample", self.execute_sample, callback_group=self.callback_group)
        self.force_profile_action_server = ActionServer(self, MeasureForceProfile, "measure_force_profile", self.execute_force_profile, goal_callback=self.force_profile_goal_callback, cancel_callback=self.accept_cancel_callback, callback_group=self.callback_group)
        self.force_profile_publisher = self.create_publisher(ForceProfileSample, "force_profile/sample", 10)
        self.move_horizontal_action_server = ActionServer(self, MoveHorizontal, "move_horizontal", self.execute_move_horizontal, goal_callback=self.move_horizontal_goal_callback, cancel_callback=self.accept_cancel_callback, callback_group=self.callback_group)
        self.go_to_rest_depth_action_server = ActionServer(self, GoToRestDepth, "go_to_rest_depth", self.execute_go_to_rest_depth, goal_callback=self.go_to_rest_depth_goal_callback, cancel_callback=self.accept_cancel_callback, callback_group=self.callback_group)

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
        self.humidity_subscription = self.create_subscription(RelativeHumidity, "sen0658/humidity", self.humidity_callback, 10, callback_group=self.callback_group)
        self.air_temperature_subscription = self.create_subscription(Temperature, "sen0658/temperature", self.air_temperature_callback, 10, callback_group=self.callback_group)
        self.noise_subscription = self.create_subscription(Float32, "sen0658/noise", self.noise_callback, 10, callback_group=self.callback_group)
        self.illuminance_subscription = self.create_subscription(Illuminance, "sen0658/illuminance", self.illuminance_callback, 10, callback_group=self.callback_group)
        self.pm2_5_subscription = self.create_subscription(Float32, "sen0658/pm2_5", self.pm2_5_callback, 10, callback_group=self.callback_group)
        self.pm10_subscription = self.create_subscription(Float32, "sen0658/pm10", self.pm10_callback, 10, callback_group=self.callback_group)
        self.pressure_subscription = self.create_subscription(FluidPressure, "sen0658/atmospheric_pressure", self.pressure_callback, 10, callback_group=self.callback_group)
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
        self.latest_illuminance: int | None = None
        self.latest_rainfall: float | None = None

        self.sampling_active = False

        self.force_profile_lock = threading.Lock()
        self.force_profile_recorder: ForceProfileRecorder | None = None
        self.force_profile_run_id = ""
        self.force_profile_force_threshold = 0.0
        self.force_profile_rate_estimator: MaxRateEstimator | None = None
        self.force_profile_active = False
        self.force_profile_sample_count = 0
        self.force_profile_peak_force = 0.0
        self.force_profile_stop_reason = STOP_REASON_NONE

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
        self.record_force_profile_sample(self.latest_force)

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

    def humidity_callback(self, message: RelativeHumidity) -> None:
        self.latest_humidity = float(message.relative_humidity)

    def air_temperature_callback(self, message: Temperature) -> None:
        self.latest_air_temperature = float(message.temperature)

    def noise_callback(self, message: Float32) -> None:
        self.latest_noise = float(message.data)

    def pm2_5_callback(self, message: Float32) -> None:
        self.latest_pm2_5 = float(message.data)

    def pm10_callback(self, message: Float32) -> None:
        self.latest_pm10 = float(message.data)

    def pressure_callback(self, message: FluidPressure) -> None:
        self.latest_pressure = float(message.fluid_pressure)

    def illuminance_callback(self, message: Illuminance) -> None:
        self.latest_illuminance = int(message.illuminance)

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

    def wait_for_vertical_position(self, target_position: float, direction: int, timeout: float, goal_handle: "ServerGoalHandle[TakeSoilSample.Goal]", feedback: TakeSoilSample.Feedback, detect_rock: bool = False) -> tuple[bool, str, bool]:
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

    def wait_for_horizontal_position(self, target_position: float, direction: int, timeout: float, goal_handle: "ServerGoalHandle[TakeSoilSample.Goal]", feedback: TakeSoilSample.Feedback) -> tuple[bool, str]:
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

    def retract_to_home_after_rock(self, goal_handle: "ServerGoalHandle[TakeSoilSample.Goal]", feedback: TakeSoilSample.Feedback, double_row: bool) -> bool:
        self.state_machine.rock_detected()

        self.stop_all_actuators()

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

            if position is not None and position <= self.rest_depth:
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

                if time.monotonic() - start_time > self.horizontal_actuator_timeout:
                    self.stop_horizontal()
                    self.get_logger().error("Horizontal actuator failed to return to home after rock detection.")
                    return False

                time.sleep(0.05)

            self.stop_horizontal()

        return True

    def collect_sample(self, goal_handle: "ServerGoalHandle[TakeSoilSample.Goal]", feedback: TakeSoilSample.Feedback, target_depth: float, dwell_time: float) -> tuple[bool, str, bool, SoilSample | None]:
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
        illuminance_samples: list[int] = []
        rainfall_samples: list[float] = []

        self.state_machine.start_insertion()

        feedback.current_state = SamplerState.INSERTING.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"Starting vertical insertion to {target_depth:.3f} m.")

        self.publish_vertical_command(EXTEND)

        success, reason, rock_detected = self.wait_for_vertical_position(target_position=target_depth, direction=EXTEND, timeout=self.vertical_actuator_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=True)

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

            if self.latest_illuminance is not None:
                illuminance_samples.append(self.latest_illuminance)

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
            (illuminance_samples, "illuminance"),
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

        success, reason, _ = self.wait_for_vertical_position(target_position=self.rest_depth, direction=RETRACT, timeout=self.vertical_actuator_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=False)

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
        illuminance_statistics = calculate_statistics(illuminance_samples)
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
        sample.sen0658.illuminance = round(illuminance_statistics.mean)
        sample.sen0658.rainfall = rainfall_statistics.mean

        return True, "", False, sample

    def execute_sample(self, goal_handle: "ServerGoalHandle[TakeSoilSample.Goal]") -> TakeSoilSample.Result:
        request: TakeSoilSample.Goal = goal_handle.request

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

                success, reason = self.wait_for_horizontal_position(target_position=self.maximum_horizontal_distance, direction=EXTEND, timeout=self.horizontal_actuator_timeout, goal_handle=goal_handle, feedback=feedback)

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

                success, reason = self.wait_for_horizontal_position(target_position=0.0, direction=RETRACT, timeout=self.horizontal_actuator_timeout, goal_handle=goal_handle, feedback=feedback)

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
            self.get_logger().error(f"Soil sampling failed: {exc}")
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

    def force_profile_goal_callback(self, goal_request: MeasureForceProfile.Goal) -> GoalResponse:
        threshold = float(goal_request.force_threshold)

        if self.sampling_active:
            self.get_logger().warning("Rejecting force profile goal: the vertical actuator is in use.")
            return GoalResponse.REJECT

        if not 0.0 < threshold <= self.force_profile_maximum_threshold:
            self.get_logger().warning(f"Rejecting force profile goal: threshold {threshold:.1f} N is outside the allowed range 0-{self.force_profile_maximum_threshold:.1f} N.")
            return GoalResponse.REJECT

        if not goal_request.rate_threshold > 0.0:
            self.get_logger().warning("Rejecting force profile goal: rate threshold must be greater than zero.")
            return GoalResponse.REJECT

        if goal_request.dwell_time < 0.0:
            self.get_logger().warning("Rejecting force profile goal: dwell time cannot be negative.")
            return GoalResponse.REJECT

        if goal_request.return_after_profile:
            return_depth = self.resolve_force_profile_return_depth(goal_request)

            if return_depth > self.maximum_depth:
                self.get_logger().warning(f"Rejecting force profile goal: return depth {return_depth:.3f} m exceeds maximum depth {self.maximum_depth:.3f} m.")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def resolve_force_profile_return_depth(self, goal_request: MeasureForceProfile.Goal) -> float:
        if goal_request.return_depth > 0.0:
            return float(goal_request.return_depth)

        return self.rest_depth

    def accept_cancel_callback(self, goal_handle: "ServerGoalHandle[MeasureForceProfile.Goal]") -> CancelResponse:
        return CancelResponse.ACCEPT

    def record_force_profile_sample(self, force: float) -> None:
        with self.force_profile_lock:
            recorder = self.force_profile_recorder
            estimator = self.force_profile_rate_estimator

            if not self.force_profile_active or estimator is None or self.force_profile_stop_reason != STOP_REASON_NONE:
                return

            depth = self.vertical_position()

            if depth is None:
                return

            rate = estimator.update(depth, force)

            now = self.get_clock().now()

            sample = ForceProfileSample()
            sample.header.stamp = now.to_msg()
            sample.run_id = self.force_profile_run_id
            sample.depth = depth
            sample.force = force
            sample.rate = rate if rate is not None else math.nan

            if recorder is not None:
                recorder.write(sample, now.nanoseconds)

            self.force_profile_publisher.publish(sample)

            self.force_profile_sample_count += 1
            self.force_profile_peak_force = max(self.force_profile_peak_force, force)

            if force >= self.force_profile_force_threshold:
                self.force_profile_stop_reason = STOP_REASON_FORCE_THRESHOLD
                self.stop_vertical()

    def stop_force_profile_recording(self) -> ForceProfileOutcome:
        with self.force_profile_lock:
            recorder = self.force_profile_recorder
            self.force_profile_recorder = None
            self.force_profile_active = False

            estimator = self.force_profile_rate_estimator

            outcome = ForceProfileOutcome(
                stop_reason=self.force_profile_stop_reason,
                sample_count=self.force_profile_sample_count,
                peak_force=self.force_profile_peak_force,
                max_rate=estimator.max_rate if estimator is not None else 0.0,
            )

        if recorder is not None:
            recorder.close()

        return outcome

    def measure_soil(self, dwell_time: float, goal_handle: "ServerGoalHandle[MeasureForceProfile.Goal]", feedback: MeasureForceProfile.Feedback) -> tuple[bool, str, SoilSample | None]:
        series: dict[str, list[float]] = {name: [] for name, _ in SOIL_MEASUREMENTS}

        depth = self.vertical_position() or 0.0
        sample_timestamp = self.get_clock().now().to_msg()
        dwell_start = time.monotonic()

        self.get_logger().info(f"Starting measurement dwell for {dwell_time:.2f} s at {depth:.3f} m.")

        while time.monotonic() - dwell_start < dwell_time:
            if goal_handle.is_cancel_requested:
                return False, "Force profile cancelled during dwell.", None

            force = self.latest_force if self.latest_force is not None else 0.0

            if force > self.force_profile_maximum_threshold:
                return False, f"Maximum force exceeded during dwell: {force:.1f} > {self.force_profile_maximum_threshold:.1f} N.", None

            for name, _ in SOIL_MEASUREMENTS:
                value = getattr(self, f"latest_{name}")

                if value is not None:
                    series[name].append(value)

            feedback.current_state = SamplerState.DWELLING.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = force
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        for name, label in SOIL_MEASUREMENTS:
            if not series[name]:
                return False, f"No {label} measurements received during dwell.", None

        vwc = calculate_statistics(series["vwc"])
        soil_temperature = calculate_statistics(series["temperature"])
        ec = calculate_statistics(series["ec"])
        wind_speed = calculate_statistics(series["wind_speed"])
        wind_direction = calculate_circular_statistics(series["wind_direction"])
        humidity = calculate_statistics(series["humidity"])
        air_temperature = calculate_statistics(series["air_temperature"])
        noise = calculate_statistics(series["noise"])
        pm2_5 = calculate_statistics(series["pm2_5"])
        pm10 = calculate_statistics(series["pm10"])
        pressure = calculate_statistics(series["pressure"])
        illuminance = calculate_statistics(series["illuminance"])
        rainfall = calculate_statistics(series["rainfall"])

        sample = SoilSample()
        sample.timestamp = sample_timestamp
        sample.depth = depth

        sample.teros12.volumetric_water_content_mean = vwc.mean
        sample.teros12.volumetric_water_content_stddev = vwc.stddev
        sample.teros12.temperature_mean = soil_temperature.mean
        sample.teros12.temperature_stddev = soil_temperature.stddev
        sample.teros12.electrical_conductivity_mean = ec.mean
        sample.teros12.electrical_conductivity_stddev = ec.stddev

        sample.sen0658.wind_speed_mean = wind_speed.mean
        sample.sen0658.wind_speed_stddev = wind_speed.stddev
        sample.sen0658.wind_direction_gear = calculate_mode(series["wind_direction_gear"])
        sample.sen0658.wind_direction_mean = wind_direction.mean
        sample.sen0658.wind_direction_stddev = wind_direction.stddev
        sample.sen0658.humidity_mean = humidity.mean
        sample.sen0658.humidity_stddev = humidity.stddev
        sample.sen0658.temperature_mean = air_temperature.mean
        sample.sen0658.temperature_stddev = air_temperature.stddev
        sample.sen0658.noise_mean = noise.mean
        sample.sen0658.noise_stddev = noise.stddev
        sample.sen0658.pm2_5_mean = pm2_5.mean
        sample.sen0658.pm2_5_stddev = pm2_5.stddev
        sample.sen0658.pm10_mean = pm10.mean
        sample.sen0658.pm10_stddev = pm10.stddev
        sample.sen0658.pressure_mean = pressure.mean
        sample.sen0658.pressure_stddev = pressure.stddev
        sample.sen0658.illuminance = round(illuminance.mean)
        sample.sen0658.rainfall = rainfall.mean

        return True, "", sample

    def classify_force_profile(self, request: MeasureForceProfile.Goal, outcome: ForceProfileOutcome) -> tuple[int, str]:
        rate_summary = f"max rate {outcome.max_rate:.0f} N/m against threshold {request.rate_threshold:.0f} N/m"

        if outcome.stop_reason != STOP_REASON_FORCE_THRESHOLD:
            return CLASSIFICATION_NO_ROCK, f"No rock: maximum depth reached before the force threshold, {rate_summary}."

        if outcome.max_rate >= request.rate_threshold:
            return CLASSIFICATION_ROCK, f"Rock: force threshold reached with {rate_summary}."

        return CLASSIFICATION_NO_ROCK, f"No rock: force threshold reached with {rate_summary}."

    def wait_for_force_profile_inputs(self) -> bool:
        deadline = time.monotonic() + self.measurement_timeout

        while time.monotonic() < deadline:
            if self.vertical_actuator_state is not None and self.latest_force is not None:
                return True

            time.sleep(0.05)

        return False

    def wait_for_force_profile_stop(self, goal_handle: "ServerGoalHandle[MeasureForceProfile.Goal]", feedback: MeasureForceProfile.Feedback) -> str:
        start_time = time.monotonic()

        while True:
            with self.force_profile_lock:
                stop_reason = self.force_profile_stop_reason
                estimator = self.force_profile_rate_estimator
                max_rate = estimator.max_rate if estimator is not None else 0.0

            if stop_reason != STOP_REASON_NONE:
                self.stop_vertical()
                return ""

            if goal_handle.is_cancel_requested:
                self.stop_vertical()
                return "Force profile cancelled."

            if not self.vertical_actuator_enabled():
                self.stop_vertical()
                return "Vertical actuator is disabled."

            if self.check_actuator_fault and self.vertical_actuator_state is not None and self.vertical_actuator_state.fault:
                self.stop_vertical()
                return "Vertical actuator reported a fault."

            position = self.vertical_position()

            if position is None:
                self.stop_vertical()
                return "No vertical actuator state received."

            if position >= self.maximum_depth:
                with self.force_profile_lock:
                    if self.force_profile_stop_reason == STOP_REASON_NONE:
                        self.force_profile_stop_reason = STOP_REASON_MAXIMUM_DEPTH

                self.stop_vertical()
                return ""

            if time.monotonic() - start_time > self.vertical_actuator_timeout:
                self.stop_vertical()
                return "Force profile extension timed out."

            feedback.current_state = self.state_machine.state.name
            feedback.current_depth = position
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            feedback.max_rate = max_rate
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

    def execute_force_profile(self, goal_handle: "ServerGoalHandle[MeasureForceProfile.Goal]") -> MeasureForceProfile.Result:
        request: MeasureForceProfile.Goal = goal_handle.request

        result = MeasureForceProfile.Result()
        feedback = MeasureForceProfile.Feedback()

        if self.sampling_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another operation is already using the vertical actuator."
            return result

        self.sampling_active = True
        self.state_machine.reset()

        try:
            if not self.wait_for_force_profile_inputs():
                goal_handle.abort()
                result.success = False
                result.message = "No vertical actuator state or load cell reading received."
                return result

            if not self.vertical_actuator_enabled():
                goal_handle.abort()
                result.success = False
                result.message = "Vertical actuator is disabled."
                return result

            start_position = self.vertical_position() or 0.0
            run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            recorder = None

            if request.record_bag:
                recorder = ForceProfileRecorder(self.force_profile_bag_directory, run_id, self.force_profile_publisher.topic_name, FORCE_PROFILE_MESSAGE_TYPE, self.force_profile_storage_id)
                result.bag_path = recorder.path

            result.start_depth = start_position
            feedback.run_id = run_id

            with self.force_profile_lock:
                self.force_profile_run_id = run_id
                self.force_profile_force_threshold = float(request.force_threshold)
                self.force_profile_rate_estimator = MaxRateEstimator(self.force_profile_rate_window, self.force_profile_rate_min_span)
                self.force_profile_sample_count = 0
                self.force_profile_peak_force = 0.0
                self.force_profile_stop_reason = STOP_REASON_NONE
                self.force_profile_recorder = recorder
                self.force_profile_active = True

            self.get_logger().info(f"Starting force profile {run_id} with threshold {request.force_threshold:.1f} N and rate threshold {request.rate_threshold:.0f} N/m from {start_position:.3f} m. Recording to {recorder.path if recorder is not None else 'no bag'}")

            self.state_machine.start_insertion()
            self.publish_vertical_command(EXTEND)

            failure = self.wait_for_force_profile_stop(goal_handle, feedback)
            outcome = self.stop_force_profile_recording()

            result.stop_reason = outcome.stop_reason
            result.sample_count = outcome.sample_count
            result.peak_force = outcome.peak_force
            result.max_rate = outcome.max_rate
            result.final_depth = self.vertical_position() or 0.0
            result.final_force = self.latest_force if self.latest_force is not None else 0.0

            if failure:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()

                self.state_machine.error()
                result.success = False
                result.message = failure
                return result

            classification, message = self.classify_force_profile(request, outcome)
            result.classification = classification

            dwell_failure = ""

            if request.dwell_time > 0.0 and classification == CLASSIFICATION_NO_ROCK:
                self.state_machine.start_dwell()

                dwell_success, dwell_reason, soil_sample = self.measure_soil(float(request.dwell_time), goal_handle, feedback)

                if dwell_success and soil_sample is not None:
                    result.sample_collected = True
                    result.sample = soil_sample
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.state_machine.error()
                    result.success = False
                    result.message = dwell_reason
                    return result
                else:
                    dwell_failure = dwell_reason

            if request.return_after_profile:
                return_depth = self.resolve_force_profile_return_depth(request)

                self.get_logger().info(f"Returning vertical actuator to {return_depth:.3f} m.")

                success, reason = self.move_with_pretension(return_depth, goal_handle, feedback)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    self.state_machine.error()
                    result.success = False
                    result.message = f"Force profile finished ({message}) but returning to {return_depth:.3f} m failed: {reason}"
                    return result

            if dwell_failure:
                goal_handle.abort()
                self.state_machine.error()
                result.success = False
                result.message = dwell_failure
                return result

            self.state_machine.complete()

            result.message = message

            feedback.current_state = SamplerState.COMPLETE.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = result.final_force
            feedback.max_rate = outcome.max_rate
            goal_handle.publish_feedback(feedback)

            result.success = True
            goal_handle.succeed()
            self.get_logger().info(message)

            return result

        except Exception as exc:
            self.get_logger().error(f"Force profile failed: {exc}")
            self.stop_vertical()
            self.state_machine.error()

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = str(exc)
            return result

        finally:
            self.stop_force_profile_recording()
            self.stop_vertical()
            self.sampling_active = False

    def move_with_pretension(self, target_position: float, goal_handle: "ServerGoalHandle[GoToRestDepth.Goal]", feedback: GoToRestDepth.Feedback) -> tuple[bool, str]:
        if self.pretension_distance > 0.0:
            pretension_position = max(0.0, target_position - self.pretension_distance)

            self.get_logger().info(f"Pretensioning: moving to {pretension_position:.3f} m before settling at {target_position:.3f} m.")

            success, reason = self.move_vertical_to_depth(pretension_position, goal_handle, feedback)

            if not success:
                return False, reason

        return self.move_vertical_to_depth(target_position, goal_handle, feedback)

    def go_to_rest_depth_goal_callback(self, goal_request: GoToRestDepth.Goal) -> GoalResponse:
        if self.sampling_active:
            self.get_logger().warning("Rejecting go to rest depth goal: the vertical actuator is in use.")
            return GoalResponse.REJECT

        if not 0.0 <= self.rest_depth <= self.maximum_depth:
            self.get_logger().warning(f"Rejecting go to rest depth goal: rest depth {self.rest_depth:.3f} m is outside 0-{self.maximum_depth:.3f} m.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def move_vertical_to_depth(self, target_position: float, goal_handle: "ServerGoalHandle[GoToRestDepth.Goal]", feedback: GoToRestDepth.Feedback) -> tuple[bool, str]:
        position = self.vertical_position()

        if position is None:
            return False, "No vertical actuator state received."

        if abs(position - target_position) <= self.position_threshold:
            return True, ""

        direction = EXTEND if position < target_position else RETRACT

        if direction == EXTEND:
            self.state_machine.start_insertion()
        else:
            self.state_machine.start_retraction()

        start_time = time.monotonic()

        self.publish_vertical_command(direction)

        while True:
            if goal_handle.is_cancel_requested:
                self.stop_vertical()
                return False, "Move cancelled."

            if not self.vertical_actuator_enabled():
                self.stop_vertical()
                return False, "Vertical actuator is disabled."

            if self.check_actuator_fault and self.vertical_actuator_state is not None and self.vertical_actuator_state.fault:
                self.stop_vertical()
                return False, "Vertical actuator reported a fault."

            position = self.vertical_position()

            if position is None:
                self.stop_vertical()
                return False, "No vertical actuator state received."

            force = self.latest_force if self.latest_force is not None else 0.0

            if direction == EXTEND and force > self.state_machine.maximum_force:
                self.stop_vertical()
                return False, f"Maximum force exceeded while extending: {force:.1f} > {self.state_machine.maximum_force:.1f} N."

            reached = position >= target_position - self.position_threshold if direction == EXTEND else position <= target_position + self.position_threshold

            if reached:
                self.stop_vertical()
                return True, ""

            if time.monotonic() - start_time > self.vertical_actuator_timeout:
                self.stop_vertical()
                return False, "Move timed out."

            feedback.current_state = self.state_machine.state.name
            feedback.current_depth = position
            feedback.current_force = force
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

    def execute_go_to_rest_depth(self, goal_handle: "ServerGoalHandle[GoToRestDepth.Goal]") -> GoToRestDepth.Result:
        result = GoToRestDepth.Result()
        feedback = GoToRestDepth.Feedback()

        if self.sampling_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another operation is already using the vertical actuator."
            return result

        self.sampling_active = True
        self.state_machine.reset()

        try:
            if not self.wait_for_force_profile_inputs():
                goal_handle.abort()
                result.success = False
                result.message = "No vertical actuator state or load cell reading received."
                return result

            if not self.vertical_actuator_enabled():
                goal_handle.abort()
                result.success = False
                result.message = "Vertical actuator is disabled."
                return result

            self.get_logger().info(f"Moving vertical actuator to rest depth {self.rest_depth:.3f} m.")

            success, reason = self.move_with_pretension(self.rest_depth, goal_handle, feedback)

            result.final_depth = self.vertical_position() or 0.0

            if not success:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()

                self.state_machine.error()
                result.success = False
                result.message = reason
                return result

            self.state_machine.complete()

            result.success = True
            result.message = f"Vertical actuator at rest depth {result.final_depth:.3f} m."

            feedback.current_state = SamplerState.COMPLETE.name
            feedback.current_depth = result.final_depth
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()

            self.get_logger().info(result.message)

            return result

        except Exception as exc:
            self.get_logger().error(f"Move to rest depth failed: {exc}")
            self.stop_vertical()
            self.state_machine.error()

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = str(exc)
            return result

        finally:
            self.stop_vertical()
            self.sampling_active = False

    def move_horizontal_goal_callback(self, goal_request: MoveHorizontal.Goal) -> GoalResponse:
        if self.sampling_active:
            self.get_logger().warning("Rejecting move horizontal goal: the actuators are in use.")
            return GoalResponse.REJECT

        if not 0.0 <= goal_request.position <= self.maximum_horizontal_distance:
            self.get_logger().warning(f"Rejecting move horizontal goal: position {goal_request.position:.3f} m is outside 0-{self.maximum_horizontal_distance:.3f} m.")
            return GoalResponse.REJECT

        vertical_position = self.vertical_position()

        if vertical_position is None or vertical_position > self.rest_depth + HORIZONTAL_MOVE_VERTICAL_CLEARANCE:
            self.get_logger().warning("Rejecting move horizontal goal: the vertical actuator is not at rest depth.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def execute_move_horizontal(self, goal_handle: "ServerGoalHandle[MoveHorizontal.Goal]") -> MoveHorizontal.Result:
        request: MoveHorizontal.Goal = goal_handle.request

        result = MoveHorizontal.Result()
        feedback = MoveHorizontal.Feedback()

        if self.sampling_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another operation is already using the actuators."
            return result

        self.sampling_active = True
        self.state_machine.reset()

        try:
            position = self.horizontal_position()

            if position is None or not self.horizontal_actuator_enabled():
                goal_handle.abort()
                result.success = False
                result.message = "Horizontal actuator state unavailable or actuator disabled."
                return result

            target_position = float(request.position)
            result.final_position = position

            if abs(position - target_position) > self.position_threshold:
                direction = EXTEND if position < target_position else RETRACT

                if direction == EXTEND:
                    self.state_machine.start_horizontal_extension()
                else:
                    self.state_machine.start_horizontal_retraction()

                self.get_logger().info(f"Moving horizontal actuator from {position:.3f} m to {target_position:.3f} m.")

                self.publish_horizontal_command(direction)

                success, reason = self.wait_for_horizontal_position(target_position, direction, self.horizontal_actuator_timeout, goal_handle, feedback)

                result.final_position = self.horizontal_position() or 0.0

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    self.state_machine.error()
                    result.success = False
                    result.message = reason
                    return result

            self.state_machine.complete()

            result.success = True
            result.message = f"Horizontal actuator at {result.final_position:.3f} m."

            goal_handle.succeed()

            return result

        except Exception as exc:
            self.get_logger().error(f"Horizontal move failed: {exc}")
            self.stop_horizontal()
            self.state_machine.error()

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = str(exc)
            return result

        finally:
            self.stop_horizontal()
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
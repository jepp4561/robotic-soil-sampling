import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Illuminance, NavSatFix, RelativeHumidity, Temperature
from std_msgs.msg import Float32, Int32

from soil_sampler_interfaces.action import MeasureForceProfile, MoveHorizontal, RunExperiment
from soil_sampler_interfaces.msg import ExperimentSample, ForceProfileSample, SkippedLocation

from .sensor_recorder import SensorRecorder

CLASSIFICATION_ROCK = MeasureForceProfile.Result.CLASSIFICATION_ROCK
DWELLING_STATE = "DWELLING"


class SoilSamplerExperimentNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler_experiment")

        self.declare_parameter("robot_namespace", "a300_00008")
        self.declare_parameter("force_profile_action", "/soil_sampler/measure_force_profile")
        self.declare_parameter("move_horizontal_action", "/soil_sampler/move_horizontal")
        self.declare_parameter("sensor_namespace", "/soil_sampler")
        self.declare_parameter("force_profile_topic", "/soil_sampler/force_profile/sample")
        self.declare_parameter("gps_topic", "")
        self.declare_parameter("record_bag", True)
        self.declare_parameter("bag_directory", "~/soil_sampler_bags/experiments")
        self.declare_parameter("bag_storage_id", "mcap")
        self.declare_parameter("horizontal_sample_offset_x", 0.0)
        self.declare_parameter("horizontal_sample_offset_y", -0.25)
        self.declare_parameter("odom_timeout", 5.0)
        self.declare_parameter("movement_timeout", 120.0)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("stop_duration", 0.5)
        self.declare_parameter("measurement_server_timeout", 5.0)
        self.declare_parameter("cancel_timeout", 30.0)

        robot_namespace = str(self.get_parameter("robot_namespace").value).strip("/")
        force_profile_action = str(self.get_parameter("force_profile_action").value)
        move_horizontal_action = str(self.get_parameter("move_horizontal_action").value)
        sensor_namespace = str(self.get_parameter("sensor_namespace").value).rstrip("/")
        force_profile_topic = str(self.get_parameter("force_profile_topic").value)
        gps_topic = str(self.get_parameter("gps_topic").value) or f"/{robot_namespace}/sensors/gps_0/fix"
        self.record_bag = bool(self.get_parameter("record_bag").value)
        self.bag_directory = str(self.get_parameter("bag_directory").value)
        self.bag_storage_id = str(self.get_parameter("bag_storage_id").value)
        self.horizontal_sample_offset_x = float(self.get_parameter("horizontal_sample_offset_x").value)
        self.horizontal_sample_offset_y = float(self.get_parameter("horizontal_sample_offset_y").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.movement_timeout = float(self.get_parameter("movement_timeout").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.stop_duration = float(self.get_parameter("stop_duration").value)
        self.measurement_server_timeout = float(self.get_parameter("measurement_server_timeout").value)
        self.cancel_timeout = float(self.get_parameter("cancel_timeout").value)

        self.horizontal_extension = math.hypot(self.horizontal_sample_offset_x, self.horizontal_sample_offset_y)

        self.cmd_vel_topic = f"/{robot_namespace}/cmd_vel"
        self.odom_topic = f"/{robot_namespace}/platform/odom/filtered"

        dwell_topics: list[tuple[str, type]] = [
            (f"{sensor_namespace}/teros12/volumetric_water_content", Float32),
            (f"{sensor_namespace}/teros12/temperature", Temperature),
            (f"{sensor_namespace}/teros12/electrical_conductivity", Float32),
            (f"{sensor_namespace}/sen0658/wind_speed", Float32),
            (f"{sensor_namespace}/sen0658/wind_direction_gear", Int32),
            (f"{sensor_namespace}/sen0658/wind_direction", Float32),
            (f"{sensor_namespace}/sen0658/humidity", RelativeHumidity),
            (f"{sensor_namespace}/sen0658/temperature", Temperature),
            (f"{sensor_namespace}/sen0658/noise", Float32),
            (f"{sensor_namespace}/sen0658/illuminance", Illuminance),
            (f"{sensor_namespace}/sen0658/pm2_5", Float32),
            (f"{sensor_namespace}/sen0658/pm10", Float32),
            (f"{sensor_namespace}/sen0658/atmospheric_pressure", FluidPressure),
            (f"{sensor_namespace}/sen0658/rainfall", Float32),
        ]

        self.gated_topics = {topic for topic, _ in dwell_topics}
        self.sensor_topics: list[tuple[str, type]] = dwell_topics + [
            (force_profile_topic, ForceProfileSample),
            (gps_topic, NavSatFix),
        ]

        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, RunExperiment, "run_experiment", self.execute_experiment, callback_group=self.callback_group)
        self.force_profile_action_client = ActionClient(self, MeasureForceProfile, force_profile_action, callback_group=self.callback_group)
        self.move_horizontal_action_client = ActionClient(self, MoveHorizontal, move_horizontal_action, callback_group=self.callback_group)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.sample_publisher = self.create_publisher(ExperimentSample, "sample", 10)
        self.skipped_location_publisher = self.create_publisher(SkippedLocation, "skipped_location", 10)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10, callback_group=self.callback_group)

        self.latest_x: float | None = None
        self.latest_y: float | None = None
        self.latest_heading: float | None = None
        self.latest_odom_time: float | None = None

        self.experiment_active = False
        self.horizontal_extended = False
        self.recorder: SensorRecorder | None = None

        self.get_logger().info("Soil sampler experiment node started.")
        self.get_logger().info(f"Odometry topic: {self.odom_topic}")
        self.get_logger().info(f"Command topic: {self.cmd_vel_topic}")
        self.get_logger().info(f"GPS topic: {gps_topic}")

    def odom_callback(self, message: Odometry) -> None:
        self.latest_x = float(message.pose.pose.position.x)
        self.latest_y = float(message.pose.pose.position.y)

        q = message.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.latest_heading = math.atan2(sin_yaw, cos_yaw)
        self.latest_odom_time = time.monotonic()

    def publish_velocity(self, linear_x: float, angular_z: float = 0.0) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.x = linear_x
        command.twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(command)

    def stop_robot(self) -> None:
        self.publish_velocity(0.0, 0.0)

    def have_recent_odom(self) -> bool:
        if self.latest_x is None or self.latest_y is None or self.latest_heading is None or self.latest_odom_time is None:
            return False

        return time.monotonic() - self.latest_odom_time <= self.odom_timeout

    def wait_for_odom(self) -> bool:
        start_time = time.monotonic()

        while time.monotonic() - start_time < self.odom_timeout:
            if self.have_recent_odom():
                return True

            time.sleep(0.05)

        return False

    def create_sample_location(self, sample, x: float, y: float, heading: float) -> ExperimentSample:
        result = ExperimentSample()
        result.timestamp = sample.timestamp
        result.x = x
        result.y = y
        result.heading = heading
        result.sample = sample
        return result

    def calculate_second_sample_position(self, x: float, y: float, heading: float) -> tuple[float, float]:
        offset_x = self.horizontal_sample_offset_x
        offset_y = self.horizontal_sample_offset_y

        sample_x = x + math.cos(heading) * offset_x - math.sin(heading) * offset_y
        sample_y = y + math.sin(heading) * offset_x + math.cos(heading) * offset_y

        return sample_x, sample_y

    def publish_state(self, goal_handle, feedback, current_sample: int, total_samples: int, state: str, distance_travelled: float = 0.0) -> None:
        feedback.current_sample = current_sample
        feedback.total_samples = total_samples
        feedback.current_state = state
        feedback.distance_travelled = distance_travelled
        goal_handle.publish_feedback(feedback)

    def move_distance(self, distance: float, speed: float, goal_handle, feedback, current_sample: int, total_samples: int) -> tuple[bool, str]:
        if distance <= 0.0:
            return True, ""

        if speed <= 0.0:
            return False, "Drive speed must be greater than zero."

        if not self.have_recent_odom():
            if not self.wait_for_odom():
                return False, "No recent Husky odometry received."

        start_x = self.latest_x
        start_y = self.latest_y

        if start_x is None or start_y is None:
            return False, "Husky position is unavailable."

        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                return False, "Experiment cancelled."

            if not self.have_recent_odom():
                self.stop_robot()
                return False, "Husky odometry became unavailable."

            current_x = self.latest_x
            current_y = self.latest_y

            if current_x is None or current_y is None:
                self.stop_robot()
                return False, "Husky position is unavailable."

            distance_travelled = math.sqrt((current_x - start_x) ** 2 + (current_y - start_y) ** 2)

            self.publish_state(goal_handle, feedback, current_sample, total_samples, "MOVING", distance_travelled)

            if distance_travelled >= distance - self.position_tolerance:
                self.stop_robot()
                time.sleep(self.stop_duration)
                return True, ""

            if time.monotonic() - start_time > self.movement_timeout:
                self.stop_robot()
                return False, "Husky movement timed out."

            self.publish_velocity(speed)
            time.sleep(0.05)

    def call_action(self, client: ActionClient, goal, goal_handle, server_name: str, feedback_callback=None):
        if not client.wait_for_server(timeout_sec=self.measurement_server_timeout):
            return None, f"The {server_name} action server is unavailable."

        goal_future = client.send_goal_async(goal, feedback_callback=feedback_callback)

        while not goal_future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None, f"Experiment cancelled while waiting for the {server_name} goal."

            time.sleep(0.05)

        child_goal_handle = goal_future.result()

        if child_goal_handle is None or not child_goal_handle.accepted:
            return None, f"The {server_name} rejected the goal."

        result_future = child_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                child_goal_handle.cancel_goal_async()

                deadline = time.monotonic() + self.cancel_timeout

                while not result_future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)

                return None, f"Experiment cancelled while waiting for the {server_name} result."

            time.sleep(0.05)

        return result_future.result().result, ""

    def move_horizontal(self, position: float, goal_handle, feedback, current_sample: int, total_samples: int) -> tuple[bool, str]:
        self.publish_state(goal_handle, feedback, current_sample, total_samples, "MOVING_HORIZONTAL")

        goal = MoveHorizontal.Goal()
        goal.position = float(position)

        result, reason = self.call_action(self.move_horizontal_action_client, goal, goal_handle, "horizontal actuator")

        if result is None:
            return False, reason

        if not result.success:
            return False, f"Horizontal move to {position:.3f} m failed: {result.message}"

        return True, ""

    def restore_horizontal(self) -> None:
        try:
            success, reason = self.move_horizontal_without_feedback(0.0)

            if success:
                self.horizontal_extended = False
            else:
                self.get_logger().warning(f"Could not retract the horizontal actuator: {reason}")
        except Exception as exc:
            self.get_logger().warning(f"Could not retract the horizontal actuator: {exc}")

    def move_horizontal_without_feedback(self, position: float) -> tuple[bool, str]:
        goal = MoveHorizontal.Goal()
        goal.position = float(position)

        result, reason = self.call_action(self.move_horizontal_action_client, goal, None, "horizontal actuator")

        if result is None:
            return False, reason

        return bool(result.success), result.message

    def set_recording_gate(self, is_open: bool) -> None:
        recorder = self.recorder

        if recorder is not None:
            recorder.set_gate(is_open)

    def probe_feedback_callback(self, feedback_message) -> None:
        self.set_recording_gate(feedback_message.feedback.current_state == DWELLING_STATE)

    def probe_location(self, request, result, goal_handle, feedback, location_index: int, row: int, x: float, y: float, heading: float) -> tuple[bool, str]:
        total_samples = request.number_of_samples
        current_sample = location_index + 1
        description = f"sampling location {current_sample}, row {row}"

        self.publish_state(goal_handle, feedback, current_sample, total_samples, "PROBING")

        probe_goal = MeasureForceProfile.Goal()
        probe_goal.force_threshold = float(request.force_threshold)
        probe_goal.rate_threshold = float(request.rate_threshold)
        probe_goal.dwell_time = float(request.dwell_time)
        probe_goal.record_bag = False
        probe_goal.return_after_profile = True
        probe_goal.return_depth = 0.0

        try:
            probe, reason = self.call_action(self.force_profile_action_client, probe_goal, goal_handle, "force profile", feedback_callback=self.probe_feedback_callback)
        finally:
            self.set_recording_gate(False)

        if probe is None:
            return False, reason

        if not probe.success:
            return False, f"Force profile failed at {description}: {probe.message}"

        if probe.classification == CLASSIFICATION_ROCK:
            skipped = SkippedLocation()
            skipped.timestamp = self.get_clock().now().to_msg()
            skipped.x = x
            skipped.y = y
            skipped.heading = heading
            skipped.row = row
            skipped.depth = probe.final_depth
            skipped.peak_force = probe.peak_force
            skipped.max_rate = probe.max_rate

            result.skipped_locations.append(skipped)
            self.skipped_location_publisher.publish(skipped)

            self.get_logger().warning(f"Rock at {description}: peak force {probe.peak_force:.1f} N, max rate {probe.max_rate:.0f} N/m at {probe.final_depth:.3f} m. Skipping.")

            return True, ""

        if not probe.sample_collected:
            return False, f"No soil sample was collected at {description}."

        experiment_sample = self.create_sample_location(sample=probe.sample, x=x, y=y, heading=heading)

        result.samples.append(experiment_sample)
        self.sample_publisher.publish(experiment_sample)

        return True, ""

    def process_location(self, request, result, goal_handle, feedback, location_index: int, husky_x: float, husky_y: float, husky_heading: float) -> tuple[bool, str]:
        current_sample = location_index + 1
        total_samples = request.number_of_samples
        rows = 2 if request.double_row else 1

        for row in range(1, rows + 1):
            if row == 1:
                sample_x, sample_y = husky_x, husky_y
            else:
                self.horizontal_extended = True

                success, reason = self.move_horizontal(self.horizontal_extension, goal_handle, feedback, current_sample, total_samples)

                if not success:
                    return False, reason

                sample_x, sample_y = self.calculate_second_sample_position(husky_x, husky_y, husky_heading)

            success, reason = self.probe_location(request, result, goal_handle, feedback, location_index, row, sample_x, sample_y, husky_heading)

            if not success:
                return False, reason

        if request.double_row:
            success, reason = self.move_horizontal(0.0, goal_handle, feedback, current_sample, total_samples)

            if not success:
                return False, reason

            self.horizontal_extended = False

        return True, ""

    def finish_unsuccessfully(self, goal_handle, result: RunExperiment.Result, message: str) -> RunExperiment.Result:
        self.stop_robot()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        result.success = False
        result.message = message
        return result

    def execute_experiment(self, goal_handle) -> RunExperiment.Result:
        request = goal_handle.request
        result = RunExperiment.Result()
        feedback = RunExperiment.Feedback()

        if self.experiment_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another experiment is already active."
            return result

        if request.number_of_samples <= 0:
            goal_handle.abort()
            result.success = False
            result.message = "Number of samples must be greater than zero."
            return result

        if request.distance_between_samples < 0.0:
            goal_handle.abort()
            result.success = False
            result.message = "Distance between samples cannot be negative."
            return result

        if request.drive_speed <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = "Drive speed must be greater than zero."
            return result

        if request.dwell_time <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = "Dwell time must be greater than zero."
            return result

        if request.force_threshold <= 0.0 or request.rate_threshold <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = "Force threshold and rate threshold must be greater than zero."
            return result

        if request.double_row and self.horizontal_extension <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = "Double row sampling requires a non-zero horizontal sample offset."
            return result

        self.experiment_active = True
        self.horizontal_extended = False
        self.stop_robot()

        try:
            self.get_logger().info(f"Starting experiment with {request.number_of_samples} sampling locations.")

            if not self.wait_for_odom():
                goal_handle.abort()
                result.success = False
                result.message = "No Husky odometry received."
                return result

            for client, name in (
                (self.force_profile_action_client, "force profile"),
                (self.move_horizontal_action_client, "horizontal actuator"),
            ):
                if not client.wait_for_server(timeout_sec=self.odom_timeout):
                    goal_handle.abort()
                    result.success = False
                    result.message = f"The {name} action server is unavailable."
                    return result

            if self.record_bag:
                self.recorder = SensorRecorder(self, self.bag_directory, self.sensor_topics, self.bag_storage_id, gated_topics=self.gated_topics, callback_group=self.callback_group)
                self.recorder.start()
                result.bag_path = self.recorder.path
                self.get_logger().info(f"Recording to {self.recorder.path}")

            for sample_index in range(request.number_of_samples):
                if goal_handle.is_cancel_requested:
                    return self.finish_unsuccessfully(goal_handle, result, "Experiment cancelled.")

                if not self.have_recent_odom():
                    if not self.wait_for_odom():
                        return self.finish_unsuccessfully(goal_handle, result, "Husky odometry unavailable before sampling.")

                husky_x = self.latest_x
                husky_y = self.latest_y
                husky_heading = self.latest_heading

                if husky_x is None or husky_y is None or husky_heading is None:
                    return self.finish_unsuccessfully(goal_handle, result, "Husky pose unavailable before sampling.")

                success, reason = self.process_location(request, result, goal_handle, feedback, sample_index, husky_x, husky_y, husky_heading)

                if not success:
                    return self.finish_unsuccessfully(goal_handle, result, reason)

                self.get_logger().info(f"Sampling location {sample_index + 1} completed. Samples: {len(result.samples)}, skipped: {len(result.skipped_locations)}.")

                if sample_index < request.number_of_samples - 1:
                    success, reason = self.move_distance(
                        distance=float(request.distance_between_samples),
                        speed=float(request.drive_speed),
                        goal_handle=goal_handle,
                        feedback=feedback,
                        current_sample=sample_index + 1,
                        total_samples=request.number_of_samples,
                    )

                    if not success:
                        return self.finish_unsuccessfully(goal_handle, result, reason)

            self.stop_robot()

            summary = f"Experiment completed with {len(result.samples)} soil samples and {len(result.skipped_locations)} skipped locations."

            self.get_logger().info(summary)

            self.publish_state(goal_handle, feedback, request.number_of_samples, request.number_of_samples, "COMPLETE")

            result.success = True
            result.message = summary

            goal_handle.succeed()

            return result

        except Exception as exc:
            self.get_logger().error(f"Experiment failed: {exc}")
            self.stop_robot()

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = str(exc)
            return result

        finally:
            self.stop_robot()

            if self.horizontal_extended:
                self.restore_horizontal()

            if self.recorder is not None:
                self.recorder.stop()
                self.recorder = None

            self.experiment_active = False

    def destroy_node(self) -> bool:
        self.stop_robot()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SoilSamplerExperimentNode()

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

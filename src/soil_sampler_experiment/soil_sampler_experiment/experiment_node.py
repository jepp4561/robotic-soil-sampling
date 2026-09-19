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

from soil_sampler_interfaces.action import TakeSoilSample

from soil_sampler_interfaces.action import RunExperiment
from soil_sampler_interfaces.msg import ExperimentSample


class SoilSamplerExperimentNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler_experiment")

        self.declare_parameter("robot_namespace", "husky")
        self.declare_parameter("sampler_action", "/soil_sampler/take_sample")
        self.declare_parameter("horizontal_sample_offset_x", 0.0)
        self.declare_parameter("horizontal_sample_offset_y", -0.25)
        self.declare_parameter("odom_timeout", 5.0)
        self.declare_parameter("movement_timeout", 120.0)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("stop_duration", 0.5)
        self.declare_parameter("measurement_server_timeout", 5.0)

        robot_namespace = str(self.get_parameter("robot_namespace").value).strip("/")
        sampler_action = str(self.get_parameter("sampler_action").value)
        self.horizontal_sample_offset_x = float(self.get_parameter("horizontal_sample_offset_x").value)
        self.horizontal_sample_offset_y = float(self.get_parameter("horizontal_sample_offset_y").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.movement_timeout = float(self.get_parameter("movement_timeout").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.stop_duration = float(self.get_parameter("stop_duration").value)
        self.measurement_server_timeout = float(self.get_parameter("measurement_server_timeout").value)

        self.cmd_vel_topic = f"/{robot_namespace}/cmd_vel"
        self.odom_topic = f"/{robot_namespace}/platform/odom/filtered"

        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, RunExperiment, "run_experiment", self.execute_experiment, callback_group=self.callback_group)
        self.sampler_action_client = ActionClient(self, TakeSoilSample, sampler_action, callback_group=self.callback_group)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.sample_publisher = self.create_publisher(ExperimentSample, "sample", 10)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10, callback_group=self.callback_group)

        self.latest_x: float | None = None
        self.latest_y: float | None = None
        self.latest_heading: float | None = None
        self.latest_odom_time: float | None = None

        self.experiment_active = False

        self.get_logger().info("Soil sampler experiment node started.")
        self.get_logger().info(f"Odometry topic: {self.odom_topic}")
        self.get_logger().info(f"Command topic: {self.cmd_vel_topic}")

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

            feedback.current_sample = current_sample
            feedback.total_samples = total_samples
            feedback.current_state = "MOVING"
            feedback.distance_travelled = distance_travelled
            goal_handle.publish_feedback(feedback)

            if distance_travelled >= distance - self.position_tolerance:
                self.stop_robot()
                time.sleep(self.stop_duration)
                return True, ""

            if time.monotonic() - start_time > self.movement_timeout:
                self.stop_robot()
                return False, "Husky movement timed out."

            self.publish_velocity(speed)
            time.sleep(0.05)

    def send_sample_goal(self, target_depth: float, dwell_time: float, double_row: bool, goal_handle, feedback, current_sample: int, total_samples: int):
        if not self.sampler_action_client.wait_for_server(timeout_sec=self.measurement_server_timeout):
            return None, "Soil sampler action server is unavailable."

        sampler_goal = TakeSoilSample.Goal()
        sampler_goal.target_depth = target_depth
        sampler_goal.dwell_time = dwell_time
        sampler_goal.double_row = double_row

        feedback.current_sample = current_sample
        feedback.total_samples = total_samples
        feedback.current_state = "SAMPLING"
        feedback.distance_travelled = 0.0
        goal_handle.publish_feedback(feedback)

        goal_future = self.sampler_action_client.send_goal_async(sampler_goal)

        while not goal_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Experiment cancelled while waiting for sampler goal."
            time.sleep(0.05)

        sampler_goal_handle = goal_future.result()

        if sampler_goal_handle is None or not sampler_goal_handle.accepted:
            return None, "Soil sampler rejected the goal."

        result_future = sampler_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                return None, "Experiment cancelled while waiting for soil sampler result."
            time.sleep(0.05)

        return result_future.result().result, ""

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

        self.experiment_active = True
        self.stop_robot()

        try:
            self.get_logger().info(f"Starting experiment with {request.number_of_samples} sampling locations.")

            if not self.wait_for_odom():
                goal_handle.abort()
                result.success = False
                result.message = "No Husky odometry received."
                return result

            if not self.sampler_action_client.wait_for_server(timeout_sec=self.odom_timeout):
                goal_handle.abort()
                result.success = False
                result.message = "Soil sampler action server is unavailable."
                return result

            for sample_index in range(request.number_of_samples):
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Experiment cancelled."
                    return result

                if not self.have_recent_odom():
                    if not self.wait_for_odom():
                        goal_handle.abort()
                        result.success = False
                        result.message = "Husky odometry unavailable before sampling."
                        return result

                husky_x = self.latest_x
                husky_y = self.latest_y
                husky_heading = self.latest_heading

                if husky_x is None or husky_y is None or husky_heading is None:
                    goal_handle.abort()
                    result.success = False
                    result.message = "Husky pose unavailable before sampling."
                    return result

                feedback.current_sample = sample_index + 1
                feedback.total_samples = request.number_of_samples
                feedback.current_state = "SAMPLING"
                feedback.distance_travelled = 0.0
                goal_handle.publish_feedback(feedback)

                sampler_result, reason = self.send_sample_goal(
                    target_depth=float(request.target_depth),
                    dwell_time=float(request.dwell_time),
                    double_row=bool(request.double_row),
                    goal_handle=goal_handle,
                    feedback=feedback,
                    current_sample=sample_index + 1,
                    total_samples=request.number_of_samples,
                )

                if sampler_result is None:
                    self.stop_robot()

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason
                    return result

                if not sampler_result.success:
                    self.stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Soil sampler failed at sampling location {sample_index + 1}: {sampler_result.message}"
                    return result

                for sample_index_within_operation, sample in enumerate(sampler_result.samples):
                    if sample_index_within_operation == 0:
                        sample_x = husky_x
                        sample_y = husky_y
                    else:
                        sample_x, sample_y = self.calculate_second_sample_position(husky_x, husky_y, husky_heading)

                    experiment_sample = self.create_sample_location(
                        sample=sample,
                        x=sample_x,
                        y=sample_y,
                        heading=husky_heading,
                    )

                    result.samples.append(experiment_sample)
                    self.sample_publisher.publish(experiment_sample)

                self.get_logger().info(f"Sampling location {sample_index + 1} completed with {len(sampler_result.samples)} sample(s).")

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
                        self.stop_robot()

                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                        else:
                            goal_handle.abort()

                        result.success = False
                        result.message = reason
                        return result

            self.stop_robot()

            self.get_logger().info(f"Experiment completed with {len(result.samples)} soil samples.")

            feedback.current_sample = request.number_of_samples
            feedback.total_samples = request.number_of_samples
            feedback.current_state = "COMPLETE"
            feedback.distance_travelled = 0.0
            goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = f"Experiment completed with {len(result.samples)} soil samples."

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
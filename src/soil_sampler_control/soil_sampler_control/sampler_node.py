import time

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

from soil_sampler_interfaces.action import TakeSoilSample, MoveRelative
from soil_sampler_interfaces.msg import ActuatorCommand
from soil_sampler_interfaces.msg import ActuatorState

from .sampler_state_machine import SamplerState
from .sampler_state_machine import SamplerStateMachine
from .sampler_statistics import calculate_statistics


class SoilSamplerNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler")

        self.declare_parameter("maximum_depth", 0.2)
        self.declare_parameter("maximum_force", 50.0)
        self.declare_parameter("insertion_timeout", 30.0)
        self.declare_parameter("retraction_timeout", 30.0)
        self.declare_parameter("position_tolerance", 5.0)
        self.declare_parameter("measurement_timeout", 5.0)

        self.declare_parameter("maximum_attempts", 3)
        self.declare_parameter("reposition_distance", 0.05)

        self.maximum_depth = float(self.get_parameter("maximum_depth").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.insertion_timeout = float(self.get_parameter("insertion_timeout").value)
        self.retraction_timeout = float(self.get_parameter("retraction_timeout").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.measurement_timeout = float(self.get_parameter("measurement_timeout").value)
        self.maximum_attempts = int(self.get_parameter("maximum_attempts").value)
        self.reposition_distance = float(self.get_parameter("reposition_distance").value)
        self.state_machine = SamplerStateMachine(maximum_depth=self.maximum_depth, maximum_force=self.maximum_force, maximum_attempts=self.maximum_attempts)

        self.callback_group = ReentrantCallbackGroup()
        self.robot_move_client = ActionClient(self, MoveRelative, "/robot/move_relative", callback_group=self.callback_group)
        self.action_server = ActionServer(self, TakeSoilSample, "take_sample", self.execute_sample, callback_group=self.callback_group)
        self.actuator_command_publisher = self.create_publisher(ActuatorCommand, "actuator_command", 10)
        self.actuator_state_subscription = self.create_subscription(ActuatorState, "actuator_state", self.actuator_state_callback, 10, callback_group=self.callback_group)
        self.force_subscription = self.create_subscription(Float32, "load_cell_reading", self.force_callback, 10, callback_group=self.callback_group)
        self.vwc_subscription = self.create_subscription(Float32, "teros12/volumetric_water_content", self.vwc_callback, 10, callback_group=self.callback_group)
        self.temperature_subscription = self.create_subscription(Temperature, "teros12/temperature", self.temperature_callback, 10, callback_group=self.callback_group)
        self.ec_subscription = self.create_subscription(Float32, "teros12/electrical_conductivity", self.ec_callback, 10, callback_group=self.callback_group)

        self.current_actuator_state: ActuatorState | None = None
        self.latest_force: float | None = None
        self.latest_vwc: float | None = None
        self.latest_temperature: float | None = None
        self.latest_ec: float | None = None

        self.sampling_active = False

        self.get_logger().info("Soil sampler control node started.")

    def actuator_state_callback(self, message: ActuatorState) -> None:
        self.current_actuator_state = message

        force = self.latest_force if self.latest_force is not None else 0.0

        self.state_machine.update_actuator_measurement(
            depth=float(message.position),
            force=force,
        )

    def force_callback(self, message: Float32) -> None:
        self.latest_force = float(message.data)
        depth = float(self.current_actuator_state.position) if self.current_actuator_state is not None else 0.0
        self.state_machine.update_actuator_measurement(depth=depth, force=self.latest_force)

    def vwc_callback(self, message: Float32) -> None:
        self.latest_vwc = float(message.data)

    def temperature_callback(
        self,
        message: Temperature,
    ) -> None:
        self.latest_temperature = float(message.temperature)

    def ec_callback(self, message: Float32) -> None:
        self.latest_ec = float(message.data)

    def publish_stop_command(self) -> None:
        command = ActuatorCommand()
        command.direction = 0
        self.actuator_command_publisher.publish(command)

    def publish_extend_command(self) -> None:
        command = ActuatorCommand()
        command.direction = 1
        self.actuator_command_publisher.publish(command)

    def publish_retract_command(self) -> None:
        command = ActuatorCommand()
        command.direction = -1
        self.actuator_command_publisher.publish(command)

    def actuator_position(self) -> float | None:
        if self.current_actuator_state is None:
            return None

        return float(self.current_actuator_state.position)

    # def actuator_fault(self) -> bool:
    #     if self.current_actuator_state is None:
    #         return True

    #     return bool(self.current_actuator_state.fault)

    def actuator_enabled(self) -> bool:
        if self.current_actuator_state is None:
            return False

        return bool(self.current_actuator_state.enabled)

    def wait_for_position(self, target_position: float, direction: int, timeout: float, goal_handle, feedback, detect_rock: bool = False) -> tuple[bool, str, bool]:
        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                self.publish_stop_command()
                return False, "Sampling action cancelled.", False

            if not self.actuator_enabled():
                self.publish_stop_command()
                return False, "Actuator is disabled.", False

            if detect_rock:
                safe, reason = self.state_machine.insertion_safety_check()

                if not safe:
                    if self.state_machine.measurement.force > self.maximum_force:
                        self.publish_stop_command()
                        self.state_machine.rock_detected()

                        self.get_logger().warning(f"Rock detected at " f"{self.state_machine.measurement.depth:.2f} m. " f"with force " f"{self.state_machine.measurement.force:.2f} N.")

                        return False, reason, True

                    self.publish_stop_command()
                    return False, reason, False

            else:
                safe, reason = self.state_machine.safety_check()

                if not safe:
                    self.publish_stop_command()
                    return False, reason, False

            position = self.actuator_position()

            if position is None:
                if time.monotonic() - start_time > self.measurement_timeout:
                    self.publish_stop_command()
                    return False, "No actuator state received.", False

                time.sleep(0.05)
                continue

            feedback.current_depth = position
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            feedback.current_state = self.state_machine.state.name

            goal_handle.publish_feedback(feedback)

            if direction > 0:
                target_reached = position >= target_position
            else:
                target_reached = position <= target_position

            if target_reached:
                self.get_logger().info(f"Target position reached: " f"{position:.2f} m")
                self.publish_stop_command()
                return True, "", False

            if time.monotonic() - start_time > timeout:
                self.publish_stop_command()
                return False, "Actuator movement timed out.", False

            time.sleep(0.05)

    def execute_sample(self, goal_handle) -> TakeSoilSample.Result:
        request = goal_handle.request

        result = TakeSoilSample.Result()
        feedback = TakeSoilSample.Feedback()

        if self.sampling_active:
            goal_handle.abort()
            result.success = False
            result.message = "Another sampling operation is already active."
            return result

        valid, reason = self.state_machine.validate_target_depth(float(request.target_depth))

        if not valid:
            goal_handle.abort()
            result.success = False
            result.message = reason
            return result

        if self.current_actuator_state is None:
            goal_handle.abort()
            result.success = False
            result.message = "No actuator state received."
            return result

        self.sampling_active = True
        self.state_machine.reset()

        vwc_samples: list[float] = []
        temperature_samples: list[float] = []
        ec_samples: list[float] = []

        try:
            target_depth = float(request.target_depth)

            self.get_logger().info(f"Starting soil sampling at " f"{target_depth:.1f} m.")

            while True:
                self.state_machine.start_insertion()
                attempt = self.state_machine.retry_count()
                self.get_logger().info(f"Starting insertion attempt " f"{attempt}/{self.maximum_attempts}.")
                feedback.current_state = SamplerState.INSERTING.name
                feedback.current_depth = self.actuator_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)
                self.publish_extend_command()
                success, reason, rock_detected = self.wait_for_position(target_position=target_depth, direction=1, timeout=self.insertion_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=True)

                if success:
                    break

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()

                    result.success = False
                    result.message = reason

                    return result

                if not rock_detected:
                    goal_handle.abort()

                    result.success = False
                    result.message = reason

                    return result

                if not self.state_machine.can_retry():
                    self.get_logger().error("Maximum insertion attempts reached.")
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Rock detected and maximum insertion " f"attempts of {self.maximum_attempts} " f"reached."
                    return result

                self.state_machine.start_retraction()
                feedback.current_state = SamplerState.RETRACTING.name
                feedback.current_depth = self.actuator_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)
                self.get_logger().info("Retracting after rock detection.")
                self.publish_retract_command()

                success, reason, _ = self.wait_for_position(target_position=0.0, direction=-1, timeout=self.retraction_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=False)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason

                    return result

                self.state_machine.start_repositioning()
                feedback.current_state = SamplerState.REPOSITIONING.name
                feedback.current_depth = self.actuator_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f"Repositioning placeholder complete. " f"Would move approximately " f"{self.reposition_distance:.1f} m " f"before the next attempt.")
                time.sleep(1.0)

            self.state_machine.start_dwell()
            feedback.current_state = SamplerState.DWELLING.name
            feedback.current_depth = target_depth
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Starting measurement dwell for " f"{request.dwell_time:.2f} s.")
            dwell_start = time.monotonic()

            while time.monotonic() - dwell_start < request.dwell_time:
                if goal_handle.is_cancel_requested:
                    self.publish_stop_command()
                    goal_handle.canceled()

                    result.success = False
                    result.message = "Sampling action cancelled."

                    return result

                safe, reason = self.state_machine.safety_check()

                if not safe:
                    self.publish_stop_command()
                    goal_handle.abort()

                    result.success = False
                    result.message = reason

                    return result

                if self.latest_vwc is not None:
                    vwc_samples.append(self.latest_vwc)

                if self.latest_temperature is not None:
                    temperature_samples.append(self.latest_temperature)

                if self.latest_ec is not None:
                    ec_samples.append(self.latest_ec)

                feedback.current_state = SamplerState.DWELLING.name
                feedback.current_depth = self.actuator_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)

            if not vwc_samples:
                raise RuntimeError("No VWC measurements received during dwell.")

            if not temperature_samples:
                raise RuntimeError("No temperature measurements received " "during dwell.")

            if not ec_samples:
                raise RuntimeError("No EC measurements received during dwell.")

            vwc_statistics = calculate_statistics(vwc_samples)
            temperature_statistics = calculate_statistics(temperature_samples)
            ec_statistics = calculate_statistics(ec_samples)

            self.state_machine.start_retraction()
            feedback.current_state = SamplerState.RETRACTING.name
            goal_handle.publish_feedback(feedback)

            self.publish_retract_command()

            success, reason, _ = self.wait_for_position(target_position=0.0, direction=-1, timeout=self.retraction_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=False)

            if not success:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()

                result.success = False
                result.message = reason

                return result

            self.state_machine.complete()

            result.vwc = vwc_statistics.mean
            result.vwc_stddev = vwc_statistics.stddev
            result.temperature = temperature_statistics.mean
            result.temperature_stddev = temperature_statistics.stddev
            result.ec = ec_statistics.mean
            result.ec_stddev = ec_statistics.stddev
            result.success = True

            result.message = f"Sampling completed using " f"{len(vwc_samples)} measurements."
            feedback.current_state = SamplerState.COMPLETE.name
            feedback.current_depth = self.actuator_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()

            self.get_logger().info(
                f"Sampling completed: "
                f"VWC={result.vwc:.3f}, "
                f"stdev={result.vwc_stddev:.3f}, "
                f"T={result.temperature:.2f}, "
                f"stdev={result.temperature_stddev:.2f}, "
                f"EC={result.ec:.3f}, "
                f"stdev={result.ec_stddev:.3f}"
            )

            return result

        except Exception as exc:
            self.get_logger().error(f"Soil sampling failed: {exc}")

            self.publish_stop_command()
            self.state_machine.error()

            goal_handle.abort()

            result.success = False
            result.message = str(exc)

            return result

        finally:
            self.publish_stop_command()
            self.sampling_active = False

    def destroy_node(self) -> bool:
        self.publish_stop_command()
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

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

from soil_sampler_interfaces.action import TakeSoilSample
from soil_sampler_interfaces.msg import ActuatorCommand
from soil_sampler_interfaces.msg import ActuatorState

from .sampler_state_machine import SamplerState
from .sampler_state_machine import SamplerStateMachine
from .sampler_statistics import calculate_statistics


class SoilSamplerNode(Node):

    def __init__(self) -> None:
        super().__init__("soil_sampler")

        self.declare_parameter("maximum_depth", 0.2)
        self.declare_parameter("maximum_horizontal_distance", 0.3)
        self.declare_parameter("maximum_force", 50.0)
        self.declare_parameter("insertion_timeout", 30.0)
        self.declare_parameter("retraction_timeout", 30.0)
        self.declare_parameter("horizontal_extension_timeout", 30.0)
        self.declare_parameter("horizontal_retraction_timeout", 30.0)
        self.declare_parameter("measurement_timeout", 5.0)

        self.maximum_depth = float(self.get_parameter("maximum_depth").value)
        self.maximum_horizontal_distance = float(self.get_parameter("maximum_horizontal_distance").value)
        self.maximum_force = float(self.get_parameter("maximum_force").value)
        self.insertion_timeout = float(self.get_parameter("insertion_timeout").value)
        self.retraction_timeout = float(self.get_parameter("retraction_timeout").value)
        self.horizontal_extension_timeout = float(self.get_parameter("horizontal_extension_timeout").value)
        self.horizontal_retraction_timeout = float(self.get_parameter("horizontal_retraction_timeout").value)
        self.measurement_timeout = float(self.get_parameter("measurement_timeout").value)

        self.state_machine = SamplerStateMachine(maximum_depth=self.maximum_depth, maximum_force=self.maximum_force)
        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(self, TakeSoilSample, "take_sample", self.execute_sample, callback_group=self.callback_group)

        self.vertical_actuator_command_publisher = self.create_publisher(ActuatorCommand, "vertical_actuator/command", 10)
        self.horizontal_actuator_command_publisher = self.create_publisher(ActuatorCommand, "horizontal_actuator/command", 10)
        self.vertical_actuator_state_subscription = self.create_subscription(ActuatorState, "vertical_actuator/state", self.vertical_actuator_state_callback, 10, callback_group=self.callback_group)
        self.horizontal_actuator_state_subscription = self.create_subscription(ActuatorState, "horizontal_actuator/state", self.horizontal_actuator_state_callback, 10, callback_group=self.callback_group)
        self.force_subscription = self.create_subscription(Float32, "load_cell_reading", self.force_callback, 10, callback_group=self.callback_group)
        self.vwc_subscription = self.create_subscription(Float32, "teros12/volumetric_water_content", self.vwc_callback, 10, callback_group=self.callback_group)
        self.temperature_subscription = self.create_subscription(Temperature, "teros12/temperature", self.temperature_callback, 10, callback_group=self.callback_group)
        self.ec_subscription = self.create_subscription(Float32, "teros12/electrical_conductivity", self.ec_callback, 10, callback_group=self.callback_group)

        self.vertical_actuator_state: ActuatorState | None = None
        self.horizontal_actuator_state: ActuatorState | None = None
        self.latest_force: float | None = None
        self.latest_vwc: float | None = None
        self.latest_temperature: float | None = None
        self.latest_ec: float | None = None

        self.sampling_active = False

        self.get_logger().info("Soil sampler control node started.")

    def vertical_actuator_state_callback(self, message: ActuatorState,) -> None:
        self.vertical_actuator_state = message

        force = self.latest_force if self.latest_force is not None else 0.0

        self.state_machine.update_actuator_measurement(depth=float(message.position), force=force)

    def horizontal_actuator_state_callback(self, message: ActuatorState) -> None:
        self.horizontal_actuator_state = message

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

    def publish_vertical_command(self, direction: int) -> None:
        command = ActuatorCommand()
        command.direction = direction
        self.vertical_actuator_command_publisher.publish(command)

    def publish_horizontal_command(self, direction: int) -> None:
        command = ActuatorCommand()
        command.direction = direction
        self.horizontal_actuator_command_publisher.publish(command)

    def stop_vertical(self) -> None:
        self.publish_vertical_command(0)

    def stop_horizontal(self) -> None:
        self.publish_horizontal_command(0)

    def stop_all_actuators(self) -> None:
        self.stop_vertical()
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

                        self.get_logger().warning( f"Rock detected at {self.state_machine.measurement.depth:.2f} m with force {self.state_machine.measurement.force:.2f} N.")

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
                target_reached = position <= target_position

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
                target_reached = position <= target_position

            if target_reached:
                self.get_logger().info(f"Horizontal actuator reached target position: {position:.3f} m")
                self.stop_horizontal()
                return True, ""

            if time.monotonic() - start_time > timeout:
                self.stop_horizontal()
                return False, "Horizontal actuator movement timed out."

            time.sleep(0.05)

    def collect_sample(self, goal_handle, feedback, target_depth: float, dwell_time: float, vwc_samples: list[float], temperature_samples: list[float], ec_samples: list[float]) -> tuple[bool, str, bool]:
        self.state_machine.start_insertion()

        feedback.current_state = SamplerState.INSERTING.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"Starting vertical insertion to {target_depth:.3f} m.")

        self.publish_vertical_command(1)

        success, reason, rock_detected = self.wait_for_vertical_position(target_position=target_depth, direction=1, timeout=self.insertion_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=True)

        if not success:
            return False, reason, rock_detected

        self.state_machine.start_dwell()

        feedback.current_state = SamplerState.DWELLING.name
        feedback.current_depth = target_depth
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f"Starting measurement dwell for {dwell_time:.2f} s.")

        dwell_start = time.monotonic()

        while time.monotonic() - dwell_start < dwell_time:
            if goal_handle.is_cancel_requested:
                self.stop_vertical()
                return False, "Sampling action cancelled.", False

            safe, reason = self.state_machine.safety_check()

            if not safe:
                self.stop_vertical()
                return False, reason, False

            if self.latest_vwc is not None:
                vwc_samples.append(self.latest_vwc)

            if self.latest_temperature is not None:
                temperature_samples.append(self.latest_temperature)

            if self.latest_ec is not None:
                ec_samples.append(self.latest_ec)

            feedback.current_state = SamplerState.DWELLING.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        if not vwc_samples:
            return False, "No VWC measurements received during dwell.", False

        if not temperature_samples:
            return False, "No temperature measurements received during dwell.", False

        if not ec_samples:
            return False, "No EC measurements received during dwell.", False

        self.state_machine.start_retraction()

        feedback.current_state = SamplerState.RETRACTING.name
        feedback.current_depth = self.vertical_position() or 0.0
        feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
        goal_handle.publish_feedback(feedback)

        self.get_logger().info("Retracting vertical actuator.")

        self.publish_vertical_command(-1)

        success, reason, _ = self.wait_for_vertical_position(target_position=0.0, direction=-1, timeout=self.retraction_timeout, goal_handle=goal_handle, feedback=feedback, detect_rock=False)

        if not success:
            return False, reason, False

        return True, "", False

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

        vwc_samples: list[float] = []
        temperature_samples: list[float] = []
        ec_samples: list[float] = []

        try:
            self.get_logger().info(f"Starting soil sampling at {target_depth:.3f} m. double_row={double_row}")

            success, reason, rock_detected = self.collect_sample(goal_handle=goal_handle, feedback=feedback, target_depth=target_depth, dwell_time=dwell_time, vwc_samples=vwc_samples, temperature_samples=temperature_samples, ec_samples=ec_samples)

            if not success:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = reason
                    return result

                if rock_detected:
                    goal_handle.abort()
                    result.success = False
                    result.message = "Rock detected during vertical insertion."
                    return result

                goal_handle.abort()
                result.success = False
                result.message = reason
                return result

            if double_row:
                self.state_machine.start_horizontal_extension()
                feedback.current_state = SamplerState.HORIZONTAL_EXTENDING.name
                feedback.current_depth = self.vertical_position() or 0.0
                feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
                goal_handle.publish_feedback(feedback)

                self.get_logger().info("Extending horizontal actuator for second row.")

                self.publish_horizontal_command(1)

                success, reason = self.wait_for_horizontal_position(target_position=self.maximum_horizontal_distance, direction=1, timeout=self.horizontal_extension_timeout, goal_handle=goal_handle, feedback=feedback)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason

                    self.stop_all_actuators()
                    return result

                success, reason, rock_detected = self.collect_sample(goal_handle=goal_handle, feedback=feedback, target_depth=target_depth, dwell_time=dwell_time, vwc_samples=vwc_samples, temperature_samples=temperature_samples, ec_samples=ec_samples)

                if not success:
                    self.stop_vertical()

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    result.success = False

                    if rock_detected:
                        result.message = "Rock detected during vertical insertion of the second sample."
                    else:
                        result.message = reason

                    self.publish_horizontal_command(-1)
                    self.wait_for_horizontal_position(target_position=0.0, direction=-1, timeout=self.horizontal_retraction_timeout, goal_handle=goal_handle, feedback=feedback)

                    return result

                self.state_machine.start_horizontal_retraction()
                feedback.current_state = SamplerState.HORIZONTAL_RETRACTING.name
                self.get_logger().info("Retracting horizontal actuator.")
                self.publish_horizontal_command(-1)
                success, reason = self.wait_for_horizontal_position(target_position=0.0, direction=-1, timeout=self.horizontal_retraction_timeout, goal_handle=goal_handle, feedback=feedback)

                if not success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()

                    result.success = False
                    result.message = reason

                    self.stop_all_actuators()
                    return result

            vwc_statistics = calculate_statistics(vwc_samples)
            temperature_statistics = calculate_statistics(temperature_samples)
            ec_statistics = calculate_statistics(ec_samples)

            self.state_machine.complete()

            result.vwc = vwc_statistics.mean
            result.vwc_stddev = vwc_statistics.stddev
            result.temperature = temperature_statistics.mean
            result.temperature_stddev = temperature_statistics.stddev
            result.ec = ec_statistics.mean
            result.ec_stddev = ec_statistics.stddev
            result.success = True

            if double_row:
                result.message = f"Double-row sampling completed using {len(vwc_samples)} VWC measurements."
            else:
                result.message = f"Sampling completed using {len(vwc_samples)} VWC measurements."

            feedback.current_state = SamplerState.COMPLETE.name
            feedback.current_depth = self.vertical_position() or 0.0
            feedback.current_force = self.latest_force if self.latest_force is not None else 0.0
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()

            self.get_logger().info(f"Sampling completed: VWC={result.vwc:.3f}, stdev={result.vwc_stddev:.3f}, T={result.temperature:.2f}, stdev={result.temperature_stddev:.2f}, EC={result.ec:.3f}, stdev={result.ec_stddev:.3f}")

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
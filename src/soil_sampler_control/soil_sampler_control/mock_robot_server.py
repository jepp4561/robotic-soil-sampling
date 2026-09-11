import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from soil_sampler_interfaces.action import MoveRelative


class MockRobotServer(Node):

    def __init__(self) -> None:
        super().__init__("mock_robot")

        self.action_server = ActionServer(
            self,
            MoveRelative,
            "move_relative",
            self.execute_callback,
        )

        self.get_logger().info(
            "Mock robot repositioning server started."
        )

    def execute_callback(self, goal_handle):
        request = goal_handle.request

        direction = request.direction
        distance = float(request.distance)

        direction_name = (
            "FORWARD"
            if direction == MoveRelative.Goal.FORWARD
            else "BACKWARD"
            if direction == MoveRelative.Goal.BACKWARD
            else "UNKNOWN"
        )

        self.get_logger().info(
            f"Received reposition request: "
            f"{direction_name}, {distance:.3f} m"
        )

        if direction not in (
            MoveRelative.Goal.FORWARD,
            MoveRelative.Goal.BACKWARD,
        ):
            goal_handle.abort()

            result = MoveRelative.Result()
            result.success = False
            result.message = "Invalid movement direction."

            return result

        if distance <= 0.0:
            goal_handle.abort()

            result = MoveRelative.Result()
            result.success = False
            result.message = "Distance must be greater than zero."

            return result

        feedback = MoveRelative.Feedback()

        steps = 20
        step_distance = distance / steps

        for step in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = MoveRelative.Result()
                result.success = False
                result.message = "Movement cancelled."

                return result

            remaining = distance - (
                step_distance * (step + 1)
            )

            feedback.distance_remaining = max(
                0.0,
                remaining,
            )

            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        goal_handle.succeed()

        result = MoveRelative.Result()
        result.success = True
        result.message = (
            f"Mock robot moved {distance:.3f} m "
            f"{direction_name.lower()}."
        )

        self.get_logger().info(
            f"Repositioning complete: {result.message}"
        )

        return result


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MockRobotServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
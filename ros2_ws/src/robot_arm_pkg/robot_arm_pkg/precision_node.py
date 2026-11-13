import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse, GoalStatus

from robot_interfaces.action import Precison
from robot_interfaces.action import Motor


class PrecisionNode(Node):

    def __init__(self):
        super().__init__('precision_node')

        self._action_server = ActionServer(
            self,
            Precison,
            'precision',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.motor_client = ActionClient(
            self,
            Motor,
            'motor'
        )

        self.get_logger().info('Action server started')

    def generate_trajectory(self, point):
        trajectory = []

        for i in range(10):
            trajectory.append((
                point.x,
                point.y,
                point.z + (i + 1) * 0.02
            ))

        return trajectory

    async def move_robot(self, points, speeds):

        while not self.motor_client.server_is_ready():
            self.get_logger().info('Waiting for MotorNode...')
            await asyncio.sleep(0.1)

        goal = Motor.Goal()
        goal.points = points
        goal.speeds = speeds

        send_goal_future = self.motor_client.send_goal_async(goal)

        motor_goal_handle = await send_goal_future

        if not motor_goal_handle.accepted:
            self.get_logger().error('Motor goal rejected')
            return False

        self.get_logger().info('Motor goal accepted')

        result_future = motor_goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Movement completed')
            return True

        self.get_logger().error('Movement failed')
        return False

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing movement')

        x = goal_handle.request.x
        y = goal_handle.request.y
        z = goal_handle.request.z

        try:
            # TODO: Calculate IK
            # TODO: Generate trajectory
            # TODO: Send motor action goal

            pass

        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

            goal_handle.abort()

            result = Precison.Result()
            result.success = False
            result.message = 'IK failed'

            return result

        result = Precison.Result()
        result.success = True
        result.message = 'Position reached'

        goal_handle.succeed()

        return result


def main(args=None):

    rclpy.init(args=args)

    node = PrecisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus

from robot_interfaces.action import Ball
from robot_interfaces.action import Motor

from geometry_msgs.msg import PointStamped


class BallNode(Node):

    def __init__(self):
        super().__init__('ball_node')

        self.green_point = None
        self.red_point = None

        # Subscribers
        self.sub_green = self.create_subscription(
            PointStamped,
            '/goal_green_base',
            self.green_callback,
            10
        )

        self.sub_red = self.create_subscription(
            PointStamped,
            '/goal_red_base',
            self.red_callback,
            10
        )

        # Action Server
        self._action_server = ActionServer(
            self,
            Ball,
            'ball',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Action server started')

        # Motor Action Client
        self.motor_client = ActionClient(
            self,
            Motor,
            'motor'
        )

    # --------------------------------------------------
    # MOTOR ACTION CLIENT
    # --------------------------------------------------

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

    # --------------------------------------------------
    # SUBSCRIBERS
    # --------------------------------------------------

    def green_callback(self, msg):
        self.green_point = msg.point

    def red_callback(self, msg):
        self.red_point = msg.point

    # --------------------------------------------------
    # ACTION CALLBACKS
    # --------------------------------------------------

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    # --------------------------------------------------
    # EXECUTE BALL ACTION
    # --------------------------------------------------

    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing ball action')

        first_ball = goal_handle.request.first_ball

        # Select first and second ball
        if first_ball == "green":
            first_point = self.green_point
            second_point = self.red_point

        elif first_ball == "red":
            first_point = self.red_point
            second_point = self.green_point

        else:
            self.get_logger().error('Unknown first_ball value')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = 'Unknown first_ball value'

            return result

        # Check ball positions
        if first_point is None or second_point is None:

            self.get_logger().error('Waiting for ball positions')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = 'Ball positions not available'

            return result

        self.get_logger().info(
            f'First ball position: '
            f'x={first_point.x:.3f}, '
            f'y={first_point.y:.3f}, '
            f'z={first_point.z:.3f}'
        )

        self.get_logger().info(
            f'Second ball position: '
            f'x={second_point.x:.3f}, '
            f'y={second_point.y:.3f}, '
            f'z={second_point.z:.3f}'
        )

        try:

            points = [
                first_point,
                second_point
            ]

            speeds = [
                42.0,
                46.0 # .0 dulezite - prevadi na float
            ]

            success = await self.move_robot(points, speeds)

            if not success:

                goal_handle.abort()

                result = Ball.Result()
                result.success = False
                result.message = 'Motor movement failed'

                return result

        except Exception as e:

            self.get_logger().error(f'Movement error: {e}')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = str(e)

            return result

        # Action completed successfully
        goal_handle.succeed()

        result = Ball.Result()
        result.success = True
        result.message = 'Both balls collected'

        return result


def main(args=None):

    rclpy.init(args=args)

    node = BallNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

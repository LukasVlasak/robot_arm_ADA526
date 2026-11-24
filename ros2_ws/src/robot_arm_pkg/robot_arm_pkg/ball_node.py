import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from robot_interfaces.action import Ball
from robot_interfaces.action import Motor
from robot_interfaces.msg import MotorSpeeds

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty

CAPTURE_TIMEOUT = 15.0
CAPTURE_POLL = 0.1
APPROACH_HEIGHT = 0.15
BACKUP = 0.05


# speeds for parts of trajectory - one value for one motor 
# 42-45 = slow, 46-52 = really fast (see dxl_motors.setMotorsSpeed)
SPEED_APPROACH = [40.0, 40.0, 40.0, 40.0, 40.0]  
SPEED_STRIKE = [162.0, 350.0, 220.0, 132.0, 132.0]    
SPEED_HOME = [46.0, 46.0, 46.0, 46.0, 46.0]    

HOME_SENTINEL = -1.0

class BallNode(Node):

    def __init__(self):
        super().__init__('ball_node')

        self.green_point = None
        self.red_point = None

        # Reentrant group -> subscribers (green/red) can run CONCURRENTLY with an action
        # that is waiting (blocking) for fresh positions (see __capture_fresh_points).
        # The node must be run using a MultiThreadedExecutor (see main).
        self.cb_group = ReentrantCallbackGroup()

        self.sub_green = self.create_subscription(
            PointStamped,
            '/goal_green_base',
            self.green_callback,
            10,
            callback_group=self.cb_group
        )

        self.sub_red = self.create_subscription(
            PointStamped,
            '/goal_red_base',
            self.red_callback,
            10,
            callback_group=self.cb_group
        )

        # Capture request - tells transformer to forget old points
        # and sends recent position of balls (viz __capture_fresh_points)
        self.pub_capture = self.create_publisher(
            Empty,
            '/ball_capture',
            10
        )

        self._action_server = ActionServer(
            self,
            Ball,
            'ball',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Action server started')

        self.motor_client = ActionClient(
            self,
            Motor,
            'motor'
        )

    # ==========================================
    # Helpers
    # ==========================================

    def __make_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def generate_trajectory(self, first_point, second_point):

        f = np.array([first_point.x, first_point.y, first_point.z], dtype=float)
        s = np.array([second_point.x, second_point.y, second_point.z], dtype=float)

        d = s - f
        d[2] = 0.0
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            raise ValueError("Micky jsou na sobe - nelze urcit smer svihu")
        d = d / dist

        strike_z = f[2]

        prestrike_xy = f - BACKUP * d
        after_first_xy = f + 0.01 * d

        p_high = prestrike_xy.copy()
        p_high[2] = strike_z + APPROACH_HEIGHT

        p_down = prestrike_xy.copy()
        p_down[2] = strike_z

        p_after_first = after_first_xy.copy()
        p_after_first[2] = strike_z

        p_second = np.array([s[0], s[1], s[2]])

        points = [
            self.__make_point(*p_high),    
            self.__make_point(*p_down),    
            self.__make_point(*p_after_first),
            self.__make_point(*p_second),  
            self.__make_point(HOME_SENTINEL, HOME_SENTINEL, HOME_SENTINEL),  
        ]

        speeds = [
            SPEED_APPROACH,  
            SPEED_APPROACH,  
            SPEED_STRIKE,    
            SPEED_STRIKE,    
            SPEED_HOME,      
        ]

        # Index of point where the arm have to settle before strike
        settle_point_index = 1

        return points, speeds, settle_point_index


    async def move_robot(self, points, speeds, settle_point_index=-1):

        if not self.motor_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MotorNode not available')
            return False

        goal = Motor.Goal()
        goal.points = points
        goal.speeds = [MotorSpeeds(speeds=s) for s in speeds]
        goal.is_ball_action = True  
        goal.settle_point_index = settle_point_index 

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

    # ==========================================
    # Callbacks
    # ==========================================

    def green_callback(self, msg):
        self.green_point = msg.point

    def red_callback(self, msg):
        self.red_point = msg.point

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def __capture_fresh_points(self):
        self.green_point = None
        self.red_point = None

        self.pub_capture.publish(Empty())
        self.get_logger().info('Capture request send, waiting for setteled positions...')

        waited = 0.0
        while self.green_point is None or self.red_point is None:
            if waited >= CAPTURE_TIMEOUT:
                return False
            time.sleep(CAPTURE_POLL)
            waited += CAPTURE_POLL

        return True

 
    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing ball action')

        first_ball = goal_handle.request.first_ball

        if first_ball not in ("green", "red"):
            self.get_logger().error('Unknown first_ball value')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = 'Unknown first_ball value'

            return result

        if not self.__capture_fresh_points():

            self.get_logger().error('Timeout - neprisly ustalene pozice micku')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = 'Ball positions not available'

            return result

        if first_ball == "green":
            first_point = self.green_point
            second_point = self.red_point
        else:
            first_point = self.red_point
            second_point = self.green_point

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

            points, speeds, settle_point_index = self.generate_trajectory(
                first_point, second_point
            )

            print(f"Trajectory: [{points}]")

            success = await self.move_robot(points, speeds, settle_point_index)

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

        goal_handle.succeed()

        result = Ball.Result()
        result.success = True
        result.message = 'Both balls collected'

        return result


def main(args=None):

    rclpy.init(args=args)

    node = BallNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import asyncio

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse

from robot_interfaces.action import Precision
from robot_interfaces.action import Motor
from robot_interfaces.msg import MotorSpeeds
from action_msgs.msg import GoalStatus


from geometry_msgs.msg import Point

APPROACH_HEIGHT = 0.15     
APPROACH_HEIGHT_2 = 0.05   

# by how much the print head travels below the specified paper height [m]
PRESS_DEPTH = 0.003

# how many intermediate points to divide the descent from the approach to the point
N_DESCENT = 20

# 0.75 = at 3/4 of the descent. The controller applies a longer SETTLE_TIME at this point.
DWELL_FRACTION = 0.9

# speeds for individual trajectory segments – one value per motor (order according to ACTIVE_MOTORS)
SPEED_TRAVEL = [-1.0, 26.0, 26.0, 26.0, 60.0]  
SPEED_TOUCH = [-1.0, 7.0, 7.0, 20.0, 65.0]   
SPEED_LIFT = [-1.0, 40.0, 40.0, 40.0, 40.0]    
SPEED_HOME = [-1.0, 46.0, 46.0, 46.0, 46.0]    

HOME_SENTINEL = -1.0

class PrecisionNode(Node):

    def __init__(self):
        super().__init__('precision_node')

        self._action_server = ActionServer(
            self,
            Precision,
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

    def __make_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def generate_trajectory(self, x, y, z):

        z_high = z + APPROACH_HEIGHT    
        z_top = z + APPROACH_HEIGHT_2    
        z_bottom = z - PRESS_DEPTH

        levels = [(float(zi), False) for zi in np.linspace(z_top, z_bottom, N_DESCENT)[1:]]

        z_dwell = z_top + DWELL_FRACTION * (z_bottom - z_top)
        levels.append((z_dwell, True))
        levels.sort(key=lambda lvl: lvl[0], reverse=True)  # klesame -> sestupne

        descent_points = [self.__make_point(x, y, zi) for zi, _ in levels]

        points = [
            self.__make_point(x, y, z_high),
            self.__make_point(x, y, z_top),
            *descent_points,
            self.__make_point(x, y, z_top),
            self.__make_point(HOME_SENTINEL, HOME_SENTINEL, HOME_SENTINEL),
        ]

        speeds = [
            SPEED_TRAVEL,   
            SPEED_TRAVEL,   
            *([SPEED_TOUCH] * len(descent_points)),
            SPEED_LIFT,
            SPEED_HOME,
        ]

        dwell_local = next(i for i, (_, is_dwell) in enumerate(levels) if is_dwell)
        settle_point_index = 2 + dwell_local

        return points, speeds, settle_point_index

    async def move_robot(self, points, speeds, settle_point_index=-1):

        while not self.motor_client.server_is_ready():
            self.get_logger().info('Waiting for MotorNode...')
            await asyncio.sleep(0.1)

        goal = Motor.Goal()
        goal.points = points
        goal.speeds = [MotorSpeeds(speeds=s) for s in speeds]
        goal.is_ball_action = False  
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
            points, speeds, settle_point_index = self.generate_trajectory(x, y, z)

            print(f"Trajetory: [{points}]")

            success = await self.move_robot(points, speeds, settle_point_index)

            if not success:
                goal_handle.abort()

                result = Precision.Result()
                result.success = False
                result.message = 'Motor movement failed'

                return result

        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

            goal_handle.abort()

            result = Precision.Result()
            result.success = False
            result.message = str(e)

            return result

        result = Precision.Result()
        result.success = True
        result.message = 'Dot made, back in home'

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

import asyncio

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus

from robot_interfaces.action import Ball
from robot_interfaces.action import Motor
from robot_interfaces.msg import MotorSpeeds

from geometry_msgs.msg import Point, PointStamped


# ==========================================
# TRAJECTORY TUNING (menit pri testovani)
# ==========================================

# oba micky se schodi JEDNIM primym svihem po primce prvni -> druhy micek.
# pred micky se najede ZEZADU (z opacne strany nez je druhy micek), aby svih
# prosel pres prvni i druhy. Priprava je rozdelena tak, aby se prvni micek
# NESCHODIL driv nez ma:
#   1) daleko za prvnim mickem a VYSOKO  -> klidny najezd, mimo micky
#   2) tesne za prvnim mickem (strike height) -> klidny najezd na primku
#   3) SETTLE (nechat rameno ustalit)
#   4) BOOM - jeden rychly primy svih pres oba micky
#   5) HOME

# jak VYSOKO nad rovinou svihu je pripravny "daleko" bod [m]
APPROACH_HEIGHT = 0.06

# jak DALEKO za prvnim mickem (proti smeru svihu) je vysoky pripravny bod [m]
FAR_BACK = 0.12

# jak daleko za prvnim mickem je odpalova pozice tesne pred svihem [m]
# dost na to, aby rameno pri najezdu micek jeste netrklo, ale uz na primce
BACKUP = 0.05

# o kolik svih pokracuje ZA druhy micek (dojezd), aby ho spolehlive sundal [m]
FOLLOW_THROUGH = 0.04

# posun vysky svihu vuci stredu micku [m] - zaporne = nize (vic do micku).
# 0 = svih prochazi stredem micku
STRIKE_HEIGHT_OFFSET = 0.0

# rychlosti pro useky - jedna hodnota na motor (poradi dle ACTIVE_MOTORS)
# 42-45 = pomalu, 46-52 = hodne rychle (viz dxl_motors.setMotorsSpeed)
SPEED_APPROACH = [40.0, 40.0, 40.0, 40.0, 40.0]  # klidny najezd na pripravne body
SPEED_STRIKE = [52.0, 52.0, 52.0, 52.0, 52.0]    # BOOM - rychly primy svih pres oba micky
SPEED_HOME = [46.0, 46.0, 46.0, 46.0, 46.0]      # navrat do home

# sentinel pro home pozici - controller si pohlida, ze souradnice == -1 a jede do HOME
HOME_SENTINEL = -1.0


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
    # TRAJECTORY
    # --------------------------------------------------

    def __make_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def generate_trajectory(self, first_point, second_point):
        """
        Vrati (points, speeds, settle_point_index) pro schozeni obou micku
        jednim primym svihem a navrat do home.

        Svih jde po primce prvni -> druhy micek. Pripravne body jsou ZA prvnim
        mickem (proti smeru svihu), takze najezd micek netrkne. Nejdriv se jede
        daleko a vysoko, pak tesne za prvni micek na primku, tam SETTLE, a pak
        jeden rychly svih pres oba micky az za druhy micek.
        """

        f = np.array([first_point.x, first_point.y, first_point.z], dtype=float)
        s = np.array([second_point.x, second_point.y, second_point.z], dtype=float)

        # vodorovny smer svihu (prvni -> druhy micek); svih je rovnobezny s deskou
        d = s - f
        d[2] = 0.0
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            raise ValueError("Micky jsou na sobe - nelze urcit smer svihu")
        d = d / dist

        # vyska svihu - stred prvniho micku +- offset; cely svih je v teto vysce
        strike_z = f[2] + STRIKE_HEIGHT_OFFSET

        # 1) daleko za prvnim mickem a vysoko (mimo micky)
        p_far_high = f - FAR_BACK * d
        p_far_high[2] = strike_z + APPROACH_HEIGHT

        # 2) odpalova pozice tesne za prvnim mickem, uz na primce a ve vysce svihu
        p_prestrike = f - BACKUP * d
        p_prestrike[2] = strike_z

        # 3) dojezd za druhy micek - konec rychleho svihu
        p_strike_end = s + FOLLOW_THROUGH * d
        p_strike_end[2] = strike_z

        points = [
            self.__make_point(*p_far_high),    # 0: klidny najezd daleko a vysoko
            self.__make_point(*p_prestrike),   # 1: klidny najezd pred micek (tady SETTLE)
            self.__make_point(*p_strike_end),  # 2: BOOM - rychly svih pres oba micky
            self.__make_point(HOME_SENTINEL, HOME_SENTINEL, HOME_SENTINEL),  # 3: home
        ]

        speeds = [
            SPEED_APPROACH,  # 0
            SPEED_APPROACH,  # 1
            SPEED_STRIKE,    # 2 BOOM
            SPEED_HOME,      # 3
        ]

        # po dojezdu na odpalovou pozici (index 1) necha controller rameno ustalit,
        # nez vyrazi rychly svih
        settle_point_index = 1

        return points, speeds, settle_point_index

    # --------------------------------------------------
    # MOTOR ACTION CLIENT
    # --------------------------------------------------

    async def move_robot(self, points, speeds, settle_point_index=-1):

        while not self.motor_client.server_is_ready():
            self.get_logger().info('Waiting for MotorNode...')
            await asyncio.sleep(0.1)

        goal = Motor.Goal()
        goal.points = points
        goal.speeds = [MotorSpeeds(speeds=s) for s in speeds]
        goal.is_ball_action = True  # ball -> predpocita vsechnu kinematiku dopredu
        goal.settle_point_index = settle_point_index  # odpalovy bod s ustalenim pred svihem

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

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
from std_msgs.msg import Empty


# ==========================================
# CAPTURE (cerstve pozice micku z kamery pred kazdou akci)
# ==========================================

# jak dlouho cekat, nez transformer posle ustalene pozice OBOU micku [s]
CAPTURE_TIMEOUT = 15.0

# perioda kontroly, jestli uz cerstve pozice dorazily [s]
CAPTURE_POLL = 0.1


# ==========================================
# TRAJECTORY TUNING (menit pri testovani)
# ==========================================

# oba micky se schodi JEDNIM primym svihem po primce prvni -> druhy micek.
# odpalova pozice je ZEZADU za prvnim mickem (z opacne strany nez druhy micek):
#   1) VYSOKO nad odpalovou pozici    -> klidny najezd, mimo micky
#   2) kolmo DOLU na vysku svihu       -> SETTLE (nechat rameno ustalit)
#   3) primo na PRVNI micek            (rychle)
#   4) primo na DRUHY micek            (rychle)
#   5) HOME

# jak VYSOKO nad vyskou svihu je pripravny bod nad odpalovou pozici [m]
APPROACH_HEIGHT = 0.15

# jak daleko za prvnim mickem (proti smeru svihu) je odpalova pozice [m]
# dost na to, aby rameno pri sjezdu dolu prvni micek jeste netrklo, ale uz na primce
BACKUP = 0.05


# rychlosti pro useky - jedna hodnota na motor (poradi dle ACTIVE_MOTORS)
# 42-45 = pomalu, 46-52 = hodne rychle (viz dxl_motors.setMotorsSpeed)
SPEED_APPROACH = [40.0, 40.0, 40.0, 40.0, 40.0]  # klidny najezd na pripravne body
SPEED_STRIKE = [162.0, 160.0, 160.0, 132.0, 132.0]    # BOOM - rychly primy svih pres oba micky
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

        # Capture request - rekne transformeru, at zahodi stare vzorky
        # a posle cerstve ustalene pozice micku (viz __capture_fresh_points)
        self.pub_capture = self.create_publisher(
            Empty,
            '/ball_capture',
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
        a navrat do home.

        Odpalova pozice je ZA prvnim mickem (proti smeru prvni -> druhy micek),
        takze sjezd dolu prvni micek netrkne. Nejdriv se najede vysoko nad
        odpalovou pozici, pak kolmo dolu na vysku svihu, tam SETTLE, a pak
        rychle primo na prvni a hned na druhy micek.
        """

        f = np.array([first_point.x, first_point.y, first_point.z], dtype=float)
        s = np.array([second_point.x, second_point.y, second_point.z], dtype=float)

        # vodorovny smer prvni -> druhy micek; svih je rovnobezny s deskou
        d = s - f
        d[2] = 0.0
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            raise ValueError("Micky jsou na sobe - nelze urcit smer svihu")
        d = d / dist

        # vyska svihu - stred prvniho micku +- offset; cely svih je v teto vysce
        strike_z = f[2]

        # odpalova pozice za prvnim mickem (na primce), x,y spolecne pro bod 0 i 1
        prestrike_xy = f - BACKUP * d
        after_first_xy = f + BACKUP * d

        # 0) vysoko nad odpalovou pozici
        p_high = prestrike_xy.copy()
        p_high[2] = strike_z + APPROACH_HEIGHT

        # 1) kolmo dolu na vysku svihu (tady SETTLE)
        p_down = prestrike_xy.copy()
        p_down[2] = strike_z

        p_after_first = after_first_xy.copy()
        p_after_first[2] = strike_z

        # 2) primo prvni micek, 3) primo druhy micek - oba ve vysce svihu
        p_first = np.array([f[0], f[1], strike_z])
        p_second = np.array([s[0], s[1], s[2]])

        points = [
            self.__make_point(*p_high),    # 0: klidny najezd vysoko nad odpalovou pozici
            self.__make_point(*p_down),    # 1: kolmo dolu na vysku svihu (tady SETTLE)
            self.__make_point(*p_after_first),
            self.__make_point(*p_second),  # 3: primo druhy micek (rychle)
            self.__make_point(HOME_SENTINEL, HOME_SENTINEL, HOME_SENTINEL),  # 4: home
        ]

        speeds = [
            SPEED_APPROACH,  # 0 najezd vysoko
            SPEED_APPROACH,  # 1 sjezd dolu
            SPEED_STRIKE,    # 2 prvni micek
            SPEED_STRIKE,    # 3 druhy micek
            SPEED_HOME,      # 4 home
        ]

        # po sjezdu na vysku svihu (index 1) necha controller rameno ustalit,
        # nez vyrazi na micky
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
    # CAPTURE CERSTVYCH POZIC
    # --------------------------------------------------

    async def __capture_fresh_points(self):
        """
        Zahodi stare pozice, pozada transformer o cerstve vzorky a pocka,
        nez dorazi ustalene pozice OBOU micku. Diky tomu se nikdy nepouzije
        stara pozice z predchozi akce (po presunu micku). Vraci True pri uspechu,
        False pri timeoutu.
        """
        self.green_point = None
        self.red_point = None

        self.pub_capture.publish(Empty())
        self.get_logger().info('Capture request poslan, cekam na ustalene pozice...')

        waited = 0.0
        while self.green_point is None or self.red_point is None:
            if waited >= CAPTURE_TIMEOUT:
                return False
            await asyncio.sleep(CAPTURE_POLL)
            waited += CAPTURE_POLL

        return True

    # --------------------------------------------------
    # EXECUTE BALL ACTION
    # --------------------------------------------------

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

        # cerstve ustalene pozice obou micku z kamery (zahodi stare hodnoty)
        if not await self.__capture_fresh_points():

            self.get_logger().error('Timeout - neprisly ustalene pozice micku')

            goal_handle.abort()

            result = Ball.Result()
            result.success = False
            result.message = 'Ball positions not available'

            return result

        # Select first and second ball
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

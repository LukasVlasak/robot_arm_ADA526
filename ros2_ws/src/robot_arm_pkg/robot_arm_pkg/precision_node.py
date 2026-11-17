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


# ==========================================
# TRAJECTORY TUNING (menit pri testovani)
# ==========================================

# dve vysky bezpecnych bodu primo nad teckou [m] - z home se najede nejdriv na vyssi
# (APPROACH_HEIGHT), pak na nizsi (APPROACH_HEIGHT_2), a odtud klesa KOLMO dolu.
# Klesani i vypocty z_dwell / levels vychazeji z nizsi vysky (APPROACH_HEIGHT_2).
APPROACH_HEIGHT = 0.15     # prvni (vyssi) priblizeni nad teckou
APPROACH_HEIGHT_2 = 0.05   # druhe (nizsi) priblizeni - odtud zacina kolme klesani

# o kolik zajede POD zadanou vysku papiru [m] - pruzina v pen holderu to pohlti
PRESS_DEPTH = 0.003

# na kolik mezibodu rozdelit klesani z approach az na tecku (vic = plynulejsi, ale pomalejsi)
N_DESCENT = 20

# kde v klesani je extra bod s delsim ustalenim: 0.0 = approach, 1.0 = tecka.
# 0.75 = ve 3/4 klesani. Controller na tenhle bod aplikuje vetsi SETTLE_TIME.
DWELL_FRACTION = 0.9

# rychlosti pro jednotlive useky trajektorie - jedna hodnota na motor (poradi dle ACTIVE_MOTORS)
# zatim vsude stejne cislo, uprav si jednotlive motory podle potreby
# base (motor 1, prvni v poradi) ma -1 -> rychlost se mu NEnastavuje
SPEED_TRAVEL = [-1.0, 26.0, 26.0, 26.0, 60.0]  # home -> nad tecku (rychle, vysoko nad deskou)
SPEED_TOUCH = [-1.0, 7.0, 7.0, 20.0, 65.0]   # klesani z approach az na tecku (pomalu, presne)
SPEED_LIFT = [-1.0, 40.0, 40.0, 40.0, 40.0]    # zvednuti od papiru
SPEED_HOME = [-1.0, 46.0, 46.0, 46.0, 46.0]    # navrat do home

# sentinel pro home pozici - controller si pohlida, ze souradnice == -1 a jede do HOME
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
        """
        Vrati (points, speeds) pro udelani tecky a navrat do home.

        Nad teckou se vzdy najede vysoko a dolu se klesa KOLMO (stejne x, y),
        aby pero nedrhlo o desku driv nez dorazi na papir. Klesani z approach
        az na tecku je pomale (SPEED_TOUCH). Do papiru zajede o PRESS_DEPTH,
        coz pohlti pruzina pen holderu -> spolehlivy dotyk i pri male chybe
        kalibrace vysky.
        """

        z_high = z + APPROACH_HEIGHT     # prvni (vyssi) priblizeni nad teckou
        z_top = z + APPROACH_HEIGHT_2    # druhe (nizsi) priblizeni - odtud kolme klesani
        z_bottom = z - PRESS_DEPTH

        # rovnomerne klesani; [1:] vynecha approach vysku - tu uz najizdi travel bod nize
        # (z_level, is_dwell) - dwell bod ma v controlleru vetsi SETTLE_TIME
        levels = [(float(zi), False) for zi in np.linspace(z_top, z_bottom, N_DESCENT)[1:]]

        # extra bod ve DWELL_FRACTION klesani, zaradi se mezi ostatni podle vysky
        z_dwell = z_top + DWELL_FRACTION * (z_bottom - z_top)
        levels.append((z_dwell, True))
        levels.sort(key=lambda lvl: lvl[0], reverse=True)  # klesame -> sestupne

        descent_points = [self.__make_point(x, y, zi) for zi, _ in levels]

        points = [
            # 1) prvni priblizeni - vysoko primo nad teckou (z home sem)
            self.__make_point(x, y, z_high),
            # 2) druhe priblizeni - nize nad teckou, odtud zacina kolme klesani
            self.__make_point(x, y, z_top),
            # 3) dotyk - pomalu kolmo dolu po mezibodech, zajede pod povrch -> tecka
            *descent_points,
            # # pokus - aby to tam chvili dyl zustalo
            # self.__make_point(x, y, z - PRESS_DEPTH),
            # 4) zvednuti kolmo nahoru od papiru
            self.__make_point(x, y, z_top),
            # 5) navrat do home (sentinel - resi controller)
            self.__make_point(HOME_SENTINEL, HOME_SENTINEL, HOME_SENTINEL),
        ]

        speeds = [
            SPEED_TRAVEL,   # prvni priblizeni
            SPEED_TRAVEL,   # druhe priblizeni
            *([SPEED_TOUCH] * len(descent_points)),
            # SPEED_TOUCH,
            SPEED_LIFT,
            SPEED_HOME,
        ]

        # index dwell bodu v celem seznamu points (pred descentem jsou 2 approach body)
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
        goal.is_ball_action = False  # precision -> po kazdem bodu ustaleni
        goal.settle_point_index = settle_point_index  # bod s delsim ustalenim (controller)

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

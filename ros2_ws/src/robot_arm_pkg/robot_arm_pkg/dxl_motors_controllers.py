import dxl_motors
import rclpy
from rclpy.node import Node
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from utils import rad2steps, steps2rad

from std_msgs.msg import String, Empty
from robot_interfaces.msg import MotorCommand

from robot_interfaces.action import Motor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
import asyncio
import time


# TODO: self.current_q IS NOT UPDATING - NOT USING IT YET, so its ok


# k 12 0.15
HOME_POS_MOTORS = {
    1: 2112,
    2: 142,
    3: 5104,
    4: 2903,
    5: 12,
}

# k 5 0.2
WEIGHT_POS_MOTORS = {
    1: 1250,
    2: 1250,
    3: 1250,
    4: 1250,
    5: 1250,
}

HOME_SENTINEL = -1.0

# precision has 2 approaches height
SETTLE_TIME_APPROACH_1 = 2  
SETTLE_TIME_APPROACH_2 = 4  
SETTLE_TIME_LOW = 0.1

# settle time at dwell point 
DWELL_SETTLE_TIME = 2

BALL_SETTLE_TIME = 1.5

PRECISION_MAX_TILT_DEG = 10

# time to wait until check is_moving
MOVE_START_DELAY = 0.2

LINK1 = 0.220
LINK2 = 0.300
LINK3 = 0.280
LINK4 = 0.202
LINK4_OFFSET = -0.00262
LINK5_OFFSET = 0.043

JOINT1_LIM = [-90*pi/180, 90*pi/180]
JOINT2_LIM = [ -70*pi/180, 70*pi/180]
JOINT3_LIM = [-50*pi/180, 180*pi/180]
JOINT4_LIM = [ -92*pi/180, 85*pi/180]
JOINT5_LIM = [-200*pi/180, 200*pi/180]

DH_OFFSETS = [2106, 2004, 4019, 2285, 12]
DIRECTIONS = [1, -1, -1, -1, 1]

ROBOT_QR = [0, 0, 0, 0, 0]

ACTIVE_MOTORS = [1, 2, 3, 4, 5]

TESTING_MOTORS_SPEED = 50

GEAR = np.array([1, 4, 3, 1, 1])

# just for testing
ARROW_UPDATE_EVERY = 20 # update current_q every 20 steps

CALIB_MEASURED_XYZ = [0.588, 0.0, 0.106]
CALIB_Q = [0, 0, 0, 0, 0]

SAG_K2 = -0.051


ERR_CMD_Z = 0.005

ERR_SAMPLES = [
    # ((x_set, y_set) [m],  [x_real, y_real, z_real] [m] | None)

    ((0.575,  0.000), [0.579,-0.002,-0.004]),
    ((0.575,  0.125), [0.579, 0.128, -0.005]),
    ((0.575,  0.225), [0.582, 0.228, -0.006]),
    ((0.575, -0.125), [0.582, -0.136, 0.000]),
    ((0.575, -0.225), [0.583, -0.229, -0.005]),

    ((0.500,  0.000), [0.503, -0.003, -0.004]),
    ((0.500,  0.125), [0.503, 0.125, -0.003]),
    ((0.500,  0.225), [0.502, 0.227, -0.004]),
    ((0.500, -0.125), [0.503, -0.130, -0.003]),
    ((0.500, -0.225), [0.501, -0.231, -0.003]),

    ((0.425,  0.000), [0.426, -0.001, -0.001]),
    ((0.425,  0.125), [0.429, 0.121, -0.001]),
    ((0.425,  0.225), [0.428, 0.224, -0.002]),
    ((0.425, -0.125), [0.426, -0.124, -0.002]),
    ((0.425, -0.225), [0.427, -0.223, -0.003]),

    ((0.350,  0.000), [0.351, -0.002, 0.003]),
    ((0.350,  0.125), [0.352, 0.123, 0.002]),
    ((0.350,  0.225), [0.352, 0.224, 0.000]),
    ((0.350, -0.125), [0.350, -0.128, 0.002]),
    ((0.350, -0.225), [0.351, -0.224, -0.001]),

    ((0.275,  0.000), [0.272, -0.004, 0.001]),
    ((0.275,  0.125), [0.278, 0.124, 0.004]),
    ((0.275,  0.225), [0.277, 0.224, 0.002]),
    ((0.275, -0.125), [0.276, -0.128, 0.004]),
    ((0.275, -0.225), [0.276, -0.225, 0.002]),

    ((0.200,  0.000), [0.200, -0.003, 0.002]),
    ((0.200,  0.125), [0.200, 0.122, 0.002]),
    ((0.200,  0.225), [0.201, 0.222, 0.000]),
    ((0.200, -0.125), [0.197, -0.127, 0.002]),
    ((0.200, -0.225), [0.195, -0.225, 0.000]),
]

# bivariat polynom in (x, y) 
def _err_basis(x, y):
    return np.array([1.0, x, y, x * x, x * y, y * y])

MOTOR_STATUS_PERIOD = 1.0

class DXLMotorsController(Node):
    def __init__(self):
        super().__init__("dxl_motors_controllers")
        self.dxlMotors = dxl_motors.DXLMotors(ACTIVE_MOTORS)
        
        self.robot = self.__get_robot_config()
        self.offsets = np.array([steps2rad(s) for s in DH_OFFSETS])
        self.directions = DIRECTIONS
        self.current_q = np.array(self.robot.qr, dtype=float)
        self.arrow_counter = 0
        self.q_bias = self.__compute_joint_bias()
        self.err_coeffs = self.__compute_err_coeffs()

        self.create_subscription(String, 'arrow_user_input', self.arrow_movement, 10)
        self.create_subscription(String, 'set_speed', self.__set_speed_callback, 10)
        self.create_subscription(Empty, 'disable_torque', self.__disable_torque_callback, 10)
        self.create_subscription(Empty, 'go_home', self.__go_home_callback, 10)
        self.create_subscription(MotorCommand, '/motor_controller/command', self.__motor_cmd_callback, 10)

        # print(self.robot)
        # print(self.robot.fkine([0,0,0,0,0]))
        # self.robot.plot([0,0,0,0,0], block=True)
        # print(self.robot.fkine([0,0,0,0,0]))

        self._action_server = ActionServer(
            self,
            Motor,
            'motor',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # /motor_status
        # self.motor_status_pub = self.create_publisher(String, 'motor_status', 10)
        # try:
        #     self.current_limits = self.dxlMotors.getMotorsCurrentLimit(ACTIVE_MOTORS)
        # except Exception as e:
        #     self.current_limits = {}
        #     self.get_logger().warn(f'Nelze precist current limity motoru: {e}')
        # self.create_timer(MOTOR_STATUS_PERIOD, self.__publish_motor_status)

    def __publish_motor_status(self):
        try:
            temps = self.dxlMotors.getMotorsTemperature(ACTIVE_MOTORS)
            currs = self.dxlMotors.getMotorsCurrent(ACTIVE_MOTORS)
        except Exception as e:
            self.get_logger().warn(f'Cteni stavu motoru selhalo: {e}')
            return

        parts = []
        for id in ACTIVE_MOTORS:
            t = temps.get(id)
            c = currs.get(id)
            lim = self.current_limits.get(id)
            if lim:
                parts.append(f'M{id} T={t}C I={c}/{lim} ({abs(c) / lim * 100:.0f}%)')
            else:
                parts.append(f'M{id} T={t}C I={c}')

        msg = String()
        msg.data = ' | '.join(parts)
        self.motor_status_pub.publish(msg)

    def calibrateBase(self):
        self.dxlMotors.calibrateBase()

    def arrow_movement(self, msg):
        direction, dxl_id = msg.data.split()
        dxl_id = int(dxl_id)
        if dxl_id not in ACTIVE_MOTORS:
            self.get_logger().error(f'DXL ID out of active motors, dxl_id: {str(dxl_id)}')
        else:
            currPos = self.dxlMotors.getSingleMotorPos(dxl_id)

            goalPos = currPos
            if direction == 'UP':
                goalPos+=TESTING_MOTORS_SPEED*GEAR[dxl_id-1]
            else:
                goalPos-=TESTING_MOTORS_SPEED*GEAR[dxl_id-1]


            i = ACTIVE_MOTORS.index(dxl_id)
            if not self.__check_motor_limits(i, self.__stepsToQ(i, goalPos)):
                return
            
            try:
                self.dxlMotors.setSingleMotorGoalPos(goalPos, int(dxl_id))
                self.get_logger().info(f'MOVING MOTOR {dxl_id} TO POSITION {goalPos}')

                self.arrow_counter += 1
                if self.arrow_counter % ARROW_UPDATE_EVERY == 0:
                    self.__updateCurrentQ()

            except ValueError as e:
                self.get_logger().error(f'Error at DXL occured: {e}')

    # updating kinematic model
    def __updateCurrentQ(self):
        actual_steps = self.dxlMotors.getMotorsPos()

        q_motor = np.array([
            steps2rad(pos) for pos in actual_steps
        ])

        # steps of motor to the angle in joint of DH model
        self.current_q = self.directions * (q_motor - self.offsets) / GEAR

    def __pen_tilt(self, q):
        Tq = self.robot.fkine(q)
        return np.degrees(np.arccos(np.clip(-Tq.R[2, 2], -1, 1)))

    def setSingleMotorAcc(self, id, acc):
        self.dxlMotors.setSingleMotorProfileAcc(id, acc)
        self.get_logger().info(f"Setting motor {id} acc to: {acc}")

    def coordsToSteps(self, x, y, z, max_tilt_deg=25, ball=False):
        dx, dy, dz = self.__err_comp(x, y)   # kompenzace zmerene chyby hrotu ve vsech osach
        x, y, z = x + dx, y + dy, z + dz

        q = self.__solve_ik(x, y, z, max_tilt_deg, ball)
        if q is None:
            raise ValueError(f"Bod ({x}, {y}, {z}) je mimo dosah")

        self.get_logger().info(f'IK q [deg]: {np.round(np.degrees(q), 2)}, tilt: {self.__pen_tilt(q):.1f}')
        q = self.__correct_q(q)

        steps = []
        for i, angle in enumerate(q):
            a = self.directions[i] * GEAR[i] * angle + self.offsets[i]
            steps.append(rad2steps(a))

        return steps

    # vertical distance of joint 4 from base
    def __wrist_reach(self, q):
        a2 = q[1] + self.robot.links[1].offset
        a3 = a2 + q[2] + self.robot.links[2].offset
        return LINK2 * np.cos(a2) + LINK3 * np.cos(a3)

    def __compute_joint_bias(self):
        bias = np.zeros(5)
        if CALIB_MEASURED_XYZ is None:
            return bias

        q0 = np.array(CALIB_Q, dtype=float)
        err = np.array(CALIB_MEASURED_XYZ, dtype=float) - self.robot.fkine(q0).t
        J = self.robot.jacob0(q0)[:3, :3]
        bias[:3] = np.linalg.lstsq(J, err, rcond=None)[0]

        self.get_logger().info(f'Kompenzace: chyba hrotu v CALIB_Q [mm]: {np.round(err * 1000, 1)}, '
                               f'offsety kloubu 1-3 [deg]: {np.round(np.degrees(bias[:3]), 2)}')
        if np.any(np.abs(bias) > np.radians(8)):
            self.get_logger().warn('Kompenzace: offset kloubu > 8 deg, zkontroluj CALIB_MEASURED_XYZ')
        return bias

    def __correct_q(self, q):
        if CALIB_MEASURED_XYZ is None:
            return q

        delta = self.q_bias.copy()
        delta[1] += SAG_K2 * (self.__wrist_reach(q) - self.__wrist_reach(CALIB_Q))
        return np.array(q) - delta

    def __compute_err_coeffs(self):
        pts = [((xc, yc), m) for (xc, yc), m in ERR_SAMPLES if m is not None]
        n_basis = len(_err_basis(0.0, 0.0))
        if len(pts) < n_basis:
            self.get_logger().info(
                f'ERR: zmereno {len(pts)} bodu (potreba >={n_basis}) -> XYZ korekce vypnuta')
            return None

        A = np.array([_err_basis(xc, yc) for (xc, yc), _ in pts])
        B = np.array([[xc - m[0], yc - m[1], ERR_CMD_Z - m[2]]
                      for (xc, yc), m in pts])
        coeffs = np.linalg.lstsq(A, B, rcond=None)[0]

        rms_raw = np.sqrt(np.mean(B ** 2, axis=0)) * 1000
        rms_res = np.sqrt(np.mean((B - A @ coeffs) ** 2, axis=0)) * 1000
        self.get_logger().info(
            f'ERR fit z {len(pts)} bodu | RMS chyby [mm] x/y/z: {np.round(rms_raw, 2)} '
            f'-> rezidua [mm]: {np.round(rms_res, 2)}')
        return coeffs

    def __err_comp(self, x, y):
        if self.err_coeffs is None:
            return 0.0, 0.0, 0.0
        d = _err_basis(x, y) @ self.err_coeffs
        d = np.clip(d, -0.03, 0.03) # just for typo
        return float(d[0]), float(d[1]), float(d[2])

    def goToPos(self, x, y, z): 
        try:
            steps = self.coordsToSteps(x, y, z)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        self.makeMove(steps)
        
    def makeMove(self, steps):
        # check the limits
        for i, step in enumerate(steps):
            if not self.__check_motor_limits(i, self.__stepsToQ(i, step)):
                return

        self.dxlMotors.setMotorsGoalPos(steps)
        self.get_logger().info(f'Moving motors to motor position: {steps}')    

    def __motor_cmd_callback(self, msg):
        if msg.command == 'move':
            self.goToPos(msg.x, msg.y, msg.z)

    def __seeds(self, x, y):
        q1 = float(np.clip(np.arctan2(y, x), *JOINT1_LIM))
        return [np.array([q1, 0, 0, -45*pi/180, q5]) for q5 in (0.0, pi)]

    def __limit_margin(self, q):
        lo, hi = self.robot.qlim
        return np.min(np.minimum(q - lo, hi - q) / (hi - lo))

    def __solve_ik(self, x, y, z, max_tilt_deg, ball=False):
        # one for precision one for ball
        masks = [[1, 1, 1, 0, 0, 0]] if ball else [[1, 1, 1, 1, 1, 0], [1, 1, 1, 0, 0, 0]]

        for mask in masks:
            best_q, best_m = None, 0.0
            for q0 in self.__seeds(x, y):
                R0 = self.robot.fkine(q0).R
                yaw = np.arctan2(R0[1, 0], R0[0, 0])
                T = SE3(x, y, z) * SE3.Rz(yaw) * SE3.Rx(pi)

                sol = self.robot.ikine_LM(T, q0=q0, mask=mask, joint_limits=True, slimit=1)
                if not sol.success:
                    continue

                q = np.array(sol.q)
                if not ball and self.__pen_tilt(q) > max_tilt_deg:
                    continue

                m = self.__limit_margin(self.__correct_q(q))   # limits after the correction
                if m > best_m:
                    best_q, best_m = q, m

            if best_q is not None:
                return best_q
        return None


    # for cli cmd set_speed
    def __set_speed_callback(self, msg):
        dxl_id, speed = (int(v) for v in msg.data.split())
        self.setMotorSpeed(dxl_id, speed)

    # for cli cmd disable_torque
    def __disable_torque_callback(self, msg):
        self.dxlMotors.setDXLTorqueEnabled(False)

    # for cli cmd go_home
    def __go_home_callback(self, msg):
        self.dxlMotors.setMotorsGoalPos(list(HOME_POS_MOTORS.values()))
        self.get_logger().info('Going home...')

    def setMotorSpeed(self, dxl_id, speed):
        self.dxlMotors.setSingleMotorSpeed(dxl_id, speed)
        self.get_logger().info(f'Speed {speed} was set to motor {dxl_id}')

    def __stepsToQ(self, i, steps):
        self.get_logger().info(f'steps: {steps}')
        return self.directions[i] * (steps2rad(steps) - self.offsets[i]) / GEAR[i]

    def __check_motor_limits(self, i, q):
        lo, hi = self.robot.qlim[:, i]
        if not lo <= q <= hi:
            self.get_logger().error(f'Kloub {i+1} je mimo limit [{np.degrees(lo):.0f}, {np.degrees(hi):.0f}], do q:{q}')
            return False
        return True

    def __get_robot_config(self):
        robot = rtb.DHRobot(
                [
                    rtb.RevoluteDH(d=LINK1, alpha=pi/2, qlim=JOINT1_LIM), 
                    rtb.RevoluteDH(a=LINK2, qlim=JOINT2_LIM, offset=45*pi/180),
                    rtb.RevoluteDH(a=LINK3, qlim=JOINT3_LIM, offset=-90*pi/180),
                    rtb.RevoluteDH(a=LINK4_OFFSET, alpha=pi/2, qlim=JOINT4_LIM, offset=90*pi/180), 
                    rtb.RevoluteDH(d=LINK4, a=LINK5_OFFSET, qlim=JOINT5_LIM), 
                ], name="Robot arm")
        robot.qr = ROBOT_QR

        return robot
        
    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def __is_home_point(self, point):
        return (point.x == HOME_SENTINEL
                and point.y == HOME_SENTINEL
                and point.z == HOME_SENTINEL)

    # ball & precision actions
    def __wait_until_stopped(self, goal_handle, move_start_delay = 0.2, time_sleep = 0.1):
        time.sleep(move_start_delay)

        while self.dxlMotors.getMotorsMoving():
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(time_sleep)
        return True

    def __move_to_point(self, point, speeds, max_tilt_deg=25):
        self.dxlMotors.setMotorsSpeed(speeds)

        if self.__is_home_point(point):
            self.get_logger().info('Moving to HOME')
            self.makeMove([HOME_POS_MOTORS[m] for m in ACTIVE_MOTORS])
        else:
            steps = self.coordsToSteps(point.x, point.y, point.z, max_tilt_deg)
            self.get_logger().info(f'Moving to point: speeds={speeds}, steps={steps}')
            self.makeMove(steps)


    async def __run_ball_sequence(self, goal_handle, points, speeds):
        settle_idx = goal_handle.request.settle_point_index
        self.setSingleMotorAcc(1, 50)
        self.setSingleMotorAcc(2, 120)

        moves = [
            [HOME_POS_MOTORS[m] for m in ACTIVE_MOTORS]
            if self.__is_home_point(point)
            else self.coordsToSteps(point.x, point.y, point.z, ball=True)
            for point in points
        ]

        for i, steps in enumerate(moves):
            if i != 4:
                if not self.__wait_until_stopped(goal_handle, 0.1, 0.05):
                    return False
            else:
                if not self.__wait_until_stopped(goal_handle):
                    return False

            self.dxlMotors.setMotorsSpeed(speeds[i].speeds)
            self.makeMove(steps)

            if i == settle_idx:
                if not self.__wait_until_stopped(goal_handle):
                    return False
                time.sleep(BALL_SETTLE_TIME)

        self.setSingleMotorAcc(2, 0)

        return self.__wait_until_stopped(goal_handle)

    async def __run_precision_sequence(self, goal_handle, points, speeds):
        dwell_idx = goal_handle.request.settle_point_index
        for i, point in enumerate(points):
            if not self.__wait_until_stopped(goal_handle):
                return False

            self.__move_to_point(point, speeds[i].speeds, PRECISION_MAX_TILT_DEG)

            if not self.__wait_until_stopped(goal_handle):
                return False

            if (i == 0):
                time.sleep(SETTLE_TIME_APPROACH_1)
            elif (i == 1):
                time.sleep(SETTLE_TIME_APPROACH_2)
            elif (i == dwell_idx):
                time.sleep(DWELL_SETTLE_TIME)
            else:
                time.sleep(SETTLE_TIME_LOW)

        return True

    async def execute_callback(self, goal_handle):
        points = goal_handle.request.points
        speeds = goal_handle.request.speeds

        try:
            if len(points) != len(speeds):
                raise ValueError("Number of points and speeds does not match")

            if goal_handle.request.is_ball_action:
                completed = await self.__run_ball_sequence(goal_handle, points, speeds)
            else:
                completed = await self.__run_precision_sequence(goal_handle, points, speeds)

            if not completed:
                goal_handle.canceled()
                return Motor.Result()

            goal_handle.succeed()
            self.calibrateBase()
            self.get_logger().info('Base was calibrated')
            return Motor.Result()

        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            goal_handle.abort()

            return Motor.Result()


               
        
def main():
    rclpy.init()
    node = DXLMotorsController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
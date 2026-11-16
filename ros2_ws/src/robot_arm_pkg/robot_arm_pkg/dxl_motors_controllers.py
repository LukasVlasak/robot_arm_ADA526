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


# TODO: self.current_q SE TED NEUPDATUJE - ZATIM NEPOUZIVAM TAKZE OK 


HOME_POS_MOTORS = {
    1: 2109,
    2: -3240,
    3: 9019,
    4: 2828,
    5: 2,
}
WEIGHT_POS_MOTORS = {
    1: 1250,
    2: 1250,
    3: 1250,
    4: 1250,
    5: 1250,
}

# bod (-1, -1, -1) v Motor akci znamena "jed do HOME pozice"
HOME_SENTINEL = -1.0

# jak dlouho cekat po dojezdu na bod, nez se rameno dokmita [s]
# precision ma 2 approach vysky nad teckou: prvni (vyssi) a druha (nizsi)
SETTLE_TIME_APPROACH_1 = 2   # prvni (vyssi) priblizeni
SETTLE_TIME_APPROACH_2 = 4   # druhe (nizsi) priblizeni
SETTLE_TIME_LOW = 0.1

# delsi ustaleni na dwell bodu (precision: settle_point_index z Motor goalu) [s]
DWELL_SETTLE_TIME = 2

# ustaleni na odpalovem bodu pred rychlym svihem (ball: settle_point_index) [s]
BALL_SETTLE_TIME = 1.5

# precision potrebuje pero vic kolmo k desce nez micek -> mensi povoleny naklon [deg]
PRECISION_MAX_TILT_DEG = 10

# pauza po zadani pohybu, nez motory nabehnou a is_moving spolehlive hlasi pohyb [s]
MOVE_START_DELAY = 0.2

LINK1 = 0.220
LINK2 = 0.300
LINK3 = 0.280
LINK4 = 0.202
LINK4_OFFSET = -0.00262
LINK5_OFFSET = 0.043

JOINT1_LIM = [-90*pi/180, 90*pi/180]
JOINT2_LIM = [ -70*pi/180, 70*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT3_LIM = [-50*pi/180, 180*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT4_LIM = [ -92*pi/180, 85*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT5_LIM = [-180*pi/180, 180*pi/180]

# jsem dat primo namerenou hodnotu curr pos motoru kdyz bude sedet s DH modelem
DH_OFFSETS = [2109, -2064, 8167, 2313, 2]
DIRECTIONS = [1, -1, -1, -1, 1]

R_FLIP = 0.2 # kdy pero natocito 180 jako vychozi hodnota solveru

ROBOT_QR = [0, 0, 0, 0, 0]

ACTIVE_MOTORS = [1, 2, 3, 4, 5]

TESTING_MOTORS_SPEED = 250

GEAR = np.array([1, 4, 3, 1, 1]) # kdyz nebude fungovat tak toto upravit 

# just for testing
ARROW_UPDATE_EVERY = 20 # update current_q every 10 steps

# --- kompenzace pruhybu ---
# realne zmerena poloha hrotu [x, y, z] v metrech v kalibracni poloze CALIB_Q se ZAPNUTYM torque
# po kazde nove kalibraci zmerit a prepsat; None = kompenzace vypnuta
CALIB_MEASURED_XYZ = [0.587, 0.0, 0.104]
CALIB_Q = [0, 0, 0, 0, 0]
# pruhyb ramene (kloub 2) podle vodorovneho dosahu zapesti [rad/m], z fitu namerenych bodu
SAG_K2 = -0.051

# --- zbytkova kompenzace pruhybu primo v Z (dolad po vyse uvedene kompenzaci) ---
SAG_Z_CMD = 0.005   # vyska [m], na ktere meris vsechny body (y = 0)
SAG_Z_SAMPLES = [
    # (x_real [m],  z_skutecne_zmerene [m])   # None = jeste nezmereno
    (0.550, 0.004), # 0.004
    (0.500, 0.004), # 0.004
    (0.450, 0.004), # 0.004
    (0.400, 0.003),# 0.003
    (0.350, 0.003),# 0.003
    (0.300, 0.002),# 0.002
    (0.250, 0.001),
]

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
        self.sag_z_coeffs = self.__compute_sag_z_coeffs()

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
    # dat do moving pomoci tech arrows
    # ale volat to ne pokazde ale po chvili
    def __updateCurrentQ(self):
        actual_steps = self.dxlMotors.getMotorsPos()

        q_motor = np.array([
            steps2rad(pos) for pos in actual_steps
        ])

        # kroky motoru na uhel v kloubu DH modelu
        self.current_q = self.directions * (q_motor - self.offsets) / GEAR

    def __pen_tilt(self, q):
        Tq = self.robot.fkine(q)
        return np.degrees(np.arccos(np.clip(-Tq.R[2, 2], -1, 1)))

    def setSingleMotorAcc(self, id, acc):
        self.dxlMotors.setSingleMotorProfileAcc(id, acc)
        self.get_logger().info(f"Setting motor {id} acc to: {acc}")

    def coordsToSteps(self, x, y, z, max_tilt_deg=25, ball=False):
        """
            max_tilt_deg o jak moc muze bejt pero nakloneno k base
            tudiz u precision mene, nebo jak moc jsme schopny aby to bylo aby to jeste udelalo tecku
            u micku nam to je jedno

            ball=True -> IK resi POUZE pozici (maska 1,1,1,0,0,0), orientace pera je
            jedno, takze pero muze dojet sikmo. Kolme reseni je pro micky kravina.
        """
        z = z + self.__sag_z(x, y)   # zbytkova kompenzace pruhybu v Z (viz SAG_Z_SAMPLES)
        T = SE3(x, y, z) * SE3.Rx(np.pi)

        q = self.__solve_ik(T, x, y, max_tilt_deg, ball)
        if q is None:
            raise ValueError(f"Bod ({x}, {y}, {z}) je mimo dosah")

        self.get_logger().info(f'IK q [deg]: {np.round(np.degrees(q), 2)}, tilt: {self.__pen_tilt(q):.1f}')
        q = self.__correct_q(q)

        steps = []
        for i, angle in enumerate(q):
            a = self.directions[i] * GEAR[i] * angle + self.offsets[i]
            steps.append(rad2steps(a))

        return steps

    # vodorovna vzdalenost zapesti (kloub 4) od osy zakladny
    def __wrist_reach(self, q):
        a2 = q[1] + self.robot.links[1].offset
        a3 = a2 + q[2] + self.robot.links[2].offset
        return LINK2 * np.cos(a2) + LINK3 * np.cos(a3)

    # z rozdilu mezi zmerenou a modelovou polohou hrotu v CALIB_Q dopocita offsety kloubu 1-3
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

    # realny uhel kloubu = zadany + delta, takze zadavame q - delta
    def __correct_q(self, q):
        if CALIB_MEASURED_XYZ is None:
            return q

        delta = self.q_bias.copy()
        delta[1] += SAG_K2 * (self.__wrist_reach(q) - self.__wrist_reach(CALIB_Q))
        return np.array(q) - delta

    # z namerenych bodu nafituje Dz(r) = a*r + b*r^2 (bez konst. clenu -> u zakladny 0)
    def __compute_sag_z_coeffs(self):
        pts = [(x, zm) for (x, zm) in SAG_Z_SAMPLES if zm is not None]
        if len(pts) < 2:
            self.get_logger().info(
                f'SAG_Z: zmereno {len(pts)} bodu (potreba >=2) -> Z korekce vypnuta')
            return None

        r = np.array([abs(x) for x, _ in pts])            # y=0 pri mereni -> r = |x|
        sag = np.array([SAG_Z_CMD - zm for _, zm in pts])  # kladne = hrot byl niz nez zadano
        A = np.vstack([r, r ** 2]).T                       # Dz = a*r + b*r^2
        coeffs = np.linalg.lstsq(A, sag, rcond=None)[0]

        resid = sag - A @ coeffs
        self.get_logger().info(
            f'SAG_Z fit z {len(pts)} bodu: a={coeffs[0]:.4f}, b={coeffs[1]:.4f} | '
            f'rezidua [mm]: {np.round(resid * 1000, 2)}')
        return coeffs

    # o kolik [m] zvednout zadanou z v bode (x, y), aby se vyrusil zbytkovy pruhyb
    def __sag_z(self, x, y):
        if self.sag_z_coeffs is None:
            return 0.0
        r = np.hypot(x, y)
        a, b = self.sag_z_coeffs
        dz = float(np.clip(a * r + b * r * r, -0.03, 0.03))  # pojistka proti preklepu v mereni
        return dz

    def goToPos(self, x, y, z): # hlidat max min motoru
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
        r = np.hypot(x, y)
        q5 = pi if r < R_FLIP else 0.0          

        preferred = np.array([0, 0, 0, -45*pi/180, q5])
        other     = np.array([0, 0, 0, -45*pi/180, pi - q5])
        return [preferred, other]

    def __solve_ik(self, T, x, y, max_tilt_deg, ball=False):
        seeds = self.__seeds(x, y)

        # micek: staci dojet na POZICI, orientace (naklon) pera je jedno -> pero
        # klidne sikmo. Zadne hledani kolmo, zadne omezeni naklonu.
        if ball:
            for q0 in seeds:
                sol = self.robot.ikine_LM(T, q0=q0, mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
                if sol.success:
                    return np.array(sol.q)
            return None

        for mask in ([1, 1, 1, 1, 1, 0], [1, 1, 1, 0, 0, 0]):   # nejdriv kolmo, pak fallback
            for q0 in seeds:                                    # nejdriv preferovana vetev
                sol = self.robot.ikine_LM(T, q0=q0, mask=mask, joint_limits=True)
                if sol.success and self.__pen_tilt(sol.q) <= max_tilt_deg:
                    return np.array(sol.q)
        return None


    # pro cli set_speed
    def __set_speed_callback(self, msg):
        dxl_id, speed = (int(v) for v in msg.data.split())
        self.setMotorSpeed(dxl_id, speed)

    # pro cli disable_torque
    def __disable_torque_callback(self, msg):
        self.dxlMotors.setDXLTorqueEnabled(False)

    # pro cli go_home
    def __go_home_callback(self, msg):
        self.dxlMotors.setMotorsGoalPos(list(HOME_POS_MOTORS.values()))
        self.get_logger().info('Going home...')

    def setMotorSpeed(self, dxl_id, speed):
        self.dxlMotors.setSingleMotorSpeed(dxl_id, speed)
        self.get_logger().info(f'Speed {speed} was set to motor {dxl_id}')

    # kroky motoru na uhel v kloubu v DH modelu
    def __stepsToQ(self, i, steps):
        self.get_logger().info(f'steps: {steps}')
        return self.directions[i] * (steps2rad(steps) - self.offsets[i]) / GEAR[i]

    # i je index kloubu -> 0 - 4
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
                    rtb.RevoluteDH(a=LINK4_OFFSET, alpha=pi/2, qlim=JOINT4_LIM, offset=90*pi/180), # tady kdyztak a misto d
                    rtb.RevoluteDH(d=LINK4, a=LINK5_OFFSET, qlim=JOINT5_LIM), # tady kdyztak a misto d a d misto a 
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

    # --------------------------------------------------
    # SPOLECNE POHYBOVE METODY (ball i precision)
    # --------------------------------------------------

    # ceka dokud se motory tocej; vraci False pokud prisel cancel
    def __wait_until_stopped(self, goal_handle, move_start_delay = 0.2, time_sleep = 0.1):
        # kratka pauza, aby motory stacily nabehnout a is_moving spolehlive hlasil pohyb
        time.sleep(move_start_delay)

        while self.dxlMotors.getMotorsMoving():
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(time_sleep)
        return True

    # nastavi rychlosti (pole - jedna na motor) a jede na jeden bod (karteziansky, nebo HOME sentinel)
    def __move_to_point(self, point, speeds, max_tilt_deg=25):
        self.dxlMotors.setMotorsSpeed(speeds)

        if self.__is_home_point(point):
            self.get_logger().info('Moving to HOME')
            self.makeMove([HOME_POS_MOTORS[m] for m in ACTIVE_MOTORS])
        else:
            steps = self.coordsToSteps(point.x, point.y, point.z, max_tilt_deg)
            self.get_logger().info(f'Moving to point: speeds={speeds}, steps={steps}')
            self.makeMove(steps)

    # --------------------------------------------------
    # SEKVENCE BODU
    # --------------------------------------------------

    # ball: DULEZITE - spocita vsechnu kinematiku DOPREDU (soutez meri cas pohybu),
    # pak jede body za sebou a ceka jen na dokonceni pohybu.
    # HOME sentinel se neresi pres IK - jedou se pevne kroky HOME_POS_MOTORS.
    # Na odpalovem bodu (settle_point_index) necha rameno ustalit pred rychlym svihem.
    async def __run_ball_sequence(self, goal_handle, points, speeds):
        settle_idx = goal_handle.request.settle_point_index  # -1 = zadny
        self.setSingleMotorAcc(1, 50)
        self.setSingleMotorAcc(2, 120)

        moves = [
            [HOME_POS_MOTORS[m] for m in ACTIVE_MOTORS]
            if self.__is_home_point(point)
            else self.coordsToSteps(point.x, point.y, point.z, ball=True)
            for point in points
        ]

        for i, steps in enumerate(moves):
            if not self.__wait_until_stopped(goal_handle, 0.1, 0.05):
                return False

            self.dxlMotors.setMotorsSpeed(speeds[i].speeds)
            self.makeMove(steps)

            # odpalovy bod pred svihem - nechat rameno ustalit, nez vyrazi BOOM
            if i == settle_idx:
                if not self.__wait_until_stopped(goal_handle):
                    return False
                time.sleep(BALL_SETTLE_TIME)

        self.setSingleMotorAcc(2, 0)

        return self.__wait_until_stopped(goal_handle)

    # precision: jako ball, ale po dojezdu na kazdy bod necha rameno SETTLE_TIME ustalit
    async def __run_precision_sequence(self, goal_handle, points, speeds):
        dwell_idx = goal_handle.request.settle_point_index  # -1 = zadny
        for i, point in enumerate(points):
            if not self.__wait_until_stopped(goal_handle):
                return False

            self.__move_to_point(point, speeds[i].speeds, PRECISION_MAX_TILT_DEG)

            if not self.__wait_until_stopped(goal_handle):
                return False
            # po najeti nad bod, pockat nez se dokmita

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

            # is_ball_action flag z Motor goalu rozhoduje: ball predpocita vse dopredu
            # a jede bez ustalovani, precision necha po kazdem bodu rameno ustalit
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
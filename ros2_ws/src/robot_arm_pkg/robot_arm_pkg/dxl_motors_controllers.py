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
    1: 1250,
    2: 1250,
    3: 1250,
    4: 1250,
    5: 1250,
}
WEIGHT_POS_MOTORS = {
    1: 1250,
    2: 1250,
    3: 1250,
    4: 1250,
    5: 1250,
}

LINK1 = 0.220
LINK2 = 0.300
LINK3 = 0.280
LINK4 = 0.202
LINK4_OFFSET = -0.00262
LINK5_OFFSET = 0.043

JOINT1_LIM = [-90*pi/180, 90*pi/180]
JOINT2_LIM = [ -70*pi/180, 70*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT3_LIM = [-90*pi/180, 180*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT4_LIM = [ -92*pi/180, 85*pi/180] # odecetl jsem 5 stupnu v zapornych i v kladnych kvuli testovani
JOINT5_LIM = [-180*pi/180, 180*pi/180]

# jsem dat primo namerenou hodnotu curr pos motoru kdyz bude sedet s DH modelem
DH_OFFSETS = [2112, 2009, 4172, 2330, 16]
DIRECTIONS = [1, -1, -1, -1, 1]

R_FLIP = 0.2 # kdy pero natocito 180 jako vychozi hodnota solveru

ROBOT_QR = [0, 0, 0, 0, 0]

ACTIVE_MOTORS = [1, 2, 3, 4, 5]

TESTING_MOTORS_SPEED = 250

GEAR = np.array([1, 4, 3, 1, 1]) # kdyz nebude fungovat tak toto upravit 

# just for testing
ARROW_UPDATE_EVERY = 20 # update current_q every 10 steps

class DXLMotorsController(Node):
    def __init__(self):
        super().__init__("dxl_motors_controllers")
        self.dxlMotors = dxl_motors.DXLMotors(ACTIVE_MOTORS)
        
        self.robot = self.__get_robot_config()
        self.offsets = np.array([steps2rad(s) for s in DH_OFFSETS])
        self.directions = DIRECTIONS
        self.current_q = np.array(self.robot.qr, dtype=float)
        self.arrow_counter = 0
        
        self.create_subscription(String, 'arrow_user_input', self.arrow_movement, 10)
        self.create_subscription(String, 'set_speed', self.__set_speed_callback, 10)
        self.create_subscription(Empty, 'disable_torque', self.__disable_torque_callback, 10)
        self.create_subscription(MotorCommand, '/motor_controller/command', self.__motor_cmd_callback, 10)

        # print(self.robot)
        # print(self.robot.fkine([0,0,0,0,0]))
        # self.robot.plot([0,0,0,0,0], block=True)

        self._action_server = ActionServer(
            self,
            Motor,
            'motor',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

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

    def coordsToSteps(self, x, y, z, max_tilt_deg=25):
        """
            max_tilt_deg o jak moc muze bejt pero nakloneno k base
            tudiz u precision mene, nebo jak moc jsme schopny aby to bylo aby to jeste udelalo tecku
            u micku nam to je jedno
        """
        T = SE3(x, y, z) * SE3.Rx(np.pi)          

        q = self.__solve_ik(T, x, y, max_tilt_deg)
        if q is None:
            raise ValueError(f"Bod ({x}, {y}, {z}) je mimo dosah")

        steps = []
        for i, angle in enumerate(q):
            a = self.directions[i] * GEAR[i] * angle + self.offsets[i]
            steps.append(rad2steps(a))

        return steps

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

    def __solve_ik(self, T, x, y, max_tilt_deg):
        seeds = self.__seeds(x, y)

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

    async def execute_callback(self, goal_handle):
        points = goal_handle.request.points
        speeds = goal_handle.request.speeds

        try:
            moves = [
                self.coordsToSteps(point.x, point.y, point.z)
                for point in points
            ]

            if len(moves) != len(speeds):
                raise ValueError("Number of moves and speeds does not match")

            for i in range(len(moves)):

                while self.dxlMotors.getMotorsMoving():
                    time.sleep(0.1)

                # if goal_handle.is_cancel_requested:
                #     goal_handle.canceled()
                #     return Motor.Result()
                print(speeds[i])
                print(moves[i])
                self.dxlMotors.setMotorsSpeed(int(speeds[i]))
                self.makeMove(moves[i])

            
            # Wait for the last movement to finish
            while self.dxlMotors.getMotorsMoving():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Motor.Result()

                time.sleep(0.1)

            goal_handle.succeed()

            result = Motor.Result()
            return result

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

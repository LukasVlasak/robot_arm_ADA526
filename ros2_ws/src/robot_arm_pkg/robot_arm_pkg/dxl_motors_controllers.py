import dxl_motors
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

HOME_POS_MOTOR_1 = 1250
HOME_POS_MOTOR_2 = 1250
HOME_POS_MOTOR_3 = 1250
HOME_POS_MOTOR_4 = 2363
HOME_POS_MOTOR_5 = 2975

MIN_POS_MOTOR_1 = 1500
MIN_POS_MOTOR_2 = 1500
MIN_POS_MOTOR_3 = 1500
MIN_POS_MOTOR_4 = 1412
MIN_POS_MOTOR_5 = 0

MAX_POS_MOTOR_1 = 1500
MAX_POS_MOTOR_2 = 1500
MAX_POS_MOTOR_3 = 1500
MAX_POS_MOTOR_4 = 3073
MAX_POS_MOTOR_5 = 4095

HOME_POS_MOTORS = {
    1: HOME_POS_MOTOR_1,
    2: HOME_POS_MOTOR_2,
    3: HOME_POS_MOTOR_3,
    4: HOME_POS_MOTOR_4,
    5: HOME_POS_MOTOR_5,
}
MIN_POS_MOTORS = {
    1: MIN_POS_MOTOR_1,
    2: MIN_POS_MOTOR_2,
    3: MIN_POS_MOTOR_3,
    4: MIN_POS_MOTOR_4,
    5: MIN_POS_MOTOR_5,
}
MAX_POS_MOTORS = {
    1: MAX_POS_MOTOR_1,
    2: MAX_POS_MOTOR_2,
    3: MAX_POS_MOTOR_3,
    4: MAX_POS_MOTOR_4,
    5: MAX_POS_MOTOR_5,
}

ACTIVE_MOTORS = [4,5]

TESTING_MOTORS_SPEED = 25

class DXLMotorsController(Node):
    def __init__(self):
        super().__init__("dxl_motors_controllers")
        self.dxlMotors = dxl_motors.DXLMotors(ACTIVE_MOTORS)

        self.arrowTestingSub = self.create_subscription(String, 'arrow_user_input', self.arrow_movement, 10)

    def arrow_movement(self, msg):
        direction, dxl_id = msg.data.split()
        dxl_id = int(dxl_id)
        if int(dxl_id) not in ACTIVE_MOTORS:
            self.get_logger().error(f'DXL ID out of active motors, dxl_id: {str(dxl_id)}')
        else:
            #raw = self.deg2raw(2, HOME_POS_MOTORS[dxl_id], MIN_POS_MOTORS[dxl_id], MAX_POS_MOTORS[dxl_id])
            currPos = self.dxlMotors.getSingleMotorPos(dxl_id)

            goalPos = currPos
            if direction == 'UP':
                goalPos+=TESTING_MOTORS_SPEED
            else:
                goalPos-=TESTING_MOTORS_SPEED
            
            try:
                self.dxlMotors.setSingleMotorGoalPos(goalPos, int(dxl_id))
                self.get_logger().info(f'MOVING MOTOR {dxl_id} TO POSITION {goalPos}')
            except ValueError as e:
                self.get_logger().error(f'Error at DXL occured: {e}')
                
                 
        
def main():
    rclpy.init()
    node = DXLMotorsController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

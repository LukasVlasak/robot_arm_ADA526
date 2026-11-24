import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robot_interfaces.action import Ball
from robot_interfaces.action import Precision
from robot_interfaces.msg import MotorCommand
from action_msgs.msg import GoalStatus


class OperatorNode(Node):

    def __init__(self):
        super().__init__('operator_node')

        self.motor_publisher = self.create_publisher(
            MotorCommand,
            '/motor_controller/command',
            10
        )

        self.ball_client = ActionClient(
            self,
            Ball,
            'ball'
        )

        self.precision_client = ActionClient(
            self,
            Precision,
            'precision'
        )

        self.get_logger().info('Operator node initialized')

        self.input_thread = threading.Thread(
            target=self.command_loop,
            daemon=True
        )

        self.input_thread.start()

    def command_loop(self):

        print('\n===== ROBOT OPERATOR =====')
        print('Available commands:')
        print('  home')
        print('  weight')
        print('  move K 10 z')
        print('  precision K 10')
        print('  ball green')
        print('  ball red')
        print('  exit')
        print('==========================\n')

        while rclpy.ok():
            try:
                user_input = input('Robot > ').strip().lower()

                if not user_input:
                    continue

                args = user_input.split()
                command = args[0]

                if command == 'home':
                    self.publish_preset('home')

                elif command == 'weight':
                    self.publish_preset('weight')

                elif command == 'move':
                    self.handle_move(args)

                elif command == 'precision':
                    self.handle_precision(args)

                elif command == 'ball':
                    self.handle_ball(args)

                elif command == 'exit':
                    rclpy.shutdown()
                    break

                else:
                    print('Unknown command!')

            except ValueError:
                print('Invalid input')

            except EOFError:
                break

    # ==========================================
    # PUBLISH PREDEFINED COMMANDS
    # ==========================================

    def publish_preset(self, command):

        msg = MotorCommand()
        msg.command = command

        self.motor_publisher.publish(msg)

        self.get_logger().info(
            f'Published motor command: {command}'
        )

    # ==========================================
    # PUBLISH CARTESIAN POSITION
    # ==========================================

    def handle_move(self, args):

        if len(args) != 4:
            print('Usage: move C 12 0.1')
            return

        letter = args[1]
        number = int(args[2])
        z = float(args[3])

        x, y = self.__convert_coordinates(letter, number)

        msg = MotorCommand()

        msg.command = 'move'
        msg.x = x
        msg.y = y
        msg.z = z

        self.motor_publisher.publish(msg)

        self.get_logger().info(
            f'Published position: x={x}, y={y}, z={z}'
        )

    # ==========================================
    # PRECISION ACTION CLIENT
    # ==========================================

    def handle_precision(self, args):

        if len(args) != 3:
            print('Usage: precision K 10')
            return

        letter = args[1]
        number = int(args[2])

        x, y = self.__convert_coordinates(letter, number)
        z = 0.005

        if not self.precision_client.wait_for_server(timeout_sec=5.0):
            print('Precision action server not available')
            return

        goal = Precision.Goal()
        goal.x = x
        goal.y = y
        goal.z = z

        self.get_logger().info(
            f'Sending Precision goal: x={x}, y={y}, z={z}'
        )

        send_goal_future = self.precision_client.send_goal_async(goal)

        event = threading.Event()
        result_data = {}

        def result_callback(future):
            try:
                result_data['result'] = future.result()
            except Exception as e:
                result_data['error'] = e
            finally:
                event.set()

        def goal_response_callback(future):
            try:
                goal_handle = future.result()

                if not goal_handle.accepted:
                    result_data['accepted'] = False
                    event.set()
                    return

                result_data['accepted'] = True

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)

            except Exception as e:
                result_data['error'] = e
                event.set()

        send_goal_future.add_done_callback(goal_response_callback)

        print('Waiting for PrecisionNode...')

        event.wait()

        if 'error' in result_data:
            print(f'Precision action error: {result_data["error"]}')
            return

        if not result_data.get('accepted', False):
            print('Precision goal rejected')
            return

        response = result_data['result']

        if response.status == GoalStatus.STATUS_SUCCEEDED:
            if response.result.success:
                print(f'Precision action successful: {response.result.message}')
            else:
                print(f'Precision action failed: {response.result.message}')

        elif response.status == GoalStatus.STATUS_ABORTED:
            print('Precision action aborted')

        elif response.status == GoalStatus.STATUS_CANCELED:
            print('Precision action canceled')

        else:
            print(f'Unknown result status: {response.status}')

    # ==========================================
    # BALL ACTION CLIENT
    # ==========================================

    def handle_ball(self, args):

        if len(args) != 2:
            print('Usage: ball <green|red>')
            return

        color = args[1]

        if color not in ['green', 'red']:
            print('Color must be green or red')
            return

        if not self.ball_client.wait_for_server(timeout_sec=5.0):
            print('Ball action server not available')
            return

        goal = Ball.Goal()
        goal.first_ball = color

        self.get_logger().info(
            f'Sending Ball goal: first_ball={color}'
        )

        send_goal_future = self.ball_client.send_goal_async(
            goal,
            feedback_callback=self.ball_feedback_callback
        )

        event = threading.Event()
        result_data = {}

        def result_callback(future):
            try:
                result_data['result'] = future.result()
            except Exception as e:
                result_data['error'] = e
            finally:
                event.set()

        def goal_response_callback(future):

            try:
                goal_handle = future.result()

                if not goal_handle.accepted:
                    result_data['accepted'] = False
                    event.set()
                    return

                result_data['accepted'] = True

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)

            except Exception as e:
                result_data['error'] = e
                event.set()

        send_goal_future.add_done_callback(goal_response_callback)

        print('Waiting for BallNode...')

        event.wait()

        if 'error' in result_data:
            print(f'Ball action error: {result_data["error"]}')
            return

        if not result_data.get('accepted', False):
            print('Ball goal rejected')
            return

        response = result_data['result']

        if response.status == GoalStatus.STATUS_SUCCEEDED:

            if response.result.success:
                print(f'Ball action successful: {response.result.message}')
            else:
                print(f'Ball action failed: {response.result.message}')

        elif response.status == GoalStatus.STATUS_ABORTED:
            print('Ball action aborted')

        elif response.status == GoalStatus.STATUS_CANCELED:
            print('Ball action canceled')

        else:
            print(f'Unknown result status: {response.status}')

    def ball_feedback_callback(self, feedback_msg):

        progress = feedback_msg.feedback.progress

        print(f'\rBall action progress: {progress:.1f}%', end='', flush=True)

    # ==========================================
    # COORDINATE CONVERSION
    # ==========================================

    def __convert_coordinates(self, letter, number):

        number = int(number)

        x = 600 - (number) * 25
        x = float(x / 1000)

        ascii_code = int(ord(letter))

        y = 250 + (ascii_code - 117) * 25
        y = float(y / 1000)

        return x, y


def main(args=None):

    rclpy.init(args=args)

    node = OperatorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':
    main()


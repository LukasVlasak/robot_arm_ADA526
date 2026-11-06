import threading

import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_interfaces.action import Precision

from robot_interfaces.msg import MotorCommand

class OperatorNode(Node):
	def __init__(self):
		super().__init__('operator_node')

	    # Publisher
		self.motor_publisher = self.create_publisher(MotorCommand, '/motor_controller/command', 10)

		self.get_logger().info('Operator node initialized')

		# CLI input thread
		self.input_thread = threading.Thread(
		    target=self.command_loop,
		    daemon=True
		)

		self.input_thread.start()

# ==========================================
# CLI
# ==========================================

	def command_loop(self):

		print('\n===== ROBOT OPERATOR =====')
		print('Available commands:')
		print('  home')
		print('  weight')
		print('  move x y z')
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

				elif command == 'exit':
					rclpy.shutdown()
					break

				else:
					print('Unknown command!')

			except ValueError:
				print('Invalid input!')

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
			print('Usage: move x y z')
			return

		x, y, z = map(float, args[1:])

		msg = MotorCommand()

		msg.command = 'move'
		msg.x = x
		msg.y = y
		msg.z = z

		self.motor_publisher.publish(msg)

		self.get_logger().info(
		    f'Published position: x={x}, y={y}, z={z}'
		)


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


if name == 'main': main()

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pynput import keyboard
import termios
import sys
import subprocess

class ArrowMovementTestingPub(Node):
    """
        Posila left, down, up nebo right mezeru a kolik asi sekund to bylo drzene
        speed budou uhly za sekundu
    """
    def __init__(self):
        # set window title to check for active window
        self.window_title = "ROS2_MOVEMENT_CONTROL"
        sys.stdout.write(f"\x1b]2;{self.window_title}\x07")
        sys.stdout.flush()

        super().__init__("arrow_movement_testing_pub")
        self.pub = self.create_publisher(String, 'arrow_user_input', 10)

        self.active_motor_id = 1

        # vypnuti echa v terminalu
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self.disable_echo()
        
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.get_logger().info('Listening for arrow key input, active motor id is 1')

    def disable_echo(self):
        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] = new_settings[3] & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new_settings)

    def restore_echo(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def is_window_active(self):
        try:
            window_name = subprocess.check_output(["xdotool", "getactivewindow", "getwindowname"], stderr=subprocess.DEVNULL).decode("utf-8", errors="ignore")

            return self.window_title in window_name
        except Exception:
            return False

    def on_press(self, key):
        if not self.is_window_active():
            return

        move_directions = {
            keyboard.Key.up: 'UP',
            keyboard.Key.down: 'DOWN',
        }

        if key in move_directions:
            msg = String()
            msg.data = f'{move_directions[key]} {self.active_motor_id}'
            self.pub.publish(msg)

        if key == keyboard.Key.left:
            self.active_motor_id -= 1
            self.get_logger().info(f'Active motor ID is now {self.active_motor_id}')

        if key == keyboard.Key.right:
            self.active_motor_id += 1
            self.get_logger().info(f'Active motor ID is now {self.active_motor_id}')

            

def main():
    rclpy.init()

    node = ArrowMovementTestingPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: # pro cistejsi ukonceni
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        node.restore_echo()
        rclpy.shutdown()

    rclpy.shutdown()

if __name__ == 'main':
    main()
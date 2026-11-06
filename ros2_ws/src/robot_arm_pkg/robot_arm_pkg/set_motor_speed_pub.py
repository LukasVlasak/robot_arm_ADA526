import sys
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

def main():
    if (len(sys.argv) != 3):
        print("Pouziti: set_speed <dxl_id> <speed>")
        return

    rclpy.init()
    node = Node('set_motor_speed_cli')
    pub = node.create_publisher(String, 'set_speed', 10)

    deadline = time.time() + 2.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        print("Motor controller nebezi")
    else:
        pub.publish(String(data=f'{sys.argv[1]} {sys.argv[2]}'))
        rclpy.spin_once(node, timeout_sec=0.2) # at se zprava stihne poslat

    node.destroy_node()
    rclpy.shutdown()
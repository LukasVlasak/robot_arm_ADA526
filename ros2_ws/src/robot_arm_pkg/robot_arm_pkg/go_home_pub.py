import sys
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Empty

def main():
    if (len(sys.argv) != 1):
        print("Pouziti: go_home")
        return

    rclpy.init()
    node = Node('go_home_cli')
    pub = node.create_publisher(Empty, 'go_home', 10)

    deadline = time.time() + 2.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        print("Motor controller nebezi")
    else:
        pub.publish(Empty())
        rclpy.spin_once(node, timeout_sec=0.2) 

    node.destroy_node()
    rclpy.shutdown()
import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from geometry_msgs.msg import PointStamped


class BallTransformer(Node):

    def __init__(self):
        super().__init__('ball_transformer')


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # Subscribers
        self.sub_green = self.create_subscription(
            PointStamped,
            '/goal_green_point',
            self.green_callback,
            10
        )

        self.sub_red = self.create_subscription(
            PointStamped,
            '/goal_red_point',
            self.red_callback,
            10
        )

        # Publishers
        self.pub_green = self.create_publisher(
            PointStamped,
            '/goal_green_base',
            10
        )

        self.pub_red = self.create_publisher(
            PointStamped,
            '/goal_red_base',
            10
        )

        self.get_logger().info('BallTransformer started')

    def transform_point(self, msg, publisher, color):

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_frame',
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            transformed = do_transform_point(msg, transform)

            publisher.publish(transformed)

            self.get_logger().info(
                f'{color} ball in base_frame: '
                f'x={transformed.point.x:.3f}, '
                f'y={transformed.point.y:.3f}, '
                f'z={transformed.point.z:.3f}'
            )

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(
                f'Transform failed: {ex}'
            )

    def green_callback(self, msg):
        self.transform_point(msg, self.pub_green, 'Green')

    def red_callback(self, msg):
        self.transform_point(msg, self.pub_red, 'Red')


def main(args=None):

    rclpy.init(args=args)

    node = BallTransformer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


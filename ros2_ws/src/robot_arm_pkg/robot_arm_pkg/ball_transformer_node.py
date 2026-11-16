from collections import deque

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty


# ==========================================
# FILTR SOURADNIC (kamera obcas hodi nesmysl, napr. 6 m v x)
# ==========================================

# povolene limity v base_frame [m] - bod mimo se zahodi
X_MIN, X_MAX = 0.0, 0.600
Y_MIN, Y_MAX = -0.300, 0.300
Z_MIN, Z_MAX = 0.0, 0.200

# kolik po sobe jdoucich PLATNYCH (v limitech) vzorku musi sedet, nez se publikuje
STABLE_WINDOW = 15

# jak moc smi byt tech STABLE_WINDOW vzorku rozhazenych [m] (rozptyl na kazde ose)
# dokud se kamera "neuklidni" pod tuhle hranici, nic se neposila
STABLE_TOL = 0.0012  # 1.2 mm


class BallTransformer(Node):

    def __init__(self):
        super().__init__('ball_transformer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # kruhove buffery poslednich platnych vzorku (v base_frame) na barvu
        self.green_buffer = deque(maxlen=STABLE_WINDOW)
        self.red_buffer = deque(maxlen=STABLE_WINDOW)

        # filtr NENI prubezny - bezi az po capture requestu (z ball_node).
        # Jinak by po presunu micku hrozilo publikovani stare pozice drive,
        # nez se buffer stihne naplnit novymi vzorky.
        self.armed = {'Green': False, 'Red': False}

        # Capture request - vymaze buffery a spusti sber cerstvych vzorku
        self.sub_capture = self.create_subscription(
            Empty,
            '/ball_capture',
            self.capture_callback,
            10
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

    # --------------------------------------------------
    # FILTR
    # --------------------------------------------------

    def __in_limits(self, p):
        return (X_MIN <= p.x <= X_MAX
                and Y_MIN <= p.y <= Y_MAX
                and Z_MIN <= p.z <= Z_MAX)

    def __is_stable(self, buffer):
        # potreba plne okno a rozptyl na kazde ose pod toleranci
        if len(buffer) < STABLE_WINDOW:
            return False

        xs = [p.x for p in buffer]
        ys = [p.y for p in buffer]
        zs = [p.z for p in buffer]

        return (max(xs) - min(xs) <= STABLE_TOL
                and max(ys) - min(ys) <= STABLE_TOL
                and max(zs) - min(zs) <= STABLE_TOL)

    def __mean_point(self, buffer, header):
        n = len(buffer)
        out = PointStamped()
        out.header = header
        out.point.x = sum(p.x for p in buffer) / n
        out.point.y = sum(p.y for p in buffer) / n
        out.point.z = sum(p.z for p in buffer) / n
        return out

    # --------------------------------------------------
    # CAPTURE REQUEST
    # --------------------------------------------------

    def capture_callback(self, msg):
        # novy pozadavek -> zahodit stare vzorky a zacit sbirat cerstve
        self.green_buffer.clear()
        self.red_buffer.clear()
        self.armed['Green'] = True
        self.armed['Red'] = True
        self.get_logger().info('Capture request - sbiram cerstve vzorky')

    # --------------------------------------------------
    # ZPRACOVANI
    # --------------------------------------------------

    def process_point(self, msg, buffer, publisher, color):
        # bez aktivniho capture requestu kameru ignorujeme (zadne stare hodnoty)
        if not self.armed[color]:
            return

        # 1) prevod do base_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_frame',
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            transformed = do_transform_point(msg, transform)

        except tf2_ros.TransformException as ex:
            self.get_logger().warning(
                f'Transform failed: {ex}',
                throttle_duration_sec=2.0
            )
            return

        p = transformed.point

        # 2) zahodit nesmyslne hodnoty mimo limity (kamera obcas blbne)
        if not self.__in_limits(p):
            self.get_logger().warning(
                f'{color} mimo limity, zahozeno: '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}',
                throttle_duration_sec=2.0
            )
            return

        # 3) platny vzorek do bufferu
        buffer.append(p)

        # 4) dokud nemame plne okno stabilnich vzorku, jen cekame
        if not self.__is_stable(buffer):
            self.get_logger().info(
                f'{color} ceka na ustaleni: '
                f'{len(buffer)}/{STABLE_WINDOW} platnych vzorku',
                throttle_duration_sec=1.0
            )
            return

        # 5) stabilni -> publikuj prumer okna JEDNOU a uspi se (disarm),
        # aby se pozdeji neposlala stara hodnota pred dalsim capture requestem
        stable = self.__mean_point(buffer, transformed.header)
        publisher.publish(stable)
        self.armed[color] = False

        self.get_logger().info(
            f'{color} STABILNI v base_frame: '
            f'x={stable.point.x:.3f}, '
            f'y={stable.point.y:.3f}, '
            f'z={stable.point.z:.3f}'
        )

    def green_callback(self, msg):
        self.process_point(msg, self.green_buffer, self.pub_green, 'Green')

    def red_callback(self, msg):
        self.process_point(msg, self.red_buffer, self.pub_red, 'Red')


def main(args=None):

    rclpy.init(args=args)

    node = BallTransformer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

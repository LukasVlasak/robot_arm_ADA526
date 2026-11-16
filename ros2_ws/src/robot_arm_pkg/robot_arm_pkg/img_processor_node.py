import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as image_msg
from sensor_msgs.msg import CameraInfo as camerainfo_msg
from geometry_msgs.msg import PointStamped as point_msg
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
import machinevisiontoolbox as mvt
import numpy as np
import pyrealsense2 as rs


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor_node')

        self.sub_intrinsics = self.create_subscription(
            camerainfo_msg,
            '/camera/aligned_depth_to_color/camera_info',
            self.sub_intrinsics_callback,
            10)

        self.sub_color = self.create_subscription(
            image_msg,
            '/camera/color/image_raw',
            self.sub_color_callback,
            10)

        self.sub_depth = self.create_subscription(
            image_msg,
            '/camera/aligned_depth_to_color/image_raw',
            self.sub_depth_callback,
            10)

        self.pub_green = self.create_publisher(
            point_msg,
            '/goal_green_point',
            10)

        self.pub_red = self.create_publisher(
            point_msg,
            '/goal_red_point',
            10)

        self.br = CvBridge()
        self.fig = plt.figure(figsize=(12, 8))
        self.fig.add_subplot(2, 2, 1)   # axes[0] = color
        self.fig.add_subplot(2, 2, 2)   # axes[1] = depth
        self.fig.add_subplot(2, 2, 3)   # axes[2] = green mask
        self.fig.add_subplot(2, 2, 4)   # axes[3] = red mask

        # polomer micku [mm] - depth je na predni stenu, korekce na stred
        self.BALL_R = 20.0

        self.centroids_green = None
        self.centroids_red = None
        self.depth_green = 5.0
        self.depth_red = 5.0
        self.intrinsics = rs.intrinsics()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def find_centroids(self, mask_img, min_area=1000, max_area=100000):
        try:
            blobs = mask_img.blobs()
            
        except Exception as e:
            print("blobs() failed:", repr(e))
            return None

        best, best_score = None, 0.0
        for bl in blobs:
            if not (min_area <= bl.area <= max_area):
                continue


            umin, umax, vmin, vmax = bl.bbox          
            w, h = umax - umin, vmax - vmin
            
            cx, cy = bl.centroid

            inside = (umin <= cx <= umax) and (vmin <= cy <= vmax)
            if w <= 0 or h <= 0:
                continue

            aspect = min(w, h) / max(w, h)
            fill = bl.area / (w * h)
            if aspect < 0.75 or not (0.62 < fill < 0.88) or bl.circularity < 0.5:
                continue

            score = bl.area * aspect
            if score > best_score:
                best, best_score = bl, score

        if best is None:
            return None
        return [best.centroid[0], best.centroid[1]]

    def publish_point(self, point, publisher):
        msg = point_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x = point[0] / 1000
        msg.point.y = point[1] / 1000
        msg.point.z = point[2] / 1000
        publisher.publish(msg)

    def process_ball(self, name, centroid, depth, publisher):
        if centroid is None:
            print(f"No {name} blob found")
        elif self.intrinsics.fx == 0:
            print("Waiting for camera_info")
        else:
            try:
                point = rs.rs2_deproject_pixel_to_point(
                    self.intrinsics, centroid, depth)
                # depth je k prednimu POVRCHU micku -> posun bod po paprsku
                # o polomer na STRED micku (spravi x i y zaroven).
                # metricke (mm), nezavisle na rozliseni kamery.
                if depth > 0:
                    scale = (depth + self.BALL_R) / depth
                    point = [c * scale for c in point]
                print(f"{name}:", point)
                self.publish_point(point, publisher)
            except Exception as e:
                print(f"Deprojection {name} failed:", repr(e))

    def read_depth(self, img, centroid, old_depth):
        if centroid is None:
            return old_depth
        h, w = img.image.shape[:2]
        # order of coordinates is reversed in numpy array img.image
        x, y = int(centroid[0]), int(centroid[1])
        if 0 <= x < w and 0 <= y < h and img.image[y, x] != 0:
            return float(img.image[y, x])
        return old_depth

    # ------------------------------------------------------------------
    # Debug plots
    # ------------------------------------------------------------------
    def plot_centroids(self, ax):
        if self.centroids_green is not None:
            ax.plot(self.centroids_green[0], self.centroids_green[1], 'go')
        if self.centroids_red is not None:
            ax.plot(self.centroids_red[0], self.centroids_red[1], 'bo')

    def plot_color(self, img):
        ax = self.fig.axes[0]
        ax.cla()  # clear axis from last iteration
        ax.title.set_text("Color image")
        self.plot_centroids(ax)
        img.disp(fig=self.fig, ax=ax, fps=30)

    def plot_depth(self, img):
        ax = self.fig.axes[1]
        ax.cla()
        ax.title.set_text("Depth image")
        self.plot_centroids(ax)
        img.disp(fig=self.fig, ax=ax, fps=30)

    def plot_masks(self, mask_green, mask_red):
        ax = self.fig.axes[2]
        ax.cla()
        ax.title.set_text("Mask green (white = green)")
        self.plot_centroids(ax)
        mask_green.disp(fig=self.fig, ax=ax, fps=30)

        ax = self.fig.axes[3]
        ax.cla()
        ax.title.set_text("Mask red (white = red)")
        self.plot_centroids(ax)
        mask_red.disp(fig=self.fig, ax=ax, fps=30)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def sub_color_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        img = mvt.Image(current_frame)

        lab = img.colorspace('lab')
        L = lab.plane(0).image.astype(np.float32)
        a_img = lab.plane(1).image.astype(np.float32)

        thr_a_green = 108 
        thr_a_red = 150    
        L_ref = 128
        L_min = 20

        k = np.clip(L / L_ref, 0.3, 1.0)
        thr_g = 128 - (128 - thr_a_green) * k
        thr_r = 128 + (thr_a_red - 128) * k    

        green = (a_img < thr_g) & (L > L_min)
        red   = (a_img > thr_r) & (L > L_min)

        mask_green = mvt.Image(green.astype(np.uint8) * 255)
        mask_green = mask_green.open(np.ones((5, 5))).close(np.ones((9, 9)))
        mask_red = mvt.Image(red.astype(np.uint8) * 255)
        mask_red = mask_red.open(np.ones((5, 5))).close(np.ones((9, 9)))

        self.centroids_green = self.find_centroids(mask_green)
        self.centroids_red = self.find_centroids(mask_red)

        self.process_ball("green", self.centroids_green,
                          self.depth_green, self.pub_green)
        self.process_ball("red", self.centroids_red,
                          self.depth_red, self.pub_red)

        ## plotting (only recommended for debugging):
        ## self.plot_color(img)
        ## self.plot_masks(mask_green, mask_red)

    def sub_intrinsics_callback(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        self.intrinsics.model = rs.distortion.none
        self.intrinsics.coeffs = [i for i in msg.d]

    def sub_depth_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        img = mvt.Image(current_frame)

        self.depth_green = self.read_depth(img, self.centroids_green, self.depth_green)
        self.depth_red = self.read_depth(img, self.centroids_red, self.depth_red)

        ## plotting (only recommended for debugging):
       ## self.plot_depth(img)


def main(args=None):
    rclpy.init(args=args)

    img_processor = ImageProcessor()

    rclpy.spin(img_processor)

    img_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

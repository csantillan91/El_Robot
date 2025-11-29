# === ROS Boilerplate ===
import rclpy
from rclpy.node import Node

# === Your Code (Custom Logic) ===
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2  # we now actually use OpenCV to visualize  #-------

class DepthPointDemo(Node):
    def __init__(self):
        super().__init__('depth_point_demo')

        # --- CvBridge for converting ROS Image to numpy / OpenCV --- 
        self.bridge = CvBridge()

        # --- Intrinsics storage --- 
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.intrinsics_ready = False

        # --- Latest RGB frame storage ---  #-------
        self.latest_rgb = None  #------- 
        self.rgb_ready = False  #-------

        # --- Topics (adjust if your names are different) ---
        depth_topic = '/camera/stereo/image_raw'  # <--- use your depth-like topic  #-------
        camera_info_topic = '/camera/rgb/camera_info'
        rgb_topic = '/camera/rgb/image_raw'  #------- NEW

        # --- Subscriptions ---
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # --- RGB subscription for visualization ---  #-------
        self.rgb_sub = self.create_subscription(  #-------
            Image,  #-------
            rgb_topic,  #-------
            self.rgb_callback,  #-------
            10  #-------
        )  #-------

        self.get_logger().info(f'Subscribed to depth topic: {depth_topic}')
        self.get_logger().info(f'Subscribed to camera info topic: {camera_info_topic}')
        self.get_logger().info(f'Subscribed to RGB topic: {rgb_topic}')  #-------

        # counter to throttle logs
        self.print_counter = 0

    # === Callbacks ===

    def camera_info_callback(self, msg: CameraInfo):
        # K is a 3x3 row-major matrix stored as a flat array of length 9
        K = msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = K[0]
        self.fy = K[4]
        self.cx = K[2]
        self.cy = K[5]

        if not self.intrinsics_ready:
            self.intrinsics_ready = True
            self.get_logger().info(
                f'Camera intrinsics set: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def rgb_callback(self, msg: Image):  #------- NEW
        """Store the latest RGB frame as an OpenCV BGR image."""  #-------
        try:  #-------
            rgb_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #-------
        except CvBridgeError as e:  #-------
            self.get_logger().error(f'CvBridge error in rgb_callback: {e}')  #-------
            return  #-------

        if rgb_frame is None:  #-------
            self.get_logger().warn('Converted RGB image is None')  #-------
            return  #-------

        self.latest_rgb = rgb_frame  #-------
        self.rgb_ready = True  #-------

    def depth_callback(self, msg: Image):
        if not self.intrinsics_ready:
            self.get_logger().warn('Camera intrinsics not ready yet, skipping depth frame...')
            return

        try:
            # Use "passthrough" to keep the original depth encoding
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error in depth_callback: {e}')
            return

        if depth_img is None:
            self.get_logger().warn('Converted depth image is None')
            return

        # Ensure depth image is 2D
        if len(depth_img.shape) != 2:
            self.get_logger().warn(f'Unexpected depth image shape: {depth_img.shape}')
            return

        height, width = depth_img.shape
        u = width // 2   # center pixel x
        v = height // 2  # center pixel y

        raw_depth = depth_img[v, u]

        # Convert raw_depth to meters based on encoding
        encoding = msg.encoding.lower()
        if encoding == '16uc1':
            Z = float(raw_depth) / 1000.0  # mm → m
        elif encoding == '32fc1':
            Z = float(raw_depth)  # already meters
        else:
            self.get_logger().error(f'Unsupported depth encoding: {msg.encoding}')
            return

        if Z <= 0.0 or not np.isfinite(Z):
            # No valid depth at center pixel
            return

        # Back-project to 3D using pinhole camera model
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        # --- Visualization on RGB frame (if available) ---  #-------
        if self.rgb_ready and self.latest_rgb is not None:  #-------
            # Make a copy so we don't overwrite the stored frame  #-------
            vis_frame = self.latest_rgb.copy()  #-------

            # Draw a small circle at the center pixel  #-------
            cv2.circle(vis_frame, (u, v), 5, (0, 0, 255), -1)  # red dot  #-------

            # Put text with the depth and maybe X,Y,Z  #-------
            text = f"Z={Z:.2f}m X={X:.2f}m Y={Y:.2f}m"  #-------
            cv2.putText(  #-------
                vis_frame, text, (10, 30),  #-------
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2  #-------
            )  #-------

            cv2.imshow('OAK-D RGB + center 3D', vis_frame)  #-------
            cv2.waitKey(1)  #-------

        # Throttle logging to avoid spamming the console
        self.print_counter += 1
        if self.print_counter % 10 == 0:
            self.get_logger().info(
                f'Center pixel (u={u}, v={v}), depth={Z:.3f} m → 3D point: '
                f'X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DepthPointDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  #------- make sure window closes
        rclpy.shutdown()

if __name__ == '__main__':
    main()

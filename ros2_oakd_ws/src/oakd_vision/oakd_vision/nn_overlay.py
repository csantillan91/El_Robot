# === ROS Boilerplate ===
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
import cv2

# === Your Code ===
class NNDrawingNode(Node):
    def __init__(self):
        super().__init__('oakd_nn_overlay')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None

        # Subscribe to RGB
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to neural network detections
        self.nn_sub = self.create_subscription(             # -------
            Detection3DArray,                          # -------
            '/camera/nn/spatial_detections',                # -------
            self.nn_callback,                               # -------
            10
        )

        self.get_logger().info("Overlay node started. Listening for RGB + NN detections.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))

        self.try_draw()

    def nn_callback(self, msg):                             # -------
        self.latest_detections = msg                        # -------
        self.try_draw()                                     # -------

    def try_draw(self):
        if self.latest_image is None or self.latest_detections is None:
            return

        frame = self.latest_image.copy()

        for det in self.latest_detections.detections:       # -------
            # Extract bbox (center + size)
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w  = det.bbox.size.x
            h  = det.bbox.size.y

            x1 = int(cx - w/2)
            y1 = int(cy - h/2)
            x2 = int(cx + w/2)
            y2 = int(cy + h/2)

            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

            # Extract spatial coords
            X = det.results[0].pose.pose.position.x
            Y = det.results[0].pose.pose.position.y
            Z = det.results[0].pose.pose.position.z

            label = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}"        # -------

            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255,255,255), 2)

        cv2.imshow("NN Overlay", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = NNDrawingNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

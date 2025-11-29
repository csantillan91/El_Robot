# === ROS Boilerplate ===
import rclpy  #-------
from rclpy.node import Node  #-------

# === Your Code (Custom Logic) ===
from sensor_msgs.msg import Image  #-------
from cv_bridge import CvBridge, CvBridgeError  #-------
import cv2  #-------


class OakdImageViewer(Node):  #-------
    def __init__(self):  #-------
        super().__init__('oakd_image_viewer')  #-------

        # bridge: ROS Image ↔ OpenCV image
        self.bridge = CvBridge()  #-------

        # TODO: replace with your actual RGB topic if different
        rgb_topic = '/camera/rgb/image_raw'  #------- change if needed

        # subscriber to RGB images
        self.subscription = self.create_subscription(  #-------
            Image,               # message type  #-------
            rgb_topic,           # topic name    #-------
            self.image_callback, # callback      #-------
            10                   # QoS           #-------
        )
        self.subscription  # prevent unused variable warning  #-------

        self.get_logger().info(f'Subscribed to RGB topic: {rgb_topic}')  #-------


    def image_callback(self, msg: Image):  #-------
        try:  #-------
            # Convert ROS Image → OpenCV BGR image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #-------
        except CvBridgeError as e:  #-------
            self.get_logger().error(f'CvBridge error: {e}')  #-------
            return  #-------

        # Show the frame using OpenCV
        cv2.imshow('OAK-D RGB', frame)  #-------
        # waitKey(1) is required for OpenCV window to update
        cv2.waitKey(1)  #-------


def main(args=None):  #-------
    rclpy.init(args=args)  #-------

    node = OakdImageViewer()  #-------
    try:  #-------
        rclpy.spin(node)  #-------
    except KeyboardInterrupt:  #-------
        pass  #-------
    finally:  #-------
        node.get_logger().info('Shutting down OAK-D image viewer')  #-------
        node.destroy_node()  #-------
        cv2.destroyAllWindows()  #-------
        rclpy.shutdown()  #-------


if __name__ == '__main__':  #-------
    main()  #-------

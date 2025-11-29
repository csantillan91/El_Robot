# === ROS Boilerplate ===
import rclpy  #-------
from rclpy.node import Node  #-------


class OakdHelloNode(Node):  #-------
    def __init__(self):  #-------
        super().__init__('oakd_hello_node')  #-------
        # === Your Code (Custom Logic) ===
        self.timer = self.create_timer(1.0, self.timer_callback)  #-------


    def timer_callback(self):  #-------
        self.get_logger().info('Hello from OAK-D workspace 👋')  #-------


def main(args=None):  #-------
    rclpy.init(args=args)  #-------

    node = OakdHelloNode()  #-------
    rclpy.spin(node)  #-------

    node.destroy_node()  #-------
    rclpy.shutdown()  #-------


if __name__ == '__main__':  #-------
    main()  #-------

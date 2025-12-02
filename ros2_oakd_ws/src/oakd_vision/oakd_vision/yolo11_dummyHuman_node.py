# === ROS Boilerplate ===
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#------- new
import cv2  #------- new
from ultralytics import YOLO  #------- new

# === Your Code (YOLO11 Dummy Detector) ===
class Yolo11dummyHumanNode(Node):
    def __init__(self):
        super().__init__('yolo11_dummyHuman_node')

        # Bridge to convert ROS Image → OpenCV
        self.bridge = CvBridge()

        # YOLO11 model (use nano model for speed)
        self.get_logger().info('Loading YOLO11 model...')  #------- new
        self.model = YOLO('yolo11n.pt')  #------- new

        # RGB topic (adjust if your driver uses /camera/color/image_raw)
        rgb_topic = '/camera/rgb/image_raw'  #------- new

        self.subscription = self.create_subscription(
            Image,
            rgb_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to RGB topic: {rgb_topic}')

    def image_callback(self, msg: Image):
        # 1) ROS Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 2) Run YOLO11 inference on the frame
        #    imgsz controls internal resize; 640 is a good default.
        results = self.model(frame, imgsz=640)[0]  #------- new

        # 3) Draw detections and pick humanoid-ish candidates
        #    YOLOv8/11 results API:
        #    results.boxes.xyxy, results.boxes.cls, results.boxes.conf
        boxes = results.boxes  #------- new
        best_center = None     #------- new
        best_conf = 0.0        #------- new

        for box in boxes:  #------- new
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())

            # COCO class 0 is "person" for default models.
            # For now, we use this as our "humanoid-ish" filter.
            is_person = (cls_id == 0)  #------- new

            if not is_person:
                # You can relax this later if your toy isn't detected as "person"
                continue

            # Compute bbox center (u, v)
            u = int((x1 + x2) / 2.0)  #------- new
            v = int((y1 + y2) / 2.0)  #------- new

            # Track best (highest-confidence) detection
            if conf > best_conf:
                best_conf = conf
                best_center = (u, v)

            # Draw bbox and label on frame
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )
            label = f'person {conf:.2f}'  #------- new
            cv2.putText(
                frame,
                label,
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            # Draw a small circle at the center
            cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)  #------- new

        # 4) Log the "best" detection center (if any)
        if best_center is not None:  #------- new
            u, v = best_center
            self.get_logger().info(
                f'Best humanoid detection at (u={u}, v={v}), conf={best_conf:.2f}'
            )

        # 5) Show the frame
        cv2.imshow('YOLO11 dummyHuman', frame)  #------- new
        cv2.waitKey(1)  #------- new


def main(args=None):
    rclpy.init(args=args)
    node = Yolo11dummyHumanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()  #------- new
        rclpy.shutdown()


if __name__ == '__main__':
    main()

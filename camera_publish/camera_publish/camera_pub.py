import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublish(Node):
    def __init__(self):
        super().__init__('camera_publish')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.get_logger().info('Camera publisher node started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            
        else:
            self.get_logger().warn("카메라 프레임을 읽을 수 없습니다.")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.serial_port = None

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.get_logger().info("Serial port connected to /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_port = None        
        
        # 구독자 생성: /cmd_vel 토픽을 Twist 메시지 타입으로 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10  # QoS profile
        )
        self.subscription  # prevent unused variable warning

        

    def listener_callback(self, msg):

        
        
        if self.serial_port and self.serial_port.is_open:
            try:
                data = f"{msg.linear.x} {msg.linear.y} {msg.linear.z}\n"
                self.serial_port.write(data.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Serial send error: {e}")

        


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()

    rclpy.spin(cmd_vel_subscriber)

    # 노드 종료 처리
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

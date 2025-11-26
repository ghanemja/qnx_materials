import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/QNX_1/data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('IMU Subscriber node started.')

    def listener_callback(self, msg: Imu):
        roll  = msg.orientation.x
        pitch = msg.orientation.y
        yaw   = msg.orientation.z
        self.get_logger().info(f'Received IMU data: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


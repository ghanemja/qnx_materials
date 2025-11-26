import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import math
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/QNX_1/data', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_fake_imu)
        self.get_logger().info('IMU Publisher node started.')

    def publish_fake_imu(self):
        msg = Imu()
        # Simulate roll, pitch, yaw using sin waves or random noise
        t = time.time()
        roll  = math.sin(t) * 0.1
        pitch = math.cos(t) * 0.1
        yaw   = math.sin(t / 2) * 0.1

        # Fill IMU message (radians)
        msg.orientation.x = roll
        msg.orientation.y = pitch
        msg.orientation.z = yaw
        msg.orientation.w = 1.0

        msg.angular_velocity.x = random.uniform(-0.05, 0.05)
        msg.angular_velocity.y = random.uniform(-0.05, 0.05)
        msg.angular_velocity.z = random.uniform(-0.05, 0.05)

        msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.z = random.uniform(-9.8, -9.6)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing IMU data: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


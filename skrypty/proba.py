import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.subscription = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        accel = msg.linear_acceleration
        self.get_logger().info(f'Przyspieszenie X: {accel.x:.2f}, Y: {accel.y:.2f}, Z: {accel.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    imu_reader = ImuReader()
    print("Odczytuję dane z IMU... (Ctrl+C aby przerwać)")
    rclpy.spin(imu_reader)
    imu_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

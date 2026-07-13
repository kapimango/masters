import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
import math

class HeadingAutopilot(Node):
    def __init__(self):
        super().__init__('heading_autopilot')

        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Cele
        self.abs_start_x = -532.0
        self.abs_target_x = -482.0
        
        # Nawigacja
        self.initial_lon = None
        self.current_x = -532.0
        self.current_yaw = 0.0
        self.state = "CALIBRATING"

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def gps_cb(self, msg):
        if self.initial_lon is None:
            self.initial_lon = msg.longitude
            self.state = "GO_TO_TARGET"
            return
        self.current_x = self.abs_start_x + (msg.longitude - self.initial_lon) * 90000.0

    def control_loop(self):
        if self.state == "CALIBRATING":
            return

        target_yaw = 0.0 if self.state == "GO_TO_TARGET" else math.pi
        
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        thrust_l, thrust_r = 0.0, 0.0

        # 3. Logika Maszyny Stanów
        if self.state == "GO_TO_TARGET":
            if self.current_x < self.abs_target_x:
                if abs(yaw_error) > 0.5:
                    thrust_l, thrust_r = self.get_turn_thrust(yaw_error)
                else:
                    thrust_l, thrust_r = self.get_move_thrust(yaw_error, 60.0)
            else:
                self.state = "RETURN"
                self.get_logger().info("CEL OSIĄGNIĘTY. ZAWRACAM.")

        elif self.state == "RETURN":
            if self.current_x > self.abs_start_x:
                if abs(yaw_error) > 0.5:
                    thrust_l, thrust_r = self.get_turn_thrust(yaw_error)
                else:
                    thrust_l, thrust_r = self.get_move_thrust(yaw_error, 60.0)
            else:
                self.state = "STOP"

        elif self.state == "STOP":
            thrust_l, thrust_r = 0.0, 0.0

        self.send_thrust(thrust_l, thrust_r)
        print(f"[{self.state}] X: {self.current_x:.1f} | Yaw Err: {math.degrees(yaw_error):.1f}°")

    def get_turn_thrust(self, error):
        p_gain = 40.0
        val = p_gain * error
        return -val, val

    def get_move_thrust(self, error, base_speed):
        steering_gain = 30.0
        correction = steering_gain * error
        return base_speed - correction, base_speed + correction

    def send_thrust(self, left, right):
        left = max(min(left, 100.0), -100.0)
        right = max(min(right, 100.0), -100.0)
        self.pub_left.publish(Float64(data=float(left)))
        self.pub_right.publish(Float64(data=float(right)))

def main():
    rclpy.init()
    node = HeadingAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_thrust(0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

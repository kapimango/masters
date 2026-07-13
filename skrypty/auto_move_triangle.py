import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
import math
import matplotlib.pyplot as plt

class SequentialPlottingAutopilot(Node):
    def __init__(self):
        super().__init__('sequential_plotting_autopilot')

        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.waypoints = [(-482.0, 162.0), (-482.0, 212.0), (-532.0, 162.0)]
        self.current_wp_idx = 0
        
        self.initial_lon, self.initial_lat = None, None
        self.current_x, self.current_y = -532.0, 162.0
        self.current_yaw = 0.0
        self.history_x, self.history_y = [], []
        
        self.dist_margin = 3.5
        self.angle_tolerance = 0.1
        self.is_rotating = True
        self.state = "CALIBRATING"

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title("Nawigacja Sekwencyjna: Obrót -> Płynięcie")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)
        wx, wy = zip(*self.waypoints)
        self.ax.scatter(wx, wy, c='red', marker='x', label='Waypointy')
        self.path_line, = self.ax.plot([], [], 'b-', label='Ścieżka')
        self.current_pos_dot, = self.ax.plot([], [], 'go', label='Katamaran')
        self.ax.legend()


    def imu_cb(self, msg):
        q = msg.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def gps_cb(self, msg):
        if self.initial_lon is None:
            self.initial_lat, self.initial_lon = msg.latitude, msg.longitude
            self.state = "NAVIGATING"
            return
        self.current_x = -532.0 + (msg.longitude - self.initial_lon) * 89000.0
        self.current_y = 162.0 + (msg.latitude - self.initial_lat) * 111000.0
        self.history_x.append(self.current_x)
        self.history_y.append(self.current_y)

    def control_loop(self):
        if self.state != "NAVIGATING": return

        if self.history_x:
            self.path_line.set_data(self.history_x, self.history_y)
            self.current_pos_dot.set_data([self.current_x], [self.current_y])
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)

        if self.current_wp_idx >= len(self.waypoints):
            self.send_thrust(0.0, 0.0)
            self.get_logger().info("MISJA ZAKOŃCZONA", once=True)
            return


        tx, ty = self.waypoints[self.current_wp_idx]
        dx, dy = tx - self.current_x, ty - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        

        yaw_error = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))

        thrust_l, thrust_r = 0.0, 0.0


        if self.is_rotating:
            if abs(yaw_error) > self.angle_tolerance:
                p_gain = 35.0
                thrust_l = -p_gain * yaw_error
                thrust_r = p_gain * yaw_error
                msg_status = "OBRACANIE"
            else:
                self.is_rotating = False
                self.get_logger().info(f"Wycelowano w WP {self.current_wp_idx+1}. Ruszam!")
                return
        else:
            if dist > self.dist_margin:
                base_speed = 50.0
                steering_gain = 25.0
                correction = steering_gain * yaw_error
                thrust_l = base_speed - correction
                thrust_r = base_speed + correction
                msg_status = "PŁYNIĘCIE"
            else:
                self.current_wp_idx += 1
                self.is_rotating = True
                self.get_logger().info(f"Osiągnięto WP. Szukam kolejnego...")
                return

        self.send_thrust(thrust_l, thrust_r)
        print(f"[{msg_status}] WP: {self.current_wp_idx+1} | Dist: {dist:.1f}m | Err: {math.degrees(yaw_error):.1f}deg")

    def send_thrust(self, left, right):
        left = max(min(left, 100.0), -100.0)
        right = max(min(right, 100.0), -100.0)
        self.pub_left.publish(Float64(data=float(left)))
        self.pub_right.publish(Float64(data=float(right)))

def main():
    rclpy.init()
    node = SequentialPlottingAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.send_thrust(0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

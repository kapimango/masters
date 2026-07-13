import math
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu



def clamp(value, v_min, v_max):
    return max(v_min, min(v_max, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def deg2rad(deg):
    return math.radians(deg)


class MovingRMS:
    def __init__(self, max_len=45):
        self.values = deque(maxlen=max_len)

    def add(self, value):
        self.values.append(float(value))

    def get(self):
        if not self.values:
            return 0.0
        return math.sqrt(sum(v * v for v in self.values) / len(self.values))


class MissionIndicators:
    def __init__(self):
        self.reset()

    def reset(self):
        self.t = 0.0
        self.I_roll = 0.0
        self.I_pitch = 0.0
        self.I_roll_rate = 0.0
        self.I_pitch_rate = 0.0
        self.I_goal_heading = 0.0
        self.I_no_progress = 0.0
        self.I_control = 0.0
        self.J_total = 0.0
        self.J_roll = 0.0
        self.J_pitch = 0.0
        self.J_roll_rate = 0.0
        self.J_pitch_rate = 0.0
        self.J_goal_heading = 0.0
        self.J_no_progress = 0.0
        self.J_control = 0.0

    def update(self, dt, *, roll, pitch, roll_rate, pitch_rate,
               goal_heading_error, no_progress, left, right, max_thrust, weights):
        if dt <= 0.0:
            return

        control_norm = (left * left + right * right) / max(max_thrust * max_thrust, 1e-9)

        self.t += dt
        self.I_roll += roll * roll * dt
        self.I_pitch += pitch * pitch * dt
        self.I_roll_rate += roll_rate * roll_rate * dt
        self.I_pitch_rate += pitch_rate * pitch_rate * dt
        self.I_goal_heading += goal_heading_error * goal_heading_error * dt
        self.I_no_progress += no_progress * dt
        self.I_control += control_norm * dt

        self.J_roll = weights['roll'] * self.I_roll
        self.J_pitch = weights['pitch'] * self.I_pitch
        self.J_roll_rate = weights['roll_rate'] * self.I_roll_rate
        self.J_pitch_rate = weights['pitch_rate'] * self.I_pitch_rate
        self.J_goal_heading = weights['goal_heading'] * self.I_goal_heading
        self.J_no_progress = weights['no_progress'] * self.I_no_progress
        self.J_control = weights['control'] * self.I_control
        self.J_total = (
            self.J_roll
            + self.J_pitch
            + self.J_roll_rate
            + self.J_pitch_rate
            + self.J_goal_heading
            + self.J_no_progress
            + self.J_control
        )

    def rms_roll(self):
        return math.sqrt(self.I_roll / self.t) if self.t > 1e-9 else 0.0

    def rms_pitch(self):
        return math.sqrt(self.I_pitch / self.t) if self.t > 1e-9 else 0.0


class PID:
    def __init__(self, kp, ki, kd,
                 output_min=-100.0, output_max=100.0,
                 integral_limit=50.0,
                 derivative_filter_coeff=0.15,
                 name="PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.alpha = derivative_filter_coeff
        self.name = name

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_time = None

    def compute(self, error, now=None):
        if now is None:
            now = time.monotonic()

        if self.last_time is None:
            dt = 0.1
        else:
            dt = now - self.last_time
            if dt <= 0.0 or dt > 2.0:
                dt = 0.1
        self.last_time = now

        p = self.kp * error

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        i = self.ki * self.integral

        raw_derivative = (error - self.prev_error) / dt
        derivative = self.alpha * raw_derivative + (1.0 - self.alpha) * self.prev_derivative
        d = self.kd * derivative

        self.prev_error = error
        self.prev_derivative = derivative

        output = clamp(p + i + d, self.output_min, self.output_max)
        return output, p, i, d


class RollCostIntegralAutopilot(Node):
    WAYPOINTS = [(-482.0, 190.0), (-482.0, 212.0), (-532.0, 190.0)]
    START_X, START_Y = -532.0, 190.0

    WAYPOINT_RADIUS = 4.0
    WAYPOINT_HOLD_TIME = 0.35

    WAVE_HEADING_DEG = 0.0

    PREFERRED_REL_TO_WAVE_DEG = 0.0

    MAX_DETOUR_ANGLE_DEG = 70.0
    CANDIDATE_STEP_DEG = 5.0

    W_GOAL = 1.00
    W_WAVE = 5.00
    W_PROGRESS = 1.60
    W_SMOOTH = 0.25

    W_INT_ROLL = 12.0
    W_INT_PITCH = 2.0
    W_INT_ROLL_RATE = 2.5
    W_INT_PITCH_RATE = 0.5
    W_INT_GOAL_HEADING = 0.8
    W_INT_NO_PROGRESS = 2.0
    W_INT_CONTROL = 0.05

    ROLL_BAD_RAD = 0.08
    ROLL_SLOWDOWN_GAIN = 0.70

    PITCH_BAD_RAD = 0.14
    PITCH_SLOWDOWN_GAIN = 0.20

    MAX_THRUST = 100.0
    MAX_REVERSE_THRUST = -20.0
    MAX_SLEW = 5.0

    MAX_SPEED_REF = 1.45
    MIN_SPEED_REF = 0.20
    SLOWDOWN_RADIUS = 13.0
    SPEED_FILTER_ALPHA = 0.25

    YAW_SPEED_REDUCTION_DEG = 80.0

    CONTROL_PERIOD = 0.1
    STATUS_UPDATE_PERIOD = 0.35
    PLOT_UPDATE_PERIOD = 0.6

    def __init__(self):
        super().__init__('roll_cost_integral_autopilot')

        self.pub_l = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self._gps_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self._imu_cb, 10)
        self.timer = self.create_timer(self.CONTROL_PERIOD, self._control_loop)

        self.state = 'CALIBRATING'
        self.wp_idx = 0
        self.x, self.y = self.START_X, self.START_Y
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0

        self.init_lat = None
        self.init_lon = None
        self.m_per_deg_lon = None
        self.prev_gps_time = None
        self.speed_meas = 0.0

        self.roll_rms = MovingRMS(max_len=45)
        self.pitch_rms = MovingRMS(max_len=45)
        self.ind = MissionIndicators()
        self._last_indicator_time = None

        self._last_tl = 0.0
        self._last_tr = 0.0
        self._last_yaw_ref = 0.0
        self._wp_enter_time = None
        self._finished = False
        self._last_status_update = 0.0
        self._last_plot_update = 0.0

        self.pid_yaw = PID(
            kp=24.0, ki=0.08, kd=3.2,
            output_min=-45.0, output_max=45.0,
            integral_limit=10.0,
            derivative_filter_coeff=0.12,
            name='Yaw'
        )

        self.pid_speed = PID(
            kp=34.0, ki=2.0, kd=3.5,
            output_min=self.MAX_REVERSE_THRUST, output_max=72.0,
            integral_limit=7.0,
            derivative_filter_coeff=0.20,
            name='Speed'
        )

        self.pid_roll_bias = PID(
            kp=4.0, ki=0.0, kd=1.5,
            output_min=-6.0, output_max=6.0,
            integral_limit=0.0,
            derivative_filter_coeff=0.20,
            name='RollBias'
        )

        self._t0 = time.monotonic()
        self._log_file = open('/tmp/wamv_roll_cost_integral_log.csv', 'w')
        self._log_file.write(
            't,controller_id,state_code,wp_idx,x,y,dist,yaw_deg,yaw_to_wp_deg,yaw_ref_deg,yaw_err_deg,yaw_rate_deg_s,speed_ref,speed_meas,roll_deg,roll_rate_deg_s,roll_rms_window_deg,roll_rms_total_deg,pitch_deg,pitch_rate_deg_s,pitch_rms_window_deg,pitch_rms_total_deg,base_cmd,diff_cmd,T_left,T_right,J_inst,J_goal,J_wave_roll,J_progress,J_smooth,J_mpc_distance,J_mpc_final,J_mpc_heading,J_mpc_wrong_way,J_mpc_pitch,J_mpc_control,J_mpc_slew,J_mpc_turn,J_total,J_roll,J_pitch,J_roll_rate,J_pitch_rate,J_goal_heading,J_no_progress,J_control,I_roll,I_pitch,I_roll_rate,I_pitch_rate,I_control,I_no_progress,I_goal_heading\n'
        )
        self.history_x = []
        self.history_y = []
        self._setup_plot()

        self.get_logger().info('RollCostIntegralAutopilot uruchomiony ✓')
        self.get_logger().info('Brak stanu ROTATING: po GPS przechodzę bezpośrednio do RUNNING.')

    def _imu_cb(self, msg):
        q = msg.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        sin_p = 2.0 * (q.w * q.y - q.z * q.x)
        sin_p = clamp(sin_p, -1.0, 1.0)
        self.pitch = math.asin(sin_p)

        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y

        self.roll_rms.add(self.roll)
        self.pitch_rms.add(self.pitch)

    def _gps_cb(self, msg):
        now = time.monotonic()

        if self.init_lon is None:
            self.init_lat = msg.latitude
            self.init_lon = msg.longitude
            self.m_per_deg_lon = 111320.0 * math.cos(math.radians(self.init_lat))
            self.prev_gps_time = now
            self.state = 'RUNNING'
            self.get_logger().info(
                f'GPS zainicjalizowany: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}'
            )
            return

        old_x, old_y = self.x, self.y
        self.x = self.START_X + (msg.longitude - self.init_lon) * self.m_per_deg_lon
        self.y = self.START_Y + (msg.latitude - self.init_lat) * 110540.0

        dt = now - self.prev_gps_time if self.prev_gps_time is not None else None
        if dt is not None and 0.02 <= dt <= 2.0:
            raw_speed = math.hypot(self.x - old_x, self.y - old_y) / dt
            self.speed_meas = (
                self.SPEED_FILTER_ALPHA * raw_speed
                + (1.0 - self.SPEED_FILTER_ALPHA) * self.speed_meas
            )
        self.prev_gps_time = now

        self.history_x.append(self.x)
        self.history_y.append(self.y)

    def _control_loop(self):
        now = time.monotonic()

        if self._finished:
            return

        if self.state == 'CALIBRATING':
            self._send_raw(0.0, 0.0)
            return

        if self.wp_idx >= len(self.WAYPOINTS):
            self._finish()
            return

        tx, ty = self.WAYPOINTS[self.wp_idx]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        yaw_to_wp = math.atan2(dy, dx)

        if self._waypoint_reached(dist, now):
            self.wp_idx += 1
            self._wp_enter_time = None
            self.pid_yaw.reset()
            self.pid_speed.reset()
            self.pid_roll_bias.reset()

            if self.wp_idx >= len(self.WAYPOINTS):
                self._finish()
                return

            self.get_logger().info(f'Waypoint osiągnięty -> WP{self.wp_idx + 1}')
            return

        yaw_ref, cost = self._choose_heading_from_cost(yaw_to_wp, dist)
        self._last_yaw_ref = yaw_ref
        yaw_err = wrap_pi(yaw_ref - self.yaw)

        turn, yaw_p, yaw_i, yaw_d = self.pid_yaw.compute(yaw_err, now)

        speed_ref = self._speed_reference(dist, yaw_ref, yaw_to_wp, yaw_err)
        speed_error = speed_ref - self.speed_meas
        base, sp_p, sp_i, sp_d = self.pid_speed.compute(speed_error, now)

        roll_bias, *_ = self.pid_roll_bias.compute(-(self.roll + 0.25 * self.roll_rate), now)

        left = base - turn - roll_bias
        right = base + turn + roll_bias

        self._send(left, right)
        self._update_mission_indicators(now, yaw_ref, yaw_to_wp)
        self._log(now, dist, yaw_to_wp, yaw_ref, yaw_err, speed_ref, cost)
        self._print_status(now, dist, yaw_to_wp, yaw_ref, yaw_err, speed_ref, cost)
        self._update_plot(now)

    def _choose_heading_from_cost(self, yaw_to_wp, dist):
        wave_heading = deg2rad(self.WAVE_HEADING_DEG)
        preferred = deg2rad(self.PREFERRED_REL_TO_WAVE_DEG)
        preferred_a = wrap_pi(wave_heading + preferred)
        preferred_b = wrap_pi(preferred_a + math.pi)

        roll_window = self.roll_rms.get()
        roll_total = self.ind.rms_roll()
        roll_severity = clamp(max(roll_window, roll_total) / self.ROLL_BAD_RAD, 0.0, 1.0)

        far_from_wp = clamp(dist / (2.5 * self.WAYPOINT_RADIUS), 0.0, 1.0)
        goal_weight = self.W_GOAL * (1.0 + 1.6 * (1.0 - far_from_wp))
        wave_weight = self.W_WAVE * roll_severity * far_from_wp

        max_detour = deg2rad(self.MAX_DETOUR_ANGLE_DEG)
        step = deg2rad(self.CANDIDATE_STEP_DEG)
        n_steps = int(round((2.0 * max_detour) / step))

        best_heading = yaw_to_wp
        best = {
            'J': float('inf'),
            'goal': 0.0,
            'wave': 0.0,
            'progress': 0.0,
            'smooth': 0.0,
            'roll_severity': roll_severity,
        }

        for k in range(n_steps + 1):
            offset = -max_detour + k * step
            candidate = wrap_pi(yaw_to_wp + offset)

            e_goal = wrap_pi(candidate - yaw_to_wp)
            e_wave = min(
                abs(wrap_pi(candidate - preferred_a)),
                abs(wrap_pi(candidate - preferred_b))
            )
            e_smooth = wrap_pi(candidate - self._last_yaw_ref)

            progress = math.cos(e_goal)
            progress_penalty = max(0.0, 1.0 - progress)

            J_goal = goal_weight * e_goal * e_goal
            J_wave = wave_weight * e_wave * e_wave
            J_progress = self.W_PROGRESS * progress_penalty
            J_smooth = self.W_SMOOTH * e_smooth * e_smooth
            J = J_goal + J_wave + J_progress + J_smooth

            if J < best['J']:
                best_heading = candidate
                best = {
                    'J': J,
                    'goal': J_goal,
                    'wave': J_wave,
                    'progress': J_progress,
                    'smooth': J_smooth,
                    'roll_severity': roll_severity,
                }

        return best_heading, best

    def _speed_reference(self, dist, yaw_ref, yaw_to_wp, yaw_err):
        wp_factor = clamp(dist / self.SLOWDOWN_RADIUS, 0.0, 1.0)

        detour = abs(wrap_pi(yaw_ref - yaw_to_wp))
        detour_factor = clamp(math.cos(detour), 0.35, 1.0)

        yaw_factor = 1.0 - clamp(abs(yaw_err) / deg2rad(self.YAW_SPEED_REDUCTION_DEG), 0.0, 1.0)
        yaw_factor = clamp(yaw_factor, 0.25, 1.0)

        roll_severity = clamp(max(self.roll_rms.get(), self.ind.rms_roll()) / self.ROLL_BAD_RAD, 0.0, 1.0)
        roll_factor = clamp(1.0 - self.ROLL_SLOWDOWN_GAIN * roll_severity, 0.30, 1.0)

        pitch_severity = clamp(max(self.pitch_rms.get(), self.ind.rms_pitch()) / self.PITCH_BAD_RAD, 0.0, 1.0)
        pitch_factor = clamp(1.0 - self.PITCH_SLOWDOWN_GAIN * pitch_severity, 0.75, 1.0)

        speed = self.MAX_SPEED_REF * wp_factor * detour_factor * yaw_factor * roll_factor * pitch_factor
        if dist > 1.5 * self.WAYPOINT_RADIUS:
            speed = max(speed, self.MIN_SPEED_REF)
        return speed

    def _update_mission_indicators(self, now, yaw_ref, yaw_to_wp):
        if self._last_indicator_time is None:
            self._last_indicator_time = now
            return

        dt = now - self._last_indicator_time
        self._last_indicator_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        goal_heading_error = wrap_pi(yaw_ref - yaw_to_wp)
        progress = math.cos(goal_heading_error)
        no_progress = max(0.0, 1.0 - progress)

        weights = {
            'roll': self.W_INT_ROLL,
            'pitch': self.W_INT_PITCH,
            'roll_rate': self.W_INT_ROLL_RATE,
            'pitch_rate': self.W_INT_PITCH_RATE,
            'goal_heading': self.W_INT_GOAL_HEADING,
            'no_progress': self.W_INT_NO_PROGRESS,
            'control': self.W_INT_CONTROL,
        }

        self.ind.update(
            dt,
            roll=self.roll,
            pitch=self.pitch,
            roll_rate=self.roll_rate,
            pitch_rate=self.pitch_rate,
            goal_heading_error=goal_heading_error,
            no_progress=no_progress,
            left=self._last_tl,
            right=self._last_tr,
            max_thrust=self.MAX_THRUST,
            weights=weights,
        )

    def _waypoint_reached(self, dist, now):
        if dist > self.WAYPOINT_RADIUS:
            self._wp_enter_time = None
            return False

        if self._wp_enter_time is None:
            self._wp_enter_time = now
            return False

        return (now - self._wp_enter_time) >= self.WAYPOINT_HOLD_TIME

    def _slew(self, target_l, target_r):
        def limit_delta(prev, target):
            delta = clamp(target - prev, -self.MAX_SLEW, self.MAX_SLEW)
            return prev + delta

        new_l = limit_delta(self._last_tl, target_l)
        new_r = limit_delta(self._last_tr, target_r)

        new_l = clamp(new_l, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        new_r = clamp(new_r, self.MAX_REVERSE_THRUST, self.MAX_THRUST)

        self._last_tl = new_l
        self._last_tr = new_r
        return new_l, new_r

    def _send(self, left, right):
        left, right = self._slew(left, right)
        self._publish_thrust(left, right)

    def _send_raw(self, left, right):
        self._last_tl = float(left)
        self._last_tr = float(right)
        self._publish_thrust(left, right)

    def _publish_thrust(self, left, right):
        if not rclpy.ok():
            return
        self.pub_l.publish(Float64(data=float(left)))
        self.pub_r.publish(Float64(data=float(right)))

    def _finish(self):
        if self._finished:
            return

        self._finished = True
        self.state = 'FINISHED'
        self._send_raw(0.0, 0.0)
        self._save_final_plot()
        self._log_file.flush()

        print('\033[H\033[J', end='')
        print('══════════════════════════════════════════')
        print('  FINISHED')
        print('══════════════════════════════════════════')
        print('  Wszystkie waypointy osiągnięte.')
        print(f'  J_total:          {self.ind.J_total:.6f}')
        print(f'  J_roll:           {self.ind.J_roll:.6f}')
        print(f'  J_pitch:          {self.ind.J_pitch:.6f}')
        print(f'  Roll RMS misji:   {math.degrees(self.ind.rms_roll()):.3f} deg')
        print(f'  Pitch RMS misji:  {math.degrees(self.ind.rms_pitch()):.3f} deg')
        print('  Log:     /tmp/wamv_roll_cost_integral_log.csv')
        print('  Wykres:  /tmp/wamv_roll_cost_integral_path.png')
        print('══════════════════════════════════════════')
        self.get_logger().info('FINISHED')

        self.timer.cancel()
        rclpy.shutdown()

    def _log(self, now, dist, yaw_to_wp, yaw_ref, yaw_err, speed_ref, cost):
        t = now - self._t0
        state_code = {'CALIBRATING': 0, 'RUNNING': 1, 'FINISHED': 2}.get(self.state, -1)

        self._log_file.write(
            f'{t:.3f},1,{state_code},{self.wp_idx + 1},'
            f'{self.x:.3f},{self.y:.3f},{dist:.3f},'
            f'{math.degrees(self.yaw):.6f},{math.degrees(yaw_to_wp):.6f},'
            f'{math.degrees(yaw_ref):.6f},{math.degrees(yaw_err):.6f},0.000000,'
            f'{speed_ref:.6f},{self.speed_meas:.6f},'
            f'{math.degrees(self.roll):.6f},{math.degrees(self.roll_rate):.6f},'
            f'{math.degrees(self.roll_rms.get()):.6f},{math.degrees(self.ind.rms_roll()):.6f},'
            f'{math.degrees(self.pitch):.6f},{math.degrees(self.pitch_rate):.6f},'
            f'{math.degrees(self.pitch_rms.get()):.6f},{math.degrees(self.ind.rms_pitch()):.6f},'
            f'{0.5 * (self._last_tl + self._last_tr):.6f},{0.5 * (self._last_tr - self._last_tl):.6f},'
            f'{self._last_tl:.6f},{self._last_tr:.6f},'
            f'{cost["J"]:.8f},{cost["goal"]:.8f},{cost["wave"]:.8f},{cost["progress"]:.8f},{cost["smooth"]:.8f},'
            f'0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,'
            f'{self.ind.J_total:.8f},{self.ind.J_roll:.8f},{self.ind.J_pitch:.8f},'
            f'{self.ind.J_roll_rate:.8f},{self.ind.J_pitch_rate:.8f},'
            f'{self.ind.J_goal_heading:.8f},{self.ind.J_no_progress:.8f},{self.ind.J_control:.8f},'
            f'{self.ind.I_roll:.8f},{self.ind.I_pitch:.8f},'
            f'{self.ind.I_roll_rate:.8f},{self.ind.I_pitch_rate:.8f},{self.ind.I_control:.8f},'
            f'{self.ind.I_no_progress:.8f},{self.ind.I_goal_heading:.8f}\n'
        )

    def _print_status(self, now, dist, yaw_to_wp, yaw_ref, yaw_err, speed_ref, cost):
        if (now - self._last_status_update) < self.STATUS_UPDATE_PERIOD:
            return
        self._last_status_update = now

        print('\033[H\033[J', end='')
        print('════════════════════════════════════════════════════════════')
        print('  Roll-cost integral – WAM-V Autopilot bez stanu ROTATING')
        print('════════════════════════════════════════════════════════════')
        print(f'  Stan:             {self.state}')
        print(f'  Waypoint:         {self.wp_idx + 1}/{len(self.WAYPOINTS)}')
        print(f'  Pozycja:          ({self.x:.1f}, {self.y:.1f})')
        print(f'  Dystans:          {dist:.2f} m   promień WP: {self.WAYPOINT_RADIUS:.1f} m')
        print(f'  Yaw:              {math.degrees(self.yaw):.1f} deg')
        print(f'  Yaw do WP:        {math.degrees(yaw_to_wp):.1f} deg')
        print(f'  Yaw ref z J:      {math.degrees(yaw_ref):.1f} deg')
        print(f'  Błąd yaw:         {math.degrees(yaw_err):.1f} deg')
        print(f'  v_ref / v:        {speed_ref:.2f} / {self.speed_meas:.2f} m/s')
        print(f'  Roll/RMS okno:    {math.degrees(self.roll):.2f} / {math.degrees(self.roll_rms.get()):.2f} deg')
        print(f'  Roll RMS misji:   {math.degrees(self.ind.rms_roll()):.2f} deg')
        print(f'  Pitch/RMS okno:   {math.degrees(self.pitch):.2f} / {math.degrees(self.pitch_rms.get()):.2f} deg')
        print(f'  Pitch RMS misji:  {math.degrees(self.ind.rms_pitch()):.2f} deg')
        print('────────────────────────────────────────────────────────────')
        print('  Chwilowa funkcja kosztu wyboru kursu:')
        print('  J_candidate = J_goal + J_wave + J_progress + J_smooth')
        print(f'  J_total:          {cost["J"]:.5f}')
        print(f'  J_goal:           {cost["goal"]:.5f}   kara za odejście od kierunku do WP')
        print(f'  J_wave:           {cost["wave"]:.5f}   kara zależna od roll względem fali')
        print(f'  J_progress:       {cost["progress"]:.5f}   kara za mały postęp do celu')
        print(f'  J_smooth:         {cost["smooth"]:.5f}   kara za skok yaw_ref')
        print(f'  roll_severity:    {cost["roll_severity"]:.2f}')
        print('────────────────────────────────────────────────────────────')
        print('  Narastająca funkcja kosztu całej misji:')
        print('  J_total = ∫(w_roll*roll² + w_pitch*pitch² + w_rate*rate² + ...) dt')
        print(f'  J_total:          {self.ind.J_total:.6f}')
        print(f'  J_roll:           {self.ind.J_roll:.6f}   GŁÓWNA KARA: lewo/prawo')
        print(f'  J_pitch:          {self.ind.J_pitch:.6f}   mniejsza kara: góra/dół')
        print(f'  J_roll_rate:      {self.ind.J_roll_rate:.6f}')
        print(f'  J_pitch_rate:     {self.ind.J_pitch_rate:.6f}')
        print(f'  J_goal_heading:   {self.ind.J_goal_heading:.6f}')
        print(f'  J_no_progress:    {self.ind.J_no_progress:.6f}')
        print(f'  J_control:        {self.ind.J_control:.6f}')
        print(f'  Czas oceny:       {self.ind.t:.1f} s')
        print('────────────────────────────────────────────────────────────')
        print(f'  Ciąg L/P:         {self._last_tl:.1f} / {self._last_tr:.1f} N')
        print('  Log:              /tmp/wamv_roll_cost_integral_log.csv')
        print('════════════════════════════════════════════════════════════')

    def _setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.path_line, = self.ax.plot([], [], '-', label='Ślad katamaranu')
        self.current_pos_plot, = self.ax.plot([], [], 'o', markersize=8, label='Katamaran')

        for i, (wp_x, wp_y) in enumerate(self.WAYPOINTS, start=1):
            self.ax.add_patch(Circle(
                (wp_x, wp_y),
                radius=self.WAYPOINT_RADIUS,
                fill=False,
                linewidth=1.8
            ))
            self.ax.text(wp_x, wp_y, f'WP{i}', ha='center', va='center')

        self.ax.set_title('Waypointy jako okręgi zaliczenia + ślad katamaranu')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_aspect('equal', adjustable='box')

        xs = [p[0] for p in self.WAYPOINTS] + [self.START_X]
        ys = [p[1] for p in self.WAYPOINTS] + [self.START_Y]
        margin = self.WAYPOINT_RADIUS + 10.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)

    def _update_plot(self, now):
        if not self.history_x:
            return
        if (now - self._last_plot_update) < self.PLOT_UPDATE_PERIOD:
            return

        self._last_plot_update = now
        self.path_line.set_data(self.history_x, self.history_y)
        self.current_pos_plot.set_data([self.x], [self.y])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _save_final_plot(self):
        try:
            if self.history_x:
                self.path_line.set_data(self.history_x, self.history_y)
                self.current_pos_plot.set_data([self.x], [self.y])
            self.fig.savefig('/tmp/wamv_roll_cost_integral_path.png', dpi=150)
        except Exception as exc:
            self.get_logger().warning(f'Nie udało się zapisać wykresu: {exc}')

    def destroy_node(self):
        try:
            self._send_raw(0.0, 0.0)
        except Exception:
            pass
        try:
            self._log_file.close()
        except Exception:
            pass
        try:
            plt.close(self.fig)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RollCostIntegralAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Przerwano Ctrl+C')
    finally:
        if rclpy.ok():
            node._send_raw(0.0, 0.0)
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()

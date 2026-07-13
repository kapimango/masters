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
            self.J_roll + self.J_pitch + self.J_roll_rate + self.J_pitch_rate
            + self.J_goal_heading + self.J_no_progress + self.J_control
        )

    def rms_roll(self):
        return math.sqrt(self.I_roll / self.t) if self.t > 1e-9 else 0.0

    def rms_pitch(self):
        return math.sqrt(self.I_pitch / self.t) if self.t > 1e-9 else 0.0


class SamplingMPCStableAutopilot(Node):
    WAYPOINTS = [(-482.0, 190.0), (-482.0, 212.0), (-532.0, 190.0)]
    START_X, START_Y = -532.0, 190.0

    WAYPOINT_RADIUS = 4.0
    WAYPOINT_HOLD_TIME = 0.0

    WAVE_HEADING_DEG = 0.0

    PREFERRED_REL_TO_WAVE_DEG = 0.0

    CONTROL_PERIOD = 0.10
    MPC_DT = 0.25
    MPC_HORIZON = 22


    BASE_CANDIDATES = [-12.0, 0.0, 8.0, 16.0, 26.0, 38.0, 52.0, 66.0]
    DIFF_CANDIDATES = [-85.0, -65.0, -45.0, -28.0, -14.0, 0.0, 14.0, 28.0, 45.0, 65.0, 85.0]

    MAX_THRUST = 100.0
    MAX_REVERSE_THRUST = -20.0
    MAX_SLEW = 12.0

    K_SPEED = 0.018
    D_SPEED = 0.55
    K_YAW = 0.030
    D_YAW = 0.65
    MAX_PRED_SPEED = 2.2

    W_MPC_DISTANCE = 1.20
    W_MPC_FINAL_DISTANCE = 7.00
    W_MPC_PROGRESS = 38.00
    W_MPC_HEADING = 50.00
    W_MPC_WRONG_WAY = 105.00
    
    W_MPC_WAVE_ROLL = 0.70
    W_MPC_PITCH = 0.80
    W_MPC_CONTROL = 0.030
    W_MPC_SLEW = 0.080
    W_MPC_TURN = 0.040

    W_INT_ROLL = 12.0
    W_INT_PITCH = 2.0
    W_INT_ROLL_RATE = 2.5
    W_INT_PITCH_RATE = 0.5
    W_INT_GOAL_HEADING = 0.8
    W_INT_NO_PROGRESS = 2.0
    W_INT_CONTROL = 0.05

    ROLL_BAD_RAD = 0.08
    PITCH_BAD_RAD = 0.14

    SPEED_FILTER_ALPHA = 0.25
    STATUS_UPDATE_PERIOD = 0.35
    PLOT_UPDATE_PERIOD = 0.6


    DRIFT_FILTER_ALPHA = 0.12
    DRIFT_COMPENSATION_GAIN = 0.80
    MAX_DRIFT_SPEED = 1.00


    ACTION_HOLD_STEPS = 2 
    WAYPOINT_BRAKE_TIME = 0.80 
    STUCK_TIMEOUT = 5.0
    STUCK_PROGRESS_EPS = 0.35
    CRUISE_SPEED = 1.35
    APPROACH_SPEED = 0.65
    HEADING_LOCK_DEG = 70.0

    def __init__(self):
        super().__init__('sampling_mpc_robust_wave_autopilot')

        self.pub_l = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.sub_gps = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self._gps_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self._imu_cb, 10)
        self.timer = self.create_timer(self.CONTROL_PERIOD, self._control_loop)

        self.state = 'CALIBRATING'
        self.wp_idx = 0
        self.x, self.y = self.START_X, self.START_Y
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.speed_meas = 0.0
        self.gps_vx = 0.0
        self.gps_vy = 0.0
        self.forward_speed_est = 0.0
        self.drift_x = 0.0
        self.drift_y = 0.0

        self.init_lat = None
        self.init_lon = None
        self.m_per_deg_lon = None
        self.prev_gps_time = None

        self.roll_rms = MovingRMS(max_len=45)
        self.pitch_rms = MovingRMS(max_len=45)
        self.ind = MissionIndicators()
        self._last_indicator_time = None

        self._last_tl = 0.0
        self._last_tr = 0.0
        self._last_base = 0.0
        self._last_diff = 0.0
        self._wp_enter_time = None
        self._finished = False
        self._last_status_update = 0.0
        self._last_plot_update = 0.0
        self._last_mpc_info = None
        self._last_prediction = None
        self._post_wp_brake_until = 0.0
        self._best_dist_current_wp = float('inf')
        self._last_progress_time = time.monotonic()
        self._progress_wp_idx = self.wp_idx
        self._stuck_recovery = False

        self._t0 = time.monotonic()
        self._log_file = open('/tmp/wamv_sampling_mpc_robust_wave_log.csv', 'w')
        self._log_file.write(
            't,controller_id,state_code,wp_idx,x,y,dist,yaw_deg,yaw_to_wp_deg,yaw_ref_deg,yaw_err_deg,yaw_rate_deg_s,speed_ref,speed_meas,roll_deg,roll_rate_deg_s,roll_rms_window_deg,roll_rms_total_deg,pitch_deg,pitch_rate_deg_s,pitch_rms_window_deg,pitch_rms_total_deg,base_cmd,diff_cmd,T_left,T_right,J_inst,J_goal,J_wave_roll,J_progress,J_smooth,J_mpc_distance,J_mpc_final,J_mpc_heading,J_mpc_wrong_way,J_mpc_pitch,J_mpc_control,J_mpc_slew,J_mpc_turn,J_total,J_roll,J_pitch,J_roll_rate,J_pitch_rate,J_goal_heading,J_no_progress,J_control,I_roll,I_pitch,I_roll_rate,I_pitch_rate,I_control,I_no_progress,I_goal_heading\n'
        )
        self.history_x = []
        self.history_y = []
        self._setup_plot()

        self.get_logger().info('SamplingMPCRobustWaveAutopilot DUZE FALE + predykcja z dryfem uruchomiony ✓')
        self.get_logger().info('Regulator MPC-lite wybiera bezpośrednio T_L/T_R bez osobnego PID yaw.')


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
        self.yaw_rate = msg.angular_velocity.z

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
            vx = (self.x - old_x) / dt
            vy = (self.y - old_y) / dt
            raw_speed = math.hypot(vx, vy)
            self.speed_meas = (
                self.SPEED_FILTER_ALPHA * raw_speed
                + (1.0 - self.SPEED_FILTER_ALPHA) * self.speed_meas
            )

            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            forward_raw = vx * c + vy * s
            forward = clamp(forward_raw, 0.0, self.MAX_PRED_SPEED)
            drift_x_raw = vx - forward * c
            drift_y_raw = vy - forward * s
            drift_norm = math.hypot(drift_x_raw, drift_y_raw)
            if drift_norm > self.MAX_DRIFT_SPEED:
                scale = self.MAX_DRIFT_SPEED / max(drift_norm, 1e-9)
                drift_x_raw *= scale
                drift_y_raw *= scale

            a = self.DRIFT_FILTER_ALPHA
            self.gps_vx = a * vx + (1.0 - a) * self.gps_vx
            self.gps_vy = a * vy + (1.0 - a) * self.gps_vy
            self.forward_speed_est = a * forward + (1.0 - a) * self.forward_speed_est
            self.drift_x = a * drift_x_raw + (1.0 - a) * self.drift_x
            self.drift_y = a * drift_y_raw + (1.0 - a) * self.drift_y
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
        yaw_err_to_wp = wrap_pi(yaw_to_wp - self.yaw)

        if self._waypoint_reached(dist, now):
            self.wp_idx += 1
            self._wp_enter_time = None
            self._post_wp_brake_until = now + self.WAYPOINT_BRAKE_TIME
            self._best_dist_current_wp = float('inf')
            self._last_progress_time = now
            self._progress_wp_idx = self.wp_idx
            self._stuck_recovery = False
            self._last_base = 0.0
            self._last_diff = 0.0
            if self.wp_idx >= len(self.WAYPOINTS):
                self._finish()
                return
            self.get_logger().info(f'Waypoint osiągnięty -> WP{self.wp_idx + 1}; krótka stabilizacja przed następnym odcinkiem')
            return

        self._update_progress_guard(dist, now)

        if now < self._post_wp_brake_until:
            base_cmd, diff_cmd = 0.0, 0.0
            mpc_info = self._zero_mpc_info(dist)
            self._last_mpc_info = mpc_info
            self._last_base = base_cmd
            self._last_diff = diff_cmd
            self._last_prediction = self._build_prediction_visualization(tx, ty, base_cmd, diff_cmd)
            self._send(0.0, 0.0)
            self._update_mission_indicators(now, yaw_to_wp)
            self._log(now, dist, yaw_to_wp, yaw_err_to_wp, mpc_info)
            self._print_status(now, dist, yaw_to_wp, yaw_err_to_wp, base_cmd, diff_cmd, mpc_info)
            self._update_plot(now)
            return

        base_cmd, diff_cmd, mpc_info = self._choose_control_mpc(tx, ty)
        self._last_mpc_info = mpc_info
        self._last_base = base_cmd
        self._last_diff = diff_cmd

        self._last_prediction = self._build_prediction_visualization(tx, ty, base_cmd, diff_cmd)

        left = base_cmd - diff_cmd
        right = base_cmd + diff_cmd

        self._send(left, right)
        self._update_mission_indicators(now, yaw_to_wp)
        self._log(now, dist, yaw_to_wp, yaw_err_to_wp, mpc_info)
        self._print_status(now, dist, yaw_to_wp, yaw_err_to_wp, base_cmd, diff_cmd, mpc_info)
        self._update_plot(now)


    def _actual_base_diff(self):
        return 0.5 * (self._last_tl + self._last_tr), 0.5 * (self._last_tr - self._last_tl)

    def _slew_preview(self, target_l, target_r):
        def limit_delta(prev, target):
            delta = clamp(target - prev, -self.MAX_SLEW, self.MAX_SLEW)
            return prev + delta

        new_l = limit_delta(self._last_tl, target_l)
        new_r = limit_delta(self._last_tr, target_r)
        new_l = clamp(new_l, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        new_r = clamp(new_r, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        return new_l, new_r

    def _alloc_base_diff(self, base, diff):
        left = clamp(base - diff, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        right = clamp(base + diff, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        return 0.5 * (left + right), 0.5 * (right - left), left, right

    def _unique_sorted(self, values):
        return sorted(set(round(float(v), 3) for v in values))

    def _update_progress_guard(self, dist, now):
        if self._progress_wp_idx != self.wp_idx:
            self._progress_wp_idx = self.wp_idx
            self._best_dist_current_wp = dist
            self._last_progress_time = now
            self._stuck_recovery = False
            return

        if dist < self._best_dist_current_wp - self.STUCK_PROGRESS_EPS:
            self._best_dist_current_wp = dist
            self._last_progress_time = now
            self._stuck_recovery = False
            return

        self._stuck_recovery = (
            (now - self._last_progress_time) > self.STUCK_TIMEOUT
            and dist > 2.5 * self.WAYPOINT_RADIUS
        )

    def _zero_mpc_info(self, dist):
        return {
            'J': 0.0,
            'distance': (dist / 12.0) ** 2 * self.W_MPC_DISTANCE,
            'final': (dist / 6.0) ** 2 * self.W_MPC_FINAL_DISTANCE,
            'progress': 0.0,
            'heading': 0.0,
            'wrong_way': 0.0,
            'wave_roll': 0.0,
            'pitch': 0.0,
            'control': 0.0,
            'slew': 0.0,
            'turn': 0.0,
            'roll_severity': 0.0,
            'pitch_severity': 0.0,
            'pred_final_dist': dist,
            'pred_final_x': self.x,
            'pred_final_y': self.y,
            'pred_final_yaw_deg': math.degrees(self.yaw),
        }

    def _candidate_sets(self, tx, ty):
        dist_now = math.hypot(tx - self.x, ty - self.y)
        yaw_to_wp = math.atan2(ty - self.y, tx - self.x)
        yaw_err = abs(wrap_pi(yaw_to_wp - self.yaw))
        roll_severity = max(abs(self.roll), self.roll_rms.get()) / max(self.ROLL_BAD_RAD, 1e-9)
        actual_base, actual_diff = self._actual_base_diff()

        base_candidates = list(self.BASE_CANDIDATES)
        diff_candidates = list(self.DIFF_CANDIDATES)

        base_candidates += [actual_base]
        diff_candidates += [actual_diff]

        if dist_now < 2.5 * self.WAYPOINT_RADIUS:
            near_actual_base = clamp(actual_base, -4.0, 22.0)
            near_actual_diff = clamp(actual_diff, -38.0, 38.0)
            base_candidates = [0.0, 4.0, 8.0, 12.0, 18.0, 22.0, near_actual_base]
            diff_candidates = [-38.0, -25.0, -12.0, 0.0, 12.0, 25.0, 38.0, near_actual_diff]
            return self._unique_sorted(base_candidates), self._unique_sorted(diff_candidates)

        if yaw_err > math.radians(110.0):
            base_candidates = [-20.0, -12.0, 0.0, 6.0, 12.0, 20.0, actual_base]
            diff_candidates = [-90.0, -75.0, -55.0, -35.0, -18.0, 18.0, 35.0, 55.0, 75.0, 90.0, actual_diff]
        elif yaw_err > math.radians(self.HEADING_LOCK_DEG):
            base_candidates = [-12.0, 0.0, 8.0, 16.0, 26.0, 38.0, actual_base]
            diff_candidates = [-85.0, -65.0, -45.0, -28.0, 0.0, 28.0, 45.0, 65.0, 85.0, actual_diff]

        if roll_severity > 1.30:
            base_candidates = [-16.0, -8.0, 0.0, 8.0, 18.0, 30.0, 42.0, 55.0, actual_base]
            diff_candidates += [-95.0, -75.0, -55.0, 0.0, 55.0, 75.0, 95.0]

        if self._stuck_recovery:
            base_candidates = [-20.0, -12.0, 0.0, 8.0, 18.0, 30.0]
            diff_candidates = [-95.0, -75.0, -55.0, -35.0, 0.0, 35.0, 55.0, 75.0, 95.0]

        return self._unique_sorted(base_candidates), self._unique_sorted(diff_candidates)

    def _choose_control_mpc(self, tx, ty):
        best_J = float('inf')
        best_base = 0.0
        best_diff = 0.0
        best_info = None

        base_candidates, diff_candidates = self._candidate_sets(tx, ty)
        tested = set()

        for base in base_candidates:
            for diff in diff_candidates:
                _, _, target_l, target_r = self._alloc_base_diff(base, diff)


                applied_l, applied_r = self._slew_preview(target_l, target_r)
                real_base = 0.5 * (applied_l + applied_r)
                real_diff = 0.5 * (applied_r - applied_l)

                key = (round(real_base, 3), round(real_diff, 3))
                if key in tested:
                    continue
                tested.add(key)

                J, info = self._simulate_candidate(real_base, real_diff, tx, ty)
                if J < best_J:
                    best_J = J
                    best_base = real_base
                    best_diff = real_diff
                    best_info = info

        if best_info is None:
            best_info = self._zero_mpc_info(math.hypot(tx - self.x, ty - self.y))
        return best_base, best_diff, best_info

    def _tail_policy(self, x, y, yaw, v, yaw_rate, tx, ty, prev_base, prev_diff):
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        yaw_to_wp = math.atan2(dy, dx)
        yaw_err = wrap_pi(yaw_to_wp - yaw)
        abs_err = abs(yaw_err)

        if abs_err > math.radians(115.0):
            speed_ref = 0.0
        elif abs_err > math.radians(self.HEADING_LOCK_DEG):
            speed_ref = 0.25
        elif dist < 2.5 * self.WAYPOINT_RADIUS:
            speed_ref = 0.25
        elif dist < 10.0:
            speed_ref = 0.55
        elif dist < 25.0:
            speed_ref = 1.05
        else:
            speed_ref = self.CRUISE_SPEED

        base_ff = (self.D_SPEED / max(self.K_SPEED, 1e-9)) * speed_ref
        base_fb = 18.0 * (speed_ref - v)
        desired_base = base_ff + base_fb

        if abs_err > math.radians(100.0) and v > 0.45:
            desired_base = min(desired_base, -8.0)
        elif dist < 2.5 * self.WAYPOINT_RADIUS and v > 0.85:
            desired_base = min(desired_base, 0.0)

        desired_base = clamp(desired_base, self.MAX_REVERSE_THRUST, 70.0)

        desired_diff = 58.0 * yaw_err - 18.0 * yaw_rate
        if dist < 2.5 * self.WAYPOINT_RADIUS:
            desired_diff = clamp(desired_diff, -42.0, 42.0)
            desired_base = clamp(desired_base, 0.0, 24.0)
        elif abs_err > math.radians(100.0):
            desired_diff = clamp(desired_diff, -90.0, 90.0)
        else:
            desired_diff = clamp(desired_diff, -75.0, 75.0)

        max_du = self.MAX_SLEW * max(self.MPC_DT / max(self.CONTROL_PERIOD, 1e-9), 1.0)
        desired_base = prev_base + clamp(desired_base - prev_base, -max_du, max_du)
        desired_diff = prev_diff + clamp(desired_diff - prev_diff, -max_du, max_du)

        real_base, real_diff, _, _ = self._alloc_base_diff(desired_base, desired_diff)
        return real_base, real_diff

    def _simulate_rollout(self, first_base, first_diff, tx, ty, collect_path=False):
        x = self.x
        y = self.y
        x0 = self.x
        y0 = self.y
        yaw = self.yaw
        v = max(0.0, self.forward_speed_est)
        yaw_rate = self.yaw_rate
        drift_x = self.DRIFT_COMPENSATION_GAIN * self.drift_x
        drift_y = self.DRIFT_COMPENSATION_GAIN * self.drift_y

        prev_dist = math.hypot(tx - x, ty - y)
        drift_gate = clamp((prev_dist - self.WAYPOINT_RADIUS) / (3.0 * self.WAYPOINT_RADIUS), 0.35, 1.0)
        drift_x *= drift_gate
        drift_y *= drift_gate
        min_dist = prev_dist
        captured_in_prediction = prev_dist <= self.WAYPOINT_RADIUS
        wave_heading = deg2rad(self.WAVE_HEADING_DEG)
        preferred_rel = deg2rad(self.PREFERRED_REL_TO_WAVE_DEG)

        roll_severity = clamp(max(abs(self.roll), self.roll_rms.get()) / self.ROLL_BAD_RAD, 0.0, 2.0)
        pitch_severity = clamp(max(abs(self.pitch), self.pitch_rms.get()) / self.PITCH_BAD_RAD, 0.0, 2.0)

        J_distance = 0.0
        J_progress = 0.0
        J_heading = 0.0
        J_wrong_way = 0.0
        J_wave_roll = 0.0
        J_pitch = 0.0
        J_control = 0.0
        J_slew = 0.0
        J_turn = 0.0

        prev_base, prev_diff = self._actual_base_diff()
        path_x = [0.0]
        path_y = [0.0]

        for k in range(self.MPC_HORIZON):
            if k < self.ACTION_HOLD_STEPS:
                cmd_base, cmd_diff = first_base, first_diff
            else:
                cmd_base, cmd_diff = self._tail_policy(x, y, yaw, v, yaw_rate, tx, ty, prev_base, prev_diff)

            cmd_base, cmd_diff, _, _ = self._alloc_base_diff(cmd_base, cmd_diff)

            dbase = cmd_base - prev_base
            ddiff = cmd_diff - prev_diff
            J_slew += (dbase / self.MAX_THRUST) ** 2 + (ddiff / self.MAX_THRUST) ** 2
            prev_base, prev_diff = cmd_base, cmd_diff

            acc = self.K_SPEED * cmd_base - self.D_SPEED * v
            v = clamp(v + acc * self.MPC_DT, 0.0, self.MAX_PRED_SPEED)

            yaw_acc = self.K_YAW * cmd_diff - self.D_YAW * yaw_rate
            yaw_rate = clamp(yaw_rate + yaw_acc * self.MPC_DT, -2.0, 2.0)
            yaw = wrap_pi(yaw + yaw_rate * self.MPC_DT)

            x += (v * math.cos(yaw) + drift_x) * self.MPC_DT
            y += (v * math.sin(yaw) + drift_y) * self.MPC_DT
            if collect_path:
                path_x.append(x - x0)
                path_y.append(y - y0)

            dist = math.hypot(tx - x, ty - y)
            yaw_to_wp = math.atan2(ty - y, tx - x)
            yaw_goal_err = wrap_pi(yaw_to_wp - yaw)

            min_dist = min(min_dist, dist)
            if dist <= self.WAYPOINT_RADIUS:
                captured_in_prediction = True
                break

            near_wp_gate = clamp((dist - self.WAYPOINT_RADIUS) / (3.0 * self.WAYPOINT_RADIUS), 0.15, 1.0)
            J_distance += near_wp_gate * (dist / 12.0) ** 2
            J_heading += near_wp_gate * yaw_goal_err * yaw_goal_err

            heading_badness = max(0.0, abs(yaw_goal_err) - math.radians(50.0)) / math.radians(130.0)
            if dist > 1.7 * self.WAYPOINT_RADIUS:
                J_wrong_way += heading_badness * heading_badness * (v / max(self.MAX_PRED_SPEED, 1e-9)) ** 2

            progress = prev_dist - dist
            progress_gate = clamp((dist - self.WAYPOINT_RADIUS) / (3.0 * self.WAYPOINT_RADIUS), 0.0, 1.0)
            if progress < 0.0:
                J_progress += progress_gate * (10.0 * (-progress / 1.0) ** 2 + 2.5)
            else:
                J_progress += progress_gate * max(0.0, 1.0 - progress / max(v * self.MPC_DT + 1e-6, 1e-6))
            prev_dist = dist

            wave_err = min(
                abs(wrap_pi(yaw - wave_heading - preferred_rel)),
                abs(wrap_pi(yaw - wave_heading + preferred_rel))
            )
            roll_gate = clamp((roll_severity - 0.45) / 1.20, 0.0, 1.0)
            navigation_gate = 1.0 - clamp((abs(yaw_goal_err) - math.radians(35.0)) / math.radians(55.0), 0.0, 1.0)
            if self._stuck_recovery:
                roll_gate = 0.0
            J_wave_roll += (roll_gate ** 2) * navigation_gate * wave_err * wave_err

            speed_pitch_proxy = (v / max(self.MAX_PRED_SPEED, 1e-9)) ** 2
            J_pitch += (0.35 + pitch_severity) * speed_pitch_proxy

            control_norm = (cmd_base / self.MAX_THRUST) ** 2 + (cmd_diff / self.MAX_THRUST) ** 2
            J_control += control_norm
            J_turn += (yaw_rate / 1.5) ** 2

        final_dist = math.hypot(tx - x, ty - y)
        if captured_in_prediction:
            J_final = 0.0
            final_dist = min_dist
        else:
            J_final = (min_dist / 6.0) ** 2

        J = (
            self.W_MPC_DISTANCE * J_distance
            + self.W_MPC_FINAL_DISTANCE * J_final
            + self.W_MPC_PROGRESS * J_progress
            + self.W_MPC_HEADING * J_heading
            + self.W_MPC_WRONG_WAY * J_wrong_way
            + self.W_MPC_WAVE_ROLL * J_wave_roll
            + self.W_MPC_PITCH * J_pitch
            + self.W_MPC_CONTROL * J_control
            + self.W_MPC_SLEW * J_slew
            + self.W_MPC_TURN * J_turn
        )

        info = {
            'J': J,
            'distance': self.W_MPC_DISTANCE * J_distance,
            'final': self.W_MPC_FINAL_DISTANCE * J_final,
            'progress': self.W_MPC_PROGRESS * J_progress,
            'heading': self.W_MPC_HEADING * J_heading,
            'wrong_way': self.W_MPC_WRONG_WAY * J_wrong_way,
            'wave_roll': self.W_MPC_WAVE_ROLL * J_wave_roll,
            'pitch': self.W_MPC_PITCH * J_pitch,
            'control': self.W_MPC_CONTROL * J_control,
            'slew': self.W_MPC_SLEW * J_slew,
            'turn': self.W_MPC_TURN * J_turn,
            'roll_severity': roll_severity,
            'pitch_severity': pitch_severity,
            'pred_final_dist': final_dist,
            'pred_final_x': x,
            'pred_final_y': y,
            'pred_final_yaw_deg': math.degrees(yaw),
        }
        if collect_path:
            return J, info, path_x, path_y
        return J, info

    def _simulate_candidate(self, base, diff, tx, ty):
        return self._simulate_rollout(base, diff, tx, ty, collect_path=False)


    def _predict_path_for_visualization(self, base, diff, tx, ty):

        _, _, xs, ys = self._simulate_rollout(base, diff, tx, ty, collect_path=True)
        final_x = self.x + xs[-1]
        final_y = self.y + ys[-1]
        final_dist = math.hypot(tx - final_x, ty - final_y)
        return xs, ys, final_dist

    def _build_prediction_visualization(self, tx, ty, selected_base, selected_diff):

        candidates = []
        base_candidates, diff_candidates = self._candidate_sets(tx, ty)

        for base in base_candidates:
            for diff in diff_candidates:
                left = clamp(base - diff, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
                right = clamp(base + diff, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
                real_base = 0.5 * (left + right)
                real_diff = 0.5 * (right - left)
                xs, ys, final_dist = self._predict_path_for_visualization(real_base, real_diff, tx, ty)
                candidates.append({
                    'base': real_base,
                    'diff': real_diff,
                    'xs': xs,
                    'ys': ys,
                    'final_dist': final_dist,
                })

        best_xs, best_ys, best_final_dist = self._predict_path_for_visualization(selected_base, selected_diff, tx, ty)

        return {
            'candidates': candidates,
            'best_xs': best_xs,
            'best_ys': best_ys,
            'best_final_dist': best_final_dist,
            'wp_local_x': tx - self.x,
            'wp_local_y': ty - self.y,
            'selected_base': selected_base,
            'selected_diff': selected_diff,
        }


    def _update_mission_indicators(self, now, yaw_to_wp):
        if self._last_indicator_time is None:
            self._last_indicator_time = now
            return

        dt = now - self._last_indicator_time
        self._last_indicator_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        goal_heading_error = wrap_pi(yaw_to_wp - self.yaw)
        progress_measure = math.cos(goal_heading_error)
        no_progress = max(0.0, 1.0 - progress_measure)

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

        if self.WAYPOINT_HOLD_TIME <= 0.0:
            return True

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
        self._last_tl, self._last_tr = new_l, new_r
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
        print(f'  Regulator:        Sampling MPC / receding horizon')
        print(f'  J_total:          {self.ind.J_total:.6f}')
        print(f'  J_roll:           {self.ind.J_roll:.6f}')
        print(f'  J_pitch:          {self.ind.J_pitch:.6f}')
        print(f'  Roll RMS misji:   {math.degrees(self.ind.rms_roll()):.3f} deg')
        print(f'  Pitch RMS misji:  {math.degrees(self.ind.rms_pitch()):.3f} deg')
        print('  Log:     /tmp/wamv_sampling_mpc_robust_wave_log.csv')
        print('  Wykres:  /tmp/wamv_sampling_mpc_robust_wave_path.png')
        print('  Predykcja: /tmp/wamv_sampling_mpc_robust_wave_prediction_viz.png')
        print('══════════════════════════════════════════')
        self.get_logger().info('FINISHED')
        self.timer.cancel()
        rclpy.shutdown()


    def _log(self, now, dist, yaw_to_wp, yaw_err_to_wp, info):
        t = now - self._t0
        state_code = {'CALIBRATING': 0, 'RUNNING': 1, 'FINISHED': 2}.get(self.state, -1)

        if info is None:
            info = {k: 0.0 for k in ['J', 'distance', 'final', 'progress', 'heading', 'wrong_way', 'wave_roll', 'pitch', 'control', 'slew', 'turn']}

        self._log_file.write(
            f'{t:.3f},2,{state_code},{self.wp_idx + 1},'
            f'{self.x:.3f},{self.y:.3f},{dist:.3f},'
            f'{math.degrees(self.yaw):.6f},{math.degrees(yaw_to_wp):.6f},'
            f'{math.degrees(yaw_to_wp):.6f},{math.degrees(yaw_err_to_wp):.6f},{math.degrees(self.yaw_rate):.6f},'
            f'{float("nan"):.6f},{self.speed_meas:.6f},'
            f'{math.degrees(self.roll):.6f},{math.degrees(self.roll_rate):.6f},'
            f'{math.degrees(self.roll_rms.get()):.6f},{math.degrees(self.ind.rms_roll()):.6f},'
            f'{math.degrees(self.pitch):.6f},{math.degrees(self.pitch_rate):.6f},'
            f'{math.degrees(self.pitch_rms.get()):.6f},{math.degrees(self.ind.rms_pitch()):.6f},'
            f'{self._last_base:.6f},{self._last_diff:.6f},{self._last_tl:.6f},{self._last_tr:.6f},'
            f'{info["J"]:.8f},0.000000,{info["wave_roll"]:.8f},{info["progress"]:.8f},0.000000,'
            f'{info["distance"]:.8f},{info["final"]:.8f},{info["heading"]:.8f},{info["wrong_way"]:.8f},'
            f'{info["pitch"]:.8f},{info["control"]:.8f},{info["slew"]:.8f},{info["turn"]:.8f},'
            f'{self.ind.J_total:.8f},{self.ind.J_roll:.8f},{self.ind.J_pitch:.8f},'
            f'{self.ind.J_roll_rate:.8f},{self.ind.J_pitch_rate:.8f},'
            f'{self.ind.J_goal_heading:.8f},{self.ind.J_no_progress:.8f},{self.ind.J_control:.8f},'
            f'{self.ind.I_roll:.8f},{self.ind.I_pitch:.8f},'
            f'{self.ind.I_roll_rate:.8f},{self.ind.I_pitch_rate:.8f},{self.ind.I_control:.8f},'
            f'{self.ind.I_no_progress:.8f},{self.ind.I_goal_heading:.8f}\n'
        )

    def _print_status(self, now, dist, yaw_to_wp, yaw_err_to_wp, base_cmd, diff_cmd, info):
        if (now - self._last_status_update) < self.STATUS_UPDATE_PERIOD:
            return
        self._last_status_update = now

        print('\033[H\033[J', end='')
        print('════════════════════════════════════════════════════════════')
        print('  Sampling MPC FIXED WP3 DUZE FALE – predykcja z estymowanym dryfem GPS')
        print('════════════════════════════════════════════════════════════')
        print(f'  Stan:             {self.state}')
        if self._stuck_recovery:
            print('  Tryb:             ODZYSKIWANIE POSTĘPU – ignoruję falę i łapię kurs do WP')
        print(f'  Waypoint:         {self.wp_idx + 1}/{len(self.WAYPOINTS)}')
        print(f'  Pozycja:          ({self.x:.1f}, {self.y:.1f})')
        print(f'  Dystans:          {dist:.2f} m   promień WP: {self.WAYPOINT_RADIUS:.1f} m')
        print(f'  Yaw:              {math.degrees(self.yaw):.1f} deg')
        print(f'  Yaw do WP:        {math.degrees(yaw_to_wp):.1f} deg')
        print(f'  Błąd yaw do WP:   {math.degrees(yaw_err_to_wp):.1f} deg')
        print(f'  v GPS:            {self.speed_meas:.2f} m/s')
        print(f'  v wzdłuż kadłuba: {self.forward_speed_est:.2f} m/s')
        print(f'  dryf est.:        ({self.drift_x:.2f}, {self.drift_y:.2f}) m/s')
        print(f'  Roll/RMS okno:    {math.degrees(self.roll):.2f} / {math.degrees(self.roll_rms.get()):.2f} deg')
        print(f'  Roll RMS misji:   {math.degrees(self.ind.rms_roll()):.2f} deg')
        print(f'  Pitch/RMS okno:   {math.degrees(self.pitch):.2f} / {math.degrees(self.pitch_rms.get()):.2f} deg')
        print(f'  Pitch RMS misji:  {math.degrees(self.ind.rms_pitch()):.2f} deg')
        print('────────────────────────────────────────────────────────────')
        print('  Sterowanie wybrane przez MPC:')
        print(f'  base:             {base_cmd:.1f} N')
        print(f'  diff:             {diff_cmd:.1f} N')
        print(f'  Ciąg L/P:         {self._last_tl:.1f} / {self._last_tr:.1f} N')
        print('────────────────────────────────────────────────────────────')
        print('  Funkcja kosztu MPC na horyzoncie predykcji:')
        print('  J_mpc = J_dist + J_final + J_progress + J_heading + J_wrong_way + J_wave_roll + J_pitch + J_control + J_slew + J_turn')
        print(f'  J_mpc:            {info["J"]:.5f}')
        print(f'  J_dist:           {info["distance"]:.5f}')
        print(f'  J_final:          {info["final"]:.5f}')
        print(f'  J_progress:       {info["progress"]:.5f}')
        print(f'  J_heading:        {info["heading"]:.5f}')
        print(f'  J_wrong_way:      {info["wrong_way"]:.5f}   kara za płynięcie nie w stronę WP')
        print(f'  J_wave_roll:      {info["wave_roll"]:.5f}   GŁÓWNA KARA: roll względem fali')
        print(f'  J_pitch:          {info["pitch"]:.5f}   pomocniczo')
        print(f'  J_control:        {info["control"]:.5f}')
        print(f'  J_slew:           {info["slew"]:.5f}')
        print(f'  J_turn:           {info["turn"]:.5f}')
        print(f'  Pred. dist final: {info["pred_final_dist"]:.2f} m')
        print('────────────────────────────────────────────────────────────')
        print('  Narastająca ocena misji – taka sama jak dla poprzedniego regulatora:')
        print(f'  J_total:          {self.ind.J_total:.6f}')
        print(f'  J_roll:           {self.ind.J_roll:.6f}')
        print(f'  J_pitch:          {self.ind.J_pitch:.6f}')
        print(f'  J_roll_rate:      {self.ind.J_roll_rate:.6f}')
        print(f'  J_pitch_rate:     {self.ind.J_pitch_rate:.6f}')
        print(f'  J_no_progress:    {self.ind.J_no_progress:.6f}')
        print(f'  Czas oceny:       {self.ind.t:.1f} s')
        print('  Log:              /tmp/wamv_sampling_mpc_robust_wave_log.csv')
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

        self.ax.set_title('Sampling MPC FIXED WP3 DUZE FALE: trajektoria + okręgi waypointów')
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

        self.fig_pred, self.ax_pred = plt.subplots(figsize=(7, 6))
        self.ax_pred.set_title('Pasywna wizualizacja MPC: predykcja kilku kroków w przód')
        self.ax_pred.set_xlabel('ΔX względem katamaranu [m]')
        self.ax_pred.set_ylabel('ΔY względem katamaranu [m]')
        self.ax_pred.grid(True)
        self.ax_pred.set_aspect('equal', adjustable='box')

    def _update_plot(self, now):
        if (now - self._last_plot_update) < self.PLOT_UPDATE_PERIOD:
            return
        self._last_plot_update = now

        if not self.history_x:
            return

        self.path_line.set_data(self.history_x, self.history_y)
        self.current_pos_plot.set_data([self.x], [self.y])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self._update_prediction_plot()
        plt.pause(0.001)


    def _update_prediction_plot(self):
        if not self._last_prediction:
            return

        p = self._last_prediction
        self.ax_pred.clear()
        self.ax_pred.set_title(
            f'Predykcja MPC lokalnie: base={p["selected_base"]:.1f} N, '
            f'diff={p["selected_diff"]:.1f} N, horyzont={self.MPC_HORIZON * self.MPC_DT:.1f} s'
        )
        self.ax_pred.set_xlabel('ΔX względem aktualnej pozycji [m]')
        self.ax_pred.set_ylabel('ΔY względem aktualnej pozycji [m]')
        self.ax_pred.grid(True)
        self.ax_pred.set_aspect('equal', adjustable='box')

        self.ax_pred.axhline(0.0, linewidth=0.8)
        self.ax_pred.axvline(0.0, linewidth=0.8)

        self.ax_pred.plot([0.0], [0.0], 'o', markersize=8, label='katamaran teraz')

        self.ax_pred.plot([p['wp_local_x']], [p['wp_local_y']], 'o', markersize=7, label='waypoint lokalnie')
        self.ax_pred.add_patch(Circle(
            (p['wp_local_x'], p['wp_local_y']),
            radius=self.WAYPOINT_RADIUS,
            fill=False,
            linewidth=1.3
        ))

        for cand in p['candidates']:
            self.ax_pred.plot(cand['xs'], cand['ys'], '-', alpha=0.18, linewidth=0.8)

        self.ax_pred.plot(p['best_xs'], p['best_ys'], '-', linewidth=2.5, label='wybrana predykcja MPC')
        self.ax_pred.plot([p['best_xs'][-1]], [p['best_ys'][-1]], 'o', markersize=7, label='koniec wybranej predykcji')

        all_x = [0.0, p['wp_local_x']] + [x for c in p['candidates'] for x in c['xs']]
        all_y = [0.0, p['wp_local_y']] + [y for c in p['candidates'] for y in c['ys']]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        span = max(max_x - min_x, max_y - min_y, 8.0)
        cx = 0.5 * (min_x + max_x)
        cy = 0.5 * (min_y + max_y)
        margin = 0.25 * span
        self.ax_pred.set_xlim(cx - span / 2 - margin, cx + span / 2 + margin)
        self.ax_pred.set_ylim(cy - span / 2 - margin, cy + span / 2 + margin)

        self.ax_pred.legend(loc='best')
        self.fig_pred.canvas.draw()
        self.fig_pred.canvas.flush_events()

    def _save_final_plot(self):
        if self.history_x:
            self.path_line.set_data(self.history_x, self.history_y)
            self.current_pos_plot.set_data([self.x], [self.y])
        self.fig.savefig('/tmp/wamv_sampling_mpc_robust_wave_path.png', dpi=160, bbox_inches='tight')
        if hasattr(self, 'fig_pred'):
            self.fig_pred.savefig('/tmp/wamv_sampling_mpc_robust_wave_prediction_viz.png', dpi=160, bbox_inches='tight')

    def destroy_node(self):
        try:
            self._log_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SamplingMPCStableAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send_raw(0.0, 0.0)
    finally:
        if rclpy.ok():
            node._send_raw(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()

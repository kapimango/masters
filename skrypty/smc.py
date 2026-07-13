import math
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RMSWindow:
    def __init__(self, window_s=4.0):
        self.window_s = window_s
        self.samples = deque()

    def add(self, t, value):
        self.samples.append((t, value))
        while self.samples and (t - self.samples[0][0]) > self.window_s:
            self.samples.popleft()

    def rms(self):
        if not self.samples:
            return 0.0
        return math.sqrt(sum(v * v for _, v in self.samples) / len(self.samples))


class IntegralIndicators:
    def __init__(self):
        self.t_eval = 0.0

        self.I_roll = 0.0
        self.I_pitch = 0.0
        self.I_roll_rate = 0.0
        self.I_pitch_rate = 0.0
        self.I_control = 0.0
        self.I_no_progress = 0.0
        self.I_heading = 0.0

        self.J_total = 0.0

    def update(self, dt, roll, pitch, roll_rate, pitch_rate,
               left, right, yaw_err, progress):
        if dt <= 0.0:
            return

        self.t_eval += dt

        self.I_roll += roll * roll * dt
        self.I_pitch += pitch * pitch * dt
        self.I_roll_rate += roll_rate * roll_rate * dt
        self.I_pitch_rate += pitch_rate * pitch_rate * dt

        u_norm = ((left / 100.0) ** 2 + (right / 100.0) ** 2) / 2.0
        self.I_control += u_norm * dt

        no_progress = max(0.0, 0.15 - progress)
        self.I_no_progress += no_progress * no_progress * dt

        self.I_heading += yaw_err * yaw_err * dt

        W_ROLL = 14.0
        W_PITCH = 2.0
        W_ROLL_RATE = 2.5
        W_PITCH_RATE = 0.5
        W_CONTROL = 0.20
        W_NO_PROGRESS = 4.0
        W_HEADING = 0.6

        self.J_total = (
            W_ROLL * self.I_roll
            + W_PITCH * self.I_pitch
            + W_ROLL_RATE * self.I_roll_rate
            + W_PITCH_RATE * self.I_pitch_rate
            + W_CONTROL * self.I_control
            + W_NO_PROGRESS * self.I_no_progress
            + W_HEADING * self.I_heading
        )

    def roll_rms_total(self):
        if self.t_eval <= 1e-9:
            return 0.0
        return math.sqrt(self.I_roll / self.t_eval)

    def pitch_rms_total(self):
        if self.t_eval <= 1e-9:
            return 0.0
        return math.sqrt(self.I_pitch / self.t_eval)


class LOSSMCAdaptiveController(Node):
    WAYPOINTS = [(-482.0, 190.0), (-482.0, 212.0), (-532.0, 190.0)]
    START_X = -532.0
    START_Y = 190.0

    WAYPOINT_RADIUS = 4.0
    WAYPOINT_HOLD_TIME = 0.35

    MAX_THRUST = 100.0
    MAX_REVERSE_THRUST = -20.0
    MAX_SLEW = 8.0

    BASE_THRUST_MAX = 72.0
    BASE_THRUST_MIN = 8.0
    BASE_THRUST_NEAR_WP = 18.0
    DIST_SLOWDOWN_RADIUS = 18.0
    ROLL_BAD_RAD = math.radians(7.0)
    PITCH_BAD_RAD = math.radians(8.0)

    ROLL_SLOWDOWN_GAIN = 0.75
    PITCH_SLOWDOWN_GAIN = 0.25
    HEADING_SLOWDOWN_GAIN = 0.70

    LOS_LOOKAHEAD = 10.0


    SMC_LAMBDA = 0.65
    SMC_K_LINEAR = 34.0
    SMC_K_SWITCH = 26.0
    SMC_EPS = math.radians(8.0)
    SMC_K_YAW_RATE = 10.0 
    YAW_INT_LIMIT = math.radians(50.0)
    TURN_LIMIT = 72.0

    WRONG_WAY_DEG = 110.0
    WRONG_WAY_BASE_LIMIT = 18.0

    CONTROL_DT = 0.1
    PLOT_DT = 0.5
    PRINT_DT = 0.5

    def __init__(self):
        super().__init__("los_smc_adaptive_controller")

        self.pub_l = self.create_publisher(Float64, "/wamv/thrusters/left/thrust", 10)
        self.pub_r = self.create_publisher(Float64, "/wamv/thrusters/right/thrust", 10)

        self.sub_gps = self.create_subscription(
            NavSatFix,
            "/wamv/sensors/gps/gps/fix",
            self._gps_cb,
            10,
        )
        self.sub_imu = self.create_subscription(
            Imu,
            "/wamv/sensors/imu/imu/data",
            self._imu_cb,
            10,
        )

        self.timer = self.create_timer(self.CONTROL_DT, self._control_loop)

        self.state = "CALIBRATING"
        self.wp_idx = 0

        self.init_lat = None
        self.init_lon = None
        self.m_per_deg_lon = None
        self.m_per_deg_lat = 111320.0

        self.x = self.START_X
        self.y = self.START_Y
        self.prev_x = None
        self.prev_y = None
        self.prev_gps_t = None

        self.speed = 0.0
        self.speed_filter_alpha = 0.25

        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0

        self.prev_roll = None
        self.prev_pitch = None
        self.prev_imu_t = None

        self.yaw_integral = 0.0

        self._last_tl = 0.0
        self._last_tr = 0.0

        self._last_loop_time = time.monotonic()
        self._t0 = time.monotonic()
        self._last_print = 0.0
        self._last_plot = 0.0
        self._finished_printed = False

        self._wp_inside_since = None

        self.roll_window = RMSWindow(window_s=4.0)
        self.pitch_window = RMSWindow(window_s=4.0)
        self.ind = IntegralIndicators()

        self.hist_t = []
        self.hist_x = []
        self.hist_y = []
        self.hist_left = []
        self.hist_right = []
        self.hist_base = []
        self.hist_turn = []
        self.hist_roll_deg = []
        self.hist_pitch_deg = []
        self.hist_J = []

        self.log_path = "/tmp/wamv_los_smc_adaptive_log.csv"
        self._log_file = open(self.log_path, "w", buffering=1)
        self._log_file.write(
            "t,x,y,wp_idx,dist,"
            "yaw_deg,yaw_ref_deg,yaw_err_deg,yaw_rate_deg_s,"
            "roll_deg,pitch_deg,roll_rate_deg_s,pitch_rate_deg_s,"
            "speed,base,turn,T_left,T_right,"
            "roll_rms_window_deg,pitch_rms_window_deg,"
            "roll_rms_total_deg,pitch_rms_total_deg,"
            "I_roll,I_pitch,I_roll_rate,I_pitch_rate,I_control,I_no_progress,I_heading,J_total,"
            "state\n"
        )

        plt.ion()
        self.fig_path, self.ax_path = plt.subplots(figsize=(7, 6))
        self.path_line, = self.ax_path.plot([], [], "-", label="Ślad katamaranu")
        self.current_point, = self.ax_path.plot([], [], "o", label="Katamaran")
        self._setup_path_plot()

        self.fig_ctrl, self.ax_ctrl = plt.subplots(figsize=(8, 4.5))
        self.left_line, = self.ax_ctrl.plot([], [], "-", label="T_left [N]")
        self.right_line, = self.ax_ctrl.plot([], [], "-", label="T_right [N]")
        self.base_line, = self.ax_ctrl.plot([], [], "--", label="base [N]")
        self.turn_line, = self.ax_ctrl.plot([], [], ":", label="turn [N]")
        self._setup_control_plot()

        self.get_logger().info("LOS + SMC + adaptacja uruchomione.")

    def _setup_path_plot(self):
        self.ax_path.set_title("LOS + SMC + adaptacja: trajektoria + okręgi waypointów")
        self.ax_path.set_xlabel("X [m]")
        self.ax_path.set_ylabel("Y [m]")
        self.ax_path.grid(True)
        self.ax_path.axis("equal")

        for i, (wx, wy) in enumerate(self.WAYPOINTS, start=1):
            c = Circle((wx, wy), self.WAYPOINT_RADIUS, fill=False, linewidth=1.8)
            self.ax_path.add_patch(c)
            self.ax_path.text(wx, wy, f"WP{i}", ha="center", va="center", fontsize=9)

        xs = [p[0] for p in self.WAYPOINTS] + [self.START_X]
        ys = [p[1] for p in self.WAYPOINTS] + [self.START_Y]
        margin = 15.0
        self.ax_path.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax_path.set_ylim(min(ys) - margin, max(ys) + margin)
        self.ax_path.legend(loc="best")

    def _setup_control_plot(self):
        self.ax_ctrl.set_title("LOS + SMC + adaptacja: sterowanie w czasie")
        self.ax_ctrl.set_xlabel("t [s]")
        self.ax_ctrl.set_ylabel("ciąg / składowa [N]")
        self.ax_ctrl.grid(True)
        self.ax_ctrl.legend(loc="best")

    def _gps_cb(self, msg):
        now = time.monotonic()

        if self.init_lon is None:
            self.init_lat = msg.latitude
            self.init_lon = msg.longitude
            self.m_per_deg_lon = 111320.0 * math.cos(math.radians(self.init_lat))
            self.state = "RUNNING"
            self.get_logger().info(
                f"GPS zainicjalizowany: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}"
            )
            return

        new_x = self.START_X + (msg.longitude - self.init_lon) * self.m_per_deg_lon
        new_y = self.START_Y + (msg.latitude - self.init_lat) * self.m_per_deg_lat

        if self.prev_x is not None and self.prev_gps_t is not None:
            dt = now - self.prev_gps_t
            if 1e-3 < dt < 2.0:
                raw_speed = math.hypot(new_x - self.prev_x, new_y - self.prev_y) / dt
                self.speed = (
                    self.speed_filter_alpha * raw_speed
                    + (1.0 - self.speed_filter_alpha) * self.speed
                )

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_gps_t = now

        self.x = new_x
        self.y = new_y

    def _imu_cb(self, msg):
        now = time.monotonic()
        q = msg.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )

        sin_p = 2.0 * (q.w * q.y - q.z * q.x)
        sin_p = clamp(sin_p, -1.0, 1.0)
        self.pitch = math.asin(sin_p)

        self.yaw_rate = msg.angular_velocity.z

        if self.prev_imu_t is not None:
            dt = now - self.prev_imu_t
            if 1e-3 < dt < 1.0:
                self.roll_rate = angle_wrap(self.roll - self.prev_roll) / dt
                self.pitch_rate = angle_wrap(self.pitch - self.prev_pitch) / dt

        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.prev_imu_t = now

    def _control_loop(self):
        now = time.monotonic()
        dt = now - self._last_loop_time
        if dt <= 0.0 or dt > 1.0:
            dt = self.CONTROL_DT
        self._last_loop_time = now

        if self.state == "CALIBRATING":
            self._send(0.0, 0.0)
            return

        if self.state == "FINISHED":
            self._send_raw(0.0, 0.0)
            return

        if self.wp_idx >= len(self.WAYPOINTS):
            self._finish()
            return

        tx, ty = self.WAYPOINTS[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist <= self.WAYPOINT_RADIUS:
            if self._wp_inside_since is None:
                self._wp_inside_since = now
            elif now - self._wp_inside_since >= self.WAYPOINT_HOLD_TIME:
                self.wp_idx += 1
                self._wp_inside_since = None
                self.yaw_integral = 0.0
                if self.wp_idx >= len(self.WAYPOINTS):
                    self._finish()
                    return
                else:
                    self.get_logger().info(f"Waypoint osiągnięty -> WP{self.wp_idx + 1}")
        else:
            self._wp_inside_since = None

        tx, ty = self.WAYPOINTS[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        yaw_ref = self._los_yaw_ref(dx, dy, dist)
        yaw_err = angle_wrap(yaw_ref - self.yaw)

        base = self._adaptive_base_thrust(dist, yaw_err)
        turn = self._smc_turn(yaw_err, dt)

        if abs(math.degrees(yaw_err)) > self.WRONG_WAY_DEG:
            base = min(base, self.WRONG_WAY_BASE_LIMIT)

        left_cmd = base - turn
        right_cmd = base + turn

        left_cmd = clamp(left_cmd, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        right_cmd = clamp(right_cmd, self.MAX_REVERSE_THRUST, self.MAX_THRUST)

        left, right = self._send(left_cmd, right_cmd)

        progress = math.cos(yaw_err)

        t = now - self._t0
        self.roll_window.add(t, self.roll)
        self.pitch_window.add(t, self.pitch)

        self.ind.update(
            dt=dt,
            roll=self.roll,
            pitch=self.pitch,
            roll_rate=self.roll_rate,
            pitch_rate=self.pitch_rate,
            left=left,
            right=right,
            yaw_err=yaw_err,
            progress=progress,
        )

        self._append_history(t, left, right, base, turn)
        self._log(t, dist, yaw_ref, yaw_err, base, turn, left, right)

        if now - self._last_print >= self.PRINT_DT:
            self._last_print = now
            self._print_status(dist, yaw_ref, yaw_err, base, turn, left, right)

        if now - self._last_plot >= self.PLOT_DT:
            self._last_plot = now
            self._update_plots()

    def _los_yaw_ref(self, dx, dy, dist):
        if dist < 1e-6:
            return self.yaw

        direct_yaw = math.atan2(dy, dx)

        alpha = clamp(dist / (dist + self.LOS_LOOKAHEAD), 0.35, 1.0)
        err = angle_wrap(direct_yaw - self.yaw)
        return angle_wrap(self.yaw + alpha * err)

    def _adaptive_base_thrust(self, dist, yaw_err):
        roll_rms = self.roll_window.rms()
        pitch_rms = self.pitch_window.rms()

        roll_sev = clamp(roll_rms / self.ROLL_BAD_RAD, 0.0, 1.5)
        pitch_sev = clamp(pitch_rms / self.PITCH_BAD_RAD, 0.0, 1.5)

        roll_factor = clamp(1.0 - self.ROLL_SLOWDOWN_GAIN * roll_sev, 0.20, 1.0)
        pitch_factor = clamp(1.0 - self.PITCH_SLOWDOWN_GAIN * pitch_sev, 0.60, 1.0)

        heading_factor = clamp(
            1.0 - self.HEADING_SLOWDOWN_GAIN * (abs(yaw_err) / math.pi),
            0.20,
            1.0,
        )

        dist_factor = clamp(dist / self.DIST_SLOWDOWN_RADIUS, 0.25, 1.0)

        base = self.BASE_THRUST_MAX * roll_factor * pitch_factor * heading_factor * dist_factor

        if dist < self.DIST_SLOWDOWN_RADIUS:
            base = min(base, self.BASE_THRUST_NEAR_WP + 2.0 * dist)

        return clamp(base, self.BASE_THRUST_MIN, self.BASE_THRUST_MAX)

    def _smc_turn(self, yaw_err, dt):
        self.yaw_integral += yaw_err * dt
        self.yaw_integral = clamp(self.yaw_integral, -self.YAW_INT_LIMIT, self.YAW_INT_LIMIT)

        s = yaw_err + self.SMC_LAMBDA * self.yaw_integral

        turn = (
            self.SMC_K_LINEAR * s
            + self.SMC_K_SWITCH * math.tanh(s / self.SMC_EPS)
            - self.SMC_K_YAW_RATE * self.yaw_rate
        )

        return clamp(turn, -self.TURN_LIMIT, self.TURN_LIMIT)

    def _slew(self, target_l, target_r):
        def step(prev, target):
            delta = clamp(target - prev, -self.MAX_SLEW, self.MAX_SLEW)
            return prev + delta

        new_l = step(self._last_tl, target_l)
        new_r = step(self._last_tr, target_r)

        new_l = clamp(new_l, self.MAX_REVERSE_THRUST, self.MAX_THRUST)
        new_r = clamp(new_r, self.MAX_REVERSE_THRUST, self.MAX_THRUST)

        self._last_tl = new_l
        self._last_tr = new_r
        return new_l, new_r

    def _send(self, left, right):
        left_s, right_s = self._slew(left, right)
        self.pub_l.publish(Float64(data=float(left_s)))
        self.pub_r.publish(Float64(data=float(right_s)))
        return left_s, right_s

    def _send_raw(self, left, right):
        self._last_tl = float(left)
        self._last_tr = float(right)
        self.pub_l.publish(Float64(data=float(left)))
        self.pub_r.publish(Float64(data=float(right)))

    def _append_history(self, t, left, right, base, turn):
        self.hist_t.append(t)
        self.hist_x.append(self.x)
        self.hist_y.append(self.y)
        self.hist_left.append(left)
        self.hist_right.append(right)
        self.hist_base.append(base)
        self.hist_turn.append(turn)
        self.hist_roll_deg.append(math.degrees(self.roll))
        self.hist_pitch_deg.append(math.degrees(self.pitch))
        self.hist_J.append(self.ind.J_total)

    def _log(self, t, dist, yaw_ref, yaw_err, base, turn, left, right):
        self._log_file.write(
            f"{t:.3f},{self.x:.3f},{self.y:.3f},{self.wp_idx + 1},{dist:.3f},"
            f"{math.degrees(self.yaw):.3f},{math.degrees(yaw_ref):.3f},"
            f"{math.degrees(yaw_err):.3f},{math.degrees(self.yaw_rate):.3f},"
            f"{math.degrees(self.roll):.3f},{math.degrees(self.pitch):.3f},"
            f"{math.degrees(self.roll_rate):.3f},{math.degrees(self.pitch_rate):.3f},"
            f"{self.speed:.3f},{base:.3f},{turn:.3f},{left:.3f},{right:.3f},"
            f"{math.degrees(self.roll_window.rms()):.3f},"
            f"{math.degrees(self.pitch_window.rms()):.3f},"
            f"{math.degrees(self.ind.roll_rms_total()):.3f},"
            f"{math.degrees(self.ind.pitch_rms_total()):.3f},"
            f"{self.ind.I_roll:.8f},{self.ind.I_pitch:.8f},"
            f"{self.ind.I_roll_rate:.8f},{self.ind.I_pitch_rate:.8f},"
            f"{self.ind.I_control:.8f},{self.ind.I_no_progress:.8f},"
            f"{self.ind.I_heading:.8f},{self.ind.J_total:.8f},"
            f"{self.state}\n"
        )

    def _print_status(self, dist, yaw_ref, yaw_err, base, turn, left, right):
        print("\033[H\033[J", end="")
        print("══════════════════════════════════════════════════════")
        print(" LOS + SMC + adaptacja prędkości — WAM-V")
        print("══════════════════════════════════════════════════════")
        print(f" Stan:              {self.state}")
        print(f" Waypoint:          {self.wp_idx + 1}/{len(self.WAYPOINTS)}")
        print(f" Pozycja:           ({self.x:.1f}, {self.y:.1f})")
        print(f" Dystans:           {dist:.2f} m, promień WP: {self.WAYPOINT_RADIUS:.1f} m")
        print(f" Yaw:               {math.degrees(self.yaw):.1f} deg")
        print(f" Yaw_ref LOS:       {math.degrees(yaw_ref):.1f} deg")
        print(f" Błąd yaw:          {math.degrees(yaw_err):.1f} deg")
        print(f" v GPS:             {self.speed:.2f} m/s")
        print("──────────────────────────────────────────────────────")
        print(" Bujanie:")
        print(f" Roll/Pitch chwil.: {math.degrees(self.roll):.2f} / {math.degrees(self.pitch):.2f} deg")
        print(f" Roll/Pitch RMS okno: {math.degrees(self.roll_window.rms()):.2f} / "
              f"{math.degrees(self.pitch_window.rms()):.2f} deg")
        print(f" Roll/Pitch RMS misji: {math.degrees(self.ind.roll_rms_total()):.2f} / "
              f"{math.degrees(self.ind.pitch_rms_total()):.2f} deg")
        print("──────────────────────────────────────────────────────")
        print(" Sterowanie:")
        print(f" base:              {base:.1f} N")
        print(f" turn SMC:          {turn:.1f} N")
        print(f" ciąg L/P:          {left:.1f} / {right:.1f} N")
        print("──────────────────────────────────────────────────────")
        print(" Narastająca funkcja jakości:")
        print(" J_total = głównie roll + pomocniczo pitch + sterowanie + postęp")
        print(f" J_total:           {self.ind.J_total:.6f}")
        print(f" I_roll:            {self.ind.I_roll:.6f}    GŁÓWNA KARA")
        print(f" I_pitch:           {self.ind.I_pitch:.6f}   pomocniczo")
        print(f" I_roll_rate:       {self.ind.I_roll_rate:.6f}")
        print(f" I_pitch_rate:      {self.ind.I_pitch_rate:.6f}")
        print(f" I_control:         {self.ind.I_control:.6f}")
        print(f" I_no_progress:     {self.ind.I_no_progress:.6f}")
        print(f" I_heading:         {self.ind.I_heading:.6f}")
        print(f" Czas oceny:        {self.ind.t_eval:.1f} s")
        print(f" Log:               {self.log_path}")
        print("══════════════════════════════════════════════════════")

    def _update_plots(self):
        if self.hist_x:
            self.path_line.set_data(self.hist_x, self.hist_y)
            self.current_point.set_data([self.x], [self.y])
            self.fig_path.canvas.draw()
            self.fig_path.canvas.flush_events()

        if self.hist_t:
            self.left_line.set_data(self.hist_t, self.hist_left)
            self.right_line.set_data(self.hist_t, self.hist_right)
            self.base_line.set_data(self.hist_t, self.hist_base)
            self.turn_line.set_data(self.hist_t, self.hist_turn)
            self.ax_ctrl.relim()
            self.ax_ctrl.autoscale_view()
            self.fig_ctrl.canvas.draw()
            self.fig_ctrl.canvas.flush_events()

        plt.pause(0.001)

    def _finish(self):
        if self._finished_printed:
            return

        self.state = "FINISHED"
        self._finished_printed = True

        self._send_raw(0.0, 0.0)

        print("\033[H\033[J", end="")
        print("FINISHED")
        print("══════════════════════════════════════════════════════")
        print(" Podsumowanie LOS + SMC + adaptacja")
        print(f" J_total:              {self.ind.J_total:.6f}")
        print(f" I_roll:               {self.ind.I_roll:.6f}    GŁÓWNA KARA")
        print(f" I_pitch:              {self.ind.I_pitch:.6f}   pomocniczo")
        print(f" Roll RMS misji:       {math.degrees(self.ind.roll_rms_total()):.3f} deg")
        print(f" Pitch RMS misji:      {math.degrees(self.ind.pitch_rms_total()):.3f} deg")
        print(f" Czas oceny:           {self.ind.t_eval:.2f} s")
        print(f" Log:                  {self.log_path}")
        print("══════════════════════════════════════════════════════")

        self._save_figures()

        rclpy.shutdown()

    def _save_figures(self):
        try:
            if self.hist_x:
                self.path_line.set_data(self.hist_x, self.hist_y)
                self.current_point.set_data([self.x], [self.y])
            self.fig_path.savefig("/tmp/wamv_los_smc_adaptive_path.png", dpi=180, bbox_inches="tight")
            self.fig_ctrl.savefig("/tmp/wamv_los_smc_adaptive_control.png", dpi=180, bbox_inches="tight")
            print(" Zapisano wykresy:")
            print(" /tmp/wamv_los_smc_adaptive_path.png")
            print(" /tmp/wamv_los_smc_adaptive_control.png")
        except Exception as exc:
            print(f" Nie udało się zapisać wykresów: {exc}")

    def destroy_node(self):
        try:
            self._log_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LOSSMCAdaptiveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nPrzerwano Ctrl+C")
    finally:
        try:
            node._send_raw(0.0, 0.0)
            node._save_figures()
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.try_shutdown()

        plt.ioff()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3
import math
import time
from std_srvs.srv import Trigger
from std_msgs.msg import Int64, Int32        
#harus tuning ulang speed dengan keseimbangan sumbu x agar lebih stabil
class DualLegWalker(Node):
    def __init__(self):
        super().__init__('dual_leg_walker')
        
        # ================= CONFIGURATION =================
        self.speed = 10.0   # Kecepatan Jalan
        self.step_height = 0.07  # Tinggi angkat kaki (5 cm)
        self.walking_state = 0   
        
        # --- KAKI KANAN (RIGHT) ---
        self.topic_right = '/target_pose/right_leg'
        self.base_right = {'x': -0.03, 'y': -0.0, 'z': 0.025}
        
        # --- KAKI KIRI (LEFT) ---
        self.topic_left = '/target_pose/left_leg'
        self.base_left = {'x': 0.03, 'y': -0.0, 'z': 0.025}
        
        # Orientasi (Default Lurus)
        self.default_quat = {'x': -0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        # =================================================
        
        # Publishers
        self.pub_right = self.create_publisher(Pose, self.topic_right, 10)
        self.pub_left = self.create_publisher(Pose, self.topic_left, 10)

        # === [BARU] Subscribers untuk IMU & Sensor Kontak ===
        self.imu_sub = self.create_subscription(Vector3, '/imu', self.imu_callback, 10)
        self.contact_r_sub = self.create_subscription(Int32, '/foot_contact/right', self.contact_r_callback, 10)
        self.contact_l_sub = self.create_subscription(Int32, '/foot_contact/left', self.contact_l_callback, 10)
        
        # === [BARU] Variabel State Sensor ===
        self.imu_pitch_rad = 0.0
        self.imu_roll_rad = 0.0
        
        self.contact_right = 0
        self.contact_left = 0
        
        # Variabel untuk "nge-hold" orientasi terakhir sebelum napak (Poin 2)
        self.hold_pitch_r = 0.0
        self.hold_roll_r = 0.0
        self.hold_pitch_l = 0.0
        self.hold_roll_l = 0.0
        # ====================================================

        # === [BARU] KONFIGURASI PD CONTROLLER & FILTER ===
        self.target_pitch = math.radians(0.0)  # Target Pitch (Nunduk Maju/Mundur)
        self.target_roll = math.radians(0.0)  # Target Roll (Miring Kiri/Kanan)
        
        # Gain PID (Mulai dari angka kecil dulu untuk tuning awal)
        self.kp_pitch = 0.3
        self.kd_pitch = 0.05
        self.kp_roll = 0.8
        self.kd_roll = 0.05
        
        # Variabel State PD
        self.prev_error_pitch = 0.0
        self.prev_error_roll = 0.0
        
        # Filter EMA & Deadband
        self.imu_pitch_filtered = 0.0
        self.imu_roll_filtered = 0.0
        self.alpha_ema = 0.2  # 0.0 (Kebal noise, tapi super delay) - 1.0 (Responsif, tapi full noise)
        
        self.deadband_rad = math.radians(1.0) # Abaikan error di bawah 1 derajat
        self.pid_limit_rad = math.radians(40.0) # Hard limit kompensasi
        
        self.z_lock_r = 0.0
        self.z_lock_l = 0.0
        self.is_z_interrupted_r = False
        self.is_z_interrupted_l = False
        self.prev_z_target_r = 0.0
        self.prev_z_target_l = 0.0
        # =================================================

        # === [BARU] KONFIGURASI FOOT PLACEMENT PD & LIMITS ===
        self.kp_step_pitch = 0.15
        self.kd_step_pitch = 0.02
        self.kp_step_roll = 0.15
        self.kd_step_roll = 0.02
        
        self.prev_step_err_pitch = 0.0
        self.prev_step_err_roll = 0.0
        
        # Variabel penyimpan offset langkah (Current State)
        self.step_offset_x_r = 0.0
        self.step_offset_y_r = 0.0
        self.step_offset_x_l = 0.0
        self.step_offset_y_l = 0.0
        
        # Limit Kompensasi Langkah (dalam meter)
        self.limit_step_y = 0.045       # 7 cm depan/belakang
        self.limit_step_x_out = 0.05   # 5 cm ke arah luar
        self.limit_step_x_in = 0.025    # 2.5 cm ke arah dalam (selangkangan)
        
        # Alpha untuk Smooth Decay saat Take-off (0.1 = cukup halus)
        self.alpha_step = 0.1
        
        # === [BARU] KONFIGURASI STOMP REFLEX (EMERGENCY DROP) ===
        self.stomp_threshold = math.radians(20.0)  # Batas darurat 20 derajat (Pitch / Roll)
        self.stomp_drop_rate = 0.01  # Turun 1 cm per tick (~0.08 detik dari puncak ke tanah)
        
        self.is_stomping_r = False
        self.is_stomping_l = False
        self.current_stomp_z_r = 0.0
        self.current_stomp_z_l = 0.0

        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.t = 0
        self.dt = 0.02
        
        self.get_logger().info("🚶 DUAL LEG WALKER STARTED")
        self.get_logger().info(f"   Speed: {self.speed} | Height: {self.step_height}m")

        # Subscription
        self.speed_pub = self.create_subscription(Int64, '/speed_step', self.callback_change_speed_step, 10)
        self.step_height_pub = self.create_subscription(Int64, '/step_height', self.callback_change_step_height, 10)
        self.startwalking = self.create_subscription(Int64, '/start_walking', self.callback_start_walking, 10)
        self.settargetpitch = self.create_subscription(Int64, '/target_pitch', self.callback_set_target_pitch, 10)

        self.reset_client = self.create_client(Trigger, '/reset_ik_memory')
        self.reset_ik_node()

        time.sleep(0.5)

    # === [BARU] Fungsi Callbacks Sensor ===
    def imu_callback(self, msg):
        # msg.x = Pitch, msg.y = Roll (dalam Derajat). Langsung ubah ke Radian.
        self.imu_pitch_rad = math.radians(msg.x)
        self.imu_roll_rad = math.radians(msg.y)

    def contact_r_callback(self, msg):
        self.contact_right = msg.data

    def contact_l_callback(self, msg):
        self.contact_left = msg.data
    # ======================================

    def callback_start_walking(self, msg):
        self.walking_state = msg.data
        if self.walking_state == 1:
            self.get_logger().info("Starting Walking...")
        else:
            self.get_logger().info("Stopping Walking...")

    def callback_change_speed_step(self, msg):
        self.speed = msg.data
        self.get_logger().info(f"Step Speed Updated to: {self.speed}")

    def callback_change_step_height(self, msg):
        self.step_height = msg.data * 0.01
        self.get_logger().info(f"Step Height Updated to: {self.step_height}")
    
    def callback_set_target_pitch(self, msg):
        self.target_pitch = math.radians(msg.data)
        self.get_logger().info(f"Target Pitch Updated to: {msg.data} degrees")

    def timer_callback(self):
        self.t += self.dt
        elapsed = self.t

        self.step_height2 = 0.04
        sway_amp = 0.035

        phase = self.speed * self.t - 0.5
        sway_phase = phase - 0.8


        # =================================================================
        z_target_r = self.cycloid_z(phase, self.step_height2, swing_ratio=0.85)
        z_target_l = self.cycloid_z(phase + math.pi, self.step_height2, swing_ratio=0.85)

        # === [BARU] LOGIKA STOMP REFLEX (EMERGENCY DROP) ===
        # Cek apakah Pitch ATAU Roll melewati batas darurat (20 derajat)
        # Kita pakai imu_pitch_filtered dari tick sebelumnya, sangat aman.
        is_emergency = (abs(self.imu_pitch_filtered) > self.stomp_threshold) or (abs(self.imu_roll_filtered) > self.stomp_threshold)

        # -- KAKI KANAN STOMP --
        # Syarat: Kondisi darurat, kaki sedang di udara (0), dan belum napak tanah
        if is_emergency and self.contact_right == 0 and z_target_r > 0:
            if not self.is_stomping_r:
                self.is_stomping_r = True
                self.current_stomp_z_r = z_target_r  # Tangkap ketinggian Z saat ini
            
            # Paksa turun 1 cm tiap tick
            self.current_stomp_z_r -= self.stomp_drop_rate
            z_target_r = max(0.0, self.current_stomp_z_r) # Mentok di 0, jangan tembus lantai
        else:
            self.is_stomping_r = False

        # -- KAKI KIRI STOMP --
        if is_emergency and self.contact_left == 0 and z_target_l > 0:
            if not self.is_stomping_l:
                self.is_stomping_l = True
                self.current_stomp_z_l = z_target_l
            
            self.current_stomp_z_l -= self.stomp_drop_rate
            z_target_l = max(0.0, self.current_stomp_z_l)
        else:
            self.is_stomping_l = False
        # ===================================================

        is_moving_down_r = (z_target_r < self.prev_z_target_r)
        is_moving_down_l = (z_target_l < self.prev_z_target_l)

        if self.contact_right == 1 and z_target_r > 0 and is_moving_down_r:
            if not self.is_z_interrupted_r:
                self.z_lock_r = z_target_r
                self.is_z_interrupted_r = True
            z_final_r = self.z_lock_r
        else:
            if z_target_r == 0 or self.contact_right == 0:
                self.is_z_interrupted_r = False
            z_final_r = z_target_r


        if self.contact_left == 1 and z_target_l > 0 and is_moving_down_l:
            if not self.is_z_interrupted_l:
                self.z_lock_l = z_target_l
                self.is_z_interrupted_l = True
            z_final_l = self.z_lock_l
        else:
            if z_target_l == 0 or self.contact_left == 0:
                self.is_z_interrupted_l = False
            z_final_l = z_target_l

        self.prev_z_target_r = z_target_r
        self.prev_z_target_l = z_target_l

        raw_x_left = (0.0125 * math.sin(self.speed * elapsed - 0.5) * 3 - 0.02) *-1
        x_left = self.smooth_clamp(raw_x_left, -0.025, 0.01)

        raw_x_right = (0.0125 * math.sin(self.speed * elapsed - 0.5) * 3 + 0.02) * -1
        x_right = self.smooth_clamp(raw_x_right, -0.01, 0.025)

        y_right = math.sin(self.speed * elapsed - 1.57 - 0.5) * 0.01
        y_left = math.sin(self.speed * elapsed - 1.57 + 3.14 - 0.5) * 0.01

        # =================================================================
        # === [BARU] 1. FILTERING IMU (EMA) ===
        self.imu_pitch_filtered = (self.alpha_ema * self.imu_pitch_rad) + ((1 - self.alpha_ema) * self.imu_pitch_filtered)
        self.imu_roll_filtered = (self.alpha_ema * self.imu_roll_rad) + ((1 - self.alpha_ema) * self.imu_roll_filtered)

        # === [BARU] 2. KALKULASI ERROR & DEADBAND ===
        error_pitch = self.target_pitch - self.imu_pitch_filtered
        if abs(error_pitch) < self.deadband_rad:
            error_pitch = 0.0
            
        error_roll = self.target_roll - self.imu_roll_filtered
        if abs(error_roll) < self.deadband_rad:
            error_roll = 0.0

        # === [BARU] 3. PD CONTROLLER KINEMATICS ===
        # Pitch
        p_pitch = self.kp_pitch * error_pitch
        d_pitch = self.kd_pitch * (error_pitch - self.prev_error_pitch) / self.dt
        out_pitch = p_pitch + d_pitch
        self.prev_error_pitch = error_pitch
        
        # Roll
        p_roll = self.kp_roll * error_roll
        d_roll = self.kd_roll * (error_roll - self.prev_error_roll) / self.dt
        out_roll = p_roll + d_roll
        self.prev_error_roll = error_roll

        # CLAMPING LIMITER (Maksimal 20 Derajat)
        out_pitch = max(-self.pid_limit_rad, min(out_pitch, self.pid_limit_rad))
        out_roll = max(-self.pid_limit_rad, min(out_roll, self.pid_limit_rad))

        # === [BARU] 4. DISTRIBUSI DSP CERDAS (BERDASARKAN KAKI AKTIF) ===
        pd_pitch_r, pd_pitch_l = 0.0, 0.0
        pd_roll_r, pd_roll_l = 0.0, 0.0
        
        # Kalkulasi Posisi Y Relatif (Maju-Mundur) saat ini
        # Mengambil dari logic di bawah: Y = base - y_wave
        y_curr_r = self.base_right['y'] - y_right if self.walking_state == 1 else self.base_right['y']
        y_curr_l = self.base_left['y'] - y_left if self.walking_state == 1 else self.base_left['y']

        # -- Distribusi Roll (X Axis) --
        if error_roll > 0: # Miring Kiri -> Output ke Kaki Kiri
            pd_roll_l = out_roll
        elif error_roll < 0: # Miring Kanan -> Output ke Kaki Kanan
            pd_roll_r = out_roll

        # -- Distribusi Pitch (Y Axis) --
        if abs(y_curr_r - y_curr_l) <= 0.01:
            # Jika kaki sejajar (selisih <= 1 cm), bagi rata 50:50
            pd_pitch_r = out_pitch * 0.5
            pd_pitch_l = out_pitch * 0.5
        else:
            if error_pitch > 0: # Nunduk Depan -> Lempar ke kaki terdepan (Y Terbesar)
                if y_curr_r > y_curr_l: pd_pitch_r = out_pitch
                else: pd_pitch_l = out_pitch
            elif error_pitch < 0: # Nunduk Belakang -> Lempar ke kaki terbelakang (Y Terkecil)
                if y_curr_r < y_curr_l: pd_pitch_r = out_pitch
                else: pd_pitch_l = out_pitch

        # === [BARU] 5. INTEGRASI FASE NAPAK DAN MELAYANG ===
        tilt_limit_rad = math.radians(20.0) 
        is_safe = (abs(self.imu_pitch_filtered) <= tilt_limit_rad) and (abs(self.imu_roll_filtered) <= tilt_limit_rad)

        final_pitch_r, final_roll_r = 0.0, 0.0
        final_pitch_l, final_roll_l = 0.0, 0.0

        # KAKI KANAN
        if self.contact_right == 0:
            # Fase Melayang (Gimbal Mode)
            if is_safe:
                self.hold_pitch_r = -self.imu_pitch_filtered
                self.hold_roll_r = -self.imu_roll_filtered
            else:
                self.hold_pitch_r, self.hold_roll_r = 0.0, 0.0
            # Hasil final adalah murni hold gimbal
            final_pitch_r = self.hold_pitch_r
            final_roll_r = self.hold_roll_r
        else:
            # Fase Napak (Kunci hold terakhir + Tambahkan Kompensasi PD)
            final_pitch_r = (self.hold_pitch_r + pd_pitch_r) * -1
            final_roll_r = (self.hold_roll_r + pd_roll_r) * -1

        # KAKI KIRI
        if self.contact_left == 0:
            if is_safe:
                self.hold_pitch_l = -self.imu_pitch_filtered
                self.hold_roll_l = -self.imu_roll_filtered
            else:
                self.hold_pitch_l, self.hold_roll_l = 0.0, 0.0
            final_pitch_l = self.hold_pitch_l
            final_roll_l = self.hold_roll_l
        else:
            final_pitch_l = (self.hold_pitch_l + pd_pitch_l) * -1
            final_roll_l = (self.hold_roll_l + pd_roll_l) * -1

        # Konversi ke Quaternion menggunakan konvensi Darput (X=Pitch, Y=Roll)
        quat_r = self.euler_to_quaternion(final_pitch_r, final_roll_r, 0.0) 
        quat_l = self.euler_to_quaternion(final_pitch_l, final_roll_l, 0.0)
        # =================================================================

        # =================================================================
        # === [BARU] 1. KALKULASI PD UNTUK FOOT PLACEMENT ===
        # Pitch -> Maju Mundur (Y)
        out_step_y = (self.kp_step_pitch * error_pitch) + \
                     (self.kd_step_pitch * (error_pitch - self.prev_step_err_pitch) / self.dt) * -1
        self.prev_step_err_pitch = error_pitch
        
        # Roll -> Kanan Kiri (X)
        out_step_x = (self.kp_step_roll * error_roll) + \
                     (self.kd_step_roll * (error_roll - self.prev_step_err_roll) / self.dt)
        self.prev_step_err_roll = error_roll

        # === [BARU] 2. LOGIKA LATCH, ASYMMETRIC CLAMP, & SMOOTH DECAY ===
        
        # --- KAKI KANAN (RIGHT) ---
        if self.contact_right == 0:  # FASE SWING (MELAYANG)
            # Target Maju-Mundur (Y)
            target_y_r = max(-self.limit_step_y, min(out_step_y, self.limit_step_y))
            
            # Target Kanan-Kiri (X) - Kanan Outward adalah X negatif (-), Inward X positif (+)
            # Limit: Outward -0.05, Inward +0.01
            target_x_r = max(-self.limit_step_x_out, min(out_step_x, self.limit_step_x_in))
            
            # Apply Smooth Decay / EMA Transition
            self.step_offset_y_r = (self.alpha_step * target_y_r) + (1 - self.alpha_step) * self.step_offset_y_r
            self.step_offset_x_r = (self.alpha_step * target_x_r) + (1 - self.alpha_step) * self.step_offset_x_r
        else:
            # FASE STANCE (NAPAK): LATCH / HOLD nilai terakhir, jangan diubah
            pass

        # --- KAKI KIRI (LEFT) ---
        if self.contact_left == 0:  # FASE SWING (MELAYANG)
            target_y_l = max(-self.limit_step_y, min(out_step_y, self.limit_step_y))
            
            # Target Kanan-Kiri (X) - Kiri Outward adalah X positif (+), Inward X negatif (-)
            # Limit: Inward -0.01, Outward +0.05
            target_x_l = max(-self.limit_step_x_in, min(out_step_x, self.limit_step_x_out))
            
            self.step_offset_y_l = (self.alpha_step * target_y_l) + (1 - self.alpha_step) * self.step_offset_y_l
            self.step_offset_x_l = (self.alpha_step * target_x_l) + (1 - self.alpha_step) * self.step_offset_x_l
        else:
            # FASE STANCE (NAPAK): LATCH / HOLD
            pass


        # === PUBLISH KANAN ===
        # --- PUBLISH KANAN ---
        msg_r = Pose()
        # Tambahkan kompensasi langkah ke X dan Y
        msg_r.position.x = self.base_right['x'] + x_right + self.step_offset_x_r
        
        if self.walking_state == 1:
            msg_r.position.y = (self.base_right['y'] - y_right) + self.step_offset_y_r
        else:
            msg_r.position.y = self.base_right['y'] + self.step_offset_y_r
            
        msg_r.position.z = self.base_right['z'] + z_final_r 
        
        msg_r.orientation.w = quat_r['w']
        msg_r.orientation.x = quat_r['x']
        msg_r.orientation.y = quat_r['y']
        msg_r.orientation.z = quat_r['z']
        self.pub_right.publish(msg_r)
        
        # --- PUBLISH KIRI ---
        msg_l = Pose()
        msg_l.position.x = self.base_left['x'] + x_left + self.step_offset_x_l
        
        if self.walking_state == 1:
            msg_l.position.y = (self.base_left['y'] - y_left) + self.step_offset_y_l
        else:
            msg_l.position.y = self.base_left['y'] + self.step_offset_y_l
            
        msg_l.position.z = self.base_left['z'] + z_final_l

        print(f"step_offset_y_l: {self.step_offset_y_l:.4f} | step_offset_y_r: {self.step_offset_y_r:.4f}")
        
        msg_l.orientation.w = quat_l['w']
        msg_l.orientation.x = quat_l['x']
        msg_l.orientation.y = quat_l['y']
        msg_l.orientation.z = quat_l['z']
        self.pub_left.publish(msg_l)


    def reset_ik_node(self):
        self.get_logger().info("🔄 Requesting IK Memory Reset...")
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Reset Service not available!")
            return
        req = Trigger.Request()
        future = self.reset_client.call_async(req)

    def set_ik_speed(self, nanosecond):
        msg = Int64()
        msg.data = int(nanosecond)
        self.speed_pub.publish(msg)
        self.get_logger().info(f"🐢 Setting IK Speed to {nanosecond} nanoseconds...")
        time.sleep(0.1)

    def cycloid_z(self, phase, step_height, swing_ratio=0.8):
        phase = phase % (2 * math.pi)
        swing_duration = math.pi * swing_ratio
        if phase > swing_duration:
            return 0.0
        theta = phase * (math.pi / swing_duration) 
        z_normalized = (1 - math.cos(theta)) / 2
        if theta > 2.5:  
            landing_progress = (theta - 2.5) / (math.pi - 2.5)
            smoothing = math.exp(-3 * landing_progress)
            z_normalized *= smoothing
        return step_height * z_normalized
    
    def smooth_clamp(self, x, lo, hi):
        if x <= lo: return lo
        if x >= hi: return hi
        t = (x - lo) / (hi - lo)
        smooth_t = t * t * (3.0 - 2.0 * t)
        return lo + (smooth_t * (hi - lo))

    # === [BARU] Fungsi Matematika Murni Euler to Quaternion ===
    def euler_to_quaternion(self, x_angle, y_angle, z_angle):
        """
        Konversi rotasi sumbu X, Y, Z (dalam radian) menjadi Quaternion.
        Sumbu X = Pitch, Sumbu Y = Roll, Sumbu Z = Yaw (Konvensi Darput).
        """
        qx = math.sin(x_angle/2) * math.cos(y_angle/2) * math.cos(z_angle/2) - math.cos(x_angle/2) * math.sin(y_angle/2) * math.sin(z_angle/2)
        qy = math.cos(x_angle/2) * math.sin(y_angle/2) * math.cos(z_angle/2) + math.sin(x_angle/2) * math.cos(y_angle/2) * math.sin(z_angle/2)
        qz = math.cos(x_angle/2) * math.cos(y_angle/2) * math.sin(z_angle/2) - math.sin(x_angle/2) * math.sin(y_angle/2) * math.cos(z_angle/2)
        qw = math.cos(x_angle/2) * math.cos(y_angle/2) * math.cos(z_angle/2) + math.sin(x_angle/2) * math.sin(y_angle/2) * math.sin(z_angle/2)
        
        return {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    # ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = DualLegWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
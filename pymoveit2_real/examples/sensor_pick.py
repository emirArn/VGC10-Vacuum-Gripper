#!/usr/bin/env python3

import sys
import time
import math
import subprocess
from threading import Thread, Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  

# Mesaj Tipleri
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan

# pymoveit2_real kütüphanesi importları
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot

# Lidar ve temas sinyalleri için kilit mekanizmaları
BOX_DETECTED_EVENT = Event()        # 1. Lidar (Başlangıç)
END_BOX_DETECTED_EVENT = Event()    # 2. Lidar (Bitiş/WP3 Kesici)
CONTACT_DETECTED_EVENT = Event()    # 3. Temas sensörü (Bağımsız executor'da çalışır)

# ── Box Spawn Ayarları ──
BOX_SPAWN_POSITION = [0.6490, -0.3422, 0.8748]

# Box SDF şablonu - {box_name} runtime'da değiştirilir
# NOT: Kutu modeli değişecekse bu şablonu güncelleyin
BOX_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{box_name}">
    <static>false</static>
    <link name="{box_name}_link">
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000065</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.000171</iyy>
          <iyz>0.0</iyz>
          <izz>0.000221</izz>
        </inertia>
      </inertial>
      <collision name="{box_name}_collision">
        <geometry>
          <box>
            <size>0.14 0.083 0.03</size>
          </box>
        </geometry>
      </collision>
      <visual name="{box_name}_visual">
        <geometry>
          <box>
            <size>0.14 0.083 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1</ambient>
          <diffuse>1.0 0.0 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

# ====================================================
# DÜĞÜM 1: LİDARLARI VE TEMAS SENSÖRÜNÜ DİNLEYEN 
#           BAĞIMSIZ SİSTEM (Kendi executor'unda çalışır)
# ====================================================
class LidarMonitorNode(Node):
    def __init__(self):
        super().__init__("lidar_monitor_node")
        self.get_logger().info("📡 Lidar & Contact Monitor ayrı bir düğüm olarak başlatıldı!")
        
        # 1. LIDAR (Başlangıç Tarayıcısı)
        self.sub_start = self.create_subscription(
            LaserScan,
            '/sensor_start_scan',
            self.lidar_start_callback,
            qos_profile_sensor_data
        )
        self.debug_start_count = 0
        self.monitoring_start_active = False 

        # 2. LIDAR (Bitiş Tarayıcısı - WP3 İptal Edici)
        self.sub_end = self.create_subscription(
            LaserScan,
            '/sensor_end_scan',
            self.lidar_end_callback,
            qos_profile_sensor_data
        )
        self.debug_end_count = 0
        self.monitoring_end_active = False 

        # 3. TEMAS SENSÖRÜ (MoveIt2'nin spin_once'ından bağımsız çalışır!)
        self.contact_sub = self.create_subscription(
            Contacts,
            '/world/cem/model/whole_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact',
            self.contact_callback,
            qos_profile_sensor_data
        )
        self.monitoring_contact_active = False
        self._debug_contact_count = 0

    # --- 1. LIDAR FONKSİYONU ---
    def lidar_start_callback(self, msg):
        if len(msg.ranges) > 0:
            current_range = msg.ranges[0]
            self.debug_start_count += 1
            
            if self.debug_start_count % 15 == 0:
                self.get_logger().info(f"🟢 [LIDAR 1] Mesafe: {current_range:.5f} | Aktif mi?: {self.monitoring_start_active}")

            if math.isnan(current_range) or math.isinf(current_range):
                return
            
            if self.monitoring_start_active and current_range < 0.250:
                self.get_logger().info(f"📏 [LIDAR 1] Kutu Algılandı! ({current_range:.5f}m)")
                self.monitoring_start_active = False 
                BOX_DETECTED_EVENT.set() 

    # --- 2. LIDAR FONKSİYONU ---
    def lidar_end_callback(self, msg):
        if len(msg.ranges) > 0:
            current_range = msg.ranges[0]
            self.debug_end_count += 1
            
            if self.debug_end_count % 15 == 0:
                self.get_logger().info(f"🔵 [LIDAR 2] Mesafe: {current_range:.5f} | Aktif mi?: {self.monitoring_end_active}")

            if math.isnan(current_range) or math.isinf(current_range):
                return
            
            if self.monitoring_end_active and current_range < 0.250:
                self.get_logger().info(f"🚨 [LIDAR 2] Kutu WP3 yolunda algılandı! ({current_range:.5f}m) İptal tetiği çekiliyor...")
                self.monitoring_end_active = False 
                END_BOX_DETECTED_EVENT.set() 

    # --- 3. TEMAS SENSÖRÜ FONKSİYONU ---
    def contact_callback(self, msg):
        self._debug_contact_count += 1
        if self._debug_contact_count % 50 == 1:
            self.get_logger().info(
                f"🔍 [CONTACT CB #{self._debug_contact_count}] "
                f"aktif={self.monitoring_contact_active}, "
                f"contacts_len={len(msg.contacts)}"
            )

        if not self.monitoring_contact_active:
            return

        if len(msg.contacts) > 0:
            self.get_logger().info(f"📡 TEMAS ALGILANDI! (contacts={len(msg.contacts)})")
            self.monitoring_contact_active = False
            CONTACT_DETECTED_EVENT.set()

# ====================================================
# DÜĞÜM 2: SADECE ROBOTU YÖNETEN SİSTEM
# ====================================================
class SmartPickSequence(Node):
    def __init__(self, lidar_node):
        super().__init__("smart_pick_sequence_node")
        self.lidar_node = lidar_node 
        self.state = "INIT" 
        self.box_counter = 0  # box0, box1, box2, ...
        
        cbg = ReentrantCallbackGroup()
        self.arm = MoveIt2_Real(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        
        self.arm.planner_id = "RRTConnectkConfigDefault"
        self.arm.max_velocity = 0.08
        self.arm.max_acceleration = 0.08

        self.vacuum_pub = self.create_publisher(Bool, '/suction/enable', 10)

        # ── Ortak Waypoint'ler (WP1 ~ WP4) ──
        self.wp1 = [0.11156, -0.87965, -0.98520, 1.07003, 1.18878, 0.56172, 0.34306]
        self.wp2 = [0.11156, -0.92922, -0.71436, 0.87503, 1.69312, 0.54695, -0.28651]
        self.wp3 = [0.98461, -0.91358, -0.86834, 1.02667, 1.18878, 0.55418, 0.26389]
        self.wp4 = [0.89208, -0.84949, -1.25664, 0.73011, 1.18878, 0.84949, 1.12092]

        # ── 4 Toolkit Waypoint'i (box_counter % 4 ile cyclic seçim) ──
        self.toolkit_waypoints = [
            # Toolkit 1 (Ray: 0.89208)
            [0.89208, -0.37322, -0.85326, 0.42474, 1.27172, 0.71377, 0.84949],
            # Toolkit 2 (Ray: 1.06669)
            [1.06669, -0.37322, -0.85326, 0.42474, 1.27172, 0.71377, 0.84949],
            # Toolkit 3 (Ray: 1.27208)
            [1.27208, -0.37322, -0.85326, 0.42474, 1.27172, 0.71377, 0.84949],
            # Toolkit 4 (Ray: 1.47918)
            [1.47918, -0.37322, -0.85326, 0.42474, 1.27172, 0.71377, 0.84949],
        ]

        self.get_logger().info("🟢 Robot Kontrolcüsü hazır! (4 Toolkit Cyclic Mod)")

    def spawn_box(self):
        """Gazebo'da yeni bir box spawn eder (box0, box1, box2, ...)."""
        box_name = f"box{self.box_counter}"
        x, y, z = BOX_SPAWN_POSITION
        self.get_logger().info(f"📦 {box_name} spawn ediliyor → ({x}, {y}, {z})")
        
        sdf_str = BOX_SDF_TEMPLATE.format(box_name=box_name)
        
        try:
            result = subprocess.run(
                [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-string', sdf_str,
                    '-name', box_name,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                ],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                self.get_logger().info(f"✅ {box_name} başarıyla spawn edildi!")
            else:
                self.get_logger().error(f"❌ {box_name} spawn hatası: {result.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"❌ {box_name} spawn zaman aşımı!")
        except Exception as e:
            self.get_logger().error(f"❌ {box_name} spawn exception: {e}")

    def force_cancel_motion(self):
        """Aktif trajectory execution'ı zorla ve garantili iptal eder."""
        self.get_logger().info("🛑 Hareket zorla iptal ediliyor...")

        self.arm.cancel_execution()

        try:
            goal_handle = self.arm._MoveIt2__execution_goal_handle
            if goal_handle is not None:
                self.get_logger().info("   Action goal handle cancel ediliyor...")
                cancel_future = goal_handle.cancel_goal_async()
                # Diğer koddaki bekleme (while) mantığı burada:
                timeout = time.time() + 2.0
                while not cancel_future.done() and time.time() < timeout:
                    time.sleep(0.01)
                self.get_logger().info("   Action goal cancel isteği gönderildi.")
        except AttributeError:
            self.get_logger().warn("   Goal handle'a erişilemedi (name mangling).")

        try:
            self.arm._MoveIt2__is_motion_requested = False
            self.arm._MoveIt2__is_executing = False
            self.arm._MoveIt2__execution_goal_handle = None
            self.arm.motion_suceeded = False
        except AttributeError:
            pass

        self.get_logger().info("✅ Hareket iptal edildi.")

    def set_vacuum(self, state: bool):
        suction_msg = Bool()
        suction_msg.data = state
        for _ in range(5):
            self.vacuum_pub.publish(suction_msg)
            time.sleep(0.05)
        self.get_logger().info(f"🔧 Vakum {'AÇIK' if state else 'KAPALI'}")

    def run_mission(self):
        time.sleep(1.0) 

        while True:
            box_name = f"box{self.box_counter}"
            toolkit_index = self.box_counter % 4

            self.get_logger().info(f"\n{'='*55}")
            self.get_logger().info(f"🔄 DÖNGÜ #{self.box_counter}: {box_name} → Toolkit {toolkit_index + 1}")
            self.get_logger().info(f"{'='*55}")

            # ================== AŞAMA 1: WP1 + BOX SPAWN ==================
            self.get_logger().info("➡️ AŞAMA 1: WP1'e gidiliyor...")
            self.arm.move_to_configuration(joint_positions=self.wp1)
            self.arm.wait_until_executed()
            
            # WP1'e ulaştık → Box spawn et
            self.spawn_box()
            time.sleep(0.5)  # Gazebo'da oluşmasını bekle
            
            self.get_logger().info("⏳ WP1'e ulaşıldı. Kutu (< 0.250m) bekleniyor...")
            BOX_DETECTED_EVENT.clear()
            self.lidar_node.monitoring_start_active = True
            
            BOX_DETECTED_EVENT.wait()
            
            # ================== AŞAMA 2: WP2 PICK (TEMAS İLE) ==================
            self.get_logger().info("🚀 Lidar'dan Sinyal alındı! WP2'ye iniliyor!")
            self.get_logger().info("➡️ AŞAMA 2: WP2'ye iniliyor. Temas dinleme AKTİF!")
            
            self.arm.max_velocity = 0.07
            self.arm.max_acceleration = 0.07
            
            CONTACT_DETECTED_EVENT.clear()
            self.lidar_node.monitoring_contact_active = True

            contact_detected_flag = [False]

            def contact_interrupt_monitor():
                if CONTACT_DETECTED_EVENT.wait():
                    self.get_logger().info("💥 TEMAS EVENT'İ TETİKLENDİ! Hareket kesiliyor...")
                    self.force_cancel_motion()
                    
                    suction_msg = Bool()
                    suction_msg.data = True
                    for _ in range(3):
                        self.vacuum_pub.publish(suction_msg)
                    self.get_logger().info("✅ Vakum AÇILDI (temas anında)")
                    contact_detected_flag[0] = True

            contact_thread = Thread(target=contact_interrupt_monitor, daemon=True)
            contact_thread.start()

            self.arm.move_to_configuration(joint_positions=self.wp2)
            self.arm.wait_until_executed()

            if not CONTACT_DETECTED_EVENT.is_set():
                self.get_logger().info("⏳ Hareket bitti, temas event'i bekleniyor (max 1.5s)...")
                CONTACT_DETECTED_EVENT.wait(timeout=1.5)

            self.lidar_node.monitoring_contact_active = False

            if contact_detected_flag[0] or CONTACT_DETECTED_EVENT.is_set():
                self.get_logger().info("💥 Temas başarıyla yakalandı ve duruldu!")
                self.get_logger().info("⚡ Vakum yapıştı! Yukarı çıkılıyor...")
                time.sleep(0.1)
            else:
                self.get_logger().warn("⚠️ Temas algılanmadı, robot doğrudan WP2 noktasına kadar indi!")
                self.set_vacuum(True)
                time.sleep(0.1)

            self.arm.max_velocity = 0.04
            self.arm.max_acceleration = 0.04
            time.sleep(0.2)

            # ================== AŞAMA 3: WP3 (LİDAR 2 İPTAL KORUMASI) ==================
            self.get_logger().info("➡️ AŞAMA 3: WP3'e gidiliyor (Lidar 2 İptal Koruması AKTİF)...")
            
            self.state = "MOVING_TO_WP3"
            END_BOX_DETECTED_EVENT.clear()
            self.lidar_node.monitoring_end_active = True

            def interrupt_monitor():
                if END_BOX_DETECTED_EVENT.wait(): 
                    if self.state == "MOVING_TO_WP3":
                        self.get_logger().info("🛑 HAVADA MÜDAHALE: WP3 iptal ediliyor!")
                        self.force_cancel_motion()

            monitor_thread = Thread(target=interrupt_monitor, daemon=True)
            monitor_thread.start()

            self.arm.move_to_configuration(joint_positions=self.wp3)
            success = self.arm.wait_until_executed()
            
            self.state = "DONE_WP3"
            self.lidar_node.monitoring_end_active = False 

            if not success:
                if END_BOX_DETECTED_EVENT.is_set():
                    self.get_logger().info("⏭️ WP3 yarıda kesildi! Direkt WP4'e atlanıyor.")
                else:
                    self.get_logger().error(f"❌ WP3 başarısız! {box_name} atlanıyor.")
                    self.set_vacuum(False)
                    self.box_counter += 1
                    continue
            else:
                self.get_logger().info("✅ WP3 sorunsuz tamamlandı.")

            time.sleep(0.5)

            # ================== AŞAMA 4: WP4 ==================
            self.get_logger().info("➡️ AŞAMA 4: WP4'e gidiliyor...")
            self.arm.move_to_configuration(joint_positions=self.wp4)
            if not self.arm.wait_until_executed():
                self.get_logger().error(f"❌ WP4 başarısız! {box_name} atlanıyor.")
                self.set_vacuum(False)
                self.box_counter += 1
                continue
            time.sleep(0.5)

            # ================== AŞAMA 5: TOOLKIT (CYCLİC) ==================
            toolkit_wp = self.toolkit_waypoints[toolkit_index]
            self.get_logger().info(f"➡️ AŞAMA 5: Toolkit {toolkit_index + 1}'e gidiliyor...")
            self.arm.move_to_configuration(joint_positions=toolkit_wp)
            if not self.arm.wait_until_executed():
                self.get_logger().error(f"❌ Toolkit {toolkit_index + 1}'e gidiş başarısız!")
                self.set_vacuum(False)
                self.box_counter += 1
                continue

            self.get_logger().info(f"📦 Toolkit {toolkit_index + 1}'e ulaşıldı! {box_name} bırakılıyor...")
            self.set_vacuum(False)
            time.sleep(1.0) 

            self.get_logger().info(f"🎉 {box_name} → Toolkit {toolkit_index + 1} operasyonu tamamlandı!")
            self.box_counter += 1
            time.sleep(0.5)

# ====================================================
# ANA BAŞLATICIMIZ
# ====================================================
def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarMonitorNode()
    lidar_executor = rclpy.executors.SingleThreadedExecutor()
    lidar_executor.add_node(lidar_node)
    Thread(target=lidar_executor.spin, daemon=True).start()

    arm_node = SmartPickSequence(lidar_node)
    arm_executor = rclpy.executors.MultiThreadedExecutor(5)
    arm_executor.add_node(arm_node)
    Thread(target=arm_node.run_mission, daemon=True).start()

    try:
        arm_executor.spin()
    except KeyboardInterrupt:
        arm_node.get_logger().info("Kullanıcı tarafından durduruldu.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import os
import sys
import time
import math
import subprocess
from threading import Thread, Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Mesaj ve Servis Tipleri
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
from conveyorbelt_msgs.srv import ConveyorBeltControl

# pymoveit2_real kütüphanesi importları
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot

# vacuum.py içinden VacuumGripper'ı çekiyoruz
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if _CURRENT_DIR not in sys.path:
    sys.path.insert(0, _CURRENT_DIR)

try:
    from vacuum import VacuumGripper
except ImportError:
    from gemini_robotics_ros.vacuum import VacuumGripper

# Lidar / Fotosel ve temas sinyalleri için kilit mekanizmaları
BOX_DETECTED_EVENT = Event()        # 1. Giriş Sensörü (Sim: Lidar 1 / Real: DIN3)
END_BOX_DETECTED_EVENT = Event()    # 2. Çıkış Sensörü (Sim: Lidar 2 / Real: DIN4 - WP3 Kesici)
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
# DÜĞÜM 1: SİMÜLASYON LİDARLARI VE TEMAS SENSÖRÜNÜ DİNLEYEN
#           BAĞIMSIZ SİSTEM (Kendi executor'unda çalışır)
# ====================================================
class LidarMonitorNode(Node):
    def __init__(self):
        super().__init__("lidar_monitor_node")
        self.get_logger().info("📡 Lidar & Contact Monitor ayrı bir düğüm olarak başlatıldı!")

        # SmartPickSequence başlatıldığında kendi use_fake_hardware değerini buraya aktarır
        self.use_fake_hardware = True

        # 1. LIDAR (Başlangıç Tarayıcısı)
        self.sub_start = self.create_subscription(
            LaserScan,
            "/sensor_start_scan",
            self.lidar_start_callback,
            qos_profile_sensor_data,
        )
        self.debug_start_count = 0
        self.monitoring_start_active = False

        # 2. LIDAR (Bitiş Tarayıcısı - WP3 İptal Edici)
        self.sub_end = self.create_subscription(
            LaserScan,
            "/sensor_end_scan",
            self.lidar_end_callback,
            qos_profile_sensor_data,
        )
        self.debug_end_count = 0
        self.monitoring_end_active = False

        # 3. TEMAS SENSÖRÜ (MoveIt2'nin spin_once'ından bağımsız çalışır!)
        self.contact_sub = self.create_subscription(
            Contacts,
            "/world/cem/model/whole_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact",
            self.contact_callback,
            qos_profile_sensor_data,
        )
        self.monitoring_contact_active = False
        self._debug_contact_count = 0
        self.contact_callback_fn = None
        self.end_sensor_callback_fn = None

    # --- 1. LIDAR FONKSİYONU (Yalnızca Simülasyon Modunda Tetikler) ---
    def lidar_start_callback(self, msg):
        if not self.use_fake_hardware:
            return  # Gerçek modda giriş sinyali DIN3 fotoselinden (io_states_callback) gelir

        if len(msg.ranges) > 0:
            current_range = msg.ranges[0]
            self.debug_start_count += 1

            if self.debug_start_count % 15 == 0:
                self.get_logger().info(
                    f"🟢 [LIDAR 1] Mesafe: {current_range:.5f} | Aktif mi?: {self.monitoring_start_active}"
                )

            if math.isnan(current_range) or math.isinf(current_range):
                return

            if self.monitoring_start_active and current_range < 0.250:
                self.get_logger().info(f"📏 [LIDAR 1] Kutu Algılandı! ({current_range:.5f}m)")
                self.monitoring_start_active = False
                BOX_DETECTED_EVENT.set()

    # --- 2. LIDAR FONKSİYONU (Yalnızca Simülasyon Modunda Tetikler) ---
    def lidar_end_callback(self, msg):
        if not self.use_fake_hardware:
            return  # Gerçek modda çıkış sinyali DIN4 fotoselinden (io_states_callback) gelir

        if len(msg.ranges) > 0:
            current_range = msg.ranges[0]
            self.debug_end_count += 1

            if self.debug_end_count % 15 == 0:
                self.get_logger().info(
                    f"🔵 [LIDAR 2] Mesafe: {current_range:.5f} | Aktif mi?: {self.monitoring_end_active}"
                )

            if math.isnan(current_range) or math.isinf(current_range):
                return

            if self.monitoring_end_active and current_range < 0.250:
                self.monitoring_end_active = False
                END_BOX_DETECTED_EVENT.set()
                self.get_logger().info(
                    f"🚨 [LIDAR 2] Kutu WP3 yolunda algılandı! ({current_range:.5f}m) İptal tetiği çekiliyor..."
                )
                if self.end_sensor_callback_fn is not None:
                    try:
                        self.end_sensor_callback_fn()
                    except Exception as e:
                        self.get_logger().error(f"end_sensor_callback_fn hatası: {e}")

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
            if self.contact_callback_fn is not None:
                try:
                    self.contact_callback_fn()
                except Exception as e:
                    self.get_logger().error(f"contact_callback_fn hatası: {e}")


# ====================================================
# DÜĞÜM 2: ROBOTU, KONVEYÖRÜ VE VAKUMU YÖNETEN SİSTEM
# ====================================================
class SmartPickSequence(Node):
    def __init__(self, lidar_node):
        super().__init__("smart_pick_sequence_node")
        self.lidar_node = lidar_node
        self.state = "INIT"
        self.box_counter = 0  # box0, box1, box2, ...

        cbg = ReentrantCallbackGroup()

        # ── 1. Parametreleri Tanımla ve Oku ──
        self.declare_parameter("use_fake_hardware", True)
        self.declare_parameter("conveyor_power", 2.5)
        self.declare_parameter("conveyor_scale", 0.5)  # Gerçek voltaj = power * scale (2.5 * 0.5 = 1.25V)
        self.declare_parameter("conveyor_forward_pin", 2)
        self.declare_parameter("sensor_start_pin", 3)
        self.declare_parameter("sensor_end_pin", 4)
        self.declare_parameter("analog_voltage_pin", 0)

        self.use_fake_hardware = bool(self.get_parameter("use_fake_hardware").value)
        self.conveyor_power = float(self.get_parameter("conveyor_power").value)
        self.conveyor_scale = float(self.get_parameter("conveyor_scale").value)
        self.conveyor_forward_pin = int(self.get_parameter("conveyor_forward_pin").value)
        self.sensor_start_pin = int(self.get_parameter("sensor_start_pin").value)
        self.sensor_end_pin = int(self.get_parameter("sensor_end_pin").value)
        self.analog_voltage_pin = int(self.get_parameter("analog_voltage_pin").value)

        # LidarMonitorNode'a hangi modda olduğumuzu bildir
        self.lidar_node.use_fake_hardware = self.use_fake_hardware

        self.get_logger().info(
            f"Mod: {'SİMÜLASYON (Fake Hardware)' if self.use_fake_hardware else 'GERÇEK + SİMÜLASYON EŞZAMANLI'}"
        )

        # ── 2. Vakum Arayüzü (vacuum.py üzerinden hem Sim hem Real) ──
        self.vacuum_gripper = VacuumGripper(
            node=self,
            command_topic="/OnRobotVGOutput",
            status_topic="/OnRobotVGInput",
            vacuum_pct=60,
            require_confirmation=False,
            sim_suction_topic="/suction/enable",
            use_fake_hardware=self.use_fake_hardware,
        )
        self.lidar_node.contact_callback_fn = self.on_contact_detected
        self.lidar_node.end_sensor_callback_fn = self.on_end_sensor_detected

        # ── 3. Konveyör Arayüzleri (Sim: /CONVEYORPOWER, Real: UR10e SetIO & IOStates) ──
        self.sim_conveyor_client = self.create_client(
            ConveyorBeltControl, "/CONVEYORPOWER", callback_group=cbg
        )
        self.set_io_client = self.create_client(
            SetIO, "/io_and_status_controller/set_io", callback_group=cbg
        )
        self.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_states_callback,
            10,
            callback_group=cbg,
        )

        # ── 4. Ortak MoveIt2 Arayüzü ──
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

    # =========================================================================
    # BÖLÜM 1: SİMÜLASYON (GAZEBO) FONKSİYONLARI
    # =========================================================================
    def set_sim_vacuum(self, state: bool, repeat: int = 5, delay: float = 0.05):
        """vacuum.py içindeki sim_grip / sim_release fonksiyonlarını çağırır."""
        if state:
            self.vacuum_gripper.sim_grip(repeat=repeat, delay=delay)
        else:
            self.vacuum_gripper.sim_release(repeat=repeat, delay=delay)

    def spawn_box(self):
        """Gazebo simülasyonunda yeni bir kutu spawn eder (box0, box1, box2, ...)."""
        box_name = f"box{self.box_counter}"
        x, y, z = BOX_SPAWN_POSITION
        self.get_logger().info(f"📦 [SİM] {box_name} spawn ediliyor → ({x}, {y}, {z})")

        sdf_str = BOX_SDF_TEMPLATE.format(box_name=box_name)

        try:
            result = subprocess.run(
                [
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "create",
                    "-string",
                    sdf_str,
                    "-name",
                    box_name,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    str(z),
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )
            if result.returncode == 0:
                self.get_logger().info(f"✅ [SİM] {box_name} başarıyla spawn edildi!")
            else:
                self.get_logger().error(f"❌ [SİM] {box_name} spawn hatası: {result.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"❌ [SİM] {box_name} spawn zaman aşımı!")
        except Exception as e:
            self.get_logger().error(f"❌ [SİM] {box_name} spawn exception: {e}")

    # =========================================================================
    # BÖLÜM 2: GERÇEK DONANIM (REAL ROBOT I/O, KONVEYÖR & VGC10 VAKUM) FONKSİYONLARI
    # =========================================================================
    def io_states_callback(self, msg: IOStates):
        """Gerçek UR10e kontrolcüsünden gelen dijital girişleri (DIN3, DIN4) dinler."""
        if self.use_fake_hardware:
            return  # Simülasyon modundaysak gerçek I/O dinlenmez

        for digital in msg.digital_in_states:
            # 1. Giriş Sensörü (DIN3) -> Kutu geldi!
            if digital.pin == self.sensor_start_pin and digital.state:
                if self.lidar_node.monitoring_start_active:
                    self.get_logger().info(
                        f"📏 [REAL FOTOSEL GİRİŞ - DIN{self.sensor_start_pin}] Kutu Algılandı!"
                    )
                    self.lidar_node.monitoring_start_active = False
                    BOX_DETECTED_EVENT.set()

            # 2. Çıkış Sensörü (DIN4) -> WP3 Kesici!
            if digital.pin == self.sensor_end_pin and digital.state:
                if self.lidar_node.monitoring_end_active:
                    self.lidar_node.monitoring_end_active = False
                    END_BOX_DETECTED_EVENT.set()
                    self.get_logger().info(
                        f"🚨 [REAL FOTOSEL ÇIKIŞ - DIN{self.sensor_end_pin}] Kutu Algılandı! WP3 İptal Tetiği!"
                    )
                    self.on_end_sensor_detected()

    def set_digital_output(self, pin: int, state: bool):
        """Gerçek UR10e kontrolcüsünde DOUT pinini HIGH (1.0) veya LOW (0.0) yapar."""
        if self.use_fake_hardware:
            return True
        if not self.set_io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[REAL] set_io servisi bulunamadı!")
            return False
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = int(pin)
        req.state = float(1.0 if state else 0.0)
        self.set_io_client.call_async(req)
        return True

    def set_analog_output(self, pin: int, voltage: float):
        """Gerçek UR10e kontrolcüsünde AOUT pinine voltaj (0.0 - 10.0V) yazar."""
        if self.use_fake_hardware:
            return True
        if not self.set_io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[REAL] set_io servisi bulunamadı!")
            return False
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_ANALOG_OUT
        req.pin = int(pin)
        req.state = float(voltage)
        self.set_io_client.call_async(req)
        return True

    def start_conveyor(self):
        """Konveyörü başlatır (Dijital İkiz: Gazebo bandı her zaman döner; gerçek modda fiziksel bant da döner)."""
        # 1. Gazebo Simülasyon Bandı (Her iki modda da kutu taşınsın diye her zaman çalışır!)
        if self.sim_conveyor_client.wait_for_service(timeout_sec=2.0):
            req = ConveyorBeltControl.Request()
            req.power = float(self.conveyor_power)
            self.sim_conveyor_client.call_async(req)
            self.get_logger().info(f"🚀 [SİM] Konveyör başlatıldı (power={self.conveyor_power})")
        else:
            self.get_logger().warn("[SİM] /CONVEYORPOWER servisi bulunamadı!")

        # 2. Gerçek Donanım Bandı (Yalnızca use_fake_hardware:=false iken devreye girer)
        if not self.use_fake_hardware:
            target_voltage = min(max(self.conveyor_power * self.conveyor_scale, 0.0), 10.0)
            self.get_logger().info(
                f"🚀 [REAL] Konveyör başlatılıyor... (power={self.conveyor_power} -> AOUT{self.analog_voltage_pin}={target_voltage:.2f}V, DOUT{self.conveyor_forward_pin}=HIGH)"
            )
            self.set_analog_output(self.analog_voltage_pin, target_voltage)
            time.sleep(0.05)
            self.set_digital_output(self.conveyor_forward_pin, True)

    def stop_conveyor(self):
        """Konveyörü durdurur (Dijital İkiz: Hem Gazebo hem de Gerçek bant durdurulur)."""
        # 1. Simülasyon bandını durdur
        if self.sim_conveyor_client.service_is_ready():
            req = ConveyorBeltControl.Request()
            req.power = 0.0
            self.sim_conveyor_client.call_async(req)
            self.get_logger().info("🛑 [SİM] Konveyör durduruldu (power=0.0)")

        # 2. Gerçek bandı durdur
        if not self.use_fake_hardware:
            self.get_logger().info(f"🛑 [REAL] Konveyör durduruluyor (DOUT{self.conveyor_forward_pin}=LOW)...")
            self.set_digital_output(self.conveyor_forward_pin, False)

    def set_real_vacuum(self, state: bool, wait: bool = True):
        """vacuum.py içindeki real_grip / real_release fonksiyonlarını çağırır."""
        if state:
            self.vacuum_gripper.real_grip(wait=wait)
        else:
            self.vacuum_gripper.real_release(wait=wait)

    # =========================================================================
    # BÖLÜM 3: ORTAK HAREKET, VAKUM BİRLEŞTİRME VE GÖREV DÖNGÜSÜ FONKSİYONLARI
    # =========================================================================
    def set_vacuum(self, state: bool):
        """Hem Simülasyon (Gazebo) hem de Gerçek (VGC10) vakumu vacuum.py üzerinden kontrol eder."""
        # 1. Simülasyon vakumu (her iki modda da Gazebo'daki kutuyu tutmak/bırakmak için çalışır)
        self.set_sim_vacuum(state, repeat=5, delay=0.05)

        # 2. Gerçek donanım vakumu (yalnızca use_fake_hardware:=false iken çalışır)
        self.set_real_vacuum(state, wait=True)

        self.get_logger().info(f"🔧 Vakum {'AÇIK' if state else 'KAPALI'}")

    def on_contact_detected(self):
        """Temas algılandığı mikrosaniyede doğrudan sensor callback'i içinden çalışır (0 ms gecikme)."""
        self.get_logger().info(
            "⚡ [ANINDA MÜDAHALE] Temas algılandı! Vakum açılıyor ve hareket kesiliyor..."
        )

        # 1. HEMEN SİMÜLASYON VAKUMUNU BAS (0 ms, beklemesiz)
        self.set_sim_vacuum(True, repeat=3, delay=0.0)

        # 2. HEMEN GERÇEK VAKUMU BAS (0 ms, beklemesiz - wait=False)
        self.set_real_vacuum(True, wait=False)

        # 3. HAREKETİ ANINDA DURDUR (0 ms gecikme)
        self.arm.cancel_execution()

        # 4. Action Goal Handle üzerinden iptal gönder
        try:
            goal_handle = getattr(self.arm, "_MoveIt2__execution_goal_handle", None)
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
        except Exception:
            pass

    def on_end_sensor_detected(self):
        """Çıkış sensörü (Sim: Lidar 2 / Real: DIN4) kutu gördüğünde doğrudan çağrılır."""
        if self.state == "MOVING_TO_WP3":
            self.get_logger().info("🛑 [HAVADA ANINDA MÜDAHALE] Arkadan kutu geldi, WP3 iptal ediliyor!")
            self.force_cancel_motion()

    def force_cancel_motion(self):
        """Aktif trajectory execution'ı zorla ve güvenli şekilde iptal eder."""
        self.get_logger().info("🛑 Hareket iptal ediliyor...")

        # 1. MoveIt trajectory execution manager'a durdur emri
        self.arm.cancel_execution()

        # 2. Action Goal Handle üzerinden cancel isteği gönder
        try:
            goal_handle = getattr(self.arm, "_MoveIt2__execution_goal_handle", None)
            if goal_handle is not None:
                self.get_logger().info("   Action goal handle cancel ediliyor...")
                goal_handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f"   Goal handle cancel hatası: {e}")

        # 3. İptalin MoveIt tarafından işlenip tamamlanmasını bekle (race condition koruması)
        timeout = time.time() + 1.0
        while time.time() < timeout:
            if not getattr(self.arm, "_MoveIt2__is_executing", False):
                break
            time.sleep(0.01)

        # 4. State bayraklarını temizle
        try:
            self.arm._MoveIt2__is_motion_requested = False
            self.arm._MoveIt2__is_executing = False
            self.arm._MoveIt2__execution_goal_handle = None
            self.arm.motion_suceeded = False
        except AttributeError:
            pass

        self.get_logger().info("✅ Hareket iptali tamamlandı.")

    def run_mission(self):
        time.sleep(1.0)

        # 1. Önce robot WP1 (bekleme) pozisyonuna gitsin
        self.get_logger().info("➡️ BAŞLANGIÇ: Robot WP1 pozisyonuna gidiyor...")
        self.arm.move_to_configuration(joint_positions=self.wp1)
        self.arm.wait_until_executed()
        self.get_logger().info("✅ Robot WP1'e yerleşti!")

        # 2. Robot WP1'e oturduktan sonra konveyör başlasın!
        self.start_conveyor()

        while True:
            box_name = f"box{self.box_counter}"
            toolkit_index = self.box_counter % 4

            self.get_logger().info(f"\n{'='*55}")
            self.get_logger().info(
                f"🔄 DÖNGÜ #{self.box_counter}: {box_name} → Toolkit {toolkit_index + 1}"
            )
            self.get_logger().info(f"{'='*55}")

            # ================== AŞAMA 1: WP1 KONTROLÜ + KUTU SPAWN & SENSÖR ==================
            # Robot zaten WP1'de değilse WP1'e gitsin (ilk döngüde zaten WP1'dedir)
            self.arm.move_to_configuration(joint_positions=self.wp1)
            self.arm.wait_until_executed()

            if self.use_fake_hardware:
                # SİMÜLASYON MODU: Robot WP1'de, konveyör akıyor -> Şimdi kutuyu spawn et!
                self.spawn_box()
                time.sleep(0.5)
                self.get_logger().info("⏳ Simülasyon Lidar 1 (< 0.250m) bekleniyor...")
                BOX_DETECTED_EVENT.clear()
                self.lidar_node.monitoring_start_active = True
                BOX_DETECTED_EVENT.wait()
            else:
                # GERÇEK + SİMÜLASYON EŞZAMANLI MOD:
                # Robot WP1'de bekliyor; gerçek kutu DIN3 fotoselinden geçtiği an sanal ikizi spawn et!
                self.get_logger().info(
                    f"⏳ Gerçek Giriş Sensörü (DIN{self.sensor_start_pin}) bekleniyor..."
                )
                BOX_DETECTED_EVENT.clear()
                self.lidar_node.monitoring_start_active = True
                BOX_DETECTED_EVENT.wait()
                # Gerçek kutu algılandı -> Gazebo'da da eşzamanlı ikiz kutuyu spawn et (arka planda)
                Thread(target=self.spawn_box, daemon=True).start()

            # ================== AŞAMA 2: WP2 PICK (TEMAS İLE) ==================
            self.get_logger().info("🚀 Giriş sensöründen sinyal alındı! WP2'ye iniliyor!")
            self.get_logger().info("➡️ AŞAMA 2: WP2'ye iniliyor. Temas dinleme AKTİF!")

            # max_acceleration yüksek (0.8) tutularak temas algılandığında
            # robotun anında fren yapması ve kutuyu ezmemesi/sekmemesi sağlanır
            self.arm.max_velocity = 0.30
            self.arm.max_acceleration = 0.8

            CONTACT_DETECTED_EVENT.clear()
            self.lidar_node.monitoring_contact_active = True

            self.arm.move_to_configuration(joint_positions=self.wp2)
            self.arm.wait_until_executed()

            self.lidar_node.monitoring_contact_active = False

            if CONTACT_DETECTED_EVENT.is_set():
                self.get_logger().info("💥 Temas başarıyla yakalandı, vakum kilitlendi ve duruldu!")
            else:
                self.get_logger().warn("⚠️ Temas algılanmadı, robot doğrudan WP2 noktasına kadar indi!")
                self.set_vacuum(True)

            # MoveIt ve controller'ın eski iptal edilmiş hedefi temizlemesini garantile (WP3 koruması)
            self.force_cancel_motion()

            self.arm.max_velocity = 0.08
            self.arm.max_acceleration = 0.10

            # ================== AŞAMA 3: WP3 (ÇIKIŞ SENSÖRÜ İPTAL KORUMASI) ==================
            self.get_logger().info("➡️ AŞAMA 3: WP3'e gidiliyor (Çıkış Sensörü İptal Koruması AKTİF)...")

            self.state = "MOVING_TO_WP3"
            END_BOX_DETECTED_EVENT.clear()
            self.lidar_node.monitoring_end_active = True

            # Doğrudan WP3'e git (İptal gerektiğinde on_end_sensor_detected fonksiyonu 0ms'de keser!)
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

            time.sleep(0.05)

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

            # 🛡️ GÜVENLİ GERİ DÖNÜŞ: Kasanın içinden kestirmeden değil, önce WP4 kapısına çık!
            self.get_logger().info("🛡️ Güvenli ara noktaya (WP4) geri dönülüyor...")
            self.arm.move_to_configuration(joint_positions=self.wp4)
            self.arm.wait_until_executed()

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
        arm_node.stop_conveyor()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
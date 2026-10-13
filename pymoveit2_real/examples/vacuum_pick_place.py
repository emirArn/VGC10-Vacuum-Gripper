#!/usr/bin/env python3

"""
Akış: Belirtilen X, Y, Z koordinatlarına git, tek bir hareketle aşağı inmeye başla, 
temas anında aniden dur ve vakumu aç, 30cm yukarı kaldır, ardından belirtilen place konumuna 
giderek kutuyu bırak.

Bu script pymoveit2_real kütüphanesi ile gerçek/real robot konfigürasyonu
üzerinden çalışacak şekilde yazılmıştır.

Bridge komutu (önce çalıştırılmalı):
  ros2 run ros_gz_bridge parameter_bridge \
    /suction/enable@std_msgs/msg/Bool]ignition.msgs.Boolean \
    /world/cem/model/my_robot_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts
"""

import sys
import argparse
from threading import Thread, Event
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# --- Mesaj Tipleri ---
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot


class VacuumController(Node):
    def __init__(self):
        super().__init__("vacuum_waypoint_controller")

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
        self.arm.max_velocity = 0.04
        self.arm.max_acceleration = 0.04

        # --- TCP ve Yönelim Ayarları ---
        self.TCP_OFFSET_IN_TOOL0 = [0.113, 0.100, 0.000]
        self.GRIPPER_DOWN_QUAT = [-0.669, -0.230, 0.693, 0.136]

        # --- Vakum ve Sensör Ayarları ---
        self.contact_detected = False
        self._contact_event = Event()  # Thread-safe temas event'i
        self._contact_listening = False  # Contact callback'i sadece pick sırasında aktif

        # Temas sensörü dinleyicisi (bridge ile köprülenmiş Gazebo contact topic'i)
        self.contact_sub = self.create_subscription(
            Contacts,
            '/world/cem/model/whole_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact',
            self.contact_callback,
            10,
            callback_group=cbg
        )

        # MoveIt Planning Scene (Planlama Sahnesi) Yayıncısı
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Vakum kontrolcüsü (Gazebo plugin'indeki topic)
        self.vacuum_pub = self.create_publisher(
            Bool,
            '/suction/enable',
            10
        )

        self.get_logger().info("Vakum controller hazır, konum komutları bekleniyor.")

    def contact_callback(self, msg):
        """Temas sensöründen veri geldiğinde vakumu açar ve robotu durdurur.
        Sadece pick iniş sırasında aktif (_contact_listening=True)."""
        if not self._contact_listening:
            return  # Pick dışında contact callback'i yoksay

        if len(msg.contacts) > 0:
            if not self.contact_detected:
                self.get_logger().info("📡 Contact callback tetiklendi! Temas algılandı.")
                suction_msg = Bool()
                suction_msg.data = True
                self.vacuum_pub.publish(suction_msg)
                self.vacuum_pub.publish(suction_msg)
                self.vacuum_pub.publish(suction_msg)
                self.get_logger().info("✅ Vakum AÇILDI (contact callback'ten)")
                
                # --- HAREKETİ ANINDA DURDUR ---
                self.force_cancel_motion()
                
            self.contact_detected = True
            self._contact_event.set()

    def set_vacuum(self, enable: bool):
        """Vakum durumunu değiştirir ve birden fazla kez yayımlar (güvenilirlik için)."""
        suction_msg = Bool()
        suction_msg.data = enable
        state_str = "AÇILIYOR" if enable else "KAPATILIYOR"
        self.get_logger().info(f"🔧 Vakum {state_str}...")
        for _ in range(5):
            self.vacuum_pub.publish(suction_msg)
            time.sleep(0.05)
        self.get_logger().info(f"✅ Vakum {'AÇIK' if enable else 'KAPALI'}")
    
    def add_conveyor_to_scene(self):
        """Konveyörü bir engel (Collision Object) olarak MoveIt'e tanıtır."""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        conveyor = CollisionObject()
        conveyor.id = "my_conveyor_belt"
        conveyor.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.3584, 0.9689, 0.8] 

        pose = Pose()
        pose.position.x = 0.7422
        pose.position.y = 0.0966
        pose.position.z = 0.4428  
        pose.orientation.w = 1.0

        conveyor.primitives.append(primitive)
        conveyor.primitive_poses.append(pose)
        conveyor.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(conveyor)
        self.planning_scene_pub.publish(scene_msg)
        
        self.get_logger().info("🚧 Konveyör bant MoveIt sahnesine eklendi!")

    def force_cancel_motion(self):
        """Aktif trajectory execution'ı zorla iptal eder."""
        self.get_logger().info("🛑 Hareket zorla iptal ediliyor...")

        self.arm.cancel_execution()

        try:
            goal_handle = self.arm._MoveIt2__execution_goal_handle
            if goal_handle is not None:
                self.get_logger().info("   Action goal handle cancel ediliyor...")
                cancel_future = goal_handle.cancel_goal_async()
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

    @staticmethod
    def _quat_rotate(quat_xyzw, vec):
        x, y, z, w = quat_xyzw
        r00 = 1 - 2*(y*y + z*z)
        r01 = 2*(x*y - z*w)
        r02 = 2*(x*z + y*w)
        r10 = 2*(x*y + z*w)
        r11 = 1 - 2*(x*x + z*z)
        r12 = 2*(y*z - x*w)
        r20 = 2*(x*z - y*w)
        r21 = 2*(y*z + x*w)
        r22 = 1 - 2*(x*x + y*y)
        return [
            r00*vec[0] + r01*vec[1] + r02*vec[2],
            r10*vec[0] + r11*vec[1] + r12*vec[2],
            r20*vec[0] + r21*vec[1] + r22*vec[2],
        ]

    def pick_object(self, target_x, target_y, target_z, retries=3, sleep_between=0.4):
        """Objeyi alır: approach → iniş (temasta dur + vakum aç) → 30cm yukarı kaldır."""
        quat = self.GRIPPER_DOWN_QUAT
        tcp_world = self._quat_rotate(quat, self.TCP_OFFSET_IN_TOOL0)

        tool0_x = target_x - tcp_world[0]
        tool0_y = target_y - tcp_world[1]
        tool0_z = target_z - tcp_world[2]

        approach_z = tool0_z + 0.10  

        self.get_logger().info(f"İstenen Vakum Ucu Konumu: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")

        # --- 1. AŞAMA: YAKLAŞMA (APPROACH) ---
        self.get_logger().info("AŞAMA 1: Hedefin 10 cm üstüne hizalanıyor (Approach)...")
        approach_success = False
        for i in range(1, retries + 1):
            self.arm.move_to_pose(
                position=[tool0_x, tool0_y, approach_z],
                quat_xyzw=quat,
                frame_id="world",
                cartesian=False,
                tolerance_position=0.005,
                tolerance_orientation=0.02,
            )
            if self.arm.wait_until_executed():
                approach_success = True
                self.get_logger().info("Hizalama tamamlandı ✅")
                break
            time.sleep(sleep_between)
            
        if not approach_success:
            self.get_logger().error("Approach başarısız!")
            return False

        # --- 2. AŞAMA: İNİŞ (TEMASTA KESİLECEK) ---
        self.get_logger().info("AŞAMA 2: Kutuya doğru iniliyor (Temas anında fren yapılacak)...")
        self.contact_detected = False
        self._contact_event.clear()
        self._contact_listening = True  # Contact callback'i AKTİF ET

        # Sensör görmezse çarpışmamak için maksimum inebileceği derinlik (15 cm aşağı)
        max_depth_z = approach_z - 0.15 

        # Hedefe tek bir lineer iniş komutu ver
        self.arm.move_to_pose(
            position=[tool0_x, tool0_y, max_depth_z],
            quat_xyzw=quat,
            frame_id="world",
            cartesian=True, 
            tolerance_position=0.005,
            tolerance_orientation=0.05,
        )
        
        # Hareketi bekle. Sensör teması algılarsa callback içindeki force_cancel_motion() 
        # robotu durduracak ve bu bekleme otomatik olarak sonlanacaktır.
        self.arm.wait_until_executed()

        # İniş bitti, contact dinlemeyi KAPAT (place sırasında tetiklenmesin)
        self._contact_listening = False

        if self.contact_detected:
            self.get_logger().info("💥 Temas algılandı ve iniş başarıyla durduruldu!")
        else:
            self.get_logger().warn("⚠️ Maksimum derinliğe (15cm) inildi ancak temas sağlanamadı!")

        # Vakumu aç (contact callback'te zaten açılmış olabilir ama emin olmak için)
        self.set_vacuum(True)
        time.sleep(0.5) 

        # --- 3. AŞAMA: 30cm YUKARI KALDIR (LIFT) ---
        self.get_logger().info("AŞAMA 3: Obje ile birlikte 30cm yukarı kaldırılıyor (Lift)...")
        time.sleep(0.5)

        lift_target_z = approach_z + 0.30  # Approach noktasından 30cm yukarı

        self.get_logger().info(f"  ↑ Cartesian olarak yukarı çıkılıyor (Hedef Z: {lift_target_z:.4f})...")
        self.arm.move_to_pose(
            position=[tool0_x, tool0_y, lift_target_z],
            quat_xyzw=quat,
            frame_id="world",
            cartesian=True,
            tolerance_position=0.01,
            tolerance_orientation=0.1,
        )
        self.arm.wait_until_executed()

        self.get_logger().info("Alma (Pick) işlemi tamamlandı! ✅")
        return True

    def place_object(self, target_x, target_y, target_z, retries=3, sleep_between=0.4):
        """Kutuyu belirtilen hedefe bırakır (Place işlemi)."""
        quat = self.GRIPPER_DOWN_QUAT
        tcp_world = self._quat_rotate(quat, self.TCP_OFFSET_IN_TOOL0)

        tool0_x = target_x - tcp_world[0]
        tool0_y = target_y - tcp_world[1]
        tool0_z = target_z - tcp_world[2]

        approach_z = tool0_z + 0.10  

        self.get_logger().info(f"İstenen Bırakma Konumu: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}")

        # --- 1. AŞAMA: BIRAKMA NOKTASINA YAKLAŞMA ---
        self.get_logger().info("AŞAMA 1 (Bırakma): Bırakma noktasının 10 cm üstüne gidiliyor...")
        approach_success = False
        for i in range(1, retries + 1):
            self.arm.move_to_pose(
                position=[tool0_x, tool0_y, approach_z],
                quat_xyzw=quat,
                frame_id="world",
                cartesian=False,
                tolerance_position=0.005,
                tolerance_orientation=0.02,
            )
            if self.arm.wait_until_executed():
                approach_success = True
                break
            time.sleep(sleep_between)
            
        if not approach_success:
            self.get_logger().error("Bırakma noktasına hizalanılamadı!")
            return False

        # --- 2. AŞAMA: ALÇALMA ---
        self.get_logger().info("AŞAMA 2 (Bırakma): Yavaşça hedefe iniliyor...")
        self.arm.move_to_pose(
            position=[tool0_x, tool0_y, tool0_z],
            quat_xyzw=quat,
            frame_id="world",
            cartesian=True,
            tolerance_position=0.005,
            tolerance_orientation=0.05,
        )
        self.arm.wait_until_executed()

        # --- 3. AŞAMA: VAKUMU KAPAT ---
        self.get_logger().info("AŞAMA 3 (Bırakma): Vakum kapatılıyor (Kutu bırakıldı)...")
        self.set_vacuum(False)
        time.sleep(1.0) 

        # --- 4. AŞAMA: GERİ ÇEKİLME ---
        self.get_logger().info("AŞAMA 4 (Bırakma): Güvenli yüksekliğe çıkılıyor...")
        self.arm.move_to_pose(
            position=[tool0_x, tool0_y, approach_z],
            quat_xyzw=quat,
            frame_id="world",
            cartesian=True,
            tolerance_position=0.005,
            tolerance_orientation=0.05,
        )
        self.arm.wait_until_executed()
        self.get_logger().info("Kutu başarıyla istenen konuma bırakıldı! 📦✅")

        return True


def main(args=None):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)

    parser = argparse.ArgumentParser(description="Vakum Gripper ile objeyi alır ve belirtilen yere bırakır.")
    parser.add_argument('x', type=float, help='Alma X koordinatı (metre)')
    parser.add_argument('y', type=float, help='Alma Y koordinatı (metre)')
    parser.add_argument('z', type=float, help='Alma Z koordinatı (metre)')
    parser.add_argument('--place-x', type=float, default=None, help='Bırakma X (opsiyonel)')
    parser.add_argument('--place-y', type=float, default=None, help='Bırakma Y (opsiyonel)')
    parser.add_argument('--place-z', type=float, default=None, help='Bırakma Z (opsiyonel)')

    try:
        parsed_args = parser.parse_args(args_without_ros[1:])
    except SystemExit:
        print("\n❌ KULLANIM: ros2 run <paket_adi> <node_adi> <X> <Y> <Z> [--place-x PX --place-y PY --place-z PZ]")
        print("ÖRNEK: ros2 run pymoveit2_real vacuum_pick_place.py 0.75 0.10 0.88\n")
        rclpy.shutdown()
        return

    ctrl = VacuumController()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(ctrl)
    t = Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(1.0)

    # ctrl.add_conveyor_to_scene()
    
    time.sleep(0.5) 

    try:
        # --- PICK ---
        if ctrl.pick_object(parsed_args.x, parsed_args.y, parsed_args.z, retries=3):
            ctrl.get_logger().info("Kutu kancalandı! ✅")

            # --- PLACE ---
            if parsed_args.place_x is not None and parsed_args.place_y is not None and parsed_args.place_z is not None:
                ctrl.get_logger().info("Şimdi bırakma rotasına geçiliyor...")
                if not ctrl.place_object(parsed_args.place_x, parsed_args.place_y, parsed_args.place_z):
                    ctrl.get_logger().error("Bırakma (Place) işlemi başarısız oldu!")
                else:
                    ctrl.get_logger().info("Tüm Pick & Place operasyonu başarıyla tamamlandı! 🎉")
            else:
                ctrl.get_logger().info("Bırakma koordinatları verilmedi. Pick tamamlandı, vakum açık kalıyor.")
        else:
            ctrl.get_logger().error("Alma (Pick) işlemi için hedefe gidiş başarısız oldu.")
    finally:
        rclpy.shutdown()
        t.join()


if __name__ == "__main__":
    main()
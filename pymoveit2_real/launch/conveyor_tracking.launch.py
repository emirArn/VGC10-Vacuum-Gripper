from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -----------------------------------------------------------------
    # LAUNCH ARGÜMANLARI
    # -----------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="true: Gazebo simülasyonu, false: Gerçek robot + Gazebo ikizi eşzamanlı",
        ),
        DeclareLaunchArgument(
            "conveyor_power",
            default_value="2.5",
            description="Simülasyon konveyör gücü (Gazebo /CONVEYORPOWER). Gerçekte bunun yarısı (0.5 katı) voltaj olarak gider.",
        ),
        DeclareLaunchArgument(
            "conveyor_scale",
            default_value="0.5",
            description="Simülasyon power -> Gerçek Voltaj katsayısı (varsayılan: 0.5, yani 2.5 power = 1.25V)",
        ),
        DeclareLaunchArgument(
            "conveyor_forward_pin",
            default_value="2",
            description="Konveyör inverter forward çıkış pini (DOUT2)",
        ),
        DeclareLaunchArgument(
            "sensor_start_pin",
            default_value="3",
            description="Konveyör giriş fotosel sensör giriş pini (DIN3)",
        ),
        DeclareLaunchArgument(
            "sensor_end_pin",
            default_value="4",
            description="Konveyör çıkış fotosel sensör giriş pini (DIN4)",
        ),
        DeclareLaunchArgument(
            "analog_voltage_pin",
            default_value="0",
            description="Konveyör hız analog çıkış pini (AOUT0)",
        ),
    ]

    # =======================================================
    # 1. KISIM: GAZEBO - ROS 2 KÖPRÜSÜ
    # =======================================================
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_node",
        output="screen",
        arguments=[
            # 1. Lidar (Başlangıç)
            "/sensor_start_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # 2. Lidar (Bitiş / WP3 İptal Edici)
            "/sensor_end_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # 3. Vakum Komutu (Gazebo parça tutuşu için)
            "/suction/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
            # 4. Temas Sensörü
            "/world/cem/model/whole_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
    )

    # =======================================================
    # 2. KISIM: BİZİM ASIL PYTHON KODUMUZ
    # =======================================================
    smart_pick_sequence_node = Node(
        package="pymoveit2_real",
        executable="sensor_pick.py",
        name="smart_pick_sequence_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_fake_hardware": ParameterValue(
                    LaunchConfiguration("use_fake_hardware"), value_type=bool
                ),
                "conveyor_power": ParameterValue(
                    LaunchConfiguration("conveyor_power"), value_type=float
                ),
                "conveyor_scale": ParameterValue(
                    LaunchConfiguration("conveyor_scale"), value_type=float
                ),
                "conveyor_forward_pin": ParameterValue(
                    LaunchConfiguration("conveyor_forward_pin"), value_type=int
                ),
                "sensor_start_pin": ParameterValue(
                    LaunchConfiguration("sensor_start_pin"), value_type=int
                ),
                "sensor_end_pin": ParameterValue(
                    LaunchConfiguration("sensor_end_pin"), value_type=int
                ),
                "analog_voltage_pin": ParameterValue(
                    LaunchConfiguration("analog_voltage_pin"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_bridge_node,
            smart_pick_sequence_node,
        ]
    )
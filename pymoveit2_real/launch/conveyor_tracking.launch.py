import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # =======================================================
        # 1. KISIM: GAZEBO - ROS 2 KÖPRÜSÜ
        # =======================================================
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_node',
            output='screen',
            arguments=[
                # 1. Lidar (Başlangıç)
                '/sensor_start_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                
                # 2. Lidar (Bitiş / WP3 İptal Edici)
                '/sensor_end_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                
                # 3. Vakum Komutu (Ayrı bir eleman olarak)
                '/suction/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
                
                # 4. Temas Sensörü (Ayrı bir eleman olarak)
                '/world/cem/model/whole_cell_sim/link/sim_ur10e_suction_cup/sensor/suction_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
            ]
        ),

        # =======================================================
        # 2. KISIM: BİZİM ASIL PYTHON KODUMUZ
        # =======================================================
        Node(
            package='pymoveit2_real',
            # DİKKAT: Buraya setup.py'de kaydettiğin adı yazmalısın. 
            # Uzantısız ve dizin belirtmeden sadece ismini yazıyoruz.
            executable='sensor_pick.py',  
            name='smart_pick_sequence_node',
            output='screen'
        )
    ])
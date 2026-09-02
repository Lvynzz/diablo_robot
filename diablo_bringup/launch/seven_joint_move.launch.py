import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Mendapatkan path ke direktori 'diablo_bringup'
    pkg = get_package_share_directory('diablo_bringup')
    
    # Path ke file xacro (model robot)
    xacro_file = os.path.join(pkg, 'urdf', 'seven_joint.urdf.xacro')
    
    # Path ke file konfigurasi controller
    controllers_yaml = os.path.join(pkg, 'config', 'seven_joint_controllers.yaml')

    # Perintah untuk memproses xacro menjadi URDF
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Node robot_state_publisher: mempublikasikan transformasi TF dan state dari robot berdasarkan URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Node controller_manager: membaca URDF (dengan tag ros2_control) dan konfigurasi YAML,
    # lalu berkomunikasi dengan hardware dynamixel (melalui hardware interface)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    # Spawner untuk joint_state_broadcaster: membaca state dari hardware (posisi, dll)
    # dan mempublikasikannya ke topik /joint_states agar bisa dibaca oleh robot_state_publisher
    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawner untuk controller posisi/trajectory: menerima perintah (command) dan mengirimkannya ke motor dynamixel
    spawn_pos_ctrl = Node(
        package='controller_manager', executable='spawner',
        arguments=['seven_joint_trajectory_controller'],  # Sesuaikan dengan nama di yaml
    )

    return LaunchDescription([
        robot_state_publisher, 
        controller_manager, 
        spawn_jsb, 
        spawn_pos_ctrl,
    ])

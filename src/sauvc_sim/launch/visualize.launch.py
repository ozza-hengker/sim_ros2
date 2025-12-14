import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'sauvc_sim'
    
    # 1. URDF Processing
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'auv_base.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 2. Gazebo Setup (Load Underwater World)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'underwater.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 3. Spawn Robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sauvc_bot',
            '-string', robot_desc,
            '-z', '-0.5' 
        ],
        output='screen',
    )

    # 4. Bridge (ROS <-> Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Thruster Kiri & Kanan
            '/model/sauvc_bot/joint/thruster_left_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/sauvc_bot/joint/thruster_right_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
            
            # Thruster Vertikal
            '/model/sauvc_bot/joint/vertical_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',

            # Kamera
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn,
        bridge
    ])

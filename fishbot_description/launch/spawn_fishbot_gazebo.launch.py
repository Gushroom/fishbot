import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the URDF file
    package_name = 'fishbot_description'
    urdf_file_name = 'fishbot_base.urdf'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # Start Gazebo (Ignition) simulator
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4'],
        output='screen'
    )

    # Spawn the robot in Gazebo using the URDF file
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fishbot', '-file', urdf_path],
        output='screen'
    )

    # Return the description of what to launch
    return LaunchDescription([
        gz_sim,
        spawn_robot,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

# this function have to be with exact name
def generate_launch_description():
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pkg_rp_arm'
    file_subpath = '/home/nadamamdouh/Robotics_proj/src/pkg_rp_arm/urdf/rp.urdf.xacro'  # change with ur xacro file path

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    rviz_config_path = '/home/nadamamdouh/Robotics_proj/src/pkg_rp_arm/rviz/view_urdf.rviz'

    declare_rviz_arg = DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        )
    # Configure the node
    node_rp_arm = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robot_description_raw}], # add other parameters here if required
    )
    run_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz"))
            )

    # Run the node
    return LaunchDescription([
        declare_rviz_arg,
        node_rp_arm,    
        run_rviz
    ])
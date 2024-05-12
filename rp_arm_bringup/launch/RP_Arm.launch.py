import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

# this function have to be with exact name
def generate_launch_description():
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pkg_rp_arm'
    file_subpath = '/home/nadamamdouh/Robotics_proj/src/pkg_rp_arm/urdf/rp.urdf.xacro'  # chance with ur xacro file path

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_rp_arm = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
#    node_script = Node(
#        package="pkg_rp_arm",
#        executable="exec_rp_arm",
#    )

    # Run the node
    return LaunchDescription([
        node_rp_arm
        #,node_script
    ])
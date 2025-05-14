from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_folder = get_package_share_directory('simplified_arm_assembly')
    xacro_file = share_folder + '/urdf/simplified_arm_assembly.urdf'
    robot_description = xacro.process_file(xacro_file).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ],  # add other parameters here if required
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    return LaunchDescription([
        robot_state_publisher_node,
    ])


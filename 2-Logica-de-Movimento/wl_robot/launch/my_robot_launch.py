from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'my_robot'
    
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'my_robot.urdf.xacro'
    )

    gazebo_world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'my_world.world'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )]),
            launch_arguments={'world': gazebo_world_path}.items()
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'controller_config.yaml')],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['left_wheel_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['right_wheel_controller'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

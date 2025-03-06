from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        # Specify the world file to load
        launch_arguments={'world': os.path.join(
            get_package_share_directory('hospital_world'), 'worlds', 'hospital_world.sdf')}.items(),
    )

    # Node to spawn the robot in the Gazebo simulation
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'hospital_robot', '-file', os.path.join(
            get_package_share_directory('hospital_robot'), 'urdf', 'robot.urdf')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Node to publish the robot description
    robot_description_path = os.path.join(
        get_package_share_directory('hospital_robot'), 'urdf', 'robot.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(robot_description_path).read()},
            {'use_sim_time': True}
        ]
    )


    # Return the launch description with all nodes
    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher_node,
    ])

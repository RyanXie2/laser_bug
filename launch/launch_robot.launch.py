import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command

import xacro



# robot_description = os.path.join(get_package_share_directory(
#         "laser_bug"), "description", "robot.urdf.xacro")
# robot_description_config = xacro.process_file(robot_description)
robot_description = os.path.join(get_package_share_directory(
        "laser_bug"), "description", "robot.urdf.xacro")
robot_description_config = xacro.process_file(robot_description)

controller_params = os.path.join(
        get_package_share_directory('laser_bug'), # <-- Replace with your package name
        'config',
        'my_controllers.yaml'
        )
def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='laser_bug' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    controller_abc = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params],
        output = "screen",
        )
    
    state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )
    #delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'laser_bug'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_spawner],
    #     )
    # )
    
    joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_broad_spawner],
    #     )
    # )

    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_abc,
        diff_drive_spawner,
        joint_broad_spawner,
        state_publisher
    ])

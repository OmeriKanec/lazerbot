import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import xacro


def generate_launch_description():

    pkg_control= get_package_share_directory('lazerbot_control')

    pkg_desc = get_package_share_directory('lazerbot_description')

    robot_controller = PathJoinSubstitution(
        [
            pkg_control,
            'config',
            'omni_wheel_drive_controller.yaml',
        ]
    )
    
    launch_rviz = LaunchConfiguration('launch_rviz')

    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='True',
        description='True if to launch rviz, false otherwise'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='cubes.sdf',
        description='Name of the Gazebo world file to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    xacro_file = os.path.join(
        pkg_desc,
        'urdf',
        'lazerbot.urdf.xacro'
    )

    rviz_config = os.path.join(
      pkg_desc,
      'config',
      'lazerbot.rviz'
    )

    gz_bridge_params_path = os.path.join(
        pkg_control,
        'config',
        'gz_bridge.yaml'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )



    robot_description_raw = xacro.process_file(xacro_file).toxml()


    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}],
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lazerbot_launch'), 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Спавним URDF из robot_description через ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lazerbot',
            '-topic', 'robot_description',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller",
                   '--param-file',
                    robot_controller,]
    )

    omni_base_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_base_controller_spawner]
        )
    )

    rviz_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=omni_base_controller_spawner,
            on_exit=[rviz_node]
        )
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controller,
            ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(launch_rviz_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_entity)
    launchDescriptionObject.add_action(joint_state_broadcaster_event_handler)
    launchDescriptionObject.add_action(omni_base_controller_event_handler)
    launchDescriptionObject.add_action(rviz_event_handler)
    launchDescriptionObject.add_action(joint_trajectory_controller_spawner)

    return launchDescriptionObject
    
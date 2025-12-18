import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    moveit_config = MoveItConfigsBuilder("lazerbot", package_name="lazerbot_moveit_config").to_moveit_configs()

    

    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz =  LaunchConfiguration('use_rviz')
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    publish_monitored_planning_scene = LaunchConfiguration("publish_monitored_planning_scene")
    capabilities = LaunchConfiguration("capabilities")
    disable_capabilities = LaunchConfiguration("disable_capabilities")

    declare_use_rviz_arg = DeclareLaunchArgument(
    name='use_rviz',
    default_value='true',
    description='Whether to start RViz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Whether to use sim time'
    )

    declare_allow_trajectory_execution_arg = DeclareLaunchArgument(
    name='allow_trajectory_execution',
    default_value='true',
    description=None
    )

    declare_publish_monitored_arg = DeclareLaunchArgument(
    name='publish_monitored_planning_scene',
    default_value='true',
    description=None
    )

    declare_capabilities_arg = DeclareLaunchArgument(
    name='capabilities',
    default_value=moveit_config.move_group_capabilities["capabilities"],
    description=None
    )

    declare_disable_capabilities_arg = DeclareLaunchArgument(
    name='disable_capabilities',
    default_value=moveit_config.move_group_capabilities["disable_capabilities"],
    description=None
    )

    move_group_configuration = {
    "publish_robot_description_semantic": True,
    "allow_trajectory_execution": allow_trajectory_execution,
    "capabilities": ParameterValue(capabilities, value_type=str),
    "disable_capabilities": ParameterValue(disable_capabilities, value_type=str),
    "publish_planning_scene": publish_monitored_planning_scene,
    "publish_geometry_updates": publish_monitored_planning_scene,
    "publish_state_updates": publish_monitored_planning_scene,
    "publish_transforms_updates": publish_monitored_planning_scene
    }
    move_group_params = {
    **moveit_config.to_dict(),
    **move_group_configuration
    }
    move_group_cmd = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[move_group_params,
            {'use_sim_time': use_sim_time}]
    )

    rviz_file = os.path.join(get_package_share_directory('lazerbot_moveit_config'), 'config', 'moveit.rviz')

    rviz_parameters = {
    **moveit_config.planning_pipelines,
    **moveit_config.robot_description_kinematics,
    **moveit_config.joint_limits,
    }
    rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    output='screen',
    respawn=False,
    arguments=['-d', rviz_file],
    parameters=[rviz_parameters, {'use_sim_time': use_sim_time}]
    )

    ld.add_action(declare_use_rviz_arg)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_allow_trajectory_execution_arg)
    ld.add_action(declare_publish_monitored_arg)
    ld.add_action(declare_capabilities_arg)
    ld.add_action(declare_disable_capabilities_arg)

    ld.add_action(move_group_cmd)
    ld.add_action(rviz_cmd)


    return ld
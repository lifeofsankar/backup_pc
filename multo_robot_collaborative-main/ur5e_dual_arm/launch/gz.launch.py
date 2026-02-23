import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('ur5e_dual_arm')
    ur_pkg = get_package_share_directory('ur_description')

    urdf_path = os.path.join(pkg, 'urdf', 'ur5e_dual_arm.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'empty_world.world')

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_path]),
        value_type=str
    )

    # allow Gazebo to find UR meshes
    gz_resource_path = os.path.dirname(ur_pkg)

    # publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen'
    )

    # spawn entity into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'dual_ur', '-z', '0.05'],
        output='screen'
    )

    # controllers (connect to Gazebo internal controller_manager)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    left_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', "-c", "/left/controller_manager"],
        output='screen'
    )

    right_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', "-c", "/right/controller_manager"],
        output='screen'
    )

    # correct order
    start_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )

    start_left = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[left_controller])
    )

    start_right = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[right_controller])
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        ExecuteProcess(cmd=['gz', 'sim', world_path, '-v', '4'], output='screen'),

        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),

        start_broadcaster,
        start_left,
        start_right,
    ])

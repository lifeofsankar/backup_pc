import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg    = get_package_share_directory('ur5e_dual_arm')
    ur_pkg = get_package_share_directory('ur_description')

    urdf_path  = os.path.join(pkg, 'urdf', 'ur5e_dual_arm.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'empty_world.world')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str
    )

    gz_resource_path = os.path.dirname(ur_pkg)

    # ── 1. robot_state_publisher ───────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
        output='screen'
    )

    # ── 2. Gazebo ──────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-v', '4'],
        output='screen'
    )

    # ── 3. Spawn robot — wait 5s for Gazebo to be ready ───────────────────
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'ur5e_dual_arm',
                    '-topic', '/robot_description',
                    '-x', '0', '-y', '0', '-z', '0.05',
                    '-R', '0', '-P', '0', '-Y', '0',
                ],
                output='screen'
            )
        ]
    )

    # ── 4. joint_state_broadcaster — wait for gz_ros2_control to init ─────
    spawn_jsb = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # ── 5. Arm controllers — after JSB is active ───────────────────────────
    spawn_left = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller', '-c', '/controller_manager'],
                output='screen'
            )
        ]
    )

    spawn_right = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_arm_controller', '-c', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        spawn_jsb,
        spawn_left,
        spawn_right,
    ])
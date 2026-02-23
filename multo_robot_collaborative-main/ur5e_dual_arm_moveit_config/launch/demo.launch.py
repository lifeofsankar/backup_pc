import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config_pkg = get_package_share_directory('ur5e_dual_arm_moveit_config')
    ur5e_pkg = get_package_share_directory('ur5e_dual_arm')
    
    # URDF path
    urdf_path = os.path.join(ur5e_pkg, 'urdf', 'ur5e_dual_arm.urdf.xacro')
    
    # Process xacro
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str
    )
    
    # Controllers
    controllers_yaml = os.path.join(ur5e_pkg, 'config', 'ros2_controllers.yaml')
    
    # SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'ur5e_dual_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    # ==========================================
    # robot_state_publisher
    # ==========================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ==========================================
    # controller_manager
    # ==========================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_yaml
        ]
    )

    # ==========================================
    # Controller spawners
    # ==========================================
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    left_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller'],
        output='screen'
    )
    
    right_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller'],
        output='screen'
    )

    # ==========================================
    # static TF launch
    # ==========================================
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'static_virtual_joint_tfs.launch.py')
        )
    )

    # ==========================================
    # move_group launch
    # ==========================================
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # ==========================================
    # RViz - FIXED VERSION
    # Don't pass robot_description directly
    # RViz reads it from /robot_description topic
    # ==========================================
    rviz_config = os.path.join(moveit_config_pkg, 'rviz', 'moveit.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            {
                'robot_description_semantic': robot_description_semantic_content
            }
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        left_arm_controller,
        right_arm_controller,
        static_tf_launch,
        move_group_launch,
        rviz,
    ])
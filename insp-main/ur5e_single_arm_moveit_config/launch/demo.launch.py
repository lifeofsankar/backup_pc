from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory("ur5e_single_arm_moveit_config")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[os.path.join(pkg, "config/moveit_controllers.yaml"),
                    os.path.join(pkg, "config/kinematics.yaml"),
                    os.path.join(pkg, "config/ompl_planning.yaml"),
                    os.path.join(pkg, "config/joint_limits.yaml"),
                    os.path.join(pkg, "config/pilz_cartesian_limits.yaml")]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg, "config/moveit.rviz")]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    return LaunchDescription([
        robot_state_pub,
        move_group,
        rviz
    ])

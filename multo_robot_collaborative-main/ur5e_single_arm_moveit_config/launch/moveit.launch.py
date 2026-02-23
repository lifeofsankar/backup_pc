from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory("ur5e_single_arm_moveit_config")

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen"
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                os.path.join(pkg, "config/kinematics.yaml"),
                os.path.join(pkg, "config/ompl_planning.yaml"),
                os.path.join(pkg, "config/moveit_controllers.yaml"),
            ]
        ),

    ])

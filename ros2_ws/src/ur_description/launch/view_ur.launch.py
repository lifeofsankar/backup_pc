from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []
    
    # 1. Update ur_type to default to ur5e
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        )
    )
    
    # 2. Point to your new dual-arm top-level file!
    # (Assuming you saved it in ur_description/urdf/dual_arm_gazebo.urdf.xacro for now)
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "dual_arm_gazebo.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_file = LaunchConfiguration("description_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # 3. Simplify the Command. We removed tf_prefix and name because 
    # the dual_arm xacro handles that internally now!
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", description_file,
            " ", "safety_limits:=", safety_limits,
            " ", "safety_pos_margin:=", safety_pos_margin,
            " ", "safety_k_position:=", safety_k_position,
            " ", "ur_type:=", ur_type,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Start the GUI slider node
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    # Publish the TF tree
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Start RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
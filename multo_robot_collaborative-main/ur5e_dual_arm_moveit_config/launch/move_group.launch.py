"""
MOVE_GROUP LAUNCH

This is the BRAIN of MoveIt!

move_group node:
- Loads SRDF, kinematics, planning configs
- Provides planning services
- Manages planning scene (collision objects)
- Executes trajectories via controllers

All planning requests go through move_group.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==========================================
    # PATHS TO ALL CONFIG FILES
    # ==========================================
    
    moveit_config_pkg = get_package_share_directory('ur5e_dual_arm_moveit_config')
    ur5e_pkg = get_package_share_directory('ur5e_dual_arm')
    
    # SRDF - semantic robot description
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'ur5e_dual_arm.srdf')
    
    # Kinematics - IK solver config
    kinematics_file = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    
    # Joint limits - velocity/acceleration
    joint_limits_file = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
    
    # OMPL - path planning algorithms
    ompl_planning_file = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
    
    # Controllers - execution interface
    controllers_file = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')
    
    # Pilz - cartesian limits
    pilz_file = os.path.join(moveit_config_pkg, 'config', 'pilz_cartesian_limits.yaml')

    # ==========================================
    # READ FILES INTO MEMORY
    # ==========================================
    
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics as dict under correct namespace
    kinematics_config = {
        'robot_description_kinematics': kinematics_file
    }
    
    # Joint limits as dict
    joint_limits_config = {
        'robot_description_planning': joint_limits_file
    }

    # ==========================================
    # MOVE_GROUP NODE - THE BRAIN
    # ==========================================
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            # Robot description (from robot_state_publisher topic)
            # Note: We don't set robot_description here
            # move_group reads it from /robot_description topic
            
            # SRDF - semantic description
            robot_description_semantic,
            
            # Kinematics solvers
            kinematics_config,
            
            # Joint limits
            joint_limits_config,
            
            # OMPL planning config
            ompl_planning_file,
            
            # Controller interface
            controllers_file,
            
            # Pilz cartesian limits
            pilz_file,
            
            # Additional settings
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                },
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            }
        ]
    )

    return LaunchDescription([
        move_group_node,
    ])
# ```

# # **Key concept:**
# # ```
# # move_group is like a restaurant kitchen:

# # Customer (your code): "I want to move to position X"
# #                      ↓
# # move_group (chef):  "Let me check what's possible"
# #                      ├─→ Check IK: can we reach X?
# #                      ├─→ Check collisions: path clear?
# #                      ├─→ Plan path: RRTConnect algorithm
# #                      └─→ Send to controller: execute!
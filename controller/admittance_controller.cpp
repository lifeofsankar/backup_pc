controller_interface::return_type DualArmAdmittanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Track elapsed time for the trajectory
  time_elapsed_ += period.seconds();
  double t = std::min(time_elapsed_, trajectory_duration_);
  double T = trajectory_duration_;

  // =========================================================
  // 1. MINIMUM JERK TRAJECTORY (For the Virtual Object)
  // =========================================================
  // Calculates smooth, jerk-free progression from 0.0 to 1.0
  double s = 10 * pow(t/T, 3) - 15 * pow(t/T, 4) + 6 * pow(t/T, 5);
  
  Eigen::VectorXd obj_current_target = obj_start_pose_ + (obj_goal_pose_ - obj_start_pose_) * s;

  // =========================================================
  // 2. COORDINATION CONSTRAINTS
  // =========================================================
  // Derive where the arms MUST be based on the object's position
  Eigen::VectorXd left_target = obj_current_target + left_grasp_offset_;
  Eigen::VectorXd right_target = obj_current_target + right_grasp_offset_;

  // =========================================================
  // 3. ADMITTANCE CONTROL (Force Feedback)
  // =========================================================
  // Read actual forces from the wrist FT sensors (F_ext)
  Eigen::VectorXd F_ext_left = read_ft_sensor("left");
  Eigen::VectorXd F_ext_right = read_ft_sensor("right");

  // Calculate Cartesian Error (Where we are vs Where we should be)
  Eigen::VectorXd error_left = left_target - current_cartesian_pose_left_;
  Eigen::VectorXd error_right = right_target - current_cartesian_pose_right_;

  // The Admittance Equation: a = M^-1 * (F_ext - D*v - K*e)
  // This makes the robot act like a spring, moving in proportion to the measured wrench
  Eigen::VectorXd accel_left = M_inv_ * (F_ext_left - D_ * cartesian_vel_left_ - K_ * error_left);
  Eigen::VectorXd accel_right = M_inv_ * (F_ext_right - D_ * cartesian_vel_right_ - K_ * error_right);

  // Integrate acceleration to get compliant target velocities
  cartesian_vel_left_ += accel_left * period.seconds();
  cartesian_vel_right_ += accel_right * period.seconds();

  // =========================================================
  // 4. INVERSE KINEMATICS & HARDWARE COMMAND
  // =========================================================
  // Get current Jacobian from your kinematics library (e.g., KDL)
  Eigen::MatrixXd J_left = get_jacobian("left");
  Eigen::MatrixXd J_right = get_jacobian("right");

  // Calculate pseudo-inverse: J_pinv = (J^T * J)^-1 * J^T
  Eigen::MatrixXd J_pinv_left = J_left.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd J_pinv_right = J_right.completeOrthogonalDecomposition().pseudoInverse();

  // Convert compliant Cartesian velocities to Joint velocities (q_dot = J_pinv * x_dot)
  Eigen::VectorXd joint_vel_left = J_pinv_left * cartesian_vel_left_;
  Eigen::VectorXd joint_vel_right = J_pinv_right * cartesian_vel_right_;

  // Send the calculated velocities directly to the hardware interfaces
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces_[left_joint_indices_[i]].set_value(joint_vel_left(i));
    command_interfaces_[right_joint_indices_[i]].set_value(joint_vel_right(i));
  }

  return controller_interface::return_type::OK;
}
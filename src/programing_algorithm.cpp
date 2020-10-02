#include <moveit_programing/programing_function.h>

void moveit_move::move_joint(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
{
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  group.setMaxVelocityScalingFactor(MaxVelScale);
  group.setMaxAccelerationScalingFactor(MaxAccScale);

  std::vector<double> joint_pose;

  joint_pose.push_back(joint1);
  joint_pose.push_back(joint2);
  joint_pose.push_back(joint3);
  joint_pose.push_back(joint4);
  joint_pose.push_back(joint5);
  joint_pose.push_back(joint6);

  group.setJointValueTarget(joint_pose);
  bool success = (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success == 1)
  {
    ROS_INFO("Robot arm motion success.");
  }
  else
  {
    ROS_INFO("Robot arm motion Fail, the robot arm joint over limit.");
  }
}

void moveit_move::move_pose(double x, double y, double z, double rx, double ry, double rz)
{
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  group.setMaxVelocityScalingFactor(MaxVelScale);
  group.setMaxAccelerationScalingFactor(MaxAccScale);

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);

  group.setPoseTarget(target_pose);
  bool success = (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success == 1)
  {
    ROS_INFO("Robot arm motion success.");
  }
  else
  {
    ROS_INFO("Robot arm motion Fail, the robot arm joint over limit.");
  }
}

void moveit_move::move_pose_cartesian(double x, double y, double z, double rx, double ry, double rz)
{
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  group.setMaxVelocityScalingFactor(MaxVelScale);
  group.setMaxAccelerationScalingFactor(MaxAccScale);

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  bool success = (group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success == 1)
  {
    ROS_INFO("Robot arm motion success.");
  }
  else
  {
    ROS_INFO("Robot arm motion Fail, the robot arm joint over limit.");
  }
}

std::vector<double> moveit_move::getJointValue()
{
  //double J1, J2, J3, J4, J5, J6;

  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  const moveit::core::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  moveit::core::RobotStatePtr current_state = group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  /*
  ROS_INFO("current robot arm joint pose: J1 = %.3f, J2 = %.3f, J3 = %.3f, J4 = %.3f, J5 = %.3f, J6 = %.3f", 
     joint_group_positions[0], joint_group_positions[1], joint_group_positions[2], joint_group_positions[3], 
     joint_group_positions[4], joint_group_positions[5]);
  */

  //J1 = joint_group_positions[0]; J2 = joint_group_positions[1]; J3 = joint_group_positions[2];
  //J4 = joint_group_positions[3]; J5 = joint_group_positions[4]; J6 = joint_group_positions[5];

  return joint_group_positions;
}

std::vector<double> moveit_move::getPoseValue()
{
  double pose_x, pose_y, pose_z;
  double rotation_x, rotation_y, rotation_z;

  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  group.setEndEffectorLink(ROBOT_ARM_END_TF);
  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();

  pose_x = current_pose.pose.position.x;
  pose_y = current_pose.pose.position.y;
  pose_z = current_pose.pose.position.z;

  // Orientation quaternion
  tf2::Quaternion q(current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y, 
                    current_pose.pose.orientation.z, 
                    current_pose.pose.orientation.w);

  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);

  // Roll Pitch and Yaw from rotation matrix
  m.getRPY(rotation_x, rotation_y, rotation_z);

  std::vector<double> robot_end_positions;

  robot_end_positions.push_back(pose_x);
  robot_end_positions.push_back(pose_y);
  robot_end_positions.push_back(pose_z);
  robot_end_positions.push_back(rotation_x);
  robot_end_positions.push_back(rotation_y);
  robot_end_positions.push_back(rotation_z);
  
  return robot_end_positions;
}

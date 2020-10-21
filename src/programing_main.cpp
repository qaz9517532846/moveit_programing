#include <moveit_programing/programing_function.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "programing_main", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /////// Set robot arm group //////

  moveit_move Mymoveit_move("manipulator");

  Mymoveit_move.ROBOT_ARM_END_TF = "tool0";

  ////// Set robot arm max vel scal and acc scale //////
  
  Mymoveit_move.MaxVelScale = 0.1;
  Mymoveit_move.MaxAccScale = 0.1;

  /////// programing start //////
  
  Mymoveit_move.move_joint(1, 0, 0, 0, 0, 0);

  sleep(1);
  
  Mymoveit_move.move_pose(0.5, 0.5, 0.5, 0, 0, 0);

  std::vector<double> robot_arm_position;

  robot_arm_position = Mymoveit_move.getJointValue();

  std::cout << robot_arm_position[0] << std::endl;

  std::vector<double> robot_arm_pose;

  robot_arm_pose = Mymoveit_move.getPoseValue();

  std::cout << robot_arm_pose[0] << std::endl;

  Mymoveit_move.move_pose_cartesian(0.6, 0, 0, 0, 0, 0);

  /////// programing END //////

  return 0;
}

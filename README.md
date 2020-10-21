# moveit_programing
The robot arm control motion using programing under MoveIt! environment. 

Software: Robot Operating System and MoveIt! .

Version: melodic.

# Install package.

Your PC need to install MoveIt.

``` bash
$ sudo apt-get install ros-melodic-moveit
```

# Using moveit_programing, take Universal Robot UR5 as an example.

Please open UR5 gazebo and moveit package.

``` bash
$ roslaunch ur_gazebo ur5.launch limited:=true
```

``` bash
$ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
```

``` bash
$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

# Start to using moveit_programing package.

``` bash
$ rosrun moveit_programing programing_main
```

The programing_main.cpp example.

``` bash
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
```

# illustration code function.

| Function                                               | DataType            | Description                                                               |
| ---                                                    | ---                 | ---                                                                       | 
| Mymoveit_move.PLANNING_GROUP                           | std::string         | Set robot arm planning name.                                              |
| Mymoveit_move.ROBOT_ARM_END_TF                         | std::string         | Set robot arm End-Effector TF.                                            |
| Mymoveit_move.MaxVelScale                              | double              | Set robot arm move max velocity scale.                                    |
| Mymoveit_move.MaxAccScale                              | double              | Set robot arm move max acceleration scale.                                |
| Mymoveit_move.move_joint(J1, J2, J3, J4, J5, J6)       | motion function     | The robot arm can move to designated joint position using joint mode.     |
| Mymoveit_move.move_pose(x, y, z, rx, ry, rz)           | motion function     | The robot arm can move to designated x-y-z position using joint mode.     |
| Mymoveit_move.move_pose_cartesian(x, y, z, rx, ry, rz) | motion function     | The robot arm can move to designated x-y-z position using cartesian mode. |
| Mymoveit_move.getJointValue()                          | std::vector<double> | Get robot arm current joint position.                                     |
| Mymoveit_move.getPoseValue()                           | std::vector<double> | Get robot arm current pose position.                                      ||

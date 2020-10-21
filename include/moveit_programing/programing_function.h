#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <ros/ros.h>

class moveit_move{
  public:
    std::string PLANNING_GROUP;
    std::string ROBOT_ARM_END_TF;

    double MaxVelScale = 0.5;
    double MaxAccScale = 0.5;

    moveit_move(std::string PLANNING_GROUP);
    ~moveit_move();

    void move_joint(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);
    void move_pose(double x, double y, double z, double rx, double ry, double rz);
    void move_pose_cartesian(double x, double y, double z, double rx, double ry, double rz);
    
    std::vector<double> getJointValue();
    std::vector<double> getPoseValue();

  private:
    moveit::planning_interface::MoveGroupInterface group;
    std::string planning_group;
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
};

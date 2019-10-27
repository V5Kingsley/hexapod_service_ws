#ifndef CONTROL_H
#define CONTROL_H

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/RPY.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <string>
#include <boost/format.hpp>
#include "gait.h"
#include "ik.h"
#include <actionlib/client/simple_action_client.h>
#include <hexapodservice/hexapodserviceAction.h>
#include "define.h"
#include <std_msgs/Int32.h>

class Control
{
public:
  actionlib::SimpleActionClient<hexapodservice::hexapodserviceAction> hexapod_client_;
  hexapodservice::hexapodserviceGoal maxpoints_goal_;
  hexapodservice::hexapodserviceGoal leg_goal_;
  hexapodservice::hexapodserviceGoal readall_goal_;
  void maxpoint_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result);
  void legcontrol_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result);
  void readall_goal_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result);

  Control(const std::string name, bool spin_thread) : hexapod_client_(name, spin_thread)
  {
    ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    ros::param::get("NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS);
    ros::param::get("MASTER_LOOP_RATE", MASTER_LOOP_RATE);
    ros::param::get("VELOCITY_DIVISION", VELOCITY_DIVISION);
    ros::param::get("STICK_FORCE", STICK_FORCE);
    ros::param::get("JOINT_NAME", joint_name_);

    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Control::cmd_velCallback, this);
    //发布的话题
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    boost::format roll;
    boost::format pitch1;
    boost::format pitch2;
    boost::format pitch3;
    for (int leg_index = 0, j = 1; leg_index < NUMBER_OF_LEGS; leg_index++, j++)
    {
      roll = boost::format("/hexapod/leg%d_roll_joint_position_controller/command") % j;
      pitch1 = boost::format("/hexapod/leg%d_pitch1_joint_position_controller/command") % j;
      pitch2 = boost::format("/hexapod/leg%d_pitch2_joint_position_controller/command") % j;
      pitch3 = boost::format("/hexapod/leg%d_pitch3_joint_position_controller/command") % j;
      leg_topic[leg_index] = roll.str();
      leg_topic[leg_index + 1] = pitch1.str();
      leg_topic[leg_index + 2] = pitch2.str();
      leg_topic[leg_index + 3] = pitch3.str();
      leg_roll_p[leg_index] = nh_.advertise<std_msgs::Float64>(leg_topic[leg_index], 10);
      leg_pitch1_p[leg_index] = nh_.advertise<std_msgs::Float64>(leg_topic[leg_index + 1], 10);
      leg_pitch2_p[leg_index] = nh_.advertise<std_msgs::Float64>(leg_topic[leg_index + 2], 10);
      leg_pitch3_p[leg_index] = nh_.advertise<std_msgs::Float64>(leg_topic[leg_index + 3], 10);
    }

    linear_x_max = 0.1;
    linear_y_max = 0.1;
    angular_z_max = 0.5;

    /*   //吸盘
  * 
    boost::format stick;
    for ( int leg_index = 0,  j=1; leg_index < NUMBER_OF_LEGS; leg_index++, j++ )
    {
      stick = boost::format( "leg%d_stick3" ) % j;
      force_to_stick[leg_index] = stick.str();
    }
    
    feet_position = nh_.advertise<hexapod_msgs::FeetPositions>("feet_position", 10); */
  }

  void partitionCmd_vel(geometry_msgs::Twist *cmd_vel); //把速度离散化？
  void publishJointStates(const hexapod_msgs::LegsJoints &legs, int &origin_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet);
  void robotInit();
  int MASTER_LOOP_RATE;              // Master loop rate
  hexapod_msgs::Pose body_;          // Body link rotation,没有用到
  hexapod_msgs::LegsJoints legs_;    //各个关节的角度信息
  hexapod_msgs::FeetPositions feet_; //足端轨迹
  geometry_msgs::Twist cmd_vel_;
  void feedDrives(const bool &start_cycle, const int &cycle_period_, const bool& is_traveling_);
  Gait gait;
  IK ik;

private:
  double VELOCITY_DIVISION;
  int NUMBER_OF_LEGS;       // Number of legs
  int NUMBER_OF_LEG_JOINTS; // Number of leg segments
  int STICK_FORCE;          //吸盘吸附力大小
  int SAMPLE_RATE;
  geometry_msgs::Twist cmd_vel_incoming_;

  // 订阅速度信息
  ros::Subscriber cmd_vel_sub_;
  void cmd_velCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg);

  // Node Handle
  ros::NodeHandle nh_;

  // 发布每个关节的角度话题
  std::string leg_topic[24];
  ros::Publisher leg_roll_p[6];
  ros::Publisher leg_pitch1_p[6];
  ros::Publisher leg_pitch2_p[6];
  ros::Publisher leg_pitch3_p[6];
  std_msgs::Float64 leg_roll[6];
  std_msgs::Float64 leg_pitch1[6];
  std_msgs::Float64 leg_pitch2[6];
  std_msgs::Float64 leg_pitch3[6];

  /* //吸盘吸附力Client
  std::string force_to_stick[6];
  ros::ServiceClient force_client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench srv;*/

  ros::Publisher feet_position;

  long long freeSpace_;
  long long minimumBufferFreeBytes_;
  long maxpoints_;
  bool motionActive_;

  ros::Publisher joint_state_pub_;
  sensor_msgs::JointState joint_states_;
  std::vector<std::string> joint_name_;

  double linear_x_max;
  double linear_y_max;
  double angular_z_max;

  void resetMotion(const hexapod_msgs::LegsJoints &smFbLegs);
  bool motionCorrect(const hexapod_msgs::LegsJoints &smFbLegs);
};

#endif

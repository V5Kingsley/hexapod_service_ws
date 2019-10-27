/***********************************************
 *        六足控制算法步态规划程序                 *
 *        Copyright (c) V5_Lab, 2018           *
 *        Author:    Kingsley                  *
 *        Version number:  0.00                *
 *        Date:                                *
 * *********************************************/
#include "gait.h"

Gait::Gait(void)
{
  ros::param::get("CYCLE_LENGTH", CYCLE_LENGTH_ORIGIN);
  ros::param::get("LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT);
  ros::param::get("LEG_GAIT_ORDER", cycle_leg_number_);
  ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
  ros::param::get("LINEAR_X_MAX", linear_x_max);
  ros::param::get("LINEAR_Y_MAX", linear_y_max);
  ros::param::get("ANGULAR_Z_MAX", angular_z_max);
  cycle_period_ = 0;
  period_seg = 0.3;
  legChangeFlag = true;
}


//摆动腿和支撑腿切换
void Gait::gaitCycle(const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet)
{
  
  double period_distance, period_height;

//六足运动一次分两个周期完成
//每个周期分三个部分，分别为抬腿、摆腿、放腿
//摆动腿第一个周期period_distance从0至0.5，第二个周期从-0.5至0；支撑腿则加负号即可
//两个周期后，六足处于初始姿态
  if (legChangeFlag)
  {
    if (cycle_period_ <= LIFT_LENGTH)
    {
      period_distance = 0;
      period_height = -0.5 * cos(M_PI * cycle_period_ / LIFT_LENGTH) + 0.5;
    }
    else if (cycle_period_ >= (CYCLE_LENGTH - DROP_LENGTH))
    {
      period_distance = 0.5;
      period_height = 0.5 * cos(M_PI * (cycle_period_ - (CYCLE_LENGTH - DROP_LENGTH)) / (DROP_LENGTH)) + 0.5;
    }
    else
    {
      period_distance = -0.25 * cos(M_PI * (cycle_period_ - LIFT_LENGTH) / MOVE_LENGTH) + 0.25; 
      period_height = 1;
    }
  }
  else
  {
    if (cycle_period_ <= LIFT_LENGTH)
    {
      period_distance = - 0.5;
      period_height = -0.5 * cos(M_PI * cycle_period_ / LIFT_LENGTH) + 0.5;
    }
    else if (cycle_period_ >= (CYCLE_LENGTH - DROP_LENGTH))
    {
      period_distance = 0;
      period_height = 0.5 * cos(M_PI * (cycle_period_ - (CYCLE_LENGTH - DROP_LENGTH)) / (DROP_LENGTH)) + 0.5;
    }
    else
    {
      period_distance = - 0.25 * cos(M_PI * (cycle_period_ - LIFT_LENGTH) / MOVE_LENGTH) - 0.25; 
      period_height = 1;
    }
  }

  //摆动腿和支撑腿歩幅控制
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    //摆动腿
    if (cycle_leg_number_[leg_index] == 1)
    {
      feet->foot[leg_index].position.x = cmd_vel.linear.x * period_distance;
      feet->foot[leg_index].position.y = cmd_vel.linear.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = cmd_vel.angular.z * period_distance;
    }
    //支撑腿
    if (cycle_leg_number_[leg_index] == 0)
    {
      feet->foot[leg_index].position.x = -cmd_vel.linear.x * period_distance;
      feet->foot[leg_index].position.y = -cmd_vel.linear.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -cmd_vel.angular.z * period_distance;
    }
  }

  cycle_period_++;

  if (cycle_period_ == CYCLE_LENGTH)
  {
    cycle_period_ = 0;
    std::reverse(cycle_leg_number_.begin(), cycle_leg_number_.end());  //一个周期结束后更换摆动腿组和支撑腿组
    legChangeFlag = !legChangeFlag;  //一次运动中的两个周期切换
  }
}


//周期长度调整，返回周期长度值
int Gait::getCycleLength(const geometry_msgs::Twist &cmd_vel)
{
  //周期调整,
  LIFT_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
  DROP_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
  double x_cycle = std::abs(cmd_vel.linear.x) / linear_x_max;
  double y_cycle = std::abs(cmd_vel.linear.y) / linear_y_max;
  double z_cycle = std::abs(cmd_vel.angular.z) / angular_z_max;
  double max_cycle = (x_cycle > y_cycle ? x_cycle : y_cycle) > z_cycle ? (x_cycle > y_cycle ? x_cycle : y_cycle) : z_cycle;
  MOVE_LENGTH = (1 - 2 * period_seg) * CYCLE_LENGTH_ORIGIN * max_cycle;
  CYCLE_LENGTH = LIFT_LENGTH + DROP_LENGTH + MOVE_LENGTH;
  return CYCLE_LENGTH;
}

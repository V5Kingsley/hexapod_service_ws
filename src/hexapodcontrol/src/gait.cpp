/*****************************************
 *                   六足控制算法步态规划程序                 *
 *                   Copyright (c) V5_Lab, 2018                  *
 *                   Author:                  Kingsley                  *
 *                   Version number:  0.00                         *
 *                   Date:                                                      *
 * ***************************************/
#include "gait.h"
static const double PI = 3.141592653;

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
  stop_cycle_ = 0;
  period_seg = 0.3;
  is_travelling_ = false;
  hexapod_stop_flag = false;
}

//每条摆动腿和支撑腿一个周期内的步幅控制
void Gait::cyclePeriod(const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet)
{
  double period_distance, period_height;

  /*停止复位*/
  if (stop_cycle_ == 1)
  {
    if (cycle_period_ <= LIFT_LENGTH)
    {
      period_distance = -1;
      period_height = -0.5 * cos(M_PI * cycle_period_ / LIFT_LENGTH) + 0.5;
    }
    else if (cycle_period_ >= (CYCLE_LENGTH - DROP_LENGTH))
    {
      period_distance = 0;
      period_height = 0.5 * cos(M_PI * (cycle_period_ - (CYCLE_LENGTH - DROP_LENGTH)) / (DROP_LENGTH)) + 0.5;
    }
    else
    {
      period_distance = -0.5 * cos(M_PI * (cycle_period_ - LIFT_LENGTH) / MOVE_LENGTH) - 0.5; //每条腿PI/CYCLE_LENGTH时间的步幅
      period_height = 1;
    }
  }

  /*正常行驶*/
  if (stop_cycle_ == 0 && start_cycle == 0)
  {
    if (cycle_period_ <= LIFT_LENGTH)
    {
      period_distance = -1;
      period_height = -0.5 * cos(M_PI * cycle_period_ / LIFT_LENGTH) + 0.5;
    }
    else if (cycle_period_ >= (CYCLE_LENGTH - DROP_LENGTH))
    {
      period_distance = 1;
      period_height = 0.5 * cos(M_PI * (cycle_period_ - (CYCLE_LENGTH - DROP_LENGTH)) / (DROP_LENGTH)) + 0.5;
    }
    else
    {
      period_distance = -cos(M_PI * (cycle_period_ - LIFT_LENGTH) / MOVE_LENGTH); //每条腿PI/CYCLE_LENGTH时间的步幅
      period_height = 1;
    }
  }

  /*起步调整位姿*/
  if (start_cycle == 1)
  {
    if (cycle_period_ <= LIFT_LENGTH)
    {
      period_distance = 0;
      period_height = -0.5 * cos(M_PI * cycle_period_ / LIFT_LENGTH) + 0.5;
    }
    else if (cycle_period_ >= (CYCLE_LENGTH - DROP_LENGTH))
    {
      period_distance = 1;
      period_height = 0.5 * cos(M_PI * (cycle_period_ - (CYCLE_LENGTH - DROP_LENGTH)) / (DROP_LENGTH)) + 0.5;
    }
    else
    {
      period_distance = -0.5 * cos(M_PI * (cycle_period_ - LIFT_LENGTH) / MOVE_LENGTH) + 0.5; //每条腿PI/CYCLE_LENGTH时间的步幅
      period_height = 1;
    }
  }

  //摆动腿和支撑腿歩幅控制
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    //摆动腿
    if (cycle_leg_number_[leg_index] == 1)
    {
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
    //支撑腿
    if (cycle_leg_number_[leg_index] == 0)
    {
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
    }
  }

  /*停止复位完成*/
  if (stop_cycle_ == 1 && cycle_period_ == (CYCLE_LENGTH - 1))
  {
    stop_cycle_ = 0;
    hexapod_stop_flag = true;  //六足处于停止时标志位，供control中使用
    is_travelling_ = false;
    ROS_INFO("robot stoped");
  }

  /*启动调整位姿完成*/
  if (start_cycle == 1 && cycle_period_ == (CYCLE_LENGTH - 1))
  {
    start_cycle = 0;
  }
}

//摆动腿和支撑腿切换
void Gait::gaitCycle(const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet)
{
  if (cycle_period_ == 0 || cycle_period_ == 1)  //新的周期才接收速度指令,调整周期。 cycle_period从0开始，为1时六足处于吸盘控制阶段
  {
    if (is_travelling_ == false && (std::abs(cmd_vel.linear.x) > 0.01 || std::abs(cmd_vel.linear.y) > 0.01 || std::abs(cmd_vel.angular.z) > 0.01))  //六足从停止开始运动时，置开始标志位为1
    {
      start_cycle = 1;
      ROS_INFO("start_cycle = 1");
    }
    else if (is_travelling_ == true && std::abs(cmd_vel.linear.x) < 0.001 && std::abs(cmd_vel.linear.y) < 0.001 && std::abs(cmd_vel.angular.z) < 0.001)   //六足停止运动时，置停止标志位为1
    {
      stop_cycle_ = 1;
      ROS_INFO("stop_cycle = 1");
    }
    else if (is_travelling_ == true && ((base.x - cmd_vel.linear.x) != 0 || (base.y - cmd_vel.linear.y) != 0 || (base.theta - cmd_vel.angular.z) != 0))   //六足速度改变时，置停止标志位为1
    {
      stop_cycle_ = 1;
      ROS_INFO("vel change stop_cycle = 1");
    }

    if (stop_cycle_ == 0)   //当六足不处于停止复位阶段时，进行步幅赋值。当六足处于停止复位阶段时，需执行完才将步幅赋为0
    {
      smooth_base_.x = base.x = cmd_vel.linear.x;
      smooth_base_.y = base.y = cmd_vel.linear.y;
      smooth_base_.theta = base.theta = cmd_vel.angular.z;
      ROS_INFO("smooth_base change");
    }

    //周期调整, 抬腿和落腿周期设为priod_seg*CYCLE_LENGTH_ORIGIN，移动腿时的周期与速度成比例关系，启动和停止时移动腿周期减少一半
    LIFT_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
    DROP_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
    if (start_cycle == 1 || stop_cycle_ == 1)
    {
      double x_cycle = 2 * std::abs(base.x) / linear_x_max;
      double y_cycle = 2 * std::abs(base.y) / linear_y_max;
      double z_cycle = 2 * std::abs(base.theta) / angular_z_max;
      double max_cycle = (x_cycle > y_cycle ? x_cycle : y_cycle) > z_cycle ? (x_cycle > y_cycle ? x_cycle : y_cycle) : z_cycle;
      MOVE_LENGTH = 0.5 * (1 - 2 * period_seg) * CYCLE_LENGTH_ORIGIN * max_cycle;
    }
    else
    {
      double x_cycle = 2 * std::abs(base.x) / linear_x_max;
      double y_cycle = 2 * std::abs(base.y) / linear_y_max;
      double z_cycle = 2 * std::abs(base.theta) / angular_z_max;
      double max_cycle = (x_cycle > y_cycle ? x_cycle : y_cycle) > z_cycle ? (x_cycle > y_cycle ? x_cycle : y_cycle) : z_cycle;
      MOVE_LENGTH = (1 - 2 * period_seg) * CYCLE_LENGTH_ORIGIN * max_cycle;
    }
    CYCLE_LENGTH = LIFT_LENGTH + DROP_LENGTH + MOVE_LENGTH;
  }

  //当步幅为0时，置is_travelling_为false，否则置为true
  if ((std::abs(smooth_base_.y) > 0.001) ||           // 1 mm
      (std::abs(smooth_base_.x) > 0.001) ||           // 1 mm
      (std::abs(smooth_base_.theta) > 0.00436332313)) // 0.25 degree
  {
    is_travelling_ = true;
  }
  else
  {
    is_travelling_ = false;
  }

//当is_travelling为真时，计算足端轨迹
  if (is_travelling_ == true)
  {
    //给下一个period/CYCLE_LENGTH足端歩幅
    cyclePeriod(smooth_base_, feet);
    cycle_period_++;
  }

  //一个周期结束后更换摆动腿组和支撑腿组
  if (cycle_period_ == CYCLE_LENGTH)
  {
    cycle_period_ = 0;
    std::reverse(cycle_leg_number_.begin(), cycle_leg_number_.end());
  }
}

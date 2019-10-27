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
  //  origin_period_ = 0;
  stop_cycle_ = 0;
  stop_cycle_start = 0;
  stop_finished = 0;
  vel_change_period = 0;
  period_seg = 0.3;
}

//每条摆动腿和支撑腿一个周期内的步幅控制
void Gait::cyclePeriod(const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet)
{
  double period_distance, period_height;

  /*停止复位*/
  if (stop_cycle_start == 1)
  {
    if (cycle_period_ == 0)
    {
      period_distance = 0;
      period_height = 0;
    }
    else
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
  }

  /*正常行驶*/
  if (stop_cycle_start == 0 && start_cycle == 0)
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
    if (cycle_leg_number_[leg_index] == 0)
    {
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
    //支撑腿
    if (cycle_leg_number_[leg_index] == 1)
    {
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
    }
  }

  /*停止复位完成*/
  if (stop_cycle_ == 1 && stop_cycle_start == 1 && cycle_period_ == 0)
  {
    stop_cycle_ = 0;
    stop_cycle_start = 0;
    stop_finished = 1;
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
  if ((base.x - cmd_vel.linear.x) != 0 || (base.y - cmd_vel.linear.y) != 0 || (base.theta - cmd_vel.angular.z) != 0)
  {

    /*当六足速度位0时，开始停止复位*/
    if (cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.angular.z == 0 && stop_cycle_start == 0 && stop_finished == 0)
    {
      stop_cycle_ = 1;
      //当六足走完上一个周期时，开始停止复位周期
      if (cycle_period_ == 1)
      {
        stop_cycle_start = 1;
      }
    }

    //如果六足不是处于停止复位周期时，则把速度值赋给base，否则要等待停止复位周期完成后才赋值，即赋0值
    if (stop_cycle_ == 0)
    {
      base.x = cmd_vel.linear.x;
      base.y = cmd_vel.linear.y;
      base.theta = cmd_vel.angular.z;
      stop_finished = 0;
    }

    /*当上一时刻六足速度为0时，启动调整位姿标志位赋1*/
    if (smooth_base_.x == 0 && smooth_base_.y == 0 && smooth_base_.theta == 0)
    {
      start_cycle = 1;
    }
  }

  //周期调整, 抬腿和落腿周期设为priod_seg*CYCLE_LENGTH_ORIGIN，移动腿时的周期与速度成比例关系，启动和停止时移动腿周期减少一半
  LIFT_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
  DROP_LENGTH = period_seg * CYCLE_LENGTH_ORIGIN;
  if (start_cycle == 1 || stop_cycle_start == 1)
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
  // ROS_INFO("%d, %d, %d", LIFT_LENGTH, MOVE_LENGTH, DROP_LENGTH);

  //如果前一时刻速度位0，则不需要处理速度变化
  if (smooth_base_.x == 0 && smooth_base_.y == 0 && smooth_base_.theta == 0)
  {
    smooth_base_.x = base.x;
    smooth_base_.y = base.y;
    smooth_base_.theta = base.theta;
  }
  else
  {
    //Low pass filter on the values to avoid jerky movements due to rapid value changes
    //     smooth_base_.x = base.x * 0.05 + ( smooth_base_.x * ( 1.0 - 0.05 ) );
    //     smooth_base_.y = base.y * 0.05 + ( smooth_base_.y * ( 1.0 - 0.05 ) );
    //     smooth_base_.theta = base.theta * 0.05 + ( smooth_base_.theta * ( 1.0 - 0.05 ) );

    //否则用三角函数处理速度变化
    if (std::abs(smooth_base_.x - base.x) > 0.00001 || std::abs(smooth_base_.y - base.y) > 0.00001 || std::abs(smooth_base_.theta - base.theta) > 0.00001)
    {
      if (vel_change_period == 0)
      {
        origin_base_.x = smooth_base_.x;
        origin_base_.y = smooth_base_.y;
        origin_base_.theta = smooth_base_.theta;
      }
      smooth_base_.x = -0.5 * (base.x - origin_base_.x) * cos(M_PI * vel_change_period / (CYCLE_LENGTH)) + 0.5 * (base.x + origin_base_.x);

      smooth_base_.y = -0.5 * (base.y - origin_base_.y) * cos(M_PI * vel_change_period / (CYCLE_LENGTH)) + 0.5 * (base.y + origin_base_.y);

      smooth_base_.theta = -0.5 * (base.theta - origin_base_.theta) * cos(M_PI * vel_change_period / (CYCLE_LENGTH)) + 0.5 * (base.theta + origin_base_.theta);

      vel_change_period++;
    }
    else
    {
      vel_change_period = 0;
    }
  }

  //六组速度为0时，直接设置为0
  if (base.x == 0 && base.y == 0 && base.theta == 0)
  {
    smooth_base_.x = 0;
    smooth_base_.y = 0;
    smooth_base_.theta = 0;
  }

  // Check to see if we are actually travelling
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
  if (is_travelling_ == true)
  {
    //给下一个period/CYCLE_LENGTH足端歩幅
    cyclePeriod(smooth_base_, feet);
    //    origin_period_++;
    //       cycle_period_ = -0.5 * CYCLE_LENGTH * cos(M_PI*origin_period_/CYCLE_LENGTH) + 0.5 * CYCLE_LENGTH;
    cycle_period_++;
    //  ROS_INFO("cycle_period_: %d", cycle_period_);
  }

  //一个周期结束后更换摆动腿组和支撑腿组
  if (cycle_period_ == CYCLE_LENGTH)
  {
    cycle_period_ = 0;
    //  origin_period_ = 0;
    std::reverse(cycle_leg_number_.begin(), cycle_leg_number_.end());
  }
}

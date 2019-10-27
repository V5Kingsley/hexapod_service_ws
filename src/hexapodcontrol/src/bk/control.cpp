/*****************************************
 *                   六足话题发布及客户端程序                  *
 *                   Copyright (c) V5_Lab, 2018                  *
 *                   Author:                  Kingsley                  *
 *                   Version number:  0.00                         *
 *                   Date:                                                      *
 * ***************************************/

#include "control.h"

/*Control::Control(const std::string name, bool spin_thread): hexapod_client_(name, spin_thread)
{
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  ros::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS );
  ros::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
   ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
   ros::param::get( "STICK_FORCE", STICK_FORCE );
   ros::param::get("SAMPLE_RATE", SAMPLE_RATE);
   setup_ = 1;
   
   
    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/cmd_vel", 1, &Control::cmd_velCallback, this );
    
    //发布的话题
    boost::format roll;
    boost::format pitch1;
    boost::format pitch2;
    boost::format pitch3;
    for ( int leg_index = 0,  j =1;  leg_index < NUMBER_OF_LEGS;  leg_index ++, j ++ )
    {
       roll = boost::format("/hexapod/leg%d_roll_joint_position_controller/command") % j;
       pitch1 = boost::format("/hexapod/leg%d_pitch1_joint_position_controller/command") % j;
       pitch2 = boost::format("/hexapod/leg%d_pitch2_joint_position_controller/command") % j; 
       pitch3 = boost::format("/hexapod/leg%d_pitch3_joint_position_controller/command") % j;
      leg_topic[leg_index] = roll.str();
      leg_topic[leg_index+1] = pitch1.str();
      leg_topic[leg_index+2] = pitch2.str();
      leg_topic[leg_index+3] = pitch3.str();
      leg_roll_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index], 10 );
      leg_pitch1_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+1], 10 );
      leg_pitch2_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+2], 10 );
      leg_pitch3_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+3], 10 );
    }

}*/

void Control::robotInit()
{
  //关节转动角度
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    leg_roll[leg_index].data = 0.0;
    leg_pitch1[leg_index].data = 0.0;
    leg_pitch2[leg_index].data = 0.0;
    leg_pitch3[leg_index].data = 0.0;
  }
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    leg_roll_p[leg_index].publish(leg_roll[leg_index]);
    leg_pitch1_p[leg_index].publish(leg_pitch1[leg_index]);
    leg_pitch2_p[leg_index].publish(leg_pitch2[leg_index]);
    leg_pitch3_p[leg_index].publish(leg_pitch3[leg_index]);
  }
}

void Control::publishJointStates(const hexapod_msgs::LegsJoints &legs, int &origin_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet)
{
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    leg_roll[leg_index].data = legs.leg[leg_index].coxa;
    leg_pitch1[leg_index].data = legs.leg[leg_index].femur;
    leg_pitch2[leg_index].data = legs.leg[leg_index].tibia;
    leg_pitch3[leg_index].data = legs.leg[leg_index].tarsus;
  }

  //发布关节角度话题
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    leg_roll_p[leg_index].publish(leg_roll[leg_index]);
    leg_pitch1_p[leg_index].publish(leg_pitch1[leg_index]);
    leg_pitch2_p[leg_index].publish(leg_pitch2[leg_index]);
    leg_pitch3_p[leg_index].publish(leg_pitch3[leg_index]);
  }

  //  feet_position.publish(*feet);

  /*   //吸盘吸附力控制
//   if ( origin_period_ == 1 )
//   {
// 
//      for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
//   {
// 
//      if (  cycle_leg_number_[leg_index] == 0)
//      {
//       srv.request.body_name = force_to_stick[leg_index];
//       srv.request.reference_frame = force_to_stick[leg_index];
//       srv.request.wrench.force.z = 0;
//       srv.request.duration.sec = 2;
//       force_client.call(srv);
//      }  
//      if (  cycle_leg_number_[leg_index] == 1 )
//     {
//       srv.request.body_name = force_to_stick[leg_index];
//       srv.request.reference_frame = force_to_stick[leg_index];
//       srv.request.wrench.force.z = - STICK_FORCE;
//       srv.request.duration.sec = 3;
//       force_client.call(srv);
//     }
//   }
//     ros::Duration(15).sleep(); 
   }*/
}

//订阅发布的速度信息
void Control::cmd_velCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg)
{
  if (cmd_vel_msg->linear.x > linear_x_max || cmd_vel_msg->linear.x < -linear_x_max)
  {
    ROS_FATAL("The linear.x exceeds the upper limit, set it to max: %f.", linear_x_max);
    cmd_vel_incoming_.linear.x = (cmd_vel_msg->linear.x > 0 ? linear_x_max : -linear_x_max);
  }
  else
  {
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
  }

  if (cmd_vel_msg->linear.y > linear_y_max || cmd_vel_msg->linear.y < -linear_y_max)
  {
    ROS_FATAL("The linear.y exceeds the upper limit, set it to max: %f.", linear_y_max);
    cmd_vel_incoming_.linear.y = (cmd_vel_msg->linear.y > 0 ? linear_y_max : -linear_y_max);
    ;
  }
  else
  {
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
  }

  if (cmd_vel_msg->angular.z > angular_z_max || cmd_vel_msg->angular.z < -angular_z_max)
  {
    ROS_FATAL("The angular.z exceeds the upper limit, set it to max: %f.", angular_z_max);
    cmd_vel_incoming_.angular.z = (cmd_vel_msg->angular.z > 0 ? angular_z_max : -angular_z_max);
  }
  else
  {
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
  }
}

//速度转化成歩幅
void Control::partitionCmd_vel(geometry_msgs::Twist *cmd_vel)
{
  // Instead of getting delta time we are calculating with a static division
  double dt = VELOCITY_DIVISION;

  double delta_th = cmd_vel_incoming_.angular.z * dt;
  double delta_x = (cmd_vel_incoming_.linear.x * cos(delta_th) - cmd_vel_incoming_.linear.y * sin(delta_th)) * dt;
  double delta_y = (cmd_vel_incoming_.linear.x * sin(delta_th) + cmd_vel_incoming_.linear.y * cos(delta_th)) * dt;
  cmd_vel->linear.x = delta_x;
  cmd_vel->linear.y = delta_y;
  cmd_vel->angular.z = delta_th;
}

//向服务器请求maxpoints，填充buffer
void Control::feedDrives(const bool &start_cycle, const int &origin_period_, const bool &is_traveling_)
{
  if ((cmd_vel_incoming_.linear.x != 0 || cmd_vel_incoming_.linear.y != 0 || cmd_vel_incoming_.angular.z != 0) || is_traveling_ == true)
  {
    //请求maxpoints
    maxpoints_goal_.MODE = MAXPOINT_REQUEST;
    bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));

    if (!server_exists)
    {
      ROS_WARN("Could not connect to hexapod server, retrying...");
      server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
    }
    hexapod_client_.sendGoal(maxpoints_goal_, boost::bind(&Control::maxpoint_doneCb, this, _1, _2));

    bool finished = hexapod_client_.waitForResult(ros::Duration(5.0));
    if (!finished)
    {
      ROS_WARN("Waiting for result...");
      finished = hexapod_client_.waitForResult(ros::Duration(5.0));
      if (!finished)
      {
        ROS_WARN("Connecting failed.");
        return;
      }
    }

    if (motionActive_ == false)
    { /*
    readall_goal_.MODE = READ_ALL_LEGS;
    bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
    if (!server_exists)
    {
      ROS_WARN("Could not connect to hexapod server, retrying...");
      server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
    }
    hexapod_client_.sendGoal(readall_goal_, boost::bind(&Control::readall_goal_doneCb, this, _1, _2));
    finished = hexapod_client_.waitForResult(ros::Duration(5.0));
    if (!finished)
    {
      ROS_WARN("Waiting for result...");
      finished = hexapod_client_.waitForResult(ros::Duration(5.0));
    }

    while (!motionActive_)
    {
      ROS_INFO("Waiting motion restarts...");
      hexapod_client_.sendGoal(maxpoints_goal_, boost::bind(&Control::maxpoint_doneCb, this, _1, _2));
      finished = hexapod_client_.waitForResult(ros::Duration(5.0));
      ros::Duration(1.0).sleep();
    }*/
      ros::shutdown();
      return;
    }

    if (maxpoints_ > 1 && freeSpace_ >= minimumBufferFreeBytes_) //当buffer空间充足时，填充buffer
    {
      ROS_INFO("maxpoints: %d", maxpoints_);
      //每个关节的填充buffer数值初始化
      for (int position_resize = 0; position_resize < 6; position_resize++)
      {
        leg_goal_.ALLLEGS.leg[position_resize].coxa.resize(maxpoints_);
        leg_goal_.ALLLEGS.leg[position_resize].femur.resize(maxpoints_);
        leg_goal_.ALLLEGS.leg[position_resize].tibia.resize(maxpoints_);
        leg_goal_.ALLLEGS.leg[position_resize].tarsus.resize(maxpoints_);
      }

      //调用maxpoints次算法
      for (int i = 0; i < maxpoints_; i++)
      {
        partitionCmd_vel(&cmd_vel_);
        gait.gaitCycle(cmd_vel_, &feet_);
        ik.calculateIK(feet_, &legs_);
        /*if (is_traveling_ == false)
        {
          ros::Duration(3).sleep();
          return;
        }*/
        for (int leg_index = 0; leg_index < 6; leg_index++)
        {
          leg_goal_.ALLLEGS.leg[leg_index].coxa[i] = round(4096 * (3005640.0 / 1300.0) * (legs_.leg[leg_index].coxa / M_PI * 180) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].femur[i] = round(4096 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].femur / M_PI * 180) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].tibia[i] = round(4096 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].tibia / M_PI * 180) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].tarsus[i] = round(4096 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].tarsus / M_PI * 180) / 360.0);
        }
      }

      //发送关节运动控制请求
      leg_goal_.MODE = ALLLEGS_CONTROL;
      leg_goal_.MAXPOINTS = maxpoints_;
      server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
      if (!server_exists)
      {
        ROS_WARN("could not connect to server, retrying...");
        server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
      }
      hexapod_client_.sendGoal(leg_goal_, boost::bind(&Control::legcontrol_doneCb, this, _1, _2));
      finished = hexapod_client_.waitForResult(ros::Duration(5.0));
      if (!finished)
      {
        ROS_WARN("Waiting for result...");
        finished = hexapod_client_.waitForResult(ros::Duration(5.0));
        if (!finished)
        {
          ROS_WARN("Connecting failed.");
          return;
        }
      }
    }
  }
}

//读取最大填充点回调函数
void Control::maxpoint_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Request maxpoint: server responded with state[%s]", state.toString().c_str());
  freeSpace_ = result->freespace;
  maxpoints_ = result->maxpoints;
  minimumBufferFreeBytes_ = result->minimumBufferFreeBytes;
  //   ROS_INFO("maxpoints: %d", maxpoints_);
  //   ROS_INFO("freespace: %d", freeSpace_);
  //   ROS_INFO("minimumBufferFreeBytes: %d", minimumBufferFreeBytes_);
  //   ROS_INFO("status: %d", result->status);
  if (result->status == 1 && result->motionActive == true)
  {
    motionActive_ = true;
  }
  else
  {
    motionActive_ = false;
  }
}

//发送关节控制运动控制回调函数，发布角度话题，与rviz同步
void Control::legcontrol_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Leg control: server responded with state[%s]", state.toString().c_str());
  /*if (result->status == 1)
  {
    motionActive_ = true;
  }
  else
  {
    motionActive_ = false;
  }*/

  /* for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    ROS_INFO("leg%d: %f, %f, %f, %f.", leg_index, result->ALLLEGS_fdbk.leg[leg_index].coxa, result->ALLLEGS_fdbk.leg[leg_index].femur, result->ALLLEGS_fdbk.leg[leg_index].tibia, result->ALLLEGS_fdbk.leg[leg_index].tarsus);
  }*/

  //publish joint_states
  joint_states_.header.stamp = ros::Time::now();
  int i = 0;
  joint_states_.name.resize(36);
  joint_states_.position.resize(36);
  for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = result->ALLLEGS_fdbk.leg[leg_index].coxa;
    i++;
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = -result->ALLLEGS_fdbk.leg[leg_index].femur;
    i++;
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = -result->ALLLEGS_fdbk.leg[leg_index].tibia;
    i++;
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = -result->ALLLEGS_fdbk.leg[leg_index].tarsus;
    i++;
    //吸盘
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = 0;
    i++;
    //吸盘
    joint_states_.name[i] = joint_name_[i];
    joint_states_.position[i] = 0;
    i++;
  }
  joint_state_pub_.publish(joint_states_);
}

void Control::readall_goal_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Read all motors: server responded with state[%s]", state.toString().c_str());
  hexapod_msgs::LegsJoints smFbLegs;
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    ROS_INFO("leg%d: %f, %f, %f, %f", leg_index, result->ALLLEGS_fdbk.leg[leg_index].coxa, result->ALLLEGS_fdbk.leg[leg_index].femur, result->ALLLEGS_fdbk.leg[leg_index].tibia, result->ALLLEGS_fdbk.leg[leg_index].tarsus);
    smFbLegs.leg[leg_index].coxa = result->ALLLEGS_fdbk.leg[leg_index].coxa;
    smFbLegs.leg[leg_index].femur = result->ALLLEGS_fdbk.leg[leg_index].femur;
    smFbLegs.leg[leg_index].tibia = result->ALLLEGS_fdbk.leg[leg_index].tibia;
    smFbLegs.leg[leg_index].tarsus = result->ALLLEGS_fdbk.leg[leg_index].tarsus;
  }
  resetMotion(smFbLegs);
}

void Control::resetMotion(const hexapod_msgs::LegsJoints &smFbLegs)
{
  while (!motionCorrect(smFbLegs))
  {
    gait.gaitCycle(cmd_vel_, &feet_);
    ik.calculateIK(feet_, &legs_);
  }
}

bool Control::motionCorrect(const hexapod_msgs::LegsJoints &smFbLegs)
{
  double coxaErr = std::abs(smFbLegs.leg[0].coxa - legs_.leg[0].coxa);
  double femurErr = std::abs(smFbLegs.leg[0].femur - legs_.leg[0].femur);
  double tibiaErr = std::abs(smFbLegs.leg[0].tibia - legs_.leg[0].tibia);
  double tarsusErr = std::abs(smFbLegs.leg[0].tarsus - legs_.leg[0].tarsus);
  ROS_INFO("%f, %f, %f, %f", coxaErr, femurErr, tibiaErr, tarsusErr);
  if (coxaErr < 0.02 && femurErr < 0.01 && tibiaErr < 0.01 && tarsusErr < 0.01)
    return true;
  else
    return false;
}

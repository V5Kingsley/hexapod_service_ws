/********************************************
 *          六足话题发布及客户端程序            *
 *          Copyright (c) V5_Lab, 2018      *
 *          Author:    Kingsley             *
 *          Version number:  0.00           *
 *          Date:                           *
 * *****************************************/

#include "control.h"

Control::Control(const std::string name, bool spin_thread) : hexapod_client_(name, spin_thread)
{
  ros::param::get("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
  ros::param::get("NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS);
  ros::param::get("MASTER_LOOP_RATE", MASTER_LOOP_RATE);
  ros::param::get("VELOCITY_DIVISION", VELOCITY_DIVISION);
  nh_.param<int>("VkBHexSM/sm_max_point_one_transmit", feedDriver_points_, 300);
  nh_.param<int>("VkBHexSM/sm_point_buf_size", sm_point_buf_size, 3000);
  ros::param::get("KPALIMIT", KPALIMIT);

  bufferFree_ = true;
  motionActive_ = true;
  stick_control_ = false;

  // Topics we are subscribing
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Control::cmd_velCallback, this);

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

  ros::param::get("LINEAR_X_MAX", linear_x_max);
  ros::param::get("LINEAR_Y_MAX", linear_y_max);
  ros::param::get("ANGULAR_Z_MAX", angular_z_max);

  feet_position = nh_.advertise<hexapod_msgs::FeetPositions>("feet_position", 10);

  ros::param::get("JOINT_NAME", joint_name);
  sm_pos_sub = nh_.subscribe<hexapodservice::leg>("/hexapod_sm_pose", 1, &Control::sm_pos_Cb, this);
  sm_pos_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

  stick_client_ = nh_.serviceClient<link_com::hexcom>("hexapod_st_service");
  heartbag_sub_ = nh_.subscribe<link_com::heartbag>("/hexapod_st_heartbag", 1, &Control::heartbag_Callback, this);

  robot_mode = hexapod_mode;
  hexapod_mode_sub = nh_.subscribe<std_msgs::Int32>("hexapod_mode_select", 1, &Control::hexapod_mode_cb, this);
  hexapod2crab_flag = false;
  crab2hexapod_flag = false;
  climb2wall_flag = false;
  ros::param::get("INIT_COXA_ANGLE", INIT_COXA_ANGLE);
  ros::param::get("BODY_RADIUS", BODY_RADIUS);
  ros::param::get("COXA_LENGTH", COXA_LENGTH);
  ros::param::get("FEMUR_LENGTH", FEMUR_LENGTH);
  ros::param::get("TIBIA_LENGTH", TIBIA_LENGTH);
  ros::param::get("TARSUS_LENGTH", TARSUS_LENGTH);
//符号位， 角度大于等于0时sign取1， 否则取-1
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    if (INIT_COXA_ANGLE[leg_index] >= 0)
    {
      sign[leg_index] = 1.0;
    }
    else
    {
      sign[leg_index] = -1.0;
    }
  }
    //存储关节角角度buffer初始化
  for (int i = 0; i < 24; i++)
  {
    posBuffer[i].reserve(3000);
  }
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

  if (robot_mode == crab_mode)
  {
    cmd_vel_incoming_.linear.x = 0;
    cmd_vel_incoming_.angular.z = 0;
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

//发布角度话题，gazebo仿真
void Control::publishJointStates(const hexapod_msgs::LegsJoints &legs, const int &origin_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet)
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

  ros::Duration(0.005).sleep();

  //  feet_position.publish(*feet);

#if 0
  //吸盘吸附力控制
  if (origin_period_ == 1)
  {

    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {

      if (cycle_leg_number_[leg_index] == 0)
      {
        srv.request.body_name = force_to_stick[leg_index];
        srv.request.reference_frame = force_to_stick[leg_index];
        srv.request.wrench.force.z = 0;
        srv.request.duration.sec = 2;
        force_client.call(srv);
      }
      if (cycle_leg_number_[leg_index] == 1)
      {
        srv.request.body_name = force_to_stick[leg_index];
        srv.request.reference_frame = force_to_stick[leg_index];
        srv.request.wrench.force.z = -STICK_FORCE;
        srv.request.duration.sec = 3;
        force_client.call(srv);
      }
    }
    ros::Duration(15).sleep();
  }
#endif
}

//六足螃蟹姿态和蜘蛛姿态模式订阅回调函数，当模式改变时，置相应的模式切换标志位为真
void Control::hexapod_mode_cb(const std_msgs::Int32ConstPtr &mode)
{
  if (mode->data != robot_mode)
  {
    if (mode->data == hexapod_mode)
      crab2hexapod_flag = (hexapod2crab_flag | climb2wall_flag) ? false : true;  //三个标志位只能有一个为真

    if (mode->data == crab_mode && robot_mode != climb2wall_mode)                 //当执行完爬墙程序时，机器人处于螃蟹姿态
      hexapod2crab_flag = (crab2hexapod_flag | climb2wall_flag) ? false : true;   
    if (mode->data == crab_mode && robot_mode == climb2wall_mode)                 //此时置模式为crab_mode时，直接改变不需执行变换程序
      robot_mode = crab_mode;

    if (mode->data == climb2wall_mode)
      climb2wall_flag = (hexapod2crab_flag | crab2hexapod_flag) ? false : true;
  }
}


//蜘蛛形态转为螃蟹形态控制函数
void Control::hexapod2crab_control()
{
  ROS_INFO("-----Hexapod2crab-----");
  hexapod_msgs::LegsJoints initLegs;
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    initLegs.leg[leg_index].coxa = 0;
    initLegs.leg[leg_index].femur = 0;
    initLegs.leg[leg_index].tibia = 0;
    initLegs.leg[leg_index].tarsus = 0;
  }
  
  geometry_msgs::Point initPos[6];
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    positionCalculate(leg_index, initLegs.leg[leg_index], initPos[leg_index]);
  }

  //计算螃蟹形态时的终止姿态
  geometry_msgs::Point finalPos[6];
  initLegs.leg[0].coxa = M_PI / 3.0;
  initLegs.leg[2].coxa = -M_PI / 3.0;
  initLegs.leg[3].coxa = M_PI / 3.0;
  initLegs.leg[5].coxa = -M_PI / 3.0;
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    positionCalculate(leg_index, initLegs.leg[leg_index], finalPos[leg_index]);
  }

  //还原关节角度为0
  initLegs.leg[0].coxa = 0;
  initLegs.leg[2].coxa = 0;
  initLegs.leg[3].coxa = 0;
  initLegs.leg[5].coxa = 0;

  double liftHeight = 0.1; //抬腿高度
  int cycle_length = 1800; //1800

  legAdjustOnGround(0, initPos[0], finalPos[0], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(2, initPos[2], finalPos[2], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(3, initPos[3], finalPos[3], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(5, initPos[5], finalPos[5], liftHeight, cycle_length, initLegs);

  ROS_INFO("Hexapod2crab finished. Now robot is in state of crab.");
}

//螃蟹形态转为蜘蛛形态控制函数
void Control::crab2hexapod_control()
{
  ROS_INFO("-----Crab2hexapod-----");
  hexapod_msgs::LegsJoints initLegs;
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    initLegs.leg[leg_index].coxa = 0;
    initLegs.leg[leg_index].femur = 0;
    initLegs.leg[leg_index].tibia = 0;
    initLegs.leg[leg_index].tarsus = 0;
  }
  initLegs.leg[0].coxa = M_PI / 3.0;
  initLegs.leg[2].coxa = -M_PI / 3.0;
  initLegs.leg[3].coxa = M_PI / 3.0;
  initLegs.leg[5].coxa = -M_PI / 3.0;

  
  geometry_msgs::Point initPos[6];
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    positionCalculate(leg_index, initLegs.leg[leg_index], initPos[leg_index]);
  }

  //计算螃蟹形态时的终止姿态
  geometry_msgs::Point finalPos[6];
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    initLegs.leg[leg_index].coxa = 0;
    initLegs.leg[leg_index].femur = 0;
    initLegs.leg[leg_index].tibia = 0;
    initLegs.leg[leg_index].tarsus = 0;
  }
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    positionCalculate(leg_index, initLegs.leg[leg_index], finalPos[leg_index]);
  }

  //还原关节角度为0
  initLegs.leg[0].coxa = M_PI / 3.0;
  initLegs.leg[2].coxa = -M_PI / 3.0;
  initLegs.leg[3].coxa = M_PI / 3.0;
  initLegs.leg[5].coxa = -M_PI / 3.0;;

  double liftHeight = 0.1; //抬腿高度
  int cycle_length = 1800; //1800

  legAdjustOnGround(0, initPos[0], finalPos[0], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(2, initPos[2], finalPos[2], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(3, initPos[3], finalPos[3], liftHeight, cycle_length, initLegs);
  legAdjustOnGround(5, initPos[5], finalPos[5], liftHeight, cycle_length, initLegs);
  ROS_INFO("Crab2hexapod finished. Now robot is in state of hexapod.");
}

//向服务器请求maxpoints，填充buffer
void Control::feedDrives(const int &cycle_period_, const bool &is_traveling_, std::vector<int> &cycle_leg_number_, bool &hexapod_stop_flag)
{
  //feedDriver条件： 所给的六足速度不为0或者给的速度为0但六足仍在运动（复位周期）
  if (std::abs(cmd_vel_incoming_.linear.x) > 0.001 || std::abs(cmd_vel_incoming_.linear.y) > 0.001 || std::abs(cmd_vel_incoming_.angular.z) > 0.001 || is_traveling_ == true)
  {
    if(robot_mode == climb2wall_mode)
      {
        ROS_INFO("Robot is in state of climb2wall. Please set speed to zero.");
        return;
      }

    //status不为1或者abortMotion时，关闭算法节点
    if (motionActive_ == false)
    {
      ros::shutdown();
      return;
    }

    if (bufferFree_) /*当buffer空间充足，即大于sm_max_point_one_transmit，填充buffer*/
    {
      maxpoints_ = feedDriver_points_; //固定每次填充buffer点数

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
        if(robot_mode == hexapod_mode)
        {
          hexapod_ik.calculateIK(feet_, &legs_);
        }
        else
        {
          crab_ik.calculateIK(feet_, &legs_);
        }
#if !MACHINE
        publishJointStates(legs_, cycle_period_, cycle_leg_number_, &feet_);
#endif
        for (int leg_index = 0; leg_index < 6; leg_index++)
        {
          leg_goal_.ALLLEGS.leg[leg_index].coxa[i] = round(4096.0 * (3005640.0 / 1300.0) * (legs_.leg[leg_index].coxa / M_PI * 180.0) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].femur[i] = round(4096.0 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].femur / M_PI * 180.0) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].tibia[i] = round(4096.0 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].tibia / M_PI * 180.0) / 360.0);
          leg_goal_.ALLLEGS.leg[leg_index].tarsus[i] = round(4096.0 * (3005640.0 / 1300.0) * (-legs_.leg[leg_index].tarsus / M_PI * 180.0) / 360.0);
        }

        if (cycle_period_ == 1)
        {
          maxpoints_ = i + 1;
          stick_control_ = true;
          break;
        }
      }
#if MACHINE
      //发送关节运动控制请求
      leg_goal_.MODE = ALLLEGS_CONTROL;
      leg_goal_.MAXPOINTS = maxpoints_;
      bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
      while (!server_exists) //查看服务器是否开启
      {
        ROS_WARN("could not connect to server, retrying...");
        server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
      }
      hexapod_client_.sendGoal(leg_goal_, boost::bind(&Control::legcontrol_doneCb, this, _1, _2));
      

      bool finished = hexapod_client_.waitForResult(ros::Duration(5.0));
      if (!finished) //未在规定时间发送成功时，关闭算法节点
      {
        ROS_WARN("Waiting for result...");
        finished = hexapod_client_.waitForResult(ros::Duration(5.0));
        if (!finished)
        {
          ROS_WARN("Connecting failed.");
          ros::shutdown();
          return;
        }
      }
#endif

#if STICK
      if (stick_control_)
      {
        ROS_INFO("------stick control-------");
        ROS_INFO("io: %d, %d, %d, %d, %d, %d", cycle_leg_number_[0], cycle_leg_number_[1], cycle_leg_number_[2], cycle_leg_number_[3], cycle_leg_number_[4], cycle_leg_number_[5]);
        ros::Duration(3).sleep();

#if MACHINE
        //发送请求确保六足buffer已接近空
        while (freeSpace_ < (sm_point_buf_size - 30))
        {
          maxpoints_goal_.MODE = MAXPOINT_REQUEST;

          bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
          while (!server_exists)
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
              ros::shutdown();
              return;
            }
          }
          ros::Duration(1).sleep();
        }
#endif

        ROS_INFO("---Buffer is almost empty. Start to control stick---");
        int Reqio[7];

        if (hexapod_stop_flag == true) /*当六足停止时，把所有吸盘都抽至真空*/
        {
          /*****把所有的吸盘与真空泵连接*****/
          stick_srv_.request.chose = 2;
          for (int i = 0; i < 6; i++)
          {
            stick_srv_.request.io[i] = 0;
            Reqio[i] = 0;
          }
          stick_srv_.request.io[6] = 1;
          while (!stick_client_.call(stick_srv_))
          {
            ROS_WARN("Failed to call stick service. Retrying...");
            ros::Duration(1).sleep();
          }
          ROS_INFO("%s", stick_srv_.response.back.c_str());

          ros::Duration(3).sleep();
          //查看吸盘是否吸放气完毕
          while (!isStickDone(Reqio))
          {
            ros::Duration(3).sleep();
          }
          ROS_INFO("All sticks are vacuumed.");

          if (hexapod_stop_flag == true)
          {
            partitionCmd_vel(&cmd_vel_); /*走完下一个点使得is_traveling 和smooth_base更新*/
            gait.gaitCycle(cmd_vel_, &feet_);
            
            while (hexapod_stop_flag == true) /*当新的速度到来且不为爬墙模式时才跳出循环，同时处理姿态改变*/
            {
              if (std::abs(cmd_vel_incoming_.linear.x) > 0.001 || std::abs(cmd_vel_incoming_.linear.y) > 0.001 || std::abs(cmd_vel_incoming_.angular.z) > 0.001)
              {
                if (robot_mode == climb2wall_mode)  //当处于爬墙模式时，不跳出循坏
                {
                  ROS_INFO("Robot is in state of climb2wall. Please set speed to zero.");
                  ros::Duration(1.0).sleep();
                  continue;
                }
                hexapod_stop_flag = false; //六足停止标志位复位
                break;
              }
              ros::Duration(1.0).sleep();
              ROS_INFO("hexapod stoped.");
        
              if (hexapod2crab_flag == true)  /*处理模式切换*/
              {
                hexapod2crab_control();
                robot_mode = crab_mode;
                hexapod2crab_flag = false;
              }
              if (crab2hexapod_flag == true)
              {
                crab2hexapod_control();
                robot_mode = hexapod_mode;
                crab2hexapod_flag = false;
              }
              if (climb2wall_flag == true)
              {
                robot_mode = climb2wall_mode;
                climb2wall_flag = false;
              }
            }
          }
        }

        /*****先把所有的吸盘与真空泵连接*****/
        stick_srv_.request.chose = 2;
        for (int i = 0; i < 6; i++)
        {
          stick_srv_.request.io[i] = 0;
          Reqio[i] = 0;
        }
        stick_srv_.request.io[6] = 1;
        while (!stick_client_.call(stick_srv_))
        {
          ROS_WARN("Failed to call stick service. Retrying...");
          ros::Duration(1).sleep();
        }
        ROS_INFO("%s", stick_srv_.response.back.c_str());

        ros::Duration(3).sleep();
        //查看吸盘是否吸放气完毕
        while (!isStickDone(Reqio))
        {
          ros::Duration(3).sleep();
        }
        ROS_INFO("All sticks are vacuumed.");

        /****把摆动腿的吸盘与真空泵阻塞****/
        stick_srv_.request.chose = 2;
        for (int i = 0; i < 6; i++)
        {
          stick_srv_.request.io[i] = cycle_leg_number_[i];
          Reqio[i] = stick_srv_.request.io[i];
        }
        stick_srv_.request.io[6] = 1;
        while (!stick_client_.call(stick_srv_))
        {
          ROS_WARN("Failed to call stick service. Retrying...");
          ros::Duration(1).sleep();
        }
        ROS_INFO("%s", stick_srv_.response.back.c_str());

        ros::Duration(6).sleep(); //吸盘放气等待时间
        //查看吸盘是否吸放气完毕
        while (!isStickDone(Reqio))
        {
          ros::Duration(3).sleep();
        }
        ROS_INFO("Stick control done. Ready to move.");

        stick_control_ = false; //吸盘控制标志位复位
      }
#endif
    }
    else /*buffer空间不足时，请求maxpoints，直至sm反馈buffer空间充足*/
    {
      maxpoints_goal_.MODE = MAXPOINT_REQUEST;

      bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
      while (!server_exists)
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
          ros::shutdown();
          return;
        }
      }
    }
  }
  else   /*当六足停止时，检查姿态变换标志位，是否需要进行螃蟹姿态和蜘蛛形态的切换*/
  {
    if (hexapod2crab_flag == true)
    {
      hexapod2crab_control();
      robot_mode = crab_mode;
      hexapod2crab_flag = false;
    }
    if (crab2hexapod_flag == true)
    {
      crab2hexapod_control();
      robot_mode = hexapod_mode;
      crab2hexapod_flag = false;
    }
    if(climb2wall_flag == true)
    {
      robot_mode = climb2wall_mode;
      climb2wall_flag = false;
    }

  }
}

//关节控制回调函数，确认六足status/freespace/motionActive
void Control::legcontrol_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Leg control: server responded with state[%s]", state.toString().c_str());
  motionActive_ = result->motionActive;
  status_ = result->status;
  freeSpace_ = result->freespace;

  if (status_ != 1)
  {
    ROS_WARN("simplemotion status fault, status: %d", status_);
    motionActive_ = false;
  }

  if (freeSpace_ < feedDriver_points_)
  {
    bufferFree_ = false;
  }
  else
  {
    bufferFree_ = true;
  }
}

//读取最大填充点回调函数
void Control::maxpoint_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Request maxpoint: server responded with state[%s]", state.toString().c_str());
  motionActive_ = result->motionActive;
  status_ = result->status;
  freeSpace_ = result->freespace;

  if (status_ != 1)
  {
    ROS_WARN("simplemotion status fault, status: %d", status_);
    motionActive_ = false;
  }

  if (freeSpace_ < feedDriver_points_)
  {
    bufferFree_ = false;
  }
  else
  {
    bufferFree_ = true;
  }
}

void Control::sm_pos_Cb(const hexapodservice::legConstPtr &leg)
{
  joint_states.header.stamp = ros::Time::now();
  int i = 0;
  joint_states.name.resize(30);
  joint_states.position.resize(30);
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = leg->leg[leg_index].coxa;
    i++;
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = -leg->leg[leg_index].femur;
    i++;
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = -leg->leg[leg_index].tibia;
    i++;
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = -leg->leg[leg_index].tarsus;
    i++;
    //吸盘
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = 0;
    i++;
  }
  sm_pos_pub.publish(joint_states);
}

/*心跳包订阅程序， 订阅吸盘气压值和io口*/
void Control::heartbag_Callback(const link_com::heartbagConstPtr &heartbag)
{
  kpa = heartbag->kpa;
  for (int i = 0; i < 7; i++)
  {
    io[i] = heartbag->io[i];
  }
}

/*吸盘控制， 当io口和气压值均满足条件时返回true*/
bool Control::isStickDone(const int Reqio[])
{
  for (int i = 0; i < 6; i++)
  {
    if (io[i] != Reqio[i])
    {
      ROS_FATAL("io is not correct!!");
      return false;
    }
  }

  if (kpa > KPALIMIT)
  {
    ROS_INFO("stick kpa is not enough.");
    return false;
  }

  return true;
}




/*平面调整位姿*/
void Control::legAdjustOnGround(const int leg_index, const geometry_msgs::Point &initPos, const geometry_msgs::Point &finalPos, const double liftHeight, const int cycle_length, hexapod_msgs::LegsJoints &legs)
{
  
  geometry_msgs::Point pos;
  double i = 0.0;

  for (int i = 0; i < 24; i++)
  {
    posBuffer[i].clear(); //清空缓存
  }

  while(i <= cycle_length)
  {
    interpolationOnGround(initPos, finalPos, liftHeight, i, cycle_length, pos); //插值
    jointCalculate(leg_index, pos, 0, legs.leg[leg_index]);                     //角度计算
    //存入缓存
    for (int leg_index = 0; leg_index < 6; leg_index++)
    {
      posBuffer[leg_index * 4].push_back(legs.leg[leg_index].coxa);
      posBuffer[leg_index * 4 + 1].push_back(legs.leg[leg_index].femur);
      posBuffer[leg_index * 4 + 2].push_back(legs.leg[leg_index].tibia);
      posBuffer[leg_index * 4 + 3].push_back(legs.leg[leg_index].tarsus);
    }
    if (i <= 1.0 / 3.0 * cycle_length || i >= 2.0 / 3.0 * cycle_length)
      i += 2.0;
    else
      i++;
  }
  publishTransformJointStates(leg_index);

}

/*******************************************************************************
*                      %从初始位姿到终止位姿插值%                                  *
*        输入：初始位姿、终止位姿、抬腿高度、cycle_period、周期长度、插值所得位姿        *
*                         输出： 插值所得位姿                                     *
********************************************************************************/
void Control::interpolationOnGround(const geometry_msgs::Point &initPos, const geometry_msgs::Point &finalPos, const double liftHeight, const double cycle_period, const int cycle_length, geometry_msgs::Point &outputPos)
{
  double sigmoid = 1.0 / (1.0 + exp(-25.0 * (float(cycle_period) / float(cycle_length) - 0.5)));
  outputPos.x = initPos.x + (finalPos.x - initPos.x) * sigmoid;
  outputPos.y = initPos.y + (finalPos.y - initPos.y) * sigmoid;
  double cosInterp = -0.5 * cos(2.0 * M_PI * cycle_period / cycle_length) + 0.5;
  outputPos.z = initPos.z + liftHeight * cosInterp;
}

/**********************************************************
*             %腿在墙上或者在地上时由位姿计算关节角%            *
*           输入： 墙或地、腿序、位姿、俯仰角、关节角度          *
*                       输出： 关节角度                      *
***********************************************************/
void Control::jointCalculate(const int leg_index, const geometry_msgs::Point &pos, const double roll_t, hexapod_msgs::LegJoints &leg)
{
  leg.coxa = atan2(pos.y - BODY_RADIUS * sin(INIT_COXA_ANGLE[leg_index]), pos.x - BODY_RADIUS * cos(INIT_COXA_ANGLE[leg_index])) - INIT_COXA_ANGLE[leg_index];

  double beta = 0.0;

  double A = pos.z + TARSUS_LENGTH * cos(beta - sign[leg_index] * roll_t);

  double B = (pos.y - BODY_RADIUS * sin(INIT_COXA_ANGLE[leg_index])) / sin(INIT_COXA_ANGLE[leg_index] + leg.coxa) - COXA_LENGTH - TARSUS_LENGTH * sin(beta - sign[leg_index] * roll_t);

  double C = FEMUR_LENGTH;

  double D = -TIBIA_LENGTH;

  double temp = 4.0 * pow(C, 2) * pow(D, 2) - pow((A * A + B * B - C * C - D * D), 2);
  temp = temp > 0 ? temp : 0;

  double x = 2.0 * atan(
                       (2.0 * A * C + sqrt(temp)) /
                       (A * A + pow((B + C), 2) - D * D));

  double y = 2.0 * atan(
                       (-2.0 * B * D - sqrt(temp)) /
                       (pow((A + D), 2) + B * B - C * C));

  leg.femur = x;
  leg.tibia = y - x;
  leg.tarsus = beta - sign[leg_index] * roll_t - y;

}


/************************************************
*          %姿态切换发布关节角度话题%                *
*                                                *
*************************************************/
void Control::publishTransformJointStates(const int leg_index)
{
  if(transformFeedDrviers(leg_index) != true)
    ros::shutdown();
  
}

/******************************************************
 *                 %由给定角度计算六足位姿%               *
 *                  输入： 腿序号、腿角度                 *
 *                  输出： 六足相应位姿                  *
 ******************************************************/
void Control::positionCalculate(const int leg_index, const hexapod_msgs::LegJoints &leg, geometry_msgs::Point &pos)
{
  pos.x = BODY_RADIUS * cos(INIT_COXA_ANGLE[leg_index]) +
          (COXA_LENGTH + FEMUR_LENGTH * cos(leg.femur) + TIBIA_LENGTH * sin(leg.femur + leg.tibia) + TARSUS_LENGTH * sin(leg.femur + leg.tibia + leg.tarsus)) * cos(INIT_COXA_ANGLE[leg_index] + leg.coxa);

  pos.y = BODY_RADIUS * sin(INIT_COXA_ANGLE[leg_index]) +
          (COXA_LENGTH + FEMUR_LENGTH * cos(leg.femur) + TIBIA_LENGTH * sin(leg.femur + leg.tibia) + TARSUS_LENGTH * sin(leg.femur + leg.tibia + leg.tarsus)) * sin(INIT_COXA_ANGLE[leg_index] + leg.coxa);

  pos.z = FEMUR_LENGTH * sin(leg.femur) - TIBIA_LENGTH * cos(leg.femur + leg.tibia) - TARSUS_LENGTH * cos(leg.femur + leg.tibia + leg.tarsus);
}


//姿态切换发送buffer至服务器控制函数
bool Control::transformFeedDrviers(const int leg_index)
{
#if MACHINE
  maxpointsRequest();  //请求maxpoint, 取得freeSpace

  while (freeSpace_ < (sm_point_buf_size - 30)) //sm服务器buffer接近空时，进行吸盘控制
  {
    ros::Duration(3).sleep();
    maxpointsRequest();
  }

  if (!motionActive_)
  {
    ROS_FATAL("Motion not active!");
    ros::shutdown();
    return false;
  }
#endif

#if STICK
  stickControl(leg_index);  //将相应的吸盘口关闭，放气
#endif

#if !MACHINE
  for (int i = 0; i < posBuffer[0].size(); i++)
  {
    for (int leg = 0; leg < 6; leg++)
    {
      leg_roll[leg].data = posBuffer[leg * 4][i];
      leg_pitch1[leg].data = posBuffer[leg * 4 + 1][i];
      leg_pitch2[leg].data = posBuffer[leg * 4 + 2][i];
      leg_pitch3[leg].data = posBuffer[leg * 4 + 3][i];
    }
    for (int leg = 0; leg < 6; leg++)
    {
      leg_roll_p[leg].publish(leg_roll[leg]);
      leg_pitch1_p[leg].publish(leg_pitch1[leg]);
      leg_pitch2_p[leg].publish(leg_pitch2[leg]);
      leg_pitch3_p[leg].publish(leg_pitch3[leg]);
    }
    ros::Duration(0.005).sleep();
  }
#else
  if(legControl() != true)  //将buffer发送至服务器
    return false;
#endif

#if STICK
  stickControl(6);  //将所有的吸盘口打开，均吸气
#endif

  return true;
}


void Control::maxpointsRequest()
{
  maxpoints_goal_.MODE = MAXPOINT_REQUEST;
  bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
  while(!server_exists)
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
      ros::shutdown();
    }
  }
}


//吸盘控制，0-5对应1-6腿的io口，对应的吸盘放气
//6代表将所有io口设为0，即所有吸盘均吸气
void Control::stickControl(const int leg_index)
{
  ROS_INFO("-----Stick Control-----");
  stick_srv_.request.chose = 2;
  int requestIO[7];

  if (leg_index == 6)
  {
    for (int i = 0; i < 6; i++)
    {
      stick_srv_.request.io[i] = 0;
      requestIO[i] = 0;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      if (i == leg_index)
      {
        stick_srv_.request.io[i] = 1;
        requestIO[i] = 1;
      }
      else
      {
        stick_srv_.request.io[i] = 0;
        requestIO[i] = 0;
      }
    }
  }
  stick_srv_.request.io[6] = requestIO[6] = 1;

  while (!stick_client_.call(stick_srv_))
  {
    ROS_WARN("Failed to call stick service. Retrying...");
    ros::Duration(2).sleep();
  }
  ROS_INFO("%s, request io: %d, %d, %d, %d, %d, %d", stick_srv_.response.back.c_str(), requestIO[0], requestIO[1], requestIO[2], requestIO[3], requestIO[4], requestIO[5]);

  //查看吸盘是否吸放气完毕
  while (!isStickDone(requestIO))
  {
    ros::Duration(3).sleep();
  }
  ROS_INFO("The control of all sticks are finished. ");

  if (leg_index != 6)
    ros::Duration(5).sleep(); //吸盘放气等待时间

  ROS_INFO("Hexapod is ready to move.");
}

bool Control::legControl()
{  
  leg_goal_.MODE = ALLLEGS_CONTROL;
  leg_goal_.MAXPOINTS = posBuffer[0].size();

  for (int i = 0; i < 6; i++)
  {
    leg_goal_.ALLLEGS.leg[i].coxa.resize(leg_goal_.MAXPOINTS);
    leg_goal_.ALLLEGS.leg[i].femur.resize(leg_goal_.MAXPOINTS);
    leg_goal_.ALLLEGS.leg[i].tibia.resize(leg_goal_.MAXPOINTS);
    leg_goal_.ALLLEGS.leg[i].tarsus.resize(leg_goal_.MAXPOINTS);
  }

  for (int i = 0; i < leg_goal_.MAXPOINTS; i++)
  {
    for (int leg_index = 0; leg_index < 6; leg_index++)
    {
      leg_goal_.ALLLEGS.leg[leg_index].coxa[i] = round(4096.0 * (3005640.0 / 1300.0) * (posBuffer[leg_index * 4][i] / M_PI * 180.0) / 360.0);
      leg_goal_.ALLLEGS.leg[leg_index].femur[i] = round(4096.0 * (3005640.0 / 1300.0) * (-posBuffer[leg_index * 4 + 1][i] / M_PI * 180.0) / 360.0);
      leg_goal_.ALLLEGS.leg[leg_index].tibia[i] = round(4096.0 * (3005640.0 / 1300.0) * (-posBuffer[leg_index * 4 + 2][i] / M_PI * 180.0) / 360.0);
      leg_goal_.ALLLEGS.leg[leg_index].tarsus[i] = round(4096.0 * (3005640.0 / 1300.0) * (-posBuffer[leg_index * 4 + 3][i] / M_PI * 180.0) / 360.0);
    }
  }

  bool server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
  while (!server_exists)
  {
    ROS_WARN("could not connect to server, retrying...");
    server_exists = hexapod_client_.waitForServer(ros::Duration(5.0));
  }

  hexapod_client_.sendGoal(leg_goal_, boost::bind(&Control::legcontrol_doneCb, this, _1, _2));

  bool finished = hexapod_client_.waitForResult(ros::Duration(5.0));
  if (!finished)
  {
    ROS_WARN("Waiting for result...");
    finished = hexapod_client_.waitForResult(ros::Duration(5.0));
    if (!finished)
    {
      ROS_FATAL("Connecting failed.");
      ros::shutdown();
      return false;
    }
  }
  return true;
}

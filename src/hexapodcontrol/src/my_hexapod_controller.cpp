#include <ros/ros.h>
#include "gait.h"
#include "ik.h"
#include "control.h"  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_hexapod_controller");
  Control control("hexapod_sm_service", true);

  ros::AsyncSpinner spinner(3); // Using 3 threads
  spinner.start();
  ros::Rate loop_rate(control.MASTER_LOOP_RATE);

  ros::Time last_time, current_time;

  //初始化机器人位姿
  while (ros::ok())
  {
   // last_time = ros::Time::now();
    control.feedDrives(control.gait.cycle_period_, control.gait.is_travelling_, control.gait.cycle_leg_number_, control.gait.hexapod_stop_flag);
    loop_rate.sleep();
   // current_time = ros::Time::now();
   // double d_sec = current_time.toSec() - last_time.toSec();
   // ROS_INFO("loop time: %f", d_sec);
  }

  return 0;
}

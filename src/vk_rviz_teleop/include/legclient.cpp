#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hexapodservice/hexapodserviceAction.h>
#include "define.h"

void doneCb(const actionlib::SimpleClientGoalState& state, const hexapodservice::hexapodserviceResultConstPtr& result)
{
   ROS_INFO("server responded with state[%s]", state.toString().c_str());
  ROS_INFO("simplemotion status is: %d", result->status);
  int i1 = result->position_fdbk[0];
  int i2 = result->position_fdbk[1];
  int i3 = result->position_fdbk[2];
  int i4 = result->position_fdbk[3];
  ROS_INFO("position feedback is: coxa[%d], femur[%d], tibia[%d], tarsus[%d]", i1, i2, i3, i4);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "onelegclient_node");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<hexapodservice::hexapodserviceAction> onelegclient("hexapodservice", true);
  hexapodservice::hexapodserviceGoal goal;
  char a;
  while(ros::ok())
  {
    
    goal.MODE = ONELEG_CONTROL;
    std::cout<<"input coxa: ";
    std::cin>>goal.ONELEG.coxa;
    std::cout<<"input femur: ";
    std::cin>>goal.ONELEG.femur;
    std::cout<<"input tibia: ";
    std::cin>>goal.ONELEG.tibia;
    std::cout<<"input tarsus: ";
    std::cin>>goal.ONELEG.tarsus;
    bool server_exists = onelegclient.waitForServer(ros::Duration(5.0));
    if (!server_exists)
  {
    ROS_WARN("Could not connect to hexapod service, retrying...");
    server_exists = onelegclient.waitForServer(ros::Duration(5.0));
  }
  onelegclient.sendGoal(goal, &doneCb);
  bool finished = onelegclient.waitForResult(ros::Duration(5.0));
  if (!finished)
  {
    ROS_WARN("giving up waiting on result");
  }
  std::cout<<"continue to control?(y/n)";
  std::cin>>a;
  if (a == 'n')
  {
    return 0;
  }
  }
  
  
}
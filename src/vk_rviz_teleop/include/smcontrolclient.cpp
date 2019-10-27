#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hexapodservice/hexapodserviceAction.h>
#include <std_msgs/Int32.h>
#include "define.h"

class smcontrolclient
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<hexapodservice::hexapodserviceAction> smcontrol_client;
  hexapodservice::hexapodserviceGoal goal;
  ros::Subscriber client_sub;
public:
  smcontrolclient();
  ~smcontrolclient(){};
  void callback(const std_msgs::Int32ConstPtr& mode_msg);
  static void doneCb(const actionlib::SimpleClientGoalState& state, const hexapodservice::hexapodserviceResultConstPtr& result);
};

smcontrolclient::smcontrolclient(): smcontrol_client("hexapodservice", true)
{
  ROS_INFO("simplemotion client started.");
  client_sub = nh_.subscribe("/simplemotion_mode", 1, &smcontrolclient::callback, this);
}

void smcontrolclient::callback(const std_msgs::Int32ConstPtr& mode_msg)
{
  goal.MODE = SIMPLEMOTION_CONTROL;
  switch(mode_msg->data){
    case OPENBUS:{
      goal.SIMPLEMOTION_MODE = OPENBUS;
    }
    break;
    case CLOSEBUS:{
      goal.SIMPLEMOTION_MODE = CLOSEBUS;
    }
    break;
    case ENABLEMOTOR:{
      goal.SIMPLEMOTION_MODE = ENABLEMOTOR;
    }
    break;
    case DISABLEMOTOR:{
      goal.SIMPLEMOTION_MODE = DISABLEMOTOR;
    }
    break;
    case GETSTATUS:{
      goal.SIMPLEMOTION_MODE = GETSTATUS;
    }
    break;
    case CLEARFAULT:{
      goal.SIMPLEMOTION_MODE = CLEARFAULT;
    }
    break;
    
    case ONELEGRESET:{
      goal.SIMPLEMOTION_MODE = ONELEGRESET;
    }
    break;
  }
  bool server_exists = smcontrol_client.waitForServer(ros::Duration(5.0));
  if (!server_exists)
  {
    ROS_WARN("Could not connect to hexapod server, retrying...");
    server_exists = smcontrol_client.waitForServer(ros::Duration(5.0));
  }
  smcontrol_client.sendGoal(goal, &smcontrolclient::doneCb);
  bool finished = smcontrol_client.waitForResult(ros::Duration(5.0));
  if(!finished)
  {
    ROS_WARN("giving up waiting on result");
  }
}

void smcontrolclient::doneCb(const actionlib::SimpleClientGoalState& state, const hexapodservice::hexapodserviceResultConstPtr& result)
{
  ROS_INFO("server responded with state[%s]", state.toString().c_str());
  const char* r = result->result.data();
  ROS_INFO("%s", r);
  ROS_INFO("simplemotion status is: %d", result->status);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smcontrol_client_node");
  smcontrolclient smcontrol_client;
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
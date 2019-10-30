#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hexapodservice/hexapodserviceAction.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapodservice/hexapodserviceAction.h>
#include "define.h"

class Pos_adjust
{
  private:
    ros::NodeHandle nh;
    int freeSpace;
    bool motionActive;
    int cycle_length;
    //客户端
    actionlib::SimpleActionClient<hexapodservice::hexapodserviceAction> hexapodClient;
    hexapodservice::hexapodserviceGoal legGoal;
    hexapodservice::hexapodserviceGoal maxpointsGoal;
    hexapod_msgs::LegsJoints finalLegs;

    void maxpoint_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result);

    void maxpointsRequest();

  public:
    Pos_adjust(const std::string name, bool spin_thread);
    void adjustment();
};

Pos_adjust::Pos_adjust(const std::string name, bool spin_thread):hexapodClient(name, spin_thread)
{
  finalLegs.leg[0].coxa = 1.0471975511965979;
  finalLegs.leg[0].femur = -0.038758520830718061;
  finalLegs.leg[0].tibia = -0.22365627124495968;
  finalLegs.leg[0].tarsus = 0.26241479207567775;

  finalLegs.leg[1].coxa = 0;
  finalLegs.leg[1].femur = -0.059724262471019439;
  finalLegs.leg[1].tibia = -0.16509683859324015;
  finalLegs.leg[1].tarsus = 0.22482110106425959;

  finalLegs.leg[2].coxa = -1.0471975511965979;
  finalLegs.leg[2].femur = -0.038758537257535017;
  finalLegs.leg[2].tibia = -0.22365627515359282;
  finalLegs.leg[2].tarsus = 0.26241481241112785;

  finalLegs.leg[3].coxa = 1.0471975511965979;
  finalLegs.leg[3].femur = -0.018897554015098201;
  finalLegs.leg[3].tibia = -0.24467115841576767;
  finalLegs.leg[3].tarsus = 0.26356871243086588;

  finalLegs.leg[4].coxa = 0;
  finalLegs.leg[4].femur = -0.036513660820203096;
  finalLegs.leg[4].tibia = -0.19001813422958463;
  finalLegs.leg[4].tarsus = 0.22653179504978774;

  finalLegs.leg[5].coxa = -1.0471975511965979;
  finalLegs.leg[5].femur = -0.018897537546665254;
  finalLegs.leg[5].tibia = -0.24467115439928608;
  finalLegs.leg[5].tarsus = 0.26356869194595134;

  cycle_length = 2000;
}

void Pos_adjust::maxpoint_doneCb(const actionlib::SimpleClientGoalState &state, const hexapodservice::hexapodserviceResultConstPtr &result)
{
  ROS_INFO("Request maxpoint: server responded with state[%s]", state.toString().c_str());
  motionActive = result->motionActive;
  int status = result->status;
  freeSpace = result->freespace;
  ROS_INFO("simple motion buffer free space: %d", freeSpace);

  if (status != 1)
  {
    ROS_WARN("simplemotion status fault, status: %d", status);
    motionActive = false;
  }
}

void Pos_adjust::maxpointsRequest()
{
  maxpointsGoal.MODE = MAXPOINT_REQUEST;

  bool server_exists = hexapodClient.waitForServer(ros::Duration(5.0));
  while (!server_exists)
  {
    ROS_WARN("Could not connect to hexapod server, retrying...");
    server_exists = hexapodClient.waitForServer(ros::Duration(5.0));
  }
  hexapodClient.sendGoal(maxpointsGoal, boost::bind(&Pos_adjust::maxpoint_doneCb, this, _1, _2));

  bool finished = hexapodClient.waitForResult(ros::Duration(5.0));
  while (!finished)
  {
    ROS_WARN("Waiting for result...");
    finished = hexapodClient.waitForResult(ros::Duration(5.0));
  }
}

void Pos_adjust::adjustment()
{
  maxpointsRequest();
  while (freeSpace < cycle_length)
  {
    ros::Duration(1).sleep();
    maxpointsRequest();
  }
  if (!motionActive)
  {
    ROS_FATAL("Motion not active!");
    ros::shutdown();
    return;
  }

  //发布关节角度
  legGoal.MODE = ALLLEGS_CONTROL;
  legGoal.MAXPOINTS = cycle_length;

  for (int i = 0; i < 6; i++)
  {
    legGoal.ALLLEGS.leg[i].coxa.resize(legGoal.MAXPOINTS);
    legGoal.ALLLEGS.leg[i].femur.resize(legGoal.MAXPOINTS);
    legGoal.ALLLEGS.leg[i].tibia.resize(legGoal.MAXPOINTS);
    legGoal.ALLLEGS.leg[i].tarsus.resize(legGoal.MAXPOINTS);
  }

  for (int i = 0; i <= cycle_length; i++)
  {
    for (int leg_index = 0; leg_index < 6; leg_index++)
    {
      legGoal.ALLLEGS.leg[leg_index].coxa[i] = round(4096.0 * (3005640.0 / 1300.0) * double(i)/double(cycle_length)*finalLegs.leg[leg_index].coxa / M_PI * 180.0 / 360.0);
      legGoal.ALLLEGS.leg[leg_index].femur[i] = - round(4096.0 * (3005640.0 / 1300.0) * double(i)/double(cycle_length)*finalLegs.leg[leg_index].femur / M_PI * 180.0 / 360.0);
      legGoal.ALLLEGS.leg[leg_index].tibia[i] = - round(4096.0 * (3005640.0 / 1300.0) * double(i)/double(cycle_length)*finalLegs.leg[leg_index].tibia / M_PI * 180.0 / 360.0);
      legGoal.ALLLEGS.leg[leg_index].tarsus[i] = - round(4096.0 * (3005640.0 / 1300.0) * double(i)/double(cycle_length)*finalLegs.leg[leg_index].tarsus / M_PI * 180.0 / 360.0);
    }
  }

  hexapodClient.sendGoal(legGoal, boost::bind(&Pos_adjust::maxpoint_doneCb, this, _1, _2));

  bool finished = hexapodClient.waitForResult(ros::Duration(5.0));

  while (!finished)
  {
    ROS_WARN("Waiting for result...");
    finished = hexapodClient.waitForResult(ros::Duration(5.0));
  }

  ROS_INFO("adjustment done");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_pos_adjust");
    Pos_adjust object("hexapod_sm_service", true);

    object.adjustment();

}

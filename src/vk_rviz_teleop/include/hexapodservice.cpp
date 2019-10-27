#include <ros/ros.h>
#include <simplemotion.h>
#include <hexapodservice/hexapodserviceAction.h>
#include <actionlib/server/simple_action_server.h>
#include <hexapodservice/legs.h>
#include <hexapodservice/legjoints.h>
#include "define.h"

class Hexapodservice
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<hexapodservice::hexapodserviceAction> as_;
  hexapodservice::hexapodserviceGoal goal_;
  hexapodservice::hexapodserviceResult result_;
  hexapodservice::hexapodserviceFeedback feedback_;
  hexapodservice::legjoints oneleg_;
  hexapodservice::legs legs_[6];
  smbus bushandle;
  smint32 Temp;
public:
  Hexapodservice();
  ~Hexapodservice(){};
  smint32 pulseout_[12];
  smint32 pulsefb_[12];
  void executeCB(const actionlib::SimpleActionServer<hexapodservice::hexapodserviceAction>::GoalConstPtr& goal);
  void singlejoint2motor(const float pos, smint32 *pulse);
  bool velControl(smint32 pulse[4], smint32 vel[4]);
  smint32 MaxVel;
};

Hexapodservice::Hexapodservice(): as_(nh_, "hexapodservice", boost::bind(&Hexapodservice::executeCB, this, _1), false)
{
  as_.start();
  ROS_INFO("hexapod service started.");
  bushandle = -1;
  pulseout_[4] = {0};
  MaxVel = 10000;
}

void Hexapodservice::singlejoint2motor(const float pos, smint32* pulse)
{
  *pulse = round(4096*(3005640.0/1300.0)*pos/360.0);
}

bool Hexapodservice::velControl(smint32* pulse, smint32* vel)
{
  smint32 max_pulse;
  max_pulse = pulse[0];
  float t = 1.0f;
  int i = 0;
  
  
  for (i = 1; i < 4; i ++)
  {
    max_pulse = (max_pulse >= pulse[i] ? max_pulse : pulse[i] );
  }
  
  if ( max_pulse == 0) 
  {
    return false;
  }
  else
  {
//     MaxVel = max_pulse / 0.1;
    t = max_pulse / (float)MaxVel;
    for(i = 0; i < 4; i ++)
    {
      vel[i] = pulse[i] / t;
    }
    
    return true;
  }
}


void Hexapodservice::executeCB(const actionlib::SimpleActionServer< hexapodservice::hexapodserviceAction >::GoalConstPtr& goal)
{
  //simplemotion 控制
  if (goal->MODE == SIMPLEMOTION_CONTROL)
  {
    switch(goal->SIMPLEMOTION_MODE)
    {
      case OPENBUS:{
	bushandle = smOpenBus("/dev/ttyUSB0");
	if(bushandle<0)
	{
	  ROS_WARN("Open bus error!");
	  result_.result = "Open bus error!";
	  as_.setAborted(result_);
	}
	else
	{
	  ROS_INFO("Open bus succeeded.");
	  result_.result = "Open bus succeeded.";
	  result_.status = getCumulativeStatus(bushandle);
	  ROS_INFO("simplemotion status: %d", result_.status);
	  as_.setSucceeded(result_);  
	}
      }
      break;
      
      case CLOSEBUS:{
	smCloseBus(bushandle);
	ROS_INFO("Closed bus.");
	result_.result = "Closed bus.";
	result_.status = getCumulativeStatus(bushandle);
	ROS_INFO("simplemotion status: %d", result_.status);
	as_.setSucceeded(result_);  
      }
      break;
      
      case ENABLEMOTOR:{
	for(int i=1; i<13; i++ )
	{
	  smSetParameter(bushandle, i, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
	  smSetParameter(bushandle, i, SMP_CONTROL_BITS2, SMP_CB2_ENABLE);
	}
	result_.status = getCumulativeStatus(bushandle);
	ROS_INFO("simplemotion status: %d", result_.status);
	if (result_.status == 1)
	{
	  result_.result = "enable motor finished";
	  as_.setSucceeded(result_);  
	}
	else
	{
	  result_.result = "enable motor failed";
	  as_.setAborted(result_);  
	}
      }
      break;
      
      case DISABLEMOTOR:{
	for(int i=1; i<5; i++ )
	{
	  smSetParameter(bushandle, i, SMP_CONTROL_BITS1, 0);
	  smSetParameter(bushandle, i, SMP_CONTROL_BITS2, 0);
	}
	 result_.result = "disable motor finished";
	 result_.status = getCumulativeStatus(bushandle);
	 as_.setSucceeded(result_);  
      }
      break;
      
      case GETSTATUS:{
	result_.status = getCumulativeStatus(bushandle);
	boost::format result_fm = boost::format("simplemotion status: %d") % result_.status;
	result_.result = result_fm.str();
	as_.setSucceeded(result_);
      }
      break;
      
      case CLEARFAULT:{
	for(int i=1; i<13; i++)
	{
	  smSetParameter(bushandle, i, SMP_FAULTS, 0);
	}
	result_.result = "clear fault  finished";
	result_.status = getCumulativeStatus(bushandle);
	as_.setSucceeded(result_);  
      }
      break;
      
      case ONELEGRESET:{
	for(int i=0; i<12; i++)
	{
	  smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_ABSOLUTE_POS_TARGET);
          smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, 0);
          smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_RETURN_PARAM_ADDR);
          smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_24B, SMP_ACTUAL_POSITION_FB);
          smExecuteCommandQueue(bushandle, i+1);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &pulsefb_[i]);
     //     result_.position_fdbk[i] = pulsefb_[i];
	}
	result_.status = getCumulativeStatus(bushandle);
        as_.setSucceeded(result_);
      }
      break;
      
    }
  }
  //控制一条腿
  if (goal->MODE == ONELEG_CONTROL)
  {
    smint32 pulse_diff[4];
    smint32 vel[4] = {0};
    for(int i = 0; i < 4; i ++)
    {
      pulse_diff[i] = pulseout_[i];
    }

    singlejoint2motor(goal->ONELEG.coxa, &pulseout_[0]);
    singlejoint2motor(goal->ONELEG.femur, &pulseout_[1]);
    singlejoint2motor(goal->ONELEG.tibia, &pulseout_[2]);
    singlejoint2motor(goal->ONELEG.tarsus, &pulseout_[3]);
    
    for(int i = 0; i < 4; i ++)
    {
      pulse_diff[i] =std::abs(pulseout_[i] - pulse_diff[i]);
    }
    if (velControl(pulse_diff, vel) == true)
    {
      for(int i=0; i<4; i++)
      {

	smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_TRAJ_PLANNER_VEL);
        smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, vel[i]);
	smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_ABSOLUTE_POS_TARGET);
        smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, pulseout_[i]);
        smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_RETURN_PARAM_ADDR);
        smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_24B, SMP_ACTUAL_POSITION_FB);
        smExecuteCommandQueue(bushandle, i+1);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
	smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &pulsefb_[i]);
	ROS_INFO("writing vel: %d", vel[i]);
        ROS_INFO("writing pulse: %d", pulseout_[i]);
        ROS_INFO("reading pulse: %d", pulsefb_[i]);
        result_.position_fdbk[i] = pulsefb_[i];
      }
      result_.status = getCumulativeStatus(bushandle);
      as_.setSucceeded(result_);
    }
    //不加速度控制
    else
    {
      for(int i=0; i<4; i++)
      {
	smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_ABSOLUTE_POS_TARGET);
        smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, pulseout_[i]);
        smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_RETURN_PARAM_ADDR);
        smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_24B, SMP_ACTUAL_POSITION_FB);
        smExecuteCommandQueue(bushandle, i+1);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &Temp);
        smGetQueuedSMCommandReturnValue(bushandle, &pulsefb_[i]);
        ROS_INFO("writing pulse: %d", pulseout_[i]);
        ROS_INFO("reading pulse: %d", pulsefb_[i]);
        result_.position_fdbk[i] = pulsefb_[i];
      }
      result_.status = getCumulativeStatus(bushandle);
      as_.setSucceeded(result_);
    }

  }
  
  //控制六条腿
  if(goal->MODE == ALLLEGS_CONTROL)
  {
    
  }
  
  //一条腿复位
  if(goal->MODE == ONELEG_RESET)
  {
    for(int i=0; i<4; i++)
    {
      smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_ABSOLUTE_POS_TARGET);
      smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, 0);
      smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_RETURN_PARAM_ADDR);
      smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_24B, SMP_ACTUAL_POSITION_FB);
      smExecuteCommandQueue(bushandle, i+1);
      smGetQueuedSMCommandReturnValue(bushandle, &Temp);
      smGetQueuedSMCommandReturnValue(bushandle, &Temp);
      smGetQueuedSMCommandReturnValue(bushandle, &Temp);
      smGetQueuedSMCommandReturnValue(bushandle, &pulsefb_[i]);
      result_.position_fdbk[i] = pulsefb_[i];
    }
    result_.status = getCumulativeStatus(bushandle);
    as_.setSucceeded(result_);
  }
  
  //六条腿复位
  if(goal->MODE == ALLLEGS_RESET)
  {
    
  }
  
  
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "hexapod_service_node");
  ROS_INFO("instantiating the hexapod service");
  Hexapodservice as_object;
  ROS_INFO("going in to spin");
  
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}

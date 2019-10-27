#include <ros/ros.h>
#include <hexapodservice/leg.h>
#include <vector>
#include <sensor_msgs/JointState.h>

class Hexapod_sm_pos
{  
private:
  ros::NodeHandle nh;
  ros::Subscriber sm_pos_sub;
  ros::Publisher sm_pos_pub;
  void sm_pos_Cb(const hexapodservice::legConstPtr &leg);
  std::vector<std::string> joint_name;
  sensor_msgs::JointState joint_states;

public:
  Hexapod_sm_pos();  
  ~Hexapod_sm_pos(){};
};

Hexapod_sm_pos::Hexapod_sm_pos()
{
  ros::param::get("JOINT_NAME", joint_name);
  sm_pos_sub = nh.subscribe<hexapodservice::leg>("/hexapod_sm_pose", 1, &Hexapod_sm_pos::sm_pos_Cb, this);
  sm_pos_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

void Hexapod_sm_pos::sm_pos_Cb(const hexapodservice::legConstPtr &leg)
{
  joint_states.header.stamp = ros::Time::now();
  int i = 0;
  joint_states.name.resize(36);
  joint_states.position.resize(36);
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
    //吸盘
    joint_states.name[i] = joint_name[i];
    joint_states.position[i] = 0;
    i++;
  }
  sm_pos_pub.publish(joint_states);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VkHexapod/sm_pos_publish");
  Hexapod_sm_pos object;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

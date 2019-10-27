#ifndef IK_H_
#define IK_H_

#include <cmath>
#include <ros/ros.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>

struct Trig
{
  double sine;
  double cosine;
};

class IK
{
protected:
  Trig getSinCos(double angle_rad);
  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS;

private:
  std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;


public:
  IK(void);
  virtual void calculateIK(const hexapod_msgs::FeetPositions &feet, hexapod_msgs::LegsJoints *legs) = 0;
  virtual ~IK(){}

};


class Hexapod_IK : public IK
{
private:
  std::vector<double> HEXAPOD_INIT_FOOT_POS_X, HEXAPOD_INIT_FOOT_POS_Y, HEXAPOD_INIT_FOOT_POS_Z;

public:
  Hexapod_IK();
  virtual void calculateIK(const hexapod_msgs::FeetPositions &feet, hexapod_msgs::LegsJoints *legs);
  virtual ~Hexapod_IK(){}
};

class Crab_IK : public IK
{
private:
  std::vector<double> CRAB_INIT_FOOT_POS_X, CRAB_INIT_FOOT_POS_Y, CRAB_INIT_FOOT_POS_Z;

public:
  Crab_IK();
  virtual void calculateIK(const hexapod_msgs::FeetPositions &feet, hexapod_msgs::LegsJoints *legs);
  virtual ~Crab_IK(){}
};

#endif

#include "solution.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "climb2wall");
  Solution Solution("hexapod_sm_service", true);
  ros::AsyncSpinner spinner(2);  //2线程，1线程也行？
  spinner.start();

  //六足关节初始化
  hexapod_msgs::LegsJoints legs;  //全局角度变量
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    legs.leg[leg_index].coxa = 0;
    legs.leg[leg_index].femur = 0;
    legs.leg[leg_index].tibia = 0;
    legs.leg[leg_index].tarsus = 0;
  }
  int cycle_length;         //每一步周期长度
  int prePress_cycle = 1200; //预压周期
  int meclErrRecover_cycle = 800;  //机械误差恢复周期

  float roll_t = 0.0;  //终止俯仰角
  float roll_0 = 0.0;  //初始俯仰角
  float prePress;  //预压距离

  /*****************平面调整位姿*********************************/ //stepCnt = 0
  //计算关节角度全为0时的初始位姿态
  geometry_msgs::Point initPos[6];
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    Solution.positionCalculate(leg_index, legs.leg[leg_index], initPos[leg_index]);
  }

  //计算螃蟹形态时的终止姿态
  geometry_msgs::Point finalPos[6];
  legs.leg[0].coxa = M_PI / 3.0;
  legs.leg[2].coxa = -M_PI / 3.0;
  legs.leg[3].coxa = M_PI / 3.0;
  legs.leg[5].coxa = -M_PI / 3.0;
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    Solution.positionCalculate(leg_index, legs.leg[leg_index], finalPos[leg_index]);
  }

  //还原关节角度为0
  legs.leg[0].coxa = 0;
  legs.leg[2].coxa = 0;
  legs.leg[3].coxa = 0;
  legs.leg[5].coxa = 0;

  double liftHeight = 0.1; //抬腿高度

  ///*调整成螃蟹姿态*///
  cycle_length = 2000; 
  Solution.legAdjustOnGround(0, initPos[0], finalPos[0], liftHeight, cycle_length, legs);
  //一腿预压///
  prePress = 0.015;
  //Solution.cyclePosPrePress(0, prePress, prePress_cycle, legs);
  Solution.prePress(0, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.legAdjustOnGround(2, initPos[2], finalPos[2], liftHeight, cycle_length, legs);
  //三腿预压///
  prePress = 0.015;
  //Solution.cyclePosPrePress(2, prePress, prePress_cycle, legs);
  Solution.prePress(2, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.legAdjustOnGround(3, initPos[3], finalPos[3], liftHeight, cycle_length, legs);
  //四腿预压///
  prePress = 0.020;
  //Solution.cyclePosPrePress(3, prePress, prePress_cycle, legs);
  Solution.prePress(3, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.legAdjustOnGround(5, initPos[5], finalPos[5], liftHeight, cycle_length, legs);
  //六腿预压///
  prePress = 0.020;
  //Solution.cyclePosPrePress(5, prePress, prePress_cycle, legs);
  Solution.prePress(5, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  /*********************抬第二腿前往后平移********************************/ //stepCnt = 4
  cycle_length = 2800;        //2800
  double translation = -0.1; //沿y轴平移-0.1
  double height = 0;         //沿z轴平移0
  Solution.publishRollTranslationLift(GROUND, roll_t, roll_0, translation, height, legs, cycle_length);
  Solution.stepCnt++;

  /******************抬第二腿************************/ //stepCnt = 5
  cycle_length = 3500;                                                     //3500
  Solution.legInitPos(1, legs.leg[1], 0, initPos[1]);                      //计算初始位姿
  double distance2Wall = 0.58 - 0.02;       //六足中心到墙体距离0.58,二腿预留0.02距离，在二腿预压时将此距离补上
  double givenPosZ = 0.24;                                                 //抬腿高度，相对于六足坐标系
  Solution.leftLeg2WallFinalPos(1, distance2Wall, givenPosZ, finalPos[1]); //腿上墙后的终止姿态
  Solution.leftLeg2Wall(1, initPos[1], finalPos[1], legs, cycle_length);   //腿上墙控制函数
  Solution.keepMeclErr(legs);   //让legs的角度保持当前的误差去计算下一步姿态
  Solution.stepCnt++;

  /*****************前移****************************/ //stepCnt = 6 
  cycle_length = 1800; //1800
  translation = 0.1;  //机体沿y轴平移0.1
  height = 0.0;
  Solution.publishRollTranslationLiftBut2(GROUND, roll_t, roll_0, translation, height, legs, cycle_length);
  //Solution.meclErrRecover(meclErrRecover_cycle, legs);
  //Solution.keepMeclErr(legs);   //让legs的角度保持当前的误差去计算下一步姿态
  Solution.stepCnt++; 

  /*********************前移后提升重心********************************/ //stepCnt = 7 //此步骤不修复误差，误差比例全置0
  cycle_length = 1800; //1800
  translation = 0;    //沿y轴平移0
  height = 0.04;      //沿z轴平移0.04
  Solution.publishRollTranslationLiftBut2(GROUND, roll_t, roll_0, translation, height, legs, cycle_length);

  //二腿预压///
  prePress = 0.015;
  Solution.stepCnt -= 2;   //legs的角度恢复上一步保持的误差值
  Solution.resetMeclErr(legs);   //legs的角度恢复上一步保持的误差值
  Solution.leg2SpecialPrePress(1, prePress, roll_t, prePress_cycle, legs);  //二腿预压特殊处理，修正了抬腿时预留的0.02距离
  Solution.meclErrRecover(meclErrRecover_cycle, legs);  //此处的误差回复的是上一步的误差
  Solution.stepCnt+=3;   //stepCnt恢复

  /******************抬第一腿************************/  //stepCnt = 8
  cycle_length = 3500; 
  Solution.legInitPos(0, legs.leg[0], 0, initPos[0]);
  givenPosZ = 0.15;     //抬腿高度
  distance2Wall = 0.58; //距离墙体距离
  Solution.leftLeg2WallFinalPos(0, distance2Wall, givenPosZ, finalPos[0]);
  Solution.leftLeg2Wall(0, initPos[0], finalPos[0], legs, cycle_length);
  //一腿预压///
  prePress = 0.015;
  Solution.prePress(0, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  /******************抬第三腿************************/  //stepCnt = 9
  cycle_length = 3500; //3500
  Solution.legInitPos(2, legs.leg[2], 0, initPos[2]);
  givenPosZ = 0.15;     //抬腿高度
  distance2Wall = 0.58; //距离墙体距离
  Solution.leftLeg2WallFinalPos(2, distance2Wall, givenPosZ, finalPos[2]);
  Solution.leftLeg2Wall(2, initPos[2], finalPos[2], legs, cycle_length);
  //三腿预压///
  prePress = 0.015;
  Solution.prePress(2, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;
  //std::cin>>y;

  /********************后三腿跨步*********************/ //stepCnt = 10
  cycle_length = 2000; //2800
  for (int leg_index = 3; leg_index < 6; leg_index++)
  {
    Solution.legInitPos(leg_index, legs.leg[leg_index], 0, initPos[leg_index]); //计算初始位姿
  }
  double stride = 0.1;                                                               //跨步距离
  liftHeight = 0.1;                                                                  //抬腿高度

  Solution.rightLegStride(4, stride, liftHeight, initPos[4], 0, legs, cycle_length);
  //五腿预压///
  prePress = 0.015;
  Solution.prePress(4, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.rightLegStride(3, stride, liftHeight, initPos[3], 0, legs, cycle_length); //右腿跨步函数控制
  //四腿预压///
  prePress = 0.015;
  Solution.prePress(3, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.rightLegStride(5, stride, liftHeight, initPos[5], 0, legs, cycle_length);
  //六腿预压///
  prePress = 0.015;
  Solution.prePress(5, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;
  //std::cin>>y;

  /************************第一次俯仰****************************/ //stepCnt = 13  
  cycle_length = 2800; //2800
  roll_0 = 0.0;
  roll_t = 45.0 / 180.0 * M_PI; //俯仰45°
  translation = 0.08;           //沿y轴平移0.05
  height = 0.0;
  Solution.publishRollTranslationLiftFirst45(WALL, roll_t, roll_0, translation, height, legs, cycle_length); //第一次俯仰45°时，平移的插值需要额外控制
  Solution.stepCnt++;
  //std::cin>>y;

  /***********************前三腿跨步******************************/ //stepCnt = 14
  cycle_length = 2800; //2800
  stride = 0.13;       //跨步距离
  liftHeight = 0.06;   //抬腿高度

  Solution.leftLegStride(1, stride, liftHeight, roll_t, cycle_length, legs);
  //二腿预压///
  prePress = 0.015;
  Solution.prePress(1, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(0, stride, liftHeight, roll_t, cycle_length, legs);
  //一腿预压///
  prePress = 0.015;
  Solution.prePress(0, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(2, stride, liftHeight, roll_t, cycle_length, legs);
  //三腿预压///
  prePress = 0.015;
  Solution.prePress(2, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;
  //std::cin>>y;

  /**************************提重心******************************/ //stepCnt = 17
  cycle_length = 2000; //2800
  translation = 0.1;   //沿y轴平移0.11
  height = 0.04;       //提升0.04
  roll_t = roll_0 = 45.0 / 180.0 * M_PI;
  Solution.publishRollTranslationLift(WALL, roll_t, roll_0, translation, height, legs, cycle_length);
  Solution.stepCnt++;
  // std::cin>>y;

  /**********************后三腿跨步*********************************/ //stepCnt = 18
  cycle_length = 2000; //2800
  stride = 0.25;       //跨步距离0.25
  liftHeight = 0.1;    //抬腿高度0.1

  Solution.leftLegStride(4, stride, liftHeight, roll_t, cycle_length, legs);
  //五腿预压///
  prePress = 0.015;
  Solution.prePress(4, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(3, stride, liftHeight, roll_t, cycle_length, legs);
  //四腿预压///
  prePress = 0.015;
  Solution.prePress(3, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(5, stride, liftHeight, roll_t, cycle_length, legs);
  //六腿预压///
  prePress = 0.015;
  Solution.prePress(5, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;
  // std::cin>>y;

  /**********************第二次俯仰****************************/  //stepCnt = 21
  cycle_length = 2800;          //2800
  roll_0 = 45.0 / 180.0 * M_PI; //俯仰前初始角45°
  roll_t = 90.0 / 180.0 * M_PI; //俯仰后角度90°
  translation = 0.14;           //沿y轴平移0.14
  height = -0.1;                //沿z轴平移-0.1
  Solution.publishRollTranslationLift(WALL, roll_t, roll_0, translation, height, legs, cycle_length);
  Solution.stepCnt++;
  // std::cin>>y;

  /**********************前三腿跨步***********************/  //stepCnt = 22
  cycle_length = 2800; //2800
  stride = 0.23;       //跨步距离0.23
  liftHeight = 0.1;    //抬腿高度0.1

  Solution.leftLegStride(1, stride, liftHeight, roll_t, cycle_length, legs);
  //二腿预压///
  prePress = 0.015;
  Solution.prePress(1, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(0, stride, liftHeight, roll_t, cycle_length, legs);
  //一腿预压///
  prePress = 0.015;
  Solution.prePress(0, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.leftLegStride(2, stride, liftHeight, roll_t, cycle_length, legs);
  //三腿预压///
  prePress = 0.015;
  Solution.prePress(2, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  /*********************重心上移****************************/
  /* roll_0 = 90.0 / 180.0 * M_PI;
  roll_t = 90.0 / 180.0 * M_PI;
  translation = 0.03; //沿y轴平移0.03， 即向上平移0.03
  height = 0;
  Solution.publishRollTranslationLift(WALL, roll_t, roll_0, translation, height, legs, cycle_length);*/

  /*********************后三腿上墙*************************/  //stepCnt = 25
  cycle_length = 4000; //4000
  //后三腿初始位姿
  for (int leg_index = 0; leg_index < 3; leg_index++)
  {
    Solution.legAfterRollPosCalculate(leg_index, roll_t, legs.leg[leg_index], initPos[leg_index]);
  }

  //前三腿初始位姿
  for (int leg_index = 3; leg_index < 6; leg_index++)
  {
    Solution.legAfterRollPosCalculate(leg_index, roll_t, legs.leg[leg_index], initPos[leg_index]);
  }

  //前三腿初始位姿y取反后即为后三腿上墙后的终止位姿， 一腿对应六腿， 二腿对应五腿， 三腿对应四腿
  initPos[2].y = -initPos[2].y;
  initPos[1].y = -initPos[1].y;
  initPos[0].y = -initPos[0].y;

  Solution.rightLeg2Wall(4, initPos[4], initPos[1], roll_t, legs, cycle_length);
  //五腿预压///
  prePress = 0.03;
  //Solution.cyclePosPrePress(4, prePress, prePress_cycle, legs);
  Solution.prePress(4, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.rightLeg2Wall(3, initPos[3], initPos[2], roll_t, legs, cycle_length); //右腿上墙
  //四腿预压///
  prePress = 0.015;
  //Solution.cyclePosPrePress(3, prePress, prePress_cycle, legs);
  Solution.prePress(3, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  Solution.rightLeg2Wall(5, initPos[5], initPos[0], roll_t, legs, cycle_length);
  //六腿预压///
  prePress = 0.015;
  //Solution.cyclePosPrePress(5, prePress, prePress_cycle, legs);
  Solution.prePress(5, prePress, prePress_cycle, legs);
  Solution.meclErrRecover(meclErrRecover_cycle, legs);
  Solution.stepCnt++;

  /***********************重心下移*************************/   //stepCnt = 28
  cycle_length = 2800;                                                                //2800
  double resetHeight = initPos[0].z + Solution.TIBIA_LENGTH + Solution.TARSUS_LENGTH; //六条腿都上墙后重心机体中心恢复初始状态
  roll_t = roll_0 = 0.0;
  height = resetHeight;
  translation = 0.0;
  Solution.publishRollTranslationLift(GROUND, roll_t, roll_0, translation, height, legs, cycle_length);
  Solution.stepCnt++;

#if 0
  /**********************还原六足初始姿态******************************/  //stepCnt = 29
  cycle_length = 2800; //2800
  //初始姿态
  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    Solution.positionCalculate(leg_index, legs.leg[leg_index], initPos[leg_index]);
  }

  //还原姿态
  hexapod_msgs::LegsJoints initLegs;
  for (int leg_index = 0; leg_index < 6; leg_index++)
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

  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    Solution.positionCalculate(leg_index, initLegs.leg[leg_index], finalPos[leg_index]);
  }

  liftHeight = 0.1; //抬腿高度

  for (int leg_index = 0; leg_index < 6; leg_index++)
  {
    if(leg_index > 3)
      prePress = 0.025;
    Solution.legAdjustOnGround(leg_index, initPos[leg_index], finalPos[leg_index], liftHeight, cycle_length, legs);
    //Solution.cyclePosPrePress(leg_index, prePress, prePress_cycle, legs); //预压
    Solution.prePress(leg_index, prePress, prePress_cycle, legs);
    Solution.meclErrRecover(meclErrRecover_cycle, legs);
    Solution.stepCnt++;
  }
#endif

  Solution.saveMeclBalance();
}

 #include <iostream>

#include "simplemotion.h"

int main(int argc, char** argv) {
     smbus bushandle;
    bushandle = smOpenBus("/dev/ttyUSB0");
    smint32 i = 0;
     smint32 v = 0;
    smint32 Temp;  
    char a;
    if (bushandle<0)
    {
      std::cout<<"open error";
      return 0;
    }
    smSetParameter(bushandle, 2, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);
    smSetParameter(bushandle, 2, SMP_CONTROL_BITS2, SMP_CB2_ENABLE);
             std::cout<<"input pulse:";     
          std::cin>>i;
    while(1)
    {

	  std::cout<<"input vel:";     
	  std::cin>>v;
	  
          smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_TRAJ_PLANNER_VEL);
          smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, v);
          smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_ABSOLUTE_POS_TARGET);
          smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_32B, i);
          smAppendSMCommandToQueue(bushandle, SM_SET_WRITE_ADDRESS, SMP_RETURN_PARAM_ADDR);
          smAppendSMCommandToQueue(bushandle, SM_WRITE_VALUE_24B, SMP_ACTUAL_POSITION_FB);
          smExecuteCommandQueue(bushandle, 2);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
          smGetQueuedSMCommandReturnValue(bushandle, &Temp);
      std::cout<<Temp<<std::endl;
//       std::cout<<"continue to control?(y/n)";
//      std::cin>>a;

    }  
    
    return 0;
}
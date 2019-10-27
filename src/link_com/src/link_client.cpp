#include "ros/ros.h"			
#include "link_com/hexcom.h"	//服务类型头文件

#include <std_msgs/String.h> 
#include <cstdlib>
#include <iostream>
#include <string>
using namespace std;


int main(int argc, char **argv)
{
        ros::init(argc, argv, "link_client");       //定义节点名
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<link_com::hexcom>("hexapod_st_service");   //定义服务名称   
        link_com::hexcom srv;       //定义服务消息对象

        char command_chose; 		//接收键盘输入，传递给服务的变量
        int  pwm_val;
        int  io_val[7]={0}	;
        
        while(ros::ok())
        {
	cout << "PWM      --> a"<<endl
	        << "IO       --> b"<<endl
	        << "feedback --> c"<<endl
	        << "open     --> d"<<endl
	        << "close    --> e"<<endl;
	cin >> command_chose;
	if(command_chose == 'a')			//修改pwm
	{
	        cout << "PWM :";
	        cin >> pwm_val;
	        if(pwm_val<=100&&pwm_val>=0) 
	        {
		srv.request.pwm = pwm_val;
		srv.request.chose = 1;
	        }
	        else
		cout <<"INPUT PWM ERROR"<<endl;
	        command_chose ='0';
	}
	else  if(command_chose == 'b')		//修改IO
	{
	        for(int i =0;i<7;i++)		//依次输入IO口状态
	        {
		cout << "io"<<i+1<<": ";
		cin >> io_val[i];
		if(io_val[i] == 0) 
		        srv.request.io[i] = 0;
		else
		        srv.request.io[i] = 1;
	        }
	        srv.request.chose = 2;
	}
	else if(command_chose == 'c')
	{
	         srv.request.chose = 3;		//请求反馈
	}
	else if(command_chose == 'd')
	{
	         srv.request.chose = 4;		//重开串口
	}
	else if(command_chose == 'e')
	{
	         srv.request.chose = 5;		//关闭串口
	}
	else
	        cout << "INPUT IO ERROR"<<endl;
	command_chose ='0';
	if (client.call(srv))       //呼叫服务
	{
	        cout << srv.response.back<<endl;
	        srv.request.chose = 0;   //清除命令标志位
	}
	else
	        ROS_ERROR("Failed to call service: link_service");
        }
        return 0;
}

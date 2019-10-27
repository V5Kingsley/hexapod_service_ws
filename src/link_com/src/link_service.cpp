#include "ros/ros.h"
#include <serial/serial.h> //ROS内置串口包
#include <std_msgs/String.h>
#include <string>
#include <cstring>
#include <iostream>
#include "link_com/hexcom.h"	 //服务格式文件
#include <link_com/heartbag.h> //话题格式文件

using namespace std;

//常量定义
#define BAUDRATE 115200
#define UART_NAME "/dev/hexapod_stm32"

enum conChoice
{
	PwmSet = 1,
	IoSet,
	SerialOpen,
	SerialClose,
	ReadFB,
};

//变量定义
serial::Serial ser;																			//声明串口对象
std_msgs::String heart_bag;															//存储接收数据
int pwm = 80;																								//存储命令变量
//char io[8] = {'0', '0', '0', '0', '0', '0', '0', '\0'}; //最后一位为截止符，帮助确定长度
int io[7] = {0};
char i2c[3] = {'0', '0', '\0'};
string command;		 //发送命令变量
bool msg_flag = 0; //发送信息标志位
float vin = 0;		 //输入电压值
float pout = 0;		 //输出压力值
//参数传递
std::string st_ttyusb;

//服务回调函数（接收主程序发来的命令，并发送到串口）
bool command_recv(link_com::hexcom::Request &req, link_com::hexcom::Response &res)
{
	if (req.chose == 1) //修改pwm
	{
		pwm = req.pwm; //获取pwm
		if (pwm >= 100)
		{
			pwm = 100;
			command = "VP100WK"; //发送该命令
		}
		else
		{
			i2c[0] = pwm / 10 + '0';
			i2c[1] = pwm % 10 + '0';							//转为字符串
			command = "VP0" + string(i2c) + "WK"; //2位数补成3位数*/
		}
		if (command.size() == 7) //pwm命令长为7
		{
			ser.write(command);			 //将命令写入串口
			res.back = "set pwm ok"; //返馈回主机
		}
		else
		{
			res.back = "set pwm fail"; //返馈回主机
		}
	}
	else if (req.chose == 2) //修改IO
	{
		for (int i = 0; i < 7; i++)
		{
		//	io[i] = req.io[i] + '0'; //数值转为字符串
		    io[i] = req.io[i]; //数值转为字符串
		}
		//command = "VP" + string(io) + "NK"; //char数组转string
		// if (command.size() == 11)						//io命令长为11
		// {
		// 	ser.write(command);			//发送命令
		// 	res.back = "set io ok"; //反馈客户端
		// }
		// else
		// {
		// 	res.back = "set io fail"; //反馈客户端
		// }
	}
	else if (req.chose == 3) //重置串口
	{
		if (ser.isOpen() != 1) //如果串口未打开
		{
			try
			{
				ser.setPort(UART_NAME);
				ser.setBaudrate(BAUDRATE);
				serial::Timeout to = serial::Timeout::simpleTimeout(1000); //设置超时时间
				ser.setTimeout(to);
				ser.open();
			}
			catch (serial::IOException &e)
			{
				res.back = "fail to open serial";
				return false;
			}
		}
		res.back = "open serial";
	}
	else if (req.chose == 4) //关闭串口
	{
		ser.close();
		res.back = "close serial";
	}
	else if (req.chose == 5) //请求反馈
	{
		command = "VFBK"; //请求反馈命令
		ser.write(command);
		res.back = "read to get feedback";
		msg_flag = 1; //反馈标志位置1
	}
	else
	{
		//cout<<"无匹配chose";
		return false;
	}
	command = '\0'; //清空命令
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "link_service");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("hexapod_st_service", command_recv);	//服务器接收数据 链接函数
	ros::Publisher heart_pub = nh.advertise<link_com::heartbag>("hexapod_st_heartbag", 1); //发布读取串口的数据 即心跳包
	ros::Rate r(20);																																			 //设置循环频率，主要为发布话题频率
	link_com::heartbag hb;

	//从参数服务器读取节点参数
	ros::param::get("~stm32_ttyusb_name", st_ttyusb);
	if (st_ttyusb != "")
	{
		ROS_INFO("~stm32_ttyusb_name = %s", st_ttyusb.c_str());
	}
	else
	{
		st_ttyusb = UART_NAME;
		ROS_INFO("~stm32_ttyusb_name = %s", st_ttyusb.c_str());
	}

	/*try //尝试打开串口
	{
		//设置串口属性，并打开串口
		ser.setPort(st_ttyusb.c_str());
		ser.setBaudrate(BAUDRATE);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); //设置超时时间
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException &e)
	{
		//cout<<"please give chmod "<<endl;
		ROS_INFO_STREAM("please give chmod");
		return -1;
	}
	//检测串口是否已经打开，并给出提示信息
	if (ser.isOpen())
	{
		//cout<<"Serial init"<<endl;
		//cout<<"ready to r&t"<<endl;
		ROS_INFO_STREAM("Serial init, ready to r&t");
	}
	else
	{
		return -1;
	}*/
	while (ros::ok())
	{
		if (1)
		{
			if (1)
			{
				// heart_bag.data = ser.read(ser.available());
				// if (heart_bag.data.size() > 24 && heart_bag.data[15] == 'V' && heart_bag.data[20] == 'E' && heart_bag.data[0] == 'P' && heart_bag.data[6] == 'P') //数据格式符合要求
				// {
				// 	for (uint8_t i = 0; i < 7; i++)
				// 	{
				// 		if (heart_bag.data[8 + i] == '0')
				// 		{
				// 			hb.io[i] = 0;
				// 		}
				// 		else if (heart_bag.data[8 + i] == '1')
				// 		{
				// 			hb.io[i] = 1;
				// 		}
				// 		else
				// 		{
				// 			hb.io[i] = 0xff;
				// 		}
				// 	}
				// 	vin = (heart_bag.data[16] - '0') + (heart_bag.data[17] - '0') * 0.1 + (heart_bag.data[18] - '0') * 0.01 + (heart_bag.data[19] - '0') * 0.001;
				// 	//pout =float( vin* 50-24) ;
				// 	hb.kpa = -51.8606 * vin + 23.8596;
				for(int i = 0; i < 7; i++)
				{
					hb.io[i] = io[i];
				}
				hb.kpa = -pwm;
					heart_pub.publish(hb); //发布心跳包
				
				heart_bag.data.clear(); //变量清零
			}
			else
			{
				//请求反馈命令
				ser.write("VFBK");
			}
		}
		else
		{
			ROS_INFO_STREAM("serial can not be used!");
			sleep(1);
		}
		ros::spinOnce(); //进入回调函数 观察是否接收命令
		r.sleep();			 //发布频率控制
	}
	return 0;
}

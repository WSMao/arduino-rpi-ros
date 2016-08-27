#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"i_robot/ir_msg.h"                   //using the message created by myself
#include "i_robot/line_follower.h"       //defintion of line_follower.cpp
#include <cstddef>                                //for transformation of int type like uint16_t

#include <i_robot/button_msg.h>

bool     ir_digital[5] ;
uint16_t    ir_analog[5] ;
using namespace std;
void ArduinoMegaCallback(const i_robot::ir_msg&  msg)
{
  ROS_INFO_ONCE("i heard ir_sensor:");
//get ir data
  for(int i=0;i<5;i++){
      ir_digital[i]= msg.ir_digital[i];
      ir_analog[i]=msg.ir_analog[i];
//      ROS_INFO("%d",ir_digital[i]);
//      ROS_INFO("%d",ir_analog[i]);
  }
ROS_INFO("  %d  %d  %d  %d  %d  ",ir_digital[0], ir_digital[1], ir_digital[2], ir_digital[3], ir_digital[4]);
ROS_INFO("%d",ir_analog[0]);
//cout<<"ll:  "<<ir_digital[0]<<"ml"<<ir_digital[1]<<"mm"<<ir_digital[2]<<"mr"<<ir_digital[3]<<"rr"<<ir_digital[4]<<endl;
 // if(ir_digital[0]? 1:0){ ROS_INFO("HIGH!!!") ;}

}

int main(int argc, char **argv)
{

  ros::Time::init();
  ros::Rate r (50);
  ros::init(argc, argv, "ir_sensor_node");    //node name
  ros::NodeHandle n;

  i_robot::button_msg button;
  button.get_ready=true;
  if(!button.get_ready){
  bool fuck ;
  fuck=button.get_ready;
  std::cout<<"fuck:   "<<fuck<<std::endl;
  sleep(5);
  }

  std::cout<<"get_ready:   "<<button.get_ready<<std::endl;


  sleep(5);

  ros::Publisher button_pub = n.advertise<i_robot::button_msg>("button",1000);
  //ros::Publisher arm_pub   =n.advertise<i_robot::arm_msg> ("arm_topic",1000);



  ros::Subscriber sub = n.subscribe("/arduino_zero/ir_topic", 1000, ArduinoMegaCallback);  //topic and callback function
  //r.sleep();
  while(ros::ok())  //use ros::ok() for ctrl+c to jump out the while loop!  ; don't use while(1),it can not be stop!
  {

  std::cout<<"hey hey hey "<<std::endl;
  std::cout<<"la la la  "<<std::endl;
  std::cout<<"hello "<<std::endl;

  if(ir_digital[0]? 1:0){ ROS_INFO("HIGH!!!") ;}
  r.sleep();  //for 1hz for this while loop!!

  ros::spinOnce();

    }

  return 0;
}




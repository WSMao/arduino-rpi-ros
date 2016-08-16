#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"i_robot/ir_msg.h"                   //using the message created by myself
#include "i_robot/motor_msg.h"
#include "i_robot/line_follower.h"       //defintion of line_follower.cpp
#include <cstddef>                                //for transformation of int type like uint16_t
#include "i_robot/mix_irimu_msg.h"

///////////////////////////////////global variabes////////////////////////////////////////

ir_struct ir={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1023,1023,1023,1023,1023} ;
imu_struct imu={0};
pwm_struct pwm={0};

using namespace std;
int calibration_times=0;
//void ArduinoMegaCallback(const i_robot::ir_msg&  msg)
void ArduinoMegaCallback(const i_robot::mix_irimu_msg&  msg)
{
  ROS_INFO_ONCE("i heard ir_sensor:");
//get ir data
     for(int i=0;i<5;i++){
             ir. ir_analog[i] =msg.ir_msg.ir_analog[i];
             ir.ir_digital[i]=(ir.ir_analog[i] > (ir.an[i]-ir.a1[i])/2 )?  1:0 ;    // ir_digital is derived from the ir_analog
          //   ir. ir_digital[i]  =msg.ir_msg.ir_digital [i];   //don't use this cause it can not really distiguish black or white !!

            // ir. ir_digital[i] =msg.ir_digital[i];
             //ir. ir_analog[i]=msg.ir_analog[i];
     }
ir.new_t=msg.ir_msg.new_t;   //get time from arduino timer

ROS_INFO("ir_digital:  %d  %d  %d  %d  %d  ",ir.ir_digital[0], ir.ir_digital[1],ir. ir_digital[2],ir. ir_digital[3], ir.ir_digital[4]);
ROS_INFO("ir_analog: %d  %d  %d  %d  %d  ",ir.ir_analog[0],ir.ir_analog[1],ir.ir_analog[2],ir.ir_analog[3],ir.ir_analog[4]);
ROS_INFO("ir_time: %f",ir.new_t);

ROS_INFO("yaw:  %f",msg.imu_msg.yaw );
ROS_INFO("imu_time:  %f",msg.imu_msg.new_t );

}

int main(int argc, char **argv)
{
  ros::Time::init();
  ros::Rate loop_rate (20);
  ros::init(argc, argv, "main_node");    //node name
  ros::NodeHandle n;
  int mission =0;  //start from mission " "
  ros::Subscriber sub = n.subscribe("sensor_state_topic", 1000, ArduinoMegaCallback);  //topic and callback function

  i_robot::motor_msg motor_message ;
  ros::Publisher pub = n.advertise<i_robot::motor_msg> ("motor_topic", 1000);
  usleep(500000);    //must sleep here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //for(int i=0;i<5;i++){ir.a1[i]=1023;cout<<ir.a1[i]; } //initialize the a1 for calibration
  while(ros::ok())  //use ros::ok() for ctrl+c to jump out the while loop!  ; don't use while(1),it can not be stop!
  {
//      pub.publish(motor_message);
//      loop_rate.sleep();  //for 20hz for this while loop!!
//      ros::spinOnce();
switch(mission){  //select mission to do !

case 0:  //calibration for ir sensor

     if(calibration_times>20 && calibration_times<=200){  //don't take the message when just start
         ir_stdl(ir);
         cout<<"calibration"<<endl; cout<<calibration_times<<endl;
     }

     else if(calibration_times>200) {
         mission=3;
         for(int i=0;i<5;i++){
             cout<<"a1["<<i<<"]"<<ir.a1[i]<<endl;
             cout<<"an["<<i<<"]"<<ir.an[i]<<endl;
         }
     }
     calibration_times++;
     break;

case 1:  //line_follower

      std::cout<<"mission 1 now!!"<<std::endl;
      pwm=line_follow( ir );
      std::cout<<"pwm_l: "<<pwm.pwm_l; //<<std::endl;
      std::cout<<"  pwm_r: "<<pwm.pwm_r<<std::endl;
      motor_message.pwm_l=pwm.pwm_l;
      motor_message.pwm_r=pwm.pwm_r;
      //if(ir.ir_digital=){mission=2;}
      //if(Cny70_read(ir.ir_digital)==0b11111) {  std::cout<<"go to mission 2!!"<<std::endl;  mission=2; }
      break;

case 2: //white line_follower
         std::cout<<"mission 2 now!!"<<std::endl;
         pwm=white_line_follow( ir );
         std::cout<<"pwm_l: "<<pwm.pwm_l; //<<std::endl;
         std::cout<<"  pwm_r: "<<pwm.pwm_r<<std::endl;
         motor_message.pwm_l=pwm.pwm_l;
         motor_message.pwm_r=pwm.pwm_r;
      break;

case 3: //black line_follower
         std::cout<<"mission 3 now!!"<<std::endl;
         pwm=black_line_follow( ir );
         std::cout<<"pwm_l: "<<pwm.pwm_l; //<<std::endl;
         std::cout<<"  pwm_r: "<<pwm.pwm_r<<std::endl;
         motor_message.pwm_l=pwm.pwm_l;
         motor_message.pwm_r=pwm.pwm_r;
      break;
default:
      std::cout<<" mission: "<< mission <<std::endl;
      break;
      }

  pub.publish(motor_message);
  loop_rate.sleep();  //for 20hz for this while loop!!
  ros::spinOnce();
  //ros::spin();

    }

  return 0;
}




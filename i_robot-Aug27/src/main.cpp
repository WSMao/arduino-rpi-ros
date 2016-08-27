#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "i_robot/ir_msg.h"                   //using the message created by myself
#include "i_robot/motor_msg.h"
#include "i_robot/line_follower.h"       //defintion of line_follower.cpp
#include <cstddef>                                //for transformation of int type like uint16_t
#include "i_robot/sonar_msg.h"
#include "i_robot/imu_msg.h"
#include "i_robot/button_msg.h"
#include "i_robot/arm_msg.h"
#include "i_robot/ROS_MEGA2_msg.h"
///////////////////////////////////global variabes////////////////////////////////////////

ir_struct  ir={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1023,1023,1023,1023,1023} ;

imu_struct imu={0,0};
sonar_struct sonar={0,0,0};
pwm_struct pwm={0,0};
//Mega2_struct Mega2;

int mission =0;  //start from mission " "
int stage =1;//stage 1 or stage 2
bool check_turn=0;//check if turned already
int mission0_control=0;
int mission1_control=0;
int mission2_control=0;
int mission3_control=0;
float dst=0;//destination of turn
bool jump=0;

using namespace std;
//int calibration_times=0;
//void ArduinoMegaCallback(const i_robot::ir_msg&  msg)

void ir_Cb(const i_robot::ir_msg &msg)
{
    ir.an[0]=548;ir.an[1]=656;ir.an[2]=607;ir.an[3]=705;ir.an[4]=596;//這些值可能要供校正修改
    ir.a1[0]=58;ir.a1[1]=72;ir.a1[2]=62;ir.a1[3]=74;ir.a1[4]=63;//這些值可能要供校正修改
     ROS_INFO_ONCE("i heard ir_sensor:");
//get ir data
     for(int i=0;i<5;i++){
             ir. ir_analog[i] =msg.ir_analog[i];
             ir.ir_digital[i]=(ir.ir_analog[i] > (ir.an[i]-ir.a1[i])/2 )?  1:0 ;    // ir_digital is derived from the ir_analog

     }
    ir.new_t=msg.new_t;   //get time from arduino timer

    ROS_INFO("ir_digital:  %d  %d  %d  %d  %d  ",ir.ir_digital[0], ir.ir_digital[1],ir. ir_digital[2],ir. ir_digital[3], ir.ir_digital[4]);
    ROS_INFO("ir_analog: %d  %d  %d  %d  %d  ",ir.ir_analog[0],ir.ir_analog[1],ir.ir_analog[2],ir.ir_analog[3],ir.ir_analog[4]);
    ROS_INFO("ir_time: %f",ir.new_t);

    select_mission(mission ,ir);
}

void imu_Cb(const i_robot::imu_msg &msg){
    ROS_INFO_ONCE("i heard imu_sensor:");
    imu.yaw=msg.yaw;
    imu.new_t=msg.new_t;
    ROS_INFO("imu.yaw:   %f",imu.yaw);
    ROS_INFO("imu.new_t: %f",imu.new_t);


}

void sonar_Cb(const i_robot::sonar_msg &msg){
    ROS_INFO_ONCE("i heard sonar_sensor:");
    sonar.sonar_front =msg.sonar_cm[0];
    sonar.sonar_right =msg.sonar_cm[1];
    sonar.sonar_back  =msg.sonar_cm[2];
    sonar.new_t=msg.new_t;

    ROS_INFO("sonar.back:   %d",sonar.sonar_back );
    ROS_INFO("sonar.front:  %d",sonar.sonar_front);
    ROS_INFO("sonar.right:  %d",sonar.sonar_right);
    ROS_INFO("sonar.new_t:  %f",sonar.new_t);
}

void button_Cb(const i_robot::button_msg &msg){
if(msg.get_ready==1 && msg.start_stage1==1)
{stage=1; ROS_INFO("stage1");}
else if(msg.get_ready==1 && msg.start_stage2==1)
{stage=2;}
//else
//{stage=0;}

}


i_robot::ROS_MEGA2_msg ros_mega2;

void Mega2Ros_Cb(const i_robot::ROS_MEGA2_msg &msg){
    ROS_INFO("got the Mega2Ros Message!");
    ros_mega2.start_follow_black=msg.start_follow_black;
    ros_mega2.start_follow_white=msg.start_follow_white;
    ROS_INFO("start_follow_white: %d",ros_mega2.start_follow_white);
    ROS_INFO("start_follow_black: %d",ros_mega2.start_follow_black);
   // sleep(5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_node");    //node name
  ros::Time::init();
  ros::Rate loop_rate (100);
  ros::NodeHandle n;

  ros::Subscriber ir_sub = n.subscribe("ir_topic", 1000, ir_Cb);  //topic and callback function
  ros::Subscriber imu_sub = n.subscribe("imu_topic", 1000, imu_Cb);  //topic and callback function
  ros::Subscriber sonar_sub = n.subscribe("sonar_topic", 1000, sonar_Cb);  //topic and callback function
  ros::Subscriber button_sub = n.subscribe("button_topic", 1000, button_Cb);  //topic and callback function

  ros::Subscriber ros_mega2_sub = n.subscribe ("Mega2toRos_topic",1000, Mega2Ros_Cb);

 // i_robot::ROS_MEGA2_msg mega2_message;
  ros::Publisher ros_mega2_pub = n.advertise<i_robot::ROS_MEGA2_msg> ("RostoMega2_topic",1000);


  i_robot::arm_msg arm_message;
  arm_message.AI_degree[0]=150;
  ros::Publisher arm_pub   =n.advertise<i_robot::arm_msg> ("arm_topic",1000);

  i_robot::motor_msg motor_message ;
  ros::Publisher motor_pub = n.advertise<i_robot::motor_msg> ("motor_topic", 1000);

  i_robot::imu_msg imu_message ;
  ros::Publisher imu_pub = n.advertise<i_robot::imu_msg> ("imu_to_mega2", 1000);

  usleep(500000);    //must sleep here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  while(ros::ok()){
  /////////////////////////////////////////////////stage 0///////////////////////////////////////////////////////////

  while(ros::ok() && stage==0){

      ROS_INFO("wait for button_msg then choose stage!!");
      ROS_INFO("stage: 0");
      motor_message.pwm_l=0;
      motor_message.pwm_r=0;
      loop_rate.sleep();
      ros::spinOnce();
  }


  /////////////////////////////////////////////////stage 1///////////////////////////////////////////////////////////


  while(ros::ok() && stage==1)  //use ros::ok() for ctrl+c to jump out the while loop!  ; don't use while(1),it can not be stop!
  {
      ROS_INFO("stage: 1");
      ROS_INFO("mission: %d",mission);
//      pub.publish(motor_message);
//      loop_rate.sleep();  //for 20hz for this while loop!!
//      ros::spinOnce();
switch(mission){  //select mission to do !

case 0:  //calibration for ir sensor and turn right to the white line



      ir_stdl(ir); //send ir reference &ir then get ir.an and ir.a1
      cout<<"calibration"<<endl;

      if(mission0_control==0){//go straight
          cout<<"go straight"<<endl;
          motor_message.pwm_l=40;
          motor_message.pwm_r=40;
      }
      if(sonar.sonar_back>=20 && sonar.sonar_back!=0 && mission0_control==0){//turn right

          cout<<"turn right now"<<endl;

          if(imu.yaw> 90 && imu.yaw<180){//第四象限，代表右轉會發生跳躍!
            jump=1;    //發生跳躍
          }
          dst=imu.yaw+90;  //turn right
          mission0_control=1; //switch to turn right section
      }

      if(mission0_control==1){  //turn now!!

          if(jump==1 && imu.yaw <0){//發生跳躍!   yaw>0 代表跳到另一區了
           imu.yaw=imu.yaw+360;   //則做修正，使其符合上一個象限的邏輯
          }

          if(abs(dst-imu.yaw) <=2){
        //  if(dst-imu.yaw >0){
            motor_message.pwm_l=0;
            motor_message.pwm_r=0;
            mission0_control=2;

          }
          else{
            pwm=turn_right(imu,dst);
            motor_message.pwm_l=pwm.pwm_l;
            motor_message.pwm_r=pwm.pwm_r;
          }

      }

      if(mission0_control==2){//go straight!!

          motor_message.pwm_l=0;
          motor_message.pwm_r=0;
          motor_pub.publish(motor_message);//pub motor message!
          //ros::spinOnce();
          sleep(1);
          mission0_control=3;
      }

      if(mission0_control==3){
          motor_message.pwm_l=40;
          motor_message.pwm_r=40;
          //ros::spinOnce();
          motor_pub.publish(motor_message);//pub motor message!
          sleep(1); //直走一秒
          mission=1; //go to white_line_follower mission!!
          cout<< "go to white_line_follower mission" << endl;
      }

          //mission0_control=3;

      break;


case 1: //white line_follower
//    ir_stdl(ir); //send ir reference &ir then get ir.an and ir.a1
//    cout<<"calibration"<<endl;
    //test
       std::cout<<"white_line_follower now!!"<<std::endl;
//       std::cout<<"start_follow_white: "<<mega2_message.start_follow_white<<std::endl;
       //sleep(3);//為了測試尋縣放的 因為如果mega在還沒spin之前就pub給他的話會街不到
       ir.an[0]=548;ir.an[1]=656;ir.an[2]=607;ir.an[3]=705;ir.an[4]=596;//這些值可能要供校正修改
       ir.a1[0]=58;ir.a1[1]=72;ir.a1[2]=62;ir.a1[3]=74;ir.a1[4]=63;//這些值可能要供校正修改
       if(mission1_control==0){  //send the a1 and an to mega2 then start follow the white line in mega
           for(int i=0;i<5;i++){
                ros_mega2.a1[i]=ir.a1[i];
                ros_mega2.an[i]=ir.an[i];
           }
           ros_mega2.start_follow_white=1;
           ros_mega2_pub.publish(ros_mega2);
           sleep(1);
//           ros::spinOnce();
           mission1_control=1;
       }

       //if(mission1_control==1){//wait the message from mega that it's the end of line follower!
       while(ros::ok() && mission1_control==1){
          // ros_mega2_pub.publish(ros_mega2);   //這邊發對方要收的到才行要剛好在聽
           std::cout<<"waitting the line_follower"<<std::endl;
           ros::spinOnce();//listen the message
           imu_message.yaw=imu.yaw;
           imu_message.new_t=imu.new_t;
           //
           if(abs(-90-imu.yaw)<10){

                std::cout<<"yes imu is almost -90 degree!!!"<<std::endl;
                imu_pub.publish(imu_message);
                ros::spinOnce();//listen the message
                usleep(500000);//spinOnce這裡一定要等待 似乎是接收arduino 資訊 怕漏接訊息
           }
           //
           if(ros_mega2.start_follow_white==0){ // if Mega2 tell the line_follower mission has finished
                mission1_control=2;
                mission=2;
                std::cout<<"change to mission 2!!"<<std::endl;
                std::cout<<"finish follow the line!"<<std::endl;
                //sleep(1);
                break;  //break while loop
           }

       }

      break;//break switch loop

case 2:  //avoid and turn left!!

        if(mission2_control==0){//go straight
            cout<<"go straight"<<endl;
            cout<<"sonar_front: "<<sonar.sonar_front<<endl;
            motor_message.pwm_l=30;
            motor_message.pwm_r=30;
            motor_pub.publish(motor_message);
        }
        if(sonar.sonar_front<=25 && sonar.sonar_front!=0 && mission2_control==0){//turn left

            cout<<"turn left now"<<endl;
            if(imu.yaw> -180 && imu.yaw<-90){//第3象限，代表轉會發生跳躍!
              jump=1;    //發生跳躍
            }
            dst=imu.yaw-90;  //turn right
            mission2_control=1; //switch to turn right section
        }

        if(mission2_control==1){  //turn now!!

            if(jump==1 && imu.yaw >0){//發生跳躍!   yaw>0 代表跳到另一區了
             imu.yaw=imu.yaw-360;   //則做修正，使其符合上一個象限的邏輯
            }

            cout<<"yaw: "<<imu.yaw<<endl;

            if(abs(dst-imu.yaw) <=2){  //誤差內及停止
          //  if(dst-imu.yaw >0){
              motor_message.pwm_l=0;
              motor_message.pwm_r=0;
              motor_pub.publish(motor_message);
              mission2_control=2;

            }
            else{  //不然就左轉
              pwm=turn_left(imu,dst);
              motor_message.pwm_l=pwm.pwm_l;
              motor_message.pwm_r=pwm.pwm_r;
              motor_pub.publish(motor_message);
            }

        }

        if(mission2_control==2){//go straight!!

            motor_message.pwm_l=0;
            motor_message.pwm_r=0;
            motor_pub.publish(motor_message);
            ros::spinOnce();
            sleep(1);
            mission2_control=3;
        }

        if(mission2_control==3){
//            motor_message.pwm_l=ini;
//            motor_message.pwm_r=40;
            //ros::spinOnce();
            // go along the wall with a constant distance!!!

//            PWM_L=40+(error)*P1-(R_distance_goal-distance_R)*P2;
//            PWM_R=40+(-error)*P1+(R_distance_goal-distance_R)*P2;

            if(imu.yaw >0){//發生跳躍!   yaw>0 代表跳到另一區了
             imu.yaw=imu.yaw-360;   //則做修正，使其符合上一個象限的邏輯
            }

            pwm=follow_wall(imu,sonar,dst,22); //22cm to the wall !

            motor_message.pwm_l=pwm.pwm_l;
            motor_message.pwm_r=pwm.pwm_r;
            motor_pub.publish(motor_message);
            std::cout<<"motor_message.pwm_l"<<motor_message.pwm_l<<std::endl;
            std::cout<<"motor_message.pwm_r"<<motor_message.pwm_r<<std::endl;
            std::cout<<"follow the wall  !!!!!!!"<<std::endl;

        }

      break; //break switch loop
case 3:
         std::cout<<"wait for the right position"<<std::endl;

         if(mission3_control==0){
             motor_message.pwm_l=30;
             motor_message.pwm_r=30;
             motor_pub.publish(motor_message);
             //ros::spinOnce();
             mission3_control=1;
             usleep(1500000);
             //sleep(2); //0.5 second!
         }

         if(mission3_control==1){
             motor_message.pwm_l=0;
             motor_message.pwm_r=0;
             motor_pub.publish(motor_message);
             mission3_control=2;
             sleep(1); //0.5 second!
         }

         if(mission3_control==2){
             std::cout<<"press the button!!"<<std::endl;
             arm_message.AI_degree[1]=235; //ID_2 235 to touch the button
             arm_pub.publish(arm_message);
             //ros::spinOnce();
             sleep(1.5);
             arm_message.AI_degree[1]=150;
             arm_pub.publish(arm_message);
             //ros::spinOnce();
             sleep(1.5);
         }


      break;//break switch loop

case 4:

      break;

case 7: //black line_follower

         std::cout<<"mission 3 now!!"<<std::endl;
         pwm=black_line_follow( ir );
         std::cout<<"pwm_l: "<<pwm.pwm_l; //<<std::endl;
         std::cout<<"  pwm_r: "<<pwm.pwm_r<<std::endl;
         motor_message.pwm_l=pwm.pwm_l;
         motor_message.pwm_r=pwm.pwm_r;
   // usleep(1000000);
      break;
default:
         std::cout<<" mission: "<< mission <<std::endl;
      break;
      }

  motor_pub.publish(motor_message);
  loop_rate.sleep();  //for 20hz for this while loop!!
  ros::spinOnce();
  //ros::spin();

    }

  /////////////////////////////////////////////////stage 2///////////////////////////////////////////////////////////

  while(ros::ok() && stage==2){
    ROS_INFO("stage: 2");
    motor_pub.publish(motor_message);
    loop_rate.sleep();
    ros::spinOnce();
  }


  cout<<"stage: "<<stage<<"mission: "<<mission<<endl;



  loop_rate.sleep();
  ros::spinOnce();
  }//big while(ros::ok())

  if(!( ros::ok() )){
      motor_message.pwm_l=0;
      motor_message.pwm_r=0;

  }
  ros::spinOnce();
  //usleep(1000000);
  return 0;
}




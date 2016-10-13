#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "i_robot/motor_msg.h"
#include "i_robot/arm_msg.h"
#include <iostream>
#include "i_robot/line_follower.h"
#include "i_robot/camera_msg.h"

using namespace std;
float pwm_l=0,pwm_r=0;
bool joy_button_left,joy_button_right,joy_button_up,joy_button_down;
bool joy_button_lb  ,joy_button_rb;
float  joy_axis_linear,joy_axis_angular;
float  joy_axis_lt    ,joy_axis_rt;

void JoyCb(const sensor_msgs::Joy & msg)
{
    cout<<"yeah!!"<<endl;
    joy_button_left  = msg.buttons[11];
    joy_button_right = msg.buttons[12];
    joy_button_up    = msg.buttons[13];
    joy_button_down  = msg.buttons[14];

    joy_button_lb    = msg.buttons[4];
    joy_button_rb    = msg.buttons[5];

    joy_axis_linear  = msg.axes[1];
    joy_axis_angular = msg.axes[3];

    joy_axis_lt    = msg.axes[2];
    joy_axis_rt    = msg.axes[5];

}

int main( int argc, char ** argv)
{
    ros::Time::init();
    ros::init(argc,argv,"controller");    // 初始化，"  <ndoe_name>  "
    ros::NodeHandle n;
    ros::Subscriber joy_sub = n.subscribe("joy", 1000, JoyCb);  //topic and callback function
    ros::Rate loop_rate (20);

    i_robot::motor_msg motor_message ;
    ros::Publisher motor_pub = n.advertise<i_robot::motor_msg> ("joy_motor_topic", 1000);

    i_robot::camera_msg camera_message;
    ros::Publisher camera_pub =n.advertise<i_robot::camera_msg> ("joy_camera_topic",1000);

    i_robot::arm_msg arm_message;
    ros::Publisher arm_pub =n.advertise<i_robot::arm_msg> ("joy_arm_topic",1000);

    bool arm_control=0;
    float init_axis_lt=0;
    float init_axis_rt=0;
    int init_arm0_pos=0;
    int init_arm1_pos=0;
    arm_message.AI_degree[0]=150;
    arm_message.AI_degree[1]=180;
    int arm0_pos=0;
    int arm1_pos=0;

    bool grab_control=0;
    int  init_grab_pos=0;
    arm_message.Servo_degree=150;  //open at initial
    int  grab_pos=0;

    bool head_control=0;
    camera_message.Servo_shake=90;  //head pos at initial
    while(ros::ok()){

/////////////////////////////////////////////////////motor control/////////////////////////////////////////////////////

        pwm_l=joy_axis_linear*150-joy_axis_angular*80;
        pwm_r=joy_axis_linear*150+joy_axis_angular*80;
        motor_message.pwm_l=pwm_l;
        motor_message.pwm_r=pwm_r;

        motor_pub.publish(motor_message);

/////////////////////////////////////////////////////arm_control/////////////////////////////////////////////////////

        if(joy_button_lb==1){

            if(arm_control==0){

                init_axis_lt=joy_axis_lt;                //remember the initial pos of axis
                init_arm0_pos=arm_message.AI_degree[0];  //remeber the initial pos of arm

                init_axis_rt=joy_axis_rt;                //remember the initial pos of axis
                init_arm1_pos=arm_message.AI_degree[1];  //remeber the initial pos of arm

                arm_control=1;                           //change state

            }

            if(arm_control==1){

                //desired range : 0~150 and initial at 150
                arm0_pos=init_arm0_pos-(joy_axis_lt-init_axis_lt)*50;
                arm0_pos=(arm0_pos<0)?     0:arm0_pos;
                arm0_pos=(arm0_pos>150)? 150:arm0_pos;
                arm_message.AI_degree[0]=arm0_pos;

                //desired range : 130~240 and initial at 180
                arm1_pos=init_arm1_pos+(joy_axis_rt-init_axis_rt)*40;
                arm1_pos=(arm1_pos<130)? 130:arm1_pos;
                arm1_pos=(arm1_pos>240)? 240:arm1_pos;
                arm_message.AI_degree[1]=arm1_pos;

                arm_pub.publish(arm_message);

            }
            ROS_INFO("init_axis_lt: %f",init_axis_lt);
            ROS_INFO("arm0_pos: %d",arm_message.AI_degree[0]);

            ROS_INFO("init_axis_rt: %f",init_axis_rt);
            ROS_INFO("arm1_pos: %d",arm_message.AI_degree[1]);

        }

        else if (joy_button_lb==0){
            arm_control=0;
        }

//grab control//
        if(joy_button_rb==1 && joy_button_lb==0){ //acting if joy_button_lb doesn't be pressed

            if(grab_control==0){

                init_axis_rt=joy_axis_rt;                //remember the initial pos of axis
                init_grab_pos=arm_message.Servo_degree;  //remeber the initial pos of arm

                grab_control=1;                           //change state

            }

            if(grab_control==1){

                grab_pos=init_grab_pos+(joy_axis_rt-init_axis_rt)*45;
                grab_pos=(grab_pos<60)?     60:grab_pos;
                grab_pos=(grab_pos>150)?   150:grab_pos;
                arm_message.Servo_degree=grab_pos;

                arm_pub.publish(arm_message);

            }
            ROS_INFO("init_axis_rt: %f",init_axis_rt);
            ROS_INFO("grab_pos: %d",arm_message.Servo_degree);
        }
        else if (joy_button_rb==0){
            grab_control=0;
        }

/////////////////////////////////////////////////////head control/////////////////////////////////////////////////////


        if(joy_button_left==1 && joy_button_right==0){

            if(head_control==0){
                camera_message.Servo_shake += 5;
                camera_pub.publish(camera_message);
                head_control=1;
            }
            ROS_INFO("camera_shake: %d",camera_message.Servo_shake);
        }

        if(joy_button_right==1 && joy_button_left==0){

            if(head_control==0){
                camera_message.Servo_shake += -5;
                camera_pub.publish(camera_message);
                head_control=1;
            }
            ROS_INFO("camera_shake: %d",camera_message.Servo_shake);
        }

        if(joy_button_left==0 && joy_button_right==0){
            head_control=0;
        }



        loop_rate.sleep();  //for 20hz for this while loop!!
        ros::spinOnce();
    }

    return 0;
}

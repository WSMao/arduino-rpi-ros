#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "i_robot/motor_msg.h"
#include "i_robot/arm_msg.h"
#include <iostream>
#include "i_robot/line_follower.h"
//#include "i_robot/camera_msg.h"
using namespace std;
float pwm_l=0,pwm_r=0;
bool joy_button_left,joy_button_right,joy_button_up,joy_button_down;
bool joy_button_lb  ,joy_button_rb;
int  joy_axis_linear,joy_axis_angular;
int  joy_axis_lt    ,joy_axis_rt;

void JoyCb(const sensor_msgs::Joy & msg)
{
    cout<<"yeah!!"<<endl;
    joy_button_left  = msg.buttons[12];
    joy_button_right = msg.buttons[13];
    joy_button_up    = msg.buttons[14];
    joy_button_down  = msg.buttons[15];

    joy_button_lb    = msg.buttons[4];
    joy_button_rb    = msg.buttons[5];

    joy_axis_linear  = msg.axes[1]*150;
    joy_axis_angular = msg.axes[3]*50;

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
   // i_robot::camera_msg camera_message;
   // ros::Publisher camera_pub =n.advertise<i_robot::camera_msg> ("joy_camera_topic",1000);
    while(ros::ok()){

        pwm_l=joy_axis_linear-joy_axis_angular;
        pwm_r=joy_axis_linear+joy_axis_angular;
        motor_message.pwm_l=pwm_l;
        motor_message.pwm_r=pwm_r;

        motor_pub.publish(motor_message);
        loop_rate.sleep();  //for 20hz for this while loop!!
        ros::spinOnce();
    }

    return 0;
}

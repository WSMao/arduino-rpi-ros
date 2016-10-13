#include <ros/ros.h>
#include "i_robot/imu_msg.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
double imu=0;
ros::Publisher imu_pub;

void imu_transferCb(const i_robot::imu_msg&  msg){

    tf::Quaternion q(msg.yaw,0,0);

    sensor_msgs::Imu u;

    tf::quaternionTFToMsg(q, u.orientation);


    imu_pub.publish(u);
}


int main( int argc, char ** argv)
{

    ros::Time::init();
    ros::init(argc,argv,"sensor");    // 初始化，而"  <ndoe_name>  "
    ros::Rate r (20);
    ros::NodeHandle n;
    ros::Subscriber imu_sub=n.subscribe ("imu_topic", 1000, imu_transferCb);

    i_robot::imu_msg imu_message ;
    imu_pub = n.advertise<sensor_msgs::Imu> ("imu", 1000);

    while( ros::ok() ){

       ROS_INFO("hello!!");

    r.sleep();  //for 1hz for this while loop!!
    ros::spinOnce();

    }
    return 0;
}

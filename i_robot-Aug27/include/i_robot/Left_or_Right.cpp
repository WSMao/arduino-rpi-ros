#include<iostream>
#include "i_robot/line_follower.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


pwm_struct turn_right (imu_struct imu ,float dst){

    pwm_struct pwm;

    float P=0.8;
    float D=0.001;

    static float old_t;
    static float err_old=0;

    float new_t;
    float err_new;
    float err_rate;

    float error=dst-imu.yaw;

    err_new=error;
    new_t=imu.new_t;

    if((new_t-old_t)==0){err_rate=0;}
    else{
    err_rate=(err_new-err_old)/(new_t-old_t);
    }

    pwm.pwm_l=error*P+err_rate*D;
    pwm.pwm_r=(-error)*P-err_rate*D;

    cout<<"avoid!!!  "<< endl;
    cout<<"turn right!!!  "<< endl;
    cout<<"err_new:  "<< err_new  << endl;
    cout<<"err_old:  "<< err_old  << endl;
    cout<<"err_rate: "<< err_rate << endl;
    cout<<"pwm_l:    "<< pwm.pwm_l << endl;
    cout<<"pwm_r:    "<< pwm.pwm_r << endl;

    old_t=new_t;
    err_old=err_new;

    return pwm;

}



pwm_struct turn_left (imu_struct imu ,float dst){

    pwm_struct pwm;

    float P=0.8;
    float D=0.001;

    static float old_t;
    static float err_old=0;

    float new_t;
    float err_new;
    float err_rate;

    float error=dst-imu.yaw;

    err_new=error;
    new_t=imu.new_t;

    if((new_t-old_t)==0){err_rate=0;}
    else{
    err_rate=(err_new-err_old)/(new_t-old_t);
    }

    pwm.pwm_l=error*P+err_rate*D;
    pwm.pwm_r=(-error)*P-err_rate*D;

    cout<<"avoid!!!  "<< endl;
    cout<<"turn left!!!  "<< endl;
    cout<<"err_new:  "<< err_new  << endl;
    cout<<"err_old:  "<< err_old  << endl;
    cout<<"err_rate: "<< err_rate << endl;
    cout<<"pwm_l:    "<< pwm.pwm_l << endl;
    cout<<"pwm_r:    "<< pwm.pwm_r << endl;

    old_t=new_t;
    err_old=err_new;

    return pwm;


}

pwm_struct follow_wall(imu_struct imu, sonar_struct sonar, float dst_yaw, float dst_sonar){
    pwm_struct pwm;
    int initial=40;
    float P_control_yaw=1;
    float P_control_sonar=0.7;

    float error=dst_yaw-imu.yaw;

    cout<<"dst_sonar: "<<dst_sonar<<endl;
    cout<<"sonar_right: "<<sonar.sonar_right<<endl;

    cout<<"error: "<< error <<endl;

    pwm.pwm_l=initial+(error)*P_control_yaw-(dst_sonar-sonar.sonar_right)*P_control_sonar;
    pwm.pwm_r=initial+(-error)*P_control_yaw+(dst_sonar-sonar.sonar_right)*P_control_sonar;

    cout<<"pwm_l :"<<pwm.pwm_l<<endl;
    cout<<"pwm_r :"<<pwm.pwm_r<<endl;

    return pwm;

}

#include<iostream>
#include "i_robot/line_follower.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


pwm_struct turn_right (imu_struct imu ,float dst){

    pwm_struct pwm;

    float P=1;
    float D=0.001;

    static float old_t;
    static float err_old=0;

    float new_t;
    float err_new;
    float err_rate;

    float error=dst-imu.yaw;



    err_new=error;
    new_t=imu.new_t;

    err_rate=(err_new-err_old)/(new_t-old_t);


    pwm.pwm_l=error*P+err_rate*D;
    pwm.pwm_r=(-error)*P-err_rate*D;

    cout<<"avoid!!!  "<< endl;
    cout<<"err_new:  "<< err_new  << endl;
    cout<<"err_old:  "<< err_old  << endl;
    cout<<"err_rate: "<< err_rate << endl;
    cout<<"pwm_l:    "<< pwm.pwm_l << endl;
    cout<<"pwm_r:    "<< pwm.pwm_r << endl;

    old_t=new_t;
    err_old=err_new;

    return pwm;

      //turn right
//      MOTOR.Motor_left(0,pwmL);
//      MOTOR.Motor_right(1,pwmR);


}



pwm_struct turn_left (imu_struct imu ,float dst){



}

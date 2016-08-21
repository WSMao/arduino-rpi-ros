#include <iostream>
#include "ros/ros.h"
//#include <cstddef>                                //for transformation of int type like uint16_t

typedef enum{
    FuzzyMode=1,
    WaitToTurnLeft,
    WaitToTurnRight,
    NowTurnLeft,
    NowTurnRight
}Mode;
//Mode mode;

struct sonar_struct{
    uint16_t sonar_back;
    uint16_t sonar_front;
    uint16_t sonar_right;
    float new_t;

};//sonar struct

struct ir_struct{
    uint16_t ir_analog[5];
    bool ir_digital[5];
    float new_t;
    int an[5];
    int a1[5];
};//ir;

struct imu_struct{
    float yaw;
    float new_t;
};

//float new_t=0;

struct pwm_struct{
    float pwm_l;
    float pwm_r;
};//pwm;//pwm 不該於此定義 會重複定義

//pwm_struct line_follow(ir_struct ir, float new_t);
pwm_struct line_follow(ir_struct ir);
int Cny70_read(bool[5]);
void cny70_print(ir_struct ir);
float err(float a1,float an,float ir);
float Fuzzy(float err_new, float err_rate) ;
float White_Fuzzy(float err_new,float err_rate);
float Black_Fuzzy(float err_new,float err_rate);
//
void ir_stdl(ir_struct &ir);  //send ir reference (&ir)

pwm_struct white_line_follow( ir_struct ir );
pwm_struct black_line_follow( ir_struct ir );

pwm_struct turn_left (imu_struct imu ,float dst);
pwm_struct turn_right(imu_struct imu ,float dst);

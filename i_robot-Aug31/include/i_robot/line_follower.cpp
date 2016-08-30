#include<iostream>
#include "i_robot/line_follower.h"
#include <stdio.h>
#include <stdlib.h>

//typedef enum{
//    FuzzyMode=1,
//    WaitToTurnLeft,
//    WaitToTurnRight,
//    NowTurnLeft,
//    NowTurnRight
//}Mode;
//Mode mode;
Mode mode;

pwm_struct line_follow(ir_struct ir){
    static bool busy=0;
    static bool wait =0;

    pwm_struct pwm;
    static float old_t;
    static float err_old=0;
    float err_new;
    float err_ll=0,err_ml=0,err_mm=0,err_mr=0,err_rr=0;
    //int an_ll=546,an_ml=680,an_mm=530,an_mr=689,an_rr=500;//這些值可能要供校正修改
    //int a1_ll=20,a1_ml=20,a1_mm=20,a1_mr=20,a1_rr=20;
    int an_ll=ir.an[0],an_ml=ir.an[1],an_mm=ir.an[2],an_mr=ir.an[3],an_rr=ir.an[4];//這些值可能要供校正修改
    int a1_ll=ir.a1[0],a1_ml=ir.a1[1],a1_mm=ir.a1[2],a1_mr=ir.a1[3],a1_rr=ir.a1[4];

    int initial;
    //the digital value is not good!!
    int cny70=Cny70_read(ir.ir_digital);    //std::cout<<cny70<<std::endl;

    uint16_t ir_ll=ir.ir_analog[0];   //讀入CNY70的類比資訊
    uint16_t ir_ml=ir.ir_analog[1];
    uint16_t ir_mm=ir.ir_analog[2];
    uint16_t ir_mr=ir.ir_analog[3];
    uint16_t ir_rr=ir.ir_analog[4];


    if(!busy){  //如果沒有在執行特殊任務，則判斷任務

        if(cny70==0b11110 || cny70==0b11100 ){
        mode=WaitToTurnLeft;
        busy=1;
        wait=1;
        }

        else if(cny70==0b11000 || cny70==0b10000){
        mode=NowTurnLeft;
        busy=1;
        }

        else if(cny70==0b01111 || cny70==0b00111 ){
        mode=WaitToTurnRight;
        busy=1;
        wait=1;
        }

        else if(cny70==0b00111 || cny70==0b00001){
        mode=NowTurnRight;
        busy=1;
        }
        else{  mode =FuzzyMode; }


    }

    if(mode==WaitToTurnLeft){

        if(ir.ir_digital[2]==1 && wait==1){ //waiting
        std::cout<<"waiting"<<std::endl;
        pwm.pwm_l=160;
        pwm.pwm_r=160;
        }

        else{   //turning left
        wait=0;
        std::cout<<"turning left"<<std::endl;
        pwm.pwm_l=100;
        pwm.pwm_r=190;
        if(ir.ir_digital[2]==1){mode=FuzzyMode;busy=0;}
        }
    }

    if(mode==WaitToTurnRight){

        if(ir.ir_digital[2]==1 && wait==1){ //waiting
        std::cout<<"waiting"<<std::endl;
        pwm.pwm_l=160;
        pwm.pwm_r=160;
        }

        else{   //turning right
        wait=0;
        std::cout<<"turning right"<<std::endl;
        pwm.pwm_l=190;
        pwm.pwm_r=100;
        if(ir.ir_digital[2]==1){mode=FuzzyMode;busy=0;}
        }
    }

    if(mode==NowTurnLeft){
        std::cout<<"turning left"<<std::endl;
        pwm.pwm_l=100;
        pwm.pwm_r=190;
        if(ir.ir_digital[2]==1){mode=FuzzyMode;busy=0;}
    }
    if(mode==NowTurnRight){
        std::cout<<"turning right"<<std::endl;
        pwm.pwm_l=190;
        pwm.pwm_r=100;
        if(ir.ir_digital[2]==1){mode=FuzzyMode;busy=0;}
    }
///////////////////////////////////////////////////FuzzyMode/////////////////////////////////////////////////
    if(mode==FuzzyMode){
           std::cout<<"fuzzy ing"<<std::endl;

          //算出每顆感測器的側偏差
            //err_ll=err(a1_ll,an_ll,ir_ll);
            err_ml=(-0.7)*err(a1_ml,an_ml,ir_ml);
            //err_mm=err(a1_mm,an_mm,ir_mm);
            err_mr=(0.7) *err(a1_mr,an_mr,ir_mr);
            //err_rr=err(a1_rr,an_rr,ir_rr);
            err_new =err_ll + err_ml + err_mm + err_mr + err_rr; //計算總誤差

            std::cout<<"old_t"<<old_t<<std::endl;
            std::cout<<"new_t"<<ir.new_t<<std::endl;
            float dt=(ir.new_t-old_t)/1000.0; //將毫秒換算成秒
//            if(dt==0){dt=0.00001;}//預防措施

            if(dt==0){//預防措施
                pwm.pwm_l=0;
                pwm.pwm_r=0;
                return pwm;
            }

            //dt=10;   // if dt=0   then memory section failed,core dumped    !!!!
        std::cout<<"dt"<<dt<<std::endl;
            old_t=ir.new_t;    //上次讀取完誤差的時間
//if dt=0   then memory section failed,core dumped    !!!!
     float err_rate= (err_new-err_old)/ dt; //變動率（每秒的誤差變化）
            err_old=err_new;//上次讀取總誤差

            //float err_rate=0; //暫時
            //std::cout<<"ir_ml: "<<ir_ml<<"\t"<<std::endl;
//            std::cout<<"err_ml: "<<err_ml<<"\t";     //<<std::endl;
//            //std::cout<<"ir_mr: "<<ir_mr<<"\t"<<std::endl;
//            std::cout<<"err_mr: "<<err_mr<<"\t"<<std::endl;
//            std::cout<<"err_old: "<<err_old<<"\t"<<std::endl;
//            std::cout<<"err_new: "<<err_new<<"\t"<<std::endl;
//            std::cout<<"err_rate: "<<err_rate<<std::endl;
           // _sleep(300);
            //usleep(300);

            float difference=Fuzzy(err_new,err_rate);
          //如果ir_mm沒有在黑線上，則初速往下調，達到過彎減速
            if(ir.ir_digital[2]==0){initial=220;std::cout<<"brake\t"<<std::endl;}
            else {initial=230;}
          //計算後輸出轉速
            pwm.pwm_l=initial+difference;   // initial:220   difference:+-0~30
            pwm.pwm_r=initial-difference;
           //左右馬達前進輸出
            if(pwm.pwm_l<=0){pwm.pwm_l=0;}
            if(pwm.pwm_r<=0){pwm.pwm_r=0;}
            //MOTOR.Motor_left(0,pwm_l);
            //MOTOR.Motor_right(0,pwm_r);
    }
//    //usleep(1000);
    return pwm;
    }//line follower end


//////////////////////////////////////////////////////////////function//////////////////////////////////////////////////////////////////////////////////
int Cny70_read(bool a[5])
{
  int cny70=0;
  cny70+=a[0];
  cny70<<=1;
  cny70+=a[1];
  cny70<<=1;
  cny70+=a[2];
  cny70<<=1;
  cny70+=a[3];
  cny70<<=1;
  cny70+=a[4];
  return  cny70;
}


void cny70_print(ir_struct ir){
    for(int i=0;i<5;i++){
        if(ir.ir_digital[i]==1){std::cout<<1;} else {std::cout<<0;}
    }
    //std::cout<<std::endl;
}


float err(float a1,float an,float ir){
float error;
if(ir<=a1){error=0;}
else if(ir>=an){error=1;}
else if(ir>a1) { error=(ir-a1)/(an-a1);}
return error;
}

///////////////////////////////////////////////////////////////////////////////////Fuzzy//////////////////////////////////////////////////////////////////////////
float Fuzzy(float err_new, float err_rate)     //  模糊控制
{
  float difference;
  float input1=err_new,input2=err_rate;
  float weight1,weight2,weight3,weight4;
  int a1,a2,NA,j;
  int k=8;//NB=-130  NS=-25
  float NB=-130 , NM=-50 , NS=-30 , Z0=0 , PS=-NS , PM=-NM , PB=-NB ;
  float rule[8][8] = {
    //    車體偏右                             車體偏左
    //區間1     2     3     4     5     6     7     8
    //      NB    NM    NS    Z0    PS    PM    PB        //區間
    { NA, -0.7, -0.3, -0.1,  0  ,  0.1,  0.3,  0.7 }
    ,                                                     // 1
    { 70 ,  Z0 ,  Z0 ,  PS ,  PM ,  PM ,  PB ,  PB  } //PB     往左變化
    ,                                                     // 2
    { 20 ,  NM ,  Z0 ,  Z0 ,  Z0 ,  PM ,  PB ,  PB  } //PM
    ,                                                     // 3
    { 10 ,  NB ,  NS ,  Z0 ,  Z0 ,  PS ,  PM ,  PB  } //PS
    ,                                                     // 4
    { 0  ,  NB ,  NM ,  NS ,  Z0 ,  PS ,  PM ,  PB  } //Z0
    ,                                                     // 5
    {-10 ,  NB ,  NM ,  NS ,  Z0 ,  Z0 ,  PS ,  PB  } //NS
    ,                                                     // 6
    {-20 ,  NB ,  NB ,  NM ,  Z0 ,  Z0 ,  Z0 ,  PM  } //NM
    ,                                                     // 7
    {-70 ,  NB ,  NB ,  NM ,  NM ,  NS ,  Z0 ,  Z0  } //NB     往右變化
                                                    };    // 8

  for ( j=1 ; j<k-1 ; j++ ) {
    if(input1>=rule[0][j] && input1<=rule[0][j+1]) {
      weight1=1-((input1-rule[0][j])/(rule[0][j+1]-rule[0][j]))  ;
      a1=j+1;
    }
    else if(input1<=rule[0][1]) {
      weight1=1;
      a1=1;
    }
    else if(input1>=rule[0][k-1]) {
      weight1=0;
      a1=k;
    }
  }
  for ( j=1 ; j<k-1 ; j++ ) {
    if(input2<=rule[j][0] && input2>=rule[j+1][0]) {
      weight3=(input2-rule[j+1][0])/(rule[j][0]-rule[j+1][0]) ;
      a2=j+1;
    }
    else if(input2>=rule[1][0]) {
      weight3=1;
      a2=1;
    }
    else if(input2<=rule[k-1][0]) {
      weight3=0;
      a2=k;
    }
  }
  weight2=1-weight1;
  weight4=1-weight3;
  if(a1==1) {
    if(a2==1) {
      difference=weight1*weight3*rule[a2][a1];
    }
    else if(a2==k) {
      difference=weight1*weight4*rule[a2-1][a1];
    }
    else {
      difference=weight1*weight3*rule[a2-1][a1]+weight1*weight4*rule[a2][a1];
    }
  }
  else if(a1==k) {
    if(a2==1) {
      difference=weight2*weight3*rule[a2][a1-1];
    }
    else if(a2==k) {
      difference=weight2*weight4*rule[a2-1][a1-1];
    }
    else {
      difference=weight2*weight3*rule[a2-1][a1-1]+weight2*weight4*rule[a2][a1-1];
    }
  }
  else {
    if(a2==1) {
      difference=weight1*weight3*rule[a2][a1-1]+weight2*weight3*rule[a2][a1];
    }
    else if(a2==k) {
      difference=weight1*weight4*rule[a2-1][a1-1]+weight2*weight4*rule[a2-1][a1];
    }
    else {
      difference=weight1*weight3*rule[a2-1][a1-1]+weight1*weight4*rule[a2][a1-1]+weight2*weight3*rule[a2-1][a1]+weight2*weight4*rule[a2][a1];
    }
  }

//  if(cny70_3(hi)!=0){
  //Serial.print(input1,4);Serial.print("\t");
     // std::cout<<input1<<"\t"<<std::endl;
  //Serial.print(input2,4);Serial.print("\t");
     // std::cout<<input2<<"\t"<<std::endl;
  //Serial.print(a1);Serial.print("\t");
     // std::cout<<a1<<"\t"<<std::endl;
  //Serial.print(a2);Serial.print("\t");
     // std::cout<<a2<<"\t"<<std::endl;
  //Serial.print(difference,4);Serial.println("\t");
     // std::cout<<difference<<"\t"<<std::endl;
  //}
 // else   Serial.println("out of line");
     //else std::cout<<"out of line"<<std::endl;
  return difference;
}

//void ir_stdl(ir_struct ir,int LL,int ML,int MM,int MR,int RR){
//int i=0;
void ir_stdl(ir_struct &ir){
    std::cout<<"ir_calibration"<<std::endl;
    if(ir.ir_analog[0]==0 || ir.ir_analog[1]==0 || ir.ir_analog[2]==0 || ir.ir_analog[3]==0 || ir.ir_analog[4]==0)
    {   //do nothing
    }

    else{
        if(ir.ir_analog[0]>=ir.an[0]){ir.an[0]=ir.ir_analog[0];}
        if(ir.ir_analog[1]>=ir.an[1]){ir.an[1]=ir.ir_analog[1];}
        if(ir.ir_analog[2]>=ir.an[2]){ir.an[2]=ir.ir_analog[2];}
        if(ir.ir_analog[3]>=ir.an[3]){ir.an[3]=ir.ir_analog[3];}
        if(ir.ir_analog[4]>=ir.an[4]){ir.an[4]=ir.ir_analog[4];}

        if(ir.ir_analog[0]<=ir.a1[0]){ir.a1[0]=ir.ir_analog[0];}
        if(ir.ir_analog[1]<=ir.a1[1]){ir.a1[1]=ir.ir_analog[1];}
        if(ir.ir_analog[2]<=ir.a1[2]){ir.a1[2]=ir.ir_analog[2];}
        if(ir.ir_analog[3]<=ir.a1[3]){ir.a1[3]=ir.ir_analog[3];}
        if(ir.ir_analog[4]<=ir.a1[4]){ir.a1[4]=ir.ir_analog[4];}
    }
    for(int i=0;i<5;i++){
        std::cout<<"a1["<<i<<"]"<<ir.a1[i]<<"   "<<"an["<<i<<"]"<<ir.an[i]<<std::endl;
    }
}



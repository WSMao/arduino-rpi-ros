#include<iostream>
#include "i_robot/line_follower.h"
#include <stdio.h>
#include <stdlib.h>

Mode black_mode;

pwm_struct   black_line_follow(ir_struct ir){

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

    int initial=180;

    for(int i=0;i<5;i++){
    ir.ir_digital[i]=! ir.ir_digital[i];
    }

    int cny70=Cny70_read(ir.ir_digital);    //std::cout<<cny70<<std::endl;
    //cny70=~cny70;///////////////////////////////////////
    std::cout<<cny70<<std::endl;

    uint16_t ir_ll=ir.ir_analog[0];   //讀入CNY70的類比資訊
    uint16_t ir_ml=ir.ir_analog[1];
    uint16_t ir_mm=ir.ir_analog[2];
    uint16_t ir_mr=ir.ir_analog[3];
    uint16_t ir_rr=ir.ir_analog[4];
    /////////////////////////////////////////////////////////////////////////////////////////////
    ir_ll    =ir.a1[0]+ir.an[0]-ir_ll;
    ir_ml  =ir.a1[1]+ir.an[1]-ir_ml;
    ir_mm=ir.a1[2]+ir.an[2]-ir_mm;
    ir_mr  =ir.a1[3]+ir.an[3]-ir_mr;
    ir_rr    =ir.a1[4]+ir.an[4]-ir_rr;

    std::cout<<ir_ll<<"\t"<<ir_ml<<"\t"<<ir_mm<<"\t"<<ir_mr<<"\t"<<ir_rr<<std::endl;



    if(!busy){  //如果沒有在執行特殊任務，則判斷任務

        if(cny70==0b11110 || cny70==0b11100 ){
        black_mode=WaitToTurnLeft;
        busy=1;
        wait=1;
        }

        else if(cny70==0b11000 || cny70==0b10000){
        black_mode=NowTurnLeft;
        busy=1;
        }

        else if(cny70==0b01111 || cny70==0b00111 ){
        black_mode=WaitToTurnRight;
        busy=1;
        wait=1;
        }

        else if(cny70==0b00111 || cny70==0b00001){
        black_mode=NowTurnRight;
        busy=1;
        }
        else{  black_mode =FuzzyMode; }


    }

    if(black_mode==WaitToTurnLeft){

        if(ir.ir_digital[2]==1 && wait==1){ //waiting
        std::cout<<"waiting"<<std::endl;
        pwm.pwm_l=140;
        pwm.pwm_r=140;
        }

        else{   //turning left
        wait=0;
        std::cout<<"turning left"<<std::endl;
        pwm.pwm_l= -80;//100
        pwm.pwm_r=100;//190
        if(ir.ir_digital[2]==1){black_mode=FuzzyMode;busy=0;}
        }
    }

    if(black_mode==WaitToTurnRight){

        if(ir.ir_digital[2]==1 && wait==1){ //waiting
        std::cout<<"waiting"<<std::endl;
        pwm.pwm_l=140;
        pwm.pwm_r=140;
        }

        else{   //turning right
        wait=0;
        std::cout<<"turning right"<<std::endl;
        pwm.pwm_l=100;
        pwm.pwm_r= -80;
        if(ir.ir_digital[2]==1){black_mode=FuzzyMode;busy=0;}
        }
    }

    if(black_mode==NowTurnLeft){
        std::cout<<"turning left"<<std::endl;
        pwm.pwm_l=-80;
        pwm.pwm_r=100;
        if(ir.ir_digital[2]==1){black_mode=FuzzyMode;busy=0;}
    }
    if(black_mode==NowTurnRight){
        std::cout<<"turning right"<<std::endl;
        pwm.pwm_l=100;
        pwm.pwm_r=-80;
        if(ir.ir_digital[2]==1){black_mode=FuzzyMode;busy=0;}
    }
///////////////////////////////////////////////////FuzzyMode/////////////////////////////////////////////////
    if(black_mode==FuzzyMode){
           std::cout<<"fuzzy ing"<<std::endl;

          //算出每顆感測器的側偏差
            //err_ll=err(a1_ll,an_ll,ir_ll);
            err_ml=(-1)*err(a1_ml,an_ml,ir_ml);
            //err_mm=err(a1_mm,an_mm,ir_mm);
            err_mr=(1) *err(a1_mr,an_mr,ir_mr);
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
/////////////////////////////////////////////////////////////White_Fuzzy///////////////////////////////////////////////
            float difference=Black_Fuzzy( err_new , err_rate );
/////////////////////////////////////////////////////////////White_Fuzzy///////////////////////////////////////////////
          //如果ir_mm沒有在黑線上，則初速往下調，達到過彎減速
          //  if(ir.ir_digital[2]==0){initial=220;std::cout<<"brake\t"<<std::endl;}
          //  else {initial=230;}
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


///////////////////////////////////////////////////////////////////////////////////////Fuzzy//////////////////////////////////////////////////////////////////////////
float Black_Fuzzy(float err_new, float err_rate)     //  模糊控制
{
  float difference;
  float input1=err_new,input2=err_rate;
  float weight1,weight2,weight3,weight4;
  int a1,a2,NA,j;
  int k=8;//NB=-130  NS=-25
  float NB=-130 , NM=-50 , NS=-20 , Z0=0 , PS=-NS , PM=-NM , PB=-NB ;
  float rule[8][8] = {
    //    車體偏右                                    車體偏左
    //區間1     2     3     4     5     6     7     8
    //      NB    NM    NS    Z0    PS    PM    PB        //區間
    { NA, -0.7, -0.2, -0.1,  0  ,  0.1,  0.2,  0.7 }
    ,                                                                               // 1
    { 70 ,  Z0 ,  PS ,  PM ,  PM ,  PB ,  PB ,  PB  } //PB         往左變化
    ,                                                                               // 2
    { 15 ,  NM ,  Z0 ,  PS ,  PM ,  PB ,  PB ,  PB  } //PM
    ,                                                                               // 3
    {   5 ,  NB ,  NM ,  Z0 ,  Z0 ,  PM ,  PB ,  PB  } //PS
    ,                                                                               // 4
    { 0  ,  NB ,  NM ,  NS ,  Z0 ,  PM ,  PM ,  PB  } //Z0
    ,                                                                               // 5
    {  -5 ,  NB ,  NB ,  NM ,  NM ,  Z0 ,  PM ,  PB  } //NS
    ,                                                                               // 6
    {-15 ,  NB ,  NB ,  NB ,  NS ,  NS ,  Z0 ,  PM  } //NM
    ,                                                                               // 7
    {-70 ,  NB ,  NB ,  NB ,  NM ,  NM ,  NS ,  Z0  } //NB         往右變化
                                                                               };   // 8

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

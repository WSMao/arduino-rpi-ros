#include<iostream>
#include "i_robot/line_follower.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

void select_mission(int &mission, ir_struct ir){
    int cny70=Cny70_read(ir.ir_digital);

    if(mission==1){//white line follower
//        if(cny70==0b00000){
//            cout<<"change to mission 2 now!!"<<endl;
//            sleep(1);// to see if succeed!!!
//            mission=2;
//        }
    }
    if(mission==2){//avoid and turn left

        if(cny70==0b11111){
            cout<<"change to mission 3 now!!"<<endl;
            //sleep(0.5);
            mission=3;

        }
    }

    if(mission==3){//push the button

//        if(cny70==0b00000){
//            cout<<"change to mission 4 now!!"<<endl;
//            sleep(5);
//            mission=4;

//        }
    }

}

#ifndef __SVPWM_H
#define __SVPWM_H

#include "main.h"

//SVPWM结构体
typedef struct
{
    int N;
    int na;
    int nb;
    int nc;

    float alpha;
    float beta;

    float ts;
    float t1;
    float t2;
    float t0;

    float vta;
    float vtb;
    float vtc;
} SVPWM_STRUCT;

//SVPWM结构体初始化
void my_svpwm_Init(SVPWM_STRUCT *p);
//SVPWM调制
void my_svpwm_calc(SVPWM_STRUCT *p, float alpha, float beta);
//更新占空比
void update_hrtim_duty(float dutyA, float dutyB, float dutyC);

#endif

#ifndef __INITAL_H
#define __INITAL_H
#include "suanfa.h"
#include "svpwm.h"
#include "sample.h"

typedef float fp32;
typedef double fp64;

extern FixedangleGenerator dianjiaodu;  //电角度结构体

extern CLARK_STRUCT clark;              //clark变换结构体

extern SVPWM_STRUCT SVPWM;              //SVPWM调制结构体
extern ST_ELEC_OBS  input_volt1;        //电压数据结构体

extern ST_PR PR_Volt_PhaseA; //A相电压PR控制器
extern ST_PR PR_Volt_PhaseC; //C相电压PR控制器

extern ST_PID P_Crt_PhaseA; //A相电流P控制器
extern ST_PID P_Crt_PhaseC; //C相电流P控制器

	
#endif 

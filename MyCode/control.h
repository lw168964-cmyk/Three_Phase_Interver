#ifndef __CONTROL_H

#define __CONTROL_H
#include "main.h"
#define sqrt3 1.732f
extern float Udc;//直流母线电压

extern float Ia_rms;//A想线电流有效值

extern float Uab_rms;//AB线电压有效值

void Volt_Loop_Control(float des_d,float des_q,float sita,uint16_t f); //单电压环控制

#endif

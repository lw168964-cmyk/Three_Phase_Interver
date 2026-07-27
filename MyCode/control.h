#ifndef __CONTROL_H

#define __CONTROL_H
#include "main.h"
#define sqrt3 1.732f
extern float Udc;//直流母线电压

extern float Ia_rms;//A相线电流基波有效值(OLED显示用)
extern float Ia_rms_raw;//A相线电流原始RMS(含开关纹波,排查用)

extern float Uab_rms;//AB线电压有效值

/* 控制环时序实测量(DWT cycle, 170MHz)。一个20kHz周期=8500 cycle。
   ctrl_cycles_max<8500 表示中断跑得完; ctrl_missed_trig>0 才是频率不稳的时序性证据。 */
extern volatile uint32_t ctrl_cycles;
extern volatile uint32_t ctrl_cycles_max;
extern volatile uint32_t ctrl_entry_delta;
extern volatile uint32_t ctrl_entry_delta_max;
extern volatile uint32_t ctrl_missed_trig;
extern volatile uint32_t ctrl_isr_count;
extern volatile uint32_t adc_error_count;

void Volt_Loop_Control(float des_d,float des_q,float sita,uint16_t f); //单电压环控制
//按新的基波频率重算两相PR的谐振系数(改频率时必须调,否则谐振峰留在旧频率上)
void Control_SetFundamentalFreq(float hz);
void Control_Reset(void);
void Control_Enable(void);
void Control_Disable(void);

#endif

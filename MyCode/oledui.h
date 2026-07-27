#ifndef __OLEDUI_H
#define __OLEDUI_H
#include "stm32g4xx_hal.h"
#include "OLED.h"
#include "key.h"

// -------------------------- UI状态枚举 --------------------------
typedef enum {
    UI_START = 0,        // 开始界面
    UI_Measure,   // 调频界面
    UI_Set,

} UI_State;

// -------------------------- 全局变量声明 --------------------------
extern UI_State g_current_ui;  // 当前UI界面状态
extern float Line_U1_Set;      // 输出电压（测量值：32.00V）
extern float Line_I1;    // I1干路电流（默认0.00A）
extern float Line_f1;    // 频率（默认50Hz）
extern float Line_U1_Measure;      // 输出电压（测量值：32.00V）


// -------------------------- 函数声明 --------------------------
// 初始化OLED UI系统：初始化UI状态、默认参数，显示开始界面
void OLEDUI_Init(void);

// UI界面刷新函数：根据当前UI状态，调用对应界面的显示逻辑
void OLEDUI_Refresh(void);

// UI按键处理函数：读取按键状态，根据当前UI状态执行对应操作（切换界面/调节参数）
void OLEDUI_Key_Handle(void);

#endif /* __OLEDUI_H */

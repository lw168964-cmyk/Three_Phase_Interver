#ifndef __KEY_H
#define __KEY_H

#include "stm32g4xx_hal.h"  
#include "main.h"           

/************************* 按键配置 *************************/
// 按键消抖时间（单位：ms，默认20ms，可根据硬件调整）
#define KEY_DELAY_TIME      10
// 按键电平定义：0=按下，1=释放（根据硬件接线修改：上拉输入选此定义，下拉输入则互换）
#define KEY_PRESS_LEVEL     0
#define KEY_RELEASE_LEVEL   1

/************************* 按键枚举 *************************/
// 定义按键编号，对应PB3-PB6，方便后续调用
typedef enum
{
    KEY1 = 0,  // 对应PB3
    KEY2,      // 对应PB4
    KEY3,      // 对应PB5
    KEY4,      // 对应PB6
    KEY_NUM    // 按键总数，用于循环遍历
} Key_Enum;

/************************* 函数声明 *************************/
uint8_t Key_Scan(Key_Enum key);        // 单次扫描按键（带消抖，返回当前状态）
uint8_t Key_Get_Single_Click(Key_Enum key); // 单次按键触发（按下一次仅触发一次）

#endif /* __KEY_H */

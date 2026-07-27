#include "key.h"

/************************* 按键引脚映射 *************************/
// 按键编号对应PB3-PB6引脚，直接使用HAL库GPIO读取函数
#define KEY1_GPIO_PIN  GPIO_PIN_3
#define KEY1_GPIO_PORT GPIOB
#define KEY2_GPIO_PIN  GPIO_PIN_4
#define KEY2_GPIO_PORT GPIOB
#define KEY3_GPIO_PIN  GPIO_PIN_5
#define KEY3_GPIO_PORT GPIOB
#define KEY4_GPIO_PIN  GPIO_PIN_6
#define KEY4_GPIO_PORT GPIOB

/************************* 静态全局变量 *************************/
// 按键状态锁存，用于单次触发检测（避免长按重复返回按下）
static uint8_t g_key_state[KEY_NUM] = {KEY_RELEASE_LEVEL};

/************************* 私有函数：读取单个按键原始电平 *************************/
static uint8_t Key_Read_Original(Key_Enum key)
{
    uint8_t level = KEY_RELEASE_LEVEL;
    switch(key)
    {
        case KEY1:
            level = HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN);
            break;
        case KEY2:
            level = HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN);
            break;
        case KEY3:
            level = HAL_GPIO_ReadPin(KEY3_GPIO_PORT, KEY3_GPIO_PIN);
            break;
        case KEY4:
            level = HAL_GPIO_ReadPin(KEY4_GPIO_PORT, KEY4_GPIO_PIN);
            break;
        default:
            break;
    }
    return level;
}

/************************* 公有函数：按键扫描（带消抖，返回当前状态） *************************/
// 返回值：KEY_PRESS_LEVEL(按下) / KEY_RELEASE_LEVEL(释放)
uint8_t Key_Scan(Key_Enum key)
{
    uint8_t level1, level2;
    if(key >= KEY_NUM) return KEY_RELEASE_LEVEL;
    
    // 软件消抖：连续两次读取电平一致，才判定为有效状态
    level1 = Key_Read_Original(key);
//    HAL_Delay(KEY_DELAY_TIME);  // 直接使用HAL库延时
    level2 = Key_Read_Original(key);
    
    if(level1 == level2 && level1 == KEY_PRESS_LEVEL)
    {
        return KEY_PRESS_LEVEL;  // 两次都是按下电平，判定为按键按下
    }
    else
    {
        return KEY_RELEASE_LEVEL;// 电平不一致或为释放电平，判定为按键释放
    }
}

/************************* 公有函数：单次按键触发检测（按下一次仅触发一次） *************************/
// 返回值：1(按键按下触发) / 0(无触发)
uint8_t Key_Get_Single_Click(Key_Enum key)
{
    uint8_t ret = 0;
    uint8_t current_state;
    if(key >= KEY_NUM) return 0;
    
    current_state = Key_Scan(key);  // 先扫描当前按键状态（带消抖）
    // 状态判断：上一次是释放，当前是按下 → 触发单次按键
    if(g_key_state[key] == KEY_RELEASE_LEVEL && current_state == KEY_PRESS_LEVEL)
    {
        ret = 1;
        g_key_state[key] = KEY_PRESS_LEVEL;  // 锁存为按下状态，避免长按重复触发
    }
    // 按键释放时，解锁状态
    else if(g_key_state[key] == KEY_PRESS_LEVEL && current_state == KEY_RELEASE_LEVEL)
    {
        g_key_state[key] = KEY_RELEASE_LEVEL;
    }
    
    return ret;
}

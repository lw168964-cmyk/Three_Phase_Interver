#include "oledui.h"
#include "hrtim.h"
#include "tim.h"
#include "suanfa.h"
#include "control.h"
#include "inital.h"
//                               按键位置
//
//                       |PB6|               |PB4|               
//                       确认              增加/上选
//
//                       |PB5|                 |PB3|
//                       返回              减小/下选
//
// -------------------------- 全局变量定义 --------------------------
UI_State g_current_ui = UI_START;                   // 默认初始界面为"开始界面"

// 参数（I1=0.00A，U1=32.00V，f1=50Hz）
float Line_U1_Set = 24.00f;   // 输出电压（设定值）
float Line_U1_Measure = 24.00f;   // 输出电压（测量值）(要改)
float Line_I1 = 0.00f;                              // I1干路电流
float Line_f1 = 50.00f;                             // 频率（默认50Hz）

// -------------------------- 私有函数声明--------------------------------
static void UI_Show_Start(void);                    // 显示"开始界面"
static void UI_Show_Measure(void);               // 显示"参数显示界面"
static void UI_Show_Set(void);               // 显示"参数显示界面"
// -------------------------- UI初始化函数 -------------------------------
void OLEDUI_Init(void) {
    OLED_Init();          // 初始化OLED
    OLED_Clear();         // 清屏防止花屏
    g_current_ui = UI_START; // 初始化为开始界面
    OLEDUI_Refresh();     // 刷新显示初始界面
}

// -------------------------- UI刷新核心函数 -----------------------------
void OLEDUI_Refresh(void) {
    OLED_Clear();  // 每次刷新前清屏（避免界面残留）
    
    // 根据当前UI状态，调用对应界面的显示函数
    switch (g_current_ui) {
        case UI_START:
            UI_Show_Start();
            break;
        case UI_Set:
            UI_Show_Set();
            break;
        case UI_Measure:
            UI_Show_Measure();
            break;
        default:
            g_current_ui = UI_START; // 异常状态下返回开始界面
            UI_Show_Start();
            break;
    }
    
    OLED_Update(); // 更新OLED显存到屏幕
}

// -------------------------- 按键处理核心函数 --------------------------
void OLEDUI_Key_Handle(void) {
    // 读取按键状态（使用已有Key驱动的单次触发函数，避免长按重复触发）
    uint8_t key_up = Key_Get_Single_Click(KEY2);    // PB4（KEY2）= 增加/上移
    uint8_t key_down = Key_Get_Single_Click(KEY1);  // PB3（KEY1）= 减少/下移
    uint8_t key_ok = Key_Get_Single_Click(KEY4);    // PB6（KEY4）= 确认
    uint8_t key_back = Key_Get_Single_Click(KEY3);  // PB5（KEY3）= 返回

    // 根据当前UI状态，处理按键逻辑
    switch (g_current_ui) {
        // -------------------------- 1. 开始界面按键逻辑 --------------------------
        case UI_START:

            if (key_ok) {          // 确认：根据选中项进入对应参数设定界面
                g_current_ui = UI_Set;  
                OLEDUI_Refresh(); // 刷新到参数设定界面
            }
            if (key_back) {        // 返回：回到开始界面
                g_current_ui = UI_START;
                OLEDUI_Refresh();
            }
            break;

            // -------------------------- 2. 参数设定显示界面按键逻辑 --------------------------
        case UI_Set:
            if (key_up && Line_f1 <= 100) {  // 增加：频率+1（上限100Hz）
                Line_f1 = Line_f1 + 1.0f;
							fixed_angle_init(&dianjiaodu,Line_f1,10000); //初始化电角度结构体
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 >= 20) { // 减少：频率-1（下限20Hz）
                Line_f1 = Line_f1 - 1.0f;
							fixed_angle_init(&dianjiaodu,Line_f1,10000); //初始化电角度结构体
                OLEDUI_Refresh(); // 刷新参数显示
							
            }
            if (key_ok) {          // 确认：进入单网测量界面
                g_current_ui = UI_Measure;
								//控制环路已改由HRTIM波峰->ADC-DMA完成回调驱动(见control.c),不再用TIM6

								HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER);//开启通道输出
								HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_A);
								HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
								HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);


								HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_B);
								HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
								HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_B,HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
								
								HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_TIMER_C);
								HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
								HAL_HRTIM_SimplePWMStart(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C,HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：回到开始界面
                g_current_ui = UI_START;
                OLEDUI_Refresh();
            }
            break;
            // -------------------------- 3. 参数测量显示界面按键逻辑 --------------------------
        case UI_Measure:
            if (key_up && Line_f1 <= 100) {  // 增加：频率+1（上限100Hz）
                Line_f1 = Line_f1 + 1.0f;
							fixed_angle_init(&dianjiaodu,Line_f1,10000); //初始化电角度结构体
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 >= 20) { // 减少：频率-1（下限20Hz）
                Line_f1 = Line_f1 - 1.0f;
							fixed_angle_init(&dianjiaodu,Line_f1,10000); //初始化电角度结构体
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_ok) {          // 确认：进入单网测量界面
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：回到开始界面
                g_current_ui = UI_Set;
								//停HRTIM即停ADC波峰触发,控制环路随之停止,无需再操作TIM6
								HAL_HRTIM_WaveformCounterStop(&hhrtim1,HRTIM_TIMERID_MASTER);
								HAL_HRTIM_WaveformCounterStop(&hhrtim1,HRTIM_TIMERID_TIMER_A);
								HAL_HRTIM_WaveformCounterStop(&hhrtim1,HRTIM_TIMERID_TIMER_B);
								HAL_HRTIM_WaveformCounterStop(&hhrtim1,HRTIM_TIMERID_TIMER_C);
								HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
								HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);
								HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
                OLEDUI_Refresh();
            }
            break;
        default:
            break;
      }
    }   
    
// -------------------------- 私有界面显示函数-------------------------------------------
/**
 * 显示开始界面：显示系统标题和当前参数值
 * 界面布局：
 * - 标题："三相变流系统1"（X=15,Y=0）
 * - 参数：U1, I1, f1 显示在相应位置
 */
static void UI_Show_Start(void) {
    // 显示标题
    OLED_ShowString(15, 0, "三相变流系统1", OLED_8X16);

    OLED_ShowString(20, 16, "U1:", OLED_8X16);
    OLED_ShowFloatNum(50, 16, Line_U1_Set, 2, 2, OLED_8X16);
    OLED_ShowString(20, 32, "I1:", OLED_8X16);
    OLED_ShowFloatNum(50, 32, Line_I1, 1, 1, OLED_8X16);
    OLED_ShowString(20, 48, "f1:", OLED_8X16);
    OLED_ShowFloatNum(50, 48, Line_f1, 2, 1, OLED_8X16);

    

}

/**
 * 显示调频界面：显示参数并允许调节频率f1
 * 界面布局：
 * - 标题："调频界面"（X=15,Y=0）
 * - 参数：U1, I1, f1 显示在相应位置
 * - 操作：上键增加频率，下键减少频率
 */
static void UI_Show_Set(void) {
    // 显示标题
    OLED_ShowString(15, 0, "调频界面", OLED_8X16);
    OLED_ShowString(20, 16, "U1:", OLED_8X16);
    OLED_ShowFloatNum(50, 16, Line_U1_Set, 2, 2, OLED_8X16);
    OLED_ShowString(20, 32, "I1:", OLED_8X16);
    OLED_ShowFloatNum(50, 32, Line_I1, 1, 1, OLED_8X16);
    OLED_ShowString(20, 48, "f1:", OLED_8X16);
    OLED_ShowFloatNum(50, 48, Line_f1, 2, 1, OLED_8X16);

}

/**
 * 显示测量界面：显示参数并允许调节频率f1
 * 界面布局：
 * - 标题："测量界面"（X=15,Y=0）
 * - 参数：U1, I1, f1 显示在相应位置
 * - 操作：上键增加频率，下键减少频率
 */
static void UI_Show_Measure(void) {
    // 显示标题 
    OLED_ShowString(15, 0, "测量界面", OLED_8X16);
    OLED_ShowString(20, 16, "U1:", OLED_8X16);
    OLED_ShowFloatNum(50, 16,Uab_rms , 2, 2, OLED_8X16);
    OLED_ShowString(20, 32, "I1:", OLED_8X16);
    OLED_ShowFloatNum(50, 32,Ia_rms , 2, 2, OLED_8X16);
    OLED_ShowString(20, 48, "f1:", OLED_8X16);
    OLED_ShowFloatNum(50, 48, Line_f1, 2, 1, OLED_8X16);

}

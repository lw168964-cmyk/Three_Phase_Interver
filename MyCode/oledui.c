#include "oledui.h"
#include "hrtim.h"
#include "tim.h"
#include "suanfa.h"
#include "control.h"
#include "inital.h"
#include "svpwm.h"
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

#define HRTIM_ACTIVE_OUTPUTS \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | \
     HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)
#define HRTIM_ACTIVE_COUNTERS \
    (HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | \
     HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C)
#define HRTIM_PWM_UPDATES \
    (HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_C)
#define HRTIM_PWM_RESETS \
    (HRTIM_TIMERRESET_MASTER | HRTIM_TIMERRESET_TIMER_A | \
     HRTIM_TIMERRESET_TIMER_B | HRTIM_TIMERRESET_TIMER_C)

// -------------------------- 私有函数声明--------------------------------
static void UI_Show_Start(void);                    // 显示"开始界面"
static void UI_Show_Measure(void);               // 显示"参数显示界面"
static void UI_Show_Set(void);               // 显示"参数显示界面"
static void HRTIM_ApplyNeutralState(void);
static void HRTIM_ForcePrimaryOutputsInactive(void);

/* Outputs must be disabled before this function is called. */
static void HRTIM_ApplyNeutralState(void)
{
    update_hrtim_duty(0.5f, 0.5f, 0.5f);

    /* Apply the preload and reset all counters, so the next start begins at
       a known period boundary instead of resuming from the former stop phase. */
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_PWM_UPDATES) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_PWM_RESETS) != HAL_OK)
    {
        Error_Handler();
    }
}

/* With dead-time insertion enabled, force the primary output state before
   entering RUN so HRTIM establishes the complementary state deterministically. */
static void HRTIM_ForcePrimaryOutputsInactive(void)
{
    if ((HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                           HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) ||
        (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                           HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) ||
        (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
                                           HRTIM_OUTPUT_TC1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK))
    {
        Error_Handler();
    }
}
/* 改频率时统一走这里:角度发生器 + PR谐振系数必须一起更新。
   原先只调fixed_angle_init(),PR的omiga_0留在2*pi*50不动,偏离50Hz后
   |PR|从30.2掉到个位数,退化成Kp=0.2的纯比例。

   与fixed_angle_init()的两点区别:
   1. 保留theta。fixed_angle_init会把theta清零,运行中改频率就是一次相位跳变,
      对2mH/9.9uF这种ζ极小的LC是一记阶跃冲击。改频率只该改角速度,不该动当前相位。
   2. 整组更新期间屏蔽中断。ISR每周期读theta/w/Fo,非原子更新会让某一拍用到
      新w配旧Fo(RMS窗口)的混搭状态。 */
void OLEDUI_Apply_Freq(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    dianjiaodu.w  = 2.0f * 3.1415926f * Line_f1;
    dianjiaodu.Ts = 1.0f / (float)CTRL_FREQUENCY;
    dianjiaodu.Fo = (uint16_t)Line_f1;
    if (primask == 0U)
    {
        __enable_irq();
    }

    Control_SetFundamentalFreq(Line_f1);
}

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
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 >= 20) { // 减少：频率-1（下限20Hz）
                Line_f1 = Line_f1 - 1.0f;
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示

            }
            if (key_ok) {          // 确认：进入单网测量界面
                g_current_ui = UI_Measure;
								/* Start from a zero line-voltage state. The first ADC DMA callback
								   ramps the closed loop instead of reusing stale controller state. */
								Control_Reset();
								HRTIM_ApplyNeutralState();
								if (HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_ACTIVE_OUTPUTS) != HAL_OK)
								{
									Error_Handler();
								}
								HRTIM_ForcePrimaryOutputsInactive();
								if (HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_ACTIVE_COUNTERS) != HAL_OK)
								{
									HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_ACTIVE_OUTPUTS);
									Error_Handler();
								}
								Control_Enable();
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
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 >= 20) { // 减少：频率-1（下限20Hz）
                Line_f1 = Line_f1 - 1.0f;
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_ok) {          // 确认：进入单网测量界面
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：回到开始界面
                g_current_ui = UI_Set;
								/* Outputs must be disabled before a waveform counter can stop. */
								Control_Disable();
								if (HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_ACTIVE_OUTPUTS) != HAL_OK)
								{
									Error_Handler();
								}
								if (HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_ACTIVE_COUNTERS) != HAL_OK)
								{
									Error_Handler();
								}
								Control_Reset();
								HRTIM_ApplyNeutralState();
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

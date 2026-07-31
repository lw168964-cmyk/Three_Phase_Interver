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
//线电压有效值给定。运行中可在 UI_VoltSet 界面用 PB4/PB3 以 0.05V 步进调节。
//折算成相电压峰值在control.c的调用处完成: Line_U1_Set*0.8164966
float Line_U1_Set = 32.00f;   // 输出电压（设定值）

/* ===== 电压给定的步进与上下限 =====
 * 两个步进值对应两个使用场景, 上下限共用:
 *   开始界面(未启动) 0.02V —— 开机前预置目标值, 细一点方便一次到位
 *   调压界面(运行中) 0.05V —— 运行中现场微调, 粗一点少按几次
 * 步进 0.05V 相当于 32V 上的 0.16%, PR环几个基波周期内跟上; 慢环误差项按给定值
 * 归一化, 所以 rms_trim 会自己重新收敛, 不需要复位。
 *
 * 上限 36.00V 的来历(不是随便取的整数):
 *   control.c 把给定折算成相电压峰值 Line_U1_Set*sqrt2/sqrt3, 再乘 sqrt3/Udc 归一化,
 *   于是 m = Line_U1_Set*sqrt2/Udc = Line_U1_Set/42.43。
 *   CONTROL_MODULATION_LIMIT=0.85 -> 给定上限 0.85*42.43 = 36.07V。
 *   取 36.00V: 再往上调制限幅就会削顶并触发抗积分饱和, 给定加了但输出不跟,
 *   界面上看着能加实际加不上去 —— 所以在这里就拦住。
 *   注意这条上限跟着 Udc 走(60V母线), 换母线电压必须重算。
 * 下限 20.00V: 低于此值 m<0.47, 死区压降占比变大且无实际用途。 */
#define VOLT_SET_STEP        0.05f   //运行中(调压界面)步进
#define VOLT_INIT_STEP       0.02f   //开机前(开始界面)步进
#define VOLT_SET_MIN    20.00f
#define VOLT_SET_MAX    36.00f
float Line_U1_Measure = 32.00f;   // 输出电压（测量值）(未使用,显示走Uab_rms)
float Line_I1 = 0.00f;                              // I1干路电流
float Line_f1 = 50.00f;                             // 频率（默认50Hz）
extern uint16_t ADC1_Value[4];

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
static void UI_Show_VoltSet(void);           // 显示"调压界面"
static void OLEDUI_Stop_Output(void);        // 停机(UI_Measure/UI_VoltSet 共用)
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
        case UI_VoltSet:
            UI_Show_VoltSet();
            break;
        default:
            g_current_ui = UI_START; // 异常状态下返回开始界面
            UI_Show_Start();
            break;
    }
    
    OLED_Update(); // 更新OLED显存到屏幕
}

/* 停机序列。UI_Measure 与 UI_VoltSet 同属运行态, 两处按PB5都要走同一套流程,
   提出来避免两份拷贝走偏(顺序错了会留下带电的输出)。 */
static void OLEDUI_Stop_Output(void)
{
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
            /* 开机前预置目标线电压。此时 HRTIM 未启动、控制未使能, 改的只是一个
               普通全局量, 等 UI_Set 按 PB6 启动时软起动会从0爬到这个给定。
               上下限与运行中调压界面共用(见 VOLT_SET_MIN/MAX 的推导)。 */
            if (key_up) {          // PB4: 目标线电压 +0.02V
                Line_U1_Set += VOLT_INIT_STEP;
                if (Line_U1_Set > VOLT_SET_MAX) { Line_U1_Set = VOLT_SET_MAX; }
                OLEDUI_Refresh();
            }
            if (key_down) {        // PB3: 目标线电压 -0.02V
                Line_U1_Set -= VOLT_INIT_STEP;
                if (Line_U1_Set < VOLT_SET_MIN) { Line_U1_Set = VOLT_SET_MIN; }
                OLEDUI_Refresh();
            }
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
            /* 边界用 < / > 而非 <= / >=: 原写法在 Line_f1 恰为 100 时还允许 +1 到 101,
               恰为 20 时还允许 -1 到 19。19Hz 一个周期 1052 点, 会顶到 RMS 窗口
               的点数上限, 窗口不再是整周期, 有效值读数随之偏掉。 */
            if (key_up && Line_f1 < 100) {  // 增加：频率+1（上限100Hz）
                Line_f1 = Line_f1 + 1.0f;
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 > 20) { // 减少：频率-1（下限20Hz）
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
								/* 计数器和输出都起来后抄一次寄存器, 供调试器 watch hrtim_regs 判读(见 hrtim.c)。 */
								HRTIM_CaptureRegs();
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：回到开始界面
                g_current_ui = UI_START;
                OLEDUI_Refresh();
            }
            break;
            // -------------------------- 3. 参数测量显示界面按键逻辑 --------------------------
        case UI_Measure:
            //边界同 UI_Set, 见那里的说明
            if (key_up && Line_f1 < 100) {  // 增加：频率+1（上限100Hz）
                Line_f1 = Line_f1 + 1.0f;
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_down && Line_f1 > 20) { // 减少：频率-1（下限20Hz）
                Line_f1 = Line_f1 - 1.0f;
							OLEDUI_Apply_Freq();
                OLEDUI_Refresh(); // 刷新参数显示
            }
            if (key_ok) {          // 确认(PB6)：切到调压界面, 输出保持运行不打断
                g_current_ui = UI_VoltSet;
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：停机并回到调频设定界面
                g_current_ui = UI_Set;
                OLEDUI_Stop_Output();
                OLEDUI_Refresh();
            }
            break;
            // -------------------------- 4. 调压界面按键逻辑(运行中) --------------------------
            /* 与 UI_Measure 是同一个运行态的两个视图: 这里不碰 HRTIM 也不碰控制使能,
               只改给定值, 所以 PB6 来回切换不会打断输出。
               Line_U1_Set 是4字节对齐的float, M4上单次写入本身原子, 控制中断读到的
               要么是旧值要么是新值, 不会撕裂 —— 所以不需要像改频率那样屏蔽中断
               (改频率要同时更新 w/Fo/谐振系数多个量, 那才必须成组保护)。 */
        case UI_VoltSet:
            if (key_up) {          // PB4: 线电压有效值 +0.05V
                Line_U1_Set += VOLT_SET_STEP;
                if (Line_U1_Set > VOLT_SET_MAX) { Line_U1_Set = VOLT_SET_MAX; }
                OLEDUI_Refresh();
            }
            if (key_down) {        // PB3: 线电压有效值 -0.05V
                Line_U1_Set -= VOLT_SET_STEP;
                if (Line_U1_Set < VOLT_SET_MIN) { Line_U1_Set = VOLT_SET_MIN; }
                OLEDUI_Refresh();
            }
            if (key_ok) {          // 确认(PB6)：切回调频界面
                g_current_ui = UI_Measure;
                OLEDUI_Refresh();
            }
            if (key_back) {        // 返回：与调频界面一致, 停机回到设定界面
                g_current_ui = UI_Set;
                OLEDUI_Stop_Output();
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
    //整数位必须>=2: OLED_ShowNum按IntLength截位, 写1时32.00会显示成"2.000"
    // OLED_ShowFloatNum(50, 16,(float)(ADC1_Value[1])*3.3f/4096.0f, 1, 3, OLED_8X16);
    OLED_ShowFloatNum(50, 16,Uab_rms, 2, 3, OLED_8X16);
    OLED_ShowString(20, 32, "I1:", OLED_8X16);
    // OLED_ShowFloatNum(50, 32,(float)(ADC1_Value[3])*3.3f/4096.0f, 1    , 3, OLED_8X16);
    OLED_ShowFloatNum(50, 32,Ia_rms , 2, 2, OLED_8X16);
    OLED_ShowString(20, 48, "f1:", OLED_8X16);
    OLED_ShowFloatNum(50, 48, Line_f1, 2, 1, OLED_8X16);

}

/**
 * 显示调压界面(运行中)：PB4/PB3 以 0.05V 步进调线电压给定, PB6 切回调频界面
 * 界面布局：
 * - 标题："调压界面"（X=15,Y=0）
 * - Us: 给定值   Um: 实测值(Uab_rms)   f1: 当前频率
 * 给定与实测并排显示是有意的: 调压时要盯着两个数一起看, 才知道给定加上去以后
 * 实测跟没跟上(碰到调制限幅时给定会继续加而实测不动)。
 */
static void UI_Show_VoltSet(void) {
    // 显示标题
    OLED_ShowString(15, 0, "调压界面", OLED_8X16);
    OLED_ShowString(20, 16, "Us:", OLED_8X16);
    OLED_ShowFloatNum(50, 16, Line_U1_Set, 2, 2, OLED_8X16);
    OLED_ShowString(20, 32, "Um:", OLED_8X16);
    //整数位必须>=2(见UI_Show_Measure的说明)
    OLED_ShowFloatNum(50, 32, Uab_rms, 2, 3, OLED_8X16);
    OLED_ShowString(20, 48, "f1:", OLED_8X16);
    OLED_ShowFloatNum(50, 48, Line_f1, 2, 1, OLED_8X16);

}

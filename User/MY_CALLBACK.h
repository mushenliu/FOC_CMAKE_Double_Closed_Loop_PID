#ifndef FOC_MY_CALLBACK_H
#define FOC_MY_CALLBACK_H

#include "main.h"

//数学常数
#define SQRT3 1.7320508065

//电机极对数
#define MOTOR_POLE_PAIRS 14

//速度环一阶低通滤波系数
#define Speed_Filter 0.1

//D轴电流环连续域PID参数
#define D_Kp 30
#define D_Ki 10000
#define D_Kd 0
#define D_N 10000
#define D_ANTI_SAT (D_Kp/D_Ki)

//Q轴电流环连续域PID参数
#define Q_Kp 30
#define Q_Ki 10000
#define Q_Kd 0
#define Q_N 10000
#define Q_ANTI_SAT (Q_Kp/Q_Ki)

//速度环连续域PID参数
#define Speed_Kp 0.001
#define Speed_Ki 0.005
#define Speed_Kd 0
#define Speed_N 100
#define Speed_ANTI_SAT (1/Speed_Ki)

//速度环和电流环采样周期，单位s
#define Ts_Current ((__HAL_TIM_GET_AUTORELOAD(&htim1)+1)/170000000.0) * 2
#define Ts_Speed    ((__HAL_TIM_GET_AUTORELOAD(&htim2)+1)/170000000.0)

//默认母线电压，单位V
#define U_DC_Default 12

//默认给定电流，单位A
#define ID_Target_Default 0
#define IQ_Target_Default 0
//默认给定转速，单位rpm
#define Speed_Target_Default 0

//速度环输出限幅（电流环给定限幅）
#define Speed_Output_Limit 0.8

//速度环给定限幅
#define Speed_Target_Limit 400

//PID控制律结构体
typedef struct {
    float a1, a2;
    float b0, b1, b2;
    float Error_Record[2];
    float Output_Record[2];
    float Error_Now;
    float Output_Now;
    float Setvalue;
} Discrete_PID_Struct;

//ADC注入组转换完成中断回调函数——获取电流值
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

//TIM1CH4比较中断回调函数——获取角度值，电流环控制入口
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

//TIM2更新终端回调函数——速度环入口
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//UART接收完成中断——更改指令值
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif

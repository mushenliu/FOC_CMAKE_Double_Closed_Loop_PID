#ifndef FOC_MY_CONTROL_H
#define FOC_MY_CONTROL_H

#include "main.h"
#include "MY_CALLBACK.h"

//克拉克变换、帕克变换、反克拉克变换、反帕克变换
void Clarke_Trans(float A, float B, float C, float *alpha, float *beta);

void Inv_Clarke_Trans(float alpha, float beta, float *A, float *B, float *C);

void Park_Trans(float alpha, float beta, float Sin, float Cos, float *D, float *Q);

void Inv_Park_Trans(float D, float Q, float Sin, float Cos, float *alpha, float *beta);

//使用CORDIC计算三角函数
void CORDIC_Calc_SinCos(float theta, float *Sin, float *Cos);

//使用DSP浮点计算三角函数
void DSP_Float_Calc_SinCos(float theta, float *Sin, float *Cos);

//使用DSP定点计算三角函数
void DSP_Fixed_Calc_SinCos(float theta, float *Sin, float *Cos);

//Q31到float类型转化
void Q31_to_Float(int Q31, float *Data);

//SVPWM调制法计算三相占空比
void SVPWM_Modulation(float Ud, float Uq, float Sin, float Cos, float Udc, float *Duty_A, float *Duty_B, float *Duty_C);

//SVPWM计算法计算三相占空比
void SVPWM_Calculation(float *Ud, float *Uq, float Sin, float Cos, float Udc, float *Duty_A, float *Duty_B,
                       float *Duty_C);

//将三相占空比转换为CCR值并更新TIM
void Set_CCR(float Duty_A, float Duty_B, float Duty_C);

//PID控制器
void Discrete_PID_Controller(Discrete_PID_Struct *PID);

//电流环控制函数
void Current_Control();

//速度环控制函数
void Speed_Control();

//PID结构体初始化函数
void PID_Struct_Init(float Kp, float Ki, float Kd, float N, float Ts, float Default_Set, Discrete_PID_Struct *PID);

#endif //FOC_MY_CONTROL_H

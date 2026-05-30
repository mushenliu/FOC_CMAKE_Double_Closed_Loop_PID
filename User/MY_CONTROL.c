#include "MY_CONTROL.h"
#include "MY_JUSTFLOAT.h"

void Clarke_Trans(float A, float B, float C, float *alpha, float *beta) {
    // *alpha = 2 / 3.0 * (A - 0.5 * B - 0.5 * C);
    // *beta = 1 / 3.0 * (SQRT3 * B - SQRT3 * C);
    arm_clarke_f32(A, B, alpha, beta);
}

void Inv_Clarke_Trans(float alpha, float beta, float *A, float *B, float *C) {
    // *A = alpha;
    // *B = -0.5 * alpha + SQRT3 / 2.0 * beta;
    // *C = -0.5 * alpha - SQRT3 / 2.0 * beta;
    arm_inv_clarke_f32(alpha, beta, A, B);
    *C = 0 - *A - *B;
}

void Park_Trans(float alpha, float beta, float Sin, float Cos, float *D, float *Q) {
    // DSP_Float_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    //DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    //CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // *D = alpha * cos_theta + beta * sin_theta;
    // *Q = -alpha * sin_theta + beta * cos_theta;
    arm_park_f32(alpha, beta, D, Q, Sin, Cos);
}

void Inv_Park_Trans(float D, float Q, float Sin, float Cos, float *alpha, float *beta) {
    //DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    //CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // *alpha = D * cos_theta - Q * sin_theta;
    // *beta = D * sin_theta + Q * cos_theta;
    arm_inv_park_f32(D, Q, alpha, beta, Sin, Cos);
}

void CORDIC_Calc_SinCos(float theta, float *Sin, float *Cos) {
    int32_t OutputBuff[2];
    int32_t cordic31[2];
    while (theta > 360) {
        theta = theta - 360;
    }
    float theta_sin = 90 + theta;
    float theta_cos = theta / 360;
    theta_sin = theta_sin / 360;
    if (theta_cos > 0.5) {
        theta_cos = theta_cos - 1;
    }
    if (theta_sin > 0.5) {
        theta_sin = theta_sin - 1;
    }
    cordic31[0] = (int32_t) ((theta_cos / 0.5f) * 0x80000000); //value对coeff归一化，然后扩大2^31倍，取整得到Q31定点数据
    cordic31[1] = (int32_t) ((theta_sin / 0.5f) * 0x80000000);

    HAL_CORDIC_Calculate(&hcordic, cordic31, OutputBuff, 2, 0);

    Q31_to_Float(OutputBuff[0], Cos);
    Q31_to_Float(-OutputBuff[1], Sin);
}

void DSP_Float_Calc_SinCos(float theta, float *Sin, float *Cos) {
    arm_sin_cos_f32(theta, Sin, Cos);
}

void DSP_Fixed_Calc_SinCos(float theta, float *Sin, float *Cos) {
    int32_t OutputBuff[2];
    int32_t cordic31;
    while (theta > 360) {
        theta = theta - 360;
    }
    float theta_cos = theta / 360;
    if (theta_cos > 0.5) {
        theta_cos = theta_cos - 1;
    }
    cordic31 = (int32_t) ((theta_cos / 0.5f) * 0x80000000); //value对coeff归一化，然后扩大2^31倍，取整得到Q31定点数据


    arm_sin_cos_q31(cordic31, &OutputBuff[0], &OutputBuff[1]);


    Q31_to_Float(OutputBuff[0], Sin);
    Q31_to_Float(OutputBuff[1], Cos);
}

void Q31_to_Float(int Q31, float *Data) {
    if (Q31 & 0x80000000) //为负数
    {
        Q31 = Q31 & 0x7fffffff;
        *Data = ((float) (Q31) - 0x80000000) / 0x80000000;
    } else //为正数
    {
        *Data = (float) (Q31) / 0x80000000;
    }
}

void SVPWM_Modulation(float Ud, float Uq, float Sin, float Cos, float Udc, float *Duty_A, float *Duty_B,
                      float *Duty_C) {
    float U_alpha, U_beta = 0;
    float A0, A1, A2 = 0;
    float K = SQRT3 / Udc;
    float T_U1, T_U2, T_U0 = 0;
    float D_0, D_1, D_2;
    uint8_t N = 0;
    Inv_Park_Trans(Ud, Uq, Sin, Cos, &U_alpha, &U_beta);
    A0 = U_beta;
    A1 = SQRT3 / 2 * U_alpha - 0.5 * U_beta;
    A2 = -SQRT3 / 2 * U_alpha - 0.5 * U_beta;
    N = (A0 > 0 ? 1 : 0) + 2 * (A1 > 0 ? 1 : 0) + 4 * (A2 > 0 ? 1 : 0);
    switch (N) {
        case 3: {
            T_U1 = A1 * K;
            T_U2 = A0 * K;
            break;
        }
        case 1: {
            T_U1 = -A1 * K;
            T_U2 = -A2 * K;
            break;
        }
        case 5: {
            T_U1 = A0 * K;
            T_U2 = A2 * K;
            break;
        }
        case 4: {
            T_U1 = -A0 * K;
            T_U2 = -A1 * K;
            break;
        }
        case 6: {
            T_U1 = A2 * K;
            T_U2 = A1 * K;
            break;
        }
        case 2: {
            T_U1 = -A2 * K;
            T_U2 = -A0 * K;
            break;
        }
    }
    T_U0 = 1 - T_U1 - T_U2;
    D_0 = T_U0 / 2;
    D_1 = D_0 + T_U2;
    D_2 = D_1 + T_U1;
    switch (N) {
        case 3: {
            *Duty_A = D_2;
            *Duty_B = D_1;
            *Duty_C = D_0;
            break;
        }
        case 1: {
            *Duty_A = D_1;
            *Duty_B = D_2;
            *Duty_C = D_0;
            break;
        }
        case 5: {
            *Duty_A = D_0;
            *Duty_B = D_2;
            *Duty_C = D_1;
            break;
        }
        case 4: {
            *Duty_A = D_0;
            *Duty_B = D_1;
            *Duty_C = D_2;
            break;
        }
        case 6: {
            *Duty_A = D_1;
            *Duty_B = D_0;
            *Duty_C = D_2;
            break;
        }
        case 2: {
            *Duty_A = D_2;
            *Duty_B = D_0;
            *Duty_C = D_1;
            break;
        }
    }
}

void SVPWM_Calculation(float *Ud, float *Uq, float Sin, float Cos, float Udc, float *Duty_A, float *Duty_B,
                       float *Duty_C) {
    float U_alpha, U_beta = 0;
    float U_ABC[3] = {0};
    float U_max, U_min = 0;
    float U_0 = 0;
    uint32_t p = 0;
    float u_mag = 0;
    if (Udc == 0) {
        return;
    }
    arm_sqrt_f32((*Ud) * (*Ud) + (*Uq) * (*Uq), &u_mag);
    if (u_mag > Udc / SQRT3) {
        u_mag = Udc / SQRT3 / u_mag;
    } else {
        u_mag = 1;
    }
    *Ud = (*Ud) * u_mag;
    *Uq = (*Uq) * u_mag;
    Inv_Park_Trans(*Ud, *Uq, Sin, Cos, &U_alpha, &U_beta);
    Inv_Clarke_Trans(U_alpha, U_beta, &U_ABC[0], &U_ABC[1], &U_ABC[2]);
    arm_max_f32(U_ABC, 3, &U_max, &p);
    arm_min_f32(U_ABC, 3, &U_min, &p);
    U_0 = -0.5 * (U_max + U_min);
    *Duty_A = 0.5 + (U_0 + U_ABC[0]) / Udc;
    *Duty_B = 0.5 + (U_0 + U_ABC[1]) / Udc;
    *Duty_C = 0.5 + (U_0 + U_ABC[2]) / Udc;
}

void Set_CCR(float Duty_A, float Duty_B, float Duty_C) {
    int ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ARR*Duty_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ARR*Duty_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ARR*Duty_C);
}

void Discrete_PID_Controller(Discrete_PID_Struct *PID) {
    float Temp_Output = 0;
    Temp_Output += PID->a1 * PID->Output_Record[0];
    Temp_Output += PID->a2 * PID->Output_Record[1];
    Temp_Output += PID->b0 * PID->Error_Now;
    Temp_Output += PID->b1 * PID->Error_Record[0];
    Temp_Output += PID->b2 * PID->Error_Record[1];
    PID->Output_Now = Temp_Output;
    PID-> Output_Record[1] = PID->Output_Record[0];
    PID-> Output_Record[0] = PID->Output_Now;
    PID-> Error_Record[1] = PID->Error_Record[0];
    PID-> Error_Record[0] = PID->Error_Now;
}

void Current_Control() {
    //机械角度
    extern float theta;
    extern float Angel_ZERO;
    //电角度
    extern float theta_e;
    //三相自然坐标系
    extern float Current_abc[3];
    //两相静止坐标
    extern float alpha;
    extern float beta;
    //同步旋转坐标系
    extern float D;
    extern float Q;
    //三相占空比
    extern float Duty_A;
    extern float Duty_B;
    extern float Duty_C;
    //DQ轴电压
    extern float Ud;
    extern float Uq;
    //母线电压
    extern float Udc;
    //DQ轴电流控制器
    extern Discrete_PID_Struct D_PID;
    extern Discrete_PID_Struct Q_PID;
    //三角函数
    extern float Sin;
    extern float Cos;
    //计算电角度
    theta_e = theta * MOTOR_POLE_PAIRS + Angel_ZERO;
    while (theta_e > 360) {
        theta_e -= 360;
    }
    while (theta_e < 0) {
        theta_e += 360;
    }
    //计算三角函数
    DSP_Float_Calc_SinCos(theta_e, &Sin, &Cos);
    //计算电流
    Clarke_Trans(Current_abc[0], Current_abc[1], Current_abc[2], &alpha, &beta);
    Park_Trans(alpha, beta, Sin, Cos, &D, &Q);
    //PID计算
    D_PID.Error_Now = D_PID.Setvalue - D;
    Q_PID.Error_Now = Q_PID.Setvalue - Q;
    Discrete_PID_Controller(&D_PID);
    Discrete_PID_Controller(&Q_PID);
    Ud = D_PID.Output_Now;
    Uq = Q_PID.Output_Now;
    //SVPWM调制
    SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
    Q_PID.Output_Now = Uq;
    Q_PID.Output_Record[0] = Q_PID.Output_Now;
    D_PID.Output_Now = Ud;
    D_PID.Output_Record[0] = D_PID.Output_Now;
    //设定CCR值
    Set_CCR(Duty_A, Duty_B, Duty_C);
    HAL_GPIO_WritePin(Test_GPIO_Port,Test_Pin, GPIO_PIN_RESET);
}

void Speed_Control() {
    //DQ轴电流控制器
    extern Discrete_PID_Struct D_PID;
    extern Discrete_PID_Struct Q_PID;
    //速度环控制器
    extern Discrete_PID_Struct Speed_PID;
    //当前角速度
    extern float wm;

    //PID计算
    Speed_PID.Error_Now = Speed_PID.Setvalue - wm;
    Discrete_PID_Controller(&Speed_PID);
    //采用Id=0，Iq给定的控制策略
    if (Speed_PID.Output_Now < -Speed_Output_Limit) {
        Speed_PID.Output_Now = -Speed_Output_Limit;
    } else if (Speed_PID.Output_Now > Speed_Output_Limit) {
        Speed_PID.Output_Now = Speed_Output_Limit;
    }
    Q_PID.Setvalue = Speed_PID.Output_Now;
    Speed_PID.Output_Record[0] = Speed_PID.Output_Now;
}

void PID_Struct_Init(float Kp, float Ki, float Kd, float N, float Ts, float Default_Set, Discrete_PID_Struct *PID) {
    float P = Kp;
    float I = Ki * Ts / 2.0;
    float D = (2 * Kd * N) / (N * Ts + 2.0);
    float Nd = (N * Ts - 2.0) / (N * Ts + 2.0);

    PID->Output_Now = 0;
    PID->Output_Record[0] = 0;
    PID->Output_Record[1] = 0;

    PID->Error_Now = 0;
    PID->Error_Record[0] = 0;
    PID->Error_Record[1] = 0;

    PID->Setvalue = Default_Set;

    PID->a1 = 1 - Nd;
    PID->a2 = Nd;
    PID->b0 = P + I + D;
    PID->b1 = P * Nd - P + I * Nd + I - 2 * D;
    PID->b2 = D + I * Nd - P * Nd;
}

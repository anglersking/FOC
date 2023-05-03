#include "math.h"

#define PI 3.14159265358979323846 // 定义圆周率

void clarke_transform(float* current, float* alpha_beta)
{
    alpha_beta[0] = current[0];
    alpha_beta[1] = (current[1] - current[0] * 0.5 - current[2] * 0.5) * sqrtf(3.0);
}

void park_transform(float* alpha_beta, float theta, float* dq)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    dq[0] = alpha_beta[0] * cos_theta + alpha_beta[1] * sin_theta;
    dq[1] = -alpha_beta[0] * sin_theta + alpha_beta[1] * cos_theta;
}

void inverse_park_transform(float* dq, float theta, float* alpha_beta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    alpha_beta[0] = dq[0] * cos_theta - dq[1] * sin_theta;
    alpha_beta[1] = dq[0] * sin_theta + dq[1] * cos_theta;
}

float get_theta()
{
    // 假设使用GPIO连接三组霍尔传感器，A相连接PA0、PB1，B相连接PA1、PB2，C相连接PA2、PB10
    uint8_t hall_state = ((GPIO_ReadInputData(GPIOA) & 0x7) << 1) | ((GPIO_ReadInputData(GPIOB) & 0xC) >> 2);
    float theta = atan2f(sin60[hall_state], cos60[hall_state]); // 计算瞬时角度，sin60和cos60存储了不同状态下的正弦和余弦值

    return theta;
}
float* current_control_PI(float* dq, float* voltage_dq)
{
    static float error_integral[2] = {0.0, 0.0};
    float error[2];
    float Kp = 1.0;
    float Ki = 0.01;

    error[0] = dq[0] - voltage_dq[0];
    error[1] = dq[1] - voltage_dq[1];

    error_integral[0] += error[0];
    error_integral[1] += error[1];

    voltage_dq[0] += Kp * error[0] + Ki * error_integral[0];
    voltage_dq[1] += Kp * error[1] + Ki * error_integral[1];

    return voltage_dq;
}
int main()
{
    float current[3] = {1.0, 2.0, 3.0}; // 假设当前的三相电流分别为1A、2A和3A
    float alpha_beta[2];
    float theta = get_theta(); // 获取瞬时角度
    float dq[2];

    clarke_transform(current, alpha_beta); // 进行Clarke变换
    park_transform(alpha_beta, theta, dq); // 进行PARK变换

    // 进行电流控制
    current_control_PI(dq, dq); // 在dq上进行修改
   
    inverse_park_transform(dq, theta, alpha_beta); // 进行反PARK变换

    printf("Ia = %f\n", current[0]);
    printf("Ib = %f\n", current[1]);
    printf("Ic = %f\n", current[2]);
    printf("Id = %f\n", dq[0]);
    printf("Iq = %f\n", dq[1]);
    printf("Va = %f\n", alpha_beta[0]);
    printf("Vb = %f\n", alpha_beta[1] * cosf(PI / 3) - alpha_beta[0] * sinf(PI / 3));
    printf("Vc = %f\n", alpha_beta[1] * cosf(PI / 3) + alpha_beta[0] * sinf(PI / 3));

    return 0;
}
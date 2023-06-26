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
float* velocity_control_PI(float speed_ref, float speed_fb, float* voltage_dq)
{
    static float error_integral = 0.0;
    float error;
    float Kp_v = 1.0;
    float Ki_v = 0.01;

    error = speed_ref - speed_fb;

    error_integral += error;

    voltage_dq[0] += Kp_v * error + Ki_v * error_integral;
    voltage_dq[1] += Kp_v * error + Ki_v * error_integral;

    return voltage_dq;
}
float* position_control_PID(float angle_ref, float angle_fb, float* speed_ref, float* voltage_dq)
{
    static float error_integral = 0.0;
    static float prev_error = 0.0;
    float error;
    float Kp_p = 1.0;
    float Ki_p = 0.01;
    float Kd_p = 0.001;

    error = angle_ref - angle_fb;

    error_integral += error;

    *speed_ref = Kp_p * error + Ki_p * error_integral + Kd_p * (error - prev_error);
    prev_error = error;

    velocity_control_PI(*speed_ref, get_speed(), voltage_dq);

    return voltage_dq;
}
float get_angle()
{
    // 假设使用GPIO连接三组霍尔传感器，A相连接PA0、PB1，B相连接PA1、PB2，C相连接PA2、PB10
    uint8_t hall_state = ((GPIO_ReadInputData(GPIOA) & 0x7) << 1) | ((GPIO_ReadInputData(GPIOB) & 0xC) >> 2);

    static const float angle_table[8] = {0.0, 60.0, 120.0, 180.0, -120.0, -60.0}; // 存储不同状态下的角度值

    return angle_table[hall_state];
}
void svpwm(float* alpha_beta, float* duty_cycle)
{
    float Udc = 1.0; // 假设直流电压为1V
    float T = 1.0 / (2 * PI * 50); // 假设频率为50Hz，周期T
    float m = sqrtf(3); // 调制比
    float Vd = alpha_beta[0] * m / 2;
    float Vq = alpha_beta[1] * m / 2;
    float t0, t1, t2, dt;

    float sector = floorf((atan2f(Vq, Vd) + PI) / (PI / 3)) + 1;
    float magnitude = sqrtf(Vd * Vd + Vq * Vq);
    float angle = atan2f(Vq, Vd) - (sector - 1) * (PI / 3);

    if (magnitude > Udc / 2)
        magnitude = Udc / 2;

    t1 = magnitude * sinf(PI / 3 - angle) / Udc;
    t2 = magnitude * sinf(angle) / Udc;
    t0 = (T - 3 * (t1 + t2)) / 6;

    duty_cycle[0] = t1 + t0;
    duty_cycle[1] = t2 + t0;
    duty_cycle[2] = t0;
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

     // 进行速度控制
    float speed_ref = 100.0; // 设定速度参考值为100rpm
    float speed_fb = get_speed(); // 获取实际速度反馈值，单位为rpm
    velocity_control_PI(speed_ref, speed_fb, dq); // 在dq上进行修改

      // 进行位置控制
    float angle_ref = 10.0; // 设定位置参考值为10度
    float angle_fb = get_angle(); // 获取实际位置反馈值，单位为度 可以是get_theta精确
    position_control_PID(angle_ref, angle_fb, &speed_ref, dq); // 在dq上进行修改

   
    inverse_park_transform(dq, theta, alpha_beta); // 进行反PARK变换

    printf("Ia = %f\n", current[0]);
    printf("Ib = %f\n", current[1]);
    printf("Ic = %f\n", current[2]);
    printf("Id = %f\n", dq[0]);
    printf("Iq = %f\n", dq[1]);
    printf("Va = %f\n", alpha_beta[0]);
    printf("Vb = %f\n", alpha_beta[1] * cosf(PI / 3) - alpha_beta[0] * sinf(PI / 3));
    printf("Vc = %f\n", alpha_beta[1] * cosf(PI / 3) + alpha_beta[0] * sinf(PI / 3));

    float duty_cycle[3];
    svpwm(alpha_beta, duty_cycle);

    printf("SVPWM Duty Cycle A: %f\n", duty_cycle[0]);
    printf("SVPWM Duty Cycle B: %f\n", duty_cycle[1]);
    printf("SVPWM Duty Cycle C: %f\n", duty_cycle[2]);

    return 0;
}

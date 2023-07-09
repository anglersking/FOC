///**
//  ******************************************************************************
//  * �ļ�����: main.c
//  * ��    ��: wanshan.pang
//  * ��    ��: V1.0
//  * ��д����: 2020-07-21
//  * ��    ��: hall�������ӿڻ�����������
//  ******************************************************************************
//
//  ******************************************************************************
//  */
//
///* ����ͷ�ļ� ----------------------------------------------------------------*/
//#include "bsp_hall.h"
//#include "bsp_bldc.h"
//#include "retarget.h"
///* ˽�����Ͷ��� --------------------------------------------------------------*/
//
///* ˽�к궨�� ----------------------------------------------------------------*/
//
///* ˽�б��� ------------------------------------------------------------------*/
//TIM_HandleTypeDef htimx_hall;
//
//__IO uint32_t RT_hallPhase = 0; // �����ź���λ RT-> Real Time
////uint32_t LS_hallPhase = 0; // ��һ�εĻ����ź���λ
//
////__IO uint32_t RT_hallcomp = 0;  // ��������ֵ
////__IO uint32_t RT_hallcnt = 0;   // ��������ֵ
//
//MotorDir_Typedef RT_hallDir = MOTOR_DIR_CW; // ����˳��õ��ĵ��ת������
//
///* �����ź�˳��,����ȷ�����ת������
// * �±��ǵ�ǰ�Ļ�������,���±��Ӧ��Ԫ��ֵ����һ�εĻ�������
// * ֻҪ��¼��һ�εĻ���ֵ,Ȼ��Ա����ݼ���֪����ǰ���ʵ��ת������
// */
////#ifndef USE_PMSMMOTOR
////const uint8_t HallDirCcw [7] = {0, 3, 6, 2, 5, 1, 4};   // 0 ������
////#else
////const uint8_t HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM ����ʱ����ת����
////#endif
//
///* ��չ���� ------------------------------------------------------------------*/
//
///* ˽�к���ԭ�� --------------------------------------------------------------*/
//
///* ������ --------------------------------------------------------------------*/
//
//
///**
//  * ��������: ��ʱ��HALL�ӿ����ų�ʼ��
//  * �������: ��
//  * �� �� ֵ: ��
//  * ˵    ��: ��
//  */
//void YS_TIM_HALL_Base_MspInit(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    HALL_TIM_GPIO_RCC_CLK_ENABLE();
//    GPIO_InitStruct.Pin       = HALL_TIM_CH1_PIN;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = HALL_TIM_GPIO_AF;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    HAL_GPIO_Init(HALL_TIM_CH1_PORT, &GPIO_InitStruct);
//    GPIO_InitStruct.Pin       = HALL_TIM_CH2_PIN;
//    HAL_GPIO_Init(HALL_TIM_CH2_PORT, &GPIO_InitStruct);
//    GPIO_InitStruct.Pin       = HALL_TIM_CH3_PIN;
//    HAL_GPIO_Init(HALL_TIM_CH3_PORT, &GPIO_InitStruct);
//    HAL_NVIC_SetPriority(HALL_TIM_IRQn, 1, 1);
//    HAL_NVIC_EnableIRQ(HALL_TIM_IRQn);
//}
//
///**
//  * ��������: ��ʱ�������������ӿڳ�ʼ��
//  * �������: ��
//  * �� �� ֵ: ��
//  * ˵    ��: ��
//  */
//void HALLSensor_TIMx_Init(void)
//{
//    TIM_HallSensor_InitTypeDef sHallConfig = {0};
//    /* ͨ�ö�ʱ������ʱ��ʹ�� */
//    HALL_TIM_RCC_CLK_ENABLE();
//    /* ��ʼ�������豸����ʱ��ͨ�����ţ�ʹ��ʱ�� */
////    YS_TIM_HALL_Base_MspInit();
//    //
//    /* ���ö�ʱ�������������� */
//    htimx_hall.Instance           = HALL_TIMx;          // ��ʱ��TIM
//    htimx_hall.Init.Prescaler     = HALL_TIM_PRESCALER; // PSC����Ԥ��Ƶֵ
//    htimx_hall.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED1;// ���Ķ���ģʽ
//    htimx_hall.Init.Period        = HALL_TIM_PERIOD;    // ��������
//    htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    HAL_TIM_Base_Init(&htimx_hall);
//
//
//    sHallConfig.Commutation_Delay = 0x00; // �ӳٴ���,7us,ʵ�ʲ�����Ҫ7us
//    sHallConfig.IC1Filter         = 0x0F; // �����˲�;
////    sHallConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;// ���벶����//��������
//    sHallConfig.IC1Prescaler      = TIM_ICPSC_DIV1;       // ���벶��Ԥ��Ƶ
//    HAL_TIMEx_HallSensor_Init(&htimx_hall, &sHallConfig);
//    HAL_TIMEx_HallSensor_Start(&htimx_hall);
//}
///**
//  * ��������: ��ȡ��������״̬
//  * �������: ��
//  * �� �� ֵ: ��������״̬
//  * ˵    ��: ֱ�Ӷ�ȡ���ŵ�״̬,�����ֽڵĵ���λ�ֱ��ӦUVW(HALL)�ĵ�ƽ״̬
//  */
////#define FILTER_ALPHA 0.1f // �˲���ϵ��
////int32_t HALL_GetPhase() {
////    static float u, v, w;
////    int32_t tmp = 0;
////    u = u * (1 - FILTER_ALPHA) + HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN) * FILTER_ALPHA;
////    v = v * (1 - FILTER_ALPHA) + HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN) * FILTER_ALPHA;
////    w = w * (1 - FILTER_ALPHA) + HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN) * FILTER_ALPHA;
////
////    tmp |= (u > 0.5f ? 1 : 0); // U(A)
////    tmp <<= 1;
////    tmp |= (v > 0.5f ? 1 : 0); // V(B)
////    tmp <<= 1;
////    tmp |= (w > 0.5f ? 1 : 0); // W(C)
////
////    return (tmp & 0x0007); // ȡ����λ
////}
//
//int32_t HALL_GetPhase()
//{
//    int32_t tmp = 0;
//    tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN);//U(A)
//    tmp <<= 1;
//    tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN);//V(B)
//    tmp <<= 1;
//    tmp |= HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN);//W(C)
////    printf("u,v,w,%d,%d,%d\n",HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN),HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN)
////    , HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN));
//    return (tmp & 0x0007); // ȡ����λ
//}
////int32_t HALL_GetPhase() {
////    int32_t tmp = 0;
////    uint8_t u, v, w;
////    u = HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN);
////    v = HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN);
////    w = HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN);
////    HAL_Delay(10); // ��ʱ 10ms
////
////    if (u != HAL_GPIO_ReadPin(HALL_TIM_CH1_PORT, HALL_TIM_CH1_PIN)) {
////        return HALL_GetPhase(); // �������������¶�ȡ
////    }
////    if (v != HAL_GPIO_ReadPin(HALL_TIM_CH2_PORT, HALL_TIM_CH2_PIN)) {
////        return HALL_GetPhase();
////    }
////    if (w != HAL_GPIO_ReadPin(HALL_TIM_CH3_PORT, HALL_TIM_CH3_PIN)) {
////        return HALL_GetPhase();
////    }
////
////    tmp |= u; // U(A)
////    tmp <<= 1;
////    tmp |= v; // V(B)
////    tmp <<= 1;
////    tmp |= w; // W(C)
////    printf("u,v,w,%d,%d,%d\n",u,v,w);
////
////    return (tmp & 0x0007); // ȡ����λ
////}
///**
//  * ��������: �����������ص�����
//  * �������: @htim,�����������ӿڶ�ʱ��
//  * �� �� ֵ: ��
//  * ˵    ��:
//  */
////void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
////{
////
//////    printf("inin---------\n");
////    int32_t RT_hallPhase = 0; // �����ź�
////    RT_hallPhase = HALL_GetPhase();     // ��ȡ�������ŵ���λ
////    /* ������� */
//////    BLDCMotor_PhaseCtrl(RT_hallPhase);
////
////    /* ��ȡ�����źŵļ������ *//* �ٶ�������� */
////    RT_hallcomp += __HAL_TIM_GET_COMPARE(htim,HALL_TIM_CH1);
////    RT_hallcnt++;
////    /* �жϷ��� */
////    if(HallDirCcw[RT_hallPhase] == LS_hallPhase) // ��������е�һ��
////    {
////        RT_hallDir = MOTOR_DIR_CCW;
////        printf("��ת%d\n",RT_hallPhase);
////    }
////    else {
////        RT_hallDir = MOTOR_DIR_CW;
////        printf("��ת%d\n",RT_hallPhase);
////    }
////    LS_hallPhase = RT_hallPhase; // ��¼��һ���Ļ���ֵ
////}
//
//

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include "math.h"
#include <string.h>


#define PI 3.14159265358979323846 // ����Բ����
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALL_U_Pin GPIO_PIN_0
#define HALL_U_GPIO_Port GPIOA
#define HALL_V_Pin GPIO_PIN_1
#define HALL_V_GPIO_Port GPIOA
#define HALL_W_Pin GPIO_PIN_2
#define HALL_W_GPIO_Port GPIOA
#define MOTOR_ENCODER_PPR 24 //���������������
#define POLE_PAIRES                      4 // ��ˢ���4�Լ�
#define PPR                             (POLE_PAIRES*2*3) // (4*2*3) ����ÿת,
__IO uint32_t RT_hallcomp = 0;  // ��������ֵ
__IO uint32_t RT_hallcnt = 0;   // ��������ֵ
void convert();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//TIM_HandleTypeDef htim2;
int LS_hallPhase=0;
uint16_t hallState = 0, currentCount = 0, prevCount = 0, rpm = 0;
////const int HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM ��???ʱ����ת��??
//
const int HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM ��???ʱ����ת��??
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    char buffer[8];
    char buffer_sta[32];

    if (htim == &htim2)
    {
//       printf("time2\n");
//        hallState = ((HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)) << 2) |
//                    ((HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)) << 1) |
//                    HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
//        uint8_t hall_u_value = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
//        uint8_t hall_v_value = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
//        uint8_t hall_w_value = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
//        HAL_UART_Transmit(&huart1,(uint8_t *)hallState, sizeof((uint8_t *)hallState),10);

//        sprintf(buffer, "%u\r\n", hallState);
//
//        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 100);
//        sprintf(buffer_sta, "Hall U: %u, Hall V: %u, Hall W: %u\r\n", hall_u_value, hall_v_value, hall_w_value);
//        HAL_UART_Transmit(&huart1, (uint8_t *)buffer_sta, strlen(buffer_sta), 100);
//
//        HAL_UART_Transmit(&huart1,(uint8_t *) "Time2 in\n", 9, 100);
        convert();

//
//        if (hallState != 0 && hallState != 7)
//        {
//            // ������תһȦ������????
//            currentCount = __HAL_TIM_GET_COUNTER(&htim2);
//            int16_t countDiff = currentCount - prevCount;
//            if (countDiff < 0)
//            {
//                countDiff += htim2.Init.Period;
//            }
//            prevCount = currentCount;
//
//            //����ת???(rpm)
//            rpm = ((countDiff * 60 * HAL_RCC_GetPCLK2Freq()) / (htim2.Init.Prescaler * MOTOR_ENCODER_PPR * htim2.Instance->ARR));
//        }
    }
}
//static float speed = 0.0f;
//static uint32_t last_capture_time = 0;
//#define TICK_FREQ 84000000
//#define SENSOR_PPR 6
//#define PRINT_PERIOD 500 // ��ӡ���ڣ���λms
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//        uint32_t current_time = HAL_GetTick();
//        uint32_t interval = current_time - last_capture_time;
//        last_capture_time = current_time;
//        if (interval > 0 && interval < PRINT_PERIOD) {
//            // ���㵱ǰ�ٶ�
//            float freq = (float)TICK_FREQ / (float)interval;
//            speed = freq * 60.0f / SENSOR_PPR;
//            printf("%f\n",speed);
//        }
//    }
//}
// void HALLSensor_Init()
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//
//    GPIO_InitStruct.Pin = HALL_U_Pin | HALL_V_Pin | HALL_W_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    __HAL_RCC_TIM1_CLK_ENABLE();
//
//    htim2.Instance = TIM2;
//    htim2.Init.Prescaler = 0;
//    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim2.Init.Period = 65535;
//    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    htim2.Init.RepetitionCounter = 0;
//    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//    HAL_TIM_Base_Init(&htim2);
//
//
//
//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(TIM2_IRQn);
//
//
//
//
//}
void HALLSensor_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE; // ���ݴ������źż����������벶����
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
}

int32_t HALL_GetPhase1()
{
    int32_t tmp = 0;
    tmp |= HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);//W(C)
    return (tmp & 0x0007); // ȡ����λ
}
//float get_angle()
//{
//    // ����ʹ��GPIO�������������������A������PA0��PB1��B������PA1��PB2��C������PA2��PB10
//    uint8_t hall_state = HALL_GetPhase();
//
//    static const float angle_table[8] = {0.0, 60.0, 120.0, 180.0, -120.0, -60.0}; // �洢��ͬ״̬�µĽǶ�ֵ
//
//    return angle_table[hall_state];
//}
//float get_theta()
//{
//    // ����ʹ��GPIO�������������������A������PA0��PB1��B������PA1��PB2��C������PA2��PB10
//    uint8_t hall_state = HALL_GetPhase();
//    float theta = atan2f(sin60[hall_state], cos60[hall_state]); // ����˲ʱ�Ƕȣ�sin60��cos60�洢�˲�ͬ״̬�µ����Һ�����ֵ
//
//    return theta;
//}
void convert()
{
    // printf("a:%d,b:%d,c:%d\n",u,v,w);
    int RT_hallPhase = 0; // �����ź�
    RT_hallPhase = HALL_GetPhase1();     // ��ȡ�������ŵ���??
    printf("RT_hallPhase:%d\n",RT_hallPhase);
    printf("LS_hallPhase:%d\n",LS_hallPhase);
    printf("��λ dic HallDirCcw [��λ:%d]:ֵ:%d,��һ����λ:LS_hallPhase:%d\n",RT_hallPhase,HallDirCcw[RT_hallPhase],LS_hallPhase);

            hallState = ((HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)) << 2) |
                    ((HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)) << 1) |
                    HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);




    /* �жϷ��� */
    if(HallDirCcw[RT_hallPhase] == LS_hallPhase) // ��������е�????
    {

        printf("��ת\n");
    }
    else{
       printf("��ת\n");
    }

//

    if (hallState != 0 && hallState != 7)
    {
        // ������תһȦ������????
        currentCount = __HAL_TIM_GET_COUNTER(&htim2);

        int16_t countDiff = currentCount - prevCount;
        if (countDiff < 0)
        {
            countDiff += htim2.Init.Period;
        }
        prevCount = currentCount;

        //����ת???(rpm)
        rpm = ((countDiff * 60 * HAL_RCC_GetPCLK2Freq()) / (htim2.Init.Prescaler * MOTOR_ENCODER_PPR * htim2.Instance->ARR));
        printf("currentCount:%d,prevCount:%d, htim2.Init.Period:%d,����:%d,��ĸ:%d,ת��:%d\n",currentCount,prevCount, htim2.Init.Period,
               (countDiff * 60 * HAL_RCC_GetPCLK2Freq()),htim2.Init.Prescaler ,rpm);
    }
    LS_hallPhase = RT_hallPhase; // ��¼��һ���Ļ���??
    printf("now LS_hallPhase:%d,RT_hallPhase:%d\n",LS_hallPhase,RT_hallPhase);
    printf("------------------------------\n");
}
float MotorSpeed  = 0.0f ;// ???????,?????????,
//
//_Bool isTimeUp    = 0;     // ??????
//uint32_t timeTick = 0;   // ??????
//extern MotorSta_Typedef Motor_State; // ??????????
//extern MotorDir_Typedef Motor_Dir;  // ?????? ,?????
//extern float PWM_Duty;        // 25%?????
//
//extern MotorDir_Typedef RT_hallDir; // ????????????????????
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    float Speed_hz  = 0 ;
    int encoder_left, encoder_right;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

   HALLSensor_Init();

// HAL_TIMEx_HallSensor_MspInit(&htim2);

   RetargetInit(&huart1);
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


////    Motor_Dir = MOTOR_DIR_CW;
////    timeTick = TIMECNT;
//    HAL_Delay(1000);
   MotorSpeed = 0.0f;
//   BLDCMotor_SetSpeed(MotorSpeed);
//    BLDCMotor_Start();

    printf("ini2 over\n");
//   HALLSensor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      encoder_left = (short)TIM1->CNT ;


      TIM1->CNT = 0;
      printf("encoder_left %d\n",encoder_left);


      HAL_Delay(100);

//      printf("rpm%d\n",rpm);
//      rpm=0;

//      uint32_t tmpCC = 0;
//      if(RT_hallcnt == 0) // �������Ϊ0
//      {
//          Speed_hz = 0;
//      }
//      else
//      {
//          tmpCC = RT_hallcomp / RT_hallcnt; // tmpCC:���β���֮��Ĳ���ֵ,
//          Speed_hz = (float)HALL_TIM_FREQ/(float)(tmpCC);
//      }
//      RT_hallcomp = 0;
//      RT_hallcnt  = 0;
//      /* �����źŵ�Ƶ��,ת��rps,ת��rpm */
//      if(RT_hallDir == MOTOR_DIR_CW) {
//          printf("��ת");
//          Speed_hz = fabs(Speed_hz);
//      }
//      else {
//          printf("��ת");
//          Speed_hz = -fabs(Speed_hz);
//      }
//      /* δ���κ��˲����ٶ�ֵ */
//      printf("%.3f Hz, %.2f RPS, %.2fRPM\n", Speed_hz, Speed_hz/PPR, (Speed_hz/PPR)*60);
//
//      isTimeUp = 0;
//      timeTick = TIMECNT;
//      printf("running \n");
//      int phase = HALL_GetPhase();
//      printf("HALL phase: %d\n", phase);
//      printf("RPM: %d\n", rpm);
//      hallState = ((HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)) << 2) |
//                  ((HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)) << 1) |
//                  HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
//      printf("%d\n",hallState);
////      hallState=19;
//      printf("gpio uvw %d,%d,%d\n", HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin),
//             HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin),
//             HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin));
      HAL_Delay(100);
//     printf("%d\n",HALL_GetPhase());
//      convert();
//      printf("angle %f",get_angle());

      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//void HAL_SYSTICK_Callback()
//{
////    printf("hal_systick\n");
//    if(timeTick != 0)
//        timeTick--;
//    if(timeTick == 0)
//    {
//        isTimeUp = 1;
//    }
//
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

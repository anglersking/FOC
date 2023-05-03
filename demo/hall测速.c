#define HALL_U_Pin GPIO_PIN_0
#define HALL_U_GPIO_Port GPIOA
#define HALL_V_Pin GPIO_PIN_1
#define HALL_V_GPIO_Port GPIOA
#define HALL_W_Pin GPIO_PIN_2
#define HALL_W_GPIO_Port GPIOA
#define MOTOR_ENCODER_PPR 6 //电机编码器的线数

TIM_HandleTypeDef htim1;
uint16_t hallState = 0, currentCount = 0, prevCount = 0, rpm = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    hallState = ((HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin)) << 2) |
                ((HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin)) << 1) |
                HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);

    if (hallState != 0 && hallState != 7)
    {
      // 计算电机转一圈的脉冲数
      currentCount = __HAL_TIM_GET_COUNTER(&htim1);
      int16_t countDiff = currentCount - prevCount;
      if (countDiff < 0)
      {
        countDiff += htim1.Init.Period;
      }
      prevCount = currentCount;

      //计算转速(rpm)
      rpm = ((countDiff * 60 * HAL_RCC_GetPCLK2Freq()) / (htim1.Init.Prescaler * MOTOR_ENCODER_PPR * htim1.Instance->ARR));
    }
  }
}

void HALLSensor_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = HALL_U_Pin | HALL_V_Pin | HALL_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim1);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

int main(void)
{
  HAL_Init();
  HALLSensor_Init();

  while (1)
  {
    printf("RPM: %d\n", rpm);
    HAL_Delay(1000); //延迟一段时间再次读取
  }
}
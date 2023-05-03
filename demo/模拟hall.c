#include "stdio.h"
const int HallDirCcw [7] = {0, 5, 3, 1, 6, 4, 2};    // PMSM 的逆时针旋转序列
int LS_hallPhase=0;
int HALL_GetPhase(int a,int b,int c)
{
  int tmp = 0;
  tmp |= a;//U(A)
  tmp <<= 1;
  tmp |= b;//V(B)
  tmp <<= 1;
  tmp |= c;//W(C)
  return (tmp & 0x0007); // 取低三位
}
void HAL_TIM_IC_CaptureCallback(int u,int v,int w)
{
    // printf("a:%d,b:%d,c:%d\n",u,v,w);
    int RT_hallPhase = 0; // 霍尔信号
    RT_hallPhase = HALL_GetPhase(u,v,w);     // 获取霍尔引脚的相位
    printf("a:%d,b:%d,c:%d,RT_hallPhase:%d\n",u,v,w,RT_hallPhase);
    // printf("RT_hallPhase:%d\n",RT_hallPhase);
    printf("本次相位对应的值 dic HallDirCcw [相位：%d]:值：%d,上一次相位:LS_hallPhase :%d\n",RT_hallPhase,HallDirCcw[RT_hallPhase],LS_hallPhase);
  

    
    /* 判断方向 */
    if(HallDirCcw[RT_hallPhase] == LS_hallPhase) // 序列与表中的一致
    {
        printf("反转\n");
    }
    else{
         printf("正转\n");
    }
    LS_hallPhase = RT_hallPhase; // 记录这一个的霍尔值
    // printf("本次 相位 LS_hallPhase:%d,RT_hallPhase:%d\n",LS_hallPhase,RT_hallPhase);
    printf("------------------------------\n");
}
int main()
{
    printf("hello\n");
    
    HAL_TIM_IC_CaptureCallback(1,0,0);
    HAL_TIM_IC_CaptureCallback(1,0,1);
    HAL_TIM_IC_CaptureCallback(0,0,1);
    HAL_TIM_IC_CaptureCallback(0,1,1);
    HAL_TIM_IC_CaptureCallback(0,1,0);
    HAL_TIM_IC_CaptureCallback(1,1,0);
    HAL_TIM_IC_CaptureCallback(1,0,0);
   
    printf("正转==========================");
    HAL_TIM_IC_CaptureCallback(1,0,0);
    HAL_TIM_IC_CaptureCallback(1,1,0);
    HAL_TIM_IC_CaptureCallback(0,1,0);
    HAL_TIM_IC_CaptureCallback(0,1,1);
    HAL_TIM_IC_CaptureCallback(0,0,1);
    HAL_TIM_IC_CaptureCallback(1,0,1);
    HAL_TIM_IC_CaptureCallback(1,0,0);
    

}
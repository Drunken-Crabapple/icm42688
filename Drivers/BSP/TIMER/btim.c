#include "./BSP/TIMER/btim.h"


TIM_HandleTypeDef g_tim6_cnt_handle;
volatile float roll = 0;
volatile float yaw = 0;
volatile float pitch = 0;
static uint16_t times = 0;
volatile uint8_t tim_1ms_flag = 0;
volatile uint8_t tim_100ms_flag = 0;

void btim_tim6_cnt_init(void)
{
    g_tim6_cnt_handle.Instance = TIM6;
    g_tim6_cnt_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim6_cnt_handle.Init.Prescaler = 8400 - 1;
    g_tim6_cnt_handle.Init.Period = 10 - 1;
    
    HAL_TIM_Base_Init(&g_tim6_cnt_handle);
    HAL_TIM_Base_Start_IT(&g_tim6_cnt_handle);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    
    if(htim->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn,2,0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_tim6_cnt_handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        times++;
        tim_1ms_flag = 1;
        if(times % 100 == 0)
        {
            tim_100ms_flag = 1;
        }
    }
}

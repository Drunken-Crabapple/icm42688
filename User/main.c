#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/SPI/spi.h"
#include "./BSP/ICM/icm.h"
#include "./BSP/INS/ins_task.h"
#include "./BSP/TIMER/btim.h"

static void pd14_pd15_low_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    gpio_init_struct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}

int main(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz  n m p q*/
    delay_init(168);                    /* 延时初始化 */
    pd14_pd15_low_init();
    usart_init(115200);
    usart_send_string("0.000000\r\n");
    icm_init();
    icm_gyro_offset_calibrate(1000);
    INS_Init();
    btim_tim6_cnt_init();

    while(1)
    {
        if(tim_1ms_flag)
        {
            tim_1ms_flag = 0;
            INS_Task();
            roll = INS.Roll;
            pitch = INS.Pitch;
            yaw = INS.Yaw;
        }

        if(tim_100ms_flag)
        {
            tim_100ms_flag = 0;
            printf("%f\r\n", yaw);
        }
    }
}


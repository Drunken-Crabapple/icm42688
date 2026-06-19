#include "./BSP/SPI/spi.h"

SPI_HandleTypeDef g_spi2_handle;


void spi2_init(void)
{
    g_spi2_handle.Instance = SPI2;
    g_spi2_handle.Init.Mode = SPI_MODE_MASTER;
    g_spi2_handle.Init.Direction = SPI_DIRECTION_2LINES;
    g_spi2_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spi2_handle.Init.CLKPolarity = SPI_POLARITY_LOW;      //空闲是低电位，第一个边沿是上升沿
    g_spi2_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_spi2_handle.Init.NSS = SPI_NSS_SOFT;
    g_spi2_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    g_spi2_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi2_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi2_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&g_spi2_handle);

    __HAL_SPI_ENABLE(&g_spi2_handle);

}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    if(hspi->Instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio_init_struct.Pin = GPIO_PIN_2;      //MISO->PC2
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Alternate = GPIO_AF5_SPI2;
        gpio_init_struct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOC,&gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_3;      //MOSI->PC3
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC,&gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_10;     //SCK->PB10
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB,&gpio_init_struct);
    }
}


uint8_t spi2_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&g_spi2_handle,&txdata,&rxdata,1,1000);
    return rxdata;
}

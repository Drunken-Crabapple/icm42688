#include "./BSP/ICM/icm.h"
#include "./BSP/SPI/spi.h"
#include "./BSP/ICM/icm_reg.h"
#include "./SYSTEM/delay/delay.h"

extern SPI_HandleTypeDef g_spi2_handle;
#define DEG_TO_RAD  0.01745329251f
#define ICM_STATIC_ACCEL_MIN_SQ     (0.95f * 0.95f)
#define ICM_STATIC_ACCEL_MAX_SQ     (1.05f * 1.05f)
#define ICM_STATIC_GYRO_MAX_SQ      (0.025f * 0.025f)
#define ICM_GYRO_BIAS_ADAPT_ALPHA   0.0005f
#define ICM_STATIC_CONFIRM_COUNT    50U
#define ICM_GYRO_LPF_ALPHA          0.35f

volatile float icm_gyro_bias_x = 0.0f;
volatile float icm_gyro_bias_y = 0.0f;
volatile float icm_gyro_bias_z = 0.0f;
volatile uint8_t icm_static_flag = 0;
volatile uint16_t icm_static_count = 0;

void icm_cs_init(void)
{
    spi2_init();

    __HAL_RCC_GPIOE_CLK_ENABLE();       //片选信号CS->PE7
    GPIO_InitTypeDef gpio_init_struct = {0};
    gpio_init_struct.Pin = GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE,&gpio_init_struct);

    ICM_CS(1);
}

/*
返回值为0：正常
返回值为1：错误
*/
uint8_t SPI2_ReadWriteBytes(uint8_t addr,uint8_t ptxdata,uint8_t *prxdata,uint16_t datasize)
{
    uint16_t totalsize = datasize + 1;
    uint8_t txbuffer[totalsize];
    uint8_t rxbuffer[totalsize];

    if(prxdata)     //如果prxdata不为NULL，就说明这次是读数据
    {
        txbuffer[0] = addr | 0x80;  //对于读数据，需要知道读哪个地址的数据，这一位是发送地址给从机 高位为1是读命令
        for(uint16_t i = 1;i <= datasize;i++)
        {
            txbuffer[i] = 0xFF;
        }
    }
    else            //如果prxdata为NULL，说明这次是写数据，高位为0，其余七位地址位即为写数据
    {
        txbuffer[0] = addr & 0x7F;  //最高位清零
        txbuffer[1] = ptxdata;      //发送写地址后即为数据
    }

    ICM_CS(0);      //拉低片选 选中从机
    if(HAL_SPI_TransmitReceive(&g_spi2_handle,txbuffer,rxbuffer,totalsize,1000) != HAL_OK)
    {
        return 1;
    }
    ICM_CS(1);
    if(prxdata)     //接收区不为空，把rxbuffer存储数据给prxdaata
    {
        for(uint16_t i = 0;i < datasize;i++)
        {
            prxdata[i] = rxbuffer[i + 1];       //第一位发送的是要读取数据的地址，所以传回来的rxbuffer第一个是废数据
        }
    }
    return 0;       
}

/*
返回一，通信失败
返回0x47,即71为正常
*/
uint8_t ICM_ReadID(void)
{
    uint8_t temp;
    if(SPI2_ReadWriteBytes(WHO_AM_I,0,&temp,1))
    {
        return 1;
    }
    return temp;
}

void icm_init(void)
{
    icm_cs_init();
    SPI2_ReadWriteBytes(REG_BANK_SEL,0x00,0,1);         //确保是处于BANK0
    SPI2_ReadWriteBytes(DEVICE_CONFIG,0x01,0,1);        //SPI模式0、3  软件复位
    delay_ms(1);                                        //等待1ms使软件复位生效
    SPI2_ReadWriteBytes(DRIVE_CONFIG,0x05,0,1);         //SPI速率<2ns
    SPI2_ReadWriteBytes(FIFO_CONFIG,0x00,0,1);          //FIFO旁路模式
    SPI2_ReadWriteBytes(PWR_MGMT0,0x0F,0,1);            //加速度计和陀螺仪开启，同时设置为低噪声模式
    delay_us(200);                                      //当从OFF模式切换至其他模式时，200us内不得对任何寄存器进行写入操作
    SPI2_ReadWriteBytes(GYRO_CONFIG0,0x46,0,1);         //设置陀螺仪量程为+-500dps  ODR为1kHz
    SPI2_ReadWriteBytes(ACCEL_CONFIG0,0x66,0,1);        //设置加速度计量程为+-2g    ODR为1kHz
}

void icm_gyro_offset_calibrate(uint16_t samples)
{
    uint8_t data[12] = {0};
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    if(samples == 0)
    {
        samples = 1;
    }

    delay_ms(100);

    for(uint16_t i = 0; i < samples; i++)
    {
        SPI2_ReadWriteBytes(ACCEL_DATA_X_H,0,data,12);
        gx_sum += ( (int16_t) ((data[6] << 8) | data[7]) ) / 65.536f * DEG_TO_RAD;
        gy_sum += ( (int16_t) ((data[8] << 8) | data[9]) ) / 65.536f * DEG_TO_RAD;
        gz_sum += ( (int16_t) ((data[10]<< 8) | data[11])) / 65.536f * DEG_TO_RAD;
        delay_ms(1);
    }

    icm_gyro_bias_x = gx_sum / samples;
    icm_gyro_bias_y = gy_sum / samples;
    icm_gyro_bias_z = gz_sum / samples;
}

void icm_getdata(float *ax,float *ay,float *az,
                 float *gx,float *gy,float *gz)
{
    static uint8_t gyro_lpf_init = 0;
    static float gx_lpf = 0.0f;
    static float gy_lpf = 0.0f;
    static float gz_lpf = 0.0f;
    uint8_t data[12] = {0};
    float ax_g;
    float ay_g;
    float az_g;
    float gx_raw;
    float gy_raw;
    float gz_raw;
    float gx_corr;
    float gy_corr;
    float gz_corr;
    float accel_norm_sq;
    float gyro_norm_sq;

    SPI2_ReadWriteBytes(ACCEL_DATA_X_H,0,data,12);
//    *temp = ( ( (int16_t)((data[0] << 8) | data[1]) ) / 132.48 ) + 25;
    ax_g = ( (int16_t) ((data[0] << 8) | data[1]) ) /(16384.0f);
    ay_g = ( (int16_t) ((data[2] << 8) | data[3]) ) / (16384.0f);
    az_g = ( (int16_t) ((data[4] << 8) | data[5]) ) / (16384.0f);

    gx_raw = ( (int16_t) ((data[6] << 8) | data[7]) ) / 65.536f * DEG_TO_RAD;  //从dps转为rad/s
    gy_raw = ( (int16_t) ((data[8] << 8) | data[9]) ) / 65.536f * DEG_TO_RAD;
    gz_raw = ( (int16_t) ((data[10]<< 8) | data[11])) / 65.536f * DEG_TO_RAD;

    gx_corr = gx_raw - icm_gyro_bias_x;
    gy_corr = gy_raw - icm_gyro_bias_y;
    gz_corr = gz_raw - icm_gyro_bias_z;

    accel_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    gyro_norm_sq = gx_corr * gx_corr + gy_corr * gy_corr + gz_corr * gz_corr;
    if(accel_norm_sq > ICM_STATIC_ACCEL_MIN_SQ &&
       accel_norm_sq < ICM_STATIC_ACCEL_MAX_SQ &&
       gyro_norm_sq < ICM_STATIC_GYRO_MAX_SQ)
    {
        if(icm_static_count < 1000U)
        {
            icm_static_count++;
        }

        if(icm_static_count >= ICM_STATIC_CONFIRM_COUNT)
        {
            icm_static_flag = 1;
            icm_gyro_bias_x += ICM_GYRO_BIAS_ADAPT_ALPHA * gx_corr;
            icm_gyro_bias_y += ICM_GYRO_BIAS_ADAPT_ALPHA * gy_corr;
            icm_gyro_bias_z += ICM_GYRO_BIAS_ADAPT_ALPHA * gz_corr;
        }
    }
    else
    {
        icm_static_flag = 0;
        icm_static_count = 0;

        if(!gyro_lpf_init)
        {
            gx_lpf = gx_corr;
            gy_lpf = gy_corr;
            gz_lpf = gz_corr;
            gyro_lpf_init = 1;
        }
        else
        {
            gx_lpf += ICM_GYRO_LPF_ALPHA * (gx_corr - gx_lpf);
            gy_lpf += ICM_GYRO_LPF_ALPHA * (gy_corr - gy_lpf);
            gz_lpf += ICM_GYRO_LPF_ALPHA * (gz_corr - gz_lpf);
        }

        gx_corr = gx_lpf;
        gy_corr = gy_lpf;
        gz_corr = gz_lpf;
    }

    *ax = ax_g;
    *ay = ay_g;
    *az = az_g;
    *gx = gx_corr;
    *gy = gy_corr;
    *gz = gz_corr;
}


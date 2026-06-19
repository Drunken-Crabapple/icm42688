#ifndef __ICM_H
#define __ICM_H

#include "./SYSTEM/sys/sys.h"
#define ICM_CS(x)       do{ x ? \
                            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET); \
                        }while (0)
uint8_t SPI2_ReadWriteBytes(uint8_t addr,uint8_t ptxdata,uint8_t *prxdata,uint16_t datasize);
void icm_cs_init(void);
uint8_t ICM_ReadID(void);
void icm_init(void);
void icm_gyro_offset_calibrate(uint16_t samples);
void icm_getdata(float *ax,float *ay,float *az,
                 float *gx,float *gy,float *gz);

extern volatile float icm_gyro_bias_x;
extern volatile float icm_gyro_bias_y;
extern volatile float icm_gyro_bias_z;
extern volatile uint8_t icm_static_flag;
extern volatile uint16_t icm_static_count;


#endif

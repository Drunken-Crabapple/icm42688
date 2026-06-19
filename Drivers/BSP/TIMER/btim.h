#ifndef __BTIM_H
#define __BTIM_H

#include "./SYSTEM/sys/sys.h"
void btim_tim6_cnt_init(void);
extern volatile float roll;
extern volatile float yaw;
extern volatile float pitch;
extern volatile uint8_t tim_1ms_flag;
extern volatile uint8_t tim_100ms_flag;



#endif

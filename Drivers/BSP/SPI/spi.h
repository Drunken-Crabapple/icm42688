#ifndef __SPI_H
#define __SPI_H

#include "./SYSTEM/sys/sys.h"
void spi2_init(void);
extern SPI_HandleTypeDef g_spi2_handle;
uint8_t spi2_read_write_byte(uint8_t txdata);


#endif

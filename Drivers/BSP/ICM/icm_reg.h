#ifndef __ICM_REG_H
#define __ICM_REG_H

#define DEVICE_CONFIG       0x11
#define DRIVE_CONFIG        0x13
#define FIFO_CONFIG         0x16

#define TEMP_DATA_H         0x1D
#define TEMP_DATA_L         0x1E
#define ACCEL_DATA_X_H      0x1F
#define ACCEL_DATA_X_L      0x20
#define ACCEL_DATA_Y_H      0x21
#define ACCEL_DATA_Y_L      0x22
#define ACCEL_DATA_Z_H      0x23
#define ACCEL_DATA_Z_L      0x24
#define GYRO_DATA_X_H       0x25
#define GYRO_DATA_X_L       0x26
#define GYRO_DATA_Y_H       0x27
#define GYRO_DATA_Y_L       0x28
#define GYRO_DATA_Z_H       0x29
#define GYRO_DATA_Z_L       0x2A

#define INT_STATUS          0x2D
#define PWR_MGMT0           0x4E
#define GYRO_CONFIG0        0x4F
#define ACCEL_CONFIG0       0x50
#define WHO_AM_I            0x75
#define REG_BANK_SEL        0x76


#endif

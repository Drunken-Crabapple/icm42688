#ifndef __ICM_INS_ADAPTER_H
#define __ICM_INS_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

#define ICM_INS_GRAVITY_MPS2 9.80665f

typedef struct
{
    float accel_mps2[3];
    float gyro_radps[3];
    float temperature_c;
} ICM_INS_Input_t;

void ICM_INS_BuildInput(float ax_g, float ay_g, float az_g,
                        float gx_radps, float gy_radps, float gz_radps,
                        float temperature_c,
                        ICM_INS_Input_t *input);

#ifdef __cplusplus
}
#endif

#endif

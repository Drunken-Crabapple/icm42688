#include "./BSP/ICM/icm_ins_adapter.h"

void ICM_INS_BuildInput(float ax_g, float ay_g, float az_g,
                        float gx_radps, float gy_radps, float gz_radps,
                        float temperature_c,
                        ICM_INS_Input_t *input)
{
    if (input == 0) {
        return;
    }

    input->accel_mps2[0] = ax_g * ICM_INS_GRAVITY_MPS2;
    input->accel_mps2[1] = ay_g * ICM_INS_GRAVITY_MPS2;
    input->accel_mps2[2] = az_g * ICM_INS_GRAVITY_MPS2;

    input->gyro_radps[0] = gx_radps;
    input->gyro_radps[1] = gy_radps;
    input->gyro_radps[2] = gz_radps;

    input->temperature_c = temperature_c;
}

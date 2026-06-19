#include <math.h>
#include <stdio.h>

#include "../Drivers/BSP/ICM/icm_ins_adapter.h"

static int close_enough(float actual, float expected)
{
    return fabsf(actual - expected) < 0.0001f;
}

static int expect_close(const char *name, float actual, float expected)
{
    if (!close_enough(actual, expected)) {
        printf("%s expected %.6f, got %.6f\n", name, expected, actual);
        return 1;
    }
    return 0;
}

int main(void)
{
    ICM_INS_Input_t sample;
    int failures = 0;

    ICM_INS_BuildInput(1.0f, -0.5f, 0.25f,
                       0.1f, -0.2f, 0.3f,
                       36.5f,
                       &sample);

    failures += expect_close("accel x", sample.accel_mps2[0], 9.80665f);
    failures += expect_close("accel y", sample.accel_mps2[1], -4.903325f);
    failures += expect_close("accel z", sample.accel_mps2[2], 2.4516625f);
    failures += expect_close("gyro x", sample.gyro_radps[0], 0.1f);
    failures += expect_close("gyro y", sample.gyro_radps[1], -0.2f);
    failures += expect_close("gyro z", sample.gyro_radps[2], 0.3f);
    failures += expect_close("temperature", sample.temperature_c, 36.5f);

    if (failures != 0) {
        return 1;
    }

    return 0;
}

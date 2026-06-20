#include "./BSP/INS/ins_task.h"

#include <math.h>
#include <string.h>

#include "./BSP/ICM/icm.h"
#include "./BSP/ICM/icm_ins_adapter.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

INS_t INS;
IMU_Param_t IMU_Param;

const float xb[3] = {1.0f, 0.0f, 0.0f};
const float yb[3] = {0.0f, 1.0f, 0.0f};
const float zb[3] = {0.0f, 0.0f, 1.0f};

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
    IMU_Param.scale[X] = 1.0f;
    IMU_Param.scale[Y] = 1.0f;
    IMU_Param.scale[Z] = 1.0f;
    IMU_Param.Yaw = 0.0f;
    IMU_Param.Pitch = 0.0f;
    IMU_Param.Roll = 0.0f;
    IMU_Param.flag = 1U;

    IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 0.9996f, 0.0085f);
    INS.AccelLPF = 0.0085f;
}

void INS_Task(void)
{
    float ax_g;
    float ay_g;
    float az_g;
    float gx_radps;
    float gy_radps;
    float gz_radps;
    ICM_INS_Input_t imu_sample;
    const float gravity[3] = {0.0f, 0.0f, ICM_INS_GRAVITY_MPS2};
    float gravity_b[3];

    icm_getdata(&ax_g, &ay_g, &az_g, &gx_radps, &gy_radps, &gz_radps);
    ICM_INS_BuildInput(ax_g, ay_g, az_g,
                       gx_radps, gy_radps, gz_radps,
                       0.0f,
                       &imu_sample);

    INS.Accel[X] = imu_sample.accel_mps2[X];
    INS.Accel[Y] = imu_sample.accel_mps2[Y];
    INS.Accel[Z] = imu_sample.accel_mps2[Z];
    INS.Gyro[X] = imu_sample.gyro_radps[X];
    INS.Gyro[Y] = imu_sample.gyro_radps[Y];
    INS.Gyro[Z] = imu_sample.gyro_radps[Z];

    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180.0f / PI;
    INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180.0f / PI;

    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z],
                             INS.Accel[X], INS.Accel[Y], INS.Accel[Z],
                             INS_TASK_DT);
    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3U; i++) {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * INS_TASK_DT / (INS.AccelLPF + INS_TASK_DT) +
                               INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + INS_TASK_DT);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.Yaw = QEKF_INS.Yaw;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);
    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);
    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);
    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);
    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset;
    static float lastPitchOffset;
    static float lastRollOffset;
    static float c_11;
    static float c_12;
    static float c_13;
    static float c_21;
    static float c_22;
    static float c_23;
    static float c_31;
    static float c_32;
    static float c_33;
    float gyro_temp[3];
    float accel_temp[3];

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f ||
        param->flag) {
        float cosYaw = cosf(param->Yaw / 57.295779513f);
        float cosPitch = cosf(param->Pitch / 57.295779513f);
        float cosRoll = cosf(param->Roll / 57.295779513f);
        float sinYaw = sinf(param->Yaw / 57.295779513f);
        float sinPitch = sinf(param->Pitch / 57.295779513f);
        float sinRoll = sinf(param->Roll / 57.295779513f);

        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0U;
    }

    for (uint8_t i = 0; i < 3U; i++) {
        gyro_temp[i] = gyro[i] * param->scale[i];
        accel_temp[i] = accel[i];
    }

    gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

    accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa;
    float qb;
    float qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch;
    float cosYaw;
    float cosRoll;
    float sinPitch;
    float sinYaw;
    float sinRoll;

    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = cosf(Pitch / 2.0f);
    cosYaw = cosf(Yaw / 2.0f);
    cosRoll = cosf(Roll / 2.0f);
    sinPitch = sinf(Pitch / 2.0f);
    sinYaw = sinf(Yaw / 2.0f);
    sinRoll = sinf(Roll / 2.0f);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

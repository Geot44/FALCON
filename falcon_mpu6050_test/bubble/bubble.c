#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"

/* MPU-6050 I2C configuration */
#define MPU6050_ADDR                    0x68U
#define MPU6050_REG_SMPLRT_DIV           0x19U
#define MPU6050_REG_CONFIG               0x1AU
#define MPU6050_REG_GYRO_CONFIG          0x1BU
#define MPU6050_REG_ACCEL_CONFIG         0x1CU
#define MPU6050_REG_ACCEL_XOUT_H         0x3BU
#define MPU6050_REG_PWR_MGMT_1           0x6BU
#define MPU6050_REG_WHO_AM_I             0x75U
#define MPU6050_WHO_AM_I_VALUE           0x68U

/* Configured ranges: accelerometer +/-8 g; gyroscope +/-500 degrees/s */
#define ACCEL_SENSITIVITY                4096L
#define GYRO_SENSITIVITY                 65L  /* approximate; actual +/-500 dps is 65.5 LSB/(deg/s) */

/* Timing: DLPF enabled, sample rate = 1 kHz / (1 + divider). */
#define CALIBRATION_SAMPLES              300U
#define CALIBRATION_DELAY_US             10000U
#define SAMPLE_DELAY_US                  10000U  /* 100 Hz target */
#define PRINT_EVERY_SAMPLES              10U     /* print at about 10 Hz */

/* Experimental fall-detection thresholds. Tune using collected data. */
#define FREEFALL_THRESHOLD_MG            550L
#define IMPACT_THRESHOLD_MG              2200L
#define ROTATION_THRESHOLD_CENTI_DPS     15000L  /* 150.00 degrees/s */
#define STILL_ACCEL_MIN_MG               700L
#define STILL_ACCEL_MAX_MG               1300L
#define STILL_GYRO_THRESHOLD_CENTI_DPS    3500L  /* 35.00 degrees/s */
#define FREEFALL_TIMEOUT_SAMPLES         100U    /* 1 second at 100 Hz */
#define STILL_CONFIRM_SAMPLES            150U    /* 1.5 seconds at 100 Hz */
#define FALL_SEQUENCE_TIMEOUT_SAMPLES    400U    /* 4 seconds at 100 Hz */

/* Existing movement indicator thresholds */
#define ACCEL_MOVEMENT_THRESHOLD_MG       250L
#define GYRO_MOVEMENT_THRESHOLD_CENTI     1500L
#define MOVEMENT_CONFIRM_SAMPLES          2U

typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t temperature;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} MPU6050_Data_t;

typedef struct
{
    int32_t gyroX;
    int32_t gyroY;
    int32_t gyroZ;
} MPU6050_Offsets_t;

typedef enum
{
    FALL_STATE_NORMAL = 0,
    FALL_STATE_WAIT_IMPACT,
    FALL_STATE_WAIT_STILLNESS,
    FALL_STATE_DETECTED
} FallState_t;

static status_t MPU6050_Read(uint8_t reg, uint8_t *data, uint8_t length)
{
    return BOARD_Accel_I2C_Receive(MPU6050_ADDR, reg, 1U, data, length);
}

static status_t MPU6050_Write(uint8_t reg, uint8_t value)
{
    return BOARD_Accel_I2C_Send(MPU6050_ADDR, reg, 1U, value);
}

static int16_t MPU6050_CombineBytes(uint8_t high, uint8_t low)
{
    uint16_t combined = ((uint16_t)high << 8U) | (uint16_t)low;
    return (int16_t)combined;
}

static status_t MPU6050_ReadData(MPU6050_Data_t *data)
{
    uint8_t raw[14];
    status_t status = MPU6050_Read(MPU6050_REG_ACCEL_XOUT_H,
                                   raw, (uint8_t)sizeof(raw));
    if (status != kStatus_Success)
    {
        return status;
    }

    data->accelX = MPU6050_CombineBytes(raw[0], raw[1]);
    data->accelY = MPU6050_CombineBytes(raw[2], raw[3]);
    data->accelZ = MPU6050_CombineBytes(raw[4], raw[5]);
    data->temperature = MPU6050_CombineBytes(raw[6], raw[7]);
    data->gyroX = MPU6050_CombineBytes(raw[8], raw[9]);
    data->gyroY = MPU6050_CombineBytes(raw[10], raw[11]);
    data->gyroZ = MPU6050_CombineBytes(raw[12], raw[13]);
    return kStatus_Success;
}

/* Print signed fixed-point values without requiring %f support. */
static void MPU6050_PrintFixed(int32_t value, uint32_t decimals)
{
    int32_t divisor = 1;
    int32_t magnitude;
    uint32_t i;

    for (i = 0U; i < decimals; i++)
    {
        divisor *= 10;
    }

    magnitude = (value < 0) ? -value : value;
    if (value < 0)
    {
        PRINTF("-");
    }
    PRINTF("%d", (int)(magnitude / divisor));
    PRINTF(".");

    {
        int32_t fraction = magnitude % divisor;
        int32_t place = divisor / 10;
        while (place > 0)
        {
            PRINTF("%d", (int)((fraction / place) % 10));
            place /= 10;
        }
    }
}

/* Print the readings associated with a fall-detection event. */
static void MPU6050_PrintFallReadings(uint32_t accelMagnitudeMg,
                                      int32_t gx_centi,
                                      int32_t gy_centi,
                                      int32_t gz_centi)
{
    PRINTF("MAG=");
    MPU6050_PrintFixed((int32_t)accelMagnitudeMg, 3U);
    PRINTF(" g | GYRO X=");
    MPU6050_PrintFixed(gx_centi, 2U);
    PRINTF(" Y=");
    MPU6050_PrintFixed(gy_centi, 2U);
    PRINTF(" Z=");
    MPU6050_PrintFixed(gz_centi, 2U);
    PRINTF(" deg/s\r\n");
}

/* Calibrate gyro bias only. Acceleration magnitude is calculated from raw
 * acceleration so that detection does not depend on sensor orientation. */
static status_t MPU6050_Calibrate(MPU6050_Offsets_t *offsets)
{
    MPU6050_Data_t sample;
    int64_t sumGyroX = 0;
    int64_t sumGyroY = 0;
    int64_t sumGyroZ = 0;
    uint32_t i;

    PRINTF("\r\n====================================\r\n");
    PRINTF("  MPU-6050 Gyroscope Calibration\r\n");
    PRINTF("====================================\r\n");
    PRINTF("Keep the sensor completely still.\r\n");
    PRINTF("Collecting %u samples...\r\n", (unsigned int)CALIBRATION_SAMPLES);
    SDK_DelayAtLeastUs(1000000U, SystemCoreClock);

    for (i = 0U; i < CALIBRATION_SAMPLES; i++)
    {
        if (MPU6050_ReadData(&sample) != kStatus_Success)
        {
            PRINTF("ERROR: Sensor read failed during calibration.\r\n");
            return kStatus_Fail;
        }
        sumGyroX += sample.gyroX;
        sumGyroY += sample.gyroY;
        sumGyroZ += sample.gyroZ;
        SDK_DelayAtLeastUs(CALIBRATION_DELAY_US, SystemCoreClock);
    }

    offsets->gyroX = (int32_t)(sumGyroX / CALIBRATION_SAMPLES);
    offsets->gyroY = (int32_t)(sumGyroY / CALIBRATION_SAMPLES);
    offsets->gyroZ = (int32_t)(sumGyroZ / CALIBRATION_SAMPLES);

    PRINTF("Calibration complete. Gyro offsets: X=%d Y=%d Z=%d\r\n",
           (int)offsets->gyroX, (int)offsets->gyroY, (int)offsets->gyroZ);
    return kStatus_Success;
}

static int32_t MPU6050_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Acceleration magnitude in mg. Integer square root avoids floating point. */
static uint32_t MPU6050_AccelerationMagnitudeMg(int32_t ax_mg,
                                                int32_t ay_mg,
                                                int32_t az_mg)
{
    uint64_t value = (uint64_t)((int64_t)ax_mg * ax_mg) +
                     (uint64_t)((int64_t)ay_mg * ay_mg) +
                     (uint64_t)((int64_t)az_mg * az_mg);
    uint64_t result = 0U;
    uint64_t bit = (uint64_t)1U << 62U;

    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return (uint32_t)result;
}

static bool MPU6050_IsMoving(uint32_t accelMagnitudeMg,
                             int32_t gx_centi,
                             int32_t gy_centi,
                             int32_t gz_centi)
{
    if ((accelMagnitudeMg < (1000U - ACCEL_MOVEMENT_THRESHOLD_MG)) ||
        (accelMagnitudeMg > (1000U + ACCEL_MOVEMENT_THRESHOLD_MG)))
    {
        return true;
    }

    return (MPU6050_Abs(gx_centi) > GYRO_MOVEMENT_THRESHOLD_CENTI) ||
           (MPU6050_Abs(gy_centi) > GYRO_MOVEMENT_THRESHOLD_CENTI) ||
           (MPU6050_Abs(gz_centi) > GYRO_MOVEMENT_THRESHOLD_CENTI);
}

/* Fall sequence: low-g event -> impact/rapid rotation -> post-event stillness. */
static bool MPU6050_UpdateFallDetection(FallState_t *state,
                                        uint32_t accelMagnitudeMg,
                                        int32_t gx_centi,
                                        int32_t gy_centi,
                                        int32_t gz_centi,
                                        uint32_t *stateSamples,
                                        uint32_t *stillSamples)
{
    bool rapidRotation =
        (MPU6050_Abs(gx_centi) > ROTATION_THRESHOLD_CENTI_DPS) ||
        (MPU6050_Abs(gy_centi) > ROTATION_THRESHOLD_CENTI_DPS) ||
        (MPU6050_Abs(gz_centi) > ROTATION_THRESHOLD_CENTI_DPS);
    bool impact = (accelMagnitudeMg >= IMPACT_THRESHOLD_MG);
    bool still = (accelMagnitudeMg >= STILL_ACCEL_MIN_MG) &&
                 (accelMagnitudeMg <= STILL_ACCEL_MAX_MG) &&
                 (MPU6050_Abs(gx_centi) < STILL_GYRO_THRESHOLD_CENTI_DPS) &&
                 (MPU6050_Abs(gy_centi) < STILL_GYRO_THRESHOLD_CENTI_DPS) &&
                 (MPU6050_Abs(gz_centi) < STILL_GYRO_THRESHOLD_CENTI_DPS);

    switch (*state)
    {
        case FALL_STATE_NORMAL:
            if (accelMagnitudeMg < FREEFALL_THRESHOLD_MG)
            {
                *state = FALL_STATE_WAIT_IMPACT;
                *stateSamples = 0U;
                *stillSamples = 0U;
                PRINTF("[FALL] Low-acceleration trigger: ");
                MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
                PRINTF("[FALL] Waiting for impact or rapid rotation.\r\n");
            }
            break;

        case FALL_STATE_WAIT_IMPACT:
            (*stateSamples)++;
            if (impact || rapidRotation)
            {
                *state = FALL_STATE_WAIT_STILLNESS;
                *stateSamples = 0U;
                *stillSamples = 0U;
                PRINTF("[FALL] Impact/rotation trigger (%s%s): ",
                       impact ? "impact" : "",
                       rapidRotation ? (impact ? " + rapid rotation" : "rapid rotation") : "");
                MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
                PRINTF("[FALL] Waiting for stillness: need %u consecutive samples.\r\n",
                       (unsigned int)STILL_CONFIRM_SAMPLES);
            }
            else if (*stateSamples >= FREEFALL_TIMEOUT_SAMPLES)
            {
                *state = FALL_STATE_NORMAL;
                *stateSamples = 0U;
                PRINTF("[FALL] Candidate cancelled: no impact/rotation within timeout. Last reading: ");
                MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
            }
            break;

        case FALL_STATE_WAIT_STILLNESS:
            (*stateSamples)++;
            if (still)
            {
                (*stillSamples)++;
                if (*stillSamples >= STILL_CONFIRM_SAMPLES)
                {
                    *state = FALL_STATE_DETECTED;
                    PRINTF("\r\n!!! POSSIBLE FALL DETECTED !!!\r\n");
                    PRINTF("Stillness confirmed after %u consecutive samples.\r\n",
                           (unsigned int)*stillSamples);
                    PRINTF("Final readings: ");
                    MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
                    PRINTF("This is an experimental sensor alert, not a confirmed fall.\r\n\r\n");
                    return true;
                }
            }
            else
            {
                if (*stillSamples > 0U)
                {
                    PRINTF("[FALL] Stillness counter reset at %u/%u samples. Reading: ",
                           (unsigned int)*stillSamples,
                           (unsigned int)STILL_CONFIRM_SAMPLES);
                    MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
                }
                *stillSamples = 0U;
            }

            /* Progress report every 25 samples while waiting for stillness. */
            if ((*stateSamples % 25U) == 0U)
            {
                PRINTF("[FALL] Stillness progress: %u/%u consecutive samples; sequence %u/%u. ",
                       (unsigned int)*stillSamples,
                       (unsigned int)STILL_CONFIRM_SAMPLES,
                       (unsigned int)*stateSamples,
                       (unsigned int)FALL_SEQUENCE_TIMEOUT_SAMPLES);
                MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
            }

            if (*stateSamples >= FALL_SEQUENCE_TIMEOUT_SAMPLES)
            {
                PRINTF("[FALL] Candidate timed out: reached %u sequence samples with stillness at %u/%u.\r\n",
                       (unsigned int)*stateSamples,
                       (unsigned int)*stillSamples,
                       (unsigned int)STILL_CONFIRM_SAMPLES);
                PRINTF("[FALL] Last reading: ");
                MPU6050_PrintFallReadings(accelMagnitudeMg, gx_centi, gy_centi, gz_centi);
                *state = FALL_STATE_NORMAL;
                *stateSamples = 0U;
                *stillSamples = 0U;
                PRINTF("[FALL] Monitoring resumed.\r\n");
            }
            break;

        case FALL_STATE_DETECTED:
            /* Latch the alert to avoid repeated messages. Reset by restarting. */
            break;

        default:
            *state = FALL_STATE_NORMAL;
            *stateSamples = 0U;
            *stillSamples = 0U;
            break;
    }
    return (*state == FALL_STATE_DETECTED);
}

int main(void)
{
    status_t status;
    uint8_t whoAmI = 0U;
    MPU6050_Data_t data;
    MPU6050_Offsets_t offsets;

    int32_t accelX_mg;
    int32_t accelY_mg;
    int32_t accelZ_mg;
    int32_t gyroX_centi;
    int32_t gyroY_centi;
    int32_t gyroZ_centi;
    int32_t temperature_centi;
    uint32_t accelMagnitudeMg;

    bool isMoving = false;
    uint32_t consecutiveSamples = 0U;
    uint32_t printCounter = 0U;
    uint32_t fallStateSamples = 0U;
    uint32_t stillSamples = 0U;
    FallState_t fallState = FALL_STATE_NORMAL;

    BOARD_InitHardware();
    BOARD_Accel_I2C_Init();

    PRINTF("\r\n====================================\r\n");
    PRINTF(" FALCON MPU-6050 Fall Detection Test\r\n");
    PRINTF("====================================\r\n");
    PRINTF("I2C address: 0x%02X\r\n", MPU6050_ADDR);

    status = MPU6050_Read(MPU6050_REG_WHO_AM_I, &whoAmI, 1U);
    if ((status != kStatus_Success) || (whoAmI != MPU6050_WHO_AM_I_VALUE))
    {
        PRINTF("ERROR: MPU-6050 not detected correctly. WHO_AM_I=0x%02X\r\n", whoAmI);
        while (1)
        {
            SDK_DelayAtLeastUs(500000U, SystemCoreClock);
        }
    }
    PRINTF("MPU-6050 detected successfully.\r\n");

    status = MPU6050_Write(MPU6050_REG_PWR_MGMT_1, 0x00U);
    if (status != kStatus_Success)
    {
        PRINTF("ERROR: Could not wake MPU-6050.\r\n");
        while (1)
        {
            SDK_DelayAtLeastUs(500000U, SystemCoreClock);
        }
    }

    /* DLPF=3 gives a 1 kHz gyro sample base; divider=9 targets 100 Hz.
       Accel +/-8 g (AFS_SEL=2); gyro +/-500 dps (FS_SEL=1). */
    if ((MPU6050_Write(MPU6050_REG_SMPLRT_DIV, 0x09U) != kStatus_Success) ||
        (MPU6050_Write(MPU6050_REG_CONFIG, 0x03U) != kStatus_Success) ||
        (MPU6050_Write(MPU6050_REG_GYRO_CONFIG, 0x08U) != kStatus_Success) ||
        (MPU6050_Write(MPU6050_REG_ACCEL_CONFIG, 0x10U) != kStatus_Success))
    {
        PRINTF("ERROR: MPU-6050 configuration failed.\r\n");
        while (1)
        {
            SDK_DelayAtLeastUs(500000U, SystemCoreClock);
        }
    }

    PRINTF("Accelerometer range: +/-8 g\r\n");
    PRINTF("Gyroscope range: +/-500 degrees/s\r\n");
    PRINTF("Target sampling interval: 10 ms (100 Hz)\r\n");
    PRINTF("Fall thresholds are experimental; do not use as a safety guarantee.\r\n");
    PRINTF("Stillness progress is reported every 25 samples while waiting.\r\n");

    status = MPU6050_Calibrate(&offsets);
    if (status != kStatus_Success)
    {
        PRINTF("Calibration failed. Check sensor connections.\r\n");
        while (1)
        {
            SDK_DelayAtLeastUs(500000U, SystemCoreClock);
        }
    }

    PRINTF("\r\nMonitoring started. Keep sensor still, then test gentle movements.\r\n");
    PRINTF("Do not drop the development board.\r\n\r\n");

    while (1)
    {
        status = MPU6050_ReadData(&data);
        if (status == kStatus_Success)
        {
            /* Use raw acceleration axes for orientation-independent magnitude. */
            accelX_mg = (int32_t)data.accelX * 1000L / ACCEL_SENSITIVITY;
            accelY_mg = (int32_t)data.accelY * 1000L / ACCEL_SENSITIVITY;
            accelZ_mg = (int32_t)data.accelZ * 1000L / ACCEL_SENSITIVITY;

            gyroX_centi = ((int32_t)data.gyroX - offsets.gyroX) * 100L / GYRO_SENSITIVITY;
            gyroY_centi = ((int32_t)data.gyroY - offsets.gyroY) * 100L / GYRO_SENSITIVITY;
            gyroZ_centi = ((int32_t)data.gyroZ - offsets.gyroZ) * 100L / GYRO_SENSITIVITY;
            temperature_centi = ((int32_t)data.temperature * 100L) / 340L + 3653L;

            accelMagnitudeMg = MPU6050_AccelerationMagnitudeMg(
                accelX_mg, accelY_mg, accelZ_mg);

            {
                bool movementDetected = MPU6050_IsMoving(
                    accelMagnitudeMg, gyroX_centi, gyroY_centi, gyroZ_centi);
                if (movementDetected)
                {
                    if (consecutiveSamples < MOVEMENT_CONFIRM_SAMPLES)
                    {
                        consecutiveSamples++;
                    }
                    if (consecutiveSamples >= MOVEMENT_CONFIRM_SAMPLES)
                    {
                        isMoving = true;
                    }
                }
                else
                {
                    consecutiveSamples = 0U;
                    isMoving = false;
                }
            }

            (void)MPU6050_UpdateFallDetection(
                &fallState, accelMagnitudeMg,
                gyroX_centi, gyroY_centi, gyroZ_centi,
                &fallStateSamples, &stillSamples);

            printCounter++;
            if (printCounter >= PRINT_EVERY_SAMPLES)
            {
                printCounter = 0U;
                PRINTF("ACC: X=");
                MPU6050_PrintFixed(accelX_mg, 3U);
                PRINTF(" Y=");
                MPU6050_PrintFixed(accelY_mg, 3U);
                PRINTF(" Z=");
                MPU6050_PrintFixed(accelZ_mg, 3U);
                PRINTF(" g | MAG=");
                MPU6050_PrintFixed((int32_t)accelMagnitudeMg, 3U);
                PRINTF(" g | GYRO: X=");
                MPU6050_PrintFixed(gyroX_centi, 2U);
                PRINTF(" Y=");
                MPU6050_PrintFixed(gyroY_centi, 2U);
                PRINTF(" Z=");
                MPU6050_PrintFixed(gyroZ_centi, 2U);
                PRINTF(" deg/s | TEMP=");
                MPU6050_PrintFixed(temperature_centi, 2U);
                PRINTF(" C | STATUS: %s | FALL STATE: %s\r\n",
                       isMoving ? "MOVING" : "STATIONARY",
                       (fallState == FALL_STATE_NORMAL) ? "NORMAL" :
                       (fallState == FALL_STATE_WAIT_IMPACT) ? "WAIT_IMPACT" :
                       (fallState == FALL_STATE_WAIT_STILLNESS) ? "WAIT_STILLNESS" :
                       "DETECTED");
            }
        }
        else
        {
            PRINTF("ERROR: MPU-6050 data read failed.\r\n");
        }

        SDK_DelayAtLeastUs(SAMPLE_DELAY_US, SystemCoreClock);
    }
}

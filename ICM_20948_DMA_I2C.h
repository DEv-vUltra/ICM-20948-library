/*
 * ICM_20948_DMA_I2C.h
 *
 *  Created on: Jul 21, 2026
 *      Author: vultra_dev
 */

#ifndef INC_ICM_20948_DMA_I2C_H_
#define INC_ICM_20948_DMA_I2C_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "ICM_20948_I2C.h"

#define ICM_AXIS_COUNT                 (3U)
#define ICM_RAW_PACKET_LENGTH          (23U)
#define ICM_MAX_REGISTER_READ_LENGTH   (32U)
#define ICM_I2C_TIMEOUT_MS             (20U)
#define ICM_DMA_TIMEOUT_MS             (20U)
#define ICM_RECOVERY_MAX               (3U)
#define ICM_MAG_SKIP_MAX               (50U)
#define ICM_MAG_OVF_MAX                (10U)
#define ICM_GYRO_BIAS_LIMIT_LSB        (1310.0F)

/* Sensitivity LSB Scale Factors */
#define ICM_GYRO_LSB_PER_DPS_250       (131.0F)
#define ICM_GYRO_LSB_PER_DPS_500       (65.5F)
#define ICM_GYRO_LSB_PER_DPS_1000      (32.8F)
#define ICM_GYRO_LSB_PER_DPS_2000      (16.4F)

#define ICM_ACCEL_LSB_PER_G_2          (16384.0F)
#define ICM_ACCEL_LSB_PER_G_4          (8192.0F)
#define ICM_ACCEL_LSB_PER_G_8          (4096.0F)
#define ICM_ACCEL_LSB_PER_G_16         (2048.0F)

#define ICM_MAG_UT_PER_LSB             (0.15F)
#define ICM_TEMP_LSB_PER_DEG_C         (333.87F)
#define ICM_TEMP_OFFSET_C               (21.0F)
#define ICM_SATURATION_LIMIT            ((int16_t)32512)

typedef enum
{
    ICM_STATE_IDLE = 0U,
    ICM_STATE_DMA_BUSY,
    ICM_STATE_DATA_READY,
    ICM_STATE_ERROR
} ICM_State_t;

typedef enum
{
    ICM_MAG_OK = 0U,
    ICM_MAG_NOT_READY,
    ICM_MAG_OVERFLOW
} ICM_MagResult_t;

typedef enum
{
    ICM_FAULT_TRANSFER        = (1UL << 0U),
    ICM_FAULT_WHO_AM_I        = (1UL << 1U),
    ICM_FAULT_BANK_SELECT     = (1UL << 2U),
    ICM_FAULT_DMA             = (1UL << 3U),
    ICM_FAULT_DMA_TIMEOUT     = (1UL << 4U),
    ICM_FAULT_MAG_NACK        = (1UL << 5U),
    ICM_FAULT_MAG_NO_RESPONSE = (1UL << 6U),
    ICM_FAULT_MAG_OVERFLOW    = (1UL << 7U),
    ICM_FAULT_ACCEL_SATURATED = (1UL << 8U),
    ICM_FAULT_GYRO_SATURATED  = (1UL << 9U),
    ICM_FAULT_GYRO_BIAS       = (1UL << 10U),
    ICM_FAULT_MAG_BIAS        = (1UL << 11U),
    ICM_FAULT_RECOVERY_LIMIT  = (1UL << 12U)
} ICM_Fault_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t i2c_address;
//    GPIO_TypeDef *int_port;  /* Port GPIO connect INT pin (GPIOA) */
//    uint16_t int_pin;        /* Pin GPIO  connect INT pin (GPIO_PIN_0) */
} ICM_HW_t;

typedef struct
{
    uint8_t rx_buffer [ICM_RAW_PACKET_LENGTH];
    uint8_t raw_buffer[ICM_RAW_PACKET_LENGTH];
    uint32_t start_tick;
} ICM_DMA_t;

typedef struct
{
    float accel_g [ICM_AXIS_COUNT];
    float gyro_dps[ICM_AXIS_COUNT];
    float mag_ut  [ICM_AXIS_COUNT];
    float temp_c;
} ICM_Data_t;

typedef struct
{
    float gyro_bias [ICM_AXIS_COUNT];
    float mag_bias  [ICM_AXIS_COUNT];
    float accel_bias[ICM_AXIS_COUNT];
} ICM_Calib_t;

typedef struct
{
    volatile ICM_State_t state;
    ICM_Bank_t current_bank;
    uint32_t fault_flags;
    uint8_t mag_skip_count;
    uint8_t mag_overflow_count;
    uint8_t recovery_count;
    bool mag_data_ready;
    bool mag_overflow;
    bool mag_data_overrun;
    bool mag_available;
} ICM_Status_t;

typedef struct
{
    const ICM_HW_t *phw;
    ICM_DMA_t dma;
    ICM_Data_t data;
    ICM_Calib_t calib;
    ICM_Status_t status;
} ICM_Handle_t;

HAL_StatusTypeDef ICM_Init(ICM_Handle_t *h, const ICM_HW_t *hw_config);
HAL_StatusTypeDef ICM_WriteReg(ICM_Handle_t *h, uint16_t icm_reg, uint8_t data);
HAL_StatusTypeDef ICM_ReadReg(ICM_Handle_t *h, uint16_t icm_reg, uint8_t *buffer, uint16_t length);
HAL_StatusTypeDef ICM_StartBurstRead(ICM_Handle_t *h);

void ICM_DMA_Callback(ICM_Handle_t *h);
void ICM_DMA_ErrorCallback(ICM_Handle_t *h);
void ICM_ProcessRaw(ICM_Handle_t *h);
void ICM_CheckTimeout(ICM_Handle_t *h);
void ICM_ErrorRecovery(ICM_Handle_t *h);
void ICM_CalibrateGyro(ICM_Handle_t *h, uint16_t samples);
void ICM_CalibrateMag(ICM_Handle_t *h, uint32_t samples);

#endif /* INC_ICM_20948_DMA_I2C_H_ */

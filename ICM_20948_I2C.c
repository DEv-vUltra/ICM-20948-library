#include "ICM_20948_DMA_I2C.h"

#include <limits.h>
#include <string.h>

#define ICM_PACKET_ACCEL_OFFSET      (0U)
#define ICM_PACKET_GYRO_OFFSET       (6U)
#define ICM_PACKET_TEMP_OFFSET       (12U)
#define ICM_PACKET_MAG_ST1_OFFSET    (14U)
#define ICM_PACKET_MAG_OFFSET        (15U)
#define ICM_PACKET_MAG_ST2_OFFSET    (22U)

static bool ICM_IsHandleValid(const ICM_Handle_t *h)
{
    return ((h != NULL) && (h->phw != NULL) && (h->phw->hi2c != NULL));
}

static void ICM_SetFault(ICM_Handle_t *h, uint32_t fault)
{
    h->status.fault_flags |= fault;
}

static uint8_t ICM_RegisterBank(uint16_t reg)
{
    return (uint8_t)(reg >> 8U);
}

static uint8_t ICM_RegisterAddress(uint16_t reg)
{
    return (uint8_t)(reg & 0x00FFU);
}

static int16_t ICM_DecodeS16BE(const uint8_t *buffer, uint8_t offset)
{
    const uint16_t value = ((uint16_t)buffer[offset] << 8U) | (uint16_t)buffer[offset + 1U];
    int32_t signed_value = (int32_t)value;

    if ((value & 0x8000U) != 0U)
    {
        signed_value -= 65536L;
    }

    return (int16_t)signed_value;
}

static int16_t ICM_DecodeS16LE(const uint8_t *buffer, uint8_t offset)
{
    const uint16_t value = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1U] << 8U);
    int32_t signed_value = (int32_t)value;

    if ((value & 0x8000U) != 0U)
    {
        signed_value -= 65536L;
    }

    return (int16_t)signed_value;
}

static HAL_StatusTypeDef ICM_SelectBank(ICM_Handle_t *h, ICM_Bank_t bank)
{
    HAL_StatusTypeDef result;
    uint8_t bank_value;

    if ((!ICM_IsHandleValid(h)) || ((uint32_t)bank > (uint32_t)ICM_BANK_3))
    {
        return HAL_ERROR;
    }

    if (h->status.current_bank == bank)
    {
        return HAL_OK;
    }

    bank_value = (uint8_t)((uint8_t)bank << 4U);
    result = HAL_I2C_Mem_Write(h->phw->hi2c, h->phw->i2c_address, ICM_REG_BANK_SELECT, I2C_MEMADD_SIZE_8BIT, &bank_value, 1U, ICM_I2C_TIMEOUT_MS);
    if (result == HAL_OK)
    {
        h->status.current_bank = bank;
    }
    else
    {
        ICM_SetFault(h, ICM_FAULT_BANK_SELECT | ICM_FAULT_TRANSFER);
    }

    return result;
}

HAL_StatusTypeDef ICM_WriteReg(ICM_Handle_t *h, uint16_t icm_reg, uint8_t data)
{
    HAL_StatusTypeDef result;
    const ICM_Bank_t bank = (ICM_Bank_t)ICM_RegisterBank(icm_reg);

    if (!ICM_IsHandleValid(h))
    {
        return HAL_ERROR;
    }
    if (h->status.state == ICM_STATE_DMA_BUSY)
    {
        return HAL_BUSY;
    }

    result = ICM_SelectBank(h, bank);
    if (result != HAL_OK)
    {
        return result;
    }

    result = HAL_I2C_Mem_Write(h->phw->hi2c, h->phw->i2c_address, ICM_RegisterAddress(icm_reg), I2C_MEMADD_SIZE_8BIT, &data, 1U, ICM_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        ICM_SetFault(h, ICM_FAULT_TRANSFER);
    }

    return result;
}

HAL_StatusTypeDef ICM_ReadReg(ICM_Handle_t *h, uint16_t icm_reg, uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef result;
    const ICM_Bank_t bank = (ICM_Bank_t)ICM_RegisterBank(icm_reg);

    if ((!ICM_IsHandleValid(h)) || (buffer == NULL) || (length == 0U) ||
        (length > ICM_MAX_REGISTER_READ_LENGTH))
    {
        return HAL_ERROR;
    }
    if (h->status.state == ICM_STATE_DMA_BUSY)
    {
        return HAL_BUSY;
    }

    result = ICM_SelectBank(h, bank);
    if (result != HAL_OK)
    {
        return result;
    }

    result = HAL_I2C_Mem_Read(h->phw->hi2c, h->phw->i2c_address, ICM_RegisterAddress(icm_reg), I2C_MEMADD_SIZE_8BIT, buffer, length, ICM_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        ICM_SetFault(h, ICM_FAULT_TRANSFER);
    }

    return result;
}

static HAL_StatusTypeDef ICM_ConfigureMagnetometer(ICM_Handle_t *h)
{
    HAL_StatusTypeDef result;
    uint8_t status = 0U;
    uint8_t wia2 = 0U;

    result = ICM_WriteReg(h, ICM_REG_INT_PIN_CFG, 0U);

    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_USER_CTRL,
                              ICM_USER_CTRL_I2C_MST_EN | ICM_USER_CTRL_I2C_MST_RST);
    }
    if (result == HAL_OK)
    {
        HAL_Delay(1U);
        result = ICM_WriteReg(h, ICM_REG_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_MST_CTRL, ICM_I2C_MST_CLK_345KHZ);
    }

    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_ADDR,
                              AK09916_I2C_ADDRESS | ICM_I2C_SLV_READ);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_REG, AK09916_REG_WIA2);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_CTRL, ICM_I2C_SLV_ENABLE | 1U);
    }
    if (result == HAL_OK)
    {
        HAL_Delay(10U);
        result = ICM_ReadReg(h, ICM_REG_I2C_MST_STATUS, &status, 1U);
    }
    if ((result == HAL_OK) && ((status & ICM_I2C_MST_STATUS_SLV0_NACK) != 0U))
    {
        ICM_SetFault(h, ICM_FAULT_MAG_NO_RESPONSE);
        return HAL_ERROR;
    }
    if (result == HAL_OK)
    {
        result = ICM_ReadReg(h, ICM_REG_EXT_SLV_SENS_DATA_00, &wia2, 1U);
    }
    if ((result == HAL_OK) && (wia2 != AK09916_DEVICE_ID))
    {
        ICM_SetFault(h, ICM_FAULT_MAG_NO_RESPONSE);
        return HAL_ERROR;
    }

    result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDRESS);
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_REG, AK09916_REG_CNTL2);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_DO, AK09916_MODE_CONTINUOUS_100HZ);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_CTRL, ICM_I2C_SLV_ENABLE | 1U);
    }

    if (result == HAL_OK)
    {
        HAL_Delay(10U);
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_CTRL, 0U);
    }
    if (result == HAL_OK)
    {
        result = ICM_ReadReg(h, ICM_REG_I2C_MST_STATUS, &status, 1U);
    }
    if ((result == HAL_OK) && ((status & ICM_I2C_MST_STATUS_SLV0_NACK) != 0U))
    {
        ICM_SetFault(h, ICM_FAULT_MAG_NACK);
        result = HAL_ERROR;
    }

    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDRESS | ICM_I2C_SLV_READ);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_REG, AK09916_REG_ST1);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_I2C_SLV0_CTRL, ICM_I2C_SLV_ENABLE | 9U);
    }

    return result;
}

static HAL_StatusTypeDef ICM_ConfigureSensors(ICM_Handle_t *h)
{
    HAL_StatusTypeDef result;

    /* Given SMPLRT_DIV = 21 -> ODR = 1125 / (1 + 21) = ~51.13 Hz */
    const uint8_t divider = ICM_SMPLRT_DIV_225HZ;
    const uint8_t accel_config = ICM_ACCEL_CONFIG_FCHOICE | ICM_ACCEL_CONFIG_FS_8G | ICM_ACCEL_CONFIG_DLPF_6HZ;
    const uint8_t gyro_config = ICM_GYRO_CONFIG_1_FCHOICE | ICM_GYRO_CONFIG_1_FS_500DPS | ICM_GYRO_CONFIG_1_DLPF_6HZ;

    /* 1. Config Sample Rate Dividers */
    result = ICM_WriteReg(h, ICM_REG_GYRO_SMPLRT_DIV, divider);
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_ACCEL_SMPLRT_DIV_1, 0U);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_ACCEL_SMPLRT_DIV_2, divider);
    }

    /* 2. Config FS and DLPF for Accelerometer & Gyroscope */
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_ACCEL_CONFIG, accel_config);
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_GYRO_CONFIG_1, gyro_config);
    }

    /* 3. Config interrupt pin Hardware (INT1 Pin & Interrupt Enable) */
//    if (result == HAL_OK)
//    {
//        /* Act High, Push-Pull, Delete interrupt when read any registers */
//        const uint8_t int_pin_cfg = ICM_INT_PIN_CFG_ACTL_HIGH | ICM_INT_PIN_CFG_PUSH_PULL;
//        result = ICM_WriteReg(h, ICM_REG_INT_PIN_CFG, int_pin_cfg);
//    }
//    if (result == HAL_OK)
//    {
//        /* Set interrupt RAW_DATA_0_RDY to pull up High for INT when having new samples */
//        result = ICM_WriteReg(h, ICM_REG_INT_ENABLE_1, ICM_INT_ENABLE_1_RAW_DATA_0_RDY_EN);
//    }

    return result;
}

static HAL_StatusTypeDef ICM_Configure(ICM_Handle_t *h)
{
    HAL_StatusTypeDef result;
    uint8_t id = 0U;

    if (!ICM_IsHandleValid(h))
    {
        return HAL_ERROR;
    }

    h->status.current_bank = ICM_BANK_INVALID;
    HAL_Delay(10U);
    result = ICM_ReadReg(h, ICM_REG_WHO_AM_I, &id, 1U);
    if ((result == HAL_OK) && (id != ICM_WHO_AM_I_VALUE))
    {
        ICM_SetFault(h, ICM_FAULT_WHO_AM_I);
        result = HAL_ERROR;
    }
    if (result == HAL_OK)
    {
        result = ICM_WriteReg(h, ICM_REG_PWR_MGMT_1, ICM_PWR_MGMT_1_DEVICE_RESET);
    }
    if (result == HAL_OK)
    {
        HAL_Delay(100U);
        h->status.current_bank = ICM_BANK_INVALID;
        result = ICM_WriteReg(h, ICM_REG_PWR_MGMT_1, ICM_PWR_MGMT_1_CLKSEL_AUTO);
    }
    if (result == HAL_OK)
    {
        HAL_Delay(10U);
        result = ICM_WriteReg(h, ICM_REG_PWR_MGMT_2, ICM_PWR_MGMT_2_ENABLE_ALL);
    }
    if (result == HAL_OK)
    {
        h->status.mag_available = (ICM_ConfigureMagnetometer(h) == HAL_OK);
    }
    if (result == HAL_OK)
    {
        result = ICM_ConfigureSensors(h);
    }
    if (result == HAL_OK)
    {
        result = ICM_ReadReg(h, ICM_REG_WHO_AM_I, &id, 1U);
        if (id != ICM_WHO_AM_I_VALUE)
        {
            ICM_SetFault(h, ICM_FAULT_WHO_AM_I);
            result = HAL_ERROR;
        }
    }
    if (result == HAL_OK)
    {
        result = ICM_SelectBank(h, ICM_BANK_0);
    }

    return result;
}

HAL_StatusTypeDef ICM_Init(ICM_Handle_t *h, const ICM_HW_t *hw_config)
{
    HAL_StatusTypeDef result;

    if ((h == NULL) || (hw_config == NULL) || (hw_config->hi2c == NULL))
    {
        return HAL_ERROR;
    }

    (void)memset(h, 0, sizeof(*h));
    h->phw = hw_config;
    h->status.current_bank = ICM_BANK_INVALID;
    result = ICM_Configure(h);
    if (result == HAL_OK)
    {
        h->status.state = ICM_STATE_IDLE;
    }
    else
    {
        h->status.state = ICM_STATE_ERROR;
    }

    return result;
}

HAL_StatusTypeDef ICM_StartBurstRead(ICM_Handle_t *h)
{
    HAL_StatusTypeDef result;

    if (!ICM_IsHandleValid(h))
    {
        return HAL_ERROR;
    }
    if (h->status.state != ICM_STATE_IDLE)
    {
        return HAL_BUSY;
    }

    /* Checking for Interrupt PA0:
       If int_port and int_pin is config (!= NULL), read state of PA0.
       If PA0 = LOW (ICM doesn't notify Data Ready), return HAL_BUSY to avoid read repeatedly. */
//    if ((h->phw->int_port != NULL) &&
//        (HAL_GPIO_ReadPin(h->phw->int_port, h->phw->int_pin) == GPIO_PIN_RESET))
//    {
//        return HAL_BUSY;
//    }

    result = ICM_SelectBank(h, ICM_BANK_0);
    if (result != HAL_OK)
    {
        h->status.state = ICM_STATE_ERROR;
        return result;
    }

    h->dma.start_tick = HAL_GetTick();
    h->status.state = ICM_STATE_DMA_BUSY;
    result = HAL_I2C_Mem_Read_DMA(h->phw->hi2c, h->phw->i2c_address, ICM_RegisterAddress(ICM_REG_ACCEL_XOUT_H), I2C_MEMADD_SIZE_8BIT, h->dma.rx_buffer, ICM_RAW_PACKET_LENGTH);
    if (result != HAL_OK)
    {
        h->status.state = ICM_STATE_IDLE;
        if (result != HAL_BUSY)
        {
            ICM_SetFault(h, ICM_FAULT_DMA | ICM_FAULT_TRANSFER);
            h->status.state = ICM_STATE_ERROR;
        }
    }

    return result;
}

void ICM_DMA_Callback(ICM_Handle_t *h)
{
    if ((!ICM_IsHandleValid(h)) || (h->status.state != ICM_STATE_DMA_BUSY))
    {
        return;
    }

    (void)memcpy(h->dma.raw_buffer, h->dma.rx_buffer, ICM_RAW_PACKET_LENGTH);
    h->status.state = ICM_STATE_DATA_READY;
}

void ICM_DMA_ErrorCallback(ICM_Handle_t *h)
{
    if ((!ICM_IsHandleValid(h)) || (h->status.state != ICM_STATE_DMA_BUSY))
    {
        return;
    }

    ICM_SetFault(h, ICM_FAULT_DMA | ICM_FAULT_TRANSFER);
    h->status.state = ICM_STATE_ERROR;
}

static void ICM_ProcessAccel(ICM_Handle_t *h, const uint8_t *packet)
{
    uint8_t axis;
    bool saturated = false;

    for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
    {
        const int16_t raw = ICM_DecodeS16BE(packet, (uint8_t)(ICM_PACKET_ACCEL_OFFSET + (axis * 2U)));
        h->data.accel_g[axis] = ((float)raw - h->calib.accel_bias[axis]) / ICM_ACCEL_LSB_PER_G_8;
        if ((raw >= ICM_SATURATION_LIMIT) || (raw <= -ICM_SATURATION_LIMIT))
        {
            saturated = true;
        }
    }
    if (saturated)
    {
        ICM_SetFault(h, ICM_FAULT_ACCEL_SATURATED);
    }
}

static void ICM_ProcessGyro(ICM_Handle_t *h, const uint8_t *packet)
{
    uint8_t axis;
    bool saturated = false;

    for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
    {
        const int16_t raw = ICM_DecodeS16BE(packet, (uint8_t)(ICM_PACKET_GYRO_OFFSET + (axis * 2U)));
        h->data.gyro_dps[axis] = ((float)raw - h->calib.gyro_bias[axis]) / ICM_GYRO_LSB_PER_DPS_500;
        if ((raw >= ICM_SATURATION_LIMIT) || (raw <= -ICM_SATURATION_LIMIT))
        {
            saturated = true;
        }
    }
    if (saturated)
    {
        ICM_SetFault(h, ICM_FAULT_GYRO_SATURATED);
    }
}

static ICM_MagResult_t ICM_ProcessMagnetometer(ICM_Handle_t *h, const uint8_t *packet)
{
    uint8_t axis;

    h->status.mag_data_ready = ((packet[ICM_PACKET_MAG_ST1_OFFSET] & AK09916_ST1_DRDY) != 0U);
    h->status.mag_data_overrun = ((packet[ICM_PACKET_MAG_ST1_OFFSET] & AK09916_ST1_DOR) != 0U);
    if (!h->status.mag_data_ready)
    {
        if (h->status.mag_skip_count < UINT8_MAX)
        {
            ++h->status.mag_skip_count;
        }
        if (h->status.mag_skip_count >= ICM_MAG_SKIP_MAX)
        {
            ICM_SetFault(h, ICM_FAULT_MAG_NO_RESPONSE);
        }
        return ICM_MAG_NOT_READY;
    }
    h->status.mag_skip_count = 0U;

    h->status.mag_overflow = ((packet[ICM_PACKET_MAG_ST2_OFFSET] & AK09916_ST2_HOFL) != 0U);
    if (h->status.mag_overflow)
    {
        if (h->status.mag_overflow_count < UINT8_MAX)
        {
            ++h->status.mag_overflow_count;
        }
        if (h->status.mag_overflow_count >= ICM_MAG_OVF_MAX)
        {
            ICM_SetFault(h, ICM_FAULT_MAG_OVERFLOW);
        }
        return ICM_MAG_OVERFLOW;
    }
    h->status.mag_overflow_count = 0U;

    for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
    {
        const int16_t raw = ICM_DecodeS16LE(packet, (uint8_t)(ICM_PACKET_MAG_OFFSET + (axis * 2U)));
        h->data.mag_ut[axis] = ((float)raw - h->calib.mag_bias[axis]) * ICM_MAG_UT_PER_LSB;
    }

    return ICM_MAG_OK;
}

void ICM_ProcessRaw(ICM_Handle_t *h)
{
    if ((!ICM_IsHandleValid(h)) || (h->status.state != ICM_STATE_DATA_READY))
    {
        return;
    }

    ICM_ProcessAccel(h, h->dma.raw_buffer);
    ICM_ProcessGyro(h, h->dma.raw_buffer);
    h->data.temp_c = ((float)ICM_DecodeS16BE(h->dma.raw_buffer, ICM_PACKET_TEMP_OFFSET) / ICM_TEMP_LSB_PER_DEG_C) + ICM_TEMP_OFFSET_C;
    if (h->status.mag_available)
    {
        (void)ICM_ProcessMagnetometer(h, h->dma.raw_buffer);
    }
    h->status.state = ICM_STATE_IDLE;
}

void ICM_CheckTimeout(ICM_Handle_t *h)
{
    if ((!ICM_IsHandleValid(h)) || (h->status.state != ICM_STATE_DMA_BUSY))
    {
        return;
    }
    const uint32_t elapsed = HAL_GetTick() - h->dma.start_tick;
    
    if (elapsed >= ICM_DMA_TIMEOUT_MS)
    {
        ICM_SetFault(h, ICM_FAULT_DMA_TIMEOUT | ICM_FAULT_DMA);
        h->status.state = ICM_STATE_ERROR;
    }
}

void ICM_ErrorRecovery(ICM_Handle_t *h)
{
    HAL_StatusTypeDef result;
    uint8_t recovery_count;

    if ((!ICM_IsHandleValid(h)) || (h->status.state != ICM_STATE_ERROR))
    {
        return;
    }
    if (h->status.recovery_count >= ICM_RECOVERY_MAX)
    {
        ICM_SetFault(h, ICM_FAULT_RECOVERY_LIMIT);
        return;
    }

    ++h->status.recovery_count;
    recovery_count = h->status.recovery_count;
    (void)HAL_I2C_DeInit(h->phw->hi2c);
    HAL_Delay(1U);
    result = HAL_I2C_Init(h->phw->hi2c);
    if (result == HAL_OK)
    {
        result = ICM_Configure(h);
    }
    if (result == HAL_OK)
    {
        const uint32_t transient_faults = (uint32_t)ICM_FAULT_TRANSFER | (uint32_t)ICM_FAULT_BANK_SELECT | (uint32_t)ICM_FAULT_DMA | (uint32_t)ICM_FAULT_DMA_TIMEOUT | (uint32_t)ICM_FAULT_MAG_NACK;
        h->status.fault_flags &= ~transient_faults;
        h->status.recovery_count = recovery_count;
        h->status.state = ICM_STATE_IDLE;
    }
}

void ICM_CalibrateGyro(ICM_Handle_t *h, uint16_t samples)
{
    int32_t sum[ICM_AXIS_COUNT] = { 0L, 0L, 0L };
    uint16_t accepted = 0U;
    uint16_t sample;
    uint8_t axis;
    bool bias_suspect = false;
    uint8_t packet[ICM_RAW_PACKET_LENGTH];

    if ((!ICM_IsHandleValid(h)) || (samples == 0U) || (h->status.state != ICM_STATE_IDLE))
    {
        return;
    }

    for (sample = 0U; sample < samples; ++sample)
    {
        if (ICM_ReadReg(h, ICM_REG_ACCEL_XOUT_H, packet, ICM_RAW_PACKET_LENGTH) != HAL_OK)
        {
            break;
        }
        for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
        {
            sum[axis] += (int32_t)ICM_DecodeS16BE(packet, (uint8_t)(ICM_PACKET_GYRO_OFFSET + (axis * 2U)));
        }
        ++accepted;
        HAL_Delay(20U);
    }

    if (accepted == 0U)
    {
        ICM_SetFault(h, ICM_FAULT_GYRO_BIAS);
        return;
    }
    for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
    {
        h->calib.gyro_bias[axis] = (float)sum[axis] / (float)accepted;
        if ((h->calib.gyro_bias[axis] > ICM_GYRO_BIAS_LIMIT_LSB) || (h->calib.gyro_bias[axis] < -ICM_GYRO_BIAS_LIMIT_LSB))
        {
            bias_suspect = true;
        }
    }
    if (bias_suspect)
    {
        ICM_SetFault(h, ICM_FAULT_GYRO_BIAS);
    }
}

void ICM_CalibrateMag(ICM_Handle_t *h, uint32_t samples)
{
    int16_t maximum[ICM_AXIS_COUNT] = { INT16_MIN, INT16_MIN, INT16_MIN };
    int16_t minimum[ICM_AXIS_COUNT] = { INT16_MAX, INT16_MAX, INT16_MAX };
    uint32_t accepted = 0U;
    uint32_t sample;
    uint8_t axis;
    uint8_t packet[ICM_RAW_PACKET_LENGTH];

    if ((!ICM_IsHandleValid(h)) || (samples == 0U) || (h->status.state != ICM_STATE_IDLE))
    {
        return;
    }

    for (sample = 0U; sample < samples; ++sample)
    {
        if (ICM_ReadReg(h, ICM_REG_ACCEL_XOUT_H, packet, ICM_RAW_PACKET_LENGTH) != HAL_OK)
        {
            break;
        }
        if (((packet[ICM_PACKET_MAG_ST1_OFFSET] & AK09916_ST1_DRDY) != 0U) && ((packet[ICM_PACKET_MAG_ST2_OFFSET] & AK09916_ST2_HOFL) == 0U))
        {
            for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
            {
                const int16_t value = ICM_DecodeS16LE(packet, (uint8_t)(ICM_PACKET_MAG_OFFSET + (axis * 2U)));
                if (value > maximum[axis])
                {
                    maximum[axis] = value;
                }
                if (value < minimum[axis])
                {
                    minimum[axis] = value;
                }
            }
            ++accepted;
        }
        HAL_Delay(10U);
    }

    if (accepted == 0U)
    {
        ICM_SetFault(h, ICM_FAULT_MAG_BIAS);
        return;
    }
    for (axis = 0U; axis < ICM_AXIS_COUNT; ++axis)
    {
        h->calib.mag_bias[axis] = ((float)maximum[axis] + (float)minimum[axis]) * 0.5F;
    }
}

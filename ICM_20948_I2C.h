/* ICM-20948 register definitions. */
#ifndef ICM_20948_I2C_H
#define ICM_20948_I2C_H

#include <stdint.h>

typedef enum
{
    ICM_BANK_0 = 0U,
    ICM_BANK_1 = 1U,
    ICM_BANK_2 = 2U,
    ICM_BANK_3 = 3U,
    ICM_BANK_INVALID = 4U
} ICM_Bank_t;

/* Combines Bank (upper 8 bits) and Register Address (lower 8 bits) into a 16-bit code */
#define ICM_REG(bank_, address_) \
    ((uint16_t)((((uint16_t)(bank_)) << 8U) | ((uint16_t)(address_))))

/* --- BANK 0 REGISTERS (General, Power, Interrupts, Status) --- */
#define ICM_REG_BANK_SELECT             (0x7FU)
#define ICM_REG_WHO_AM_I                ICM_REG(ICM_BANK_0, 0x00U)
#define ICM_REG_USER_CTRL               ICM_REG(ICM_BANK_0, 0x03U)
#define ICM_REG_PWR_MGMT_1              ICM_REG(ICM_BANK_0, 0x06U)
#define ICM_REG_PWR_MGMT_2              ICM_REG(ICM_BANK_0, 0x07U)
#define ICM_REG_INT_PIN_CFG             ICM_REG(ICM_BANK_0, 0x0FU)
#define ICM_REG_INT_ENABLE_1            ICM_REG(ICM_BANK_0, 0x11U)
#define ICM_REG_I2C_MST_STATUS          ICM_REG(ICM_BANK_0, 0x17U)
#define ICM_REG_ACCEL_XOUT_H            ICM_REG(ICM_BANK_0, 0x2DU)
#define ICM_REG_EXT_SLV_SENS_DATA_00    ICM_REG(ICM_BANK_0, 0x3BU)

/* --- BANK 2 REGISTERS (Gyroscope & Accelerometer Configuration) --- */
#define ICM_REG_GYRO_SMPLRT_DIV         ICM_REG(ICM_BANK_2, 0x00U)
#define ICM_REG_GYRO_CONFIG_1           ICM_REG(ICM_BANK_2, 0x01U)
#define ICM_REG_ACCEL_SMPLRT_DIV_1      ICM_REG(ICM_BANK_2, 0x10U)
#define ICM_REG_ACCEL_SMPLRT_DIV_2      ICM_REG(ICM_BANK_2, 0x11U)
#define ICM_REG_ACCEL_CONFIG            ICM_REG(ICM_BANK_2, 0x14U)

/* --- BANK 3 REGISTERS (Auxiliary I2C Master for Magnetometer) --- */
#define ICM_REG_I2C_MST_CTRL            ICM_REG(ICM_BANK_3, 0x01U)
#define ICM_REG_I2C_SLV0_ADDR           ICM_REG(ICM_BANK_3, 0x03U)
#define ICM_REG_I2C_SLV0_REG            ICM_REG(ICM_BANK_3, 0x04U)
#define ICM_REG_I2C_SLV0_CTRL           ICM_REG(ICM_BANK_3, 0x05U)
#define ICM_REG_I2C_SLV0_DO             ICM_REG(ICM_BANK_3, 0x06U)

#define ICM_WHO_AM_I_VALUE              (0xEAU) /* Standard ICM-20948 WHO_AM_I ID value */

/* System Control Bitfields (USER_CTRL, INT_PIN_CFG, PWR_MGMT) */
#define ICM_USER_CTRL_I2C_MST_EN        (0x20U) /* Bit 5: Enable I2C Master mode */
#define ICM_USER_CTRL_I2C_MST_RST       (0x02U) /* Bit 1: Reset I2C Master module */
#define ICM_INT_PIN_CFG_BYPASS_EN       (0x02U) /* Bit 1: Enable magnetometer bypass mode */
#define ICM_PWR_MGMT_1_DEVICE_RESET     (0x80U) /* Bit 7: Reset entire device */
#define ICM_PWR_MGMT_1_CLKSEL_AUTO      (0x01U) /* Bits [2:0]: Automatically select best clock source */
#define ICM_PWR_MGMT_2_ENABLE_ALL       (0x00U) /* Enable all Gyro and Accel axes */

/* Interrupt Pin & Enable Configurations */
#define ICM_INT_PIN_CFG_ACTL_HIGH       (0x00U) /* Interrupt pin active level: HIGH */
#define ICM_INT_PIN_CFG_ACTL_LOW        (0x80U) /* Interrupt pin active level: LOW */
#define ICM_INT_PIN_CFG_PUSH_PULL       (0x00U) /* Pin driver type: Push-Pull */
#define ICM_INT_PIN_CFG_OPEN_DRAIN      (0x40U) /* Pin driver type: Open-Drain */
#define ICM_INT_PIN_CFG_LATCH_EN        (0x20U) /* Latch interrupt until read/cleared */
#define ICM_INT_PIN_CFG_50US_PULSE      (0x00U) /* 50us interrupt pulse width */
#define ICM_INT_PIN_CFG_ANYRD_2CLEAR    (0x10U) /* Clear interrupt on any register read */

#define ICM_INT_ENABLE_1_RAW_DATA_0_RDY_EN (0x01U) /* Enable Raw Data Ready interrupt */

/* --- SAMPLE RATE DIVIDER (SMPLRT_DIV) --- */
/* ODR Formula: ODR = 1125Hz / (1 + SMPLRT_DIV) */
#define ICM_SMPLRT_DIV_1125HZ           (0U)   /* ODR = 1125.0 Hz */
#define ICM_SMPLRT_DIV_225HZ            (4U)   /* ODR =  225.0 Hz */
#define ICM_SMPLRT_DIV_102HZ            (10U)  /* ODR = ~102.27 Hz */
#define ICM_SMPLRT_DIV_51HZ             (21U)  /* ODR = ~51.13 Hz  */

/* --- GYROSCOPE CONFIGURATION 1 --- */
#define ICM_GYRO_CONFIG_1_FCHOICE       (0x01U) /* Bit 0: Enable DLPF for Gyroscope */

/* Gyro Full Scale Range (GYRO_FS_SEL - Bits [2:1]) */
#define ICM_GYRO_CONFIG_1_FS_250DPS     (0x00U) /* ±250 dps  */
#define ICM_GYRO_CONFIG_1_FS_500DPS     (0x02U) /* ±500 dps  */
#define ICM_GYRO_CONFIG_1_FS_1000DPS    (0x04U) /* ±1000 dps */
#define ICM_GYRO_CONFIG_1_FS_2000DPS    (0x06U) /* ±2000 dps */

/* Gyro DLPF Cutoff Frequency (GYRO_DLPFCFG - Bits [5:3]) */
#define ICM_GYRO_CONFIG_1_DLPF_196HZ    (0x00U)
#define ICM_GYRO_CONFIG_1_DLPF_151HZ    (0x08U)
#define ICM_GYRO_CONFIG_1_DLPF_120HZ    (0x10U)
#define ICM_GYRO_CONFIG_1_DLPF_51HZ     (0x18U)
#define ICM_GYRO_CONFIG_1_DLPF_24HZ     (0x20U)
#define ICM_GYRO_CONFIG_1_DLPF_12HZ     (0x28U)
#define ICM_GYRO_CONFIG_1_DLPF_6HZ      (0x30U)
#define ICM_GYRO_CONFIG_1_DLPF_361HZ    (0x38U)

/* --- ACCELEROMETER CONFIGURATION --- */
#define ICM_ACCEL_CONFIG_FCHOICE        (0x01U) /* Bit 0: Enable DLPF for Accelerometer */

/* Accel Full Scale Range (ACCEL_FS_SEL - Bits [2:1]) */
#define ICM_ACCEL_CONFIG_FS_2G          (0x00U) /* ±2G  */
#define ICM_ACCEL_CONFIG_FS_4G          (0x02U) /* ±4G  */
#define ICM_ACCEL_CONFIG_FS_8G          (0x04U) /* ±8G  */
#define ICM_ACCEL_CONFIG_FS_16G         (0x06U) /* ±16G */

/* Accel DLPF Cutoff Frequency (ACCEL_DLPFCFG - Bits [5:3]) */
#define ICM_ACCEL_CONFIG_DLPF_246HZ    (0x00U)
#define ICM_ACCEL_CONFIG_DLPF_111HZ    (0x10U)
#define ICM_ACCEL_CONFIG_DLPF_50HZ     (0x18U)
#define ICM_ACCEL_CONFIG_DLPF_24HZ     (0x20U)
#define ICM_ACCEL_CONFIG_DLPF_12HZ     (0x28U)
#define ICM_ACCEL_CONFIG_DLPF_6HZ      (0x30U)
#define ICM_ACCEL_CONFIG_DLPF_473HZ    (0x38U)

/* Auxiliary I2C Master Register Fields */
#define ICM_I2C_MST_CLK_345KHZ          (0x07U) /* I2C Master clock speed ~345 kHz */
#define ICM_I2C_SLV_READ                (0x80U) /* Read operation flag for auxiliary slave */
#define ICM_I2C_SLV_ENABLE              (0x80U) /* Enable corresponding I2C slave */
#define ICM_I2C_SLV_LENGTH_MASK         (0x0FU) /* Data length mask (Bits [3:0]) */
#define ICM_I2C_MST_STATUS_SLV0_NACK    (0x01U) /* Slave 0 NACK error flag */

/* AK09916 Magnetometer Registers */
#define AK09916_I2C_ADDRESS             (0x0CU) /* Magnetometer hardware I2C address */
#define AK09916_REG_WIA2                (0x01U) /* Magnetometer Device ID register */
#define AK09916_DEVICE_ID               (0x09U) /* Standard AK09916 ID value (0x09) */
#define AK09916_REG_ST1                 (0x10U) /* Status register 1 (Data Ready) */
#define AK09916_REG_CNTL2               (0x31U) /* Control mode register */
#define AK09916_MODE_CONTINUOUS_100HZ   (0x08U) /* Continuous measurement mode at 100 Hz */
#define AK09916_ST1_DRDY                (0x01U) /* Bit 0: Magnetometer data ready */
#define AK09916_ST1_DOR                 (0x02U) /* Bit 1: Data Overrun error */
#define AK09916_ST2_HOFL                (0x08U) /* Bit 3: Sensor Overflows error */

#endif /* ICM_20948_I2C_H */

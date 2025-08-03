
#ifndef __STM32_MAX6650_H
#define __STM32_MAX6650_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Fan control modes
#define CFF_ON   0x01  // control_fan_full_on
#define CFF_OFF  0x02  // control_fan_full_off
#define CFF_AUTO 0x03  // control_fan_max6650_automatic_mode

/**
 * @brief MAX6650 data structure
 */
typedef struct {
    uint16_t fan_rpm;
    uint8_t status_flag;
} max6650_data_t;

/**
 * @brief MAX6650 ADD line connection options
 */
typedef enum {
    MAX6650_ADD_LINE_GND = 0,
    MAX6650_ADD_LINE_VCC,
    MAX6650_ADD_LINE_NOT_CONNECTED,
    MAX6650_ADD_LINE_RES10K
} max6650_add_line_t;

/**
 * @brief MAX6650 Operating Mode
 */
typedef enum {
    MAX6650_MODE_SOFTWARE_FULL_ON = 0,
    MAX6650_MODE_SOFTWARE_OFF,
    MAX6650_MODE_CLOSED_LOOP,
    MAX6650_MODE_OPEN_LOOP
} max6650_operating_mode_t;

/**
 * @brief MAX6650 Fan Voltage
 */
typedef enum {
    MAX6650_FAN_VOLTAGE_5V = 0,
    MAX6650_FAN_VOLTAGE_12V
} max6650_fan_voltage_t;

/**
 * @brief MAX6650 K-scale prescaler
 */
typedef enum {
    MAX6650_KSCALE_1 = 0,
    MAX6650_KSCALE_2,
    MAX6650_KSCALE_4,
    MAX6650_KSCALE_8,
    MAX6650_KSCALE_16
} max6650_kscale_t;

/**
 * @brief MAX6650 Configuration structure
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    max6650_add_line_t add_line_connection;
    max6650_operating_mode_t operating_mode;
    max6650_fan_voltage_t fan_voltage;
    max6650_kscale_t k_scale;
    uint16_t rpm_max;
    uint32_t i2c_timeout;
} max6650_config_t;

/**
 * @brief MAX6650 handle structure
 */
typedef struct {
    max6650_config_t config;
    uint8_t i2c_address;
    bool initialized;
} max6650_handle_t;

// Function prototypes

/**
 * @brief Initialize MAX6650 device
 * @param handle Pointer to MAX6650 handle
 * @param config Pointer to configuration structure
 * @retval true if initialization successful, false otherwise
 */
bool MAX6650_Init(max6650_handle_t *handle, max6650_config_t *config);

/**
 * @brief Read fan data (RPM and status)
 * @param handle Pointer to MAX6650 handle
 * @param data Pointer to data structure to fill
 * @retval true if read successful, false otherwise
 */
bool MAX6650_ReadData(max6650_handle_t *handle, max6650_data_t *data);

/**
 * @brief Control fan based on control flag
 * @param handle Pointer to MAX6650 handle
 * @param control_flag Control flag (CFF_ON, CFF_OFF, CFF_AUTO)
 * @retval true if control successful, false otherwise
 */
bool MAX6650_ControlFan(max6650_handle_t *handle, uint8_t control_flag);

/**
 * @brief Set fan speed percentage (0-100%)
 * @param handle Pointer to MAX6650 handle
 * @param speed_percent Speed percentage (0-100)
 * @retval true if setting successful, false otherwise
 */
bool MAX6650_SetSpeedPercent(max6650_handle_t *handle, uint8_t speed_percent);

/**
 * @brief Get current fan speed percentage
 * @param handle Pointer to MAX6650 handle
 * @param speed_percent Pointer to store speed percentage
 * @retval true if read successful, false otherwise
 */
bool MAX6650_GetSpeedPercent(max6650_handle_t *handle, uint8_t *speed_percent);

/**
 * @brief Read alarm status register
 * @param handle Pointer to MAX6650 handle
 * @param status Pointer to store status flags
 * @retval true if read successful, false otherwise
 */
bool MAX6650_ReadAlarmStatus(max6650_handle_t *handle, uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_MAX6650_H */

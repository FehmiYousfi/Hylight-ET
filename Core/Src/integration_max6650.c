
#include "integration_max6650.h"

// MAX6650 Register addresses
#define MAX6650_SPEED_REG           0x00  // Fan speed R/W
#define MAX6650_CONFIG_REG          0x02  // Configuration R/W
#define MAX6650_GPIODEF_REG         0x04  // GPIO definition R/W
#define MAX6650_DAC_REG             0x06  // DAC R/W
#define MAX6650_ALARMENABLE_REG     0x08  // Alarm enable R/W
#define MAX6650_ALARM_REG           0x0A  // Alarm status R
#define MAX6650_TACHO_0_REG         0x0C  // Tachometer 0 count R
#define MAX6650_GPIOSTAT_REG        0x14  // GPIO status R
#define MAX6650_COUNT_REG           0x16  // Tachometer count time R/W

// I2C addresses based on ADD pin connection
#define MAX6650_I2C_ADDR_GND        0x48  // ADD pin to GND (0x90 >> 1)
#define MAX6650_I2C_ADDR_VCC        0x4B  // ADD pin to VCC (0x96 >> 1)
#define MAX6650_I2C_ADDR_NC         0x1B  // ADD pin not connected (0x36 >> 1)
#define MAX6650_I2C_ADDR_RES10K     0x1F  // ADD pin with 10k resistor (0x3E >> 1)

// Configuration register bit masks
#define MAX6650_CONFIG_MODE_MASK    0x30
#define MAX6650_CONFIG_MODE_SHIFT   4
#define MAX6650_CONFIG_VOLTAGE_MASK 0x08
#define MAX6650_CONFIG_VOLTAGE_SHIFT 3
#define MAX6650_CONFIG_KSCALE_MASK  0x07

// Default values
#define MAX6650_DEFAULT_COUNT_TIME  2
#define MAX6650_INTERNAL_CLOCK_KHZ  254
#define MAX6650_DEFAULT_TIMEOUT     500

// Private function prototypes
static uint8_t get_i2c_address(max6650_add_line_t add_line);
static uint8_t get_kscale_value(max6650_kscale_t k_scale);
static bool max6650_write_register(max6650_handle_t *handle, uint8_t reg, uint8_t data);
static bool max6650_read_register(max6650_handle_t *handle, uint8_t reg, uint8_t *data);
static uint16_t calculate_rpm_from_tacho(uint8_t tacho_count, max6650_kscale_t k_scale);

bool MAX6650_Init(max6650_handle_t *handle, max6650_config_t *config)
{
    if (handle == NULL || config == NULL || config->hi2c == NULL) {
        return false;
    }

    // Copy configuration
    handle->config = *config;
    handle->i2c_address = get_i2c_address(config->add_line_connection);
    handle->initialized = false;

    // Set default timeout if not specified
    if (handle->config.i2c_timeout == 0) {
        handle->config.i2c_timeout = MAX6650_DEFAULT_TIMEOUT;
    }

    // Configure the MAX6650
    uint8_t config_byte = 0;
    config_byte |= (config->operating_mode & 0x03) << MAX6650_CONFIG_MODE_SHIFT;
    config_byte |= (config->fan_voltage & 0x01) << MAX6650_CONFIG_VOLTAGE_SHIFT;
    config_byte |= (config->k_scale & 0x07);

    // Write configuration register
    if (!max6650_write_register(handle, MAX6650_CONFIG_REG, config_byte)) {
        return false;
    }

    // Set count time register
    if (!max6650_write_register(handle, MAX6650_COUNT_REG, MAX6650_DEFAULT_COUNT_TIME)) {
        return false;
    }

    handle->initialized = true;
    return true;
}

bool MAX6650_ReadData(max6650_handle_t *handle, max6650_data_t *data)
{
    if (handle == NULL || data == NULL || !handle->initialized) {
        return false;
    }

    uint8_t tacho_count = 0;
    uint8_t alarm_status = 0;

    // Read tachometer count
    if (!max6650_read_register(handle, MAX6650_TACHO_0_REG, &tacho_count)) {
        return false;
    }

    // Read alarm status
    if (!max6650_read_register(handle, MAX6650_ALARM_REG, &alarm_status)) {
        return false;
    }

    // Calculate RPM from tachometer count
    data->fan_rpm = calculate_rpm_from_tacho(tacho_count, handle->config.k_scale);
    data->status_flag = alarm_status;

    return true;
}

bool MAX6650_ControlFan(max6650_handle_t *handle, uint8_t control_flag)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }

    uint8_t config_byte = 0;
    max6650_operating_mode_t new_mode;

    switch (control_flag) {
        case CFF_ON:
            new_mode = MAX6650_MODE_SOFTWARE_FULL_ON;
            break;
        case CFF_OFF:
            new_mode = MAX6650_MODE_SOFTWARE_OFF;
            break;
        case CFF_AUTO:
            new_mode = MAX6650_MODE_CLOSED_LOOP;
            break;
        default:
            return false;
    }

    // Read current configuration
    if (!max6650_read_register(handle, MAX6650_CONFIG_REG, &config_byte)) {
        return false;
    }

    // Update mode bits
    config_byte &= ~MAX6650_CONFIG_MODE_MASK;
    config_byte |= (new_mode & 0x03) << MAX6650_CONFIG_MODE_SHIFT;

    // Write updated configuration
    if (!max6650_write_register(handle, MAX6650_CONFIG_REG, config_byte)) {
        return false;
    }

    // Update handle configuration
    handle->config.operating_mode = new_mode;
    return true;
}

bool MAX6650_SetSpeedPercent(max6650_handle_t *handle, uint8_t speed_percent)
{
    if (handle == NULL || !handle->initialized || speed_percent > 100) {
        return false;
    }

    // Calculate target RPM
    uint16_t target_rpm = (handle->config.rpm_max * speed_percent) / 100;

    if (target_rpm == 0) {
        // Set fan off
        return MAX6650_ControlFan(handle, CFF_OFF);
    }

    // Calculate KTACH value based on datasheet formula
    // KTACH = (992 * KSCALE / target_rps) - 1
    uint16_t target_rps = target_rpm / 60;
    if (target_rps == 0) target_rps = 1; // Avoid division by zero

    uint8_t kscale_value = get_kscale_value(handle->config.k_scale);
    uint8_t ktach = ((992 * kscale_value) / target_rps) - 1;

    // Write speed register
    return max6650_write_register(handle, MAX6650_SPEED_REG, ktach);
}

bool MAX6650_GetSpeedPercent(max6650_handle_t *handle, uint8_t *speed_percent)
{
    if (handle == NULL || speed_percent == NULL || !handle->initialized) {
        return false;
    }

    max6650_data_t data;
    if (!MAX6650_ReadData(handle, &data)) {
        return false;
    }

    // Calculate percentage based on max RPM
    if (handle->config.rpm_max > 0) {
        *speed_percent = (data.fan_rpm * 100) / handle->config.rpm_max;
        if (*speed_percent > 100) *speed_percent = 100;
    } else {
        *speed_percent = 0;
    }

    return true;
}

bool MAX6650_ReadAlarmStatus(max6650_handle_t *handle, uint8_t *status)
{
    if (handle == NULL || status == NULL || !handle->initialized) {
        return false;
    }

    return max6650_read_register(handle, MAX6650_ALARM_REG, status);
}

// Private function implementations

static uint8_t get_i2c_address(max6650_add_line_t add_line)
{
    switch (add_line) {
        case MAX6650_ADD_LINE_GND:
            return MAX6650_I2C_ADDR_GND;
        case MAX6650_ADD_LINE_VCC:
            return MAX6650_I2C_ADDR_VCC;
        case MAX6650_ADD_LINE_NOT_CONNECTED:
            return MAX6650_I2C_ADDR_NC;
        case MAX6650_ADD_LINE_RES10K:
            return MAX6650_I2C_ADDR_RES10K;
        default:
            return MAX6650_I2C_ADDR_GND;
    }
}

static uint8_t get_kscale_value(max6650_kscale_t k_scale)
{
    switch (k_scale) {
        case MAX6650_KSCALE_1:  return 1;
        case MAX6650_KSCALE_2:  return 2;
        case MAX6650_KSCALE_4:  return 4;
        case MAX6650_KSCALE_8:  return 8;
        case MAX6650_KSCALE_16: return 16;
        default:                return 4;
    }
}

static bool max6650_write_register(max6650_handle_t *handle, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(handle->config.hi2c, 
                               handle->i2c_address << 1, 
                               reg, 
                               I2C_MEMADD_SIZE_8BIT, 
                               &data, 
                               1, 
                               handle->config.i2c_timeout);
    return (status == HAL_OK);
}

static bool max6650_read_register(max6650_handle_t *handle, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(handle->config.hi2c, 
                              handle->i2c_address << 1, 
                              reg, 
                              I2C_MEMADD_SIZE_8BIT, 
                              data, 
                              1, 
                              handle->config.i2c_timeout);
    return (status == HAL_OK);
}

static uint16_t calculate_rpm_from_tacho(uint8_t tacho_count, max6650_kscale_t k_scale)
{
    if (tacho_count == 0) {
        return 0;
    }

    // Based on datasheet formula: FanSpeed = (tacho / 2) x count_t
    // Then multiply by 60 to convert from RPS to RPM
    uint16_t rps = (tacho_count / 2) / MAX6650_DEFAULT_COUNT_TIME;
    uint16_t rpm = rps * 60;

    return rpm;
}

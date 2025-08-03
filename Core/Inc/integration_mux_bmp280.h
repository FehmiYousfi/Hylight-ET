
/**
 * Multi-sensor BMP280 system with TCA9548A I2C multiplexer
 * For STM32 microcontrollers using HAL library
 */

#ifndef INTEGRATION_MUX_BMP280_H
#define INTEGRATION_MUX_BMP280_H

#include "stm32l4xx_hal.h"
#include "bmp280.h"
#include "i2c-mux.h"
#include <stdint.h>
#include <stdbool.h>

#define NUM_BMP280_SENSORS 6

typedef struct {
    int16_t delta_pres_0; // difference between sensor1 and sensor2
    int16_t delta_pres_1; // difference between sensor3 and sensor4
    int16_t delta_pres_2; // difference between sensor5 and sensor6
} bmp280_sensors_data_t;

typedef struct {
    i2c_mux_t mux;
    BMP280_HandleTypedef sensors[NUM_BMP280_SENSORS];
    bmp280_params_t sensor_params;
    uint8_t sensor_channels[NUM_BMP280_SENSORS]; // TCA9548A channel for each sensor
    uint32_t last_pressures[NUM_BMP280_SENSORS]; // Store last pressure readings (Pa * 256)
    bool sensors_initialized[NUM_BMP280_SENSORS];
} bmp280_multi_system_t;

/**
 * Initialize the multi-sensor BMP280 system
 * @param system: Pointer to the multi-sensor system structure
 * @param hi2c: I2C handle for communication
 * @param rst_port: GPIO port for TCA9548A reset pin (can be NULL)
 * @param rst_pin: GPIO pin for TCA9548A reset (can be 0)
 * @param mux_addr_offset: Address offset for TCA9548A (0-7)
 * @param sensor_addresses: Array of 6 BMP280 I2C addresses
 * @param sensor_channels: Array of 6 TCA9548A channel numbers (0-7)
 * @return: true on success, false on failure
 */
bool bmp280_multi_init(bmp280_multi_system_t *system, 
                       I2C_HandleTypeDef *hi2c,
                       GPIO_TypeDef *rst_port,
                       uint16_t rst_pin,
                       uint8_t mux_addr_offset,
                       uint8_t sensor_addresses[NUM_BMP280_SENSORS],
                       uint8_t sensor_channels[NUM_BMP280_SENSORS]);

/**
 * Read all sensors and calculate pressure differences
 * @param system: Pointer to the multi-sensor system structure
 * @param data: Pointer to store the calculated differences
 * @return: true on success, false on failure
 */
bool bmp280_multi_read_differences(bmp280_multi_system_t *system, 
                                   bmp280_sensors_data_t *data);

/**
 * Read individual sensor pressure
 * @param system: Pointer to the multi-sensor system structure
 * @param sensor_index: Index of the sensor (0-5)
 * @param pressure: Pointer to store pressure reading (Pa)
 * @return: true on success, false on failure
 */
bool bmp280_multi_read_sensor(bmp280_multi_system_t *system, 
                              uint8_t sensor_index, 
                              float *pressure);

/**
 * Force measurement on all sensors
 * @param system: Pointer to the multi-sensor system structure
 * @return: true on success, false on failure
 */
bool bmp280_multi_force_measurement(bmp280_multi_system_t *system);

/**
 * Check if any sensor is currently measuring
 * @param system: Pointer to the multi-sensor system structure
 * @return: true if any sensor is measuring, false otherwise
 */
bool bmp280_multi_is_measuring(bmp280_multi_system_t *system);

#endif // INTEGRATION_MUX_BMP280_H


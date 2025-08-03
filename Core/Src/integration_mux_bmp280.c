
/**
 * Multi-sensor BMP280 system with TCA9548A I2C multiplexer
 * For STM32 microcontrollers using HAL library
 */

#include "integration_mux_bmp280.h"

// Conversion factor from fixed-point pressure to Pascals
#define PRESSURE_FIXED_TO_FLOAT(x) ((float)(x) / 256.0f)

// Conversion factor from Pascals to fixed-point for storage
#define PRESSURE_FLOAT_TO_FIXED(x) ((uint32_t)((x) * 256.0f))

bool bmp280_multi_init(bmp280_multi_system_t *system, 
                       I2C_HandleTypeDef *hi2c,
                       GPIO_TypeDef *rst_port,
                       uint16_t rst_pin,
                       uint8_t mux_addr_offset,
                       uint8_t sensor_addresses[NUM_BMP280_SENSORS],
                       uint8_t sensor_channels[NUM_BMP280_SENSORS])
{
    if (system == NULL || hi2c == NULL || sensor_addresses == NULL || sensor_channels == NULL) {
        return false;
    }

    // Initialize multiplexer
    system->mux.hi2c = hi2c;
    system->mux.rst_port = rst_port;
    system->mux.rst_pin = rst_pin;
    system->mux.addr_offset = mux_addr_offset;

    // Reset multiplexer if reset pin is provided
    if (rst_port != NULL && rst_pin != 0) {
        if (i2c_mux_reset(&system->mux) != 0) {
            return false;
        }
    }

    // Initialize default parameters for all sensors
    bmp280_init_default_params(&system->sensor_params);
    system->sensor_params.mode = BMP280_MODE_FORCED; // Use forced mode for better control

    // Initialize all sensors
    for (uint8_t i = 0; i < NUM_BMP280_SENSORS; i++) {
        // Store channel mapping
        system->sensor_channels[i] = sensor_channels[i];
        
        // Initialize sensor structure
        system->sensors[i].addr = sensor_addresses[i];
        system->sensors[i].i2c = hi2c;
        
        // Select multiplexer channel
        if (i2c_mux_select(&system->mux, sensor_channels[i]) != 0) {
            system->sensors_initialized[i] = false;
            continue;
        }

        // Initialize BMP280 sensor
        if (bmp280_init(&system->sensors[i], &system->sensor_params)) {
            system->sensors_initialized[i] = true;
            system->last_pressures[i] = 0;
        } else {
            system->sensors_initialized[i] = false;
        }

        // Small delay between sensor initializations
        HAL_Delay(10);
    }

    // Disable all multiplexer channels
    i2c_mux_select(&system->mux, 8); // Invalid channel disables all

    return true;
}

bool bmp280_multi_read_sensor(bmp280_multi_system_t *system, 
                              uint8_t sensor_index, 
                              float *pressure)
{
    if (system == NULL || pressure == NULL || sensor_index >= NUM_BMP280_SENSORS) {
        return false;
    }

    if (!system->sensors_initialized[sensor_index]) {
        return false;
    }

    // Select multiplexer channel
    if (i2c_mux_select(&system->mux, system->sensor_channels[sensor_index]) != 0) {
        return false;
    }

    // Read sensor data
    int32_t temperature;
    uint32_t pressure_fixed;
    
    if (!bmp280_read_fixed(&system->sensors[sensor_index], &temperature, &pressure_fixed, NULL)) {
        return false;
    }

    // Convert to float pressure in Pascals
    *pressure = PRESSURE_FIXED_TO_FLOAT(pressure_fixed);
    
    // Store for later use
    system->last_pressures[sensor_index] = pressure_fixed;

    return true;
}

bool bmp280_multi_force_measurement(bmp280_multi_system_t *system)
{
    if (system == NULL) {
        return false;
    }

    bool success = true;

    for (uint8_t i = 0; i < NUM_BMP280_SENSORS; i++) {
        if (!system->sensors_initialized[i]) {
            continue;
        }

        // Select multiplexer channel
        if (i2c_mux_select(&system->mux, system->sensor_channels[i]) != 0) {
            success = false;
            continue;
        }

        // Force measurement
        if (!bmp280_force_measurement(&system->sensors[i])) {
            success = false;
        }

        // Small delay between operations
        HAL_Delay(1);
    }

    // Disable all multiplexer channels
    i2c_mux_select(&system->mux, 8);

    return success;
}

bool bmp280_multi_is_measuring(bmp280_multi_system_t *system)
{
    if (system == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < NUM_BMP280_SENSORS; i++) {
        if (!system->sensors_initialized[i]) {
            continue;
        }

        // Select multiplexer channel
        if (i2c_mux_select(&system->mux, system->sensor_channels[i]) != 0) {
            continue;
        }

        // Check if measuring
        if (bmp280_is_measuring(&system->sensors[i])) {
            return true;
        }
    }

    return false;
}

bool bmp280_multi_read_differences(bmp280_multi_system_t *system, 
                                   bmp280_sensors_data_t *data)
{
    if (system == NULL || data == NULL) {
        return false;
    }

    float pressures[NUM_BMP280_SENSORS];
    uint8_t valid_readings = 0;

    // Read all sensors
    for (uint8_t i = 0; i < NUM_BMP280_SENSORS; i++) {
        if (bmp280_multi_read_sensor(system, i, &pressures[i])) {
            valid_readings |= (1 << i);
        } else {
            pressures[i] = 0.0f;
        }
    }

    // Calculate differences between sensor pairs
    // Only calculate if both sensors in the pair are valid
    
    // delta_pres_0: difference between sensor1 (index 0) and sensor2 (index 1)
    if ((valid_readings & 0x03) == 0x03) { // Both sensor 0 and 1 are valid
        float diff = pressures[0] - pressures[1];
        // Convert to int16_t (Pa), clamp to prevent overflow
        if (diff > 32767.0f) diff = 32767.0f;
        if (diff < -32768.0f) diff = -32768.0f;
        data->delta_pres_0 = (int16_t)diff;
    } else {
        data->delta_pres_0 = 0;
    }

    // delta_pres_1: difference between sensor3 (index 2) and sensor4 (index 3)
    if ((valid_readings & 0x0C) == 0x0C) { // Both sensor 2 and 3 are valid
        float diff = pressures[2] - pressures[3];
        if (diff > 32767.0f) diff = 32767.0f;
        if (diff < -32768.0f) diff = -32768.0f;
        data->delta_pres_1 = (int16_t)diff;
    } else {
        data->delta_pres_1 = 0;
    }

    // delta_pres_2: difference between sensor5 (index 4) and sensor6 (index 5)
    if ((valid_readings & 0x30) == 0x30) { // Both sensor 4 and 5 are valid
        float diff = pressures[4] - pressures[5];
        if (diff > 32767.0f) diff = 32767.0f;
        if (diff < -32768.0f) diff = -32768.0f;
        data->delta_pres_2 = (int16_t)diff;
    } else {
        data->delta_pres_2 = 0;
    }

    // Disable all multiplexer channels
    i2c_mux_select(&system->mux, 8);

    // Return true if at least one pair was successfully read
    return ((valid_readings & 0x03) == 0x03) || 
           ((valid_readings & 0x0C) == 0x0C) || 
           ((valid_readings & 0x30) == 0x30);
}


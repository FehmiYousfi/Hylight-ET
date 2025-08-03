
/**
 * @file thread_proto.h
 * @brief Thread Protocol Header for Embedded Data Collection System
 *
 * This header defines the interface for a multi-threaded data collection system
 * that integrates:
 * - BMP280 pressure sensors via I2C multiplexer
 * - MAX6650 fan controller via I2C
 * - CAN bus data transmission
 * - UART data logging with COBS encoding
 * - UART command reception and processing
 *
 * The system is designed for STM32 microcontrollers using FreeRTOS and HAL drivers.
 */

#ifndef THREAD_PROTO_H
#define THREAD_PROTO_H

/* ============================================================================
 * SYSTEM INCLUDES
 * ============================================================================ */

#include "main.h"                    // STM32 HAL main header
#include "cmsis_os.h"               // FreeRTOS CMSIS-OS interface
#include <cobs_crc16.h>             // COBS encoding and CRC16 functions
#include <integration_max6650.h>     // MAX6650 fan controller driver
#include <integration_mux_bmp280.h>  // BMP280 multi-sensor system driver
#include <stdbool.h>                // Boolean type definitions

/* ============================================================================
 * GLOBAL HARDWARE INTERFACE HANDLES
 * ============================================================================ */

/** @brief I2C interface handle for sensor multiplexer communication */
extern I2C_HandleTypeDef *g_mux_iface;

/** @brief I2C interface handle for fan driver communication */
extern I2C_HandleTypeDef *g_driver_iface;

/** @brief UART handle for data logging and command reception */
extern UART_HandleTypeDef *g_data_logger;

/** @brief UART handle for local monitoring and debug output */
extern UART_HandleTypeDef *g_local_monitor;

/** @brief CAN bus handle for sensor data transmission */
extern CAN_HandleTypeDef *g_can_iface;

/** @brief DMA handle for UART reception operations */
extern DMA_HandleTypeDef *g_logger_dma;

/* ============================================================================
 * GPIO AND VERIFICATION VARIABLES
 * ============================================================================ */

/** @brief Flag indicating CAN GPIO verification status */
extern bool can_gpio_verification;

/** @brief GPIO port for CAN loader functionality */
extern GPIO_TypeDef *can_loader;

/** @brief GPIO pin for CAN loader functionality */
extern uint16_t *loader_gpio;

/* ============================================================================
 * SHARED DATA STRUCTURES
 * ============================================================================ */

/** @brief Pointer to fan controller data (RPM, status, etc.) */
extern max6650_data_t *g_driver_data;

/** @brief Pointer to pressure sensor data (differential pressures) */
extern bmp280_sensors_data_t *g_sensor_data;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Initialize the thread system with hardware handles
 *
 * This function assigns all necessary hardware peripheral handles to global
 * variables for use by the various thread cycles. Must be called before
 * starting any threads.
 *
 * @param data_logger_place_holder UART handle for data logging output
 * @param local_monitor_place_holder UART handle for local monitoring/debug
 * @param driver_iface_placeholder I2C handle for MAX6650 fan controller
 * @param mux_iface_placeholder I2C handle for BMP280 sensor multiplexer
 * @param logger_dma_placeholder DMA handle for UART reception
 * @param target_placeholder UART instance for target identification
 * @param can_placeholder CAN handle for data transmission
 */
void perform_setup_threads(UART_HandleTypeDef *data_logger_place_holder,
                          UART_HandleTypeDef *local_monitor_place_holder,
                          I2C_HandleTypeDef *driver_iface_placeholder,
                          I2C_HandleTypeDef *mux_iface_placeholder,
                          DMA_HandleTypeDef *logger_dma_placeholder,
                          USART_TypeDef *target_placeholder,
                          CAN_HandleTypeDef *can_placeholder);

/**
 * @brief UART data logger thread function
 *
 * Continuously collects sensor and fan data, creates UART packets with CRC,
 * encodes using COBS algorithm, verifies encoding integrity, and transmits
 * via UART every 500ms.
 *
 * @param argument Thread argument (unused)
 */
void uart_logger_cycle(void const *argument);

/**
 * @brief CAN data logger thread function
 *
 * Continuously packages sensor data (3x pressure + 1x fan RPM) into CAN
 * messages and transmits every 5ms. Uses different CAN IDs based on GPIO
 * verification status.
 *
 * @param argument Thread argument (unused)
 */
void can_logger_cycle(void const *argument);

/**
 * @brief Sensor and fan controller management thread function
 *
 * Handles initialization and operation of:
 * - MAX6650 fan controller (I2C)
 * - BMP280 multi-sensor system (I2C via multiplexer)
 * - Periodic pressure measurements (100ms cycle)
 * - Fan control command processing
 * - Data storage in global structures
 *
 * @param argument Thread argument (unused)
 */
void mux_pres_cycle(void const *argument);

/**
 * @brief UART command reception and processing thread function
 *
 * Monitors for incoming UART commands via DMA, decodes COBS-encoded packets,
 * verifies CRC integrity, and executes valid fan control commands.
 *
 * @param argument Thread argument (unused)
 */
void uart_notifications_cycle(void const *argument);

/* ============================================================================
 * LOGGING AND MESSAGING MACROS
 * ============================================================================ */

/**
 * @brief Simple message logging via UART data logger
 *
 * Transmits a simple message through the data logger UART interface.
 * Note: This macro has a limitation - it uses sizeof() which may not work
 * correctly with string literals.
 *
 * @param msg Message to transmit
 */
#define LOG_MESSAGE(msg) \
    HAL_UART_Transmit(g_data_logger, (uint8_t*)&(msg), sizeof(msg), 100)

/**
 * @brief Encoded data logging via UART data logger
 *
 * Transmits encoded data with specified size through the data logger UART.
 * Used for COBS-encoded data packets.
 *
 * @param msg Encoded data buffer to transmit
 * @param size_ Size of the encoded data in bytes
 */
#define LOG_DATA_ENCODED(msg,size_) \
    HAL_UART_Transmit(g_data_logger, (uint8_t*)&(msg), size_, 100)

/* Include required headers for printf-style formatting */
#include <stdarg.h>  // For va_list support
#include <stdio.h>   // For vsnprintf

/**
 * @brief Formatted notification message via local monitor UART
 *
 * Creates formatted messages with "[notif] : " prefix and transmits through
 * the local monitor UART interface. Supports printf-style formatting.
 *
 * Usage: NOTIFY_MESSAGE("Sensor value: %d", sensor_reading);
 *
 * @param fmt Format string (printf-style)
 * @param ... Variable arguments for formatting
 */
#define NOTIFY_MESSAGE(fmt, ...) \
    do { \
        char _buffer[128]; \
        int _prefix_len = snprintf(_buffer, sizeof(_buffer), "[notif] : "); \
        int _msg_len = snprintf(_buffer + _prefix_len, sizeof(_buffer) - _prefix_len, fmt "\n", ##__VA_ARGS__); \
        HAL_UART_Transmit(g_local_monitor, (uint8_t*)_buffer, _prefix_len + _msg_len, 100); \
    } while(0)

#endif /* THREAD_PROTO_H */

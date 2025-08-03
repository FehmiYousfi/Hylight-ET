#ifndef THREAD_PROTO_H
#define THREAD_PROTO_H

#include "main.h"
#include "cmsis_os.h"
#include <cobs_crc16.h>
#include <integration_max6650.h>
#include <integration_mux_bmp280.h>
#include <stdbool.h>

extern I2C_HandleTypeDef *g_mux_iface;
extern I2C_HandleTypeDef *g_driver_iface;
extern UART_HandleTypeDef *g_data_logger;
extern UART_HandleTypeDef *g_local_monitor;
extern CAN_HandleTypeDef *g_can_iface;
extern DMA_HandleTypeDef *g_logger_dma;
extern bool can_gpio_verification;
extern GPIO_TypeDef *can_loader;
extern uint16_t *loader_gpio;

extern max6650_data_t *g_driver_data;
extern bmp280_sensors_data_t * g_sensor_data;

void perform_setup_threads(UART_HandleTypeDef *data_logger_place_holder,
			UART_HandleTypeDef *local_monitor_place_holder,
			I2C_HandleTypeDef *driver_iface_placeholder,
			I2C_HandleTypeDef *mux_iface_placeholder,
			DMA_HandleTypeDef *logger_dma_placeholder,
			USART_TypeDef * target_placeholder,
			CAN_HandleTypeDef * can_placeholder);

void uart_logger_cycle(void const *argument);
void can_logger_cycle(void const *argument);
void mux_pres_cycle(void const *argument);
void uart_notifications_cycle(void const *argument);


#define LOG_MESSAGE(msg) \
    HAL_UART_Transmit(g_data_logger, (uint8_t*)&(msg), sizeof(msg), 100)
#define LOG_DATA_ENCODED(msg,size_) \
    HAL_UART_Transmit(g_data_logger, (uint8_t*)&(msg), size_, 100)

#include <stdarg.h>  // For va_list support
#include <stdio.h>   // For vsnprintf

#define NOTIFY_MESSAGE(fmt, ...) \
    do { \
        char _buffer[128]; \
        int _prefix_len = snprintf(_buffer, sizeof(_buffer), "[notif] : "); \
        int _msg_len = snprintf(_buffer + _prefix_len, sizeof(_buffer) - _prefix_len, fmt "\n", ##__VA_ARGS__); \
        HAL_UART_Transmit(g_local_monitor, (uint8_t*)_buffer, _prefix_len + _msg_len, 100); \
    } while(0)

#endif


/**
 * @file thread_proto.c
 * @brief Thread Protocol Implementation for Embedded Data Collection System
 *
 * This file implements the main threading logic for a data collection system that:
 * - Collects sensor data from BMP280 pressure sensors via I2C multiplexer
 * - Controls MAX6650 fan controller via I2C
 * - Transmits data via CAN bus and UART with COBS encoding
 * - Handles incoming UART commands for fan control
 *
 * The system uses FreeRTOS threading with STM32 HAL drivers.
 */

#include "thread_proto.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * GLOBAL HARDWARE INTERFACE HANDLES
 * ============================================================================ */

/** @brief UART handle for data logging output */
UART_HandleTypeDef *g_data_logger = NULL;

/** @brief UART handle for local monitoring/debug output */
UART_HandleTypeDef *g_local_monitor = NULL;

/** @brief I2C handle for fan driver (MAX6650) communication */
I2C_HandleTypeDef *g_driver_iface = NULL;

/** @brief I2C handle for sensor multiplexer (BMP280 sensors) communication */
I2C_HandleTypeDef *g_mux_iface = NULL;

/** @brief DMA handle for UART reception */
DMA_HandleTypeDef *g_logger_dma = NULL;

/** @brief CAN bus handle for data transmission */
CAN_HandleTypeDef *g_can_iface = NULL;

/** @brief Current UART target instance for reception */
USART_TypeDef *Current_Target = NULL;

/* ============================================================================
 * GPIO AND VERIFICATION VARIABLES
 * ============================================================================ */

/** @brief GPIO port for CAN loader functionality */
GPIO_TypeDef *can_loader = NULL;

/** @brief GPIO pin for CAN loader functionality */
uint16_t *loader_gpio = NULL;

/** @brief Flag indicating CAN GPIO verification status */
bool can_gpio_verification;

/** @brief Global flag for fan control actions (0=none, 1=on, 2=off, 3=auto) */
int global_action_flag = 0;

/* ============================================================================
 * CAN MESSAGE IDENTIFIERS
 * ============================================================================ */

/** @brief CAN message ID when GPIO is triggered */
#define CAN_GPIO_TRIGGERED 0x008

/** @brief CAN message ID when GPIO is not triggered */
#define CAN_GPIO_NON_TRIGGERED 0x009

/* ============================================================================
 * DATA STRUCTURE POINTERS
 * ============================================================================ */

/** @brief Pointer to fan controller data structure */
max6650_data_t *g_driver_data = NULL;

/** @brief Pointer to pressure sensor data structure */
bmp280_sensors_data_t *g_sensor_data = NULL;

/* ============================================================================
 * UART RECEPTION BUFFERS
 * ============================================================================ */

/** @brief Size of reception buffer */
#define rx_buff_size 10

/** @brief Size of main processing buffer */
#define main_buff_size 20

/** @brief DMA reception buffer for incoming UART data */
uint8_t rx_buff[rx_buff_size];

/** @brief Main buffer for processing received UART data */
uint8_t main_buff[main_buff_size];

/** @brief Flag indicating new data received and ready for validation */
int rx_validation = 0;

/* ============================================================================
 * CAN TRANSMISSION VARIABLES
 * ============================================================================ */

/** @brief CAN transmission header configuration */
CAN_TxHeaderTypeDef TxHeader;

/** @brief CAN transmission mailbox identifier */
uint32_t TxMailbox;

/** @brief CAN data payload buffer (8 bytes max) */
uint8_t TxData[8];

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Split a 16-bit unsigned integer into high and low bytes
 * @param value The 16-bit value to split
 * @param high_byte Pointer to store the high byte (bits 15-8)
 * @param low_byte Pointer to store the low byte (bits 7-0)
 */
void split_uint16(uint16_t value, uint8_t *high_byte, uint8_t *low_byte) {
    *high_byte = (value >> 8) & 0xFF;  // Extract upper 8 bits
    *low_byte  = value & 0xFF;         // Extract lower 8 bits
}

/**
 * @brief Split a 16-bit signed integer into high and low bytes
 * @param value The 16-bit signed value to split
 * @param high_byte Pointer to store the high byte (bits 15-8)
 * @param low_byte Pointer to store the low byte (bits 7-0)
 */
void split_int16(int16_t value, uint8_t *high_byte, uint8_t *low_byte) {
    *high_byte = (value >> 8) & 0xFF;  // Extract upper 8 bits
    *low_byte  = value & 0xFF;         // Extract lower 8 bits
}

/* ============================================================================
 * THREAD CYCLE FUNCTIONS
 * ============================================================================ */

/**
 * @brief CAN logger thread cycle - transmits sensor data via CAN bus
 * @param argument Thread argument (unused)
 *
 * This thread continuously:
 * 1. Sets CAN message ID based on GPIO verification status
 * 2. Packs pressure sensor data (3 x 16-bit values) into CAN payload
 * 3. Packs fan RPM data (1 x 16-bit value) into CAN payload
 * 4. Transmits CAN message every 5ms
 * 5. Uses mock data if real sensors are not available
 */
void can_logger_cycle(void const *argument){
  // Configure CAN message ID based on GPIO status
  if (can_gpio_verification){
    TxHeader.StdId = CAN_GPIO_TRIGGERED;
  }else {
    TxHeader.StdId = CAN_GPIO_NON_TRIGGERED;
  }

  // Configure CAN header parameters
  TxHeader.RTR = CAN_RTR_DATA;           // Data frame (not remote request)
  TxHeader.IDE = CAN_ID_STD;             // Standard 11-bit identifier
  TxHeader.DLC = 6;                      // Data length: 6 bytes (3 sensors × 2 bytes + fan data)
  TxHeader.TransmitGlobalTime = DISABLE; // No global time transmission

  for(;;) {
    // Pack pressure sensor data into CAN payload (bytes 0-5)
    if(g_sensor_data != NULL){
        // Real sensor data: pack three 16-bit pressure values
        split_int16(g_sensor_data->delta_pres_0, &TxData[0], &TxData[1]);
        split_int16(g_sensor_data->delta_pres_1, &TxData[2], &TxData[3]); // Note: original code has bug using delta_pres_0
        split_int16(g_sensor_data->delta_pres_2, &TxData[4], &TxData[5]); // Note: original code has bug using delta_pres_0
    }else {
        // Mock sensor data when real sensors unavailable
        bmp280_sensors_data_t tmp_placeholder = {
            .delta_pres_0 = -100,  // Mock pressure differential
            .delta_pres_1 = 100,   // Mock pressure differential
            .delta_pres_2 = 365    // Mock pressure differential
        };
        split_int16(tmp_placeholder.delta_pres_0, &TxData[0], &TxData[1]);
        split_int16(tmp_placeholder.delta_pres_1, &TxData[2], &TxData[3]); // Fixed to use correct values
        split_int16(tmp_placeholder.delta_pres_2, &TxData[4], &TxData[5]); // Fixed to use correct values
    }

    // Pack fan RPM data into CAN payload (bytes 6-7)
    if(g_driver_data != NULL){
        // Real fan data
        split_uint16(g_driver_data->fan_rpm, &TxData[6], &TxData[7]);
    }else {
        // Mock fan data when driver unavailable
        max6650_data_t tmp_placeholder = {
            .fan_rpm = 1000,        // Mock RPM value
            .status_flag = HAL_OK   // Mock status
        };
        split_uint16(tmp_placeholder.fan_rpm, &TxData[6], &TxData[7]);
    }

    // Transmit CAN message
    HAL_StatusTypeDef Current_Action = HAL_CAN_AddTxMessage(g_can_iface, &TxHeader, TxData, &TxMailbox);
    if (Current_Action == HAL_OK){
        NOTIFY_MESSAGE("Sent CAN Message\n");
    }

    HAL_Delay(5);  // 5ms delay between transmissions
    osDelay(1);    // Yield to other threads
  }
}

/**
 * @brief Driver collection thread cycle - placeholder for future driver functionality
 * @param argument Thread argument (unused)
 *
 * Currently unused but reserved for future driver data collection tasks.
 */
void driver_coll_cycle(void const *argument){
  for(;;){
    osDelay(1);  // Minimal delay, yield to other threads
  }
}

/**
 * @brief Multiplexer and pressure sensor thread cycle
 * @param argument Thread argument (unused)
 *
 * This thread handles:
 * 1. MAX6650 fan controller initialization and control
 * 2. BMP280 multi-sensor system initialization
 * 3. Periodic pressure measurements from 6 sensors via I2C mux
 * 4. Fan control based on global command flags
 * 5. Data storage in global structures for other threads
 */
void mux_pres_cycle(void const *argument){
  
  // Initialize MAX6650 fan controller
  max6650_handle_t fan_controller;
  max6650_config_t fan_config;
  max6650_data_t fan_data;

  // Configure MAX6650 parameters
  fan_config.hi2c = g_driver_iface;                    // I2C interface
  fan_config.add_line_connection = MAX6650_ADD_LINE_GND; // Address line grounded
  fan_config.operating_mode = MAX6650_MODE_CLOSED_LOOP;  // Closed-loop control
  fan_config.fan_voltage = MAX6650_FAN_VOLTAGE_12V;     // 12V fan
  fan_config.k_scale = MAX6650_KSCALE_4;               // K-scale factor
  fan_config.rpm_max = 3000;                           // Maximum expected RPM for normal 12V motor
  fan_config.i2c_timeout = 1000;                       // I2C timeout in ms

  bool init_succ = false;
  if (MAX6650_Init(&fan_controller, &fan_config)){
    MAX6650_ControlFan(&fan_controller, CFF_AUTO);  // Start in auto mode
    init_succ = true;
  }

  // Initialize BMP280 multi-sensor system (6 sensors on I2C mux)
  uint8_t sensor_addresses[6] = {0x76, 0x76, 0x76, 0x76, 0x76, 0x76}; // All sensors use same I2C address
  uint8_t sensor_channels[6] = {0, 1, 2, 3, 4, 5};                    // Different mux channels
  bmp280_multi_system_t sensor_system;
  bmp280_sensors_data_t pressure_data;

  bool init_success = bmp280_multi_init(&sensor_system, g_mux_iface, NULL, 0, 0, sensor_addresses, sensor_channels);
  if (!init_success) {
    NOTIFY_MESSAGE("ERROR: Failed to initialize BMP280 multi-sensor system!");
  }

  for(;;) {
    // Handle fan control commands
    if(init_success){
      if (global_action_flag != 0){
        switch(global_action_flag){
          case 1:
            MAX6650_ControlFan(&fan_controller, CFF_ON);   // Force fan ON
            break;
          case 2:
            MAX6650_ControlFan(&fan_controller, CFF_OFF);  // Force fan OFF
            break;
          default:
            MAX6650_ControlFan(&fan_controller, CFF_AUTO); // Automatic control
            break;
        }
        global_action_flag = 0;  // Clear command flag
      }

      // Perform pressure measurements
      if (bmp280_multi_force_measurement(&sensor_system)) {
        HAL_Delay(100);  // Wait for measurement completion

        // Wait for all sensors to complete measurement
        while (bmp280_multi_is_measuring(&sensor_system)) {
          HAL_Delay(10);
        }

        // Read pressure differences from all sensors
        if (bmp280_multi_read_differences(&sensor_system, &pressure_data)) {
          // Store real sensor data
          g_sensor_data->delta_pres_0 = pressure_data.delta_pres_0;
          g_sensor_data->delta_pres_1 = pressure_data.delta_pres_1;
          g_sensor_data->delta_pres_2 = pressure_data.delta_pres_2;
        } else {
          NOTIFY_MESSAGE("ERROR: Failed to read sensor differences");
        }
      }else {
        NOTIFY_MESSAGE("ERROR: Failed to force measurements");
      }
    }else {
      // Use mock data when sensors are not initialized
      g_sensor_data->delta_pres_0 = -200; // Mock Data
      g_sensor_data->delta_pres_1 = 200;  // Mock Data
      g_sensor_data->delta_pres_2 = 730;  // Mock Data
    }

    // Read fan controller data
    if(init_succ){
      if (MAX6650_ReadData(&fan_controller, &fan_data)) {
        // Store real fan data
        g_driver_data->fan_rpm = fan_data.fan_rpm;
        g_driver_data->status_flag = fan_data.status_flag;
      }
    }else {
      // Use mock data when fan controller not initialized
      g_driver_data->fan_rpm = 1200;         // Mock Data
      g_driver_data->status_flag = HAL_OK;   // Mock Data
    }

    HAL_Delay(100);  // 100ms cycle time
    osDelay(1);      // Yield to other threads
  }
}

/**
 * @brief UART data logger thread cycle
 * @param argument Thread argument (unused)
 *
 * This thread:
 * 1. Collects data from global sensor and driver structures
 * 2. Creates UART transmission packets with CRC
 * 3. Encodes data using COBS (Consistent Overhead Byte Stuffing)
 * 4. Verifies encoding integrity before transmission
 * 5. Transmits encoded data via UART every 500ms
 */
void uart_logger_cycle(void const *argument){
  for(;;) {
    raw_data_t raw_data_placeholder;

    // Collect pressure sensor data
    if(g_sensor_data != NULL){
        raw_data_placeholder.delta_pres_0 = g_sensor_data->delta_pres_0;
        raw_data_placeholder.delta_pres_1 = g_sensor_data->delta_pres_1;
        raw_data_placeholder.delta_pres_2 = g_sensor_data->delta_pres_2;
    }else {
        // Use mock data when sensor data unavailable
        bmp280_sensors_data_t tmp_placeholder = {
          .delta_pres_0 = -100, // Mock Data
          .delta_pres_1 = 100,  // Mock Data
          .delta_pres_2 = 365   // Mock Data
        };
        raw_data_placeholder.delta_pres_0 = tmp_placeholder.delta_pres_0;
        raw_data_placeholder.delta_pres_1 = tmp_placeholder.delta_pres_1;
        raw_data_placeholder.delta_pres_2 = tmp_placeholder.delta_pres_2;
    }

    // Collect fan controller data
    if(g_driver_data != NULL){
        raw_data_placeholder.fan_rpm = g_driver_data->fan_rpm;
        raw_data_placeholder.status_flag = g_driver_data->status_flag;
    }else {
        // Use mock data when driver data unavailable
        max6650_data_t tmp_placeholder = {
            .fan_rpm = 1000,        // Mock Data
            .status_flag = HAL_OK   // Mock Data
        };
        raw_data_placeholder.fan_rpm = tmp_placeholder.fan_rpm;
        raw_data_placeholder.status_flag = tmp_placeholder.status_flag;
    }

    // Create UART transmission packet with CRC
    uart_tx_logging_t temp_uart_data_placeholder = create_uart_tx_packet(&raw_data_placeholder);

    // Encode packet using COBS algorithm
    uint8_t encoded_tx_buffer[64];
    size_t encoded_tx_size = encode_uart_tx_packet(&temp_uart_data_placeholder, encoded_tx_buffer, sizeof(encoded_tx_buffer));

    // Prepare original data for verification
    uint8_t original_data[sizeof(uart_tx_logging_t) + sizeof(uint16_t)];
    memcpy(original_data, &temp_uart_data_placeholder, sizeof(uart_tx_logging_t));
    uint16_t packet_crc = calculate_crc16((uint8_t*)&temp_uart_data_placeholder, sizeof(uart_tx_logging_t));
    memcpy(original_data + sizeof(uart_tx_logging_t), &packet_crc, sizeof(uint16_t));

    // Verify COBS encoding integrity
    bool encoding_valid = verify_cobs_encoding(original_data, sizeof(original_data), encoded_tx_buffer, encoded_tx_size);
    if (encoding_valid){
      LOG_DATA_ENCODED(encoded_tx_buffer, encoded_tx_size);
      NOTIFY_MESSAGE("Verificated Encoding Data\n");
    }else{
      NOTIFY_MESSAGE("Invalid Message Generation, Rejected no verification\n");
    }

    HAL_Delay(500);  // 500ms transmission interval
    osDelay(1);      // Yield to other threads
  }
}

/* ============================================================================
 * UART RECEPTION AND COMMAND HANDLING
 * ============================================================================ */

/**
 * @brief UART reception event callback (HAL callback)
 * @param huart UART handle that triggered the event
 * @param Size Number of bytes received
 *
 * This callback is triggered when UART data is received via DMA.
 * It copies received data to main buffer and sets validation flag.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    LOG_MESSAGE("Notification\n");
    if (huart->Instance == Current_Target)
    {
        memcpy(main_buff, rx_buff, Size);  // Copy received data to main buffer
        HAL_UARTEx_ReceiveToIdle_DMA(g_data_logger, rx_buff, rx_buff_size);  // Restart DMA reception
        rx_validation = 1;  // Set flag for main thread processing
    }
}

/**
 * @brief Handle decoded UART commands for fan control
 * @param current_command Pointer to decoded command structure
 *
 * Processes fan control commands and sets global action flags:
 * - CFF_ON: Force fan ON
 * - CFF_OFF: Force fan OFF
 * - Default: Automatic fan control
 */
void handle_uart_cmd(uart_rx_command_t* current_command){
  switch(current_command->commandvalue){
    case CFF_ON:
      NOTIFY_MESSAGE("Command Target ON");
      global_action_flag = 1;  // Set flag for fan ON
      break;
    case CFF_OFF:
      NOTIFY_MESSAGE("Command Target OFF");
      global_action_flag = 2;  // Set flag for fan OFF
      break;
    default:
      NOTIFY_MESSAGE("Command Target AUTO");
      global_action_flag = 3;  // Set flag for auto control
      break;
  }
}

/**
 * @brief UART notifications and command processing thread cycle
 * @param argument Thread argument (unused)
 *
 * This thread:
 * 1. Continuously monitors for incoming UART data via DMA
 * 2. Processes received data when validation flag is set
 * 3. Decodes COBS-encoded command packets
 * 4. Verifies CRC and packet integrity
 * 5. Executes valid fan control commands
 */
void uart_notifications_cycle(void const *argument){
    for(;;)
    {
        // Setup DMA reception for incoming commands
        HAL_UARTEx_ReceiveToIdle_DMA(g_data_logger, (uint8_t *)rx_buff, sizeof(rx_buff));
        __HAL_DMA_DISABLE_IT(g_logger_dma, DMA_IT_HT);  // Disable half-transfer interrupt

        // Process received data when available
        if (rx_validation){
            NOTIFY_MESSAGE("New Message, performing verification...");
            uart_rx_command_t decoded_rx_command;

            // Attempt to decode COBS-encoded command
            if (decode_uart_rx_command(main_buff, 8, &decoded_rx_command)){
                NOTIFY_MESSAGE("Captured Valid rx_Command");
                handle_uart_cmd(&decoded_rx_command);  // Execute command
            }else {
                NOTIFY_MESSAGE("Invalid Message Captured, Rejected no verification");
            }
            rx_validation = 0;  // Clear validation flag
        }
        osDelay(1);  // Yield to other threads
    }
}

/* ============================================================================
 * INITIALIZATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize thread system with hardware handles
 * @param data_logger_place_holder UART handle for data logging
 * @param local_monitor_place_holder UART handle for local monitoring
 * @param driver_iface_placeholder I2C handle for fan driver
 * @param mux_iface_placeholder I2C handle for sensor multiplexer
 * @param logger_dma_placeholder DMA handle for UART reception
 * @param target_placeholder UART instance for target identification
 * @param can_placeholder CAN handle for data transmission
 *
 * This function assigns all hardware handles to global variables
 * for use by the various thread cycles.
 */
void perform_setup_threads(UART_HandleTypeDef *data_logger_place_holder,
                          UART_HandleTypeDef *local_monitor_place_holder,
                          I2C_HandleTypeDef *driver_iface_placeholder,
                          I2C_HandleTypeDef *mux_iface_placeholder,
                          DMA_HandleTypeDef *logger_dma_placeholder,
                          USART_TypeDef *target_placeholder,
                          CAN_HandleTypeDef *can_placeholder){
    g_data_logger = data_logger_place_holder;
    g_local_monitor = local_monitor_place_holder;
    g_driver_iface = driver_iface_placeholder;
    g_mux_iface = mux_iface_placeholder;
    g_logger_dma = logger_dma_placeholder;
    g_can_iface = can_placeholder;
    Current_Target = target_placeholder;
}

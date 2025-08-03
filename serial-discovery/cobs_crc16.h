
#ifndef COBS_CRC_H
#define COBS_CRC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Command definitions
#define CFF_ON   0x01
#define CFF_OFF  0x02
#define CFF_AUTO 0x03

// Data structures
typedef struct {
    int16_t delta_pres_0;
    int16_t delta_pres_1;
    int16_t delta_pres_2;
    uint16_t fan_rpm;
    uint8_t status_flag;
} raw_data_t;

typedef struct {
    raw_data_t live_data;
    uint16_t CRC16;
} uart_tx_logging_t;

typedef struct {
    uint8_t commandvalue;
    uint16_t CRC16;
} uart_rx_command_t;

// Function prototypes

/**
 * @brief Calculate CRC16 checksum for given data
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC16 checksum
 */
uint16_t calculate_crc16(const uint8_t *data, size_t length);

/**
 * @brief Create UART TX logging packet from raw data
 * @param raw_data Input raw data structure
 * @return Complete UART TX logging packet with CRC
 */
uart_tx_logging_t create_uart_tx_packet(const raw_data_t *raw_data);

/**
 * @brief Convert struct to byte array and perform COBS encoding
 * @param tx_packet Input UART TX packet
 * @param encoded_buffer Output buffer for encoded data (must be large enough)
 * @param buffer_size Size of output buffer
 * @return Length of encoded data including delimiter, or 0 on error
 */
size_t encode_uart_tx_packet(const uart_tx_logging_t *tx_packet, uint8_t *encoded_buffer, size_t buffer_size);

/**
 * @brief Verify COBS encoding by re-encoding and comparing
 * @param original_data Original data before encoding
 * @param original_size Size of original data
 * @param encoded_data Encoded data to verify
 * @param encoded_size Size of encoded data
 * @return true if encoding is valid, false otherwise
 */
bool verify_cobs_encoding(const uint8_t *original_data, size_t original_size, 
                         const uint8_t *encoded_data, size_t encoded_size);

/**
 * @brief Decode COBS encoded data
 * @param encoded_data Input COBS encoded data (with delimiter)
 * @param encoded_size Size of encoded data
 * @param decoded_buffer Output buffer for decoded data
 * @param buffer_size Size of output buffer
 * @return Length of decoded data, or 0 on error
 */
size_t decode_cobs_data(const uint8_t *encoded_data, size_t encoded_size, 
                       uint8_t *decoded_buffer, size_t buffer_size);

/**
 * @brief Decode and verify UART RX command packet
 * @param encoded_data COBS encoded command data (with delimiter)
 * @param encoded_size Size of encoded data
 * @param rx_command Output command structure
 * @return true if decoding and CRC verification successful, false otherwise
 */
bool decode_uart_rx_command(const uint8_t *encoded_data, size_t encoded_size, 
                           uart_rx_command_t *rx_command);

/**
 * @brief Extract and validate command from RX command structure
 * @param rx_command Input command structure
 * @param command_name Output string buffer for command name (optional, can be NULL)
 * @param name_size Size of command name buffer
 * @return Command value if valid (CFF_ON, CFF_OFF, CFF_AUTO), or 0 if invalid
 */
uint8_t extract_command(const uart_rx_command_t *rx_command, char *command_name, size_t name_size);

/**
 * @brief COBS encode data with delimiter
 * @param input Input data buffer
 * @param input_size Size of input data
 * @param output Output buffer for encoded data
 * @param output_size Size of output buffer
 * @return Length of encoded data including delimiter, or 0 on error
 */
size_t cobs_encode(const uint8_t *input, size_t input_size, uint8_t *output, size_t output_size);

#endif // COBS_CRC_H


#include "cobs_crc16.h"
#include <string.h>
#include <stdio.h>

// CRC16-CCITT polynomial: 0x1021
#define CRC16_POLY 0x1021

uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

uart_tx_logging_t create_uart_tx_packet(const raw_data_t *raw_data) {
    uart_tx_logging_t tx_packet;
    
    // Copy raw data
    tx_packet.live_data = *raw_data;
    
    // Calculate CRC16 over the raw data portion
    uint8_t *data_ptr = (uint8_t *)&tx_packet.live_data;
    tx_packet.CRC16 = calculate_crc16(data_ptr, sizeof(raw_data_t));
    
    return tx_packet;
}

size_t cobs_encode(const uint8_t *input, size_t input_size, uint8_t *output, size_t output_size) {
    if (input_size == 0 || output_size < input_size + 2) {
        return 0; // Not enough space
    }
    
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;
    
    while (read_index < input_size) {
        if (input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            if (write_index >= output_size) return 0;
        } else {
            output[write_index++] = input[read_index];
            code++;
            if (write_index >= output_size) return 0;
            
            if (code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
                if (write_index >= output_size) return 0;
            }
        }
        read_index++;
    }
    
    output[code_index] = code;
    
    // Add delimiter
    if (write_index >= output_size) return 0;
    output[write_index++] = 0x00;
    
    return write_index;
}

static size_t cobs_decode(const uint8_t *input, size_t input_size, uint8_t *output, size_t output_size) {
    if (input_size < 2 || input[input_size - 1] != 0x00) {
        return 0; // Invalid COBS packet (should end with delimiter)
    }
    
    // Remove delimiter for processing
    input_size--;
    
    size_t read_index = 0;
    size_t write_index = 0;
    
    while (read_index < input_size) {
        uint8_t code = input[read_index++];
        
        if (code == 0) {
            return 0; // Invalid: zero found in encoded data
        }
        
        for (int i = 1; i < code && read_index < input_size; i++) {
            if (write_index >= output_size) return 0;
            output[write_index++] = input[read_index++];
        }
        
        if (code < 0xFF && read_index < input_size) {
            if (write_index >= output_size) return 0;
            output[write_index++] = 0x00;
        }
    }
    
    return write_index;
}

size_t encode_uart_tx_packet(const uart_tx_logging_t *tx_packet, uint8_t *encoded_buffer, size_t buffer_size) {
    // Convert struct to byte array
    uint8_t raw_bytes[sizeof(uart_tx_logging_t)];
    memcpy(raw_bytes, tx_packet, sizeof(uart_tx_logging_t));
    
    // Calculate CRC for the entire packet
    uint16_t packet_crc = calculate_crc16(raw_bytes, sizeof(uart_tx_logging_t));
    
    // Create extended packet with CRC
    uint8_t extended_packet[sizeof(uart_tx_logging_t) + sizeof(uint16_t)];
    memcpy(extended_packet, raw_bytes, sizeof(uart_tx_logging_t));
    memcpy(extended_packet + sizeof(uart_tx_logging_t), &packet_crc, sizeof(uint16_t));
    
    // COBS encode
    return cobs_encode(extended_packet, sizeof(extended_packet), encoded_buffer, buffer_size);
}

bool verify_cobs_encoding(const uint8_t *original_data, size_t original_size, 
                         const uint8_t *encoded_data, size_t encoded_size) {
    uint8_t temp_buffer[original_size * 2 + 10]; // Safe size for re-encoding
    
    size_t re_encoded_size = cobs_encode(original_data, original_size, temp_buffer, sizeof(temp_buffer));
    
    if (re_encoded_size != encoded_size) {
        return false;
    }
    
    return memcmp(temp_buffer, encoded_data, encoded_size) == 0;
}

size_t decode_cobs_data(const uint8_t *encoded_data, size_t encoded_size, 
                       uint8_t *decoded_buffer, size_t buffer_size) {
    return cobs_decode(encoded_data, encoded_size, decoded_buffer, buffer_size);
}

bool decode_uart_rx_command(const uint8_t *encoded_data, size_t encoded_size, 
                           uart_rx_command_t *rx_command) {
    uint8_t decoded_buffer[sizeof(uart_rx_command_t) + sizeof(uint16_t) + 10];
    
    // Decode COBS data
    size_t decoded_size = cobs_decode(encoded_data, encoded_size, decoded_buffer, sizeof(decoded_buffer));
    
    if (decoded_size < sizeof(uart_rx_command_t)) {
        return false; // Not enough data
    }
    
    // Extract command structure
    memcpy(rx_command, decoded_buffer, sizeof(uart_rx_command_t));
    
    // Verify CRC
    uint8_t command_data = rx_command->commandvalue;
    uint16_t calculated_crc = calculate_crc16(&command_data, sizeof(uint8_t));
    
    if (calculated_crc != rx_command->CRC16) {
        return false; // CRC mismatch
    }
    
    // Additional verification: check if there's a packet CRC after the command structure
    if (decoded_size >= sizeof(uart_rx_command_t) + sizeof(uint16_t)) {
        uint16_t packet_crc;
        memcpy(&packet_crc, decoded_buffer + sizeof(uart_rx_command_t), sizeof(uint16_t));
        
        uint16_t calculated_packet_crc = calculate_crc16(decoded_buffer, sizeof(uart_rx_command_t));
        
        if (calculated_packet_crc != packet_crc) {
            return false; // Packet CRC mismatch
        }
    }
    
    return true;
}

uint8_t extract_command(const uart_rx_command_t *rx_command, char *command_name, size_t name_size) {
    uint8_t command = rx_command->commandvalue;
    
    // Validate command
    switch (command) {
        case CFF_ON:
            if (command_name && name_size > 6) {
                strncpy(command_name, "CFF_ON", name_size - 1);
                command_name[name_size - 1] = '\0';
            }
            return command;
            
        case CFF_OFF:
            if (command_name && name_size > 7) {
                strncpy(command_name, "CFF_OFF", name_size - 1);
                command_name[name_size - 1] = '\0';
            }
            return command;
            
        case CFF_AUTO:
            if (command_name && name_size > 8) {
                strncpy(command_name, "CFF_AUTO", name_size - 1);
                command_name[name_size - 1] = '\0';
            }
            return command;
            
        default:
            if (command_name && name_size > 8) {
                strncpy(command_name, "INVALID", name_size - 1);
                command_name[name_size - 1] = '\0';
            }
            return 0; // Invalid command
    }
}

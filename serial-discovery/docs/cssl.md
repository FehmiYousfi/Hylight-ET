
# CSSL Library - C Serial Support Library

## Overview

The CSSL (C Serial Support Library) is an enhanced serial communication library that provides event-driven serial I/O with advanced data processing capabilities. It's designed for robust communication between embedded systems and Linux hosts.

## Core Features

### 1. Event-Driven Serial Communication
- **Signal-based I/O**: Uses real-time signals (SIGRTMIN-SIGRTMAX) for asynchronous data reception
- **Non-blocking operations**: Supports both blocking and non-blocking serial operations
- **Automatic port detection**: Dynamically finds available signals for serial operations

### 2. Enhanced Data Type Support
```c
typedef enum {
    CSSL_DATA_RAW,           /* Raw binary data */
    CSSL_DATA_ASCII,         /* ASCII text data */
    CSSL_DATA_BINARY,        /* Binary protocol data */
    CSSL_DATA_FRAMED,        /* Framed data with delimiters */
    CSSL_DATA_FIXED_LENGTH   /* Fixed length packets */
} cssl_data_type_t;
```

### 3. Frame Delimiter Types
```c
typedef enum {
    CSSL_DELIM_NONE,         /* No delimiter */
    CSSL_DELIM_NEWLINE,      /* \n delimiter */
    CSSL_DELIM_CRLF,         /* \r\n delimiter */
    CSSL_DELIM_CUSTOM,       /* Custom delimiter */
    CSSL_DELIM_LENGTH_PREFIX /* Length-prefixed frames */
} cssl_delimiter_t;
```

## Key Functions

### Port Management
```c
// Enhanced port opening with data type specification
cssl_t *cssl_open_enhanced(const char *fname, cssl_enhanced_callback_t callback,
                          int id, int baud, int bits, int parity, int stop,
                          cssl_data_type_t data_type);

// Close and cleanup port
void cssl_close(cssl_t *serial);
```

### Configuration Functions
```c
// Set data processing type
void cssl_set_data_type(cssl_t *serial, cssl_data_type_t type);

// Configure frame delimiters
void cssl_set_delimiter(cssl_t *serial, cssl_delimiter_t delim_type, 
                       const uint8_t *custom_delim, int delim_len);

// Set fixed frame size for FIXED_LENGTH mode
void cssl_set_frame_size(cssl_t *serial, int frame_size);

// Configure flow control (RTS/CTS and XON/XOFF)
void cssl_setflowcontrol(cssl_t *serial, int rtscts, int xonxoff);
```

### Data Transmission
```c
// Send single character
void cssl_putchar(cssl_t *serial, char c);

// Send string data
void cssl_putstring(cssl_t *serial, char *str);

// Send binary data with length
void cssl_putdata(cssl_t *serial, uint8_t *data, int datalen);
```

### Statistics and Monitoring
```c
typedef struct {
    uint64_t bytes_received;
    uint64_t bytes_sent;
    uint32_t frames_received;
    uint32_t frames_sent;
    uint32_t errors;
    struct timespec last_activity;
} cssl_stats_t;

// Get port statistics
void cssl_get_stats(cssl_t *serial, cssl_stats_t *stats);

// Reset statistics counters
void cssl_reset_stats(cssl_t *serial);
```

## Enhanced Callback System

The library supports an enhanced callback system that provides detailed information about received data:

```c
typedef void (*cssl_enhanced_callback_t)(int id,           /* port id */
                                        uint8_t *buffer,    /* data received */
                                        int len,            /* length of data */
                                        cssl_data_type_t type, /* data type */
                                        struct timespec *timestamp); /* timestamp */
```

### Callback Features
- **Precise timestamps**: Microsecond-precision timestamps for each data packet
- **Automatic type detection**: Intelligent detection of ASCII vs binary data
- **Frame assembly**: Automatic assembly of framed data based on delimiters
- **Error handling**: Built-in error detection and recovery

## Data Extraction Utilities

### ASCII Line Extraction
```c
// Extract ASCII lines with printable character filtering
int cssl_extract_ascii_line(const uint8_t *buffer, int len, char *output, int max_len);
```

### Binary Frame Extraction
```c
// Extract binary frames using start/end sequences
int cssl_extract_binary_frame(const uint8_t *buffer, int len, 
                             const uint8_t *start_seq, int start_len,
                             const uint8_t *end_seq, int end_len,
                             uint8_t *output, int max_len);
```

### Length-Prefixed Data
```c
// Extract length-prefixed frames (big-endian 16-bit length)
int cssl_extract_length_prefixed(const uint8_t *buffer, int len, 
                                uint8_t *output, int max_len);
```

## Utility Functions

### CRC Validation
```c
// Calculate CRC16 checksum
uint16_t cssl_calculate_crc16(const uint8_t *data, int len);

// Validate frame with expected CRC
int cssl_validate_frame(const uint8_t *frame, int len, uint16_t expected_crc);
```

### Debug Output
```c
// Hex dump utility for debugging
void cssl_hex_dump(const uint8_t *data, int len, const char *prefix);
```

## Error Handling

```c
typedef enum {
    CSSL_OK,                      /* Operation successful */
    CSSL_ERROR_NOSIGNAL,          /* No free signal available */
    CSSL_ERROR_NOTSTARTED,        /* Library not initialized */
    CSSL_ERROR_NULLPOINTER,       /* Null pointer passed */
    CSSL_ERROR_OOPS,              /* Internal error */
    CSSL_ERROR_MEMORY,            /* Memory allocation failed */
    CSSL_ERROR_OPEN,              /* Failed to open device */
    CSSL_ERROR_TIMEOUT,           /* Frame assembly timeout */
    CSSL_ERROR_FRAME_TOO_LARGE,   /* Frame exceeds maximum size */
    CSSL_ERROR_INVALID_DELIMITER  /* Invalid delimiter configuration */
} cssl_error_t;

// Get last error message
const char *cssl_geterrormsg();

// Get last error code
int cssl_geterror();
```

## Usage Examples

### Basic Serial Communication
```c
#include "cssl.h"

void data_callback(int id, uint8_t *buffer, int len, 
                  cssl_data_type_t type, struct timespec *timestamp) {
    printf("Received %d bytes of %s data\n", len, 
           type == CSSL_DATA_ASCII ? "ASCII" : "BINARY");
}

int main() {
    cssl_start();
    
    cssl_t *port = cssl_open_enhanced("/dev/ttyUSB0", data_callback, 1,
                                     9600, 8, 0, 1, CSSL_DATA_RAW);
    
    if (port) {
        cssl_setflowcontrol(port, 0, 0);  // No flow control
        
        // Send test data
        cssl_putstring(port, "Hello, STM32!\n");
        
        // Let it run for processing
        sleep(10);
        
        cssl_close(port);
    }
    
    cssl_stop();
    return 0;
}
```

### Framed Data Processing
```c
// Configure for newline-delimited frames
cssl_set_data_type(port, CSSL_DATA_FRAMED);
cssl_set_delimiter(port, CSSL_DELIM_NEWLINE, NULL, 0);

// Configure for custom delimiter (0x00)
uint8_t delimiter[] = {0x00};
cssl_set_delimiter(port, CSSL_DELIM_CUSTOM, delimiter, 1);
```

### Statistics Monitoring
```c
void print_port_stats(cssl_t *port) {
    cssl_stats_t stats;
    cssl_get_stats(port, &stats);
    
    printf("Bytes RX: %llu, TX: %llu\n", 
           stats.bytes_received, stats.bytes_sent);
    printf("Frames RX: %u, TX: %u\n", 
           stats.frames_received, stats.frames_sent);
    printf("Errors: %u\n", stats.errors);
}
```

## Configuration Constants

```c
#define CSSL_BUFFER_SIZE 2048        /* Enhanced buffer size */
#define CSSL_MAX_FRAME_SIZE 512      /* Maximum frame size */
#define CSSL_FRAME_TIMEOUT_MS 100    /* Frame assembly timeout */
```

## Thread Safety

- **Signal-safe operations**: All signal handlers are designed to be async-signal-safe
- **Re-entrant callbacks**: Callback functions should be designed to handle re-entrant calls
- **Resource protection**: Proper cleanup ensures no resource leaks

## Performance Features

- **Optimized buffer management**: Configurable read sizes for efficiency
- **Frame assembly optimization**: Efficient delimiter detection and frame building
- **Minimal memory allocation**: Static buffers with configurable sizes
- **Low latency**: Direct signal-based I/O without polling

This enhanced CSSL library provides a robust foundation for serial communication in embedded Linux applications, with particular strength in handling framed protocols and real-time data processing.


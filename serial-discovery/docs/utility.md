
# Serial-Discovery Tool Documentation

## Overview

The `serial-discovery` tool is a comprehensive serial data collector designed for communication with STM32 embedded systems. It features advanced packet processing, COBS decoding, and automatic data validation with real-time statistics.

## Features

### Core Capabilities
- **Multi-format data processing**: RAW, ASCII, BINARY, FRAMED, FIXED_LENGTH
- **COBS protocol support**: Automatic decoding and validation
- **CRC16 verification**: Data integrity checking
- **Real-time statistics**: Comprehensive monitoring and reporting
- **Auto-control modes**: Automated command transmission
- **Signal handling**: Graceful shutdown and statistics on demand

### Communication Protocols
- **UART/Serial**: Standard serial communication protocols
- **Custom framing**: Configurable delimiters and frame sizes
- **Flow control**: RTS/CTS and XON/XOFF support
- **Timeouts**: Configurable read timeouts

## Command Line Interface

### Basic Usage
```bash
./serial-discovery [options]
```

### Connection Options
```bash
-d, --device <device>     # Serial device (default: /dev/ttyUSB0)
-b, --baudrate <rate>     # Baudrate (default: 9600)
-D, --databits <bits>     # Data bits: 5-8 (default: 8)
-p, --parity <parity>     # Parity: 0=none, 1=odd, 2=even (default: 0)
-s, --stopbits <bits>     # Stop bits: 1-2 (default: 1)
-r, --rtscts              # Enable RTS/CTS flow control
-x, --xonxoff             # Enable XON/XOFF flow control
```

### Data Processing Options
```bash
-t, --type <type>         # Data type: raw, ascii, binary, framed (default: raw)
-l, --delimiter <delim>   # Frame delimiter: newline, crlf, or custom
                          # Custom format: "\n" or "0x0A" or "ABC"
                          # (default: 0x00)
-f, --frame-size <size>   # Fixed frame size in bytes
```

### Output Options
```bash
-o, --output <file>       # Output file (default: stdout)
-v, --verbose             # Verbose output
-H, --hex                 # Hex output format
-S, --statistics          # Show periodic statistics
```

### Control Options
```bash
-T, --timeout <decisec>   # Read timeout in deciseconds
-m, --max-packets <num>   # Maximum packets to collect
-u, --duration <seconds>  # Collection duration in seconds
-c, --control <type>      # Auto-control mode: FON, FOFF, FOA
-C, --control-timeout <ms> # Control command interval in milliseconds
-h, --help                # Show help
```

## Usage Examples

### Basic Data Collection
```bash
# Listen on USB serial with default settings
./serial-discovery -d /dev/ttyUSB0 -v

# High-speed data collection with hex output
./serial-discovery -d /dev/ttyUSB0 -b 115200 -H -v
```

### STM32 Communication
```bash
# COBS-framed data from STM32 with 0x00 delimiter
./serial-discovery -d /dev/ttyACM0 -b 9600 -t framed -l "0x00" -v

# Auto-control fan every 2 seconds
./serial-discovery -d /dev/ttyACM0 -b 9600 -c FON -C 2000 -v
```

### Data Logging
```bash
# Log ASCII data to file
./serial-discovery -d /dev/ttyS0 -t ascii -l newline -o sensor_data.log

# Binary data collection with statistics
./serial-discovery -d /dev/ttyUSB0 -t binary -f 32 -o data.bin -S
```

### Advanced Configuration
```bash
# Complex delimiter with flow control
./serial-discovery -d /dev/ttyUSB0 -t framed -l "0x0D0A" -r -v

# Limited collection (100 packets or 60 seconds)
./serial-discovery -d /dev/ttyUSB0 -m 100 -u 60 -v
```

## Data Type Processing

### RAW Data Mode
- **Description**: Processes all received data without interpretation
- **Use case**: General-purpose data collection
- **Output**: Raw bytes with automatic type detection

### ASCII Data Mode  
- **Description**: Optimized for text-based protocols
- **Features**: Printable character filtering, escape sequence handling
- **Use case**: Text-based sensor outputs, debug messages

### BINARY Data Mode
- **Description**: Handles binary protocols with byte-level processing
- **Features**: Hex output, binary data validation
- **Use case**: Binary sensor data, protocol analysis

### FRAMED Data Mode
- **Description**: Assembles packets based on delimiters
- **Features**: Configurable delimiters, frame timeout handling
- **Use case**: COBS-encoded data, packet-based protocols

### FIXED_LENGTH Data Mode
- **Description**: Collects data in fixed-size packets
- **Features**: Automatic frame assembly, size validation
- **Use case**: Structured data packets, sensor arrays

## Delimiter Configuration

### Predefined Delimiters
```bash
-l newline    # Line feed (0x0A)
-l crlf       # Carriage return + line feed (0x0D0A)
```

### Custom Delimiters
```bash
-l "0x00"         # Null byte (COBS delimiter)
-l "0x0D0A"       # Custom hex sequence
-l "\n"           # Escape sequence
-l "END"          # ASCII string
```

## Control Commands

### Fan Control Commands
```bash
FON   (0x01)  # Fan control ON
FOFF  (0x02)  # Fan control OFF
FOA   (0x03)  # Fan control AUTO mode
```

### Auto-Control Mode
```bash
# Send FON command every 1 second
./serial-discovery -d /dev/ttyUSB0 -c FON -C 1000 -v

# Automated testing with FOFF every 5 seconds
./serial-discovery -d /dev/ttyUSB0 -c FOFF -C 5000 -o test_log.txt
```

## Real-Time Statistics

### Statistics Display
```bash
=== Port Statistics ===
Bytes received: 15420
Bytes sent: 156
Frames received: 245
Frames sent: 12
Errors: 0
Total packets processed: 245
Average bytes/second: 128.50
Average packets/second: 2.04
=======================
```

### Accessing Statistics
- **Automatic**: Displayed on program exit
- **On-demand**: Send SIGUSR1 signal to running process
- **Periodic**: Use `-S` flag for periodic display

```bash
# Send statistics signal to running process
kill -USR1 <pid>
```

## Signal Handling

### Supported Signals
```bash
SIGINT/SIGTERM  # Graceful shutdown (Ctrl+C)
SIGUSR1         # Display current statistics
```

### Signal Usage
```bash
# Start collector in background
./serial-discovery -d /dev/ttyUSB0 -o data.log &

# Get statistics without stopping
kill -USR1 $!

# Stop gracefully
kill -TERM $!
```

## Output Formats

### Console Output (Verbose Mode)
```
[14:30:25.123] Packet #1 (RAW, 16 bytes): 01 45 A2 3F 12 34 56 78 9A BC DE F0 42 85 FF 00
                     -> Valid TX Buffer Decoded:
                        Delta Pres 0: 325
                        Delta Pres 1: 16191
                        Delta Pres 2: 4660
                        Fan RPM: 22136
                        Status Flag: 0x9A
```

### File Output
```
# Enhanced Serial Data Collector Log
# Device: /dev/ttyUSB0, Baudrate: 9600
# Started at: Sat Aug  3 14:30:20 2025
[14:30:25.123] Packet #1 (RAW, 16 bytes): 01 45 A2 3F 12 34 56 78 9A BC DE F0 42 85 FF 00
[14:30:25.150] Packet #2 (RAW, 16 bytes): 01 46 A3 40 12 35 56 79 9B BD DF F1 42 86 FF 00
```

### Hex Output Mode
```
[14:30:25.123] Packet #1 (BINARY, 16 bytes): 
01 45 A2 3F 12 34 56 78 9A BC DE F0 42 85 FF 00
```

## COBS Packet Processing

### Automatic COBS Detection
The tool automatically detects and processes COBS-encoded packets:

1. **Frame Detection**: Identifies 0x00-terminated frames
2. **COBS Decoding**: Removes byte stuffing and recovers original data
3. **CRC Validation**: Verifies data integrity using CRC16
4. **Data Extraction**: Extracts sensor data from validated packets

### STM32 Data Extraction
```
Valid TX Buffer Decoded:
  Delta Pres 0: 325      # Differential pressure sensor 0 (int16_t)
  Delta Pres 1: 16191    # Differential pressure sensor 1 (int16_t)  
  Delta Pres 2: 4660     # Differential pressure sensor 2 (int16_t)
  Fan RPM: 22136         # Fan speed in RPM (uint16_t)
  Status Flag: 0x9A      # System status flags (uint8_t)
```

## Error Handling

### Common Errors
```bash
# Device access error
Error opening serial port /dev/ttyUSB0: Permission denied

# Invalid configuration
Data bits must be 5-8
Frame size must be 1-512

# Communication errors
Frame assembly timeout
CRC mismatch detected
```

### Error Recovery
- **Automatic retry**: Connection attempts with exponential backoff
- **Frame resync**: Automatic resynchronization on delimiter detection
- **Statistics tracking**: Error counting for debugging

## Performance Optimization

### Buffer Management
- **Read optimization**: Configurable read sizes for efficiency
- **Frame assembly**: Optimized delimiter detection
- **Memory usage**: Static buffers with minimal allocation

### Throughput Optimization
```bash
# High-speed data collection
./serial-discovery -d /dev/ttyUSB0 -b 921600 -t binary -f 64

# Bulk data logging
./serial-discovery -d /dev/ttyUSB0 -b 115200 -o /tmp/data.bin -T 50
```

## Integration Examples

### Shell Scripting
```bash
#!/bin/bash
# Automated data collection script

LOG_FILE="/var/log/sensor_data_$(date +%Y%m%d_%H%M%S).log"

# Start data collection
./serial-discovery -d /dev/ttyACM0 -b 9600 \
    -t framed -l "0x00" \
    -o "$LOG_FILE" \
    -v -S &

PID=$!

# Run for 1 hour
sleep 3600

# Stop gracefully
kill -TERM $PID
wait $PID

echo "Data collection completed: $LOG_FILE"
```

### Python Integration
```python
import subprocess
import signal
import time

def collect_sensor_data(duration=60):
    cmd = [
        './serial-discovery',
        '-d', '/dev/ttyACM0',
        '-b', '9600',
        '-t', 'framed',
        '-l', '0x00',
        '-v'
    ]
    
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:
        stdout, stderr = proc.communicate(timeout=duration)
        return stdout.decode('utf-8')
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGTERM)
        stdout, stderr = proc.communicate()
        return stdout.decode('utf-8')
```

## Troubleshooting

### Common Issues

1. **Permission Denied**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **Device Not Found**
   ```bash
   ls /dev/tty*  # List available devices
   dmesg | grep tty  # Check kernel messages
   ```

3. **No Data Received**
   ```bash
   # Check device settings
   stty -F /dev/ttyUSB0
   
   # Test with different baudrates
   ./serial-discovery -d /dev/ttyUSB0 -b 115200 -v
   ```

4. **Frame Sync Issues**
   ```bash
   # Try different delimiters
   ./serial-discovery -d /dev/ttyUSB0 -l newline -v
   ./serial-discovery -d /dev/ttyUSB0 -l "0x00" -v
   ```

The serial-discovery tool provides a comprehensive solution for STM32-Linux communication with robust error handling, flexible configuration, and advanced protocol support.



# STM32 Core Source Code Design Documentation

## Overview

The Hylight-ET STM32 firmware implements a multi-threaded embedded system using FreeRTOS for real-time task management. The system integrates multiple sensors (BMP280 pressure sensors), fan control (MAX6650), and communication interfaces (UART, CAN, I2C) to create a comprehensive environmental monitoring and control solution.

## Architecture Overview

### System Components
- **STM32L432KC**: Ultra-low-power ARM Cortex-M4 microcontroller
- **FreeRTOS**: Real-time operating system for task scheduling
- **Multi-sensor Integration**: BMP280 pressure sensors via I2C multiplexer
- **Fan Control**: MAX6650 PWM fan controller
- **Communication**: UART (data logging), CAN (system communication)

### Data Flow Architecture
```
[BMP280 Sensors] → [I2C Mux] → [STM32] → [Data Processing] → [UART/CAN Output]
      ↓                                         ↑
[MAX6650 Fan Controller] ← [Control Logic] ← [Command Input]
```

## Core Files Analysis

## 1. main.c - System Initialization and Entry Point

### Purpose
Central system initialization and FreeRTOS task creation.

### Key Functions
- **`main()`**: System entry point and initialization sequence
- **`SystemClock_Config()`**: Configure system clock (MSI at 4MHz)
- **Peripheral Initialization**: CAN, UART, I2C, GPIO, DMA setup

### Initialization Sequence
```c
1. HAL_Init() - HAL library initialization
2. SystemClock_Config() - Clock configuration
3. Peripheral initialization (GPIO, DMA, CAN, UART, I2C)
4. Custom setup: perform_setup_threads()
5. FreeRTOS task creation
6. osKernelStart() - Start scheduler
```

### Hardware Configuration
- **CAN1**: 500kbps, normal mode, standard frames
- **USART1/2**: 9600 baud, 8N1 configuration
- **I2C1**: 400kHz fast mode
- **GPIO PA6**: Input pin for CAN verification

### Memory and Performance
- Stack size per task: 128 words (512 bytes)
- Task priority: Below normal for all tasks
- Static memory allocation for FreeRTOS idle task

## 2. thread_proto.c - Multi-Threading and Task Management

### Purpose
Implements the main application logic through FreeRTOS tasks and manages inter-task communication.

### Task Architecture

#### Task 1: `can_logger_cycle`
**Function**: CAN message transmission
**Period**: 5ms + 1ms OS delay
**Data Transmitted**:
- Pressure differentials (6 bytes): delta_pres_0, delta_pres_1, delta_pres_2
- Fan RPM (2 bytes): uint16_t fan speed
- CAN ID based on GPIO PA6 state

```c
// CAN Message Format
TxData[0-1]: delta_pres_0 (high/low byte)
TxData[2-3]: delta_pres_1 (high/low byte)  
TxData[4-5]: delta_pres_2 (high/low byte)
TxData[6-7]: fan_rpm (high/low byte)
```

#### Task 2: `mux_pres_cycle`
**Function**: Sensor reading and fan control
**Components**:
- BMP280 multi-sensor system (6 sensors via I2C multiplexer)
- MAX6650 fan controller
- Data acquisition and processing

**Process Flow**:
1. Initialize BMP280 multi-sensor system
2. Initialize MAX6650 fan controller
3. Force measurement on all sensors
4. Wait for measurement completion
5. Read pressure differences
6. Read fan status
7. Update global data structures

#### Task 3: `uart_logger_cycle`
**Function**: UART data transmission with COBS encoding
**Period**: 500ms
**Process**:
1. Collect sensor and fan data
2. Create UART transmission packet
3. Apply COBS (Consistent Overhead Byte Stuffing) encoding
4. CRC16 verification
5. Transmit encoded data

#### Task 4: `uart_notifications_cycle`
**Function**: UART command reception and processing
**Features**:
- DMA-based reception
- COBS decoding
- Command validation with CRC16
- Fan control command handling (CFF_ON, CFF_OFF, CFF_AUTO)

### Data Structures

#### Global Data Management
```c
// Sensor data structure
typedef struct {
    int16_t delta_pres_0;  // Pressure difference sensor 1-2
    int16_t delta_pres_1;  // Pressure difference sensor 3-4  
    int16_t delta_pres_2;  // Pressure difference sensor 5-6
} bmp280_sensors_data_t;

// Fan controller data
typedef struct {
    uint16_t fan_rpm;      // Current fan speed
    uint8_t status_flag;   // Operating status
} max6650_data_t;
```

### Inter-Task Communication
- **Global pointers**: Shared data between tasks
- **Mock data fallback**: System continues operation with placeholder data if sensors fail
- **Thread-safe access**: FreeRTOS manages task scheduling

## 3. bmp280.c - Pressure Sensor Management

### Purpose
Bosch BMP280 pressure sensor driver with temperature compensation and calibration.

### Key Features
- **I2C Communication**: HAL-based I2C interface
- **Calibration**: Automatic calibration data reading and storage
- **Compensation**: Temperature and pressure compensation algorithms
- **Multiple Modes**: Normal, forced, sleep modes
- **BME280 Support**: Humidity sensing for BME280 variant

### Core Functions

#### Initialization
```c
bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params)
```
- Validates I2C address
- Performs soft reset
- Reads calibration coefficients
- Configures operating parameters

#### Data Reading
```c
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature, 
                       uint32_t *pressure, uint32_t *humidity)
```
- Reads raw ADC values
- Applies temperature compensation
- Calculates compensated pressure
- Returns fixed-point values

### Compensation Algorithms
- **Temperature**: 32-bit signed compensation with fine temperature calculation
- **Pressure**: 64-bit compensation using temperature fine value
- **Precision**: 24-bit integer + 8-bit fractional for pressure

### Configuration Parameters
- **Oversampling**: Temperature, pressure, humidity
- **Filter**: IIR filter coefficient
- **Standby Time**: Power management
- **Operating Mode**: Sleep, forced, normal

## 4. integration_mux_bmp280.c - Multi-Sensor System

### Purpose
Manages multiple BMP280 sensors through TCA9548A I2C multiplexer for differential pressure measurement.

### System Configuration
- **6 BMP280 Sensors**: Connected through I2C multiplexer
- **3 Pressure Pairs**: Calculates differential pressures
- **Channel Mapping**: Each sensor on separate multiplexer channel

### Core Functions

#### System Initialization
```c
bool bmp280_multi_init(bmp280_multi_system_t *system, 
                       I2C_HandleTypeDef *hi2c,
                       GPIO_TypeDef *rst_port, uint16_t rst_pin,
                       uint8_t mux_addr_offset,
                       uint8_t sensor_addresses[NUM_BMP280_SENSORS],
                       uint8_t sensor_channels[NUM_BMP280_SENSORS])
```

#### Differential Pressure Calculation
```c
bool bmp280_multi_read_differences(bmp280_multi_system_t *system, 
                                   bmp280_sensors_data_t *data)
```
- **delta_pres_0**: Sensor1 - Sensor2 pressure difference
- **delta_pres_1**: Sensor3 - Sensor4 pressure difference  
- **delta_pres_2**: Sensor5 - Sensor6 pressure difference

### Error Handling
- Individual sensor failure tolerance
- Graceful degradation with partial sensor operation
- CRC and validation for data integrity

## 5. integration_max6650.c - Fan Controller

### Purpose
MAX6650 PWM fan controller driver for variable speed fan control with tachometer feedback.

### Features
- **PWM Control**: Variable speed control
- **Tachometer Reading**: RPM measurement
- **Multiple Operating Modes**: Software control, closed-loop, full-on/off
- **I2C Interface**: Register-based configuration

### Core Functions

#### Initialization
```c
bool MAX6650_Init(max6650_handle_t *handle, max6650_config_t *config)
```
- I2C address configuration based on ADD pin
- Operating mode setup
- Voltage and scaling configuration

#### Fan Control
```c
bool MAX6650_ControlFan(max6650_handle_t *handle, uint8_t control_flag)
```
- **CFF_ON**: Full speed operation
- **CFF_OFF**: Fan stop
- **CFF_AUTO**: Closed-loop speed control

#### Speed Control
```c
bool MAX6650_SetSpeedPercent(max6650_handle_t *handle, uint8_t speed_percent)
```
- Percentage-based speed control
- KTACH calculation for target RPM
- Hardware PWM generation

## 6. i2c-mux.c - I2C Multiplexer Management

### Purpose
TCA9548A/PCA9548A I2C multiplexer driver for managing multiple I2C devices on single bus.

### Features
- **8-Channel Multiplexing**: Up to 8 I2C devices
- **Channel Selection**: Individual or multiple channel enable
- **Reset Control**: Hardware reset capability
- **Verification**: Read-back verification of channel selection

### Core Functions
```c
int i2c_mux_select(i2c_mux_t* mux, int ch)        // Single channel
int i2c_mux_select_multi(i2c_mux_t* mux, uint8_t mask)  // Multiple channels
int i2c_mux_reset(i2c_mux_t* mux)                // Hardware reset
```

## 7. cobs_crc16.c - Data Encoding and Integrity

### Purpose
Implements COBS (Consistent Overhead Byte Stuffing) encoding and CRC16 error detection for reliable UART communication.

### COBS Encoding
- **Zero-byte elimination**: Removes zero bytes from data stream
- **Framing**: Clear packet boundaries with delimiter
- **Overhead**: Minimal encoding overhead (~0.4%)

### CRC16 Implementation
- **Polynomial**: CRC16-CCITT (0x1021)
- **Initial Value**: 0xFFFF
- **Application**: Packet integrity verification

### Data Structures
```c
typedef struct {
    raw_data_t live_data;    // Sensor and fan data
    uint16_t CRC16;          // Data integrity checksum
} uart_tx_logging_t;

typedef struct {
    uint8_t commandvalue;    // Command byte
    uint16_t CRC16;          // Command integrity checksum
} uart_rx_command_t;
```

### Encoding Process
1. Create packet with sensor data
2. Calculate CRC16 over data
3. Apply COBS encoding
4. Add packet delimiter
5. Verify encoding integrity

## 8. freertos.c - RTOS Configuration

### Purpose
FreeRTOS integration and idle task memory management.

### Memory Management
- **Static Allocation**: Idle task uses static memory
- **Stack Size**: Minimal stack for idle task (configMINIMAL_STACK_SIZE)
- **Memory Efficiency**: Optimized for low-power operation

## Data Management Strategy

### 1. Sensor Data Pipeline
```
BMP280 Sensors → I2C Mux → Raw ADC → Compensation → Differential Calculation → Global Storage
```

### 2. Communication Data Flow
```
Global Data → Packet Creation → CRC Generation → COBS Encoding → UART Transmission
```

### 3. Command Processing
```
UART Reception → DMA Buffer → COBS Decoding → CRC Verification → Command Execution
```

### 4. Real-time Constraints
- **CAN**: 5ms transmission period
- **UART Log**: 500ms data logging
- **Sensor Read**: 100ms measurement cycle
- **Command Response**: Immediate processing

## Error Handling and Fault Tolerance

### 1. Sensor Failures
- Individual sensor failure detection
- Mock data generation for failed sensors
- System continues operation with available sensors

### 2. Communication Errors
- CRC16 verification for all transmitted/received data
- COBS encoding validation
- Timeout handling for I2C operations

### 3. Hardware Abstraction
- HAL library provides hardware abstraction
- Consistent error return codes
- Graceful degradation strategies

## Performance Characteristics

### Memory Usage
- **RAM**: ~2KB for task stacks + global variables
- **Flash**: ~50KB including HAL libraries and FreeRTOS
- **Stack per task**: 512 bytes

### Timing Performance
- **I2C Operations**: <10ms per sensor
- **CAN Transmission**: <1ms
- **UART Encoding**: <5ms
- **Task Switching**: <10μs (FreeRTOS overhead)

### Power Management
- **Low-power STM32L4**: Ultra-low power consumption
- **Sleep modes**: Between measurements
- **Peripheral gating**: Unused peripherals disabled

This documentation provides a comprehensive overview of the STM32 core source code design, focusing on the processes and data management strategies implemented in the Hylight-ET embedded system.


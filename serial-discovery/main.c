
/* Enhanced Serial Data Collector using CSSL Library
 * Supports various data types, frame detection, and robust data extraction
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>
#include "cssl.h"
#include "cobs_crc16.h"

/* Configuration structure */
typedef struct {
    char device[256];
    int baudrate;
    int databits;
    int parity;
    int stopbits;
    int rtscts;
    int xonxoff;
    cssl_data_type_t data_type;
    cssl_delimiter_t delimiter_type;
    char custom_delimiter[16];
    int delimiter_len;
    int frame_size;
    char output_file[256];
    int verbose;
    int hex_output;
    int statistics;
    int timeout;
    int max_packets;
    int duration;
    int auto_control;
    char control_type[16];
    int control_timeout;
} config_t;

/* Global variables */
static volatile int running = 1;
static cssl_t *serial_port = NULL;
static FILE *output_fp = NULL;
static int packet_count = 0;
static time_t start_time;
static config_t config;
static time_t last_control_time = 0;

/* Function prototypes */
void signal_handler(int sig);
void print_usage(const char *progname);
int parse_arguments(int argc, char *argv[], config_t *cfg);
void enhanced_data_callback(int id, uint8_t *buffer, int len, 
                           cssl_data_type_t type, struct timespec *timestamp);
void print_statistics(cssl_t *port);
void setup_output_file(const char *filename);
void cleanup_and_exit(int code);
int parse_delimiter(const char *delim_str, uint8_t *delimiter, int *len);
const char *data_type_to_string(cssl_data_type_t type);
const char *delimiter_type_to_string(cssl_delimiter_t type);
int validate_and_decode_tx_buffer(uint8_t *buffer, int len, raw_data_t *extracted_data);
void send_control_command(cssl_t *port, const char *control_type);
uint8_t get_command_value(const char *control_type);

int main(int argc, char *argv[])
{
    /* Initialize default configuration */
    memset(&config, 0, sizeof(config_t));
    strcpy(config.device, "/dev/ttyUSB0");
    config.baudrate = 9600;
    config.databits = 8;
    config.parity = 0;
    config.stopbits = 1;
    config.data_type = CSSL_DATA_RAW;
    config.delimiter_type = CSSL_DELIM_CUSTOM;
    config.custom_delimiter[0] = 0x00;
    config.delimiter_len = 1;
    config.timeout = 0; /* No timeout by default */
    config.max_packets = 0; /* No limit by default */
    config.duration = 0; /* No duration limit by default */
    config.auto_control = 0;
    config.control_timeout = 1000; /* Default 1 second */
    strcpy(config.control_type, "FON");

    /* Parse command line arguments */
    if (parse_arguments(argc, argv, &config) != 0) {
        print_usage(argv[0]);
        return 1;
    }

    /* Setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler); /* For statistics */

    /* Setup output file if specified */
    if (strlen(config.output_file) > 0) {
        setup_output_file(config.output_file);
    }

    printf("Enhanced Serial Data Collector\n");
    printf("==============================\n");
    printf("Device: %s\n", config.device);
    printf("Baudrate: %d\n", config.baudrate);
    printf("Data bits: %d, Parity: %d, Stop bits: %d\n", 
           config.databits, config.parity, config.stopbits);
    printf("Data type: %s\n", data_type_to_string(config.data_type));
    printf("Delimiter: %s\n", delimiter_type_to_string(config.delimiter_type));
    if (config.frame_size > 0) {
        printf("Fixed frame size: %d bytes\n", config.frame_size);
    }
    if (config.output_file[0]) {
        printf("Output file: %s\n", config.output_file);
    }
    if (config.auto_control) {
        printf("Auto-control: %s every %dms\n", config.control_type, config.control_timeout);
    }
    printf("Press Ctrl+C to stop, SIGUSR1 for statistics\n\n");

    /* Start CSSL library */
    cssl_start();
    if (cssl_geterror() != CSSL_OK) {
        fprintf(stderr, "Error starting CSSL: %s\n", cssl_geterrormsg());
        cleanup_and_exit(1);
    }

    /* Open serial port with enhanced callback */
    serial_port = cssl_open_enhanced(config.device, enhanced_data_callback, 1,
                                    config.baudrate, config.databits, 
                                    config.parity, config.stopbits,
                                    config.data_type);
    
    if (!serial_port) {
        fprintf(stderr, "Error opening serial port %s: %s\n", 
                config.device, cssl_geterrormsg());
        cleanup_and_exit(1);
    }

    /* Configure flow control */
    cssl_setflowcontrol(serial_port, config.rtscts, config.xonxoff);

    /* Configure delimiter and frame settings */
    if (config.delimiter_type != CSSL_DELIM_NONE) {
        uint8_t delim_bytes[8];
        int delim_len = config.delimiter_len;
        
        if (config.delimiter_type == CSSL_DELIM_CUSTOM) {
            memcpy(delim_bytes, config.custom_delimiter, delim_len);
        }
        
        cssl_set_delimiter(serial_port, config.delimiter_type, 
                          delim_bytes, delim_len);
    }

    if (config.frame_size > 0) {
        cssl_set_frame_size(serial_port, config.frame_size);
        cssl_set_data_type(serial_port, CSSL_DATA_FIXED_LENGTH);
    }

    /* Set timeout if specified */
    if (config.timeout > 0) {
        cssl_settimeout(serial_port, config.timeout);
    }

    start_time = time(NULL);
    last_control_time = start_time;
    printf("Listening for data...\n");

    /* Main loop */
    while (running) {
        /* Check duration limit */
        if (config.duration > 0) {
            if (time(NULL) - start_time >= config.duration) {
                printf("\nDuration limit reached (%d seconds)\n", config.duration);
                break;
            }
        }

        /* Check packet limit */
        if (config.max_packets > 0 && packet_count >= config.max_packets) {
            printf("\nPacket limit reached (%d packets)\n", config.max_packets);
            break;
        }

        /* Handle auto-control if enabled */
        if (config.auto_control) {
            time_t current_time = time(NULL);
            if (current_time - last_control_time >= (config.control_timeout / 1000)) {
                send_control_command(serial_port, config.control_type);
                last_control_time = current_time;
            }
        }

        /* Sleep briefly to prevent high CPU usage */
        usleep(10000); /* 10ms */
    }

    /* Print final statistics */
    printf("\nSession completed.\n");
    print_statistics(serial_port);

    cleanup_and_exit(0);
    return 0;
}

void enhanced_data_callback(int id, uint8_t *buffer, int len, 
                           cssl_data_type_t type, struct timespec *timestamp)
{
    static char time_str[64];
    struct tm *tm_info;
    time_t sec = timestamp->tv_sec;
    
    packet_count++;
    
    /* Format timestamp */
    tm_info = localtime(&sec);
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d.%03ld",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             timestamp->tv_nsec / 1000000);

    /* Print to console if verbose */
    if (config.verbose || !config.output_file[0]) {
        printf("[%s] Packet #%d (%s, %d bytes): ", 
               time_str, packet_count, data_type_to_string(type), len);
        
        if (config.hex_output || type == CSSL_DATA_BINARY) {
            /* Hex output */
            for (int i = 0; i < len; i++) {
                printf("%02X ", buffer[i]);
                if ((i + 1) % 16 == 0) printf("\n                           ");
            }
            printf("\n");
        } else {
            /* ASCII output with non-printable character filtering */
            for (int i = 0; i < len; i++) {
                if (buffer[i] >= 32 && buffer[i] <= 126) {
                    printf("%c", buffer[i]);
                } else if (buffer[i] == '\n') {
                    printf("\\n");
                } else if (buffer[i] == '\r') {
                    printf("\\r");
                } else if (buffer[i] == '\t') {
                    printf("\\t");
                } else {
                    printf("\\x%02X", buffer[i]);
                }
            }
            printf("\n");
        }
    }

    /* Write to output file if specified */
    if (output_fp) {
        fprintf(output_fp, "[%s] Packet #%d (%s, %d bytes): ",
                time_str, packet_count, data_type_to_string(type), len);
        
        if (config.hex_output || type == CSSL_DATA_BINARY) {
            for (int i = 0; i < len; i++) {
                fprintf(output_fp, "%02X ", buffer[i]);
            }
        } else {
            fwrite(buffer, 1, len, output_fp);
        }
        fprintf(output_fp, "\n");
        fflush(output_fp);
    }

    /* Try to validate and decode as TX buffer */
    raw_data_t extracted_data;
    if (validate_and_decode_tx_buffer(buffer, len, &extracted_data)) {
        if (config.verbose) {
            printf("                     -> Valid TX Buffer Decoded:\n");
            printf("                        Delta Pres 0: %d\n", extracted_data.delta_pres_0);
            printf("                        Delta Pres 1: %d\n", extracted_data.delta_pres_1);
            printf("                        Delta Pres 2: %d\n", extracted_data.delta_pres_2);
            printf("                        Fan RPM: %u\n", extracted_data.fan_rpm);
            printf("                        Status Flag: 0x%02X\n", extracted_data.status_flag);
        }
    }

    /* Extract specific data types */
    if (type == CSSL_DATA_ASCII && config.delimiter_type == CSSL_DELIM_NEWLINE) {
        char line[512];
        int extracted = cssl_extract_ascii_line(buffer, len, line, sizeof(line));
        if (extracted > 0 && config.verbose) {
            printf("                     -> Extracted line: \"%s\"\n", line);
        }
    }
}

void signal_handler(int sig)
{
    switch (sig) {
        case SIGINT:
        case SIGTERM:
            printf("\nReceived termination signal. Shutting down...\n");
            running = 0;
            break;
        case SIGUSR1:
            printf("\n--- Statistics ---\n");
            if (serial_port) {
                print_statistics(serial_port);
            }
            printf("Current packet count: %d\n", packet_count);
            printf("Session duration: %ld seconds\n", time(NULL) - start_time);
            printf("--- End Statistics ---\n");
            break;
    }
}

void print_statistics(cssl_t *port)
{
    if (!port) return;
    
    cssl_stats_t stats;
    cssl_get_stats(port, &stats);
    
    printf("=== Port Statistics ===\n");
    printf("Bytes received: %llu\n", (unsigned long long)stats.bytes_received);
    printf("Bytes sent: %llu\n", (unsigned long long)stats.bytes_sent);
    printf("Frames received: %u\n", stats.frames_received);
    printf("Frames sent: %u\n", stats.frames_sent);
    printf("Errors: %u\n", stats.errors);
    printf("Total packets processed: %d\n", packet_count);
    
    time_t session_time = time(NULL) - start_time;
    if (session_time > 0) {
        printf("Average bytes/second: %.2f\n", 
               (double)stats.bytes_received / session_time);
        printf("Average packets/second: %.2f\n", 
               (double)packet_count / session_time);
    }
    printf("=======================\n");
}

void setup_output_file(const char *filename)
{
    output_fp = fopen(filename, "w");
    if (!output_fp) {
        fprintf(stderr, "Warning: Could not open output file %s: %s\n",
                filename, strerror(errno));
        config.output_file[0] = '\0';
    } else {
        printf("Output will be written to: %s\n", filename);
        fprintf(output_fp, "# Enhanced Serial Data Collector Log\n");
        fprintf(output_fp, "# Device: %s, Baudrate: %d\n", 
                config.device, config.baudrate);
        fprintf(output_fp, "# Started at: %s", ctime(&start_time));
        fflush(output_fp);
    }
}

void cleanup_and_exit(int code)
{
    if (serial_port) {
        cssl_close(serial_port);
        serial_port = NULL;
    }
    
    if (output_fp) {
        fprintf(output_fp, "# Session ended at: %s", ctime(&(time_t){time(NULL)}));
        fclose(output_fp);
        output_fp = NULL;
    }
    
    cssl_stop();
    
    if (code == 0) {
        printf("Cleanup completed successfully.\n");
    }
    
    exit(code);
}

int parse_delimiter(const char *delim_str, uint8_t *delimiter, int *len)
{
    if (strncmp(delim_str, "0x", 2) == 0 || strncmp(delim_str, "\\x", 2) == 0) {
        /* Hex format */
        const char *hex = delim_str + 2;
        *len = 0;
        while (*hex && *(hex+1) && *len < 8) {
            unsigned int byte;
            if (sscanf(hex, "%2x", &byte) == 1) {
                delimiter[*len] = (uint8_t)byte;
                (*len)++;
                hex += 2;
            } else {
                return -1;
            }
        }
    } else {
        /* ASCII format with escape sequences */
        int i = 0, out = 0;
        while (delim_str[i] && out < 8) {
            if (delim_str[i] == '\\' && delim_str[i+1]) {
                switch (delim_str[i+1]) {
                    case 'n': delimiter[out++] = '\n'; i += 2; break;
                    case 'r': delimiter[out++] = '\r'; i += 2; break;
                    case 't': delimiter[out++] = '\t'; i += 2; break;
                    case '\\': delimiter[out++] = '\\'; i += 2; break;
                    default: delimiter[out++] = delim_str[i++]; break;
                }
            } else {
                delimiter[out++] = delim_str[i++];
            }
        }
        *len = out;
    }
    return 0;
}

const char *data_type_to_string(cssl_data_type_t type)
{
    switch (type) {
        case CSSL_DATA_RAW: return "RAW";
        case CSSL_DATA_ASCII: return "ASCII";
        case CSSL_DATA_BINARY: return "BINARY";
        case CSSL_DATA_FRAMED: return "FRAMED";
        case CSSL_DATA_FIXED_LENGTH: return "FIXED_LENGTH";
        default: return "UNKNOWN";
    }
}

const char *delimiter_type_to_string(cssl_delimiter_t type)
{
    switch (type) {
        case CSSL_DELIM_NONE: return "NONE";
        case CSSL_DELIM_NEWLINE: return "NEWLINE";
        case CSSL_DELIM_CRLF: return "CRLF";
        case CSSL_DELIM_CUSTOM: return "CUSTOM (0x00)";
        case CSSL_DELIM_LENGTH_PREFIX: return "LENGTH_PREFIX";
        default: return "UNKNOWN";
    }
}

int parse_arguments(int argc, char *argv[], config_t *cfg)
{
    int opt;
    static struct option long_options[] = {
        {"device", required_argument, 0, 'd'},
        {"baudrate", required_argument, 0, 'b'},
        {"databits", required_argument, 0, 'D'},
        {"parity", required_argument, 0, 'p'},
        {"stopbits", required_argument, 0, 's'},
        {"rtscts", no_argument, 0, 'r'},
        {"xonxoff", no_argument, 0, 'x'},
        {"type", required_argument, 0, 't'},
        {"delimiter", required_argument, 0, 'l'},
        {"frame-size", required_argument, 0, 'f'},
        {"output", required_argument, 0, 'o'},
        {"verbose", no_argument, 0, 'v'},
        {"hex", no_argument, 0, 'H'},
        {"statistics", no_argument, 0, 'S'},
        {"timeout", required_argument, 0, 'T'},
        {"max-packets", required_argument, 0, 'm'},
        {"duration", required_argument, 0, 'u'},
        {"control", required_argument, 0, 'c'},
        {"control-timeout", required_argument, 0, 'C'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "d:b:D:p:s:rxt:l:f:o:vHST:m:u:c:C:h", 
                              long_options, NULL)) != -1) {
        switch (opt) {
            case 'd':
                strncpy(cfg->device, optarg, sizeof(cfg->device) - 1);
                break;
            case 'b':
                cfg->baudrate = atoi(optarg);
                break;
            case 'D':
                cfg->databits = atoi(optarg);
                if (cfg->databits < 5 || cfg->databits > 8) {
                    fprintf(stderr, "Data bits must be 5-8\n");
                    return -1;
                }
                break;
            case 'p':
                cfg->parity = atoi(optarg);
                if (cfg->parity < 0 || cfg->parity > 2) {
                    fprintf(stderr, "Parity must be 0 (none), 1 (odd), or 2 (even)\n");
                    return -1;
                }
                break;
            case 's':
                cfg->stopbits = atoi(optarg);
                if (cfg->stopbits < 1 || cfg->stopbits > 2) {
                    fprintf(stderr, "Stop bits must be 1 or 2\n");
                    return -1;
                }
                break;
            case 'r':
                cfg->rtscts = 1;
                break;
            case 'x':
                cfg->xonxoff = 1;
                break;
            case 't':
                if (strcmp(optarg, "raw") == 0) {
                    cfg->data_type = CSSL_DATA_RAW;
                } else if (strcmp(optarg, "ascii") == 0) {
                    cfg->data_type = CSSL_DATA_ASCII;
                } else if (strcmp(optarg, "binary") == 0) {
                    cfg->data_type = CSSL_DATA_BINARY;
                } else if (strcmp(optarg, "framed") == 0) {
                    cfg->data_type = CSSL_DATA_FRAMED;
                } else {
                    fprintf(stderr, "Invalid data type: %s\n", optarg);
                    return -1;
                }
                break;
            case 'l':
                if (strcmp(optarg, "newline") == 0) {
                    cfg->delimiter_type = CSSL_DELIM_NEWLINE;
                } else if (strcmp(optarg, "crlf") == 0) {
                    cfg->delimiter_type = CSSL_DELIM_CRLF;
                } else {
                    cfg->delimiter_type = CSSL_DELIM_CUSTOM;
                    if (parse_delimiter(optarg, (uint8_t*)cfg->custom_delimiter, 
                                       &cfg->delimiter_len) != 0) {
                        fprintf(stderr, "Invalid custom delimiter: %s\n", optarg);
                        return -1;
                    }
                }
                if (cfg->data_type == CSSL_DATA_RAW) {
                    cfg->data_type = CSSL_DATA_FRAMED;
                }
                break;
            case 'f':
                cfg->frame_size = atoi(optarg);
                if (cfg->frame_size <= 0 || cfg->frame_size > CSSL_MAX_FRAME_SIZE) {
                    fprintf(stderr, "Frame size must be 1-%d\n", CSSL_MAX_FRAME_SIZE);
                    return -1;
                }
                break;
            case 'o':
                strncpy(cfg->output_file, optarg, sizeof(cfg->output_file) - 1);
                break;
            case 'v':
                cfg->verbose = 1;
                break;
            case 'H':
                cfg->hex_output = 1;
                break;
            case 'S':
                cfg->statistics = 1;
                break;
            case 'T':
                cfg->timeout = atoi(optarg);
                break;
            case 'm':
                cfg->max_packets = atoi(optarg);
                break;
            case 'u':
                cfg->duration = atoi(optarg);
                break;
            case 'c':
                cfg->auto_control = 1;
                if (strcmp(optarg, "FON") == 0 || strcmp(optarg, "FOFF") == 0 || strcmp(optarg, "FOA") == 0) {
                    strncpy(cfg->control_type, optarg, sizeof(cfg->control_type) - 1);
                } else {
                    fprintf(stderr, "Invalid control type: %s (use FON, FOFF, or FOA)\n", optarg);
                    return -1;
                }
                break;
            case 'C':
                cfg->control_timeout = atoi(optarg);
                if (cfg->control_timeout < 100) {
                    fprintf(stderr, "Control timeout must be at least 100ms\n");
                    return -1;
                }
                break;
            case 'h':
                print_usage(argv[0]);
                exit(0);
            default:
                return -1;
        }
    }

    return 0;
}

void print_usage(const char *progname)
{
    printf("Enhanced Serial Data Collector\n");
    printf("Usage: %s [options]\n\n", progname);
    printf("Connection Options:\n");
    printf("  -d, --device <device>     Serial device (default: /dev/ttyUSB0)\n");
    printf("  -b, --baudrate <rate>     Baudrate (default: 9600)\n");
    printf("  -D, --databits <bits>     Data bits: 5-8 (default: 8)\n");
    printf("  -p, --parity <parity>     Parity: 0=none, 1=odd, 2=even (default: 0)\n");
    printf("  -s, --stopbits <bits>     Stop bits: 1-2 (default: 1)\n");
    printf("  -r, --rtscts              Enable RTS/CTS flow control\n");
    printf("  -x, --xonxoff             Enable XON/XOFF flow control\n\n");
    
    printf("Data Processing Options:\n");
    printf("  -t, --type <type>         Data type: raw, ascii, binary, framed (default: raw)\n");
    printf("  -l, --delimiter <delim>   Frame delimiter: newline, crlf, or custom\n");
    printf("                            Custom format: \"\\n\" or \"0x0A\" or \"ABC\"\n");
    printf("                            (default: 0x00)\n");
    printf("  -f, --frame-size <size>   Fixed frame size in bytes\n\n");
    
    printf("Output Options:\n");
    printf("  -o, --output <file>       Output file (default: stdout)\n");
    printf("  -v, --verbose             Verbose output\n");
    printf("  -H, --hex                 Hex output format\n");
    printf("  -S, --statistics          Show periodic statistics\n\n");
    
    printf("Control Options:\n");
    printf("  -T, --timeout <decisec>   Read timeout in deciseconds\n");
    printf("  -m, --max-packets <num>   Maximum packets to collect\n");
    printf("  -u, --duration <seconds>  Collection duration in seconds\n");
    printf("  -c, --control <type>      Auto-control mode: FON, FOFF, FOA\n");
    printf("  -C, --control-timeout <ms> Control command interval in milliseconds\n");
    printf("  -h, --help                Show this help\n\n");
    
    printf("Examples:\n");
    printf("  %s -d /dev/ttyUSB0 -b 115200 -t ascii -l newline -v\n", progname);
    printf("  %s -d /dev/ttyS0 -b 9600 -t binary -f 32 -o data.log\n", progname);
    printf("  %s -d /dev/ttyACM0 -t framed -l \"0x0D0A\" -H\n", progname);
    printf("  %s -d /dev/ttyUSB0 -b 9600 -c FON -C 2000 -v  # Auto-control every 2s\n", progname);
    printf("\nSignals:\n");
    printf("  CTRL+C / SIGTERM - Stop collection\n");
    printf("  SIGUSR1 - Display current statistics\n");
}

int validate_and_decode_tx_buffer(uint8_t *buffer, int len, raw_data_t *extracted_data)
{
    if (!buffer || !extracted_data || len == 0) {
        return 0;
    }

    /* Check if buffer ends with 0x00 delimiter */
    if (buffer[len - 1] != 0x00) {
        return 0;
    }

    /* Try to decode as COBS encoded TX buffer */
    uint8_t decoded_buffer[sizeof(uart_tx_logging_t) + sizeof(uint16_t) + 10];
    size_t decoded_size = decode_cobs_data(buffer, len, decoded_buffer, sizeof(decoded_buffer));
    
    if (decoded_size < sizeof(uart_tx_logging_t)) {
        return 0; /* Not enough data for TX packet */
    }

    /* Extract TX packet */
    uart_tx_logging_t tx_packet;
    memcpy(&tx_packet, decoded_buffer, sizeof(uart_tx_logging_t));

    /* Verify CRC16 of the live data */
    uint8_t *data_ptr = (uint8_t *)&tx_packet.live_data;
    uint16_t calculated_crc = calculate_crc16(data_ptr, sizeof(raw_data_t));
    
    if (calculated_crc != tx_packet.CRC16) {
        return 0; /* CRC mismatch */
    }

    /* Additional packet-level CRC check if present */
    if (decoded_size >= sizeof(uart_tx_logging_t) + sizeof(uint16_t)) {
        uint16_t packet_crc;
        memcpy(&packet_crc, decoded_buffer + sizeof(uart_tx_logging_t), sizeof(uint16_t));
        
        uint16_t calculated_packet_crc = calculate_crc16(decoded_buffer, sizeof(uart_tx_logging_t));
        
        if (calculated_packet_crc != packet_crc) {
            return 0; /* Packet CRC mismatch */
        }
    }

    /* Extract the raw data */
    *extracted_data = tx_packet.live_data;
    return 1; /* Valid TX buffer */
}

uint8_t get_command_value(const char *control_type)
{
    if (strcmp(control_type, "FON") == 0) {
        return CFF_ON;
    } else if (strcmp(control_type, "FOFF") == 0) {
        return CFF_OFF;
    } else if (strcmp(control_type, "FOA") == 0) {
        return CFF_AUTO;
    }
    return 0; /* Invalid */
}

void send_control_command(cssl_t *port, const char *control_type)
{
    if (!port || !control_type) {
        return;
    }

    uint8_t command_value = get_command_value(control_type);
    if (command_value == 0) {
        fprintf(stderr, "Invalid control type: %s\n", control_type);
        return;
    }

    /* Create RX command structure */
    uart_rx_command_t rx_command;
    rx_command.commandvalue = command_value;
    
    /* Calculate CRC for the command value */
    rx_command.CRC16 = calculate_crc16(&rx_command.commandvalue, sizeof(uint8_t));

    /* Convert to byte array */
    uint8_t raw_bytes[sizeof(uart_rx_command_t)];
    memcpy(raw_bytes, &rx_command, sizeof(uart_rx_command_t));

    /* Calculate packet CRC */
    uint16_t packet_crc = calculate_crc16(raw_bytes, sizeof(uart_rx_command_t));

    /* Create extended packet with packet CRC */
    uint8_t extended_packet[sizeof(uart_rx_command_t) + sizeof(uint16_t)];
    memcpy(extended_packet, raw_bytes, sizeof(uart_rx_command_t));
    memcpy(extended_packet + sizeof(uart_rx_command_t), &packet_crc, sizeof(uint16_t));

    /* COBS encode the command */
    uint8_t encoded_buffer[sizeof(extended_packet) * 2 + 10];
    size_t encoded_size = cobs_encode(extended_packet, sizeof(extended_packet), 
                                     encoded_buffer, sizeof(encoded_buffer));

    if (encoded_size > 0) {
        /* Send the encoded command */
        cssl_putdata(port, encoded_buffer, encoded_size);
        
        if (config.verbose) {
            printf("Sent control command: %s (0x%02X) [%zu bytes encoded]\n", 
                   control_type, command_value, encoded_size);
        }
    } else {
        fprintf(stderr, "Failed to encode control command\n");
    }
}


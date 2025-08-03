
/* Copyright 2003 Marcin Siennicki <m.siennicki@cloos.pl>
 * Enhanced version with optimized event mechanisms and data extraction
 * see COPYING file for details */

#ifndef __CSSL_H__
#define __CSSL_H__

#include <stdint.h>
#include <signal.h>
#include <termios.h>
#include <time.h>

/* Enhanced buffer size for better data handling */
#define CSSL_BUFFER_SIZE 2048
#define CSSL_MAX_FRAME_SIZE 512
#define CSSL_FRAME_TIMEOUT_MS 100

/* Data types for enhanced processing */
typedef enum {
    CSSL_DATA_RAW,           /* Raw binary data */
    CSSL_DATA_ASCII,         /* ASCII text data */
    CSSL_DATA_BINARY,        /* Binary protocol data */
    CSSL_DATA_FRAMED,        /* Framed data with delimiters */
    CSSL_DATA_FIXED_LENGTH   /* Fixed length packets */
} cssl_data_type_t;

/* Frame delimiter types */
typedef enum {
    CSSL_DELIM_NONE,         /* No delimiter */
    CSSL_DELIM_NEWLINE,      /* \n delimiter */
    CSSL_DELIM_CRLF,         /* \r\n delimiter */
    CSSL_DELIM_CUSTOM,       /* Custom delimiter */
    CSSL_DELIM_LENGTH_PREFIX /* Length-prefixed frames */
} cssl_delimiter_t;

/* Enhanced callback with data type information */
typedef void (*cssl_enhanced_callback_t)(int id,           /* port id */
                                        uint8_t *buffer,    /* data received */
                                        int len,            /* length of data */
                                        cssl_data_type_t type, /* data type */
                                        struct timespec *timestamp); /* timestamp */

/* Legacy callback for backward compatibility */
typedef void (*cssl_callback_t)(int id,  /* id passed to callback */
                               uint8_t *buffer, /* data received */
                               int len); /* length of data in bytes */

/* Enhanced statistics structure */
typedef struct {
    uint64_t bytes_received;
    uint64_t bytes_sent;
    uint32_t frames_received;
    uint32_t frames_sent;
    uint32_t errors;
    struct timespec last_activity;
} cssl_stats_t;

/* Enhanced CSSL structure */
typedef struct __cssl_t {
    /* Basic structure - kept for compatibility */
    uint8_t buffer[CSSL_BUFFER_SIZE];    /* Enhanced input buffer */
    int fd;                              /* tty file descriptor */
    struct termios tio;                  /* termios structure for the port */
    struct termios oldtio;               /* old termios structure */
    cssl_callback_t callback;            /* legacy callback function */
    int id;                              /* id which would be passed to callback */
    struct __cssl_t *next;

    /* Enhanced features */
    cssl_enhanced_callback_t enhanced_callback; /* Enhanced callback */
    cssl_data_type_t data_type;          /* Expected data type */
    cssl_delimiter_t delimiter_type;     /* Frame delimiter type */
    uint8_t custom_delimiter[8];         /* Custom delimiter bytes */
    int delimiter_len;                   /* Length of custom delimiter */
    int frame_size;                      /* Fixed frame size (if applicable) */
    
    /* Frame assembly */
    uint8_t frame_buffer[CSSL_MAX_FRAME_SIZE]; /* Frame assembly buffer */
    int frame_pos;                       /* Current position in frame */
    struct timespec frame_start_time;    /* Frame assembly start time */
    
    /* Statistics */
    cssl_stats_t stats;                  /* Port statistics */
    
    /* Buffer management */
    int buffer_pos;                      /* Current buffer position */
    int min_read_size;                   /* Minimum read size for efficiency */
    int max_read_size;                   /* Maximum read size */
    
} cssl_t;

typedef enum {
    CSSL_OK,                 /* everything is all right */
    CSSL_ERROR_NOSIGNAL,     /* there's no free signal */
    CSSL_ERROR_NOTSTARTED,   /* you should first start cssl */
    CSSL_ERROR_NULLPOINTER,  /* you gave a null pointer to the function */
    CSSL_ERROR_OOPS,         /* internal error, something's wrong */
    CSSL_ERROR_MEMORY,       /* there's no memory for cssl_t structure */
    CSSL_ERROR_OPEN,         /* file doesn't exist or you aren't good user */
    CSSL_ERROR_TIMEOUT,      /* frame assembly timeout */
    CSSL_ERROR_FRAME_TOO_LARGE, /* frame exceeds maximum size */
    CSSL_ERROR_INVALID_DELIMITER /* invalid delimiter configuration */
} cssl_error_t;

/* Basic functions - kept for compatibility */
const char *cssl_geterrormsg();
int cssl_geterror();
void cssl_start();
void cssl_stop();

cssl_t *cssl_open(const char *fname, cssl_callback_t callback, int id,
                  int baud, int bits, int parity, int stop);
void cssl_close(cssl_t *serial);
void cssl_setup(cssl_t *serial, int baud, int bits, int parity, int stop);
void cssl_setflowcontrol(cssl_t *serial, int rtscts, int xonxoff);
void cssl_settimeout(cssl_t *serial, int timeout);

void cssl_putchar(cssl_t *serial, char c);
void cssl_putstring(cssl_t *serial, char *str);
void cssl_putdata(cssl_t *serial, uint8_t *data, int datalen);
void cssl_drain(cssl_t *serial);

int cssl_getchar(cssl_t *serial);
int cssl_getdata(cssl_t *serial, uint8_t *buffer, int size);

/* Enhanced functions */
cssl_t *cssl_open_enhanced(const char *fname, cssl_enhanced_callback_t callback,
                          int id, int baud, int bits, int parity, int stop,
                          cssl_data_type_t data_type);

void cssl_set_data_type(cssl_t *serial, cssl_data_type_t type);
void cssl_set_delimiter(cssl_t *serial, cssl_delimiter_t delim_type, 
                       const uint8_t *custom_delim, int delim_len);
void cssl_set_frame_size(cssl_t *serial, int frame_size);
void cssl_set_read_params(cssl_t *serial, int min_size, int max_size);

/* Statistics and monitoring */
void cssl_get_stats(cssl_t *serial, cssl_stats_t *stats);
void cssl_reset_stats(cssl_t *serial);

/* Data extraction utilities */
int cssl_extract_ascii_line(const uint8_t *buffer, int len, char *output, int max_len);
int cssl_extract_binary_frame(const uint8_t *buffer, int len, 
                             const uint8_t *start_seq, int start_len,
                             const uint8_t *end_seq, int end_len,
                             uint8_t *output, int max_len);
int cssl_extract_length_prefixed(const uint8_t *buffer, int len, 
                                uint8_t *output, int max_len);

/* Utility functions */
void cssl_hex_dump(const uint8_t *data, int len, const char *prefix);
int cssl_validate_frame(const uint8_t *frame, int len, uint16_t expected_crc);
uint16_t cssl_calculate_crc16(const uint8_t *data, int len);

#endif /* __CSSL_H__ */


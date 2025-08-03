
/* Copyright 2003 Marcin Siennicki <m.siennicki@cloos.pl>
 * Enhanced version with optimized event mechanisms and data extraction
 * see COPYING file for details */

#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include "cssl.h"

/*
 * Static variables and constants
 */

/* signal number for serial i/o read */
static int CSSL_SIGNAL=0;

/* boolean that say if we have started cssl */
static int cssl_started=0;

/* sigactions */
static struct sigaction sa;
static struct sigaction oldsa;

/* head of the cssl_t list */
static cssl_t *head=0;

/* error messages table */
static const char *cssl_errors[]= {
    "cssl: OK",
    "cssl: there's no free signal",
    "cssl: not started",
    "cssl: null pointer",
    "cssl: internal error, something's wrong",
    "cssl: out of memory",
    "cssl: cannot open file",
    "cssl: frame assembly timeout",
    "cssl: frame too large",
    "cssl: invalid delimiter configuration"
};

/* status of last cssl function */ 
static cssl_error_t cssl_error=CSSL_OK;

/* prototype of signal handler */
static void cssl_handler(int signo, siginfo_t *info, void *ignored);

/* Enhanced internal functions */
static int process_received_data(cssl_t *serial, uint8_t *data, int len);
static int assemble_frame(cssl_t *serial, uint8_t *data, int len);
static int check_frame_timeout(cssl_t *serial);
static void update_stats(cssl_t *serial, int bytes_received);
static cssl_data_type_t detect_data_type(const uint8_t *data, int len);

/**************************************
 * Public functions
 **************************************/

/*-------------------------------------
 * Error handling
 */

const char *cssl_geterrormsg()
{
    return cssl_errors[cssl_error];
}

int cssl_geterror()
{
    return cssl_error;
}

/*-------------------------------------
 * Starting/stopping cssl
 */

void cssl_start()
{
    int sig;

    if (cssl_started) {
        return;
    }

    /* Here we scan for unused real time signal */
    sig=SIGRTMIN;

    do {
        /* get old sigaction */
        sigaction(sig,0,&oldsa);
        
        /* if signal's handler is empty */
        if (oldsa.sa_handler == 0) 
        {
            /* set the signal handler, and others */
            CSSL_SIGNAL=sig;
            sa.sa_sigaction = cssl_handler;
            sa.sa_flags = SA_SIGINFO;
            sa.sa_restorer = NULL;
            sigemptyset(&sa.sa_mask);
            sigaction(CSSL_SIGNAL,&sa,0);

            /* OK, the cssl is started */
            cssl_started=1;
            cssl_error=CSSL_OK;
            return;
        } else {
            /* signal handler was not empty, restore original */
            sigaction(CSSL_SIGNAL,&oldsa,0);
        }
        sig++;
    } while(sig<=SIGRTMAX);
    
    /* Sorry, there's no free signal */
    cssl_error=CSSL_ERROR_NOSIGNAL;
}

void cssl_stop()
{
    /* if not started we do nothing */
    if (!cssl_started)
        return;

    /* we close all ports, and free the list */
    while (head)
        cssl_close(head);

    /* then we remove the signal handler */
    sigaction(CSSL_SIGNAL,&oldsa,NULL);

    /* And at last : */
    cssl_started=0;
    cssl_error=CSSL_OK;
}

/*-------------------------------------
 * Basic port operation - open/close
 */

cssl_t *cssl_open(const char *fname, cssl_callback_t callback, int id,
                  int baud, int bits, int parity, int stop)
{
    return cssl_open_enhanced(fname, NULL, id, baud, bits, parity, stop, CSSL_DATA_RAW);
}

cssl_t *cssl_open_enhanced(const char *fname, cssl_enhanced_callback_t callback,
                          int id, int baud, int bits, int parity, int stop,
                          cssl_data_type_t data_type)
{
    cssl_t *serial;

    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return NULL;
    }
    
    /* create new cssl_t structure */
    serial=calloc(1,sizeof(cssl_t));

    /* oops, no memory */
    if (!serial) {
        cssl_error=CSSL_ERROR_MEMORY;
        return NULL;
    }

    /* Initialize enhanced fields */
    serial->enhanced_callback = callback;
    serial->data_type = data_type;
    serial->delimiter_type = CSSL_DELIM_NONE;
    serial->frame_size = 0;
    serial->buffer_pos = 0;
    serial->frame_pos = 0;
    serial->min_read_size = 1;
    serial->max_read_size = CSSL_BUFFER_SIZE / 4;
    memset(&serial->stats, 0, sizeof(cssl_stats_t));
    clock_gettime(CLOCK_MONOTONIC, &serial->stats.last_activity);

    /* opening the file */
    if(callback) {
        /* user wants event driven reading */
        serial->fd=open(fname,O_RDWR|O_NOCTTY|O_NONBLOCK);
        fcntl(serial->fd,F_SETSIG,CSSL_SIGNAL);
        fcntl(serial->fd,F_SETOWN,getpid());
        fcntl(serial->fd,F_SETFL,O_ASYNC|O_NONBLOCK);
    } else {
        /* the read/write operations will be blocking */
        serial->fd=open(fname,O_RDWR|O_NOCTTY);
    }

    /* oops, cannot open */
    if (serial->fd == -1) {
        cssl_error=CSSL_ERROR_OPEN;
        free(serial);
        return NULL;
    }

    /* we remember old termios */
    tcgetattr(serial->fd,&(serial->oldtio));
    
    /* now we set new values */
    cssl_setup(serial,baud,bits,parity,stop);

    /* and id */
    serial->id=id;
    
    /* we add the serial to our list */
    serial->next=head;
    head=serial;
        
    cssl_error=CSSL_OK;
    return serial;
}

void cssl_close(cssl_t *serial)
{
    cssl_t *cur;
    
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    /* first we flush the port */
    tcflush(serial->fd,TCOFLUSH); 
    tcflush(serial->fd,TCIFLUSH); 
    
    /* then we restore old settings */
    tcsetattr(serial->fd,TCSANOW,&(serial->oldtio));
    
    /* and close the file */
    close(serial->fd);
    
    /* now we can remove the serial from the list */
    if (head==serial) {
        head=serial->next;
        free(serial);
        cssl_error=CSSL_OK;
        return;
    }

    for (cur=head;cur;cur=cur->next) {
        if (cur->next==serial) {
            cur->next=serial->next;
            free(serial);
            cssl_error=CSSL_OK;
            return;
        }
    }

    /* we should never reach here */
    cssl_error=CSSL_ERROR_OOPS;
}

/*-------------------------------------
 * Port setup
 */

void cssl_setup(cssl_t *serial, int baud, int bits, int parity, int stop)
{
    tcflag_t baudrate;
    tcflag_t databits;
    tcflag_t stopbits;
    tcflag_t checkparity;

    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    /* get the proper baudrate */
    switch (baud) {
    case 75: baudrate=B75; break;
    case 110: baudrate=B110; break;
    case 150: baudrate=B150; break;
    case 300: baudrate=B300; break;
    case 600: baudrate=B600; break;
    case 1200: baudrate=B1200; break;
    case 2400: baudrate=B2400; break;
    case 4800: baudrate=B4800; break;
    case 9600: baudrate=B9600; break;
    case 19200: baudrate=B19200; break;
    case 38400: baudrate=B38400; break;
    case 57600: baudrate=B57600; break;
    case 115200: baudrate=B115200; break;
    case 230400: baudrate=B230400; break;
    case 460800: baudrate=B460800; break;
    case 500000: baudrate=B500000; break;
    case 576000: baudrate=B576000; break;
    case 921600: baudrate=B921600; break;
    case 1000000: baudrate=B1000000; break;
    default: baudrate=B9600;
    }

    /* databits */
    switch (bits) {
    case 5: databits=CS5; break;
    case 6: databits=CS6; break;
    case 7: databits=CS7; break;
    case 8: databits=CS8; break;
    default: databits=CS8;
    }
    
    /* parity */
    switch (parity) {
    case 0: checkparity=0; break;
    case 1: checkparity=PARENB|PARODD; break;  // odd
    case 2: checkparity=PARENB; break;         // even
    default: checkparity=0;
    }
    
    /* stop bits */
    switch (stop) {
    case 1: stopbits=0; break;
    case 2: stopbits=CSTOPB; break;
    default: stopbits=0;
    }
    
    /* now we setup the values in port's termios */
    serial->tio.c_cflag=baudrate|databits|checkparity|stopbits|CLOCAL|CREAD;
    serial->tio.c_iflag=IGNPAR;
    serial->tio.c_oflag=0;
    serial->tio.c_lflag=0;
    serial->tio.c_cc[VMIN]=1;
    serial->tio.c_cc[VTIME]=0;

    /* we flush the port */
    tcflush(serial->fd,TCOFLUSH);
    tcflush(serial->fd,TCIFLUSH);
    
    /* we send new config to the port */
    tcsetattr(serial->fd,TCSANOW,&(serial->tio));

    cssl_error=CSSL_OK;
}

void cssl_setflowcontrol(cssl_t *serial, int rtscts, int xonxoff)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    /* We setup rts/cts (hardware) flow control */
    if (rtscts) {
        serial->tio.c_cflag |= CRTSCTS;
    } else {
        serial->tio.c_cflag &= ~CRTSCTS;
    }
    
    /* We setup xon/xoff (soft) flow control */
    if (xonxoff) {
        serial->tio.c_iflag |= (IXON|IXOFF);
    } else {
        serial->tio.c_iflag &= ~(IXON|IXOFF);
    }
    
    tcsetattr(serial->fd,TCSANOW,&(serial->tio));
    cssl_error=CSSL_OK;
}

void cssl_settimeout(cssl_t *serial, int timeout)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    serial->tio.c_cc[VTIME]=timeout;
    tcsetattr(serial->fd,TCSANOW,&(serial->tio));
    cssl_error=CSSL_OK;
}

/*-------------------------------------
 * Enhanced configuration functions
 */

void cssl_set_data_type(cssl_t *serial, cssl_data_type_t type)
{
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    serial->data_type = type;
    cssl_error=CSSL_OK;
}

void cssl_set_delimiter(cssl_t *serial, cssl_delimiter_t delim_type, 
                       const uint8_t *custom_delim, int delim_len)
{
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    serial->delimiter_type = delim_type;
    
    if (delim_type == CSSL_DELIM_CUSTOM) {
        if (!custom_delim || delim_len <= 0 || delim_len > 8) {
            cssl_error=CSSL_ERROR_INVALID_DELIMITER;
            return;
        }
        memcpy(serial->custom_delimiter, custom_delim, delim_len);
        serial->delimiter_len = delim_len;
    }
    
    cssl_error=CSSL_OK;
}

void cssl_set_frame_size(cssl_t *serial, int frame_size)
{
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    if (frame_size > CSSL_MAX_FRAME_SIZE) {
        cssl_error=CSSL_ERROR_FRAME_TOO_LARGE;
        return;
    }
    
    serial->frame_size = frame_size;
    cssl_error=CSSL_OK;
}

void cssl_set_read_params(cssl_t *serial, int min_size, int max_size)
{
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    serial->min_read_size = min_size > 0 ? min_size : 1;
    serial->max_read_size = max_size > 0 ? max_size : CSSL_BUFFER_SIZE / 4;
    cssl_error=CSSL_OK;
}

/*-------------------------------------
 * Serial communication
 */

void cssl_putchar(cssl_t *serial, char c)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    if (write(serial->fd,&c,1) == 1) {
        serial->stats.bytes_sent++;
    }
}

void cssl_putstring(cssl_t *serial, char *str)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    
    
    int len = strlen(str);
    int written = write(serial->fd,str,len);
    if (written > 0) {
        serial->stats.bytes_sent += written;
    }
}

void cssl_putdata(cssl_t *serial, uint8_t *data, int datalen)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    int written = write(serial->fd,data,datalen);
    if (written > 0) {
        serial->stats.bytes_sent += written;
        serial->stats.frames_sent++;
    }
}

void cssl_drain(cssl_t *serial)
{
    if (!cssl_started) {
        cssl_error=CSSL_ERROR_NOTSTARTED;
        return;
    }
    
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }    

    tcdrain(serial->fd);
}

int cssl_getchar(cssl_t *serial)
{
    int result;
    uint8_t c;
    
    result=read(serial->fd,&c,sizeof(c));
    if (result<=0)
        return -1;
    
    update_stats(serial, 1);
    return c;
}

int cssl_getdata(cssl_t *serial, uint8_t *buffer, int size)
{
    int result = read(serial->fd,buffer,size);
    if (result > 0) {
        update_stats(serial, result);
    }
    return result;
}

/*-------------------------------------
 * Statistics and monitoring
 */

void cssl_get_stats(cssl_t *serial, cssl_stats_t *stats)
{
    if (!serial || !stats) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    memcpy(stats, &serial->stats, sizeof(cssl_stats_t));
    cssl_error=CSSL_OK;
}

void cssl_reset_stats(cssl_t *serial)
{
    if (!serial) {
        cssl_error=CSSL_ERROR_NULLPOINTER;
        return;
    }
    
    memset(&serial->stats, 0, sizeof(cssl_stats_t));
    clock_gettime(CLOCK_MONOTONIC, &serial->stats.last_activity);
    cssl_error=CSSL_OK;
}

/*-------------------------------------
 * Data extraction utilities
 */

int cssl_extract_ascii_line(const uint8_t *buffer, int len, char *output, int max_len)
{
    int i, out_pos = 0;
    
    for (i = 0; i < len && out_pos < max_len - 1; i++) {
        if (buffer[i] == '\n' || buffer[i] == '\r') {
            output[out_pos] = '\0';
            return i + 1; /* Return bytes consumed */
        }
        if (buffer[i] >= 32 && buffer[i] <= 126) { /* Printable ASCII */
            output[out_pos++] = buffer[i];
        }
    }
    
    if (out_pos < max_len) {
        output[out_pos] = '\0';
    }
    return i;
}

int cssl_extract_binary_frame(const uint8_t *buffer, int len, 
                             const uint8_t *start_seq, int start_len,
                             const uint8_t *end_seq, int end_len,
                             uint8_t *output, int max_len)
{
    int i, j;
    int start_found = 0, start_pos = 0;
    
    /* Find start sequence */
    for (i = 0; i <= len - start_len; i++) {
        if (memcmp(&buffer[i], start_seq, start_len) == 0) {
            start_found = 1;
            start_pos = i + start_len;
            break;
        }
    }
    
    if (!start_found) return 0;
    
    /* Find end sequence */
    for (i = start_pos; i <= len - end_len; i++) {
        if (memcmp(&buffer[i], end_seq, end_len) == 0) {
            int frame_len = i - start_pos;
            if (frame_len <= max_len) {
                memcpy(output, &buffer[start_pos], frame_len);
                return i + end_len; /* Return total bytes consumed */
            }
        }
    }
    
    return 0; /* Frame not complete */
}

int cssl_extract_length_prefixed(const uint8_t *buffer, int len, 
                                uint8_t *output, int max_len)
{
    if (len < 2) return 0; /* Need at least length byte */
    
    int frame_len = (buffer[0] << 8) | buffer[1]; /* Big-endian length */
    
    if (len < frame_len + 2) return 0; /* Frame not complete */
    if (frame_len > max_len) return -1; /* Frame too large */
    
    memcpy(output, &buffer[2], frame_len);
    return frame_len + 2; /* Return total bytes consumed */
}

/*-------------------------------------
 * Utility functions
 */

void cssl_hex_dump(const uint8_t *data, int len, const char *prefix)
{
    int i;
    printf("%s", prefix ? prefix : "");
    for (i = 0; i < len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n%s", prefix ? prefix : "");
    }
    if (len % 16 != 0) printf("\n");
}

uint16_t cssl_calculate_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    int i, j;
    
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int cssl_validate_frame(const uint8_t *frame, int len, uint16_t expected_crc)
{
    if (len < 2) return 0;
    uint16_t calculated = cssl_calculate_crc16(frame, len - 2);
    return calculated == expected_crc;
}

/*-------------------------------------
 * Internal helper functions
 */

static void update_stats(cssl_t *serial, int bytes_received)
{
    serial->stats.bytes_received += bytes_received;
    clock_gettime(CLOCK_MONOTONIC, &serial->stats.last_activity);
}

static cssl_data_type_t detect_data_type(const uint8_t *data, int len)
{
    int i, ascii_count = 0;
    
    for (i = 0; i < len; i++) {
        if ((data[i] >= 32 && data[i] <= 126) || data[i] == '\t' || 
            data[i] == '\n' || data[i] == '\r') {
            ascii_count++;
        }
    }
    
    if (ascii_count > len * 0.8) {
        return CSSL_DATA_ASCII;
    }
    return CSSL_DATA_BINARY;
}

static int process_received_data(cssl_t *serial, uint8_t *data, int len)
{
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    
    update_stats(serial, len);
    
    /* Handle different data types */
    switch (serial->data_type) {
        case CSSL_DATA_FRAMED:
            return assemble_frame(serial, data, len);
            
        case CSSL_DATA_FIXED_LENGTH:
            if (serial->frame_size > 0) {
                /* Accumulate until we have a complete frame */
                int space = serial->frame_size - serial->frame_pos;
                int to_copy = (len < space) ? len : space;
                
                memcpy(&serial->frame_buffer[serial->frame_pos], data, to_copy);
                serial->frame_pos += to_copy;
                
                if (serial->frame_pos >= serial->frame_size) {
                    /* Complete frame received */
                    if (serial->enhanced_callback) {
                        serial->enhanced_callback(serial->id, serial->frame_buffer, 
                                                serial->frame_size, serial->data_type, &timestamp);
                    }
                    serial->stats.frames_received++;
                    serial->frame_pos = 0;
                }
                return to_copy;
            }
            break;
            
        default:
            /* Raw data or auto-detected type */
            if (serial->data_type == CSSL_DATA_RAW) {
                cssl_data_type_t detected = detect_data_type(data, len);
                if (serial->enhanced_callback) {
                    serial->enhanced_callback(serial->id, data, len, detected, &timestamp);
                }
            } else {
                if (serial->enhanced_callback) {
                    serial->enhanced_callback(serial->id, data, len, serial->data_type, &timestamp);
                }
            }
            serial->stats.frames_received++;
            break;
    }
    
    return len;
}

static int assemble_frame(cssl_t *serial, uint8_t *data, int len)
{
    int i, consumed = 0;
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    /* Check for frame timeout */
    if (serial->frame_pos > 0) {
        long timeout_ms = (current_time.tv_sec - serial->frame_start_time.tv_sec) * 1000 +
                         (current_time.tv_nsec - serial->frame_start_time.tv_nsec) / 1000000;
        if (timeout_ms > CSSL_FRAME_TIMEOUT_MS) {
            /* Frame timeout - reset */
            serial->frame_pos = 0;
        }
    }
    
    for (i = 0; i < len; i++) {
        /* Start new frame if needed */
        if (serial->frame_pos == 0) {
            serial->frame_start_time = current_time;
        }
        
        /* Add byte to frame buffer */
        if (serial->frame_pos < CSSL_MAX_FRAME_SIZE) {
            serial->frame_buffer[serial->frame_pos++] = data[i];
            consumed++;
        } else {
            /* Frame too large - reset */
            serial->frame_pos = 0;
            serial->stats.errors++;
            continue;
        }
        
        /* Check for frame delimiter */
        int frame_complete = 0;
        switch (serial->delimiter_type) {
            case CSSL_DELIM_NEWLINE:
                if (data[i] == '\n') frame_complete = 1;
                break;
                
            case CSSL_DELIM_CRLF:
                if (serial->frame_pos >= 2 && 
                    serial->frame_buffer[serial->frame_pos-2] == '\r' &&
                    serial->frame_buffer[serial->frame_pos-1] == '\n') {
                    frame_complete = 1;
                }
                break;
                
            case CSSL_DELIM_CUSTOM:
                if (serial->frame_pos >= serial->delimiter_len) {
                    if (memcmp(&serial->frame_buffer[serial->frame_pos - serial->delimiter_len],
                              serial->custom_delimiter, serial->delimiter_len) == 0) {
                        frame_complete = 1;
                    }
                }
                break;
                
            default:
                break;
        }
        
        if (frame_complete) {
            /* Complete frame received */
            if (serial->enhanced_callback) {
                serial->enhanced_callback(serial->id, serial->frame_buffer, 
                                        serial->frame_pos, serial->data_type, &current_time);
            }
            serial->stats.frames_received++;
            serial->frame_pos = 0;
        }
    }
    
    return consumed;
}

/*-------------------------------------
 * Enhanced signal handler
 */

void cssl_handler(int signo, siginfo_t *info, void *ignored)
{
    cssl_t *cur;
    int n;

    /* is this signal which says about incoming data? */
    if (info->si_code==POLL_IN) {
        /* Yes, we got some data */
        for(cur=head;cur;cur=cur->next) {
            /* Let's find proper cssl_t */
            if (cur->fd==info->si_fd) {
                /* Got it - read with optimized size */
                int read_size = cur->max_read_size;
                if (cur->buffer_pos + read_size > CSSL_BUFFER_SIZE) {
                    read_size = CSSL_BUFFER_SIZE - cur->buffer_pos;
                }
                
                n = read(cur->fd, &cur->buffer[cur->buffer_pos], read_size);
                
                if (n > 0) {
                    /* Process with enhanced callback if available */
                    if (cur->enhanced_callback) {
                        int processed = process_received_data(cur, &cur->buffer[cur->buffer_pos], n);
                        cur->buffer_pos += processed;
                        
                        /* Reset buffer if full */
                        if (cur->buffer_pos >= CSSL_BUFFER_SIZE - cur->max_read_size) {
                            cur->buffer_pos = 0;
                        }
                    } else if (cur->callback) {
                        /* Legacy callback */
                        cur->callback(cur->id, &cur->buffer[cur->buffer_pos], n);
                    }
                }
                return;
            }
        }
    }
}


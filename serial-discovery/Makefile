
# Enhanced CSSL Library and Serial Data Collector Makefile

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -D_GNU_SOURCE
LDFLAGS = -lrt

# Directories
SRCDIR = $(PWD)
OBJDIR = obj
BINDIR = bin

# Source files
CSSL_SOURCES = $(SRCDIR)/cssl.c
COBS_SOURCES = $(SRCDIR)/cobs_crc16.c
MAIN_SOURCES = main.c
ALL_SOURCES = $(CSSL_SOURCES) $(COBS_SOURCES) $(MAIN_SOURCES)

# Object files
CSSL_OBJECTS = $(OBJDIR)/cssl.o
COBS_OBJECTS = $(OBJDIR)/cobs_crc16.o
MAIN_OBJECTS = $(OBJDIR)/main.o
ALL_OBJECTS = $(CSSL_OBJECTS) $(COBS_OBJECTS) $(MAIN_OBJECTS)

# Target executables
TARGET = $(BINDIR)/serial_collector
LIBRARY = $(BINDIR)/libcssl.a

# Default target
all: directories $(TARGET) $(LIBRARY)

# Create directories
directories:
	@mkdir -p $(OBJDIR) $(BINDIR)

# Main executable
$(TARGET): $(ALL_OBJECTS)
	@echo "Linking $(TARGET)..."
	$(CC) $(ALL_OBJECTS) -o $@ $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Static library
$(LIBRARY): $(CSSL_OBJECTS) $(COBS_OBJECTS)
	@echo "Creating static library $(LIBRARY)..."
	ar rcs $@ $^
	@echo "Library created: $(LIBRARY)"

# Object files
$(OBJDIR)/cssl.o: $(SRCDIR)/cssl.c $(SRCDIR)/cssl.h
	@echo "Compiling CSSL library..."
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/cobs_crc16.o: $(SRCDIR)/cobs_crc16.c $(SRCDIR)/cobs_crc16.h
	@echo "Compiling COBS CRC16 library..."
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/main.o: main.c $(SRCDIR)/cssl.h $(SRCDIR)/cobs_crc16.h
	@echo "Compiling main application..."
	$(CC) $(CFLAGS) -c $< -o $@

# Test targets
test: $(TARGET)
	@echo "Running basic functionality test..."
	@echo "Note: Connect a serial device to test properly"
	./$(TARGET) --help

# Example usage targets
example-ascii: $(TARGET)
	@echo "Example: ASCII data collection with newline delimiter"
	./$(TARGET) -d /dev/ttyUSB0 -b 9600 -t ascii -l newline -v

example-binary: $(TARGET)
	@echo "Example: Binary data collection with fixed frame size"
	./$(TARGET) -d /dev/ttyUSB0 -b 115200 -t binary -f 32 -H -o binary_data.log

example-custom: $(TARGET)
	@echo "Example: Custom delimiter (CRLF) with verbose output"
	./$(TARGET) -d /dev/ttyACM0 -b 57600 -t framed -l crlf -v -S

example-control: $(TARGET)
	@echo "Example: Auto-control with FON command every 2 seconds"
	./$(TARGET) -d /dev/ttyUSB0 -b 9600 -c FON -C 2000 -v

# Clean targets
clean:
	@echo "Cleaning build files..."
	rm -rf $(OBJDIR) $(BINDIR)
	rm -f *.log *.dat core

# Help target
help:
	@echo "Enhanced CSSL Library Build System"
	@echo "=================================="
	@echo ""
	@echo "Build targets:"
	@echo "  all         - Build main executable and library (default)"
	@echo "  clean       - Remove all build files"
	@echo ""
	@echo "Example targets:"
	@echo "  example-ascii   - Run ASCII data collection example"
	@echo "  example-binary  - Run binary data collection example"  
	@echo "  example-custom  - Run custom delimiter example"
	@echo "  example-control - Run auto-control example"
	@echo ""
	@echo "Usage: make [target]"

# Declare phony targets
.PHONY: all directories test example-ascii example-binary example-custom example-control clean help

# Default target if none specified
.DEFAULT_GOAL := all


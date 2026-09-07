#pragma once

#include <Arduino.h>

/**
 * @file uart_protocol.h
 * @brief Shared constants and protocol definitions for Nano <-> ESP32-S3 UART communication.
 */

// Baud rate used for the inter-board UART link.
// 9600 bps provides rock-solid signal integrity with SoftwareSerial and through resistor dividers.
#define UART_COMM_BAUD 9600

// Serial monitor baud rate for PC debugging on both boards.
#define SERIAL_DEBUG_BAUD 115200

// Line termination character for framing messages
#define PROTOCOL_DELIMITER '\n'

// Maximum message buffer length (including null terminator)
#define PROTOCOL_BUFFER_SIZE 64

// Message prefixes
#define PREFIX_PING     "PING:"
#define PREFIX_PONG     "PONG:"
#define PREFIX_SET_RPM  "SET_RPM:"
#define PREFIX_ACK_RPM  "ACK_RPM:"
#define PREFIX_RPM      "RPM:"
#define PREFIX_GET_RPM  "GET_RPM"

// BLDC motor physics constants: v = f * 60 / 7
#define BLDC_POLE_PAIRS 7
#define BLDC_MAX_RPM    1000

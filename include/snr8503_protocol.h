/**
 * @file snr8503_protocol.h
 * @brief Binary UART Protocol Driver for SNR8503M BLDC Motor Controller
 *
 * Implements:
 * - 9-byte command frame packing with Big-Endian fields and 8-bit checksum.
 * - Non-blocking streaming 16-byte telemetry frame unpacking.
 * - Engineering conversions: Electrical Hz <-> RPM <-> rad/s (P = 7 for T-200).
 * - Fixed-point Q8 decoding for bus voltage (V) and bus current (A).
 * - Fault bitmask decoding.
 */

#pragma once

#include <Arduino.h>

constexpr uint8_t  SNR8503_CMD_FRAME_LEN = 9;
constexpr uint8_t  SNR8503_TEL_FRAME_LEN = 16;
constexpr uint8_t  SNR8503_FRAME_HEADER  = 0xAA;
constexpr uint8_t  SNR8503_FRAME_TAIL    = 0x55;
constexpr uint8_t  DEFAULT_POLE_PAIRS    = 7;

// Fault Bitmask Definitions (Matching Global_Variable.h)
namespace SnrFault {
    constexpr uint16_t SHORT_CIRCUIT    = 0x0001;
    constexpr uint16_t UNDER_VOLTAGE    = 0x0002;
    constexpr uint16_t OVER_VOLTAGE     = 0x0004;
    constexpr uint16_t ROTOR_BLOCKED    = 0x0008;
    constexpr uint16_t DC_OFFSET_ERR    = 0x0010;
    constexpr uint16_t MOS_OVER_TEMP    = 0x0020;
    constexpr uint16_t MOS_LOW_TEMP     = 0x0040;
    constexpr uint16_t BAT_OVER_TEMP    = 0x0080;
    constexpr uint16_t BAT_LOW_TEMP     = 0x0100;
    constexpr uint16_t OVER_LOAD        = 0x0200;
    constexpr uint16_t PHASE_DROP       = 0x0400;
    constexpr uint16_t MOSFET_CHECK_ERR = 0x0800;
}

// Main State Machine Enum (Matching M1_StateMachine.h)
enum class SnrMainState : uint8_t {
    Fault = 0,
    Init  = 1,
    Stop  = 2,
    Run   = 3
};

struct Snr8503Telemetry {
    SnrMainState state;
    uint16_t speed_hz;
    float    rpm;
    float    rad_s;
    uint16_t power_mw;
    float    power_w;
    uint16_t fault_bits;
    float    bus_voltage_v;
    float    bus_current_a;
    uint32_t timestamp_ms;
    bool     valid;
};

class Snr8503Protocol {
public:
    Snr8503Protocol(uint8_t polePairs = DEFAULT_POLE_PAIRS);

    // Frame Building (Host -> SNR8503M, 9 Bytes)
    size_t buildCommandFrame(uint8_t* outBuf, size_t maxLen, uint16_t speedHz, uint16_t powerLimitMw = 50000);
    size_t buildCommandFromRpm(uint8_t* outBuf, size_t maxLen, float targetRpm, uint16_t powerLimitMw = 50000);
    size_t buildCommandFromRadS(uint8_t* outBuf, size_t maxLen, float targetRadS, uint16_t powerLimitMw = 50000);

    // Stream Parser (Feeds 1 byte at a time, returns true when a valid 16-byte frame is completed)
    bool parseByte(uint8_t b, Snr8503Telemetry& outTelemetry);

    // Checksum Helper
    static uint8_t calculateChecksum(const uint8_t* data, size_t length);

    // Human-Readable Helpers
    static const char* getStateName(SnrMainState state);
    static void getFaultString(uint16_t faultBits, char* outStr, size_t maxLen);

private:
    uint8_t _polePairs;
    uint8_t _rxBuffer[SNR8503_TEL_FRAME_LEN];
    uint8_t _rxIndex;
    bool    _inFrame;
};

/**
 * @file snr8503_protocol.cpp
 * @brief Implementation of Binary UART Protocol Driver for SNR8503M
 */

#include "snr8503_protocol.h"

constexpr float TWO_PI_CONST = 6.283185307179586f;

Snr8503Protocol::Snr8503Protocol(uint8_t polePairs)
    : _polePairs(polePairs > 0 ? polePairs : DEFAULT_POLE_PAIRS),
      _rxIndex(0),
      _inFrame(false)
{
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
}

uint8_t Snr8503Protocol::calculateChecksum(const uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

size_t Snr8503Protocol::buildCommandFrame(uint8_t* outBuf, size_t maxLen, uint16_t speedHz, uint16_t powerLimitMw) {
    if (outBuf == nullptr || maxLen < SNR8503_CMD_FRAME_LEN) return 0;

    outBuf[0] = SNR8503_FRAME_HEADER;                // 0xAA
    outBuf[1] = (uint8_t)((speedHz >> 8) & 0xFF);    // Speed MSB
    outBuf[2] = (uint8_t)(speedHz & 0xFF);           // Speed LSB
    outBuf[3] = (uint8_t)((powerLimitMw >> 8) & 0xFF);// Power Limit MSB
    outBuf[4] = (uint8_t)(powerLimitMw & 0xFF);      // Power Limit LSB
    outBuf[5] = 0x00;                                // Reserved
    outBuf[6] = 0x00;                                // Reserved
    outBuf[7] = calculateChecksum(outBuf, 7);        // Checksum (sum of Bytes 0..6)
    outBuf[8] = SNR8503_FRAME_TAIL;                  // 0x55

    return SNR8503_CMD_FRAME_LEN;
}

size_t Snr8503Protocol::buildCommandFromRpm(uint8_t* outBuf, size_t maxLen, float targetRpm, uint16_t powerLimitMw) {
    if (targetRpm < 0.0f) targetRpm = 0.0f;
    // f_elec = (RPM * P) / 60
    uint16_t speedHz = (uint16_t)((targetRpm * (float)_polePairs) / 60.0f + 0.5f);
    return buildCommandFrame(outBuf, maxLen, speedHz, powerLimitMw);
}

size_t Snr8503Protocol::buildCommandFromRadS(uint8_t* outBuf, size_t maxLen, float targetRadS, uint16_t powerLimitMw) {
    if (targetRadS < 0.0f) targetRadS = 0.0f;
    // rps = rad_s / (2*pi)
    // f_elec = rps * P = (rad_s * P) / (2*pi)
    uint16_t speedHz = (uint16_t)((targetRadS * (float)_polePairs) / TWO_PI_CONST + 0.5f);
    return buildCommandFrame(outBuf, maxLen, speedHz, powerLimitMw);
}

bool Snr8503Protocol::parseByte(uint8_t b, Snr8503Telemetry& outTelemetry) {
    if (!_inFrame) {
        if (b == SNR8503_FRAME_HEADER) {
            _rxBuffer[0] = b;
            _rxIndex = 1;
            _inFrame = true;
        }
        return false;
    }

    _rxBuffer[_rxIndex++] = b;

    // Check frame length
    if (_rxIndex >= SNR8503_TEL_FRAME_LEN) {
        _inFrame = false;
        _rxIndex = 0;

        // Verify Tail Byte
        if (_rxBuffer[SNR8503_TEL_FRAME_LEN - 1] != SNR8503_FRAME_TAIL) {
            return false;
        }

        // Verify Checksum (Bytes 0..13)
        uint8_t expectedCkm = calculateChecksum(_rxBuffer, 14);
        if (_rxBuffer[14] != expectedCkm) {
            return false;
        }

        // Frame is valid: Unpack telemetry
        outTelemetry.state = (SnrMainState)_rxBuffer[1];
        outTelemetry.speed_hz = ((uint16_t)_rxBuffer[2] << 8) | _rxBuffer[3];
        outTelemetry.rpm = ((float)outTelemetry.speed_hz * 60.0f) / (float)_polePairs;
        outTelemetry.rad_s = (((float)outTelemetry.speed_hz / (float)_polePairs) * TWO_PI_CONST);
        outTelemetry.power_mw = ((uint16_t)_rxBuffer[4] << 8) | _rxBuffer[5];
        outTelemetry.power_w = (float)outTelemetry.power_mw / 1000.0f;
        outTelemetry.fault_bits = ((uint16_t)_rxBuffer[8] << 8) | _rxBuffer[9];

        // Voltage & Current Q8 fixed-point (divided by 256.0)
        uint16_t rawVoltage = ((uint16_t)_rxBuffer[10] << 8) | _rxBuffer[11];
        outTelemetry.bus_voltage_v = (float)rawVoltage / 256.0f;

        uint16_t rawCurrent = ((uint16_t)_rxBuffer[12] << 8) | _rxBuffer[13];
        outTelemetry.bus_current_a = (float)rawCurrent / 256.0f;

        outTelemetry.timestamp_ms = millis();
        outTelemetry.valid = true;
        return true;
    }

    return false;
}

const char* Snr8503Protocol::getStateName(SnrMainState state) {
    switch (state) {
        case SnrMainState::Fault: return "FAULT";
        case SnrMainState::Init:  return "INIT";
        case SnrMainState::Stop:  return "STOP";
        case SnrMainState::Run:   return "RUN";
        default:                  return "UNKNOWN";
    }
}

void Snr8503Protocol::getFaultString(uint16_t faultBits, char* outStr, size_t maxLen) {
    if (outStr == nullptr || maxLen == 0) return;
    if (faultBits == 0) {
        snprintf(outStr, maxLen, "OK (No Faults)");
        return;
    }

    outStr[0] = '\0';
    size_t written = 0;

    auto appendFault = [&](const char* name) {
        if (written > 0 && written < maxLen - 2) {
            strncat(outStr, ", ", maxLen - written - 1);
            written += 2;
        }
        strncat(outStr, name, maxLen - written - 1);
        written = strlen(outStr);
    };

    if (faultBits & SnrFault::SHORT_CIRCUIT)    appendFault("SHORT_CIRCUIT");
    if (faultBits & SnrFault::UNDER_VOLTAGE)    appendFault("UNDER_VOLTAGE");
    if (faultBits & SnrFault::OVER_VOLTAGE)     appendFault("OVER_VOLTAGE");
    if (faultBits & SnrFault::ROTOR_BLOCKED)    appendFault("ROTOR_BLOCKED");
    if (faultBits & SnrFault::DC_OFFSET_ERR)    appendFault("DC_OFFSET_ERR");
    if (faultBits & SnrFault::MOS_OVER_TEMP)    appendFault("MOS_OVER_TEMP");
    if (faultBits & SnrFault::MOS_LOW_TEMP)     appendFault("MOS_LOW_TEMP");
    if (faultBits & SnrFault::BAT_OVER_TEMP)    appendFault("BAT_OVER_TEMP");
    if (faultBits & SnrFault::BAT_LOW_TEMP)     appendFault("BAT_LOW_TEMP");
    if (faultBits & SnrFault::OVER_LOAD)        appendFault("OVER_LOAD");
    if (faultBits & SnrFault::PHASE_DROP)       appendFault("PHASE_DROP");
    if (faultBits & SnrFault::MOSFET_CHECK_ERR) appendFault("MOSFET_CHECK_ERR");
}

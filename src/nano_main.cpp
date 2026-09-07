/**
 * @file nano_main.cpp
 * @brief Arduino Nano UART & Virtual BLDC Sensor Firmware (ATmega328P)
 * 
 * Functions:
 * 1. Virtual BLDC Motor Speed Sensor:
 *    - Pin D4 outputs pulses proportional to motor velocity.
 *    - Formula: v = f * 60 / 7  <=>  f = v * 7 / 60
 *    - Velocity setpoint (0 - 1000 RPM) can be controlled via UART command: "SET_RPM:<val>\n".
 * 
 * 2. UART Communication:
 *    - SoftwareSerial on D2 (RX) and D3 (TX) at 9600 baud.
 *    - Hardware Serial on USB at 115200 baud for PC monitoring.
 * 
 * Hardware Connections:
 *   Nano D3 (TX) --------[ 1kΩ ]-------+-------- ESP32-S3 RX (GPIO 4)
 *                                      |
 *                                   [ 2kΩ ]
 *                                      |
 *                                     GND
 *   Nano D4 (Pulse Out) -[ 1kΩ ]-------+-------- ESP32-S3 Pulse In (GPIO 5)
 *                                      |
 *                                   [ 2kΩ ]
 *                                      |
 *                                     GND
 *   Nano D2 (RX) <------------------------------ ESP32-S3 TX (GPIO 1)
 *   Nano GND     <-----------------------------> ESP32-S3 GND
 */

#include <Arduino.h>
#include "uart_protocol.h"

#define USE_SOFTWARE_SERIAL 1

#if USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  constexpr uint8_t PIN_NANO_RX = 2; // From ESP32 TX
  constexpr uint8_t PIN_NANO_TX = 3; // To ESP32 RX via divider
  SoftwareSerial CommSerial(PIN_NANO_RX, PIN_NANO_TX);
#else
  #define CommSerial Serial
#endif

// Hardware pins
constexpr uint8_t PIN_PULSE_OUT = 4; // Virtual BLDC pulse output (5V -> needs divider to ESP32 pin 5)
constexpr uint8_t PIN_LED = 13;

// BLDC speed state
static uint16_t currentRpm = 500;           // Default 500 RPM on boot (adjustable 0-1000)
static uint32_t halfPeriodUs = 0;           // Half period in microseconds
static uint32_t lastToggleUs = 0;
static bool pulseState = false;

// UART protocol buffers and counters
static char rxBuffer[PROTOCOL_BUFFER_SIZE];
static uint8_t rxIndex = 0;
static uint32_t lastTelemetryTime = 0;

void updatePulsePeriod(uint16_t rpm) {
    if (rpm > BLDC_MAX_RPM) rpm = BLDC_MAX_RPM;
    currentRpm = rpm;

    if (currentRpm == 0) {
        halfPeriodUs = 0;
        pulseState = false;
        digitalWrite(PIN_PULSE_OUT, LOW);
    } else {
        // f = (rpm * 7) / 60
        // T_us = 1,000,000 / f = 60,000,000 / (rpm * 7)
        // halfPeriod = T_us / 2 = 30,000,000 / (rpm * 7)
        halfPeriodUs = 30000000UL / (7UL * (uint32_t)currentRpm);
    }

    float freq = (currentRpm * 7.0f) / 60.0f;
    Serial.print(F("[BLDC SENSOR] Speed set to: "));
    Serial.print(currentRpm);
    Serial.print(F(" RPM | Target Freq: "));
    Serial.print(freq, 2);
    Serial.print(F(" Hz | Half-period: "));
    Serial.print(halfPeriodUs);
    Serial.println(F(" us"));
}

void processMessage(const char* msg) {
    // 1. Check for SET_RPM:<val>
    if (strncmp(msg, PREFIX_SET_RPM, strlen(PREFIX_SET_RPM)) == 0) {
        int val = atoi(msg + strlen(PREFIX_SET_RPM));
        if (val < 0) val = 0;
        if (val > BLDC_MAX_RPM) val = BLDC_MAX_RPM;
        updatePulsePeriod((uint16_t)val);

        // Acknowledge back to ESP32
        CommSerial.print(PREFIX_ACK_RPM);
        CommSerial.print(currentRpm);
        CommSerial.print(PROTOCOL_DELIMITER);

        Serial.print(F("[NANO RX] Set RPM Command: "));
        Serial.println(currentRpm);
        return;
    }

    // 2. Check for GET_RPM
    if (strcmp(msg, PREFIX_GET_RPM) == 0) {
        CommSerial.print(PREFIX_RPM);
        CommSerial.print(currentRpm);
        CommSerial.print(PROTOCOL_DELIMITER);
        return;
    }

    // 3. Check for PING:<seq>
    if (strncmp(msg, PREFIX_PING, strlen(PREFIX_PING)) == 0) {
        const char* seq = msg + strlen(PREFIX_PING);
        // Reply with PONG:<seq>,ACK
        CommSerial.print(PREFIX_PONG);
        CommSerial.print(seq);
        CommSerial.print(F(",ACK"));
        CommSerial.print(PROTOCOL_DELIMITER);
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        return;
    }

    // 4. Other messages
    Serial.print(F("[NANO RX] <-- "));
    Serial.println(msg);
}

void sendBinaryTelemetry(uint16_t speedHz) {
    uint8_t txFrame[16];
    txFrame[0] = 0xAA;
    txFrame[1] = (currentRpm > 0) ? 3 : 2; // 3 = Run, 2 = Stop
    txFrame[2] = (uint8_t)(speedHz >> 8);
    txFrame[3] = (uint8_t)(speedHz & 0xFF);
    
    // Simulated power: ~35W (35,000 mW) at 1000 RPM
    uint16_t powerMw = (uint16_t)(((uint32_t)currentRpm * 35UL));
    txFrame[4] = (uint8_t)(powerMw >> 8);
    txFrame[5] = (uint8_t)(powerMw & 0xFF);
    txFrame[6] = 0x00;
    txFrame[7] = 0x00;
    txFrame[8] = 0x00; // Faults MSB (0 = No Faults)
    txFrame[9] = 0x00; // Faults LSB
    
    // Simulated 24.0V bus in Q8 format (24.0 * 256 = 6144 = 0x1800)
    uint16_t vBusQ8 = 6144;
    txFrame[10] = (uint8_t)(vBusQ8 >> 8);
    txFrame[11] = (uint8_t)(vBusQ8 & 0xFF);
    
    // Simulated bus current in Q8 format (~0.3A idle up to ~4.5A at 1000 RPM)
    float iBus = (currentRpm > 0) ? (0.3f + (float)currentRpm * 0.0042f) : 0.0f;
    uint16_t iBusQ8 = (uint16_t)(iBus * 256.0f);
    txFrame[12] = (uint8_t)(iBusQ8 >> 8);
    txFrame[13] = (uint8_t)(iBusQ8 & 0xFF);
    
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 14; i++) {
        sum += txFrame[i];
    }
    txFrame[14] = (uint8_t)(sum & 0xFF);
    txFrame[15] = 0x55;

    CommSerial.write(txFrame, 16);
}

void readIncomingData() {
    static uint8_t binBuffer[9];
    static uint8_t binIndex = 0;
    static bool inBinFrame = false;

    while (CommSerial.available() > 0) {
        uint8_t c = (uint8_t)CommSerial.read();

        // 1. Check for Binary SNR8503M Frame Start (0xAA)
        if (!inBinFrame && c == 0xAA) {
            binBuffer[0] = 0xAA;
            binIndex = 1;
            inBinFrame = true;
            continue;
        }

        if (inBinFrame) {
            binBuffer[binIndex++] = c;
            if (binIndex >= 9) {
                inBinFrame = false;
                binIndex = 0;

                // Validate Tail (0x55) and Checksum
                if (binBuffer[8] == 0x55) {
                    uint16_t sum = 0;
                    for (uint8_t i = 0; i < 7; i++) sum += binBuffer[i];
                    if ((uint8_t)(sum & 0xFF) == binBuffer[7]) {
                        uint16_t speedHz = ((uint16_t)binBuffer[1] << 8) | binBuffer[2];
                        uint16_t targetRpm = (uint16_t)(((uint32_t)speedHz * 60UL) / 7UL);
                        updatePulsePeriod(targetRpm);
                        sendBinaryTelemetry(speedHz);

                        Serial.print(F("[NANO RX] SNR8503M Frame: "));
                        Serial.print(speedHz);
                        Serial.print(F(" Hz ("));
                        Serial.print(targetRpm);
                        Serial.println(F(" RPM) -> Sent 16B Telemetry"));
                        continue;
                    }
                }
            }
            continue;
        }

        // 2. Fallback: ASCII command parser for legacy commands
        if (c == PROTOCOL_DELIMITER) {
            rxBuffer[rxIndex] = '\0';
            if (rxIndex > 0) {
                if (rxBuffer[rxIndex - 1] == '\r') {
                    rxBuffer[rxIndex - 1] = '\0';
                }
                processMessage(rxBuffer);
            }
            rxIndex = 0;
        } else if (c != '\r') {
            if (rxIndex < sizeof(rxBuffer) - 1) {
                rxBuffer[rxIndex++] = (char)c;
            } else {
                rxIndex = 0;
            }
        }
    }
}

// Non-blocking pulse train generation on D4
void updatePulseGenerator() {
    if (halfPeriodUs == 0) return; // Motor stopped

    uint32_t now = micros();
    if (now - lastToggleUs >= halfPeriodUs) {
        lastToggleUs += halfPeriodUs;
        // In case of any execution delay, prevent runaway catch-up
        if (now - lastToggleUs >= halfPeriodUs) {
            lastToggleUs = now;
        }
        pulseState = !pulseState;
        digitalWrite(PIN_PULSE_OUT, pulseState ? HIGH : LOW);
    }
}

void setup() {
    pinMode(PIN_PULSE_OUT, OUTPUT);
    digitalWrite(PIN_PULSE_OUT, LOW);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    Serial.begin(SERIAL_DEBUG_BAUD);
    while (!Serial && millis() < 2000);

    Serial.println(F("\n=============================================="));
    Serial.println(F("  NANO VIRTUAL BLDC SENSOR & UART FIRMWARE    "));
    Serial.println(F("=============================================="));

#if USE_SOFTWARE_SERIAL
    Serial.print(F("[CONFIG] SoftwareSerial on RX=D"));
    Serial.print(PIN_NANO_RX);
    Serial.print(F(", TX=D"));
    Serial.print(PIN_NANO_TX);
    Serial.print(F(" at "));
    Serial.print(UART_COMM_BAUD);
    Serial.println(F(" bps"));
    CommSerial.begin(UART_COMM_BAUD);
#endif

    // Initialize default speed
    updatePulsePeriod(currentRpm);
    Serial.println(F("[READY] Outputting pulses on D4. Waiting for UART commands...\n"));
}

void loop() {
    // 1. Non-blocking pulse generation
    updatePulseGenerator();

    // 2. Process incoming UART commands from ESP32
    readIncomingData();

    // 3. Periodic telemetry to PC debug console every 2s
    uint32_t now = millis();
    if (now - lastTelemetryTime >= 2000) {
        lastTelemetryTime = now;
        float freq = (currentRpm * 7.0f) / 60.0f;
        Serial.print(F("[STATUS] Virtual BLDC Speed: "));
        Serial.print(currentRpm);
        Serial.print(F(" RPM (f = "));
        Serial.print(freq, 1);
        Serial.println(F(" Hz)"));
    }
}

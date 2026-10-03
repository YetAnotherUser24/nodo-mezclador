/**
 * @file esp32s3_main.cpp
 * @brief ESP32-S3 T-200 Thruster Closed-Loop Velocity Controller
 * 
 * Hardware Architecture:
 * - Driver: SNR8503M BLDC Sensorless/Hall Driver (Open-loop factory firmware)
 * - Actuation: 10 kHz 12-bit LEDC Hardware PWM on GPIO 5 into VSP (Pin 3 of J1)
 *   via Option B (2N7002 / 2N3904 N-MOSFET open-drain with 330Ω stiff pull-up to +5V)
 * - Analytical Software Linearization: Exact closed-form inverse compensation
 *   eliminates the internal RC divider/filter non-linearity (toggleable via macro)
 * - Feedback: Driver FG (Pin 4 of J1) via 1kΩ/2kΩ divider into GPIO 4
 * - Tachometer: Microsecond reciprocal edge timing with dynamic deceleration decay
 * - Control: 100 Hz discrete PID velocity controller in canonical SI units (rad/s)
 * 
 * Wiring Interconnections:
 *   ESP32-S3 GPIO 13 (PWM) ----> [ 100Ω ] ----> Gate / Base (2N7002 / 2N3904)
 *   SNR8503M +5V (J1 Pin 2) ---> [ 330Ω ] ----> Drain / Collector (2N7002)
 *   SNR8503M VSP (J1 Pin 3) <------------------ Drain / Collector (2N7002)
 *   ESP32-S3 GND <-------------> Source / Emitter & SNR8503M GND (J1 Pin 1/5)
 *   
 *   SNR8503M FG (J1 Pin 4) ----> [ 1kΩ ] -----> ESP32-S3 GPIO 7 (Pulse IN)
 *                                           |
 *                                         [ 2kΩ ]
 *                                           |
 *                                          GND
 */

#include <Arduino.h>
#include <math.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// =============================================================================
#ifndef PIN_VSP_PWM
  #define PIN_VSP_PWM         13       // GPIO for VSP PWM Actuation
#endif

#ifndef PIN_PULSE_IN
  #define PIN_PULSE_IN        7       // GPIO for FG Tachometer Pulse Input
#endif

#ifndef PIN_LED
  #define PIN_LED             2       // Built-in status LED
#endif

constexpr uint8_t  BLDC_POLE_PAIRS        = 7;       // BlueRobotics T-200 Thruster
constexpr float    TWO_PI_CONST           = 6.283185307179586f;
constexpr float    MAX_THRUSTER_RPM       = 3800.0f; // T-200 rated max forward speed
constexpr float    MAX_THRUSTER_RAD_S     = (MAX_THRUSTER_RPM * TWO_PI_CONST) / 60.0f; // ~397.9 rad/s

// =============================================================================
// LEDC PWM HARDWARE CONFIGURATION (10 kHz, 12-bit)
// =============================================================================
constexpr uint8_t  LEDC_PWM_CHANNEL       = 0;
constexpr uint32_t LEDC_PWM_FREQ_HZ       = 10000;   // 10 kHz (Driver spec: 1 - 20 kHz)
constexpr uint8_t  LEDC_PWM_RES_BITS      = 10;      // 10-bit resolution (0 .. 1023)
constexpr uint32_t LEDC_PWM_MAX_TICKS     = (1UL << LEDC_PWM_RES_BITS); // 1024 (100% duty)

// Open-drain transistor polarity:
// 1 = Transistor inverts: GPIO HIGH -> VSP = 0V (Stop). GPIO LOW -> VSP = 5V (Full).
// 0 = Non-inverting / Push-pull: GPIO HIGH -> VSP = 5V.
#define VSP_PWM_INVERTED                  1

// SNR8503M Driver Thresholds (Empirical from Open-Loop Telemetry):
// 20.0% duty -> 0 RPM (Stall/Deadband)
// 21.0% duty -> 253 RPM (Startup)
constexpr float MOTOR_MIN_SPIN_DUTY       = 0.25f;  // True empirical startup threshold
constexpr float MOTOR_MAX_ALLOWED_DUTY    = 0.989f;  // Max achievable duty with 330Ω pull-up

// =============================================================================
// TOGGLEABLE ANALYTICAL PWM LINEARIZATION COMPENSATOR
// =============================================================================
#define ENABLE_PWM_LINEARIZATION          1          // 1 = Enabled (Option B), 0 = Pure Linear
constexpr float PULLUP_RESISTOR_OHMS      = 200.0f;  // Rp in Ohms
constexpr float R30_INTERNAL_OHMS         = 10000.0f;// Internal series resistor
constexpr float GAIN_DIVIDER_RATIO        = (2.0f / 3.0f); // R32 / (R30 + R32) = 20k / 30k

/**
 * @brief Exact analytical inverse of the driver's onboard RC integrator.
 * Maps desired normalized throttle D_target -> pre-warped PWM duty cycle D_pwm.
 */
float compensate_pwm_duty(float d_target) {
    if (d_target <= 0.0f) return 0.0f;
    if (d_target > 1.0f) d_target = 1.0f;

#if ENABLE_PWM_LINEARIZATION
    constexpr float Rp = PULLUP_RESISTOR_OHMS;
    constexpr float R30 = R30_INTERNAL_OHMS;
    constexpr float G = GAIN_DIVIDER_RATIO;

    // Exact analytical inverse: D_pwm = ((Rp + R30) * D) / (R30 + G * Rp * D)
    float d_comp = ((Rp + R30) * d_target) / (R30 + G * Rp * d_target);
    if (d_comp > 1.0f) d_comp = 1.0f;
    return d_comp;
#else
    return d_target;
#endif
}

// =============================================================================
// VELOCITY STATE & TACHOMETER ENGINE
// =============================================================================
struct VelocityState {
    float rad_s;           // Angular velocity ω [rad/s] (Primary control variable)
    float rpm;             // Rotational speed [rev/min]
    float rps;             // Mechanical speed [rev/s]
    float freq_hz;         // FG electrical pulse frequency [Hz]
    uint32_t period_us;    // Measured pulse period [microseconds]
    uint32_t total_pulses; // Cumulative pulse counter
};

static VelocityState velocity = {0};

constexpr uint32_t STOP_TIMEOUT_US        = 3500000UL; // 3.5s timeout (~2.4 RPM floor)
constexpr uint32_t MIN_VALID_PERIOD_US    = 200;       // Glitch filter (> 5000 Hz)
constexpr uint8_t  FILTER_SIZE            = 5;

enum SensorState {
    STATE_STOPPED = 0,
    STATE_FIRST_PULSE,
    STATE_RUNNING
};

static volatile uint32_t isrLastPulseUs   = 0;
static volatile uint32_t isrPeriodUs      = 0;
static volatile uint32_t isrTotalPulses   = 0;
static volatile bool     isrNewPulse      = false;

static SensorState currentSensorState     = STATE_STOPPED;
static uint32_t periodBuffer[FILTER_SIZE] = {0};
static uint8_t  filterIdx                 = 0;
static uint8_t  filterCount               = 0;
static uint32_t lastStablePeriodUs        = 0;

static uint8_t lastFaultCode = 0;
static uint8_t currentFaultCount = 0;

void IRAM_ATTR onPulseRisingEdge() {
    uint32_t now = micros();
    if (isrLastPulseUs > 0) {
        uint32_t p = now - isrLastPulseUs;
        if (p >= MIN_VALID_PERIOD_US) {
            isrPeriodUs = p;
            isrLastPulseUs = now;
            isrTotalPulses++;
            isrNewPulse = true;
        }
    } else {
        isrLastPulseUs = now;
        isrTotalPulses++;
        isrNewPulse = true;
    }
}

void initSpeedSensor() {
    pinMode(PIN_PULSE_IN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PULSE_IN), onPulseRisingEdge, RISING);
    Serial.printf("[INIT] Reciprocal FG Tachometer active on GPIO %d (ISR Edge Timing)\n", PIN_PULSE_IN);
}

void updateSpeedMeasurement() {
    uint32_t nowUs = micros();

    noInterrupts();
    uint32_t lastPulseUs = isrLastPulseUs;
    uint32_t rawPeriodUs = isrPeriodUs;
    uint32_t totalPulses = isrTotalPulses;
    bool     hasNewPulse = isrNewPulse;
    isrNewPulse = false;
    interrupts();

    velocity.total_pulses = totalPulses;
    uint32_t elapsedSincePulseUs;
    if (lastPulseUs == 0) {
        elapsedSincePulseUs = 99999999;
    } else if (nowUs >= lastPulseUs) {
        elapsedSincePulseUs = nowUs - lastPulseUs;
    } else {
        elapsedSincePulseUs = 0; // ISR fired between micros() and noInterrupts()
    }

    // 1. Zero-Speed Timeout
    if (elapsedSincePulseUs > STOP_TIMEOUT_US || lastPulseUs == 0) {
        currentSensorState = STATE_STOPPED;
        velocity.rad_s = 0.0f;
        velocity.rpm = 0.0f;
        velocity.rps = 0.0f;
        velocity.freq_hz = 0.0f;
        velocity.period_us = 0;
        filterCount = 0;
        lastStablePeriodUs = 0;
        return;
    }

    // 2. First Pulse Detection
    if (currentSensorState == STATE_STOPPED) {
        currentSensorState = STATE_FIRST_PULSE;
        velocity.rad_s = 0.0f;
        velocity.rpm = 0.0f;
        velocity.rps = 0.0f;
        velocity.freq_hz = 0.0f;
        velocity.period_us = 0;
        return;
    }

    // 3. New Pulse Processing
    if (hasNewPulse && rawPeriodUs > 0) {
        // --- FG Fault Pulse Decoder ---
        // Fault codes are transmitted as sequences of ~400ms pulses, separated by a 2.2s gap.
        if (rawPeriodUs > 150000) { // Any pulse >150ms is a fault code, not normal speed
            if (rawPeriodUs > 1500000) { // >1.5s gap marks the end of a sequence
                if (currentFaultCount > 0) {
                    lastFaultCode = currentFaultCount; // Commit decoded fault
                }
                currentFaultCount = 1; // This edge is the first pulse of the next sequence
            } else {
                currentFaultCount++; // Count pulses in current sequence
            }
            
            // Skip moving average for fault pulses, force 0 RPM
            velocity.period_us = rawPeriodUs;
            velocity.freq_hz = 1000000.0f / (float)rawPeriodUs;
            velocity.rpm = 0.0f;
            velocity.rad_s = 0.0f;
            velocity.rps = 0.0f;
            return;
        } else {
            // Normal running, clear faults
            lastFaultCode = 0;
            currentFaultCount = 0;
        }

        currentSensorState = STATE_RUNNING;

        if (lastStablePeriodUs > 0) {
            bool rapidAcceleration = (rawPeriodUs * 4 < lastStablePeriodUs * 3);
            bool rapidDeceleration = (rawPeriodUs * 3 > lastStablePeriodUs * 4);
            if (rapidAcceleration || rapidDeceleration) {
                filterCount = 0;
                filterIdx = 0;
            }
        }

        periodBuffer[filterIdx] = rawPeriodUs;
        filterIdx = (filterIdx + 1) % FILTER_SIZE;
        if (filterCount < FILTER_SIZE) filterCount++;

        uint64_t sum = 0;
        for (uint8_t i = 0; i < filterCount; i++) {
            sum += periodBuffer[i];
        }
        lastStablePeriodUs = (uint32_t)(sum / filterCount);
    }

    // 4. Dynamic Deceleration Decay
    if (currentSensorState == STATE_RUNNING && lastStablePeriodUs > 0) {
        uint32_t effectivePeriodUs = lastStablePeriodUs;
        if (elapsedSincePulseUs > lastStablePeriodUs) {
            effectivePeriodUs = elapsedSincePulseUs;
        }

        velocity.period_us = effectivePeriodUs;
        velocity.freq_hz = 1000000.0f / (float)effectivePeriodUs;
        velocity.rps = velocity.freq_hz / (float)BLDC_POLE_PAIRS;
        velocity.rad_s = velocity.rps * TWO_PI_CONST;
        velocity.rpm = velocity.rps * 60.0f;
    }
}

// =============================================================================
// HARDWARE ACTUATION (Option B Open-Drain PWM Driver)
// =============================================================================
static float currentCommandedDuty = 0.0f;
static float currentCompensatedDuty = 0.0f;
static bool isPwmAttached = false;

void applyHardwareDuty(float targetDuty);

void initPwmHardware() {
    // We will initialize the pin in applyHardwareDuty(0) cleanly.
    // Just force a safe stop initially.
    applyHardwareDuty(0.0f);

    Serial.printf("[INIT] LEDC PWM active on GPIO %d (10 kHz, 12-bit, Inverted=%d)\n",
                  PIN_VSP_PWM, VSP_PWM_INVERTED);
    Serial.printf("       Option B Active: Rp=%.1fΩ | Linearization=%s\n",
                  PULLUP_RESISTOR_OHMS, ENABLE_PWM_LINEARIZATION ? "ENABLED" : "BYPASSED");
}

void applyHardwareDuty(float targetDuty) {
    targetDuty = constrain(targetDuty, 0.0f, 1.0f);
    currentCommandedDuty = targetDuty;

    if (targetDuty < 0.005f) {
        // Complete stop: Bypass PWM entirely and use strict digital logic
        currentCompensatedDuty = 0.0f;
        if (isPwmAttached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
            ledcDetach(PIN_VSP_PWM);
#else
            ledcDetachPin(PIN_VSP_PWM);
#endif
            isPwmAttached = false;
        }
        pinMode(PIN_VSP_PWM, OUTPUT);
#if VSP_PWM_INVERTED
        digitalWrite(PIN_VSP_PWM, HIGH); // Transistor ON -> VSP 0V -> STOP
#else
        digitalWrite(PIN_VSP_PWM, LOW);  // Direct VSP 0V -> STOP
#endif
        return;
    }

    // Scale duty into active motor spinning band [MIN_SPIN_DUTY .. MAX_ALLOWED_DUTY]
    float effectiveDuty = MOTOR_MIN_SPIN_DUTY + targetDuty * (MOTOR_MAX_ALLOWED_DUTY - MOTOR_MIN_SPIN_DUTY);
    float compensatedDuty = compensate_pwm_duty(effectiveDuty);
    currentCompensatedDuty = compensatedDuty;

    uint32_t ticks;
#if VSP_PWM_INVERTED
    ticks = (uint32_t)roundf((1.0f - compensatedDuty) * (float)LEDC_PWM_MAX_TICKS);
#else
    ticks = (uint32_t)roundf(compensatedDuty * (float)LEDC_PWM_MAX_TICKS);
#endif
    if (ticks > LEDC_PWM_MAX_TICKS) ticks = LEDC_PWM_MAX_TICKS;

    if (!isPwmAttached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcAttach(PIN_VSP_PWM, LEDC_PWM_FREQ_HZ, LEDC_PWM_RES_BITS);
#else
        ledcSetup(LEDC_PWM_CHANNEL, LEDC_PWM_FREQ_HZ, LEDC_PWM_RES_BITS);
        ledcAttachPin(PIN_VSP_PWM, LEDC_PWM_CHANNEL);
#endif
        isPwmAttached = true;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_VSP_PWM, ticks);
#else
    ledcWrite(LEDC_PWM_CHANNEL, ticks);
#endif
}

// =============================================================================
// DISCRETE 100 Hz PID VELOCITY CONTROLLER
// =============================================================================
enum ControlMode {
    MODE_STOPPED = 0,
    MODE_OPEN_LOOP_DUTY,
    MODE_CLOSED_LOOP_PID
};

struct PidController {
    float Kp;
    float Ki;
    float Kd;
    float Ts;              // Sample time (0.01s for 100 Hz)
    float target_rad_s;    // Commanded angular velocity [rad/s]
    float integral;        // Integral accumulator
    float prev_meas_rad_s; // For derivative on measurement
    float max_slew_rate;   // Maximum output change per second
    float prev_output;     // Last commanded output duty
};

// Initial conservative tuning parameters for T-200 thruster
static PidController pid = {
    .Kp = 0.0012f,
    .Ki = 0.0035f,
    .Kd = 0.00004f,
    .Ts = 0.010f,          // 10 ms = 100 Hz
    .target_rad_s = 0.0f,
    .integral = 0.0f,
    .prev_meas_rad_s = 0.0f,
    .max_slew_rate = 2.0f, // Max duty change: 2.0/s (0 -> 100% in 500ms)
    .prev_output = 0.0f
};

static ControlMode currentControlMode = MODE_STOPPED;
static float openLoopDuty = 0.0f;
static uint32_t lastPidTimeUs = 0;

// --- Esc State & Fault globals ---
// --- Esc State & Fault globals ---
static uint32_t stallTimerMs = 0;
static bool inStallRecovery = false;
static uint32_t recoveryStartMs = 0;

const char* getDriverFaultStr(uint8_t code) {
    switch(code) {
        case 1: return "SHORT_CIRCUIT";
        case 2: return "UNDER_VOLTAGE";
        case 3: return "OVER_VOLTAGE";
        case 4: return "STALL_FAULT";
        case 5: return "SYSTEM_BIAS";
        case 6: return "MOS_OVERTEMP";
        case 10: return "OVER_CURRENT";
        case 12: return "MOS_SELFTEST";
        default: return "UNKNOWN_FAULT";
    }
}

const char* getEscStatusStr() {
    if (inStallRecovery) return "RECOVERING";
    if (lastFaultCode > 0) return getDriverFaultStr(lastFaultCode);
    if (currentControlMode == MODE_STOPPED) return "STOPPED";
    if (currentControlMode == MODE_OPEN_LOOP_DUTY) {
        return (openLoopDuty > 0.001f) ? "RUNNING(OL)" : "STOPPED";
    }
    if (pid.target_rad_s < 0.1f) return "IDLE";
    if (stallTimerMs > 0 && (millis() - stallTimerMs > 500)) return "STALL_WARN";
    return "RUNNING";
}

void pidReset() {
    pid.integral = 0.0f;
    pid.prev_meas_rad_s = velocity.rad_s;
    pid.prev_output = 0.0f;
}

void updatePidLoop() {
    if (currentControlMode == MODE_STOPPED) {
        applyHardwareDuty(0.0f);
        return;
    }

    if (currentControlMode == MODE_OPEN_LOOP_DUTY) {
        applyHardwareDuty(openLoopDuty);
        return;
    }

    // --- Stall Fault Recovery ---

    if (inStallRecovery) {
        // Force 0% PWM to clear the driver's Stall Fault
        applyHardwareDuty(0.0f);
        pidReset(); // Prevent PID windup during reset

        if (millis() - recoveryStartMs > 500) { // 500ms reset pulse
            inStallRecovery = false;
            stallTimerMs = millis();
        }
        return;
    }

    // Detect stall: Commanded to spin, but FG reports < 50 RPM (or 5Hz fault pulses)
    if (pid.target_rad_s > 0.1f && velocity.rpm < 50.0f) {
        if (stallTimerMs == 0) {
            stallTimerMs = millis();
        } else if (millis() - stallTimerMs > 1500) { // 1.5s timeout
            inStallRecovery = true;
            recoveryStartMs = millis();
            stallTimerMs = 0;
            Serial.println("\n[FAULT] Driver Stall Detected! Auto-resetting ESC (0% PWM for 500ms)...");
            return;
        }
    } else {
        stallTimerMs = 0; // Not stalled, reset timer
    }

    // --- Closed-Loop PID Mode (100 Hz) ---
    float error = pid.target_rad_s - velocity.rad_s;

    // Proportional term
    float p_term = pid.Kp * error;

    // Integral accumulation with conditional integration (anti-windup)
    pid.integral += pid.Ki * error * pid.Ts;
    pid.integral = constrain(pid.integral, -0.5f, 1.0f);

    // Derivative on measurement (eliminates setpoint derivative kick)
    float d_meas = (velocity.rad_s - pid.prev_meas_rad_s) / pid.Ts;
    pid.prev_meas_rad_s = velocity.rad_s;
    float d_term = -pid.Kd * d_meas;

    // Feedforward estimate: approximate linear duty estimate from target velocity
    float ff_term = (pid.target_rad_s / MAX_THRUSTER_RAD_S) * 0.70f;

    // Raw control output
    float u_raw = p_term + pid.integral + d_term + ff_term;
    float u_clamped = constrain(u_raw, 0.0f, 1.0f);

    // Slew rate limiter
    float max_delta = pid.max_slew_rate * pid.Ts;
    float delta = u_clamped - pid.prev_output;
    delta = constrain(delta, -max_delta, max_delta);
    float u_slewed = pid.prev_output + delta;
    pid.prev_output = u_slewed;

    // Apply to hardware PWM
    applyHardwareDuty(u_slewed);
}

// =============================================================================
// COMMAND INTERFACE & SERIAL CLI
// =============================================================================
static char pcRxBuffer[64];
static uint8_t pcRxIndex = 0;
static uint32_t lastTelemetryTimeMs = 0;

void printHelp() {
    Serial.println("\n--- T-200 Thruster Controller CLI ---");
    Serial.println("  SPEED <rad/s>       : Closed-loop angular velocity (e.g. 'SPEED 150')");
    Serial.println("  RPM <rpm>           : Closed-loop speed in RPM (e.g. 'RPM 1200')");
    Serial.println("  DUTY <0.0 - 1.0>    : Open-loop throttle override (e.g. 'DUTY 0.35')");
    Serial.println("  STOP                : Safely stop thruster");
    Serial.println("  TUNE <Kp> <Ki> <Kd> : Update PID gains live (e.g. 'TUNE 0.0015 0.004 0.00005')");
    Serial.println("  TEST HIGH           : Force GPIO HIGH (Should STOP the motor via transistor)");
    Serial.println("  TEST LOW            : Force GPIO LOW (Should FULL SPEED the motor via transistor)");
    Serial.println("  STATUS              : Print full controller & telemetry snapshot");
    Serial.println("  HELP                : Display this menu");
    Serial.println("-------------------------------------\n");
}

void printStatus() {
    float targetRpm = (pid.target_rad_s * 60.0f) / TWO_PI_CONST;
    Serial.println("\n========== T-200 CONTROLLER STATUS ==========");
    Serial.printf("  Mode:               %s\n",
                  (currentControlMode == MODE_STOPPED) ? "STOPPED" :
                  (currentControlMode == MODE_OPEN_LOOP_DUTY) ? "OPEN_LOOP_DUTY" : "CLOSED_LOOP_PID");
    Serial.printf("  Target Speed:       %6.2f rad/s (%6.1f RPM)\n", pid.target_rad_s, targetRpm);
    Serial.printf("  Measured Speed:     %6.2f rad/s (%6.1f RPM)\n", velocity.rad_s, velocity.rpm);
    Serial.printf("  Velocity Error:     %+6.2f rad/s\n", pid.target_rad_s - velocity.rad_s);
    Serial.printf("  FG Pulse Freq:      %6.1f Hz (Period: %lu us, Total: %lu)\n",
                  velocity.freq_hz, velocity.period_us, velocity.total_pulses);
    Serial.printf("  Commanded Duty:     %5.1f%%\n", currentCommandedDuty * 100.0f);
    Serial.printf("  Compensated PWM:    %5.1f%% (Linearization: %s)\n",
                  currentCompensatedDuty * 100.0f, ENABLE_PWM_LINEARIZATION ? "ON" : "OFF");
    Serial.printf("  PID Parameters:     Kp=%.6f, Ki=%.6f, Kd=%.6f, Int=%.4f\n",
                  pid.Kp, pid.Ki, pid.Kd, pid.integral);
    Serial.println("=============================================\n");
}

void handleCommand(char* cmd) {
    while (*cmd == ' ') cmd++;
    if (strlen(cmd) == 0) return;

    if (strncasecmp(cmd, "STOP", 4) == 0) {
        currentControlMode = MODE_STOPPED;
        pid.target_rad_s = 0.0f;
        openLoopDuty = 0.0f;
        pidReset();
        applyHardwareDuty(0.0f);
        Serial.println("[OK] Thruster STOPPED.");
    }
    else if (strncasecmp(cmd, "SPEED", 5) == 0) {
        float val = atof(cmd + 5);
        if (val < 0.0f) val = 0.0f;
        if (val > MAX_THRUSTER_RAD_S) val = MAX_THRUSTER_RAD_S;
        pid.target_rad_s = val;
        currentControlMode = MODE_CLOSED_LOOP_PID;
        Serial.printf("[OK] Set Closed-Loop Target: %.2f rad/s (%.1f RPM)\n",
                      pid.target_rad_s, (pid.target_rad_s * 60.0f) / TWO_PI_CONST);
    }
    else if (strncasecmp(cmd, "RPM", 3) == 0) {
        float val = atof(cmd + 3);
        if (val < 0.0f) val = 0.0f;
        if (val > MAX_THRUSTER_RPM) val = MAX_THRUSTER_RPM;
        pid.target_rad_s = (val * TWO_PI_CONST) / 60.0f;
        currentControlMode = MODE_CLOSED_LOOP_PID;
        Serial.printf("[OK] Set Closed-Loop Target: %.1f RPM (%.2f rad/s)\n",
                      val, pid.target_rad_s);
    }
    else if (strncasecmp(cmd, "DUTY", 4) == 0) {
        float val = atof(cmd + 4);
        val = constrain(val, 0.0f, 1.0f);
        openLoopDuty = val;
        currentControlMode = MODE_OPEN_LOOP_DUTY;
        Serial.printf("[OK] Open-Loop Throttle Override: %.1f%%\n", openLoopDuty * 100.0f);
    }
    else if (strncasecmp(cmd, "TUNE", 4) == 0) {
        float kp, ki, kd;
        if (sscanf(cmd + 4, "%f %f %f", &kp, &ki, &kd) == 3) {
            pid.Kp = kp;
            pid.Ki = ki;
            pid.Kd = kd;
            Serial.printf("[OK] PID Tuned: Kp=%.6f, Ki=%.6f, Kd=%.6f\n", pid.Kp, pid.Ki, pid.Kd);
        } else {
            Serial.println("[ERROR] Format: TUNE <Kp> <Ki> <Kd>");
        }
    }
    else if (strncasecmp(cmd, "TEST HIGH", 9) == 0) {
        currentControlMode = MODE_STOPPED; // prevent PID from running
        if (isPwmAttached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
            ledcDetach(PIN_VSP_PWM);
#else
            ledcDetachPin(PIN_VSP_PWM);
#endif
            isPwmAttached = false;
        }
        pinMode(PIN_VSP_PWM, OUTPUT);
        digitalWrite(PIN_VSP_PWM, HIGH);
        Serial.println("[TEST] GPIO 13 set to HIGH. If NPN is wired right, motor MUST STOP (VSP=0V).");
    }
    else if (strncasecmp(cmd, "TEST LOW", 8) == 0) {
        currentControlMode = MODE_STOPPED;
        if (isPwmAttached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
            ledcDetach(PIN_VSP_PWM);
#else
            ledcDetachPin(PIN_VSP_PWM);
#endif
            isPwmAttached = false;
        }
        pinMode(PIN_VSP_PWM, OUTPUT);
        digitalWrite(PIN_VSP_PWM, LOW);
        Serial.println("[TEST] GPIO 13 set to LOW. If NPN is wired right, motor MUST SPIN MAX (VSP=5V).");
    }
    else if (strncasecmp(cmd, "STATUS", 6) == 0) {
        printStatus();
    }
    else if (strncasecmp(cmd, "HELP", 4) == 0) {
        printHelp();
    }
    else {
        // Direct numeric input fallback: treat as RPM
        float val = atof(cmd);
        if (val > 0.0f && val <= MAX_THRUSTER_RPM) {
            pid.target_rad_s = (val * TWO_PI_CONST) / 60.0f;
            currentControlMode = MODE_CLOSED_LOOP_PID;
            Serial.printf("[OK] Set Closed-Loop Target: %.1f RPM (%.2f rad/s)\n",
                          val, pid.target_rad_s);
        } else {
            Serial.println("[ERROR] Unknown command. Type 'HELP' for options.");
        }
    }
}

void readUserCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (pcRxIndex > 0) {
                pcRxBuffer[pcRxIndex] = '\0';
                handleCommand(pcRxBuffer);
                pcRxIndex = 0;
            }
        } else if (pcRxIndex < sizeof(pcRxBuffer) - 1) {
            pcRxBuffer[pcRxIndex++] = c;
        }
    }
}

// =============================================================================
// ARDUINO SETUP & MAIN LOOP
// =============================================================================
void setup() {
    Serial.begin(115200);
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 3000));

    Serial.println("\n========================================================");
    Serial.println("  BLUE-ROBOTICS T-200 THRUSTER VELOCITY CONTROLLER      ");
    Serial.println("  LEDC PWM (Option B Stiff Pull-Up) + Reciprocal FG     ");
    Serial.println("========================================================");

#if defined(PIN_LED) && (PIN_LED >= 0)
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
#endif

    // 1. Initialize PWM Actuation Stage (GPIO 5, 10 kHz, 12-bit)
    initPwmHardware();

    // 2. Initialize Tachometer Edge Timing Engine (GPIO 4, Reciprocal ISR)
    initSpeedSensor();

    // 3. Ready
    printHelp();
}

void loop() {
    // 1. Update pulse timing & speed estimation continuously
    updateSpeedMeasurement();

    // 2. Process USB Serial CLI commands
    readUserCommands();

    // 3. Execute 100 Hz Discrete PID Velocity Loop (every 10,000 us)
    uint32_t nowUs = micros();
    if (nowUs - lastPidTimeUs >= 10000) {
        lastPidTimeUs = nowUs;
        updatePidLoop();
    }

    // 4. Periodic Telemetry Streaming (every 500 ms)
    uint32_t nowMs = millis();
    if (nowMs - lastTelemetryTimeMs >= 500) {
        lastTelemetryTimeMs = nowMs;

        float targetRpm = (pid.target_rad_s * 60.0f) / TWO_PI_CONST;
        float errRadS = pid.target_rad_s - velocity.rad_s;

        Serial.printf("[TELEMETRY] Status: %-11s | RPM: %6.1f / %4.0f | Duty: %4.1f%% (PWM: %4.1f%%) | Err: %+5.2f rad/s | Freq: %5.1f Hz\n",
                      getEscStatusStr(),
                      velocity.rpm,
                      targetRpm,
                      currentCommandedDuty * 100.0f,
                      currentCompensatedDuty * 100.0f,
                      errRadS,
                      velocity.freq_hz);

#if defined(PIN_LED) && (PIN_LED >= 0)
        if (velocity.rad_s > 0.5f) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        } else {
            digitalWrite(PIN_LED, LOW);
        }
#endif
    }

    yield();
}

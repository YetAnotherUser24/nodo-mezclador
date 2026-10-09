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
#include "cloud_worker.h"
#include "mixer_bus.h"

// Hardware is reached exclusively through the injected bus. Real builds bind it to LEDC PWM +
// the FG tachometer ISR; virtual builds bind it to a duty->rpm model. PID/CLI code is identical.
static IPwmActuator& actuator = mixerActuator();
static ITachometer& tachometer = mixerTachometer();
static IKnobInput& knobInput = mixerKnob();

// =============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// =============================================================================
#ifndef PIN_LED
  #define PIN_LED             2       // Built-in status LED
#endif

constexpr uint8_t  BLDC_POLE_PAIRS        = 7;       // BlueRobotics T-200 Thruster
constexpr float    TWO_PI_CONST           = 6.283185307179586f;
constexpr float    MAX_THRUSTER_RPM       = 3800.0f; // T-200 rated max forward speed
constexpr float    MAX_THRUSTER_RAD_S     = (MAX_THRUSTER_RPM * TWO_PI_CONST) / 60.0f; // ~397.9 rad/s

// =============================================================================
// VELOCITY STATE (bus-backed snapshot)
// =============================================================================
// The real tachometer engine (ISR edge timing, fault-pulse decoding, moving-average filter)
// and the PWM actuation stage (LEDC + analytical RC linearizer) now live behind the bus in
// mixer_bus_real.cpp. Here we keep a local snapshot so the PID/CLI/telemetry code can keep
// reading `velocity.rad_s`, `velocity.rpm`, ... unchanged.
static MixerVelocity velocity;

static void refreshTachometer() {
    tachometer.update(micros());
    velocity = tachometer.read();
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
    .Kp = 0.00274565f,
    .Ki = 0.00642846f,
    .Kd = 0.0f,
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
    if (velocity.fault_code > 0) return getDriverFaultStr(velocity.fault_code);
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
        actuator.setDuty(0.0f);
        return;
    }

    if (currentControlMode == MODE_OPEN_LOOP_DUTY) {
        actuator.setDuty(openLoopDuty);
        return;
    }

    // --- Stall Fault Recovery ---

    if (inStallRecovery) {
        // Force 0% PWM to clear the driver's Stall Fault
        actuator.setDuty(0.0f);
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

    // Derivative term removed for pure PI operation.
    // (Prevents IEEE-754 0.0 * NaN = NaN poisoning if tachometer ever glitches)
    
    // Feedforward disabled: let the PI loop do 100% of the work to match MATLAB LTI models.
    float ff_term = 0.0f;

    // Raw control output
    float u_raw = p_term + pid.integral + ff_term;
    float u_clamped = constrain(u_raw, 0.0f, 1.0f);

    // Slew rate limiter
    float max_delta = pid.max_slew_rate * pid.Ts;
    float delta = u_clamped - pid.prev_output;
    delta = constrain(delta, -max_delta, max_delta);
    float u_slewed = pid.prev_output + delta;
    pid.prev_output = u_slewed;

    // Apply to hardware PWM
    actuator.setDuty(u_slewed);
}

// =============================================================================
// COMMAND INTERFACE & SERIAL CLI
// =============================================================================
static char pcRxBuffer[64];
static uint8_t pcRxIndex = 0;
static uint32_t lastTelemetryTimeMs = 0;
static bool sysidMode = false;

void printHelp() {
    Serial.println("\n--- T-200 Thruster Controller CLI ---");
    Serial.println("  SPEED <rad/s>       : Closed-loop angular velocity (e.g. 'SPEED 150')");
    Serial.println("  RPM <rpm>           : Closed-loop speed in RPM (e.g. 'RPM 1200')");
    Serial.println("  DUTY <0.0 - 1.0>    : Open-loop throttle override (e.g. 'DUTY 0.35')");
    Serial.println("  STOP                : Safely stop thruster");
    Serial.println("  TUNE <Kp> <Ki> <Kd> : Update PID gains live (e.g. 'TUNE 0.0015 0.004 0.00005')");
    Serial.println("  SYSID ON/OFF        : Toggle 50Hz compact CSV telemetry for System ID");
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
    Serial.printf("  Commanded Duty:     %5.1f%%\n", actuator.duty() * 100.0f);
    Serial.printf("  Compensated PWM:    %5.1f%%\n", actuator.compensatedDuty() * 100.0f);
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
        actuator.setDuty(0.0f);
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
        // Drive PA0 high (transistor ON -> VSP = 0V) via the actuator bus.
        actuator.setDuty(1.0f);
        Serial.println("[TEST] GPIO 13 set to HIGH. If NPN is wired right, motor MUST STOP (VSP=0V).");
    }
    else if (strncasecmp(cmd, "TEST LOW", 8) == 0) {
        currentControlMode = MODE_STOPPED;
        // Drive PA0 low (transistor OFF -> VSP = 5V) via the actuator bus.
        actuator.setDuty(0.0f);
        Serial.println("[TEST] GPIO 13 set to LOW. If NPN is wired right, motor MUST SPIN MAX (VSP=5V).");
    }
    else if (strncasecmp(cmd, "STATUS", 6) == 0) {
        printStatus();
    }
    else if (strncasecmp(cmd, "SYSID ON", 8) == 0) {
        sysidMode = true;
        Serial.println("SYSID_MODE,Time_ms,Target_Duty,Compensated_Duty,Measured_RPM");
    }
    else if (strncasecmp(cmd, "SYSID OFF", 9) == 0) {
        sysidMode = false;
        Serial.println("[OK] SysID Mode OFF. Normal telemetry restored.");
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
// MANUAL KNOB CONTROL LOGIC
// =============================================================================
enum KnobState {
    KNOB_LOCKED = 0,
    KNOB_WAITING_100,
    KNOB_WAITING_0,
    KNOB_ACTIVE
};
static KnobState knobState = KNOB_LOCKED;
static uint32_t knobZeroTimerMs = 0;
static uint32_t lastKnobUpdateMs = 0;

void updateKnobLogic() {
    uint32_t now = millis();
    if (now - lastKnobUpdateMs < 50) return; // 20 Hz update
    lastKnobUpdateMs = now;

    uint16_t rawAdc = knobInput.read();
    float knobPct = (float)rawAdc / 4095.0f; // 12-bit ADC (0.0 to 1.0)

    bool isZero = (knobPct < 0.05f); // 5% deadband at bottom
    bool isFull = (knobPct > 0.95f); // 5% deadband at top

    switch(knobState) {
        case KNOB_LOCKED:
            if (isZero) knobState = KNOB_WAITING_100;
            break;
        case KNOB_WAITING_100:
            if (isFull) knobState = KNOB_WAITING_0;
            break;
        case KNOB_WAITING_0:
            if (isZero) {
                knobState = KNOB_ACTIVE;
                Serial.println("\n[KNOB] Unlocked! Manual Override Active.");
            }
            break;
        case KNOB_ACTIVE:
            if (isZero) {
                if (knobZeroTimerMs == 0) knobZeroTimerMs = now;
                else if (now - knobZeroTimerMs > 5000) { // 5 seconds timeout
                    knobState = KNOB_LOCKED;
                    knobZeroTimerMs = 0;
                    currentControlMode = MODE_STOPPED;
                    Serial.println("\n[KNOB] Relocked (5s at zero). Manual Control Disabled.");
                    return;
                }
            } else {
                knobZeroTimerMs = 0;
            }

            // Command speed proportional to knob position
            float activePct = 0.0f;
            if (knobPct > 0.05f) {
                activePct = (knobPct - 0.05f) / 0.90f; // Scale 5%-95% to 0-100%
                if (activePct > 1.0f) activePct = 1.0f;
            }
            
            // Override with target speed
            pid.target_rad_s = activePct * MAX_THRUSTER_RAD_S;
            currentControlMode = MODE_CLOSED_LOOP_PID;
            break;
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
    actuator.init();

    // 2. Initialize Tachometer Edge Timing Engine (GPIO 4, Reciprocal ISR)
    tachometer.init();

    // 3. Initialize Analog Knob
    knobInput.init();

    // 4. Start Network Tasks (Core 0)
    startCloudWorker();

    // 5. Ready
    printHelp();
}

void loop() {
    // 1. Update pulse timing & speed estimation continuously
    refreshTachometer();

    // 2. Process USB Serial CLI commands
    readUserCommands();

    // 3. Update Knob Logic
    updateKnobLogic();

    // 4. Execute 100 Hz Discrete PID Velocity Loop (every 10,000 us)
    uint32_t nowUs = micros();
    if (nowUs - lastPidTimeUs >= 10000) {
        lastPidTimeUs = nowUs;
        updatePidLoop();
    }

    // 4. Periodic Telemetry Streaming & Web Dashboard Polling
    uint32_t nowMs = millis();
    uint32_t teleDelay = sysidMode ? 20 : 500; // 50 Hz for SysID, 2 Hz normal
    if (nowMs - lastTelemetryTimeMs >= teleDelay) {
        lastTelemetryTimeMs = nowMs;

        if (sysidMode) {
            // Fast compact CSV output for System ID (Strict SI units)
            Serial.printf("SYSID,%lu,%.4f,%.4f,%.4f\n", 
                          nowMs, actuator.duty(), actuator.compensatedDuty(), velocity.rad_s);
        } else {
            float targetRpm = (pid.target_rad_s * 60.0f) / TWO_PI_CONST;
            float errRadS = pid.target_rad_s - velocity.rad_s;

#if DEBUG_TELEMETRY
             Serial.printf("[TELEMETRY] Status: %-11s | RPM: %6.1f / %4.0f | Duty: %4.1f%% (PWM: %4.1f%%) | Err: %+5.2f rad/s | Freq: %5.1f Hz\n",
                           getEscStatusStr(),
                           velocity.rpm,
                           targetRpm,
                           actuator.duty() * 100.0f,
                           actuator.compensatedDuty() * 100.0f,
                           errRadS,
                           velocity.freq_hz);
#endif

            // Fetch any incoming commands from web dashboard
            SharedTelemetry cmd = getSharedCommand();
            static float lastCmdRadS = -1.0f;
            if (cmd.target_rad_s != lastCmdRadS && cmd.target_rad_s >= 0.0f) {
                lastCmdRadS = cmd.target_rad_s;
                pid.target_rad_s = cmd.target_rad_s;
                currentControlMode = MODE_CLOSED_LOOP_PID;
                Serial.printf("[WEB] Received new target: %.2f rad/s\n", pid.target_rad_s);
            }

            // Push current state to network task
            SharedTelemetry telemPush;
            telemPush.target_rpm = targetRpm;
            telemPush.actual_rpm = velocity.rpm;
            telemPush.target_rad_s = pid.target_rad_s;
            telemPush.actual_rad_s = velocity.rad_s;
            telemPush.commanded_duty = actuator.duty();
            telemPush.pso_kp = pid.Kp;
            telemPush.pso_ki = pid.Ki;
            telemPush.pso_kd = pid.Kd;
            telemPush.status_code = velocity.fault_code;
            telemPush.is_running = (currentControlMode != MODE_STOPPED);
            updateSharedTelemetry(telemPush);
        }

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

#pragma once

#include <Arduino.h>

// Shared Telemetry Struct for Core 0 / Core 1 communication
struct SharedTelemetry {
    float target_rpm = 0.0f;
    float actual_rpm = 0.0f;
    float target_rad_s = 0.0f;
    float actual_rad_s = 0.0f;
    float commanded_duty = 0.0f;
    float pso_kp = 0.0f;
    float pso_ki = 0.0f;
    float pso_kd = 0.0f;
    uint32_t status_code = 0; // Or last fault code
    bool is_running = false;
};

// Start the network tasks (WiFi, HTTP Polling, OTA) pinned to Core 0
void startCloudWorker();

// To be called from Core 1 to update telemetry for Core 0 to send
void updateSharedTelemetry(const SharedTelemetry& telem);

// To be called from Core 1 to fetch any commands received by Core 0
SharedTelemetry getSharedCommand();

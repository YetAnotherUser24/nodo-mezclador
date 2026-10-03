#pragma once

#include <Arduino.h>

#define OTA_ENABLED 1

// ---------------------------------------------------------
// WiFi Credentials
// ---------------------------------------------------------
#define WIFI_SSID       "WifiDev"
#define WIFI_PASSWORD   "rpidev24"

// ---------------------------------------------------------
// OTA Configuration
// ---------------------------------------------------------
#define OTA_HOSTNAME    "mixer-t200"
#define OTA_PASSWORD    ""

// ---------------------------------------------------------
// Web Dashboard API
// ---------------------------------------------------------
#define API_BASE_URL    "https://tesisutec.vercel.app" // Production Vercel backend
#define API_TELEMETRY   API_BASE_URL "/api/telemetry/motor"
#define API_COMMANDS    API_BASE_URL "/api/commands/pending"
#define DEVICE_KEY      "ESP32_T_200" // Actual device key in DB

// Polling and Telemetry Intervals
#define TELEMETRY_INTERVAL_MS   3000
#define COMMAND_POLL_INTERVAL_MS 1000

#ifndef DEBUG_TELEMETRY
#define DEBUG_TELEMETRY 1
#endif

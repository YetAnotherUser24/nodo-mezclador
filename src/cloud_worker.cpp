#include "cloud_worker.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#if defined(OTA_ENABLED) && OTA_ENABLED
#include <ArduinoOTA.h>
#endif

static SharedTelemetry s_sharedTelem;
static portMUX_TYPE s_telemMux = portMUX_INITIALIZER_UNLOCKED;

static SharedTelemetry s_sharedCommand;
static portMUX_TYPE s_commandMux = portMUX_INITIALIZER_UNLOCKED;

static unsigned long lastTelemetryPush = 0;
static unsigned long lastCommandPoll = 0;

void updateSharedTelemetry(const SharedTelemetry& telem) {
    portENTER_CRITICAL(&s_telemMux);
    s_sharedTelem = telem;
    portEXIT_CRITICAL(&s_telemMux);
}

static SharedTelemetry getSharedTelemetry() {
    SharedTelemetry copy;
    portENTER_CRITICAL(&s_telemMux);
    copy = s_sharedTelem;
    portEXIT_CRITICAL(&s_telemMux);
    return copy;
}

SharedTelemetry getSharedCommand() {
    SharedTelemetry copy;
    portENTER_CRITICAL(&s_commandMux);
    copy = s_sharedCommand;
    portEXIT_CRITICAL(&s_commandMux);
    return copy;
}

static void updateSharedCommand(const SharedTelemetry& cmd) {
    portENTER_CRITICAL(&s_commandMux);
    s_sharedCommand = cmd;
    portEXIT_CRITICAL(&s_commandMux);
}

void cloudWorkerTask(void* parameter) {
    Serial.print("[WIFI] Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WIFI] Connected!");
        Serial.print("[WIFI] IP Address: ");
        Serial.println(WiFi.localIP());

#if defined(OTA_ENABLED) && OTA_ENABLED
        ArduinoOTA.setHostname(OTA_HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart([]() { Serial.println("\n[OTA] Start updating..."); });
        ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] End"); });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        Serial.println("[OTA] Service initialized.");
#endif
    } else {
        Serial.println("\n[WIFI] Connection failed. Task continuing for OTA fallback/reconnect.");
    }

    while (true) {
        // Reconnect if connection is lost
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
            delay(5000);
            continue;
        }

#if defined(OTA_ENABLED) && OTA_ENABLED
        ArduinoOTA.handle();
#endif

        unsigned long now = millis();

        // 1. Telemetry Push (POST)
        if (now - lastTelemetryPush >= TELEMETRY_INTERVAL_MS) {
            lastTelemetryPush = now;
            SharedTelemetry currentTelem = getSharedTelemetry();

            HTTPClient http;
            http.begin(API_TELEMETRY);
            http.addHeader("Content-Type", "application/json");

            JsonDocument doc;
            doc["target_rpm"] = currentTelem.target_rpm;
            doc["actual_rpm"] = currentTelem.actual_rpm;
            doc["target_rad_s"] = currentTelem.target_rad_s;
            doc["actual_rad_s"] = currentTelem.actual_rad_s;
            doc["commanded_duty"] = currentTelem.commanded_duty;
            doc["kp"] = currentTelem.pso_kp;
            doc["ki"] = currentTelem.pso_ki;
            doc["kd"] = currentTelem.pso_kd;
            doc["status_code"] = currentTelem.status_code;
            doc["is_running"] = currentTelem.is_running;

            String payload;
            serializeJson(doc, payload);
            
            int httpResponseCode = http.POST(payload);
            if (httpResponseCode > 0) {
                // Serial.printf("[HTTP] POST telemetry code: %d\n", httpResponseCode);
            } else {
                Serial.printf("[HTTP] POST telemetry failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
            }
            http.end();
        }

        // 2. Command Poll (GET)
        if (now - lastCommandPoll >= COMMAND_POLL_INTERVAL_MS) {
            lastCommandPoll = now;

            HTTPClient http;
            http.begin(API_COMMANDS);
            
            int httpResponseCode = http.GET();
            if (httpResponseCode == HTTP_CODE_OK) {
                String payload = http.getString();
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);

                if (!error) {
                    SharedTelemetry newCmd = getSharedCommand();
                    if (doc["target_rpm"].is<float>()) {
                        newCmd.target_rpm = doc["target_rpm"].as<float>();
                        // If web sets RPM, convert to rad/s for internal use if needed, but let Core 1 handle conversion
                    }
                    if (doc["target_rad_s"].is<float>()) {
                        newCmd.target_rad_s = doc["target_rad_s"].as<float>();
                    }
                    updateSharedCommand(newCmd);
                } else {
                    Serial.printf("[HTTP] GET deserialize failed: %s\n", error.c_str());
                }
            }
            http.end();
        }

        // Allow idle tasks to run
        delay(10);
    }
}

void startCloudWorker() {
    // Run on Core 0
    xTaskCreatePinnedToCore(
        cloudWorkerTask,     // Task function
        "CloudWorker",       // Task name
        8192,                // Stack size
        NULL,                // Parameters
        1,                   // Priority (lower than control loop if control loop is on Core 1)
        NULL,                // Task handle
        0                    // Core 0
    );
}

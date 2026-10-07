#include "cloud_worker.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>

#if defined(OTA_ENABLED) && OTA_ENABLED
#include <ArduinoOTA.h>
static bool otaInProgress = false;
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
    Serial.print("[WIFI] Initializing for ");
    Serial.println(WIFI_SSID);

    // 1. Forzar modo Estación
    WiFi.mode(WIFI_STA);
    // 2. Delegar la reconexión básica al hardware
    WiFi.setAutoReconnect(true);
    // 3. Estabilidad de hardware
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    // 4. Iniciar (SOLO UNA VEZ)
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long lastWifiRetry = millis();
    bool wasConnected = false;
    bool otaConfigured = false;

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            if (wasConnected) {
                Serial.println("[Red] Desconectado. Auto-reconnect activo...");
                wasConnected = false;
                lastWifiRetry = millis();
            }
            
            // SEGURO DE VIDA: Si el AutoReconnect interno falla por 30 segundos
            if (millis() - lastWifiRetry > 30000) {
                Serial.println("[Red] Pila WiFi atascada. Reiniciando hardware de red...");
                WiFi.disconnect(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
                lastWifiRetry = millis();
            }
        } else {
            // Cuando se reconecta exitosamente
            if (!wasConnected) {
                Serial.printf("[Red] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
                wasConnected = true;

                // Sync time via NTP for precise ACK timestamps
                configTime(0, 0, "pool.ntp.org", "time.nist.gov");
                Serial.println("[NTP] Time synchronization requested.");

#if defined(OTA_ENABLED) && OTA_ENABLED
                if (!otaConfigured) {
                    ArduinoOTA.setHostname(OTA_HOSTNAME);
                    if (strlen(OTA_PASSWORD) > 0) {
                        ArduinoOTA.setPassword(OTA_PASSWORD);
                    }
                    ArduinoOTA.onStart([]() { 
                        otaInProgress = true;
                        Serial.println("\n[OTA] Start updating..."); 
                        SharedTelemetry cmd = getSharedCommand();
                        cmd.target_rad_s = 0.0f;
                        cmd.target_rpm = 0.0f;
                        updateSharedCommand(cmd); // DETIENE MOTORES
                    });
                    ArduinoOTA.onEnd([]() { 
                        otaInProgress = false;
                        Serial.println("\n[OTA] End"); 
                    });
                    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
                    });
                    ArduinoOTA.onError([](ota_error_t error) {
                        otaInProgress = false;
                        Serial.printf("[OTA] Error[%u]: ", error);
                        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                        else if (error == OTA_END_ERROR) Serial.println("End Failed");
                    });
                    ArduinoOTA.begin();
                    Serial.println("[OTA] Service initialized.");
                    otaConfigured = true;
                }
#endif
            }
        }

        // Si no estamos conectados, no hacemos HTTP
        if (!wasConnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

#if defined(OTA_ENABLED) && OTA_ENABLED
        ArduinoOTA.handle();
        // Skip blocking HTTP tasks if OTA is actively receiving a payload
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
#endif

        unsigned long now = millis();

        // 1. Telemetry Push (POST) / Heartbeat
        SharedTelemetry currentTelem = getSharedTelemetry();
        unsigned long currentPushInterval = currentTelem.is_running ? TELEMETRY_INTERVAL_MS : 20000;
        
        if (now - lastTelemetryPush >= currentPushInterval) {
            lastTelemetryPush = now;

            WiFiClientSecure client;
            client.setInsecure(); // Accept any certificate
            HTTPClient http;
            http.setTimeout(5000); // Prevent infinite blocking
            http.begin(client, API_TELEMETRY);
            http.addHeader("Content-Type", "application/json");
            http.addHeader("X-Device-Key", DEVICE_KEY);

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

            WiFiClientSecure client;
            client.setInsecure();
            HTTPClient http;
            http.begin(client, API_COMMANDS);
            http.addHeader("X-Device-Key", DEVICE_KEY);
            
            int httpResponseCode = http.GET();
            if (httpResponseCode == HTTP_CODE_OK) {
                String payload = http.getString();
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);

                if (!error) {
                    if (doc["has_command"].as<bool>() && doc["command"].is<JsonObject>()) {
                        JsonObject cmdObj = doc["command"].as<JsonObject>();
                        SharedTelemetry newCmd = getSharedCommand();
                        
                        if (cmdObj["target_rpm"].is<float>()) {
                            newCmd.target_rpm = cmdObj["target_rpm"].as<float>();
                        }
                        
                        if (cmdObj["target_rad_s"].is<float>()) {
                            newCmd.target_rad_s = cmdObj["target_rad_s"].as<float>();
                        } else if (cmdObj["speed_percent"].is<float>()) {
                            // Fallback if frontend sends speed_percent
                            float speed_pct = cmdObj["speed_percent"].as<float>();
                            newCmd.target_rad_s = (speed_pct / 100.0f) * 397.9f;
                            newCmd.target_rpm = (newCmd.target_rad_s * 60.0f) / (2.0f * 3.14159265f);
                        }
                        updateSharedCommand(newCmd);

                        // Acknowledge the command
                        if (cmdObj["id"].is<const char*>()) {
                            String cmdId = cmdObj["id"].as<String>();
                            String ackUrl = String(API_BASE_URL) + "/api/commands/" + cmdId + "/acknowledge";
                            
                            WiFiClientSecure ackClient;
                            ackClient.setInsecure();
                            HTTPClient ackHttp;
                            ackHttp.begin(ackClient, ackUrl);
                            ackHttp.addHeader("Content-Type", "application/json");
                            ackHttp.addHeader("X-Device-Key", DEVICE_KEY);
                            
                            JsonDocument ackDoc;
                            ackDoc["success"] = true;
                            
                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            uint64_t rtcMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
                            if (tv.tv_sec > 1600000000) {
                                ackDoc["rtc_timestamp_ms"] = rtcMs;
                            }
                            
                            ackDoc["message"] = "Command received by T-200";
                            String ackPayload;
                            serializeJson(ackDoc, ackPayload);
                            
                            int ackCode = ackHttp.POST(ackPayload);
                            if (ackCode <= 0) {
                                Serial.printf("[Cloud] ACK failed: %s\n", ackHttp.errorToString(ackCode).c_str());
                            }
                            ackHttp.end();
                        }
                    }
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

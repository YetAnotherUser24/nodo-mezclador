#include "cloud_worker.h"
#include "config.h"
#include "aqua_link.h"  // schema-v2 transport; expands to nothing unless AQUA_TRANSPORT_MQTT
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

#if !AQUA_TRANSPORT_MQTT
/** V4: identidad del experimento activo para la bitácora de eventos del mixer. */
static char s_mixerExperimentId[40] = "idle";
#endif

/**
 * Velocidad de mezcla por defecto (RPM).
 * El ESC tiene un tope de 1000 RPM (`BLDC_MAX_RPM` en include/uart_protocol.h). El valor anterior
 * forzaba 1500 RPM —por encima del límite del motor— con un `target_rad_s` derivado de 2000 RPM,
 * inconsistente con el propio `target_rpm`. 600 RPM es el 60 % del tope.
 */
static const float MIXER_DEFAULT_RPM = 600.0f;

#if !AQUA_TRANSPORT_MQTT
/** Publica un evento start_mixer/stop_mixer con el instante exacto (epoch UTC ms, ADR-3). */
static void postMixerEvent(const char* eventType) {
    WiFiClientSecure c;
    c.setInsecure();
    HTTPClient h;
    h.setTimeout(4000);
    if (!h.begin(c, API_BASE_URL "/api/events/mixer")) {
        return;
    }
    h.addHeader("Content-Type", "application/json");
    h.addHeader("X-Device-Key", DEVICE_KEY);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t rtcMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);

    JsonDocument d;
    d["experiment_id"] = s_mixerExperimentId;
    d["event_type"] = eventType;
    d["status"] = "ok";
    d["rtc_timestamp_ms"] = rtcMs;

    String p;
    serializeJson(d, p);
    int code = h.POST(p);
    Serial.printf("[Cloud] Mixer event '%s' -> HTTP %d\n", eventType, code);
    h.end();
}
#endif  // !AQUA_TRANSPORT_MQTT

/**
 * @brief Record which experiment the mixer is now serving.
 *
 * One accessor for both transports so the command applier never has to know which one is
 * compiled in. The schema models "no experiment" as JSON null, so `nullptr` clears
 * attribution; the legacy HTTPS body keeps its historical `"idle"` string, which the old
 * dashboard still reads.
 */
static void setMixerExperiment(const char* experimentId) {
#if AQUA_TRANSPORT_MQTT
    aquaLinkSetExperiment(experimentId);
#else
    if (experimentId == nullptr) {
        strncpy(s_mixerExperimentId, "idle", sizeof(s_mixerExperimentId) - 1);
        s_mixerExperimentId[sizeof(s_mixerExperimentId) - 1] = '\0';
        return;
    }
    strncpy(s_mixerExperimentId, experimentId, sizeof(s_mixerExperimentId) - 1);
    s_mixerExperimentId[sizeof(s_mixerExperimentId) - 1] = '\0';
#endif
}

/** @brief Report a firmware-originated mixer event over whichever transport is compiled in. */
static void reportMixerEvent(const char* eventType) {
#if AQUA_TRANSPORT_MQTT
    aquaLinkEvent(eventType, nullptr);
#else
    postMixerEvent(eventType);
#endif
}

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

#if !AQUA_TRANSPORT_MQTT
/** POST the legacy acknowledgement for one command. */
static void acknowledgeHttp(const String& cmdId, bool success) {
    if (cmdId.length() == 0) return;

    String ackUrl = String(API_BASE_URL) + "/api/commands/" + cmdId + "/acknowledge";

    WiFiClientSecure ackClient;
    ackClient.setInsecure();
    HTTPClient ackHttp;
    ackHttp.begin(ackClient, ackUrl);
    ackHttp.addHeader("Content-Type", "application/json");
    ackHttp.addHeader("X-Device-Key", DEVICE_KEY);

    JsonDocument ackDoc;
    ackDoc["success"] = success;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t rtcMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
    if (tv.tv_sec > 1600000000) {
        ackDoc["rtc_timestamp_ms"] = rtcMs;
    }

    ackDoc["message"] = success ? "Command received by T-200" : "unsupported_action";
    String ackPayload;
    serializeJson(ackDoc, ackPayload);

    int ackCode = ackHttp.POST(ackPayload);
    if (ackCode <= 0) {
        Serial.printf("[Cloud] ACK failed: %s\n", ackHttp.errorToString(ackCode).c_str());
    }
    ackHttp.end();
}
#endif  // !AQUA_TRANSPORT_MQTT

/**
 * Apply one command to the shared command slot and report whether it was understood.
 *
 * Extracted from the HTTP poll handler so the polled and pushed paths run the *same* decision.
 * The mixer's rules are the part worth having exactly once: `set_state` moves the actuator only
 * when the server names `mixer` explicitly (the mixing/aeration overlap is experimental,
 * AGENTS.md rule 11), a bare start floors at `MIXER_DEFAULT_RPM`, and `target_rad_s` is always
 * derived from the *effective* `target_rpm` so the two cannot disagree.
 *
 * @param params    the v2 `target` object, or the legacy `payload`
 * @param fallback  the legacy envelope's top level, which carried the target fields before
 *                  they were nested; either object may be unbound
 * @return true when the action was understood. The caller acknowledges with this value, so an
 *         unknown action is reported as failed rather than silently succeeding — a node that
 *         acknowledges a verb it never performed is worse than one that admits it cannot.
 */
static bool applyMixerCommand(const String& cmdId, const String& action,
                              JsonObjectConst params, JsonObjectConst fallback) {
    // The legacy envelope split the parameters across two levels; v2 nests them all under
    // `target`. Reading both with a precedence rule keeps one applier instead of two.
    auto read = [&](const char* key) -> JsonVariantConst {
        JsonVariantConst v = params[key];
        if (v.isUnbound() || v.isNull()) v = fallback[key];
        return v;
    };

    SharedTelemetry newCmd = getSharedCommand();

    if (read("target_rpm").is<float>()) {
        newCmd.target_rpm = read("target_rpm").as<float>();
    }
    // Accept both int and float: the dashboard/backend may send mixer_rpm as a JSON integer.
    if (read("mixer_rpm").is<float>()) {
        newCmd.target_rpm = read("mixer_rpm").as<float>();
    } else if (read("mixer_rpm").is<int>()) {
        newCmd.target_rpm = (float)read("mixer_rpm").as<int>();
    }

    if (read("target_rad_s").is<float>()) {
        newCmd.target_rad_s = read("target_rad_s").as<float>();
    } else if (read("speed_percent").is<float>()) {
        // Fallback if the frontend sends speed_percent instead of a target.
        const float speedPct = read("speed_percent").as<float>();
        newCmd.target_rad_s = (speedPct / 100.0f) * 397.9f;
        newCmd.target_rpm = (newCmd.target_rad_s * 60.0f) / (2.0f * 3.14159265f);
    }

    const String state = read("state").is<const char*>() ? read("state").as<String>() : String("");
    const String mixer = read("mixer").is<const char*>() ? read("mixer").as<String>() : String("");

    bool handled = false;
    bool mixerCommand = false;
    bool mixerOn = false;

    if (action == "start_mixer") {
        handled = true; mixerCommand = true; mixerOn = true;
    } else if (action == "stop_mixer") {
        handled = true; mixerCommand = true; mixerOn = false;
    } else if (action == "stop_experiment") {
        handled = true; mixerCommand = true; mixerOn = false;
    } else if (action == "start_experiment") {
        // Understood, but not a reason to spin the impeller. The orchestrator sends this to
        // every role; rejecting it would make its transition fail on this role and nowhere
        // else, which reads as a broken node. The mixer still only starts when the server
        // also asks for `mixer` explicitly.
        handled = true;
    } else if (action == "set_state") {
        if (state == "IDLE" || state == "MANUAL_OVERRIDE") {
            // Abort any recipe: the mixer must not keep turning on its own.
            handled = true; mixerCommand = true; mixerOn = false;
        } else if (state == "ACTIVE_EXPERIMENT") {
            handled = true;  // valid transition; the actuator is driven only by `mixer`
        } else if (mixer == "on") {
            handled = true; mixerCommand = true; mixerOn = true;
        } else if (mixer == "off") {
            handled = true; mixerCommand = true; mixerOn = false;
        }
        // ACTIVE_EXPERIMENT with no `mixer` field: the actuator is deliberately untouched.
    }

    if (mixerCommand) {
        if (mixerOn) {
            if (newCmd.target_rpm < 100.0f) {
                newCmd.target_rpm = MIXER_DEFAULT_RPM;
            }
            // rad/s coherent with the effective target_rpm.
            newCmd.target_rad_s = (newCmd.target_rpm * 2.0f * 3.14159265f) / 60.0f;
        } else {
            newCmd.target_rpm = 0.0f;
            newCmd.target_rad_s = 0.0f;
        }
    }
    updateSharedCommand(newCmd);

    if (mixerCommand) {
        reportMixerEvent(mixerOn ? "start_mixer" : "stop_mixer");
    }

#if AQUA_TRANSPORT_MQTT
    if (cmdId.length() > 0) {
        aquaLinkAck(cmdId.c_str(), handled, handled ? "applied" : "unsupported_action");
    }
#else
    acknowledgeHttp(cmdId, handled);
#endif

    if (!handled) {
        Serial.printf("[Cloud] Unsupported mixer action '%s'\n", action.c_str());
    }
    return handled;
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

#if AQUA_TRANSPORT_MQTT
        // Serviced here rather than at the top of the loop: this task `continue`s before this
        // point while WiFi is down, and there is no point driving an MQTT client without a
        // network. This is also the only task that touches the transport, which is what the
        // module's thread-safety contract requires.
        aquaLinkLoop();
#endif

        // 1. Telemetría
        SharedTelemetry currentTelem = getSharedTelemetry();
        unsigned long currentPushInterval = currentTelem.is_running ? TELEMETRY_INTERVAL_MS : 20000;

        if (now - lastTelemetryPush >= currentPushInterval) {
            lastTelemetryPush = now;

#if AQUA_TRANSPORT_MQTT
            // Store-and-forward: queue one sample, then try to ship the batch. A broker
            // outage costs latency rather than data, because the batch stays in RAM until
            // it lands — which one blocking POST per interval could not promise.
            struct timeval tv;
            gettimeofday(&tv, NULL);
            const uint64_t unixMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
            aquaLinkQueue(currentTelem, unixMs);
            aquaLinkFlush();
#else
            WiFiClientSecure client;
            client.setInsecure(); // Accept any certificate
            HTTPClient http;
            http.setTimeout(5000); // Prevent infinite blocking
            http.begin(client, API_TELEMETRY);
            http.addHeader("Content-Type", "application/json");
            http.addHeader("X-Device-Key", DEVICE_KEY);

            JsonDocument doc;
            // Alinear con el contrato del dashboard (`motor_telemetry`): `is_on`,
            // `speed_percent` y `pwm_us` son los campos que pinta la tarjeta de motor.
            // El mixer no mide bus (sin INA226), así que V/A/W se omiten (quedan null).
            // `MIXER_MAX_RPM` is the same ceiling the firmware's own percent scaling uses;
            // a second literal here is how the two silently disagree later.
            doc["is_on"] = currentTelem.is_running;
            doc["speed_percent"] = (currentTelem.actual_rpm / MIXER_MAX_RPM) * 100.0f;
            doc["pwm_us"] = 1500 + (int)(currentTelem.commanded_duty * 500.0f);
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
            // V4 ADR-3: epoch UTC ms para alinear la serie del mixer con el sensor y el ODrive.
            {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                if (tv.tv_sec > 1600000000) {
                    doc["rtc_timestamp_ms"] = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
                }
            }

            String payload;
            serializeJson(doc, payload);
            
            int httpResponseCode = http.POST(payload);
            if (httpResponseCode > 0) {
                // Serial.printf("[HTTP] POST telemetry code: %d\n", httpResponseCode);
            } else {
                Serial.printf("[HTTP] POST telemetry failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
            }
            http.end();
#endif  // AQUA_TRANSPORT_MQTT
        }

#if !AQUA_TRANSPORT_MQTT
        // 2. Command Poll (GET)
        // Once MQTT is on, commands are pushed and this 1 Hz GET is pure cost: on the
        // aerator the same poll was ~77% of the node's traffic and nearly every response
        // was empty, so it is compiled out rather than left polling an unused endpoint.
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
                        // A const *view* of the command, not a mutable handle: `JsonObject` has no
                        // `as<>()` member, so asking it for a `JsonObjectConst` does not compile —
                        // and this line only exists in the legacy build, which the MQTT build never
                        // compiles. Converting at the point of capture keeps one form.
                        JsonObjectConst cmdObj = doc["command"].as<JsonObjectConst>();

                        String cmdId = cmdObj["id"].is<const char*>() ? cmdObj["id"].as<String>() : String("");
                        String action = "";
                        if (cmdObj["payload"].is<JsonObject>() && cmdObj["payload"]["action"].is<const char*>()) {
                            action = cmdObj["payload"]["action"].as<String>();
                        }
                        if (action.length() == 0 && cmdObj["command_type"].is<const char*>()) {
                            action = cmdObj["command_type"].as<String>();
                        }

                        // Attribution travels with the command here exactly as it does over
                        // MQTT: set when the envelope names one, cleared only by the actions
                        // that end a run. A bare `start_mixer` must *keep* the current id -- a
                        // UI sending a start has no reason to repeat the experiment it began.
                        JsonObjectConst params = cmdObj["payload"].is<JsonObjectConst>()
                                                     ? cmdObj["payload"].as<JsonObjectConst>()
                                                     : JsonObjectConst();
                        const char* exp = params["experiment_id"].is<const char*>()
                                              ? params["experiment_id"].as<const char*>()
                                              : nullptr;
                        const String stateArg = params["state"].is<const char*>()
                                                    ? params["state"].as<String>()
                                                    : String("");
                        const bool endsRun = action == "stop_experiment" ||
                                             (action == "set_state" && stateArg != "ACTIVE_EXPERIMENT" &&
                                              stateArg != "");
                        if (endsRun) {
                            setMixerExperiment(nullptr);
                        } else if (exp != nullptr && exp[0] != '\0') {
                            setMixerExperiment(exp);
                        }

                        applyMixerCommand(cmdId, action, params, cmdObj);
                    }
                } else {
                    Serial.printf("[HTTP] GET deserialize failed: %s\n", error.c_str());
                }
            }
            http.end();
        }
#endif  // !AQUA_TRANSPORT_MQTT

        // Allow idle tasks to run
        delay(10);
    }
}

void startCloudWorker() {    // Run on Core 0
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

#if AQUA_TRANSPORT_MQTT
/**
 * Translate a schema-v2 command and hand it to the shared applier.
 *
 * The v2 message carries `cmd_id`, `action`, `target` and `exp`. The legacy envelope nested
 * the same information under `command:{id, command_type, payload:{...}}`. Only the shape
 * differs, so this reshapes rather than re-deciding anything — `applyMixerCommand` remains
 * the one place that knows what starting the mixer means.
 *
 * A command with no action is dropped rather than acknowledged: there is nothing to carry
 * out, and acking it would tell the gateway the node acted on an empty verb.
 */
void mixerHandleMqttCommand(const char* payload, size_t length) {
    JsonDocument doc;
    if (deserializeJson(doc, payload, length) != DeserializationError::Ok) {
        Serial.println("[AQUA] command payload was not valid JSON; ignored");
        return;
    }

    String cmdId = doc["cmd_id"].is<const char*>() ? doc["cmd_id"].as<String>() : String("");
    String action = doc["action"].is<const char*>() ? doc["action"].as<String>() : String("");
    if (action.length() == 0) {
        Serial.println("[AQUA] command had no action; ignored");
        return;
    }

    JsonObjectConst target =
        doc["target"].is<JsonObjectConst>() ? doc["target"].as<JsonObjectConst>() : JsonObjectConst();

    const char* exp = doc["exp"].is<const char*>() ? doc["exp"].as<const char*>() : nullptr;

    // Attribution follows the command, because the mixer keeps no record of the experiment
    // anywhere else. Clearing is explicit and only on the actions that end a run: a
    // `start_mixer` arriving without an id mid-run must *keep* the current one, since a UI
    // sending a start has no reason to repeat the experiment it already began.
    const String stateArg = target["state"].is<const char*>() ? target["state"].as<String>() : String("");
    const bool endsRun = action == "stop_experiment" ||
                         (action == "set_state" && stateArg != "ACTIVE_EXPERIMENT" && stateArg != "");
    if (endsRun) {
        aquaLinkSetExperiment(nullptr);
    } else if (exp != nullptr && exp[0] != '\0') {
        aquaLinkSetExperiment(exp);
    }

    applyMixerCommand(cmdId, action, target, JsonObjectConst());
}
#endif  // AQUA_TRANSPORT_MQTT

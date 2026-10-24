#pragma once

#include <Arduino.h>

#define OTA_ENABLED 1

// ---------------------------------------------------------
// WiFi Credentials
// ---------------------------------------------------------
// Guarded so a credential can be injected at build time; see the note in
// nodo-aerador/src/config.h. An unguarded #define here silently wins over the flag.
#ifndef AQUA_WIFI_SSID
#define AQUA_WIFI_SSID  "AquaStation"
#endif
#ifndef AQUA_WIFI_PASSWORD
#define AQUA_WIFI_PASSWORD "AquaPass"
#endif
#define WIFI_SSID       AQUA_WIFI_SSID
#define WIFI_PASSWORD   AQUA_WIFI_PASSWORD

#ifndef AQUA_STATIC_IP
#define AQUA_STATIC_IP          192, 168, 4, 11
#endif
#ifndef AQUA_STATIC_GATEWAY
#define AQUA_STATIC_GATEWAY     0, 0, 0, 0
#endif
#ifndef AQUA_STATIC_SUBNET
#define AQUA_STATIC_SUBNET      255, 255, 255, 0
#endif

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
#define TELEMETRY_INTERVAL_MS   5000
#define COMMAND_POLL_INTERVAL_MS 1000

// ---------------------------------------------------------
// MQTT transport (schema v2)
// ---------------------------------------------------------
// Replaces the HTTPS cloud path above. Kept switchable so the migration is
// revertible by one macro rather than by reflashing older firmware; the legacy
// bodies stay behind `#else`.
//
// Every setting below is individually `#ifndef`-guarded, which is not decoration.
// A bench node has to point at a developer's broker instead of the station's, and
// the alternative is editing this file before every bench session and remembering
// to revert it afterward. Guarding only the macro above does not achieve that:
// skipping the outer guard does not skip the definitions, so `-DAQUA_MQTT_HOST=...`
// loses to the value here or emits a redefinition warning and then loses anyway.
// (That mistake shipped in patch 02 and was only caught by grepping the ELF.)
#ifndef AQUA_TRANSPORT_MQTT
#define AQUA_TRANSPORT_MQTT      1
#endif
/* Tailscale name of the station. mDNS would be nicer, but the ESP32 resolver for
   `<host>.local` is already used for OTA, and resolving two names costs more than
   it saves. */
#ifndef AQUA_MQTT_HOST
#define AQUA_MQTT_HOST           "192.168.4.1"  // Pi del banco (Hotspot AquaStation)
#endif
#ifndef AQUA_MQTT_PORT
#define AQUA_MQTT_PORT           1883
#endif
/* The Pi's broker runs `allow_anonymous false`, so each node carries its own
   credential (`/etc/aqua/mqtt-node-credentials` on the station). The user is the role
   name; the password is per-node. Previously left blank for the isolated bench, which
   worked only while the broker allowed anonymous -- once it did not, the broker kept
   answering `disconnected, not authorised` and the node looked dead. */
#ifndef AQUA_MQTT_USER
#define AQUA_MQTT_USER           "mixer"
#endif
#ifndef AQUA_MQTT_PASSWORD
#define AQUA_MQTT_PASSWORD       "0zbPwvIrH7J4yJ2PNtRSOnya"
#endif
#ifndef AQUA_MQTT_KEEPALIVE_S
#define AQUA_MQTT_KEEPALIVE_S    30
#endif
/* Bound for how long the library waits for a broker response. PubSubClient defaults
   to 15 s and busy-waits while it does, which starves the idle task and trips the
   watchdog on a broker that accepts the connection but never answers. */
#ifndef AQUA_MQTT_SOCKET_TIMEOUT_S
#define AQUA_MQTT_SOCKET_TIMEOUT_S 5
#endif
/* Unique per node. Two clients sharing an id fight over one session and the broker
   disconnects them in a loop, which the old heartbeat heuristic could not tell
   apart from a healthy node. */
#ifndef AQUA_MQTT_CLIENT_ID
#define AQUA_MQTT_CLIENT_ID      "aqua-mixer"
#endif
/* Schema v2 header plus a 9-field mixer batch is well under 1 KB, but the envelope
   limit is 8 KB and PubSubClient refuses a publish larger than its buffer. Sized to
   match so a full batch fits. */
#ifndef AQUA_MQTT_BUFFER_BYTES
#define AQUA_MQTT_BUFFER_BYTES   8192
#endif
/* Retry cadence, so a missing broker backs off instead of busy-looping. */
#ifndef AQUA_MQTT_RETRY_MS
#define AQUA_MQTT_RETRY_MS       5000UL
#endif

/* Stall confirmation window, matching the local loop's detector in esp32s3_main.cpp:
   commanded to spin while the tachometer reports under ~50 rpm for this long. */
#ifndef MIXER_STALL_RPM
#define MIXER_STALL_RPM          50.0f
#endif
#ifndef MIXER_MAX_RPM
#define MIXER_MAX_RPM            3800.0f  // T-200 ceiling, used for percent scaling
#endif

#ifndef DEBUG_TELEMETRY
#define DEBUG_TELEMETRY 1
#endif

/**
 * Schema-v2 transport for the mixer node. See `aqua_link.h` for the contract and why.
 *
 * The mixer's telemetry is a snapshot of a motor controller rather than a sensor reading, so
 * two of its eight fields are derived rather than measured:
 *
 * - `drv_running` is the control loop's own view, not "duty is non-zero". A commanded 0 % and a
 *   stopped motor are the same reading otherwise, and the difference is what an operator is
 *   looking for.
 * - `drv_stall` comes from the local detector in `esp32s3_main.cpp` (commanded to spin, tacho
 *   under 50 rpm for 500 ms). It is recomputed there and copied through `SharedTelemetry`
 *   rather than re-derived here: two definitions of "stalled" would eventually disagree, and the
 *   one on the wire is the one nobody would be looking at.
 */

#include "aqua_link.h"

#if AQUA_TRANSPORT_MQTT

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_random.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <ArduinoJson.h>

#include "aqua_protocol.h"

namespace {

/** One queued snapshot. A copy, not a reference: core 1 keeps writing while this is held. */
struct Queued {
  SharedTelemetry state;
  uint64_t unixMs;
};

WiFiClient s_net;
PubSubClient s_mqtt(s_net);

char s_buffer[aqua::AQUA_ENVELOPE_MAX_BYTES];
aqua::EnvelopeBuilder s_builder(aqua::Role::Mixer, 0, s_buffer, sizeof(s_buffer));

Queued s_batch[aqua::AQUA_ENVELOPE_BATCH_LIMIT];
size_t s_queued = 0;

uint32_t s_sequence = 0;
uint32_t s_dropped = 0;
uint32_t s_bootId = 0;
uint32_t s_nextAttemptMs = 0;
uint64_t s_lastUnixMs = 0;
bool s_presencePublished = false;
bool s_everConnected = false;
uint32_t s_connectedAtMs = 0;
bool s_configured = false;

/**
 * Newest `aqua/time` broadcast (`docs/TIME-SYNC.md` tier 3) and the uptime it arrived at.
 *
 * Kept with the monotonic reading rather than used as-is, so it can be carried forward. The
 * design exists for the case with an AP and a broker but no internet, where this is the only
 * thing that can say what time it is; a sample stamped 0 is worse than one stamped from a
 * slightly drifting extrapolation.
 */
uint64_t s_broadcastMs = 0;
uint32_t s_broadcastAtMs = 0;
bool s_haveBroadcast = false;

/**
 * Experiment the samples and events in flight belong to, or empty for none.
 *
 * Held as a copy because the caller's buffer can be rewritten between queueing and flushing.
 * The legacy path kept the literal "idle" here, which is not the same claim as null: it gives
 * the gateway a run to file idle data under, and that run then shows up in an experiment list.
 */
char s_experimentId[40] = {0};

/** Provenance of the device clock, decided once per flush from the newest sample. */
aqua::TimeSource timeSourceFor(uint64_t unixSeconds) {
  return unixSeconds >= aqua::kPlausibleEpochS ? aqua::TimeSource::Ntp : aqua::TimeSource::Monotonic;
}

void logLine(const char* message) { Serial.println(message); }

void logf(const char* format, ...) {
  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

/** `aqua/presence/<node_id>` — one topic per node, retained. */
bool presenceTopic(char* out, size_t capacity) {
  return aqua::presenceTopic(aqua::Role::Mixer, out, capacity);
}

/**
 * Publish presence, retained.
 *
 * Retained because presence is state, not an event: a dashboard that connects later still has to
 * learn the node exists without waiting for the next reconnect.
 */
void publishPresence(bool online, const char* reason) {
  char topic[80];
  if (!presenceTopic(topic, sizeof(topic))) return;

  char payload[176];
  const int written = snprintf(payload, sizeof(payload),
                               "{\"v\":2,\"node_id\":\"%s\",\"role\":\"mixer\",\"online\":%s,"
                               "\"reason\":\"%s\",\"boot_id\":%lu}",
                               aqua::nodeId(aqua::Role::Mixer), online ? "true" : "false", reason,
                               (unsigned long)s_bootId);
  if (written <= 0 || (size_t)written >= sizeof(payload)) return;

  // The four-argument form takes `(topic, const uint8_t*, length, retained)`, so the payload has
  // to be cast *and* the length passed. The three-argument `(topic, const char*, retained)`
  // overload is the one that traps: with a `uint8_t*` and a bare `false` the compiler picks the
  // *length* form instead and publishes zero bytes. Here the argument count forces the right
  // overload, which is why this call is safe where the ack's was not.
  s_mqtt.publish(topic, (const uint8_t*)payload, (size_t)written, /*retained=*/true);
  s_presencePublished = true;
}

/** One sample's fields. Always 9: the per-sample time plus the 8 the registry declares. */
size_t buildFields(const Queued& entry, aqua::FieldValue* out) {
  namespace f = aqua::mixer_fields;
  const SharedTelemetry& st = entry.state;

  size_t n = 0;
  // The sample's own instant, first and always. A batch spans time, so an envelope-level
  // timestamp alone puts every sample in the batch on one millisecond and a plot draws one
  // point where it was given five.
  out[n++] = aqua::FieldValue::integer64(aqua::AQUA_SAMPLE_TIME_FIELD,
                                         static_cast<long long>(entry.unixMs));
  out[n++] = aqua::FieldValue::num(f::kRpm, st.actual_rpm, 1);
  out[n++] = aqua::FieldValue::num(f::kRpmTarget, st.target_rpm, 1);
  out[n++] = aqua::FieldValue::num(f::kRadS, st.actual_rad_s, 3);
  out[n++] = aqua::FieldValue::num(f::kRadSTarget, st.target_rad_s, 3);
  out[n++] = aqua::FieldValue::num(f::kDutyPct, st.commanded_duty, 2);
  out[n++] = aqua::FieldValue::flag(f::kRunning, st.is_running);
  out[n++] = aqua::FieldValue::flag(f::kStall, st.stall);
  // The raw fault word passes through untouched. Interpreting ESCs' bit layouts here would
  // duplicate knowledge that belongs to the controller, and a wrong interpretation is worse
  // than none.
  out[n++] = aqua::FieldValue::integer(f::kStatus, static_cast<long>(st.status_code));
  return n;
}

}  // namespace

/* ------------------------------- Inbound -------------------------------- */

/**
 * Has this command's deadline already passed?
 *
 * The gateway stamps `deadline_ms` when it issues a command, and nothing on the device ever
 * read it. A command that arrived late -- queued while the session was down, or redelivered --
 * was carried out exactly as if it had just been requested. An operator's intent expires, and
 * acting after it does is not neutral: it moves an actuator for a reason nobody is waiting on.
 *
 * Enforced here rather than in each node's handler, so all four nodes behave the same.
 *
 * Skipped outright when the device clock is not anchored (before 2020). Comparing an epoch
 * deadline against an unanchored clock would reject *every* command, which is a worse failure
 * than the one being fixed.
 *
 * @return true when the command is stale and must not be applied
 */
/**
 * Best-known epoch milliseconds, or 0 when no clock is anchored.
 *
 * Prefers the node's own clock: NTP puts it within milliseconds of real time, and the retained
 * broadcast is the fallback for when there is no internet. The order mirrors
 * `docs/TIME-SYNC.md` -- the point being that a sample is never stamped 0 merely because one
 * source is missing.
 */
uint64_t aquaLinkNowMs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t local = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
  if (local >= aqua::kPlausibleEpochS * 1000ULL) return local;
  if (s_haveBroadcast) return aqua::extrapolateEpochMs(s_broadcastMs, s_broadcastAtMs, millis());
  return 0;
}

/**
 * Record an `aqua/time` broadcast.
 *
 * Rejected if it is not plausible, so a retained message left behind by a node with an
 * unanchored clock cannot drag this one down with it.
 */
static void acceptTimeBroadcast(const char* payload, size_t length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;
  if (!doc["t_dev_ms"].is<long long>()) return;

  const long long ms = doc["t_dev_ms"].as<long long>();
  if (!aqua::plausibleEpochMs(ms)) return;

  const char* src = doc["t_src"].is<const char*>() ? doc["t_src"].as<const char*>() : "?";
  s_broadcastMs = (uint64_t)ms;
  s_broadcastAtMs = millis();
  s_haveBroadcast = true;
  logf("[AQUA] time broadcast %lld ms (t_src=%s)\n", ms, src);
}

static bool commandExpired(const char* payload, size_t length) {
  // 768, not 256. The command envelope is ~230 bytes with a nested `target`, and a document
  // too small to hold it does not truncate -- it fails to parse. That failure used to return
  // false here, which silently disabled the whole check while every layer still looked right.
  // A probe that cannot read the deadline must say so rather than pretend there isn't one.
  StaticJsonDocument<768> probe;
  const DeserializationError err = deserializeJson(probe, payload, length);
  if (err) {
    logf("[AQUA] deadline probe could not parse (%s); applying\n", err.c_str());
    return false;
  }
  if (!probe["deadline_ms"].is<long long>()) return false;

  const long long deadlineMs = probe["deadline_ms"].as<long long>();

  // The rule itself lives in the shared library: whether this command is stale is one decision,
  // and a copy per node is how the four came to disagree. Only the clock differs per node.
  const uint64_t now = aquaLinkNowMs();
  if (!aqua::deadlinePassed(deadlineMs, now)) return false;
  const long long nowMs = (long long)now;

  const char* cmdId = probe["cmd_id"].is<const char*>() ? probe["cmd_id"].as<const char*>() : "";
  logf("[AQUA] command %s expired %lld ms ago; rejected\n", cmdId, nowMs - deadlineMs);
  if (cmdId[0] != '\0') aquaLinkAck(cmdId, false, "deadline_expired");
  return true;
}

static void onMessage(char* topic, uint8_t* payload, unsigned int length) {
  // The topic is carried into the log because a command that is dropped silently and a command
  // that was never delivered look identical from the handler's point of view.
  logf("[AQUA] rx %s (%u bytes)\n", topic ? topic : "?", (unsigned)length);

  // Route by topic. `aqua/time` is a retained broadcast, not a command -- handing it to the
  // command handler would have it report "no action" at best, and at worst bury a real time
  // message in the noise that the broker's retained-message behaviour already creates.
  if (topic != nullptr && strcmp(topic, aqua::AQUA_TIME_TOPIC) == 0) {
    acceptTimeBroadcast((const char*)payload, (size_t)length);
    return;
  }

  if (commandExpired((const char*)payload, (size_t)length)) return;
  mixerHandleMqttCommand((const char*)payload, (size_t)length);
}

/* ------------------------------- Connect -------------------------------- */

static bool connectOnce() {
  char willTopic[80];
  if (!presenceTopic(willTopic, sizeof(willTopic))) return false;

  // The will is what makes a node that dies mid-spin visible immediately rather than after the
  // dashboard's staleness timeout. A mixer left running by a crashed node is a physical hazard,
  // so "unknown" is the one state this must not be able to reach.
  char willPayload[176];
  const int willWritten =
      snprintf(willPayload, sizeof(willPayload),
               "{\"v\":2,\"node_id\":\"%s\",\"role\":\"mixer\",\"online\":false,"
               "\"reason\":\"lwt\",\"boot_id\":%lu}",
               aqua::nodeId(aqua::Role::Mixer), (unsigned long)s_bootId);
  if (willWritten <= 0 || (size_t)willWritten >= sizeof(willPayload)) return false;

  const char* user = AQUA_MQTT_USER[0] ? AQUA_MQTT_USER : nullptr;
  const char* pass = AQUA_MQTT_PASSWORD[0] ? AQUA_MQTT_PASSWORD : nullptr;

  if (!s_mqtt.connect(AQUA_MQTT_CLIENT_ID, user, pass, willTopic, /*willQos=*/1,
                      /*willRetain=*/true, willPayload)) {
    return false;
  }

  char cmdTopic[64];
  if (aqua::commandTopic(aqua::Role::Mixer, cmdTopic, sizeof(cmdTopic))) {
    s_mqtt.subscribe(cmdTopic, 1);
  }
  // Tier 3 of `docs/TIME-SYNC.md`. Retained, so it arrives immediately on subscribe -- but only
  // if something has published it, which is why the node also keeps its own NTP fallback.
  s_mqtt.subscribe(aqua::AQUA_TIME_TOPIC, 1);

  s_presencePublished = false;  // republished by the next loop so a restart is visible
  return true;
}

/* -------------------------------- Public -------------------------------- */

bool aquaLinkBegin() {
  if (s_configured) return true;
  if (WiFi.status() != WL_CONNECTED) return false;

  s_bootId = esp_random() | 1u;

  s_mqtt.setServer(AQUA_MQTT_HOST, AQUA_MQTT_PORT);
  s_mqtt.setBufferSize(AQUA_MQTT_BUFFER_BYTES);
  s_mqtt.setKeepAlive(AQUA_MQTT_KEEPALIVE_S);
  // Bounded explicitly. The library default is 15 s of busy-waiting for a CONNACK, which is long
  // enough to starve the idle task and trip the watchdog when a broker accepts the TCP
  // connection and then says nothing. Recovery from a dead broker is the case this path exists
  // for, so it must not be the case that resets the node — and a reset discards the pending
  // batch, which is the data an outage makes worth keeping.
  s_mqtt.setSocketTimeout(AQUA_MQTT_SOCKET_TIMEOUT_S);
  s_mqtt.setCallback(onMessage);

  s_configured = true;
  logf("[AQUA] mqtt %s:%d as %s\n", AQUA_MQTT_HOST, AQUA_MQTT_PORT, AQUA_MQTT_CLIENT_ID);
  return true;
}

void aquaLinkLoop() {
  // Lazy init, so WiFi coming up by any route leads here without every path remembering to call
  // aquaLinkBegin().
  if (!s_configured) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!aquaLinkBegin()) return;
  }

  if (!s_mqtt.connected()) {
    const uint32_t now = millis();
    // Signed-difference compare so this survives the millis() rollover at ~49 days.
    if (static_cast<int32_t>(now - s_nextAttemptMs) >= 0) {
      if (connectOnce()) {
        s_nextAttemptMs = now;
        s_connectedAtMs = now;
        s_everConnected = true;
        logLine("[AQUA] mqtt connected");
      } else {
        s_nextAttemptMs = now + AQUA_MQTT_RETRY_MS;
      }
    }
    return;
  }

  s_mqtt.loop();

  if (!s_presencePublished) {
    publishPresence(true, s_everConnected ? "reconnect" : "boot");
  }
}

bool aquaLinkConnected() { return s_configured && s_mqtt.connected(); }

void aquaLinkQueue(const SharedTelemetry& state, uint64_t unixMs) {
  if (s_queued >= aqua::AQUA_ENVELOPE_BATCH_LIMIT) {
    // Refuse the newest rather than evict the oldest. Evicting would silently discard the start
    // of a window; refusing loses one sample and shows up as a sequence gap.
    return;
  }

  Queued& entry = s_batch[s_queued];
  entry.state = state;
  entry.unixMs = unixMs;
  s_lastUnixMs = unixMs;
  ++s_queued;
}

bool aquaLinkFlush() {
  if (s_queued == 0) return true;
  if (!s_mqtt.connected()) return false;  // caller keeps the batch and retries

  ++s_sequence;  // one number per envelope, never reused
  // Null rather than an empty string or the legacy "idle": the format and the gateway both read
  // null as "no experiment", and a sentinel would create a run to group rows under.
  s_builder.setExperiment(s_experimentId[0] != '\0' ? s_experimentId : nullptr);
  if (!s_builder.begin(s_sequence)) {
    logf("[AQUA] flush: begin() refused seq %lu\n", (unsigned long)s_sequence);
    ++s_dropped;
    s_queued = 0;
    return false;
  }

  aqua::FieldValue fields[9];
  size_t fieldCount = 0;
  for (size_t i = 0; i < s_queued; ++i) {
    fieldCount = buildFields(s_batch[i], fields);
    if (!s_builder.addSample(fields, fieldCount)) {
      // A truncated envelope is invalid JSON, so the gateway would reject the whole batch with no
      // way to say which node produced it. Discard it and let the sequence gap report it.
      logf("[AQUA] flush: envelope overflow at sample %u (%u fields)\n", (unsigned)i,
           (unsigned)fieldCount);
      ++s_dropped;
      s_queued = 0;
      return false;
    }
  }

  const uint64_t unixS = s_batch[s_queued - 1].unixMs / 1000ULL;
  const size_t length = s_builder.finish(s_lastUnixMs, timeSourceFor(unixS), WiFi.RSSI());
  if (length == 0) {
    logf("[AQUA] flush: finish() produced 0 bytes (%u samples, %u fields each)\n", (unsigned)s_queued,
         (unsigned)fieldCount);
    ++s_dropped;
    s_queued = 0;
    return false;
  }

  char topic[64];
  if (!aqua::telemetryTopic(aqua::Role::Mixer, topic, sizeof(topic))) {
    logLine("[AQUA] flush: telemetryTopic() failed");
    ++s_dropped;
    s_queued = 0;
    return false;
  }

  if (!s_mqtt.publish(topic, (const uint8_t*)s_buffer, length, /*retained=*/false)) {
    // Queue retained: a broker outage costs latency, not data. The sequence number is spent, so
    // the gateway sees a gap rather than a silent hole.
    logf("[AQUA] flush: publish refused (%u bytes) seq %lu\n", (unsigned)length,
         (unsigned long)s_sequence);
    return false;
  }

  logf("[AQUA] flush ok: %u bytes, %u samples, seq %lu\n", (unsigned)length, (unsigned)s_queued,
       (unsigned long)s_sequence);
  s_queued = 0;
  return true;
}

size_t aquaLinkQueued() { return s_queued; }
uint32_t aquaLinkDropped() { return s_dropped; }
bool aquaLinkPresencePublished() { return s_presencePublished; }

void aquaLinkSetExperiment(const char* experimentId) {
  if (experimentId == nullptr || experimentId[0] == '\0') {
    s_experimentId[0] = '\0';
    return;
  }
  // Bounded: the id arrives over the wire, and a truncated id is still a usable grouping key
  // whereas an unbounded copy is a buffer overflow.
  snprintf(s_experimentId, sizeof(s_experimentId), "%s", experimentId);
}

/**
 * The experiment id as a JSON fragment: a quoted string, or the literal `null`.
 *
 * The id arrives over the wire, so it is not assumed safe. Anything outside the characters an
 * identifier has any business containing is rejected rather than escaped — silently rewriting an
 * id would produce a label that matches nothing in the registry, which is worse than admitting
 * the id was unusable. The event is still published, just without attribution.
 */
static const char* experimentJsonFragment(char* scratch, size_t capacity) {
  if (s_experimentId[0] == '\0') return "null";
  for (const char* p = s_experimentId; *p != '\0'; ++p) {
    const char c = *p;
    const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                      c == '-' || c == '_' || c == '.' || c == ':';
    if (!safe) return "null";
  }
  const int written = snprintf(scratch, capacity, "\"%s\"", s_experimentId);
  if (written <= 0 || (size_t)written >= capacity) return "null";
  return scratch;
}

void aquaLinkEvent(const char* eventType, const char* detail) {
  if (!s_mqtt.connected() || eventType == nullptr) return;

  char topic[64];
  if (!aqua::eventsTopic(aqua::Role::Mixer, topic, sizeof(topic))) return;

  char experiment[48];
  const char* experimentField = experimentJsonFragment(experiment, sizeof(experiment));

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);

  static constexpr size_t kEventMax = 384;
  char payload[kEventMax];
  // Matches what the gateway's event handler reads: `event_type`, `node_id`, `experiment_id`,
  // `t_dev_ms`, `detail`. Type-specific content goes under `detail`, which is where it looks.
  const int written = snprintf(payload, sizeof(payload),
                               "{\"v\":2,\"event_type\":\"%s\",\"node_id\":\"%s\","
                               "\"experiment_id\":%s,\"t_dev_ms\":%llu,\"detail\":%s}",
                               eventType, aqua::nodeId(aqua::Role::Mixer), experimentField,
                               (unsigned long long)ms,
                               (detail != nullptr && detail[0] != '\0') ? detail : "{}");
  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    // A truncated event must not be published: a correct event with a mangled body is worse than
    // a missing one, because it reads as authoritative.
    logLine("[AQUA] event payload too long; not sent");
    return;
  }

  s_mqtt.publish(topic, (const uint8_t*)payload, (size_t)written, /*retained=*/false);
}

void aquaLinkAck(const char* cmdId, bool applied, const char* detail) {
  if (!s_mqtt.connected() || cmdId == nullptr || cmdId[0] == '\0') return;

  char topic[64];
  if (!aqua::ackTopic(aqua::Role::Mixer, topic, sizeof(topic))) return;

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const uint64_t ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);

  char payload[256];
  // The gateway maps `result` onto an event type and only understands applied / rejected /
  // failed / expired. Anything else is filed as a protocol error, which is the right outcome for
  // an unknown word rather than silently counting it as success.
  const int written =
      snprintf(payload, sizeof(payload),
               "{\"cmd_id\":\"%s\",\"result\":\"%s\",\"node_id\":\"%s\",\"t_dev_ms\":%llu,"
               "\"detail\":\"%s\"}",
               cmdId, applied ? "applied" : "rejected", aqua::nodeId(aqua::Role::Mixer),
               (unsigned long long)ms, detail ? detail : "");
  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    logLine("[AQUA] ack payload too long; not sent");
    return;
  }

  // The length is passed explicitly, and that is not tidiness. PubSubClient overloads on the
  // payload pointer type: `publish(topic, const char*, bool)` reads the third argument as
  // `retained`, while `publish(topic, const uint8_t*, unsigned int)` reads it as the length.
  // Casting to `uint8_t*` therefore selects the *length* overload, and passing `false` gives
  // length 0 — an empty publish. It compiles, the call reads as correct, and the node simply
  // never acknowledges anything: the gateway drops the empty payload on JSON decode and every
  // command sits at 'published' forever. Patch 02 shipped that bug on the pump.
  if (!s_mqtt.publish(topic, (const uint8_t*)payload, (size_t)written, /*retained=*/false)) {
    logLine("[AQUA] ack publish refused");
  }
}

#endif  // AQUA_TRANSPORT_MQTT

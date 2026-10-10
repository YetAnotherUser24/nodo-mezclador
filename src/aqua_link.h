#pragma once

/**
 * Schema-v2 MQTT transport for the mixer node.
 *
 * Same shape as `nodo-sensor-od`'s and `nodo-bomba`'s: a small transport that the existing
 * cloud task diverts into, leaving the legacy HTTPS bodies in place behind `#else` so the
 * migration is revertible with one macro.
 *
 * Brings the node onto `aqua/tlm/mixer` + `aqua/cmd/mixer` + `aqua/ack/mixer` + `aqua/event/mixer`,
 * which is what lets the gateway push commands instead of having each node poll a shared Vercel
 * endpoint. Two properties are deliberately carried over from the earlier patches because both
 * were found the hard way:
 *
 * - **Every sample carries its own `t_dev_ms`.** A batch spans time, and one envelope timestamp
 *   for five samples made a plot collapse them onto a single instant.
 * - **The acknowledgement is published with an explicit length.** PubSubClient overloads on the
 *   payload pointer type, and `publish(topic, (const uint8_t*)buf, false)` selects the
 *   `(payload, length)` overload — an empty publish that compiles and reads correctly. The
 *   gateway drops it on JSON decode, so the node silently never acknowledges anything.
 */

#include <stddef.h>
#include <stdint.h>

#include "config.h"  // before any guard: AQUA_TRANSPORT_MQTT is resolved here

#if AQUA_TRANSPORT_MQTT

#include "cloud_worker.h"

/**
 * @brief Connect on first use. Safe to call from the cloud task every iteration.
 *
 * Does nothing until WiFi is up, so it can be called unconditionally.
 *
 * @return true once the client is configured
 */
bool aquaLinkBegin();

/** Retry the connection if needed, keep the session alive, publish presence. Non-blocking. */
void aquaLinkLoop();

/** @return true when the client is configured and holds a live session. */
bool aquaLinkConnected();

/**
 * @brief Queue one telemetry snapshot.
 *
 * @param state         the telemetry as the control loop last published it
 * @param unixMs        the sample's own instant, epoch milliseconds UTC
 */
void aquaLinkQueue(const SharedTelemetry& state, uint64_t unixMs);

/**
 * @brief Publish everything queued as one envelope.
 *
 * On failure the batch is retained, so a broker outage costs latency rather than data.
 *
 * @return true if the envelope was published
 */
bool aquaLinkFlush();

/** @return samples currently held for the next envelope. */
size_t aquaLinkQueued();

/** @return envelopes discarded since boot, for the node's own diagnostics. */
uint32_t aquaLinkDropped();

/** @return true once presence has been published for the current session. */
bool aquaLinkPresencePublished();

/**
 * @brief Record which experiment subsequent telemetry and events belong to.
 *
 * A mixer run is only meaningful in the context of the experiment it served, and the schema
 * models "no experiment" as JSON null — so null/"" clears attribution rather than storing a
 * sentinel like "idle", which would invent a run to group rows under.
 *
 * @param experimentId id from `start_experiment`, or null/"" when none is active
 */
void aquaLinkSetExperiment(const char* experimentId);

/**
 * @brief Publish a firmware-originated event.
 *
 * @param eventType  name the gateway maps to an `EventType`
 * @param detail     JSON object body, or null for none
 */
void aquaLinkEvent(const char* eventType, const char* detail);

/**
 * @brief Acknowledge a command on `aqua/ack/mixer`.
 *
 * @param cmdId    the `cmd_id` from the command
 * @param applied  true for `applied`, false for `rejected`
 * @param detail   short human-readable context, or null
 */
void aquaLinkAck(const char* cmdId, bool applied, const char* detail);

/**
 * @brief Best-known epoch milliseconds, or 0 when no clock is anchored.
 *
 * Prefers the node's own NTP-synced clock and falls back to the retained `aqua/time` broadcast
 * (`docs/TIME-SYNC.md` tier 3) carried forward on the monotonic clock. A return of 0 means
 * nothing is anchored, and the caller should stamp its samples as monotonic rather than pretend
 * they are epoch time.
 */
uint64_t aquaLinkNowMs();

/** Signature the cloud worker calls when a command arrives. Implemented in `cloud_worker.cpp`. */
void mixerHandleMqttCommand(const char* payload, size_t length);

#endif  // AQUA_TRANSPORT_MQTT

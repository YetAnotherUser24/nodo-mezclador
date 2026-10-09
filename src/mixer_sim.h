#pragma once
/**
 * @file mixer_sim.h
 * @brief Declarative model of the SNR8503M driver + T-200 thruster. Host-testable (no Arduino).
 *
 * Models the *device*: duty -> steady-state rpm with a first-order spin-up, a deadband below
 * MOTOR_MIN_SPIN_DUTY, and the FG fault-pulse codes the real tachometer decodes. Faults are
 * data-driven via a schedule.
 */

#include <stdint.h>

/** FG fault-pulse sequence the real firmware decodes (pulse count -> code). */
struct MixerSimFaultEvent {
  uint32_t atMs;
  uint8_t code;  // reported as MixerVelocity::fault_code
};

struct MixerSimSpec {
  float maxRpm = 3800.0f;          // T-200 free-run ceiling (electrical/mechanical)
  float minSpinDuty = 0.25f;       // Deadband: below this the motor does not spin
  float maxAllowedDuty = 0.989f;   // Firmware's upper duty clamp
  float spinUpTauS = 0.45f;        // First-order velocity time constant (s)
  uint8_t polePairs = 7;           // FG pulses per revolution
};

class MixerSim {
public:
  explicit MixerSim(const MixerSimSpec& spec = MixerSimSpec());

  void reset();
  void setSpec(const MixerSimSpec& spec) { _spec = spec; }

  /** Command a throttle duty in [0, 1] (already clamped by the caller's model). */
  void setDuty(float duty);

  /** Advance the model. Deterministic. */
  void tick(uint32_t nowMs);

  float rpm() const { return _rpm; }
  float duty() const { return _duty; }
  uint8_t faultCode() const { return _faultCode; }

  void addFaultEvent(const MixerSimFaultEvent& ev);
  void clearFaultSchedule() { _faultCount = 0; }
  void injectFault(uint8_t code) { _faultCode = code; _rpm = 0.0f; }

  static constexpr uint8_t kMaxFaults = 8;

private:
  MixerSimSpec _spec;
  float _duty = 0.0f;
  float _rpm = 0.0f;
  uint8_t _faultCode = 0;
  uint32_t _lastMs = 0;
  uint32_t _faultFired = 0;
  MixerSimFaultEvent _faults[kMaxFaults] = {};
  uint8_t _faultCount = 0;
};

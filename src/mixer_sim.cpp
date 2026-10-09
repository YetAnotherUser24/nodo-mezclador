#include "mixer_sim.h"

#include <math.h>

MixerSim::MixerSim(const MixerSimSpec& spec) : _spec(spec) { reset(); }

void MixerSim::reset() {
  _duty = 0.0f;
  _rpm = 0.0f;
  _faultCode = 0;
  _lastMs = 0;
  _faultFired = 0;
}

void MixerSim::setDuty(float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  _duty = duty;
}

void MixerSim::addFaultEvent(const MixerSimFaultEvent& ev) {
  if (_faultCount < kMaxFaults) _faults[_faultCount++] = ev;
}

void MixerSim::tick(uint32_t nowMs) {
  if (_lastMs == 0) {
    _lastMs = nowMs;
    return;
  }
  float dt = (float)(nowMs - _lastMs) / 1000.0f;
  _lastMs = nowMs;
  if (dt <= 0.0f) return;
  if (dt > 0.5f) dt = 0.5f;

  // Data-driven faults fire once.
  for (uint8_t i = 0; i < _faultCount; ++i) {
    const bool fired = (_faultFired & (1u << i)) != 0;
    if (!fired && nowMs >= _faults[i].atMs) {
      _faultFired |= (1u << i);
      injectFault(_faults[i].code);
    }
  }

  // Deadband: below minSpinDuty the ESC does not spin the motor.
  float targetRpm = 0.0f;
  if (_duty >= _spec.minSpinDuty && _faultCode == 0) {
    // Map [minSpinDuty .. maxAllowedDuty] onto [0 .. maxRpm] (post-deadband band).
    const float span = (_spec.maxAllowedDuty > _spec.minSpinDuty)
                           ? (_spec.maxAllowedDuty - _spec.minSpinDuty)
                           : 1.0f;
    float norm = (_duty - _spec.minSpinDuty) / span;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    targetRpm = norm * _spec.maxRpm;
  }

  const float alpha = dt / (_spec.spinUpTauS + dt);
  _rpm += (targetRpm - _rpm) * alpha;
  if (_rpm < 0.5f && targetRpm == 0.0f) _rpm = 0.0f;
}

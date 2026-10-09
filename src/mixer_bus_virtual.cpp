/**
 * @file mixer_bus_virtual.cpp
 * @brief Virtual `IPwmActuator` + `ITachometer` + `IKnobInput`: binds MixerSim to the node.
 *
 * Build: selected by `-DMIXER_VIRTUAL=1` (env `esp32-s3-virtual`). The PID above it is the
 * production controller, unchanged.
 */

#include "mixer_bus.h"
#include "mixer_sim.h"

#include <Arduino.h>

#ifndef TWO_PI_CONST
#define TWO_PI_CONST 6.28318530718f
#endif

namespace {

class VirtualActuator : public IPwmActuator {
public:
  void init() override {
    _sim.reset();
    Serial.println("[MixerBus/Virtual] Simulated SNR8503M + T-200 active.");
  }
  void setDuty(float duty) override {
    _duty = (duty < 0.0f) ? 0.0f : ((duty > 1.0f) ? 1.0f : duty);
    _sim.setDuty(_duty);
    _compensated = _duty;  // no RC linearization in the model
  }
  float duty() const override { return _duty; }
  float compensatedDuty() const override { return _compensated; }

  void tick(uint32_t nowMs) { _sim.tick(nowMs); }
  float rpm() const { return _sim.rpm(); }
  uint8_t faultCode() const { return _sim.faultCode(); }

private:
  MixerSim _sim;
  float _duty = 0.0f;
  float _compensated = 0.0f;
};

class VirtualTachometer : public ITachometer {
public:
  explicit VirtualTachometer(VirtualActuator& act) : _act(act) {}

  void init() override {}
  void update(uint32_t nowUs) override {
    _act.tick((uint32_t)(nowUs / 1000ULL));
    const float rpm = _act.rpm();
    const float rps = rpm / 60.0f;
    _v.rpm = rpm;
    _v.rps = rps;
    _v.rad_s = rps * TWO_PI_CONST;
    _v.freq_hz = rps * (float)_polePairs;
    _v.period_us = (_v.freq_hz > 0.001f) ? (uint32_t)(1000000.0f / _v.freq_hz) : 0;
    _v.total_pulses = _pulses;
    _v.fault_code = _act.faultCode();
    // Integrate pulses at the model rate.
    static uint32_t lastPulseUs = 0;
    if (nowUs != lastPulseUs) {
      lastPulseUs = nowUs;
    }
  }
  MixerVelocity read() const override { return _v; }

private:
  VirtualActuator& _act;
  MixerVelocity _v;
  uint32_t _pulses = 0;
  static constexpr uint8_t _polePairs = 7;
};

class VirtualKnob : public IKnobInput {
public:
  void init() override {}
  uint16_t read() override { return 0; }  // knob locked in virtual mode
};

VirtualActuator g_actuator;
VirtualTachometer g_tach(g_actuator);
VirtualKnob g_knob;

}  // namespace

IPwmActuator& mixerActuator() { return g_actuator; }
ITachometer& mixerTachometer() { return g_tach; }
IKnobInput& mixerKnob() { return g_knob; }

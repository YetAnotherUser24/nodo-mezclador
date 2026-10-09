/**
 * @file mixer_bus_real.cpp
 * @brief Real `IPwmActuator` + `ITachometer` + `IKnobInput` for the mixer node.
 *
 * Extracted verbatim from the original `esp32s3_main.cpp`: LEDC PWM with the analytic RC
 * linearizer, the reciprocal FG tachometer ISR with the fault-pulse decoder, and the analog knob.
 * No control logic lives here.
 */

#include "mixer_bus.h"

#include <Arduino.h>
#include <math.h>

#ifndef PIN_VSP_PWM
#define PIN_VSP_PWM 13
#endif
#ifndef PIN_PULSE_IN
#define PIN_PULSE_IN 7
#endif
#ifndef PIN_KNOB_ANALOG
#define PIN_KNOB_ANALOG 4
#endif

namespace {

constexpr uint8_t BLDC_POLE_PAIRS = 7;
constexpr float TWO_PI_CONST = 6.283185307179586f;

constexpr uint8_t LEDC_PWM_CHANNEL = 0;
constexpr uint32_t LEDC_PWM_FREQ_HZ = 10000;
constexpr uint8_t LEDC_PWM_RES_BITS = 10;
constexpr uint32_t LEDC_PWM_MAX_TICKS = (1UL << LEDC_PWM_RES_BITS);

#define VSP_PWM_INVERTED 1
#define ENABLE_PWM_LINEARIZATION 1

constexpr float MOTOR_MIN_SPIN_DUTY = 0.25f;
constexpr float MOTOR_MAX_ALLOWED_DUTY = 0.989f;

constexpr float PULLUP_RESISTOR_OHMS = 200.0f;
constexpr float R30_INTERNAL_OHMS = 10000.0f;
constexpr float GAIN_DIVIDER_RATIO = (2.0f / 3.0f);

constexpr uint32_t STOP_TIMEOUT_US = 3500000UL;
constexpr uint32_t MIN_VALID_PERIOD_US = 200;
constexpr uint8_t FILTER_SIZE = 5;

float compensatePwmDuty(float dTarget) {
  if (dTarget <= 0.0f) return 0.0f;
  if (dTarget > 1.0f) dTarget = 1.0f;
#if ENABLE_PWM_LINEARIZATION
  constexpr float Rp = PULLUP_RESISTOR_OHMS;
  constexpr float R30 = R30_INTERNAL_OHMS;
  constexpr float G = GAIN_DIVIDER_RATIO;
  float dComp = ((Rp + R30) * dTarget) / (R30 + G * Rp * dTarget);
  if (dComp > 1.0f) dComp = 1.0f;
  return dComp;
#else
  return dTarget;
#endif
}

/* ------------------------------- PWM actuator ----------------------------- */

class LedcPwmActuator : public IPwmActuator {
public:
  void init() override { setDuty(0.0f); }

  void setDuty(float targetDuty) override {
    targetDuty = constrain(targetDuty, 0.0f, 1.0f);
    _duty = targetDuty;

    if (targetDuty < 0.005f) {
      _compensated = 0.0f;
      if (_attached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcDetach(PIN_VSP_PWM);
#else
        ledcDetachPin(PIN_VSP_PWM);
#endif
        _attached = false;
      }
      pinMode(PIN_VSP_PWM, OUTPUT);
#if VSP_PWM_INVERTED
      digitalWrite(PIN_VSP_PWM, HIGH);
#else
      digitalWrite(PIN_VSP_PWM, LOW);
#endif
      return;
    }

    const float effective = MOTOR_MIN_SPIN_DUTY + targetDuty * (MOTOR_MAX_ALLOWED_DUTY - MOTOR_MIN_SPIN_DUTY);
    const float compensated = compensatePwmDuty(effective);
    _compensated = compensated;

    uint32_t ticks;
#if VSP_PWM_INVERTED
    ticks = (uint32_t)roundf((1.0f - compensated) * (float)LEDC_PWM_MAX_TICKS);
#else
    ticks = (uint32_t)roundf(compensated * (float)LEDC_PWM_MAX_TICKS);
#endif
    if (ticks > LEDC_PWM_MAX_TICKS) ticks = LEDC_PWM_MAX_TICKS;

    if (!_attached) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      ledcAttach(PIN_VSP_PWM, LEDC_PWM_FREQ_HZ, LEDC_PWM_RES_BITS);
#else
      ledcSetup(LEDC_PWM_CHANNEL, LEDC_PWM_FREQ_HZ, LEDC_PWM_RES_BITS);
      ledcAttachPin(PIN_VSP_PWM, LEDC_PWM_CHANNEL);
#endif
      _attached = true;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_VSP_PWM, ticks);
#else
    ledcWrite(LEDC_PWM_CHANNEL, ticks);
#endif
  }

  float duty() const override { return _duty; }
  float compensatedDuty() const override { return _compensated; }

private:
  float _duty = 0.0f;
  float _compensated = 0.0f;
  bool _attached = false;
};

/* ------------------------------ FG tachometer ----------------------------- */

volatile uint32_t s_isrLastPulseUs = 0;
volatile uint32_t s_isrPeriodUs = 0;
volatile uint32_t s_isrTotalPulses = 0;
volatile bool s_isrNewPulse = false;

void IRAM_ATTR onPulseRisingEdge() {
  uint32_t now = micros();
  if (s_isrLastPulseUs > 0) {
    uint32_t p = now - s_isrLastPulseUs;
    if (p >= MIN_VALID_PERIOD_US) {
      s_isrPeriodUs = p;
      s_isrLastPulseUs = now;
      s_isrTotalPulses++;
      s_isrNewPulse = true;
    }
  } else {
    s_isrLastPulseUs = now;
    s_isrTotalPulses++;
    s_isrNewPulse = true;
  }
}

class FgTachometer : public ITachometer {
public:
  void init() override {
    pinMode(PIN_PULSE_IN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PULSE_IN), onPulseRisingEdge, RISING);
  }

  void update(uint32_t nowUs) override {
    noInterrupts();
    uint32_t lastPulseUs = s_isrLastPulseUs;
    uint32_t rawPeriodUs = s_isrPeriodUs;
    uint32_t totalPulses = s_isrTotalPulses;
    bool hasNewPulse = s_isrNewPulse;
    s_isrNewPulse = false;
    interrupts();

    _v.total_pulses = totalPulses;

    uint32_t elapsedSincePulseUs;
    if (lastPulseUs == 0) {
      elapsedSincePulseUs = 99999999;
    } else if (nowUs >= lastPulseUs) {
      elapsedSincePulseUs = nowUs - lastPulseUs;
    } else {
      elapsedSincePulseUs = 0;
    }

    if (elapsedSincePulseUs > STOP_TIMEOUT_US || lastPulseUs == 0) {
      _state = STATE_STOPPED;
      clearVelocity();
      _filterCount = 0;
      _lastStablePeriodUs = 0;
      return;
    }

    if (_state == STATE_STOPPED) {
      _state = STATE_FIRST_PULSE;
      clearVelocity();
      return;
    }

    if (hasNewPulse && rawPeriodUs > 0) {
      // FG fault-pulse decoder (>150 ms pulse = fault code; >1.5 s gap ends a sequence).
      if (rawPeriodUs > 150000) {
        if (rawPeriodUs > 1500000) {
          if (_faultCount > 0) _lastFaultCode = _faultCount;
          _faultCount = 1;
        } else {
          _faultCount++;
        }
        _v.fault_code = _lastFaultCode;
        _v.period_us = rawPeriodUs;
        _v.freq_hz = 1000000.0f / (float)rawPeriodUs;
        _v.rpm = 0.0f;
        _v.rad_s = 0.0f;
        _v.rps = 0.0f;
        return;
      }
      _v.fault_code = 0;

      _periodBuffer[_filterIdx] = rawPeriodUs;
      _filterIdx = (_filterIdx + 1) % FILTER_SIZE;
      if (_filterCount < FILTER_SIZE) _filterCount++;

      uint32_t sum = 0;
      for (uint8_t i = 0; i < _filterCount; i++) sum += _periodBuffer[i];
      _lastStablePeriodUs = (uint32_t)(sum / _filterCount);
    }

    if (_state == STATE_RUNNING && _lastStablePeriodUs > 0) {
      uint32_t effectivePeriodUs = _lastStablePeriodUs;
      if (elapsedSincePulseUs > _lastStablePeriodUs) effectivePeriodUs = elapsedSincePulseUs;

      _v.period_us = effectivePeriodUs;
      _v.freq_hz = 1000000.0f / (float)effectivePeriodUs;
      _v.rps = _v.freq_hz / (float)BLDC_POLE_PAIRS;
      _v.rad_s = _v.rps * TWO_PI_CONST;
      _v.rpm = _v.rps * 60.0f;
    }
    if (_state == STATE_FIRST_PULSE) _state = STATE_RUNNING;
  }

  MixerVelocity read() const override { return _v; }

private:
  enum SensorState { STATE_STOPPED = 0, STATE_FIRST_PULSE, STATE_RUNNING };

  void clearVelocity() {
    _v.rad_s = 0.0f;
    _v.rpm = 0.0f;
    _v.rps = 0.0f;
    _v.freq_hz = 0.0f;
    _v.period_us = 0;
  }

  MixerVelocity _v;
  SensorState _state = STATE_STOPPED;
  uint32_t _periodBuffer[FILTER_SIZE] = {0};
  uint8_t _filterIdx = 0;
  uint8_t _filterCount = 0;
  uint32_t _lastStablePeriodUs = 0;
  uint8_t _lastFaultCode = 0;
  uint8_t _faultCount = 0;
};

/* --------------------------------- Knob ----------------------------------- */

class AdcKnob : public IKnobInput {
public:
  void init() override {
    pinMode(PIN_KNOB_ANALOG, INPUT);
    analogReadResolution(12);
  }
  uint16_t read() override { return (uint16_t)analogRead(PIN_KNOB_ANALOG); }
};

LedcPwmActuator g_actuator;
FgTachometer g_tach;
AdcKnob g_knob;

}  // namespace

IPwmActuator& mixerActuator() { return g_actuator; }
ITachometer& mixerTachometer() { return g_tach; }
IKnobInput& mixerKnob() { return g_knob; }

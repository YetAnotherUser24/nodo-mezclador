#pragma once
/**
 * @file mixer_bus.h
 * @brief Device-boundary interface for the T-200 mixer (SNR8503M driver + thruster).
 *
 * The node's PID/navigation logic talks to the thruster only through this seam. The real build
 * binds it to LEDC PWM + the FG tachometer ISR; the virtual build binds it to a duty->rpm model.
 * `esp32s3_main.cpp` is identical in both builds.
 */

#include <stdint.h>

/** Measured/estimated velocity state (mirrors the firmware's VelocityState). */
struct MixerVelocity {
  float rad_s = 0.0f;
  float rpm = 0.0f;
  float rps = 0.0f;
  float freq_hz = 0.0f;
  uint32_t period_us = 0;
  uint32_t total_pulses = 0;
  uint8_t fault_code = 0;  // 0 = none; >0 = decoded FG fault-pulse code
};

class IPwmActuator {
public:
  virtual ~IPwmActuator() = default;
  /** Initialize the PWM stage (idempotent). */
  virtual void init() = 0;
  /** Set throttle duty in [0, 1]. */
  virtual void setDuty(float duty) = 0;
  /** Last commanded duty (post-clamp). */
  virtual float duty() const = 0;
  /** Last hardware duty after linearization/deadband (for SYSID logging). */
  virtual float compensatedDuty() const = 0;
};

class ITachometer {
public:
  virtual ~ITachometer() = default;
  /** Initialize the tachometer input. */
  virtual void init() = 0;
  /** Poll/refresh the velocity estimate (reads ISR state or steps the model). */
  virtual void update(uint32_t nowUs) = 0;
  /** Current velocity snapshot. */
  virtual MixerVelocity read() const = 0;
};

class IKnobInput {
public:
  virtual ~IKnobInput() = default;
  virtual void init() = 0;
  /** Raw ADC reading (0..4095). */
  virtual uint16_t read() = 0;
};

/** Compile-time factories. Each build provides exactly one definition of each. */
IPwmActuator& mixerActuator();
ITachometer& mixerTachometer();
IKnobInput& mixerKnob();

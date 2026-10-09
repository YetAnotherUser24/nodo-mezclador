/**
 * @file test_mixer_bus.cpp
 * @brief Host conformance test for the mixer thruster device model.
 *
 * Runs `src/mixer_sim.cpp` on a deterministic clock and asserts the contract the PID relies on:
 * deadband behaviour, spin-up toward the duty-mapped RPM, duty clamping, and fault-pulse codes.
 *
 * Build: g++ -std=c++17 -I src test/host/test_mixer_bus.cpp src/mixer_sim.cpp -o /tmp/test_mixer_bus
 */

#include "mixer_sim.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static void advance(MixerSim& sim, uint32_t& t, uint32_t ms, uint32_t stepMs = 10) {
  for (uint32_t e = 0; e < ms; e += stepMs) {
    t += stepMs;
    sim.tick(t);
  }
}

int main() {
  MixerSimSpec spec;
  spec.maxRpm = 3800.0f;
  spec.minSpinDuty = 0.25f;
  spec.maxAllowedDuty = 0.989f;

  MixerSim sim(spec);
  uint32_t t = 1000;
  sim.tick(t);

  // 1. At rest.
  assert(near(sim.rpm(), 0.0f, 0.001f));
  assert(sim.faultCode() == 0);

  // 2. Deadband: below minSpinDuty the motor does not spin.
  sim.setDuty(0.20f);
  advance(sim, t, 2000);
  assert(near(sim.rpm(), 0.0f, 1.0f));

  // 3. Above the deadband the motor spins up.
  sim.setDuty(0.60f);
  advance(sim, t, 4000);
  assert(sim.rpm() > 100.0f);

  // 4. Full duty approaches the ceiling but never exceeds it.
  sim.setDuty(1.0f);
  advance(sim, t, 6000);
  assert(sim.rpm() > 3800.0f * 0.95f);
  assert(sim.rpm() <= spec.maxRpm + 0.001f);

  // 5. Duty is clamped to [0, 1].
  sim.setDuty(2.0f);
  assert(near(sim.duty(), 1.0f, 0.0001f));
  sim.setDuty(-1.0f);
  assert(near(sim.duty(), 0.0f, 0.0001f));

  // 6. A fault stops the motor and reports its code.
  sim.setDuty(0.6f);
  advance(sim, t, 1000);
  sim.injectFault(4u);  // STALL_FAULT, per getDriverFaultStr()
  assert(sim.faultCode() == 4u);
  assert(near(sim.rpm(), 0.0f, 0.001f));

  // 7. Data-driven fault schedule fires once.
  MixerSim scheduled(spec);
  scheduled.addFaultEvent({5000u, 2u});
  uint32_t ts = 0;
  scheduled.tick(ts);
  advance(scheduled, ts, 4000);
  assert(scheduled.faultCode() == 0u);
  advance(scheduled, ts, 2000);
  assert(scheduled.faultCode() == 2u);

  std::printf("test_mixer_bus: all assertions passed\n");
  return 0;
}

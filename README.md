# Nodo mezclador — Control de velocidad del T-200 (`nodo-mezclador`)

> **Repositorio:** [github.com/YetAnotherUser24/nodo-mezclador](https://github.com/YetAnotherUser24/nodo-mezclador)

High-performance, closed-loop angular velocity controller for the **BlueRobotics T-200 Thruster** driven by the **SNR8503M 6–80V 20A BLDC Driver Module** and controlled by an **ESP32-S3**.

---

## 1. System Overview

```
+-----------------------------------------------------------------------------------------+
|                                    SYSTEM ARCHITECTURE                                  |
|                                                                                         |
|  [ ESP32-S3 Controller ]                                       [ SNR8503M BLDC Driver ] |
|  - 100 Hz Discrete PID (rad/s)                                  - 3-Phase Inverter      |
|  - Exact Analytical Linearization                               - BEMF Zero-Crossing    |
|  - 10 kHz 10-bit LEDC PWM (GPIO 13) -> [Open-Drain Stage] ---->  - VSP Analog/PWM Input |
|  - Reciprocal ISR Tachometer (GPIO 7) <-- [1k/2k Divider] <---  - FG Tachometer Output  |
|                                                                          |              |
|                                                                   [ T-200 Thruster ]    |
|                                                                   - 7 Pole Pairs BLDC   |
+-----------------------------------------------------------------------------------------+
```

### Key Engineering Features
1. **Actuation (Option B Open-Drain + Stiff Pull-Up)**:
   - Hardware **10 kHz LEDC PWM** with **10-bit resolution** (1,024 discrete steps; $0.098\%$ step size).
   - A discrete N-channel MOSFET (`2N7002` / `BSS138`) or NPN transistor (`2N3904`) with a stiff $330\ \Omega$ pull-up resistor to the driver's $+5\text{V}$ rail.
2. **Analytical Software Linearization Pre-Compensator**:
   - Closed-form mathematical inverse function completely cancels the non-linearity introduced by the driver's internal $10\text{k}\Omega / 20\text{k}\Omega / 10\ \mu\text{F}$ low-pass filter ($0.0000\%$ mathematical error).
   - Toggleable via compile-time preprocessor macro (`ENABLE_PWM_LINEARIZATION`).
3. **Autonomous Microsecond Tachometer Engine**:
   - Measures time intervals between rising edges on the driver's `FG` pulse output using microsecond timestamps (`micros()`) in an `IRAM_ATTR` ISR.
   - Sub-RPM precision across the full range ($2.4\text{ RPM}$ to $3800\text{ RPM}$) with zero pulse-counting quantization jitter.
   - **Dynamic Deceleration Decay**: Automatically scales down velocity in real time during braking ($v \propto 1 / \Delta t_{\text{elapsed}}$), preventing telemetry freeze when pulses slow down.
4. **100 Hz Discrete PID Velocity Controller**:
   - Operates strictly in canonical SI units ($\omega$ in $\text{rad/s}$).
   - Includes anti-windup clamping, derivative-on-measurement (eliminating setpoint derivative kicks), and slew-rate limiting to protect the thruster from rapid cavitation shock and current spikes.
5. **Interactive Serial CLI**:
   - USB CDC serial terminal ($115200\text{ baud}$) for setpoint commands (`SPEED`, `RPM`, `DUTY`), real-time gain tuning (`TUNE`), and continuous telemetry streaming.

---

## 2. Complete Wiring & Pinout Guide

> [!CAUTION]
> **Voltage Domain Warning**:
> - The SNR8503M driver logic headers operate at **+5.0V**.
> - The ESP32-S3 GPIO pins tolerate a maximum of **+3.3V**.
> - Always use the **1kΩ / 2kΩ voltage divider** on the `FG` tachometer line to step $5.0\text{V}$ down to $3.33\text{V}$.

### Pin Interconnection Table

| SNR8503M Header J1 Pin | Signal Name | Target Connection | Circuit Description |
| :---: | :---: | :---: | :--- |
| **Pin 1** | **GND** | **ESP32-S3 GND** | Common ground reference |
| **Pin 2** | **+5V** | Top of **$330\ \Omega$ Resistor** | From driver's onboard 78L05 regulator ($20 - 30\text{ mA}$ continuous capacity) |
| **Pin 3** | **VSP** | Transistor **Drain / Collector** | Motor speed input ($0.5\text{V} - 5.0\text{V}$) |
| **Pin 4** | **FG** | Top of **$1\text{k}\Omega$ Resistor** | Tachometer pulse train ($7\text{ pulses/revolution}$) |
| **Pin 5** | **GND** | Bottom of **$2\text{k}\Omega$ Resistor** | Divider ground reference |
| **Pin 6** | **CW/CCW** | Open / GND | Rotation direction select |

### Schematic Diagram (Option B Circuit)

```
  ESP32-S3 (3.3V)                                SNR8503M Driver (5V)
+------------------+                           +-----------------------+
|                  |                           |                       |
|              GND +---------------------------+ Pin 1: GND            |
|                  |                           |                       |
|                  |         +---[ 330Ω ]------+ Pin 2: +5V            |
|                  |         |                 |                       |
|                  |         +-----------------+ Pin 3: VSP            |
|                  |         | (Drain/Coll)    |                       |
|                  |       [FET] 2N7002        |                       |
|                  |         | (Source/Emit)   |                       |
|                  |        GND                |                       |
|                  |                           |                       |
|     GPIO 13 (PWM) +--[ 100Ω ]-> (Gate/Base)  |                       |
|                  |                           |                       |
|   GPIO 7 (Pulse) +<----+                     | Pin 4: FG (Pulse Out) |
|                  |     |                     |                       |
|                  |   [ 1kΩ ]                 +-----------------------+
|                  |     |
|                  |     +---------------------+
|                  |     |
|                  |   [ 2kΩ ]
|                  |     |
|              GND +-----+
+------------------+
```

---

## 3. Mathematical Formulation & Physics

### A. Thruster Velocity & Pulse Frequency Relations

The BlueRobotics T-200 thruster has **$P = 7$ rotor pole pairs**. The driver's `FG` output toggles at every electrical commutation cycle:

```
Electrical Frequency:
  f_elec = (RPM * P) / 60 = RPS * 7   [Hz]

Rotor Speed in Mechanical Revolutions per Second:
  RPS = f_elec / P = 10^6 / (7 * T_us)

Angular Velocity in Canonical SI Units:
  omega = 2 * pi * RPS = (2 * pi * 10^6) / (7 * T_us)   [rad/s]

Rotational Speed in RPM:
  RPM = RPS * 60 = (60 * 10^6) / (7 * T_us)
```

$$\omega = \frac{2\pi \times 10^6}{7 \times T_{\mu\text{s}}} \quad [\text{rad/s}]$$

$$\text{RPM} = \frac{60 \times 10^6}{7 \times T_{\mu\text{s}}}$$

*(where $T_{\mu\text{s}}$ is the period in microseconds between consecutive rising edges on the `FG` pin).*

#### Reference Operational Range (T-200 Thruster)
| Mechanical Speed (RPM) | Angular Velocity $\omega$ (rad/s) | FG Frequency $f$ (Hz) | FG Period $T$ ($\mu\text{s}$) |
| :---: | :---: | :---: | :---: |
| **3500** *(Near Max Forward)* | $366.52\text{ rad/s}$ | $408.3\text{ Hz}$ | $2,449\ \mu\text{s}$ |
| **2000** *(Cruising Speed)* | $209.44\text{ rad/s}$ | $233.3\text{ Hz}$ | $4,286\ \mu\text{s}$ |
| **1000** *(Low Speed)* | $104.72\text{ rad/s}$ | $116.7\text{ Hz}$ | $8,571\ \mu\text{s}$ |
| **100** *(Trolling)* | $10.47\text{ rad/s}$ | $11.7\text{ Hz}$ | $85,714\ \mu\text{s}$ |
| **10** *(Creep)* | $1.05\text{ rad/s}$ | $1.17\text{ Hz}$ | $857,143\ \mu\text{s}$ |
| **2.4** *(Floor Cutoff)* | $0.25\text{ rad/s}$ | $0.28\text{ Hz}$ | $3,500,000\ \mu\text{s}$ |

---

### B. Option B Modeling & Exact Analytical Linearization

The driver's `VSP` input has an internal voltage divider and low-pass filter:
* $R_{30} = 10,000\ \Omega$ (series resistor)
* $R_{32} = 20,000\ \Omega$ (shunt resistor to ground)
* $C_{28} = 10\ \mu\text{F}$ (filter capacitor to ground)

Because the $10\text{ kHz}$ PWM period ($T = 100\ \mu\text{s}$) is far faster than the filter time constant ($\tau = R_{\text{th}} \times C_{28} \approx 66.7\text{ ms}$), the capacitor integrates the charging and discharging current.

#### Forward Transfer Function (Non-Linearity with Pull-Up $R_p$):
When using an open-drain transistor with pull-up resistor $R_p = 330\ \Omega$:

```
Perceived Duty Cycle:
  D_perceived = ( (Rp + R30) * D_pwm ) / ( (Rp + R30) * D_pwm + R30 * (1 - D_pwm) + (R30 * (Rp + R30) / R32) )
```

$$\bar{V}(D_{\text{pwm}}) = \frac{\frac{V_{cc}}{R_p + R_{30}} D_{\text{pwm}}}{\frac{D_{\text{pwm}}}{R_p + R_{30}} + \frac{1 - D_{\text{pwm}}}{R_{30}} + \frac{1}{R_{32}}}$$

#### Exact Analytical Inverse Compensator:
To eliminate this curvature, the firmware applies the exact inverse formula before writing to the PWM hardware:

```
Compensated PWM Command:
  D_pwm = ( (Rp + 10000) * D_target ) / ( 10000 + (2/3) * Rp * D_target )
```

$$\mathbf{D_{\text{pwm}} = \frac{(R_p + 10000) \cdot D_{\text{target}}}{10000 + \frac{2}{3} R_p \cdot D_{\text{target}}}}$$

* **With $R_p = 330\ \Omega$**:
  * Residual non-linearity error: **$0.0000\%$** (mathematically exact).
  * Maximum full-throttle voltage: **$3.297\text{ V}$** out of $3.333\text{ V}$ ($98.9\%$ of full motor power).
  * Conduction current when ON: $15.15\text{ mA}$ ($75.7\text{ mW}$ on resistor; safe for $1/4\text{W}$ ratings and the 78L05 regulator).

---

### C. 100 Hz Discrete PID Velocity Controller

The control loop executes every $T_s = 0.01\text{ s}$ ($100\text{ Hz}$):

```
1. Error Calculation (Canonical rad/s):
   e[k] = omega_target - omega_measured

2. Proportional Term:
   P[k] = Kp * e[k]

3. Integral Term with Anti-Windup Clamping:
   I[k] = constrain(I[k-1] + Ki * e[k] * Ts, -0.5, 1.0)

4. Derivative on Measurement (Prevents Setpoint Kick):
   D[k] = -Kd * ( (omega_measured[k] - omega_measured[k-1]) / Ts )

5. Feedforward Duty Estimate:
   FF[k] = (omega_target / MAX_RAD_S) * 0.70

6. Unconstrained Control Output:
   u_raw = P[k] + I[k] + D[k] + FF[k]
   u_clamped = constrain(u_raw, 0.0, 1.0)

7. Slew Rate Limiter (Limits Acceleration Surge):
   delta = constrain(u_clamped - u_prev, -MaxSlew * Ts, +MaxSlew * Ts)
   u_out = u_prev + delta
```

---

## 4. Building, Uploading & Operating

### Prerequisites
* PlatformIO CLI or PlatformIO IDE extension.

`default_envs = esp32-s3`, so plain `pio run` already targets the node's board.

```bash
# 1. Compile the default environment (esp32-s3)
pio run

# 2. Upload to the ESP32-S3 via USB CDC
pio run -t upload

# 3. Open the serial monitor at 115200 baud
pio device monitor -b 115200

# 4. Wireless update (OTA) at mixer-t200.local
pio run -e esp32-s3-ota -t upload
```

---

## 5. Serial Command Reference (CLI)

The firmware provides an interactive text-based console over USB Serial ($115200\text{ baud}$):

| Command | Example | Description |
| :--- | :--- | :--- |
| `SPEED <rad/s>` | `SPEED 150.0` | Set closed-loop target velocity in canonical SI units ($\text{rad/s}$) |
| `RPM <rpm>` | `RPM 1500` | Set closed-loop target speed in mechanical RPM |
| `<number>` | `1200` | Shorthand: typing a raw number sets closed-loop RPM directly |
| `DUTY <0.0-1.0>` | `DUTY 0.30` | Open-loop direct PWM throttle override ($30\%$ duty) |
| `STOP` | `STOP` | Safely ramps down target to 0 and stops PWM output |
| `TUNE <Kp> <Ki> <Kd>` | `TUNE 0.0015 0.0040 0.00005` | Update PID gains live without re-flashing |
| `STATUS` | `STATUS` | Print full snapshot of controller state, velocity, error, and PWM |
| `HELP` | `HELP` | Print command cheat sheet |

### Telemetry Stream Example
```text
[TELEMETRY] Measured: 157.08 rad/s (1500.0 RPM) | Target: 157.1 rad/s (1500 RPM) | Err: +0.02 rad/s | Duty: 38.2% (PWM: 39.4%) | Freq: 175.0 Hz
[TELEMETRY] Measured: 157.06 rad/s (1499.8 RPM) | Target: 157.1 rad/s (1500 RPM) | Err: +0.04 rad/s | Duty: 38.2% (PWM: 39.4%) | Freq: 175.0 Hz
```

---

## 6. Verification & Tuning Workflow

### Phase 1: Electrical Checkout (Motor Disconnected)
1. Power up the ESP32-S3 and connect to the Serial Monitor.
2. Probe `VSP` (J1 Pin 3) with a digital multimeter:
   * Send `DUTY 0.0` $\rightarrow$ Confirm $V_{\text{VSP}} = 0.00\text{V}$.
   * Send `DUTY 0.25` $\rightarrow$ Confirm $V_{\text{VSP}} \approx 1.25\text{V}$.
   * Send `DUTY 0.50` $\rightarrow$ Confirm $V_{\text{VSP}} \approx 2.50\text{V}$.
   * Send `DUTY 0.75` $\rightarrow$ Confirm $V_{\text{VSP}} \approx 3.75\text{V}$.
   * Send `DUTY 1.00` $\rightarrow$ Confirm $V_{\text{VSP}} \approx 4.95\text{V}$.

### Phase 2: Open-Loop Thruster Spin-Up
1. Connect the T-200 thruster phases ($U, V, W$) to the driver.
2. Send `DUTY 0.15` to confirm smooth sensorless startup without stutter.
3. Verify that the `[TELEMETRY]` log displays matching pulse frequency and RPM.
4. Send `STOP` and confirm that measured RPM smoothly decays to $0.0$ without freezing.

### Phase 3: Closed-Loop PID Tuning
1. Send `RPM 1000` ($\approx 104.7\text{ rad/s}$) to engage the closed-loop controller.
2. If the thruster oscillates, reduce $K_p$ using `TUNE`.
4. Test step responses from `RPM 1000` $\rightarrow$ `RPM 2000` to verify settling time and disturbance rejection.

---

## 7. System Identification & Optimization Toolchain

We developed a completely automated Python/MATLAB pipeline (`pipeline.py`) to extract empirical physics models from the physical thruster and use Metaheuristics to find the global optimal PI gains.

### How to use the Pipeline
You can run the entire toolchain sequentially (Data -> Identify -> Tune -> Validate) or execute isolated stages:
```bash
# Run the complete automated pipeline using the Return-to-Zero Staircase method
python pipeline.py full --method stair

# Run isolated sections
python pipeline.py data --method prbs
python pipeline.py ident-pso
python pipeline.py run --rpm 1500
```

### A. Data Acquisition Techniques
The pipeline supports three independent signal-injection techniques to map the physics of the motor:
1. **Single-Step (`--method step`)**: Traditional approach. Hits the motor with a single 60% duty cycle step. Good for local operating points, but blind to other speeds.
2. **PRBS (`--method prbs`)**: Pseudo-Random Binary Sequence. Toggles random duty cycles at random intervals. Forces the motor to reveal its high-pass and low-pass frequency responses across the entire speed range.
3. **Return-to-Zero Pulse-Step (`--method stair`)**: Steps up through incremental duty cycles (0.2, 0.4, 0.6, 0.8), but fundamentally forces the motor to brake to a dead stop (`0.0 Duty`) between every step. This perfectly isolates **Static Friction (Stiction)** and absolute inertia mapping.

### B. Plant Identification
The MATLAB backend (`scripts/sysid_and_tune.m`) automatically ingests the CSV telemetry and calculates five distinct models:
- Continuous: **FOPDT**
- Discrete Polynomials: **ARX(1,1,1), ARX(2,2,1), ARMAX(2,2,2,1), OE(2,2,1)**

The script utilizes the **Akaike Information Criterion (AIC)** and **Final Prediction Error (FPE)** to mathematically penalize overfitting and select the true most optimal model order (which consistently falls on **ARMAX(2,2,2,1)** for this thruster).

### C. Metaheuristic Optimization
Once the ARMAX model is extracted, `scripts/advanced_thesis_tuning.m` runs three separate optimization algorithms to find the ultimate PI gains ($K_p$, $K_i$):
1. **Nelder-Mead (`fminsearch`) with ITAE**: A Local Search algorithm that often gets stuck in mathematical local minima (traps).
2. **Particle Swarm Optimization (PSO) with ITAE**: A Global Search swarm algorithm that successfully escapes local minima to find the theoretical minimum error limit of the system.
3. **PSO with Custom Penalty**: A global swarm optimizing a custom cost function that brutally penalizes any overshoot above 2%. 

### Academic Discovery: The Gain Scheduling Imperative
By running the **Return-to-Zero Pulse-Step** test and plotting the resulting PSO validation, we proved a critical thesis conclusion: **A single Linear Model (LTI) fundamentally cannot govern a highly non-linear underwater thruster.** 

Because aerodynamic drag is quadratic ($v^2$) and the static friction deadband requires massive energy to break, a single set of PI gains will always result in massive overshoot (up to 145%) at certain speeds. 

**Next Steps**: We will partition the `stair` dataset into three perfectly linear zones (Low, Mid, High) and implement a **Gain Scheduler** inside `esp32s3_main.cpp`.

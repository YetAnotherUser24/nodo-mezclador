# System Identification & Tuning Pipeline Tutorial

This tutorial outlines the complete end-to-end pipeline for performing system identification on the T-200 thruster, tuning a PID controller via MATLAB, and validating the performance on the physical hardware.

The pipeline ensures all data is saved properly so you can directly compare a basic `pidtune` method with an advanced `PSO ITAE` method for your thesis.

---

## Folder Structure

Before running the pipeline, ensure you understand the directory structure:
```text
scripts/
├── data/                       <-- Raw CSV telemetry from ESP32
├── results/
│   ├── models/                 <-- Saved MATLAB workspaces (.mat)
│   └── figures/                <-- Generated PNG plots
│       ├── pidtune/            <-- Basic FOPDT + pidtune results
│       └── pso/                <-- Advanced ARMAX + PSO ITAE results
```

---

## Step 1: Open-Loop System ID (Hardware)

First, we need to gather raw physical data of the thruster spinning up in open-loop to understand its physics. Make sure the thruster is safely submerged in water (or open air for dry testing).

Run the python logger to record a 50% duty cycle step response:
```bash
cd scripts
python sysid_logger.py --duty 0.5 --duration 5.0
```
- **What this does:** Sends a 50% PWM command to the ESC, records the RPM telemetry at 50Hz, and saves it.
- **Output:** `scripts/data/step_response.csv`

---

## Step 2: System Modeling and Tuning (MATLAB)

Now that we have the raw physics data, we feed it to MATLAB to generate mathematical models and find the optimal PID gains. You can run either (or both) of the following scripts:

### Method A: Basic FOPDT + `pidtune`
Run the basic tuning script from MATLAB or terminal:
```bash
matlab -batch "cd scripts; run('sysid_and_tune.m')"
```
- **What this does:** Uses `tfest` to generate a continuous First-Order system and uses MATLAB's robust `pidtune` algorithm to find conservative gains.
- **Output:** `results/models/sysid_results_pidtune.mat` and figures in `results/figures/pidtune/`.

### Method B: Advanced ARMAX + PSO ITAE
Run the advanced thesis tuning script:
```bash
matlab -batch "cd scripts; run('advanced_thesis_tuning.m')"
```
- **What this does:** Compares multiple discrete models (ARX, ARMAX, OE), uses AIC to pick the best one, and runs Particle Swarm Optimization to minimize the ITAE cost function for highly aggressive gains.
- **Output:** `results/models/sysid_results.mat` and figures in `results/figures/pso/`.

*(Note: Take note of the `Kp`, `Ki`, and `Kd` values printed in the terminal for the next step!)*

---

## Step 3: Closed-Loop Hardware Validation

Take the PID gains calculated by MATLAB in Step 2, and upload them to the physical ESP32 to test how they perform in reality.

**For the Basic `pidtune` gains:**
*(Replace the values below with the actual output from Step 2)*
```bash
python scripts/test_gains.py --kp 0.00188 --ki 0.00319 --kd 0.0 --rpm 1500 --output scripts/data/closed_loop_pidtune.csv
```

**For the Advanced `PSO` gains:**
*(Replace the values below with the actual output from Step 2)*
```bash
python scripts/test_gains.py --kp 0.00456 --ki 0.00890 --kd 0.001 --rpm 1500 --output scripts/data/closed_loop_pso.csv
```

- **What this does:** Flashes the gains to the running ESP32, clears the PID integral, commands a 1500 RPM target, and records the physical closed-loop response.
- **Output:** Two distinct CSV files in `scripts/data/`.

---

## Step 4: Generate Final Comparison Plots (MATLAB)

Finally, combine the raw physical validation data with the mathematical simulation to generate thesis-ready comparison plots. 

Because `plot_validation.m` is parameterized, you can generate both plots consecutively from the terminal using variable overrides:

```bash
# Generate Basic plot
matlab -batch "cd scripts; csv_file='data/closed_loop_pidtune.csv'; mat_file='results/models/sysid_results_pidtune.mat'; out_img='results/figures/pidtune/closed_loop_validation.png'; run('plot_validation.m')"

# Generate Advanced plot
matlab -batch "cd scripts; csv_file='data/closed_loop_pso.csv'; mat_file='results/models/sysid_results.mat'; out_img='results/figures/pso/closed_loop_validation.png'; run('plot_validation.m')"
```

- **Output:** Final, annotated `.png` images featuring both theoretical and physical step responses overlayed, complete with PID gain titles and `OS%`/`Settling Time` statistics printed directly on the image.

**You are now ready to drop these images straight into your thesis document!**

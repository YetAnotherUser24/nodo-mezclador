# SNR8503M ESC Hardware Quirks & Control Rules

When working on the motor control loop for this repository, you MUST adhere to the following hardware constraints regarding the SNR8503M ESC:

1. **Stall Recovery Auto-Reset is Mandatory:**
   - If the ESC encounters an error (e.g., commanded speed is inside the deadband), it enters a Fault State and shuts off the motor.
   - **CRITICAL:** The ESC will NOT recover from a fault until the PWM input is dropped to 0%. 
   - A naive PID controller will wind up the duty cycle when the motor stops, keeping the PWM > 0% and permanently locking the ESC in the fault state. 
   - You MUST implement an explicit override that detects a stall (e.g., commanded speed > 0 but measured RPM < 50 for > 1.5s) and forces the PWM to 0.0% for 500ms while resetting the PID integral.

2. **FG Pin Diagnostic Pulses:**
   - During a fault, the ESC outputs digital diagnostic codes on the FG pin using 200ms low pulses (e.g., 4 pulses = Stall Fault, 10 pulses = Over-current), followed by a 2s gap.
   - Your speed measurement logic MUST detect sequences of > 150ms periods and intercept them. DO NOT feed fault pulses into the RPM moving average, as it will corrupt the velocity measurement.

3. **Strict Deadband Mapping:**
   - The ESC has a hard physical minimum startup threshold (e.g., ~25% PWM with a 200Ω pull-up). 
   - If the controller commands a PWM below this physical threshold, the ESC will instantly stall and fault. 
   - The `MOTOR_MIN_SPIN_DUTY` software constant must perfectly map the logical 0% bound to the physical hardware deadband to ensure the PID loop never commands a fault-inducing voltage.

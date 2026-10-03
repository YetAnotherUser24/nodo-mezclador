import serial
import serial.tools.list_ports
import time
import csv
import argparse
import sys

def find_com_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB" in port.description or "Serial" in port.description:
            return port.device
    if len(ports) > 0:
        return ports[0].device
    return None

def main():
    parser = argparse.ArgumentParser(description="T-200 PID Gain Tester")
    parser.add_argument('--port', type=str, help='COM port (e.g., COM16). Auto-detects if omitted.')
    parser.add_argument('--kp', type=float, required=True, help='Proportional Gain (Kp)')
    parser.add_argument('--ki', type=float, required=True, help='Integral Gain (Ki)')
    parser.add_argument('--kd', type=float, required=True, help='Derivative Gain (Kd)')
    parser.add_argument('--rpm', type=float, default=1500.0, help='Target RPM for step response. Default: 1500')
    parser.add_argument('--duration', type=float, default=5.0, help='Duration to record (s). Default: 5.0')
    parser.add_argument('--output', type=str, default='scripts/closed_loop_test.csv', help='Output CSV file')
    args = parser.parse_args()

    port = args.port or find_com_port()
    if not port:
        print("Error: Could not find a COM port.")
        sys.exit(1)

    print(f"Connecting to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        sys.exit(1)

    time.sleep(2)
    ser.write(b"STOP\n")
    time.sleep(0.5)

    print(f"Uploading Gains: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    ser.write(f"TUNE {args.kp} {args.ki} {args.kd}\n".encode('ascii'))
    time.sleep(0.5)

    print("Activating SYSID Mode...")
    ser.write(b"SYSID ON\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"Testing Closed-Loop Step Response: Target = {args.rpm} RPM...")
    ser.write(f"RPM {args.rpm}\n".encode('ascii'))

    start_time = time.time()
    data_points = []

    try:
        while time.time() - start_time < args.duration:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("SYSID,"):
                parts = line.split(',')
                if len(parts) == 5:
                    _, t_ms, target_duty, comp_duty, rad_s = parts
                    data_points.append({
                        'Time_ms': t_ms,
                        'Target_Duty': target_duty,
                        'Compensated_Duty': comp_duty,
                        'Rad_s': rad_s
                    })
    except KeyboardInterrupt:
        pass

    print("Test complete. Stopping motor...")
    ser.write(b"STOP\n")
    time.sleep(0.1)
    ser.write(b"SYSID OFF\n")
    ser.close()

    if not data_points:
        print("Warning: No data received.")
        sys.exit(1)

    t0 = int(data_points[0]['Time_ms'])
    with open(args.output, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['Time_s', 'Target_Duty', 'Compensated_Duty', 'Rad_s'])
        writer.writeheader()
        for row in data_points:
            t_s = (int(row['Time_ms']) - t0) / 1000.0
            writer.writerow({
                'Time_s': f"{t_s:.3f}",
                'Target_Duty': row['Target_Duty'],
                'Compensated_Duty': row['Compensated_Duty'],
                'Rad_s': row['Rad_s']
            })

    print(f"Saved {len(data_points)} points to {args.output}")

if __name__ == '__main__':
    main()

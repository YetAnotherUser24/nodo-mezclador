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
    parser = argparse.ArgumentParser(description="T-200 System ID Data Logger")
    parser.add_argument('--port', type=str, help='COM port (e.g., COM16). Auto-detects if omitted.')
    parser.add_argument('--duty', type=float, default=0.5, help='Target open-loop duty for the step response (0.0 - 1.0). Default: 0.5')
    parser.add_argument('--duration', type=float, default=5.0, help='How many seconds to record. Default: 5.0')
    parser.add_argument('--output', type=str, default='scripts/data/step_response.csv', help='Output CSV file name. Default: step_response.csv')
    args = parser.parse_args()

    port = args.port
    if not port:
        port = find_com_port()
        if not port:
            print("Error: Could not automatically find a COM port.")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    print(f"Connecting to {port} at 115200 baud...")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 0.1
        ser.setDTR(False)
        ser.setRTS(False)
        ser.open()
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        print("Make sure the Serial Monitor in PlatformIO is CLOSED before running this script!")
        sys.exit(1)

    time.sleep(2) # Wait for ESP32 to reset if DTR is triggered

    print("Sending STOP to ensure safe state...")
    ser.write(b"STOP\n")
    time.sleep(0.5)

    print("Activating SYSID Mode (50Hz telemetry)...")
    ser.write(b"SYSID ON\n")
    time.sleep(0.1)
    ser.reset_input_buffer()

    print(f"Recording zero-state for 0.5s, then executing step response: DUTY {args.duty} for {args.duration} seconds...")
    
    start_time = time.time()
    step_applied = False
    data_points = []

    try:
        while time.time() - start_time < (args.duration + 0.5):
            # Apply step after 0.5 seconds
            if not step_applied and (time.time() - start_time >= 0.5):
                cmd = f"DUTY {args.duty}\n".encode('ascii')
                ser.write(cmd)
                step_applied = True
                
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
        print("Interrupted by user.")

    print("Step response complete. Stopping motor...")
    ser.write(b"STOP\n")
    time.sleep(0.1)
    ser.write(b"SYSID OFF\n")
    time.sleep(0.1)
    ser.close()

    if not data_points:
        print("Warning: No SYSID data points were received! Did the ESP32 reboot or fail to recognize the command?")
        sys.exit(1)

    # Normalize time so it starts at 0
    t0 = int(data_points[0]['Time_ms'])

    # Write to CSV
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

    print(f"Successfully saved {len(data_points)} samples to {args.output}")
    print(f"Average sample rate: {len(data_points)/args.duration:.1f} Hz")

if __name__ == '__main__':
    main()

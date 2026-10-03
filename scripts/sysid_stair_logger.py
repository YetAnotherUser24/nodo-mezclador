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
    parser = argparse.ArgumentParser(description="T-200 Stair-Step System ID Logger")
    parser.add_argument('--port', type=str, help='COM port (e.g., COM16). Auto-detects if omitted.')
    parser.add_argument('--step_time', type=float, default=4.0, help='Time in seconds to hold each step. Default: 4.0')
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

    # The predefined return-to-zero staircase sequence
    sequence = [0.2, 0.0, 0.4, 0.0, 0.6, 0.0, 0.8, 0.0]
    total_duration = len(sequence) * args.step_time
    
    print(f"Recording zero-state for 0.5s, then executing Stair-Step sequence for {total_duration} seconds...")
    print(f"Sequence: {sequence}")
    
    start_time = time.time()
    last_step_time = start_time + 0.5 # Wait 0.5s at 0.0 duty before starting
    
    data_points = []
    current_step_idx = 0

    try:
        while time.time() - start_time < (total_duration + 0.5):
            now = time.time()
            
            # Apply next step in sequence
            if now >= last_step_time and current_step_idx < len(sequence):
                target_duty = sequence[current_step_idx]
                cmd = f"DUTY {target_duty}\n".encode('ascii')
                ser.write(cmd)
                print(f"[{now - start_time:.1f}s] Applying Duty: {target_duty}")
                
                last_step_time = now + args.step_time
                current_step_idx += 1
                
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

    print("Stair-step complete. Stopping motor...")
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
    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_s', 'Target_Duty', 'Compensated_Duty', 'Rad_s'])
        for dp in data_points:
            t_s = (int(dp['Time_ms']) - t0) / 1000.0
            writer.writerow([f"{t_s:.3f}", dp['Target_Duty'], dp['Compensated_Duty'], dp['Rad_s']])

    print(f"Successfully saved {len(data_points)} samples to {args.output}")

if __name__ == '__main__':
    main()

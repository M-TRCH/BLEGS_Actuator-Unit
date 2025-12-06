#!/usr/bin/env python3
"""
Gait Control Test Script - CSV Playback
Reads motor positions from CSV and sends commands via binary protocol

Author: M-TRCH
Date: 2025-12-06
Version: 1.0
"""

import struct
import serial
import time
import csv
import argparse
from pathlib import Path
from typing import List, Tuple
from enum import IntEnum

# Protocol Constants
HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_PING = 0x03
    PKT_FB_STATUS = 0x81
    PKT_FB_ERROR = 0x83

class ControlMode(IntEnum):
    """Control mode enumeration"""
    MODE_DIRECT_POSITION = 0x00
    MODE_SCURVE_PROFILE = 0x01


def calculate_crc16(data: bytes) -> int:
    """Calculate CRC-16-IBM for data buffer"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def send_direct_position(port: serial.Serial, target_degrees: float, verbose: bool = False) -> bool:
    """Send direct position command (fast, no S-Curve)"""
    # Build packet
    header = bytes([HEADER_1, HEADER_2])
    pkt_type = bytes([PacketType.PKT_CMD_SET_GOAL])
    
    # Build payload: control_mode + target_pos (int32, degrees*100)
    mode = bytes([ControlMode.MODE_DIRECT_POSITION])
    target_pos = struct.pack('<i', int(target_degrees * 100))
    
    payload = mode + target_pos
    payload_len = bytes([len(payload)])
    
    # Calculate CRC
    crc_data = pkt_type + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    # Assemble and send packet
    packet = header + pkt_type + payload_len + payload + crc_bytes
    port.write(packet)
    
    if verbose:
        print(f"[TX] Position: {target_degrees:7.2f}° | Packet: {packet.hex(' ')}")
    
    return True


def send_scurve_position(port: serial.Serial, target_degrees: float, duration_ms: int, verbose: bool = False) -> bool:
    """Send S-Curve position command (smooth motion)"""
    # Build packet
    header = bytes([HEADER_1, HEADER_2])
    pkt_type = bytes([PacketType.PKT_CMD_SET_GOAL])
    
    # Build payload: control_mode + target_pos (int32) + duration (uint16)
    mode = bytes([ControlMode.MODE_SCURVE_PROFILE])
    target_pos = struct.pack('<i', int(target_degrees * 100))
    duration = struct.pack('<H', duration_ms)
    
    payload = mode + target_pos + duration
    payload_len = bytes([len(payload)])
    
    # Calculate CRC
    crc_data = pkt_type + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    # Assemble and send packet
    packet = header + pkt_type + payload_len + payload + crc_bytes
    port.write(packet)
    
    if verbose:
        print(f"[TX] S-Curve: {target_degrees:7.2f}° in {duration_ms}ms | Packet: {packet.hex(' ')}")
    
    return True


def receive_feedback(port: serial.Serial, timeout: float = 0.01) -> dict:
    """Receive and parse feedback packet (non-blocking)"""
    port.timeout = timeout
    
    # Read header
    header = port.read(2)
    if len(header) != 2:
        return None
    
    if header[0] != HEADER_1 or header[1] != HEADER_2:
        return None
    
    # Read packet type and length
    meta = port.read(2)
    if len(meta) != 2:
        return None
    
    pkt_type = meta[0]
    payload_len = meta[1]
    
    # Read payload
    payload = port.read(payload_len)
    if len(payload) != payload_len:
        return None
    
    # Read CRC
    crc_bytes = port.read(2)
    if len(crc_bytes) != 2:
        return None
    
    received_crc = struct.unpack('<H', crc_bytes)[0]
    calculated_crc = calculate_crc16(bytes([pkt_type, payload_len]) + payload)
    
    if received_crc != calculated_crc:
        return {'type': 'error', 'msg': 'CRC failed'}
    
    # Parse payload based on type
    if pkt_type == PacketType.PKT_FB_STATUS and len(payload) >= 7:
        actual_pos_raw = struct.unpack('<i', payload[0:4])[0]
        actual_current = struct.unpack('<h', payload[4:6])[0]
        status_flags = payload[6]
        
        return {
            'type': 'status',
            'position_deg': actual_pos_raw / 100.0,
            'current_ma': actual_current,
            'flags': status_flags
        }
    
    return None


def load_csv_trajectory(csv_path: Path, column_name: str = 'motor_A_shaft_deg') -> List[float]:
    """Load trajectory from CSV file"""
    positions = []
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        
        # Check if column exists
        if column_name not in reader.fieldnames:
            raise ValueError(f"Column '{column_name}' not found in CSV. Available: {reader.fieldnames}")
        
        for row in reader:
            try:
                pos = float(row[column_name])
                positions.append(pos)
            except ValueError:
                print(f"Warning: Invalid value in row, skipping")
                continue
    
    return positions


def playback_trajectory(
    port: serial.Serial, 
    positions: List[float], 
    rate_hz: float = 200.0,
    mode: str = 'direct',
    verbose: bool = False,
    loop: bool = False
):
    """
    Playback trajectory at specified rate
    
    Args:
        port: Serial port object
        positions: List of position setpoints (degrees)
        rate_hz: Playback rate in Hz
        mode: 'direct' or 'scurve'
        verbose: Print debug info
        loop: Loop trajectory continuously
    """
    period_s = 1.0 / rate_hz
    total_points = len(positions)
    
    print(f"\n{'='*60}")
    print(f"Starting Trajectory Playback")
    print(f"{'='*60}")
    print(f"Total Points:  {total_points}")
    print(f"Rate:          {rate_hz} Hz ({period_s*1000:.2f} ms/point)")
    print(f"Control Mode:  {mode.upper()}")
    print(f"Duration:      {total_points * period_s:.2f} seconds")
    print(f"Loop:          {'Enabled' if loop else 'Disabled'}")
    print(f"{'='*60}\n")
    
    try:
        iteration = 0
        while True:
            iteration += 1
            print(f"[Iteration {iteration}] Starting trajectory...")
            
            start_time = time.time()
            
            for idx, target_pos in enumerate(positions):
                point_start_time = time.time()
                
                # Send position command
                if mode == 'direct':
                    send_direct_position(port, target_pos, verbose=verbose)
                elif mode == 'scurve':
                    duration_ms = int(period_s * 1000)
                    send_scurve_position(port, target_pos, duration_ms, verbose=verbose)
                
                # Receive feedback (non-blocking)
                feedback = receive_feedback(port, timeout=0.005)  # Increased timeout
                
                if verbose:
                    # Verbose mode - show detailed packet info
                    if feedback and feedback.get('type') == 'status':
                        actual_pos = feedback['position_deg']
                        error = abs(actual_pos - target_pos)
                        print(f"  [RX] Actual: {actual_pos:7.2f}° | Error: {error:5.2f}°")
                else:
                    # Progress bar mode
                    if feedback and feedback.get('type') == 'status':
                        actual_pos = feedback['position_deg']
                        error = abs(actual_pos - target_pos)
                        
                        # Progress bar
                        progress = (idx + 1) / total_points * 100
                        bar_len = 40
                        filled = int(bar_len * (idx + 1) / total_points)
                        bar = '█' * filled + '░' * (bar_len - filled)
                        
                        print(f'\r[{bar}] {progress:5.1f}% | '
                              f'Point {idx+1}/{total_points} | '
                              f'Target: {target_pos:7.2f}° | '
                              f'Actual: {actual_pos:7.2f}° | '
                              f'Error: {error:5.2f}°', 
                              end='', flush=True)
                    else:
                        # No feedback - show just progress
                        progress = (idx + 1) / total_points * 100
                        bar_len = 40
                        filled = int(bar_len * (idx + 1) / total_points)
                        bar = '█' * filled + '░' * (bar_len - filled)
                        
                        print(f'\r[{bar}] {progress:5.1f}% | '
                              f'Point {idx+1}/{total_points} | '
                              f'Target: {target_pos:7.2f}° | '
                              f'[No feedback]', 
                              end='', flush=True)
                
                # Timing control - maintain constant rate
                elapsed = time.time() - point_start_time
                sleep_time = period_s - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif verbose:
                    print(f"  Warning: Running {-sleep_time*1000:.2f}ms behind schedule")
            
            # End of trajectory
            total_time = time.time() - start_time
            print(f"\n\n[Iteration {iteration}] Completed in {total_time:.2f}s "
                  f"(avg rate: {total_points/total_time:.1f} Hz)")
            
            if not loop:
                break
            
            print("Looping...\n")
            time.sleep(0.5)  # Small pause between loops
            
    except KeyboardInterrupt:
        print("\n\nPlayback stopped by user")


def send_start_command(port: serial.Serial):
    """Send ASCII 'S' to start motor"""
    port.write(b'S')
    print("[TX] Start command sent (ASCII 'S')")
    time.sleep(1.5)  # Wait for motor to initialize


def main():
    parser = argparse.ArgumentParser(
        description='Gait Control Test - CSV Trajectory Playback',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Play gait at 200 Hz with direct position control
  python test_gait_csv.py COM44 gait_setpoints_300steps_200hz.csv --rate 200
  
  # Play with S-Curve smoothing at 100 Hz
  python test_gait_csv.py COM44 gait.csv --rate 100 --mode scurve
  
  # Loop continuously with verbose output
  python test_gait_csv.py COM44 gait.csv --loop --verbose
  
  # Use different column name
  python test_gait_csv.py COM44 data.csv --column motor_B_angle
        """
    )
    
    parser.add_argument('port', help='Serial port (e.g., COM44 or /dev/ttyUSB0)')
    parser.add_argument('csv_file', help='Path to CSV file with trajectory data')
    parser.add_argument('--column', default='motor_A_shaft_deg', 
                        help='CSV column name for position data (default: motor_A_shaft_deg)')
    parser.add_argument('--rate', type=float, default=200.0, 
                        help='Playback rate in Hz (default: 200)')
    parser.add_argument('--mode', choices=['direct', 'scurve'], default='direct',
                        help='Control mode: direct (fast) or scurve (smooth) (default: direct)')
    parser.add_argument('--baudrate', type=int, default=921600,
                        help='Serial baudrate (default: 921600)')
    parser.add_argument('--loop', action='store_true',
                        help='Loop trajectory continuously')
    parser.add_argument('--verbose', action='store_true',
                        help='Print detailed debug information')
    parser.add_argument('--no-start', action='store_true',
                        help='Skip sending start command (motor already running)')
    
    args = parser.parse_args()
    
    # Validate CSV file
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1
    
    # Load trajectory
    print(f"Loading trajectory from: {csv_path}")
    try:
        positions = load_csv_trajectory(csv_path, args.column)
        print(f"Loaded {len(positions)} position setpoints from column '{args.column}'")
        print(f"Range: {min(positions):.2f}° to {max(positions):.2f}°")
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return 1
    
    # Open serial port
    try:
        print(f"\nConnecting to {args.port} at {args.baudrate} baud...")
        port = serial.Serial(args.port, args.baudrate, timeout=0.1)
        time.sleep(0.5)  # Wait for connection
        print("Connected!")
        
        # Clear any pending data
        port.reset_input_buffer()
        port.reset_output_buffer()
        
        # Send start command if needed
        if not args.no_start:
            send_start_command(port)
        
        # Start playback
        playback_trajectory(
            port, 
            positions, 
            rate_hz=args.rate,
            mode=args.mode,
            verbose=args.verbose,
            loop=args.loop
        )
        
        print("\nDone!")
        port.close()
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())

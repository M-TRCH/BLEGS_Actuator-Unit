#!/usr/bin/env python3
"""
IMU Unit - Binary Protocol Test Client
Compatible with Motor Protocol v1.2

Receives and displays IMU streaming data at 100 Hz
"""

import serial
import struct
import time
import sys

# Platform-specific non-blocking keyboard input
if sys.platform == 'win32':
    import msvcrt
    def check_key():
        """Non-blocking key check (Windows)"""
        if msvcrt.kbhit():
            try:
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            except Exception:
                return None
        return None
else:
    import select
    def check_key():
        """Non-blocking key check (Unix/Mac)"""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
        except Exception:
            pass
        return None

# Protocol Constants
PROTOCOL_HEADER1 = 0xFE
PROTOCOL_HEADER2 = 0xEE

# Packet Types
FB_IMU_DATA = 0x85
FB_IMU_RAW = 0x86
FB_IMU_CALIBRATION = 0x87
CMD_SET_ZERO = 0x06

# IMU Unit ID
IMU_UNIT_ID = 0xFF

# Status Flags
IMU_STATUS_CALIBRATED = 0x01
IMU_STATUS_GYRO_CAL = 0x02
IMU_STATUS_ACCEL_CAL = 0x04
IMU_STATUS_MAG_CAL = 0x08
IMU_STATUS_ERROR = 0x80

# Global error tracking
crc_error_count = 0

def calculate_crc16(data):
    """Calculate CRC16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)"""
    crc = 0xFFFF
    
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    
    return crc

def send_set_zero(port):
    """Send SET_ZERO command to IMU unit"""
    buffer = bytearray()
    
    # Headers
    buffer.append(PROTOCOL_HEADER1)
    buffer.append(PROTOCOL_HEADER2)
    buffer.append(CMD_SET_ZERO)
    buffer.append(0)  # No payload
    
    # Calculate CRC
    crc = calculate_crc16(buffer[2:4])  # CRC over type and length
    buffer.append((crc >> 8) & 0xFF)
    buffer.append(crc & 0xFF)
    
    # Send command
    port.write(buffer)
    port.flush()

def receive_packet(port, timeout=0.02):
    """Receive and validate a binary packet"""
    global crc_error_count
    start_time = time.time()
    
    # Look for header
    while time.time() - start_time < timeout:
        if port.in_waiting > 0:
            byte1 = port.read(1)
            if len(byte1) == 0:
                continue
            
            if byte1[0] == PROTOCOL_HEADER1:
                byte2 = port.read(1)
                if len(byte2) > 0 and byte2[0] == PROTOCOL_HEADER2:
                    # Found header, read rest of packet
                    pkt_type_byte = port.read(1)
                    length_byte = port.read(1)
                    
                    if len(pkt_type_byte) == 0 or len(length_byte) == 0:
                        continue
                    
                    pkt_type = pkt_type_byte[0]
                    payload_length = length_byte[0]
                    
                    # Read payload
                    payload = port.read(payload_length)
                    if len(payload) != payload_length:
                        print(f"[ERROR] Incomplete payload: expected {payload_length}, got {len(payload)}")
                        continue
                    
                    # Read CRC
                    crc_bytes = port.read(2)
                    if len(crc_bytes) != 2:
                        print("[ERROR] Incomplete CRC")
                        continue
                    
                    received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
                    
                    # Verify CRC
                    crc_data = bytes([pkt_type, payload_length]) + payload
                    calculated_crc = calculate_crc16(crc_data)
                    
                    if received_crc != calculated_crc:
                        crc_error_count += 1
                        continue
                    
                    return (pkt_type, payload)
    
    return None

def parse_imu_data(payload):
    """Parse IMU Data packet (0x85)"""
    if len(payload) != 10:
        print(f"[ERROR] Invalid IMU data payload length: {len(payload)}")
        return None
    
    # Unpack: unit_id, roll, pitch, yaw, sequence, status
    # Format: >BhhhHB (big-endian, uint8, 3x int16, uint16, uint8)
    # Total: 1 + 2*3 + 2 + 1 = 10 bytes
    unit_id, roll, pitch, yaw, sequence, status = struct.unpack('>BhhhHB', payload)
    
    return {
        'unit_id': unit_id,
        'roll': roll / 100.0,
        'pitch': pitch / 100.0,
        'yaw': yaw / 100.0,
        'sequence': sequence,
        'status': status,
        'calibrated': bool(status & IMU_STATUS_CALIBRATED),
        'gyro_cal': bool(status & IMU_STATUS_GYRO_CAL),
        'accel_cal': bool(status & IMU_STATUS_ACCEL_CAL),
        'mag_cal': bool(status & IMU_STATUS_MAG_CAL),
        'error': bool(status & IMU_STATUS_ERROR)
    }

def parse_calibration(payload):
    """Parse Calibration Status packet (0x87)"""
    if len(payload) != 5:
        print(f"[ERROR] Invalid calibration payload length: {len(payload)}")
        return None
    
    # Unpack: unit_id, system_cal, gyro_cal, accel_cal, mag_cal
    unit_id, system_cal, gyro_cal, accel_cal, mag_cal = struct.unpack('>BBBBB', payload)
    
    return {
        'unit_id': unit_id,
        'system': system_cal,
        'gyro': gyro_cal,
        'accel': accel_cal,
        'mag': mag_cal
    }

def main():
    """Main test program"""
    
    # Serial port configuration
    PORT = 'COM22'  # Change to your port
    BAUD_RATE = 921600
    
    print("="  * 60)
    print("IMU Unit - Binary Protocol Test Client")
    print("="  * 60)
    print("Commands:")
    print("  Press 'z' + Enter: Set current orientation as zero")
    print("  Press Ctrl+C: Exit")
    print("="  * 60)
    
    try:
        port = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        time.sleep(0.5)  # Wait for connection
        port.reset_input_buffer()  # Drain any stale data
        print(f"Connected to {PORT} @ {BAUD_RATE} baud\n")
    except Exception as e:
        print(f"[ERROR] Failed to open {PORT}: {e}")
        sys.exit(1)
    
    print("Receiving IMU data stream...")
    print("-" * 60)
    
    packet_count = 0
    last_print_time = time.time()
    packets_per_second = 0
    total_packets = 0
    
    last_sequence = None
    sequence_gaps = 0
    
    try:
        while True:
            # Check for keyboard input (non-blocking, cross-platform)
            key = check_key()
            if key == 'z':
                print("\n[CMD] Sending SET_ZERO command...")
                send_set_zero(port)
                print("[CMD] Waiting for acknowledgment...")
                time.sleep(0.1)
            
            result = receive_packet(port, timeout=0.015)  # 15ms timeout for 100 Hz (10ms + margin)
            
            if result:
                pkt_type, payload = result
                packet_count += 1
                total_packets += 1
                
                if pkt_type == FB_IMU_DATA:
                    data = parse_imu_data(payload)
                    if data:
                        # Track sequence gaps
                        if last_sequence is not None:
                            expected_seq = (last_sequence + 1) & 0xFFFF
                            if data['sequence'] != expected_seq:
                                sequence_gaps += 1
                        last_sequence = data['sequence']
                        
                        # Calculate packet rate
                        current_time = time.time()
                        if current_time - last_print_time >= 1.0:
                            packets_per_second = packet_count
                            packet_count = 0
                            last_print_time = current_time
                        
                        # Clear line and print data
                        cal_mark = '✓' if data['calibrated'] else '✗'
                        print(f"\r[{data['sequence']:5d}] R:{data['roll']:7.2f}° P:{data['pitch']:7.2f}° Y:{data['yaw']:7.2f}° | "
                              f"{packets_per_second:3d}Hz | "
                              f"Cal:{cal_mark} "
                              f"Err:{crc_error_count} Gap:{sequence_gaps}", end='', flush=True)
                
                elif pkt_type == FB_IMU_CALIBRATION:
                    cal = parse_calibration(payload)
                    if cal:
                        print(f"\n[CALIBRATION] System: {cal['system']}/3 | "
                              f"Gyro: {cal['gyro']}/3 | Accel: {cal['accel']}/3 | Mag: {cal['mag']}/3")
                        print("-" * 60)
                
                else:
                    print(f"\n[UNKNOWN] Packet type: 0x{pkt_type:02X}")
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
        print(f"Total packets received: {total_packets}")
        if crc_error_count > 0:
            print(f"CRC errors: {crc_error_count}")
        if sequence_gaps > 0:
            print(f"Sequence gaps: {sequence_gaps}")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        port.close()
        print("Port closed")

if __name__ == "__main__":
    main()

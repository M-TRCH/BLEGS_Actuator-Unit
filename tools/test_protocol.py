#!/usr/bin/env python3
"""
High-Speed Binary Protocol - Python Test Client
Example implementation for controlling motor via binary protocol

Author: M-TRCH
Date: 2025-12-03
Version: 1.0
"""

import struct
import serial
import time
from enum import IntEnum
from typing import Optional, Tuple

# Protocol Constants
HEADER_1 = 0xFE
HEADER_2 = 0xEE

class PacketType(IntEnum):
    """Packet type enumeration"""
    PKT_CMD_SET_GOAL = 0x01
    PKT_CMD_SET_CONFIG = 0x02
    PKT_CMD_PING = 0x03
    PKT_FB_STATUS = 0x81
    PKT_FB_CONFIG = 0x82
    PKT_FB_ERROR = 0x83
    PKT_FB_PONG = 0x84

class ControlMode(IntEnum):
    """Control mode enumeration"""
    MODE_DIRECT_POSITION = 0x00
    MODE_SCURVE_PROFILE = 0x01

class StatusFlags(IntEnum):
    """Status flags bitmask"""
    STATUS_MOVING = (1 << 0)
    STATUS_ERROR = (1 << 1)
    STATUS_AT_GOAL = (1 << 2)
    STATUS_OVERHEAT = (1 << 3)
    STATUS_OVERCURRENT = (1 << 4)
    STATUS_ENCODER_ERROR = (1 << 5)

class ErrorCode(IntEnum):
    """Error code enumeration"""
    ERR_NONE = 0x00
    ERR_CRC_FAILED = 0x01
    ERR_INVALID_PACKET = 0x02
    ERR_TIMEOUT = 0x03
    ERR_UNKNOWN_COMMAND = 0x04
    ERR_INVALID_PAYLOAD = 0x05
    ERR_MOTOR_FAULT = 0x06


def calculate_crc16(data: bytes) -> int:
    """
    Calculate CRC-16-IBM for data buffer
    
    Args:
        data: Bytes to calculate CRC over
        
    Returns:
        CRC-16 value
    """
    crc = 0xFFFF
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    
    return crc & 0xFFFF


def send_direct_position(port: serial.Serial, target_degrees: float) -> bool:
    """
    Send direct position command
    
    Args:
        port: Serial port object
        target_degrees: Target position in degrees
        
    Returns:
        True if sent successfully
    """
    # Build packet
    header = bytes([HEADER_1, HEADER_2])
    pkt_type = bytes([PacketType.PKT_CMD_SET_GOAL])
    
    # Build payload: control_mode + target_pos (int32, degrees*100)
    mode = bytes([ControlMode.MODE_DIRECT_POSITION])
    target_pos = struct.pack('<i', int(target_degrees * 100))
    
    payload = mode + target_pos
    payload_len = bytes([len(payload)])
    
    # Calculate CRC over: pkt_type + payload_len + payload
    crc_data = pkt_type + payload_len + payload
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    # Assemble and send packet
    packet = header + pkt_type + payload_len + payload + crc_bytes
    port.write(packet)
    
    print(f"[TX] Direct Position: {target_degrees}°")
    print(f"     Packet: {packet.hex(' ')}")
    
    return True


def send_scurve_position(port: serial.Serial, target_degrees: float, duration_ms: int) -> bool:
    """
    Send S-Curve position command
    
    Args:
        port: Serial port object
        target_degrees: Target position in degrees
        duration_ms: Duration in milliseconds
        
    Returns:
        True if sent successfully
    """
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
    
    print(f"[TX] S-Curve Position: {target_degrees}° in {duration_ms}ms")
    print(f"     Packet: {packet.hex(' ')}")
    
    return True


def send_ping(port: serial.Serial) -> bool:
    """
    Send ping command
    
    Args:
        port: Serial port object
        
    Returns:
        True if sent successfully
    """
    # Build packet
    header = bytes([HEADER_1, HEADER_2])
    pkt_type = bytes([PacketType.PKT_CMD_PING])
    payload = b''
    payload_len = bytes([0])
    
    # Calculate CRC
    crc_data = pkt_type + payload_len
    crc = calculate_crc16(crc_data)
    crc_bytes = struct.pack('<H', crc)
    
    # Assemble and send packet
    packet = header + pkt_type + payload_len + crc_bytes
    port.write(packet)
    
    print(f"[TX] Ping")
    print(f"     Packet: {packet.hex(' ')}")
    
    return True


def send_start_command(port: serial.Serial) -> bool:
    """
    Send ASCII start command to begin motor operation
    
    Args:
        port: Serial port object
        
    Returns:
        True if sent successfully
    """
    port.write(b'S')
    print(f"[TX] Start Command (ASCII 'S')")
    time.sleep(0.1)  # Wait for motor to start
    return True


def receive_packet(port: serial.Serial, timeout: float = 0.1) -> Optional[Tuple[int, bytes]]:
    """
    Receive and parse a binary packet
    
    Args:
        port: Serial port object
        timeout: Timeout in seconds
        
    Returns:
        Tuple of (packet_type, payload) if successful, None otherwise
    """
    port.timeout = timeout
    
    # Read header
    header = port.read(2)
    if len(header) != 2 or header[0] != HEADER_1 or header[1] != HEADER_2:
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
    
    # Verify CRC
    crc_data = bytes([pkt_type, payload_len]) + payload
    calculated_crc = calculate_crc16(crc_data)
    
    if received_crc != calculated_crc:
        print(f"[RX] CRC ERROR: Expected {calculated_crc:04X}, Got {received_crc:04X}")
        return None
    
    return (pkt_type, payload)


def parse_status_feedback(payload: bytes) -> dict:
    """
    Parse status feedback payload
    
    Args:
        payload: Payload bytes
        
    Returns:
        Dictionary with parsed data
    """
    if len(payload) < 7:
        return None
    
    actual_pos_raw = struct.unpack('<i', payload[0:4])[0]
    actual_current = struct.unpack('<h', payload[4:6])[0]
    status_flags = payload[6]
    
    return {
        'position_deg': actual_pos_raw / 100.0,
        'current_ma': actual_current,
        'flags': status_flags,
        'is_moving': bool(status_flags & StatusFlags.STATUS_MOVING),
        'at_goal': bool(status_flags & StatusFlags.STATUS_AT_GOAL),
        'error': bool(status_flags & StatusFlags.STATUS_ERROR)
    }


def parse_error_feedback(payload: bytes) -> dict:
    """
    Parse error feedback payload
    
    Args:
        payload: Payload bytes
        
    Returns:
        Dictionary with parsed data
    """
    if len(payload) < 4:
        return None
    
    error_code = payload[0]
    last_pkt_type = payload[1]
    debug_info = struct.unpack('<H', payload[2:4])[0]
    
    return {
        'error_code': error_code,
        'last_packet_type': last_pkt_type,
        'debug_info': debug_info
    }


def main():
    """Main test function"""
    # Configure serial port
    PORT = 'COM44'  # Change to your port
    BAUDRATE = 921600
    
    print("=" * 60)
    print("High-Speed Binary Protocol - Test Client")
    print("=" * 60)
    
    try:
        # Open serial port
        port = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        time.sleep(0.5)  # Wait for connection
        
        print(f"Connected to {PORT} @ {BAUDRATE} baud\n")
        
        # Send start command to begin motor operation
        print("\n--- Sending Start Command ---")
        send_start_command(port)
        print("Motor should now be running...\n")
        time.sleep(3.0)  # Wait for motor to initialize
        
        # Test 1: Ping
        print("\n--- Test 1: Ping ---")
        send_ping(port)
        time.sleep(0.05)
        
        result = receive_packet(port)
        if result:
            pkt_type, payload = result
            print(f"[RX] Packet Type: 0x{pkt_type:02X}")
            if pkt_type == PacketType.PKT_FB_STATUS:
                status = parse_status_feedback(payload)
                print(f"     Position: {status['position_deg']:.2f}°")
                print(f"     Current: {status['current_ma']} mA")
                print(f"     Flags: 0x{status['flags']:02X}")
        
        # Test 2: Direct Position Command
        print("\n--- Test 2: Direct Position Command ---")
        send_direct_position(port, -45.0)
        time.sleep(0.05)
        
        result = receive_packet(port)
        if result:
            pkt_type, payload = result
            if pkt_type == PacketType.PKT_FB_STATUS:
                status = parse_status_feedback(payload)
                print(f"[RX] Status Feedback:")
                print(f"     Position: {status['position_deg']:.2f}°")
                print(f"     Moving: {status['is_moving']}")
                print(f"     At Goal: {status['at_goal']}")
        
        # Test 3: S-Curve Position Command
        print("\n--- Test 3: S-Curve Position Command ---")
        send_scurve_position(port, 0.0, 1000)
        time.sleep(0.05)
        
        result = receive_packet(port)
        if result:
            pkt_type, payload = result
            if pkt_type == PacketType.PKT_FB_STATUS:
                status = parse_status_feedback(payload)
                print(f"[RX] Status Feedback:")
                print(f"     Position: {status['position_deg']:.2f}°")
        
        # Test 4: Multiple position commands
        print("\n--- Test 4: Sequence of Positions ---")
        positions = [45.0, -45.0, 0.0]
        for pos in positions:
            send_scurve_position(port, pos, 500)
            time.sleep(0.05)
            result = receive_packet(port)
            if result:
                pkt_type, payload = result
                if pkt_type == PacketType.PKT_FB_STATUS:
                    status = parse_status_feedback(payload)
                    print(f"[RX] Target: {pos:6.2f}°, Current: {status['position_deg']:6.2f}°")
        
        port.close()
        print("\n" + "=" * 60)
        print("Test completed successfully!")
        print("=" * 60)
        
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")


if __name__ == '__main__':
    main()

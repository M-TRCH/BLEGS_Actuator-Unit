"""
IMU Reader for BNO055 - Binary Protocol Integration
Author: M-TRCH
Date: February 11, 2026

This module provides IMU reading capabilities for yaw control in the quadruped robot.
Based on test_bno055_imu.py with thread-safe non-blocking operation.
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
from typing import Optional, Dict


# Protocol Constants
PROTOCOL_HEADER1 = 0xFE
PROTOCOL_HEADER2 = 0xEE

# Packet Types
FB_IMU_DATA = 0x85
FB_IMU_CALIBRATION = 0x87
CMD_SET_ZERO = 0x06

# Status Flags
IMU_STATUS_CALIBRATED = 0x01
IMU_STATUS_ERROR = 0x80


class IMUReader:
    """
    Thread-safe IMU Reader for BNO055 using binary protocol.
    
    This class runs a background thread to continuously read IMU data
    and provides thread-safe access to the latest orientation (yaw).
    
    Attributes:
        yaw (float): Current yaw angle in degrees (thread-safe)
        roll (float): Current roll angle in degrees (thread-safe)
        pitch (float): Current pitch angle in degrees (thread-safe)
        calibrated (bool): Whether IMU is calibrated
        connected (bool): Whether IMU is connected and streaming
    """
    
    def __init__(self, port: str, baud_rate: int = 921600):
        """
        Initialize IMU reader.
        
        Args:
            port: Serial port (e.g., 'COM22')
            baud_rate: Baud rate (default: 921600)
        """
        self.port = port
        self.baud_rate = baud_rate
        
        # Serial connection
        self._serial = None
        
        # IMU state (thread-safe)
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._calibrated = False
        self._error = False
        self._last_update_time = None
        self._packet_count = 0
        self._crc_errors = 0
        
        # Connection state
        self._connected = False
        self._running = False
        
        # Background thread
        self._read_thread = None
        
        # Debug
        self._first_packet_received = False
        
    def connect(self) -> bool:
        """
        Connect to IMU and start reading thread.
        
        Returns:
            True if connected successfully
        """
        try:
            # Check if port exists
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
            if self.port not in available_ports:
                print(f"  ❌ Port {self.port} not found")
                print(f"  💡 Available ports: {', '.join(available_ports) if available_ports else 'None'}")
                self._connected = False
                return False
            
            print(f"  🔌 Opening serial port: {self.port} @ {self.baud_rate} baud...")
            self._serial = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(0.5)  # Wait for connection
            self._serial.reset_input_buffer()  # Drain stale data
            
            self._connected = True
            self._running = True
            
            # Start background reading thread
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            
            return True
            
        except serial.SerialException as e:
            print(f"  ❌ Serial port error: {e}")
            if "PermissionError" in str(e) or "Access is denied" in str(e):
                print(f"  💡 Port may be in use by another program")
            elif "FileNotFoundError" in str(e) or "could not open port" in str(e):
                print(f"  💡 Port {self.port} not found - check if device is connected")
            self._connected = False
            return False
        except Exception as e:
            print(f"  ❌ Unexpected error: {type(e).__name__}: {e}")
            self._connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from IMU and stop reading thread."""
        self._running = False
        
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        
        if self._serial and self._serial.is_open:
            self._serial.close()
        
        self._connected = False
        print("  📡 IMU disconnected")
    
    def set_zero(self) -> bool:
        """
        Set current orientation as zero reference.
        
        Returns:
            True if command sent successfully
        """
        if not self._connected or not self._serial:
            return False
        
        try:
            buffer = bytearray()
            
            # Headers
            buffer.append(PROTOCOL_HEADER1)
            buffer.append(PROTOCOL_HEADER2)
            buffer.append(CMD_SET_ZERO)
            buffer.append(0)  # No payload
            
            # Calculate CRC
            crc = self._calculate_crc16(buffer[2:4])
            buffer.append((crc >> 8) & 0xFF)
            buffer.append(crc & 0xFF)
            
            # Send command
            self._serial.write(buffer)
            self._serial.flush()
            
            print("  🧭 IMU zero set")
            return True
            
        except Exception as e:
            print(f"  ❌ Failed to set IMU zero: {e}")
            return False
    
    def get_yaw(self) -> float:
        """
        Get current yaw angle (thread-safe).
        
        Returns:
            Yaw angle in degrees
            Convention: Left turn = negative (-), Right turn = positive (+)
        """
        with self._lock:
            return self._yaw
    
    def get_orientation(self) -> Dict[str, float]:
        """
        Get current orientation (thread-safe).
        
        Returns:
            Dict with 'roll', 'pitch', 'yaw' in degrees
        """
        with self._lock:
            return {
                'roll': self._roll,
                'pitch': self._pitch,
                'yaw': self._yaw,
                'calibrated': self._calibrated,
                'error': self._error
            }
    
    def is_calibrated(self) -> bool:
        """Check if IMU is calibrated (thread-safe)."""
        with self._lock:
            return self._calibrated
    
    def is_connected(self) -> bool:
        """Check if IMU serial port is connected."""
        return self._connected and self._serial is not None and self._serial.is_open
    
    def is_receiving_data(self) -> bool:
        """Check if IMU is actively receiving data."""
        with self._lock:
            if self._last_update_time is None:
                return False
            
            # Check if we received data in last 0.5 seconds
            return (time.time() - self._last_update_time) < 0.5
    
    def get_stats(self) -> Dict:
        """Get reader statistics."""
        with self._lock:
            return {
                'packets': self._packet_count,
                'crc_errors': self._crc_errors,
                'connected': self.is_connected(),
                'receiving_data': self.is_receiving_data()
            }
    
    # ========================================================================
    # PRIVATE METHODS
    # ========================================================================
    
    def _read_loop(self) -> None:
        """Background thread loop for reading IMU data."""
        while self._running:
            try:
                result = self._receive_packet(timeout=0.02)
                
                if result:
                    pkt_type, payload = result
                    
                    if pkt_type == FB_IMU_DATA:
                        self._parse_imu_data(payload)
                    elif pkt_type == FB_IMU_CALIBRATION:
                        self._parse_calibration(payload)
                        
            except Exception as e:
                # Don't spam errors, just continue
                if self._running:  # Only log if intentionally running
                    pass  # Silent fail for robustness
                    
            time.sleep(0.001)  # Small sleep to prevent CPU spinning
    
    def _receive_packet(self, timeout: float = 0.02) -> Optional[tuple]:
        """
        Receive and validate a binary packet.
        
        Args:
            timeout: Timeout in seconds
        
        Returns:
            (packet_type, payload) if successful, None otherwise
        """
        if not self._serial or not self._serial.is_open:
            return None
        
        start_time = time.time()
        
        # Look for header
        while time.time() - start_time < timeout:
            if self._serial.in_waiting > 0:
                byte1 = self._serial.read(1)
                if len(byte1) == 0:
                    continue
                
                if byte1[0] == PROTOCOL_HEADER1:
                    byte2 = self._serial.read(1)
                    if len(byte2) > 0 and byte2[0] == PROTOCOL_HEADER2:
                        # Found header, read rest of packet
                        pkt_type_byte = self._serial.read(1)
                        length_byte = self._serial.read(1)
                        
                        if len(pkt_type_byte) == 0 or len(length_byte) == 0:
                            continue
                        
                        pkt_type = pkt_type_byte[0]
                        payload_length = length_byte[0]
                        
                        # Read payload
                        payload = self._serial.read(payload_length)
                        if len(payload) != payload_length:
                            continue
                        
                        # Read CRC
                        crc_bytes = self._serial.read(2)
                        if len(crc_bytes) != 2:
                            continue
                        
                        received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
                        
                        # Verify CRC
                        crc_data = bytes([pkt_type, payload_length]) + payload
                        calculated_crc = self._calculate_crc16(crc_data)
                        
                        if received_crc != calculated_crc:
                            with self._lock:
                                self._crc_errors += 1
                            continue
                        
                        return (pkt_type, payload)
        
        return None
    
    def _parse_imu_data(self, payload: bytes) -> None:
        """Parse IMU Data packet (0x85)."""
        if len(payload) != 10:
            return
        
        try:
            # Unpack: unit_id, roll, pitch, yaw, sequence, status
            unit_id, roll, pitch, yaw, sequence, status = struct.unpack('>BhhhHB', payload)
            
            # Update state (thread-safe)
            with self._lock:
                self._roll = roll / 100.0
                self._pitch = pitch / 100.0
                self._yaw = yaw / 100.0
                self._calibrated = bool(status & IMU_STATUS_CALIBRATED)
                self._error = bool(status & IMU_STATUS_ERROR)
                self._last_update_time = time.time()
                self._packet_count += 1
                
                # Print first packet received
                if not self._first_packet_received:
                    self._first_packet_received = True
                    print(f"  ✅ First IMU packet received (Yaw: {self._yaw:+.1f}°)")
                
        except Exception:
            pass  # Silent fail
    
    def _parse_calibration(self, payload: bytes) -> None:
        """Parse Calibration Status packet (0x87)."""
        if len(payload) != 5:
            return
        
        try:
            unit_id, system_cal, gyro_cal, accel_cal, mag_cal = struct.unpack('>BBBBB', payload)
            
            # Could store calibration levels if needed
            # For now, just acknowledge receipt
            
        except Exception:
            pass  # Silent fail
    
    def _calculate_crc16(self, data: bytes) -> int:
        """Calculate CRC16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)."""
        crc = 0xFFFF
        
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        
        return crc


# ============================================================================
# CONVENIENCE FUNCTIONS
# ============================================================================

def create_imu_reader(port: str = 'COM22', auto_connect: bool = True) -> Optional[IMUReader]:
    """
    Create and optionally connect IMU reader.
    
    Args:
        port: Serial port for IMU
        auto_connect: Auto-connect on creation
    
    Returns:
        IMUReader instance if successful, None otherwise
    """
    reader = IMUReader(port)
    
    if auto_connect:
        if not reader.connect():
            return None
    
    return reader

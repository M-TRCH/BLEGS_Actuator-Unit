"""
GY-25 Tilt Angle Sensor Test Script
------------------------------------
This script tests the GY-25 gyroscope/accelerometer module via Serial Port (USB2TTL - CO17).
The GY-25 provides roll and pitch angle measurements over UART communication.

Hardware:
- GY-25 Tilt Angle Sensor Module
- USB2TTL Converter (CO17)

Connection:
- GY-25 TX  -> USB2TTL RX
- GY-25 RX  -> USB2TTL TX
- GY-25 VCC -> 5V
- GY-25 GND -> GND

Protocol:
- Baud Rate: 115200 (default), 9600 (configurable)
- Data Format: 8N1 (8 data bits, no parity, 1 stop bit)
- Output Mode: Auto output or query mode
- Data Frame: [0xAA] [Yaw_H] [Yaw_L] [Pitch_H] [Pitch_L] [Roll_H] [Roll_L] [0x55]
"""

import serial
import serial.tools.list_ports
import time
import struct
import sys


class GY25Sensor:
    """GY-25 Tilt Angle Sensor Interface"""
    
    # Command bytes
    CMD_AUTO_OUTPUT = b'\xA5\x54'      # Enable automatic output mode
    CMD_QUERY_MODE = b'\xA5\x55'       # Enable query mode
    CMD_QUERY_ANGLE = b'\xA5\x51'      # Query angle data
    CMD_CALIBRATE = b'\xA5\x53'        # Calibrate sensor
    CMD_ZERO = b'\xA5\x52'             # Set current angle as zero
    
    # Baud rates
    BAUD_9600 = 9600
    BAUD_115200 = 115200
    
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Initialize GY-25 sensor connection
        
        Args:
            port: Serial port name (e.g., 'COM3')
            baudrate: Communication baud rate (default: 115200)
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        
    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            time.sleep(0.5)  # Wait for connection to stabilize
            
            # Clear any existing data in buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.2)
            
            self.connected = True
            print(f"✓ Connected to GY-25 on {self.port} at {self.baudrate} baud")
            print("  Buffers cleared, sensor ready")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            # Clear buffers before closing
            try:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                time.sleep(0.1)
            except:
                pass
            
            self.serial.close()
            self.connected = False
            print("✓ Disconnected from GY-25")
    
    def set_auto_output_mode(self):
        """Set sensor to automatic output mode"""
        if not self.connected:
            return False
        try:
            # Clear buffer before sending command
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.1)
            
            # Send auto-output command
            self.serial.write(self.CMD_AUTO_OUTPUT)
            time.sleep(0.3)  # Wait longer for mode change
            
            # Clear any response/garbage data
            self.serial.reset_input_buffer()
            time.sleep(0.1)
            
            print("✓ Set to auto-output mode")
            return True
        except Exception as e:
            print(f"✗ Failed to set auto-output mode: {e}")
            return False
    
    def set_query_mode(self):
        """Set sensor to query mode (manual request)"""
        if not self.connected:
            return False
        try:
            self.serial.write(self.CMD_QUERY_MODE)
            time.sleep(0.1)
            print("✓ Set to query mode")
            return True
        except Exception as e:
            print(f"✗ Failed to set query mode: {e}")
            return False
    
    def calibrate(self):
        """Calibrate the sensor (place on level surface)"""
        if not self.connected:
            return False
        try:
            print("Calibrating... Keep sensor on level surface!")
            self.serial.write(self.CMD_CALIBRATE)
            time.sleep(2)
            print("✓ Calibration complete")
            return True
        except Exception as e:
            print(f"✗ Calibration failed: {e}")
            return False
    
    def set_zero(self):
        """Set current position as zero reference"""
        if not self.connected:
            return False
        try:
            self.serial.write(self.CMD_ZERO)
            time.sleep(0.1)
            print("✓ Zero position set")
            return True
        except Exception as e:
            print(f"✗ Failed to set zero: {e}")
            return False
    
    def read_angle_data(self, timeout_sec=0.2):
        """
        Read angle data from sensor
        
        Returns:
            dict: {'roll': float, 'pitch': float, 'yaw': float} in degrees, or None if error
        
        GY-25 Data Format (8 bytes):
        - Header: 0xAA
        - Yaw: 2 bytes (high byte first)
        - Pitch: 2 bytes (high byte first)
        - Roll: 2 bytes (high byte first)
        - Footer: 0x55
        Total: 8 bytes
        """
        if not self.connected:
            return None
        
        try:
            # Wait for enough data
            timeout = time.time() + timeout_sec
            while self.serial.in_waiting < 8 and time.time() < timeout:
                time.sleep(0.001)
            
            # Read available data and search for header
            if self.serial.in_waiting >= 8:
                # Read all available bytes (up to 50 bytes)
                bytes_to_read = min(50, self.serial.in_waiting)
                buffer = self.serial.read(bytes_to_read)
                
                # Search for header (0xAA) and footer (0x55)
                for i in range(len(buffer) - 7):
                    if buffer[i] == 0xAA and buffer[i+7] == 0x55:
                        # Found valid frame, extract data
                        data = buffer[i:i+8]
                        
                        # Extract yaw, pitch, roll (16-bit integers, high byte first)
                        yaw_raw = (data[1] << 8) | data[2]
                        pitch_raw = (data[3] << 8) | data[4]
                        roll_raw = (data[5] << 8) | data[6]
                        
                        # Convert to signed integer (handle negative values)
                        if yaw_raw > 32767:
                            yaw_raw -= 65536
                        if pitch_raw > 32767:
                            pitch_raw -= 65536
                        if roll_raw > 32767:
                            roll_raw -= 65536
                        
                        # Convert to degrees (divide by 100)
                        yaw = yaw_raw / 100.0
                        pitch = pitch_raw / 100.0
                        roll = roll_raw / 100.0
                        
                        return {
                            'yaw': yaw,
                            'pitch': pitch,
                            'roll': roll,
                            'valid': True
                        }
            
            return None
            
        except Exception as e:
            print(f"✗ Read error: {e}")
            return None
    
    def read_raw_data(self, duration=10):
        """
        Read and display raw bytes from sensor
        
        Args:
            duration: Reading duration in seconds
        """
        if not self.connected:
            print("✗ Not connected!")
            return
        
        print(f"\nReading raw data for {duration} seconds...")
        print("Displaying raw bytes (HEX format):")
        print("-" * 60)
        
        start_time = time.time()
        byte_count = 0
        
        try:
            while (time.time() - start_time) < duration:
                if self.serial.in_waiting > 0:
                    # Read all available bytes
                    raw_bytes = self.serial.read(self.serial.in_waiting)
                    
                    # Display in hex format
                    hex_str = ' '.join([f'{b:02X}' for b in raw_bytes])
                    ascii_str = ''.join([chr(b) if 32 <= b < 127 else '.' for b in raw_bytes])
                    
                    print(f"[{byte_count:04d}] HEX: {hex_str}")
                    print(f"       ASC: {ascii_str}")
                    print("-" * 60)
                    
                    byte_count += len(raw_bytes)
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n✓ Reading stopped by user")
        
        print(f"\nTotal bytes received: {byte_count}")
        print("✓ Raw data reading complete")
    
    def continuous_read(self, duration=10, frequency=10):
        """
        Continuously read and display angle data
        
        Args:
            duration: Reading duration in seconds
            frequency: Approximate reading frequency in Hz
        """
        if not self.connected:
            print("✗ Not connected!")
            return
        
        print(f"\nReading angle data for {duration} seconds...")
        print("Yaw (°)   | Pitch (°) | Roll (°)")
        print("-" * 40)
        
        start_time = time.time()
        read_count = 0
        error_count = 0
        
        try:
            while (time.time() - start_time) < duration:
                data = self.read_angle_data(timeout_sec=0.1)
                if data:
                    print(f"{data['yaw']:7.2f}  | {data['pitch']:8.2f}  | {data['roll']:7.2f}  [{read_count:04d}]", end='\r')
                    sys.stdout.flush()
                    read_count += 1
                else:
                    error_count += 1
                    if error_count > 10:
                        print(f"\n⚠ Warning: {error_count} consecutive read errors")
                        error_count = 0
                
                # Don't sleep too long - let it read as fast as data comes
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n\n✓ Reading stopped by user")
        
        print(f"\n✓ Reading complete - {read_count} frames read")
        if error_count > 0:
            print(f"  Note: {error_count} read errors occurred")


def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("\nAvailable Serial Ports:")
    print("-" * 50)
    if ports:
        for port in ports:
            print(f"  {port.device}: {port.description}")
    else:
        print("  No serial ports found")
    print("-" * 50)
    return [port.device for port in ports]


def main():
    """Main test function"""
    print("=" * 60)
    print("GY-25 Tilt Angle Sensor Test")
    print("USB2TTL - CO17 Serial Interface")
    print("=" * 60)
    
    # List available ports
    available_ports = list_serial_ports()
    
    if not available_ports:
        print("\n✗ No serial ports detected. Check USB2TTL connection.")
        return
    
    # Configuration
    PORT = input(f"\nEnter COM port (e.g., {available_ports[0]}): ").strip() or available_ports[0]
    BAUDRATE = int(input("Enter baud rate [9600/115200] (default: 115200): ").strip() or "115200")
    
    # Create sensor instance
    sensor = GY25Sensor(port=PORT, baudrate=BAUDRATE)
    
    # Connect to sensor
    if not sensor.connect():
        return
    
    try:
        # Set to auto-output mode (only once at startup)
        print("\nInitializing sensor...")
        sensor.set_auto_output_mode()
        
        # Wait and verify data is coming
        print("Waiting for sensor data...")
        for i in range(5):
            test_data = sensor.read_angle_data(timeout_sec=0.5)
            if test_data:
                print(f"✓ Sensor is sending data (Yaw: {test_data['yaw']:.1f}°, Pitch: {test_data['pitch']:.1f}°, Roll: {test_data['roll']:.1f}°)")
                break
            time.sleep(0.2)
        else:
            print("⚠ Warning: No data received from sensor. It may need power cycle.")
        
        # Menu
        while True:
            print("\n" + "=" * 60)
            print("GY-25 Sensor Menu:")
            print("  1. Read RAW data (HEX format)")
            print("  2. Read angle data (continuous)")
            print("  3. Read single measurement")
            print("  4. Calibrate sensor")
            print("  5. Set zero position")
            print("  6. Switch to query mode")
            print("  7. Switch to auto-output mode")
            print("  0. Exit")
            print("=" * 60)
            
            choice = input("Select option: ").strip()
            
            if choice == '1':
                duration = int(input("Duration (seconds, default 5): ").strip() or "5")
                sensor.read_raw_data(duration=duration)
            
            elif choice == '2':
                duration = int(input("Duration (seconds, default 10): ").strip() or "10")
                freq = int(input("Frequency (Hz, default 10): ").strip() or "10")
                sensor.continuous_read(duration=duration, frequency=freq)
            
            elif choice == '3':
                print("\nReading single measurement...")
                data = sensor.read_angle_data()
                if data:
                    print(f"  Yaw:      {data['yaw']:7.2f}°")
                    print(f"  Pitch:    {data['pitch']:7.2f}°")
                    print(f"  Roll:     {data['roll']:7.2f}°")
                else:
                    print("✗ No data received")
            
            elif choice == '4':
                sensor.calibrate()
            
            elif choice == '5':
                sensor.set_zero()
            
            elif choice == '6':
                sensor.set_query_mode()
            
            elif choice == '7':
                sensor.set_auto_output_mode()
            
            elif choice == '0':
                break
            
            else:
                print("✗ Invalid option")
    
    except KeyboardInterrupt:
        print("\n\n✓ Program interrupted by user")
    
    finally:
        # Cleanup
        sensor.disconnect()
        print("\n" + "=" * 60)
        print("Test complete!")
        print("=" * 60)


if __name__ == "__main__":
    main()

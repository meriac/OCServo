# OCServo.py - Python library for controlling ROBS-802 servos via serial communication
#
# Copyright 2025 Milosch Meriac <milosch@meriac.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import serial
import struct
import time

class OCServo:
    """OCServo protocol implementation for ROBS-802 servo"""

    # Instruction constants
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03

    # Memory addresses
    ADDR_ID = 0x05
    ADDR_P_GAIN = 0x15
    ADDR_D_GAIN = 0x16
    ADDR_I_GAIN = 0x17
    ADDR_RUNNING_MODE = 0x23
    ADDR_GOAL_POSITION_L = 0x2A
    ADDR_GOAL_POSITION_H = 0x2B
    ADDR_GOAL_TIME_L = 0x2C
    ADDR_GOAL_TIME_H = 0x2D
    ADDR_GOAL_SPEED_L = 0x2E
    ADDR_GOAL_SPEED_H = 0x2F
    ADDR_LOCK = 0x30  # Lock register address
    ADDR_CURRENT_POSITION_L = 0x38
    ADDR_CURRENT_POSITION_H = 0x39
    ADDR_TEMPERATURE = 0x3F

    # Class variables for lock settings (immutable tuples)
    # Addresses that remain writable when servo is locked
    ADDR_WRITABLE_WHEN_LOCKED = (
        ADDR_GOAL_POSITION_L,
        ADDR_GOAL_POSITION_H,
        ADDR_GOAL_TIME_L,
        ADDR_GOAL_TIME_H,
        ADDR_GOAL_SPEED_L,
        ADDR_GOAL_SPEED_H
    )

    # Addresses that are protected when servo is locked
    ADDR_PROTECTED_WHEN_LOCKED = (
        ADDR_ID,
        ADDR_P_GAIN,
        ADDR_D_GAIN,
        ADDR_I_GAIN,
        ADDR_RUNNING_MODE,
        ADDR_LOCK
    )

    def __init__(self, port, baudrate=1000000, servo_id=1):
        """
        Initialize OCServo connection

        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' or 'COM3')
            baudrate: Communication speed (default 1M)
            servo_id: Servo ID (default 1)
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        self.servo_id = servo_id
        self._debug = False
        self._motor_mode = False  # Track current mode (False=servo, True=motor)

        # Flush any pending data in the serial buffers
        self.ser.reset_input_buffer()  # Clear input buffer
        self.ser.reset_output_buffer()  # Clear output buffer

        # Also read and discard any remaining data
        self.ser.timeout = 0.01  # Temporary short timeout for flushing
        while self.ser.read(1):  # Read until buffer is empty
            pass
        self.ser.timeout = 0.1  # Restore normal timeout

        # Verify baudrate was set correctly
        actual_baudrate = self.get_baudrate()
        if actual_baudrate != baudrate:
            print(f"WARNING: Baudrate mismatch! Requested {baudrate}, but got {actual_baudrate}")
        elif self._debug:
            print(f"Baudrate verified: {actual_baudrate}")

    def __del__(self):
        """Close serial connection on deletion"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

    def debug(self, enabled):
        """
        Enable or disable debug mode for packet hexdump

        Args:
            enabled: True to enable debug output, False to disable
        """
        self._debug = enabled

    def get_baudrate(self):
        """
        Get the actual baudrate of the serial port

        Returns:
            The baudrate currently set on the serial port
        """
        return self.ser.baudrate

    def _hexdump(self, data, label=""):
        """Create hexdump string for debugging"""
        hex_str = ' '.join(f'{b:02X}' for b in data)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
        return f"{label}[{len(data)} bytes]: {hex_str}  |{ascii_str}|"

    def _calculate_checksum(self, data):
        """Calculate checksum for packet"""
        return (~sum(data)) & 0xFF

    def _send_packet(self, instruction, parameters=None):
        """
        Send a packet to the servo

        Args:
            instruction: Instruction byte
            parameters: List of parameter bytes

        Returns:
            Response packet or None (error)
        """
        if parameters is None:
            parameters = []

        try:
            # Clear any stale data before sending
            self.ser.reset_input_buffer()

            # Build packet
            packet = [0xFF, 0xFF]  # Prefix
            packet.append(self.servo_id)
            packet.append(len(parameters) + 2)  # Length
            packet.append(instruction)
            packet.extend(parameters)

            # Calculate checksum
            checksum_data = packet[2:]  # ID, Length, Instruction, Parameters
            checksum = self._calculate_checksum(checksum_data)
            packet.append(checksum)

            # Send packet
            packet_bytes = bytes(packet)
            if self._debug:
                print(self._hexdump(packet_bytes, "TX"))
            self.ser.write(packet_bytes)
            self.ser.flush()  # Ensure data is sent

            # Always read response
            response = self._read_response()
            return response

        except serial.SerialException as e:
            if self._debug:
                print(f"Serial error in _send_packet: {e}")
            return None

    def _read_response(self):
        """Read response packet from servo"""
        try:
            # Look for prefix
            prefix = self.ser.read(2)
            if len(prefix) < 2 or prefix != b'\xFF\xFF':
                return None

            # Read ID and length
            header = self.ser.read(2)
            if len(header) < 2:
                return None

            servo_id = header[0]
            length = header[1]

            # Read remaining data
            data = self.ser.read(length)
            if len(data) < length:
                return None

            # Handle empty data case
            if len(data) == 0:
                return None

            # Verify checksum
            received_checksum = data[-1]
            # Calculate checksum on: ID, Length, Error, Parameters (everything except the checksum itself)
            checksum_data = [servo_id, length] + list(data[:-1])
            calculated_checksum = self._calculate_checksum(checksum_data)

            if received_checksum != calculated_checksum:
                if self._debug:
                    print(f"Checksum mismatch! Received: 0x{received_checksum:02X}, "
                          f"Calculated: 0x{calculated_checksum:02X}")
                    print(f"  Data: {self._hexdump(bytes([servo_id, length] + list(data)))}")
                return None

            return {
                'id': servo_id,
                'length': length,
                'error': data[0] if len(data) > 0 else 0,
                'parameters': list(data[1:-1]) if len(data) > 1 else [],
                'checksum': received_checksum
            }
        except serial.SerialException as e:
            if self._debug:
                print(f"Serial error in _read_response: {e}")
            # Try to recover by flushing buffers
            try:
                self.ser.reset_input_buffer()
            except:
                pass
            return None

    def ping(self):
        """
        Ping the servo

        Returns:
            True if servo responds, False otherwise
        """
        response = self._send_packet(self.INST_PING)
        return response is not None and response['id'] == self.servo_id

    def _set_running_mode(self, motor_mode):
        """
        Internal function to set running mode

        Args:
            motor_mode: True for motor mode (2), False for servo mode (0)

        Returns:
            True if successful, False otherwise
        """
        mode_value = 2 if motor_mode else 0
        response = self._send_packet(self.INST_WRITE, [self.ADDR_RUNNING_MODE, mode_value])
        if response is not None:
            self._motor_mode = motor_mode
            return True
        return False

    def read_temperature(self):
        """
        Read servo temperature

        Returns:
            Temperature in Celsius or None
        """
        response = self._send_packet(self.INST_READ, [self.ADDR_TEMPERATURE, 0x01])
        if response and response['parameters']:
            return response['parameters'][0]
        return None

    def read_position(self, raw_angle=False):
        """
        Read current position

        Returns:
            Position (0-4095) or None
        """
        response = self._send_packet(self.INST_READ, [self.ADDR_CURRENT_POSITION_L, 0x02])
        if response and len(response['parameters']) >= 2:
            pos_l = response['parameters'][0]
            pos_h = response['parameters'][1]
            pos = pos_l + (pos_h << 8)
            return pos if raw_angle else self.position_to_angle(pos)
        return None

    def read_voltage(self):
        """
        Read servo input voltage

        Returns:
            Voltage in volts (0.1V resolution) or None
        """
        response = self._send_packet(self.INST_READ, [0x3E, 0x01])  # Address 0x3E for voltage
        if response and response['parameters']:
            # Value is in units of 0.1V
            return response['parameters'][0] / 10.0
        return None

    def read_speed(self):
        """
        Read current servo speed

        Returns:
            Speed value or None
        """
        response = self._send_packet(self.INST_READ, [0x3A, 0x02])  # Address 0x3A-0x3B for speed
        if response and len(response['parameters']) >= 2:
            speed_l = response['parameters'][0]
            speed_h = response['parameters'][1]
            return speed_l + (speed_h << 8)
        return None

    def read_load(self):
        """
        Read current servo load/torque

        Returns:
            Load value or None
        """
        response = self._send_packet(self.INST_READ, [0x3C, 0x02])  # Address 0x3C-0x3D for load
        if response and len(response['parameters']) >= 2:
            load_l = response['parameters'][0]
            load_h = response['parameters'][1]
            load_value = load_l + (load_h << 8)
            # Bit 10 indicates direction (0=CCW, 1=CW)
            direction = "CW" if load_value & 0x400 else "CCW"
            magnitude = load_value & 0x3FF
            return {'magnitude': magnitude, 'direction': direction}
        return None

    def read_current(self):
        """
        Read servo current consumption

        Returns:
            Current in mA or None
        """
        response = self._send_packet(self.INST_READ, [0x45, 0x02])  # Address 0x45-0x46 for current
        if response and len(response['parameters']) >= 2:
            current_l = response['parameters'][0]
            current_h = response['parameters'][1]
            return current_l + (current_h << 8)
        return None

    def read_servo_status(self):
        """
        Read comprehensive servo status

        Returns:
            Dictionary with all status information or None
        """
        status = {}

        # Read all available parameters
        status['id'] = self.servo_id
        status['temperature'] = self.read_temperature()
        status['position'] = self.read_position()
        status['voltage'] = self.read_voltage()
        status['speed'] = self.read_speed()
        status['load'] = self.read_load()
        status['current'] = self.read_current()

        # Check if servo is responding
        status['responding'] = self.ping()

        return status

    def set_angle(self, angle, time_ms=0):
        """
        Set servo angle

        Args:
            angle: Angle in degrees (0-360)
            time_ms: Time to reach position in milliseconds (0 = max speed)

        Returns:
            True if successful, False otherwise
        """
        # Check if in motor mode and switch to servo mode if needed
        if self._motor_mode:
            if not self._set_running_mode(False):
                return False

        # Convert angle to position value (0-360 degrees = 0-4095)
        position = int(angle * 4095 / 360)
        position = max(0, min(4095, position))

        # Split position into low and high bytes
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF

        # Split time into low and high bytes
        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF

        # Write goal position and time
        parameters = [self.ADDR_GOAL_POSITION_L, pos_l, pos_h, time_l, time_h]
        response = self._send_packet(self.INST_WRITE, parameters)

        return response is not None

    def set_speed(self, speed):
        """
        Set servo speed (0-1023)

        Args:
            speed: Speed value (0-1023)
                   0 = stopped
                   1-1023 = speed levels (1023 is maximum speed)

        Returns:
            True if successful, False otherwise
        """
        # Clamp speed to valid range
        speed = max(0, min(1023, speed))

        # Split speed into low and high bytes
        speed_l = speed & 0xFF
        speed_h = (speed >> 8) & 0xFF

        # Write speed registers
        parameters = [self.ADDR_GOAL_SPEED_L, speed_l, speed_h]
        response = self._send_packet(self.INST_WRITE, parameters)

        return response is not None

    def motor_speed(self, torque_percent, cw=True):
        """
        Set motor speed in motor mode

        Args:
            torque_percent: Torque as percentage (0-100)
            cw: True for clockwise, False for counter-clockwise

        Returns:
            True if successful, False otherwise
        """
        # Check if in servo mode and switch to motor mode if needed
        if not self._motor_mode:
            if not self._set_running_mode(True):
                return False

        # Convert percentage to torque value (0-1000)
        torque = int(torque_percent * 10)
        torque = max(0, min(1000, torque))

        # Set direction bit (bit 10): 1 for CW, 0 for CCW
        if cw:
            torque |= 0x400  # Set bit 10

        # Split torque+direction into low and high bytes
        time_l = torque & 0xFF
        time_h = (torque >> 8) & 0xFF

        # Write to time registers (used for torque in motor mode)
        parameters = [self.ADDR_GOAL_TIME_L, time_l, time_h]
        response = self._send_packet(self.INST_WRITE, parameters)

        return response is not None

    def position_to_angle(self, position):
        """Convert position value (0-4095) to angle in degrees (0-360)"""
        return position * 360 / 4095

    def angle_to_position(self, angle):
        """Convert angle in degrees (0-360) to position value (0-4095)"""
        return int(angle * 4095 / 360)

    def set_gain(self, p_gain):
        """
        Set the proportional (P) gain for PID control

        Args:
            p_gain: Proportional gain value (1-15)

        Returns:
            bool: True if successful, False otherwise
        """
        # Limit to valid range (1-15)
        p_gain = max(1, min(15, p_gain))

        # Set P gain
        response1 = self._send_packet(self.INST_WRITE, [self.ADDR_P_GAIN, p_gain])
        # Set I and D gains to 0
        response2 = self._send_packet(self.INST_WRITE, [self.ADDR_I_GAIN, 0])
        response3 = self._send_packet(self.INST_WRITE, [self.ADDR_D_GAIN, 0])

        return response1 is not None and response2 is not None and response3 is not None

    def is_connected(self):
        """Check if serial port is open and connected"""
        try:
            return hasattr(self, 'ser') and self.ser.is_open
        except:
            return False

    def reconnect(self):
        """Attempt to reconnect to the serial port"""
        try:
            if hasattr(self, 'ser'):
                try:
                    self.ser.close()
                except:
                    pass
                time.sleep(0.1)
                self.ser.open()
                # Flush buffers after reconnection
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return True
        except Exception as e:
            if self._debug:
                print(f"Reconnection failed: {e}")
            return False

    def read_lock_status(self):
        """
        Read the lock status of the servo

        Returns:
            int: Lock status value (0=unlocked, 1=locked) or None on error
        """
        response = self._send_packet(self.INST_READ, [self.ADDR_LOCK, 0x01])
        if response and response['parameters']:
            return response['parameters'][0]
        return None

    def check_lock_settings(self):
        """
        Check comprehensive lock settings and protection status

        Returns:
            dict: Dictionary containing lock status information:
                - 'locked': Boolean indicating if servo is locked
                - 'lock_value': Raw lock register value
                - 'writable_addresses': List of addresses that can be written
                - 'protected_addresses': List of protected addresses
                - 'status_message': Human-readable status message
            None: If unable to read lock status
        """
        lock_value = self.read_lock_status()

        if lock_value is None:
            return None

        # Determine lock status
        is_locked = lock_value == 1

        # Create status message and determine which addresses are accessible
        if is_locked:
            status_message = "Servo is LOCKED - Configuration parameters are protected"
            writable = list(self.ADDR_WRITABLE_WHEN_LOCKED)
            protected = list(self.ADDR_PROTECTED_WHEN_LOCKED)
        else:
            status_message = "Servo is UNLOCKED - All parameters can be modified"
            writable = list(self.ADDR_WRITABLE_WHEN_LOCKED) + list(self.ADDR_PROTECTED_WHEN_LOCKED)
            protected = []

        return {
            'locked': is_locked,
            'lock_value': lock_value,
            'writable_addresses': writable,
            'protected_addresses': protected,
            'status_message': status_message
        }

    def lock(self, lock_state=True):
        """
        Lock or unlock the servo's EEPROM settings

        Args:
            lock_state: True to lock (protect settings), False to unlock

        Returns:
            bool: True if successful, False otherwise
        """
        lock_value = 1 if lock_state else 0

        # Send lock command
        response = self._send_packet(self.INST_WRITE, [self.ADDR_LOCK, lock_value])

        if response is not None:
            # Verify the lock state was set correctly
            time.sleep(0.1)  # Small delay for EEPROM write
            current_lock = self.read_lock_status()

            if current_lock == lock_value:
                if self._debug:
                    print(f"Servo {'locked' if lock_state else 'unlocked'} successfully")
                return True
            else:
                if self._debug:
                    print(f"Failed to {'lock' if lock_state else 'unlock'} servo - verification failed")
                return False
        else:
            if self._debug:
                print(f"Failed to send {'lock' if lock_state else 'unlock'} command")
            return False

    def unlock(self):
        """
        Unlock the servo's EEPROM settings (convenience method)

        Returns:
            bool: True if successful, False otherwise
        """
        return self.lock(False)
#!/usr/bin/env python3
"""
OCServo CLI - Command-line interface for ROBS-802 servo control
Similar to bladeRF-cli style with interactive shell and scripting support
"""

import sys
import os
import time
import argparse
import cmd
import readline
import serial
import serial.tools.list_ports
from OCServo import OCServo

class OCServoCLI(cmd.Cmd):
    """Interactive command-line interface for OCServo"""

    intro = 'OCServo CLI - Type "help" for commands, "quit" to exit\n'
    prompt = 'ocservo> '

    BROADCAST_ID = 0xFE

    def __init__(self, port=None, baudrate=1000000, script=None):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.servo = None
        self.current_id = 1
        self.connected = False
        self.script_mode = script is not None
        self.script_file = script

        # Configure readline for better interaction
        readline.set_completer_delims(' \t\n')

        # Auto-connect if port provided
        if self.port:
            self.do_open(f"{self.port} {self.baudrate}")

    def preloop(self):
        """Run before the command loop starts"""
        if self.script_file:
            self.run_script(self.script_file)
            return True  # Exit after script

    def run_script(self, filename):
        """Execute commands from a script file"""
        try:
            with open(filename, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if line and not line.startswith('#'):
                        print(f"{self.prompt}{line}")
                        self.onecmd(line)
        except FileNotFoundError:
            print(f"Error: Script file '{filename}' not found")
        except Exception as e:
            print(f"Error executing script: {e}")

    def emptyline(self):
        """Do nothing on empty line"""
        pass

    def do_help(self, arg):
        """Show help for commands
        Usage: help [command]"""
        if arg:
            # Show help for specific command
            try:
                func = getattr(self, 'do_' + arg)
                print(func.__doc__)
            except AttributeError:
                print(f"Unknown command: {arg}")
        else:
            print("\nAvailable commands:")
            print("  open <port> [baudrate]  - Open serial port")
            print("  close                   - Close serial port")
            print("  probe                   - List available serial ports")
            print("  scan [start] [end]      - Scan for servos")
            print("  set id <new_id>         - Set servo ID")
            print("  set id <old> <new>      - Change specific servo ID")
            print("  set lock                - Lock servo settings")
            print("  set unlock              - Unlock servo settings")
            print("  get id                  - Get current servo ID")
            print("  get temp                - Read temperature")
            print("  get pos                 - Read position")
            print("  get lock                - Read lock status")
            print("  move <angle> [time_ms]  - Move to angle")
            print("  ping [id]               - Ping servo")
            print("  select <id>             - Select servo ID to work with")
            print("  info                    - Show connection info")
            print("  info status             - Show full servo status")
            print("  script <file>           - Run script file")
            print("  quit                    - Exit CLI")
            print("\nUse 'help <command>' for detailed help")

    def do_open(self, arg):
        """Open serial port connection
        Usage: open <port> [baudrate]
        Example: open /dev/ttyUSB0 1000000"""
        args = arg.split()
        if not args:
            print("Error: Port required. Use 'probe' to list ports")
            return

        port = args[0]
        baudrate = int(args[1]) if len(args) > 1 else self.baudrate

        # Close existing connection
        if self.connected:
            self.do_close('')

        try:
            print(f"Opening {port} at {baudrate} baud...")
            self.servo = OCServo(port, baudrate, self.current_id)
            self.port = port
            self.baudrate = baudrate
            self.connected = True
            print(f"Connected to {port}")
        except serial.SerialException as e:
            print(f"Error: Failed to open port: {e}")
            self.connected = False

    def do_close(self, arg):
        """Close serial port connection
        Usage: close"""
        if self.connected and self.servo:
            try:
                del self.servo
                self.servo = None
                self.connected = False
                print("Port closed")
            except:
                pass
        else:
            print("No connection to close")

    def do_probe(self, arg):
        """List available serial ports
        Usage: probe"""
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found")
            return

        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device:<15} - {port.description}")

    def do_scan(self, arg):
        """Scan for servos on the bus
        Usage: scan [start_id] [end_id]
        Example: scan        # Scan 1-253
                 scan 1 10   # Scan 1-10"""
        if not self._check_connection():
            return

        args = arg.split()
        start_id = int(args[0]) if len(args) > 0 else 1
        end_id = int(args[1]) if len(args) > 1 else 253

        if not (1 <= start_id <= 253 and 1 <= end_id <= 253):
            print("Error: IDs must be between 1 and 253")
            return

        print(f"Scanning IDs {start_id}-{end_id}...")
        found = []

        for servo_id in range(start_id, end_id + 1):
            try:
                self.servo.servo_id = servo_id
                if self.servo.ping():
                    found.append(servo_id)
                    print(f"  Found servo at ID {servo_id}")
            except:
                pass

        if found:
            print(f"Found {len(found)} servo(s): {found}")
        else:
            print("No servos found")

        # Restore original ID
        self.servo.servo_id = self.current_id

    def do_set(self, arg):
        """Set servo parameters
        Usage: set id <new_id>           # Set current servo's ID
               set id <old_id> <new_id>   # Change specific servo's ID
               set broadcast <new_id>     # Set ID using broadcast (DANGER!)
               set lock                   # Lock servo settings
               set unlock                 # Unlock servo settings"""
        if not self._check_connection():
            return

        args = arg.split()
        if len(args) < 1:
            print("Error: Invalid syntax. Use 'help set'")
            return

        match args[0]:
            case 'id':
                if len(args) < 2:
                    print("Error: ID value required")
                    return
                match len(args):
                    case 2:
                        # Set current servo's ID
                        new_id = int(args[1])
                        self._set_servo_id(self.current_id, new_id)
                    case 3:
                        # Change specific servo's ID
                        old_id = int(args[1])
                        new_id = int(args[2])
                        self._set_servo_id(old_id, new_id)
                    case _:
                        print("Error: Invalid syntax for 'set id'")

            case 'broadcast':
                if len(args) != 2:
                    print("Error: Usage: set broadcast <new_id>")
                    return
                new_id = int(args[1])
                self._set_broadcast_id(new_id)

            case 'lock':
                if self.servo.lock(True):
                    print("Servo locked successfully - Configuration parameters are now protected")
                else:
                    print("Error: Failed to lock servo")

            case 'unlock':
                if self.servo.unlock():
                    print("Servo unlocked successfully - All parameters can now be modified")
                else:
                    print("Error: Failed to unlock servo")

            case _:
                print(f"Error: Unknown parameter '{args[0]}'")

    def _set_servo_id(self, old_id, new_id):
        """Internal method to set servo ID"""
        if not (1 <= new_id <= 253):
            print("Error: ID must be between 1 and 253")
            return

        print(f"Setting servo ID {old_id} -> {new_id}")

        # Temporarily set servo ID to old_id
        original_id = self.servo.servo_id
        self.servo.servo_id = old_id

        # Send ID change command
        response = self.servo._send_packet(OCServo.INST_WRITE, [OCServo.ADDR_ID, new_id])

        if response is not None:
            print("Command sent successfully")
            time.sleep(0.5)

            # Verify new ID
            self.servo.servo_id = new_id
            if self.servo.ping():
                print(f"Verified: Servo responds at ID {new_id}")
                self.current_id = new_id
            else:
                print(f"Warning: Servo not responding at ID {new_id}")
                self.servo.servo_id = original_id
        else:
            print("Error: Failed to send command")
            self.servo.servo_id = original_id

    def _set_broadcast_id(self, new_id):
        """Set servo ID using broadcast"""
        if not (1 <= new_id <= 253):
            print("Error: ID must be between 1 and 253")
            return

        print("WARNING: Broadcast mode will change ALL connected servos!")
        if not self.script_mode:
            confirm = input("Continue? (yes/no): ")
            if confirm.lower() != 'yes':
                print("Cancelled")
                return

        # Use broadcast ID
        self.servo.servo_id = self.BROADCAST_ID
        print(f"Broadcasting ID change to {new_id}")

        self.servo._send_packet(OCServo.INST_WRITE, [OCServo.ADDR_ID, new_id])
        print("Broadcast sent (no response expected)")

        time.sleep(0.5)

        # Try to verify
        self.servo.servo_id = new_id
        if self.servo.ping():
            print(f"Verified: Servo responds at ID {new_id}")
            self.current_id = new_id
        else:
            print(f"Could not verify servo at ID {new_id}")
            self.servo.servo_id = self.current_id

    def do_get(self, arg):
        """Get servo parameters
        Usage: get id       - Show current working ID
               get temp     - Read temperature
               get pos      - Read position
               get voltage  - Read input voltage
               get speed    - Read current speed
               get load     - Read load/torque
               get current  - Read current consumption
               get lock     - Read lock status"""
        if not self._check_connection():
            return

        match arg:
            case 'id':
                print(f"Current servo ID: {self.current_id}")

            case 'temp':
                temp = self.servo.read_temperature()
                if temp is not None:
                    print(f"Temperature: {temp}°C")
                else:
                    print("Error: Failed to read temperature")

            case 'pos':
                pos = self.servo.read_position()
                if pos is not None:
                    print(f"Position: {pos:.1f}°")
                else:
                    print("Error: Failed to read position")

            case 'voltage':
                voltage = self.servo.read_voltage()
                if voltage is not None:
                    print(f"Voltage: {voltage:.1f}V")
                else:
                    print("Error: Failed to read voltage")

            case 'speed':
                speed = self.servo.read_speed()
                if speed is not None:
                    print(f"Speed: {speed}")
                else:
                    print("Error: Failed to read speed")

            case 'load':
                load = self.servo.read_load()
                if load:
                    print(f"Load: {load['magnitude']}/1023 ({load['direction']})")
                else:
                    print("Error: Failed to read load")

            case 'current':
                current = self.servo.read_current()
                if current is not None:
                    print(f"Current: {current}mA")
                else:
                    print("Error: Failed to read current")

            case 'lock':
                lock_info = self.servo.check_lock_settings()
                if lock_info:
                    print(f"\nLock Status:")
                    print(f"  {lock_info['status_message']}")
                    print(f"  Lock value: {lock_info['lock_value']} ({'LOCKED' if lock_info['locked'] else 'UNLOCKED'})")
                    print(f"  Writable addresses: {len(lock_info['writable_addresses'])}")
                    print(f"  Protected addresses: {len(lock_info['protected_addresses'])}")
                else:
                    print("Error: Failed to read lock status")

            case _:
                print(f"Error: Unknown parameter '{arg}'")

    def do_move(self, arg):
        """Move servo to angle
        Usage: move <angle> [time_ms]
        Example: move 90        # Move to 90° at max speed
                 move 180 2000  # Move to 180° in 2 seconds"""
        if not self._check_connection():
            return

        args = arg.split()
        if not args:
            print("Error: Angle required")
            return

        try:
            angle = float(args[0])
            time_ms = int(args[1]) if len(args) > 1 else 0

            if not (0 <= angle <= 360):
                print("Error: Angle must be between 0 and 360")
                return

            print(f"Moving to {angle}°" + (f" in {time_ms}ms" if time_ms else ""))
            if self.servo.set_angle(angle, time_ms):
                print("Command sent")
            else:
                print("Error: Failed to send command")
        except ValueError:
            print("Error: Invalid angle or time")

    def do_ping(self, arg):
        """Ping servo
        Usage: ping [id]
        Example: ping      # Ping current servo
                 ping 5    # Ping servo ID 5"""
        if not self._check_connection():
            return

        if arg:
            try:
                servo_id = int(arg)
                original_id = self.servo.servo_id
                self.servo.servo_id = servo_id
                result = self.servo.ping()
                self.servo.servo_id = original_id

                if result:
                    print(f"Servo ID {servo_id}: OK")
                else:
                    print(f"Servo ID {servo_id}: No response")
            except ValueError:
                print("Error: Invalid ID")
        else:
            if self.servo.ping():
                print(f"Servo ID {self.current_id}: OK")
            else:
                print(f"Servo ID {self.current_id}: No response")

    def do_select(self, arg):
        """Select servo ID to work with
        Usage: select <id>
        Example: select 5"""
        if not arg:
            print(f"Current servo ID: {self.current_id}")
            return

        try:
            servo_id = int(arg)
            if not (1 <= servo_id <= 254):
                print("Error: ID must be between 1 and 254")
                return

            self.current_id = servo_id
            if self.servo:
                self.servo.servo_id = servo_id
            print(f"Selected servo ID {servo_id}")
        except ValueError:
            print("Error: Invalid ID")

    def do_info(self, arg):
        """Show servo information
        Usage: info           - Show connection info
               info status    - Show full servo status"""

        match arg:
            case 'status':
                if not self._check_connection():
                    return

                print(f"\n{'='*50}")
                print(f"  SERVO STATUS - ID {self.current_id}")
                print(f"{'='*50}")

                # Read comprehensive status
                status = self.servo.read_servo_status()

                if status:
                    # Connection status
                    print(f"\nConnection:")
                    print(f"  Port:        {self.port}")
                    print(f"  Baudrate:    {self.baudrate}")
                    print(f"  Responding:  {'Yes' if status.get('responding') else 'No'}")

                    # Position and movement
                    print(f"\nPosition:")
                    pos = status.get('position')
                    print(f"  Angle:       {pos:.1f}°" if pos is not None else "  Angle:       N/A")
                    speed = status.get('speed')
                    print(f"  Speed:       {speed}" if speed is not None else "  Speed:       N/A")

                    # Load/Torque
                    load = status.get('load')
                    if load:
                        print(f"\nLoad:")
                        print(f"  Magnitude:   {load['magnitude']}/1023")
                        print(f"  Direction:   {load['direction']}")
                    else:
                        print(f"\nLoad:          N/A")

                    # Electrical
                    print(f"\nElectrical:")
                    voltage = status.get('voltage')
                    print(f"  Voltage:     {voltage:.1f}V" if voltage is not None else "  Voltage:     N/A")
                    current = status.get('current')
                    print(f"  Current:     {current}mA" if current is not None else "  Current:     N/A")

                    # Temperature
                    temp = status.get('temperature')
                    print(f"\nTemperature:   {temp}°C" if temp is not None else "\nTemperature:   N/A")

                    # Lock Status
                    lock_info = self.servo.check_lock_settings()
                    if lock_info:
                        print(f"\nLock Status:")
                        print(f"  Status:      {lock_info['status_message']}")
                        print(f"  Lock value:  {lock_info['lock_value']} ({'LOCKED' if lock_info['locked'] else 'UNLOCKED'})")

                    print(f"\n{'='*50}")
                else:
                    print("Error: Failed to read servo status")

            case _:
                # Basic connection info
                print("\nConnection info:")
                print(f"  Port:        {self.port if self.port else 'Not set'}")
                print(f"  Baudrate:    {self.baudrate}")
                print(f"  Status:      {'Connected' if self.connected else 'Disconnected'}")
                print(f"  Servo ID:    {self.current_id}")
                print("\nUse 'info status' for full servo status")

    def do_script(self, arg):
        """Run commands from script file
        Usage: script <filename>
        Example: script setup.txt"""
        if not arg:
            print("Error: Filename required")
            return

        self.run_script(arg)

    def do_quit(self, arg):
        """Exit the CLI
        Usage: quit"""
        if self.connected:
            self.do_close('')
        print("Goodbye!")
        return True

    def do_exit(self, arg):
        """Exit the CLI (alias for quit)"""
        return self.do_quit(arg)

    def do_EOF(self, arg):
        """Handle Ctrl-D"""
        print()  # New line after ^D
        return self.do_quit(arg)

    def _check_connection(self):
        """Check if connected to serial port"""
        if not self.connected or not self.servo:
            print("Error: Not connected. Use 'open <port>' to connect")
            return False
        return True

    def default(self, line):
        """Handle unknown commands"""
        print(f"Unknown command: {line}. Type 'help' for commands")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='OCServo CLI - Command-line interface for ROBS-802 servos',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ocservo-cli                          # Interactive mode
  ocservo-cli -d /dev/ttyUSB0          # Connect to specific port
  ocservo-cli -d /dev/ttyUSB0 -s script.txt  # Run script
  ocservo-cli -d COM3 -b 115200        # Custom baudrate

Script file format:
  # Comments start with #
  open /dev/ttyUSB0
  scan 1 10
  set id 1 5
  move 90 1000
  quit
        """
    )

    parser.add_argument('-d', '--device',
                        help='Serial port device (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baudrate',
                        type=int,
                        default=1000000,
                        help='Serial baudrate (default: 1000000)')
    parser.add_argument('-s', '--script',
                        help='Script file to execute')
    parser.add_argument('-v', '--version',
                        action='version',
                        version='OCServo CLI 1.0')

    args = parser.parse_args()

    # Create CLI instance
    cli = OCServoCLI(
        port=args.device,
        baudrate=args.baudrate,
        script=args.script
    )

    try:
        # Run script mode or interactive mode
        if args.script:
            cli.preloop()
        else:
            cli.cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupted")
        if cli.connected:
            cli.do_close('')
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
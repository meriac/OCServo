#!/usr/bin/env python3

# test-motion.py - Test script with position logging and matplotlib visualization
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

import matplotlib.pyplot as plt
from OCServo import OCServo
from ServoPositionLogger import ServoPositionLogger

def main():
    # Configuration
    SERIAL_PORT = '/dev/ttyUSB0'
    SERVO_ID = 0x01
    INITIAL_ANGLE = 180  # Starting position (center)

    try:
        # Initialize servo
        print("Connecting to servo...")
        servo = OCServo(port=SERIAL_PORT, baudrate=1000000, servo_id=SERVO_ID)

        # Test connection
        if not servo.ping():
            print("Failed to ping servo!")
            return
        print("Servo connected successfully!")

        # Read initial temperature
        temp = servo.read_temperature()
        if temp:
            print(f"Servo temperature: {temp}°C")

        # Move to initial position
        print(f"Moving to initial position ({INITIAL_ANGLE}°)...")
        servo.set_angle(INITIAL_ANGLE, time_ms=1000)

        # Create position logger
        logger = ServoPositionLogger(servo)
        logger.start_session()

        # Move forward 90 degrees (to 270°)
        print("Moving forward 90 degrees...")
        target_angle = INITIAL_ANGLE + 90
        servo.set_angle(target_angle, time_ms=1000)

        # Sleep and log during the movement
        print("Logging forward movement...")
        logger.wait(1.5)

        # Move backward 90 degrees (back to 180°)
        print("Moving backward 90 degrees...")
        servo.set_angle(INITIAL_ANGLE, time_ms=1000)

        # Sleep and log during the movement
        print("Logging backward movement...")
        logger.wait(1.5)

        # Get logged data
        timestamps, positions = logger.get_data()
        count = logger.count()
        duration = logger.duration()
        print(f"\nLogged {count} position samples over {duration:.1f}s ({round(count/duration)} per second)")

        # Plot results
        if positions:
            plt.figure(figsize=(12, 6))
            plt.plot(timestamps, positions, 'b-', linewidth=1, marker='.', markersize=1)
            plt.xlabel('Time (milliseconds)')
            plt.ylabel('Position (degrees)')
            plt.title('Servo Position Over Time')
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
        else:
            print("No position data logged")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
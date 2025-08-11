#!/usr/bin/env python3

# test-motor-mode.py - Test script demonstrating motor mode and servo/motor switching
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

"""
Test script for OCServo motor mode functionality
Demonstrates switching between servo and motor modes
"""

import matplotlib.pyplot as plt
from OCServo import OCServo
from ServoPositionLogger import ServoPositionLogger

def main():
    # Initialize servo on /dev/ttyUSB0 (adjust port as needed)
    servo = OCServo('/dev/ttyUSB0', baudrate=1000000, servo_id=1)

    print("OCServo Motor Mode Test")
    print("=" * 40)

    # Test 1: Verify connection
    print("\n1. Testing connection...")
    if servo.ping():
        print("✓ Servo connected")
    else:
        print("✗ Failed to connect to servo")
        return

    # Create position logger
    logger = ServoPositionLogger(servo)
    logger.start_session()

    # Test 2: Motor mode - continuous rotation
    print("\n2. Testing motor mode (continuous rotation)...")

    print("Rotating CW at 50% torque for 10 seconds")
    servo.motor_speed(50, cw=True)
    logger.wait(10)

    print("Rotating CCW at 30% torque for 10 seconds")
    servo.motor_speed(30, cw=False)
    logger.wait(10)

    print("Rotating CW at 100% torque for 10 seconds")
    servo.motor_speed(100, cw=True)
    logger.wait(10)

    print("Stopping motor (0% torque)")
    servo.motor_speed(0, cw=True)
    logger.wait(3)

    print("\n" + "=" * 40)
    print("Motor mode test completed!")

    # Final position
    print("\nReturning to center position...")
    servo.set_angle(180, time_ms=1000)
    logger.wait(2)

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

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
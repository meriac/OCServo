#!/usr/bin/env python3

# test-acceleration.py - PID tuning script for ROBS-802 servo acceleration control
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

import time
import matplotlib.pyplot as plt
from OCServo import OCServo
from ServoPositionLogger import ServoPositionLogger

def test_motion_profile(servo, p_gain, test_name):
    """
    Test motion with specific P gain and record position
    """
    print(f"\n{test_name}")
    print("=" * len(test_name))


    # Create position logger
    logger = ServoPositionLogger(servo)

    # Move to start position
    servo.set_angle(90, time_ms=1000)
    time.sleep(1.5)

    # Set P gain to tested value
    servo.set_gain(p_gain)
    print(f"P gain set to {p_gain}")

    # Start logging
    logger.start_session()

    # Command step change
    servo.set_angle(270, time_ms=0)  # Immediate command

    # Log position for 4 seconds
    logger.wait(4.0)

    # Get logged data
    timestamps, positions = logger.get_data()

    # Restore defaults if needed
    if p_gain!=15:
        print("Restoring default P gain...")
        servo.set_gain(15)

    return timestamps, positions

def acceleration_profile_comparison():
    """Compare different PID settings and their effect on acceleration"""

    SERIAL_PORT = '/dev/ttyUSB0'  # Update this!
    servo = OCServo(port=SERIAL_PORT, servo_id=1)

    print("PID Acceleration Profile Comparison")
    print("==================================")

    if not servo.ping():
        print("Servo not responding!")
        return

    # Test configurations - P values from 0 to 15
    test_configs = [
        # (P_gain, name)
        (1, "P=1"),
        (3, "P=3"),
        (6, "P=6"),
        (9, "P=9"),
        (12, "P=12"),
        (15, "P=15"),
    ]

    results = []

    for p_gain, name in test_configs:
        timestamps, positions = test_motion_profile(servo, p_gain, name)
        results.append((name, timestamps, positions))

    # Plot results
    plt.figure(figsize=(12, 8))

    for name, timestamps, positions in results:
        plt.plot(timestamps, positions, label=name, linewidth=2)

    plt.axhline(y=270, color='r', linestyle='--', alpha=0.5, label='Target')
    plt.xlabel('Time (milliseconds)')
    plt.ylabel('Position (degrees)')
    plt.title('PID Tuning Effects on Acceleration/Deceleration')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Run acceleration comparison
    acceleration_profile_comparison()
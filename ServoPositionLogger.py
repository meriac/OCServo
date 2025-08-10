# OCServoPositionLogger.py - Python library for reading servo positions while waiting
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

class ServoPositionLogger:
    """Class to log servo position during sleep periods"""

    def __init__(self, servo):
        self.servo = servo
        self.positions = []
        self.timestamps = []
        self.overall_start_time = None

    def start_session(self):
        """Start a new logging session"""
        self.positions = []
        self.timestamps = []
        self.overall_start_time = time.time()

    def count(self):
        return len(self.positions)

    def duration(self):
        return (time.time() - self.overall_start_time) if self.overall_start_time else None

    def wait(self, duration):
        """
        Sleep for the specified duration while continuously logging positions

        Args:
            duration: Sleep duration in seconds
        """
        start_time = time.time()
        end_time = start_time + duration

        if self.overall_start_time is None:
            self.overall_start_time = start_time

        # Continuously read positions until duration has passed
        while time.time() < end_time:
            try:
                angle = self.servo.read_position()
                if angle is not None:
                    self.timestamps.append((time.time() - self.overall_start_time) * 1000)
                    self.positions.append(angle)

            except Exception as e:
                print(f"\nError reading position: {e}")

    def get_data(self):
        """Return the logged position and timestamp data"""
        return self.timestamps, self.positions
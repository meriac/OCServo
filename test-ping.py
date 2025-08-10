#!/usr/bin/env python3

# test-ping.py - High-speed connectivity and latency testing for OCServo
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
import argparse
import signal
import sys
from OCServo import OCServo

class PingTester:
    """High-speed ping tester for OCServo"""

    def __init__(self, port, baudrate, servo_id, delay_ms=0):
        """
        Initialize ping tester

        Args:
            port: Serial port
            baudrate: Communication baudrate
            servo_id: Servo ID to ping
            delay_ms: Optional delay between pings in milliseconds
        """
        self.servo = OCServo(port=port, baudrate=baudrate, servo_id=servo_id)
        self.delay_ms = delay_ms
        self.running = False

        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.start_time = None
        self.last_report_time = None
        self.last_report_sent = 0
        self.last_report_received = 0

        # Response time tracking
        self.min_time = float('inf')
        self.max_time = 0
        self.total_time = 0

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\n\nInterrupted by user")
        self.running = False
        self.print_final_stats()
        sys.exit(0)

    def print_stats(self):
        """Print current statistics"""
        current_time = time.time()
        elapsed = current_time - self.start_time

        # Calculate rates
        if elapsed > 0:
            total_rate = self.packets_received / elapsed
            loss_percent = ((self.packets_sent - self.packets_received) / self.packets_sent * 100) if self.packets_sent > 0 else 0
        else:
            total_rate = 0
            loss_percent = 0

        # Calculate interval stats (last second)
        if self.last_report_time:
            interval = current_time - self.last_report_time
            interval_sent = self.packets_sent - self.last_report_sent
            interval_received = self.packets_received - self.last_report_received

            if interval > 0:
                interval_rate = interval_received / interval
                interval_loss = ((interval_sent - interval_received) / interval_sent * 100) if interval_sent > 0 else 0
            else:
                interval_rate = 0
                interval_loss = 0
        else:
            interval_rate = 0
            interval_loss = 0

        # Calculate average response time
        avg_time = (self.total_time / self.packets_received * 1000) if self.packets_received > 0 else 0

        # Print stats line
        print(f"\rPkts: Sent={self.packets_sent:6d} Recv={self.packets_received:6d} | "
              f"Rate: {interval_rate:6.1f} pkt/s | "
              f"Loss: {loss_percent:5.1f}% | "
              f"RTT(ms): min={self.min_time*1000:.2f} avg={avg_time:.2f} max={self.max_time*1000:.2f}",
              end='', flush=True)

        # Update last report values
        self.last_report_time = current_time
        self.last_report_sent = self.packets_sent
        self.last_report_received = self.packets_received

    def print_final_stats(self):
        """Print final statistics summary"""
        if not self.start_time:
            return

        elapsed = time.time() - self.start_time

        print("\n" + "="*60)
        print("PING TEST SUMMARY")
        print("="*60)
        print(f"Test duration: {elapsed:.2f} seconds")
        print(f"Packets sent: {self.packets_sent}")
        print(f"Packets received: {self.packets_received}")
        print(f"Packets lost: {self.packets_sent - self.packets_received}")

        if self.packets_sent > 0:
            loss_percent = ((self.packets_sent - self.packets_received) / self.packets_sent * 100)
            print(f"Packet loss: {loss_percent:.1f}%")

        if elapsed > 0:
            sent_rate = self.packets_sent / elapsed
            recv_rate = self.packets_received / elapsed
            print(f"Send rate: {sent_rate:.1f} packets/second")
            print(f"Receive rate: {recv_rate:.1f} packets/second")

        if self.packets_received > 0:
            avg_time = self.total_time / self.packets_received * 1000
            print(f"\nResponse times (milliseconds):")
            print(f"  Minimum: {self.min_time*1000:.3f} ms")
            print(f"  Average: {avg_time:.3f} ms")
            print(f"  Maximum: {self.max_time*1000:.3f} ms")

        print("="*60)

    def run(self):
        """Run the ping test"""
        # Set up signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        print(f"Starting ping test to servo ID {self.servo.servo_id}")
        print(f"Delay between pings: {self.delay_ms} ms")
        print("Press Ctrl+C to stop\n")

        self.running = True
        self.start_time = time.time()
        self.last_report_time = self.start_time

        # Main ping loop
        while self.running:
            # Send ping and measure response time
            ping_start = time.time()
            self.packets_sent += 1

            try:
                success = self.servo.ping()
                ping_time = time.time() - ping_start

                if success:
                    self.packets_received += 1
                    self.total_time += ping_time
                    self.min_time = min(self.min_time, ping_time)
                    self.max_time = max(self.max_time, ping_time)

            except Exception as e:
                # Handle any errors silently to maintain high speed
                pass

            # Print stats every second
            if time.time() - self.last_report_time >= 1.0:
                self.print_stats()

            # Optional delay between pings
            if self.delay_ms > 0:
                time.sleep(self.delay_ms / 1000.0)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='High-speed ping test for OCServo')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=1000000,
                       help='Baudrate (default: 1000000)')
    parser.add_argument('--id', '-i', type=int, default=1,
                       help='Servo ID (default: 1)')
    parser.add_argument('--delay', '-d', type=int, default=0,
                       help='Delay between pings in milliseconds (default: 0)')

    args = parser.parse_args()

    # Validate delay
    if args.delay < 0:
        print("Error: Delay must be non-negative")
        sys.exit(1)

    # Create and run ping tester
    try:
        tester = PingTester(
            port=args.port,
            baudrate=args.baudrate,
            servo_id=args.id,
            delay_ms=args.delay
        )
        tester.run()

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
"""
encoder_feedback.py

Reads the RobStride motor's built-in encoder feedback (position, velocity,
torque, temperature) over CAN - no control, zero torque commanded the
whole time. Useful for:
  - confirming the adapter/wiring/CAN id are correct before trusting any
    control commands (same purpose as the 'swing' stage in
    ../src/inverted_pendulum.py)
  - checking encoder sign convention by nudging the rod/shaft by hand
  - logging feedback to a CSV for offline inspection

Edit PORT / MOTOR_ID below for your setup.
  Windows: 'COM6', 'COM8', ...
  Linux:   '/dev/ttyUSB0'
"""

import argparse
import csv
import math
import sys
import time

from at_can_bus import (
    ATCanBus,
    control,
    decode_feedback,
    enable,
    set_run_mode_mit,
    stop,
)

PORT = 'COM8'      # <-- CHANGE THIS
MOTOR_ID = 1       # <-- CHANGE THIS (check via RobStride's Motor Studio tool)

POLL_INTERVAL_S = 0.05  # ~20Hz, matches the swing-test cadence elsewhere in this project


def read_feedback_loop(bus, motor_id, duration=None, log_path=None):
    """Print (and optionally log) encoder feedback until Ctrl+C or `duration`
    seconds elapse. Sends zero torque/kp/kd each cycle - this is read-only."""
    writer = None
    log_file = None
    if log_path:
        log_file = open(log_path, 'w', newline='')
        writer = csv.writer(log_file)
        writer.writerow(['t_s', 'pos_rad', 'pos_deg', 'vel_rad_s', 'torque_Nm', 'temp_C'])

    t0 = time.time()
    print("Reading encoder feedback (zero torque). Ctrl+C to stop.\n")
    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if duration is not None and elapsed >= duration:
                break

            resp = control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"t={elapsed:6.2f}s  pos={pos:+.3f}rad ({math.degrees(pos):+7.1f}deg)  "
                      f"vel={vel:+.3f}rad/s  torque={tor:+.3f}Nm  temp={temp:5.1f}C")
                if writer:
                    writer.writerow([f"{elapsed:.3f}", f"{pos:.5f}", f"{math.degrees(pos):.2f}",
                                      f"{vel:.5f}", f"{tor:.5f}", f"{temp:.1f}"])
            else:
                print(f"t={elapsed:6.2f}s  (no feedback received)")

            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if log_file:
            log_file.close()
            print(f"Logged feedback to {log_path}")


def main():
    parser = argparse.ArgumentParser(description="Read RobStride motor encoder feedback.")
    parser.add_argument('--port', default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument('--motor-id', type=int, default=MOTOR_ID, help=f"CAN motor id (default: {MOTOR_ID})")
    parser.add_argument('--duration', type=float, default=None,
                         help="Stop after N seconds instead of running until Ctrl+C")
    parser.add_argument('--log', default=None, help="Optional CSV file path to log feedback to")
    args = parser.parse_args()

    print(f"Opening {args.port}, motor id {args.motor_id}...")
    bus = ATCanBus(args.port)
    try:
        print("Waiting for the adapter/motor to settle (power-cycle the motor now if it "
              "was already on)...")
        time.sleep(3)

        set_run_mode_mit(bus, args.motor_id)
        time.sleep(0.1)
        enable(bus, args.motor_id)
        time.sleep(0.1)
        read_feedback_loop(bus, args.motor_id, duration=args.duration, log_path=args.log)
    finally:
        stop(bus, args.motor_id)
        bus.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()

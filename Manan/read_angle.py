"""
read_angle.py

Script to read the angle of the motor shaft with respect to the reference position.
Provides a reusable `get_position()` function and an interactive command-line mode
where you can rotate the shaft by hand and view the output on the terminal.
"""

import argparse
import math
import time
import json
import os

from at_can_bus import (
    ATCanBus,
    control,
    decode_feedback,
    enable,
    set_run_mode_mit,
    stop,
)

PORT = 'COM8'
MOTOR_ID = 1
POLL_INTERVAL_S = 0.1

def get_position(bus, motor_id):
    """
    Sends a zero-torque control frame and decodes the feedback to return
    the current shaft position in radians with respect to the zero reference.
    
    Returns:
        float: Position in radians, or None if no feedback was received.
    """
    resp = control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
    fb = decode_feedback(resp)
    if fb:
        pos, _, _, _ = fb
        return pos
    return None

def monitor_angle(bus, motor_id, poll_interval=POLL_INTERVAL_S):
    """
    Continuously queries the motor and prints the shaft angle.
    """
    print("Reading shaft angle (0 torque). Rotate the shaft by hand.")
    print("Press Ctrl+C to stop.\n")
    try:
        while True:
            pos = get_position(bus, motor_id)
            if pos is not None:
                deg = math.degrees(pos)
                print(f"Angle: {pos:+.4f} rad ({deg:+7.1f} deg)")
            else:
                print("No feedback received.")
            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nStopping...")

def get_default_motor(log_file="zero_calibration_log.json"):
    default_port = PORT
    default_id = MOTOR_ID
    if os.path.exists(log_file):
        try:
            with open(log_file, 'r') as f:
                logs = json.load(f)
                if isinstance(logs, dict) and logs:
                    most_recent = max(logs.values(), key=lambda x: x.get("timestamp", ""))
                    if "port" in most_recent and "motor_id" in most_recent:
                        default_port = most_recent["port"]
                        default_id = most_recent["motor_id"]
        except (json.JSONDecodeError, ValueError):
            pass
    return default_port, default_id

def main():
    default_port, default_id = get_default_motor()

    parser = argparse.ArgumentParser(description="Read the motor shaft angle interactively.")
    parser.add_argument('--port', default=default_port, help=f"Serial port (default: {default_port})")
    parser.add_argument('--motor-id', type=int, default=default_id, help=f"CAN motor id (default: {default_id})")
    args = parser.parse_args()

    print(f"Opening {args.port}, motor id {args.motor_id}...")
    bus = ATCanBus(args.port)
    
    try:
        print("Waiting for adapter to settle...")
        time.sleep(3)
        
        # Must be in MIT mode
        set_run_mode_mit(bus, args.motor_id)
        time.sleep(0.1)
        
        # Must be enabled to reply to control frames
        enable(bus, args.motor_id)
        time.sleep(0.1)
        
        monitor_angle(bus, args.motor_id)
        
    finally:
        stop(bus, args.motor_id)
        bus.shutdown()
        print("Done.")

if __name__ == "__main__":
    main()
